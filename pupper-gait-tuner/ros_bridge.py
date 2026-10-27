"""ROS2 bridge: stream a gait cache to the Pupper's forward_command_controller.

This is a drop-in replacement for lab_4.py's publishing role. It publishes 12
joint position targets to /forward_command_controller/commands at 200 Hz, with
the play head advanced by the live frequency parameter and a smooth ramp when
enabling the gait or returning to a stand pose.

It does NOT launch the ros2_control stack — bring that up the same way you do for
lab 4 (so /forward_command_controller exists), then run this instead of lab_4.py.
"""

import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import gait
from gait import GaitParams, generate_cache

JOINT_ORDER = [
    'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3',
    'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3',
    'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3',
    'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3',
]

PUBLISH_HZ = 200.0
RAMP_TIME = 1.2          # seconds to blend into a new target pose/gait


def stand_pose(body_height):
    """12-vector neutral standing pose: feet planted at nominal stance, no stride."""
    j, _ = generate_cache(GaitParams(body_height=body_height, step_length=0.0,
                                     step_height=0.0, n_frames=10))
    return j[0].copy()


class RobotBridge(Node):
    def __init__(self):
        super().__init__('gait_tuner_bridge')
        self.pub = self.create_publisher(Float64MultiArray, '/forward_command_controller/commands', 10)
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10)

        self._lock = threading.Lock()
        self.cache = None            # (N,12)
        self.frequency = 2.0
        self.phase = 0.0             # play head in [0,1)
        self.enabled = False         # gait running?
        self.measured = None         # latest measured joint vector (for ramp start)
        self.body_height = 0.14

        # arbitration: only one tuner ("owner") may drive the motors at a time
        self.owner = None
        self._listeners = []         # callbacks(owner) fired when ownership changes

        # ramp state: blend from `ramp_from` to live target over RAMP_TIME
        self.ramp_from = stand_pose(self.body_height)
        self.ramp_start_t = -1e9
        self.last_cmd = self.ramp_from.copy()
        self.hold_pose = self.ramp_from.copy()   # target when gait disabled

        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info("gait_tuner_bridge up — publishing to /forward_command_controller/commands")

    # --------------------------------------------------------- subscriptions
    def _on_joint_states(self, msg):
        try:
            idx = {n: i for i, n in enumerate(msg.name)}
            self.measured = np.array([msg.position[idx[j]] for j in JOINT_ORDER])
        except (KeyError, IndexError):
            pass

    # ------------------------------------------------------------- public API
    def register_listener(self, cb):
        """cb(owner) is called whenever the active owner changes."""
        self._listeners.append(cb)

    def _notify(self, owner):
        for cb in list(self._listeners):
            cb(owner)

    def update_cache(self, joints, frequency, owner=None):
        """Push a new gait. Ignored unless `owner` currently holds control
        (owner=None bypasses the check, for single-user / standalone use)."""
        with self._lock:
            if owner is not None and owner != self.owner:
                return
            self.cache = np.asarray(joints, float)
            self.frequency = float(frequency)

    def set_body_height(self, h):
        self.body_height = float(h)

    def _begin_ramp(self):
        self.ramp_from = (self.measured.copy() if self.measured is not None
                          else self.last_cmd.copy())
        self.ramp_start_t = time.time()

    def external_publishers(self):
        """How many OTHER nodes are publishing motor commands (e.g. the normal
        robot stack). >0 means something outside this app owns the robot."""
        n = self.count_publishers('/forward_command_controller/commands')
        return max(0, n - 1)         # subtract our own publisher

    def is_free(self, owner):
        """True if `owner` may deploy: nobody else owns it and no external driver."""
        return self.owner in (None, owner) and self.external_publishers() == 0

    def try_acquire(self, owner):
        """Cooperative lock: take control only if the robot is free.
        Returns True on success, False if occupied (does NOT bump anyone)."""
        with self._lock:
            if self.owner not in (None, owner):
                return False
            if self.external_publishers() > 0:
                return False
            self.owner = owner
            self.phase = 0.0
            self._begin_ramp()
            self.enabled = True
        self._notify(owner)
        return True

    def request_control(self, owner):
        """Take exclusive control of the robot, bumping any previous owner.
        Returns the previous owner (or None)."""
        with self._lock:
            prev = self.owner
            self.owner = owner
            self.phase = 0.0
            self._begin_ramp()
            self.enabled = True
        self._notify(owner)          # tells the previous owner it lost control
        return prev

    def release(self, owner):
        """Release control if `owner` holds it; the robot returns to a stand."""
        changed = False
        with self._lock:
            if self.owner == owner:
                self.owner = None
                self.hold_pose = stand_pose(self.body_height)
                self._begin_ramp()
                self.enabled = False
                changed = True
        if changed:
            self._notify(None)

    # single-user convenience (no arbitration)
    def enable(self):
        self.request_control("default")

    def disable(self):
        self.release(self.owner if self.owner is not None else "default")

    def stand(self):
        """Stop whoever is driving and hold a stand pose."""
        cur = self.owner
        with self._lock:
            self.owner = None
            self.hold_pose = stand_pose(self.body_height)
            self._begin_ramp()
            self.enabled = False
        if cur is not None:
            self._notify(None)

    # ----------------------------------------------------------------- loop
    def _sample_gait(self, dt):
        """Advance the play head and return the interpolated 12-vector."""
        cache = self.cache
        n = cache.shape[0]
        self.phase = (self.phase + self.frequency * dt) % 1.0
        f = self.phase * n
        i0 = int(f) % n
        i1 = (i0 + 1) % n
        a = f - int(f)
        return cache[i0] * (1 - a) + cache[i1] * a

    def _tick(self):
        now = time.time()
        with self._lock:
            dt = 1.0 / PUBLISH_HZ
            if self.enabled and self.cache is not None:
                target = self._sample_gait(dt)
            else:
                target = self.hold_pose
            # smooth ramp blend
            alpha = np.clip((now - self.ramp_start_t) / RAMP_TIME, 0.0, 1.0)
            cmd = (1 - alpha) * self.ramp_from + alpha * target
            self.last_cmd = cmd

        msg = Float64MultiArray()
        msg.data = cmd.tolist()
        self.pub.publish(msg)


def spin_bridge_in_thread():
    """Init rclpy, build the bridge, spin it on a daemon thread, return the bridge."""
    if not rclpy.ok():
        rclpy.init()
    bridge = RobotBridge()
    threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True).start()
    return bridge


if __name__ == "__main__":
    # standalone test: publish a default trot, no Viser
    bridge = spin_bridge_in_thread()
    j, _ = generate_cache(GaitParams())
    bridge.update_cache(j, 2.0)
    bridge.enable()
    print("publishing default trot to /forward_command_controller/commands (Ctrl-C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        bridge.disable()
        time.sleep(0.5)
