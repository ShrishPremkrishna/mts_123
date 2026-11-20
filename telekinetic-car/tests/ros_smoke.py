"""Exercise BCI ROS transport with the real mux and a fake Pupper.

Run in the network-disabled ROS Jazzy test container. No native controller or
hardware interface is started. Only the production runtime identity probe is
replaced. The enable state, command transport, and timeouts use real ROS.
"""

from argparse import ArgumentParser
from contextlib import contextmanager
import json
import os
from pathlib import Path
import subprocess
from threading import Thread
import time
from unittest import TestCase
from unittest.mock import patch

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, String


ENABLE_BUTTON = 4
NATIVE_NAME = "neural_controller_default"
MOTION = (0.15, 0.0, 0.0, 0.0, 0.0, 0.35)
ZERO = (0.0,) * 6


class FakePupper:
    def __init__(self, context, policy):
        self.held = True
        self.joy_online = True
        self.latched = False
        self.sequence = 0
        self.inventory_reads = 0
        self.velocity = []
        self.nodes = [Node(name, context=context) for name in (
            "joy_node", "estop_controller", NATIVE_NAME, "controller_manager"
        )]
        joy, gateway, native, manager = self.nodes
        self._joy = joy.create_publisher(Joy, "/joy", 10)
        self._gateway = gateway.create_publisher(
            String, "/estop_controller/stop_status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._observation = native.create_publisher(
            Float32MultiArray, f"/{NATIVE_NAME}/observation", 10
        )
        native.create_subscription(Twist, "/cmd_vel", self._observe_velocity, 10)
        manager.create_service(ListControllers, "/controller_manager/list_controllers", self._inventory)
        native.declare_parameters("", [
            ("model_path", str(policy)), ("max_vx_forward", 0.49), ("max_vx_backward", 0.3),
            ("speed_cap", 0.49), ("game_buttons", False),
            ("cmd_vel_expiry_supported", True), ("cmd_vel_expiry_timeout_s", 0.25),
        ])
        joy.create_timer(0.025, self._publish_feedback)

    def _publish_feedback(self):
        if self.joy_online:
            joy = Joy()
            joy.header.stamp = self.nodes[0].get_clock().now().to_msg()
            joy.buttons = [0] * 13
            joy.buttons[ENABLE_BUTTON] = int(self.held)
            self._joy.publish(joy)
        self.sequence += 1
        self._gateway.publish(String(data=json.dumps({
            "protocol": "pupper_stop_v1", "instance_id": "a" * 32,
            "epoch": int(self.latched), "sequence": self.sequence,
            "latched": self.latched, "fault": False, "stop_required": False,
            "in_flight": False, "unknown_remote_submission": False, "shutting_down": False,
        })))
        observation = [0.0] * 960
        # Mirror just the fake walker's current command slots in the stock layout.
        if self.velocity:
            previous_command = self.velocity[-1][1]
            observation[6:9] = previous_command[0], previous_command[1], previous_command[5]
        self._observation.publish(Float32MultiArray(data=observation))

    def _inventory(self, _request, response):
        self.inventory_reads += 1
        response.controller = [ControllerState(
            name=NATIVE_NAME, state="active", type="neural_controller/NeuralController",
            claimed_interfaces=["leg_front_l_1/position"],
        )]
        return response

    def _observe_velocity(self, msg):
        self.velocity.append((time.monotonic(), (
            msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z
        )))

    def saw_velocity(self, value, since):
        return any(stamp >= since and command == value for stamp, command in self.velocity)

    def moved_since(self, since):
        return any(stamp >= since and command != ZERO for stamp, command in self.velocity)


@contextmanager
def fake_pupper(policy):
    context = Context()
    rclpy.init(context=context)
    fake = FakePupper(context, policy)
    executor = SingleThreadedExecutor(context=context)
    for node in fake.nodes:
        executor.add_node(node)
    worker = Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        yield fake
    finally:
        executor.shutdown(timeout_sec=2)
        worker.join(timeout=2)
        for node in fake.nodes:
            node.destroy_node()
        rclpy.try_shutdown(context=context)


def exercise(fake, sink):
    def drive(seconds, *, condition=None, expected=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            start = time.monotonic()
            enabled = sink.poll()
            if expected is not None:
                assert enabled is expected, f"Unexpected enable={enabled}: {sink.waiting_reason}"
            forward, yaw = (MOTION[0], MOTION[5]) if enabled else (0.0, 0.0)
            sink.publish(forward, yaw)
            if condition is not None and condition():
                return
            time.sleep(max(0.0, start + 0.05 - time.monotonic()))
        if condition is not None:
            raise AssertionError(f"Timed out waiting for command: {sink.waiting_reason}")

    def hold_and_move():
        begins = time.monotonic()
        fake.held = True
        drive(2.0, condition=lambda: fake.saw_velocity(MOTION, begins))

    def assert_zero_after(begins, condition_name):
        drive(0.5, condition=lambda: fake.saw_velocity(ZERO, begins))
        quiet_from = time.monotonic()
        drive(0.25, expected=False)
        assert not fake.moved_since(quiet_from), f"Motion resumed after {condition_name}"

    drive(2.5, condition=lambda: sink.waiting_reason == "release then hold the enable button",
          expected=False)
    drive(0.25, expected=False)
    assert fake.inventory_reads > 0, "Held startup was tested before service discovery"
    assert not fake.moved_since(0), "Holding enable on startup produced motion"
    print("PASS startup-held enable refused", flush=True)

    fake.held = False
    drive(0.35, expected=False)
    hold_and_move()
    print("PASS deliberate hold reached /cmd_vel at +0.15 m/s and +0.35 rad/s", flush=True)

    released_at = time.monotonic()
    fake.held = False
    assert_zero_after(released_at, "release")
    print("PASS deadman release reached /cmd_vel as zero", flush=True)

    hold_and_move()
    lost_at = time.monotonic()
    fake.joy_online = False
    assert_zero_after(lost_at, "Joy loss")
    fake.joy_online = True
    reconnected_at = time.monotonic()
    drive(0.35, expected=False)
    assert not fake.moved_since(reconnected_at), "Held reconnect produced motion"
    print("PASS Joy loss stopped output and held reconnect remained inhibited", flush=True)

    fake.held = False
    drive(0.35, expected=False)
    hold_and_move()
    stopped_at = time.monotonic()
    fake.latched = True
    checks = TestCase()
    with checks.assertRaisesRegex(RuntimeError, "Stop gateway"):
        drive(0.5)
    until = stopped_at + 0.5
    while not fake.saw_velocity(ZERO, stopped_at) and time.monotonic() < until:
        time.sleep(0.01)
    assert fake.saw_velocity(ZERO, stopped_at), "Latched Stop did not reach /cmd_vel as zero"
    fake.held = False
    time.sleep(0.1)
    fake.held = True
    time.sleep(0.1)
    with checks.assertRaisesRegex(RuntimeError, "Stop gateway"):
        sink.poll()
    assert fake.velocity[-1][1] == ZERO, "Latched Stop let motion resume"
    print("PASS gateway Stop reached /cmd_vel as zero and inhibited release/press", flush=True)


def main():
    # Disabled kernel tunnel interfaces can appear even in a networkless container.
    if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1" or any(
        flags.parent.name != "lo" and int(flags.read_text(), 16) & 1
        for flags in Path("/sys/class/net").glob("*/flags")
    ):
        raise SystemExit("Run only in the network-disabled ROS test container.")
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--mux", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    args = parser.parse_args()

    from pupper_bci import robot

    with subprocess.Popen([
        str(args.mux), "--ros-args", "-p", "inputs:=[/kick_cmd_vel]", "-p", "timeout_ms:=150",
    ]) as mux:
        try:
            with fake_pupper(args.policy.resolve(strict=True)) as fake, patch.object(
                robot, "_check_runtime", return_value=lambda: True,
            ), robot.open_robot(Path("/fake-runtime-receipt.json"), ENABLE_BUTTON) as sink:
                exercise(fake, sink)
            assert mux.poll() is None, "Mux stopped during smoke test"
        finally:
            mux.terminate()
            try:
                mux.wait(timeout=3)
            except subprocess.TimeoutExpired:
                mux.kill()
                mux.wait(timeout=3)


if __name__ == "__main__":
    main()
