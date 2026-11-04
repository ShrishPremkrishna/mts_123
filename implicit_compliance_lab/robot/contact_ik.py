"""Lab 3 triangle trajectory on the front-right leg, paused for 1 s on implicit contact.

Contact comes only from the joint states (position, velocity, effort) of the three
front-right motors, through the exported implicit classifier. There is no foot sensor.

Run on Pupper, robot on its stand, two terminals:
    ros2 launch ../../ik_heuristic_walking_lab/ik.launch.py
    python3 contact_ik.py --no-pause --log free_swing.csv   # 1. detect only, nothing in reach
    python3 contact_ik.py --no-pause --log ramp.csv         # 2. detect only, ramp under the foot
    python3 contact_ik.py                                   # 3. pause 1 s on each touchdown

The IK and PD publishing are the team's Lab 3 node (ik_heuristic_walking_lab/ik.py); this file
subclasses it and adds the detector and the pause. By default it replaces the Lab 3 triangle
(+-15 cm stroke) with a dataset-sized one: the Lab 3 triangle is out of the leg's reach, so the
IK drives joints 2 and 3 past their limits, and the effort (kp times the tracking error) is
5 to 50 times larger than anything in the training data.
"""

import argparse
import csv
import os
import sys
import time

# One BLAS thread: the classifier is small, and extra threads compete with the 1 kHz callbacks.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
IK_LAB = os.environ.get('IK_LAB_DIR', os.path.join(HERE, '..', '..', 'ik_heuristic_walking_lab'))
sys.path.insert(0, os.path.abspath(IK_LAB))

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.utilities import remove_ros_args  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from contact_detector import (JOINTS, Hysteresis, JointStateBuffer, JointStateCNN,  # noqa: E402
                              PauseOnContact, PhaseBaseline)
from ik import InverseKinematics  # noqa: E402
from kinematics import fr_leg_fk, inverse_kinematics  # noqa: E402

# Foot vertices relative to the hip: touchdown, liftoff, mid-swing (m). "dataset" follows the
# foot path that forward kinematics gives for the recorded dataset joint angles
# (x -3.6..+4.5 cm, z -13.0..-6.9 cm). Mid-swing is at -8 cm so that the knee stays 0.14 rad
# inside its 0.61 rad limit.
TRIANGLES = {
    'dataset': [[0.04, 0.0, -0.125], [-0.035, 0.0, -0.125], [0.0, 0.0, -0.08]],
    'lab3': None,  # keep ik.py's own triangle
}
CENTER_TO_RF_HIP = np.array([0.07500, -0.08350, 0])


class ContactIK(InverseKinematics):

    def __init__(self, args):
        super().__init__()
        self.args = args
        if TRIANGLES[args.triangle] is not None:
            self.ee_triangle_positions = np.array(TRIANGLES[args.triangle]) + CENTER_TO_RF_HIP
        # The classifier was trained on the ~1 kHz joint_states stream; a deeper queue
        # than ik.py's 10 keeps samples while a slow IK callback runs.
        self.destroy_subscription(self.joint_subscription)
        self.joint_subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback, 200)
        self.model = JointStateCNN(args.model)
        self.buffer = JointStateBuffer(self.model.window)
        self.phase = np.zeros(self.model.window)   # trajectory time of each buffered sample
        if args.detector == 'residual':
            # Score = z-score of the effort residual, so the thresholds are in standard deviations.
            self.baseline = PhaseBaseline(args.baseline)
            self.hysteresis = Hysteresis(args.z_on, args.z_off, args.n_on, args.n_off)
        else:
            self.hysteresis = Hysteresis(args.p_on, args.p_off, args.n_on, args.n_off)
        self.pauser = PauseOnContact(args.pause_s, args.rearm_s)
        self.paused = False
        self.p_contact = 0.0
        self.contact = False
        self._index = None
        self._n_names = 0
        self._warned_effort = False
        self._warned_nan = False
        self._report = (0.0, 0)  # (time, buffer.count) at the last status line
        self.detect_timer = self.create_timer(1.0 / args.rate_hz, self.detect_timer_callback)
        self.log_file = open(args.log, 'w', newline='') if args.log else None
        if self.log_file:
            self.log = csv.writer(self.log_file)
            self.log.writerow(['timestamp'] + [f'{j}_{q}' for q in ('position', 'velocity', 'effort') for j in JOINTS]
                              + ['p_contact', 'contact', 'paused', 'traj_t'])
        self.get_logger().info(
            f'model {args.model}: window {self.model.window} samples, channels {self.model.channels.tolist()}, '
            f'inference {args.rate_hz} Hz, pause {"off" if args.no_pause else f"{args.pause_s} s"}, '
            f'triangle {args.triangle}, detector {args.detector}')

    def listener_callback(self, msg):
        if self._index is None or len(msg.name) != self._n_names:
            self._index = [msg.name.index(j) for j in JOINTS]
            self._n_names = len(msg.name)
        pos = [msg.position[i] for i in self._index]
        vel = [msg.velocity[i] for i in self._index]
        if len(msg.effort) == len(msg.name):
            eff = [msg.effort[i] for i in self._index]
        else:
            eff = [0.0, 0.0, 0.0]
            if not self._warned_effort:
                self.get_logger().error('joint_states has no effort field; the classifier needs it. '
                                        'Pause disabled.')
                self._warned_effort = True
                self.args.no_pause = True
        self.joint_positions = np.array(pos)
        self.joint_velocities = np.array(vel)
        self.phase[self.buffer.count % self.buffer.size] = self.t
        self.buffer.push(pos, vel, eff)
        if self.log_file:
            stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            self.log.writerow([f'{stamp:.6f}', *pos, *vel, *eff,
                               f'{self.p_contact:.4f}', int(self.contact), int(self.paused), f'{self.t:.4f}'])

    def detect_timer_callback(self):
        if not self.buffer.full():
            return
        window = self.buffer.window()
        p, incline, material = self.model(window)
        self.p_contact = float(p[0])
        if self.args.detector == 'residual':
            i = self.buffer.count % self.buffer.size
            phase = np.concatenate([self.phase[i:], self.phase[:i]])
            self.p_contact = self.baseline.score(window[:, 6:9], phase)
        if not np.isfinite(self.p_contact):
            # A NaN joint state would otherwise freeze the hysteresis; treat it as release.
            if not self._warned_nan:
                self.get_logger().error('non-finite joint state in the window; treating it as no contact')
                self._warned_nan = True
            self.p_contact = 0.0
        self.contact, rising = self.hysteresis.update(self.p_contact)
        now = time.monotonic()  # pause timing must not jump with the system clock
        if now - self._report[0] >= 5.0:
            if self._report[0]:
                rate = (self.buffer.count - self._report[1]) / (now - self._report[0])
                self.get_logger().info(f'joint_states {rate:.0f} Hz (trained at ~970 Hz), '
                                       f'p_contact {self.p_contact:.2f}, pauses {self.pauser.n_pauses}')
            self._report = (now, self.buffer.count)
        was_paused = self.paused
        self.paused = False if self.args.no_pause else self.pauser.update(now, self.contact, rising)
        if rising:
            self.get_logger().info(
                f'CONTACT score={self.p_contact:.2f} at triangle t={self.t % 3:.2f} | CNN guess: '
                f'{self.model.incline_classes[int(incline[0].argmax())]} deg, '
                f'{self.model.material_classes[int(material[0].argmax())]}'
                + (' -> pause' if self.paused and not was_paused else ''))

    def ik_timer_callback(self):
        # While paused, keep the last joint target (pd_timer keeps publishing it) and freeze t.
        if self.joint_positions is None or self.paused:
            return
        target_ee = self.interpolate_triangle(self.t)
        # Start the IK from its previous solution (zero pose at first), not from the measured
        # joints: a leg that rests at an odd pose could otherwise send the IK to another branch.
        guess = self.target_joint_positions if self.target_joint_positions is not None else np.zeros(3)
        self.target_joint_positions = inverse_kinematics(fr_leg_fk, target_ee, guess)
        self.t += 3 * self.step_frequency * self.ik_timer_period


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=os.path.join(HERE, 'implicit_classifier.npz'))
    ap.add_argument('--triangle', choices=list(TRIANGLES), default='dataset',
                    help="foot triangle: 'dataset' (default, reachable) or 'lab3' (ik.py's own)")
    ap.add_argument('--detector', choices=['residual', 'cnn'], default='residual',
                    help="'residual': effort against a free-swing baseline of this robot (default); "
                         "'cnn': the dataset-trained classifier alone")
    ap.add_argument('--baseline', default=os.path.join(HERE, 'free_swing_baseline.npz'),
                    help='output of: python3 contact_detector.py calibrate free_swing.csv ...')
    ap.add_argument('--z-on', type=float, default=4.0, help='residual z-score that counts as contact')
    ap.add_argument('--z-off', type=float, default=2.0, help='residual z-score that counts as release')
    ap.add_argument('--rate-hz', type=float, default=50.0, help='classifier evaluations per second')
    ap.add_argument('--p-on', type=float, default=0.7, help='probability that counts as contact')
    ap.add_argument('--p-off', type=float, default=0.3, help='probability that counts as release')
    ap.add_argument('--n-on', type=int, default=3, help='consecutive evaluations to set contact')
    ap.add_argument('--n-off', type=int, default=5, help='consecutive evaluations to clear contact')
    ap.add_argument('--pause-s', type=float, default=1.0)
    ap.add_argument('--rearm-s', type=float, default=0.2,
                    help='continuous release after a pause that arms the next pause')
    ap.add_argument('--no-pause', action='store_true', help='detect and log only; never pause')
    ap.add_argument('--log', help='CSV of every joint_states sample with detector output')
    return ap.parse_args(argv)


def main():
    args = parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = ContactIK(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Program terminated by user')
    except RuntimeError:
        # Ctrl-C during a message conversion surfaces as RuntimeError after the context shuts down.
        if rclpy.ok():
            raise
        print('Program terminated by user')
    finally:
        # The position controller keeps the last command, so the leg holds still.
        # (ik.py publishes [0, 0, 0] here, which moves the leg to the zero pose.)
        print(f'{node.pauser.n_pauses} pauses')
        if node.log_file:
            node.log_file.close()
        node.destroy_node()
        if rclpy.ok():  # Ctrl-C can already have shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
