"""Inverse kinematics and trajectory tracking on a single leg (handout Part 3).

Run the controller stack first (separate terminal):
    ros2 launch ik.launch.py
Then run this node:
    python3 ik.py

Only the three front-right joints are commanded here (see ik.yaml).
FK and IK (TODOs 1-4) go in kinematics.py, which walking.py reuses.
"""

import argparse
from pathlib import Path
import sys
import time
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

from kinematics import fr_leg_fk, inverse_kinematics, LEG_LIMITS
from controller_support import joint_sample, state_fresh, request_position_stop, confirm_position_stop

np.set_printoptions(precision=3, suppress=True)

# Gains for the on-board joint controller. We command positions here; the motor
# controller closes the PD loop around them.
Kp = 3
Kd = 0.1


class InverseKinematics(Node):

    def __init__(self, contact_model=None):
        super().__init__('inverse_kinematics')
        self.faulted = False
        self.stop_future = None
        self.last_state_at = None
        self.contact_predictor = None
        self.contact_clock = None
        self.pending_contact = None
        self.contact_started_at = None
        self.last_prediction_at = None
        if contact_model:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
            from implicit_compliance.streaming import ContactPredictor, ContactPauseClock
            self.contact_predictor = ContactPredictor(contact_model)
            self.contact_clock = ContactPauseClock()
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )

        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 40  # 40 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None

        self.ee_triangle_positions = np.array([
            [0.05, 0.0, -0.12],  # Touchdown; former 0.15 m exceeded bounded reach
            [-0.05, 0.0, -0.12], # Liftoff
            [0.0, 0.0, -0.09]    # Mid-swing
        ])
        # [0.05, 0.0, -0.12],  # Touchdown
        # [-0.05, 0.0, -0.12], # Liftoff
        # [0.0, 0.0, -0.06]    # Mid-swing

        center_to_rf_hip = np.array([0.07500, -0.08350, 0])
        self.ee_triangle_positions = self.ee_triangle_positions + center_to_rf_hip
        self.current_target = 0
        self.t = 0
        self.step_frequency = 0.33  # Hz; one triangle is 3 units of t

    def listener_callback(self, msg):
        joints_of_interest = ['leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3']
        sample = joint_sample(msg, joints_of_interest)
        if sample is None:
            self.fail_stop('Invalid joint state; restart only after checking the sensors.')
            return
        self.joint_positions, self.joint_velocities = sample
        now = time.monotonic()
        self.last_state_at = now
        if self.contact_predictor is not None and not self.faulted:
            from implicit_compliance.streaming import motor_channels
            try:
                channels = motor_channels(msg.name, msg.position, msg.velocity, msg.effort)
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                prediction = self.contact_predictor.push(stamp, channels)
            except (ValueError, IndexError, RuntimeError) as exc:
                self.fail_stop(f'Contact inference failed: {exc}')
                return
            if self.contact_started_at is None:
                self.contact_started_at = now
            if prediction is not None:
                self.pending_contact = prediction['contact']
                self.last_prediction_at = now

    def fail_stop(self, message):
        if not self.faulted:
            self.faulted = True
            self.target_joint_positions = None
            self.get_logger().error(message)
            self.stop_future = request_position_stop(self, (self.pd_timer, self.ik_timer))

    def sensors_ready(self):
        if self.faulted or self.last_state_at is None:
            return False
        now = time.monotonic()
        if not state_fresh(self.last_state_at, now):
            self.fail_stop('Joint states stopped arriving; controller stop requested.')
            return False
        if self.contact_predictor is not None:
            last = self.last_prediction_at or self.contact_started_at
            if last is not None and now - last > 1.0:
                self.fail_stop('No valid contact-inference window for one second; check sample timing and efforts.')
                return False
        return True

    def interpolate_triangle(self, t):
        # Interpolate between the three triangle positions in self.ee_triangle_positions
        # based on the current time t
        ################################################################################################
        # TODO 5: Implement the interpolation function
        td, l, md = self.ee_triangle_positions
        t = t%3
        if t<1:
            start=td
            end=l
            frac=t
        elif t<2:
            start=l
            end=md
            frac=t-1
        elif t<3:
            start=md
            end=td
            frac=t-2
            
        return start + frac*(end-start)
        ################################################################################################

    def ik_timer_callback(self):
        if self.sensors_ready():
            if self.contact_clock is not None:
                phase_time, contact = self.contact_clock.advance(time.monotonic(), self.pending_contact)
                self.pending_contact = None  # Every prediction is consumed exactly once.
                self.t = 3 * self.step_frequency * phase_time
                if contact:
                    self.get_logger().info('Contact detected; pausing the trajectory for one second.')
            target_ee = self.interpolate_triangle(self.t)
            # Ablation: bad init for IK causes divergence
            #             self.target_joint_positions = np.clip(inverse_kinematics(fr_leg_fk, target_ee, np.array([np.pi/2, -np.pi/2, np.pi/2])), -1.6, 1.6)
            try:
                self.target_joint_positions = inverse_kinematics(
                    fr_leg_fk, target_ee, self.joint_positions, max_iterations=150,
                    tolerance=1e-4, strict=True, joint_limits=LEG_LIMITS[0])
            except ValueError as exc:
                self.fail_stop(f'Trajectory rejected: {exc}')
                return
            current_ee = fr_leg_fk(self.joint_positions)

            # update the current time for the triangle interpolation
            ################################################################################################
            # TODO 6: Implement the time update
            if self.contact_clock is None:
                self.t += 3 * self.step_frequency * self.ik_timer_period
            ################################################################################################

            self.get_logger().info(f'Target EE: {target_ee}, Current EE: {current_ee}, Target Angles: {self.target_joint_positions}, Target Angles to EE: {fr_leg_fk(self.target_joint_positions)}, Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.sensors_ready() and self.target_joint_positions is not None:

            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contact-model', type=Path,
                        help='Optional Lab 1 exported model directory; enables contact pause')
    args, _ = parser.parse_known_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    inverse_kinematics = None
    def interrupt(sig, frame):
        raise KeyboardInterrupt
    previous = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        inverse_kinematics = InverseKinematics(contact_model=args.contact_model)
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        stop_node = inverse_kinematics or Node('ik_initialization_failed')
        timers = (inverse_kinematics.pd_timer, inverse_kinematics.ik_timer) if inverse_kinematics else ()
        future = (inverse_kinematics.stop_future if inverse_kinematics else None)
        future = future or request_position_stop(stop_node, timers)
        confirm_position_stop(stop_node, future)
        stop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for s, handler in previous.items():
            signal.signal(s, handler)


if __name__ == '__main__':
    main()
