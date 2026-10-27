import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import signal
from control_math import bounded_sine_target, pd_torque, SampleDelay

JOINT_NAME = "leg_front_l_3"
JOINT_LOWER, JOINT_UPPER = -0.71, 2.79  # Pupper URDF, front-left knee
####
####
KP = 7.0  # YOUR KP VALUE
KD = 0.5  # YOUR KD VALUE
####
####
LOOP_RATE = 200  # Hz
MAX_TORQUE = 3.0


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__("joint_state_subscriber")
        # Create a subscriber to the /joint_states topic
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.get_joint_info, 10  # QoS profile history depth
        )
        self.subscription  # prevent unused variable warning

        # Publisher to the /forward_command_controller/commands topic
        self.command_publisher = self.create_publisher(Float64MultiArray, "/forward_command_controller/commands", 10)
        self.print_counter = 0
        self.calculated_torque = 0
        self.joint_pos = 0
        self.joint_vel = 0
        self.target_joint_pos = 0
        self.target_joint_vel = 0
        self.started_at = time.monotonic()
        self.last_state_at = None
        self.faulted = False
        self.declare_parameter('delay_seconds', 0.0)
        self.declare_parameter('target_amplitude', 0.2)
        self.delay = SampleDelay(float(self.get_parameter('delay_seconds').value), LOOP_RATE)
        self.target_center = None
        self.timer = self.create_timer(1.0 / LOOP_RATE, self.control_loop)

    def get_target_joint_info(self):
        # Start from the measured pose instead of commanding an arbitrary angle.
        return bounded_sine_target(time.monotonic() - self.started_at,
                           amplitude=float(self.get_parameter('target_amplitude').value),
                           center=self.target_center, lower=JOINT_LOWER, upper=JOINT_UPPER)

    def calculate_torque(self, joint_pos, joint_vel, target_joint_pos, target_joint_vel):
        return pd_torque(joint_pos, joint_vel, target_joint_pos, target_joint_vel, KP, KD, MAX_TORQUE)

    def print_info(self):
        """Print joint information every 2 control loops"""
        if self.print_counter == 0:
            self.get_logger().info(
                f"Pos: {self.joint_pos:.2f}, Target Pos: {self.target_joint_pos:.2f}, Vel: {self.joint_vel:.2f}, Target Vel: {self.target_joint_vel:.2f}, Tor: {self.calculated_torque:.2f}"
            )
        self.print_counter += 1
        self.print_counter %= 2

    def get_joint_info(self, msg):
        """Callback function to process incoming JointState messages"""
        try:
            if len(msg.name) != len(set(msg.name)) or len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
                raise ValueError('incomplete or duplicate joints')
            joint_index = msg.name.index(JOINT_NAME)
            joint_pos, joint_vel = float(msg.position[joint_index]), float(msg.velocity[joint_index])
        except (ValueError, IndexError, TypeError):
            self.fail_stop('Malformed joint state; torque disabled until restart.')
            return
        if not np.isfinite([joint_pos, joint_vel]).all():
            self.fail_stop('Nonfinite joint state; torque disabled until restart.')
            return
        self.joint_pos, self.joint_vel = joint_pos, joint_vel
        self.last_state_at = time.monotonic()
        if self.target_center is None:
            self.target_center = joint_pos
            self.started_at = self.last_state_at
        return joint_pos, joint_vel

    def fail_stop(self, reason):
        self.faulted = True
        self.delay.clear()
        self.publish_torque(0.0)
        self.get_logger().error(reason)

    def control_loop(self):
        if self.faulted or self.last_state_at is None:
            self.delay.clear()
            self.publish_torque(0.0)
            return
        if not 0 <= time.monotonic() - self.last_state_at <= 0.25:
            self.fail_stop('Joint states stopped arriving; torque disabled until restart.')
            return
        try:
            self.target_joint_pos, self.target_joint_vel = self.get_target_joint_info()
        except (ValueError, TypeError) as exc:
            self.fail_stop(f'Invalid target: {exc}')
            return
        observed = self.delay.push((self.joint_pos, self.joint_vel))
        if observed is None:
            self.publish_torque(0.0)
            return
        self.calculated_torque = self.calculate_torque(
            *observed, self.target_joint_pos, self.target_joint_vel)
        self.print_info()
        self.publish_torque(self.calculated_torque)

    def publish_torque(self, torque=0.0):
        # Create a Float64MultiArray message with zero kp and kd values
        command_msg = Float64MultiArray()
        torque = float(np.clip(torque, -MAX_TORQUE, MAX_TORQUE)) if np.isfinite(torque) else 0.0
        command_msg.data = [torque, 0.0, 0.0]  # Zero kp and kd values

        # Publish the message
        self.command_publisher.publish(command_msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # Create the node
    joint_state_subscriber = JointStateSubscriber()

    def _interrupt(sig, frame):
        raise KeyboardInterrupt
    previous = {s: signal.signal(s, _interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_subscriber.timer.cancel()
        if rclpy.ok():
            joint_state_subscriber.publish_torque(0.0)
            rclpy.spin_once(joint_state_subscriber, timeout_sec=0.05)
            joint_state_subscriber.publish_torque(0.0)
        joint_state_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for s, handler in previous.items():
            signal.signal(s, handler)
  

if __name__ == "__main__":
    main()
