import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import math 
from collections import deque
import signal
from control_math import sine_target, pd_torque, SampleDelay

JOINT_NAME = "leg_front_l_3"
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
        self.declare_parameter('delay_seconds', 0.0)
        self.declare_parameter('target_amplitude', 0.2)
        self.delay = SampleDelay(float(self.get_parameter('delay_seconds').value), LOOP_RATE)
        self.target_center = None
        self.create_timer(1.0 / LOOP_RATE, self.control_loop)

    def get_target_joint_info(self):
        # Start from the measured pose instead of commanding an arbitrary angle.
        return sine_target(time.monotonic() - self.started_at,
                           amplitude=float(self.get_parameter('target_amplitude').value),
                           center=self.target_center or 0.0)

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
            joint_index = msg.name.index(JOINT_NAME)
            joint_pos, joint_vel = float(msg.position[joint_index]), float(msg.velocity[joint_index])
        except (ValueError, IndexError, TypeError):
            return
        if not np.isfinite([joint_pos, joint_vel]).all():
            return
        self.joint_pos, self.joint_vel = joint_pos, joint_vel
        self.last_state_at = time.monotonic()
        if self.target_center is None:
            self.target_center = joint_pos
            self.started_at = self.last_state_at
        return joint_pos, joint_vel

    def control_loop(self):
        if self.last_state_at is None or time.monotonic() - self.last_state_at > 0.25:
            self.delay.clear()
            self.publish_torque(0.0)
            return
        self.target_joint_pos, self.target_joint_vel = self.get_target_joint_info()
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
        torque = np.clip(torque, -MAX_TORQUE, MAX_TORQUE)
        command_msg.data = [torque, 0.0, 0.0]  # Zero kp and kd values

        # Publish the message
        self.command_publisher.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    joint_state_subscriber = JointStateSubscriber()

    # Install a SIGINT handler that sends zero torque BEFORE shutting down the context
    def _handle_sigint(sig, frame):
        joint_state_subscriber.get_logger().info("SIGINT received: sending zero torque and shutting down...")
        joint_state_subscriber.publish_torque(0.0)
        time.sleep(0.1) 
        joint_state_subscriber.publish_torque(0.0)
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _handle_sigint)

    rclpy.spin(joint_state_subscriber)
  

if __name__ == "__main__":
    main()
