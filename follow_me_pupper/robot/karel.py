# karel.py - Enhanced with Object Tracking
import time
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pygame

SOUNDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sounds')


class KarelPupper:
    def start():
        if not rclpy.ok():
            rclpy.init()

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('karel_node')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

        # Tracking control publisher
        self.tracking_control_publisher = self.node.create_publisher(
            String, '/tracking_control', 10
        )

    def _play_sound(self, filename):
        """Play a wav from sounds/. Never fatal -- a missing file just means silence."""
        path = os.path.join(SOUNDS_DIR, filename)
        try:
            pygame.mixer.init()
            pygame.mixer.Sound(path).play()
            self.node.get_logger().info(f'Playing {filename}')
        except Exception as e:
            self.node.get_logger().warning(f'Could not play {path}: {e}')

    def _warn_if_unheard(self):
        """Tracking commands are silent no-ops when follow_me.py is not running."""
        if self.tracking_control_publisher.get_subscription_count() == 0:
            self.node.get_logger().warning(
                'Nothing is subscribed to /tracking_control - is follow_me.py running? '
                'The command was published, but no one is there to act on it.'
            )

    def begin_tracking(self, obj: str = "person"):
        """
        Start tracking a specific object from the COCO dataset.

        Unlike move_forward() or bark(), this does not drive the robot itself --
        it asks the state machine in follow_me.py to switch into tracking mode. That
        node has to be running, or the message goes nowhere.

        Args:
            obj: Object class to track (e.g., "person", "dog", "cat", "car", etc.)
                 Default is "person". Uses COCO dataset class names.
        """
        self._warn_if_unheard()

        # Publish tracking command
        msg = String()
        msg.data = f"start:{obj}"
        self.tracking_control_publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().info(f'Started tracking: {obj}')

    def end_tracking(self):
        """Stop tracking and return to idle state."""
        self._warn_if_unheard()

        # Publish stop command
        msg = String()
        msg.data = "stop"
        self.tracking_control_publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Stop movement
        self.stop()

        self.node.get_logger().info('Stopped tracking')

    def move(self, linear_x, linear_y, angular_z):
        move_cmd = Twist()
        move_cmd.linear.x = linear_x
        move_cmd.linear.y = linear_y
        move_cmd.angular.z = angular_z
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move...')
        self.stop()

    def wiggle(self, wiggle_time=6, play_sound=True):
        # Play wiggle sound if requested
        if play_sound:
            self._play_sound('puppy_wiggle.wav')

        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        # Alternate wiggle directions for a total of 1 second
        single_wiggle_duration = 0.2  # seconds per half-wiggle
        angular_speed = 0.8

        start_time = time.time()
        direction = 1
        while time.time() - start_time < wiggle_time:
            move_cmd.angular.z = direction * angular_speed
            self.publisher.publish(move_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(single_wiggle_duration)
            direction *= -1  # Switch direction

        self.stop()

        self.node.get_logger().info('Wiggle!')

    def bob(self, bob_time=5, play_sound=True):
        # Play puppy bob sound if requested
        if play_sound:
            self._play_sound('puppy_bob.wav')

        twist = Twist()
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        half_bob_duration = 0.2  # seconds per half-bob
        bob_speed = 0.35

        start = time.time()
        direction = 1
        while time.time() - start < bob_time:
            twist.linear.x = direction * bob_speed
            self.publisher.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(half_bob_duration)
            direction *= -1  # Switch direction

        self.stop()

        self.node.get_logger().info('Bob!')

    def move_forward(self):
        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move forward...')
        self.stop()

    def move_backward(self):
        move_cmd = Twist()
        move_cmd.linear.x = -1.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move backward...')
        self.stop()

    def move_left(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.7
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move left...')
        self.stop()

    def move_right(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = -0.7
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move right...')
        self.stop()

    def turn_left(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 1.5
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Turn left...')
        self.stop()

    def turn_right(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -1.5
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Turn right...')
        self.stop()

    def bark(self):
        self.node.get_logger().info('Bark...')
        self._play_sound('dog_bark.wav')
        self.stop()

    def dance(self):
        self.node.get_logger().info('Rick Rolling...')
        self._play_sound('rickroll.wav')
        self.wiggle(wiggle_time=4, play_sound=False)
        self.turn_left()
        self.bob(bob_time=2.5, play_sound=False)
        self.turn_right()
        self.turn_right()
        self.bob(bob_time=2.5, play_sound=False)
        self.turn_left()
        self.stop()


    def stop(self):
        self.node.get_logger().info('Stopping...')
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)

    def __del__(self):
        # Runs during interpreter shutdown, when rclpy's own modules may already
        # be torn down -- never let cleanup throw a traceback in a student's face.
        try:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

