import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
import numpy as np
import socket
import subprocess

VISER_PORT = 8080  # must match viser_port in forward_kinematics.launch.py


def print_viewer_links(port=VISER_PORT):
    """Print where to open the 3D viewer (started by forward_kinematics.launch.py)."""
    try:
        ips = [ip for ip in subprocess.check_output(["hostname", "-I"], text=True).split() if ":" not in ip]
    except Exception:
        ips = []
    target = ips[0] if ips else f"{socket.gethostname()}.local"
    print("\n" + "=" * 64)
    print("  3D viewer (robot + end-effector markers)")
    print("=" * 64)
    print("\n  On the same wifi as the Pupper, open in your browser:")
    for ip in ips:
        print(f"      http://{ip}:{port}")
    print("\n  Over SSH only? Run this on your LAPTOP (new terminal),")
    print(f"  then open http://localhost:{port} there:")
    print(f"\n      ssh -N -L {port}:localhost:{port} pi@{target}")
    print("\n" + "=" * 64 + "\n", flush=True)

# The four legs, in the same order as the joints in forward_kinematics.yaml.
LEGS = ["front_r", "front_l", "back_r", "back_l"]

# RGB color used for each leg's end-effector marker (RViz and viser).
LEG_COLORS = {
    "front_r": (1.0, 0.0, 0.0),  # red
    "front_l": (0.0, 1.0, 0.0),  # green
    "back_r": (1.0, 1.0, 0.0),  # yellow
    "back_l": (0.0, 0.5, 1.0),  # blue
}


class ForwardKinematics(Node):

    def __init__(self):
        super().__init__("forward_kinematics")
        self.joint_subscription = self.create_subscription(JointState, "joint_states", self.listener_callback, 10)
        self.joint_subscription  # prevent unused variable warning

        # One end-effector position topic per leg, e.g. leg_front_l_end_effector_position
        self.position_publishers = {
            leg: self.create_publisher(Float64MultiArray, f"leg_{leg}_end_effector_position", 10) for leg in LEGS
        }
        self.marker_publisher = self.create_publisher(Marker, "marker", 10)

        self.joint_positions = None  # dict: leg name -> [theta1, theta2, theta3]
        timer_period = 0.02  # publish FK information and markers at 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.kp_publisher = self.create_publisher(Float64MultiArray, "/forward_kp_controller/commands", 10)
        self.kd_publisher = self.create_publisher(Float64MultiArray, "/forward_kd_controller/commands", 10)

        # Periodically set gains to 0 so legs go limp
        self.create_timer(0.1, self.publish_zero_gains)

        self.fk_functions = {
            "front_l": self.fk_front_left,
            "front_r": self.fk_front_right,
            "back_l": self.fk_back_left,
            "back_r": self.fk_back_right,
        }

        print_viewer_links()
        self.get_logger().info("Publishing end-effector positions on leg_<leg>_end_effector_position and spheres on /marker")

    def publish_zero_gains(self):
        self.kp_publisher.publish(Float64MultiArray(data=[0.0] * 12))
        self.kd_publisher.publish(Float64MultiArray(data=[0.0] * 12))

    def listener_callback(self, msg):
        # Extract the positions of the three joints of every leg, e.g. leg_front_l_1, leg_front_l_2, leg_front_l_3
        self.joint_positions = {
            leg: [msg.position[msg.name.index(f"leg_{leg}_{i}")] for i in (1, 2, 3)] for leg in LEGS
        }

    ######################## Homogeneous transforms ########################

    def rotation_x(self, angle):
        # rotation about the x-axis implemented for you
        return np.array(
            [
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1],
            ]
        )

    def rotation_y(self, angle):
        ## TODO: Implement the rotation matrix about the y-axis
        # return np.array([
        # ])
        raise NotImplementedError()

    def rotation_z(self, angle):
        ## TODO: Implement the rotation matrix about the z-axis
        # return np.array([
        # ])
        raise NotImplementedError()

    def translation(self, x, y, z):
        ## TODO: Implement the translation matrix
        # return np.array([
        # ])
        raise NotImplementedError()

    ######################## Per-leg forward kinematics ########################
    #
    # Each function takes the three joint angles of one leg (hip abduction/adduction,
    # hip flexion/extension, knee) and returns the position of that leg's end effector
    # (the foot) as a 3-vector in the base_link frame.
    #
    # Hip motor (joint 1) positions in base_link, as in the lab diagrams. Each is
    # followed by the 0.039 offset from motor 1 to motor 2, expressed in the frame
    # of motor 1 (see the leg_front_l_1 -> leg_front_l_2 diagram).
    #
    #   leg        x        y
    #   front_l   +0.07500  +0.04450
    #   front_r   +0.07500  -0.04450
    #   back_l    -0.07500  +0.03350
    #   back_r    -0.07500  -0.03350
    #
    # The right legs are mirror images of the left legs. Move each leg by hand and check
    # that its marker follows the foot to verify your transforms.

    def fk_front_left(self, theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            self.rotation_x,
            self.rotation_y,
            self.rotation_z,
            self.translation,
        )

        ############# Motor conventions according to slides #########

        # T_0_1 (base_link to leg_front_l_1)
        T_0_1 = translation(0.07500, 0.04450, 0) @ rotation_x(1.57080) @ rotation_z(-theta1)

        # T_1_2 (leg_front_l_1 to leg_front_l_2)
        ## TODO: Implement the transformation matrix from leg_front_l_1 to leg_front_l_2
        T_1_2 = None

        # T_2_3 (leg_front_l_2 to leg_front_l_3)
        ## TODO: Implement the transformation matrix from leg_front_l_2 to leg_front_l_3
        T_2_3 = None

        # T_3_ee (leg_front_l_3 to end-effector)
        T_3_ee = None

        # TODO: Compute the final transformation. T_0_ee is the multiplication of the previous transformation matrices
        T_0_ee = None

        # TODO: Extract the end-effector position. The end effector position is a 3x1 vector (not in homogenous coordinates)
        end_effector_position = None

        return end_effector_position

    def fk_front_right(self, theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            self.rotation_x,
            self.rotation_y,
            self.rotation_z,
            self.translation,
        )

        ## TODO: Implement the forward kinematics of the front-right leg, following the same
        ## structure as fk_front_left (T_0_1, T_1_2, T_2_3, T_3_ee, T_0_ee). See the hip origin table above.

        # T_0_1 (base_link to leg_front_r_1)
        T_0_1 = None

        # T_1_2 (leg_front_r_1 to leg_front_r_2)
        T_1_2 = None

        # T_2_3 (leg_front_r_2 to leg_front_r_3)
        T_2_3 = None

        # T_3_ee (leg_front_r_3 to end-effector)
        T_3_ee = None

        # Compute the final transformation
        T_0_ee = None

        # Extract the end-effector position
        end_effector_position = None

        return end_effector_position

    def fk_back_left(self, theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            self.rotation_x,
            self.rotation_y,
            self.rotation_z,
            self.translation,
        )

        ## TODO: Implement the forward kinematics of the back-left leg, following the same
        ## structure as fk_front_left (T_0_1, T_1_2, T_2_3, T_3_ee, T_0_ee). See the hip origin table above.

        # T_0_1 (base_link to leg_back_l_1)
        T_0_1 = None

        # T_1_2 (leg_back_l_1 to leg_back_l_2)
        T_1_2 = None

        # T_2_3 (leg_back_l_2 to leg_back_l_3)
        T_2_3 = None

        # T_3_ee (leg_back_l_3 to end-effector)
        T_3_ee = None

        # Compute the final transformation
        T_0_ee = None

        # Extract the end-effector position
        end_effector_position = None

        return end_effector_position

    def fk_back_right(self, theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            self.rotation_x,
            self.rotation_y,
            self.rotation_z,
            self.translation,
        )

        ## TODO: Implement the forward kinematics of the back-right leg, following the same
        ## structure as fk_front_left (T_0_1, T_1_2, T_2_3, T_3_ee, T_0_ee). See the hip origin table above.

        # T_0_1 (base_link to leg_back_r_1)
        T_0_1 = None

        # T_1_2 (leg_back_r_1 to leg_back_r_2)
        T_1_2 = None

        # T_2_3 (leg_back_r_2 to leg_back_r_3)
        T_2_3 = None

        # T_3_ee (leg_back_r_3 to end-effector)
        T_3_ee = None

        # Compute the final transformation
        T_0_ee = None

        # Extract the end-effector position
        end_effector_position = None

        return end_effector_position

    ######################## Publishing ########################

    def make_marker(self, leg, marker_id, end_effector_position):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = leg
        marker.id = marker_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        r, g, b = LEG_COLORS[leg]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(end_effector_position[0])
        marker.pose.position.y = float(end_effector_position[1])
        marker.pose.position.z = float(end_effector_position[2])
        return marker

    def timer_callback(self):
        """Timer callback for publishing end-effector markers and positions of all four legs."""
        if self.joint_positions is None:
            return

        for marker_id, leg in enumerate(LEGS):
            theta1, theta2, theta3 = self.joint_positions[leg]

            try:
                end_effector_position = self.fk_functions[leg](theta1, theta2, theta3)
            except NotImplementedError:
                end_effector_position = None

            if end_effector_position is None:
                # This leg's FK is not implemented yet -- skip it so the other legs still show up.
                continue

            end_effector_position = np.asarray(end_effector_position, dtype=float).reshape(3)

            self.marker_publisher.publish(self.make_marker(leg, marker_id, end_effector_position))

            position = Float64MultiArray()
            position.data = end_effector_position.tolist()
            self.position_publishers[leg].publish(position)

def main(args=None):
    rclpy.init(args=args)

    forward_kinematics = ForwardKinematics()

    rclpy.spin(forward_kinematics)


if __name__ == "__main__":
    main()
