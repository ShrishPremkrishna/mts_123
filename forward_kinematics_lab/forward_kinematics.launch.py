from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pupper_v3_description"),
                    "description",
                    "pupper_v3.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Get config file relative to this launch file
    lab_dir = os.path.dirname(os.path.abspath(__file__))
    robot_controllers = PathJoinSubstitution([lab_dir, "forward_kinematics.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    kp_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_kp_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    kd_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_kd_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="both",
    )

    # Browser-based 3D viewer (robot model + end-effector markers) at http://<pupper-ip>:8080.
    #
    #   ros2 launch forward_kinematics.launch.py viser:=false      # no web viewer (use RViz2)
    #   ros2 launch forward_kinematics.launch.py viser_port:=8081  # different port
    viser_viewer = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(lab_dir, "forward_kinematics_viser.py"),
            "--port",
            LaunchConfiguration("viser_port"),
        ],
        cwd=lab_dir,
        output="both",
        condition=IfCondition(LaunchConfiguration("viser")),
    )

    nodes = [
        DeclareLaunchArgument(
            "viser",
            default_value="true",
            description="Serve the robot model and end-effector markers in a viser web viewer.",
        ),
        DeclareLaunchArgument("viser_port", default_value="8080", description="Port for the viser viewer."),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        kp_spawner,
        kd_spawner,
        foxglove_bridge,
        viser_viewer,
    ]

    return LaunchDescription(nodes)
