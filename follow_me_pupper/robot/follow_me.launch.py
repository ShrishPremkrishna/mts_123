import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    ThisLaunchFileDir,
)
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_white_balance():
    """camera_ros controls for the white_balance block in camera_params.yaml."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_params.yaml")
    with open(path) as f:
        wb = yaml.safe_load(f).get("white_balance", {})
    if wb.get("awb_enable", True):
        return {}
    return {"AwbEnable": False, "ColourGains": [float(g) for g in wb["colour_gains"]]}


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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Use neural_controller package config (same as lab_6)
    robot_controllers = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare("neural_controller"),
                "launch",
                "config.yaml",
            ]
        ),
        allow_substs=True,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    # The default reference-gait policy from pupper_gait_deploy is the gait for
    # this lab -- it is the one that actually keeps up with a walking person.
    # Everything the policy pins down (kp, kd, action scale, gait reference
    # tables) lives in default_policy.json, which the neural_controller_default
    # section of the controller config points at.
    #
    #   ros2 launch robot/follow_me.launch.py controller:=neural_controller   # old walk gait
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            LaunchConfiguration("controller"),
            "--param-file",
            PathJoinSubstitution([ThisLaunchFileDir(), "speed_cap.yaml"]),
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
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

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        output="both",
        parameters=[{"format": "RGB888", "width": 1400, "height": 1050, **load_white_balance()}],
    )

    # Live fisheye view in the browser, with YOLOv8n-seg running on the Hailo
    # chip for bounding boxes and segmentation masks. Subscribes to the
    # camera_node topic above (the sensor can only be opened by one process), so
    # it just piggybacks, and it publishes /detections for follow_me.py to track.
    #
    #   ros2 launch robot/follow_me.launch.py viser:=false    # no web viewer at all
    #   ros2 launch robot/follow_me.launch.py detect:=false   # viewer, no model (frees the
    #                                               # Hailo accelerator, but then
    #                                               # nothing publishes /detections
    #                                               # for follow_me.py to track)
    viser_camera = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([ThisLaunchFileDir(), "viser_camera.py"]),
            "--source",
            "ros",
            "--port",
            LaunchConfiguration("viser_port"),
            PythonExpression(
                ["'--detect' if '", LaunchConfiguration("detect"), "'.lower() in ",
                 "('true', '1') else '--no-detect'"]
            ),
        ],
        cwd=ThisLaunchFileDir(),
        output="both",
        condition=IfCondition(LaunchConfiguration("viser")),
    )

    nodes = [
        DeclareLaunchArgument(
            "controller",
            default_value="neural_controller_default",
            description=(
                "Which neural_controller instance to spawn. Defaults to the "
                "reference-gait policy (default_policy.json); pass neural_controller for "
                "the older walking policy."
            ),
        ),
        DeclareLaunchArgument(
            "viser",
            default_value="true",
            description="Serve the fisheye camera stream in a viser web viewer.",
        ),
        DeclareLaunchArgument(
            "viser_port", default_value="8080", description="Port for the viser viewer."
        ),
        DeclareLaunchArgument(
            "detect",
            default_value="true",
            description=(
                "Run YOLOv8n-seg (boxes + segmentation masks) in the viser viewer "
                "and publish /detections, which follow_me.py tracks. Set false for a "
                "plain camera view that leaves the Hailo accelerator free."
            ),
        ),
        robot_state_publisher,
        imu_sensor_broadcaster_spawner,
        control_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        camera_node,
        viser_camera,
    ]

    return LaunchDescription(nodes)
