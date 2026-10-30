"""Persistent stock-policy Lab 6 session. Starting this launch homes the robot.

Run only under the reviewed native v5 environment with the operator present.
Motion controllers are configured inactive. A deliberate physical Options press
releases the existing gateway; subsequent Lab 6 API actions reuse this session.
Physical R3 latches Stop. There is no automatic restart or timed trial shutdown.
Root owns explicit session shutdown; API Stop alone requests zero walking speed.
"""
import hashlib
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

ROOT = Path(__file__).resolve().parent
CONFIG_SHA = "d7542febd4fd382d3c3e71e0c5a3fb8ea7a5b140f87ff1cb12ead1982d3854c0"
URDF_SHA = "4858c01f6ce722455b2c9fa554e158bef2215bc1c3e75b2b005fdf9b89aaed44"


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "168":
        raise RuntimeError("Lab 6 session requires explicit ROS_DOMAIN_ID=168")
    config_path = ROOT / "lab6-session.yaml"
    urdf = ROOT / "floor_stock" / "baseline-expanded.urdf"
    for path, expected in ((config_path, CONFIG_SHA), (urdf, URDF_SHA)):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError("Selected Lab 6 input changed: " + str(path))
    config = ParameterFile(str(config_path), allow_substs=True)
    description = {"robot_description": urdf.read_text()}
    state = Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[description], output="screen")
    manager = Node(package="controller_manager", executable="ros2_control_node",
                   parameters=[config, description], output="screen",
                   remappings=[("controller_manager:/controller_manager/switch_controller",
                                "/controller_manager/switch_controller_backend")])
    gateway = Node(package="joy_utils", executable="estop_controller",
                   parameters=[config], output="screen")
    mux = Node(package="cmd_vel_mux", executable="cmd_vel_mux_node",
               parameters=[config], output="screen")
    joy = Node(package="joy", executable="joy_node", name="floor_physical_joy",
               namespace="/", parameters=[config], output="screen")
    # A silent mux can otherwise leave a controller holding its last velocity.
    # Any required process exit tears down this owned session. No respawn.
    required = (state, manager, gateway, mux, joy)
    guards = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason="required Lab 6 session process exited"))]))
        for node in required]
    common = ["--controller-manager", "/controller_manager",
              "--controller-manager-timeout", "60", "--service-call-timeout", "10"]
    feedback = Node(package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "imu_sensor_broadcaster", *common], output="screen")
    motion = Node(package="controller_manager", executable="spawner",
        arguments=["neural_controller_default", "gemini_walk", "--inactive", *common], output="screen")
    return LaunchDescription([*guards, *required, feedback, motion])
