"""Explicit supervised stock startup. This CAN HOME; preflight alone cannot.

Uses a pinned expanded config, one walker, the maintained stop gateway and one
private velocity mux. No camera, voice, alternate policies or automatic restart.
"""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from runtime_identity import managers, read_json, verify_prepared


def selected_inputs():
    receipt_path = os.environ.get("PUPPER_RUNTIME_RECEIPT")
    if not receipt_path:
        raise ValueError("Set PUPPER_RUNTIME_RECEIPT to a completed preflight receipt")
    files = verify_prepared(read_json(receipt_path))
    if managers(Path("/proc")):
        raise ValueError("Controller manager already running; retain its session or coordinate shutdown")
    if files["launch"]["path"] != str(Path(__file__).resolve()):
        raise ValueError("This launch is not the selected launch")
    expected = {
        "driver": ("control_board_hardware_interface", "lib/libcontrol_board_hardware_interface.so"),
        "controller": ("neural_controller", "lib/libneural_controller.so"),
        "gateway": ("joy_utils", "lib/joy_utils/estop_controller"),
        "mux": ("cmd_vel_mux", "lib/cmd_vel_mux/cmd_vel_mux_node"),
    }
    for role, (package, relative) in expected.items():
        actual = (Path(get_package_prefix(package)) / relative).resolve(strict=True)
        if str(actual) != files[role]["path"]:
            raise ValueError(f"ROS selects the wrong {role}: {actual}")
    if "$(" in Path(files["config"]["path"]).read_text():
        raise ValueError("Expand configuration substitutions before preflight")
    return files


def generate_launch_description():
    files = selected_inputs()  # all checks occur before any launch action
    config = files["config"]["path"]
    state = Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[{"robot_description": Path(files["urdf"]["path"]).read_text()}], output="both")
    manager = Node(package="controller_manager", executable="ros2_control_node",
                   parameters=[config], output="both",
                   remappings=[("controller_manager:/controller_manager/switch_controller",
                                "/controller_manager/switch_controller_backend")])
    gateway = Node(package="joy_utils", executable="estop_controller", parameters=[config], output="both")
    mux = Node(package="cmd_vel_mux", executable="cmd_vel_mux_node", parameters=[config], output="both")
    joy = Node(package="joy", executable="joy_node", parameters=[config], output="both")
    required = (state, manager, gateway, mux, joy)
    exits = [RegisterEventHandler(OnProcessExit(target_action=n,
              on_exit=[EmitEvent(event=Shutdown(reason="required stock-session process exited"))])) for n in required]
    common = ["--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"]
    feedback = Node(package="controller_manager", executable="spawner", output="both",
                    arguments=["joint_state_broadcaster", "imu_sensor_broadcaster", *common])
    walker = Node(package="controller_manager", executable="spawner", output="both",
                  arguments=["neural_controller_default", "--inactive", "--param-file", config, *common])
    def spawner_finished(event, context):
        return ([EmitEvent(event=Shutdown(reason="stock controller setup failed"))]
                if event.returncode != 0 else [])
    exits.extend(RegisterEventHandler(OnProcessExit(target_action=n, on_exit=spawner_finished))
                 for n in (feedback, walker))
    return LaunchDescription([*exits, *required, feedback, walker])
