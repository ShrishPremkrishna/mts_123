"""Supervised hardware homing/feedback candidate. EXECUTION ENABLES MOTORS.

All motion controllers stay unloaded. Physical Start/face buttons must not be
used until the separately approved default-loading phase. No activation is
scheduled by this file. Shutdown timer is software only, not a power cutoff.
"""
import hashlib
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
ROOT = Path(__file__).resolve().parent
CONFIG_SHA = '6b25c2a68b99b99f1679e701dce49fc72ce7992c9754a517f70a983cfb4c4661'
URDF_SHA = '4858c01f6ce722455b2c9fa554e158bef2215bc1c3e75b2b005fdf9b89aaed44'

def generate_launch_description():
    import sys
    sys.path.insert(0,str(ROOT))
    from motion_gate import require_motion_enabled
    require_motion_enabled()
    if os.environ.get('ROS_DOMAIN_ID') != '168':
        raise RuntimeError('Console supervised stack requires isolated ROS_DOMAIN_ID=168')
    config_path, urdf = ROOT / 'course-config-v4.yaml', ROOT / 'baseline-expanded.urdf'
    for path, expected in [(config_path, CONFIG_SHA), (urdf, URDF_SHA)]:
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Pinned hardware input differs: ' + str(path))
    config = ParameterFile(str(config_path), allow_substs=True)
    description = {'robot_description': urdf.read_text()}
    state = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[description], output='screen')
    manager = Node(package='controller_manager', executable='ros2_control_node',
        parameters=[config, description], output='screen',
        remappings=[('controller_manager:/controller_manager/switch_controller', '/controller_manager/switch_controller_backend')])
    gateway = Node(package='joy_utils', executable='estop_controller', parameters=[config], output='screen')
    # Losing the manager or software authority terminates this
    # minimal stack. Homing itself need not respond promptly to ROS shutdown.
    exit_handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='required supervised process exited'))]))
        for node in (manager, gateway)]
    feedback = [Node(package='controller_manager', executable='spawner',
        arguments=[name, '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60', '--service-call-timeout', '10'],
        output='screen') for name in ('joint_state_broadcaster', 'imu_sensor_broadcaster')]
    limit = TimerAction(period=180.0, actions=[EmitEvent(event=Shutdown(reason='180-second supervised feedback bound'))])
    return LaunchDescription([*exit_handlers, state, manager, gateway, *feedback, limit])
