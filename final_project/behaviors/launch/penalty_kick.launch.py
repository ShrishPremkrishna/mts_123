"""Penalty kick on top of a running course stack (``ros2 launch neural_controller launch.py teleop:=False``).

Starts a cmd_vel_mux with /kick_cmd_vel as its only input, penalty_kick_node,
and (detector:=true) the ball detector with the 2026-11-16 beach-ball calibration.
The leg kick policy must be in the kick slot (neural_controller_student).

    ros2 launch ~/kick/launch/penalty_kick.launch.py gateway:=false detector:=true target_x:=2.0 target_y:=0.2
    ros2 service call /activate_penalty_kick std_srvs/srv/Trigger

target_x/target_y: the goal point from the ball, along the robot's start heading
(goal_frame ball). See PENALTY_KICK.md for the walker calibration values.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parents[1]
FLOATS = ("target_x", "target_y", "kick_delay_s", "speed_efficiency", "creep_efficiency",
          "walker_lag_s", "walk_drift", "stage_x")
STRINGS = ("goal_frame", "side", "strike")


def _nodes(context):
  arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
  gateway = arg("gateway").lower() in ("true", "1")
  params = ["-p", f"require_gateway:={str(gateway).lower()}", "-p", f"start_button:={int(arg('start_button'))}"]
  for name in FLOATS:
    params += ["-p", f"{name}:={float(arg(name))}"]
  for name in STRINGS:
    params += ["-p", f"{name}:={arg(name)}"]
  nodes = [
    Node(package="cmd_vel_mux", executable="cmd_vel_mux_node", name="kick_cmd_vel_mux",
         parameters=[{"inputs": ["/kick_cmd_vel"], "deadband": 0.05, "timeout_ms": 150}],
         output="both"),
    ExecuteProcess(cmd=["python3", "-u", str(HERE / "penalty_kick_node.py"), "--ros-args", *params],
                   output="both"),
  ]
  if arg("detector").lower() in ("true", "1"):
    nodes.append(ExecuteProcess(
      cmd=["python3", "-u", str(HERE / "ball_detector_node.py"), "--ros-args",
           "-p", f"ball_radius:={float(arg('ball_radius'))}", "-p", "mode:=color",
           "-p", f"range_scale:={float(arg('range_scale'))}",
           "-p", f"cam_y:={float(arg('cam_y'))}", "-p", f"cam_yaw_deg:={float(arg('cam_yaw_deg'))}",
           "-p", f"use_imu:={arg('detector_imu').lower()}"],
      cwd=str(HERE), output="both"))
  return nodes


def generate_launch_description():
  args = [
    ("gateway", "true", "false on older course stacks with no e-stop gateway"),
    ("detector", "false", "true: also start ball_detector_node (color mode)"),
    ("detector_imu", "false", "detector IMU levelling; false until the IMU tilt fault is fixed"),
    ("ball_radius", "0.097", "m, beach ball"),
    ("range_scale", "0.616", "detector range calibration (2026-11-16)"),
    ("cam_y", "0.0", "m, detector lateral offset (PENALTY_KICK.md calibration step 1)"),
    ("cam_yaw_deg", "0.0", "deg, detector yaw offset (same step)"),
    ("goal_frame", "ball", "ball, start or topic"),
    ("target_x", "2.0", "m, goal point from the ball along the start heading"),
    ("target_y", "0.0", "m, goal point to the left"),
    ("side", "auto", "right, left or auto"),
    ("strike", "leg_kick", "leg_kick, or none for a dry run that stops at the strike point"),
    ("kick_delay_s", "5.5", "switch-in to clip request; 2.0 with the 0.5 s + 0.5 s slot override"),
    ("speed_efficiency", "0.8", "walker achieved / commanded speed at 0.3 m/s"),
    ("creep_efficiency", "0.8", "same at 0.08 to 0.15 m/s"),
    ("walker_lag_s", "0.15", "walker velocity lag"),
    ("walk_drift", "0.0", "m left per m forward"),
    ("stage_x", "0.60", "m, ball distance for the fine alignment"),
    ("start_button", "-1", "joystick button that starts an attempt; -1 = service only"),
  ]
  return LaunchDescription([DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in args]
                           + [OpaqueFunction(function=_nodes)])
