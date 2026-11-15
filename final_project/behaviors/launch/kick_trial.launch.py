"""Kick trial nodes on top of a running course stack (``ros2 launch neural_controller launch.py``).

Run-through kick: a cmd_vel_mux with /kick_cmd_vel as its only input (it sends
zero when the kick node goes quiet) plus run_through_kick_node. Learned kick:
learned_kick_node, which writes the command slot on /cmd_vel itself.

    ros2 launch final_project/behaviors/launch/kick_trial.launch.py kick:=run_through input_mode:=manual charge_speed:=0.45
    ros2 launch final_project/behaviors/launch/kick_trial.launch.py kick:=learned input_mode:=robot

Only with the plain course stack, started WITHOUT joystick teleop:
    ros2 launch neural_controller launch.py teleop:=False
(teleop defaults to True and would publish /cmd_vel at 50 Hz over the kick; both
nodes refuse to start while another node publishes /cmd_vel). The Gemini stack
has its own mux: add /kick_cmd_vel to its inputs instead of using this launch.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parents[1]


def _nodes(context):
  kick = LaunchConfiguration("kick").perform(context)
  mode = LaunchConfiguration("input_mode").perform(context)
  if kick == "run_through":
    speed = LaunchConfiguration("charge_speed").perform(context)
    efficiency = LaunchConfiguration("speed_efficiency").perform(context)
    return [
      Node(package="cmd_vel_mux", executable="cmd_vel_mux_node", name="kick_cmd_vel_mux",
           parameters=[{"inputs": ["/kick_cmd_vel"], "deadband": 0.05, "timeout_ms": 150}],
           output="both"),
      ExecuteProcess(cmd=["python3", "-u", str(HERE / "run_through_kick_node.py"), "--ros-args",
                          "-p", f"input_mode:={mode}", "-p", f"charge_speed:={float(speed)}",
                          "-p", f"speed_efficiency:={float(efficiency)}"],
                     output="both"),
    ]
  if kick == "learned":
    if mode == "manual":
      raise RuntimeError("the learned kick needs live ball updates: input_mode robot or field")
    fade = LaunchConfiguration("fade_in_s").perform(context)
    end_range = 0.30 if mode == "field" else 0.45  # overhead tracking has no camera blind zone
    return [ExecuteProcess(cmd=["python3", "-u", str(HERE / "learned_kick_node.py"), "--ros-args",
                                "-p", f"input_mode:={mode}", "-p", f"fade_in_s:={float(fade)}",
                                "-p", f"end_range:={end_range}"], output="both")]
  raise RuntimeError("kick must be run_through or learned")


def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument("kick", default_value="run_through", description="run_through or learned"),
    DeclareLaunchArgument("input_mode", default_value="manual", description="manual, robot or field"),
    DeclareLaunchArgument("charge_speed", default_value="0.45",
                          description="run-through speed [m/s]; above 0.49 needs tools/make_course_config.py"),
    DeclareLaunchArgument("speed_efficiency", default_value="0.8",
                          description="run-through: achieved / commanded speed (stage B measures it)"),
    DeclareLaunchArgument("fade_in_s", default_value="4.0",
                          description="learned: kick slot init + fade-in [s] (1.0 with the kick config copy)"),
    OpaqueFunction(function=_nodes),
  ])
