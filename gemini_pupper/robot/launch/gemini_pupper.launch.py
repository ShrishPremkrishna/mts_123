"""Robot stack for Gemini Pupper.

Includes the course's neural_controller launch (walking policies, camera, joystick
e-stop) exactly as pupper_gait_deploy installs it, and adds:
  - Lab 5's fixed white balance on the course's camera node (camera_white_balance)
  - cmd_vel_mux, so the joystick, Gemini and the person follower share /cmd_vel
  - the explicitly selected person follower: Lab 5 by default, requiring its
    service and detector model; legacy or disabled mode must be selected explicitly
  - gemini_walk, the walking controller Gemini drives (default policy, running
    allowed, reference tricks such as the jump enabled), and one controller per
    policy trick in <repo>/tricks (see robot/tricks.py). Every motion Gemini asks
    for is one of these policies; there is no open-loop trick player.
The joystick still drives the robot, now through the mux, where it outranks Gemini.

    ros2 launch robot/launch/gemini_pupper.launch.py
"""
import os
import sys
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HERE = Path(__file__).resolve().parent
PARAMS = str(HERE / "gemini_pupper.yaml")
sys.path.insert(0, str(HERE.parent))
from spawn_controllers import PARAM_FILE as TRICK_PARAMS  # noqa: E402
from tricks import (WALK_CONTROLLER, controller_params, load_tricks, walk_controller_params,
                    walk_policy_params, validate_locomotion_contract)  # noqa: E402
from runtime_config import follower_config  # noqa: E402

COURSE_CONFIG = PathJoinSubstitution([FindPackageShare("neural_controller"), "launch", "config.yaml"])


def gemini_controllers():
    """Write one parameter file for the controllers Gemini adds, and spawn them inactive:
    gemini_walk (the course default policy, able to run and play reference tricks)
    and one NeuralController per policy trick. All start from the course default
    controller's settings; the course's own controllers are left as they are."""
    from ament_index_python.packages import get_package_share_directory

    course = Path(get_package_share_directory("neural_controller")) / "launch" / "config.yaml"
    base = yaml.safe_load(course.read_text())["neural_controller_default"]["ros__parameters"]
    kind = {"type": "neural_controller/NeuralController"}
    params = {WALK_CONTROLLER: {"ros__parameters": {**kind, **walk_controller_params(base)}}}
    tricks = [t for t in load_tricks().values() if t.kind == "policy"]
    for t in tricks:
        params[t.controller] = {"ros__parameters": {**kind, **controller_params(t, base)}}
        if t.walks:  # a trick that walks in its own stance: its walking policy too
            params[t.walk_controller] = {"ros__parameters": {**kind, **walk_policy_params(t, base)}}
    # The course config writes paths as $(find-pkg-share pkg)/...; the spawner's
    # parameter file isn't substituted, so expand them here.
    for entry in params.values():
        p = entry["ros__parameters"]
        path = p.get("model_path", "")
        while "$(find-pkg-share " in path:
            start = path.index("$(find-pkg-share ")
            end = path.index(")", start)
            pkg = path[start + len("$(find-pkg-share "):end].strip()
            path = path[:start] + get_package_share_directory(pkg) + path[end + 1:]
        p["model_path"] = path
    validate_locomotion_contract(Path(params[WALK_CONTROLLER]["ros__parameters"]["model_path"]),
                                 [t for t in load_tricks().values() if t.kind == "reference"])
    TRICK_PARAMS.parent.mkdir(parents=True, exist_ok=True)
    # ROS parameter files reject YAML aliases, which safe_dump emits for the lists
    # the controllers share with the course config.
    class NoAliasDumper(yaml.SafeDumper):
        def ignore_aliases(self, data):
            return True

    TRICK_PARAMS.write_text(yaml.dump(params, Dumper=NoAliasDumper, sort_keys=False))
    # After the course's own controllers, and checked one by one: see spawn_controllers.py.
    spawn = HERE.parent / "spawn_controllers.py"
    return [ExecuteProcess(cmd=["python3", "-u", str(spawn), str(TRICK_PARAMS), *params], output="both")]


def camera_white_balance():
    """Lab 5's fixed white balance (robot/camera_params.yaml) on the course's camera node.

    With auto white balance this camera comes out pink indoors. The course launch
    starts the camera node with its own parameters, so the gains are set once the
    node is up, the same libcamera controls follow_me.launch.py starts it with.
    """
    wb = yaml.safe_load((HERE.parent / "camera_params.yaml").read_text()).get("white_balance", {})
    if wb.get("awb_enable", True):
        return []
    red, blue = (float(g) for g in wb["colour_gains"])
    script = ("until ros2 param set /camera AwbEnable false; do sleep 2; done; "
              f"ros2 param set /camera ColourGains '[{red}, {blue}]'")
    return [ExecuteProcess(cmd=["bash", "-c", script], output="both")]


def person_follower():
    """The follower behind activate/deactivate_person_following.

    The Lab 5 service/model resolve through FOLLOW_ME_DIR, a sibling repo or
    ~/follow_me_pupper. FOLLOW_ALWAYS_DETECT keeps detection on between follows.
    FOLLOW_USE_COLOR and FOLLOW_AVOID_OBSTACLES explicitly enable the optional
    experiments. Missing dependencies fail startup; selecting legacy or disabled
    mode is an explicit choice through PUPPER_FOLLOWER_MODE.
    """
    config = follower_config()  # fail visibly if the intended stack/model is absent
    service = config.service_dir
    if config.mode == "disabled":
        print("[gemini_pupper] following explicitly disabled; no detector/follower started")
        return []
    if config.mode == "lab5":
        print(f"[gemini_pupper] person follower: {service}/follow_service.py"
              f"; color recovery={config.use_color}; obstacle detours={config.avoid_obstacles}"
              + (" (always detecting, for the eyes)" if config.always_detect else ""))
        return [ExecuteProcess(
            cmd=["python3", "-u", "follow_service.py", "--ros-args", "-r", "__node:=person_follower_node",
                 "-p", f"always_detect:={str(config.always_detect).lower()}",
                 "-p", f"use_color:={str(config.use_color).lower()}",
                 "-p", f"avoid_obstacles:={str(config.avoid_obstacles).lower()}"],
            cwd=str(service), output="both", respawn=True, respawn_delay=3.0)]
    print("[gemini_pupper] explicit legacy follower: no Lab 5 target lock, stopping distance, color recovery or detours")
    return [
        Node(package="hailo", executable="hailo_detection", output="both", respawn=True, respawn_delay=3.0),
        Node(package="person_follower", executable="person_follower_node", output="both"),
    ]


def generate_launch_description():
    course = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("neural_controller"), "launch", "launch.py"])),
        launch_arguments={"teleop": "False"}.items(),  # re-added below, routed through the mux
    )
    return LaunchDescription([
        course,
        Node(package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy_node",
             parameters=[COURSE_CONFIG], remappings=[("cmd_vel", "teleop_cmd_vel")], output="both"),
        Node(package="cmd_vel_mux", executable="cmd_vel_mux_node", parameters=[PARAMS], output="both"),
        Node(package="topic_tools", executable="throttle", name="joint_state_throttler",
             parameters=[PARAMS], arguments=["messages"], output="both"),
        *camera_white_balance(),
        *person_follower(),
        *gemini_controllers(),
    ])
