"""Persistent baseline or explicitly selected Lab 6 skills. Startup homes the robot.

Run only under the reviewed native v5 environment with the operator present.
Motion controllers are configured inactive. A deliberate physical Options press
releases the existing gateway; subsequent Lab 6 API actions reuse this session.
Physical R3 latches Stop. There is no automatic restart or timed trial shutdown.
Root owns explicit session shutdown; API Stop alone requests zero walking speed.
"""
import contextlib
from copy import deepcopy
import hashlib
import importlib.util
import io
import json
import os
import sys
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

ROOT = Path(__file__).resolve().parent
CONFIG_SHA = "d7542febd4fd382d3c3e71e0c5a3fb8ea7a5b140f87ff1cb12ead1982d3854c0"
URDF_SHA = "4858c01f6ce722455b2c9fa554e158bef2215bc1c3e75b2b005fdf9b89aaed44"


STOCK_POLICY_SHA = "ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712"


def prepare_session_profile(config_path, urdf):
    """Prepare files only. Reuse catalog/controller builders; no ROS execution.

    Export PUPPER_SESSION_SKILLS in BOTH launch and server shells: empty is the
    baseline; e.g. sit_stay_shake selects only that installed policy. Explicit
    selection is configuration, never evidence of course/hardware approval.
    PUPPER_GEMINI_DIR identifies the same checkout the server will execute.
    """
    repo = Path(os.environ.get("PUPPER_GEMINI_DIR", str(ROOT.parents[1] / "gemini_pupper"))).expanduser().resolve()
    if not (repo / "robot/tricks.py").is_file():
        raise ValueError("PUPPER_GEMINI_DIR must identify the selected Gemini checkout")
    sys.path.insert(0, str(repo / "robot"))
    import runtime_config
    import tricks as catalog_module
    from controller_coverage import NEURAL_TYPE, combined_course_config
    from ament_index_python.packages import get_package_share_directory

    for module in (runtime_config, catalog_module):
        if Path(module.__file__).resolve().parent != repo / "robot":
            raise ValueError("profile imported a different Gemini checkout; use a fresh launch process")
    names = runtime_config.session_skill_names()
    if names is None:
        raise ValueError("set PUPPER_SESSION_SKILLS explicitly in launch and server: empty selects baseline")
    loaded = catalog_module.load_tricks()
    selected = {name: loaded[name] for name in names}
    if any(t.kind != "policy" or t.walks for t in selected.values()):
        raise ValueError("persistent skill profile accepts policy clips only; reference jumps and walking research pairs require separate review")
    baseline = yaml.safe_load(config_path.read_text())
    models = []
    for name in ("neural_controller_default", "gemini_walk"):
        value = baseline[name]["ros__parameters"]["model_path"]
        prefix = "$(find-pkg-share neural_controller)"
        if not value.startswith(prefix):
            raise ValueError("baseline model must use the pinned installed neural_controller policy")
        model = Path(get_package_share_directory("neural_controller")) / value[len(prefix):].lstrip("/")
        if hashlib.sha256(model.read_bytes()).hexdigest() != STOCK_POLICY_SHA:
            raise ValueError("installed stock policy differs from the demonstrated baseline")
        models.append(model.resolve())
    controllers = [selected[name].controller for name in names]
    combined = combined_course_config(baseline, controllers) if controllers else deepcopy(baseline)
    base = baseline["neural_controller_default"]["ros__parameters"]
    for name, model in zip(("neural_controller_default", "gemini_walk"), models):
        combined[name]["ros__parameters"]["model_path"] = str(model)
    for name in names:
        trick = selected[name]
        combined["controller_manager"]["ros__parameters"][trick.controller] = {"type": NEURAL_TYPE}
        combined[trick.controller] = {"ros__parameters": catalog_module.controller_params(trick, base)}

    validator_path = runtime_config.component_dir("pupper_gait_deploy", "PUPPER_GAIT_DEPLOY_DIR") / "download_policy.py"
    spec = importlib.util.spec_from_file_location("persistent_policy_validator", validator_path)
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    with contextlib.redirect_stdout(io.StringIO()):
        for model in dict.fromkeys([*models, *(t.path for t in selected.values())]):
            validator.validate(model)

    manifest_path = runtime_config.session_manifest_path()
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    config_out = manifest_path.parent / "persistent_controllers.yaml"
    if config_out.resolve() == config_path.resolve():
        raise ValueError("generated profile must not overwrite the pinned baseline")
    class NoAliasDumper(yaml.SafeDumper):
        def ignore_aliases(self, data):
            return True
    config_out.write_text(yaml.dump(combined, Dumper=NoAliasDumper, sort_keys=False))
    assets = [config_path, urdf, Path(__file__), config_out, *models,
              Path(runtime_config.__file__), Path(catalog_module.__file__), validator_path,
              *(repo / "robot" / name for name in ("robot_api.py", "server.py", "tool_registry.py")),
              *sorted((repo / "lab").glob("*.py")), repo / "lab/prompt.md"]
    manifest = {"schema": runtime_config.SESSION_SCHEMA, "skills": list(names),
                "motion_controllers": ["neural_controller_default", "gemini_walk", *controllers],
                "catalog": runtime_config.session_catalog_identity(selected),
                "assets": {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in assets},
                "configuration": str(config_out),
                "hardware_approval": "Not supplied by this profile; selected policies still require applicable approval and supervised readiness."}
    temporary = manifest_path.with_suffix(manifest_path.suffix + ".tmp")
    temporary.write_text(json.dumps(manifest, indent=2) + "\n")
    temporary.replace(manifest_path)
    return config_out, manifest["motion_controllers"]


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "168":
        raise RuntimeError("Lab 6 session requires explicit ROS_DOMAIN_ID=168")
    config_path = ROOT / "lab6-session.yaml"
    urdf = ROOT / "floor_stock" / "baseline-expanded.urdf"
    for path, expected in ((config_path, CONFIG_SHA), (urdf, URDF_SHA)):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError("Selected Lab 6 input changed: " + str(path))
    generated_config, motion_controllers = prepare_session_profile(config_path, urdf)
    config = ParameterFile(str(generated_config), allow_substs=True)
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
        arguments=[*motion_controllers, "--inactive", "--param-file", str(generated_config), *common], output="screen")
    return LaunchDescription([*guards, *required, feedback, motion])
