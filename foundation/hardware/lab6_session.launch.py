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

ROOT = Path(__file__).resolve().parent
CONFIG_SHA = "d7542febd4fd382d3c3e71e0c5a3fb8ea7a5b140f87ff1cb12ead1982d3854c0"
URDF_SHA = "4858c01f6ce722455b2c9fa554e158bef2215bc1c3e75b2b005fdf9b89aaed44"


FULL_FEEDBACK = ("joint_state_broadcaster", "imu_sensor_broadcaster")
STOCK_POLICY_SHA = "ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712"


def selected_robot_description(urdf, directory, motion_mode):
    recovery = os.environ.get("PUPPER_FC_IMU_RECOVERY", "strict")
    if recovery == "strict":
        return urdf, recovery
    if motion_mode != "fc_walk_kick" or recovery not in {"supported_rehome", "supported_standby"}:
        raise ValueError("unknown or non-FC PUPPER_FC_IMU_RECOVERY selection")
    source = urdf.read_text()
    plugin = "<plugin>control_board_hardware_interface/ControlBoardHardwareInterface</plugin>"
    if source.count(plugin) != 1 or 'name="fc_imu_recovery"' in source:
        raise ValueError("unexpected control board description for FC IMU recovery")
    derived = directory / f"fc-{recovery}.urdf"
    temporary = derived.with_suffix(".urdf.tmp")
    temporary.write_text(source.replace(plugin, plugin + f'\n      <param name="fc_imu_recovery">{recovery}</param>'))
    temporary.replace(derived)
    return derived, recovery


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

    for module in (runtime_config, catalog_module):
        if Path(module.__file__).resolve().parent != repo / "robot":
            raise ValueError("profile imported a different Gemini checkout; use a fresh launch process")
    names = runtime_config.session_skill_names()
    if names is None:
        raise ValueError("set PUPPER_SESSION_SKILLS explicitly in launch and server: empty selects baseline")
    loaded = catalog_module.load_tricks()
    selected = {name: loaded[name] for name in names}
    if any(t.kind != "policy" for t in selected.values()):
        raise ValueError("persistent skill profile accepts policy skills only; reference jumps require separate review")
    if "PUPPER_FOLLOWER_MODE" not in os.environ:
        raise ValueError("set PUPPER_FOLLOWER_MODE explicitly to disabled or lab5")
    follower = runtime_config.follower_config()
    motion_mode = runtime_config.session_motion_mode()
    if follower.mode == "legacy":
        raise ValueError("persistent profile supports the Lab 5 follower or disabled mode, not legacy")
    baseline = yaml.safe_load(config_path.read_text())
    if motion_mode == "fc_walk_kick":
        import fc_release_profile
    models = []
    for name in ("neural_controller_default", "gemini_walk"):
        value = baseline[name]["ros__parameters"]["model_path"]
        prefix = "$(find-pkg-share neural_controller)"
        if not value.startswith(prefix):
            raise ValueError("baseline model must use the pinned installed neural_controller policy")
        if motion_mode == "fc_walk_kick":
            model = repo.parent / fc_release_profile.WALK_MODEL
        else:
            from ament_index_python.packages import get_package_share_directory
            model = Path(get_package_share_directory("neural_controller")) / value[len(prefix):].lstrip("/")
        if hashlib.sha256(model.read_bytes()).hexdigest() != STOCK_POLICY_SHA:
            raise ValueError("installed stock policy differs from the demonstrated baseline")
        models.append(model.resolve())
    controllers = runtime_config.session_skill_controllers(selected)
    combined = combined_course_config(baseline, controllers) if controllers else deepcopy(baseline)
    base = baseline["neural_controller_default"]["ros__parameters"]
    for name, model in zip(("neural_controller_default", "gemini_walk"), models):
        combined[name]["ros__parameters"]["model_path"] = str(model)
    if motion_mode == "class_run":
        # Reuse the course API controller settings with this same pinned policy.
        # This opt-in profile is a candidate, not observed running capability.
        combined["gemini_walk"]["ros__parameters"] = catalog_module.walk_controller_params(base)
        combined["gemini_walk"]["ros__parameters"]["model_path"] = str(models[1])
    for name in names:
        trick = selected[name]
        combined["controller_manager"]["ros__parameters"][trick.controller] = {"type": NEURAL_TYPE}
        combined[trick.controller] = {"ros__parameters": catalog_module.controller_params(trick, base)}
        if trick.walks:
            combined["controller_manager"]["ros__parameters"][trick.walk_controller] = {"type": NEURAL_TYPE}
            combined[trick.walk_controller] = {"ros__parameters": catalog_module.walk_policy_params(trick, base)}

    motion_controllers = ["neural_controller_default", "gemini_walk", *controllers]
    feedback_broadcasters = FULL_FEEDBACK
    if motion_mode == "fc_walk_kick":
        fc_release_profile.select(combined, names, follower.mode)
        motion_controllers = list(fc_release_profile.CONTROLLERS)
        if os.environ.get("PUPPER_FC_RECORDING") == "events":
            # Native controllers read joint handles directly. The field uses overhead poses.
            feedback_broadcasters = ("imu_sensor_broadcaster",)
        plugins = (*motion_controllers, *feedback_broadcasters)
        used_nodes = ("controller_manager", "estop_controller", "floor_physical_joy", *plugins)
        combined = {name: combined[name] for name in used_nodes}
        manager = combined["controller_manager"]["ros__parameters"]
        for name, value in list(manager.items()):
            if isinstance(value, dict) and "type" in value and name not in plugins:
                del manager[name]

    validator_path = runtime_config.component_dir("pupper_gait_deploy", "PUPPER_GAIT_DEPLOY_DIR") / "download_policy.py"
    spec = importlib.util.spec_from_file_location("persistent_policy_validator", validator_path)
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    with contextlib.redirect_stdout(io.StringIO()):
        for model in dict.fromkeys([*models, *(t.path for t in selected.values()),
                                    *(t.walk_path for t in selected.values() if t.walks)]):
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
    description_path, imu_recovery = selected_robot_description(urdf, manifest_path.parent, motion_mode)
    assets = [config_path, urdf, description_path, Path(__file__), config_out, *models,
              Path(runtime_config.__file__), Path(catalog_module.__file__), validator_path]
    if motion_mode == "fc_walk_kick":
        assets.extend(repo.parent / p for p in (*fc_release_profile.FIELD_FILES, fc_release_profile.WALK_MODEL))
        for trick in selected.values():
            assets.extend((trick.path, trick.path.parent / "trick.yaml"))
    else:
        assets.extend(repo / "robot" / name for name in ("robot_api.py", "server.py", "tool_registry.py"))
        assets.extend(sorted((repo / "lab").glob("*.py")))
        assets.append(repo / "lab/prompt.md")
    if follower.service_dir is not None:
        service = follower.service_dir
        required_follower = [service / name for name in (
            "follow_service.py", "fisheye_converter.py", "perception_safety.py", "yolo_seg.py",
            "coco.txt", "camera_params.yaml", "yolov8n_seg.hef")]
        required_follower += [service.parent / "lab" / name for name in (
            "follow_me.py", "boxes.py", "target_lock.py", "avoidance.py")]
        missing = [str(path) for path in required_follower if not path.is_file()]
        if missing:
            raise ValueError("Lab 5 follower runtime dependencies missing: " + ", ".join(missing))
        assets.extend(required_follower)
        # Pin any further local support modules present in the installed package.
        assets.extend(sorted(service.glob("*.py")))
        assets.extend(sorted((service.parent / "lab").glob("*.py")))
    manifest = {"schema": runtime_config.SESSION_SCHEMA, "skills": list(names),
                "feedback_broadcasters": feedback_broadcasters,
                "follower": runtime_config.follower_identity(follower),
                "motion_mode": motion_mode,
                "motion_controllers": motion_controllers,
                "catalog": runtime_config.session_catalog_identity(selected),
                "assets": {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in assets},
                "configuration": str(config_out),
                "hardware_approval": "Not supplied by this profile; selected policies still require applicable approval and supervised readiness."}
    if motion_mode == "fc_walk_kick":
        manifest["fc_imu_recovery"] = imu_recovery
        manifest["robot_description"] = str(description_path.resolve())
        fc_release_profile.verify_profile(manifest, repo.parent, combined)
    temporary = manifest_path.with_suffix(manifest_path.suffix + ".tmp")
    temporary.write_text(json.dumps(manifest, indent=2) + "\n")
    temporary.replace(manifest_path)
    return config_out, manifest["motion_controllers"]


def launch_profile(config_path, urdf):
    """Home consumes the exact files pinned by Prepare. Direct launches still prepare their own."""
    digest = os.environ.get("PUPPER_FC_PREPARED_MANIFEST_SHA256")
    if digest is None:
        prepare_session_profile(config_path, urdf)
        import runtime_config
        return json.loads(runtime_config.session_manifest_path().read_text())

    repo = Path(os.environ["PUPPER_GEMINI_DIR"]).resolve()
    sys.path.insert(0, str(repo / "robot"))
    import runtime_config
    from tricks import load_tricks

    manifest_path = runtime_config.session_manifest_path()
    if (runtime_config.REPO != repo or runtime_config.session_motion_mode() != "fc_walk_kick"
            or hashlib.sha256(manifest_path.read_bytes()).hexdigest() != digest):
        raise ValueError("Home requires the FC manifest pinned by Prepare")
    manifest = runtime_config.validate_session_catalog(load_tricks())
    if (not manifest or manifest["configuration"] != str(manifest_path.parent / "persistent_controllers.yaml")
            or manifest.get("fc_imu_recovery") != os.environ.get("PUPPER_FC_IMU_RECOVERY", "strict")):
        raise ValueError("prepared FC configuration or board recovery selection differs")
    return manifest


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterFile
    if os.environ.get("ROS_DOMAIN_ID") != "168":
        raise RuntimeError("Lab 6 session requires explicit ROS_DOMAIN_ID=168")
    config_path = ROOT / "lab6-session.yaml"
    urdf = ROOT / "floor_stock" / "baseline-expanded.urdf"
    for path, expected in ((config_path, CONFIG_SHA), (urdf, URDF_SHA)):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError("Selected Lab 6 input changed: " + str(path))
    manifest = launch_profile(config_path, urdf)
    generated_config, motion_controllers = manifest["configuration"], manifest["motion_controllers"]
    config = ParameterFile(str(generated_config), allow_substs=True)
    import runtime_config
    description = {"robot_description": Path(manifest.get("robot_description", urdf)).read_text()}
    state = Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[description], output="screen")
    manager = Node(package="controller_manager", executable="ros2_control_node",
                   parameters=[config, description], output="screen",
                   remappings=[("controller_manager:/controller_manager/switch_controller",
                                "/controller_manager/switch_controller_backend")])
    gateway = Node(package="joy_utils", executable="estop_controller",
                   parameters=[config], output="screen")
    joy = Node(package="joy", executable="joy_node", name="floor_physical_joy",
               namespace="/", parameters=[config], output="screen")
    # A silent mux can otherwise leave a controller holding its last velocity.
    # Any required process exit tears down this owned session. No respawn.
    # Camera is owned once by the maintained session starter. This service only
    # subscribes to /camera/image_raw/compressed and never opens the sensor.
    follower = runtime_config.follower_config()
    follower_actions = []
    if follower.mode == "lab5":
        follower_actions.append(ExecuteProcess(
            cmd=["python3", "-u", "follow_service.py", "--ros-args", "-r", "__node:=person_follower_node",
                 "-p", f"always_detect:={str(follower.always_detect).lower()}",
                 "-p", f"use_color:={str(follower.use_color).lower()}",
                 "-p", f"avoid_obstacles:={str(follower.avoid_obstacles).lower()}"],
            cwd=str(follower.service_dir), output="screen"))
    extra = []
    if runtime_config.session_motion_mode() == "fc_walk_kick":
        from fc_release_profile import GUARD, GUARD_ARGS
        root = runtime_config.REPO.parent
        guard = ExecuteProcess(cmd=[sys.executable, "-B", "-s", str(root / GUARD), *GUARD_ARGS],
                               cwd=str(root), output="screen", respawn=False)
        required = (state, manager, gateway, joy)
        # The native walk deadline zeros the request if the guard exits.
        # Keep the manager and physical Stop alive during a policy transfer.
        extra.append(guard)
    else:
        mux = Node(package="cmd_vel_mux", executable="cmd_vel_mux_node",
                   parameters=[config], output="screen")
        required = (state, manager, gateway, mux, joy, *follower_actions)
    guards = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason="required Lab 6 session process exited"))]))
        for node in required]
    common = ["--controller-manager", "/controller_manager",
              "--controller-manager-timeout", "60", "--service-call-timeout", "10"]
    feedback = Node(package="controller_manager", executable="spawner",
        arguments=[*manifest.get("feedback_broadcasters", FULL_FEEDBACK), *common], output="screen")
    motion = Node(package="controller_manager", executable="spawner",
        arguments=[*motion_controllers, "--inactive", "--param-file", str(generated_config), *common], output="screen")
    return LaunchDescription([*guards, *required, feedback, motion, *extra])
