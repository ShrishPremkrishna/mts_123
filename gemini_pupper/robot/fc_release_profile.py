"""Pinned choices for the explicit field walking and leg-kick profile."""
from pathlib import Path

MODE = "fc_walk_kick"
WALK_MODEL = "pupper_gait_deploy/policies/default_policy.json"
KICK_DIR = "gemini_pupper/tricks/operator/leg_kick"
WALK_SHA = "ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712"
KICK_SHA = "9c841f2680d0747cb4fd484bd99218e2f661ac5e4257076f0a8027a76e22fc81"
DESCRIPTOR_SHA = "de482d6d0e8373c34c1e9a7bbc31225fc4e755373147fb719e66b81ee0db5ffa"
CONTROLLERS = ("gemini_walk", "trick_leg_kick")
RUNNER = "final_project/behaviors/combined_fc_runner.py"
GUARD = "final_project/behaviors/body_command_guard.py"
GUARD_ARGS = ("--command-contract", "fc_deadline_v1", "--arena-ball-contact")
TELEMETRY = "foundation/hardware/fc-walk-kick-topics.json"
EVENT_TELEMETRY = "foundation/hardware/fc-match-events.json"
# Local imports required by the field runner and the guarded native path.
FIELD_FILES = (
    RUNNER, GUARD, TELEMETRY, EVENT_TELEMETRY, "final_project/behaviors/board_imu.py", "final_project/behaviors/command_deadline.py",
    "final_project/behaviors/operator_drive.py", "final_project/behaviors/command_path_evidence.py", "final_project/behaviors/fc_executor_bridge.py",
    "final_project/behaviors/overhead_ingress.py", "final_project/behaviors/live_field_attack.py",
    "final_project/overhead_perception/fieldcam/__init__.py", "final_project/overhead_perception/fieldcam/net.py",
    "final_project/overhead_perception/fieldcam/observations.py",
    "final_project/overhead_perception/fieldcam/pitch.py",
    *(f"follow_me_pupper/robot/{name}" for name in
      ("viser_camera.py", "fisheye_converter.py", "yolo_seg.py", "perception_safety.py", "onboard_ball.py", "pink_ball.py", "camera_params.yaml", "coco.txt")),
    *(f"final_project/pupper_fc/{name}.py" for name in
      ("__init__", "model", "controller", "paw_approach", "integration", "tactics", "match_director",
       "obstacles", "kick_handoff", "operator_kick_receipts")),
    "gemini_pupper/robot/fc_walk_kick.py", "gemini_pupper/robot/fc_release_profile.py",
    "gemini_pupper/lab/operator_kick.py",
    *(f"gemini_pupper/robot/{name}.py" for name in (
        "controller_coverage", "session_processes", "robot_api", "motion_contract", "runtime_config", "tricks",
        "spawn_controllers", "stop_status", "secret_gate")),
    "foundation/hardware/fc_walk_kick_release.py", "foundation/hardware/stage_app.py",
    "foundation/hardware/software_start.py", "foundation/hardware/fc_session.py", "foundation/hardware/fc_operator.py",
    "final_project/pupper_fc/arena.json",
    "foundation/hardware/persistent_session.py", "foundation/hardware/floor_stock/shutdown_coordinator.py",
)


def select(config, names, follower_mode):
    if tuple(names) != ("leg_kick",) or follower_mode != "disabled":
        raise ValueError("fc_walk_kick requires only leg_kick and disabled motion following")
    walk = config["gemini_walk"]["ros__parameters"]
    walk.update(cmd_vel_contract="fc_deadline_v1", game_buttons=False,
                reference_tricks=False, chord_partner="")
    # Options selects the normal walker; both controllers remain in the Stop set.
    stop = config["estop_controller"]["ros__parameters"]
    stop["controller_names"] = list(CONTROLLERS)
    stop["switch_button_indices"] = [stop["switch_button_indices"][0]]


def verify_profile(manifest, root, config):
    root = Path(root).resolve()
    if (manifest.get("motion_mode") != MODE or manifest.get("skills") != ["leg_kick"]
            or manifest.get("motion_controllers") != list(CONTROLLERS)
            or manifest.get("follower", {}).get("mode") != "disabled"):
        raise ValueError("incomplete combined walker/kick selection")
    for path in (*FIELD_FILES, WALK_MODEL, "foundation/hardware/lab6_session.launch.py",
                 f"{KICK_DIR}/policy.json", f"{KICK_DIR}/trick.yaml"):
        if str((root / path).resolve()) not in manifest["assets"]:
            raise ValueError("missing combined release input: " + path)
    walk = config["gemini_walk"]["ros__parameters"]
    kick = config["trick_leg_kick"]["ros__parameters"]
    manager = config["controller_manager"]["ros__parameters"]
    owner = "neural_controller/NeuralController"
    if manifest.get("fc_imu_recovery", "strict") != "strict":
        raise ValueError("this candidate supports strict IMU fault handling only")
    if manager["gemini_walk"]["type"] != owner:
        raise ValueError("FC actuator owner type differs from the selected board profile")
    if (str(Path(manifest["configuration"]).resolve()) not in manifest["assets"]
            or Path(walk["model_path"]).resolve() != (root / WALK_MODEL).resolve()
            or manifest["assets"].get(str((root / WALK_MODEL).resolve())) != WALK_SHA
            or manifest["assets"].get(str(Path(kick["model_path"]).resolve())) != KICK_SHA
            or Path(kick["model_path"]).resolve() != (root / KICK_DIR / "policy.json").resolve()
            or manifest["assets"][str((root / KICK_DIR / "trick.yaml").resolve())] != DESCRIPTOR_SHA):
        raise ValueError("combined release selected different walking/kick policy or descriptor")
    if (walk.get("cmd_vel_contract") != "fc_deadline_v1" or walk.get("game_buttons") is not False
            or walk.get("reference_tricks") is not False or walk.get("chord_partner") != ""
            or (walk["init_duration"], walk["fade_in_duration"]) != (2.0, 2.0)
            or (kick["init_duration"], kick["fade_in_duration"]) != (.5, 1.)
            or kick.get("cmd_vel_contract", "legacy_twist") != "legacy_twist"
            or config["estop_controller"]["ros__parameters"]["controller_names"] != list(CONTROLLERS)
            or config["estop_controller"]["ros__parameters"]["switch_button_indices"] != [0]):
        raise ValueError("combined release controller settings do not match the selected profile")
