"""Shared, ROS-free startup configuration for the standalone and combined repo layouts."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent


def component_dir(name: str, variable: str) -> Path:
    if os.environ.get(variable):
        return Path(os.environ[variable]).expanduser().resolve()
    sibling = REPO.parent / name
    return sibling if sibling.is_dir() else Path.home() / name


def env_flag(name: str) -> bool:
    value = os.environ.get(name, "0").strip().lower()
    if value not in ("0", "false", "no", "1", "true", "yes"):
        raise ValueError(f"{name} must be 0/1, false/true or no/yes")
    return value in ("1", "true", "yes")


@dataclass(frozen=True)
class FollowerConfig:
    mode: str
    service_dir: Path | None
    always_detect: bool
    use_color: bool
    avoid_obstacles: bool


def follower_mode() -> str:
    """Validate capability selection without requiring physical inference assets."""
    mode = os.environ.get("PUPPER_FOLLOWER_MODE", "lab5").strip().lower()
    if mode not in ("lab5", "disabled", "legacy"):
        raise ValueError("PUPPER_FOLLOWER_MODE must be lab5, disabled or legacy")
    return mode


def follower_config() -> FollowerConfig:
    mode = follower_mode()
    flags = {"always_detect": env_flag("FOLLOW_ALWAYS_DETECT"),
             "use_color": env_flag("FOLLOW_USE_COLOR"),
             "avoid_obstacles": env_flag("FOLLOW_AVOID_OBSTACLES")}
    service = None
    if mode == "lab5":
        repo = component_dir("follow_me_pupper", "FOLLOW_ME_DIR")
        service = next((p for p in (repo / "robot", repo)
                        if (p / "follow_service.py").is_file() and (p / "yolov8n_seg.hef").is_file()), None)
        if service is None:
            raise ValueError(f"Lab 5 follower unavailable in {repo}: requires follow_service.py and yolov8n_seg.hef. "
                             "Set FOLLOW_ME_DIR to the installed Lab 5 repo, or explicitly select "
                             "PUPPER_FOLLOWER_MODE=disabled for operation without following.")
    elif flags["use_color"] or flags["avoid_obstacles"]:
        raise ValueError("FOLLOW_USE_COLOR and FOLLOW_AVOID_OBSTACLES require PUPPER_FOLLOWER_MODE=lab5")
    return FollowerConfig(mode, service, **flags)


SESSION_SCHEMA = "pupper_persistent_session_v1"


def session_skill_names() -> tuple[str, ...] | None:
    """None keeps the ordinary full catalog; explicit empty selects baseline."""
    raw = os.environ.get("PUPPER_SESSION_SKILLS")
    if raw is None:
        return None
    if not raw.strip():
        return ()
    names = [name.strip() for name in raw.split(",")]
    if any(not re.fullmatch(r"[A-Za-z0-9_-]+", name) for name in names):
        raise ValueError("PUPPER_SESSION_SKILLS must be comma-separated installed names or empty for baseline")
    if len(set(names)) != len(names):
        raise ValueError("PUPPER_SESSION_SKILLS contains duplicate names")
    return tuple(sorted(names))


def session_manifest_path() -> Path:
    path = Path(os.environ.get("PUPPER_SESSION_MANIFEST", "/tmp/gemini-pupper/runtime_manifest.json")).expanduser()
    if not path.is_absolute():
        raise ValueError("PUPPER_SESSION_MANIFEST must be an absolute path")
    return path


def session_catalog_identity(tricks: dict) -> dict:
    """Bind the advertised catalog to actual descriptor and model bytes."""
    return {name: {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
                   for path in [trick.path, trick.path.parent / "trick.yaml"]
                   + ([trick.walk_path] if trick.walk_path else [])}
            for name, trick in sorted(tricks.items())}


def validate_session_catalog(tricks: dict) -> dict | None:
    """Real server startup check; never starts ROS or repairs a missing profile.

    The persistent launcher writes the existing runtime-manifest location.
    Ordinary full launch retains its schema3 manifest and unset-selector behavior.
    A scoped manifest cannot silently be consumed by an unscoped server.
    """
    names = session_skill_names()
    path = session_manifest_path()
    if not path.exists():
        if names is None:
            return None
        raise ValueError("selected session manifest missing; prepare the matching persistent profile first")
    try:
        manifest = json.loads(path.read_text())
        if not isinstance(manifest, dict):
            raise ValueError("persistent session manifest must be an object")
        if manifest.get("schema") != SESSION_SCHEMA:
            if names is None:
                return None
            raise ValueError("selected session requires a persistent-profile manifest")
        if names is None:
            raise ValueError("running persistent profile requires explicit PUPPER_SESSION_SKILLS in the server environment")
        if list(names) != manifest["skills"] or sorted(tricks) != list(names):
            raise ValueError("server skill selection differs from persistent launch")
        if session_catalog_identity(tricks) != manifest["catalog"]:
            raise ValueError("selected descriptors or policies differ from persistent launch")
        expected = ["neural_controller_default", "gemini_walk"] + [tricks[name].controller for name in names]
        if manifest["motion_controllers"] != expected:
            raise ValueError("selected controller set differs from persistent launch")
        if not manifest["assets"]:
            raise ValueError("persistent profile has no bound input assets")
        for filename, digest in manifest["assets"].items():
            if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != digest:
                raise ValueError("persistent profile input changed: " + filename)
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid persistent session manifest: " + str(exc)) from exc
    return manifest


def session_motion_bounds(manifest: dict | None = None) -> dict | None:
    """Intersect the selected controller caps with its actual policy limits.

    Native startup supplies the validated manifest. Explicit fake sessions use
    the checked-in baseline and policy, without consulting a physical manifest.
    Unscoped full-catalog operation retains its existing command contract.
    """
    if session_skill_names() is None:
        return None
    if manifest is None:
        config_path = REPO.parent / "foundation/hardware/lab6-session.yaml"
        model_path = component_dir("pupper_gait_deploy", "PUPPER_GAIT_DEPLOY_DIR") / "policies/default_policy.json"
    else:
        config_path = Path(manifest["configuration"])
        model_path = None
    config = yaml.safe_load(config_path.read_text())
    params = config["gemini_walk"]["ros__parameters"]
    policy = json.loads((model_path or Path(params["model_path"])).read_text())

    def number(value, label):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"selected motion {label} must be a finite number")
        return float(value)

    ranges = {}
    for axis in ("vx", "vy", "wz"):
        pair = policy["command_clip"][axis]
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError(f"selected policy command_clip {axis} must have two bounds")
        lo, hi = (number(v, axis) for v in pair)
        if not lo < 0 < hi:
            raise ValueError(f"selected policy command_clip {axis} must span zero")
        ranges[axis] = (lo, hi)
    forward, backward = ranges["vx"][1], -ranges["vx"][0]
    sideways = min(-ranges["vy"][0], ranges["vy"][1])
    yaw = math.degrees(min(-ranges["wz"][0], ranges["wz"][1]))
    forward_cap = number(params.get("max_vx_forward", 0.0), "max_vx_forward")
    backward_cap = number(params.get("max_vx_backward", 0.0), "max_vx_backward")
    if forward_cap > 0:
        forward = min(forward, forward_cap)
    if backward_cap > 0:
        backward = min(backward, backward_cap)
    if policy.get("jump_slot"):
        if params.get("game_buttons", True):
            raise ValueError("selected API profile requires game_buttons false for deterministic command caps")
        cap = number(params.get("speed_cap", 0.0), "speed_cap")
        if cap <= 0:
            cap = number(policy["jump_slot"]["walk_speed_cap"], "jump-slot walk cap")
        if cap <= 0:
            raise ValueError("selected jump-slot walk cap must be positive")
        forward, backward, sideways = (min(v, cap) for v in (forward, backward, sideways))
    return {"forward": forward, "backward": backward, "sideways": sideways, "yaw_deg_s": yaw}
