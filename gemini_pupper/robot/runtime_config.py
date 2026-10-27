"""Shared, ROS-free startup configuration for the standalone and combined repo layouts."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

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


def follower_config() -> FollowerConfig:
    mode = os.environ.get("PUPPER_FOLLOWER_MODE", "lab5").strip().lower()
    if mode not in ("lab5", "disabled", "legacy"):
        raise ValueError("PUPPER_FOLLOWER_MODE must be lab5, disabled or legacy")
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
