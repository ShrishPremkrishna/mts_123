"""Shared types: the only contract between fc modules. SI units, field frame (see SPEC.md)."""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

CONFIG = Path(__file__).parent / "config"


def wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Ball:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    t: float = 0.0  # capture time


@dataclass(frozen=True)
class RobotObs:
    id: str
    pose: Pose
    t: float  # capture time


@dataclass(frozen=True)
class World:
    t: float  # time this snapshot was assembled
    ball: Ball | None
    robots: dict[str, RobotObs] = field(default_factory=dict)


@dataclass(frozen=True)
class Cmd:
    vx: float = 0.0  # body frame, m/s
    vy: float = 0.0
    wz: float = 0.0  # rad/s
    tag: str = ""

    @staticmethod
    def stop(tag: str = "stop") -> "Cmd":
        return Cmd(0.0, 0.0, 0.0, tag)


@dataclass(frozen=True)
class Field:
    length: float       # goal line to goal line, m
    width: float        # sideline to sideline, m
    goal_width: float   # mouth width, m
    goal_depth: float   # net depth behind the goal line, m
    ball_radius: float
    robot_radius: float  # footprint circle used for clearance

    def goal_center(self, team: str, own: bool) -> tuple[float, float]:
        """Centre of a goal mouth. Blue defends west (x<0)."""
        west = (team == "blue") == own
        return (-self.length / 2 if west else self.length / 2, 0.0)


@dataclass(frozen=True)
class RobotCfg:
    id: str
    team: str          # "blue" | "red"
    tag: int           # overhead AprilTag id
    host: str          # ssh host / IP of the Pi ("" = sim only)
    profile: str       # robot software profile name
    udp_host: str = ""  # LAN address for control packets ("" = resolve ssh host)


def load_field(path: Path = CONFIG / "field.json") -> Field:
    return Field(**json.loads(path.read_text()))


def load_params(path: Path = CONFIG / "params.json") -> dict:
    return json.loads(path.read_text())


def load_robots(path: Path = CONFIG / "robots.json") -> list[RobotCfg]:
    return [RobotCfg(**r) for r in json.loads(path.read_text())]


# ---- per-robot overrides (calibration of one physical robot) -------------------------------------
ROBOTS_DIR = CONFIG / "robots"
ROBOT_ID_RE = re.compile(r"^[A-Za-z0-9_.-]{1,32}$")   # same rule as robot/stage.sh


def _kind(v) -> str:
    if isinstance(v, bool):
        return "bool"
    if isinstance(v, (int, float)):
        return "number"
    return type(v).__name__


def load_robot_overrides(robot_id: str, root: Path = ROBOTS_DIR, known: dict | None = None) -> dict:
    """Keys from `<root>/<robot_id>.json` that override the shared params for that one robot.

    No file -> {} (a robot without calibration runs on the shared params). Keys starting with "_"
    are notes (e.g. `_note`, `_measured`) and are dropped. With `known` (the params dict the
    overrides apply to), every key must already exist there with the same kind of value
    (number / str / bool), so a typo or a wrong type fails loudly instead of being ignored.
    Raises ValueError with the file and key on any problem."""
    if not ROBOT_ID_RE.match(robot_id or ""):
        raise ValueError(f"bad robot id {robot_id!r}")
    path = Path(root) / f"{robot_id}.json"
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError as e:
        raise ValueError(f"{path}: not valid JSON ({e})") from e
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a JSON object")
    out = {k: v for k, v in data.items() if not k.startswith("_")}
    if known is not None:
        for k, v in out.items():
            if k not in known:
                raise ValueError(f"{path}: unknown param {k!r}")
            if _kind(v) != _kind(known[k]):
                raise ValueError(f"{path}: {k!r} is {_kind(v)}, expected {_kind(known[k])}")
    return out


def robot_params(base: dict, robot_id: str, root: Path = ROBOTS_DIR) -> dict:
    """The params one robot runs with: `base` (shared params, presets applied) + its overrides.
    Overrides describe the hardware, so they win over a preset. `base` is not modified."""
    return {**base, **load_robot_overrides(robot_id, root, known=base)}
