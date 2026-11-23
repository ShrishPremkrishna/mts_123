"""Shared types: the only contract between fc modules. SI units, field frame (see SPEC.md)."""
from __future__ import annotations

import json
import math
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
