"""Small provisional seams shared by gameplay, simulation and future adapters.

Field coordinates are metres, centred on the field; +x is right in the viewer,
+y is up. Yaw is counter-clockwise radians from +x. Commands use the robot's
body frame: vx forward, vy left, wz counter-clockwise radians/second. Times are
seconds on one monotonic simulation clock. Real adapters must reconcile clocks.
Defaults are sandbox assumptions, not measured Pupper operating limits.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Field:
    length: float = 3.66
    width: float = 2.44
    goal_width: float = 0.82
    attack_sign: int = 1
    robot_radius: float = 0.16
    ball_radius: float = 0.06
    bumper_offset: float = 0.19


@dataclass(frozen=True)
class Limits:
    vx: float = 0.65
    vy: float = 0.35
    wz: float = 1.8
    acceleration: float = 1.0
    yaw_acceleration: float = 3.5


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


@dataclass(frozen=True)
class Velocity:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class Observation:
    captured_at: float
    robot: Pose | None
    ball: Ball | None
    robot_id: str = "pupper-1"
    fallen: bool = False
    goal: int = 0  # +1 attacking goal, -1 own goal, 0 none; referee event.


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    expires_at: float = 0.0
    robot_id: str = "pupper-1"


@dataclass(frozen=True)
class Decision:
    phase: str
    reason: str
    command: Command
    target: tuple[float, float] | None = None
    target_heading: float | None = None
    position_error: float | None = None
    heading_error: float | None = None
    recovery_requested: bool = False


def zero(now: float, robot_id: str = "pupper-1") -> Command:
    return Command(expires_at=now + 0.15, robot_id=robot_id)
