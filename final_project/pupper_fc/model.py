"""Small provisional seams shared by gameplay, simulation and future adapters.

Field coordinates are metres, centred on the field; +x is right in the viewer,
+y is up. Yaw is counter-clockwise radians from +x. Commands use the robot's
body frame: vx forward, vy left, wz counter-clockwise radians/second. Times are
seconds on one monotonic simulation clock. Real adapters must reconcile clocks.
Defaults are sandbox assumptions, not measured Pupper operating limits.
"""
from dataclasses import dataclass, replace
import math


@dataclass(frozen=True)
class Field:
    length: float = 3.66
    width: float = 2.44
    goal_width: float = 0.82
    attack_sign: int = 1
    robot_radius: float = 0.16
    ball_radius: float = 0.06
    bumper_offset: float = 0.19


    def measured_pitch(self, pitch):
        """Apply only the measured floor rectangle. Hardware dimensions stay enrolled. A measured goal mouth may be included."""
        if not isinstance(pitch, dict) or set(pitch) not in ({"length", "width"}, {"length", "width", "goal_width"}):
            raise ValueError("Pitch requires a length and width in metres")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in pitch.values()):
            raise ValueError("Pitch dimensions must be finite metres")
        clearance = max(self.robot_radius, self.ball_radius, self.bumper_offset) * 2
        mouth = pitch.get("goal_width", self.goal_width)
        if min(pitch["length"], pitch["width"]) <= clearance or not 2*self.ball_radius < mouth <= pitch["width"]:
            raise ValueError("Measured pitch is too small for the enrolled robot or goal")
        return replace(self, **pitch)


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
    role: str | None = None  # current autonomous role; explicit actions leave it unset


def zero(now: float, robot_id: str = "pupper-1") -> Command:
    return Command(expires_at=now + 0.15, robot_id=robot_id)
