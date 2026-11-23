"""Small provisional seams shared by gameplay, simulation and future adapters.

Field coordinates are metres, centred on the field; +x is right in the viewer,
+y is up. Yaw is counter-clockwise radians from +x. Commands use the robot's
body frame: vx forward, vy left, wz counter-clockwise radians/second. Times are
seconds on one monotonic simulation clock. Real adapters must reconcile clocks.
Defaults are sandbox assumptions, not measured Pupper operating limits.

Geometry has three distinct parts. ``length``/``width`` are the marker
rectangle that defines coordinates. ``bounds`` are where bodies and the ball
may play. ``own_goal``/``opponent_goal`` are physical goal fronts used for
aiming, defense and scoring. Without a selected ``pitch`` all three coincide
with the nominal centred rectangle and mouth. A selected pitch may put goal
fronts beyond the marker ends and off centre; measured poses are never shifted.
"""
from dataclasses import dataclass, replace
from functools import cached_property
import math

PITCH_KEYS = ({"length", "width"}, {"length", "width", "goal_width"})


@dataclass(frozen=True)
class Rect:
    """Executor view of a playable rectangle (values from the selected Pitch when present)."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def length(self):
        return self.x_max - self.x_min

    @property
    def width(self):
        return self.y_max - self.y_min

    def contains(self, x, y, margin=0.):
        return (self.x_min + margin <= x <= self.x_max - margin
                and self.y_min + margin <= y <= self.y_max - margin)


@dataclass(frozen=True)
class Mouth:
    """Executor view of one physical goal front: plane x and post ordinates."""
    x: float
    y_min: float
    y_max: float

    @property
    def width(self):
        return self.y_max - self.y_min

    @property
    def center(self):
        return self.x, (self.y_min + self.y_max) / 2

    def encloses(self, y, radius):
        return self.y_min + radius <= y <= self.y_max - radius

    def record(self):
        return {"x": self.x, "y_min": self.y_min, "y_max": self.y_max}


@dataclass(frozen=True)
class Field:
    length: float = 3.66
    width: float = 2.44
    goal_width: float = 0.82
    attack_sign: int = 1
    robot_radius: float = 0.16
    ball_radius: float = 0.06
    bumper_offset: float = 0.19
    pitch: object = None  # selected canonical fieldcam.pitch.Pitch; None keeps the nominal rectangle

    def measured_pitch(self, pitch):
        """Apply a measured floor rectangle and optional goal mouth, in metres.

        Robot, ball and bumper dimensions and the attack direction stay enrolled.
        This legacy form describes a centred rectangle; it clears any selected pitch.
        """
        if not isinstance(pitch, dict) or set(pitch) not in PITCH_KEYS:
            raise ValueError("pitch requires length and width, and optionally goal_width, in metres")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in pitch.values()):
            raise ValueError("pitch dimensions must be finite numbers of metres")
        clearance = 2 * max(self.robot_radius, self.ball_radius, self.bumper_offset)
        mouth = pitch.get("goal_width", self.goal_width)
        if min(pitch["length"], pitch["width"]) <= clearance or not 2 * self.ball_radius < mouth <= pitch["width"]:
            raise ValueError("measured pitch is too small for the enrolled robot, ball or goal")
        return replace(self, pitch=None, **{key: float(value) for key, value in pitch.items()})

    def with_pitch(self, pitch):
        """Adopt one complete version-2 pitch record (or Pitch) in the marker frame.

        Marker dimensions replace length/width and the selected nominal mouth
        replaces goal_width. The ball radius must equal the enrolled one: the
        controller's contact geometry and the camera's ball profile must agree.
        Imported here so pinned legacy releases without the shared pitch module
        still load; they cannot select a version-2 pitch.
        """
        try:
            from final_project.overhead_perception.fieldcam.pitch import Pitch
        except ImportError:
            raise ValueError("this release does not stage the shared version-2 pitch geometry") from None
        selected = Pitch.read(pitch.record() if hasattr(pitch, "record") else pitch)
        markers = selected.marker_bounds
        if abs(markers.x_min + markers.x_max) > 1e-9 or abs(markers.y_min + markers.y_max) > 1e-9:
            raise ValueError("pitch marker rectangle must be centred on the field origin")
        if abs(selected.ball_radius - self.ball_radius) > 1e-9:
            raise ValueError("selected pitch ball radius differs from the enrolled ball")
        clearance = 2 * max(self.robot_radius, self.ball_radius, self.bumper_offset)
        if min(selected.playable_bounds.length, selected.playable_bounds.width) <= clearance:
            raise ValueError("selected playable area is too small for the enrolled robot")
        return replace(self, length=markers.length, width=markers.width,
                       goal_width=selected.goal_width, pitch=selected)

    @cached_property
    def bounds(self) -> Rect:
        """Playable rectangle: robots and ball centres must stay within it."""
        if self.pitch is not None:
            b = self.pitch.playable_bounds
            return Rect(b.x_min, b.x_max, b.y_min, b.y_max)
        return Rect(-self.length / 2, self.length / 2, -self.width / 2, self.width / 2)

    def goal(self, sign) -> Mouth:
        """Physical goal front at +x (sign 1) or -x (sign -1)."""
        if type(sign) is not int or sign not in (-1, 1):
            raise ValueError("goal side must be the integer -1 or 1")
        if self.pitch is not None:
            g = self.pitch.goal(sign)
            return Mouth(g.x, g.y_min, g.y_max)
        return Mouth(sign * self.length / 2, -self.goal_width / 2, self.goal_width / 2)

    @property
    def opponent_goal(self) -> Mouth:
        return self.goal(self.attack_sign)

    @property
    def own_goal(self) -> Mouth:
        return self.goal(-self.attack_sign)


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
    kick_active: bool = False  # referee holds an unresolved native kick for this robot


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
    prediction: dict | None = None  # forecast basis (capture, age, model, profile source) when used


def zero(now: float, robot_id: str = "pupper-1") -> Command:
    return Command(expires_at=now + 0.15, robot_id=robot_id)
