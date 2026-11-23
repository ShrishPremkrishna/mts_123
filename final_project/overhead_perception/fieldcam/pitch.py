"""Metric pitch geometry shared by perception, referee and execution.

Markers establish coordinates, playable bounds constrain motion, and surveyed
goal fronts determine aiming/scoring. They need not share the same rectangle.
This record describes selected geometry, not survey accuracy or physical readiness.
"""
from dataclasses import dataclass

from .observations import finite_number

PITCH_VERSION = 2
BOUND_KEYS = ("x_min", "x_max", "y_min", "y_max")
DETECTION_PADDING_M = .25  # Existing camera crop padding, not a motion clearance or accuracy claim.


@dataclass(frozen=True)
class Bounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self):
        for key in BOUND_KEYS:
            finite_number(getattr(self, key), key)
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("pitch bounds must have positive length and width")

    @property
    def length(self):
        return self.x_max - self.x_min

    @property
    def width(self):
        return self.y_max - self.y_min

    def contains(self, x, y, margin=0.):
        return (self.x_min + margin <= x <= self.x_max - margin
                and self.y_min + margin <= y <= self.y_max - margin)

    def record(self):
        return {key: getattr(self, key) for key in BOUND_KEYS}

    def corners(self):
        return [[self.x_min, self.y_min], [self.x_max, self.y_min],
                [self.x_max, self.y_max], [self.x_min, self.y_max]]

    @classmethod
    def read(cls, value):
        if not isinstance(value, dict) or set(value) != set(BOUND_KEYS):
            raise ValueError("pitch bounds require x_min/x_max/y_min/y_max")
        return cls(*(finite_number(value[key], key) for key in BOUND_KEYS))


@dataclass(frozen=True)
class Goal:
    x: float
    y_min: float
    y_max: float

    def __post_init__(self):
        for key in ("x", "y_min", "y_max"):
            finite_number(getattr(self, key), "goal " + key)
        if self.y_min >= self.y_max:
            raise ValueError("goal posts must enclose a positive mouth")

    @property
    def width(self):
        return self.y_max - self.y_min

    @property
    def center(self):
        return self.x, (self.y_min + self.y_max) / 2

    def record(self):
        return {key: getattr(self, key) for key in ("x", "y_min", "y_max")}

    @classmethod
    def read(cls, value):
        if not isinstance(value, dict) or set(value) != {"x", "y_min", "y_max"}:
            raise ValueError("goal front requires x/y_min/y_max")
        return cls(*(finite_number(value[key], "goal " + key) for key in ("x", "y_min", "y_max")))


@dataclass(frozen=True)
class Pitch:
    marker_bounds: Bounds
    playable_bounds: Bounds
    west: Goal
    east: Goal
    ball_radius: float
    goal_width: float

    def __post_init__(self):
        radius = finite_number(self.ball_radius, "ball radius")
        mouth = finite_number(self.goal_width, "nominal goal width")
        bounds = self.playable_bounds
        if radius <= 0 or min(bounds.length, bounds.width, mouth) <= 2 * radius or mouth > bounds.width:
            raise ValueError("pitch and goal mouth must enclose the selected ball")
        if not self.west.x < 0 < self.east.x:
            raise ValueError("goal fronts must lie on opposite sides of the marker origin")
        for goal, end in ((self.west, bounds.x_min), (self.east, bounds.x_max)):
            if abs(goal.x - end) > 1e-6:
                raise ValueError("playable end lines must agree with surveyed goal fronts")
            if (goal.width <= 2 * radius or goal.y_min < bounds.y_min
                    or goal.y_max > bounds.y_max):
                raise ValueError("goal mouth must enclose the ball within the playable sidelines")

    @property
    def goals(self):
        return {"west": self.west.record(), "east": self.east.record()}

    @property
    def ball_search_bounds(self):
        # Scoring needs the centre a radius beyond the front; retain another
        # radius for the whole ball outline used by the detector.
        b, margin = self.playable_bounds, max(DETECTION_PADDING_M, 2 * self.ball_radius)
        return Bounds(b.x_min - margin, b.x_max + margin, b.y_min - margin, b.y_max + margin)

    def record(self):
        return {"version": PITCH_VERSION, "frame": "field_metres",
                "marker_bounds": self.marker_bounds.record(), "playable_bounds": self.playable_bounds.record(),
                "goals": self.goals, "ball_radius": self.ball_radius, "goal_width": self.goal_width}

    @classmethod
    def read(cls, value):
        required = {"version", "frame", "marker_bounds", "playable_bounds", "goals", "ball_radius", "goal_width"}
        if (not isinstance(value, dict) or set(value) != required or type(value["version"]) is not int
                or value["version"] != PITCH_VERSION or value["frame"] != "field_metres"):
            raise ValueError("match pitch requires the complete version 2 field-metres record")
        goals = value["goals"]
        if not isinstance(goals, dict) or set(goals) != {"west", "east"}:
            raise ValueError("match pitch requires both physical goal fronts")
        return cls(Bounds.read(value["marker_bounds"]), Bounds.read(value["playable_bounds"]),
                   Goal.read(goals["west"]), Goal.read(goals["east"]),
                   finite_number(value["ball_radius"], "ball radius"), finite_number(value["goal_width"], "goal width"))

    @classmethod
    def from_field(cls, field, goals=None):
        """Use the marker rectangle and actual goal fronts, retaining the field frame.

        The supplied mouth segments are survey estimates. ``goal_width`` is the
        independently selected physical width; no unmeasured survey tolerance
        is invented here. UI calibration reports their discrepancy separately.
        """
        if not isinstance(field, dict):
            raise ValueError("marker field must be an object")
        length = finite_number(field.get("length"), "marker length")
        width = finite_number(field.get("width"), "marker width")
        mouth = finite_number(field.get("goal_width"), "goal width")
        radius = finite_number(field.get("ball_radius"), "ball radius")
        markers = Bounds(-length / 2, length / 2, -width / 2, width / 2)
        if goals is None:
            west, east = Goal(markers.x_min, -mouth / 2, mouth / 2), Goal(markers.x_max, -mouth / 2, mouth / 2)
        else:
            if not isinstance(goals, dict) or set(goals) != {"west", "east"}:
                raise ValueError("both surveyed goal fronts are required")
            west, east = Goal.read(goals["west"]), Goal.read(goals["east"])
        playable = Bounds(west.x, east.x, markers.y_min, markers.y_max)
        return cls(markers, playable, west, east, radius, mouth)
