"""Small per-robot role selection and reactive goal guarding, with no I/O.

Inputs use model.py's field frame and one caller-supplied monotonic clock.
This is a positional baseline, not interception prediction or localization.
Create a separate Tactics instance for each opponent. A hold preserves its role
but supplies no action target; the caller must stop/cancel current execution.
"""
from dataclasses import dataclass
import math

from .model import Ball, Field, Observation, Pose
from .controller import defensive_clearance


@dataclass(frozen=True)
class TacticsConfig:
    # Fractions of field length, measured from this robot's own goal line.
    enter_defense: float = 0.30
    exit_defense: float = 0.45
    emergency_defense: float = 0.14
    minimum_dwell: float = 1.0
    max_observation_age: float = 0.35
    guard_depth: float = 0.35  # metres inside own goal line
    boundary_clearance: float = 0.04  # beyond robot radius, metres


@dataclass(frozen=True)
class TacticalDecision:
    role: str
    action: str
    target: Pose | None
    reason: str


def _finite(*values):
    try:
        return all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in values)
    except OverflowError:
        return False


class Tactics:
    """tick(observation, now) returns attack, defend(target), or hold.

    Defense enters near our own goal and exits farther away. Minimum dwell
    applies to ordinary changes; an emergency can enter defense immediately.
    The first valid observation chooses a role without an artificial dwell.
    Config defaults are provisional decisions, not hardware operating limits.
    Clock rollback holds until the previous time is reached; create a new
    instance for a new clock/session. No past motion is replayed by this class.
    """

    def __init__(self, field: Field, robot_id: str, config=TacticsConfig()):
        if not isinstance(field, Field) or not isinstance(config, TacticsConfig):
            raise ValueError("field and config must be Field and TacticsConfig")
        if not isinstance(robot_id, str) or not robot_id.strip():
            raise ValueError("robot_id must be nonempty")
        if (type(field.attack_sign) is not int or field.attack_sign not in (-1, 1)
                or not _finite(field.length, field.width, field.goal_width,
                               field.robot_radius, field.ball_radius, field.bumper_offset)
                or min(field.length, field.width, field.goal_width, field.robot_radius, field.bumper_offset) <= 0
                or not 0 <= field.ball_radius < min(field.length, field.width) / 2
                or field.goal_width > field.width):
            raise ValueError("field geometry must be finite and physically bounded")
        if (not _finite(*vars(config).values())
                or not 0 <= config.emergency_defense < config.enter_defense
                < config.exit_defense < 1
                or config.minimum_dwell < 0 or config.max_observation_age <= 0
                or config.guard_depth < 0 or config.boundary_clearance < 0
                or field.robot_radius + config.boundary_clearance
                >= min(field.length, field.width) / 2):
            raise ValueError("invalid tactic thresholds, timing or clearance")
        self.field, self.robot_id, self.config = field, robot_id, config
        self.role = "offense"
        self._changed_at = self._last_now = self._last_capture = None

    def _hold(self, reason):
        return TacticalDecision(self.role, "hold", None, reason)

    def tick(self, observation: Observation, now: float) -> TacticalDecision:
        if not _finite(now):
            return self._hold("invalid control clock")
        if self._last_now is not None and now < self._last_now:
            return self._hold("control clock moved backwards")
        self._last_now = now
        if not isinstance(observation, Observation) or observation.robot_id != self.robot_id:
            return self._hold("missing observation or wrong robot identity")
        stamp = observation.captured_at
        if not _finite(stamp) or not 0 <= now - stamp <= self.config.max_observation_age:
            return self._hold("observation is stale, future or has an invalid timestamp")
        if self._last_capture is not None and stamp < self._last_capture:
            return self._hold("observation capture moved backwards")
        self._last_capture = stamp
        if type(observation.fallen) is not bool or observation.fallen:
            return self._hold("robot is fallen or fall state is invalid")
        robot, ball = observation.robot, observation.ball
        if not isinstance(robot, Pose) or not _finite(robot.x, robot.y, robot.yaw):
            return self._hold("robot pose is missing or invalid")
        if not isinstance(ball, Ball) or not _finite(ball.x, ball.y, ball.vx, ball.vy):
            return self._hold("ball state is missing or invalid")
        f, c = self.field, self.config
        if abs(robot.x) > f.length / 2 or abs(robot.y) > f.width / 2:
            return self._hold("robot is outside the field")
        if abs(ball.x) > f.length / 2 or abs(ball.y) > f.width / 2:
            return self._hold("ball is outside the field; reassess/reset is required")

        distance_fraction = (f.attack_sign * ball.x + f.length / 2) / f.length
        emergency = distance_fraction <= c.emergency_defense
        desired = self.role
        if emergency or (self.role == "offense" and distance_fraction <= c.enter_defense):
            desired = "defense"
        elif self.role == "defense" and distance_fraction >= c.exit_defense:
            desired = "offense"
        dwell_complete = self._changed_at is None or now - self._changed_at >= c.minimum_dwell
        reason = "ball position retains current role"
        if desired != self.role:
            if dwell_complete or emergency:
                self.role = desired
                self._changed_at = now
                reason = "emergency own-goal threat" if emergency else "ball crossed role threshold"
            else:
                reason = "minimum role dwell"
        elif emergency:
            reason = "emergency own-goal threat"
        if self._changed_at is None:
            self._changed_at = now
        if self.role == "offense":
            return TacticalDecision(self.role, "attack", None, reason)
        target = self.guard_pose(ball)
        if target is None:
            return self._hold(reason + "; no ball-clear goalward guard pose fits inside the field and goal span")
        return TacticalDecision(self.role, "defend", target, reason)

    def guard_pose(self, ball, own_goal=None):
        f, c = self.field, self.config
        margin = f.robot_radius + c.boundary_clearance
        depth = min(max(c.guard_depth, margin), f.length / 2)
        goal = own_goal or {"x": -f.attack_sign * f.length / 2,
                            "y_min": -f.goal_width / 2, "y_max": f.goal_width / 2}
        x = goal["x"] + f.attack_sign * depth
        centre = (goal["y_min"] + goal["y_max"]) / 2
        lower = max(-f.width / 2 + margin, min(centre, goal["y_min"] + f.robot_radius))
        upper = min(f.width / 2 - margin, max(centre, goal["y_max"] - f.robot_radius))
        if lower > upper or abs(x) > f.length / 2 - margin:
            return None
        y = min(max(ball.y, lower), upper)
        clearance = defensive_clearance(f)
        if math.hypot(ball.x - x, ball.y - y) < clearance:
            # Preserve the nominal goal-mouth lane; only move goalward until
            # the bumper-inclusive exclusion circle clears that lane.
            goalward = math.sqrt(max(0.0, clearance ** 2 - (ball.y - y) ** 2))
            x = ball.x - f.attack_sign * goalward
            if (abs(x) > f.length / 2 - margin or not lower <= y <= upper
                    or abs(y) > f.width / 2 - margin):
                return None
        dx, dy = ball.x - x, ball.y - y
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else math.atan2(0.0, f.attack_sign)
        return Pose(x, y, yaw)
