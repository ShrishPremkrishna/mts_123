"""Command bridge for the learned header-kick policy (Mjlab-HeaderKick-Flat-Pupper-v3).

The policy reads the kick target in the command slot of the standard 36-value
frame, so on the robot it receives a Twist on /cmd_vel whose fields are NOT a
velocity:

    linear.x  = ball centre x, base_link yaw-only frame [m]
    linear.y  = ball centre y, same frame [m]
    angular.z = aim: wanted ball direction minus robot heading [rad]

This module turns ball (and optional goal) estimates into that command and
decides when the attempt is over. Pure Python, no ROS; ``learned_kick_node``
wraps it.

A zero Twist does NOT stop this policy (it means "ball at my feet"); the attempt
ends by switching back to a walking controller, which the node does.

Timing follows training: the onboard estimate freezes in the camera blind zone
(the policy trained with exactly that), so the kick window starts when the ball
enters ``end_range`` and lasts ``post_kick_s``. Outside it, a lost or frozen
estimate aborts quickly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


class Phase(Enum):
  IDLE = "idle"
  RUNNING = "running"
  DONE = "done"
  ABORTED = "aborted"


@dataclass(frozen=True)
class BridgeParams:
  # Entry window: the trained spawn ranges at full difficulty (pupper_kick README).
  min_distance: float = 0.45  # m, base_link to ball centre
  max_distance: float = 1.5
  max_bearing: float = 0.5  # rad
  max_cut: float = 0.4  # rad, |aim - bearing|
  # The onboard colour detector gives estimates 115-166 ms old at 10-17 Hz, so the
  # newest one can be up to about 0.3 s old (training saw up to about 0.3 s).
  start_max_age: float = 0.4  # s, newest estimate when the kick starts
  # Same clamp as the exported command_clip (vx, vy, wz).
  clip_x: tuple[float, float] = (-0.5, 2.0)
  clip_y: tuple[float, float] = (-1.0, 1.0)
  clip_aim: tuple[float, float] = (-1.2, 1.2)
  # Kick window. end_range: the camera blind zone (training froze the estimate
  # 0.30-0.45 m from base_link); for overhead tracking use about 0.30.
  end_range: float = 0.45
  post_kick_s: float = 1.25  # 0.25 s to contact + the 1 s training window after it
  # Loss handling: short hold far away, long hold inside the blind zone.
  hold_far_s: float = 0.4  # 0.25 s aborted robot trial L1 (2026-11-16) after 0.03 s
  hold_near_s: float = 1.5
  # Aborts.
  fade_in_s: float = 4.0  # kick controller init + fade-in (2 s + 2 s in config.yaml)
  max_run_s: float = 3.0  # after the fade-in; a 1.5 m approach plus the window is about 2.2 s
  abort_bearing: float = 1.0  # rad: ball well off the nose before the window = missed
  frozen_s: float = 0.5  # target unchanged this long after the fade-in, outside end_range
  frozen_eps: float = 0.01  # m


@dataclass(frozen=True)
class Target:
  x: float
  y: float
  stamp: float


def clamp(value: float, bounds: tuple[float, float]) -> float:
  return max(bounds[0], min(bounds[1], value))


def wrap(angle: float) -> float:
  return (angle + math.pi) % (2.0 * math.pi) - math.pi


def entry_check(p: BridgeParams, x: float, y: float, aim: float) -> str:
  """Empty string if a kick may start, else the reason it may not."""
  if not all(math.isfinite(v) for v in (x, y, aim)):
    return "non-finite ball or aim"
  distance = math.hypot(x, y)
  bearing = math.atan2(y, x)
  if not p.min_distance <= distance <= p.max_distance:
    return f"ball {distance:.2f} m away, need {p.min_distance}-{p.max_distance} m"
  if abs(bearing) > p.max_bearing:
    return f"ball bearing {math.degrees(bearing):.0f} deg, need within {math.degrees(p.max_bearing):.0f}"
  if abs(wrap(aim - bearing)) > p.max_cut:
    return f"aim {math.degrees(wrap(aim - bearing)):.0f} deg off the ball line, need within {math.degrees(p.max_cut):.0f}"
  return ""


@dataclass
class LearnedKickBridge:
  params: BridgeParams = field(default_factory=BridgeParams)

  def __post_init__(self) -> None:
    self.phase = Phase.IDLE
    self.reason = ""
    self._t_start = 0.0
    self._t_window = None  # type: float | None  # ball entered end_range
    self._target = None  # type: Target | None
    self._aim = 0.0  # aim at the start, robot frame at the start
    self._moved_ref = (0.0, 0.0)
    self._t_moved = 0.0

  @property
  def active(self) -> bool:
    return self.phase == Phase.RUNNING

  def start(self, now: float, ball: Target | None, aim: float | None) -> str:
    """Begin if the ball is fresh and inside the trained window; returns '' or why not."""
    p = self.params
    if ball is None or not math.isfinite(ball.stamp) or now - ball.stamp > p.start_max_age:
      return "no fresh ball estimate"
    aim = math.atan2(ball.y, ball.x) if aim is None else wrap(aim)
    why = entry_check(p, ball.x, ball.y, aim)
    if why:
      return why
    self.phase, self.reason = Phase.RUNNING, ""
    self._t_start = self._t_moved = now
    self._t_window = None
    self._target, self._aim = ball, aim
    self._moved_ref = (ball.x, ball.y)
    return ""

  def stop(self, reason: str = "stopped") -> None:
    if self.phase == Phase.RUNNING:
      self.phase, self.reason = Phase.ABORTED, reason

  def _aim_now(self, aim: float | None, yaw_delta: float | None) -> float:
    if aim is not None and math.isfinite(aim):
      return wrap(aim)
    if yaw_delta is not None and math.isfinite(yaw_delta):
      return wrap(self._aim - yaw_delta)  # held in the world, as in training
    return self._aim

  def _take(self, ball: Target | None) -> None:
    if (ball is not None and all(math.isfinite(v) for v in (ball.x, ball.y, ball.stamp))
        and ball.stamp > self._target.stamp):  # only newer estimates count
      self._target = ball

  def target_now(self, ball: Target | None, aim: float | None = None,
                 yaw_delta: float | None = None) -> tuple[float, float, float]:
    """The clamped command for the newest estimate, whatever the phase (used while
    the exit switch is pending, so the policy sees a target from training)."""
    p = self.params
    if self._target is None:
      return (0.0, 0.0, 0.0)
    self._take(ball)
    return (clamp(self._target.x, p.clip_x), clamp(self._target.y, p.clip_y),
            clamp(self._aim_now(aim, yaw_delta), p.clip_aim))

  def step(self, now: float, ball: Target | None, aim: float | None = None,
           yaw_delta: float | None = None) -> tuple[float, float, float]:
    """The command-slot values to publish now; (0, 0, 0) once not running.

    ``aim``: from a goal, robot frame now. Without one, the start aim is held in
    the world: aim = start aim - ``yaw_delta`` (robot yaw change since start).
    """
    if self.phase != Phase.RUNNING:
      return (0.0, 0.0, 0.0)
    p = self.params
    self._take(ball)
    tx, ty = self._target.x, self._target.y
    distance = math.hypot(tx, ty)
    if math.hypot(tx - self._moved_ref[0], ty - self._moved_ref[1]) > p.frozen_eps:
      self._moved_ref, self._t_moved = (tx, ty), now
    if self._t_window is None and distance <= p.end_range:
      self._t_window = now
    age = now - self._target.stamp
    after_fade = now - self._t_start > p.fade_in_s

    if self._t_window is not None:
      if now - self._t_window >= p.post_kick_s:
        self.phase, self.reason = Phase.DONE, "kick window over"
      elif age > p.hold_near_s:
        self.phase, self.reason = Phase.ABORTED, "ball estimate lost near the ball"
    elif age > p.hold_far_s:
      self.phase, self.reason = Phase.ABORTED, "ball estimate lost"
    elif abs(math.atan2(ty, tx)) > p.abort_bearing:
      self.phase, self.reason = Phase.ABORTED, "ball off to the side: missed"
    elif after_fade and now - self._t_moved > p.frozen_s:
      self.phase, self.reason = Phase.ABORTED, "target frozen (stale perception or robot stuck)"
    if self.phase == Phase.RUNNING and now - self._t_start > p.fade_in_s + p.max_run_s:
      self.phase, self.reason = Phase.ABORTED, "time limit"
    if self.phase != Phase.RUNNING:
      return (0.0, 0.0, 0.0)
    return self.target_now(None, aim, yaw_delta)
