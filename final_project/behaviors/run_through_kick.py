"""Run-through kick: line up behind the ball, then drive the walker through it.

Pure Python (no ROS, no numpy) so it runs on the robot and in tests unchanged.
The walking policy does the locomotion; this controller only chooses velocity
commands ``(vx, vy, wz)`` for it, in the robot frame (x forward, y left; m/s,
m/s, rad/s).

Inputs each tick:
  * ball: centre of the ball relative to ``base_link``, yaw-only robot frame [m],
    with the capture time of the image it came from (``BallObs``), or None.
  * aim: wanted ball travel direction, relative to the robot heading [rad]
    (``aim_from_goal`` makes it from a goal position), or None to kick straight
    through the ball.

States:
  ALIGN   walk to the point ``run_up`` behind the ball on the aim line while
          facing the ball (so the camera keeps it); at that point the bearing
          to the ball equals the aim. A ball inside ``keep_out`` of the straight
          path is passed on the side the robot is already on.
  CHARGE  ramp to ``charge_speed`` along the aim, nudging sideways to keep the
          ball on the body axis.
  COAST   the ball is in the camera blind zone: hold the command, dead-reckon
          the ball from the commanded motion, and keep going ``overrun`` past
          contact.
  BRAKE   zero command for ``brake_time``, then DONE.
  ABORTED zero command (stale/lost ball, timeout, or ``stop()``).

Stale-track rule (team AGENTS.md): while aligning, an observation older than
``max_obs_age`` gives zero travel; ``lost_timeout`` without one aborts.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum


class State(Enum):
  IDLE = "idle"
  ALIGN = "align"
  CHARGE = "charge"
  COAST = "coast"
  BRAKE = "brake"
  DONE = "done"
  ABORTED = "aborted"


@dataclass(frozen=True)
class BallObs:
  x: float  # m, forward of base_link
  y: float  # m, left of base_link
  stamp: float  # s, capture time on the controller's clock


@dataclass(frozen=True)
class Command:
  vx: float = 0.0
  vy: float = 0.0
  wz: float = 0.0


ZERO = Command()


@dataclass(frozen=True)
class KickParams:
  # Geometry. contact_x: ball-centre x at contact = body/bumper front (0.15-0.17 m)
  # + ball radius (0.075 m nominal).
  run_up: float = 0.8  # m, base_link to ball centre when the charge starts
  contact_x: float = 0.24
  overrun: float = 0.25  # m driven past the contact point before braking
  # m, min clearance from the ball centre while aligning; above blind_range so
  # the camera keeps the ball during a detour.
  keep_out: float = 0.5
  # Alignment.
  align_speed: float = 0.35  # m/s; stays in the walker's walk band (cap 0.49)
  align_kp: float = 1.5  # 1/s
  yaw_kp: float = 2.0  # 1/s
  pos_tol: float = 0.06  # m
  yaw_tol: float = 0.08  # rad
  lat_tol: float = 0.04  # m, ball offset from the body axis
  # Charge. charge_speed above 0.49 m/s needs the walker's speed_cap raised
  # (follow_me_pupper/robot/speed_cap.yaml sets 1.5).
  charge_speed: float = 1.0  # m/s
  charge_accel: float = 1.5  # m/s^2
  lateral_kp: float = 1.5  # 1/s
  blind_range: float = 0.40  # m; closer ball-centre ranges are not trusted
  speed_efficiency: float = 0.8  # achieved / commanded speed (measure on the floor)
  # Limits (the walker's trained ranges: |vy| <= 0.5 m/s, |wz| <= 2 rad/s).
  max_vy: float = 0.4
  max_wz: float = 1.5
  # Timing.
  brake_time: float = 0.6
  max_obs_age: float = 0.5  # s
  lost_timeout: float = 1.5  # s without a fresh ball while aligning
  align_timeout: float = 20.0
  charge_timeout: float = 4.0


def wrap(angle: float) -> float:
  return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clip(value: float, limit: float) -> float:
  return max(-limit, min(limit, value))


def aim_from_goal(ball_x: float, ball_y: float, goal_x: float, goal_y: float) -> float:
  """Aim [rad, robot frame] that sends the ball from its position to the goal."""
  return math.atan2(goal_y - ball_y, goal_x - ball_x)


def _segment_distance(px: float, py: float, qx: float, qy: float) -> float:
  """Distance from point (px, py) to the segment from the origin to (qx, qy)."""
  qq = qx * qx + qy * qy
  t = 0.0 if qq == 0.0 else max(0.0, min(1.0, (px * qx + py * qy) / qq))
  return math.hypot(px - t * qx, py - t * qy)


@dataclass
class RunThroughKick:
  params: KickParams = field(default_factory=KickParams)

  def __post_init__(self) -> None:
    self.state = State.IDLE
    self.reason = ""
    self._t_state = 0.0
    self._t_last = None  # type: float | None
    self._t_fresh = 0.0
    self._vx = 0.0  # last commanded forward speed (for the ramp)
    self._last = ZERO
    # (time, Command) since the newest observation, for latency compensation.
    self._history: deque[tuple[float, Command]] = deque(maxlen=200)
    self._ball = None  # type: tuple[float, float] | None  # best estimate, robot frame
    self._aim = 0.0

  # ------------------------------------------------------------------ control

  def start(self, now: float) -> None:
    self._enter(State.ALIGN, now)
    self._t_fresh = now
    self._vx = 0.0
    self._ball = None

  def stop(self, now: float, reason: str = "stopped") -> Command:
    self._abort(now, reason)
    return ZERO

  @property
  def active(self) -> bool:
    return self.state in (State.ALIGN, State.CHARGE, State.COAST, State.BRAKE)

  def step(self, now: float, ball: BallObs | None, aim: float | None = None) -> Command:
    """Advance one tick and return the velocity command to publish."""
    dt = 0.0 if self._t_last is None else max(0.0, now - self._t_last)
    self._t_last = now
    self._dead_reckon(dt)

    p = self.params
    fresh = ball is not None and now - ball.stamp <= p.max_obs_age
    if fresh:
      self._ball = self._compensate(ball, now)
      self._t_fresh = now
    if aim is not None:
      self._aim = wrap(aim)
    elif self._ball is not None and self.state == State.ALIGN:
      self._aim = math.atan2(self._ball[1], self._ball[0])  # straight through

    if self.state == State.ALIGN:
      cmd = self._align(now, fresh)
    elif self.state == State.CHARGE:
      cmd = self._charge(now, fresh, dt)
    elif self.state == State.COAST:
      cmd = self._coast(now, dt)
    elif self.state == State.BRAKE:
      cmd = ZERO
      if now - self._t_state >= p.brake_time:
        self._enter(State.DONE, now)
    else:
      cmd = ZERO

    self._last = cmd
    self._history.append((now, cmd))
    return cmd

  # ------------------------------------------------------------------ states

  def _align(self, now: float, fresh: bool) -> Command:
    p = self.params
    if now - self._t_state > p.align_timeout:
      return self._abort_cmd(now, "align timeout")
    if not fresh:
      if now - self._t_fresh > p.lost_timeout:
        return self._abort_cmd(now, "ball lost while aligning")
      return ZERO  # stale track: request zero travel
    bx, by = self._ball
    ax, ay = math.cos(self._aim), math.sin(self._aim)
    tx, ty = bx - p.run_up * ax, by - p.run_up * ay  # run-up point
    heading_err = self._aim
    # At the run-up point, facing the aim, the ball sits on the body axis.
    if math.hypot(tx, ty) < p.pos_tol and abs(heading_err) < p.yaw_tol and abs(by) < p.lat_tol:
      self._enter(State.CHARGE, now)
      self._vx = max(0.0, self._last.vx)
      return self._charge(now, True, 0.0)

    # Ball in the way: orbit it at radius keep_out + 0.1, at most 45 deg per
    # waypoint toward the run-up side (chord midpoints stay >= 0.92 x radius).
    if _segment_distance(bx, by, tx, ty) < p.keep_out:
      radius = p.keep_out + 0.1
      th_robot = math.atan2(-by, -bx)  # robot as seen from the ball
      th_goal = math.atan2(-ay, -ax)  # run-up point as seen from the ball
      th = th_robot + clip(wrap(th_goal - th_robot), math.pi / 4)
      tx, ty = bx + radius * math.cos(th), by + radius * math.sin(th)

    vx, vy = p.align_kp * tx, p.align_kp * ty
    norm = math.hypot(vx, vy)
    if norm > p.align_speed:
      vx, vy = vx * p.align_speed / norm, vy * p.align_speed / norm
    bearing = math.atan2(by, bx)  # face the ball: equals the aim at the run-up point
    return Command(vx, clip(vy, p.max_vy), clip(p.yaw_kp * bearing, p.max_wz))

  def _charge(self, now: float, fresh: bool, dt: float) -> Command:
    p = self.params
    if now - self._t_state > p.charge_timeout:
      return self._abort_cmd(now, "charge timeout")
    bx, by = self._ball
    if math.hypot(bx, by) < p.blind_range or bx < p.contact_x + 0.05:
      self._enter(State.COAST, now)
      return self._coast(now, dt)
    if not fresh:
      return self._abort_cmd(now, "ball lost before the blind zone")
    self._vx = min(p.charge_speed, self._vx + p.charge_accel * dt)
    return Command(
      self._vx, clip(p.lateral_kp * by, p.max_vy), clip(p.yaw_kp * self._aim, p.max_wz)
    )

  def _coast(self, now: float, dt: float) -> Command:
    p = self.params
    if now - self._t_state > p.charge_timeout:
      return self._abort_cmd(now, "coast timeout")
    if self._ball[0] < p.contact_x - p.overrun:
      self._enter(State.BRAKE, now)
      return ZERO
    self._vx = min(p.charge_speed, self._vx + p.charge_accel * dt)
    return Command(self._vx, 0.0, 0.0)

  # ------------------------------------------------------------------ helpers

  def _enter(self, state: State, now: float) -> None:
    self.state = state
    self._t_state = now

  def _abort(self, now: float, reason: str) -> None:
    self.reason = reason
    self._enter(State.ABORTED, now)
    self._vx = 0.0

  def _abort_cmd(self, now: float, reason: str) -> Command:
    self._abort(now, reason)
    return ZERO

  def _compensate(self, ball: BallObs, now: float) -> tuple[float, float]:
    """Move an observation from its capture time to ``now`` using the commands
    sent since then (scaled by ``speed_efficiency``)."""
    x, y = ball.x, ball.y
    prev_t, active = ball.stamp, ZERO
    for t, cmd in self._history:  # each command holds from its time to the next
      if t <= ball.stamp:
        active = cmd
        continue
      x, y = self._move(x, y, active, t - prev_t)
      prev_t, active = t, cmd
    return self._move(x, y, active, now - prev_t)

  def _dead_reckon(self, dt: float) -> None:
    if self._ball is not None and dt > 0.0:
      self._ball = self._move(*self._ball, self._last, dt)

  def _move(self, x: float, y: float, cmd: Command, dt: float) -> tuple[float, float]:
    """Ball position in the robot frame after the robot moves with ``cmd`` for dt."""
    if dt <= 0.0:
      return x, y
    e = self.params.speed_efficiency
    x -= cmd.vx * e * dt
    y -= cmd.vy * e * dt
    th = -cmd.wz * e * dt
    c, s = math.cos(th), math.sin(th)
    return c * x - s * y, s * x + c * y
