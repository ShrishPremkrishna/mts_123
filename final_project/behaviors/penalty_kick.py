"""Penalty kick: find the ball, line up on the ball-to-target line, kick with a front leg.

Pure Python (no ROS, no numpy), so the same code runs on the robot, in the
kinematic simulation (``penalty_sim.py``) and in the tests.
``penalty_kick_node.py`` connects it to ROS.

Frames
  robot  base_link yaw-only frame: x forward, y left [m]. The ball detector
         (/soccer/ball) publishes in this frame.
  W      the robot's pose when the attempt starts (x = start heading). The
         heading in W comes from the IMU yaw, with the drift rate removed. The
         drift rate is measured while the robot stands still at the start.

Target point (a point in the goal), parameter ``goal_frame``:
  ball   (target_x, target_y) from the ball, along the W axes. Put the robot
         down facing the goal direction: (2.0, 0.2) is 2 m past the ball and
         0.2 m to the left. Only the start heading matters, not the position.
  start  (target_x, target_y) in W.
  topic  a live goal point in the robot frame (/soccer/goal), for goal
         perception or overhead tracking.

Kick geometry (leg-kick handoff 2026-11-16, policy run 2): at the strike, the
ball centre is at (strike_x, strike_y_<side>) in the robot frame, and the ball
leaves at shot_bias_<side> from the body heading. Thus the kick heading is
aim - shot_bias, and the ball is offset toward the kicking paw.

States
  SEARCH    stand still and look; with no consistent ball, turn scan_step and
            look again. The first look also measures the IMU yaw drift.
  APPROACH  walk to the staging pose: ball at (stage_x, strike_y) in the kick
            heading frame. The robot faces the ball, so the camera keeps it. A
            ball in the way is passed on an orbit outside keep_out.
  MEASURE   stand still and average the ball detections.
  CORRECT   turn on the IMU yaw, then a timed step, to remove the measured
            error; then MEASURE again (fine_max_iters times at most).
  CREEP     walk to the strike point. The camera does not see a ball closer
            than blind_range, so the last part uses odometry: commanded
            velocity x creep_efficiency through a first-order lag
            (walker_lag_s), plus the walker's sideways drift (walk_drift).
  SETTLE    stand still, then check the placement (strike "none" ends here).
  STRIKE    StrikeSequencer: switch to the kick slot, request the clip, switch back.
  OBSERVE   stand and watch the ball; record the shot direction.
  DONE, ABORTED  zero command.

Inputs per tick: ball observations (capture-stamped, robot frame), the raw IMU
yaw, and the goal (topic mode). Outputs: a velocity command for the walker and
strike actions (``pop_actions``): ("switch", "in" | "out"), ("clip", name),
("estop",). The caller reports results with ``on_switch``, ``on_clip_status``
and ``on_joint_speed``.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum

from run_through_kick import ZERO, BallObs, Command, clip, finite, wrap


class State(Enum):
  IDLE = "idle"
  SEARCH = "search"
  APPROACH = "approach"
  MEASURE = "measure"
  CORRECT = "correct"
  CREEP = "creep"
  SETTLE = "settle"
  STRIKE = "strike"
  OBSERVE = "observe"
  DONE = "done"
  ABORTED = "aborted"


ACTIVE = (State.SEARCH, State.APPROACH, State.MEASURE, State.CORRECT, State.CREEP,
          State.SETTLE, State.STRIKE, State.OBSERVE)
SIDES = ("right", "left")
GOAL_FRAMES = ("ball", "start", "topic")
STRIKES = ("leg_kick", "none")


@dataclass(frozen=True)
class PenaltyParams:
  # Target (module docstring).
  goal_frame: str = "ball"
  target_x: float = 2.0  # m
  target_y: float = 0.0  # m
  side: str = "auto"  # right, left, or auto (the side with the shorter walk; ties go right)
  # leg_kick: the standing leg-kick trick in the kick slot. none: dry run, stop
  # after the placement check (alignment trials without the kick slot).
  strike: str = "leg_kick"
  # Kick geometry, robot frame. Beach ball (radius 0.097 m) placement from the
  # 2026-11-16 robot kicks; sim run 2 for the left y and the biases.
  strike_x: float = 0.258  # m
  strike_y_right: float = -0.046  # m
  strike_y_left: float = 0.040  # m
  shot_bias_right_deg: float = -3.4  # ball direction minus body heading
  shot_bias_left_deg: float = 3.3
  # Camera limits and staging.
  stage_x: float = 0.60  # m, ball x at the staging pose (detector checked at 0.59 m)
  blind_range: float = 0.45  # m, closer ball ranges are not used for control
  max_range: float = 3.0  # m, the detector's limit
  keep_out: float = 0.45  # m, min ball distance on the orbit (>= blind_range keeps the ball in view)
  # Search.
  look_s: float = 1.0
  bias_s: float = 2.0  # length of the first look, which also measures the IMU drift
  max_bias_dps: float = 3.0  # deg/s; more drift aborts (fused-yaw fault, fr-leg-jiggle handoff)
  min_hits: float = 4.0  # consistent detections to accept a ball
  hit_spread: float = 0.15  # m, max distance of a hit from the median
  scan_step_deg: float = 90.0  # the fisheye sees +-110 deg, so steps overlap
  scan_wz: float = 0.6  # rad/s
  scan_dir: float = 1.0  # +1 turns left, -1 right
  scan_max_deg: float = 450.0
  max_reacquire: float = 2.0  # SEARCH again after a lost ball, then abort
  # Approach.
  approach_speed: float = 0.30  # m/s
  approach_kp: float = 1.2  # 1/s
  min_speed: float = 0.05  # m/s, the walker barely moves below this
  yaw_kp: float = 1.5  # 1/s
  max_vy: float = 0.30
  max_wz: float = 1.0
  coarse_pos_tol: float = 0.05  # m, staging pose reached: start the fine alignment
  coarse_yaw_tol_deg: float = 5.0
  max_obs_age: float = 0.5  # s; an older ball gives zero travel
  lost_timeout: float = 2.0  # s without a fresh ball: SEARCH again
  approach_timeout: float = 30.0
  approach_travel_max: float = 6.0  # m of commanded travel
  obs_gain: float = 0.5  # blend of a new detection into the ball estimate
  # Walker model. Measure all four on the floor (PENALTY_KICK.md).
  speed_efficiency: float = 0.8  # achieved / commanded speed at approach speeds
  creep_efficiency: float = 0.8  # same at creep and correction speeds
  walker_lag_s: float = 0.15  # first-order velocity lag
  walk_drift: float = 0.0  # m the walker drifts left per m forward
  # Fine alignment at the staging pose.
  settle_s: float = 0.6
  measure_s: float = 0.8
  min_measure_obs: float = 5.0
  fine_lat_tol: float = 0.01  # m; the shot turns approximately 7 deg per cm of lateral error
  fine_yaw_tol_deg: float = 1.5
  accept_lat_tol: float = 0.02  # m, used after fine_max_iters and at the strike
  accept_yaw_tol_deg: float = 3.0
  stage_x_tol: float = 0.10  # m; the creep length absorbs an x error
  fine_max_iters: float = 4.0
  corr_speed: float = 0.08  # m/s, correction steps
  corr_wz: float = 0.3  # rad/s, correction turns
  corr_min_wz: float = 0.12
  corr_turn_timeout: float = 4.0
  # Creep to the strike point.
  creep_speed: float = 0.15  # m/s
  creep_accel: float = 0.5  # m/s^2
  creep_lat_kp: float = 1.0  # 1/s, while the ball is still visible
  creep_lat_ki: float = 1.0  # 1/s^2, holds the learned sideways pull into the blind part
  creep_yaw_kp: float = 2.0
  creep_margin: float = 0.06  # m of commanded travel past the plan before an abort
  creep_timeout: float = 8.0
  # Strike (StrikeSequencer). The defaults match leg_kick_buttons.py on the robot.
  pre_kick_settle_s: float = 1.0
  strike_x_tol: float = 0.05  # m, placement check after the creep
  kick_delay_s: float = 5.5  # switch-in to clip request: slot init + fade-in (2 s + 2 s) + margin
  ready_timeout_s: float = 3.0  # extra wait for the slot's first clip status
  clip_start_timeout_s: float = 2.0  # the clip starts at the next stand_idle boundary (<= 1 s)
  clip_timeout_s: float = 4.5  # kick clip 3.0 s
  exit_delay_s: float = 0.5  # after the clip returns to stand_idle
  switch_timeout_s: float = 2.0
  exit_retries: float = 2.0
  status_gap_s: float = 0.5  # no clip status this long: the slot is inactive
  watch_rad_s: float = 1.5  # window mean of max |joint velocity| outside a kick
  watch_window_s: float = 0.3
  watch_grace_s: float = 1.0
  # Outcome.
  observe_s: float = 3.0
  min_shot_travel: float = 0.15  # m; less: the ball did not move


def check_params(p: PenaltyParams) -> None:
  """Raise ValueError for a setting that would make the attempt unsafe or meaningless."""
  if p.goal_frame not in GOAL_FRAMES:
    raise ValueError(f"goal_frame must be one of {GOAL_FRAMES}")
  if p.side not in SIDES + ("auto",):
    raise ValueError("side must be right, left or auto")
  if p.strike not in STRIKES:
    raise ValueError(f"strike must be one of {STRIKES}")
  if p.goal_frame != "topic" and math.hypot(p.target_x, p.target_y) < 0.3:
    raise ValueError("set target_x/target_y (the goal point is less than 0.3 m away)")
  if not p.strike_x < p.blind_range <= p.stage_x:
    raise ValueError("need strike_x < blind_range <= stage_x")
  if p.keep_out < p.blind_range:
    raise ValueError("keep_out below blind_range would lose the ball on the orbit")
  for name in ("speed_efficiency", "creep_efficiency"):
    if not 0.3 <= getattr(p, name) <= 1.5:
      raise ValueError(f"{name} must be in [0.3, 1.5]")
  if not 0.0 < p.creep_speed <= 0.3 or not 0.0 < p.approach_speed <= 0.49:
    raise ValueError("creep_speed must be in (0, 0.3] and approach_speed in (0, 0.49] m/s")


def rot(x: float, y: float, a: float) -> tuple[float, float]:
  c, s = math.cos(a), math.sin(a)
  return c * x - s * y, s * x + c * y


def move(p: tuple[float, float], dx: float, dy: float, dyaw: float) -> tuple[float, float]:
  """A fixed point in the robot frame after the robot moves (dx, dy) and turns dyaw."""
  return rot(p[0] - dx, p[1] - dy, -dyaw)


def segment_distance(px: float, py: float, qx: float, qy: float) -> float:
  """Distance from (px, py) to the segment from the origin to (qx, qy)."""
  qq = qx * qx + qy * qy
  t = 0.0 if qq == 0.0 else max(0.0, min(1.0, (px * qx + py * qy) / qq))
  return math.hypot(px - t * qx, py - t * qy)


def slope(samples: list[tuple[float, float]]) -> float:
  """Least-squares slope of (t, v) samples; 0 for fewer than 2."""
  n = len(samples)
  if n < 2:
    return 0.0
  mt = sum(t for t, _ in samples) / n
  mv = sum(v for _, v in samples) / n
  den = sum((t - mt) ** 2 for t, _ in samples)
  return 0.0 if den <= 0 else sum((t - mt) * (v - mv) for t, v in samples) / den


def cluster(points: list[tuple[float, float]], spread: float, min_n: int):
  """Mean of the points within ``spread`` of the median, or None if fewer than min_n."""
  if len(points) < min_n:
    return None
  xs, ys = sorted(p[0] for p in points), sorted(p[1] for p in points)
  mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
  near = [p for p in points if math.hypot(p[0] - mx, p[1] - my) <= spread]
  if len(near) < min_n:
    return None
  return sum(p[0] for p in near) / len(near), sum(p[1] for p in near) / len(near)


class StrikeSequencer:
  """Switch to the kick slot, play one kick clip, switch back to the walker.

  Same contract as leg_kick_buttons.py (tested on the robot 2026-11-16), but the
  sequencer makes the switches itself (STRICT), so no button is necessary:
    in       switch walker -> kick slot
    wait     kick_delay_s for the slot's init and fade-in, then request the clip
    request  the clip starts at the next stand_idle boundary
    kick     the clip plays; exit_delay_s after it returns to stand_idle: out
    out      switch kick slot -> walker; retries, then ("estop",)
  Watchdog: while the slot is active outside a kick clip (and watch_grace_s
  after it), a window mean of max |joint velocity| above watch_rad_s switches
  out (the trick policy oscillates with its feet unloaded).
  """

  IDLE, IN, WAIT, REQUEST, KICK, OUT, DONE, FAILED = (
    "idle", "switch_in", "wait", "request", "kick", "switch_out", "done", "failed")

  def __init__(self, params: PenaltyParams):
    self.params = params
    self.phase = self.IDLE
    self.ok = False
    self.reason = ""
    self.side = "right"
    self.actions: list[tuple] = []
    self._reset(0.0)

  def _reset(self, now: float) -> None:
    self.t_phase = now
    self.last_status = None
    self.clip = None
    self.kick_start = None
    self.kick_end = None
    self.window: deque = deque()
    self.tries = 0
    self.cancel = None
    self.exit_ok = False

  @property
  def finished(self) -> bool:
    return self.phase in (self.DONE, self.FAILED)

  @property
  def slot_may_be_active(self) -> bool:
    return self.phase in (self.IN, self.WAIT, self.REQUEST, self.KICK, self.OUT)

  def start(self, now: float, side: str) -> None:
    self._reset(now)
    self.side, self.ok, self.reason = side, False, ""
    self._set(self.IN, now)
    self.actions.append(("switch", "in"))

  def _set(self, phase: str, now: float) -> None:
    self.phase, self.t_phase = phase, now

  def _fail(self, now: float, reason: str) -> None:
    self.ok, self.reason = False, reason
    self._set(self.FAILED, now)

  def _exit(self, now: float, reason: str, ok: bool = False) -> None:
    self.exit_ok, self.reason, self.tries = ok, reason, 1
    self._set(self.OUT, now)
    self.actions.append(("switch", "out"))

  def abort(self, now: float, reason: str, switch: bool = True) -> None:
    """switch=False: the e-stop or the operator owns the controllers now."""
    if not self.slot_may_be_active:
      return
    if not switch:
      return self._fail(now, reason)
    if self.phase == self.IN:
      self.cancel = reason  # acted on when the switch answer arrives
    elif self.phase != self.OUT:
      self._exit(now, reason)

  def on_switch(self, now: float, which: str, ok: bool) -> None:
    if which == "in" and self.phase == self.IN:
      if not ok:
        return self._fail(now, "the switch to the kick slot was refused")
      if self.cancel:
        return self._exit(now, self.cancel)
      self._set(self.WAIT, now)
    elif which == "out" and self.phase == self.OUT:
      if ok:
        self.ok = self.exit_ok
        self._set(self.DONE if self.exit_ok else self.FAILED, now)
      else:
        self._retry(now)

  def _retry(self, now: float) -> None:
    self.tries += 1
    if self.tries > 1 + int(self.params.exit_retries):
      self.actions.append(("estop",))
      return self._fail(now, self.reason + "; switch back refused, emergency stop sent")
    self.t_phase = now
    self.actions.append(("switch", "out"))

  def on_clip_status(self, now: float, clip_name: str) -> None:
    if self.phase not in (self.WAIT, self.REQUEST, self.KICK):
      return
    self.last_status = now
    if clip_name == self.clip:
      return
    if self.phase == self.REQUEST and clip_name.startswith("kick"):
      self.kick_start = now
      self._set(self.KICK, now)
    elif self.phase == self.KICK and clip_name == "stand_idle" and self.kick_end is None:
      self.kick_end = now
    self.clip = clip_name

  def on_joint_speed(self, now: float, max_speed: float) -> None:
    p = self.params
    if self.phase not in (self.WAIT, self.REQUEST, self.KICK) or self.last_status is None:
      return
    in_kick = self.phase == self.KICK and self.kick_end is None
    settling = self.kick_end is not None and now - self.kick_end < p.watch_grace_s
    if in_kick or settling:
      self.window.clear()
      return
    self.window.append((now, max_speed))
    while self.window[0][0] < now - p.watch_window_s:
      self.window.popleft()
    if now - self.window[0][0] < 0.8 * p.watch_window_s:
      return  # not a full window yet
    mean = sum(v for _, v in self.window) / len(self.window)
    if mean > p.watch_rad_s:
      self._exit(now, f"watchdog: joint speed {mean:.1f} rad/s (window mean) outside the kick")

  def tick(self, now: float) -> None:
    p = self.params
    age = now - self.t_phase
    if self.phase in (self.WAIT, self.REQUEST, self.KICK) and self.last_status is not None \
        and now - self.last_status > p.status_gap_s:
      return self._fail(now, "the kick slot stopped publishing clip status (inactive?)")
    if self.phase == self.IN and age > p.switch_timeout_s:
      self._exit(now, "no answer to the switch to the kick slot")
    elif self.phase == self.WAIT:
      if self.last_status is not None and age >= p.kick_delay_s:
        self.actions.append(("clip", f"kick_{self.side}"))
        self._set(self.REQUEST, now)
      elif age > p.kick_delay_s + p.ready_timeout_s:
        self._exit(now, "the kick slot published no clip status")
    elif self.phase == self.REQUEST and age > p.clip_start_timeout_s:
      self._exit(now, "the kick clip did not start")
    elif self.phase == self.KICK:
      if self.kick_end is not None and now - self.kick_end >= p.exit_delay_s:
        self._exit(now, "kick done", ok=True)
      elif self.kick_end is None and now - self.kick_start > p.clip_timeout_s:
        self._exit(now, "the kick clip did not end")
    elif self.phase == self.OUT and age > p.switch_timeout_s:
      self._retry(now)


class PenaltyKick:
  def __init__(self, params: PenaltyParams | None = None):
    self.params = params or PenaltyParams()
    self.state = State.IDLE
    self.reason = ""
    self.strike = StrikeSequencer(self.params)
    self._clear()

  # ------------------------------------------------------------------ public

  @property
  def active(self) -> bool:
    return self.state in ACTIVE

  def start(self, now: float) -> None:
    p = self.params
    check_params(p)
    self._clear()
    self.strike = StrikeSequencer(p)
    self.reason = ""
    self._enter(State.SEARCH, now, "look")

  def stop(self, now: float, reason: str = "stopped", switch: bool = True) -> None:
    """End the attempt. In STRIKE with switch=True, the walker is switched back
    first and the state goes to ABORTED when that is confirmed."""
    if not self.active:
      return
    if self.state == State.STRIKE and not self.strike.finished:
      self.strike.abort(now, reason, switch)
      if not self.strike.finished:
        return
    self._abort(now, reason)

  def pop_actions(self) -> list[tuple]:
    out, self.strike.actions = self.strike.actions, []
    return out

  def on_switch(self, now: float, which: str, ok: bool) -> None:
    self.strike.on_switch(now, which, ok)

  def on_clip_status(self, now: float, clip_name: str) -> None:
    self.strike.on_clip_status(now, clip_name)

  def on_joint_speed(self, now: float, max_speed: float) -> None:
    self.strike.on_joint_speed(now, max_speed)

  @property
  def yaw(self) -> float:
    """Heading in W [rad], drift removed."""
    return self._yaw

  def status(self) -> dict:
    return {"state": self.state.value, "phase": self.phase, "side": self.side,
            "reason": self.reason, **self.info}

  def step(self, now: float, ball: BallObs | None = None, yaw: float | None = None,
           goal: tuple[float, float] | None = None) -> Command:
    """Advance one tick. ``yaw``: raw IMU yaw [rad] or None; ``goal``: fresh goal
    point in the robot frame (topic mode) or None."""
    if not self.active:
      return ZERO
    dt = 0.0 if self._t_last is None else max(0.0, now - self._t_last)
    self._t_last = now
    self._advance(now, dt, yaw)
    if goal is not None and finite(*goal):
      self._goal = (float(goal[0]), float(goal[1]))
    if ball is not None and finite(ball.x, ball.y, ball.stamp) \
        and -0.05 <= now - ball.stamp <= self.params.max_obs_age and ball.stamp > self._last_stamp:
      self._observe(now, ball)
    handler = {
      State.SEARCH: self._search, State.APPROACH: self._approach, State.MEASURE: self._measure,
      State.CORRECT: self._correct, State.CREEP: self._creep, State.SETTLE: self._settle,
      State.STRIKE: self._strike_tick, State.OBSERVE: self._observe_tick,
    }[self.state]
    cmd = handler(now, dt)
    if not finite(cmd.vx, cmd.vy, cmd.wz):
      self._abort(now, "non-finite command")
      cmd = ZERO
    if not self.active:
      cmd = ZERO
    self._cmd = cmd
    return cmd

  # ------------------------------------------------------------------ state

  def _clear(self) -> None:
    self.phase = ""
    self.side = None
    self.info: dict = {}
    self._t_state = self._t_phase = 0.0
    self._t_last = None
    self._cmd = ZERO
    self._v = [0.0, 0.0]  # modelled walker velocity, robot frame [m/s]
    self._incs: deque = deque(maxlen=400)  # (t0, t1, dx, dy, dyaw) robot motion per tick
    self._yaw = 0.0
    self._yaw_t0 = None
    self._yaw_raw_prev = None
    self._yaw_unwrapped = 0.0
    self._bias = 0.0
    self._bias_samples: list = []
    self._bias_done = False
    self._pos = [0.0, 0.0]  # odometry in W (logging; goal_frame start)
    self._ball = None  # estimate, robot frame
    self._t_fresh = -math.inf  # capture time of the newest usable detection
    self._last_stamp = -math.inf
    self._goal = None
    self._hits: list = []
    self._samples: list = []
    self._near: list = []
    self._looks = 0
    self._scan_turned = 0.0
    self._turn_from = 0.0
    self._seen_in_turn = False
    self._aim = None  # wanted ball direction in W
    self._heading = None  # kick heading in W
    self._iters = 0
    self._reacquire = 0
    self._travel = 0.0
    self._vx = 0.0
    self._lat_i = 0.0
    self._step = None
    self._creep_budget = 0.0
    self._strike_yaw = 0.0
    self._spot = None
    self._best = None

  def _enter(self, state: State, now: float, phase: str = "") -> None:
    self.state, self._t_state = state, now
    self.phase, self._t_phase = phase, now
    self._travel = 0.0

  def _set_phase(self, phase: str, now: float) -> None:
    self.phase, self._t_phase = phase, now

  def _abort(self, now: float, reason: str) -> None:
    self.reason = reason
    self._enter(State.ABORTED, now)
    self._vx = 0.0

  def _abort_cmd(self, now: float, reason: str) -> Command:
    self._abort(now, reason)
    return ZERO

  def _sy(self, side: str | None = None) -> float:
    p = self.params
    return p.strike_y_right if (side or self.side) == "right" else p.strike_y_left

  def _bias_rad(self, side: str) -> float:
    p = self.params
    return math.radians(p.shot_bias_right_deg if side == "right" else p.shot_bias_left_deg)

  def _h(self) -> float:
    """Kick heading relative to the body [rad] (0 = facing the kick heading)."""
    return wrap(self._heading - self._yaw)

  def _in_kick_frame(self) -> tuple[float, float, float]:
    """(ball x, ball y) in the kick-heading frame, and the heading error h."""
    h = self._h()
    kx, ky = rot(self._ball[0], self._ball[1], -h)
    return kx, ky, h

  # ------------------------------------------------------------------ odometry and perception

  def _advance(self, now: float, dt: float, yaw_raw: float | None) -> None:
    p = self.params
    prev = self._yaw
    eff = p.creep_efficiency if self.state in (State.CORRECT, State.CREEP, State.SETTLE) \
      else p.speed_efficiency
    if yaw_raw is not None and math.isfinite(yaw_raw):
      if self._yaw_raw_prev is None:
        self._yaw_t0, self._yaw_unwrapped = now, 0.0
      else:
        self._yaw_unwrapped += wrap(yaw_raw - self._yaw_raw_prev)
      self._yaw_raw_prev = yaw_raw
      if self.state == State.SEARCH and not self._bias_done:
        self._bias_samples.append((now, self._yaw_unwrapped))
      self._yaw = self._yaw_unwrapped - self._bias * (now - self._yaw_t0)
    else:  # no IMU sample: the commanded turn
      self._yaw += self._cmd.wz * eff * dt
    if dt <= 0.0:
      return
    a = 1.0 - math.exp(-dt / p.walker_lag_s) if p.walker_lag_s > 0 else 1.0
    self._v[0] += (self._cmd.vx - self._v[0]) * a
    self._v[1] += (self._cmd.vy - self._v[1]) * a
    dx = self._v[0] * eff * dt
    dy = (self._v[1] + p.walk_drift * self._v[0]) * eff * dt
    dyaw = self._yaw - prev
    self._incs.append((now - dt, now, dx, dy, dyaw))
    self._travel += math.hypot(dx, dy)
    wx, wy = rot(dx, dy, prev)
    self._pos[0] += wx
    self._pos[1] += wy
    if self._ball is not None:
      self._ball = move(self._ball, dx, dy, dyaw)

  def _compensate(self, obs: BallObs) -> tuple[float, float, float]:
    """(x, y) of an observation moved from its capture time to now, and the W yaw at capture."""
    x, y, yaw_c = obs.x, obs.y, self._yaw
    for t0, t1, dx, dy, dyaw in self._incs:
      if t1 <= obs.stamp:
        continue
      f = 1.0 if t0 >= obs.stamp else (t1 - obs.stamp) / (t1 - t0)
      x, y = move((x, y), dx * f, dy * f, dyaw * f)
      yaw_c -= dyaw * f
    return x, y, yaw_c

  def _observe(self, now: float, obs: BallObs) -> None:
    p = self.params
    self._last_stamp = obs.stamp
    rng = math.hypot(obs.x, obs.y)
    if rng > p.max_range:
      return
    x, y, yaw_c = self._compensate(obs)
    if self.state == State.OBSERVE:
      return self._record_shot(obs, yaw_c)
    if rng < p.blind_range:  # not trusted for control; kept as evidence
      if len(self._near) < 50:
        self._near.append((round(obs.x, 3), round(obs.y, 3)))
      return
    if self.state in (State.SETTLE, State.STRIKE):
      return
    if self.state == State.SEARCH:
      if self.phase == "turn":
        self._seen_in_turn = True
      elif obs.stamp >= self._t_phase:
        # W axes from the robot's current position (the robot turns in place)
        self._hits.append(rot(obs.x, obs.y, yaw_c) if self._bias_done else (obs.x, obs.y))
    if self.state == State.MEASURE and self.phase == "collect" and obs.stamp >= self._t_phase:
      self._samples.append((x, y))
    if self._ball is None or now - self._t_fresh > p.max_obs_age:
      self._ball = (x, y)
    else:
      g = p.obs_gain
      self._ball = (self._ball[0] + g * (x - self._ball[0]), self._ball[1] + g * (y - self._ball[1]))
    self._t_fresh = obs.stamp

  # ------------------------------------------------------------------ states

  def _search(self, now: float, dt: float) -> Command:
    p = self.params
    if self.phase == "backoff":
      vx, vy, t_end = self._step
      if now < t_end:
        return Command(vx, vy, 0.0)
      self._hits = []
      self._set_phase("look", now)
      return ZERO
    if self.phase == "turn":
      turned = abs(self._yaw - self._turn_from)
      if turned >= math.radians(p.scan_step_deg) or (self._seen_in_turn and turned > 0.05):
        self._scan_turned += turned
        self._hits, self._seen_in_turn = [], False
        self._set_phase("look", now)
        return ZERO
      if now - self._t_phase > math.radians(p.scan_step_deg) / (0.25 * p.scan_wz):
        return self._abort_cmd(now, "scan turn timeout (is the IMU yaw updating?)")
      return Command(0.0, 0.0, math.copysign(p.scan_wz, p.scan_dir))

    first = self._looks == 0 and self._reacquire == 0
    if now - self._t_phase < (max(p.look_s, p.bias_s) if first else p.look_s):
      return ZERO
    self._looks += 1
    if not self._bias_done:
      self._bias_done = True
      if self._yaw_t0 is not None:
        self._bias = slope(self._bias_samples)
        self._yaw = self._yaw_unwrapped - self._bias * (now - self._yaw_t0)
        self.info["imu_drift_dps"] = round(math.degrees(self._bias), 2)
        if abs(math.degrees(self._bias)) > p.max_bias_dps:
          return self._abort_cmd(
            now, f"IMU yaw drifts {math.degrees(self._bias):.1f} deg/s at rest (limit {p.max_bias_dps})")
      else:
        self.info["imu"] = "none: yaw from commanded turns"
    found = cluster(self._hits, p.hit_spread, int(p.min_hits))
    if found is None:
      if self._scan_turned >= math.radians(p.scan_max_deg):
        return self._abort_cmd(now, "no ball found after a full scan (start 0.8 to 3 m from the ball)")
      self._turn_from = self._yaw
      self._set_phase("turn", now)
      return Command(0.0, 0.0, math.copysign(p.scan_wz, p.scan_dir))
    bx, by = rot(found[0], found[1], -self._yaw)
    self._ball, self._t_fresh = (bx, by), now
    if self._aim is None and not self._plan(now):
      if self.active:
        self._set_phase("look", now)  # topic mode: wait for a goal
        self._hits = []
      return ZERO
    self._enter(State.APPROACH, now)
    return ZERO

  def _aim_from_goal(self) -> float | None:
    p = self.params
    bx, by = self._ball
    if p.goal_frame == "ball":
      return math.atan2(p.target_y, p.target_x)
    if p.goal_frame == "start":
      wx, wy = rot(bx, by, self._yaw)
      wx, wy = wx + self._pos[0], wy + self._pos[1]
      return math.atan2(p.target_y - wy, p.target_x - wx)
    if self._goal is None:
      return None
    return wrap(self._yaw + math.atan2(self._goal[1] - by, self._goal[0] - bx))

  def _plan(self, now: float) -> bool:
    """Aim, side and kick heading at the first sighting. False: no goal yet."""
    p = self.params
    aim = self._aim_from_goal()
    if aim is None:
      if now - self._t_state > 10.0:
        self._abort(now, "no goal on the goal topic")
      return False
    self._aim = aim
    if p.side in SIDES:
      self.side = p.side
    else:
      dist = {}
      for side in SIDES:
        h = wrap(aim - self._bias_rad(side) - self._yaw)
        ox, oy = rot(p.stage_x, self._sy(side), h)
        dist[side] = math.hypot(self._ball[0] - ox, self._ball[1] - oy)
      self.side = "left" if dist["left"] < dist["right"] - 0.03 else "right"
    self._heading = wrap(aim - self._bias_rad(self.side))
    wx, wy = rot(self._ball[0], self._ball[1], self._yaw)
    self.info.update(ball_w=[round(wx + self._pos[0], 3), round(wy + self._pos[1], 3)],
                     aim_deg=round(math.degrees(aim), 1),
                     heading_deg=round(math.degrees(self._heading), 1))
    return True

  def _update_topic_aim(self) -> None:
    if self.params.goal_frame == "topic" and self._goal is not None and self._ball is not None:
      aim = self._aim_from_goal()
      self._aim = aim
      self._heading = wrap(aim - self._bias_rad(self.side))

  def _lost(self, now: float, reason: str) -> Command:
    p = self.params
    if self._reacquire >= int(p.max_reacquire):
      return self._abort_cmd(now, reason)
    self._reacquire += 1
    self.info["reacquired"] = self._reacquire
    self._step = None
    if self._ball is not None:
      bx, by = self._ball
      rng = math.hypot(bx, by)
      if 0.0 < rng < p.blind_range + 0.05:  # too close for the camera: step away first
        speed = 0.1
        self._step = (-speed * bx / rng, -speed * by / rng,
                      now + (p.blind_range + 0.15 - rng) / (speed * p.speed_efficiency))
    self._hits, self._scan_turned, self._ball = [], 0.0, None
    self._enter(State.SEARCH, now, "backoff" if self._step else "look")
    return ZERO

  def _approach(self, now: float, dt: float) -> Command:
    p = self.params
    if now - self._t_state > p.approach_timeout:
      return self._abort_cmd(now, "approach timeout")
    if self._travel > p.approach_travel_max:
      return self._abort_cmd(now, "approach travel budget used (is the ball estimate fresh?)")
    if now - self._t_fresh > p.max_obs_age:
      if now - self._t_fresh > p.lost_timeout:
        return self._lost(now, "ball lost while approaching")
      return ZERO  # stale track: zero travel
    self._update_topic_aim()
    sy = self._sy()
    h = self._h()
    bx, by = self._ball
    ox, oy = rot(p.stage_x, sy, h)
    tx, ty = bx - ox, by - oy
    if math.hypot(tx, ty) < p.coarse_pos_tol and abs(h) < math.radians(p.coarse_yaw_tol_deg):
      self._enter(State.MEASURE, now, "settle")
      return ZERO
    if segment_distance(bx, by, tx, ty) < p.keep_out:  # ball in the way: orbit it
      radius = p.keep_out + 0.15
      th_robot = math.atan2(-by, -bx)  # robot as seen from the ball
      th_goal = math.atan2(ty - by, tx - bx)  # staging point as seen from the ball
      th = th_robot + clip(wrap(th_goal - th_robot), math.pi / 4)
      tx, ty = bx + radius * math.cos(th), by + radius * math.sin(th)
    vx, vy = p.approach_kp * tx, p.approach_kp * ty
    norm = math.hypot(vx, vy)
    if norm > p.approach_speed:
      vx, vy = vx * p.approach_speed / norm, vy * p.approach_speed / norm
    elif 0.0 < norm < p.min_speed:
      vx, vy = vx * p.min_speed / norm, vy * p.min_speed / norm
    rng = math.hypot(bx, by)
    if rng < p.keep_out:  # inside keep_out (the orbit cuts corners): push away from the ball
      push = 2.0 * p.approach_kp * (p.keep_out - rng)
      vx, vy = vx - push * bx / rng, vy - push * by / rng
    # Face the ball, offset so that the heading is the kick heading at the staging pose.
    wz = p.yaw_kp * wrap(math.atan2(by, bx) - math.atan2(sy, p.stage_x))
    return Command(vx, clip(vy, p.max_vy), clip(wz, p.max_wz))

  def _measure(self, now: float, dt: float) -> Command:
    p = self.params
    if self.phase == "settle":
      if now - self._t_phase >= p.settle_s:
        self._samples = []
        self._set_phase("collect", now)
      return ZERO
    n, age = len(self._samples), now - self._t_phase
    if age < p.measure_s or (n < p.min_measure_obs and age < p.measure_s + 1.5):
      return ZERO
    if n < p.min_measure_obs:
      return self._lost(now, f"ball not seen at the staging pose ({n} detections in {age:.1f} s)")
    self._ball = (sum(s[0] for s in self._samples) / n, sum(s[1] for s in self._samples) / n)
    self._t_fresh = now
    self._update_topic_aim()
    kx, ky, h = self._in_kick_frame()
    ex, ey = kx - p.stage_x, ky - self._sy()
    self.info.update(measure=[round(ex, 3), round(ey, 3), round(math.degrees(h), 2)],
                     iters=self._iters, n_obs=n)
    x_ok = abs(ex) <= p.stage_x_tol
    if x_ok and abs(ey) <= p.fine_lat_tol and abs(h) <= math.radians(p.fine_yaw_tol_deg):
      return self._begin_creep(now)
    if self._iters >= int(p.fine_max_iters):
      if x_ok and abs(ey) <= p.accept_lat_tol and abs(h) <= math.radians(p.accept_yaw_tol_deg):
        self.info["fine_tolerance_met"] = False
        return self._begin_creep(now)
      return self._abort_cmd(now, f"could not line up in {self._iters} corrections "
                                  f"(x {ex:+.3f} m, y {ey:+.3f} m, yaw {math.degrees(h):+.1f} deg)")
    self._iters += 1
    self._enter(State.CORRECT, now, "turn")
    return self._correct(now, 0.0)

  def _correct(self, now: float, dt: float) -> Command:
    p = self.params
    h = self._h()
    if self.phase == "turn":
      if abs(h) > 0.5 * math.radians(p.fine_yaw_tol_deg) and now - self._t_phase < p.corr_turn_timeout:
        wz = clip(p.yaw_kp * h, p.corr_wz)
        if abs(wz) < p.corr_min_wz:
          wz = math.copysign(p.corr_min_wz, h)
        return Command(0.0, 0.0, wz)
      kx, ky, h = self._in_kick_frame()
      ex, ey = kx - p.stage_x, ky - self._sy()
      if abs(ex) <= p.stage_x_tol:
        ex = 0.0  # the creep length takes up an x error; a step would add sideways error
      tx, ty = rot(ex, ey, h)  # robot displacement that puts the ball at the staging point
      d = math.hypot(tx, ty)
      if d < 0.004:
        self._enter(State.MEASURE, now, "settle")
        return ZERO
      self._step = (p.corr_speed * tx / d, p.corr_speed * ty / d,
                    now + d / (p.corr_speed * p.creep_efficiency))
      self._set_phase("step", now)
    vx, vy, t_end = self._step
    if now >= t_end:
      self._enter(State.MEASURE, now, "settle")
      return ZERO
    return Command(vx, vy, clip(p.yaw_kp * h, p.corr_wz))

  def _begin_creep(self, now: float) -> Command:
    p = self.params
    kx, _, _ = self._in_kick_frame()
    self._creep_budget = kx - p.strike_x + p.creep_margin
    self._vx, self._lat_i = 0.0, 0.0
    self._enter(State.CREEP, now)
    return self._creep(now, 0.0)

  def _creep(self, now: float, dt: float) -> Command:
    p = self.params
    if now - self._t_state > p.creep_timeout:
      return self._abort_cmd(now, "creep timeout")
    if self._travel > self._creep_budget:
      return self._abort_cmd(now, "creep travel budget used")
    kx, ky, h = self._in_kick_frame()
    ey = ky - self._sy()
    coast = self._v[0] * p.creep_efficiency * p.walker_lag_s  # travel after a zero command
    if kx - coast <= p.strike_x:
      self._enter(State.SETTLE, now)
      return ZERO
    self._vx = min(p.creep_speed, self._vx + p.creep_accel * dt)
    seen = now - self._t_fresh <= p.max_obs_age
    vy = 0.0
    if seen:  # ball still visible: feedback, and learn the pull for the blind part
      vy = p.creep_lat_kp * ey
      if p.creep_lat_ki > 0.0:
        self._lat_i = clip(self._lat_i + ey * dt, 0.5 * p.max_vy / p.creep_lat_ki)
    vy += p.creep_lat_ki * self._lat_i - p.walk_drift * self._vx
    return Command(self._vx, clip(vy, p.max_vy), clip(p.creep_yaw_kp * h, p.max_wz))

  def _settle(self, now: float, dt: float) -> Command:
    p = self.params
    if now - self._t_state < p.pre_kick_settle_s:
      return ZERO
    kx, ky, h = self._in_kick_frame()
    ex, ey = kx - p.strike_x, ky - self._sy()
    self.info["placement"] = [round(ex, 3), round(ey, 3), round(math.degrees(h), 2)]
    if self._near:
      self.info["near_detections"] = self._near[-5:]
    if abs(ex) > p.strike_x_tol or abs(ey) > p.accept_lat_tol \
        or abs(h) > math.radians(p.accept_yaw_tol_deg):
      return self._abort_cmd(now, f"placement outside tolerance (x {ex:+.3f} m, y {ey:+.3f} m, "
                                  f"yaw {math.degrees(h):+.1f} deg)")
    self._strike_yaw, self._spot = self._yaw, self._ball
    if p.strike == "none":
      self.info["dry_run"] = True
      self._enter(State.DONE, now)
      return ZERO
    self.strike.start(now, self.side)
    self._enter(State.STRIKE, now, self.strike.phase)
    return ZERO

  def _strike_tick(self, now: float, dt: float) -> Command:
    self.strike.tick(now)
    self.phase = self.strike.phase
    if self.strike.finished:
      if self.strike.ok:
        self._best = None
        self._enter(State.OBSERVE, now)
      else:
        self._abort(now, self.strike.reason)
    return ZERO

  def _record_shot(self, obs: BallObs, yaw_c: float) -> None:
    x, y = rot(obs.x, obs.y, yaw_c - self._strike_yaw)  # strike frame
    dx, dy = x - self._spot[0], y - self._spot[1]
    d = math.hypot(dx, dy)
    if self._best is None or d > self._best[0]:
      self._best = (d, dx, dy)

  def _observe_tick(self, now: float, dt: float) -> Command:
    p = self.params
    if now - self._t_state < p.observe_s:
      return ZERO
    if self._best is not None and self._best[0] >= p.min_shot_travel:
      d, dx, dy = self._best
      intended = wrap(self._aim - self._strike_yaw)
      self.info.update(shot_travel_m=round(d, 2),
                       shot_err_deg=round(math.degrees(wrap(math.atan2(dy, dx) - intended)), 1))
    else:
      self.info["shot"] = "ball not seen moving (missed, or out of the camera's range)"
    self._enter(State.DONE, now)
    return ZERO
