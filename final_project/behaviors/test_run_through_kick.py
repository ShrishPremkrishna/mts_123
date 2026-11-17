"""Run-through kick controller against a kinematic walker and camera model.

The stand-in walker tracks the commanded velocity with a first-order lag at 80%
efficiency; the camera reports the ball at 10 Hz, 0.1 s late, with range noise,
and not at all inside 0.40 m. This checks the controller logic, not physics.

    python3 -m pytest final_project/behaviors/test_run_through_kick.py -q
"""

from __future__ import annotations

import math
import random

import pytest

from run_through_kick import (
  ZERO,
  BallObs,
  Command,
  KickParams,
  RunThroughKick,
  State,
  aim_from_goal,
  field_to_robot,
  wrap,
)

DT = 0.05  # controller tick (20 Hz, as the node runs)
SUB = 5  # plant substeps per tick
FRONT_X = 0.151  # chassis front in base_link [m] (no bumper)
HALF_W = 0.057  # chassis half-width [m]
BALL_R = 0.075


class World:
  def __init__(self, ball, robot=(0.0, 0.0, 0.0), *, latency=0.1, rate=10.0,
               efficiency=0.8, tau=0.15, noise=0.01, blind=0.40, seed=0,
               goal_latency=None):
    self.bx, self.by = ball
    self.x, self.y, self.yaw = robot
    self.v = [0.0, 0.0, 0.0]
    self.t = 0.0
    self.latency, self.period = latency, 1.0 / rate
    self.eff, self.tau, self.noise, self.blind = efficiency, tau, noise, blind
    self.rng = random.Random(seed)
    self.frames = []  # (capture time, x, y) not yet delivered
    self.next_frame = 0.0
    self.latest = None
    self.goal_world, self.latest_goal = None, None
    self.goal_frames = []
    self.goal_latency = latency if goal_latency is None else goal_latency
    self.contact = None  # dict once the body front reaches the ball
    self.min_clearance = float("inf")

  def ball_in_robot(self):
    dx, dy = self.bx - self.x, self.by - self.y
    c, s = math.cos(self.yaw), math.sin(self.yaw)
    return c * dx + s * dy, -s * dx + c * dy

  def advance(self, cmd, aligning):
    h = DT / SUB
    for _ in range(SUB):
      a = h / self.tau
      target = (cmd.vx * self.eff, cmd.vy * self.eff, cmd.wz * self.eff)
      self.v = [v + a * (g - v) for v, g in zip(self.v, target)]
      c, s = math.cos(self.yaw), math.sin(self.yaw)
      self.x += (c * self.v[0] - s * self.v[1]) * h
      self.y += (s * self.v[0] + c * self.v[1]) * h
      self.yaw = wrap(self.yaw + self.v[2] * h)
      self.t += h
      rx, ry = self.ball_in_robot()
      if aligning:
        self.min_clearance = min(self.min_clearance, math.hypot(rx, ry))
      hit = rx - BALL_R <= FRONT_X and rx > 0 and abs(ry) <= HALF_W + BALL_R
      if self.contact is None and hit:
        self.contact = dict(speed=self.v[0], lateral=ry, yaw=self.yaw, t=self.t)
    # Camera: capture on a fixed clock, deliver after the latency.
    while self.t >= self.next_frame:
      rx, ry = self.ball_in_robot()
      rng = math.hypot(rx, ry)
      if rng >= self.blind and rx > 0 and self.contact is None:
        sd = self.noise + 0.02 * rng
        self.frames.append((self.next_frame, rx + self.rng.gauss(0, sd), ry + self.rng.gauss(0, sd)))
      if self.goal_world is not None:
        gx, gy = field_to_robot(self.x, self.y, self.yaw, *self.goal_world)
        self.goal_frames.append(BallObs(gx, gy, self.next_frame))
      self.next_frame += self.period
    while self.frames and self.frames[0][0] + self.latency <= self.t:
      stamp, fx, fy = self.frames.pop(0)
      self.latest = BallObs(fx, fy, stamp)
    while self.goal_frames and self.goal_frames[0].stamp + self.goal_latency <= self.t:
      self.latest_goal = self.goal_frames.pop(0)


def run(world, ctrl, goal=None, max_time=25.0, drop_after=None, log=None):
  """Drive the controller until it finishes; ``goal`` is in world coordinates."""
  world.goal_world = goal
  ctrl.start(world.t, require_goal=goal is not None)
  while ctrl.active and world.t < max_time:
    obs = world.latest
    if drop_after is not None and world.t > drop_after:
      obs = None
    cmd = ctrl.step(world.t, obs, goal=world.latest_goal)
    if log is not None:
      log.append((ctrl.state, cmd))
    world.advance(cmd, ctrl.state == State.ALIGN)
  return world


def test_straight_kick_runs_through_the_ball():
  w = run(World(ball=(1.2, 0.05)), RunThroughKick())
  assert w.contact is not None
  assert w.contact["speed"] > 0.6  # 1.0 m/s commanded at 80% efficiency
  assert abs(w.contact["lateral"]) < 0.03


@pytest.mark.parametrize("side", [-1.0, 1.0])
def test_charge_heading_follows_recentered_camera_ball(side):
  ctrl = RunThroughKick()
  ctrl.start(0.0)
  first = ctrl.step(0.0, BallObs(0.8, side * 0.03, 0.0))
  assert ctrl.state == State.CHARGE
  assert side * first.wz > 0.0

  centered = ctrl.step(0.05, BallObs(0.65, 0.0, 0.05))
  assert ctrl.state == State.CHARGE
  assert centered.vy == 0.0 and centered.wz == 0.0
  crossed = ctrl.step(0.1, BallObs(0.60, -side * 0.02, 0.1))
  assert side * crossed.vy < 0.0 and side * crossed.wz < 0.0

  # The blind-zone transition still drops lateral and heading correction.
  coast = ctrl.step(0.15, BallObs(0.35, side * 0.02, 0.15))
  assert ctrl.state == State.COAST
  assert coast.vx > 0.0 and coast.vy == 0.0 and coast.wz == 0.0


def test_charge_explicit_aim_overrides_camera_ball_bearing():
  ctrl = RunThroughKick()
  ctrl.start(0.0)
  ctrl.step(0.0, BallObs(0.8, 0.0, 0.0), aim=0.02)
  assert ctrl.state == State.CHARGE
  cmd = ctrl.step(0.05, BallObs(0.65, -0.02, 0.05), aim=0.02)
  assert cmd.vy < 0.0  # center the ball while preserving the separate goal aim
  assert cmd.wz == pytest.approx(ctrl.params.yaw_kp * 0.02)
  # If the goal expires, None again means straight through the current ball.
  fallback = ctrl.step(0.1, BallObs(0.60, -0.02, 0.1))
  assert fallback.wz < 0.0


def test_manual_charge_without_explicit_aim_reduces_heading_correction():
  ctrl = RunThroughKick()
  ctrl.start(0.0, static_ball=(0.8, 0.03))
  first = ctrl.step(0.0, None)
  assert ctrl.state == State.CHARGE
  later = ctrl.step(0.1, None)
  assert 0.0 < later.wz < first.wz  # commanded motion recenters its ball estimate


@pytest.mark.parametrize("motion", ["translation", "turn", "turn_then_translation"])
def test_goal_and_ball_from_different_epochs_agree_with_world_geometry(motion):
  ctrl = RunThroughKick(KickParams(speed_efficiency=1.0))
  ctrl.start(1.0, require_goal=True)
  # Known commanded motion, independent analytic world geometry. Captures are
  # at t=1.0 (ball) and t=1.2 (goal), delivery at t=1.4.
  if motion == "translation":
    ctrl._history.append((1.0, Command(vx=0.2)))
    goal = BallObs(2.96, 1.0, 1.2)
    current_ball, expected_aim = (0.92, 0.0), math.atan2(1, 2)
  else:
    ctrl._history.append((1.0, Command(wz=0.5)))
    gx, gy = field_to_robot(0, 0, 0.1, 3, 1)
    goal = BallObs(gx, gy, 1.2)
    if motion == "turn":
      current_ball = (math.cos(0.2), -math.sin(0.2))
      expected_aim = math.atan2(1, 2) - 0.2
    else:
      ctrl._history.append((1.2, Command(vx=0.2)))
      current_ball = (math.cos(0.1) - 0.04, -math.sin(0.1))
      expected_aim = math.atan2(1, 2) - 0.1
  ctrl.step(1.4, BallObs(1, 0, 1.0), goal=goal)
  assert ctrl._ball == pytest.approx(current_ball, abs=1e-12)
  assert ctrl._aim == pytest.approx(expected_aim, abs=1e-12)


def test_required_goal_loss_pauses_alignment_then_aborts_without_changing_intent():
  ctrl = RunThroughKick(KickParams(goal_max_age=0.1, lost_timeout=0.2))
  ctrl.start(1.0, require_goal=True)
  assert ctrl.step(1.0, BallObs(1.2, 0, 1.0)) == ZERO
  assert ctrl.step(1.05, BallObs(1.2, 0, 1.05), goal=BallObs(3, 0.4, 1.05)).vx > 0
  assert ctrl.step(1.2, BallObs(1.2, 0, 1.2), goal=BallObs(3, 0.4, 1.05)) == ZERO
  assert ctrl.state == State.ALIGN
  assert ctrl.step(1.3, BallObs(1.2, 0, 1.3), goal=BallObs(3, 0.4, 1.05)) == ZERO
  assert ctrl.state == State.ABORTED and "goal lost" in ctrl.reason


@pytest.mark.parametrize("ball_x", [0.65, 0.35])
def test_goal_intent_latches_and_loss_during_charge_aborts_even_with_fresh_ball(ball_x):
  ctrl = RunThroughKick()
  ctrl.start(1.0)
  ctrl.step(1.0, BallObs(0.8, 0, 1.0), goal=BallObs(3, 0, 1.0))
  assert ctrl.state == State.CHARGE
  assert ctrl.step(1.05, BallObs(ball_x, 0, 1.05)) == ZERO
  assert ctrl.state == State.ABORTED and "goal lost" in ctrl.reason


def test_captures_outside_known_motion_history_or_in_future_are_not_used():
  ctrl = RunThroughKick()
  ctrl.start(2.0)
  assert ctrl.step(2.0, BallObs(1.2, 0, 1.9)) == ZERO
  assert ctrl.step(2.05, BallObs(1.2, 0, 2.1)) == ZERO
  assert ctrl._ball is None
  assert ctrl.step(2.1, BallObs(1.2, 0, 2.1)).vx > 0


def test_camera_clock_regression_aborts_instead_of_reusing_an_old_epoch():
  ctrl = RunThroughKick()
  ctrl.start(2.0)
  assert ctrl.step(2.0, BallObs(1.2, 0, 2.0)).vx > 0
  assert ctrl.step(1.9, BallObs(1.2, 0, 1.9)) == ZERO
  assert ctrl.state == State.ABORTED and "clock moved backwards" in ctrl.reason


def test_manual_inputs_ignore_stamped_goal_and_required_goal_flag():
  ordinary, extra_inputs = RunThroughKick(), RunThroughKick()
  ordinary.start(1.0, static_ball=(1.1, 0.1), static_aim=0.03)
  extra_inputs.start(1.0, static_ball=(1.1, 0.1), static_aim=0.03, require_goal=True)
  for tick in range(100):
    now = 1.0 + tick * DT
    assert extra_inputs.step(now, BallObs(3, 2, now), goal=BallObs(0, 3, now)) == ordinary.step(now, None)
    assert extra_inputs.state == ordinary.state


@pytest.mark.parametrize("side", [-1, 1])
def test_stock_goal_shot_with_different_camera_delays(side):
  ball, goal = (1.1, side * 0.15), (4.0, -side * 0.4)
  ctrl = RunThroughKick(KickParams(charge_speed=0.45, align_speed=0.3))
  w = run(World(ball=ball, latency=0.1, goal_latency=0.25, seed=3), ctrl, goal=goal)
  assert w.contact is not None, ctrl.reason
  assert abs(w.contact["lateral"]) < 0.05
  expected = math.atan2(goal[1] - ball[1], goal[0] - ball[0])
  assert abs(wrap(w.contact["yaw"] - expected)) < math.radians(5)


@pytest.mark.parametrize("seed", range(5))
def test_aimed_kick_lines_up_on_the_goal(seed):
  ball, goal = (1.0, 0.3), (4.0, -0.5)
  w = run(World(ball=ball, seed=seed), RunThroughKick(), goal=goal)
  ctrl_aim = math.atan2(goal[1] - ball[1], goal[0] - ball[0])
  assert w.contact is not None
  assert abs(wrap(w.contact["yaw"] - ctrl_aim)) < math.radians(4.0)
  assert abs(w.contact["lateral"]) < 0.03
  assert w.contact["speed"] > 0.6


def test_passes_a_ball_that_blocks_the_run_up():
  # Aim points back past the robot: the run-up point is behind the ball.
  ctrl = RunThroughKick()
  w = run(World(ball=(0.5, 0.0)), ctrl, goal=(-3.0, 0.0))
  assert w.contact is not None, ctrl.reason
  assert abs(wrap(w.contact["yaw"] - math.pi)) < math.radians(5.0)
  # The robot starts 0.5 m away; the detour must keep the ball outside the
  # camera blind zone (so it is never lost) and untouched before the charge.
  assert w.min_clearance > ctrl.params.blind_range


def test_stale_track_requests_zero_then_aborts():
  w = World(ball=(1.0, 0.0))
  ctrl = RunThroughKick()
  log = []
  run(w, ctrl, drop_after=0.0, log=log)
  assert ctrl.state == State.ABORTED and "lost" in ctrl.reason
  assert all(cmd == ZERO for _, cmd in log)


def test_ball_lost_far_away_during_the_charge_aborts():
  w = World(ball=(2.5, 0.0))
  ctrl = RunThroughKick(KickParams(run_up=1.5))
  log = []
  # Drop the camera shortly after the charge starts (ball still ~1.4 m away).
  ctrl.start(w.t)
  t_drop = None
  while ctrl.active and w.t < 25.0:
    obs = None if (t_drop is not None and w.t > t_drop) else w.latest
    cmd = ctrl.step(w.t, obs)
    log.append((ctrl.state, cmd))
    if ctrl.state == State.CHARGE and t_drop is None:
      t_drop = w.t + 0.1
    w.advance(cmd, ctrl.state == State.ALIGN)
  assert ctrl.state == State.ABORTED and "before the blind zone" in ctrl.reason
  assert log[-1][1] == ZERO
  assert w.contact is None


def test_stop_returns_zero_and_aborts():
  ctrl = RunThroughKick()
  ctrl.start(0.0)
  ctrl.step(0.0, BallObs(1.0, 0.0, 0.0))
  assert ctrl.stop(0.05) == ZERO
  assert ctrl.state == State.ABORTED and not ctrl.active
  assert ctrl.step(0.1, BallObs(1.0, 0.0, 0.1)) == ZERO


def test_commands_stay_inside_the_walker_limits():
  p = KickParams()
  for seed in range(3):
    log = []
    run(World(ball=(1.3, -0.4), seed=seed), RunThroughKick(p), goal=(4.0, 1.0), log=log)
    for state, cmd in log:
      assert abs(cmd.vy) <= p.max_vy + 1e-9 and abs(cmd.wz) <= p.max_wz + 1e-9
      assert cmd.vx <= p.charge_speed + 1e-9
      if state == State.ALIGN:
        assert math.hypot(cmd.vx, cmd.vy) <= p.align_speed + 1e-9


def test_latency_compensation_moves_the_estimate_to_now():
  ctrl = RunThroughKick(KickParams(speed_efficiency=1.0))
  ctrl.start(0.0)
  ctrl.step(0.0, BallObs(2.0, 0.0, 0.0))  # first command
  first = ctrl._last
  ctrl.step(0.2, BallObs(2.0, 0.0, 0.0))  # same frame, 0.2 s old
  # The robot moved first.vx * 0.2 toward the ball since the capture.
  assert ctrl._ball[0] == pytest.approx(2.0 - first.vx * 0.2, abs=1e-9)


def run_manual(world, ctrl, ball, aim=None, max_time=15.0):
  """Manual mode: one measured ball position, no camera."""
  ctrl.start(world.t, static_ball=ball, static_aim=aim)
  while ctrl.active and world.t < max_time:
    world.advance(ctrl.step(world.t, None), ctrl.state == State.ALIGN)
  return world


@pytest.mark.parametrize("efficiency", [0.8, 0.7])
def test_manual_mode_kicks_without_a_camera(efficiency):
  # The controller assumes 80%; at 70% the 0.25 m overrun must still carry it through.
  ctrl = RunThroughKick()
  w = run_manual(World(ball=(1.0, 0.0), efficiency=efficiency), ctrl, (1.0, 0.0))
  assert ctrl.state == State.DONE, ctrl.reason
  assert w.contact is not None
  assert w.contact["speed"] > 0.5 and abs(w.contact["lateral"]) < 0.04


def test_manual_mode_holds_the_aim_in_the_world():
  ctrl = RunThroughKick()
  # Ball ahead and left; kick it straight along the robot's start heading (+x).
  w = run_manual(World(ball=(1.0, 0.3)), ctrl, (1.0, 0.3), aim=0.0)
  assert w.contact is not None, ctrl.reason
  assert abs(wrap(w.contact["yaw"])) < math.radians(6.0)
  assert abs(w.contact["lateral"]) < 0.05


def test_field_to_robot():
  # Robot at (1, 2) facing +y (90 deg): a point 1 m further along +y is dead ahead.
  assert field_to_robot(1.0, 2.0, math.pi / 2, 1.0, 3.0) == pytest.approx((1.0, 0.0))
  assert field_to_robot(1.0, 2.0, math.pi / 2, 0.0, 2.0) == pytest.approx((0.0, 1.0))
