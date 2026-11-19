#!/usr/bin/env python3
"""Kinematic stand-in world for the penalty kick (offline checks, no ROS).

Per attempt (seeded):
  walker  first-order velocity lag, efficiency, sideways drift per m forward,
          turn efficiency. No gait physics.
  camera  15 Hz, latency 0.13 +- 0.03 s, 10 % dropouts, noise that grows with
          range, an optional lateral bias. It sees the ball centre at ranges
          0.45 to 3.0 m and |bearing| <= 100 deg.
  IMU     yaw + constant drift + noise.
  slot    answers switches after 0.05 s, publishes clip status at 50 Hz while
          active, starts a requested clip 0.5 s later; the kick clip is 3.0 s.
  shot    direction = heading + shot bias + 7 deg per cm of lateral ball offset
          from the strike point (leg-kick sim run 2: +-1.4 cm gave -12.5 to
          +6.5 deg about -3.4 deg). The ball then rolls 1.2 m.

It checks the controller logic and its tolerance to walker and sensor errors.
It does not model gait dynamics, the ball contact or the real detector.

    python3 penalty_sim.py --runs 300            # calibrated walker model
    python3 penalty_sim.py --runs 300 --uncalibrated
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass, replace

from penalty_kick import PenaltyKick, PenaltyParams, State, rot
from run_through_kick import BallObs, wrap

DT = 0.02  # s, the node's 50 Hz tick


@dataclass
class WorldCfg:
  eff: float = 0.8  # true achieved / commanded speed
  eff_wz: float = 0.9  # true achieved / commanded turn rate
  lag_s: float = 0.15
  drift: float = 0.0  # m left per m forward
  cam_hz: float = 15.0
  latency: float = 0.13
  latency_sd: float = 0.03
  dropout: float = 0.1
  noise_x: float = 0.008  # sd per m of range (plus 0.003 m)
  noise_y: float = 0.003  # sd per m of range (plus 0.002 m)
  bias_y: float = 0.0  # m, detector lateral bias (calibration error)
  fov_deg: float = 100.0
  min_range: float = 0.45
  max_range: float = 3.0
  imu_drift_dps: float = 0.0
  imu_noise: float = 0.002  # rad
  lat_slope_deg_per_cm: float = 7.0
  switch_ok: bool = True
  clip_plays: bool = True
  ball_visible: bool = True


class World:
  def __init__(self, cfg: WorldCfg, robot=(0.0, 0.0, 0.0), ball=(1.5, 0.0), seed: int = 0,
               params: PenaltyParams | None = None):
    self.cfg, self.rng = cfg, random.Random(seed)
    self.params = params or PenaltyParams()
    self.x, self.y, self.psi = robot
    self.bx, self.by = ball
    self.v = [0.0, 0.0, 0.0]
    self.t = 0.0
    self.next_frame = self.rng.uniform(0.0, 1.0 / cfg.cam_hz)
    self.queue: list[tuple[float, BallObs]] = []
    self.latest = None
    self.answers: list[tuple[float, str, bool]] = []
    self.slot_active = False
    self.clip_req = None  # (name, start time)
    self.estopped = False
    self.kick = None  # true placement and shot at the kick clip start
    self.min_ball_dist = math.inf  # before the creep

  def ball_in_robot(self) -> tuple[float, float]:
    return rot(self.bx - self.x, self.by - self.y, -self.psi)

  def imu_yaw(self) -> float:
    c = self.cfg
    return wrap(self.psi + math.radians(c.imu_drift_dps) * self.t + self.rng.gauss(0.0, c.imu_noise))

  def advance(self, cmd, dt: float = DT) -> None:
    c = self.cfg
    a = 1.0 - math.exp(-dt / c.lag_s) if c.lag_s > 0 else 1.0
    for i, target in enumerate((cmd.vx, cmd.vy, cmd.wz)):
      if self.slot_active:
        target = 0.0  # the walker is switched out
      self.v[i] += (target - self.v[i]) * a
    vx, vy = self.v[0] * c.eff, (self.v[1] + c.drift * self.v[0]) * c.eff
    wx, wy = rot(vx, vy, self.psi)
    self.x, self.y = self.x + wx * dt, self.y + wy * dt
    self.psi = wrap(self.psi + self.v[2] * c.eff_wz * dt)
    self.t += dt
    while self.t >= self.next_frame:
      self._capture(self.next_frame)
      self.next_frame += 1.0 / c.cam_hz

  def _capture(self, tc: float) -> None:
    c = self.cfg
    bx, by = self.ball_in_robot()
    rng = math.hypot(bx, by)
    if not (c.ball_visible and c.min_range <= rng <= c.max_range
            and abs(math.atan2(by, bx)) <= math.radians(c.fov_deg)):
      return
    if self.rng.random() < c.dropout:
      return
    ox = bx + self.rng.gauss(0.0, 0.003 + c.noise_x * rng)
    oy = by + c.bias_y + self.rng.gauss(0.0, 0.002 + c.noise_y * rng)
    delay = max(0.05, self.rng.gauss(c.latency, c.latency_sd))
    self.queue.append((tc + delay, BallObs(ox, oy, tc)))

  def sense(self):
    due = [q for q in self.queue if q[0] <= self.t]
    self.queue = [q for q in self.queue if q[0] > self.t]
    for _, obs in due:
      if self.latest is None or obs.stamp > self.latest.stamp:
        self.latest = obs
    return self.latest

  # ----------------------------------------------------------- kick slot

  def handle(self, actions) -> None:
    for a in actions:
      if a[0] == "switch":
        self.answers.append((self.t + 0.05, a[1], self.cfg.switch_ok))
      elif a[0] == "clip":
        self.clip_req = (a[1], self.t + 0.5) if self.cfg.clip_plays else None
      elif a[0] == "estop":
        self.estopped = True

  def events(self):
    out = []
    for item in [a for a in self.answers if a[0] <= self.t]:
      self.answers.remove(item)
      _, which, ok = item
      if ok:
        self.slot_active = which == "in"
      out.append(("switch", which, ok))
    if self.slot_active:
      clip_name = "stand_idle"
      if self.clip_req is not None and self.t >= self.clip_req[1]:
        if self.t < self.clip_req[1] + 3.0:
          clip_name = self.clip_req[0]
          if self.kick is None:
            self._shoot(clip_name.split("_")[-1])
        else:
          self.clip_req = None
      out.append(("status", clip_name))
      out.append(("joints", 0.3))
    return out

  def _shoot(self, side: str) -> None:
    p = self.params
    bx, by = self.ball_in_robot()
    sy = p.strike_y_right if side == "right" else p.strike_y_left
    bias = p.shot_bias_right_deg if side == "right" else p.shot_bias_left_deg
    ey = by - sy
    shot = self.psi + math.radians(bias + self.cfg.lat_slope_deg_per_cm * ey * 100.0)
    self.kick = dict(side=side, ex=bx - p.strike_x, ey=ey, shot=shot)
    self.bx += 1.2 * math.cos(shot)
    self.by += 1.2 * math.sin(shot)


def run_attempt(params: PenaltyParams, cfg: WorldCfg, robot, ball, target, seed: int = 0,
                max_t: float = 120.0, stop_at: float | None = None) -> dict:
  """One attempt. ``target``: the goal point in the world (sets goal_frame ball params)."""
  x0, y0, psi0 = robot
  tx, ty = rot(target[0] - ball[0], target[1] - ball[1], -psi0)
  if params.goal_frame == "ball":
    params = replace(params, target_x=tx, target_y=ty)
  elif params.goal_frame == "start":
    sx, sy = rot(target[0] - x0, target[1] - y0, -psi0)
    params = replace(params, target_x=sx, target_y=sy)
  world = World(cfg, robot, ball, seed, params)
  kick = PenaltyKick(params)
  kick.start(0.0)
  aim = math.atan2(target[1] - ball[1], target[0] - ball[0])
  t_strike, stopped = None, False
  while kick.active and world.t < max_t:
    t = world.t
    for ev in world.events():
      if ev[0] == "switch":
        kick.on_switch(t, ev[1], ev[2])
      elif ev[0] == "status":
        kick.on_clip_status(t, ev[1])
      else:
        kick.on_joint_speed(t, ev[1])
    if stop_at is not None and t >= stop_at and not stopped:
      stopped = True
      kick.stop(t, "test stop")
    goal = rot(target[0] - world.x, target[1] - world.y, -world.psi) \
      if params.goal_frame == "topic" else None
    cmd = kick.step(t, world.sense(), world.imu_yaw(), goal)
    world.handle(kick.pop_actions())
    if kick.state in (State.SEARCH, State.APPROACH, State.MEASURE, State.CORRECT):
      world.min_ball_dist = min(world.min_ball_dist, math.hypot(*world.ball_in_robot()))
    if kick.state == State.STRIKE and t_strike is None:
      t_strike = t
    world.advance(cmd)
  out = dict(state=kick.state.value, reason=kick.reason, side=kick.side, t=world.t,
             t_strike=t_strike, min_ball_dist=world.min_ball_dist, info=kick.status(),
             estopped=world.estopped, slot_active=world.slot_active, world=world)
  if world.kick is not None:
    k = world.kick
    shot_err = wrap(k["shot"] - aim)
    dist = math.hypot(target[0] - ball[0], target[1] - ball[1])
    out.update(ex=k["ex"], ey=k["ey"], shot_err_deg=math.degrees(shot_err),
               miss_m=dist * math.tan(shot_err))
  return out


def random_setup(rng: random.Random):
  """Robot at the origin with a random heading; ball 0.8-2.5 m away at a bearing
  within +-150 deg; target 1.5-3 m past the ball, within +-100 deg of the robot-to-ball line."""
  psi0 = rng.uniform(-math.pi, math.pi)
  d, bearing = rng.uniform(0.8, 2.5), math.radians(rng.uniform(-150, 150))
  ball = (d * math.cos(psi0 + bearing), d * math.sin(psi0 + bearing))
  line = math.atan2(ball[1], ball[0]) + math.radians(rng.uniform(-100, 100))
  gd = rng.uniform(1.5, 3.0)
  target = (ball[0] + gd * math.cos(line), ball[1] + gd * math.sin(line))
  return (0.0, 0.0, psi0), ball, target


def random_world(rng: random.Random, params: PenaltyParams, calibrated: bool = True):
  """Walker and sensor errors. calibrated: the params match the walker within the
  measurement error of the PENALTY_KICK.md calibration; otherwise the defaults."""
  cfg = WorldCfg(eff=rng.uniform(0.65, 0.95), eff_wz=rng.uniform(0.75, 1.0),
                 lag_s=rng.uniform(0.10, 0.25), drift=rng.uniform(-0.05, 0.05),
                 imu_drift_dps=rng.uniform(-1.0, 1.0),
                 bias_y=rng.gauss(0.0, 0.003 if calibrated else 0.01))
  if calibrated:
    params = replace(params, speed_efficiency=cfg.eff * rng.uniform(0.95, 1.05),
                     creep_efficiency=cfg.eff * rng.uniform(0.95, 1.05),
                     walker_lag_s=cfg.lag_s + rng.uniform(-0.03, 0.03),
                     walk_drift=cfg.drift + rng.uniform(-0.01, 0.01))
  return cfg, params


def pct(values, q):
  if not values:
    return float("nan")
  s = sorted(values)
  return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))]


def sweep(runs: int, seed: int = 0, calibrated: bool = True, params: PenaltyParams | None = None):
  rng = random.Random(seed)
  base = params or PenaltyParams()
  results = []
  for i in range(runs):
    robot, ball, target = random_setup(rng)
    cfg, p = random_world(rng, base, calibrated)
    results.append(run_attempt(p, cfg, robot, ball, target, seed=seed * 100003 + i))
  return results


def summary(results) -> dict:
  kicked = [r for r in results if "ey" in r]
  states = {}
  for r in results:
    key = r["state"] if r["state"] == "done" else f"{r['state']}: {r['reason'].split(' (')[0]}"
    states[key] = states.get(key, 0) + 1
  ab = lambda k: [abs(r[k]) for r in kicked]  # noqa: E731
  return dict(
    runs=len(results), kicked=len(kicked), states=states,
    ey_cm=[round(100 * pct(ab("ey"), q), 2) for q in (0.5, 0.9, 0.95)],
    ex_cm=[round(100 * pct(ab("ex"), q), 2) for q in (0.5, 0.9, 0.95)],
    shot_err_deg=[round(pct(ab("shot_err_deg"), q), 1) for q in (0.5, 0.9, 0.95)],
    miss_m=[round(pct(ab("miss_m"), q), 3) for q in (0.5, 0.9, 0.95)],
    t_strike_s=[round(pct([r["t_strike"] for r in kicked], q), 1) for q in (0.5, 0.9)],
    min_ball_dist_m=round(min((r["min_ball_dist"] for r in results), default=float("nan")), 3),
  )


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  ap.add_argument("--runs", type=int, default=200)
  ap.add_argument("--seed", type=int, default=1)
  ap.add_argument("--uncalibrated", action="store_true",
                  help="default walker params against a random walker (no floor calibration)")
  args = ap.parse_args(argv)
  s = summary(sweep(args.runs, args.seed, calibrated=not args.uncalibrated))
  print(f"runs {s['runs']}, kicked {s['kicked']}")
  for k, v in sorted(s["states"].items(), key=lambda kv: -kv[1]):
    print(f"  {v:4d}  {k}")
  print("p50/p90/p95  |lateral placement error| cm", s["ey_cm"])
  print("             |x placement error| cm      ", s["ex_cm"])
  print("             |shot direction error| deg  ", s["shot_err_deg"])
  print("             |miss at the target| m      ", s["miss_m"])
  print("p50/p90 time to strike s", s["t_strike_s"], "| min ball distance before the creep m",
        s["min_ball_dist_m"])
  return 0


if __name__ == "__main__":
  sys.exit(main())
