"""Kick scenario, episode state, curriculum and ball-perception model.

One command term owns one kick attempt per episode:

* At reset it places the robot and the ball, picks the aim, and samples the
  perception model. It writes root states directly. It must not read derived
  quantities there, because the env runs ``forward()`` only after the reset.
* Each step (after ``forward()``) it updates the header/foul flags from the ball
  contact sensors and emits the 3-vector that fills the deploy frame's command
  slot: ``[ball_x, ball_y, aim]`` in the robot heading frame (m, m, rad).

The ball estimate imitates the soccer perception stack: latency, a slower update
rate, dropouts, range-dependent noise, a calibration bias and scale error, and a
blind zone near the nose where the estimate either freezes or keeps updating
(dead reckoning). The aim is exact up to a small bias and noise, because the
robot computes it from its IMU yaw.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import torch

from mjlab.managers.command_manager import CommandTerm, CommandTermCfg
from mjlab.utils.lab_api.math import quat_from_euler_xyz, wrap_to_pi

if TYPE_CHECKING:
  from mjlab.entity import Entity
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.sensor import ContactSensor


def contact_any(sensor: "ContactSensor", force_threshold: float) -> torch.Tensor:
  """True where any primary touched the secondary during the last control step.

  ``found`` (last substep) OR the per-substep force history: the history catches
  a short impact between two control steps, and ``found`` covers sensors whose
  reduced force reads zero (mujoco_warp's netforce returned 0 N for the box-on-
  plane chassis contacts while native MuJoCo reported several hundred N).
  """
  data = sensor.data
  assert data.found is not None
  hit = (data.found > 0).flatten(1).any(-1)
  if data.force_history is not None:
    # [B, N, H, 3] -> any primary, any substep.
    hit |= (torch.norm(data.force_history, dim=-1) > force_threshold).flatten(1).any(-1)
  return hit


def heading_frame(yaw: torch.Tensor, vec_xy: torch.Tensor) -> torch.Tensor:
  """Rotate world-frame xy vectors into the yaw-only robot frame."""
  c, s = torch.cos(yaw), torch.sin(yaw)
  x, y = vec_xy[:, 0], vec_xy[:, 1]
  return torch.stack([c * x + s * y, -s * x + c * y], dim=-1)


def _uniform(n: int, lo: float, hi: float, device) -> torch.Tensor:
  return torch.empty(n, device=device).uniform_(lo, hi)


def _lerp(a: float, b: float, t: float) -> float:
  return a + (b - a) * t


@dataclass(kw_only=True)
class BallPerceptionCfg:
  """Per-episode ranges for the ball-estimate model (see module docstring).

  The proposal assumes approximately 0.25 s of camera latency at 10 Hz before any
  odometry correction; these ranges cover a soccer node that forward-predicts
  part of it. Measure the real pipeline and narrow them.
  """

  min_latency_steps: int = 1
  max_latency_steps: int = 10  # 20 to 200 ms
  update_period_steps: tuple[int, int] = (1, 5)  # 50 to 10 Hz
  dropout_prob: tuple[float, float] = (0.0, 0.2)
  noise_std_base: tuple[float, float] = (0.0, 0.01)
  noise_std_per_m: tuple[float, float] = (0.0, 0.03)
  bias_m: float = 0.02
  scale_error: float = 0.10  # floor-plane ranging is sensitive to camera pitch
  blind_distance: tuple[float, float] = (0.30, 0.45)
  # 1.0 = the deploy contract: hold the last estimate once the ball is unseen.
  freeze_prob: float = 1.0
  aim_bias: float = 0.03
  aim_noise_std: float = 0.02


def perfect_perception() -> BallPerceptionCfg:
  """No latency, noise, dropout or blind zone (for diagnostics)."""
  return BallPerceptionCfg(
    min_latency_steps=0,
    max_latency_steps=0,
    update_period_steps=(1, 1),
    dropout_prob=(0.0, 0.0),
    noise_std_base=(0.0, 0.0),
    noise_std_per_m=(0.0, 0.0),
    bias_m=0.0,
    scale_error=0.0,
    blind_distance=(0.0, 0.0),
    freeze_prob=0.0,
    aim_bias=0.0,
    aim_noise_std=0.0,
  )


@dataclass(kw_only=True)
class KickCommandCfg(CommandTermCfg):
  robot_name: str = "robot"
  ball_name: str = "ball"
  head_sensor_name: str = "ball_head_contact"
  leg_sensor_name: str = "ball_leg_contact"
  torso_sensor_name: str = "ball_torso_contact"
  # One scenario per episode: never resample mid-episode.
  resampling_time_range: tuple[float, float] = (1.0e9, 1.0e9)

  # Scenario ranges at difficulty 0 and 1 (base_link to ball center, heading frame).
  distance_easy: tuple[float, float] = (0.45, 0.8)
  distance_hard: tuple[float, float] = (0.45, 1.5)
  bearing_easy: float = 0.15
  bearing_hard: float = 0.5
  cut_easy: float = 0.0
  cut_hard: float = 0.4

  # Adaptive curriculum on the success rate of each window of num_envs episodes.
  initial_level: float = 0.0
  adaptive_curriculum: bool = True
  level_up_rate: float = 0.6
  level_down_rate: float = 0.25
  level_step: float = 0.1

  # Robot start: z jitter above the keyframe, and a walking handoff in a share of
  # episodes (forward speed along the start heading).
  robot_height_jitter: float = 0.02
  moving_start_prob: float = 0.5
  moving_start_speed: tuple[float, float] = (0.0, 0.25)
  ball_drift_speed: float = 0.05
  # Rolling resistance [m/s^2], sampled per episode, applied as a force against
  # the ball's horizontal velocity while it is on the floor (see mdp.scene).
  rolling_decel: tuple[float, float] = (0.1, 0.6)

  # Header bookkeeping.
  kick_window_s: float = 1.0
  eval_delay_s: float = 0.3
  contact_force_threshold: float = 0.1
  success_min_speed: float = 0.3
  success_max_aim_error: float = math.radians(20.0)

  perception: BallPerceptionCfg = field(default_factory=BallPerceptionCfg)

  # Deploy clamp for the command slot, in the exporter's vx/vy/wz vocabulary.
  deploy_clip: dict[str, tuple[float, float]] = field(
    default_factory=lambda: {
      # Slightly wider than the spawn ranges (x 0.25-1.6 m, |y| <= 0.75 m,
      # |aim| <= 0.9 rad); the soccer node must only start a kick inside them.
      "vx": (-0.5, 2.0),
      "vy": (-1.0, 1.0),
      "wz": (-1.2, 1.2),
    }
  )

  def build(self, env: "ManagerBasedRlEnv") -> "KickCommand":
    return KickCommand(self, env)


# Metrics kept as exponential moving averages over finished episodes, so every
# reset step logs every key (rsl_rl averages per-step log dicts).
_METRIC_KEYS = (
  "header_rate",
  "success_rate",
  "foul_rate",
  "ball_speed_aim",
  "aim_error_deg",
  "impact_speed",
  "time_to_header",
  "level",
)


class KickCommand(CommandTerm):
  cfg: KickCommandCfg

  def __init__(self, cfg: KickCommandCfg, env: "ManagerBasedRlEnv"):
    super().__init__(cfg, env)
    self.robot: Entity = env.scene[cfg.robot_name]
    self.ball: Entity = env.scene[cfg.ball_name]
    n, dev = self.num_envs, self.device

    self.kick_command = torch.zeros(n, 3, device=dev)
    self.aim_yaw_w = torch.zeros(n, device=dev)
    self.init_distance = torch.zeros(n, device=dev)

    # Episode state.
    self.kicked = torch.zeros(n, dtype=torch.bool, device=dev)
    self.kick_step = torch.zeros(n, dtype=torch.long, device=dev)
    self.leg_touched = torch.zeros(n, dtype=torch.bool, device=dev)
    self.torso_touched = torch.zeros(n, dtype=torch.bool, device=dev)
    self.recorded = torch.zeros(n, dtype=torch.bool, device=dev)
    self.ball_speed_aim = torch.zeros(n, device=dev)
    self.aim_error = torch.zeros(n, device=dev)
    self.impact_speed = torch.zeros(n, device=dev)
    self.time_to_header = torch.zeros(n, device=dev)

    # Perception model.
    p = cfg.perception
    self._hist = torch.zeros(n, p.max_latency_steps + 1, 2, device=dev)
    self._ptr = 0
    self._lag = torch.zeros(n, dtype=torch.long, device=dev)
    self._period = torch.ones(n, dtype=torch.long, device=dev)
    self._phase = torch.zeros(n, dtype=torch.long, device=dev)
    self._dropout = torch.zeros(n, device=dev)
    self._noise0 = torch.zeros(n, device=dev)
    self._noise1 = torch.zeros(n, device=dev)
    self._bias = torch.zeros(n, 2, device=dev)
    self._scale = torch.ones(n, device=dev)
    self._blind = torch.zeros(n, device=dev)
    self._freeze = torch.zeros(n, dtype=torch.bool, device=dev)
    self._aim_bias = torch.zeros(n, device=dev)
    self._held = torch.zeros(n, 2, device=dev)
    self._roll_decel = torch.zeros(n, device=dev)

    # Curriculum and logged averages.
    self.level = float(cfg.initial_level)
    self._window_success = 0.0
    self._window_episodes = 0.0
    self._ema = {k: 0.0 for k in _METRIC_KEYS}
    self._ema_init = {k: False for k in _METRIC_KEYS}

    self._ball_geom_id = int(self.ball.indexing.geom_ids[0].item())

  # ------------------------------------------------------------------ queries

  @property
  def command(self) -> torch.Tensor:
    return self.kick_command

  @property
  def step_dt(self) -> float:
    return self._env.step_dt

  def ball_radius(self, env_ids: torch.Tensor | slice = slice(None)) -> torch.Tensor:
    size = self._env.sim.model.geom_size
    if size.ndim == 3:
      r = size[:, self._ball_geom_id, 0]
      if r.shape[0] == 1:
        r = r.expand(self.num_envs)
    else:
      r = size[self._ball_geom_id, 0].expand(self.num_envs)
    return r[env_ids]

  def ball_mass(self) -> torch.Tensor:
    mass = self._env.sim.model.body_mass
    bid = int(self.ball.indexing.root_body_id)
    if mass.ndim == 2:
      m = mass[:, bid]
      return m.expand(self.num_envs) if m.shape[0] == 1 else m
    return mass[bid].expand(self.num_envs)

  def ball_rel_heading(self) -> torch.Tensor:
    """True ball-center position from base_link, heading frame (num_envs, 2)."""
    rel = self.ball.data.root_link_pos_w[:, :2] - self.robot.data.root_link_pos_w[:, :2]
    return heading_frame(self.robot.data.heading_w, rel)

  def ball_vel_world(self) -> torch.Tensor:
    return self.ball.data.root_link_lin_vel_w[:, :2]

  def aim_vec_world(self) -> torch.Tensor:
    return torch.stack([torch.cos(self.aim_yaw_w), torch.sin(self.aim_yaw_w)], dim=-1)

  def ball_inertia_factor(self) -> torch.Tensor:
    """1 + I / (m r^2) per env (7/5 for a solid sphere at nominal size).

    The radius DR scales only the geom, so the ratio drifts from 2/5 (0.40 to
    0.52 over the radius range); a rolling ball's response to a center force uses it.
    """
    inertia = self._env.sim.model.body_inertia
    bid = int(self.ball.indexing.root_body_id)
    i0 = inertia[:, bid, 0] if inertia.ndim == 3 else inertia[bid, 0].expand(self.num_envs)
    if i0.shape[0] == 1:
      i0 = i0.expand(self.num_envs)
    return 1.0 + i0 / (self.ball_mass() * self.ball_radius().square())

  def steps_since_kick(self) -> torch.Tensor:
    """Control steps since the header (0 before it). Integer, so thresholds do not
    depend on float32 rounding of step counts times dt."""
    k = self._env.episode_length_buf - self.kick_step
    return torch.where(self.kicked, k, torch.zeros_like(k))

  def steps(self, seconds: float) -> int:
    return int(round(seconds / self.step_dt))

  def time_since_kick(self) -> torch.Tensor:
    return self.steps_since_kick().float() * self.step_dt

  def fresh(self) -> torch.Tensor:
    """Envs reset this step: the sensors' ``found`` still reports the last physics
    state of the old episode until the next substep."""
    return self._env.episode_length_buf == 0

  def head_contact(self) -> torch.Tensor:
    s = self._env.scene[self.cfg.head_sensor_name]
    return contact_any(s, self.cfg.contact_force_threshold) & ~self.fresh()

  def leg_contact(self) -> torch.Tensor:
    s = self._env.scene[self.cfg.leg_sensor_name]
    return contact_any(s, self.cfg.contact_force_threshold) & ~self.fresh()

  def torso_contact(self) -> torch.Tensor:
    s = self._env.scene[self.cfg.torso_sensor_name]
    return contact_any(s, self.cfg.contact_force_threshold) & ~self.fresh()

  @property
  def fouled(self) -> torch.Tensor:
    """A leg or chassis touched the ball before the header."""
    return self.leg_touched | self.torso_touched

  def success(self) -> torch.Tensor:
    return (
      self.kicked
      & self.recorded
      & ~self.fouled
      & (self.ball_speed_aim >= self.cfg.success_min_speed)
      & (self.aim_error <= self.cfg.success_max_aim_error)
    )

  # ------------------------------------------------------------------ reset

  def reset(self, env_ids: torch.Tensor | slice | None) -> dict[str, float]:
    if env_ids is None or isinstance(env_ids, slice):
      env_ids = torch.arange(self.num_envs, device=self.device)
    if len(env_ids) > 0 and self._env.common_step_counter > 0:
      self._log_finished(env_ids)
    super().reset(env_ids)
    # The manager prefixes these with "Metrics/<term name>/".
    return dict(self._ema)

  def _ema_update(self, key: str, values: torch.Tensor) -> None:
    if values.numel() == 0:
      return
    mean = float(values.float().mean().item())
    if not self._ema_init[key]:
      self._ema[key], self._ema_init[key] = mean, True
      return
    # Weight per episode, so a large reset batch moves the average more.
    alpha = 1.0 - (1.0 - 0.002) ** values.numel()
    self._ema[key] += alpha * (mean - self._ema[key])

  def _log_finished(self, env_ids: torch.Tensor) -> None:
    kicked = self.kicked[env_ids]
    rec = self.recorded[env_ids]
    succ = self.success()[env_ids]
    self._ema_update("header_rate", kicked)
    self._ema_update("success_rate", succ)
    self._ema_update("foul_rate", self.fouled[env_ids])
    self._ema_update("ball_speed_aim", self.ball_speed_aim[env_ids][rec])
    self._ema_update("aim_error_deg", torch.rad2deg(self.aim_error[env_ids][rec]))
    self._ema_update("impact_speed", self.impact_speed[env_ids][kicked])
    self._ema_update("time_to_header", self.time_to_header[env_ids][kicked])

    if self.cfg.adaptive_curriculum:
      self._window_success += float(succ.float().sum().item())
      self._window_episodes += float(len(env_ids))
      if self._window_episodes >= self.num_envs:
        rate = self._window_success / self._window_episodes
        if rate > self.cfg.level_up_rate:
          self.level = min(1.0, self.level + self.cfg.level_step)
        elif rate < self.cfg.level_down_rate:
          self.level = max(0.0, self.level - 0.5 * self.cfg.level_step)
        self._window_success = 0.0
        self._window_episodes = 0.0
    self._ema["level"], self._ema_init["level"] = self.level, True

  def _resample_command(self, env_ids: torch.Tensor) -> None:
    cfg, dev, n = self.cfg, self.device, len(env_ids)
    lvl = self.level

    d_lo = _lerp(cfg.distance_easy[0], cfg.distance_hard[0], lvl)
    d_hi = _lerp(cfg.distance_easy[1], cfg.distance_hard[1], lvl)
    b_max = _lerp(cfg.bearing_easy, cfg.bearing_hard, lvl)
    c_max = _lerp(cfg.cut_easy, cfg.cut_hard, lvl)
    dist = _uniform(n, d_lo, d_hi, dev)
    bearing = _uniform(n, -b_max, b_max, dev)
    cut = _uniform(n, -c_max, c_max, dev)
    yaw = _uniform(n, -math.pi, math.pi, dev)

    # Robot: keyframe pose at the env origin, random yaw, optional forward speed.
    root = self.robot.data.default_root_state[env_ids].clone()
    pos = root[:, :3] + self._env.scene.env_origins[env_ids]
    pos[:, 2] += _uniform(n, 0.0, cfg.robot_height_jitter, dev)
    zeros = torch.zeros(n, device=dev)
    quat = quat_from_euler_xyz(zeros, zeros, yaw)
    moving = torch.rand(n, device=dev) < cfg.moving_start_prob
    speed = _uniform(n, *cfg.moving_start_speed, dev) * moving.float()
    lin = torch.stack([speed * torch.cos(yaw), speed * torch.sin(yaw), zeros], dim=-1)
    self.robot.write_root_link_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=env_ids)
    self.robot.write_root_link_velocity_to_sim(
      torch.cat([lin, torch.zeros(n, 3, device=dev)], dim=-1), env_ids=env_ids
    )

    # Ball: on the floor at (dist, bearing) in the start heading frame.
    rel_h = torch.stack([dist * torch.cos(bearing), dist * torch.sin(bearing)], dim=-1)
    c, s = torch.cos(yaw), torch.sin(yaw)
    rel_w = torch.stack(
      [c * rel_h[:, 0] - s * rel_h[:, 1], s * rel_h[:, 0] + c * rel_h[:, 1]], dim=-1
    )
    ball_pos = torch.zeros(n, 3, device=dev)
    ball_pos[:, :2] = pos[:, :2] + rel_w
    ball_pos[:, 2] = self._env.scene.env_origins[env_ids, 2] + self.ball_radius(env_ids)
    ball_quat = torch.zeros(n, 4, device=dev)
    ball_quat[:, 0] = 1.0
    ball_vel = torch.zeros(n, 6, device=dev)
    ball_vel[:, :2] = torch.empty(n, 2, device=dev).uniform_(
      -cfg.ball_drift_speed, cfg.ball_drift_speed
    )
    self.ball.write_root_link_pose_to_sim(torch.cat([ball_pos, ball_quat], dim=-1), env_ids=env_ids)
    self.ball.write_root_link_velocity_to_sim(ball_vel, env_ids=env_ids)

    self.ball.write_external_wrench_to_sim(
      torch.zeros(n, 1, 3, device=dev), torch.zeros(n, 1, 3, device=dev), env_ids=env_ids
    )
    self._roll_decel[env_ids] = _uniform(n, *cfg.rolling_decel, dev)
    self.aim_yaw_w[env_ids] = wrap_to_pi(yaw + bearing + cut)
    self.init_distance[env_ids] = dist

    # Episode state.
    for buf in (self.kicked, self.leg_touched, self.torso_touched, self.recorded):
      buf[env_ids] = False
    for buf in (self.ball_speed_aim, self.aim_error, self.impact_speed, self.time_to_header):
      buf[env_ids] = 0.0
    self.kick_step[env_ids] = 0

    # Perception parameters for this episode.
    p = cfg.perception
    self._lag[env_ids] = torch.randint(
      p.min_latency_steps, p.max_latency_steps + 1, (n,), device=dev
    )
    self._period[env_ids] = torch.randint(
      p.update_period_steps[0], p.update_period_steps[1] + 1, (n,), device=dev
    )
    self._phase[env_ids] = 0
    self._dropout[env_ids] = _uniform(n, *p.dropout_prob, dev)
    self._noise0[env_ids] = _uniform(n, *p.noise_std_base, dev)
    self._noise1[env_ids] = _uniform(n, *p.noise_std_per_m, dev)
    self._bias[env_ids] = torch.empty(n, 2, device=dev).uniform_(-p.bias_m, p.bias_m)
    self._scale[env_ids] = 1.0 + _uniform(n, -p.scale_error, p.scale_error, dev)
    self._blind[env_ids] = _uniform(n, *p.blind_distance, dev)
    self._freeze[env_ids] = torch.rand(n, device=dev) < p.freeze_prob
    self._aim_bias[env_ids] = _uniform(n, -p.aim_bias, p.aim_bias, dev)

    # Start with a full history at the true start position and one measurement.
    self._hist[env_ids] = rel_h[:, None, :]
    self._held[env_ids] = self._measure(rel_h, env_ids)
    self.kick_command[env_ids, :2] = self._held[env_ids]
    self.kick_command[env_ids, 2] = wrap_to_pi(bearing + cut + self._aim_bias[env_ids])

  def _measure(self, rel: torch.Tensor, env_ids: torch.Tensor | slice) -> torch.Tensor:
    std = self._noise0[env_ids] + self._noise1[env_ids] * rel.norm(dim=-1)
    noise = torch.randn_like(rel) * std[:, None]
    return rel * self._scale[env_ids, None] + self._bias[env_ids] + noise

  # ------------------------------------------------------------------ step

  def _update_metrics(self) -> None:
    head = self.head_contact()
    leg = self.leg_contact()
    torso = self.torso_contact()
    step = self._env.episode_length_buf

    new = head & ~self.kicked
    if new.any():
      vel_h = heading_frame(self.robot.data.heading_w, self.robot.data.root_link_lin_vel_w[:, :2])
      self.kick_step[new] = step[new]
      self.impact_speed[new] = vel_h[new, 0]
      self.time_to_header[new] = step[new].float() * self.step_dt
    # A leg or chassis touch before the header is a foul.
    self.leg_touched |= leg & ~self.kicked
    self.torso_touched |= torso & ~self.kicked
    self.kicked |= head

    # Score the kick once, a fixed time after contact.
    snap = (
      self.kicked & ~self.recorded & (self.steps_since_kick() >= self.steps(self.cfg.eval_delay_s))
    )
    if snap.any():
      v = self.ball_vel_world()
      along = (v * self.aim_vec_world()).sum(-1)
      err = wrap_to_pi(torch.atan2(v[:, 1], v[:, 0]) - self.aim_yaw_w).abs()
      self.ball_speed_aim[snap] = along[snap]
      self.aim_error[snap] = err[snap]
      self.recorded |= snap

  def _update_command(self) -> None:
    p = self.cfg.perception
    true_rel = self.ball_rel_heading()
    size = self._hist.shape[1]
    self._ptr = (self._ptr + 1) % size
    self._hist[:, self._ptr] = true_rel
    idx = (self._ptr - self._lag) % size
    lagged = self._hist[torch.arange(self.num_envs, device=self.device), idx]

    self._phase += 1
    due = (self._phase % self._period) == 0
    dropped = torch.rand(self.num_envs, device=self.device) < self._dropout
    blind = (true_rel.norm(dim=-1) < self._blind) & self._freeze
    update = due & ~dropped & ~blind
    self._held = torch.where(update[:, None], self._measure(lagged, slice(None)), self._held)

    aim = self.aim_yaw_w - self.robot.data.heading_w + self._aim_bias
    aim = aim + torch.randn_like(aim) * p.aim_noise_std
    # Same clamp as the robot's command_clip, so training sees what deploy sends
    # (matters after the kick, when the ball rolls out of range).
    clip = self.cfg.deploy_clip
    self.kick_command[:, 0] = self._held[:, 0].clamp(*clip["vx"])
    self.kick_command[:, 1] = self._held[:, 1].clamp(*clip["vy"])
    self.kick_command[:, 2] = wrap_to_pi(aim)
    self._apply_rolling_resistance()

  def _apply_rolling_resistance(self) -> None:
    """Constant deceleration against the horizontal ball velocity, held for the
    next control step (xfrc_applied persists). Capped so that it stops the ball
    within one step instead of reversing it."""
    v = self.ball_vel_world()
    speed = v.norm(dim=-1)
    on_floor = self.ball.data.root_link_pos_w[:, 2] < self.ball_radius() + 0.01
    decel = torch.minimum(self._roll_decel, speed / self.step_dt) * on_floor.float()
    force = torch.zeros(self.num_envs, 1, 3, device=self.device)
    # A force at the center of a rolling sphere decelerates it by F / ((1 + I/(m r^2)) m),
    # because floor friction also spins it down; scale so ``decel`` is the
    # deceleration of the rolling ball.
    mag = self.ball_inertia_factor() * self.ball_mass() * decel
    force[:, 0, :2] = -(mag / speed.clamp_min(1e-6))[:, None] * v
    self.ball.write_external_wrench_to_sim(force, torch.zeros_like(force))
