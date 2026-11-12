"""Header-kick task: scene wiring, scenario placement, contacts, rewards, perception."""

import io
import warnings
from contextlib import redirect_stdout
from types import SimpleNamespace

import mujoco
import pytest
import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.pupper_kick import mdp
from mjlab.tasks.pupper_kick.kick_env_cfg import pupper_header_kick_env_cfg

NUM_ENVS = 4


@pytest.fixture(scope="module")
def env():
  cfg = pupper_header_kick_env_cfg(perception=mdp.perfect_perception())
  cfg.scene.num_envs = NUM_ENVS
  with warnings.catch_warnings(), redirect_stdout(io.StringIO()):
    warnings.simplefilter("ignore")
    e = ManagerBasedRlEnv(cfg=cfg, device="cpu")
  e.reset()
  yield e
  e.close()


def _cmd(env) -> mdp.KickCommand:
  return env.command_manager.get_term("kick")


def _zero(env) -> torch.Tensor:
  return torch.zeros(env.num_envs, env.action_manager.total_action_dim)


def _place_ball(env, rel_h: tuple[float, float], vel_h: tuple[float, float]) -> None:
  """Put the ball at a heading-frame offset from base_link with a heading-frame velocity."""
  cmd = _cmd(env)
  yaw = cmd.robot.data.heading_w
  base = cmd.robot.data.root_link_pos_w
  c, s = torch.cos(yaw), torch.sin(yaw)
  pos = torch.zeros(env.num_envs, 7)
  pos[:, 0] = base[:, 0] + c * rel_h[0] - s * rel_h[1]
  pos[:, 1] = base[:, 1] + s * rel_h[0] + c * rel_h[1]
  pos[:, 2] = cmd.ball_radius()
  pos[:, 3] = 1.0
  vel = torch.zeros(env.num_envs, 6)
  vel[:, 0] = c * vel_h[0] - s * vel_h[1]
  vel[:, 1] = s * vel_h[0] + c * vel_h[1]
  cmd.ball.write_root_link_pose_to_sim(pos)
  cmd.ball.write_root_link_velocity_to_sim(vel)
  env.sim.forward()


def _fresh_episode(env) -> None:
  env.reset()
  for _ in range(15):  # settle on the feet
    env.step(_zero(env))


def test_collision_bits(env):
  m = env.sim.mj_model

  def gid(name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)

  for name in mdp.HEAD_GEOMS + mdp.CHASSIS_GEOMS:
    g = gid(f"robot/{name}")
    assert (m.geom_contype[g], m.geom_conaffinity[g]) == (2, 3)
  ball = gid("ball/ball_geom")
  assert (m.geom_contype[ball], m.geom_conaffinity[ball], m.geom_condim[ball]) == (3, 3, 3)
  foot = gid("robot/leg_front_r_3_collision")
  assert (m.geom_contype[foot], m.geom_conaffinity[foot]) == (1, 1)
  # The chassis boxes touch the floor, so the body cannot sink through it.
  floor = gid("terrain")
  assert m.geom_contype[floor] & m.geom_conaffinity[gid("robot/torso_shell")]
  # The leg spheres would match the boxes' conaffinity; body excludes keep them apart.
  base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "robot/base_link")
  excluded = {(int(a), int(b)) for a, b in zip(m.exclude_signature >> 16, m.exclude_signature & 0xFFFF)}
  for leg in ("front_r", "front_l", "back_r", "back_l"):
    for k in (2, 3):
      body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"robot/leg_{leg}_{k}")
      assert (min(base, body), max(base, body)) in excluded


def test_reset_places_ball_and_command(env):
  env.reset()
  cmd = _cmd(env)
  rel = cmd.ball_rel_heading()
  dist = rel.norm(dim=-1)
  lo, hi = cmd.cfg.distance_easy
  assert torch.all(dist > lo - 1e-3) and torch.all(dist < hi + 1e-3)
  assert torch.allclose(cmd.ball.data.root_link_pos_w[:, 2], cmd.ball_radius(), atol=1e-4)
  # Perfect perception: the command slot is the true heading-frame ball position.
  assert torch.allclose(cmd.command[:, :2], rel, atol=2e-3)
  # Level 0 has no cut: the aim points at the ball.
  bearing = torch.atan2(rel[:, 1], rel[:, 0])
  assert torch.allclose(cmd.command[:, 2], bearing, atol=2e-2)
  assert not cmd.kicked.any()


def test_ball_rolls_and_slows(env):
  _fresh_episode(env)
  # Across the front of the robot, clear of it, so no termination fires.
  _place_ball(env, (0.8, -0.5), (0.0, 1.0))
  cmd = _cmd(env)
  speeds = []
  for _ in range(50):
    env.step(_zero(env))
    speeds.append(cmd.ball_vel_world().norm(dim=-1))
  speeds = torch.stack(speeds)
  assert torch.all(env.episode_length_buf > 50)  # same episode throughout
  # Sliding settles into rolling at 5/7 of the launch speed, then the rolling
  # resistance (0.1 to 0.6 m/s^2) removes at most 0.6 m/s in 1 s.
  assert torch.all(speeds[-1] < speeds[5])
  assert torch.all(speeds[-1] > 5 / 7 - 0.6 - 0.05)
  decel = (speeds[10] - speeds[-1]) / (39 * env.step_dt)
  assert torch.allclose(decel, cmd._roll_decel, atol=0.05)
  assert not cmd.kicked.any() and not cmd.leg_touched.any()


def test_header_detection_and_rewards(env):
  _fresh_episode(env)
  # Ball rolls straight back into the bumper.
  _place_ball(env, (0.40, 0.0), (-1.5, 0.0))
  cmd = _cmd(env)
  terms = env.reward_manager.active_terms
  i_header, i_quality = terms.index("header"), terms.index("kick_quality")
  header_steps = torch.zeros(env.num_envs)
  quality_steps = torch.zeros(env.num_envs)
  for _ in range(30):
    env.step(_zero(env))
    step_r = env.reward_manager._step_reward
    header_steps += (step_r[:, i_header] != 0).float()
    quality_steps += (step_r[:, i_quality] != 0).float()
  assert cmd.kicked.all(), "the bumper contact was not detected"
  assert not cmd.fouled.any()
  assert torch.all(header_steps == 1), "the header event must fire exactly once"
  # Scored once, 0.3 s after contact; the ball bounced back toward the start, so
  # the event can be zero (along clipped at 0) but never fires twice.
  assert torch.all(quality_steps <= 1)
  assert cmd.recorded.all()
  assert torch.all(cmd.impact_speed.abs() < 0.5)  # the robot stood still


def test_leg_contact_is_foul(env):
  _fresh_episode(env)
  # In line with the right front foot (y = -0.10). At the largest random radius
  # (0.086 m) the ball still clears the 0.057 m half-width of the chassis.
  _place_ball(env, (0.40, -0.15), (-1.5, 0.0))
  cmd = _cmd(env)
  fouled = torch.zeros(env.num_envs, dtype=torch.bool)
  headed = torch.zeros(env.num_envs, dtype=torch.bool)
  for _ in range(20):
    env.step(_zero(env))
    fouled |= env.termination_manager.get_term("foul")
    headed |= env.termination_manager.get_term("kick_done")
  assert fouled.all(), "the leg contact did not end the attempt as a foul"
  assert not headed.any()


def test_kick_done_ends_episode(env):
  _fresh_episode(env)
  _place_ball(env, (0.40, 0.0), (-1.5, 0.0))
  cmd = _cmd(env)
  window_steps = int(round(cmd.cfg.kick_window_s / env.step_dt))
  ended = torch.zeros(env.num_envs, dtype=torch.bool)
  for _ in range(window_steps + 25):
    env.step(_zero(env))
    ended |= env.termination_manager.get_term("kick_done")
  assert ended.all()


def test_missed_ball_terminates(env):
  _fresh_episode(env)
  _place_ball(env, (-0.3, 0.3), (0.0, 0.0))  # behind base_link, no header
  env.step(_zero(env))
  assert env.termination_manager.get_term("missed_ball").all()


def test_perception_latency_and_blind_freeze():
  """Pure tensor check of the hold/latency logic on a minimal stand-in env."""
  p = mdp.BallPerceptionCfg(
    max_latency_steps=3,
    update_period_steps=(1, 1),
    dropout_prob=(0.0, 0.0),
    noise_std_base=(0.0, 0.0),
    noise_std_per_m=(0.0, 0.0),
    bias_m=0.0,
    scale_error=0.0,
    blind_distance=(0.3, 0.3),
    freeze_prob=1.0,
    aim_bias=0.0,
    aim_noise_std=0.0,
  )
  term = mdp.KickCommand.__new__(mdp.KickCommand)
  term.cfg = mdp.KickCommandCfg(perception=p)
  n = 1
  term._hist = torch.zeros(n, p.max_latency_steps + 1, 2)
  term._ptr = 0
  term._lag = torch.tensor([2])
  term._period = torch.tensor([1])
  term._phase = torch.zeros(n, dtype=torch.long)
  term._dropout = torch.zeros(n)
  term._noise0 = torch.zeros(n)
  term._noise1 = torch.zeros(n)
  term._bias = torch.zeros(n, 2)
  term._scale = torch.ones(n)
  term._blind = torch.tensor([0.3])
  term._freeze = torch.tensor([True])
  term._aim_bias = torch.zeros(n)
  term._held = torch.tensor([[1.0, 0.0]])
  term._hist[:] = torch.tensor([1.0, 0.0])
  term.kick_command = torch.zeros(n, 3)
  term.aim_yaw_w = torch.zeros(n)

  term.robot = SimpleNamespace(data=SimpleNamespace(heading_w=torch.zeros(1)))
  term._env = SimpleNamespace(num_envs=1, device="cpu")
  term._apply_rolling_resistance = lambda: None  # type: ignore[method-assign]
  xs = [0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.29, 0.2]
  seen = []
  for x in xs:
    term.ball_rel_heading = lambda x=x: torch.tensor([[x, 0.0]])  # type: ignore[method-assign]
    term._update_command()
    seen.append(round(float(term.command[0, 0]), 3))
  # Two-step latency while visible (the history starts full at 1.0); frozen at
  # the last estimate once the true range is inside 0.3 m.
  assert seen[:6] == [1.0, 1.0, 0.9, 0.8, 0.7, 0.6]
  assert seen[6:] == [0.6, 0.6]


def test_event_rewards_scale_to_weight(env):
  """An event term contributes exactly its weight once (dt cancels)."""
  r = mdp.termination_event
  dt = env.step_dt
  fake = torch.ones(env.num_envs, dtype=torch.bool)
  env.termination_manager._term_dones["fell_over"] = fake
  value = r(env, term_name="fell_over") * dt
  assert torch.allclose(value, torch.ones(env.num_envs))


def test_export_command_clip(env):
  from mjlab.tasks.pupper.export import command_clip_from_env

  clip = command_clip_from_env(env)
  assert clip == {"vx": (-0.5, 2.0), "vy": (-1.0, 1.0), "wz": (-1.2, 1.2)}


def test_body_on_floor_ends_episode(env):
  _fresh_episode(env)
  cmd = _cmd(env)
  pose = cmd.robot.data.root_link_pose_w.clone()
  pose[:, 2] = 0.04  # chassis bottom (z = -0.032 in base_link) under the floor
  cmd.robot.write_root_link_pose_to_sim(pose)
  env.sim.forward()
  env.step(_zero(env))
  assert env.termination_manager.get_term("body_ground").all()
