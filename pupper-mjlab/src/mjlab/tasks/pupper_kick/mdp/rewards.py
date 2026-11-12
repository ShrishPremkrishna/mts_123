"""Header-kick reward, termination and privileged-observation terms.

Every term reads the episode state from the ``kick`` command term. Event terms
(one step each) return ``1 / step_dt``: the reward manager multiplies by dt, so
the weight of an event term is its total reward.

Ordering inside one env step: terminations and rewards run before the command
term updates its flags. Thus ``cmd.kicked`` is the state before this step's
contacts, and ``header`` detects the first contact as ``head & ~kicked``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.tasks.pupper_kick.mdp.kick_command import KickCommand, contact_any, heading_frame
from mjlab.utils.lab_api.math import wrap_to_pi

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


def _cmd(env: "ManagerBasedRlEnv", command_name: str) -> KickCommand:
  term = env.command_manager.get_term(command_name)
  assert isinstance(term, KickCommand)
  return term


def _foul_now(cmd: KickCommand) -> torch.Tensor:
  """A leg or chassis touches the ball this step, before any header."""
  return (cmd.leg_contact() | cmd.torso_contact()) & ~cmd.kicked


# ---------------------------------------------------------------- rewards


def approach_velocity(
  env: "ManagerBasedRlEnv", command_name: str = "kick", max_speed: float = 1.5
) -> torch.Tensor:
  """Base velocity toward the ball center before the header [m/s], capped above."""
  cmd = _cmd(env, command_name)
  rel = cmd.ball.data.root_link_pos_w[:, :2] - cmd.robot.data.root_link_pos_w[:, :2]
  u = rel / rel.norm(dim=-1, keepdim=True).clamp_min(1e-6)
  v = cmd.robot.data.root_link_lin_vel_w[:, :2]
  toward = (v * u).sum(-1).clamp(max=max_speed)
  return toward * (~cmd.kicked).float()


def kick_alignment(
  env: "ManagerBasedRlEnv",
  command_name: str = "kick",
  heading_std: float = 0.35,
  lateral_std: float = 0.05,
  near: float = 0.3,
  falloff: float = 0.25,
) -> torch.Tensor:
  """Near the ball: body heading on the aim and the ball centered on the bumper.

  The bumper face is flat, so the ball leaves along the body heading. The term
  is weighted by proximity (1 inside ``near``, then exp decay) so that far from
  the ball the policy is free to run along the shortest path.
  """
  cmd = _cmd(env, command_name)
  rel_h = cmd.ball_rel_heading()
  heading_err = wrap_to_pi(cmd.aim_yaw_w - cmd.robot.data.heading_w)
  prox = torch.exp(-(rel_h.norm(dim=-1) - near).clamp_min(0.0) / falloff)
  align = torch.exp(-torch.square(heading_err / heading_std))
  center = torch.exp(-torch.square(rel_h[:, 1] / lateral_std))
  return align * center * prox * (~cmd.kicked).float()


def header(env: "ManagerBasedRlEnv", command_name: str = "kick") -> torch.Tensor:
  """Event: the first bumper contact with the ball, without a foul in the same step."""
  cmd = _cmd(env, command_name)
  return (cmd.head_contact() & ~cmd.kicked & ~_foul_now(cmd)).float() / env.step_dt


def kick_quality(
  env: "ManagerBasedRlEnv",
  command_name: str = "kick",
  across_gain: float = 0.6,
  max_speed: float = 3.0,
) -> torch.Tensor:
  """Event, once, ``eval_delay_s`` after the header: ball speed along the aim
  minus ``across_gain`` x speed across it [m/s] (along clipped to [0, max_speed]).

  Paid once rather than per step: a per-step version paid more for pushing the
  ball through the whole window than for a strike followed by braking. This is
  the same snapshot the command term scores (``ball_speed_aim``, ``aim_error``).
  """
  cmd = _cmd(env, command_name)
  due = cmd.kicked & ~cmd.recorded & (cmd.steps_since_kick() >= cmd.steps(cmd.cfg.eval_delay_s))
  v, a = cmd.ball_vel_world(), cmd.aim_vec_world()
  along = (v * a).sum(-1).clamp(0.0, max_speed)
  across = (a[:, 0] * v[:, 1] - a[:, 1] * v[:, 0]).abs().clamp(max=max_speed)
  return (along - across_gain * across) * due.float() / env.step_dt


def head_recontact(
  env: "ManagerBasedRlEnv", command_name: str = "kick", grace_s: float = 0.1
) -> torch.Tensor:
  """1 while the bumper touches the ball more than ``grace_s`` after the header
  (pushing or dribbling instead of a strike; use a negative weight)."""
  cmd = _cmd(env, command_name)
  late = cmd.kicked & (cmd.steps_since_kick() > cmd.steps(grace_s))
  return (cmd.head_contact() & late).float()


def ball_contact(
  env: "ManagerBasedRlEnv", sensor_name: str, force_threshold: float = 0.1
) -> torch.Tensor:
  """1 while the sensor's primaries touch the ball (use a negative weight)."""
  hit = contact_any(env.scene[sensor_name], force_threshold)
  return (hit & (env.episode_length_buf > 0)).float()


def post_kick_stop(
  env: "ManagerBasedRlEnv", command_name: str = "kick", delay_s: float = 0.2, std: float = 0.3
) -> torch.Tensor:
  """After the header: low horizontal base speed (brake, do not chase the ball)."""
  cmd = _cmd(env, command_name)
  active = cmd.kicked & (cmd.steps_since_kick() >= cmd.steps(delay_s))
  speed = cmd.robot.data.root_link_lin_vel_w[:, :2].norm(dim=-1)
  return torch.exp(-torch.square(speed / std)) * active.float()


def termination_event(
  env: "ManagerBasedRlEnv", term_name: str | tuple[str, ...]
) -> torch.Tensor:
  """Event: any of the named terminations fired this step (use a negative weight)."""
  names = (term_name,) if isinstance(term_name, str) else term_name
  fired = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
  for name in names:
    fired |= env.termination_manager.get_term(name)
  return fired.float() / env.step_dt


def base_below(
  env: "ManagerBasedRlEnv", min_height: float, command_name: str = "kick"
) -> torch.Tensor:
  """How far base_link is below ``min_height`` [m] (use a negative weight)."""
  z = _cmd(env, command_name).robot.data.root_link_pos_w[:, 2]
  return (min_height - z).clamp_min(0.0)


def joint_speed_excess(
  env: "ManagerBasedRlEnv", limit: float, command_name: str = "kick"
) -> torch.Tensor:
  """Sum over joints of (|qd| - limit)^2 above ``limit`` [rad/s].

  The PD actuator has no torque-speed curve, so the sim gives full torque at any
  speed; this keeps joint speeds in the range a real motor can drive.
  """
  qd = _cmd(env, command_name).robot.data.joint_vel
  return torch.square((qd.abs() - limit).clamp_min(0.0)).sum(-1)


# ---------------------------------------------------------------- terminations


def kick_done(env: "ManagerBasedRlEnv", command_name: str = "kick") -> torch.Tensor:
  """The scoring window after the header is over: the attempt is complete."""
  cmd = _cmd(env, command_name)
  return cmd.kicked & (cmd.steps_since_kick() >= cmd.steps(cmd.cfg.kick_window_s))


def foul(env: "ManagerBasedRlEnv", command_name: str = "kick") -> torch.Tensor:
  """A leg or the chassis touched the ball before the header: the attempt failed.

  Terminal, so a poked ball is never chased and a foul cannot be followed by a
  paid header.
  """
  return _foul_now(_cmd(env, command_name)) & (env.episode_length_buf > 1)


def body_ground(
  env: "ManagerBasedRlEnv", sensor_name: str, force_threshold: float = 1.0
) -> torch.Tensor:
  """The chassis or bumper touched the floor: a fall."""
  hit = contact_any(env.scene[sensor_name], force_threshold)
  return hit & (env.episode_length_buf > 1)


def missed_ball(
  env: "ManagerBasedRlEnv", command_name: str = "kick", lost_margin: float = 0.6
) -> torch.Tensor:
  """No header yet, and the ball is behind base_link or far away (knocked or left)."""
  cmd = _cmd(env, command_name)
  rel_h = cmd.ball_rel_heading()
  behind = rel_h[:, 0] < 0.0
  lost = rel_h.norm(dim=-1) > cmd.init_distance + lost_margin
  return ~cmd.kicked & (behind | lost) & (env.episode_length_buf > 1)


# ---------------------------------------------------------------- critic


def kick_privileged(env: "ManagerBasedRlEnv", command_name: str = "kick") -> torch.Tensor:
  """True ball state, aim, kick phase and ball parameters (critic only).

  Layout (12): ball_rel_heading(2), ball_vel_heading(2), aim_heading cos/sin(2),
  kicked(1), time_since_kick(1), header_contact(1), leg_touched(1),
  ball_radius(1), ball_mass(1).
  """
  cmd = _cmd(env, command_name)
  yaw = cmd.robot.data.heading_w
  aim = wrap_to_pi(cmd.aim_yaw_w - yaw)
  return torch.cat(
    [
      cmd.ball_rel_heading(),
      heading_frame(yaw, cmd.ball_vel_world()),
      torch.stack([torch.cos(aim), torch.sin(aim)], dim=-1),
      cmd.kicked.float()[:, None],
      cmd.time_since_kick().clamp(max=cmd.cfg.kick_window_s)[:, None],
      cmd.head_contact().float()[:, None],
      cmd.leg_touched.float()[:, None],
      cmd.ball_radius()[:, None],
      cmd.ball_mass()[:, None],
    ],
    dim=-1,
  )
