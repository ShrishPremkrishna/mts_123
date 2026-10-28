"""Opt-in continuous joint-command slew after absolute bounds, before latency.

This limits requested position change, not measured motor speed or acceleration.
The policy's raw action and observation history retain their original semantics.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import math
import re
import torch

from mjlab.envs.mdp.actions.actions import JointPositionAction, JointPositionActionCfg


def validate_slew_settings(rate, max_step_s):
  if (isinstance(rate, bool) or not math.isfinite(rate) or not 0 < rate <= 15
      or isinstance(max_step_s, bool) or not math.isfinite(max_step_s)
      or not 0 < max_step_s <= .02):
    raise ValueError('Target slew requires 0<rate<=15 rad/s and 0<max_step_s<=0.02')


@dataclass(kw_only=True)
class SlewLimitedJointPositionActionCfg(JointPositionActionCfg):
  max_rate_rad_s: float
  max_step_s: float = .02
  driver_limits_sha256: str

  def build(self, env):
    return SlewLimitedJointPositionAction(self, env)


class SlewLimitedJointPositionAction(JointPositionAction):
  def __init__(self, cfg, env):
    validate_slew_settings(cfg.max_rate_rad_s, cfg.max_step_s)
    if not isinstance(cfg.driver_limits_sha256, str) or not re.fullmatch('[0-9a-f]{64}', cfg.driver_limits_sha256):
      raise ValueError('Target slew requires an identified driver-limit contract')
    if not cfg.clip:
      raise ValueError('Target slew requires absolute target bounds')
    super().__init__(cfg, env)
    if not torch.isfinite(self._clip).all() or not torch.all(self._clip[:, :, 0] < self._clip[:, :, 1]):
      raise ValueError('Target slew requires finite bounds for every controlled joint')
    self._physics_dt = float(env.physics_dt)
    if not math.isfinite(self._physics_dt) or not 0 < self._physics_dt <= cfg.max_step_s:
      raise ValueError('Physics interval must be positive and within the slew time cap')
    self._slewed_actions = torch.zeros_like(self._processed_actions)
    self._needs_seed = torch.ones(env.num_envs, dtype=torch.bool, device=self.device)

  @property
  def applied_actions(self):
    """Last filtered target in encoder coordinates, before bias and actuator delay."""
    return self._slewed_actions

  def apply_actions(self):
    # Runs at physics frequency; process_actions keeps the actor's most recent
    # affine/bounded request, held between policy decisions. Seed after reset
    # events from current measured (biased) joints, not a zero/default command.
    measured = self._entity.data.joint_pos_biased[:, self._target_ids]
    measured = torch.clamp(measured, self._clip[:, :, 0], self._clip[:, :, 1])
    previous = torch.where(self._needs_seed[:, None], measured, self._slewed_actions)
    max_delta = self.cfg.max_rate_rad_s * min(self._physics_dt, self.cfg.max_step_s)
    self._slewed_actions[:] = previous + torch.clamp(self._processed_actions - previous, -max_delta, max_delta)
    self._needs_seed[:] = False
    encoder_bias = self._entity.data.encoder_bias[:, self._target_ids]
    self._entity.set_joint_position_target(self._slewed_actions - encoder_bias, joint_ids=self._target_ids)

  def reset(self, env_ids=None):
    super().reset(env_ids)
    ids = slice(None) if env_ids is None else env_ids
    self._needs_seed[ids] = True
    self._slewed_actions[ids] = 0.


def apply_position_target_slew(cfg, rate, driver_contract):
  """Replace only the known fixed-actuator-delay action; no legacy changes."""
  validate_slew_settings(rate, .02)
  action = cfg.actions.get('joint_pos')
  if type(action) is not JointPositionActionCfg or action.entity_name != 'robot':
    raise ValueError('Target slew currently requires the plain Pupper position action with actuator latency')
  if not driver_contract or driver_contract.get('contract') != 'controller_then_driver_joint_target_clamp_v1':
    raise ValueError('Target slew requires explicit controller and driver clamps')
  expected = driver_contract['effective_limits']
  if action.clip != expected:
    raise ValueError('Target slew bounds differ from the driver contract')
  cfg.actions['joint_pos'] = SlewLimitedJointPositionActionCfg(
      **{f.name: getattr(action, f.name) for f in fields(action)},
      max_rate_rad_s=float(rate), driver_limits_sha256=driver_contract['source']['sha256'])
  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES
  return dict(contract='pupper_position_target_slew_v1', max_rate_rad_s=float(rate), max_step_s=.02,
              joint_names=list(JOINT_NAMES), lower_limits=[expected[n][0] for n in JOINT_NAMES],
              upper_limits=[expected[n][1] for n in JOINT_NAMES],
              driver_limits_sha256=driver_contract['source']['sha256'])


def target_slew_metadata(env):
  term = env.action_manager.get_term('joint_pos')
  if not isinstance(term, SlewLimitedJointPositionAction):
    return None
  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES
  if list(term.target_names) != list(JOINT_NAMES):
    raise ValueError('Target slew export requires fixed Pupper joint order')
  return dict(contract='pupper_position_target_slew_v1', max_rate_rad_s=float(term.cfg.max_rate_rad_s),
              max_step_s=float(term.cfg.max_step_s), joint_names=list(term.target_names),
              lower_limits=term._clip[0, :, 0].detach().cpu().tolist(),
              upper_limits=term._clip[0, :, 1].detach().cpu().tolist(),
              driver_limits_sha256=term.cfg.driver_limits_sha256)
