"""Opt-in measurement and penalty for torso support on flat ground.

A signed penetration tolerance alone misses sustained body load. This sensor
reads actual contact forces every physics substep without changing collisions,
observations, action interfaces, reference poses or deployment metadata.
"""
from __future__ import annotations
import math
import torch
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg

SENSOR_NAME = 'torso_ground_support'


def enable_torso_support_audit(cfg):
  from mjlab.tasks.pupper.locomotion_contact_contract import get_spec_with_torso_contact
  if cfg.scene.entities['robot'].spec_fn is not get_spec_with_torso_contact:
    raise ValueError('Torso support audit requires the explicit locomotion torso hull')
  if cfg.scene.terrain is None or cfg.scene.terrain.terrain_type != 'plane':
    raise ValueError('Vertical torso-load interpretation requires flat plane terrain')
  if any(s.name == SENSOR_NAME for s in cfg.scene.sensors or ()):
    raise ValueError('Torso support audit already configured')
  cfg.scene.sensors = tuple(cfg.scene.sensors or ()) + (ContactSensorCfg(
    name=SENSOR_NAME,
    primary=ContactMatch(mode='geom', pattern=r'^base_link_hull$', entity='robot'),
    secondary=ContactMatch(mode='body', pattern='terrain'),
    fields=('found', 'force'), reduce='netforce', num_slots=1,
    history_length=cfg.decimation,
  ),)
  return dict(contract='flat_torso_support_audit_v1', sensor=SENSOR_NAME,
              force_frame='world', history_physics_steps=cfg.decimation,
              reduction='Sum torso contacts against terrain, absolute vertical force averaged over the previous policy interval, normalized by actual randomized robot weight.',
              sampling='Pre-step audit reads the preceding interval. Reward reads the just-completed interval. Reset rows must be censored.',
              physical_model_unchanged=True, actor_observation_unchanged=True)


def normalized_torso_load(force_history, bodyweight):
  """Mean vertical ground load / actual bodyweight, without a reward cap.

  history is [world, primary, physics_step, xyz], bodyweight is [world].
  Flat-plane vertical contact is the normal support direction. Absolute value
  makes the primary/secondary contact sign convention immaterial.
  """
  if force_history.ndim != 4 or force_history.shape[-1] != 3 or bodyweight.ndim != 1:
    raise ValueError('Expected force history [B,N,H,3] and bodyweight [B]')
  if force_history.shape[0] != bodyweight.shape[0] or force_history.shape[2] < 1:
    raise ValueError('Force history and bodyweight dimensions disagree')
  return force_history[..., 2].sum(dim=1).abs().mean(dim=-1) / bodyweight


def torso_support_diagnostics(env):
  robot=env.scene['robot'];sensor=env.scene[SENSOR_NAME]
  history=sensor.data.force_history
  if history is None:
    raise ValueError('Torso support sensor must record each physics substep')
  # Mechanics are startup-only in this contract; validate once and cache the
  # actual randomized weights without adding a GPU synchronization each step.
  bodyweight=getattr(env,'_torso_support_weight_n',None)
  if bodyweight is None:
    gx,gy,gz=map(float,env.sim.mj_model.opt.gravity)
    if not all(math.isfinite(g) for g in (gx,gy,gz)) or abs(gx)>1e-8 or abs(gy)>1e-8 or gz>=0:
      raise ValueError('Torso support requires finite vertical downward gravity')
    masses=env.sim.model.body_mass[:,robot.indexing.body_ids].sum(dim=1)
    if masses.shape[0]==1:masses=masses.expand(env.num_envs)
    if not torch.isfinite(masses).all() or not (masses>0).all():
      raise ValueError('Torso support requires finite positive actual robot mass')
    bodyweight=masses*(-gz)
    env._torso_support_weight_n=bodyweight
  return dict(torso_force_world=sensor.data.force.sum(dim=1),
              torso_mean_normal_force_n=history[...,2].sum(dim=1).abs().mean(dim=-1),
              torso_peak_normal_force_n=history[...,2].sum(dim=1).abs().amax(dim=-1),
              torso_normal_load_fraction=normalized_torso_load(history,bodyweight),
              robot_weight_n=bodyweight)


def torso_support_penalty(env):
  # A cap bounds isolated impact contributions without discarding their uncapped
  # audit values. Sustained 100% bodyweight support produces a cost of one.
  return torso_support_diagnostics(env)['torso_normal_load_fraction'].clamp(0.,2.)


def apply_torso_support_penalty(cfg, weight):
  if isinstance(weight,bool) or not math.isfinite(weight) or not -5. <= weight <= 0.:
    raise ValueError('Torso support penalty weight must be finite in [-5,0]')
  contract=enable_torso_support_audit(cfg)
  cfg.rewards['torso_support']=RewardTermCfg(func=torso_support_penalty,weight=float(weight))
  return {**contract,'reward_weight':float(weight),'reward_cap_bodyweights':2.,
          'scope':'Explicit reward intervention; weight0 is the matched sensor-only control. No change to reference stance or physical limits.'}


def nominal_mechanics(cfg):
  """Diagnostic only: remove sampled mechanics, preserve sensing/latency/reset noise."""
  names=('foot_friction','encoder_bias','base_com','pd_gains','inertial')
  removed=[name for name in names if cfg.events.pop(name,None) is not None]
  return dict(contract='nominal_mechanics_diagnostic_v1',removed_events=removed,
              retained='Observation noise, IMU and actuator latency, and reset randomness remain. Startup RNG draw consumption differs from the randomized case.',
              scope='Diagnostic comparison, not deployment/training robustness evidence.')


def summarize_torso_support(trace):
  import numpy as np
  valid=trace['valid']&~trace['terminated']&~trace['timeout']
  rows=[]
  for seg in np.unique(trace['segment']):
    mask=(trace['segment']==seg)[:,None]&valid
    load=trace['torso_normal_load_fraction'][mask]
    force=trace['torso_mean_normal_force_n'][mask]
    rows.append(dict(segment=int(seg),valid_samples=int(mask.sum()),
                     mean_normal_load_fraction=float(load.mean()) if load.size else None,
                     fraction_above_10pct_bodyweight=float(np.mean(load>.1)) if load.size else None,
                     mean_normal_force_n=float(force.mean()) if force.size else None,
                     maximum_substep_normal_force_n=float(trace['torso_peak_normal_force_n'][mask].max()) if force.size else None))
  return dict(contract='flat_torso_support_audit_v1',
              meaning='Pre-state contact history from preceding policy interval; warmup and terminated/reset rows excluded. Actual forces, independent of signed-distance tolerance.',segments=rows)
