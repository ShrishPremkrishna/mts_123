"""Opt-in simulator actuation matching the deployed Pupper position controller.

The controller preserves the policy's raw action in its observation history, but
clips default + scale * action to the joint limits before commanding the motor.
Use the action term's absolute-target clip, never pre-clip the policy action.
Legacy training remains unchanged unless this helper is explicitly requested.
"""
from __future__ import annotations

import mujoco

from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES, get_spec
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import requires_model_fields, RecomputeLevel


def deployment_target_limits() -> dict[str, tuple[float, float]]:
  model = get_spec().compile()
  return {name: tuple(float(v) for v in model.jnt_range[
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]) for name in JOINT_NAMES}


def apply_deployment_target_clamp(cfg) -> dict:
  action = cfg.actions.get("joint_pos")
  if not isinstance(action, JointPositionActionCfg) or action.entity_name != "robot":
    raise ValueError("Deployment target clamp requires Pupper's robot joint-position action")
  limits = deployment_target_limits()
  action.clip = limits
  return {"contract": "deployment_joint_target_clamp_v1", "limits": limits,
          "order": "raw action retained; affine target; absolute joint clipping; encoder-bias correction; actuator latency",
          "raw_action_history_preserved": True}


def _require_density_only(params):
  for key, value in params.items():
    if key.endswith("_range") and key != "alpha_range" and value is not None:
      if any(float(v) != 0.0 for v in value):
        raise ValueError(f"COM composition cannot overwrite a nonzero pseudo-inertia transform: {key}")


@requires_model_fields("body_mass", "body_ipos", "body_inertia", "body_iquat", recompute=RecomputeLevel.set_const)
def density_randomization_preserving_com(env, env_ids, *, asset_cfg, **params):
  """Preserve existing COM while drawing exactly the original density samples."""
  from mjlab.envs.mdp import dr
  import torch

  _require_density_only(params)
  asset = env.scene[asset_cfg.name]
  ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int) if env_ids is None else env_ids.to(env.device, dtype=torch.int)
  bodies = asset.indexing.body_ids[asset_cfg.body_ids]
  grid, body_grid = torch.meshgrid(ids, bodies, indexing="ij")
  com = env.sim.model.body_ipos[grid, body_grid].clone()
  dr.pseudo_inertia(env, env_ids, asset_cfg=asset_cfg, **params)
  env.sim.model.body_ipos[grid, body_grid] = com


def compose_pupper_com_randomization(cfg) -> dict:
  """Opt in to density changes preserving the independently sampled COM offset.

  Keep event order and RNG draw order unchanged. The density-only wrapper restores
  the existing COM after pseudo_inertia reconstructs it from defaults; independent
  mass, gain, friction and bias samples remain paired with the legacy same-seed run.
  """
  from mjlab.envs.mdp import dr

  inertia = cfg.events.get("inertial")
  com = cfg.events.get("base_com")
  if inertia is None or com is None:
    raise ValueError("COM composition requires both inertial and base_com events")
  if inertia.func not in (dr.pseudo_inertia, density_randomization_preserving_com) or com.func is not dr.body_com_offset:
    raise ValueError("COM composition supports the known Pupper pseudo_inertia/body_com_offset pair")
  if inertia.mode != "startup" or com.mode != "startup":
    raise ValueError("COM composition requires both events in startup mode")
  if com.params.get("operation", "add") != "add":
    raise ValueError("COM composition requires a default-relative additive base COM event")
  _require_density_only(inertia.params)
  inertia.func = density_randomization_preserving_com
  return {"contract": "density_preserving_independent_base_com_v1", "rng_draw_order_preserved": True,
          "event_order": list(cfg.events), "effective_at": "startup; sampled once per world, retained across resets",
          "scope": "density-only pseudo-inertia retaining the already sampled additive body-frame COM offset"}
