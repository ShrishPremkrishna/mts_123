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
