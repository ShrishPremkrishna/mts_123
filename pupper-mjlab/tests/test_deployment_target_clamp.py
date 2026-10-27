"""The opt-in deployment contract clips motor targets and preserves raw action history."""
import numpy as np
import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
from mjlab.tasks.pupper.deployment_contract import apply_deployment_target_clamp, deployment_target_limits


def test_opt_in_does_not_change_legacy_task_defaults():
  legacy = load_env_cfg("Mjlab-StableGait-Flat-Pupper-v3", play=True)
  changed = load_env_cfg("Mjlab-StableGait-Flat-Pupper-v3", play=True)
  metadata = apply_deployment_target_clamp(changed)
  assert legacy.actions["joint_pos"].clip is None
  assert load_env_cfg("Mjlab-StableGait-Flat-Pupper-v3").actions["joint_pos"].clip is None
  assert changed.actions["joint_pos"].clip == deployment_target_limits()
  assert metadata["raw_action_history_preserved"]


def test_real_action_term_matches_cpp_affine_then_clamp_without_clipping_history():
  cfg = load_env_cfg("Mjlab-StableGait-Flat-Pupper-v3", play=True)
  cfg.scene.num_envs = 1
  apply_deployment_target_clamp(cfg)
  env = ManagerBasedRlEnv(cfg, device="cpu")
  try:
    term = env.action_manager.get_term("joint_pos")
    raw = torch.linspace(-10, 10, 12)[None]
    env.action_manager.process_action(raw)
    ordered_limits = torch.tensor([deployment_target_limits()[name] for name in term.target_names])
    expected = torch.clamp(term.offset + term.scale * raw, ordered_limits[:, 0], ordered_limits[:, 1])
    torch.testing.assert_close(term._processed_actions, expected)
    torch.testing.assert_close(term.raw_action, raw)
    torch.testing.assert_close(env.action_manager.action, raw)
    assert not torch.equal(term._processed_actions, term.offset + term.scale * raw)
    np.testing.assert_allclose(ordered_limits, env.scene["robot"].data.joint_pos_limits[0, term.target_ids], atol=1e-7)
  finally:
    env.close()
