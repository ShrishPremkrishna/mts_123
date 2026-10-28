"""Exercise continuous target filtering on the actual Pupper action/model interface."""
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch
from torch import nn

from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
from mjlab.tasks.pupper.deployment_contract import apply_deployment_target_clamp, apply_driver_target_clamp
from mjlab.tasks.pupper.target_slew import (
    apply_position_target_slew, target_slew_metadata, validate_slew_settings)
from mjlab.tasks.pupper.export import export_pupper_policy_from_env
from mjlab.tasks.pupper_gait.bootstrap import compare_evaluations

LIMITS = Path(__file__).resolve().parents[2]/'foundation/configs/class-driver-joint-limits.json'


@pytest.fixture
def env():
    cfg = load_env_cfg('Mjlab-StableGait-Flat-Pupper-v3', play=True)
    cfg.scene.num_envs = 2
    apply_deployment_target_clamp(cfg)
    contract = apply_driver_target_clamp(cfg, LIMITS)
    apply_position_target_slew(cfg, 15., contract)
    environment = ManagerBasedRlEnv(cfg, device='cpu')
    environment.reset()
    try:
        yield environment
    finally:
        environment.close()


def test_physics_step_slew_holds_raw_actions_and_respects_bias_and_bounds(env):
    term = env.action_manager.get_term('joint_pos')
    robot = env.scene['robot']
    raw = torch.full((2, 12), 100.)
    env.action_manager.process_action(raw)
    previous = torch.clamp(robot.data.joint_pos_biased[:, term.target_ids], term._clip[:, :, 0], term._clip[:, :, 1])
    for _ in range(env.cfg.decimation):
        term.apply_actions()
        assert torch.max(torch.abs(term.applied_actions - previous)) <= 15 * env.physics_dt + 1e-6
        assert torch.all(term.applied_actions >= term._clip[:, :, 0])
        assert torch.all(term.applied_actions <= term._clip[:, :, 1])
        torch.testing.assert_close(robot.data.joint_pos_target[:, term.target_ids],
                                   term.applied_actions - robot.data.encoder_bias[:, term.target_ids])
        previous = term.applied_actions.clone()
    torch.testing.assert_close(term.raw_action, raw)
    torch.testing.assert_close(env.action_manager.action, raw)
    assert torch.any(torch.abs(term.applied_actions - term._processed_actions) > .1)
    env.action_manager.process_action(-raw)
    term.apply_actions()
    assert torch.max(torch.abs(term.applied_actions - previous)) <= 15 * env.physics_dt + 1e-6


def test_partial_reset_seeds_current_measured_joints_without_erasing_other_world(env):
    term = env.action_manager.get_term('joint_pos')
    env.action_manager.process_action(torch.full((2, 12), -100.))
    for _ in range(8): term.apply_actions()
    old_second = term.applied_actions[1].clone()
    # Use an intentionally different simulated measured pose; no physical command.
    robot = env.scene['robot']
    pose = robot.data.default_joint_pos[:1].clone() + .1
    robot.write_joint_state_to_sim(pose, torch.zeros_like(pose), env_ids=torch.tensor([0]))
    term.reset(torch.tensor([0]))
    measured = torch.clamp(robot.data.joint_pos_biased[0, term.target_ids], term._clip[0, :, 0], term._clip[0, :, 1])
    term.apply_actions()
    bound = 15 * env.physics_dt + 1e-6
    assert torch.max(torch.abs(term.applied_actions[0] - measured)) <= bound
    assert torch.max(torch.abs(term.applied_actions[1] - old_second)) <= bound


def test_export_carries_exact_runtime_contract_and_position_only_requirement(env):
    actor = SimpleNamespace(mlp=nn.Sequential(nn.Linear(36, 12)), obs_normalizer=nn.Identity())
    policy = export_pupper_policy_from_env(actor, env)
    assert policy['position_target_slew'] == target_slew_metadata(env)
    assert policy['required_controller_features'] == ['position_target_slew_v1']
    assert policy['action_types'] == ['position'] * 12


@pytest.mark.parametrize('rate,dt', [(0,.02),(-1,.02),(15.01,.02),(float('nan'),.02),(True,.02),(15,0),(15,.021),(15,float('inf'))])
def test_invalid_slew_contract_refused(rate,dt):
    with pytest.raises(ValueError): validate_slew_settings(rate,dt)


def test_slew_requires_prior_driver_clamp_and_does_not_mutate_legacy_defaults():
    cfg = load_env_cfg('Mjlab-StableGait-Flat-Pupper-v3', play=True)
    original = copy.deepcopy(cfg.actions['joint_pos'])
    with pytest.raises(ValueError, match='driver clamps'):
        apply_position_target_slew(cfg, 15., None)
    assert cfg.actions['joint_pos'] == original


def test_comparison_refuses_different_slew_physics():
    first = {'provenance':{}, 'segments':[]}
    second = {'provenance':{'position_target_slew_contract':{'max_rate_rad_s':15}},'segments':[]}
    with pytest.raises(ValueError, match='position_target_slew_contract'):
        compare_evaluations(first, second)
