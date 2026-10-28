"""Driver clipping is a second named-coordinate bound, independent of joint physics."""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
from mjlab.tasks.pupper.deployment_contract import (
    apply_deployment_target_clamp, apply_driver_target_clamp,
    deployment_target_limits, load_driver_target_limits)
from mjlab.scripts.pupper_locomotion import evaluation_episode_length_s

LIMITS = Path(__file__).resolve().parents[2] / 'foundation/configs/class-driver-joint-limits.json'


def test_actual_action_composes_two_clamps_and_retains_policy_history():
    cfg = load_env_cfg('Mjlab-StableGait-Flat-Pupper-v3', play=True)
    cfg.scene.num_envs = 1
    with pytest.raises(ValueError, match='before the driver'):
        apply_driver_target_clamp(cfg, LIMITS)
    apply_deployment_target_clamp(cfg)
    contract = apply_driver_target_clamp(cfg, LIMITS)
    assert contract['source']['sha256'] == hashlib.sha256(LIMITS.read_bytes()).hexdigest()
    assert contract['source']['installed_robot_verified'] is False
    env = ManagerBasedRlEnv(cfg, device='cpu')
    try:
        term = env.action_manager.get_term('joint_pos')
        raw = torch.linspace(-10, 10, 12)[None]
        env.action_manager.process_action(raw)
        controller = torch.tensor([deployment_target_limits()[n] for n in term.target_names])
        driver = torch.tensor([contract['driver_limits'][n] for n in term.target_names])
        requested = term.offset + term.scale * raw
        first = torch.clamp(requested, controller[:, 0], controller[:, 1])
        second = torch.clamp(first, driver[:, 0], driver[:, 1])
        torch.testing.assert_close(term._processed_actions, second)
        assert torch.any(first != second)
        torch.testing.assert_close(term.raw_action, raw)
        torch.testing.assert_close(env.action_manager.action, raw)
        # Command clipping must not move mechanical stops or alter observation scales.
        np.testing.assert_allclose(env.scene['robot'].data.joint_pos_limits[0, term.target_ids], controller, atol=1e-7)
    finally:
        env.close()


@pytest.mark.parametrize('fault', ['missing_joint', 'extra_joint', 'nonfinite', 'bool', 'reversed', 'units'])
def test_malformed_driver_contract_refused_before_simulation(tmp_path, fault):
    data = copy.deepcopy(json.loads(LIMITS.read_text()))
    name = next(iter(data['joint_limits']))
    if fault == 'missing_joint': del data['joint_limits'][name]
    elif fault == 'extra_joint': data['joint_limits']['unknown'] = [-1, 1]
    elif fault == 'nonfinite': data['joint_limits'][name][0] = float('nan')
    elif fault == 'bool': data['joint_limits'][name][0] = False
    elif fault == 'reversed': data['joint_limits'][name] = [1, -1]
    else: data['units'] = 'degrees'
    path = tmp_path / 'limits.json'; path.write_text(json.dumps(data))
    with pytest.raises(ValueError): load_driver_target_limits(path)


def test_long_randomized_custom_sweep_cannot_timeout_at_default_sequence_length():
    # Training/randomized config starts at10s;22x10s commands must remain continuous.
    assert evaluation_episode_length_s(10, 10, 22, .02) > 220
    assert evaluation_episode_length_s(10, 10.001, 22, .02) > 22 * 501 * .02
    assert evaluation_episode_length_s(1e9, 10, 22, .02) == 1e9
