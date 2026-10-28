"""COM continuation must be a controlled experiment, never a relaxed limit test."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('sit_comx', ROOT / 'foundation/scripts/colab_sit_comx.py')
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


def test_plan_preserves_initial_policy_reference_limits_and_matched_scores(tmp_path, monkeypatch):
    repo = tmp_path / 'repo'; inputs = tmp_path / 'inputs'; inputs.mkdir()
    driver = repo / 'foundation/configs/class-driver-joint-limits.json'
    driver.parent.mkdir(parents=True); driver.write_bytes(b'fixed limits')
    for name in runner.EXPECTED:
        (inputs / name).write_bytes(name.encode())
    (inputs / 'prior-foundation-run.json').write_text(json.dumps({'source': {'files': {}}}))
    monkeypatch.setattr(runner, 'EXPECTED', {name: runner.sha(inputs/name) for name in runner.EXPECTED})
    monkeypatch.setattr(runner, 'DRIVER_SHA', runner.sha(driver))
    monkeypatch.setattr(runner, 'check_source', lambda *_: 'fixed-revision')
    result = runner.plan(repo, inputs, tmp_path/'out', 'fixed-revision')
    command = result['training_command']
    assert command[command.index('--tracking-com-x-half-range')+1] == '.02'
    assert command[command.index('--iterations')+1] == '1000'
    assert command[command.index('--seed')+1] == '42'
    assert command[command.index('--initial-checkpoint')+1].endswith('inputs/retained3998.pt')
    assert command[command.index('--motion')+1].endswith('inputs/sit_stay_shake.npz')
    assert '--deployment-target-clamp' in command and '--tracking-ground-hulls' in command
    assert '--driver-limits' in command and '--compose-com-randomization' in command
    assert '--linear-tracking-std' not in command
    assert len(result['comparisons']) == 3
    assert [case['com_x_m'] for case in result['held_out_cases']] == [-.015, .015, -.025, .025]
    (inputs / 'retained3998.pt').write_bytes(b'wrong checkpoint')
    with pytest.raises(ValueError, match='input hash mismatch'):
        runner.plan(repo, inputs, tmp_path/'out', 'fixed-revision')


def saved_run(tmp_path):
    inputs = tmp_path/'inputs'; inputs.mkdir()
    run = tmp_path/'run'; (run/'params').mkdir(parents=True)
    (run/'model_4997.pt').write_bytes(b'candidate fixture')
    prior = dict(state='trained_unvalidated', profile_config={'task': 'tracking'}, seed=42,
                 num_envs=4096, iterations=1000, reward_parameter_overrides={},
                 com_randomization_contract={'contract': 'composed'}, collision_contract={'contract': 'groundhulls'},
                 actuation_contract={'effective_limits': {'joint': [-1, 1]},
                                     'source': {'path': '/old/limits.json', 'sha256': runner.DRIVER_SHA}})
    (inputs/'prior-foundation-run.json').write_text(json.dumps(prior))
    env = {'events': {'base_com': {'params': {'ranges': {0: [-.01,.01], 1: [-.01,.01], 2: [-.01,.01]}}}},
           'commands': {'motion': {'motion_file': '/old/reference.npz'}},
           'rewards': {'example': {'weight': 1.0}}, 'limit': [-1.,1.]}
    (inputs/'prior-env.yaml').write_text(yaml.safe_dump(env))
    candidate_env = copy.deepcopy(env)
    candidate_env['events']['base_com']['params']['ranges'][0] = [-.02,.02]
    candidate_env['commands']['motion']['motion_file'] = '/new/reference.npz'
    (run/'params/env.yaml').write_text(yaml.safe_dump(candidate_env))
    candidate = copy.deepcopy(prior)
    candidate.update(initial_checkpoint_sha256=runner.EXPECTED['retained3998.pt'],
                     motion_sha256=runner.EXPECTED['sit_stay_shake.npz'],
                     checkpoint='model_4997.pt', checkpoint_sha256=runner.sha(run/'model_4997.pt'),
                     domain_randomization_overrides={'base_com.ranges': {'value': {'0': [-.02,.02], '1': [-.01,.01], '2': [-.01,.01]}}})
    candidate['actuation_contract']['source']['path'] = '/new/limits.json'
    manifest = run/'foundation-run.json'; manifest.write_text(json.dumps(candidate))
    return manifest, inputs, candidate, candidate_env


def test_saved_config_audit_allows_only_motion_path_and_x_range(tmp_path):
    manifest, inputs, _, _ = saved_run(tmp_path)
    checkpoint, result = runner.audit_training(manifest, inputs)
    assert checkpoint.name == 'model_4997.pt'
    assert len(result['saved_env_match_except']) == 2


@pytest.mark.parametrize('field', ['reward', 'yz_range', 'driver_limits', 'checkpoint', 'reference'])
def test_saved_config_audit_rejects_changed_experiment_or_limits(tmp_path, field):
    manifest, inputs, candidate, env = saved_run(tmp_path)
    if field == 'reward':
        env['rewards']['example']['weight'] = 2.
    elif field == 'yz_range':
        env['events']['base_com']['params']['ranges'][1] = [-.02,.02]
    elif field == 'driver_limits':
        candidate['actuation_contract']['effective_limits']['joint'] = [-2,2]
    elif field == 'checkpoint':
        candidate['initial_checkpoint_sha256'] = 'wrong'
    else:
        candidate['motion_sha256'] = 'wrong'
    manifest.write_text(json.dumps(candidate))
    (manifest.parent/'params/env.yaml').write_text(yaml.safe_dump(env))
    with pytest.raises(ValueError):
        runner.audit_training(manifest, inputs)


def test_python_yaml_tags_are_inert(tmp_path):
    config = tmp_path/'tagged.yaml'
    config.write_text('pair: !!python/tuple [1, 2]\ncall: !!python/name:some.module.function \'\'\n')
    assert runner.yaml_data(config) == {'pair': [1,2], 'call': {'python_tag': 'name:some.module.function', 'value': ''}}
