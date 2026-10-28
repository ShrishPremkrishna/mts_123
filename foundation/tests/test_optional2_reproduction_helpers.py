"""Keep identified class templates separate from measured capture evidence."""
from pathlib import Path
from types import SimpleNamespace
import importlib.util
import json
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]

def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'foundation/scripts'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_actual_class_seed_preserves_exact_motion_and_does_not_invent_touchdown(tmp_path):
    seed = load('prepare_class_jump_seed')
    out = tmp_path/'class-seed'
    report = seed.prepare(ROOT, out)
    policy = json.loads((ROOT/'pupper_gait_deploy/policies/default_policy.json').read_text())
    with np.load(out/'captured_jump_gait.npz', allow_pickle=False) as capture:
        np.testing.assert_array_equal(capture['jump'], np.asarray(policy['jump_slot']['jump_table'])+np.asarray(policy['default_joint_pos']))
        assert 'touchdown_s' not in capture.files
        assert 'not_measured_capture' in str(capture['metadata_json'])
    assert not report['physical_flight_validated'] and not report['hardware_approved']
    assert report['nominal_timing']['touchdown_is_measured'] is False
    assert report['reference_sha256'] == seed.sha(out/'captured_jump_gait.npz')
    with pytest.raises(FileExistsError):
        seed.prepare(ROOT, out)


def test_class_seed_refuses_unreviewed_source_identity_without_output(tmp_path):
    seed = load('prepare_class_jump_seed')
    policy = tmp_path/'pupper_gait_deploy/policies/default_policy.json'
    policy.parent.mkdir(parents=True)
    policy.write_text('{}')
    with pytest.raises(ValueError, match='identity changed'):
        seed.prepare(tmp_path, tmp_path/'output')
    assert not (tmp_path/'output').exists()


def test_coverage_protocol_cannot_silently_claim_short_or_warmup_only_slots():
    coverage = load('mystery_coverage_cli')
    event = SimpleNamespace(params={'grid_s': .75, 'p_no_jump': .15})
    cfg = SimpleNamespace(events={'reset_jump_schedule': event})
    args = SimpleNamespace(command='evaluate', task='Mjlab-Mystery-Flat-Pupper-v3', seconds=10., warmup=2., reset_each_command=False)
    coverage.apply_coverage_schedule(args, cfg)
    assert args.reset_each_command and event.params['grid_s'] == .75
    assert event.params['p_no_jump'] == 0. and event.params['min_start_s'] > args.warmup
    for field, invalid in [('seconds', .2), ('warmup', 7), ('command', 'train'), ('task', 'Mjlab-MixedGaits-Flat-Pupper-v3')]:
        bad = SimpleNamespace(**vars(args));setattr(bad, field, invalid)
        with pytest.raises(ValueError):
            coverage.apply_coverage_schedule(bad, cfg)
