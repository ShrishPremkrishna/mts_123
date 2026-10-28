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
        np.testing.assert_array_equal(capture['jump'], np.asarray(policy['jump_slot']['jump_table']))
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
    cfg = SimpleNamespace(events={'reset_jump_schedule': event}, episode_length_s=10.)
    args = SimpleNamespace(command='evaluate', task='Mjlab-Mystery-Flat-Pupper-v3', seconds=10., warmup=2., reset_each_command=False)
    coverage.apply_coverage_schedule(args, cfg)
    assert cfg.episode_length_s >= 30.
    assert args.reset_each_command and event.params['grid_s'] == .75
    assert event.params['p_no_jump'] == 0. and event.params['min_start_s'] > args.warmup
    for field, invalid in [('seconds', .2), ('warmup', 7), ('command', 'train'), ('task', 'Mjlab-MixedGaits-Flat-Pupper-v3')]:
        bad = SimpleNamespace(**vars(args));setattr(bad, field, invalid)
        with pytest.raises(ValueError):
            coverage.apply_coverage_schedule(bad, cfg)


def test_class_seed_loader_matches_actual_training_and_exported_reference_runtime(tmp_path, monkeypatch):
    import torch
    from mjlab.tasks.pupper_gait.mdp import gait_reference, gait, mixed_jump
    from mjlab.tasks.pupper_gait.export import mixed_jump_reference_offset_numpy
    seed=load('prepare_class_jump_seed');out=tmp_path/'runtime-seed';seed.prepare(ROOT,out)
    p=json.loads((ROOT/'pupper_gait_deploy/policies/default_policy.json').read_text())
    monkeypatch.setattr(gait_reference,'_CAPTURED_JUMP_FILE',out/'captured_jump_gait.npz')
    monkeypatch.setattr(gait_reference,'_TABLE_CACHE',{})
    monkeypatch.setattr(gait,'_TABLE_CACHE',{})
    # Exercise the actual capture loader, including its canonical order and clamp,
    # then the real training schedule/reference path. Expected values come from
    # the separately exported shipped policy, never from the builder formula.
    loaded=gait_reference.build_joint_reference_table(100,'jump')
    np.testing.assert_allclose(loaded,p['jump_slot']['jump_table'],rtol=0,atol=1e-7)
    ticks=torch.arange(0,111,dtype=torch.int64);n=len(ticks);default=torch.tensor(p['default_joint_pos'],dtype=torch.float32)
    limits=torch.tensor(np.array([[-10,10]]*12),dtype=torch.float32)
    env=SimpleNamespace(device=torch.device('cpu'),num_envs=n,step_dt=.01,episode_length_buf=ticks,
                        scene={'robot':SimpleNamespace(data=SimpleNamespace(default_joint_pos=default.expand(n,12),joint_pos_limits=limits[None]))},
                        command_manager=SimpleNamespace(get_command=lambda _:torch.zeros(n,3)))
    starts=torch.full((n,3),float('inf'));starts[:,0]=0;setattr(env,'_pupper_mixed_jump_slots',starts)
    j=p['jump_slot'];g=p['gait_reference']
    actual=mixed_jump.mixed_jump_reference_offset(env,'twist',1/.75,.1,100,.5,j['playback_s'],j['active_s'],j['cross_fade_s']).numpy()
    t=(ticks.float()*.01).numpy();zeros=np.zeros(n)
    expected=mixed_jump_reference_offset_numpy(g,j,np.asarray(p['default_joint_pos']),t,zeros,zeros,zeros,zeros)
    np.testing.assert_allclose(actual,expected,rtol=0,atol=2e-6)


def test_coverage_short_randomized_horizon_still_schedules_real_slots():
    import torch
    from mjlab.tasks.pupper_gait.mdp.mixed_jump import reset_jump_schedule, _JUMP_SCHEDULE_ATTR
    coverage=load('mystery_coverage_cli')
    cfg=SimpleNamespace(events={'reset_jump_schedule':SimpleNamespace(params={'grid_s':.75})},episode_length_s=20.)
    args=SimpleNamespace(command='evaluate',task='Mjlab-Mystery-Flat-Pupper-v3',seconds=10.,warmup=2.,reset_each_command=False)
    coverage.apply_coverage_schedule(args,cfg)
    env=SimpleNamespace(num_envs=32,device='cpu',step_dt=.02,max_episode_length=cfg.episode_length_s/.02)
    torch.manual_seed(101);reset_jump_schedule(env,None,**cfg.events['reset_jump_schedule'].params)
    slots=getattr(env,_JUMP_SCHEDULE_ATTR)
    assert torch.isfinite(slots).any(dim=1).all()
    first=slots[:,0]
    assert (first>2.).all() and (first+.8<10.).all()
