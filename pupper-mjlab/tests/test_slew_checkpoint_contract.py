"""Saved slew actuation must survive a fresh standalone export without old paths."""
import copy
from dataclasses import asdict, replace
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.scripts import export_pupper_policy as exporter
from mjlab.scripts import pupper_policy_audit as auditor
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
from mjlab.tasks.pupper.deployment_contract import apply_deployment_target_clamp, apply_driver_target_clamp
from mjlab.tasks.pupper.target_slew import (
    CHECKPOINT_SLEW_KEY, SlewLimitedJointPositionActionCfg, apply_position_target_slew,
    checkpoint_infos_with_position_target_slew, checkpoint_position_target_slew,
    restore_position_target_slew, target_slew_metadata, validate_position_target_slew_metadata)

TASK = 'Mjlab-SpiderWalk-Flat-Pupper-v3'
LIMITS = Path(__file__).resolve().parents[2]/'foundation/configs/class-driver-joint-limits.json'


def slew_config():
    cfg = load_env_cfg(TASK, play=True)
    cfg.scene.num_envs = 1
    apply_deployment_target_clamp(cfg)
    contract = apply_driver_target_clamp(cfg, LIMITS)
    metadata = apply_position_target_slew(cfg, 15., contract)
    return cfg, metadata


@pytest.fixture(scope='module')
def runtime_metadata():
    cfg, _ = slew_config()
    env = ManagerBasedRlEnv(cfg, device='cpu')
    try:
        yield target_slew_metadata(env)
    finally:
        env.close()


def test_portable_restore_preserves_bounds_order_and_fixed_latency(runtime_metadata):
    cfg = load_env_cfg(TASK, play=True)
    original = copy.deepcopy(cfg.actions['joint_pos'])
    restored = restore_position_target_slew(cfg, runtime_metadata)
    assert restored == runtime_metadata
    term = cfg.actions['joint_pos']
    assert isinstance(term, SlewLimitedJointPositionActionCfg)
    assert term.clip == dict(zip(runtime_metadata['joint_names'], zip(runtime_metadata['lower_limits'],runtime_metadata['upper_limits'])))
    assert term.scale == original.scale and term.use_default_offset == original.use_default_offset
    assert term.max_rate_rad_s == 15 and term.max_step_s == .02
    assert term.driver_limits_sha256 == runtime_metadata['driver_limits_sha256']
    assert all(a.delay_min_lag == a.delay_max_lag == 4 for a in cfg.scene.entities['robot'].articulation.actuators)
    assert load_env_cfg(TASK, play=True).actions['joint_pos'] == original


@pytest.mark.parametrize('defect', ['order','nan','bool_bound','wide','empty','rate','rate_type','dt','hash','missing','extra'])
def test_invalid_metadata_fails_before_mutating_config(runtime_metadata, defect):
    metadata = copy.deepcopy(runtime_metadata)
    if defect == 'order': metadata['joint_names'].reverse()
    elif defect == 'nan': metadata['lower_limits'][0] = float('nan')
    elif defect == 'bool_bound': metadata['lower_limits'][0] = False
    elif defect == 'wide': metadata['upper_limits'][0] = 999.
    elif defect == 'empty': metadata['upper_limits'][0] = metadata['lower_limits'][0]
    elif defect == 'rate': metadata['max_rate_rad_s'] = 15.01
    elif defect == 'rate_type': metadata['max_rate_rad_s'] = '15'
    elif defect == 'dt': metadata['max_step_s'] = .021
    elif defect == 'hash': metadata['driver_limits_sha256'] = 'not-a-sha'
    elif defect == 'missing': metadata.pop('max_step_s')
    else: metadata['historical_path'] = '/a/no/longer/existing/driver.json'
    cfg = load_env_cfg(TASK, play=True)
    old = copy.deepcopy(cfg.actions['joint_pos'])
    with pytest.raises(ValueError): restore_position_target_slew(cfg, metadata)
    assert cfg.actions['joint_pos'] == old


@pytest.mark.parametrize('change', ['random_delay','no_delay','other_timestep'])
def test_restore_refuses_changed_latency(runtime_metadata, change):
    cfg = load_env_cfg(TASK, play=True)
    robot = cfg.scene.entities['robot']
    if change == 'other_timestep': cfg.sim.mujoco.timestep = .002
    else:
        minimum, maximum = (0,4) if change == 'random_delay' else (0,0)
        robot.articulation = replace(robot.articulation, actuators=tuple(
            replace(a,delay_min_lag=minimum,delay_max_lag=maximum) for a in robot.articulation.actuators))
    with pytest.raises(ValueError,match='fixed Pupper'): restore_position_target_slew(cfg,runtime_metadata)


def test_save_info_stamping_is_nonmutating_and_legacy_infos_unchanged(monkeypatch,runtime_metadata):
    import mjlab.tasks.pupper.target_slew as slew
    env=object();original={'unrelated':{'experiment':'keep'}}
    monkeypatch.setattr(slew,'target_slew_metadata',lambda _:runtime_metadata)
    stamped=checkpoint_infos_with_position_target_slew(env,original)
    assert stamped[CHECKPOINT_SLEW_KEY]==runtime_metadata and CHECKPOINT_SLEW_KEY not in original
    with pytest.raises(ValueError,match='disagree'):
        checkpoint_infos_with_position_target_slew(env,{CHECKPOINT_SLEW_KEY:{'wrong':True}})
    monkeypatch.setattr(slew,'target_slew_metadata',lambda _:None)
    assert checkpoint_infos_with_position_target_slew(env,original) is original
    assert checkpoint_infos_with_position_target_slew(env,None) is None
    with pytest.raises(ValueError,match='stale'):
        checkpoint_infos_with_position_target_slew(env,stamped)


def test_checkpoint_missing_metadata_is_legacy_but_present_invalid_is_refused(tmp_path):
    path=tmp_path/'checkpoint.pt'
    for infos in (None,{}, {'unrelated':1}):
        torch.save({'infos':infos},path)
        assert checkpoint_position_target_slew(path) is None
    torch.save({'infos':{CHECKPOINT_SLEW_KEY:None}},path)
    with pytest.raises(ValueError,match='schema'):checkpoint_position_target_slew(path)


def test_standalone_export_refuses_invalid_metadata_before_env_creation(tmp_path, monkeypatch):
    path=tmp_path/'invalid.pt'
    torch.save({'infos':{CHECKPOINT_SLEW_KEY:None}},path)
    monkeypatch.setattr(exporter,'ManagerBasedRlEnv',lambda **_:pytest.fail('invalid metadata reached physics creation'))
    with pytest.raises(ValueError,match='schema'):
        exporter.run_export(TASK,exporter.ExportConfig(checkpoint_file=str(path),device='cpu'))


@pytest.mark.parametrize('case', ['legacy_to_slew','matching_slew','missing_slew','different_slew'])
def test_actual_runner_load_enforces_declared_contract_only(tmp_path,monkeypatch,runtime_metadata,case):
    import mjlab.tasks.pupper.target_slew as slew
    from mjlab.tasks.pupper.rl.runner import PupperOnPolicyRunner
    from mjlab.tasks.velocity.rl.runner import VelocityOnPolicyRunner
    path=tmp_path/'model.pt'
    infos={} if case=='legacy_to_slew' else {CHECKPOINT_SLEW_KEY:runtime_metadata}
    torch.save({'infos':infos},path)
    actual=None if case=='missing_slew' else copy.deepcopy(runtime_metadata)
    if case=='different_slew':actual['max_rate_rad_s']=14.
    monkeypatch.setattr(slew,'target_slew_metadata',lambda _:actual)
    loaded=[]
    def parent_load(self,path,**kwargs):
        loaded.append((path,kwargs));return {'original_parent_result':True}
    monkeypatch.setattr(VelocityOnPolicyRunner,'load',parent_load)
    runner=object.__new__(PupperOnPolicyRunner)
    runner.env=SimpleNamespace(unwrapped=object())
    if case in ('missing_slew','different_slew'):
        with pytest.raises(ValueError,match='differs from the active environment'):
            runner.load(str(path),load_cfg={'actor':True},map_location='cpu')
        assert loaded==[]  # Reject before any actor/optimizer/curriculum mutation.
    else:
        assert runner.load(str(path),load_cfg={'actor':True},map_location='cpu')=={'original_parent_result':True}
        assert loaded==[(str(path),{'load_cfg':{'actor':True},'strict':True,'map_location':'cpu'})]


def test_actual_save_standalone_export_and_audit_roundtrip(tmp_path):
    import json
    cfg,_=slew_config();env=ManagerBasedRlEnv(cfg,device='cpu')
    checkpoint=tmp_path/'model.pt';agent=load_rl_cfg(TASK);agent.upload_model=False;agent.logger='tensorboard'
    try:
        wrapped=RslRlVecEnvWrapper(env,clip_actions=agent.clip_actions)
        runner=load_runner_cls(TASK)(wrapped,asdict(agent),device='cpu')
        metadata=target_slew_metadata(env)
        runner.save(str(checkpoint),infos={'purpose':'contract test; no training occurred'})
        saved=torch.load(checkpoint,map_location='cpu',weights_only=False)
        assert saved['infos'][CHECKPOINT_SLEW_KEY]==metadata
        assert saved['infos']['purpose'].startswith('contract test')
    finally:env.close()
    policy_path=exporter.run_export(TASK,exporter.ExportConfig(checkpoint_file=str(checkpoint),output=str(tmp_path/'policy.json'),device='cpu'))
    policy=json.loads(policy_path.read_text())
    assert policy['position_target_slew']==metadata
    assert policy['required_controller_features']==['position_target_slew_v1']
    assert policy['action_types']==['position']*12
    args=SimpleNamespace(task=TASK,policy=policy_path,checkpoint=checkpoint,motion=None,training_config=None,seed=42,steps=2,output=tmp_path/'audit.json')
    review=auditor.audit(args)
    assert review['passed'] and review['checkpoint_slew_export_differences']==[]
    policy['position_target_slew']['max_rate_rad_s']=14.
    policy_path.write_text(json.dumps(policy))
    review=auditor.audit(args)
    assert not review['passed'] and review['checkpoint_slew_export_differences']
    # A genuine old checkpoint still exports with no invented slew declaration.
    del saved['infos'][CHECKPOINT_SLEW_KEY]
    legacy=tmp_path/'legacy.pt';torch.save(saved,legacy)
    legacy_out=exporter.run_export(TASK,exporter.ExportConfig(checkpoint_file=str(legacy),output=str(tmp_path/'legacy.json'),device='cpu'))
    legacy_policy=json.loads(legacy_out.read_text())
    assert 'position_target_slew' not in legacy_policy
    assert 'required_controller_features' not in legacy_policy
    assert 'action_types' not in legacy_policy
