"""Sensor/command coordinate parity and explicit historical-run compatibility."""
import copy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch
from torch import nn

from mjlab.tasks.pupper import encoder_observation as encoder
from mjlab.tasks.pupper.mdp.observations import pupper_actor_obs
from mjlab.tasks.registry import load_env_cfg


def fake_env(bias=.013):
    q=torch.arange(24,dtype=torch.float32).reshape(2,12)/100
    b=torch.full_like(q,bias);default=torch.ones_like(q)*.1
    data=SimpleNamespace(joint_pos=q,joint_pos_biased=q+b,encoder_bias=b,
        default_joint_pos=default,root_link_ang_vel_b=torch.zeros(2,3),
        projected_gravity_b=torch.tensor([[0.,0.,-1.]]).repeat(2,1))
    robot=SimpleNamespace(data=data)
    return SimpleNamespace(num_envs=2,scene={'robot':robot},command_manager=SimpleNamespace(
        get_command=lambda _:torch.zeros(2,3)),action_manager=SimpleNamespace(action=torch.zeros(2,12)))


def test_true_legacy_and_measured_actor_coordinate_with_unchanged_reference():
    env=fake_env();q=env.scene['robot'].data
    old=pupper_actor_obs(env,add_noise=False)
    measured=pupper_actor_obs(env,add_noise=False,encoder_observation_mode='deployment_measured')
    assert old.shape==measured.shape==(2,36)
    torch.testing.assert_close(old[:,12:24],q.joint_pos-q.default_joint_pos)
    torch.testing.assert_close(measured[:,12:24],q.joint_pos+q.encoder_bias-q.default_joint_pos)
    expected=torch.zeros_like(old);expected[:,12:24]=q.encoder_bias
    torch.testing.assert_close(measured-old,expected,atol=2e-8,rtol=1e-5)
    assert torch.equal(q.default_joint_pos,torch.full((2,12),.1))


def test_noise_rng_and_zero_bias_equivalence():
    for bias in (0.,.013):
        env=fake_env(bias)
        torch.manual_seed(123);old=pupper_actor_obs(env)
        torch.manual_seed(123);new=pupper_actor_obs(env,encoder_observation_mode='deployment_measured')
        expected=old.clone();expected[:,12:24]+=bias
        torch.testing.assert_close(new,expected,atol=3e-8,rtol=1e-5)
        if bias==0:assert torch.equal(new,old)
    with pytest.raises(ValueError,match='Encoder observation mode'):
        pupper_actor_obs(fake_env(),encoder_observation_mode='guess')


def test_measured_observation_and_actual_position_action_use_bias_once():
    from mjlab.envs.mdp.actions import JointPositionAction
    env=fake_env();robot=env.scene['robot'];u=robot.data.joint_pos_biased.clone()
    applied=[];robot.set_joint_position_target=lambda value,joint_ids:applied.append(value.clone())
    action=SimpleNamespace(_entity=robot,_target_ids=slice(None),_processed_actions=u)
    JointPositionAction.apply_actions(action)
    torch.testing.assert_close(applied[0],u-robot.data.encoder_bias)
    torch.testing.assert_close(applied[0],robot.data.joint_pos)
    seen=pupper_actor_obs(env,add_noise=False,encoder_observation_mode='deployment_measured')[:,12:24]
    torch.testing.assert_close(seen+robot.data.default_joint_pos,u)


@pytest.mark.parametrize('task',['Mjlab-VelocityFS-Flat-Pupper-v3','Mjlab-SpiderWalk-Flat-Pupper-v3',
    'Mjlab-StableGait-Flat-Pupper-v3','Mjlab-MixedGaits-Flat-Pupper-v3',
    'Mjlab-StableGait-Bumpy-Pupper-v3','Mjlab-Mystery-Flat-Pupper-v3'])
def test_configuration_threads_all_registered_actor_families_without_changing_physics(task):
    cfg=load_env_cfg(task,play=True);old=copy.deepcopy(cfg);before=encoder.metadata(cfg)
    assert before['mode']=='legacy_true'
    encoder.apply_mode(cfg,'deployment_measured')
    assert cfg.actions==old.actions and cfg.events==old.events
    assert cfg.observations['actor'].history_length==old.observations['actor'].history_length
    term=cfg.observations['actor'].terms['policy']
    assert term.params['encoder_observation_mode']=='deployment_measured'
    if term.func.__name__=='pupper_gait_shared_obs':
        assert cfg.observations['critic'].terms['policy'].params['encoder_observation_mode']=='deployment_measured'
    else:assert cfg.observations['critic']==old.observations['critic']
    assert encoder.metadata(old)['mode']=='legacy_true'


def checkpoint(path,mode=None,marker=0):
    infos=None if mode is None else {encoder.CHECKPOINT_KEY:encoder.contract(mode)}
    torch.save({'infos':infos,'marker':marker},path)
    return path


@pytest.mark.parametrize('mode',[None,'legacy_true','deployment_measured'])
def test_checkpoint_mode_inheritance_and_exact_export_restore(tmp_path,mode):
    path=checkpoint(tmp_path/'policy.pt',mode);cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True)
    context=encoder.configure(cfg,checkpoint=path,purpose='export')
    assert context['active_observation_mode']==(mode or 'legacy_true')
    assert context['checkpoint_mode_tagged']==(mode is not None)
    encoder.check_load(SimpleNamespace(cfg=cfg),path)
    assert encoder.checkpoint_infos(SimpleNamespace(cfg=cfg),None)[encoder.CHECKPOINT_KEY]==encoder.contract(mode or 'legacy_true')


@pytest.mark.parametrize('purpose',['training','evaluation'])
def test_mode_change_requires_explicit_bound_checkpoint_permission(tmp_path,purpose):
    path=checkpoint(tmp_path/'old.pt');cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True)
    with pytest.raises(ValueError,match='allow-encoder'):
        encoder.configure(cfg,checkpoint=path,requested_mode='deployment_measured',purpose=purpose)
    context=encoder.configure(cfg,checkpoint=path,requested_mode='deployment_measured',allow_change=True,purpose=purpose)
    assert context['checkpoint_training_mode']=='legacy_true'
    assert context['active_observation_mode']=='deployment_measured' and context['mode_changed_explicitly']
    encoder.check_load(SimpleNamespace(cfg=cfg),path)
    checkpoint(path,marker=1)
    with pytest.raises(ValueError,match='exact-checkpoint'):
        encoder.check_load(SimpleNamespace(cfg=cfg),path)


def test_direct_runner_load_cannot_silently_change_tagged_mode(tmp_path):
    path=checkpoint(tmp_path/'measured.pt','deployment_measured');cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True)
    with pytest.raises(ValueError,match='mode differs'):
        encoder.check_load(SimpleNamespace(cfg=cfg),path)
    with pytest.raises(ValueError,match='exact checkpoint'):
        encoder.configure(cfg,requested_mode='deployment_measured',allow_change=True,purpose='training')
    with pytest.raises(ValueError,match='exact checkpoint'):
        encoder.configure(cfg,checkpoint=path,allow_change=True,purpose='evaluation')
    with pytest.raises(ValueError,match='exact checkpoint'):
        encoder.configure(cfg,checkpoint=path,requested_mode='legacy_true',allow_change=True,purpose='export')


def test_metadata_refuses_malformed_or_stale_contract_and_tracking():
    with pytest.raises(ValueError):encoder.validate_contract({'contract':'wrong','mode':'deployment_measured'})
    cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True);encoder.apply_mode(cfg,'deployment_measured')
    with pytest.raises(ValueError,match='disagree'):
        encoder.checkpoint_infos(SimpleNamespace(cfg=cfg),{encoder.CHECKPOINT_KEY:encoder.contract('legacy_true')})
    with pytest.raises(ValueError,match='tracking already'):
        encoder.apply_mode(SimpleNamespace(observations={}), 'deployment_measured')


def test_common_measured_evaluation_compares_different_training_modes_but_not_sensor_modes():
    from mjlab.tasks.pupper_gait.bootstrap import compare_evaluations
    def record(training,active):
        return {'provenance':{'encoder_observation_context':{'checkpoint_training_mode':training,'active_observation_mode':active}},'segments':[]}
    with pytest.raises(ValueError,match='encoder_observation_mode'):
        compare_evaluations(record('legacy_true','legacy_true'),record('legacy_true','deployment_measured'))
    # Both evaluated with the same sensing semantics despite different training lineage.
    compare_evaluations(record('legacy_true','deployment_measured'),record('deployment_measured','deployment_measured'))


def test_cli_settings_records_training_and_evaluation_modes_separately(tmp_path):
    from mjlab.scripts.pupper_locomotion import settings
    path=checkpoint(tmp_path/'legacy.pt');args=SimpleNamespace(task='Mjlab-SpiderWalk-Flat-Pupper-v3',
        num_envs=2,seed=101,checkpoint=path,command='evaluate',encoder_observation_mode='deployment_measured',
        allow_encoder_observation_mode_change=True)
    cfg,_=settings(args,True)
    assert encoder.metadata(cfg)['mode']=='deployment_measured'
    assert args.encoder_observation_context['purpose']=='evaluation'
    assert args.encoder_observation_context['checkpoint_training_mode']=='legacy_true'


def test_real_observation_history_and_export_keep_measured_coordinates():
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.tasks.pupper.export import export_pupper_policy_from_env,to_robot_order
    cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True);cfg.scene.num_envs=2
    encoder.apply_mode(cfg,'deployment_measured')
    cfg.observations['actor'].terms['policy'].params.update(add_noise=False,imu_latency_distribution=None)
    if cfg.observations['actor'].terms['policy'].func.__name__=='pupper_gait_shared_obs':
        cfg.observations['critic'].terms['policy'].params.update(add_noise=False,imu_latency_distribution=None)
    env=ManagerBasedRlEnv(cfg,device='cpu')
    try:
        robot=env.scene['robot'];robot.data.encoder_bias[:]=.013
        env.observation_manager.reset();first=env.observation_manager.compute_group('actor',update_history=True)
        d=48;h=first.shape[-1]//d;expected=robot.data.joint_pos_biased-robot.data.default_joint_pos
        torch.testing.assert_close(first.reshape(2,h,d)[:,-1,12:24],expected)
        robot_order=to_robot_order(first,h,d).reshape(2,h,d)
        np.testing.assert_allclose(robot_order[:,0,12:24],expected.numpy(),atol=1e-7)
        actor=SimpleNamespace(mlp=nn.Sequential(nn.Linear(h*d,12)),obs_normalizer=nn.Identity())
        from mjlab.tasks.pupper_gait.export import GAIT_OBS_COMPONENTS
        policy=export_pupper_policy_from_env(actor,env,single_obs_dim=d,obs_components=GAIT_OBS_COMPONENTS)
        assert policy['encoder_observation_contract']==encoder.contract('deployment_measured')
        assert policy['observation_history']==h and policy['single_observation_size']==d
        np.testing.assert_allclose(policy['default_joint_pos'],robot.data.default_joint_pos[0].numpy())
    finally:env.close()


@pytest.mark.parametrize('module_name,function,reference',[('gait','pupper_gait_actor_obs','reference_offset'),
    ('mixed_gaits','mixed_gait_actor_obs','mixed_reference_offset'),
    ('mixed_gaits','zero_reference_actor_obs',None),('jump','jump_actor_obs','jump_reference_offset'),
    ('mixed_jump','mixed_jump_actor_obs','mixed_jump_reference_offset')])
def test_each_reference_wrapper_passes_mode_without_biasing_the_reference(monkeypatch,module_name,function,reference):
    import importlib,inspect
    module=importlib.import_module('mjlab.tasks.pupper_gait.mdp.'+module_name)
    if reference:
        monkeypatch.setattr(module,reference,lambda *a,**k:torch.full((2,12),.123))
    func=getattr(module,function);kwargs={name:('twist' if name=='command_name' else 1.)
        for name,p in inspect.signature(func).parameters.items()
        if name!='env' and p.default is inspect.Parameter.empty}
    env=fake_env();kwargs['add_noise']=False
    legacy=func(env,**kwargs);measured=func(env,**kwargs,encoder_observation_mode='deployment_measured')
    assert legacy.shape==measured.shape==(2,48)
    expected=torch.zeros_like(legacy);expected[:,12:24]=env.scene['robot'].data.encoder_bias
    torch.testing.assert_close(measured-legacy,expected,atol=2e-8,rtol=1e-5)
    assert torch.equal(legacy[:,36:],measured[:,36:])


def test_privileged_critic_remains_true_state():
    from mjlab.tasks.pupper.pupper_env_cfg import pupper_flat_env_cfg
    cfg=pupper_flat_env_cfg(play=True);critic=copy.deepcopy(cfg.observations['critic'])
    encoder.apply_mode(cfg,'deployment_measured')
    assert cfg.observations['critic']==critic


def test_runner_load_and_save_enforce_and_stamp_mode(tmp_path,monkeypatch):
    from mjlab.tasks.pupper.rl.runner import PupperOnPolicyRunner
    from mjlab.tasks.velocity.rl.runner import VelocityOnPolicyRunner
    cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True)
    runner=object.__new__(PupperOnPolicyRunner)
    runner.env=SimpleNamespace(unwrapped=SimpleNamespace(cfg=cfg,
        action_manager=SimpleNamespace(get_term=lambda _:None)))
    measured=checkpoint(tmp_path/'measured.pt','deployment_measured')
    with pytest.raises(ValueError,match='mode differs'):runner.load(str(measured))
    encoder.apply_mode(cfg,'deployment_measured')
    saved={}
    monkeypatch.setattr(VelocityOnPolicyRunner,'save',lambda self,path,infos=None:saved.update(infos=infos,path=path))
    monkeypatch.setattr(runner,'_try_export',lambda:None)
    runner.save(str(tmp_path/'saved.pt'))
    assert saved['infos'][encoder.CHECKPOINT_KEY]==encoder.contract('deployment_measured')


def test_portable_contract_refuses_integer_in_place_of_boolean():
    malformed=encoder.contract('deployment_measured');malformed['fixed_reference_and_default_unchanged']=1
    with pytest.raises(ValueError):encoder.validate_contract(malformed)


def test_verified_explicit_transition_load_then_save_relabels_inherited_infos(tmp_path,monkeypatch):
    from mjlab.tasks.pupper.rl.runner import PupperOnPolicyRunner
    from mjlab.tasks.velocity.rl.runner import VelocityOnPolicyRunner
    path=checkpoint(tmp_path/'legacy.pt','legacy_true');cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True)
    context=encoder.configure(cfg,checkpoint=path,requested_mode='deployment_measured',allow_change=True,purpose='training')
    runner=object.__new__(PupperOnPolicyRunner)
    runner.env=SimpleNamespace(unwrapped=SimpleNamespace(cfg=cfg,action_manager=SimpleNamespace(get_term=lambda _:None)))
    runner.logger=SimpleNamespace(log_dir=tmp_path)
    inherited={encoder.CHECKPOINT_KEY:encoder.contract('legacy_true'),'unrelated':'keep'}
    monkeypatch.setattr(VelocityOnPolicyRunner,'load',lambda *a,**k:copy.deepcopy(inherited))
    infos=runner.load(str(path));assert context['checkpoint_load_verified']
    assert (tmp_path/'encoder-checkpoint-load.json').is_file()
    saved={};monkeypatch.setattr(VelocityOnPolicyRunner,'save',lambda self,path,infos=None:saved.update(infos=infos))
    monkeypatch.setattr(runner,'_try_export',lambda:None);runner.save(str(tmp_path/'continued.pt'),infos)
    assert saved['infos'][encoder.CHECKPOINT_KEY]==encoder.contract('deployment_measured')
    assert saved['infos'][encoder.LOAD_KEY]['checkpoint_training_mode']=='legacy_true'
    assert saved['infos']['unrelated']=='keep' and inherited[encoder.CHECKPOINT_KEY]['mode']=='legacy_true'


def test_remote_foundation_resume_must_name_mode_before_building_environment(tmp_path,monkeypatch):
    from mjlab.scripts import pupper_foundation
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES','0');monkeypatch.setattr(torch.cuda,'is_available',lambda:True)
    args=SimpleNamespace(profile='spider_walk',motion=None,output=tmp_path,resume_run='team/project/run')
    with pytest.raises(ValueError,match='requires explicit --encoder-observation-mode'):
        pupper_foundation.train(args)
    args.encoder_observation_mode='deployment_measured';args.allow_encoder_observation_mode_change=True
    with pytest.raises(ValueError,match='exact local --initial-checkpoint'):
        pupper_foundation.train(args)


def test_checkpoint_atomic_replacement_cannot_mix_metadata_and_identity(tmp_path,monkeypatch):
    path=checkpoint(tmp_path/'changing.pt','legacy_true');real_load=torch.load
    def replace_then_load(buffer,*a,**kw):
        checkpoint(path,'deployment_measured')
        return real_load(buffer,*a,**kw)
    monkeypatch.setattr(torch,'load',replace_then_load)
    with pytest.raises(RuntimeError,match='changed while reading'):
        encoder.checkpoint_contract(path)


def test_actual_load_replacement_is_detected_before_recording_success(tmp_path):
    path=checkpoint(tmp_path/'changing.pt','legacy_true');cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True)
    env=SimpleNamespace(cfg=cfg);receipt=encoder.check_load(env,path)
    checkpoint(path,'legacy_true',marker=1)
    with pytest.raises(RuntimeError,match='changed during actual runner load'):
        encoder.record_successful_load(env,receipt,path=path)


def test_explicit_null_checkpoint_tag_is_invalid_not_legacy(tmp_path):
    path=tmp_path / "null.pt"
    torch.save({"infos": {encoder.CHECKPOINT_KEY: None}}, path)
    with pytest.raises(ValueError, match="Invalid Pupper encoder observation contract"):
        encoder.checkpoint_contract(path)
