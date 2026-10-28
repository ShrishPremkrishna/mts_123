"""Recorded startup transfer preserves literal mechanics and explicit sensing semantics."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import numpy as np
import pytest
import mujoco

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('startup_handoff',ROOT/'foundation/scripts/check_spider_handoff.py')
h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES


def transfer(model=None):
    if model is None:model,_=h.make_model('full_hulls')
    return dict(schema='pupper_native_startup_transfer_v1',name='identity',
        source=dict(revision='a'*40,snapshot_sha256='b'*64,capture_sha256='c'*64,seed=101,world_index=11,original_fields_bit_identical=True),
        bodies={model.body(i).name:dict(mass_kg=float(model.body_mass[i]),principal_inertia_kg_m2=model.body_inertia[i].tolist(),com_body_m=model.body_ipos[i].tolist(),inertial_quaternion_wxyz=model.body_iquat[i].tolist()) for i in range(1,model.nbody)},
        joints={n:dict(kp_scale=1.,kd_scale=1.,encoder_bias_rad=0.) for n in JOINT_NAMES},
        foot_friction=.8,foot_material='sphere_only',observation_semantics='deployment_measured')


@pytest.mark.parametrize('defect',['unknown','schema','missing_body','wrong_body','missing_joint','source','seed','unverified','semantics','bias','gain','damping','mass','inertia','quat','body_com','density','friction','material'])
def test_malformed_or_out_of_contract_transfer_refused(defect):
    m,_=h.make_model('full_hulls');v=transfer(m);b=v['bodies']['base_link'];j=v['joints'][JOINT_NAMES[0]]
    if defect=='unknown':v['other']=0
    elif defect=='schema':v['schema']=1
    elif defect=='missing_body':v['bodies'].pop('base_link')
    elif defect=='wrong_body':v['bodies']['bad']=v['bodies'].pop('base_link')
    elif defect=='missing_joint':v['joints'].pop(JOINT_NAMES[0])
    elif defect=='source':v['source']['capture_sha256']='oops'
    elif defect=='seed':v['source']['seed']=True
    elif defect=='unverified':v['source']['original_fields_bit_identical']=False
    elif defect=='semantics':v['observation_semantics']='unspecified'
    elif defect=='bias':j['encoder_bias_rad']=.016
    elif defect=='gain':j['kp_scale']=.59
    elif defect=='damping':j['kd_scale']=float('nan')
    elif defect=='mass':b['mass_kg']=True
    elif defect=='inertia':b['principal_inertia_kg_m2'][0]*=1.05
    elif defect=='quat':b['inertial_quaternion_wxyz']=[2,0,0,0]
    elif defect=='body_com':b['com_body_m'][0]+=.02001
    elif defect=='density':b['mass_kg']*=1.31
    elif defect=='friction':v['foot_friction']=1.41
    elif defect=='material':v['foot_material']='unknown'
    old=[x.copy() for x in (m.body_mass,m.body_inertia,m.body_ipos,m.body_iquat,m.geom_friction)]
    with pytest.raises(ValueError):h.apply_startup_transfer(m,v)
    for actual,expected in zip((m.body_mass,m.body_inertia,m.body_ipos,m.body_iquat,m.geom_friction),old):np.testing.assert_array_equal(actual,expected)


def test_named_tensor_transfer_composes_per_body_density_and_literal_principal_axes():
    m,_=h.make_model('full_hulls');v=transfer(m)
    # Equivalent inertial tensor represented by cyclic principal-axis permutation.
    for i,(name,b) in enumerate(reversed(list(v['bodies'].items()))):
        scale=.91+i*.025;b['mass_kg']*=scale
        inertia=np.asarray(b['principal_inertia_kg_m2'])*scale
        q=np.asarray(b['inertial_quaternion_wxyz']);rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,q)
        perm=np.array([[0,0,1],[1,0,0],[0,1,0]])
        mat=rotation.reshape(3,3)@perm;newq=np.empty(4);mujoco.mju_mat2Quat(newq,mat.ravel())
        b['principal_inertia_kg_m2']=inertia[[1,2,0]].tolist();b['inertial_quaternion_wxyz']=newq.tolist()
    v['bodies']['base_link']['com_body_m'][0]+=.017
    result=h.apply_startup_transfer(m,v)
    assert result['realized_bodies']==v['bodies']
    assert m.body_subtreemass[m.body('base_link').id]==pytest.approx(m.body_mass.sum())


def test_identity_transfer_has_bit_identical_integrated_state():
    plain,_=h.make_model('full_hulls');altered,_=h.make_model('full_hulls');h.apply_startup_transfer(altered,transfer(altered))
    a,b=mujoco.MjData(plain),mujoco.MjData(altered)
    for d in (a,b):d.qpos[:7]=[0,0,.18,1,0,0,0];d.qvel[:6]=[.01,-.02,.01,.01,.02,-.01]
    for _ in range(200):
        mujoco.mj_step(plain,a);mujoco.mj_step(altered,b)
        np.testing.assert_array_equal(a.qpos,b.qpos);np.testing.assert_array_equal(a.qvel,b.qvel);np.testing.assert_array_equal(a.qacc_warmstart,b.qacc_warmstart)


def test_named_gains_and_bias_are_applied_at_every_initialization_fade_and_held_step():
    v=transfer()
    for i,n in enumerate(JOINT_NAMES):v['joints'][n]=dict(kp_scale=.61+i*.03,kd_scale=.81+i*.04,encoder_bias_rad=-.014+i*.002)
    v['joints']=dict(reversed(list(v['joints'].items())))
    bias,kps,kds=h.startup_joint_vectors(v,JOINT_NAMES)
    q=np.linspace(-.2,.2,12)
    for controller in ('trick','walk','default'):
        np.testing.assert_array_equal(h.startup_actor_joint_position(q,bias,controller),q+bias)
        expected=q+bias if controller=='trick' else q
        np.testing.assert_array_equal(h.startup_actor_joint_position(q,bias,controller,'training_by_controller'),expected)
        for kp,kd in [(7.5,.25),(5.5,.25),(5.5,.25),(7.5,.25)]:
            measured=np.clip(q+bias,-.15,.15)
            target,actual_kp,actual_kd=h.startup_physical_command(measured,kp,kd,bias,kps,kds)
            np.testing.assert_array_equal(target,measured-bias)
            np.testing.assert_array_equal(actual_kp,kp*kps);np.testing.assert_array_equal(actual_kd,kd*kds)


@pytest.fixture(scope='module')
def env():
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.tasks.registry import load_env_cfg
    from mjlab.tasks.pupper.deployment_contract import apply_deployment_target_clamp,apply_driver_target_clamp
    from mjlab.tasks.pupper.target_slew import apply_position_target_slew
    cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True);cfg.scene.num_envs=2
    apply_deployment_target_clamp(cfg);contract=apply_driver_target_clamp(cfg,ROOT/'foundation/configs/class-driver-joint-limits.json');apply_position_target_slew(cfg,15.,contract)
    e=ManagerBasedRlEnv(cfg,device='cpu');e.reset()
    try:yield e
    finally:e.close()


def test_actual_training_observation_and_action_functions_confirm_sensing_mismatch(env):
    import torch
    from mjlab.tasks.pupper.mdp.observations import pupper_actor_obs
    from mjlab.envs.mdp.observations import joint_pos_rel
    robot=env.scene['robot'];robot.data.encoder_bias[:]=torch.linspace(-.014,.014,12)
    true_q=robot.data.joint_pos;bias=robot.data.encoder_bias;default=robot.data.default_joint_pos
    gait=pupper_actor_obs(env,add_noise=False,imu_latency_distribution=None)
    torch.testing.assert_close(gait[:,12:24],true_q-default)
    tracking=joint_pos_rel(env,biased=True);torch.testing.assert_close(tracking,true_q+bias-default)
    # The native deployment path uses the actual measured encoder positions;
    # optional training emulation is explicitly a different input for gait actors.
    for controller in ('trick','walk','default'):
        q=h.startup_actor_joint_position(true_q[0].numpy(),bias[0].numpy(),controller)
        np.testing.assert_array_equal(q,(true_q[0]+bias[0]).numpy())
    term=env.action_manager.get_term('joint_pos');raw=torch.full((2,12),100.)
    env.action_manager.process_action(raw);prev=torch.clamp(robot.data.joint_pos_biased[:,term.target_ids],term._clip[:,:,0],term._clip[:,:,1])
    for _ in range(5):
        term.apply_actions();applied=term.applied_actions
        assert torch.max(torch.abs(applied-prev))<=15*env.physics_dt+1e-6
        physical,_,_=h.startup_physical_command(applied.numpy(),5.5,.25,bias.numpy(),np.ones(12),np.ones(12))
        np.testing.assert_array_equal(physical,robot.data.joint_pos_target[:,term.target_ids].numpy())
        prev=applied.clone()
    torch.testing.assert_close(env.action_manager.action,raw)


def test_compiled_native_input_and_initialization_receive_measured_joint_coordinates(tmp_path):
    binary,_=h.build_controller(tmp_path)
    components=[('base_ang_vel',3),('projected_gravity',3),('command',3),('desired_world_z',3),('joint_pos_rel',12),('last_action',12)]
    policy=dict(default_joint_pos=[.1]*12,joint_lower_limits=[-1.5]*12,joint_upper_limits=[1.5]*12,action_scale=.75,kp=5.5,kd=.25,observation_history=1,in_shape=[None,36],obs_spec={'components':[dict(name=n,size=s) for n,s in components]},layers=[dict(type='dense',activation='',shape=[None,12],weights=[np.zeros((36,12)).tolist(),[0.]*12])])
    p=tmp_path/'p.json';p.write_text(json.dumps(policy));proc=subprocess.Popen([str(binary),str(p),str(p)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    def rpc(v):
        proc.stdin.write(json.dumps(v)+'\n');proc.stdin.flush();return json.loads(proc.stdout.readline())
    true_q=np.linspace(-.1,.1,12);bias=np.linspace(-.014,.014,12);measured=true_q+bias
    try:
        rpc(dict(op='activate',controller='walk',joint_pos=measured.tolist(),init_s=2.,fade_s=2.,clip_anchor=False))
        request=dict(op='step',joint_pos=measured.tolist(),joint_vel=[0.]*12,quaternion=[1,0,0,0],angular_velocity=[0,0,0],command=[0,0,0],audit_inference=True)
        initial=rpc(dict(request,elapsed_s=0.));np.testing.assert_allclose(initial['target'],measured,atol=1e-8)
        infer=rpc(dict(request,elapsed_s=2.));np.testing.assert_allclose(infer['model_input'][12:24],measured-.1,atol=1e-8)
    finally:proc.stdin.close();assert proc.wait(timeout=5)==0


@pytest.mark.parametrize('duration',[True,float('nan'),float('inf'),1.99,30.01,-1])
def test_invalid_default_stop_diagnostic_refused(duration):
    with pytest.raises(ValueError):h.default_diagnostic_stages(h.default_return_stages(),stop_duration_s=duration)


def test_default_only_and_long_tail_preserve_exact_original_prefix():
    original=[('entry','trick',3.,[0,0,0],'entry')]+h.default_return_stages()
    assert h.default_diagnostic_stages(original)==original
    longer=h.default_diagnostic_stages(original,stop_duration_s=8.)
    assert longer[:-1]==original[:-1] and longer[-1]==('default_stop','default',8.,[0,0,0],'')
    only=h.default_diagnostic_stages(original,default_only=True,stop_duration_s=8.)
    assert only==longer[1:]
    with pytest.raises(ValueError):h.default_diagnostic_stages(original[:1],stop_duration_s=8.)
