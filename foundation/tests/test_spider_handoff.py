"""Exercise the actual compiled integration controller, including history feedback."""
import importlib.util
import json
from pathlib import Path
import subprocess
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('handoff',ROOT/'foundation/scripts/check_spider_handoff.py')
handoff=importlib.util.module_from_spec(spec);spec.loader.exec_module(handoff)


@pytest.fixture(scope='module')
def host(tmp_path_factory):
    path=tmp_path_factory.mktemp('handoff-host');binary,_=handoff.build_controller(path)
    # A small deterministic exported policy, with last_action feeding back into
    # raw output, exposes history/reset/clipping bugs independently of physics.
    components=[('motion_command',24),('motion_anchor_ori_b',6),('base_ang_vel',3),
                ('joint_pos_rel',12),('joint_vel_rel',12),('last_action',12)]
    weights=np.zeros((69,12));weights[57:69]=np.eye(12)*.1
    policy=dict(default_joint_pos=[0.]*12,joint_lower_limits=[-1.5]*12,joint_upper_limits=[1.5]*12,
                action_scale=.75,kp=5.5,kd=.25,observation_history=1,in_shape=[None,69],
                obs_spec={'components':[dict(name=n,size=s) for n,s in components]},
                layers=[dict(type='dense',activation='',shape=[None,12],weights=[weights.tolist(),[10.]*12])],
                trick=dict(fps=50,clips=[dict(name='entry',start=0,end=2),dict(name='exit',start=2,end=4)]),
                motion=dict(joint_pos=[[0.]*12,[0.]*12,[1.]*12,[0.]*12],joint_vel=[[0.]*12]*4,
                            anchor_quat_wxyz=[[1,0,0,0]]*4))
    p=path/'policy.json';p.write_text(json.dumps(policy))
    return binary,p


def session(host,clip,anchor=True):
    binary,path=host
    proc=subprocess.Popen([str(binary),str(path),str(path)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    def rpc(message):
        proc.stdin.write(json.dumps(message)+'\n');proc.stdin.flush()
        line=proc.stdout.readline()
        assert line,proc.stderr.read()
        return json.loads(line)
    rpc(dict(op='activate',controller='trick',joint_pos=[1.]*12,init_s=.5,fade_s=1.,start_clip=clip,clip_anchor=anchor))
    return proc,rpc


def step(t):
    return dict(op='step',elapsed_s=t,joint_pos=[1.]*12,joint_vel=[0.]*12,
                quaternion=[1,0,0,0],angular_velocity=[0,0,0],command=[0,0,0])


def test_inference_audit_captures_pre_rotation_input_without_changing_target(host):
    outputs=[]
    for audit in (False,True):
        proc,rpc=session(host,'exit')
        try:
            rows=[]
            for i in range(11):
                request=step(.5+i/520);request['audit_inference']=audit;rows.append(rpc(request))
            outputs.append(rows)
        finally:
            proc.stdin.close();assert proc.wait(timeout=5)==0
    for plain,audited in zip(*outputs):
        assert plain['target']==audited['target'] and plain['raw_action']==audited['raw_action']
    first,last=outputs[1][0],outputs[1][-1]
    assert len(first['model_input'])==69
    np.testing.assert_array_equal(first['model_input'][57:],0)
    np.testing.assert_allclose(last['model_input'][57:],1/.75,atol=1e-7)
    assert first['effective_command']==[0,0,0] and first['gait_reference_offset']==[]


def test_selected_exit_preserves_low_pose_then_fades_with_unclipped_history(host):
    proc,rpc=session(host,'exit')
    try:
        np.testing.assert_allclose(rpc(step(.25))['target'],1)
        first=rpc(step(.5));np.testing.assert_allclose(first['target'],1,atol=2e-7)
        assert first['frame']==2 and first['raw_action']==[10.]*12
        for i in range(1,11): second=rpc(step(.5+i/520))
        # Full pre-clamp effective action at first inference is1/.75, not0.
        np.testing.assert_allclose(second['raw_action'],10+.1/.75,atol=2e-6)
        for i in range(11,531): final=rpc(step(.5+i/520))
        np.testing.assert_allclose(final['target'],1.5)
        assert min(final['raw_action'])>10.2 # history includes unclipped requested action
    finally:
        proc.stdin.close();assert proc.wait(timeout=5)==0


@pytest.mark.parametrize('anchor',[False,True])
def test_first_clip_default_activation_unchanged(host,anchor):
    proc,rpc=session(host,'entry',anchor)
    try:
        np.testing.assert_allclose(rpc(step(.25))['target'],.5)
        result=rpc(step(.5));np.testing.assert_allclose(result['target'],0)
        assert result['frame']==0
    finally:
        proc.stdin.close();assert proc.wait(timeout=5)==0


def test_compiled_contact_worlds_are_fixed_and_distinct():
    for kind,hulls in [('full_hulls',9),('torso_only',1)]:
        model,_=handoff.make_model(kind)
        active_hulls=[g for g in range(model.ngeom) if model.geom(g).name.endswith('_hull') and model.geom_conaffinity[g]]
        balls=[g for g in range(model.ngeom) if model.geom(g).name.endswith('_collision')]
        assert len(active_hulls)==hulls and len(balls)==8
        assert all((model.geom_contype[g],model.geom_conaffinity[g])==(0,1) for g in active_hulls)
        assert all((model.geom_contype[g],model.geom_conaffinity[g])==(1,1) for g in balls)
        assert model.opt.timestep==1/520


def test_single_finite_motion_holds_without_restarting_reference():
    policy={'trick':{'fps':50},'motion':{'joint_pos':[[0.]*12]*298}}
    stages=handoff.stage_plan(policy,True)
    assert stages[0]==('motion','trick',.5+1+297/50,[0,0,0],'all')
    assert stages[1]==('terminal_hold','trick',2,[0,0,0],'')
    policy['trick']['clips']=[{'name':'loop','start':0,'end':298,'loop':True}]
    with pytest.raises(ValueError,match='finite clip'):
        handoff.stage_plan(policy,True)


@pytest.mark.parametrize('angle',[0,-1,90,float('nan'),float('inf')])
def test_body_angle_override_cannot_loosen_existing_stop(angle):
    from types import SimpleNamespace
    with pytest.raises(ValueError,match='Body-angle'):
        handoff.run(SimpleNamespace(max_body_angle_deg=angle))


def test_internal_reset_warning_refuses_finite_recovered_state():
    model,_=handoff.make_model('full_hulls');data=handoff.mujoco.MjData(model)
    assert handoff.numerical_failures(data)==[]
    data.warning[handoff.mujoco.mjtWarning.mjWARN_BADQACC].number=1
    assert handoff.numerical_failures(data)
    data.warning[handoff.mujoco.mjtWarning.mjWARN_BADQACC].number=0
    data.qvel[0]=float('nan')
    assert handoff.numerical_failures(data)==['nonfinite state']


def test_inference_record_distinguishes_scenario_and_activation_stages():
    state={'stage':'policy','raw_action':[1.]}
    event=handoff.inference_event(state,stage='walk_forward',time_s=3)
    assert event=={'stage':'walk_forward','activation_stage':'policy','time_s':3,'raw_action':[1.]}
    assert state['stage']=='policy'
