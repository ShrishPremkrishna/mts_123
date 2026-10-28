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


@pytest.fixture
def slew_host(host, tmp_path):
    binary, source = host
    policy = json.loads(source.read_text())
    names = [f'leg_{leg}_{j}' for leg in ('front_r', 'front_l', 'back_r', 'back_l') for j in (1,2,3)]
    policy.update(required_controller_features=['position_target_slew_v1'], action_types=['position']*12,
                  position_target_slew=dict(contract='pupper_position_target_slew_v1', max_rate_rad_s=.4,
                      max_step_s=.02, joint_names=names, lower_limits=[-1.2]*12, upper_limits=[1.2]*12,
                      driver_limits_sha256='a'*64))
    path=tmp_path/'slew-policy.json'; path.write_text(json.dumps(policy))
    return binary, path


def test_slew_covers_initialization_and_held_inference_without_changing_history(host, slew_host):
    results=[]
    for selected in (host,slew_host):
        proc,rpc=session(selected,'entry')
        try:
            rows=[]
            for i in range(1041):
                request=step(i/520);request['audit_inference']=True;rows.append(rpc(request))
            results.append(rows)
        finally:
            proc.stdin.close();assert proc.wait(timeout=5)==0
    plain,filtered=results
    targets=np.asarray([r['target'] for r in filtered])
    np.testing.assert_array_equal(targets[0],1.)
    assert np.max(np.abs(np.diff(targets,axis=0))) <= .4/520+1e-12
    assert np.all(np.abs(targets)<=1.2)
    # Before/after fade history still contains the effective unclipped actor
    # request; no filtered target gets fed back in place of last_action.
    for legacy,row in zip(plain,filtered):
        assert row['requested_target']==legacy['target']
        if 'raw_action' in row:assert row['raw_action']==legacy['raw_action']
        if 'model_input' in row:assert row['model_input']==legacy['model_input']
    assert filtered[260]['stage']=='fade' and filtered[260]['target'][0] > .79
    held=[(a,b) for a,b in zip(filtered[261:-1],filtered[262:])
          if not b['inference'] and a['requested_target']==b['requested_target']]
    assert any(a['target']!=b['target'] for a,b in held)
    assert any(abs(r['requested_target'][0]-r['target'][0])>.5 for r in filtered)


def test_slew_activation_reset_zero_time_and_scheduler_gap(slew_host):
    proc,rpc=session(slew_host,'entry')
    try:
        start=rpc(step(0));np.testing.assert_array_equal(start['target'],1.)
        same=rpc(step(0));assert same['target']==start['target']
        delayed=rpc(step(.4))
        # A missing scheduler interval cannot accrue an unsafe catch-up jump.
        np.testing.assert_allclose(delayed['target'],1-.4*.02,atol=1e-12)
        rpc(dict(op='activate',controller='trick',joint_pos=[-2.]*12,init_s=.5,fade_s=1.,start_clip='entry',clip_anchor=True))
        reset=rpc(step(0));np.testing.assert_array_equal(reset['target'],-1.2)
    finally:
        proc.stdin.close();assert proc.wait(timeout=5)==0


def test_slew_backward_clock_refuses_without_another_command(slew_host):
    proc,rpc=session(slew_host,'entry')
    rpc(step(.1))
    proc.stdin.write(json.dumps(step(.09))+'\n');proc.stdin.flush()
    assert proc.stdout.readline()==''
    proc.stdin.close();assert proc.wait(timeout=5)==1
    assert 'invalid slew command clock' in proc.stderr.read()


def test_default_return_extends_identical_spider_sequence():
    policy={'trick':{'fps':50,'clips':[{'name':'spider_down','start':0,'end':101},
                                     {'name':'spider_up','start':101,'end':202}]}}
    original=handoff.stage_plan(policy)
    extended=handoff.stage_plan(policy,return_default=True)
    assert extended[:len(original)]==original
    assert extended[len(original):]==[('default_activation','default',4.2,[0,0,0],''),
                                     ('default_forward','default',2,[.1,0,0],''),
                                     ('default_stop','default',2,[0,0,0],'')]


def test_actual_default_return_matches_independent_native_host_and_numpy(host,tmp_path):
    from mjlab.tasks.pupper.export import json_forward
    from mjlab.tasks.pupper_gait.export import reference_offset_numpy
    independent_spec=importlib.util.spec_from_file_location('independent_default',ROOT/'foundation/scripts/check_export_locomotion.py')
    independent=importlib.util.module_from_spec(independent_spec);independent_spec.loader.exec_module(independent)
    other_dir=tmp_path/'other';other_dir.mkdir()
    other_binary,_=independent.build_controller(other_dir)
    default=ROOT/'pupper_gait_deploy/policies/default_policy.json'
    metadata=json.loads(default.read_text())
    other=independent.Host(other_binary,default,1,other_dir/'controller.log')
    binary,fixture=host
    proc=subprocess.Popen([str(binary),str(fixture),str(fixture),'--return-default-no-jump',str(default)],
                          stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    def rpc(message):
        proc.stdin.write(json.dumps(message)+'\n');proc.stdin.flush()
        line=proc.stdout.readline();assert line,proc.stderr.read()
        return json.loads(line)
    measured=(np.array(metadata['default_joint_pos'])+.1).tolist()
    try:
        # First visit another controller with incompatible history, then verify
        # the default activation starts from measured joints and its own history.
        rpc(dict(op='activate',controller='trick',joint_pos=measured,init_s=.5,fade_s=1.,start_clip='exit',clip_anchor=True))
        rpc(step(.7))
        rpc(dict(op='activate',controller='default',joint_pos=measured,init_s=2.,fade_s=2.,start_clip='',clip_anchor=True))
        np.testing.assert_allclose(rpc(step(0))['target'],measured,atol=1e-7)
        expected_mid=(np.array(measured)+np.array(metadata['default_joint_pos']))/2
        np.testing.assert_allclose(rpc(step(1))['target'],expected_mid,atol=1e-7)
        for tick in range(3101):
            t=2+tick/520
            index=tick//520
            command=([0,0,0],[.1,0,0],[2.,2.,.2],[-.1,0,-.2],[0,0,0],[0,0,0])[min(index,5)]
            request=dict(op='step',elapsed_s=t,joint_pos=measured,joint_vel=[0.]*12,
                         angular_velocity=[.01,-.02,.03],quaternion=[1,0,0,0],command=command,audit_inference=True)
            result=rpc(request)
            if not result['inference']:continue
            expected=other.request(dict(op='infer',states=[{**request,'debug':True}]))[0]
            for ours,theirs in [('target','target'),('raw_action','raw_action'),('model_input','observation'),('gait_reference_offset','reference')]:
                np.testing.assert_allclose(result[ours],expected[theirs],atol=3e-5,rtol=3e-5)
            raw=json_forward(metadata,np.array(result['model_input'])[None,:])[0]
            np.testing.assert_allclose(result['raw_action'],raw,atol=3e-5,rtol=3e-5)
            effective=result['effective_command']
            ref=reference_offset_numpy(metadata['gait_reference'],np.array(metadata['default_joint_pos']),
                                       np.array([t-2]),*[np.array([v]) for v in effective])[0]
            np.testing.assert_allclose(result['gait_reference_offset'],ref,atol=2e-6,rtol=2e-6)
            assert max(abs(v) for v in effective[:2])<=1.5
    finally:
        proc.stdin.close();assert proc.wait(timeout=5)==0
        other.close()


@pytest.mark.parametrize('label',['default','typo'])
def test_unavailable_or_unknown_controller_is_not_silently_walk(host,label):
    binary,path=host
    proc=subprocess.run([str(binary),str(path),str(path)],input=json.dumps(dict(op='activate',controller=label))+'\n',
                        capture_output=True,text=True)
    assert proc.returncode==1 and 'unknown or unavailable controller' in proc.stderr


@pytest.mark.parametrize('field,value',[('action_types',['effort']*12),('action_type','velocity')])
def test_handoff_rejects_unsupported_action_contract(host,tmp_path,field,value):
    binary,path=host
    metadata=json.loads(path.read_text());metadata[field]=value
    bad=tmp_path/'bad.json';bad.write_text(json.dumps(metadata))
    proc=subprocess.run([str(binary),str(bad),str(path)],capture_output=True,text=True)
    assert proc.returncode==1 and 'position actions only' in proc.stderr


def test_finite_default_return_extends_identical_terminal_hold():
    policy={'trick':{'fps':50},'motion':{'joint_pos':[[0.]*12]*298}}
    original=handoff.stage_plan(policy,single_motion=True)
    extended=handoff.stage_plan(policy,single_motion=True,return_default=True)
    assert extended[:2]==original
    assert extended[2:]==handoff.default_return_stages()
    policy['trick']['clips']=[{'name':'loop','start':0,'end':298,'loop':True}]
    with pytest.raises(ValueError,match='finite clip'):
        handoff.stage_plan(policy,single_motion=True,return_default=True)


def test_finite_return_requires_matching_standing_descriptor(tmp_path):
    policy=tmp_path/'policy.json';descriptor=tmp_path/'trick.yaml'
    descriptor.write_text('type: policy\npolicy: policy.json\nstart_pose: standing\nend_pose: standing\n')
    assert handoff.validate_single_return_descriptor(descriptor,policy)==descriptor.read_bytes()
    with pytest.raises(ValueError,match='requires --trick-descriptor'):
        handoff.validate_single_return_descriptor(None,policy)
    with pytest.raises(ValueError,match='does not match'):
        handoff.validate_single_return_descriptor(descriptor,tmp_path/'other.json')
    for ending in ('lying','sitting'):
        descriptor.write_text('type: policy\npolicy: policy.json\nstart_pose: standing\nend_pose: '+ending+'\n')
        with pytest.raises(ValueError,match='standing-to-standing'):
            handoff.validate_single_return_descriptor(descriptor,policy)
