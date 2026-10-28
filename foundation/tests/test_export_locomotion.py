"""Native exported-policy contract checks before interpreting physics comparisons."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('export_screen',ROOT/'foundation/scripts/check_export_locomotion.py')
screen=importlib.util.module_from_spec(spec);sys.modules[spec.name]=screen;spec.loader.exec_module(screen)
DEFAULT=ROOT/'pupper_gait_deploy/policies/default_policy.json'
STABLE=ROOT/'artifacts/recovered-policies/4hkyvuxh/policy.json'
MIXED=ROOT/'artifacts/optional2/clamped-batch/mixed-baseline/policy.json'
FAST=ROOT/'artifacts/optional2/clamped-batch/mixed-fast-rewards/policy.json'
LIMITS=ROOT/'foundation/configs/class-driver-joint-limits.json'


@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    return screen.build_controller(tmp_path_factory.mktemp('export-host'))[0]


def state(policy,t,command=(0,0,0),yaw=0,debug=True):
    return dict(elapsed_s=t,command=list(command),joint_pos=policy['default_joint_pos'],
                angular_velocity=[.01,-.02,.03],quaternion=[np.cos(yaw/2),0,0,np.sin(yaw/2)],debug=debug)


@pytest.mark.parametrize('path',[DEFAULT,STABLE,MIXED,FAST],ids=['shipped','stable','mixed','fast'])
def test_real_native_network_and_reference_match_export_math(binary,tmp_path,path):
    if not path.exists():pytest.skip('local trained artifact unavailable')
    from mjlab.tasks.pupper.export import json_forward
    from mjlab.tasks.pupper_gait.export import reference_offset_numpy
    policy=json.loads(path.read_text());host=screen.Host(binary,path,1,tmp_path/'controller.log')
    try:
        for index,command in enumerate(([0,0,0],[.49,0,.2],[.51,0,.2],[-.7,0,-.2],[0,.3,.2],[0,0,-1])):
            t=4+index/52
            result=host.request(dict(op='infer',states=[state(policy,t,command)]))[0]
            observation=np.array(result['observation'])
            expected=json_forward(policy,observation[None,:])[0]
            np.testing.assert_allclose(result['raw_action'],expected,atol=3e-5,rtol=3e-5)
            filtered=np.asarray(result['effective_command'])
            reference=reference_offset_numpy(policy['gait_reference'],np.asarray(policy['default_joint_pos']),
                np.array([t-2]),*[np.array([v]) for v in filtered])[0]
            np.testing.assert_allclose(result['reference'],reference,atol=2e-6,rtol=2e-6)
    finally:host.close()


def test_history_reset_fade_and_preclamp_action_feedback(binary,tmp_path):
    policy=json.loads(DEFAULT.read_text());policy.pop('jump_slot')
    policy.update(observation_history=2,in_shape=[None,96])
    policy['obs_spec']['history']=2
    # Output follows prior action plus a distinct older-frame position term.
    weights=np.zeros((96,12));weights[24:36]=np.eye(12)*.1;weights[60:72]=np.eye(12)*.2
    policy['layers']=[dict(type='dense',activation='',shape=[None,12],weights=[weights.tolist(),[10.]*12])]
    path=tmp_path/'history.json';path.write_text(json.dumps(policy));host=screen.Host(binary,path,1,tmp_path/'controller.log')
    try:
        first=host.request(dict(op='infer',states=[state(policy,2)]))[0]
        assert first['fade']==0
        np.testing.assert_allclose(first['target'],policy['default_joint_pos'])
        np.testing.assert_allclose(first['observation'][48:],np.eye(1,48,5).ravel()*-1)
        changed=state(policy,3);changed['joint_pos']=(np.array(policy['default_joint_pos'])+.1).tolist()
        second=host.request(dict(op='infer',states=[changed]))[0]
        np.testing.assert_allclose(second['raw_action'],10)
        np.testing.assert_allclose(second['effective_action'],5)
        third=host.request(dict(op='infer',states=[state(policy,4)]))[0]
        np.testing.assert_allclose(third['raw_action'],10.52,atol=2e-6)
        np.testing.assert_allclose(third['observation'][24:36],5)
        assert np.any(np.array(third['before_clip'])>np.array(third['target']))
        host.request({'op':'reset'})
        repeat=host.request(dict(op='infer',states=[state(policy,2)]))[0]
        assert repeat==first
    finally:host.close()


def test_heading_policy_parameters_and_replica_isolation(binary,tmp_path):
    policy=json.loads(DEFAULT.read_text());host=screen.Host(binary,DEFAULT,2,tmp_path/'controller.log')
    try:
        host.request(dict(op='infer',states=[state(policy,4,[.3,0,0]),state(policy,4,[.3,0,0],yaw=.4)]))
        results=host.request(dict(op='infer',states=[state(policy,4+1/52,[.3,0,0],yaw=.2),state(policy,4+1/52,[.3,0,0],yaw=.4)]))
        np.testing.assert_allclose(results[0]['effective_command'],[.3,0,-.3],atol=1e-8)
        np.testing.assert_allclose(results[1]['effective_command'],[.3,0,0],atol=1e-8)
    finally:host.close()


def test_explicit_nojump_required(binary):
    proc=subprocess.run([str(binary),str(DEFAULT),'1'],capture_output=True,text=True)
    assert proc.returncode and 'jump capability unmeasured' in proc.stderr


def test_common_range_rejects_stable_high_speed():
    if not STABLE.exists():pytest.skip('local trained artifact unavailable')
    with pytest.raises(ValueError,match='exceed exported vx'):
        screen.validate_inputs({'stable':STABLE},[[1.5,0,0]],LIMITS)


def test_paired_initial_conditions_reproducible_and_nominal_first():
    default=json.loads(DEFAULT.read_text())['default_joint_pos']
    a=screen.initial_conditions(default,4,101)
    assert a==screen.initial_conditions(default,4,101)
    assert a!=screen.initial_conditions(default,4,202)
    assert a[0]['initial_rpy']==[0,0,0] and a[0]['joint_position']==default
    assert max(abs(x) for r in a for x in r['joint_offsets_rad'])<=.01


def test_short_native_physics_screen_preserves_contact_and_contract(binary,tmp_path):
    helper=screen.module(ROOT/'foundation/scripts/check_spider_handoff.py','export_test_helper')
    policies,lower,upper,_=screen.validate_inputs({'shipped':DEFAULT},[[0,0,0]],LIMITS)
    policy=policies['shipped'];initial=screen.initial_conditions(policy['default_joint_pos'],1,101)
    result=screen.evaluate_policy(binary,DEFAULT,policy,initial,[[0,0,0]],.12,.02,lower,upper,tmp_path,helper.make_model)
    trace=np.load(tmp_path/'trace.npz')
    assert result['completed_all_replicas']
    assert sum(g['name'].endswith('_hull') and g['conaffinity']==1 for g in result['collision_masks'])==9
    assert result['mass_kg']==pytest.approx(3.17)
    assert np.all(trace['driver_target']<upper+1e-6)
    assert np.all(trace['driver_target']>lower-1e-6)
    assert len(result['segments'])==1 and result['segments'][0]['samples']>0
    assert np.isfinite(trace['torso_clearance']).all()


def test_failure_reports_withhold_survivor_only_aggregate():
    failure=dict(replica=0,time_s=3,reason='tilt stop')
    result=screen.summary([], [failure], [[0,0,0]])
    assert result['aggregate_comparison_withheld'] and not result['completed_all_replicas']
    assert result['segments'][0]['samples']==0 and result['failures']==[failure]
