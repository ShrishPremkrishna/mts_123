"""Probe the actual shipped table and conservative full-contact event detection."""
import gzip
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('shipped_jump_test',ROOT/'foundation/scripts/check_shipped_jump.py')
jump=importlib.util.module_from_spec(spec);sys.modules[spec.name]=jump;spec.loader.exec_module(jump)
screen=jump.load(ROOT/'foundation/scripts/check_export_locomotion.py','jump_test_screen')
POLICY=ROOT/'pupper_gait_deploy/policies/default_policy.json'
LIMITS=ROOT/'foundation/configs/class-driver-joint-limits.json'

@pytest.fixture(scope='module')
def binary(tmp_path_factory):return screen.build_controller(tmp_path_factory.mktemp('scheduled-jump-host'))[0]

def state(policy,t,vx=0):
    return dict(elapsed_s=t,command=[vx,0,0],joint_pos=policy['default_joint_pos'],angular_velocity=[0,0,0],quaternion=[1,0,0,0],debug=True)


def test_actual_table_reference_and_network_parity_at_window_boundaries(binary,tmp_path):
    from mjlab.tasks.pupper_gait.export import mixed_jump_reference_offset_numpy
    from mjlab.tasks.pupper.export import json_forward
    p=json.loads(POLICY.read_text());host=screen.Host(binary,POLICY,2,tmp_path/'controller.log',scheduled_jump_start=3.)
    try:
        for elapsed in (4.9,5.,5.03,5.06,5.4,5.74,5.79,5.8,5.9):
            result=host.request(dict(op='infer',states=[state(p,elapsed),state(p,elapsed,.3)]))
            for out in result:
                expected=mixed_jump_reference_offset_numpy(p['gait_reference'],p['jump_slot'],np.asarray(p['default_joint_pos']),
                    np.array([elapsed-2]),*[np.array([v]) for v in out['effective_command']],np.array([3.]))[0]
                np.testing.assert_allclose(out['reference'],expected,atol=2e-6,rtol=2e-6)
                np.testing.assert_allclose(out['raw_action'],json_forward(p,np.array(out['observation'])[None,:])[0],atol=3e-5,rtol=3e-5)
                assert out['scheduled_slot_start_s']==3.
                assert out['jump_reference_active']==(5.<=elapsed<5.8)
    finally:host.close()


def test_reset_replays_immutable_schedule_and_preserves_replica_independence(binary,tmp_path):
    p=json.loads(POLICY.read_text());host=screen.Host(binary,POLICY,2,tmp_path/'controller.log',scheduled_jump_start=3.)
    request=dict(op='infer',slot_start_s=99,states=[state(p,5.3),state(p,5.3,.3)])
    try:
        first=host.request(request)
        host.request(dict(op='infer',states=[state(p,5.4),None]))
        host.request(dict(op='reset'))
        assert host.request(request)==first
        assert first[0]['effective_command']!=first[1]['effective_command']
        assert first[0]['scheduled_slot_start_s']==3.
    finally:host.close()


@pytest.mark.parametrize('start',['nan','inf','1.5','3.1','3oops'])
def test_invalid_schedule_rejected(binary,start):
    result=subprocess.run([str(binary),str(POLICY),'1','--scheduled-jump',start],input='',text=True,capture_output=True)
    assert result.returncode


def test_hop_selection_rejected(binary):
    p=json.loads(POLICY.read_text())
    result=subprocess.run([str(binary),str(POLICY),'1','--scheduled-jump','3.0'],
        input=json.dumps(dict(op='infer',states=[state(p,5.,.5)]))+'\n',text=True,capture_output=True)
    assert result.returncode and 'hop branch' in result.stderr


def test_contact_intervals_reject_support_and_nonjump_bounce():
    t=np.arange(100)/520;z=np.linspace(.2,.24,100);vz=np.ones(100)*.5
    assert jump.flight_events(t,np.ones(100),z,vz,0,1)==[] # hull support still counts even when feet lift
    contact=np.ones(100);contact[10:40]=0
    event=jump.flight_events(t,contact,z,vz,0,1)[0]
    assert event['meets_jump_evidence'] and event['landed']
    assert not jump.flight_events(t,contact,z,vz,.03,1)[0]['meets_jump_evidence'] # already airborne before the slot
    assert not jump.flight_events(t,contact,z*0+.2,vz*0,0,1)[0]['meets_jump_evidence']
    contact[:]=1;contact[10:12]=0
    assert not jump.flight_events(t,contact,z,vz,0,1)[0]['meets_jump_evidence']


def test_physics_samples_are_aligned_full_contact_and_driver_bounded(binary,tmp_path):
    shared=jump.load(ROOT/'foundation/scripts/check_spider_handoff.py','jump_test_shared')
    values,lower,upper,_=screen.validate_inputs({'shipped':POLICY},[[0,0,0]],LIMITS)
    p=values['shipped'];initial=screen.initial_conditions(p['default_joint_pos'],1,101)
    result=jump.evaluate(screen,shared,binary,POLICY,p,initial,[0,0,0],True,3.,3.,lower,upper,tmp_path)
    trace=np.load(tmp_path/'trace.npz')
    np.testing.assert_allclose(np.diff(trace['time_s']),1/520,atol=1e-12)
    np.testing.assert_allclose(trace['time_s']-trace['control_time_s'],1/520,atol=1e-12)
    assert np.all(trace['driver_target']<=upper+1e-6) and np.all(trace['driver_target']>=lower-1e-6)
    with gzip.open(tmp_path/'contacts.jsonl.gz','rt') as stream:contacts=[json.loads(x) for x in stream]
    assert len(contacts)==len(trace['time_s'])
    for row,reported in zip(contacts,trace['ground_contact_count']):
        assert sum(c['distance_m']<=jump.CONTACT_DISTANCE_M for c in row['contacts'])==reported
    assert result['mass_kg']==pytest.approx(3.17)
    # The outcome is evidence, not a test that assumes the shipped policy can jump.
    assert all('flight_events' in x and 'recovery' in x for x in result['replicas'])


@pytest.mark.parametrize('metadata',[{'action_types':['velocity']*12},{'action_types':['position']*11},
    {'action_types':'position'},{'action_type':'velocity'},{'position_target_slew':{}},
    {'required_controller_features':['position_target_slew_v1']}])
def test_unsupported_actuation_contract_refused(metadata):
    with pytest.raises(ValueError):screen.validate_position_contract(metadata)


def test_legacy_and_explicit_position_contract_allowed():
    screen.validate_position_contract({})
    screen.validate_position_contract({'action_types':['position']*12})


def test_native_host_rejects_velocity_metadata(binary,tmp_path):
    p=json.loads(POLICY.read_text());p['action_types']=['velocity']*12
    path=tmp_path/'unsupported.json';path.write_text(json.dumps(p))
    result=subprocess.run([str(binary),str(path),'1','--no-jump'],input='',text=True,capture_output=True)
    assert result.returncode and 'legacy position actions' in result.stderr


def test_build_failure_persists_error_status(tmp_path,monkeypatch):
    from types import SimpleNamespace
    real_load=jump.load
    def failed_build(output):raise RuntimeError('deliberate test build failure')
    def intercepted(path,name):
        value=real_load(path,name)
        if path.name=='check_export_locomotion.py':value.build_controller=failed_build
        return value
    monkeypatch.setattr(jump,'load',intercepted)
    args=SimpleNamespace(policy=POLICY,driver_limits=LIMITS,replicas=1,recovery_seconds=3.,slot_start=3.,
                         seed=101,output=tmp_path/'failed-run',plan_only=False)
    with pytest.raises(RuntimeError,match='deliberate test'):jump.run(args)
    status=json.loads((args.output/'status.json').read_text())
    assert status['status']=='error' and status['error_type']=='RuntimeError'
