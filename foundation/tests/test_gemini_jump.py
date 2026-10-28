"""Actual Float32, immediate-trigger jump path without changing the old screen."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('gemini_jump_test',ROOT/'foundation/scripts/check_gemini_jump.py')
api=importlib.util.module_from_spec(spec);sys.modules[spec.name]=api;spec.loader.exec_module(api)
screen=api.load(ROOT/'foundation/scripts/check_export_locomotion.py','gemini_jump_screen_test')
POLICY=ROOT/'pupper_gait_deploy/policies/default_policy.json'
REFERENCE=ROOT/'gemini_pupper/tricks/jump/reference.json'

@pytest.fixture(scope='module')
def binary(tmp_path_factory):return screen.build_controller(tmp_path_factory.mktemp('api-jump-host'))[0]

def state(p,t):
    return dict(elapsed_s=t,command=[0,0,0],joint_pos=p['default_joint_pos'],angular_velocity=[0,0,0],quaternion=[1,0,0,0],debug=True)


def test_actual_transport_and_non_grid_immediate_trigger_match_reference_math(binary,tmp_path):
    from mjlab.tasks.pupper_gait.export import mixed_jump_reference_offset_numpy
    from mjlab.tasks.pupper.export import json_forward
    p=json.loads(POLICY.read_text());ref=json.loads(REFERENCE.read_text());payload=api.message_payload(ref)
    before=REFERENCE.read_bytes()
    assert np.array_equal(np.array(payload),np.asarray([ref[k] for k in ('playback_s','active_s','cross_fade_s')]+np.array(ref['table']).ravel().tolist(),np.float32).astype(float))
    updated=dict(p['jump_slot'],playback_s=payload[0],active_s=payload[1],cross_fade_s=payload[2],jump_table=np.array(payload[3:]).reshape(100,12).tolist())
    host=api.ApiHost(binary,POLICY,1,tmp_path/'controller.log');trigger=2450/520
    try:
        assert host.request({'op':'infer','states':[state(p,4.2)]})[0]['reference_start_s'] is None
        for i,t in enumerate([trigger,trigger+.03,trigger+.06,trigger+.4,trigger+.79,trigger+.81]):
            r=state(p,t)
            if i==0:r['reference_payload']=payload
            out=host.request({'op':'infer','states':[r]})[0]
            expected=mixed_jump_reference_offset_numpy(p['gait_reference'],updated,np.asarray(p['default_joint_pos']),np.array([t-2]),*[np.array([v]) for v in out['effective_command']],np.array([trigger-2]))[0]
            np.testing.assert_allclose(out['reference'],expected,atol=2e-6,rtol=2e-6)
            np.testing.assert_allclose(out['raw_action'],json_forward(p,np.array(out['observation'])[None,:])[0],atol=3e-5,rtol=3e-5)
            assert out['reference_start_s']==trigger-2
            assert out['reference_received']==(i==0)
        assert (trigger-2)/p['jump_slot']['grid_s'] != round((trigger-2)/p['jump_slot']['grid_s'])
    finally:host.close()
    assert REFERENCE.read_bytes()==before


def test_api_mode_does_not_change_reference_without_a_message(binary,tmp_path):
    p=json.loads(POLICY.read_text());a=api.ApiHost(binary,POLICY,1,tmp_path/'api.log');b=screen.Host(binary,POLICY,1,tmp_path/'plain.log')
    try:
        for t in [2.,2.5,4.2,5.1,5.7]:
            request={'op':'infer','states':[state(p,t)]};x=a.request(request)[0];y=b.request(request)[0]
            for field in ['reference','raw_action','effective_action','target','observation']:assert x[field]==y[field]
    finally:a.close();b.close()


def test_reset_clears_api_slot_and_replays_same_payload(binary,tmp_path):
    p=json.loads(POLICY.read_text());payload=api.message_payload(json.loads(REFERENCE.read_text()));host=api.ApiHost(binary,POLICY,1,tmp_path/'controller.log')
    try:
        r=state(p,4.7);r['reference_payload']=payload
        first=host.request({'op':'infer','states':[r]})
        host.request({'op':'infer','states':[state(p,5.)]});host.request({'op':'reset'})
        assert host.request({'op':'infer','states':[r]})==first
    finally:host.close()


@pytest.mark.parametrize('fault',['before_fade','duplicate','wrong_mode','bad_payload','bad_timing'])
def test_invalid_or_unsupported_api_request_is_rejected(binary,fault):
    p=json.loads(POLICY.read_text());payload=api.message_payload(json.loads(REFERENCE.read_text()));r=state(p,3. if fault=='before_fade' else 4.7)
    if fault=='bad_payload':payload=payload[:-1]
    if fault=='bad_timing':payload[1]=2.
    r['reference_payload']=payload;requests=[{'op':'infer','states':[r]}]
    if fault=='duplicate':requests.append({'op':'infer','states':[dict(r,elapsed_s=4.8)]})
    out=subprocess.run([str(binary),str(POLICY),'1','--no-jump' if fault=='wrong_mode' else '--api-reference'],input=''.join(json.dumps(x)+'\n' for x in requests),text=True,capture_output=True)
    assert out.returncode


def test_jump_descriptor_reports_evidence_boundary_and_preserves_loader_contract():
    sys.path.insert(0,str(ROOT/'gemini_pupper/robot'))
    from tricks import load_tricks,reference_message_data,validate_locomotion_contract
    trick=load_tricks()['jump'];description=trick.description.lower()
    assert 'tilt stop' in description and 'not been validated' in description
    assert 'lands standing' not in description
    validate_locomotion_contract(POLICY,[trick])
    payload=reference_message_data(trick)
    np.testing.assert_array_equal(np.asarray(payload,np.float32),api.message_payload(json.loads(REFERENCE.read_text())))
