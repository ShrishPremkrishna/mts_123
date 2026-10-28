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


@pytest.mark.parametrize('field',['playback_s','active_s','cross_fade_s','table'])
def test_diagnostic_refuses_motion_redesign_disguised_as_class_reference(field):
    p=json.loads(POLICY.read_text());r=json.loads(REFERENCE.read_text())
    if field=='table':r['table'][0][0]+=.01
    else:r[field]*=.99
    with pytest.raises(ValueError,match='class table and timing'):
        api.validate_class_reference(r,p)


def test_input_mutation_is_not_accepted_as_a_completed_diagnostic(tmp_path):
    p=tmp_path/'reference.json';p.write_bytes(REFERENCE.read_bytes())
    expected={'reference.json':api.sha(p)};api.verify_input_hashes({'reference.json':p},expected)
    p.write_text(p.read_text()+' ')
    with pytest.raises(RuntimeError,match='input changed'):
        api.verify_input_hashes({'reference.json':p},expected)


def test_nominal_return_window_reports_actual_grid_end_and_censoring():
    # Nominal .9996s admits1.0s, and the last-second screen contains520
    # observations spanning519/520s. A stopped replica cannot borrow them.
    full = np.arange(1, 522) / 520
    stopped = np.arange(1, 200) / 520
    trace = {'time_s': np.concatenate([full, stopped]),
             'replica': np.concatenate([np.zeros(len(full), dtype=int),
                                        np.ones(len(stopped), dtype=int)]),
             'value': np.arange(len(full) + len(stopped))}
    window, evidence = api.nominal_return_window(trace, .9996, 3)
    expected = trace['time_s'] <= .9996 + 1 / 520
    for key in trace:
        np.testing.assert_array_equal(window[key], trace[key][expected])
    complete, censored, absent = evidence['replicas']
    assert complete['last_observed_elapsed_s'] == 1.
    assert complete['endpoint_minus_nominal_return_s'] == pytest.approx(.0004)
    assert complete['recovery_samples'] == 520
    assert complete['recovery_first_to_last_span_s'] == pytest.approx(519 / 520)
    assert complete['recovery_nominal_sample_bins_s'] == 1.
    assert censored['last_observed_elapsed_s'] == 199 / 520
    assert censored['endpoint_minus_nominal_return_s'] < 0
    assert censored['recovery_samples'] == 199
    assert absent['last_observed_elapsed_s'] is None
    assert absent['recovery_first_to_last_span_s'] is None
    assert absent['recovery_samples'] == 0


def test_exact_grid_return_keeps_and_discloses_legacy_step_allowance():
    trace = {'time_s': np.array([1., 1. + 1 / 520, 1. + 2 / 520]),
             'replica': np.zeros(3, dtype=int)}
    window, evidence = api.nominal_return_window(trace, 1., 1)
    assert len(window['time_s']) == 2
    assert evidence['replicas'][0]['endpoint_minus_nominal_return_s'] == pytest.approx(1 / 520)
    assert 'not the actual API-return instant' in evidence['timing_scope']


def test_return_grid_with_full_prehistory_exposes_shifted_one_second_span():
    times = np.arange(1, 1042) / 520
    trace = {'time_s': times, 'replica': np.zeros(len(times), dtype=int)}
    _, evidence = api.nominal_return_window(trace, 1.9996, 1)
    observed = evidence['replicas'][0]
    assert observed['recovery_samples'] == 521
    assert observed['recovery_first_sample_elapsed_s'] == 1.
    assert observed['recovery_last_sample_elapsed_s'] == 2.
    assert observed['recovery_first_to_last_span_s'] == 1.
    assert observed['recovery_nominal_sample_bins_s'] == pytest.approx(521 / 520)
    assert observed['endpoint_minus_nominal_return_s'] == pytest.approx(.0004)
