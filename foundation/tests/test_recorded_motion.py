import copy
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).parents[2]
sys.path.insert(0,str(ROOT/'ik_heuristic_walking_lab'))
from kinematics import fr_leg_fk
from recorded_motion import RecordedMotion, RecordedReplay
spec=importlib.util.spec_from_file_location('motion_recorder',ROOT/'forward_kinematics_lab/record_motion.py')
recorder=importlib.util.module_from_spec(spec);spec.loader.exec_module(recorder)


def trajectory():
    q=np.array([[0,.4,-.8],[.001,.401,-.801],[.002,.402,-.802]])
    return dict(schema='pupper_recorded_foot_v1',frame='base_link',units='metres',leg='front_r',
                times_s=[0,.02,.04],joint_positions=q.tolist(),positions=[fr_leg_fk(x).tolist() for x in q])


def test_finite_motion_uses_body_frame_and_holds_terminal_pose():
    data=trajectory();motion=RecordedMotion(data)
    np.testing.assert_allclose(motion.at(.01),(np.array(data['positions'][0])+data['positions'][1])/2)
    np.testing.assert_allclose(motion.at(10),data['positions'][-1])
    np.testing.assert_allclose(motion.at(0),fr_leg_fk(data['joint_positions'][0]))


@pytest.mark.parametrize('fault',['gap','backwards','nan','offset','fast','frame'])
def test_bad_recordings_refused(fault):
    data=trajectory()
    if fault=='gap':data['times_s'][2]=1
    if fault=='backwards':data['times_s'][2]=.01
    if fault=='nan':data['positions'][0][0]=float('nan')
    if fault=='offset':data['positions'][0][0]+=.075
    if fault=='fast':data['times_s']=[0,.00001,.00002]
    if fault=='frame':data['frame']='hip'
    with pytest.raises(ValueError):RecordedMotion(data)


def recording(tmp_path):
    data=trajectory();rows=[dict(schema='pupper_topic_recording_v1',frame='base_link',units='metres')]
    for t,q,p in zip(data['times_s'],data['joint_positions'],data['positions']):
        rows.append(dict(kind='joints',receipt_s=t+1,header_s=t+100,names=['leg_front_r_3','leg_front_r_1','leg_front_r_2'],positions=[q[2],q[0],q[1]]))
        rows.append(dict(kind='foot',leg='front_r',receipt_s=t+1.001,position=p))
    source=tmp_path/'recording.jsonl';source.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    return source,rows


def test_export_aligns_named_joints_without_overwriting(tmp_path):
    source,_=recording(tmp_path);out=tmp_path/'motion.json'
    result=recorder.export_motion(source,out,'front_r')
    assert result['timing']['maximum_skew_s']==pytest.approx(.001)
    np.testing.assert_allclose(RecordedMotion(out).positions,trajectory()['positions'])
    with pytest.raises(FileExistsError):recorder.export_motion(source,out,'front_r')


@pytest.mark.parametrize('fault',['stale','missing','nonfinite','stamp_reset','clock_jump','frame','array_length'])
def test_export_never_fills_invalid_feedback(tmp_path,fault):
    source,rows=recording(tmp_path)
    if fault=='stale':rows[2]['receipt_s']-=.1
    if fault=='missing':rows[1]['names'][0]='unknown'
    if fault=='nonfinite':rows[2]['position'][0]=None
    if fault=='stamp_reset':rows[3]['header_s']=99
    if fault=='clock_jump':rows[3]['header_s']+=10
    if fault=='frame':rows[0]['frame']='hip'
    if fault=='array_length':rows[1]['positions'].append(0)
    source.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(ValueError):recorder.export_motion(source,tmp_path/'bad.json','front_r')


def test_replay_entry_continuity_pause_and_terminal_hold():
    motion=RecordedMotion(trajectory());replay=RecordedReplay(motion)
    measured=np.array([.05,.5,-.9]);first=measured.copy()
    previous=first.copy()
    for step in range(121):
        elapsed=step*.025
        # Phase starts at an arbitrary monotonic epoch; half-second pause at 1s.
        phase=10+max(0,elapsed-max(0,min(.5,elapsed-1)))
        target=replay.target(phase,measured,100+elapsed)
        assert np.max(abs(target-previous))<=15*.025+1e-9
        if step==0:np.testing.assert_array_equal(target,first)
        if 41<=step<=60:np.testing.assert_array_equal(target,previous)
        measured=target.copy();previous=target.copy()
    np.testing.assert_allclose(fr_leg_fk(target),motion.at(motion.duration),atol=1e-4)


def test_replay_rejects_scheduler_catchup_and_preserves_last_target():
    data=trajectory();q=np.array([[0,.4,-.8],[.4,.4,-.8],[.8,.4,-.8]])
    data.update(times_s=[0,.1,.2],joint_positions=q.tolist(),positions=[fr_leg_fk(x).tolist() for x in q])
    replay=RecordedReplay(RecordedMotion(data));measured=q[0].copy()
    previous=replay.target(0,measured,100)
    with pytest.raises(ValueError,match='target step'):
        replay.target(2.2,measured,102.2)
    np.testing.assert_array_equal(replay.previous_target,previous)


def test_replay_rejects_branch_jump_and_backwards_clock(monkeypatch):
    import recorded_motion
    replay=RecordedReplay(RecordedMotion(trajectory()));measured=np.array([0,.4,-.8])
    previous=replay.target(0,measured,10)
    with pytest.raises(ValueError,match='backward'):replay.target(.1,measured,9)
    monkeypatch.setattr(recorded_motion,'inverse_kinematics',lambda *args,**kwargs:previous+np.array([1,0,0]))
    with pytest.raises(ValueError,match='target step'):replay.target(.025,measured,10.025)
    np.testing.assert_array_equal(replay.previous_target,previous)


@pytest.mark.parametrize('limit',[0,-1,float('nan'),float('inf')])
def test_invalid_timing_limits_fail_closed(tmp_path,limit):
    with pytest.raises(ValueError):RecordedMotion(trajectory(),max_gap=limit)
    source,_=recording(tmp_path)
    with pytest.raises(ValueError):recorder.export_motion(source,tmp_path/'bad.json','front_r',max_skew=limit)


def test_explicit_trim_preserves_source_window_and_never_repairs_internal_gap(tmp_path):
    source,rows=recording(tmp_path)
    rows.insert(1,dict(kind='foot',leg='front_r',receipt_s=.1,position=trajectory()['positions'][0]))
    source.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(ValueError,match='fresh joint'):recorder.export_motion(source,tmp_path/'untrimmed.json','front_r')
    payload=recorder.export_motion(source,tmp_path/'trimmed.json','front_r',start_s=1)
    assert payload['source_window']['excluded_foot_samples']==1
    assert payload['source_window']['requested_start_s']==1
    assert payload['source_window']['selected_foot_samples']==3
    rows[4]['receipt_s']=1.2
    source.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(ValueError):recorder.export_motion(source,tmp_path/'invalid.json','front_r',start_s=1)


def test_actual_mcap_import_is_offline_and_explicitly_trimmed(tmp_path):
    pytest.importorskip('mcap_ros2')
    source=ROOT/'forward_kinematics_lab/wave_01/wave_01_0.mcap'
    output=tmp_path/'record.jsonl'
    counts=recorder.import_mcap(source,output)
    assert counts['/joint_states']==872
    assert counts['/leg_front_r_end_effector_position']==485
    with pytest.raises(FileExistsError):recorder.import_mcap(source,output)
    with pytest.raises(ValueError,match='fresh joint'):recorder.export_motion(output,tmp_path/'whole.json','front_r')
    payload=recorder.export_motion(output,tmp_path/'trimmed.json','front_r',start_s=.92)
    assert payload['source_window']['excluded_foot_samples']==47
    assert payload['timing']['maximum_skew_s']<.05
    motion=RecordedMotion(payload);replay=RecordedReplay(motion)
    measured=motion.joints[0].copy()
    for elapsed in np.arange(0,2+motion.duration+.1,.025):
        measured=replay.target(float(elapsed),measured,float(100+elapsed))
    np.testing.assert_allclose(fr_leg_fk(measured),motion.at(motion.duration),atol=1e-4)
    # The moving rear-right wave exceeds the model joint limits. Do not clip or
    # relabel it into a different leg merely to make the existing bag replayable.
    with pytest.raises(ValueError,match='outside model bounds'):
        recorder.export_motion(output,tmp_path/'back_r.json','back_r',start_s=.92)
    assert not (tmp_path/'back_r.json').exists()


def test_derived_foot_associates_with_its_joint_sample_despite_topic_delay(tmp_path):
    source,rows=recording(tmp_path)
    for row in rows[1:]:
        if row['kind']=='foot':row['receipt_s']+=.011
    source.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    data=recorder.export_motion(source,tmp_path/'delayed.json','front_r')
    np.testing.assert_array_equal(data['joint_positions'],trajectory()['joint_positions'])
    assert data['timing']['maximum_skew_s']==pytest.approx(.012)
