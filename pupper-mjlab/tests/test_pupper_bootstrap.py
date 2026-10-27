"""Capture regression tests load the NumPy core without starting a simulator."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('pupper_bootstrap_core',Path(__file__).parents[1]/'src/mjlab/tasks/pupper_gait/bootstrap.py')
b=importlib.util.module_from_spec(spec); spec.loader.exec_module(b)


def trace(cycles=8,n=50):
    phase=np.mod(np.arange(cycles*n+1)/n,1.)[:,None]
    q=np.repeat(np.sin(2*np.pi*phase)[:,:,None],12,axis=2)
    shape=phase.shape
    return dict(joint_pos=q,phase=phase,episode=np.zeros(shape,int),valid=np.ones(shape,bool),
                command=np.broadcast_to([1.5,0,0],(*shape,3)).copy())


def test_cycle_alignment_preserves_motion_and_rejects_wrong_command():
    t=trace()
    table,report=b.fold_cycles(t,[1.5,0,0],samples=100)
    assert report['cycles']>=5
    np.testing.assert_allclose(table[:,0],np.sin(2*np.pi*np.arange(100)/100),atol=.004)
    with pytest.raises(ValueError): b.fold_cycles(t,[-1.5,0,0])


def test_no_folding_across_resets_or_invalid_intervals():
    t=trace(cycles=5)
    # Put a reset inside every full cycle: no cycle can remain admissible.
    t['episode'][:,0]=np.arange(len(t['phase']))//25
    with pytest.raises(ValueError): b.fold_cycles(t,[1.5,0,0],min_cycles=1)
    t=trace(); t['valid'][:]=False
    with pytest.raises(ValueError): b.fold_cycles(t,[1.5,0,0],min_cycles=1)


def test_periodicity_symmetry_and_validation():
    t=trace(); table,_=b.fold_cycles(t,[1.5,0,0])
    fixed=b.mirror_average(table)
    for right,left in ((0,3),(6,9)):
        np.testing.assert_allclose(fixed[:,left:left+3],-np.roll(fixed[:,right:right+3],50,axis=0),atol=1e-10)
    assert b.validate_reference(table)['peak_joint_velocity_rad_s']>0
    with pytest.raises(ValueError): b.validate_reference(np.zeros((8,11)))
    bad=table.copy(); bad[3,0]=np.nan
    with pytest.raises(ValueError): b.validate_reference(bad)
    with pytest.raises(ValueError): b.validate_reference(table,limits=np.tile([-.1,.1],(12,1)))


def test_reference_artifact_is_direction_split_and_hashed(tmp_path):
    t=trace(); table,_=b.fold_cycles(t,[1.5,0,0])
    result=b.save_references(tmp_path,table,table*-.5,{'test':True},frequency_mult=2.)
    assert result['reference_sha256']==b.file_hash(tmp_path/'captured_reach_gaits.npz')
    with np.load(tmp_path/'captured_reach_gaits.npz',allow_pickle=False) as f:
        np.testing.assert_allclose(f['reach_back'],table*-.5)
        assert list(f['joint_names'])==b.JOINT_ORDER


def test_summary_counts_failure_outside_steady_state_and_measures_achieved_speed():
    t=trace(cycles=2); n,e=t['phase'].shape
    t.update(segment=np.zeros(n,int),terminated=np.zeros((n,e),bool),timeout=np.zeros((n,e),bool),
             velocity=np.broadcast_to([1.2,.1,.2],(n,e,3)).copy(),torque=np.ones((n,e,12))*2,
             foot_contact=np.ones((n,e,4),bool),foot_speed_xy=np.ones((n,e,4))*.05,heading=np.zeros((n,e)))
    t['terminated'][0]=True; t['valid'][0]=False
    result=b.summarize_trace(t,.02)['segments'][0]
    assert result['failures']==1
    assert result['mean_achieved_vx_m_s']==pytest.approx(1.2)
    assert result['mean_xy_error_m_s']==pytest.approx(np.sqrt(.3**2+.1**2))
    assert result['mean_torque_squared_nm2']==4
    assert result['mean_contact_slip_m_s']==pytest.approx(.05)


def test_promotion_rejects_faster_failing_policy():
    import copy
    segment=dict(command=[1.5,0,0],valid_samples=1000,failures=0,mean_xy_error_m_s=.2,
                 mean_torque_squared_nm2=1.,mean_contact_slip_m_s=.1,mean_abs_yaw_error_rad_s=.1)
    baseline=dict(provenance=dict(task='Mixed',seed=1,zero_policy=False),segments=[segment])
    candidate=copy.deepcopy(baseline); candidate['segments'][0]['mean_xy_error_m_s']=.1
    assert b.compare_evaluations(baseline,candidate)['promotable_for_this_seed']
    candidate['segments'][0]['failures']=1
    assert not b.compare_evaluations(baseline,candidate)['promotable_for_this_seed']
    candidate['provenance']['seed']=2
    with pytest.raises(ValueError): b.compare_evaluations(baseline,candidate)


def test_jump_selection_requires_flight_and_landing_in_same_episode():
    t=trace(cycles=3); n,e=t['phase'].shape
    t.update(dt=np.asarray(.02),foot_contact=np.ones((n,e,4),bool),height=np.ones((n,e))*.15,
             terminated=np.zeros((n,e),bool))
    with pytest.raises(ValueError): b.select_jump(t)
    t['foot_contact'][40:50]=False
    t['height'][40:50]=.25
    selected=b.select_jump(t)
    assert selected['start']<40 and selected['end']>50
    assert selected['flight_s']==pytest.approx(.2)
    t['terminated'][49]=True
    with pytest.raises(ValueError): b.select_jump(t)
