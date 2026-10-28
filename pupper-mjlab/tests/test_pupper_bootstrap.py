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
    t['target_clipped']=np.zeros((n,e,12),bool); t['target_clipped'][:,:,4]=True
    t['target_range_excess_rad']=np.zeros((n,e,12)); t['target_range_excess_rad'][:,:,4]=.2
    result=b.summarize_trace(t,.02)['segments'][0]
    assert result['failures']==1
    assert result['mean_achieved_vx_m_s']==pytest.approx(1.2)
    assert result['mean_xy_error_m_s']==pytest.approx(np.sqrt(.3**2+.1**2))
    assert result['mean_torque_squared_nm2']==4
    assert result['mean_contact_slip_m_s']==pytest.approx(.05)
    assert result['target_clipped_fraction']==pytest.approx(1/12)
    assert result['any_target_clipped_sample_fraction']==1
    assert result['target_clipped_fraction_by_joint'][4]==1
    assert result['maximum_target_range_excess_rad']==pytest.approx(.2)


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


@pytest.mark.parametrize('key,value',[
    ('mean_xy_error_m_s',float('nan')),
    ('mean_torque_squared_nm2',None),
    ('mean_contact_slip_m_s',float('inf')),
    ('mean_abs_yaw_error_rad_s',-1.),
    ('failures',float('nan')),
    ('valid_samples',1000.5),
    ('command',[1.5,0,float('nan')]),
])
def test_promotion_rejects_invalid_evidence(key,value):
    import copy
    segment=dict(command=[1.5,0,0],valid_samples=1000,failures=0,mean_xy_error_m_s=.2,
                 mean_torque_squared_nm2=1.,mean_contact_slip_m_s=.1,mean_abs_yaw_error_rad_s=.1)
    baseline=dict(provenance={},segments=[segment])
    candidate=copy.deepcopy(baseline);candidate['segments'][0]['mean_xy_error_m_s']=.1
    candidate['segments'][0][key]=value
    with pytest.raises(ValueError,match='Promotion requires'):
        b.compare_evaluations(baseline,candidate)


def test_promotion_rejects_missing_cost_and_invalid_thresholds():
    segment=dict(command=[1.5,0,0],valid_samples=1000,failures=0,mean_xy_error_m_s=.2,
                 mean_contact_slip_m_s=.1,mean_abs_yaw_error_rad_s=.1)
    report=dict(provenance={},segments=[segment])
    with pytest.raises(ValueError,match='mean_torque_squared'):
        b.compare_evaluations(report,report)
    with pytest.raises(ValueError,match='thresholds'):
        b.compare_evaluations(report,report,min_improvement=float('nan'))


def test_target_audit_preserves_actor_history_and_exposes_preclip_request():
    import torch
    from types import SimpleNamespace
    source=Path(__file__).parents[1]/'src/mjlab/scripts/pupper_locomotion.py'
    module_spec=importlib.util.spec_from_file_location('pupper_locomotion_audit',source)
    module=importlib.util.module_from_spec(module_spec); module_spec.loader.exec_module(module)
    limits=torch.tensor([[[-.3,.3],[-.4,.4]]])
    robot=SimpleNamespace(data=SimpleNamespace(joint_pos_limits=limits,encoder_bias=torch.tensor([[.05,-.05]])))
    term=SimpleNamespace(scale=.5,offset=0.,target_ids=torch.tensor([0,1]),cfg=SimpleNamespace(clip=None))
    raw=torch.tensor([[2.,-2.]])
    before=module.action_target_diagnostics(raw,term,robot,clip_actions=1.)
    torch.testing.assert_close(before['unclipped_target_joint_pos'],torch.tensor([[.5,-.5]]))
    torch.testing.assert_close(before['physical_target_joint_pos'],torch.tensor([[.45,-.45]]))
    assert before['target_clipped'].all()
    term.cfg.clip={'.*':(-.3,.3)}; term._clip=limits
    after=module.action_target_diagnostics(raw,term,robot,clip_actions=1.)
    torch.testing.assert_close(after['raw_actions'],torch.tensor([[2.,-2.]]))
    torch.testing.assert_close(after['unclipped_target_joint_pos'],before['unclipped_target_joint_pos'])
    torch.testing.assert_close(after['target_joint_pos'],torch.tensor([[.3,-.4]]))
    torch.testing.assert_close(after['physical_target_joint_pos'],torch.tensor([[.25,-.35]]))
    torch.testing.assert_close(after['target_range_excess_rad'],torch.tensor([[.2,.1]]))


def test_promotion_requires_same_deployment_clamp_contract():
    a=dict(provenance=dict(deployment_target_clamp=False))
    c=dict(provenance=dict(deployment_target_clamp=True))
    with pytest.raises(ValueError,match='deployment_target_clamp'):
        b.compare_evaluations(a,c)


@pytest.mark.parametrize('change',['driver_added','bounds_changed','source_changed','torso_added'])
def test_promotion_rejects_changed_driver_or_contact_physics(change):
    import copy
    a={'provenance':{},'segments':[]}
    contract={'contract':'controller_then_driver_joint_target_clamp_v1',
              'effective_limits':{'joint':[-1,1]},'source':{'path':'/one/limits.json','sha256':'same'}}
    if change in ('bounds_changed','source_changed'):a['provenance']['driver_target_contract']=contract
    c=copy.deepcopy(a)
    if change=='driver_added':c['provenance']['driver_target_contract']=contract
    if change=='bounds_changed':c['provenance']['driver_target_contract']['effective_limits']['joint']=[-.5,.5]
    if change=='source_changed':c['provenance']['driver_target_contract']['source']['sha256']='different'
    if change=='torso_added':c['provenance']['locomotion_torso_contact']=True
    with pytest.raises(ValueError,match='driver_target_contract|locomotion_torso_contact'):
        b.compare_evaluations(a,c)


def test_driver_contract_comparison_ignores_only_location_and_description():
    import copy
    a={'provenance':{'driver_target_contract':{'effective_limits':{'joint':[-1,1]},
         'source':{'path':'/old/path','sha256':'same'},'scope':'old description'}},'segments':[]}
    c=copy.deepcopy(a);c['provenance']['driver_target_contract']['source']['path']='/restored/path'
    c['provenance']['driver_target_contract']['scope']='new description'
    assert not b.compare_evaluations(a,c)['promotable_for_this_seed']  # no high-speed data, but contracts match


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


def test_new_jump_capture_uses_measured_touchdown_instead_of_legacy_time(tmp_path,monkeypatch):
    from mjlab.tasks.pupper_gait.mdp import gait_reference as ref
    path=tmp_path/'captured_jump_gait.npz'
    np.savez(path,jump=np.zeros((40,12)),duration_s=.8,touchdown_s=.5)
    monkeypatch.setattr(ref,'_CAPTURED_JUMP_FILE',path)
    assert ref.jump_capture_duration_s()==pytest.approx(.8)
    assert ref.jump_capture_touchdown_s()==pytest.approx(.5)
    np.savez(path,jump=np.zeros((40,12)),duration_s=.8,touchdown_s=.9)
    with pytest.raises(ValueError):ref.jump_capture_touchdown_s()


def test_jump_launch_reward_cannot_grow_without_bound(monkeypatch):
    from types import SimpleNamespace
    import torch
    from mjlab.tasks.pupper_gait.mdp import mixed_jump
    monkeypatch.setattr(mixed_jump,'_slot_progress',lambda env,slot:(torch.tensor([False,True,True]),None))
    velocity=torch.tensor([[0.,0.,100.],[0.,0.,1.],[0.,0.,100.]])
    env=SimpleNamespace(scene={'robot':SimpleNamespace(data=SimpleNamespace(root_link_lin_vel_w=velocity))})
    result=mixed_jump.slot_jump_up_velocity(env,.8,velocity_cap=2.)
    torch.testing.assert_close(result,torch.tensor([0.,1.,2.]))


def test_jump_selection_checks_bounds_before_ranking_actual_windows():
    t=trace(cycles=3);n,e=t['phase'].shape
    t.update(dt=np.asarray(.02),foot_contact=np.ones((n,e,4),bool),height=np.full((n,e),.15),terminated=np.zeros((n,e),bool))
    t['foot_contact'][40:50]=False;t['height'][40:50]=.25
    t['foot_contact'][100:110]=False;t['height'][100:110]=.22
    t['joint_pos'][40,0,0]=1.1
    original=t['joint_pos'].copy();limits=np.tile([-1.,1.],(12,1))
    assert b.select_jump(t)['start']==25
    selected=b.select_jump(t,limits=limits)
    assert selected['start']==85 and selected['end']>110
    np.testing.assert_array_equal(t['joint_pos'],original)
    t['joint_pos'][100,0,0]=1.1
    with pytest.raises(ValueError,match='No clean jump'):
        b.select_jump(t,limits=limits)
