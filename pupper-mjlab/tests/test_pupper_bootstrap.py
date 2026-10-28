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


def jump_trace():
    t=trace(cycles=3);n,e=t['phase'].shape
    t.update(dt=np.asarray(.02),foot_contact=np.ones((n,e,4),bool),robot_ground_contact_count=np.full((n,e),4),
             height=np.full((n,e),.15),vertical_velocity=np.zeros((n,e)),
             root_quaternion=np.broadcast_to([1.,0.,0.,0.],(n,e,4)).copy(),terminated=np.zeros((n,e),bool))
    t['robot_com_height_m']=t['height'];t['robot_com_vertical_velocity_m_s']=t['vertical_velocity']
    return t


def add_observed_flight(t,start,end,peak=.25):
    t['foot_contact'][start:end]=False;t['robot_ground_contact_count'][start:end]=0
    phase=np.linspace(0,1,end-start)
    t['height'][start:end,0]=.18+(peak-.18)*4*phase*(1-phase)
    t['vertical_velocity'][start:end,0]=4*(peak-.18)*(1-2*phase)/((end-start-1)*.02)


def test_jump_selection_requires_flight_and_landing_in_same_episode():
    t=jump_trace()
    with pytest.raises(ValueError):b.select_jump(t)
    add_observed_flight(t,40,50)
    selected=b.select_jump(t)
    assert selected['start']<40 and selected['end']>50
    assert selected['flight_s']==pytest.approx(.18)
    assert selected['flight_nominal_bin_s']==pytest.approx(.2)
    t['terminated'][49]=True
    with pytest.raises(ValueError):b.select_jump(t)


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
    t=jump_trace();add_observed_flight(t,40,50,.25);add_observed_flight(t,100,110,.22)
    t['joint_pos'][40,0,0]=1.1
    original=t['joint_pos'].copy();limits=np.tile([-1.,1.],(12,1))
    assert b.select_jump(t)['start']==25
    selected=b.select_jump(t,limits=limits)
    assert selected['start']==85 and selected['end']>110
    np.testing.assert_array_equal(t['joint_pos'],original)
    t['joint_pos'][100,0,0]=1.1
    with pytest.raises(ValueError,match='No clean jump'):b.select_jump(t,limits=limits)


@pytest.mark.parametrize('defect',['missing_body','missing_com','downward','no_rise_in_flight','torso_support','body_support','one_touch','unstable_recovery','nan_height'])
def test_jump_capture_rejects_incomplete_or_false_physical_evidence(defect):
    t=jump_trace();add_observed_flight(t,40,50)
    if defect=='missing_body':del t['robot_ground_contact_count']
    elif defect=='missing_com':del t['robot_com_height_m']
    elif defect=='downward':t['vertical_velocity'][40:50]=-.2
    elif defect=='no_rise_in_flight':t['height'][40:50]=.25
    elif defect=='torso_support':
        t['torso_ground_signed_distance_m']=np.ones_like(t['height'])*.04;t['torso_ground_signed_distance_m'][40:50]=-.01
    elif defect=='body_support':t['robot_ground_contact_count'][40:50]=1
    elif defect=='one_touch':t['foot_contact'][50:]=False;t['foot_contact'][51]=True
    elif defect=='unstable_recovery':t['root_quaternion'][60:,0]=[np.cos(.5/2),np.sin(.5/2),0,0]
    elif defect=='nan_height':t['height'][44]=np.nan
    with pytest.raises(ValueError):b.select_jump(t)


def test_capture_metadata_serializes_and_manual_window_cannot_bypass_gate(tmp_path,monkeypatch):
    import json,sys
    t=jump_trace();add_observed_flight(t,40,50)
    selected=b.select_jump(t,limits=np.tile([-1.,1.],(12,1)))
    json.dumps(selected,allow_nan=False)
    assert type(selected['start']) is int and type(selected['end']) is int
    assert b.select_jump(t,window=(selected['start'],selected['end'],0))['flight_start']==40
    t['contact_audit_metadata_json']=np.asarray(json.dumps({'source_trace_sha256':'unit-test-source','motion_unchanged':True}))
    source=tmp_path/'trace.npz';np.savez_compressed(source,**t)
    monkeypatch.setattr(b,'robot_limits',lambda:np.tile([-1.,1.],(12,1)))
    output=tmp_path/'capture';monkeypatch.setattr(sys,'argv',['bootstrap','jump',str(source),str(output),'--auto'])
    b.main()
    assert json.loads((output/'jump_report.json').read_text())['contact_audit_provenance']['source_trace_sha256']=='unit-test-source'
    with np.load(output/'captured_jump_gait.npz') as capture:
        np.testing.assert_array_equal(capture['jump'],t['joint_pos'][selected['start']:selected['end'],0])
    t['vertical_velocity'][40:50]=-.2;np.savez_compressed(source,**t)
    rejected=tmp_path/'bad';monkeypatch.setattr(sys,'argv',['bootstrap','jump',str(source),str(rejected),'--start',str(selected['start']),'--end',str(selected['end'])])
    with pytest.raises(ValueError):b.main()
    assert not rejected.exists()


def test_capture_can_include_later_observed_recovery_without_time_stretch():
    t=jump_trace();add_observed_flight(t,40,50)
    t['foot_contact'][50:75]=False;t['foot_contact'][51]=True
    selected=b.select_jump(t)
    assert selected['end']==81
    assert (selected['end']-selected['start']-1)*float(t['dt'])==pytest.approx(1.10)
    assert selected['touchdown_s']==pytest.approx(.5)


def test_positive_com_rise_uses_disclosed_numeric_tolerance_not_five_mm():
    t=jump_trace();add_observed_flight(t,40,50)
    t['height'][40:50,0]=.20+np.sin(np.linspace(0,np.pi,10))*.004
    selected=b.select_jump(t)
    assert .0001<selected['rise_after_liftoff_m']<.005
    assert selected['gate']['version']=='all_body_com_sample_span_v3'
    t['height'][40:50,0]=.20+np.sin(np.linspace(0,np.pi,10))*.00005
    with pytest.raises(ValueError):b.select_jump(t)


def test_recovery_requires_six_50hz_samples_to_span_one_tenth_second():
    t=jump_trace();add_observed_flight(t,40,50)
    t['foot_contact'][50:]=False;t['foot_contact'][61:66]=True
    with pytest.raises(ValueError,match='No clean jump'):b.select_jump(t)
    t['foot_contact'][66]=True
    selected=b.select_jump(t)
    assert selected['end']==67 and selected['recovery_sample_count']==6
    assert selected['recovery_s']==pytest.approx(.10)
    assert selected['recovery_nominal_bin_s']==pytest.approx(.12)
    assert (selected['recovery_sample_count']-1)*float(t['dt'])==pytest.approx(.10)


def test_minimum_flight_uses_observed_span_not_count_times_dt():
    short=jump_trace();add_observed_flight(short,40,44)
    with pytest.raises(ValueError,match='No clean jump'):b.select_jump(short)
    enough=jump_trace();add_observed_flight(enough,40,45)
    selected=b.select_jump(enough)
    assert selected['flight_sample_count']==5
    assert selected['flight_s']==pytest.approx(.08)
    assert selected['flight_nominal_bin_s']==pytest.approx(.10)


def test_evaluation_enumerates_all_admissible_events_before_any_ranking():
    t=jump_trace();add_observed_flight(t,40,50,.25);add_observed_flight(t,100,110,.22)
    all_events=b.select_jump(t,return_all=True)
    assert [x['flight_start'] for x in all_events]==[40,100]
    assert b.select_jump(t)==all_events[0]
    t['joint_pos'][40,0,0]=1.1
    bounded=b.select_jump(t,return_all=True,limits=np.tile([-1.,1.],(12,1)))
    assert [x['flight_start'] for x in bounded]==[100]


@pytest.mark.parametrize('minimum',[float('nan'),float('inf'),-.1,0.,.079])
def test_capture_flight_minimum_cannot_be_disabled_or_loosened(minimum):
    t=jump_trace();add_observed_flight(t,40,50)
    with pytest.raises(ValueError,match='cannot be loosened'):
        b.select_jump(t,min_flight_s=minimum)


def test_stricter_capture_flight_minimum_is_recorded_in_evidence():
    t=jump_trace();add_observed_flight(t,40,50)
    selected=b.select_jump(t,min_flight_s=.1)
    assert selected['gate']['min_flight_s']==.1
    assert b.JUMP_CAPTURE_GATE['min_flight_s']==.08
