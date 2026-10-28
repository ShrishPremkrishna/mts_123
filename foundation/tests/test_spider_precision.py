import importlib.util
from pathlib import Path
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]


def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'foundation/scripts'/f'{name}.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def test_stops_measure_sustained_motion_and_censor_resets(tmp_path):
    module=load('analyze_spider_stops');n=20;worlds=3
    segment=np.repeat([0,1],10);command=np.zeros((n,worlds,3));command[:10,:,0]=.1
    velocity=np.zeros_like(command);velocity[:13,:,0]=.1;velocity[13:,1,0]=.1
    episode=np.zeros((n,worlds),dtype=int);episode[15:,2]=1
    trace=tmp_path/'trace.npz'
    np.savez(trace,segment=segment,command=command,velocity=velocity,
             root_position=np.cumsum(velocity*.1,axis=0),episode=episode,
             terminated=np.zeros((n,worlds),bool),timeout=np.zeros((n,worlds),bool),
             torque=np.zeros((n,worlds,12)),dt=.1)
    result=module.analyze(trace)
    assert result['total_stops']==3 and result['settled_stops']==1
    assert result['reset_or_timeout_stops']==1
    assert result['stops'][0]['time_to_sustained_stop_s']==pytest.approx(.3)
    assert result['stops'][1]['time_to_sustained_stop_s'] is None
    assert result['stops'][2]['reset_or_timeout']
    assert result['stops'][2]['displacement_full_segment_m'] is None
    assert result['stops'][2]['final_planar_speed_m_s'] is None


def test_precision_plan_changes_only_std_and_isolated_destination(monkeypatch,tmp_path):
    module=load('colab_spider_precision')
    monkeypatch.setattr(module,'sha',lambda _:module.CHECKPOINT_SHA)
    monkeypatch.setattr(module.subprocess,'check_output',lambda *a,**k:'expected\n')
    a,common_a=module.plan(ROOT,tmp_path/'checkpoint',tmp_path,'baseline','expected')
    b,common_b=module.plan(ROOT,tmp_path/'checkpoint',tmp_path,'precision','expected')
    assert common_a==common_b
    ca=a['training_command'];cb=b['training_command']
    differing=[i for i,(x,y) in enumerate(zip(ca,cb,strict=True)) if x!=y]
    assert differing==[ca.index('--output')+1,ca.index('--linear-tracking-std')+1]
    assert a['std']==pytest.approx(.1**.5) and b['std']==.08
    assert ca[ca.index('--iterations')+1]=='1000'
    assert '--driver-limits' in ca and '--compose-com-randomization' in ca
    assert '--locomotion-torso-contact' in ca


def test_precision_plan_refuses_wrong_checkpoint(monkeypatch,tmp_path):
    module=load('colab_spider_precision');monkeypatch.setattr(module,'sha',lambda _:'wrong')
    with pytest.raises(ValueError,match='exact corrected'):
        module.plan(ROOT,tmp_path/'checkpoint',tmp_path,'precision','expected')
