import json
from pathlib import Path
import numpy as np
import pytest
from implicit_compliance.data import labels_for, split_recordings, resample_recording, valid_starts, CHANNELS
from implicit_compliance.streaming import MotorWindow, motor_channels, ContactPauseClock


def test_label_boundaries_and_material_precedence():
    assert labels_for('45 degrees/45-very-rough') == (3,4)
    assert labels_for('None/0_None') == (0,0)
    assert labels_for('foam/75 degrees') == (5,5)
    with pytest.raises(ValueError): labels_for('30/60-rough')


def test_split_entire_recordings_and_conditions():
    records=[dict(id=f'{angle}-{material}-{session}',incline=angle,material=material)
             for angle in range(6) for material in range(6) for session in range(2)]
    split=split_recordings(records,group_by='condition')
    assert split==split_recordings(records,group_by='condition')
    assert set(split.values())=={'train','val','test'}
    for angle in range(6):
        for material in range(6):
            assert split[f'{angle}-{material}-0']==split[f'{angle}-{material}-1']


def test_resampling_is_physical_time_and_contact_is_causal():
    t=np.arange(0,.501,.01)
    joints=np.column_stack([t]+[t*i for i in range(1,10)])
    labels=np.array([[0,0],[.2,1],[.4,0],[.5,0]])
    ts,x,y,valid=resample_recording(joints,labels,sample_hz=100)
    assert y[19]==0 and y[20]==1 and y[40]==0
    np.testing.assert_allclose(x[:,0],ts,atol=1e-6)
    valid[10]=False
    starts=valid_starts(valid,5,1)
    assert all(not (s<=10<s+5) for s in starts)


def test_streaming_matches_offline_resampling_and_rejects_gaps():
    ring=MotorWindow(window=10,sample_hz=100,max_gap_s=.025)
    result=None
    for t in np.arange(0,.21,.01): result=ring.push(t,np.arange(9)*t)
    assert result.shape==(10,9)
    np.testing.assert_allclose(result[:,1],np.arange(.11,.21,.01),atol=1e-6)
    assert ring.push(.4,np.ones(9)) is None
    assert ring.push(.3,np.ones(9)) is None


def test_joint_state_name_mapping_requires_effort():
    names=['leg_front_r_3','leg_front_r_1','leg_front_r_2']
    np.testing.assert_array_equal(motor_channels(names,[3,1,2],[6,4,5],[9,7,8]),np.arange(1,10))
    with pytest.raises(ValueError): motor_channels(names,[3,1,2],[6,4,5],[])


def test_pause_exact_once_until_released_then_rearms():
    clock=ContactPauseClock()
    assert clock.advance(0,.9)==(0.,False)
    _,event=clock.advance(.1,.9); assert event
    phase,_=clock.advance(.5,.99); assert phase==pytest.approx(.1)
    phase,event=clock.advance(1.2,.99); assert phase==pytest.approx(.2) and not event
    for t in (1.6,1.7,1.8): clock.advance(t,.1)
    clock.advance(1.9,.9)
    _,event=clock.advance(2.,.9); assert event


def test_models_losses_pcgrad_and_export(tmp_path):
    torch=pytest.importorskip('torch')
    from implicit_compliance.models import JointStateCNN, DeployContact, PCGrad
    from implicit_compliance.train import task_losses,classification_metrics
    torch.set_num_threads(1)
    model=JointStateCNN()
    x=torch.randn(4,100,9)
    y=torch.tensor([[0,0,0]]*4)
    losses=task_losses(model(x),y,'implicit')
    assert len(losses)==1
    optimizer=PCGrad(torch.optim.Adam(model.parameters(),lr=.001))
    optimizer.step(losses)
    assert model.incline.weight.grad is not None
    assert torch.count_nonzero(model.incline.weight.grad)==0
    y[:,0]=1
    optimizer.step(task_losses(model(x),y,'implicit'))
    wrapper=DeployContact(model.eval(),list(range(9)),[2.]*9).eval()
    exported=torch.jit.script(wrapper)
    for a,b in zip(wrapper(x),exported(x)): torch.testing.assert_close(a,b)
    report=classification_metrics([0,0,1,1],[0,1,1,1],2)
    assert report['sensitivity']==1 and report['false_positive_rate']==.5
