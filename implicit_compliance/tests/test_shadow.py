"""Exercise the read-only processor with synthetic streams; no ROS or hardware."""
import json
import numpy as np
import pytest

from implicit_compliance.shadow import ShadowProcessor, json_safe
from implicit_compliance.streaming import MotorWindow


class FakePredictor:
    """Temporal infrastructure fixture, not a trained contact model."""
    def __init__(self):
        self.window=MotorWindow(window=10,sample_hz=1000.,max_gap_s=.025)
        self.last_status='warming_up';self.calls=[]

    def invalidate(self,reason):
        self.window.samples.clear();self.window.last_status=reason;self.last_status=reason

    def push(self,timestamp,values):
        self.calls.append((timestamp,np.array(values)))
        result=self.window.push(timestamp,values)
        self.last_status=self.window.last_status
        if result is None:return None
        self.last_status='predicted'
        return dict(contact=.7,incline=[1.,0.,0.,0.,0.,0.],material=[1.,0.,0.,0.,0.,0.],inference_ms=.2)


def message(t,**updates):
    sec=int(t);nsec=round((t-sec)*1e9)
    args=dict(names=['leg_front_r_3','leg_front_r_1','leg_front_r_2'],
              positions=[3.,1.,2.],velocities=[6.,4.,5.],efforts=[9.,7.,8.],
              header_sec=sec,header_nanosec=nsec,receipt_monotonic_s=t-90,
              receipt_ros_s=t+.002)
    return {**args,**updates}


def test_logs_published_effort_and_distinguishes_receive_rate_from_resampling():
    processor=ShadowProcessor(FakePredictor())
    first=processor.process(**message(100.))
    assert first['status']=='warming_up' and first['prediction'] is None
    result=processor.process(**message(100.01))
    assert result['prediction_available'] and result['window_valid']
    assert result['published_effort']==[9.,7.,8.]
    assert result['channels']==list(range(1,10))
    summary=processor.summary()
    assert summary['received_message_rate_hz']==pytest.approx(100.)
    assert summary['model_resampling_hz']==1000.
    assert summary['header_age_s']['mean']==pytest.approx(.002)
    json.dumps(result,allow_nan=False)
    json.dumps(summary,allow_nan=False)


@pytest.mark.parametrize('changes,expected',[
    ({'efforts':[]},'invalid_channels'),
    ({'efforts':[1.,float('nan'),3.]},'invalid_channels'),
    ({'header_sec':0,'header_nanosec':0},'invalid_timestamp'),
    ({'receipt_ros_s':101.},'stale_header'),
    ({'receipt_ros_s':99.},'future_header'),
])
def test_invalid_input_is_unavailable_and_discards_history(changes,expected):
    predictor=FakePredictor();processor=ShadowProcessor(predictor)
    processor.process(**message(99.99))
    count=len(predictor.calls)
    result=processor.process(**message(100.,**changes))
    assert result['status']==expected
    assert result['prediction'] is None and not result['prediction_available']
    assert len(predictor.calls)==count and not predictor.window.samples
    json.dumps(result,allow_nan=False)


def test_duplicate_and_rewound_header_do_not_reuse_evidence():
    p=ShadowProcessor(FakePredictor())
    p.process(**message(100.))
    duplicate=p.process(**message(100.,receipt_monotonic_s=10.01))
    assert duplicate['status']=='duplicate_timestamp'
    rewind=p.process(**message(99.99,receipt_monotonic_s=10.02))
    assert rewind['status']=='time_rewind'
    assert not p.predictor.window.samples


def test_long_sensor_gap_is_unavailable_and_cannot_create_zero_filled_prediction():
    p=ShadowProcessor(FakePredictor())
    p.process(**message(100.));p.process(**message(100.001))
    result=p.process(**message(100.05))
    assert result['status']=='sensor_gap'
    assert result['prediction'] is None and not result['window_valid']
    assert p.summary()['outcome_counts']['sensor_gap']==1


def test_empty_capture_reports_unavailable_statistics_not_zero_measurements():
    result=ShadowProcessor(FakePredictor()).summary()
    assert result['messages']==0 and result['received_message_rate_hz'] is None
    assert result['channel_statistics']['mean'] is None
    assert json_safe([float('nan'),float('inf'),1.])==[None,None,1.]


def test_cli_is_subscription_only_and_writes_bounded_capture(tmp_path,monkeypatch):
    import sys
    from types import SimpleNamespace
    from implicit_compliance import shadow
    model=tmp_path/'model';model.mkdir()
    (model/'contact.pt').write_bytes(b'infrastructure fixture, not a trained model')
    metadata={'channels':['fixture'],'sample_hz':1000.}
    (model/'metadata.json').write_text(json.dumps(metadata))
    predictor=FakePredictor();predictor.metadata=metadata
    monkeypatch.setattr(shadow,'ContactPredictor',lambda *a,**k:predictor)
    state={'clock':0.,'messages':0,'active':True,'subscriptions':0}
    def monotonic():
        state['clock']+=.01
        return state['clock']
    monkeypatch.setattr(shadow,'time',SimpleNamespace(monotonic=monotonic,perf_counter=monotonic))
    class FakeNode:
        # Deliberately expose no publisher, client, service or controller API.
        def __init__(self,name):self.callback=None
        def create_subscription(self,kind,topic,callback,qos):
            assert topic=='/joint_states'
            self.callback=callback;state['subscriptions']+=1
            return object()
        def get_clock(self):
            return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round((100+state['messages']*.001+.002)*1e9)))
        def destroy_node(self):pass
    def spin_once(node,timeout_sec):
        state['messages']+=1
        sample=message(100+state['messages']*.001)
        node.callback(SimpleNamespace(name=sample['names'],position=sample['positions'],
            velocity=sample['velocities'],effort=sample['efforts'],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sample['header_sec'],nanosec=sample['header_nanosec']))))
    modules={'rclpy':SimpleNamespace(init=lambda:None,ok=lambda:state['active'],spin_once=spin_once,
                                     shutdown=lambda:state.update(active=False)),
             'rclpy.node':SimpleNamespace(Node=FakeNode),'rclpy.qos':SimpleNamespace(qos_profile_sensor_data=object()),
             'sensor_msgs':SimpleNamespace(),'sensor_msgs.msg':SimpleNamespace(JointState=object)}
    for name,module in modules.items():monkeypatch.setitem(sys.modules,name,module)
    out=tmp_path/'capture'
    monkeypatch.setattr(sys,'argv',['shadow','--model',str(model),'--out',str(out),'--seconds','.2'])
    shadow.main()
    manifest=json.loads((out/'manifest.json').read_text());summary=json.loads((out/'summary.json').read_text())
    records=[json.loads(s) for s in (out/'samples.jsonl').read_text().splitlines()]
    assert manifest['status']=='complete' and summary['status']=='complete'
    assert 0<len(records)<20 and len(records)==summary['messages']
    assert state['subscriptions']==1
    assert manifest['model_sha256']==shadow.sha256(model/'contact.pt')
    assert manifest['metadata_sha256']==shadow.sha256(model/'metadata.json')
