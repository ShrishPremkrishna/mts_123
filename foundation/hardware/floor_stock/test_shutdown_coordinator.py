import importlib.util
from pathlib import Path
import pytest
spec=importlib.util.spec_from_file_location('coordinator',Path(__file__).with_name('shutdown_coordinator.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def spec_for(**updates):
    result={'deadline_monotonic':1.5,'manager_exit_allowance_s':1.,'launch_exit_allowance_s':1.,
            'manager':{'pid':1},'launch':{'pid':2},'probe_parent':{'pid':3}}
    result.update(updates);return result


class Hooks:
    def __init__(self, *, requested=False, parent_exit=999., manager_delay=.3):
        self.t=0.;self.events=[];self.signals=[];self.requested_now=requested
        self.parent_exit=parent_exit;self.manager_delay=manager_delay
        self.manager_exit=999.;self.launch_exit=999.;self.reused=False
    def now(self):return self.t
    def sleep(self,dt):self.t+=dt
    def requested(self):return self.requested_now
    def state(self,target):
        pid=target['pid']
        if pid==1 and self.reused:return 'reused'
        return 'exited' if self.t>={1:self.manager_exit,2:self.launch_exit,3:self.parent_exit}[pid] else 'running'
    def send(self,target):
        pid=target['pid'];self.signals.append((pid,self.t))
        if pid==1:
            assert len([p for p,t in self.signals if p==1])==1,'duplicate manager SIGINT after default handler restored'
            self.manager_exit=self.t+self.manager_delay
        else:
            assert self.state({'pid':1})!='running','launch forwards SIGINT while manager still destroying'
            self.launch_exit=self.t+.02
        return True
    def event(self,name,**fields):
        row={'event':name,'monotonic':self.t,**fields};self.events.append(row);return row


def test_fixed_deadline_single_signal_then_slow_cleanup_then_launch():
    h=Hooks();result=m.coordinate(spec_for(),h)
    assert result['passed_cleanup'] and h.signals[0]==(1,1.5)
    assert h.signals[1][0]==2 and h.signals[1][1]>=1.8
    assert [e['event'] for e in h.events].index('manager_sigint_sent')< [e['event'] for e in h.events].index('shutdown_triggered')

@pytest.mark.parametrize('early,parent_exit',[(True,999.),(False,.2)])
def test_request_and_probe_death_trigger_early_without_cancellation(early,parent_exit):
    h=Hooks(requested=early,parent_exit=parent_exit);m.coordinate(spec_for(),h)
    assert h.signals[0][1]<1.5


def test_unresponsive_manager_fails_without_launch_forwarding_or_repeat():
    h=Hooks(requested=True,manager_delay=999.);result=m.coordinate(spec_for(),h)
    assert not result['passed_cleanup'] and result['manager_state']=='running'
    assert h.signals==[(1,0.)]


def test_owner_dies_after_ack_fallback_waits_without_resignal():
    h=Hooks();h.t=.1;h.manager_exit=.3
    previous=[{'event':'manager_sigint_sent','monotonic':0.}]
    result=m.coordinate(spec_for(),h,previous,fallback=True)
    assert result['passed_cleanup'] and [p for p,t in h.signals]==[2]


def test_owner_dies_before_ack_fallback_sends_and_discloses_ambiguity():
    h=Hooks();result=m.coordinate(spec_for(),h,[],fallback=True)
    assert result['passed_cleanup'] and h.signals[0][0]==1
    assert result['prior_signal_delivery_ambiguous']


def test_reused_manager_pid_never_signaled():
    h=Hooks(requested=True);h.reused=True
    result=m.coordinate(spec_for(),h)
    assert result['passed_cleanup'] and [p for p,t in h.signals]==[2]
    assert result['manager_state']=='reused'


def test_atomic_writer_validates_before_existing_receipt_is_touched(tmp_path):
    p=tmp_path/'record.json';m.atomic_json(p,{'old':True})
    with pytest.raises(TypeError):m.atomic_json(p,{'bad':object()})
    assert p.read_text().strip()=='{"old": true}'


def test_natural_launch_exit_after_manager_does_not_receive_extra_signal():
    h=Hooks(requested=True)
    send=h.send
    def manager_then_natural_exit(target):
        result=send(target)
        if target['pid']==1:h.launch_exit=h.manager_exit+.2
        return result
    h.send=manager_then_natural_exit
    result=m.coordinate(spec_for(),h)
    assert result['passed_cleanup'] and [pid for pid,t in h.signals]==[1]


@pytest.mark.parametrize('interruption',[KeyboardInterrupt,SystemExit])
def test_interrupted_readiness_keeps_spawned_owner_handle(tmp_path,monkeypatch,interruption):
    from types import SimpleNamespace
    def identity(pid,session,required_arg=None):
        return {'pid':pid,'start_ticks':str(pid),'executable':'/python','required_arg':required_arg}
    monkeypatch.setattr(m,'capture_process',identity)
    monkeypatch.setattr(m,'process_state',lambda *args:'running')
    monkeypatch.setattr(m.subprocess,'Popen',lambda *args,**kw:SimpleNamespace(pid=9999,poll=lambda:None))
    monkeypatch.setattr(m.time,'sleep',lambda *args:(_ for _ in ()).throw(interruption('interrupted readiness')))
    with pytest.raises(m.ArmError) as caught:
        m.arm(tmp_path/'coord',manager=identity(1,'x'),launch=identity(2,'x'),session_id='x')
    assert caught.value.handle['spawned_pid']==9999
    restored=m.check_handle(caught.value.handle)
    assert restored['spawned_identity']['pid']==9999


@pytest.mark.parametrize('failure',[PermissionError('proc disappeared'),ValueError('parent identity changed')])
def test_probe_monitor_failure_triggers_verified_manager_stop_instead_of_killing_owner(failure):
    h=Hooks();state=h.state
    def inspect(target):
        if target['pid']==3:raise failure
        return state(target)
    h.state=inspect
    result=m.coordinate(spec_for(),h)
    assert result['passed_cleanup'] and h.signals[0]==(1,0.)
    assert result['early_shutdown_reason']=='probe_identity_unavailable'
    assert type(failure).__name__ in result['probe_monitor_error']
    assert result['fallback'] is False
    assert not m.nominal_cleanup(result), 'recovery is not readiness for another motion'


@pytest.mark.parametrize('receipt',[
    {'passed_cleanup':True},
    {'passed_cleanup':True,'fallback':True},
    {'passed_cleanup':True,'fallback':False,'prior_signal_delivery_ambiguous':True},
    {'passed_cleanup':True,'fallback':False,'probe_monitor_error':'unavailable'},
    {'passed_cleanup':False,'fallback':False},
])
def test_recovery_or_incomplete_receipt_is_an_explicit_motion_readiness_blocker(receipt):
    assert not m.nominal_cleanup(receipt)
    assert m.nominal_cleanup({'passed_cleanup':True,'fallback':False})


def test_dead_owner_fallback_preserves_cause_and_original_failure_receipt(tmp_path,monkeypatch):
    import json,time
    spec=spec_for(deadline_monotonic=time.monotonic()+1.,session_id='x',protocol=m.PROTOCOL)
    m.atomic_json(tmp_path/'targets.json',spec);handle=m.load_handle(tmp_path)
    m.atomic_json(tmp_path/'owner.json',{'spec_sha256':handle['spec_sha256'],'identity':{'pid':9}})
    (tmp_path/'events.jsonl').write_text(json.dumps({'event':'owner_failed','error':'PermissionError example','monotonic':0.})+'\n')
    h=Hooks(requested=True)
    monkeypatch.setattr(m,'SystemHooks',lambda *args:h)
    monkeypatch.setattr(m,'process_state',lambda *args:'exited')
    result=m.request_and_wait(handle,'test cleanup')
    assert result['passed_cleanup'] and result['fallback']
    assert result['fallback_reason']=='owner_exited'
    assert result['owner_failures'][0]['error']=='PermissionError example'
    assert not m.nominal_cleanup(result)


@pytest.mark.parametrize('failure', [PermissionError('launch metadata denied'), ValueError('launch identity changed')])
def test_launch_inspection_failure_is_bounded_and_never_signals_unknown_identity(failure):
    h=Hooks(requested=True); state=h.state
    def inspect(target):
        if target['pid']==2: raise failure
        return state(target)
    h.state=inspect
    result=m.coordinate(spec_for(),h)
    assert not result['passed_cleanup'] and result['launch_state']=='unverified'
    assert h.signals==[(1,0.)]
    assert h.t==pytest.approx(.3+1.)
    assert result['identity_inspection_errors']['launch']['count']>0
    assert not m.nominal_cleanup(result)


def test_transient_launch_inspection_failure_does_not_kill_owner_or_repeat_manager_signal():
    h=Hooks(requested=True); state=h.state
    def inspect(target):
        if target['pid']==2 and h.t<.4: raise PermissionError('exit transition')
        return state(target)
    h.state=inspect
    result=m.coordinate(spec_for(),h)
    assert result['passed_cleanup'] and result['fallback'] is False
    assert [pid for pid,_ in h.signals]==[1,2]
    assert h.signals[1][1]>=.8 and h.t<=1.3
    assert not m.nominal_cleanup(result), 'Recovered uncertainty is retained for review'


def test_unknown_manager_exit_never_allows_launch_signal():
    h=Hooks(requested=True);state=h.state
    def inspect(target):
        if target['pid']==1 and h.signals: raise PermissionError('manager inspection denied')
        return state(target)
    h.state=inspect
    result=m.coordinate(spec_for(),h)
    assert not result['passed_cleanup'] and result['manager_state']=='unverified'
    assert h.signals==[(1,0.)] and h.t==pytest.approx(1.)
    assert 'original_manager_exited' not in [e['event'] for e in h.events]


def test_initial_unverified_manager_is_never_signaled_or_mistaken_for_exit():
    h=Hooks(requested=True);state=h.state
    def inspect(target):
        if target['pid']==1: raise PermissionError('manager inspection denied')
        return state(target)
    h.state=inspect
    result=m.coordinate(spec_for(),h)
    assert not result['passed_cleanup'] and h.signals==[]
    assert result['manager_state']=='unverified' and h.t==pytest.approx(1.)


def test_launch_signal_identity_race_is_recorded_without_repeated_attempt():
    h=Hooks(requested=True);send=h.send;attempts=[]
    def send_with_race(target):
        if target['pid']==2:
            attempts.append(h.t)
            raise PermissionError('changed before syscall')
        return send(target)
    h.send=send_with_race
    result=m.coordinate(spec_for(),h)
    assert not result['passed_cleanup'] and len(attempts)==1
    assert h.signals==[(1,0.)]
    assert 'PermissionError' in result['signal_errors']['launch']
    assert h.t==pytest.approx(1.3)


@pytest.mark.parametrize('after', ['exited','reused','running'])
def test_proc_permission_race_requires_fresh_terminal_proof(tmp_path,monkeypatch,after):
    proc=tmp_path/'42';proc.mkdir()
    def stat(state='S',start='100'):
        return '42 (python) '+ ' '.join([state]+['0']*18+[start])
    (proc/'stat').write_text(stat())
    target={'pid':42,'start_ticks':'100','executable':'/python'}
    def denied(*args):
        if after=='exited': (proc/'stat').write_text(stat('Z'))
        if after=='reused': (proc/'stat').write_text(stat(start='101'))
        raise PermissionError('metadata unavailable')
    monkeypatch.setattr(m,'capture_process',denied)
    if after=='running':
        with pytest.raises(PermissionError):m.process_state(target,'session',tmp_path)
    else:
        assert m.process_state(target,'session',tmp_path)==after


@pytest.mark.parametrize('field',['identity_inspection_errors','signal_errors'])
def test_inspection_or_signal_recovery_is_not_nominal_readiness(field):
    assert not m.nominal_cleanup({'passed_cleanup':True,'fallback':False,field:{'launch':'denied'}})
