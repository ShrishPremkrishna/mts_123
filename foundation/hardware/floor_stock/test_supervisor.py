import importlib.util,json
from pathlib import Path
from types import SimpleNamespace
import pytest
spec=importlib.util.spec_from_file_location('supervisor',Path(__file__).with_name('run_once.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

@pytest.mark.parametrize('fail_at',[None,'input','start','wait-feedback','load','wait-configured','placement','probe'])
def test_one_sequence_cleanup_on_every_phase_failure(fail_at):
    class Fake:
        def __init__(self):self.calls=[]
        def phase(self,name):
            self.calls.append(name)
            if name==fail_at:raise RuntimeError(name)
        def preflight_input(self):self.phase('input')
        def start_stack(self):self.phase('start')
        def wait_feedback(self,inactive,seconds):
            assert seconds==(10 if inactive else 30)
            self.phase('wait-configured' if inactive else 'wait-feedback');return {'instance_id':'x'}
        def load_default_inactive(self):self.phase('load')
        def wait_placement_ready(self):self.phase('placement')
        def run_frozen_probe(self,instance):assert instance=='x';self.phase('probe')
        def cleanup(self):self.calls.append('cleanup')
    f=Fake()
    if fail_at:
        with pytest.raises(RuntimeError):m.execute_once(f)
    else:m.execute_once(f)
    assert f.calls[-1]=='cleanup' and f.calls.count('cleanup')==1
    assert f.calls.count('load')<=1 and f.calls.count('probe')<=1
    if fail_at=='start':assert f.calls==['input','start','cleanup']

@pytest.mark.parametrize('manager_identity',['123','reused'])
def test_actual_cleanup_checks_identity_and_does_not_signal_reaped_launch(tmp_path,monkeypatch,manager_identity):
    calls=[]
    probe=SimpleNamespace(manager_process_identity=lambda *args:manager_identity,
        original_process_state=lambda *args:'absent',check_driver_lock=lambda:{'available':True})
    o=m.Operations(tmp_path,tmp_path,'a'*32,probe)
    o.launch=SimpleNamespace(pid=88888,poll=lambda:0) # already reaped, never signal
    o.launch_identity='old';o.manager={'pid':99999,'start_ticks':'123','executable':'/fake'}
    monkeypatch.setattr(m.os,'kill',lambda *args:calls.append(args));o.cleanup()
    assert all(pid!=88888 for pid,_ in calls)
    if manager_identity=='123':assert [pid for pid,_ in calls]==[99999]
    else:assert not calls and 'manager_signal_error' in o.report['events'][-1]

def test_check_only_has_no_runtime_lifecycle(tmp_path,monkeypatch,capsys):
    monkeypatch.setattr(m,'validate_files_and_environment',lambda path:{'status':'PASS'})
    monkeypatch.setattr(m.os,'environ',{'ROS_DOMAIN_ID':'168'})
    monkeypatch.setattr(m.sys,'argv',['run_once.py','--check-only','--candidate',str(tmp_path)])
    monkeypatch.setattr(m,'Operations',lambda *args:(_ for _ in ()).throw(AssertionError('must not construct runtime')))
    assert m.main()==0 and json.loads(capsys.readouterr().out)['passed']


def test_interruption_still_runs_finally():
    calls=[]
    ops=SimpleNamespace(preflight_input=lambda:None,start_stack=lambda:(_ for _ in ()).throw(KeyboardInterrupt()),cleanup=lambda:calls.append('cleanup'))
    with pytest.raises(KeyboardInterrupt):m.execute_once(ops)
    assert calls==['cleanup']


def test_popen_success_then_metadata_failure_still_stops_owned_launch(tmp_path,monkeypatch):
    signals=[];running=[True]
    launch=SimpleNamespace(pid=88888,poll=lambda:None if running[0] else 0)
    probe=SimpleNamespace(check_driver_lock=lambda:{'available':True},
        process_identity=lambda pid,path,session:'captured-during-cleanup')
    o=m.Operations(tmp_path,tmp_path,'a'*32,probe)
    monkeypatch.setattr(m.subprocess,'Popen',lambda *args,**kw:launch)
    monkeypatch.setattr(o,'save',lambda *args,**kw:(_ for _ in ()).throw(OSError('metadata write failed')))
    monkeypatch.setattr(o,'capture_manager',lambda:None)
    monkeypatch.setattr(o,'preflight_input',lambda:None)
    def kill(pid,sig):signals.append((pid,sig));running[0]=False
    monkeypatch.setattr(m.os,'kill',kill)
    with pytest.raises(OSError,match='metadata write failed'):m.execute_once(o)
    assert o.launch is launch and o.launch_identity=='captured-during-cleanup'
    assert [pid for pid,_ in signals]==[88888] and not running[0]


def test_exec_identity_race_fallback_only_uses_owned_unreaped_child(monkeypatch):
    calls=[];child=SimpleNamespace(pid=88888,poll=lambda:None,send_signal=lambda sig:calls.append(sig))
    monkeypatch.setattr(m.os,'waitid',lambda *args:None,raising=False)
    for name,value in [('P_PID',1),('WEXITED',4),('WNOHANG',1),('WNOWAIT',0x1000000)]:
        if not hasattr(m.os,name):monkeypatch.setattr(m.os,name,value,raising=False)
    assert m.signal_owned_unreaped_child(child) and calls==[m.signal.SIGINT]
    calls.clear();monkeypatch.setattr(m.os,'waitid',lambda *args:SimpleNamespace(si_pid=88888))
    assert not m.signal_owned_unreaped_child(child) and not calls
    monkeypatch.setattr(m.os,'waitid',lambda *args:(_ for _ in ()).throw(ChildProcessError()))
    with pytest.raises(ChildProcessError):m.signal_owned_unreaped_child(child)
    assert not calls


def coordinated_ops(tmp_path, monkeypatch, *, running=False, mismatch=None):
    calls=[];coord=tmp_path/'shutdown-coordinator-v2';coord.mkdir();(coord/'targets.json').write_text('{}')
    spec={'session_id':'a'*32,'manager':{'pid':99999,'start_ticks':'123','executable':'/manager'},
          'launch':{'pid':88888,'start_ticks':'100','required_arg':str((tmp_path/'feedback-only.launch.py').resolve())}}
    if mismatch=='session':spec['session_id']='b'*32
    if mismatch=='manager':spec['manager']['start_ticks']='different'
    if mismatch=='launch':spec['launch']['required_arg']='/different/launch.py'
    helper=SimpleNamespace(nominal_cleanup=lambda r:r['passed_cleanup'] and not r['fallback'],load_handle=lambda path:{'spec':spec},
        request_and_wait=lambda handle,reason:calls.append('coordinator') or {'passed_cleanup':not running,'fallback':False})
    probe=SimpleNamespace(_shutdown=helper,original_process_state=lambda *args:'same_identity_running' if running else 'absent',
        check_driver_lock=lambda:{'available':not running})
    o=m.Operations(tmp_path,tmp_path,'a'*32,probe);o.report['passed']=True
    o.launch=SimpleNamespace(pid=88888,poll=lambda:None if running else 0);o.launch_identity='100'
    o.manager={'pid':99999,'start_ticks':'123','executable':'/manager'}
    monkeypatch.setattr(m.os,'kill',lambda *args:(_ for _ in ()).throw(AssertionError('supervisor must not race coordinator with direct signal')))
    monkeypatch.setattr(m.time,'sleep',lambda *args:(_ for _ in ()).throw(AssertionError('must not add another manager wait')))
    return o,calls


def test_armed_cleanup_only_requests_owner_and_does_not_resignal(tmp_path,monkeypatch):
    o,calls=coordinated_ops(tmp_path,monkeypatch);o.cleanup()
    assert calls==['coordinator'] and o.report['passed']


@pytest.mark.parametrize('mismatch',['session','manager','launch'])
def test_mismatched_coordinator_not_adopted_or_raced(tmp_path,monkeypatch,mismatch):
    o,calls=coordinated_ops(tmp_path,monkeypatch,mismatch=mismatch);o.cleanup()
    assert not calls and not o.report['passed']
    assert 'coordinator_error' in o.report['events'][-1]


def test_unresponsive_manager_keeps_failure_without_repeated_signal_or_wait(tmp_path,monkeypatch):
    o,calls=coordinated_ops(tmp_path,monkeypatch,running=True);o.cleanup()
    assert calls==['coordinator'] and not o.report['passed']
    assert o.report['events'][-1]['manager_running']



@pytest.mark.parametrize('mode',['floor-stock-zero'])
def test_explicit_mode_reaches_exact_one_probe_invocation(tmp_path,monkeypatch,mode):
    ops=m.Operations(tmp_path,tmp_path,'a'*32,SimpleNamespace(),mode=mode)
    ops.launch=SimpleNamespace(pid=111);ops.manager={'pid':222};calls=[]
    (tmp_path/'probe.json').write_text('{"passed_software_sequence":false}')
    monkeypatch.setattr(ops,'bounded',lambda command,label,seconds:calls.append((command,label,seconds)))
    ops.run_frozen_probe('b'*32)
    assert len(calls)==1
    command,label,seconds=calls[0]
    assert command[command.index('--mode')+1]==mode and label=='single-console-probe' and seconds==40
    assert ops.report['supervised_mode']==mode



def test_placement_marker_session_gate_and_bounded_timeout(tmp_path,monkeypatch):
 clock=[0.];monkeypatch.setattr(m.time,'monotonic',lambda:clock[0]);monkeypatch.setattr(m.time,'sleep',lambda s:clock.__setitem__(0,clock[0]+s))
 o=m.Operations(tmp_path,tmp_path,'a'*32,SimpleNamespace());o.launch=SimpleNamespace(poll=lambda:None)
 with pytest.raises(RuntimeError,match='60s'):o.wait_placement_ready()
 assert clock[0]>=60
 for payload,passed in [({'session_id':'b'*32,'operator_confirmed_ready':True},False),({'session_id':'a'*32,'operator_confirmed_ready':True},True)]:
  (tmp_path/'placement-ready.json').unlink(missing_ok=True);clock[0]=0.
  def step(s):
   clock[0]+=s;(tmp_path/'placement-ready.json').write_text(json.dumps(payload))
  monkeypatch.setattr(m.time,'sleep',step)
  if passed:o.wait_placement_ready()
  else:
   with pytest.raises(RuntimeError,match='differs'):o.wait_placement_ready()
  (tmp_path/'placement-ready.json').unlink()
 (tmp_path/'placement-ready.json').write_text('{}')
 with pytest.raises(RuntimeError,match='fresh'):o.wait_placement_ready()

def test_physical_ready_is_forwarded_once_with_exact_session(tmp_path,monkeypatch,capsys):
    o=m.Operations(tmp_path,tmp_path,'a'*32,SimpleNamespace());calls=[]
    def wait(timeout):
        calls.append(timeout)
        if len(calls)==1:
            (tmp_path/'physical-input-ready.json').write_text(json.dumps({'session_id':'a'*32,'ready':True,'deadline_monotonic':123.}))
            raise m.subprocess.TimeoutExpired(['fake'],timeout)
        return 0
    child=SimpleNamespace(pid=99999,wait=wait)
    monkeypatch.setattr(m.subprocess,'Popen',lambda *args,**kw:child)
    o.bounded(['fake'],'single-console-probe',2.)
    rows=[json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert len(rows)==1 and rows[0]['event']=='physical-input-ready' and rows[0]['session_id']=='a'*32
    assert len(calls)==2 and max(calls)<=.1
    for stream in o.logs:stream.close()


def input_success():
    return dict(passed=True,graph_contract_passed=True,physical_buttons_verified=True,
                button_exercise_requested=True,no_motor_stack_confirmed=True,integrity_ok=True,
                child_running_after_cleanup=False,child_cleanup={'nominal':True},publishers_after_cleanup=[])


def test_input_exercise_requires_current_operator_approval(tmp_path,monkeypatch):
    o=m.Operations(tmp_path,tmp_path,'a'*32,SimpleNamespace())
    monkeypatch.setattr(o,'bounded',lambda *args:pytest.fail('must not start input child'))
    with pytest.raises(RuntimeError,match='explicit supervised'):o.preflight_input()


@pytest.mark.parametrize('failure',[None,'passed','graph_contract_passed','physical_buttons_verified',
    'button_exercise_requested','no_motor_stack_confirmed','integrity_ok','survivor','escalation','publisher'])
def test_input_exercise_checks_physical_result_and_owned_cleanup(tmp_path,monkeypatch,failure):
    o=m.Operations(tmp_path,tmp_path,'a'*32,SimpleNamespace(),operator_approved=True)
    report=input_success();calls=[]
    if failure in report:report[failure]=False
    if failure=='survivor':report['child_running_after_cleanup']=True
    if failure=='escalation':report['child_cleanup']['nominal']=False
    if failure=='publisher':report['publishers_after_cleanup']=[{'name':'leftover'}]
    def bounded(command,label,seconds):
        calls.append((command,label,seconds));(tmp_path/'input-preflight.json').write_text(json.dumps(report))
    monkeypatch.setattr(o,'bounded',bounded)
    if failure:
        with pytest.raises(RuntimeError,match='hardware not started'):o.preflight_input()
    else:o.preflight_input()
    command,label,seconds=calls[0]
    assert '--exercise-buttons' in command and '--no-motor-stack-confirmed' in command
    assert label=='input-preflight' and seconds==130 and len(calls)==1


def test_input_ready_log_waits_for_complete_json_and_rejects_changed_contract(tmp_path):
    p=tmp_path/'input.log';p.write_text('a native warning\n{"event":"input-only-ready"')
    assert m.input_ready_receipt(p) is None
    ready={'event':'input-only-ready','domain':170,'motor_software':False,'instructions':'Options then R3'}
    p.write_text('a native warning\n'+json.dumps(ready)+'\n')
    assert m.input_ready_receipt(p)==ready
    ready['motor_software']=True;p.write_text(json.dumps(ready))
    with pytest.raises(RuntimeError,match='contract differs'):m.input_ready_receipt(p)


def test_input_ready_is_forwarded_once_before_child_completion(tmp_path,monkeypatch,capsys):
    o=m.Operations(tmp_path,tmp_path,'a'*32,SimpleNamespace());calls=[]
    def wait(timeout):
        calls.append(timeout)
        if len(calls)==1:
            with (tmp_path/'input-preflight.log').open('a') as stream:
                stream.write(json.dumps({'event':'input-only-ready','domain':170,'motor_software':False,'instructions':'Options then R3'})+'\n')
        if len(calls)<3:raise m.subprocess.TimeoutExpired(['fake'],timeout)
        return 0
    child=SimpleNamespace(pid=99999,wait=wait)
    monkeypatch.setattr(m.subprocess,'Popen',lambda *args,**kw:child)
    o.bounded(['fake'],'input-preflight',2.)
    rows=[json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert len(rows)==1 and rows[0]['event']=='input-only-ready' and rows[0]['session_id']=='a'*32
    assert rows[0]['motor_software'] is False and len(calls)==3
    for stream in o.logs:stream.close()


def environment_fixture(tmp_path,monkeypatch):
    import sys,types,hashlib
    root=tmp_path/'candidate';root.mkdir();env=tmp_path/'environment';env.mkdir()
    for name in m.CANDIDATE_FILES:(root/name).write_text('selected '+name)
    (env/'overlay-env-v5.sh').write_text('pinned wrapper')
    (env/'environment-v5-pins.json').write_text('{"schema":"test-pins"}')
    (env/'verify-overlay-environment-v5.py').write_text('''def verify(pins, **kwargs):
    assert pins == {"schema":"test-pins"}
    assert kwargs['package_prefix']('neural_controller') == '/expected/prefix'
    assert kwargs['package_share']('neural_controller') == '/expected/share'
    return {'status':'PASS','python_executable':kwargs['python_executable']}
''')
    pins={'candidate':{n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in m.CANDIDATE_FILES},
          'environment':{n:hashlib.sha256((env/n).read_bytes()).hexdigest() for n in m.ENVIRONMENT_PINS}}
    (root/'input-pins.json').write_text(json.dumps(pins))
    packages=types.ModuleType('ament_index_python.packages')
    packages.get_package_prefix=lambda name:'/expected/prefix'
    packages.get_package_share_directory=lambda name:'/expected/share'
    monkeypatch.setitem(sys.modules,'ament_index_python',types.ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules,'ament_index_python.packages',packages)
    monkeypatch.setattr(m,'ROOT',root);monkeypatch.setattr(m,'ENVIRONMENT_ROOT',env)
    monkeypatch.setattr(m,'ENVIRONMENT_PINS',dict(pins['environment']))
    return root,env,pins


def test_environment_validation_runs_pinned_verifier_and_retains_inputs(tmp_path,monkeypatch):
    root,env,pins=environment_fixture(tmp_path,monkeypatch)
    result=m.validate_files_and_environment(root)
    assert result['status']=='PASS' and result['python_executable']==m.sys.executable
    assert result['candidate_sources']==pins['candidate']
    assert result['environment_sources']==pins['environment']


@pytest.mark.parametrize('change',['candidate','environment','omitted_candidate','unreviewed_environment','helper'])
def test_changed_input_or_environment_refused_before_verifier(tmp_path,monkeypatch,change):
    import shutil
    root,env,pins=environment_fixture(tmp_path,monkeypatch);candidate=root
    if change=='candidate':(root/'course-config-v4.yaml').write_text('changed')
    elif change=='environment':(env/'verify-overlay-environment-v5.py').write_text('raise AssertionError("must not execute")')
    elif change=='omitted_candidate':del pins['candidate']['joy_contract.py']
    elif change=='unreviewed_environment':pins['environment']['environment-v5-pins.json']='0'*64
    elif change=='helper':
        candidate=tmp_path/'other';shutil.copytree(root,candidate)
        (root/'input_preflight.py').write_text('changed running helper')
    (root/'input-pins.json').write_text(json.dumps(pins))
    with pytest.raises(RuntimeError):m.validate_files_and_environment(candidate)
