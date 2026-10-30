import json
from pathlib import Path
from types import SimpleNamespace
import sys
import pytest
import input_preflight as p
import joy_contract as c


def endpoint(**changes):
    row={'name':c.NODE_NAME,'namespace':'/','topic_type':c.MESSAGE_TYPE,'gid':'ab'*16,
         'reliability':'RELIABLE','durability':'VOLATILE','history':'KEEP_LAST','depth':10}
    row.update(changes);return row


@pytest.mark.parametrize('rows,reason',[
    ([], 'no_publisher_discovered'),([endpoint(),endpoint(gid='cd'*16)],'multiple_publishers'),
    ([endpoint(name='joy_node')],'unexpected_node_name'),([endpoint(namespace='/other')],'unexpected_namespace'),
    ([endpoint(topic_type='std_msgs/msg/String')],'unexpected_message_type'),
    ([endpoint(gid='00'*16)],'endpoint_identity_unavailable'),
    ([endpoint(reliability='BEST_EFFORT')],'reliability_not_proven_compatible'),
    ([endpoint(reliability='UNKNOWN')],'reliability_not_proven_compatible'),
    ([endpoint(durability='UNKNOWN')],'durability_unknown'),
])
def test_graph_refusal_keeps_actual_distinct_evidence(rows,reason):
    actual=c.graph_diagnostic(rows)
    assert not actual['ready'] and actual['reason']==reason and actual['publishers']==rows


def test_exact_and_compatible_transient_local_graphs_pass_without_name_normalization():
    for durability in ('VOLATILE','TRANSIENT_LOCAL'):
        assert c.graph_diagnostic([endpoint(durability=durability)])['ready']
    assert not c.graph_diagnostic([endpoint(namespace='')])['ready']


def test_both_consumers_use_the_identical_command_namespace_device_and_subscriber():
    import ast
    tree=ast.parse((Path(__file__).parent/'console_init_stop_probe.py').read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    assert 'joy_contract.command(joy_executable)' in ast.unparse(main)
    assert 'joy_contract.SUBSCRIBER_DEPTH' in ast.unparse(main)
    command=c.command('/prefix/lib/joy/joy_node')
    assert command[0]=='/prefix/lib/joy/joy_node' and '__ns:=/' in command
    assert '__node:='+c.NODE_NAME in command and 'device_name:='+c.DEVICE_NAME in command
    assert 'coalesce_interval_ms:=20' in command
    assert c.PARAMETERS['coalesce_interval_ms']==20 and 'coalesce_interval' not in c.PARAMETERS


def native_fakes(tmp_path,monkeypatch,*,graph_change=None,parameter_bad=False,preexisting=False,motor=False,stuck=False,interrupt=False,exercise=True,host_errors=False,graph_cleanup_error=False,context_error=False,source_change=False,malformed=False):
    monkeypatch.setenv('ROS_DOMAIN_ID','0');monkeypatch.setenv('ROS_LOCALHOST_ONLY','0')
    monkeypatch.setattr(p,'known_hardware_processes',lambda:{'owners':[],'inspection_errors':[{'error':'PermissionError'}] if host_errors else []})
    if source_change:
        hashes=iter([{'source':'before'},{'source':'after'}]);monkeypatch.setattr(p,'source_hashes',lambda:next(hashes))
    clock=[0.];alive=[False];calls=[];callbacks=[];raised=[False]
    monkeypatch.setattr(p.time,'monotonic',lambda:clock[0])
    class Context:
        def ok(self):return True
        def shutdown(self):
            calls.append('context_shutdown')
            if context_error:raise RuntimeError('injected context cleanup')
    class Executor:
        def __init__(self,context):pass
        def add_node(self,node):pass
        def shutdown(self,timeout_sec):calls.append('executor_shutdown')
        def spin_once(self,timeout_sec):
            clock[0]+=.02
            if alive[0]:
                if interrupt and not raised[0]:raised[0]=True;raise KeyboardInterrupt()
                buttons=[0]*13
                if 1.0<=clock[0]<1.06:buttons[9]=1
                if 1.2<=clock[0]<1.26:buttons[12]=1
                for callback in callbacks:callback(SimpleNamespace(buttons=buttons,axes=[] if malformed and clock[0]>=.84 else [0.]*8),SimpleNamespace(publisher_gid=bytes.fromhex('ab'*16)))
    class Node:
        def create_subscription(self,typ,topic,callback,qos):
            assert topic==c.TOPIC and qos==c.SUBSCRIBER_DEPTH;callbacks.append(callback)
        def create_client(self,typ,name):
            assert name=='/'+c.NODE_NAME+'/get_parameters'
            values=[SimpleNamespace(type=4,string_value=c.DEVICE_NAME),SimpleNamespace(type=3,double_value=50.),SimpleNamespace(type=2,integer_value=20)]
            if parameter_bad:values[-1]=SimpleNamespace(type=0)
            return SimpleNamespace(service_is_ready=lambda:True,call_async=lambda request:SimpleNamespace(done=lambda:(not malformed or clock[0]>=.84),result=lambda:SimpleNamespace(values=values)))
        def get_publishers_info_by_topic(self,topic):
            if graph_cleanup_error and not alive[0] and any(isinstance(x,tuple) and x[0]=='signal' for x in calls):raise RuntimeError('injected graph cleanup')
            if not alive[0] and not preexisting:return []
            row=endpoint(**(graph_change or {}))
            return [SimpleNamespace(node_name=row['name'],node_namespace=row['namespace'],topic_type=row['topic_type'],endpoint_gid=bytes.fromhex(row['gid']),qos_profile=SimpleNamespace(reliability=row['reliability'],durability=row['durability'],history=row['history'],depth=row['depth']))]
        def get_node_names_and_namespaces(self):return [('controller_manager','/')] if motor else [('floor_input_preflight','/')]
        def get_service_names_and_types(self):return []
        def destroy_node(self):calls.append('node_destroyed')
    class GetParameters:
        class Request:pass
    modules={'rclpy':SimpleNamespace(init=lambda **kw:calls.append('rclpy_init'),create_node=lambda *a,**kw:Node()),
        'rclpy.context':SimpleNamespace(Context=Context),'rclpy.executors':SimpleNamespace(SingleThreadedExecutor=Executor),
        'sensor_msgs.msg':SimpleNamespace(Joy=object),'rcl_interfaces.srv':SimpleNamespace(GetParameters=GetParameters),
        'ament_index_python.packages':SimpleNamespace(get_package_prefix=lambda name:str(tmp_path))}
    for name,module in modules.items():monkeypatch.setitem(sys.modules,name,module)
    executable=tmp_path/'lib/joy/joy_node';executable.parent.mkdir(parents=True);executable.write_bytes(b'fake reviewed executable')
    class Child:
        pid=12345
        def poll(self):return None if alive[0] else 0
        def send_signal(self,signum):
            calls.append(('signal',signum))
            if not stuck:alive[0]=False
        def wait(self,timeout):
            if stuck:raise p.subprocess.TimeoutExpired('fake',timeout)
            return 0
    def popen(command,**kw):calls.append(('Popen',command));alive[0]=True;return Child()
    monkeypatch.setattr(p.subprocess,'Popen',popen)
    args=SimpleNamespace(report=tmp_path/'report.json',seconds=3.,exercise_buttons=exercise,no_motor_stack_confirmed=exercise)
    code=p.run_native(args)
    return code,json.loads(args.report.read_text()),calls


def test_actual_native_entry_wires_exact_input_and_owned_cleanup_without_motor_paths(tmp_path,monkeypatch):
    code,result,calls=native_fakes(tmp_path,monkeypatch)
    assert code==0 and result['passed'] and result['physical_button_cycles']['released']==[9,12]
    assert result['child_running_after_cleanup'] is False and result['publishers_after_cleanup']==[]
    assert [x for x in calls if isinstance(x,tuple) and x[0]=='Popen']==[('Popen',c.command(tmp_path/'lib/joy/joy_node'))]
    assert len([x for x in calls if isinstance(x,tuple) and x[0]=='signal'])==1


@pytest.mark.parametrize('kwargs',[
    {'graph_change':{'namespace':'/other'}},{'graph_change':{'reliability':'BEST_EFFORT'}},
    {'parameter_bad':True},{'stuck':True},{'interrupt':True},
])
def test_native_entry_failure_still_stops_only_its_child_and_never_passes(tmp_path,monkeypatch,kwargs):
    code,result,calls=native_fakes(tmp_path,monkeypatch,**kwargs)
    assert code==1 and not result['passed']
    assert len([x for x in calls if isinstance(x,tuple) and x[0]=='Popen'])==1
    assert len([x for x in calls if isinstance(x,tuple) and x[0]=='signal'])==(3 if kwargs.get('stuck') else 1)
    if kwargs.get('stuck'):assert result['child_running_after_cleanup'] is True


@pytest.mark.parametrize('kwargs',[{'preexisting':True},{'motor':True}])
def test_existing_input_or_motor_graph_refuses_before_any_child(tmp_path,monkeypatch,kwargs):
    code,result,calls=native_fakes(tmp_path,monkeypatch,**kwargs)
    assert code==1 and not result['passed']
    assert not any(isinstance(x,tuple) and x[0] in ('Popen','signal') for x in calls)


def test_default_input_command_and_canonical_motion_are_unavailable_without_execution_flag(monkeypatch,capsys):
    import motion_gate,run_once
    monkeypatch.setattr(motion_gate,'MOTION_EXECUTION_ENABLED',False)
    monkeypatch.setattr(sys,'argv',['input_preflight.py'])
    monkeypatch.setattr(p,'run_native',lambda *a:pytest.fail('default may not initialize ROS'))
    assert p.main()==0 and json.loads(capsys.readouterr().out)['motion_enabled'] is False
    monkeypatch.setattr(run_once.sys,'argv',['run_once.py','--operator-approved'])
    monkeypatch.setattr(run_once.subprocess,'Popen',lambda *a,**k:pytest.fail('motion gate precedes any process'))
    with pytest.raises(RuntimeError,match='disabled'):run_once.main()
    assert motion_gate.MOTION_EXECUTION_ENABLED is False


def test_expected_eight_axis_shape_and_noncentered_trigger_axes():
    assert c.valid_joy([0]*13,[0.,0.,1.,0.,0.,1.,0.,0.])
    assert not c.valid_joy([0]*13,[])
    assert not c.valid_joy([0]*13,[0.]*6)
    assert not c.valid_joy([0]*13,[float('nan')]*8)


def test_known_host_process_screen_is_cross_domain_and_does_not_read_argv(tmp_path):
    for pid,name in [(1,'ros2_control_no'),(2,'driver_init_onl'),(3,'hailort_service'),(4,'pupper-rs')]:
        directory=tmp_path/str(pid);directory.mkdir();(directory/'comm').write_text(name+'\n')
    result=p.known_hardware_processes(tmp_path)
    assert result['owners']==[{'pid':1,'name':'ros2_control_no'},{'pid':2,'name':'driver_init_onl'}]
    assert not result['inspection_errors']
    assert p.known_hardware_processes(tmp_path/'missing')['inspection_errors']


def test_button_exercise_requires_explicit_cross_domain_host_confirmation(monkeypatch):
    monkeypatch.setattr(sys,'argv',['input_preflight.py','--run-input-only','--exercise-buttons','--report','unused.json'])
    monkeypatch.setattr(p,'run_native',lambda *a:pytest.fail('must reject before native initialization'))
    with pytest.raises(SystemExit) as caught:p.main()
    assert caught.value.code==2


def test_owned_input_child_escalation_is_bounded_and_never_nominal(monkeypatch):
    calls=[];alive=[True]
    class Child:
        def poll(self):return None if alive[0] else -9
        def send_signal(self,sig):
            calls.append(sig)
            if sig==p.signal.SIGKILL:alive[0]=False
        def wait(self,timeout):
            assert timeout in (3.,1.)
            if alive[0]:raise p.subprocess.TimeoutExpired('owned joy',timeout)
            return -9
    result=p.stop_owned_child(Child())
    assert calls==[p.signal.SIGINT,p.signal.SIGTERM,p.signal.SIGKILL]
    assert not result['running'] and result['escalated'] and not result['nominal']
    assert [r['wait_allowance_s'] for r in result['attempts']]==[3.,1.,1.]


def test_default_native_diagnostic_has_no_physical_button_prompt_or_readiness_claim(tmp_path,monkeypatch,capsys):
    code,result,calls=native_fakes(tmp_path,monkeypatch,exercise=False)
    assert code==0 and result['graph_contract_passed'] and not result['passed']
    assert not result['physical_buttons_verified'] and 'readiness_blocker' in result
    assert 'input-only-ready' not in capsys.readouterr().out
    assert any(row['qualified_exact_source'] for row in result['samples'])


def test_incomplete_host_inspection_blocks_button_exercise_but_allows_passive_graph(tmp_path,monkeypatch,capsys):
    first=tmp_path/'graph';first.mkdir()
    code,result,calls=native_fakes(first,monkeypatch,exercise=False,host_errors=True)
    assert code==0 and result['graph_contract_passed'] and not result['passed']
    assert 'input-only-ready' not in capsys.readouterr().out
    second=tmp_path/'buttons';second.mkdir()
    code,result,calls=native_fakes(second,monkeypatch,exercise=True,host_errors=True)
    assert code==1 and not result['passed']
    assert not any(isinstance(c,tuple) and c[0]=='Popen' for c in calls)


@pytest.mark.parametrize('kwargs',[
    {'malformed':True},{'graph_cleanup_error':True},{'context_error':True},{'source_change':True},
])
def test_graph_only_never_returns_success_after_input_cleanup_or_source_error(tmp_path,monkeypatch,kwargs):
    code,result,calls=native_fakes(tmp_path,monkeypatch,exercise=False,**kwargs)
    assert code==1 and not result['graph_contract_passed'] and not result['passed']
    assert result['integrity_ok'] is False
    if kwargs.get('malformed'):assert result['input_errors']==['invalid Joy shape/value']
    if kwargs.get('source_change'):assert result['source_changed']


def test_message_source_and_endpoint_are_bound_after_initial_discovery():
    first=c.graph_diagnostic([endpoint()]);assert first['ready']
    assert not c.graph_diagnostic([endpoint(gid='cd'*16)],first['gid'])['ready']
    assert c.source_matches(first,'ab'*16) and not c.source_matches(first,'cd'*16)
    evidence=p.InputEvidence()
    evidence.receive(1.,[0]*13,[0.]*8,'cd'*16,first)
    assert evidence.last_received is None and not evidence.errors
    evidence.receive(1.1,[0]*13,[0.]*8,'ab'*16,first)
    evidence.receive(1.2,[0]*13,[0.]*8,'cd'*16,first)
    assert evidence.errors==['unexpected Joy message source after qualification']


def test_actual_floor_callback_ignores_unqualified_initial_input_then_aborts_changed_source():
    import ast
    tree=ast.parse((Path(__file__).parent/'console_init_stop_probe.py').read_text())
    callback=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='physical_joy')
    events=[];probe=SimpleNamespace(neutral_seen=False,joy=lambda *a:events.append('accepted'),fail=lambda *a:events.append('abort'))
    namespace={'time':SimpleNamespace(monotonic=lambda:1.),'joy_contract':c,'joy_graph':c.graph_diagnostic([endpoint()]),'probe':probe,'record':lambda *a:events.append('record')}
    exec(compile(ast.Module(body=[callback],type_ignores=[]),'<actual floor callback>','exec'),namespace)
    msg=SimpleNamespace(buttons=[0]*13,axes=[0.]*8,header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0)))
    receive=namespace['physical_joy'];receive(msg,SimpleNamespace(publisher_gid=bytes.fromhex('cd'*16)))
    assert events==['record']
    events.clear();receive(msg,SimpleNamespace(publisher_gid=bytes.fromhex('ab'*16)));assert events==['accepted','record']
    probe.neutral_seen=True;events.clear();receive(msg,SimpleNamespace(publisher_gid=bytes.fromhex('cd'*16)))
    assert events==['abort','record']


def test_native_mapping_message_info_and_object_adapter_bind_same_gid():
    value=bytes.fromhex('ab'*16)
    assert c.message_gid({'publisher_gid':value})==c.message_gid(SimpleNamespace(publisher_gid=value))=='ab'*16
    with pytest.raises(KeyError):c.message_gid({})


def test_unknown_discovery_waits_only_before_first_qualified_endpoint():
    unknown=c.graph_diagnostic([endpoint(name='_NODE_NAME_UNKNOWN_',namespace='_NODE_NAMESPACE_UNKNOWN_')])
    assert c.graph_fault(unknown) is None
    assert c.graph_fault(unknown,'ab'*16)
    assert c.graph_fault(c.graph_diagnostic([endpoint(name='other')]))
    assert c.graph_fault(c.graph_diagnostic([endpoint(),endpoint()]))
    assert c.graph_fault(c.graph_diagnostic([]),'ab'*16)
    assert c.graph_fault(c.graph_diagnostic([endpoint()])) is None


def test_jazzy_metadata_uses_explicit_graph_contract_without_origin_claim():
    assert c.message_gid({'source_timestamp':1,'received_timestamp':2}) is None
    diagnostic=c.graph_diagnostic([endpoint()])
    assert not c.source_matches(diagnostic,None)
    assert c.source_matches(diagnostic,None,allow_graph_only=True)
    assert not c.source_matches(c.graph_diagnostic([endpoint(gid='cd'*16)],'ab'*16),None,allow_graph_only=True)
    evidence=p.InputEvidence();evidence.receive(1.,[0]*13,[0.]*8,None,diagnostic)
    assert evidence.last_received==1. and not evidence.samples[0]['qualified_exact_source']
    assert evidence.samples[0]['source_validation']=='single_endpoint_graph'
    with pytest.raises(ValueError):c.message_gid({'publisher_gid':bytes(16)})
