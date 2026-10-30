import ast,importlib.util,hashlib,json
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('floor_probe',ROOT/'console_init_stop_probe.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)

def state(epoch=1,latched=True,**extra):
 d=dict(epoch=epoch,latched=latched,fault=False,unknown_remote_submission=False,shutting_down=False,stop_required=False,in_flight=False);d.update(extra);return d

def inv(active=False):return {**{n:{'state':'active','type':t} for n,t in m.BROADCASTERS.items()},m.DEFAULT:{'state':'active' if active else 'inactive','type':'neural_controller/NeuralController'}}
def buttons(*pressed):return [int(i in pressed) for i in range(13)]
def enabled():
 p=m.Probe(0.);p.enable(.1,1);p.joy(.2,buttons());return p

def tick(p,t,active=False,st=None,fault=None):
 p.joy(t,buttons());p.tick(t,state() if st is None else st,inv(active),t,status_at=t,fault=fault)

def running():
 p=enabled();p.joy(.3,buttons(9));tick(p,.4,True,state(latched=False));return p

def test_actual_options_then_r3_requires_fresh_latch_and_dwell():
 p=running();tick(p,5.,True,state(latched=False));p.joy(5.1,buttons(12));p.tick(5.1,state(latched=False),inv(True),5.1,status_at=5.1)
 tick(p,5.2,False,state(2));tick(p,5.69,False,state(2));assert not p.done
 tick(p,5.71,False,state(2));assert p.done and p.result()['passed_software_sequence']
 assert p.result()['physical_r3_gateway_verified'] and not p.result()['floor_support_proven_by_software']

def test_r3_before_fade_is_verified_stop_not_completed_stand():
 p=running();p.joy(.6,buttons(12));tick(p,.7,False,state(2));tick(p,1.21,False,state(2))
 assert p.done and p.result()['physical_r3_gateway_verified'] and not p.result()['passed_software_sequence']

def test_stop_wins_simultaneous_options_without_new_activation():
 p=enabled();p.joy(.3,buttons(9,12));assert p.stop_time==.3 and p.start_time is None
 tick(p,.4,False,state(2));tick(p,.91,False,state(2));assert not p.result()['passed_software_sequence']

@pytest.mark.parametrize('case',['no_neutral','second_options','face','missing_input','stale_input','unobserved_activation','unknown_actor','fault','deactivation'])
def test_faults_do_not_publish_or_retry(case):
 p=enabled()
 if case=='no_neutral':p=m.Probe(0);p.enable(.1,1);p.joy(.2,buttons(9))
 elif case=='second_options':p=running();p.joy(.5,buttons(9))
 elif case=='face':p.joy(.3,buttons(0))
 elif case=='missing_input':p=m.Probe(0);p.enable(.1,1);p.tick(3.2,state(),inv(),3.2,status_at=3.2)
 elif case=='stale_input':p.tick(.5,state(),inv(),.5,status_at=.5)
 elif case=='unobserved_activation':tick(p,.3,True,state(latched=False))
 elif case=='unknown_actor':v=inv();v['other']={'state':'active','type':'neural_controller/NeuralController'};p.tick(.3,state(),v,.3,status_at=.3)
 elif case=='fault':tick(p,.3,fault='invalid IMU')
 else:p=running();tick(p,.5,False,state(latched=False))
 assert p.done and p.errors and not p.result()['passed_software_sequence']

def test_inactive_dwell_resets_on_stale_status_and_reactivation_fails():
 p=running();p.joy(5.,buttons(12));tick(p,5.1,False,state(2));p.joy(5.3,buttons());p.tick(5.3,None,inv(),5.3,status_at=None)
 assert p.quiet_since is None
 tick(p,5.4,True,state(2));assert p.done and 'reactivation' in p.errors[-1]

@pytest.mark.parametrize('mode,cap',[('partial-init',1.5),('full-stock-zero',6.),('floor-stock-zero',20.)])
def test_explicit_caps_cannot_be_extended(tmp_path,monkeypatch,mode,cap):
 h=m._shutdown;seen=[];target={'pid':5,'start_ticks':'1','executable':'/fake'}
 monkeypatch.setattr(h.time,'monotonic',lambda:100.);monkeypatch.setattr(h,'capture_process',lambda *a:target);monkeypatch.setattr(h,'process_state',lambda *a:'running')
 class End(Exception):pass
 def write(path,value):seen.append(value);raise End()
 monkeypatch.setattr(h,'atomic_json',write)
 with pytest.raises(End):h.arm(tmp_path/mode,manager=target,launch=target,session_id='a'*32,mode=mode)
 assert seen[0]['deadline_monotonic']==100.+cap
 with pytest.raises(ValueError):h.arm(tmp_path/(mode+'bad'),manager=target,launch=target,session_id='a'*32,mode=mode,deadline_monotonic=101.+cap)

def test_actual_main_has_no_joy_publisher_no_teleop_and_arm_precedes_owned_joy():
 tree=ast.parse((ROOT/'console_init_stop_probe.py').read_text());main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
 pubs=[n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_publisher']
 assert len(pubs)==1 and pubs[0].args[0].id=='Twist' and pubs[0].args[1].value=='/cmd_vel'
 arm=next(n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='arm_shutdown')
 spawn=next(n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='Popen')
 assert arm.lineno<spawn.lineno
 assert 'joy_contract.command(joy_executable)' in ast.unparse(main)
 assert 'teleop' not in ast.unparse(main)


def test_physical_phase_requires_policy_and_targets_after_fade():
 d={'supervised_mode':'floor-stock-zero','first_active_receipt_monotonic':1.,'physical_stop_monotonic':5.6,'passed_software_sequence':True}
 rows=[{'monotonic':5.+i*.02,'valid':True,'values':[0]*12} for i in range(26)]
 raw={'policy_output':rows,'position_command':rows};m.phase_evidence(d,raw);assert d['passed_software_sequence']
 raw['policy_output']=[];m.phase_evidence(d,raw);assert not d['passed_software_sequence']


def test_operator_ready_receipt_rechecks_actual_fresh_physical_and_authority_state():
 p=enabled();s=state();v=inv();sensor={'joint_states':.2,'imu':.2}
 assert m.physical_input_ready(.3,p,s,.2,v,.2,sensor)
 assert not m.physical_input_ready(.5,p,s,.5,v,.5,{'joint_states':.5,'imu':.5}) # stale physical Joy
 assert not m.physical_input_ready(.3,p,s,.2,v,-.3,sensor) # stale controller inventory
 assert not m.physical_input_ready(.3,p,state(latched=False),.2,v,.2,sensor)
 p.joy(.3,buttons(9));assert not m.physical_input_ready(.3,p,s,.3,v,.3,sensor)
