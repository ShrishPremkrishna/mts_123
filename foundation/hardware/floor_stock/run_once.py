"""One explicitly approved homing/floor placement/physical stock stand-stop trial. No retries."""
import argparse,hashlib,importlib.util,json,os,signal,subprocess,sys,time,uuid
from datetime import datetime,timezone
from pathlib import Path
ROOT=Path(__file__).resolve().parent
from motion_gate import require_motion_enabled

ENVIRONMENT_ROOT=Path("/home/pi/pupper-foundation-session-20261029/environment-v5-review")
CANDIDATE_FILES=("run_once.py","console_init_stop_probe.py","shutdown_coordinator.py",
    "probe_recording.py","joy_contract.py","input_preflight.py","motion_gate.py",
    "feedback-only.launch.py","course-config-v4.yaml","baseline-expanded.urdf","wait_feedback.py")
ENVIRONMENT_PINS={
    "overlay-env-v5.sh":"4b9bc797866577539cc6bd5e2d37093bb17ccf3cc335892705c7a7ab3cf607e7",
    "verify-overlay-environment-v5.py":"436b2fa6a49c26b4f1d289da109c8d85ea90f7705820570658465db800c0eff1",
    "environment-v5-pins.json":"cfc5447581cd0d6f47221499bb91ae8143a4bc07d48346a7c5647beef5367929",
}

def input_ready_receipt(path):
    """Only relay a complete ready event from our own input-only helper log."""
    for line in path.read_text().splitlines():
        try:row=json.loads(line)
        except json.JSONDecodeError:continue
        if not isinstance(row,dict) or row.get("event")!="input-only-ready":continue
        if row.get("domain")!=170 or row.get("motor_software") is not False or not isinstance(row.get("instructions"),str):
            raise RuntimeError("input-only ready receipt contract differs")
        return row
    return None

def execute_once(ops):
    """Small lifecycle boundary used by real operations and failure-path tests."""
    try:
        ops.preflight_input()
        ops.start_stack()
        ops.wait_feedback(False,30)
        ops.load_default_inactive()
        prepared=ops.wait_feedback(True,10)
        ops.wait_placement_ready()
        ops.run_frozen_probe(prepared['instance_id'])
    finally:ops.cleanup()

def signal_owned_unreaped_child(child):
    # The only fallback is the original Popen child we just created. waitid with
    # WNOWAIT verifies it is still our child without reaping it; send_signal's
    # own poll then avoids targeting a PID recycled after exit. No process search.
    state=os.waitid(os.P_PID,child.pid,os.WEXITED|os.WNOHANG|os.WNOWAIT)
    if state is not None:child.poll();return False
    child.send_signal(signal.SIGINT);return True

class Operations:
    def __init__(self,candidate,run_dir,session_id,probe,mode='floor-stock-zero',operator_approved=False):
        self.operator_approved=operator_approved
        self.mode=mode;self.candidate=candidate;self.run_dir=run_dir;self.session_id=session_id;self.probe=probe
        self.launch=None;self.launch_identity=None;self.manager=None;self.logs=[];self.children=[]
        self.report={'supervisor_pid':os.getpid(),'session_id':session_id,'supervised_mode':mode,'events':[],'passed':False,'hardware_execution_requested':True}
    def save(self,event,**fields):
        self.report['events'].append({'event':event,'monotonic':time.monotonic(),**fields})
        (self.run_dir/'supervisor.json').write_text(json.dumps(self.report,indent=2)+'\n')
        if event.startswith('awaiting operator floor placement'):
            print(json.dumps({'event':'placement-await','run_dir':str(self.run_dir),'session_id':self.session_id,**fields}),flush=True)
    def bounded(self,command,label,seconds):
        stream=(self.run_dir/(label+'.log')).open('x');self.logs.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT);self.children.append(child)
        self.save(label+' started',command=command,pid=child.pid)
        try:
            deadline=time.monotonic()+seconds;ready_announced=False
            while True:
                remaining=deadline-time.monotonic()
                if remaining<=0:raise subprocess.TimeoutExpired(command,seconds)
                if label=='input-preflight' and not ready_announced:
                    ready=input_ready_receipt(self.run_dir/(label+'.log'))
                    if ready is not None:
                        print(json.dumps({**ready,'run_dir':str(self.run_dir),'session_id':self.session_id}),flush=True);ready_announced=True
                if label=='single-console-probe' and not ready_announced:
                    ready_path=self.run_dir/'physical-input-ready.json'
                    if ready_path.exists():
                        try:ready=json.loads(ready_path.read_text())
                        except json.JSONDecodeError:ready=None
                        if ready is not None:
                            if ready.get('session_id')!=self.session_id or ready.get('ready') is not True:raise RuntimeError('physical ready receipt identity differs')
                            print(json.dumps({'event':'physical-input-ready','run_dir':str(self.run_dir),**ready}),flush=True);ready_announced=True
                try:code=child.wait(timeout=min(.1,remaining));break
                except subprocess.TimeoutExpired:continue
        except subprocess.TimeoutExpired:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=2)
            except subprocess.TimeoutExpired:pass
            raise RuntimeError(label+' exceeded bound; no retry or SIGKILL')
        if code!=0:raise RuntimeError(label+' failed with exit '+str(code))
        self.save(label+' passed',exit_code=code)
    def capture_manager(self):
        if self.launch is None:return None
        children=Path('/proc')/str(self.launch.pid)/'task'/str(self.launch.pid)/'children'
        candidates=[]
        for token in children.read_text().split():
            pid=int(token);proc=Path('/proc')/token
            try:
                executable=str((proc/'exe').resolve(strict=True))
                if executable!='/opt/ros/jazzy/lib/controller_manager/ros2_control_node':continue
                start=self.probe.manager_process_identity(pid,self.session_id,executable)
                candidates.append({'pid':pid,'start_ticks':start,'executable':executable})
            except FileNotFoundError:continue
        if len(candidates)>1:raise RuntimeError('multiple direct manager children')
        return candidates[0] if candidates else None
    def preflight_input(self):
        # Operator approval includes the current root host-wide no-motor check.
        # The child repeats the native host/graph screen before prompting buttons.
        if self.operator_approved is not True:
            raise RuntimeError('explicit supervised input/button and motion approval required')
        report=self.run_dir/'input-preflight.json'
        self.bounded([sys.executable,str(ROOT/'input_preflight.py'),'--run-input-only',
            '--exercise-buttons','--no-motor-stack-confirmed','--seconds','120','--report',str(report)],'input-preflight',130)
        result=json.loads(report.read_text())
        required=('passed','graph_contract_passed','physical_buttons_verified',
                  'button_exercise_requested','no_motor_stack_confirmed','integrity_ok')
        if (any(result.get(k) is not True for k in required)
                or result.get('child_running_after_cleanup') is not False
                or result.get('child_cleanup',{}).get('nominal') is not True
                or result.get('publishers_after_cleanup') != []):
            raise RuntimeError('exact input/button preflight failed; hardware not started')
    def start_stack(self):
        lock=self.probe.check_driver_lock()
        if not lock['available']:raise RuntimeError('driver lock already held')
        stream=(self.run_dir/'stack.log').open('x');self.logs.append(stream)
        command=['ros2','launch',str(self.candidate/'feedback-only.launch.py')]
        self.launch=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.save('stack launch requested; hardware homing begins',command=command,pid=self.launch.pid)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if self.launch.poll() is not None:raise RuntimeError('launch exited during preparation')
            try:
                self.launch_identity=self.probe.process_identity(self.launch.pid,self.candidate/'feedback-only.launch.py',self.session_id)
                self.manager=self.capture_manager()
            except FileNotFoundError:pass
            if self.manager:break
            time.sleep(.05)
        if not self.manager:raise RuntimeError('owned manager child not discovered within5s')
        self.save('owned processes captured',launch_start_ticks=self.launch_identity,manager=self.manager)
    def wait_feedback(self,default_inactive,seconds):
        label='configured' if default_inactive else 'feedback';path=self.run_dir/(label+'.json')
        command=[sys.executable,str(ROOT/'wait_feedback.py'),'--candidate',str(self.candidate),'--report',str(path),'--seconds',str(seconds)]
        if default_inactive:command.append('--default-inactive')
        self.bounded(command,label,seconds+3)
        result=json.loads(path.read_text())
        if not result['passed']:raise RuntimeError(label+' did not pass')
        return result
    def load_default_inactive(self):
        self.bounded(['ros2','run','controller_manager','spawner','neural_controller_default','--inactive',
             '--controller-manager','/controller_manager','--controller-manager-timeout','10','--service-call-timeout','5'], 'inactive-default',15)
    def wait_placement_ready(self):
        path=self.run_dir/'placement-ready.json'
        if path.exists():raise RuntimeError('placement marker must be fresh')
        self.save('awaiting operator floor placement; controllers inactive, homed hardware may hold pose',marker=str(path),timeout_s=60)
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            if self.launch.poll() is not None:raise RuntimeError('stack exited while awaiting placement')
            if path.exists():
                payload=json.loads(path.read_text())
                if payload!={'session_id':self.session_id,'operator_confirmed_ready':True}:raise RuntimeError('placement marker/session differs')
                self.save('operator confirmed floor placement; preparing physical-input window');return
            time.sleep(.05)
        raise RuntimeError('60s placement window expired; no activation retry')
    def run_frozen_probe(self,instance):
        self.bounded([sys.executable,str(self.candidate/'console_init_stop_probe.py'),
            '--mode',self.mode,'--stack-pid',str(self.launch.pid),'--stack-launch',str(self.candidate/'feedback-only.launch.py'),
            '--manager-pid',str(self.manager['pid']),'--instance-id',instance,
            '--session-id',self.session_id,'--report',str(self.run_dir/'probe.json')], 'single-console-probe',40)
        self.report['passed']=json.loads((self.run_dir/'probe.json').read_text()).get('passed_software_sequence',False)
    def cleanup(self):
        results={}
        coordinator_directory=self.run_dir/'shutdown-coordinator-v2'
        coordinated=(coordinator_directory/'targets.json').exists()
        if coordinated:
            # Once armed, the detached coordinator is the sole normal signal
            # owner. A malformed receipt is a failure, not authority to race it.
            try:
                helper=self.probe._shutdown
                handle=helper.load_handle(coordinator_directory);spec=handle['spec']
                if spec['session_id']!=self.session_id:raise RuntimeError('coordinator session mismatch')
                if self.launch is None or self.launch_identity is None or self.manager is None:raise RuntimeError('supervisor target identity unavailable')
                if (spec['launch']['pid']!=self.launch.pid or spec['launch']['start_ticks']!=self.launch_identity
                    or spec['manager']['pid']!=self.manager['pid'] or spec['manager']['start_ticks']!=self.manager['start_ticks']
                    or spec['manager']['executable']!=self.manager['executable']):raise RuntimeError('coordinator targets differ from captured stack')
                if spec['launch'].get('required_arg')!=str((self.candidate/'feedback-only.launch.py').resolve()):raise RuntimeError('coordinator launch command differs')
                results['coordinator']=helper.request_and_wait(handle,'supervisor final cleanup')
                if not helper.nominal_cleanup(results['coordinator']):
                    self.report['passed']=False
                    results['readiness_blocker']='Fallback/ambiguous owner recovery is not a ready-for-motion result.'
            except Exception as error:
                results['coordinator_error']=repr(error);self.report['passed']=False
        else:
            # Popen may succeed before metadata persistence/discovery fails. A live
            # unreaped Popen child is still ours: recover its exact path/UUID/start
            # identity rather than leave a launch able to begin homing afterward.
            if self.launch is not None and self.launch.poll() is None and self.launch_identity is None:
                try:
                    self.launch_identity=self.probe.process_identity(self.launch.pid,self.candidate/'feedback-only.launch.py',self.session_id)
                    results['launch_identity_recovered']=True
                except FileNotFoundError:results['launch_already_exited']=True
                except Exception as error:results['launch_identity_error']=repr(error)
            # Capture a late-created manager before stopping its owned launch.
            if self.launch is not None and self.manager is None:
                try:self.manager=self.capture_manager()
                except Exception as error:results['manager_capture_error']=repr(error)
            if self.manager:
                try:
                    m=self.manager
                    if self.probe.manager_process_identity(m['pid'],self.session_id,m['executable'])!=m['start_ticks']:raise RuntimeError('manager PID reused')
                    os.kill(m['pid'],signal.SIGINT);results['manager_sigint']=True
                except FileNotFoundError:results['manager_already_exited']=True
                except Exception as error:results['manager_signal_error']=repr(error)
            if self.launch is not None and self.launch.poll() is None:
                try:
                    if self.launch_identity is None:
                        results['launch_sigint_via_owned_unreaped_child']=signal_owned_unreaped_child(self.launch)
                    else:
                        if self.probe.process_identity(self.launch.pid,self.candidate/'feedback-only.launch.py',self.session_id)!=self.launch_identity:raise RuntimeError('launch identity changed')
                        os.kill(self.launch.pid,signal.SIGINT);results['launch_sigint']=True
                except FileNotFoundError:results['launch_already_exited']=True
                except Exception as error:results['launch_signal_error']=repr(error)
        for child in self.children:
            if child.poll() is None:
                try:
                    child.send_signal(signal.SIGINT);child.wait(timeout=1)
                except subprocess.TimeoutExpired:results.setdefault('surviving_helper_pids',[]).append(child.pid)
                except Exception as error:results.setdefault('helper_cleanup_errors',[]).append(repr(error))
        deadline=time.monotonic()+(0 if coordinated else 10)
        manager_running=(self.manager is not None and self.probe.original_process_state(self.manager['pid'],self.manager['start_ticks'])=='same_identity_running')
        launch_running=self.launch is not None and self.launch.poll() is None
        while time.monotonic()<deadline:
            if self.launch is not None:self.launch.poll() # reap our child if exited
            manager_running=(self.manager is not None and self.probe.original_process_state(self.manager['pid'],self.manager['start_ticks'])=='same_identity_running')
            launch_running=self.launch is not None and self.launch.poll() is None
            if not manager_running and not launch_running:break
            time.sleep(.05)
        results.update(manager_running=manager_running,launch_running=launch_running,driver_lock=self.probe.check_driver_lock())
        if manager_running or launch_running or not results['driver_lock']['available'] or any('error' in k for k in results) or results.get('surviving_helper_pids'):self.report['passed']=False
        self.save('final owned cleanup',**results)
        for stream in self.logs:stream.close()

def validate_files_and_environment(candidate):
    pins=json.loads((ROOT/'input-pins.json').read_text())
    if set(pins.get('candidate',{}))!=set(CANDIDATE_FILES):
        raise RuntimeError('selected candidate pin allowlist differs')
    if pins.get('environment')!=ENVIRONMENT_PINS:
        raise RuntimeError('reviewed environment pin selection differs')
    for name,digest in pins['candidate'].items():
        if hashlib.sha256((candidate/name).read_bytes()).hexdigest()!=digest:
            raise RuntimeError('selected candidate differs: '+name)
    # These files execute from the supervisor directory even with --candidate.
    for name in ('run_once.py','input_preflight.py','joy_contract.py','probe_recording.py','motion_gate.py','wait_feedback.py'):
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=pins['candidate'][name]:
            raise RuntimeError('running supervisor/helper differs: '+name)
    for name,digest in ENVIRONMENT_PINS.items():
        if hashlib.sha256((ENVIRONMENT_ROOT/name).read_bytes()).hexdigest()!=digest:
            raise RuntimeError('canonical environment input differs: '+name)
    spec=importlib.util.spec_from_file_location('canonical_environment',ENVIRONMENT_ROOT/'verify-overlay-environment-v5.py')
    verifier=importlib.util.module_from_spec(spec);spec.loader.exec_module(verifier)
    from ament_index_python.packages import get_package_prefix,get_package_share_directory
    result=verifier.verify(json.loads((ENVIRONMENT_ROOT/'environment-v5-pins.json').read_text()),
        package_prefix=get_package_prefix,package_share=get_package_share_directory,env=os.environ,
        python_prefix=sys.prefix,python_base_prefix=sys.base_prefix,python_executable=sys.executable)
    if result.get('status')!='PASS':raise RuntimeError('installed environment verifier did not pass')
    return {**result,'candidate_sources':pins['candidate'],'environment_sources':pins['environment']}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--operator-approved',action='store_true');parser.add_argument('--check-only',action='store_true')
    parser.add_argument('--candidate',type=Path,default=ROOT)
    parser.add_argument('--mode',choices=('floor-stock-zero',),default='floor-stock-zero')
    parser.add_argument('--runs-dir',type=Path,default=Path('/home/pi/pupper-foundation-session-20261029/supervised-console-runs'))
    args=parser.parse_args()
    if not args.check_only and not args.operator_approved:parser.error('explicit approval of input/button exercise after root no-motor check, then homing+placement+physical stock floor stand+R3 trial required')
    if not args.check_only:require_motion_enabled()
    if os.environ.get('ROS_DOMAIN_ID')!='168':parser.error('explicit ROS_DOMAIN_ID=168 required')
    candidate=args.candidate.resolve();environment_receipt=validate_files_and_environment(candidate)
    if args.check_only:
        print(json.dumps({'passed':True,'supervised_mode':args.mode,'scope':'Pinned source and installed environment verification; no ROS initialization, Popen, device or lock access','environment':environment_receipt},indent=2));return 0
    spec=importlib.util.spec_from_file_location('frozen_probe',candidate/'console_init_stop_probe.py')
    probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)
    session_id=uuid.uuid4().hex;os.environ['PUPPER_PROBE_SESSION_ID']=session_id
    run_dir=args.runs_dir.resolve()/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+session_id)
    run_dir.mkdir(parents=True,mode=0o700);os.chmod(run_dir,0o700)
    (run_dir/'roslogs').mkdir(mode=0o700)
    os.environ['ROS_LOG_DIR']=str(run_dir/'roslogs')
    ops=Operations(candidate,run_dir,session_id,probe,mode=args.mode,operator_approved=args.operator_approved)
    (run_dir/'environment.json').write_text(json.dumps(environment_receipt,indent=2)+'\n')
    (run_dir/'input-pins.json').write_bytes((ROOT/'input-pins.json').read_bytes())
    ops.save('combined trial accepted; fresh owned session',candidate=str(candidate))
    def interrupted(signum,frame):raise KeyboardInterrupt('supervisor signal '+str(signum))
    for signum in (signal.SIGTERM,signal.SIGHUP):signal.signal(signum,interrupted)
    try:execute_once(ops)
    except BaseException as error:
        ops.report['passed']=False;ops.save('trial failed; no retry',error=repr(error))
    print(json.dumps({'run_dir':str(run_dir),'passed':ops.report['passed'],
        'homing_started':any(e['event']=='stack launch requested; hardware homing begins' for e in ops.report['events']),
        'last_event':ops.report['events'][-1],
        'cleanup':next((e for e in reversed(ops.report['events']) if e['event']=='final owned cleanup'),None)},indent=2))
    return 0 if ops.report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
