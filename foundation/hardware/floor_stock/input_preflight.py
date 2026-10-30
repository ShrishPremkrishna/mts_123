#!/usr/bin/env python3
"""Exact native joystick preflight in isolated domain170; never loads motor software.

Default invocation only describes the contract. --run-input-only starts the one
installed joy executable and subscribes to its actual messages. No synthetic Joy,
controller manager, launch, hardware plugin or motion service is created.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
from datetime import datetime,timezone
import subprocess
import time

import joy_contract as contract
from probe_recording import json_value

ROOT=Path(__file__).resolve().parent
DOMAIN='170'


class InputEvidence:
    def __init__(self):
        self.previous=[0]*13;self.button_neutral=False;self.pressed=set();self.released=set()
        self.last_received=None;self.samples=[];self.errors=[]
    def receive(self,now,buttons,axes,gid,diagnostic):
        row={'monotonic':now,'buttons':list(buttons),'axes':list(axes),'publisher_gid':gid}
        qualified=contract.source_matches(diagnostic,gid,allow_graph_only=gid is None)
        row['qualified_exact_source']=qualified and gid is not None
        row['qualified_single_endpoint']=qualified
        row['source_validation']='single_endpoint_graph' if gid is None else 'message_gid'
        self.samples.append(row)
        if not contract.valid_joy(list(buttons),list(axes)):
            self.errors.append('invalid Joy shape/value');return
        if not qualified:
            if self.last_received is not None:self.errors.append('unexpected Joy message source after qualification')
            return
        self.last_received=now
        if not any(buttons):self.button_neutral=True
        if self.button_neutral:
            for index in (9,12):
                if buttons[index] and not self.previous[index]:self.pressed.add(index)
                if not buttons[index] and self.previous[index] and index in self.pressed:self.released.add(index)
        self.previous=list(buttons)
    def complete(self,now):
        return (not self.errors and self.button_neutral and self.released=={9,12}
                and self.last_received is not None and 0<=now-self.last_received<=.25
                and not any(self.previous))


def stop_owned_child(child,timeout=3.):
    """Only the direct input-only Joy Popen child; never manager/launch processes.

    Popen.poll/send_signal preserve unreaped-child identity. Escalation is
    bounded and recorded, and prevents a nominal preflight pass.
    """
    attempts=[]
    for signum,allowance in ((signal.SIGINT,timeout),(signal.SIGTERM,1.),(signal.SIGKILL,1.)):
        if child.poll() is not None:break
        child.send_signal(signum)
        attempts.append({'signal':signum.name,'monotonic':time.monotonic(),'wait_allowance_s':allowance})
        try:child.wait(timeout=allowance)
        except subprocess.TimeoutExpired:pass
    running=child.poll() is None;escalated=len(attempts)>1
    return {'attempts':attempts,'exit_code':child.poll(),'running':running,'escalated':escalated,
            'nominal':not running and not escalated and child.poll()==0}


def known_hardware_processes(proc_root=Path('/proc')):
    """Small stdlib host-wide known-process screen; no argv, devices or kills.

    Linux comm names can be truncated to15bytes, hence the specific prefixes.
    This is not proof of all hardware absence or of physical motor power state.
    """
    found=[];errors=[]
    try:entries=list(Path(proc_root).iterdir())
    except OSError as error:
        return {'owners':[],'inspection_errors':[{'error':type(error).__name__}]}
    for entry in entries:
        if not entry.name.isdigit():continue
        try:name=(entry/'comm').read_text().strip()
        except FileNotFoundError:continue # exited during the read-only snapshot
        except OSError as error:
            errors.append({'pid':int(entry.name),'error':type(error).__name__});continue
        if (name.startswith(('ros2_control','driver_init','driver_homing','control_board','pupper_hardware'))
                or name in ('spidev_test','spi_runner')):
            found.append({'pid':int(entry.name),'name':name})
    return {'owners':found,'inspection_errors':errors}


def source_hashes():
    return {n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ('joy_contract.py','input_preflight.py','probe_recording.py')}


def describe():
    return {'schema':'pupper_exact_input_preflight_v1','scope':'Input-only; default description does not initialize ROS or start processes',
            'domain':int(DOMAIN),'node':contract.NODE_NAME,'namespace':contract.NAMESPACE,
            'topic':contract.TOPIC,'message_type':contract.MESSAGE_TYPE,'parameters':contract.PARAMETERS,
            'subscriber_qos':'RELIABLE VOLATILE depth10, identical to the floor consumer',
            'optional_physical_sequence':'Release buttons, press/release Options(index9), press/release R3(index12), release buttons; trigger/stick centering is not required',
            'external_domain_limit':'ROS domain isolation does not isolate the physical gamepad from another running motor stack. Button exercise requires current root confirmation plus a read-only known-process check.',
            'motion_enabled':False,'source_sha256':source_hashes()}


def unsafe_graph(node):
    nodes=[name for name,namespace in node.get_node_names_and_namespaces()
           if name in ('controller_manager','estop_controller')]
    services=[name for name,types in node.get_service_names_and_types()
              if name.startswith('/controller_manager/') or any('controller_manager_msgs/' in t for t in types)]
    return {'motor_nodes':nodes,'motor_services':services}


def run_native(args):
    # This changes only this standalone child process and its one Joy child.
    # It cannot change the eventual supervisor/robot domain.
    os.environ['ROS_DOMAIN_ID']=DOMAIN;os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Joy
    from rcl_interfaces.srv import GetParameters
    from ament_index_python.packages import get_package_prefix
    report=describe();report.update(button_exercise_requested=args.exercise_buttons,no_motor_stack_confirmed=args.no_motor_stack_confirmed,started_utc=datetime.now(timezone.utc).isoformat(),platform={key:getattr(os.uname(),key) for key in ('sysname','release','version','machine')},python=sys.version,passed=False,scope='Actual native ROS and physical input only; no motor services or launch',samples=[],graphs=[])
    args.report.parent.mkdir(parents=True,exist_ok=True)
    if args.report.exists():raise RuntimeError('report already exists; preserve previous evidence')
    context=Context();node=executor=child=log=None;body_ok=False;evidence=InputEvidence()
    diagnostic={'ready':False,'reason':'not_discovered'};graph_ok=False;host_clear=False;bound_gid=None
    try:
        rclpy.init(context=context);node=rclpy.create_node('floor_input_preflight',context=context)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        def receive(msg,info):
            evidence.receive(time.monotonic(),list(msg.buttons),list(msg.axes),contract.message_gid(info),diagnostic)
        node.create_subscription(Joy,contract.TOPIC,receive,contract.SUBSCRIBER_DEPTH)
        # Bounded discovery before opening the gamepad; still no publishers here.
        until=time.monotonic()+.75
        while time.monotonic()<until:executor.spin_once(timeout_sec=.02)
        unsafe=unsafe_graph(node);report['initial_graph_guard']=unsafe
        host=known_hardware_processes();report['initial_host_process_screen']=host
        if host['owners']:raise RuntimeError('known hardware process exists on host; isolated domain is insufficient')
        host_clear=not host['inspection_errors']
        if args.exercise_buttons and not host_clear:raise RuntimeError('host process inspection incomplete; physical button exercise refused')
        if unsafe['motor_nodes'] or unsafe['motor_services']:raise RuntimeError('motor software found in isolated input domain')
        initial=[contract.endpoint(i) for i in node.get_publishers_info_by_topic(contract.TOPIC)]
        report['existing_publishers']=initial
        if initial:raise RuntimeError('existing Joy publisher; input preflight will not reuse or kill it')
        executable=Path(get_package_prefix('joy'))/'lib/joy/joy_node'
        report['executable']=contract.executable_identity(executable);report['command']=contract.command(executable)
        log=args.report.with_suffix('.joy.log').open('x')
        child=subprocess.Popen(report['command'],stdout=log,stderr=subprocess.STDOUT)
        report['child_pid']=child.pid
        client=node.create_client(GetParameters,'/'+contract.NODE_NAME+'/get_parameters')
        pending=None;parameters=None;parameter_error=None;sent=None;next_graph=0.;announced=False
        deadline=time.monotonic()+args.seconds;next_host_check=0.
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02);now=time.monotonic()
            if child.poll() is not None:raise RuntimeError('owned Joy child exited before preflight completed')
            if now>=next_host_check:
                host=known_hardware_processes();next_host_check=now+.5
                host_clear=not host['owners'] and not host['inspection_errors']
                if args.exercise_buttons and host['inspection_errors']:raise RuntimeError('host process inspection became incomplete; button exercise aborted')
                if host['owners']:
                    report['host_hardware_processes_appeared']=host['owners'];host_clear=False
                    raise RuntimeError('known hardware process appeared across ROS domains')
            if now>=next_graph:
                rows=[contract.endpoint(i) for i in node.get_publishers_info_by_topic(contract.TOPIC)]
                diagnostic=contract.graph_diagnostic(rows,bound_gid)
                if diagnostic['ready'] and bound_gid is None:bound_gid=diagnostic['gid']
                report['graphs'].append({'monotonic':now,**diagnostic});next_graph=now+.1
                unsafe=unsafe_graph(node)
                if unsafe['motor_nodes'] or unsafe['motor_services']:raise RuntimeError('motor software appeared in input-only domain')
                graph_error=contract.graph_fault(diagnostic,bound_gid)
                if graph_error:raise RuntimeError(graph_error)
            if pending is None and parameters is None and client.service_is_ready():
                request=GetParameters.Request();request.names=list(contract.PARAMETERS);pending=client.call_async(request);sent=now
            if pending is not None:
                if pending.done():
                    response=pending.result();pending=None
                    if response is None or len(response.values)!=len(contract.PARAMETERS):raise RuntimeError('Joy parameters unavailable')
                    parameters={n:(v.string_value if v.type==4 else v.double_value if v.type==3 else v.integer_value if v.type==2 else None)
                                for n,v in zip(contract.PARAMETERS,response.values)}
                    report['runtime_parameters']=parameters
                    if parameters!=contract.PARAMETERS:parameter_error='Joy runtime parameters do not match exact requested contract'
                elif now-sent>contract.STARTUP_TIMEOUT_S:raise RuntimeError('Joy parameter response timeout during bounded input startup')
            if evidence.errors:raise RuntimeError(evidence.errors[-1])
            graph_ok=(diagnostic['ready'] and parameters==contract.PARAMETERS and evidence.last_received is not None
                      and 0<=now-evidence.last_received<=.25)
            if graph_ok and not args.exercise_buttons:
                break
            if graph_ok and args.exercise_buttons and args.no_motor_stack_confirmed and host_clear and not announced:
                print(json.dumps({'event':'input-only-ready','domain':170,'motor_software':False,
                    'instructions':'Root confirmed no competing motor stack. Release buttons, tap/release Options, tap/release R3, then release buttons. Trigger-axis centering is not required.'}),flush=True)
                announced=True
            if args.exercise_buttons and args.no_motor_stack_confirmed and host_clear and evidence.complete(now) and graph_ok:
                body_ok=True;break
        report['last_graph']=diagnostic
        if parameter_error:report['parameter_error']=parameter_error
        if args.exercise_buttons and not body_ok:report['error']='bounded physical-input exercise incomplete; exact graph/parameters, host precondition and both button cycles must pass'
        elif not graph_ok:report['error']='bounded exact graph/device preflight incomplete'
    except BaseException as error:report['error']=repr(error)
    finally:
        if child is not None:
            try:
                receipt=stop_owned_child(child);report['child_cleanup']=receipt
                report['child_running_after_cleanup']=receipt['running']
            except BaseException as error:
                report['child_cleanup_error']=repr(error);report['child_running_after_cleanup']=True
        else:report['child_running_after_cleanup']=False
        if node is not None and executor is not None:
            try:
                until=time.monotonic()+1.
                remaining=[contract.endpoint(i) for i in node.get_publishers_info_by_topic(contract.TOPIC)]
                while remaining and time.monotonic()<until:
                    executor.spin_once(timeout_sec=.02)
                    remaining=[contract.endpoint(i) for i in node.get_publishers_info_by_topic(contract.TOPIC)]
                report['publishers_after_cleanup']=remaining
            except BaseException as error:report['graph_cleanup_error']=repr(error)
        for cleanup in (lambda:executor.shutdown(timeout_sec=.5) if executor else None,
                        lambda:node.destroy_node() if node else None,
                        lambda:context.shutdown() if context.ok() else None):
            try:cleanup()
            except BaseException as error:report.setdefault('cleanup_errors',[]).append(repr(error))
        if log:log.close()
        report['samples']=evidence.samples
        report['input_errors']=list(evidence.errors)
        report['message_origin_verified']=bool(evidence.samples and all(row['publisher_gid'] is not None for row in evidence.samples))
        report['source_validation']='message_gid' if report['message_origin_verified'] else 'single_endpoint_graph'
        report['source_validation_limit']='Installed Jazzy metadata does not identify each message publisher; graph mode relies on the owned joy process and one pinned endpoint, and cannot authenticate individual message origin.'
        report['physical_button_cycles']={'button_neutral_seen':evidence.button_neutral,'pressed':sorted(evidence.pressed),'released':sorted(evidence.released)}
        cleanup_nominal=report.get('child_cleanup',{}).get('nominal') is True
        report['graph_contract_passed']=bool(graph_ok and cleanup_nominal and not report.get('publishers_after_cleanup') and not report.get('error'))
        report['physical_buttons_verified']=bool(body_ok and host_clear)
        report['passed']=bool(body_ok and cleanup_nominal and not report['child_running_after_cleanup'] and not report.get('publishers_after_cleanup')
                              and not any(k.endswith('error') or k=='cleanup_errors' for k in report))
        if not args.exercise_buttons:report['readiness_blocker']='Graph/device diagnostic only; physical Options/R3 exercise has not been verified.'
        if report.get('child_cleanup',{}).get('escalated'):report['readiness_blocker']='Owned Joy required signal escalation; investigate before another trial.'
        report['source_sha256_after']=source_hashes()
        if report['source_sha256_after']!=report['source_sha256']:report['source_changed']=True
        integrity_ok=not any(bool(value) for key,value in report.items()
            if key=='error' or key.endswith('_error') or key in ('cleanup_errors','input_errors','source_changed'))
        report['integrity_ok']=integrity_ok
        report['graph_contract_passed']=bool(report['graph_contract_passed'] and integrity_ok)
        report['passed']=bool(report['passed'] and integrity_ok)
        # Final formatting is outside the input interval; no motor timing exists.
        temporary=args.report.with_suffix('.tmp');temporary.write_text(json.dumps(json_value(report),indent=2,allow_nan=False)+'\n');temporary.replace(args.report)
    return 0 if (report['passed'] if args.exercise_buttons else report['graph_contract_passed']) else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-input-only',action='store_true')
    parser.add_argument('--report',type=Path)
    parser.add_argument('--exercise-buttons',action='store_true',help='Separately opt in to physical Options/R3 prompts; never enabled for graph-only diagnostics')
    parser.add_argument('--no-motor-stack-confirmed',action='store_true',help='Root has just confirmed no competing hardware stack across all ROS domains')
    parser.add_argument('--seconds',type=float,default=30.)
    args=parser.parse_args()
    if not args.run_input_only:
        print(json.dumps(describe(),indent=2));return 0
    if args.exercise_buttons and not args.no_motor_stack_confirmed:parser.error('--exercise-buttons requires a current explicit --no-motor-stack-confirmed confirmation')
    if args.report is None:parser.error('--report is required for native input-only execution')
    maximum=120 if args.exercise_buttons else 30
    if not 3<=args.seconds<=maximum:parser.error('input interval exceeds bounded graph/button allowance')
    return run_native(args)

if __name__=='__main__':raise SystemExit(main())
