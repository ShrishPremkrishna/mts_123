#!/usr/bin/env python3
"""Explicit supervised stock floor stand and physical gamepad stop test.

Only zero velocity is published by this probe. One real joy_node is the sole Joy
publisher; Options and R3 are operator inputs. No API or teleop node is used. Root must load the pinned default
inactive and obtain the operator's concrete test approval before invoking this.
After arming, one detached coordinator owns manager/launch teardown; no physical-stop claim.
"""
import argparse
import hashlib
import fcntl
import math
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import joy_contract
from motion_gate import require_motion_enabled

# Local evidence-only module; works when this probe is imported by the supervisor.
_recording_spec=importlib.util.spec_from_file_location('pupper_probe_recording',Path(__file__).with_name('probe_recording.py'))
_recording=importlib.util.module_from_spec(_recording_spec);_recording_spec.loader.exec_module(_recording)
Recorder=_recording.Recorder
_shutdown_spec=importlib.util.spec_from_file_location('pupper_probe_shutdown',Path(__file__).with_name('shutdown_coordinator.py'))
_shutdown=importlib.util.module_from_spec(_shutdown_spec);_shutdown_spec.loader.exec_module(_shutdown)

DEFAULT = 'neural_controller_default'
JOINTS = [f'leg_{leg}_{axis}' for leg in ('front_r','front_l','back_r','back_l') for axis in (1,2,3)]
BROADCASTERS = {'joint_state_broadcaster':'joint_state_broadcaster/JointStateBroadcaster',
                'imu_sensor_broadcaster':'imu_sensor_broadcaster/IMUSensorBroadcaster'}

def valid_joints(names, position, velocity):
    return (len(names)==12 and set(names)==set(JOINTS) and len(position)==12 and len(velocity)==12
            and all(math.isfinite(x) for x in list(position)+list(velocity)))

def imu_quality(q, angular, acceleration):
    norm=math.sqrt(sum(x*x for x in q));tilt=None
    valid=all(math.isfinite(x) for x in q+angular+acceleration) and .95<=norm<=1.05
    if valid:
        tilt=math.acos(max(-1.,min(1.,1.-2.*(q[0]**2+q[1]**2)/(norm*norm))))
        valid=tilt<=1.0 # same bound as the unchanged default controller
    return valid,norm,tilt

def release_evidence_ready(now,status,status_at,inventory,inventory_at,sensor_at):
    return (status is not None and status_at is not None and 0<=now-status_at<=.5
        and inventory is not None and inventory_at is not None and 0<=now-inventory_at<=.5
        and all(t is not None and 0<=now-t<=.2 for t in sensor_at.values())
        and status['latched'] and not any(status[k] for k in ('fault','unknown_remote_submission','shutting_down','stop_required','in_flight'))
        and inventory.get(DEFAULT)=={'type':'neural_controller/NeuralController','state':'inactive'}
        and all(inventory.get(n)=={'type':t,'state':'active'} for n,t in BROADCASTERS.items())
        and all(v['state']!='active' or n in BROADCASTERS for n,v in inventory.items()))

def physical_input_ready(now,probe,status,status_at,inventory,inventory_at,sensor_at):
    return (probe.phase=='await_options' and probe.neutral_seen and probe.last_joy is not None
        and 0<=now-probe.last_joy<=.25 and probe.start_time is None
        and release_evidence_ready(now,status,status_at,inventory,inventory_at,sensor_at))

class Probe:
    """Passive physical-button state machine. Never publishes Joy or activates a controller."""
    def __init__(self,now):
        self.started=now;self.phase='preflight';self.events=[];self.errors=[];self.done=False
        self.start_time=None;self.stop_time=None;self.active_time=None;self.active_seen=False
        self.initial_epoch=-1;self.neutral_seen=False;self.previous=[0]*13;self.last_joy=None
        self.inactive_after_stop=False;self.quiet_since=None;self.ever_inactive_after_stop=False
    def event(self,name,now):self.events.append({'event':name,'elapsed_s':now-self.started})
    def fail(self,now,reason):
        if reason not in self.errors:self.errors.append(reason)
        self.done=True;self.event('abort: '+reason,now)
    def enable(self,now,epoch):
        self.phase='await_options';self.enabled_at=now;self.initial_epoch=epoch;self.event('physical input enabled after placement',now)
    def joy(self,now,buttons):
        if len(buttons)!=13 or any(x not in (0,1) for x in buttons):self.fail(now,'malformed physical Joy');return
        self.last_joy=now
        if self.phase=='preflight':self.fail(now,'Joy before physical input enable');return
        if any(buttons[i] for i in (0,1,2,3)):self.fail(now,'face buttons outside stock test scope');return
        if not any(buttons):self.neutral_seen=True
        if buttons[12] and not self.previous[12]:
            if self.stop_time is None:self.stop_time=now;self.phase='stopping';self.event('physical R3 Stop received',now)
        elif buttons[9] and not self.previous[9]:
            if self.start_time is not None or self.stop_time is not None:self.fail(now,'second or post-Stop Options refused')
            elif not self.neutral_seen:self.fail(now,'Options arrived before observed neutral gamepad')
            else:self.start_time=now;self.phase='released';self.event('physical Options received',now)
        self.previous=list(buttons)
    def tick(self,now,status,inventory,inventory_at,fault=None,status_at=None):
        if self.done:return
        if fault:self.fail(now,fault);return
        if self.phase=='preflight':
            if now-self.started>5:self.fail(now,'preflight deadline')
            return
        fresh=inventory is not None and inventory_at is not None and 0<=now-inventory_at<=.5
        if not fresh:self.fail(now,'inventory stale during physical test');return
        if (self.last_joy is None and now-self.enabled_at>3.) or (self.last_joy is not None and now-self.last_joy>.25):self.fail(now,'physical gamepad missing/stale');return
        if any(v['state']=='active' and n not in BROADCASTERS and n!=DEFAULT for n,v in inventory.items()):self.fail(now,'unexpected motion controller active');return
        active=inventory.get(DEFAULT,{}).get('state')=='active'
        if active:
            if self.start_time is None:self.fail(now,'activation without observed physical Options');return
            if not self.active_seen:self.active_time=now;self.active_seen=True;self.event('stock active observed',now)
            if self.ever_inactive_after_stop:self.fail(now,'reactivation after physical Stop');return
        elif self.active_seen and self.stop_time is None:self.fail(now,'unexpected deactivation before physical R3');return
        if self.start_time is not None and not self.active_seen and now-self.start_time>.75:self.fail(now,'physical Options did not activate stock within.75s');return
        if self.stop_time is not None:
            valid=status is not None and status_at is not None and status_at>=self.stop_time and now-status_at<=.5
            if (valid and inventory_at>=self.stop_time and status['epoch']>self.initial_epoch and status['latched']
                    and not any(status[k] for k in ('fault','unknown_remote_submission','shutting_down','stop_required','in_flight'))
                    and inventory.get(DEFAULT,{}).get('state')=='inactive'):
                self.ever_inactive_after_stop=True
                if self.quiet_since is None:self.quiet_since=now;self.event('fresh physical-stop inactive/latch dwell starts',now)
                if now-self.quiet_since>=.5:self.inactive_after_stop=True;self.done=True
            else:self.quiet_since=None
            if now-self.stop_time>2:self.fail(now,'physical R3 deactivation confirmation deadline')
    def result(self):
        duration=self.stop_time-self.active_time if self.stop_time is not None and self.active_time is not None else None
        passed=self.active_seen and self.inactive_after_stop and duration is not None and duration>=4.5 and not self.errors
        return {'passed_software_sequence':passed,'supervised_mode':'floor-stock-zero','physical_gamepad_tested':self.start_time is not None and self.stop_time is not None,'physical_r3_gateway_verified':self.active_seen and self.stop_time is not None and self.inactive_after_stop,
                'physical_stopping_proven':False,'floor_support_proven_by_software':False,'activation_observed':self.active_seen,
                'physical_options_monotonic':self.start_time,'physical_stop_monotonic':self.stop_time,
                'first_active_receipt_monotonic':self.active_time,'active_receipt_to_stop_s':duration,
                'inactive_after_stop_observed':self.inactive_after_stop,'errors':self.errors,'events':self.events}

def process_identity(pid, expected, session_id, proc_root=Path('/proc')):
    if pid<=1 or pid==os.getpid():raise ValueError('invalid stack launch PID')
    proc=proc_root/str(pid)
    args=(proc/'cmdline').read_bytes().split(b'\0')
    if str(expected.resolve()).encode() not in args:
        raise ValueError('PID is not the exact supplied minimal launch command')
    if ('PUPPER_PROBE_SESSION_ID='+session_id).encode() not in (proc/'environ').read_bytes().split(b'\0'):
        raise ValueError('launch does not have the exact owned probe session identity')
    stat=(proc/'stat').read_text();start=stat[stat.rfind(')')+2:].split()[19]
    return start

LIBRARIES = {
 '/home/pi/pupper-foundation-session-20261029/driver-overlay-v2/install/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so': '925954b1b0c773bedb483da77bcad5c08aa707b0baac9ca16db50772df14e4e2',
 '/home/pi/pupper-foundation-session-20261029/gateway-overlay-v4/install/neural_controller/lib/libneural_controller.so': '52ab3c9c48e6e980bd935699b21ac5de09076d1aee9f8d3552146be16a24ed91'}
def manager_identity(pid, stack_pid, session_id, proc_root=Path('/proc'), libraries=LIBRARIES, expected_executable=Path('/opt/ros/jazzy/lib/controller_manager/ros2_control_node')):
    if pid<=1 or pid in (stack_pid,os.getpid()):raise ValueError('invalid owned manager PID')
    proc=proc_root/str(pid);args=(proc/'cmdline').read_bytes().split(b'\0')
    if b'/opt/ros/jazzy/lib/controller_manager/ros2_control_node' not in args:raise ValueError('not the expected manager executable')
    stat=(proc/'stat').read_text();fields=stat[stat.rfind(')')+2:].split()
    if int(fields[1])!=stack_pid:raise ValueError('manager is not a direct child of owned launch')
    if ('PUPPER_PROBE_SESSION_ID='+session_id).encode() not in (proc/'environ').read_bytes().split(b'\0'):
        raise ValueError('manager does not share owned session identity')
    executable=str((proc/'exe').resolve(strict=True))
    if executable!=str(expected_executable.resolve(strict=True)):raise ValueError('manager actual executable differs')
    maps=(proc/'maps').read_text();result={'pid':pid,'parent_pid':stack_pid,'start_ticks':fields[19],'executable':executable,'mapped_files':{}}
    for filename,expected in libraries.items():
        path=Path(filename);st=path.stat();digest=hashlib.sha256(path.read_bytes()).hexdigest()
        rows=[line.split(None,5) for line in maps.splitlines() if len(line.split(None,5))==6 and line.split(None,5)[5]==filename]
        if not rows or any(int(row[4])!=st.st_ino or tuple(int(x,16) for x in row[3].split(':'))!=(os.major(st.st_dev),os.minor(st.st_dev)) for row in rows):
            raise ValueError('library mapping/inode differs: '+filename)
        if digest!=expected:raise ValueError('mapped library on-disk hash differs: '+filename)
        result['mapped_files'][filename]={'sha256':digest,'inode':st.st_ino,'device':st.st_dev}
    result['scope']='Exact process ownership and mapped-file inode/on-disk hash; not an in-memory byte hash'
    return result

def original_process_state(pid, start_ticks, proc_root=Path('/proc')):
    try:
        stat=(proc_root/str(pid)/'stat').read_text();fields=stat[stat.rfind(')')+2:].split()
    except FileNotFoundError:return 'absent'
    if fields[19]!=start_ticks:return 'pid_reused_original_gone'
    return 'zombie_exited' if fields[0]=='Z' else 'same_identity_running'

def check_driver_lock(path=Path('/tmp/control_board_hardware.lock')):
    created=False;fd=None
    try:
        try:fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_RDWR,0o600);created=True
        except FileExistsError:fd=os.open(path,os.O_RDONLY)
        identity=os.fstat(fd)
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        result={'available':True,'checked_inode':identity.st_ino,'created_probe_file':created}
        if created:
            current=path.stat()
            if current.st_dev!=identity.st_dev or current.st_ino!=identity.st_ino:
                raise RuntimeError('lock path inode changed during probe')
            path.unlink() # only this probe-created inode, still held exclusively
        fcntl.flock(fd,fcntl.LOCK_UN)
        return result
    except Exception as error:return {'available':False,'error':repr(error),'created_probe_file':created}
    finally:
        if fd is not None:os.close(fd)

def publication_result(data, raw):
    starts=[r for r in raw.get('commands',[]) if r.get('topic')=='/joy' and r['buttons'][9]==1]
    if len(starts)!=1:
        data['passed_software_sequence']=False;data['partial_initialization_only_expected']=False
        data['publication_interval_error']='exactly one recorded Start required';return
    start=starts[0]
    stops=[r for r in raw['commands'] if r.get('topic')=='/joy' and r['buttons'][12]==1 and r['publish_begin_s']>=start['publish_end_s']]
    if not stops:
        data['passed_software_sequence']=False;data['partial_initialization_only_expected']=False
        data['publication_interval_error']='no recorded Stop following Start';return
    stop=stops[0]
    duration=stop['publish_end_s']-start['publish_begin_s']
    data['publication_start_to_stop_upper_s']=duration
    data['publication_start_end_s']=start['publish_end_s'];data['publication_stop_end_s']=stop['publish_end_s']
    data['partial_initialization_only_expected']=0<=duration<2.0
    if data.get('supervised_mode','partial-init')=='partial-init' and not data['partial_initialization_only_expected']:data['passed_software_sequence']=False

def phase_evidence(data, raw):
    """Receipt evidence plus pinned controller timing; not actuator tracking proof."""
    if data.get('supervised_mode')!='floor-stock-zero':return
    active=data.get('first_active_receipt_monotonic');stop=data.get('physical_stop_monotonic')
    report={'scope':'Finite 12-value policy and final-target receipts in the fully-faded window; receipt times, not controller timestamps or physical tracking','streams':{}}
    for name in ('policy_output','position_command'):
        rows=[r for r in raw.get(name,[]) if active is not None and stop is not None and active+4.0<=r['monotonic']<=stop and r.get('valid')]
        span=rows[-1]['monotonic']-rows[0]['monotonic'] if rows else 0.
        report['streams'][name]={'samples':len(rows),'observed_span_s':span,'passed':len(rows)>=2 and span>=.4}
    report['passed']=all(v['passed'] for v in report['streams'].values())
    data['full_stock_phase_evidence']=report
    if not report['passed']:data['passed_software_sequence']=False

def manager_process_identity(pid,session_id,expected_executable,proc_root=Path('/proc')):
    proc=proc_root/str(pid);args=(proc/'cmdline').read_bytes().split(b'\0')
    if b'/opt/ros/jazzy/lib/controller_manager/ros2_control_node' not in args:
        raise ValueError('manager executable command identity differs')
    if ('PUPPER_PROBE_SESSION_ID='+session_id).encode() not in (proc/'environ').read_bytes().split(b'\0'):
        raise ValueError('manager session identity differs')
    if str((proc/'exe').resolve(strict=True))!=expected_executable:raise ValueError('manager actual executable changed')
    stat=(proc/'stat').read_text();fields=stat[stat.rfind(')')+2:].split()
    return fields[19] # deliberately no PPid test after original ownership capture

def arm_shutdown(args, identity, manager):
    """Capture the same already-reviewed identities; retain ArmError.handle in caller."""
    launch=_shutdown.capture_process(args.stack_pid,args.session_id,args.stack_launch.resolve())
    target=_shutdown.capture_process(manager['pid'],args.session_id,manager['executable'])
    if launch['start_ticks']!=identity:raise RuntimeError('launch identity changed before coordinator arm')
    if target['start_ticks']!=manager['start_ticks'] or target['executable']!=manager['executable']:
        raise RuntimeError('manager identity changed before coordinator arm')
    return _shutdown.arm(args.report.parent/'shutdown-coordinator-v2',manager=target,launch=launch,session_id=args.session_id,mode=getattr(args,'mode','partial-init'))

def finish_owned_shutdown(args, identity, handle, data):
    """After arm, exactly one coordinator owns signals; no direct retry on failure."""
    if handle is not None:
        data['shutdown_coordinator_handle']=handle
        try:
            result=_shutdown.request_and_wait(handle,'probe finally')
            data['shutdown_coordinator_result']=result
            if not _shutdown.nominal_cleanup(result):
                data['passed_software_sequence']=False
                data['shutdown_readiness_blocker']='Non-nominal owner recovery or incomplete cleanup requires review before another motion trial.'
        except Exception as error:
            data['shutdown_coordinator_error']=repr(error);data['passed_software_sequence']=False
        return
    # Before any owner exists, preserve the original exact-launch cleanup.
    try:
        if process_identity(args.stack_pid,args.stack_launch,args.session_id)!=identity:raise RuntimeError('stack PID reused')
        os.kill(args.stack_pid,signal.SIGINT);data['stack_sigint_sent']=True
    except FileNotFoundError:data['stack_already_exited']=True
    except Exception as error:data['stack_shutdown_error']=repr(error);data['passed_software_sequence']=False

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('floor-stock-zero',),default='floor-stock-zero')
    parser.add_argument('--stack-pid',type=int,required=True)
    parser.add_argument('--stack-launch',type=Path,required=True)
    parser.add_argument('--manager-pid',type=int,required=True)
    parser.add_argument('--instance-id',required=True)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--session-id',required=True)
    args=parser.parse_args()
    require_motion_enabled()
    if os.environ.get('ROS_DOMAIN_ID')!='168':parser.error('requires isolated ROS_DOMAIN_ID=168')
    if not re.fullmatch('[0-9a-f]{32}',args.instance_id):parser.error('invalid read-only authority instance')
    if args.report.exists():parser.error('report exists; choose a new record')
    if not re.fullmatch('[0-9a-f]{32}',args.session_id):parser.error('session identity must be a fresh 32-hex UUID')
    identity=process_identity(args.stack_pid,args.stack_launch,args.session_id)
    data={'scope':'Supervised floor stock zero-speed stand and actual single-source physical Options/R3; no walking velocity or software proof of floor support',
          'supervised_mode':args.mode,'stack_pid':args.stack_pid,'stack_identity':identity,'session_id':args.session_id,'requested_authority_instance':args.instance_id}
    initial_manager=None;shutdown_handle=None
    node=executor=context=None;recorder=None;joy_child=None;joy_log=None
    try:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        recorder=Recorder(args.report.with_suffix('.evidence.jsonl'),data)
        initial_manager=manager_identity(args.manager_pid,args.stack_pid,args.session_id)
        data['initial_manager_identity']=initial_manager
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Joy,JointState,Imu
        from rcl_interfaces.srv import GetParameters
        from std_msgs.msg import String,Float32MultiArray,Float32
        from controller_manager_msgs.srv import ListControllers
        source=Path(os.environ['PUPPER_STAGED_GEMINI_DIR'])/'robot'/'stop_status.py'
        spec=importlib.util.spec_from_file_location('probe_stop_status',source)
        module=importlib.util.module_from_spec(spec)
        if hashlib.sha256(source.read_bytes()).hexdigest()!='ccebd758a101f15023946363114c0b32f3b5717914326cac77605058ca0bb9ca':
            raise RuntimeError('stop parser source differs from reviewed v4')
        spec.loader.exec_module(module)
        tracker=module.StopStatus()
        context=Context();rclpy.init(context=context)
        node=rclpy.create_node('supervised_console_init_stop_probe',context=context)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        zero=node.create_publisher(Twist,'/cmd_vel',10)
        probe=Probe(time.monotonic())
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        raw=recorder.raw;last_sensor={'joint_states':None,'imu':None}
        joy_graph={'ready':False,'reason':'not_discovered'};bound_joy_gid=None
        def record(kind,row):
            recorder.append(kind,row)
        def status(msg):
            now=time.monotonic();accepted=tracker.accept(msg.data,now)
            record('status',{'monotonic':now,'payload':msg.data,'accepted':accepted})
        def joint(msg):
            now=time.monotonic();values=list(msg.position)+list(msg.velocity)
            valid=valid_joints(msg.name,msg.position,msg.velocity)
            last_sensor['joint_states']=now if valid else None
            record('joint_states',{'monotonic':now,'stamp_sec':msg.header.stamp.sec,'stamp_nanosec':msg.header.stamp.nanosec,'name':list(msg.name),'position':list(msg.position),'velocity':list(msg.velocity),'effort':list(msg.effort)})
        def imu(msg):
            now=time.monotonic();q=[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w];w=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z];a=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
            valid,norm,tilt=imu_quality(q,w,a)
            last_sensor['imu']=now if valid else None
            record('imu',{'monotonic':now,'stamp_sec':msg.header.stamp.sec,'stamp_nanosec':msg.header.stamp.nanosec,'frame_id':msg.header.frame_id,'quaternion_xyzw':q,'angular_velocity':w,'linear_acceleration':a,'quaternion_norm':norm,'tilt_rad':tilt,'valid_for_release':valid})
        node.create_subscription(String,module.TOPIC,status,qos)
        sensor_qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(JointState,'/joint_states',joint,sensor_qos)
        node.create_subscription(Imu,'/imu_sensor_broadcaster/imu',imu,sensor_qos)
        if args.mode=='floor-stock-zero':
            def action_receipt(kind,msg):
                values=list(msg.data);valid=len(values)==12 and all(math.isfinite(x) for x in values)
                record(kind,{'monotonic':time.monotonic(),'values':values,'valid':valid})
                if not valid:probe.fail(time.monotonic(),'invalid '+kind+' telemetry')
            def latency_receipt(kind,msg):
                record(kind,{'monotonic':time.monotonic(),'seconds':float(msg.data)})
            for kind in ('policy_output','position_command'):
                node.create_subscription(Float32MultiArray,'/'+DEFAULT+'/'+kind,lambda msg,k=kind:action_receipt(k,msg),sensor_qos)
            for kind,topic in [('imu_latency','imu_latency_seconds'),('policy_inference_latency','policy_inference_latency_seconds')]:
                node.create_subscription(Float32,'/'+DEFAULT+'/'+topic,lambda msg,k=kind:latency_receipt(k,msg),sensor_qos)
        def physical_joy(msg,info):
            now=time.monotonic();gid=bytes(info.publisher_gid).hex()
            qualified=joy_contract.source_matches(joy_graph,gid)
            if not joy_contract.valid_joy(list(msg.buttons),list(msg.axes)):probe.fail(now,'physical Joy shape/value differs from exact input contract')
            elif qualified:probe.joy(now,list(msg.buttons))
            elif probe.neutral_seen:probe.fail(now,'unexpected Joy source after readiness qualification')
            record('physical_joy',{'monotonic':now,'publisher_gid':gid,'qualified_exact_source':qualified,'buttons':list(msg.buttons),'axes':list(msg.axes),'stamp_sec':msg.header.stamp.sec,'stamp_nanosec':msg.header.stamp.nanosec})
        node.create_subscription(Joy,joy_contract.TOPIC,physical_joy,joy_contract.SUBSCRIBER_DEPTH)
        def cmd_check(msg):
            values=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
            if any(not math.isfinite(v) or v!=0 for v in values):probe.fail(time.monotonic(),'unexpected nonzero cmd_vel')
        node.create_subscription(Twist,'/cmd_vel',cmd_check,10)
        client=node.create_client(ListControllers,'/controller_manager/list_controllers')
        parameter_client=node.create_client(GetParameters,'/'+DEFAULT+'/get_parameters')
        expected={'init_duration':2.0,'fade_in_duration':2.0,'gain_multiplier':1.0,'max_body_angle':1.0,'game_buttons':False,'init_kps':[7.5]*12,'init_kds':[.25]*12,'kp_scale':[1.0]*12}
        if args.mode=='floor-stock-zero':expected['use_sim_time']=False
        parameter_future=None;parameters_verified=False;parameter_started=None
        inventory=inventory_at=future=None;pending_at=0.;next_query=0.;next_zero=0.
        deadline=time.monotonic()+25.;next_graph=0.
        while not probe.done and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01);now=time.monotonic()
            if probe.done:break
            if now>=next_zero:
                zero.publish(Twist());next_zero=now+.05
                record('commands',{'monotonic':now,'topic':'/cmd_vel','linear':[0,0,0],'angular':[0,0,0]})
            fault=None
            if not parameters_verified:
                if parameter_future is None and parameter_client.service_is_ready():
                    request=GetParameters.Request();request.names=list(expected)+['model_path'];parameter_future=parameter_client.call_async(request);parameter_started=now
                elif parameter_future is not None and parameter_future.done():
                    response=parameter_future.result();values={}
                    if response is None or len(response.values)!=len(expected)+1:raise RuntimeError('default runtime parameters missing')
                    for name,value in zip(list(expected)+['model_path'],response.values):
                        values[name]=value.bool_value if value.type==1 else value.double_value if value.type==3 else value.string_value if value.type==4 else list(value.double_array_value) if value.type==8 else None
                    if any(values[name]!=value for name,value in expected.items()):raise RuntimeError('default init/fade/gains/settings differ')
                    policy=Path(values['model_path']);policy_sha=hashlib.sha256(policy.read_bytes()).hexdigest()
                    if policy_sha!='ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712':raise RuntimeError('default policy identity differs')
                    data['runtime_default_parameters']=values;data['runtime_policy_sha256']=policy_sha
                    data['loaded_library_preflight']=manager_identity(args.manager_pid,args.stack_pid,args.session_id)
                    parameters_verified=True
                elif parameter_started is not None and now-parameter_started>.75:fault='default parameter read timeout'
            if future is not None:
                if future.done():
                    try:
                        response=future.result()
                        if response is None:raise RuntimeError('null controller inventory')
                        inventory={c.name:{'state':c.state,'type':c.type} for c in response.controller};inventory_at=now
                        record('controller_states',{'monotonic':now,'controllers':inventory})
                    except Exception as error:fault='inventory response failed: '+repr(error)
                    future=None;next_query=now+.05
                elif now-pending_at>.75:fault='controller inventory timeout'
            elif now>=next_query and client.service_is_ready():
                future=client.call_async(ListControllers.Request());pending_at=now
            state=tracker.state
            if tracker.instance_id not in (None,args.instance_id):fault='authority instance mismatch'
            if tracker.permanent_error:fault=tracker.permanent_error
            fresh_status=tracker.received is not None and 0<=now-tracker.received<=.5 and tracker.error is None
            if probe.phase!='preflight' and not fresh_status:fault='shared stop status missing/stale'
            if fresh_status and state and any(state[k] for k in ('fault','unknown_remote_submission','shutting_down')):
                fault='shared stop authority fault/unknown/shutdown'
            sensors_ready=all(t is not None and 0<=now-t<=.2 for t in last_sensor.values())
            if probe.phase!='preflight' and not sensors_ready:fault='joint/IMU feedback stale or invalid'
            if probe.phase=='preflight' and not fault and parameters_verified and release_evidence_ready(now,state,tracker.received,inventory,inventory_at,last_sensor):
                if node.get_publishers_info_by_topic('/joy'):raise RuntimeError('Joy already has a publisher; single-source test refused')
                data['loaded_library_before_release']=manager_identity(args.manager_pid,args.stack_pid,args.session_id)
                from ament_index_python.packages import get_package_prefix
                joy_executable=Path(get_package_prefix('joy'))/'lib/joy/joy_node'
                command=joy_contract.command(joy_executable)
                joy_sha=hashlib.sha256(joy_executable.read_bytes()).hexdigest()
                recorder.checkpoint('pre_physical_enable',data)
                try:shutdown_handle=arm_shutdown(args,identity,initial_manager)
                except _shutdown.ArmError as error:shutdown_handle=error.handle;raise
                data['shutdown_coordinator_handle']=shutdown_handle
                if not release_evidence_ready(time.monotonic(),tracker.state,tracker.received,inventory,inventory_at,last_sensor):raise RuntimeError('feedback became stale while arming; physical Joy not enabled')
                joy_log=(args.report.parent/'physical-joy.log').open('x')
                probe.enable(time.monotonic(),state['epoch'])
                joy_child=subprocess.Popen(command,stdout=joy_log,stderr=subprocess.STDOUT)
                data['physical_joy_process']={'pid':joy_child.pid,'command':command,'executable_sha256':joy_sha}
                recorder.append('lifecycle',{'event':'physical_joy_process_started',**data['physical_joy_process']})
                (args.report.parent/'physical-input-enabled.json').write_text(json.dumps({'session_id':args.session_id,'deadline_monotonic':shutdown_handle['spec']['deadline_monotonic'],'instructions':'Options once, observe stock stand, R3 to stop; physical power disconnect if necessary'}))
            if joy_child is not None:
                if joy_child.poll() is not None:fault='physical Joy process exited'
                if now>=next_graph:
                    infos=node.get_publishers_info_by_topic(joy_contract.TOPIC)
                    rows=[joy_contract.endpoint(i) for i in infos];diagnostic=joy_contract.graph_diagnostic(rows,bound_joy_gid)
                    if diagnostic['ready'] and bound_joy_gid is None:bound_joy_gid=diagnostic['gid']
                    joy_graph=diagnostic
                    record('joy_publishers',{'monotonic':now,**diagnostic});next_graph=now+.1
                    if rows and not diagnostic['ready']:fault='Joy graph rejected: '+diagnostic['reason']
                    if (diagnostic['ready'] and not fault
                            and physical_input_ready(now,probe,state,tracker.received,inventory,inventory_at,last_sensor)
                            and not (args.report.parent/'physical-input-ready.json').exists()):
                        (args.report.parent/'physical-input-ready.json').write_text(json.dumps({'session_id':args.session_id,'ready':True,'monotonic':now,'deadline_monotonic':shutdown_handle['spec']['deadline_monotonic'],'instructions':'Options once; after about5s of stable standing press R3. Stop immediately if unusual.'}))
                if shutdown_handle and now>=shutdown_handle['spec']['deadline_monotonic']-.1:fault='20s physical-input window exhausted'
            probe.tick(now,state if fresh_status and sensors_ready and parameters_verified else None,inventory,inventory_at,fault,status_at=tracker.received)
        if not probe.done:probe.fail(time.monotonic(),'overall probe deadline')
        data.update(probe.result());phase_evidence(data,raw);data['raw_receipts']=raw
        recorder.append('lifecycle',{'event':'probe_loop_finished','result':probe.result()})
    except BaseException as error:
        if 'probe' in locals():
            try:probe.fail(time.monotonic(),'exception before completed sequence')
            except Exception:pass
            data.update(probe.result())
        if 'raw' in locals():phase_evidence(data,raw);data['raw_receipts']=raw
        data.update(passed_software_sequence=False,exception=repr(error))
        if recorder is not None:
            try:recorder.append('lifecycle',{'event':'probe_exception','error':repr(error)})
            except Exception:pass
    finally:
        finish_owned_shutdown(args,identity,shutdown_handle,data)
        if joy_child is not None and joy_child.poll() is None:
            joy_child.send_signal(signal.SIGINT)
            try:joy_child.wait(timeout=3)
            except subprocess.TimeoutExpired:data['physical_joy_cleanup_error']='owned child did not exit';data['passed_software_sequence']=False
        if joy_child is not None:data['physical_joy_exit_code']=joy_child.poll()
        data['physical_joy_cleanup_scope']='Normal probe finally owns this child. Abrupt probe death may leave Joy alive; detached manager/launch shutdown remains independent. Verify no Joy survivor before another launch.'
        if joy_log is not None:joy_log.close()
        for name,cleanup in [('executor',lambda:executor.shutdown(timeout_sec=.5) if executor is not None else None),
                             ('node',lambda:node.destroy_node() if node is not None else None),
                             ('context',lambda:context.shutdown() if context is not None and context.ok() else None)]:
            try:cleanup()
            except Exception as error:
                data.setdefault('local_cleanup_errors',[]).append(name+': '+repr(error));data['passed_software_sequence']=False
        # The armed coordinator already waited for owned exits; no second signal/wait owner.
        limit=time.monotonic()+(10. if shutdown_handle is None else 0.)
        while time.monotonic()<limit:
            stack_state=original_process_state(args.stack_pid,identity)
            manager_state=original_process_state(args.manager_pid,initial_manager['start_ticks']) if initial_manager else 'unverifiable'
            if stack_state!='same_identity_running' and manager_state!='same_identity_running':break
            time.sleep(.05)
        data['stack_process_state']=original_process_state(args.stack_pid,identity)
        data['manager_process_state']=original_process_state(args.manager_pid,initial_manager['start_ticks']) if initial_manager else 'unverifiable'
        data['driver_lock_check']=check_driver_lock()
        if (initial_manager is None or 'same_identity_running' in (data['stack_process_state'],data['manager_process_state'])
                or not data['driver_lock_check']['available']):data['passed_software_sequence']=False
        args.report.parent.mkdir(parents=True,exist_ok=True)
        # All sync/final report I/O is after the existing cleanup sequence.
        outcome=_recording.finish_report(args.report,data,recorder)
        print(json.dumps(outcome,indent=2,allow_nan=False))
    return 0 if data.get('passed_software_sequence') else 1
if __name__=='__main__':sys.exit(main())
