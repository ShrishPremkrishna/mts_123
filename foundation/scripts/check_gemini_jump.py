#!/usr/bin/env python3
"""Bounded physics screen of Gemini's actual stopped reference-jump request.

Uses Float32 message values, immediate first-inference receipt and paired controls.
The native host transcribes production math but does not execute ROS/API/mux.
No policy, hardware or runtime feature gate is modified by this diagnostic.
"""
from __future__ import annotations
import argparse
from collections import deque
import gzip, hashlib, importlib.util, json, shutil, subprocess, sys, tarfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
HZ=520
CONTACT_DISTANCE_M=1e-6

def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module;spec.loader.exec_module(module);return module
jump=load(ROOT/'foundation/scripts/check_shipped_jump.py','api_jump_shared')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

class ApiHost:
    def __init__(self,binary,policy,replicas,log):
        self.log=log.open('w')
        self.proc=subprocess.Popen([str(binary),str(policy),str(replicas),'--api-reference'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
    def request(self,value):
        self.proc.stdin.write(json.dumps(value,allow_nan=False)+'\n');self.proc.stdin.flush()
        line=self.proc.stdout.readline()
        if not line:raise RuntimeError('native API-reference host terminated; inspect controller.log')
        return json.loads(line)
    def close(self):
        if self.proc.poll() is None:self.proc.stdin.close();self.proc.wait(timeout=10)
        self.log.close()
        if self.proc.returncode:raise RuntimeError(f'native API-reference host exit {self.proc.returncode}')


def message_payload(reference):
    values=[reference[k] for k in ('playback_s','active_s','cross_fade_s')]+[v for row in reference['table'] for v in row]
    payload=np.asarray(values,dtype=np.float32)
    if not np.isfinite(payload).all() or len(reference['table'])<2 or any(len(row)!=12 for row in reference['table']):raise ValueError('invalid Float32 reference')
    if not 0<payload[1]<=payload[0] or not 0<payload[2]<=payload[1]:raise ValueError('invalid reference timing')
    return payload.astype(float).tolist()

def validate_class_reference(reference, policy):
    payload=message_payload(reference)
    slot=policy['jump_slot']
    if (np.asarray(reference['table']).shape != np.asarray(slot['jump_table']).shape
            or not np.allclose(reference['table'],slot['jump_table'],atol=1e-6,rtol=0)
            or any(not np.isclose(reference[k],slot[k],atol=1e-6,rtol=0)
                   for k in ('playback_s','active_s','cross_fade_s'))):
        raise ValueError('reference must match actual class table and timing, without redesign')
    return payload


def verify_input_hashes(paths, expected):
    for name,path in paths.items():
        if sha(path)!=expected[name]:raise RuntimeError(f'diagnostic input changed: {name}')


def evaluate(screen,shared,binary,policy,metadata,initial,scenario,trigger,payload,recovery_s,lower,upper,output):
    import mujoco
    model,contact_contract=shared.make_model('full_hulls')
    mujoco.mj_saveModel(model,str(output/'model.mjb'),None)
    names=screen.NAMES;body=model.body('base_link').id;terrain=model.geom('terrain').id;torso=model.geom('base_link_hull').id
    qadr=model.jnt_qposadr[[model.joint(n).id for n in names]];vadr=model.jnt_dofadr[[model.joint(n).id for n in names]]
    controls=np.array([int(np.flatnonzero(model.actuator_trnid[:,0]==model.joint(n).id)[0]) for n in names])
    free=int(np.flatnonzero(model.jnt_type==mujoco.mjtJoint.mjJNT_FREE)[0]);rq=int(model.jnt_qposadr[free]);rv=int(model.jnt_dofadr[free])
    states=[];queues=[]
    for init in initial:
        d=mujoco.MjData(model);d.qpos[rq:rq+3]=init['root_position'];d.qpos[rq+3:rq+7]=init['root_quaternion'];d.qpos[qadr]=init['joint_position']
        mujoco.mj_forward(model,d);states.append(d)
        target=np.clip(d.qpos[qadr],lower,upper);queues.append(deque([(-1.,target.copy()),(0.,target.copy())]))
    stop_at=4.2 if scenario=='fresh' else 6.2
    request_at=stop_at+.5
    trigger_tick=int(np.ceil((request_at-2)*52-1e-10))*10+2*HZ
    actual_start=trigger_tick/HZ
    end_s=actual_start+float(payload[1])+recovery_s
    total_ticks=int(np.ceil(end_s*HZ));end_s=total_ticks/HZ
    host=ApiHost(binary,policy,len(initial),output/'controller.log')
    outputs=[None]*len(initial);active=[True]*len(initial);rows=[];failures=[]
    unstable=[mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC,mujoco.mjtWarning.mjWARN_BADCTRL]
    try:
        with gzip.open(output/'contacts.jsonl.gz','wt') as contact_log:
            for tick in range(total_ticks):
                t=tick/HZ;requested=[.3,0.,0.] if scenario=='walk_then_stop' and 4.2<=t<stop_at else [0.,0.,0.]
                if tick>=2*HZ and (tick-2*HZ)%10==0:
                    request=[]
                    for i,d in enumerate(states):
                        q=d.qpos[rq+3:rq+7]
                        tilt=float(np.arccos(np.clip(1-2*(q[1]**2+q[2]**2),-1,1)))
                        if active[i] and tilt>1:
                            failures.append(dict(replica=i,time_s=t,reason='inherited Gemini1.0rad tilt stop checked at52Hz before inference; trace censored before damping',tilt_rad=tilt));active[i]=False
                        state=dict(elapsed_s=t,command=requested,joint_pos=d.qpos[qadr].tolist(),
                            angular_velocity=d.qvel[rv+3:rv+6].tolist(),quaternion=q.tolist()) if active[i] else None
                        if state is not None and trigger and tick==trigger_tick:state['reference_payload']=payload
                        request.append(state)
                    outputs=host.request(dict(op='infer',states=request))
                    if trigger and tick==trigger_tick:
                        assert all(o is None or o['reference_received'] and abs(o['reference_start_s']-(actual_start-2))<1e-12 for o in outputs)
                if not any(active):break
                for i,d in enumerate(states):
                    if not active[i]:continue
                    if tick<2*HZ:
                        target=(1-t/2)*np.array(initial[i]['joint_position'])+(t/2)*np.array(metadata['default_joint_pos']);kp,kd=7.5,.25
                        before=target.copy();reference=np.zeros(12);raw=np.zeros(12);effective_command=np.zeros(3)
                    else:
                        out=outputs[i];target=np.asarray(out['target']);before=np.asarray(out['before_clip']);reference=np.asarray(out['reference'])
                        raw=np.asarray(out['raw_action']);effective_command=np.asarray(out['effective_command']);kp,kd=metadata['kp'],metadata['kd']
                    final=np.clip(target,lower,upper);delay=queues[i];delay.append((t,final.copy()));cutoff=t-.016
                    while len(delay)>2 and delay[1][0]<=cutoff:delay.popleft()
                    t0,v0=delay[0];t1,v1=delay[1];fraction=np.clip((cutoff-t0)/(t1-t0),0,1);applied=(1-fraction)*v0+fraction*v1
                    d.ctrl[controls]=applied;model.actuator_gainprm[controls,0]=kp;model.actuator_biasprm[controls,1]=-kp;model.actuator_biasprm[controls,2]=-kd
                    mujoco.mj_step(model,d)
                    warnings=[str(w) for w in unstable if d.warning[w].number]
                    if warnings or not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
                        failures.append(dict(replica=i,time_s=(tick+1)/HZ,reason='numerical instability; never count an internal reset as recovery',warnings=warnings));active[i]=False;continue
                    mujoco.mj_forward(model,d);mujoco.mj_subtreeVel(model,d)
                    quat=d.qpos[rq+3:rq+7];tilt=float(np.arccos(np.clip(1-2*(quat[1]**2+quat[2]**2),-1,1)))
                    contacts=[];support=0;torso_contact=False
                    for ci,c in enumerate(d.contact):
                        a,b=int(c.geom1),int(c.geom2)
                        if terrain not in (a,b):continue
                        other=b if a==terrain else a
                        if int(model.geom_bodyid[other])==0:continue
                        force=np.zeros(6);mujoco.mj_contactForce(model,d,ci,force)
                        touching=bool(c.dist<=CONTACT_DISTANCE_M)
                        support+=int(touching);torso_contact|=touching and other==torso
                        contacts.append(dict(geom=other,distance_m=float(c.dist),efc_address=int(c.efc_address),normal_force_n=float(force[0]),touching=touching))
                    contact_log.write(json.dumps(dict(replica=i,time_s=(tick+1)/HZ,contacts=contacts),allow_nan=False)+'\n')
                    rows.append(dict(replica=i,time_s=(tick+1)/HZ,control_time_s=t,
                        root_position=d.qpos[rq:rq+3].copy(),root_quaternion=quat.copy(),joint_position=d.qpos[qadr].copy(),joint_velocity=d.qvel[vadr].copy(),
                        com_position=d.subtree_com[body].copy(),com_velocity=d.subtree_linvel[body].copy(),angular_velocity=d.qvel[rv+3:rv+6].copy(),
                        tilt_rad=tilt,ground_contact_count=support,torso_contact=torso_contact,torque=d.actuator_force[controls].copy(),
                        command=np.asarray(requested),effective_command=effective_command.copy(),reference=reference.copy(),raw_action=raw.copy(),
                        before_clip=before.copy(),controller_target=target.copy(),driver_target=final.copy(),delayed_target=applied.copy(),
                        controller_clipped=np.abs(before-target)>1e-6,driver_clipped=np.abs(final-target)>1e-6))
    finally:host.close()
    trace={key:np.asarray([row[key] for row in rows]) for key in rows[0]}
    np.savez_compressed(output/'trace.npz',**trace)
    reports=jump.summarize(trace,len(initial),actual_start,float(payload[1]),end_s,failures)
    return_time=actual_start+float(payload[0])+1.2
    return_trace={k:v[trace['time_s']<=return_time+1/HZ] for k,v in trace.items()}
    at_return=jump.summarize(return_trace,len(initial),actual_start,float(payload[1]),return_time,[f for f in failures if f['time_s']<=return_time],)
    result=dict(mode='api_reference' if trigger else 'gait_only',scenario=scenario,stop_command_elapsed_s=stop_at,requested_reference_elapsed_s=request_at,
        accepted_reference_elapsed_s=actual_start,nominal_api_return_elapsed_s=return_time,api_return_window=[r['recovery'] for r in at_return],end_s=end_s,
        policy_sha256=sha(policy),model_sha256=sha(output/'model.mjb'),contact_contract=contact_contract,
        geometry_names={str(g):model.geom(g).name for g in range(model.ngeom)},mass_kg=float(model.body_mass.sum()),
        failures=failures,replicas=reports,trace_sha256=sha(output/'trace.npz'),contacts_sha256=sha(output/'contacts.jsonl.gz'))
    write(output/'metrics.json',result);return result


def run(args):
    screen=load(ROOT/'foundation/scripts/check_export_locomotion.py','api_jump_screen')
    shared=load(ROOT/'foundation/scripts/check_spider_handoff.py','api_jump_model')
    if not 1<=args.replicas<=8 or not np.isfinite(args.recovery_seconds) or args.recovery_seconds<3:raise ValueError('bounded replicas and at least3s recovery required')
    values,lower,upper,driver=screen.validate_inputs({'shipped':args.policy},[[0,0,0],[.3,0,0]],args.driver_limits)
    policy=values['shipped'];reference=json.loads(args.reference.read_text());payload=validate_class_reference(reference,policy)
    initial=screen.initial_conditions(policy['default_joint_pos'],args.replicas,args.seed)
    if any(np.any(np.array(x['joint_position'])<lower) or np.any(np.array(x['joint_position'])>upper) for x in initial):raise ValueError('initial pose outside driver limits')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    sources={ROOT/'foundation/scripts/check_gemini_jump.py',ROOT/'foundation/scripts/check_shipped_jump.py',ROOT/'foundation/scripts/check_export_locomotion.py',ROOT/'foundation/scripts/check_spider_handoff.py',ROOT/'foundation/cpp/export_locomotion_controller.cpp',ROOT/'pupper_gait_deploy/src/neural_controller.cpp',ROOT/'gemini_pupper/robot/robot_api.py',ROOT/'gemini_pupper/robot/tricks.py',ROOT/'gemini_pupper/robot/launch/gemini_pupper.yaml',ROOT/'pupperv3-monorepo/ros2_ws/src/cmd_vel_mux/src/cmd_vel_mux_node.cpp',ROOT/'pupper_gait_deploy/config.yaml',*list((ROOT/'pupper_gait_deploy/src').glob('*.hpp'))}
    sources.update(p for p in (ROOT/'pupper-mjlab/src/mjlab/asset_zoo/robots/pupper_v3').rglob('*') if p.is_file() and p.suffix in {'.py','.xml','.obj','.stl'})
    sources.update((ROOT/'pupper-mjlab/src/mjlab/tasks/tracking/config/pupper').glob('*.py'))
    sources.update([ROOT/'pupper-mjlab/src/mjlab/tasks/pupper/deployment_contract.py',ROOT/'pupper-mjlab/src/mjlab/tasks/pupper/locomotion_contact_contract.py'])
    hashes={str(p.relative_to(ROOT)):sha(p) for p in sorted(sources)}
    inputs=output/'inputs';inputs.mkdir()
    original_inputs={'default_policy.json':args.policy,'reference.json':args.reference,'driver-limits.json':args.driver_limits,'trick.yaml':ROOT/'gemini_pupper/tricks/jump/trick.yaml'}
    for name,path in original_inputs.items():shutil.copy2(path,inputs/name)
    write(inputs/'float32-message.json',payload)
    plan=dict(protocol='gemini_stopped_jump_physics_v1',source_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),source_sha256=hashes,
      inputs_sha256={p.name:sha(p) for p in inputs.iterdir()},initial_conditions=initial,seed=args.seed,
      cases=['fresh-gait','fresh-jump','walk_then_stop-gait','walk_then_stop-jump'],
      timeline={'init_s':2,'fade_s':2,'ready_s':4.2,'prior_walk_vx_m_s':.3,'prior_walk_start_s':4.2,'prior_walk_stop_s':6.2,'zero_command_wait_s':.5,'immediate_trigger':'first52Hz inference at or after API publication; no grid snapping','recovery_after_active_s':args.recovery_seconds},
      unchanged={'policy_sha256':sha(args.policy),'reference_sha256':sha(args.reference),'driver_limits':driver},
      hardware_validated=False,installed=False,
      limitations=['Nominal full-hull MuJoCo mechanics; same four engineering initial poses as old screen, not domain-randomized trials.',
        'Offline native math/history host, not actual ROS service/topic/mux execution. Quiet joystick and stopped follower assumed; a live higher-priority joystick can override API zero commands.',
        'Tilt checked before inference at52Hz, then trace censored. Production instead enters emergency damping; no post-stop recovery claim.',
        'Perfect instantaneous joint/IMU feedback, fixed16ms target delay and model3Nm cap are assumptions, not verified robot hardware.',
        'API waits0.5s after zero command, not a measured standing-state guard. Model return-time screen is independent evidence; actual receipt only confirms playback.',
        'Message table/timing are actual Float32 transport values, including class table decimal rounding; no hop or table redesign.'])
    write(output/'plan.json',plan);write(output/'status.json',dict(status='running'))
    try:
      binary,build=screen.build_controller(output);results={}
      for scenario in ['fresh','walk_then_stop']:
       for trigger in [False,True]:
        name=scenario+('-jump' if trigger else '-gait');folder=output/name;folder.mkdir()
        results[name]=evaluate(screen,shared,binary,args.policy,policy,initial,scenario,trigger,payload,args.recovery_seconds,lower,upper,folder)
        write(output/'status.json',dict(status='running',completed=list(results)))
        print(json.dumps({'case':name,'failures':results[name]['failures'],'recoveries':[x['recovery']['meets_stability_screen'] for x in results[name]['replicas']]}),flush=True)
      if hashes!={str(p.relative_to(ROOT)):sha(p) for p in sorted(sources)}:raise RuntimeError('source changed during diagnostic')
      verify_input_hashes(original_inputs,plan['inputs_sha256'])
      pairs=[]
      for scenario in ['fresh','walk_then_stop']:
       with np.load(output/(scenario+'-gait')/'trace.npz') as a,np.load(output/(scenario+'-jump')/'trace.npz') as b:
        before=results[scenario+'-jump']['accepted_reference_elapsed_s'];ma=a['time_s']<=before;mb=b['time_s']<=before
        assert np.array_equal(a['time_s'][ma],b['time_s'][mb]) and np.array_equal(a['replica'][ma],b['replica'][mb])
        assert np.array_equal(a['root_position'][ma],b['root_position'][mb])
       pairs.append(dict(scenario=scenario,pre_trigger_physical_states_exact=True))
      report=dict(protocol=plan['protocol'],hardware_validated=False,installed=False,cases=results,paired=pairs,plan_sha256=sha(output/'plan.json'),build_command=build,binary_sha256=sha(binary),source_hashes_unchanged=True)
      write(output/'report.json',report)
      with tarfile.open(output/'sources.tar.gz','w:gz') as archive:
       for p in sorted(sources):archive.add(p,arcname=str(p.relative_to(ROOT)),recursive=False)
      write(output/'status.json',dict(status='completed',report_sha256=sha(output/'report.json'),sources_sha256=sha(output/'sources.tar.gz')))
    except BaseException as error:
      write(output/'status.json',dict(status='error',error_type=type(error).__name__,error=str(error)));raise

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--policy',type=Path,default=ROOT/'pupper_gait_deploy/policies/default_policy.json')
    p.add_argument('--reference',type=Path,default=ROOT/'gemini_pupper/tricks/jump/reference.json')
    p.add_argument('--driver-limits',type=Path,default=ROOT/'foundation/configs/class-driver-joint-limits.json')
    p.add_argument('--replicas',type=int,default=4);p.add_argument('--seed',type=int,default=101)
    p.add_argument('--recovery-seconds',type=float,default=3.);p.add_argument('--output',type=Path,required=True)
    run(p.parse_args())
