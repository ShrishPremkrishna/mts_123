#!/usr/bin/env python3
"""Actual shipped jump-table physics probe; no robot or button/ROS validation.

Runs matched gait-only and preplanned jump reference windows through the native
RTNeural host. All state/contact samples are aligned after each 520 Hz integration
step and mj_forward. The shipped policy and its reference table are unchanged.
"""
from __future__ import annotations
import argparse
from collections import deque
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
HZ=520
CONTACT_DISTANCE_M=1e-6
MIN_FLIGHT_S=.02
MIN_RISE_M=.005
MIN_TAKEOFF_VELOCITY_M_S=.05


def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    return module


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def flight_events(time,contact_count,height,vertical_velocity,start,end):
    """Conservative airborne intervals: no terrain contact within 1um, not just feet."""
    t=np.asarray(time);contact=np.asarray(contact_count);z=np.asarray(height);vz=np.asarray(vertical_velocity)
    airborne=contact==0;events=[];i=0
    while i<len(t):
        if not airborne[i]:i+=1;continue
        first=i
        while i+1<len(t) and airborne[i+1]:i+=1
        last=i;i+=1
        if t[last]<start or t[first]>end:continue
        duration=float(t[last]-t[first]+1/HZ)
        rise=float(np.max(z[first:last+1])-z[first])
        has_liftoff=first>0 and contact[first-1]>0
        landed=last+1<len(t) and contact[last+1]>0
        events.append(dict(liftoff_time_s=float(t[first]),last_airborne_time_s=float(t[last]),
            duration_s=duration,grounded_before=bool(has_liftoff),landed=bool(landed),
            landing_time_s=float(t[last+1]) if landed else None,
            rise_after_liftoff_m=rise,takeoff_vertical_velocity_m_s=float(vz[first]),
            apex_time_s=float(t[first+np.argmax(z[first:last+1])]),
            meets_jump_evidence=bool(t[first]>=start and has_liftoff and duration>=MIN_FLIGHT_S and rise>=MIN_RISE_M
                                    and vz[first]>=MIN_TAKEOFF_VELOCITY_M_S)))
    return events


def summarize(trace,replicas,slot_elapsed,active_s,end_s,failures):
    result=[]
    for replica in range(replicas):
        chosen=trace['replica']==replica
        x={k:v[chosen] for k,v in trace.items()};t=x['time_s']
        if not len(t):
            result.append(dict(replica=replica,no_samples=True));continue
        baseline=(t>=slot_elapsed-.5)&(t<slot_elapsed)
        event_window=(t>=slot_elapsed)&(t<=slot_elapsed+active_s+1)
        recovery=t>=end_s-1
        events=flight_events(t,x['ground_contact_count'],x['com_position'][:,2],x['com_velocity'][:,2],
                             slot_elapsed,slot_elapsed+active_s+1)
        complete=bool(t[-1]>=end_s-.5/HZ and not any(f['replica']==replica for f in failures))
        tail_complete=bool(complete and np.count_nonzero(recovery)>=HZ)
        rec=dict(complete_last_second=tail_complete,
            max_tilt_rad=float(np.max(x['tilt_rad'][recovery])) if np.any(recovery) else None,
            rms_angular_velocity_rad_s=float(np.sqrt(np.mean(x['angular_velocity'][recovery]**2))) if np.any(recovery) else None,
            torso_contact_samples=int(np.count_nonzero(x['torso_contact'][recovery])),
            final_grounded=bool(x['ground_contact_count'][-1]>0))
        rec['meets_stability_screen']=bool(tail_complete and rec['max_tilt_rad']<.35
            and rec['rms_angular_velocity_rad_s']<2 and rec['torso_contact_samples']==0 and rec['final_grounded'])
        result.append(dict(replica=replica,completed=complete,
            baseline_com_height_m=float(np.mean(x['com_position'][baseline,2])) if np.any(baseline) else None,
            peak_com_rise_from_baseline_m=float(np.max(x['com_position'][event_window,2])-np.mean(x['com_position'][baseline,2])) if np.any(event_window) and np.any(baseline) else None,
            flight_events=events,flight_and_landing_evidence=any(e['meets_jump_evidence'] and e['landed'] for e in events),
            recovery=rec,max_tilt_rad=float(np.max(x['tilt_rad'])),
            peak_abs_torque_nm=float(np.max(np.abs(x['torque']))),
            controller_clip_joint_fraction=float(np.mean(x['controller_clipped'])),
            driver_clip_joint_fraction=float(np.mean(x['driver_clipped'])),
            max_target_step_rad=float(np.max(np.abs(np.diff(x['driver_target'],axis=0))))))
    return result


def evaluate(screen,shared,binary,policy,metadata,initial,command,scheduled,slot_start,recovery_s,lower,upper,output):
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
    end_s=2+slot_start+metadata['jump_slot']['active_s']+recovery_s
    total_ticks=int(np.ceil(end_s*HZ));end_s=total_ticks/HZ
    host=screen.Host(binary,policy,len(initial),output/'controller.log',scheduled_jump_start=slot_start if scheduled else None)
    outputs=[None]*len(initial);active=[True]*len(initial);rows=[];failures=[]
    unstable=[mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC,mujoco.mjtWarning.mjWARN_BADCTRL]
    try:
        with gzip.open(output/'contacts.jsonl.gz','wt') as contact_log:
            for tick in range(total_ticks):
                t=tick/HZ;requested=command if t>=4.2 else [0.,0.,0.]
                if tick>=2*HZ and (tick-2*HZ)%10==0:
                    request=[]
                    for i,d in enumerate(states):
                        request.append(dict(elapsed_s=t,command=requested,joint_pos=d.qpos[qadr].tolist(),
                            angular_velocity=d.qvel[rv+3:rv+6].tolist(),quaternion=d.qpos[rq+3:rq+7].tolist()) if active[i] else None)
                    outputs=host.request(dict(op='infer',states=request))
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
                    if tilt>1:
                        failures.append(dict(replica=i,time_s=(tick+1)/HZ,reason='inherited Gemini1.0rad tilt stop',tilt_rad=tilt));active[i]=False
    finally:host.close()
    trace={key:np.asarray([row[key] for row in rows]) for key in rows[0]}
    np.savez_compressed(output/'trace.npz',**trace)
    reports=summarize(trace,len(initial),2+slot_start,metadata['jump_slot']['active_s'],end_s,failures)
    result=dict(mode='scheduled_jump' if scheduled else 'gait_only',command=command,end_s=end_s,
        policy_sha256=sha(policy),model_sha256=sha(output/'model.mjb'),contact_contract=contact_contract,
        geometry_names={str(g):model.geom(g).name for g in range(model.ngeom)},mass_kg=float(model.body_mass.sum()),
        failures=failures,replicas=reports,trace_sha256=sha(output/'trace.npz'),contacts_sha256=sha(output/'contacts.jsonl.gz'))
    write(output/'metrics.json',result);return result


def run(args):
    screen=load(ROOT/'foundation/scripts/check_export_locomotion.py','jump_screen')
    shared=load(ROOT/'foundation/scripts/check_spider_handoff.py','jump_shared')
    commands=[[0.,0.,0.],[.3,0.,0.]]
    if not 1<=args.replicas<=8 or args.recovery_seconds<3 or not np.isfinite([args.slot_start,args.recovery_seconds]).all():raise ValueError('bounded replicas and at least3s finite recovery required')
    policies,lower,upper,driver=screen.validate_inputs({'shipped':args.policy},commands,args.driver_limits)
    metadata=policies['shipped'];slot=metadata.get('jump_slot')
    if not slot or args.slot_start<2 or abs(args.slot_start/slot['grid_s']-round(args.slot_start/slot['grid_s']))>1e-9:raise ValueError('actual shipped jump_slot and grid-aligned postfade slot required')
    screen.validate_position_contract(metadata)
    if slot.get('hop_speed',float('inf'))<=.3:raise ValueError('commands would select running hop rather than actual jump')
    initial=screen.initial_conditions(metadata['default_joint_pos'],args.replicas,args.seed)
    if any(np.any(np.array(x['joint_position'])<lower) or np.any(np.array(x['joint_position'])>upper) for x in initial):raise ValueError('initial pose exceeds driver limits')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    sources=[Path(__file__).resolve(),ROOT/'foundation/scripts/check_export_locomotion.py',ROOT/'foundation/scripts/check_spider_handoff.py',
        ROOT/'foundation/cpp/export_locomotion_controller.cpp',ROOT/'pupper_gait_deploy/src/neural_controller.cpp',ROOT/'pupper_gait_deploy/config.yaml',
        *sorted((ROOT/'pupper_gait_deploy/src').glob('*.hpp')),
        *sorted((ROOT/'pupper-mjlab/src/mjlab/tasks/tracking/config/pupper').glob('*.py')),
        ROOT/'pupper-mjlab/src/mjlab/tasks/pupper/deployment_contract.py',ROOT/'pupper-mjlab/src/mjlab/tasks/pupper/locomotion_contact_contract.py']
    sources+=sorted(p for p in (ROOT/'pupper-mjlab/src/mjlab/asset_zoo/robots/pupper_v3').rglob('*') if p.is_file() and p.suffix in {'.py','.xml','.obj','.stl'})
    hashes={str(p.relative_to(ROOT)):sha(p) for p in sources}
    plan=dict(protocol='shipped_jump_reference_physics_v1',screening_only=True,hardware_validated=False,
        source_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),source_sha256=hashes,
        policy=dict(path=str(args.policy.resolve()),sha256=sha(args.policy),jump_slot=slot),driver_limits=dict(path=str(args.driver_limits.resolve()),sha256=sha(args.driver_limits),contract=driver),
        seed=args.seed,initial_conditions=initial,commands=commands,slot_start_phase_s=args.slot_start,slot_start_elapsed_s=2+args.slot_start,hop=False,
        init_duration_s=2,fade_in_duration_s=2,command_start_elapsed_s=4.2,inference_hz=52,physics_hz=HZ,action_delay_s=.016,recovery_s=args.recovery_seconds,
        contact_criterion=dict(all_robot_ground_geometries=True,max_distance_m=CONTACT_DISTANCE_M,force_filter=False,
            explanation='Any robot-ground pair within1um counts as contact, including hulls and weak contact; active constraint and normal force recorded separately.'),
        flight_criteria=dict(min_duration_s=MIN_FLIGHT_S,min_rise_after_liftoff_m=MIN_RISE_M,min_takeoff_vertical_velocity_m_s=MIN_TAKEOFF_VELOCITY_M_S,requires_grounded_before=True,landing_recorded_separately=True),
        recovery_criteria=dict(last_second_complete=True,max_tilt_rad=.35,rms_angular_velocity_rad_s=2.,zero_torso_contact=True,final_grounded=True),
        limitations=['Single nominal full-hull model, four paired initial poses only; no domain randomization or hardware validation.',
            'Preplanned immutable reference slot bypasses gamebutton/ROS scheduling; no claim those pathways were tested.',
            'Native physics520Hz/inference52Hz differ from training250Hz/50Hz.',
            'Flight thresholds are explicit engineering evidence screens, not course acceptance criteria.',
            'Only actual jump table at vx0 and0.3; running-hop branch and altered reference tables excluded.'])
    write(output/'plan.json',plan)
    if args.plan_only:return
    write(output/'status.json',dict(status='running'))
    try:
        binary,build=screen.build_controller(output);results={}
        for command in commands:
            tag='stand' if command[0]==0 else 'forward03'
            for scheduled in (False,True):
                name=tag+('-jump' if scheduled else '-gait');directory=output/name;directory.mkdir()
                write(output/'status.json',dict(status='running',current=name,completed=list(results)))
                results[name]=evaluate(screen,shared,binary,args.policy,metadata,initial,command,scheduled,args.slot_start,args.recovery_seconds,lower,upper,directory)
                print(json.dumps(dict(case=name,failures=results[name]['failures'],replicas=[dict(replica=x['replica'],flight=x['flight_and_landing_evidence'],recovery=x['recovery']['meets_stability_screen']) for x in results[name]['replicas']])),flush=True)
        if hashes!={str(p.relative_to(ROOT)):sha(p) for p in sources} or sha(args.policy)!=plan['policy']['sha256'] or sha(args.driver_limits)!=plan['driver_limits']['sha256']:raise RuntimeError('source/config/policy changed during run')
        if len({x['model_sha256'] for x in results.values()})!=1:raise RuntimeError('paired compiled models differ')
        paired=[]
        for tag in ('stand','forward03'):
            control=results[tag+'-gait'];jump=results[tag+'-jump']
            with np.load(output/(tag+'-gait')/'trace.npz') as ctrace, np.load(output/(tag+'-jump')/'trace.npz') as jtrace:
                cmask=ctrace['time_s']<=2+args.slot_start;jmask=jtrace['time_s']<=2+args.slot_start
                if not np.array_equal(ctrace['replica'][cmask],jtrace['replica'][jmask]) or not np.array_equal(ctrace['time_s'][cmask],jtrace['time_s'][jmask]):raise RuntimeError('paired pre-slot trace coverage differs')
                pre_slot_error=float(np.max(np.abs(ctrace['root_position'][cmask]-jtrace['root_position'][jmask])))
                if pre_slot_error>1e-12:raise RuntimeError('paired states differ before scheduled jump')
            for c,j in zip(control['replicas'],jump['replicas']):
                paired.append(dict(command_tag=tag,replica=j['replica'],pre_slot_max_root_position_difference_m=pre_slot_error,jump_flight_and_landing=j['flight_and_landing_evidence'],gait_flight_and_landing=c['flight_and_landing_evidence'],
                    jump_recovery=j['recovery']['meets_stability_screen'],gait_recovery=c['recovery']['meets_stability_screen'],
                    jump_peak_com_rise_m=j['peak_com_rise_from_baseline_m'],gait_peak_com_rise_m=c['peak_com_rise_from_baseline_m'],
                    extra_peak_com_rise_m=j['peak_com_rise_from_baseline_m']-c['peak_com_rise_from_baseline_m'] if j['peak_com_rise_from_baseline_m'] is not None and c['peak_com_rise_from_baseline_m'] is not None else None))
        report=dict(protocol=plan['protocol'],hardware_validated=False,policy_sha256=plan['policy']['sha256'],paired=paired,
            cases=results,build_command=build,binary_sha256=sha(binary),plan_sha256=sha(output/'plan.json'),source_hashes_unchanged=True)
        write(output/'report.json',report);write(output/'status.json',dict(status='completed',report_sha256=sha(output/'report.json')))
    except BaseException as error:
        write(output/'status.json',dict(status='error',error_type=type(error).__name__,error=str(error)))
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path,default=ROOT/'pupper_gait_deploy/policies/default_policy.json')
    parser.add_argument('--driver-limits',type=Path,default=ROOT/'foundation/configs/class-driver-joint-limits.json')
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--replicas',type=int,default=4);parser.add_argument('--seed',type=int,default=101)
    parser.add_argument('--slot-start',type=float,default=3.);parser.add_argument('--recovery-seconds',type=float,default=3.);parser.add_argument('--plan-only',action='store_true')
    run(parser.parse_args())
