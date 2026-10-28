#!/usr/bin/env python3
"""Bounded CPU screen of native exported gait policies; no robot, training or promotion.

Production RTNeural/reference/filter headers run at52Hz with explicit startup,
history and command plumbing. Native MuJoCo nominal dynamics run at520Hz. Four
paired initial conditions are not domain randomization or hardware validation.
"""
from __future__ import annotations
import argparse
from collections import deque
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
NAMES=[f'leg_{leg}_{j}' for leg in ('front_r','front_l','back_r','back_l') for j in (1,2,3)]
COMMANDS=[[0,0,0],[.3,0,0],[.49,0,0],[.51,0,0],[.6,0,0],[.7,0,0],
          [.51,0,0],[.49,0,0],[0,0,0],[-.3,0,0],[-.7,0,0],
          [0,0,1],[0,0,-1],[0,.3,0],[0,-.3,0],[0,0,0]]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    loaded=importlib.util.module_from_spec(spec);sys.modules[name]=loaded;spec.loader.exec_module(loaded)
    return loaded


def build_controller(output):
    source=ROOT/'foundation/cpp/export_locomotion_controller.cpp'
    rt=ROOT/'pupperv3-monorepo/ros2_ws/src/neural_controller/modules/RTNeural'
    binary=output/'export_locomotion_controller'
    include=output/'include';include.mkdir()
    (include/'neural_controller').symlink_to(ROOT/'pupper_gait_deploy/src',target_is_directory=True)
    command=['clang++','-O2','-std=c++17','-DRTNEURAL_DEFAULT_ALIGNMENT=16','-DRTNEURAL_USE_EIGEN=1',
             f'-I{rt}',f'-I{rt}/modules/Eigen',f'-I{ROOT}/pupper_gait_deploy/src',f'-I{include}',str(source),'-o',str(binary)]
    subprocess.run(command,check=True)
    return binary,command


def validate_inputs(policies,commands,driver_limits):
    validator=module(ROOT/'pupper_gait_deploy/download_policy.py','export_screen_validator')
    values=np.asarray(commands,dtype=float)
    if values.ndim!=2 or values.shape[1]!=3 or not len(values) or not np.isfinite(values).all():
        raise ValueError('commands must be a nonempty finite N×3 array')
    result={}
    for label,path in policies.items():
        d=validator.validate(path)
        if d.get('contract_version')!=2 or d.get('single_observation_size')!=48 or 'motion' in d:
            raise ValueError(f'{label}: only fixed v2 locomotion exports are supported')
        if d['gait_reference']['joint_names']!=NAMES or not isinstance(d['action_scale'],(float,int)) or d['action_scale']<=0:
            raise ValueError(f'{label}: canonical joint names and positive scalar action scale required')
        if not isinstance(d['kp'],(int,float)) or not isinstance(d['kd'],(int,float)):
            raise ValueError('screen requires scalar exported PD gains')
        for j,axis in enumerate(('vx','vy','wz')):
            lo,hi=d['command_clip'][axis]
            if np.any(values[:,j]<lo) or np.any(values[:,j]>hi):
                raise ValueError(f'{label}: commands exceed exported {axis} range; no silent clipping in matched screen')
        result[label]=d
    from mjlab.tasks.pupper.deployment_contract import load_driver_target_limits
    limits,contract=load_driver_target_limits(driver_limits)
    lower=np.array([limits[n][0] for n in NAMES]);upper=np.array([limits[n][1] for n in NAMES])
    return result,lower,upper,contract


def initial_conditions(default,replicas,seed):
    """Nominal first replica, then paired small pose perturbations; no model changes."""
    rng=np.random.default_rng(seed);result=[]
    for i in range(replicas):
        # Explicit engineering screen ranges, not claims about training distribution.
        roll,pitch,yaw=(0.,0.,0.) if i==0 else (*rng.uniform(-.02,.02,2),rng.uniform(-np.pi,np.pi))
        cr,cp,cy=np.cos(np.array([roll,pitch,yaw])/2);sr,sp,sy=np.sin(np.array([roll,pitch,yaw])/2)
        quat=[cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]
        offsets=np.zeros(12) if i==0 else rng.uniform(-.01,.01,12)
        result.append(dict(replica=i,root_position=[0.,0.,.18],root_quaternion=quat,
                           initial_rpy=[float(roll),float(pitch),float(yaw)],joint_offsets_rad=offsets.tolist(),
                           joint_position=(np.asarray(default)+offsets).tolist(),joint_velocity=[0.]*12))
    return result


class Host:
    def __init__(self,binary,policy,replicas,log):
        self.log=log.open('w')
        self.proc=subprocess.Popen([str(binary),str(policy),str(replicas),'--no-jump'],stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
    def request(self,value):
        self.proc.stdin.write(json.dumps(value,allow_nan=False)+'\n');self.proc.stdin.flush()
        line=self.proc.stdout.readline()
        if not line: raise RuntimeError('native controller exited; inspect controller.log')
        return json.loads(line)
    def close(self):
        self.proc.stdin.close()
        try:
            if self.proc.wait(timeout=10): raise RuntimeError('native controller exited unsuccessfully')
        finally:self.log.close()


def summary(rows,failures,commands):
    reports=[]
    for segment,command in enumerate(commands):
        selected=[r for r in rows if r['segment']==segment and r['scored']]
        record=dict(segment=segment,command=command,samples=len(selected),
                    represented_replicas=sorted({r['replica'] for r in selected}))
        if selected:
            v=np.array([r['velocity'] for r in selected]);c=np.array([r['effective_command'] for r in selected])
            slips=[speed for r in selected for speed,contact in zip(r['foot_speed_xy'],r['foot_contact']) if contact]
            record.update(mean_velocity_body=v.mean(0).tolist(),mean_effective_command=c.mean(0).tolist(),
                mean_xy_tracking_error_m_s=float(np.linalg.norm(v[:,:2]-c[:,:2],axis=1).mean()),
                mean_abs_yaw_error_rad_s=float(np.abs(v[:,2]-c[:,2]).mean()),
                mean_torque_squared_nm2=float(np.square([r['torque'] for r in selected]).mean()),
                mean_contact_slip_m_s=float(np.mean(slips)) if slips else None,
                controller_clipped_joint_fraction=float(np.mean([r['controller_clipped'] for r in selected])),
                driver_clipped_joint_fraction=float(np.mean([r['driver_clipped'] for r in selected])),
                min_torso_clearance_m=min(r['torso_clearance'] for r in selected),
                torso_penetration_state_fraction=float(np.mean([r['torso_clearance']<0 for r in selected])),
                max_tilt_rad=max(r['tilt'] for r in selected))
        reports.append(record)
    return dict(segments=reports,failures=failures,completed_all_replicas=not failures,
                aggregate_comparison_withheld=bool(failures),
                aggregation_note='Per-segment surviving samples are shown, with replica coverage. No survivor-only aggregate ranking after a failure.')


def evaluate_policy(binary,path,metadata,initial,commands,seconds,warmup,lower,upper,output,make_model):
    import mujoco
    model,contact_contract=make_model('full_hulls')
    mujoco.mj_saveModel(model,str(output/'model.mjb'),None)
    body=model.body('base_link').id
    qadr=model.jnt_qposadr[[model.joint(n).id for n in NAMES]]
    vadr=model.jnt_dofadr[[model.joint(n).id for n in NAMES]]
    controls=np.array([int(np.flatnonzero(model.actuator_trnid[:,0]==model.joint(n).id)[0]) for n in NAMES])
    root_joint=int(np.flatnonzero(model.jnt_type==mujoco.mjtJoint.mjJNT_FREE)[0])
    root_q,root_v=int(model.jnt_qposadr[root_joint]),int(model.jnt_dofadr[root_joint])
    sites=[model.site(f'leg_{leg}_3_foot_site').id for leg in ('front_r','front_l','back_r','back_l')]
    foot_bodies=[int(model.site_bodyid[s]) for s in sites]
    torso=model.geom('base_link_hull').id;mesh=int(model.geom_dataid[torso]);adr=int(model.mesh_vertadr[mesh]);count=int(model.mesh_vertnum[mesh])
    vertices=model.mesh_vert[adr:adr+count].copy()
    terrain=model.geom('terrain').id
    data=[];delayed=[]
    for init in initial:
        d=mujoco.MjData(model);d.qpos[root_q:root_q+3]=init['root_position'];d.qpos[root_q+3:root_q+7]=init['root_quaternion']
        d.qpos[qadr]=init['joint_position'];mujoco.mj_forward(model,d);data.append(d)
        target=np.clip(d.qpos[qadr],lower,upper);delayed.append(deque([(-1.,target.copy()),(0.,target.copy())]))
    host=Host(binary,path,len(initial),output/'controller.log')
    active=[True]*len(initial);outputs=[None]*len(initial);rows=[];failures=[]
    unstable_warnings=[mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,
                       mujoco.mjtWarning.mjWARN_BADQACC,mujoco.mjtWarning.mjWARN_BADCTRL]
    activation_ticks=round(4.2*520);segment_ticks=round(seconds*520)
    total_ticks=activation_ticks+len(commands)*segment_ticks
    try:
        for tick in range(total_ticks):
            t=tick/520;segment=(tick-activation_ticks)//segment_ticks
            requested=commands[segment] if segment>=0 else [0,0,0]
            if tick>=1040 and (tick-1040)%10==0:
                states=[]
                for i,d in enumerate(data):
                    quat=d.qpos[root_q+3:root_q+7]
                    tilt=float(np.arccos(np.clip(1-2*(quat[1]**2+quat[2]**2),-1,1)))
                    if active[i] and (not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all() or tilt>1.):
                        failures.append(dict(replica=i,time_s=t,segment=int(segment),tilt_rad=tilt if np.isfinite(tilt) else None,
                            reason='nonfinite state or gemini_walk inherited 1.0rad tilt stop'))
                        active[i]=False
                    states.append(dict(elapsed_s=t,command=requested,joint_pos=d.qpos[qadr].tolist(),
                        angular_velocity=d.qvel[root_v+3:root_v+6].tolist(),quaternion=quat.tolist()) if active[i] else None)
                if not any(active):break
                outputs=host.request(dict(op='infer',states=states))
            for i,d in enumerate(data):
                if not active[i]:continue
                if tick<1040:
                    a=t/2;target=(1-a)*np.array(initial[i]['joint_position'])+a*np.array(metadata['default_joint_pos'])
                    kp,kd=7.5,.25
                else:
                    target=np.asarray(outputs[i]['target']);kp,kd=metadata['kp'],metadata['kd']
                final=np.clip(target,lower,upper)
                delay=delayed[i];delay.append((t,final.copy()));cutoff=t-.016
                while len(delay)>2 and delay[1][0]<=cutoff:delay.popleft()
                t0,v0=delay[0];t1,v1=delay[1];fraction=np.clip((cutoff-t0)/(t1-t0),0,1)
                d.ctrl[controls]=(1-fraction)*v0+fraction*v1
                model.actuator_gainprm[controls,0]=kp;model.actuator_biasprm[controls,1]=-kp;model.actuator_biasprm[controls,2]=-kd
                if tick>=1040 and (tick-1040)%10==0:
                    out=outputs[i];foot_contact=[False]*4
                    for contact in d.contact:
                        a,b=int(contact.geom1),int(contact.geom2)
                        if terrain not in (a,b):continue
                        other=b if a==terrain else a;cb=int(model.geom_bodyid[other])
                        for j,fb in enumerate(foot_bodies):
                            if cb==fb:foot_contact[j]=True
                    speeds=[]
                    for site in sites:
                        velocity=np.zeros(6);mujoco.mj_objectVelocity(model,d,mujoco.mjtObj.mjOBJ_SITE,site,velocity,0)
                        speeds.append(float(np.linalg.norm(velocity[3:5])))
                    quat=d.qpos[root_q+3:root_q+7]
                    linear=d.xmat[body].reshape(3,3).T@d.qvel[root_v:root_v+3]
                    clearance=float(np.min(vertices@d.geom_xmat[torso].reshape(3,3).T+d.geom_xpos[torso],axis=0)[2])
                    rows.append(dict(replica=i,time_s=t,segment=int(segment),scored=segment>=0 and (tick-activation_ticks)%segment_ticks/520>=warmup,
                        velocity=[float(linear[0]),float(linear[1]),float(d.qvel[root_v+5])],
                        command=requested,effective_command=out['effective_command'],torque=d.actuator_force[controls].tolist(),
                        foot_contact=foot_contact,foot_speed_xy=speeds,torso_clearance=clearance,
                        tilt=float(np.arccos(np.clip(1-2*(quat[1]**2+quat[2]**2),-1,1))),
                        before_clip=out['before_clip'],target=target.tolist(),driver_target=final.tolist(),
                        controller_clipped=(np.abs(np.asarray(out['before_clip'])-target)>1e-6).tolist(),
                        driver_clipped=(np.abs(final-target)>1e-6).tolist(),
                        root_position=d.qpos[root_q:root_q+3].tolist(),root_quaternion=quat.tolist(),joint_position=d.qpos[qadr].tolist()))
                mujoco.mj_step(model,d)
                # MuJoCo can reset an unstable state internally. Never count that
                # automatic recovery as an upright successful continuation.
                warnings=[str(w) for w in unstable_warnings if d.warning[w].number]
                if warnings:
                    failures.append(dict(replica=i,time_s=(tick+1)/520,segment=int(segment),
                        reason='MuJoCo numerical instability warning; possible internal reset',warnings=warnings))
                    active[i]=False
    finally:host.close()
    arrays={key:np.array([r[key] for r in rows]) for key in rows[0]} if rows else {'time_s':np.array([])}
    np.savez_compressed(output/'trace.npz',**arrays)
    report=summary(rows,failures,commands)
    report.update(policy_sha256=sha(path),model_sha256=sha(output/'model.mjb'),contact_contract=contact_contract,
                  collision_masks=[dict(name=model.geom(g).name,contype=int(model.geom_contype[g]),conaffinity=int(model.geom_conaffinity[g])) for g in range(model.ngeom) if model.geom(g).name.endswith(('_hull','_collision'))],
                  mass_kg=float(model.body_mass.sum()),trace_sha256=sha(output/'trace.npz'))
    (output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def run(args):
    policies={}
    for item in args.policy:
        label,path=item.split('=',1)
        if not label.replace('-','').replace('_','').isalnum() or label in policies:raise ValueError('unique safe policy labels required')
        policies[label]=Path(path).expanduser().resolve()
    if not args.no_jump:raise ValueError('--no-jump required: this screen does not measure jump capability')
    if (not 1<=args.replicas<=32 or not np.isfinite([args.seconds,args.warmup]).all()
            or args.seconds<=args.warmup or args.warmup<0):raise ValueError('invalid replica count/timing')
    commands=json.loads(args.commands.read_text()) if args.commands else COMMANDS
    metadata,lower,upper,driver_contract=validate_inputs(policies,commands,args.driver_limits)
    initial=initial_conditions(next(iter(metadata.values()))['default_joint_pos'],args.replicas,args.seed)
    if any(np.any(np.array(x['joint_position'])<lower) or np.any(np.array(x['joint_position'])>upper) for x in initial):raise ValueError('initial conditions exceed class driver bounds')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    helper=ROOT/'foundation/scripts/check_spider_handoff.py'
    sources=[Path(__file__).resolve(),ROOT/'foundation/cpp/export_locomotion_controller.cpp',helper,
             ROOT/'pupper_gait_deploy/src/neural_controller.cpp',ROOT/'pupper_gait_deploy/config.yaml',
             *sorted((ROOT/'pupper_gait_deploy/src').glob('*.hpp')),
             *sorted((ROOT/'pupper-mjlab/src/mjlab/tasks/tracking/config/pupper').glob('*.py')),
             ROOT/'pupper-mjlab/src/mjlab/tasks/pupper/deployment_contract.py',
             ROOT/'pupper-mjlab/src/mjlab/asset_zoo/robots/pupper_v3/pupper_constants.py']
    hashes={str(p.relative_to(ROOT)):sha(p) for p in sources}
    plan=dict(protocol='native_export_locomotion_screen_v1',screening_only=True,hardware_validated=False,promotion_authorized=False,
        seed=args.seed,replicas=args.replicas,initial_conditions=initial,commands=commands,seconds=args.seconds,warmup=args.warmup,
        policy_files={label:dict(path=str(path),sha256=sha(path),metadata={k:v for k,v in metadata[label].items() if k!='layers'}) for label,path in policies.items()},
        driver_limits=dict(path=str(args.driver_limits.resolve()),sha256=sha(args.driver_limits),contract=driver_contract),
        sources=hashes,physics_hz=520,inference_hz=52,init_duration_s=2,fade_in_duration_s=2,activation_wait_s=4.2,
        initialization_gains=dict(kp=7.5,kd=.25),action_delay_s=.016,tilt_stop_rad=1.,
        dynamics='nominal full-hull ground contact, identical compiled model for all policies',
        no_jump=True,jump_capability='unmeasured; all jump slots +infinity, game buttons and reference requests disabled',
        command_caps=dict(gemini_speed_cap=1.5,per_policy_command_clip=True),
        limitations=['No mass/friction/gain randomization, sensor noise/latency, encoder bias, external pushes or hardware tests.',
            'Initial pose perturbations only; four replicas do not establish robustness.',
            'Production RTNeural/reference/filter headers; explicit transcription of ROS lifecycle/history plumbing, not ROS execution.',
            'Native CPU solver520Hz differs from training solver250Hz; inference52Hz differs from training50Hz.',
            'Slip uses foot-site XY speed when any lower-leg geometry contacts ground, including hulls.',
            'Stop criterion is configured Gemini tilt stop; metric is controller-stop incidents, not every possible physical failure.',
            'No jumps or automatic controller/policy promotion.'])
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    status=dict(status='planned',policies={});(output/'status.json').write_text(json.dumps(status,indent=2)+'\n')
    if args.plan_only:return
    binary,command=build_controller(output);shared=module(helper,'export_screen_shared')
    try:
        for label,path in policies.items():
            directory=output/label;directory.mkdir();status.update(status='running',current=label)
            (output/'status.json').write_text(json.dumps(status,indent=2)+'\n')
            report=evaluate_policy(binary,path,metadata[label],initial,commands,args.seconds,args.warmup,lower,upper,directory,shared.make_model)
            status['policies'][label]=dict(metrics=str(directory/'metrics.json'),sha256=sha(directory/'metrics.json'),failures=len(report['failures']))
            print(json.dumps(dict(policy=label,completed=report['completed_all_replicas'],failures=report['failures'])),flush=True)
        changed={str(p.relative_to(ROOT)):sha(p) for p in sources}!=hashes
        inputs_changed=any(sha(path)!=plan['policy_files'][label]['sha256'] for label,path in policies.items()) or sha(args.driver_limits)!=plan['driver_limits']['sha256']
        models={json.loads((output/label/'metrics.json').read_text())['model_sha256'] for label in policies}
        status.update(status='invalidated' if changed or inputs_changed or len(models)!=1 else 'complete',source_changed=changed,inputs_changed=inputs_changed,identical_models=len(models)==1,compiler_command=command)
        if status['status']!='complete':raise RuntimeError('source/input changed or model contracts differ; screen invalidated')
    except BaseException as exc:
        status.update(status='error',error=str(exc));raise
    finally:(output/'status.json').write_text(json.dumps(status,indent=2)+'\n')


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--policy',action='append',required=True,help='LABEL=/absolute/export.json; repeat for paired policies')
    p.add_argument('--driver-limits',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--commands',type=Path);p.add_argument('--replicas',type=int,default=4);p.add_argument('--seed',type=int,default=101)
    p.add_argument('--seconds',type=float,default=10);p.add_argument('--warmup',type=float,default=2)
    p.add_argument('--no-jump',action='store_true');p.add_argument('--plan-only',action='store_true')
    return p


if __name__=='__main__':run(parser().parse_args())
