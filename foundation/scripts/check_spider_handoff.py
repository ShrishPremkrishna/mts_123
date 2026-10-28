#!/usr/bin/env python3
"""Continuous Spider entry -> walking -> stop -> exit integration diagnostic.

No physical robot, ROS service, training job or controller installation is used.
The host controller uses production RTNeural/clip/gait/quaternion/filter headers.
Lifecycle plumbing is an explicit transcription, not a ROS execution test.
"""
from __future__ import annotations
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import mujoco
import yaml

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_controller(output):
    source = ROOT/'foundation/cpp/handoff_controller.cpp'
    rt = ROOT/'pupperv3-monorepo/ros2_ws/src/neural_controller/modules/RTNeural'
    binary = output/'handoff_controller'
    include=output/'include';include.mkdir()
    (include/'neural_controller').symlink_to(ROOT/'pupper_gait_deploy/src',target_is_directory=True)
    command = ['clang++','-O2','-std=c++17','-DRTNEURAL_DEFAULT_ALIGNMENT=16',
               '-DRTNEURAL_USE_EIGEN=1',f'-I{rt}',f'-I{rt}/modules/Eigen',
               f'-I{ROOT}/pupper_gait_deploy/src',f'-I{include}',str(source),'-o',str(binary)]
    subprocess.run(command, check=True)
    return binary, command


def make_model(contact_model):
    from mjlab.entity import Entity
    from mjlab.tasks.tracking.config.pupper.env_cfgs import pupper_tracking_env_cfg
    from mjlab.tasks.tracking.config.pupper.contact_contract import enable_tracking_ground_hulls
    from mjlab.tasks.pupper.locomotion_contact_contract import enable_locomotion_torso_contact
    from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import get_spec, PUPPER_MUJOCO_OPTIONS
    from mjlab.sim import MujocoCfg
    cfg = pupper_tracking_env_cfg(play=True)
    if contact_model == 'full_hulls':
        contract = enable_tracking_ground_hulls(cfg)
    else:
        cfg.scene.entities['robot'].spec_fn = get_spec
        contract = enable_locomotion_torso_contact(cfg)
    spec = Entity(cfg.scene.entities['robot']).spec
    spec.worldbody.add_geom(name='terrain', type=mujoco.mjtGeom.mjGEOM_PLANE,
                            size=[5,5,.1], contype=1, conaffinity=1, friction=[1,.005,.0001])
    model = spec.compile()
    MujocoCfg(**PUPPER_MUJOCO_OPTIONS).apply(model)
    # Native integration at the controller manager's actual 520 Hz update rate;
    # inference is every 10 updates, per config.yaml. This deliberately differs
    # from the 250 Hz training solver and is recorded in the result scope.
    model.opt.timestep = 1/520
    return model, contract


def validate_physical_variant(value):
    """Explicit deterministic mechanics within the inspected training ranges.

    Shared foot-end material is a separate contact-model sensitivity, because
    native lower-leg hulls overlap foot spheres but do not share their priority.
    This is not a draw from the GPU randomization distribution.
    """
    import re
    required={'schema','name','base_com_offset_m','uniform_density_scale','foot_friction','foot_material'}
    if not isinstance(value,dict) or set(value)!=required or type(value['schema']) is not int or value['schema']!=1:
        raise ValueError('Physical variant requires the exact schema1 fields')
    if not isinstance(value['name'],str) or not re.fullmatch(r'[a-z][a-z0-9_-]{0,63}',value['name']):
        raise ValueError('Invalid physical variant name')
    def number(x):
        return type(x) in (int,float) and np.isfinite(x)
    com=value['base_com_offset_m']
    if not isinstance(com,list) or len(com)!=3 or any(not number(x) or abs(x)>limit for x,limit in zip(com,(.02,.005,.005))):
        raise ValueError('COM offsets must stay within x +/- .02m and y/z +/- .005m')
    scale=value['uniform_density_scale']
    if not number(scale) or not np.exp(-.1054)<=scale<=np.exp(.2624):
        raise ValueError('Density scale must stay within the configured exp(2*alpha) range')
    friction=value['foot_friction']
    if friction is not None and (not number(friction) or not .6<=friction<=1.4):
        raise ValueError('Foot friction must be absent or within [.6,1.4]')
    if value['foot_material'] not in ('sphere_only','foot_end_union'):
        raise ValueError('Unknown foot material scope')
    if value['foot_material']=='foot_end_union' and friction is None:
        raise ValueError('Shared foot-end material requires an explicit friction value')
    return dict(value)


def apply_physical_variant(model, value):
    """Modify a new model before creating live simulation state; log actual arrays."""
    value=validate_physical_variant(value)
    base=model.body('base_link').id
    sphere_ids=[g for g in range(model.ngeom) if model.geom(g).name.startswith('leg_') and model.geom(g).name.endswith('_3_collision')]
    hull_ids=[g for g in range(model.ngeom) if model.geom(g).name.startswith('leg_') and model.geom(g).name.endswith('_3_hull')]
    if len(sphere_ids)!=4 or (value['foot_material']=='foot_end_union' and len(hull_ids)!=4):
        raise ValueError('Expected exactly four foot spheres and four foot-end hulls')
    def snapshot():
        return dict(total_mass_kg=float(model.body_mass.sum()),
            bodies=[dict(name=model.body(b).name,mass_kg=float(model.body_mass[b]),
                com_body_m=model.body_ipos[b].tolist(),principal_inertia_kg_m2=model.body_inertia[b].tolist(),
                inertial_quaternion_wxyz=model.body_iquat[b].tolist()) for b in range(1,model.nbody)],
            foot_geoms=[dict(name=model.geom(g).name,friction=model.geom_friction[g].tolist(),
                priority=int(model.geom_priority[g])) for g in sphere_ids+hull_ids])
    before=snapshot()
    model.body_mass[1:]*=value['uniform_density_scale']
    model.body_inertia[1:]*=value['uniform_density_scale']
    model.body_ipos[base]+=np.asarray(value['base_com_offset_m'])
    if value['foot_friction'] is not None:
        ids=sphere_ids+(hull_ids if value['foot_material']=='foot_end_union' else [])
        model.geom_friction[ids,0]=value['foot_friction']
        if value['foot_material']=='foot_end_union':model.geom_priority[ids]=1
    # Recompute derived mass/inertia constants before any initial state exists.
    scratch=mujoco.MjData(model);mujoco.mj_setConst(model,scratch)
    return dict(requested=value,before=before,realized=snapshot(),
        scope=('Deterministic mechanical corner using training parameter ranges; independent per-body training samples, gains, encoder bias and observation noise are not reproduced.'
               if value['foot_material']=='sphere_only' else
               'Separate foot-end material sensitivity: sphere and overlapping lower-leg hull share priority1 and sliding friction. This changes the native contact-material assumption and is not exact GPU domain randomization.'))


def default_return_stages():
    return [('default_activation','default',4.2,[0,0,0],''),
            ('default_forward','default',2,[.10,0,0],''),
            ('default_stop','default',2,[0,0,0],'')]


def validate_single_return_descriptor(path, policy_path, *, later_return_from_lying=False):
    if path is None:
        raise ValueError('Finite default return requires --trick-descriptor with a standing endpoint')
    content=path.read_bytes();descriptor=yaml.safe_load(content)
    expected_end='lying' if later_return_from_lying else 'standing'
    if not isinstance(descriptor,dict) or descriptor.get('type')!='policy' or descriptor.get('start_pose')!='standing' or descriptor.get('end_pose')!=expected_end:
        raise ValueError('Finite default return requires a standing-to-standing policy descriptor, or explicit later return from a declared lying endpoint')
    if not isinstance(descriptor.get('policy'),str) or (path.parent/descriptor['policy']).resolve()!=policy_path.resolve():
        raise ValueError('Descriptor policy path does not match the evaluated input')
    return content


def validate_interactive_return_descriptor(path, policy_path):
    if path is None:raise ValueError('Interactive Sit requires --trick-descriptor')
    content=path.read_bytes();d=yaml.safe_load(content)
    if (not isinstance(d,dict) or d.get('type')!='policy' or d.get('start_pose')!='standing'
            or d.get('end_pose')!='sitting' or d.get('exit_clip')!='stand_up'
            or not isinstance(d.get('actions'),dict) or not {'shake_left','shake_right','stand_up'}<=d['actions'].keys()):
        raise ValueError('Interactive Sit requires sitting endpoint and explicit shake/standing-exit actions')
    if not isinstance(d.get('policy'),str) or (path.parent/d['policy']).resolve()!=policy_path.resolve():
        raise ValueError('Descriptor policy path does not match the evaluated input')
    return content


def interactive_sit_stages(trick):
    clips={c['name']:c for c in trick['trick']['clips']};fps=float(trick['trick']['fps'])
    if set(clips)!={'sit_down','sit_idle','shake_left','shake_right','stand_up'} or not np.isfinite(fps) or fps<=0:
        raise ValueError('Interactive Sit requires the reviewed five-clip contract')
    if (not clips['sit_idle'].get('loop') or clips['stand_up'].get('loop') or clips['stand_up'].get('next')
            or any(clips[n].get('loop') or clips[n].get('next')!='sit_idle' for n in ('sit_down','shake_left','shake_right'))):
        raise ValueError('Interactive Sit requires idle loops, shake returns and a finite standing exit')
    durations={name:(c['end']-c['start']-1)/fps for name,c in clips.items()}
    if any(v<=0 or not np.isfinite(v) for v in durations.values()):raise ValueError('Invalid interactive clip duration')
    # Queued actions begin on the next idle boundary. Allow one full idle loop
    # plus action duration, then observe idle/terminal hold before continuing.
    return [('sit_entry','trick',.5+1+durations['sit_down']+.3,[0,0,0],'sit_down'),
            ('sit_idle_before_left','trick',2.,[0,0,0],''),
            ('shake_left','trick',durations['sit_idle']+durations['shake_left']+.3,[0,0,0],'shake_left'),
            ('sit_idle_before_right','trick',2.,[0,0,0],''),
            ('shake_right','trick',durations['sit_idle']+durations['shake_right']+.3,[0,0,0],'shake_right'),
            ('sit_idle_before_exit','trick',2.,[0,0,0],''),
            ('sit_stand_up','trick',durations['sit_idle']+durations['stand_up']+.3,[0,0,0],'stand_up'),
            ('sit_standing_hold','trick',2.,[0,0,0],'')] + default_return_stages()


def stage_plan(trick, single_motion=False, return_default=False):
    if single_motion:
        metadata=trick['trick'];clips=metadata.get('clips')
        if clips:
            if len(clips)!=1 or clips[0].get('loop') or clips[0].get('next'):
                raise ValueError('single-motion mode requires one finite clip without a next clip')
            clip=clips[0];name=clip['name'];frames=clip['end']-clip['start']
        else:
            name='all';frames=len(trick['motion']['joint_pos'])
        duration=(frames-1)/metadata['fps']
        stages=[('motion','trick',.5+1+duration,[0,0,0],name),
                ('terminal_hold','trick',2,[0,0,0],'')]
        return stages + default_return_stages() if return_default else stages
    clips={c['name']:c for c in trick['trick']['clips']};fps=trick['trick']['fps']
    down=(clips['spider_down']['end']-clips['spider_down']['start']-1)/fps
    up=(clips['spider_up']['end']-clips['spider_up']['start']-1)/fps
    stages = [('entry','trick',.5+1+down+.3,[0,0,0],'spider_down'),
            ('walk_activation','walk',4.2,[0,0,0],''),
            ('walk_forward','walk',2,[.10,0,0],''),
            ('walk_stop_1','walk',1,[0,0,0],''),
            ('walk_backward','walk',2,[-.10,0,0],''),
            ('walk_turn','walk',2,[0,0,.2],''),
            ('walk_stop_before_exit','walk',2,[0,0,0],''),
            ('exit','trick',.5+1+up+.3,[0,0,0],'spider_up'),
            ('standing_hold','trick',1,[0,0,0],'')]
    if return_default:
        stages += default_return_stages()
    return stages


def numerical_failures(data):
    warnings=[mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,
              mujoco.mjtWarning.mjWARN_BADQACC,mujoco.mjtWarning.mjWARN_BADCTRL]
    failures=[str(w) for w in warnings if data.warning[w].number]
    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():failures.append('nonfinite state')
    return failures


def contact_point_telemetry(model, data):
    """Exact ground contact-point velocities at the supplied forward-solved state.

    Jacobians include translation, rotation about each body's COM and joint
    motion. Subtract both point velocities, then project onto the contact plane;
    root translation alone cannot identify sliding or rolling.
    """
    contacts=[]
    for contact_id,c in enumerate(data.contact):
        names=[model.geom(int(g)).name for g in (c.geom1,c.geom2)]
        if 'terrain' not in names:continue
        force=np.zeros(6);mujoco.mj_contactForce(model,data,contact_id,force)
        point=np.array(c.pos,copy=True);frame=np.asarray(c.frame).reshape(3,3)
        jacobians=[]
        for geom in (c.geom1,c.geom2):
            jac=np.zeros((3,model.nv));mujoco.mj_jac(model,data,jac,None,point,int(model.geom_bodyid[geom]));jacobians.append(jac)
        relative=(jacobians[1]-jacobians[0])@data.qvel
        normal_speed=float(frame[0]@relative)
        tangential=relative-normal_speed*frame[0]
        contacts.append(dict(geom=names[1-names.index('terrain')],distance_m=float(c.dist),friction=float(c.friction[0]),normal_force_n=float(force[0]),
            position_world_m=point.tolist(),normal_world=frame[0].tolist(),relative_velocity_geom2_minus_geom1_world_m_s=relative.tolist(),
            normal_relative_velocity_m_s=normal_speed,tangential_relative_velocity_world_m_s=tangential.tolist(),tangential_speed_m_s=float(np.linalg.norm(tangential))))
    return contacts


def sample_contact_state(model, data, scratch):
    # Solve on an isolated exact copy so contact positions, forces and velocities
    # all describe the pre-integration state. Never re-forward live simulation
    # data after stepping or replace its solver warmstart/history.
    mujoco.mj_copyData(scratch,model,data)
    mujoco.mj_forward(model,scratch)
    return contact_point_telemetry(model,scratch)


def inference_event(state, **context):
    # Scenario stage (e.g. walk_forward) and activation stage (fade/policy) are
    # distinct. Preserve both without colliding dict keyword arguments.
    return {**state,**context,'activation_stage':state['stage']}


def run(args):
    angle_limit=float(getattr(args,'max_body_angle_deg',np.degrees(1.5)))
    if not np.isfinite(angle_limit) or not 0<angle_limit<=np.degrees(1.5):
        raise ValueError('Body-angle threshold must be positive and no looser than the1.5rad diagnostic stop')
    single_motion=getattr(args,'single_motion',False)
    interactive_sit=getattr(args,'interactive_sit',False)
    if single_motion and interactive_sit:raise ValueError('Choose finite single motion or interactive Sit, not both')
    default_path=getattr(args,'return_default_policy',None)
    descriptor_path=getattr(args,'trick_descriptor',None)
    later_return_from_lying=getattr(args,'later_return_from_lying',False)
    if later_return_from_lying and (not single_motion or default_path is None or interactive_sit):
        raise ValueError('--later-return-from-lying requires single finite motion and explicit default policy')
    if interactive_sit and default_path is None:raise ValueError('Interactive Sit requires explicit --return-default-policy')
    descriptor_bytes=(validate_interactive_return_descriptor(descriptor_path,args.trick_policy) if interactive_sit else
        validate_single_return_descriptor(descriptor_path,args.trick_policy,later_return_from_lying=later_return_from_lying) if single_motion and default_path is not None else None)
    variant_path=getattr(args,'physical_variant',None)
    variant_bytes=None if variant_path is None else variant_path.read_bytes()
    variant_spec=None if variant_bytes is None else validate_physical_variant(json.loads(variant_bytes))
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    if variant_bytes is not None:(output/'physical-variant.json').write_bytes(variant_bytes)
    if descriptor_bytes is not None:(output/'trick-descriptor.yaml').write_bytes(descriptor_bytes)
    sources = [ROOT/'foundation/cpp/handoff_controller.cpp', Path(__file__).resolve(),
               *sorted((ROOT/'pupper_gait_deploy/src').glob('*.hpp')),
               ROOT/'pupper_gait_deploy/src/neural_controller.cpp', ROOT/'pupper_gait_deploy/config.yaml',
               ROOT/'gemini_pupper/robot/tricks.py']
    sources += [ROOT/'pupper-mjlab/src/mjlab'/p for p in (
        'tasks/tracking/config/pupper/env_cfgs.py','tasks/tracking/config/pupper/contact_contract.py',
        'tasks/pupper/locomotion_contact_contract.py','tasks/pupper/deployment_contract.py',
        'asset_zoo/robots/pupper_v3/pupper_constants.py','entity/entity.py','sim/sim.py')]
    if descriptor_bytes is not None:sources.append(descriptor_path.resolve())
    source_before = {str(p.relative_to(ROOT)):sha(p) for p in sources}
    binary, compile_command = build_controller(output)
    if args.walk_policy is None and not (single_motion or interactive_sit):raise ValueError('--walk-policy required for Spider handoff')
    walk_path=args.walk_policy or args.trick_policy  # second host slot unused in single-motion mode
    # Read each original once. Parsing, preservation and native execution use
    # those exact bytes even if an external process changes the original path.
    policy_paths={'trick':args.trick_policy,'walk':walk_path}
    if default_path is not None: policy_paths['default']=default_path
    policy_bytes={name:path.read_bytes() for name,path in policy_paths.items()}
    policy_before={name:hashlib.sha256(raw).hexdigest() for name,raw in policy_bytes.items()}
    policies={name:json.loads(raw) for name,raw in policy_bytes.items()}
    trick=policies['trick'];walking=policies['walk']
    for name,raw in policy_bytes.items(): (output/f'{name}-policy.json').write_bytes(raw)
    preserved_hashes={name:sha(output/f'{name}-policy.json') for name in policy_bytes}
    if preserved_hashes!=policy_before:raise RuntimeError('Preserved policy identity mismatch')
    from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES
    model, contract = make_model(args.contact_model)
    physical_variant=None if variant_spec is None else apply_physical_variant(model,variant_spec)
    data = mujoco.MjData(model)
    contact_scratch = mujoco.MjData(model)
    mujoco.mj_saveModel(model,str(output/'model.mjb'),None)
    joints = [model.joint(name).id for name in JOINT_NAMES]
    qadr, vadr = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    controls = [int(np.flatnonzero(model.actuator_trnid[:,0] == j)[0]) for j in joints]
    free = int(np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)[0])
    root_q, root_v = int(model.jnt_qposadr[free]), int(model.jnt_dofadr[free])
    body = model.body('base_link').id
    data.qpos[root_q:root_q+7] = [0,0,.18,1,0,0,0]
    data.qpos[qadr] = trick['default_joint_pos'] if single_motion or interactive_sit else trick['motion']['joint_pos'][0]
    mujoco.mj_forward(model,data)
    lower = np.full(12,-np.inf); upper=np.full(12,np.inf)
    if args.driver_limits:
        from mjlab.tasks.pupper.deployment_contract import load_driver_target_limits
        limits,_=load_driver_target_limits(args.driver_limits)
        lower=np.asarray([limits[name][0] for name in JOINT_NAMES])
        upper=np.asarray([limits[name][1] for name in JOINT_NAMES])
        if lower.shape!=(12,) or upper.shape!=(12,) or not np.all(lower<upper): raise ValueError('invalid driver bounds')
    # Exact application timing: entry init/fade plus clip and .3s; walking
    # activation 2+2s plus .2s; bounded commands followed by a 2s zero stop.
    stages=interactive_sit_stages(trick) if interactive_sit else stage_plan(trick,single_motion,default_path is not None)
    host_command=[str(binary),str((output/'trick-policy.json').resolve()),str((output/'walk-policy.json').resolve())]
    if default_path is not None: host_command += ['--return-default-no-jump',str((output/'default-policy.json').resolve())]
    proc=subprocess.Popen(host_command,
                          stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(output/'controller.log').open('w'),text=True,bufsize=1)
    def rpc(request):
        proc.stdin.write(json.dumps(request)+'\n'); proc.stdin.flush()
        line=proc.stdout.readline()
        if not line: raise RuntimeError('controller exited; inspect controller.log')
        return json.loads(line)
    samples=[]; activations=[];clip_requests=[];inference_events=[]; previous_controller=None; activation_time=0.; previous_target=data.qpos[qadr].copy()
    # Interpolate between timestamped old targets to realize exactly 16ms at
    # 520Hz. Buffer persists through all controller switches; initial values are
    # measured standing joints, initialized once at the start of the world.
    delayed=deque([(-1.,previous_target.copy()),(0.,previous_target.copy())])
    failure=None; current_time=0.; tick=0; interval_target_step=0.
    try:
        for stage,controller,duration,command,start_clip in stages:
            if controller!=previous_controller:
                activation_time=current_time
                rpc(dict(op='activate',controller=controller,joint_pos=data.qpos[qadr].tolist(),
                         init_s=.5 if controller=='trick' else 2.,fade_s=1. if controller=='trick' else 2.,
                         start_clip=start_clip,clip_anchor=args.clip_anchor))
                activations.append(dict(stage=stage,controller=controller,policy_sha256=policy_before[controller],time_s=current_time,root_pose=data.qpos[root_q:root_q+7].tolist(),
                                        joint_pos=data.qpos[qadr].tolist(),joint_vel=data.qvel[vadr].tolist(),
                                        physical_state_reset=False,observation_history_reset=True))
                previous_controller=controller
            elif interactive_sit and start_clip:
                response=rpc(dict(op='request_clip',clip=start_clip))
                clip_requests.append(dict(stage=stage,time_s=current_time,clip=start_clip,**response,
                    activation_clock_reset=False,observation_history_reset=False,physical_state_reset=False))
                if not response['accepted']:
                    failure=dict(time_s=current_time,reason='Interactive clip request refused',clip=start_clip);break
            observed_stage_clips=set()
            for _ in range(round(duration*520)):
                quat=data.qpos[root_q+3:root_q+7].copy()
                state=rpc(dict(op='step',elapsed_s=current_time-activation_time,joint_pos=data.qpos[qadr].tolist(),
                    joint_vel=data.qvel[vadr].tolist(),quaternion=quat.tolist(),angular_velocity=data.qvel[root_v+3:root_v+6].tolist(),command=command,
                    audit_inference=getattr(args,'record_inference',False)))
                if state.get('clip'):observed_stage_clips.add(state['clip'])
                target=np.asarray(state['target']); final_target=np.clip(target,lower,upper)
                if state.get('inference') and getattr(args,'record_inference',False):
                    inference_events.append(inference_event(state,time_s=current_time,stage=stage,controller=controller,command=command,
                        measured_joint_pos=data.qpos[qadr].tolist(),measured_joint_vel=data.qvel[vadr].tolist(),
                        previous_controller_target=previous_target.tolist(),driver_target=final_target.tolist()))
                interval_target_step=max(interval_target_step,float(np.max(np.abs(target-previous_target))))
                delayed.append((current_time,final_target.copy())); cutoff=current_time-.016
                while len(delayed)>2 and delayed[1][0]<=cutoff: delayed.popleft()
                t0,v0=delayed[0];t1,v1=delayed[1];a=np.clip((cutoff-t0)/(t1-t0),0,1)
                data.ctrl[controls]=(1-a)*v0+a*v1
                model.actuator_gainprm[controls,0]=state['kp']
                model.actuator_biasprm[controls,1]=-state['kp'];model.actuator_biasprm[controls,2]=-state['kd']
                contacts = sample_contact_state(model,data,contact_scratch) if tick%10==0 else None
                mujoco.mj_step(model,data)
                numerical=numerical_failures(data)
                if numerical:
                    failure=dict(time_s=current_time+1/520,reason='Numerical instability; possible MuJoCo internal reset',details=numerical)
                    break
                if tick%10==0:
                    w,x,y,z=data.qpos[root_q+3:root_q+7]
                    tilt=np.degrees(np.arccos(np.clip(1-2*(x*x+y*y),-1,1)))
                    samples.append(dict(time_s=current_time,contact_state_time_s=current_time,stage=stage,controller=controller,activation_stage=state['stage'],
                        clip=state.get('clip'),frame=state['frame'],holding=state.get('holding'),
                        root_pos=data.qpos[root_q:root_q+3].tolist(),root_quat=data.qpos[root_q+3:root_q+7].tolist(),
                        root_velocity_body=(data.xmat[body].reshape(3,3).T@data.qvel[root_v:root_v+3]).tolist(),
                        joint_pos=data.qpos[qadr].tolist(),joint_vel=data.qvel[vadr].tolist(),
                        target=target.tolist(),driver_target=final_target.tolist(),before_clip=state['before_clip'],
                        command=command,tilt_deg=float(tilt),target_delta_max_rad=interval_target_step,
                        torque_max_nm=float(np.max(np.abs(data.actuator_force[controls]))),contacts=contacts))
                    interval_target_step=0.
                    if not np.isfinite(data.qpos).all() or tilt>angle_limit:
                        failure=dict(time_s=current_time,reason=f'nonfinite state or configured {angle_limit:g}deg body-angle stop'); break
                previous_target=target.copy();tick+=1;current_time=tick/520
            if failure: break
            if interactive_sit and stage in ('sit_entry','shake_left','shake_right'):
                expected='sit_down' if stage=='sit_entry' else stage
                if expected not in observed_stage_clips or state.get('clip')!='sit_idle':
                    failure=dict(time_s=current_time,reason='Interactive action did not play and return to idle',stage=stage);break
            if interactive_sit and stage in ('sit_stand_up','sit_standing_hold'):
                if state.get('clip')!='stand_up' or not state.get('holding'):
                    failure=dict(time_s=current_time,reason='Interactive standing exit did not reach final hold');break
            if stage=='entry' and state.get('clip')!='spider_idle':
                failure=dict(time_s=current_time,reason='entry controller did not confirm spider_idle');break
            if stage=='exit' and not (state.get('clip')=='spider_up' and state.get('holding')):
                failure=dict(time_s=current_time,reason='exit controller did not confirm final hold');break
            if stage=='terminal_hold' and not state.get('holding'):
                failure=dict(time_s=current_time,reason='finite clip did not confirm terminal hold');break
    finally:
        proc.stdin.close();proc.wait(timeout=10)
    traces=output/'trajectory.json';traces.write_text(json.dumps(samples,separators=(',',':'))+'\n')
    if getattr(args,'record_inference',False):
        (output/'inference.json').write_text(json.dumps(inference_events,separators=(',',':'))+'\n')
    np.savez_compressed(output/'trajectory.npz',fps=52,actual_root_pos=np.array([s['root_pos'] for s in samples]),
                        actual_root_quat=np.array([s['root_quat'] for s in samples]),actual_joint_pos=np.array([s['joint_pos'] for s in samples]))
    summaries={}
    for name,*_ in stages:
        rows=[s for s in samples if s['stage']==name]
        if not rows: continue
        default=np.asarray(policies[rows[-1]['controller']]['default_joint_pos'])
        endpoint=(np.asarray(trick['motion']['joint_pos'][rows[-1]['frame']])
                  if rows[-1]['controller']=='trick' and rows[-1]['frame']>=0 else default)
        summaries[name]=dict(samples=len(rows),max_tilt_deg=max(s['tilt_deg'] for s in rows),
            mean_velocity_body=np.mean([s['root_velocity_body'] for s in rows],axis=0).tolist(),
            min_height_m=min(s['root_pos'][2] for s in rows),max_height_m=max(s['root_pos'][2] for s in rows),
            max_joint_speed_rad_s=float(np.max(np.abs([s['joint_vel'] for s in rows]))),
            final_max_joint_speed_rad_s=float(np.max(np.abs(rows[-1]['joint_vel']))),
            final_error_vs_controller_default_max_rad=float(np.max(np.abs(np.asarray(rows[-1]['joint_pos'])-default))),
            final_error_vs_current_reference_max_rad=float(np.max(np.abs(np.asarray(rows[-1]['joint_pos'])-endpoint))),
            max_target_step_rad=max(s['target_delta_max_rad'] for s in rows),
            driver_clipped_joint_fraction=float(np.mean(np.abs(np.array([s['driver_target'] for s in rows])-np.array([s['target'] for s in rows]))>1e-7)),
            last_clip=rows[-1]['clip'],last_frame=rows[-1]['frame'],last_holding=rows[-1]['holding'])
    source_after={str(p.relative_to(ROOT)):sha(p) for p in sources}
    policy_after={name:sha(path) for name,path in policy_paths.items()}
    report=dict(protocol=('finite_lying_later_default_nojump_v1' if later_return_from_lying else
        'interactive_sit_to_default_nojump_v1' if interactive_sit else
        'finite_trick_to_default_nojump_v1' if single_motion and default_path is not None else
        'exported_finite_trick_activation_and_hold_v1' if single_motion else
        'continuous_spider_to_default_nojump_v2' if default_path is not None else 'continuous_exported_controller_spider_handoff_v1'),hardware_validated=False,
        interactive_sit=interactive_sit,clip_requests=clip_requests,later_return_from_lying=later_return_from_lying,
        declared_trick_end_pose=yaml.safe_load(descriptor_bytes)['end_pose'] if descriptor_bytes is not None else None,
        scope='Native MuJoCo diagnostic with production RTNeural and math headers, explicitly transcribed ROS activation plumbing; not a ROS execution or hardware approval.',
        contact_telemetry_contract='exact_preintegration_contact_point_velocity_v1',contact_contract=contract,clip_activation_anchor='selected_clip_first_frame' if args.clip_anchor else 'legacy_network_default',
        physical_variant=physical_variant,
        physical_variant_input_sha256=None if variant_bytes is None else hashlib.sha256(variant_bytes).hexdigest(),
        physical_variant_input_changed=False if variant_path is None else sha(variant_path)!=hashlib.sha256(variant_bytes).hexdigest(),
        physical_state_resets_after_initialization=0,physics_hz=520,controller_inference_hz=52,action_delay_s=.016,
        numerical_instability_monitor='Check BADQPOS/BADQVEL/BADQACC/BADCTRL warnings and finite qpos/qvel after every physics step; stop rather than count automatic recovery as success.',
        maximum_body_angle_deg=angle_limit,
        caveats=['Native CPU solver at520Hz differs from250Hz training Warp solver.',
                 ('Nominal model: no observation noise, IMU delay, encoder bias, parameter randomization or external push.' if physical_variant is None else
                  'Explicit fixed mechanics/material variant; no observation noise, IMU delay, encoder bias, gain randomization or external push. Nominal trained policies and controller limits remain unchanged.'),
                 'Clip completion is controller clock/status, not measured joint settlement; endpoint errors reported separately.',
                 'Finite trick retains its declared lying endpoint through2s terminal hold; a separate later move request directly activates the default controller. No implicit standing-exit clip is invented.' if later_return_from_lying else
                 'Interactive Sit queues left shake, right shake and standing exit through production ClipPlayer with persistent actor history and clock; this does not execute ROS messages.' if interactive_sit else
                 'Single finite clip plus2s hold; no walking handoff tested.' if single_motion and default_path is None else
                 'Default controller returns after the finite terminal hold or Spider exit with its own measured-pose initialization, reset history, no scheduled jumps, 0.1m/s forward command and stop; no ROS services executed.' if default_path is not None else
                 'No final walking-controller activation after standing exit; bounded diagnostic stops in trick standing hold.',
                 'Contacts/forces and Jacobian-derived relative contact-point velocities are sampled on an isolated forward-solved pre-integration state; recorded qpos is after integration, at most1/520s later. No live solver state is changed by telemetry.'],
        trick_descriptor_sha256=hashlib.sha256(descriptor_bytes).hexdigest() if descriptor_bytes is not None else None,
        trick_policy_sha256=policy_before['trick'],walk_policy_sha256=None if single_motion or interactive_sit else policy_before['walk'],
        default_policy_sha256=policy_before.get('default'),policy_hashes_start=policy_before,policy_hashes_end=policy_after,
        preserved_policy_hashes={name:sha(output/f'{name}-policy.json') for name in policy_bytes},
        policy_changed=policy_before!=policy_after,host_command=host_command,
        driver_limits=None if args.driver_limits is None else dict(path=str(args.driver_limits),sha256=sha(args.driver_limits)),
        sources_start=source_before,sources_end=source_after,source_changed=source_before!=source_after,
        compiler_command=compile_command,activations=activations,stage_summaries=summaries,stop=failure,
        trajectory_sha256=sha(traces),completed_sequence=failure is None,
        inference_audit_sha256=sha(output/'inference.json') if getattr(args,'record_inference',False) else None,
        model_sha256=sha(output/'model.mjb'),mass_kg=float(model.body_mass.sum()),
        collision_masks=[dict(name=model.geom(g).name,contype=int(model.geom_contype[g]),conaffinity=int(model.geom_conaffinity[g])) for g in range(model.ngeom) if model.geom(g).name.endswith(('_hull','_collision'))])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),completed=report['completed_sequence'],stages=summaries),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trick-policy',type=Path,required=True);parser.add_argument('--walk-policy',type=Path)
    parser.add_argument('--return-default-policy',type=Path,help='Continue after Spider exit or an explicitly described finite standing-ending trick into actual normal walking; no jumps, continuous physical state')
    parser.add_argument('--trick-descriptor',type=Path,help='Required for finite default return; confirms standing start/end and exact policy path')
    parser.add_argument('--later-return-from-lying',action='store_true',help='Explicit diagnostic of a later move request after a finite trick holds its declared lying endpoint; requires --single-motion and --return-default-policy')
    parser.add_argument('--interactive-sit',action='store_true',help='Explicit sit/idle/left/right/standing-exit queue sequence followed by actual default walking')
    parser.add_argument('--single-motion',action='store_true',help='One finite trick plus2s terminal hold; append normal walking only with explicit --return-default-policy')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--contact-model',choices=['full_hulls','torso_only'],required=True)
    parser.add_argument('--clip-anchor',action='store_true',help='Compare selected-clip anchor proposal; default reproduces legacy controller.')
    parser.add_argument('--driver-limits',type=Path)
    parser.add_argument('--physical-variant',type=Path,help='Explicit schema1 deterministic model-perturbation JSON; kept separate from nominal checks and GPU DR')
    parser.add_argument('--record-inference',action='store_true',help='Save read-only actor inputs, raw outputs, references and target transitions at every inference step')
    parser.add_argument('--max-body-angle-deg',type=float,default=float(np.degrees(1.5)),help='Optional tighter descriptor body-angle stop; may not exceed the existing1.5rad threshold')
    run(parser.parse_args())
