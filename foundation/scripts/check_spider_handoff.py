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


def stage_plan(trick, single_motion=False):
    if single_motion:
        metadata=trick['trick'];clips=metadata.get('clips')
        if clips:
            if len(clips)!=1 or clips[0].get('loop') or clips[0].get('next'):
                raise ValueError('single-motion mode requires one finite clip without a next clip')
            clip=clips[0];name=clip['name'];frames=clip['end']-clip['start']
        else:
            name='all';frames=len(trick['motion']['joint_pos'])
        duration=(frames-1)/metadata['fps']
        return [('motion','trick',.5+1+duration,[0,0,0],name),
                ('terminal_hold','trick',2,[0,0,0],'')]
    clips={c['name']:c for c in trick['trick']['clips']};fps=trick['trick']['fps']
    down=(clips['spider_down']['end']-clips['spider_down']['start']-1)/fps
    up=(clips['spider_up']['end']-clips['spider_up']['start']-1)/fps
    return [('entry','trick',.5+1+down+.3,[0,0,0],'spider_down'),
            ('walk_activation','walk',4.2,[0,0,0],''),
            ('walk_forward','walk',2,[.10,0,0],''),
            ('walk_stop_1','walk',1,[0,0,0],''),
            ('walk_backward','walk',2,[-.10,0,0],''),
            ('walk_turn','walk',2,[0,0,.2],''),
            ('walk_stop_before_exit','walk',2,[0,0,0],''),
            ('exit','trick',.5+1+up+.3,[0,0,0],'spider_up'),
            ('standing_hold','trick',1,[0,0,0],'')]


def run(args):
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    sources = [ROOT/'foundation/cpp/handoff_controller.cpp', Path(__file__).resolve(),
               *sorted((ROOT/'pupper_gait_deploy/src').glob('*.hpp')),
               ROOT/'pupper_gait_deploy/src/neural_controller.cpp', ROOT/'pupper_gait_deploy/config.yaml']
    sources += [ROOT/'pupper-mjlab/src/mjlab'/p for p in (
        'tasks/tracking/config/pupper/env_cfgs.py','tasks/tracking/config/pupper/contact_contract.py',
        'tasks/pupper/locomotion_contact_contract.py','tasks/pupper/deployment_contract.py',
        'asset_zoo/robots/pupper_v3/pupper_constants.py','entity/entity.py','sim/sim.py')]
    source_before = {str(p.relative_to(ROOT)):sha(p) for p in sources}
    binary, compile_command = build_controller(output)
    single_motion=getattr(args,'single_motion',False)
    if args.walk_policy is None and not single_motion:raise ValueError('--walk-policy required for Spider handoff')
    walk_path=args.walk_policy or args.trick_policy  # second host slot unused in single-motion mode
    trick = json.loads(args.trick_policy.read_text()); walking = json.loads(walk_path.read_text())
    from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES
    model, contract = make_model(args.contact_model); data = mujoco.MjData(model)
    mujoco.mj_saveModel(model,str(output/'model.mjb'),None)
    joints = [model.joint(name).id for name in JOINT_NAMES]
    qadr, vadr = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    controls = [int(np.flatnonzero(model.actuator_trnid[:,0] == j)[0]) for j in joints]
    free = int(np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)[0])
    root_q, root_v = int(model.jnt_qposadr[free]), int(model.jnt_dofadr[free])
    body = model.body('base_link').id
    data.qpos[root_q:root_q+7] = [0,0,.18,1,0,0,0]
    data.qpos[qadr] = trick['default_joint_pos'] if single_motion else trick['motion']['joint_pos'][0]
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
    stages=stage_plan(trick,single_motion)
    proc=subprocess.Popen([str(binary),str(args.trick_policy.resolve()),str(walk_path.resolve())],
                          stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(output/'controller.log').open('w'),text=True,bufsize=1)
    def rpc(request):
        proc.stdin.write(json.dumps(request)+'\n'); proc.stdin.flush()
        line=proc.stdout.readline()
        if not line: raise RuntimeError('controller exited; inspect controller.log')
        return json.loads(line)
    samples=[]; activations=[]; previous_controller=None; activation_time=0.; previous_target=data.qpos[qadr].copy()
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
                activations.append(dict(stage=stage,time_s=current_time,root_pose=data.qpos[root_q:root_q+7].tolist(),
                                        joint_pos=data.qpos[qadr].tolist(),joint_vel=data.qvel[vadr].tolist(),
                                        physical_state_reset=False,observation_history_reset=True))
                previous_controller=controller
            for _ in range(round(duration*520)):
                quat=data.qpos[root_q+3:root_q+7].copy()
                state=rpc(dict(op='step',elapsed_s=current_time-activation_time,joint_pos=data.qpos[qadr].tolist(),
                    joint_vel=data.qvel[vadr].tolist(),quaternion=quat.tolist(),angular_velocity=data.qvel[root_v+3:root_v+6].tolist(),command=command))
                target=np.asarray(state['target']); final_target=np.clip(target,lower,upper)
                interval_target_step=max(interval_target_step,float(np.max(np.abs(target-previous_target))))
                delayed.append((current_time,final_target.copy())); cutoff=current_time-.016
                while len(delayed)>2 and delayed[1][0]<=cutoff: delayed.popleft()
                t0,v0=delayed[0];t1,v1=delayed[1];a=np.clip((cutoff-t0)/(t1-t0),0,1)
                data.ctrl[controls]=(1-a)*v0+a*v1
                model.actuator_gainprm[controls,0]=state['kp']
                model.actuator_biasprm[controls,1]=-state['kp'];model.actuator_biasprm[controls,2]=-state['kd']
                mujoco.mj_step(model,data)
                if tick%10==0:
                    w,x,y,z=data.qpos[root_q+3:root_q+7]
                    tilt=np.degrees(np.arccos(np.clip(1-2*(x*x+y*y),-1,1)))
                    contacts=[]
                    for contact_id,c in enumerate(data.contact):
                        names=[model.geom(int(g)).name for g in (c.geom1,c.geom2)]
                        if 'terrain' in names:
                            force=np.zeros(6);mujoco.mj_contactForce(model,data,contact_id,force)
                            contacts.append(dict(geom=names[1-names.index('terrain')],distance_m=float(c.dist),friction=float(c.friction[0]),normal_force_n=float(force[0])))
                    samples.append(dict(time_s=current_time,stage=stage,controller=controller,activation_stage=state['stage'],
                        clip=state.get('clip'),frame=state['frame'],holding=state.get('holding'),
                        root_pos=data.qpos[root_q:root_q+3].tolist(),root_quat=data.qpos[root_q+3:root_q+7].tolist(),
                        root_velocity_body=(data.xmat[body].reshape(3,3).T@data.qvel[root_v:root_v+3]).tolist(),
                        joint_pos=data.qpos[qadr].tolist(),joint_vel=data.qvel[vadr].tolist(),
                        target=target.tolist(),driver_target=final_target.tolist(),before_clip=state['before_clip'],
                        command=command,tilt_deg=float(tilt),target_delta_max_rad=interval_target_step,
                        torque_max_nm=float(np.max(np.abs(data.actuator_force[controls]))),contacts=contacts))
                    interval_target_step=0.
                    if not np.isfinite(data.qpos).all() or tilt>np.degrees(1.5):
                        failure=dict(time_s=current_time,reason='nonfinite state or configured 1.5rad controller tilt stop'); break
                previous_target=target.copy();tick+=1;current_time=tick/520
            if failure: break
            if stage=='entry' and state.get('clip')!='spider_idle':
                failure=dict(time_s=current_time,reason='entry controller did not confirm spider_idle');break
            if stage=='exit' and not (state.get('clip')=='spider_up' and state.get('holding')):
                failure=dict(time_s=current_time,reason='exit controller did not confirm final hold');break
            if stage=='terminal_hold' and not state.get('holding'):
                failure=dict(time_s=current_time,reason='finite clip did not confirm terminal hold');break
    finally:
        proc.stdin.close();proc.wait(timeout=10)
    traces=output/'trajectory.json';traces.write_text(json.dumps(samples,separators=(',',':'))+'\n')
    np.savez_compressed(output/'trajectory.npz',fps=52,actual_root_pos=np.array([s['root_pos'] for s in samples]),
                        actual_root_quat=np.array([s['root_quat'] for s in samples]),actual_joint_pos=np.array([s['joint_pos'] for s in samples]))
    summaries={}
    for name,*_ in stages:
        rows=[s for s in samples if s['stage']==name]
        if not rows: continue
        default=np.asarray(walking['default_joint_pos'] if rows[-1]['controller']=='walk' else trick['default_joint_pos'])
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
    report=dict(protocol='exported_finite_trick_activation_and_hold_v1' if single_motion else 'continuous_exported_controller_spider_handoff_v1',hardware_validated=False,
        scope='Native MuJoCo diagnostic with production RTNeural and math headers, explicitly transcribed ROS activation plumbing; not a ROS execution or hardware approval.',
        contact_contract=contract,clip_activation_anchor='selected_clip_first_frame' if args.clip_anchor else 'legacy_network_default',
        physical_state_resets_after_initialization=0,physics_hz=520,controller_inference_hz=52,action_delay_s=.016,
        caveats=['Native CPU solver at520Hz differs from250Hz training Warp solver.',
                 'Nominal model: no observation noise, IMU delay, encoder bias, parameter randomization or external push.',
                 'Clip completion is controller clock/status, not measured joint settlement; endpoint errors reported separately.',
                 'Single finite clip plus2s hold; no walking handoff tested.' if single_motion else 'No final walking-controller activation after standing exit; bounded diagnostic stops in trick standing hold.',
                 'MuJoCo derived contacts/forces accompany the forward solve immediately before integration; recorded qpos is after integration, at most1/520s later.'],
        trick_policy_sha256=sha(args.trick_policy),walk_policy_sha256=None if single_motion else sha(args.walk_policy),
        driver_limits=None if args.driver_limits is None else dict(path=str(args.driver_limits),sha256=sha(args.driver_limits)),
        sources_start=source_before,sources_end=source_after,source_changed=source_before!=source_after,
        compiler_command=compile_command,activations=activations,stage_summaries=summaries,stop=failure,
        trajectory_sha256=sha(traces),completed_sequence=failure is None,
        model_sha256=sha(output/'model.mjb'),mass_kg=float(model.body_mass.sum()),
        collision_masks=[dict(name=model.geom(g).name,contype=int(model.geom_contype[g]),conaffinity=int(model.geom_conaffinity[g])) for g in range(model.ngeom) if model.geom(g).name.endswith(('_hull','_collision'))])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),completed=report['completed_sequence'],stages=summaries),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trick-policy',type=Path,required=True);parser.add_argument('--walk-policy',type=Path)
    parser.add_argument('--single-motion',action='store_true',help='One finite trick plus2s terminal hold; no walking controller used')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--contact-model',choices=['full_hulls','torso_only'],required=True)
    parser.add_argument('--clip-anchor',action='store_true',help='Compare selected-clip anchor proposal; default reproduces legacy controller.')
    parser.add_argument('--driver-limits',type=Path)
    run(parser.parse_args())
