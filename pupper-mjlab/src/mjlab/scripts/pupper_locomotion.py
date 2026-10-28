"""Reproducible Optional Lab 2 training and command-conditioned rollout capture.

Set --reference-dir before task registration; the run stores exact reference hashes.
No command in this module connects to a robot.
"""
from __future__ import annotations
import argparse
import hashlib
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import subprocess
import numpy as np

DEFAULT_SEQUENCE=[[0,0,0],[.3,0,0],[.49,0,0],[.51,0,0],[1.,0,0],[1.5,0,0],[.51,0,0],[.49,0,0],[0,0,0],[-.3,0,0],[-1.5,0,0],[0,0,1.],[0,0,-1.],[0,.3,0],[0,-.3,0],[0,0,0]]
BASE_WEIGHTS={"track_linear_velocity":10.,"track_yaw_velocity":7.,"gait_tracking":10.,"upright":1.,
              "lin_vel_z_l2":-.5,"ang_vel_xy_l2":-.05,"action_rate_l2":-.02,"knee_ground_contact":-1.}
# Hypotheses for the first controlled fast-gait comparison, not tuned results.
FAST_WEIGHTS={**BASE_WEIGHTS,"track_linear_velocity":15.,"gait_tracking":5.,"roll_asymmetry":-2.,"foot_slip":-.1,"joint_torques_l2":-.0001}


def evaluation_episode_length_s(current, seconds, command_count, step_dt):
    return max(current, math.ceil(seconds/step_dt)*step_dt*command_count+10)


def jump_schedule_diagnostics(env, reference_window_s, reward_window_s):
    """Read the actual Mystery schedule at the sampled pre-step clock."""
    from mjlab.tasks.pupper_gait.mdp.mixed_jump import _JUMP_SCHEDULE_ATTR, _slot_progress
    slots=getattr(env,_JUMP_SCHEDULE_ATTR,None)
    if slots is None:
        raise ValueError("Mystery evaluation requires its initialized jump schedule")
    reference_active,reference_elapsed=_slot_progress(env,reference_window_s)
    reward_active,reward_elapsed=_slot_progress(env,reward_window_s)
    return dict(episode_time_s=env.episode_length_buf.float()*env.step_dt,
                jump_slot_starts_s=slots,jump_reference_active=reference_active,
                jump_reference_elapsed_s=reference_elapsed,jump_reward_active=reward_active,
                jump_reward_elapsed_s=reward_elapsed)


def action_target_diagnostics(actions, action_term, robot, clip_actions=None):
    """Read-only affine action audit in action-term order; never change targets.

    The deployment-clamped target is a counterfactual for the C++ controller.
    Training applies the configured action clip, latency and encoder bias.
    These requested targets precede latency; the rollout also records the actuator
    input after stepping, before internal actuator delay. Reset transitions are excluded.
    Joint dynamics/torque saturation are separate and are not inferred here.
    """
    import torch
    applied=actions if clip_actions is None else actions.clamp(-clip_actions,clip_actions)
    requested=applied*action_term.scale+action_term.offset
    target=requested
    if action_term.cfg.clip is not None:
        target=torch.clamp(target,min=action_term._clip[:,:,0],max=action_term._clip[:,:,1])
    ids=action_term.target_ids
    limits=robot.data.joint_pos_limits[:,ids]
    clipped=torch.clamp(requested,min=limits[:,:,0],max=limits[:,:,1])
    return dict(raw_actions=actions,unclipped_target_joint_pos=requested,target_joint_pos=target,
                deployment_clipped_target_joint_pos=clipped,
                target_clipped=torch.abs(requested-clipped)>1e-6,
                target_range_excess_rad=torch.abs(requested-clipped),
                physical_target_joint_pos=target-robot.data.encoder_bias[:,ids])


def provenance(args):
    from mjlab.tasks.pupper_gait.bootstrap import file_hash
    result={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}
    try: result["git_revision"]=subprocess.check_output(["git","rev-parse","HEAD"],text=True,stderr=subprocess.DEVNULL).strip()
    except (OSError,subprocess.CalledProcessError): result["git_revision"]=None
    package=Path(__file__).resolve().parents[1]
    source_hashes={str(p.relative_to(package)):file_hash(p) for p in sorted(package.rglob("*")) if p.is_file() and "__pycache__" not in p.parts and p.suffix in (".py",".xml",".urdf",".yaml",".yml",".json",".csv",".npz")}
    result["source_sha256"]=hashlib.sha256(json.dumps(source_hashes,sort_keys=True).encode()).hexdigest()
    result["source_files_hashed"]=len(source_hashes)
    if getattr(args,"checkpoint",None): result["checkpoint_sha256"]=file_hash(args.checkpoint)
    if args.reference_dir:
        result["references"]={p.name:file_hash(p) for p in args.reference_dir.iterdir() if p.suffix in (".npz",".json")}
    return result


def settings(args,play):
    from mjlab.tasks.registry import load_env_cfg,load_rl_cfg
    cfg=load_env_cfg(args.task,play=play)
    agent=load_rl_cfg(args.task)
    cfg.scene.num_envs=args.num_envs; cfg.seed=args.seed; agent.seed=args.seed
    if getattr(args,"deployment_target_clamp",False):
        from mjlab.tasks.pupper.deployment_contract import apply_deployment_target_clamp
        apply_deployment_target_clamp(cfg)
    if getattr(args,"driver_limits",None):
        if not args.deployment_target_clamp:
            raise ValueError('--driver-limits requires --deployment-target-clamp')
        from mjlab.tasks.pupper.deployment_contract import apply_driver_target_clamp
        args.driver_target_contract=apply_driver_target_clamp(cfg,args.driver_limits)
    if getattr(args,"compose_com_randomization",False):
        from mjlab.tasks.pupper.deployment_contract import compose_pupper_com_randomization
        compose_pupper_com_randomization(cfg)
    if getattr(args,"locomotion_torso_contact",False):
        from mjlab.tasks.pupper.locomotion_contact_contract import enable_locomotion_torso_contact
        args.locomotion_contact_contract=enable_locomotion_torso_contact(cfg)
    return cfg,agent


def train(args):
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.tasks.registry import load_runner_cls
    from mjlab.utils.os import dump_yaml
    from mjlab.utils.torch import configure_torch_backends
    cfg,agent=settings(args,False)
    weights=json.loads(args.weights.read_text()) if args.weights else (BASE_WEIGHTS if args.profile=="baseline" else FAST_WEIGHTS)
    if "Mystery" in args.task and not args.weights:
        raise ValueError("Mystery training requires an explicit reviewed reward JSON; no unreviewed jump reward defaults")
    unknown=set(weights)-set(cfg.rewards)
    if unknown: raise ValueError(f"Unknown rewards: {sorted(unknown)}")
    for reward in cfg.rewards.values(): reward.weight=0.
    for name,value in weights.items():
        if not np.isfinite(value): raise ValueError(f"Nonfinite reward {name}")
        cfg.rewards[name].weight=float(value)
    if not any(weights.values()): raise ValueError("At least one nonzero reward is required")
    if args.checkpoint:
        # A walking warm start must not spend 500 iterations held aloft with all
        # non-imitation rewards zero. Matching the course's robust finetune path.
        cfg.events.pop("hold_base_airborne",None); cfg.curriculum.pop("pretrain_rewards",None)
    agent.max_iterations=args.iterations; agent.logger=args.logger
    agent.experiment_name="pupper_foundation_optional2"; agent.run_name=args.out.name
    args.out.mkdir(parents=True,exist_ok=False)
    run=provenance(args); run.update(rewards=weights,status="training_started",reference_claim="candidate, not a measured improvement")
    (args.out/"provenance.json").write_text(json.dumps(run,indent=2)+"\n")
    dump_yaml(args.out/"params"/"env.yaml",asdict(cfg)); dump_yaml(args.out/"params"/"agent.yaml",asdict(agent))
    configure_torch_backends()
    env=ManagerBasedRlEnv(cfg,device=args.device)
    try:
        from mjlab.tasks.pupper.randomization_audit import write_randomization_snapshot
        from mjlab.tasks.pupper.collision_audit import collision_model_contract
        run["domain_randomization"]=write_randomization_snapshot(env,args.out)
        run["collision_model"]=collision_model_contract(env)
        (args.out/"provenance.json").write_text(json.dumps(run,indent=2)+"\n")
        wrapped=RslRlVecEnvWrapper(env,clip_actions=agent.clip_actions)
        runner=load_runner_cls(args.task)(wrapped,asdict(agent),str(args.out),args.device)
        if args.checkpoint: runner.load(str(args.checkpoint),map_location=args.device)
        runner.learn(num_learning_iterations=args.iterations,init_at_random_ep_len=True)
    finally:
        env.close()
    run["status"]="training_finished_not_evaluated"
    (args.out/"provenance.json").write_text(json.dumps(run,indent=2)+"\n")


def evaluate(args):
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.tasks.registry import load_runner_cls
    from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES,FOOT_SITE_NAMES
    from mjlab.tasks.pupper_gait.mdp.gait_reference import _GAIT_FREQ_MULT
    from mjlab.tasks.pupper_gait.pupper_gait_env_cfg import GAIT_FREQUENCY,TROTGALLOP_ONSET
    from mjlab.tasks.pupper_gait.bootstrap import summarize_trace
    from mjlab.utils.os import dump_yaml
    from mjlab.utils.torch import configure_torch_backends
    cfg,agent=settings(args,not args.randomized)
    cfg.events.pop("hold_base_airborne",None); cfg.curriculum.pop("pretrain_rewards",None)
    term_cfg=cfg.commands["twist"]
    term_cfg.resampling_time_range=(1e8,1e8)
    for name in ("rel_standing_envs","rel_heading_envs","rel_world_envs","rel_forward_envs","rel_turn_envs","init_velocity_prob"):
        if hasattr(term_cfg,name): setattr(term_cfg,name,0.)
    term_cfg.heading_command=False
    if args.no_heading_hold: term_cfg.heading_hold_kp=None
    commands=json.loads(args.commands.read_text()) if args.commands else DEFAULT_SEQUENCE
    if not commands or np.asarray(commands).ndim!=2 or np.asarray(commands).shape[1]!=3 or not np.isfinite(commands).all(): raise ValueError("commands JSON must be finite list of [vx,vy,yaw]")
    # Long custom sweeps must not reset merely because the default sequence has
    # fewer commands. Account for step rounding, including randomized configs.
    dt=cfg.sim.mujoco.timestep*cfg.decimation
    cfg.episode_length_s=evaluation_episode_length_s(cfg.episode_length_s,args.seconds,len(commands),dt)
    ranges=[term_cfg.ranges.lin_vel_x,term_cfg.ranges.lin_vel_y,term_cfg.ranges.ang_vel_z]
    if not args.allow_out_of_range and any(not lo<=float(cmd[j])<=hi for cmd in commands for j,(lo,hi) in enumerate(ranges)):
        raise ValueError(f"Commands exceed task training ranges {ranges}; supply a task-appropriate --commands JSON (or explicitly --allow-out-of-range)")
    args.out.mkdir(parents=True,exist_ok=False)
    run=provenance(args); run.update(commands=commands,randomized=args.randomized,zero_policy=args.zero_policy)
    (args.out/"provenance.json").write_text(json.dumps(run,indent=2)+"\n")
    dump_yaml(args.out/"env.yaml",asdict(cfg))
    configure_torch_backends()
    env=ManagerBasedRlEnv(cfg,device=args.device)
    samples={k:[] for k in ("joint_pos","root_position","root_quaternion","phase","episode","command","effective_command","valid","velocity","torque","foot_contact","foot_speed_xy","heading","terminated","timeout","segment","height","vertical_velocity","foot_force_norm","raw_actions","unclipped_target_joint_pos","target_joint_pos","deployment_clipped_target_joint_pos","target_clipped","target_range_excess_rad","physical_target_joint_pos","actuator_input_target_joint_pos")}
    try:
        from mjlab.tasks.pupper.randomization_audit import write_randomization_snapshot
        from mjlab.tasks.pupper.collision_audit import collision_model_contract,flat_torso_clearance
        run["domain_randomization"]=write_randomization_snapshot(env,args.out)
        run["collision_model"]=collision_model_contract(env)
        run["randomized_flag_semantics"]="selects training task configuration; play configuration already retains startup physics randomization and observation noise"
        wrapped=RslRlVecEnvWrapper(env,clip_actions=agent.clip_actions)
        if args.zero_policy:
            policy=lambda obs:torch.zeros((args.num_envs,wrapped.num_actions),device=args.device)
        else:
            runner=load_runner_cls(args.task)(wrapped,asdict(agent),device=args.device)
            runner.load(str(args.checkpoint),load_cfg={"actor":True},strict=True,map_location=args.device)
            policy=runner.get_inference_policy(device=args.device)
        robot=env.scene["robot"]
        joint_ids=[robot.joint_names.index(n) for n in JOINT_NAMES]
        geometry_model=env.sim.mj_model
        geometry_root_id=int(robot.indexing.root_body_id)
        geometry_root_qpos_ids=robot.indexing.free_joint_q_adr.cpu().numpy()
        geometry_joint_qpos_ids=robot.indexing.joint_q_adr[joint_ids].cpu().numpy()
        if args.geometry_audit and geometry_model.nhfield:
            raise ValueError("--geometry-audit signed torso distance is only validated for plane terrain")
        if env.action_manager.active_terms != ["joint_pos"]:
            raise ValueError("Action-target audit requires the Pupper joint_pos-only control contract")
        action_term=env.action_manager.get_term("joint_pos")
        action_ids=[action_term.target_names.index(n) for n in JOINT_NAMES]
        run["action_target_audit"]=dict(wrapper_clip_actions=agent.clip_actions,
            action_term_clip=action_term.cfg.clip,
            meaning="raw actor actions; unclipped post-wrapper affine request; target_joint_pos after configured clip before latency/bias; physical_target_joint_pos subtracts bias before latency; actuator_input_target_joint_pos is post-step target after action-term latency/bias but before internal actuator delay (reset steps excluded); deployment target clamps affine request to model joint ranges; clipping fractions use unclipped request")
        jump_windows=None
        if "Mystery" in args.task:
            jump_windows=(float(cfg.observations["actor"].terms["policy"].params["slot_s"]),
                          float(cfg.rewards["jump_airborne_height"].params["slot_s"]))
            run["jump_schedule_audit"]=dict(reference_window_s=jump_windows[0],reward_window_s=jump_windows[1],
                meaning="Actual reset-time slot schedule and unmodified episode clock before stepping; +inf denotes unused slots. Activity masks use the same scheduler function as the policy reference and reward, without the tracking reward's phase-lead rewind.")
            for key in jump_schedule_diagnostics(env,*jump_windows):samples[key]=[]
        (args.out/"provenance.json").write_text(json.dumps(run,indent=2)+"\n")
        site_ids,_=robot.find_sites(FOOT_SITE_NAMES,preserve_order=True)
        contact_sensor=env.scene["feet_ground_contact"]
        command_term=env.command_manager.get_term("twist")
        episodes=np.zeros(args.num_envs,dtype=np.int64)
        age=np.zeros(args.num_envs,dtype=float)
        # Observe each state BEFORE stepping, then attach the next transition's
        # terminal flags. An autoreset pose can never masquerade as the fall state.
        dt=env.step_dt; steps=int(np.ceil(args.seconds/dt))
        with torch.inference_mode():
            for segment,requested in enumerate(commands):
                if args.reset_each_command:
                    wrapped.reset(); episodes+=1; age[:]=0
                for step in range(steps):
                    command_term.vel_command_b[:]=torch.tensor(requested,device=args.device)
                    command_term.vel_command_w[:]=command_term.vel_command_b
                    command_term.time_left.fill_(1e8)
                    for name in ("is_standing_env","is_heading_env","is_world_env","is_turn_env"):
                        if hasattr(command_term,name): getattr(command_term,name).fill_(False)
                    command_term.compute(0.)
                    obs=wrapped.get_observations()
                    actions=policy(obs)
                    action_diagnostics=action_target_diagnostics(actions,action_term,robot,agent.clip_actions)
                    for key,value in action_diagnostics.items():
                        samples[key].append(value[:,action_ids].detach().cpu().numpy().copy())
                    effective=command_term.command.detach().cpu().numpy().copy()
                    raw=np.broadcast_to(requested,(args.num_envs,3)).copy()
                    # Match the gait reward's phase clock; spider has the task's
                    # actor frequency (not necessarily GAIT_FREQUENCY).
                    reward=cfg.rewards.get("gait_tracking")
                    freq=float(reward.params.get("frequency",GAIT_FREQUENCY)) if reward else GAIT_FREQUENCY
                    fast=(np.abs(raw[:,0])>=TROTGALLOP_ONSET)&("MixedGaits" in args.task or "Mystery" in args.task)
                    frequency=freq*np.where(fast,_GAIT_FREQ_MULT.get("reach",1.),1.)
                    phase=np.mod(env.episode_length_buf.cpu().numpy()*dt*frequency,1.)
                    arrays=dict(joint_pos=robot.data.joint_pos[:,joint_ids],velocity=torch.cat([robot.data.root_link_lin_vel_b[:,:2],robot.data.root_link_ang_vel_b[:,2:3]],dim=1),
                                root_position=robot.data.root_link_pos_w,root_quaternion=robot.data.root_link_quat_w,
                                torque=robot.data.qfrc_actuator[:,joint_ids],foot_contact=contact_sensor.data.found>0,
                                foot_speed_xy=torch.linalg.vector_norm(robot.data.site_lin_vel_w[:,site_ids,:2],dim=-1),heading=robot.data.heading_w,height=robot.data.root_link_pos_w[:,2],
                                vertical_velocity=robot.data.root_link_lin_vel_w[:,2],
                                foot_force_norm=torch.linalg.vector_norm(contact_sensor.data.force,dim=-1) if contact_sensor.data.force is not None else torch.full_like(contact_sensor.data.found,float("nan"),dtype=torch.float32))
                    if jump_windows is not None:arrays.update(jump_schedule_diagnostics(env,*jump_windows))
                    for key,value in arrays.items(): samples[key].append(value.detach().cpu().numpy().copy())
                    samples["phase"].append(phase); samples["episode"].append(episodes.copy())
                    samples["command"].append(raw); samples["effective_command"].append(effective)
                    samples["valid"].append((age>=args.warmup)&(step*dt>=args.warmup))
                    samples["segment"].append(segment)
                    _,_,done,_=wrapped.step(actions)
                    samples["actuator_input_target_joint_pos"].append(robot.data.joint_pos_target[:,joint_ids].detach().cpu().numpy().copy())
                    terminated=env.reset_terminated.cpu().numpy().copy(); timeout=env.reset_time_outs.cpu().numpy().copy()
                    samples["terminated"].append(terminated); samples["timeout"].append(timeout)
                    done_np=done.cpu().numpy().astype(bool)
                    # A cycle ending in a failure is never admissible for bootstrapping.
                    samples["valid"][-1][done_np]=False
                    episodes+=done_np; age=np.where(done_np,0,age+dt)
    finally:
        env.close()
    trace={k:np.asarray(v) for k,v in samples.items()}; trace["dt"]=np.asarray(dt)
    trace["joint_names"]=np.asarray(JOINT_NAMES)
    if args.geometry_audit:
        geometry_report,distance=flat_torso_clearance(geometry_model,trace,geometry_root_id,geometry_root_qpos_ids,geometry_joint_qpos_ids,args.geometry_audit_stride)
        trace["torso_ground_signed_distance_m"]=distance
        (args.out/"torso_geometry_audit.json").write_text(json.dumps(geometry_report,indent=2)+"\n")
        run["torso_geometry_audit"]="torso_geometry_audit.json"
    (args.out/"provenance.json").write_text(json.dumps(run,indent=2)+"\n")
    np.savez_compressed(args.out/"trace.npz",**trace)
    report=summarize_trace(trace,dt,args.warmup); report["provenance"]=run
    (args.out/"metrics.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__); sub=p.add_subparsers(dest="command",required=True)
    tr=sub.add_parser("train"); ev=sub.add_parser("evaluate")
    for parser in (tr,ev):
        parser.add_argument("--task",default="Mjlab-MixedGaits-Flat-Pupper-v3")
        parser.add_argument("--reference-dir",type=Path); parser.add_argument("--checkpoint",type=Path)
        parser.add_argument("--out",type=Path,required=True); parser.add_argument("--seed",type=int,default=42)
        parser.add_argument("--device",default="cuda:0")
        parser.add_argument("--deployment-target-clamp",action="store_true",help="Opt in to the C++ joint-target clamp before simulation; preserves normalized last_action observations")
        parser.add_argument("--driver-limits",type=Path,help="Explicit JSON with named hardware-driver joint limits; composes a second clamp and requires --deployment-target-clamp")
        parser.add_argument("--compose-com-randomization",action="store_true",help="Opt in to preserving the independent base COM offset during density randomization; legacy behavior remains default")
        parser.add_argument("--locomotion-torso-contact",action="store_true",help="Opt in to one torso hull while retaining the original knee/foot spheres; new physical model requires reevaluation")
    tr.add_argument("--num-envs",type=int,default=4096); tr.add_argument("--iterations",type=int,default=3000)
    tr.add_argument("--profile",choices=["baseline","fast"],default="baseline"); tr.add_argument("--weights",type=Path)
    tr.add_argument("--logger",choices=["wandb","tensorboard"],default="wandb")
    ev.add_argument("--num-envs",type=int,default=32); ev.add_argument("--seconds",type=float,default=10.)
    ev.add_argument("--warmup",type=float,default=2.); ev.add_argument("--commands",type=Path)
    ev.add_argument("--randomized",action="store_true"); ev.add_argument("--reset-each-command",action="store_true")
    ev.add_argument("--no-heading-hold",action="store_true"); ev.add_argument("--allow-out-of-range",action="store_true")
    ev.add_argument("--zero-policy",action="store_true",help="Infrastructure smoke test, never a trained-policy result")
    ev.add_argument("--geometry-audit",action="store_true",help="Read-only signed torso/plane distance at recorded states; no collision response changes; rejects heightfield terrain")
    ev.add_argument("--geometry-audit-stride",type=int,default=1)
    args=p.parse_args()
    if args.reference_dir:
        args.reference_dir=args.reference_dir.resolve()
        if not args.reference_dir.is_dir(): p.error("reference directory does not exist")
        os.environ["PUPPER_REFERENCE_DIR"]=str(args.reference_dir)
    if args.num_envs<1: p.error("num-envs must be positive")
    if args.command=="evaluate":
        if not args.checkpoint and not args.zero_policy: p.error("evaluation requires --checkpoint or explicit --zero-policy")
        if args.seconds<=args.warmup or args.warmup<0: p.error("seconds must exceed nonnegative warmup")
        if args.geometry_audit_stride<1:p.error("geometry-audit-stride must be positive")
        evaluate(args)
    else: train(args)

if __name__=="__main__": main()
