#!/usr/bin/env python3
"""Inspect actual compiled locomotion collisions and deliberately forced contacts.

No policy is trained or changed. Forced overlap probes verify collision/sensor
wiring, not successful behavior, impact safety or hardware clearance.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
from mjlab.tasks.velocity.mdp.rewards import self_collision_cost

TASKS = ["Mjlab-StableGait-Flat-Pupper-v3", "Mjlab-MixedGaits-Flat-Pupper-v3",
         "Mjlab-SpiderWalk-Flat-Pupper-v3", "Mjlab-MixedGaits-Bumpy-Pupper-v3",
         "Mjlab-Mystery-Flat-Pupper-v3", "Mjlab-Mystery-Bumpy-Pupper-v3"]


def geometry(model, gid):
  return dict(id=int(gid), name=model.geom(int(gid)).name,
              body=model.body(int(model.geom_bodyid[gid])).name,
              type=mujoco.mjtGeom(int(model.geom_type[gid])).name,
              contype=int(model.geom_contype[gid]), conaffinity=int(model.geom_conaffinity[gid]),
              priority=int(model.geom_priority[gid]))


def bottom(model, data, ids):
  values=[]
  for gid in ids:
    box=np.asarray(model.geom_aabb[gid]).reshape(-1)
    corners=np.asarray([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*box[3:]+box[:3]
    world=corners@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
    values.append(float(world[:,2].min()))
  return min(values) if values else None


def audit_task(task):
  cfg=load_env_cfg(task,play=True)
  cfg.scene.num_envs=1;cfg.seed=123
  env=ManagerBasedRlEnv(cfg,device="cpu")
  try:
    robot=env.scene["robot"];model=env.sim.mj_model;data=mujoco.MjData(model)
    gids=robot.indexing.geom_ids.cpu().tolist();body_ids=robot.indexing.body_ids.cpu().tolist()
    root_id=int(robot.indexing.root_body_id)
    torso=[g for g in gids if int(model.geom_bodyid[g])==root_id]
    root_indices=robot.indexing.free_joint_q_adr.cpu().tolist()
    joint_indices=robot.indexing.joint_q_adr.cpu().tolist()
    origin=env.scene.env_origins[0].cpu().numpy()
    base=robot.data.default_root_state[0].cpu().numpy().copy();base[:3]+=origin
    joints=robot.data.default_joint_pos[0].cpu().numpy().copy()
    data.qpos[root_indices]=base[:7];data.qpos[joint_indices]=joints
    mujoco.mj_forward(model,data)
    geoms=[geometry(model,g) for g in gids]
    active=[g for g in geoms if g['contype'] or g['conaffinity']]
    fields=[]
    for h in range(model.nhfield):
      begin=int(model.hfield_adr[h]);count=int(model.hfield_nrow[h]*model.hfield_ncol[h])
      values=model.hfield_data[begin:begin+count]
      fields.append(dict(id=h,size=model.hfield_size[h].tolist(),
                         normalized_min=float(values.min()),normalized_max=float(values.max()),
                         surface_height_span_m=float(np.ptp(values)*model.hfield_size[h,2])))
    probes=[]
    # Nominal pose followed by forced lowering; joint angles stay unchanged.
    for dz in (0.,-.04,-.08,-.12,-.18):
      data.qpos[root_indices]=base[:7];data.qpos[root_indices[2]]+=dz
      mujoco.mj_forward(model,data)
      contacts=[]
      for contact in data.contact:
        pair=[int(contact.geom1),int(contact.geom2)]
        if not any(g in gids for g in pair):continue
        contacts.append(dict(geoms=[model.geom(g).name for g in pair],
                             world_z=float(contact.pos[2]),distance_m=float(contact.dist)))
      probes.append(dict(base_lowering_m=dz,base_world_z=float(data.qpos[root_indices[2]]),
                         torso_aabb_bottom_world_z=bottom(model,data,torso),contacts=contacts))
    # The Warp sensor uses the actual environment's compiled contact wiring.
    forced=robot.data.default_root_state.clone();forced[:,:3]+=env.scene.env_origins
    forced[:,2]-=.12
    robot.write_root_state_to_sim(forced)
    robot.write_joint_state_to_sim(robot.data.default_joint_pos,torch.zeros_like(robot.data.default_joint_pos))
    env.scene.write_data_to_sim();env.sim.forward();env.scene.update(env.physics_dt)
    sensors={}
    for name in ("feet_ground_contact","knee_ground_contact"):
      sensor=env.scene[name];s=sensor.data
      sensors[name]=dict(found=s.found.cpu().tolist(),force=s.force.cpu().tolist(),
                         history_length=sensor.cfg.history_length,
                         cost_threshold_1=float(self_collision_cost(env,name,1.)[0]),
                         cost_threshold_1000000=float(self_collision_cost(env,name,1e6)[0]))
    torso_reward=cfg.rewards.get("torso_clearance")
    return dict(task=task,play=True,worlds=1,seed=123,
                spec_function=f"{cfg.scene.entities['robot'].spec_fn.__module__}.{cfg.scene.entities['robot'].spec_fn.__name__}",
                robot_body_count=len(body_ids),robot_geometry_count=len(gids),active_robot_geometries=active,
                torso_geometries=[geometry(model,g) for g in torso],
                terrain_geometry_count=sum(g not in gids for g in range(model.ngeom)),
                heightfields=fields,forced_mujoco_contact_probes=probes,forced_warp_sensor_probe=sensors,
                configured_knee_cost=cfg.rewards["knee_ground_contact"].params,
                torso_clearance=dict(present=torso_reward is not None,default_weight=torso_reward.weight if torso_reward else None,
                                     reference="world z=0; no local terrain-height query") if torso_reward else dict(present=False),
                limitation="Collision/sensor wiring audit only. Forced overlaps are not rollout performance, safe impacts or verified hardware geometry.")
  finally:
    env.close()


def main():
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--task',action='append',choices=TASKS)
  parser.add_argument('--out',type=Path,required=True)
  args=parser.parse_args()
  args.out.parent.mkdir(parents=True,exist_ok=True)
  results=[]
  for task in args.task or TASKS:
    print(f'AUDIT {task}',flush=True);results.append(audit_task(task))
    args.out.write_text(json.dumps(dict(results=results,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')


if __name__=='__main__':main()
