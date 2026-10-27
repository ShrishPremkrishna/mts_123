"""Read-only collision-contract and flat-ground torso-distance diagnostics."""
from __future__ import annotations
import hashlib
import json
import numpy as np
import mujoco


def collision_model_contract(env):
  model=env.sim.mj_model;robot=env.scene["robot"]
  root=int(robot.indexing.root_body_id)
  records=[]
  for gid in robot.indexing.geom_ids.cpu().tolist():
    records.append(dict(id=int(gid),name=model.geom(gid).name,body=model.body(int(model.geom_bodyid[gid])).name,
                        type=mujoco.mjtGeom(int(model.geom_type[gid])).name,
                        contype=int(model.geom_contype[gid]),conaffinity=int(model.geom_conaffinity[gid]),
                        torso=bool(model.geom_bodyid[gid]==root)))
  active=[r for r in records if r["contype"] or r["conaffinity"]]
  return dict(robot_geometries=records,active_robot_geometry_count=len(active),
              active_torso_geometry_count=sum(r["torso"] for r in active),
              terrain_heightfield_count=int(model.nhfield),
              geometry_contract_sha256=hashlib.sha256(json.dumps(records,sort_keys=True).encode()).hexdigest(),
              scope="actual compiled collision masks; disabled torso geometry can be measured diagnostically but contributes no contact response")


def flat_torso_clearance(model,trace,root_body_id,root_qpos_ids,joint_qpos_ids,stride=1):
  """Signed geom distance at recorded poses, independent of contact enable bits.

  Restricted to plane terrain: direct mesh/heightfield distance queries have not
  been validated for this model. No collision masks or simulation state change.
  """
  if stride<1:raise ValueError("Geometry audit stride must be positive")
  if model.nhfield:raise ValueError("Signed torso audit is only validated on plane terrain; heightfields require a separate validated distance method")
  robot_bodies={i for i in range(model.nbody) if i==root_body_id or str(model.body(i).name).startswith("robot/")}
  torso=[g for g in range(model.ngeom) if int(model.geom_bodyid[g])==root_body_id]
  ground=[g for g in range(model.ngeom) if int(model.geom_bodyid[g]) not in robot_bodies]
  if not torso or not ground or any(model.geom_type[g]!=mujoco.mjtGeom.mjGEOM_PLANE for g in ground):
    raise ValueError("Torso audit requires known robot torso geometries and plane-only external terrain")
  t,n=trace["valid"].shape
  distance=np.full((t,n),np.nan,dtype=np.float64)
  data=mujoco.MjData(model)
  for step in range(0,t,stride):
    for world in range(n):
      data.qpos[root_qpos_ids]=np.r_[trace["root_position"][step,world],trace["root_quaternion"][step,world]]
      data.qpos[joint_qpos_ids]=trace["joint_pos"][step,world]
      mujoco.mj_forward(model,data)
      distance[step,world]=min(mujoco.mj_geomDistance(model,data,a,b,10.,None) for a in torso for b in ground)
  records=[]
  for segment in np.unique(trace["segment"]):
    selected=(trace["segment"]==segment)[:,None]&np.isfinite(distance)
    valid=selected&trace["valid"].astype(bool)
    values=distance[selected];scored=distance[valid]
    records.append(dict(segment=int(segment),sampled_states=int(selected.sum()),
                        minimum_clearance_m=float(values.min()) if len(values) else None,
                        penetrated_sampled_states=int((values<-.0001).sum()),
                        valid_sampled_states=int(valid.sum()),
                        minimum_valid_clearance_m=float(scored.min()) if len(scored) else None,
                        penetrated_valid_sampled_states=int((scored<-.0001).sum())))
  return dict(method="MuJoCo signed distance from compiled torso geom to plane at recorded root/joint poses, independent of collision masks",
              scope="compiled mesh geometry, not verified hardware body dimensions; diagnostic does not add contact response",
              sample_stride=stride,penetration_tolerance_m=.0001,segments=records,
              limitations=["Samples precede each policy transition; transient penetration between steps or on the final failing transition can be missed.",
                           "Correlated trajectory states are not independent safety trials.",
                           "Heightfield terrain is deliberately unsupported by this distance method."]),distance
