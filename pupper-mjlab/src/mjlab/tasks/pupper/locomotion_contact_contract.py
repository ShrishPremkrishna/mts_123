"""Opt-in torso contact for walking; preserve the original knee/foot spheres.

This is a new physical-model experiment, not a silent change to course policies.
It addresses measured torso-through-floor trajectories in SpiderWalk without
adding the tracking model's eight lower-leg hulls.
"""
import dataclasses

import mujoco

from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import get_spec
from mjlab.utils.spec_config import CollisionCfg


def get_spec_with_torso_contact():
  spec=get_spec()
  body=spec.body("base_link")
  visual=next(g for g in body.geoms if g.type==mujoco.mjtGeom.mjGEOM_MESH)
  body.add_geom(name="base_link_hull",type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname=visual.meshname,pos=visual.pos,quat=visual.quat,
                contype=0,conaffinity=1,condim=3,group=3,density=0,
                friction=(0.8,0.02,0.01))
  # Match the existing tracking torso hull's bounded convex approximation.
  spec.mesh(visual.meshname).maxhullvert=32
  return spec


def enable_locomotion_torso_contact(cfg):
  robot=cfg.scene.entities["robot"]
  if robot.spec_fn not in (get_spec,get_spec_with_torso_contact):
    raise ValueError("Locomotion torso contact requires the original Pupper walking model")
  robot.spec_fn=get_spec_with_torso_contact
  # Preserve exact existing sphere properties. Appending a second CollisionCfg
  # with its default disable_other_geoms=True would silently disable the balls.
  existing=tuple(c for c in robot.collisions if c.geom_names_expr!=(r"base_link_hull$",))
  robot.collisions=tuple(dataclasses.replace(c,disable_other_geoms=False) for c in existing)+(
    CollisionCfg(geom_names_expr=(r"base_link_hull$",),contype=0,conaffinity=1,
                 condim=3,friction=(0.8,0.02,0.01),disable_other_geoms=False),
  )
  return {"contract":"locomotion_torso_contact_v1","hull_count":1,
          "hull_body":"base_link","maximum_hull_vertices":32,
          "contype":0,"conaffinity":1,"density":0,
          "scope":"Torso contacts terrain and nonadjacent active knee/foot spheres; no lower-leg hulls added. Original masses, sphere geometry and sphere collision properties retained."}
