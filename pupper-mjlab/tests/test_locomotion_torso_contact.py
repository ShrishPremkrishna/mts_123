import numpy as np
import mujoco
import torch

from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import get_spec
from mjlab.envs import ManagerBasedRlEnv
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.registry import load_env_cfg
from mjlab.tasks.pupper.locomotion_contact_contract import (
  get_spec_with_torso_contact, enable_locomotion_torso_contact,
)


def test_torso_only_hull_preserves_mass_inertia_and_legacy_spheres():
  old=get_spec().compile();new=get_spec_with_torso_contact().compile()
  np.testing.assert_array_equal(old.body_mass,new.body_mass)
  np.testing.assert_array_equal(old.body_inertia,new.body_inertia)
  np.testing.assert_array_equal(old.body_ipos,new.body_ipos)
  assert new.ngeom==old.ngeom+1
  for gid in range(old.ngeom):
    name=old.geom(gid).name
    if name and name.endswith('_collision'):
      peer=new.geom(name).id
      for attr in ('geom_size','geom_pos','geom_quat','geom_contype','geom_conaffinity','geom_friction'):
        np.testing.assert_array_equal(getattr(old,attr)[gid],getattr(new,attr)[peer])


def test_actual_compiled_torso_contact_survives_config_and_generates_ground_force():
  cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3',play=True)
  cfg.scene.num_envs=1;cfg.seed=101
  enable_locomotion_torso_contact(cfg)
  first=cfg.scene.entities['robot'].collisions
  contract=enable_locomotion_torso_contact(cfg)
  assert cfg.scene.entities['robot'].collisions==first
  assert contract['hull_count']==1
  cfg.scene.sensors=tuple(cfg.scene.sensors)+(ContactSensorCfg(
    name='audit_torso_ground',primary=ContactMatch(mode='geom',pattern='base_link_hull',entity='robot'),
    secondary=ContactMatch(mode='body',pattern='terrain'),fields=('found','force'),reduce='netforce',num_slots=1,
  ),)
  env=ManagerBasedRlEnv(cfg,device='cpu')
  try:
    robot=env.scene['robot'];m=env.sim.mj_model
    ids=robot.indexing.geom_ids.cpu().tolist()
    active=[g for g in ids if m.geom_contype[g] or m.geom_conaffinity[g]]
    hull=m.geom('robot/base_link_hull').id
    assert len(active)==9
    assert (m.geom_contype[hull],m.geom_conaffinity[hull])==(0,1)
    balls=[g for g in ids if m.geom(g).name.endswith('_collision')]
    assert len(balls)==8
    assert all((m.geom_contype[g],m.geom_conaffinity[g])==(1,1) for g in balls)
    root=robot.data.default_root_state.clone();root[:,:3]+=env.scene.env_origins
    root[:,2]=.02
    # Turn the deliberately overlapping probe upside down so the foot/knee
    # spheres do not take all the separating load before the torso constraint.
    root[:,3:7]=torch.tensor([0.,1.,0.,0.])
    robot.write_root_state_to_sim(root)
    robot.write_joint_state_to_sim(robot.data.default_joint_pos,torch.zeros_like(robot.data.default_joint_pos))
    env.scene.write_data_to_sim();env.sim.forward();env.scene.update(env.physics_dt)
    sensor=env.scene['audit_torso_ground'].data
    assert sensor.found.sum()>0
    assert torch.linalg.vector_norm(sensor.force)>0
    data=mujoco.MjData(m)
    data.qpos[robot.indexing.free_joint_q_adr.cpu().numpy()]=root[0,:7].cpu().numpy()
    data.qpos[robot.indexing.joint_q_adr.cpu().numpy()]=robot.data.default_joint_pos[0].cpu().numpy()
    mujoco.mj_forward(m,data)
    assert any(hull in (c.geom1,c.geom2) for c in data.contact)
  finally:
    env.close()
