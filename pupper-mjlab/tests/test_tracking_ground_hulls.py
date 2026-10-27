"""Regression for collision masks after Entity collision configuration is applied."""
import mujoco
import numpy as np
import pytest

from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES, SPIDER_POSE
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper.contact_contract import enable_tracking_ground_hulls
from mjlab.tasks.tracking.config.pupper.env_cfgs import HULL_BODIES, pupper_tracking_env_cfg


@pytest.mark.parametrize("corrected", [False, True])
def test_actual_environment_masks_enable_torso_ground_contacts_only_when_requested(tmp_path, corrected):
  motion=md.build_motion([md.stand(0),md.stand(.2)]).save_npz(tmp_path/'stand.npz')
  cfg=pupper_tracking_env_cfg(str(motion),play=True);cfg.scene.num_envs=1
  if corrected:
    contract=enable_tracking_ground_hulls(cfg)
    assert contract['hull_count']==len(HULL_BODIES)
  env=ManagerBasedRlEnv(cfg,device='cpu')
  try:
    model=env.sim.mj_model
    hulls=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'robot/'+b+'_hull') for b in HULL_BODIES]
    assert min(hulls)>=0
    np.testing.assert_array_equal(model.geom_contype[hulls],0)
    np.testing.assert_array_equal(model.geom_conaffinity[hulls],1 if corrected else 0)
    names=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or '' for i in range(model.ngeom)]
    balls=[i for i,n in enumerate(names) if n.endswith('_collision')]
    np.testing.assert_array_equal(model.geom_contype[balls],1)
    np.testing.assert_array_equal(model.geom_conaffinity[balls],1)
    # Place the torso through the floor. A real contact must be generated; merely
    # observing get_spec_with_hulls() before Entity construction missed the bug.
    data=mujoco.MjData(model)
    free=int(np.where(model.jnt_type==mujoco.mjtJoint.mjJNT_FREE)[0][0]);root=int(model.jnt_qposadr[free])
    data.qpos[root:root+7]=[0,0,.02,1,0,0,0]
    for name,q in zip(JOINT_NAMES,SPIDER_POSE,strict=True):
      joint=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'robot/'+name)
      data.qpos[model.jnt_qposadr[joint]]=q
    mujoco.mj_forward(model,data)
    base=hulls[0]
    contacts=[c for c in data.contact[:data.ncon] if base in c.geom]
    assert bool(contacts)==corrected
    if corrected:
      assert min(c.dist for c in contacts)<-.005
      assert all('terrain' in [names[g] for g in c.geom] for c in contacts)
  finally:
    env.close()
  legacy=pupper_tracking_env_cfg(str(motion))
  assert legacy.scene.entities['robot'].collisions[0].disable_other_geoms
