import mujoco
import numpy as np
import pytest
from mjlab.tasks.pupper.collision_audit import flat_torso_clearance


def test_actual_pupper_mesh_plane_distance_matches_lowest_transformed_vertex():
  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import get_spec
  spec=get_spec()
  spec.worldbody.add_geom(name='audit_floor',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[1,1,.1])
  model=spec.compile();data=mujoco.MjData(model)
  torso=[g for g in range(model.ngeom) if model.geom_bodyid[g]==model.body('base_link').id][0]
  ground=model.geom('audit_floor').id
  mesh_id=model.geom_dataid[torso]
  vertices=model.mesh_vert[model.mesh_vertadr[mesh_id]:model.mesh_vertadr[mesh_id]+model.mesh_vertnum[mesh_id]]
  for z,roll,pitch in [(.2,0,0),(.03,0,0),(.05,.2,-.4),(.02,.9,.6)]:
    q=np.empty(4)
    mujoco.mju_euler2Quat(q,np.array([roll,pitch,0.]),'xyz')
    data.qpos[:7]=[0,0,z,*q]
    mujoco.mj_forward(model,data)
    world=vertices@data.geom_xmat[torso].reshape(3,3).T+data.geom_xpos[torso]
    np.testing.assert_allclose(mujoco.mj_geomDistance(model,data,torso,ground,10.,None),world[:,2].min(),atol=1e-8)


def test_signed_plane_audit_detects_overlap_even_when_body_collision_is_disabled():
  model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
    <geom name="ground" type="plane" size="1 1 .1"/>
    <body name="robot/base_link"><freejoint/><geom type="box" size=".1 .1 .05" mass="1" contype="0" conaffinity="0"/></body>
    </worldbody></mujoco>''')
  trace=dict(root_position=np.array([[[0,0,.1]],[[0,0,.02]]]),
             root_quaternion=np.tile([1,0,0,0],(2,1,1)),joint_pos=np.zeros((2,1,0)),
             valid=np.ones((2,1),bool),segment=np.zeros(2,int))
  report,distance=flat_torso_clearance(model,trace,1,np.arange(7),np.array([],dtype=int))
  np.testing.assert_allclose(distance[:,0],[.05,-.03],atol=1e-8)
  assert report['segments'][0]['penetrated_valid_sampled_states']==1
  assert model.geom_contype[1]==model.geom_conaffinity[1]==0


def test_signed_audit_rejects_unvalidated_heightfield_distance():
  model=mujoco.MjModel.from_xml_string('''<mujoco><asset><hfield name="h" nrow="2" ncol="2" size="1 1 .1 .1"/></asset><worldbody>
    <geom type="hfield" hfield="h"/><body name="robot/base_link"><freejoint/><geom type="box" size=".1 .1 .05" mass="1"/></body>
    </worldbody></mujoco>''')
  with pytest.raises(ValueError,match='heightfields'):
    flat_torso_clearance(model,{},1,np.arange(7),np.array([],dtype=int))
