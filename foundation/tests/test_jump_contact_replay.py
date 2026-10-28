"""Independent saved-pose replay tests; no synthetic motion is used for training."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('jump_contact_audit_test',ROOT/'foundation/scripts/audit_jump_capture_contacts.py')
audit=importlib.util.module_from_spec(spec);sys.modules[spec.name]=audit;spec.loader.exec_module(audit)


def test_geometry_identity_rejects_different_contact_masks():
    helper=audit.load(ROOT/'foundation/scripts/check_spider_handoff.py','contact_replay_model')
    model,_=helper.make_model('torso_only')
    rows=audit.normalized_records(model)
    record=dict(robot_geometries=[dict(row,id=i,torso=False) for i,row in enumerate(rows)],terrain_heightfield_count=0)
    assert audit.verify_collision_identity(model,record)
    record['robot_geometries'][1]['conaffinity']=0
    with pytest.raises(ValueError,match='do not match'):audit.verify_collision_identity(model,record)


def test_geometric_replay_com_and_contact_preserve_input_motion():
    helper=audit.load(ROOT/'foundation/scripts/check_spider_handoff.py','contact_replay_pose_model')
    model,_=helper.make_model('torso_only')
    names=[model.body(i).name for i in range(1,model.nbody)]
    masses=model.body_mass[1:].copy()
    t=dict(joint_pos=np.zeros((3,1,12)),root_position=np.array([[[0,0,1]],[[0,0,2]],[[0,0,3]]],float),
           root_quaternion=np.broadcast_to([1.,0,0,0],(3,1,4)).copy(),dt=np.asarray(.02))
    original={k:v.copy() for k,v in t.items()}
    result,geoms=audit.audit_poses(model,t,masses,names,np.zeros(3))
    assert len(geoms)==9 and not result['robot_ground_contact_count'].any()
    assert not result['audited_foot_contact'].any()
    np.testing.assert_allclose(np.diff(result['robot_com_height_m'][:,0]),1.,atol=1e-12)
    np.testing.assert_allclose(result['robot_com_vertical_velocity_m_s'],50.,atol=1e-10)
    for key,value in original.items():np.testing.assert_array_equal(t[key],value)
