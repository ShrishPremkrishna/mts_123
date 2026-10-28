"""Contact-point measurements distinguish actual surface motion from base motion."""
import importlib.util
from pathlib import Path
import mujoco
import numpy as np

spec=importlib.util.spec_from_file_location('native_contact_handoff',Path(__file__).resolve().parents[1]/'scripts/check_spider_handoff.py')
handoff=importlib.util.module_from_spec(spec);spec.loader.exec_module(handoff)


def world():
    model=mujoco.MjModel.from_xml_string('''<mujoco><option timestep="0.002" integrator="implicitfast"/>
    <worldbody><geom name="terrain" type="plane" size="2 2 .1"/>
    <body name="moving" pos="0 0 .0999"><freejoint/><geom name="body" type="box" size=".1 .1 .1" mass="2"/></body></worldbody></mujoco>''')
    return model,mujoco.MjData(model)


def test_translation_and_rotation_use_actual_contact_point():
    model,data=world();data.qvel[:]=[1.2,-.7,.3,.4,-.2,.6];mujoco.mj_forward(model,data)
    rows=handoff.contact_point_telemetry(model,data);assert len(rows)==4
    for row in rows:
        relative=np.array(row['position_world_m'])-data.xpos[model.body('moving').id]
        expected=data.qvel[:3]+np.cross(data.qvel[3:6],relative)
        # Ground is geom1 in this scene, so relative velocity is moving-ground.
        np.testing.assert_allclose(row['relative_velocity_geom2_minus_geom1_world_m_s'],expected,atol=1e-12)
        normal=np.array(row['normal_world']);tangent=expected-normal*np.dot(normal,expected)
        np.testing.assert_allclose(row['tangential_relative_velocity_world_m_s'],tangent,atol=1e-12)
        np.testing.assert_allclose(row['tangential_speed_m_s'],np.linalg.norm(tangent),atol=1e-12)
    assert np.ptp([r['tangential_speed_m_s'] for r in rows])>.05


def test_rolling_point_can_be_still_while_base_translates():
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody><geom name="terrain" type="plane" size="2 2 .1"/>
    <body pos="0 0 .1"><freejoint/><geom name="body" type="sphere" size=".1"/></body></worldbody></mujoco>''')
    data=mujoco.MjData(model);data.qvel[:]=[1,0,0,0,10,0];mujoco.mj_forward(model,data)
    rows=handoff.contact_point_telemetry(model,data);assert len(rows)==1
    assert rows[0]['tangential_speed_m_s']<1e-12
    assert np.linalg.norm(data.qvel[:2])==1


def test_sampling_is_exact_preintegration_and_preserves_live_physics():
    model,plain=world();sampled=mujoco.MjData(model);scratch=mujoco.MjData(model)
    plain.qvel[:]=sampled.qvel[:]=[.6,-.2,.1,.2,-.1,.3]
    for step in range(200):
        before_qpos=sampled.qpos.copy();before_qvel=sampled.qvel.copy();before_warmstart=sampled.qacc_warmstart.copy()
        rows=handoff.sample_contact_state(model,sampled,scratch)
        np.testing.assert_array_equal(sampled.qpos,before_qpos);np.testing.assert_array_equal(sampled.qvel,before_qvel)
        np.testing.assert_array_equal(sampled.qacc_warmstart,before_warmstart)
        for row in rows:assert np.isfinite(row['tangential_speed_m_s'])
        if step==0:
            assert rows and any(r['tangential_speed_m_s']>.4 for r in rows)
            np.testing.assert_array_equal(scratch.qvel,before_qvel)
        mujoco.mj_step(model,plain);mujoco.mj_step(model,sampled)
        np.testing.assert_array_equal(plain.qpos,sampled.qpos);np.testing.assert_array_equal(plain.qvel,sampled.qvel)
        np.testing.assert_array_equal(plain.qacc_warmstart,sampled.qacc_warmstart)
