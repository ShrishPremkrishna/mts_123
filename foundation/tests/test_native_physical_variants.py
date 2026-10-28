"""Mechanical corner tests verify realized constants, not only requested flags."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest
import mujoco
spec=importlib.util.spec_from_file_location('variant_handoff',Path(__file__).resolve().parents[1]/'scripts/check_spider_handoff.py')
handoff=importlib.util.module_from_spec(spec);spec.loader.exec_module(handoff)


def variant(**changes):
    value=dict(schema=1,name='test',base_com_offset_m=[0.,0.,0.],uniform_density_scale=1.,foot_friction=None,foot_material='sphere_only')
    value.update(changes);return value


@pytest.mark.parametrize('change',[
    {'schema':True},{'unknown':1},{'name':'../bad'},
    {'base_com_offset_m':[.02001,0,0]},{'base_com_offset_m':[0,.00501,0]},
    {'base_com_offset_m':[0,0,float('nan')]},{'base_com_offset_m':[True,0,0]},
    {'uniform_density_scale':1.31},{'uniform_density_scale':.89},{'uniform_density_scale':float('inf')},
    {'foot_friction':.59},{'foot_friction':1.41},{'foot_friction':True},
    {'foot_material':'unknown'},{'foot_material':'foot_end_union'},
])
def test_invalid_or_outside_contract_variants_refused(change):
    with pytest.raises(ValueError):handoff.validate_physical_variant(variant(**change))


def test_realized_density_and_com_are_composed_and_derived_mass_recomputed():
    model,_=handoff.make_model('full_hulls');base=model.body('base_link').id
    mass=model.body_mass.copy();inertia=model.body_inertia.copy();com=model.body_ipos.copy();q=model.body_iquat.copy()
    mask=model.geom_conaffinity.copy();friction=model.geom_friction.copy()
    report=handoff.apply_physical_variant(model,variant(base_com_offset_m=[.02,.005,.005],uniform_density_scale=1.3))
    np.testing.assert_allclose(model.body_mass,mass*1.3,rtol=1e-14)
    np.testing.assert_allclose(model.body_inertia,inertia*1.3,rtol=1e-14)
    expected=com.copy();expected[base]+=[.02,.005,.005]
    np.testing.assert_array_equal(model.body_ipos,expected)
    np.testing.assert_array_equal(model.body_iquat,q);np.testing.assert_array_equal(model.geom_conaffinity,mask)
    np.testing.assert_array_equal(model.geom_friction,friction)
    assert model.body_subtreemass[base]==pytest.approx(model.body_mass.sum())
    assert report['realized']['total_mass_kg']==pytest.approx(report['before']['total_mass_kg']*1.3)


@pytest.mark.parametrize('material',['sphere_only','foot_end_union'])
def test_foot_material_is_applied_only_to_named_geoms(material):
    model,_=handoff.make_model('full_hulls');old=model.geom_friction.copy();priority=model.geom_priority.copy()
    handoff.apply_physical_variant(model,variant(foot_friction=.6,foot_material=material))
    for i in range(model.ngeom):
        name=model.geom(i).name
        touched=name.startswith('leg_') and (name.endswith('_3_collision') or (material=='foot_end_union' and name.endswith('_3_hull')))
        if touched:
            assert model.geom_friction[i,0]==.6
            assert model.geom_priority[i]==1
            np.testing.assert_array_equal(model.geom_friction[i,1:],old[i,1:])
        else:
            np.testing.assert_array_equal(model.geom_friction[i],old[i]);assert model.geom_priority[i]==priority[i]


def test_explicit_identity_variant_preserves_nominal_physics():
    plain,_=handoff.make_model('full_hulls');altered,_=handoff.make_model('full_hulls')
    handoff.apply_physical_variant(altered,variant())
    a,b=mujoco.MjData(plain),mujoco.MjData(altered)
    for d in [a,b]:
        d.qpos[:7]=[0,0,.18,1,0,0,0]
        d.qvel[:6]=[.01,-.02,.01,.01,.02,-.01]
    for _ in range(200):
        mujoco.mj_step(plain,a);mujoco.mj_step(altered,b)
        np.testing.assert_array_equal(a.qpos,b.qpos);np.testing.assert_array_equal(a.qvel,b.qvel)
        np.testing.assert_array_equal(a.qacc_warmstart,b.qacc_warmstart)
