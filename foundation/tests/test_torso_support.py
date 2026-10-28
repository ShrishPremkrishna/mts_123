from pathlib import Path
import math
import numpy as np
import pytest
import torch
from mjlab.tasks.pupper.torso_support import (normalized_torso_load,enable_torso_support_audit,apply_torso_support_penalty,nominal_mechanics,SENSOR_NAME)
from mjlab.tasks.registry import load_env_cfg
from mjlab.tasks.pupper.locomotion_contact_contract import enable_locomotion_torso_contact


def test_normal_support_averages_physics_substeps_and_actual_mass():
    history=torch.zeros(2,1,5,3);history[0,0,:,2]=torch.tensor([0.,10.,20.,30.,40.]);history[1,0,:,2]=-20.
    history[...,0]=1000. # tangential load is not vertical support
    assert torch.allclose(normalized_torso_load(history,torch.tensor([20.,40.])),torch.tensor([1.,.5]))


def test_dimensions_rejected():
    with pytest.raises(ValueError):normalized_torso_load(torch.zeros(2,1,3),torch.ones(2))
    with pytest.raises(ValueError):normalized_torso_load(torch.zeros(2,1,5,3),torch.ones(1))


def test_sensor_preserves_physics_and_actor_contract():
    cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3');enable_locomotion_torso_contact(cfg)
    original=cfg.scene.entities['robot'];collisions=original.collisions;observations=cfg.observations['actor'];old=len(cfg.scene.sensors)
    meta=enable_torso_support_audit(cfg);sensor=cfg.scene.sensors[-1]
    assert sensor.name==SENSOR_NAME and sensor.history_length==cfg.decimation and sensor.reduce=='netforce'
    assert cfg.scene.entities['robot'] is original and original.collisions==collisions and cfg.observations['actor'] is observations
    assert len(cfg.scene.sensors)==old+1 and meta['physical_model_unchanged']
    with pytest.raises(ValueError,match='already'):enable_torso_support_audit(cfg)


def test_missing_hull_refuses_false_zero_force():
    cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3')
    with pytest.raises(ValueError,match='hull'):enable_torso_support_audit(cfg)


@pytest.mark.parametrize('weight',[float('nan'),float('inf'),.01,-5.01,True])
def test_bad_reward_weight_refused_without_mutation(weight):
    cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3');enable_locomotion_torso_contact(cfg);n=len(cfg.scene.sensors)
    with pytest.raises(ValueError):apply_torso_support_penalty(cfg,weight)
    assert len(cfg.scene.sensors)==n and 'torso_support' not in cfg.rewards


def test_zero_weight_control_and_nominal_diagnostic_are_explicit():
    cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3');enable_locomotion_torso_contact(cfg)
    meta=apply_torso_support_penalty(cfg,0.)
    assert meta['reward_weight']==0. and cfg.rewards['torso_support'].weight==0.
    obs=cfg.observations['actor'];events=set(cfg.events);audit=nominal_mechanics(cfg)
    assert set(audit['removed_events'])==set(('foot_friction','encoder_bias','base_com','pd_gains','inertial'))
    assert set(cfg.events)==events-set(audit['removed_events']) and cfg.observations['actor'] is obs


@pytest.mark.parametrize('gravity,mass',[(np.array([1.,0.,-9.81]),3.),(np.array([0.,0.,9.81]),3.),(np.array([0.,0.,-9.81]),0.),(np.array([0.,0.,-9.81]),float('nan'))])
def test_invalid_gravity_or_actual_mass_refused(gravity,mass):
    from types import SimpleNamespace as S
    from mjlab.tasks.pupper.torso_support import torso_support_diagnostics
    env=S(num_envs=1,scene={'robot':S(indexing=S(body_ids=[0])),SENSOR_NAME:S(data=S(force_history=torch.zeros(1,1,5,3),force=torch.zeros(1,1,3)))},sim=S(mj_model=S(opt=S(gravity=gravity)),model=S(body_mass=torch.tensor([[mass]]))))
    with pytest.raises(ValueError):torso_support_diagnostics(env)
