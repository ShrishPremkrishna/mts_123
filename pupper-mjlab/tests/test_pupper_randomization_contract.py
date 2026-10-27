"""Verify physical model fields after all startup events, including reset persistence."""
import math
import numpy as np
import pytest

from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
from mjlab.tasks.pupper.deployment_contract import compose_pupper_com_randomization
from mjlab.tasks.pupper.randomization_audit import snapshot_domain_randomization


def configured(play=True):
  cfg = load_env_cfg("Mjlab-MixedGaits-Flat-Pupper-v3", play=play)
  cfg.scene.num_envs = 3
  cfg.seed = 123
  cfg.events.pop("hold_base_airborne", None)
  cfg.curriculum.clear()
  cfg.events["base_com"].params["ranges"] = {0:(.02,.02), 1:(-.003,-.003), 2:(.004,.004)}
  cfg.events["inertial"].params["alpha_range"] = (math.log(1.2)/2,)*2
  cfg.events["foot_friction"].params["ranges"] = (.63,.63)
  cfg.events["encoder_bias"].params["bias_range"] = (.013,.013)
  cfg.events["pd_gains"].params["kp_range"] = (.7,.7)
  cfg.events["pd_gains"].params["kd_range"] = (1.2,1.2)
  return cfg


@pytest.mark.parametrize("composed", [False, True])
def test_real_fields_preserve_mass_gains_friction_bias_and_expose_com_order(composed):
  cfg = configured()
  if composed:
    compose_pupper_com_randomization(cfg)
  env = ManagerBasedRlEnv(cfg,device="cpu")
  try:
    report, fields = snapshot_domain_randomization(env)
    np.testing.assert_allclose(fields["body_mass_ratio"],1.2,atol=2e-5)
    np.testing.assert_allclose(fields["base_com_offset_m"],np.tile([.02,-.003,.004] if composed else [0,0,0],(3,1)),atol=1e-6)
    np.testing.assert_allclose(fields["foot_tangential_friction"],.63,atol=1e-6)
    # Feet have priority over the floor, so floor friction does not mask low foot friction.
    assert (fields["foot_geom_priority"] > 0).all()
    assert fields["foot_geom_priority"].min() > fields["external_geom_priority"].max()
    np.testing.assert_allclose(fields["encoder_bias_rad"],.013,atol=1e-6)
    np.testing.assert_allclose(fields["pd_stiffness"],5.5*.7,atol=1e-6)
    np.testing.assert_allclose(fields["pd_damping"],.25*1.2,atol=1e-6)
    assert bool(report["warnings"]) == (not composed)
    env.reset()
    _, after = snapshot_domain_randomization(env)
    for name in ("base_com_offset_m","body_mass_ratio","foot_tangential_friction","encoder_bias_rad","pd_stiffness","pd_damping"):
      np.testing.assert_array_equal(after[name],fields[name])
  finally:
    env.close()


def test_composition_rejects_transforms_whose_com_would_be_discarded():
  cfg = configured()
  cfg.events["inertial"].params["t1_range"] = (.01,.02)
  with pytest.raises(ValueError,match="cannot overwrite"):
    compose_pupper_com_randomization(cfg)


def test_play_config_already_has_physics_randomization_and_observation_noise():
  for play in (False,True):
    cfg = configured(play)
    assert all(cfg.events[name].mode == "startup" for name in ("base_com","inertial","foot_friction","pd_gains","encoder_bias"))
    actor = cfg.observations["actor"]
    assert any(term.params.get("add_noise") for term in actor.terms.values())


def test_corrected_com_preserves_other_same_seed_physics_samples_exactly():
  fields = []
  for composed in (False,True):
    cfg = load_env_cfg("Mjlab-MixedGaits-Flat-Pupper-v3",play=True)
    cfg.scene.num_envs=4;cfg.seed=543
    order=list(cfg.events)
    if composed:
      contract=compose_pupper_com_randomization(cfg)
      assert contract["rng_draw_order_preserved"]
      assert list(cfg.events)==order
    env=ManagerBasedRlEnv(cfg,device="cpu")
    try:fields.append(snapshot_domain_randomization(env)[1])
    finally:env.close()
  for name in ("body_mass_kg","body_mass_ratio","foot_tangential_friction","encoder_bias_rad","pd_stiffness","pd_damping"):
    np.testing.assert_array_equal(fields[0][name],fields[1][name])
  np.testing.assert_allclose(fields[0]["base_com_offset_m"],0,atol=1e-6)
  assert np.abs(fields[1]["base_com_offset_m"]).max()>.001
