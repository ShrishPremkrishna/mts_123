"""Exercise the training launcher without GPU optimization or modifying defaults."""
import argparse
import json
from pathlib import Path

import pytest
import torch

from mjlab.scripts import pupper_foundation as foundation
from mjlab.scripts import train as training
from mjlab.tasks.registry import load_env_cfg


@pytest.mark.parametrize("clamp,compose,hulls,com_x", [
  (False, False, False, None), (True, False, False, None),
  (True, True, False, None), (True, True, True, None), (True, True, True, .02),
])
def test_tracking_finetune_keeps_frozen_motion_and_explicit_contracts(tmp_path, monkeypatch, clamp, compose, hulls, com_x):
  monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0")
  monkeypatch.setattr(torch.cuda, "is_available", lambda: True)
  monkeypatch.setattr(torch.cuda, "get_device_name", lambda device: "mocked launcher test, no GPU")
  monkeypatch.setattr(foundation, "source_manifest", lambda: {"test": True})
  monkeypatch.setattr(foundation, "prepare", lambda *_: pytest.fail("Explicit motion must not be regenerated"))
  motion = tmp_path / "frozen-baseline.npz"
  motion.write_bytes(b"launcher treats frozen reference as immutable bytes")
  checkpoint = tmp_path / "baseline.pt"
  checkpoint.write_bytes(b"immutable checkpoint bytes")
  captured = {}
  def fake_train(task, config, run_dir):
    captured.update(task=task, config=config, run_dir=run_dir)
    (run_dir / "model_3999.pt").write_bytes(b"mocked output, no training occurred")
    (run_dir / "policy.json").write_text("{}")
  monkeypatch.setattr(training, "run_train", fake_train)
  args = argparse.Namespace(profile="spider_clips", motion=motion, output=tmp_path / "run",
                            deployment_target_clamp=clamp, compose_com_randomization=compose, tracking_ground_hulls=hulls,
                            tracking_com_x_half_range=com_x,
                            num_envs=64, iterations=1000, seed=42, logger="tensorboard", project="test",
                            initial_checkpoint=checkpoint, resume_run=None, resume_checkpoint=None)
  run = foundation.train(args)
  cfg = captured["config"].env
  agent = captured["config"].agent
  assert Path(cfg.commands["motion"].motion_file).read_bytes() == motion.read_bytes()
  assert Path(cfg.commands["motion"].motion_file).parent == run
  assert (run / "initial_checkpoint.pt").read_bytes() == checkpoint.read_bytes()
  assert agent.resume and agent.max_iterations == 1000
  assert agent.load_checkpoint == "initial_checkpoint\\.pt$"
  assert (cfg.actions["joint_pos"].clip is not None) == clamp
  assert cfg.scene.entities["robot"].collisions[0].disable_other_geoms != hulls
  order = list(cfg.events)
  assert order.index("base_com") < order.index("inertial")
  if compose:
    from mjlab.tasks.pupper.deployment_contract import density_randomization_preserving_com
    assert cfg.events["inertial"].func is density_randomization_preserving_com
  report = json.loads((run / "foundation-run.json").read_text())
  assert report["initial_checkpoint_sha256"] == foundation.sha256(checkpoint)
  assert report["motion_sha256"] == foundation.sha256(motion)
  assert ("_deployclamp_" in run.name) == clamp
  assert ("_composedcom_" in run.name) == compose
  assert ("_groundhulls_" in run.name) == hulls
  assert not report["approved_for_hardware"]
  default = load_env_cfg("Mjlab-Tracking-Flat-Pupper-v3")
  assert default.actions["joint_pos"].clip is None
  assert list(default.events).index("base_com") < list(default.events).index("inertial")
  course_ranges = {0: (-.01, .01), 1: (-.01, .01), 2: (-.01, .01)}
  assert default.events["base_com"].params["ranges"] == course_ranges
  assert {name: (term.weight, term.params) for name, term in cfg.rewards.items()} == {
    name: (term.weight, term.params) for name, term in default.rewards.items()}
  assert report["reward_parameter_overrides"] == {}
  if com_x is None:
    assert report["domain_randomization_overrides"] == {}
    assert cfg.events["base_com"].params["ranges"] == course_ranges
  else:
    assert cfg.events["base_com"].params["ranges"] == {**course_ranges, 0: (-.02, .02)}
    override = report["domain_randomization_overrides"]["base_com.ranges"]
    assert override["previous"] == {"0": [-.01, .01], "1": [-.01, .01], "2": [-.01, .01]}
    assert override["value"] == {**override["previous"], "0": [-.02, .02]}
    assert override["changed_axes"] == ["x"] and override["rng_draw_order_preserved"]
    assert "_comx0p02_" in run.name


@pytest.mark.parametrize("value", [0., -.01, .020001, float("nan"), float("inf"), True])
def test_com_x_override_rejects_unbounded_or_invalid_values(value):
  cfg = load_env_cfg("Mjlab-Tracking-Flat-Pupper-v3")
  with pytest.raises(ValueError, match="finite and in"):
    foundation.set_tracking_com_x_half_range(cfg, value)


def test_com_x_override_requires_composition_and_untouched_course_axes():
  from mjlab.tasks.pupper.deployment_contract import compose_pupper_com_randomization
  cfg = load_env_cfg("Mjlab-Tracking-Flat-Pupper-v3")
  with pytest.raises(ValueError, match="compose-com-randomization"):
    foundation.set_tracking_com_x_half_range(cfg, .02)
  compose_pupper_com_randomization(cfg)
  cfg.events["base_com"].params["ranges"][1] = (-.02, .02)
  with pytest.raises(ValueError, match="unchanged course"):
    foundation.set_tracking_com_x_half_range(cfg, .02)


@pytest.mark.parametrize("mutation", ["locomotion", "other_body", "other_distribution", "other_event"])
def test_com_x_override_refuses_different_sampling_contract(mutation):
  from mjlab.tasks.pupper.deployment_contract import compose_pupper_com_randomization
  cfg = load_env_cfg("Mjlab-Tracking-Flat-Pupper-v3")
  compose_pupper_com_randomization(cfg)
  if mutation == "locomotion":
    cfg.commands.pop("motion")
  elif mutation == "other_body":
    cfg.events["base_com"].params["asset_cfg"].body_names = ("base_link", "leg_front_r_1")
  elif mutation == "other_distribution":
    cfg.events["base_com"].params["distribution"] = "gaussian"
  else:
    cfg.events["base_com"].func = lambda *_: None
  with pytest.raises(ValueError, match="requires"):
    foundation.set_tracking_com_x_half_range(cfg, .02)


@pytest.mark.parametrize('linear_std',[None,0.08])
@pytest.mark.parametrize('slew_rate',[None,15.])
def test_spiderwalk_contact_continuation_keeps_full_original_reward_profile(tmp_path, monkeypatch,linear_std,slew_rate):
  monkeypatch.setenv('CUDA_VISIBLE_DEVICES','0')
  monkeypatch.setattr(torch.cuda,'is_available',lambda:True)
  monkeypatch.setattr(torch.cuda,'get_device_name',lambda _: 'mock launcher test')
  monkeypatch.setattr(foundation,'source_manifest',lambda:{'test':True})
  captured={}
  def fake_train(task,config,run_dir):
    captured.update(task=task,config=config)
    (run_dir/'model_4999.pt').write_bytes(b'test fixture, no training')
    (run_dir/'policy.json').write_text('{}')
  monkeypatch.setattr(training,'run_train',fake_train)
  checkpoint=tmp_path/'original-4000.pt';checkpoint.write_bytes(b'original fixture bytes')
  args=argparse.Namespace(profile='spider_walk',motion=None,output=tmp_path/'continuation',
                         deployment_target_clamp=True,compose_com_randomization=True,
                         tracking_ground_hulls=False,locomotion_torso_contact=True,
                         num_envs=64,iterations=1500,seed=42,logger='tensorboard',project='test',
                         initial_checkpoint=checkpoint,resume_run=None,resume_checkpoint=None)
  args.linear_tracking_std=linear_std
  args.target_slew_rad_s=slew_rate
  if slew_rate is not None:
    args.driver_limits=Path(__file__).resolve().parents[2]/'foundation/configs/class-driver-joint-limits.json'
  run=foundation.train(args)
  cfg=captured['config'].env
  assert captured['task']=='Mjlab-SpiderWalk-Flat-Pupper-v3'
  profile=foundation.load_profile('spider_walk')
  for name,term in cfg.rewards.items():
    assert term.weight==profile['rewards'].get(name,0.)
  assert cfg.rewards['base_height'].weight==3.
  assert cfg.rewards['termination'].weight==-5.
  from mjlab.tasks.pupper.locomotion_contact_contract import get_spec_with_torso_contact
  assert cfg.scene.entities['robot'].spec_fn is get_spec_with_torso_contact
  assert cfg.actions['joint_pos'].clip is not None
  record=json.loads((run/'foundation-run.json').read_text())
  assert record['collision_contract']['contract']=='locomotion_torso_contact_v1'
  assert record['com_randomization_contract']['rng_draw_order_preserved']
  assert '_torsocontact_' in run.name
  assert record['motion_file'] is None
  assert record['initial_checkpoint_sha256']==foundation.sha256(checkpoint)
  if slew_rate is not None:
    from mjlab.tasks.pupper.target_slew import SlewLimitedJointPositionActionCfg
    assert isinstance(cfg.actions['joint_pos'],SlewLimitedJointPositionActionCfg)
    assert record['position_target_slew_contract']['max_rate_rad_s']==slew_rate
    assert record['position_target_slew_contract']['driver_limits_sha256']==foundation.sha256(args.driver_limits)
    assert '_slew15_' in run.name
  else:
    assert record['position_target_slew_contract'] is None
  if linear_std is None:
    assert record['reward_parameter_overrides']=={}
    assert cfg.rewards['track_linear_velocity'].params['std']==pytest.approx(.1**.5)
  else:
    assert cfg.rewards['track_linear_velocity'].params['std']==linear_std
    assert '_linstd0p08_' in run.name
    override=record['reward_parameter_overrides']['track_linear_velocity.std']
    assert override['value']==linear_std and override['previous']==pytest.approx(.1**.5)
    assert override['weight_unchanged']==10.
  assert load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3').rewards['track_linear_velocity'].params['std']==pytest.approx(.1**.5)


@pytest.mark.parametrize('value',[0.,-1.,float('nan'),float('inf'),True])
def test_linear_tracking_override_rejects_invalid_values(value):
  cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3')
  with pytest.raises(ValueError,match='positive and finite'):
    foundation.set_linear_tracking_std(cfg,value)


def test_linear_tracking_override_refuses_other_reward_or_inactive_term():
  cfg=load_env_cfg('Mjlab-SpiderWalk-Flat-Pupper-v3')
  with pytest.raises(ValueError,match='active standard'):
    foundation.set_linear_tracking_std(cfg,.08)
  cfg.rewards['track_linear_velocity'].weight=10.
  cfg.rewards['track_linear_velocity'].func=lambda *_:None
  with pytest.raises(ValueError,match='active standard'):
    foundation.set_linear_tracking_std(cfg,.08)
