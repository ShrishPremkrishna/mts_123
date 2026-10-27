"""Verify a recovered checkpoint/export against the current task before evaluation.

Strict actor loading, current export metadata/reference comparison, 512 varied
inputs and live CPU observations are checked. This does not validate optimizer
resume, policy performance, or hardware. No robot connection is made.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np
import torch

from mjlab.scripts.pupper_foundation import sha256, source_manifest, write_json


def canonical(value):
  if callable(value): return value.__module__ + "." + value.__qualname__
  if isinstance(value, dict): return {key: canonical(item) for key, item in value.items()}
  if isinstance(value, (list, tuple)): return [canonical(item) for item in value]
  if value is None or isinstance(value, (str, int, float, bool)): return value
  return str(value)


def differences(expected, actual, prefix=""):
  if isinstance(expected, dict) and isinstance(actual, dict):
    result = []
    for key in sorted(expected.keys() | actual.keys()):
      name = f"{prefix}.{key}" if prefix else key
      if key not in expected or key not in actual: result.append(name + ": missing field")
      else: result += differences(expected[key], actual[key], name)
    return result
  if isinstance(expected, (list, tuple)) and isinstance(actual, (list, tuple)):
    a, b = np.asarray(expected), np.asarray(actual)
    if a.shape == b.shape and np.issubdtype(a.dtype, np.number) and np.issubdtype(b.dtype, np.number):
      return [] if np.allclose(a, b, rtol=0, atol=1e-7) else [prefix + ": numeric table differs"]
  return [] if expected == actual else [prefix + ": value differs"]


def audit(args):
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import RslRlVecEnvWrapper
  from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
  from mjlab.tasks.pupper.export import json_forward, to_robot_order
  from mjlab.tasks.pupper_gait.export import export_pupper_gait_policy_from_env
  from mjlab.tasks.tracking.config.pupper.export_trick import export_trick_policy_from_env

  saved = json.loads(args.policy.read_text())
  cfg, agent = load_env_cfg(args.task, play=True), load_rl_cfg(args.task)
  cfg.scene.num_envs, cfg.seed = 1, args.seed
  if "motion" in saved:
    if args.motion is None: raise ValueError("Tracking-policy audit requires its exact --motion NPZ")
    cfg.commands["motion"].motion_file = str(args.motion.resolve())
  report = {"task": args.task, "checkpoint_sha256": sha256(args.checkpoint),
            "policy_sha256": sha256(args.policy), "motion_sha256": sha256(args.motion) if args.motion else None,
            "seed": args.seed, "device": "cpu", "random_inputs": 512, "live_steps": args.steps,
            "tolerance": 1e-4, "source": source_manifest(), "training_resume_validated": False,
            "policy_performance_validated": False, "approved_for_hardware": False}
  if args.training_config:
    import yaml
    old = yaml.safe_load(args.training_config.read_text())
    old = old["env_cfg"]["value"] if "env_cfg" in old else old
    current = canonical(asdict(cfg))
    report["training_config_sha256"] = sha256(args.training_config)
    report["actor_observation_config_differences"] = differences(old["observations"]["actor"], current["observations"]["actor"])
    actuator_fields = ("stiffness", "damping", "delay_min_lag", "delay_max_lag", "velocity_limit_sim", "effort_limit_sim")
    def actuators(config):
      return [{key: value.get(key) for key in actuator_fields}
              for value in config["scene"]["entities"]["robot"]["articulation"]["actuators"]]
    report["actuator_config_old_current"] = [actuators(old), actuators(current)]
    report["actuator_config_equal"] = report["actuator_config_old_current"][0] == report["actuator_config_old_current"][1]
    report["decimation_old_current"] = [old["decimation"], current["decimation"]]
    report["timestep_old_current"] = [old["sim"]["mujoco"]["timestep"], current["sim"]["mujoco"]["timestep"]]
  env = ManagerBasedRlEnv(cfg, device="cpu")
  try:
    wrapped = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
    runner = load_runner_cls(args.task)(wrapped, asdict(agent), device="cpu")
    runner.load(str(args.checkpoint), load_cfg={"actor": True}, strict=True, map_location="cpu")
    report["strict_actor_load_passed"] = True
    actor = runner.get_inference_policy(device="cpu")
    current_export = (export_trick_policy_from_env(actor, env, saved["trick"]["name"]) if "motion" in saved
                      else export_pupper_gait_policy_from_env(actor, env))
    report["metadata_differences"] = differences({k: v for k, v in saved.items() if k != "layers"},
                                               {k: v for k, v in current_export.items() if k != "layers"})
    history, frame = saved["observation_history"], saved["single_observation_size"]
    rng = torch.Generator().manual_seed(args.seed)
    varied = torch.randn((512, history * frame), generator=rng)
    with torch.inference_mode():
      expected = actor({"actor": varied}).numpy()
      got = json_forward(saved, to_robot_order(varied, history, frame))
      report["random_input_max_abs_error"] = float(np.max(np.abs(got - expected)))
      wrapped.reset()
      obs = wrapped.get_observations()
      errors = []
      for _ in range(args.steps):
        action = actor(obs)
        exported = json_forward(saved, to_robot_order(obs["actor"], history, frame))
        errors.append(float(np.max(np.abs(exported - action.numpy()))))
        obs, *_ = wrapped.step(action)
      report["live_input_max_abs_error"] = max(errors)
    report["passed"] = (not report["metadata_differences"]
                        and not report.get("actor_observation_config_differences", [])
                        and report.get("actuator_config_equal", True)
                        and report["random_input_max_abs_error"] < report["tolerance"]
                        and report["live_input_max_abs_error"] < report["tolerance"]
                        and all(a == b for key in ("decimation_old_current", "timestep_old_current")
                                for a, b in [report.get(key, [None, None])]))
  finally:
    env.close()
  write_json(args.output, report)
  print(json.dumps({k: v for k, v in report.items() if k != "source"}, indent=2))
  return report


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--task", required=True)
  parser.add_argument("--checkpoint", type=Path, required=True)
  parser.add_argument("--policy", type=Path, required=True)
  parser.add_argument("--training-config", type=Path)
  parser.add_argument("--motion", type=Path)
  parser.add_argument("--seed", type=int, default=42)
  parser.add_argument("--steps", type=int, default=100)
  parser.add_argument("--output", type=Path, required=True)
  args = parser.parse_args()
  if args.steps < 1: parser.error("steps must be positive")
  report = audit(args)
  if not report["passed"]: raise SystemExit("Policy compatibility failed; inspect the saved differences before evaluation.")


if __name__ == "__main__":
  main()
