"""Reproducible engineering runs for Lab 6; checkpoints are never evidence alone.

Examples (run from pupper-mjlab with its environment installed):
  python -m mjlab.scripts.pupper_foundation prepare --output /persistent/foundation
  python -m mjlab.scripts.pupper_foundation train sit_stay_shake --output /persistent/foundation
  python -m mjlab.scripts.pupper_foundation train spider_clips --output /persistent/foundation
  python -m mjlab.scripts.pupper_foundation train spider_walk --output /persistent/foundation
  python -m mjlab.scripts.pupper_foundation score --motion FILE --checkpoint FILE --output DIR

Use --num-envs 64 --iterations 2 --logger tensorboard for a GPU smoke test.
Profiles specify starting hypotheses, not tuned or hardware-approved policies.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
import os
import re
import subprocess
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[3]
PROFILE_FILE = PROJECT / "configs" / "foundation" / "training.json"


def sha256(path: Path) -> str:
  return hashlib.sha256(path.read_bytes()).hexdigest()


def source_manifest() -> dict:
  """Capture exact source contents even before a working tree has been committed."""
  files = [p for p in (PROJECT / "src").rglob("*") if p.suffix in (".py", ".xml", ".npz")]
  files += [PROJECT / "pyproject.toml", PROFILE_FILE]
  hashes = {str(p.relative_to(PROJECT)): sha256(p) for p in sorted(files)}
  revision = subprocess.run(
    ["git", "rev-parse", "HEAD"], cwd=PROJECT, capture_output=True, text=True, check=False
  )
  versions = {}
  for name in ("torch", "mujoco", "mujoco-warp", "warp-lang", "rsl-rl-lib", "numpy"):
    try:
      versions[name] = importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
      versions[name] = None
  return {
    "git_revision": revision.stdout.strip() or None,
    "source_sha256": hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest(),
    "files": hashes,
    "packages": versions,
    "python": sys.version,
  }


def write_json(path: Path, data: dict) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_suffix(path.suffix + ".tmp")
  temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
  temporary.replace(path)


def load_profile(name: str) -> dict:
  profiles = json.loads(PROFILE_FILE.read_text())
  if name not in profiles:
    raise ValueError(f"unknown profile {name}; choose from {list(profiles)}")
  return profiles[name]


def prepare(output: Path) -> dict[str, Path]:
  from mjlab.tasks.tracking.config.pupper import motion_design as md
  from mjlab.tasks.tracking.config.pupper.tricks import sit_stay_shake, spider

  out = output.resolve() / "motions"
  out.mkdir(parents=True, exist_ok=True)
  paths = {}
  for name, module in (("sit_stay_shake", sit_stay_shake), ("spider_clips", spider)):
    motion = md.concat_clips(module.clips(), order=module.ORDER)
    path = out / ("spider_walk.npz" if name == "spider_clips" else f"{name}.npz")
    motion.save_npz(path)
    motion.save_robot_csv(path.with_name(path.stem + "_robot.csv"))
    paths[name] = path
    write_json(path.with_suffix(".json"), {
      "state": "reference_only_untrained",
      "source": source_manifest(),
      "motion_sha256": sha256(path),
      "training_order": module.ORDER,
      "clips": motion.clips,
      "geometry_report": motion.report,
    })
    print(f"{name}: {path}\n{json.dumps(motion.report, indent=2)}")
  return paths


def prepare_stress(profile: str, output: Path, idle_seconds: float = 15.0) -> Path:
  """One continuous held-out sequence; do not use --loops to simulate endurance,
  because the course MotionCommand teleports the robot at whole-file wraparound."""
  from mjlab.tasks.tracking.config.pupper import motion_design as md
  from mjlab.tasks.tracking.config.pupper.motion_ops import hold
  from mjlab.tasks.tracking.config.pupper.tricks import sit_stay_shake, spider

  if profile not in ("sit_stay_shake", "spider_clips"):
    raise ValueError("stress references are defined for sit_stay_shake and spider_clips")
  module = sit_stay_shake if profile == "sit_stay_shake" else spider
  clips = module.clips()
  for clip in clips:
    if clip.loop:
      clip.motion = hold(clip.motion, 0, idle_seconds)
  order = (["sit_down", "sit_idle", "shake_right", "sit_idle", "shake_left",
            "shake_right", "sit_idle", "shake_left", "sit_idle", "stand_up"]
           if profile == "sit_stay_shake" else ["spider_down", "spider_idle", "spider_idle", "spider_up"])
  library = md.concat_clips(clips, order=order)
  output.mkdir(parents=True, exist_ok=True)
  path = library.save_npz(output / f"{profile}_stress.npz")
  write_json(path.with_suffix(".json"), {
    "purpose": "held-out continuous long-idle/repeated-action reference; never use as the training file",
    "order": order, "idle_seconds": idle_seconds, "geometry_report": library.report,
    "motion_sha256": sha256(path), "source": source_manifest(),
  })
  return path


def check_generated_motion(path: Path) -> None:
  """Recheck the file itself instead of trusting a mutable sidecar's pass flag."""
  import numpy as np
  from mjlab.tasks.tracking.config.pupper import motion_design as md
  from mjlab.tasks.tracking.config.pupper.unimate.repair import geometry_issues

  with np.load(path, allow_pickle=False) as data:
    names = list(data["body_names"])
    base = names.index("base_link")
    motion = md.Motion(
      fps=float(data["fps"][0]), root_pos=data["body_pos_w"][:, base].copy(),
      root_quat=data["body_quat_w"][:, base].copy(), joint_pos=data["joint_pos"].copy(),
    )
    for key in ("joint_vel", "body_lin_vel_w", "body_ang_vel_w"):
      if not np.isfinite(data[key]).all():
        raise ValueError(f"non-finite {key} in generated motion")
  issues = geometry_issues(motion)
  if issues:
    raise ValueError("Generated motion failed geometry validation: " + "; ".join(issues))


def set_linear_tracking_std(cfg, value: float | None) -> dict:
  """Explicit experiment override; preserve task defaults and reward weights.

  This reward combines XY tracking error and body vertical velocity, so narrowing
  its standard deviation affects both. Refuse a changed reward implementation.
  """
  if value is None:
    return {}
  if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
    raise ValueError("--linear-tracking-std must be positive and finite")
  from mjlab.tasks.velocity.mdp.rewards import track_linear_velocity
  term = cfg.rewards.get("track_linear_velocity")
  if term is None or term.func is not track_linear_velocity or term.weight <= 0:
    raise ValueError("Linear std override requires the active standard linear-velocity reward")
  previous = term.params.get("std")
  if not isinstance(previous, (float, int)) or not math.isfinite(previous) or previous <= 0:
    raise ValueError("Existing linear-velocity std must be positive and finite")
  term.params["std"] = float(value)
  return {"track_linear_velocity.std": {
    "previous": float(previous), "value": float(value), "weight_unchanged": float(term.weight),
    "units": "m/s", "error": "squared XY command error plus squared BODY vertical velocity",
    "formula": "exp(-(xy_error_squared + body_vertical_velocity_squared) / std**2)",
  }}


def set_tracking_com_x_half_range(cfg, value: float | None) -> dict:
  """Separate tracking experiment: widen only torso body-frame X COM sampling.

  The course samples each axis in +/-1 cm. This bounded override supports an
  explicit X range up to +/-2 cm while retaining the original Y/Z distribution,
  event/RNG order, reference, rewards and mechanical/command limits. Require COM
  composition so a later density event cannot silently discard this experiment.
  """
  if value is None:
    return {}
  if isinstance(value, bool) or not math.isfinite(value) or not 0 < value <= 0.02:
    raise ValueError("--tracking-com-x-half-range must be finite and in (0, 0.02] meters")
  from mjlab.envs.mdp import dr
  from mjlab.tasks.pupper.deployment_contract import density_randomization_preserving_com

  term = cfg.events.get("base_com")
  inertia = cfg.events.get("inertial")
  if ("motion" not in cfg.commands or term is None or term.func is not dr.body_com_offset
      or term.mode != "startup" or term.params.get("operation", "add") != "add"
      or term.params.get("distribution", "uniform") != "uniform"):
    raise ValueError("COM X override requires the standard startup uniform additive tracking COM event")
  asset = term.params.get("asset_cfg")
  if asset is None or asset.name != "robot" or tuple(asset.body_names) != ("base_link",):
    raise ValueError("COM X override requires only robot/base_link")
  if inertia is None or inertia.func is not density_randomization_preserving_com:
    raise ValueError("COM X override requires --compose-com-randomization")
  previous = term.params.get("ranges")
  expected = {0: (-0.01, 0.01), 1: (-0.01, 0.01), 2: (-0.01, 0.01)}
  if previous != expected:
    raise ValueError("COM X override requires the unchanged course +/-1 cm XYZ ranges")
  updated = {**previous, 0: (-float(value), float(value))}
  term.params["ranges"] = updated
  return {"base_com.ranges": {
    "previous": {str(k): list(v) for k, v in previous.items()},
    "value": {str(k): list(v) for k, v in updated.items()},
    "units": "m", "frame": "base_link body frame", "distribution": "uniform",
    "effective_at": "startup; sampled once per world, retained across resets",
    "changed_axes": ["x"], "rng_draw_order_preserved": True,
    "reason": "Separate robustness experiment; the +2 cm course test is outside the original +/-1 cm training distribution",
  }}


def train(args: argparse.Namespace) -> Path:
  # run_train uses CUDA_VISIBLE_DEVICES to choose the device. An explicitly
  # disabled GPU must fail, not silently start 4096 CPU environments.
  os.environ.setdefault("CUDA_VISIBLE_DEVICES", "0")
  import torch

  if not os.environ["CUDA_VISIBLE_DEVICES"] or not torch.cuda.is_available():
    raise RuntimeError("Foundation RL requires a visible NVIDIA GPU; no training was started")
  if len(os.environ["CUDA_VISIBLE_DEVICES"].split(",")) != 1:
    raise ValueError("This runner uses one GPU; set CUDA_VISIBLE_DEVICES to a single device")
  from mjlab.scripts.train import TrainConfig, run_train
  from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

  profile = load_profile(args.profile)
  motion_path = args.motion.resolve() if args.motion else None
  if profile.get("motion_builder") and motion_path is None:
    motion_path = prepare(args.output)[profile["motion_builder"]]
  if profile.get("requires_motion") and motion_path is None:
    raise ValueError("Provide --motion with the inspected, cleaned generated-motion NPZ")
  if motion_path is not None and not motion_path.is_file():
    raise FileNotFoundError(motion_path)
  if profile.get("requires_motion"):
    check_generated_motion(motion_path)
  cfg = load_env_cfg(profile["task"])
  agent = load_rl_cfg(profile["task"])
  encoder_context = None
  encoder_mode = getattr(args, "encoder_observation_mode", None)
  encoder_change = getattr(args, "allow_encoder_observation_mode_change", False)
  if args.profile == "spider_walk":
    if getattr(args, "resume_run", None) and encoder_mode is None:
      raise ValueError("Remote --resume-run requires explicit --encoder-observation-mode; only local checkpoints can auto-inherit before environment construction")
    if getattr(args, "resume_run", None) and encoder_change:
      raise ValueError("Mode-changing continuation requires an exact local --initial-checkpoint, not --resume-run")
    from mjlab.tasks.pupper.encoder_observation import configure as configure_encoder
    encoder_context = configure_encoder(cfg, checkpoint=getattr(args,"initial_checkpoint",None),
        requested_mode=encoder_mode, allow_change=encoder_change, purpose="training")
  elif encoder_mode is not None or encoder_change:
    raise ValueError("Foundation encoder mode experiment supports spider_walk; tracking already observes measured joints")
  deploy_clamp = getattr(args, "deployment_target_clamp", False)
  contract = {"contract": "legacy_unclipped_target"}
  if deploy_clamp:
    from mjlab.tasks.pupper.deployment_contract import apply_deployment_target_clamp
    contract = apply_deployment_target_clamp(cfg)
  if getattr(args, "driver_limits", None):
    if not deploy_clamp:
      raise ValueError("--driver-limits requires --deployment-target-clamp")
    from mjlab.tasks.pupper.deployment_contract import apply_driver_target_clamp
    contract = apply_driver_target_clamp(cfg, args.driver_limits)
  slew_contract = None
  slew_rate = getattr(args, 'target_slew_rad_s', None)
  if slew_rate is not None:
    if args.profile != 'spider_walk':
      raise ValueError('Target-slew continuation currently supports spider_walk only; tracking export needs a separate contract')
    from mjlab.tasks.pupper.target_slew import apply_position_target_slew
    slew_contract = apply_position_target_slew(cfg, slew_rate, contract)
  compose_com = getattr(args, "compose_com_randomization", False)
  com_contract = {"contract": "legacy_event_order"}
  if compose_com:
    from mjlab.tasks.pupper.deployment_contract import compose_pupper_com_randomization
    com_contract = compose_pupper_com_randomization(cfg)
  com_x_half_range = getattr(args, "tracking_com_x_half_range", None)
  randomization_overrides = set_tracking_com_x_half_range(cfg, com_x_half_range)
  ground_hulls = getattr(args, "tracking_ground_hulls", False)
  torso_contact = getattr(args, "locomotion_torso_contact", False)
  if ground_hulls and torso_contact:
    raise ValueError("Choose tracking ground hulls OR locomotion torso contact, not both")
  collision_contract = {"contract": "legacy_inherited_collision_config"}
  if ground_hulls:
    from mjlab.tasks.tracking.config.pupper.contact_contract import enable_tracking_ground_hulls
    collision_contract = enable_tracking_ground_hulls(cfg)
  if torso_contact:
    from mjlab.tasks.pupper.locomotion_contact_contract import enable_locomotion_torso_contact
    collision_contract = enable_locomotion_torso_contact(cfg)
  cfg.scene.num_envs = args.num_envs or profile["num_envs"]
  agent.max_iterations = args.iterations or profile["iterations"]
  agent.seed = args.seed
  cfg.seed = args.seed
  agent.logger = args.logger
  agent.wandb_project = args.project
  agent.save_interval = min(agent.save_interval, agent.max_iterations)
  agent.wandb_tags = ("pupper-foundation", args.profile, "unvalidated")
  if profile.get("rewards") is not None:
    # Prevent defaults changing underneath an experiment after an upstream update.
    unknown = set(profile["rewards"]) - set(cfg.rewards)
    if unknown:
      raise ValueError(f"Unknown reward terms: {sorted(unknown)}")
    for name, reward in cfg.rewards.items():
      reward.weight = profile["rewards"].get(name, 0.0)
  linear_std = getattr(args, "linear_tracking_std", None)
  reward_parameters = set_linear_tracking_std(cfg, linear_std)
  torso_support_contract = None
  torso_support_weight = getattr(args, "torso_support_weight", None)
  if torso_support_weight is not None:
    if args.profile != "spider_walk":
      raise ValueError("Torso support experiment currently supports spider_walk only")
    from mjlab.tasks.pupper.torso_support import apply_torso_support_penalty
    torso_support_contract = apply_torso_support_penalty(cfg, torso_support_weight)
  if motion_path is not None:
    cfg.commands["motion"].motion_file = str(motion_path)
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  contract_suffix = "_deployclamp" if deploy_clamp else "_legacytargets"
  if getattr(args, "driver_limits", None):
    contract_suffix += "_driverclamp"
  if slew_contract is not None:
    contract_suffix += '_slew' + format(slew_rate, '.12g').replace('.', 'p')
  contract_suffix += "_composedcom" if compose_com else "_legacycom"
  contract_suffix += "_groundhulls" if ground_hulls else "_torsocontact" if torso_contact else "_legacyhulls"
  if linear_std is not None:
    contract_suffix += "_linstd" + format(linear_std, ".12g").replace(".", "p")
  if com_x_half_range is not None:
    contract_suffix += "_comx" + format(com_x_half_range, ".12g").replace(".", "p")
  if torso_support_weight is not None:
    contract_suffix += "_torsoload" + format(abs(torso_support_weight), ".12g").replace(".", "p")
  if encoder_context is not None and (encoder_mode is not None or encoder_context["active_observation_mode"] != "legacy_true"):
    contract_suffix += "_enc" + encoder_context["active_observation_mode"]
  agent.run_name = f"foundation_{args.profile}{contract_suffix}_s{args.seed}_{stamp}_{uuid.uuid4().hex[:8]}"
  run_dir = args.output.resolve() / "runs" / agent.run_name
  run_dir.mkdir(parents=True, exist_ok=False)
  if motion_path is not None:
    # The training run owns an immutable copy, independent of future prepare runs.
    frozen_motion = run_dir / motion_path.name
    frozen_motion.write_bytes(motion_path.read_bytes())
    cfg.commands["motion"].motion_file = str(frozen_motion)
    motion_path = frozen_motion
  initial_checkpoint = getattr(args, "initial_checkpoint", None)
  if initial_checkpoint is not None:
    initial_checkpoint = initial_checkpoint.resolve()
    if not initial_checkpoint.is_file():
      raise FileNotFoundError(initial_checkpoint)
    # Freeze the exact warm-start bytes inside this distinct run. run_train's
    # local resolver selects this directory/file only, never another latest run.
    (run_dir / "initial_checkpoint.pt").write_bytes(initial_checkpoint.read_bytes())
    agent.load_run = re.escape(run_dir.name) + "$"
    agent.load_checkpoint = "initial_checkpoint\\.pt$"
  agent.resume = args.resume_run is not None or initial_checkpoint is not None
  train_cfg = TrainConfig(
    env=cfg, agent=agent, enable_nan_guard=True,
    wandb_run_path=args.resume_run, wandb_checkpoint_name=args.resume_checkpoint,
  )
  manifest = {
    "state": "running", "created_utc": stamp, "profile": args.profile,
    "profile_config": profile, "seed": args.seed,
    "actuation_contract": contract, "com_randomization_contract": com_contract,
    "position_target_slew_contract": slew_contract,
    "encoder_observation_context": encoder_context,
    "collision_contract": collision_contract,
    "reward_parameter_overrides": reward_parameters,
    "domain_randomization_overrides": randomization_overrides,
    "num_envs": cfg.scene.num_envs, "iterations": agent.max_iterations,
    "resume_run": args.resume_run, "resume_checkpoint": args.resume_checkpoint,
    "torso_support_contract": torso_support_contract,
    "initial_checkpoint": str(initial_checkpoint) if initial_checkpoint else None,
    "initial_checkpoint_sha256": sha256(initial_checkpoint) if initial_checkpoint else None,
    "source": source_manifest(), "gpu": torch.cuda.get_device_name(0),
    "motion_file": str(motion_path) if motion_path else None,
    "motion_sha256": sha256(motion_path) if motion_path else None,
    "validated_in_simulation": False, "approved_for_hardware": False,
  }
  write_json(run_dir / "foundation-run.json", manifest)
  print(f"Persistent run directory: {run_dir}", flush=True)
  try:
    run_train(profile["task"], train_cfg, run_dir)
    checkpoints = sorted(run_dir.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[-1]))
    if not checkpoints or not (run_dir / "policy.json").exists():
      raise RuntimeError("Training returned without both a checkpoint and exported policy.json")
    manifest.update(
      state="trained_unvalidated", checkpoint=checkpoints[-1].name,
      checkpoint_sha256=sha256(checkpoints[-1]), policy_sha256=sha256(run_dir / "policy.json"),
    )
  except BaseException as exc:
    manifest.update(state="interrupted" if isinstance(exc, KeyboardInterrupt) else "failed", error=str(exc))
    raise
  finally:
    write_json(run_dir / "foundation-run.json", manifest)
    # Preserve provenance next to the automatically uploaded policy/checkpoints.
    if args.logger == "wandb":
      import wandb
      if wandb.run is not None:
        manifest["wandb_run"] = f"{wandb.run.entity}/{wandb.run.project}/{wandb.run.id}"
        write_json(run_dir / "foundation-run.json", manifest)
        try:
          wandb.save(str(run_dir / "foundation-run.json"), base_path=str(run_dir), policy="now")
          wandb.finish(exit_code=0 if manifest["state"] == "trained_unvalidated" else 1)
        except Exception as upload_error:
          # Keep the original training failure (if any) and the local artifact.
          manifest["provenance_upload_error"] = str(upload_error)
          write_json(run_dir / "foundation-run.json", manifest)
          print(f"[WARN] Manifest kept locally; W&B finalization failed: {upload_error}")
  return run_dir


def score(args: argparse.Namespace) -> None:
  """Nominal and held-out dynamics scores, with exact checkpoint/motion provenance."""
  from mjlab.tasks.tracking.config.pupper.play_motion import main as replay

  evaluation_source = source_manifest()
  args.output.mkdir(parents=True, exist_ok=True)
  variants = [("nominal", 1.0, 0.0, 0.0, False)]
  if args.suite:
    variants += [("heavy", 1.3, 0.0, 0.0, False), ("com_left", 1.0, 0.0, 0.02, False),
                 ("com_front", 1.0, 0.02, 0.0, False), ("randomized", 1.0, 0.0, 0.0, True)]
  reports = []
  for label, mass, x, y, randomized in variants:
    for seed in (args.seeds if randomized else [args.seeds[0]]):
      report_file = args.output / f"{label}-s{seed}.json"
      replay(args.motion.resolve(), checkpoint=args.checkpoint.resolve(), loops=args.loops,
             mass_scale=mass, com_x=x, com_y=y, seed=seed, randomize=randomized,
             report_file=report_file, deployment_target_clamp=getattr(args, "deployment_target_clamp", False),
             tracking_ground_hulls=getattr(args, "tracking_ground_hulls", False),
             driver_limits=getattr(args, "driver_limits", None))
      reports.append(json.loads(report_file.read_text()))
  if args.stress_profile:
    stress = prepare_stress(args.stress_profile, args.output)
    stress_report = args.output / "continuous-stress.json"
    replay(stress, checkpoint=args.checkpoint.resolve(), loops=1,
           seed=args.seeds[0], report_file=stress_report,
           deployment_target_clamp=getattr(args, "deployment_target_clamp", False),
             tracking_ground_hulls=getattr(args, "tracking_ground_hulls", False),
             driver_limits=getattr(args, "driver_limits", None))
    result = json.loads(stress_report.read_text())
    result["purpose"] = "continuous held-out long-idle/repeated-action sequence"
    reports.append(result)
  final_source = source_manifest()
  write_json(args.output / "evaluation.json", {
    "state": "simulation_evaluated", "approved_for_hardware": False,
    "checkpoint_sha256": sha256(args.checkpoint), "motion_sha256": sha256(args.motion),
    "deployment_target_clamp": getattr(args, "deployment_target_clamp", False),
    "driver_limits_sha256": sha256(args.driver_limits) if getattr(args, "driver_limits", None) else None,
    "tracking_ground_hulls": getattr(args, "tracking_ground_hulls", False),
    "source": evaluation_source, "source_after": final_source,
    "source_changed_during_evaluation": evaluation_source["source_sha256"] != final_source["source_sha256"],
    "reports": reports,
    "all_hold_up": all(r["verdict"] == "HOLDS UP" for r in reports),
    "scope": "Reference tracking; does not test thermal limits or real robot policy hand-offs",
  })


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = parser.add_subparsers(dest="command", required=True)
  prep = sub.add_parser("prepare")
  prep.add_argument("--output", type=Path, required=True)
  launch = sub.add_parser("train")
  launch.add_argument("profile", choices=list(json.loads(PROFILE_FILE.read_text())))
  launch.add_argument("--output", type=Path, required=True)
  launch.add_argument("--motion", type=Path)
  launch.add_argument("--num-envs", type=int)
  launch.add_argument("--iterations", type=int)
  launch.add_argument("--seed", type=int, default=42)
  launch.add_argument("--logger", choices=("wandb", "tensorboard"), default="wandb")
  launch.add_argument("--project", default="mjlab")
  launch.add_argument("--resume-run", help="Exact entity/project/run-id to resume")
  launch.add_argument("--resume-checkpoint", help="Exact model_N.pt within --resume-run")
  launch.add_argument("--initial-checkpoint", type=Path, help="Exact local checkpoint for a separate finetune run; the original is retained")
  launch.add_argument("--deployment-target-clamp", action="store_true", help="New explicit actuation contract matching on-robot joint target clipping")
  launch.add_argument("--driver-limits", type=Path, help="Named driver joint limits JSON; requires --deployment-target-clamp")
  launch.add_argument('--target-slew-rad-s', type=float, help='SpiderWalk-only continuous command slew, at most15rad/s; requires controller/driver clamps and matched native support')
  launch.add_argument("--linear-tracking-std", type=float, help="Explicit reward-width experiment in m/s; changes XY tracking AND body vertical-velocity tolerance, preserving reward weights")
  launch.add_argument("--encoder-observation-mode", choices=("legacy_true","deployment_measured"), help="SpiderWalk: inherit tagged local checkpoint mode or legacy_true; remote resume requires explicit mode; measured actor reads q+b")
  launch.add_argument("--allow-encoder-observation-mode-change", action="store_true", help="Explicit separate experiment changing a local initial checkpoint observation mode; exact bytes are recorded")
  launch.add_argument("--compose-com-randomization", action="store_true", help="Separate experiment preserving COM offsets after inertial randomization; legacy runs stay unchanged")
  launch.add_argument("--tracking-com-x-half-range", type=float, help="Separate tracking experiment: torso X COM uniform +/-range in meters, at most 0.02; requires --compose-com-randomization and retains Y/Z +/-0.01")
  launch.add_argument("--tracking-ground-hulls", action="store_true", help="Separate corrected body-contact contract for Pupper tracking; legacy course configuration disables these hulls")
  launch.add_argument("--locomotion-torso-contact", action="store_true", help="Separate SpiderWalk physics contract adding only the torso collider; preserve knee/foot spheres")
  launch.add_argument("--torso-support-weight", type=float, help="Explicit flat-torso normal-load penalty weight [-5,0];0 is the sensor-only matched control, requires --locomotion-torso-contact")
  evaluate = sub.add_parser("score")
  evaluate.add_argument("--motion", type=Path, required=True)
  evaluate.add_argument("--checkpoint", type=Path, required=True)
  evaluate.add_argument("--output", type=Path, required=True)
  evaluate.add_argument("--loops", type=int, default=1)
  evaluate.add_argument("--suite", action="store_true")
  evaluate.add_argument("--deployment-target-clamp", action="store_true")
  evaluate.add_argument("--driver-limits", type=Path)
  evaluate.add_argument("--tracking-ground-hulls", action="store_true")
  evaluate.add_argument("--stress-profile", choices=("sit_stay_shake", "spider_clips"))
  evaluate.add_argument("--seeds", nargs="+", type=int, default=[101, 202, 303])
  args = parser.parse_args()
  for name in ("num_envs", "iterations", "loops"):
    if getattr(args, name, None) is not None and getattr(args, name) <= 0:
      parser.error(f"--{name.replace('_', '-')} must be positive")
  if getattr(args, "resume_checkpoint", None) and not args.resume_run:
    parser.error("--resume-checkpoint requires --resume-run")
  if getattr(args, "initial_checkpoint", None) and getattr(args, "resume_run", None):
    parser.error("choose --initial-checkpoint or --resume-run")
  if args.command == "prepare":
    prepare(args.output)
  elif args.command == "train":
    print(train(args))
  else:
    score(args)


if __name__ == "__main__":
  main()
