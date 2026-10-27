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
import os
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
  if profile.get("motion_builder"):
    motion_path = prepare(args.output)[profile["motion_builder"]]
  if profile.get("requires_motion") and motion_path is None:
    raise ValueError("Provide --motion with the inspected, cleaned generated-motion NPZ")
  if motion_path is not None and not motion_path.is_file():
    raise FileNotFoundError(motion_path)
  cfg = load_env_cfg(profile["task"])
  agent = load_rl_cfg(profile["task"])
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
  if motion_path is not None:
    cfg.commands["motion"].motion_file = str(motion_path)
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  agent.run_name = f"foundation_{args.profile}_s{args.seed}_{stamp}_{uuid.uuid4().hex[:8]}"
  run_dir = args.output.resolve() / "runs" / agent.run_name
  run_dir.mkdir(parents=True, exist_ok=False)
  if motion_path is not None:
    # The training run owns an immutable copy, independent of future prepare runs.
    frozen_motion = run_dir / motion_path.name
    frozen_motion.write_bytes(motion_path.read_bytes())
    cfg.commands["motion"].motion_file = str(frozen_motion)
    motion_path = frozen_motion
  agent.resume = args.resume_run is not None
  train_cfg = TrainConfig(
    env=cfg, agent=agent, enable_nan_guard=True,
    wandb_run_path=args.resume_run, wandb_checkpoint_name=args.resume_checkpoint,
  )
  manifest = {
    "state": "running", "created_utc": stamp, "profile": args.profile,
    "profile_config": profile, "seed": args.seed,
    "num_envs": cfg.scene.num_envs, "iterations": agent.max_iterations,
    "resume_run": args.resume_run, "resume_checkpoint": args.resume_checkpoint,
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
        wandb.save(str(run_dir / "foundation-run.json"), base_path=str(run_dir), policy="now")
        wandb.finish(exit_code=0 if manifest["state"] == "trained_unvalidated" else 1)
  return run_dir


def score(args: argparse.Namespace) -> None:
  """Nominal and held-out dynamics scores, with exact checkpoint/motion provenance."""
  from mjlab.tasks.tracking.config.pupper.play_motion import main as replay

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
             report_file=report_file)
      reports.append(json.loads(report_file.read_text()))
  write_json(args.output / "evaluation.json", {
    "state": "simulation_evaluated", "approved_for_hardware": False,
    "checkpoint_sha256": sha256(args.checkpoint), "motion_sha256": sha256(args.motion),
    "source": source_manifest(), "reports": reports,
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
  evaluate = sub.add_parser("score")
  evaluate.add_argument("--motion", type=Path, required=True)
  evaluate.add_argument("--checkpoint", type=Path, required=True)
  evaluate.add_argument("--output", type=Path, required=True)
  evaluate.add_argument("--loops", type=int, default=1)
  evaluate.add_argument("--suite", action="store_true")
  evaluate.add_argument("--seeds", nargs="+", type=int, default=[101, 202, 303])
  args = parser.parse_args()
  for name in ("num_envs", "iterations", "loops"):
    if getattr(args, name, None) is not None and getattr(args, name) <= 0:
      parser.error(f"--{name.replace('_', '-')} must be positive")
  if getattr(args, "resume_checkpoint", None) and not args.resume_run:
    parser.error("--resume-checkpoint requires --resume-run")
  if args.command == "prepare":
    prepare(args.output)
  elif args.command == "train":
    print(train(args))
  else:
    score(args)


if __name__ == "__main__":
  main()
