#!/usr/bin/env python3
"""Install an exported trick from local files or Weights & Biases into tricks/<name>/.

Pupper trick training (pupper-mjlab's Mjlab-Tracking-Flat-Pupper-v3) uploads the
trick policy to its W&B run as policy.json; the training log prints the exact
command to run here. The Pupper is already logged into W&B for policy deploys.

    python3 robot/get_trick.py <entity>/mjlab/<run-id> --name sit
    python3 robot/get_trick.py mjlab/<run-id> --name sit          # your default entity
    python3 robot/get_trick.py --policy-file artifacts/sit/policy.json --name sit_stay_shake

A trick that walks in its own stance (the spider) has two policies: its clips
(a trick run) and its walking policy (a Mjlab-SpiderWalk run). Fetch both at once:

    python3 robot/get_trick.py <clip-trick-run> --walk <spider-walk-run> --name spider_walk
    python3 robot/get_trick.py --policy-file clip_policy.json --walk-policy-file walk_policy.json --walk-limits evaluated_limits.json --name spider_walk

The trick lands as tricks/<name>/policy.json. Every trick on the robot is a
policy: check a design in simulation (pupper-mjlab's play_motion) before
training it, not by replaying it open loop on the robot.

If tricks/<name>/trick.yaml does not exist yet a template is written; edit its
description (it is what Gemini reads). Restart Gemini Pupper afterwards
(robot/stop.sh, then robot/start.sh): the launch file loads one controller per
policy trick at startup.
"""

from __future__ import annotations

import argparse
import json
import math
import hashlib
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from runtime_config import component_dir
from tricks import load_tricks

TRICK_DIR = Path(__file__).resolve().parent.parent / "tricks"
VALIDATOR = component_dir("pupper_gait_deploy", "PUPPER_GAIT_DEPLOY_DIR") / "download_policy.py"


def validate_artifact(path: Path) -> None:
  if not VALIDATOR.is_file():
    raise SystemExit("Policy validator missing; set PUPPER_GAIT_DEPLOY_DIR. Refusing to install unchecked policies.")
  check = subprocess.run([sys.executable, str(VALIDATOR), "--validate-only", str(path)])
  if check.returncode != 0:
    raise SystemExit("validation failed; not installing")


def resolve_run_path(run: str, project: str, entity: str | None) -> str:
  """Accept entity/project/run-id, project/run-id or a bare run id."""
  import wandb

  parts = run.split("/")
  if len(parts) == 3:
    return run
  if len(parts) == 2:
    project, run = parts
  entity = entity or wandb.Api().default_entity
  if entity is None:
    sys.exit("Could not determine the W&B entity; pass a full entity/project/run-id path.")
  return f"{entity}/{project}/{run}"


def fetch_policy(run_path: str, dest: Path) -> dict:
  import wandb

  run = wandb.Api().run(run_path)
  if "policy.json" not in [f.name for f in run.files()]:
    sys.exit(
      f"{run_path} has no policy.json yet. Trick runs upload it at every checkpoint "
      "save and on Ctrl+C; wait for the first checkpoint, or check the run id."
    )
  with tempfile.TemporaryDirectory() as tmp:
    path = Path(run.file("policy.json").download(root=tmp, replace=True).name)
    policy = json.loads(path.read_text())
    if "trick" not in policy or "motion" not in policy:
      sys.exit(f"{run_path}'s policy.json is not a trick policy (no trick/motion blocks).")
    validate_artifact(path)
    dest.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, dest / "policy.json")
  print(f"Run {run.name} ({run_path}) -> {dest / 'policy.json'}")
  return policy


def fetch_walk_policy(run_path: str, dest: Path) -> None:
  """A walking trick's walking policy (a velocity policy, not a trick) -> walk_policy.json."""
  import wandb

  run = wandb.Api().run(run_path)
  if "policy.json" not in [f.name for f in run.files()]:
    sys.exit(f"{run_path} has no policy.json yet; wait for the run to save a checkpoint.")
  with tempfile.TemporaryDirectory() as tmp:
    path = Path(run.file("policy.json").download(root=tmp, replace=True).name)
    if "trick" in json.loads(path.read_text()):
      sys.exit(f"{run_path} is a trick run; --walk takes the walking policy's run (Mjlab-SpiderWalk).")
    validate_artifact(path)
    dest.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, dest / "walk_policy.json")
  print(f"Run {run.name} ({run_path}) -> {dest / 'walk_policy.json'}")


def copy_local_policy(source: Path, dest: Path, *, walking: bool = False) -> dict:
  """Validate exactly the bytes staged for installation; no W&B dependency/auth."""
  if not source.is_file():
    raise ValueError(f"local policy file does not exist: {source}")
  dest.mkdir(parents=True, exist_ok=True)
  staged = dest / ("walk_policy.json" if walking else "policy.json")
  shutil.copy2(source, staged)
  policy = json.loads(staged.read_text())
  if not isinstance(policy, dict):
    raise ValueError("policy file must contain a JSON object")
  if walking and "trick" in policy:
    raise ValueError("--walk-policy-file requires an exported velocity policy, not a trick policy")
  if not walking and not ("trick" in policy and "motion" in policy):
    raise ValueError("--policy-file requires an exported trick policy with trick/motion blocks")
  validate_artifact(staged)
  print(f"Local policy {source.resolve()} -> {staged}")
  return policy


def validate_walk_evidence(dest: Path, limits: dict) -> None:
  """Bind reviewed operating limits to the exact staged walking export."""
  claimed = limits.get("walk_policy_sha256") if isinstance(limits, dict) else None
  if not isinstance(claimed, str) or not re.fullmatch(r"[0-9a-f]{64}", claimed):
    raise ValueError("--walk-limits requires walk_policy_sha256 for the evaluated walking export")
  actual = hashlib.sha256((dest / "walk_policy.json").read_bytes()).hexdigest()
  if claimed != actual:
    raise ValueError("--walk-limits walk_policy_sha256 does not match the staged walking policy; evaluate this export before selecting limits")


def write_template(dest: Path, name: str, policy: dict, walks: bool = False, walk_limits: dict | None = None) -> None:
  spec = dest / "trick.yaml"
  if walk_limits is not None:
    if not isinstance(walk_limits, dict) or not isinstance(walk_limits.get("evaluation_source"), str) or not walk_limits["evaluation_source"].strip():
      raise ValueError("--walk-limits must identify evaluation_source; training caps are not measured performance")
    for key in ("speed", "max_forward", "max_sideways", "turn"):
      value = walk_limits.get(key)
      if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"--walk-limits {key} must be a finite positive number")
  if spec.exists():
    if walks and walk_limits is not None:
      import yaml
      previous = yaml.safe_load(spec.read_text())
      previous["walk"] = {**previous.get("walk", {}), "policy": "walk_policy.json",
                          **{key: walk_limits[key] for key in ("speed", "max_forward", "max_sideways", "turn")}, "run": False}
      previous["evaluation_source"] = walk_limits["evaluation_source"]
      spec.write_text(yaml.safe_dump(previous, sort_keys=False))
      print(f"Updated evaluated walking limits in {spec}; kept its descriptions and actions")
      return
    print(f"Kept the existing {spec}")
    return
  if name in ("sit_stay_shake", "spider_walk"):
    import yaml

    common = {"type": "policy", "policy": "policy.json", "start_pose": "standing", "max_body_angle_deg": 60}
    if name == "sit_stay_shake":
      common.update(
        description="Sit and stay sitting until asked to stand. Use for a plain sit request; shake either paw while seated.",
        end_pose="sitting", exit_clip="stand_up",
        actions={"shake_left": "Shake the left paw, then keep sitting.",
                 "shake_right": "Shake the right paw, then keep sitting.",
                 "stand_up": "Stand up and return to normal locomotion."})
    else:
      if not walks or not walk_limits:
        raise ValueError("spider_walk needs both trained policies and --walk-limits from simulation evaluation")
      if not walk_limits.get("evaluation_source"):
        raise ValueError("--walk-limits must identify evaluation_source; training command caps are not measured performance")
      common.update(
        description="Enter a low spider stance and stay there. Move, turn and arc walk slowly in this stance; stand_up leaves it. Cannot run or follow people.",
        end_pose="spider", exit_clip="spider_up", actions={"spider_up": "Wiggle back up to standing and return to normal locomotion."},
        walk={"policy": "walk_policy.json", **{k: walk_limits[k] for k in ("speed", "max_forward", "max_sideways", "turn")},
              "run": False, "note": "Limits selected from simulation evaluation; hardware validation remains required."},
        evaluation_source=walk_limits["evaluation_source"])
    spec.write_text(yaml.safe_dump(common, sort_keys=False))
    print(f"Wrote {spec}; inspect it before supervised deployment.")
    return
  lines = [
    f"# {name}: fetched with get_trick.py. Edit the description -- it is what Gemini reads",
    "# when deciding whether to play this trick.",
    f"description: TODO describe what the robot does in {name}.",
  ]
  lines += ["type: policy", "policy: policy.json", "start_pose: standing", "end_pose: standing"]
  lines.append("max_body_angle_deg: 60   # the tilt e-stop while the trick plays")
  clips = policy.get("trick", {}).get("clips", [])
  if clips:
    lines += [
      "# This policy has clips. Name the ones Gemini may ask for while the trick runs,",
      "# and the clip that returns to standing (played before any other motion):",
      "# actions:",
    ]
    lines += [f"#   {c['name']}: TODO what this clip does" for c in clips]
    lines += ["# exit_clip: <clip>", "# end_pose: <the pose the looping clip holds>"]
  if walks:
    cmd = json.loads((dest / "walk_policy.json").read_text())["command_clip"]
    lines += [
      "walk:                      # it walks in its stance: move, turn and arc drive this policy",
      "  policy: walk_policy.json",
      "  speed: TODO              # default m/s for move and arc",
      f"  max_forward: TODO        # m/s; at most {max(map(abs, cmd['vx'])):g} (what it was trained for)",
      f"  max_sideways: TODO       # m/s; at most {max(map(abs, cmd['vy'])):g}",
      f"  turn: TODO               # deg/s; at most {math.degrees(max(map(abs, cmd['wz']))):.0f}",
      "  run: false",
      "  note: TODO anything else Gemini should know about walking like this",
    ]
  spec.write_text("\n".join(lines) + "\n")
  print(f"Wrote a template {spec}; edit its description.")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("run", nargs="?", help="W&B training run: entity/project/run-id, project/run-id or run-id")
  parser.add_argument("--policy-file", type=Path, help="local exported trick policy.json; requires no W&B account")
  parser.add_argument("--name", required=True, help="trick name: the folder under tricks/ and Gemini's id")
  parser.add_argument("--project", default="mjlab")
  parser.add_argument("--entity", default=None)
  parser.add_argument("--walk", default=None, help="a walking policy's run (Mjlab-SpiderWalk), for a trick that walks")
  parser.add_argument("--walk-policy-file", type=Path, help="local exported walking policy, paired with --policy-file")
  parser.add_argument("--walk-limits", type=Path, help="JSON with evaluation_source, walk_policy_sha256, and measured speed, max_forward, max_sideways (m/s), turn (deg/s)")
  parser.add_argument("--secret", action="store_true", help="hide the trick behind the local password gate")
  args = parser.parse_args()
  if bool(args.run) == bool(args.policy_file):
    parser.error("choose exactly one W&B run or --policy-file")
  if args.walk and args.walk_policy_file:
    parser.error("choose --walk or --walk-policy-file, not both")
  if (args.policy_file and args.walk) or (args.run and args.walk_policy_file):
    parser.error("do not mix W&B and local sources; use a run with --walk, or --policy-file with --walk-policy-file")
  if not re.fullmatch(r"[a-z][a-z0-9_]*", args.name):
    parser.error("--name must be lowercase letters, digits and underscores, starting with a letter")
  has_walk = bool(args.walk or args.walk_policy_file)
  if args.name == "spider_walk" and not has_walk:
    parser.error("spider_walk requires both trained policies and --walk-limits from simulation evaluation")
  if has_walk and not args.walk_limits:
    parser.error("every walking-policy installation requires --walk-limits bound to its evaluated export")
  if args.walk_limits and not has_walk:
    parser.error("--walk-limits requires a walking policy")
  limits_bytes = args.walk_limits.read_bytes() if args.walk_limits else None
  limits = json.loads(limits_bytes) if limits_bytes is not None else None
  dest = TRICK_DIR / args.name
  run_path = resolve_run_path(args.run, args.project, args.entity) if args.run else None
  walk_run = resolve_run_path(args.walk, args.project, args.entity) if args.walk else None
  # Stage both policies and their descriptor before replacing an existing trick.
  # A failed second download or contract check must leave the working copy intact.
  TRICK_DIR.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(prefix=".fetch-", dir=TRICK_DIR) as temp:
    staged = Path(temp) / args.name
    if dest.exists():
      shutil.copytree(dest, staged)
    policy = (copy_local_policy(args.policy_file, staged) if args.policy_file
              else fetch_policy(run_path, staged))
    if walk_run:
      fetch_walk_policy(walk_run, staged)
    elif args.walk_policy_file:
      copy_local_policy(args.walk_policy_file, staged, walking=True)
    if has_walk:
      validate_walk_evidence(staged, limits)
      # Keep the exact reviewed input with the installation; an external path may
      # later move or be overwritten. The manifest hashes these preserved bytes.
      (staged / "walk_limits.json").write_bytes(limits_bytes)
    write_template(staged, args.name, policy, walks=has_walk, walk_limits=limits)
    if args.secret:
      import yaml
      spec_path = staged / "trick.yaml"
      spec = yaml.safe_load(spec_path.read_text())
      spec["secret"] = True
      spec_path.write_text(yaml.safe_dump(spec, sort_keys=False))
    load_tricks(Path(temp))  # validates clips, transitions and walking limits together
    manifest = {"policy_run": run_path, "walk_run": walk_run,
                "policy_source": ({"type": "local", "path": str(args.policy_file.resolve())}
                                  if args.policy_file else {"type": "wandb", "run": run_path}),
                "walk_source": ({"type": "local", "path": str(args.walk_policy_file.resolve())}
                                if args.walk_policy_file else {"type": "wandb", "run": walk_run} if walk_run else None),
                "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in staged.iterdir() if p.suffix in (".json", ".yaml") and p.name != "artifact_manifest.json"},
                "hardware_validated": False, "ta_approval_recorded": False}
    if args.walk_limits:
      manifest["walk_limits_source"] = {"path": str(args.walk_limits.resolve()),
                                      "sha256": manifest["files"]["walk_limits.json"],
                                      "walk_policy_sha256": limits["walk_policy_sha256"]}
    manifest["policy_source"]["sha256"] = manifest["files"]["policy.json"]
    if manifest["walk_source"]:
      manifest["walk_source"]["sha256"] = manifest["files"]["walk_policy.json"]
    (staged / "artifact_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    backup = None
    if dest.exists():
      from datetime import datetime, timezone
      backup = TRICK_DIR / ".backups" / f"{args.name}-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}"
      backup.parent.mkdir(exist_ok=True)
      dest.rename(backup)
    try:
      staged.rename(dest)
    except BaseException:
      if backup is not None:
        backup.rename(dest)
      raise
  trick = policy["trick"]
  print(f"Trick '{trick['name']}': {trick['frames']} frames, {trick['duration_s']:.1f} s")
  for c in trick.get("clips", []):
    print(f"  clip {c['name']:14} {(c['end'] - c['start']) / trick['fps']:.1f} s"
          f"{' loops' if c.get('loop') else ''}{' -> ' + str(c['next']) if c.get('next') is not None else ''}")
  print("Artifacts installed; no controllers were activated. Obtain TA approval for new policies before supervised robot use.")


if __name__ == "__main__":
  main()
