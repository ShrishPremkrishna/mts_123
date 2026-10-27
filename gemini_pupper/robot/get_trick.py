#!/usr/bin/env python3
"""Fetch a trick from Weights & Biases into tricks/<name>/.

Pupper trick training (pupper-mjlab's Mjlab-Tracking-Flat-Pupper-v3) uploads the
trick policy to its W&B run as policy.json; the training log prints the exact
command to run here. The Pupper is already logged into W&B for policy deploys.

    python3 robot/get_trick.py <entity>/mjlab/<run-id> --name sit
    python3 robot/get_trick.py mjlab/<run-id> --name sit          # your default entity

A trick that walks in its own stance (the spider) has two policies: its clips
(a trick run) and its walking policy (a Mjlab-SpiderWalk run). Fetch both at once:

    python3 robot/get_trick.py <clip-trick-run> --walk <spider-walk-run> --name spider_walk

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


def write_template(dest: Path, name: str, policy: dict, walks: bool = False, walk_limits: dict | None = None) -> None:
  spec = dest / "trick.yaml"
  if spec.exists():
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
  parser.add_argument("run", help="W&B training run: entity/project/run-id, project/run-id or run-id")
  parser.add_argument("--name", required=True, help="trick name: the folder under tricks/ and Gemini's id")
  parser.add_argument("--project", default="mjlab")
  parser.add_argument("--entity", default=None)
  parser.add_argument("--walk", default=None, help="a walking policy's run (Mjlab-SpiderWalk), for a trick that walks")
  parser.add_argument("--walk-limits", type=Path, help="JSON with evaluation_source and measured speed, max_forward, max_sideways (m/s), turn (deg/s)")
  parser.add_argument("--secret", action="store_true", help="hide the trick behind the local password gate")
  args = parser.parse_args()
  if not re.fullmatch(r"[a-z][a-z0-9_]*", args.name):
    parser.error("--name must be lowercase letters, digits and underscores, starting with a letter")
  if args.name == "spider_walk" and (not args.walk or not args.walk_limits):
    parser.error("spider_walk requires --walk and --walk-limits from simulation evaluation")
  limits = json.loads(args.walk_limits.read_text()) if args.walk_limits else None
  dest = TRICK_DIR / args.name
  run_path = resolve_run_path(args.run, args.project, args.entity)
  walk_run = resolve_run_path(args.walk, args.project, args.entity) if args.walk else None
  # Stage both policies and their descriptor before replacing an existing trick.
  # A failed second download or contract check must leave the working copy intact.
  TRICK_DIR.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(prefix=".fetch-", dir=TRICK_DIR) as temp:
    staged = Path(temp) / args.name
    if dest.exists():
      shutil.copytree(dest, staged)
    policy = fetch_policy(run_path, staged)
    if walk_run:
      fetch_walk_policy(walk_run, staged)
    write_template(staged, args.name, policy, walks=bool(walk_run), walk_limits=limits)
    if args.secret:
      import yaml
      spec_path = staged / "trick.yaml"
      spec = yaml.safe_load(spec_path.read_text())
      spec["secret"] = True
      spec_path.write_text(yaml.safe_dump(spec, sort_keys=False))
    load_tricks(Path(temp))  # validates clips, transitions and walking limits together
    manifest = {"policy_run": run_path, "walk_run": walk_run,
                "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in staged.glob("*.json") if p.name != "artifact_manifest.json"},
                "hardware_validated": False, "ta_approval_recorded": False}
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
