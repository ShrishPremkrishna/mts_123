#!/usr/bin/env python3
"""Opt Lab 2 Part 3: Bumpy + Lab 4 DR robustness fine-tune (launch recipe).

Spec: "Run the Bumpy variant with your lab 4 DR ranges as the robustness pass."
This warm-starts a MixedGaits checkpoint on ``Mjlab-MixedGaits-Bumpy-Pupper-v3``
(the repo's Bumpy task: Perlin bumps up to 5 cm, same actor/critic observation as
flat, so flat checkpoints load) with:

* the **same reference dir** the checkpoint trained on (the policy observes it);
* the Part 2 reward weights ``configs/opt-lab-2/weights/splay_stand_still.json``;
* ``--num-envs 4096`` (decision ``2026-11-18-infra-num-envs``);
* Lab 4 DR = the task defaults (kp x[0.6, 1.1], kd x[0.8, 1.5], foot friction
  [0.6, 1.4]); checked here against ``lab4_runs/shrish_stablegait/training-settings.json``
  before anything launches;
* the reviewed **checked Bumpy** contract (``launch.py --checked-bumpy`` ->
  ``foundation/scripts/bumpy_checked_cli.py``: 0.10 m grid + physics-overflow audit);
* the standard physics flags and pushed-input checks of ``launch.py``.

The iteration budget is decision ``report/decisions/2026-11-18-w4-finetune-budget.md``
(1500). ``--flat-control`` launches the matched control arm the decision asks for: the
same warm start, weights, seed and budget on ``Mjlab-MixedGaits-Flat-Pupper-v3``
(REPRO_PLAN section 3, "always run a control arm").

  uv run --no-sync python scripts/opt_lab_2/part3_finetune.py --checkpoint CKPT \
      --reference-dir REF --name p3-bumpy-<parent> [--iterations N] [--smoke] [--dry-run]
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

MJLAB_ROOT = Path(__file__).resolve().parents[2]
LAUNCH = MJLAB_ROOT / "scripts/opt_lab_2/launch.py"
BUMPY_TASK = "Mjlab-MixedGaits-Bumpy-Pupper-v3"
FLAT_TASK = "Mjlab-MixedGaits-Flat-Pupper-v3"
WEIGHTS = "configs/opt-lab-2/weights/splay_stand_still.json"
FULL_ITERATIONS = 1500  # decision report/decisions/2026-11-18-w4-finetune-budget.md (2/2 graders)
SMOKE_MAX = 50
# Decision report/decisions/2026-11-18-w4-kp-coverage.md (arbitrated): the WIDEN branch trains both
# arms with these PD-gain DR ranges (a superset of Lab 4); evaluation stays on the Lab 4 ranges.
PD_RANGES = {"lab4": None, "wide": dict(kp_range=(0.6, 1.3), kd_range=(0.7, 1.5))}


def check_dr(reference_dir: Path | None, task: str = BUMPY_TASK) -> dict:
  """Load the Bumpy env config in a fresh process (the reference dir is read at import)."""
  env = dict(os.environ)
  if reference_dir:
    env["PUPPER_REFERENCE_DIR"] = str(reference_dir.resolve())
  code = (
    "import json, mjlab.tasks; from mjlab.tasks.registry import load_env_cfg;"
    "from mjlab.scripts.part3_common import check_lab4_dr;"
    f"c = load_env_cfg('{task}');"
    "t = c.scene.terrain; g = t.terrain_generator; s = g.sub_terrains if g else {};"
    "print(json.dumps(dict(dr=check_lab4_dr(c), terrain=t.terrain_type,"
    " perlin={k: dict(height_range=list(v.height_range), resolution=v.resolution) for k, v in s.items()},"
    " fell_over_deg=round(c.terminations['fell_over'].params['limit_angle'] * 57.29578, 1),"
    " vx_range=list(c.commands['twist'].ranges.lin_vel_x))))"
  )
  r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env, cwd=MJLAB_ROOT)
  if r.returncode:
    raise SystemExit(f"[part3_finetune] Lab 4 DR / Bumpy config check FAILED:\n{r.stderr[-3000:]}")
  return json.loads(r.stdout.strip().splitlines()[-1])


def main() -> int:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--checkpoint", type=Path, required=True)
  p.add_argument("--reference-dir", type=Path, required=True, help="The reference dir the checkpoint trained on")
  p.add_argument("--name", required=True, help="Run name (launch.py prefixes rp-)")
  p.add_argument("--iterations", type=int, default=None)
  p.add_argument("--smoke", action="store_true", help=f"Smoke run: at most {SMOKE_MAX} iterations, tagged smoke")
  p.add_argument("--seed", type=int, default=42)
  p.add_argument("--num-envs", type=int, default=4096)
  p.add_argument("--weights", default=WEIGHTS)
  p.add_argument("--flat-control", action="store_true",
                 help="Matched control arm: same warm start/weights/seed/budget on the Flat task")
  p.add_argument("--pd-ranges", choices=sorted(PD_RANGES), default="lab4",
                 help="Training PD-gain DR: lab4 (task default) or wide (kp x[0.6,1.3], kd x[0.7,1.5])")
  p.add_argument("--dry-run", action="store_true")
  p.add_argument("--quiet", action="store_true")
  args, extra = p.parse_known_args()
  iters = args.iterations or (SMOKE_MAX if args.smoke else FULL_ITERATIONS)
  if args.smoke and iters > SMOKE_MAX:
    p.error(f"--smoke allows at most {SMOKE_MAX} iterations")
  if args.num_envs != 4096 and not args.smoke:
    p.error("full runs use --num-envs 4096 (decision 2026-11-18-infra-num-envs)")
  if not args.checkpoint.is_file():
    p.error(f"checkpoint not found: {args.checkpoint}")
  task = FLAT_TASK if args.flat_control else BUMPY_TASK
  cfg = check_dr(args.reference_dir, task)
  print(f"[part3_finetune] config check: {json.dumps(cfg)}", flush=True)
  dr_tag = "lab4dr" if args.pd_ranges == "lab4" else "p3dr-widekp"
  tags = (f"p3,flat-control,{dr_tag}" if args.flat_control else f"p3,bumpy,{dr_tag}") + (",smoke" if args.smoke else "")
  pd = PD_RANGES[args.pd_ranges]
  pd_flag = ["--pd-gain-override", ",".join(str(x) for x in (*pd["kp_range"], *pd["kd_range"]))] if pd else []
  kind = "flat-continuation CONTROL" if args.flat_control else "Bumpy + Lab 4 DR fine-tune"
  cmd = [
    sys.executable, str(LAUNCH), "--name", args.name, *([] if args.flat_control else ["--checked-bumpy"]),
    "--reference-dir", str(args.reference_dir), "--weights", args.weights, "--tags", tags,
    "--notes", f"Part 3 {kind} ({'SMOKE' if args.smoke else 'full'}); "
               f"warm start {args.checkpoint}; DR {cfg['dr']['ranges']}"
               + (f"; TRAINING PD-gain override {pd}" if pd else ""),
    *pd_flag,
    *(["--dry-run"] if args.dry_run else []), *(["--quiet"] if args.quiet else []),
    "--task", task, "--checkpoint", str(args.checkpoint.resolve()), "--iterations", str(iters),
    "--num-envs", str(args.num_envs), "--seed", str(args.seed), *extra,
  ]
  print("[part3_finetune] " + " ".join(cmd), flush=True)
  return subprocess.call(cmd, cwd=MJLAB_ROOT)


if __name__ == "__main__":
  sys.exit(main())
