#!/usr/bin/env python3
"""Opt Lab 2 Part 3 runbook: one command from a final MixedGaits checkpoint to Part 3 sim evidence.

Stages (run in order, each skipped when its output already exists; simulation only):

  export    part3_export on the checkpoint: policy.json + golden fixture, round trip
            (torch / JSON / run auto-export / ONNX / W&B copy), limit audit, deploy validator,
            host C++ gait-reference test           -> report/part3/export/<name>.{json,md}
  eval      part3_eval: suite + seam on flat/Bumpy x Lab 4 DR on/off, push on DR-on
                                                    -> report/part3/eval/<name>/
  video     seam-sweep transition videos (flat_dr, bumpy_dr)  -> ~/runs/opt-lab-2/p3-videos/<name>/
  finetune  part3_finetune: Bumpy + Lab 4 DR warm start (budget per decision; --smoke = 50 it)
  eval-ft   part3_eval on the fine-tuned checkpoint  -> report/part3/eval/<name>-ft<N>/
  export-ft part3_export on the fine-tuned checkpoint
  table     summary table over both                  -> report/part3/eval/<name>_summary.md

Every GPU stage waits until fewer than ``--max-gpu-procs`` (default 4) compute processes
are on the GPU, and this driver runs one GPU process at a time.

  uv run --no-sync python scripts/opt_lab_2/part3_run.py --checkpoint CKPT --reference-dir REF \
      --name p3-final [--run-dir RUN_DIR] [--wandb-run mjlab/<id>] [--stages ...] [--smoke-finetune]
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

MJLAB_ROOT = Path(__file__).resolve().parents[2]
RUNS = Path.home() / "runs" / "opt-lab-2"
STAGES = ("export", "eval", "video", "finetune", "eval-ft", "export-ft", "table")
PY = [sys.executable]


def gpu_procs() -> int:
  r = subprocess.run(["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader"],
                     capture_output=True, text=True)
  return len([ln for ln in r.stdout.splitlines() if ln.strip()])


def wait_gpu(limit: int, poll: float = 5.0):
  t0 = time.time()
  while (n := gpu_procs()) >= limit:
    if int(time.time() - t0) % 600 < poll:
      print(f"[part3_run] GPU has {n} compute processes (limit {limit}); waiting...", flush=True)
    time.sleep(poll)


def run(cmd, limit, log: Path, gpu=True) -> int:
  if gpu:
    wait_gpu(limit)
  print(f"[part3_run] $ {' '.join(map(str, cmd))}\n[part3_run]   log: {log}", flush=True)
  log.parent.mkdir(parents=True, exist_ok=True)
  env = dict(os.environ, MUJOCO_GL="egl", PYOPENGL_PLATFORM="egl", PYTHONUNBUFFERED="1")
  with open(log, "a") as f:
    rc = subprocess.call(list(map(str, cmd)), cwd=MJLAB_ROOT, stdout=f, stderr=subprocess.STDOUT, env=env)
  print(f"[part3_run]   exit {rc}", flush=True)
  return rc


def latest_model(run_dir: Path) -> Path | None:
  models = sorted(run_dir.glob("model_*.pt"), key=lambda p: int(re.search(r"model_(\d+)", p.name).group(1)))
  return models[-1] if models else None


def main() -> int:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--checkpoint", type=Path, required=True)
  p.add_argument("--reference-dir", type=Path, required=True)
  p.add_argument("--name", required=True)
  p.add_argument("--run-dir", type=Path, help="Training run dir of the checkpoint (default: its parent)")
  p.add_argument("--wandb-run", help="entity/project/id of the checkpoint's run (checks the deploy.sh copy)")
  p.add_argument("--stages", nargs="+", choices=STAGES, default=list(STAGES))
  p.add_argument("--smoke-finetune", action="store_true", help="50-iteration smoke fine-tune instead of the full budget")
  p.add_argument("--finetune-iterations", type=int)
  p.add_argument("--max-gpu-procs", type=int, default=4)
  p.add_argument("--logs", type=Path, default=RUNS / "p3-logs")
  args = p.parse_args()
  ckpt, ref, name = args.checkpoint.resolve(), args.reference_dir, args.name
  run_dir = (args.run_dir or ckpt.parent).resolve()
  logs = args.logs / name
  status: dict[str, object] = {}
  common = ["--reference-dir", ref]

  def export(c, n, rd, wb):
    if (MJLAB_ROOT / f"report/part3/export/{n}.json").exists():
      return "skip"
    cmd = [*PY, "-m", "mjlab.scripts.part3_export", "--checkpoint", c, *common, "--name", n, "--run-dir", rd]
    return run(cmd + (["--wandb-run", wb] if wb else []), args.max_gpu_procs, logs / f"export-{n}.log")

  def evaluate(c, n):
    rc = run([*PY, "-m", "mjlab.scripts.part3_eval", "run", "--checkpoint", c, *common, "--name", n,
              "--modes", "suite", "seam", "--skip-existing"], args.max_gpu_procs, logs / f"eval-{n}.log")
    rc |= run([*PY, "-m", "mjlab.scripts.part3_eval", "run", "--checkpoint", c, *common, "--name", n,
               "--modes", "push", "--conditions", "flat_dr", "bumpy_dr", "--skip-existing"],
              args.max_gpu_procs, logs / f"eval-{n}.log")
    return rc

  ft_name = f"{name}-bumpyft" + ("-smoke" if args.smoke_finetune else "")
  ft_dir = RUNS / f"rp-{ft_name}"
  for stage in args.stages:
    t0 = time.time()
    if stage == "export":
      status[stage] = export(ckpt, name, run_dir, args.wandb_run)
    elif stage == "eval":
      status[stage] = evaluate(ckpt, name)
    elif stage == "video":
      rc = 0
      for cond in ("flat_dr", "bumpy_dr"):
        out = RUNS / "p3-videos" / name
        if (out / f"{name}-{cond}_seam_sweep.mp4").exists():
          continue
        rc |= run([*PY, "-m", "mjlab.scripts.part3_seam", "video", "--checkpoint", ckpt, *common,
                   "--name", f"{name}-{cond}", "--condition", cond, "--out-dir", out],
                  args.max_gpu_procs, logs / "video.log")
      status[stage] = rc
    elif stage == "finetune":
      if latest_model(ft_dir) is not None and (ft_dir.with_name(ft_dir.name + ".log")).exists() and \
         "exit code 0" in ft_dir.with_name(ft_dir.name + ".log").read_text():
        status[stage] = "skip"
        continue
      cmd = [*PY, "scripts/opt_lab_2/part3_finetune.py", "--checkpoint", ckpt, *common, "--name", ft_name, "--quiet"]
      if args.smoke_finetune:
        cmd.append("--smoke")
      if args.finetune_iterations:
        cmd += ["--iterations", str(args.finetune_iterations)]
      status[stage] = run(cmd, args.max_gpu_procs, logs / "finetune.log")
    elif stage in ("eval-ft", "export-ft"):
      ft = latest_model(ft_dir)
      if ft is None:
        status[stage] = f"no checkpoint in {ft_dir}"
        continue
      n = f"{ft_name}-{ft.stem.replace('model_', 'it')}"
      status[stage] = evaluate(ft, n) if stage == "eval-ft" else export(ft, n, ft_dir, None)
    elif stage == "table":
      dirs = [MJLAB_ROOT / "report/part3/eval" / name]
      ft = latest_model(ft_dir)
      if ft is not None:
        dirs.append(MJLAB_ROOT / "report/part3/eval" / f"{ft_name}-{ft.stem.replace('model_', 'it')}")
      dirs = [d for d in dirs if d.exists()]
      status[stage] = run([*PY, "-m", "mjlab.scripts.part3_eval", "table", *dirs, "--out",
                           MJLAB_ROOT / f"report/part3/eval/{name}_summary.md"], 99, logs / "table.log", gpu=False)
    print(f"[part3_run] stage {stage}: {status[stage]} ({time.time() - t0:.0f}s)", flush=True)
  print(json.dumps(status, indent=1, default=str))
  return 0 if all(v in (0, "skip") for v in status.values()) else 1


if __name__ == "__main__":
  sys.exit(main())
