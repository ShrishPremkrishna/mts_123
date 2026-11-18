"""Run Part 1 training arms through launch.py with a concurrency cap and crash resume.

Each job is (name, reference_dir or None, seed). At most --max-concurrent run at once
as plain separate processes (no MPS). If a job exits non-zero, it is resumed once per
crash from its last 50-iteration checkpoint into `<name>-r<k>` with the remaining
iterations (the warm start drops the airborne pretrain, as the old -r2 runs did), up to
--max-resumes times. A JSON status file records PIDs, exit codes and resumes.

  uv run --no-sync python scripts/opt_lab_2/part1_queue.py --max-concurrent 3 \
      --job p1-gallop_c-s42:configs/opt-lab-2/gallop_c:42 --job p1-baseline-s42::42 ...
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

RUNS = Path.home() / "runs/opt-lab-2"
WEIGHTS = "configs/opt-lab-2/weights/splay.json"


def last_checkpoint(out: Path):
  ckpts = sorted(out.glob("model_*.pt"), key=lambda p: int(re.findall(r"\d+", p.name)[0]))
  return ckpts[-1] if ckpts else None


def start(job, iterations, checkpoint=None, suffix=""):
  name = job["name"] + suffix
  cmd = [sys.executable, "scripts/opt_lab_2/launch.py", "--out", str(RUNS / name), "--weights", WEIGHTS,
         "--iterations", str(iterations), "--seed", str(job["seed"]), "--num-envs", "4096",
         "--profile", "baseline", "--logger", "wandb", "--quiet"]
  if job["ref"]:
    cmd += ["--reference-dir", job["ref"]]
  if checkpoint:
    cmd += ["--checkpoint", str(checkpoint)]
  proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=open(RUNS / f"{name}.queue.err", "w"))
  return proc, name


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--job", action="append", required=True, help="name:refdir:seed (empty refdir = stock trot)")
  p.add_argument("--max-concurrent", type=int, required=True)
  p.add_argument("--iterations", type=int, default=3000)
  p.add_argument("--max-resumes", type=int, default=3)
  p.add_argument("--status", type=Path, default=RUNS / "part1_queue_status.json")
  args = p.parse_args()
  RUNS.mkdir(parents=True, exist_ok=True)
  pending = []
  for spec in args.job:
    name, ref, seed = spec.split(":")
    pending.append(dict(name=name, ref=ref or None, seed=int(seed), segments=[], resumes=0))
  running, done = [], []

  def save():
    rows = [dict(j, **({"pid": None} if "proc" not in j else {"pid": j["proc"].pid})) for j in pending + running + done]
    for r in rows:
      r.pop("proc", None)
    args.status.write_text(json.dumps(rows, indent=1, default=str) + "\n")

  while pending or running:
    while pending and len(running) < args.max_concurrent:
      job = pending.pop(0)
      job["proc"], seg = start(job, args.iterations)
      job["segments"].append(dict(name=seg, start=time.strftime("%H:%M:%S"), iterations=args.iterations))
      running.append(job); save(); time.sleep(20)  # stagger startup compilation
    time.sleep(30)
    for job in list(running):
      rc = job["proc"].poll()
      if rc is None:
        continue
      job["segments"][-1].update(exit=rc, end=time.strftime("%H:%M:%S"))
      out = RUNS / ("rp-" + job["segments"][-1]["name"])
      ck = last_checkpoint(out)
      final_iter = int(re.findall(r"\d+", ck.name)[0]) if ck else -1
      target = job["segments"][0]["iterations"] - 1
      if rc != 0 and job["resumes"] < args.max_resumes and ck is not None and final_iter < target - 1:
        job["resumes"] += 1
        remaining = target - final_iter
        job["proc"], seg = start(job, remaining, checkpoint=ck, suffix=f"-r{job['resumes'] + 1}")
        job["segments"].append(dict(name=seg, start=time.strftime("%H:%M:%S"), iterations=remaining,
                                    resumed_from=str(ck)))
      else:
        job.pop("proc"); job["final_checkpoint"] = str(ck) if ck else None
        running.remove(job); done.append(job)
      save()
  save()


if __name__ == "__main__":
  main()
