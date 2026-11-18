"""Evaluate Part 2 legs as they finish (W3): W2 suite + push on each final checkpoint with the
reference it trained on, one eval process at a time, only while the GPU has a free slot
(total compute processes < --max-gpu, all agents), sharing the queue's launch lock.

Sources: the part2_queue status files (train jobs in state "done") plus --extra
name=ckpt:ref entries (the two parents). Results: <out-dir>/<name>.json and <name>_push.json.

  uv run --no-sync python scripts/opt_lab_2/part2_evalq.py --status S1.json S2.json \
      --extra p1-gallop_c-s42=CKPT:REF --out-dir report/part2/eval/w3
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
import subprocess
import sys
import time
from pathlib import Path

MJ = Path(__file__).resolve().parents[2]
W3 = Path.home() / "runs/opt-lab-2/w3"
LAUNCH_LOCK = W3 / "launch.lock"
ENV = dict(os.environ, MUJOCO_GL="egl", PYOPENGL_PLATFORM="egl")


def gpu_procs() -> int:
  try:
    out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader"],
                         capture_output=True, text=True, timeout=30).stdout
    return len([l for l in out.splitlines() if l.strip()])
  except Exception:
    return 99


def log(m):
  print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)


def entries(status_files, extra):
  out = {}
  for e in extra:
    name, rest = e.split("=", 1)
    ck, ref = rest.split(":", 1)
    out[name] = (ck, ref)
  pending_any = False
  for sf in status_files:
    if not Path(sf).exists():
      pending_any = True
      continue
    jobs = {j["name"]: j for j in json.loads(Path(sf).read_text())}
    for j in jobs.values():
      if j["kind"] != "train":
        continue
      if j["state"] in ("pending", "running"):
        pending_any = True
      if j["state"] != "done":
        continue
      r = j["ref"]
      ref = jobs[r[1:]]["out"] if r.startswith("@") else r
      out[j["name"]] = (j["final_checkpoint"], ref)
  return out, pending_any


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--status", nargs="+", default=[])
  p.add_argument("--extra", action="append", default=[])
  p.add_argument("--out-dir", type=Path, default=MJ / "report/part2/eval/w3")
  p.add_argument("--max-gpu", type=int, default=4)
  p.add_argument("--once", action="store_true", help="exit when nothing is left to evaluate now")
  a = p.parse_args()
  a.out_dir.mkdir(parents=True, exist_ok=True)
  failed: dict[str, int] = {}
  while True:
    todo, pending_any = entries(a.status, a.extra)
    left = [(n, ck, ref) for n, (ck, ref) in todo.items()
            if not ((a.out_dir / f"{n}.json").exists() and (a.out_dir / f"{n}_push.json").exists())
            and failed.get(n, 0) < 2]
    if not left:
      if a.once or not pending_any:
        break
      time.sleep(60)
      continue
    name, ck, ref = left[0]
    LAUNCH_LOCK.touch()
    with open(LAUNCH_LOCK) as lk:
      fcntl.flock(lk, fcntl.LOCK_EX)
      if gpu_procs() >= a.max_gpu:
        busy = True
      else:
        busy = False
        log(f"eval {name}: {ck} ref {ref}")
        proc = subprocess.Popen(
          [sys.executable, "-m", "mjlab.scripts.part2_eval", "batch", f"{ck}:{(MJ / ref).resolve()}={name}",
           "--modes", "suite", "push", "--skip-existing", "--out-dir", str(a.out_dir)],
          cwd=MJ, env=ENV, stdout=open(W3 / f"eval-{name}.log", "a"), stderr=subprocess.STDOUT)
        n0 = gpu_procs(); t0 = time.time()
        while time.time() - t0 < 120 and gpu_procs() <= n0 and proc.poll() is None:
          time.sleep(5)
    if busy:
      time.sleep(60)
      continue
    rc = proc.wait()
    ok = (a.out_dir / f"{name}.json").exists() and (a.out_dir / f"{name}_push.json").exists()
    log(f"eval {name} rc={rc} ok={ok}")
    if not ok:
      failed[name] = failed.get(name, 0) + 1
  log("eval queue finished; failed: " + json.dumps(failed))


if __name__ == "__main__":
  main()
