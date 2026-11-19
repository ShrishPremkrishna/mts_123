"""Run gpu_bench.py configs one at a time on an otherwise idle GPU (report/infra/gpu_bottleneck.md).

For each config: wait until the GPU's compute processes are a subset of --background PIDs (the
long-running viewers), launch through gpu_slot (shared launch lock), and record alongside the run
  <tag>.dmon      nvidia-smi dmon -s puct -d 1 (power, SM%, mem%, clocks, PCIe)
  <tag>.apps      compute-app PIDs every 2 s (contamination label)
  <tag>.pidstat   pidstat -t 1 of the bench process (per-thread CPU%)
Concurrent mode (--concurrent N) launches N copies of the same config at once.
"""
from __future__ import annotations

import argparse
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gpu_slot  # noqa: E402


def pids():
  out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader"],
                       capture_output=True, text=True).stdout
  return {int(x) for x in out.split() if x.strip().isdigit()}


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--root", type=Path, required=True)
  p.add_argument("--background", default="", help="comma-separated PIDs tolerated as background load")
  p.add_argument("--config", action="append", required=True, help="tag=extra gpu_bench args")
  p.add_argument("--concurrent", type=int, default=1)
  a = p.parse_args()
  bg = {int(x) for x in a.background.split(",") if x}
  a.root.mkdir(parents=True, exist_ok=True)
  py = HERE.parents[1] / ".venv/bin/python"
  for spec in a.config:
    tag, extra = spec.split("=", 1)
    while not pids() <= bg:
      print(f"[sweep] {tag}: foreign GPU procs {sorted(pids() - bg)}; waiting", flush=True)
      time.sleep(20)
    stop = threading.Event()
    def poll_apps():
      with open(a.root / f"{tag}.apps", "w") as f:
        while not stop.is_set():
          f.write(f"{time.time():.1f} {sorted(pids())}\n"); f.flush(); time.sleep(2)
    th = threading.Thread(target=poll_apps, daemon=True); th.start()
    dmon = subprocess.Popen(["bash", "-c", "nvidia-smi dmon -s puct -d 1 | while IFS= read -r l; do echo \"$(date +%s) $l\"; done"],
                            stdout=open(a.root / f"{tag}.dmon", "w"), stderr=subprocess.DEVNULL,
                            start_new_session=True)
    procs, stats = [], []
    lk = None
    if a.concurrent > 1:  # take the shared launch lock ourselves and launch all copies under it
      import fcntl
      lk = open(gpu_slot.LAUNCH_LOCK); fcntl.flock(lk, fcntl.LOCK_EX)
      assert gpu_slot.gpu_procs() + a.concurrent <= gpu_slot.cap(), "concurrency cap"
    for i in range(a.concurrent):
      sfx = f"-{i}" if a.concurrent > 1 else ""
      cmd = [str(py), str(HERE / "gpu_bench.py"), *shlex.split(extra),
             "--out", str(a.root / f"{tag}{sfx}"), "--json", str(a.root / f"{tag}{sfx}.json")]
      log = open(a.root / f"{tag}{sfx}.log", "w")
      if a.concurrent > 1:
        pr = subprocess.Popen(cmd, cwd=HERE.parents[1], stdout=log, stderr=subprocess.STDOUT)  # lock held below
      else:
        pr = gpu_slot.launch(cmd, cwd=HERE.parents[1], stdout=log, stderr=subprocess.STDOUT,
                             log=lambda m: print(m, flush=True))
      procs.append(pr)
      stats.append(subprocess.Popen(["pidstat", "-t", "-u", "-p", str(pr.pid), "1"],
                                    stdout=open(a.root / f"{tag}{sfx}.pidstat", "w")))
    if lk:
      t0 = time.time()
      while len(pids() - bg) < a.concurrent and time.time() - t0 < 150: time.sleep(3)
      lk.close()
    rc = [pr.wait() for pr in procs]
    for s in stats: s.terminate()
    os.killpg(dmon.pid, signal.SIGTERM); stop.set(); th.join()
    print(f"[sweep] {tag} done rc={rc}", flush=True)


if __name__ == "__main__":
  main()
