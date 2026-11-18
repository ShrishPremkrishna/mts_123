"""Atomic GPU-slot acquisition shared with W3's ``part2_queue.py`` (check + launch as one step).

``part2_queue.py`` takes an exclusive ``flock`` on ``~/runs/opt-lab-2/w3/launch.lock``, counts the
GPU compute processes, launches only if the count is below its cap, and keeps the lock until the new
process shows up in ``nvidia-smi`` (at most 150 s). Part 3 launches do exactly the same under the
same lock file, with the cap read from ``~/recovery/CONCURRENCY`` (decision 2026-11-18-w0-concurrency),
so two agents can no longer both see "3" and both launch (which once gave 5 trainers).

Not for nesting: a process holding the lock must not start a child that takes it too.
"""

from __future__ import annotations

import fcntl
import os
import re
import subprocess
import time
from pathlib import Path

LAUNCH_LOCK = Path.home() / "runs" / "opt-lab-2" / "w3" / "launch.lock"
CONCURRENCY_FILE = Path.home() / "recovery" / "CONCURRENCY"
DEFAULT_CAP = 4
REGISTER_WAIT_S = 150


def gpu_procs() -> int:
  try:
    out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader"],
                         capture_output=True, text=True, timeout=30).stdout
    return len([ln for ln in out.splitlines() if ln.strip()])
  except Exception:  # noqa: BLE001 - unknown means "full"
    return 99


def cap() -> int:
  """The global concurrency cap (first integer in ~/recovery/CONCURRENCY; 4 if unreadable)."""
  try:
    m = re.search(r"\d+", CONCURRENCY_FILE.read_text())
    return int(m.group()) if m else DEFAULT_CAP
  except OSError:
    return DEFAULT_CAP


def launch(cmd, *, cwd=None, env=None, stdout=None, stderr=None, poll: float = 5.0, log=print) -> subprocess.Popen:
  """Wait for a free slot under the shared lock, start ``cmd``, keep the lock until it registers.

  Returns the Popen. Registration = the GPU process count rises above the pre-launch count, or
  ``REGISTER_WAIT_S`` passes, or the process exits (CPU-only failures exit early).
  """
  LAUNCH_LOCK.parent.mkdir(parents=True, exist_ok=True)
  LAUNCH_LOCK.touch()
  last_msg = 0.0
  while True:
    with open(LAUNCH_LOCK) as lk:
      fcntl.flock(lk, fcntl.LOCK_EX)
      n0, limit = gpu_procs(), cap()
      if n0 < limit:
        proc = subprocess.Popen(list(map(str, cmd)), cwd=cwd, env=env, stdout=stdout, stderr=stderr)
        t0 = time.time()
        while time.time() - t0 < REGISTER_WAIT_S and gpu_procs() <= n0 and proc.poll() is None:
          time.sleep(5)
        log(f"[gpu_slot] launched pid {proc.pid} at {n0}/{limit} GPU processes "
            f"(registered after {time.time() - t0:.0f}s)")
        return proc
    if time.time() - last_msg > 600:
      log(f"[gpu_slot] GPU has {n0} compute processes (cap {limit} from {CONCURRENCY_FILE}); waiting...")
      last_msg = time.time()
    time.sleep(poll)


def run(cmd, **kw) -> int:
  """``launch`` + wait; returns the exit code."""
  return launch(cmd, **kw).wait()


if __name__ == "__main__":
  print(dict(cap=cap(), gpu_procs=gpu_procs(), lock=str(LAUNCH_LOCK), pid=os.getpid()))
