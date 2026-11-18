"""Run the Part 2 bootstrapping DAG (W3): capture/fold/commit/push reference jobs and
500-iteration warm-start legs through launch.py, with a GPU-process cap and crash resume.

Plan file (JSON list, order = priority). Job kinds:

  {"name": "s42-r1-lag", "kind": "train", "seed": 42, "ref": "<refdir>|@<refjob>",
   "checkpoint": "<path>|@<trainjob>", "iterations": 500, "deps": [...]}
  {"name": "ref-s42-r2", "kind": "ref", "from": "@<trainjob>", "out": "configs/opt-lab-2/p2/...",
   "fold_args": [...], "deps": [...]}

A ref job captures the source job's final checkpoint with the reference that job trained on,
folds it, runs the extra fold check (scripts/opt_lab_2/part2_foldcheck.py), gates on
fold_summary warnings / flagged channels / kp*dq > 3 Nm / limit margin, then commits ONLY the
reference dir and pushes it (rebase on origin/markus/opt-lab-2, never force) under a git lock,
before any dependent leg can launch. A failed gate blocks the dependents and is reported.

Launch only while `nvidia-smi --query-compute-apps` counts fewer than --max-gpu processes
(all agents) and this queue runs fewer than --max-own. A train leg that exits non-zero is resumed
from its last checkpoint into `<name>-r<k>` so the final index equals parent_iter + iterations - 1.

  uv run --no-sync python scripts/opt_lab_2/part2_queue.py PLAN.json --status STATUS.json
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

RUNS = Path.home() / "runs/opt-lab-2"
W3 = RUNS / "w3"
MJ = Path(__file__).resolve().parents[2]  # pupper-mjlab/
WEIGHTS = "configs/opt-lab-2/weights/splay_stand_still.json"
GIT_LOCK = W3 / "git.lock"
LAUNCH_LOCK = W3 / "launch.lock"
BRANCH = "markus/opt-lab-2"
TRAILER = "\n\nCo-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
ENV = dict(os.environ, MUJOCO_GL="egl", PYOPENGL_PLATFORM="egl")


def log(msg):
  line = f"[{time.strftime('%H:%M:%S')}] {msg}"
  print(line, flush=True)


def gpu_procs() -> int:
  try:
    out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader"],
                         capture_output=True, text=True, timeout=30).stdout
    return len([l for l in out.splitlines() if l.strip()])
  except Exception:
    return 99


def ckpt_iter(p: Path) -> int:
  return int(re.findall(r"model_(\d+)\.pt", p.name)[0])


def last_checkpoint(out: Path):
  ck = sorted(out.glob("model_*.pt"), key=ckpt_iter)
  return ck[-1] if ck else None


def git(*a, check=True):
  r = subprocess.run(["git", "-C", str(MJ), *a], capture_output=True, text=True)
  if check and r.returncode:
    raise RuntimeError(f"git {' '.join(a)}: {r.stderr.strip()}")
  return r.stdout.strip()


def commit_and_push(paths: list[str], msg: str) -> str:
  """Commit only `paths`, rebase onto origin and push (no force). Returns the commit SHA."""
  GIT_LOCK.touch()
  with open(GIT_LOCK) as lk:
    fcntl.flock(lk, fcntl.LOCK_EX)
    git("add", "--", *paths)
    git("commit", "-m", msg + TRAILER, "--", *paths)
    for attempt in range(5):
      git("fetch", "origin", BRANCH)
      git("-c", "rebase.autoStash=true", "rebase", f"origin/{BRANCH}")
      r = subprocess.run(["git", "-C", str(MJ), "push", "origin", f"HEAD:{BRANCH}"], capture_output=True, text=True)
      if r.returncode == 0:
        break
      log(f"push attempt {attempt} failed: {r.stderr.strip()[:200]}")
      time.sleep(10)
    else:
      raise RuntimeError("push failed 5 times")
    return git("log", "-1", "--format=%H", "--", paths[0])


class Q:
  def __init__(self, plan, status_path, max_gpu, max_own):
    self.jobs = {j["name"]: dict(j, state="pending", segments=[]) for j in plan}
    self.order = [j["name"] for j in plan]
    self.status_path, self.max_gpu, self.max_own = status_path, max_gpu, max_own
    if status_path.exists():  # crash resume of the queue itself: keep finished jobs
      old = {j["name"]: j for j in json.loads(status_path.read_text())}
      for n, j in old.items():
        if n in self.jobs and j["state"] in ("done", "blocked"):
          self.jobs[n].update(j)
    self.procs: dict[str, subprocess.Popen] = {}

  def save(self):
    tmp = self.status_path.with_suffix(".tmp")
    tmp.write_text(json.dumps([self.jobs[n] for n in self.order], indent=1, default=str) + "\n")
    tmp.replace(self.status_path)

  # resolution -----------------------------------------------------------
  def train_ref(self, name):
    j = self.jobs[name]
    r = j["ref"]
    return self.jobs[r[1:]]["out"] if r.startswith("@") else r

  def checkpoint(self, spec):
    if spec.startswith("@"):
      return self.jobs[spec[1:]]["final_checkpoint"]
    return spec

  def deps(self, j):
    d = list(j.get("deps", []))
    for key in ("ref", "checkpoint", "from"):
      v = j.get(key)
      if isinstance(v, str) and v.startswith("@"):
        d.append(v[1:])
    return d

  def ready(self, j):
    return all(self.jobs[d]["state"] == "done" for d in self.deps(j))

  def blocked(self, j):
    return any(self.jobs[d]["state"] in ("blocked", "failed") for d in self.deps(j))

  # launching ------------------------------------------------------------
  def start_train(self, j, checkpoint, iterations, suffix=""):
    seg = j["name"] + suffix
    cmd = [sys.executable, "scripts/opt_lab_2/launch.py", "--out", str(RUNS / seg),
           "--reference-dir", self.train_ref(j["name"]), "--weights", WEIGHTS,
           "--checkpoint", checkpoint, "--iterations", str(iterations), "--seed", str(j["seed"]),
           "--num-envs", "4096", "--profile", "baseline", "--logger", "wandb", "--quiet",
           "--tags", "part2,w3", "--group", "rp-p2"]
    err = open(RUNS / f"{seg}.queue.err", "w")
    self.procs[j["name"]] = subprocess.Popen(cmd, cwd=MJ, stdout=subprocess.DEVNULL, stderr=err, env=ENV)
    j["segments"].append(dict(name="rp-" + seg, start=time.strftime("%H:%M:%S"), checkpoint=checkpoint,
                              iterations=iterations, head=git("rev-parse", "HEAD")))
    log(f"start {seg} ({iterations} it from {Path(checkpoint).parent.name}/{Path(checkpoint).name})")

  def start_ref(self, j):
    src = self.jobs[j["from"][1:]]
    ckpt = src["final_checkpoint"]
    ref = self.train_ref(src["name"])
    cap = W3 / f"capture-{j['name']}"
    script = W3 / f"{j['name']}.sh"
    fold = " ".join(j.get("fold_args", []))
    script.write_text(f"""set -euo pipefail
cd {MJ}
[ -f {cap}/trace.npz ] || {{ rm -rf {cap}; uv run --no-sync python -m mjlab.scripts.part2_reference capture --checkpoint {ckpt} --reference-dir {ref} --out {cap}; }}
rm -rf {j['out']}
uv run --no-sync python -m mjlab.scripts.part2_reference fold {cap} {j['out']} {fold}
cp {cap}/capture.json {j['out']}/capture.json
uv run --no-sync python scripts/opt_lab_2/part2_foldcheck.py {cap} {ref} {j['out']} --lag {j.get('lag', 2)} --out {j['out']}/foldcheck.json > /dev/null
""")
    log_f = open(W3 / f"{j['name']}.log", "w")
    self.procs[j["name"]] = subprocess.Popen(["bash", str(script)], stdout=log_f, stderr=subprocess.STDOUT, env=ENV)
    j["segments"].append(dict(name=j["name"], start=time.strftime("%H:%M:%S"), capture_from=ckpt, capture_ref=ref))
    log(f"start ref job {j['name']} (capture {Path(ckpt).parent.name}/{Path(ckpt).name})")

  def gate(self, j):
    out = MJ / j["out"]
    s = json.loads((out / "fold_summary.json").read_text())
    fc = json.loads((out / "foldcheck.json").read_text())
    problems = []
    if any(s.get("warnings", {}).values()):
      problems.append(f"warnings {s['warnings']}")
    if any(s.get("flagged_channels", {}).values()):
      problems.append(f"flagged {s['flagged_channels']}")
    if not s.get("verify", {}).get("ok", False):
      problems.append("verify failed")
    name = out.name
    for k, d in fc["directions"].items():
      e = d[name]
      if e["kp_dq_nm"] > 3.0:
        problems.append(f"{k} kp*dq {e['kp_dq_nm']:.2f} Nm > 3")
      if e["limit_margin_min_rad"] < 0.05:
        problems.append(f"{k} limit margin {e['limit_margin_min_rad']:.3f}")
      if e["rms_resid_rad_mean"] > d["parent"]["rms_resid_rad_mean"] + 0.02:
        problems.append(f"{k} fold residual {e['rms_resid_rad_mean']:.3f} > source ref {d['parent']['rms_resid_rad_mean']:.3f}")
    j["gate"] = dict(problems=problems, measured_lag=s.get("measured_lag_steps"),
                     rms_vs_parent=s.get("rms_diff_vs_parent_rad"),
                     kp_dq={k: fc["directions"][k][name]["kp_dq_nm"] for k in fc["directions"]})
    return problems

  # main loop ------------------------------------------------------------
  def step(self):
    for name, proc in list(self.procs.items()):
      rc = proc.poll()
      if rc is None:
        continue
      del self.procs[name]
      j = self.jobs[name]
      j["segments"][-1].update(exit=rc, end=time.strftime("%H:%M:%S"))
      if j["kind"] == "ref":
        if rc != 0:
          j["state"] = "failed"; log(f"REF JOB FAILED {name} rc={rc}")
          continue
        problems = self.gate(j)
        if problems and not j.get("allow_gate"):
          j["state"] = "blocked"; log(f"GATE BLOCKED {name}: {problems}")
          continue
        try:
          j["commit"] = commit_and_push([j["out"]], j.get("message", f"Part 2: install reference {j['out']}"))
          j["state"] = "done"; log(f"ref {name} committed+pushed {j['commit'][:10]}")
        except Exception as e:
          j["state"] = "failed"; j["error"] = str(e); log(f"PUSH FAILED {name}: {e}")
        continue
      seg = j["segments"][-1]
      out = RUNS / seg["name"]
      ck = last_checkpoint(out)
      target = j["target_iter"]
      fin = ckpt_iter(ck) if ck else -1
      if fin == target and rc == 0:
        j["state"] = "done"; j["final_checkpoint"] = str(ck); log(f"done {name} -> {ck.name}")
      elif rc != 0 and ck is not None and fin < target and j.get("resumes", 0) < 3:
        j["resumes"] = j.get("resumes", 0) + 1
        log(f"CRASH {seg['name']} rc={rc} at {fin}; resuming")
        self.start_train(j, str(ck), target - fin + 1, suffix=f"-r{j['resumes'] + 1}")
      else:
        j["state"] = "failed"; j["final_checkpoint"] = str(ck) if ck else None
        log(f"FAILED {name} rc={rc} last={ck}")
    # launch (a global lock serializes check+launch across W3 queues; wait until the new
    # process shows in nvidia-smi so the next count is right)
    for name in self.order:
      j = self.jobs[name]
      if j["state"] != "pending":
        continue
      if self.blocked(j):
        j["state"] = "blocked"; log(f"blocked {name} (dependency)"); continue
      if not self.ready(j) or len(self.procs) >= self.max_own:
        continue
      LAUNCH_LOCK.touch()
      with open(LAUNCH_LOCK) as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        n0 = gpu_procs()
        if n0 >= self.max_gpu:
          break
        if j["kind"] == "ref":
          self.start_ref(j)
        else:
          ck = self.checkpoint(j["checkpoint"])
          j["parent_iter"] = ckpt_iter(Path(ck))
          j["target_iter"] = j["parent_iter"] + j["iterations"] - 1
          self.start_train(j, ck, j["iterations"])
        j["state"] = "running"
        self.save()
        t0 = time.time()
        while time.time() - t0 < 150 and gpu_procs() <= n0 and self.procs[name].poll() is None:
          time.sleep(5)
    self.save()

  def run(self):
    # Jobs left "running" by a dead queue: mark pending again only if no output exists.
    for j in self.jobs.values():
      if j["state"] == "running":
        raise SystemExit(f"{j['name']} was running when the queue died; inspect and fix the status file")
    while True:
      self.step()
      if not any(j["state"] in ("pending", "running") for j in self.jobs.values()):
        break
      if not self.procs and not any(self.ready(j) and j["state"] == "pending" for j in self.jobs.values()):
        break  # nothing running and nothing launchable: dependencies blocked or failed
      time.sleep(30)
    self.save()
    log("queue finished: " + ", ".join(f"{n}={self.jobs[n]['state']}" for n in self.order))


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("plan", type=Path)
  p.add_argument("--status", type=Path, required=True)
  p.add_argument("--max-gpu", type=int, default=4, help="total GPU compute processes, all agents")
  p.add_argument("--max-own", type=int, default=4)
  a = p.parse_args()
  W3.mkdir(parents=True, exist_ok=True)
  Q(json.loads(a.plan.read_text()), a.status, a.max_gpu, a.max_own).run()


if __name__ == "__main__":
  main()
