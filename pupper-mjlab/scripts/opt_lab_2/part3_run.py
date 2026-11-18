#!/usr/bin/env python3
"""Opt Lab 2 Part 3 runbook: one command from a final MixedGaits checkpoint to Part 3 sim evidence.

Stages (run in order; finished work is skipped on re-runs; simulation only):

  export      part3_export on the checkpoint: policy.json + golden fixture, round trip, limit audit,
              deploy validator, host C++ gait-reference test     -> report/part3/export/<name>.{json,md}
  eval        part3_eval: suite + seam on flat/Bumpy x Lab 4 DR on/off, push on DR on -> report/part3/eval/<name>/
  video       seam-sweep transition videos (flat_dr, bumpy_dr) -> ~/runs/opt-lab-2/p3-videos/<name>/
  kpcheck     kp-coverage decision step 0: flat_dr vs nominal-PD ("nompd") falls on eval seeds 1234 + 4321;
              mean delta > 15 -> WIDEN (train kp x[0.6,1.3], kd x[0.7,1.5]), else KEEP Lab 4
                                                               -> report/part3/eval/<name>_kp_coverage.{md,json}
  smoke-wide  WIDEN only: <= 50-iteration smoke of both arms (concurrently) + the decision's smoke checks;
              a failed check stops the runbook                -> report/part3/eval/<name>_kpwide_smoke.{md,json}
  train       the Bumpy fine-tune and its matched flat control, trained TOGETHER in 500-iteration legs
              (budget 1500 = 3 legs). After every leg, both arms' leg checkpoints are evaluated identically
              (flat_dr + bumpy_dr suite + seam and nompd; the full matrix + push from the third leg on), and a
              gate decides: overflow/NaN or the +500 G2 rule -> STOP before the next leg; after the planned
              legs, the extension rule extends BOTH arms by one leg (cap +3000) or neither
                                                               -> report/part3/eval/<name><branch>_gates.md
  select      matched selection: ship the last leg unless it regresses vs the previous leg by > 10 falls
              (flat_dr, bumpy_dr or nompd) or flips the flat_dr seam verdict on seed 1234 AND the same
              regression replicates on seed 4321 (evaluated here); the matched control is the control at the
              same leg                                         -> report/part3/eval/<name><branch>_selection.{md,json}
  confirm     the selected checkpoint on eval seed 4321 (flat_dr, bumpy_dr, nompd) + pooled seam verdict
  hold        the kp-coverage deployment HOLD rule (matched control) -> <name><branch>_deploy_hold.{md,json};
              a HOLD in the KEEP branch launches the pre-authorized WIDEN pair (unless --no-escalate)
  export-ft   part3_export on the selected checkpoint (+ W&B check; --upload-wandb publishes it)
  table       summary table over everything                     -> report/part3/eval/<name><branch>_summary.md

Decisions: 2026-11-18-w4-finetune-budget (amended by its devil's-advocate review), -seam-thresholds,
-kp-coverage. GPU: every launch goes through scripts/opt_lab_2/gpu_slot.py, which takes the same flock
as W3's part2_queue.py and reads the cap from ~/recovery/CONCURRENCY, so check + launch are one step.
W4 runs at most 2 GPU processes at once (the two training arms); evaluations run one at a time.

Wall clock (measured, 4-way shared A100): ~50 min pre-stages, ~3 h training (2 arms x 1500 iterations),
~45-75 min of gate/selection evals: about 4.5-5 h end to end (~2.5 h if the GPU is otherwise idle).
A KEEP-branch HOLD that escalates to WIDEN adds ~4 h.

  uv run --no-sync python scripts/opt_lab_2/part3_run.py --checkpoint CKPT --reference-dir REF --name p3-final \
      [--run-dir RUN_DIR] [--wandb-run ENTITY/mjlab/ID] [--stages ...] [--smoke-finetune] [--dry-run]
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gpu_slot  # noqa: E402

MJLAB_ROOT = Path(__file__).resolve().parents[2]
EVAL_ROOT = MJLAB_ROOT / "report/part3/eval"
RUNS = Path.home() / "runs" / "opt-lab-2"
STAGES = ("export", "eval", "video", "kpcheck", "smoke-wide", "train", "select", "confirm", "hold", "export-ft", "table")
NOISE = 10  # falls; budget-decision guard band
KP_DELTA = 15  # falls; kp-coverage decision threshold
LEG = 500  # iterations per training leg (the gate granularity)
FULL_ITERATIONS = 1500  # decision 2026-11-18-w4-finetune-budget
MAX_ITERATIONS = 3000  # extension cap
SMOKE_LEG = 50
SEED2 = 4321
WIDE = dict(kp_range=(0.6, 1.3), kd_range=(0.7, 1.5))
NOMINAL_KP, NOMINAL_KD = 5.5, 0.25
PY = [sys.executable]
DRY = False


def say(msg):
  print(f"[part3_run] {msg}", flush=True)


def run(cmd, log: Path, gpu=True) -> int:
  """Run a subprocess; GPU ones acquire a slot under the shared lock first."""
  cmd = list(map(str, cmd))
  say(f"$ {' '.join(cmd)}\n[part3_run]   log: {log}")
  if DRY:
    return 0
  log.parent.mkdir(parents=True, exist_ok=True)
  env = dict(os.environ, MUJOCO_GL="egl", PYOPENGL_PLATFORM="egl", PYTHONUNBUFFERED="1")
  with open(log, "a") as f:
    if gpu:
      rc = gpu_slot.run(cmd, cwd=MJLAB_ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, log=say)
    else:
      rc = subprocess.call(cmd, cwd=MJLAB_ROOT, stdout=f, stderr=subprocess.STDOUT, env=env)
  say(f"  exit {rc}")
  return rc


def run_parallel(cmds: list[tuple[list, Path]]) -> int:
  """Start several part3_finetune launches together (each takes its own GPU slot); wait for all."""
  procs = []
  for cmd, log in cmds:
    cmd = list(map(str, cmd))
    say(f"$ {' '.join(cmd)}  &\n[part3_run]   log: {log}")
    if DRY:
      continue
    log.parent.mkdir(parents=True, exist_ok=True)
    f = open(log, "a")  # noqa: SIM115 - closed below
    env = dict(os.environ, MUJOCO_GL="egl", PYOPENGL_PLATFORM="egl", PYTHONUNBUFFERED="1")
    procs.append((subprocess.Popen(cmd, cwd=MJLAB_ROOT, stdout=f, stderr=subprocess.STDOUT, env=env), f))
  rc = 0
  for p, f in procs:
    rc |= p.wait()
    f.close()
  say(f"  parallel exit {rc}")
  return rc


def wandb_run_path(run_dir: Path) -> str | None:
  """entity/project/id of a launch.py run, from its log ('fetch with: ... download_policy.py <path>')."""
  log = run_dir.with_name(run_dir.name + ".log")
  if not log.exists():
    return None
  hits = re.findall(r"download_policy\.py (\S+/\S+/\S+)", log.read_text(errors="ignore"))
  return hits[-1] if hits else None


def latest_model(run_dir: Path) -> Path | None:
  models = sorted(run_dir.glob("model_*.pt"), key=lambda p: int(re.search(r"model_(\d+)", p.name).group(1)))
  return models[-1] if models else None


def model_iter(p: Path) -> int:
  return int(re.search(r"model_(\d+)", p.name).group(1))


def eval_name(prefix: str, ckpt: Path) -> str:
  return f"{prefix}-it{model_iter(ckpt)}"


def run_ok(run_dir: Path) -> bool:
  log = run_dir.with_name(run_dir.name + ".log")
  return latest_model(run_dir) is not None and log.exists() and "[launch] exit code 0" in log.read_text(errors="ignore")


##
# Readers of evaluation outputs.
##


def load_nompd(n: str, seed: int) -> int | None:
  tag = "" if seed == 1234 else f"-seed{seed}"
  f = EVAL_ROOT / n / "ablate" / f"{n}-flat-nominal_pd_gains{tag}.json"
  return json.loads(f.read_text())["metrics"]["falls_total"] if f.exists() else None


def load_falls(n: str, cond: str) -> int | None:
  f = EVAL_ROOT / n / f"{n}-{cond}.json"
  return json.loads(f.read_text())["metrics"]["falls_total"] if f.exists() else None


def load_seam(n: str, cond: str) -> str | None:
  f = EVAL_ROOT / n / f"{n}-{cond}_seam.json"
  return json.loads(f.read_text())["metrics"]["verdict"]["verdict"] if f.exists() else None


def overflow_ok(n: str) -> bool:
  for f in (EVAL_ROOT / n).glob(f"{n}-*.json"):
    if f.name.endswith("_overflow.json"):
      continue
    ov = (json.loads(f.read_text()).get("part3_condition") or {}).get("overflow_audit")
    if ov is not None and not ov.get("zero_overflow", False):
      return False
  return True


def point_metrics(n: str, seed: int = 1234) -> dict:
  """{flat_dr, bumpy_dr, nompd: falls, seam_flat: verdict} of one evaluated checkpoint (eval name n)."""
  m = n if seed == 1234 else f"{n}-seed{seed}"
  return dict(flat_dr=load_falls(m, "flat_dr"), bumpy_dr=load_falls(m, "bumpy_dr"), nompd=load_nompd(n, seed),
              seam_flat=load_seam(m, "flat_dr"))


##
# Pure decision logic (unit tested in tests/test_part3_run.py).
##


def gate_after_leg(k: int, base: dict, bumpy: dict, invalid: list[str]) -> dict:
  """Between-leg gate. k = completed leg index (1-based); base/bumpy = point_metrics of +0 / the bumpy arm."""
  if invalid:
    return dict(action="stop", reasons=invalid)
  if k == 1 and None not in (bumpy["flat_dr"], base["flat_dr"], bumpy["bumpy_dr"], base["bumpy_dr"]):
    if bumpy["flat_dr"] >= 2 * max(base["flat_dr"], 1) and bumpy["bumpy_dr"] >= base["bumpy_dr"]:
      return dict(action="stop", reasons=[
        f"G2 at +{LEG}: flat_dr {bumpy['flat_dr']} >= 2 x +0's {base['flat_dr']} and bumpy_dr {bumpy['bumpy_dr']} "
        f"no better than +0's {base['bumpy_dr']} -> stop training, re-decide"])
  return dict(action="continue", reasons=[])


def extension_due(base: dict, prev: dict, last: dict, total_iters: int) -> tuple[bool, str]:
  """Lockstep extension rule: bumpy_dr still dropping > NOISE over the last leg and flat_dr within NOISE of +0."""
  if total_iters + LEG > MAX_ITERATIONS:
    return False, f"cap +{MAX_ITERATIONS} reached"
  if None in (prev["bumpy_dr"], last["bumpy_dr"], last["flat_dr"], base["flat_dr"]):
    return False, "missing evaluations"
  drop, drift = prev["bumpy_dr"] - last["bumpy_dr"], last["flat_dr"] - base["flat_dr"]
  ok = drop > NOISE and abs(drift) <= NOISE
  return ok, f"bumpy_dr drop {drop:+d} (> {NOISE}?) with flat_dr drift {drift:+d} vs +0 (|.| <= {NOISE}?)"


def regressions(final: dict, prev: dict) -> list[str]:
  """Metrics on which `final` is worse than `prev` by > NOISE falls, plus a flat_dr seam PASS -> FAIL flip."""
  out = [m for m in ("flat_dr", "bumpy_dr", "nompd")
         if None not in (final[m], prev[m]) and final[m] - prev[m] > NOISE]
  if prev.get("seam_flat") == "PASS" and final.get("seam_flat") == "FAIL":
    out.append("seam_flat")
  return out


def matched_selection(keys: list[str], ev1: dict, ev2_lookup) -> dict:
  """Ship the last point unless a seed-1234 regression vs the previous point replicates on seed 4321.

  keys: ordered point keys ("+500", ...); ev1[key] = point_metrics(seed 1234) of the bumpy arm;
  ev2_lookup(key) -> point_metrics(seed 4321) (evaluated on demand by the caller).
  """
  final = keys[-1]
  if len(keys) < 2:
    return dict(selected=final, notes=[f"ship {final} (only point)"])
  prev = keys[-2]
  r1 = regressions(ev1[final], ev1[prev])
  if not r1:
    return dict(selected=final, notes=[f"ship {final} (no regression vs {prev} on seed 1234)"])
  r2 = regressions(ev2_lookup(final), ev2_lookup(prev))
  both = [m for m in r1 if m in r2]
  if both:
    return dict(selected=prev, notes=[f"fallback to {prev}: {final} regresses on {both} on BOTH eval seeds "
                                      f"(seed 1234: {r1}; seed {SEED2}: {r2}); report as a SELECTION"])
  return dict(selected=final, notes=[f"ship {final}: seed-1234 regression {r1} did not replicate on seed {SEED2} ({r2})"])


##
# Checks reused from the kp-coverage decision.
##


def _check(name, ok, detail):
  return dict(name=name, ok=bool(ok), detail=detail)


def smoke_checks(bumpy_dir: Path, ctrl_dir: Path, bumpy_eval: str | None, it_b: int, it_c: int) -> dict:
  """The kp-coverage decision's WIDEN smoke items 1-5 (CPU; reads run dirs and reports)."""
  import numpy as np

  checks = []
  lab4 = MJLAB_ROOT / "lab4_runs/shrish_stablegait/training-settings.json"
  r = subprocess.run(["git", "-C", str(MJLAB_ROOT), "diff", "--quiet", "origin/main", "--", str(lab4)])
  checks.append(_check("1. Lab 4 record byte-unchanged vs origin/main", r.returncode == 0, str(lab4.relative_to(MJLAB_ROOT))))
  for label, d, iters in (("bumpy arm", bumpy_dir, it_b), ("control arm", ctrl_dir, it_c)):
    lj = d / "launch.json"
    meta = json.loads(lj.read_text()) if lj.exists() else {}
    ov = meta.get("pd_gain_override") or {}
    checks.append(_check(f"1. {label}: run config records the WIDEN override and tag",
                         ov.get("kp_range") == list(WIDE["kp_range"]) and ov.get("kd_range") == list(WIDE["kd_range"])
                         and "p3dr-widekp" in (meta.get("tags") or []), f"override {ov}, tags {meta.get('tags')}"))
    snap = d / "randomization_snapshot.npz"
    if snap.exists():
      z = np.load(snap)
      kp, kd = z["pd_stiffness"] / NOMINAL_KP, z["pd_damping"] / NOMINAL_KD
      frac = float((kp.mean(1) >= 1.0).mean())
      checks.append(_check(f"2. {label}: realized kp/5.5 spans ~0.6-1.3, kd/0.25 ~0.7-1.5, 17-23 % envs mean kp >= 1.0",
                           0.58 <= kp.min() <= 0.65 and 1.25 <= kp.max() <= 1.32 and 0.68 <= kd.min() <= 0.75
                           and 1.45 <= kd.max() <= 1.52 and 0.17 <= frac <= 0.23,
                           f"kp [{kp.min():.3f}, {kp.max():.3f}], kd [{kd.min():.3f}, {kd.max():.3f}], "
                           f"frac(mean kp >= 1.0) {frac:.3f}"))
    else:
      checks.append(_check(f"2. {label}: randomization snapshot", False, f"missing {snap}"))
    log = d.with_name(d.name + ".log")
    text = log.read_text(errors="ignore") if log.exists() else ""
    checks.append(_check(f"3. {label}: training exited 0, no NaN", "[launch] exit code 0" in text
                         and not re.search(r":\s*nan\b", text, re.I), str(log)))
    if iters >= 40:
      last = text.rsplit("Learning iteration", 1)[-1]
      ep = re.search(r"Mean episode length:\s*([0-9.]+)", last)
      err = re.search(r"Metrics/twist/error_vel_xy:\s*([0-9.]+)", last)
      ep_v, err_v = (float(ep.group(1)) if ep else None), (float(err.group(1)) if err else None)
      checks.append(_check(f"3. {label}: final episode length >= 450 and err_xy <= 0.30",
                           ep_v is not None and err_v is not None and ep_v >= 450 and err_v <= 0.30,
                           f"ep length {ep_v}, err_xy {err_v}"))
    else:
      checks.append(_check(f"3. {label}: ep length / err_xy", True, f"not judged ({iters} iterations < 40)"))
  audit = bumpy_dir / "physics_overflow_audit.json"
  checks.append(_check("3. bumpy arm: physics overflow 0", audit.exists() and json.loads(audit.read_text()).get("zero_overflow"),
                       str(audit)))
  ex = MJLAB_ROOT / f"report/part3/export/{bumpy_eval}.json" if bumpy_eval else None
  if ex is not None and ex.exists():
    rep = json.loads(ex.read_text())
    pol = json.loads(Path(rep["artifacts"]["policy_json"]).read_text())
    checks.append(_check("4. export PASS with policy.json kp 5.5 / kd 0.25", rep["verdict"] == "PASS"
                         and pol["kp"] == NOMINAL_KP and pol["kd"] == NOMINAL_KD, f"{rep['verdict']}, kp {pol['kp']}, kd {pol['kd']}"))
  else:
    checks.append(_check("4. export of the bumpy smoke checkpoint", False, f"missing {ex}"))
  nom = load_nompd(bumpy_eval, 1234) if bumpy_eval else None
  checks.append(_check("5. nompd ran on the smoke checkpoint (pipeline check, not a result)", nom is not None, f"falls {nom}"))
  return dict(verdict="PASS" if all(c["ok"] for c in checks) else "FAIL", checks=checks)


def deploy_hold(base: str, n1: str, seed2: int, branch: str, ctrl_eval: str | None) -> dict:
  """kp-coverage decision HOLD rule on two-seed means; the WIDEN control check uses the MATCHED control."""
  def mean2(a, b):
    return None if None in (a, b) else (a + b) / 2

  fd = mean2(load_falls(n1, "flat_dr"), load_falls(f"{n1}-seed{seed2}", "flat_dr"))
  nm = mean2(load_nompd(n1, 1234), load_nompd(n1, seed2))
  nm0 = mean2(load_nompd(base, 1234), load_nompd(base, seed2))
  if None in (fd, nm, nm0):
    return dict(verdict="INCOMPLETE", lines=[f"missing inputs: flat_dr {fd}, nompd {nm}, nompd(P) {nm0}"])
  lines = [f"(a) nompd - flat_dr = {nm:.1f} - {fd:.1f} = {nm - fd:+.1f} (hold if > {KP_DELTA})",
           f"(b) nompd(selected) - nompd(P) = {nm:.1f} - {nm0:.1f} = {nm - nm0:+.1f} (hold if > {NOISE})"]
  hold = (nm - fd > KP_DELTA) or (nm - nm0 > NOISE)
  if branch == "wide" and ctrl_eval:
    c, p0 = load_falls(ctrl_eval, "flat_dr"), load_falls(base, "flat_dr")
    if None not in (c, p0):
      lines.append(f"(WIDEN) matched control flat_dr - P flat_dr = {c} - {p0} = {c - p0:+d} (hold if > {NOISE})")
      hold = hold or (c - p0 > NOISE)
  lines.append(f"branch {branch}: " + ("HOLD -> " + ("launch the pre-authorized WIDEN pair" if branch == "lab4"
                                                      else "no more training; re-decision with the human") if hold else "clear"))
  return dict(verdict="HOLD" if hold else "CLEAR", branch=branch, flat_dr_mean=fd, nompd_mean=nm, nompd_P_mean=nm0, lines=lines)


def main() -> int:
  global DRY
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--checkpoint", type=Path, required=True)
  p.add_argument("--reference-dir", type=Path, required=True)
  p.add_argument("--name", required=True)
  p.add_argument("--run-dir", type=Path, help="Training run dir of the checkpoint (default: its parent)")
  p.add_argument("--wandb-run", help="entity/project/id of the checkpoint's run (default: parsed from its launch log)")
  p.add_argument("--stages", nargs="+", choices=STAGES, default=list(STAGES))
  p.add_argument("--smoke-finetune", action="store_true", help=f"One {SMOKE_LEG}-iteration leg per arm instead of the budget")
  p.add_argument("--finetune-iterations", type=int, default=FULL_ITERATIONS, help="Planned budget (multiple of 500)")
  p.add_argument("--upload-wandb", action="store_true",
                 help="Upload each export that PASSES to its W&B run (deploy.sh fetches it) and verify the read-back")
  p.add_argument("--pd-ranges", choices=("auto", "lab4", "wide"), default="auto",
                 help="Training PD-gain DR branch; auto = from the kpcheck stage (decision 2026-11-18-w4-kp-coverage)")
  p.add_argument("--smoke-iterations", default="50,50", help="WIDEN smoke iterations: BUMPY,CONTROL (each <= 50)")
  p.add_argument("--no-escalate", action="store_true", help="Do not launch the WIDEN pair after a KEEP-branch HOLD")
  p.add_argument("--dry-run", action="store_true", help="Print every command and decision step; launch nothing")
  p.add_argument("--logs", type=Path, default=RUNS / "p3-logs")
  args = p.parse_args()
  DRY = args.dry_run
  if args.finetune_iterations % LEG or not LEG <= args.finetune_iterations <= MAX_ITERATIONS:
    p.error(f"--finetune-iterations must be a multiple of {LEG} in [{LEG}, {MAX_ITERATIONS}]")
  ckpt, ref, name = args.checkpoint.resolve(), args.reference_dir, args.name
  run_dir = (args.run_dir or ckpt.parent).resolve()
  wandb_parent = args.wandb_run or wandb_run_path(run_dir)
  logs = args.logs / name
  status: dict[str, object] = {}
  common = ["--reference-dir", ref]
  P = model_iter(ckpt)
  leg_iters = SMOKE_LEG if args.smoke_finetune else LEG
  planned_legs = 1 if args.smoke_finetune else args.finetune_iterations // LEG
  suffix = "-smoke" if args.smoke_finetune else ""
  kp_path = EVAL_ROOT / f"{name}_kp_coverage.json"
  say(f"GPU cap {gpu_slot.cap()} (from {gpu_slot.CONCURRENCY_FILE}); shared lock {gpu_slot.LAUNCH_LOCK}"
      + ("; DRY RUN" if DRY else ""))

  def branch() -> str | None:
    if args.pd_ranges != "auto":
      return args.pd_ranges
    return json.loads(kp_path.read_text())["branch"] if kp_path.exists() else None

  def arms():
    b = branch() or "lab4"
    tag = "-kpwide" if b == "wide" else ""
    return f"{name}-bumpyft{tag}{suffix}", f"{name}-flatctrl{tag}{suffix}", tag

  def leg_dir(arm: str, k: int) -> Path:
    return RUNS / f"rp-{arm}-leg{k}"

  def leg_final(arm: str, k: int) -> Path | None:
    d = leg_dir(arm, k)
    if DRY and latest_model(d) is None:
      return d / f"model_{P + k * leg_iters - 1}.pt"  # where the leg's last checkpoint will be
    return latest_model(d) if run_ok(d) else None

  def legs_done(arm: str) -> int:
    k = 0
    while leg_final(arm, k + 1) is not None and (not DRY or (leg_dir(arm, k + 1)).exists()):
      k += 1
    return k

  def point(arm: str, k: int) -> tuple[str, Path] | None:
    f = leg_final(arm, k)
    return (eval_name(f"{arm}-leg{k}", f), f) if f is not None else None

  def export(c, n, rd, wb):
    if (MJLAB_ROOT / f"report/part3/export/{n}.json").exists():
      return "skip"
    cmd = [*PY, "-m", "mjlab.scripts.part3_export", "--checkpoint", c, *common, "--name", n, "--run-dir", rd]
    if wb:
      cmd += ["--wandb-run", wb] + (["--upload-wandb"] if args.upload_wandb else [])
    return run(cmd, logs / f"export-{n}.log")

  def evaluate(c, n, light=False, seed=None):
    base = [*PY, "-m", "mjlab.scripts.part3_eval", "run", "--checkpoint", c, *common, "--name", n, "--skip-existing"]
    if seed is not None:
      base += ["--seed", str(seed)]
    if light:
      return run(base + ["--modes", "suite", "seam", "--conditions", "flat_dr", "bumpy_dr"], logs / f"eval-{n}.log")
    rc = run(base + ["--modes", "suite", "seam"], logs / f"eval-{n}.log")
    rc |= run(base + ["--modes", "push", "--conditions", "flat_dr", "bumpy_dr"], logs / f"eval-{n}.log")
    return rc

  def nompd(c, n, seed=1234):
    if load_nompd(n, seed) is not None:
      return 0
    return run([*PY, "-m", "mjlab.scripts.part3_eval", "ablate", "--checkpoint", c, *common, "--name", n,
                "--events", "pd_gains", "--seed", str(seed)], logs / f"nompd-{n}.log")

  def finetune_cmd(arm: str, start: Path, flat: bool, iters: int, smoke: bool, run_name: str | None = None):
    cmd = [*PY, "scripts/opt_lab_2/part3_finetune.py", "--checkpoint", start, *common, "--name", run_name or arm,
           "--quiet", "--iterations", str(iters), "--pd-ranges", branch() or "lab4"]
    cmd += ["--smoke"] if smoke else []
    cmd += ["--flat-control"] if flat else []
    return cmd

  def eval_seed2(n, c):
    """Seed-4321 light eval + nompd of one point, on demand; returns its point_metrics."""
    evaluate(c, f"{n}-seed{SEED2}", light=True, seed=SEED2)
    nompd(c, n, SEED2)
    return point_metrics(n, SEED2)

  for stage in args.stages:
    t0 = time.time()
    ft, ctrl, btag = arms()  # the kp branch may have just been decided
    sel_path = EVAL_ROOT / f"{name}{btag}_selection.json"
    if stage == "export":
      status[stage] = export(ckpt, name, run_dir, wandb_parent)
    elif stage == "eval":
      status[stage] = evaluate(ckpt, name)
    elif stage == "video":
      rc = 0
      for cond in ("flat_dr", "bumpy_dr"):
        out = RUNS / "p3-videos" / name
        if (out / f"{name}-{cond}_seam_sweep.mp4").exists():
          continue
        rc |= run([*PY, "-m", "mjlab.scripts.part3_seam", "video", "--checkpoint", ckpt, *common,
                   "--name", f"{name}-{cond}", "--condition", cond, "--out-dir", out], logs / "video.log")
      status[stage] = rc
    elif stage == "kpcheck":
      if args.pd_ranges != "auto":
        status[stage] = f"skipped (--pd-ranges {args.pd_ranges})"
        continue
      rc = 0
      if load_falls(name, "flat_dr") is None:
        rc |= run([*PY, "-m", "mjlab.scripts.part3_eval", "run", "--checkpoint", ckpt, *common, "--name", name,
                   "--conditions", "flat_dr", "--modes", "suite", "--skip-existing"], logs / "kpcheck.log")
      n2 = f"{name}-seed{SEED2}"
      rc |= run([*PY, "-m", "mjlab.scripts.part3_eval", "run", "--checkpoint", ckpt, *common, "--name", n2,
                 "--conditions", "flat_dr", "--modes", "suite", "--seed", str(SEED2), "--skip-existing"], logs / "kpcheck.log")
      rc |= nompd(ckpt, name, 1234)
      rc |= nompd(ckpt, name, SEED2)
      fd = [load_falls(name, "flat_dr"), load_falls(n2, "flat_dr")]
      nm = [load_nompd(name, 1234), load_nompd(name, SEED2)]
      if DRY:
        status[stage] = "dry run (branch decided from these four suites)"
        continue
      if rc or None in fd + nm:
        status[stage] = f"incomplete (rc {rc}, flat_dr {fd}, nompd {nm})"
        break
      deltas = [a - b for a, b in zip(nm, fd, strict=True)]
      mean = sum(deltas) / 2
      res = dict(decision="report/decisions/2026-11-18-w4-kp-coverage.md", checkpoint=str(ckpt),
                 flat_dr_falls=dict(zip(("1234", str(SEED2)), fd, strict=True)),
                 nompd_falls=dict(zip(("1234", str(SEED2)), nm, strict=True)),
                 delta_per_seed=deltas, mean_delta=mean, threshold=KP_DELTA,
                 branch="wide" if mean > KP_DELTA else "lab4",
                 training_pd_ranges=WIDE if mean > KP_DELTA else "Lab 4 (kp x[0.6, 1.1], kd x[0.8, 1.5])")
      kp_path.write_text(json.dumps(res, indent=2, default=str) + "\n")
      md = (f"# kp-coverage step 0: {name}\n\nflat_dr falls {fd}, nominal-PD falls {nm} (seeds 1234, {SEED2}); "
            f"delta {deltas}, mean {mean:.1f} vs threshold {KP_DELTA} -> **{res['branch'].upper()}** "
            f"(training PD ranges: {res['training_pd_ranges']}). Evaluation stays on the Lab 4 ranges.\n")
      kp_path.with_suffix(".md").write_text(md)
      say(md)
      status[stage] = 0
    elif stage == "smoke-wide":
      if branch() != "wide" and not DRY:
        status[stage] = "skipped (KEEP branch)"
        continue
      it_b, it_c = (int(x) for x in args.smoke_iterations.split(","))
      if not (0 < it_b <= SMOKE_LEG and 0 < it_c <= SMOKE_LEG):
        status[stage] = "bad --smoke-iterations"
        break
      sb, sc = f"{name}-bumpyft-kpwide-smoke", f"{name}-flatctrl-kpwide-smoke"
      jobs = [(finetune_cmd(sb, ckpt, False, it_b, True), logs / f"train-{sb}.log"),
              (finetune_cmd(sc, ckpt, True, it_c, True), logs / f"train-{sc}.log")]
      jobs = [j for j, d in zip(jobs, (RUNS / f"rp-{sb}", RUNS / f"rp-{sc}"), strict=True) if not run_ok(d)]
      rc = run_parallel(jobs) if jobs else 0
      fb = latest_model(RUNS / f"rp-{sb}")
      if fb is not None:
        rc |= nompd(fb, eval_name(sb, fb))
        e = export(fb, eval_name(sb, fb), RUNS / f"rp-{sb}", None)
        rc |= 0 if e == "skip" else e
      if DRY:
        status[stage] = "dry run"
        continue
      res = smoke_checks(RUNS / f"rp-{sb}", RUNS / f"rp-{sc}", eval_name(sb, fb) if fb else None, it_b, it_c)
      out = EVAL_ROOT / f"{name}_kpwide_smoke.json"
      out.write_text(json.dumps(res, indent=2, default=str) + "\n")
      lines = [f"# WIDEN smoke checks: {name} -- **{res['verdict']}**", "",
               "Decision [2026-11-18-w4-kp-coverage](../../decisions/2026-11-18-w4-kp-coverage.md), smoke items 1-5.", "",
               "| check | ok | detail |", "|---|---|---|"]
      lines += [f"| {c['name']} | {'yes' if c['ok'] else '**NO**'} | {c['detail']} |" for c in res["checks"]]
      out.with_suffix(".md").write_text("\n".join(lines) + "\n")
      say("\n".join(lines))
      status[stage] = 0 if (rc == 0 and res["verdict"] == "PASS") else f"FAIL (rc {rc})"
      if status[stage] != 0:
        say(f"stage {stage}: {status[stage]} -- stopping before the full WIDEN run")
        break
    elif stage == "train":
      base = point_metrics(name)
      gates_md = EVAL_ROOT / f"{name}{btag}_gates.md"
      glines = [f"# Training gates: {name}{btag} (legs of {leg_iters} iterations, both arms together)", "",
                f"+0 = `{ckpt}`: flat_dr {base['flat_dr']}, bumpy_dr {base['bumpy_dr']}, nompd {base['nompd']}", ""]
      k, target_legs, rc, stopped = 0, planned_legs, 0, None
      while k < target_legs:
        k += 1
        jobs = []
        for arm, flat in ((ft, False), (ctrl, True)):
          if leg_final(arm, k) is not None and not DRY:
            continue  # leg already trained
          start = ckpt if k == 1 else leg_final(arm, k - 1)
          jobs.append((finetune_cmd(arm, start, flat, leg_iters, args.smoke_finetune, f"{arm}-leg{k}"),
                       logs / f"train-{arm}-leg{k}.log"))
        rc |= run_parallel(jobs) if jobs else 0
        pts = {arm: point(arm, k) for arm in (ft, ctrl)}
        if None in pts.values():
          stopped = f"leg {k} produced no checkpoint for {[a for a, v in pts.items() if v is None]}"
          break
        last_leg = k >= planned_legs
        for arm in (ft, ctrl):  # identical evaluation of both arms: matched at every leg
          n, c = pts[arm]
          rc |= evaluate(c, n, light=not last_leg)
          rc |= nompd(c, n)
        nb = pts[ft][0]
        invalid = []
        audit = leg_dir(ft, k) / "physics_overflow_audit.json"
        if audit.exists() and not json.loads(audit.read_text()).get("zero_overflow", False):
          invalid.append(f"training physics overflow in {audit}")
        for arm in (ft, ctrl):
          log = leg_dir(arm, k).with_name(leg_dir(arm, k).name + ".log")
          if log.exists() and re.search(r":\s*nan\b", log.read_text(errors="ignore"), re.I):
            invalid.append(f"NaN in {log}")
          if not DRY and not overflow_ok(pts[arm][0]):
            invalid.append(f"eval physics overflow in {pts[arm][0]}")
        mb = point_metrics(nb)
        mc = point_metrics(pts[ctrl][0])
        g = gate_after_leg(k, base, mb, invalid)
        glines.append(f"- leg {k} (+{k * LEG if not args.smoke_finetune else leg_iters}): bumpy arm {mb}, "
                      f"control {mc} -> **{g['action']}** {g['reasons']}")
        if g["action"] == "stop" and not DRY:
          stopped = "; ".join(g["reasons"])
          break
        if last_leg and k == target_legs and k >= 2 and not args.smoke_finetune:
          due, why = extension_due(base, point_metrics(point(ft, k - 1)[0]), mb, k * LEG)
          glines.append(f"  - extension rule (both arms or neither): {why} -> {'EXTEND +500' if due else 'no'}")
          if due:
            target_legs += 1
      glines.append("")
      glines.append(f"Stopped: {stopped}" if stopped else f"Finished {k} legs ({k * leg_iters} iterations per arm).")
      if not DRY:
        gates_md.write_text("\n".join(glines) + "\n")
      say("\n".join(glines))
      status[stage] = f"STOPPED: {stopped}" if stopped else rc
      if stopped:
        break
    elif stage == "select":
      n_legs = legs_done(ft)
      keys = [f"+{k * leg_iters}" for k in range(1, n_legs + 1)]
      if not keys:
        status[stage] = "no fine-tune legs"
        continue
      pts = {f"+{k * leg_iters}": {arm: point(arm, k) for arm in (ft, ctrl)} for k in range(1, n_legs + 1)}
      ev1 = {key: point_metrics(pts[key][ft][0]) for key in keys}
      res = matched_selection(keys, ev1, lambda key, pts=pts, ft=ft: eval_seed2(*pts[key][ft]))
      key = res["selected"]
      sel, mctrl = pts[key][ft], pts[key][ctrl]
      out = dict(res, selected_key=key, selected=str(sel[1]), selected_eval=sel[0],
                 matched_control=str(mctrl[1]) if mctrl else None, matched_control_eval=mctrl[0] if mctrl else None,
                 base=name, base_metrics=point_metrics(name),
                 points={k2: {arm: dict(eval=v[0], ckpt=str(v[1]), **point_metrics(v[0])) for arm, v in pv.items() if v}
                         for k2, pv in pts.items()})
      if not DRY:
        sel_path.write_text(json.dumps(out, indent=2, default=str) + "\n")
      lines = [f"# Part 3 checkpoint selection: {name}{btag}", "",
               "Matched selection per the [budget decision](../../decisions/2026-11-18-w4-finetune-budget.md) as amended by "
               "its [devil's-advocate review](../../decisions/2026-11-18-w4-finetune-budget/devils-advocate.md): a fallback "
               f"must replicate on eval seeds 1234 and {SEED2}; the control is compared at the same iteration.", "",
               f"Selected: **{key}** `{sel[1]}`; matched control `{mctrl[1] if mctrl else None}`.", ""]
      lines += [f"- {x}" for x in res["notes"]]
      lines += ["", "| point | arm | flat_dr | bumpy_dr | nompd | flat_dr seam |", "|---|---|---|---|---|---|"]
      b0 = point_metrics(name)
      lines.append(f"| +0 | parent | {b0['flat_dr']} | {b0['bumpy_dr']} | {b0['nompd']} | {b0['seam_flat']} |")
      for k2, pv in pts.items():
        for arm, v in pv.items():
          if v:
            mm = point_metrics(v[0])
            lines.append(f"| {k2} | {'bumpy' if arm == ft else 'flat control'} | {mm['flat_dr']} | {mm['bumpy_dr']} | "
                         f"{mm['nompd']} | {mm['seam_flat']} |")
      if not DRY:
        sel_path.with_suffix(".md").write_text("\n".join(lines) + "\n")
      say("\n".join(lines))
      status[stage] = 0
    elif stage in ("confirm", "hold", "export-ft"):
      if not sel_path.exists():
        status[stage] = "no selection (run the select stage)" + (" [dry run]" if DRY else "")
        continue
      sel = json.loads(sel_path.read_text())
      m, n1 = Path(sel["selected"]), sel["selected_eval"]
      if stage == "confirm":
        rc = evaluate(m, f"{n1}-seed{SEED2}", light=True, seed=SEED2)
        rc |= nompd(m, n1, SEED2)
        for cond in ("flat_dr", "bumpy_dr"):  # seam close-call rule: pooled windows of both eval seeds
          metas = [RUNS / "p3-raw" / f"{x}-{cond}_seam_meta.json" for x in (n1, f"{n1}-seed{SEED2}")]
          if all(mm.exists() for mm in metas):
            rc |= run([*PY, "-m", "mjlab.scripts.part3_seam", "pool", *metas, "--name", f"{n1}-{cond}-pooled",
                       "--out-dir", EVAL_ROOT / n1], logs / "pool.log", gpu=False)
        status[stage] = rc
      elif stage == "hold":
        res = deploy_hold(name, n1, SEED2, branch() or "lab4", sel.get("matched_control_eval"))
        hp = EVAL_ROOT / f"{name}{btag}_deploy_hold.json"
        hp.write_text(json.dumps(res, indent=2, default=str) + "\n")
        md = (f"# Deployment hold check: {name}{btag}\n\nDecision [2026-11-18-w4-kp-coverage](../../decisions/2026-11-18-w4-kp-coverage.md). "
              f"Selected `{m}`; matched control `{sel.get('matched_control')}`. Verdict: **{res['verdict']}**.\n\n"
              + "\n".join(f"- {r}" for r in res["lines"]) + "\n")
        hp.with_suffix(".md").write_text(md)
        say(md)
        status[stage] = 0 if res["verdict"] in ("CLEAR", "INCOMPLETE") else res["verdict"]
        if res["verdict"] == "HOLD" and (branch() or "lab4") == "lab4" and not args.no_escalate:
          say("HOLD in the KEEP branch: launching the pre-authorized WIDEN pair")
          cmd = [*PY, str(Path(__file__).resolve()), "--checkpoint", str(ckpt), "--reference-dir", str(ref), "--name", name,
                 "--pd-ranges", "wide", "--smoke-iterations", args.smoke_iterations,
                 "--finetune-iterations", str(args.finetune_iterations),
                 "--stages", "smoke-wide", "train", "select", "confirm", "hold", "export-ft", "table"]
          cmd += ["--upload-wandb"] if args.upload_wandb else []
          status["escalation"] = run(cmd, logs / "escalation-wide.log", gpu=False)
        elif res["verdict"] == "HOLD":
          say("HOLD in the WIDEN branch: stop; re-decision with the human")
      else:
        status[stage] = export(m, n1, m.parent, wandb_run_path(m.parent))
    elif stage == "table":
      dirs = [EVAL_ROOT / name]
      for k in range(1, legs_done(ft) + 1):
        for arm in (ft, ctrl):
          pt = point(arm, k)
          if pt:
            dirs.append(EVAL_ROOT / pt[0])
      dirs += sorted(EVAL_ROOT.glob(f"{ft}-leg*-it*-seed*"))
      dirs = [d for d in dirs if d.exists()]
      status[stage] = run([*PY, "-m", "mjlab.scripts.part3_eval", "table", *dirs, "--out",
                           EVAL_ROOT / f"{name}{btag}_summary.md"], logs / "table.log", gpu=False) if dirs else "no evals"
    say(f"stage {stage}: {status[stage]} ({time.time() - t0:.0f}s)")
  print(json.dumps(status, indent=1, default=str))
  return 0 if all(v in (0, "skip") or str(v).startswith(("skipped", "dry run")) for v in status.values()) else 1


if __name__ == "__main__":
  sys.exit(main())
