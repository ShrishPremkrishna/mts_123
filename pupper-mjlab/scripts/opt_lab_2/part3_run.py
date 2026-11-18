#!/usr/bin/env python3
"""Opt Lab 2 Part 3 runbook: one command from a final MixedGaits checkpoint to Part 3 sim evidence.

Stages (run in order, each skipped when its output already exists; simulation only):

  export    part3_export on the checkpoint: policy.json + golden fixture, round trip
            (torch / JSON / run auto-export / ONNX / W&B copy), limit audit, deploy validator,
            host C++ gait-reference test           -> report/part3/export/<name>.{json,md}
  eval      part3_eval: suite + seam on flat/Bumpy x Lab 4 DR on/off, push on DR-on
                                                    -> report/part3/eval/<name>/
  video     seam-sweep transition videos (flat_dr, bumpy_dr)  -> ~/runs/opt-lab-2/p3-videos/<name>/
  kpcheck   decision 2026-11-18-w4-kp-coverage step 0: flat_dr vs nominal-PD ("nompd") falls on eval
            seeds 1234 + 4321; mean delta > 15 -> WIDEN (train kp x[0.6,1.3], kd x[0.7,1.5]), else KEEP Lab 4
                                                    -> report/part3/eval/<name>_kp_coverage.{md,json}
  smoke-wide  WIDEN branch only: <= 50-iteration smoke of both arms + the decision's smoke checks;
            a failed check stops the runbook   -> report/part3/eval/<name>_kpwide_smoke.{md,json}
  finetune  part3_finetune: Bumpy warm start, 1500 iterations, PD-gain DR per the kp branch
            (decision 2026-11-18-w4-finetune-budget; --smoke-finetune = 50 iterations)
  control   the matched control arm: the same warm start/weights/seed/budget on Flat
  eval-ft   fine-tune at +500 and +1000 (flat_dr, bumpy_dr: suite + seam) and at the end (full);
            the control at the end (full)       -> report/part3/eval/<name>-bumpyft-it<N>/, <name>-flatctrl-it<N>/
  select    apply the decision's pre-registered guards -> report/part3/eval/<name>_selection.{md,json}
  confirm   the selected checkpoint on a second eval seed (4321; flat_dr, bumpy_dr, nompd), then the seam
            verdict on both seeds' pooled windows (the seam decision's close-call rule)
  hold      the kp decision's deployment HOLD rule -> report/part3/eval/<name><branch>_deploy_hold.{md,json};
            a HOLD in the KEEP branch launches the pre-authorized WIDEN pair (unless --no-escalate)
  export-ft part3_export on the selected checkpoint (+ W&B check; --upload-wandb to publish it)
  table     summary table over all of the above    -> report/part3/eval/<name>_summary.md

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
STAGES = ("export", "eval", "video", "kpcheck", "smoke-wide", "finetune", "control", "eval-ft", "select", "confirm",
          "hold", "export-ft", "table")
NOISE = 10  # falls; budget-decision guard band
KP_DELTA = 15  # falls; kp-coverage decision threshold
WIDE = dict(kp_range=(0.6, 1.3), kd_range=(0.7, 1.5))
NOMINAL_KP, NOMINAL_KD = 5.5, 0.25
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


def milestone(run_dir: Path, target: int) -> Path | None:
  models = list(run_dir.glob("model_*.pt"))
  if not models:
    return None
  best = min(models, key=lambda m: abs(model_iter(m) - target))
  return best if abs(model_iter(best) - target) <= 25 else None


def eval_name(prefix: str, ckpt: Path) -> str:
  return f"{prefix}-it{model_iter(ckpt)}"


def load_eval(n: str) -> dict:
  d = MJLAB_ROOT / "report/part3/eval" / n
  out = {}
  for cond in ("flat_dr", "bumpy_dr", "flat_nodr", "bumpy_nodr"):
    suite, seam = d / f"{n}-{cond}.json", d / f"{n}-{cond}_seam.json"
    row = {}
    if suite.exists():
      m = json.loads(suite.read_text())
      row["falls"] = m["metrics"]["falls_total"]
      ov = (m.get("part3_condition") or {}).get("overflow_audit")
      row["overflow_ok"] = True if ov is None else ov["zero_overflow"]
    if seam.exists():
      row["seam"] = json.loads(seam.read_text())["metrics"]["verdict"]["verdict"]
    if row:
      out[cond] = row
  nom = load_nompd(n, 1234)
  if nom is not None:
    out["nompd"] = dict(falls=nom)
  return out


def load_nompd(n: str, seed: int) -> int | None:
  tag = "" if seed == 1234 else f"-seed{seed}"
  f = MJLAB_ROOT / "report/part3/eval" / n / "ablate" / f"{n}-flat-nominal_pd_gains{tag}.json"
  return json.loads(f.read_text())["metrics"]["falls_total"] if f.exists() else None


def load_falls(n: str, cond: str) -> int | None:
  f = MJLAB_ROOT / "report/part3/eval" / n / f"{n}-{cond}.json"
  return json.loads(f.read_text())["metrics"]["falls_total"] if f.exists() else None


def select(base_name: str, ft_points: dict, ft_dir: Path, noise: int = 10) -> dict:
  """Guards of decision 2026-11-18-w4-finetune-budget (fixed before launch)."""
  base = load_eval(base_name)
  ev = {k: load_eval(n) for k, (n, _) in ft_points.items()}
  final_key = max(ft_points, key=lambda k: model_iter(ft_points[k][1]))
  notes, flags = [], []
  audit = ft_dir / "physics_overflow_audit.json"
  train_ok = (not audit.exists()) or json.loads(audit.read_text()).get("zero_overflow", False)
  evals_ok = all(r.get("overflow_ok", True) for e in ev.values() for r in e.values())
  if not (train_ok and evals_ok):
    return dict(selected=None, verdict="INVALID", notes=["nonzero physics overflow (training or eval)"], evals=ev)

  def f(key, cond):
    return ev.get(key, {}).get(cond, {}).get("falls")

  b = {c: base.get(c, {}).get("falls") for c in ("flat_dr", "bumpy_dr")}
  if "+500" in ev and None not in (f("+500", "flat_dr"), b["flat_dr"], f("+500", "bumpy_dr"), b["bumpy_dr"]):
    if f("+500", "flat_dr") >= 2 * max(b["flat_dr"], 1) and f("+500", "bumpy_dr") >= b["bumpy_dr"]:
      flags.append("G2 stop-early condition met at +500 (flat_dr falls >= 2x +0 and bumpy_dr no better): re-decide")
  selected = final_key
  if "+1000" in ev and final_key != "+1000":
    worse = [c for c in ("flat_dr", "bumpy_dr")
             if None not in (f(final_key, c), f("+1000", c)) and f(final_key, c) - f("+1000", c) > noise]
    seam_flip = ev["+1000"].get("flat_dr", {}).get("seam") == "PASS" and ev[final_key].get("flat_dr", {}).get("seam") == "FAIL"
    nom_worse = None not in (f(final_key, "nompd"), f("+1000", "nompd")) and f(final_key, "nompd") - f("+1000", "nompd") > noise
    if nom_worse:
      worse.append("nompd (kp-coverage decision)")
    if worse or seam_flip:
      selected = "+1000"
      notes.append(f"fallback to +1000: {final_key} worse by > {noise} falls on {worse}" + (" ; flat_dr seam PASS->FAIL" if seam_flip else "")
                   + " (report as a SELECTION; confirm on a second eval seed)")
    if None not in (f("+1000", "bumpy_dr"), f(final_key, "bumpy_dr"), f(final_key, "flat_dr"), b["flat_dr"]):
      if f("+1000", "bumpy_dr") - f(final_key, "bumpy_dr") > noise and abs(f(final_key, "flat_dr") - b["flat_dr"]) <= noise:
        flags.append("extension rule met: bumpy_dr still dropping > 10 falls from +1000 to the end with flat_dr within 10 of +0 "
                     "-> continue +500 (cap +3000) from the final checkpoint")
  if selected == final_key:
    notes.append(f"ship {final_key} (no guard fired)")
  return dict(selected=str(ft_points[selected][1]), selected_key=selected, verdict="OK", notes=notes, flags=flags,
              base=base_name, base_evals=base, evals=ev)


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


def deploy_hold(base: str, n1: str, n2: str, seed2: int, branch: str, ctrl_eval: str | None) -> dict:
  """kp-coverage decision HOLD rule on two-seed means (seeds 1234 + seed2)."""
  def mean2(a, b):
    return None if None in (a, b) else (a + b) / 2

  fd = mean2(load_falls(n1, "flat_dr"), load_falls(n2, "flat_dr"))
  nm = mean2(load_nompd(n1, 1234), load_nompd(n1, seed2))
  nm0 = mean2(load_nompd(base, 1234), load_nompd(base, seed2))
  lines, hold = [], False
  if None in (fd, nm, nm0):
    return dict(verdict="INCOMPLETE", lines=[f"missing inputs: flat_dr {fd}, nompd {nm}, nompd(P) {nm0}"])
  lines.append(f"(a) nompd - flat_dr = {nm:.1f} - {fd:.1f} = {nm - fd:+.1f} (hold if > {KP_DELTA})")
  lines.append(f"(b) nompd(selected) - nompd(P) = {nm:.1f} - {nm0:.1f} = {nm - nm0:+.1f} (hold if > {NOISE})")
  hold = (nm - fd > KP_DELTA) or (nm - nm0 > NOISE)
  if branch == "wide" and ctrl_eval:
    c, p0 = load_falls(ctrl_eval, "flat_dr"), load_falls(base, "flat_dr")
    if None not in (c, p0):
      lines.append(f"(WIDEN) control flat_dr - P flat_dr = {c} - {p0} = {c - p0:+d} (hold if > {NOISE})")
      hold = hold or (c - p0 > NOISE)
  lines.append(f"branch {branch}: " + ("HOLD -> " + ("launch the pre-authorized WIDEN pair" if branch == "lab4"
                                                      else "no more training; re-decision with the human") if hold else "clear"))
  return dict(verdict="HOLD" if hold else "CLEAR", branch=branch, flat_dr_mean=fd, nompd_mean=nm, nompd_P_mean=nm0, lines=lines)


def main() -> int:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--checkpoint", type=Path, required=True)
  p.add_argument("--reference-dir", type=Path, required=True)
  p.add_argument("--name", required=True)
  p.add_argument("--run-dir", type=Path, help="Training run dir of the checkpoint (default: its parent)")
  p.add_argument("--wandb-run", help="entity/project/id of the checkpoint's run (default: parsed from its launch log)")
  p.add_argument("--stages", nargs="+", choices=STAGES, default=list(STAGES))
  p.add_argument("--smoke-finetune", action="store_true", help="50-iteration smoke fine-tune instead of the full budget")
  p.add_argument("--finetune-iterations", type=int)
  p.add_argument("--no-control", action="store_true", help="Skip the flat control arm (state the confound in the report)")
  p.add_argument("--upload-wandb", action="store_true",
                 help="Upload each export that PASSES to its W&B run (deploy.sh fetches it) and verify the read-back")
  p.add_argument("--confirm-seed", type=int, default=4321)
  p.add_argument("--pd-ranges", choices=("auto", "lab4", "wide"), default="auto",
                 help="Training PD-gain DR branch; auto = from the kpcheck stage (decision 2026-11-18-w4-kp-coverage)")
  p.add_argument("--smoke-iterations", default="50,50", help="WIDEN smoke iterations: BUMPY,CONTROL (each <= 50)")
  p.add_argument("--no-escalate", action="store_true", help="Do not launch the WIDEN pair after a KEEP-branch HOLD")
  p.add_argument("--max-gpu-procs", type=int, default=4)
  p.add_argument("--logs", type=Path, default=RUNS / "p3-logs")
  args = p.parse_args()
  ckpt, ref, name = args.checkpoint.resolve(), args.reference_dir, args.name
  run_dir = (args.run_dir or ckpt.parent).resolve()
  wandb_parent = args.wandb_run or wandb_run_path(run_dir)
  logs = args.logs / name
  status: dict[str, object] = {}
  common = ["--reference-dir", ref]
  P = model_iter(ckpt)

  def export(c, n, rd, wb):
    if (MJLAB_ROOT / f"report/part3/export/{n}.json").exists():
      return "skip"
    cmd = [*PY, "-m", "mjlab.scripts.part3_export", "--checkpoint", c, *common, "--name", n, "--run-dir", rd]
    if wb:
      cmd += ["--wandb-run", wb] + (["--upload-wandb"] if args.upload_wandb else [])
    return run(cmd, args.max_gpu_procs, logs / f"export-{n}.log")

  def evaluate(c, n, light=False, seed=None):
    base = [*PY, "-m", "mjlab.scripts.part3_eval", "run", "--checkpoint", c, *common, "--name", n, "--skip-existing"]
    if seed is not None:
      base += ["--seed", str(seed)]
    if light:
      return run(base + ["--modes", "suite", "seam", "--conditions", "flat_dr", "bumpy_dr"],
                 args.max_gpu_procs, logs / f"eval-{n}.log")
    rc = run(base + ["--modes", "suite", "seam"], args.max_gpu_procs, logs / f"eval-{n}.log")
    rc |= run(base + ["--modes", "push", "--conditions", "flat_dr", "bumpy_dr"], args.max_gpu_procs, logs / f"eval-{n}.log")
    return rc

  def train(run_name, flat_control, smoke_iters=None):
    d = RUNS / f"rp-{run_name}"
    log = d.with_name(d.name + ".log")
    if latest_model(d) is not None and log.exists() and "exit code 0" in log.read_text(errors="ignore"):
      return "skip"
    cmd = [*PY, "scripts/opt_lab_2/part3_finetune.py", "--checkpoint", ckpt, *common, "--name", run_name, "--quiet"]
    cmd += ["--smoke"] if (args.smoke_finetune or smoke_iters) else []
    cmd += ["--flat-control"] if flat_control else []
    if smoke_iters:
      cmd += ["--iterations", str(smoke_iters)]
    elif args.finetune_iterations:
      cmd += ["--iterations", str(args.finetune_iterations)]
    cmd += ["--pd-ranges", branch() or "lab4"]
    return run(cmd, args.max_gpu_procs, logs / f"train-{run_name}.log")

  suffix = "-smoke" if args.smoke_finetune else ""
  kp_path = MJLAB_ROOT / f"report/part3/eval/{name}_kp_coverage.json"

  def branch() -> str | None:
    if args.pd_ranges != "auto":
      return args.pd_ranges
    return json.loads(kp_path.read_text())["branch"] if kp_path.exists() else None

  def names():
    b = branch() or "lab4"
    tag = "-kpwide" if b == "wide" else ""
    ft_n, ctrl_n = f"{name}-bumpyft{tag}{suffix}", f"{name}-flatctrl{tag}{suffix}"
    return ft_n, ctrl_n, RUNS / f"rp-{ft_n}", RUNS / f"rp-{ctrl_n}", tag

  ft_name, ctrl_name, ft_dir, ctrl_dir, btag = names()
  sel_path = MJLAB_ROOT / f"report/part3/eval/{name}{btag}_selection.json"

  def nompd(c, n, seed=1234):
    if load_nompd(n, seed) is not None:
      return 0
    return run([*PY, "-m", "mjlab.scripts.part3_eval", "ablate", "--checkpoint", c, *common, "--name", n,
                "--events", "pd_gains", "--seed", str(seed)], args.max_gpu_procs, logs / f"nompd-{n}.log")

  def ft_points():
    pts = {}
    final = latest_model(ft_dir)
    for k in (500, 1000):
      m = milestone(ft_dir, P + k)
      if m is not None and (final is None or model_iter(m) < model_iter(final)):
        pts[f"+{k}"] = (eval_name(ft_name, m), m)
    if final is not None:
      pts[f"+{model_iter(final) - P}"] = (eval_name(ft_name, final), final)
    return pts

  for stage in args.stages:
    t0 = time.time()
    ft_name, ctrl_name, ft_dir, ctrl_dir, btag = names()  # the kp branch may have just been decided
    sel_path = MJLAB_ROOT / f"report/part3/eval/{name}{btag}_selection.json"
    if stage == "kpcheck":
      if args.pd_ranges != "auto":
        status[stage] = f"skipped (--pd-ranges {args.pd_ranges})"
        continue
      rc = 0
      if load_falls(name, "flat_dr") is None:
        rc |= run([*PY, "-m", "mjlab.scripts.part3_eval", "run", "--checkpoint", ckpt, *common, "--name", name,
                   "--conditions", "flat_dr", "--modes", "suite", "--skip-existing"], args.max_gpu_procs, logs / "kpcheck.log")
      n2 = f"{name}-seed{args.confirm_seed}"
      rc |= run([*PY, "-m", "mjlab.scripts.part3_eval", "run", "--checkpoint", ckpt, *common, "--name", n2,
                 "--conditions", "flat_dr", "--modes", "suite", "--seed", str(args.confirm_seed), "--skip-existing"],
                args.max_gpu_procs, logs / "kpcheck.log")
      rc |= nompd(ckpt, name, 1234)
      rc |= nompd(ckpt, name, args.confirm_seed)
      fd = [load_falls(name, "flat_dr"), load_falls(n2, "flat_dr")]
      nm = [load_nompd(name, 1234), load_nompd(name, args.confirm_seed)]
      if rc or None in fd + nm:
        status[stage] = f"incomplete (rc {rc}, flat_dr {fd}, nompd {nm})"
        break
      deltas = [a - b for a, b in zip(nm, fd, strict=True)]
      mean = sum(deltas) / 2
      res = dict(decision="report/decisions/2026-11-18-w4-kp-coverage.md", checkpoint=str(ckpt),
                 flat_dr_falls=dict(zip(("1234", str(args.confirm_seed)), fd, strict=True)),
                 nompd_falls=dict(zip(("1234", str(args.confirm_seed)), nm, strict=True)),
                 delta_per_seed=deltas, mean_delta=mean, threshold=KP_DELTA,
                 branch="wide" if mean > KP_DELTA else "lab4",
                 training_pd_ranges=WIDE if mean > KP_DELTA else "Lab 4 (kp x[0.6, 1.1], kd x[0.8, 1.5])")
      kp_path.write_text(json.dumps(res, indent=2, default=str) + "\n")
      md = (f"# kp-coverage step 0: {name}\n\nflat_dr falls {fd}, nominal-PD falls {nm} (seeds 1234, {args.confirm_seed}); "
            f"delta {deltas}, mean {mean:.1f} vs threshold {KP_DELTA} -> **{res['branch'].upper()}** "
            f"(training PD ranges: {res['training_pd_ranges']}). Evaluation stays on the Lab 4 ranges.\n")
      kp_path.with_suffix(".md").write_text(md)
      print(md, flush=True)
      status[stage] = 0
    elif stage == "smoke-wide":
      if branch() != "wide":
        status[stage] = "skipped (KEEP branch)"
        continue
      it_b, it_c = (int(x) for x in args.smoke_iterations.split(","))
      if not (0 < it_b <= 50 and 0 < it_c <= 50):
        status[stage] = "bad --smoke-iterations"
        break
      sb, sc = f"{name}-bumpyft-kpwide-smoke", f"{name}-flatctrl-kpwide-smoke"
      rc = train(sb, False, it_b)
      rc = (0 if rc == "skip" else rc) | (0 if (r2 := train(sc, True, it_c)) == "skip" else r2)
      fb = latest_model(RUNS / f"rp-{sb}")
      if fb is not None:
        rc |= nompd(fb, eval_name(sb, fb))
        e = export(fb, eval_name(sb, fb), RUNS / f"rp-{sb}", None)
        rc |= 0 if e == "skip" else e
      res = smoke_checks(RUNS / f"rp-{sb}", RUNS / f"rp-{sc}", eval_name(sb, fb) if fb else None, it_b, it_c)
      out = MJLAB_ROOT / f"report/part3/eval/{name}_kpwide_smoke.json"
      out.write_text(json.dumps(res, indent=2, default=str) + "\n")
      lines = [f"# WIDEN smoke checks: {name} -- **{res['verdict']}**", "",
               "Decision [2026-11-18-w4-kp-coverage](../../decisions/2026-11-18-w4-kp-coverage.md), smoke items 1-5.", "",
               "| check | ok | detail |", "|---|---|---|"]
      lines += [f"| {c['name']} | {'yes' if c['ok'] else '**NO**'} | {c['detail']} |" for c in res["checks"]]
      out.with_suffix(".md").write_text("\n".join(lines) + "\n")
      print("\n".join(lines), flush=True)
      status[stage] = 0 if (rc == 0 and res["verdict"] == "PASS") else f"FAIL (rc {rc})"
      if status[stage] != 0:
        print(f"[part3_run] stage {stage}: {status[stage]} -- stopping before the full WIDEN run", flush=True)
        break
    elif stage == "hold":
      if not sel_path.exists():
        status[stage] = "no selection"
        continue
      sel = json.loads(sel_path.read_text())
      if not sel.get("selected"):
        status[stage] = f"selection {sel.get('verdict')}"
        continue
      m = Path(sel["selected"])
      n1, n2 = eval_name(ft_name, m), eval_name(ft_name, m) + f"-seed{args.confirm_seed}"
      res = deploy_hold(name, n1, n2, args.confirm_seed, branch() or "lab4",
                        eval_name(ctrl_name, latest_model(ctrl_dir)) if latest_model(ctrl_dir) else None)
      hp = MJLAB_ROOT / f"report/part3/eval/{name}{btag}_deploy_hold.json"
      hp.write_text(json.dumps(res, indent=2, default=str) + "\n")
      md = (f"# Deployment hold check: {name}{btag}\n\nDecision [2026-11-18-w4-kp-coverage](../../decisions/2026-11-18-w4-kp-coverage.md). "
            f"Selected `{m}`. Verdict: **{res['verdict']}**.\n\n" + "\n".join(f"- {r}" for r in res["lines"]) + "\n")
      hp.with_suffix(".md").write_text(md)
      print(md, flush=True)
      status[stage] = 0 if res["verdict"] in ("CLEAR", "INCOMPLETE") else res["verdict"]
      if res["verdict"] == "HOLD" and (branch() or "lab4") == "lab4" and not args.no_escalate:
        print("[part3_run] HOLD in the KEEP branch: launching the pre-authorized WIDEN pair", flush=True)
        cmd = [*PY, str(Path(__file__).resolve()), "--checkpoint", str(ckpt), "--reference-dir", str(ref), "--name", name,
               "--pd-ranges", "wide", "--smoke-iterations", args.smoke_iterations, "--max-gpu-procs", str(args.max_gpu_procs),
               "--stages", "smoke-wide", "finetune", "control", "eval-ft", "select", "confirm", "hold", "export-ft", "table"]
        cmd += ["--upload-wandb"] if args.upload_wandb else []
        status["escalation"] = run(cmd, 99, logs / "escalation-wide.log", gpu=False)
      elif res["verdict"] == "HOLD":
        print("[part3_run] HOLD in the WIDEN branch: stop; re-decision with the human", flush=True)
    elif stage == "export":
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
                   "--name", f"{name}-{cond}", "--condition", cond, "--out-dir", out],
                  args.max_gpu_procs, logs / "video.log")
      status[stage] = rc
    elif stage == "finetune":
      status[stage] = train(ft_name, False)
    elif stage == "control":
      status[stage] = "skipped (--no-control)" if args.no_control else train(ctrl_name, True)
    elif stage == "eval-ft":
      pts = ft_points()
      if not pts:
        status[stage] = f"no checkpoint in {ft_dir}"
        continue
      final_key = max(pts, key=lambda k: model_iter(pts[k][1]))
      rc = 0
      for k, (n, m) in pts.items():
        rc |= evaluate(m, n, light=(k != final_key))
        rc |= nompd(m, n)  # kp-coverage decision: nominal-PD flat suite at every point, seed 1234
      cfinal = latest_model(ctrl_dir)
      if cfinal is not None and not args.no_control:
        rc |= evaluate(cfinal, eval_name(ctrl_name, cfinal))
        rc |= nompd(cfinal, eval_name(ctrl_name, cfinal))
      status[stage] = rc
    elif stage == "select":
      pts = ft_points()
      if not pts:
        status[stage] = "no fine-tune checkpoints"
        continue
      res = select(name, pts, ft_dir)
      sel_path.write_text(json.dumps(res, indent=2, default=str) + "\n")
      lines = [f"# Part 3 checkpoint selection: {name}", "",
               "Guards from [the budget decision](../../decisions/2026-11-18-w4-finetune-budget.md), applied mechanically.", "",
               f"Verdict: **{res['verdict']}**. Selected: `{res.get('selected')}` ({res.get('selected_key')}).", ""]
      lines += [f"- {x}" for x in res.get("notes", []) + res.get("flags", [])]
      lines += ["", "| point | " + " | ".join(f"{c} falls / seam" for c in ("flat_dr", "bumpy_dr", "flat_nodr", "bumpy_nodr")) + " |",
                "|---|---|---|---|---|"]
      rows = [("+0 (warm-start parent)", res.get("base_evals", {}))] + list(res["evals"].items())
      cfinal = latest_model(ctrl_dir)
      if cfinal is not None:
        rows.append((f"flat control +{model_iter(cfinal) - P} (matched)", load_eval(eval_name(ctrl_name, cfinal))))
      for k, e in rows:
        lines.append(f"| {k} | " + " | ".join(f"{e.get(c, {}).get('falls', '-')} / {e.get(c, {}).get('seam', '-')}"
                                              for c in ("flat_dr", "bumpy_dr", "flat_nodr", "bumpy_nodr")) + " |")
      sel_path.with_suffix(".md").write_text("\n".join(lines) + "\n")
      print("\n".join(lines), flush=True)
      status[stage] = 0 if res["verdict"] == "OK" else res["verdict"]
    elif stage in ("confirm", "export-ft"):
      if not sel_path.exists():
        status[stage] = "no selection (run the select stage)"
        continue
      sel = json.loads(sel_path.read_text())
      if not sel.get("selected"):
        status[stage] = f"selection {sel.get('verdict')}"
        continue
      m = Path(sel["selected"])
      if stage == "confirm":
        n1, n2 = eval_name(ft_name, m), eval_name(ft_name, m) + f"-seed{args.confirm_seed}"
        rc = evaluate(m, n2, light=True, seed=args.confirm_seed)
        rc |= nompd(m, n1, args.confirm_seed)  # kp-coverage decision: nompd on the selected checkpoint, seed 4321
        # Seam close-call rule: judge flat_dr / bumpy_dr on the pooled windows of both eval seeds.
        raw = RUNS / "p3-raw"
        for cond in ("flat_dr", "bumpy_dr"):
          metas = [raw / f"{x}-{cond}_seam_meta.json" for x in (n1, n2)]
          if all(mm.exists() for mm in metas):
            rc |= run([*PY, "-m", "mjlab.scripts.part3_seam", "pool", *metas, "--name", f"{n1}-{cond}-pooled",
                       "--out-dir", MJLAB_ROOT / "report/part3/eval" / n1], 99, logs / "pool.log", gpu=False)
        status[stage] = rc
      else:
        status[stage] = export(m, eval_name(ft_name, m), ft_dir, wandb_run_path(ft_dir))
    elif stage == "table":
      root = MJLAB_ROOT / "report/part3/eval"
      dirs = [root / name] + [root / n for n, _ in ft_points().values()]
      cfinal = latest_model(ctrl_dir)
      if cfinal is not None:
        dirs.append(root / eval_name(ctrl_name, cfinal))
      dirs += sorted(root.glob(f"{ft_name}-it*-seed*"))
      dirs = [d for d in dirs if d.exists()]
      status[stage] = run([*PY, "-m", "mjlab.scripts.part3_eval", "table", *dirs, "--out",
                           root / f"{name}{btag}_summary.md"], 99, logs / "table.log", gpu=False)
    print(f"[part3_run] stage {stage}: {status[stage]} ({time.time() - t0:.0f}s)", flush=True)
  print(json.dumps(status, indent=1, default=str))
  return 0 if all(v in (0, "skip") or str(v).startswith("skipped") for v in status.values()) else 1


if __name__ == "__main__":
  sys.exit(main())
