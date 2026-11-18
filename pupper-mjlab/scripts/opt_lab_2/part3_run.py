#!/usr/bin/env python3
"""Opt Lab 2 Part 3 runbook: one command from a final MixedGaits checkpoint to Part 3 sim evidence.

Stages (run in order, each skipped when its output already exists; simulation only):

  export    part3_export on the checkpoint: policy.json + golden fixture, round trip
            (torch / JSON / run auto-export / ONNX / W&B copy), limit audit, deploy validator,
            host C++ gait-reference test           -> report/part3/export/<name>.{json,md}
  eval      part3_eval: suite + seam on flat/Bumpy x Lab 4 DR on/off, push on DR-on
                                                    -> report/part3/eval/<name>/
  video     seam-sweep transition videos (flat_dr, bumpy_dr)  -> ~/runs/opt-lab-2/p3-videos/<name>/
  finetune  part3_finetune: Bumpy + Lab 4 DR warm start, 1500 iterations
            (decision 2026-11-18-w4-finetune-budget; --smoke-finetune = 50 iterations)
  control   the matched control arm: the same warm start/weights/seed/budget on Flat
  eval-ft   fine-tune at +500 and +1000 (flat_dr, bumpy_dr: suite + seam) and at the end (full);
            the control at the end (full)       -> report/part3/eval/<name>-bumpyft-it<N>/, <name>-flatctrl-it<N>/
  select    apply the decision's pre-registered guards -> report/part3/eval/<name>_selection.{md,json}
  confirm   the selected checkpoint on a second eval seed (4321; flat_dr, bumpy_dr), then the seam
            verdict on both seeds' pooled windows (the seam decision's close-call rule)
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
STAGES = ("export", "eval", "video", "finetune", "control", "eval-ft", "select", "confirm", "export-ft", "table")
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
  return out


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
  sel_path = MJLAB_ROOT / f"report/part3/eval/{name}_selection.json"

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

  def train(run_name, flat_control):
    d = RUNS / f"rp-{run_name}"
    log = d.with_name(d.name + ".log")
    if latest_model(d) is not None and log.exists() and "exit code 0" in log.read_text(errors="ignore"):
      return "skip"
    cmd = [*PY, "scripts/opt_lab_2/part3_finetune.py", "--checkpoint", ckpt, *common, "--name", run_name, "--quiet"]
    cmd += ["--smoke"] if args.smoke_finetune else []
    cmd += ["--flat-control"] if flat_control else []
    cmd += ["--iterations", str(args.finetune_iterations)] if args.finetune_iterations else []
    return run(cmd, args.max_gpu_procs, logs / f"train-{run_name}.log")

  suffix = "-smoke" if args.smoke_finetune else ""
  ft_name, ctrl_name = f"{name}-bumpyft{suffix}", f"{name}-flatctrl{suffix}"
  ft_dir, ctrl_dir = RUNS / f"rp-{ft_name}", RUNS / f"rp-{ctrl_name}"

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
      cfinal = latest_model(ctrl_dir)
      if cfinal is not None and not args.no_control:
        rc |= evaluate(cfinal, eval_name(ctrl_name, cfinal))
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
                           root / f"{name}_summary.md"], 99, logs / "table.log", gpu=False)
    print(f"[part3_run] stage {stage}: {status[stage]} ({time.time() - t0:.0f}s)", flush=True)
  print(json.dumps(status, indent=1, default=str))
  return 0 if all(v in (0, "skip") or str(v).startswith("skipped") for v in status.values()) else 1


if __name__ == "__main__":
  sys.exit(main())
