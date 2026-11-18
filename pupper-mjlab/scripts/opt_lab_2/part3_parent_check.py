#!/usr/bin/env python3
"""Part 3 check on the Part 1 parent pick at nominal PD gains (simulation only).

Question (from the coordinator, 2026-11-18): does the nominal-PD finding reopen the Part 1 parent decision
(gallop_c over fast_trot, report/decisions/2026-11-18-p1-parent-gait.md)?

PRE-REGISTERED RULE (fixed and committed before any of these numbers were collected):
  For each arm, over its training seeds (s42, s43) and eval seeds (1234, 4321):
    lab4    = mean flat_dr suite falls (Lab 4 DR, play config)          [part3_eval run --conditions flat_dr]
    nompd   = mean flat suite falls with the PD-gain DR event at nominal [part3_eval ablate --events pd_gains]
    excess  = nompd - lab4
  fast_trot REOPENS the parent decision only if BOTH
    (a) excess(fast_trot) <= 15, and
    (b) nompd(fast_trot) <= nompd(gallop_c) - 20.
  Otherwise the parent pick stands. This script changes nothing else; a reopen goes to the coordinator.

gallop_c s42 is the Part 3 parent; its four numbers are the ones already measured with the same tools
(report/part3/eval/p3-parent{,-seed4321}/ and p3-parent/ablate/). Every other number is measured here,
one GPU process at a time, through gpu_slot (W3's launch lock + ~/recovery/CONCURRENCY).

  uv run --no-sync python scripts/opt_lab_2/part3_parent_check.py [--dry-run]
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gpu_slot  # noqa: E402

MJ = Path(__file__).resolve().parents[2]
RUNS = Path.home() / "runs" / "opt-lab-2"
OUT = MJ / "report" / "part3" / "parent_check"
EVAL = MJ / "report" / "part3" / "eval"
SEEDS = (1234, 4321)
EXCESS_MAX = 15
NOMPD_MARGIN = 20
ARMS = {
  "fast_trot": [("s42", RUNS / "rp-p1-fast_trot-s42/model_2999.pt", "configs/opt-lab-2/fast_trot"),
                ("s43", RUNS / "rp-p1-fast_trot-s43/model_2999.pt", "configs/opt-lab-2/fast_trot")],
  "gallop_c": [("s42", RUNS / "rp-p1-gallop_c-s42/model_2999.pt", "configs/opt-lab-2/gallop_c"),
               ("s43", RUNS / "rp-p1-gallop_c-s43/model_2999.pt", "configs/opt-lab-2/gallop_c")],
}
EXISTING = {("gallop_c", "s42"): "p3-parent"}  # already measured as the Part 3 parent


def falls(path: Path):
  return json.loads(path.read_text())["metrics"]["falls_total"] if path.exists() else None


def paths(arm, seed_tag, eval_seed):
  name = EXISTING.get((arm, seed_tag), f"p1check-{arm}-{seed_tag}")
  root = EVAL if (arm, seed_tag) in EXISTING else OUT
  sfx = "" if eval_seed == 1234 else f"-seed{eval_seed}"
  return (name, root, root / f"{name}{sfx}" / f"{name}{sfx}-flat_dr.json",
          root / name / "ablate" / f"{name}-flat-nominal_pd_gains{sfx}.json")


def run(cmd, dry):
  print("$ " + " ".join(map(str, cmd)), flush=True)
  if dry:
    return 0
  env = dict(os.environ, MUJOCO_GL="egl", PYOPENGL_PLATFORM="egl")
  log = OUT / "logs" / "parent_check.log"
  log.parent.mkdir(parents=True, exist_ok=True)
  with open(log, "a") as f:
    return gpu_slot.run(cmd, cwd=MJ, env=env, stdout=f, stderr=subprocess.STDOUT, log=print)


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--dry-run", action="store_true")
  a = ap.parse_args()
  OUT.mkdir(parents=True, exist_ok=True)
  rc = 0
  for arm, items in ARMS.items():
    for seed_tag, ckpt, ref in items:
      for es in SEEDS:
        name, root, fd, nm = paths(arm, seed_tag, es)
        sfx = "" if es == 1234 else f"-seed{es}"
        if fd.exists() is False:
          rc |= run([sys.executable, "-m", "mjlab.scripts.part3_eval", "run", "--checkpoint", ckpt, "--reference-dir", ref,
                     "--name", f"{name}{sfx}", "--out-root", root, "--conditions", "flat_dr", "--modes", "suite",
                     "--seed", es, "--skip-existing"], a.dry_run)
        if nm.exists() is False:
          rc |= run([sys.executable, "-m", "mjlab.scripts.part3_eval", "ablate", "--checkpoint", ckpt, "--reference-dir", ref,
                     "--name", name, "--out-root", root, "--events", "pd_gains", "--seed", es], a.dry_run)
  if a.dry_run:
    return 0
  rows, agg = [], {}
  for arm, items in ARMS.items():
    l4s, nms = [], []
    for seed_tag, _, _ in items:
      for es in SEEDS:
        name, _, fd, nm = paths(arm, seed_tag, es)
        l4, n = falls(fd), falls(nm)
        rows.append((arm, seed_tag, es, l4, n))
        l4s.append(l4)
        nms.append(n)
    if None in l4s + nms:
      print(f"incomplete: {arm} lab4 {l4s} nompd {nms}")
      return 1
    agg[arm] = dict(lab4=sum(l4s) / len(l4s), nompd=sum(nms) / len(nms))
    agg[arm]["excess"] = agg[arm]["nompd"] - agg[arm]["lab4"]
  ft, gc = agg["fast_trot"], agg["gallop_c"]
  cond_a = ft["excess"] <= EXCESS_MAX
  cond_b = ft["nompd"] <= gc["nompd"] - NOMPD_MARGIN
  verdict = "REOPEN" if (cond_a and cond_b) else "STANDS"
  res = dict(rule=f"reopen iff excess(fast_trot) <= {EXCESS_MAX} AND nompd(fast_trot) <= nompd(gallop_c) - {NOMPD_MARGIN}",
             rows=[dict(arm=r[0], train_seed=r[1], eval_seed=r[2], lab4_falls=r[3], nompd_falls=r[4]) for r in rows],
             aggregate=agg, condition_a=cond_a, condition_b=cond_b, verdict=verdict)
  (OUT / "result.json").write_text(json.dumps(res, indent=2) + "\n")
  lines = ["# Parent-pick check at nominal PD gains: result", "",
           f"Rule (pre-registered in `scripts/opt_lab_2/part3_parent_check.py` and `README.md` here): {res['rule']}.", "",
           "| arm | train seed | eval seed | Lab 4 DR falls (flat_dr) | nominal-PD falls |", "|---|---|---|---|---|"]
  lines += [f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} | {r[4]} |" for r in rows]
  lines += ["", "| arm | mean Lab 4 | mean nominal-PD | excess |", "|---|---|---|---|"]
  lines += [f"| {k} | {v['lab4']:.1f} | {v['nompd']:.1f} | {v['excess']:+.1f} |" for k, v in agg.items()]
  lines += ["", f"(a) excess(fast_trot) {ft['excess']:+.1f} <= {EXCESS_MAX}: **{cond_a}**; "
            f"(b) nompd(fast_trot) {ft['nompd']:.1f} <= nompd(gallop_c) {gc['nompd']:.1f} - {NOMPD_MARGIN}: **{cond_b}**.", "",
            f"**Verdict: the parent pick {verdict}.**" + (" (Goes to the coordinator; nothing else changed.)" if verdict == "REOPEN" else ""),
            "", "Simulation only; 384 robot-runs per suite; GPU rollouts are not bit-reproducible (gaps < 10-15 falls are ties)."]
  (OUT / "result.md").write_text("\n".join(lines) + "\n")
  print("\n".join(lines))
  return rc


if __name__ == "__main__":
  sys.exit(main())
