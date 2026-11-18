"""Training-log comparison of new Part 1 runs against the recovered old runs.

Reads new runs' tfevents (~/runs/opt-lab-2/rp-*/events*) and old runs' recovered per-iteration
CSVs (~/recovery/metrics/<run>.csv, `_step` = iteration). Reports the mean of a 50-iteration
window ending at each requested iteration for reward, error_vel_xy, error_vel_yaw, fell_over and
gait_tracking, and plots error_vel_xy curves. A resumed run (-r2...) is concatenated onto its parent.

  uv run --no-sync python scripts/opt_lab_2/part1_curves.py OUT_PREFIX \
      --new p1-gallop_c-s42 ... --old gallop_c-s42+gallop_c-s42-r2 ... --at 500 1000 2000 3000
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np

KEYS = {"reward": "Train/mean_reward", "err_xy": "Metrics/twist/error_vel_xy", "err_yaw": "Metrics/twist/error_vel_yaw",
        "fell": "Episode_Termination/fell_over", "gait_tracking": "Episode_Reward/gait_tracking"}
RUNS = Path.home() / "runs/opt-lab-2"
OLD = Path.home() / "recovery/metrics"


def load_new(name):
  from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
  series = {k: {} for k in KEYS}
  dirs = sorted(glob.glob(str(RUNS / f"rp-{name}"))) + sorted(glob.glob(str(RUNS / f"rp-{name}-r*")))
  for d in dirs:
    for f in sorted(glob.glob(f"{d}/events*")):
      acc = EventAccumulator(f, size_guidance={"scalars": 0}); acc.Reload()
      tags = acc.Tags()["scalars"]
      for k, tag in KEYS.items():
        if tag in tags:
          for ev in acc.Scalars(tag):
            series[k][ev.step] = ev.value  # later segments overwrite overlapping steps
  return {k: (np.array(sorted(v)), np.array([v[s] for s in sorted(v)])) for k, v in series.items()}


def load_old(spec):
  import csv
  series = {k: {} for k in KEYS}
  for part in spec.split("+"):
    with open(OLD / f"{part}.csv") as f:
      for row in csv.DictReader(f):
        step = int(float(row["_step"]))
        for k, tag in KEYS.items():
          v = row.get(tag, "")
          if v not in ("", "nan"):
            series[k][step] = float(v)
  return {k: (np.array(sorted(v)), np.array([v[s] for s in sorted(v)])) for k, v in series.items()}


def window(series, at, width=50):
  steps, vals = series
  if not len(steps):
    return None
  m = (steps > at - width - 1) & (steps <= at - 1)
  return float(vals[m].mean()) if m.sum() >= width // 2 else None


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("out", type=Path)
  p.add_argument("--new", nargs="*", default=[])
  p.add_argument("--old", nargs="*", default=[])
  p.add_argument("--at", nargs="*", type=int, default=[500, 1000, 2000, 3000])
  args = p.parse_args()
  data = {f"new:{n}": load_new(n) for n in args.new}
  data.update({f"old:{o}": load_old(o) for o in args.old})
  table = {}
  for name, series in data.items():
    table[name] = {"last_iter": int(series["reward"][0].max()) if len(series["reward"][0]) else None}
    for at in args.at:
      table[name][str(at)] = {k: window(series[k], at) for k in KEYS}
  args.out.parent.mkdir(parents=True, exist_ok=True)
  args.out.with_suffix(".json").write_text(json.dumps(table, indent=1) + "\n")
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  fig, axes = plt.subplots(1, 3, figsize=(14, 3.8))
  for name, series in data.items():
    for ax, k in zip(axes, ("err_xy", "reward", "fell")):
      s, v = series[k]
      if len(s):
        kern = np.ones(25) / 25
        sm = np.convolve(v, kern, mode="valid") if len(v) > 25 else v
        ax.plot(s[:len(sm)] + (len(s) - len(sm)), sm, label=name, ls="-" if name.startswith("new") else ":")
  for ax, k in zip(axes, ("error_vel_xy (training log)", "mean reward", "fell_over")):
    ax.set_title(k, fontsize=9); ax.set_xlabel("iteration")
  axes[0].legend(fontsize=7)
  fig.tight_layout(); fig.savefig(args.out.with_suffix(".png"), dpi=100)
  for name, row in table.items():
    print(name, row["last_iter"], {at: {k: (round(v, 3) if v is not None else None) for k, v in row[at].items()} for at in map(str, args.at) if at in row})


if __name__ == "__main__":
  main()
