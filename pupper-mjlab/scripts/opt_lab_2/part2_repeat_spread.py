"""Eval repeat spread for Part 2 (W3): the same checkpoint evaluated twice (same eval seed; the GPU
rollout is not bit-reproducible), to set noise bands for the matched deltas, and the rep-averaged
+1500 row.

Pairs `report/part2/eval/w3/<name>.json` with `report/part2/eval/w3-rep/<name>-rep2.json` (+ `_push`).
Writes `report/part2/analysis/repeat_spread.{md,json}`:
  * per readout: |rep - orig| per checkpoint, its mean and max -> band = max |diff| over pairs
    (a single-eval delta between two arms carries ~sqrt(2) x the per-eval spread);
  * the +1500 arms averaged over both evals, as levels and as deltas vs the same-seed control.

  uv run --no-sync python scripts/opt_lab_2/part2_repeat_spread.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

MJ = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from part2_analysis import READOUTS, readouts  # noqa: E402

EV, REP, OUT = MJ / "report/part2/eval/w3", MJ / "report/part2/eval/w3-rep", MJ / "report/part2/analysis"
KEY = ["falls", "rev", "stop", "err±1.5", "F_runmax_med_BW", "F_p99_BW", "sat@1.5", "jitter", "drift+", "drift-",
       "splay+", "splay-", "tau2", "slip", "top+", "top-"]


def load(d, name):
  s, p = d / f"{name}.json", d / f"{name}_push.json"
  if not s.exists():
    return None
  return readouts(name, json.loads(s.read_text()), json.loads(p.read_text()) if p.exists() else None)


def main():
  OUT.mkdir(parents=True, exist_ok=True)
  push_keys = [c for c in READOUTS if c.startswith("R:")]
  keys = KEY + push_keys
  pairs = {}
  for f in sorted(REP.glob("*-rep2.json")):
    name = f.stem[:-5]
    a, b = load(EV, name), load(REP, f.stem)
    if a and b:
      pairs[name] = (a, b)
  spread = {}
  for k in keys:
    d = [abs(b[k] - a[k]) for a, b in pairs.values() if a.get(k) is not None and b.get(k) is not None]
    if d:
      spread[k] = dict(mean_abs_diff=float(np.mean(d)), max_abs_diff=float(np.max(d)), n=len(d))
  lines = ["# Part 2 eval repeat spread (simulation)\n",
           f"Same checkpoint, same eval seed, evaluated twice ({len(pairs)} checkpoints: {', '.join(pairs)}). "
           "GPU rollouts are not bit-reproducible, so this is the eval-to-eval noise of a single readout. "
           "A gap between two single evals carries about sqrt(2) x this spread; the band used in the notebook is "
           "max |diff| (conservative).\n",
           "| readout | mean \\|diff\\| | max \\|diff\\| | n |", "|---|---|---|---|"]
  for k, v in spread.items():
    lines.append(f"| {k} | {v['mean_abs_diff']:.4g} | {v['max_abs_diff']:.4g} | {v['n']} |")
  lines += ["", "## Per checkpoint (orig / rep2)", "",
            "| checkpoint | " + " | ".join(keys) + " |", "|---|" + "---|" * len(keys)]
  for n, (a, b) in pairs.items():
    lines.append(f"| {n} | " + " | ".join(f"{a.get(k):.4g} / {b.get(k):.4g}" if a.get(k) is not None and b.get(k) is not None
                                          else "-" for k in keys) + " |")
  # Rep-averaged +1500 row.
  avg = {n: {k: (a[k] + b[k]) / 2 for k in keys if a.get(k) is not None and b.get(k) is not None}
         for n, (a, b) in pairs.items() if n.startswith("p2-")}
  lines += ["", "## +1500 row averaged over the two evals: delta vs the same-seed ctrl-1500, two-seed mean", ""]
  arms = ["r3", "r2-cont-1500", "r1-cont-1500"]
  lines += ["| readout | " + " | ".join(f"{a} s42 / s43 (mean)" for a in arms) + " | ctrl-1500 s42 / s43 |",
            "|---|" + "---|" * (len(arms) + 1)]
  out_avg = {}
  for k in keys:
    cells = []
    for arm in arms:
      ds = []
      for s in (42, 43):
        c, x = avg.get(f"p2-s{s}-ctrl-1500", {}).get(k), avg.get(f"p2-s{s}-{arm}", {}).get(k)
        ds.append(None if c is None or x is None else x - c)
      out_avg.setdefault(arm, {})[k] = ds
      cells.append(" / ".join("-" if d is None else f"{d:+.4g}" for d in ds)
                   + (f" ({np.mean(ds):+.4g})" if None not in ds else ""))
    ctrl = " / ".join("-" if avg.get(f"p2-s{s}-ctrl-1500", {}).get(k) is None else f"{avg[f'p2-s{s}-ctrl-1500'][k]:.4g}"
                      for s in (42, 43))
    lines.append(f"| {k} | " + " | ".join(cells) + f" | {ctrl} |")
  (OUT / "repeat_spread.md").write_text("\n".join(lines) + "\n")
  (OUT / "repeat_spread.json").write_text(json.dumps(dict(spread=spread, pairs={n: dict(orig=a, rep2=b) for n, (a, b) in pairs.items()},
                                                          plus1500_rep_avg=avg, plus1500_rep_avg_deltas=out_avg), indent=1) + "\n")
  print("\n".join(lines))


if __name__ == "__main__":
  main()
