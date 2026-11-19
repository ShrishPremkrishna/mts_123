# Standing push sweep: 50 % shove velocity (Opt Lab 2 Part 3, simulation only)

2026-11-19. Fall rate vs lateral shove velocity while **standing still** (zero command), flat terrain with the
Lab 4 DR (`flat_dr`: kp x[0.6, 1.1], kd x[0.8, 1.5], friction [0.6, 1.4], plus COM / encoder-bias / inertia
events, standard physics flags), eval seed 1234, the same push schedule for every policy. Reference dir for
all three: `configs/opt-lab-2/p2/s42/r2-lag` (training reference check `ok` for all three).

Data: [`push_sweep.csv`](push_sweep.csv) (pooled over both rounds). Plot: [`push_sweep.png`](push_sweep.png).

## Result

| policy | v50 (m/s) | 95 % interval (Wilson bands) | bracketing points (rate, falls/pushes) | logistic v50 [quasi-binomial 95 %] | dispersion φ |
|---|---|---|---|---|---|
| Bumpy +2000 (Part 3 sim candidate) | **0.652** | 0.621 – 0.670 | 0.65 → 0.496 (485/978); 0.70 → 0.584 (597/1022) | 0.633 [0.606, 0.656] | 30.0 |
| Flat control +2000 | **0.736** | 0.732 – 0.739 | 0.70 → 0.266 (272/1024); 0.75 → 0.593 (1101/1857) | 0.737 [0.734, 0.741] | 2.3 |
| Part 2 final (P) | **0.760** | 0.748 – 0.775 | 0.75 → 0.484 (906/1873); 0.80 → 0.568 (593/1044) | 0.766 [0.750, 0.783] | 25.0 |

- **v50** = linear interpolation of the fall rate between the two grid points that bracket 50 % (0.05 m/s
  apart, so every crossing is bracketed to ±0.025 m/s). **Interval** = where the 95 % Wilson upper band
  first reaches 0.5 (low end) and where the Wilson lower band first reaches 0.5 (high end), linear between
  points. This is the primary, model-free estimate.
- The logistic fit (binomial GLM in velocity, fitted on the points with rate in [0.02, 0.98] plus one neighbour
  each side, CI by profile likelihood) is secondary. Its deviance is far above its dof for Bumpy (φ = 30) and
  Part 2 (φ = 25): those curves are not logistic, so the plain profile CI is too narrow, and the table gives the
  φ-scaled (quasi-binomial) interval. Bumpy's curve has a shoulder (0.45 → 0.50 → 0.58 over 0.60–0.70) that
  pulls the logistic v50 below the interpolated crossing.
- **Ordering:** Bumpy +2000 falls at a ~0.09 m/s lower shove than flat control and ~0.11 m/s lower than
  Part 2 final; the intervals do not overlap. Flat control vs Part 2 final: 0.736 vs 0.760, which is separated
  by the Wilson intervals, but it is a small gap given the GPU non-determinism below. Flat control has the
  steepest curve: it holds to 0.6 m/s (1 %) and then fails over a narrower velocity range.
- Bumpy +2000 already falls 19 % (148/771) at 0.5 m/s, where the other two fall ≤ 0.3 %, and 34 % at 0.55.
  At the frozen ranked rung, 0.65 m/s, the rates are 0.496 / 0.089 / 0.118 (Bumpy / flat control / Part 2).
  This agrees with the HOLD(c) push gate in `report/decisions/2026-11-18-w4-p3-candidate.md`.
- All three reach 100 % well before 5 m/s: ≥ 99.4 % at 1.0 m/s, and 100 % from 2 m/s on. No policy fails to
  reach 50 %. The baseline at 0 m/s is 0 falls for all three (zero-magnitude push slots, ~865 per policy).

## Method

- **Harness, reused unchanged:** `part2_eval.run_push` (30 s rollouts; the first push ≥ 4 s, then one every
  3 s ± 0.25 s at a random per-env offset; the push is a heading-perpendicular world-frame root velocity
  change of ±v, random sign, written after the policy acts; a counted push fails if the env terminates within
  2 s; pushes < 2 s after a reset or a previous push are not counted). The Lab 4 DR condition comes from
  `part3_common.rollout_conditions(dr=True)` on `Mjlab-MixedGaits-Flat-Pupper-v3`, as `part3_eval run --modes
  push --conditions flat_dr` does.
- **Only three module-level overrides** (driver below): `PUSH_CONDITIONS = (("stand", 0.0),)`, so every env
  stands; `PUSH_MAGNITUDES` = the round's velocity set, so each push draws its magnitude uniformly from the
  set; and the same tuple as `compute_push_metrics`'s `magnitudes` default, which was bound at import.
- **Adaptive search, 2 rounds, 1024 envs each**, all three policies sequentially in one GPU process per round
  (launched through `gpu_slot_run.py`):
  - r1 (coarse): 0, 0.25, 0.5, 0.75, 1, 1.5, 2, 3, 4, 5 m/s, giving 753–871 counted pushes per point. All
    three crossings fell in (0.5, 0.75] (Part 2: 0.491 at 0.75, so (0.75, 1.0]).
  - r2 (refine): 0.55, 0.60, …, 0.90 m/s in 0.05 steps (union over policies, so the schedule stays shared),
    giving 953–1078 pushes per point. 0.75 m/s appears in both rounds and is pooled (1853–1873 pushes).
  - Stopping rule met after r2: every crossing is bracketed by points 0.05 apart (±0.025 m/s), with ≥ 953
    pushes on each bracketing point (≥ 200 required).
- **Same schedule:** within a round the velocity set, seed (1234) and env count are identical, so
  `make_push_schedule` gives every policy the same push times, signs and magnitudes (8197 scheduled pushes per
  run). Counted pushes differ slightly, because falls reset envs.
- **Checks:** the harness's applied-impulse self-check reads 0.88–0.94 × v of lateral velocity one control
  step after the push, for every policy and velocity. Ground contact acts during that step, the same as in the
  Part 2 pushes. Falls outside push windows: 0–1 per run. Code at git `1164e766`, with `src` clean.

## Caveats

- **Seeds:** single eval seed (1234), as instructed. The GPU rollout is not bit-reproducible
  (`PART2_TOOLING.md`). The 0.75 m/s point, measured in two rounds with different schedules, gave
  0.779 / 0.760 (Bumpy), 0.588 / 0.597 (flat control) and 0.491 / 0.478 (Part 2): run-to-run spread is about
  ±0.01–0.02 in rate, so ~±0.005–0.01 m/s in v50. The Wilson intervals cover sampling noise only.
- The fall rates pool both push signs (left and right) and the whole Lab 4 DR distribution. They also pool
  pushes landing at any time after the previous one (≥ 2.5 s apart), as in the Part 2 protocol.
- The push is a velocity impulse; read v as shove velocity, not as force. Simulation only; no hardware claim.

## Commands

Run from `pupper-mjlab/` (worktree of `markus/opt-lab-2` at `1164e76`), with `export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl`:

```bash
python3 scripts/opt_lab_2/gpu_slot_run.py -- uv run --no-sync python ~/runs/opt-lab-2/push_sweep/push_sweep_round.py \
    --round r1 --mags 0,0.25,0.5,0.75,1,1.5,2,3,4,5          # 1024 envs, seed 1234 (defaults)
python3 scripts/opt_lab_2/gpu_slot_run.py -- uv run --no-sync python ~/runs/opt-lab-2/push_sweep/push_sweep_round.py \
    --round r2 --mags 0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9
uv run --no-sync python ~/runs/opt-lab-2/push_sweep/push_sweep_analyze.py report/part3/push_sweep
```

Per-run JSONs (full `run_push` payloads with provenance) are in `~/runs/opt-lab-2/push_sweep/r{1,2}/` and
are not committed. Both scripts are reproduced verbatim below, because they live outside the repository.

<details><summary><code>push_sweep_round.py</code> (driver)</summary>

```python
"""One round of the standing push sweep: the Part 2 push test (part2_eval.run_push) under the
Part 3 flat_dr condition (Lab 4 DR, part3_common.rollout_conditions), with every env standing
(zero command) and the push magnitudes replaced by this round's velocity set.
All policies run sequentially in one process (one GPU job), same seed -> same push schedule.

  python push_sweep_round.py --round R --mags 0,0.25,... [--envs 1024] [--seed 1234]
"""
import argparse
import os
import sys
from pathlib import Path

REF = "configs/opt-lab-2/p2/s42/r2-lag"
RUNS = Path.home() / "runs" / "opt-lab-2"
POLICIES = {
  "bumpy_p2000": RUNS / "rp-p3-final-bumpyft-kpwide-leg4" / "model_6492.pt",
  "flatctrl_p2000": RUNS / "rp-p3-final-flatctrl-kpwide-leg4" / "model_6492.pt",
  "part2_final": RUNS / "rp-p2-s42-r2-cont-1500" / "model_4496.pt",
}
OUT = RUNS / "push_sweep"

if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("--round", required=True)
  ap.add_argument("--mags", required=True)
  ap.add_argument("--envs", type=int, default=1024)
  ap.add_argument("--seed", type=int, default=1234)
  ap.add_argument("--policies", default=",".join(POLICIES))
  cli = ap.parse_args()
  mags = tuple(float(m) for m in cli.mags.split(","))

  from mjlab.scripts import part2_eval as pe
  from mjlab.scripts.part3_common import CONDITIONS, rollout_conditions

  # Standing only, this round's velocities (both are read at call time by run_push,
  # push_commands, make_push_schedule and compute_push_metrics; the latter's
  # keyword default was bound at import, so rebind it too).
  pe.PUSH_CONDITIONS = (("stand", 0.0),)
  pe.PUSH_MAGNITUDES = mags
  pe.compute_push_metrics.__kwdefaults__["magnitudes"] = mags

  task, dr = CONDITIONS["flat_dr"]
  for name in cli.policies.split(","):
    p = argparse.ArgumentParser()
    pe._add_rollout_args(p)
    p.add_argument("--envs", type=int)
    out_dir = OUT / cli.round
    stem = f"{name}-{cli.round}"
    a = p.parse_args(["--checkpoint", str(POLICIES[name]), "--reference-dir", REF, "--name", stem,
                      "--envs", str(cli.envs), "--seed", str(cli.seed), "--task", task,
                      "--out-dir", str(out_dir)])
    if (out_dir / f"{stem}_push.json").exists():
      print(f"[push_sweep] skip existing {stem}", flush=True)
      continue
    pe._prepare(a, lambda m: sys.exit(m))
    a.dr, a.condition = dr, "flat_dr"
    with rollout_conditions(a, dr=dr, audit_path=out_dir / f"{stem}_push_overflow.json"):
      pe.run_push(a)
```
</details>

<details><summary><code>push_sweep_analyze.py</code> (CSV, intervals, fit, plot)</summary>

```python
"""Aggregate push-sweep rounds -> CSV, 50 % point per policy (bracket + logistic fit), PNG.

  python push_sweep_analyze.py OUT_DIR [--fit-window LO,HI]
Pools counts per (policy, velocity) over all rounds under ~/runs/opt-lab-2/push_sweep/r*/.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy import optimize, stats

SRC = Path.home() / "runs" / "opt-lab-2" / "push_sweep"
LABELS = {
  "bumpy_p2000": "Bumpy +2000 (Part 3 candidate)",
  "flatctrl_p2000": "Flat control +2000",
  "part2_final": "Part 2 final",
}


def wilson(f, n, z=1.96):
  p = f / n
  d = 1 + z * z / n
  c = (p + z * z / (2 * n)) / d
  h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
  return c - h, c + h


def load():
  counts = defaultdict(lambda: [0, 0])
  for f in sorted(SRC.glob("r*/*_push.json")):
    d = json.loads(f.read_text())
    pol = d["name"].rsplit("-", 1)[0]
    for m, c in d["metrics"]["by_condition"]["stand"].items():
      if m in ("all", "achieved_vx_m_s"):
        continue
      counts[(pol, round(float(m), 4))][0] += c["pushes"]
      counts[(pol, round(float(m), 4))][1] += c["falls"]
  return counts


def nll(theta, v, n, f):
  v50, s = theta
  z = (v - v50) / s
  return -(f * -np.logaddexp(0, -z) + (n - f) * -np.logaddexp(0, z)).sum()


def logistic_fit(v, n, f):
  """Binomial logistic in velocity; 95 % CI on v50 by profile likelihood, plain and
  quasi-binomial (threshold scaled by phi = deviance/dof when phi > 1)."""
  v, n, f = map(np.asarray, (v, n, f))
  r = optimize.minimize(nll, [np.interp(0.5, f / n, v) if np.all(np.diff(f / n) >= 0) else v.mean(), 0.05],
                        args=(v, n, f), method="Nelder-Mead", options=dict(xatol=1e-6, fatol=1e-8, maxiter=5000))
  v50, s = r.x
  best = r.fun
  dev = 2 * sum(nll([v50, s], np.array([vi]), np.array([ni]), np.array([fi]))
                - nll_sat(ni, fi) for vi, ni, fi in zip(v, n, f))
  phi = max(1.0, dev / max(len(v) - 2, 1))

  def ci(scale):
    thr = best + scale * stats.chi2.ppf(0.95, 1) / 2

    def prof(x):
      return optimize.minimize_scalar(lambda s_: nll([x, s_], v, n, f), bounds=(1e-4, 2), method="bounded").fun - thr

    lo = optimize.brentq(prof, v50 - 1.0, v50) if prof(v50 - 1.0) > 0 else None
    hi = optimize.brentq(prof, v50, v50 + 1.0) if prof(v50 + 1.0) > 0 else None
    return lo, hi

  return dict(v50=v50, scale=s, ci=ci(1.0), ci_quasi=ci(phi), phi=phi, deviance=dev, dof=len(v) - 2)


def band_interval(v, n, f):
  """Model-free 95 % interval: where the Wilson upper band first reaches 0.5 (lower end)
  and where the Wilson lower band first reaches 0.5 (upper end), linear between points."""
  lo_b, hi_b = wilson(f, n)

  def first_cross(y):
    for i in range(1, len(v)):
      if y[i - 1] < 0.5 <= y[i]:
        return v[i - 1] + (0.5 - y[i - 1]) * (v[i] - v[i - 1]) / (y[i] - y[i - 1])
    return None

  return first_cross(hi_b), first_cross(lo_b)


def nll_sat(n, f):
  p = f / n
  out = 0.0
  if f > 0:
    out -= f * np.log(p)
  if n - f > 0:
    out -= (n - f) * np.log(1 - p)
  return out


if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("out", type=Path)
  ap.add_argument("--fit-window", default="0.02,0.98", help="fit on points whose rate lies in [lo,hi] plus one neighbour each side")
  a = ap.parse_args()
  a.out.mkdir(parents=True, exist_ok=True)
  counts = load()
  pols = [p for p in LABELS if any(k[0] == p for k in counts)]
  rows = []
  with open(a.out / "push_sweep.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["policy", "shove_mps", "pushes", "falls", "fall_rate"])
    for p in pols:
      for v in sorted(k[1] for k in counts if k[0] == p):
        n, f = counts[(p, v)]
        w.writerow([p, f"{v:g}", n, f, f"{f / n:.4f}"])
        rows.append((p, v, n, f))
  lo_r, hi_r = map(float, a.fit_window.split(","))
  result = {}
  for p in pols:
    pts = sorted((v, n, f) for q, v, n, f in rows if q == p)
    v = np.array([x[0] for x in pts]); n = np.array([x[1] for x in pts]); f = np.array([x[2] for x in pts])
    r = f / n
    # bracket: highest v with rate < 0.5 and lowest v with rate >= 0.5 above it
    below = [i for i in range(len(v)) if r[i] < 0.5]
    above = [i for i in range(len(v)) if r[i] >= 0.5]
    br = None
    if below and above:
      i = max(below)
      j = min(k for k in above if v[k] > v[i]) if any(v[k] > v[i] for k in above) else None
      if j is not None:
        vi = v[i] + (0.5 - r[i]) * (v[j] - v[i]) / (r[j] - r[i])
        br = dict(lo=dict(v=v[i], n=int(n[i]), f=int(f[i]), rate=r[i], wilson=wilson(f[i], n[i])),
                  hi=dict(v=v[j], n=int(n[j]), f=int(f[j]), rate=r[j], wilson=wilson(f[j], n[j])),
                  interp=vi, nonmonotone=bool(any(r[k] >= 0.5 for k in range(len(v)) if v[k] < v[i])))
    inner = np.where((r >= lo_r) & (r <= hi_r))[0]
    fit = None
    if len(inner):
      sel = np.arange(max(inner.min() - 1, 0), min(inner.max() + 2, len(v)))
      if len(sel) >= 3:
        fit = logistic_fit(v[sel], n[sel], f[sel])
        fit["points"] = [float(x) for x in v[sel]]
    result[p] = dict(bracket=br, band_ci=band_interval(v, n, f), fit=fit)
  (a.out / "fit.json").write_text(json.dumps(result, indent=1, default=float) + "\n")
  print(json.dumps(result, indent=1, default=float))

  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw=dict(width_ratios=[1.25, 1]))
  colors = {"bumpy_p2000": "#2a6fdb", "flatctrl_p2000": "#e07b39", "part2_final": "#3a9e5c"}
  for ax, zoom in zip(axes, (False, True)):
    for p in pols:
      pts = sorted((v, n, f) for q, v, n, f in rows if q == p)
      v = np.array([x[0] for x in pts]); n = np.array([x[1] for x in pts]); f = np.array([x[2] for x in pts])
      lo, hi = wilson(f, n)
      c = colors[p]
      ax.plot(v, f / n, "-o", color=c, ms=3.5, lw=1.6, label=LABELS[p])
      ax.fill_between(v, lo, hi, color=c, alpha=0.15, lw=0)
      fit = result[p]["fit"]
      if zoom and fit:
        ax.axvline(fit["v50"], color=c, ls=":", lw=1)
    ax.axhline(0.5, color="0.4", ls="--", lw=1)
    ax.set_xlabel("lateral shove velocity (m/s)")
    ax.set_ylim(-0.02, 1.02)
    ax.grid(alpha=0.3)
  axes[0].set_ylabel("fall rate (fall within 2 s of push)")
  axes[0].set_xlim(0, 5)
  axes[0].set_title("Standing push sweep, flat + Lab 4 DR (95 % Wilson bands)")
  xs = [fv for p in pols if result[p]["fit"] for fv in result[p]["fit"]["points"]]
  if xs:
    axes[1].set_xlim(min(xs) - 0.05, max(xs) + 0.05)
  axes[1].set_title("Zoom on the 50 % crossing (dotted: logistic v50)")
  axes[0].legend(loc="lower right", fontsize=8)
  fig.tight_layout()
  fig.savefig(a.out / "push_sweep.png", dpi=150)
```
</details>
