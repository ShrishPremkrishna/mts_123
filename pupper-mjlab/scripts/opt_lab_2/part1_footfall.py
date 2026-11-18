"""Measure the footfall pattern a trained policy actually uses, from foot contacts.

Reads one or more `pupper_locomotion evaluate` traces (trace.npz) and, for every
commanded segment, uses only valid (post-warmup, non-failing) samples to report:
duty factor per foot, feet-down histogram, flight fraction, stride frequency,
touchdown phase lags relative to the left hind foot (circular means), the
cyclic footfall order and a coarse gait label:

  trot     diagonal pairs (FR+LB, FL+BR) land together and the pairs alternate
  pace     lateral pairs land together
  bound    front pair together, hind pair together, pairs alternate
  rotary gallop / transverse gallop  4-beat with the hind pair then the fore pair,
           rotary when the fore lead is on the same side as the second hind foot
  walk / other  anything else (4-beat with lags near quarters is called walk)

Contact order is the robot leg order FR, FL, BR, LB (checked by --check-order).

Usage: uv run --no-sync python scripts/opt_lab_2/part1_footfall.py OUT.json TRACE.npz [TRACE.npz ...]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

LEGS = ("FR", "FL", "BR", "LB")
FR, FL, BR, LB = range(4)


def _circ_mean(x):
  a = np.exp(2j * np.pi * np.asarray(x))
  m = a.mean()
  return float((np.angle(m) / (2 * np.pi)) % 1.0), float(abs(m))


def _close(a, b, tol=0.12):
  d = abs(((a - b) + 0.5) % 1.0 - 0.5)
  return d < tol


def classify(lag):
  """lag: touchdown phase of each leg relative to LB (LB = 0)."""
  fr, fl, br, lb = lag
  if _close(fr, 0) and _close(fl, br) and _close(fl, .5):
    return "trot"
  if _close(br, 0) and _close(fr, fl) and _close(fr, .5):
    return "bound"
  if _close(fl, 0) and _close(fr, br) and _close(fr, .5):
    return "pace"
  hind_pair = _close(br, 0, .3)
  fore_pair = _close(fr, fl, .3)
  if hind_pair and fore_pair and not _close(fr, 0, .2):
    # Lead fore = the fore foot that lands last. Rotary: second hind (BR, right)
    # then right fore first (FR before FL). Transverse: LB, BR, FL, FR.
    first_fore = "FR" if ((fr - br) % 1) < ((fl - br) % 1) else "FL"
    return "rotary gallop" if first_fore == "FR" else "transverse gallop"
  if all(_close(l, q, .08) for l, q in zip(sorted([fr, fl, br]), (.25, .5, .75))):
    return "walk (4-beat, even)"
  return "other"


def debounce(contact, k=1):
  """Remove contact flicker: fill air gaps and drop contact blips of <= k samples (k=1: 20 ms)."""
  c = contact.copy()
  for want in (True, False):  # first fill short gaps, then drop short blips
    for length in range(1, k + 1):
      for start in range(1, c.shape[0] - length):
        seg = c[start:start + length]
        mask = (c[start - 1] == want) & (c[start + length] == want) & np.all(seg != want, axis=0)
        c[start:start + length][:, mask] = want
  return c


def analyze_segment(contact, valid, dt, debounce_k=1):
  """contact: (T, E, 4) bool; valid: (T, E) bool."""
  out = {}
  if debounce_k:
    contact = debounce(contact, debounce_k)
  c = contact[valid]  # (S, 4)
  if len(c) < 50:
    return dict(valid_samples=int(len(c)))
  out["valid_samples"] = int(len(c))
  out["duty_factor"] = dict(zip(LEGS, c.mean(axis=0).round(3).tolist()))
  n = c.sum(axis=1)
  out["feet_down_hist"] = {int(k): round(float((n == k).mean()), 3) for k in range(5)}
  out["flight_fraction"] = round(float((n == 0).mean()), 3)
  out["min_feet_down_p01"] = int(np.percentile(n, 1))
  # Touchdown events per env, contiguous valid runs only.
  T, E, _ = contact.shape
  periods, lags = [], {leg: [] for leg in range(4)}
  for e in range(E):
    v = valid[:, e]
    if v.sum() < 50:
      continue
    ce = contact[:, e, :] & v[:, None]
    td = {leg: np.flatnonzero(ce[1:, leg] & ~ce[:-1, leg] & v[:-1]) + 1 for leg in range(4)}
    ref = td[LB]
    if len(ref) < 3:
      continue
    p = np.diff(ref)
    periods.extend(p.tolist())
    for k in range(len(ref) - 1):
      a, b = ref[k], ref[k + 1]
      for leg in range(4):
        inside = td[leg][(td[leg] >= a) & (td[leg] < b)]
        if len(inside) == 1:
          lags[leg].append((inside[0] - a) / (b - a))
  if not periods:
    out["note"] = "too few left-hind touchdowns (foot dragging or standing)"
    return out
  period = float(np.median(periods)) * dt
  out["stride_period_s"] = round(period, 4)
  out["stride_hz"] = round(1 / period, 3)
  lag, conc = [], []
  for leg in range(4):
    if lags[leg]:
      m, r = _circ_mean(lags[leg]); lag.append(m); conc.append(r)
    else:
      lag.append(float("nan")); conc.append(0.0)
  out["touchdown_lag_rel_LB"] = dict(zip(LEGS, np.round(lag, 3).tolist()))
  out["lag_concentration"] = dict(zip(LEGS, np.round(conc, 2).tolist()))
  if np.isfinite(lag).all():
    order = [LEGS[i] for i in np.argsort([l % 1 for l in lag])]
    out["footfall_order"] = order
    out["gait_guess"] = classify(lag)
    out["gait"] = out["gait_guess"] if min(conc) > 0.5 else "irregular (low lag concentration)"
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("out", type=Path)
  p.add_argument("traces", type=Path, nargs="+")
  args = p.parse_args()
  result = {}
  for path in args.traces:
    with np.load(path, allow_pickle=False) as f:
      tr = {k: f[k] for k in ("foot_contact", "valid", "segment", "command", "dt")}
    dt = float(tr["dt"])
    segs = []
    for s in np.unique(tr["segment"]):
      m = tr["segment"] == s
      cmd = tr["command"][m][0, 0].tolist()
      r = analyze_segment(tr["foot_contact"][m].astype(bool), tr["valid"][m].astype(bool), dt)
      r["command"] = cmd
      segs.append(r)
    result[str(path)] = segs
  args.out.parent.mkdir(parents=True, exist_ok=True)
  args.out.write_text(json.dumps(result, indent=1) + "\n")
  for path, segs in result.items():
    print(path)
    for r in segs:
      print(f"  cmd {r['command']}: gait={r.get('gait')} order={r.get('footfall_order')} "
            f"duty={r.get('duty_factor')} flight={r.get('flight_fraction')} hz={r.get('stride_hz')} lags={r.get('touchdown_lag_rel_LB')}")


if __name__ == "__main__":
  main()
