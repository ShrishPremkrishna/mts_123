"""Part 2 fold diagnostics beyond fold_summary.json (W3, round 0 and every round).

For a capture (trace.npz) and one or more folded reference dirs, reports per direction:

* cadence: dominant joint-trajectory frequency (FFT of the hip/knee channels in steady
  windows) and foot touchdown rate per foot, against the reference clock
  (frequency_mult * 4/3 Hz);
* fold residual: RMS over steady samples of q(t) - ref(phase(t) + shift), for the
  folded table and for the parent table (same shift), plus R^2 = 1 - resid var / var.
  A fold that does not explain the captured motion better than the parent is broken;
* amplitude retention: per-channel range of the folded table / the 5-95 % range of the
  captured samples (smearing an off-clock gait over the clock shrinks amplitude);
* kp * max|dq| per 20 ms control step at the playback cadence (course 3 Nm feasibility
  proxy, kp = 5.5; same formula as part1_gait_check.py), peak joint speed, and the
  minimum margin to the class driver limits, for both tables (fwd and back).

  uv run --no-sync python scripts/opt_lab_2/part2_foldcheck.py CAPTURE_DIR PARENT_REF NEW_REF [NEW_REF ...] \
      --lag 2 --out report/part2/foldcheck/<name>.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

KP = 5.5
CONTROL_DT = 0.02
BASE_HZ = 1.0 / 0.75
REPO = Path(__file__).resolve().parents[2]
DRIVER_LIMITS = REPO.parent / "foundation/configs/class-driver-joint-limits.json"
DIRS = {"reach_fwd": 1.0, "reach_back": -1.0}


def load_ref(d: Path):
  d = Path(d)
  man = json.loads((d / "reference_manifest.json").read_text())
  z = np.load(d / "captured_reach_gaits.npz")
  return {k: np.asarray(z[k], dtype=np.float64) for k in DIRS}, man


def table_checks(table: np.ndarray, hz: float, names):
  n = len(table)
  dt_sample = 1.0 / hz / n
  grid = np.arange(n * 10) / (n * 10)
  xp = np.arange(n + 1) / n
  fp = np.vstack([table, table[:1]])
  dense = np.stack([np.interp(grid, xp, fp[:, j]) for j in range(table.shape[1])], axis=1)
  shift = int(round(CONTROL_DT / dt_sample * 10))
  dq = np.abs(np.roll(dense, -shift, axis=0) - dense).max(axis=0)
  vel = np.abs(np.diff(fp, axis=0)).max(axis=0) / dt_sample
  lim = json.loads(DRIVER_LIMITS.read_text())["joint_limits"]
  lo = np.array([lim[k][0] for k in names]); hi = np.array([lim[k][1] for k in names])
  margin = np.minimum(table.min(0) - lo, hi - table.max(0))
  return dict(kp_dq_nm=float(KP * dq.max()), kp_dq_worst=names[int(dq.argmax())],
              peak_speed_rad_s=float(vel.max()), limit_margin_min_rad=float(margin.min()),
              limit_worst=names[int(margin.argmin())], wrap_step_rad=float(np.abs(table[0] - table[-1]).max()))


def interp_table(table: np.ndarray, phase: np.ndarray):
  n = len(table)
  x = (phase % 1.0) * n
  i0 = np.floor(x).astype(int) % n
  f = (x - np.floor(x))[..., None]
  return table[i0] * (1 - f) + table[(i0 + 1) % n] * f


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("capture", type=Path)
  p.add_argument("parent_ref", type=Path)
  p.add_argument("new_refs", type=Path, nargs="+")
  p.add_argument("--lag", type=float, default=2.0, help="control steps (fold shift)")
  p.add_argument("--out", type=Path)
  a = p.parse_args()
  tr = np.load(a.capture / "trace.npz" if a.capture.is_dir() else a.capture)
  q, ph, valid, cmd, con = tr["joint_pos"], tr["phase"], tr["valid"], tr["command"], tr["foot_contact"]
  names = [str(x) for x in tr["joint_names"]]
  parent, pman = load_ref(a.parent_ref)
  mult = float(pman["frequency_mult"])
  hz = BASE_HZ * mult
  T, E, J = q.shape
  res: dict = dict(capture=str(a.capture), parent_ref=str(a.parent_ref), clock_hz=hz, lag_steps=a.lag, directions={})
  for key, sign in DIRS.items():
    seg = (np.sign(cmd[..., 0]) == sign) & valid
    d: dict = {}
    # Cadence: longest valid runs per env; FFT on hip (_1) and knee (_3) channels.
    spec_acc, nwin = None, 0
    L = 256
    freqs = np.fft.rfftfreq(L, CONTROL_DT)
    td_rates = []
    for e in range(E):
      v = seg[:, e]
      idx = np.flatnonzero(v)
      if len(idx) < L:
        continue
      # contiguous runs
      breaks = np.flatnonzero(np.diff(idx) != 1)
      starts = np.r_[idx[0], idx[breaks + 1]]; ends = np.r_[idx[breaks], idx[-1]]
      for s, t in zip(starts, ends):
        if t - s + 1 < L:
          continue
        x = q[s:s + L, e, :] - q[s:s + L, e, :].mean(0)
        sp = (np.abs(np.fft.rfft(x * np.hanning(L)[:, None], axis=0)) ** 2).sum(1)
        spec_acc = sp if spec_acc is None else spec_acc + sp
        nwin += 1
        c = con[s:t + 1, e, :]
        tds = (c[1:] & ~c[:-1]).sum(0)
        td_rates.append(tds / ((t - s) * CONTROL_DT))
    if spec_acc is not None:
      band = freqs > 0.5
      f_dom = float(freqs[band][spec_acc[band].argmax()])
      clock_bin = np.abs(freqs - hz).argmin()
      dom_bin = np.flatnonzero(band)[spec_acc[band].argmax()]
      d["joint_dominant_hz"] = f_dom
      d["spectral_power_ratio_clock_over_dominant"] = float(spec_acc[clock_bin] / spec_acc[dom_bin])
      d["fft_windows"] = nwin
      d["fft_resolution_hz"] = float(freqs[1])
    if td_rates:
      tdr = np.array(td_rates)
      d["touchdown_hz_per_foot_median"] = np.median(tdr, 0).round(3).tolist()
    # Residuals.
    m = seg
    qs = q[m]  # (N, J)
    # The fold builds ref(p) from q at clock p + shift (shift = lag/cycle_steps), so the
    # predicted q at clock c is ref(c - shift).
    cycle_steps = 1.0 / hz / CONTROL_DT
    phase_q = ph[m] - a.lag / cycle_steps
    var = qs.var(0)
    lo5, hi95 = np.percentile(qs, 5, 0), np.percentile(qs, 95, 0)
    d["captured_samples"] = int(m.sum())
    d["captured_range_5_95"] = dict(zip(names, (hi95 - lo5).round(3).tolist()))
    for label, table in [("parent", parent[key])] + [(Path(r).name, load_ref(r)[0][key]) for r in a.new_refs]:
      pred = interp_table(table, phase_q)
      r = qs - pred
      rms = np.sqrt((r ** 2).mean(0))
      move = var > 1e-3
      r2 = 1 - (r.var(0)[move] / var[move])
      ent = dict(rms_resid_rad_mean=float(rms.mean()), rms_resid_rad_moving=float(rms[move].mean()),
                 r2_mean_moving=float(r2.mean()), r2_min_moving=float(r2.min()),
                 amplitude_retention_moving=float(np.mean(np.ptp(table, 0)[move] / (hi95 - lo5)[move])),
                 **table_checks(table, hz, names))
      d[label] = ent
    res["directions"][key] = d
  txt = json.dumps(res, indent=2)
  print(txt)
  if a.out:
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(txt + "\n")


if __name__ == "__main__":
  main()
