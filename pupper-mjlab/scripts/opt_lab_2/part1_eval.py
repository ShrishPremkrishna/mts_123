"""Part 1 evaluation driver: the stock `pupper_locomotion evaluate` under two fixed protocols.

  seq    the CLI's DEFAULT_SEQUENCE (16 segments: 0, .3, .49, .51, 1, 1.5, .51, .49, 0, -.3, -1.5,
         yaw ±1, vy ±.3, 0), no resets between segments, so transitions (and reversal falls) count.
  sweep  ±0.3, ±0.75, ±1.0, ±1.5 m/s, each from a fresh reset (--reset-each-command), for steady-state
         tracking and the contact-measured footfall pattern (scripts/opt_lab_2/part1_footfall.py).

Both use 64 envs, play-mode physics (startup randomization and observation noise kept), seed 1234,
and the physics flags of training. Every checkpoint is evaluated with the reference dir it trained on.

  uv run --no-sync python scripts/opt_lab_2/part1_eval.py ARM CHECKPOINT [--reference-dir DIR] [--out-root ~/runs/opt-lab-2/p1-evals]
  uv run --no-sync python scripts/opt_lab_2/part1_eval.py --table OUT_PREFIX ARM [ARM ...]
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path.home() / "runs/opt-lab-2/p1-evals"
MJLAB = Path(__file__).resolve().parents[2]
PHYS = ["--deployment-target-clamp", "--compose-com-randomization", "--locomotion-torso-contact",
        "--driver-limits", str(MJLAB.parent / "foundation/configs/class-driver-joint-limits.json")]
SWEEP = [[v, 0, 0] for v in (0.3, 0.75, 1.0, 1.5, -0.3, -0.75, -1.0, -1.5)]
EFFORT = 3.0


def run(arm, checkpoint, refdir, root):
  out = root / arm
  out.mkdir(parents=True, exist_ok=True)
  (out / "sweep_cmds.json").write_text(json.dumps(SWEEP))
  env = dict(os.environ, MUJOCO_GL="egl")
  base = [sys.executable, "-m", "mjlab.scripts.pupper_locomotion", "evaluate", "--checkpoint", str(checkpoint),
          "--num-envs", "64", "--seed", "1234", *PHYS]
  if refdir:
    base += ["--reference-dir", str(refdir)]
  jobs = {"seq": ["--seconds", "10", "--warmup", "2"],
          "sweep": ["--seconds", "8", "--warmup", "3", "--reset-each-command", "--commands", str(out / "sweep_cmds.json")]}
  for name, extra in jobs.items():
    if (out / name / "metrics.json").exists():
      continue
    with open(out / f"{name}.log", "w") as log:
      subprocess.run(base + ["--out", str(out / name)] + extra, cwd=MJLAB, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
  subprocess.run([sys.executable, str(MJLAB / "scripts/opt_lab_2/part1_footfall.py"), str(out / "footfall.json"),
                  str(out / "sweep" / "trace.npz")], cwd=MJLAB, check=True, stdout=subprocess.DEVNULL)
  extra_metrics(out)


def extra_metrics(out):
  """Per segment: 3 Nm saturation fraction and peak foot force in body weights (not in summarize_trace)."""
  res = {}
  for name in ("seq", "sweep"):
    with np.load(out / name / "trace.npz", allow_pickle=False) as f:
      tr = {k: f[k] for k in ("torque", "valid", "segment", "command", "foot_force_norm")}
    rows = []
    for s in np.unique(tr["segment"]):
      m = (tr["segment"] == s)[:, None] & tr["valid"].astype(bool)
      tq = np.abs(tr["torque"][m])
      ff = tr["foot_force_norm"][m]
      rows.append(dict(command=tr["command"][tr["segment"] == s][0, 0].tolist(),
                       torque_saturation_fraction=float((tq >= 0.99 * EFFORT).mean()) if tq.size else None,
                       foot_force_p99_n=float(np.nanpercentile(ff, 99)) if ff.size else None,
                       foot_force_max_n=float(np.nanmax(ff)) if ff.size else None))
    res[name] = rows
  (out / "extra_metrics.json").write_text(json.dumps(res, indent=1) + "\n")


def robot_weight_n():
  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import get_spec
  model = get_spec().compile()
  return float(model.body_subtreemass[1] * 9.81)


def table(out_prefix, arms, root):
  bw = robot_weight_n()
  rows = {}
  for arm in arms:
    d = root / arm
    seq = json.loads((d / "seq/metrics.json").read_text())["segments"]
    sweep = json.loads((d / "sweep/metrics.json").read_text())["segments"]
    extra = json.loads((d / "extra_metrics.json").read_text())
    ff = json.loads((d / "footfall.json").read_text())
    ff = next(iter(ff.values()))
    r = {}
    for seg, ex, foot in zip(sweep, extra["sweep"], ff):
      v = seg["command"][0]
      r[f"{v:+.2f}"] = dict(err_xy=seg.get("mean_xy_error_m_s"), err_yaw=seg.get("mean_abs_yaw_error_rad_s"),
                            vx=seg.get("mean_achieved_vx_m_s"), torque2=seg.get("mean_torque_squared_nm2"),
                            slip=seg.get("mean_contact_slip_m_s"), failures=seg["failures"],
                            drift_rad=seg.get("max_abs_heading_drift_rad"),
                            sat3nm=ex["torque_saturation_fraction"],
                            foot_force_p99_bw=(ex["foot_force_p99_n"] or np.nan) / bw,
                            foot_force_max_bw=(ex["foot_force_max_n"] or np.nan) / bw,
                            gait=foot.get("gait"), gait_guess=foot.get("gait_guess"), order=foot.get("footfall_order"),
                            duty=foot.get("duty_factor"), flight=foot.get("flight_fraction"), stride_hz=foot.get("stride_hz"),
                            feet_down=foot.get("feet_down_hist"), lags=foot.get("touchdown_lag_rel_LB"),
                            lag_concentration=foot.get("lag_concentration"))
    hs = [r["+1.50"], r["-1.50"]]
    r["hs_mean"] = {k: float(np.mean([h[k] for h in hs])) for k in ("err_xy", "err_yaw", "torque2", "slip", "sat3nm", "foot_force_p99_bw")}
    r["hs_mean"]["foot_force_max_bw"] = float(max(h["foot_force_max_bw"] for h in hs))
    r["seq_failures"] = {f"{s['command']}": s["failures"] for s in seq}
    r["seq_failures_total"] = int(sum(s["failures"] for s in seq))
    r["seq_segments"] = [dict(command=s["command"], err_xy=s.get("mean_xy_error_m_s"), err_yaw=s.get("mean_abs_yaw_error_rad_s"),
                              failures=s["failures"], drift=s.get("max_abs_heading_drift_rad")) for s in seq]
    r["robot_weight_n"] = bw
    rows[arm] = r
  Path(out_prefix).with_suffix(".json").write_text(json.dumps(rows, indent=1, default=float) + "\n")
  for arm, r in rows.items():
    h = r["hs_mean"]
    print(f"{arm:22s} hs err_xy {h['err_xy']:.3f} err_yaw {h['err_yaw']:.3f} tq2 {h['torque2']:.3f} slip {h['slip']:.3f} "
          f"sat {h['sat3nm']:.3f} ffp99 {h['foot_force_p99_bw']:.2f} ffmax {h['foot_force_max_bw']:.2f} seqfail {r['seq_failures_total']} | "
          + " ".join(f"{k}:{v['gait_guess']}" for k, v in r.items() if k[0] in "+-"))


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("args", nargs="+")
  p.add_argument("--reference-dir", type=Path)
  p.add_argument("--out-root", type=Path, default=ROOT)
  p.add_argument("--table", type=Path)
  a = p.parse_args()
  if a.table:
    table(a.table, a.args, a.out_root)
  else:
    arm, ckpt = a.args
    run(arm, Path(ckpt).expanduser(), a.reference_dir.resolve() if a.reference_dir else None, a.out_root)


if __name__ == "__main__":
  main()
