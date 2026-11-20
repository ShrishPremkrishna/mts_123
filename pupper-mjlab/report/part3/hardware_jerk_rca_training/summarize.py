"""Print markdown tables from raw/jerk_<policy>.json (output of jerk_probe.py)."""
import json
import sys
from pathlib import Path

RAW = Path(__file__).resolve().parent / "raw"
POL = [p for p in ("fasxqy4z", "bumpy", "flatctrl", "p1") if (RAW / f"jerk_{p}.json").exists()]
D = {p: json.loads((RAW / f"jerk_{p}.json").read_text()) for p in POL}
CONDS = ["stand_nodr", "stand_dr", "flat_nodr", "flat_dr", "bumpy_dr"]
GROUPS = ["cmd0", "fwd0.1", "fwd0.3", "fwd0.5"]
METRICS = [
  ("action_abs_step", "mean abs action change per step", "{:.3f}"),
  ("action_rate_l2", "action_rate_l2 quantity (sum sq per step)", "{:.3f}"),
  ("action_hf_share", "share of action power above 5 Hz", "{:.2f}"),
  ("action_peak_hz", "action spectrum peak (Hz)", "{:.1f}"),
  ("joint_speed_abs", "mean abs joint speed (rad/s)", "{:.2f}"),
  ("torque_abs_p99", "p99 abs torque (Nm)", "{:.2f}"),
  ("torque_sat_frac", "share of samples at the 3 Nm limit", "{:.3f}"),
  ("target_at_clip_frac", "share of targets at the action clip", "{:.3f}"),
  ("q_near_limit_frac", "share of samples within 0.05 rad of a joint limit", "{:.3f}"),
  ("feet_contact_frac", "foot contact share", "{:.2f}"),
]


def fmt(v, f):
  return "–" if v is None else f.format(v)


def table(cond, metric_keys=None):
  ms = [m for m in METRICS if metric_keys is None or m[0] in metric_keys]
  lines = [f"#### {cond}", "", "| Metric | Command | " + " | ".join(POL) + " |",
           "|---|---|" + "---|" * len(POL)]
  for key, label, f in ms:
    for g in GROUPS:
      vals = []
      for p in POL:
        r = D[p].get(cond, {}).get(g, {})
        vals.append(fmt(r.get(key), f) + ("" if not r.get("fell") else f" ({r['fell']} fell)" if key == ms[0][0] else ""))
      lines.append(f"| {label} | {g} | " + " | ".join(vals) + " |")
  return "\n".join(lines)


def jac_table():
  lines = ["| Policy | Command | Ja spectral radius | Ja diag mean | Jq Frobenius | Jq diag mean | Jq 2-norm |",
           "|---|---|---|---|---|---|---|"]
  for p in POL:
    for cond in ("stand_nodr", "flat_nodr"):
      for g in GROUPS:
        j = D[p].get(cond, {}).get(g, {}).get("jacobian")
        if j:
          lines.append(f"| {p} ({cond}) | {g} | {j['Ja_spec_radius']:.2f} | {j['Ja_diag_mean']:.2f} | "
                       f"{j['Jq_fro']:.2f} | {j['Jq_diag_mean']:.2f} | {j['Jq_2norm']:.2f} |")
  return "\n".join(lines)


if __name__ == "__main__":
  which = sys.argv[1:] or CONDS
  for c in which:
    print(table(c))
    print()
  print(jac_table())
  for p in POL:
    for c in CONDS:
      m = D[p].get(c, {}).get("_meta")
      if m:
        print(p, c, "table_maxdiff", m["table_maxdiff"], "freq", m["frequency"], "gallop_speed", m["gallop_speed"])
