"""Opt Lab 2 Part 3: evaluation matrix, flat/Bumpy x Lab 4 DR on/off (simulation only).

For one checkpoint (and the reference dir it trained on) this runs, per
condition, the Part 2 extended suite (``part2_eval.run_suite``: 384 robot-runs,
falls by category, +-1.5 tracking, 3 Nm saturation, foot force, drift, splay,
slip, top speed), optionally the push test, and the Part 3 seam test
(``part3_seam``). Conditions (``part3_common.CONDITIONS``):

* ``flat_dr`` / ``bumpy_dr``: play config with the task's startup DR, asserted to
  equal the Lab 4 ranges (kp x[0.6, 1.1], kd x[0.8, 1.5], friction [0.6, 1.4]).
* ``flat_nodr`` / ``bumpy_nodr``: ``nominal_mechanics``: sampled friction,
  encoder bias, COM, PD gains and inertia removed; observation noise, latency
  and reset randomness kept.
* Bumpy = ``Mjlab-MixedGaits-Bumpy-Pupper-v3`` on the reviewed 0.10 m Perlin grid
  with the accumulated physics-overflow audit (nonzero overflow fails the run).

Everything runs in one process, one GPU job, sequentially.

  uv run --no-sync python -m mjlab.scripts.part3_eval run --checkpoint CKPT \
      --reference-dir REF --name NAME [--conditions flat_dr bumpy_dr ...] [--push]
  uv run --no-sync python -m mjlab.scripts.part3_eval table report/part3/eval/NAME ...
"""

from __future__ import annotations

import argparse
import copy
import json
import time
from pathlib import Path

from mjlab.scripts import part3_seam
from mjlab.scripts.part3_common import CONDITIONS, REPO_ROOT

DEFAULT_OUT_ROOT = REPO_ROOT / "report" / "part3" / "eval"

SUMMARY_SUITE = (
  ("falls", lambda m: m.get("falls_total")),
  ("rev/stop/steady/turn", lambda m: "/".join(str(m["falls_by_category"][c]) for c in ("reversal", "stop", "steady", "turn"))),
  ("err+1.5", lambda m: m.get("track_err_fwd")),
  ("err-1.5", lambda m: m.get("track_err_back")),
  ("vx+1.5", lambda m: m.get("mean_vx_fwd")),
  ("vx-1.5", lambda m: m.get("mean_vx_back")),
  ("sat@1.5", lambda m: m.get("torque_sat_frac_fast")),
  ("F_runmax_med_BW", lambda m: m.get("foot_force_runmax_median_bw")),
  ("drift+/-", lambda m: f"{_f(m['heading_drift_deg_per_10s_fwd']['mean'])}/{_f(m['heading_drift_deg_per_10s_back']['mean'])}"),
  ("slip", lambda m: m.get("slip_m_s")),
  ("jitter", lambda m: m.get("stand_jitter_rad_s")),
)
SUMMARY_SEAM = (
  ("seam fall (sweep)", lambda s, v: s["seam_sweep"]["fall_rate"]),
  ("seam incident (sweep)", lambda s, v: s["seam_sweep"]["incident_rate"]),
  ("dither fall/incident", lambda s, v: f"{_f(s['seam_dither']['fall_rate'])}/{_f(s['seam_dither']['incident_rate'])}"),
  ("control incident", lambda s, v: s["control_all"]["incident_rate"]),
  ("worst cell (excess, p)", lambda s, v: _worst(v)),
  ("step-sweep fall", lambda s, v: s["seam_sweep_step"]["fall_rate"]),
  ("seam sat", lambda s, v: s["seam_sweep"]["sat_frac_mean"]),
  ("seam F max med", lambda s, v: s["seam_sweep"]["foot_force_max_bw_median"]),
)


def _worst(v):
  cells = (v or {}).get("cells") or {}
  if not cells:
    return "-"
  k, c = max(cells.items(), key=lambda kv: kv[1]["excess"])
  return f"{k[5:]} {c['excess']:+.3f} (p={c['p']:.1g})"


def _f(v):
  if v is None:
    return "-"
  if isinstance(v, float):
    return f"{v:.3g}"
  return str(v)


def run(args):
  from mjlab.scripts import part2_eval
  from mjlab.scripts.part3_common import rollout_conditions

  out = Path(args.out_root) / args.name
  out.mkdir(parents=True, exist_ok=True)
  done = {}
  for cond in args.conditions:
    task, dr = CONDITIONS[cond]
    stem = f"{args.name}-{cond}"
    a = copy.copy(args)
    a.task, a.dr, a.condition, a.name, a.out_dir = task, dr, cond, stem, out
    t0 = time.time()
    if "suite" in args.modes and not (args.skip_existing and (out / f"{stem}.json").exists()):
      with rollout_conditions(a, dr=dr, audit_path=out / f"{stem}_overflow.json") as rec:
        payload = part2_eval.run_suite(a)
      _stamp(out / f"{stem}.json", rec)
      print(f"[part3_eval] {stem} suite {time.time() - t0:.0f}s falls={payload['metrics']['falls_total']} "
            f"overflow={rec.get('overflow_audit')}", flush=True)
    if "push" in args.modes and not (args.skip_existing and (out / f"{stem}_push.json").exists()):
      a.envs = args.push_envs
      with rollout_conditions(a, dr=dr, audit_path=out / f"{stem}_push_overflow.json") as rec:
        part2_eval.run_push(a)
      _stamp(out / f"{stem}_push.json", rec)
    if "seam" in args.modes and not (args.skip_existing and (out / f"{stem}_seam.json").exists()):
      for k, v in vars(part3_seam_defaults()).items():
        if not hasattr(a, k):
          setattr(a, k, v)
      a.envs_per_schedule = args.seam_envs_per_schedule
      part3_seam.run_seam(a)
    done[cond] = time.time() - t0
  table([out], out / "summary.md")
  print(json.dumps(dict(name=args.name, seconds=done), indent=1))


def _stamp(path, rec):
  """Record the Part 3 condition (terrain contract, DR, overflow) in the output JSON."""
  payload = json.loads(path.read_text())
  payload["part3_condition"] = rec
  path.write_text(json.dumps(payload, indent=2, default=str) + "\n")


def part3_seam_defaults():
  p = argparse.ArgumentParser()
  part3_seam.add_args(p)
  ns = p.parse_args(["--checkpoint", "x", "--name", "x"])
  for k in ("checkpoint", "name", "reference_dir", "condition", "out_dir", "seed", "device"):
    delattr(ns, k)
  return ns


def table(dirs, out_path=None):
  """One markdown table over evaluation dirs: suite + push + seam per condition."""
  from mjlab.scripts.part2_eval import _push_columns, rank_push  # noqa: F401

  lines = []
  head = ["run", "condition"] + [h for h, _ in SUMMARY_SUITE] + ["R:push_stand", "R:push_move"] + \
         [h for h, _ in SUMMARY_SEAM] + ["seam verdict", "overflow"]
  lines += ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
  for d in map(Path, dirs):
    for cond in CONDITIONS:
      suite = _load(d, f"*-{cond}.json")
      push = _load(d, f"*-{cond}_push.json")
      seam = _load(d, f"*-{cond}_seam.json")
      if not (suite or push or seam):
        continue
      row = [d.name, cond]
      row += [_f(fn(suite["metrics"])) if suite else "-" for _, fn in SUMMARY_SUITE]
      if push:
        r = rank_push(push["metrics"])
        row += [_f(r["standing_at_rung"]["rate"]), _f(r["moving_at_rung"]["rate"])]
      else:
        row += ["-", "-"]
      if seam:
        s, v = seam["metrics"]["summary"], seam["metrics"]["verdict"]
        row += [_f(fn(s, v)) for _, fn in SUMMARY_SEAM] + [f"{v['verdict']} ({v.get('role', '?')})"]
      else:
        row += ["-"] * (len(SUMMARY_SEAM) + 1)
      ov = [(x or {}).get("part3_condition", {}).get("overflow_audit") for x in (suite, push, seam)]
      ov = [o for o in ov if o]
      row.append("n/a (flat)" if not ov else ("0" if all(o["zero_overflow"] for o in ov) else "OVERFLOW"))
      lines.append("| " + " | ".join(row) + " |")
  text = "\n".join(lines) + "\n"
  if out_path:
    Path(out_path).write_text(text)
  print(text)
  return text


def _load(d, pattern):
  hits = sorted(p for p in Path(d).glob(pattern) if not p.name.endswith("_overflow.json"))
  return json.loads(hits[0].read_text()) if hits else None


def main():
  from mjlab.scripts import part2_eval

  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest="mode", required=True)
  r = sub.add_parser("run")
  part2_eval._add_rollout_args(r)
  r.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
  r.add_argument("--conditions", nargs="+", choices=sorted(CONDITIONS), default=list(CONDITIONS))
  r.add_argument("--modes", nargs="+", choices=("suite", "push", "seam"), default=["suite", "seam"])
  r.add_argument("--envs-per-schedule", type=int, default=24, help="suite (16 schedules)")
  r.add_argument("--seam-envs-per-schedule", type=int, default=64, help="seam (16 schedules; 64 -> 1024 envs)")
  r.add_argument("--push-envs", type=int, default=part2_eval.PUSH_ENVS_DEFAULT)
  r.add_argument("--skip-existing", action="store_true")
  t = sub.add_parser("table")
  t.add_argument("dirs", nargs="+", type=Path)
  t.add_argument("--out", type=Path)
  args = p.parse_args()
  if args.mode == "table":
    table(args.dirs, args.out)
    return
  part2_eval._prepare(args, p.error)
  run(args)


if __name__ == "__main__":
  main()
