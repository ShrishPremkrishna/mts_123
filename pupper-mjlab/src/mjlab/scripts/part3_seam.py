"""Opt Lab 2 Part 3: command-sweep gait-seam test (simulation only).

The spec: "sweep the command from 0 through the fast-branch threshold and back.
The blend logic hands the policy a different choreography mid-stride; does it
stumble at the seam?" In ``mixed_gaits.mixed_reference_offset`` the reference
switches from the trot table at the base 1.33 Hz clock to the fast table at
``1.33 Hz x frequency_mult`` the moment ``|vx| >= 0.5`` (``TROTGALLOP_ONSET``).
Both the table *and* the phase jump at that instant (the phase is
``frac(t * f * mult)``, not continuous across the switch).

Schedules (each env runs one; every schedule is shifted by a per-env random
delay of 0..``--delay-max`` s so seam crossings land at uniformly spread gait
phases, i.e. genuinely mid-stride):

* ``sweep_<dir>_r<rate>``: 0 -> +-1.5 -> 0 m/s, ramping at ``rate`` m/s^2
  (``inf`` = step). Two seam crossings: ``up`` (trot -> fast) and ``down``.
* ``dither_<dir>``: hold 0.4, then step 0.6 / 0.4 every 1.5 s: eight step
  crossings right at the threshold.
* ``control_<dir>``: the same accelerations with no reference switch, as the
  comparator: a 0 -> 0.45 -> 0 ramp (trot only) and, after settling at 0.8, a
  0.8 -> 1.5 -> 0.8 ramp (fast only), both at ``--control-rate``. Its anchors
  are where the command passes 0.3 and 1.0.

Per crossing (window ``[-pre, +post]`` around the first sample whose command
is on the other side): fall (termination), stumble (non-fall: knee/torso ground
contact or body tilt above ``--tilt-deg``), 3 Nm saturation fraction, peak foot
force in actual body weights, tracking error, and the jump in the 12-dim
reference the policy observes. Windows whose env reset less than ``--settle`` s
before the window are not valid.

Rollouts reuse ``part2_eval._Rollout``; Part 3 conditions (Bumpy checked
terrain, DR on/off) come from :mod:`mjlab.scripts.part3_common`.

  uv run --no-sync python -m mjlab.scripts.part3_seam run --checkpoint CKPT \
      --reference-dir REF --name NAME [--condition flat_dr|flat_nodr|bumpy_dr|bumpy_nodr]
  uv run --no-sync python -m mjlab.scripts.part3_seam video --checkpoint CKPT \
      --reference-dir REF --name NAME --out-dir DIR [--condition ...]
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np

from mjlab.scripts.part3_common import CONDITIONS, REPO_ROOT, wilson

SEAM_SPEED = 0.5  # TROTGALLOP_ONSET; checked against the live env at runtime.
DIR_THRESHOLD = 0.05  # gait._DIR_THRESHOLD
TOP = 1.5
DT = 0.02
EFFORT_LIMIT_NM = 3.0
SAT_FRACTION = 0.99
DEFAULT_RATES = (0.25, 0.5, 1.0, "inf")
DEFAULT_OUT_DIR = REPO_ROOT / "report" / "part3" / "eval"
DEFAULT_RAW_DIR = Path.home() / "runs" / "opt-lab-2" / "p3-raw"

# Provisional stumble/pass thresholds. The final values are a lazy decision:
# report/decisions/2026-11-18-w4-seam-thresholds.md.
DEFAULT_TILT_DEG = 30.0
DEFAULT_PRE_S = 0.2
DEFAULT_POST_S = 1.0
DEFAULT_SETTLE_S = 1.0

##
# Schedules: list of (vx, seconds) holds/ramps -> per-step vx.
##


def _ramp_steps(a, b, rate, dt=DT):
  """Per-step values from a (exclusive) to b (inclusive) at |rate| m/s^2."""
  if rate == "inf" or (isinstance(rate, float) and math.isinf(rate)):
    return np.array([b], float)
  n = max(1, int(round(abs(b - a) / float(rate) / dt)))
  return a + (b - a) * (np.arange(1, n + 1) / n)


def _hold(v, seconds, dt=DT):
  return np.full(int(round(seconds / dt)), float(v))


def sweep(direction, rate, dt=DT):
  d = float(direction)
  return np.concatenate([
    _hold(0.0, 1.0, dt),
    _ramp_steps(0.0, d * TOP, rate, dt),
    _hold(d * TOP, 2.0, dt),
    _ramp_steps(d * TOP, 0.0, rate, dt),
    _hold(0.0, 2.0, dt),
  ])


def dither(direction, n_pairs=4, dt=DT):
  d = float(direction)
  parts = [_hold(0.0, 1.0, dt), _ramp_steps(0.0, d * 0.4, 0.5, dt), _hold(d * 0.4, 1.5, dt)]
  for _ in range(n_pairs):
    parts += [_hold(d * 0.6, 1.5, dt), _hold(d * 0.4, 1.5, dt)]
  parts += [_ramp_steps(d * 0.4, 0.0, 0.5, dt), _hold(0.0, 1.0, dt)]
  return np.concatenate(parts)


def control(direction, rate=0.5, dt=DT):
  d = float(direction)
  return np.concatenate([
    _hold(0.0, 1.0, dt),
    _ramp_steps(0.0, d * 0.45, rate, dt),
    _hold(d * 0.45, 1.5, dt),
    _ramp_steps(d * 0.45, 0.0, rate, dt),
    _hold(0.0, 1.0, dt),
    _ramp_steps(0.0, d * 0.8, "inf", dt),  # seam crossing; nearby sham anchors are dropped
    _hold(d * 0.8, 3.0, dt),
    _ramp_steps(d * 0.8, d * TOP, rate, dt),
    _hold(d * TOP, 1.5, dt),
    _ramp_steps(d * TOP, d * 0.8, rate, dt),
    _hold(d * 0.8, 1.5, dt),
  ])


def control_dither(direction, regime, n_pairs=4, dt=DT):
  """Step-matched control for the dither: the same 0.2 m/s steps, never crossing 0.5.

  regime 'trot': 0.2 <-> 0.4 (sham anchor 0.3); 'fast': 0.8 <-> 1.0 (sham anchor 0.9;
  its 0 -> 0.8 jump-in crosses the seam and nearby anchors are dropped).
  """
  d = float(direction)
  lo, hi = (0.2, 0.4) if regime == "trot" else (0.8, 1.0)
  if regime == "trot":
    parts = [_hold(0.0, 1.0, dt), _ramp_steps(0.0, d * lo, 0.5, dt), _hold(d * lo, 1.5, dt)]
  else:
    parts = [_hold(0.0, 1.0, dt), _ramp_steps(0.0, d * lo, "inf", dt), _hold(d * lo, 2.5, dt)]
  for _ in range(n_pairs):
    parts += [_hold(d * hi, 1.5, dt), _hold(d * lo, 1.5, dt)]
  parts += [_ramp_steps(d * lo, 0.0, 0.5, dt), _hold(0.0, 1.0, dt)]
  return np.concatenate(parts)


def rate_label(rate):
  return "inf" if rate == "inf" else f"{float(rate):g}"


def schedule_set(rates=DEFAULT_RATES, control_rate=0.5, dt=DT):
  """[(name, kind, direction, rate, vx (steps,), sham_levels)]."""
  out = []
  for d, dn in ((1, "fwd"), (-1, "back")):
    for r in rates:
      out.append((f"sweep_{dn}_r{rate_label(r)}", "sweep", dn, rate_label(r), sweep(d, r, dt), ()))
    out.append((f"dither_{dn}", "dither", dn, "step", dither(d, dt=dt), ()))
    out.append((f"control_{dn}", "control", dn, rate_label(control_rate), control(d, control_rate, dt), (0.3, 1.0)))
    out.append((f"control_dither_trot_{dn}", "control", dn, "step", control_dither(d, "trot", dt=dt), (0.3,)))
    out.append((f"control_dither_fast_{dn}", "control", dn, "step", control_dither(d, "fast", dt=dt), (0.9,)))
  return out


def build_commands(schedules, envs_per_schedule, delay_max_s, seed, dt=DT, tail_s=0.5):
  """Per-env (T, N, 3) commands with a random leading delay; env_schedule (N,), delay (N,)."""
  rng = np.random.default_rng(seed)
  n = len(schedules) * envs_per_schedule
  env_schedule = np.arange(n) // envs_per_schedule
  max_delay = int(round(delay_max_s / dt))
  delay = rng.integers(0, max_delay + 1, size=n) if max_delay > 0 else np.zeros(n, int)
  T = max(len(s[4]) for s in schedules) + max_delay + int(round(tail_s / dt))
  cmd = np.zeros((T, n, 3))
  for e in range(n):
    vx = schedules[env_schedule[e]][4]
    cmd[delay[e] : delay[e] + len(vx), e, 0] = vx
  return cmd, env_schedule, delay


def crossings(vx, level, dir_threshold=DIR_THRESHOLD):
  """Indices k where the command crosses ``level`` (as the fast branch tests it).

  Returns [(k, 'up'|'down')]: k is the first sample on the new side. 'up' means
  into ``|vx| >= level``. Crossing through zero (direction flip) is not a seam.
  """
  vx = np.asarray(vx, float)
  above = (np.abs(vx) >= level) & (np.abs(vx) >= dir_threshold)
  ks = np.flatnonzero(above[1:] != above[:-1]) + 1
  return [(int(k), "up" if above[k] else "down") for k in ks]


##
# Metrics (pure NumPy; unit tested on synthetic arrays).
##


def body_tilt_deg(projected_gravity):
  """Angle between body z and world up from the projected gravity vector (..., 3)."""
  g = np.asarray(projected_gravity, float)
  cos = np.clip(-g[..., 2] / np.maximum(np.linalg.norm(g, axis=-1), 1e-9), -1, 1)
  return np.degrees(np.arccos(cos))


def episode_age_steps(done):
  done = np.asarray(done, bool)
  age = np.zeros(done.shape, int)
  for k in range(1, len(done)):
    age[k] = np.where(done[k - 1], 0, age[k - 1] + 1)
  return age


def _window_events(sl, terminated, knee, torso, tilt, tilt_deg):
  fell = bool(terminated[sl].any())
  # Signals after the terminating sample belong to the next episode.
  if fell:
    stop = sl.start + int(np.argmax(terminated[sl])) + 1
    sl = slice(sl.start, stop)
  k = bool(knee[sl].any())
  t = bool(torso[sl].any())
  tl = bool((tilt[sl] > tilt_deg).any())
  return dict(fall=fell, knee=k, torso=t, tilt=tl, stumble=(not fell) and (k or t or tl),
              incident=fell or k or t or tl)


def seam_metrics(
  *,
  cmd,
  env_schedule,
  schedules,
  terminated,
  velocity,
  tilt,
  knee,
  torso,
  torque,
  foot_force,
  mass_kg,
  ref,
  dt=DT,
  gravity=9.81,
  pre_s=DEFAULT_PRE_S,
  post_s=DEFAULT_POST_S,
  settle_s=DEFAULT_SETTLE_S,
  tilt_deg=DEFAULT_TILT_DEG,
  seam_speed=SEAM_SPEED,
  effort_limit=EFFORT_LIMIT_NM,
) -> dict[str, Any]:
  """Per-window events and continuous metrics, grouped by schedule kind/direction/crossing.

  Shapes: cmd (T,N,3); terminated/knee/torso (T,N) bool; velocity (T,N,3)
  [vx, vy, wz body]; tilt (T,N) deg; torque (T,N,12); foot_force (T,N,4) N;
  mass_kg (N,); ref (T,N,12) observed reference offset.
  """
  T, N = terminated.shape
  pre, post = int(round(pre_s / dt)), int(round(post_s / dt))
  settle = int(round(settle_s / dt))
  age = episode_age_steps(terminated)
  sat = np.abs(torque) >= SAT_FRACTION * effort_limit  # (T,N,12)
  bw = foot_force / (np.asarray(mass_kg)[None, :, None] * gravity)
  err = np.linalg.norm(velocity[..., :2] - cmd[..., :2], axis=-1)
  dref = np.linalg.norm(np.diff(ref, axis=0), axis=-1)  # (T-1,N): ref[k]-ref[k-1] at k-1
  windows = []
  invalid = dict(edge=0, reset=0)
  for e in range(N):
    name, kind, dn, rate, _, sham = schedules[env_schedule[e]]
    anchors = []
    if kind == "control":
      # Sham anchors near a real seam crossing (the control's 0 -> 0.8 jump-in)
      # are not seam-free; drop them.
      seam_ks = [k for k, _ in crossings(cmd[:, e, 0], seam_speed)]
      for lvl in sham:
        anchors += [(k, c, f"sham{lvl:g}") for k, c in crossings(cmd[:, e, 0], lvl)
                    if all(abs(k - ks) > pre + post for ks in seam_ks)]
    else:
      anchors += [(k, c, "seam") for k, c in crossings(cmd[:, e, 0], seam_speed)]
    for k, c, what in anchors:
      a, b = k - pre, k + post + 1
      if a < 1 or b > T:
        invalid["edge"] += 1
        continue
      if age[a, e] < settle:
        invalid["reset"] += 1
        continue
      sl = slice(a, b)
      ev = _window_events(sl, terminated[:, e], knee[:, e], torso[:, e], tilt[:, e], tilt_deg)
      alive = slice(a, a + int(np.argmax(terminated[sl, e])) + 1) if ev["fall"] else sl
      base = slice(max(1, a - int(round(0.5 / dt))), a)
      windows.append(dict(
        env=e, schedule=name, kind=kind, direction=dn, rate=rate, crossing=c, anchor=what, k=k,
        **ev,
        max_tilt_deg=float(tilt[alive, e].max()),
        sat_frac=float(sat[alive, e].mean()),
        foot_force_max_bw=float(bw[alive, e].max()),
        track_err=float(err[alive, e].mean()),
        track_err_pre=float(err[base, e].mean()) if base.stop > base.start else None,
        ref_jump=float(dref[k - 1, e]) if what == "seam" else None,
        ref_step_typical=float(np.median(dref[max(0, k - 26) : k - 1, e])) if k > 2 else None,
      ))
  groups = _group(windows)
  return dict(
    params=dict(pre_s=pre_s, post_s=post_s, settle_s=settle_s, tilt_deg=tilt_deg,
                seam_speed=seam_speed, effort_limit_nm=effort_limit,
                sat_threshold_nm=SAT_FRACTION * effort_limit),
    n_windows=len(windows),
    invalid_windows=invalid,
    groups=groups,
    summary=_summary(windows),
    windows=windows,
  )


def _rate_block(ws):
  n = len(ws)
  out = dict(n=n)
  for key in ("fall", "stumble", "incident", "knee", "torso", "tilt"):
    k = sum(w[key] for w in ws)
    lo, hi = wilson(k, n)
    out[key] = k
    out[f"{key}_rate"] = k / n if n else None
    if key in ("fall", "stumble", "incident"):
      out[f"{key}_ci95"] = [lo, hi]

  def med(key):
    v = [w[key] for w in ws if w[key] is not None]
    return float(np.median(v)) if v else None

  def p90(key):
    v = [w[key] for w in ws if w[key] is not None]
    return float(np.percentile(v, 90)) if v else None

  out.update(
    sat_frac_mean=float(np.mean([w["sat_frac"] for w in ws])) if ws else None,
    foot_force_max_bw_median=med("foot_force_max_bw"),
    foot_force_max_bw_p90=p90("foot_force_max_bw"),
    max_tilt_deg_median=med("max_tilt_deg"),
    max_tilt_deg_p90=p90("max_tilt_deg"),
    track_err_median=med("track_err"),
    track_err_pre_median=med("track_err_pre"),
    ref_jump_median=med("ref_jump"),
    ref_step_typical_median=med("ref_step_typical"),
  )
  return out


def _group(windows):
  keys = sorted({(w["kind"], w["rate"], w["direction"], w["anchor"], w["crossing"]) for w in windows})
  rows = []
  for kind, rate, dn, anchor, c in keys:
    ws = [w for w in windows if (w["kind"], w["rate"], w["direction"], w["anchor"], w["crossing"]) == (kind, rate, dn, anchor, c)]
    rows.append(dict(kind=kind, rate=rate, direction=dn, anchor=anchor, crossing=c, **_rate_block(ws)))
  return rows


def _summary(windows):
  """Headline blocks. ``seam_sweep`` = finite ramp rates only: a step sweep (rate
  inf) also commands a 0 <-> 1.5 m/s jump, so its falls mix the seam with a hard
  start/stop and it is reported apart (``seam_sweep_step``). ``seam_dither`` is
  the cleanest isolation of the seam: a 0.2 m/s step straddling 0.5 m/s."""
  seam = [w for w in windows if w["anchor"] == "seam"]
  ctrl = [w for w in windows if w["anchor"] != "seam"]
  sweep = [w for w in seam if w["kind"] == "sweep" and w["rate"] != "inf"]
  out = dict(
    seam_sweep=_rate_block(sweep),
    seam_sweep_r05=_rate_block([w for w in sweep if w["rate"] == "0.5"]),
    seam_sweep_step=_rate_block([w for w in seam if w["kind"] == "sweep" and w["rate"] == "inf"]),
    seam_dither=_rate_block([w for w in seam if w["kind"] == "dither"]),
    control_all=_rate_block(ctrl),
    control_ramp=_rate_block([w for w in ctrl if w["rate"] != "step"]),
    control_dither=_rate_block([w for w in ctrl if w["rate"] == "step"]),
  )
  # The 8 judged cells (decision 2026-11-18-w4-seam-thresholds): block x direction x crossing.
  for block, ws in (("sweep", sweep), ("dither", [w for w in seam if w["kind"] == "dither"])):
    for dn in ("fwd", "back"):
      for c in ("up", "down"):
        out[f"cell_{block}_{dn}_{c}"] = _rate_block([w for w in ws if w["direction"] == dn and w["crossing"] == c])
  for c in ("up", "down"):
    out[f"seam_sweep_{c}"] = _rate_block([w for w in sweep if w["crossing"] == c])
  for dn in ("fwd", "back"):
    out[f"seam_sweep_{dn}"] = _rate_block([w for w in sweep if w["direction"] == dn])
  return out


GROUP_COLUMNS = (
  ("kind", "kind"), ("rate", "rate"), ("direction", "dir"), ("anchor", "anchor"), ("crossing", "cross"),
  ("n", "n"), ("fall_rate", "fall"), ("incident_rate", "incident"), ("stumble_rate", "stumble"), ("knee_rate", "knee"),
  ("torso_rate", "torso"), ("tilt_rate", "tilt>thr"), ("max_tilt_deg_median", "tilt med"),
  ("sat_frac_mean", "sat3Nm"), ("foot_force_max_bw_median", "F max med BW"),
  ("foot_force_max_bw_p90", "F max p90"), ("track_err_pre_median", "err pre"),
  ("track_err_median", "err win"), ("ref_jump_median", "ref jump"),
  ("ref_step_typical_median", "ref step"),
)


def _fmt(v):
  if v is None:
    return "-"
  if isinstance(v, float):
    return f"{v:.3g}"
  return str(v)


def markdown(metrics, title):
  lines = [f"## {title}", ""]
  s = metrics["summary"]
  lines.append("| block | n | fall | incident | stumble (95% CI) | sat 3 Nm | F max med BW | tilt med | err win / pre | ref jump / typical step |")
  lines.append("|---|---|---|---|---|---|---|---|---|---|")
  for key in ("seam_sweep", "seam_sweep_up", "seam_sweep_down", "seam_sweep_fwd", "seam_sweep_back",
              "seam_sweep_r05", "seam_dither", "control_all", "control_ramp", "control_dither",
              "seam_sweep_step", *CELLS):
    b = s[key]
    ci = b.get("stumble_ci95", [None, None])
    lines.append(
      f"| {key} | {b['n']} | {_fmt(b.get('fall_rate'))} | {_fmt(b.get('incident_rate'))} | {_fmt(b.get('stumble_rate'))} "
      f"({_fmt(ci[0])}-{_fmt(ci[1])}) | {_fmt(b.get('sat_frac_mean'))} | {_fmt(b.get('foot_force_max_bw_median'))} "
      f"| {_fmt(b.get('max_tilt_deg_median'))} | {_fmt(b.get('track_err_median'))} / {_fmt(b.get('track_err_pre_median'))} "
      f"| {_fmt(b.get('ref_jump_median'))} / {_fmt(b.get('ref_step_typical_median'))} |"
    )
  lines += ["", "Per group:", "", "| " + " | ".join(h for _, h in GROUP_COLUMNS) + " |",
            "|" + "---|" * len(GROUP_COLUMNS)]
  for g in metrics["groups"]:
    lines.append("| " + " | ".join(_fmt(g.get(k)) for k, _ in GROUP_COLUMNS) + " |")
  p = metrics["params"]
  lines += ["", f"Window [-{p['pre_s']}, +{p['post_s']}] s around each crossing; stumble = no fall and "
            f"(knee or torso ground contact, or tilt > {p['tilt_deg']} deg); sat = |tau| >= {p['sat_threshold_nm']:.2f} Nm. "
            f"Invalid windows: {metrics['invalid_windows']}.", ""]
  return "\n".join(lines)


CELLS = tuple(f"cell_{b}_{d}_{c}" for b in ("sweep", "dither") for d in ("fwd", "back") for c in ("up", "down"))


def fisher_greater(k, n, c, n_c):
  """One-sided Fisher exact p that the cell's incident rate exceeds the control's."""
  from scipy.stats import fisher_exact

  return float(fisher_exact([[k, n - k], [c, n_c - c]], alternative="greater")[1])


def verdict(metrics, criteria):
  """Seam verdict for one condition (decision report/decisions/2026-11-18-w4-seam-thresholds.md).

  FAIL if the pooled ``seam_sweep`` (finite rates) or ``seam_dither`` fall rate exceeds
  ``max_fall_rate``, or any of the 8 cells (sweep|dither x fwd|back x up|down) has an
  incident rate exceeding the pooled no-seam control's by more than
  ``max_incident_excess`` with one-sided Fisher p < ``fisher_alpha``. A cell with
  n < ``min_cell_n`` (or an empty control) is NO DATA. Step sweep and r0.5 are
  reported, not judged. Cells with an excess in ``close_call`` are flagged for a
  second-seed rerun.
  """
  s = metrics["summary"]
  ctrl = s["control_all"]
  reasons, cells, no_data, close = [], {}, [], []
  if ctrl["n"] == 0:
    return dict(verdict="NO DATA", reasons=["no control windows"], criteria=criteria)
  for key in ("seam_sweep", "seam_dither"):
    b = s[key]
    if b["n"] and b["fall_rate"] > criteria["max_fall_rate"]:
      reasons.append(f"{key} fall rate {b['fall_rate']:.3f} > {criteria['max_fall_rate']}")
  cr = ctrl["incident"] / ctrl["n"]
  for key in CELLS:
    b = s[key]
    if b["n"] < criteria["min_cell_n"]:
      no_data.append(f"{key} n={b['n']}")
      continue
    excess = b["incident_rate"] - cr
    pval = fisher_greater(b["incident"], b["n"], ctrl["incident"], ctrl["n"])
    cells[key] = dict(n=b["n"], incidents=b["incident"], excess=excess, p=pval)
    if excess > criteria["max_incident_excess"] and pval < criteria["fisher_alpha"]:
      reasons.append(f"{key}: incidents {b['incident']}/{b['n']} vs control {ctrl['incident']}/{ctrl['n']} "
                     f"(excess {excess:.3f}, p={pval:.1e})")
    lo, hi = criteria.get("close_call", (0.03, 0.07))
    if lo <= excess <= hi:
      close.append(key)
  v = "FAIL" if reasons else ("NO DATA" if no_data else "PASS")
  return dict(verdict=v, reasons=reasons, no_data=no_data, close_calls_rerun_second_seed=close,
              cells=cells, control_incident_rate=cr, criteria=criteria)


# Frozen before any final-policy numbers (decision 2026-11-18-w4-seam-thresholds).
SEAM_CRITERIA = dict(max_fall_rate=0.02, max_incident_excess=0.05, fisher_alpha=0.01, min_cell_n=60,
                     close_call=(0.03, 0.07))
CONDITION_ROLE = dict(flat_dr="headline", bumpy_dr="robustness", flat_nodr="diagnostic", bumpy_nodr="diagnostic")

##
# GPU rollout.
##


def _setup_args(args):
  from mjlab.scripts import part2_eval

  task, dr = CONDITIONS[args.condition]
  args.task = task
  args.dr = dr
  if not hasattr(args, "heading_hold"):
    args.heading_hold = False
  part2_eval._prepare(args, _raise)
  return args


def _raise(msg):
  raise ValueError(msg)


def run_seam(args) -> dict[str, Any]:
  import torch

  from mjlab.scripts import part2_eval
  from mjlab.scripts.part3_common import TORSO_SENSOR, rollout_conditions

  t0 = time.time()
  rates = tuple("inf" if r == "inf" else float(r) for r in args.rates)
  schedules = schedule_set(rates, args.control_rate)
  cmd, env_schedule, delay = build_commands(schedules, args.envs_per_schedule, args.delay_max, args.seed)
  T, n, _ = cmd.shape
  out_dir = Path(args.out_dir)
  audit = out_dir / f"{args.name}_overflow.json"
  with rollout_conditions(args, dr=args.dr, checked_bumpy=not args.unchecked_bumpy,
                          audit_path=audit, torso_sensor=True) as cond:
    roll = part2_eval._Rollout(args, n, T)
    try:
      if abs(roll.dt - DT) > 1e-9:
        raise ValueError(f"dt {roll.dt} != {DT}")
      part2_eval.check_ranges(cmd, roll.ranges)
      from mjlab.tasks.pupper_gait.export import gait_params_from_env
      from mjlab.tasks.pupper_gait.mdp.mixed_gaits import mixed_reference_offset

      params = gait_params_from_env(roll.env)
      if abs(params["gallop_speed"] - SEAM_SPEED) > 1e-9:
        raise ValueError(f"env gallop_speed {params['gallop_speed']} != {SEAM_SPEED}")
      env, robot, data = roll.env, roll.robot, roll.robot.data
      from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES, PUPPER_EFFORT_LIMIT

      if abs(PUPPER_EFFORT_LIMIT - EFFORT_LIMIT_NM) > 1e-9:
        raise ValueError(f"PUPPER_EFFORT_LIMIT {PUPPER_EFFORT_LIMIT} != {EFFORT_LIMIT_NM}")

      joint_ids = [robot.joint_names.index(j) for j in JOINT_NAMES]
      feet = env.scene["feet_ground_contact"]
      knee = env.scene["knee_ground_contact"]
      torso = env.scene[TORSO_SENSOR] if cond.get("torso_sensor") else None
      dev = args.device
      z = lambda *s, dtype=torch.float32: torch.zeros(T, n, *s, dtype=dtype, device=dev)  # noqa: E731
      bufs = dict(velocity=z(3), gravity=z(3), torque=z(12), foot_force=z(4), ref=z(12),
                  knee=z(dtype=torch.bool), torso=z(dtype=torch.bool))

      def record(k):
        bufs["velocity"][k] = torch.cat([data.root_link_lin_vel_b[:, :2], data.root_link_ang_vel_b[:, 2:3]], 1)
        bufs["gravity"][k] = data.projected_gravity_b
        bufs["torque"][k] = data.qfrc_actuator[:, joint_ids]
        bufs["foot_force"][k] = torch.linalg.vector_norm(feet.data.force, dim=-1)
        bufs["knee"][k] = (knee.data.found > 0).reshape(n, -1).any(-1)
        if torso is not None:
          bufs["torso"][k] = (torso.data.found > 0).reshape(n, -1).any(-1)
        bufs["ref"][k] = mixed_reference_offset(
          env, "twist", params["frequency"], params["blend_speed"], int(params["n_samples"]), params["gallop_speed"])

      mass, nominal, g, mass_source = roll.masses()
      cmd_tn = torch.tensor(cmd, dtype=torch.float32, device=dev)
      t_loop = time.time()
      terminated, timeout = roll.run_loop(cmd_tn, record=record)
      arrays = {k: v.cpu().numpy() for k, v in bufs.items()}
      loop_s = time.time() - t_loop
      run_prov = roll.run
    finally:
      roll.close()
  raw = dict(cmd=cmd.astype(np.float32), env_schedule=env_schedule, delay=delay, terminated=terminated,
             mass_kg=mass, gravity_m_s2=np.float64(g), **arrays)
  raw_dir = Path(args.raw_dir) if args.raw_dir else DEFAULT_RAW_DIR
  raw_dir.mkdir(parents=True, exist_ok=True)
  raw_path = raw_dir / f"{args.name}_seam_raw.npz"
  np.savez_compressed(raw_path, **{k: v.astype(np.float16) if v.dtype == np.float32 and k not in ("cmd",) else v
                                   for k, v in raw.items()})
  meta = dict(
    name=args.name, kind="seam", condition=args.condition,
    rates=[rate_label(r) for r in rates], control_rate=args.control_rate,
    schedule_names=[s[0] for s in schedules],
    envs_per_schedule=args.envs_per_schedule, delay_max_s=args.delay_max, steps=T, dt=DT, seed=args.seed,
    mass_source=mass_source, mass_nominal_kg=nominal, wall_clock_s=time.time() - t0, rollout_loop_s=loop_s,
    raw=str(raw_path), provenance=run_prov, training=args.training, part3_condition=cond,
  )
  (raw_dir / f"{args.name}_seam_meta.json").write_text(json.dumps(meta, indent=2, default=_jsonable) + "\n")
  return analyze(raw, meta, schedules, args, out_dir)


def analyze(raw, meta, schedules, args, out_dir, criteria=None):
  """Metrics + outputs from raw rollout arrays (also used offline by ``metrics``)."""
  criteria = criteria or SEAM_CRITERIA
  cmd = np.asarray(raw["cmd"], float)
  terminated = np.asarray(raw["terminated"], bool)
  f = lambda k: np.asarray(raw[k], np.float32)  # noqa: E731
  tilt = body_tilt_deg(f("gravity"))
  knee, torso = np.asarray(raw["knee"], bool), np.asarray(raw["torso"], bool)
  metrics = seam_metrics(
    cmd=cmd, env_schedule=np.asarray(raw["env_schedule"]), schedules=schedules, terminated=terminated,
    velocity=f("velocity"), tilt=tilt, knee=knee, torso=torso, torque=f("torque"),
    foot_force=f("foot_force"), mass_kg=np.asarray(raw["mass_kg"], float), ref=f("ref"),
    gravity=float(raw["gravity_m_s2"]), pre_s=args.pre, post_s=args.post, settle_s=args.settle,
    tilt_deg=args.tilt_deg, effort_limit=EFFORT_LIMIT_NM)
  # Steady-state distributions, for threshold setting (alive, settled >= 1 s).
  age = episode_age_steps(terminated)
  settled = (age >= int(1.0 / DT)) & ~terminated
  steady = (np.abs(cmd[..., 0]) >= TOP - 1e-6) & settled
  stand = (np.abs(cmd[..., 0]) < 1e-9) & settled
  trot = (np.abs(cmd[..., 0]) > 0.2) & (np.abs(cmd[..., 0]) < 0.45) & settled

  def pct(mask):
    return [float(np.percentile(tilt[mask], q)) for q in (50, 99, 99.9)] if mask.any() else None

  metrics["baseline"] = dict(
    tilt_deg_p50_p99_p999=dict(stand=pct(stand), trot=pct(trot), steady_fast=pct(steady)),
    knee_contact_frac=dict(stand=_frac(knee, stand), trot=_frac(knee, trot), steady_fast=_frac(knee, steady)),
    torso_contact_frac=dict(stand=_frac(torso, stand), trot=_frac(torso, trot), steady_fast=_frac(torso, steady)),
    total_terminations=int(terminated.sum()),
    torso_sensor=bool(meta.get("part3_condition", {}).get("torso_sensor")),
  )
  metrics["verdict"] = verdict(metrics, criteria)
  metrics["verdict"]["role"] = CONDITION_ROLE.get(meta.get("condition"), "?")
  sens = {}
  for tdeg in (25.0, 35.0):
    if abs(tdeg - args.tilt_deg) < 1e-9:
      continue
    m2 = seam_metrics(
      cmd=cmd, env_schedule=np.asarray(raw["env_schedule"]), schedules=schedules, terminated=terminated,
      velocity=f("velocity"), tilt=tilt, knee=knee, torso=torso, torque=f("torque"),
      foot_force=f("foot_force"), mass_kg=np.asarray(raw["mass_kg"], float), ref=f("ref"),
      gravity=float(raw["gravity_m_s2"]), pre_s=args.pre, post_s=args.post, settle_s=args.settle,
      tilt_deg=tdeg, effort_limit=EFFORT_LIMIT_NM)
    v2 = verdict(m2, criteria)
    sens[f"tilt_{tdeg:g}"] = dict(verdict=v2["verdict"], reasons=v2["reasons"],
                                  max_cell_excess=max((c["excess"] for c in v2["cells"].values()), default=None),
                                  control_max_tilt_deg=_ctrl_max_tilt(m2))
  metrics["sensitivity"] = sens
  metrics["control_max_tilt_deg"] = _ctrl_max_tilt(metrics)
  payload = dict(meta, metrics=metrics, schedules=[
    dict(name=s[0], kind=s[1], direction=s[2], rate=s[3], steps=len(s[4]), sham=list(s[5])) for s in schedules])
  if not getattr(args, "keep_windows", False):
    payload["metrics"] = {k: v for k, v in metrics.items() if k != "windows"}
  out_dir = Path(out_dir)
  out_dir.mkdir(parents=True, exist_ok=True)
  name = meta["name"]
  (out_dir / f"{name}_seam.json").write_text(json.dumps(payload, indent=2, default=_jsonable) + "\n")
  (out_dir / f"{name}_seam.md").write_text(
    markdown(metrics, f"Seam test: {name} ({meta['condition']})")
    + f"\nBaseline (settled, alive): {json.dumps(metrics['baseline'], default=_jsonable)}\n"
    + f"\nVerdict ({metrics['verdict']['role']}; criteria {criteria}): **{metrics['verdict']['verdict']}** "
    f"{metrics['verdict']['reasons']}  \nCells (excess over control, Fisher p): "
    + ", ".join(f"{k[5:]} {v['incidents']}/{v['n']} {v['excess']:+.3f} p={v['p']:.2g}" for k, v in metrics['verdict']['cells'].items())
    + f"  \nClose calls (rerun with a second seed): {metrics['verdict']['close_calls_rerun_second_seed']}; "
    f"no data: {metrics['verdict']['no_data']}  \nControl max tilt {metrics['control_max_tilt_deg']:.1f} deg "
    f"(re-derive the threshold if > 25). Sensitivity: {json.dumps(metrics['sensitivity'], default=_jsonable)}\n")
  print(json.dumps(dict(name=name, condition=meta["condition"], wall_clock_s=meta.get("wall_clock_s"),
                        summary={k: {kk: vv for kk, vv in v.items() if kk in ("n", "fall_rate", "stumble_rate")}
                                 for k, v in metrics["summary"].items()},
                        baseline=metrics["baseline"], verdict=metrics["verdict"],
                        overflow=meta.get("part3_condition", {}).get("overflow_audit")), indent=1, default=_jsonable))
  return payload


def _ctrl_max_tilt(m):
  v = [w["max_tilt_deg"] for w in m["windows"] if w["anchor"] != "seam"]
  return float(max(v)) if v else None


def _frac(x, mask):
  return float(x[mask].mean()) if mask.any() else None


def recompute(args):
  meta = json.loads(Path(args.meta).read_text())
  raw = dict(np.load(meta["raw"]))
  rates = tuple("inf" if r == "inf" else float(r) for r in meta["rates"])
  full = schedule_set(rates, meta["control_rate"])
  # Rebuild exactly the schedules that were recorded, in their recorded order.
  names = meta.get("schedule_names") or [s[0] for s in full if not s[0].startswith("control_dither_")]
  by_name = {s[0]: s for s in full}
  schedules = [by_name[n] for n in names]
  if int(np.max(raw["env_schedule"])) >= len(schedules):
    raise ValueError("raw env_schedule indexes beyond the reconstructed schedule set")
  return analyze(raw, meta, schedules, args, args.out_dir)


def _jsonable(o):
  if isinstance(o, (np.integer,)):
    return int(o)
  if isinstance(o, (np.floating,)):
    return float(o)
  if isinstance(o, np.ndarray):
    return o.tolist()
  if isinstance(o, Path):
    return str(o)
  return str(o)


def video_schedule(rate=0.5, dt=DT):
  """0 -> +1.5 -> 0 -> -1.5 -> 0, then a dither at the seam (one camera-tracked robot)."""
  return np.concatenate([sweep(1, rate, dt), sweep(-1, rate, dt)[int(1.0 / dt):], dither(1, n_pairs=2, dt=dt)[int(1.0 / dt):]])


def run_video(args):
  import imageio.v2 as imageio
  import torch

  from mjlab.scripts import part2_eval
  from mjlab.scripts.part3_common import rollout_conditions

  vx = video_schedule(args.video_rate)
  T = len(vx) + 25
  cmd = np.zeros((T, 1, 3))
  cmd[:len(vx), 0, 0] = vx
  frames = []
  out_dir = Path(args.out_dir)
  out_dir.mkdir(parents=True, exist_ok=True)
  viewer = dict(width=args.width, height=args.height, max_extra_envs=0, env_idx=0, distance=1.2, elevation=-15.0,
                azimuth=135.0)
  with rollout_conditions(args, dr=args.dr, checked_bumpy=True, audit_path=out_dir / f"{args.name}_video_overflow.json",
                          render=True, viewer=viewer) as cond:
    roll = part2_eval._Rollout(args, 1, T)
    falls = []
    try:
      env = roll.env

      def record(k):
        if k % 2 == 0:
          frame = env.render()
          frames.append(_annotate(frame, k, cmd[k, 0, 0]))

      terminated, _ = roll.run_loop(torch.tensor(cmd, dtype=torch.float32, device=args.device), record=record)
      falls = np.flatnonzero(terminated[:, 0]).tolist()
    finally:
      roll.close()
  path = out_dir / f"{args.name}_seam_sweep.mp4"
  imageio.mimsave(path, frames, fps=25)  # every 2nd 50 Hz step -> real time
  meta = dict(name=args.name, condition=args.condition, video=str(path), frames=len(frames), seconds=len(frames) / 25,
              rate_m_s2=args.video_rate, falls_at_step=falls, part3_condition=cond,
              checkpoint=str(args.checkpoint), reference_dir=str(args.reference_dir))
  (out_dir / f"{args.name}_seam_sweep.json").write_text(json.dumps(meta, indent=2, default=_jsonable) + "\n")
  print(json.dumps(meta, default=_jsonable))
  return meta


def _annotate(frame, k, vx):
  try:
    from PIL import Image, ImageDraw
  except ImportError:
    return frame
  img = Image.fromarray(frame)
  d = ImageDraw.Draw(img)
  slot = "stand" if abs(vx) < DIR_THRESHOLD else ("FAST" if abs(vx) >= SEAM_SPEED else "trot")
  d.rectangle([0, 0, 250, 22], fill=(0, 0, 0))
  d.text((6, 5), f"t={k * DT:5.2f}s  cmd vx={vx:+.2f}  ref={slot}", fill=(255, 255, 255))
  return np.asarray(img)


def add_args(p, video=False):
  p.add_argument("--checkpoint", type=Path, required=True)
  p.add_argument("--reference-dir", type=Path)
  p.add_argument("--name", required=True)
  p.add_argument("--condition", choices=sorted(CONDITIONS), default="flat_dr")
  p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
  p.add_argument("--seed", type=int, default=1234)
  p.add_argument("--device", default="cuda:0")
  p.add_argument("--allow-reference-mismatch", action="store_true")
  p.add_argument("--no-standard-physics", action="store_true")
  p.add_argument("--driver-limits", type=Path)
  p.add_argument("--target-slew-rad-s", type=float)
  p.add_argument("--encoder-observation-mode", choices=("legacy_true", "deployment_measured"))
  p.add_argument("--allow-encoder-observation-mode-change", action="store_true")
  for f in ("--deployment-target-clamp", "--compose-com-randomization", "--locomotion-torso-contact"):
    p.add_argument(f, action="store_true")
  p.add_argument("--train-num-envs", type=int)
  p.add_argument("--train-seed", type=int)
  p.add_argument("--parent-iter", type=int)
  if video:
    p.add_argument("--video-rate", type=float, default=0.5)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
  else:
    p.add_argument("--envs-per-schedule", type=int, default=64, help="16 schedules x 64 = 1024 envs (shared-GPU cap)")
    p.add_argument("--rates", nargs="+", default=[str(r) for r in DEFAULT_RATES])
    p.add_argument("--control-rate", type=float, default=0.5)
    p.add_argument("--delay-max", type=float, default=0.75, help="Per-env random start delay (s); 0.75 = one trot cycle")
    _add_window_args(p)
    p.add_argument("--raw-dir", type=Path, help="Raw arrays (default ~/runs/opt-lab-2/p3-raw; never commit)")
    p.add_argument("--unchecked-bumpy", action="store_true", help="Diagnostic: stock 0.05 m grid (overflow-prone)")


def _add_window_args(p):
  p.add_argument("--pre", type=float, default=DEFAULT_PRE_S)
  p.add_argument("--post", type=float, default=DEFAULT_POST_S)
  p.add_argument("--settle", type=float, default=DEFAULT_SETTLE_S)
  p.add_argument("--tilt-deg", type=float, default=DEFAULT_TILT_DEG)
  p.add_argument("--keep-windows", action="store_true")


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest="mode", required=True)
  add_args(sub.add_parser("run"))
  add_args(sub.add_parser("video"), video=True)
  m = sub.add_parser("metrics", help="Recompute metrics offline from a run's *_seam_meta.json")
  m.add_argument("meta", type=Path)
  m.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
  _add_window_args(m)
  args = p.parse_args()
  if args.mode == "metrics":
    recompute(args)
    return
  _setup_args(args)
  if args.mode == "run":
    if not 1 <= len(schedule_set(args.rates)) * args.envs_per_schedule <= 1024:
      p.error("keep 1..1024 envs on the shared GPU")
    run_seam(args)
  else:
    run_video(args)


if __name__ == "__main__":
  main()
