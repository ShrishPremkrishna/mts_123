"""Opt Lab 2 Part 2: capture a policy's own gait and fold it into a new reference.

Subcommands (run from ``pupper-mjlab/``)::

  capture  roll a checkpoint out at constant commands (default +1.5 and -1.5 m/s)
           with its own reference dir; record joint positions, the reference
           phase and foot contacts per env step (wraps ``pupper_locomotion
           evaluate``, which marks warmup/reset samples invalid).
  fold     phase-align steady-state cycles, reduce them to one representative
           cycle, resample to the 100-sample grid, enforce periodicity, optionally
           symmetrize, pin channels to the parent reference, and write
           ``captured_reach_gaits.npz`` + ``reference_manifest.json`` (the loader
           contract in ``gait_reference.py``) plus a diff report vs the parent.
  verify   load a reference dir through the real loader in a fresh process and
           check it round-trips (checksum, keys, limits, reach_back time order).

Fold variants (``--variant``):

  naive  fold on the commanded phase clock the policy observed.
  lag    shift the clock by the measured policy/actuator lag (cross-correlation
         of the naive fold against the parent reference; ``--lag-steps`` sets it
         explicitly, e.g. 1.8 = gait_tracking's phase_lead_steps).
  phase  re-time every cycle between consecutive touchdowns of one foot, anchored
         at that foot's mean touchdown phase (removes cycle-to-cycle jitter).
  sym    lag + left/right mirror averaging (the pair phase offset is measured).

The pin (default: back-leg abduction ``leg_back_r_2,leg_back_l_2``) holds chosen
channels at the parent reference's values. Without it the first recovered round
collapsed into a crouch: the captured near-constant channel had moved outside
the policy's input distribution. ``--pin none`` disables it.

Conventions: tables are (100, 12) in ``JOINT_NAMES`` order. ``reach_back`` is
stored in forward chronological order (indexed by the forward-running clock); the
loader reverses it once. Parent tables are converted to the same convention.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from mjlab.tasks.pupper_gait.bootstrap import (
  JOINT_ORDER,
  file_hash,
  periodic_smooth,
  save_references,
  validate_reference,
)

REPO = Path(__file__).resolve().parents[3]  # pupper-mjlab/
DRIVER_LIMITS = REPO.parent / "foundation/configs/class-driver-joint-limits.json"
STANDARD_PHYSICS = [
  "--deployment-target-clamp",
  "--compose-com-randomization",
  "--locomotion-torso-contact",
  "--driver-limits",
  str(DRIVER_LIMITS),
]
DEFAULT_PIN = ("leg_back_r_2", "leg_back_l_2")
GAIT_FREQUENCY = 1.0 / 0.75  # pupper_gait_env_cfg.GAIT_FREQUENCY
VARIANTS = {
  # name: (alignment, lag, symmetrize)
  "naive": ("clock", None, False),
  "lag": ("clock", "auto", False),
  "phase": ("touchdown", None, False),
  "sym": ("clock", "auto", True),
}
FOOT_NAMES = ("front_r", "front_l", "back_r", "back_l")


# --------------------------------------------------------------------------- #
# Pure NumPy core (unit tested)
# --------------------------------------------------------------------------- #


def contiguous_runs(mask: np.ndarray) -> list[tuple[int, int]]:
  """[start, end) index pairs of True runs in a 1-D boolean array."""
  m = np.concatenate([[False], np.asarray(mask, dtype=bool), [False]])
  edges = np.flatnonzero(np.diff(m.astype(np.int8)))
  return list(zip(edges[0::2].tolist(), edges[1::2].tolist(), strict=True))


def steady_mask(trace: dict, command, tolerance: float = 0.025) -> np.ndarray:
  """(T, E) samples at the constant command, past warmup, same episode, no fall.

  ``valid`` from ``pupper_locomotion evaluate`` already excludes the warmup after
  every command change and reset and the step that ends in a reset.
  """
  cmd = np.asarray(trace["command"], dtype=float)
  ok = np.array(trace["valid"], dtype=bool)  # copy: never mutate the trace
  ok &= np.abs(cmd - np.asarray(command, dtype=float)).max(axis=-1) <= tolerance
  if "terminated" in trace:
    ok &= ~np.asarray(trace["terminated"], dtype=bool)
  return ok


def clock_phase_runs(trace: dict, mask: np.ndarray):
  """Yield (env, start, end, unwrapped phase) for each steady run on the clock."""
  phase = np.asarray(trace["phase"], dtype=float)
  episode = np.asarray(trace["episode"])
  for env in range(phase.shape[1]):
    for a, b in contiguous_runs(mask[:, env]):
      # Split further on episode changes (belt and braces; valid already does).
      cuts = np.flatnonzero(np.diff(episode[a:b, env]) != 0) + 1 + a
      for lo, hi in zip([a, *cuts], [*cuts, b], strict=True):
        if hi - lo < 4:
          continue
        p = np.unwrap(phase[lo:hi, env] * 2 * np.pi) / (2 * np.pi)
        if np.any(np.diff(p) <= 0):
          continue
        yield env, lo, hi, p


def touchdowns(contact: np.ndarray, debounce: int = 2) -> np.ndarray:
  """Indices where a foot lands: >=debounce samples off, then >=debounce on."""
  c = np.asarray(contact, dtype=bool)
  idx = []
  for i in range(debounce, len(c) - debounce + 1):
    if (
      c[i]
      and not c[i - 1]
      and not c[i - debounce : i].any()
      and c[i : i + debounce].all()
    ):
      idx.append(i)
  return np.asarray(idx, dtype=int)


def touchdown_cadence(trace: dict, mask: np.ndarray, foot: int) -> float:
  """Median steps between touchdowns of ``foot`` over the steady runs."""
  contact = np.asarray(trace["foot_contact"], dtype=bool)[..., foot]
  gaps = [
    np.diff(touchdowns(contact[lo:hi, env]))
    for env, lo, hi, _ in clock_phase_runs(trace, mask)
  ]
  gaps = np.concatenate(gaps) if gaps else np.zeros(0)
  return float(np.median(gaps)) if len(gaps) else float("nan")


def touchdown_phase_runs(
  trace: dict, mask: np.ndarray, foot: int, cycle_steps: float, tol: float = 0.3
):
  """Yield (env, start, end, phase) re-timed between touchdowns of ``foot``.

  Within each touchdown-to-touchdown interval the phase advances linearly by
  exactly one cycle. The anchor (the phase assigned to touchdown) is the circular
  mean of the commanded clock at all touchdowns, so the variant removes per-cycle
  timing jitter without changing the average timing. Intervals whose length is
  outside (1 +- tol) x the commanded cycle are discarded (missed/double steps).
  """
  clock = np.asarray(trace["phase"], dtype=float)
  contact = np.asarray(trace["foot_contact"], dtype=bool)[..., foot]
  segments = []
  anchors = []
  for env, lo, hi, _ in clock_phase_runs(trace, mask):
    td = touchdowns(contact[lo:hi, env]) + lo
    if len(td) < 2:
      continue
    anchors.append(clock[td, env])
    segments.append((env, td))
  if not segments:
    return [], float("nan")
  ang = 2 * np.pi * np.concatenate(anchors)
  anchor = float(np.mod(np.angle(np.exp(1j * ang).mean()) / (2 * np.pi), 1.0))
  runs = []
  for env, td in segments:
    # Group consecutive acceptable intervals into continuous runs.
    good = [
      (a, b)
      for a, b in zip(td[:-1], td[1:], strict=True)
      if abs((b - a) / cycle_steps - 1) <= tol
    ]
    run: list[tuple[int, int]] = []
    for a, b in good + [(-1, -1)]:
      if run and a != run[-1][1]:
        lo, hi = run[0][0], run[-1][1] + 1
        p = np.empty(hi - lo)
        for k, (s, e) in enumerate(run):
          p[s - lo : e - lo + 1] = anchor + k + np.arange(e - s + 1) / (e - s)
        runs.append((env, lo, hi, p))
        run = []
      if a >= 0:
        run.append((a, b))
  return runs, anchor


def extract_cycles(q: np.ndarray, runs, samples: int = 100, shift: float = 0.0):
  """Interpolate every complete integer-phase cycle onto ``samples + 1`` points.

  ``runs`` yields (env, lo, hi, unwrapped phase); ``shift`` (cycles) is subtracted
  from the phase first, so the folded table at phase p holds q at clock p + shift.
  Returns (cycles (n, samples + 1, 12) including the closing point, origins).
  """
  cycles, origins = [], []
  for env, lo, hi, p in runs:
    p = np.asarray(p) - shift
    for k in range(int(np.ceil(p[0] + 1e-9)), int(np.floor(p[-1]))):
      grid = k + np.arange(samples + 1) / samples
      if grid[-1] > p[-1] + 1e-9:
        continue
      cycle = np.stack(
        [np.interp(grid, p, q[lo:hi, env, j]) for j in range(q.shape[2])], axis=1
      )
      if np.isfinite(cycle).all():
        cycles.append(cycle)
        origins.append((int(env), int(lo), int(hi), int(k)))
  if not cycles:
    return np.zeros((0, samples + 1, q.shape[2])), origins
  return np.stack(cycles), origins


def reduce_cycles(cycles: np.ndarray, how: str = "median") -> tuple[np.ndarray, dict]:
  """One representative cycle: pointwise ``mean``/``median`` or the ``medoid`` cycle."""
  if len(cycles) == 0:
    raise ValueError("No cycles to reduce")
  med = np.median(cycles, axis=0)
  if how == "median":
    rep = med
    info: dict = {}
  elif how == "mean":
    rep = cycles.mean(axis=0)
    info = {}
  elif how == "medoid":
    d = ((cycles - med) ** 2).mean(axis=(1, 2))
    i = int(np.argmin(d))
    rep = cycles[i]
    info = {"medoid_index": i}
  else:
    raise ValueError(f"Unknown reduction {how}")
  info["cycle_rms_dispersion_rad"] = float(np.sqrt(((cycles - rep) ** 2).mean()))
  info["per_channel_dispersion_rad"] = np.sqrt(
    ((cycles - rep) ** 2).mean(axis=(0, 1))
  ).tolist()
  return rep, info


def enforce_periodic(closed: np.ndarray, harmonics: int = 6) -> tuple[np.ndarray, dict]:
  """(samples + 1, 12) closed cycle -> (samples, 12) periodic table.

  The closing point is the next cycle's start. Any mismatch (slow drift during
  the cycle) is removed as a linear ramp, then the table is low-passed to
  ``harmonics`` Fourier harmonics (0 keeps every harmonic; the table is periodic
  by construction after the ramp removal either way).
  """
  closed = np.asarray(closed, dtype=float)
  n = len(closed) - 1
  mismatch = closed[-1] - closed[0]
  table = closed[:-1] - np.outer(np.arange(n) / n, mismatch)
  if harmonics > 0:
    table = periodic_smooth(table, harmonics)
  wrap_after = np.abs(
    (2 * table[-1] - table[-2]) - table[0]
  )  # 1-step extrapolation miss
  return table, dict(
    wrap_mismatch_before_rad=np.abs(mismatch).tolist(),
    max_wrap_mismatch_before_rad=float(np.abs(mismatch).max()),
    max_wrap_extrapolation_miss_after_rad=float(wrap_after.max()),
    harmonics=harmonics,
  )


def circular_shift(table: np.ndarray, shift: float) -> np.ndarray:
  """Fractional periodic shift: out[i] = table at phase i/n + shift (cycles)."""
  n = len(table)
  spectrum = np.fft.rfft(table, axis=0)
  k = np.arange(spectrum.shape[0])
  return np.fft.irfft(spectrum * np.exp(2j * np.pi * k * shift)[:, None], n=n, axis=0)


def measure_lag(
  captured: np.ndarray, parent: np.ndarray, max_shift: float = 0.25
) -> dict:
  """Phase lag (cycles) of the captured cycle behind the parent reference.

  Finds s minimizing ||captured(p) - parent(p - s)||^2 over |s| <= max_shift
  (mean-removed per channel, so posture offsets do not bias it), on a fine grid
  refined by a parabola. A positive lag means the robot follows the reference late.
  """
  c = captured - captured.mean(axis=0)
  par = parent - parent.mean(axis=0)
  shifts = np.linspace(
    -max_shift, max_shift, int(round(2 * max_shift * len(c) * 10)) + 1
  )
  cost = np.array([((c - circular_shift(par, -s)) ** 2).mean() for s in shifts])
  i = int(np.argmin(cost))
  s = float(shifts[i])
  if 0 < i < len(shifts) - 1:
    y0, y1, y2 = cost[i - 1 : i + 2]
    denom = y0 - 2 * y1 + y2
    if denom > 0:
      s += float(0.5 * (y0 - y2) / denom * (shifts[1] - shifts[0]))
  base = float((c**2).mean())
  return dict(
    lag_cycles=s,
    residual_rms_rad=float(np.sqrt(cost[i])),
    zero_shift_rms_rad=float(np.sqrt(cost[len(cost) // 2])),
    signal_rms_rad=float(np.sqrt(base)),
    at_search_edge=bool(i in (0, len(shifts) - 1)),
  )


def mirror_pairs(table: np.ndarray, shift: float) -> np.ndarray:
  """Left/right mirror average; left(t) = -right(t - shift) for every motor.

  Generalizes ``bootstrap.mirror_average`` to a fractional pair offset (0.5 for a
  diagonal trot; gallops differ). Signs follow the trot reference, whose left
  column is the negated right column.
  """
  out = np.asarray(table, dtype=float).copy()
  for r, left in ((0, 3), (6, 9)):
    a = out[:, r : r + 3]
    b = out[:, left : left + 3]
    mean = 0.5 * (a - circular_shift(b, shift))
    out[:, r : r + 3] = mean
    out[:, left : left + 3] = -circular_shift(mean, -shift)
  return out


def measure_mirror_shift(table: np.ndarray) -> dict:
  """Pair phase offset s minimizing ||right(t) + left(t + s)||^2 (both pairs)."""
  n = len(table)
  shifts = np.arange(n) / n
  cost = []
  for s in shifts:
    e = 0.0
    for r, left in ((0, 3), (6, 9)):
      e += (
        (table[:, r : r + 3] + circular_shift(table[:, left : left + 3], s)) ** 2
      ).mean()
    cost.append(e / 2)
  i = int(np.argmin(cost))
  return dict(shift_cycles=float(shifts[i]), asymmetry_rms_rad=float(np.sqrt(cost[i])))


def parent_stored_tables(reach: np.ndarray, reach_back_loaded: np.ndarray) -> dict:
  """Convert loader tables to stored (forward-chronological) convention.

  The loader reverses a stored ``reach_back`` with ``roll(x[::-1], 1)``, which is
  ``loaded[k] = stored[-k mod n]``; the inverse is the same index map.
  """
  n = len(reach_back_loaded)
  return dict(
    reach_fwd=np.asarray(reach),
    reach_back=np.asarray(reach_back_loaded)[(-np.arange(n)) % n],
  )


def pin_channels(table: np.ndarray, parent: np.ndarray, channels) -> np.ndarray:
  out = np.asarray(table, dtype=float).copy()
  for name in channels:
    j = JOINT_ORDER.index(name)
    out[:, j] = parent[:, j]
  return out


def diff_rows(new: np.ndarray, parent: np.ndarray, pinned=()) -> list[dict]:
  rows = []
  for j, name in enumerate(JOINT_ORDER):
    d = new[:, j] - parent[:, j]
    prange = float(np.ptp(parent[:, j]))
    rows.append(
      dict(
        channel=name,
        rms_diff_rad=float(np.sqrt((d**2).mean())),
        max_abs_diff_rad=float(np.abs(d).max()),
        mean_offset_rad=float(d.mean()),
        parent_range_rad=prange,
        new_range_rad=float(np.ptp(new[:, j])),
        pinned=name in pinned,
        # The crouch-collapse signature: a near-constant parent channel moved
        # by more than it ever varied. Pin it or justify it.
        flag_near_constant_shift=bool(
          prange < 0.2 and abs(float(d.mean())) > max(0.05, prange)
        ),
      )
    )
  return rows


# --------------------------------------------------------------------------- #
# Fold pipeline
# --------------------------------------------------------------------------- #


def fold_direction(
  trace: dict,
  command,
  parent: np.ndarray,
  *,
  align: str,
  lag,
  symmetrize: bool,
  reduce: str,
  harmonics: int,
  samples: int,
  min_cycles: int,
  frequency_mult: float,
  touchdown_foot: int,
  pin,
  touchdown_fallback: bool = False,
) -> tuple[np.ndarray, dict]:
  dt = float(trace["dt"])
  cycle_steps = 1.0 / (GAIT_FREQUENCY * frequency_mult * dt)
  q = np.asarray(trace["joint_pos"], dtype=float)
  mask = steady_mask(trace, command)
  report: dict = dict(
    command=list(map(float, command)),
    align=align,
    reduce=reduce,
    steady_samples=int(mask.sum()),
  )
  if align == "clock":
    runs = list(clock_phase_runs(trace, mask))
  elif align == "touchdown":
    cadence = touchdown_cadence(trace, mask, touchdown_foot)
    report.update(
      touchdown_foot=FOOT_NAMES[touchdown_foot],
      median_touchdown_interval_steps=cadence,
      cycle_steps=cycle_steps,
    )
    if not np.isfinite(cadence) or abs(cadence / cycle_steps - 1) > 0.3:
      msg = (
        f"{FOOT_NAMES[touchdown_foot]} lands every {cadence:.1f} steps at {command}, not once per "
        f"{cycle_steps:.1f}-step reference cycle: the policy does not step on the reference clock"
      )
      if not touchdown_fallback:
        raise ValueError(
          msg + " (pass --touchdown-fallback to fold this direction on the clock)"
        )
      report["touchdown_fallback"] = msg
      align = "clock"
      runs = list(clock_phase_runs(trace, mask))
    else:
      runs, anchor = touchdown_phase_runs(trace, mask, touchdown_foot, cycle_steps)
      report["touchdown_anchor_phase"] = anchor
  else:
    raise ValueError(f"Unknown alignment {align}")
  shift = 0.0
  if lag is not None:
    if lag == "auto":
      naive, _ = extract_cycles(q, runs, samples)
      if len(naive) < min_cycles:
        raise ValueError(
          f"Only {len(naive)} clean cycles at {command}; need {min_cycles}"
        )
      rep, _ = reduce_cycles(naive, reduce)
      table0, _ = enforce_periodic(rep, harmonics)
      info = measure_lag(table0, parent)
      shift = info["lag_cycles"]
      report["lag_measurement"] = info
      if info["at_search_edge"]:
        raise ValueError(
          f"Lag search hit its bound at {command}: captured gait does not match the parent; pass --lag-steps"
        )
    else:
      shift = float(lag) / cycle_steps
    report.update(
      lag_cycles=shift,
      lag_steps=shift * cycle_steps,
      lag_ms=shift * cycle_steps * dt * 1000,
    )
  # Robot lags the reference by `shift`: the new reference at phase p is where
  # the robot was at clock p + shift.
  cycles, origins = extract_cycles(q, runs, samples, shift=shift)
  if len(cycles) < min_cycles:
    raise ValueError(
      f"Only {len(cycles)} clean complete cycles at {command}; need {min_cycles}"
    )
  rep, info = reduce_cycles(cycles, reduce)
  table, periodic = enforce_periodic(rep, harmonics)
  report.update(
    cycles=len(cycles), envs=len({o[0] for o in origins}), periodicity=periodic, **info
  )
  if symmetrize:
    m = measure_mirror_shift(table)
    table = mirror_pairs(table, m["shift_cycles"])
    report["symmetrize"] = m
  if "velocity" in trace:
    v = np.asarray(trace["velocity"])[..., 0][mask]
    report["achieved_vx_mean_m_s"] = float(v.mean()) if v.size else None
  if pin:
    table = pin_channels(table, parent, pin)
  report["pinned_channels"] = list(pin)
  return table, report


_PARENT_CACHE: dict = {}


def load_parent(parent_dir: Path | None) -> dict:
  """Parent tables (stored convention), frequency multiplier and identity.

  Runs in a subprocess because ``gait_reference`` reads PUPPER_REFERENCE_DIR at
  import time. Tables are clamped to joint limits exactly like the training env.
  """
  code = (
    "import json,sys,numpy as np\n"
    "from mjlab.tasks.pupper_gait.mdp import gait_reference as g\n"
    "from mjlab.tasks.pupper_gait.bootstrap import robot_limits\n"
    "lim=robot_limits()\n"
    "out={k:np.clip(g.build_joint_reference_table(100,k),lim[:,0],lim[:,1]).tolist() for k in ('reach','reach_back','trot')}\n"
    "out['frequency_mult']=g._GAIT_FREQ_MULT.get('reach',1.0)\n"
    "out['captured']=g._CAPTURED_GAITS_FILE.exists(); out['dir']=str(g._REFERENCE_DIR)\n"
    "out['limits']=lim.tolist()\n"
    "json.dump(out,sys.stdout)\n"
  )
  files = {}
  if parent_dir is not None:
    files = {
      p.name: file_hash(p)
      for p in sorted(Path(parent_dir).iterdir())
      if p.suffix in (".npz", ".json")
    }
  key = (
    str(Path(parent_dir).resolve()) if parent_dir else None,
    tuple(sorted(files.items())),
  )
  if key in _PARENT_CACHE:
    return _PARENT_CACHE[key]
  env = dict(os.environ)
  env.pop("PUPPER_REFERENCE_DIR", None)
  if parent_dir is not None:
    env["PUPPER_REFERENCE_DIR"] = str(Path(parent_dir).resolve())
  res = subprocess.run(
    [sys.executable, "-c", code],
    env=env,
    capture_output=True,
    text=True,
    check=True,
    cwd=REPO,
  )
  data = json.loads(res.stdout)
  tables = parent_stored_tables(np.array(data["reach"]), np.array(data["reach_back"]))
  _PARENT_CACHE[key] = result = dict(
    tables=tables,
    frequency_mult=float(data["frequency_mult"]),
    captured=bool(data["captured"]),
    dir=str(parent_dir.resolve()) if parent_dir else None,
    description=(
      "captured reference dir"
      if data["captured"]
      else "shipped default (fast slot falls back to trot)"
    ),
    files=files,
    limits=np.array(data["limits"]),
  )
  return result


def write_diff_report(
  out: Path, tables: dict, parents: dict, reports: dict, pin
) -> dict:
  diffs = {
    name: diff_rows(tables[name], parents[name], pin)
    for name in ("reach_fwd", "reach_back")
  }
  lines = [f"# Reference diff: `{out.name}` vs parent", ""]
  for name, rows in diffs.items():
    lines += [
      f"## {name}",
      "",
      "| channel | rms diff | max abs diff | mean offset | parent range | new range | note |",
      "|---|---|---|---|---|---|---|",
    ]
    for r in rows:
      note = (
        "pinned"
        if r["pinned"]
        else (
          "**near-constant channel shifted**" if r["flag_near_constant_shift"] else ""
        )
      )
      lines.append(
        f"| {r['channel']} | {r['rms_diff_rad']:.3f} | {r['max_abs_diff_rad']:.3f} | {r['mean_offset_rad']:+.3f} | "
        f"{r['parent_range_rad']:.3f} | {r['new_range_rad']:.3f} | {note} |"
      )
    lines.append("")
  (out / "diff_report.md").write_text("\n".join(lines))
  try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 6, figsize=(20, 11), sharex=True)
    x = np.arange(len(tables["reach_fwd"])) / len(tables["reach_fwd"])
    for d, name in enumerate(("reach_fwd", "reach_back")):
      for j, ch in enumerate(JOINT_ORDER):
        ax = axes[j // 3, (j % 3) + 3 * d]
        ax.plot(x, parents[name][:, j], color="0.55", lw=1.5, label="parent")
        ax.plot(x, tables[name][:, j], color="C0", lw=1.5, label="new")
        ax.set_title(f"{name} {ch}", fontsize=8)
        ax.tick_params(labelsize=7)
    axes[0, 0].legend(fontsize=7)
    for ax in axes[-1]:
      ax.set_xlabel("phase (cycles)", fontsize=8)
    fig.suptitle(f"{out.name}: new reference (blue) vs parent (grey), rad", fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "diff.png", dpi=80)
    plt.close(fig)
  except Exception as exc:  # plotting is a convenience, never a gate
    print(f"plot skipped: {exc}", file=sys.stderr)
  return diffs


def fold_main(args) -> dict:
  t0 = time.time()
  capture = Path(args.capture)
  trace_path = capture / "trace.npz" if capture.is_dir() else capture
  with np.load(trace_path, allow_pickle=False) as f:
    trace = {k: f[k] for k in f.files}
  cap_meta = {}
  meta_path = trace_path.parent / "capture.json"
  if meta_path.exists():
    cap_meta = json.loads(meta_path.read_text())
  parent_dir = args.parent_ref_dir
  if parent_dir is None and cap_meta.get("reference_dir"):
    parent_dir = Path(cap_meta["reference_dir"])
  parent = load_parent(parent_dir)
  # The trace phase used the capture-time multiplier; the new reference keeps it.
  mult = (
    args.frequency_mult if args.frequency_mult is not None else parent["frequency_mult"]
  )
  align, lag, sym = VARIANTS[args.variant]
  if args.align:
    align = args.align
  if args.lag_steps is not None:
    lag = None if args.lag_steps == "none" else args.lag_steps
  if args.symmetrize is not None:
    sym = args.symmetrize
  pin: tuple[str, ...] = (
    () if args.pin.lower() == "none" else tuple(s for s in args.pin.split(",") if s)
  )
  unknown = [c for c in pin if c not in JOINT_ORDER]
  if unknown:
    raise ValueError(f"Unknown pin channels {sorted(unknown)}")
  tables, reports = {}, {}
  for name, sign in (("reach_fwd", 1.0), ("reach_back", -1.0)):
    tables[name], reports[name] = fold_direction(
      trace,
      [sign * args.speed, 0.0, 0.0],
      parent["tables"][name],
      align=align,
      lag=lag,
      symmetrize=sym,
      reduce=args.reduce,
      harmonics=args.harmonics,
      samples=args.samples,
      min_cycles=args.min_cycles,
      frequency_mult=mult,
      touchdown_foot=FOOT_NAMES.index(args.touchdown_foot),
      pin=pin,
      touchdown_fallback=args.touchdown_fallback,
    )
  limits = parent["limits"]
  clipped = {}
  for name, table in tables.items():
    over = np.maximum(limits[:, 0] - table, 0) + np.maximum(table - limits[:, 1], 0)
    if over.max() > 0:
      bad = {
        JOINT_ORDER[j]: float(over[:, j].max())
        for j in range(12)
        if over[:, j].max() > 0
      }
      if args.limit_policy == "error":
        raise ValueError(
          f"{name} exceeds joint limits {bad}; repair or pass --limit-policy clip"
        )
      tables[name] = np.clip(table, limits[:, 0], limits[:, 1])
      clipped[name] = bad
  out = Path(args.out)
  if out.exists() and any(out.iterdir()) and not args.force:
    raise FileExistsError(f"{out} is not empty (pass --force to overwrite)")
  meta = dict(
    method="part2_reference fold",
    variant=args.variant,
    align=align,
    lag=lag,
    symmetrize=sym,
    reduce=args.reduce,
    harmonics=args.harmonics,
    pin=list(pin),
    limit_policy=args.limit_policy,
    clipped=clipped,
    speed=args.speed,
    trace_sha256=file_hash(trace_path),
    capture=cap_meta,
    parent=dict(
      dir=parent["dir"],
      description=parent["description"],
      files=parent["files"],
      frequency_mult=parent["frequency_mult"],
    ),
    forward=reports["reach_fwd"],
    backward=reports["reach_back"],
  )
  manifest: dict = save_references(
    out,
    tables["reach_fwd"],
    tables["reach_back"],
    meta,
    limits=limits,
    frequency_mult=mult,
  )
  diffs = write_diff_report(out, tables, parent["tables"], reports, pin)
  summary = dict(
    out=str(out),
    variant=args.variant,
    frequency_mult=mult,
    cycles={k: r["cycles"] for k, r in reports.items()},
    lag_steps={k: r.get("lag_steps") for k, r in reports.items()},
    dispersion_rad={k: r["cycle_rms_dispersion_rad"] for k, r in reports.items()},
    wrap_before_rad={
      k: r["periodicity"]["max_wrap_mismatch_before_rad"] for k, r in reports.items()
    },
    peak_joint_velocity_rad_s={
      k: manifest["checks"][k]["peak_joint_velocity_rad_s"] for k in tables
    },
    rms_diff_vs_parent_rad={
      k: float(np.sqrt(np.mean([r["rms_diff_rad"] ** 2 for r in v])))
      for k, v in diffs.items()
    },
    flagged_channels={
      k: [r["channel"] for r in v if r["flag_near_constant_shift"]]
      for k, v in diffs.items()
    },
    reference_sha256=manifest["reference_sha256"],
    seconds=round(time.time() - t0, 1),
  )
  if not args.no_verify:
    summary["verify"] = verify_dir(out)
  (out / "fold_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
  print(json.dumps(summary, indent=2))
  return summary


# --------------------------------------------------------------------------- #
# Verify (real loader, fresh process)
# --------------------------------------------------------------------------- #


def verify_dir(ref_dir: Path) -> dict:
  ref_dir = Path(ref_dir).resolve()
  with np.load(ref_dir / "captured_reach_gaits.npz", allow_pickle=False) as f:
    stored = {k: f[k] for k in f.files}
  manifest = json.loads((ref_dir / "reference_manifest.json").read_text())
  required = {"reach_fwd", "reach_back", "joint_names", "metadata_json"}
  if not required <= set(stored):
    raise ValueError(f"npz missing keys {required - set(stored)}")
  if tuple(stored["joint_names"]) != tuple(JOINT_ORDER):
    raise ValueError("joint order differs")
  for key in ("version", "frequency_mult", "reference_sha256", "joint_order", "checks"):
    if key not in manifest:
      raise ValueError(f"manifest missing {key}")
  if file_hash(ref_dir / "captured_reach_gaits.npz") != manifest["reference_sha256"]:
    raise ValueError("checksum mismatch")
  parent = load_parent(ref_dir)  # imports the real loader with this dir
  if not parent["captured"]:
    raise ValueError("loader did not pick up the capture")
  checks = {}
  for name in ("reach_fwd", "reach_back"):
    err = float(
      np.abs(
        parent["tables"][name]
        - np.clip(stored[name], parent["limits"][:, 0], parent["limits"][:, 1])
      ).max()
    )
    if err > 1e-9:
      raise ValueError(f"{name} does not round-trip through the loader (max err {err})")
    checks[name] = validate_reference(
      stored[name], limits=parent["limits"], cycle_s=0.75 / manifest["frequency_mult"]
    )
  if abs(parent["frequency_mult"] - float(manifest["frequency_mult"])) > 1e-12:
    raise ValueError("loader frequency multiplier differs from manifest")
  return dict(
    ok=True,
    frequency_mult=parent["frequency_mult"],
    sha256=manifest["reference_sha256"],
    peak_joint_velocity_rad_s={
      k: v["peak_joint_velocity_rad_s"] for k, v in checks.items()
    },
  )


# --------------------------------------------------------------------------- #
# Capture (GPU; wraps pupper_locomotion evaluate)
# --------------------------------------------------------------------------- #


def parse_commands(text: str) -> list[list[float]]:
  """'1.5,-1.5' -> [[1.5,0,0],[-1.5,0,0]]; a JSON list of triples also works."""
  text = text.strip()
  if text.startswith("["):
    cmds = json.loads(text)
  elif Path(text).suffix == ".json" and Path(text).exists():
    cmds = json.loads(Path(text).read_text())
  else:
    cmds = [[float(v), 0.0, 0.0] for v in text.split(",") if v.strip()]
  arr = np.asarray(cmds, dtype=float)
  if arr.ndim != 2 or arr.shape[1] != 3 or not np.isfinite(arr).all():
    raise ValueError("commands must be speeds 'a,b' or a JSON list of [vx,vy,yaw]")
  return arr.tolist()


def summarize_capture(trace: dict, commands) -> list[dict]:
  """Steady-state coverage per command: samples, clean clock cycles, falls, speed."""
  dt = float(trace["dt"])
  rows = []
  for c in commands:
    mask = steady_mask(trace, c)
    cyc, _ = extract_cycles(
      np.asarray(trace["joint_pos"]), list(clock_phase_runs(trace, mask)), 20
    )
    seg = np.abs(np.asarray(trace["command"]) - np.asarray(c)).max(axis=-1) <= 1e-6
    rows.append(
      dict(
        command=list(map(float, c)),
        steady_samples=int(mask.sum()),
        steady_robot_seconds=float(mask.sum() * dt),
        clean_cycles=len(cyc),
        falls=int(np.asarray(trace["terminated"])[seg].sum()),
        achieved_vx_mean_m_s=float(np.asarray(trace["velocity"])[..., 0][mask].mean())
        if mask.any()
        else None,
      )
    )
  return rows


def capture_main(args) -> dict:
  out = Path(args.out).resolve()
  out.mkdir(parents=True, exist_ok=False)
  commands = parse_commands(args.commands)
  (out / "commands.json").write_text(json.dumps(commands) + "\n")
  cmd = [
    sys.executable, "-m", "mjlab.scripts.pupper_locomotion", "evaluate",
    "--checkpoint", str(Path(args.checkpoint).resolve()),
    "--out", str(out / "eval"),
    "--num-envs", str(args.num_envs), "--seconds", str(args.seconds), "--warmup", str(args.warmup),
    "--commands", str(out / "commands.json"), "--seed", str(args.seed), "--device", args.device,
  ]  # fmt: skip
  if args.reference_dir:
    cmd += ["--reference-dir", str(Path(args.reference_dir).resolve())]
  if not args.heading_hold:
    cmd += ["--no-heading-hold"]
  if not args.no_standard_physics:
    cmd += STANDARD_PHYSICS
  env = dict(os.environ)
  env.setdefault("MUJOCO_GL", "egl")
  env.setdefault("PYOPENGL_PLATFORM", "egl")
  env.pop("PUPPER_REFERENCE_DIR", None)
  t0 = time.time()
  with (out / "evaluate.log").open("w") as log:
    subprocess.run(
      cmd, cwd=REPO, env=env, stdout=log, stderr=subprocess.STDOUT, check=True
    )
  wall = time.time() - t0
  with np.load(out / "eval" / "trace.npz", allow_pickle=False) as f:
    full = {k: f[k] for k in f.files}
  keep = (
    "joint_pos",
    "phase",
    "episode",
    "command",
    "valid",
    "terminated",
    "timeout",
    "foot_contact",
    "velocity",
    "segment",
    "dt",
    "joint_names",
  )
  slim = {k: full[k] for k in keep if k in full}
  np.savez_compressed(out / "trace.npz", **slim)
  if not args.keep_full_trace:
    (out / "eval" / "trace.npz").unlink()
  dt = float(full["dt"])
  per_command = summarize_capture(slim, commands)
  prov = json.loads((out / "eval" / "provenance.json").read_text())
  meta = dict(
    checkpoint=str(Path(args.checkpoint).resolve()),
    checkpoint_sha256=prov.get("checkpoint_sha256"),
    reference_dir=str(Path(args.reference_dir).resolve())
    if args.reference_dir
    else None,
    reference_files=prov.get("references"),
    git_revision=prov.get("git_revision"),
    commands=commands,
    num_envs=args.num_envs,
    seconds_per_command=args.seconds,
    warmup_s=args.warmup,
    seed=args.seed,
    heading_hold=bool(args.heading_hold),
    standard_physics=not args.no_standard_physics,
    dt=dt,
    per_command=per_command,
    wall_clock_s=round(wall, 1),
  )
  (out / "capture.json").write_text(json.dumps(meta, indent=2) + "\n")
  print(json.dumps({k: meta[k] for k in ("per_command", "wall_clock_s")}, indent=2))
  return meta


def main() -> None:
  p = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
  )
  sub = p.add_subparsers(dest="command", required=True)
  cap = sub.add_parser("capture", help="roll out a checkpoint at constant commands")
  cap.add_argument("--checkpoint", type=Path, required=True)
  cap.add_argument(
    "--reference-dir",
    type=Path,
    help="the reference dir the checkpoint trained on (default: shipped)",
  )
  cap.add_argument(
    "--out", type=Path, required=True, help="run-output dir (not committed)"
  )
  cap.add_argument(
    "--commands", default="1.5,-1.5", help="speeds 'a,b,...' or JSON [[vx,vy,yaw],...]"
  )
  cap.add_argument("--num-envs", type=int, default=64)
  cap.add_argument("--seconds", type=float, default=15.0, help="per command")
  cap.add_argument(
    "--warmup",
    type=float,
    default=3.0,
    help="transient excluded after each command change/reset",
  )
  cap.add_argument("--seed", type=int, default=42)
  cap.add_argument("--device", default="cuda:0")
  cap.add_argument(
    "--heading-hold",
    action="store_true",
    help="keep the command term's heading hold (default off: constant command)",
  )
  cap.add_argument("--no-standard-physics", action="store_true")
  cap.add_argument("--keep-full-trace", action="store_true")
  fo = sub.add_parser("fold", help="fold a capture into a reference dir")
  fo.add_argument("capture", type=Path, help="capture dir (or trace.npz)")
  fo.add_argument(
    "out", type=Path, help="reference dir, e.g. configs/opt-lab-2/bootstrap/<name>"
  )
  fo.add_argument("--variant", choices=sorted(VARIANTS), default="lag")
  fo.add_argument(
    "--align", choices=("clock", "touchdown"), help="override the variant's alignment"
  )
  fo.add_argument(
    "--lag-steps",
    help="'auto', 'none' or control steps (e.g. 1.8); overrides the variant",
  )
  sym = fo.add_mutually_exclusive_group()
  sym.add_argument("--symmetrize", dest="symmetrize", action="store_true", default=None)
  sym.add_argument("--no-symmetrize", dest="symmetrize", action="store_false")
  fo.add_argument(
    "--pin",
    default=",".join(DEFAULT_PIN),
    help="channels held at the parent's values, or 'none'",
  )
  fo.add_argument(
    "--parent-ref-dir",
    type=Path,
    help="default: the capture's reference dir (shipped if none)",
  )
  fo.add_argument("--reduce", choices=("median", "mean", "medoid"), default="median")
  fo.add_argument(
    "--harmonics", type=int, default=6, help="Fourier harmonics kept (0 = no smoothing)"
  )
  fo.add_argument("--samples", type=int, default=100)
  fo.add_argument("--speed", type=float, default=1.5)
  fo.add_argument("--min-cycles", type=int, default=20)
  fo.add_argument("--frequency-mult", type=float, help="default: the parent's")
  fo.add_argument("--touchdown-foot", choices=FOOT_NAMES, default="front_r")
  fo.add_argument(
    "--touchdown-fallback",
    action="store_true",
    help="phase variant: fold on the clock where the foot does not land once per cycle",
  )
  fo.add_argument("--limit-policy", choices=("error", "clip"), default="error")
  fo.add_argument("--force", action="store_true")
  fo.add_argument("--no-verify", action="store_true")
  ve = sub.add_parser("verify", help="check a reference dir through the real loader")
  ve.add_argument("ref_dir", type=Path)
  args = p.parse_args()
  if args.command == "capture":
    capture_main(args)
  elif args.command == "fold":
    fold_main(args)
  else:
    print(json.dumps(verify_dir(args.ref_dir), indent=2))


if __name__ == "__main__":
  main()
