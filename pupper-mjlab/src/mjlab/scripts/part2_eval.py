"""Opt Lab 2 extended evaluation: scripted command suite, push recovery, comparison.

Simulation only. Three subcommands:

* ``suite``: 16 fixed command schedules x ``--envs-per-schedule`` robots in one
  batched play-config env (env i runs schedule i // envs_per_schedule). Falls are
  attributed to the command context of the step they happen in; tracking, jitter,
  torque saturation, foot force (in actual per-env body weights), heading drift,
  hind splay, slip and top speed come from steady windows.
* ``push``: instantaneous lateral (heading-perpendicular, world-frame) root
  velocity impulses of 0.5 and 1.0 m/s while standing and while moving; a push
  fails when the robot terminates within 2 s.
* ``compare``: one markdown table (+ CSV) over many suite JSONs, joined with the
  matching ``<name>_push.json`` when present.

``--reference-dir`` must be the reference directory the checkpoint trained on; it
is exported as ``PUPPER_REFERENCE_DIR`` before any task module is imported. The
standard physics flags of every run in this project are the default
(``--no-standard-physics`` opts out). The pure-NumPy metric functions are kept
separate from the GPU rollout so they can be unit tested on synthetic arrays.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import io
import json
import math
import os
import time
from pathlib import Path
from typing import Any, cast

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
STANDARD_DRIVER_LIMITS = (
  REPO_ROOT.parent / "foundation" / "configs" / "class-driver-joint-limits.json"
)
DEFAULT_TASK = "Mjlab-MixedGaits-Flat-Pupper-v3"
DEFAULT_OUT_DIR = REPO_ROOT / "report" / "part2" / "eval"

# Context categories, in priority order (reversal > stop > turn > steady).
CATEGORIES = ("steady", "turn", "stop", "reversal")
STEADY, TURN, STOP, REVERSAL = range(4)
CONTEXT_WINDOW_S = 2.0  # reversal/stop label lasts this long after the step.
STEADY_AFTER_S = 2.0  # steady window starts this long after a command change/reset.
FAST_SPEED = 1.5
EFFORT_SAT_FRACTION = 0.99
MIN_DRIFT_WINDOW_S = 1.0
TOP_SPEED_WINDOW_S = 1.0
FORCE_SETTLE_S = 1.0  # foot-force stats skip this long after each (re)set.
POST_RESET_FALL_S = 2.0  # falls this soon after a (re)set are also counted apart.

# Hind abduction ("leg roll", motor 2) joints and their outward sign. Measured with
# forward kinematics (tests/test_part2_eval.py::test_hind_splay_sign_matches_fk):
# +0.05 rad on leg_back_r_2 moves the right hind foot site by dy=-0.0067 m (away
# from the midline, the right feet sit at y<0); +0.05 rad on leg_back_l_2 moves the
# left hind foot by dy=-0.0067 m (toward the midline, left feet sit at y>0). So
# outward = +delta_q for the right leg and -delta_q for the left leg.
HIND_ABDUCTION_JOINTS = ("leg_back_r_2", "leg_back_l_2")
HIND_OUTWARD_SIGN = np.array([1.0, -1.0])

##
# Command schedules (versioned data; hashed into every output).
# ("hold", seconds, (vx, vy, wz)) or ("ramp", seconds, (from), (to)).
##

_S = (0.0, 0.0, 0.0)


def _x(v):
  return (float(v), 0.0, 0.0)


def _stops(v):
  segs = [("hold", 2.0, _S)]
  for _ in range(3):
    segs += [("ramp", 1.0, _S, _x(v)), ("hold", 2.0, _x(v)), ("hold", 2.5, _S)]
  return tuple(segs)


def _reversals(v):
  return (
    ("hold", 2.0, _S),
    ("ramp", 1.0, _S, _x(v)),
    ("hold", 4.0, _x(v)),
    ("hold", 4.0, _x(-v)),
    ("hold", 4.0, _x(v)),
    ("ramp", 2.0, _x(v), _S),
  )


def _steady(v):
  return (
    ("hold", 2.0, _S),
    ("ramp", 2.0, _S, _x(v)),
    ("hold", 14.0, _x(v)),
    ("ramp", 2.0, _x(v), _S),
  )


def _ramp(v):
  return (
    ("hold", 2.0, _S),
    ("ramp", 6.0, _S, _x(v)),
    ("hold", 4.0, _x(v)),
    ("ramp", 6.0, _x(v), _S),
  )


SCHEDULE_SECONDS = 20.0
SCHEDULES: tuple[tuple[str, tuple], ...] = (
  ("stand", (("hold", 20.0, _S),)),
  ("ramp_fwd", _ramp(1.5)),
  ("ramp_back", _ramp(-1.5)),
  ("steady_fwd", _steady(1.5)),
  ("steady_back", _steady(-1.5)),
  ("reversal_fwd_first", _reversals(1.5)),
  ("reversal_back_first", _reversals(-1.5)),
  ("stops_fwd", _stops(1.5)),
  ("stops_back", _stops(-1.5)),
  (
    "turn_in_place",
    (
      ("hold", 2.0, _S),
      ("hold", 7.0, (0.0, 0.0, 1.0)),
      ("hold", 2.0, _S),
      ("hold", 7.0, (0.0, 0.0, -1.0)),
    ),
  ),
  (
    "walk_and_turn",
    (
      ("hold", 2.0, _S),
      ("ramp", 1.0, _S, _x(0.5)),
      ("hold", 6.0, (0.5, 0.0, 0.8)),
      ("hold", 6.0, (0.5, 0.0, -0.8)),
      ("hold", 3.0, _x(0.5)),
      ("ramp", 1.0, _x(0.5), _S),
    ),
  ),
  (
    "sidestep",
    (
      ("hold", 2.0, _S),
      ("hold", 7.0, (0.0, 0.3, 0.0)),
      ("hold", 2.0, _S),
      ("hold", 7.0, (0.0, -0.3, 0.0)),
    ),
  ),
  (
    "slow_walk",
    (
      ("hold", 2.0, _S),
      ("hold", 7.0, _x(0.3)),
      ("hold", 2.0, _S),
      ("hold", 7.0, _x(-0.3)),
    ),
  ),
  (
    "gait_seam",
    (
      ("hold", 2.0, _S),
      ("ramp", 1.0, _S, _x(0.49)),
      ("hold", 2.5, _x(0.49)),
      ("hold", 2.5, _x(0.51)),
      ("hold", 2.5, _x(0.49)),
      ("ramp", 1.0, _x(0.49), _x(-0.49)),
      ("hold", 2.5, _x(-0.49)),
      ("hold", 2.5, _x(-0.51)),
      ("hold", 2.5, _x(-0.49)),
      ("ramp", 1.0, _x(-0.49), _S),
    ),
  ),
  (
    "reversal_moderate",
    (
      ("hold", 2.0, _S),
      ("ramp", 1.0, _S, _x(1.0)),
      ("hold", 3.0, _x(1.0)),
      ("hold", 3.0, _x(-1.0)),
      ("hold", 3.0, _x(1.0)),
      ("hold", 3.0, _x(-1.0)),
      ("ramp", 1.0, _x(-1.0), _S),
    ),
  ),
  (
    "mixed",
    (
      ("hold", 2.0, _S),
      ("ramp", 1.5, _S, _x(1.5)),
      ("hold", 2.0, _x(1.5)),
      ("hold", 1.5, _S),  # stop
      ("hold", 2.0, (0.0, 0.0, 1.0)),
      ("hold", 1.5, _x(0.5)),
      ("hold", 2.0, _x(-1.0)),  # sign flip from |0.5|: not a reversal
      ("hold", 2.0, _x(1.0)),  # reversal
      ("hold", 1.5, (0.0, 0.3, 0.0)),
      ("hold", 1.5, (0.0, 0.0, -1.0)),
    ),
  ),
)


def schedule_hash(schedules=SCHEDULES, dt=0.02, steps=None):
  payload = json.dumps(dict(schedules=schedules, dt=dt, steps=steps), sort_keys=True)
  return hashlib.sha256(payload.encode()).hexdigest()


def build_schedule(segments, dt, steps):
  """Per-step (steps, 3) command; ramps end exactly on their target; pad with stand."""
  rows = []
  for seg in segments:
    kind, seconds = seg[0], float(seg[1])
    n = int(round(seconds / dt))
    if n < 1 or abs(n * dt - seconds) > 1e-6:
      raise ValueError(f"Segment duration {seconds} is not a positive multiple of dt")
    if kind == "hold":
      rows.append(np.tile(np.asarray(seg[2], float), (n, 1)))
    elif kind == "ramp":
      a, b = np.asarray(seg[2], float), np.asarray(seg[3], float)
      frac = (np.arange(n) + 1.0) / n
      rows.append(a + (b - a) * frac[:, None])
    else:
      raise ValueError(f"Unknown segment kind {kind}")
  cmd = np.concatenate(rows) if rows else np.zeros((0, 3))
  if len(cmd) > steps:
    raise ValueError(f"Schedule needs {len(cmd)} steps > {steps}")
  return np.concatenate([cmd, np.zeros((steps - len(cmd), 3))])


def build_all_schedules(dt, steps, schedules=SCHEDULES):
  return np.stack([build_schedule(segs, dt, steps) for _, segs in schedules])


def check_ranges(schedule_cmd, ranges):
  """ranges: [(lo, hi)] for vx, vy, wz. Raise if any command leaves them."""
  for j, (lo, hi) in enumerate(ranges):
    v = schedule_cmd[..., j]
    if v.min() < lo - 1e-9 or v.max() > hi + 1e-9:
      raise ValueError(
        f"Command axis {j} spans [{v.min()}, {v.max()}] outside task range {(lo, hi)}"
      )


def label_schedule(cmd, dt, window_s=CONTEXT_WINDOW_S):
  """Context per step of one (T, 3) schedule, plus time since last command change.

  reversal: within window_s after a step whose vx flips sign with |vx|>=1 on both
  sides; stop: within window_s after a step from |vx|>=1 to an all-zero command;
  turn: vy or yaw nonzero; else steady (stand, straight walking, ramps).
  """
  cmd = np.asarray(cmd, float)
  T = len(cmd)
  prev = np.vstack([cmd[:1], cmd[:-1]])
  changed = np.any(np.abs(cmd - prev) > 1e-9, axis=1)
  changed[0] = True
  vx, pvx = cmd[:, 0], prev[:, 0]
  reversal_onset = (
    changed & (vx * pvx < 0) & (np.minimum(abs(vx), abs(pvx)) >= 1.0 - 1e-9)
  )
  stop_onset = changed & (abs(pvx) >= 1.0 - 1e-9) & np.all(abs(cmd) < 1e-9, axis=1)
  reversal_onset[0] = stop_onset[0] = False
  context = np.full(T, STEADY, dtype=np.int8)
  context[(abs(cmd[:, 1]) > 1e-9) | (abs(cmd[:, 2]) > 1e-9)] = TURN
  w = int(round(window_s / dt))
  for onsets, code in ((stop_onset, STOP), (reversal_onset, REVERSAL)):
    for k in np.flatnonzero(onsets):
      context[k : k + w] = code
  last = np.maximum.accumulate(np.where(changed, np.arange(T), 0))
  since_change = (np.arange(T) - last) * dt
  return dict(
    context=context,
    since_change=since_change,
    reversal_onset=reversal_onset,
    stop_onset=stop_onset,
  )


def episode_age(done, dt):
  """Seconds since the sample's episode began; done[k] => sample k+1 is fresh."""
  done = np.asarray(done, bool)
  age = np.zeros(done.shape, float)
  for k in range(1, len(done)):
    age[k] = np.where(done[k - 1], 0.0, age[k - 1] + dt)
  return age


def outward_splay(hind_delta):
  """(..., 2) hind abduction offsets from default [back_r_2, back_l_2] -> mean outward."""
  return (np.asarray(hind_delta) * HIND_OUTWARD_SIGN).mean(axis=-1)


def _true_runs(mask):
  """Start/stop (exclusive) of contiguous True runs in a 1-D mask."""
  m = np.concatenate([[False], np.asarray(mask, bool), [False]])
  d = np.diff(m.astype(np.int8))
  return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1), strict=True))


def _mean(x):
  x = np.asarray(x)
  return float(x.mean()) if x.size else None


def _f(x):
  return None if x is None else float(x)


def heading_drift(
  heading, mask, dt, terminated=None, min_window_s=MIN_DRIFT_WINDOW_S
) -> dict[str, Any]:
  """Per env, per contiguous masked run: |delta unwrapped heading| in deg per 10 s.

  A run that ends because the robot falls (terminated at the first sample after
  it) is dropped and counted: the pre-fall spin is a fall, not open-loop drift.
  """
  rates = []
  ending_in_fall = 0
  T = heading.shape[0]
  for e in range(heading.shape[1]):
    for a, b in _true_runs(mask[:, e]):
      if (b - a - 1) * dt < min_window_s - 1e-9:
        continue
      if terminated is not None and b < T and terminated[b, e]:
        ending_in_fall += 1
        continue
      h = np.unwrap(heading[a:b, e])
      rates.append(math.degrees(h[-1] - h[0]) / ((b - a - 1) * dt) * 10.0)
  signed = np.asarray(rates)
  rates = np.abs(signed).tolist()
  return dict(
    mean=_mean(rates),
    signed_mean=_mean(signed),  # + = counter-clockwise (left); cancels across envs
    max=float(max(rates)) if rates else None,
    windows=len(rates),
    windows_ending_in_fall=ending_in_fall,
  )


def top_speed(vx, alive, done, dt, window_s=TOP_SPEED_WINDOW_S) -> dict[str, Any]:
  """Max / min over envs of the window_s moving-average vx in reset-free windows."""
  T, N = vx.shape
  w = int(round(window_s / dt))
  if T < w:
    return dict(fwd=None, back=None)
  episode = np.vstack([np.zeros((1, N)), np.cumsum(done[:-1], axis=0)])
  csum = np.vstack([np.zeros((1, N)), np.cumsum(np.where(alive, vx, 0.0), axis=0)])
  cdead = np.vstack([np.zeros((1, N)), np.cumsum(~alive, axis=0)])
  avg = (csum[w:] - csum[:-w]) / w
  ok = (cdead[w:] - cdead[:-w] == 0) & (episode[w - 1 :] == episode[: T - w + 1])
  if not ok.any():
    return dict(fwd=None, back=None)
  per_env_max = np.where(ok, avg, -np.inf).max(axis=0)
  per_env_min = np.where(ok, avg, np.inf).min(axis=0)
  return dict(
    fwd=float(per_env_max.max()),
    back=float(per_env_min.min()),
    fwd_median_env=float(np.median(per_env_max[np.isfinite(per_env_max)])),
    back_median_env=float(np.median(per_env_min[np.isfinite(per_env_min)])),
  )


def compute_suite_metrics(
  *,
  schedule_cmd,
  env_schedule,
  velocity,
  joint_vel,
  torque,
  foot_force,
  foot_contact,
  foot_speed,
  heading,
  hind_delta,
  terminated,
  timeout,
  mass_kg,
  dt,
  gravity=9.81,
  effort_limit=3.0,
  schedule_names=None,
  steady_after_s=STEADY_AFTER_S,
) -> dict[str, Any]:
  """Pure-NumPy suite metrics.

  Shapes: schedule_cmd (S,T,3); env_schedule (N,); velocity (T,N,3) body
  [vx,vy,wz]; joint_vel/torque (T,N,12); foot_force/foot_contact/foot_speed
  (T,N,4); heading (T,N); hind_delta (T,N,2) [back_r_2, back_l_2] minus default;
  terminated/timeout (T,N) flags of the transition taken from sample k; mass_kg (N,).
  Samples are states observed before each step; sample k is "alive" unless its
  transition terminated. A steady sample is alive, >= steady_after_s after the
  last command change (ramps change every step) and after the episode began.
  """
  schedule_cmd = np.asarray(schedule_cmd, float)
  env_schedule = np.asarray(env_schedule, int)
  S, T, _ = schedule_cmd.shape
  N = len(env_schedule)
  terminated = np.asarray(terminated, bool)
  timeout = np.asarray(timeout, bool)
  done = terminated | timeout
  labels = [label_schedule(c, dt) for c in schedule_cmd]
  sched_ctx = np.stack([lab["context"] for lab in labels])  # (S,T)
  sched_since = np.stack([lab["since_change"] for lab in labels])
  cmd = np.transpose(schedule_cmd[env_schedule], (1, 0, 2))  # (T,N,3)
  ctx = sched_ctx[env_schedule].T
  since = sched_since[env_schedule].T
  age = episode_age(done, dt)
  alive = ~done
  eps = 1e-9
  steady = alive & (since >= steady_after_s - eps) & (age >= steady_after_s - eps)

  def at(v):
    return np.all(np.abs(cmd - np.asarray(v)) < 1e-6, axis=-1)

  fwd = steady & at((FAST_SPEED, 0, 0))
  back = steady & at((-FAST_SPEED, 0, 0))
  stand = steady & at((0, 0, 0))
  moving = steady & (np.linalg.norm(cmd, axis=-1) > 1e-9)
  fast = fwd | back

  # Falls and denominators.
  names = (
    list(schedule_names) if schedule_names is not None else [str(s) for s in range(S)]
  )
  falls_by_category = {
    c: int((terminated & (ctx == i)).sum()) for i, c in enumerate(CATEGORIES)
  }
  robot_runs = {
    c: int(sum(np.any(sched_ctx[s] == i) for s in env_schedule))
    for i, c in enumerate(CATEGORIES)
  }
  robot_seconds = {c: float((ctx == i).sum() * dt) for i, c in enumerate(CATEGORIES)}
  events = {
    "reversal": int(sum(labels[s]["reversal_onset"].sum() for s in env_schedule)),
    "stop": int(sum(labels[s]["stop_onset"].sum() for s in env_schedule)),
  }
  falls_by_schedule = {
    names[s]: int(terminated[:, env_schedule == s].sum()) for s in range(S)
  }
  # Decision (a): per-fall episode age and time since the last command change; falls
  # < 2 s after a (re)set are counted separately (categories unchanged).
  fall_k, fall_e = np.nonzero(terminated)
  falls_detail = [
    dict(
      schedule=names[env_schedule[e]],
      env=int(e),
      time_s=round(float(k * dt), 3),
      episode_age_s=round(float(age[k, e]), 3),
      since_command_change_s=round(float(since[k, e]), 3),
      category=CATEGORIES[int(ctx[k, e])],
    )
    for k, e in zip(fall_k, fall_e, strict=True)
  ]
  early = age[fall_k, fall_e] < POST_RESET_FALL_S - eps
  falls_soon_after_reset = int(early.sum())
  falls_soon_after_reset_by_category = {
    c: int((early & (ctx[fall_k, fall_e] == i)).sum()) for i, c in enumerate(CATEGORIES)
  }

  def track(mask):
    if not mask.any():
      return None, None, 0
    err = np.linalg.norm(velocity[..., :2][mask] - cmd[..., :2][mask], axis=-1)
    return float(err.mean()), float(velocity[..., 0][mask].mean()), int(mask.sum())

  err_f, vx_f, n_f = track(fwd)
  err_b, vx_b, n_b = track(back)

  thr = EFFORT_SAT_FRACTION * effort_limit
  sat = np.abs(torque) >= thr
  # Body weight per env: actual (randomized) total robot mass times g.
  bw = np.asarray(mass_kg, float) * gravity
  force_bw = np.asarray(foot_force, float) / bw[None, :, None]
  # Exclude the first FORCE_SETTLE_S after every (re)set: the play reset drops the
  # robot from its spawn height, an impact that is not policy behaviour.
  contact_all = (
    np.asarray(foot_contact, bool) & alive[..., None] & np.isfinite(force_bw)
  )
  contact = contact_all & (age >= FORCE_SETTLE_S - eps)[..., None]
  fc = force_bw[contact]
  fc_all = force_bw[contact_all]
  # Decision (c): ranked impact statistic = median over robot-runs of each run's max
  # (in-contact, alive, post-reset exclusion), robust to one-sample peaks.
  run_max = np.where(contact, force_bw, -np.inf).max(axis=(0, 2))
  run_max = run_max[np.isfinite(run_max)]
  peak_where = None
  if fc.size:
    k, e, foot = np.unravel_index(
      np.argmax(np.where(contact, force_bw, -np.inf)), force_bw.shape
    )
    peak_where = dict(
      schedule=names[env_schedule[e]],
      env=int(e),
      time_s=float(k * dt),
      episode_age_s=float(age[k, e]),
      foot=int(foot),
      context=CATEGORIES[int(ctx[k, e])],
      cmd=cmd[k, e].tolist(),
    )
  stand_force = force_bw[stand]
  speeds = np.asarray(foot_speed, float)

  def slip(mask):
    m = np.asarray(foot_contact, bool) & mask[..., None]
    return _mean(speeds[m])

  splay = outward_splay(hind_delta)
  drift_f = heading_drift(heading, fwd, dt, terminated)
  drift_b = heading_drift(heading, back, dt, terminated)
  top = top_speed(velocity[..., 0], alive, done, dt)

  result: dict[str, Any] = dict(
    robot_runs=N,
    falls_total=int(terminated.sum()),
    timeouts_total=int(timeout.sum()),
    falls_by_category=falls_by_category,
    falls_by_schedule=falls_by_schedule,
    falls_soon_after_reset=falls_soon_after_reset,
    falls_soon_after_reset_by_category=falls_soon_after_reset_by_category,
    falls_detail=falls_detail,
    category_robot_runs=robot_runs,
    category_robot_seconds=robot_seconds,
    category_robot_events=events,
    steady_samples=dict(
      fwd=n_f, back=n_b, stand=int(stand.sum()), moving=int(moving.sum())
    ),
    track_err_fwd=err_f,
    track_err_back=err_b,
    mean_vx_fwd=vx_f,
    mean_vx_back=vx_b,
    stand_jitter_rad_s=_mean(np.abs(joint_vel[stand])),
    torque_sat_frac=_mean(sat[alive]),
    torque_sat_frac_fast=_mean(sat[fast]),
    torque_abs_max_nm=float(np.abs(torque[alive]).max()) if alive.any() else None,
    torque_sq_mean=_mean(np.asarray(torque)[alive] ** 2),
    foot_force_peak_bw=float(fc.max()) if fc.size else None,
    foot_force_runmax_median_bw=float(np.median(run_max)) if run_max.size else None,
    foot_force_runmax_robot_runs=int(run_max.size),
    foot_force_p999_bw=float(np.percentile(fc, 99.9)) if fc.size else None,
    foot_force_p99_bw=float(np.percentile(fc, 99)) if fc.size else None,
    foot_force_peak_where=peak_where,
    foot_force_peak_bw_incl_reset_drop=float(fc_all.max()) if fc_all.size else None,
    foot_force_p99_bw_incl_reset_drop=(
      float(np.percentile(fc_all, 99)) if fc_all.size else None
    ),
    foot_force_stand_mean_bw_per_foot=_mean(stand_force),
    foot_force_stand_total_bw=_mean(stand_force.sum(-1)) if stand_force.size else None,
    heading_drift_deg_per_10s_fwd=drift_f,
    heading_drift_deg_per_10s_back=drift_b,
    hind_splay_rad_fwd=_mean(splay[fwd]),
    hind_splay_rad_back=_mean(splay[back]),
    hind_splay_rad_stand=_mean(splay[stand]),
    slip_m_s=slip(moving),
    slip_m_s_fwd=slip(fwd),
    slip_m_s_back=slip(back),
    top_speed_fwd=top["fwd"],
    top_speed_back=top["back"],
    top_speed_fwd_median_env=top.get("fwd_median_env"),
    top_speed_back_median_env=top.get("back_median_env"),
    mass_kg=dict(
      min=float(np.min(mass_kg)),
      mean=float(np.mean(mass_kg)),
      max=float(np.max(mass_kg)),
    ),
  )
  return result


##
# Push recovery (pure parts).
##

PUSH_CONDITIONS = (
  ("stand", 0.0),
  ("walk_+0.5", 0.5),
  ("run_+1.5", 1.5),
  ("run_-1.5", -1.5),
)
# report/decisions/2026-11-18-w2-eval-metrics.md (b): three rungs, 1.0 dropped
# (saturated standing); 128 envs per condition.
PUSH_MAGNITUDES = (0.5, 0.65, 0.8)
PUSH_ENVS_DEFAULT = 512
# Pre-registered standing rung (ranked), frozen by the 2026-11-18 baseline pilot:
# the rung of {0.65, 0.8} whose mean baseline standing falls/push lies in 5-60 %
# and is closest to 25 %. None = not yet frozen (ranked standing value absent).
# Pilot (4 runs: wandb-2999 and cont4500 x 2 repeats, 128 envs/condition): standing
# falls/push 0.65 -> 0.060, 0.069, 0.491, 0.515 (mean 0.284); 0.8 -> 0.576, 0.581,
# 0.969, 0.975 (mean 0.775). Frozen: 0.65.
PUSH_STANDING_RUNG: float | None = 0.65
PUSH_MOVING_RUNG = 0.5  # ranked moving rung, pooled over the moving conditions
PUSH_SECONDS = 30.0
PUSH_FIRST_S = 4.0  # stand 1 s, ramp 1.5 s, settle; first push no earlier.
PUSH_PERIOD_S = 3.0
PUSH_JITTER_S = 0.25
PUSH_WINDOW_S = 2.0


def push_commands(n_envs, dt, steps):
  """(T,N,3) commands: stand 1 s, ramp 1.5 s to the env's condition, hold."""
  cond = np.arange(n_envs) % len(PUSH_CONDITIONS)
  target = np.array([v for _, v in PUSH_CONDITIONS])[cond]
  t = np.arange(steps) * dt
  frac = np.clip((t - 1.0) / 1.5, 0.0, 1.0)
  cmd = np.zeros((steps, n_envs, 3))
  cmd[:, :, 0] = frac[:, None] * target[None, :]
  return cmd, cond


def make_push_schedule(n_envs, dt, steps, seed, window_s=PUSH_WINDOW_S):
  """Per env pushes every ~PUSH_PERIOD_S at a random offset; -1 pads unused slots.

  Returns step (N,P) int, magnitude (N,P), sign (N,P). Consecutive pushes are at
  least PUSH_PERIOD_S - 2*PUSH_JITTER_S apart; every push's window fits the run.
  """
  rng = np.random.default_rng(seed)
  total = steps * dt
  slots = int(math.ceil(total / PUSH_PERIOD_S)) + 1
  offset = rng.uniform(0.0, PUSH_PERIOD_S, size=(n_envs, 1))
  jitter = rng.uniform(-PUSH_JITTER_S, PUSH_JITTER_S, size=(n_envs, slots))
  times = PUSH_FIRST_S + offset + PUSH_PERIOD_S * np.arange(slots)[None] + jitter
  mag = rng.choice(np.asarray(PUSH_MAGNITUDES), size=(n_envs, slots))
  sign = rng.choice(np.array([-1.0, 1.0]), size=(n_envs, slots))
  step = np.round(times / dt).astype(int)
  valid = step + int(round(window_s / dt)) <= steps
  step = np.where(valid, step, -1)
  return step, np.where(valid, mag, 0.0), np.where(valid, sign, 0.0)


def rank_push(metrics, standing_rung=None) -> dict[str, Any]:
  """Ranked push readouts (decision (b)); every other push number is informational."""
  rung = PUSH_STANDING_RUNG if standing_rung is None else standing_rung
  by = metrics.get("by_condition") or {}
  stand = by.get("stand", {}).get(str(rung)) if rung is not None else None
  moving = [n for n, v in PUSH_CONDITIONS if v != 0.0]
  cells = [
    by[n][str(PUSH_MOVING_RUNG)]
    for n in moving
    if str(PUSH_MOVING_RUNG) in by.get(n, {})
  ]
  pushes = sum(c["pushes"] for c in cells)
  falls = sum(c["falls"] for c in cells)
  return dict(
    standing_rung=rung,
    standing_at_rung=stand,
    moving_rung=PUSH_MOVING_RUNG,
    moving_at_rung=dict(
      pushes=pushes, falls=falls, rate=falls / pushes if pushes else None
    ),
    moving_at_rung_by_condition={
      n: by[n].get(str(PUSH_MOVING_RUNG)) for n in moving if n in by
    },
  )


def compute_push_metrics(
  *,
  push_step,
  push_mag,
  env_condition,
  terminated,
  timeout,
  dt,
  condition_names=None,
  vx=None,
  magnitudes=PUSH_MAGNITUDES,
) -> dict[str, Any]:
  """Counted pushes: >= 2 s after a reset and after the previous push, window fits.
  A counted push fails if the env terminates within 2 s (steps k .. k+W-1).
  vx (T,N): body vx; achieved speed per condition = mean over alive samples
  >= PUSH_FIRST_S, >= 2 s into the episode and outside every 2 s post-push window."""
  terminated = np.asarray(terminated, bool)
  done = terminated | np.asarray(timeout, bool)
  T, N = terminated.shape
  w = int(round(PUSH_WINDOW_S / dt))
  age = episode_age(done, dt)
  names = condition_names or [n for n, _ in PUSH_CONDITIONS]
  env_condition = np.asarray(env_condition, int)
  rows = []
  covered = np.zeros((T, N), bool)
  for e in range(N):
    prev = None
    for k, m in zip(push_step[e], push_mag[e], strict=True):
      if k < 0:
        continue
      covered[k : k + w, e] = True
      counted = (
        k + w <= T
        and age[k, e] >= PUSH_WINDOW_S - 1e-9
        and (prev is None or (k - prev) * dt >= PUSH_WINDOW_S - 1e-9)
      )
      prev = k
      if counted:
        rows.append(
          (int(env_condition[e]), float(m), bool(terminated[k : k + w, e].any()))
        )
  rows_a = np.array(rows, dtype=float).reshape(-1, 3)

  def summary(mask):
    n = int(mask.sum())
    f = int(rows_a[mask, 2].sum())
    return dict(pushes=n, falls=f, rate=(f / n if n else None))

  cond, mag = rows_a[:, 0], rows_a[:, 1]
  standing_ids = [i for i, n in enumerate(names) if PUSH_CONDITIONS[i][1] == 0.0]
  is_stand = np.isin(cond, standing_ids)
  achieved = {}
  if vx is not None:
    vx = np.asarray(vx, float)
    t = np.arange(T)[:, None] * dt
    ok = ~done & (t >= PUSH_FIRST_S - 1e-9) & (age >= PUSH_WINDOW_S - 1e-9) & ~covered
    for i, n in enumerate(names):
      m = ok & (env_condition == i)[None, :]
      achieved[n] = _mean(vx[m])
  result: dict[str, Any] = dict(
    overall=summary(np.ones(len(rows_a), bool)),
    standing=summary(is_stand),
    moving=summary(~is_stand),
    by_magnitude={str(m): summary(mag == m) for m in magnitudes},
    by_condition={
      names[i]: dict(
        all=summary(cond == i),
        achieved_vx_m_s=achieved.get(names[i]),
        **{str(m): summary((cond == i) & (mag == m)) for m in magnitudes},
      )
      for i in range(len(names))
    },
    falls_total=int(terminated.sum()),
    falls_outside_push_windows=int((terminated & ~covered).sum()),
    timeouts_total=int(np.asarray(timeout, bool).sum()),
    robot_seconds=float(T * N * dt),
  )
  result["ranked"] = rank_push(result)
  return result


##
# Training metadata (what was this checkpoint trained with, and from what parent).
##

_MODEL_RE = r"model_(\d+)\.pt$"


def _iter_from_name(path):
  import re

  m = re.search(_MODEL_RE, str(path))
  return int(m.group(1)) if m else None


def checkpoint_iter(path):
  """(iteration, source): the checkpoint's stored 'iter', else model_N.pt's N."""
  path = Path(path)
  if path.is_file():
    try:
      import torch

      it = torch.load(path, map_location="cpu", weights_only=False).get("iter")
      if it is not None:
        return int(it), "checkpoint['iter']"
    except Exception:  # noqa: BLE001 - fall back to the file name
      pass
  it = _iter_from_name(path)
  return it, ("file name model_N.pt" if it is not None else None)


def _cli_value(args, flag):
  """Value after `flag` in a CLI arg list (also accepts --flag=value)."""
  for i, a in enumerate(args):
    if a == flag and i + 1 < len(args):
      return args[i + 1]
    if a.startswith(flag + "="):
      return a.split("=", 1)[1]
  return None


def _nested(d, *keys):
  for k in keys:
    if not isinstance(d, dict) or k not in d:
      return None
    d = d[k]
  return d


def read_training_metadata(checkpoint):
  """Training num_envs / seed / parent / reference dir from files beside a checkpoint.

  (a) ``provenance.json`` (+ ``params/{env,agent}.yaml``) written by
  ``pupper_locomotion train``; (b) a WandB ``config.yaml`` (CLI args list, then
  env_cfg/train_cfg values). Returns {field: (value, source)}; missing -> absent.
  """
  import yaml

  class _Loader(yaml.SafeLoader):
    """Safe loader for dump_yaml output (params/*.yaml): python tags become plain data."""

  def _plain(loader, _suffix, node):
    if isinstance(node, yaml.SequenceNode):
      return loader.construct_sequence(node, deep=True)
    if isinstance(node, yaml.MappingNode):
      return loader.construct_mapping(node, deep=True)
    return loader.construct_scalar(node)

  _Loader.add_multi_constructor("tag:yaml.org,2002:python/", _plain)

  def load(text):
    return yaml.load(text, Loader=_Loader)  # noqa: S506 (safe loader subclass)

  d = Path(checkpoint).resolve().parent
  out: dict[str, tuple[Any, str]] = {}

  def put(key, value, source):
    if value is not None and key not in out:
      out[key] = (value, source)

  prov = d / "provenance.json"
  if prov.is_file():
    p = json.loads(prov.read_text())
    src = f"{prov.name}"
    put("train_num_envs", p.get("num_envs"), src)
    put("train_seed", p.get("seed"), src)
    put("parent", p.get("checkpoint") or "", src)
    put("train_reference_dir", p.get("reference_dir") or "", src)
    put("train_iterations_arg", p.get("iterations"), src)
    put("train_git_revision", p.get("git_revision"), src)
    for name, keys, key in (
      ("env.yaml", ("scene", "num_envs"), "train_num_envs"),
      ("agent.yaml", ("seed",), "train_seed"),
    ):
      f = d / "params" / name
      if f.is_file():
        put(key, _nested(load(f.read_text()), *keys), f"params/{name}")
  cfg = d / "config.yaml"
  if cfg.is_file():
    c = load(cfg.read_text())
    runs = _nested(c, "_wandb", "value", "e") or {}
    args = next((r.get("args") for r in runs.values() if r.get("args")), None)
    if args:
      src = "config.yaml (wandb CLI args)"
      v = _cli_value(args, "--num-envs")
      put("train_num_envs", int(v) if v is not None else None, src)
      v = _cli_value(args, "--seed")
      put("train_seed", int(v) if v is not None else None, src)
      put("parent", _cli_value(args, "--checkpoint") or "", src)
      put("train_reference_dir", _cli_value(args, "--reference-dir") or "", src)
      v = _cli_value(args, "--iterations")
      put("train_iterations_arg", int(v) if v is not None else None, src)
    src = "config.yaml (cfg values)"
    put("train_num_envs", _nested(c, "env_cfg", "value", "scene", "num_envs"), src)
    put("train_seed", _nested(c, "train_cfg", "value", "seed"), src)
    commit = next((_nested(r, "git", "commit") for r in runs.values()), None)
    put("train_git_revision", commit, "config.yaml (wandb git commit)")
  return out


def training_block(checkpoint, eval_reference_dir=None, overrides=None):
  """JSON 'training' block with a *_source for every field.

  iters_past_parent counts PPO iterations run: iter + 1 from scratch (model_N.pt
  ran iterations 0..N) and iter - parent_iter + 1 for a warm start (rsl_rl resumes
  at the parent's stored iter, so a 1500-iteration leg from model_2999 saves
  model_4498). It is cross-checked against the recorded --iterations argument.
  """
  overrides = overrides or {}
  meta = read_training_metadata(checkpoint)
  block: dict[str, Any] = {"checkpoint": str(checkpoint)}

  def setf(key, value, source):
    block[key] = value
    block[key + "_source"] = source if value is not None else None

  it, it_src = checkpoint_iter(checkpoint)
  setf("iter", it, it_src)
  for key in ("train_num_envs", "train_seed"):
    if overrides.get(key) is not None:
      setf(key, int(overrides[key]), "cli override")
    else:
      setf(key, *meta.get(key, (None, None)))
  parent, parent_src = meta.get("parent", (None, None))
  parent = parent or None  # "" = recorded as no parent
  block["parent"] = parent
  block["parent_source"] = parent_src
  if overrides.get("parent_iter") is not None:
    setf("parent_iter", int(overrides["parent_iter"]), "cli override")
  elif parent:
    p_it, p_src = checkpoint_iter(parent)
    setf("parent_iter", p_it, ("parent " + p_src) if p_src else None)
  else:
    setf("parent_iter", None, None)
  if it is None:
    past, past_src = None, None
  elif block["parent_iter"] is not None:
    past, past_src = it - block["parent_iter"] + 1, "iter - parent_iter + 1"
  elif parent is None and parent_src is not None:
    past, past_src = it + 1, "from scratch: iter + 1"
  else:
    past, past_src = None, None  # parent unknown
  setf("iters_past_parent", past, past_src)
  arg = meta.get("train_iterations_arg", (None, None))[0]
  block["iters_past_parent_matches_iterations_arg"] = (
    None if past is None or arg is None else bool(past == arg)
  )
  ref, ref_src = meta.get("train_reference_dir", (None, None))
  block["train_reference_dir"] = ref or None
  block["train_reference_dir_source"] = ref_src
  block["train_iterations_arg"] = meta.get("train_iterations_arg", (None, None))[0]
  block["train_git_revision"] = meta.get("train_git_revision", (None, None))[0]
  # The policy observes its reference: flag evaluating with a different one.
  train_name = Path(ref).name if ref else None
  eval_name = Path(eval_reference_dir).name if eval_reference_dir else None
  if ref_src is None:
    check = "unknown (no training metadata)"
  elif train_name == eval_name:
    check = "ok" if train_name else "ok (default references)"
  else:
    check = f"MISMATCH: trained with {train_name or 'default'}, evaluated with {eval_name or 'default'}"
  block["reference_check"] = check
  return block


##
# Tables.
##


def _get(d, *keys):
  for k in keys:
    if not isinstance(d, dict) or k not in d or d[k] is None:
      return None
    d = d[k]
  return d


SUITE_COLUMNS = (
  ("falls", lambda r: r.get("falls_total")),
  ("rev", lambda r: r["falls_by_category"]["reversal"]),
  ("stop", lambda r: r["falls_by_category"]["stop"]),
  ("steady", lambda r: r["falls_by_category"]["steady"]),
  ("turn", lambda r: r["falls_by_category"]["turn"]),
  ("falls<2s_reset", lambda r: r.get("falls_soon_after_reset")),
  ("err+1.5", lambda r: r.get("track_err_fwd")),
  ("err-1.5", lambda r: r.get("track_err_back")),
  ("vx+1.5", lambda r: r.get("mean_vx_fwd")),
  ("vx-1.5", lambda r: r.get("mean_vx_back")),
  ("jitter", lambda r: r.get("stand_jitter_rad_s")),
  ("sat", lambda r: r.get("torque_sat_frac")),
  ("sat@1.5", lambda r: r.get("torque_sat_frac_fast")),
  ("F_runmax_med_BW", lambda r: r.get("foot_force_runmax_median_bw")),
  ("F_p999_BW", lambda r: r.get("foot_force_p999_bw")),
  ("F_p99_BW", lambda r: r.get("foot_force_p99_bw")),
  ("F_peak_BW(flag)", lambda r: r.get("foot_force_peak_bw")),
  ("drift+", lambda r: r["heading_drift_deg_per_10s_fwd"]["mean"]),
  ("drift-", lambda r: r["heading_drift_deg_per_10s_back"]["mean"]),
  ("splay+", lambda r: r.get("hind_splay_rad_fwd")),
  ("splay-", lambda r: r.get("hind_splay_rad_back")),
  ("slip", lambda r: r.get("slip_m_s")),
  ("top+", lambda r: r.get("top_speed_fwd")),
  ("top-", lambda r: r.get("top_speed_back")),
  ("tau2", lambda r: r.get("torque_sq_mean")),
)
_MOVING = tuple(n for n, v in PUSH_CONDITIONS if v != 0.0)


def _push_columns():
  """Ranked columns first (decision (b)); `(i)` = informational, never ranked.
  Ranking is recomputed here with the current frozen PUSH_STANDING_RUNG."""
  mr = str(PUSH_MOVING_RUNG)
  cols = [
    (
      f"R:push_stand@{PUSH_STANDING_RUNG}",
      lambda p: _get(rank_push(p), "standing_at_rung", "rate"),
    ),
    (f"R:push_move@{mr}", lambda p: _get(rank_push(p), "moving_at_rung", "rate")),
  ]
  for n in _MOVING:
    cols += [
      (f"vx_{n}", lambda p, n=n: _get(p, "by_condition", n, "achieved_vx_m_s")),
      (f"push@{mr}_{n}", lambda p, n=n: _get(p, "by_condition", n, mr, "rate")),
    ]
  cols += [
    (f"stand@{m}(i)", lambda p, m=m: _get(p, "by_condition", "stand", str(m), "rate"))
    for m in PUSH_MAGNITUDES
  ]
  cols += [
    ("push(i)", lambda p: _get(p, "overall", "rate")),
    ("push_stand(i)", lambda p: _get(p, "standing", "rate")),
    ("push_move(i)", lambda p: _get(p, "moving", "rate")),
  ]
  cols += [
    (f"push{m}(i)", lambda p, m=m: _get(p, "by_magnitude", str(m), "rate"))
    for m in PUSH_MAGNITUDES
  ]
  cols.append(("n_push", lambda p: _get(p, "overall", "pushes")))
  return tuple(cols)


def _fmt(v):
  if v is None:
    return "-"
  if isinstance(v, str):
    return v
  if isinstance(v, (bool, np.bool_)):
    return str(bool(v))
  if isinstance(v, (int, np.integer)):
    return str(int(v))
  v = float(v)
  if not math.isfinite(v):
    return str(v)
  return f"{v:.3g}" if abs(v) < 100 else f"{v:.0f}"


def _parent_label(parent):
  if not parent:
    return None
  p = Path(parent)
  return f"{p.parent.name}/{p.stem}"


def _eval_reference_label(payload):
  prov = (payload or {}).get("provenance", {})
  ref = prov.get("reference_dir")
  name = Path(ref).name if ref else "default"
  sha = (prov.get("effective_reference_sha256") or {}).get("captured_reach_gaits.npz")
  return f"{name}@{sha[:8]}" if sha else f"{name}(trot fallback)"


TRAIN_COLUMNS = (
  ("train_num_envs", lambda t, _: t.get("train_num_envs")),
  ("train_seed", lambda t, _: t.get("train_seed")),
  ("iter", lambda t, _: t.get("iter")),
  ("parent", lambda t, _: _parent_label(t.get("parent"))),
  ("iters_past_parent", lambda t, _: t.get("iters_past_parent")),
  ("reference", lambda _, payload: _eval_reference_label(payload)),
  ("ref_check", lambda t, _: t.get("reference_check")),
)
MATCH_FIELDS = ("train_num_envs", "iters_past_parent")


def table_rows(entries):
  """entries: [(name, suite_dict|None, push_dict|None)] -> header, rows (raw values)."""
  use_suite = any(s is not None for _, s, _ in entries)
  use_push = any(p is not None for _, _, p in entries)
  header = ["name"] + [c for c, _ in TRAIN_COLUMNS]
  if use_suite:
    header += [c for c, _ in SUITE_COLUMNS]
  if use_push:
    header += [c for c, _ in _push_columns()]
  rows = []
  for name, suite, push in entries:
    payload = suite or push or {}
    training = payload.get("training") or (push or {}).get("training") or {}
    row = [name] + [fn(training, payload) for _, fn in TRAIN_COLUMNS]
    if use_suite:
      metrics = suite.get("metrics", suite) if suite else None
      row += [fn(metrics) if metrics else None for _, fn in SUITE_COLUMNS]
    if use_push:
      metrics = push.get("metrics", push) if push else None
      row += [fn(metrics) if metrics else None for _, fn in _push_columns()]
    rows.append(row)
  return header, rows


def matched_check(header, rows):
  """{field: sorted distinct values}; a matched table needs one known value each."""
  return {f: sorted({r[header.index(f)] for r in rows}, key=str) for f in MATCH_FIELDS}


def markdown_table(header, rows):
  lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
  for row in rows:
    lines.append("| " + " | ".join([str(row[0])] + [_fmt(v) for v in row[1:]]) + " |")
  return "\n".join(lines) + "\n"


def csv_table(header, rows):
  buf = io.StringIO()
  writer = csv.writer(buf, lineterminator="\n")
  writer.writerow(header)
  for row in rows:
    writer.writerow(["" if v is None else v for v in row])
  return buf.getvalue()


LEGEND = (
  "Ranked columns (report/decisions/2026-11-18-w2-eval-metrics.md): falls and their "
  "categories, F_runmax_med_BW, R:push_stand@<frozen standing rung>, R:push_move@0.5 "
  "(pooled over the moving conditions); columns marked (i) and F_peak_BW(flag) are "
  "informational only. falls: terminations over the 384 robot-runs (rev/stop/steady/turn "
  "= context of the fall step; falls<2s_reset = falls < 2 s after a (re)set, also inside "
  "the categories); err/vx: mean |v_xy-cmd_xy| and mean body vx in steady +-1.5 m/s "
  "windows; jitter: mean |joint vel| (rad/s) standing; sat: fraction of joint-torque "
  "samples at >=0.99*3 Nm (all alive / steady +-1.5); F: per-foot contact force in "
  "actual body weights, in-contact samples >= 1 s after a reset: median over robot-runs "
  "of each run's max (ranked), p99.9, p99, single-sample peak (flag); drift: mean "
  "|heading change| deg/10 s in steady +-1.5 windows (heading hold off unless noted); "
  "splay: mean outward hind abduction (rad, + = feet away from midline) at +-1.5; slip: "
  "mean in-contact foot xy speed (m/s) in steady moving windows; top: best 1-s mean body "
  "vx; tau2: mean torque^2 (Nm^2); push: falls/push within 2 s; vx_<cond>: achieved mean "
  "body vx of that push condition outside push windows (moving push rates are confounded "
  "by achieved speed). Simulation only; GPU rollouts are not bit-reproducible."
)


class MismatchError(ValueError):
  pass


def compare(paths, out, push_dir=None, allow_mismatch=False):
  files = []
  for p in paths:
    p = Path(p)
    if p.is_dir():
      files += sorted(f for f in p.glob("*.json") if not f.name.endswith("_push.json"))
    else:
      files.append(p)
  entries = []
  for f in files:
    suite = json.loads(f.read_text())
    name = suite.get("name", f.stem)
    push_path = Path(push_dir or f.parent) / f"{name}_push.json"
    push = json.loads(push_path.read_text()) if push_path.exists() else None
    entries.append((name, suite, push))
  header, rows = table_rows(entries)
  distinct = matched_check(header, rows)
  bad = {f: v for f, v in distinct.items() if len(v) != 1 or None in v}
  if bad and not allow_mismatch:
    raise MismatchError(
      f"Refusing a matched table: {bad} (train num_envs and iterations past the parent "
      "must be known and identical; pass --allow-mismatch to tabulate anyway, marked)"
    )
  if bad:
    ref = rows[0]
    header = header[:1] + ["mismatch"] + header[1:]
    rows = [
      r[:1]
      + [
        ",".join(f for f in bad if r[header.index(f) - 1] != ref[header.index(f) - 1])
        or "-"
      ]
      + r[1:]
      for r in rows
    ]
    banner = (
      f"**UNMATCHED TABLE (--allow-mismatch)**: {bad}. Rows differ in training batch "
      f"size and/or iterations past the parent; do not read gaps as effects. The "
      f"`mismatch` column names the fields that differ from the first row ({rows[0][0]}).\n\n"
    )
  else:
    banner = (
      f"Matched table: train_num_envs={distinct['train_num_envs'][0]}, "
      f"iters_past_parent={distinct['iters_past_parent'][0]}.\n\n"
    )
  out = Path(out)
  out.parent.mkdir(parents=True, exist_ok=True)
  out.write_text(banner + markdown_table(header, rows) + "\n" + LEGEND + "\n")
  out.with_suffix(".csv").write_text(csv_table(header, rows))
  return header, rows


##
# GPU rollout.
##

_COMMAND_FLAGS = (
  "is_standing_env",
  "is_heading_env",
  "is_world_env",
  "is_turn_env",
  "is_forward_env",
)


def _reference_hashes(directory):
  from mjlab.tasks.pupper_gait.bootstrap import file_hash

  directory = Path(directory)
  hashes: dict[str, Any] = {
    p.name: file_hash(p)
    for p in sorted(directory.iterdir())
    if p.suffix in (".npz",) or p.name == "reference_manifest.json"
  }
  if "captured_reach_gaits.npz" not in hashes:
    # gait_reference: without a capture the fast slots fall back to the trot.
    hashes["captured_reach_gaits.npz"] = None
  return hashes


def _git_state():
  import subprocess

  def git(*a):
    return subprocess.check_output(
      ["git", "-C", str(REPO_ROOT), *a], text=True, stderr=subprocess.DEVNULL
    ).strip()

  try:
    return git("rev-parse", "HEAD"), bool(git("status", "--porcelain", "--", "src"))
  except (OSError, subprocess.CalledProcessError):
    return None, None


def _apply_physics_defaults(args):
  args.standard_physics = not args.no_standard_physics
  if args.standard_physics:
    args.deployment_target_clamp = True
    args.compose_com_randomization = True
    args.locomotion_torso_contact = True
    if args.driver_limits is None:
      args.driver_limits = STANDARD_DRIVER_LIMITS
  if args.driver_limits is not None:
    args.driver_limits = Path(args.driver_limits).resolve()
    if not args.driver_limits.is_file():
      raise FileNotFoundError(f"driver limits JSON not found: {args.driver_limits}")


class _Rollout:
  """Play-config env with a frozen, per-env, per-step command."""

  def __init__(self, args, num_envs, steps):
    import torch

    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.scripts.pupper_locomotion import provenance, settings
    from mjlab.tasks.registry import load_runner_cls
    from mjlab.utils.torch import configure_torch_backends

    self.torch = torch
    s_args = copy.copy(args)
    s_args.num_envs = num_envs
    s_args.command = "evaluate"  # encoder-observation purpose, as pupper_locomotion
    cfg, agent = settings(s_args, play=True)
    cfg.events.pop("hold_base_airborne", None)
    cfg.curriculum.pop("pretrain_rewards", None)
    term_cfg: Any = cfg.commands["twist"]
    term_cfg.resampling_time_range = (1e8, 1e8)
    for name in (
      "rel_standing_envs",
      "rel_heading_envs",
      "rel_world_envs",
      "rel_forward_envs",
      "rel_turn_envs",
      "init_velocity_prob",
    ):
      if hasattr(term_cfg, name):
        setattr(term_cfg, name, 0.0)
    term_cfg.heading_command = False
    if not args.heading_hold:
      term_cfg.heading_hold_kp = None
    self.ranges = [
      tuple(term_cfg.ranges.lin_vel_x),
      tuple(term_cfg.ranges.lin_vel_y),
      tuple(term_cfg.ranges.ang_vel_z),
    ]
    self.dt = cfg.sim.mujoco.timestep * cfg.decimation
    cfg.episode_length_s = max(cfg.episode_length_s, steps * self.dt + 10.0)
    self.cfg, self.agent, self.args = cfg, agent, s_args
    self.num_envs, self.steps, self.device = num_envs, steps, args.device
    self.run = provenance(s_args)
    # provenance() asks git in the cwd; pin it to this repository instead.
    self.run["git_revision"], self.run["git_src_dirty"] = _git_state()
    from mjlab.tasks.pupper_gait.mdp import gait_reference

    self.run["effective_reference_dir"] = str(gait_reference._REFERENCE_DIR)
    self.run["effective_reference_sha256"] = _reference_hashes(
      gait_reference._REFERENCE_DIR
    )
    self.run["heading_hold"] = bool(args.heading_hold)
    self.run["episode_length_s"] = cfg.episode_length_s
    configure_torch_backends()
    self.env = ManagerBasedRlEnv(cfg, device=args.device)
    self.wrapped = RslRlVecEnvWrapper(self.env, clip_actions=agent.clip_actions)
    from dataclasses import asdict

    runner_cls = cast(Any, load_runner_cls(args.task))
    runner = runner_cls(self.wrapped, asdict(agent), device=args.device)
    runner.load(
      str(args.checkpoint),
      load_cfg={"actor": True},
      strict=True,
      map_location=args.device,
    )
    self.policy = runner.get_inference_policy(device=args.device)
    self.robot = self.env.scene["robot"]
    self.term = cast(Any, self.env.command_manager.get_term("twist"))
    self.pending = torch.zeros(num_envs, 3, device=args.device)
    original = self.term._resample_command

    def resample_to_schedule(env_ids):
      # A reset resamples a random command; replace it with the scheduled one so
      # the post-reset observation already carries the scheduled command.
      original(env_ids)
      self.term.vel_command_b[env_ids] = self.pending[env_ids]
      self.term.vel_command_w[env_ids] = self.pending[env_ids]
      for flag in _COMMAND_FLAGS:
        if hasattr(self.term, flag):
          getattr(self.term, flag)[env_ids] = False

    self.term._resample_command = resample_to_schedule

  def set_command(self, cmd):
    self.pending.copy_(cmd)
    self.term.vel_command_b[:] = cmd
    self.term.vel_command_w[:] = cmd
    self.term.time_left.fill_(1e8)
    for flag in _COMMAND_FLAGS:
      if hasattr(self.term, flag):
        getattr(self.term, flag).fill_(False)
    self.term.compute(0.0)

  def masses(self):
    """Per-env total robot mass from the per-world model (domain randomized)."""
    body_ids = self.robot.indexing.body_ids
    field = self.env.sim.model.body_mass
    nominal = float(
      np.asarray(self.env.sim.mj_model.body_mass)[body_ids.cpu().numpy()].sum()
    )
    if field.ndim == 2 and field.shape[0] == self.num_envs:
      mass = field[:, body_ids].sum(-1).detach().cpu().numpy().astype(float)
      source = "env.sim.model.body_mass[world, robot.indexing.body_ids] (per-world)"
    else:
      mass = np.full(self.num_envs, nominal)
      source = "nominal compiled mj_model.body_mass (per-world field unavailable)"
    g = float(np.linalg.norm(np.asarray(self.env.sim.mj_model.opt.gravity)))
    return mass, nominal, g, source

  def actuator_limits(self):
    m = self.env.sim.mj_model
    ids = [i for i in range(m.nu)]
    return dict(
      actuator_forcelimited=[int(v) for v in np.asarray(m.actuator_forcelimited)[ids]],
      actuator_forcerange=np.asarray(m.actuator_forcerange)[ids].tolist(),
      jnt_actfrclimited=[int(v) for v in np.asarray(m.jnt_actfrclimited)],
    )

  def run_loop(self, cmd_tn, record=None, pushes=None):
    """cmd_tn: (T,N,3) tensor. The policy observing sample k sees cmd[k] (the
    command is written before the step whose post-step observation feeds k)."""
    torch = self.torch
    T = cmd_tn.shape[0]
    terminated = torch.zeros(T, self.num_envs, dtype=torch.bool, device=self.device)
    timeout = torch.zeros_like(terminated)
    self.set_command(cmd_tn[0])
    obs, _ = self.wrapped.reset()
    with torch.inference_mode():
      for k in range(T):
        if record is not None:
          record(k)
        actions = self.policy(obs)
        if pushes is not None:
          pushes(k)
        self.set_command(cmd_tn[min(k + 1, T - 1)])
        obs, _, _, _ = self.wrapped.step(actions)
        terminated[k] = self.env.reset_terminated
        timeout[k] = self.env.reset_time_outs
    return terminated.cpu().numpy(), timeout.cpu().numpy()

  def close(self):
    self.env.close()


def _write(out_dir, stem, payload, md):
  out_dir = Path(out_dir)
  out_dir.mkdir(parents=True, exist_ok=True)
  (out_dir / f"{stem}.json").write_text(json.dumps(payload, indent=2) + "\n")
  (out_dir / f"{stem}.md").write_text(md)


def run_suite(args):
  import torch

  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import (
    FOOT_SITE_NAMES,
    JOINT_NAMES,
    PUPPER_EFFORT_LIMIT,
  )

  t0 = time.time()
  dt_nominal = 0.02
  steps = int(round(SCHEDULE_SECONDS / dt_nominal))
  S = len(SCHEDULES)
  n = S * args.envs_per_schedule
  roll = _Rollout(args, n, steps)
  dt = roll.dt
  if abs(dt - dt_nominal) > 1e-9:
    raise ValueError(f"Schedules assume dt={dt_nominal}, env has {dt}")
  schedule_cmd = build_all_schedules(dt, steps)
  check_ranges(schedule_cmd, roll.ranges)
  env_schedule = np.arange(n) // args.envs_per_schedule
  cmd_tn = torch.tensor(
    np.transpose(schedule_cmd[env_schedule], (1, 0, 2)),
    dtype=torch.float32,
    device=args.device,
  )
  robot = roll.robot
  joint_ids = [robot.joint_names.index(j) for j in JOINT_NAMES]
  hind_ids = [robot.joint_names.index(j) for j in HIND_ABDUCTION_JOINTS]
  site_ids, _ = robot.find_sites(FOOT_SITE_NAMES, preserve_order=True)
  contact = roll.env.scene["feet_ground_contact"]
  dev = args.device

  def buf(*shape, dtype=torch.float32):
    return torch.zeros(steps, n, *shape, dtype=dtype, device=dev)

  bufs = dict(
    velocity=buf(3),
    joint_vel=buf(12),
    torque=buf(12),
    foot_force=buf(4),
    foot_contact=buf(4, dtype=torch.bool),
    foot_speed=buf(4),
    heading=buf(),
    hind_delta=buf(2),
  )
  data = robot.data

  def record(k):
    bufs["velocity"][k] = torch.cat(
      [data.root_link_lin_vel_b[:, :2], data.root_link_ang_vel_b[:, 2:3]], dim=1
    )
    bufs["joint_vel"][k] = data.joint_vel[:, joint_ids]
    bufs["torque"][k] = data.qfrc_actuator[:, joint_ids]
    force = contact.data.force
    if force is None:
      raise ValueError("feet_ground_contact sensor has no force field")
    bufs["foot_force"][k] = torch.linalg.vector_norm(force, dim=-1)
    bufs["foot_contact"][k] = contact.data.found > 0
    bufs["foot_speed"][k] = torch.linalg.vector_norm(
      data.site_lin_vel_w[:, site_ids, :2], dim=-1
    )
    bufs["heading"][k] = data.heading_w
    bufs["hind_delta"][k] = (
      data.joint_pos[:, hind_ids] - data.default_joint_pos[:, hind_ids]
    )

  mass, nominal, g, mass_source = roll.masses()
  t_loop = time.time()
  try:
    terminated, timeout = roll.run_loop(cmd_tn, record=record)
    arrays: dict[str, Any] = {k: v.cpu().numpy() for k, v in bufs.items()}
    limits = roll.actuator_limits()
  finally:
    roll.close()
  loop_s = time.time() - t_loop
  metrics = compute_suite_metrics(
    schedule_cmd=schedule_cmd,
    env_schedule=env_schedule,
    terminated=terminated,
    timeout=timeout,
    mass_kg=mass,
    dt=dt,
    gravity=g,
    effort_limit=PUPPER_EFFORT_LIMIT,
    schedule_names=[s for s, _ in SCHEDULES],
    **arrays,
  )
  metrics["mass_source"] = mass_source
  metrics["mass_nominal_kg"] = nominal
  metrics["gravity_m_s2"] = g
  metrics["torque_model"] = dict(
    effort_limit_nm=PUPPER_EFFORT_LIMIT,
    saturation_threshold_nm=EFFORT_SAT_FRACTION * 3.0,
    **limits,
  )
  wall = time.time() - t0
  payload = dict(
    name=args.name,
    kind="suite",
    metrics=metrics,
    wall_clock_s=wall,
    rollout_loop_s=loop_s,
    schedule=dict(
      names=[s for s, _ in SCHEDULES],
      sha256=schedule_hash(SCHEDULES, dt, steps),
      steps=steps,
      dt=dt,
      envs_per_schedule=args.envs_per_schedule,
      context_window_s=CONTEXT_WINDOW_S,
      steady_after_s=STEADY_AFTER_S,
    ),
    provenance=roll.run,
    training=args.training,
  )
  header, rows = table_rows([(args.name, payload, None)])
  _write(args.out_dir, args.name, payload, markdown_table(header, rows))
  if args.raw_dir:
    raw = Path(args.raw_dir)
    raw.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
      raw / f"{args.name}_suite_summary.npz",
      env_schedule=env_schedule,
      mass_kg=mass,
      terminated=terminated,
      timeout=timeout,
      vx=arrays["velocity"][..., 0].astype(np.float16),
      heading=arrays["heading"].astype(np.float32),
    )
  print(
    json.dumps(
      dict(
        name=args.name,
        wall_clock_s=wall,
        **{k: v for k, v in metrics.items() if not isinstance(v, dict)},
      ),
      indent=2,
    )
  )
  return payload


def run_push(args):
  import torch

  t0 = time.time()
  dt_nominal = 0.02
  steps = int(round(PUSH_SECONDS / dt_nominal))
  n = args.envs
  roll = _Rollout(args, n, steps)
  dt = roll.dt
  cmd, cond = push_commands(n, dt, steps)
  check_ranges(cmd, roll.ranges)
  push_step, push_mag, push_sign = make_push_schedule(n, dt, steps, args.seed)
  cmd_tn = torch.tensor(cmd, dtype=torch.float32, device=args.device)
  by_step = {}
  for e in range(n):
    for k, m, s in zip(push_step[e], push_mag[e], push_sign[e], strict=True):
      if k >= 0:
        by_step.setdefault(int(k), []).append((e, float(m * s)))
  robot = roll.robot
  dev = args.device
  checks = []  # (step, env, signed magnitude, lateral v before, lateral v next sample)
  pending_check = {}

  def lateral(ids, h):
    v = robot.data.root_link_vel_w[ids, :2]
    return -torch.sin(h) * v[:, 0] + torch.cos(h) * v[:, 1]

  def pushes(k):
    if k not in by_step:
      return
    ids = torch.tensor([e for e, _ in by_step[k]], dtype=torch.long, device=dev)
    dv = torch.tensor([v for _, v in by_step[k]], dtype=torch.float32, device=dev)
    vel = robot.data.root_link_vel_w[ids].clone()  # (lin_w, ang_w)
    h = robot.data.heading_w[ids]
    pending_check[k] = (ids, h, dv, lateral(ids, h))
    # Heading-perpendicular world-frame direction (robot's left for +dv).
    vel[:, 0] += -torch.sin(h) * dv
    vel[:, 1] += torch.cos(h) * dv
    robot.write_root_link_velocity_to_sim(vel, env_ids=ids)

  vx_buf = torch.zeros(steps, n, dtype=torch.float32, device=dev)

  def record(k):
    vx_buf[k] = robot.data.root_link_lin_vel_b[:, 0]
    # Self-check: lateral velocity change one control step after each push.
    if k - 1 in pending_check:
      ids, h, dv, before = pending_check.pop(k - 1)
      after = lateral(ids, h)
      for row in zip(
        ids.tolist(), dv.tolist(), before.tolist(), after.tolist(), strict=True
      ):
        checks.append((k - 1, *row))

  try:
    terminated, timeout = roll.run_loop(cmd_tn, record=record, pushes=pushes)
    vx = vx_buf.cpu().numpy()
  finally:
    roll.close()
  check = np.array(checks, dtype=float).reshape(-1, 5)
  keep = ~terminated[check[:, 0].astype(int), check[:, 1].astype(int)]
  gain = (check[:, 4] - check[:, 3]) * np.sign(check[:, 2])
  push_check = {
    str(m): dict(
      mean_lateral_dv_after_one_step_m_s=_mean(
        gain[keep & np.isclose(np.abs(check[:, 2]), m, atol=1e-4)]
      ),
      pushes=int((keep & np.isclose(np.abs(check[:, 2]), m, atol=1e-4)).sum()),
    )
    for m in PUSH_MAGNITUDES
  }
  metrics = compute_push_metrics(
    push_step=push_step,
    push_mag=push_mag,
    env_condition=cond,
    terminated=terminated,
    timeout=timeout,
    dt=dt,
    vx=vx,
  )
  wall = time.time() - t0
  payload = dict(
    name=args.name,
    kind="push",
    metrics=metrics,
    wall_clock_s=wall,
    design=dict(
      conditions=[dict(name=nm, vx=v) for nm, v in PUSH_CONDITIONS],
      envs_per_condition=int(np.bincount(cond).min()),
      magnitudes_m_s=list(PUSH_MAGNITUDES),
      seconds=PUSH_SECONDS,
      first_push_s=PUSH_FIRST_S,
      period_s=PUSH_PERIOD_S,
      jitter_s=PUSH_JITTER_S,
      window_s=PUSH_WINDOW_S,
      mechanism="root_link_vel_w += sign*mag*(-sin h, cos h, 0) via "
      "Entity.write_root_link_velocity_to_sim, applied after the policy acts and "
      "before the step (same mechanism as mjlab.envs.mdp.events.push_by_setting_velocity)",
      scheduled_pushes=int((push_step >= 0).sum()),
      applied_impulse_check=push_check,
    ),
    provenance=roll.run,
    training=args.training,
  )
  header, rows = table_rows([(args.name, None, payload)])
  _write(args.out_dir, f"{args.name}_push", payload, markdown_table(header, rows))
  print(
    json.dumps(
      dict(
        name=args.name,
        wall_clock_s=wall,
        ranked=metrics["ranked"],
        by_condition=metrics["by_condition"],
        by_magnitude=metrics["by_magnitude"],
      ),
      indent=2,
    )
  )
  return payload


def _add_training_overrides(p):
  g = p.add_argument_group("training metadata overrides (for runs lacking metadata)")
  g.add_argument("--train-num-envs", type=int)
  g.add_argument("--train-seed", type=int)
  g.add_argument("--parent-iter", type=int, help="Iteration of the warm-start parent")


def _overrides(args):
  return dict(
    train_num_envs=args.train_num_envs,
    train_seed=args.train_seed,
    parent_iter=args.parent_iter,
  )


def _add_rollout_args(p, single=True):
  if single:
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--reference-dir", type=Path)
    p.add_argument("--name", required=True)
  p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
  p.add_argument(
    "--raw-dir", type=Path, help="Optional dir for compressed per-env arrays"
  )
  p.add_argument("--seed", type=int, default=1234)
  p.add_argument("--task", default=DEFAULT_TASK)
  p.add_argument("--device", default="cuda:0")
  p.add_argument(
    "--heading-hold",
    action="store_true",
    help="Keep the task's command-side heading hold (default off: open-loop drift)",
  )
  p.add_argument(
    "--no-standard-physics",
    action="store_true",
    help="Do not force --deployment-target-clamp --compose-com-randomization "
    "--locomotion-torso-contact --driver-limits <class limits>; use flags as given",
  )
  p.add_argument(
    "--allow-reference-mismatch",
    action="store_true",
    help="Evaluate even though the training metadata names a different reference dir",
  )
  # Same physics flags as pupper_locomotion (consumed by its settings()).
  p.add_argument("--deployment-target-clamp", action="store_true")
  p.add_argument("--driver-limits", type=Path)
  p.add_argument("--target-slew-rad-s", type=float)
  p.add_argument(
    "--encoder-observation-mode", choices=("legacy_true", "deployment_measured")
  )
  p.add_argument("--allow-encoder-observation-mode-change", action="store_true")
  p.add_argument("--compose-com-randomization", action="store_true")
  p.add_argument("--locomotion-torso-contact", action="store_true")
  _add_training_overrides(p)


def _prepare(args, error):
  """Resolve paths, export the reference dir, physics defaults, training block."""
  if args.reference_dir:
    args.reference_dir = Path(args.reference_dir).resolve()
    if not args.reference_dir.is_dir():
      error(f"reference directory does not exist: {args.reference_dir}")
  # Must precede every mjlab task import: the policy observes its reference, and
  # gait_reference reads PUPPER_REFERENCE_DIR once, at import.
  current = os.environ.get("PUPPER_REFERENCE_DIR")
  wanted = str(args.reference_dir) if args.reference_dir else None
  if current != wanted:
    import sys

    if any(m.startswith("mjlab.tasks") for m in sys.modules):
      error("reference dir differs from the one already imported in this process")
    if wanted is None:
      os.environ.pop("PUPPER_REFERENCE_DIR", None)
    else:
      os.environ["PUPPER_REFERENCE_DIR"] = wanted
  args.checkpoint = Path(args.checkpoint).resolve()
  if not args.checkpoint.is_file():
    error(f"checkpoint not found: {args.checkpoint}")
  _apply_physics_defaults(args)
  args.training = training_block(args.checkpoint, args.reference_dir, _overrides(args))
  check = args.training["reference_check"]
  if check.startswith("MISMATCH") and not args.allow_reference_mismatch:
    error(f"{args.name}: {check} (pass --allow-reference-mismatch to evaluate anyway)")
  print(f"[part2_eval] {args.name}: training {json.dumps(args.training)}", flush=True)


def _check_env_count(args, mode, error):
  n = args.envs if mode == "push" else 16 * args.envs_per_schedule
  if not 1 <= n <= 1024:
    error("keep 1..1024 envs on the shared GPU")


def parse_batch_entry(entry):
  """CKPT[:REFDIR][=NAME] -> (checkpoint, reference_dir|None, name)."""
  name = None
  if "=" in entry:
    entry, name = entry.rsplit("=", 1)
  ckpt, _, ref = entry.partition(":")
  ckpt_p = Path(ckpt)
  return ckpt_p, (Path(ref) if ref else None), (name or ckpt_p.resolve().parent.name)


def annotate(paths, overrides):
  """Fill the training block (and push ranking) into existing JSONs (no rollout)."""
  files = []
  for p in map(Path, paths):
    files += sorted(p.glob("*.json")) if p.is_dir() else [p]
  done = []
  for f in files:
    payload = json.loads(f.read_text())
    if payload.get("kind") not in ("suite", "push"):
      continue
    prov = payload["provenance"]
    payload["training"] = training_block(
      Path(prov["checkpoint"]), prov.get("reference_dir"), overrides
    )
    if payload["kind"] == "push" and "by_condition" in payload["metrics"]:
      # Refresh the ranked block with the currently frozen standing rung.
      payload["metrics"]["ranked"] = rank_push(payload["metrics"])
    f.write_text(json.dumps(payload, indent=2) + "\n")
    header, rows = table_rows(
      [(payload["name"], payload, None)]
      if payload["kind"] == "suite"
      else [(payload["name"], None, payload)]
    )
    f.with_suffix(".md").write_text(markdown_table(header, rows))
    done.append(str(f))
  return done


def _raise(msg):
  raise ValueError(msg)


def run_batch(args, error):
  """Suite + push for many checkpoints, sequentially (one GPU job at a time).

  gait_reference reads PUPPER_REFERENCE_DIR once at import, so the batch runs one
  child process per distinct reference dir; within it every entry shares the
  imported task modules and only the env is rebuilt per rollout.
  """
  import subprocess
  import sys

  entries = [parse_batch_entry(e) for e in args.entries]
  for ckpt, ref, _ in entries:
    if not ckpt.is_file():
      error(f"checkpoint not found: {ckpt}")
    if ref is not None and not ref.is_dir():
      error(f"reference directory does not exist: {ref}")
  if args.worker:
    failures = 0
    for ckpt, ref, name in entries:
      for mode in args.modes:
        stem = name if mode == "suite" else f"{name}_push"
        if args.skip_existing and (Path(args.out_dir) / f"{stem}.json").exists():
          print(f"[part2_eval] skip existing {stem}", flush=True)
          continue
        run_args = copy.copy(args)
        run_args.checkpoint, run_args.reference_dir, run_args.name = ckpt, ref, name
        try:
          _prepare(run_args, _raise)
          (run_suite if mode == "suite" else run_push)(run_args)
        except Exception as exc:  # noqa: BLE001 - keep the queue going, report at end
          failures += 1
          print(f"[part2_eval] FAILED {name} {mode}: {exc!r}", flush=True)
    if failures:
      sys.exit(f"{failures} batch job(s) failed")
    return
  groups: dict[str, list[str]] = {}
  for raw, (_, ref, _) in zip(args.entries, entries, strict=True):
    groups.setdefault(str(ref.resolve()) if ref else "", []).append(raw)
  passthrough = [a for a in sys.argv[2:] if a not in args.entries]
  status = 0
  for ref, raws in groups.items():
    env = dict(os.environ)
    if ref:
      env["PUPPER_REFERENCE_DIR"] = ref
    else:
      env.pop("PUPPER_REFERENCE_DIR", None)
    # Entries first: a trailing nargs="+" option (--modes) would swallow them.
    cmd = [
      sys.executable,
      "-m",
      "mjlab.scripts.part2_eval",
      "batch",
      *raws,
      "--worker",
      *passthrough,
    ]
    print(f"[part2_eval] batch group ref={ref or 'default'}: {raws}", flush=True)
    status |= subprocess.call(cmd, env=env)
  if status:
    sys.exit("batch finished with failures")


def main():
  p = argparse.ArgumentParser(description=__doc__)
  sub = p.add_subparsers(dest="mode", required=True)
  suite = sub.add_parser("suite")
  _add_rollout_args(suite)
  suite.add_argument("--envs-per-schedule", type=int, default=24)
  push = sub.add_parser("push")
  _add_rollout_args(push)
  push.add_argument("--envs", type=int, default=PUSH_ENVS_DEFAULT)
  batch = sub.add_parser(
    "batch", help="suite+push for CKPT[:REFDIR][=NAME] entries, one process per ref dir"
  )
  _add_rollout_args(batch, single=False)
  batch.add_argument("entries", nargs="+")
  batch.add_argument("--envs-per-schedule", type=int, default=24)
  batch.add_argument("--envs", type=int, default=PUSH_ENVS_DEFAULT)
  batch.add_argument(
    "--modes", nargs="+", choices=("suite", "push"), default=["suite", "push"]
  )
  batch.add_argument("--skip-existing", action="store_true")
  batch.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
  cmp_ = sub.add_parser("compare")
  cmp_.add_argument("inputs", nargs="+", type=Path, help="suite JSONs or directories")
  cmp_.add_argument("--push-dir", type=Path)
  cmp_.add_argument("--out", type=Path, required=True)
  cmp_.add_argument(
    "--allow-mismatch",
    action="store_true",
    help="Tabulate runs with differing train num_envs / iterations past the parent (marked)",
  )
  ann = sub.add_parser("annotate", help="Fill the training block into existing JSONs")
  ann.add_argument(
    "inputs", nargs="+", type=Path, help="suite/push JSONs or directories"
  )
  _add_training_overrides(ann)
  args = p.parse_args()
  if args.mode == "compare":
    try:
      header, rows = compare(args.inputs, args.out, args.push_dir, args.allow_mismatch)
    except MismatchError as exc:
      p.error(str(exc))
    print(args.out.read_text())
    return
  if args.mode == "annotate":
    for f in annotate(args.inputs, _overrides(args)):
      print(f"annotated {f}")
    return
  if args.mode == "batch":
    _check_env_count(args, "suite", p.error)
    _check_env_count(args, "push", p.error)
    run_batch(args, p.error)
    return
  _check_env_count(args, args.mode, p.error)
  _prepare(args, p.error)
  if args.mode == "suite":
    run_suite(args)
  else:
    run_push(args)


if __name__ == "__main__":
  main()
