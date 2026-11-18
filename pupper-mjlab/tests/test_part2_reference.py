"""Part 2 capture/fold tooling: NumPy core on synthetic traces, plus a loader round trip."""

from __future__ import annotations

import argparse
import json
from typing import Any

import numpy as np
import pytest

from mjlab.scripts import part2_reference as pr

DT = 0.02
CYCLE_STEPS = 1.0 / (pr.GAIT_FREQUENCY * DT)  # 37.5 control steps per cycle


def smooth_table(n=100, seed=0):
  rng = np.random.default_rng(seed)
  x = np.arange(n) / n
  table = np.zeros((n, 12))
  for j in range(12):
    for k in (1, 2, 3):
      table[:, j] += rng.normal(0, 0.15 / k) * np.sin(
        2 * np.pi * k * x + rng.uniform(0, 2 * np.pi)
      )
  return table


def at_phase(table, p):
  """Periodic linear lookup of ``table`` at phases ``p`` (any shape)."""
  n = len(table)
  x = np.mod(p, 1.0) * n
  i0 = np.floor(x).astype(int) % n
  w = (x - np.floor(x))[..., None]
  return (1 - w) * table[i0] + w * table[(i0 + 1) % n]


def synthetic_trace(
  parent_fwd, parent_back, lag_steps=2.0, steps=1500, envs=6, noise=0.0, seed=1
) -> dict[str, Any]:
  """Two 750-step segments (+1.5, -1.5); robot follows the parent late by lag_steps."""
  rng = np.random.default_rng(seed)
  t = np.arange(steps)
  phase = np.mod(t * DT * pr.GAIT_FREQUENCY, 1.0)
  q = np.zeros((steps, envs, 12))
  half = steps // 2
  lag = lag_steps / CYCLE_STEPS
  for table, sl in ((parent_fwd, slice(0, half)), (parent_back, slice(half, steps))):
    q[sl] = at_phase(table, phase[sl] - lag)[:, None, :]
  q += rng.normal(0, noise, q.shape)
  command = np.zeros((steps, envs, 3))
  command[:half, :, 0] = 1.5
  command[half:, :, 0] = -1.5
  valid = np.ones((steps, envs), bool)
  valid[:150] = False
  valid[half : half + 150] = False
  episode = np.zeros((steps, envs), int)
  terminated = np.zeros((steps, envs), bool)
  # env 0 falls mid-segment: the cycles around it must not be folded together.
  terminated[400, 0] = True
  valid[400:550, 0] = False
  episode[401:, 0] = 1
  q[400:550, 0] += 1.0  # garbage that must be excluded
  # Contacts: front_r lands once per cycle at reference phase 0.3 (delayed by lag).
  contact = np.zeros((steps, envs, 4), bool)
  ph_robot = np.mod(phase - lag, 1.0)
  contact[..., 0] = ((ph_robot >= 0.3) & (ph_robot < 0.8))[:, None]
  return dict(
    joint_pos=q, phase=np.broadcast_to(phase[:, None], (steps, envs)).copy(), episode=episode,
    command=command, valid=valid, terminated=terminated, timeout=np.zeros_like(terminated),
    foot_contact=contact, velocity=command.copy(), segment=(t >= half).astype(int), dt=np.asarray(DT),
  )  # fmt: skip


def fold(trace, parent, **kw):
  opts: dict[str, Any] = dict(
    align="clock", lag=None, symmetrize=False, reduce="median", harmonics=0, samples=100,
    min_cycles=5, frequency_mult=1.0, touchdown_foot=0, pin=(),
  )  # fmt: skip
  opts.update(kw)
  return pr.fold_direction(trace, [1.5, 0, 0], parent, **opts)


def rms(a, b):
  return float(np.sqrt(((a - b) ** 2).mean()))


def test_contiguous_runs():
  assert pr.contiguous_runs(np.array([0, 1, 1, 0, 1, 0, 1, 1, 1], bool)) == [
    (1, 3),
    (4, 5),
    (6, 9),
  ]
  assert pr.contiguous_runs(np.zeros(3, bool)) == []


def test_steady_mask_does_not_mutate_trace():
  parent = smooth_table()
  trace = synthetic_trace(parent, parent)
  before = trace["valid"].copy()
  pr.steady_mask(trace, [1.5, 0, 0])
  assert (trace["valid"] == before).all()
  assert pr.steady_mask(trace, [-1.5, 0, 0]).sum() > 0


def test_naive_fold_reproduces_the_lagged_trajectory_and_lag_variant_undoes_it():
  parent = smooth_table()
  trace = synthetic_trace(parent, parent, lag_steps=2.0)
  naive, rep = fold(trace, parent)
  lagged = at_phase(parent, np.arange(100) / 100 - 2.0 / CYCLE_STEPS)
  assert rms(naive, lagged) < 2e-3
  assert rep["cycles"] > 20
  fixed, rep = fold(trace, parent, lag="auto")
  assert rep["lag_steps"] == pytest.approx(2.0, abs=0.1)
  assert rms(fixed, parent) < 3e-3
  explicit, rep = fold(trace, parent, lag="2.0")
  assert rms(explicit, parent) < 3e-3


def test_fixed_lag_is_checked_against_the_measured_lag():
  parent = smooth_table()
  ok, rep = fold(
    parent=parent, trace=synthetic_trace(parent, parent, lag_steps=2.2), lag=2.0
  )
  assert "lag_check_warning" not in rep
  assert rep["lag_measurement"]["lag_steps"] == pytest.approx(2.2, abs=0.1)
  _, rep = fold(
    parent=parent, trace=synthetic_trace(parent, parent, lag_steps=3.5), lag=2.0
  )
  assert "differs from the fixed" in rep["lag_check_warning"]


def test_default_fold_matches_the_decision():
  assert pr.VARIANTS["lag"] == ("clock", 2.0, False)
  assert set(pr.DEFAULT_PIN) == {
    f"leg_{s}_2" for s in ("front_r", "front_l", "back_r", "back_l")
  }


def test_fold_excludes_fall_and_reset():
  parent = smooth_table()
  trace = synthetic_trace(parent, parent)
  table, rep = fold(trace, parent, reduce="mean")
  # The +1.0 rad garbage around the fall would bias a mean if it leaked in.
  assert (
    np.abs(table - at_phase(parent, np.arange(100) / 100 - 2.0 / CYCLE_STEPS)).max()
    < 5e-3
  )


def test_reductions_and_medoid():
  rng = np.random.default_rng(0)
  base = smooth_table()
  closed = np.vstack([base, base[:1]])
  cycles = closed[None] + rng.normal(0, 0.01, (9, 101, 12))
  cycles[0] += 2.0  # one outlier
  med, _ = pr.reduce_cycles(cycles, "median")
  medoid, info = pr.reduce_cycles(cycles, "medoid")
  assert rms(med, closed) < 0.01 and info["medoid_index"] != 0
  mean, _ = pr.reduce_cycles(cycles, "mean")
  assert rms(mean, closed) > 0.1


def test_enforce_periodic_removes_drift():
  base = smooth_table()
  closed = np.vstack([base, base[:1]]) + np.linspace(0, 0.2, 101)[:, None]
  table, info = pr.enforce_periodic(closed, harmonics=0)
  assert info["max_wrap_mismatch_before_rad"] == pytest.approx(0.2)
  assert rms(table, base) < 1e-9
  smooth, info = pr.enforce_periodic(closed, harmonics=6)
  assert info["max_wrap_extrapolation_miss_after_rad"] < 0.02


def test_touchdown_alignment_removes_cycle_jitter():
  parent = smooth_table()
  trace = synthetic_trace(parent, parent, lag_steps=0.0)
  # Jitter: robot runs alternately early/late; contacts follow the robot.
  t = np.arange(len(trace["phase"]))
  wobble = 0.03 * np.sin(2 * np.pi * t * DT * pr.GAIT_FREQUENCY / 3.7)
  robot_phase = np.mod(trace["phase"][:, 0] + wobble, 1.0)
  trace["joint_pos"][:750] = at_phase(parent, robot_phase[:750])[:, None, :]
  trace["foot_contact"][..., 0] = ((robot_phase >= 0.3) & (robot_phase < 0.8))[:, None]
  clock, _ = fold(trace, parent)
  td, rep = fold(trace, parent, align="touchdown")
  assert rep["touchdown_anchor_phase"] == pytest.approx(0.3, abs=0.03)
  assert rms(td, parent) < 0.7 * rms(clock, parent)  # step quantization limits it


def test_touchdown_cadence_mismatch_raises_or_falls_back():
  parent = smooth_table()
  trace = synthetic_trace(parent, parent)
  fast = (np.arange(len(trace["phase"])) % 8) < 4
  trace["foot_contact"][..., 0] = fast[:, None]
  with pytest.raises(ValueError, match="does not step on the reference clock"):
    fold(trace, parent, align="touchdown")
  _, rep = fold(trace, parent, align="touchdown", touchdown_fallback=True)
  assert "touchdown_fallback" in rep and rep["align"] == "touchdown"


def test_mirror_shift_and_symmetrize():
  rng = np.random.default_rng(3)
  right_front, right_back = smooth_table(seed=4)[:, :3], smooth_table(seed=5)[:, :3]
  table = np.zeros((100, 12))
  table[:, 0:3], table[:, 6:9] = right_front, right_back
  table[:, 3:6] = -pr.circular_shift(right_front, -0.5)  # left(t) = -right(t - .5)
  table[:, 9:12] = -pr.circular_shift(right_back, -0.5)
  m = pr.measure_mirror_shift(table)
  assert m["shift_cycles"] == pytest.approx(0.5) and m["asymmetry_rms_rad"] < 1e-9
  assert rms(pr.mirror_pairs(table, 0.5), table) < 1e-9
  noisy = table + rng.normal(0, 0.02, table.shape)
  assert (
    pr.measure_mirror_shift(pr.mirror_pairs(noisy, 0.5))["asymmetry_rms_rad"] < 1e-9
  )


def test_measure_lag_sign_and_fractional_shift():
  parent = smooth_table()
  late = pr.circular_shift(parent, -0.043)  # robot 0.043 cycles behind
  info = pr.measure_lag(late, parent)
  assert info["lag_cycles"] == pytest.approx(0.043, abs=2e-3)


def test_parent_stored_tables_inverts_loader_reversal():
  stored = smooth_table()
  loaded = np.roll(stored[::-1], 1, axis=0)  # what gait_reference does to reach_back
  assert np.allclose(pr.parent_stored_tables(stored, loaded)["reach_back"], stored)


def test_pin_and_diff_flag():
  parent = smooth_table()
  new = parent.copy()
  j = pr.JOINT_ORDER.index("leg_back_r_2")
  parent[:, j] = 0.26
  new[:, j] = 0.45  # near-constant channel moved: the crouch signature
  rows = {r["channel"]: r for r in pr.diff_rows(new, parent)}
  assert rows["leg_back_r_2"]["flag_near_constant_shift"]
  pinned = pr.pin_channels(new, parent, pr.DEFAULT_PIN)
  assert np.allclose(pinned[:, j], 0.26)
  assert not any(r["flag_near_constant_shift"] for r in pr.diff_rows(pinned, parent))


def test_parse_commands():
  assert pr.parse_commands("1.5,-1.5") == [[1.5, 0, 0], [-1.5, 0, 0]]
  assert pr.parse_commands("[[1,0,0.5]]") == [[1, 0, 0.5]]


@pytest.mark.slow
def test_fold_main_writes_loader_compatible_reference(tmp_path):
  """End to end on a synthetic capture: default parent, all variants, real loader."""
  parent = pr.load_parent(None)
  fwd, back = parent["tables"]["reach_fwd"], parent["tables"]["reach_back"]
  trace = synthetic_trace(fwd, back, lag_steps=1.8, noise=0.002)
  cap = tmp_path / "capture"
  cap.mkdir()
  np.savez_compressed(cap / "trace.npz", **trace)
  (cap / "capture.json").write_text(json.dumps({"reference_dir": None}))
  for variant in ("naive", "lag", "phase", "sym"):
    out = tmp_path / variant
    args = argparse.Namespace(
      capture=cap, out=out, variant=variant, align=None, lag_steps=None, symmetrize=None,
      pin="leg_back_r_2,leg_back_l_2", parent_ref_dir=None, reduce="median", harmonics=6,
      samples=100, speed=1.5, min_cycles=5, frequency_mult=None, touchdown_foot="front_r",
      touchdown_fallback=False, limit_policy="clip", force=False, no_verify=False,
    )  # fmt: skip
    summary = pr.fold_main(args)
    assert summary["verify"]["ok"]
    manifest = json.loads((out / "reference_manifest.json").read_text())
    assert manifest["frequency_mult"] == 1.0
    with np.load(out / "captured_reach_gaits.npz") as f:
      assert f["reach_fwd"].shape == (100, 12) and f["reach_back"].shape == (100, 12)
      for name in ("leg_back_r_2", "leg_back_l_2"):
        j = pr.JOINT_ORDER.index(name)
        assert np.allclose(f["reach_fwd"][:, j], fwd[:, j]) and np.allclose(
          f["reach_back"][:, j], back[:, j]
        )
    assert (out / "diff_report.md").exists()
    assert summary["measured_lag_steps"]["reach_fwd"] == pytest.approx(1.8, abs=0.15)
    if variant in ("lag", "sym"):
      assert summary["lag_steps"]["reach_fwd"] == pytest.approx(pr.DEFAULT_LAG_STEPS)
      assert not summary["warnings"]
