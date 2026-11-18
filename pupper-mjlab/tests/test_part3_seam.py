"""Unit tests for the Part 3 seam test's pure-NumPy pieces (no GPU, no env)."""

import numpy as np
import pytest

from mjlab.scripts import part3_seam as ps
from mjlab.scripts.part3_common import LAB4_DR, lab4_dr_from_file, wilson


def test_sweep_reaches_top_and_returns_at_rate():
  vx = ps.sweep(1, 0.5)
  assert vx[0] == 0.0 and vx[-1] == 0.0
  assert np.isclose(vx.max(), ps.TOP)
  # 0 -> 1.5 at 0.5 m/s^2 takes 3 s = 150 steps.
  first_top = int(np.argmax(vx >= ps.TOP - 1e-12))
  assert first_top == int(1.0 / ps.DT) + 150 - 1
  assert np.all(np.abs(np.diff(vx)) <= 0.5 * ps.DT + 1e-9)
  back = ps.sweep(-1, "inf")
  assert np.isclose(back.min(), -ps.TOP)
  assert np.max(np.abs(np.diff(back))) == pytest.approx(ps.TOP)


def test_crossings_up_down_and_ignores_direction_flip():
  vx = np.array([0.0, 0.3, 0.5, 0.7, 0.49, 0.0, -0.3, -0.6, -0.2])
  assert ps.crossings(vx, 0.5) == [(2, "up"), (4, "down"), (7, "up"), (8, "down")]
  # A sign flip below the threshold is not a seam crossing.
  assert ps.crossings(np.array([0.3, -0.3, 0.3]), 0.5) == []


def test_schedules_cross_seam_expected_times():
  sch = {s[0]: s for s in ps.schedule_set()}
  assert len(ps.crossings(sch["sweep_fwd_r0.5"][4], 0.5)) == 2
  assert len(ps.crossings(sch["sweep_back_r1"][4], 0.5)) == 2
  assert len(ps.crossings(sch["dither_fwd"][4], 0.5)) == 8
  ctrl = sch["control_back"][4]
  # The control never ramps through the seam at the control rate: its only seam
  # crossing is the excluded 0 -> 0.8 jump-in; the sham anchors exist.
  assert len(ps.crossings(ctrl, 0.3)) == 3 and len(ps.crossings(ctrl, 1.0)) == 2
  assert len(ps.crossings(ctrl, 0.5)) == 1


def test_control_drops_sham_anchor_at_seam_jump_in():
  sch = [s for s in ps.schedule_set(rates=(0.5,)) if s[0] == "control_fwd"]
  vx = sch[0][4]
  T = len(vx)
  cmd = np.zeros((T, 1, 3))
  cmd[:, 0, 0] = vx
  z = lambda *s: np.zeros((T, 1, *s))  # noqa: E731
  m = ps.seam_metrics(
    cmd=cmd, env_schedule=np.zeros(1, int), schedules=sch, terminated=np.zeros((T, 1), bool),
    velocity=z(3), tilt=np.zeros((T, 1)), knee=np.zeros((T, 1), bool), torso=np.zeros((T, 1), bool),
    torque=z(12), foot_force=z(4), mass_kg=np.ones(1), ref=z(12))
  assert m["n_windows"] == 4 and all(w["anchor"] != "seam" for w in m["windows"])


def test_build_commands_delay_shifts_schedule():
  sch = ps.schedule_set(rates=(0.5,))
  cmd, env_schedule, delay = ps.build_commands(sch, 3, 0.5, seed=0)
  assert cmd.shape[1] == len(sch) * 3
  for e in range(cmd.shape[1]):
    vx = sch[env_schedule[e]][4]
    np.testing.assert_allclose(cmd[delay[e] : delay[e] + len(vx), e, 0], vx)
    assert np.all(cmd[: delay[e], e] == 0)
  assert delay.max() <= 25


def test_body_tilt():
  g = np.array([[0, 0, -1.0], [np.sin(np.radians(30)), 0, -np.cos(np.radians(30))]])
  np.testing.assert_allclose(ps.body_tilt_deg(g), [0, 30], atol=1e-9)


def _synthetic(fall_at=None, knee_at=None, tilt_at=None, n_env=2):
  sch = [("sweep_fwd_r1", "sweep", "fwd", "1", ps.sweep(1, 1.0), ())]
  vx = sch[0][4]
  T = len(vx)
  cmd = np.zeros((T, n_env, 3))
  cmd[:, :, 0] = vx[:, None]
  z = lambda *s: np.zeros((T, n_env, *s))  # noqa: E731
  terminated = np.zeros((T, n_env), bool)
  knee = np.zeros((T, n_env), bool)
  tilt = np.full((T, n_env), 5.0)
  if fall_at is not None:
    terminated[fall_at, 0] = True
  if knee_at is not None:
    knee[knee_at, 0] = True
  if tilt_at is not None:
    tilt[tilt_at, 0] = 40.0
  ref = z(12)
  k_up = ps.crossings(vx, 0.5)[0][0]
  ref[k_up:, :, 0] = 1.0  # a 1 rad jump at the up crossing
  m = ps.seam_metrics(
    cmd=cmd, env_schedule=np.zeros(n_env, int), schedules=sch, terminated=terminated,
    velocity=np.concatenate([cmd[..., :2], z(1)], -1), tilt=tilt, knee=knee, torso=knee * False,
    torque=z(12), foot_force=np.full((T, n_env, 4), 9.81), mass_kg=np.ones(n_env), ref=ref)
  return m, ps.crossings(vx, 0.5)


def test_seam_metrics_clean_run():
  m, cr = _synthetic()
  assert m["n_windows"] == 2 * len(cr)
  s = m["summary"]["seam_sweep"]
  assert s["fall"] == 0 and s["stumble"] == 0 and s["sat_frac_mean"] == 0
  assert s["foot_force_max_bw_median"] == pytest.approx(1.0)
  assert s["track_err_median"] == pytest.approx(0.0)
  up = [g for g in m["groups"] if g["crossing"] == "up"][0]
  assert up["ref_jump_median"] == pytest.approx(1.0)


def test_seam_metrics_attributes_fall_and_stumble_to_window():
  _, cr = _synthetic()
  k_up, k_down = cr[0][0], cr[1][0]
  m, _ = _synthetic(knee_at=k_up + 3, tilt_at=k_down + 10)
  s = m["summary"]["seam_sweep"]
  assert s["stumble"] == 2 and s["knee"] == 1 and s["tilt"] == 1 and s["fall"] == 0
  m, _ = _synthetic(fall_at=k_up + 5)
  s = m["summary"]["seam_sweep"]
  assert s["fall"] == 1
  # The env resets after the fall; its down window still counts if settled.
  assert m["n_windows"] + sum(m["invalid_windows"].values()) == 4


def test_seam_metrics_window_after_reset_is_invalid():
  _, cr = _synthetic()
  k_down = cr[1][0]
  m, _ = _synthetic(fall_at=k_down - 20)  # reset 0.4 s before the down window
  assert m["invalid_windows"]["reset"] == 1


def test_verdict():
  m, _ = _synthetic()
  ctrl = ps._rate_block([])
  m["summary"]["control_all"] = dict(ctrl, n=10, stumble_rate=0.0)
  v = ps.verdict(m, dict(max_fall_rate=0.02, max_stumble_excess=0.05))
  assert v["verdict"] == "PASS"
  _, cr = _synthetic()
  m2, _ = _synthetic(fall_at=cr[0][0] + 5)
  m2["summary"]["control_all"] = dict(ctrl, n=10, stumble_rate=0.0)
  assert ps.verdict(m2, dict(max_fall_rate=0.02, max_stumble_excess=0.05))["verdict"] == "FAIL"


def test_wilson_and_lab4_record():
  lo, hi = wilson(0, 100)
  assert lo == 0 and 0.03 < hi < 0.04
  assert wilson(0, 0) == (None, None)
  assert lab4_dr_from_file() == LAB4_DR
