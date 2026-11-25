"""Eye on fieldcam's synthetic renderer: tracking accuracy and no hot-path stalls.

The floor session of 2026-11-24 stalled ~1 s once a second because a configured robot that was
not on the field triggered a 2x whole-frame relaxed tag search on every 10th frame. Here the
second robot is configured but never rendered."""
import importlib.util
import math
import random
import time

import pytest

from fc.ballvel import BallVelocity, fit_decel
from fc.core import RobotCfg, load_field, wrap

HAVE_CV2 = importlib.util.find_spec("cv2") is not None
needs_cv2 = pytest.mark.skipif(not HAVE_CV2, reason="needs OpenCV (.venv-fc)")
if HAVE_CV2:
    from fc.eye import Eye, SyntheticSource

BLUE = RobotCfg("blue", "blue", 10, "", "stock")
RED = RobotCfg("red", "red", 11, "", "stock")   # configured, not on the field


def run_eye(seconds: float):
    field = load_field()
    eye = Eye(field, [BLUE, RED], {}, source="synthetic")
    src = {}

    def only_blue():
        src["s"] = SyntheticSource(eye.cfg, field, [BLUE], 1920, 1080)
        return src["s"]
    eye._open_source = only_blue
    eye.start()
    t_end, errs = time.monotonic() + seconds, []
    try:
        while time.monotonic() < t_end:
            time.sleep(0.1)
            w = eye.latest()
            o = w.robots.get("blue") if w else None
            if o is not None and "s" in src:
                x, y, yaw = src["s"].poses(o.t - src["s"].t0)[0]["blue"]
                errs.append((math.hypot(o.pose.x - x, o.pose.y - y), abs(wrap(o.pose.yaw - yaw))))
        return eye, errs
    finally:
        eye.close()


@needs_cv2
def test_tracks_robot_and_never_stalls_for_an_absent_one():
    eye, errs = run_eye(8.0)   # the synthetic renderer gives ~5 fps
    assert eye.perc.calibration_valid, eye.message
    assert len(errs) > 20
    pos = sorted(e[0] for e in errs)
    assert pos[len(pos) // 2] < 0.03 and max(e[1] for e in errs) < math.radians(8)
    frames = [r for r in eye.trace if r.get("seen") and "calibrate" not in r]   # calibrated frames
    assert len(frames) > 15
    assert sum(r["seen"].get("blue", False) for r in frames) > 0.9 * len(frames)
    assert sum(r["seen"]["ball"] for r in frames) > 0.9 * len(frames)
    # the hot path never runs the whole-frame search: generous bound for a loaded CI machine
    assert max(r["process"] for r in frames) < 150, max(r["process"] for r in frames)
    assert "deep" in eye.stage_ms and "scan" in eye.stage_ms
    assert "stage_ms" in eye.status()


@needs_cv2
def test_crop_search_outlives_a_short_tag_gap():
    """A robot missed for ~1 s is still looked for around its last sighting (the full-frame scan
    rarely finds the small robot tag, so dropping the crop after 0.5 s turned short misses into
    stale-pose stops); a sighting older than SEED_KEEP_S is left to the scan."""
    import numpy as np
    from fc import eye as eye_mod

    eye = Eye(load_field(), [BLUE], {}, source="synthetic")
    eye.perc = type("P", (), {"calibration_valid": True})()
    eye._robot_tags, eye._px, eye._roi_det = {BLUE.tag}, 1.0, (None, None)
    looked = []
    eye._look = lambda gray, centre, half, want, dets, scales=(1, 2): looked.append((centre, half)) or {}
    gray = np.zeros((1080, 1920), np.uint8)
    for age, expect in ((1.0, True), (eye_mod.SEED_KEEP_S + 0.1, False)):
        looked.clear()
        eye._seeds = {BLUE.tag: (900.0, 500.0, 10.0)}
        eye._t_frame = 10.0 + age
        eye._detect_tags(gray)
        assert bool(looked) == expect, (age, looked)
    assert eye_mod.SEED_KEEP_S >= 1.0


# ---- ball velocity (fc.ballvel, stdlib) -------------------------------------------------------
def _noisy(rng, x, y, noise=0.003, flicker=0.03):
    if rng.random() < flicker:   # single-frame 2 cm detection flicker, as at rest on Nov 24
        a = rng.uniform(0, 2 * math.pi)
        x, y = x + 0.02 * math.cos(a), y + 0.02 * math.sin(a)
    return x + rng.gauss(0, noise), y + rng.gauss(0, noise)


def roll(v0=1.0, a=0.45, rest_s=1.0, fps=31.0, seed=3, direction=(0.8, 0.6)):
    """Ball at rest, released at v0 at t = rest_s, slowing at a. -> [(t, x, y, v_true)]."""
    rng, out, t = random.Random(seed), [], 0.0
    t_stop = rest_s + v0 / a
    while t < t_stop + 1.0:
        tr = min(max(0.0, t - rest_s), v0 / a)
        s = v0 * tr - 0.5 * a * tr * tr
        x, y = _noisy(rng, -1.0 + direction[0] * s, direction[1] * s)
        out.append((t, x, y, max(0.0, v0 - a * max(0.0, t - rest_s)) if t >= rest_s else 0.0))
        t += 1 / fps
    return out


def test_ball_at_rest_reports_zero_and_does_not_drift():
    f, rng = BallVelocity(), random.Random(1)
    for i in range(300):
        x, y, vx, vy = f.update(i / 31, *_noisy(rng, 0.3, -0.2))
        assert vx == 0.0 and vy == 0.0
        if i > 5:
            assert math.hypot(x - 0.3, y + 0.2) < 0.012


def test_rolling_ball_speed_lag_and_error():
    for v0, a in ((0.5, 0.3), (1.0, 0.45), (2.0, 0.6)):
        f = BallVelocity()
        rows = [(t, vt, math.hypot(*f.update(t, x, y, a)[2:])) for t, x, y, vt in roll(v0, a)]
        t90 = next(t - 1.0 for t, vt, v in rows if t >= 1.0 and v >= 0.9 * vt)
        assert t90 <= 0.12, (v0, t90)                      # <= ~4 frames after release
        rolling = [abs(v - vt) for t, vt, v in rows if 1.5 <= t <= 1.0 + v0 / a - 0.3]
        assert max(rolling) < 0.07 and sorted(rolling)[len(rolling) // 2] < 0.03, (v0, max(rolling))
        assert rows[-1][2] == 0.0                          # stopped again


def test_single_outlier_is_dropped_and_a_real_jump_restarts():
    f = BallVelocity()
    for i in range(20):
        f.update(i / 30, 0.0, 0.0)
    x, y, vx, vy = f.update(20 / 30, 0.5, 0.5)            # one false sighting far away
    assert (vx, vy) == (0.0, 0.0) and math.hypot(x, y) < 0.01
    x, y, vx, vy = f.update(21 / 30, 0.0, 0.0)
    assert (vx, vy) == (0.0, 0.0) and f.outliers == 1
    f.update(22 / 30, 0.8, 0.0)                            # the ball really is elsewhere now (picked up, put down)
    x, y, vx, vy = f.update(23 / 30, 0.8, 0.0)
    assert math.isclose(x, 0.8, abs_tol=1e-6) and (vx, vy) == (0.0, 0.0) and f.resets == 1


def test_reacquired_after_a_gap_has_no_velocity_spike():
    f = BallVelocity()
    for i in range(10):
        f.update(i / 30, 0.0, 0.0)
    for i in range(5):   # seen again 1 s later, 0.4 m away: no 0.4 m/s phantom
        _, _, vx, vy = f.update(1.3 + i / 30, 0.4, 0.0)
        assert (vx, vy) == (0.0, 0.0)
    assert f.resets == 1


def test_fit_decel_recovers_deceleration_from_rolls():
    pts = []
    for k, (v0, a) in enumerate(((0.9, 0.3), (1.2, 0.3), (0.7, 0.3))):
        pts += [(10 * k + t, x, y) for t, x, y, _ in roll(v0, a, seed=k, direction=(1.0, 0.0) if k % 2 else (0.6, -0.8))]
    fit = fit_decel(pts)
    assert fit["n_used"] == 3 and abs(fit["decel"] - 0.3) < 0.05, fit
    assert fit_decel([(i / 30, 0.0, 0.0) for i in range(100)])["decel"] is None   # nothing rolled


@needs_cv2
def test_frozen_picture_marks_camera_paused():
    """The phone feed froze into byte-identical frames at full rate (074158Z). Identical frames are
    not processed (observation times stop advancing) and > 0.5 s of them is CAMERA PAUSED."""
    import numpy as np
    from fc import eye as eye_mod
    eye = Eye(load_field(), [BLUE], {}, source="synthetic")
    eye.src = object()
    rng = np.random.default_rng(0)
    a = rng.integers(0, 255, (1080, 1920, 3), dtype=np.uint8)
    now = time.monotonic()
    assert not eye._same_picture(a, now - 2.0)
    assert eye._same_picture(a.copy(), now - 1.9) and eye.paused   # same picture for 1.9 s
    noisy = np.clip(a.astype(np.int16) + rng.integers(-2, 3, a.shape), 0, 255).astype(np.uint8)   # sensor noise
    assert not eye._same_picture(noisy, now) and not eye.paused
    assert eye.pauses == 1 and eye.status()["paused"] is False
    assert eye_mod.FROZEN_S <= 0.5


@needs_cv2
def test_eye_stops_refreshing_a_frozen_feed():
    field = load_field()
    eye = Eye(field, [BLUE], {}, source="synthetic")
    src = {}

    def freezing():
        s = SyntheticSource(eye.cfg, field, [BLUE], 1920, 1080)
        read, held = s.read, {}

        def frozen_read():
            frame, t = read()
            if time.monotonic() - s.t0 > 3.0:   # the picture freezes after 3 s, frames keep coming
                held.setdefault("f", frame)
                return held["f"].copy(), t
            return frame, t
        s.read = frozen_read
        src["s"] = s
        return s
    eye._open_source = freezing
    eye.start()
    try:
        assert _wait(lambda: eye.latest() is not None and eye.latest().robots, 6)
        assert _wait(lambda: eye.status()["paused"], 6)
        t_obs = eye.latest().robots["blue"].t
        time.sleep(0.6)
        assert eye.latest().robots.get("blue") is None or eye.latest().robots["blue"].t == t_obs
        assert eye.status()["picture_age"] > 0.5 and eye.pauses == 1
    finally:
        eye.close()


def _wait(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False
