"""Eye on fieldcam's synthetic renderer: tracking accuracy and no hot-path stalls.

The floor session of 2026-11-24 stalled ~1 s once a second because a configured robot that was
not on the field triggered a 2x whole-frame relaxed tag search on every 10th frame. Here the
second robot is configured but never rendered."""
import math
import time

import pytest

cv2 = pytest.importorskip("cv2")

from fc.core import RobotCfg, load_field, wrap  # noqa: E402
from fc.eye import Eye, SyntheticSource  # noqa: E402

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
