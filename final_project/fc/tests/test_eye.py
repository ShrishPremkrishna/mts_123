"""Eye on fieldcam's synthetic renderer: tracking accuracy and no hot-path stalls.

The floor session of 2026-11-24 stalled ~1 s once a second because a configured robot that was
not on the field triggered a 2x whole-frame relaxed tag search on every 10th frame. Here the
second robot is configured but never rendered."""
import importlib.util
import math
import random
import threading
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
    """Frames keep coming but the picture is frozen: the eye itself marks CAMERA PAUSED and its
    World stops refreshing. Synchronised on events, not wall-clock: the freeze starts once the robot
    is tracked, and "stops refreshing" is checked over a count of frozen frames the eye consumed
    (status()["paused"] alone can also flag a merely slow frame on a loaded machine)."""
    field = load_field()
    eye = Eye(field, [BLUE], {}, source="synthetic")
    freeze, frozen_reads = threading.Event(), [0]

    def freezing():
        s = SyntheticSource(eye.cfg, field, [BLUE], 1920, 1080)
        read, held = s.read, {}

        def frozen_read():
            frame, t = read()
            if freeze.is_set():   # the picture freezes on cue, frames keep coming
                held.setdefault("f", frame)
                frozen_reads[0] += 1
                return held["f"].copy(), t
            return frame, t
        s.read = frozen_read
        return s
    eye._open_source = freezing
    eye.start()
    try:
        assert _wait(lambda: eye.latest() is not None and eye.latest().robots, 60)
        freeze.set()
        assert _wait(lambda: eye.paused, 60)   # the eye thread saw only identical pictures for > FROZEN_S
        assert eye.status()["paused"]
        w = eye.latest()
        t_obs = w.robots["blue"].t if "blue" in w.robots else None
        n = frozen_reads[0]
        assert _wait(lambda: frozen_reads[0] >= n + 5, 60)   # 4+ more frozen frames fully handled by the eye...
        assert eye.latest() is w                               # ...and none of them refreshed the World
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


@needs_cv2
def test_synthetic_ignores_a_floor_ball_colour(tmp_path):
    """The synthetic renderer paints the default pink ball; a floor-calibrated ball_color.json (purple,
    Nov 25: hue 150-162 vs pink ~164) made the synthetic ball invisible. A real camera still loads it."""
    from fc import ballcal
    path = tmp_path / "ball_color.json"
    ballcal.save(path, [{"lo": [150, 145, 198], "hi": [162, 255, 255]}], how="auto", u=1.0, v=1.0, k=3.0, saved="x")
    eye = Eye(load_field(), [BLUE], {}, source="synthetic", ball_color_path=path)
    assert eye.ball_color_source.startswith("fieldcam default")
    assert eye.cfg["ball"]["colors"] != ballcal.load(path)


# ---- robot tag board blob (tag blurred at speed) ---------------------------------------------------------
BOARD_BGR = (230, 140, 40)   # bright blue tape (OpenCV BGR): HSV ~ (104, 211, 230)
BLUE_HSV = [{"lo": [98, 120, 150], "hi": [114, 255, 255]}]


def _board_frame(u, v, side=40, angle=20.0, blur=0, extra=()):
    """Carpet-grey frame with a blue board (white tag square in the middle) centred at (u, v)."""
    import cv2
    import numpy as np
    f = np.full((540, 960, 3), (70, 75, 72), np.uint8)
    for (cu, cv_, s, a), col in [((u, v, side, angle), BOARD_BGR), ((u, v, side * 0.55, angle), (250, 250, 250))]:
        box = cv2.boxPoints(((cu, cv_), (s, s), a)).astype(np.int32)
        cv2.fillConvexPoly(f, box, col)
    for (eu, ev, r) in extra:
        cv2.circle(f, (eu, ev), r, BOARD_BGR, -1)
    if blur:
        f = cv2.blur(f, (blur, 1))
    return f


@needs_cv2
def test_find_board_picks_the_board_not_small_or_far_blue_things():
    import cv2
    from fc.eye import color_mask, find_board
    exp = 40 * 40
    f = _board_frame(400, 300, extra=[(372, 330, 5), (380, 336, 4), (700, 100, 22)])   # display icons, far blob
    m = color_mask(cv2.cvtColor(f, cv2.COLOR_BGR2HSV), BLUE_HSV)
    b = find_board(m, (410, 295), exp, 60)
    assert b is not None and abs(b[0] - 400) < 2 and abs(b[1] - 300) < 2, b
    assert find_board(m, (650, 120), exp, 30) is None          # the far blob is too small for a board / gated
    blurred = _board_frame(400, 300, blur=15)                   # ~0.1 m of horizontal motion blur
    b = find_board(color_mask(cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV), BLUE_HSV), (400, 300), exp, 60)
    assert b is not None and math.hypot(b[0] - 400, b[1] - 300) < 3, b


@needs_cv2
def test_board_colour_from_the_ring_around_a_tag():
    import cv2
    import numpy as np
    from fc import ballcal
    from fc.eye import board_ring_pixels
    f = _board_frame(480, 270, side=60, angle=0)
    tag = np.array([[464, 254], [496, 254], [496, 286], [464, 286]], np.float32)   # the white square
    cols = ballcal.hsv_ranges(board_ring_pixels(f, tag))
    h, s, v = cv2.cvtColor(np.uint8([[BOARD_BGR]]), cv2.COLOR_BGR2HSV)[0, 0]
    assert len(cols) == 1 and cols[0]["lo"][0] <= h <= cols[0]["hi"][0] and cols[0]["lo"][1] <= s and cols[0]["lo"][2] <= v


def test_integrate_cmds_holds_each_command_at_most_the_bridge_ttl():
    from fc.eye import integrate_cmds
    cmds = [(0.0, 0.2, 0.0, 1.0), (0.1, 0.2, 0.0, 1.0), (0.2, 0.7, 0.1, -0.5)]   # then silence
    c = integrate_cmds(cmds, 0.0, 0.2)
    assert c["turn"] == pytest.approx(0.2) and c["known"] and c["vx_min"] == 0.2
    c = integrate_cmds(cmds, 0.2, 1.0)        # last command counts for 0.25 s only
    assert c["turn"] == pytest.approx(-0.125) and not c["known"]
    assert integrate_cmds([], 0.0, 1.0)["known"] is False


def test_heading_carried_by_commands_then_pulled_to_travel():
    from fc.eye import TURN_GAIN, RobotTrack
    p = {"eye_turn_lag_s": 0.0, "fast_yaw_bias_deg": 0.0}
    tr = RobotTrack()
    tr.on_tag(0.0, 0.0, (0.0, 0.0), [], p)
    cmds = [(i * 0.033, 0.1, 0.0, 1.0) for i in range(16)]       # slow turn in place, 1 rad/s for ~0.5 s
    yaw = tr.carry_yaw(0.5, cmds, p)
    assert yaw == pytest.approx(TURN_GAIN * 0.5, abs=0.03) and tr.conf < 1.0
    # walking fast and straight, but along 30 deg while the carried heading says 0: travel wins over ~1 s
    tr = RobotTrack()
    tr.on_tag(0.0, 0.0, (0.0, 0.0), [], p)
    a = math.radians(30)
    cmds = [(i * 0.033, 0.7, 0.0, 0.0) for i in range(40)]
    for i in range(1, 37):
        t = i * 0.033
        tr.fixes.append((t, 0.6 * t * math.cos(a), 0.6 * t * math.sin(a)))
        yaw = tr.carry_yaw(t, cmds, p)
    assert abs(math.degrees(yaw) - 30) < 4, math.degrees(yaw)
    # the fast gait's offset between walking direction and tag heading is learned at decoded tags
    tr.on_tag(1.2, math.radians(38), (0.6 * 1.2 * math.cos(a), 0.6 * 1.2 * math.sin(a)), cmds, p)
    assert math.degrees(tr.travel_bias["fast"]) == pytest.approx(-8, abs=2)


class _FlatCam:
    """Straight-down camera: 2 mm per pixel at any height, field origin at pixel (480, 270)."""
    def pixel_to_plane(self, uv, z):
        import numpy as np
        uv = np.asarray(uv, float).reshape(-1, 2)
        return np.stack([(uv[:, 0] - 480) * 0.002, (270 - uv[:, 1]) * 0.002], axis=1)


@needs_cv2
def test_blob_fix_when_the_tag_blurs_lands_where_the_tag_would():
    """Tag decoded once (heading reset, board offset learned), then the robot moves 6 cm with the tag
    unreadable: the board gives a fresh 'blob' observation at the right place; heading carried."""
    import numpy as np
    eye = Eye(load_field(), [BLUE], {"eye_board_m": 0.08}, source="synthetic")
    eye.perc = type("P", (), {"cam": _FlatCam(), "calibration_valid": True})()
    eye.robot_colors = {"blue": BLUE_HSV}
    mount = {"tag_height": 0.267, "tag_offset_x": -0.02, "yaw_offset": 0.0, "floor_size": 0.15}
    tr = eye._tracks["blue"]
    f = _board_frame(480, 270, side=40, angle=0)
    tag = np.array([[469, 259], [491, 259], [491, 281], [469, 281]], np.float32)   # TL TR BR BL: nose +y
    x, y, yaw = Eye.tag_pose(eye.perc.cam.pixel_to_plane(tag, 0.267), mount)
    eye._track_tag("blue", tr, f, tag, yaw, 10.0, mount, True)
    assert tr.n_off == 1 and math.hypot(*tr.off) < 0.005
    f2 = _board_frame(480, 240, side=40, angle=0, blur=9)     # 30 px = 6 cm along +y, blurred
    obs = eye._track_blob("blue", BLUE.tag, tr, f2, 10.1, mount)
    assert obs is not None and obs.src == "blob" and obs.t == 10.1 and obs.yaw_t == 10.0
    assert math.hypot(obs.pose.x - x, obs.pose.y - (y + 0.06)) < 0.006, (obs.pose, x, y)
    assert obs.pose.yaw == pytest.approx(yaw) and 0 < obs.yaw_conf < 1
    assert eye._blob_seeds[BLUE.tag][:2] == pytest.approx((480, 240), abs=2)   # the tag crop follows the board
    # far too long since a decoded tag: no blob fix (a lost track never wanders off on its own)
    assert eye._track_blob("blue", BLUE.tag, tr, f2, 10.0 + eye.params.get("eye_blob_max_s", 3.0) + 0.5, mount) is None
    eye.robot_colors = {}
    assert eye._track_blob("blue", BLUE.tag, tr, f2, 10.2, mount) is None      # unknown colour: no blob tracking


@needs_cv2
def test_robot_colors_file_roundtrip_and_synthetic_ignores_it(tmp_path):
    from fc.eye import load_robot_colors, save_robot_colors
    path = tmp_path / "robot_colors.json"
    assert load_robot_colors(path) == {}
    save_robot_colors(path, "blue", BLUE_HSV, how="test")
    save_robot_colors(path, "red", [{"lo": [5, 100, 100], "hi": [15, 255, 255]}])
    assert load_robot_colors(path)["blue"] == BLUE_HSV and set(load_robot_colors(path)) == {"blue", "red"}
    assert Eye(load_field(), [BLUE], {}, source="synthetic", robot_colors_path=path).robot_colors == {}


def test_world_json_marks_blob_fixes_and_ctl_has_robotcal():
    from fc.app import world_json
    from fc.core import Pose, RobotObs, World
    from fc.ctl import fmt_robotcal
    w = World(5.0, None, {"blue": RobotObs("blue", Pose(0, 0, 0), 4.9, src="blob", yaw_t=4.0, yaw_conf=0.6),
                          "red": RobotObs("red", Pose(1, 0, 0), 4.9)})
    j = world_json(w, 5.0)
    assert j["robots"]["blue"]["src"] == "blob" and j["robots"]["blue"]["yc"] == 0.6 and "src" not in j["robots"]["red"]
    lines = fmt_robotcal({"robot": "blue", "colors": BLUE_HSV, "source": "x", "candidates": 2,
                          "fixes": {"tag": 3, "blob": 9, "miss": 0, "old": 0, "yaw_conf": 0.5, "off_cm": [0, 0]}})
    assert any("want exactly 1" in ln for ln in lines)
