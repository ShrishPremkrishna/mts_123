"""End-to-end checks on rendered frames with known ground truth."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam import planner  # noqa: E402
from fieldcam.camera_model import default_intrinsics  # noqa: E402
from fieldcam.config import SIM_CONFIG, load_config  # noqa: E402
from fieldcam.filters import wrap  # noqa: E402
from fieldcam.pipeline import Perception  # noqa: E402
from fieldcam.sim import SimArena  # noqa: E402

SIZE = (1920, 1080)


def make(cam_pos, hfov=69.0, **kw):
    cfg = load_config(SIM_CONFIG)
    cfg["camera"]["intrinsics"] = "config/_none_.yaml"  # tests never use a real calibration file
    cfg["camera"]["hfov_deg"] = hfov
    K, _ = default_intrinsics(*SIZE, hfov)
    arena = SimArena(cfg, K, cam_pos=cam_pos, size=SIZE, **kw)
    perc = Perception(cfg, *SIZE)
    assert perc.calibrate(arena.render({}, None)), "calibration on empty field failed"
    return cfg, arena, perc


CAMERAS = {
    "overhead": dict(cam_pos=(0.0, -0.2, 2.6)),
    "oblique_side": dict(cam_pos=(0.0, -2.2, 2.2)),
}


@pytest.mark.parametrize("cam", CAMERAS)
def test_static_localisation(cam):
    cfg, arena, perc = make(**CAMERAS[cam])
    assert perc.calib_err_px < 1.5
    rng = np.random.default_rng(1)
    pos_err, yaw_err, ball_err = [], [], []
    for _ in range(12):
        truth = {
            "striker": (rng.uniform(-1.1, 0.0), rng.uniform(-0.6, 0.6), rng.uniform(-math.pi, math.pi)),
            "keeper": (rng.uniform(0.4, 1.1), rng.uniform(-0.6, 0.6), rng.uniform(-math.pi, math.pi)),
        }
        ball = (rng.uniform(-1.2, 1.2), rng.uniform(-0.8, 0.8))
        if min(math.hypot(ball[0] - r[0], ball[1] - r[1]) for r in truth.values()) < 0.3:
            continue  # ball hidden under a robot is a separate (occlusion) case
        perc2 = Perception(cfg, *SIZE)  # fresh filters each scene, same calibration
        perc2.cam, perc2.roi_mask, perc2.ball_radius_px = perc.cam, perc.roi_mask, perc.ball_radius_px
        st = perc2.process(arena.render(truth, ball), t_capture=0.0)
        for name, (x, y, yaw) in truth.items():
            r = st["robots"][name]
            assert r["visible"], f"{name} tag not detected"
            pos_err.append(math.hypot(r["x"] - x, r["y"] - y))
            yaw_err.append(abs(wrap(r["yaw"] - yaw)))
        assert st["ball"]["visible"], "ball not detected"
        ball_err.append(math.hypot(st["ball"]["x"] - ball[0], st["ball"]["y"] - ball[1]))
    print(f"\n[{cam}] robot pos err max {max(pos_err) * 100:.1f} cm, "
          f"yaw err max {math.degrees(max(yaw_err)):.1f} deg, ball err max {max(ball_err) * 100:.1f} cm")
    assert max(pos_err) < 0.02
    assert max(yaw_err) < math.radians(3)
    assert max(ball_err) < 0.02


def test_ball_velocity_and_dropout():
    cfg, arena, perc = make(**CAMERAS["overhead"])
    v = np.array([0.8, -0.3])
    p0 = np.array([-1.0, 0.5])
    st = None
    for k in range(20):
        t = k / 30
        st = perc.process(arena.render({}, p0 + v * t), t_capture=t)
    assert abs(st["ball"]["vx"] - v[0]) < 0.1 and abs(st["ball"]["vy"] - v[1]) < 0.1
    # Ball disappears: coast on prediction, then report lost.
    t = 20 / 30 + 0.2
    st = perc.process(arena.render({}, None), t_capture=t)
    truth = p0 + v * t
    assert st["ball"]["visible"] and math.hypot(st["ball"]["x"] - truth[0], st["ball"]["y"] - truth[1]) < 0.06
    st = perc.process(arena.render({}, None), t_capture=t + 1.0)
    assert not st["ball"]["visible"]


def test_latency_extrapolation():
    """State is reported for t_now, not for the (older) capture time."""
    cfg, arena, perc = make(**CAMERAS["overhead"])
    v, p0, lat = np.array([1.0, 0.0]), np.array([-1.0, 0.0]), 0.15
    for k in range(20):
        t = k / 30
        st = perc.process(arena.render({}, p0 + v * t), t_capture=t, t_now=t + lat)
    truth_now = p0 + v * (t + lat)
    assert abs(st["ball"]["x"] - truth_now[0]) < 0.03
    assert abs(st["ball"]["age_s"] - lat) < 1e-6


def test_shot_geometry():
    cfg = load_config(SIM_CONFIG)
    goal = cfg["goals"]["east"]
    target = planner.pick_target((0.0, 0.5), goal, 0.1)
    assert target.tolist() == [1.5, 0.0]
    shot = planner.shot_geometry((0.0, 0.5), target, 0.3)
    sx, sy, syaw = shot["staging"]
    assert math.isclose(math.hypot(sx - 0.0, sy - 0.5), 0.3, abs_tol=1e-9)
    # staging, ball and target are collinear, and yaw points at the target
    assert abs(wrap(math.atan2(target[1] - sy, target[0] - sx) - syaw)) < 1e-9
    # keeper standing left of centre -> aim at the right-hand gap
    assert planner.pick_target((0.0, 0.0), goal, 0.1, keeper_xy=(1.4, 0.1))[1] < 0


@pytest.mark.parametrize("start", [(-1.0, -0.5, 0.0), (1.0, 0.3, math.pi), (0.6, 0.0, math.pi / 2)])
def test_approach_reaches_staging_without_hitting_ball(start):
    """Kinematic robot following approach_cmd: must end 'ready' and never clip the ball."""
    cfg = load_config(SIM_CONFIG)
    ball = {"visible": True, "x": 0.2, "y": 0.1, "vx": 0.0, "vy": 0.0, "age_s": 0.03}
    shot = planner.shot_geometry((ball["x"], ball["y"]),
                                 planner.pick_target((ball["x"], ball["y"]), cfg["goals"]["east"], 0.1),
                                 cfg["shot"]["standoff"])
    x, y, yaw = start
    dt, min_ball_dist, ready = 0.05, 1e9, False
    for _ in range(600):
        robot = {"visible": True, "x": x, "y": y, "yaw": yaw, "age_s": 0.03}
        if planner.shot_ready(robot, ball, shot, cfg["shot"]):
            ready = True
            break
        cmd = planner.approach_cmd(robot, ball, shot, cfg["approach"])
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * cmd["vx"] - s * cmd["vy"]) * dt
        y += (s * cmd["vx"] + c * cmd["vy"]) * dt
        yaw = wrap(yaw + cmd["wz"] * dt)
        min_ball_dist = min(min_ball_dist, math.hypot(x - ball["x"], y - ball["y"]))
    assert ready, f"never reached staging pose, ended at {(x, y, yaw)}"
    assert min_ball_dist > 0.2, f"walked through the ball (closest {min_ball_dist:.2f} m)"


# ---- calibration without floor tags: clicked points ---------------------------
@pytest.mark.parametrize("cam", CAMERAS)
@pytest.mark.parametrize("click_noise_px", [0.0, 4.0])
def test_click_calibration(cam, click_noise_px):
    """Clicking the four field corners (with sloppy clicks) localises about as well as tags."""
    cfg, arena, _ = make(**CAMERAS[cam])
    points = cfg["manual_points"]
    field_xy = [p["xy"] for p in points]
    rng = np.random.default_rng(3)
    clicks = arena.cam.project([[x, y, 0.0] for x, y in field_xy])
    clicks = clicks + rng.uniform(-click_noise_px, click_noise_px, clicks.shape)

    perc = Perception(cfg, *SIZE)
    assert not perc.calibrate_from_points(clicks[:3], field_xy[:3]), "3 points must be refused"
    assert perc.calibrate_from_points(clicks, field_xy)
    pos_err, yaw_err, ball_err = [], [], []
    for _ in range(10):
        truth = {
            "striker": (rng.uniform(-1.1, 0.0), rng.uniform(-0.6, 0.6), rng.uniform(-math.pi, math.pi)),
            "keeper": (rng.uniform(0.4, 1.1), rng.uniform(-0.6, 0.6), rng.uniform(-math.pi, math.pi)),
        }
        ball = (rng.uniform(-1.2, 1.2), rng.uniform(-0.8, 0.8))
        if min(math.hypot(ball[0] - r[0], ball[1] - r[1]) for r in truth.values()) < 0.3:
            continue
        perc2 = Perception(cfg, *SIZE)
        perc2.cam, perc2.roi_mask, perc2.ball_radius_px = perc.cam, perc.roi_mask, perc.ball_radius_px
        st = perc2.process(arena.render(truth, ball), t_capture=0.0)
        for name, (x, y, yaw) in truth.items():
            r = st["robots"][name]
            assert r["visible"]
            pos_err.append(math.hypot(r["x"] - x, r["y"] - y))
            yaw_err.append(abs(wrap(r["yaw"] - yaw)))
        assert st["ball"]["visible"]
        ball_err.append(math.hypot(st["ball"]["x"] - ball[0], st["ball"]["y"] - ball[1]))
    print(f"\n[{cam}, clicks +-{click_noise_px:.0f}px] camera off by "
          f"{np.linalg.norm(perc.cam.center - arena.cam.center) * 100:.1f} cm, "
          f"robot pos err max {max(pos_err) * 100:.1f} cm, yaw err max {math.degrees(max(yaw_err)):.1f} deg, "
          f"ball err max {max(ball_err) * 100:.1f} cm")
    limit = 0.02 if click_noise_px == 0 else 0.04
    assert max(pos_err) < limit and max(ball_err) < limit
    assert max(yaw_err) < math.radians(4)


def test_click_window_flow(monkeypatch, tmp_path):
    """The click loop: one click per point, undo, window scaling, cancel, save/load."""
    import cv2

    from fieldcam import clickcal

    points = load_config(SIM_CONFIG)["manual_points"]
    frame = np.zeros((1080, 1920, 3), np.uint8)
    handler = {}
    monkeypatch.setattr(cv2, "imshow", lambda *_a: None)
    monkeypatch.setattr(cv2, "setMouseCallback", lambda _w, fn: handler.update(fn=fn))

    def run(script, scale):
        script = list(script)

        def wait_key(_ms):
            step = script.pop(0)
            if isinstance(step, tuple):
                handler["fn"](cv2.EVENT_LBUTTONDOWN, step[0], step[1], 0, None)
                return 255
            return step

        monkeypatch.setattr(cv2, "waitKey", wait_key)
        return clickcal.collect_clicks("w", frame, points, scale)

    # A wrong click, undone, then the four real ones; window shown at half size.
    pts = run([(5, 5), ord("u"), (100, 500), (900, 500), (800, 100), (200, 100)], scale=0.5)
    assert pts.tolist() == [[200, 1000], [1800, 1000], [1600, 200], [400, 200]]
    assert run([(100, 500), 27], scale=1.0) is None  # Esc cancels

    path = tmp_path / "clicks.json"
    clickcal.save_clicks(path, pts, points, (1920, 1080))
    assert clickcal.load_clicks(path, points, (1920, 1080)).tolist() == pts.tolist()
    assert clickcal.load_clicks(path, points, (1280, 720)) is None  # other resolution
    assert clickcal.load_clicks(path, points[:3], (1920, 1080)) is None  # other point list
    assert clickcal.load_clicks(tmp_path / "missing.json", points, (1920, 1080)) is None


# ---- no-setup measuring: distances between tags, from the tags alone ------------
@pytest.mark.parametrize("cam", CAMERAS)
@pytest.mark.parametrize("ids", [(0, 1, 2), (0, 4, 5), (3, 4, 1)])
def test_tag_distances_from_three_tags(cam, ids):
    """Three tags of known size, positions unknown: recover the distances between them."""
    import cv2

    from fieldcam.detect import TagDetector
    from fieldcam.tagmeasure import pair_distances, survey_tags

    cfg, arena, _ = make(**CAMERAS[cam])
    truth = cfg["tags"]["field"]
    worst = {}
    for label, refine in (("reference tag only", False), ("all three tags", True)):
        errs = []
        for _ in range(5):  # different image noise each time
            tags = TagDetector().detect(cv2.cvtColor(arena.render({}, None), cv2.COLOR_BGR2GRAY))
            tags = {i: tags[i] for i in ids}
            poses, _ = survey_tags(tags, {i: 0.15 for i in ids}, ref_id=ids[0], refine=refine)
            for (a, b), d in pair_distances(poses).items():
                errs.append(abs(d - float(np.linalg.norm(truth[a] - truth[b]))))
        worst[label] = max(errs)
    print(f"\n[{cam}, tags {ids}] worst distance error: "
          + ", ".join(f"{k} {v * 100:.1f} cm" for k, v in worst.items()))
    assert worst["all three tags"] < 0.03
    assert survey_tags({}, {}, ref_id=0) == (None, None)


# ---- main pipeline calibrated from three tags, nothing measured ----------------
@pytest.mark.parametrize("cam", CAMERAS)
@pytest.mark.parametrize("hfov_guess", [69.0, 60.0])
def test_survey_calibration_three_tags(cam, hfov_guess):
    """Three floor tags at unmeasured spots are enough to track robots and the ball.

    The frame is anchored on the lowest tag (id 0 here, truly at (-1.40, -0.90)),
    so everything should come out shifted by exactly that. hfov_guess != 69 is a
    wrong guess at the lens, as on a phone that has not been lens-calibrated.
    """
    cfg, arena, _ = make(**CAMERAS[cam])
    cfg["tags"]["floor_size"] = 0.15  # what the simulator draws
    cfg["camera"]["hfov_deg"] = hfov_guess
    origin = cfg["tags"]["field"][0]
    perc = Perception(cfg, *SIZE)
    assert not perc.calibrate_from_survey(arena.render({}, None), use_ids={0})
    assert perc.calibrate_from_survey(arena.render({}, None), use_ids={0, 1, 5})
    assert perc.calib_mode == "survey" and perc.survey["ref"] == 0

    rng = np.random.default_rng(5)
    pos_err, yaw_err, ball_err = [], [], []
    for _ in range(10):
        # Scenes in the part of the arena that falls inside the survey frame's field.
        truth = {
            "striker": (rng.uniform(-1.2, -0.5), rng.uniform(-0.7, 0.2), rng.uniform(-math.pi, math.pi)),
            "keeper": (rng.uniform(-0.2, 0.3), rng.uniform(-0.7, 0.2), rng.uniform(-math.pi, math.pi)),
        }
        ball = (rng.uniform(-1.2, 0.2), rng.uniform(-0.8, 0.3))
        if min(math.hypot(ball[0] - r[0], ball[1] - r[1]) for r in truth.values()) < 0.3:
            continue
        perc2 = Perception(cfg, *SIZE)
        perc2.cam, perc2.roi_mask, perc2.ball_radius_px = perc.cam, perc.roi_mask, perc.ball_radius_px
        perc2.floor_layout = perc.floor_layout
        st = perc2.process(arena.render(truth, ball), t_capture=0.0)
        for name, (x, y, yaw) in truth.items():
            r = st["robots"][name]
            assert r["visible"]
            pos_err.append(math.hypot(r["x"] - (x - origin[0]), r["y"] - (y - origin[1])))
            yaw_err.append(abs(wrap(r["yaw"] - yaw)))
        assert st["ball"]["visible"]
        ball_err.append(math.hypot(st["ball"]["x"] - (ball[0] - origin[0]), st["ball"]["y"] - (ball[1] - origin[1])))
        assert st["calib_err_px"] < 3.0  # the surveyed tags double as the bumped-camera check
    print(f"\n[{cam}, 3 unmeasured tags, lens guess {hfov_guess:.0f} deg vs true 69] "
          f"robot pos err max {max(pos_err) * 100:.1f} cm, yaw err max {math.degrees(max(yaw_err)):.1f} deg, "
          f"ball err max {max(ball_err) * 100:.1f} cm")
    # A wrong lens guess costs more on the ball than before: its centre is 8 cm up, not on the floor.
    assert max(pos_err) < 0.05 and max(ball_err) < (0.03 if hfov_guess == 69.0 else 0.05)
    assert max(yaw_err) < math.radians(4)


# ---- every pixel has a floor position -----------------------------------------
@pytest.mark.parametrize("cam", CAMERAS)
def test_every_pixel_has_a_floor_position(cam):
    from fieldcam.probe import Probe, draw_grid, floor_map, pixel_to_floor

    cfg, arena, perc = make(**CAMERAS[cam])
    fmap = floor_map(perc.cam, *SIZE)
    assert fmap.shape == (SIZE[1], SIZE[0], 2)
    # Known floor points land on the right pixels, and those pixels map back to the points.
    pts = np.array([[0.0, 0.0], [1.2, 0.7], [-1.4, -0.9], [0.3, -0.8], [-1.0, 0.6]])
    px = arena.cam.project(np.column_stack([pts, np.zeros(len(pts))]))
    looked_up = np.array([fmap[int(round(v)), int(round(u))] for u, v in px])
    err = np.linalg.norm(looked_up - pts, axis=1)
    print(f"\n[{cam}] pixel -> floor error at 5 known points: max {err.max() * 100:.2f} cm")
    assert err.max() < 0.01
    assert np.allclose(pixel_to_floor(perc.cam, px), pts, atol=0.005)
    # Neighbouring pixels are neighbouring floor spots (a few mm apart at this height).
    step = np.linalg.norm(fmap[540, 961] - fmap[540, 960])
    assert 0.0005 < step < 0.01

    # A camera looking almost level sees the horizon: pixels above it have no floor position.
    from fieldcam.camera_model import CameraModel
    from fieldcam.sim import look_at
    level = CameraModel(perc.K, np.zeros(5), *look_at((0.0, -3.0, 0.5), (0.0, 0.0, 0.4)))
    lm = floor_map(level, *SIZE)
    assert np.isnan(lm[0]).all() and np.isfinite(lm[-1]).all()

    # The overlay draws without complaint, pins a click and reports it in field coordinates.
    img = arena.render({}, None)
    probe = Probe(scale=0.5)
    probe.grid = True
    import cv2
    probe.on_mouse(cv2.EVENT_LBUTTONDOWN, px[1][0] * 0.5, px[1][1] * 0.5, 0, None)
    probe.draw(img, perc.cam)
    assert np.allclose(probe.pins[0], pts[1], atol=0.01)
    assert draw_grid(img.copy(), level) > 0


# ---- bridge to the gameplay controller (final_project/pupper_fc) ---------------
def test_gameplay_observation_bridge():
    """Perception state -> gameplay Observation: capture-time poses, missing stays missing,
    and the real gameplay controller accepts it and produces a command."""
    from fieldcam.gameplay import to_observation

    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from final_project.pupper_fc.controller import Controller
    from final_project.pupper_fc.model import Field, Observation

    cfg, arena, perc = make(**CAMERAS["overhead"])
    truth = {"striker": (-0.6, -0.2, 0.4), "keeper": (1.1, 0.1, math.pi)}
    ball = (0.3, 0.2)
    # Frames 1/30 s apart, each arriving 0.12 s after capture (the camera latency).
    for k in range(12):
        t_cap = 100.0 + k / 30
        st = perc.process(arena.render(truth, ball), t_capture=t_cap, t_now=t_cap + 0.12)
    obs = to_observation(st, robot="striker", robot_id="pupper-1")
    assert isinstance(obs, Observation)
    assert abs(obs.captured_at - t_cap) < 1e-9           # when the camera saw it, not when we received it
    assert math.hypot(obs.robot.x - truth["striker"][0], obs.robot.y - truth["striker"][1]) < 0.02
    assert abs(wrap(obs.robot.yaw - truth["striker"][2])) < math.radians(3)
    assert math.hypot(obs.ball.x - ball[0], obs.ball.y - ball[1]) < 0.02
    assert obs.fallen is False and obs.goal == 0

    # The gameplay controller, on its own field model, takes it and asks the robot to move.
    field = Field(length=cfg["field"]["length"], width=cfg["field"]["width"], goal_width=cfg["field"]["goal_width"])
    decision = Controller(field).tick(obs, now=st["stamp"])
    assert decision.phase not in ("invalid_observation", "tracking_lost"), decision.reason
    assert abs(decision.command.vx) + abs(decision.command.vy) + abs(decision.command.wz) > 0.01

    # Ball gone from view: the very next frame reports no ball, even though the tracker still coasts.
    t_cap += 1 / 30
    st2 = perc.process(arena.render(truth, None), t_capture=t_cap, t_now=t_cap + 0.12)
    assert st2["ball"]["visible"]                         # the published state coasts for consumers that want it
    obs2 = to_observation(st2)
    assert obs2.ball is None and obs2.robot is not None   # gameplay is told the truth: not seen
    # Not calibrated: nothing is claimed.
    blank = Perception(cfg, *SIZE).process(arena.render(truth, ball), t_capture=1.0, t_now=1.1)
    obs3 = to_observation(blank)
    assert obs3.robot is None and obs3.ball is None
