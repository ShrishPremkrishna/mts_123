"""Ball Eye geometry, synthetic-sphere measurement and worker timing without a camera.

Synthetic frames are ray traced with the same lens and mount the detector uses,
so errors here are pipeline errors, not physical camera accuracy.
"""
import json
import math
from pathlib import Path
import threading
import time

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

ROOT = Path(__file__).resolve().parents[2]
ROBOT = ROOT / "follow_me_pupper/robot"
MODEL = ROOT / "outputs/pupper-fc-reconcile-20261122/pi/.config/gemini-pupper/ball_eye.onnx"
REAL = sorted((ROOT / "pupperv3-monorepo/ai/playground/image-description-benchmark/src/"
               "image_description_benchmark/images").glob("camera_image_raw_compressed-*.jpg"))
STAMP = 1_000_000_000


@pytest.fixture
def eye_mod(monkeypatch):
    monkeypatch.syspath_prepend(str(ROBOT))
    import ball_eye
    return ball_eye


@pytest.fixture
def profile(eye_mod):
    lens = eye_mod.Lens.from_camera_params(ROBOT / "camera_params.yaml")
    return eye_mod.MetricProfile("synthetic", .11, "test radius", False, eye_mod.ESTIMATED_MOUNT,
                                 "u6 estimate", False, eye_mod.ESTIMATED_BASE_HEIGHT_M, lens)


def render(profile, xy, background=None, **kwargs):
    from final_project.perception.synthetic_ball import plain_background, render_ball
    background = plain_background(profile.lens) if background is None else background
    return render_ball(background, profile.lens, profile.mount, xy, profile.ball_radius_m,
                       profile.base_height_m, **kwargs)


def test_circle_and_body_round_trip_through_cone_geometry(eye_mod, profile):
    # x = 0.25 m is about the nearest a 0.11 m ball can be to this estimated mount.
    for x, y in ((.25, 0.), (.258, .04), (.4, -.2), (.9, .3), (2.0, -.4)):
        centre = np.array([x, y, profile.ball_radius_m - profile.base_height_m])
        circle = eye_mod.body_to_circle(profile.view, profile.mount, centre, profile.ball_radius_m)
        back, rng, rho = eye_mod.circle_to_body(profile.view, profile.mount, circle, profile.ball_radius_m)
        np.testing.assert_allclose(back, centre, atol=1e-9)
        assert rng == pytest.approx(np.linalg.norm(profile.mount.to_optical(centre)))


def test_view_lens_and_panorama_projections_invert(eye_mod, profile):
    view = profile.view
    u, v = np.meshgrid(np.linspace(5, 630, 9), np.linspace(5, 630, 9))
    uu, vv = view.pixel(view.ray(u, v))
    np.testing.assert_allclose(uu, u, atol=1e-9)
    np.testing.assert_allclose(vv, v, atol=1e-9)
    rays, seen = profile.lens.rays()
    sample = rays[::97, ::89][seen[::97, ::89]]
    pu, pv, valid = profile.lens.project(sample)
    grid_v, grid_u = np.mgrid[0:profile.lens.height:97, 0:profile.lens.width:89]
    np.testing.assert_allclose(pu, grid_u[seen[::97, ::89]], atol=1e-6)
    np.testing.assert_allclose(pv, grid_v[seen[::97, ::89]], atol=1e-6)
    pano = eye_mod.Pano()
    assert pano.height == 572  # the FC camera node's fixed 700 x 572 detection image
    axis = np.array([.3, .4, .8]) / np.linalg.norm([.3, .4, .8])
    back, rho, whole = pano.cone(pano.box(axis, .12))
    assert whole and rho == pytest.approx(.12, abs=1e-6)
    np.testing.assert_allclose(back, axis, atol=1e-6)


def test_close_ball_round_on_sphere_is_whole_but_pole_crossing_box_is_a_seed(eye_mod):
    pano = eye_mod.Pano()
    # A close ball low in the panorama has a very non-square pixel box, yet the
    # spherical radius test keeps it whole; the pixel aspect test would reject it.
    lat = math.radians(60)
    axis = np.array([0., math.sin(lat), math.cos(lat)])
    box = pano.box(axis, math.radians(20))
    width, height = (box[2] - box[0]) * pano.width, (box[3] - box[1]) * pano.height
    assert min(width, height) / max(width, height) < .6
    assert pano.cone(box)[2]
    assert not pano.cone(pano.box(np.array([0., math.sin(math.radians(70)), math.cos(math.radians(70))]),
                                  math.radians(20)))[2]


def test_profile_selects_one_radius_and_identifies_geometry(eye_mod, profile):
    record = profile.record()
    assert eye_mod.MetricProfile.read(record) == profile
    other_radius = eye_mod.MetricProfile.read({**record, "ball": {**record["ball"], "radius_m": .091}})
    assert other_radius.profile_id != profile.profile_id and other_radius.calibration_id == profile.calibration_id
    moved = eye_mod.MetricProfile.read({**record, "mount": {**record["mount"], "tilt_up_deg": 12.}})
    assert moved.calibration_id != profile.calibration_id
    with pytest.raises(ValueError, match="measured radius and mount"):
        eye_mod.MetricProfile.read({**record, "status": "measured"})
    with pytest.raises(ValueError, match="ball radius"):
        eye_mod.MetricProfile.read({**record, "ball": {**record["ball"], "radius_m": None}})
    with pytest.raises(ValueError, match="lens"):
        eye_mod.MetricProfile.read({k: v for k, v in record.items() if k != "lens"})
    from_file = eye_mod.MetricProfile.read({k: v for k, v in record.items() if k != "lens"},
                                           camera_params=ROBOT / "camera_params.yaml")
    assert from_file.calibration_id == profile.calibration_id
    # Range is R / sin(rho): the same image circle read with the u6 donor radius is 17 % short.
    circle = eye_mod.body_to_circle(profile.view, profile.mount, np.array([.5, 0., -.04]), .11)
    _, r_selected, _ = eye_mod.circle_to_body(profile.view, profile.mount, circle, .11)
    _, r_donor, _ = eye_mod.circle_to_body(profile.view, profile.mount, circle, .091)
    assert r_donor / r_selected == pytest.approx(.091 / .11)


def test_image_uncertainty_grows_with_range_and_box_exceeds_fit(eye_mod, profile):
    sigmas = []
    for x in (.3, .6, 1.2):
        c = eye_mod.body_to_circle(profile.view, profile.mount, np.array([x, 0., -.04]), .11)
        sigmas.append(eye_mod.image_uncertainty(profile.view, profile.mount, c, .11, .5))
    assert sigmas[0][0] < sigmas[1][0] < sigmas[2][0] and sigmas[0][1] < sigmas[2][1]


@pytest.mark.parametrize("xy", [(.30, 0.), (.40, .10), (.55, -.15), (.258, .04)])
def test_synthetic_sphere_measured_through_real_remap_and_outline_fit(eye_mod, profile, xy):
    _, rgb, label = render(profile, xy)
    result = eye_mod.BallEye(profile).measure(rgb, methods=("pinkfloor",))
    assert result.state == "measured", result.notes
    m = result.candidates[result.selected]
    assert m.method == "pinkfloor_fit"
    assert math.hypot(m.x - label.x, m.y - label.y) < max(.006, 3 * m.sigma_xy_m)
    assert m.range_m == pytest.approx(label.range_m, rel=.03)


def test_maintained_pink_panorama_box_seeds_a_metric_fit(eye_mod, profile, monkeypatch):
    from fisheye_converter import equirect_height, fisheye_to_equirectangular, load_camera_model
    from pink_ball import PinkBallTracker
    _, rgb, label = render(profile, (.45, .08))
    model = load_camera_model(ROBOT / "camera_params.yaml", 1400, 1050)
    pano = fisheye_to_equirectangular(rgb, model, 700, equirect_height(700))
    tracker = PinkBallTracker()
    tracker.observe(pano, [], STAMP, "pink")
    row = tracker.observe(pano, [], STAMP + 100_000_000, "pink")
    assert row["state"] == "tracked"
    result = eye_mod.BallEye(profile).measure(rgb, pink_box=tuple(row["box"]), methods=("pink",))
    m = result.candidates[result.selected]
    assert m.method == "pink_fit" and math.hypot(m.x - label.x, m.y - label.y) < .01


def test_two_disagreeing_fits_are_a_conflict_not_a_selection(eye_mod, profile):
    eye = eye_mod.BallEye(profile)
    base = dict(z=-.04, range_m=.5, rho=.2, circle=eye_mod.Circle(1, 1, 40), score=1., sigma_px=.5,
                sigma_xy_m=.003, sigma_range_m=.003, fit_rms_px=.5, fit_points=40)
    a = eye_mod.Measurement("eye_fit", x=.5, y=0., **base)
    b = eye_mod.Measurement("pinkfloor_fit", x=.6, y=0., **base)
    notes = {}
    assert eye.select((a, b), notes) == (None, "conflict") and "disagree" in notes["selection"]
    assert eye.select((a, eye_mod.Measurement("pinkfloor_fit", x=.505, y=0., **base)), {}) == (0, "measured")


@pytest.mark.skipif(not MODEL.is_file(), reason="recovered Ball Eye model is not present locally")
def test_recovered_model_loads_by_content_hash_and_enforces_its_input(eye_mod):
    backend = eye_mod.OnnxBackend(MODEL, expected_sha256=eye_mod.RECOVERED_MODEL_SHA256)
    assert backend.sha256 == eye_mod.RECOVERED_MODEL_SHA256
    assert backend.infer(np.zeros((320, 320, 3), np.uint8)) == []
    with pytest.raises(ValueError, match="320"):
        backend.infer(np.zeros((640, 640, 3), np.uint8))
    with pytest.raises(ValueError, match="sha256"):
        eye_mod.OnnxBackend(MODEL, expected_sha256="0" * 64)


@pytest.mark.skipif(not MODEL.is_file() or not REAL, reason="recovered model or saved real frames are absent")
def test_real_ball_free_frames_produce_no_measurement(eye_mod, profile):
    eye = eye_mod.BallEye(profile, eye_mod.OnnxBackend(MODEL, expected_sha256=eye_mod.RECOVERED_MODEL_SHA256))
    for path in REAL:
        result = eye.measure(cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB))
        assert result.state == "searching" and not result.candidates, (path.name, result)


class SlowEye:
    def __init__(self, profile, delay):
        self.profile, self.delay = profile, delay
        self.started = threading.Event()
        self.release = threading.Event()

    def measure(self, rgb, pink_box=None, pano=None):
        self.started.set()
        self.release.wait(self.delay)
        return type("R", (), dict(state="searching", selected=None, candidates=(), notes={}))()


def job(eye_mod, stamp, frame=None):
    frame = np.zeros((4, 4, 3), np.uint8) if frame is None else frame
    return eye_mod.MetricJob(frame, True, stamp, stamp, 7, None)


def test_slow_metric_work_never_blocks_submit_or_publishes_an_expired_capture(eye_mod, profile):
    from onboard_ball import MetricBallReceiver
    now = {"ns": STAMP}
    published = []
    slow = SlowEye(profile, 5.)
    worker = eye_mod.MetricWorker(slow, published.append,
                                  lambda j: 0 <= now["ns"] - j.captured_at_ros_ns <= 350_000_000)
    try:
        assert worker.submit(job(eye_mod, STAMP))
        assert slow.started.wait(1.)
        started = time.perf_counter()
        assert worker.submit(job(eye_mod, STAMP + 100_000_000))
        assert worker.submit(job(eye_mod, STAMP + 200_000_000))  # replaces the pending capture
        assert not worker.submit(job(eye_mod, STAMP + 150_000_000))  # reordered capture refused
        assert time.perf_counter() - started < .05
        now["ns"] = STAMP + 400_000_000  # the in-flight capture is now too old to publish
        slow.release.set()
        deadline = time.monotonic() + 2
        while worker.counts["published"] + worker.counts["expired"] < 2 and time.monotonic() < deadline:
            time.sleep(.01)
    finally:
        assert worker.close()
    assert worker.counts == dict(submitted=3, overwritten=1, expired=1, published=1, failed=0)
    row = json.loads(published[0])
    assert row["captured_at_ros_ns"] == STAMP + 200_000_000 and row["frame_sequence"] == 7
    receiver = MetricBallReceiver()
    assert receiver.accept(published[0], STAMP + 400_000_000, (STAMP + 400_000_000) / 1e9), receiver.reason
    # Arrival far later cannot renew it: the same row is stale by its own capture time.
    assert receiver.latest(STAMP + 600_000_000, (STAMP + 600_000_000) / 1e9) is None


def test_worker_payload_from_a_synthetic_frame_is_admissible_with_identity(eye_mod, profile):
    from onboard_ball import MetricBallReceiver
    from final_project.pupper_fc.local_ball import LocalBallObservation
    jpeg, rgb, label = render(profile, (.35, -.05))
    published = []
    worker = eye_mod.MetricWorker(eye_mod.BallEye(profile), published.append, lambda j: True)
    try:
        payload = worker.process(eye_mod.MetricJob(rgb, True, STAMP, STAMP + 5, 3, None))
    finally:
        worker.close()
    receiver = MetricBallReceiver()
    assert receiver.accept(payload, STAMP + 100_000_000, (STAMP + 5 + 100_000_000) / 1e9), receiver.reason
    observation = LocalBallObservation.from_metric_row(receiver.row)
    assert observation.captured_at == (STAMP + 5) / 1e9
    assert observation.ball_radius_m == .11 and observation.profile_id == profile.profile_id
    assert observation.calibration_id == profile.calibration_id and observation.profile_status == "synthetic"
    assert math.hypot(observation.x - label.x, observation.y - label.y) < .006


def test_shipped_estimated_profile_uses_the_selected_field_radius_and_camera_lens(eye_mod):
    arena = json.loads((ROOT / "final_project/pupper_fc/arena.json").read_text())
    record = json.loads((ROBOT / "ball_eye_profile.json").read_text())
    selected = eye_mod.MetricProfile.read(record, camera_params=ROBOT / "camera_params.yaml")
    assert selected.ball_radius_m == arena["field"]["ball_radius"]
    assert selected.status == "estimated" and not selected.radius_measured and not selected.mount_measured
    assert selected.lens == eye_mod.Lens.from_camera_params(ROBOT / "camera_params.yaml")
    assert selected.mount == eye_mod.ESTIMATED_MOUNT and selected.base_height_m == eye_mod.ESTIMATED_BASE_HEIGHT_M
