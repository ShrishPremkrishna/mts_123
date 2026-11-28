"""Field calibration that heals itself (fc/eye.py, Nov 28).

Nov 27 floor session: fieldcam dropped the whole calibration when any one floor tag showed > 8 px off (the
corner tags sit on a flexible fence) and recovery needed all four corner tags in one frame, re-fitting the
lens each time (53-68.5 deg, field 3.38 -> 3.21 m mid-match). Here: one tag moved -> play on; camera moved ->
camera pose re-solved (lens and field frozen); restart -> eye_cal.json gives poses at once; a bad full re-fit
is rejected and the old calibration kept."""
import importlib.util
import json
import math
import types

import pytest

from fc.core import RobotCfg, load_field

HAVE_CV2 = importlib.util.find_spec("cv2") is not None
pytestmark = pytest.mark.skipif(not HAVE_CV2, reason="needs OpenCV (.venv-fc)")
if HAVE_CV2:
    import cv2
    import numpy as np

    from fc import eye as eye_mod
    from fc.eye import Eye, SyntheticSource

BLUE = RobotCfg("blue", "blue", 10, "", "stock")
W, H = 1920, 1080


def _camera(hfov=64.0, centre=(0.0, -0.25, 3.8), tilt_deg=(8.0, 0.0, 0.0), dist=None):
    """A camera above the field looking down (tilted toward +y by tilt_deg[0])."""
    from fieldcam.camera_model import CameraModel, default_intrinsics
    K, d0 = default_intrinsics(W, H, hfov)
    down = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])   # field -> camera, looking straight down
    tilt, _ = cv2.Rodrigues(np.radians(np.array(tilt_deg, float)).reshape(3, 1))
    R = tilt @ down
    rvec, _ = cv2.Rodrigues(R)
    tvec = -R @ np.array(centre, float).reshape(3, 1)
    return CameraModel(K, d0 if dist is None else np.array(dist, float), rvec, tvec)


def _tag_poses(field):
    L, Wd = field.length / 2, field.width / 2
    return {0: (-L, -Wd, 0.0), 1: (L, -Wd, 0.0), 2: (L, Wd, 0.0), 3: (-L, Wd, 0.0)}


def _observe(cam, poses, size=0.15, ids=None, shift=None):
    """Floor tags as the detector would report them: {id: corners (4, 2) TL TR BR BL}, optionally shifted (px)."""
    from fieldcam.tagmeasure import tag_corners
    out = {}
    for i, p in poses.items():
        if ids is not None and i not in ids:
            continue
        c = tag_corners(p, size)
        px = cam.project(np.hstack([c, np.zeros((4, 1))])).astype(np.float32)
        out[i] = px + np.float32((shift or {}).get(i, (0.0, 0.0)))
    return out


def _eye(tmp_path, name="eye_cal.json"):
    """An Eye with a stand-in source (no camera, no renderer): perception set up, nothing running."""
    e = Eye(load_field(), [BLUE], {}, source="synthetic", cal_path=tmp_path / name)
    e._open_source = lambda: types.SimpleNamespace(width=W, height=H, latency=False, close=lambda: None)
    e._setup()
    return e


def _calibrated(tmp_path):
    e = _eye(tmp_path)
    cam = _camera()
    poses = _tag_poses(e.field)
    ok, why = e._fit(np.zeros((H, W, 3), np.uint8), "full fit", tags=_observe(cam, poses))
    assert ok, why
    return e, cam, poses


def _feed(e, cam, poses, t0, t1, **kw):
    t = t0
    while t <= t1 + 1e-9:
        e._cal_check(_observe(cam, poses, **kw), t)
        t += 0.25
    return t


def _same_floor_point(cam_a, cam_b, uv=(700.0, 400.0), z=0.24):
    a, b = cam_a.pixel_to_plane([uv], z)[0], cam_b.pixel_to_plane([uv], z)[0]
    return float(np.hypot(*(a - b)))


def test_full_fit_saves_lens_pose_field_and_tags(tmp_path):
    e, cam, poses = _calibrated(tmp_path)
    assert e.perc.calibration_valid and e.perc.floor_layout == {}   # fieldcam's all-or-nothing check is off
    assert abs(e.cal.hfov - 64.0) <= 0.5 and e.cal.err < 1.0
    d = json.loads((tmp_path / "eye_cal.json").read_text())
    assert d["how"] == "full fit" and set(d["tags"]) == {"0", "1", "2", "3"} and len(d["ref_px"]) == 4
    assert d["field"]["length"] == pytest.approx(e.field.length, abs=0.01)
    assert _same_floor_point(e.perc.cam, cam) < 0.005


def test_one_tag_moved_is_ignored_and_play_goes_on(tmp_path):
    e, cam, poses = _calibrated(tmp_path)
    rvec, field = e.perc.cam.rvec.copy(), (e.cal.length, e.cal.width)
    t = _feed(e, cam, poses, 0.0, 2.0, shift={1: (25.0, -10.0)})   # the fence flexed under tag 1
    st = e.cal_status()
    assert st["state"] == "ok" and st["ignored"] == [1] and not st["suspect"], st
    assert e.perc.calibration_valid and np.allclose(e.perc.cam.rvec, rvec)   # same camera, World keeps coming
    assert (e.cal.length, e.cal.width) == field
    assert any("tag 1 moved" in ev[1] for ev in st["events"])
    # it stays put there: re-measured where it now sits, so it can verify the camera again
    _feed(e, cam, poses, t, t + eye_mod.READOPT_S + 1.0, shift={1: (25.0, -10.0)})
    assert e.cal_status()["ignored"] == [] and e.cal.tags[1][0] != pytest.approx(poses[1][0], abs=0.02)
    assert (e.cal.length, e.cal.width) == field


def test_camera_moved_re_solves_the_pose_only(tmp_path):
    """Nov 27 goalie run: the camera tilted, tag 1 left the picture, the other three shifted 21-31 px."""
    e, cam, poses = _calibrated(tmp_path)
    K, field, hfov = e.cal.K.copy(), (e.cal.length, e.cal.width), e.cal.hfov
    old_cam = e.perc.cam
    moved = _camera(centre=(0.03, -0.27, 3.8), tilt_deg=(9.0, 0.6, 0.8))
    _feed(e, moved, poses, 0.0, 0.75, ids={0, 2, 3})
    assert e.cal_status()["state"] == "suspect" and e.message.startswith("CAL SUSPECT")
    assert e.perc.calibration_valid and e.perc.cam is old_cam   # last pose keeps serving until a re-solve passes
    _feed(e, moved, poses, 1.0, 1.5, ids={0, 2, 3})
    st = e.cal_status()
    assert st["state"] == "ok" and st["pose_resolves"] == 1, st
    assert _same_floor_point(e.perc.cam, moved) < 0.005 and _same_floor_point(old_cam, moved) > 0.03
    assert np.allclose(e.cal.K, K) and (e.cal.length, e.cal.width) == field and e.cal.hfov == hfov
    d = json.loads((tmp_path / "eye_cal.json").read_text())
    assert d["how"] == "pose re-solve" and d["field"]["length"] == pytest.approx(field[0], abs=1e-3)
    # tag 1 comes back into view: its reference followed the new pose, nothing flagged
    _feed(e, moved, poses, 2.0, 3.0)
    assert e.cal_status()["state"] == "ok" and e.cal_status()["ignored"] == []


def test_camera_move_that_cannot_be_re_solved_stays_suspect_and_keeps_the_lens(tmp_path):
    e, cam, poses = _calibrated(tmp_path)
    hfov, old_cam = e.cal.hfov, e.perc.cam
    moved = _camera(centre=(0.03, -0.27, 3.8), tilt_deg=(9.5, 0.5, 0.5))
    _feed(e, moved, poses, 0.0, 3.0, ids={0, 2})   # two tags in view, both shifted
    st = e.cal_status()
    assert st["state"] == "suspect" and "needs 3 floor tags" in st["why"] and st["pose_resolves"] == 0, st
    assert e.perc.calibration_valid and e.perc.cam is old_cam and e.cal.hfov == hfov


def test_bad_re_fit_is_rejected_and_the_calibration_kept(tmp_path):
    e, cam, poses = _calibrated(tmp_path)
    old, saved = e.cal, (tmp_path / "eye_cal.json").read_text()
    barrel = _camera(dist=[-0.35, 0.1, 0, 0, 0])   # a lens the pinhole sweep cannot fit within 3 px
    ok, why = e._fit(np.zeros((H, W, 3), np.uint8), "operator re-fit", tags=_observe(barrel, poses))
    assert not ok and "rejected" in why, why
    assert e.cal is old and (tmp_path / "eye_cal.json").read_text() == saved and e.perc.calibration_valid
    # the operator path (recalibrate -> eye thread) reports it and keeps serving
    e._full_fit = lambda frame, tags=None: (None, "fit error 12.5 px > 3.0 px (eye_cal_max_px): rejected")
    assert e.recalibrate(timeout=0.0, wait=False)["pending"]
    e.process(np.zeros((H, W, 3), np.uint8), 1.0)
    assert e._refit is None and e.cal is old and e.perc.calibration_valid
    assert e.message.startswith("re-fit failed, calibration kept: fit error 12.5 px")


def test_good_operator_re_fit_replaces_lens_and_field(tmp_path):
    e, cam, poses = _calibrated(tmp_path)
    big = {i: (p[0] * 1.1, p[1] * 1.1, 0.0) for i, p in poses.items()}   # fence rebuilt 10 % larger
    ok, why = e._fit(np.zeros((H, W, 3), np.uint8), "operator re-fit", tags=_observe(cam, big))
    assert ok, why
    assert e.cal.length == pytest.approx(1.1 * e._cfg_size[0], abs=0.01) and "was" in why
    assert json.loads((tmp_path / "eye_cal.json").read_text())["how"] == "operator re-fit"


def _blank(frame, corners, grow=1.6):
    c = np.asarray(corners, np.float32)
    ctr = c.mean(axis=0)
    cv2.fillConvexPoly(frame, np.round((c - ctr) * grow + ctr).astype(np.int32), (90, 90, 90))


def _rendered_eye(tmp_path, field, cal_path):
    e = Eye(field, [BLUE], {}, source="synthetic", cal_path=cal_path)
    e._open_source = lambda: SyntheticSource(e.cfg, field, [BLUE], W, H)
    e._setup()
    return e


def test_restart_with_eye_cal_gives_poses_at_once_without_tags(tmp_path):
    from fieldcam.detect import TagDetector
    field, path = load_field(), tmp_path / "eye_cal.json"
    a = _rendered_eye(tmp_path, field, path)
    frame, _ = a.src.read()
    ok, why = a._fit(frame, "full fit")
    assert ok, why
    b = _rendered_eye(tmp_path, field, path)   # restart
    assert b.perc.calibration_valid and b.cal_status()["state"] == "unverified"
    assert (b.cal.length, b.cal.width) == pytest.approx((a.cal.length, a.cal.width), abs=1e-3) and b.cal.hfov == a.cal.hfov
    b._full_fit = lambda *a_, **k: pytest.fail("a restart with eye_cal.json must not re-fit")
    frame, t = b.src.read()
    for tid, c in TagDetector().detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)).items():
        if tid != BLUE.tag:
            _blank(frame, c)   # people standing on every corner tag
    b.scan_once(frame, t, now=t)
    w = b.process(frame, t)
    assert w is not None and "blue" in w.robots, b.message
    x, y, yaw = b.src.truth["robots"]["blue"]
    o = w.robots["blue"].pose
    assert math.hypot(o.x - x, o.y - y) < 0.02
    assert b.cal_status()["state"] == "unverified"   # nothing to verify against yet, and said so


def test_no_calibration_reports_the_real_reason(tmp_path):
    """The warm-up of Nov 27 said 'floor_reference_drift' while the real reason was a hidden corner tag."""
    from fieldcam.detect import TagDetector
    e = _rendered_eye(tmp_path, load_field(), None)
    frame, t = e.src.read()
    _blank(frame, TagDetector().detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))[1])
    assert e.process(frame, t) is None
    assert e.message == "not calibrated: corner tags not completely in view: tag 1 (east-south)", e.message


def test_site_calibration_is_only_for_the_live_camera(tmp_path):
    assert Eye(load_field(), [BLUE], {}, source="synthetic").cal_path is None
    assert Eye(load_field(), [BLUE], {}, source="phone").cal_path == eye_mod.EYE_CAL_PATH
    assert Eye(load_field(), [BLUE], {}, source="synthetic", cal_path=tmp_path / "x.json").cal_path == tmp_path / "x.json"


def test_eye_cal_scales_to_another_picture_size_and_rejects_a_mismatch(tmp_path):
    e, cam, poses = _calibrated(tmp_path)
    path = tmp_path / "eye_cal.json"
    small, why = eye_mod.load_cal(path, 1280, 720, 0.15)
    assert small is not None and small.K[0, 0] == pytest.approx(e.cal.K[0, 0] * 1280 / 1920)
    a, b = small.camera().pixel_to_plane([(700 * 2 / 3, 400 * 2 / 3)], 0.24)[0], cam.pixel_to_plane([(700, 400)], 0.24)[0]
    assert float(np.hypot(*(a - b))) < 0.005
    assert eye_mod.load_cal(path, 1280, 960, 0.15)[0] is None
    assert "eye_floor_size" in eye_mod.load_cal(path, W, H, 0.17)[1]
    assert eye_mod.load_cal(tmp_path / "none.json", W, H, 0.15) == (None, "")
