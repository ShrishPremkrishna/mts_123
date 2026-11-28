"""Field size from the camera calibration (one live Field everywhere) and ball colour calibration."""
import json

import pytest

from fc.app import App
from fc.brain import merged
from fc.core import Ball, World


class FakeEye:
    def __init__(self, size):
        self.size = size

    def calibrated_size(self):
        return self.size


def test_camera_field_size_reaches_brain_referee_flow_and_run_log(tmp_path):
    app = App("sim", dry_run_ops=True, runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    try:
        assert app.field.goal_width == 1.22           # field.json: 4 ft mouth
        app.eye, app.sim = FakeEye((3.50, 2.80)), None
        app._sync_field()
        f = app.field
        assert (f.length, f.width, app.field_source) == (3.5, 2.8, "camera")
        for b in app.brains.values():                 # brain geometry reads the same object
            assert b.f is f and abs(b.own_x) == 1.75 and b.att_x == -b.own_x
            assert b._clamp_field((9, 9), merged({})) == pytest.approx((1.75 - 0.28, 1.40 - 0.28))   # fence margin
        # referee: goal line at the calibrated x, mouth from field.json
        w = lambda x, y: World(t=0.0, ball=Ball(x, y, 0, 0, 0.0))   # noqa: E731
        assert app._goal_events(w(1.60, 0.0), 0.0) == []           # old line (1.34) no longer scores
        assert app._goal_events(w(1.76, 0.55), 0.0) == ["blue"]    # inside a 1.22 m mouth
        app._goal_cooldown = 0.0
        assert app._goal_events(w(-1.76, 0.65), 0.0) == []         # outside the mouth
        app.params["record"] = False
        with app.lock:
            app._begin("unit", None)
            meta = json.loads((app.run.dir / "params.json").read_text())
            app._finish("ok")
        assert meta["field"]["length"] == 3.5 and meta["field_source"] == "camera"
        from fc import ctl
        st = {"field": {"length": 3.5, "width": 2.8, "goal_width": 1.22, "ball_radius": 0.09},
              "field_source": app.field_source, "eye": {"fps": 30, "calibrated": True, "calib_err_px": 0.8}}
        assert "field 3.5 x 2.8 m (camera) mouth 1.22" in ctl.fmt_status(st)[1]
        app.eye.size = None                            # calibration lost: keep the measured size
        app._sync_field()
        assert f.length == 3.5
        app.params["field_from_camera"] = False        # param off: field.json
        app._sync_field()
        assert (f.length, app.field_source) == (app.field_json[0], "field.json")
    finally:
        app.eye = None
        app.close()


def test_hsv_ranges_robust_and_hue_wrap():
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    from fc import ballcal
    rng = np.random.default_rng(0)
    purple = np.stack([rng.normal(140, 3, 500), rng.normal(180, 15, 500), rng.normal(160, 20, 500)], 1)
    outliers = np.array([[30, 20, 90]] * 40)          # a few carpet pixels must not widen the range
    (r,) = ballcal.hsv_ranges(np.vstack([purple, outliers]).clip(0, 255))
    assert 125 <= r["lo"][0] <= 134 and 146 <= r["hi"][0] <= 155 and r["lo"][1] >= 100 and r["hi"][1] == 255
    red = np.stack([rng.normal(0, 3, 500) % 180, np.full(500, 200), np.full(500, 200)], 1)
    rs = ballcal.hsv_ranges(red)
    assert len(rs) == 2 and {r["lo"][0] for r in rs} | {r["hi"][0] for r in rs} >= {0, 179}
    assert all(r["hi"][0] - r["lo"][0] < 20 for r in rs)


def test_find_blob_sample_and_save(tmp_path):
    np = pytest.importorskip("numpy")
    cv2 = pytest.importorskip("cv2")
    from fc import ballcal
    img = np.full((300, 400, 3), (90, 110, 100), np.uint8)       # dull carpet
    cv2.rectangle(img, (20, 20), (120, 40), (200, 200, 200), -1)  # white tape
    hsv_ball = np.uint8([[[140, 190, 170]]])
    bgr = tuple(int(x) for x in cv2.cvtColor(hsv_ball, cv2.COLOR_HSV2BGR)[0, 0])
    cv2.circle(img, (250, 180), 14, bgr, -1)
    u, v, r = ballcal.find_blob(img)
    assert abs(u - 250) < 2 and abs(v - 180) < 2 and 12 < r < 17
    cols = ballcal.hsv_ranges(ballcal.sample_disk(img, u, v, 0.7 * r))
    det_mask = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), np.uint8(cols[0]["lo"]), np.uint8(cols[0]["hi"]))
    assert ballcal.count_candidates(det_mask) == 1
    ballcal.save(tmp_path / "b.json", cols, how="auto")
    assert ballcal.load(tmp_path / "b.json") == cols
    assert ballcal.load(tmp_path / "missing.json") is None
