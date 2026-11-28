"""Per-robot tag mount (fc/eye.py, Nov 28): eye_tag_height / eye_tag_offset_x / eye_yaw_offset_deg come from
fc/config/robots/<id>.json when that robot has them, else from params (the default). Before, one global mount
meant robot 2 (red) inherited blue's measured +7 deg heading bias."""
import importlib.util
import json
import math

import pytest

from fc.core import CONFIG, ROBOTS_DIR, RobotCfg, load_field, load_params

HAVE_CV2 = importlib.util.find_spec("cv2") is not None
pytestmark = pytest.mark.skipif(not HAVE_CV2, reason="needs OpenCV (.venv-fc)")
if HAVE_CV2:
    import numpy as np

    from fc.eye import Eye

BLUE = RobotCfg("blue", "blue", 10, "", "stock")
RED = RobotCfg("red", "red", 11, "", "stock")
PARAMS = {"eye_tag_height": 0.267, "eye_tag_offset_x": -0.02, "eye_yaw_offset_deg": 0.0}
TAG = None if not HAVE_CV2 else np.array([[0.1, 0.6], [0.1, 0.4], [-0.1, 0.4], [-0.1, 0.6]])   # top edge -> +x


def write(d, rid, data):
    (d / f"{rid}.json").write_text(json.dumps(data))


def eye(tmp_path, params=None, **kw):
    return Eye(load_field(), [BLUE, RED], dict(params or PARAMS), source="recorded.mov", **kw)   # nothing opened


def test_each_robot_gets_its_own_mount(tmp_path):
    write(tmp_path, "blue", {"_note": "x", "eye_yaw_offset_deg": 7.0, "eye_tag_height": 0.25, "act_lag_s": 0.2})
    e = eye(tmp_path, robots_dir=tmp_path)                       # red has no file: params
    _, _, yb = e.tag_pose(TAG, e.mount("blue"))
    _, _, yr = e.tag_pose(TAG, e.mount("red"))
    assert math.degrees(yb) == pytest.approx(7.0) and yr == pytest.approx(0.0)
    assert e.mount("blue")["tag_height"] == 0.25 and e.mount("red")["tag_height"] == 0.267
    assert e.mount("blue")["tag_offset_x"] == -0.02                # not in blue's file: the default
    assert e.cfg["robots"]["blue"]["yaw_offset"] == pytest.approx(math.radians(7.0))
    assert e.cfg["robots"]["red"]["tag_height"] == 0.267
    # act_lag_s is the brain's knob: not part of the eye's mount
    assert e.robot_over == {"blue": {"eye_yaw_offset_deg": 7.0, "eye_tag_height": 0.25}, "red": {}}
    m = e.mounts()
    assert m["blue"]["own"] == ["eye_tag_height", "eye_yaw_offset_deg"] and m["red"]["own"] == []


def test_params_edits_still_apply_live_to_a_robot_without_its_own_value(tmp_path):
    write(tmp_path, "blue", {"eye_yaw_offset_deg": 7.0})
    e = eye(tmp_path, robots_dir=tmp_path)
    e.params["eye_yaw_offset_deg"] = -3.0
    e.params["eye_tag_height"] = 0.3
    assert math.degrees(e.mount("red")["yaw_offset"]) == pytest.approx(-3.0)
    assert math.degrees(e.mount("blue")["yaw_offset"]) == pytest.approx(7.0)       # the robot's own value wins
    assert e.mount("blue")["tag_height"] == 0.3
    e.robot_over["blue"]["eye_yaw_offset_deg"] = 6.5                                # live hook (e.g. a UI knob)
    assert math.degrees(e.mount("blue")["yaw_offset"]) == pytest.approx(6.5)


def test_an_edited_robot_file_is_picked_up_without_a_restart(tmp_path, capsys):
    e = eye(tmp_path, robots_dir=tmp_path)
    assert e.robot_over["red"] == {}
    write(tmp_path, "red", {"eye_yaw_offset_deg": -4.0, "_measured": "step test"})
    e._reload_robot_over()
    assert math.degrees(e.mount("red")["yaw_offset"]) == pytest.approx(-4.0)
    assert "red tag mount from red.json" in capsys.readouterr().out
    write(tmp_path, "red", {"eye_yaw_offset_deg": "-5"})                            # a bad edit: keep the last good
    (tmp_path / "red.json").touch()
    e._over_mtimes["red"] = -1.0
    e._reload_robot_over()
    assert math.degrees(e.mount("red")["yaw_offset"]) == pytest.approx(-4.0)
    assert "must be a number" in capsys.readouterr().out


def test_a_bad_robot_file_fails_at_startup(tmp_path):
    write(tmp_path, "red", {"eye_tag_height": True})
    with pytest.raises(ValueError, match="must be a number"):
        eye(tmp_path, robots_dir=tmp_path)


def test_caller_given_overrides_and_synthetic_geometry(tmp_path):
    e = eye(tmp_path, robot_over={"red": {"eye_yaw_offset_deg": 2.0, "loop_hz": 30}})
    assert e.robots_dir is None and e.robot_over == {"blue": {}, "red": {"eye_yaw_offset_deg": 2.0}}
    s = Eye(load_field(), [BLUE], dict(PARAMS), source="synthetic", robots_dir=tmp_path)
    assert s.mount("blue") == s.mount()   # the renderer's own fixed mount


def test_shipped_configs_blue_keeps_its_measured_mount_red_runs_on_defaults():
    """The values measured on blue (mts) live in robots/blue.json; params.json is the default for an unmeasured
    robot (blue's tag geometry, no heading bias), so red no longer inherits blue's +7 deg."""
    blue = json.loads((ROBOTS_DIR / "blue.json").read_text())
    assert (blue["eye_tag_height"], blue["eye_tag_offset_x"], blue["eye_yaw_offset_deg"]) == (0.267, -0.02, 7.0)
    params = load_params(CONFIG / "params.json")
    assert params["eye_yaw_offset_deg"] == 0.0
    e = Eye(load_field(), [BLUE, RED], params, source="recorded.mov")
    assert math.degrees(e.mount("blue")["yaw_offset"]) == pytest.approx(7.0)
    assert math.degrees(e.mount("red")["yaw_offset"]) == pytest.approx(0.0)
    assert e.mount("red")["tag_height"] == e.mount("blue")["tag_height"] == 0.267
