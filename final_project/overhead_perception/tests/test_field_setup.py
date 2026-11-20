"""Exercise field setup with rendered pixels, per-frame loss and explicit confirmation."""
import json
import math
import time

import cv2
import numpy as np
import pytest

from fieldcam.camera_model import default_intrinsics
from fieldcam.config import SIM_CONFIG, load_config, set_field_size
from fieldcam.corner_anchor import FarCorner, anchor_geometry
from fieldcam.pipeline import Perception
from fieldcam.sim import SimArena
from final_project.pupper_fc.arena_camera import ArenaCamera, camera_settings
from final_project.pupper_fc.lobby import PROFILE


def scene(tag_size, *, colors=False):
    size = (1280, 800)
    cfg = load_config(SIM_CONFIG)
    setup = dict(mode="corner", corner="west_north", tag_id=0, length=2.6, width=1.6,
                 inset_x=.15, inset_y=.15, rotation=30., tag_size=tag_size, goal_width=.6)
    cfg.update(field_setup=setup, robot_tracking="colors" if colors else "tags")
    set_field_size(cfg, setup["length"], setup["width"])
    cfg["tags"]["field"] = {0: anchor_geometry(setup).mean(axis=0)}
    K, _ = default_intrinsics(*size, cfg["camera"]["hfov_deg"])
    sim = SimArena(cfg, K, size=size, noise_sigma=.3)
    return Perception(cfg, *size), sim


def test_one_tag_far_corner_refines_and_watches_the_independent_mark():
    cam, sim = scene(.13)
    im = sim.render({}, None)
    assert not cam.calibrate(im)
    assert cam.calib_fail
    xy = (1.3, -.8)
    uv = sim.cam.project([[*xy, 0]])[0]
    # The independent floor mark is a tape cross with identifiable texture.
    mark = tuple(np.round(uv).astype(int))
    cv2.drawMarker(im, mark, (30, 45, 240), cv2.MARKER_TILTED_CROSS, 19, 3)
    cam.far_corner = FarCorner(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), uv)
    assert cam.calibrate(im), cam.calib_fail
    state = cam.process(im, 1.)
    assert state["calibration_valid"] and cam.setup_metrics["far_sensitivity_m"] <= .075
    error = np.linalg.norm(cam.cam.pixel_to_plane(sim.cam.project([[-1.3, -.8, 0], [1.3,.8,0]]), 0)
                           - np.array([[-1.3,-.8],[1.3,.8]]), axis=1)
    assert max(error) < .05
    x,y = mark
    im[y-24:y+25, x-24:x+25] = 0
    next_frame = cam.process(im, 2.)
    assert not next_frame["calibration_valid"] and "corner mark" in next_frame["calibration_reason"]


def test_rectified_color_heading_is_signed_and_lost_nose_never_coasts_as_measured():
    cam, sim = scene(.40, colors=True)
    im = sim.render({}, None)
    assert cam.calibrate(im), cam.calib_fail
    truth = {"striker": (-.6, .25, -1.7), "keeper": (.8, -.35, 1.45)}
    observed = sim.render(truth)
    good = cam.process(observed, 1., match_only=True)
    for name, (x, y, yaw) in truth.items():
        robot = good["robots"][name]
        assert robot["measured_this_frame"] and math.hypot(robot["x"]-x, robot["y"]-y) < .04
        assert abs(math.remainder(robot["yaw"] - yaw, math.tau)) < math.radians(5)
    # Reacquisition at the opposite heading cannot inherit an old signed yaw.
    flipped = {name: (x, y, yaw + math.pi) for name, (x,y,yaw) in truth.items()}
    fresh = cam.process(sim.render(flipped), 1.03, 1.15, match_only=True)
    for name, (_, _, yaw) in flipped.items():
        assert fresh["robots"][name]["measured_this_frame"]
        assert abs(math.remainder(fresh["robots"][name]["yaw"] - yaw, math.tau)) < math.radians(5)
    hsv = cv2.cvtColor(observed, cv2.COLOR_BGR2HSV)
    missing = cv2.inRange(hsv,(26,150,150),(50,255,255))
    observed[missing != 0] = (30,30,30)
    after = cam.process(observed, 1.05, match_only=True)
    for robot in after["robots"].values():
        assert robot["visible"] and not robot["measured_this_frame"]
        assert "nose patch" in robot["tracking_detail"]
    assert after["tracking"] == "colors"


def test_snapshot_is_bound_to_camera_revision_and_required_for_corner_mode():
    p = json.loads(PROFILE.read_text())
    anchor = dict(corner="west_north", tag_id=0, length=2.6, width=1.6, inset_x=.15, inset_y=.15,
                  rotation=30, tag_size=.40, goal_width=.6)
    p["field_setup"], p["robot_tracking"] = camera_settings(dict(field_mode="corner", anchor=anchor), p)
    worker = ArenaCamera(p, "sim")
    worker.start()
    try:
        deadline = time.monotonic() + 5
        while not worker.jpeg() and time.monotonic() < deadline:
            time.sleep(.01)
        snap = worker.capture_setup()
        assert snap["image"].startswith("data:image/jpeg;base64,") and snap["fit_valid"]
        assert worker.snapshot()["needs_confirm"] and not worker.snapshot()["frame"]["calibration_valid"]
        with pytest.raises(ValueError):
            worker.confirm_field(dict(token="other"))
        worker.confirm_field(dict(token=snap["token"]))
        while worker.snapshot()["needs_confirm"] and time.monotonic() < deadline:
            time.sleep(.01)
        assert worker.snapshot()["frame"]["calibration_valid"] and not worker.snapshot()["live"]
        with pytest.raises(ValueError):
            worker.confirm_field(dict(token=snap["token"]))
    finally:
        worker.close()
