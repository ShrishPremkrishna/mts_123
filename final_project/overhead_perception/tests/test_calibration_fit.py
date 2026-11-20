"""Calibration without a lens file: the lens angle is fitted from the tags, and the live
script waits for a steady picture before calibrating by itself."""
import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from fieldcam.camera_model import default_intrinsics  # noqa: E402
from fieldcam.config import SIM_CONFIG, load_config  # noqa: E402
from fieldcam.pipeline import Perception  # noqa: E402
from fieldcam.sim import SimArena  # noqa: E402

SIZE = (1920, 1080)
ROBOTS = {"striker": (-0.6, -0.3, 0.4)}


def arena_and_pipeline(cam_pos, true_hfov, configured_hfov):
    """A simulated camera whose real lens angle differs from the one written in the config."""
    cfg = load_config(SIM_CONFIG)
    cfg["camera"]["intrinsics"] = "config/_none_.yaml"
    cfg["camera"]["hfov_deg"] = configured_hfov
    arena = SimArena(cfg, default_intrinsics(*SIZE, true_hfov)[0], cam_pos=cam_pos, size=SIZE)
    return arena, Perception(cfg, *SIZE)


def striker_error(arena, perc):
    state = perc.process(arena.render(ROBOTS, None), 0.0)
    r = state["robots"]["striker"]
    assert r["visible"] and state["calibration_valid"]
    return math.hypot(r["x"] - ROBOTS["striker"][0], r["y"] - ROBOTS["striker"][1])


@pytest.mark.parametrize("mode", ["measured", "corners"])
def test_lens_angle_is_fitted_when_the_camera_looks_down_at_an_angle(mode):
    """The case seen on the phone: config says 69 degrees, the lens really sees 66."""
    arena, perc = arena_and_pipeline((0.0, -2.2, 2.2), true_hfov=66.0, configured_hfov=69.0)
    empty = arena.render({}, None)
    assert perc.calibrate(empty) if mode == "measured" else perc.calibrate_from_corners(empty)
    fit = perc.fov_fit
    assert fit is not None and fit["hfov_deg"] == pytest.approx(66.0, abs=0.5)
    assert fit["fit_px"] < 1.0 < fit["fit_px_configured"]
    assert perc.cam.reproj_err_px < 1.0
    error = striker_error(arena, perc)
    print(f"\n[{mode}] fitted {fit['hfov_deg']:.1f} deg, fit {fit['fit_px_configured']:.1f} -> {fit['fit_px']:.2f} px, "
          f"striker position error {error * 100:.1f} cm")
    assert error < 0.03


def test_a_correct_lens_angle_is_left_alone():
    arena, perc = arena_and_pipeline((0.0, -2.2, 2.2), true_hfov=69.0, configured_hfov=69.0)
    assert perc.calibrate(arena.render({}, None)) and perc.fov_fit is None and perc.hfov_deg == 69.0


def test_straight_down_keeps_the_configured_angle_or_stays_accurate():
    """Looking straight down, the lens angle cannot be told from the camera's height. Whatever the
    fit decides there, positions on the field must still be right."""
    arena, perc = arena_and_pipeline((0.0, 0.0, 3.0), true_hfov=66.0, configured_hfov=69.0)
    assert perc.calibrate(arena.render({}, None))
    assert perc.fov_fit is None                    # nothing to gain: the height absorbs the wrong angle
    error = striker_error(arena, perc)
    print(f"\n[straight down] lens fit: {perc.fov_fit}, striker position error {error * 100:.1f} cm")
    assert error < 0.03


def test_a_lens_file_is_never_overridden():
    arena, perc = arena_and_pipeline((0.0, -2.2, 2.2), true_hfov=66.0, configured_hfov=69.0)
    perc.has_calibrated_intrinsics = True          # as if config/intrinsics.yaml had been loaded
    K = perc.K.copy()
    perc.calibrate(arena.render({}, None))
    assert perc.fov_fit is None and np.array_equal(perc.K, K)


def test_auto_calibration_waits_for_a_steady_picture():
    spec = importlib.util.spec_from_file_location("run_perception", ROOT / "scripts" / "run_perception.py")
    run_perception = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(run_perception)
    cfg = {"robots": {"striker": {"tag_id": 10}}}

    def frame(shift, ids=(0, 1, 2, 3, 10)):
        square = np.array([[0.0, 0.0], [40.0, 0.0], [40.0, 40.0], [0.0, 40.0]])
        return SimpleNamespace(cfg=cfg, debug={"tags": {i: square + 200.0 * i + shift for i in ids}})

    settled = run_perception.Settled(hold_s=1.0, tol_px=2.0)
    # The first second: still framing, every tag sliding 6 px per frame.
    assert not any(settled.update(frame(6.0 * k), 0.1 * k) for k in range(10))
    # Then it stops moving: steady only after a full second of stillness.
    results = [settled.update(frame(60.0), 1.0 + 0.1 * k) for k in range(25)]
    assert not any(results[:10]) and all(results[18:])
    # A robot tag walking about does not count, a floor tag disappearing restarts the wait.
    moving_robot = frame(60.0)
    moving_robot.debug["tags"][10] = moving_robot.debug["tags"][10] + 300.0
    assert settled.update(moving_robot, 3.6)
    assert not settled.update(frame(60.0, ids=(0, 1, 2)), 3.7)
    assert not settled.update(SimpleNamespace(cfg=cfg, debug={"tags": {}}), 3.8)
