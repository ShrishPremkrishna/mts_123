"""Control admission regressions using rendered images, not claimed track ages."""
import copy
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.camera_model import default_intrinsics
from fieldcam.config import SIM_CONFIG, load_config
from fieldcam.filters import CVFilter2D
from fieldcam.gameplay import to_observation
from fieldcam.pipeline import Perception
from fieldcam.sim import SimArena

SIZE = (1920, 1080)
TRUTH = {"striker": (-0.6, -0.2, 0.4)}
BALL = (0.3, 0.2)


def scene():
    cfg = load_config(SIM_CONFIG)
    cfg["camera"]["intrinsics"] = "config/_none_.yaml"
    K, _ = default_intrinsics(*SIZE, cfg["camera"]["hfov_deg"])
    arena = SimArena(cfg, K, size=SIZE)
    perc = Perception(cfg, *SIZE)
    assert perc.calibrate(arena.render({}, None))
    return cfg, arena, perc


def test_camera_translation_latches_hold_until_explicit_recalibration():
    """A +20 cm camera bump cannot become a fresh admissible pose after filter reset."""
    cfg, arena, perc = scene()
    original = arena.render(TRUTH, BALL)
    for k in range(12):
        state = perc.process(original, 100.0 + k / 30, 100.12 + k / 30)
    assert to_observation(state).robot is not None
    revision = state["calibration_revision"]
    shifted = SimArena(cfg, perc.K, cam_pos=(0.2, -0.2, 2.6), cam_target=(0.2, 0.0, 0.0), size=SIZE)
    moved = shifted.render(TRUTH, BALL)
    for k in range(6):
        captured = 100.0 + (12 + k) / 30
        state = perc.process(moved, captured, captured + 0.12)
        assert state["calib_err_px"] > 80
        assert not state["calibrated"] and not state["calibration_valid"]
        assert state["calibration_status"] == "invalidated"
        assert state["calibration_reason"] == "floor_reference_drift"
        assert state["shots"] == {}
        assert state["relative"] == {}
        assert state["robots"]["striker"]["measured_this_frame"] is (k >= 2)
        obs = to_observation(json.loads(json.dumps(state)))
        assert obs.robot is None and obs.ball is None
    # The display tracker can reacquire a wrong pose, but it never becomes motion evidence.
    assert state["robots"]["striker"]["measured_this_frame"]
    assert math.hypot(state["robots"]["striker"]["x"] - TRUTH["striker"][0],
                      state["robots"]["striker"]["y"] - TRUTH["striker"][1]) > 0.15
    # Even moving the camera back does not silently clear a detected calibration change.
    state = perc.process(original, 102.0, 102.12)
    assert state["calib_err_px"] < 2
    assert not state["calibration_valid"] and state["calibration_revision"] == revision
    assert to_observation(state).robot is None

    assert perc.calibrate(shifted.render({}, None))
    assert not perc.ball_f.initialised and not perc.robot_f["striker"][0].initialised
    state = perc.process(moved, 103.0, 103.12)
    assert state["calibration_valid"] and state["calibration_revision"] == revision + 1
    obs = to_observation(state)
    assert math.hypot(obs.robot.x - TRUTH["striker"][0], obs.robot.y - TRUTH["striker"][1]) < 0.02


def test_floor_occlusion_preserves_freeze_but_cannot_clear_reconnect_hold():
    cfg, arena, perc = scene()
    seen = perc.process(arena.render(TRUTH, BALL), 100.0, 100.12)
    saved_camera = perc.cam
    hidden_cfg = copy.deepcopy(cfg)
    hidden_cfg["tags"]["field"] = {}
    hidden = SimArena(hidden_cfg, perc.K, size=SIZE).render(TRUTH, BALL)
    state = perc.process(hidden, 100.2, 100.32)
    assert state["calibration_valid"] and state["calibration_status"] == "occluded"
    assert state["calibration_checked_at"] == seen["captured_at"]
    assert to_observation(state).robot is not None and perc.cam is saved_camera

    perc.invalidate_calibration("capture_reconnected")
    state = perc.process(hidden, 100.4, 100.52)
    assert not state["calibration_valid"] and state["calibration_reason"] == "capture_reconnected"
    assert to_observation(state).robot is None
    state = perc.process(arena.render(TRUTH, BALL), 100.6, 100.72)
    assert state["calib_err_px"] < 2 and not state["calibration_valid"]
    assert perc.cam is saved_camera
    # A failed attempt is not recovery; a successful explicit calibration is.
    assert not perc.calibrate(hidden)
    assert not perc.calibration_valid
    assert perc.calibrate(arena.render({}, None))
    assert perc.calibration_valid


def test_frame_resolution_change_invalidates_frozen_geometry():
    _, arena, perc = scene()
    frame = arena.render(TRUTH, BALL)
    state = perc.process(frame[::2, ::2], 100.0, 100.12)
    assert not state["calibration_valid"] and state["calibration_reason"] == "frame_size_changed"
    assert to_observation(state).robot is None
    state = perc.process(frame, 100.2, 100.32)
    assert not state["calibration_valid"] and to_observation(state).robot is None


@pytest.mark.parametrize("fps", [30, 60, 120])
def test_missed_frame_never_retimes_coasted_robot_or_ball(fps):
    _, arena, perc = scene()
    captured = 1790000000.125
    state = perc.process(arena.render(TRUTH, BALL), captured, captured + 0.12)
    obs = to_observation(json.loads(json.dumps(state)))
    assert obs.robot is not None and obs.ball is not None
    assert state["captured_at"] == state["robots"]["striker"]["measured_at"] == captured
    assert state["ball"]["measured_at"] == captured
    missed_at = captured + 1 / fps
    missed = perc.process(arena.render({}, None), missed_at, missed_at + 0.12)
    for obj in (missed["robots"]["striker"], missed["ball"]):
        assert obj["visible"]  # previews retain their coasting behavior
        assert obj["measured_this_frame"] is False and obj["measured_at"] == captured
        obj["age_s"] = round(obj["age_s"], 2)  # display rounding must not decide admission
    missed["frame_age_s"] = round(missed["frame_age_s"], 2)
    wire = json.loads(json.dumps(missed))
    obs = to_observation(wire)
    assert obs.robot is None and obs.ball is None and obs.captured_at == missed_at
    # The marker alone is insufficient if it disagrees with the original measurement time.
    wire["robots"]["striker"]["measured_this_frame"] = True
    assert to_observation(wire).robot is None


def test_filter_rejection_reports_original_measurement_time():
    tracker = CVFilter2D(2.0, 0.01)
    assert tracker.update([0.0, 0.0], 100.0) is True
    for k in range(1, 12):
        assert tracker.update([0.0, 0.0], 100.0 + k / 60) is True
    previous = tracker.t_meas
    assert tracker.update([1.0, 1.0], 100.2) is False
    assert tracker.t_meas == previous


def test_legacy_freshness_and_missing_health_are_not_motion_evidence():
    state = {"stamp": 100.12, "frame_age_s": 0.12, "calibrated": True, "calib_err_px": 30.0,
             "robots": {"striker": {"visible": True, "x": 0.0, "y": 0.0, "yaw": 0.0,
                                      "vx": 0.0, "vy": 0.0, "wz": 0.0, "age_s": 0.136666666666}},
             "ball": {"visible": False}}
    assert to_observation(state).robot is None
    state.update(captured_at=100.0, calibration_valid=True)
    state["robots"]["striker"].update(measured_this_frame=True, measured_at=100.0 - 1 / 60)
    assert to_observation(state).robot is None
