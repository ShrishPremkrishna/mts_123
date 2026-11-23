"""Onboard detector and SSH-facing telemetry tests without a camera or accelerator."""
import importlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from follow_me_pupper.robot import onboard_ball as contract

STAMP = 1_000_000_000
ROOT = Path(__file__).resolve().parents[2]


def packet(**fields):
    return json.dumps(dict(dict(schema=contract.SCHEMA, captured_at_ros_ns=STAMP, captured_at_mono_ns=STAMP, sequence=19, mode="pink", projection_width=700,
                                state="tracked", source="pink", box=[.65, .6, .85, .8]), **fields))


def test_same_capture_ages_through_process_silence_and_ros_clock_pause():
    receiver = contract.OnboardBallReceiver()
    assert receiver.snapshot(STAMP, 0)["state"] == "stale"
    assert receiver.accept(packet(captured_at_mono_ns=4_730_000_000), STAMP + 270_000_000, 5.)
    seen = receiver.snapshot(STAMP + 270_000_000, 5.06)
    assert seen["state"] == "tracked" and seen["bearing_deg"] == 55.1
    assert seen["captured_at_ros_ns"] == str(STAMP) and seen["age_ms"] == 330
    assert not receiver.accept(packet(), STAMP + 270_000_000, 5.079)
    stale = receiver.snapshot(STAMP + 270_000_000, 5.082)
    assert stale["state"] == "stale" and "bearing_deg" not in stale and "box" not in stale
    assert receiver.snapshot(STAMP - 1, 5.06)["state"] == "stale"


@pytest.mark.parametrize("fields", [
    {"captured_at_mono_ns": STAMP - 400_000_000}, {"captured_at_mono_ns": STAMP + 1},
    {"captured_at_ros_ns": STAMP + 1}, {"captured_at_ros_ns": STAMP - contract.MAX_AGE_NS - 1},
    {"sequence": True}, {"state": "off"}, {"source": "yolo"}, {"mode": "confirmed"},
    {"box": [0, 0, 0, 1]}, {"box": [0, 0, float("nan"), 1]}, {"box": [-1, 0, 1, 1]},
])
def test_receiver_refuses_impossible_or_expired_observations(fields):
    receiver = contract.OnboardBallReceiver()
    assert not receiver.accept(packet(**fields), STAMP, 1.)
    assert receiver.snapshot(STAMP, 1.)["state"] == "stale"


@pytest.fixture
def image_test(monkeypatch):
    cv2 = pytest.importorskip("cv2")
    import numpy as np
    monkeypatch.syspath_prepend(str(ROOT / "follow_me_pupper/robot"))
    tracker = importlib.import_module("pink_ball").PinkBallTracker()
    def make_image(*circles):
        out = np.full((572, 700, 3), 50, np.uint8)
        for x, rgb in circles:
            cv2.circle(out, (x, 365), 16, rgb, -1)
        return out
    return tracker, make_image


PINK = (241, 175, 186)
MAGENTA = (230, 26, 150)
BALL = SimpleNamespace(class_name="sports ball", box=(68, 348, 101, 383))


def test_unconfirmed_track_requires_two_captures_then_loses_target_immediately(image_test):
    tracker, make = image_test
    assert tracker.observe(make((85, PINK)), [], STAMP, "pink")["state"] == "searching"
    result = tracker.observe(make((88, PINK)), [], STAMP + 100_000_000, "pink")
    assert result["state"] == "tracked" and result["source"] == "pink"
    assert tracker.observe(make(), [BALL], STAMP + 200_000_000, "pink") == {"state": "searching"}
    assert tracker.observe(make((88, PINK)), [], STAMP + 500_000_000, "pink") == {"state": "searching"}


def test_confirmed_requires_same_location_color_and_semantics(image_test):
    tracker, make = image_test
    assert tracker.observe(make((85, MAGENTA)), [BALL], STAMP, "confirmed")["source"] == "pink+yolo"
    for d, image in (([], make((85, PINK))), ([BALL], make((600, PINK))),
                     ([BALL], make((85, (235, 20, 22)))), ([BALL], make())):
        assert tracker.observe(image, d, STAMP + 100_000_000, "confirmed") == {"state": "searching"}
    assert tracker.observe(make((85, PINK)), [BALL], STAMP, "off") == {"state": "off"}


def test_competing_pink_candidates_abstain_until_distinct_evidence(image_test):
    tracker, make = image_test
    image = make((85, PINK), (490, PINK))
    assert tracker.observe(image, [], STAMP, "pink") == {"state": "ambiguous"}
    row = tracker.observe(image, [BALL], STAMP + 100_000_000, "pink")
    assert row["source"] == "pink+yolo" and row["box"][0] < .15


def test_edges_of_color_swapped_projection_are_identical(monkeypatch):
    cv2 = pytest.importorskip("cv2")
    import numpy as np
    monkeypatch.syspath_prepend(str(ROOT / "follow_me_pupper/robot"))
    camera = importlib.import_module("viser_camera")
    bgr = np.random.default_rng(123).integers(256, size=(1050, 1400, 3), dtype=np.uint8)
    model = camera.load_camera_model(ROOT / "follow_me_pupper/robot/camera_params.yaml", 1400, 1050)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    for swap in (False, True):
        previous = camera.detection_projection(camera.detector_source_frame(rgb, swap_rb=swap), model, 700)
        new = camera.detection_projection(bgr, model, 700)
        if not swap:
            new = cv2.cvtColor(new, cv2.COLOR_BGR2RGB)
        np.testing.assert_array_equal(previous, new)


IDENTITY = dict(profile_id="a" * 64, calibration_id="b" * 64, status="estimated", ball_radius_m=.11,
                radius_measured=False, mount_measured=False)
FIT = dict(method="eye_fit", x=.4, y=.05, z=-.04, range_m=.31, rho_rad=.36, circle=[300., 400., 60.], score=.8,
           sigma_px=.6, sigma_xy_m=.002, sigma_range_m=.002, fit_rms_px=.6, fit_points=40)


def metric_row(**fields):
    values = dict(producer_instance="cam", sequence=4, frame_sequence=19, captured_at_ros_ns=STAMP,
                  captured_at_mono_ns=STAMP, identity=IDENTITY, model_sha256="c" * 64, state="measured",
                  selected=0, candidates=[FIT], processing_ms=41.5)
    values.update(fields)
    return contract.metric_payload(**values)


def test_metric_row_is_admitted_by_its_own_capture_and_never_renewed_by_arrival():
    receiver = contract.MetricBallReceiver()
    assert receiver.snapshot(STAMP, 1.)["state"] == "stale"
    assert receiver.accept(metric_row(), STAMP + 100_000_000, 1.1), receiver.reason
    seen = receiver.snapshot(STAMP + 100_000_000, 1.2)
    assert seen["state"] == "measured" and seen["method"] == "eye_fit" and seen["age_ms"] == 200
    assert seen["identity"] == IDENTITY and seen["uncertainty_scope"] == "image_only"
    assert receiver.latest(STAMP + 100_000_000, 1.36) is None  # 360 ms after capture on the Pi clock
    assert receiver.snapshot(STAMP + 100_000_000, 1.36)["state"] == "stale"
    assert receiver.latest(STAMP + 360_000_000, 1.2) is None


@pytest.mark.parametrize("fields", [
    {"captured_at_ros_ns": STAMP + 200_000_001}, {"captured_at_ros_ns": STAMP - 300_000_000},
    {"captured_at_mono_ns": STAMP + 200_000_000}, {"captured_at_mono_ns": STAMP - 300_000_000},
    {"captured_at_ros_ns": None}, {"captured_at_mono_ns": 1.5}, {"sequence": -1}, {"sequence": True},
    {"identity": {**IDENTITY, "ball_radius_m": 0}}, {"identity": {**IDENTITY, "profile_id": "A" * 64}},
    {"identity": {**IDENTITY, "status": "learned"}}, {"model_sha256": "xyz"},
    {"state": "measured", "selected": None}, {"state": "searching", "selected": None},
    {"state": "conflict", "selected": None}, {"selected": 1},
    {"candidates": [{**FIT, "method": "track"}]}, {"candidates": [{**FIT, "sigma_xy_m": 0}]},
    {"candidates": [{**FIT, "method": "eye_box"}]}, {"candidates": [{**FIT, "fit_points": None}]},
    {"candidates": [FIT, FIT], "state": "conflict", "selected": None}, {"candidates": [{**FIT, "score": 1.5}]},
    {"candidates": [{**FIT, "circle": [1, 2]}]}, {"processing_ms": -1},
])
def test_metric_receiver_refuses_invalid_rows_without_state_change(fields):
    receiver = contract.MetricBallReceiver()
    assert not receiver.accept(metric_row(**fields), STAMP + 100_000_000, 1.1)
    assert receiver.row is None and receiver.rejected == 1 and receiver.reason


def test_metric_receiver_refuses_nonfinite_numbers_scope_and_oversize():
    receiver = contract.MetricBallReceiver()
    assert not receiver.accept(metric_row().replace('"x":0.4', '"x":1e999'), STAMP, 1.)
    assert not receiver.accept(metric_row().replace('"image_only"', '"total"'), STAMP, 1.)
    assert not receiver.accept(metric_row(producer_instance="p" * 9000), STAMP, 1.)
    assert receiver.row is None


def test_metric_duplicate_reordered_and_restarted_producer_rows():
    receiver = contract.MetricBallReceiver()
    assert receiver.accept(metric_row(), STAMP + 50_000_000, 1.05)
    assert not receiver.accept(metric_row(sequence=5), STAMP + 50_000_000, 1.05)  # same capture
    assert not receiver.accept(metric_row(sequence=3, captured_at_ros_ns=STAMP + 1, captured_at_mono_ns=STAMP + 1),
                               STAMP + 50_000_000, 1.05)
    assert receiver.reason == "duplicate or reordered sequence"
    # A restarted producer may restart its sequence, but never its capture order.
    assert receiver.accept(metric_row(producer_instance="new", sequence=0, captured_at_ros_ns=STAMP + 2,
                                      captured_at_mono_ns=STAMP + 2), STAMP + 50_000_000, 1.05)
    box = {**FIT, "method": "pink_box", "fit_rms_px": None, "fit_points": None}
    assert receiver.accept(metric_row(sequence=1, captured_at_ros_ns=STAMP + 3, captured_at_mono_ns=STAMP + 3,
                                      state="searching", selected=None, candidates=[]), STAMP + 50_000_000, 1.05)
    assert receiver.snapshot(STAMP + 50_000_000, 1.05)["state"] == "searching"
    assert "method" not in receiver.snapshot(STAMP + 50_000_000, 1.05)
    assert receiver.accept(metric_row(sequence=2, captured_at_ros_ns=STAMP + 4, captured_at_mono_ns=STAMP + 4,
                                      state="conflict", selected=None, candidates=[FIT, box]),
                           STAMP + 50_000_000, 1.05)
