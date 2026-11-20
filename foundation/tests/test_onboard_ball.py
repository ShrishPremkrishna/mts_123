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
