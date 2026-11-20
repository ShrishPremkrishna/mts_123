"""Front-camera hold/clear event sequences; no ROS or robot dependencies."""
from dataclasses import FrozenInstanceError, replace
import math

import pytest

from final_project.pupper_fc.obstacles import (
    Detection, DetectionFrame, ForwardObstacleGate, ObstacleConfig,
)


def frame(sequence, captured_at, *detections, **changes):
    return DetectionFrame(sequence, captured_at, tuple(detections), **changes)


def near(*, box=(0.4, 0.2, 0.6, 0.8), confidence=0.8, class_id="0"):
    return Detection(box, confidence, class_id)


def clear_gate():
    gate = ForwardObstacleGate()
    for sequence, now in enumerate((0.0, 0.1, 0.2)):
        decision = gate.update(frame(sequence, now), now)
    assert decision.permitted
    return gate


def test_startup_requires_distinct_clear_frames_and_tick_cannot_earn_clearance():
    gate = ForwardObstacleGate()
    assert gate.evaluate(0).state == "stale"
    assert gate.update(frame(0, 0), 0).clear_frames == 1
    for now in (0.01, 0.02, 0.03):
        assert not gate.evaluate(now).permitted
        assert gate.evaluate(now).clear_frames == 1
    assert not gate.update(frame(1, 0.1), 0.1).permitted
    decision = gate.update(frame(2, 0.2), 0.2)
    assert decision.permitted
    assert decision.valid_until == pytest.approx(0.55)


@pytest.mark.parametrize("class_id", ["0", "32", "56", "unknown", ""])
def test_every_class_can_stop_without_a_tracking_target(class_id):
    gate = clear_gate()
    decision = gate.update(frame(3, 0.3, near(class_id=class_id)), 0.3)
    assert decision.state == "blocked" and not decision.permitted
    assert decision.valid_until <= 0.3


def test_wide_box_blocks_even_when_its_center_is_outside_corridor():
    gate = clear_gate()
    wide = near(box=(0.0, 0.1, 0.50, 0.8))  # center=.25; route starts at .35
    assert gate.update(frame(3, 0.3, wide), 0.3).state == "blocked"


@pytest.mark.parametrize("box", [
    (0.0, 0.1, 0.4, 1.0), (0.6, 0.0, 1.0, 0.9), (0.0, 0.0, 1.0, 1.0),
    (0.35, 0.95, 0.36, 1.0),  # small, near detection is not silently exempt
])
def test_border_cropped_and_small_near_boxes_still_block(box):
    gate = clear_gate()
    assert gate.update(frame(3, 0.3, near(box=box)), 0.3).state == "blocked"


def test_off_corridor_far_and_low_confidence_detections_can_be_clear():
    gate = clear_gate()
    detections = (near(box=(0.0, 0.2, 0.3, 0.9)),
                  near(box=(0.4, 0.2, 0.6, 0.5)), near(confidence=0.39))
    assert gate.update(frame(3, 0.3, *detections), 0.3).permitted


def test_bottom_hysteresis_and_consecutive_clear_frames_resist_jitter():
    gate = clear_gate()
    assert gate.update(frame(3, 0.3, near()), 0.3).state == "blocked"
    # Retreat below the trigger is insufficient until below the release boundary.
    assert gate.update(frame(4, 0.4, near(box=(0.4, 0.2, 0.6, 0.68))), 0.4).state == "blocked"
    assert gate.update(frame(5, 0.5), 0.5).clear_frames == 1
    assert gate.update(frame(6, 0.6), 0.6).clear_frames == 2
    assert gate.update(frame(7, 0.7, near(box=(0.4, 0.2, 0.6, 0.65))), 0.7).clear_frames == 0
    for seq, now in ((8, 0.8), (9, 0.9), (10, 1.0)):
        decision = gate.update(frame(seq, now), now)
    assert decision.permitted
    # Once clear, small jitter below the stop boundary does not re-latch.
    assert gate.update(frame(11, 1.1, near(box=(0.4, 0.2, 0.6, 0.68))), 1.1).permitted


@pytest.mark.parametrize("bad_frame", [frame(2, 0.3), frame(1, 0.3), frame(3, 0.2), frame(3, 0.1)])
def test_repeated_or_reordered_identity_cannot_refresh_or_clear(bad_frame):
    gate = clear_gate()
    decision = gate.update(bad_frame, 0.3)
    assert decision.state == "invalid" and decision.clear_frames == 0
    assert not gate.evaluate(0.31).permitted
    assert gate.update(frame(4, 0.32), 0.32).clear_frames == 1


def test_expiry_stops_without_new_frames_and_clearance_does_not_survive_a_gap():
    gate = clear_gate()
    assert gate.evaluate(0.549).permitted
    assert gate.evaluate(0.55).state == "stale"
    assert gate.update(frame(3, 0.56), 0.56).clear_frames == 1
    # update must discover a gap even if the caller missed intermediate ticks.
    assert gate.update(frame(4, 1.0), 1.0).clear_frames == 1


def test_fresh_arrival_cannot_hide_expired_capture_time():
    gate = clear_gate()
    decision = gate.update(frame(3, 0.3), 0.7)
    assert decision.state == "stale" and not decision.permitted
    assert gate.update(frame(4, 0.71), 0.71).clear_frames == 1


def test_expiry_uses_exact_published_deadline_despite_float_subtraction_roundoff():
    gate = ForwardObstacleGate(ObstacleConfig(clear_frames=1))
    decision = gate.update(frame(0, 0.1), 0.1)
    assert decision.permitted
    assert gate.evaluate(decision.valid_until).state == "stale"
    delayed = ForwardObstacleGate(ObstacleConfig(clear_frames=1))
    assert delayed.update(frame(0, 0.1), decision.valid_until).state == "stale"


@pytest.mark.parametrize("changes", [
    {"captured_at": math.nan}, {"captured_at": math.inf}, {"captured_at": -1},
    {"captured_at": 0.31}, {"captured_at": True}, {"format": "raw_fisheye"},
    {"sequence": True}, {"sequence": -1}, {"sequence": 3.0},
    {"valid": 1}, {"healthy": "true"}, {"valid": False}, {"healthy": False},
])
def test_invalid_frame_contract_fails_closed(changes):
    gate = clear_gate()
    bad = replace(frame(3, 0.3), **changes)
    assert gate.update(bad, 0.3).state == "invalid"
    assert not gate.evaluate(0.31).permitted


@pytest.mark.parametrize("detection", [
    near(box=(0.4, 0.2, math.nan, 0.8)), near(box=(0.6, 0.2, 0.4, 0.8)),
    near(box=(0.4, 0.2, 0.4, 0.8)), near(box=(-0.1, 0.2, 0.6, 0.8)),
    near(box=(0.4, 0.2, 0.6, 1.1)), near(box=[0.4, 0.2, 0.6, 0.8]),
    near(confidence=math.inf), near(confidence=1.1), near(confidence=-0.1),
    near(confidence=True), near(class_id=0), object(),
])
def test_invalid_detection_never_counts_as_empty_clear_space(detection):
    gate = clear_gate()
    assert gate.update(frame(3, 0.3, detection), 0.3).state == "invalid"


def test_malformed_frame_and_mutable_detection_container_fail_closed():
    gate = clear_gate()
    assert gate.update({}, 0.3).state == "invalid"
    assert gate.update(DetectionFrame(3, 0.31, []), 0.31).state == "invalid"


def test_external_invalidation_keeps_highwaters_and_requires_new_clearance():
    gate = clear_gate()
    assert gate.invalidate("adapter rejected source clock", 0.3).state == "invalid"
    assert gate.update(frame(2, 0.2), 0.31).state == "invalid"
    assert gate.update(frame(3, 0.32), 0.32).clear_frames == 1
    assert gate.update(frame(4, 0.33), 0.33).clear_frames == 2
    assert gate.update(frame(5, 0.34), 0.34).permitted


def test_unhealthy_frame_identity_cannot_be_resent_as_new_clear_evidence():
    gate = clear_gate()
    assert gate.update(frame(3, 0.3, healthy=False), 0.3).state == "invalid"
    assert gate.update(frame(3, 0.3), 0.31).state == "invalid"
    assert gate.update(frame(4, 0.32), 0.32).clear_frames == 1


def test_producer_restart_needs_new_gate_and_startup_clearance():
    gate = clear_gate()
    assert gate.update(frame(0, 0.3), 0.3).state == "invalid"
    replacement = ForwardObstacleGate()
    assert not replacement.evaluate(0.3).permitted
    for seq, now in enumerate((0.31, 0.32, 0.33)):
        decision = replacement.update(frame(seq, now), now)
        assert decision.permitted == (seq == 2)


@pytest.mark.parametrize("now", [math.nan, math.inf, -1, True, "0.3", 0.19])
def test_invalid_or_rewound_local_clock_revokes_clearance(now):
    gate = clear_gate()
    decision = gate.evaluate(now)
    assert decision.state == "invalid" and math.isfinite(decision.valid_until)
    assert not gate.evaluate(0.21).permitted
    assert gate.update(frame(3, 0.3), 0.3).clear_frames == 1


@pytest.mark.parametrize("changes", [
    {"corridor_left": 0.8}, {"corridor_right": 1.1}, {"min_confidence": -0.1},
    {"stop_bottom": 0.4}, {"release_bottom": math.nan}, {"clear_frames": True},
    {"clear_frames": 0}, {"max_frame_age_s": 0}, {"max_frame_age_s": math.inf},
])
def test_bad_configuration_rejected_before_use(changes):
    with pytest.raises(ValueError):
        ObstacleConfig(**changes)


def test_input_records_and_configuration_are_immutable():
    for obj, attribute, value in ((near(), "confidence", 0.1),
                                  (frame(0, 0), "healthy", False),
                                  (ObstacleConfig(), "clear_frames", 1)):
        with pytest.raises(FrozenInstanceError):
            setattr(obj, attribute, value)
