"""Anonymous, image-only observations independent of the tag-driven field state."""
import json
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from fieldcam.appearance import AppearanceTracker, BodyCandidate, detect_puppers


def body(frame, x, y):
    cv2.ellipse(frame, (x, y), (18, 10), 0, 0, 360, (38, 36, 42), -1)
    for dx in (-11, 11):
        for dy in (-12, 12):
            cv2.rectangle(frame, (x + dx - 4, y + dy - 1), (x + dx + 4, y + dy + 1),
                          (228, 230, 225), -1)


def test_full_image_two_bodies_without_ids_or_heading():
    frame = np.full((200, 300, 3), 110, np.uint8)
    centers = [(85, 90), (209, 110)]
    for x, y in centers:
        body(frame, x, y)
    for image in (frame, np.ascontiguousarray(np.fliplr(frame))):
        obs = detect_puppers(image, 40)
        expected = centers if image is frame else [(299 - x, y) for x, y in centers]
        assert len(obs) == 2
        np.testing.assert_allclose(sorted(c.center_px for c in obs), sorted(expected), atol=2)
        assert all(c.score > .44 and 0 <= c.axis_rad < np.pi for c in obs)
        assert all(not hasattr(c, "robot_id") and not hasattr(c, "yaw") for c in obs)


def test_empty_blurred_and_small_unresolved_views_abstain():
    frame = np.full((200, 300, 3), 110, np.uint8)
    cv2.rectangle(frame, (50, 30), (170, 110), (220, 220, 220), -1)
    assert not detect_puppers(frame, 40)
    body(frame, 230, 150)
    assert not detect_puppers(cv2.GaussianBlur(frame, (0, 0), 9), 40)
    assert not detect_puppers(frame, 12)
    with pytest.raises(ValueError):
        detect_puppers(frame, np.nan)


def test_coalescing_and_missing_observations_never_continue_a_named_track():
    t = AppearanceTracker()
    def point(x):
        return BodyCandidate((x, 100.), .9, None)
    assert [r.consecutive_frames for r in t.update([point(40), point(78)], 1., 40)] == [1, 1]
    assert all(r.consecutive_frames == 2 for r in t.update([point(42), point(77)], 1.05, 40))
    merged, = t.update([point(58)], 1.10, 40)
    assert merged.ambiguous and merged.consecutive_frames == 1
    assert all(r.ambiguous for r in t.update([point(41), point(77)], 1.15, 40))
    assert not t.update([], 1.20, 40)
    fresh, = t.update([point(41)], 1.25, 40)
    assert fresh.consecutive_frames == 1
    assert t.update([point(41)], 1.65, 40)[0].consecutive_frames == 1


def test_printed_dark_pattern_enclosed_in_one_white_card_is_not_four_legs():
    frame = np.full((200, 300, 3), 100, np.uint8)
    cv2.rectangle(frame, (85, 70), (145, 126), (230, 230, 230), -1)
    cv2.rectangle(frame, (92, 77), (138, 119), (24, 24, 24), -1)
    for x in (104, 126):
        for y in (84, 110):
            cv2.rectangle(frame, (x-4, y-2), (x+4, y+2), (224, 224, 224), -1)
    assert not detect_puppers(frame, 50)


def test_preview_is_visible_in_camera_cli_and_absent_from_gameplay(monkeypatch, capsys):
    from scripts import run_perception

    packets = []
    sink = SimpleNamespace(source_id="sim", source_session="preview", send=packets.append, close=lambda: None)
    monkeypatch.setattr(run_perception, "StatePublisher", lambda *_args, **_kwargs: sink)
    monkeypatch.setattr(sys, "argv", ["run_perception", "--source", "sim", "--no-gui", "--print",
                                     "--appearance-preview", "--frames", "2"])
    run_perception.main()
    printed = [json.loads(line) for line in capsys.readouterr().out.splitlines() if line.startswith("{")]
    diagnostic = [s for s in printed if "appearance_preview" in s]
    assert len(diagnostic) == len(packets) == 2
    for state, packet in zip(diagnostic, packets):
        assert isinstance(state.pop("appearance_preview"), list)
        assert state == json.loads(json.dumps(packet))
