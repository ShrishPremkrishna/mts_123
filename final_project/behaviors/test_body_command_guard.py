"""Fault sequences for the actual-command gate; these do not prove robot stopping."""
import copy
import json
import sys
import types
import unittest
from unittest.mock import patch

from final_project.behaviors.body_command_guard import (
    ARENA_DETECTOR_IDENTITY, BodyCommand, BodyCommandGuard, CLOCK_JUMP_NS, FRAME_TOPIC, INPUT_TOPIC,
    INPUT_TIMEOUT_NS, MAX_DETECTIONS, OUTPUT_TOPIC, SCHEMA, STATUS_TOPIC,
    TIMER_S, ZERO, create_ros_node, main,
)


class Timeline:
    def __init__(self):
        self.mono = 1_000_000_000_000
        self.ros = 1_795_000_000_000_000_000
        self.sequence = 0

    def advance(self, seconds=0.05):
        ns = round(seconds * 1e9)
        self.mono += ns
        self.ros += ns

    def clocks(self):
        return dict(ros_ns=self.ros, mono_ns=self.mono)

    def frame(self, *, age_ns=0, producer="camera-a", detections=None):
        self.sequence += 1
        return dict(schema=SCHEMA, producer_instance=producer,
                    detector_identity=dict(ARENA_DETECTOR_IDENTITY),
                    sequence=self.sequence, captured_at_ros_ns=self.ros - age_ns,
                    valid=True, healthy=True, format="normalized_xyxy_v1",
                    image_width=700, image_height=572, horizontal_fov_deg=220,
                    detections=detections or [])


def obstacle(label="tower", confidence=0.9):
    return dict(box=[0.4, 0.3, 0.6, 0.8], confidence=confidence, class_id=label)


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = BodyCommandGuard()
        self.time = Timeline()
        self.guard.evaluate(**self.time.clocks())

    def frame(self, frame=None, **kwargs):
        return self.guard.receive_frame(json.dumps(frame or self.time.frame(**kwargs)), **self.time.clocks())

    def command(self, cmd=BodyCommand(0.2)):
        return self.guard.receive_command(cmd, **self.time.clocks())

    def clear(self):
        for _ in range(3):
            self.time.advance()
            result = self.frame()
        return result

    def passing(self):
        self.clear()
        self.time.advance(0.001)
        self.assertEqual(self.command().command, BodyCommand(0.2))

    def test_initial_frames_do_not_replay_command_and_do_not_require_zero_without_motion(self):
        self.assertEqual(self.guard.evaluate(**self.time.clocks()).command, ZERO)
        self.assertEqual(self.clear().command, ZERO)
        # A receipt at the same clock instant as clearance is not new authority.
        self.assertEqual(self.command().command, ZERO)
        self.time.advance(0.001)
        self.assertEqual(self.command().command, BodyCommand(0.2))

    def test_camera_ready_status_distinguishes_idle_input_from_invalid_input(self):
        result = self.clear()
        self.assertEqual(result.status["state"], "hold")
        self.assertEqual(result.status["input_state"], "missing")
        self.assertEqual(result.status["camera_state"], "clear")
        self.assertTrue(result.status["camera_permitted"])
        self.assertEqual(result.status["max_frame_age_s"], 0.35)
        self.time.advance(0.001)
        self.assertEqual(self.command(ZERO).status["input_state"], "valid")
        self.time.advance(0.15)
        result = self.guard.evaluate(**self.time.clocks())
        self.assertEqual(result.status["input_state"], "expired")
        self.assertTrue(result.status["camera_permitted"])
        result = self.command(BodyCommand(float("nan")))
        self.assertEqual(result.status["input_state"], "invalid")
        self.assertTrue(result.status["camera_permitted"])
        self.time.ros += CLOCK_JUMP_NS + 1
        self.assertFalse(self.guard.evaluate(**self.time.clocks()).status["camera_permitted"])

    def test_status_carries_its_sampling_time_without_refreshing_capture(self):
        result = self.clear()
        sampled = self.time.mono
        age = result.status["frame_age_s"]
        self.assertEqual(result.status["sampled_at_monotonic_ns"], sampled)
        self.time.advance(0.2)
        # Retaining/transmitting an old result must retain its original time.
        self.assertEqual(result.status["sampled_at_monotonic_ns"], sampled)
        later = self.guard.evaluate(**self.time.clocks()).status
        self.assertEqual(later["sampled_at_monotonic_ns"], self.time.mono)
        self.assertAlmostEqual(later["frame_age_s"], age + .2)
        self.assertEqual(later["sequence"], result.status["sequence"])

    def test_startup_vetoed_motion_requires_zero_then_fresh_command(self):
        self.assertEqual(self.command().command, ZERO)
        self.clear()
        self.time.advance(0.001)
        self.assertEqual(self.command().command, ZERO)
        self.assertTrue(self.command().status["requires_post_hold_zero"])
        self.command(ZERO)
        self.time.advance(0.001)
        self.assertEqual(self.command().command, BodyCommand(0.2))

    def test_all_classes_veto_the_actual_nonzero_body_command(self):
        for label in ("32", "ball", "person", "tower", "unrecognized", ""):
            with self.subTest(label=label):
                self.setUp()
                self.passing()
                self.time.advance()
                result = self.frame(detections=[obstacle(label)])
                self.assertEqual(result.command, ZERO)
                self.assertEqual(result.status["state"], "hold")
                self.assertIn("near detection", result.status["reason"])

    def test_repeated_20hz_mux_command_cannot_resume_prehold_action(self):
        self.passing()
        self.time.advance()
        self.assertEqual(self.frame(detections=[obstacle()]).command, ZERO)
        for index in range(8):
            self.time.advance(0.05)
            self.frame()
            self.time.advance(0.001)
            result = self.command()  # Copies of the old finite action from mux.
            self.assertEqual(result.command, ZERO, index)
            self.assertTrue(result.status["requires_post_hold_zero"])
        # Mux timeout/upstream cancellation emits zero, then next deliberate action.
        self.time.advance(0.05)
        self.command(ZERO)
        self.time.advance(0.05)
        self.assertEqual(self.command().command, BodyCommand(0.2))

    def test_mux_stall_timeout_before_camera_hold_does_not_cancel_old_intent(self):
        self.passing()
        self.time.advance(0.15)
        result = self.guard.evaluate(**self.time.clocks())
        self.assertEqual(result.command, ZERO)
        self.assertIn("receipt expired", result.status["reason"])
        self.time.advance(0.01)
        self.frame(detections=[obstacle()])
        for _ in range(3):
            self.time.advance(0.03)
            self.frame()
        self.time.advance(0.01)
        result = self.command()  # Stalled mux resumes its old nonzero value.
        self.assertEqual(result.command, ZERO)
        self.assertTrue(result.status["requires_post_hold_zero"])
        self.command(ZERO)
        self.time.advance(0.001)
        self.assertEqual(self.command().command, BodyCommand(0.2))

    def test_invalid_command_during_hold_requires_zero_before_later_inbounds_value(self):
        for invalid in (BodyCommand(0.5), BodyCommand(float("nan")), None):
            with self.subTest(invalid=invalid):
                self.setUp()
                self.assertEqual(self.command(invalid).command, ZERO)
                self.clear()
                self.time.advance(0.001)
                result = self.command(BodyCommand(0.4))
                self.assertEqual(result.command, ZERO)
                self.assertTrue(result.status["requires_post_hold_zero"])
                self.command(ZERO)
                self.time.advance(0.001)
                self.assertEqual(self.command(BodyCommand(0.4)).command, BodyCommand(0.4))

    def test_zero_before_hold_cannot_release_motion_requested_during_hold(self):
        self.command(ZERO)
        self.time.advance()
        self.command()
        self.clear()
        self.time.advance()
        self.assertEqual(self.command().command, ZERO)

    def test_zero_during_hold_then_nonzero_still_held_recreates_barrier(self):
        self.passing()
        self.time.advance()
        self.frame(detections=[obstacle()])
        self.time.advance()
        self.command(ZERO)
        self.time.advance()
        self.command()
        self.clear()
        self.time.advance()
        self.assertEqual(self.command().command, ZERO)

    def test_camera_expiry_even_with_continuing_fresh_commands(self):
        self.passing()
        for _ in range(5):
            self.time.advance(0.05)
            self.assertNotEqual(self.command().command, ZERO)
        self.time.advance(0.05)
        result = self.command()
        self.assertEqual(result.command, ZERO)
        self.assertGreaterEqual(result.status["frame_age_s"], 0.35)
        self.clear()
        self.time.advance(0.001)
        self.assertEqual(self.command().command, ZERO)

    def test_command_expiry_without_any_messages(self):
        self.passing()
        self.time.advance((INPUT_TIMEOUT_NS - 1) / 1e9)
        self.assertNotEqual(self.guard.evaluate(**self.time.clocks()).command, ZERO)
        self.time.advance(1e-9)
        result = self.guard.evaluate(**self.time.clocks())
        self.assertEqual(result.command, ZERO)
        self.assertIn("receipt expired", result.status["reason"])

    def test_delayed_detection_cannot_use_receipt_time_as_capture(self):
        self.passing()
        self.time.advance(0.5)
        result = self.frame(age_ns=400_000_000)
        self.assertEqual(result.command, ZERO)
        self.assertEqual(result.status["frame_age_s"], 0.45)
        self.assertEqual(result.status["clear_frames"], 0)

    def test_capture_nanosecond_future_rejected_at_large_ros_epoch(self):
        self.passing()
        self.time.advance()
        result = self.frame(age_ns=-1)
        self.assertEqual(result.command, ZERO)
        self.assertIn("future", result.status["reason"])

    def test_duplicate_sequence_or_capture_resets_clearance(self):
        for field in ("sequence", "captured_at_ros_ns"):
            with self.subTest(field=field):
                self.setUp()
                old = self.time.frame()
                self.frame(old)
                self.time.advance()
                new = self.time.frame()
                new[field] = old[field]
                result = self.frame(new)
                self.assertEqual(result.command, ZERO)
                self.assertIn("regressed", result.status["reason"])
                self.time.advance()
                self.assertEqual(self.frame().status["clear_frames"], 1)

    def test_unhealthy_identity_cannot_be_resent_as_healthy(self):
        frame = self.time.frame()
        frame["healthy"] = False
        self.frame(frame)
        frame["healthy"] = True
        self.time.advance(0.001)
        result = self.frame(frame)
        self.assertIn("regressed", result.status["reason"])
        self.assertEqual(result.status["clear_frames"], 0)

    def test_producer_restart_latches_and_old_producer_cannot_regain_authority(self):
        self.passing()
        self.time.advance()
        result = self.frame(producer="camera-b")
        self.assertEqual(result.status["state"], "fault")
        for _ in range(4):
            self.time.advance()
            self.frame(producer="camera-a")
            self.command(ZERO)
            self.assertEqual(self.command().command, ZERO)
        self.assertEqual(self.guard.evaluate(**self.time.clocks()).status["state"], "fault")
        # Explicit fresh process can bind B but must earn its own clearance.
        self.guard = BodyCommandGuard()
        self.guard.evaluate(**self.time.clocks())
        self.time.advance()
        self.assertEqual(self.frame(producer="camera-b").status["clear_frames"], 1)

    def test_clock_jumps_regressions_and_ros_pause_latch(self):
        for ros_delta, mono_delta in ((CLOCK_JUMP_NS + 1, 0), (-1, 0),
                                      (0, CLOCK_JUMP_NS + 1), (0, -1)):
            with self.subTest(ros_delta=ros_delta, mono_delta=mono_delta):
                self.setUp()
                self.passing()
                self.time.ros += ros_delta
                self.time.mono += mono_delta
                result = self.guard.evaluate(**self.time.clocks())
                self.assertEqual(result.command, ZERO)
                self.assertEqual(result.status["state"], "fault")
                self.time.advance()
                self.assertEqual(self.command().status["state"], "fault")

    def test_clock_small_cumulative_drift_cannot_evade_jump_limit(self):
        self.passing()
        for _ in range(5):
            self.time.advance(0.01)
            self.time.ros += 10_000_000
            self.assertNotEqual(self.guard.evaluate(**self.time.clocks()).status["state"], "fault")
        self.time.advance(0.01)
        self.time.ros += 1
        self.assertEqual(self.guard.evaluate(**self.time.clocks()).status["state"], "fault")

    def test_subthreshold_forward_ros_step_does_not_extend_capture_freshness(self):
        self.passing()
        # Last source capture is already 1 ms old after passing().
        for _ in range(2):
            self.time.advance(0.1)
            self.assertNotEqual(self.command().command, ZERO)
        self.time.advance(0.099)
        self.time.ros += 40_000_000  # Below the 50 ms permanent clock-fault limit.
        result = self.guard.evaluate(**self.time.clocks())
        self.assertEqual(result.command, ZERO)
        self.assertEqual(result.status["state"], "hold")
        self.assertEqual(result.status["frame_age_s"], 0.35)
        self.assertIn("expired", result.status["reason"])

    def test_receipt_offset_drop_cannot_rejuvenate_delayed_original_capture(self):
        self.passing()
        captured_ros, captured_mono = self.time.ros, self.time.mono
        self.time.advance(0.3)
        self.time.ros -= 40_000_000
        frame = self.time.frame()
        frame["captured_at_ros_ns"] = captured_ros
        self.assertEqual(self.frame(frame).command, ZERO)
        self.time.advance(0.06)
        result = self.command()
        self.assertEqual((self.time.mono - captured_mono) / 1e9, 0.36)
        self.assertEqual(result.command, ZERO)
        self.assertGreaterEqual(result.status["frame_age_s"], 0.36)

    def test_positive_capture_offset_then_lower_receipt_offset_cannot_rejuvenate(self):
        for receipt_offset in (0, -40_000_000):
            with self.subTest(receipt_offset=receipt_offset):
                self.setUp()
                self.passing()
                self.time.advance(0.01)
                self.time.ros += 40_000_000
                self.command()  # Observe +40 ms, within the admitted window.
                captured_ros, captured_mono = self.time.ros, self.time.mono
                self.time.advance(0.28)
                self.time.ros += receipt_offset - 40_000_000
                frame = self.time.frame()
                frame["captured_at_ros_ns"] = captured_ros
                self.frame(frame)
                self.time.advance(0.08)
                result = self.command()
                self.assertEqual((self.time.mono - captured_mono) / 1e9, 0.36)
                self.assertEqual(result.command, ZERO)
                self.assertGreaterEqual(result.status["frame_age_s"], 0.36)

    def test_guard_rejects_preanchor_history_then_accepts_new_distinct_captures(self):
        result = self.frame()
        self.assertEqual(result.command, ZERO)
        self.assertIn("predates guard clock history", result.status["reason"])
        self.assertEqual(result.status["clear_frames"], 0)
        self.time.advance(0.049999999)
        self.assertEqual(self.frame().status["clear_frames"], 0)
        self.time.advance(1e-9)
        self.assertEqual(self.frame().status["clear_frames"], 1)

    def test_stable_clock_reserve_expires_at_300ms_true_capture_age(self):
        self.passing()
        self.time.advance(0.298999999)
        # Source capture is 299,999,999 ns old; the reserved age is below350 ms.
        self.assertNotEqual(self.command().command, ZERO)
        self.time.advance(1e-9)
        result = self.command()
        self.assertEqual(result.command, ZERO)
        self.assertEqual(result.status["frame_age_s"], 0.35)

    def test_invalid_clocks_fail_closed_without_exception(self):
        for field, value in (("ros_ns", 0), ("ros_ns", 1.0), ("mono_ns", -1),
                             ("mono_ns", True), ("mono_ns", 10**1000)):
            with self.subTest(field=field, value=str(value)[:30]):
                clocks = self.time.clocks()
                clocks[field] = value
                result = BodyCommandGuard().evaluate(**clocks)
                self.assertEqual(result.command, ZERO)
                self.assertEqual(result.status["state"], "fault")

    def test_exact_walk_bounds_pass_and_unused_axes_are_zero(self):
        self.clear()
        self.time.advance()
        for command in (BodyCommand(0.49, 0.3, 1.0), BodyCommand(-0.3, -0.3, -1.0)):
            with self.subTest(command=command):
                self.assertEqual(self.command(command).command, command)

    def test_nonfinite_unbounded_or_unused_axis_command_revokes_previous_motion(self):
        invalid = [BodyCommand(0.490001), BodyCommand(-0.300001), BodyCommand(vy=0.300001),
                   BodyCommand(wz=1.000001), BodyCommand(vz=0.01), BodyCommand(wx=0.01),
                   BodyCommand(wy=0.01), BodyCommand(float("nan")), BodyCommand(vy=float("inf")),
                   BodyCommand(wz=-float("inf")), BodyCommand(True), BodyCommand(10**1000), None]
        for command in invalid:
            with self.subTest(command=str(command)[:80]):
                self.setUp()
                self.passing()
                self.time.advance(0.001)
                result = self.command(command)
                self.assertEqual(result.command, ZERO)
                self.assertEqual(result.status["state"], "hold")

    def test_malformed_envelopes_revoke_previous_motion(self):
        edits = [("schema", "other"), ("producer_instance", ""), ("producer_instance", "a" * 129),
                 ("format", "pixels"), ("image_width", 699), ("image_width", 700.0),
                 ("image_height", 573), ("horizontal_fov_deg", 219), ("valid", 1),
                 ("healthy", None), ("sequence", True), ("captured_at_ros_ns", 1.0),
                 ("detections", None), ("detections", [obstacle()] * (MAX_DETECTIONS + 1)),
                 ("detections", [dict(box=[0.4, 0.3, 0.6, 1.1], confidence=0.9, class_id="x")]),
                 ("detections", [dict(box=[0.4, 0.3, 0.6, 0.8], confidence=float("nan"), class_id="x")])]
        for field, value in edits:
            with self.subTest(field=field, value=str(value)[:80]):
                self.setUp()
                self.passing()
                self.time.advance()
                bad = self.time.frame()
                bad[field] = value
                self.assertEqual(self.frame(bad).command, ZERO)
                self.assertEqual(self.guard.evaluate(**self.time.clocks()).status["clear_frames"], 0)

    def test_malformed_json_duplicate_fields_and_oversized_payloads_fail_closed(self):
        for payload in ("{", "[]", '{"schema":"fc_camera_obstacles_v1","schema":"x"}',
                        " " * 65537, "[" * 1200 + "]" * 1200, None):
            with self.subTest(payload=str(payload)[:60]):
                self.setUp()
                self.passing()
                self.time.advance()
                self.assertEqual(self.guard.receive_frame(payload, **self.time.clocks()).command, ZERO)

    def test_clearance_credit_expires_between_frames(self):
        self.frame()
        self.time.advance(0.36)
        self.assertEqual(self.frame().status["clear_frames"], 1)
        self.time.advance()
        self.assertEqual(self.frame().status["clear_frames"], 2)


class ArenaContactTests(unittest.TestCase):
    frame, command = GuardTests.frame, GuardTests.command
    clear, passing = GuardTests.clear, GuardTests.passing

    def setUp(self):
        self.guard = BodyCommandGuard(arena_ball_contact=True)
        self.time = Timeline()
        self.guard.evaluate(**self.time.clocks())

    def test_exact_ball_can_earn_clearance_before_any_strike_or_command(self):
        for expected in (1, 2, 3):
            self.time.advance()
            result = self.frame(detections=[obstacle("32")])
            self.assertEqual(result.status["clear_frames"], expected)
            self.assertEqual(result.status["ignored_ball_count"], 1)
            self.assertEqual(result.status["camera_permitted"], expected == 3)
            self.assertEqual(result.command, ZERO)
        self.assertEqual(result.status["contact_policy"], "arena_ball_contact")
        self.time.advance(.001)
        self.assertEqual(self.command().command, BodyCommand(.2))

    def test_ball_never_suppresses_overlapping_person_other_or_unknown_class(self):
        for label in ("0", "person", "56", "unknown", "", "ball", "032", "32 ", "32.0"):
            with self.subTest(label=label):
                self.setUp()
                self.passing()
                self.time.advance()
                result = self.frame(detections=[obstacle("32"), obstacle(label)])
                self.assertEqual(result.command, ZERO)
                self.assertEqual(result.status["camera_state"], "blocked")
                self.assertEqual(result.status["ignored_ball_count"], 1)

    def test_missing_wrong_or_malformed_identity_never_grants_arena_clearance(self):
        bad_identities = [None, {}, "coco", [],
            dict(ARENA_DETECTOR_IDENTITY, model_sha256="0" * 64),
            dict(ARENA_DETECTOR_IDENTITY, labels_sha256="0" * 64),
            dict(ARENA_DETECTOR_IDENTITY, extra="unselected")]
        for identity in bad_identities:
            with self.subTest(identity=identity):
                self.setUp()
                self.passing()
                for _ in range(3):
                    self.time.advance()
                    frame = self.time.frame(detections=[obstacle("32")])
                    if identity is None:
                        frame.pop("detector_identity")
                    else:
                        frame["detector_identity"] = identity
                    result = self.frame(frame)
                    self.assertEqual(result.command, ZERO)
                    self.assertFalse(result.status["camera_permitted"])
                    self.assertEqual(result.status["ignored_ball_count"], 0)
                    self.assertIn("detector identity", result.status["reason"])

    def test_malformed_ball_is_validated_before_exemption(self):
        invalid = [dict(obstacle("32"), box=[.4, .3, .6, 1.1]),
                   dict(obstacle("32"), confidence=float("nan")),
                   dict(obstacle("32"), class_id=32),
                   dict(obstacle("32"), box=[.4, .3, .4, .8])]
        for detection in invalid:
            with self.subTest(detection=detection):
                self.setUp()
                self.passing()
                self.time.advance()
                result = self.frame(detections=[detection])
                self.assertEqual(result.command, ZERO)
                self.assertEqual(result.status["camera_state"], "invalid")
                self.assertEqual(result.status["ignored_ball_count"], 0)

    def test_ball_exemption_cannot_bypass_health_projection_freshness_or_restart(self):
        edits = [("valid", False), ("healthy", False), ("image_width", 699),
                 ("format", "pixels"), ("captured_at_ros_ns", "stale"),
                 ("captured_at_ros_ns", "future"), ("producer_instance", "restarted")]
        for field, value in edits:
            with self.subTest(field=field, value=value):
                self.setUp()
                self.passing()
                self.time.advance(.05)
                frame = self.time.frame(detections=[obstacle("32")])
                frame[field] = (self.time.ros - 350_000_000 if value == "stale" else
                                self.time.ros + 1 if value == "future" else value)
                result = self.frame(frame)
                self.assertEqual(result.command, ZERO)
                self.assertFalse(result.status["camera_permitted"])
                self.assertEqual(result.status["ignored_ball_count"], 0)
                if field == "producer_instance":
                    self.time.advance()
                    self.assertEqual(self.frame().status["state"], "fault")

    def test_ball_frame_identity_cannot_be_replayed_and_expires_without_new_input(self):
        self.passing()
        self.time.advance()
        frame = self.time.frame(detections=[obstacle("32")])
        self.frame(frame)
        self.time.advance(.001)
        result = self.frame(frame)
        self.assertEqual(result.command, ZERO)
        self.assertIn("regressed", result.status["reason"])
        self.clear()
        self.time.advance(.001)
        self.command(ZERO)
        self.command()
        self.time.advance(.3)
        result = self.guard.evaluate(**self.time.clocks())
        self.assertFalse(result.status["camera_permitted"])
        self.assertEqual(result.status["ignored_ball_count"], 0)

    def test_person_hold_then_ball_clear_still_requires_upstream_zero_and_new_command(self):
        self.passing()
        self.time.advance()
        self.frame(detections=[obstacle("0"), obstacle("32")])
        for _ in range(3):
            self.time.advance()
            self.frame(detections=[obstacle("32")])
            self.time.advance(.001)
            result = self.command()
            self.assertEqual(result.command, ZERO)
        self.assertTrue(result.status["camera_permitted"])
        self.assertTrue(result.status["requires_post_hold_zero"])
        self.command(ZERO)
        self.time.advance(.001)
        self.assertEqual(self.command().command, BodyCommand(.2))

    def test_default_without_identity_keeps_ball_blocking_and_nonball_replay_is_unchanged(self):
        default = BodyCommandGuard()
        default.evaluate(**self.time.clocks())
        for detections in ([], [], [], [obstacle("0")], [], [], [], [obstacle("56")]):
            self.time.advance()
            frame = self.time.frame(detections=detections)
            selected = self.frame(frame)
            frame.pop("detector_identity")
            stock = default.receive_frame(json.dumps(frame), **self.time.clocks())
            self.assertEqual(selected.command, stock.command)
            for field in ("camera_state", "camera_permitted", "clear_frames", "frame_age_s"):
                self.assertEqual(selected.status[field], stock.status[field])
            self.assertEqual(stock.status["contact_policy"], "all_classes")
            self.assertEqual(stock.status["ignored_ball_count"], 0)
        self.time.advance()
        frame = self.time.frame(detections=[obstacle("32")])
        frame.pop("detector_identity")
        result = default.receive_frame(json.dumps(frame), **self.time.clocks())
        self.assertEqual(result.status["camera_state"], "blocked")
        self.assertIn("(32)", result.status["reason"])

    def test_policy_selection_requires_an_explicit_boolean(self):
        for value in (1, "true", None):
            with self.assertRaises(ValueError):
                BodyCommandGuard(arena_ball_contact=value)


class AdapterTests(unittest.TestCase):
    """Capture real factory wiring with a minimal fake ROS API; no robot services."""

    def setUp(self):
        self.time = Timeline()
        self.published, self.subscriptions, self.timers = {}, {}, []
        owner = self

        class Twist:
            def __init__(self):
                self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

        class String:
            def __init__(self, data=""):
                self.data = data

        class Node:
            def __init__(self, name):
                self.name = name

            def create_publisher(self, kind, topic, depth):
                owner.assertEqual(depth, 1)
                owner.published[topic] = []
                return types.SimpleNamespace(publish=lambda msg: owner.published[topic].append(copy.deepcopy(msg)))

            def create_subscription(self, kind, topic, callback, depth):
                owner.subscriptions[topic] = (callback, depth)

            def create_timer(self, period, callback, *, clock):
                owner.timers.append((period, callback, clock))

            def get_clock(self):
                return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=owner.time.ros))

        class Clock:
            def __init__(self, *, clock_type):
                self.clock_type = clock_type

        modules = {}
        for name, attrs in {
            "rclpy": {}, "rclpy.node": {"Node": Node}, "rclpy.clock": {"Clock": Clock},
            "rclpy.clock_type": {"ClockType": types.SimpleNamespace(STEADY_TIME="steady")},
            "geometry_msgs": {}, "geometry_msgs.msg": {"Twist": Twist},
            "std_msgs": {}, "std_msgs.msg": {"String": String},
        }.items():
            modules[name] = types.ModuleType(name)
            vars(modules[name]).update(attrs)
        self.patch = patch.dict(sys.modules, modules)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.node = create_ros_node(monotonic_ns=lambda: self.time.mono)
        self.Twist, self.String = Twist, String

    def send_command(self, speed=0.2):
        msg = self.Twist()
        msg.linear.x = speed
        self.subscriptions[INPUT_TOPIC][0](msg)

    def send_frame(self):
        self.subscriptions[FRAME_TOPIC][0](self.String(data=json.dumps(self.time.frame())))

    def test_explicit_depth_one_steady_timer_and_no_controller_clients(self):
        self.assertEqual(set(self.subscriptions), {INPUT_TOPIC, FRAME_TOPIC})
        self.assertTrue(all(depth == 1 for _, depth in self.subscriptions.values()))
        self.assertEqual(set(self.published), {OUTPUT_TOPIC, STATUS_TOPIC})
        self.assertEqual(self.timers[0][0], TIMER_S)
        self.assertEqual(self.timers[0][2].clock_type, "steady")
        # Fake Node has no service/client APIs: factory cannot arm or touch motors.

    def test_factory_selects_arena_and_cli_preserves_ros_arguments(self):
        node = create_ros_node(monotonic_ns=lambda: self.time.mono, arena_ball_contact=True)
        self.assertEqual(node.guard.evaluate(**self.time.clocks()).status["contact_policy"],
                         "arena_ball_contact")
        with patch("final_project.behaviors.body_command_guard.create_ros_node") as factory:
            ros = sys.modules["rclpy"]
            ros.init, ros.spin, ros.ok, ros.try_shutdown = (unittest.mock.Mock() for _ in range(4))
            exceptions = types.ModuleType("rclpy.executors")
            exceptions.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
            implementation = types.ModuleType("rclpy.impl.implementation_singleton")
            implementation.rclpy_implementation = types.SimpleNamespace(RCLError=RuntimeError)
            with patch.dict(sys.modules, {"rclpy.executors": exceptions,
                                         "rclpy.impl.implementation_singleton": implementation}):
                main(["--arena-ball-contact", "--ros-args", "-r", "__ns:=/candidate"])
            factory.assert_called_once_with(arena_ball_contact=True)
            ros.init.assert_called_once_with(args=["--ros-args", "-r", "__ns:=/candidate"])

    def test_timer_stops_output_without_callbacks_and_status_stays_bounded(self):
        for _ in range(3):
            self.time.advance()
            self.send_frame()
        self.time.advance(0.001)
        self.send_command()
        self.assertEqual(self.published[OUTPUT_TOPIC][-1].linear.x, 0.2)
        status_start = len(self.published[STATUS_TOPIC])
        for _ in range(100):
            self.time.advance(TIMER_S)
            self.timers[0][1]()
        self.assertEqual(self.published[OUTPUT_TOPIC][-1].linear.x, 0)
        self.assertLessEqual(len(self.published[STATUS_TOPIC]) - status_start, 10)
        for msg in self.published[STATUS_TOPIC]:
            self.assertLess(len(msg.data), 1024)
            status = json.loads(msg.data)
            self.assertIn("frame_age_s", status)
            self.assertIn("input_age_s", status)

    def test_ros_pause_does_not_pause_watchdog_timer(self):
        for _ in range(3):
            self.time.advance()
            self.send_frame()
        self.time.advance(0.001)
        self.send_command()
        self.time.mono += CLOCK_JUMP_NS + 1
        self.timers[0][1]()
        self.assertEqual(self.published[OUTPUT_TOPIC][-1].linear.x, 0)
        self.assertEqual(self.node.guard.evaluate(**self.time.clocks()).status["state"], "fault")

    def test_adapter_zeroes_every_unused_axis_and_rejects_nonzero_unused_input(self):
        for _ in range(3):
            self.time.advance()
            self.send_frame()
        self.time.advance(0.001)
        self.send_command()
        output = self.published[OUTPUT_TOPIC][-1]
        self.assertEqual((output.linear.z, output.angular.x, output.angular.y), (0, 0, 0))
        bad = self.Twist()
        bad.linear.x, bad.angular.x = 0.2, 0.1
        self.subscriptions[INPUT_TOPIC][0](bad)
        output = self.published[OUTPUT_TOPIC][-1]
        self.assertEqual((output.linear.x, output.linear.y, output.linear.z,
                          output.angular.x, output.angular.y, output.angular.z), (0,) * 6)


if __name__ == "__main__":
    unittest.main()
