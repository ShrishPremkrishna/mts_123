"""Exercise the live command policy with a fake clock and real signal processing."""

from dataclasses import replace
import itertools
import math
import unittest

from pupper_bci.__main__ import demo_samples, replay
from pupper_bci.control import INPUT_MAX_AGE, Controller, calibrate
from pupper_bci.limits import COMMAND_PERIOD, FORWARD_SPEED, MAX_YAW_RATE
from pupper_bci.source import SAMPLE_RATE, Sample


class ControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.samples = demo_samples()
        cls.rest = cls.samples[:12 * SAMPLE_RATE]
        cls.focus = cls.samples[12 * SAMPLE_RATE:24 * SAMPLE_RATE]
        cls.calibration = calibrate(cls.rest, cls.focus)

    def setUp(self):
        self.controller = Controller(self.calibration)
        self.index = 0
        self.tick = 0

    def advance(self, steps, *, enabled=True, accel=(0.0, 0.0, 1.0), clipped=False):
        for _ in range(steps):
            self.tick += 1
            now = self.tick * COMMAND_PERIOD
            batch = []
            while self.index / SAMPLE_RATE <= now:
                t = self.index / SAMPLE_RATE
                beta = 12 * math.sin(2 * math.pi * 13 * t)
                eeg = (beta, 0.9 * beta) if not clipped else (180_000.0, beta)
                batch.append(Sample(t, self.index % 256, eeg, accel if self.index % 10 == 0 else None))
                self.index += 1
            output = self.controller.step(batch, now=now, enabled=enabled)
        return output

    def assert_stopped(self, output, state=None):
        self.assertEqual((output.forward, output.yaw), (0.0, 0.0))
        if state is not None:
            self.assertEqual(output.state, state)

    def test_demo_focus_and_tilt_are_independent_and_bounded(self):
        frames = dict((round(t, 2), output) for t, output in replay(self.samples, 12, 1))
        self.assert_stopped(frames[25.8], "enabled")
        self.assertGreater(frames[26.5].yaw, 0)
        self.assertLess(frames[28.5].yaw, 0)
        self.assertEqual(frames[26.5].forward, 0)
        self.assertEqual(frames[28.5].forward, 0)
        self.assertEqual(frames[32.5].forward, FORWARD_SPEED)
        self.assertEqual(frames[32.5].yaw, 0)
        self.assertGreater(frames[34.5].yaw, 0)
        self.assertEqual(frames[34.5].forward, FORWARD_SPEED)
        self.assert_stopped(frames[38], "enabled")
        self.assertEqual(frames[40.9].forward, FORWARD_SPEED)
        self.assert_stopped(frames[41.15], "waiting_for_headset")
        for command in frames.values():
            self.assertLessEqual(abs(command.yaw), MAX_YAW_RATE)

    def test_dry_run_reverse_steering_flag_preserves_forward(self):
        original = dict(replay(self.samples, 12, 1))
        reversed_steer = dict(replay(self.samples, 12, -1))
        for t, command in original.items():
            self.assertEqual(reversed_steer[t].yaw, -command.yaw)
            self.assertEqual(reversed_steer[t].forward, command.forward)

    def test_start_held_disconnect_and_reconnect_require_releasing(self):
        self.assert_stopped(self.advance(24), "release_to_enable")
        self.assert_stopped(self.advance(1, enabled=False), "released")
        self.assertEqual(self.advance(1).forward, FORWARD_SPEED)
        output = self.controller.step([], now=self.tick * COMMAND_PERIOD + INPUT_MAX_AGE + 0.01, enabled=True)
        self.assert_stopped(output, "waiting_for_headset")
        self.tick += 4
        self.index = self.tick * SAMPLE_RATE // 20
        self.assert_stopped(self.advance(24), "release_to_enable")
        self.assert_stopped(self.advance(1, enabled=False), "released")
        self.assertEqual(self.advance(1).forward, FORWARD_SPEED)

    def test_loss_of_acceleration_stops_both_axes_even_with_live_eeg(self):
        self.advance(24, enabled=False)
        self.assertEqual(self.advance(1).forward, FORWARD_SPEED)
        output = self.advance(4, accel=None)
        self.assert_stopped(output, "waiting_for_headset")
        self.assertIsNotNone(output.focus)
        self.assert_stopped(self.advance(1), "release_to_enable")

    def test_small_turn_overrides_stock_heading_hold_while_walking(self):
        self.advance(24, enabled=False)
        angle = math.radians(6)
        output = self.advance(2, accel=(math.sin(angle), 0.0, math.cos(angle)))
        self.assertEqual(output.forward, FORWARD_SPEED)
        self.assertGreater(output.yaw, 0.10)

    def test_clipped_sample_or_sequence_gap_revokes_motion(self):
        self.advance(24, enabled=False)
        self.assertEqual(self.advance(1).forward, FORWARD_SPEED)
        self.assert_stopped(self.advance(1, clipped=True), "waiting_for_headset")
        self.advance(24, enabled=False)
        self.assertEqual(self.advance(1).forward, FORWARD_SPEED)
        self.index += 1
        self.assert_stopped(self.advance(1), "waiting_for_headset")

    def test_replayed_or_future_samples_do_not_renew_motion(self):
        self.advance(24, enabled=False)
        self.assertEqual(self.advance(1).forward, FORWARD_SPEED)
        duplicate = Sample((self.index - 1) / SAMPLE_RATE, (self.index - 1) % 256, (1, 2), (0, 0, 1))
        future = replace(duplicate, t=self.tick * COMMAND_PERIOD + 0.5)
        output = self.controller.step([duplicate, future], now=(self.tick + 1) * COMMAND_PERIOD, enabled=True)
        self.assert_stopped(output, "waiting_for_headset")

    def test_invalid_calibration_fails_with_clear_reason(self):
        with self.assertRaisesRegex(ValueError, "separate"):
            calibrate(self.rest, self.rest)
        with self.assertRaisesRegex(ValueError, "accelerometer"):
            calibrate((replace(s, accel_g=None) for s in self.rest), self.focus)
        moved = (replace(s, accel_g=(0.5, 0.0, 0.86)) for s in self.focus)
        with self.assertRaisesRegex(ValueError, "head moved"):
            calibrate(self.rest, moved)

    def test_wrong_sampling_rate_never_qualifies_for_motion(self):
        slow_rest = (replace(sample, t=1.25 * sample.t) for sample in self.rest)
        with self.assertRaisesRegex(ValueError, "power scores"):
            calibrate(slow_rest, self.focus)

    def test_equal_timestamps_from_usb_batches_preserve_valid_windows(self):
        def batched_time(samples):
            return [replace(sample, t=round(sample.t / 0.016) * 0.016) for sample in samples]

        self.assertEqual(calibrate(batched_time(self.rest), batched_time(self.focus)), self.calibration)

    def test_calibration_does_not_shift_for_missing_initial_samples(self):
        samples = list(itertools.dropwhile(lambda sample: sample.t < 0.29, self.samples))
        self.assertEqual(next(replay(samples, 12, 1))[0], 24.0)


if __name__ == "__main__":
    unittest.main()
