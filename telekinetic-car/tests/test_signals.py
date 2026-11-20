import math
import unittest

import numpy as np

from pupper_bci.signals import (
    MIN_CALIBRATION_WINDOWS,
    SAMPLE_RATE,
    WINDOW_SIZE,
    calibrate_focus,
    focus_power,
    tilt_degrees,
)


def tone(bin_number: int, amplitude: float = 10.0) -> np.ndarray:
    frequency = bin_number * SAMPLE_RATE / WINDOW_SIZE
    time_seconds = np.arange(WINDOW_SIZE) / SAMPLE_RATE
    return amplitude * np.sin(2 * np.pi * frequency * time_seconds)


class FocusPowerTests(unittest.TestCase):
    def test_extracts_narrow_beta_and_averages_channels(self):
        beta = tone(14)
        theta = tone(8)
        higher_beta = tone(31)
        both_beta = focus_power(np.column_stack((beta, beta)))
        mixed = focus_power(np.column_stack((theta, beta)))
        outside_band = focus_power(np.column_stack((theta, higher_beta)))

        self.assertAlmostEqual(both_beta, 50.0, delta=2.0)
        self.assertAlmostEqual(mixed, both_beta / 2, delta=0.05)
        self.assertLess(outside_band, both_beta / 100)

    def test_keeps_the_power_scale_above_100(self):
        beta = tone(14, amplitude=25.0)
        self.assertGreater(focus_power(np.column_stack((beta, beta))), 300.0)

    def test_rejects_unusable_windows(self):
        beta = tone(14)
        valid = np.column_stack((beta, beta))
        nonfinite = valid.copy()
        nonfinite[32, 0] = np.nan
        saturated = valid.copy()
        saturated[32, 0] = -187_500
        spike = valid.copy()
        spike[32, 0] = 160
        large_swing = np.column_stack((tone(8, amplitude=175), beta))
        flat = np.column_stack((beta, np.zeros(WINDOW_SIZE)))

        for epoch, error in (
            (valid.T, "shape"),
            (nonfinite, "nonfinite"),
            (saturated, "rail"),
            (spike, "transient"),
            (large_swing, "transient"),
            (flat, "flat"),
        ):
            with self.subTest(error=error), self.assertRaisesRegex(ValueError, error):
                focus_power(epoch)


class CalibrationTests(unittest.TestCase):
    def test_places_thresholds_in_gap_despite_one_outlier_per_phase(self):
        rest = [2.0] * (MIN_CALIBRATION_WINDOWS - 1) + [50.0]
        focus = [0.1] + [14.0] * (MIN_CALIBRATION_WINDOWS - 1)
        thresholds = calibrate_focus(rest, focus)

        self.assertEqual(thresholds.exit, 6.0)
        self.assertEqual(thresholds.enter, 10.0)

    def test_rejects_overlapping_or_inverted_focus_ranges(self):
        rest = [3.0] * MIN_CALIBRATION_WINDOWS
        for focus in (rest, [2.0] * len(rest), [2.0, 2.0] + [7.0] * (len(rest) - 2)):
            with self.subTest(focus=focus), self.assertRaisesRegex(ValueError, "separate"):
                calibrate_focus(rest, focus)

    def test_rejects_incomplete_or_invalid_calibration(self):
        rest = [3.0] * MIN_CALIBRATION_WINDOWS
        for focus in (rest[:-1], [-1.0] * len(rest), [np.inf] * len(rest), [rest]):
            with self.subTest(focus=focus), self.assertRaises(ValueError):
                calibrate_focus(rest, focus)


class TiltTests(unittest.TestCase):
    def test_signed_gravity_referenced_tilt(self):
        for degrees in (-45, -15, 0, 15, 45):
            angle = math.radians(degrees)
            with self.subTest(degrees=degrees):
                self.assertAlmostEqual(tilt_degrees((math.sin(angle), 0, math.cos(angle))), degrees)

    def test_tilt_uses_gravity_direction_even_when_axis_values_are_scaled(self):
        self.assertAlmostEqual(tilt_degrees((0.55, 0.0, 1.1 * math.sqrt(3) / 2)), 30.0)

    def test_rejects_unusable_accelerometer_readings(self):
        for reading in ((0, 0, 0), (0, 0, 2), (np.nan, 0, 1), (0, 1), ("a", 0, 1)):
            with self.subTest(reading=reading), self.assertRaises(ValueError):
                tilt_degrees(reading)
