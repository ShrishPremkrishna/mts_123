"""Calibrate the headset and turn fresh samples into a bounded motion intent."""

from dataclasses import dataclass
import math
from statistics import median
from typing import Iterable

import numpy as np

from .limits import FORWARD_SPEED, MAX_YAW_RATE
from .signals import EEG_RAIL_UV, FocusThresholds, WINDOW_SIZE, calibrate_focus, focus_power, tilt_degrees
from .source import SAMPLE_RATE, Sample

INPUT_MAX_AGE = 0.15
FOCUS_MAX_AGE = WINDOW_SIZE / SAMPLE_RATE + INPUT_MAX_AGE
TILT_DEADBAND = 5.0
TILT_FULL_SCALE = 25.0
MIN_TURN_RATE = 0.12  # Above the stock walker's 0.1 rad/s heading-hold threshold.


@dataclass(frozen=True)
class Calibration:
    focus: FocusThresholds
    neutral_tilt_deg: float


@dataclass(frozen=True)
class Motion:
    forward: float
    yaw: float
    state: str
    focus: float | None
    tilt_deg: float | None


class _EEGWindows:
    def __init__(self) -> None:
        self.last: Sample | None = None
        self.started = 0.0
        self.epoch: list[tuple[float, float]] = []

    def add(self, sample: Sample) -> float | None:
        previous = self.last
        self.last = sample
        if not math.isfinite(sample.t) or not all(math.isfinite(x) and abs(x) < EEG_RAIL_UV for x in sample.eeg_uv):
            raise ValueError("invalid or clipped EEG sample")
        if previous and (
            sample.sequence != (previous.sequence + 1) % 256
            or not 0 <= sample.t - previous.t <= INPUT_MAX_AGE
        ):
            raise ValueError("discontinuous Cyton samples")
        if not self.epoch:
            self.started = sample.t
        self.epoch.append(sample.eeg_uv)
        if len(self.epoch) < WINDOW_SIZE:
            return None
        block = np.asarray(self.epoch)
        self.epoch.clear()
        # Measure over the whole epoch so USB buffering can vary packet times
        # without making a different sample rate look like 250 Hz.
        expected_duration = (WINDOW_SIZE - 1) / SAMPLE_RATE
        if abs(sample.t - self.started - expected_duration) > INPUT_MAX_AGE:
            raise ValueError("Cyton sample timing is inconsistent with 250 Hz")
        return focus_power(block)


def _calibration_windows(samples: Iterable[Sample]) -> tuple[list[float], list[float]]:
    window = _EEGWindows()
    scores = []
    tilts = []
    for sample in samples:
        try:
            score = window.add(sample)
        except ValueError:
            window.epoch.clear()
        else:
            if score is not None:
                scores.append(score)
        if sample.accel_g is not None:
            try:
                tilts.append(tilt_degrees(sample.accel_g))
            except ValueError:
                pass
    return scores, tilts


def calibrate(rest: Iterable[Sample], focused: Iterable[Sample]) -> Calibration:
    rest_scores, rest_tilts = _calibration_windows(rest)
    focus_scores, focus_tilts = _calibration_windows(focused)
    if min(len(rest_tilts), len(focus_tilts)) < 25:
        raise ValueError("not enough valid accelerometer samples to center the headset")
    neutral_tilt = median(rest_tilts)
    for tilts in (rest_tilts, focus_tilts):
        if sum(abs(x - neutral_tilt) <= TILT_DEADBAND for x in tilts) < 0.9 * len(tilts):
            raise ValueError("head moved during calibration; keep it level and still in both stages")
    if abs(neutral_tilt) > 60:
        raise ValueError("Cyton X axis is nearly vertical; check the headset mounting")
    return Calibration(calibrate_focus(rest_scores, focus_scores), neutral_tilt)


class Controller:
    def __init__(self, calibration: Calibration, tilt_sign: int = 1) -> None:
        if tilt_sign not in (-1, 1):
            raise ValueError("tilt sign must be +1 or -1")
        self.calibration = calibration
        self.tilt_sign = tilt_sign
        self._last_step: float | None = None
        self._clear_inputs()

    def _clear_inputs(self) -> None:
        self._eeg = _EEGWindows()
        self._focus: tuple[float, float] | None = None
        self._tilt: tuple[float, float] | None = None
        self._saw_release = False
        self._focused = False

    def _ingest(self, sample: Sample, now: float) -> None:
        if not 0 <= now - sample.t <= INPUT_MAX_AGE:
            self._clear_inputs()
            return
        try:
            score = self._eeg.add(sample)
            if score is not None:
                self._focus = sample.t, score
            if sample.accel_g is not None:
                relative = tilt_degrees(sample.accel_g) - self.calibration.neutral_tilt_deg
                self._tilt = sample.t, relative * self.tilt_sign
        except ValueError:
            self._clear_inputs()

    def step(self, samples: Iterable[Sample], *, now: float, enabled: bool) -> Motion:
        if not math.isfinite(now):
            raise ValueError("invalid control clock")
        if self._last_step is not None and not 0 <= now - self._last_step <= INPUT_MAX_AGE:
            self._clear_inputs()
        self._last_step = now
        for sample in samples:
            self._ingest(sample, now)

        latest = self._eeg.last
        focus = self._focus[1] if self._focus and now - self._focus[0] <= FOCUS_MAX_AGE else None
        tilt = self._tilt[1] if self._tilt and now - self._tilt[0] <= INPUT_MAX_AGE else None
        if latest is None or now - latest.t > INPUT_MAX_AGE or focus is None or tilt is None:
            self._saw_release = self._focused = False
            return Motion(0.0, 0.0, "waiting_for_headset", focus, tilt)
        if not enabled:
            self._saw_release, self._focused = True, False
            return Motion(0.0, 0.0, "released", focus, tilt)
        if not self._saw_release:
            return Motion(0.0, 0.0, "release_to_enable", focus, tilt)

        thresholds = self.calibration.focus
        self._focused = focus >= (thresholds.exit if self._focused else thresholds.enter)
        yaw = 0.0
        if abs(tilt) > TILT_DEADBAND:
            turn = min(1.0, (abs(tilt) - TILT_DEADBAND) / (TILT_FULL_SCALE - TILT_DEADBAND))
            yaw = math.copysign(MIN_TURN_RATE + turn * (MAX_YAW_RATE - MIN_TURN_RATE), tilt)
        return Motion(FORWARD_SPEED if self._focused else 0.0, yaw, "enabled", focus, tilt)
