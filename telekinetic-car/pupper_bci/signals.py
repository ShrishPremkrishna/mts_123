"""Pure signal measurements for Cyton EEG and head tilt."""

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.signal import butter, sosfiltfilt, welch

from .source import SAMPLE_RATE

WINDOW_SIZE = 256
MIN_CALIBRATION_WINDOWS = 8

# Screen each unfiltered channel for the default gain-24 Cyton's near-rail
# values, gross transients, and flat traces. Smaller blink and muscle artifacts
# can pass this screen, so verify the focus gate with relaxed head movements.
EEG_RAIL_UV = 180_000.0
_MAX_SPAN_UV = 300.0
_MAX_STEP_UV = 100.0
_MIN_SPAN_UV = 0.1

_BANDPASS = butter(2, [4, 40], btype="bandpass", fs=SAMPLE_RATE, output="sos")


@dataclass(frozen=True)
class FocusThresholds:
    """Power thresholds for entering focus and returning to rest."""

    enter: float
    exit: float


def _finite_array(values: ArrayLike, name: str) -> NDArray[np.float64]:
    try:
        if np.iscomplexobj(values):
            raise ValueError("complex values")
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} requires real numeric values") from error
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains nonfinite values")
    return array


def focus_power(eeg_uv: np.ndarray) -> float:
    """Return mean 12–15 Hz power in µV² for one (256 samples, 2 channels) epoch."""

    eeg = _finite_array(eeg_uv, "EEG epoch")
    if eeg.shape != (WINDOW_SIZE, 2):
        raise ValueError(f"EEG epoch must have shape ({WINDOW_SIZE}, 2)")
    if np.any(np.abs(eeg) >= EEG_RAIL_UV):
        raise ValueError("EEG epoch approaches the amplifier rail")

    spans = np.ptp(eeg, axis=0)
    if np.any(spans < _MIN_SPAN_UV):
        raise ValueError("EEG epoch contains a flat channel")
    if np.any(spans > _MAX_SPAN_UV) or np.any(np.abs(np.diff(eeg, axis=0)) > _MAX_STEP_UV):
        raise ValueError("EEG epoch contains a gross transient")

    filtered = sosfiltfilt(_BANDPASS, eeg, axis=0)
    frequencies, density = welch(
        filtered, fs=SAMPLE_RATE, window="hann", nperseg=WINDOW_SIZE, axis=0
    )
    beta = (frequencies >= 12) & (frequencies <= 15)
    bin_width = frequencies[1] - frequencies[0]
    return float(np.sum(density[beta], axis=0).mean() * bin_width)


def calibrate_focus(rest_scores: ArrayLike, focus_scores: ArrayLike) -> FocusThresholds:
    """Fit hysteresis thresholds inside a separated rest/focus power range."""

    rest = _finite_array(rest_scores, "rest calibration")
    focus = _finite_array(focus_scores, "focus calibration")
    for scores in (rest, focus):
        if scores.ndim != 1 or scores.size < MIN_CALIBRATION_WINDOWS or np.any(scores < 0):
            raise ValueError(
                f"Each calibration stage needs at least {MIN_CALIBRATION_WINDOWS} nonnegative power scores"
            )

    # Use observed quantiles so one extreme score in either stage cannot
    # dominate the threshold through interpolation in a short calibration.
    rest_high = float(np.percentile(rest, 90, method="lower"))
    focus_low = float(np.percentile(focus, 10, method="higher"))
    gap = focus_low - rest_high
    exit_threshold = rest_high + gap / 3
    enter_threshold = focus_low - gap / 3
    if not rest_high < exit_threshold < enter_threshold < focus_low:
        raise ValueError("Calibration did not separate rest and focus power")
    return FocusThresholds(enter=enter_threshold, exit=exit_threshold)


def tilt_degrees(accel_g: tuple[float, float, float]) -> float:
    """Measure signed tilt indicated by X acceleration when near 1 g."""

    acceleration = _finite_array(accel_g, "accelerometer reading")
    if acceleration.shape != (3,):
        raise ValueError("Accelerometer reading must contain X, Y, and Z")
    x, y, z = acceleration
    # Large departures from gravity indicate missing or dynamic acceleration.
    # This check cannot distinguish all head motion from gravitational tilt.
    if not 0.75 <= math.hypot(x, y, z) <= 1.25:
        raise ValueError("Accelerometer magnitude is outside 0.75–1.25 g")
    return math.degrees(math.atan2(x, math.hypot(y, z)))
