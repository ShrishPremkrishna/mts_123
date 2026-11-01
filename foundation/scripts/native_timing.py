"""Simulator-only timing perturbation; not a replay of measured motor commands."""
import math
import re
from collections import deque


def validate_timing_profile(value):
    if not isinstance(value, dict) or value.get("schema") != "native_timing_replay_v1":
        raise ValueError("timing profile requires native_timing_replay_v1")
    if value.get("controller") != "trick":
        raise ValueError("timing perturbation is scoped to the trick controller")
    if any(type(value.get(k)) is not bool for k in ("use_recorded_cadence", "use_recorded_imu_age")):
        raise ValueError("timing enable fields must be booleans")
    intervals, ages = value.get("inference_intervals_s"), value.get("imu_age_s")
    if not isinstance(intervals, list) or not isinstance(ages, list) or len(intervals) < 2 or len(intervals) != len(ages):
        raise ValueError("timing intervals and ages require matching arrays of at least two samples")
    for values, low, high in ((intervals, 1/520, .1), (ages, 0, .25)):
        if any(type(x) not in (int, float) or not low <= x <= high or not math.isfinite(x) for x in values):
            raise ValueError("nonfinite or out-of-range timing sample")
    if not isinstance(value.get("source"), dict) or not isinstance(value["source"].get("sha256"), str) or not re.fullmatch(r"[0-9a-fA-F]{64}", value["source"]["sha256"]):
        raise ValueError("timing source identity is required")
    return value


class RecordedTiming:
    """Replay relative intervals cyclically; physics steps round due times upward."""
    def __init__(self, profile):
        self.profile = validate_timing_profile(profile)
        self.reset()

    def reset(self):
        self.index = 0
        self.next_offset_s = 0.

    def inference_due(self, elapsed_s, init_s):
        if not self.profile["use_recorded_cadence"]:
            return None  # preserve the native host's original every-ten-update path
        return elapsed_s >= init_s and elapsed_s - init_s + 1e-12 >= self.next_offset_s

    def imu_age_s(self):
        if not self.profile["use_recorded_imu_age"]:
            return 0.
        return self.profile["imu_age_s"][self.index % len(self.profile["imu_age_s"])]

    def record_inference(self, elapsed_s, init_s):
        desired = init_s + self.next_offset_s if self.profile["use_recorded_cadence"] else None
        value = {"timing_sample_index": self.index, "scheduled_elapsed_s": desired,
                 "actual_elapsed_s": elapsed_s,
                 "schedule_roundup_s": None if desired is None else elapsed_s - desired}
        self.next_offset_s += self.profile["inference_intervals_s"][self.index % len(self.profile["inference_intervals_s"])]
        self.index += 1
        return value


class IMUHistory:
    """Zero-order hold of coherent quaternion/gyro samples, never future interpolation."""
    def __init__(self, max_age_s, quaternion, angular_velocity):
        self.max_age_s = max_age_s
        # The simulated initial state is constant before world time zero.
        self.rows = deque([(-max_age_s - 1/520, list(quaternion), list(angular_velocity))])

    def append(self, time_s, quaternion, angular_velocity):
        if self.rows and time_s < self.rows[-1][0]:
            raise ValueError("IMU history time moved backwards")
        self.rows.append((time_s, list(quaternion), list(angular_velocity)))
        # Retain enough history even when requested ages grow on the next call.
        while len(self.rows) > 2 and self.rows[1][0] < time_s - self.max_age_s - 1/520:
            self.rows.popleft()

    def sample(self, time_s, age_s):
        if not math.isfinite(age_s) or not 0 <= age_s <= self.max_age_s:
            raise ValueError("IMU sample age outside retained history")
        cutoff = time_s - age_s
        for stamp, quat, angular in reversed(self.rows):
            if stamp <= cutoff + 1e-12:
                return stamp, list(quat), list(angular)
        raise ValueError("IMU history does not cover requested time")
