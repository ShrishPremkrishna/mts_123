"""IMU health monitor for the Pupper's BNO085 gyro fault (lane D, 2026-11-23).

The fault: the chip's gyro raw-X axis emits one-sided negative step-and-decay
pulses (15-40 ms). They pile up into a negative bias, the chip's own fusion
integrates them into a false roll/yaw, and the walker policy reacts to that
false tilt. This monitor watches the three things that matter:

  tilt_deg       body tilt from the quaternion (angle between body z and world z)
  pulse_density  fraction of recent samples that are an isolated negative raw-X pulse
  bias_x         median raw-X rate over the window (rad/s)
  stale_ms       time since the IMU sample last changed (frozen or missing stream)

and reduces them to one level:

  bad   tilt > bad_tilt_deg for tilt_hold_s while the command is zero
        (robot told to stand still, IMU says it is leaning: lie down), or the
        stream froze for stale_bad_ms.
  warn  pulse_density, |bias_x|, tilt or staleness past their warn limits.
  ok    otherwise.

Pure stdlib (runs on the Pi's system python3). Thresholds are params; defaults
were calibrated on kickseen4 (bad) and kickseen6 (mild episode, robot walked and
kicked fine): see fc/notes/lane-D-gyro.md and fc/robot/imu/replay_health.py.

Inputs are in the published base frame (/imu_sensor_broadcaster/imu): the driver
rotates the chip frame by the URDF mount (pitch -135 deg about y). We rotate the
gyro back, so raw X here is the chip's X axis, the faulty one.
"""
from __future__ import annotations

import math
import statistics
import time
from collections import deque

DEFAULTS = {
    "mount_pitch_deg": -135.0,   # IMU mount rotation about base y (URDF imu joint)
    "window_s": 10.0,            # rolling window for pulse density and bias
    "pulse_thr": 0.02,           # rad/s; raw X below -pulse_thr is a pulse sample
    "iso_thr": 0.01,             # rad/s; raw Y/Z must stay under this (real motion moves all axes)
    "warn_pulse_density": 0.20,  # fraction of window samples
    "warn_bias": 0.03,           # rad/s, |median raw X|
    "warn_tilt_deg": 6.0,        # tilt with zero command, held tilt_hold_s
    "bad_tilt_deg": 10.0,        # tilt with zero command, held tilt_hold_s -> bad
    "tilt_hold_s": 2.0,          # real tilts (kick, pick-up) in kickseen6 lasted <= 1.3 s
    "stale_warn_ms": 200.0,      # sample unchanged this long (normal repeats reach ~90 ms)
    "stale_bad_ms": 500.0,
    "hold_s": 3.0,               # a level persists this long after its condition clears
}

LEVELS = ("ok", "warn", "bad")


def raw_axes(gyro_xyz, mount_pitch_deg: float = -135.0):
    """Base-frame gyro -> chip-frame gyro for a mount rotated about y by pitch.

    base = Ry(p) @ chip, so chip = Ry(p)^T @ base. For p = -135 deg:
    raw X = -0.707 (gx - gz), raw Y = gy, raw Z = -0.707 (gx + gz).
    """
    gx, gy, gz = gyro_xyz
    p = math.radians(mount_pitch_deg)
    c, s = math.cos(p), math.sin(p)
    return (c * gx - s * gz, gy, s * gx + c * gz)


def tilt_deg(quat_xyzw) -> float:
    """Angle between body z and world z, degrees (heading-independent)."""
    x, y, z, w = quat_xyzw
    n = x * x + y * y + z * z + w * w
    if n <= 0.0:
        return float("nan")
    cz = 1.0 - 2.0 * (x * x + y * y) / n
    return math.degrees(math.acos(max(-1.0, min(1.0, cz))))


class ImuHealth:
    def __init__(self, params: dict | None = None, clock=time.monotonic):
        p = dict(DEFAULTS)
        if params:
            # Accept either bare keys or the fc params.json style "imu_<key>".
            for k in DEFAULTS:
                if k in params:
                    p[k] = params[k]
                elif "imu_" + k in params:
                    p[k] = params["imu_" + k]
        self.p = p
        self._clock = clock
        self._win = deque()          # (t, raw_x, is_pulse)
        self._pulse_n = 0
        self._last_t = None
        self._last_wall = None
        self._last_sample = None
        self._last_change_t = None
        self._tilt = float("nan")
        self._tilt_since = None      # start of the current zero-command over-limit tilt run
        self._tilt_warn_since = None
        self._until = {"warn": -math.inf, "bad": -math.inf}
        self._reasons: dict[str, str] = {}
        self._bad_cause = None
        self.n = 0

    # -- input -------------------------------------------------------------
    def update(self, t: float, quat_xyzw, gyro_xyz, cmd_zero: bool) -> None:
        """Feed one IMU sample. t in seconds, any monotonic clock (msg stamp is fine)."""
        p = self.p
        self.n += 1
        sample = (tuple(quat_xyzw), tuple(gyro_xyz))
        if sample != self._last_sample or self._last_change_t is None:
            self._last_change_t = t
            self._last_sample = sample
        self._last_t = t
        self._last_wall = self._clock()

        rx, ry, rz = raw_axes(gyro_xyz, p["mount_pitch_deg"])
        pulse = rx < -p["pulse_thr"] and abs(ry) < p["iso_thr"] and abs(rz) < p["iso_thr"]
        self._win.append((t, rx, pulse))
        self._pulse_n += pulse
        horizon = t - p["window_s"]
        while self._win and self._win[0][0] < horizon:
            self._pulse_n -= self._win.popleft()[2]

        density = self._pulse_n / len(self._win)
        if density > p["warn_pulse_density"] and len(self._win) >= 50:
            self._flag("warn", t, "pulse_density %.2f" % density)

        self._tilt = tilt_deg(quat_xyzw)
        self._tilt_since = self._held(self._tilt_since, t, cmd_zero and self._tilt > p["bad_tilt_deg"])
        self._tilt_warn_since = self._held(self._tilt_warn_since, t, cmd_zero and self._tilt > p["warn_tilt_deg"])
        if self._tilt_since is not None and t - self._tilt_since >= p["tilt_hold_s"]:
            self._flag("bad", t, "tilt %.1f deg at zero command" % self._tilt)
            self._bad_cause = "tilt"
        elif self._tilt_warn_since is not None and t - self._tilt_warn_since >= p["tilt_hold_s"]:
            self._flag("warn", t, "tilt %.1f deg at zero command" % self._tilt)

    @staticmethod
    def _held(since, t, cond):
        if not cond:
            return None
        return t if since is None else since

    def _flag(self, level: str, t: float, why: str) -> None:
        self._until[level] = t + self.p["hold_s"]
        self._reasons[level] = why

    # -- output ------------------------------------------------------------
    def now(self) -> float | None:
        """Current time in update-clock units, extrapolated by the wall clock."""
        if self._last_t is None:
            return None
        return self._last_t + (self._clock() - self._last_wall)

    def summary(self, now: float | None = None) -> dict:
        p = self.p
        if self._last_t is None:
            return {"tilt_deg": None, "pulse_density": None, "bias_x": None,
                    "stale_ms": None, "level": "warn", "reasons": ["no IMU samples"], "n": 0}
        if now is None:
            now = self.now()
        stale_ms = max(0.0, (now - self._last_change_t) * 1000.0)
        density = self._pulse_n / len(self._win) if self._win else 0.0
        bias = statistics.median(r[1] for r in self._win) if self._win else 0.0

        level, reasons, cause = "ok", [], None
        if stale_ms > p["stale_bad_ms"]:
            level, cause = "bad", "stale"
            reasons.append("IMU unchanged for %.0f ms" % stale_ms)
        elif stale_ms > p["stale_warn_ms"]:
            level = "warn"
            reasons.append("IMU unchanged for %.0f ms" % stale_ms)
        if abs(bias) > p["warn_bias"] and len(self._win) >= 50:
            self._flag("warn", now, "raw-X bias %+.3f rad/s" % bias)
        for lv in ("bad", "warn"):
            if now <= self._until[lv]:
                if LEVELS.index(lv) > LEVELS.index(level):
                    level = lv
                    if lv == "bad":
                        cause = self._bad_cause
                reasons.append(self._reasons[lv])
        return {
            "tilt_deg": round(self._tilt, 2),
            "pulse_density": round(density, 3),
            "bias_x": round(bias, 4),
            "stale_ms": round(stale_ms, 1),
            "level": level,
            "bad_cause": cause,     # "tilt" (lie down) | "stale" (stop; IMU frozen) | None
            "reasons": reasons,
            "n": self.n,
        }
