"""Small trackers: constant-velocity Kalman filter for xy, smoothed yaw with rate."""
from __future__ import annotations

import math

import numpy as np


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class CVFilter2D:
    """State [x, y, vx, vy]. Time-stamped: call update(z, t) with capture times."""

    def __init__(self, accel_sigma: float, meas_sigma: float, gate_sigma: float = 5.0):
        self.q = accel_sigma ** 2
        self.r = meas_sigma ** 2
        self.gate = gate_sigma ** 2
        self.x: np.ndarray | None = None
        self.P = np.eye(4)
        self.t = 0.0          # time the state refers to
        self.t_meas = -1e9    # time of last accepted measurement
        self._rejects = 0

    @property
    def initialised(self) -> bool:
        return self.x is not None

    def _reset(self, z, t):
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([self.r, self.r, 4.0, 4.0])
        self.t = self.t_meas = t
        self._rejects = 0
        return True

    def _predict(self, dt: float):
        if dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + g @ g.T * self.q

    def update(self, z, t: float) -> bool:
        """Return whether this measurement was accepted (not just predicted through)."""
        z = np.asarray(z, float)
        if self.x is None:
            return self._reset(z, t)
        self._predict(t - self.t)
        self.t = t
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1
        y = z - H @ self.x
        S = H @ self.P @ H.T + np.eye(2) * self.r
        if y @ np.linalg.solve(S, y) > self.gate:
            # Outlier, or the ball really was kicked/teleported: believe it after 3 in a row.
            self._rejects += 1
            if self._rejects >= 3:
                return self._reset(z, t)
            return False
        self._rejects = 0
        Kg = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + Kg @ y
        self.P = (np.eye(4) - Kg @ H) @ self.P
        self.t_meas = t
        return True

    def predict_to(self, t: float) -> np.ndarray:
        """State extrapolated to time t without modifying the filter."""
        dt = max(0.0, t - self.t)
        x = self.x.copy()
        x[:2] += x[2:] * dt
        return x


class YawFilter:
    def __init__(self, alpha: float = 0.6, rate_alpha: float = 0.3):
        self.alpha, self.rate_alpha = alpha, rate_alpha
        self.yaw: float | None = None
        self.rate = 0.0
        self.t = 0.0

    def update(self, yaw: float, t: float) -> bool:
        if self.yaw is None:
            self.yaw, self.t = yaw, t
            return True
        dt = t - self.t
        if dt <= 0:
            return False
        new = wrap(self.yaw + self.alpha * wrap(yaw - self.yaw))
        self.rate += self.rate_alpha * (wrap(new - self.yaw) / dt - self.rate)
        self.yaw, self.t = new, t
        return True
