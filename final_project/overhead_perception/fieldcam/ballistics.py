"""Where a rolling ball is going: a velocity estimate that is good enough to predict with,
and the path the ball will follow until it stops, bounces, goes out or goes in a goal.

The physics is deliberately the simplest that fits a ball rolling on a floor:

    it slows down at a constant rate `decel` (m/s^2) along a straight line,

so from speed v it rolls for v / decel seconds and covers v^2 / (2 * decel) metres.
Rolling resistance is close to a constant force, which gives exactly this. Air
drag, spin and a bumpy floor are ignored. `decel` depends on the ball and the
floor and has to be MEASURED (scripts/fit_ball_roll.py); the default is a guess.

Two parts:

  RollTracker   position measurements in -> position, velocity and "is it moving" out.
                The pipeline's Kalman filter answers "where is the ball now" well, but
                its velocity is too noisy to extrapolate seconds ahead (measured: a still
                ball reads as 0.1-0.2 m/s). This fits a straight line through the last
                ~0.6 s of measurements instead and corrects it with the roll model.
  predict_roll  position + velocity in -> Trajectory: where the ball is at any later
                time, where and when it ends, and how (stops / goal / out).

Everything is in the field frame (metres, origin at the field centre).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class RollModel:
    decel: float = 0.35         # m/s^2, how fast a rolling ball slows down. A GUESS until measured.
    min_speed: float = 0.10     # m/s, below this the ball is treated as standing still
    walls: bool = False         # True: the field edge is a board the ball bounces off; False: it rolls out
    restitution: float = 0.4    # fraction of the speed into a wall that comes back out
    horizon_s: float = 8.0      # never predict further ahead than this
    window_s: float = 0.6       # the velocity fit uses measurements this recent
    meas_sigma: float = 0.01    # m, noise of one ball position measurement
    jump: float = 0.06          # m, a measurement this far from the fit means the ball was kicked or bounced

    @classmethod
    def from_config(cls, cfg: dict) -> "RollModel":
        """Settings from the optional `prediction:` section of the config; anything missing keeps its default."""
        m = cls(meas_sigma=float((cfg.get("tracking") or {}).get("meas_sigma", 0.01)))
        for key, value in (cfg.get("prediction") or {}).items():
            name = "decel" if key == "roll_decel" else key      # the config spells out what is decelerating
            if name == "meas_sigma" or not hasattr(m, name):
                raise ValueError(f"config prediction: unknown setting {key!r}")
            setattr(m, name, bool(value) if name == "walls" else float(value))
        return m


def roll_distance(speed: float, t: float, decel: float) -> float:
    """Metres covered in t seconds by a ball that starts at `speed` and slows at `decel`."""
    if decel > 0:
        t = min(t, speed / decel)
    return speed * t - 0.5 * decel * t * t


def time_to_roll(speed: float, distance: float, decel: float) -> float | None:
    """Seconds to cover `distance`, or None if the ball stops first."""
    if distance <= 0:
        return 0.0
    if decel <= 0:
        return distance / speed if speed > 0 else None
    disc = speed * speed - 2.0 * decel * distance
    return None if disc < 0 else (speed - math.sqrt(disc)) / decel


class Trajectory:
    """A predicted path: straight, slowing pieces joined at bounces.

    ends:   "still" (was not moving), "stops", "goal_east", "goal_west", "out" or "horizon"
    end:    (x, y) where that happens          end_in_s: seconds from now
    """

    def __init__(self, pos, decel: float):
        self.decel = decel
        self.pieces: list[tuple[float, np.ndarray, np.ndarray, float, float]] = []  # t0, p0, unit dir, speed0, t1
        self.end = np.asarray(pos, float).copy()
        self.end_in_s = 0.0
        self.ends = "still"
        self.bounces = 0

    @property
    def moving(self) -> bool:
        return bool(self.pieces)

    def at(self, t: float) -> tuple[float, float, float]:
        """(x, y, speed) t seconds from now. After the end it stays at the end point with speed 0."""
        if not self.pieces or t >= self.end_in_s:
            return float(self.end[0]), float(self.end[1]), 0.0
        t = max(t, 0.0)
        for t0, p0, u, s0, t1 in self.pieces:
            if t <= t1:
                tau = t - t0
                p = p0 + u * roll_distance(s0, tau, self.decel)
                return float(p[0]), float(p[1]), max(s0 - self.decel * tau, 0.0)
        return float(self.end[0]), float(self.end[1]), 0.0

    def samples(self, step: float = 0.2) -> list[list[float]]:
        """[[t, x, y], ...] along the path, including every bounce and the end."""
        if not self.pieces:
            return []
        times = set(np.arange(0.0, self.end_in_s, step).round(3).tolist()) | {round(self.end_in_s, 3)}
        times |= {round(piece[4], 3) for piece in self.pieces}
        return [[t, round(self.at(t)[0], 3), round(self.at(t)[1], 3)] for t in sorted(times)]   # mm is plenty on the wire

    def as_dict(self, pos, vel, sigma: float | None = None, n: int | None = None) -> dict:
        speed = float(math.hypot(*vel))
        out = {
            "moving": self.moving, "x": float(pos[0]), "y": float(pos[1]),
            "vx": float(vel[0]), "vy": float(vel[1]), "speed": speed,
            "heading": float(math.atan2(vel[1], vel[0])) if self.moving else None,
            "decel": self.decel, "ends": self.ends, "end": [float(self.end[0]), float(self.end[1])],
            "end_in_s": float(self.end_in_s), "bounces": self.bounces, "path": self.samples(),
        }
        if sigma is not None:
            out["speed_sigma"] = float(sigma)
        if n is not None:
            out["samples"] = int(n)
        return out


def predict_roll(pos, vel, length: float, width: float, goal_width: float, model: RollModel = RollModel(),
                 radius: float = 0.0) -> Trajectory:
    """The ball's path from position `pos` and velocity `vel` (field frame) on a length x width field.

    A ball slower than model.min_speed is "still". Otherwise it rolls straight and slows until it
    stops, crosses a goal line between the posts ("goal_east" / "goal_west"), or reaches the edge
    of the field: with model.walls it bounces (its edge touches the board), without it is "out"
    (its centre crosses the line).
    """
    p = np.asarray(pos, float).copy()
    v = np.asarray(vel, float)
    speed = float(np.linalg.norm(v))
    traj = Trajectory(p, model.decel)
    if not np.isfinite(speed) or speed < model.min_speed:
        return traj
    hl, hw, hg = length / 2.0, width / 2.0, goal_width / 2.0
    inset = radius if model.walls else 0.0     # a wall stops the ball's edge, a line counts its centre
    u, t = v / speed, 0.0
    for _ in range(12):
        stop_dist = speed * speed / (2 * model.decel) if model.decel > 0 else float("inf")
        # Distance along the current direction to each edge that lies ahead.
        hit, hit_axis = stop_dist, None
        for axis, half in ((0, hl - inset), (1, hw - inset)):
            if abs(u[axis]) < 1e-9:
                continue
            bound = math.copysign(half, u[axis])
            d = max((bound - p[axis]) / u[axis], 0.0)
            if d < hit:
                hit, hit_axis = d, axis
        if hit_axis == 0 and abs(p[1] + u[1] * hit) < hg:
            # Heading into a goal mouth: no wall here, carry on to the goal line itself.
            d_goal = max((math.copysign(hl, u[0]) - p[0]) / u[0], 0.0)
            y_goal = p[1] + u[1] * d_goal
            if d_goal <= stop_dist and abs(y_goal) < hg:
                hit, hit_axis = d_goal, "goal"
            elif d_goal > stop_dist:
                hit, hit_axis = stop_dist, None            # stops in the mouth, short of the line
        dt = time_to_roll(speed, hit, model.decel)
        if dt is None:                                     # numerical edge: treat as stopping
            hit_axis, hit, dt = None, stop_dist, speed / model.decel
        if t + dt > model.horizon_s:
            dt = model.horizon_s - t
            traj.pieces.append((t, p.copy(), u.copy(), speed, model.horizon_s))
            traj.end, traj.end_in_s, traj.ends = p + u * roll_distance(speed, dt, model.decel), model.horizon_s, "horizon"
            return traj
        traj.pieces.append((t, p.copy(), u.copy(), speed, t + dt))
        p = p + u * hit
        speed = max(speed - model.decel * dt, 0.0)
        t += dt
        traj.end, traj.end_in_s = p.copy(), t
        if hit_axis is None:
            traj.ends = "stops"
            return traj
        if hit_axis == "goal":
            traj.ends = "goal_east" if p[0] > 0 else "goal_west"
            return traj
        if not model.walls:
            traj.ends = "out"
            return traj
        # Bounce: the speed into the wall comes back reduced, the speed along it is kept.
        vel_now = u * speed
        vel_now[hit_axis] *= -model.restitution
        speed = float(np.linalg.norm(vel_now))
        traj.bounces += 1
        if speed < 1e-6:
            traj.ends = "stops"
            return traj
        u = vel_now / speed
    traj.ends = "stops"
    return traj


class RollTracker:
    """Ball position measurements in, a velocity you can predict with out.

    update(xy, t) with every ball measurement (t = capture time). estimate(t_now) gives
    {"x", "y", "vx", "vy", "speed", "speed_sigma", "moving", "samples"} at t_now, or None
    before there are enough measurements.

    How: a straight line fitted through the measurements of the last `window_s` gives the
    average velocity over that window, which for a steadily slowing ball is its velocity at
    the middle of the window; the roll model then brings it forward to now. A measurement
    far from the fit means the ball was kicked or bounced: if the next one agrees, the old
    history is dropped so the new motion shows up within two frames. A single stray
    detection is ignored.
    """

    def __init__(self, model: RollModel = RollModel()):
        self.model = model
        self.reset()

    def reset(self):
        self.hist: deque = deque()         # (t, x, y)
        self._pending = None               # one measurement that did not fit, waiting for a second opinion
        self._moving = False

    def _fit(self, upto: float | None = None):
        """Least-squares line through the window: (t_mid, position at t_mid, velocity, sigma of each velocity axis)."""
        pts = [h for h in self.hist if upto is None or h[0] <= upto]
        if len(pts) < 2:
            return None
        T = np.array([h[0] for h in pts])
        Z = np.array([[h[1], h[2]] for h in pts])
        t_mid = float(T.mean())
        spread = float(((T - t_mid) ** 2).sum())
        if spread < 1e-9:
            return None
        vel = ((T - t_mid)[:, None] * (Z - Z.mean(axis=0))).sum(axis=0) / spread
        return t_mid, Z.mean(axis=0), vel, self.model.meas_sigma / math.sqrt(spread)

    def _state_at(self, t: float):
        """Position and velocity at time t from the current fit, slowed by the roll model."""
        fit = self._fit()
        if fit is None:
            return None
        t_mid, pos_mid, vel_mid, sigma = fit
        dt = t - t_mid
        speed_mid = float(np.linalg.norm(vel_mid))
        if speed_mid < 1e-9:
            return pos_mid, np.zeros(2), sigma
        u = vel_mid / speed_mid
        # The line's midpoint sits slightly behind a slowing ball's true position; the 1/2 a t^2
        # term is centimetres at most and is left to the fit.
        pos = pos_mid + u * roll_distance(speed_mid, max(dt, 0.0), self.model.decel)
        speed = max(speed_mid - self.model.decel * max(dt, 0.0), 0.0)
        return pos, u * speed, sigma

    def update(self, xy, t: float) -> None:
        z = (float(t), float(xy[0]), float(xy[1]))
        if self.hist and t <= self.hist[-1][0]:
            return                                          # repeated or out-of-order frame
        if len(self.hist) >= 3:
            expect = self._state_at(t)
            off = math.hypot(z[1] - expect[0][0], z[2] - expect[0][1]) if expect else 0.0
            if off > self.model.jump:
                if self._pending is not None:
                    # Two in a row away from the old motion: it is real. Start again from these two.
                    self.hist = deque([self._pending, z])
                    self._pending = None
                else:
                    self._pending = z
                return
        self._pending = None
        self.hist.append(z)
        while self.hist and self.hist[0][0] < t - self.model.window_s and len(self.hist) > 2:
            self.hist.popleft()

    def estimate(self, t_now: float) -> dict | None:
        if len(self.hist) < 2 or t_now - self.hist[-1][0] > 2 * self.model.window_s:
            return None
        pos, vel, sigma = self._state_at(t_now)
        speed = float(np.linalg.norm(vel))
        # Moving only if the speed stands clear of both the floor and the noise; a little
        # hysteresis so it does not flicker around the threshold.
        threshold = max(self.model.min_speed, 3.0 * sigma) * (0.7 if self._moving else 1.0)
        self._moving = speed > threshold
        if not self._moving:
            # Standing still: the best position is the plain average of the window.
            pos, vel = np.array([[h[1], h[2]] for h in self.hist]).mean(axis=0), np.zeros(2)
        return {"x": float(pos[0]), "y": float(pos[1]), "vx": float(vel[0]), "vy": float(vel[1]),
                "speed": float(np.linalg.norm(vel)), "speed_sigma": float(sigma), "moving": self._moving,
                "samples": len(self.hist)}


def fit_decel(samples, min_duration: float = 0.6, min_points: int = 6) -> dict | None:
    """Measure the roll deceleration from one roll: samples = [(t, x, y), ...] while the ball slows freely.

    Fits distance-along-the-path = v0 * t - 1/2 * decel * t^2. Returns
    {"decel", "v0", "duration", "distance", "rms_m", "points"}, or None if the roll is too
    short to say anything.
    """
    pts = sorted(samples)
    if len(pts) < min_points or pts[-1][0] - pts[0][0] < min_duration:
        return None
    T = np.array([p[0] for p in pts]) - pts[0][0]
    Z = np.array([[p[1], p[2]] for p in pts])
    d = Z[-1] - Z[0]
    length = float(np.linalg.norm(d))
    if length < 0.05:
        return None
    s = (Z - Z[0]) @ (d / length)                           # distance along the roll direction
    A = np.vstack([np.ones_like(T), T, -0.5 * T * T]).T
    (s0, v0, decel), *_ = np.linalg.lstsq(A, s, rcond=None)
    rms = float(np.sqrt(np.mean((A @ np.array([s0, v0, decel]) - s) ** 2)))
    return {"decel": float(decel), "v0": float(v0), "duration": float(T[-1]), "distance": length,
            "rms_m": rms, "points": len(pts)}
