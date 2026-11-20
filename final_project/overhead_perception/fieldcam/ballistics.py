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
from dataclasses import dataclass, replace

import numpy as np


@dataclass
class RollModel:
    decel: float = 0.35         # m/s^2, how fast a rolling ball slows down: the starting guess for each roll
    decel_spread: float = 0.20  # m/s^2, how far one roll's own slow-down may sit from that guess
    area_floor: float = 0.03    # m, the stopping area is never claimed to be smaller than this
    wander_deg: float = 4.0     # degrees, how far off its line a roll may drift (divots, seams); widens the area
    coast_s: float = 1.5        # s, keep predicting a rolling ball this long after it was last seen (hidden by a robot)
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


def _fit_roll(pts, decel0: float, decel_sigma: float, meas_sigma: float, hold: float | None = None) -> dict | None:
    """One roll so far -> its own start speed and slow-down rate, with how sure that is.

    pts = [(t, x, y), ...] since the roll began. The ball's distance along its line is fitted
    with  s0 + v0 * t - 1/2 * decel * t^2.  Early in a roll the curve is too short to show the
    slow-down, so `decel` is held near decel0 (within decel_sigma); as measurements come in
    the roll's own data takes over. `hold` pins it tighter than that for the estimate while
    the reported uncertainty still uses decel_sigma. Returns None when it is not (yet) a roll.
    """
    n = len(pts)
    if n < 3:
        return None
    T = np.array([q[0] for q in pts])
    Z = np.array([[q[1], q[2]] for q in pts])
    centre = Z.mean(axis=0)
    u = np.linalg.svd(Z - centre)[2][0]                    # the line the ball is rolling along
    if (Z[-1] - Z[0]) @ u < 0:
        u = -u
    along = (Z - centre) @ u
    across = (Z - centre) @ np.array([-u[1], u[0]])
    if along.max() - along.min() < 4 * meas_sigma:
        return None
    sigma = max(meas_sigma, float(np.sqrt((across ** 2).sum() / max(n - 2, 1))))   # noise, from the wobble off the line
    tau = T - T[0]
    A = np.vstack([np.ones(n), tau, -0.5 * tau * tau]).T
    normal = A.T @ A / sigma ** 2
    prior = np.diag([0.0, 0.0, 1.0 / decel_sigma ** 2])
    cov = np.linalg.inv(normal + prior)
    held = prior if hold is None else np.diag([0.0, 0.0, 1.0 / hold ** 2])
    s0, v0, decel = np.linalg.solve(normal + held, A.T @ along / sigma ** 2 + held @ np.array([0.0, 0.0, decel0]))
    fit = {"t0": float(T[0]), "centre": centre, "u": u, "s0": float(s0), "v0": float(v0),
           # 1 / decel sets the distance, so a fit that drifts toward "never slows" would send the
           # prediction far away: keep it within two spreads of the starting guess.
           "decel": float(min(max(decel, decel0 - 2.0 * decel_sigma, 0.08), decel0 + 3.0 * decel_sigma, 3.0)),
           "cov": cov, "sigma": sigma, "n": n,
           "along_mean": float(along.mean()),
           "dir_sigma": sigma / math.sqrt(float(((along - along.mean()) ** 2).sum())),
           "decel_data": None, "decel_data_sigma": None}
    if n >= 5:                                              # what the data alone says, for spotting a push
        try:
            cov_data = np.linalg.inv(normal)
            fit["decel_data"] = float((cov_data @ (A.T @ along / sigma ** 2))[2])
            fit["decel_data_sigma"] = float(math.sqrt(max(cov_data[2, 2], 0.0)))
        except np.linalg.LinAlgError:
            pass
    return fit


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
    """Ball position measurements in; where the ball is, how it is moving and where it will stop out.

    update(xy, t) with every ball measurement (t = capture time). Then either
      estimate(t_now)   {"x", "y", "vx", "vy", "speed", "speed_sigma", "moving", "samples"}, or
      predict(t_now, length, width, goal_width, radius)   (prediction dict, Trajectory)
    Both give None before there are enough measurements.

    Is it moving: a straight line through the measurements of the last `window_s` gives the
    ball's velocity. A measurement far from that line means a kick or a bounce: if the next
    one agrees, the old history is dropped so the new motion shows within two frames. A
    single stray detection is ignored.

    Where will it stop: every roll is fitted on its own, from its first frames, and refitted
    on every frame after. Each fit gives that roll's start speed AND its slow-down rate, so
    a ball that slows differently from roll to roll (carpet grain, how it was released) is
    still predicted, and the prediction comes with a stopping AREA that starts large and
    shrinks as the roll goes on. The slow-down rates of finished rolls become the starting
    guess for the next one.
    """

    def __init__(self, model: RollModel = RollModel()):
        self.model = model
        self.learned: list[float] = []     # slow-down rates measured on finished rolls (newest last)
        self.turns: list[float] = []       # how far each finished roll turned off its first line, rad (0 = straight)
        self.noise = model.meas_sigma      # m, measurement noise: starts at the configured value, then measured
        self.reset()                       #    from how much a ball that is standing still appears to jitter

    def reset(self):
        self.hist: deque = deque()         # (t, x, y) of the last window_s: the velocity fit
        self.all: list = []                # the same measurements, kept longer: the roll fit
        self._pending = None               # one measurement that did not fit, waiting for a second opinion
        self._moving = False
        self._still_pos = None             # where the ball last stood still
        self._roll_t0 = None               # when the current roll began
        self._restart = False              # a kick or bounce happened: the next estimate starts a new roll
        self.trail: list = []              # (t, x, y): where each frame of this roll said the ball would stop
        self._carry = None                 # (slow-down, spread) carried over a bend or bounce within one roll
        self._u0 = None                    # the direction this roll had before it was first knocked off course
        self._roll_turn = 0.0              # how far (rad) it has turned from that since
        self._knocks = 0                   # times this roll was knocked off course
        self._knock_t = None               # when it was last knocked (or kicked)
        self._phase = "still"
        self._still_since = None           # when it last came to rest (noise is only learned once the window is all still)
        self.last_roll: dict | None = None
        self._last_decel = None

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
        # How sure is that velocity? Judge by how far the measurements actually sit from the line,
        # not only by the noise assumed so far: a ball that is standing still but detected a few
        # centimetres apart from frame to frame must not read as a ball that is moving.
        scatter = 0.0
        if len(pts) >= 3:
            off = Z - Z.mean(axis=0) - (T - t_mid)[:, None] * vel
            scatter = math.sqrt(float((off ** 2).sum()) / (2 * (len(pts) - 2)))
        return t_mid, Z.mean(axis=0), vel, max(self.noise, scatter) / math.sqrt(spread)

    def decel_prior(self) -> tuple[float, float]:
        """(starting guess for a roll's slow-down rate, how far a roll may sit from it)."""
        if self._carry is not None:
            return self._carry
        if len(self.learned) < 2:
            return self.model.decel, self.model.decel_spread
        recent = np.array(self.learned[-9:])
        middle = float(np.median(recent))
        spread = (max(1.4826 * float(np.median(np.abs(recent - middle))), float(recent.std()))
                  if len(recent) >= 3 else self.model.decel_spread)
        return middle, min(max(spread, 0.08), 0.35)

    def _state_at(self, t: float):
        """Position and velocity at time t from the window fit, slowed by the roll model."""
        fit = self._fit()
        if fit is None:
            return None
        t_mid, pos_mid, vel_mid, sigma = fit
        dt = t - t_mid
        speed_mid = float(np.linalg.norm(vel_mid))
        if speed_mid < 1e-9:
            return pos_mid, np.zeros(2), sigma
        u = vel_mid / speed_mid
        decel = self.decel_prior()[0]
        # The line's midpoint sits slightly behind a slowing ball's true position; the 1/2 a t^2
        # term is centimetres at most and is left to the fit.
        pos = pos_mid + u * roll_distance(speed_mid, max(dt, 0.0), decel)
        speed = max(speed_mid - decel * max(dt, 0.0), 0.0)
        return pos, u * speed, sigma

    def update(self, xy, t: float) -> None:
        z = (float(t), float(xy[0]), float(xy[1]))
        if not all(math.isfinite(v) for v in z):
            return                                          # a broken measurement must not poison the fit
        if self.hist and t <= self.hist[-1][0]:
            return                                          # repeated or out-of-order frame
        if self.hist and t - self.hist[-1][0] > self.model.coast_s:
            self.reset()                                    # not seen for too long to be the same motion
        if len(self.hist) >= 3:
            expect = self._state_at(t)
            off = math.hypot(z[1] - expect[0][0], z[2] - expect[0][1]) if expect else 0.0
            if off > max(self.model.jump, 4.0 * self.noise):
                if self._pending is not None:
                    # Two in a row away from the old motion: it is real. Start again from these two.
                    self.hist = deque([self._pending, z])
                    self.all = [self._pending, z]
                    self._pending, self._restart, self._knock_t = None, True, z[0]
                else:
                    self._pending = z
                return
        self._pending = None
        self.hist.append(z)
        self.all.append(z)
        while self.hist and self.hist[0][0] < t - self.model.window_s and len(self.hist) > 2:
            self.hist.popleft()
        if len(self.all) > 600:
            del self.all[:200]

    def _end_roll(self, t_now: float, summary: bool):
        """The roll is over (the ball stopped, or was kicked again): remember what it taught."""
        pts = [q for q in self.all if self._roll_t0 is not None and q[0] >= self._roll_t0]
        measured = fit_decel(pts, min_duration=0.8) if len(pts) > 7 else None
        if measured and 0.03 < measured["decel"] < 3.0:
            self.learned.append(measured["decel"])
            del self.learned[:-20]
        self.last_roll = ({"t": t_now, "trail": list(self.trail), "decel": self._last_decel,
                           "decel_measured": measured["decel"] if measured else None} if summary and self.trail else None)
        if summary:
            self.turns.append(self._roll_turn)
            del self.turns[:-10]
        self._roll_t0, self.trail, self._carry, self._u0, self._roll_turn = None, [], None, None, 0.0
        self._knocks = 0

    def estimate(self, t_now: float) -> dict | None:
        if len(self.hist) < 2:
            return None
        # A rolling ball hidden for a moment (a robot walks in front of it) is still predicted for
        # coast_s; a ball that was standing still is dropped sooner, since it may have been picked up.
        if t_now - self.hist[-1][0] > (self.model.coast_s if self._moving else 2 * self.model.window_s):
            return None
        pos, vel, sigma = self._state_at(t_now)
        speed = float(np.linalg.norm(vel))
        # Moving only if the speed stands clear of both the floor and the noise; a little
        # hysteresis so it does not flicker around the threshold.
        # With only three or four measurements the scatter about the line says little, so ask for more.
        sure = {2: 3.0, 3: 6.0, 4: 4.5}.get(len(self.hist), 3.0)
        threshold = max(self.model.min_speed, sure * sigma) * (0.7 if self._moving else 1.0)
        was_moving, self._moving = self._moving, speed > threshold
        if not was_moving and self._moving:
            # To START counting as moving, the ball must also have actually gone somewhere: a ball that
            # is standing still but detected a few centimetres apart (or in only some frames) has not.
            a, b = self.hist[0], self.hist[-1]
            step = max(self.model.jump, 4.0 * self.noise) if len(self.hist) < 3 else max(0.04, 4.0 * self.noise)
            self._moving = math.hypot(b[1] - a[1], b[2] - a[2]) > step
        if self._restart and was_moving and self._moving and self._roll_t0 is not None:
            # Knocked off course mid-roll (a bounce, a divot, a touch): same ball, same floor. Fit the
            # new direction from here, keep what this roll has shown about how fast it slows.
            if self._last_decel is not None:
                self._carry = (self._last_decel, max(self.decel_prior()[1] * 0.5, 0.05))
            self._roll_t0 = self.hist[0][0]
            self._knocks += 1
        elif self._restart and was_moving:
            self._end_roll(t_now, summary=False)
        if self._moving and self._roll_t0 is None:
            # A roll begins: with the first measurement that has left the spot where the ball stood.
            start = self.hist[0][0]
            if self._still_pos is not None and not self._restart:
                away = [h[0] for h in self.hist
                        if math.hypot(h[1] - self._still_pos[0], h[2] - self._still_pos[1]) > 3 * self.noise]
                start = away[0] if away else self.hist[-1][0]
            self._roll_t0, self.trail, self.last_roll = start, [], None
            self._knock_t = start if self._knock_t is None or self._knock_t < start else self._knock_t
        elif not self._moving and was_moving:
            self._end_roll(t_now, summary=True)
        self._restart = False
        if not self._moving:
            # Standing still: the best position is the plain average of the window.
            window = np.array([[h[1], h[2]] for h in self.hist])
            pos, vel = window.mean(axis=0), np.zeros(2)
            self._still_pos = (float(pos[0]), float(pos[1]))
            self._still_since = t_now if (was_moving or self._still_since is None) else self._still_since
            settled = np.array([[q[1], q[2]] for q in self.all
                                if q[0] >= max(self._still_since + self.model.window_s, t_now - 2.0)])
            if len(settled) >= 4:
                # Standing still: its apparent jitter IS the measurement noise. Follow it slowly, within
                # sane bounds, so every threshold suits this ball, these lights and this camera height.
                # (Taken over the last two seconds, so a ball found in only some frames still counts.)
                jitter = float(np.sqrt(((settled - settled.mean(axis=0)) ** 2).sum() / (2 * (len(settled) - 1))))
                self.noise = min(max(0.95 * self.noise + 0.05 * jitter, 0.003), 0.05)
        return {"x": float(pos[0]), "y": float(pos[1]), "vx": float(vel[0]), "vy": float(vel[1]),
                "speed": float(np.linalg.norm(vel)), "speed_sigma": float(sigma), "moving": self._moving,
                "samples": len(self.hist)}

    def wander(self) -> float:
        """How far off its line a roll may still drift, rad: the configured figure, widened by what
        recent rolls actually did (a field with divots teaches this within a few rolls)."""
        seen = float(np.mean(np.square(self.turns))) if self.turns else 0.0
        return min(math.sqrt(math.radians(self.model.wander_deg) ** 2 + seen), math.radians(35.0))

    def _roll_fit(self) -> dict | None:
        """The current roll fitted on its own. Measurements from before the ball was let go are dropped."""
        if self._roll_t0 is None:
            return None
        pts = [q for q in self.all if q[0] >= self._roll_t0]
        # Off course? A divot or seam can turn the ball a few degrees without any jump. If the two
        # newest frames both sit clear of the line through the earlier ones, on the same side, the
        # ball has turned: fit the new direction from the last frame that was still on the line.
        if len(pts) >= 6:
            old = np.array([[q[1], q[2]] for q in pts[:-2]])
            centre = old.mean(axis=0)
            u = np.linalg.svd(old - centre)[2][0]
            off = (np.array([[q[1], q[2]] for q in pts[-2:]]) - centre) @ np.array([-u[1], u[0]])
            wobble = float(np.sqrt((((old - centre) @ np.array([-u[1], u[0]])) ** 2).mean()))
            limit = max(3.0 * max(self.noise, wobble), 0.03)
            if off[0] * off[1] > 0 and min(abs(off[0]), abs(off[1])) > limit:
                if self._last_decel is not None:
                    self._carry = (self._last_decel, max(self.decel_prior()[1] * 0.5, 0.05))
                if self._u0 is None:
                    self._u0 = u if (old[-1] - old[0]) @ u >= 0 else -u
                pts = pts[-3:]
                self._roll_t0, self._knock_t = pts[0][0], pts[0][0]
                self._knocks += 1
        decel0, decel_sigma = self.decel_prior()
        # The roll proper starts where the ball was fastest: before that it was still being pushed,
        # and those frames would make it look as if it slows faster than it does. Speeds are taken
        # across two frames; the start only moves if a later speed clearly beats the first one.
        if len(pts) >= 6:
            speeds = [math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0]) for a, b in zip(pts, pts[2:])]
            fastest = int(np.argmax(speeds[:len(pts) - 4]))
            noise = 3.0 * math.sqrt(2.0) * self.noise / (pts[fastest + 2][0] - pts[fastest][0])
            if fastest > 0 and speeds[fastest] > speeds[0] + noise:
                pts = pts[fastest:]
                self._roll_t0 = pts[0][0]
        if pts and pts[-1][0] - pts[0][0] < 0.6:
            # Too early to see this roll's own slow-down, and its first frames may still be the push:
            # go by the newest frames only and keep the slow-down at the starting guess.
            recent = [q for q in pts if q[0] >= pts[-1][0] - 0.35]
            self._phase = "early"
            return _fit_roll(recent if len(recent) >= 3 else pts[-3:], decel0, decel_sigma, self.noise, hold=0.02)
        self._phase = "own_rate"
        while True:
            fit = _fit_roll(pts, decel0, decel_sigma, self.noise)
            if fit is None or len(pts) < 6 or fit["decel_data"] is None:
                return fit
            # A ball that is clearly still SPEEDING UP is still being pushed: the roll starts later.
            if fit["decel_data"] < -max(0.05, 2.0 * fit["decel_data_sigma"]):
                pts = pts[1:]
                self._roll_t0 = pts[0][0]
                continue
            return fit

    def _speeding_up(self) -> bool:
        """Is the ball clearly faster in the newer half of the window than the older half (being pushed or hit)?"""
        pts = list(self.hist)
        if len(pts) < 4:
            return False
        half = len(pts) // 2
        old, new = pts[:half + 1], pts[half:]
        speed = lambda q: math.hypot(q[-1][1] - q[0][1], q[-1][2] - q[0][2]) / max(q[-1][0] - q[0][0], 1e-6)  # noqa: E731
        sigma = math.sqrt(2.0) * self.noise / max(min(old[-1][0] - old[0][0], new[-1][0] - new[0][0]), 1e-6)
        # Well clear of the noise AND a real gain in speed: a rolling ball's measured speed wobbles by itself.
        return speed(new) - speed(old) > max(6.0 * sigma, 0.15)

    def _guess_from(self) -> str:
        if self._carry is not None:
            return "this roll, before it was knocked off course"
        return f"{len(self.learned)} finished rolls" if len(self.learned) >= 2 else "the config"

    def predict(self, t_now: float, length: float, width: float, goal_width: float, radius: float = 0.0):
        """(prediction dict, Trajectory) for a field of this size, or None before there is enough to go on.

        On top of Trajectory.as_dict the dict has
          decel, decel_sigma   this roll's slow-down rate so far, and how sure that is
          area                 None, or where the ball will stop as an ellipse: {"x", "y", "along",
                               "across", "heading"}, half-axes in metres at about 2 sigma
          trail                [[age_s, x, y], ...]: what each earlier frame of this roll predicted
          last_roll            for a few seconds after a roll: {"ago_s", "stopped", "trail",
                               "first_guess_error", "decel"}
          seen_ago_s, hidden   how long since the ball was last measured; hidden = predicting without seeing it
          fit                  how the numbers were arrived at, for the dashboard and the state machines:
                               {"phase": "still" | "starting" | "early" | "own_rate", "roll_s", "roll_frames",
                                "noise", "guess", "guess_spread", "guess_from", "speeding_up", "knocks",
                                "knock_ago_s", "turned_deg", "wander_deg"}
        """
        est = self.estimate(t_now)
        if est is None:
            return None
        pos, vel = (est["x"], est["y"]), (est["vx"], est["vy"])
        decel, decel_sigma = self.decel_prior()
        guess, guess_spread, guess_from = decel, decel_sigma, self._guess_from()
        along = across = None
        self._phase = "still"
        frames = 0
        if est["moving"]:
            self._phase = "starting"
            fit = self._roll_fit()
            frames = fit["n"] if fit else 0
            if fit is not None and fit["v0"] > 0:
                decel, v0, u = fit["decel"], fit["v0"], fit["u"]
                tau = min(max(t_now - fit["t0"], 0.0), v0 / decel)
                s_now = fit["s0"] + v0 * tau - 0.5 * decel * tau * tau
                s_stop = fit["s0"] + v0 * v0 / (2 * decel)
                pos = tuple(fit["centre"] + u * s_now)
                vel = tuple(u * max(v0 - decel * tau, 0.0))
                jac = np.array([1.0, v0 / decel, -v0 * v0 / (2 * decel * decel)])
                along = math.sqrt(max(float(jac @ fit["cov"] @ jac), 0.0))
                across = math.hypot(fit["sigma"] / math.sqrt(fit["n"]), fit["dir_sigma"] * (s_stop - fit["along_mean"]))
                across = math.hypot(across, max(s_stop - s_now, 0.0) * math.tan(self.wander()))   # it may yet be knocked off line
                decel_sigma = math.sqrt(max(float(fit["cov"][2, 2]), 0.0))
                if self._u0 is not None:
                    self._roll_turn = max(self._roll_turn, abs(math.atan2(self._u0[0] * u[1] - self._u0[1] * u[0], self._u0 @ u)))
            else:
                # Only a frame or two of this roll so far: the window's velocity and the starting guess.
                left = est["speed"] ** 2 / (2 * decel)
                along = math.hypot(est["speed"] / decel * est["speed_sigma"], left / decel * decel_sigma)
                across = math.hypot(left * est["speed_sigma"] / max(est["speed"], 1e-6), left * math.tan(self.wander()))
        self._last_decel = decel
        traj = predict_roll(pos, vel, length, width, goal_width, replace(self.model, decel=decel), radius)
        out = traj.as_dict(pos, vel, est["speed_sigma"], est["samples"])
        seen_ago = t_now - self.hist[-1][0]
        out.update(decel_sigma=float(decel_sigma), area=None, trail=[], last_roll=None,
                   seen_ago_s=float(seen_ago), hidden=bool(seen_ago > 0.25),
                   fit={"phase": self._phase if traj.moving else "still",
                        "roll_s": float(t_now - self._roll_t0) if self._roll_t0 is not None else 0.0,
                        "roll_frames": int(frames), "noise": float(self.noise),
                        "guess": float(guess), "guess_spread": float(guess_spread), "guess_from": guess_from,
                        "speeding_up": bool(est["moving"] and self._speeding_up()), "knocks": int(self._knocks),
                        "knock_ago_s": None if self._knock_t is None else float(t_now - self._knock_t),
                        "turned_deg": float(math.degrees(self._roll_turn)),
                        "wander_deg": float(math.degrees(self.wander()))})
        if traj.moving and traj.ends == "stops" and along is not None:
            direction = traj.pieces[-1][2]
            floor = self.model.area_floor
            out["area"] = {"x": out["end"][0], "y": out["end"][1], "heading": float(math.atan2(direction[1], direction[0])),
                           "along": float(math.hypot(2 * along, floor)), "across": float(math.hypot(2 * across, floor))}
            self.trail.append((t_now, out["end"][0], out["end"][1]))
            del self.trail[:-60]
        if traj.moving:
            out["trail"] = [[round(t_now - t, 3), round(x, 3), round(y, 3)] for t, x, y in self.trail]
        elif self.last_roll and t_now - self.last_roll["t"] < 6.0:
            done = self.last_roll
            first = done["trail"][0]
            out["last_roll"] = {
                "ago_s": float(t_now - done["t"]), "stopped": [est["x"], est["y"]],
                "trail": [[round(t - done["t"], 3), round(x, 3), round(y, 3)] for t, x, y in done["trail"]],
                "first_guess_error": float(math.hypot(first[1] - est["x"], first[2] - est["y"])),
                "decel": done["decel_measured"] if done["decel_measured"] is not None else done["decel"]}
        return out, traj


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
