"""Find the ball, walk to it and line one front paw up with it (the kick_ball approach).

Pure Python, no ROS. ``approach()`` drives an ``ApproachIO``: the tool server's
binding (``lab/part9_ball_kick.py``) sends walker speeds and reads ``BallVision``
fixes; the tests use a simulated walker and camera. After a successful approach
the caller plays the leg kick trick.

Phases (body frame: x forward, y left, m; yaw counter-clockwise, rad):

  FIND      stand still and look; if no ball, turn left in place in 60 deg steps
            and look again (the camera spans 220 deg), up to 360 deg of commanded
            yaw. A search after a lost ball also turns. All searches in one
            approach share a budget of 540 deg of commanded yaw.
  APPROACH  walk toward the ball and turn so that it sits at the paw's y, until
            the ball is near ``stage_far`` ahead (continuous, latency-compensated).
  STAGE     stop-and-look pulses: stand still, average the fixes, send one short
            timed command, repeat. Small commands fall in the walker's dead band,
            so a pulse is the reliable way to move 1 cm.
  CALIBRATE a short and a long forward pulse, measured by the camera, give the
            forward pulse model (below). A rehearsal pulse of approximately the
            creep length, after a back-up if necessary, must land where the model
            predicts (2 cm), else the approach stops.
  ALIGN     pulses until the ball is at (stage_near, paw y) within tolerance.
  CREEP     the ball is in the camera blind zone (0.30 to 0.45 m) from here, so
            walk the last ``x - contact_x`` blind, timed from the model and
            aimed short, then stop.

Pulse model: a pulse of T s at speed v moves gain * v * max(0, T - dead). The
dead time (walker start latency less the coast after the stop) is the same for
all axes. A line fit of all measured forward pulses gives the forward gain and
the dead time. The approach stops if the fit is outside the plausible bounds, or
if one pulse is more than 2 cm (and 3 standard errors) from the line of the
others. The creep is aimed short by 5 mm plus one standard error, and stops the
approach if its 3-sigma error does not fit the 3 cm kick tolerance. A pulse with
much less travel than expected increases the dead time. The sideways gain and
the sideways drift per m of forward travel are learned from the pulses.

Every loop checks ``io.check()`` (stop, fall, motion block) and ends with a zero
command. A stale ball estimate gives zero travel. Budgets bound time, commanded
travel and commanded search yaw.
"""

from __future__ import annotations

import dataclasses
import math
import os
import statistics
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Protocol

SIDES = ("right", "left")
BLIND_X = 0.45  # m, the camera does not see the ball nearer than this


@dataclass(frozen=True)
class KickGeometry:
    """Ball centre from base_link at the kick (leg kick handoff, 2026-11-16)."""

    contact_x: float = 0.258
    right_y: float = -0.046  # right paw: two good kicks on the robot
    left_y: float = 0.040  # left paw: sim only (+3.3 deg at 0.24 m)

    def paw_y(self, side: str) -> float:
        return self.right_y if side == "right" else self.left_y


@dataclass(frozen=True)
class ApproachParams:
    # FIND
    look_s: float = 1.2  # stand still and look before turning
    confirm_n: int = 3  # agreeing fixes before the ball counts as found
    confirm_spread: float = 0.15  # m
    search_wz: float = 0.6  # rad/s (34 deg/s; the walker barely turns below 20 deg/s)
    search_step: float = math.radians(60.0)  # commanded yaw between looks
    search_total: float = math.radians(360.0)  # commanded yaw in one search
    search_yaw_max: float = math.radians(540.0)  # commanded search yaw in one approach
    auto_deadband: float = math.radians(5.0)  # auto side: a centred ball gets the right paw
    # APPROACH (continuous)
    stage_far: float = 0.90  # m, ball x where the stop-and-look stage starts
    approach_speed: float = 0.30  # m/s
    approach_kx: float = 1.0  # 1/s
    yaw_kp: float = 1.5  # 1/s
    max_wz: float = 1.0  # rad/s
    turn_first: float = math.radians(25.0)  # larger bearing error: turn in place
    stage_heading: float = math.radians(6.0)  # bearing error at the hand-over to the pulses
    min_turn_wz: float = 0.35  # rad/s, in-place turns
    max_obs_age: float = 0.5  # s
    lost_s: float = 2.0  # s without a fresh fix: search again
    searches_max: int = 2
    # Stop-and-look pulses
    settle_s: float = 0.5
    measure_s: float = 1.5
    measure_min: int = 5
    pulse_vx: float = 0.2  # m/s, also the calibration and creep speed
    pulse_vy: float = 0.15  # m/s
    pulse_min_s: float = 0.05  # also never less than the dead time + 0.03 s
    pulse_max_s: float = 1.5  # correction pulses
    pulse_gain: float = 0.8  # fraction of the forward error one pulse corrects (sideways: all)
    stage_iters: int = 6
    align_iters: int = 8
    tol_far: float = 0.04  # m, x and y at stage_far
    stage_near: float = 0.50  # m, ball x before the creep (visible: >= 0.45 m)
    tol_x: float = 0.03  # m, at stage_near (the kick tolerates +-3 cm forward)
    tol_y: float = 0.010  # m (each 1 cm sideways turns the ball approximately 13 deg)
    tol_y_loose: float = 0.020  # m, accepted after align_iters
    # Walker model: travel = gain * speed * max(0, seconds - dead)
    efficiency: float = 0.8  # prior gain (achieved / commanded speed), both axes
    efficiency_bounds: tuple[float, float] = (0.45, 1.35)  # plausible gain; a forward fit outside stops
    dead_time: float = 0.1  # s, prior dead time
    dead_bounds: tuple[float, float] = (-0.15, 0.35)  # s, plausible; a fit outside stops
    cal_short_s: float = 0.5  # first calibration pulse
    cal_long_min_s: float = 0.9  # second calibration pulse: at least this long ...
    cal_long_max_s: float = 2.0  # ... and at most this long
    fit_min_s: float = 0.35  # shorter forward pulses stay out of the fit
    fit_resid_max: float = 0.02  # m, largest leave-one-out residual of the fit
    creep_margin: float = 0.005  # m, aim the creep short by this ...
    creep_k: float = 1.0  # ... plus this many standard errors of the creep travel
    creep_max_s: float = 3.0  # s, longest blind creep (and rehearsal)
    # calibrate 0: no far stage and no calibration pulses. The walker model is then
    # efficiency (gain) and dead_time, measured before on this robot, with fixed_sigma
    # as the creep travel uncertainty. The camera is used only near the ball (line-up).
    calibrate: int = 1
    fixed_sigma: float = 0.004  # m
    drift_max: float = 0.012  # m, largest sideways drift correction for the creep
    # Budgets
    travel_max: float = 5.0  # m, commanded travel in one approach
    timeout_s: float = 90.0
    rate_hz: float = 20.0


@dataclass(frozen=True)
class Fix:
    """Ball centre in the body frame [m]; stamp is the capture time on io.now()'s clock."""

    x: float
    y: float
    stamp: float
    precise: bool = True  # False: a coarse estimate (the colour-box fallback), not used to line up


@dataclass(frozen=True)
class Measure:
    """Mean ball position over one stand-still window; sx, sy are standard errors [m]."""

    x: float
    y: float
    sx: float
    sy: float
    n: int


@dataclass(frozen=True)
class Pulse:
    """One measured forward pulse at pulse_vx."""

    seconds: float
    dx: float  # m, ball x before minus after (the robot's forward travel)
    dy: float  # m, ball y after minus before
    sdx: float  # m, standard error of dx
    sdy: float  # m, standard error of dy
    label: str


@dataclass(frozen=True)
class WalkerFit:
    """Least-squares line travel = slope * seconds + offset over the forward pulses."""

    slope: float  # m/s (gain * speed)
    offset: float  # m (-gain * speed * dead)
    n: int
    t_mean: float
    sxx: float
    sigma: float  # m, standard error of one pulse's travel, from the fix spread
    loo: tuple[tuple[float, float], ...] = ()  # (leave-one-out residual, its standard error) [m]

    def travel(self, seconds: float) -> float:
        return self.slope * seconds + self.offset

    def seconds(self, travel: float) -> float:
        return (travel - self.offset) / self.slope

    def sigma_at(self, seconds: float) -> float:
        """Standard error of the line at ``seconds``."""
        return self.sigma * math.sqrt(1.0 / self.n + (seconds - self.t_mean) ** 2 / self.sxx)

    def sigma_new(self, seconds: float) -> float:
        """Standard error of a new pulse's travel about the line (fix spread only)."""
        return self.sigma * math.sqrt(1.0 + 1.0 / self.n + (seconds - self.t_mean) ** 2 / self.sxx)

    def outlier(self, limit: float, z: float = 3.0) -> float:
        """The largest leave-one-out residual above both ``limit`` and ``z`` standard errors, else 0."""
        return max((abs(r) for r, s in self.loo if abs(r) > max(limit, z * s)), default=0.0)


@dataclass
class Outcome:
    ok: bool
    reason: str
    side: str | None = None
    before_creep: tuple[float, float] | None = None  # measured ball (x, y) at stage_near
    creep_m: float = 0.0
    efficiency: float | None = None  # forward gain
    log: list[str] = field(default_factory=list)
    no_ball: bool = False  # the ball was not found, not seen, lost or moved (not a fault)


class Abort(Exception):
    """The approach ended early; the message is the reason."""


class NoBall(Abort):
    """The ball is not where the approach needs it: not found, not seen, lost or moved."""


class ApproachIO(Protocol):
    def now(self) -> float: ...
    async def sleep(self, seconds: float) -> None: ...
    def send(self, vx: float, vy: float, wz: float) -> None: ...  # m/s, m/s, rad/s
    async def poll(self) -> Fix | None: ...  # process the newest frame; the newest fix, or None
    def check(self) -> str | None: ...  # reason to stop now, or None


def choose_side(requested: str, first: Fix, p: ApproachParams = ApproachParams()) -> str:
    """The kicking paw: the requested one, or (auto) the side the ball was first seen on."""
    if requested in SIDES:
        return requested
    bearing = math.atan2(first.y, first.x)
    return "left" if bearing > p.auto_deadband else "right"


def _clip(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def _robust_mean(values: list[float]) -> tuple[float, float]:
    """Mean and its standard error, without values far from the median."""
    med = statistics.median(values)
    mad = statistics.median(abs(v - med) for v in values)
    keep = [v for v in values if abs(v - med) <= max(4.5 * mad, 0.01)]
    mean = statistics.fmean(keep)
    se = statistics.stdev(keep) / math.sqrt(len(keep)) if len(keep) > 1 else 0.01
    return mean, max(se, 0.0005)


def _line(points: list[tuple[float, float]]) -> tuple[float, float, float, float] | None:
    """Least-squares (slope, offset, t_mean, sxx); None if the durations span less than 0.3 s."""
    ts = [t for t, _ in points]
    if len(points) < 2 or max(ts) - min(ts) < 0.3:
        return None
    tm = statistics.fmean(ts)
    dm = statistics.fmean(d for _, d in points)
    sxx = sum((t - tm) ** 2 for t in ts)
    slope = sum((t - tm) * (d - dm) for t, d in points) / sxx
    return slope, dm - slope * tm, tm, sxx


def fit_walker(pulses: list[Pulse], min_s: float) -> WalkerFit | None:
    """Line fit of the forward pulses not shorter than ``min_s``; None if they do not span 0.3 s."""
    used = [q for q in pulses if q.seconds >= min_s]
    points = [(q.seconds, q.dx) for q in used]
    line = _line(points)
    if line is None:
        return None
    slope, offset, tm, sxx = line
    sigma = max(0.003, math.sqrt(statistics.fmean(q.sdx ** 2 for q in used)))
    loo = []
    if len(points) >= 3:
        for i, (t, d) in enumerate(points):
            rest = points[:i] + points[i + 1:]
            other = _line(rest)
            if other is None:  # the other pulses cannot predict this one
                continue
            s = sigma * math.sqrt(1.0 + 1.0 / len(rest) + (t - other[2]) ** 2 / other[3])
            loo.append((d - (other[0] * t + other[1]), s))
    return WalkerFit(slope, offset, len(points), tm, sxx, sigma, tuple(loo))


class Approach:
    def __init__(self, io: ApproachIO, side: str = "auto", geometry: KickGeometry = KickGeometry(),
                 params: ApproachParams = ApproachParams()):
        if side not in (*SIDES, "auto"):
            raise ValueError("side must be right, left or auto")
        self.io, self.requested, self.geo, self.p = io, side, geometry, params
        self.side: str | None = None
        self.eff = params.efficiency  # forward gain
        self.eff_y = params.efficiency  # sideways gain
        self.dead = params.dead_time  # s, the same for all axes
        self.fit: WalkerFit | None = None
        self.pulses: list[Pulse] = []  # measured forward pulses
        self.search_yaw = 0.0  # rad, commanded search yaw in this approach
        self.travel = 0.0
        self.t0 = 0.0
        self.log: list[str] = []
        self._history: list[tuple[float, float, float, float]] = []  # (t, vx, vy, wz) sent

    # ---------------------------------------------------------------- helpers
    def note(self, text: str) -> None:
        self.log.append(f"{self.io.now() - self.t0:6.2f}s {text}")

    def guard(self) -> None:
        if reason := self.io.check():
            raise Abort(reason)
        if self.io.now() - self.t0 > self.p.timeout_s:
            raise Abort(f"approach timeout ({self.p.timeout_s:g} s)")
        if self.travel > self.p.travel_max:
            raise Abort(f"commanded travel budget used ({self.p.travel_max:g} m)")

    def send(self, vx: float, vy: float, wz: float) -> None:
        now = self.io.now()
        if self._history:
            t, pvx, pvy, _ = self._history[-1]
            self.travel += math.hypot(pvx, pvy) * max(0.0, now - t)
        self._history.append((now, vx, vy, wz))
        del self._history[:-200]
        self.io.send(vx, vy, wz)

    def stop(self) -> None:
        self.send(0.0, 0.0, 0.0)

    async def tick(self) -> None:
        await self.io.sleep(1.0 / self.p.rate_hz)
        self.guard()

    async def hold(self, seconds: float) -> None:
        end = self.io.now() + seconds
        while self.io.now() < end - 1e-9:
            self.stop()
            await self.tick()

    async def fresh(self, since: float | None = None) -> Fix | None:
        fix = await self.io.poll()
        if fix is None or not all(math.isfinite(v) for v in (fix.x, fix.y, fix.stamp)):
            return None
        if self.io.now() - fix.stamp > self.p.max_obs_age or (since is not None and fix.stamp < since):
            return None
        return fix

    def compensate(self, fix: Fix) -> tuple[float, float]:
        """Move a fix from its capture time to now with the commands sent since
        (scaled by the forward gain)."""
        x, y = fix.x, fix.y
        now = self.io.now()
        for i, (t, vx, vy, wz) in enumerate(self._history):
            t_end = self._history[i + 1][0] if i + 1 < len(self._history) else now
            dt = min(t_end, now) - max(t, fix.stamp)
            if dt <= 0.0:
                continue
            e = self.eff
            x, y = x - vx * e * dt, y - vy * e * dt
            th = -wz * e * dt
            c, s = math.cos(th), math.sin(th)
            x, y = c * x - s * y, s * x + c * y
        return x, y

    async def measure(self, label: str) -> Measure:
        """Stand still, then the mean of the fixes captured after settling."""
        await self.hold(self.p.settle_s)
        since = self.io.now()
        xs, ys, seen = [], [], set()
        end = since + self.p.measure_s
        while self.io.now() < end or (len(xs) < self.p.measure_min and self.io.now() < end + self.p.measure_s):
            fix = await self.fresh(since)
            if fix is not None and fix.precise and fix.stamp not in seen:
                seen.add(fix.stamp)
                xs.append(fix.x)
                ys.append(fix.y)
            self.stop()
            await self.tick()
        if len(xs) < self.p.measure_min:
            raise NoBall(f"ball not seen ({label}: {len(xs)} fixes)")
        (x, sx), (y, sy) = _robust_mean(xs), _robust_mean(ys)
        self.note(f"measure {label}: x={x:.3f} y={y:+.3f} ({len(xs)} fixes)")
        return Measure(x, y, sx, sy, len(xs))

    async def pulse(self, vx: float, vy: float, wz: float, seconds: float) -> None:
        end = self.io.now() + seconds
        try:
            while self.io.now() < end - 1e-9:
                self.send(vx, vy, wz)
                await self.io.sleep(min(1.0 / self.p.rate_hz, end - self.io.now()))
                self.guard()
        finally:
            self.stop()

    def pulse_time(self, distance: float, speed: float, gain: float) -> float:
        """Correction pulse duration; never shorter than the dead time + 0.03 s."""
        low = max(self.p.pulse_min_s, self.dead + 0.03)
        return min(self.p.pulse_max_s, max(low, self.dead + abs(distance) / (speed * gain)))

    async def forward(self, before: Measure, seconds: float, label: str, speed: float | None = None) -> Measure:
        """One forward (or, with a negative speed, backward) pulse, then a measure."""
        v = self.p.pulse_vx if speed is None else speed
        await self.pulse(v, 0.0, 0.0, seconds)
        after = await self.measure(label)
        if v == self.p.pulse_vx:  # the model is for this speed only
            q = Pulse(seconds, before.x - after.x, after.y - before.y,
                      math.hypot(before.sx, after.sx), math.hypot(before.sy, after.sy), label)
            self.pulses.append(q)
            self.note(f"forward pulse {seconds:.2f} s: {q.dx:+.3f} m forward, ball y {q.dy:+.3f} m")
            if self.fit is None:
                self.learn_dead(seconds, q.dx, self.eff * v)
        return after

    async def sideways(self, before: Measure, distance: float, label: str) -> Measure:
        """One sideways pulse toward ``distance`` (ball y error), then a measure; learns the sideways gain."""
        p = self.p
        vy = math.copysign(p.pulse_vy, distance)
        seconds = self.pulse_time(distance, p.pulse_vy, self.eff_y)
        await self.pulse(0.0, vy, 0.0, seconds)
        after = await self.measure(label)
        moved = before.y - after.y if vy > 0 else after.y - before.y
        if not self.learn_dead(seconds, moved, self.eff_y * p.pulse_vy):
            commanded = p.pulse_vy * (seconds - self.dead)
            if commanded >= 0.012 and moved > 0:
                lo, hi = p.efficiency_bounds
                self.eff_y = min(hi, max(lo, 0.5 * self.eff_y + 0.5 * moved / commanded))
        return after

    def learn_dead(self, seconds: float, moved: float, rate: float) -> bool:
        """A pulse with much less travel than expected: increase the dead time. True if increased."""
        expected = rate * max(0.0, seconds - self.dead)
        if expected < 0.008 or moved >= 0.4 * expected:
            return False
        implied = seconds - max(0.0, moved) / rate
        old, self.dead = self.dead, min(self.p.dead_bounds[1], self.dead + 0.5 * max(0.0, implied - self.dead))
        self.note(f"pulse {seconds:.2f} s moved {moved:+.3f} m of {expected:.3f} m: "
                  f"dead time {old:.2f} -> {self.dead:.2f} s")
        return True

    def fixed_model(self) -> WalkerFit:
        """The walker model from the parameters (calibrate 0): travel = gain * v * (T - dead)."""
        p = self.p
        slope = p.efficiency * p.pulse_vx
        self.eff, self.dead = p.efficiency, p.dead_time
        self.note(f"fixed walker model: gain {p.efficiency:.3f}, dead {p.dead_time:.3f} s, sigma {p.fixed_sigma * 1000:.1f} mm")
        return WalkerFit(slope=slope, offset=-slope * p.dead_time, n=1, t_mean=0.0, sxx=1e9, sigma=p.fixed_sigma)

    def model(self, label: str) -> WalkerFit:
        """Fit the forward pulses; stop the approach if the fit is not plausible or not consistent."""
        p = self.p
        fit = fit_walker(self.pulses, p.fit_min_s)
        if fit is None:
            raise Abort(f"calibration ({label}): the forward pulses do not give a fit")
        lo, hi = p.efficiency_bounds
        gain = fit.slope / p.pulse_vx
        if not lo <= gain <= hi:
            raise Abort(f"calibration ({label}): forward gain {gain:.2f} outside {lo:g} to {hi:g}")
        dead = -fit.offset / fit.slope
        dlo, dhi = p.dead_bounds
        if not dlo <= dead <= dhi:
            raise Abort(f"calibration ({label}): dead time {dead:.2f} s outside {dlo:g} to {dhi:g} s")
        if off := fit.outlier(p.fit_resid_max):
            raise Abort(f"calibration ({label}): one forward pulse is {off * 100:.1f} cm off the others "
                        f"(limit {p.fit_resid_max * 100:g} cm); the ball moved or the walker is not consistent")
        self.fit, self.eff, self.dead = fit, gain, dead
        worst = max((abs(r) for r, _ in fit.loo), default=0.0)
        self.note(f"model ({label}): gain {gain:.3f}, dead {dead:.3f} s, {fit.n} pulses, "
                  f"sigma {fit.sigma * 1000:.1f} mm, largest leave-one-out residual {worst * 1000:.1f} mm")
        return fit

    def drift(self) -> tuple[float, float]:
        """Ball y shift per m of forward travel over all forward pulses, and its standard error."""
        used = [q for q in self.pulses if q.dx > 0.02]
        total = sum(q.dx for q in used)
        if total < 0.05:
            return 0.0, math.inf
        return sum(q.dy for q in used) / total, math.sqrt(sum(q.sdy ** 2 for q in used)) / total

    def drift_shift(self, distance: float) -> float:
        """Expected ball y shift over ``distance`` m of forward travel (0 if the drift is not significant)."""
        drift, sigma = self.drift()
        if abs(drift) <= 2.0 * sigma:  # not distinguishable from zero
            return 0.0
        return _clip(drift * distance, self.p.drift_max)

    def near_y(self, paw_y: float) -> float:
        """Ball y target before the creep: the expected creep drift ends at the paw."""
        return paw_y - self.drift_shift(self.p.stage_near - self.geo.contact_x)

    # ---------------------------------------------------------------- phases
    async def find(self) -> Fix:
        """Look, then turn in steps; returns a confirmed fix (robot standing still)."""
        p = self.p
        limit = min(p.search_total, p.search_yaw_max - self.search_yaw)
        turned = 0.0
        while True:
            found = await self.look(p.look_s)
            if found is not None:
                self.note(f"found ball x={found.x:.2f} y={found.y:+.2f} after {math.degrees(turned):.0f} deg")
                return found
            if turned >= limit - 1e-6:
                budget = " (search budget used)" if limit < p.search_total else ""
                raise NoBall(f"ball not found after {math.degrees(turned):.0f} deg of commanded turn{budget}")
            step = min(p.search_step, limit - turned)
            self.note(f"search: turn {math.degrees(step):.0f} deg")
            start = self.io.now()
            end = start + step / p.search_wz
            try:
                while self.io.now() < end - 1e-9:
                    self.send(0.0, 0.0, p.search_wz)
                    await self.io.sleep(min(1.0 / p.rate_hz, end - self.io.now()))
                    self.guard()
                    if await self.fresh(start) is not None:  # captured during this turn: stop and confirm
                        break
            finally:
                self.stop()
            done = min(step, p.search_wz * (self.io.now() - start))
            turned += done
            self.search_yaw += done

    async def look(self, seconds: float) -> Fix | None:
        """Stand still up to ``seconds``; return a fix confirmed by confirm_n agreeing fixes."""
        start = self.io.now()
        recent: list[Fix] = []
        while self.io.now() - start < seconds:
            self.stop()
            fix = await self.fresh(start)
            if fix is not None and (not recent or fix.stamp != recent[-1].stamp):
                recent.append(fix)
                recent = recent[-self.p.confirm_n:]
                if len(recent) == self.p.confirm_n:
                    xs, ys = [f.x for f in recent], [f.y for f in recent]
                    if max(xs) - min(xs) <= self.p.confirm_spread and max(ys) - min(ys) <= self.p.confirm_spread:
                        return recent[-1]
            await self.tick()
        return None

    async def drive_to_stage(self, stage_x: float | None = None) -> None:
        """Continuous approach until the ball is near stage_x (default stage_far) ahead at the paw's y."""
        p, ty = self.p, self.geo.paw_y(self.side)
        stage_x = p.stage_far if stage_x is None else stage_x
        searches, last_fresh = 0, self.io.now()
        while True:
            fix = await self.fresh()
            if fix is None:
                self.stop()
                if self.io.now() - last_fresh > p.lost_s:
                    searches += 1
                    if searches > p.searches_max:
                        raise NoBall("ball lost during the approach")
                    self.note("ball lost: search again")
                    await self.find()
                    last_fresh = self.io.now()
                await self.tick()
                continue
            last_fresh = self.io.now()
            x, y = self.compensate(fix)
            heading = math.atan2(y - ty, x)  # zero when the ball is at the paw's y
            if abs(heading) <= p.stage_heading and x - stage_x < 0.08:
                self.stop()
                return
            wz = _clip(p.yaw_kp * heading, p.max_wz)
            vx = 0.0 if abs(heading) > p.turn_first else min(
                p.approach_speed, max(0.0, p.approach_kx * (x - stage_x)))
            if vx < 0.1 and abs(heading) > p.stage_heading:  # turning in place: stay above the dead band
                wz = math.copysign(max(abs(wz), p.min_turn_wz), heading)
            self.send(vx, 0.0, wz)
            await self.tick()

    async def correct(self, label: str, target_x: float, target_y: Callable[[], float], tol_x: float,
                      tol_y: float, iters: int, m: Measure | None = None) -> tuple[Measure, bool]:
        """Stop-and-look pulses toward (target_x, target_y()); returns the last measure and success."""
        p = self.p
        if m is None:
            m = await self.measure(label)
        for _ in range(iters):
            ex, ey = m.x - target_x, m.y - target_y()
            if abs(ex) <= tol_x and abs(ey) <= tol_y:
                return m, True
            if abs(ey) > 0.2 or abs(ex) > 0.5:
                raise NoBall(f"ball moved during the line-up (x={m.x:.2f}, y={m.y:+.2f})")
            if abs(ey) > tol_y:  # sideways first: a forward pulse keeps y
                m = await self.sideways(m, ey, label)
            else:
                speed = math.copysign(p.pulse_vx, ex)
                m = await self.forward(m, self.pulse_time(p.pulse_gain * ex, p.pulse_vx, self.eff), label, speed)
        return m, abs(m.x - target_x) <= tol_x and abs(m.y - target_y()) <= tol_y

    async def calibrate(self, m0: Measure) -> Measure:
        """Short and long forward pulse, model fit, then a rehearsal pulse of the creep length.
        A ball not seen after one of these pulses is a walker-model fault (the ball can be in
        the blind zone), not an absent ball: it ends the approach as a fault."""
        try:
            return await self._calibrate(m0)
        except NoBall as error:
            raise Abort(f"calibration: {error} after a calibration pulse; the walker can have moved"
                        " farther than the model allows") from error

    async def _calibrate(self, m0: Measure) -> Measure:
        p, g = self.p, self.geo
        v, (lo, hi), dead_hi = p.pulse_vx, p.efficiency_bounds, p.dead_bounds[1]
        ma = await self.forward(m0, p.cal_short_s, "calibration 1")
        # Long pulse: the longest that keeps the ball at stage_near - tol_x or farther for
        # each walker with a gain and dead time in the bounds that explains pulse 1.
        moved, sdx = self.pulses[-1].dx, self.pulses[-1].sdx
        da = moved - 2.0 * sdx  # low bound
        if da < lo * v * (p.cal_short_s - dead_hi):
            raise Abort(f"calibration: the short pulse moved the ball {moved:.3f} +- {sdx:.3f} m, "
                        f"too little or too uncertain to time the long pulse")
        worst_dead = min(dead_hi, p.cal_short_s - da / (v * hi))
        room = ma.x - (p.stage_near - p.tol_x)
        long_s = min(p.cal_long_max_s, worst_dead + room * (p.cal_short_s - worst_dead) / da)
        if long_s < p.cal_long_min_s:
            raise Abort(f"calibration: no room for the long pulse (ball at x={ma.x:.3f})")
        mb = await self.forward(ma, long_s, "calibration 2")
        fit = self.model("2 pulses")
        # Rehearsal: a pulse of approximately the creep length that ends at stage_near.
        creep_len = p.stage_near - g.contact_x
        m, backups = mb, 0
        while m.x - p.stage_near < 0.8 * creep_len:
            if backups == 2:
                raise Abort(f"calibration: could not back up for the rehearsal (ball at x={m.x:.3f})")
            backups += 1
            back = p.stage_near + creep_len - m.x
            m = await self.forward(m, min(p.creep_max_s, self.dead + back / (self.eff * v)), "rehearsal start", -v)
        # Sideways first, so that the rehearsal (and its drift) ends at the creep's start y.
        rehearsal_y = self.near_y(g.paw_y(self.side)) - self.drift_shift(m.x - p.stage_near)
        m, _ = await self.correct("rehearsal start", m.x, lambda: rehearsal_y, 1.0, p.tol_y, p.align_iters, m)
        seconds = fit.seconds(m.x - p.stage_near)
        if seconds > p.creep_max_s:
            raise Abort(f"calibration: the rehearsal needs {seconds:.2f} s (limit {p.creep_max_s:g} s)")
        predicted = fit.travel(seconds)
        mv = await self.forward(m, seconds, "rehearsal")
        error = self.pulses[-1].dx - predicted
        sigma = fit.sigma_new(seconds)
        self.note(f"rehearsal {seconds:.2f} s: {self.pulses[-1].dx:.3f} m, predicted {predicted:.3f} "
                  f"+- {sigma:.3f} m")
        if abs(error) > max(p.fit_resid_max, 3.0 * sigma):
            raise Abort(f"calibration: the rehearsal pulse moved {self.pulses[-1].dx:.3f} m, the model "
                        f"predicted {predicted:.3f} m; the ball moved or the walker is not consistent")
        self.model("rehearsal")
        return mv

    async def run(self) -> Outcome:
        self.t0 = self.io.now()
        p, g = self.p, self.geo
        try:
            try:
                check_params(p, g)
            except ValueError as error:
                raise Abort(f"invalid approach parameters: {error}") from None
            self.guard()
            first = await self.find()
            self.side = choose_side(self.requested, first, p)
            ty = g.paw_y(self.side)
            self.note(f"side {self.side} (requested {self.requested}); paw y {ty:+.3f}")
            if p.calibrate:
                await self.drive_to_stage()
                m, _ = await self.correct("stage_far", p.stage_far, lambda: ty, p.tol_far, p.tol_far, p.stage_iters)
                m = await self.calibrate(m)
            else:
                self.fit = self.fixed_model()
                await self.drive_to_stage(p.stage_near + 0.10)
                m = None
            m, ok = await self.correct("stage_near", p.stage_near, lambda: self.near_y(ty), p.tol_x, p.tol_y,
                                       p.align_iters, m)
            near_y = self.near_y(ty)
            if not ok:
                if abs(m.x - p.stage_near) <= p.tol_x and abs(m.y - near_y) <= p.tol_y_loose:
                    self.note(f"aligned within the loose tolerance ({m.y - near_y:+.3f} m sideways)")
                else:
                    raise Abort(f"could not line up the {self.side} paw (ball at x={m.x:.3f}, y={m.y:+.3f})")
            fit = self.model("before the creep") if p.calibrate else self.fit
            drift, drift_se = self.drift()
            self.note(f"drift {drift:+.3f} +- {drift_se:.3f} m/m; ball y target {near_y:+.3f}")
            # Aim short: a ball pushed under the body cannot be kicked.
            full = m.x - g.contact_x
            seconds = fit.seconds(full)
            sigma = math.hypot(fit.sigma_at(seconds), fit.sigma / math.sqrt(2.0))  # line and last measure
            margin = p.creep_margin + p.creep_k * sigma
            if margin + 3.0 * sigma > p.tol_x:
                raise Abort(f"creep travel too uncertain ({sigma * 1000:.1f} mm standard error, "
                            f"limit {(p.tol_x - p.creep_margin) / (p.creep_k + 3.0) * 1000:.1f} mm)")
            creep = full - margin
            if creep <= 0.0 or creep > p.stage_near - g.contact_x + p.tol_x:
                raise Abort(f"creep {creep:.3f} m outside 0 to {p.stage_near - g.contact_x + p.tol_x:.3f} m")
            seconds = fit.seconds(creep)
            if seconds > p.creep_max_s:
                raise Abort(f"creep needs {seconds:.2f} s (limit {p.creep_max_s:g} s)")
            self.note(f"creep {creep:.3f} m blind of {full:.3f} m ({seconds:.2f} s at {p.pulse_vx:g} m/s, "
                      f"sigma {sigma * 1000:.1f} mm)")
            await self.pulse(p.pulse_vx, 0.0, 0.0, seconds)
            await self.hold(p.settle_s)
            return Outcome(True, "lined up", self.side, (m.x, m.y), creep, self.eff, self.log)
        except Abort as error:
            self.note(f"abort: {error}")
            return Outcome(False, str(error), self.side, None, 0.0, self.eff, self.log,
                           no_ball=isinstance(error, NoBall))
        finally:
            try:
                self.io.send(0.0, 0.0, 0.0)
            except Exception:  # a refused zero (motion blocked) is already zero
                pass


def check_params(p: ApproachParams, g: KickGeometry = KickGeometry()) -> None:
    """Refuse values that make the approach unsafe or meaningless (ValueError, all problems in one message)."""
    problems: list[str] = []

    def need(ok: bool, text: str) -> None:
        if not ok:
            problems.append(text)

    lo, hi = p.efficiency_bounds
    need(0.0 < lo < p.efficiency < hi,
         f"efficiency_bounds ({lo:g}, {hi:g}) must be positive and in order, with efficiency {p.efficiency:g} between")
    dlo, dhi = p.dead_bounds
    need(dlo < p.dead_time < dhi < p.cal_short_s,
         f"dead_bounds ({dlo:g}, {dhi:g}) must be in order, with dead_time {p.dead_time:g} between "
         f"and the upper bound below cal_short_s {p.cal_short_s:g}")
    need(0.0 < g.contact_x < p.stage_near - 0.10,
         f"contact_x {g.contact_x:g} must be positive and below stage_near - 0.10 ({p.stage_near - 0.10:g})")
    need(abs(g.right_y) <= 0.15 and abs(g.left_y) <= 0.15, "right_y and left_y must be within 0.15 m")
    need(p.stage_near >= BLIND_X + p.tol_x,
         f"stage_near {p.stage_near:g} must be at least {BLIND_X:g} + tol_x ({BLIND_X + p.tol_x:g})")
    cal_travel = hi * p.pulse_vx * (p.cal_short_s + p.cal_long_min_s)
    need(not p.calibrate or p.stage_far - p.tol_far >= p.stage_near - p.tol_x + cal_travel,
         f"stage_far {p.stage_far:g} must be at least stage_near - tol_x + tol_far + calibration travel "
         f"({p.stage_near - p.tol_x + p.tol_far + cal_travel:.3f})")
    need(p.cal_short_s + 0.3 <= p.cal_long_min_s <= p.cal_long_max_s <= p.creep_max_s <= 5.0,
         "need cal_short_s + 0.3 <= cal_long_min_s <= cal_long_max_s <= creep_max_s <= 5")
    for name, cap in (("approach_speed", 0.5), ("pulse_vx", 0.3), ("pulse_vy", 0.3),
                      ("max_wz", 1.5), ("search_wz", 1.5), ("min_turn_wz", 1.5)):
        value = getattr(p, name)
        need(0.0 < value <= cap, f"{name} {value:g} must be above 0 and at most {cap:g}")
    need(p.min_turn_wz <= p.max_wz, "min_turn_wz must not be above max_wz")
    for name in ("tol_far", "tol_x", "tol_y", "tol_y_loose", "confirm_spread", "look_s", "measure_s",
                 "max_obs_age", "lost_s", "approach_kx", "yaw_kp", "search_step", "fit_resid_max",
                 "stage_heading"):
        need(getattr(p, name) > 0.0, f"{name} must be above 0")
    need(p.tol_y_loose >= p.tol_y, "tol_y_loose must not be below tol_y")
    need(p.turn_first > p.stage_heading, "turn_first must be above stage_heading")
    for name in ("settle_s", "pulse_min_s", "fit_min_s", "creep_margin", "creep_k", "drift_max",
                 "auto_deadband", "searches_max"):
        need(getattr(p, name) >= 0.0, f"{name} must not be below 0")
    need(p.pulse_min_s < p.pulse_max_s, "pulse_min_s must be below pulse_max_s")
    need(0.0 < p.pulse_gain <= 1.0, "pulse_gain must be above 0 and at most 1")
    need(p.search_step <= p.search_total <= p.search_yaw_max <= math.radians(1080.0),
         "need search_step <= search_total <= search_yaw_max <= 1080 deg")
    for name in ("confirm_n", "stage_iters", "align_iters"):
        need(getattr(p, name) >= 1, f"{name} must be at least 1")
    need(p.measure_min >= 2, "measure_min must be at least 2")
    need(0.0 < p.travel_max <= 8.0, f"travel_max {p.travel_max:g} must be above 0 and at most 8 m")
    need(0.0 < p.timeout_s <= 180.0, f"timeout_s {p.timeout_s:g} must be above 0 and at most 180 s")
    need(5.0 <= p.rate_hz <= 50.0, f"rate_hz {p.rate_hz:g} must be 5 to 50 Hz")
    if problems:
        raise ValueError("; ".join(problems))


@dataclass(frozen=True)
class KickConfig:
    vision: Any  # ball_vision.VisionParams
    geometry: KickGeometry = KickGeometry()
    approach: ApproachParams = ApproachParams()
    dry_run: bool = False  # trials: stop after the creep, no kick
    calibrated: bool = False  # the operator calibrated the vision for this robot and ball
    snapshot_path: str | None = None  # absolute path for camera snapshots, or None


def _replace(obj, values: Mapping, where: str):
    """A copy of a frozen dataclass with ``values``; refuses unknown keys and wrong types."""
    if not isinstance(values, Mapping):
        raise ValueError(f"{where}: expected a mapping")
    known = {f.name: getattr(obj, f.name) for f in dataclasses.fields(obj)}
    changes = {}
    for key, value in values.items():
        if key not in known:
            raise ValueError(f"{where}: unknown key {key!r}; known: {sorted(known)}")
        current = known[key]
        if isinstance(current, bool):
            if not isinstance(value, bool):
                raise TypeError(f"{where}.{key}: expected true or false")
        elif isinstance(current, tuple):
            if (not isinstance(value, (list, tuple)) or len(value) != len(current)
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                               for v in value)):
                raise TypeError(f"{where}.{key}: expected {len(current)} finite numbers")
            value = tuple(type(c)(v) for c, v in zip(current, value))
        elif not isinstance(current, (int, float)):
            raise ValueError(f"{where}.{key} cannot be set from the calibration file")
        elif not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            raise TypeError(f"{where}.{key}: expected a finite number")
        elif isinstance(current, int):
            if value != int(value):
                raise TypeError(f"{where}.{key}: expected an integer")
            value = int(value)
        else:
            value = float(value)
        changes[key] = value
    return dataclasses.replace(obj, **changes)


def config_from_mapping(data: Mapping) -> KickConfig:
    """Calibration overrides, for example from YAML:

        vision: {ball_radius: 0.097, y_bias: 0.0}
        mount: {range_scale: 1.0, yaw_deg: 0.0}   # silhouette fit (the old 0.616 was for the colour box)
        color: {s_min: 140}
        geometry: {contact_x: 0.258, right_y: -0.046, left_y: 0.040}
        approach: {stage_near: 0.50, efficiency: 0.8}
        dry_run: true      # trials only: line up, then stop without the kick
        calibrated: true   # the vision values above come from a calibration
        snapshot_path: /tmp/gemini-pupper/ball_snapshot.jpg   # locate_ball writes this JPEG

    Approach and geometry values outside their safe ranges raise ValueError.
    """
    from ball_vision import VisionParams  # NumPy only

    sections = {"vision", "mount", "color", "geometry", "approach", "dry_run", "calibrated", "snapshot_path"}
    if not isinstance(data, Mapping) or set(data) - sections:
        raise ValueError(f"calibration sections must be among {sorted(sections)}")
    flags = {}
    for key in ("dry_run", "calibrated"):
        flags[key] = data.get(key, False)
        if not isinstance(flags[key], bool):
            raise TypeError(f"{key}: expected true or false")
    snapshot_path = data.get("snapshot_path")
    if snapshot_path is not None and (not isinstance(snapshot_path, str) or not os.path.isabs(snapshot_path)):
        raise ValueError("snapshot_path: expected an absolute path")
    vision = VisionParams()
    vision = dataclasses.replace(vision, mount=_replace(vision.mount, data.get("mount", {}), "mount"),
                                 color=_replace(vision.color, data.get("color", {}), "color"))
    vision = _replace(vision, data.get("vision", {}), "vision")
    geometry = _replace(KickGeometry(), data.get("geometry", {}), "geometry")
    params = _replace(ApproachParams(), data.get("approach", {}), "approach")
    check_params(params, geometry)
    return KickConfig(vision, geometry, params, flags["dry_run"], flags["calibrated"], snapshot_path)


async def approach(io: ApproachIO, side: str = "auto", geometry: KickGeometry = KickGeometry(),
                   params: ApproachParams = ApproachParams()) -> Outcome:
    return await Approach(io, side, geometry, params).run()
