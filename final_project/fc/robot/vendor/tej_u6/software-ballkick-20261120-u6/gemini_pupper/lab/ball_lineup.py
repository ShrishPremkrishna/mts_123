"""Kick line-up from Ball Eye fixes: walk to a near ball and stop with the ball at
the kick point of one front paw. Pure logic (no ROS, no camera): step() gets the
time, returns one speed command, and says when to kick.

Phases:
  wait      no track yet: zero speed.
  approach  ball far: walk ahead and turn, so that the ball comes onto the paw line.
  align     ball near: ahead/back and sideways speeds at the same time, heading
            held by the walker. Each axis runs one time and stops when the predicted
            ball position (tracker, across the frame delay and the walker delay)
            gets to its target.
  settle    zero speed, then the median of fixes from frames taken at rest.
            In the window: kick. Not in the window: one timed pulse on each axis
            that is out, then settle again. Each pulse updates the measured
            travel per commanded metre, so the next pulse is more accurate.

The walker barely moves below its minimum speeds (0.2 m/s), so position is
controlled by time at a fixed speed with camera feedback, not by a small speed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from statistics import median

from ball_eye import BallFix, BallTrack

# Kick clip (pupper-mjlab tricks/leg_kick.py): paw centre at the fastest point of the swing, and foot radius.
STRIKE_X, STRIKE_Z, FOOT_R = 0.145, 0.068, 0.020
# Ball centre y for a ball that leaves straight ahead (trained policy in simulation, 0.091 m ball,
# 2026-11-19). The direction changes approximately 6 degrees per centimetre.
AIM_Y = {"right": -0.042, "left": 0.037}


def contact_x(radius: float) -> float:
    """Ball centre x at which the paw first touches the ball at the fastest point of the swing."""
    return STRIKE_X + math.sqrt((radius + FOOT_R) ** 2 - (radius - STRIKE_Z) ** 2)


@dataclass(frozen=True)
class LineupParams:
    ball_radius: float = 0.091
    x_short: float = 0.005  # target x = contact_x - x_short: the middle of the simulated window
    tol_x: float = 0.012  # kick window half-widths [m]
    tol_y: float = 0.008
    loose: float = 2.0  # after max_pulses, kick if inside loose x the window (force)
    near: float = 0.30  # m of x error at which align starts
    v_far: float = 0.35  # m/s commands
    v_near: float = 0.20
    v_side: float = 0.20
    w_max: float = math.radians(60.0)  # rad/s
    w_min: float = math.radians(20.0)
    k_turn: float = 2.0  # 1/s
    turn_only_deg: float = 25.0  # approach: turn in place when the ball is farther off the paw line
    steer_dead_deg: float = 3.0
    lead_s: float = 0.25  # walker delay and stop time: predict this far past now
    settle_s: float = 0.30  # zero speed before a frame counts as taken at rest
    settle_fixes: int = 3
    settle_timeout_s: float = 1.5
    gain0: float = 0.84  # travel per commanded metre (leg odometry, 2026-11-19); updated from each pulse
    pulse_min_s: float = 0.05  # one command period
    pulse_max_s: float = 0.60
    max_pulses: int = 5
    wait_s: float = 2.5  # give up if no ball is seen for this long
    timeout_s: float = 25.0
    force: bool = True  # kick after max_pulses if inside the loose window


@dataclass
class Cmd:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0  # rad/s
    phase: str = "wait"
    kick: str | None = None  # "right" / "left": kick now
    done: str | None = None  # the line-up ended with no kick: the reason
    info: dict = field(default_factory=dict)


class Lineup:
    def __init__(self, params: LineupParams = LineupParams(), side: str = "auto", track: BallTrack | None = None):
        self.p, self.side = params, side
        self.track = track or BallTrack()
        self.phase, self.t0, self.last_seen = "wait", None, None
        self.dir = None  # align: direction of each axis that still runs (x, y)
        self.settle_at = None
        self.rest: list[BallFix] = []
        self.pulses = 0
        self.pulse = None  # (vx, end x, vy, end y, (x, y) before, commanded travel (x, y))
        self.gain = {"x": params.gain0, "y": params.gain0}
        self.log: list[str] = []

    def target(self) -> tuple[float, float]:
        return contact_x(self.p.ball_radius) - self.p.x_short, AIM_Y[self.side]

    def add_fix(self, fix: BallFix) -> None:
        self.track.update(fix)
        self.last_seen = fix.stamp
        if self.phase == "settle" and self.settle_at is not None and fix.stamp >= self.settle_at + self.p.settle_s:
            self.rest.append(fix)

    def _note(self, now: float, text: str) -> None:
        self.log.append(f"{now - self.t0:5.2f} s {text}")

    def _settle(self, now: float) -> None:
        self.phase, self.settle_at, self.rest = "settle", now, []

    def step(self, now: float) -> Cmd:
        p = self.p
        if self.t0 is None:
            self.t0 = now
        if now - self.t0 > p.timeout_s:
            return Cmd(phase=self.phase, done=f"no line-up in {p.timeout_s:g} s")
        ball = self.track.predict(now + p.lead_s)
        if ball is None and self.phase != "settle" and self.pulse is None:
            seen = self.last_seen if self.last_seen is not None else self.t0
            if now - seen > p.wait_s:
                return Cmd(phase=self.phase, done="ball not seen" if self.last_seen is None else "ball lost")
            return Cmd(phase=self.phase)  # zero speed until a fix comes
        if self.side == "auto":
            self.side = "left" if ball[1] > (AIM_Y["left"] + AIM_Y["right"]) / 2.0 else "right"
            self._note(now, f"side {self.side} (ball y {ball[1]:+.3f})")
        tx, ty = self.target()

        if self.phase == "wait":
            self.phase = "approach"
        if self.phase == "approach":
            ex, ey = ball[0] - tx, ball[1] - ty
            if ex <= p.near:
                self.phase, self.dir = "align", None
                self._note(now, f"align from x {ball[0]:.3f} y {ball[1]:+.3f}")
            else:
                off = math.atan2(ey, max(ball[0], 0.05))
                wz = 0.0
                if abs(math.degrees(off)) > p.steer_dead_deg:
                    wz = math.copysign(min(p.w_max, max(p.w_min, p.k_turn * abs(off))), off)
                vx = 0.0 if abs(math.degrees(off)) > p.turn_only_deg else (p.v_far if ex > 0.5 else p.v_near)
                return Cmd(vx, 0.0, wz, "approach", info={"ex": ex, "ey": ey})
        if self.phase == "align":
            ex, ey = ball[0] - tx, ball[1] - ty
            if self.dir is None:  # each axis runs one time, toward its target
                self.dir = [math.copysign(1.0, ex) if abs(ex) > 0.5 * p.tol_x else 0.0,
                            math.copysign(1.0, ey) if abs(ey) > 0.5 * p.tol_y else 0.0]
            # An axis ends when its predicted error gets to zero. It does not start again: the pulses do the rest.
            self.dir = [d if d * e > 0 else 0.0 for d, e in zip(self.dir, (ex, ey))]
            if not any(self.dir):
                self._settle(now)
                self._note(now, f"settle (predicted error x {ex:+.3f} y {ey:+.3f})")
            else:
                return Cmd(self.dir[0] * p.v_near, self.dir[1] * p.v_side, 0.0, "align", info={"ex": ex, "ey": ey})
        if self.phase == "pulse":
            vx, end_x, vy, end_y = self.pulse[:4]
            if now < max(end_x, end_y):
                return Cmd(vx if now < end_x else 0.0, vy if now < end_y else 0.0, 0.0, "pulse")
            self._settle(now)
        # settle
        if len(self.rest) < p.settle_fixes:
            if now - self.settle_at > p.settle_s + p.settle_timeout_s:
                return Cmd(phase="settle", done="ball lost at rest")
            return Cmd(phase="settle")
        x, y = median(f.x for f in self.rest), median(f.y for f in self.rest)
        ex, ey = x - tx, y - ty
        if self.pulse is not None:  # measured travel / commanded travel of the last pulse, each axis
            (x0, y0), (cx, cy) = self.pulse[4], self.pulse[5]
            for axis, moved, commanded in (("x", x0 - x, cx), ("y", y0 - y, cy)):
                if abs(commanded) > 1e-6:
                    self.gain[axis] = min(1.6, max(0.3, 0.5 * self.gain[axis] + 0.5 * moved / commanded))
            self.pulse = None
        info = {"x": x, "y": y, "ex": ex, "ey": ey, "pulses": self.pulses, "side": self.side}
        if abs(ex) <= p.tol_x and abs(ey) <= p.tol_y:
            self._note(now, f"kick {self.side}: ball x {x:.3f} y {y:+.3f} (error x {ex:+.3f} y {ey:+.3f})")
            return Cmd(phase="kick", kick=self.side, info=info)
        if self.pulses >= p.max_pulses:
            if p.force and abs(ex) <= p.loose * p.tol_x and abs(ey) <= p.loose * p.tol_y:
                self._note(now, f"kick {self.side} outside the window: error x {ex:+.3f} y {ey:+.3f}")
                return Cmd(phase="kick", kick=self.side, info=info)
            return Cmd(phase="settle", done=f"not lined up after {self.pulses} pulses (error x {ex:+.3f} y {ey:+.3f})",
                       info=info)
        if abs(ex) > p.near:  # the ball moved away: approach again
            self.phase = "approach"
            return Cmd(phase="approach", info=info)
        # One timed pulse on each axis that is out of the window: time = distance / (gain x speed).
        self.pulses += 1
        dx = ex if abs(ex) > 0.6 * p.tol_x else 0.0
        dy = ey if abs(ey) > 0.6 * p.tol_y else 0.0
        tx_s = min(p.pulse_max_s, max(p.pulse_min_s, abs(dx) / (self.gain["x"] * p.v_near))) if dx else 0.0
        ty_s = min(p.pulse_max_s, max(p.pulse_min_s, abs(dy) / (self.gain["y"] * p.v_side))) if dy else 0.0
        vx, vy = math.copysign(p.v_near, dx) if dx else 0.0, math.copysign(p.v_side, dy) if dy else 0.0
        self.pulse = (vx, now + tx_s, vy, now + ty_s, (x, y), (vx * tx_s, vy * ty_s))
        self.phase = "pulse"
        self._note(now, f"pulse {self.pulses}: error x {ex:+.3f} y {ey:+.3f} -> vx {vx:+.2f} for {tx_s:.2f} s,"
                        f" vy {vy:+.2f} for {ty_s:.2f} s")
        return Cmd(vx if tx_s else 0.0, vy if ty_s else 0.0, 0.0, "pulse", info=info)
