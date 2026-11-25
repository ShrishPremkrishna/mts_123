"""Ball velocity from the overhead camera's sightings, and the rolling-deceleration fit for
`ctl rollcal`. Stdlib only (the Eye, fc.ctl and fc.tools.ball_vel_replay import it).

Validation (2026-11-24, `python -m fc.tools.ball_vel_replay --synthetic fc/runs/20261124T07*`):
ball at rest in 14 Nov 24 floor runs (~170 s): reported speed exactly 0 (old 0.3 s line fit:
median 0.003, p95 0.03, max 0.19 m/s, non-zero 73 % of frames). Synthetic rolls (3 mm noise +
3 % 2 cm flickers, 31 fps, instant release at 0.5-2 m/s): speed within 50 % and 90 % of the true
value 0.06 s (2 frames) after release, rolling speed error median 0.01-0.02, max 0.05 m/s.
"""
from __future__ import annotations

import math

from .core import wrap


# ---------------------------------------------------------------------------------------
class BallVelocity:
    """Ball position and velocity from capture-timestamped sightings.

    Weighted least squares line through the sightings of the last `window_s` seconds (weights
    fall off with age, time constant `tau_s`), evaluated at the newest sighting. With a rolling
    deceleration `decel` (brain param ball_decel) the slope, which is the velocity at the
    weighted mean time, is brought forward to the newest sighting. Robustness:
      * a sighting farther than `gate_m` from the fit is held back; the next sighting decides
        whether it was an outlier (dropped) or the ball really jumped (window restarts there),
      * when 2 sightings in a row land > `kink_m` from where the fit predicted them, on the same
        side (a kick or a bounce), the window is cut back to them and the sighting before,
      * a gap of more than `gap_s` (ball lost, re-acquired) restarts the window,
      * speed below `v_on` (hysteresis down to `v_off`), or below 3 standard errors of the slope
        (sighting noise `sigma_m`; matters while the window is short), is reported as exactly zero
        and the position as the window mean: a ball at rest or held does not drift.
    The old estimator (plain line over 0.3 s, no gating) reported 0.03-0.05 m/s for a ball at rest
    and spikes of > 1 m/s on a false sighting."""

    def __init__(self, window_s: float = 0.4, tau_s: float = 0.25, gap_s: float = 0.3, gate_m: float = 0.12,
                 kink_m: float = 0.02, v_on: float = 0.08, v_off: float = 0.05, min_span_s: float = 0.06,
                 sigma_m: float = 0.004):
        self.window_s, self.tau_s, self.gap_s, self.gate_m = window_s, tau_s, gap_s, gate_m
        self.kink_m, self.v_on, self.v_off, self.min_span_s = kink_m, v_on, v_off, min_span_s
        self.sigma_m = sigma_m   # Nov 24 floor runs at rest: ~1-2 mm frame to frame, single 1-2.5 cm flickers
        self.reset()

    def reset(self) -> None:
        self.pts: list[tuple[float, float, float]] = []
        self.held: tuple[float, float, float] | None = None
        self.innov: tuple[float, float] | None = None   # last prediction error if > kink_m
        self.moving = False
        self.out: tuple[float, float, float, float] | None = None
        self.resets = self.outliers = 0

    def _fit(self, pts, decel: float):
        """(x, y, vx, vy, mean x, mean y, slope standard error) at the newest point of pts."""
        t1 = pts[-1][0]
        w = [math.exp(-(t1 - t) / self.tau_s) for t, _, _ in pts]
        sw = sum(w)
        tm = sum(wi * p[0] for wi, p in zip(w, pts)) / sw
        xm = sum(wi * p[1] for wi, p in zip(w, pts)) / sw
        ym = sum(wi * p[2] for wi, p in zip(w, pts)) / sw
        stt = sum(wi * (p[0] - tm) ** 2 for wi, p in zip(w, pts))
        if len(pts) < 3 or t1 - pts[0][0] < self.min_span_s or stt < 1e-9:
            return pts[-1][1], pts[-1][2], 0.0, 0.0, xm, ym, math.inf
        vx = sum(wi * (p[0] - tm) * (p[1] - xm) for wi, p in zip(w, pts)) / stt
        vy = sum(wi * (p[0] - tm) * (p[2] - ym) for wi, p in zip(w, pts)) / stt
        # sighting noise: sigma_m, or the scatter about this line if larger (a flicker in a short window)
        res = sum(wi * ((p[1] - xm - vx * (p[0] - tm)) ** 2 + (p[2] - ym - vy * (p[0] - tm)) ** 2) / 2
                  for wi, p in zip(w, pts)) / sw
        sig = max(self.sigma_m, math.sqrt(res * len(pts) / max(1, len(pts) - 2)))
        se = sig * math.sqrt(sum(wi * wi * (p[0] - tm) ** 2 for wi, p in zip(w, pts))) / stt
        x, y = xm + vx * (t1 - tm), ym + vy * (t1 - tm)
        sp = math.hypot(vx, vy)
        if decel > 0 and sp > 1e-6:   # slope = speed at tm; the ball has slowed since
            k = max(0.0, sp - decel * (t1 - tm)) / sp
            vx, vy = vx * k, vy * k
        return x, y, vx, vy, xm, ym, se

    def _predict(self, t: float, decel: float):
        x, y, vx, vy = self._fit(self.pts, decel)[:4]
        dt = t - self.pts[-1][0]
        return x + vx * dt, y + vy * dt

    def update(self, t: float, x: float, y: float, decel: float = 0.0) -> tuple[float, float, float, float]:
        """Add a sighting at capture time t; returns (x, y, vx, vy) at t (or the last estimate
        while a sighting is held back as a possible outlier)."""
        if self.pts and (t - self.pts[-1][0] > self.gap_s or t <= self.pts[-1][0] - 1e-9):
            self.pts, self.held, self.innov, self.moving = [], None, None, False
            self.resets += 1
        if self.pts and t <= self.pts[-1][0]:
            return self.out or (x, y, 0.0, 0.0)
        kink = False
        if len(self.pts) >= 3:
            px, py = self._predict(t, decel)
            ex, ey = x - px, y - py
            e = math.hypot(ex, ey)
            if e > self.gate_m:
                h = self.held
                if h is not None and math.hypot(x - h[1], y - h[2]) <= self.gate_m + 3.0 * (t - h[0]):
                    self.pts, self.moving, self.innov = [h], False, None   # two agreeing sightings: it did move there
                    self.resets += 1
                else:
                    self.held = (t, x, y)
                    self.outliers += 1
                    return self.out or (x, y, 0.0, 0.0)
            elif e > self.kink_m:
                pe = self.innov
                kink = pe is not None and ex * pe[0] + ey * pe[1] > 0
                self.innov = None if kink else (ex, ey)
            else:
                self.innov = None
        self.held = None
        if kink:   # kick / bounce: keep the sighting before the change and the two after it
            self.pts = self.pts[-2:]
        self.pts.append((t, x, y))
        self.pts = [p for p in self.pts if t - p[0] <= self.window_s]
        fx, fy, vx, vy, xm, ym, se = self._fit(self.pts, decel)
        sp = math.hypot(vx, vy)
        self.moving = sp > (self.v_off if self.moving else self.v_on) and sp > 3.0 * se
        self.out = (fx, fy, vx, vy) if self.moving else (xm, ym, 0.0, 0.0)
        return self.out


def roll_segments(points, v_min: float = 0.15, min_s: float = 0.5, min_n: int = 8, max_turn_deg: float = 20.0,
                  gap_s: float = 0.25, tail_s: float = 0.15) -> list[list[tuple[float, float, float]]]:
    """Free-rolling stretches of a raw ball track [(t, x, y)]: moving faster than v_min, from the
    peak speed on (the hand or robot has let go), straight (direction within max_turn_deg of the
    direction at the peak), not speeding up again (a hit), with the last tail_s dropped (the
    velocity window lags the stop)."""
    f = BallVelocity(v_on=v_min, v_off=v_min)
    runs, cur, last_t = [], [], None
    for p in sorted(points):
        if last_t is not None and p[0] - last_t > gap_s and cur:
            runs.append(cur)
            cur = []
        last_t = p[0]
        _, _, vx, vy = f.update(*p)
        sp = math.hypot(vx, vy)
        if sp < v_min:
            if cur:
                runs.append(cur)
            cur = []
        else:
            cur.append((p, sp, math.atan2(vy, vx)))
    if cur:
        runs.append(cur)
    out = []
    for run in runs:
        k = max(range(len(run)), key=lambda i: run[i][1])
        head, lo, seg = run[k][2], run[k][1], []
        for p, sp, h in run[k:]:
            if abs(wrap(h - head)) > math.radians(max_turn_deg) or sp > lo + 0.15:
                break   # bounced off something / hit again
            lo = min(lo, sp)
            seg.append(p)
        seg = [p for p in seg if seg and p[0] <= seg[-1][0] - tail_s]
        if len(seg) >= min_n and seg[-1][0] - seg[0][0] >= min_s:
            out.append(seg)
    return out


def fit_roll(seg) -> dict | None:
    """One free roll [(t, x, y)] -> {decel, v0, secs, n, rms}: s(t) = s0 + v0 t - decel t^2 / 2
    along the roll direction (least squares)."""
    if len(seg) < 4:
        return None
    t0, x0, y0 = seg[0]
    dx, dy = seg[-1][1] - x0, seg[-1][2] - y0
    d = math.hypot(dx, dy)
    if d < 0.05:
        return None
    ux, uy = dx / d, dy / d
    ts = [p[0] - t0 for p in seg]
    ss = [(p[1] - x0) * ux + (p[2] - y0) * uy for p in seg]
    # normal equations for s = a + b t + c t^2
    n = len(ts)
    S = [[sum(t ** (i + j) for t in ts) for j in range(3)] for i in range(3)]
    r = [sum(s * t ** i for t, s in zip(ts, ss)) for i in range(3)]
    try:
        a, b, c = _solve3(S, r)
    except ZeroDivisionError:
        return None
    rms = math.sqrt(sum((s - a - b * t - c * t * t) ** 2 for t, s in zip(ts, ss)) / n)
    return {"decel": round(-2 * c, 3), "v0": round(b, 3), "secs": round(ts[-1], 2), "n": n, "rms_m": round(rms, 4),
            "dist_m": round(d, 2)}


def _solve3(A, b):
    m = [row[:] + [v] for row, v in zip(A, b)]
    for i in range(3):
        p = max(range(i, 3), key=lambda k: abs(m[k][i]))
        if abs(m[p][i]) < 1e-12:
            raise ZeroDivisionError
        m[i], m[p] = m[p], m[i]
        for k in range(3):
            if k != i:
                f = m[k][i] / m[i][i]
                m[k] = [a - f * c for a, c in zip(m[k], m[i])]
    return [m[i][3] / m[i][i] for i in range(3)]


def fit_decel(points) -> dict:
    """Raw ball track -> {"decel": median m/s^2 or None, "rolls": [fit_roll...]}. Rolls whose fit is
    poor (rms > 1.5 cm) or nonsense (decel <= 0 or > 3) are listed but not used."""
    rolls = [r for r in (fit_roll(s) for s in roll_segments(points)) if r]
    for r in rolls:
        r["used"] = 0.0 < r["decel"] <= 3.0 and r["rms_m"] <= 0.015
    good = sorted(r["decel"] for r in rolls if r["used"])
    med = None
    if good:
        k = len(good) // 2
        med = good[k] if len(good) % 2 else 0.5 * (good[k - 1] + good[k])
    return {"decel": None if med is None else round(med, 3), "rolls": rolls, "n_used": len(good)}
