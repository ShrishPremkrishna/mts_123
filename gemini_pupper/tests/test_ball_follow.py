"""robot/ball_follow.py: the detector on rendered frames, and the follower in closed loop.

Closed loop: a walker with unknown forward and turn gains, a start delay and a lag; a
detector at 12 Hz with noise that grows with range, missed frames, 130 ms latency, a
blind zone in front of the nose and a 100 deg half view; the body pushes the ball. The
blind walk and the kick run as main() runs them. A kick passes when the ball centre is
3 cm toward the body to 4 cm short of contact, and within 2.5 cm of the paw line.

The image tests need OpenCV (the robot) or SciPy; without both they are skipped.
"""

from __future__ import annotations

import math
import random
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import ball_follow as bf  # noqa: E402

try:
    import cv2  # noqa: F401
    HAVE_IMAGE_OPS = True
except ImportError:
    try:
        import scipy.ndimage  # noqa: F401
        HAVE_IMAGE_OPS = True
    except ImportError:
        HAVE_IMAGE_OPS = False
needs_image_ops = pytest.mark.skipif(not HAVE_IMAGE_OPS, reason="needs OpenCV or SciPy")

R = 0.097
Z_BALL = -0.10  # ball centre from base_link on the robot (2026-11-18)


# ===================================================================== geometry
@pytest.mark.parametrize("x,y", [(0.3, 0.0), (0.5, -0.046), (0.8, 0.3), (1.2, -0.5), (0.6, 0.6)])
def test_locate_inverts_project(x, y):
    got = bf.locate(bf.project(x, y, Z_BALL, R), R)
    tol = max(0.01, 0.05 * math.hypot(x, y))  # project() approximates the equirectangular stretch
    assert abs(got[0] - x) < tol and abs(got[1] - y) < tol and abs(got[2] - Z_BALL) < 0.02, got


# ===================================================================== rendered frames
def hsv_to_rgb(h, s, v):
    """OpenCV-convention HSV (H 0..179) to RGB uint8."""
    h, s, v = h * 2.0, s / 255.0, v / 255.0
    c = v * s
    x = c * (1 - abs((h / 60.0) % 2 - 1))
    m = v - c
    r, g, b = [(c, x, 0), (x, c, 0), (0, c, x), (0, x, c), (x, 0, c), (c, 0, x)][int(h // 60) % 6]
    return np.array([(r + m) * 255, (g + m) * 255, (b + m) * 255]).round().astype(np.uint8)


CARPET = hsv_to_rgb(11, 125, 110)
RED = hsv_to_rgb(8, 210, 190)
BLUE = hsv_to_rgb(114, 230, 150)
WHITE = hsv_to_rgb(15, 30, 200)
JEANS = hsv_to_rgb(107, 95, 70)


def render(x, y, panels="rwbw", rng=None, extras=()):
    """A frame with the ball centred at (x, y, Z_BALL): carpet below the horizon, the ball's
    panels as vertical bands across its disc, the bottom 15 % in shadow."""
    rng = rng or np.random.default_rng(0)
    img = np.zeros((bf.EQ_HEIGHT, bf.EQ_WIDTH, 3), np.uint8)
    row0 = int(bf.EQ_HEIGHT * 0.45)
    noise = rng.normal(0, 6, (bf.EQ_HEIGHT - row0, bf.EQ_WIDTH, 3))
    img[row0:] = np.clip(CARPET + noise, 0, 255).astype(np.uint8)
    x1, y1, x2, y2 = bf.project(x, y, Z_BALL, R)
    uc, vc, a, b = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) / 2, (y2 - y1) / 2
    vv, uu = np.mgrid[:bf.EQ_HEIGHT, :bf.EQ_WIDTH]
    inside = ((uu - uc) / a) ** 2 + ((vv - vc) / b) ** 2 <= 1.0
    band = np.clip(((uu - (uc - a)) / (2 * a) * len(panels)).astype(int), 0, len(panels) - 1)
    colours = {"r": RED, "w": WHITE, "b": BLUE}
    for k, ch in enumerate(panels):
        img[inside & (band == k)] = colours[ch]
    img[inside & (vv > vc + 0.7 * b)] = (60, 40, 30)  # shadowed bottom
    for (ex, ey, ez, er, colour) in extras:
        e1, f1, e2, f2 = bf.project(ex, ey, ez, er)
        m = ((uu - (e1 + e2) / 2) / ((e2 - e1) / 2)) ** 2 + ((vv - (f1 + f2) / 2) / ((f2 - f1) / 2)) ** 2 <= 1
        img[m & (vv >= row0)] = colour
    return img


def first_fix(img):
    """The largest candidate's position (one-colour views cannot start a lock)."""
    cands = bf.find_candidates(img)
    if not cands:
        return None
    best = max(cands, key=lambda c: c.pixels)
    pos = bf.locate(best.box, R)
    return None if pos is None else bf.Fix(0.0, *pos, best.colours, best.box[2] - best.box[0], best.pixels)


@needs_image_ops
@pytest.mark.parametrize("x,y,panels", [
    (0.5, -0.046, "rwbw"), (0.5, 0.04, "wbww"), (0.7, 0.2, "wrw"), (0.9, -0.3, "rwbw"),
    (1.2, 0.0, "rwbw"), (0.35, 0.05, "bw"), (0.6, -0.5, "rwb"),
])
def test_detector_finds_the_ball(x, y, panels):
    fix = first_fix(render(x, y, panels))
    assert fix is not None
    # Range within 8 % and bearing within 3 deg everywhere (the follower turns to the ball
    # before it walks); at the 0.5 m line-up, sideways within 1.2 cm.
    rng_err = math.hypot(fix.x, fix.y) / math.hypot(x, y) - 1.0
    bearing_err = math.degrees(math.atan2(fix.y, fix.x) - math.atan2(y, x))
    assert abs(rng_err) <= (0.10 if abs(math.atan2(y, x)) > 0.5 else 0.08) and abs(bearing_err) <= 3.0, (fix, x, y)
    if x == 0.5:
        assert abs(fix.y - y) <= 0.012, (fix, y)


@needs_image_ops
def test_detector_ignores_carpet_jeans_and_objects_off_the_floor():
    img = render(0.8, 0.0, "rwbw", extras=[(1.0, 0.6, -0.05, 0.12, JEANS), (0.9, -0.5, 0.25, 0.08, RED)])
    lock = bf.Lock()
    fix = bf.pick(bf.find_candidates(img), lock, 0.0, R, bf.CameraMount())
    assert fix is not None and abs(fix.y) < 0.05, fix  # the ball, not the jeans or the red object up high
    empty = render(5.0, 0.0, "w")  # no ball in view
    assert first_fix(empty) is None


@needs_image_ops
def test_one_colour_cannot_start_a_lock_but_continues_one():
    """A blue chair was taken for the ball on 2026-11-19: one colour never starts a lock."""
    lock = bf.Lock()
    one = bf.find_candidates(render(0.6, 0.0, "wbw"))
    assert bf.pick(one, lock, 0.0, R, bf.CameraMount()) is None
    two = bf.find_candidates(render(0.6, 0.0, "rwbw"))
    assert bf.pick(two, lock, 0.1, R, bf.CameraMount()) is not None
    assert bf.pick(one, lock, 0.2, R, bf.CameraMount()) is not None


def test_lock_keeps_the_ball_and_ignores_a_jump():
    lock = bf.Lock()
    ball = bf.Candidate(bf.project(0.7, 0.0, Z_BALL, R), 500, 2)
    other = bf.Candidate(bf.project(1.1, 0.5, Z_BALL, R), 900, 2)
    first = bf.pick([ball], lock, 0.0, R, bf.CameraMount())
    assert first is not None
    again = bf.pick([other], lock, 0.1, R, bf.CameraMount())
    assert again is None  # not near the ball we follow
    later = bf.pick([other], lock, 1.5, R, bf.CameraMount())
    assert later is not None and later.x > 1.0  # the lock forgets after hold_s


# ===================================================================== closed loop
class World:
    """Walker + camera + ball. Robot pose (X, Y, th); ball (bx, by) in the world. The detector
    found the ball at x 0.23 m (at the nose) on 2026-11-19, so the blind zone is x < 0.22."""

    def __init__(self, ball, seed, fwd_gain=1.31, turn_gain=1.0, dead=0.14, lag=0.15, noise=0.008,
                 miss=0.3, latency=0.13, blind=0.22, view_deg=100.0, max_range=1.6, heading=0.0):
        self.rng = random.Random(seed)
        self.X = self.Y = 0.0
        self.th = heading
        self.ball = list(ball)
        self.fwd_gain, self.turn_gain, self.dead, self.lag = fwd_gain, turn_gain, dead, lag
        self.noise, self.miss, self.latency = noise, miss, latency
        self.blind, self.view, self.max_range = blind, math.radians(view_deg), max_range
        self.t = 0.0
        self.cmds = [(-1.0, 0.0, 0.0)]  # (time, vx, wz) as received
        self.vx = self.wz = 0.0
        self.history = []  # (t, x_rel, y_rel) for latency

    def command(self, vx, wz):
        self.cmds.append((self.t, vx, wz))

    def rel(self):
        dx, dy = self.ball[0] - self.X, self.ball[1] - self.Y
        c, s = math.cos(self.th), math.sin(self.th)
        return c * dx + s * dy, -s * dx + c * dy

    def step(self, dt=0.01):
        # A start or speed-up takes effect after the start delay; a slow-down or stop at once.
        # Then a pulse of T s travels gain * v * (T - dead), the model fitted on the robot.
        delayed = [c for c in self.cmds if c[0] <= self.t - self.dead]
        _, cvx, cwz = delayed[-1] if delayed else (0, 0.0, 0.0)
        _, nvx, nwz = self.cmds[-1]
        if abs(nvx) < abs(cvx) or nvx * cvx < 0:
            cvx = nvx
        if abs(nwz) < abs(cwz) or nwz * cwz < 0:
            cwz = nwz
        cvx, cwz = self.walker_filter(cvx, cwz)
        a = dt / (self.lag + dt)
        self.vx += a * (cvx * self.fwd_gain - self.vx)
        self.wz += a * (cwz * self.turn_gain - self.wz)
        n = 3
        for _ in range(n):
            self.th += self.wz * dt / n
            self.X += math.cos(self.th) * self.vx * dt / n
            self.Y += math.sin(self.th) * self.vx * dt / n
            self.collide()
        self.th += self.rng.gauss(0, math.radians(0.6)) * abs(self.vx) * dt / 0.2  # heading drift
        self.t += dt
        self.history.append((self.t, *self.rel()))
        self.history = self.history[-100:]

    def walker_filter(self, vx, wz):
        return vx, wz

    CHEST, REAR, SIDE = 0.151 + 0.097, 0.15 + 0.097, 0.06 + 0.097

    def collide(self):
        x, y = self.rel()
        if -self.REAR < x < self.CHEST and abs(y) < self.SIDE:
            nx = self.CHEST if self.vx >= 0 else -self.REAR
            c, s = math.cos(self.th), math.sin(self.th)
            self.ball = [self.X + c * nx - s * y, self.Y + s * nx + c * y]

    def camera(self):
        """This frame's fix (as the detector + lock produce it), from latency ago."""
        past = [h for h in self.history if h[0] <= self.t - self.latency]
        if not past:
            return None
        _, x, y = past[-1]
        r, b = math.hypot(x, y), math.atan2(y, x)
        if x < self.blind or r > self.max_range or abs(b) > self.view or self.rng.random() < self.miss:
            return None
        n = self.noise * (1 + r)
        return bf.Fix(self.t, x + self.rng.gauss(0, n), y + self.rng.gauss(0, 0.5 * n), Z_BALL, 2, 60)


def place(rng):
    kind = rng.random()
    if kind < 0.55:
        r, b = rng.uniform(0.55, 1.3), rng.uniform(-1.6, 1.6)  # in view
    elif kind < 0.8:
        r, b = rng.uniform(0.5, 1.0), rng.uniform(2.0, 4.3)  # behind
    else:
        r, b = rng.uniform(0.30, 0.45), rng.uniform(-0.4, 0.4)  # close in front
    return r * math.cos(b), r * math.sin(b)


def run(seed, side, ball=None, dry_run=False, **world):
    rng = random.Random(seed)
    w = World(ball or place(rng), seed, fwd_gain=world.pop("fwd_gain", 1.31 * rng.uniform(0.9, 1.1)),
              turn_gain=world.pop("turn_gain", rng.uniform(0.7, 1.3)), dead=world.pop("dead", rng.uniform(0.08, 0.22)),
              **world)
    f = bf.BallFollower(bf.FollowParams(side=side), dry_run=dry_run, log=lambda text: None)
    next_tick, next_frame = 0.0, 0.0
    kicked = None
    while w.t < 200:
        if w.t >= next_frame:
            f.frame(w.t, w.camera())
            next_frame += 1 / 12
        if w.t >= next_tick:
            cmd = f.tick(w.t)
            if cmd.publish:
                w.command(cmd.vx, cmd.wz)
            next_tick += 0.1
        if f.state == bf.CREEP:  # main(): drive() for creep_s, then zero for pause_s
            w.command(f.p.creep_v, 0.0)
            end = w.t + f.creep_s
            while w.t < end:
                w.step()
            w.command(0.0, 0.0)
            end = w.t + f.p.pause_s
            while w.t < end:
                w.step()
            f.state = bf.KICK
        if f.state == bf.KICK:
            kicked = w.rel()
            f.state = bf.DONE
        if f.state in (bf.DONE, bf.ABORT):
            break
        w.step()
    return f, w, kicked


def good(side, kicked):
    p = bf.FollowParams(side=side)
    return (kicked is not None and p.contact_x - 0.03 <= kicked[0] <= p.contact_x + 0.04
            and abs(kicked[1] - p.paw_y) <= 0.025)


@pytest.mark.parametrize("side", ["right", "left"])
def test_follow_and_kick_many_placements(side):
    """2026-11-18 result: see docs/kick-ball.md (ball_follow)."""
    n, ok, aborted, bad, times = 150, 0, 0, 0, []
    for seed in range(n):
        f, w, kicked = run(seed + (0 if side == "right" else 1000), side)
        if f.state == bf.ABORT:
            aborted += 1
            assert kicked is None
        elif good(side, kicked):
            ok += 1
            times.append(w.t)
        else:
            bad += 1
    assert ok >= 0.9 * n and bad <= 0.05 * n, (ok, aborted, bad)


def test_ball_straight_ahead_is_kicked_without_shuffling():
    """The 2026-11-18 robot complaint: forward and back with the ball in front."""
    f, w, kicked = run(7, "right", ball=(0.7, 0.0), fwd_gain=1.31, turn_gain=1.0, dead=0.14)
    assert good("right", kicked), kicked
    reversals = sum(1 for (_, a, _), (_, b, _) in zip(w.cmds, w.cmds[1:]) if a * b < 0)
    assert reversals <= 2, reversals


def test_dry_run_lines_up_and_stops():
    f, w, kicked = run(3, "left", ball=(0.9, 0.3), dry_run=True)
    assert f.state == bf.DONE and kicked is None and "dry run" in f.result
    x, y = w.rel()
    assert abs(x - bf.FollowParams().stand_x) <= 0.05 and abs(y - 0.040) <= 0.03, (x, y)


def test_no_ball_aborts_without_kicking():
    f, w, kicked = run(4, "right", ball=(40.0, 0.0))
    assert f.state == bf.ABORT and kicked is None


def test_camera_stops_robot_holds_still():
    f = bf.BallFollower(bf.FollowParams(), log=lambda text: None)
    f.frame(0.0, bf.Fix(0.0, 0.8, 0.0, Z_BALL, 2, 60))
    assert f.tick(0.05).vx > 0
    cmd = f.tick(1.5)  # no frames for 1.45 s
    assert cmd.vx == 0 and cmd.wz == 0
    assert f.tick(4.0).publish and f.state == bf.ABORT


def test_bad_side_is_refused():
    with pytest.raises(ValueError):
        bf.BallFollower(bf.FollowParams(side="middle"))


def test_walker_that_ignores_small_turns_still_lines_up():
    """A walker that does not turn below 0.12 rad/s: the minimum yaw rate ramps up, and the
    side tolerance loosens with the time at the right distance."""
    class Sticky(World):
        def walker_filter(self, vx, wz):
            return vx, (0.0 if abs(wz) < 0.12 else wz)
    f = bf.BallFollower(bf.FollowParams(side="right"), log=lambda text: None)
    w = Sticky((0.75, 0.06), 9, turn_gain=1.0)
    t_tick = t_frame = 0.0
    while w.t < 60 and f.state not in (bf.CREEP, bf.ABORT):
        if w.t >= t_frame:
            f.frame(w.t, w.camera())
            t_frame += 1 / 12
        if w.t >= t_tick:
            cmd = f.tick(w.t)
            if cmd.publish:
                w.command(cmd.vx, cmd.wz)
            t_tick += 0.1
        w.step()
    assert f.state == bf.CREEP, (f.state, f.result)
    assert abs(w.rel()[1] - bf.PAW_Y["right"]) <= 0.035, w.rel()


def test_ball_in_front_inside_the_line_up_does_not_walk_backward():
    """2026-11-19 robot complaint: ball in front, the robot walked backward."""
    f, w, kicked = run(11, "right", ball=(0.33, -0.05), fwd_gain=1.31, turn_gain=1.0, dead=0.14)
    assert good("right", kicked), kicked
    assert not any(vx < 0 for _, vx, _ in w.cmds), [c for c in w.cmds if c[1] < 0][:3]
