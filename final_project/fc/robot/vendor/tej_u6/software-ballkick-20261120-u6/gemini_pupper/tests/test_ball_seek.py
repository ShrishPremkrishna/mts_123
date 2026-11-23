"""ball_seek.Seeker against a simulated robot: walker with unknown gains and start delay,
camera with noise, missed frames, false detections, a 220 deg view and the blind zone.

A run passes when the kick starts with the true ball centre within the kick window of
the chosen paw: x from contact - 3 cm (pushed toward the body) to contact + 4 cm
(short is the safe side), y within 1.5 cm of the paw line.
"""

from __future__ import annotations

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
from ball_seek import PAW_Y, Abort, Obs, SeekParams, Seeker  # noqa: E402


class SimRobot:
    """World frame: robot (X, Y, th); ball (bx, by). Times in simulated seconds."""

    # Camera noise per look (locate_ball median), from the robot on 2026-11-18: x spread
    # 2 to 3 mm at 0.58 m and about 8 mm at 0.93 m; y spread 1 to 2 mm.
    def __init__(self, ball, *, fwd_gain=1.31, side_gain=1.0, turn_gain=1.0, dead=0.14, seed=0,
                 noise_x=0.004, noise_y=0.0015, miss=0.25, false_pos=0.02, blind=0.45, view_deg=105.0,
                 max_range=1.4, slip=0.03, heading=0.0):
        self.X = self.Y = 0.0
        self.th = heading
        self.ball = ball
        self.g = {"fwd": fwd_gain, "side": side_gain, "turn": turn_gain}
        self.dead, self.rng = dead, random.Random(seed)
        self.noise_x, self.noise_y, self.miss, self.false_pos = noise_x, noise_y, miss, false_pos
        self.blind, self.view, self.max_range, self.slip = blind, math.radians(view_deg), max_range, slip
        self.t = 0.0
        self.kicked = None  # (side, ball x, ball y) relative to the robot at the kick

    def now(self):
        return self.t

    def rel(self):
        dx, dy = self.ball[0] - self.X, self.ball[1] - self.Y
        c, s = math.cos(self.th), math.sin(self.th)
        return c * dx + s * dy, -s * dx + c * dy

    def look(self):
        self.t += 1.4
        if self.rng.random() < self.false_pos:  # a wrong object somewhere in view
            r, b = self.rng.uniform(0.5, 1.3), self.rng.uniform(-1.5, 1.5)
            return Obs(r * math.cos(b), r * math.sin(b), "fit")
        x, y = self.rel()
        r, b = math.hypot(x, y), math.atan2(y, x)
        if r < self.blind or r > self.max_range or abs(b) > self.view or self.rng.random() < self.miss:
            return None
        n = 1.0 + r  # noise grows with range
        return Obs(x + self.rng.gauss(0, self.noise_x * n), y + self.rng.gauss(0, self.noise_y * n), "fit")

    def walk(self, vx, vy, seconds):
        eff = max(0.0, seconds - self.dead) * (1.0 + self.rng.gauss(0, self.slip))
        dx, dy = vx * self.g["fwd"] * eff, vy * self.g["side"] * eff
        n = max(1, math.ceil(math.hypot(dx, dy) / 0.01))  # 1 cm sub-steps: the body pushes the ball
        for _ in range(n):
            c, s = math.cos(self.th), math.sin(self.th)
            self.X += (c * dx - s * dy) / n
            self.Y += (s * dx + c * dy) / n
            self.collide(dx / n, dy / n)
        self.th += self.rng.gauss(0, math.radians(1.0)) * (abs(dx) + abs(dy)) / 0.2  # small heading drift
        self.t += seconds + 0.6

    def turn(self, degrees):
        self.th += math.radians(degrees) * self.g["turn"] * (1.0 + self.rng.gauss(0, self.slip))
        self.t += abs(degrees) / 60.0 + 0.6
        self.collide(0.0, 0.0)

    # Nearest ball centre to base_link at each face of the body (face + ball radius 0.097 m).
    CHEST_X = 0.151 + 0.097
    REAR_X = 0.15 + 0.097
    SIDE_Y = 0.06 + 0.097

    def collide(self, dx, dy):
        """The body pushes the ball out along the motion (the robot cannot walk through it)."""
        x, y = self.rel()
        if not (-self.REAR_X < x < self.CHEST_X and abs(y) < self.SIDE_Y):
            return
        if abs(dx) >= abs(dy) and dx:  # moved along x: push off the face it moves toward
            nx, ny = (self.CHEST_X if dx > 0 else -self.REAR_X), y
        elif dy:
            nx, ny = x, (self.SIDE_Y if dy > 0 else -self.SIDE_Y)
        else:  # a turn: push off the nearest face
            gaps = {(self.CHEST_X, y): self.CHEST_X - x, (-self.REAR_X, y): x + self.REAR_X,
                    (x, self.SIDE_Y): self.SIDE_Y - y, (x, -self.SIDE_Y): y + self.SIDE_Y}
            nx, ny = min(gaps, key=gaps.get)
        c, s = math.cos(self.th), math.sin(self.th)
        self.ball = (self.X + c * nx - s * ny, self.Y + s * nx + c * ny)

    def kick(self, side):
        x, y = self.rel()
        self.kicked = (side, x, y)
        self.t += 6.0
        return f"kick_{side} played"

    def stop(self):
        pass


def good_kick(sim: SimRobot) -> bool:
    if sim.kicked is None:
        return False
    side, x, y = sim.kicked
    p = SeekParams()
    return p.contact_x - 0.03 <= x <= p.contact_x + 0.04 and abs(y - PAW_Y[side]) <= 0.02


def place(rng: random.Random) -> tuple[float, float]:
    """A ball near the robot: in view, behind, or inside the blind zone in front."""
    kind = rng.random()
    if kind < 0.6:
        r, b = rng.uniform(0.5, 1.2), rng.uniform(-1.6, 1.6)
    elif kind < 0.85:
        r, b = rng.uniform(0.4, 1.0), rng.uniform(2.0, 4.3)  # behind
    else:
        r, b = rng.uniform(0.28, 0.42), rng.uniform(-0.5, 0.5)  # in the blind zone, near the nose
    return r * math.cos(b), r * math.sin(b)


def run_case(seed: int, side: str, fwd_spread: float = 0.10, miss: float = 0.25, false_pos: float = 0.02):
    rng = random.Random(seed)
    sim = SimRobot(place(rng), fwd_gain=1.31 * rng.uniform(1 - fwd_spread, 1 + fwd_spread), side_gain=rng.uniform(0.7, 1.4),
                   turn_gain=rng.uniform(0.7, 1.4), dead=rng.choice([rng.uniform(0.0, 0.03), rng.uniform(0.06, 0.22)]),
                   seed=seed, miss=miss,
                   false_pos=false_pos)
    seeker = Seeker(sim, side, log=lambda text: None)
    try:
        result = seeker.run()
    except Abort as error:
        result = f"abort: {error}"
    return sim, seeker, result


@pytest.mark.parametrize("side,harsh", [("right", False), ("left", False), ("auto", False), ("right", True), ("left", True)])
def test_stress_many_placements(side, harsh):
    """150 placements each. 2026-11-18 result: about 92 % good kicks, 3 % safe aborts,
    4 % kicks outside the window (forward error sd 1.5 cm, sideways sd 0.85 cm).
    harsh: 40 % missed frames and 5 % wrong-object looks."""
    good = aborted = bad = 0
    for seed in range(150):
        offset = (1000 if side == "left" else 2000 if side == "auto" else 0) + (7000 if harsh else 0)
        sim, seeker, result = run_case(seed + offset, side, miss=0.4 if harsh else 0.25,
                                       false_pos=0.05 if harsh else 0.02)
        if result.startswith("abort"):
            aborted += 1
            assert sim.kicked is None  # an abort never kicks
        elif good_kick(sim):
            good += 1
        else:
            bad += 1
    assert good >= (0.82 if harsh else 0.88) * 150 and bad <= (0.07 if harsh else 0.05) * 150, (good, aborted, bad)


def test_never_kicks_with_the_ball_under_the_body():
    for seed in range(300):
        sim, _, _ = run_case(seed + 5000, "right")
        if sim.kicked:
            assert sim.kicked[1] >= SeekParams().contact_x - 0.03, sim.kicked


def test_ball_in_the_blind_zone_is_found_by_backing_up():
    sim = SimRobot((0.35, -0.03), seed=1, miss=0.0, false_pos=0.0)
    seeker = Seeker(sim, "right", log=lambda text: None)
    assert seeker.run().startswith("kicked")
    assert good_kick(sim), sim.kicked


@pytest.mark.parametrize("ball,turn_gain", [((0.35, -0.03), 1.31), ((-0.30, 0.05), 1.0), ((0.05, 0.38), 1.0),
                                            ((-0.30, 0.05), 0.8), ((0.33, 0.10), 1.15)])
def test_ball_in_the_blind_zone_with_an_unknown_turn_gain(ball, turn_gain):
    """The timed search turn over- or under-turns: the heading after a full turn is not known.
    Every run must find the ball and kick. Heading drift during the creep puts some kicks
    just outside the window (as in the stress test), so 4 of 5 must be in it."""
    good = 0
    for seed in range(5):
        sim = SimRobot(ball, turn_gain=turn_gain, seed=11 + seed, miss=0.0, false_pos=0.0)
        lines = []
        assert Seeker(sim, "right", log=lines.append).run().startswith("kicked"), lines
        good += good_kick(sim)
    assert good >= 4, good


def test_ball_behind_is_found_by_turning():
    sim = SimRobot((-0.7, 0.2), seed=2, miss=0.0, false_pos=0.0)
    lines = []
    seeker = Seeker(sim, "left", log=lines.append)
    assert seeker.run().startswith("kicked") and good_kick(sim), (sim.kicked, lines)
    assert any("no ball: turn left" in line for line in lines)


def test_no_ball_gives_up_without_kicking():
    sim = SimRobot((50.0, 0.0), seed=3, false_pos=0.0)
    with pytest.raises(Abort):
        Seeker(sim, "right", log=lambda text: None).run()
    assert sim.kicked is None


def test_dry_run_lines_up_and_does_not_kick():
    sim = SimRobot((0.9, 0.2), seed=4)
    result = Seeker(sim, "right", dry_run=True, log=lambda text: None).run()
    assert "dry run" in result and sim.kicked is None
    x, y = sim.rel()
    assert abs(x - SeekParams().stage_x) <= 0.06 and abs(y - PAW_Y["right"]) <= 0.02, (x, y)


def test_auto_side_follows_the_ball():
    for ball, expected in (((0.8, 0.3), "left"), ((0.8, -0.3), "right"), ((0.8, 0.0), "right")):
        sim = SimRobot(ball, seed=5, miss=0.0, false_pos=0.0)
        Seeker(sim, "auto", log=lambda text: None).run()
        assert sim.kicked[0] == expected


def test_bad_side_is_refused():
    with pytest.raises(ValueError):
        Seeker(SimRobot((1, 0)), "middle")
