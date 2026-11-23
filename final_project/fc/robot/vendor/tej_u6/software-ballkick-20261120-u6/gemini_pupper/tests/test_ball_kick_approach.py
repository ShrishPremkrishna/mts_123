"""kick_ball approach against a simulated walker and camera (no ROS, no robot).

The simulated walker has dead bands, a dead time before it starts from rest, a
first-order lag, a coast after the stop, a gain, a sideways drift and a yaw drift
that the controller does not know. The simulated camera gives noisy fixes at
12 Hz with 130 ms latency and dropouts, an optional bias (constant plus a slow
random walk), and nothing inside the 0.45 m blind zone or outside 220 deg. Pass
criterion: after the creep, the true ball centre is within the kick tolerance of
the chosen paw.
"""

from __future__ import annotations

import asyncio
import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))

import ball_kick  # noqa: E402
from ball_kick import (Approach, ApproachParams, Fix, KickGeometry, approach, check_params,  # noqa: E402
                       config_from_mapping)

GEO = KickGeometry()
P = ApproachParams()


class SimWorld:
    """World frame: robot pose (X, Y, th); the ball at (bx, by). Time in seconds."""

    def __init__(self, ball=(1.5, 0.0), *, eff=0.85, eff_y=None, drift=0.0, seed=0, noise=0.004,
                 bias_y=0.0, dropout=0.1, latency=0.13, rate=12.0, blind=0.45, stop_at=None,
                 remove_ball_at=None, deadband=(0.06, 0.06, 0.15), tau=0.12, dead=0.0, coast=0.0,
                 bias=(0.0, 0.0), bias_walk=0.0, bias_tau=10.0, yaw_drift=0.0):
        self.t = 0.0
        self.X = self.Y = self.th = 0.0
        self.ball = ball
        self.cmd = (0.0, 0.0, 0.0)
        self.vel = [0.0, 0.0, 0.0]
        self.eff, self.eff_y, self.drift = eff, eff if eff_y is None else eff_y, drift
        self.rng = random.Random(seed)
        self.noise, self.bias_y, self.dropout = noise, bias_y, dropout
        self.latency, self.period, self.blind = latency, 1.0 / rate, blind
        self.stop_at, self.remove_ball_at = stop_at, remove_ball_at
        self.deadband, self.tau = deadband, tau
        self.dead, self.coast, self.yaw_drift = dead, coast, yaw_drift  # s, s, rad per m forward
        self.bias, self.bias_walk, self.bias_tau = bias, bias_walk, bias_tau  # m, m (stationary sigma), s
        self.walk = [0.0, 0.0]
        self.bias_rng = random.Random(seed + 7919)
        self.engaged = -math.inf  # time the command became non-zero from rest
        self.released = -math.inf  # time the command became zero
        self.last = (0.0, 0.0, 0.0)  # last command that moved the walker
        self.active = False
        self.next_frame = 0.0
        self.pending: list[tuple[float, Fix]] = []  # (delivery time, fix)
        self.delivered: Fix | None = None
        self.sent = 0
        self.max_cmd = [0.0, 0.0, 0.0]
        self.cmd_yaw = 0.0  # rad, integral of the commanded yaw rate

    # ---------------------------------------------------------------- physics
    def relative(self) -> tuple[float, float] | None:
        if self.ball is None:
            return None
        dx, dy = self.ball[0] - self.X, self.ball[1] - self.Y
        c, s = math.cos(self.th), math.sin(self.th)
        return c * dx + s * dy, -s * dx + c * dy

    def shift_ball(self, forward: float) -> None:
        """Move the ball along the robot heading (positive: away from the robot)."""
        bx, by = self.ball
        self.ball = (bx + forward * math.cos(self.th), by + forward * math.sin(self.th))

    def walker_command(self) -> tuple[float, float, float]:
        """The command the walker acts on: no motion during the dead time, the last command during the coast."""
        moving = any(self.cmd)
        if moving and not self.active:
            if max(abs(v) for v in self.vel) < 1e-3:
                self.engaged = self.t
        if not moving and self.active:
            self.released = self.t
        self.active = moving
        if moving:
            if self.t - self.engaged < self.dead:
                return (0.0, 0.0, 0.0)
            self.last = self.cmd
            return self.cmd
        if self.t - self.released < self.coast:
            return self.last
        self.last = (0.0, 0.0, 0.0)
        return self.last

    def step(self, dt: float) -> None:
        db, cmd = self.deadband, self.walker_command()
        self.cmd_yaw += self.cmd[2] * dt
        target = [0.0 if abs(cmd[0]) < db[0] else cmd[0] * self.eff,
                  0.0 if abs(cmd[1]) < db[1] else cmd[1] * self.eff_y,
                  0.0 if abs(cmd[2]) < db[2] else cmd[2] * self.eff]
        a = min(1.0, dt / self.tau)
        for i in range(3):
            self.vel[i] += a * (target[i] - self.vel[i])
        vx, vy, wz = self.vel[0], self.vel[1] + self.drift * self.vel[0], self.vel[2] + self.yaw_drift * self.vel[0]
        c, s = math.cos(self.th), math.sin(self.th)
        self.X += (c * vx - s * vy) * dt
        self.Y += (s * vx + c * vy) * dt
        self.th += wz * dt
        self.t += dt
        if self.remove_ball_at is not None and self.t >= self.remove_ball_at:
            self.ball = None
        if self.t >= self.next_frame:
            self.next_frame += self.period
            k = math.exp(-self.period / self.bias_tau)
            for i in range(2):
                self.walk[i] = k * self.walk[i] + self.bias_walk * math.sqrt(1 - k * k) * self.bias_rng.gauss(0, 1)
            rel = self.relative()
            if rel is not None and self.rng.random() >= self.dropout:
                r, bearing = math.hypot(*rel), math.atan2(rel[1], rel[0])
                if self.blind <= r <= 3.0 and abs(bearing) <= math.radians(105.0):
                    fix = Fix(rel[0] + self.bias[0] + self.walk[0] + self.rng.gauss(0.0, self.noise * (1 + r)),
                              rel[1] + self.bias_y + self.bias[1] + self.walk[1] + self.rng.gauss(0.0, self.noise),
                              self.t)
                    self.pending.append((self.t + self.latency, fix))
        while self.pending and self.pending[0][0] <= self.t:
            self.delivered = self.pending.pop(0)[1]

    # ---------------------------------------------------------------- ApproachIO
    def now(self) -> float:
        return self.t

    async def sleep(self, seconds: float) -> None:
        end = self.t + max(0.0, seconds)
        while self.t < end - 1e-9:
            self.step(min(0.005, end - self.t))
        await asyncio.sleep(0)

    def send(self, vx: float, vy: float, wz: float) -> None:
        assert all(math.isfinite(v) for v in (vx, vy, wz))
        self.cmd = (vx, vy, wz)
        self.sent += 1
        self.max_cmd = [max(m, abs(v)) for m, v in zip(self.max_cmd, self.cmd)]

    async def poll(self) -> Fix | None:
        return self.delivered

    def check(self) -> str | None:
        if self.stop_at is not None and self.t >= self.stop_at:
            return "stop requested"
        return None

    def settle(self, seconds: float = 1.0) -> None:
        self.cmd = (0.0, 0.0, 0.0)
        for _ in range(int(seconds / 0.005)):
            self.step(0.005)


def run(world: SimWorld, side: str = "auto", params: ApproachParams = P):
    outcome = asyncio.run(approach(world, side, GEO, params))
    world.settle()
    return outcome


def kick_error(world: SimWorld, side: str) -> tuple[float, float]:
    x, y = world.relative()
    return x - GEO.contact_x, y - GEO.paw_y(side)


CASES = [
    # (ball in the world frame, true forward gain, true sideways gain, drift)
    ((1.5, 0.0), 0.85, None, 0.0),
    ((1.2, 0.5), 0.6, 0.7, 0.03),
    ((1.0, -0.8), 1.1, 1.0, -0.04),
    ((-1.0, 0.6), 0.85, 0.9, 0.02),  # behind: outside the 220 deg view
    ((0.65, 0.05), 0.75, 0.8, 0.0),  # already close: back up to stage_far
    ((2.4, -1.0), 0.95, 0.8, 0.05),
]


@pytest.mark.parametrize("side", ["right", "left"])
@pytest.mark.parametrize("ball,eff,eff_y,drift", CASES)
def test_approach_lines_up_the_paw(ball, eff, eff_y, drift, side):
    world = SimWorld(ball, eff=eff, eff_y=eff_y, drift=drift, seed=int(abs(ball[0] * 100 + ball[1] * 10)) + (side == "left"))
    out = run(world, side)
    assert out.ok, out.reason + "\n" + "\n".join(out.log)
    assert out.side == side
    ex, ey = kick_error(world, side)
    assert abs(ex) <= 0.03, (ex, ey, out.log)
    assert abs(ey) <= P.tol_y_loose + 0.006, (ex, ey, out.log)  # loose tolerance + noise
    assert world.cmd == (0.0, 0.0, 0.0)
    assert world.max_cmd[0] <= P.approach_speed + 1e-9 and world.max_cmd[2] <= P.max_wz + 1e-9


def test_most_cases_meet_the_strict_sideways_tolerance():
    strict = total = 0
    for i, (ball, eff, eff_y, drift) in enumerate(CASES * 3):
        for side in ("right", "left"):
            world = SimWorld(ball, eff=eff, eff_y=eff_y, drift=drift, seed=100 + i)
            out = run(world, side)
            assert out.ok, out.reason
            total += 1
            strict += abs(kick_error(world, side)[1]) <= 0.012
    assert strict / total >= 0.8, f"{strict}/{total}"


def test_auto_side_follows_the_first_sighting():
    left = SimWorld((1.0, 0.7), seed=3)
    assert run(left, "auto").side == "left"
    right = SimWorld((1.0, -0.7), seed=4)
    assert run(right, "auto").side == "right"
    centre = SimWorld((1.5, 0.02), seed=5)  # within the dead band: the tested right paw
    assert run(centre, "auto").side == "right"


def test_calibration_measures_the_walker_gain():
    world = SimWorld((1.5, 0.0), eff=0.6, eff_y=0.6, seed=7)
    out = run(world, "right")
    assert out.ok, out.reason
    assert 0.55 <= out.efficiency <= 0.65, out.efficiency


@pytest.mark.parametrize("dead", [0.1, 0.2])
@pytest.mark.parametrize("eff", [0.6, 1.2])
def test_walker_dead_time_and_coast(dead, eff):
    """A walker that starts late and coasts a little after the stop: the two-pulse fit
    measures the loss, so the blind creep still ends within the kick tolerance."""
    for seed in range(3):
        world = SimWorld((1.6, 0.2), eff=eff, dead=dead, coast=0.03, seed=40 + seed)
        out = run(world, "right")
        assert out.ok, out.reason + "\n" + "\n".join(out.log)
        ex, ey = kick_error(world, "right")
        assert -0.02 <= ex <= 0.025, (ex, out.log)
        assert abs(ey) <= P.tol_y_loose + 0.006, (ey, out.log)


def test_correlated_fix_bias_and_yaw_drift():
    for seed in range(4):
        world = SimWorld((1.8, -0.3), seed=60 + seed, dead=0.1, bias=(0.002, -0.002), bias_walk=0.001,
                         yaw_drift=0.05 * (1 if seed % 2 else -1))
        out = run(world, "left")
        assert out.ok, out.reason
        ex, ey = kick_error(world, "left")
        assert abs(ex) <= 0.03 and abs(ey) <= P.tol_y_loose, (ex, ey, out.log)


def _shift_during(monkeypatch, label: str, shift: float) -> None:
    """Move the ball ``shift`` m along the robot heading at the start of the forward pulse ``label``."""
    original = Approach.forward

    async def forward(self, before, seconds, pulse_label, speed=None):
        if pulse_label == label:
            self.io.shift_ball(shift)
        return await original(self, before, seconds, pulse_label, speed)

    monkeypatch.setattr(Approach, "forward", forward)


@pytest.mark.parametrize("label", ["calibration 1", "calibration 2", "rehearsal"])
@pytest.mark.parametrize("shift", [0.03, 0.05, -0.03, -0.05])
def test_ball_moved_during_calibration_is_caught(monkeypatch, label, shift):
    """A ball that rolls 3 to 5 cm during a calibration pulse gives an abort or a good
    line-up, never a lined-up result with the ball outside the kick tolerance."""
    _shift_during(monkeypatch, label, shift)
    for seed in range(3):
        world = SimWorld((1.5, 0.0), eff=0.85, dead=0.1, seed=80 + seed)
        out = run(world, "right")
        if out.ok:
            ex, _ = kick_error(world, "right")
            assert abs(ex) <= 0.03, (label, shift, ex, out.log)
        assert world.cmd == (0.0, 0.0, 0.0)


def test_drift_compensation_is_necessary():
    """At 0.06 m/m the uncorrected creep drift is 1.5 cm; the correction keeps the ball near the paw."""
    errors = []
    for seed in range(6):
        world = SimWorld((1.5, 0.1), drift=0.06 * (1 if seed % 2 else -1), seed=120 + seed)
        out = run(world, "right")
        assert out.ok, out.reason
        errors.append(abs(kick_error(world, "right")[1]))
    assert sum(errors) / len(errors) <= 0.009, errors


def test_compensate_moves_a_fix_to_now():
    class Clock:
        t = 2.0

        def now(self):
            return self.t

    a = Approach(Clock(), "right")
    a.eff = 0.5
    # 0.5 to 1.5 s: 0.2 m/s forward; 1.5 to 2.0 s: 0.4 rad/s left turn.
    a._history = [(0.5, 0.2, 0.0, 0.0), (1.5, 0.0, 0.0, 0.4)]
    x, y = a.compensate(Fix(1.0, 0.0, 1.0))  # captured at 1.0 s: 0.5 s of the forward command remains
    x1 = 1.0 - 0.2 * 0.5 * 0.5
    th = -0.4 * 0.5 * 0.5
    assert x == pytest.approx(math.cos(th) * x1) and y == pytest.approx(math.sin(th) * x1)
    assert y < 0  # a left turn moves the ball to the right
    assert a.compensate(Fix(1.0, 0.3, 2.0)) == (1.0, 0.3)  # captured now: unchanged


def test_no_ball_searches_one_turn_then_stops():
    world = SimWorld(None, seed=8)
    out = run(world)
    assert not out.ok and "not found" in out.reason and out.no_ball
    assert world.cmd == (0.0, 0.0, 0.0)
    # 360 deg of commanded yaw (the walker turns 0.85 of it, plus the lag).
    assert math.radians(355) <= world.cmd_yaw <= math.radians(365), math.degrees(world.cmd_yaw)
    assert math.radians(290) <= abs(world.th) <= math.radians(330), math.degrees(world.th)
    assert math.hypot(world.X, world.Y) < 0.05  # turned in place


class Flicker(SimWorld):
    """No ball; a false positive at a random place in 1 of 8 frames (never 3 agreeing fixes)."""

    def step(self, dt):
        frame = self.next_frame
        super().step(dt)
        if self.next_frame != frame and self.rng.random() < 1 / 8:
            fake = Fix(self.rng.uniform(0.8, 2.5), self.rng.uniform(-1.0, 1.0), self.t)
            self.pending.append((self.t + self.latency, fake))
            self.pending.sort(key=lambda item: item[0])


def test_flickering_false_positive_does_not_shorten_the_search():
    world = Flicker(None, seed=9)
    out = run(world)
    assert not out.ok and "not found" in out.reason and out.no_ball, out.reason
    assert math.radians(355) <= world.cmd_yaw <= math.radians(365), math.degrees(world.cmd_yaw)
    assert sum("search: turn" in line for line in out.log) > 6  # the false positives broke some turns


def test_searches_share_one_yaw_budget():
    a = Approach(SimWorld(None, seed=10), "right")
    a.search_yaw = math.radians(300)
    a.t0 = 0.0
    with pytest.raises(ball_kick.NoBall, match="search budget"):
        asyncio.run(a.find())
    assert a.search_yaw == pytest.approx(P.search_yaw_max, abs=1e-6)


def test_ball_removed_during_the_approach_stops():
    world = SimWorld((2.5, 0.0), seed=9, remove_ball_at=3.0)
    out = run(world)
    assert not out.ok and ("lost" in out.reason or "not found" in out.reason or "not seen" in out.reason)
    assert out.no_ball
    assert world.cmd == (0.0, 0.0, 0.0)


def test_ball_removed_in_the_stop_and_look_stage_stops():
    world = SimWorld((1.0, 0.0), seed=10)
    # Find the start of the stage, then remove the ball there.
    probe = SimWorld((1.0, 0.0), seed=10)
    out = run(probe, "right")
    stage = next(line for line in out.log if "measure stage_far" in line)
    world.remove_ball_at = float(stage.split("s")[0]) - 1.0
    out = run(world, "right")
    assert not out.ok and out.no_ball and world.cmd == (0.0, 0.0, 0.0)


def test_stop_ends_the_approach_at_once():
    world = SimWorld((2.0, 0.0), seed=11, stop_at=2.0)
    out = run(world)
    assert not out.ok and out.reason == "stop requested" and not out.no_ball
    assert world.cmd == (0.0, 0.0, 0.0)
    assert world.t < 2.0 + 1.5  # including the settle in run()


def test_timeout_and_travel_budget():
    slow = ApproachParams(timeout_s=3.0)
    world = SimWorld((2.5, 0.0), seed=12)
    out = run(world, "right", slow)
    assert not out.ok and "timeout" in out.reason and not out.no_ball
    tight = ApproachParams(travel_max=0.5)
    world = SimWorld((2.5, 0.0), seed=13)
    out = run(world, "right", tight)
    assert not out.ok and "travel budget" in out.reason


def test_bad_side_is_refused():
    with pytest.raises(ValueError):
        asyncio.run(approach(SimWorld(), "middle"))


@pytest.mark.parametrize("approach_values,match", [
    ({"efficiency": 0}, "efficiency_bounds"),
    ({"efficiency_bounds": [0, 1.3]}, "efficiency_bounds"),
    ({"pulse_vx": -0.2}, "pulse_vx"),
    ({"approach_speed": 3.0}, "approach_speed"),
    ({"rate_hz": 0}, "rate_hz"),
    ({"timeout_s": 600}, "timeout_s"),
    ({"travel_max": 20}, "travel_max"),
    ({"stage_near": 0.46}, "stage_near"),
    ({"stage_far": 0.7}, "stage_far"),
    ({"tol_y": 0}, "tol_y"),
    ({"search_wz": 2.0}, "search_wz"),
])
def test_config_refuses_unsafe_values(approach_values, match):
    with pytest.raises(ValueError, match=match):
        config_from_mapping({"approach": approach_values})


def test_config_geometry_and_top_level_keys():
    with pytest.raises(ValueError, match="contact_x"):
        config_from_mapping({"geometry": {"contact_x": 0.6}})
    with pytest.raises(ValueError, match="snapshot_path"):
        config_from_mapping({"snapshot_path": "relative/dir"})
    with pytest.raises(TypeError, match="calibrated"):
        config_from_mapping({"calibrated": "yes"})
    cfg = config_from_mapping({"calibrated": True, "snapshot_path": "/tmp/kick", "dry_run": True,
                               "approach": {"stage_near": 0.51, "efficiency": 0.7}})
    assert cfg.calibrated and cfg.snapshot_path == "/tmp/kick" and cfg.dry_run
    assert cfg.approach.stage_near == 0.51 and cfg.approach.efficiency == 0.7
    default = config_from_mapping({})
    assert not default.calibrated and default.snapshot_path is None
    check_params(P, GEO)  # the defaults are valid


def test_invalid_params_stop_before_motion():
    world = SimWorld((1.5, 0.0), seed=14)
    out = run(world, "right", ApproachParams(pulse_vx=0.0))
    assert not out.ok and "invalid approach parameters" in out.reason and world.max_cmd == [0.0, 0.0, 0.0]


def test_short_pulses_respect_the_dead_time():
    a = Approach(SimWorld(), "right")
    a.dead = 0.2
    assert a.pulse_time(0.001, P.pulse_vy, 1.0) == pytest.approx(0.23)  # never a pulse that cannot move
    assert a.pulse_time(0.015, P.pulse_vy, 1.0) == pytest.approx(0.3)
    # A 0.4 s pulse at 0.12 m/s should move 0.024 m; it moved 0.002 m: the dead time goes up.
    assert a.learn_dead(0.4, 0.002, 0.12)
    assert 0.2 < a.dead <= P.dead_bounds[1]
    before = a.dead
    assert not a.learn_dead(0.6, 0.12 * (0.6 - before), 0.12)  # as expected: no change
    assert a.dead == before


def test_coarse_fixes_guide_the_walk_but_never_a_line_up():
    class CoarseWorld(SimWorld):
        async def poll(self):
            fix = self.delivered
            return None if fix is None else Fix(fix.x, fix.y, fix.stamp, precise=False)
    world = CoarseWorld((1.6, 0.0), seed=21)
    out = run(world, "right")
    assert not out.ok and out.no_ball and "not seen" in out.reason
    assert world.relative()[0] < 1.2  # it walked toward the ball first
    assert world.cmd == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("true_gain", [1.31, 1.2, 1.4])
def test_fixed_walker_model_lines_up_without_far_vision(true_gain):
    """calibrate 0 (robot, 2026-11-18): the camera fails beyond about 0.75 m, so the
    approach uses the walker model measured before (gain 1.31, dead 0.14 s) and the
    camera only near the ball. A 10 % model error must stay inside the kick tolerance."""
    params = ApproachParams(calibrate=0, efficiency=1.31, dead_time=0.14, efficiency_bounds=(0.45, 1.5),
                            dead_bounds=(-0.15, 0.25), cal_short_s=0.3, cal_long_min_s=0.6, stage_near=0.55)
    world = SimWorld((0.75, -0.1), eff=true_gain, eff_y=1.0, dead=0.14, seed=31)
    world.blind = 0.45
    out = run(world, "right", params)
    assert out.ok, out.reason + "\n" + "\n".join(out.log)
    ex, ey = kick_error(world, "right")
    assert -0.03 <= ex <= 0.04 and abs(ey) <= 0.02, (ex, ey, out.log)  # short is the safe side
    assert not any("calibration" in line or "stage_far" in line for line in out.log)
