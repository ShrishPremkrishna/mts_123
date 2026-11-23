"""Forecast/reachability contracts. Synthetic tracks; no physical motion is validated."""
import math
import random

import pytest

from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.model import Pose
from final_project.pupper_fc.tactics import (
    SYNTHETIC_PROFILE, BallForecast, BallTrack, MotionProfile, arrival_time, first_intercept, travel_time)

MEASURED = MotionProfile("measured:test-fixture", False, speed=.3, turn_rate=1., acceleration=.6,
                         braking=.6, reaction_s=.1)


def roll(track, x, y, vx, vy, *, decel=0., seconds=.6, dt=.05, noise=0., seed=3, start=10.):
    rng = random.Random(seed)
    speed = math.hypot(vx, vy)
    for k in range(round(seconds / dt) + 1):
        t = k * dt
        travelled = speed * t - .5 * decel * t * t if speed else 0.
        ux, uy = (vx / speed, vy / speed) if speed else (0., 0.)
        track.update(start + t, x + ux * travelled + rng.gauss(0, noise), y + uy * travelled + rng.gauss(0, noise))
    return start + k * dt


def test_still_ball_with_centimetre_noise_never_becomes_a_roll():
    track = BallTrack()
    for seed in range(20):
        track.reset()
        roll(track, .3, -.2, 0., 0., seconds=2., noise=.01, seed=seed)
        assert track.forecast(SYNTHETIC_PROFILE) is None


def test_rolling_fit_keeps_capture_identity_model_and_profile_source():
    track = BallTrack()
    last = roll(track, 0., .1, -.8, .1, noise=.002)
    forecast = track.forecast()
    assert forecast is not None and forecast.captured_at == pytest.approx(last)
    assert forecast.speed == pytest.approx(math.hypot(.8, .1), abs=.03)
    record = forecast.record(now=last + .07)
    assert record["age_s"] == pytest.approx(.07) and record["samples"] == forecast.samples >= 4
    assert record["deceleration_source"] == "unmeasured: constant velocity"
    assert "constant velocity" in record["model"]
    assert forecast.horizon_s <= 1.5
    synthetic = track.forecast(SYNTHETIC_PROFILE)
    assert synthetic.deceleration == SYNTHETIC_PROFILE.ball_deceleration
    assert synthetic.deceleration_source.startswith("synthetic ")


def test_crossing_respects_deceleration_and_direction():
    forecast = BallForecast(0., 0., 0., -1., .2, .5, "synthetic test", 2., .002, .01, 6, .3)
    t, y = forecast.crossing(-.5)
    assert forecast.position(t)[0] == pytest.approx(-.5)
    assert y == pytest.approx(.1)
    assert forecast.crossing(.5) is None          # moving away
    assert forecast.crossing(-1.5) is None        # stops after 1.02 m along the roll
    assert forecast.position(10.)[0] == pytest.approx(forecast.position(forecast.stop_time())[0])


def test_kick_or_relocation_restarts_the_history():
    track = BallTrack()
    last = roll(track, 0., 0., 0., 0., seconds=.5)
    track.update(last + .05, .4, 0.)
    assert len(track.samples) == 1 and track.forecast() is None


def test_moving_state_has_hysteresis_near_the_threshold():
    track = BallTrack()
    last = roll(track, 0., 0., .12, 0., seconds=.5)
    assert track.forecast() is not None
    track.reset()
    roll(track, 0., 0., .08, 0., seconds=.5)
    assert track.forecast() is None  # below the entry speed from rest
    track.reset()
    track.moving = True
    roll(track, 0., 0., .08, 0., seconds=.5)
    assert track.forecast() is not None  # above the exit speed once rolling


def test_profiles_name_sources_and_missing_values_stay_missing():
    assert MotionProfile("unmeasured", False).missing("intercept") == (
        "speed", "turn_rate", "acceleration", "braking", "reaction_s")
    assert MEASURED.missing("moving_contact") == ("kick_delay_s",)
    with pytest.raises(ValueError):
        MotionProfile("", False)
    with pytest.raises(ValueError):
        MotionProfile("bad", False, speed=-.1)
    with pytest.raises(ValueError, match="complete intercept profile"):
        arrival_time(MotionProfile("unmeasured", False), Pose(0, 0, 0), Pose(1, 0, 0))


def test_synthetic_profile_is_offline_only_in_the_executor():
    with pytest.raises(ValueError, match="offline-only"):
        Executor(motion_profile=SYNTHETIC_PROFILE)
    assert Executor(motion_profile=SYNTHETIC_PROFILE, allow_synthetic_profile=True).motion_profile.synthetic
    assert Executor(motion_profile=MEASURED).motion_profile is MEASURED


def test_trapezoid_and_concurrent_turn_arrival():
    assert travel_time(0., .3, .6, .6) == 0.
    # 0.3 m/s cap: 0.075 m ramps each way, then 0.85 m cruise.
    assert travel_time(1., .3, .6, .6) == pytest.approx(.5 + .5 + .85 / .3)
    assert travel_time(.05, .3, .6, .6) == pytest.approx(2 * math.sqrt(.05 / .6))
    assert arrival_time(MEASURED, Pose(0, 0, 0), Pose(0, 0, math.pi)) == pytest.approx(.1 + math.pi)


def test_first_intercept_is_earliest_reachable_and_respects_spread():
    forecast = BallForecast(0., 1., 0., -.4, 0., 0., "unmeasured: constant velocity", 1.5, .003, .005, 8, .4)

    def on_path(point, t):
        return Pose(point[0], point[1], 0.)

    robot = Pose(.3, .05, 0.)
    found = first_intercept(forecast, robot, .05, MEASURED, on_path)
    assert found is not None and found.arrival_s + .05 + .25 <= found.t + 1e-9
    earlier = forecast.position(found.t - .05)
    assert arrival_time(MEASURED, robot, Pose(*earlier, 0.)) + .05 + .25 > found.t - .05
    assert first_intercept(forecast, Pose(-1.4, 1., 0.), .05, MEASURED, on_path) is None
    wide = BallForecast(0., 1., 0., -.4, 0., 0., "unmeasured: constant velocity", 1.5, .1, .05, 8, .4)
    assert first_intercept(wide, robot, .05, MEASURED, on_path) is None
    assert first_intercept(forecast, robot, -.1, MEASURED, on_path) is None  # future capture
