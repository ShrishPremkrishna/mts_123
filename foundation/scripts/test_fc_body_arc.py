import math
import pytest
from foundation.scripts.fc_body_arc import Arc, Trials


def sample(vx=0.0, wz=0.0):
    data = [0.0] * 960
    data[6], data[8] = vx, wz
    return data


def prepare_and_run(trial):
    trial.observe(4, sample())
    prepared = trial.operator({"command": "prepare", "speed": 0.2, "radius": 1, "sweep_degrees": 35}, 4, None)
    run = {"command": "run", "token": prepared["token"]}
    assert trial.operator(run, 4, None)["event"] == "started"
    return run


@pytest.mark.parametrize("speed,radius,degrees", [(0.2, 1.4, 35), (0.3, 0.6, -80), (0.2, 1, 0.6)])
def test_arcs_integrate_to_expected_signed_displacement(speed, radius, degrees):
    arc = Arc(speed, radius, degrees)
    x = y = yaw = 0.0
    steps = 2000
    dt = arc.duration / steps
    for i in range(steps):
        vx, wz = arc.velocity((i + 0.5) * dt)
        assert 0 <= vx <= speed and abs(wz) <= 1
        x += vx * dt * math.cos(yaw + 0.5 * wz * dt)
        y += vx * dt * math.sin(yaw + 0.5 * wz * dt)
        yaw += wz * dt
    assert x == pytest.approx(arc.describe()["nominal_forward_m"], abs=2e-6)
    assert y == pytest.approx(arc.describe()["nominal_left_m"], abs=2e-6)
    assert yaw == pytest.approx(math.radians(degrees), abs=2e-6)
    assert arc.velocity(arc.duration) == (0, 0)


@pytest.mark.parametrize(
    "args",
    [
        (0.31, 1, 10),
        (0.3, 0.1, 10),
        (0.2, 10, 90),
        (0.001, 1, 45),
        (float("nan"), 1, 20),
        (0.2, float("inf"), 20),
        (0.3, 1, 91),
        (0.2, 0, 20),
        (0.2, 1, 0),
        (True, 1, 20),
    ],
)
def test_reject_over_contract_or_unbounded(args):
    with pytest.raises(ValueError):
        Arc(*args)


def test_camera_hold_cancels_and_cannot_restart():
    trial = Trials()
    old = prepare_and_run(trial)
    trial.tick(4, None)
    trial.observe(4.05, sample(0.015, 0.015))
    assert trial.tick(4.05, None)[0] > 0
    *cmd, event = trial.tick(4.1, "camera held")
    assert cmd[0:2] == [0.0, 0.0]
    assert (event["event"], event["reason"], event["curved_walker_feedback_seen"]) == (
        "cancelled",
        "camera held",
        False,
    )
    assert trial.tick(4.11, None)[3] is None
    trial.observe(4.12, sample())
    assert trial.tick(4.12, None)[3]["event"] == "walker_zero_observed"
    with pytest.raises(ValueError, match="token"):
        trial.operator(old, 4.12, None)
    assert trial.tick(4.14, None)[0:2] == (0.0, 0.0)


def test_gap_and_missing_zero_do_not_grant_second_arc():
    trial = Trials()
    prepare_and_run(trial)
    assert trial.tick(4.16, None)[3]["event"] == "cancelled"
    with pytest.raises(RuntimeError, match="zero unconfirmed"):
        trial.tick(5.2, None)


def test_finished_profile_requires_feedback_before_another_run():
    trial = Trials()
    prepare_and_run(trial)
    finish = 4 + trial.running.duration
    for tick in range(1, 32):
        now = 4 + tick * 0.02
        vx, wz, _, event = trial.tick(now, None)
        assert event is None
        trial.observe(now + 0.005, sample(vx, wz))
    assert trial.saw_curved_feedback
    trial.previous_tick = finish - 0.02
    trial.observe(finish - 0.02, sample(0.02, 0.02))
    assert trial.tick(finish, None)[3]["event"] == "profile_ended"
    with pytest.raises(ValueError):
        trial.operator(
            {"command": "prepare", "speed": 0.22, "radius": 1, "sweep_degrees": -30}, finish + 0.01, None
        )
    trial.observe(finish + 0.02, sample())
    assert trial.tick(finish + 0.02, None)[3]["event"] == "walker_zero_observed"
    assert (
        trial.operator(
            {"command": "prepare", "speed": 0.22, "radius": 1, "sweep_degrees": -30}, finish + 0.02, None
        )["event"]
        == "prepared"
    )


def test_token_expires_and_camera_invalidates_preparation():
    trial = Trials()
    msg = {"command": "prepare", "speed": 0.22, "radius": 1, "sweep_degrees": -30}
    trial.observe(10, sample())
    old = trial.operator(msg, 10, None)
    trial.tick(10.02, "blocked")
    with pytest.raises(ValueError, match="token"):
        trial.operator({"command": "run", "token": old["token"]}, 10.03, None)
    token = trial.operator(msg, 10.05, None)["token"]
    trial.observe(26, sample())
    with pytest.raises(ValueError, match="token"):
        trial.operator({"command": "run", "token": token}, 26, None)


@pytest.mark.parametrize("next_stamp,next_clearance", [(200_000_000, 150_000_000), (800_000_000, 10)])
def test_coalesced_clear_status_or_status_outage_revokes_even_if_camera_is_clear(next_stamp, next_clearance):
    trial = Trials()
    trial.guard_sample({"sampled_at_monotonic_ns": 100_000_000, "clearance_mono_ns": 10}, 0.3)
    old = prepare_and_run(trial)
    issue = trial.guard_sample(
        {"sampled_at_monotonic_ns": next_stamp, "clearance_mono_ns": next_clearance}, 0.3
    )
    assert issue and trial.tick(4.05, issue)[3]["event"] == "cancelled"
    trial.observe(4.06, sample())
    assert trial.tick(4.07, None)[3]["event"] == "walker_zero_observed"
    with pytest.raises(ValueError, match="token"):
        trial.operator(old, 4.07, None)


def test_unmatched_and_old_observations_do_not_establish_curve_or_zero():
    trial = Trials()
    prepare_and_run(trial)
    for i in range(1, 31):
        now = 4 + i * 0.02
        trial.tick(now, None)
        trial.observe(now, sample(0.02, 0.9))
    assert not trial.saw_curved_feedback
    trial.observe(3.99, sample(0.2, 0.2))
    assert not trial.saw_curved_feedback
    # A delayed publication from before stopping cannot close the attempt.
    trial.tick(4.61, "camera hold")
    trial.observe(4.60, sample())
    assert trial.tick(4.63, None)[3] is None
