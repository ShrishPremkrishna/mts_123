"""Closed-loop behavior with the production controller, not mocked decisions."""
import json
import math

import pytest

from final_project.pupper_fc.runtime import DT, TRACE_LIMIT, Sandbox


def run_until(sandbox, predicate, seconds=35):
    for _ in range(math.ceil(seconds / DT)):
        sandbox.advance()
        if predicate():
            return
    raise AssertionError(f"Timed out: {sandbox.decision}")


@pytest.mark.parametrize("scenario", ["center", "offset", "wrong_side", "moving_ball"])
@pytest.mark.parametrize("direction", [-1, 1])
def test_closed_loop_goal_and_restart_position(scenario, direction):
    sandbox = Sandbox(scenario=scenario, attack_sign=direction)
    sandbox.control({"action": "start"})
    run_until(sandbox, lambda: sandbox.decision.phase == "await_reset")
    assert sandbox.metrics["goals"] == 1
    assert sandbox.metrics["own_goals"] == 0
    assert sandbox.metrics["shots"] >= 1
    assert abs(sandbox.sim.robot.x + direction * sandbox.field.length * .3) < .07
    assert abs(sandbox.sim.robot.y) < .07


def test_stop_brakes_then_latches_until_explicit_start():
    sandbox = Sandbox("offset")
    sandbox.control({"action": "start"})
    for _ in range(12):
        sandbox.advance()
    assert abs(sandbox.sim.velocity.vx) + abs(sandbox.sim.velocity.vy) > .1
    sandbox.control({"action": "stop"})
    run_until(sandbox, lambda: not sandbox.settling, 2)
    at_rest = sandbox.sim.robot
    for _ in range(20):
        sandbox.advance()
    assert sandbox.sim.robot == at_rest
    assert sandbox.decision.phase == "stopped"
    with pytest.raises(ValueError, match="latched"):
        sandbox.control({"action": "step"})
    sandbox.control({"action": "start"})
    sandbox.advance()
    assert sandbox.decision.phase != "stopped"


def test_pause_freezes_simulation_time_and_step_advances_one_tick():
    sandbox = Sandbox()
    sandbox.control({"action": "start"})
    sandbox.advance()
    sandbox.control({"action": "pause"})
    before = sandbox.sim.snapshot()
    sandbox.advance()
    assert sandbox.sim.snapshot() == before
    sandbox.control({"action": "step"})
    assert sandbox.sim.time == pytest.approx(before["time"] + DT)
    assert not sandbox.running


@pytest.mark.parametrize("settings", [{"ball_visible": False}, {"freeze": True}, {"delay_ms": 600}])
def test_perception_failures_zero_command_and_recover(settings):
    sandbox = Sandbox("wrong_side")
    sandbox.control({"action": "start"})
    for _ in range(4):
        sandbox.advance()
    sandbox.control({"action": "configure", "settings": settings})
    run_until(sandbox, lambda: sandbox.decision.phase == "tracking_lost", 2)
    command = sandbox.decision.command
    assert (command.vx, command.vy, command.wz) == (0, 0, 0)
    for _ in range(20):
        sandbox.advance()
    assert abs(sandbox.sim.velocity.vx) + abs(sandbox.sim.velocity.vy) < 1e-6
    sandbox.control({"action": "configure", "settings": {
        "ball_visible": True, "freeze": False, "delay_ms": 0}})
    run_until(sandbox, lambda: sandbox.decision.phase == "await_reset")
    assert sandbox.metrics["goals"] == 1


def test_ball_relocation_changes_plan_on_next_packet():
    sandbox = Sandbox()
    sandbox.control({"action": "start"})
    sandbox.advance()
    before = sandbox.decision.target
    sandbox.control({"action": "move_ball", "x": .25, "y": -.5})
    sandbox.advance()
    assert sandbox.decision.target != before
    assert sandbox.observation.ball.y == -.5


def test_goal_restart_clears_delayed_previous_goal_after_limits_change():
    sandbox = Sandbox()
    sandbox.control({"action": "start"})
    run_until(sandbox, lambda: sandbox.decision.phase == "await_reset")
    sandbox.control({"action": "configure", "settings": {"delay_ms": 150, "max_speed": .5}})
    sandbox.control({"action": "resume"})
    sandbox.advance()
    assert sandbox.observation.goal == 0
    assert sandbox.decision.phase not in ("return_home", "await_reset")
    run_until(sandbox, lambda: sandbox.decision.phase == "await_reset", 45)
    assert sandbox.metrics["goals"] == 2


def test_fallen_is_explicit_fault_not_fake_getup():
    sandbox = Sandbox()
    sandbox.control({"action": "start"})
    sandbox.control({"action": "fall", "value": True})
    for _ in range(20):
        sandbox.advance()
    assert sandbox.decision.phase == "recovery_required"
    assert sandbox.decision.recovery_requested
    assert sandbox.sim.fallen
    assert sandbox.sim.velocity.vx == 0


def test_seeded_noise_is_reproducible_and_trace_is_finite():
    a, b = Sandbox("offset", seed=13), Sandbox("offset", seed=13)
    for sandbox in (a, b):
        sandbox.control({"action": "configure", "settings": {"noise_cm": 1, "delay_ms": 100}})
        sandbox.control({"action": "start"})
        for _ in range(100):
            sandbox.advance()
    assert a.trace() == b.trace()
    assert len(a.trace()["source_sha256"]) == 4
    json.dumps(a.trace(), allow_nan=False)


@pytest.mark.parametrize("control_request", [
    {"action": "configure", "settings": {"max_speed": 0.5, "noise_cm": -1}},
    {"action": "move_robot", "x": 0, "y": 0, "yaw": float("nan")},
    {"action": "move_ball", "x": 50, "y": 0},
    {"action": "fall", "value": "yes"},
    {"action": "reset", "seed": True},
    {"action": "reset", "scenario": "nope"},
    {"action": "resume"},
])
def test_invalid_control_leaves_session_unchanged(control_request):
    sandbox = Sandbox()
    before = sandbox.snapshot()
    with pytest.raises(ValueError):
        sandbox.control(control_request)
    assert sandbox.snapshot() == before


def test_trace_has_bounded_retention_and_honest_truncation():
    sandbox = Sandbox()
    for _ in range(TRACE_LIMIT + 2):
        sandbox.control({"action": "stop"})
    trace = sandbox.trace()
    assert len(trace["frames"]) == TRACE_LIMIT
    assert trace["truncated"]


def test_paused_edits_are_in_export_before_another_tick():
    sandbox = Sandbox()
    sandbox.control({"action": "move_ball", "x": .4, "y": .8})
    sandbox.control({"action": "configure", "settings": {"noise_cm": 2}})
    frame = sandbox.trace()["frames"][-1]
    assert frame["time"] == 0
    assert frame["truth"]["ball"]["x"] == .4
    assert frame["settings"]["noise_cm"] == 2
