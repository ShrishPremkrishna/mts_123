"""Closed-loop behavior with the leased executor, not mocked decisions."""
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


def test_first_step_pairs_decision_with_its_consumed_capture():
    sandbox = Sandbox()
    state = sandbox.control({"action": "step"})
    assert state["time"] == pytest.approx(.05)
    assert state["observation"]["captured_at"] == 0
    assert state["execution"]["frame_captured_at"] == 0
    assert state["execution"]["frame_seq"] == 1
    assert state["time"] - state["execution"]["frame_captured_at"] == pytest.approx(.05)
    assert sandbox.trace()["frames"][-1]["observation"] == state["observation"]


def test_paused_edits_and_frozen_packets_do_not_refresh_decision_evidence():
    sandbox = Sandbox()
    before = sandbox.control({"action": "step"})
    preview = sandbox.control({"action": "move_ball", "x": .4, "y": .8})
    assert preview["truth"]["ball"]["x"] == .4
    assert preview["observation"] == before["observation"]
    assert preview["decision"] == before["decision"]
    assert preview["execution"]["frame_captured_at"] == 0
    assert preview["execution"]["frame_seq"] == before["execution"]["frame_seq"]
    sandbox.control({"action": "configure", "settings": {"freeze": True}})
    sequence = before["execution"]["frame_seq"]
    for _ in range(3):
        frozen = sandbox.control({"action": "step"})
        assert frozen["execution"]["frame_seq"] == sequence
        assert frozen["execution"]["frame_captured_at"] == 0
        assert frozen["observation"]["captured_at"] == 0
    assert frozen["time"] - frozen["execution"]["frame_captured_at"] == pytest.approx(.2)
    sandbox.control({"action": "configure", "settings": {"freeze": False}})
    fresh = sandbox.control({"action": "step"})
    assert fresh["execution"]["frame_captured_at"] == pytest.approx(.2)
    assert fresh["execution"]["frame_seq"] > sequence


def test_paused_robot_edit_does_not_consume_same_capture_ball_velocity():
    baseline, edited = Sandbox("moving_ball"), Sandbox("moving_ball")
    for sandbox in (baseline, edited):
        sandbox.control({"action": "step"})
        sandbox.control({"action": "step"})
    prior = edited.snapshot()["observation"]
    edited.control({"action": "pause"})
    edited.control({"action": "move_robot", "x": -1, "y": -.5, "yaw": .1})
    assert edited.snapshot()["observation"] == prior
    a, b = baseline.control({"action": "step"}), edited.control({"action": "step"})
    assert a["observation"]["ball"]["vx"] > .1
    assert b["observation"]["ball"] == a["observation"]["ball"]


@pytest.mark.parametrize("settings", [{"ball_visible": False}, {"freeze": True}, {"delay_ms": 600}])
def test_perception_failures_zero_command_and_recover(settings):
    sandbox = Sandbox("wrong_side")
    sandbox.control({"action": "start"})
    for _ in range(4):
        sandbox.advance()
    sandbox.control({"action": "configure", "settings": settings})
    expected = "tactical_hold" if settings.get("ball_visible") is False else "tracking_lost"
    run_until(sandbox, lambda: sandbox.decision.phase == expected, 2)
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


def test_robot_reposition_cancels_active_strike_before_fresh_alignment():
    sandbox = Sandbox()
    sandbox.control({"action": "start"})
    run_until(sandbox, lambda: sandbox.decision.phase == "strike")
    assert sandbox.controller._strike is not None
    sandbox.control({"action": "move_robot", "x": -.55, "y": 0, "yaw": 0})
    assert sandbox.controller._strike is None
    sandbox.advance()
    assert sandbox.decision.phase != "strike"
    run_until(sandbox, lambda: sandbox.decision.phase == "await_reset")
    assert sandbox.metrics["goals"] == 1


def test_goal_restart_clears_delayed_previous_goal_after_limits_change():
    sandbox = Sandbox()
    sandbox.control({"action": "start"})
    run_until(sandbox, lambda: sandbox.decision.phase == "await_reset")
    sandbox.control({"action": "configure", "settings": {"delay_ms": 150, "max_speed": .5}})
    # The newly delayed sample may still show the final turn into home. Resume
    # waits for observed heading, not the previous tick's await_reset phase.
    run_until(sandbox, lambda: sandbox.snapshot()["execution"]["resume_available"], 2)
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
    assert sandbox.decision.phase == "tactical_hold"
    assert "fallen" in sandbox.decision.reason
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
    assert set(a.trace()["source_sha256"]) == {
        "model.py", "controller.py", "simulation.py", "runtime.py",
        "integration.py", "tactics.py", "obstacles.py"}
    json.dumps(a.trace(), allow_nan=False)


@pytest.mark.parametrize("control_request", [
    {"action": "configure", "settings": {"max_speed": 0.5, "noise_cm": -1}},
    {"action": "move_robot", "x": 0, "y": 0, "yaw": float("nan")},
    {"action": "move_ball", "x": 50, "y": 0},
    {"action": "fall", "value": "yes"},
    {"action": "reset", "seed": True},
    {"action": "reset", "scenario": "nope"},
    {"action": "reset", "attack_sign": 1.0},
    {"action": "resume"},
    {"action": "external_hold", "held": "yes"},
    {"action": "configure", "settings": {"renew_intent": 0}},
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


def command_is_zero(sandbox):
    command = sandbox.decision.command
    return (command.vx, command.vy, command.wz) == (0, 0, 0)


@pytest.mark.parametrize("sign", [-1, 1])
def test_automatic_defense_is_the_actual_executor_target(sign):
    sandbox = Sandbox(attack_sign=sign)
    sandbox.control({"action": "move_robot", "x": -sign * .2, "y": -.4,
                     "yaw": 0 if sign == 1 else math.pi})
    sandbox.control({"action": "move_ball", "x": -sign * 1.2, "y": .6})
    sandbox.control({"action": "start"})
    sandbox.advance()
    assert list(sandbox.executor.controllers) == ["pupper-1"]
    assert sandbox.decision.role == "defense"
    assert sandbox.decision.phase == "defend"
    assert sandbox.decision.target == pytest.approx((-sign * 1.48, .25))
    run_until(sandbox, lambda: sandbox.decision.phase == "target_reached", 20)
    assert math.hypot(sandbox.sim.robot.x + sign * 1.48, sandbox.sim.robot.y - .25) < .07
    assert sandbox.metrics["shots"] == 0
    assert sandbox.metrics["goals"] == 0


def test_lease_loss_zeroes_without_fallback_and_pause_does_not_renew():
    sandbox = Sandbox("offset")
    sandbox.control({"action": "start"})
    for _ in range(10):
        sandbox.advance()
    initial_epoch = sandbox.executor.epoch
    sandbox.control({"action": "pause"})
    before = sandbox.snapshot()["execution"]
    for _ in range(5):
        sandbox.advance()
        assert sandbox.snapshot()["execution"] == before
    sandbox.control({"action": "configure", "settings": {"renew_intent": False}})
    sandbox.control({"action": "start"})  # unpause, not another epoch or lease
    assert sandbox.executor.epoch == initial_epoch
    run_until(sandbox, lambda: sandbox.decision.phase == "intent_lost", 1)
    assert command_is_zero(sandbox)
    assert sandbox.snapshot()["execution"]["lease_remaining_s"] == 0
    for _ in range(10):
        sandbox.advance()
        assert sandbox.decision.phase == "intent_lost" and command_is_zero(sandbox)
    sandbox.control({"action": "configure", "settings": {"renew_intent": True}})
    sandbox.advance()
    assert sandbox.decision.role == "offense"
    assert sandbox.decision.phase != "intent_lost"
    assert sandbox.executor.epoch == initial_epoch


def test_execution_status_does_not_present_expired_retained_intent_as_active():
    sandbox = Sandbox()
    sandbox.control({"action": "step"})
    sandbox.control({"action": "configure", "settings": {"renew_intent": False}})
    for _ in range(10):
        boundary = sandbox.control({"action": "step"})
    intent = sandbox.executor.intents["pupper-1"]
    assert intent is not None  # last tick ran before expiry; physics has now reached it
    assert sandbox.sim.time >= intent.valid_until
    assert boundary["execution"]["lease_remaining_s"] == 0
    assert boundary["execution"]["action"] is None
    assert boundary["execution"]["intent_id"] is None
    assert boundary["execution"]["intent_valid_until"] == intent.valid_until


@pytest.mark.parametrize("sign", [-1, 1])
def test_hold_cancels_play_and_released_hold_still_needs_new_authority(sign):
    sandbox = Sandbox(attack_sign=sign)
    sandbox.control({"action": "start"})
    run_until(sandbox, lambda: sandbox.decision.phase == "align")
    sandbox.control({"action": "external_hold", "held": True})
    assert sandbox.decision.phase == "external_hold" and command_is_zero(sandbox)
    assert sandbox.decision.role == "offense"
    sandbox.control({"action": "configure", "settings": {"renew_intent": False}})
    for _ in range(14):
        sandbox.advance()
        assert sandbox.decision.phase == "external_hold" and command_is_zero(sandbox)
    sandbox.control({"action": "external_hold", "held": False})
    sandbox.advance()
    assert sandbox.decision.phase == "intent_lost" and command_is_zero(sandbox)
    sandbox.control({"action": "configure", "settings": {"renew_intent": True}})
    sandbox.advance()
    assert sandbox.decision.phase != "external_hold"
    run_until(sandbox, lambda: sandbox.decision.phase == "await_reset")
    assert sandbox.metrics["goals"] == 1


def test_role_change_is_recorded_when_tactical_hold_phase_does_not_change():
    sandbox = Sandbox()
    sandbox.control({"action": "configure", "settings": {"ball_visible": False}})
    sandbox.control({"action": "start"})
    sandbox.advance()
    assert sandbox.decision.phase == "tactical_hold" and sandbox.decision.role == "offense"
    changes = sandbox.metrics["phase_changes"]
    sandbox.control({"action": "move_ball", "x": -1.65, "y": 0})
    sandbox.control({"action": "configure", "settings": {"ball_visible": True}})
    sandbox.advance()
    assert sandbox.decision.phase == "tactical_hold" and sandbox.decision.role == "defense"
    assert sandbox.metrics["phase_changes"] == changes
    assert [(event["phase"], event["role"]) for event in list(sandbox.events)[-2:]] == [
        ("tactical_hold", "offense"), ("tactical_hold", "defense")]


@pytest.mark.parametrize("fault", ["relocate", "heading", "fallen", "stale", "lease", "hold"])
def test_refused_resume_never_replaces_ball_or_changes_authority(fault):
    sandbox = Sandbox()
    sandbox.control({"action": "start"})
    run_until(sandbox, lambda: sandbox.snapshot()["execution"]["resume_available"])
    sandbox.control({"action": "pause"})
    if fault in ("relocate", "heading"):
        robot = sandbox.sim.robot
        sandbox.control({"action": "move_robot", "x": 0 if fault == "relocate" else robot.x,
                         "y": .7 if fault == "relocate" else robot.y,
                         "yaw": 1.57})
    elif fault == "fallen":
        sandbox.control({"action": "fall", "value": True})
    elif fault == "stale":
        sandbox.control({"action": "configure", "settings": {"freeze": True}})
        for _ in range(9):
            sandbox.control({"action": "step"})
    elif fault == "lease":
        sandbox.control({"action": "configure", "settings": {"renew_intent": False}})
        for _ in range(12):
            sandbox.control({"action": "step"})
    else:
        sandbox.control({"action": "external_hold", "held": True})
    before = sandbox.snapshot()
    rng = sandbox.sim._random.getstate()
    history = tuple(sandbox.sim._history)
    assert not before["execution"]["resume_available"]
    with pytest.raises(ValueError):
        sandbox.control({"action": "resume"})
    assert sandbox.snapshot() == before
    assert sandbox.sim._random.getstate() == rng
    assert tuple(sandbox.sim._history) == history
    assert sandbox.sim.goal == 1


@pytest.mark.parametrize("sign", [-1, 1])
def test_combined_resume_consumes_fresh_replacement_without_renewing_lease(sign):
    sandbox = Sandbox(attack_sign=sign)
    sandbox.control({"action": "start"})
    run_until(sandbox, lambda: sandbox.snapshot()["execution"]["resume_available"])
    sandbox.control({"action": "pause"})
    intent, epoch = sandbox.executor.intents["pupper-1"], sandbox.executor.epoch
    prior_frame = sandbox.executor.frame_seq
    before = sandbox.snapshot()
    for _ in range(5):
        assert sandbox.snapshot() == before  # availability reads do not consume randomness/state
    sandbox.control({"action": "resume"})
    assert sandbox.executor.epoch == epoch
    assert sandbox.executor.intents["pupper-1"] == intent
    assert sandbox.executor.frame_seq > prior_frame
    assert sandbox.executor.resume_frame_seq["pupper-1"] == sandbox.executor.frame_seq
    assert sandbox.sim.goal == 0 and sandbox.sim.ball.x == sandbox.sim.ball.y == 0
    assert not sandbox.snapshot()["execution"]["resume_available"]
    with pytest.raises(ValueError):
        sandbox.control({"action": "resume"})
    run_until(sandbox, lambda: sandbox.decision.phase == "await_reset", 45)
    assert sandbox.metrics["goals"] == 2


@pytest.mark.parametrize("edit", ["robot", "fall"])
def test_edit_immediately_after_resume_cannot_use_previous_same_capture_pose(edit):
    sandbox = Sandbox()
    sandbox.control({"action": "start"})
    run_until(sandbox, lambda: sandbox.snapshot()["execution"]["resume_available"])
    sandbox.control({"action": "resume"})
    capture = sandbox.executor.frame[0]
    assert capture == sandbox.sim.time
    if edit == "robot":
        sandbox.control({"action": "move_robot", "x": 0, "y": .7, "yaw": 1.57})
    else:
        sandbox.control({"action": "fall", "value": True})
    sandbox.advance()
    assert sandbox.decision.phase == "tracking_lost"
    assert command_is_zero(sandbox)
    assert sandbox.executor.frame is None  # same enclosing capture is rejected, not re-stamped
    assert sandbox.snapshot()["execution"]["frame_captured_at"] is None
    sandbox.advance()
    assert sandbox.executor.frame[0] > capture
    if edit == "fall":
        assert sandbox.decision.recovery_requested and command_is_zero(sandbox)


def test_step_arms_only_the_initial_session_and_stop_requires_new_start():
    sandbox = Sandbox()
    assert not sandbox.snapshot()["execution"]["armed"]
    sandbox.control({"action": "step"})
    epoch = sandbox.executor.epoch
    sandbox.control({"action": "step"})
    assert sandbox.executor.epoch == epoch
    sandbox.control({"action": "external_hold", "held": True})
    sandbox.control({"action": "stop"})
    sandbox.control({"action": "start"})
    assert sandbox.executor.epoch != epoch
    assert sandbox.snapshot()["execution"]["external_hold"]
    sandbox.advance()
    assert sandbox.decision.phase == "external_hold" and command_is_zero(sandbox)
