"""Resident Start caller contracts without ROS, controller calls or hardware."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from foundation.hardware.software_start import ResidentStart


def state(**changes):
    result = dict(protocol="pupper_stop_v1", instance_id="one", epoch=3, sequence=20,
                  selected_controller="neural_controller_default", latched=True,
                  fault=False, stop_required=False, in_flight=False,
                  unknown_remote_submission=False, shutting_down=False)
    result.update(changes)
    return result


@pytest.fixture
def rig():
    now = [10.0]
    available = [True]
    calls, futures = [], []
    def submit(payload):
        calls.append(payload)
        future = Future()
        futures.append(future)
        return future
    caller = ResidentStart(submit, lambda: available[0], clock=lambda: now[0])
    caller.observe(state())
    return SimpleNamespace(caller=caller, now=now, available=available, calls=calls, futures=futures)


def test_status_and_input_capture_are_inert_until_explicit_submission(rig):
    ticket = rig.caller.capture()
    assert not rig.calls
    rig.caller.send(ticket)
    assert rig.calls == [dict(instance_id="one", epoch=3, sequence=20,
                             selected_controller="neural_controller_default")]
    assert rig.caller.poll() is None
    rig.futures[0].set_result(SimpleNamespace(success=True, message="acknowledged"))
    result = rig.caller.poll()
    assert result["success"] and not result["uncertain"]
    assert result["request"] == rig.calls[0]
    assert rig.caller.poll() is None
    with pytest.raises(ValueError, match="already submitted"):
        rig.caller.send(ticket)
    assert len(rig.calls) == 1


def test_newer_sequence_does_not_replace_frozen_press(rig):
    ticket = rig.caller.capture()
    rig.caller.observe(state(sequence=21))
    rig.caller.send(ticket)
    assert rig.calls[0]["sequence"] == 20


@pytest.mark.parametrize("change", [dict(epoch=4), dict(instance_id="restarted"),
                                  dict(selected_controller="neural_controller_student")])
def test_stop_restart_or_selection_after_press_is_not_followed(rig, change):
    ticket = rig.caller.capture()
    rig.caller.observe(state(sequence=21, **change))
    with pytest.raises(ValueError):
        rig.caller.send(ticket)
    assert not rig.calls


def test_stale_press_cannot_refresh_from_new_status(rig):
    ticket = rig.caller.capture()
    rig.now[0] += .201
    rig.caller.observe(state(sequence=21))
    with pytest.raises(ValueError, match="expired"):
        rig.caller.send(ticket)
    assert not rig.calls


def test_delayed_capture_retains_actual_input_receipt_and_cannot_borrow_later_status(rig):
    pressed_at = rig.now[0]
    rig.now[0] += .1
    ticket = rig.caller.capture(pressed_at=pressed_at)
    assert ticket.pressed_at == pressed_at
    rig.caller.observe(state(sequence=21))
    with pytest.raises(ValueError, match="not observed at Start input"):
        rig.caller.capture(pressed_at=pressed_at)
    rig.now[0] += .101
    with pytest.raises(ValueError, match="expired"):
        rig.caller.send(ticket)
    with pytest.raises(ValueError, match="receipt expired"):
        rig.caller.capture(pressed_at=pressed_at)
    assert not rig.calls


def test_missing_service_refuses_press_without_discovery_wait(rig):
    rig.available[0] = False
    with pytest.raises(ValueError, match="unavailable"):
        rig.caller.capture()
    assert not rig.calls
    rig.available[0] = True
    assert rig.caller.poll() is None
    assert not rig.calls


def test_service_loss_after_press_does_not_queue(rig):
    ticket = rig.caller.capture()
    rig.available[0] = False
    with pytest.raises(ValueError, match="unavailable"):
        rig.caller.send(ticket)
    assert not rig.calls


@pytest.mark.parametrize("change", [dict(fault=True), dict(stop_required=True),
    dict(in_flight=True), dict(unknown_remote_submission=True), dict(shutting_down=True),
    dict(protocol="wrong"), dict(epoch=True), dict(sequence=-1), dict(sequence=2**64),
    dict(latched=0), dict(instance_id=""), dict(selected_controller="other"), dict(fault="false")])
def test_bad_or_blocked_status_never_authorizes_start(rig, change):
    rig.caller.observe(state(sequence=21, **change) if "sequence" not in change else state(**change))
    with pytest.raises(ValueError):
        rig.caller.capture()
    assert not rig.calls


def test_reordered_duplicate_and_invalid_packets_do_not_refresh_status(rig):
    for packet in (state(), state(sequence=19), "not json", None, "[]", "null", "2", '"string"'):
        assert not rig.caller.observe(packet)
        with pytest.raises(ValueError, match="fresh"):
            rig.caller.capture()
    assert rig.caller.observe(state(sequence=21))
    rig.caller.send(rig.caller.capture())
    assert len(rig.calls) == 1


def test_completed_acknowledgement_is_reported_if_consumer_poll_runs_late(rig):
    rig.caller.send(rig.caller.capture())
    rig.now[0] += 1
    rig.futures[0].set_result(SimpleNamespace(success=True, message="acknowledged"))
    rig.now[0] += 4
    result = rig.caller.poll()
    assert result["success"] and not result["uncertain"]
    assert rig.caller.poll() is None and len(rig.calls) == 1


def test_old_observation_cannot_be_extended_by_press_or_newer_status(rig):
    rig.now[0] += .15
    ticket = rig.caller.capture()
    rig.now[0] += .1  # press only100ms old, frozen observation250ms old
    rig.caller.observe(state(sequence=21))
    with pytest.raises(ValueError, match="expired"):
        rig.caller.send(ticket)
    assert not rig.calls


def test_gateway_periodic_sequence_keeps_deliberate_start_available(rig):
    for seq in range(21, 121):
        rig.now[0] += .05  # actual gateway50ms timer increments sequence
        assert rig.caller.observe(state(sequence=seq))
    rig.now[0] += .08
    rig.caller.send(rig.caller.capture())
    assert rig.calls[0]["sequence"] == 120


def test_selected_running_controller_is_deliberately_ensure_active_not_autostart(rig):
    rig.caller.observe(state(sequence=21, latched=False))
    assert not rig.calls
    rig.caller.send(rig.caller.capture())
    assert len(rig.calls) == 1


def test_fault_between_capture_and_dispatch_refuses_without_submission(rig):
    ticket = rig.caller.capture()
    rig.caller.observe(state(sequence=21, fault=True))
    with pytest.raises(ValueError, match="fault"):
        rig.caller.send(ticket)
    assert not rig.calls


def test_clock_error_in_status_invalidates_readiness_without_escaping_callback(rig):
    def broken_clock():
        raise RuntimeError("clock unavailable")
    rig.caller.clock = broken_clock
    assert not rig.caller.observe(state(sequence=21))
    assert rig.caller.state is None and not rig.calls


@pytest.mark.parametrize("age", [-.001, .201, float("inf"), float("nan")])
def test_clock_rollback_stale_or_invalid_local_time_refuses(rig, age):
    rig.now[0] += age
    with pytest.raises(ValueError):
        rig.caller.capture()
    assert not rig.calls


def test_pending_is_single_call_and_timeout_never_retries_or_accepts_late_success(rig):
    rig.caller.send(rig.caller.capture())
    with pytest.raises(ValueError, match="pending"):
        rig.caller.capture()
    rig.now[0] += 4
    result = rig.caller.poll()
    assert result["uncertain"] and not result["success"]
    rig.futures[0].set_result(SimpleNamespace(success=True, message="late"))
    rig.caller.observe(state(sequence=21))
    assert rig.caller.poll() is None
    with pytest.raises(ValueError, match="uncertain"):
        rig.caller.capture()
    assert len(rig.calls) == 1


@pytest.mark.parametrize("exception", [True, False])
def test_exception_or_malformed_acknowledgement_is_uncertain(rig, exception):
    rig.caller.send(rig.caller.capture())
    if exception:
        rig.futures[0].set_exception(RuntimeError("lost transport"))
    else:
        rig.futures[0].set_result(SimpleNamespace(success=1, message="bad"))
    assert rig.caller.poll()["uncertain"]
    assert len(rig.calls) == 1


def test_submission_exception_is_not_retried(rig):
    def failed(payload):
        rig.calls.append(payload)
        raise RuntimeError("transport may have sent before raising")
    rig.caller.submit = failed
    with pytest.raises(ValueError, match="uncertain"):
        rig.caller.send(rig.caller.capture())
    assert rig.caller.poll() is None
    with pytest.raises(ValueError, match="uncertain"):
        rig.caller.capture()
    assert len(rig.calls) == 1


def test_refusal_requires_a_new_deliberate_press_for_another_attempt(rig):
    rig.caller.send(rig.caller.capture())
    rig.futures[0].set_result(SimpleNamespace(success=False, message="Stop held"))
    assert rig.caller.poll()["success"] is False
    rig.caller.observe(state(sequence=21))
    assert rig.caller.poll() is None and len(rig.calls) == 1
    rig.caller.send(rig.caller.capture())
    assert len(rig.calls) == 2
