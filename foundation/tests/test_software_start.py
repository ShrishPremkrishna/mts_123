"""Resident Start caller contracts without ROS, controller calls or hardware."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from foundation.hardware.software_start import ResidentStart, STOP_TOPIC, STOP_CONFIRM_TIMEOUT_S


def state(**changes):
    result = dict(protocol="pupper_stop_v1", instance_id="one", epoch=3, sequence=20,
                  selected_controller="neural_controller_default", latched=True,
                  software_stop_topic=STOP_TOPIC,
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
    stops = []
    caller = ResidentStart(submit, lambda: available[0], clock=lambda: now[0],
                           gateway_stop=lambda: stops.append('gateway'),
                           emergency_stop=lambda: stops.append('native'))
    caller.observe(state())
    return SimpleNamespace(caller=caller, now=now, available=available, calls=calls, futures=futures, stops=stops)


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


@pytest.mark.parametrize('capability', [None, False, True, '', '/other/stop'])
def test_old_or_wrong_gateway_capability_keeps_software_start_unavailable(rig, capability):
    assert rig.caller.observe(state(sequence=21, software_stop_topic=capability))
    with pytest.raises(ValueError, match='Options only'):
        rig.caller.capture()
    assert not rig.calls and not rig.stops


def test_stop_capability_and_transport_are_checked_again_at_send(rig):
    ticket = rig.caller.capture()
    rig.caller.observe(state(sequence=21, software_stop_topic=None))
    with pytest.raises(ValueError, match='Options only'):
        rig.caller.send(ticket)
    rig.caller.observe(state(sequence=22))
    rig.caller.gateway_stop = None
    with pytest.raises(ValueError, match='Options only'):
        rig.caller.send(ticket)
    assert not rig.calls


def test_stop_after_submitted_start_sends_both_paths_once_and_requires_new_latched_epoch(rig):
    c = rig.caller
    c.send(c.capture())
    c.observe('invalid status after possible remote Start')
    requested = c.request_stop('operator Stop')
    assert requested['instance_id'] == 'one' and requested['epoch'] == 3
    assert rig.stops == ['native', 'gateway'] and c.stop_pending
    assert c.poll_stop() is None
    rig.now[0] += .05
    assert c.request_stop('repeated Stop') == requested
    assert rig.stops == ['native', 'gateway']
    assert c.observe(state(sequence=21, epoch=4, latched=True, stop_required=True, in_flight=True))
    result = c.poll_stop()
    assert result['confirmed'] and not result['uncertain'] and not c.stop_pending
    assert result is c.stop_result and c.poll_stop() is None
    assert 'not backend deactivation' in result['scope']
    rig.futures[0].set_result(SimpleNamespace(success=True, message='late original response'))
    assert c.poll()['success']  # report the correlated ACK; local cancellation remains latched
    with pytest.raises(ValueError, match='local Stop'):
        c.capture()
    assert len(rig.calls) == 1


@pytest.mark.parametrize('changes', [dict(epoch=3), dict(epoch=4, latched=False),
    dict(epoch=4, instance_id='replacement'), dict(epoch=4, software_stop_topic='/other')])
def test_stop_confirmation_refuses_old_epoch_unlatched_foreign_or_wrong_capability(rig, changes):
    c = rig.caller
    c.send(c.capture())
    c.request_stop('stop')
    rig.now[0] += .05
    assert c.observe(state(sequence=21, **changes))
    assert c.poll_stop() is None
    rig.now[0] = 10 + STOP_CONFIRM_TIMEOUT_S
    result = c.poll_stop()
    assert not result['confirmed'] and result['uncertain'] and not c.stop_pending
    assert c.poll_stop() is None


def test_preexisting_latch_cannot_confirm_new_stop_or_extend_timeout(rig):
    c = rig.caller
    c.send(c.capture())
    c.observe(state(sequence=21, epoch=4, latched=True))
    request = c.request_stop('new Stop')
    assert request['epoch'] == 4
    assert c.poll_stop() is None
    rig.now[0] += .9
    c.request_stop('duplicate cannot restart timer')
    rig.now[0] += .11
    assert c.poll_stop()['uncertain']


def test_stop_without_start_anchors_observed_identity_and_can_confirm(rig):
    c = rig.caller
    c.request_stop('Stop before any Start')
    assert not rig.calls and rig.stops == ['native', 'gateway']
    rig.now[0] += .01
    c.observe(state(sequence=21, epoch=4))
    assert c.poll_stop()['confirmed']
    with pytest.raises(ValueError, match='local Stop'):
        c.send(SimpleNamespace())


@pytest.mark.parametrize('fault', ['missing_anchor', 'stale_ack', 'clock_rollback', 'native_publish_error'])
def test_stop_reports_uncertainty_instead_of_optimistic_confirmation(rig, fault):
    c = rig.caller
    if fault == 'missing_anchor':
        c.observe('invalid')
    if fault == 'native_publish_error':
        def failed():
            raise RuntimeError('native publisher failure')
        c.emergency_stop = failed
    c.request_stop('Stop')
    if fault == 'native_publish_error':
        assert rig.stops == ['gateway']  # second publication still attempted
    rig.now[0] += .01
    c.observe(state(sequence=21, epoch=4))
    if fault == 'stale_ack':
        rig.now[0] += STOP_CONFIRM_TIMEOUT_S
    elif fault == 'clock_rollback':
        rig.now[0] = 9.
    elif fault == 'missing_anchor':
        rig.now[0] += STOP_CONFIRM_TIMEOUT_S
    result = c.poll_stop()
    assert result['uncertain']
    assert result['confirmed'] is (fault == 'native_publish_error')
