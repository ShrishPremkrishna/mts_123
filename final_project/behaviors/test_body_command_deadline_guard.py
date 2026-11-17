"""Strict camera-guard source ordering and adapter wiring, without live ROS."""
from dataclasses import replace
import json
import sys
from types import ModuleType, SimpleNamespace

import pytest

from final_project.behaviors import body_command_guard as guard_module
from final_project.behaviors.body_command_guard import (
    BodyCommand, DeadlineBodyCommandGuard, ZERO,
)
from final_project.behaviors.command_deadline import (
    ADMITTED_TOPIC, CONTRACT, DeadlineCommand, REQUEST_TOPIC, decode,
)
from final_project.behaviors.test_body_command_guard import Timeline, obstacle


CLOCK = "linux-monotonic:test:1"
SOURCE = "a" * 32


class Lane:
    def __init__(self, *, arena=False):
        self.time = Timeline()
        self.guard = DeadlineBodyCommandGuard(clock_id=CLOCK, started_mono_ns=self.time.mono,
                                              arena_ball_contact=arena)
        self.guard.evaluate(**self.time.clocks())
        self.seq = 0

    def frame(self, **changes):
        return self.guard.receive_frame(json.dumps(self.time.frame(**changes)), **self.time.clocks())

    def clear(self):
        for _ in range(3):
            self.time.advance(.05)
            result = self.frame()
        return result

    def envelope(self, vx=.2, **changes):
        self.seq += 1
        return replace(DeadlineCommand(CLOCK, SOURCE, self.seq, self.time.mono,
                                       self.time.mono + 150_000_000, vx, 0., 0.), **changes)

    def send(self, envelope):
        return self.guard.receive_command(envelope.encode() if isinstance(envelope, DeadlineCommand)
                                          else envelope, **self.time.clocks())

    def passing(self):
        self.clear()
        self.time.advance(.001)
        assert self.send(self.envelope(0.)).status["state"] == "ready"
        self.time.advance(.001)
        source = self.envelope()
        assert self.send(source).command == BodyCommand(.2)
        return source


def test_start_closed_requires_genuine_zero_then_newer_source_motion():
    lane = Lane()
    status = lane.clear().status
    assert status["camera_permitted"] and status["requires_post_hold_zero"]
    lane.time.advance(.001)
    result = lane.send(lane.envelope())
    assert result.command == ZERO and result.envelope is None
    assert not result.status["source_bound"]
    lane.time.advance(.001)
    source_zero = lane.envelope(0.)
    result = lane.send(source_zero)
    assert result.status["state"] == "ready" and result.envelope == source_zero
    assert result.status["source_bound"] and not result.status["requires_post_hold_zero"]
    lane.time.advance(.001)
    result = lane.send(lane.envelope())
    assert result.command == BodyCommand(.2) and result.status["state"] == "passing"


def test_delayed_zero_and_nonzero_cannot_release_camera_hold_by_receipt_time():
    lane = Lane()
    original = lane.passing()
    lane.time.advance(.01)
    result = lane.frame(detections=[obstacle()])
    assert result.envelope == original.veto()
    lane.time.advance(.001)
    held = lane.envelope()
    assert lane.send(held).envelope == held.veto()
    # Three clear frames spaced tightly keep pre-clear origins unexpired, so
    # the test isolates original issue time rather than expiry.
    for _ in range(3):
        lane.time.advance(.01)
        result = lane.frame()
    clear_ns = result.status["clearance_mono_ns"]
    lane.time.advance(.001)
    old_zero = lane.envelope(0., issued_mono_ns=clear_ns - 1,
                             expires_mono_ns=clear_ns + 100_000_000)
    result = lane.send(old_zero)
    assert result.envelope == old_zero.veto() and result.status["requires_post_hold_zero"]
    lane.time.advance(.001)
    newer_held = lane.envelope()
    result = lane.send(newer_held)
    assert result.envelope == newer_held.veto()  # Guard veto cannot count as source zero.
    lane.time.advance(.001)
    zero = lane.envelope(0.)
    assert lane.send(zero).status["state"] == "ready"
    lane.time.advance(.001)
    motion = lane.envelope()
    assert lane.send(motion).envelope == motion


def test_duplicate_after_veto_never_undoes_it_or_refreshes_receipt():
    lane = Lane()
    original = lane.passing()
    first_receipt = lane.guard._receipt_ns
    lane.time.advance(.01)
    assert lane.frame(detections=[obstacle()]).envelope == original.veto()
    for _ in range(3):
        lane.time.advance(.01)
        lane.frame()
    lane.time.advance(.001)
    result = lane.send(original)
    assert result.command == ZERO and result.envelope == original.veto()
    assert lane.guard._receipt_ns == first_receipt
    assert result.status["requires_post_hold_zero"]


def test_cached_publications_and_camera_frames_cannot_extend_original_deadline():
    lane = Lane()
    original = lane.passing()
    for _ in range(2):
        lane.time.advance(.05)
        lane.frame()
        result = lane.send(original)
        assert result.envelope == original
    lane.time.advance(.05)
    result = lane.guard.evaluate(**lane.time.clocks())
    assert lane.time.mono == original.expires_mono_ns
    assert result.command == ZERO and result.envelope == original.veto()
    assert result.status["source_expires_mono_ns"] == original.expires_mono_ns
    assert result.status["source_age_s"] == .15 and result.status["input_age_s"] == .15
    assert result.status["input_state"] == "expired"
    lane.time.advance(.001)
    assert lane.send(original).envelope == original.veto()


def test_newer_zero_preempts_and_older_packet_cannot_replace_it():
    lane = Lane()
    original = lane.passing()
    lane.time.advance(.001)
    zero = lane.envelope(0.)
    assert lane.send(zero).envelope == zero
    lane.time.advance(.001)
    result = lane.send(original)
    assert result.envelope == zero and result.status["source_seq"] == zero.seq
    assert result.command == ZERO


@pytest.mark.parametrize("fault", ["malformed", "duplicate_key", "changed_deadline", "changed_velocity", "false_request"])
def test_bad_current_request_vetoes_previous_without_erasing_ordering(fault):
    lane = Lane()
    original = lane.passing()
    lane.time.advance(.001)
    payload = {"malformed": "{bad", "duplicate_key": '{"seq":2,' + original.encode()[1:],
               "changed_deadline": replace(original, expires_mono_ns=original.expires_mono_ns - 1),
               "changed_velocity": replace(original, vx=.1),
               "false_request": original.veto()}[fault]
    result = lane.send(payload)
    assert result.envelope == original.veto() and result.command == ZERO
    assert result.status["input_state"] == "invalid"
    assert lane.send(original).envelope == original.veto()
    lane.time.advance(.001)
    fresh = lane.envelope()
    assert lane.send(fresh).envelope == fresh


@pytest.mark.parametrize("change", [{"clock_id": "other"}, {"source_instance": "b" * 32},
                                    {"issued_mono_ns": 1, "expires_mono_ns": 2}])
def test_unrecognized_identity_or_prestartup_input_cannot_bind_or_refresh(change):
    lane = Lane()
    original = lane.passing()
    receipt = lane.guard._receipt_ns
    lane.time.advance(.001)
    result = lane.send(replace(lane.envelope(0.), **change))
    assert result.envelope == original and result.status["source_instance"] == SOURCE
    assert lane.guard._receipt_ns == receipt


def test_expired_higher_sequence_retires_motion_but_cannot_bind_initial_source():
    lane = Lane()
    original = lane.passing()
    lane.time.advance(.01)
    old = lane.envelope(0., issued_mono_ns=original.issued_mono_ns + 1,
                        expires_mono_ns=original.issued_mono_ns + 2)
    result = lane.send(old)
    assert result.envelope == old.veto() and result.status["source_seq"] == old.seq
    assert lane.send(original).envelope == old.veto()
    unbound = Lane()
    unbound.time.advance(.01)
    expired = unbound.envelope(0., issued_mono_ns=unbound.guard.started_mono_ns + 1,
                              expires_mono_ns=unbound.guard.started_mono_ns + 2)
    result = unbound.send(expired)
    assert result.envelope is None and not result.status["source_bound"]


def test_future_issue_and_nonincreasing_issue_veto_without_allowing_replay():
    lane = Lane()
    original = lane.passing()
    bad = lane.envelope(issued_mono_ns=lane.time.mono + 1,
                        expires_mono_ns=lane.time.mono + 100_000_000)
    assert lane.send(bad).envelope == original.veto()
    assert lane.send(original).envelope == original.veto()
    bad = lane.envelope(issued_mono_ns=original.issued_mono_ns,
                        expires_mono_ns=original.expires_mono_ns)
    result = lane.send(bad)
    assert result.envelope == bad.veto() and result.status["source_seq"] == bad.seq
    assert lane.send(original).envelope == bad.veto()
    lane.time.advance(.001)
    edited = replace(bad, issued_mono_ns=lane.time.mono, expires_mono_ns=lane.time.mono + 150_000_000)
    assert lane.send(edited).envelope == bad.veto()
    assert lane.send(lane.envelope()).status["state"] == "passing"


def test_restarted_guard_cannot_pass_old_source_zero_or_cached_motion():
    lane = Lane()
    original = lane.passing()
    lane.time.advance(.001)
    restarted = DeadlineBodyCommandGuard(clock_id=CLOCK, started_mono_ns=lane.time.mono)
    restarted.evaluate(**lane.time.clocks())
    result = restarted.receive_command(original.encode(), **lane.time.clocks())
    assert result.envelope is None and not result.status["source_bound"]
    result = restarted.receive_command(replace(original, vx=0.).encode(), **lane.time.clocks())
    assert result.envelope is None and result.status["requires_post_hold_zero"]


def test_source_zero_issued_at_clearance_is_not_after_clearance():
    lane = Lane()
    lane.clear()
    zero = lane.envelope(0.)
    result = lane.send(zero)
    assert result.envelope == zero.veto() and result.status["requires_post_hold_zero"]
    lane.time.advance(.001)
    assert lane.send(zero).envelope == zero.veto()
    assert lane.send(lane.envelope(0.)).status["state"] == "ready"


def test_clock_fault_and_shutdown_only_veto_original_without_new_identity():
    lane = Lane()
    original = lane.passing()
    lane.time.ros += guard_module.CLOCK_JUMP_NS + 1
    result = lane.guard.evaluate(**lane.time.clocks())
    assert result.envelope == original.veto() and result.status["state"] == "fault"
    lane = Lane()
    original = lane.passing()
    assert lane.guard.veto(**lane.time.clocks()).envelope == original.veto()
    assert lane.send(original).envelope == original.veto()


def test_arena_ball_exception_reuses_camera_core_and_person_still_vetoes():
    lane = Lane(arena=True)
    lane.passing()
    lane.time.advance(.01)
    assert lane.frame(detections=[obstacle("32")]).status["camera_permitted"]
    lane.time.advance(.01)
    result = lane.frame(detections=[obstacle("32"), obstacle("0")])
    assert result.command == ZERO and not result.envelope.allowed
    assert result.status["contact_policy"] == "arena_ball_contact"


def test_strict_ros_adapter_has_only_string_topics_and_republishes_original_deadline(monkeypatch):
    time = Timeline()
    publishers, subscriptions, timers = {}, {}, []

    class String:
        def __init__(self, data=""):
            self.data = data

    class Node:
        def __init__(self, name):
            pass

        def create_publisher(self, kind, topic, depth):
            assert kind is String and depth == 1
            publishers[topic] = []
            return SimpleNamespace(publish=publishers[topic].append)

        def create_subscription(self, kind, topic, callback, depth):
            assert kind is String and depth == 1
            subscriptions[topic] = callback

        def create_timer(self, period, callback, *, clock):
            timers.append(callback)
            assert clock.clock_type == "steady"

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=time.ros))

    for name, attrs in {
        "rclpy": {}, "rclpy.node": {"Node": Node},
        "rclpy.clock": {"Clock": lambda **kw: SimpleNamespace(**kw)},
        "rclpy.clock_type": {"ClockType": SimpleNamespace(STEADY_TIME="steady")},
        "std_msgs": {}, "std_msgs.msg": {"String": String},
    }.items():
        module = ModuleType(name)
        vars(module).update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    node = guard_module.create_ros_node(monotonic_ns=lambda: time.mono,
                                       command_contract=CONTRACT, clock_id=CLOCK)
    assert set(publishers) == {ADMITTED_TOPIC, guard_module.STATUS_TOPIC}
    assert set(subscriptions) == {REQUEST_TOPIC, guard_module.FRAME_TOPIC}
    assert not publishers[ADMITTED_TOPIC]
    for _ in range(3):
        time.advance(.05)
        subscriptions[guard_module.FRAME_TOPIC](String(json.dumps(time.frame())))
    time.advance(.001)
    zero = DeadlineCommand(CLOCK, SOURCE, 1, time.mono, time.mono + 150_000_000, 0., 0., 0.)
    subscriptions[REQUEST_TOPIC](String(zero.encode()))
    time.advance(.001)
    motion = DeadlineCommand(CLOCK, SOURCE, 2, time.mono, time.mono + 150_000_000, .2, 0., 0.)
    subscriptions[REQUEST_TOPIC](String(motion.encode()))
    time.advance(.02)
    timers[0]()
    assert decode(publishers[ADMITTED_TOPIC][-1].data) == motion
    node.publish_zero()
    assert decode(publishers[ADMITTED_TOPIC][-1].data) == motion.veto()
    assert json.loads(publishers[guard_module.STATUS_TOPIC][0].data)["command_contract"] == CONTRACT


def test_unknown_mode_is_rejected_before_ros_import():
    with pytest.raises(ValueError, match="contract"):
        guard_module.create_ros_node(command_contract="ambiguous")


def test_false_request_cannot_bind_genuine_zero_or_satisfy_handshake():
    lane = Lane()
    lane.clear()
    lane.time.advance(.001)
    false_zero = lane.envelope(0., allowed=False)
    result = lane.send(false_zero)
    assert result.envelope is None and not result.status["source_bound"]
    assert result.status["requires_post_hold_zero"] and result.status["input_state"] == "invalid"


def test_cli_strict_selection_keeps_ros_arguments(monkeypatch):
    calls = {}
    ros = ModuleType("rclpy")
    ros.init = lambda **kw: calls.update(init=kw)
    ros.spin = lambda node: None
    ros.ok = lambda: False
    ros.try_shutdown = lambda: None
    executors = ModuleType("rclpy.executors")
    executors.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
    implementation = ModuleType("rclpy.impl.implementation_singleton")
    implementation.rclpy_implementation = SimpleNamespace(RCLError=RuntimeError)
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    monkeypatch.setitem(sys.modules, "rclpy.executors", executors)
    monkeypatch.setitem(sys.modules, "rclpy.impl.implementation_singleton", implementation)

    def factory(**kwargs):
        calls["factory"] = kwargs
        return SimpleNamespace(destroy_node=lambda: None)

    monkeypatch.setattr(guard_module, "create_ros_node", factory)
    guard_module.main(["--command-contract", CONTRACT, "--arena-ball-contact",
                       "--ros-args", "-r", "__ns:=/strict"])
    assert calls["factory"] == dict(arena_ball_contact=True, command_contract=CONTRACT)
    assert calls["init"] == dict(args=["--ros-args", "-r", "__ns:=/strict"])


@pytest.fixture(params=[guard_module.LEGACY_CONTRACT, CONTRACT])
def shutdown_main(monkeypatch, request):
    """Exercise the CLI exception boundary for either unchanged command path."""
    class RCLError(RuntimeError):
        pass

    class ExternalShutdownException(Exception):
        pass

    state = SimpleNamespace(live=True, spin_error=None, spin_shutdown=False,
                            publish_error=None, publish_shutdown=False, calls=[],
                            RCLError=RCLError, ExternalShutdownException=ExternalShutdownException)

    def spin(node):
        state.calls.append("spin")
        if state.spin_shutdown:
            state.live = False
        if state.spin_error is not None:
            raise state.spin_error

    def publish_zero():
        state.calls.append("publish_zero")
        if state.publish_shutdown:
            state.live = False
        if state.publish_error is not None:
            raise state.publish_error

    for name, attrs in {
        "rclpy": {"init": lambda **kw: None, "spin": spin, "ok": lambda: state.live,
                  "try_shutdown": lambda: state.calls.append("shutdown")},
        "rclpy.executors": {"ExternalShutdownException": ExternalShutdownException},
        "rclpy.impl.implementation_singleton": {
            "rclpy_implementation": SimpleNamespace(RCLError=RCLError)},
    }.items():
        module = ModuleType(name)
        vars(module).update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    node = SimpleNamespace(publish_zero=publish_zero,
                           destroy_node=lambda: state.calls.append("destroy"))
    monkeypatch.setattr(guard_module, "create_ros_node", lambda **kw: node)
    state.run = lambda: guard_module.main(["--command-contract", request.param])
    return state


@pytest.mark.parametrize("interrupted", [False, True])
def test_live_normal_shutdown_attempts_final_zero(shutdown_main, interrupted):
    seam = shutdown_main
    if interrupted:
        seam.spin_error = KeyboardInterrupt()
    seam.run()
    assert seam.calls == ["spin", "publish_zero", "destroy", "shutdown"]


def test_external_shutdown_skips_final_publication(shutdown_main):
    seam = shutdown_main
    seam.spin_shutdown = True
    seam.spin_error = seam.ExternalShutdownException()
    seam.run()
    assert seam.calls == ["spin", "destroy", "shutdown"]


def test_context_shutdown_during_callback_publication_exits_cleanly(shutdown_main):
    seam = shutdown_main
    seam.spin_shutdown = True
    seam.spin_error = seam.RCLError("publisher's context is invalid")
    seam.run()
    assert seam.calls == ["spin", "destroy", "shutdown"]


def test_context_shutdown_between_cleanup_check_and_publication_exits_cleanly(shutdown_main):
    seam = shutdown_main
    seam.publish_shutdown = True
    seam.publish_error = seam.RCLError("publisher's context is invalid")
    seam.run()
    # The attempt failed; the test does not treat it as a delivered zero.
    assert seam.calls == ["spin", "publish_zero", "destroy", "shutdown"]


def test_live_callback_rcl_error_propagates(shutdown_main):
    seam = shutdown_main
    seam.spin_error = seam.RCLError("unexpected live publication failure")
    with pytest.raises(seam.RCLError) as caught:
        seam.run()
    assert caught.value is seam.spin_error
    assert seam.calls == ["spin", "publish_zero", "destroy", "shutdown"]


@pytest.mark.parametrize("shutdown", [False, True])
def test_unrelated_callback_error_always_propagates(shutdown_main, shutdown):
    seam = shutdown_main
    seam.spin_shutdown = shutdown
    seam.spin_error = ValueError("unexpected callback failure")
    with pytest.raises(ValueError) as caught:
        seam.run()
    assert caught.value is seam.spin_error
    assert seam.calls[-2:] == ["destroy", "shutdown"]


def test_live_cleanup_rcl_error_propagates_after_resource_cleanup(shutdown_main):
    seam = shutdown_main
    seam.publish_error = seam.RCLError("unexpected live cleanup failure")
    with pytest.raises(seam.RCLError) as caught:
        seam.run()
    assert caught.value is seam.publish_error
    assert seam.calls == ["spin", "publish_zero", "destroy", "shutdown"]


def test_unrelated_cleanup_error_propagates_even_after_context_shutdown(shutdown_main):
    seam = shutdown_main
    seam.publish_shutdown = True
    seam.publish_error = ValueError("unexpected cleanup failure")
    with pytest.raises(ValueError) as caught:
        seam.run()
    assert caught.value is seam.publish_error
    assert seam.calls[-2:] == ["destroy", "shutdown"]


@pytest.mark.parametrize("shutdown", [False, True])
def test_original_live_error_survives_second_cleanup_error(shutdown_main, shutdown):
    seam = shutdown_main
    seam.spin_error = seam.RCLError("original live callback failure")
    seam.publish_shutdown = shutdown
    seam.publish_error = seam.RCLError("second cleanup failure")
    with pytest.raises(seam.RCLError) as caught:
        seam.run()
    assert caught.value is seam.spin_error
    if not shutdown:
        assert caught.value.__cause__ is seam.publish_error
    assert seam.calls[-2:] == ["destroy", "shutdown"]
