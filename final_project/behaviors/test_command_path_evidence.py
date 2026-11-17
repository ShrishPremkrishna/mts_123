"""Strict source binding and endpoint evidence; no live ROS graph is used."""
import json
from types import SimpleNamespace

import pytest

from final_project.behaviors.command_deadline import ADMITTED_TOPIC, CONTRACT, REQUEST_TOPIC
from final_project.behaviors.command_path_evidence import (
    GUARD_STATUS_TOPIC, NATIVE_AUTHORITY_TOPIC, decode_guard_status, decode_native_authority,
    guard_publishers_problem, guard_status_problem, native_authority_problem,
)


CLOCK, SOURCE = "linux-monotonic:test:1", "a" * 32


def status(**changes):
    row = dict(state="ready", camera_state="clear", camera_permitted=True, input_state="valid",
               requires_post_hold_zero=False, clear_frames=3, reason="ready", frame_age_s=.05,
               max_frame_age_s=.35, producer_instance="camera", sequence=1,
               sampled_at_monotonic_ns=10_000_000_000, command_contract=CONTRACT,
               clock_id=CLOCK, source_bound=True, source_instance=SOURCE, source_seq=2,
               source_issued_mono_ns=9_990_000_000, source_expires_mono_ns=10_140_000_000,
               source_vetoed=False, clearance_mono_ns=9_900_000_000, post_clear_zero_seq=1)
    row.update(changes)
    return row


def parse(row):
    return decode_guard_status(json.dumps(row), 10., command_contract=CONTRACT,
                               clock_id=CLOCK, source_instance=SOURCE)


def authority(**changes):
    row = dict(schema="pupper_command_authority_v1", clock_id=CLOCK, generation=2,
               activation_mono_ns=9_000_000_000, bound_source_instance=SOURCE,
               sampled_mono_ns=10_000_000_000)
    row.update(changes)
    return row


@pytest.mark.parametrize("changes", [
    {"schema": "wrong"}, {"clock_id": "wrong"}, {"generation": True}, {"generation": 0},
    {"generation": 2**64}, {"activation_mono_ns": 0}, {"activation_mono_ns": 10_000_000_001},
    {"sampled_mono_ns": 10_000_000_001}, {"sampled_mono_ns": 9_699_999_999},
    {"sampled_mono_ns": 10.}, {"bound_source_instance": None}, {"bound_source_instance": "x" * 32},
    {"extra": 1},
])
def test_native_authority_rejects_malformed_stale_future_or_foreign_clock(changes):
    with pytest.raises(ValueError):
        decode_native_authority(json.dumps(authority(**changes)), 10., CLOCK)


@pytest.mark.parametrize("changes", [
    {"generation": 1}, {"activation_mono_ns": 8_000_000_000},
    {"sampled_mono_ns": 9_999_999_999}, {"activation_mono_ns": 9_100_000_000},
    {"bound_source_instance": ""}, {"bound_source_instance": "b" * 32},
    {"generation": 3, "activation_mono_ns": 9_500_000_000},
])
def test_native_authority_cannot_regress_or_change_existing_sample(changes):
    with pytest.raises(ValueError):
        decode_native_authority(json.dumps(authority(**changes)), 10., CLOCK, authority())


def test_native_authority_duplicates_do_not_refresh_original_age_and_activation_can_rebind():
    original = authority()
    parsed = decode_native_authority(json.dumps(original), 10.2, CLOCK, original)
    assert native_authority_problem(parsed, 10.2, None, 10.2, SOURCE) is None
    assert "sample stale" in native_authority_problem(parsed, 10.2, None, 10.31, SOURCE)
    replacement = authority(generation=3, activation_mono_ns=10_200_000_000,
                            sampled_mono_ns=10_200_000_000, bound_source_instance="")
    unbound = decode_native_authority(json.dumps(replacement), 10.2, CLOCK, original)
    assert "pending" in native_authority_problem(unbound, 10.2, None, 10.2, SOURCE)
    replacement.update(sampled_mono_ns=10_201_000_000, bound_source_instance=SOURCE)
    bound = decode_native_authority(json.dumps(replacement), 10.201, CLOCK, unbound)
    assert native_authority_problem(bound, 10.201, None, 10.201, SOURCE) is None


@pytest.mark.parametrize("change", [
    {"command_contract": "legacy_twist"}, {"clock_id": "other"},
    {"source_instance": "b" * 32}, {"source_bound": 1}, {"source_vetoed": 0},
    {"source_seq": True}, {"source_seq": 0}, {"source_seq": 2**63},
    {"source_issued_mono_ns": 10_000_000_001}, {"source_expires_mono_ns": 10_140_000_001},
    {"source_expires_mono_ns": 10_000_000_000}, {"post_clear_zero_seq": 3},
    {"post_clear_zero_seq": None}, {"clearance_mono_ns": None},
    {"clearance_mono_ns": 10_000_000_001}, {"clearance_mono_ns": 9_990_000_000},
    {"source_vetoed": True}, {"requires_post_hold_zero": True},
])
def test_strict_status_rejects_wrong_binding_or_impossible_handshake(change):
    with pytest.raises(ValueError):
        parse(status(**change))


def test_unbound_status_is_visible_for_start_but_cannot_admit_arm():
    row = status(state="hold", source_bound=False, source_instance=None, source_seq=None,
                 source_issued_mono_ns=None, source_expires_mono_ns=None, source_vetoed=True,
                 post_clear_zero_seq=None, requires_post_hold_zero=True)
    parsed, _, deadline = parse(row)
    assert guard_status_problem(parsed, 10., None, deadline, 10., camera=False,
                                command_contract=CONTRACT) is None
    assert guard_status_problem(parsed, 10., None, deadline, 10., command_contract=CONTRACT)[0] == "camera"
    with pytest.raises(ValueError):
        parse({**row, "state": "ready"})


def test_source_deadline_status_is_not_a_new_motion_lease_or_timer_based_false_hold():
    parsed, _, deadline = parse(status())
    # Status is a sampled binding/handshake observation. Source envelopes are
    # independently renewed and expired at native; the 200 ms status cadence
    # must not create a false hold whenever its last sampled 150 ms lease ends.
    assert guard_status_problem(parsed, 10., None, deadline, 10.16, command_contract=CONTRACT) is None
    assert guard_status_problem(parsed, 10., None, deadline, 10.31, command_contract=CONTRACT)[0] == "path"


def test_actual_guard_status_crosses_startup_hold_and_fresh_zero_handshake():
    from final_project.behaviors.test_body_command_deadline_guard import Lane
    from final_project.behaviors.test_body_command_guard import obstacle
    lane = Lane()
    source = deadline = None

    def problem(result):
        nonlocal source, deadline
        now = lane.time.mono / 1e9
        parsed, source, deadline = decode_guard_status(json.dumps(result.status), now, source, deadline,
            command_contract=CONTRACT, clock_id=CLOCK, source_instance=SOURCE)
        return guard_status_problem(parsed, now, None, deadline, now, command_contract=CONTRACT)

    assert problem(lane.clear())[0] == "camera"
    lane.time.advance(.001)
    assert problem(lane.send(lane.envelope(0.))) is None
    lane.time.advance(.001)
    assert problem(lane.send(lane.envelope())) is None
    lane.time.advance(.001)
    assert problem(lane.frame(detections=[obstacle()]))[0] == "camera"
    assert problem(lane.clear())[0] == "camera"
    lane.time.advance(.001)
    assert problem(lane.send(lane.envelope(0.))) is None


def graph():
    endpoint = lambda name, kind="std_msgs/msg/String": SimpleNamespace(
        node_name=name, node_namespace="/", topic_type=kind)
    pubs = {REQUEST_TOPIC: [endpoint("fc_executor_bridge")],
            ADMITTED_TOPIC: [endpoint("fc_body_command_guard")],
            GUARD_STATUS_TOPIC: [endpoint("fc_body_command_guard")],
            NATIVE_AUTHORITY_TOPIC: [endpoint("neural_controller_default")]}
    subs = {REQUEST_TOPIC: [endpoint("fc_body_command_guard")],
            ADMITTED_TOPIC: [endpoint("neural_controller_default")]}
    node = SimpleNamespace(get_publishers_info_by_topic=lambda topic: pubs.get(topic, []),
                           count_publishers=lambda topic: len(pubs.get(topic, [])),
                           get_subscriptions_info_by_topic=lambda topic: subs.get(topic, []))
    return node, pubs, subs, endpoint


def test_strict_graph_checks_real_receivers_but_allows_read_only_observers():
    node, pubs, subs, endpoint = graph()
    assert guard_publishers_problem(node, "/fc_executor_bridge", command_contract=CONTRACT) is None
    subs[ADMITTED_TOPIC].append(endpoint("trace_observer"))
    assert guard_publishers_problem(node, "/fc_executor_bridge", command_contract=CONTRACT) is None
    subs["/cmd_vel"] = [endpoint("neural_controller_default", "geometry_msgs/msg/Twist")]
    assert guard_publishers_problem(node, "/fc_executor_bridge", command_contract=CONTRACT)[0] == "authority"


@pytest.mark.parametrize("fault", ["missing_guard", "competing_native", "wrong_native_type", "native_twist"])
def test_pending_native_activation_never_excuses_guard_or_competing_endpoint(fault):
    node, pubs, subs, endpoint = graph()
    pubs[NATIVE_AUTHORITY_TOPIC] = []
    subs[ADMITTED_TOPIC] = []
    check = lambda: guard_publishers_problem(node, "/fc_executor_bridge", command_contract=CONTRACT,
                                             allow_absent_native=True)
    assert check() is None
    if fault == "missing_guard":
        pubs[ADMITTED_TOPIC] = []
    elif fault == "competing_native":
        pubs[NATIVE_AUTHORITY_TOPIC] = [endpoint("foreign")]
    elif fault == "wrong_native_type":
        pubs[NATIVE_AUTHORITY_TOPIC] = [endpoint("neural_controller_default", "geometry_msgs/msg/Twist")]
    else:
        subs["/cmd_vel"] = [endpoint("neural_controller_default", "geometry_msgs/msg/Twist")]
    assert check()[0] == ("path" if fault == "missing_guard" else "authority")


@pytest.mark.parametrize("fault", ["rogue", "duplicate", "wrong_type", "wrong_native_authority",
                                  "missing_guard", "missing_native", "missing_native_authority"])
def test_strict_graph_rejects_ambiguous_or_missing_path(fault):
    node, pubs, subs, endpoint = graph()
    if fault == "rogue":
        pubs[REQUEST_TOPIC] = [endpoint("another_bridge")]
    elif fault == "duplicate":
        pubs[ADMITTED_TOPIC].append(endpoint("fc_body_command_guard"))
    elif fault == "wrong_type":
        pubs[REQUEST_TOPIC][0].topic_type = "geometry_msgs/msg/Twist"
    elif fault == "wrong_native_authority":
        pubs[NATIVE_AUTHORITY_TOPIC][0].node_name = "fake_controller"
    elif fault == "missing_native_authority":
        pubs[NATIVE_AUTHORITY_TOPIC] = []
    elif fault == "missing_guard":
        subs[REQUEST_TOPIC] = []
    else:
        subs[ADMITTED_TOPIC] = []
    problem = guard_publishers_problem(node, "/fc_executor_bridge", command_contract=CONTRACT)
    assert problem[0] == ("path" if fault.startswith("missing") else "authority")
