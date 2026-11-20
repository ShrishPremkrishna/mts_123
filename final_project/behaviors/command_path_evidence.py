"""Shared guarded-path observations, not DDS authentication or motion authority."""
from __future__ import annotations

import json
import math

GUARD_STATUS_TOPIC = "/fc/body_command_guard/status"
GUARD_INPUT_TOPIC = "/fc/requested_cmd_vel"
GUARD_STATUS_MAX_AGE_S = 0.3
GUARD_FRAME_MAX_AGE_S = 0.35
NATIVE_AUTHORITY_TOPIC = "/neural_controller_default/command_authority"
NATIVE_AUTHORITY_MAX_AGE_S = 0.3


def guard_publishers_problem(node, client_name, *, robot_topic="/cmd_vel", kick_topic="/kick_cmd_vel",
                             command_contract="legacy_twist", allow_absent_native=False,
                             native_node="/neural_controller_default", detector_node=None):
    if command_contract == "fc_deadline_v1":
        return _deadline_graph_problem(node, client_name, allow_absent_native=allow_absent_native,
                                       native=native_node, detector=detector_node)
    if command_contract != "legacy_twist":
        return "authority", "unknown command contract"
    expected = {robot_topic: ["/fc_body_command_guard", client_name],
                GUARD_INPUT_TOPIC: ["/cmd_vel_mux"], kick_topic: [client_name],
                GUARD_STATUS_TOPIC: ["/fc_body_command_guard"]}
    try:
        missing = None
        for topic, wanted in expected.items():
            infos = node.get_publishers_info_by_topic(topic)
            actual = [info.node_namespace.rstrip("/") + "/" + info.node_name for info in infos]
            count = node.count_publishers(topic)
            if sorted(actual) != sorted(wanted) or count != len(wanted):
                competing = count > len(wanted) or any(
                    name not in wanted or actual.count(name) > wanted.count(name) for name in actual)
                if competing:
                    return "authority", "unexpected publishers on " + topic
                missing = missing or ("path", "unexpected publishers on " + topic)
    except (AttributeError, RuntimeError):
        return "path", "guard publisher identities unavailable"
    return missing


def _deadline_graph_problem(node, client_name, *, allow_absent_native=False, native="/neural_controller_default",
                            detector=None):
    # Import only for strict selection; legacy source subsets need no helper.
    from final_project.behaviors.command_deadline import REQUEST_TOPIC, ADMITTED_TOPIC
    guard = "/fc_body_command_guard"
    publishers = {REQUEST_TOPIC: client_name, ADMITTED_TOPIC: guard, GUARD_STATUS_TOPIC: guard,
                  NATIVE_AUTHORITY_TOPIC: native}
    if detector is not None:
        publishers["/fc/obstacle_detections"] = detector
    subscribers = {REQUEST_TOPIC: guard, ADMITTED_TOPIC: native}
    name = lambda info: info.node_namespace.rstrip("/") + "/" + info.node_name
    try:
        missing = None
        for topic, owner in publishers.items():
            infos = node.get_publishers_info_by_topic(topic)
            count = node.count_publishers(topic)
            actual = [name(info) for info in infos]
            if any(value != owner for value in actual) or count > 1 or len(actual) > 1:
                return "authority", "unexpected publishers on " + topic
            if actual != [owner] or count != 1:
                if not (allow_absent_native and owner == native and not actual and count == 0):
                    missing = missing or ("camera" if owner == detector else "path",
                                          "missing selected publisher on " + topic)
            elif infos[0].topic_type != "std_msgs/msg/String":
                return "authority", "wrong deadline message type on " + topic
        for topic, owner in subscribers.items():
            # Read-only observers are harmless. The selected receiver itself
            # must have exactly one correctly typed subscription.
            infos = node.get_subscriptions_info_by_topic(topic)
            selected = [info for info in infos if name(info) == owner]
            if len(selected) > 1 or any(info.topic_type != "std_msgs/msg/String" for info in selected):
                return "authority", "invalid selected deadline subscription on " + topic
            if len(selected) != 1:
                if not (allow_absent_native and owner == native and not selected):
                    missing = missing or ("path", "missing selected subscriber on " + topic)
        if any(name(info) == native for info in node.get_subscriptions_info_by_topic("/cmd_vel")):
            return "authority", "strict native controller still subscribes to /cmd_vel"
    except (AttributeError, RuntimeError):
        return "path", "deadline topic graph unavailable"
    return missing


def decode_native_authority(raw, now, clock_id, previous=None):
    if not isinstance(raw, str) or len(raw) > 2048:
        raise ValueError("oversized native authority status")
    status = json.loads(raw)
    keys = {"schema", "clock_id", "generation", "activation_mono_ns", "bound_source_instance", "sampled_mono_ns"}
    if (not isinstance(status, dict) or set(status) != keys or
            status["schema"] != "pupper_command_authority_v1" or status["clock_id"] != clock_id):
        raise ValueError("native command authority schema or clock differs")
    generation, activation, sampled = (status[key] for key in
                                       ("generation", "activation_mono_ns", "sampled_mono_ns"))
    if (type(generation) is not int or not 1 <= generation < 2**64 or
            any(type(value) is not int or not 1 <= value < 2**63 for value in (activation, sampled)) or
            activation > sampled or
            not 0 <= round(now * 1e9) - sampled <= round(NATIVE_AUTHORITY_MAX_AGE_S * 1e9)):
        raise ValueError("native command authority timestamps or generation invalid")
    bound = status["bound_source_instance"]
    if not isinstance(bound, str) or (bound and (len(bound) != 32 or any(c not in "0123456789abcdef" for c in bound))):
        raise ValueError("native command authority source invalid")
    if previous is not None:
        if (generation < previous["generation"] or activation < previous["activation_mono_ns"] or
                sampled < previous["sampled_mono_ns"]):
            raise ValueError("native command authority regressed")
        if generation == previous["generation"] and (
                activation != previous["activation_mono_ns"] or
                (previous["bound_source_instance"] and bound != previous["bound_source_instance"])):
            raise ValueError("native binding changed inside one activation")
        if sampled == previous["sampled_mono_ns"] and status != previous:
            raise ValueError("native authority changed at the same sample time")
    return status


def native_authority_problem(status, observed_at, error, now, source_instance):
    if observed_at is None or not 0 <= now - observed_at <= NATIVE_AUTHORITY_MAX_AGE_S:
        return "native command authority unavailable or stale"
    if error:
        return error
    if not 0 <= round(now * 1e9) - status["sampled_mono_ns"] <= round(NATIVE_AUTHORITY_MAX_AGE_S * 1e9):
        return "native command authority source sample stale"
    if not status["bound_source_instance"]:
        return "native command source binding is pending"
    if status["bound_source_instance"] != source_instance:
        return "native remains bound to another bridge; explicit native Stop/Start required"
    return None


def decode_guard_status(raw, now, previous_source=None, previous_deadline=None, *,
                        command_contract="legacy_twist", clock_id=None, source_instance=None):
    if not isinstance(raw, str) or len(raw) > 8192:
        raise ValueError("oversized guard status")
    status = json.loads(raw)
    if not isinstance(status, dict):
        raise ValueError("invalid guard status")
    sampled_ns = status.get("sampled_at_monotonic_ns")
    if (type(sampled_ns) is not int or sampled_ns < 0 or
            not 0 <= round(now * 1e9) - sampled_ns <= round(GUARD_STATUS_MAX_AGE_S * 1e9)):
        raise ValueError("guard sample timestamp unavailable, future or stale")
    sampled_at = sampled_ns / 1e9
    if (status.get("state") not in ("ready", "passing", "hold", "fault") or
            status.get("camera_state") not in ("clear", "blocked", "stale", "invalid") or
            status.get("input_state") not in ("missing", "expired", "invalid", "valid") or
            type(status.get("camera_permitted")) is not bool or
            type(status.get("requires_post_hold_zero")) is not bool or
            type(status.get("clear_frames")) is not int or status["clear_frames"] < 0 or
            not isinstance(status.get("reason"), str)):
        raise ValueError("invalid guard status fields")
    age, limit = status.get("frame_age_s"), status.get("max_frame_age_s")
    if (isinstance(limit, bool) or not isinstance(limit, (int, float)) or
            not math.isfinite(limit) or not 0 < limit <= GUARD_FRAME_MAX_AGE_S):
        raise ValueError("invalid guard capture age limit")
    source, deadline = previous_source, previous_deadline
    if age is not None:
        if (isinstance(age, bool) or not isinstance(age, (int, float)) or
                not math.isfinite(age) or age < 0 or type(status.get("sequence")) is not int or
                status["sequence"] < 0 or not isinstance(status.get("producer_instance"), str) or
                not status["producer_instance"]):
            raise ValueError("invalid guard source capture evidence")
        source = (status["producer_instance"], status["sequence"])
        deadline = sampled_at + limit - age
        if previous_source is not None:
            if source[0] != previous_source[0] or source[1] < previous_source[1]:
                raise ValueError("guard source identity changed or regressed")
            if source == previous_source:
                deadline = min(deadline, previous_deadline)
    elif status["camera_permitted"]:
        raise ValueError("clear guard status lacks source capture evidence")
    if command_contract == "fc_deadline_v1":
        _validate_deadline_status(status, clock_id, source_instance)
    elif command_contract != "legacy_twist":
        raise ValueError("unknown command contract")
    return status, source, deadline


def _validate_deadline_status(status, clock_id, source_instance):
    if status.get("command_contract") != "fc_deadline_v1" or status.get("clock_id") != clock_id:
        raise ValueError("guard deadline contract or clock differs")
    for flag in ("source_bound", "source_vetoed"):
        if type(status.get(flag)) is not bool:
            raise ValueError("invalid guard source binding flags")
    if not status["source_bound"]:
        if (status.get("source_instance") is not None or status.get("source_seq") is not None or
                status.get("source_issued_mono_ns") is not None or
                status.get("source_expires_mono_ns") is not None or
                status.get("post_clear_zero_seq") is not None or not status["source_vetoed"] or
                not status["requires_post_hold_zero"] or status["state"] in ("ready", "passing")):
            raise ValueError("unbound guard claims source readiness")
        return
    if status.get("source_instance") != source_instance:
        raise ValueError("guard is bound to another bridge source instance")
    seq, issued, expiry = (status.get(key) for key in
                          ("source_seq", "source_issued_mono_ns", "source_expires_mono_ns"))
    if any(type(value) is not int or not 1 <= value < 2**63 for value in (seq, issued, expiry)):
        raise ValueError("invalid guard source sequence or timestamp")
    if not issued < expiry or expiry - issued > 150_000_000 or issued > status["sampled_at_monotonic_ns"]:
        raise ValueError("invalid guard source lifetime")
    clearance, zero_seq = status.get("clearance_mono_ns"), status.get("post_clear_zero_seq")
    if clearance is not None and (type(clearance) is not int or not 0 <= clearance <= status["sampled_at_monotonic_ns"]):
        raise ValueError("invalid guard clearance watermark")
    if zero_seq is not None and (type(zero_seq) is not int or not 1 <= zero_seq <= seq):
        raise ValueError("invalid guard source-zero sequence")
    if not status["requires_post_hold_zero"] and (clearance is None or zero_seq is None):
        raise ValueError("guard source-zero handshake is missing")
    if not status["source_vetoed"] and (status["requires_post_hold_zero"] or
            clearance is None or issued <= clearance or expiry <= status["sampled_at_monotonic_ns"] or
            status["input_state"] != "valid" or not status["camera_permitted"]):
        raise ValueError("guard passes without current deadline/clearance evidence")
    if status["state"] in ("ready", "passing") and status["source_vetoed"]:
        raise ValueError("vetoed guard claims source readiness")


def guard_status_problem(status, observed_at, error, deadline, now, *, camera=True,
                         command_contract="legacy_twist"):
    if observed_at is None or not 0 <= now - observed_at <= GUARD_STATUS_MAX_AGE_S:
        return "path", "guard status unavailable or stale"
    if error:
        return "path", error
    if not 0 <= round(now * 1e9) - status["sampled_at_monotonic_ns"] <= round(GUARD_STATUS_MAX_AGE_S * 1e9):
        return "path", "guard source status sample stale"
    if status["state"] == "fault":
        return "path", "guard fault: " + status["reason"]
    if camera and command_contract == "fc_deadline_v1" and (
            not status.get("source_bound") or status.get("source_vetoed") or
            status.get("post_clear_zero_seq") is None):
        return "camera", "deadline guard needs this bridge's post-clear source-zero handshake"
    if camera and (not status["camera_permitted"] or status["camera_state"] != "clear" or
                   deadline is None or now >= deadline or status["clear_frames"] < 3 or
                   status["requires_post_hold_zero"] or status["input_state"] == "invalid"):
        return "camera", "camera guard holds: " + status["reason"]
    return None
