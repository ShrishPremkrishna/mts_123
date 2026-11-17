"""Shared guarded-path observations, not DDS authentication or motion authority."""
from __future__ import annotations

import json
import math

GUARD_STATUS_TOPIC = "/fc/body_command_guard/status"
GUARD_INPUT_TOPIC = "/fc/requested_cmd_vel"
GUARD_STATUS_MAX_AGE_S = 0.3
GUARD_FRAME_MAX_AGE_S = 0.35


def guard_publishers_problem(node, client_name, *, robot_topic="/cmd_vel", kick_topic="/kick_cmd_vel"):
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


def decode_guard_status(raw, now, previous_source=None, previous_deadline=None):
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
    return status, source, deadline


def guard_status_problem(status, observed_at, error, deadline, now, *, camera=True):
    if observed_at is None or not 0 <= now - observed_at <= GUARD_STATUS_MAX_AGE_S:
        return "path", "guard status unavailable or stale"
    if error:
        return "path", error
    if not 0 <= round(now * 1e9) - status["sampled_at_monotonic_ns"] <= round(GUARD_STATUS_MAX_AGE_S * 1e9):
        return "path", "guard source status sample stale"
    if status["state"] == "fault":
        return "path", "guard fault: " + status["reason"]
    if camera and (not status["camera_permitted"] or status["camera_state"] != "clear" or
                   deadline is None or now >= deadline or status["clear_frames"] < 3 or
                   status["requires_post_hold_zero"] or status["input_state"] == "invalid"):
        return "camera", "camera guard holds: " + status["reason"]
    return None
