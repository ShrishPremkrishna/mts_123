"""Closed status wording may expose only a validated server-session playback count."""
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
from operator_feedback import receipt_feedback


def status(context):
    return {"success": True, "details": {"running": False, "queued": 0,
            "current_action": None, "observed_controller_mode": "sit_stay_shake",
            "session_context": context}}


def context(**changes):
    return {"schema": "pupper_session_context_v1", "scope": "this_server_session",
            "server_instance": "a" * 32, "completed_sit_entries": 2, **changes}


@pytest.mark.parametrize("count", [0, 1, 2.0, 1_000_000])
def test_count_uses_software_server_scope_and_keeps_instance_out_of_spoken_text(count):
    message = receipt_feedback("robot_status", status(context(completed_sit_entries=count)))
    assert message == ("Controller: Sit. No command running; 0 queued. "
                       f"Confirmed Sit-entry playbacks in this server session: {int(count)}.")
    assert "a" * 32 not in message
    assert "today" not in message and "physical" not in message


@pytest.mark.parametrize("patch", [
    {"schema": "unknown"}, {"scope": "all_day"}, {"scope": "this_conversation"},
    {"server_instance": None}, {"server_instance": "a" * 31}, {"server_instance": "A" * 32},
    {"server_instance": "a" * 32 + "\n"}, {"completed_sit_entries": -1},
    {"completed_sit_entries": True}, {"completed_sit_entries": None},
    {"completed_sit_entries": "2"}, {"completed_sit_entries": 1.5},
    {"completed_sit_entries": float("nan")}, {"completed_sit_entries": float("inf")},
    {"completed_sit_entries": 1_000_001},
])
def test_malformed_context_cannot_become_authoritative_count(patch):
    message = receipt_feedback("robot_status", status(context(**patch)))
    assert message == "Controller: Sit. No command running; 0 queued."


@pytest.mark.parametrize("value", [None, [], "2", 2])
def test_missing_or_nonobject_context_keeps_previous_receipt(value):
    assert "Confirmed Sit-entry" not in receipt_feedback("robot_status", status(value))


@pytest.mark.parametrize("flags", [{"success": False}, {"outcome_unknown": True},
                                   {"superseded": True}, {"not_dispatched": True}])
def test_failure_and_uncertainty_take_precedence_over_context(flags):
    assert "Confirmed Sit-entry" not in receipt_feedback("robot_status", {**status(context()), **flags})
