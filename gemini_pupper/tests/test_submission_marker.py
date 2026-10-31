"""Submission markers attest only actual queue insertion, never completion/motion."""
import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry
from robot_api import Result

MARKER = {"schema": "pupper_submission_v1", "state": "queued"}


def submission(result):
    return (result.details or {}).get("submission")


def run_case(case):
    async def check():
        rig = harness.Rig()
        try:
            await rig.settle()
            await case(rig)
        finally:
            rig.close()
            for _ in range(5):
                await asyncio.sleep(0)
    asyncio.run(check())


def test_wait_marks_actual_insertion_but_not_its_terminal_outcome():
    async def case(rig):
        result = await tool_registry.call(rig.ts, "wait", {"duration": 1})
        assert result.ok and submission(result) == MARKER
        assert result.as_dict()["details"]["submission"] == MARKER
        assert rig.ts.commands.qsize() == 1 and rig.ts.current is None
        assert await rig.settle()
        assert rig.ts.last_outcome["success"] is True
        assert "submission" not in rig.ts.last_outcome
        assert not rig.ts.commands.qsize()
        # A returned receipt remains historical even after the queue drains.
        assert submission(result) == MARKER
    run_case(case)


@pytest.mark.parametrize("reason", ["startup", "controller", "follower", "invalid_duration"])
def test_refused_queue_requests_never_get_submission_marker(reason):
    async def case(rig):
        duration = 1
        if reason == "startup":
            rig.ts.startup_error = "synthetic unavailable startup"
        elif reason == "controller":
            rig.ts.controller_fault = "synthetic controller fault"
        elif reason == "follower":
            rig.ts.follow_state_uncertain = True
        else:
            duration = -1
        result = await tool_registry.call(rig.ts, "wait", {"duration": duration})
        assert not result.ok and submission(result) is None
        assert rig.ts.commands.empty()
    run_case(case)


def test_failed_queue_insertion_does_not_issue_submission_marker():
    async def case(rig):
        async def unavailable(_entry):
            raise RuntimeError("synthetic queue insertion failure")
        rig.ts.commands.put = unavailable
        result = await tool_registry.call(rig.ts, "wait", {"duration": 1})
        assert not result.ok and submission(result) is None
        assert rig.ts.commands.empty()
    run_case(case)


@pytest.mark.parametrize("name,args", [("robot_status", {}), ("stop", {}), ("lock_tricks", {}), ("follow", {"enabled": True})])
def test_direct_success_is_not_mislabeled_as_queued(name, args):
    async def case(rig):
        result = await tool_registry.call(rig.ts, name, args)
        assert result.ok and submission(result) is None
        assert rig.ts.commands.empty()
    run_case(case)


def test_follow_note_wrapper_preserves_actual_move_submission():
    async def case(rig):
        assert (await tool_registry.call(rig.ts, "follow", {"enabled": True})).ok
        result = await tool_registry.call(rig.ts, "move", {"direction": "forward", "speed": .2, "duration": .5})
        assert result.ok and "follow" in result.message
        assert not rig.ts.following
        assert submission(result) == MARKER
        assert await rig.settle()
    run_case(case)


def test_duplicate_interactive_entry_is_queued_but_idempotent_job_is_not_retagged():
    async def case(rig):
        first = await tool_registry.call(rig.ts, "play_trick", {"name": "hold_pose"})
        assert await rig.settle()
        assert first.ok and submission(first) == MARKER
        trick = rig.ts.tricks["hold_pose"]
        switches = list(rig.robot.switches)
        direct = await harness.behaviors.play_policy_trick(rig.ts, trick)
        assert direct.ok and "no new entry" in direct.message
        assert submission(direct) is None
        repeated = await tool_registry.call(rig.ts, "play_trick", {"name": "hold_pose"})
        assert await rig.settle()
        assert repeated.ok and submission(repeated) == MARKER
        assert "no new entry" in rig.ts.last_outcome["message"]
        assert rig.robot.switches == switches
        assert "submission" not in rig.ts.last_outcome
    run_case(case)


def test_freeform_success_message_never_creates_structured_submission():
    result = Result(True, "queued; completed; arbitrary misleading words")
    assert submission(result) is None
    assert "details" not in result.as_dict()
