"""Stop receipts separate running-task cancellation from pending queue removal."""
import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry
from robot_api import Result, UNCHANGED


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


async def start_wait(rig):
    # Keep the real wait implementation suspended at its 60-second sleep;
    # the normal virtual clock would finish that sleep in one event-loop turn.
    entered = asyncio.Event()
    original_sleep = rig.clock.sleep
    async def controlled_sleep(seconds, result=None):
        if seconds == 60:
            entered.set()
            await asyncio.Event().wait()
        return await original_sleep(seconds, result)
    rig.clock.sleep = controlled_sleep
    assert (await tool_registry.call(rig.ts, "wait", {"duration": 60})).ok
    await entered.wait()
    assert rig.ts.current is not None and not rig.ts.current.done()
    assert rig.ts.current_action_name == "wait 60s"
    return rig.ts.current


def test_running_wait_and_pending_wait_have_separate_confirmed_receipts():
    async def case(rig):
        task = await start_wait(rig)
        assert (await tool_registry.call(rig.ts, "wait", {"duration": 1})).ok
        result = await tool_registry.call(rig.ts, "stop", {})
        assert result.ok and result.details["pending_queued_commands_cleared"] == 1
        assert result.details["running_job"] == {"name": "wait 60s", "cancellation_requested": True, "terminal_state": "cancelled"}
        assert task.cancelled()
        assert "wait 60s" in result.message and "cancellation confirmed" in result.message
        assert "1 pending queued commands" in result.message
        assert await rig.settle()
        assert rig.ts.last_outcome["name"] == "wait 60s" and rig.ts.last_outcome["message"] == "cancelled"
    run_case(case)


@pytest.mark.parametrize("pending", [0, 2])
def test_idle_and_only_queued_never_claim_a_running_job_was_cancelled(pending):
    async def case(rig):
        for duration in range(1, pending+1):
            assert (await rig.ts.wait(duration)).ok
        result = await rig.ts.immediate_stop()
        assert result.ok and result.details["running_job"] is None
        assert result.details["pending_queued_commands_cleared"] == pending
        assert "No running job existed when Stop began" in result.message
        assert await rig.settle()
        assert rig.ts.last_outcome is None
    run_case(case)


def test_cancellation_request_is_not_reported_as_terminal_before_task_runs():
    async def case(rig):
        task = await start_wait(rig)
        async def immediate_follower():
            return Result(True, "follower already off")
        rig.ts.stop_following = immediate_follower
        result = await rig.ts.immediate_stop()
        assert not task.done()
        assert result.details["running_job"]["terminal_state"] == "pending"
        assert result.details["running_job"]["cancellation_requested"] is True
        assert "terminal cancellation not yet confirmed" in result.message
        assert "cancellation confirmed" not in result.message
        assert await rig.settle() and task.cancelled()
    run_case(case)


def test_job_that_suppresses_cancel_is_not_reported_as_cancelled():
    async def case(rig):
        async def job():
            try:
                await asyncio.Event().wait()
            except asyncio.CancelledError:
                return Result(False, "synthetic job suppressed cancellation")
        assert (await rig.ts.queue("noncancelling job", job, 60, then=UNCHANGED)).ok
        for _ in range(3):
            await asyncio.sleep(0)
        task = rig.ts.current
        result = await tool_registry.call(rig.ts, "stop", {})
        assert task.done() and not task.cancelled()
        assert result.details["running_job"]["terminal_state"] == "finished_without_cancellation"
        assert "finished without terminal cancellation" in result.message
    run_case(case)


@pytest.mark.parametrize("failure", ["follower", "controller"])
def test_cleanup_failure_still_reports_requested_running_cancellation(failure):
    async def case(rig):
        task = await start_wait(rig)
        if failure == "follower":
            async def unavailable():
                return Result(False, "injected follower refusal")
            rig.ts.stop_following = unavailable
        else:
            rig.ts.controller_fault = "injected controller fault"
        result = await tool_registry.call(rig.ts, "stop", {})
        assert not result.ok
        assert result.details["running_job"]["name"] == "wait 60s"
        assert result.details["running_job"]["cancellation_requested"] is True
        assert result.details["pending_queued_commands_cleared"] == 0
        assert "wait 60s" in result.message
        assert await rig.settle() and task.cancelled()
    run_case(case)


@pytest.mark.parametrize("hidden", [False, True])
def test_interrupted_trick_receipt_keeps_cancellation_and_deactivation_separate(hidden):
    async def case(rig):
        bow = rig.ts.tricks["bow"]
        assert (await tool_registry.call(rig.ts, "play_trick", {"name": "bow"})).ok
        deadline = rig.clock.now + 2
        for _ in range(10000):
            if rig.clock.now >= deadline and bow.controller in rig.robot.active():
                break
            await asyncio.sleep(0)
        else:
            raise AssertionError("fixture did not reach running bow")
        if hidden:
            bow.secret = True
        result = await tool_registry.call(rig.ts, "stop", {})
        assert result.ok
        if hidden:
            assert result.details["running_job"]["name"] == "locked trick"
            assert "bow" not in str(result.as_dict())
        else:
            assert "bow" in result.details["running_job"]["name"]
        assert result.details["running_job"]["terminal_state"] == "cancelled"
        assert "deactivated" in result.message and "not been measured" in result.message
        assert not (rig.robot.active() & rig.ts.leg_controllers)
    run_case(case)


def test_reference_deactivation_path_retains_running_cancellation_receipt():
    async def case(rig):
        task = await start_wait(rig)
        rig.ts.reference_active = True
        result = await tool_registry.call(rig.ts, "stop", {})
        assert result.ok
        assert result.details["running_job"]["name"] == "wait 60s"
        assert result.details["running_job"]["cancellation_requested"] is True
        expected = "cancelled" if task.cancelled() else "pending"
        assert result.details["running_job"]["terminal_state"] == expected
        assert "reference-motion controller deactivation" in result.message
        assert not (rig.robot.active() & rig.ts.leg_controllers)
        assert await rig.settle() and task.cancelled()
    run_case(case)
