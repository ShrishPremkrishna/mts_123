"""Queued wait is accepted immediately and remains interruptible by Stop.

These tests call the real registry without DeskBackend's per-call settle shortcut.
ROS, clocks and messages use the existing fake harness; no sockets or hardware.
"""
import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
import robot_api
import tool_registry


def run_case(case):
    async def run():
        rig = harness.Rig()
        try:
            assert await rig.settle()
            await case(rig)
        finally:
            rig.close()
            for _ in range(5):
                await asyncio.sleep(0)
    asyncio.run(run())


def test_wait_is_declared_with_required_numeric_duration():
    async def case(rig):
        decl = next(d for d in tool_registry.declarations(rig.ts) if d['name'] == 'wait')
        assert decl['parameters']['required'] == ['duration']
        assert decl['parameters']['properties']['duration']['type'] == 'NUMBER'
    run_case(case)


@pytest.mark.parametrize('args', [{}, {'duration': True}, {'duration': '1'},
    {'duration': 0}, {'duration': -1}, {'duration': 60.01},
    {'duration': float('nan')}, {'duration': float('inf')}, {'duration': 1, 'extra': 0}])
def test_invalid_wait_never_queues_or_acts(args):
    async def case(rig):
        started = rig.clock.now
        before = (list(rig.robot.switches), list(rig.robot.published), list(rig.robot.follow_calls))
        result = await tool_registry.call(rig.ts, 'wait', args)
        assert not result.ok
        assert rig.ts.commands.empty() and rig.ts.current is None
        assert rig.clock.now == started
        assert before == (rig.robot.switches, rig.robot.published, rig.robot.follow_calls)
    run_case(case)


@pytest.mark.parametrize('duration', [.001, 1., 60.])
def test_wait_reply_is_acceptance_before_elapsed_then_status_has_outcome(duration):
    async def case(rig):
        started = rig.clock.now
        result = await tool_registry.call(rig.ts, 'wait', {'duration': duration})
        assert result.ok and 'queued' in result.message and 'waited' not in result.message
        assert rig.clock.now == started
        assert rig.ts.commands.qsize() == 1
        assert await rig.settle()
        assert rig.clock.now - started == pytest.approx(duration)
        status = await tool_registry.call(rig.ts, 'robot_status', {})
        assert status.ok and f'waited {duration} s' in status.message
        assert not harness.walk_twists(rig)
    run_case(case)


def test_stand_wait_move_uses_real_queue_order_without_settle_between_calls():
    async def case(rig):
        assert (await rig.ts.switch_to(None)).ok
        rig.ts.state_after_queue = robot_api.MOTORS_IDLE
        started = rig.clock.now
        replies = [
            await tool_registry.call(rig.ts, 'stand_up', {}),
            await tool_registry.call(rig.ts, 'wait', {'duration': 1.}),
            await tool_registry.call(rig.ts, 'move', {'direction': 'forward', 'speed': .2, 'duration': .1}),
        ]
        assert all(r.ok and 'queued' in r.message for r in replies), replies
        assert rig.clock.now == started
        assert rig.ts.commands.qsize() == 3
        assert await rig.settle()
        movement = harness.walk_twists(rig)
        assert movement
        first_move_time = next(t for t, _, msg in rig.robot.published if hasattr(msg, 'linear') and msg.linear.x > 0)
        assert first_move_time - started >= robot_api.WALK_READY_S + 1. - 1e-6
        outcomes = rig.outcomes()
        stand = next(i for i, v in enumerate(outcomes) if v.startswith('stand up:'))
        wait = next(i for i, v in enumerate(outcomes) if v.startswith('wait 1.0s: waited'))
        move = next(i for i, v in enumerate(outcomes) if v.startswith('move vx=0.2'))
        assert stand < wait < move
    run_case(case)


def test_stop_does_not_wait_for_previously_queued_pause():
    async def case(rig):
        started = rig.clock.now
        assert (await tool_registry.call(rig.ts, 'wait', {'duration': 60.})).ok
        stopped = await tool_registry.call(rig.ts, 'stop', {})
        assert stopped.ok
        assert rig.clock.now - started < 1.
        assert rig.ts.commands.empty()
        assert await rig.settle()
        assert not any('waited 60' in text for text in rig.outcomes())
        assert not harness.walk_twists(rig)
    run_case(case)


def test_stop_cancels_current_wait_without_holding_request_lock():
    async def case(rig):
        entered = asyncio.Event()
        original_sleep = robot_api.asyncio.sleep

        async def hold_pause(seconds, result=None):
            if seconds == 60.:
                entered.set()
                await asyncio.Event().wait()
            return await original_sleep(seconds, result)

        robot_api.asyncio.sleep = hold_pause
        assert (await tool_registry.call(rig.ts, 'wait', {'duration': 60.})).ok
        for _ in range(20):
            if entered.is_set():
                break
            await asyncio.sleep(0)
        assert entered.is_set() and not rig.ts.action_lock.locked()
        before = rig.clock.now
        stopped = await tool_registry.call(rig.ts, 'stop', {})
        assert stopped.ok and rig.clock.now - before < 1.
        assert await rig.settle()
        assert any('wait 60.0s: cancelled' in text for text in rig.outcomes())
        assert not any('waited 60' in text for text in rig.outcomes())
    run_case(case)


def test_wait_keeps_follow_mode_and_does_not_claim_stillness():
    async def case(rig):
        rig.ts.following = rig.robot.follower_on = True
        calls = list(rig.robot.follow_calls)
        switches = list(rig.robot.switches)
        result = await tool_registry.call(rig.ts, 'wait', {'duration': .25})
        assert result.ok and 'queued' in result.message
        assert await rig.settle()
        assert rig.ts.following and rig.robot.follower_on
        assert rig.robot.follow_calls == calls and rig.robot.switches == switches
        assert not harness.walk_twists(rig)
    run_case(case)
