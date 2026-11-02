"""Estimate-only regressions using the real lab functions and fake ROS clock."""
import asyncio
import math
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
import tool_registry
import robot_api
from tricks import load_tricks


def run_case(case):
    async def run():
        rig = harness.Rig(tricks=load_tricks)
        try:
            assert await rig.settle()
            await case(rig)
        finally:
            rig.close()
            for _ in range(5):
                await asyncio.sleep(0)
    asyncio.run(run())


async def enter_sit(rig):
    result = await tool_registry.call(rig.ts, 'play_trick',
        {'name': 'sit_stay_shake', 'count': 1, 'count_unit': 'routine_playback'})
    assert result.ok
    assert await rig.settle()
    return rig.ts.tricks['sit_stay_shake']


@pytest.mark.parametrize('tool,args,duration', [
    ('move', {'direction': 'forward', 'speed': .2, 'duration': .5}, .5),
    ('turn', {'direction': 'left', 'angle': 15}, None),
    ('arc', {'direction': 'left', 'radius': .75, 'angle': 30, 'speed': .2}, math.radians(30) * .75 / .2),
    ('stand_up', {}, 0.),
])
def test_sit_to_locomotion_estimate_includes_exit_and_ready(tool, args, duration):
    async def case(rig):
        sit = await enter_sit(rig)
        before = rig.clock.now
        result = await tool_registry.call(rig.ts, tool, args)
        assert result.ok, result.message
        motion_duration = 15 / rig.ts.speeds().turn if duration is None else duration
        expected = sit.clip_time_estimate(sit.exit_clip) + robot_api.WALK_READY_S + motion_duration
        assert rig.ts.busy_until - before == pytest.approx(expected)
        assert await rig.settle()
        assert rig.ts.last_outcome['success']
        # The edit must not shorten the actual locomotion initialization or motion.
        activated = rig.robot.activations(robot_api.LOCOMOTION_CONTROLLER)[-1]
        assert rig.clock.now - activated >= robot_api.WALK_READY_S + motion_duration - 1e-6
    run_case(case)


def test_sit_queued_then_two_moves_counts_planned_exit_once():
    async def case(rig):
        sit = rig.ts.tricks['sit_stay_shake']
        before = rig.clock.now
        assert (await tool_registry.call(rig.ts, 'play_trick',
            {'name': sit.name, 'count': 1, 'count_unit': 'routine_playback'})).ok
        assert rig.ts.state_now == robot_api.LOCOMOTION  # still planned, not already sitting
        for _ in range(2):
            assert (await tool_registry.call(rig.ts, 'move', {'direction': 'forward', 'speed': .2, 'duration': .5})).ok
        expected = sit.play_time + .3 + sit.clip_time_estimate(sit.exit_clip) + robot_api.WALK_READY_S + 1.
        assert rig.ts.busy_until - before == pytest.approx(expected)
        assert await rig.settle()
        assert not rig.failures()
    run_case(case)


def test_explicit_sit_exit_then_move_does_not_double_count_handoff():
    async def case(rig):
        sit = await enter_sit(rig)
        before = rig.clock.now
        assert (await tool_registry.call(rig.ts, 'interactive_action', {'action': 'stand_up'})).ok
        assert (await tool_registry.call(rig.ts, 'move', {'direction': 'forward', 'speed': .2, 'duration': .5})).ok
        assert rig.ts.busy_until - before == pytest.approx(sit.clip_time_estimate(sit.exit_clip) + robot_api.WALK_READY_S + .5)
        assert await rig.settle()
    run_case(case)


def test_idle_then_move_and_stand_pay_walking_ready_only_once():
    async def case(rig):
        assert (await rig.ts.switch_to(None)).ok
        rig.ts.state_after_queue = robot_api.MOTORS_IDLE
        before = rig.clock.now
        assert (await tool_registry.call(rig.ts, 'move', {'direction': 'forward', 'speed': .2, 'duration': .5})).ok
        assert rig.ts.busy_until - before == pytest.approx(robot_api.WALK_READY_S + .5)
        assert (await tool_registry.call(rig.ts, 'stand_up', {})).ok
        assert rig.ts.busy_until - before == pytest.approx(robot_api.WALK_READY_S + .5)
        assert (await tool_registry.call(rig.ts, 'move', {'direction': 'forward', 'speed': .2, 'duration': .5})).ok
        assert rig.ts.busy_until - before == pytest.approx(robot_api.WALK_READY_S + 1.)
        assert await rig.settle()
    run_case(case)


def test_sit_to_other_finite_skill_adds_exit_once_and_keeps_existing_return():
    async def case(rig):
        sit = await enter_sit(rig)
        finite = rig.ts.tricks['push_up']
        before = rig.clock.now
        assert (await tool_registry.call(rig.ts, 'play_trick',
            {'name': finite.name, 'count': 1, 'count_unit': 'routine_playback'})).ok
        expected = sit.clip_time_estimate(sit.exit_clip) + finite.play_time + .3 + robot_api.WALK_READY_S
        assert rig.ts.busy_until - before == pytest.approx(expected)
        assert await rig.settle()
        assert rig.ts.last_outcome['success']
    run_case(case)


def test_already_walking_does_not_add_handoff_or_exit():
    async def case(rig):
        before = rig.clock.now
        assert (await tool_registry.call(rig.ts, 'move', {'direction': 'forward', 'speed': .2, 'duration': .5})).ok
        assert rig.ts.busy_until - before == pytest.approx(.5)
        assert await rig.settle()
    run_case(case)
