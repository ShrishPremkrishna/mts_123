"""Queue estimates include existing playback pads and return-controller waits.

Fake ROS only. These tests do not shorten any controller or motion duration.
"""
import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry
from robot_api import LOCOMOTION_CONTROLLER, WALK_READY_S
from tricks import load_tricks


def run_case(case):
    async def run():
        rig = harness.Rig(tricks=load_tricks)
        try:
            assert await rig.settle()
            await case(rig)
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(run())


@pytest.mark.parametrize("name", ["stand_downward_dog", "pee", "stand_sit_stand", "push_up"])
def test_supplied_finite_estimate_covers_playback_pad_and_final_walker_ready(name):
    async def case(rig):
        trick = rig.ts.tricks[name]
        before = rig.clock.now
        reply = await tool_registry.call(rig.ts, "play_trick", {"count": 1, "count_unit": "routine_playback", "name": name})
        assert reply.ok
        assert rig.ts.busy_until - before == pytest.approx(trick.play_time + 0.3 + WALK_READY_S)
        assert await rig.settle()
        assert rig.ts.last_outcome["success"]
        # The estimate changes, while the real controller-return wait remains.
        assert rig.clock.now - rig.robot.activations(LOCOMOTION_CONTROLLER)[-1] >= WALK_READY_S
    run_case(case)


def test_nonstanding_finite_estimate_does_not_invent_a_walking_return():
    async def case(rig):
        trick = rig.ts.tricks["swim"]
        assert trick.end_pose != "standing" and not trick.interactive
        before = rig.clock.now
        reply = await tool_registry.call(rig.ts, "play_trick", {"count": 1, "count_unit": "routine_playback", "name": trick.name})
        assert reply.ok
        assert rig.ts.busy_until - before == pytest.approx(trick.play_time + 0.3)
        assert await rig.settle()
        assert rig.robot.active() == {trick.controller}
    run_case(case)


def test_sit_entry_and_exit_estimates_include_only_their_existing_waits():
    async def case(rig):
        trick = rig.ts.tricks["sit_stay_shake"]
        before = rig.clock.now
        assert (await tool_registry.call(rig.ts, "play_trick", {"count": 1, "count_unit": "routine_playback", "name": trick.name})).ok
        assert rig.ts.busy_until - before == pytest.approx(trick.play_time + 0.3)
        assert await rig.settle()
        before = rig.clock.now
        assert (await tool_registry.call(rig.ts, "interactive_action", {"action": "stand_up"})).ok
        assert rig.ts.busy_until - before == pytest.approx(trick.clip_time_estimate(trick.exit_clip) + WALK_READY_S)
        assert await rig.settle()
        assert rig.clock.now - rig.robot.activations(LOCOMOTION_CONTROLLER)[-1] >= WALK_READY_S
    run_case(case)


def test_paw_estimate_has_no_walking_return():
    async def case(rig):
        trick = rig.ts.tricks["sit_stay_shake"]
        assert (await tool_registry.call(rig.ts, "play_trick", {"count": 1, "count_unit": "routine_playback", "name": trick.name})).ok
        assert await rig.settle()
        before = rig.clock.now
        assert (await tool_registry.call(rig.ts, "interactive_action", {"action": "shake_left"})).ok
        assert rig.ts.busy_until - before == pytest.approx(trick.clip_time_estimate("shake_left"))
        assert await rig.settle()
        assert rig.robot.active() == {trick.controller}
    run_case(case)


def test_four_finite_queue_estimates_accumulate_full_return_waits():
    async def case(rig):
        before = rig.clock.now
        total = 0.0
        for name in ["stand_downward_dog", "pee", "stand_sit_stand", "push_up"]:
            trick = rig.ts.tricks[name]
            total += trick.play_time + 0.3 + WALK_READY_S
            assert (await tool_registry.call(rig.ts, "play_trick", {"count": 1, "count_unit": "routine_playback", "name": name})).ok
            assert rig.ts.busy_until - before == pytest.approx(total)
        assert total == pytest.approx(67.68)
        assert await rig.settle()
        assert not rig.failures()
    run_case(case)
