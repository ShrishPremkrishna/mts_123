"""Ordinary movement after clean Stop prepares its own walker; recovery stays gated."""
import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry
from robot_api import LOCOMOTION_CONTROLLER, MOTORS_IDLE, WALK_READY_S


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


async def stop_playing_trick(rig):
    bow = rig.ts.tricks["bow"]
    assert (await tool_registry.call(rig.ts, "play_trick", {"name": "bow"})).ok
    deadline = rig.clock.now + 2.0
    for _ in range(10000):
        if rig.clock.now >= deadline and bow.controller in rig.robot.active():
            break
        await asyncio.sleep(0)
    else:
        raise AssertionError("fixture did not reach running bow")
    assert rig.ts.current is not None
    result = await tool_registry.call(rig.ts, "stop", {})
    assert await rig.settle()
    assert result.ok and "deactivated" in result.message
    assert not (rig.robot.active() & rig.ts.leg_controllers)
    assert rig.ts.state_now == MOTORS_IDLE
    return result


def test_stop_guidance_does_not_invent_an_extra_stand_command():
    async def case(rig):
        declaration = next(d for d in tool_registry.declarations(rig.ts) if d["name"] == "stop")
        assert "required before another motion" not in declaration["description"]
        result = await stop_playing_trick(rig)
        assert "stand_up is required" not in result.message
        assert "readiness" in result.message and "fall" in result.message
        assert "not been measured" in result.message
    run_case(case)


@pytest.mark.parametrize("interrupted_trick", [False, True])
def test_new_move_after_clean_stop_needs_no_stand_tool_and_waits_for_walker(interrupted_trick):
    async def case(rig):
        if interrupted_trick:
            await stop_playing_trick(rig)
        else:
            assert (await tool_registry.call(rig.ts, "stop", {})).ok
            assert await rig.settle()
        start_published = len(rig.robot.published)
        start_switches = len(rig.robot.switches)
        result = await rig.call("move", direction="forward", speed=.2, duration=.5)
        assert result.ok
        assert rig.ts.last_outcome["success"] is True
        assert rig.ts.last_outcome["name"].startswith("move ")
        twists = [(t, m) for t, _, m in rig.robot.published[start_published:]
                  if hasattr(m, "linear") and abs(m.linear.x) > 1e-9]
        assert twists and all(m.linear.x == .2 and m.linear.y == m.angular.z == 0 for _, m in twists)
        if interrupted_trick:
            activations = [t for t, _, active in rig.robot.switches[start_switches:] if LOCOMOTION_CONTROLLER in active]
            assert len(activations) == 1
            assert twists[0][0] - activations[0] >= WALK_READY_S - 1e-6
        else:
            assert rig.robot.switches[start_switches:] == []
    run_case(case)


@pytest.mark.parametrize("block", ["fall", "latched", "stale"])
def test_clean_stop_never_removes_fall_or_readiness_blocks(block):
    async def case(rig):
        assert (await tool_registry.call(rig.ts, "stop", {})).ok
        assert await rig.settle()
        if block == "fall":
            rig.ts.fallen = "reported tilt stop"
        elif block == "latched":
            rig.robot.stop_status.update(latched=True, stop_required=True,
                                         epoch=rig.robot.stop_status["epoch"]+1)
            rig.robot.emit_stop_status()
        else:
            rig.robot.stop_status_enabled = False
            rig.clock.now += .501
        switches = len(rig.robot.switches)
        published = len(rig.robot.published)
        result = await rig.call("move", direction="forward", speed=.2, duration=.5)
        assert not result.ok
        assert not any(active for _, _, active in rig.robot.switches[switches:])
        assert not any(hasattr(m, "linear") and any(abs(v) > 1e-9 for v in (m.linear.x,m.linear.y,m.angular.z))
                       for _, _, m in rig.robot.published[published:])
        if block == "fall":
            assert rig.ts.fallen == "reported tilt stop"
    run_case(case)


def test_lie_down_describes_and_deactivates_controllers_without_pose_claim():
    async def case(rig):
        declaration = next(d for d in tool_registry.declarations(rig.ts) if d["name"] == "lie_down")
        assert "Controller deactivation does not verify physical posture" in declaration["description"]
        result = await rig.call("lie_down")
        assert result.ok and rig.ts.last_outcome["success"] is True
        assert not (rig.robot.active() & rig.ts.leg_controllers)
        assert rig.ts.state_now == MOTORS_IDLE
    run_case(case)
