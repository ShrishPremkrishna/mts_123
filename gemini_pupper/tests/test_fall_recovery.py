"""A new native tilt stop must not be cleared by an older recovery."""
import asyncio
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import robot_api
import tool_registry
from robot_api import ESTOP_LOG, FALLEN, LOCOMOTION_CONTROLLER


def run_case(case):
    async def run():
        rig = harness.Rig()
        try:
            assert await rig.settle()
            await case(rig)
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(run())


@pytest.mark.parametrize("delay_cleanup", [False, True])
def test_second_tilt_during_recovery_does_not_clear_fall(monkeypatch, delay_cleanup):
    async def case(rig):
        rig.ts.fallen = "neural_controller_default"
        rig.imu(0, seconds=2)
        pending = []
        if delay_cleanup:
            # ROS can deliver the log before asyncio gets to its cleanup task.
            monkeypatch.setattr(robot_api.asyncio, "run_coroutine_threadsafe",
                                lambda coro, loop: pending.append(coro))
        emitted = False

        def second_fall():
            nonlocal emitted
            if not emitted:
                emitted = True
                assert rig.robot.controllers[LOCOMOTION_CONTROLLER] == "active"
                rig.ts._on_rosout(harness._Msg(name=LOCOMOTION_CONTROLLER, msg=ESTOP_LOG))
        rig.clock.on_advance.append(second_fall)
        try:
            result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
            assert emitted
            assert not result.ok, result.message
            assert rig.ts.fallen == LOCOMOTION_CONTROLLER
            for coro in pending:
                await coro
            pending.clear()
            await asyncio.sleep(0)
            assert rig.ts.state_now == FALLEN
            movement = await tool_registry.call(rig.ts, "move", {"direction": "forward", "duration": 0.5})
            assert not movement.ok
            assert not harness.walk_twists(rig)
        finally:
            for coro in pending:
                coro.close()
    run_case(case)


def test_recovery_without_another_tilt_still_clears_fall():
    async def case(rig):
        rig.ts.fallen = "neural_controller_default"
        rig.imu(0, seconds=2)
        result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert result.ok, result.message
        assert rig.ts.fallen is None
        assert rig.robot.controllers[LOCOMOTION_CONTROLLER] == "active"
    run_case(case)
