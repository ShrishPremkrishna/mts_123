"""robot_status and switch_to under a controller fault or a fall (the gate-free part of
the removed Optional Lab 3 status tests): the fault stays visible and motion stays blocked."""
import asyncio
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness  # noqa: E402
import robot_api  # noqa: E402


def run_case(case):
    async def run():
        rig = harness.Rig()
        try:
            assert await rig.settle()
            await case(rig, rig.ts.tricks['hold_pose'])
        finally:
            rig.close()
            for _ in range(5):
                await asyncio.sleep(0)
    asyncio.run(run())


def test_fault_is_reported_verbatim_and_keeps_the_quarantine():
    async def case(rig, trick):
        fault = f'{trick.controller} did not confirm a fresh final playback frame; motion quarantined. Inspect/restart the controller stack.'
        rig.ts.controller_fault = fault
        switches = list(rig.robot.switches)
        result = await rig.ts.mode()
        assert not result.ok
        assert result.message.startswith(fault)
        assert result.details['controller_fault'] == fault
        assert rig.ts.motion_block_reason() == fault
        blocked = await rig.ts.switch_to(robot_api.LOCOMOTION_CONTROLLER)
        assert not blocked.ok and rig.robot.switches == switches
        assert rig.ts.controller_fault == fault
    run_case(case)


def test_fall_status_names_the_controller_and_the_fallen_state():
    async def case(rig, trick):
        rig.robot.controllers[robot_api.LOCOMOTION_CONTROLLER] = 'inactive'
        rig.robot.controllers[trick.controller] = 'active'
        rig.ts.state_now = rig.ts.state_after_queue = trick
        await rig.ts._on_fall(trick.controller)
        result = await rig.ts.mode()
        assert 'tilt/fall stop recorded' in result.message and trick.controller in result.message
        assert result.details['state'] == robot_api.FALLEN
        assert rig.ts.fallen == trick.controller
    run_case(case)
