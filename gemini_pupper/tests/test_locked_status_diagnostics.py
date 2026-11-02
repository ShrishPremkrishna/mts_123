"""Private configured identities stay private in status; safety latches do not change."""
import asyncio
import json
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
import robot_api


def run_case(case):
    async def run():
        rig = harness.Rig()
        try:
            assert await rig.settle()
            hidden = rig.ts.tricks['hold_pose']
            hidden.secret = True
            await case(rig, hidden)
        finally:
            rig.close()
            for _ in range(5):
                await asyncio.sleep(0)
    asyncio.run(run())


def test_fall_relocks_and_status_hides_exact_controller_but_retains_fall():
    async def case(rig, hidden):
        rig.ts.secret_gate.unlocked = True
        rig.robot.controllers[robot_api.LOCOMOTION_CONTROLLER] = 'inactive'
        rig.robot.controllers[hidden.controller] = 'active'
        rig.ts.state_now = rig.ts.state_after_queue = hidden
        await rig.ts._on_fall(hidden.controller)
        assert not rig.ts.secret_gate.unlocked
        result = await rig.ts.mode()
        assert hidden.name not in json.dumps(result.as_dict())
        assert 'tilt/fall stop recorded' in result.message
        assert result.details['state'] == robot_api.FALLEN
        assert rig.ts.fallen == hidden.controller
        assert hidden.name not in rig.ts.available_tricks()
    run_case(case)


@pytest.mark.parametrize('identifier', ['name', 'controller', 'walk_controller'])
def test_fault_status_masks_only_configured_identifiers_and_keeps_quarantine(identifier):
    async def case(rig, hidden):
        identity = getattr(hidden, identifier)
        fault = f'{identity} did not confirm a fresh final playback frame; motion quarantined. Inspect/restart the controller stack.'
        rig.ts.controller_fault = fault
        switches = list(rig.robot.switches)
        result = await rig.ts.mode()
        assert not result.ok
        assert identity not in json.dumps(result.as_dict())
        assert 'motion quarantined' in result.message
        assert 'Inspect/restart' in result.details['controller_fault']
        assert rig.ts.controller_fault == fault
        assert rig.ts.motion_block_reason() == fault
        blocked = await rig.ts.switch_to(robot_api.LOCOMOTION_CONTROLLER)
        assert not blocked.ok and rig.robot.switches == switches
    run_case(case)


def test_unlocked_status_retains_diagnostic_identity():
    async def case(rig, hidden):
        rig.ts.secret_gate.unlocked = True
        rig.ts.controller_fault = f'{hidden.controller}: playback failed; motion quarantined'
        result = await rig.ts.mode()
        assert hidden.controller in result.message
        assert result.details['controller_fault'] == rig.ts.controller_fault
    run_case(case)


def test_name_substrings_and_unrelated_diagnostics_are_not_rewritten():
    async def case(rig, hidden):
        fault = f'not_{hidden.name}_backup failed; unrelated controller motion quarantined'
        rig.ts.controller_fault = fault
        result = await rig.ts.mode()
        assert result.details['controller_fault'] == fault
        assert result.message.startswith(fault)
    run_case(case)
