"""Truthful status and idle-only queue checks through fake ROS; no hardware."""
import asyncio
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
            await rig.settle()
            await case(rig)
        finally:
            rig.close()
            for _ in range(5):
                await asyncio.sleep(0)
    asyncio.run(run())


@pytest.mark.parametrize('controller', ['neural_controller_default', 'neural_controller_student',
                                        'unfamiliar_joint_controller'])
def test_status_names_active_controller_without_inventing_behavior(controller):
    async def case(rig):
        rig.robot.controllers = {controller: 'active'}
        switches = list(rig.robot.switches)
        result = await rig.ts.mode()
        assert result.ok and controller in result.message
        assert 'no active' not in result.message
        assert 'physical stopping and posture have not been measured' in result.message
        if controller == 'unfamiliar_joint_controller':
            assert 'unclassified active controllers' in result.message
            assert 'leg ownership is not established' in result.message
        else:
            assert 'active leg controllers' in result.message
        assert rig.robot.switches == switches
    run_case(case)


def test_feedback_only_inventory_is_not_mistaken_for_known_leg_ownership():
    async def case(rig):
        rig.robot.controllers = {'joint_state_broadcaster': 'active',
                                 'imu_sensor_broadcaster': 'active'}
        result = await rig.ts.mode()
        assert result.ok and 'unclassified active controllers' in result.message
        assert 'leg ownership is not established' in result.message
        assert 'joint_state_broadcaster' in result.message
    run_case(case)


def test_status_reports_no_active_legs_only_when_no_active_controller_is_reported():
    async def case(rig):
        rig.robot.controllers = {'neural_controller_default': 'inactive',
                                 'unfamiliar_joint_controller': 'inactive'}
        result = await rig.ts.mode()
        assert result.ok and 'no active leg controller reported' in result.message
        assert 'physical stopping and posture have not been measured' in result.message
    run_case(case)


def test_known_locomotion_does_not_hide_additional_active_controllers():
    async def case(rig):
        rig.robot.controllers.update(neural_controller_default='active', unfamiliar_joint_controller='active')
        result = await rig.ts.mode()
        assert result.ok and 'locomotion' in result.message
        assert 'Also active leg controllers: neural_controller_default' in result.message
        assert 'Other active controllers: unfamiliar_joint_controller' in result.message
    run_case(case)


async def latch(rig):
    rig.robot.stop_status.update(latched=True, stop_required=True,
                                epoch=rig.robot.stop_status['epoch'] + 1)
    rig.robot.emit_stop_status()
    # Finish the one coalesced Stop cleanup before attempting new stationary work.
    for _ in range(20):
        await asyncio.sleep(0)
    assert not rig.ts._stop_cleanup_scheduled


@pytest.mark.parametrize('controller', ['neural_controller_default', 'unfamiliar_joint_controller', None])
def test_latched_status_stays_a_blocked_result_not_a_physical_stop_claim(controller):
    async def case(rig):
        rig.robot.controllers = {'neural_controller_default': 'inactive'}
        if controller:
            rig.robot.controllers[controller] = 'active'
        await latch(rig)
        result = await rig.ts.mode()
        assert not result.ok and 'shared stop authority blocks motion' in result.message
        assert 'not measured physical stopping' in result.message
        assert 'no active' not in result.message
    run_case(case)


def test_latched_idle_wait_completes_without_activation_or_nonzero_commands():
    async def case(rig):
        assert (await rig.ts.switch_to(None)).ok
        rig.ts.state_after_queue = robot_api.MOTORS_IDLE
        await latch(rig)
        switches = list(rig.robot.switches)
        result = await rig.ts.wait(.25)
        assert result.ok, result.message
        assert await rig.settle()
        assert rig.ts.last_outcome == {'name': 'wait 0.25s', 'success': True, 'message': 'waited 0.25 s'}
        assert rig.ts.state_now == rig.ts.state_after_queue == robot_api.MOTORS_IDLE
        assert rig.robot.switches == switches
        assert rig.ts.motion_block_reason()
        assert not harness.walk_twists(rig)
    run_case(case)


@pytest.mark.parametrize('state', [robot_api.LOCOMOTION, 'trick_state'])
def test_latched_wait_that_preserves_an_active_pose_is_still_refused(state):
    async def case(rig):
        if state == 'trick_state':
            selected = rig.ts.tricks['hold_pose']
            rig.ts.state_now = rig.ts.state_after_queue = selected
            rig.robot.controllers[robot_api.LOCOMOTION_CONTROLLER] = 'inactive'
            rig.robot.controllers[selected.controller] = 'active'
        await latch(rig)
        switches = list(rig.robot.switches)
        result = await rig.ts.wait(.25)
        assert not result.ok and 'shared stop authority blocks motion' in result.message
        assert rig.ts.commands.empty()
        assert rig.robot.switches == switches
    run_case(case)


@pytest.mark.parametrize('change', ['local_state', 'stock_controller', 'unknown_controller', 'inventory_missing'])
def test_queued_idle_wait_rechecks_mode_and_actual_inventory_before_execution(change):
    async def case(rig):
        assert (await rig.ts.switch_to(None)).ok
        rig.ts.state_after_queue = robot_api.MOTORS_IDLE
        await latch(rig)
        assert (await rig.ts.wait(.25)).ok
        # No event-loop yield: this change happens after admission, before job execution.
        if change == 'local_state':
            rig.ts.state_now = robot_api.LOCOMOTION
        elif change in ('stock_controller', 'unknown_controller'):
            name = 'neural_controller_default' if change == 'stock_controller' else 'unfamiliar_joint_controller'
            rig.robot.controllers[name] = 'active'
        else:
            original = rig.ts._call
            async def missing(client, request, timeout=3.):
                if client is rig.ts.list_client:
                    return None
                return await original(client, request, timeout)
            rig.ts._call = missing
        switches = list(rig.robot.switches)
        assert await rig.settle()
        assert not rig.ts.last_outcome['success']
        assert 'idle wait could not be confirmed' in rig.ts.last_outcome['message']
        assert rig.robot.switches == switches
        assert not harness.walk_twists(rig)
    run_case(case)


def test_latched_idle_wait_allows_exact_feedback_types_without_assuming_names():
    async def case(rig):
        assert (await rig.ts.switch_to(None)).ok
        rig.ts.state_after_queue = robot_api.MOTORS_IDLE
        await latch(rig)
        original = rig.ts._call
        async def feedback(client, request, timeout=3.):
            response = await original(client, request, timeout)
            if client is rig.ts.list_client and response is not None:
                response.controller.extend([
                    harness._Msg(name='joint_feedback', state='active', type='joint_state_broadcaster/JointStateBroadcaster'),
                    harness._Msg(name='imu_feedback', state='active', type='imu_sensor_broadcaster/IMUSensorBroadcaster'),
                ])
            return response
        rig.ts._call = feedback
        assert (await rig.ts.wait(.25)).ok
        assert await rig.settle()
        assert rig.ts.last_outcome['success']
    run_case(case)
