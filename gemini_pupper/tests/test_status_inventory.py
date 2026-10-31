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
        assert 'Physical pose, speed and stopping have not been measured' in result.message
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
        assert 'Physical pose, speed and stopping have not been measured' in result.message
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
        assert 'Physical pose, speed and stopping have not been measured' in result.message
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


def test_completed_move_status_distinguishes_commands_from_physical_motion():
    async def case(rig):
        accepted = await rig.call("move", direction="forward", speed=.2, duration=.1)
        assert accepted.ok
        assert "accepted, not completed" in accepted.message
        assert rig.ts.last_outcome["success"]
        status = await rig.ts.mode()
        assert "locomotion controller active" in status.message
        assert "software command completed" in status.message
        assert "sent velocity commands" in status.message
        assert "then requested zero velocity" in status.message
        assert "moved vx=" not in status.message
        assert "Physical pose, speed and stopping have not been measured" in status.message
        assert "standing still" not in status.message
    run_case(case)


@pytest.mark.parametrize("name", ["hold_pose", "roll_over"])
def test_active_trick_status_labels_its_target_pose_not_measured_posture(name):
    async def case(rig):
        trick = rig.ts.tricks[name]
        rig.robot.controllers[robot_api.LOCOMOTION_CONTROLLER] = "inactive"
        rig.robot.controllers[trick.controller] = "active"
        status = await rig.ts.mode()
        assert status.ok
        assert "controller active" in status.message
        assert "configured target pose" in status.message
        assert "Physical pose, speed and stopping have not been measured" in status.message
        assert "holding its end pose" not in status.message
    run_case(case)


def test_fall_status_reports_detected_stop_and_recovery_command_without_pose_claim():
    async def case(rig):
        rig.ts.fallen = robot_api.LOCOMOTION_CONTROLLER
        status = await rig.ts.mode()
        assert "tilt/fall stop recorded" in status.message
        assert "Physical pose, speed and stopping have not been measured" in status.message
        assert "motors only damp" not in status.message
        rig.imu(0, seconds=2)
        recovered = await rig.ts.switch_to(robot_api.LOCOMOTION_CONTROLLER)
        assert recovered.ok
        assert "recovery command completed" in recovered.message
        assert "stood back up" not in recovered.message
    run_case(case)


def test_stand_declaration_does_not_require_an_extra_stand_before_moving():
    async def case(rig):
        import tool_registry
        description = next(d["description"] for d in tool_registry.declarations(rig.ts)
                           if d["name"] == "stand_up")
        assert "Needed before moving" not in description
        assert "Move, turn and arc already prepare" in description
        assert "unless requested" in description
    run_case(case)


@pytest.mark.parametrize("name", ["hold_pose", "roll_over", "bow"])
def test_trick_completion_reports_controller_playback_and_distinguishes_repeat(name):
    async def case(rig):
        import tool_registry
        assert (await tool_registry.call(rig.ts, "play_trick", {"name": name})).ok
        assert await rig.settle()
        outcome = rig.ts.last_outcome
        assert outcome["success"], outcome
        assert "controller" in outcome["message"] and "playback" in outcome["message"]
        assert "it now stays" not in outcome["message"]
        assert "holds its final pose" not in outcome["message"]
        if name == "hold_pose":
            assert "entry playback confirmed" in outcome["message"]
            assert (await tool_registry.call(rig.ts, "play_trick", {"name": name})).ok
            assert await rig.settle()
            assert "no new entry playback" in rig.ts.last_outcome["message"]
    run_case(case)


def test_result_details_are_additive_and_survive_note_wrapping():
    assert robot_api.Result(True, 'legacy').as_dict() == {'success': True, 'message': 'legacy'}
    facts = {'physical_observation': {'pose': None, 'speed': None, 'stopped': None}}
    result = robot_api.with_note('Note.', robot_api.Result(True, 'status', details=facts))
    assert result.as_dict() == {'success': True, 'message': 'Note. status', 'details': facts}


@pytest.mark.parametrize('ending', ['success', 'refusal', 'exception', 'cancel'])
def test_current_action_label_follows_owned_job_and_clears_on_every_ending(ending):
    async def case(rig):
        release = asyncio.Event()
        async def job():
            await release.wait()
            if ending == 'exception':
                raise RuntimeError('synthetic offline failure')
            return robot_api.Result(ending != 'refusal', 'test result')
        assert (await rig.ts.queue('bounded test job', job, 1., then=robot_api.UNCHANGED)).ok
        for _ in range(5):
            await asyncio.sleep(0)
        running = rig.ts.status()
        assert running['current_action'] == {'name': 'bounded test job', 'state': 'running'}
        if ending == 'cancel':
            await rig.ts.immediate_stop()
        else:
            release.set()
        assert await rig.settle()
        ended = rig.ts.status()
        assert ended['current_action'] is None
        assert not ended['running']
        assert ended['last_outcome']['name'] == 'bounded test job'
        assert ended['last_outcome']['success'] is (ending == 'success')
        # The next action must replace, not inherit, the previous label.
        await rig.ts.wait(.1)
        for _ in range(3):
            await asyncio.sleep(0)
        assert rig.ts.status()['current_action']['name'] == 'wait 0.1s'
        assert await rig.settle()
        assert rig.ts.status()['current_action'] is None
    run_case(case)


def test_locked_current_action_details_do_not_leak_hidden_trick_names():
    async def case(rig):
        rig.ts.tricks['hold_pose'].secret = True
        release = asyncio.Event()
        async def job():
            await release.wait()
            return robot_api.Result(True, 'finished')
        assert (await rig.ts.queue('hold_pose wave_left', job, 1., then=robot_api.UNCHANGED)).ok
        for _ in range(3):
            await asyncio.sleep(0)
        status = await rig.ts.mode()
        assert status.as_dict()['details']['current_action'] == {'name': 'locked trick', 'state': 'running'}
        assert 'hold_pose' not in str(status.as_dict())
        assert 'wave_left' not in str(status.as_dict())
        release.set()
        assert await rig.settle()
    run_case(case)


def test_unlock_and_lock_report_their_commands_not_unmeasured_motion(tmp_path, monkeypatch):
    import tool_registry
    password = tmp_path / 'fake_secret'
    password.write_text('offline-test-only')
    monkeypatch.setenv('PUPPER_SECRET_FILE', str(password))
    async def case(rig):
        release = asyncio.Event()
        async def running_command():
            rig.ts.send_speed(.2, 0, 0)
            try:
                await release.wait()
                return robot_api.Result(True, 'test command ended')
            finally:
                rig.ts.send_speed(0, 0, 0)
        await rig.ts.queue('unrelated moving command', running_command, 1., then=robot_api.UNCHANGED)
        for _ in range(3):
            await asyncio.sleep(0)
        assert rig.ts.status()['current_action']['name'] == 'unrelated moving command'
        before = len(rig.robot.published)
        unlocked = await tool_registry.call(rig.ts, 'unlock', {'password': 'offline-test-only'})
        assert unlocked.ok and rig.ts.secret_gate.unlocked
        assert 'No motion command was issued by unlock' in unlocked.message
        assert 'Nothing moved' not in unlocked.message
        locked = await tool_registry.call(rig.ts, 'lock_tricks', {})
        assert locked.ok and not rig.ts.secret_gate.unlocked
        assert 'No motion command was issued by lock' in locked.message
        assert len(rig.robot.published) == before
        assert rig.ts.status()['current_action']['name'] == 'unrelated moving command'
        release.set()
        assert await rig.settle()
    run_case(case)
