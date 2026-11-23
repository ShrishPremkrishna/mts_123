"""Interactive endpoints must belong to this activation and this clip request."""
import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
from robot_api import LOCOMOTION_CONTROLLER


def run_case(case):
    async def run():
        rig = harness.Rig()
        try:
            await rig.settle()
            await case(rig)
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(run())


def current_activation(rig, controller):
    # The fake controller's snapshot, not its mutable next-activation parameter.
    return rig.robot.active_execution_tokens.get(controller, 'a' * 32)


def receipt(clip, holding, frame, activation, request='none'):
    state = 'holding' if holding else 'playing'
    return harness._DataMsg(f'{clip} {state} frame {frame} queued 0 '
                           f'execution {activation} request {request}')


@pytest.mark.parametrize('starting_mode', ['entry', 'walking_exit'])
def test_delayed_prior_activation_endpoint_cannot_complete_current_chain(starting_mode):
    async def case(rig):
        trick = rig.ts.tricks['hold_pose']
        if starting_mode == 'walking_exit':
            trick.walk_path = Path('test_walking_policy.json')
            rig.ts.leg_controllers.add(trick.walk_controller)
            rig.robot.controllers = {name: 'inactive' for name in rig.robot.controllers}
            rig.robot.controllers[trick.walk_controller] = 'active'
            rig.ts.state_now = rig.ts.state_after_queue = trick
        original = rig.robot.send
        def delayed_old_receipt(topic, message):
            if topic == f'/{trick.controller}/clip_status':
                endpoint = 'idle' if starting_mode == 'entry' else 'leave'
                original(topic, receipt(endpoint, starting_mode != 'entry',
                                        149 if starting_mode == 'entry' else 549, '0' * 32))
                return
            original(topic, message)
        rig.robot.send = delayed_old_receipt
        result = (await harness.behaviors.play_policy_trick(rig.ts, trick) if starting_mode == 'entry'
                  else await rig.ts.switch_to(LOCOMOTION_CONTROLLER))
        assert not result.ok, 'a previous activation endpoint was treated as current playback'
        assert rig.ts.controller_fault
        assert not rig.robot.activations(LOCOMOTION_CONTROLLER)
    run_case(case)


def test_old_request_endpoint_in_same_activation_cannot_authorize_exit():
    async def case(rig):
        trick = rig.ts.tricks['hold_pose']
        assert (await harness.behaviors.play_policy_trick(rig.ts, trick)).ok
        # Drop the current request. Deliver an older request's terminal receipt
        # in this same activation, which an activation-only guard cannot reject.
        rig.ts.clip_pubs[trick.controller].publish = lambda message: rig.robot.send(
            f'/{trick.controller}/clip_status',
            receipt('leave', True, 549, current_activation(rig, trick.controller), 'b' * 32))
        result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert not result.ok, 'an old request authorized default-controller handoff'
        assert rig.ts.controller_fault
        assert not rig.robot.activations(LOCOMOTION_CONTROLLER)
    run_case(case)


@pytest.mark.parametrize('fault_phase', ['playing', 'holding'])
def test_controller_fault_dominates_interactive_clip_receipt(fault_phase):
    async def case(rig):
        trick = rig.ts.tricks['hold_pose']
        assert (await harness.behaviors.play_policy_trick(rig.ts, trick)).ok
        fault = 'unresolved controller outcome'
        def publish(message):
            fields = message.data.split()
            request_id = fields[1] if len(fields) == 3 and fields[0] == 'pupper_clip_v1' else 'c' * 32
            activation = current_activation(rig, trick.controller)
            for phase, frame in [('playing', 399), ('holding', 549)]:
                if phase == fault_phase:
                    rig.ts.controller_fault = fault
                rig.robot.send(f'/{trick.controller}/clip_status',
                               receipt('leave', phase == 'holding', frame, activation, request_id))
        rig.ts.clip_pubs[trick.controller].publish = publish
        result = await rig.ts.play_clip(trick, 'leave')
        assert not result.ok and fault in result.message
    run_case(case)
