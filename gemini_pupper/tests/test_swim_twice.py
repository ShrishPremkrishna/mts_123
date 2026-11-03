"""Lab 6 Q8: one bounded swim-pair submission through the real queue, fake ROS.

The supplied swim descriptor/policy is used. This validates controller ordering
and receipts, not physical double-playback balance or observed swimming strokes.
"""
import asyncio
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
import tool_registry
from robot_api import Result, LOCOMOTION_CONTROLLER
from tricks import load_tricks
from trick_quantity import validate_trick_quantity


def run_case(case):
    async def run():
        swim = load_tricks()['swim']
        rig = harness.Rig(tricks=lambda: {'swim': swim})
        try:
            assert await rig.settle()
            await case(rig, swim)
        finally:
            rig.close()
            for _ in range(5): await asyncio.sleep(0)
    asyncio.run(run())


async def submit_pair(rig):
    return await tool_registry.call(rig.ts, 'play_trick',
                                   {'name': 'swim', 'count': 2, 'count_unit': 'routine_playback'})


def test_swim_twice_is_one_admission_two_fresh_playbacks_and_holds_lying_endpoint():
    async def case(rig, swim):
        confirmations = []
        confirm = rig.ts.confirm_finite_endpoint
        async def record(trick):
            token = rig.ts.policy_execution_requests[trick.controller]
            result = await confirm(trick)
            confirmations.append((token, result.ok))
            return result
        rig.ts.confirm_finite_endpoint = record
        before = rig.clock.now
        sequence = rig.ts._job_sequence
        result = await submit_pair(rig)
        assert result.ok, result.message
        assert '2 routine playbacks' in result.message and 'accepted, not completed' in result.message
        assert result.details['submission']['state'] == 'queued'
        assert rig.ts._job_sequence == sequence + 1
        assert rig.ts.busy_until - before == pytest.approx(2 * (swim.play_time + .3))
        assert rig.ts.state_after_queue is swim
        assert await rig.settle()
        activations = rig.robot.activations(swim.controller)
        assert len(activations) == 2
        # Real finite-runner branch must release the previous endpoint before re-entry.
        assert len(rig.robot.deactivations(swim.controller)) == 1
        assert activations[0] < rig.robot.deactivations(swim.controller)[0] <= activations[1]
        assert activations[1] - activations[0] >= swim.play_time + .3 - 1e-9
        assert len(confirmations) == 2 and all(ok for _, ok in confirmations)
        assert confirmations[0][0] != confirmations[1][0]
        assert rig.robot.active() == {swim.controller}
        assert not rig.robot.activations(LOCOMOTION_CONTROLLER), 'No unrequested standing between/after swim'
        assert rig.ts.last_outcome['success'] and rig.ts.last_outcome['job_id'] == result.details['submission']['job_id']
        assert '2 routine playbacks' in rig.ts.last_outcome['message']
        assert not rig.failures()
    run_case(case)


def test_swim_pair_allows_separately_requested_stand_only_after_second_endpoint():
    async def case(rig, swim):
        first = await submit_pair(rig)
        assert first.ok, first.message
        second = await tool_registry.call(rig.ts, 'stand_up', {})
        assert second.ok
        assert first.details['submission']['job_id'] != second.details['submission']['job_id']
        assert await rig.settle()
        activations = rig.robot.activations(swim.controller)
        assert len(activations) == 2
        assert rig.robot.activations(LOCOMOTION_CONTROLLER)[0] >= activations[1] + swim.play_time + .3
        assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
        assert not rig.failures()
    run_case(case)


@pytest.mark.parametrize('when', ['during_first', 'between_playbacks'])
def test_stop_cancels_remaining_swim_and_dependent_stand(when):
    async def case(rig, swim):
        barrier = asyncio.Event()
        if when == 'during_first':
            sleep = rig.ts.sleep
            async def hold(seconds):
                if seconds > 1 and swim.controller in rig.robot.active():
                    await sleep(.1)
                    barrier.set()
                    await asyncio.Event().wait()
                else:
                    await sleep(seconds)
            rig.ts.sleep = hold
        else:
            confirm = rig.ts.confirm_finite_endpoint
            async def hold(trick):
                result = await confirm(trick)
                assert result.ok
                barrier.set()
                await asyncio.Event().wait()
            rig.ts.confirm_finite_endpoint = hold
        first = await submit_pair(rig)
        assert first.ok, first.message
        assert (await tool_registry.call(rig.ts, 'stand_up', {})).ok
        for _ in range(2000):
            if barrier.is_set(): break
            await asyncio.sleep(0)
        assert barrier.is_set()
        assert (await tool_registry.call(rig.ts, 'stop', {})).ok
        assert await rig.settle()
        assert len(rig.robot.activations(swim.controller)) == 1
        assert not rig.robot.activations(LOCOMOTION_CONTROLLER)
        assert not rig.robot.active()
        outcomes = rig.ts.recent_outcomes()['outcomes']
        pair = next(item for item in outcomes if item['job_id'] == first.details['submission']['job_id'])
        assert pair['terminal_state'] == 'cancelled' and not pair['success']
    run_case(case)


def test_first_endpoint_failure_aborts_second_playback_and_later_stand():
    async def case(rig, swim):
        async def failed_endpoint(trick):
            return Result(False, 'injected missing endpoint receipt')
        rig.ts.confirm_finite_endpoint = failed_endpoint
        first = await submit_pair(rig)
        assert first.ok, first.message
        assert (await tool_registry.call(rig.ts, 'stand_up', {})).ok
        assert await rig.settle()
        assert len(rig.robot.activations(swim.controller)) == 1
        assert not rig.robot.activations(LOCOMOTION_CONTROLLER)
        assert not rig.ts.last_outcome['success']
        assert 'playback 1/2 failed' in rig.ts.last_outcome['message']
        assert rig.ts.status()['queued'] == 0
    run_case(case)


@pytest.mark.parametrize('change', ['stopped', 'locked', 'fallen'])
def test_between_playback_gate_is_rechecked_before_any_second_activation(change):
    async def case(rig, swim):
        confirm = rig.ts.confirm_finite_endpoint
        async def change_gate(trick):
            result = await confirm(trick)
            assert result.ok
            if change == 'stopped': rig.ts.stops += 1
            elif change == 'locked': rig.ts.trick_available = lambda trick: False
            else: rig.ts.fallen = 'injected tilt stop'
            return result
        rig.ts.confirm_finite_endpoint = change_gate
        first = await submit_pair(rig)
        assert first.ok, first.message
        assert await rig.settle()
        assert len(rig.robot.activations(swim.controller)) == 1
        assert not rig.ts.last_outcome['success']
    run_case(case)


@pytest.mark.parametrize('count,unit', [(3,'routine_playback'), (10,'routine_playback'),
                                       (2,'physical_repetition'), (True,'routine_playback')])
def test_unsupported_swim_quantity_has_no_prepare_or_queue_side_effect(count, unit):
    async def case(rig, swim):
        rig.ts.following = True
        switches = list(rig.robot.switches)
        before = rig.ts._job_sequence
        result = await tool_registry.call(rig.ts, 'play_trick', {'name':'swim','count':count,'count_unit':unit})
        assert not result.ok
        assert rig.ts.following and rig.robot.switches == switches
        assert rig.ts._job_sequence == before
    run_case(case)


@pytest.mark.parametrize('changes', [{'name':'push_up'}, {'kind':'reference'}, {'interactive':True},
                                     {'walks':True}, {'end_pose':'standing'}])
def test_swim_exception_requires_exact_finite_nonwalking_lying_routine(changes):
    attributes = dict(name='swim', kind='policy', interactive=False, walks=False, end_pose='lying')
    good = SimpleNamespace(**attributes)
    assert validate_trick_quantity(2, 'routine_playback', trick=good) is None
    attributes.update(changes)
    result = validate_trick_quantity(2, 'routine_playback', trick=SimpleNamespace(**attributes))
    assert not result.ok and result.details['refusal']['code'] == 'playback_count_unsupported'


def test_swim_changed_descriptor_refuses_before_leaving_follow():
    async def case(rig, swim):
        rig.ts.tricks['swim'] = replace(swim, end_pose='standing')
        rig.ts.following = True
        first = await submit_pair(rig)
        assert not first.ok
        assert rig.ts.following and rig.ts._job_sequence == 0
        assert not rig.robot.activations(swim.controller)
    run_case(case)
