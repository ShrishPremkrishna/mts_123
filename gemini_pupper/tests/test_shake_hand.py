"""One named handshake through the real queue and fake ROS; no network/devices."""
import asyncio

import pytest

from test_session_context import installed_rig, context, call
import test_tools as harness
import tool_registry
from robot_api import LOCOMOTION, LOCOMOTION_CONTROLLER, Result
from operator_feedback import receipt_feedback
from secret_gate import SecretGate


def marker(result, paw):
    assert result.ok, result.message
    assert result.details['handshake'] == {
        'schema': 'pupper_handshake_submission_v1', 'paw': paw, 'action': 'shake_' + paw}
    submission = result.details['submission']
    assert submission['schema'] == 'pupper_submission_v1' and submission['state'] == 'queued'
    assert submission['job_id'].startswith(submission['server_instance'] + ':')
    assert receipt_feedback('shake_hand', result.as_dict()) == f'{paw.capitalize()} paw command queued.'
    return submission['job_id']


@pytest.mark.parametrize('paw', ['left', 'right'])
def test_standing_entry_and_final_paw_share_one_job_identity(paw):
    async def case():
        async with installed_rig() as rig:
            sit = rig.ts.tricks['sit_stay_shake']
            result = await tool_registry.call(rig.ts, 'shake_hand', {'paw': paw})
            job_id = marker(result, paw)
            assert rig.ts.commands.qsize() == 1
            assert rig.ts.state_after_queue is sit and context(rig)['completed_sit_entries'] == 0
            assert await rig.settle()
            assert rig.robot.clips(sit.controller) == ['shake_' + paw]
            assert len(rig.robot.activations(sit.controller)) == 1
            assert rig.ts.state_now is sit and rig.robot.active() == {sit.controller}
            assert context(rig)['completed_sit_entries'] == 1
            outcome = rig.ts.last_outcome
            assert outcome['job_id'] == job_id and outcome['server_instance'] == result.details['submission']['server_instance']
            assert outcome['terminal_state'] == 'succeeded' and 'shake_' + paw in outcome['message']
            assert len(rig.ts.recent_outcomes()['outcomes']) == 1
    asyncio.run(case())


@pytest.mark.parametrize('starting', ['already_sit', 'queued_sit', 'queued_exit'])
def test_projected_state_orders_entry_and_does_not_add_redundant_entries(starting):
    async def case():
        async with installed_rig() as rig:
            sit = rig.ts.tricks['sit_stay_shake']
            entry = await tool_registry.call(rig.ts, 'play_trick', {'name': sit.name, 'count': 1, 'count_unit': 'routine_playback'})
            if starting != 'queued_sit':
                assert await rig.settle()
            if starting == 'queued_exit':
                assert (await tool_registry.call(rig.ts, 'stand_up', {})).ok
            first = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'left'})
            second = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'right'})
            marker(first, 'left'); marker(second, 'right')
            assert await rig.settle()
            expected = ['stand_up', 'shake_left', 'shake_right'] if starting == 'queued_exit' else ['shake_left', 'shake_right']
            assert rig.robot.clips(sit.controller) == expected
            assert context(rig)['completed_sit_entries'] == (2 if starting == 'queued_exit' else 1)
            assert rig.ts.state_now is sit
            outcomes = rig.ts.recent_outcomes()['outcomes']
            assert outcomes[-2]['job_id'] == first.details['submission']['job_id']
            assert outcomes[-1]['job_id'] == second.details['submission']['job_id']
    asyncio.run(case())


@pytest.mark.parametrize('args', [{}, {'paw': 'both'}, {'paw': 'LEFT'}, {'paw': None}, {'paw': True}, {'paw': 'left', 'exit': True}, []])
def test_invalid_arguments_have_no_queue_follower_or_controller_effects(args):
    async def case():
        async with installed_rig() as rig:
            rig.ts.following = True
            before = (list(rig.robot.switches), list(rig.robot.follow_calls))
            result = await tool_registry.call(rig.ts, 'shake_hand', args)
            assert not result.ok and not result.details
            assert rig.ts.commands.empty() and rig.ts.following
            assert before == (rig.robot.switches, rig.robot.follow_calls)
    asyncio.run(case())


@pytest.mark.parametrize('missing', ['absent', 'locked', 'one_paw', 'wrong_endpoint', 'walking'])
def test_catalog_and_runtime_both_require_real_available_capability(missing):
    async def case():
        async with installed_rig() as rig:
            sit = rig.ts.tricks['sit_stay_shake']
            if missing == 'absent': rig.ts.tricks.clear()
            elif missing == 'locked': sit.secret = True
            elif missing == 'one_paw': sit.actions.pop('shake_right')
            elif missing == 'walking': sit.walk_path = __import__('pathlib').Path('synthetic-walker.json')
            else: sit.clip('shake_right')['next'] = 'stand_up'
            assert 'shake_hand' not in {d['name'] for d in tool_registry.declarations(rig.ts)}
            before = list(rig.robot.switches)
            result = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'left'})
            assert not result.ok and rig.ts.commands.empty() and rig.robot.switches == before
    asyncio.run(case())


def test_declared_paw_schema_has_only_two_choices_and_one_call_contract():
    async def case():
        async with installed_rig() as rig:
            declaration = next(d for d in tool_registry.declarations(rig.ts) if d['name'] == 'shake_hand')
            assert declaration['parameters']['required'] == ['paw']
            assert declaration['parameters']['properties']['paw']['enum'] == ['left', 'right']
            assert 'no prior status' in declaration['description']
    asyncio.run(case())


@pytest.mark.parametrize('failure', ['entry', 'paw', 'entry_exception'])
def test_failed_stage_never_completes_handshake_and_cancels_following_job(failure, monkeypatch):
    import part5_interactive_action as behavior
    async def case():
        async with installed_rig() as rig:
            sit = rig.ts.tricks['sit_stay_shake']
            if failure.startswith('entry'):
                async def bad_entry(*_args):
                    if failure == 'entry_exception': raise RuntimeError('synthetic entry failure')
                    return Result(False, 'synthetic entry refusal')
                monkeypatch.setattr(behavior, 'play_policy_trick', bad_entry)
            else:
                async def bad_paw(*_args): return Result(False, 'synthetic paw refusal')
                rig.ts.play_clip = bad_paw
            accepted = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'left'})
            job_id = marker(accepted, 'left')
            later = await tool_registry.call(rig.ts, 'move', {'direction': 'forward', 'speed': .2, 'duration': .5})
            assert later.ok and await rig.settle()
            outcomes = {o['job_id']: o for o in rig.ts.recent_outcomes()['outcomes']}
            assert outcomes[job_id]['terminal_state'] == 'failed'
            assert outcomes[later.details['submission']['job_id']]['terminal_state'] == 'cancelled'
            assert not harness.walk_twists(rig)
            assert context(rig)['completed_sit_entries'] == (1 if failure == 'paw' else 0)
            assert rig.robot.clips(sit.controller) == []
    asyncio.run(case())


@pytest.mark.parametrize('phase', ['queued', 'entry', 'paw'])
def test_stop_cancels_atomic_job_and_never_runs_a_late_paw(phase, monkeypatch):
    import part5_interactive_action as behavior
    async def case():
        async with installed_rig() as rig:
            sit = rig.ts.tricks['sit_stay_shake']; entered = asyncio.Event()
            if phase == 'entry':
                async def held_entry(*_args):
                    entered.set(); await asyncio.Event().wait()
                monkeypatch.setattr(behavior, 'play_policy_trick', held_entry)
            elif phase == 'paw':
                original = rig.ts.play_clip
                async def held_paw(*_args):
                    entered.set(); await asyncio.Event().wait()
                    return await original(*_args)
                rig.ts.play_clip = held_paw
            accepted = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'left'})
            job_id = marker(accepted, 'left')
            if phase != 'queued':
                await asyncio.wait_for(entered.wait(), 2)
                assert rig.ts.current_job_id == job_id
                assert all(o['job_id'] != job_id for o in rig.ts.recent_outcomes()['outcomes'])
            stopped = await tool_registry.call(rig.ts, 'stop', {})
            assert stopped.ok and await rig.settle()
            outcomes = {o['job_id']: o for o in rig.ts.recent_outcomes()['outcomes']}
            assert outcomes[job_id]['terminal_state'] == 'cancelled'
            assert rig.robot.clips(sit.controller) == []
            assert context(rig)['completed_sit_entries'] == (1 if phase == 'paw' else 0)
    asyncio.run(case())


@pytest.mark.parametrize('when', ['before_start', 'after_entry'])
def test_secret_relock_prevents_paw_without_undoing_confirmed_entry(when, monkeypatch, tmp_path):
    import part5_interactive_action as behavior
    async def case():
        async with installed_rig() as rig:
            sit = rig.ts.tricks['sit_stay_shake']; sit.secret = True
            password_file = tmp_path / 'secret-password'
            password_file.write_text('disposable-test-password')
            rig.ts.secret_gate = SecretGate(password_file)
            assert rig.ts.secret_gate.unlock('disposable-test-password')
            original = behavior.play_policy_trick
            if when == 'after_entry':
                async def relocked(*args):
                    result = await original(*args); rig.ts.secret_gate.lock(); return result
                monkeypatch.setattr(behavior, 'play_policy_trick', relocked)
            accepted = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'right'})
            marker(accepted, 'right')
            if when == 'before_start': rig.ts.secret_gate.lock()
            assert await rig.settle()
            assert not rig.ts.last_outcome['success'] and rig.robot.clips(sit.controller) == []
            assert context(rig)['completed_sit_entries'] == (1 if when == 'after_entry' else 0)
    asyncio.run(case())


@pytest.mark.parametrize('mode', ['unknown', 'stale_sit', 'external_sit'])
def test_unknown_or_changed_active_ownership_refuses_without_motion(mode):
    async def case():
        async with installed_rig() as rig:
            sit = rig.ts.tricks['sit_stay_shake']
            if mode == 'unknown': rig.robot.controllers['foreign_motor_owner'] = 'active'
            elif mode == 'stale_sit': rig.ts.state_now = rig.ts.state_after_queue = sit
            else:
                rig.robot.controllers[LOCOMOTION_CONTROLLER] = 'inactive'
                rig.robot.controllers[sit.controller] = 'active'
            before = list(rig.robot.switches)
            result = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'left'})
            marker(result, 'left')
            assert await rig.settle()
            assert not rig.ts.last_outcome['success']
            assert rig.robot.switches == before and rig.robot.clips(sit.controller) == []
    asyncio.run(case())


def test_following_stops_before_queue():
    async def case():
        async with installed_rig() as rig:
            assert (await tool_registry.call(rig.ts, 'follow', {'enabled': True})).ok
            result = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'right'})
            marker(result, 'right')
            assert not rig.ts.following and 'follow' in result.message
            assert await rig.settle()
    asyncio.run(case())


@pytest.mark.parametrize('patch', [
    {'handshake': None}, {'handshake': {'schema': 'bad', 'paw': 'left', 'action': 'shake_left'}},
    {'handshake': {'schema': 'pupper_handshake_submission_v1', 'paw': 'left', 'action': 'shake_right'}},
    {'submission': {'schema': 'pupper_submission_v1', 'state': 'queued'}},
    {'submission': {'schema': 'pupper_submission_v1', 'state': 'queued', 'server_instance': 'a'*32, 'job_id': 'b'*32+':1'}},
])
def test_malformed_compound_or_job_identity_cannot_authorize_paw_wording(patch):
    details = {'handshake': {'schema': 'pupper_handshake_submission_v1', 'paw': 'left', 'action': 'shake_left'},
               'submission': {'schema': 'pupper_submission_v1', 'state': 'queued', 'server_instance': 'a'*32, 'job_id': 'a'*32+':1'}}
    details.update(patch)
    assert receipt_feedback('shake_hand', {'success': True, 'details': details}) != 'Left paw command queued.'


def test_unconfirmed_follower_stop_prevents_any_handshake_queueing():
    async def case():
        async with installed_rig() as rig:
            rig.ts.following = True
            async def failed_stop(): return Result(False, 'synthetic unconfirmed following stop')
            rig.ts.stop_following = failed_stop
            before = list(rig.robot.switches)
            result = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'left'})
            assert not result.ok and result.details is None
            assert rig.ts.commands.empty() and rig.robot.switches == before
    asyncio.run(case())


@pytest.mark.parametrize('when', ['admission', 'after_entry'])
def test_fall_prevents_handshake_or_remaining_paw(when, monkeypatch):
    import part5_interactive_action as behavior
    async def case():
        async with installed_rig() as rig:
            sit = rig.ts.tricks['sit_stay_shake']
            if when == 'admission': rig.ts.fallen = sit.controller
            else:
                original = behavior.play_policy_trick
                async def fallen(*args):
                    result = await original(*args); rig.ts.fallen = sit.controller; return result
                monkeypatch.setattr(behavior, 'play_policy_trick', fallen)
            result = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'left'})
            if when == 'admission': assert not result.ok and rig.ts.commands.empty()
            else:
                marker(result, 'left'); assert await rig.settle(); assert not rig.ts.last_outcome['success']
            assert rig.robot.clips(sit.controller) == []
    asyncio.run(case())


def test_adopted_active_sit_gets_paw_without_entry_or_counter_increment():
    async def case():
        async with installed_rig(adopt_sit=True) as rig:
            sit = rig.ts.tricks['sit_stay_shake']
            before = list(rig.robot.switches)
            result = await tool_registry.call(rig.ts, 'shake_hand', {'paw': 'right'})
            marker(result, 'right')
            assert await rig.settle()
            assert rig.ts.last_outcome['success']
            assert rig.robot.switches == before and rig.robot.clips(sit.controller) == ['shake_right']
            assert context(rig)['completed_sit_entries'] == 0
    asyncio.run(case())
