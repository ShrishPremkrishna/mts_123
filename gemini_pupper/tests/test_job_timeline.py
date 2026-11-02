"""Authoritative queued-job lifecycle evidence; fake ROS only, no hardware/network."""
import asyncio
import json
import math
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
import robot_api
import tool_registry


def run_case(case):
    async def run():
        rig = harness.Rig()
        try:
            assert await rig.settle()
            await case(rig)
        finally:
            rig.close()
            for _ in range(5):
                await asyncio.sleep(0)
    asyncio.run(run())


def events(rig):
    return [json.loads(record.getMessage()[10:]) for record in rig.records
            if record.getMessage().startswith('job_event ')]


def job_id(reply):
    marker = reply.details['submission']
    assert marker['schema'] == 'pupper_submission_v1' and marker['state'] == 'queued'
    assert marker['job_id'].startswith(marker['server_instance'] + ':')
    return marker['job_id']


async def held_job(rig, *, suppress=False):
    entered, release = asyncio.Event(), asyncio.Event()
    async def job():
        entered.set()
        try:
            await release.wait()
        except asyncio.CancelledError:
            if not suppress:
                raise
            return robot_api.Result(False, 'fixture suppressed cancellation')
        return robot_api.Result(True, 'software operation completed; physical outcome unmeasured')
    reply = await rig.ts.queue('bounded job', job, 1, then=robot_api.UNCHANGED)
    await entered.wait()
    return reply, release


def test_same_id_flows_from_actual_acceptance_through_current_and_terminal():
    async def case(rig):
        reply, release = await held_job(rig)
        identifier = job_id(reply)
        running = rig.ts.status()['current_action']
        assert running['job_id'] == identifier and running['execution_started'] is True
        assert running['server_instance'] == rig.ts.command_instance
        assert [e['event'] for e in events(rig)] == ['queued', 'started']
        assert rig.ts.last_outcome is None
        release.set(); assert await rig.settle()
        ended = rig.ts.status()['last_outcome']
        assert ended['job_id'] == identifier and ended['terminal_state'] == 'succeeded'
        assert ended['success'] is True and ended['execution_started'] is True
        rows = events(rig)
        assert [e['event'] for e in rows] == ['queued', 'started', 'finished']
        assert [e['event_seq'] for e in rows] == [1, 2, 3]
        assert [e['outcome'] for e in rows] == [None, None, 'succeeded']
        for row in rows:
            assert row['schema'] == 'pupper_job_event_v1'
            assert row['job_id'] == identifier and row['server_instance'] == rig.ts.command_instance
            assert all(math.isfinite(row[field]) for field in ('wall_time_s', 'monotonic_s'))
            assert 'message' not in row and 'args' not in row
        assert rows[0]['monotonic_s'] <= rows[1]['monotonic_s'] <= rows[2]['monotonic_s']
        assert rig.ts.recent_outcomes()['outcomes'] == [ended]
        assert 'recent_outcomes' not in rig.ts.status()
    run_case(case)


def test_stop_matches_running_and_pending_ids_and_confirms_them_separately():
    async def case(rig):
        first, _ = await held_job(rig)
        second = await rig.ts.wait(1.)
        first_id, second_id = job_id(first), job_id(second)
        async def instant_follow():
            return robot_api.Result(True, 'already off')
        rig.ts.stop_following = instant_follow
        stopped = await rig.ts.immediate_stop()
        assert stopped.details['running_job']['job_id'] == first_id
        assert stopped.details['running_job']['terminal_state'] == 'pending'
        assert stopped.details['pending_queued_commands_cleared'] == 1
        assert stopped.details['pending_queued_job_ids_cleared'] == [second_id]
        rows = events(rig)
        pending = [e for e in rows if e['job_id'] == second_id]
        assert [(e['event'], e['outcome'], e['execution_started']) for e in pending] == [
            ('queued', None, False), ('finished', 'cancelled', False)]
        assert pending[-1]['reason'] == 'stop'
        current = [e for e in rows if e['job_id'] == first_id]
        assert current[-1]['event'] == 'cancel_requested' and current[-1]['outcome'] is None
        assert not any(e['event'] == 'finished' for e in current)
        assert await rig.settle()
        current = [e for e in events(rig) if e['job_id'] == first_id]
        assert current[-1]['event'] == 'finished' and current[-1]['outcome'] == 'cancelled'
        assert current[-1]['execution_started'] is True
        assert len([e for e in current if e['event'] == 'finished']) == 1
        assert rig.ts.status()['physical_observation'] == {'pose': None, 'speed': None, 'stopped': None}
    run_case(case)


def test_suppressed_cancel_is_failed_not_confirmed_cancelled():
    async def case(rig):
        reply, _ = await held_job(rig, suppress=True)
        identifier = job_id(reply)
        await rig.ts.immediate_stop(); assert await rig.settle()
        rows = [e for e in events(rig) if e['job_id'] == identifier]
        assert any(e['event'] == 'cancel_requested' for e in rows)
        assert rows[-1]['outcome'] == 'failed'
        assert not any(e['outcome'] == 'cancelled' for e in rows)
    run_case(case)


@pytest.mark.parametrize('exception', [False, True])
def test_failed_job_cancels_dependent_pending_job_without_execution(exception):
    async def case(rig):
        entered = []
        async def fail():
            if exception: raise ValueError('fixture error')
            return robot_api.Result(False, 'fixture refusal')
        async def later():
            entered.append(True); return robot_api.Result(True, 'must not run')
        first = await rig.ts.queue('first', fail, 1, then=robot_api.UNCHANGED)
        second = await rig.ts.queue('dependent', later, 1, then=robot_api.UNCHANGED)
        assert await rig.settle() and not entered
        outcomes = {r['job_id']: r for r in rig.ts.recent_outcomes()['outcomes']}
        assert outcomes[job_id(first)]['terminal_state'] == 'failed'
        assert outcomes[job_id(second)]['terminal_state'] == 'cancelled'
        assert outcomes[job_id(second)]['execution_started'] is False
        cleared = [e for e in events(rig) if e['job_id'] == job_id(second) and e['event'] == 'finished']
        assert cleared[0]['reason'] == 'dependency_failed'
        assert rig.ts.last_outcome['job_id'] == job_id(first)
    run_case(case)


def test_guard_refusal_records_no_execution_started():
    async def case(rig):
        ran = []
        async def job(): ran.append(True); return robot_api.Result(True, 'must not run')
        reply = await rig.ts.queue('guarded', job, 1, then=robot_api.LOCOMOTION)
        rig.ts.controller_fault = 'fixture quarantine'
        assert await rig.settle() and not ran
        rows = [e for e in events(rig) if e['job_id'] == job_id(reply)]
        assert [e['event'] for e in rows] == ['queued', 'finished']
        assert rows[-1]['outcome'] == 'failed' and rows[-1]['execution_started'] is False
    run_case(case)


def test_refusal_or_failed_insertion_emits_no_queued_marker_or_event():
    async def case(rig):
        refused = await rig.ts.wait(-1)
        assert not refused.ok and not events(rig)
        async def unavailable(_): raise RuntimeError('fixture insertion failure')
        rig.ts.commands.put = unavailable
        reply = await tool_registry.call(rig.ts, 'wait', {'duration': 1})
        assert not reply.ok and not (reply.details or {}).get('submission')
        assert not events(rig) and not rig.ts.recent_outcomes()['outcomes']
    run_case(case)


def test_history_bounded_and_same_named_jobs_remain_distinct():
    async def case(rig):
        accepted = [job_id(await rig.ts.wait(.01)) for _ in range(70)]
        assert len(set(accepted)) == 70
        assert await rig.settle()
        history = rig.ts.recent_outcomes()
        assert history['capacity'] == 64 and history['dropped'] == 6
        assert [r['job_id'] for r in history['outcomes']] == accepted[6:]
        assert len(events(rig)) == 210
        assert [e['event_seq'] for e in events(rig)] == list(range(1, 211))
    run_case(case)


def test_lock_redacts_prior_public_outcomes_without_losing_job_identity():
    async def case(rig):
        hidden = rig.ts.tricks['hold_pose']; hidden.secret = True
        rig.ts.secret_gate.unlocked = True
        reply = await rig.ts.queue(f'{hidden.name} wave_left', lambda: asyncio.sleep(0, result=robot_api.Result(True, f'{hidden.controller} completed')), 0., then=robot_api.UNCHANGED)
        assert await rig.settle()
        assert hidden.name in str(rig.ts.recent_outcomes())
        rig.ts.secret_gate.lock()
        public = rig.ts.recent_outcomes()['outcomes'][0]
        assert hidden.name not in str(public) and 'wave_left' not in str(public)
        assert public['job_id'] == job_id(reply) and public['terminal_state'] == 'succeeded'
        assert public['name'] == 'locked trick'
        assert any(hidden.name in row['name'] for row in events(rig))  # private diagnostic log
    run_case(case)


def test_structured_logger_failure_cannot_block_queue_or_stop():
    async def case(rig):
        old = robot_api.log.info
        def faulty(message, *args, **kwargs):
            if message.startswith('job_event '): raise RuntimeError('fixture logging failure')
            return old(message, *args, **kwargs)
        robot_api.log.info = faulty
        try:
            reply, _ = await held_job(rig)
            stopped = await rig.ts.immediate_stop()
            assert stopped.ok and stopped.details['running_job']['job_id'] == job_id(reply)
            assert await rig.settle()
            assert rig.ts.last_outcome['terminal_state'] == 'cancelled'
        finally:
            robot_api.log.info = old
    run_case(case)


def test_new_server_instance_cannot_reuse_prior_job_identity():
    identifiers = []
    async def case(rig):
        identifiers.append(job_id(await rig.ts.wait(.01)))
        assert await rig.settle()
    run_case(case); run_case(case)
    assert identifiers[0] != identifiers[1]
    assert all(identifier.endswith(':1') for identifier in identifiers)


def test_repeated_stop_records_requests_but_only_one_terminal_outcome():
    async def case(rig):
        reply, _ = await held_job(rig)
        identifier = job_id(reply)
        async def instant_follow(): return robot_api.Result(True, 'already off')
        rig.ts.stop_following = instant_follow
        first = await rig.ts.immediate_stop()
        second = await rig.ts.immediate_stop()
        assert first.details['running_job']['job_id'] == second.details['running_job']['job_id'] == identifier
        assert first.details['running_job']['terminal_state'] == second.details['running_job']['terminal_state'] == 'pending'
        assert await rig.settle()
        rows = [e for e in events(rig) if e['job_id'] == identifier]
        assert sum(e['event'] == 'cancel_requested' for e in rows) == 2
        assert sum(e['event'] == 'finished' for e in rows) == 1
        assert rows[-1]['outcome'] == 'cancelled'
        snapshot = rig.ts.recent_outcomes()
        snapshot['outcomes'][0]['message'] = 'caller-side mutation'
        assert rig.ts.recent_outcomes()['outcomes'][0]['message'] == 'cancelled'
    run_case(case)


@pytest.mark.parametrize('result_kind', ['success', 'failure', 'exception'])
def test_worker_cancel_after_child_completion_keeps_actual_terminal_evidence(result_kind):
    async def case(rig):
        entered, release = asyncio.Event(), asyncio.Event()
        async def job():
            entered.set(); await release.wait()
            if result_kind == 'exception': raise ValueError('fixture terminal exception')
            return robot_api.Result(result_kind == 'success', 'actual completed result')
        reply = await rig.ts.queue('completion race', job, 1., then=robot_api.UNCHANGED)
        await entered.wait(); child = rig.ts.current
        rig.ts.closing = True
        release.set()
        asyncio.get_running_loop().call_soon(rig.ts.worker.cancel)
        await asyncio.gather(rig.ts.worker, return_exceptions=True)
        assert child.done() and not child.cancelled()
        terminal = rig.ts.last_outcome
        assert terminal['job_id'] == job_id(reply)
        assert terminal['terminal_state'] == ('succeeded' if result_kind == 'success' else 'failed')
        assert terminal['success'] is (result_kind == 'success')
        assert terminal['message'] == ('job failed: ValueError' if result_kind == 'exception' else 'actual completed result')
        finished = [e for e in events(rig) if e['event'] == 'finished']
        assert len(finished) == 1 and finished[0]['reason'] == 'worker_cancelled_after_job_finished'
    run_case(case)
