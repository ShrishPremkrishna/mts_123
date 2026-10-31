"""Real elapsed fake queue timing: no key, Live request, sockets, ROS or hardware."""
import asyncio
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
import gemini_chat
import robot_api


@pytest.fixture(autouse=True)
def no_access(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('Timing tests must not access a key, network or robot backend')
    monkeypatch.setattr(gemini_chat, 'api_key', forbidden)
    monkeypatch.setattr(gemini_chat.urllib.request, 'urlopen', forbidden)
    monkeypatch.setattr(gemini_chat.RobotBackend, 'setup', forbidden)


def run_case(case, *, realtime=True):
    async def run():
        backend = gemini_chat.DeskBackend(realtime=realtime)
        try:
            backend.setup()
            await backend.rig.ts.ready.wait()
            await case(backend)
        finally:
            await backend.aclose()
    asyncio.run(run())


async def complete(backend, timeout=2.):
    async def wait():
        while not backend.rig.ts.commands.empty() or backend.rig.ts.current is not None:
            await asyncio.sleep(.01)
    await asyncio.wait_for(wait(), timeout)


def test_accelerated_default_preserves_demonstration_but_is_labelled():
    async def case(backend):
        started = time.monotonic()
        result = await backend.call('wait', {'duration': 20.})
        assert result['success'] and 'queued' in result['message']
        # This intentional default shortcut explains why the old Live smoke
        # could not assess same-batch Stop cancellation on the real queue.
        assert backend.rig.ts.last_outcome['message'] == 'waited 20.0 s'
        assert time.monotonic() - started < 1.
        assert 'accelerated per-call completion' in backend.name
    run_case(case, realtime=False)


def test_realtime_returns_acceptance_and_stop_clears_pending_sequence():
    async def case(backend):
        started = time.monotonic()
        accepted = await backend.call('wait', {'duration': 10.})
        move = await backend.call('move', {'direction': 'forward', 'speed': .2, 'duration': 1.})
        assert accepted['success'] and move['success']
        assert 'queued' in accepted['message'] and 'waited' not in accepted['message']
        assert time.monotonic() - started < 1.
        assert backend.rig.ts.commands.qsize() == 2
        stopped = await backend.call('stop', {})
        assert stopped['success']
        await complete(backend)
        assert time.monotonic() - started < 1.
        assert not harness.walk_twists(backend.rig)
        assert not any('waited 10' in text for text in backend.rig.outcomes())
    run_case(case)


def test_realtime_current_wait_status_and_immediate_cancellation():
    async def case(backend):
        assert (await backend.call('wait', {'duration': 10.}))['success']
        await asyncio.sleep(.03)
        status = await backend.call('robot_status', {})
        assert 'running: True' in status['message']
        assert 'waited 10' not in status['message']
        started = time.monotonic()
        assert (await backend.call('stop', {}))['success']
        await complete(backend)
        assert time.monotonic() - started < 1.
        status = await backend.call('robot_status', {})
        assert 'running: False' in status['message']
        assert 'failed: cancelled' in status['message']
        assert 'waited 10' not in status['message']
    run_case(case)


def test_realtime_preserves_queue_order_and_elapsed_pause():
    async def case(backend):
        calls = [('move', {'direction': 'forward', 'speed': .2, 'duration': .1}),
                 ('wait', {'duration': .15}),
                 ('move', {'direction': 'left', 'speed': .2, 'duration': .1})]
        for name, args in calls:
            result = await backend.call(name, args)
            assert result['success'] and 'queued' in result['message']
        assert backend.rig.ts.commands.qsize() == 3
        await complete(backend)
        twists = backend.rig.robot.twists()
        forward = [t for t, m in twists if m.linear.x > 0]
        sideways = [t for t, m in twists if m.linear.y > 0]
        assert forward and sideways
        assert min(sideways) - min(forward) >= .1 + .15
        assert max(forward) < min(sideways)
        outcomes = backend.rig.outcomes()
        forward_i = next(i for i, line in enumerate(outcomes) if line.startswith('move vx=0.2'))
        wait_i = next(i for i, line in enumerate(outcomes) if line.startswith('wait 0.15s: waited'))
        sideways_i = next(i for i, line in enumerate(outcomes) if line.startswith('move vx=0 vy=0.2'))
        assert forward_i < wait_i < sideways_i
        status = await backend.call('robot_status', {})
        assert 'running: False' in status['message'] and 'completed' in status['message']
    run_case(case)


def test_realtime_feedback_stays_fresh_during_wait_without_extra_tool_calls():
    async def case(backend):
        assert (await backend.call('wait', {'duration': .7}))['success']
        initial = backend.rig.robot.stop_status['sequence']
        await asyncio.sleep(.55)
        assert backend.rig.robot.stop_status['sequence'] > initial + 5
        assert backend.rig.ts.motion_block_reason() is None
        assert backend.rig.ts.current is not None
        await complete(backend)
        assert backend.rig.ts.last_outcome['message'] == 'waited 0.7 s'
    run_case(case)


def test_close_cancels_work_ticker_and_restores_fake_bindings():
    async def run():
        old_clock = harness.VirtualClock
        bindings = {name: getattr(robot_api, name) for name in ('time', 'asyncio', 'load_tricks')}
        old_robot = harness.ROBOT
        backend = gemini_chat.DeskBackend(realtime=True)
        backend.setup()
        assert harness.VirtualClock is old_clock
        await backend.rig.ts.ready.wait()
        assert (await backend.call('wait', {'duration': 30.}))['success']
        await asyncio.sleep(.02)
        started = time.monotonic()
        await backend.aclose()
        await backend.aclose()  # idempotent cleanup
        assert time.monotonic() - started < 1.
        assert backend.rig.ts.worker.done() and backend._ticker.done()
        assert all(getattr(robot_api, name) is value for name, value in bindings.items())
        assert harness.ROBOT is old_robot
    asyncio.run(run())


def test_chat_failure_still_closes_fake_tasks_without_key_access(monkeypatch):
    backend = gemini_chat.DeskBackend(realtime=True)
    async def fail(backend, model, full, **_config):
        backend.setup()
        await backend.rig.ts.ready.wait()
        await backend.call('wait', {'duration': 30.})
        raise RuntimeError('synthetic session failure')
    monkeypatch.setattr(gemini_chat, '_chat', fail)
    with pytest.raises(RuntimeError, match='synthetic session failure'):
        asyncio.run(gemini_chat.chat(backend, 'unused', False))
    assert backend._closed and backend.rig.ts.worker.done() and backend._ticker.done()


def test_realtime_cli_cannot_select_robot(monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['gemini_chat.py', '--realtime', '--robot'])
    with pytest.raises(SystemExit) as result:
        gemini_chat.main()
    assert result.value.code == 2


def test_diagnostic_does_not_label_sideways_velocity_as_zero():
    async def case(backend):
        backend.what_happened()
        backend.rig.ts.send_speed(0., .2, 0.)
        lines = backend.what_happened()
        assert any('speed:' in line for line in lines)
        assert not any('then a zero' in line for line in lines)
        backend.rig.ts.send_speed(.2, 0., 0.)
        backend.rig.ts.send_speed(0., 0., 0.)
        lines = backend.what_happened()
        assert any('zero command (physical stopping unmeasured)' in line for line in lines)
    run_case(case)


def test_completed_move_is_distinct_from_current_wait_in_tool_evidence():
    async def case(backend):
        assert (await backend.call('move', {'direction': 'forward', 'speed': .2, 'duration': .05}))['success']
        await complete(backend)
        assert (await backend.call('wait', {'duration': 10}))['success']
        await asyncio.sleep(.03)
        status = await backend.call('robot_status', {})
        details = status['details']
        assert details['current_action'] == {'name': 'wait 10s', 'state': 'running'}
        assert details['running'] is True
        assert details['last_outcome']['name'].startswith('move vx=0.2')
        assert details['last_outcome']['success'] is True
        assert details['physical_observation'] == {'pose': None, 'speed': None, 'stopped': None}
        assert 'Current action wait 10s: running' in status['message']
        assert 'Latest finished action move' in status['message']
        assert (await backend.call('stop', {}))['success']
        await complete(backend)
        stopped = await backend.call('robot_status', {})
        assert stopped['details']['current_action'] is None
        assert stopped['details']['last_outcome'] == {'name': 'wait 10s', 'success': False, 'message': 'cancelled'}
    run_case(case)
