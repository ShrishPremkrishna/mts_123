"""A new decorator tool uses the existing HTTP/queue/Stop contract unchanged.

Reuses the transport harness's fake ROS and in-process ASGI client. No sockets,
provider calls, processes or physical hardware are permitted by that fixture.
"""
import asyncio

import pytest

from test_server_transport import client_with_rig, server  # noqa: F401
import robot_api
import test_tools as harness
import tool_registry


@pytest.fixture
def extension(monkeypatch):
    monkeypatch.setattr(tool_registry, 'TOOLS', dict(tool_registry.TOOLS))
    events = []
    entered = asyncio.Event()

    @tool_registry.tool(
        'extension_probe', 'A fake queued extension used only to test composition.',
        phase=tool_registry.Param('string', 'Which fake job to run.',
                                  enum=['first', 'last', 'fail', 'block']),
    )
    async def extension_probe(robot, phase):
        async def job():
            events.append(phase)
            if phase == 'block':
                entered.set()
                await asyncio.Event().wait()
            return robot_api.Result(phase != 'fail', 'fake extension outcome')
        return await robot.queue('extension ' + phase, job, 0., then=robot_api.UNCHANGED)

    return events, entered


TAKEOVER = 'extension_takeover_probe'


@pytest.fixture
def takeover_extension(monkeypatch):
    """A parameterless motion extension that follows the handler contract."""
    original_names = set(tool_registry.TOOLS)
    monkeypatch.setattr(tool_registry, 'TOOLS', dict(tool_registry.TOOLS))
    events = []

    @tool_registry.tool(TAKEOVER, 'A fake bounded walk used only to test follow takeover.',
                        _ends_following=True)
    async def extension_takeover_probe(robot):
        # _ends_following is catalog metadata; the backend handoff is this call.
        prepared = await harness.behaviors.prepare_to_move(robot)
        if isinstance(prepared, robot_api.Result):
            return prepared
        note = prepared
        events.append(('note', note))

        async def job():
            events.append('job')
            switched = await robot.switch_to(robot_api.LOCOMOTION_CONTROLLER)
            events.append(('switched', switched.ok, robot_api.time.monotonic()))
            if not switched.ok:
                return switched
            robot.send_speed(.2, 0, 0)
            events.append('commanded')
            await robot.sleep(.1)
            robot.send_speed(0, 0, 0)
            return robot_api.Result(True, 'fake takeover probe walked briefly')
        queued = await robot.queue('extension takeover probe', job, .1, then=robot_api.LOCOMOTION)
        return robot_api.with_note(note, queued)

    return events, original_names


class Deferred:
    """A ROS client future whose reply the test releases explicitly."""
    def __init__(self):
        self.value, self.finished, self.cancelled = None, False, False
    def done(self): return self.finished
    def result(self): return self.value
    def cancel(self): self.cancelled = True
    def release(self, value): self.value, self.finished = value, True


def _outcome(payload, job_id):
    """An exact job ID in the public terminal-history schema."""
    return next((row for row in payload['recent_outcomes']['outcomes']
                 if row['job_id'] == job_id), None)


def _activations(rig, start):
    return [s for s in rig.robot.switches[start:] if s[2]]


def test_duplicate_name_cannot_replace_stop_or_an_extension(extension):
    original_stop = tool_registry.TOOLS['stop']
    original_extension = tool_registry.TOOLS['extension_probe']

    async def replacement(robot):
        raise AssertionError('duplicate registration must never run')

    for name, original in [('stop', original_stop), ('extension_probe', original_extension)]:
        with pytest.raises(ValueError, match='duplicate tool name'):
            tool_registry.tool(name, 'Accidental duplicate from another module.')(replacement)
        assert tool_registry.TOOLS[name] is original
    # Normal repeated loading uses Python's import cache, not replacement.
    tool_registry.load_lab()
    tool_registry.load_lab()
    assert tool_registry.TOOLS['stop'] is original_stop


def test_new_tool_is_advertised_and_composes_through_generic_http(server, extension):
    async def case():
        async with client_with_rig(server) as (client, rig):
            catalog = (await client.get('/tools')).json()
            declaration = next(t for t in catalog['tools'] if t['name'] == 'extension_probe')
            assert declaration['parameters']['required'] == ['phase']
            assert declaration['parameters']['properties']['phase']['enum'] == ['first', 'last', 'fail', 'block']
            submissions = []
            for name, args in [('extension_probe', {'phase': 'first'}),
                               ('wait', {'duration': .2}),
                               ('move', {'direction': 'forward', 'speed': .2, 'duration': .1}),
                               ('extension_probe', {'phase': 'last'})]:
                response = (await client.post('/tools/' + name, json=args)).json()
                assert response['success'] is True
                submissions.append(response['details']['submission'])
            assert len({s['job_id'] for s in submissions}) == 4
            assert all(s['server_instance'] == rig.ts.command_instance and s['state'] == 'queued' for s in submissions)
            assert await rig.settle()
            history = rig.ts.recent_outcomes()['outcomes']
            assert [h['job_id'] for h in history] == [s['job_id'] for s in submissions]
            assert all(h['terminal_state'] == 'succeeded' for h in history)
            assert extension[0] == ['first', 'last']
            assert harness.walk_twists(rig)
    asyncio.run(case())


@pytest.mark.parametrize('arguments', [{}, {'phase': 'not-listed'}, {'phase': True},
                                      {'phase': 'first', 'unexpected': 1}])
def test_extension_schema_refusal_has_no_job_or_motion(server, extension, arguments):
    async def case():
        async with client_with_rig(server) as (client, rig):
            response = (await client.post('/tools/extension_probe', json=arguments)).json()
            assert response['success'] is False
            assert not response.get('details', {}).get('submission')
            assert await rig.settle()
            assert extension[0] == []
            assert rig.ts.recent_outcomes()['outcomes'] == []
            assert not harness.walk_twists(rig)
    asyncio.run(case())


def test_extension_failure_cancels_already_queued_dependents(server, extension):
    async def case():
        async with client_with_rig(server) as (client, rig):
            # Hold the worker behind a real queue job so both dependents enter
            # before the failing extension runs; HTTP clients need not settle.
            hold = asyncio.Event()
            entered = asyncio.Event()
            async def barrier():
                entered.set()
                await hold.wait()
                return robot_api.Result(True, 'barrier complete')
            await rig.ts.queue('test barrier', barrier, 0., then=robot_api.UNCHANGED)
            await entered.wait()
            fail = (await client.post('/tools/extension_probe', json={'phase': 'fail'})).json()
            later = (await client.post('/tools/move', json={'direction': 'forward', 'speed': .2, 'duration': .1})).json()
            assert fail['success'] and later['success']
            hold.set()
            assert await rig.settle()
            outcomes = {h['job_id']: h for h in rig.ts.recent_outcomes()['outcomes']}
            assert outcomes[fail['details']['submission']['job_id']]['terminal_state'] == 'failed'
            cancelled = outcomes[later['details']['submission']['job_id']]
            assert cancelled['terminal_state'] == 'cancelled' and not cancelled['execution_started']
            assert not harness.walk_twists(rig)
    asyncio.run(case())


def test_stop_interrupts_extension_and_clears_later_motion(server, extension):
    async def case():
        async with client_with_rig(server) as (client, rig):
            current = (await client.post('/tools/extension_probe', json={'phase': 'block'})).json()
            await extension[1].wait()
            later = (await client.post('/tools/move', json={'direction': 'forward', 'speed': .2, 'duration': .1})).json()
            assert current['success'] and later['success']
            assert not rig.ts.action_lock.locked()
            stopped = (await client.post('/tools/stop', json={})).json()
            assert stopped['success']
            assert await rig.settle()
            outcomes = {h['job_id']: h for h in rig.ts.recent_outcomes()['outcomes']}
            for result in (current, later):
                assert outcomes[result['details']['submission']['job_id']]['terminal_state'] == 'cancelled'
            assert extension[0] == ['block']
            assert not harness.walk_twists(rig)
    asyncio.run(case())


def test_extension_takes_over_active_follow_only_after_confirmed_off(server, takeover_extension, monkeypatch):
    events, original_names = takeover_extension

    async def case():
        async with client_with_rig(server) as (client, rig):
            assert (await client.post('/tools/follow', json={'enabled': True})).json()['success']
            assert await rig.settle()
            assert rig.robot.follower_on and rig.ts.following
            follow_calls = list(rig.robot.follow_calls)
            twists = len(harness.walk_twists(rig))
            switches = len(rig.robot.switches)
            outcomes = len(rig.ts.recent_outcomes()['outcomes'])

            off = rig.ts.follow_off
            future, pending, submitted = Deferred(), [], asyncio.Event()
            def deferred_off(request):
                pending.append(request)
                submitted.set()
                return future
            monkeypatch.setattr(off, 'call_async', deferred_off)

            request = asyncio.create_task(client.post('/tools/' + TAKEOVER, json={}))
            await asyncio.wait_for(submitted.wait(), 5)
            # Deactivation submitted but unanswered: the fake follower is still on.
            assert len(pending) == 1 and not request.done()
            assert rig.robot.follower_on and rig.robot.follow_calls == follow_calls
            assert events == []
            assert rig.ts.commands.empty() and rig.ts.current is None
            assert len(rig.ts.recent_outcomes()['outcomes']) == outcomes
            assert len(harness.walk_twists(rig)) == twists and len(rig.robot.switches) == switches

            future.release(rig.robot.handle(off.name, pending[0]))
            events.append('off_handled')
            assert not rig.robot.follower_on
            response = (await asyncio.wait_for(request, 5)).json()
            assert response['success'] is True
            submission = response['details']['submission']
            assert submission['state'] == 'queued' and submission['server_instance'] == rig.ts.command_instance
            # Released synchronously, so the handler resumes only after this marker.
            assert events[0] == 'off_handled'
            assert events[1] == ('note', 'Stopped following the person first.')
            assert response['message'].startswith(events[1][1].strip())
            assert await rig.settle()

            assert events[2] == 'job'
            assert events[3][:2] == ('switched', True) and events[4:] == ['commanded']
            status = (await client.get('/queue/status', params={'observe_controllers': 'false'})).json()
            row = _outcome(status, submission['job_id'])
            assert row is not None and row['terminal_state'] == 'succeeded'
            assert len(harness.walk_twists(rig)) > twists
            assert not rig.robot.follower_on and not rig.ts.following
            assert rig.robot.follow_calls == follow_calls + ['off'] and len(pending) == 1

            catalog = (await client.get('/tools')).json()
            declaration = next(t for t in catalog['tools'] if t['name'] == TAKEOVER)
            assert 'parameters' not in declaration
            assert TAKEOVER in catalog['follow_handoff']['tools']
            assert set(tool_registry.TOOLS) - original_names == {TAKEOVER}
    asyncio.run(case())


@pytest.mark.parametrize('ending', ['timeout', 'stop'])
def test_extension_controller_lookup_is_bounded_by_existing_deadline_or_stop(
        server, takeover_extension, monkeypatch, ending):
    events, _ = takeover_extension

    async def case():
        async with client_with_rig(server) as (client, rig):
            assert not rig.robot.follower_on
            follow_calls = list(rig.robot.follow_calls)
            switches = len(rig.robot.switches)
            lister = rig.ts.list_client
            passthrough = lister.call_async
            future, pending, submitted = Deferred(), [], asyncio.Event()
            def deferred_list(request):
                if 'job' not in events or pending:
                    return passthrough(request)
                pending.append((request, rig.clock.now))
                submitted.set()
                return future
            monkeypatch.setattr(lister, 'call_async', deferred_list)
            # Hold virtual time while the dependent is admitted; the 3 s lookup
            # deadline itself is the unchanged one in Robot._call.
            frozen = [True]
            advance = rig.clock.sleep
            async def gated_sleep(seconds, result=None):
                if frozen[0]:
                    await harness._real_sleep(0)
                    return result
                return await advance(seconds, result)
            monkeypatch.setattr(rig.clock, 'sleep', gated_sleep)

            current = (await client.post('/tools/' + TAKEOVER, json={})).json()
            assert current['success']
            await asyncio.wait_for(submitted.wait(), 5)
            running = rig.ts.current
            assert running is not None and not running.done()
            later = (await client.post('/tools/move', json={'direction': 'forward', 'speed': .2, 'duration': .1})).json()
            assert later['success']
            assert not future.finished and events[1:] == ['job']

            frozen[0] = False
            if ending == 'stop':
                stopped = (await asyncio.wait_for(client.post('/tools/stop', json={}), 5)).json()
                assert stopped['success'] and not future.finished
            assert await rig.settle()

            status = (await client.get('/queue/status', params={'observe_controllers': 'false'})).json()
            extension = _outcome(status, current['details']['submission']['job_id'])
            dependent = _outcome(status, later['details']['submission']['job_id'])
            assert running.done()
            assert running.cancelled() is (ending == 'stop')
            assert extension['terminal_state'] == ('failed' if ending == 'timeout' else 'cancelled')
            assert dependent['terminal_state'] == 'cancelled' and not dependent['execution_started']
            if ending == 'timeout':
                assert events[2][:2] == ('switched', False)
                assert 3.0 <= events[2][2] - pending[0][1] < 4.0
                assert future.cancelled
            else:
                assert events[1:] == ['job']
            assert 'commanded' not in events
            assert not harness.walk_twists(rig) and not _activations(rig, switches)

            snapshot = status['recent_outcomes']
            future.release(rig.robot.handle(lister.name, pending[0][0]))
            # The completed/cancelled job cannot be resumed by this old reply.
            assert await rig.settle()
            after = (await client.get('/queue/status', params={'observe_controllers': 'false'})).json()
            assert running.done() and after['recent_outcomes'] == snapshot
            assert 'commanded' not in events and not any(e[:2] == ('switched', True) for e in events[1:])
            assert not harness.walk_twists(rig) and not _activations(rig, switches)
            assert rig.robot.follow_calls == follow_calls and not rig.robot.follower_on
    asyncio.run(case())
