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
