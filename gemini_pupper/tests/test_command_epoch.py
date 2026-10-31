"""HTTP Stop ordering fence: in-process ASGI and fake ROS, never sockets/hardware."""
import asyncio
import json

import httpx
import pytest

from test_server_transport import server, client_with_rig
import test_tools as harness
import robot_api


def epoch_headers(rig):
    return {'X-Pupper-Command-Epoch': rig.ts.command_epoch}


@pytest.mark.parametrize('path,body', [
    ('/tools/move', {'direction': 'forward', 'speed': .2, 'duration': .1}),
    ('/walking/activate', {}), ('/walking/deactivate', {}),
    ('/move/velocity', {'forward_backward_velocity': .2, 'duration': .1}),
    ('/move/direction', {'heading': 0, 'speed': .2, 'duration': .1}),
    ('/animation', {'animation_name': 'hold_pose'}),
    ('/interactive_action', {'action': 'wave_left'}),
    ('/queue/wait', {'duration': 1}), ('/queue/reset', {}),
    ('/following/activate', {}), ('/following/deactivate', {}),
    ('/tools/unlock', {'password': 'offline-placeholder'}),
    ('/tools/lock_tricks', {}), ('/system/volume', {'volume': 50}),
])
def test_every_existing_mutation_refuses_missing_epoch_without_dispatch(server, path, body):
    async def case():
        async with client_with_rig(server, epochs=False) as (client, rig):
            response = await client.post(path, json=body)
            assert response.status_code == 428, response.text
            assert response.json()['success'] is False
            assert response.json()['command_epoch'] == rig.ts.command_epoch
            assert 'no command was submitted' in response.json()['message']
            assert rig.ts.commands.empty() and rig.ts.current is None
            assert not harness.walk_twists(rig)
    asyncio.run(case())


def test_future_mutation_route_inherits_fail_closed_guard(server):
    calls = []
    @server.app.post('/offline-new-mutation')
    async def new_route():
        calls.append(True)
        return {'success': True}
    # Put this test route before the static app's catch-all mount.
    server.app.router.routes.insert(0, server.app.router.routes.pop())
    async def case():
        async with client_with_rig(server, epochs=False) as (client, rig):
            assert (await client.post('/offline-new-mutation')).status_code == 428
            assert not calls
            assert (await client.post('/offline-new-mutation', headers=epoch_headers(rig))).status_code == 200
            assert calls == [True]
    asyncio.run(case())


@pytest.mark.parametrize('path', ['/stop/immediate', '/queue/stop', '/tools/stop'])
def test_stop_needs_no_epoch_and_returns_new_epoch_even_on_failure(server, path):
    async def case():
        async with client_with_rig(server, epochs=False) as (client, rig):
            health = (await client.get('/health')).json()
            assert health['command_epoch'] == rig.ts.command_epoch
            assert health['command_epoch_required'] is True
            old = rig.ts.command_epoch
            response = await client.post(path, json={})
            assert response.json()['success'] is True
            assert response.json()['command_epoch'] == rig.ts.command_epoch != old
            async def failed_follow_stop():
                return robot_api.Result(False, 'synthetic unconfirmed follower stop')
            rig.ts.stop_following = failed_follow_stop
            old = rig.ts.command_epoch
            response = await client.post(path, json={})
            assert response.json()['success'] is False
            assert response.json()['command_epoch'] == rig.ts.command_epoch != old
            assert (await client.post('/tools/robot_status', json={})).status_code == 200
    asyncio.run(case())


@pytest.mark.parametrize('delay', ['body', 'before_app'])
def test_old_motion_delayed_past_stop_is_rejected_without_nonzero_commands(server, delay):
    async def case():
        async with client_with_rig(server, epochs=False) as (client, rig):
            entered, release = asyncio.Event(), asyncio.Event()
            payload = json.dumps({'direction': 'forward', 'speed': .2, 'duration': .1}).encode()
            class DelayedBody(httpx.AsyncByteStream):
                async def __aiter__(self):
                    entered.set()
                    await release.wait()
                    yield payload
            async def held_app(scope, receive, send):
                entered.set()
                await release.wait()
                await server.app(scope, receive, send)
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=held_app if delay == 'before_app' else server.app),
                                         base_url='http://in-process.invalid') as delayed:
                headers = {**epoch_headers(rig), 'Content-Type': 'application/json'}
                request = asyncio.create_task(delayed.post('/tools/move', content=DelayedBody() if delay == 'body' else payload,
                                                           headers=headers))
                await asyncio.wait_for(entered.wait(), 1)
                assert (await client.post('/stop/immediate')).json()['success']
                release.set()
                response = await asyncio.wait_for(request, 1)
                assert response.status_code == 409
                assert response.json()['success'] is False
                assert 'stale command epoch' in response.json()['message']
                assert await rig.settle()
                assert not harness.walk_twists(rig)
                # A new explicitly issued command after Stop may proceed.
                accepted = await client.post('/tools/move', json=json.loads(payload), headers=epoch_headers(rig))
                assert accepted.json()['success'] is True
                assert await rig.settle()
                assert harness.walk_twists(rig)
    asyncio.run(case())


def test_stop_after_epoch_validation_still_cancels_waiting_dispatch(server):
    async def case():
        async with client_with_rig(server, epochs=False) as (client, rig):
            await rig.ts.action_lock.acquire()
            request = asyncio.create_task(client.post('/tools/move', json={'direction': 'forward'}, headers=epoch_headers(rig)))
            for _ in range(5):
                await asyncio.sleep(0)
            assert not request.done()
            assert (await client.post('/stop/immediate')).json()['success']
            rig.ts.action_lock.release()
            response = await request
            assert not response.json()['success']
            assert 'cancelled by a stop' in response.json()['message']
            assert await rig.settle()
            assert not harness.walk_twists(rig)
    asyncio.run(case())


def test_new_robot_instance_refuses_prior_instance_token(server):
    async def case():
        async with client_with_rig(server, epochs=False) as (_, first):
            old = first.ts.command_epoch
        async with client_with_rig(server, epochs=False) as (client, second):
            assert second.ts.command_epoch != old
            response = await client.post('/tools/move', json={'direction': 'forward'},
                                         headers={'X-Pupper-Command-Epoch': old})
            assert response.status_code == 409
            assert not harness.walk_twists(second)
    asyncio.run(case())


@pytest.mark.parametrize('event', ['physical_stop', 'fall'])
def test_internal_stop_events_invalidate_preexisting_http_tokens(server, event):
    async def case():
        async with client_with_rig(server, epochs=False) as (client, rig):
            old = epoch_headers(rig)
            if event == 'physical_stop':
                rig.ts._on_estop(harness._Msg())
            else:
                rig.ts._on_rosout(harness._Msg(name=robot_api.LOCOMOTION_CONTROLLER, msg=robot_api.ESTOP_LOG))
            for _ in range(30):
                await asyncio.sleep(0)
            assert rig.ts.command_epoch != old['X-Pupper-Command-Epoch']
            rejected = await client.post('/tools/move', json={'direction': 'forward'}, headers=old)
            assert rejected.status_code == 409
            assert not harness.walk_twists(rig)
    asyncio.run(case())
