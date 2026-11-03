"""Q7 two-stage handoff: fake ROS plus in-process HTTP, no live services."""
import asyncio
from types import SimpleNamespace
import pytest
from test_server_transport import server, client_with_rig
import tool_registry


def test_prepare_handoff_off_is_noop_then_on_confirms_disabled_before_motion(server):
    async def run():
        async with client_with_rig(server) as (client, rig):
            before = list(rig.robot.published)
            initial_follow_calls = list(rig.robot.follow_calls)
            off = (await client.post('/following/deactivate?if_active=true')).json()
            assert off['success'] and off['details']['following']['changed'] is False
            assert rig.robot.published == before and rig.robot.follow_calls == initial_follow_calls
            assert (await client.post('/tools/follow', json={'enabled': True})).json()['success']
            assert await rig.settle()
            active = (await client.post('/following/deactivate?if_active=true')).json()
            state = active['details']['following']
            assert active['success'] and state == {
                'schema': 'pupper_follow_state_v1', 'enabled': False, 'changed': True,
                'server_instance': rig.ts.command_instance, 'command_epoch': rig.ts.command_epoch}
            assert not rig.robot.follower_on and not rig.ts.following
            assert rig.ts.commands.qsize() == 0 and rig.ts.current is None
            repeated = (await client.post('/following/deactivate?if_active=true')).json()
            assert repeated['details']['following']['changed'] is False
            assert rig.robot.follow_calls == initial_follow_calls + ['on', 'off']
    asyncio.run(run())


def test_prepare_handoff_failure_never_confirms_disabled(server):
    async def run():
        async with client_with_rig(server) as (client, rig):
            assert (await client.post('/tools/follow', json={'enabled': True})).json()['success']
            assert await rig.settle()
            original = rig.ts._call
            async def fail_off(handle, request, timeout=3.0):
                if handle is rig.ts.follow_off: return SimpleNamespace(success=False)
                return await original(handle, request, timeout)
            rig.ts._call = fail_off
            refused = (await client.post('/following/deactivate?if_active=true')).json()
            assert refused['success'] is False and 'following' not in refused.get('details', {})
            assert rig.ts.follow_state_uncertain and not rig.ts.commands.qsize()
    asyncio.run(run())


def test_handoff_http_keeps_stop_epoch_and_extension_metadata(server):
    async def run():
        async with client_with_rig(server, epochs=False) as (client, rig):
            epoch = rig.ts.command_epoch
            name = 'future_pose_handoff_test'
            @tool_registry.tool(name, 'Future direct pose with a follow takeover.', _ends_following=True)
            async def extension(robot): raise AssertionError('catalog must not execute tools')
            try:
                catalog = (await client.get('/tools')).json()
                assert name in catalog['follow_handoff']['tools']
                decl = next(t for t in catalog['tools'] if t['name'] == name)
                assert 'parameters' not in decl and '_ends_following' not in str(decl)
                names = catalog['follow_handoff']['tools']
                assert all(n in names for n in ['stand_up','lie_down','move','turn','arc','play_trick'])
                assert not set(['wait','follow','stop','robot_status','interactive_action']) & set(names)
                await client.post('/stop/immediate')
                before_stale = list(rig.robot.follow_calls)
                stale = await client.post('/following/deactivate?if_active=true', headers={'X-Pupper-Command-Epoch': epoch})
                assert stale.status_code == 409 and stale.json()['command_epoch_required']
                assert rig.robot.follow_calls == before_stale
            finally: del tool_registry.TOOLS[name]
    asyncio.run(run())
