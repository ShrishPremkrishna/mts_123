"""Actual HTTP shape in process only: fake ROS, no network or cameras."""
import asyncio
import json
import pytest
from test_server_transport import server, client_with_rig, robot_api


def test_manual_move_estimate_from_completed_interactive_pose(server):
    async def run():
        async with client_with_rig(server) as (client, rig):
            trick = rig.ts.tricks['hold_pose']
            assert (await client.post('/tools/play_trick', json={
                'name': trick.name, 'count': 1, 'count_unit': 'routine_playback'})).json()['success']
            assert await rig.settle()
            before = rig.clock.now
            reply = (await client.post('/move/velocity', json={
                'forward_backward_velocity': .2, 'right_left_velocity': 0,
                'turning_velocity': 0, 'duration': .5})).json()
            expected = trick.clip_time_estimate(trick.exit_clip) + robot_api.WALK_READY_S + .5
            print(json.dumps({'case': 'manual_half_second_from_interactive', 'hardware': False,
                'reply': reply, 'expected_estimate_seconds': expected}))
            assert reply['success']
            assert f'estimated queue time {expected:.1f} s' in reply['message']
            assert rig.ts.busy_until - before == pytest.approx(expected)
            assert await rig.settle()
            assert rig.ts.last_outcome['success']
    asyncio.run(run())


def test_http_status_does_not_publish_locked_fault_identity(server):
    async def run():
        async with client_with_rig(server) as (client, rig):
            hidden = rig.ts.tricks['hold_pose']
            hidden.secret = True
            fault = f'{hidden.controller}: playback endpoint unknown; motion quarantined'
            rig.ts.controller_fault = fault
            for path in ['/queue/status', '/tools/robot_status']:
                result = (await (client.get(path) if path == '/queue/status' else client.post(path, json={}))).json()
                assert hidden.name not in json.dumps(result)
                assert 'motion quarantined' in json.dumps(result)
            assert rig.ts.controller_fault == fault
            blocked = (await client.post('/tools/move', json={'direction': 'forward', 'speed': .2, 'duration': .5})).json()
            assert blocked['success'] is False
            assert rig.ts.commands.empty()
    asyncio.run(run())
