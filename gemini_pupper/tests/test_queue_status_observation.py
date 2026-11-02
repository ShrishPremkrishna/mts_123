"""Read-only HTTP inventory polling after external controller changes; fake ROS only."""
import asyncio
import pytest
from test_server_transport import server, client_with_rig, harness, robot_api


def test_external_stock_activation_is_observed_without_rewriting_queue_plan(server):
    async def run():
        async with client_with_rig(server) as (client, rig):
            assert (await rig.ts.switch_to(None)).ok
            rig.ts.state_after_queue = robot_api.MOTORS_IDLE
            # Physical Options can activate stock after API startup, without switch_to.
            rig.robot.controllers['neural_controller_default'] = 'active'
            before = (rig.ts.state_now, rig.ts.state_after_queue, list(rig.robot.switches), list(rig.robot.published))
            response = await client.get('/queue/status')
            assert response.status_code == 200
            body = response.json()
            assert body['observed_controller_mode'] == 'locomotion'
            assert body['state'] == body['state_after_queue'] == 'motors idle'
            assert body['state_basis'] == 'cached_queue_plan'
            assert body['physical_observation'] == {'pose': None, 'speed': None, 'stopped': None}
            assert before == (rig.ts.state_now, rig.ts.state_after_queue, rig.robot.switches, rig.robot.published)
    asyncio.run(run())


@pytest.mark.parametrize('inventory,expected', [
    ({'neural_controller_default':'inactive'}, 'none'),
    ({'unfamiliar_joint_controller':'active'}, 'unknown'),
    ({'neural_controller_default':'active','gemini_walk':'active'}, 'unknown'),
    (None, 'unknown'),
])
def test_polled_inventory_is_fresh_or_unknown_not_old_plan(server, inventory, expected):
    async def run():
        async with client_with_rig(server) as (client, rig):
            cached = rig.ts.state_now
            old_call = rig.ts._call
            async def unavailable(client_, request, timeout=3.):
                return None if client_ is rig.ts.list_client else await old_call(client_, request, timeout)
            if inventory is None:
                rig.ts._call = unavailable
            else:
                rig.robot.controllers = inventory
            body = (await client.get('/queue/status')).json()
            assert body['observed_controller_mode'] == expected
            assert rig.ts.state_now == cached
            assert body['physical_observation']['pose'] is None
    asyncio.run(run())


@pytest.mark.parametrize('blocked', ['controller_fault','stop_latch','fallen'])
def test_poll_preserves_fault_and_fall_facts_without_pose_claim(server, blocked):
    async def run():
        async with client_with_rig(server) as (client, rig):
            if blocked == 'controller_fault':
                rig.ts.controller_fault = 'test controller fault'
            elif blocked == 'stop_latch':
                rig.robot.stop_status.update(latched=True,stop_required=True,epoch=rig.robot.stop_status['epoch']+1)
                rig.robot.emit_stop_status()
                for _ in range(20): await asyncio.sleep(0)
            else:
                rig.ts.fallen = 'neural_controller_default'
                rig.ts.state_now = rig.ts.state_after_queue = robot_api.FALLEN
            before = rig.ts.status()
            body = (await client.get('/queue/status')).json()
            assert all(body[k] == before[k] for k in ('state','state_after_queue','controller_fault','physical_observation'))
            assert body['observed_controller_mode'] in {'unknown','locomotion'}
            if blocked == 'controller_fault': assert body['controller_fault'] == 'test controller fault'
    asyncio.run(run())


def test_poll_keeps_running_job_and_projected_destination(server):
    async def run():
        async with client_with_rig(server) as (client, rig):
            gate, entered = asyncio.Event(), asyncio.Event()
            async def held_job():
                entered.set(); await gate.wait(); return robot_api.Result(True,'fixture completed')
            assert (await rig.ts.queue('fixture pending',held_job,1.,then=robot_api.MOTORS_IDLE)).ok
            await entered.wait()
            before = (rig.ts.current,rig.ts.state_now,rig.ts.state_after_queue)
            body = (await client.get('/queue/status')).json()
            assert body['observed_controller_mode'] == 'locomotion'
            assert body['running'] is True and body['state_after_queue'] == 'motors idle'
            assert before == (rig.ts.current,rig.ts.state_now,rig.ts.state_after_queue)
            gate.set(); assert await rig.settle()
    asyncio.run(run())


def test_pending_inventory_does_not_serialize_stop_or_freeze_old_queue_facts(server):
    async def run():
        async with client_with_rig(server) as (client, rig):
            gate, entered = asyncio.Event(), asyncio.Event()
            old_call = rig.ts._call
            async def slow_inventory(client_,request,timeout=3.):
                if client_ is rig.ts.list_client:
                    entered.set(); await gate.wait()
                return await old_call(client_,request,timeout)
            rig.ts._call = slow_inventory
            pending = asyncio.create_task(client.get('/queue/status'))
            await asyncio.wait_for(entered.wait(),1.)
            epoch = rig.ts.command_epoch
            stopped = await asyncio.wait_for(client.post('/stop/immediate'),1.)
            assert stopped.json()['success'] and not pending.done()
            assert rig.ts.command_epoch != epoch
            gate.set()
            body = (await pending).json()
            assert body['command_epoch'] == rig.ts.command_epoch
            assert body['queued'] == 0 and body['running'] is False
    asyncio.run(run())
