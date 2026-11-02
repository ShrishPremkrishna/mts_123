"""Actual HTTP lifecycle shape and polling history; in-process fake ROS only."""
import asyncio
from test_server_transport import server, client_with_rig


def test_http_fast_jobs_keep_exact_ids_in_terminal_history_not_model_context(server):
    async def run():
        async with client_with_rig(server) as (client, rig):
            identifiers = []
            for _ in range(3):
                reply = (await client.post('/tools/wait', json={'duration': .01})).json()
                assert reply['success']
                identifiers.append(reply['details']['submission']['job_id'])
                assert await rig.settle()
            status = (await client.get('/queue/status')).json()
            assert status['last_outcome']['job_id'] == identifiers[-1]
            assert [r['job_id'] for r in status['recent_outcomes']['outcomes']] == identifiers
            assert status['recent_outcomes']['dropped'] == 0
            model = (await client.post('/tools/robot_status', json={})).json()
            assert 'recent_outcomes' not in model['details']
            assert model['details']['last_outcome']['job_id'] == identifiers[-1]
            assert model['details']['physical_observation'] == {'pose': None, 'speed': None, 'stopped': None}
    asyncio.run(run())


def test_queue_reset_reports_pending_ids_without_inventing_running_cancellation(server):
    async def run():
        async with client_with_rig(server) as (client, rig):
            # Direct queue calls do not yield before all pending entries are inserted.
            accepted = [await rig.ts.wait(1.) for _ in range(2)]
            reset = await rig.ts.clear_queue()
            ids = [r.details['submission']['job_id'] for r in accepted]
            assert reset.message == 'cleared 2 queued commands'
            assert reset.details['pending_queued_job_ids_cleared'] == ids
            status = (await client.get('/queue/status')).json()
            assert status['last_outcome'] is None
            rows = status['recent_outcomes']['outcomes']
            assert [r['job_id'] for r in rows] == ids
            assert all(r['terminal_state'] == 'cancelled' and not r['execution_started'] for r in rows)
    asyncio.run(run())
