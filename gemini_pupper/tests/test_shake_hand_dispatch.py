"""Terminal action fencing and in-process HTTP contracts for the compound tool."""
import asyncio

import httpx
import pytest

from test_gemini_chat_dispatch import actual_chat, packet, end_turn, fake, until, no_access
from test_server_transport import server
from test_session_context import installed_rig


@pytest.mark.parametrize('reply', [{'success': False}, {'success': False, 'outcome_unknown': True}])
def test_terminal_handshake_refusal_fences_later_motion(monkeypatch, reply):
    async def case():
        calls = []
        async def call(name, args):
            calls.append(name)
            return reply if name == 'shake_hand' else {'success': True}
        async def script(replies):
            yield packet('shake_hand', 'move')
            await until(lambda: len(replies) >= 2)
            yield end_turn()
        await actual_chat(monkeypatch, fake(call), script)
        assert calls == ['shake_hand', 'stop']
    asyncio.run(case())


def test_terminal_stop_bypasses_pending_handshake_and_fences_tail(monkeypatch):
    async def case():
        entered, release, stopped = asyncio.Event(), asyncio.Event(), asyncio.Event()
        calls = []
        async def call(name, args):
            calls.append(name)
            if name == 'shake_hand':
                entered.set(); await release.wait()
            if name == 'stop': stopped.set()
            return {'success': True}
        async def script(replies):
            yield packet('shake_hand', 'move')
            await asyncio.wait_for(entered.wait(), 1)
            yield packet('stop')
            await asyncio.wait_for(stopped.wait(), 1)
            release.set()
            await until(lambda: len(replies) >= 3)
            yield end_turn()
        await actual_chat(monkeypatch, fake(call), script)
        assert calls == ['shake_hand', 'stop']
    asyncio.run(case())


def test_http_catalog_epoch_and_final_paw_job_identity(server):
    async def case():
        async with installed_rig() as rig:
            server.tools = rig.ts
            try:
                async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app), base_url='http://in-process.invalid') as client:
                    before = list(rig.robot.switches)
                    refused = await client.post('/tools/shake_hand', json={'paw': 'left'})
                    assert refused.status_code == 428 and not refused.json()['success']
                    assert rig.ts.commands.empty() and rig.robot.switches == before
                    headers = {'X-Pupper-Command-Epoch': rig.ts.command_epoch}
                    accepted = await client.post('/tools/shake_hand', json={'paw': 'right'}, headers=headers)
                    data = accepted.json()
                    assert accepted.status_code == 200 and data['success']
                    assert data['details']['handshake']['action'] == 'shake_right'
                    submission = data['details']['submission']
                    assert await rig.settle()
                    assert rig.ts.last_outcome['job_id'] == submission['job_id']
                    assert rig.ts.last_outcome['success'] and 'shake_right' in rig.ts.last_outcome['message']
                    stopped = await client.post('/tools/stop', json={})
                    assert stopped.json()['success']
                    stale = await client.post('/tools/shake_hand', json={'paw': 'left'}, headers=headers)
                    assert stale.status_code == 409 and not stale.json()['success']
            finally:
                server.tools = None
    asyncio.run(case())
