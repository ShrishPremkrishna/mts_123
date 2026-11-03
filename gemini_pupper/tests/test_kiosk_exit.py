"""Opt-in Stop after the onboard browser exits; fake browser/HTTP/ROS, no network or processes."""
import asyncio
import fcntl
import http.client
import json
from pathlib import Path
import re
import signal
import sys
import types
import urllib.error
from urllib.parse import unquote, urlsplit

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import onboard_bootstrap as ob
import kiosk
from test_server_transport import client_with_rig, harness, robot_api, server  # fake ROS fixture
import httpx

ORIGIN = 'http://localhost:18090'
INSTANCE = 'a' * 32
HEADER = 'X-Pupper-Expected-Server-Instance'
RECEIPT = {'success': True, 'message': 'stopped', 'command_epoch': 'epoch-1',
           'command_epoch_required': True, 'server_instance': INSTANCE}


@pytest.fixture
def directory(tmp_path):
    return ob.private_directory(tmp_path / 'onboard')


def healthy(directory):
    def get(_origin, path):
        if path == '/health': return {'status': 'running', 'tools': True}
        assert path == '/onboard/status'
        return {'schema': ob.SCHEMA, 'enabled': True, 'server_instance': INSTANCE,
                'directory_id': ob.directory_id(directory)}
    return get


def leftovers(directory):
    return list(directory.glob('*.grant')) + list(directory.glob('launch-*.html'))


def lock_held(directory):
    # flock conflicts across open file descriptions, even within this process.
    with (directory / 'browser.lock').open('a') as other:
        try: fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: return True
        fcntl.flock(other, fcntl.LOCK_UN)
        return False


class Browser:
    """Records the launch nonce, then returns or raises like subprocess.run."""
    def __init__(self, events, outcome=0):
        self.events, self.outcome, self.nonce = events, outcome, None
    def __call__(self, command, **_kwargs):
        page = Path(unquote(urlsplit(command[-1]).path)).read_text()
        self.nonce = re.search(r'#onboard=([0-9a-f]{64})', page).group(1)
        self.events.append('browser')
        if isinstance(self.outcome, BaseException): raise self.outcome
        return types.SimpleNamespace(returncode=self.outcome)


class Stop:
    def __init__(self, directory, events, outcome=True):
        self.directory, self.events, self.outcome = directory, events, outcome
    def __call__(self, origin, instance):
        self.events.append(('stop', origin, instance, lock_held(self.directory), bool(leftovers(self.directory))))
        if isinstance(self.outcome, BaseException): raise self.outcome
        return self.outcome


def never_stop(*_args):
    pytest.fail('Stop requested')


# ------------------------------------------------------------------ launcher
@pytest.mark.parametrize('returncode', [0, 17, -11])
def test_flag_absent_never_stops(directory, capsys, returncode):
    events = []
    assert kiosk.launch(ORIGIN, directory, get=healthy(directory), run=Browser(events, returncode), stop=never_stop) == returncode
    assert events == ['browser'] and not leftovers(directory)
    assert capsys.readouterr().err == ''


def test_interrupt_without_flag_never_stops(directory):
    with pytest.raises(KeyboardInterrupt):
        kiosk.launch(ORIGIN, directory, get=healthy(directory), run=Browser([], KeyboardInterrupt()), stop=never_stop)
    assert not leftovers(directory) and not lock_held(directory)


@pytest.mark.parametrize('returncode', [0, 17, -11])
def test_opt_in_stops_original_instance_once_after_browser_returns(directory, capsys, returncode):
    events = []
    result = kiosk.launch(ORIGIN, directory, get=healthy(directory), run=Browser(events, returncode),
                          stop_on_browser_exit=True, stop=Stop(directory, events))
    assert result == returncode
    # After the browser returned, grants already removed, launcher lock still held.
    assert events == ['browser', ('stop', ORIGIN, INSTANCE, True, False)]
    assert not leftovers(directory) and not lock_held(directory)
    assert capsys.readouterr().err == ''


@pytest.mark.parametrize('refusal', ['unhealthy', 'disabled', 'spawn'])
def test_preflight_refusal_and_spawn_failure_never_stop(directory, refusal):
    get = healthy(directory)
    if refusal == 'unhealthy': get = lambda *_: {'status': 'running', 'tools': False}
    if refusal == 'disabled': get = lambda _origin, path: {'status': 'running', 'tools': True} if path == '/health' else {'enabled': False}
    run = Browser([], FileNotFoundError('chromium')) if refusal == 'spawn' else lambda *_a, **_k: pytest.fail('browser spawned')
    with pytest.raises(OSError if refusal == 'spawn' else ob.BootstrapError):
        kiosk.launch(ORIGIN, directory, get=get, run=run, stop_on_browser_exit=True, stop=never_stop)
    assert not leftovers(directory) and not lock_held(directory)


@pytest.mark.parametrize('outcome', [False, None, 'yes', 1, RuntimeError('TEST-ONLY-SECRET'), TimeoutError()])
@pytest.mark.parametrize('returncode,expected', [(0, 1), (17, 17), (-11, -11)])
def test_unconfirmed_stop_is_nonzero_with_fixed_warning(directory, capsys, outcome, returncode, expected):
    events = []
    browser = Browser(events, returncode)
    result = kiosk.launch(ORIGIN, directory, get=healthy(directory), run=browser,
                          stop_on_browser_exit=True, stop=Stop(directory, events, outcome))
    assert result == expected
    assert events == ['browser', ('stop', ORIGIN, INSTANCE, True, False)]
    assert not leftovers(directory) and not lock_held(directory)
    err = capsys.readouterr().err
    assert err == kiosk.STOP_UNCONFIRMED + '\n'
    assert 'TEST-ONLY' not in err and browser.nonce not in err


@pytest.mark.parametrize('interrupt', [KeyboardInterrupt(), SystemExit(143), 'sigterm'])
@pytest.mark.parametrize('outcome', [True, False, RuntimeError('TEST-ONLY-SECRET'), KeyboardInterrupt()])
def test_interrupt_stops_once_cleans_and_rethrows_original(directory, capsys, interrupt, outcome):
    events = []
    if interrupt == 'sigterm':
        class Terminated:
            def __call__(self, command, **_kwargs):
                events.append('browser')
                kiosk.terminate(signal.SIGTERM, None)  # what the opt-in CLI handler raises
        run, expected = Terminated(), SystemExit
    else:
        run, expected = Browser(events, interrupt), type(interrupt)
    with pytest.raises(expected) as raised:
        kiosk.launch(ORIGIN, directory, get=healthy(directory), run=run,
                     stop_on_browser_exit=True, stop=Stop(directory, events, outcome))
    if interrupt == 'sigterm':
        assert raised.value.code == 128 + signal.SIGTERM
    else:
        assert raised.value is interrupt
    assert events == ['browser', ('stop', ORIGIN, INSTANCE, True, False)]
    assert not leftovers(directory) and not lock_held(directory)
    err = capsys.readouterr().err
    assert err == ('' if outcome is True else kiosk.STOP_UNCONFIRMED + '\n')
    assert 'TEST-ONLY' not in err


def test_sigterm_handler_unwinds_as_system_exit():
    with pytest.raises(SystemExit) as raised:
        kiosk.terminate(signal.SIGTERM, None)
    assert raised.value.code == 128 + signal.SIGTERM


@pytest.mark.parametrize('flag', [False, True])
@pytest.mark.parametrize('outcome', ['return', 'refusal', 'interrupt'])
def test_cli_plumbs_flag_and_restores_sigterm_handler(monkeypatch, tmp_path, flag, outcome):
    def sentinel(*_args): pass
    seen = {}
    def fake_launch(_url, _directory, **kwargs):
        seen.update(kwargs, handler=signal.getsignal(signal.SIGTERM))
        if outcome == 'refusal': raise ob.BootstrapError('refused')
        if outcome == 'interrupt': raise SystemExit(143)
        return 0
    monkeypatch.setattr(kiosk, 'launch', fake_launch)
    original = signal.signal(signal.SIGTERM, sentinel)
    try:
        argv = ['--url', ORIGIN, '--bootstrap-dir', str(tmp_path)] + (['--stop-on-browser-exit'] if flag else [])
        if outcome == 'interrupt':
            with pytest.raises(SystemExit): kiosk.main(argv)
        else:
            assert kiosk.main(argv) == (1 if outcome == 'refusal' else 0)
        assert seen['stop_on_browser_exit'] is flag
        assert seen['handler'] is (kiosk.terminate if flag else sentinel)
        assert signal.getsignal(signal.SIGTERM) is sentinel
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_DFL if original is None else original)


# ------------------------------------------------------------------ Stop helper
class Response:
    def __init__(self, body): self.body, self.sizes = body, []
    def __enter__(self): return self
    def __exit__(self, *_exc): return False
    def read(self, size):
        self.sizes.append(size)
        return self.body[:size]


def fake_opener(monkeypatch, body=b'', error=None):
    seen = {'opens': 0}
    class Opener:
        def open(self, request, timeout):
            seen.update(request=request, timeout=timeout, opens=seen['opens'] + 1)
            if error is not None: raise error
            seen['response'] = Response(body)
            return seen['response']
    def build(*handlers):
        seen['handlers'] = handlers
        return Opener()
    monkeypatch.setattr(kiosk.urllib.request, 'build_opener', build)
    return seen


def test_stop_helper_sends_one_exact_bounded_request(monkeypatch):
    seen = fake_opener(monkeypatch, json.dumps(RECEIPT).encode())
    assert kiosk.post_stop(ORIGIN, INSTANCE) is True
    request = seen['request']
    assert seen['opens'] == 1 and seen['timeout'] == 3
    assert request.full_url == ORIGIN + '/stop/immediate'
    assert request.get_method() == 'POST' and request.data == b''
    assert {name.lower(): value for name, value in request.header_items()} == {
        'origin': ORIGIN, HEADER.lower(): INSTANCE}  # no cookies or credentials
    assert seen['response'].sizes == [8193]
    proxies, redirects = seen['handlers']
    assert isinstance(proxies, kiosk.urllib.request.ProxyHandler) and proxies.proxies == {}
    assert isinstance(redirects, kiosk.NoRedirect)


def test_stop_helper_refuses_redirects():
    with pytest.raises(ob.BootstrapError):
        kiosk.NoRedirect().redirect_request(None, None, 307, 'Temporary Redirect', {}, 'http://elsewhere.invalid/')


@pytest.mark.parametrize('body', [
    json.dumps({**RECEIPT, 'padding': 'x' * 9000}).encode(),
    b'{"success": tr', b'[]', b'\xff', b'[' * 8000, b'',
    json.dumps({**RECEIPT, 'success': 'true'}).encode(),
    json.dumps({**RECEIPT, 'success': 1}).encode(),
    json.dumps({**RECEIPT, 'server_instance': 'b' * 32}).encode(),
    json.dumps({key: value for key, value in RECEIPT.items() if key != 'server_instance'}).encode(),
    json.dumps({**RECEIPT, 'command_epoch_required': False}).encode(),
    json.dumps({**RECEIPT, 'command_epoch': ''}).encode(),
    json.dumps({**RECEIPT, 'command_epoch': None}).encode(),
    json.dumps({'success': False, 'not_dispatched': True, 'message': 'x'}).encode(),
])
def test_stop_helper_rejects_malformed_oversized_or_negative_receipts(monkeypatch, body):
    seen = fake_opener(monkeypatch, body)
    assert kiosk.post_stop(ORIGIN, INSTANCE) is False
    assert seen['opens'] == 1 and seen['response'].sizes == [8193]


@pytest.mark.parametrize('error', [
    urllib.error.HTTPError(ORIGIN + '/stop/immediate', 409, 'Conflict', {}, None),
    urllib.error.URLError('refused'), TimeoutError(), ConnectionResetError(),
    ob.BootstrapError('Onboard API redirects are not allowed.'),
    http.client.RemoteDisconnected(), http.client.IncompleteRead(b''),
])
def test_stop_helper_transport_failure_is_unconfirmed_without_retry(monkeypatch, error):
    seen = fake_opener(monkeypatch, error=error)
    assert kiosk.post_stop(ORIGIN, INSTANCE) is False
    assert seen['opens'] == 1


# ------------------------------------------------------------------ API
async def held_move(client, rig):
    """A move admitted over HTTP and waiting at action_lock (caller holds it)."""
    await rig.ts.action_lock.acquire()
    pending = asyncio.create_task(client.post('/tools/move', json={'direction': 'forward'}))
    for _ in range(5):
        await harness._real_sleep(0)
    assert not pending.done()
    return pending


@pytest.mark.parametrize('path', ['/stop/immediate', '/queue/stop'])
@pytest.mark.parametrize('following', [False, True])
def test_matching_instance_stop_cancels_following_and_waiting_work(server, path, following):
    async def case():
        async with client_with_rig(server) as (client, rig):
            if following:
                assert (await client.post('/tools/follow', json={'enabled': True})).json()['success']
                assert await rig.settle()
                assert rig.robot.follower_on and rig.ts.following
            epoch, stops = rig.ts.command_epoch, rig.ts.stops
            pending = await held_move(client, rig)
            try:
                stopped = await asyncio.wait_for(client.post(path, headers={HEADER: rig.ts.command_instance}), timeout=1.0)
            finally:
                rig.ts.action_lock.release()
            assert 'cancelled by a stop' in (await asyncio.wait_for(pending, timeout=1.0)).json()['message']
            receipt = stopped.json()
            assert stopped.status_code == 200
            assert rig.ts.stops == stops + 1 and rig.ts.command_epoch != epoch
            assert receipt['command_epoch'] == rig.ts.command_epoch and receipt['command_epoch_required'] is True
            assert receipt['server_instance'] == rig.ts.command_instance
            assert not rig.robot.follower_on and rig.ts.following is False
            assert receipt['success'] is True
            assert await rig.settle()
            assert not harness.walk_twists(rig)
    asyncio.run(case())


@pytest.mark.parametrize('path', ['/stop/immediate', '/queue/stop'])
@pytest.mark.parametrize('expected', ['other', 'empty', 'duplicate'])
def test_mismatched_instance_is_409_without_dispatch_and_operator_stop_still_works(server, monkeypatch, path, expected):
    async def case():
        async with client_with_rig(server) as (client, rig):
            calls = []
            original = rig.ts.immediate_stop
            async def counted(*args, **kwargs):
                calls.append(kwargs)
                return await original(*args, **kwargs)
            monkeypatch.setattr(rig.ts, 'immediate_stop', counted)
            assert (await client.post('/tools/follow', json={'enabled': True})).json()['success']
            assert await rig.settle()
            assert rig.robot.follower_on and rig.ts.following
            headers = {'other': [(HEADER, 'b' * 32)], 'empty': [(HEADER, '')],
                       'duplicate': [(HEADER, rig.ts.command_instance)] * 2}[expected]
            pending = await held_move(client, rig)
            try:
                def snapshot():
                    status = rig.ts.status()
                    return (rig.ts.command_epoch, rig.ts.stops, rig.ts.following, status['queued'], status['running'],
                            rig.robot.follower_on, list(rig.robot.follow_calls),
                            list(rig.robot.switches), len(rig.robot.twists()))
                before = snapshot()
                refused = await client.post(path, headers=headers)
                for _ in range(5):
                    await harness._real_sleep(0)
                assert refused.status_code == 409
                assert refused.json() == {'success': False, 'not_dispatched': True, 'message': server.STOP_INSTANCE_MISMATCH}
                assert calls == [] and not pending.done() and snapshot() == before
                # No header: the unrestricted operator Stop.
                operator = await asyncio.wait_for(client.post(path), timeout=1.0)
            finally:
                rig.ts.action_lock.release()
            assert operator.status_code == 200 and len(calls) == 1
            assert operator.json()['server_instance'] == rig.ts.command_instance
            assert rig.ts.stops == before[1] + 1 and rig.ts.command_epoch != before[0]
            assert 'cancelled by a stop' in (await asyncio.wait_for(pending, timeout=1.0)).json()['message']
            assert await rig.settle()
            assert not harness.walk_twists(rig)
    asyncio.run(case())


def test_stop_receipt_uses_captured_robot_when_tools_swap_during_await(server):
    async def case():
        replacement = types.SimpleNamespace(command_instance='b' * 32, command_epoch='replacement-epoch')
        class Captured:
            command_instance, command_epoch = INSTANCE, 'before'
            async def immediate_stop(self, *, relax_playing_trick):
                assert relax_playing_trick is True
                server.tools = replacement
                await asyncio.sleep(0)
                self.command_epoch = 'after'
                return robot_api.Result(True, 'fake stop')
        server.tools = Captured()
        try:
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app),
                                         base_url='http://in-process.invalid') as client:
                response = await client.post('/stop/immediate', headers={HEADER: INSTANCE})
        finally:
            server.tools = None
        receipt = response.json()
        assert response.status_code == 200 and receipt['success'] is True
        assert (receipt['command_epoch'], receipt['server_instance']) == ('after', INSTANCE)
        assert receipt['command_epoch_required'] is True
    asyncio.run(case())


def test_matching_instance_stops_active_follow_and_cancels_admitted_queue(server):
    async def case():
        async with client_with_rig(server) as (client, rig):
            assert (await client.post('/tools/follow', json={'enabled': True})).json()['success']
            assert await rig.settle()
            assert rig.robot.follower_on and rig.ts.following
            entered = asyncio.Event()
            async def held_job():
                entered.set()
                await asyncio.Event().wait()
                raise AssertionError('cancelled job resumed')
            running = await rig.ts.queue('held diagnostic', held_job, 1., then=robot_api.UNCHANGED)
            await asyncio.wait_for(entered.wait(), 1.)
            queued = await rig.ts.wait(.2)
            assert running.ok and queued.ok and rig.ts.commands.qsize() == 1
            response = await client.post('/stop/immediate', headers={HEADER: rig.ts.command_instance})
            assert response.status_code == 200 and response.json()['success'] is True
            assert not rig.robot.follower_on and not rig.ts.following
            assert await rig.settle()
            outcomes = {row['job_id']: row for row in rig.ts.recent_outcomes()['outcomes']}
            for receipt in (running, queued):
                assert outcomes[receipt.details['submission']['job_id']]['terminal_state'] == 'cancelled'
            assert outcomes[queued.details['submission']['job_id']]['execution_started'] is False
            assert rig.ts.commands.empty() and not harness.walk_twists(rig)
    asyncio.run(case())
