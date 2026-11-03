"""Onboard admission and browser-only launch; fake ASGI/commands, no network/devices."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import fcntl
import json
import os
from pathlib import Path
import sys
import threading
from urllib.parse import unquote, urlsplit
import re
import stat
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import onboard_bootstrap as ob
import kiosk
from test_server_transport import server  # fake ROS + forbidden network/subprocess fixture
import httpx

ORIGIN = 'http://localhost:18090'
INSTANCE = 'a' * 32


@pytest.fixture
def bootstrap(tmp_path):
    key = tmp_path / 'credential'; key.write_text('TEST-ONLY-NOT-A-CREDENTIAL'); key.chmod(0o600)
    return ob.OnboardBootstrap(tmp_path / 'onboard', key)


def issue(bootstrap):
    return ob.issue_grant(bootstrap.directory, ORIGIN, INSTANCE)


def test_disabled_without_explicit_environment(monkeypatch):
    monkeypatch.delenv('PUPPER_ONBOARD_BOOTSTRAP_DIR', raising=False)
    assert ob.OnboardBootstrap.from_environment() is None


def test_one_use_private_grant_contains_no_key_or_nonce(bootstrap):
    nonce, path = issue(bootstrap)
    before = path.read_text()
    assert nonce not in before and 'TEST-ONLY' not in before
    result = bootstrap.consume(nonce, ORIGIN, INSTANCE)
    assert result == {'schema': ob.SCHEMA, 'apiKey': 'TEST-ONLY-NOT-A-CREDENTIAL',
                      'config': ob.CONFIG, 'autoConnect': True}
    assert not path.exists()
    with pytest.raises(ob.BootstrapError): bootstrap.consume(nonce, ORIGIN, INSTANCE)


def test_atomic_consumption_while_first_reader_owns_grant(bootstrap, monkeypatch):
    nonce, path = issue(bootstrap)
    entered = threading.Event()
    release = threading.Event()
    original = ob.read_private
    def read(target, limit):
        data = original(target, limit)
        if target == path:
            entered.set()
            assert release.wait(timeout=3)
        return data
    monkeypatch.setattr(ob, 'read_private', read)
    def consume():
        try: return bootstrap.consume(nonce, ORIGIN, INSTANCE)
        except ob.BootstrapError as error: return error.status
    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(consume)
        assert entered.wait(timeout=3)
        assert pool.submit(consume).result(timeout=3) == 410
        release.set()
        assert isinstance(first.result(timeout=3), dict)
    assert not path.exists()



@pytest.mark.parametrize('change', ['expired', 'future', 'restart', 'origin', 'config', 'malformed'])
def test_bad_grant_does_not_release_key(bootstrap, change):
    nonce, path = issue(bootstrap)
    data = json.loads(path.read_text())
    if change == 'expired': data['issued'] -= 61; data['expires'] -= 61
    if change == 'future': data['issued'] += 1; data['expires'] += 1
    if change == 'restart': data['server_instance'] = 'b' * 32
    if change == 'origin': data['origin'] = 'http://localhost:8000'
    if change == 'config': data['config']['robotApiUrl'] = 'http://foreign.invalid'
    if change == 'malformed': data = []
    path.write_text(json.dumps(data))
    with pytest.raises(ob.BootstrapError): bootstrap.consume(nonce, ORIGIN, INSTANCE)


def test_key_failure_consumes_grant_without_echoing_secret(bootstrap):
    nonce, _ = issue(bootstrap)
    bootstrap.key_file.chmod(0o644)
    with pytest.raises(ob.BootstrapError) as error: bootstrap.consume(nonce, ORIGIN, INSTANCE)
    assert error.value.status == 503 and 'TEST-ONLY' not in str(error.value)
    bootstrap.key_file.chmod(0o600)
    with pytest.raises(ob.BootstrapError): bootstrap.consume(nonce, ORIGIN, INSTANCE)


def test_reject_symlink_key_and_shared_directory(bootstrap, tmp_path):
    key = bootstrap.key_file
    link = tmp_path / 'link'; link.symlink_to(key)
    bootstrap.key_file = link
    nonce, _ = issue(bootstrap)
    with pytest.raises(ob.BootstrapError): bootstrap.consume(nonce, ORIGIN, INSTANCE)
    bootstrap.directory.chmod(0o755)
    with pytest.raises(ob.BootstrapError): ob.private_directory(bootstrap.directory)


@pytest.mark.parametrize('origin', ['http://robot.local:18090', 'https://localhost:18090',
    'http://localhost:18090/', 'http://user@localhost:18090', 'http://localhost:18090?q=x', 'http://localhost:0'])
def test_nonexact_origin_refused_before_any_launch(tmp_path, origin):
    with pytest.raises(ob.BootstrapError): kiosk.launch(origin, tmp_path / 'onboard', get=lambda *_: pytest.fail('HTTP attempted'))


def test_browser_launch_only_once_against_healthy_api(bootstrap):
    calls = []
    def get(origin, path):
        calls.append(path)
        if path == '/health': return {'status': 'running', 'tools': True}
        assert path == '/onboard/status'
        return {'schema': ob.SCHEMA, 'enabled': True, 'server_instance': INSTANCE,
                'directory_id': ob.directory_id(bootstrap.directory)}
    def run(command, **kwargs):
        assert command[0] == 'chromium'
        assert not any('debug' in arg or 'TEST-ONLY' in arg for arg in command)
        assert command[-1].startswith('file:///')
        page = Path(unquote(urlsplit(command[-1]).path))
        assert page.parent == bootstrap.directory
        assert stat.S_IMODE(page.stat().st_mode) == 0o600
        document = page.read_text()
        nonce = re.search(r'#onboard=([0-9a-f]{64})', document).group(1)
        assert 'TEST-ONLY' not in document
        assert nonce not in ' '.join(command)
        assert bootstrap.consume(nonce, ORIGIN, INSTANCE)['config'] == ob.CONFIG
        assert kwargs['stdout'] == kwargs['stderr'] == kiosk.subprocess.DEVNULL
        assert kwargs['pass_fds']
        calls.append('browser')
        return SimpleNamespace(returncode=17)
    assert kiosk.launch(ORIGIN, bootstrap.directory, get=get, run=run) == 17
    assert calls == ['/health', '/onboard/status', 'browser']
    assert not list(bootstrap.directory.glob('*.grant'))
    assert not list(bootstrap.directory.glob('launch-*.html'))


def test_unhealthy_api_and_duplicate_launcher_never_spawn(bootstrap):
    def forbidden(*_args, **_kwargs): pytest.fail('browser spawned')
    with pytest.raises(ob.BootstrapError):
        kiosk.launch(ORIGIN, bootstrap.directory, get=lambda *_: {'status': 'running', 'tools': False}, run=forbidden)
    with (bootstrap.directory / 'browser.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(ob.BootstrapError):
            kiosk.launch(ORIGIN, bootstrap.directory, get=lambda *_: pytest.fail('HTTP before ownership'), run=forbidden)


def test_existing_chromium_profile_owner_not_reused(bootstrap):
    profile = bootstrap.directory / 'browser-profile'; profile.mkdir()
    (profile / 'SingletonLock').symlink_to('hostname-12345')
    def get(_origin, path):
        if path == '/health': return {'status': 'running', 'tools': True}
        return {'schema': ob.SCHEMA, 'enabled': True, 'server_instance': INSTANCE,
                'directory_id': ob.directory_id(bootstrap.directory)}
    with pytest.raises(ob.BootstrapError):
        kiosk.launch(ORIGIN, bootstrap.directory, get=get, run=lambda *_: pytest.fail('existing browser reused'))


def test_http_contract_bypasses_motion_epoch_but_requires_one_use_origin(server, bootstrap):
    async def case():
        server.onboard_bootstrap = bootstrap
        server.tools = SimpleNamespace(command_instance=INSTANCE)  # no Robot operations available
        try:
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app), base_url=ORIGIN,
                                         headers={'Origin': ORIGIN}) as client:
                status = await client.get('/onboard/status')
                assert status.status_code == 200 and status.json()['server_instance'] == INSTANCE
                nonce, _ = issue(bootstrap)
                result = await client.post('/onboard/bootstrap', json={'nonce': nonce})
                assert result.status_code == 200 and result.json()['apiKey'] == 'TEST-ONLY-NOT-A-CREDENTIAL'
                assert result.headers['cache-control'] == 'no-store'
                assert result.headers['referrer-policy'] == 'no-referrer'
                repeat = await client.post('/onboard/bootstrap', json={'nonce': nonce})
                assert repeat.status_code == 410 and 'apiKey' not in repeat.json()
        finally: server.tools = None
    asyncio.run(case())


@pytest.mark.parametrize('change', ['disabled', 'foreign_origin', 'missing_origin', 'foreign_host', 'remote_peer', 'forwarded', 'oversized'])
def test_http_refusals_never_expose_key(server, bootstrap, change):
    async def case():
        server.onboard_bootstrap = None if change == 'disabled' else bootstrap
        server.tools = SimpleNamespace(command_instance=INSTANCE)
        nonce, path = issue(bootstrap)
        headers = {'Origin': ORIGIN}
        if change == 'foreign_origin': headers['Origin'] = 'http://localhost:9999'
        if change == 'missing_origin': headers = {}
        if change == 'foreign_host': headers['Host'] = 'robot.lan:18090'
        if change == 'forwarded': headers['X-Forwarded-For'] = '127.0.0.1'
        peer = ('10.0.0.5', 1234) if change == 'remote_peer' else ('127.0.0.1', 1234)
        try:
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app, client=peer), base_url=ORIGIN) as client:
                body = {'nonce': nonce, 'padding': 'x' * 300} if change == 'oversized' else {'nonce': nonce}
                response = await client.post('/onboard/bootstrap', json=body, headers=headers)
                assert response.status_code in (400, 403, 404)
                assert 'TEST-ONLY' not in response.text and 'apiKey' not in response.text
                assert path.exists()  # hostile/invalid requests cannot consume operator admission
        finally: server.tools = None
    asyncio.run(case())


def test_private_launch_page_and_unconsumed_grant_expire_while_browser_is_open(bootstrap, monkeypatch):
    timers = []
    class Timer:
        def __init__(self, seconds, callback):
            assert seconds == ob.TTL_SECONDS
            self.callback = callback; timers.append(self)
        def start(self): pass
        def cancel(self): self.cancelled = True
    monkeypatch.setattr(kiosk.threading, 'Timer', Timer)
    def get(_origin, path):
        if path == '/health': return {'status': 'running', 'tools': True}
        return {'schema': ob.SCHEMA, 'enabled': True, 'server_instance': INSTANCE,
                'directory_id': ob.directory_id(bootstrap.directory)}
    def run(command, **_kwargs):
        page = Path(unquote(urlsplit(command[-1]).path))
        assert page.is_file() and len(list(bootstrap.directory.glob('*.grant'))) == 1
        timers[0].callback()
        assert not page.exists() and not list(bootstrap.directory.glob('*.grant'))
        return SimpleNamespace(returncode=0)
    assert kiosk.launch(ORIGIN, bootstrap.directory, get=get, run=run) == 0
    assert timers[0].cancelled
