"""Ownership and shutdown-order checks with fake /proc; never signal the host."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
from types import SimpleNamespace

import pytest

PATH = Path(__file__).resolve().parents[1] / 'robot/session_processes.py'
spec = importlib.util.spec_from_file_location('session_processes_under_test', PATH)
owned = importlib.util.module_from_spec(spec)
spec.loader.exec_module(owned)
SESSION = '1' * 32


@pytest.fixture
def proc(tmp_path, monkeypatch):
    root = tmp_path / 'proc'; root.mkdir()
    (root / 'sys/kernel/random').mkdir(parents=True)
    (root / 'sys/kernel/random/boot_id').write_text('test-boot')
    monkeypatch.setattr(owned, 'PROC', root)
    directory = tmp_path / 'runtime'; directory.mkdir(mode=0o700)
    binaries = tmp_path / 'bin'; binaries.mkdir()
    receipt = dict(schema=owned.SCHEMA, uid=os.getuid(), root=str(owned.HERE), boot_id='test-boot',
                   session_id=SESSION, status='running', ros='created', kiosk=False, gui_stopped=False,
                   processes={}, pending={}, events=[])
    def make(pid, executable='python', args=None, session=SESSION, ticks='100'):
        path = root / str(pid); path.mkdir(exist_ok=True)
        fields = ['S'] + ['0'] * 18 + [ticks]
        (path / 'stat').write_text(str(pid) + ' (test process) ' + ' '.join(fields))
        (path / 'environ').write_bytes((owned.TAG + '=' + session).encode() + b'\0')
        (path / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in (args or [executable])) + b'\0')
        binary = binaries / executable; binary.touch()
        (path / 'exe').symlink_to(binary)
        return owned.capture(pid, session)
    def finish(pid):
        path = root / str(pid) / 'stat'
        path.write_text(path.read_text().replace(') S ', ') Z '))
    def write(): owned.save(directory, receipt)
    clock = SimpleNamespace(now=10.)
    monkeypatch.setattr(owned, 'time', SimpleNamespace(monotonic=lambda: clock.now,
        sleep=lambda seconds: setattr(clock, 'now', clock.now + seconds)))
    return SimpleNamespace(root=root, directory=directory, receipt=receipt, make=make, finish=finish,
                           write=write, clock=clock)


@pytest.mark.parametrize('change', ['ticks', 'session', 'exe', 'argv'])
def test_changed_identity_is_never_signaled(proc, monkeypatch, change):
    target = proc.make(41)
    if change == 'ticks':
        path = proc.root / '41/stat'; path.write_text(path.read_text().replace('100', '101'))
    elif change == 'session': (proc.root / '41/environ').write_bytes(b'foreign\0')
    elif change == 'argv': (proc.root / '41/cmdline').write_bytes(b'changed\0')
    else:
        path = proc.root / '41/exe'; path.unlink(); path.symlink_to('/bin/sh')
    calls = []
    monkeypatch.setattr(owned.os, 'pidfd_open', lambda pid: 99, raising=False)
    monkeypatch.setattr(owned.os, 'close', lambda descriptor: None)
    monkeypatch.setattr(owned.signal, 'pidfd_send_signal', lambda *args: calls.append(args), raising=False)
    if change == 'ticks': assert owned.send(target, SESSION, signal.SIGINT) is False
    else:
        with pytest.raises(ValueError): owned.send(target, SESSION, signal.SIGINT)
    assert calls == []


def test_pidfd_pins_verified_task_and_closes_descriptor(proc, monkeypatch):
    target = proc.make(41); calls = []
    monkeypatch.setattr(owned.os, 'pidfd_open', lambda pid: calls.append(('open', pid)) or 99, raising=False)
    monkeypatch.setattr(owned.os, 'close', lambda fd: calls.append(('close', fd)))
    monkeypatch.setattr(owned.signal, 'pidfd_send_signal', lambda fd, sig: calls.append(('signal', fd, sig)), raising=False)
    assert owned.send(target, SESSION, signal.SIGINT)
    assert calls == [('open', 41), ('signal', 99, signal.SIGINT), ('close', 99)]


def test_borrowed_ros_is_never_captured_or_signaled(proc, monkeypatch):
    proc.receipt['ros'] = 'borrowed'
    proc.make(10, 'ros2_control_node', session='f' * 32)
    proc.make(11, 'foreign-launch', session='f' * 32)
    proc.receipt['processes']['server'] = proc.make(20)
    proc.write(); sent = []
    def send(target, session, sig):
        sent.append((target['pid'], sig)); proc.finish(target['pid']); return True
    monkeypatch.setattr(owned, 'send', send)
    owned.cleanup(proc.directory)
    assert sent == [(20, signal.SIGTERM)]
    assert owned.load(proc.directory)['status'] == 'stopped'
    assert ') S ' in (proc.root / '10/stat').read_text()


def owned_ros(proc):
    for role, pid, exe in [('manager', 10, 'ros2_control_node'), ('launch', 11, 'ros2'), ('server', 20, 'python')]:
        proc.receipt['processes'][role] = proc.make(pid, exe)
    proc.write()


def test_manager_exits_before_launch_then_tagged_orphans_are_cleaned(proc, monkeypatch):
    owned_ros(proc)
    proc.make(30, 'follow_service')  # inherited session tag, parent already gone
    proc.make(31, 'foreign-service', session='f' * 32)
    sent = []
    def send(target, session, sig):
        if target['pid'] == 11: assert owned.state(proc.receipt['processes']['manager'], SESSION) == 'exited'
        sent.append((target['pid'], sig)); proc.finish(target['pid']); return True
    monkeypatch.setattr(owned, 'send', send)
    owned.cleanup(proc.directory)
    assert sent[:3] == [(20, signal.SIGTERM), (10, signal.SIGINT), (11, signal.SIGINT)]
    assert sent[3:] == [(30, signal.SIGTERM)]
    assert all(sig != signal.SIGKILL for _, sig in sent)
    assert owned.load(proc.directory)['shutdown']['physical_stopping_proven'] is False


def test_manager_timeout_never_signals_launch_or_repeats_manager_signal(proc, monkeypatch):
    owned_ros(proc); sent = []
    monkeypatch.setattr(owned, 'send', lambda target, session, sig: sent.append((target['pid'], sig)) or True)
    for _ in range(2):
        with pytest.raises(ValueError, match='manager exit unconfirmed'):
            owned.cleanup(proc.directory)
    assert sent == [(20, signal.SIGTERM), (10, signal.SIGINT)]
    assert owned.load(proc.directory)['status'] == 'stopping'


def test_failed_start_with_no_manager_does_not_forward_launch_shutdown(proc, monkeypatch):
    proc.receipt['status'] = 'starting'
    proc.receipt['processes']['launch'] = proc.make(11, 'ros2')
    proc.write()
    monkeypatch.setattr(owned, 'send', lambda *args: pytest.fail('no signal allowed'))
    with pytest.raises(ValueError, match='manager identity was never captured'):
        owned.cleanup(proc.directory)
    assert 'launch' in owned.load(proc.directory)['processes']


@pytest.mark.parametrize('fault', ['missing', 'root', 'boot_id', 'uid', 'schema'])
def test_missing_or_foreign_receipt_refuses_before_signals(proc, monkeypatch, fault):
    if fault != 'missing':
        proc.receipt[fault] = -1 if fault == 'uid' else 'foreign'
        proc.write()
    monkeypatch.setattr(owned, 'send', lambda *args: pytest.fail('no signal allowed'))
    with pytest.raises((FileNotFoundError, ValueError)):
        owned.cleanup(proc.directory)


def test_port_owner_must_match_recorded_live_server(proc, monkeypatch):
    proc.receipt['processes']['server'] = proc.make(20)
    proc.write()
    for lines in ([], ['LISTEN users:(("python",pid=21,fd=7))'], ['LISTEN no-process-info'],
                  ['LISTEN users:(("python",pid=20,fd=7))', 'LISTEN users:(("foreign",pid=21,fd=7))']):
        monkeypatch.setattr(owned, 'listeners', lambda: lines)
        with pytest.raises(ValueError): owned.api_owned(proc.directory)
    monkeypatch.setattr(owned, 'listeners', lambda: ['LISTEN users:(("python",pid=20,fd=7))'])
    owned.api_owned(proc.directory)


def test_occupied_port_refuses_begin_without_replacing_receipt(proc, monkeypatch):
    monkeypatch.setattr(owned, 'check_support', lambda directory: None)
    monkeypatch.setattr(owned, 'listeners', lambda: ['LISTEN unknown owner'])
    with pytest.raises(ValueError, match='port 8000 is occupied'): owned.begin(proc.directory, False)
    assert not (proc.directory / 'owned_session.json').exists()


def test_existing_unfinished_receipt_prevents_restart(proc, monkeypatch):
    proc.write(); before = (proc.directory / 'owned_session.json').read_bytes()
    monkeypatch.setattr(owned, 'check_support', lambda directory: None)
    monkeypatch.setattr(owned, 'free_port', lambda: pytest.fail('do not continue startup'))
    with pytest.raises(ValueError, match='session/recovery receipt exists'): owned.begin(proc.directory, False)
    assert (proc.directory / 'owned_session.json').read_bytes() == before


def test_duplicate_manager_refuses_ambiguous_cleanup(proc, monkeypatch):
    owned_ros(proc); proc.make(12, 'ros2_control_node')
    sent = []
    monkeypatch.setattr(owned, 'send', lambda target, session, sig: sent.append((target['pid'], sig)) or True)
    with pytest.raises(ValueError, match='different controller manager'): owned.cleanup(proc.directory)
    assert sent == [(20, signal.SIGTERM)]


@pytest.mark.parametrize('phase', ['gate', 'bash', 'final'])
def test_final_exec_capture_never_accepts_gate_or_ros_sourcing(proc, monkeypatch, phase):
    expected = '/robot/server.py'
    executable = str(Path(owned.sys.executable).resolve())
    args = [owned.sys.executable, '-u', expected]
    if phase == 'gate': args = [owned.sys.executable, str(PATH), '--exec', '9', *args]
    if phase == 'bash': executable = '/bin/bash'; args = ['bash', '-c', 'source ROS; exec python3 "$1"', '_', expected]
    monkeypatch.setattr(owned, 'capture', lambda *arguments: {'executable': executable, 'argv': args})
    if phase == 'final': assert owned.capture_spawn(41, SESSION, 'server', expected)['argv'] == args
    else:
        with pytest.raises(ValueError): owned.capture_spawn(41, SESSION, 'server', expected)


def test_stop_cancels_waiting_start_and_stale_starter_cannot_touch_replacement(proc, monkeypatch):
    proc.receipt['status'] = 'starting'; proc.receipt['ros'] = 'borrowed'; proc.write()
    assert owned.operate(proc.directory, 'stopping', [], None) == SESSION
    for action, arguments in [('refresh', []), ('spawn', ['server', 'server.py', 'python3', 'server.py']),
                              ('ready', []), ('ros', ['created']), ('stop-gui', [])]:
        with pytest.raises(ValueError, match='startup canceled'):
            owned.operate(proc.directory, action, arguments, SESSION)
    monkeypatch.setattr(owned, 'check_support', lambda directory: None)
    monkeypatch.setattr(owned, 'free_port', lambda: None)
    with pytest.raises(ValueError, match='session/recovery receipt exists'):
        owned.begin(proc.directory, False)
    owned.operate(proc.directory, 'cleanup', [], SESSION)
    replacement = owned.begin(proc.directory, False)
    assert replacement != SESSION
    for action, arguments in [('refresh', []), ('spawn', ['server', 'server.py', 'python3', 'server.py']),
                              ('ready', []), ('cleanup', []), ('api-owned', [])]:
        with pytest.raises(ValueError, match='caller session does not match'):
            owned.operate(proc.directory, action, arguments, SESSION)
    assert owned.load(proc.directory)['session_id'] == replacement
    assert owned.load(proc.directory)['status'] == 'starting'


def test_incomplete_ros_cleanup_still_closes_owned_command_ingress(proc, monkeypatch):
    proc.receipt['status'] = 'starting'
    proc.receipt['processes']['launch'] = proc.make(11, 'ros2')
    proc.receipt['processes']['server'] = proc.make(20)
    proc.write(); sent = []
    def send(target, session, sig):
        sent.append((target['pid'], sig)); proc.finish(target['pid']); return True
    monkeypatch.setattr(owned, 'send', send)
    with pytest.raises(ValueError, match='manager identity was never captured'):
        owned.cleanup(proc.directory)
    assert sent == [(20, signal.SIGTERM)]
    assert owned.load(proc.directory)['status'] == 'stopping'


@pytest.mark.parametrize('active', [False, True])
def test_gui_restoration_requires_prior_active_state(proc, monkeypatch, active):
    proc.receipt['status'] = 'starting'; proc.receipt['ros'] = 'borrowed'; proc.write()
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0 if active or command[0] == 'sudo' else 3)
    monkeypatch.setattr(owned.subprocess, 'run', run)
    owned.operate(proc.directory, 'stop-gui', [], SESSION)
    owned.operate(proc.directory, 'stopping', [], None)
    owned.operate(proc.directory, 'cleanup', [], SESSION)
    expected = [['systemctl', 'is-active', '--quiet', 'pupper-gui']]
    if active: expected += [['sudo', '-n', 'systemctl', 'stop', 'pupper-gui']]
    expected += [['amixer', '-q', 'sset', 'Master', '100%'], ['wpctl', 'set-volume', '@DEFAULT_SINK@', '1.0']]
    if active: expected += [['sudo', '-n', 'systemctl', 'start', 'pupper-gui']]
    assert calls == expected
    assert owned.load(proc.directory)['gui_stopped'] is False


def test_shell_does_not_lock_across_ros_readiness_waits():
    robot = PATH.parent
    assert 'flock' not in (robot / 'start.sh').read_text()
    assert 'flock' not in (robot / 'stop.sh').read_text()


def test_pending_exec_refusal_does_not_skip_known_server_cleanup(proc, monkeypatch):
    proc.receipt['status'] = 'starting'
    gate = proc.make(11, 'bash', args=['bash', '/robot/launch.py'])
    proc.receipt['pending']['launch'] = {'gate': gate, 'expected': '/robot/launch.py', 'command': ['bash']}
    proc.receipt['processes']['server'] = proc.make(20)
    proc.write(); sent = []
    def send(target, session, sig):
        sent.append((target['pid'], sig)); proc.finish(target['pid']); return True
    monkeypatch.setattr(owned, 'send', send)
    with pytest.raises(ValueError, match='expected final executable'):
        owned.cleanup(proc.directory)
    assert sent == [(20, signal.SIGTERM)]
    assert 'launch' in owned.load(proc.directory)['pending']
    assert owned.load(proc.directory)['status'] == 'stopping'
