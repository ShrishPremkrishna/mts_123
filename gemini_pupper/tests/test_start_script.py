"""Shell startup ordering with every subprocess mocked; no ROS or hardware."""
import json
import os
from pathlib import Path
import subprocess
import sys

SCRIPT = Path(__file__).resolve().parents[1] / 'robot/start.sh'


def run_start(tmp_path, mode, kiosk=False):
    root = tmp_path / 'gemini'; robot = root / 'robot'; robot.mkdir(parents=True)
    (root / 'web/dist').mkdir(parents=True); (root / 'web/dist/index.html').touch()
    runtime = tmp_path / 'runtime'; runtime.mkdir()
    script = robot / 'start.sh'
    script.write_text(SCRIPT.read_text().replace('#!/bin/bash', '#!/bin/bash\nkill() { kill_fixture "$@"; }', 1).replace('LOG=/tmp/gemini-pupper', 'LOG=' + str(runtime)))
    key = tmp_path / 'key'; key.write_text('fixture-not-a-real-key')
    commands = tmp_path / 'bin'; commands.mkdir()
    log = tmp_path / 'calls.jsonl'
    mock = '#!' + sys.executable + '\n' + r'''import json, os, sys
from pathlib import Path
name = Path(sys.argv[0]).name; args = sys.argv[1:]
with open(os.environ['MOCK_LOG'], 'a') as out: out.write(json.dumps([name, args]) + '\n')
mode = os.environ['MOCK_MODE']
if name == 'python3':
    if args[0].endswith('session_processes.py'):
        action = args[3]
        if action == 'begin' and mode == 'occupied': sys.exit(1)
        if action == 'begin': print('1' * 32)
        if action == 'spawn':
            if args[4] == 'launch': Path(args[2], 'controllers_ready').write_text('fixture ready')
            if args[4] == 'kiosk': Path(args[2], 'kiosk.log').write_text('fixture kiosk\n')
        sys.exit(0)
    if args[0] == '-c': print('/fixture-monorepo')
    sys.exit(0)
if name == 'ss' and mode == 'occupied': print('LISTEN 0 128 0.0.0.0:8000 0.0.0.0:* users:(("foreign",pid=424242,fd=7))')
if name == 'setsid': Path(os.environ['MOCK_RUNTIME'], 'controllers_ready').write_text('fixture ready')
if name == 'pgrep': sys.exit(0 if mode == 'borrowed' else 1)
if name == 'bash' and mode == 'incompatible': sys.exit(1)
sys.exit(0)
'''
    for name in ['python3', 'flock', 'pgrep', 'bash', 'curl', 'sleep', 'sudo', 'amixer', 'wpctl', 'setsid', 'pkill', 'ss', 'kill_fixture']:
        path = commands / name; path.write_text(mock); path.chmod(0o700)
    env = {**os.environ, 'PATH': str(commands) + os.pathsep + os.environ['PATH'],
           'MOCK_LOG': str(log), 'MOCK_MODE': mode, 'MOCK_RUNTIME': str(runtime), 'GEMINI_KEY_FILE': str(key)}
    result = subprocess.run(['/bin/bash', str(script), *(['--kiosk'] if kiosk else [])], env=env,
                            text=True, capture_output=True, timeout=10)
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    actions = [args[3:] for name, args in calls if name == 'python3' and args[0].endswith('session_processes.py')]
    return result, calls, actions


def test_occupied_port_refuses_before_preflight_or_any_launch(tmp_path):
    result, calls, actions = run_start(tmp_path, 'occupied')
    assert result.returncode != 0
    assert [action[0] for action in actions] == ['check', 'begin']
    assert not any(name in ['pgrep', 'bash', 'setsid', 'pkill', 'curl'] for name, _ in calls)
    assert not any(args[0].endswith('preflight.py') for name, args in calls if name == 'python3')


def test_created_stack_is_receipted_and_checked_before_server(tmp_path):
    result, calls, actions = run_start(tmp_path, 'created')
    assert result.returncode == 0
    assert ['ros', 'created'] in actions
    roles = [action[1] for action in actions if action[0] == 'spawn']
    assert roles == ['launch', 'server']
    inspect = next(i for i, (name, args) in enumerate(calls) if name == 'bash')
    server = next(i for i, (name, args) in enumerate(calls) if name == 'python3' and args[3:5] == ['spawn', 'server'])
    assert inspect < server
    assert not any(name in ['pkill', 'setsid'] for name, _ in calls)


def test_borrowed_stack_never_launches_ros(tmp_path):
    result, calls, actions = run_start(tmp_path, 'borrowed')
    assert result.returncode == 0
    assert ['ros', 'borrowed'] in actions
    assert [action[1] for action in actions if action[0] == 'spawn'] == ['server']


def test_failed_readiness_retains_recovery_and_does_not_start_server(tmp_path):
    result, calls, actions = run_start(tmp_path, 'incompatible')
    assert result.returncode != 0
    assert [action[1] for action in actions if action[0] == 'spawn'] == ['launch']
    assert 'recovery receipt retained' in result.stderr
    assert not any(action[0] == 'cleanup' for action in actions)


def test_kiosk_children_and_gui_change_are_recorded(tmp_path):
    result, calls, actions = run_start(tmp_path, 'created', kiosk=True)
    assert result.returncode == 0
    assert [action[1] for action in actions if action[0] == 'spawn'] == ['launch', 'server', 'chromium', 'kiosk']
    assert ['stop-gui'] in actions
    assert not any(name == 'pkill' for name, _ in calls)


def test_stop_progresses_while_start_is_blocked_in_readiness(tmp_path):
    """Run both real shells concurrently; fixture helpers replace all hardware."""
    import time
    root = tmp_path / 'gemini'; robot = root / 'robot'; robot.mkdir(parents=True)
    (root / 'web/dist').mkdir(parents=True); (root / 'web/dist/index.html').touch()
    runtime = tmp_path / 'runtime'; runtime.mkdir()
    for name in ('start.sh', 'stop.sh'):
        (robot / name).write_text((SCRIPT.parent / name).read_text().replace('LOG=/tmp/gemini-pupper', 'LOG=' + str(runtime)))
    commands = tmp_path / 'bin'; commands.mkdir()
    mock = '#!' + sys.executable + '\n' + r'''import fcntl, json, os, sys, time
from pathlib import Path
runtime = Path(os.environ['MOCK_RUNTIME']); name = Path(sys.argv[0]).name; args = sys.argv[1:]
if name == 'bash':
    (runtime / 'readiness-blocked').touch()
    until = time.monotonic() + 8
    while not (runtime / 'release-readiness').exists():
        if time.monotonic() > until: sys.exit(1)
        time.sleep(.02)
    sys.exit(0)
if name == 'pgrep': sys.exit(0)  # borrow; never run a real launch
if name != 'python3': raise SystemExit('unexpected command ' + name)
if args[0] == '-c': print('/fixture-monorepo'); sys.exit(0)
if args[0].endswith('preflight.py'): sys.exit(0)
assert args[0].endswith('session_processes.py')
action = args[3]
with (runtime / 'fixture.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    state_path = runtime / 'fixture.json'
    state = json.loads(state_path.read_text()) if state_path.exists() else {}
    token = os.environ.get('GEMINI_PUPPER_SESSION')
    with (runtime / 'actions.jsonl').open('a') as out: out.write(json.dumps([action, token]) + '\n')
    if action == 'check': sys.exit(0)
    if action == 'begin': state = {'status': 'starting', 'session': '1' * 32}; print(state['session'])
    elif action == 'stopping': state['status'] = 'stopping'; print(state['session'])
    else:
        assert token == state['session'], 'shell failed to carry its begin/stopping token'
        if action == 'api-owned': sys.exit(1)  # no server was started
        if action == 'cleanup': state['status'] = 'stopped'; (runtime / 'cleanup-done').touch()
        elif action in ('spawn', 'ready', 'refresh', 'ros'):
            if state['status'] != 'starting': sys.exit(1)
            if action == 'spawn': (runtime / 'unexpected-spawn').touch()
    state_path.write_text(json.dumps(state))
'''
    for name in ('python3', 'bash', 'pgrep'):
        path = commands / name; path.write_text(mock); path.chmod(0o700)
    env = {**os.environ, 'PATH': str(commands) + os.pathsep + os.environ['PATH'], 'MOCK_RUNTIME': str(runtime)}
    start = subprocess.Popen(['/bin/bash', str(robot / 'start.sh')], env=env, text=True,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        until = time.monotonic() + 5
        while not (runtime / 'readiness-blocked').exists():
            assert start.poll() is None
            assert time.monotonic() < until
            time.sleep(.02)
        stop = subprocess.run(['/bin/bash', str(robot / 'stop.sh')], env=env, text=True,
                              capture_output=True, timeout=5)
        assert stop.returncode != 0  # honestly reports no server/API acceptance
        assert (runtime / 'cleanup-done').exists()
        assert not (runtime / 'release-readiness').exists()
        assert start.poll() is None  # Stop progressed while readiness was blocked.
        (runtime / 'release-readiness').touch()
        stdout, stderr = start.communicate(timeout=5)
        assert start.returncode != 0
        assert 'recovery receipt retained' in stderr
        assert not (runtime / 'unexpected-spawn').exists()
        actions = [json.loads(line)[0] for line in (runtime / 'actions.jsonl').read_text().splitlines()]
        assert actions.index('stopping') < actions.index('cleanup') < actions.index('spawn')
    finally:
        if start.poll() is None:
            (runtime / 'release-readiness').touch()
            start.communicate(timeout=5)
