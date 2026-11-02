"""Run setup with installation/network commands mocked; never touch user setup."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

SETUP = Path(__file__).resolve().parents[1] / 'robot/setup.sh'


def run_setup(tmp_path, layout):
    root = tmp_path / 'checkout with spaces'
    robot = root / 'gemini_pupper/robot'; robot.mkdir(parents=True)
    (root / 'gemini_pupper/web').mkdir()
    coordinator = root / 'foundation/hardware/floor_stock/shutdown_coordinator.py'
    if layout != 'missing':
        target = (root / 'gemini_pupper/foundation/hardware/floor_stock/shutdown_coordinator.py'
                  if layout == 'nested' else coordinator)
        target.parent.mkdir(parents=True)
        if layout == 'directory': target.mkdir()
        else:
            target.write_text('# fixture: presence/readability check only\n')
            if layout == 'unreadable': target.chmod(0)
    commands = tmp_path / 'commands'; commands.mkdir()
    node_dir = tmp_path / 'node'; (node_dir / 'bin').mkdir(parents=True)
    log = tmp_path / 'calls.jsonl'
    mock = '#!' + sys.executable + '\n' + r'''import json, os, sys
from pathlib import Path
name = Path(sys.argv[0]).name
with open(os.environ['SETUP_CALLS'], 'a') as out:
    out.write(json.dumps({'command': name, 'args': sys.argv[1:]}) + '\n')
if name == 'node': print('v22.23.3'); sys.exit(0)
if name == 'grep': sys.exit(1 if sys.argv[-1] == '.env.local' else 0)
# Stop at the first would-be install/build/download action; never perform it.
sys.exit(73)
'''
    for command in ('npm', 'curl', 'tar', 'mkdir', 'rm', 'mktemp', 'python3', 'uname', 'grep'):
        path = commands / command; path.write_text(mock); path.chmod(0o700)
    node = node_dir / 'bin/node'; node.write_text(mock); node.chmod(0o700)
    script = robot / 'setup.sh'
    # Isolate setup's absolute Node executable without changing HOME or the
    # production guard. Other potential side effects are PATH command stubs.
    script.write_text(SETUP.read_text().replace('NODE_DIR=$HOME/.local/node', 'NODE_DIR=$SETUP_FIXTURE_NODE'))
    key = tmp_path / 'never-read-or-written-key'
    result = subprocess.run(['/bin/bash', str(script)], env={**os.environ,
        'PATH': str(commands) + os.pathsep + os.environ['PATH'],
        'SETUP_CALLS': str(log), 'SETUP_FIXTURE_NODE': str(node_dir), 'GEMINI_KEY_FILE': str(key)},
        text=True, capture_output=True, timeout=5)
    calls = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    assert not key.exists()
    return result, calls, coordinator


@pytest.mark.parametrize('layout', ['missing', 'directory', 'nested', 'unreadable'])
def test_setup_refuses_incomplete_layout_before_install_or_key_handling(tmp_path, layout):
    if layout == 'unreadable' and os.geteuid() == 0:
        pytest.skip('root can read chmod-000 files; unreadability fixture requires ordinary user')
    result, calls, coordinator = run_setup(tmp_path, layout)
    assert result.returncode == 1
    assert calls == []
    assert str(coordinator) in result.stderr
    assert 'missing, unreadable, or not a file' in result.stderr
    assert 'foundation/ and gemini_pupper/ beside each other' in result.stderr
    assert '[1/4]' not in result.stdout
    assert 'done. Start with:' not in result.stdout


def test_setup_valid_sibling_layout_reaches_existing_build_step(tmp_path):
    result, calls, coordinator = run_setup(tmp_path, 'valid')
    assert result.returncode == 73  # Deliberate mock npm sentinel, not install failure.
    assert '[2/4] building the web app' in result.stdout
    assert not result.stderr
    assert calls[-1] == {'command': 'npm', 'args': ['ci', '--no-audit', '--no-fund', '--loglevel=error']}
    assert all(call['command'] in ('node', 'grep', 'npm') for call in calls)
    assert '[3/4]' not in result.stdout and '[4/4]' not in result.stdout
