"""Linux ownership receipts for start.sh/stop.sh; never launch or discover by name alone.

Uses the combined engineering checkout's shutdown coordinator, but NOT its
short-lived probe watchdog. A receipt is recovery evidence, not physical safety.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
import uuid

HERE = Path(__file__).resolve().parent
COORDINATOR = HERE.parents[1] / 'foundation/hardware/floor_stock/shutdown_coordinator.py'
SCHEMA = 'gemini_owned_processes_v1'
TAG = 'PUPPER_PROBE_SESSION_ID'
PROC = Path('/proc')
_COORDINATOR = None


def coordinator():
    global _COORDINATOR
    if _COORDINATOR is not None: return _COORDINATOR
    spec = importlib.util.spec_from_file_location('gemini_shutdown_coordinator', COORDINATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _COORDINATOR = module
    return module


def boot_id():
    return (PROC / 'sys/kernel/random/boot_id').read_text().strip()


def check_directory(directory):
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = directory.lstat()
    if directory.is_symlink() or not directory.is_dir() or info.st_uid != os.getuid() or info.st_mode & 0o022:
        raise ValueError('runtime directory must be owned by this user and not writable by others')


def listeners():
    result = subprocess.run(['ss', '-H', '-ltnp', 'sport = :8000'], check=True,
                            capture_output=True, text=True, timeout=3)
    return result.stdout.strip().splitlines()


def free_port():
    if listeners():
        raise ValueError('port 8000 is occupied; no startup or replacement attempted; use its existing owner')
    # Also reject a bound but not yet listening socket. This reservation is only
    # a precheck: server readiness and Stop independently verify the actual owner.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(('0.0.0.0', 8000))


def check_support(directory):
    check_directory(directory)
    coordinator()  # Must exist before anything can initialize hardware.
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise ValueError('Linux Python with pidfd signal support is required')
    descriptor = os.pidfd_open(os.getpid())
    os.close(descriptor)
    boot_id()


def load(directory):
    path = directory / 'owned_session.json'
    if path.is_symlink() or path.stat().st_uid != os.getuid() or path.stat().st_mode & 0o022:
        raise ValueError('untrusted ownership receipt')
    receipt = json.loads(path.read_text())
    if (receipt['schema'] != SCHEMA or receipt['uid'] != os.getuid() or
            receipt['root'] != str(HERE) or receipt['boot_id'] != boot_id() or
            not re.fullmatch('[0-9a-f]{32}', receipt['session_id'])):
        raise ValueError('foreign ownership receipt; no API request or process signal permitted')
    return receipt


def save(directory, receipt):
    coordinator().atomic_json(directory / 'owned_session.json', receipt)


def capture(pid, session, required_arg=None):
    target = coordinator().capture_process(pid, session, required_arg, PROC)
    target['argv'] = (PROC / str(pid) / 'cmdline').read_bytes().split(b'\0')[:-1]
    target['argv'] = [arg.decode('utf-8', 'surrogateescape') for arg in target['argv']]
    # The second identity check closes an exec/exit during the two reads.
    if state(target, session) != 'running':
        raise ProcessLookupError('process exited during capture')
    return target


def state(target, session):
    result = coordinator().process_state(target, session, PROC)
    if result == 'running':
        args = (PROC / str(target['pid']) / 'cmdline').read_bytes().split(b'\0')[:-1]
        if args != [arg.encode('utf-8', 'surrogateescape') for arg in target['argv']]:
            raise ValueError('captured process changed argv')
    return result


def send(target, session, sig):
    # pidfd pins the original task across the identity check and signal syscall.
    try:
        descriptor = os.pidfd_open(target['pid'])
    except ProcessLookupError:
        return False
    try:
        if state(target, session) != 'running':
            return False
        signal.pidfd_send_signal(descriptor, sig)
        return True
    finally:
        os.close(descriptor)


def begin(directory, kiosk):
    check_support(directory)
    path = directory / 'owned_session.json'
    if path.exists():
        old = load(directory)
        if old['status'] != 'stopped':
            raise ValueError('session/recovery receipt exists; run stop.sh and inspect its result before another start')
    free_port()
    receipt = dict(schema=SCHEMA, uid=os.getuid(), root=str(HERE), boot_id=boot_id(),
                   session_id=uuid.uuid4().hex, status='starting', ros='unknown', kiosk=kiosk,
                   gui_stopped=False, processes={}, pending={}, events=[])
    save(directory, receipt)
    return receipt['session_id']


def capture_spawn(pid, session, role, expected):
    target = capture(pid, session, expected)
    if str(Path(__file__).resolve()) in target['argv'] and '--exec' in target['argv']:
        raise ValueError('spawn gate has not executed command')
    # Required args also occur in bash's argv while it sources ROS. Do not turn
    # an arbitrary stability delay into evidence that final exec has happened.
    if role == 'chromium':
        valid_executable = Path(target['executable']).name in ('chromium', 'chromium-browser')
    else:
        valid_executable = target['executable'] == str(Path(sys.executable).resolve())
    if not valid_executable:
        raise ValueError('spawn has not reached the expected final executable')
    return target


def spawn(directory, role, expected, command):
    receipt = load(directory)
    require_starting(receipt)
    if role in receipt['processes'] or role in receipt['pending']:
        raise ValueError('role was already spawned; automatic restart is forbidden')
    if role == 'launch': (directory / 'controllers_ready').unlink(missing_ok=True)
    read_fd, write_fd = os.pipe()
    env = {**os.environ, TAG: receipt['session_id']}
    try:
        # The gate cannot exec until its PID identity has a durable receipt. If
        # this helper dies first, EOF makes the gate exit without running command.
        with (directory / ('robot.log' if role == 'launch' else role + '.log')).open('ab') as log:
            child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--exec', str(read_fd), *command],
                                     env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                     pass_fds=(read_fd,), start_new_session=True)
        os.close(read_fd); read_fd = -1
        gate = capture(child.pid, receipt['session_id'])
        receipt['pending'][role] = {'gate': gate, 'expected': expected, 'command': command}
        save(directory, receipt)
        os.write(write_fd, b'1')
    finally:
        if read_fd >= 0: os.close(read_fd)
        os.close(write_fd)
    until = time.monotonic() + 5
    while time.monotonic() < until:
        try:
            target = capture_spawn(child.pid, receipt['session_id'], role, expected)
            if target['start_ticks'] != gate['start_ticks']:
                raise ValueError('spawned PID was reused')
            if state(target, receipt['session_id']) == 'running':
                receipt['processes'][role] = target
                del receipt['pending'][role]
                save(directory, receipt)
                return
        except (OSError, ValueError):
            if child.poll() is not None:
                break
        time.sleep(.05)
    raise ValueError('spawn identity not confirmed; recovery receipt retained for ' + role)


def refresh(directory, receipt, *, children=False):
    """Capture only our uniquely tagged children, including ROS respawns/orphans."""
    session = receipt['session_id']
    for role, pending in list(receipt['pending'].items()):
        gate = pending['gate']
        try:
            target = capture_spawn(gate['pid'], session, role, pending['expected'])
        except (FileNotFoundError, ProcessLookupError):
            continue
        if target['start_ticks'] != gate['start_ticks']:
            continue
        receipt['processes'][role] = target
        del receipt['pending'][role]
    known = {(target['pid'], target['start_ticks']): role for role, target in receipt['processes'].items()}
    for entry in PROC.iterdir():
        if not entry.name.isdigit(): continue
        try:
            if entry.stat().st_uid != os.getuid(): continue
            # Global cmdline/stat are readable without ptrace access. Startup
            # needs only the manager, not every unrelated user's environment.
            args = (entry / 'cmdline').read_bytes().split(b'\0')
            manager_candidate = any(Path(os.fsdecode(arg)).name == 'ros2_control_node'
                                    for arg in args if arg)
            if not children and not manager_candidate: continue
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            identity = (int(entry.name), fields[19])
            environment = entry / 'environ'
            # Linux can expose a user-owned proc directory whose protected
            # environ belongs to root. It is not an inspectable owned child.
            # A recorded target or manager candidate must still prove its tag;
            # never hide its permission/identity failure or authorize a signal.
            if (environment.stat().st_uid != os.getuid() and
                    identity not in known and not manager_candidate): continue
            if (TAG + '=' + session).encode() not in environment.read_bytes().split(b'\0'): continue
            target = capture(int(entry.name), session)
        except (FileNotFoundError, ProcessLookupError):
            continue
        key = (target['pid'], target['start_ticks'])
        if key in known: continue
        if Path(target['executable']).name != 'ros2_control_node' and not children: continue
        role = 'manager' if Path(target['executable']).name == 'ros2_control_node' else 'child_' + entry.name + '_' + target['start_ticks']
        if role == 'manager' and receipt['ros'] != 'created':
            raise ValueError('unexpected tagged manager in borrowed/unknown ROS session')
        if role in receipt['processes']:
            raise ValueError('a different controller manager appeared; cleanup ownership is ambiguous')
        receipt['processes'][role] = target
    save(directory, receipt)
    return receipt


def api_owned(directory):
    receipt = load(directory)
    target = receipt['processes'].get('server')
    if target is None or state(target, receipt['session_id']) != 'running':
        raise ValueError('owned server is not running; API Stop/deactivation cannot be attributed')
    lines = listeners()
    if not lines or any(set(re.findall(r'pid=(\d+)', line)) != {str(target['pid'])} for line in lines):
        raise ValueError('port 8000 ownership does not match the recorded server; no HTTP request sent')


class Hooks:
    def __init__(self, directory, receipt):
        self.directory, self.receipt = directory, receipt
    def now(self): return time.monotonic()
    def sleep(self, seconds): time.sleep(seconds)
    def state(self, target): return state(target, self.receipt['session_id'])
    def send(self, target): return send(target, self.receipt['session_id'], signal.SIGINT)
    def requested(self): return True
    def event(self, name, **fields):
        row = {'event': name, 'monotonic': self.now(), **fields}
        self.receipt['events'].append(row)
        save(self.directory, self.receipt)
        return row


def cleanup(directory):
    receipt = load(directory)
    receipt['status'] = 'stopping'
    save(directory, receipt)
    session = receipt['session_id']
    targets = receipt['processes']
    # Stop ingress/UI processes even when ROS ownership is incomplete. They
    # cannot keep accepting commands while a manager recovery remains pending.
    for role in ('server', 'kiosk', 'chromium'):
        if role in targets: terminate(directory, receipt, role, targets[role])
    receipt = refresh(directory, receipt, children=True)
    if receipt['ros'] == 'created' and 'launch' in targets:
        if 'manager' not in targets:
            if state(targets['launch'], session) == 'running':
                raise ValueError('manager identity was never captured; launch left running for recovery, no launch-forwarded signal')
        else:
            hooks = Hooks(directory, receipt)
            spec = dict(manager=targets['manager'], launch=targets['launch'],
                        deadline_monotonic=hooks.now(), manager_exit_allowance_s=10., launch_exit_allowance_s=3.)
            result = coordinator().coordinate(spec, hooks, receipt['events'])
            receipt['shutdown'] = result
            save(directory, receipt)
            if not result['passed_cleanup']:
                raise ValueError(result.get('failure', 'ROS cleanup unconfirmed'))
    elif receipt['ros'] == 'unknown' and ('launch' in targets or 'launch' in receipt['pending']):
        raise ValueError('ROS ownership was not recorded; recovery required')
    receipt = refresh(directory, receipt, children=True)
    # Launch has exited before follower/browser leftovers are signaled. There is
    # deliberately no process-group signal or SIGKILL escalation.
    for role, target in receipt['processes'].items():
        if role in ('manager', 'launch'): continue
        terminate(directory, receipt, role, target)
    until = time.monotonic() + 5
    while time.monotonic() < until:
        if all(state(target, session) != 'running' for target in receipt['processes'].values()): break
        time.sleep(.05)
    receipt = refresh(directory, receipt, children=True)
    live = [role for role, target in receipt['processes'].items() if state(target, session) == 'running']
    pending = [role for role, item in receipt['pending'].items()
               if coordinator().process_state(item['gate'], session, PROC) == 'running']
    if live or pending:
        raise ValueError('owned processes still live/unverified; receipt retained: ' + ', '.join(live + pending))
    if receipt['gui_stopped']:
        subprocess.run(['sudo', '-n', 'systemctl', 'start', 'pupper-gui'], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
        receipt['gui_stopped'] = False
    receipt['status'] = 'stopped'
    save(directory, receipt)
    print('Owned process exit verified. Physical motor release is not verified.' +
          (' Borrowed ROS stack was left running.' if receipt['ros'] == 'borrowed' else ''))


def terminate(directory, receipt, role, target):
    event = 'term_' + role
    if not any(row['event'] == event for row in receipt['events']):
        if send(target, receipt['session_id'], signal.SIGTERM):
            receipt['events'].append({'event': event, 'monotonic': time.monotonic()})
            save(directory, receipt)


def require_starting(receipt):
    if receipt['status'] != 'starting':
        raise ValueError('startup canceled or already finished; no later startup action is allowed')


def operate(directory, action, arguments, expected_session):
    if action == 'begin': return begin(directory, arguments == ['--kiosk'])
    receipt = load(directory)
    if action == 'stopping':
        receipt['status'] = 'stopping'
        save(directory, receipt)
        return receipt['session_id']
    if expected_session != receipt['session_id']:
        raise ValueError('caller session does not match receipt; stale start/stop cannot affect a replacement')
    if action in ('ros', 'spawn', 'refresh', 'ready', 'stop-gui'): require_starting(receipt)
    if action == 'spawn': spawn(directory, arguments[0], arguments[1], arguments[2:])
    elif action == 'api-owned': api_owned(directory)
    elif action == 'cleanup': cleanup(directory)
    elif action == 'ros':
        if receipt['ros'] != 'unknown' or arguments not in (['created'], ['borrowed']):
            raise ValueError('invalid ROS ownership transition')
        receipt['ros'] = arguments[0]
        save(directory, receipt)
    elif action == 'refresh': refresh(directory, receipt)
    elif action == 'ready':
        if receipt['pending'] or (receipt['ros'] == 'created' and 'manager' not in receipt['processes']):
            raise ValueError('startup process ownership is incomplete')
        receipt['status'] = 'running'
        save(directory, receipt)
    elif action == 'stop-gui':
        active = subprocess.run(['systemctl', 'is-active', '--quiet', 'pupper-gui'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
        if active.returncode == 0:
            # Prior active state permits restoration even if stop times out.
            receipt['gui_stopped'] = True
            save(directory, receipt)
            subprocess.run(['sudo', '-n', 'systemctl', 'stop', 'pupper-gui'], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
        for command in (['amixer', '-q', 'sset', 'Master', '100%'], ['wpctl', 'set-volume', '@DEFAULT_SINK@', '1.0']):
            try:
                subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=.5)
            except (OSError, subprocess.SubprocessError): pass  # Preserve best-effort kiosk audio setup.



def main():
    # Internal gate is intentionally not a detached supervisor or watchdog.
    if len(sys.argv) > 1 and sys.argv[1] == '--exec':
        descriptor = int(sys.argv[2])
        release = os.read(descriptor, 1)
        os.close(descriptor)
        if release != b'1': return 1
        os.execvpe(sys.argv[3], sys.argv[3:], os.environ)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=Path('/tmp/gemini-pupper'))
    parser.add_argument('action', choices=['check', 'begin', 'ros', 'spawn', 'refresh', 'api-owned', 'stopping', 'ready', 'stop-gui', 'cleanup'])
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(); directory = args.directory
    if args.action == 'check':
        check_support(directory)
        return 0
    check_directory(directory)
    # Serialize only receipt/spawn/cleanup operations, never the shell's ROS
    # readiness waits. Stop can cancel an in-progress start at the next action.
    with coordinator().execution_lock(directory, {'manager_exit_allowance_s': 5., 'launch_exit_allowance_s': 1.}):
        result = operate(directory, args.action, args.arguments, os.environ.get('GEMINI_PUPPER_SESSION'))
    if result is not None: print(result)
    return 0


if __name__ == '__main__':
    os.umask(0o077)
    try: raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        print('Lifecycle refused: ' + str(error) + '. Receipt retained; no automatic restart.', file=sys.stderr)
        raise SystemExit(1)
