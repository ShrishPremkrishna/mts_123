"""Single shutdown owner for an explicitly approved, already identified test stack.

No ROS, hardware, or controller API is imported. The manager signal deadline is
an OS-scheduled software bound, not a physical power cutoff. Cleanup allowances
are policy, not guaranteed bounds on the driver's blocking I/O or destruction.
"""
from __future__ import annotations
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

PROTOCOL = 'pupper_console_shutdown_v2'
POLL_S = .01


def atomic_json(path, value):
    path = Path(path)
    text = json.dumps(value, allow_nan=False, sort_keys=True) + '\n'
    temporary = path.with_name(path.name + '.' + str(os.getpid()) + '.tmp')
    with temporary.open('x') as stream:
        stream.write(text); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)


def capture_process(pid, session_id, required_arg=None, proc_root=Path('/proc')):
    proc = proc_root / str(pid)
    fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
    args = (proc / 'cmdline').read_bytes().split(b'\0')
    if fields[0] == 'Z':
        raise ProcessLookupError('process already exited')
    if ('PUPPER_PROBE_SESSION_ID=' + session_id).encode() not in (proc / 'environ').read_bytes().split(b'\0'):
        raise ValueError('process session mismatch')
    if required_arg is not None and str(required_arg).encode() not in args:
        raise ValueError('required command identity missing')
    return {'pid': int(pid), 'start_ticks': fields[19],
            'executable': str((proc / 'exe').resolve(strict=True)),
            'required_arg': str(required_arg) if required_arg is not None else None}


def process_state(target, session_id, proc_root=Path('/proc')):
    """Only the captured original is authoritative; reused IDs are never signaled."""
    proc = proc_root / str(target['pid'])
    try:
        fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
    except FileNotFoundError:
        return 'exited'
    if fields[19] != target['start_ticks']:
        return 'reused'
    if fields[0] == 'Z':
        return 'exited'
    try:
        current = capture_process(target['pid'], session_id, target.get('required_arg'), proc_root)
    except (FileNotFoundError, ProcessLookupError):
        return 'exited'
    except PermissionError:
        # /proc metadata can become unreadable between the first stat read and
        # process exit. Prove disappearance/zombie/reuse from a fresh stat;
        # denied identity access on a still-live original remains an error.
        try:
            fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        except FileNotFoundError:
            return 'exited'
        if fields[19] != target['start_ticks']: return 'reused'
        if fields[0] == 'Z': return 'exited'
        raise
    if current['start_ticks'] != target['start_ticks']: return 'reused'
    if current['executable'] != target['executable']:
        raise ValueError('captured process changed executable')
    return 'running'


def read_events(directory):
    path = Path(directory) / 'events.jsonl'
    if not path.exists():
        return []
    result = []
    for line in path.read_text().splitlines():
        try: result.append(json.loads(line))
        except json.JSONDecodeError: break  # killed during final append; no acknowledgment inferred
    return result


class SystemHooks:
    def __init__(self, directory, spec):
        self.directory = Path(directory); self.spec = spec
    def now(self): return time.monotonic()
    def sleep(self, duration): time.sleep(duration)
    def state(self, target): return process_state(target, self.spec['session_id'])
    def send(self, target):
        # Revalidate immediately before the syscall. A reused PID is never a target.
        if self.state(target) != 'running': return False
        os.kill(target['pid'], signal.SIGINT)
        return True
    def requested(self): return (self.directory / 'request.json').exists()
    def event(self, name, **fields):
        row = {'event': name, 'monotonic': self.now(), **fields}
        with (self.directory / 'events.jsonl').open('a') as stream:
            stream.write(json.dumps(row, allow_nan=False) + '\n')
            stream.flush(); os.fsync(stream.fileno())
        return row


def coordinate(spec, hooks, previous=(), fallback=False):
    """One owner sends manager first; launch may be signaled only after manager exit."""
    previous = list(previous)
    sent = {row['event']: row for row in previous if row['event'].endswith('_sigint_sent')}
    result = {'protocol': PROTOCOL, 'fallback': fallback, 'passed_cleanup': False,
              'physical_stopping_proven': False, 'manager_deadline_monotonic': spec['deadline_monotonic']}
    if not fallback:
        while hooks.now() < spec['deadline_monotonic']:
            if hooks.requested(): break
            try:
                parent_state=hooks.state(spec['probe_parent'])
            except Exception as error:
                # Losing the monitored probe is a reason to stop the verified
                # manager, not a reason for its independent owner to exit.
                result['probe_monitor_error']=repr(error)
                result['early_shutdown_reason']='probe_identity_unavailable'
                break
            if parent_state != 'running':
                result['early_shutdown_reason']='probe_no_longer_running'
                break
            hooks.sleep(min(POLL_S, max(0., spec['deadline_monotonic'] - hooks.now())))
    terminal = {'exited', 'reused'}
    def inspect(target, label):
        try:
            state = hooks.state(target)
            if state not in terminal | {'running'}:
                raise ValueError('unexpected process state: ' + repr(state))
            return state
        except Exception as error:
            # Retain uncertainty without killing the only shutdown owner. No
            # disk I/O precedes the deadline-critical manager signal. Coalesce
            # repeats so bounded polling cannot create unbounded diagnostics.
            errors = result.setdefault('identity_inspection_errors', {})
            item = errors.setdefault(label, {'pid': target['pid'], 'count': 0,
                                             'first_error': repr(error)})
            item.update(count=item['count'] + 1, last_error=repr(error))
            return 'unverified'

    def send(target, label):
        try:
            return hooks.send(target)  # revalidates identity at the syscall
        except Exception as error:
            result.setdefault('signal_errors', {})[label] = repr(error)
            return False

    manager = spec['manager']
    manager_state = inspect(manager, 'manager')
    if manager_state == 'running' and 'manager_sigint_sent' not in sent:
        # No disk write is allowed ahead of the deadline-critical signal.
        # Only a record written AFTER the syscall acknowledges its delivery.
        actually_sent = send(manager, 'manager')
        if actually_sent:
            sent['manager_sigint_sent'] = hooks.event('manager_sigint_sent')
    hooks.event('shutdown_triggered', fallback=fallback)
    if fallback and 'manager_sigint_sent' not in {r['event'] for r in previous}:
        result['prior_signal_delivery_ambiguous'] = True
    signal_time = sent.get('manager_sigint_sent', {}).get('monotonic', hooks.now())
    result['manager_signal_monotonic'] = sent.get('manager_sigint_sent', {}).get('monotonic')
    limit = signal_time + spec['manager_exit_allowance_s']
    manager_state = inspect(manager, 'manager')
    while manager_state not in terminal and hooks.now() < limit:
        hooks.sleep(min(POLL_S, max(0., limit - hooks.now())))
        manager_state = inspect(manager, 'manager')
    result['manager_state'] = manager_state
    if manager_state not in terminal:
        result['failure'] = 'manager exit unconfirmed after cleanup allowance; no repeated signal or launch-forwarding'
        hooks.event('manager_exit_allowance_exhausted', state=manager_state)
        return result
    exit_record = hooks.event('original_manager_exited', state=manager_state)
    prior_exit = next((r for r in previous if r['event'] == 'original_manager_exited'), exit_record)
    launch = spec['launch']
    # Natural exit and identity-inspection retries share the original allowance.
    # The launch is signaled only after proven manager exit and fresh identity
    # validation. An unknown state is never success or permission to signal.
    limit = prior_exit['monotonic'] + spec['launch_exit_allowance_s']
    natural_until = min(limit, prior_exit['monotonic'] + .5)
    launch_attempted = 'launch_sigint_sent' in sent
    launch_state = inspect(launch, 'launch')
    while launch_state not in terminal and hooks.now() < limit:
        if launch_state == 'running' and not launch_attempted and hooks.now() >= natural_until:
            launch_attempted = True
            hooks.event('launch_signal_attempt')
            if send(launch, 'launch'):
                sent['launch_sigint_sent'] = hooks.event('launch_sigint_sent')
        hooks.sleep(min(POLL_S, max(0., limit - hooks.now())))
        launch_state = inspect(launch, 'launch')
    result['launch_state'] = launch_state
    result['passed_cleanup'] = launch_state in terminal
    if not result['passed_cleanup']: result['failure'] = 'launch exit unconfirmed after cleanup allowance'
    hooks.event('shutdown_finished', passed_cleanup=result['passed_cleanup'])
    return result


def load_handle(directory):
    directory = Path(directory)
    spec_bytes = (directory / 'targets.json').read_bytes()
    spec = json.loads(spec_bytes)
    if spec['protocol'] != PROTOCOL: raise ValueError('shutdown protocol mismatch')
    return {'directory': str(directory), 'spec_sha256': hashlib.sha256(spec_bytes).hexdigest(), 'spec': spec}


def check_handle(handle):
    actual = load_handle(handle['directory'])
    if actual['spec_sha256'] != handle['spec_sha256']: raise ValueError('shutdown targets changed')
    for key in ('spawned_pid', 'spawned_identity'):
        if key in handle: actual[key] = handle[key]
    provisional = Path(actual['directory']) / 'spawned-owner.json'
    if provisional.exists():
        receipt = json.loads(provisional.read_text())
        if receipt['spec_sha256'] != actual['spec_sha256']: raise ValueError('spawned owner targets mismatch')
        actual['spawned_identity'] = receipt['identity']
        actual['spawned_pid'] = receipt['identity']['pid']
    return actual


class ArmError(RuntimeError):
    def __init__(self, message, handle): super().__init__(message); self.handle = handle


def arm(directory, *, manager, launch, session_id, deadline_monotonic=None,
        manager_exit_allowance_s=10., launch_exit_allowance_s=3., mode='partial-init'):
    """Call immediately before release; caller must retain ArmError.handle too."""
    if mode not in ('partial-init','full-stock-zero','floor-stock-zero'):raise ValueError('unknown supervised mode')
    cap={'partial-init':1.5,'full-stock-zero':6.0,'floor-stock-zero':20.0}[mode]
    directory = Path(directory).resolve(); directory.mkdir(mode=0o700)
    now = time.monotonic()
    deadline = now + cap if deadline_monotonic is None else deadline_monotonic
    if not now < deadline <= now + cap: raise ValueError('deadline exceeds explicit supervised-mode cap')
    parent = capture_process(os.getpid(), session_id)
    spec = {'protocol': PROTOCOL, 'session_id': session_id, 'supervised_mode':mode, 'manager': manager,
            'launch': launch, 'probe_parent': parent, 'deadline_monotonic': deadline,
            'manager_exit_allowance_s': manager_exit_allowance_s,
            'launch_exit_allowance_s': launch_exit_allowance_s}
    for target in (manager, launch):
        if process_state(target, session_id) != 'running': raise ValueError('original target is not running')
    atomic_json(directory / 'targets.json', spec)
    handle = load_handle(directory)
    try:
        with (directory/'owner-process.log').open('xb') as log:
            child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker', str(directory)],
                                     start_new_session=True, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT)
        handle['spawned_pid'] = child.pid
        handle['spawned_identity'] = capture_process(child.pid, session_id, str(Path(__file__).resolve()))
        atomic_json(directory / 'spawned-owner.json', {'spec_sha256': handle['spec_sha256'], 'identity': handle['spawned_identity']})
        until = min(now + .5, deadline)
        while time.monotonic() < until:
            if (directory / 'owner.json').exists():
                owner = json.loads((directory / 'owner.json').read_text())
                if owner['spec_sha256'] != handle['spec_sha256'] or owner['identity']['pid'] != child.pid:
                    raise ValueError('coordinator readiness identity mismatch')
                if process_state(owner['identity'], session_id) != 'running':
                    raise ValueError('coordinator exited before readiness')
                return handle
            if child.poll() is not None: raise ValueError('coordinator exited before readiness')
            time.sleep(POLL_S)
        raise ValueError('coordinator readiness timeout; no release permitted')
    except BaseException as error:
        raise ArmError(str(error), handle) from error


def request(handle, reason):
    handle = check_handle(handle)
    path = Path(handle['directory']) / 'request.json'
    if not path.exists(): atomic_json(path, {'reason': reason, 'monotonic': time.monotonic()})


def nominal_cleanup(result):
    return (result.get('passed_cleanup') is True and result.get('fallback') is False
            and not result.get('prior_signal_delivery_ambiguous') and not result.get('probe_monitor_error')
            and not result.get('identity_inspection_errors') and not result.get('signal_errors'))


def request_and_wait(handle, reason):
    """Normal callers defer to the owner. Dead-owner fallback never trusts intent."""
    handle = check_handle(handle); directory = Path(handle['directory']); spec = handle['spec']
    request(handle, reason)
    hooks = SystemHooks(directory, spec)
    limit = spec['deadline_monotonic'] + spec['manager_exit_allowance_s'] + spec['launch_exit_allowance_s'] + .5
    while time.monotonic() < limit:
        result_path = directory / 'result.json'
        if result_path.exists(): return json.loads(result_path.read_text())
        owner_path = directory / 'owner.json'
        if owner_path.exists():
            owner = json.loads(owner_path.read_text())
            if owner['spec_sha256'] != handle['spec_sha256']: raise ValueError('owner targets mismatch')
            state = process_state(owner['identity'], spec['session_id'])
            if state != 'running':
                fallback_reason='owner_'+state
                break
        elif time.monotonic() >= spec['deadline_monotonic']:
            fallback_reason='owner_receipt_missing_at_deadline'
            break
        time.sleep(POLL_S)
    else:
        return {'protocol': PROTOCOL, 'passed_cleanup': False, 'physical_stopping_proven': False,
                'failure': 'coordinator alive/unresponsive beyond its bound; no uncoordinated repeated signals'}
    # The worker AND fallback callers hold the same lock. Missing readiness
    # never permits two coordinators, including a delayed original worker.
    try:
        with execution_lock(directory, spec):
            if (directory / 'result.json').exists(): return json.loads((directory / 'result.json').read_text())
            result = coordinate(spec, hooks, read_events(directory), fallback=True)
            result['fallback_reason']=fallback_reason
            result['owner_failures']=[r for r in read_events(directory) if r.get('event')=='owner_failed']
            atomic_json(directory / 'result.json', result)
            return result
    except TimeoutError:
        return {'protocol': PROTOCOL, 'passed_cleanup': False, 'physical_stopping_proven': False,
                'failure': 'another shutdown owner did not complete within its allowance'}


@contextmanager
def execution_lock(directory, spec):
    import fcntl
    with (Path(directory) / 'execution.lock').open('a') as lock:
        limit = time.monotonic() + spec['manager_exit_allowance_s'] + spec['launch_exit_allowance_s'] + .5
        while True:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB); break
            except BlockingIOError:
                if time.monotonic() >= limit: raise TimeoutError('shutdown owner lock timeout')
                time.sleep(POLL_S)
        try: yield
        finally: fcntl.flock(lock, fcntl.LOCK_UN)


def worker(directory):
    handle = load_handle(directory); spec = handle['spec']; directory = Path(directory)
    hooks = SystemHooks(directory, spec)
    try:
        with execution_lock(directory, spec):
            # A fallback may have completed before this delayed worker began.
            if (directory / 'result.json').exists():
                result = json.loads((directory / 'result.json').read_text())
            else:
                owner = {'protocol': PROTOCOL, 'spec_sha256': handle['spec_sha256'],
                         'identity': capture_process(os.getpid(), spec['session_id'])}
                atomic_json(directory / 'owner.json', owner)
                result = coordinate(spec, hooks)
                atomic_json(directory / 'result.json', result)
    except BaseException as error:
        # Absence of a success receipt is intentional: request_and_wait must
        # recover after actual worker exit rather than accept unverified cleanup.
        try: hooks.event('owner_failed', error=repr(error), traceback=traceback.format_exc(limit=8))
        except Exception: pass
        return 1
    return 0 if result['passed_cleanup'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--worker', type=Path, required=True)
    raise SystemExit(worker(parser.parse_args().worker))
