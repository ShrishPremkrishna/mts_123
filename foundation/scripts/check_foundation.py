#!/usr/bin/env python3
"""Run existing offline checks with separate API and numerical Python environments.

No training, network, browser or robot operations. Known checkout environments
are discovered and import-checked, or pass --api-python and --numerical-python.
Use --output for a new writable evidence directory; the default falls back to a
printed temporary directory if checkout artifacts are not writable. No installs.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

API_ENV = '.venv-api/bin/python'
LEGACY_API_ENV = 'artifacts/hardware/2026-10-29/server-transport-review/venv/bin/python'
REQUIRED = {
    'api': ('pytest', 'numpy', 'yaml', 'fastapi', 'httpx', 'google.genai'),
    'numerical': ('pytest', 'numpy', 'yaml', 'scipy', 'torch', 'mujoco', 'warp', 'sklearn', 'mcap_ros2', 'mjlab'),
}
NUMERICAL_TESTS = [
    'test_pupper_foundation.py', 'test_pupper_tracking.py', 'test_pupper_spider.py',
    'test_pupper_unimate.py', 'test_unimate_repair.py', 'test_unimate_worker.py',
    'test_unimate_candidates.py', 'test_foundation_evaluation.py', 'test_pupper_export.py',
    'test_pupper_gait_export.py', 'test_pupper_latency.py', 'test_pupper_bootstrap.py',
    'test_pupper_encoder_observation.py', 'test_deployment_target_clamp.py',
    'test_pupper_randomization_contract.py', 'test_foundation_training_contract.py',
    'test_tracking_ground_hulls.py', 'test_locomotion_torso_contact.py',
    'test_pupper_collision_audit.py', 'test_pupper_policy_audit.py',
    'test_driver_target_clamp.py', 'test_position_target_slew.py', 'test_slew_checkpoint_contract.py',
]
WATCHED = ('foundation/', 'forward_kinematics_lab/', 'pd_control_lab/',
           'ik_heuristic_walking_lab/', 'pupper-gait-tuner/', 'follow_me_pupper/',
           'gemini_pupper/', 'implicit_compliance/', 'pupper_gait_deploy/',
           'pupper-mjlab/src/', 'pupper-mjlab/tests/', 'pupper-mjlab/configs/')
PROBE = '''import importlib,json,sys
result={'executable':sys.executable,'prefix':sys.prefix,'version':sys.version,'modules':{},'module_files':{},'errors':{}}
for name in sys.argv[1:]:
    try:
        module=importlib.import_module(name)
        result['modules'][name]=str(getattr(module,'__version__','imported'))
        result['module_files'][name]=getattr(module,'__file__',None)
    except Exception as exc:
        result['errors'][name]=type(exc).__name__+': '+str(exc)
print('FOUNDATION_PREFLIGHT='+json.dumps(result))
sys.exit(bool(result['errors']))
'''


def positive_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('must be a positive finite number of seconds')
    return value


def _new_output(path):
    path.mkdir(parents=True, exist_ok=False)
    # Test actual writes, not mode bits/access(), which can be misleading.
    probe = path / '.write-probe'
    probe.write_text('writable\n')
    probe.unlink()
    return path


def prepare_output(root, requested, stamp):
    desired = (requested or root / 'artifacts/validation' / stamp).expanduser().absolute()
    try:
        return _new_output(desired), None
    except OSError as exc:
        if requested is not None:
            raise RuntimeError(f'Cannot create new output directory {desired}: {exc}. '
                               'Choose a new writable --output path.') from exc
        fallback = Path(tempfile.mkdtemp(prefix='pupper-foundation-check-'))
        return fallback, f'Default output {desired} unavailable: {exc}'


def check_environment(output):
    env = os.environ.copy()
    env.update(FORCE_CPU='1', CUDA_VISIBLE_DEVICES='', PYTHONDONTWRITEBYTECODE='1')
    for key in ('PUPPER_REFERENCE_DIR', 'PYTHONPATH', 'PYTHONHOME', 'PYTEST_ADDOPTS', 'PYTEST_PLUGINS',
                'GEMINI_API_KEY', 'GOOGLE_API_KEY', 'OPENAI_API_KEY', 'ANTHROPIC_API_KEY'):
        env.pop(key, None)
    env['PYTHONNOUSERSITE'] = '1'
    # Do not inherit an unwritable/global cache location for this validation run.
    for key, relative in [('MPLCONFIGDIR', 'matplotlib'), ('WARP_CACHE_PATH', 'warp'),
                          ('XDG_CACHE_HOME', 'general')]:
        env[key] = str(output / 'cache' / relative)
    return env


def terminate_tree(process):
    """Only the new child process group belongs to this runner."""
    if os.name == 'posix':
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    elif process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass
    # A parent can exit on TERM while a spawned child ignores it.
    if os.name == 'posix':
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    elif process.poll() is None:
        process.kill()
    process.wait(timeout=2)


def execute(command, root, env, log, timeout):
    start = time.monotonic()
    row = {'command': command, 'log': str(log), 'timeout_s': timeout, 'timed_out': False}
    process = None
    try:
        with log.open('w') as stream:
            process = subprocess.Popen(command, cwd=root, env=env, stdout=stream,
                                       stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                       start_new_session=(os.name == 'posix'))
            try:
                row['returncode'] = process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                terminate_tree(process)
                row.update(returncode=124, timed_out=True, error=f'Exceeded {timeout:g}s; termination sent to child process group')
            except KeyboardInterrupt:
                terminate_tree(process)
                raise
    except OSError as exc:
        row.update(returncode=127, error=f'{type(exc).__name__}: {exc}')
    row['seconds'] = time.monotonic() - start
    return row


def interpreter_candidates(root, kind, explicit):
    if explicit:
        located = shutil.which(str(explicit)) if os.sep not in str(explicit) else str(explicit)
        return [Path(located or explicit).expanduser().absolute()]
    known = [root / API_ENV, root / LEGACY_API_ENV, root / '.venv/bin/python'] if kind == 'api' else [root / '.venv/bin/python']
    # Never resolve interpreter symlinks: doing so can silently leave its venv.
    return list(dict.fromkeys([p.absolute() for p in known] + [Path(sys.executable).absolute()]))


def select_interpreter(root, kind, explicit, output, env, timeout):
    attempts = []
    for index, path in enumerate(interpreter_candidates(root, kind, explicit)):
        if not path.is_file() or not os.access(path, os.X_OK):
            attempts.append({'executable': str(path), 'error': 'Not an executable file'})
            continue
        log = output / f'preflight-{kind}-{index}.log'
        attempt = execute([str(path), '-c', PROBE, *REQUIRED[kind]], root, env, log, timeout)
        attempt['executable'] = str(path)
        for line in reversed(log.read_text(errors='replace').splitlines() if log.exists() else []):
            if line.startswith('FOUNDATION_PREFLIGHT='):
                try:
                    attempt['runtime'] = json.loads(line.split('=', 1)[1])
                except json.JSONDecodeError:
                    pass
                break
        attempts.append(attempt)
        runtime = attempt.get('runtime', {})
        if kind == 'numerical' and runtime.get('errors') == {}:
            source = runtime['module_files'].get('mjlab')
            expected = (root / 'pupper-mjlab/src/mjlab/__init__.py').resolve()
            if not source or Path(source).resolve() != expected:
                attempt['error'] = f'mjlab imported from {source}. Expected {expected}.'
                continue
        if attempt['returncode'] == 0 and runtime.get('errors') == {}:
            return str(path), attempts
    return None, attempts


def build_checks(api_python, numerical_python):
    def pytest(python, *paths):
        return [python, '-m', 'pytest', *paths, '-q', '-p', 'no:cacheprovider']
    return [
        ('classical', pytest(numerical_python, 'foundation/tests')),
        ('floor_input', pytest(api_python, 'foundation/hardware/floor_stock')),
        ('tool_queue', pytest(api_python, 'gemini_pupper/tests')),
        ('compliance', pytest(numerical_python, 'implicit_compliance/tests')),
        ('course_follow', [numerical_python, 'follow_me_pupper/lab/test.py']),
        ('course_color', [numerical_python, 'follow_me_pupper/lab/test.py', 'step5']),
        ('course_avoidance', [numerical_python, 'follow_me_pupper/lab/test.py', 'step6']),
        ('course_tools', [api_python, 'gemini_pupper/lab/test.py']),
        ('course_jump', [api_python, 'gemini_pupper/lab/test.py', 'optional_jump']),
        ('motion_export', pytest(numerical_python, *[f'pupper-mjlab/tests/{name}' for name in NUMERICAL_TESTS])),
        ('controller_cpp', ['bash', 'pupper_gait_deploy/test/run_host_test.sh']),
    ]


def watched_files(root):
    candidates = subprocess.run(['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'],
                                cwd=root, capture_output=True, check=True, timeout=15).stdout.decode().split('\0')
    driver_sources = json.loads((root / 'foundation/configs/class-driver-joint-limits.json').read_text())['sources']
    return sorted({name for name in candidates if name.startswith(WATCHED) and (root / name).is_file()
                   and Path(name).suffix in ('.py', '.cpp', '.hpp', '.h', '.sh', '.json', '.yaml', '.xml', '.npz')}
                  | {str(path.relative_to(root)) for path in (root / 'foundation').rglob('*.py')}
                  | {item['path'] for item in driver_sources})


def source_hashes(root):
    hashes = {}
    for name in watched_files(root):
        path = root / name
        if not path.resolve().is_relative_to(root.resolve()):
            raise ValueError(f'Watched source escapes checkout: {name}')
        try:
            hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        except FileNotFoundError:
            hashes[name] = None
    return hashes


def write_report(destination, report):
    temporary = destination.with_name(destination.name + '.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(destination)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).absolute().parents[2],
                        help='checkout to validate (default: directory containing this runner)')
    parser.add_argument('--output', type=Path, help='new evidence directory; explicit paths never silently fall back')
    parser.add_argument('--api-python', help='API/test interpreter; default: known checkout environment, then current Python')
    parser.add_argument('--numerical-python', help='MuJoCo/PyTorch interpreter; default: checkout .venv, then current Python')
    parser.add_argument('--check-timeout', type=positive_seconds, default=600., help='seconds per check (default600)')
    parser.add_argument('--preflight-timeout', type=positive_seconds, default=45., help='seconds per interpreter probe (default45)')
    args = parser.parse_args(argv)
    root = args.repo.expanduser().resolve()
    if not (root / 'foundation/configs/class-driver-joint-limits.json').is_file():
        parser.error(f'Not a foundation checkout: {root}')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    try:
        output, fallback = prepare_output(root, args.output, stamp)
    except (OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if fallback:
        print(f'{fallback}\nUsing writable temporary output: {output}', file=sys.stderr, flush=True)
    env = check_environment(output)
    destination = output / 'checks.json'
    report = {'created_utc': stamp, 'repo': str(root), 'output': str(output), 'output_fallback': fallback,
              'runner': {'path': str(Path(__file__).absolute()), 'sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                         'python': sys.executable, 'version': sys.version},
              'scope': 'offline regression and numerical contract checks; no hardware or trained-policy performance claim',
              'interpreters': {}, 'preflight': {}, 'checks': [], 'passed': False, 'status': 'preflight'}
    def save():
        write_report(destination, report)
    previous_signals = {}
    def interrupted(signum, _frame):
        report['interrupted_signal'] = signal.Signals(signum).name
        for watched_signal in previous_signals:
            signal.signal(watched_signal, signal.SIG_IGN)
        raise KeyboardInterrupt
    try:
        save()
        for signum in (signal.SIGINT, signal.SIGTERM, getattr(signal, 'SIGHUP', signal.SIGTERM)):
            if signum not in previous_signals:
                previous_signals[signum] = signal.signal(signum, interrupted)
        revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root, text=True, capture_output=True, check=True, timeout=15)
        report['git_revision'] = revision.stdout.strip()
        hashes = source_hashes(root)
        report['source_sha256'] = hashes
        missing_sources = [name for name, digest in hashes.items() if digest is None]
        if missing_sources:
            raise FileNotFoundError('Missing watched sources: ' + ', '.join(missing_sources))
        for kind, explicit in [('api', args.api_python), ('numerical', args.numerical_python)]:
            python, attempts = select_interpreter(root, kind, explicit, output, env, args.preflight_timeout)
            report['preflight'][kind] = attempts
            report['interpreters'][kind] = python
            save()
        missing = [kind for kind, path in report['interpreters'].items() if path is None]
        if missing:
            report.update(status='preflight_failed', error='No usable ' + ', '.join(missing) +
                          ' interpreter. Inspect preflight logs and pass --api-python/--numerical-python; no dependencies were installed.')
        else:
            report['status'] = 'running'
            for name, command in build_checks(report['interpreters']['api'], report['interpreters']['numerical']):
                print(f'Running {name}; log: {output / (name + ".log")}', flush=True)
                row = execute(command, root, env, output / f'{name}.log', args.check_timeout)
                row['name'] = name
                report['checks'].append(row)
                save()
                print(f'{name}: {"PASS" if row["returncode"] == 0 else "FAIL"}', flush=True)
            after = source_hashes(root)
            report['changed_source_during_checks'] = [n for n, h in hashes.items() if after.get(n) != h]
            report['added_source_during_checks'] = sorted(set(after) - set(hashes))
            report['passed'] = (all(c['returncode'] == 0 for c in report['checks'])
                                and not report['changed_source_during_checks'] and not report['added_source_during_checks'])
            report['status'] = 'passed' if report['passed'] else 'failed'
    except KeyboardInterrupt:
        report.update(status='interrupted', error='Interrupted; active check child process group stopped')
    except Exception as exc:
        detail = getattr(exc, 'stderr', None) or ''
        if isinstance(detail, bytes):
            detail = detail.decode(errors='replace')
        report.update(status='runner_error', error=f'{type(exc).__name__}: {exc}' + (f': {detail.strip()}' if detail else ''))
    finally:
        for signum, handler in previous_signals.items():
            signal.signal(signum, handler)
    try:
        save()
    except OSError as exc:
        print(f'Cannot save final receipt {destination}: {exc}', file=sys.stderr)
        return 2
    if report.get('error'):
        print(report['error'], file=sys.stderr)
    print(destination, flush=True)
    return 0 if report['passed'] else 130 if report['status'] == 'interrupted' else 2 if report['status'] in ('preflight_failed', 'runner_error') else 1


if __name__ == '__main__':
    raise SystemExit(main())
