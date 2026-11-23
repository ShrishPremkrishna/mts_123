#!/usr/bin/env python3
"""Explicit selected Lab 6 operations. Only start-native can initialize/home hardware.

Reuses the Gemini ownership receipt and manager-first shutdown implementation;
no supervisor, automatic restart, shell environment installation or watchdog.
"""
from contextlib import contextmanager
from copy import deepcopy
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
import urllib.request
import uuid

from stage_app import sha, verify_selected, checked_path, entries_by_path

ENV_KEYS = ('PATH', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
            'LD_LIBRARY_PATH', 'ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
            'PUPPER_MONOREPO_DIR', 'PUPPER_GAIT_DEPLOY_DIR', 'FOLLOW_ME_DIR',
            'PUPPER_ONBOARD_BOOTSTRAP_DIR', 'GEMINI_KEY_FILE',
            'PUPPER_FOLLOWER_MODE', 'PUPPER_SESSION_MOTION_MODE', 'FOLLOW_USE_COLOR', 'FOLLOW_AVOID_OBSTACLES', 'FOLLOW_ALWAYS_DETECT')


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def write(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)


def pinned(path, digest=None):
    path = path.resolve(strict=True)
    actual = sha(path)
    if digest is not None and (not re.fullmatch('[0-9a-f]{64}', digest) or digest != actual):
        raise ValueError('Component identity mismatch: ' + str(path))
    return {'path': str(path), 'sha256': actual}


def interpreter_identity():
    # Resolving the executable alone discards which virtualenv supplies imports.
    return {'executable': os.path.abspath(sys.executable), 'prefix': os.path.abspath(sys.prefix)}


def check_interpreter(plan):
    if interpreter_identity() != plan['interpreter']:
        raise ValueError('Use the same selected Python executable path and virtualenv prefix used to prepare')


def prepare(args):
    # This operation imports profile generation only; no ROS process is launched.
    manifest = pinned(args.app_manifest, args.app_manifest_sha256)
    app = json.loads(args.app_manifest.read_text())
    root = Path(app['destination_remote_path']).resolve(strict=True)
    verify_selected(root, app['final_selected_files'])
    if 'PUPPER_SESSION_SKILLS' not in os.environ:
        raise ValueError('Set explicit PUPPER_SESSION_SKILLS (empty means baseline)')
    if os.environ.get('ROS_DOMAIN_ID') != '168':
        raise ValueError('Source the selected ROS environment with ROS_DOMAIN_ID=168 before preparation')
    if not 1 <= args.port <= 65535:
        raise ValueError('Invalid API port')
    launch_root = args.launch.resolve().parent
    if (args.config.resolve() != (launch_root / 'lab6-session.yaml').resolve() or
            args.urdf.resolve() != (launch_root / 'floor_stock/baseline-expanded.urdf').resolve()):
        raise ValueError('Preparation config/URDF must match the selected launch sibling inputs')
    components = {'native': pinned(args.native_library, args.native_sha256),
                  'imu': pinned(args.imu_library, args.imu_sha256),
                  'launch': pinned(args.launch), 'config': pinned(args.config),
                  'urdf': pinned(args.urdf), 'app_manifest': manifest,
                  'python': pinned(Path(sys.executable)),
                  'helper': pinned(Path(__file__)),
                  'lifecycle': pinned(root / 'gemini_pupper/robot/session_processes.py'),
                  'telemetry_topics': pinned(args.telemetry_topics)}
    topics = json.loads(args.telemetry_topics.read_text())
    if (not isinstance(topics, list) or not topics or len(set(topics)) != len(topics) or
            any(not isinstance(topic, str) or not re.fullmatch(r'/[A-Za-z0-9_/]+', topic) for topic in topics) or
            not {'/joint_states', '/llm_cmd_vel'} <= set(topics)):
        raise ValueError('Telemetry JSON must list distinct absolute topics including joint_states and llm_cmd_vel')
    run = args.run_dir.resolve()
    run.mkdir(mode=0o700, parents=True, exist_ok=False)
    tag = uuid.uuid4().hex
    env = {k: os.environ[k] for k in ENV_KEYS if k in os.environ}
    env.setdefault('PUPPER_FOLLOWER_MODE', 'disabled')
    env.setdefault('PUPPER_SESSION_MOTION_MODE', 'walking')
    env.update(PUPPER_GEMINI_DIR=str(root / 'gemini_pupper'),
               PUPPER_STAGED_GEMINI_DIR=str(root / 'gemini_pupper'),
               PUPPER_SESSION_SKILLS=os.environ['PUPPER_SESSION_SKILLS'],
               PUPPER_SESSION_MANIFEST=str(run / 'runtime-manifest.json'),
               PUPPER_PROBE_SESSION_ID=tag, ROS_LOG_DIR=str(run / 'roslogs'),
               HOST='127.0.0.1', PORT=str(args.port))
    # Keep trace output scoped to this run only when the selected driver supports it.
    if os.environ.get('PUPPER_IMU_STARTUP_TRACE'):
        env['PUPPER_IMU_STARTUP_TRACE'] = str(run / 'native-startup-trace.csv')
    os.environ.update(env)
    plan = {'schema': 'pupper_selected_session_v1', 'session_id': tag,
            'run_dir': str(run), 'app_root': str(root), 'environment': env,
            'components': components, 'interpreter': interpreter_identity(), 'prepared': False,
            'rollback_app': app['baseline_remote_path'], 'motor_launch': False}
    write(run / 'selected-session.json', plan)  # Retain partial preparation evidence.
    profile = module(args.launch.resolve(), 'selected_lab6_profile')
    config, controllers = profile.prepare_session_profile(args.config.resolve(), args.urdf.resolve())
    plan.update(prepared=True, runtime_manifest=pinned(run / 'runtime-manifest.json'),
                generated_config=pinned(Path(config)), controllers=controllers)
    write(run / 'selected-session.json', plan)
    return {'prepared': True, 'motor_launch': False, 'run_dir': str(run),
            'components': components, 'rollback_app': plan['rollback_app']}


# An API release may change application behavior, but cannot reinterpret the
# currently loaded controller/profile contract. Native lifecycle stays original.
API_FIXED_FILES = (
    'gemini_pupper/robot/runtime_config.py', 'gemini_pupper/robot/tricks.py',
    'gemini_pupper/robot/controller_coverage.py', 'gemini_pupper/robot/session_processes.py',
    'foundation/hardware/lab6_session.launch.py', 'foundation/hardware/lab6-session.yaml',
    'foundation/hardware/floor_stock/baseline-expanded.urdf',
    'foundation/hardware/floor_stock/shutdown_coordinator.py',
)


def immutable_json(path, value):
    content = json.dumps(value, indent=2, allow_nan=False) + '\n'
    if path.exists():
        if path.read_text() != content:
            raise ValueError('Existing API attachment differs; inspect it, never overwrite')
    else:
        write(path, value)
    return pinned(path)


def api_asset_may_change(relative):
    return (relative in {'gemini_pupper/robot/robot_api.py', 'gemini_pupper/robot/server.py',
                         'gemini_pupper/robot/tool_registry.py', 'gemini_pupper/lab/prompt.md'}
            or (Path(relative).parent.as_posix() == 'gemini_pupper/lab' and relative.endswith('.py')))


def prepare_api_attachment(run, plan, app_manifest, expected_sha):
    """Bind a new API to immutable existing native inputs; never generate a profile."""
    manifest_pin = pinned(app_manifest, expected_sha)
    app = json.loads(app_manifest.read_text())
    selected = entries_by_path(app['final_selected_files'])
    original_manifest = plan['components']['app_manifest']
    pinned(Path(original_manifest['path']), original_manifest['sha256'])
    original_app = json.loads(Path(original_manifest['path']).read_text())
    previous = entries_by_path(original_app['final_selected_files'])
    if {name for name in previous if not name.startswith('gemini_pupper/web/dist/')} - set(selected):
        raise ValueError('API manifest omits preserved source/dependency identities')
    root = Path(app['destination_remote_path'])
    original = Path(plan['app_root'])
    if (not root.is_absolute() or root.is_symlink() or not root.is_dir()
            or root.resolve().is_relative_to(original.resolve())):
        raise ValueError('API attachment requires a separately staged release')
    root = root.resolve()
    verify_selected(root, list(selected.values()))
    if pinned(root / 'app-stage-manifest.json')['sha256'] != manifest_pin['sha256']:
        raise ValueError('API release installed manifest differs from selected manifest')
    required = {*API_FIXED_FILES, 'gemini_pupper/robot/server.py', 'gemini_pupper/web/dist/index.html'}
    if not required <= set(selected):
        raise ValueError('API manifest omits required application/controller contract files')
    for relative in API_FIXED_FILES:
        if sha(checked_path(root, relative)) != sha(checked_path(original, relative)):
            raise ValueError('API controller/native contract changed: ' + relative)
    native = plan['runtime_manifest']
    pinned(Path(native['path']), native['sha256'])
    pinned(Path(plan['generated_config']['path']), plan['generated_config']['sha256'])
    runtime = json.loads(Path(native['path']).read_text())
    if (runtime.get('schema') != 'pupper_persistent_session_v1'
            or runtime.get('configuration') != plan['generated_config']['path']
            or runtime.get('motion_controllers') != plan['controllers'] or not runtime.get('assets')):
        raise ValueError('Original native profile is incomplete or inconsistent')
    derived = deepcopy(runtime)
    # Preserve every original native input, including old source and follower
    # paths. New source pins supplement them; they never replace old evidence.
    for filename, digest in runtime['assets'].items():
        path = Path(filename)
        pinned(path, digest)
        if path.is_relative_to(original):
            relative = str(path.relative_to(original))
            if api_asset_may_change(relative) and relative not in selected:
                raise ValueError('API manifest omits runtime-bound application source: ' + relative)
            if not api_asset_may_change(relative):
                if sha(checked_path(root, relative)) != digest:
                    raise ValueError('Native-bound asset changed in API release: ' + relative)
    for name, policies in runtime['catalog'].items():
        relocated = {}
        for filename, digest in policies.items():
            old = Path(filename)
            if not old.is_relative_to(original / 'gemini_pupper/tricks'):
                raise ValueError('Selected policy path is outside the original trick catalog')
            new = checked_path(root, str(old.relative_to(original)))
            pinned(old, digest)
            pinned(new, digest)  # descriptors, entry policy and walking policy
            relocated[str(new)] = digest
            derived['assets'][str(new)] = digest
        derived['catalog'][name] = relocated
    for relative, entry in selected.items():
        derived['assets'][str(checked_path(root, relative))] = entry['sha256']
    parent = {'session_id': plan['session_id'],
              'native_plan': pinned(run / 'selected-session.json'),
              'runtime_manifest': native, 'generated_config': plan['generated_config'],
              'native': plan['components']['native'], 'imu': plan['components']['imu'],
              'lifecycle': plan['components']['lifecycle']}
    derived['api_attachment_parent'] = parent
    # All compatibility checks precede writes or process changes.
    directory = checked_path(run, 'api-attachments/' + manifest_pin['sha256'])
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    runtime_pin = immutable_json(directory / 'runtime-manifest.json', derived)
    document = {'schema': 'pupper_api_attachment_v1', 'app_root': str(root),
                'app_manifest': manifest_pin, 'runtime_manifest': runtime_pin,
                'native_parent': parent, 'management_helper': pinned(Path(__file__)),
                'rollback_app_root': plan['app_root'],
                'rollback_app_manifest': plan['components']['app_manifest']}
    return immutable_json(directory / 'api-application.json', document), document


def verify_api_attachment(selection, plan):
    pinned(Path(selection['path']), selection['sha256'])
    document = json.loads(Path(selection['path']).read_text())
    parent = document['native_parent']
    if (document.get('schema') != 'pupper_api_attachment_v1'
            or parent['session_id'] != plan['session_id']
            or parent['runtime_manifest'] != plan['runtime_manifest']
            or parent['generated_config'] != plan['generated_config']
            or any(parent[name] != plan['components'][name] for name in ('native', 'imu', 'lifecycle'))):
        raise ValueError('API attachment belongs to a different native session')
    for item in (parent['native_plan'], document['app_manifest'], document['runtime_manifest'],
                 document['management_helper']):
        pinned(Path(item['path']), item['sha256'])
    app = json.loads(Path(document['app_manifest']['path']).read_text())
    if app['destination_remote_path'] != document['app_root']:
        raise ValueError('API application root differs from its manifest')
    verify_selected(Path(document['app_root']), app['final_selected_files'])
    runtime = json.loads(Path(document['runtime_manifest']['path']).read_text())
    for filename, digest in runtime['assets'].items():
        pinned(Path(filename), digest)
    return document


def load(run, verify=True):
    plan = json.loads((run / 'selected-session.json').read_text())
    if (plan.get('schema') != 'pupper_selected_session_v1' or not plan.get('prepared') or
            plan['run_dir'] != str(run.resolve())):
        raise ValueError('Incomplete or foreign session preparation')
    # Stop remains available when an unrelated staged asset has changed. The
    # selected process-identity implementation itself must still be authentic.
    pinned(Path(plan['components']['lifecycle']['path']), plan['components']['lifecycle']['sha256'])
    if verify:
        for item in [*plan['components'].values(), plan['runtime_manifest'], plan['generated_config']]:
            pinned(Path(item['path']), item['sha256'])
        check_interpreter(plan)
        app = json.loads(Path(plan['components']['app_manifest']['path']).read_text())
        verify_selected(Path(plan['app_root']), app['final_selected_files'])
        runtime = json.loads(Path(plan['runtime_manifest']['path']).read_text())
        if not isinstance(runtime.get('assets'), dict) or not runtime['assets']:
            raise ValueError('Prepared runtime manifest has no pinned assets')
        for path, digest in runtime['assets'].items():
            pinned(Path(path), digest)
    for name in (*ENV_KEYS, 'PUPPER_IMU_STARTUP_TRACE'):
        os.environ.pop(name, None)
    os.environ.update(plan['environment'])
    life = module(Path(plan['components']['lifecycle']['path']), 'selected_lifecycle')
    if verify and (run / 'owned_session.json').exists():
        receipt = receipt_for(life, run, plan)
        if receipt.get('api_application'):
            verify_api_attachment(receipt['api_application'], plan)
    return plan, life


@contextmanager
def lock(life, run):
    with life.coordinator().execution_lock(run, {'manager_exit_allowance_s': 10., 'launch_exit_allowance_s': 3.}):
        yield


def port_free(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
        connection.bind(('127.0.0.1', port))


def owned_api(life, receipt, port):
    target = receipt['processes'].get('server')
    if target is None or life.state(target, receipt['session_id']) != 'running':
        raise ValueError('Recorded API is not running')
    result = subprocess.run(['ss', '-H', '-ltnp', 'sport = :' + str(port)],
                            capture_output=True, text=True, timeout=3, check=True)
    lines = result.stdout.strip().splitlines()
    if not lines or any(set(re.findall(r'pid=(\d+)', line)) != {str(target['pid'])} for line in lines):
        raise ValueError('API port is not owned by the recorded server')


def http(port, path, post=False):
    request = urllib.request.Request('http://127.0.0.1:' + str(port) + path,
                                     data=b'{}' if post else None,
                                     headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=2) as response:
        return json.load(response)


def check_loaded(plan, manager, *, allow_pending=False):
    paths = {line.split(None, 5)[5] for line in
             (Path('/proc') / str(manager['pid']) / 'maps').read_text().splitlines()
             if len(line.split(None, 5)) == 6}
    missing = []
    for name in ('native', 'imu'):
        expected = plan['components'][name]['path']
        conflicting = [path for path in paths if path != expected and
                       Path(path.removesuffix(' (deleted)')).name == Path(expected).name]
        if conflicting:
            raise ValueError('Unexpected or deleted ' + name + ' library mapped by owned manager')
        if expected not in paths:
            missing.append(name)
    if missing and not allow_pending:
        raise ValueError('Selected ' + ', '.join(missing) + ' library is not mapped by owned manager')
    return not missing


def spawners_ready(log, controllers):
    """Current-run spawner completion, not just homing or a Loaded message.

    The selected launch configures motion controllers with --inactive. Each
    spawner exits cleanly only after its complete controller list is configured.
    Feedback broadcasters must also have their activation acknowledgments.
    """
    clean = re.sub(r'\x1b\[[0-9;]*m', '', log)
    finished = set(re.findall(r'^\[INFO\] \[(spawner-\d+)\]: process has finished cleanly \[pid \d+\]\s*$',
                              clean, re.MULTILINE))
    loaded = {name: owner for owner, name in re.findall(
        r'^\[(spawner-\d+)\].*?: Loaded ([A-Za-z0-9_]+)\s*$', clean, re.MULTILINE)}
    active = {name: owner for owner, name in re.findall(
        r'^\[(spawner-\d+)\].*?: Configured and activated ([A-Za-z0-9_]+)\s*$', clean, re.MULTILINE)}
    if not controllers:
        raise ValueError('Prepared profile has no selected motion controllers')
    return (all(loaded.get(name) in finished for name in controllers) and
            all(active.get(name) in finished for name in ('joint_state_broadcaster', 'imu_sensor_broadcaster')))


def receipt_for(life, run, plan):
    receipt = life.load(run)
    if receipt['session_id'] != plan['session_id']:
        raise ValueError('Session receipt does not match prepared profile')
    return receipt


def start_native(run, plan, life):
    with lock(life, run):
        life.check_support(run)
        if (run / 'owned_session.json').exists():
            raise ValueError('Session already started; inspect receipt, never retry initialization automatically')
        if any((run / name).exists() for name in ('robot.log', 'camera.log', 'recorder.log', 'telemetry')):
            raise ValueError('Startup output already exists; refuse stale readiness/reused recording')
        port_free(int(plan['environment']['PORT']))  # Before any potential homing.
        for proc in life.PROC.iterdir():
            if proc.name.isdigit():
                try:
                    # exe is ptrace-protected for unrelated system processes
                    # (including PID1); cmdline is the readable global inventory.
                    # Only vanished processes are skipped. Permission failures
                    # remain visible before any receipt/spawn or potential homing.
                    arguments = (proc / 'cmdline').read_bytes().split(b'\0')
                    names = {Path(os.fsdecode(arg)).name for arg in arguments if arg}
                    if names & {'ros2_control_node', 'camera_node'}:
                        raise ValueError('Existing controller/camera owner; do not launch another')
                except (FileNotFoundError, ProcessLookupError): pass
        receipt = dict(schema=life.SCHEMA, uid=os.getuid(), root=str(life.HERE),
                       boot_id=life.boot_id(), session_id=plan['session_id'], status='starting',
                       ros='created', kiosk=False, gui_stopped=False, processes={}, pending={}, events=[])
        life.save(run, receipt)
        launch = plan['components']['launch']['path']
        # The selected follower subscribes this existing camera; no second camera
        # or velocity mux is created by the helper.
        camera = ['ros2', 'run', 'camera_ros', 'camera_node', '--ros-args', '-r', '__node:=camera',
                  '-p', 'format:=RGB888', '-p', 'width:=1400', '-p', 'height:=1050',
                  '-p', 'AwbEnable:=false', '-p', 'ColourGains:=[1.4,3.3]',
                  '-p', 'FrameDurationLimits:=[100000,100000]']
        topics = json.loads(Path(plan['components']['telemetry_topics']['path']).read_text())
        life.spawn(run, 'camera', 'camera_node', camera)
        life.spawn(run, 'recorder', 'bag', ['ros2', 'bag', 'record', '-s', 'mcap',
                                          '-o', str(run / 'telemetry'), *topics])
        life.spawn(run, 'launch', launch, ['ros2', 'launch', launch])
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        with lock(life, run):
            receipt = receipt_for(life, run, plan); life.require_starting(receipt)
            life.refresh(run, receipt)
            target = receipt['processes']['launch']
            if life.state(target, receipt['session_id']) != 'running':
                raise ValueError('Launch exited before homing completed; receipt retained')
            log = (run / 'robot.log').read_text(errors='replace')
            if 'IMU startup check failed:' in log or 'process has died' in log:
                raise ValueError('Native startup failed; inspect robot.log and owned receipt')
            if 'Finished homing!' in log and 'manager' in receipt['processes']:
                mapped = check_loaded(plan, receipt['processes']['manager'], allow_pending=True)
                if not mapped or not spawners_ready(log, plan['controllers']):
                    # Homing precedes asynchronous motion-controller plugin loading.
                    # Missing maps/completion is pending within the same deadline;
                    # genuine identity, mapping conflicts and I/O errors still fail.
                    pass
                else:
                    life.operate(run, 'ready', [], plan['session_id'])
                    result = {'session_id': plan['session_id'], 'homing_finished': True,
                              'selected_libraries_mapped': True, 'controller_spawners_finished': True,
                              'controllers': plan['controllers'], 'physical_release_verified': False}
                    write(run / 'native-readiness.json', result)
                    return result
        time.sleep(.25)
    raise ValueError('Native readiness (homing, selected libraries, controller spawners) not observed within90s; receipt retained, no automatic retry or cleanup')


def start_api(run, plan, life, *, app_manifest=None, app_manifest_sha256=None):
    port = int(plan['environment']['PORT'])
    with lock(life, run):
        receipt = receipt_for(life, run, plan)
        readiness = json.loads((run / 'native-readiness.json').read_text())
        if (receipt['status'] != 'running' or readiness['session_id'] != plan['session_id'] or
                not readiness['homing_finished'] or life.state(receipt['processes']['manager'], plan['session_id']) != 'running'):
            raise ValueError('Prepared native session is not ready/running')
        if 'server' in receipt['processes'] or 'server' in receipt['pending']:
            raise ValueError('API receipt exists; use stop-api before an explicit restart')
        check_loaded(plan, receipt['processes']['manager'])
        port_free(port)
        if (app_manifest is None) != (app_manifest_sha256 is None):
            raise ValueError('API attachment requires both app manifest and its SHA256')
        application = None
        overrides = {}
        if app_manifest is not None:
            selection, application = prepare_api_attachment(run, plan, app_manifest, app_manifest_sha256)
            receipt['api_application'] = selection
            overrides = {'PUPPER_GEMINI_DIR': application['app_root'] + '/gemini_pupper',
                         'PUPPER_STAGED_GEMINI_DIR': application['app_root'] + '/gemini_pupper',
                         'PUPPER_SESSION_MANIFEST': application['runtime_manifest']['path']}
        receipt['status'] = 'starting'; life.save(run, receipt)
        server = str(Path(application['app_root'] if application else plan['app_root']) / 'gemini_pupper/robot/server.py')
        previous = {name: os.environ.get(name) for name in overrides}
        try:
            os.environ.update(overrides)
            life.spawn(run, 'server', server, [sys.executable, '-u', server])
        finally:
            for name, value in previous.items():
                if value is None: os.environ.pop(name, None)
                else: os.environ[name] = value
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        with lock(life, run):
            receipt = receipt_for(life, run, plan); life.require_starting(receipt)
            if life.state(receipt['processes']['server'], plan['session_id']) != 'running':
                raise ValueError('API exited before readiness; receipt retained')
            try:
                owned_api(life, receipt, port)
                health = http(port, '/health')
            except (OSError, ValueError, subprocess.SubprocessError):
                health = None
            if (isinstance(health, dict) and health.get('status') == 'running' and
                    health.get('tools') is True and health.get('runtime') == 'robot'):
                life.operate(run, 'ready', [], plan['session_id'])
                return {'api_ready': True, 'port': port, 'health': health, 'motor_launch': False,
                        'api_application': receipt.get('api_application'), 'native_app_root': plan['app_root']}
        time.sleep(.1)
    raise ValueError('API readiness timed out; receipt retained, no automatic restart')


def stop_api(run, plan, life):
    with lock(life, run):
        receipt = receipt_for(life, run, plan)
        if receipt['status'] not in ('running', 'starting'):
            raise ValueError('Session is stopping/stopped; API restart not permitted')
        target = receipt['processes'].get('server')
        if target is None:
            raise ValueError('No confirmed API identity; inspect pending receipt')
        error = None
        if life.state(target, plan['session_id']) == 'running':
            try:
                owned_api(life, receipt, int(plan['environment']['PORT']))
                result = http(int(plan['environment']['PORT']), '/stop/immediate', post=True)
                if result.get('success') is not True:
                    raise ValueError('API Stop was not acknowledged as successful')
            except (OSError, ValueError, subprocess.SubprocessError) as caught:
                error = str(caught)  # Still terminate the verified ingress process.
            life.terminate(run, receipt, 'server', target)
            deadline = time.monotonic() + 5
            while life.state(target, plan['session_id']) == 'running' and time.monotonic() < deadline:
                time.sleep(.05)
        if life.state(target, plan['session_id']) == 'running':
            raise ValueError('API exit unconfirmed; native session unchanged, receipt retained')
        receipt.setdefault('retired_servers', []).append({'target': target, 'stop_request_error': error,
            'api_application': receipt.pop('api_application', None),
            'events': [event for event in receipt['events'] if event['event'] == 'term_server']})
        del receipt['processes']['server']
        receipt['events'] = [event for event in receipt['events'] if event['event'] != 'term_server']
        receipt['status'] = 'running'; life.save(run, receipt)
        result = {'api_exited': True, 'native_session_retained': True,
                  'stop_request_error': error, 'physical_stopping_proven': False}
        write(run / 'api-stop.json', result)
        if error: raise ValueError('API exited but Stop acknowledgment unavailable: ' + error)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'start-native', 'start-api', 'stop-api', 'shutdown', 'status'])
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--app-manifest', type=Path)
    parser.add_argument('--app-manifest-sha256')
    for name in ('native', 'imu'):
        parser.add_argument('--' + name + '-library', type=Path)
        parser.add_argument('--' + name + '-sha256')
    here = Path(__file__).resolve().parent
    parser.add_argument('--launch', type=Path, default=here / 'lab6_session.launch.py')
    parser.add_argument('--config', type=Path, default=here / 'lab6-session.yaml')
    parser.add_argument('--urdf', type=Path, default=here / 'floor_stock/baseline-expanded.urdf')
    parser.add_argument('--telemetry-topics', type=Path)
    parser.add_argument('--port', type=int, default=18090)
    args = parser.parse_args(); os.umask(0o077)
    if args.action not in ('prepare', 'start-api') and (args.app_manifest or args.app_manifest_sha256):
        parser.error('app manifest selection is only supported for prepare or explicit start-api')
    if args.action == 'prepare':
        if any(getattr(args, key) is None for key in ('app_manifest', 'app_manifest_sha256', 'native_library', 'native_sha256', 'imu_library', 'imu_sha256', 'telemetry_topics')):
            parser.error('prepare requires pinned app manifest, both native/IMU libraries and telemetry topics JSON')
        result = prepare(args)
    else:
        run = args.run_dir.resolve(); plan, life = load(run, verify=args.action in ('start-native', 'start-api', 'status'))
        if args.action == 'start-native': result = start_native(run, plan, life)
        elif args.action == 'start-api': result = start_api(run, plan, life,
            app_manifest=args.app_manifest, app_manifest_sha256=args.app_manifest_sha256)
        elif args.action == 'stop-api': result = stop_api(run, plan, life)
        elif args.action == 'shutdown':
            with lock(life, run):
                receipt_for(life, run, plan)
                life.cleanup(run)
            result = {'owned_processes_exited': True, 'physical_motor_release_verified': False}
        elif not (run / 'owned_session.json').exists():
            result = {'status': 'prepared', 'motor_launch': False, 'components': plan['components'],
                      'rollback_app': plan['rollback_app']}
        else:
            receipt = receipt_for(life, run, plan)
            result = {'components': plan['components'], 'rollback_app': plan['rollback_app'],
                      'status': receipt['status'], 'api_application': receipt.get('api_application'),
                      'processes': {name: life.state(target, plan['session_id'])
                      for name, target in receipt['processes'].items()}}
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    try: main()
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        print('Session refused: ' + str(error) + '. Inspect retained evidence; no automatic retry.', file=sys.stderr)
        raise SystemExit(1)
