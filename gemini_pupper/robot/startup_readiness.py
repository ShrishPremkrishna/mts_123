"""Read-only compatibility check before reusing a running Gemini ROS stack.

The launch manifest describes files and parameters used by this checkout's
launch. It is compatibility evidence, not proof of physical readiness or torque.
"""
from __future__ import annotations

import contextlib
from dataclasses import asdict
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import re
import subprocess
import sys

import yaml

from runtime_config import component_dir, follower_config
from tricks import WALK_CONTROLLER, load_tricks, validate_locomotion_contract

MANIFEST = Path('/tmp/gemini-pupper/runtime_manifest.json')


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _follower(config):
    return {key: str(value) if isinstance(value, Path) else value for key, value in asdict(config).items()}


def _catalog(tricks):
    result = {}
    for name, trick in tricks.items():
        paths = [trick.path, trick.path.parent / 'trick.yaml']
        if trick.walk_path:
            paths.append(trick.walk_path)
        result[name] = {str(path.resolve()): _sha(path) for path in paths}
    return result


def write_launch_manifest(params, config, path=MANIFEST):
    """Validate every resolved native policy before any launch actions execute."""
    validator_path = component_dir('pupper_gait_deploy', 'PUPPER_GAIT_DEPLOY_DIR') / 'download_policy.py'
    spec = importlib.util.spec_from_file_location('pupper_startup_policy_validator', validator_path)
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    policies = {}
    for name, entry in params.items():
        model = Path(entry['ros__parameters']['model_path']).resolve()
        with contextlib.redirect_stdout(io.StringIO()):
            validator.validate(model)
        policies[name] = {'path': str(model), 'sha256': _sha(model)}
    tricks = load_tricks()
    validate_locomotion_contract(Path(policies[WALK_CONTROLLER]['path']),
                                 [t for t in tricks.values() if t.kind == 'reference'])
    manifest = {'schema': 1, 'parameters': params, 'policies': policies,
                'tricks': _catalog(tricks), 'follower': _follower(config)}
    Path(path).write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')


def inspect_running_stack(*, run=subprocess.run, path=MANIFEST):
    """Only list/get/dump operations; never launch, set, switch, publish or stop."""
    report = {'compatible': False, 'errors': []}
    errors = report['errors']
    slew_requirements = {}
    try:
        manifest = json.loads(Path(path).read_text())
        config = follower_config()
        tricks = load_tricks()
        expected = {WALK_CONTROLLER} | {t.controller for t in tricks.values() if t.kind == 'policy'}
        expected |= {t.walk_controller for t in tricks.values() if t.walks}
        if manifest.get('schema') != 1 or set(manifest['parameters']) != expected or set(manifest['policies']) != expected:
            errors.append('launch manifest does not match the installed controller set')
        if manifest.get('follower') != _follower(config):
            errors.append('running launch used different follower mode, paths or feature flags')
        if manifest.get('tricks') != _catalog(tricks):
            errors.append('trick files or descriptors changed since the combined launch')
        for name, policy in manifest['policies'].items():
            if _sha(policy['path']) != policy['sha256']:
                errors.append(f'{name}: policy file changed since the combined launch')
            data = json.loads(Path(policy['path']).read_text())
            if 'position_target_slew' in data:
                slew_requirements[name] = data['position_target_slew']
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        errors.append(f'missing or invalid combined-launch manifest: {exc}')
        return report
    if errors:
        return report

    def read(*args):
        try:
            result = run(['ros2', *args], capture_output=True, text=True, timeout=15, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ValueError(f'could not inspect ROS {" ".join(args)}: {type(exc).__name__}') from exc
        if result.returncode:
            raise ValueError(f'ROS inspection failed: {" ".join(args)}')
        return re.sub(r'\x1b\[[0-9;]*m', '', result.stdout)

    try:
        controllers = {}
        for line in read('control', 'list_controllers').splitlines():
            fields = line.split()
            if len(fields) == 3:
                controllers[fields[0]] = fields[1:]
        for name in sorted(expected):
            kind, state = controllers.get(name, ('', 'missing'))
            if kind != 'neural_controller/NeuralController' or state not in ('inactive', 'active'):
                errors.append(f'{name}: expected a configured neural controller, found {state}')
        for name, (kind, state) in controllers.items():
            if state == 'active' and 'forward_command_controller/' in kind:
                errors.append(f'{name}: a classical command controller is active; use separate operating modes')
        for name in ('imu_sensor_broadcaster', 'joint_state_broadcaster'):
            if controllers.get(name, ('', 'missing'))[1] != 'active':
                errors.append(f'{name}: required feedback broadcaster is not active')
        nodes = read('node', 'list').splitlines()
        required_nodes = {'/controller_manager', '/cmd_vel_mux', '/teleop_twist_joy_node', '/camera'}
        if config.mode != 'disabled':
            required_nodes.add('/person_follower_node')
        for name in sorted(required_nodes):
            if nodes.count(name) != 1:
                errors.append(f'{name}: expected exactly one node in the combined stack')
        services = set(read('service', 'list').splitlines())
        required_services = {'/controller_manager/list_controllers', '/controller_manager/switch_controller'}
        if config.mode != 'disabled':
            required_services |= {'/activate_person_following', '/deactivate_person_following'}
        for name in sorted(required_services - services):
            errors.append(f'missing service {name}')
        # Refuse an old/differently configured controller despite matching names.
        if not errors:
            required_parameters = dict(manifest['parameters'])
            required_parameters['cmd_vel_mux'] = {'ros__parameters': {
                'deadband': 0.05, 'timeout_ms': 500,
                'inputs': ['/teleop_cmd_vel', '/llm_cmd_vel', '/person_following_cmd_vel']}}
            if config.mode == 'lab5':
                required_parameters['person_follower_node'] = {'ros__parameters': {
                    'always_detect': config.always_detect, 'use_color': config.use_color,
                    'avoid_obstacles': config.avoid_obstacles}}
            for name, entry in required_parameters.items():
                dump = yaml.safe_load(read('param', 'dump', '/' + name))
                actual = (dump.get('/' + name) or dump.get(name) or {}).get('ros__parameters', {})
                for key, value in entry['ros__parameters'].items():
                    if key != 'type' and actual.get(key) != value:
                        errors.append(f'{name}: running parameter {key} differs from this launch')
                if name in slew_requirements:
                    # These are read-only values declared by the loaded controller,
                    # never launch overrides. Older binaries ignore unknown JSON
                    # fields, so matching a model path alone is insufficient.
                    try:
                        loaded = json.loads(actual.get('position_target_slew_contract_json', 'null'))
                    except (TypeError, ValueError):
                        loaded = None
                    if actual.get('position_target_slew_active') is not True or loaded != slew_requirements[name]:
                        errors.append(f'{name}: loaded controller does not confirm the policy position-target slew contract')
    except (ValueError, KeyError, AttributeError, yaml.YAMLError) as exc:
        errors.append(str(exc))
    report['compatible'] = not errors
    return report


def main():
    report = inspect_running_stack()
    if report['compatible']:
        print('Combined ROS stack matches this checkout, policies and follower configuration.')
        return 0
    print('Refusing to reuse an incompatible or unverified ROS stack:', file=sys.stderr)
    for error in report['errors']:
        print(' - ' + error, file=sys.stderr)
    print('No existing ROS process was stopped or changed. With the operator beside the robot, '
          'use the approved stop/setup procedure, preserve the working baseline, then launch this stack explicitly. '
          'Do not start a second motor-control stack.', file=sys.stderr)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
