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

from controller_coverage import NEURAL_TYPE, combined_course_config, estop_parameters
from runtime_config import component_dir, follower_config
from tricks import WALK_CONTROLLER, load_tricks, validate_locomotion_contract

MANIFEST = Path('/tmp/gemini-pupper/runtime_manifest.json')

# These values are declared by the loaded controller, with overrides ignored.
# Copying them into launch YAML must never stand in for actual binary support.
CAPABILITY_PARAMETERS = frozenset({
    'reference_trick_status_protocol', 'reference_trick_status_active',
    'policy_execution_status_protocol', 'policy_execution_status_active',
    'interactive_clip_status_protocol', 'interactive_clip_status_active',
    'position_target_slew_active', 'position_target_slew_contract_json',
    'position_target_slew_max_rate_rad_s', 'position_target_slew_max_step_s',
    'position_target_slew_driver_limits_sha256',
})


def _reject_capability_overrides(params):
    for name, entry in params.items():
        supplied = CAPABILITY_PARAMETERS.intersection(entry['ros__parameters'])
        if supplied:
            raise ValueError(f'{name}: launch cannot override loaded capability markers: '
                             + ', '.join(sorted(supplied)))


def _readonly_descriptor_matches(text, parameter, ros_type):
    """Parse the single-parameter ros2 param describe output, failing closed.

    The CLI calls the node's DescribeParameters service. Its output is not YAML:
    the indented fields follow a scalar Parameter name. Accept only the actual
    scalar/array descriptor forms used by these checks, including read_only=True.
    Unexpected output or CLI format changes require inspection, never a pass.
    """
    lines = [line.rstrip() for line in text.splitlines() if line.strip()]
    if len(lines) >= 3 and lines[2].startswith('  Description: '):
        lines.pop(2)
    return lines == [f'Parameter name: {parameter}', f'  Type: {ros_type}',
                     '  Constraints:', '    Read only: true']


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


def _joystick_contract(params, course_config_path, combined_config_path):
    course_path = Path(course_config_path).resolve()
    combined_path = Path(combined_config_path).resolve()
    if course_path == combined_path:
        raise ValueError('combined config must not overwrite the course configuration')
    expected = combined_course_config(yaml.safe_load(course_path.read_text()), params)
    actual = yaml.safe_load(combined_path.read_text())
    if actual != expected:
        raise ValueError('combined joystick configuration differs from current course config/controller set')
    return {'source_path': str(course_path), 'source_sha256': _sha(course_path),
            'combined_path': str(combined_path), 'combined_sha256': _sha(combined_path),
            'parameters': estop_parameters(expected)}


def write_launch_manifest(params, config, path=MANIFEST, *, course_config_path, combined_config_path):
    """Validate every resolved native policy before any launch actions execute."""
    _reject_capability_overrides(params)
    joystick = _joystick_contract(params, course_config_path, combined_config_path)
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
    manifest = {'schema': 2, 'parameters': params, 'policies': policies, 'joystick': joystick,
                'tricks': _catalog(tricks), 'follower': _follower(config)}
    Path(path).write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')


def inspect_running_stack(*, run=subprocess.run, path=MANIFEST):
    """Only list/get/dump/describe operations; never change the running stack."""
    report = {'compatible': False, 'errors': []}
    errors = report['errors']
    slew_requirements = {}
    try:
        manifest = json.loads(Path(path).read_text())
        config = follower_config()
        tricks = load_tricks()
        _reject_capability_overrides(manifest['parameters'])
        policy_controllers = {t.controller for t in tricks.values() if t.kind == 'policy'}
        interactive_controllers = {t.controller for t in tricks.values() if t.kind == 'policy' and t.interactive}
        expected = {WALK_CONTROLLER} | {t.controller for t in tricks.values() if t.kind == 'policy'}
        expected |= {t.walk_controller for t in tricks.values() if t.walks}
        if manifest.get('schema') != 2 or set(manifest['parameters']) != expected or set(manifest['policies']) != expected:
            errors.append('launch manifest does not match the installed controller set')
        joystick = manifest['joystick']
        if joystick != _joystick_contract(manifest['parameters'], joystick['source_path'], joystick['combined_path']):
            errors.append('joystick launch configuration changed since the combined launch')
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
    except (OSError, ValueError, KeyError, TypeError, AttributeError, yaml.YAMLError) as exc:
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

    def require_readonly_markers(name, markers):
        for parameter, ros_type in markers:
            descriptor = read('param', 'describe', '/' + name, parameter)
            if not _readonly_descriptor_matches(descriptor, parameter, ros_type):
                errors.append(f'{name}: loaded capability {parameter} must have a '
                              f'read-only {ros_type} parameter descriptor')

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
            if kind == NEURAL_TYPE and name not in joystick['parameters']['controller_names']:
                errors.append(f'{name}: configured neural controller is outside joystick stop coverage')
            if state == 'active' and 'forward_command_controller/' in kind:
                errors.append(f'{name}: a classical command controller is active; use separate operating modes')
        for name in ('imu_sensor_broadcaster', 'joint_state_broadcaster'):
            if controllers.get(name, ('', 'missing'))[1] != 'active':
                errors.append(f'{name}: required feedback broadcaster is not active')
        nodes = read('node', 'list').splitlines()
        required_nodes = {'/controller_manager', '/cmd_vel_mux', '/teleop_twist_joy_node', '/camera', '/estop_controller'}
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
            required_parameters['estop_controller'] = {'ros__parameters': joystick['parameters']}
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
                if name == 'estop_controller':
                    estop_parameters({'estop_controller': {'ros__parameters': actual}})
                    # joy_utils caches these values in its constructor. A writable
                    # parameter dump can diverge from that cache on old binaries.
                    require_readonly_markers(name, [('controller_names', 'string array'),
                        ('switch_button_indices', 'integer array'), ('estop_index', 'integer'),
                        ('estop_release_index', 'integer')])
                for key, value in entry['ros__parameters'].items():
                    if key != 'type' and actual.get(key) != value:
                        errors.append(f'{name}: running parameter {key} differs from this launch')
                if entry['ros__parameters'].get('reference_tricks') is True:
                    if (actual.get('reference_trick_status_protocol') != 'pupper_reference_v1'
                            or actual.get('reference_trick_status_active') is not True):
                        errors.append(f'{name}: loaded controller does not confirm reference playback acknowledgements')
                    else:
                        require_readonly_markers(name, [('reference_trick_status_protocol', 'string'),
                                                       ('reference_trick_status_active', 'boolean')])
                if name in policy_controllers:
                    if (actual.get('policy_execution_status_protocol') != 'pupper_policy_v1'
                            or actual.get('policy_execution_status_active') is not True):
                        errors.append(f'{name}: loaded controller does not confirm policy execution acknowledgements')
                    else:
                        require_readonly_markers(name, [('policy_execution_status_protocol', 'string'),
                                                       ('policy_execution_status_active', 'boolean')])
                if name in interactive_controllers:
                    if (actual.get('interactive_clip_status_protocol') != 'pupper_clip_v1'
                            or actual.get('interactive_clip_status_active') is not True):
                        errors.append(f'{name}: loaded controller does not confirm correlated interactive clip acknowledgements')
                    else:
                        require_readonly_markers(name, [('interactive_clip_status_protocol', 'string'),
                                                       ('interactive_clip_status_active', 'boolean')])
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
                    else:
                        require_readonly_markers(name, [('position_target_slew_active', 'boolean'),
                                                       ('position_target_slew_contract_json', 'string')])
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
