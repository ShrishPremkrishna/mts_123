"""Read-only startup inspection, including refusal before any stack mutation."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest
import yaml

ROBOT_DIR = Path(__file__).resolve().parents[1] / 'robot'
sys.path.insert(0, str(ROBOT_DIR))
import startup_readiness as readiness
from runtime_config import FollowerConfig


def descriptor(parameter, ros_type, *, readonly=True):
    # Exact ros2cli DescribeVerb output, not YAML or a fabricated service format.
    return (f'Parameter name: {parameter}\n  Type: {ros_type}\n'
            '  Description: Loaded controller capability (not a launch override)\n'
            '  Constraints:\n' + ('    Read only: true\n' if readonly else ''))


def readonly_markers(stack, node, markers):
    for parameter, ros_type in markers:
        stack.responses[('param', 'describe', '/' + node, parameter)] = descriptor(parameter, ros_type)


@pytest.fixture
def stack(tmp_path, monkeypatch):
    policy = tmp_path / 'default_policy.json'
    shutil.copyfile(ROBOT_DIR.parents[1] / 'pupper_gait_deploy/policies/default_policy.json', policy)
    config = FollowerConfig('disabled', None, False, False, False)
    monkeypatch.setattr(readiness, 'follower_config', lambda: config)
    monkeypatch.setattr(readiness, 'load_tricks', lambda: {})
    params = {'gemini_walk': {'ros__parameters': {
        'type': 'neural_controller/NeuralController', 'model_path': str(policy), 'game_buttons': False}}}
    manifest = tmp_path / 'runtime_manifest.json'
    readiness.write_launch_manifest(params, config, path=manifest)
    responses = {
        ('control', 'list_controllers'): '\n'.join([
            'gemini_walk neural_controller/NeuralController inactive',
            'joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active',
            'imu_sensor_broadcaster imu_sensor_broadcaster/IMUSensorBroadcaster active']),
        ('node', 'list'): '/controller_manager\n/cmd_vel_mux\n/teleop_twist_joy_node\n/camera\n',
        ('service', 'list'): '/controller_manager/list_controllers\n/controller_manager/switch_controller\n',
        ('param', 'dump', '/gemini_walk'): yaml.safe_dump({'/gemini_walk': params['gemini_walk']}),
        ('param', 'dump', '/cmd_vel_mux'): yaml.safe_dump({'/cmd_vel_mux': {'ros__parameters': {
            'deadband': .05, 'timeout_ms': 500,
            'inputs': ['/teleop_cmd_vel', '/llm_cmd_vel', '/person_following_cmd_vel']}}}),
    }
    calls = []
    def run(args, **kwargs):
        assert args[0] == 'ros2'
        key = tuple(args[1:])
        assert key in responses, f'unexpected/non-read-only operation: {args}'
        calls.append(key)
        return SimpleNamespace(returncode=0, stdout=responses[key])
    return SimpleNamespace(manifest=manifest, params=params, policy=policy, responses=responses, calls=calls, run=run)


def test_existing_compatible_stack_is_reused_with_read_only_inspection(stack):
    result = readiness.inspect_running_stack(run=stack.run, path=stack.manifest)
    assert result == {'compatible': True, 'errors': []}
    assert ('param', 'dump', '/gemini_walk') in stack.calls


@pytest.mark.parametrize('fault', ['course_only', 'no_mux', 'classical_active', 'wrong_policy', 'no_joystick_priority'])
def test_existing_incompatible_stack_is_refused_without_mutation(stack, fault):
    if fault == 'course_only':
        stack.responses[('control', 'list_controllers')] = 'neural_controller_default neural_controller/NeuralController active'
    elif fault == 'no_mux':
        stack.responses[('node', 'list')] = '/controller_manager\n/camera\n'
    elif fault == 'classical_active':
        stack.responses[('control', 'list_controllers')] += '\nforward_command_controller forward_command_controller/ForwardCommandController active'
    elif fault == 'wrong_policy':
        stack.responses[('param', 'dump', '/gemini_walk')] = '/gemini_walk:\n  ros__parameters:\n    model_path: /old/policy.json\n'
    else:
        stack.responses[('param', 'dump', '/cmd_vel_mux')] = '/cmd_vel_mux:\n  ros__parameters:\n    inputs: [/llm_cmd_vel, /teleop_cmd_vel]\n'
    result = readiness.inspect_running_stack(run=stack.run, path=stack.manifest)
    assert not result['compatible'] and result['errors']


def test_changed_policy_artifact_refuses_reuse_before_ros_queries(stack):
    stack.policy.write_text('{}')
    result = readiness.inspect_running_stack(run=stack.run, path=stack.manifest)
    assert not result['compatible'] and 'changed' in ' '.join(result['errors'])
    assert stack.calls == []


def test_launch_rejects_invalid_native_policy_before_recording_manifest(stack):
    stack.manifest.unlink()
    payload = json.loads(stack.policy.read_text())
    payload.pop('layers')
    stack.policy.write_text(json.dumps(payload))
    with pytest.raises((ValueError, SystemExit)):
        readiness.write_launch_manifest(stack.params, FollowerConfig('disabled', None, False, False, False), path=stack.manifest)
    assert not stack.manifest.exists()


def test_read_only_inspection_timeout_is_a_clear_refusal(stack):
    def timed_out(args, **kwargs):
        raise subprocess.TimeoutExpired(args, 15)
    result = readiness.inspect_running_stack(run=timed_out, path=stack.manifest)
    assert not result['compatible'] and 'TimeoutExpired' in ' '.join(result['errors'])


def test_start_checks_compatibility_before_replacing_any_server():
    # A regression in shell ordering must not kill the old server before refusal.
    source = (ROBOT_DIR / 'start.sh').read_text()
    inspect = source.index('"$HERE/startup_readiness.py" || exit 1')
    assert source.index('if pgrep -f "[r]os2_control_node"') < inspect
    assert source.index('exec ros2 launch') < inspect  # new-stack launch path retained
    assert inspect < source.index('pkill -f "[r]obot/server.py"')


@pytest.mark.parametrize('runtime', ['matching', 'old_binary', 'different_rate', 'invalid_json', 'inactive'])
def test_slew_policy_requires_actual_loaded_controller_capability(stack, runtime):
    policy = json.loads(stack.policy.read_text())
    contract = {'contract':'pupper_position_target_slew_v1','max_rate_rad_s':15.,'max_step_s':.02}
    policy['position_target_slew'] = contract
    stack.policy.write_text(json.dumps(policy))
    manifest = json.loads(stack.manifest.read_text())
    manifest['policies']['gemini_walk']['sha256'] = readiness._sha(stack.policy)
    stack.manifest.write_text(json.dumps(manifest))
    actual = dict(stack.params['gemini_walk']['ros__parameters'])
    readonly_markers(stack, 'gemini_walk', [('position_target_slew_active', 'boolean'),
                                           ('position_target_slew_contract_json', 'string')])
    if runtime != 'old_binary':
        loaded = dict(contract)
        if runtime == 'different_rate': loaded['max_rate_rad_s'] = 10.
        actual['position_target_slew_active'] = runtime != 'inactive'
        actual['position_target_slew_contract_json'] = 'invalid' if runtime == 'invalid_json' else json.dumps(loaded)
    stack.responses[('param','dump','/gemini_walk')] = yaml.safe_dump({'/gemini_walk':{'ros__parameters':actual}})
    result = readiness.inspect_running_stack(run=stack.run, path=stack.manifest)
    assert result['compatible'] is (runtime == 'matching')
    if runtime != 'matching': assert 'slew contract' in ' '.join(result['errors'])


@pytest.mark.parametrize('runtime',[
    {}, {'reference_trick_status_protocol':'old','reference_trick_status_active':True},
    {'reference_trick_status_protocol':'pupper_reference_v1','reference_trick_status_active':False}])
def test_reference_enabled_requires_actual_controller_acknowledgement_support(stack,runtime):
    manifest=json.loads(stack.manifest.read_text())
    manifest['parameters']['gemini_walk']['ros__parameters']['reference_tricks']=True
    stack.manifest.write_text(json.dumps(manifest))
    actual=dict(manifest['parameters']['gemini_walk']['ros__parameters']);actual.update(runtime)
    stack.responses[('param','dump','/gemini_walk')]=yaml.safe_dump({'/gemini_walk':{'ros__parameters':actual}})
    result=readiness.inspect_running_stack(run=stack.run,path=stack.manifest)
    assert not result['compatible'] and any('reference playback acknowledgements' in e for e in result['errors'])


def reference_stack(stack):
    manifest=json.loads(stack.manifest.read_text());manifest['parameters']['gemini_walk']['ros__parameters']['reference_tricks']=True
    stack.manifest.write_text(json.dumps(manifest))
    actual=dict(manifest['parameters']['gemini_walk']['ros__parameters'])
    actual.update(reference_trick_status_protocol='pupper_reference_v1',reference_trick_status_active=True)
    stack.responses[('param','dump','/gemini_walk')]=yaml.safe_dump({'/gemini_walk':{'ros__parameters':actual}})
    readonly_markers(stack, 'gemini_walk', [('reference_trick_status_protocol', 'string'),
                                           ('reference_trick_status_active', 'boolean')])


def test_reference_controller_readonly_support_passes(stack):
    reference_stack(stack)
    assert readiness.inspect_running_stack(run=stack.run,path=stack.manifest)['compatible']
    assert ('param', 'describe', '/gemini_walk', 'reference_trick_status_active') in stack.calls


@pytest.mark.parametrize('marker', sorted(readiness.CAPABILITY_PARAMETERS))
def test_launch_rejects_external_capability_markers_before_recording_manifest(stack, marker):
    stack.manifest.unlink()
    stack.params['gemini_walk']['ros__parameters'][marker] = True
    with pytest.raises(ValueError, match='launch cannot override loaded capability markers'):
        readiness.write_launch_manifest(stack.params, FollowerConfig('disabled', None, False, False, False), path=stack.manifest)
    assert not stack.manifest.exists()


@pytest.mark.parametrize('marker', sorted(readiness.CAPABILITY_PARAMETERS))
def test_stale_manifest_cannot_supply_capability_markers(stack, marker):
    manifest = json.loads(stack.manifest.read_text())
    manifest['parameters']['gemini_walk']['ros__parameters'][marker] = True
    stack.manifest.write_text(json.dumps(manifest))
    result = readiness.inspect_running_stack(run=stack.run, path=stack.manifest)
    assert not result['compatible']
    assert 'launch cannot override loaded capability markers' in ' '.join(result['errors'])
    assert stack.calls == []


@pytest.mark.parametrize('marker,ros_type', [
    ('reference_trick_status_protocol', 'string'), ('reference_trick_status_active', 'boolean')])
@pytest.mark.parametrize('fault', ['writable', 'wrong_type', 'wrong_name', 'empty', 'duplicate', 'description_spoof'])
def test_copied_reference_values_do_not_replace_readonly_descriptors(stack, marker, ros_type, fault):
    reference_stack(stack)
    text = descriptor(marker, ros_type)
    if fault == 'writable': text = descriptor(marker, ros_type, readonly=False)
    elif fault == 'wrong_type': text = descriptor(marker, 'integer')
    elif fault == 'wrong_name': text = descriptor('other_parameter', ros_type)
    elif fault == 'empty': text = ''
    elif fault == 'duplicate': text += text
    else:
        # A line in a human description must not masquerade as the constraint.
        text = (f'Parameter name: {marker}\n  Type: {ros_type}\n'
                '  Description: Read only: true\n  Constraints:\n')
    stack.responses[('param', 'describe', '/gemini_walk', marker)] = text
    result = readiness.inspect_running_stack(run=stack.run, path=stack.manifest)
    assert not result['compatible'] and 'parameter descriptor' in ' '.join(result['errors'])


@pytest.mark.parametrize('failure', ['timeout', 'service_error'])
def test_descriptor_service_failure_refuses_reuse(stack, failure):
    reference_stack(stack)
    def run(args, **kwargs):
        if args[1:3] == ['param', 'describe']:
            if failure == 'timeout': raise subprocess.TimeoutExpired(args, 15)
            return SimpleNamespace(returncode=1, stdout='')
        return stack.run(args, **kwargs)
    result = readiness.inspect_running_stack(run=run, path=stack.manifest)
    assert not result['compatible'] and 'inspect' in ' '.join(result['errors'])


@pytest.mark.parametrize('marker,ros_type', [
    ('position_target_slew_active', 'boolean'), ('position_target_slew_contract_json', 'string')])
def test_matching_slew_values_with_writable_descriptors_are_refused(stack, marker, ros_type):
    policy = json.loads(stack.policy.read_text())
    contract = {'contract':'pupper_position_target_slew_v1','max_rate_rad_s':15.,'max_step_s':.02}
    policy['position_target_slew'] = contract
    stack.policy.write_text(json.dumps(policy))
    manifest = json.loads(stack.manifest.read_text())
    manifest['policies']['gemini_walk']['sha256'] = readiness._sha(stack.policy)
    stack.manifest.write_text(json.dumps(manifest))
    actual = dict(stack.params['gemini_walk']['ros__parameters'],
                  position_target_slew_active=True, position_target_slew_contract_json=json.dumps(contract))
    stack.responses[('param', 'dump', '/gemini_walk')] = yaml.safe_dump({'/gemini_walk': {'ros__parameters': actual}})
    readonly_markers(stack, 'gemini_walk', [('position_target_slew_active', 'boolean'),
                                           ('position_target_slew_contract_json', 'string')])
    stack.responses[('param', 'describe', '/gemini_walk', marker)] = descriptor(marker, ros_type, readonly=False)
    result = readiness.inspect_running_stack(run=stack.run, path=stack.manifest)
    assert not result['compatible'] and 'parameter descriptor' in ' '.join(result['errors'])


def policy_stack(stack, monkeypatch):
    (stack.policy.parent / 'trick.yaml').write_text('type: policy\n')
    trick = SimpleNamespace(controller='trick_test', kind='policy', walks=True, walk_controller='walk_test',
                            path=stack.policy, walk_path=stack.policy)
    monkeypatch.setattr(readiness, 'load_tricks', lambda: {'test': trick})
    manifest = json.loads(stack.manifest.read_text())
    manifest['tricks'] = readiness._catalog({'test': trick})
    for name in ('trick_test', 'walk_test'):
        params = dict(stack.params['gemini_walk']['ros__parameters'])
        manifest['parameters'][name] = {'ros__parameters': params}
        manifest['policies'][name] = dict(manifest['policies']['gemini_walk'])
        stack.responses[('control', 'list_controllers')] += f'\n{name} neural_controller/NeuralController inactive'
        actual = dict(params)
        if name == 'trick_test':
            actual.update(policy_execution_status_protocol='pupper_policy_v1', policy_execution_status_active=True)
        stack.responses[('param', 'dump', '/' + name)] = yaml.safe_dump({'/' + name: {'ros__parameters': actual}})
    stack.manifest.write_text(json.dumps(manifest))
    readonly_markers(stack, 'trick_test', [('policy_execution_status_protocol', 'string'),
                                         ('policy_execution_status_active', 'boolean')])


def test_policy_acknowledgements_require_loaded_readonly_capability_but_walk_actor_does_not(stack, monkeypatch):
    policy_stack(stack, monkeypatch)
    assert readiness.inspect_running_stack(run=stack.run, path=stack.manifest)['compatible']
    assert ('param', 'describe', '/trick_test', 'policy_execution_status_active') in stack.calls
    assert not any(call[:3] == ('param', 'describe', '/walk_test') for call in stack.calls)


@pytest.mark.parametrize('fault', ['missing', 'inactive', 'wrong_protocol', 'writable', 'wrong_type'])
def test_policy_controller_without_confirmed_execution_capability_is_refused(stack, monkeypatch, fault):
    policy_stack(stack, monkeypatch)
    key = ('param', 'dump', '/trick_test')
    actual = yaml.safe_load(stack.responses[key])['/trick_test']['ros__parameters']
    if fault == 'missing': actual.pop('policy_execution_status_protocol')
    elif fault == 'inactive': actual['policy_execution_status_active'] = False
    elif fault == 'wrong_protocol': actual['policy_execution_status_protocol'] = 'old'
    else:
        stack.responses[('param', 'describe', '/trick_test', 'policy_execution_status_active')] = descriptor(
            'policy_execution_status_active', 'integer' if fault == 'wrong_type' else 'boolean', readonly=fault != 'writable')
    stack.responses[key] = yaml.safe_dump({'/trick_test': {'ros__parameters': actual}})
    result = readiness.inspect_running_stack(run=stack.run, path=stack.manifest)
    assert not result['compatible'] and any('trick_test' in error for error in result['errors'])


def test_mutable_execution_request_id_is_not_misclassified_as_capability_override(stack):
    stack.params['gemini_walk']['ros__parameters']['execution_request_id'] = ''
    readiness.write_launch_manifest(stack.params, FollowerConfig('disabled', None, False, False, False), path=stack.manifest)
    assert json.loads(stack.manifest.read_text())['parameters']['gemini_walk']['ros__parameters']['execution_request_id'] == ''
