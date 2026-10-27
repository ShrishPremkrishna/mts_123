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
