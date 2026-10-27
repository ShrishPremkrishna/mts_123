"""Offline sensor-loss, stop-service and full default-gait regressions (no ROS)."""
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ik_heuristic_walking_lab'))
from controller_support import joint_sample, state_fresh, request_position_stop, confirm_position_stop
from kinematics import LEG_LIMITS

NAMES = [f'leg_{leg}_{joint}' for leg in ('front_r', 'front_l', 'back_r', 'back_l')
         for joint in (1, 2, 3)]


def sample():
    return SimpleNamespace(name=NAMES.copy(), position=[0.] * 12, velocity=[0.] * 12)


@pytest.mark.parametrize('problem', ['missing', 'duplicate', 'short_position', 'short_velocity', 'nan', 'inf'])
def test_partial_or_invalid_joint_state_rejected(problem):
    msg = sample()
    if problem == 'missing':
        msg.name[0] = 'other_joint'
    elif problem == 'duplicate':
        msg.name[0] = msg.name[1]
    elif problem == 'short_position':
        msg.position.pop()
    elif problem == 'short_velocity':
        msg.velocity.clear()
    elif problem == 'nan':
        msg.position[2] = float('nan')
    else:
        msg.velocity[3] = float('inf')
    assert joint_sample(msg, NAMES) is None


def test_joint_order_and_freshness():
    msg = sample()
    msg.position = list(range(12))
    msg.name.reverse()
    positions, velocities = joint_sample(msg, NAMES)
    np.testing.assert_equal(positions, np.arange(11, -1, -1))
    assert not velocities.any()
    assert state_fresh(1., 1.25)
    for past, now in [(None, 1), (1, 1.251), (2, 1), (float('nan'), 1)]:
        assert not state_fresh(past, now)


@pytest.fixture
def walking(monkeypatch):
    class Timer:
        cancelled = False
        def cancel(self):
            self.cancelled = True
    class Future:
        def done(self): return True
        def cancelled(self): return False
        def exception(self): return None
        def result(self): return SimpleNamespace(ok=True)
    class Node:
        def __init__(self, name):
            self.sent, self.requests, self.errors = [], [], []
            self.service_available = True
        def create_subscription(self, *args): return None
        def create_publisher(self, *args): return SimpleNamespace(publish=self.sent.append)
        def create_timer(self, *args): return Timer()
        def get_logger(self):
            return SimpleNamespace(info=lambda message: None, error=self.errors.append)
        def create_client(self, *args):
            def call(request):
                self.requests.append(request)
                return Future()
            return SimpleNamespace(wait_for_service=lambda **kwargs: self.service_available,
                                   call_async=call)
    class Request:
        BEST_EFFORT = 1
        def __init__(self): self.timeout = SimpleNamespace(sec=0)
    stubs = {
        'rclpy': {'ok': lambda: True, 'spin_until_future_complete': lambda *args, **kwargs: None},
        'rclpy.node': {'Node': Node},
        'rclpy.signals': {'SignalHandlerOptions': SimpleNamespace(NO=0)},
        'sensor_msgs.msg': {'JointState': object},
        'std_msgs.msg': {'Float64MultiArray': SimpleNamespace},
        'controller_manager_msgs.srv': {'SwitchController': SimpleNamespace(Request=Request)},
    }
    for name, attributes in stubs.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location('walking_under_test', ROOT / 'ik_heuristic_walking_lab/walking.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Keep monotonic time local to this module, not the process-wide time module.
    module.time = SimpleNamespace(monotonic=lambda: 10.)
    return module


def test_default_gait_complete_cache_is_bounded_reachable_and_roundtrips(walking, tmp_path, monkeypatch):
    node = walking.InverseKinematics()
    joints, feet = node.target_joint_positions_cache, node.target_ee_cache
    assert joints.shape == feet.shape == (50, 12)
    limits = np.concatenate(LEG_LIMITS)
    assert np.all(joints >= limits[:, 0]) and np.all(joints <= limits[:, 1])
    for q, target in zip(joints, feet):
        residual = np.linalg.norm((node.forward_kinematics(q) - target).reshape(4, 3), axis=1)
        assert residual.max() <= 1e-4
    monkeypatch.setattr(walking, 'CACHE_FILE', str(tmp_path / 'cache.npz'))
    node.save_cached_joint_positions()
    loaded = node.load_cached_joint_positions()
    np.testing.assert_array_equal(loaded[0], joints)
    # A valid hash does not authorize a different reachable motion in the cache.
    np.savez(walking.CACHE_FILE, target_joint_positions=joints[::-1], target_ee=feet[::-1],
             keyframe_hash=node.keyframe_hash())
    with pytest.raises(ValueError, match='configured gait'):
        node.load_cached_joint_positions()


@pytest.mark.parametrize('malformed', [False, True])
def test_sensor_fault_latches_controller_stop_and_never_sends_zero_pose(walking, malformed):
    node = walking.InverseKinematics()
    node.listener_callback(sample())
    node.ik_timer_callback()
    node.pd_timer_callback()
    assert len(node.sent) == 1
    if malformed:
        broken = sample()
        broken.velocity.clear()
        node.listener_callback(broken)
    else:
        walking.time.monotonic = lambda: 10.251
        node.pd_timer_callback()
    assert node.sensor_fault and node.target_joint_positions is None
    assert node.pd_timer.cancelled and node.ik_timer.cancelled
    assert node.requests[-1].deactivate_controllers == ['forward_command_controller']
    # Even a fresh valid packet after repair cannot silently restart the gait.
    node.listener_callback(sample())
    node.ik_timer_callback()
    node.pd_timer_callback()
    assert len(node.sent) == 1
    assert node.stop_controller()


def test_shutdown_service_failure_is_reported(walking):
    node = walking.InverseKinematics()
    node.service_available = False
    assert not node.stop_controller()
    assert node.errors and not node.sent
    assert node.pd_timer.cancelled and node.ik_timer.cancelled
    future = SimpleNamespace(done=lambda: True, cancelled=lambda: False,
                             exception=lambda: None, result=lambda: SimpleNamespace(ok=False))
    assert not confirm_position_stop(node, future)


def test_walking_sigterm_uses_confirmed_shutdown_path(walking, monkeypatch):
    import signal
    events = []
    robot = SimpleNamespace(stop_controller=lambda: events.append('stop'),
                            destroy_node=lambda: events.append('destroy'))
    monkeypatch.setattr(walking, 'InverseKinematics', lambda **kwargs: robot)
    monkeypatch.setattr(sys, 'argv', ['walking.py'])
    walking.rclpy.init = lambda **kwargs: None
    walking.rclpy.shutdown = lambda: events.append('shutdown')
    def spin(node):
        signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
    walking.rclpy.spin = spin
    previous = signal.getsignal(signal.SIGTERM)
    walking.main()
    assert events == ['stop', 'destroy', 'shutdown']
    assert signal.getsignal(signal.SIGTERM) is previous
