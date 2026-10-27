"""Torque-controller boundary tests with fake ROS transport."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from test_position_controllers import walking, sample

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'pd_control_lab'))


@pytest.fixture
def pd(walking):
    def declare(self, name, value):
        if not hasattr(self, 'parameters'):
            self.parameters = {}
        self.parameters[name] = value
    walking.Node.declare_parameter = declare
    walking.Node.get_parameter = lambda self, key: SimpleNamespace(value=self.parameters[key])
    spec = importlib.util.spec_from_file_location('pd_under_test', ROOT / 'pd_control_lab/pd_control.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.time = SimpleNamespace(monotonic=lambda: 10.)
    return module


@pytest.mark.parametrize('failure', ['nan', 'missing', 'stale'])
def test_sensor_fault_zeroes_torque_and_cannot_auto_resume(pd, failure):
    node = pd.JointStateSubscriber()
    node.get_joint_info(sample())
    node.control_loop()
    if failure == 'stale':
        pd.time.monotonic = lambda: 10.3
        node.control_loop()
    else:
        msg = sample()
        if failure == 'nan':
            msg.position[5] = float('nan')
        else:
            msg.velocity.clear()
        node.get_joint_info(msg)
    assert node.faulted and node.sent[-1].data == [0., 0., 0.]
    node.get_joint_info(sample())
    node.control_loop()
    assert node.sent[-1].data == [0., 0., 0.]


def test_nonfinite_output_is_zero_and_invalid_target_faults(pd):
    node = pd.JointStateSubscriber()
    node.publish_torque(float('nan'))
    assert node.sent[-1].data == [0., 0., 0.]
    node.get_joint_info(sample())
    node.parameters['target_amplitude'] = -1.
    node.control_loop()
    assert node.faulted and node.sent[-1].data == [0., 0., 0.]
