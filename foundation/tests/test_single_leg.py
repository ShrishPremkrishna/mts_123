"""Run the contact-pause controller against fake ROS sensors and publishers."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from test_position_controllers import walking, sample
from implicit_compliance.streaming import ContactPauseClock

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def single_leg(walking):
    spec = importlib.util.spec_from_file_location('single_leg_under_test', ROOT / 'ik_heuristic_walking_lab/ik.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.time = SimpleNamespace(monotonic=lambda: 10.)
    return module


def test_full_triangle_bounded_and_reachable(single_leg):
    node = single_leg.InverseKinematics()
    q = np.zeros(3)
    for phase in np.linspace(0, 3, 121):
        target = node.interpolate_triangle(phase)
        q = single_leg.inverse_kinematics(single_leg.fr_leg_fk, target, q,
                strict=True, max_iterations=150, tolerance=1e-4, joint_limits=single_leg.LEG_LIMITS[0])
        assert np.linalg.norm(single_leg.fr_leg_fk(q) - target) <= 1e-4


def test_contact_prediction_consumed_once_and_phase_pauses(single_leg):
    node = single_leg.InverseKinematics()
    node.listener_callback(sample())
    node.contact_clock = ContactPauseClock()
    node.pending_contact = .95
    node.ik_timer_callback()
    assert node.pending_contact is None
    # Repeated control ticks without new inference must not invent a second sample.
    single_leg.time.monotonic = lambda: 10.05
    node.ik_timer_callback()
    assert node.contact_clock.armed
    node.pending_contact = .96
    single_leg.time.monotonic = lambda: 10.1
    node.ik_timer_callback()
    paused = node.t
    assert not node.contact_clock.armed
    single_leg.time.monotonic = lambda: 10.2
    node.listener_callback(sample())
    node.ik_timer_callback()
    assert node.t == paused
    single_leg.time.monotonic = lambda: 11.2
    node.listener_callback(sample())
    node.ik_timer_callback()
    assert node.t > paused


def test_single_leg_stale_sensor_stops_without_zero_pose(single_leg):
    node = single_leg.InverseKinematics()
    node.listener_callback(sample())
    node.ik_timer_callback()
    node.pd_timer_callback()
    assert len(node.sent) == 1
    single_leg.time.monotonic = lambda: 10.3
    node.pd_timer_callback()
    assert node.faulted and node.target_joint_positions is None
    assert len(node.sent) == 1
    assert node.requests[-1].deactivate_controllers == ['forward_command_controller']


def test_missing_effort_stops_contact_enabled_controller(single_leg):
    node = single_leg.InverseKinematics()
    node.contact_predictor = SimpleNamespace(push=lambda *a: None)
    msg = sample()
    msg.effort = []
    node.listener_callback(msg)
    assert node.faulted and not node.sent
