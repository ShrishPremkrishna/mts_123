"""Lab 2 geometry checked independently against MuJoCo, and ROS packet behavior."""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
LEGS = ["front_r", "front_l", "back_r", "back_l"]
NAMES = [f"leg_{leg}_{i}" for leg in LEGS for i in (1, 2, 3)]


@pytest.fixture
def fk():
    # Execute the actual class without importing ROS or creating a live node.
    path = ROOT / "forward_kinematics_lab/forward_kinematics.py"
    definition = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    clock = SimpleNamespace(monotonic=lambda: 10.)
    namespace = {"np": np, "Node": object, "LEGS": LEGS, "time": clock}
    exec(compile(ast.Module(body=[definition], type_ignores=[]), str(path), "exec"), namespace)
    cls = namespace["ForwardKinematics"]
    instance = cls.__new__(cls)
    instance.joint_positions = instance.last_joint_state_time = None
    return instance, clock


def test_all_four_fk_match_independent_robot_model_across_joint_limits(fk):
    from mjlab.tasks.tracking.config.pupper import motion_design as md
    node, _ = fk
    k = md.kinematics()
    functions = [node.fk_front_right, node.fk_front_left, node.fk_back_right, node.fk_back_left]
    rng = np.random.default_rng(123)
    for _ in range(100):
        q = rng.uniform(k.lower, k.upper)
        expected = k.feet(np.zeros(3), np.array([1., 0, 0, 0]), q)
        for i, (leg, function) in enumerate(zip(LEGS, functions)):
            # Lab angles/offsets are rounded relative to the URDF quaternions.
            np.testing.assert_allclose(function(*q[3*i:3*i+3]), expected[leg], atol=4e-6)


@pytest.mark.parametrize("bad", ["missing", "duplicate", "short", "nonfinite"])
def test_bad_packets_invalidate_previous_pose_without_crashing(fk, bad):
    node, _ = fk
    valid = SimpleNamespace(name=NAMES.copy(), position=[0.] * 12)
    node.listener_callback(valid)
    assert node.joint_positions is not None
    if bad == "missing": valid.name[0] = "unrelated"
    elif bad == "duplicate": valid.name[0] = valid.name[1]
    elif bad == "short": valid.position.pop()
    else: valid.position[4] = np.nan
    node.listener_callback(valid)
    assert node.joint_positions is None and node.last_joint_state_time is None
    node.timer_callback()  # No publisher access when invalid.


def test_reordered_position_only_packet_and_stale_state(fk):
    node, clock = fk
    node.listener_callback(SimpleNamespace(name=NAMES[::-1], position=list(range(12))))
    np.testing.assert_array_equal(node.joint_positions["front_r"], [11, 10, 9])
    assert node.last_joint_state_time == 10
    clock.monotonic = lambda: 10.251
    node.timer_callback()  # No publisher access once the pose becomes stale.
