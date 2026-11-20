"""Offline scheduling tests; no ROS/Hailo or inference performance claims."""
import importlib.util
import math
import random
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('cadence_safety', ROOT / 'follow_me_pupper/robot/perception_safety.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
DetectionCadence = MODULE.DetectionCadence


@pytest.mark.parametrize('fps', [1, 5, 10, 20])
def test_stable_cadence_keeps_requested_average_and_never_repeats_same_instant(fps):
    cadence = DetectionCadence()
    accepted = []
    for i in range(2001):
        now = i / 200
        if cadence.due(now, fps):
            accepted.append(now)
            assert not cadence.due(now, fps)
    assert len(accepted) == fps * 10 + 1
    assert accepted[-1] == 10


def test_jittered_loop_keeps_phase_instead_of_resetting_to_each_admission():
    rng = random.Random(27)
    cadence = DetectionCadence()
    times = [0] + [i * 0.025 + rng.uniform(-0.004, 0.004) for i in range(1, 401)]
    accepted = [t for t in times if cadence.due(t, 10)]
    assert 100 <= len(accepted) <= 101
    assert max(b - a for a, b in zip(accepted, accepted[1:])) < 0.15


def test_long_stall_admits_only_one_latest_frame_without_catch_up_burst():
    cadence = DetectionCadence()
    assert cadence.due(0, 10)
    assert cadence.due(50.03, 10)
    assert not cadence.due(50.03, 10)
    assert not cadence.due(50.04, 10)
    assert not cadence.due(50.099, 10)
    assert cadence.due(50.1, 10)


def test_rate_changes_rebase_from_last_admission_without_immediate_extra_work():
    cadence = DetectionCadence()
    assert cadence.due(0, 10)
    assert not cadence.due(0.01, 20)
    assert cadence.due(0.05, 20)
    assert not cadence.due(0.06, 5)
    assert not cadence.due(0.24, 5)
    assert cadence.due(0.25, 5)


def test_pause_resume_resets_phase_without_replaying_paused_slots():
    cadence = DetectionCadence()
    assert cadence.due(0, 10)
    assert not cadence.due(0.01, 10, enabled=False)
    assert not cadence.due(100, 10, enabled=False)
    assert cadence.due(100.01, 10)
    assert not cadence.due(100.02, 10)


@pytest.mark.parametrize('now,fps,enabled', [
    (-1, 10, True), (math.nan, 10, True), (math.inf, 10, True),
    (True, 10, True), ('0', 10, True), (1e308, 10, True),
    (0, 0, True), (0, 1e-320, True), (0, -10, True), (0, 21, True),
    (0, math.nan, True), (0, math.inf, True), (0, True, True), (0, 10, 'yes'),
])
def test_invalid_inputs_fail_closed_without_consuming_a_slot(now, fps, enabled):
    cadence = DetectionCadence()
    assert not cadence.due(now, fps, enabled=enabled)
    assert cadence.due(0, 10)


def test_backward_clock_does_not_reset_schedule_or_allow_a_repeat():
    cadence = DetectionCadence()
    assert cadence.due(10, 10)
    assert not cadence.due(9, 10)
    assert not cadence.due(10, 10)
    assert cadence.due(10.1, 10)


def test_recorded_camera_receipts_reproduce_and_remove_scheduling_loss():
    path = ROOT / 'artifacts/pupper-fc/2026-11-17/detection-cadence-review/reproduce.py'
    if not path.exists():
        pytest.skip('local camera evidence is not present in this checkout')
    spec = importlib.util.spec_from_file_location('cadence_reproduction', path)
    repro = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(repro)
    old = repro.simulate(.005)
    new = repro.simulate(.005, cadence=DetectionCadence())
    assert old['source_frames_consumed'] == new['source_frames_consumed'] == 490
    assert old['count'] == 299
    assert new['count'] == 490
    assert 9.9 <= new['rate_hz'] <= 10.1
