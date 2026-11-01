"""Simulator timing inputs only: no ROS, network, physics stepping or hardware."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location("native_timing_under_test", Path(__file__).resolve().parents[1] / "scripts/native_timing.py")
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)


def profile(**changes):
    value = {"schema": "native_timing_replay_v1", "controller": "trick",
             "use_recorded_cadence": True, "use_recorded_imu_age": True,
             "inference_intervals_s": [.020, .024, .018],
             "imu_age_s": [.003, .009, .018], "source": {"sha256": "a" * 64}}
    value.update(changes)
    return value


def test_disabled_cadence_preserves_native_counter_and_disabled_age_is_zero():
    r = timing.RecordedTiming(profile(use_recorded_cadence=False, use_recorded_imu_age=False))
    for t in (0, .49, .5, .519, 2):
        assert r.inference_due(t, .5) is None
        assert r.imu_age_s() == 0
    receipt = r.record_inference(.5, .5)
    assert receipt["scheduled_elapsed_s"] is None and receipt["schedule_roundup_s"] is None


def test_cyclic_cadence_uses_cumulative_absolute_schedule_and_bounded_tick_roundup():
    r = timing.RecordedTiming(profile())
    intervals = profile()["inference_intervals_s"]
    ages = profile()["imu_age_s"]
    actual = []
    for tick in range(520):
        elapsed = tick / 520
        if r.inference_due(elapsed, .5):
            i = len(actual)
            assert r.imu_age_s() == ages[i % len(ages)]
            receipt = r.record_inference(elapsed, .5)
            expected = .5 + sum(intervals[n % len(intervals)] for n in range(i))
            assert receipt["timing_sample_index"] == i
            assert receipt["scheduled_elapsed_s"] == pytest.approx(expected)
            assert -1e-12 <= receipt["schedule_roundup_s"] < 1 / 520 + 1e-12
            assert not r.inference_due(elapsed, .5)
            actual.append(elapsed)
    assert len(actual) > 15
    r.reset()
    assert r.index == 0 and r.next_offset_s == 0
    assert not r.inference_due(.499, .5) and r.inference_due(.5, .5)
    assert r.imu_age_s() == ages[0]


def test_age_only_arm_advances_ages_without_enabling_cadence_override():
    r = timing.RecordedTiming(profile(use_recorded_cadence=False))
    for i in range(7):
        assert r.inference_due(.5 + i / 52, .5) is None
        assert r.imu_age_s() == [.003, .009, .018][i % 3]
        r.record_inference(.5 + i / 52, .5)


def test_imu_samples_are_coherent_past_only_and_copied():
    q0, w0 = [1, 0, 0, 0], [0, 0, 0]
    history = timing.IMUHistory(.05, q0, w0)
    q0[0] = 99
    q1, w1 = [.9, .1, 0, 0], [1, 2, 3]
    history.append(0, [1, 0, 0, 0], [0, 0, 0])
    history.append(.01, q1, w1)
    history.append(.02, [.8, .2, 0, 0], [4, 5, 6])
    q1[0] = 99; w1[0] = 99
    stamp, q, w = history.sample(.025, .012)
    assert stamp == .01 and q == [.9, .1, 0, 0] and w == [1, 2, 3]
    q[0] = 99; w[0] = 99
    assert history.sample(.025, .012) == (.01, [.9, .1, 0, 0], [1, 2, 3])
    assert history.sample(0, .03)[1] == [1, 0, 0, 0]
    assert history.sample(.02, 0)[0] == .02


def test_growing_requested_age_retains_history_after_recent_zero_age_queries():
    history = timing.IMUHistory(.05, [1, 0, 0, 0], [0, 0, 0])
    for tick in range(201):
        t = tick / 520
        history.append(t, [1, 0, 0, 0], [t, -t, 0])
        assert history.sample(t, 0)[0] == t
        stamp, _, w = history.sample(t, .05)
        assert stamp <= t - .05 + 1e-12
        if t >= .05:
            assert stamp >= t - .05 - 1 / 520 - 1e-12
        if stamp >= 0:
            assert w == [stamp, -stamp, 0]
    assert len(history.rows) < 40
    with pytest.raises(ValueError, match="backwards"):
        history.append(.1, [1, 0, 0, 0], [0, 0, 0])


@pytest.mark.parametrize("age", [-.001, .051, float("nan"), float("inf")])
def test_invalid_imu_age_refused(age):
    history = timing.IMUHistory(.05, [1, 0, 0, 0], [0, 0, 0])
    with pytest.raises(ValueError):
        history.sample(0, age)


@pytest.mark.parametrize("patch", [
    {"schema": "other"}, {"controller": "default"},
    {"use_recorded_cadence": 1}, {"use_recorded_imu_age": "yes"},
    {"inference_intervals_s": [.02]}, {"imu_age_s": [.003, .005]},
    {"inference_intervals_s": [.02, 0, .02]},
    {"inference_intervals_s": [.02, .101, .02]},
    {"inference_intervals_s": [.02, True, .02]},
    {"imu_age_s": [.003, float("nan"), .018]},
    {"imu_age_s": [.003, float("inf"), .018]},
    {"imu_age_s": [.003, -.001, .018]}, {"imu_age_s": [.003, .251, .018]},
    {"source": {}}, {"source": {"sha256": True}},
    {"source": {"sha256": ["a"]}}, {"source": {"sha256": "not-a-sha256"}},
    {"source": {"sha256": "g" * 64}},
])
def test_invalid_timing_profile_refused(patch):
    with pytest.raises(ValueError):
        timing.validate_timing_profile(profile(**patch))


def test_profile_validation_preserves_all_valid_samples_and_source_identity():
    value = profile(); before = deepcopy(value)
    assert timing.validate_timing_profile(value) == before
    assert value == before
