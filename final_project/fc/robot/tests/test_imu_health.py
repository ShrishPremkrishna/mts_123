"""Tests for lane D: ImuHealth, the BNO reader's SH-2 parsing and imu_test summaries.

Run: .venv/bin/python -m pytest final_project/fc/robot/tests/test_imu_health.py
The ks4/ks6 replays need the audit's extracted recordings (FC_IMU_NPZ_DIR); they
are skipped when absent.
"""
import json
import math
import os
import struct
import subprocess
import sys

import pytest

ROBOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROBOT)
sys.path.insert(0, os.path.join(ROBOT, "imu"))

from imu_health import ImuHealth, raw_axes, tilt_deg  # noqa: E402
import bno_reader  # noqa: E402
import imu_test  # noqa: E402

NPZ_DIR = os.environ.get(
    "FC_IMU_NPZ_DIR",
    "/private/tmp/claude-501/-Users-shrish--claude-spares-spare-KtfIa4/"
    "aad0fdc8-5407-448d-92f9-73d18c7992ca/scratchpad/analysis")
LEVEL = (0.0, 0.0, 0.0, 1.0)
S = math.sqrt(0.5)


def quat_roll(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def base_gyro_from_raw_x(rx):
    """A rate on the chip's X axis, expressed in the base frame (mount pitch -135 deg)."""
    return (-S * rx, 0.0, S * rx)


def feed(h, t0, t1, quat, gyro, cmd_zero=True, hz=250, jitter=True):
    t, k = t0, 0
    while t < t1:
        q = quat if not jitter else (quat[0] + 1e-6 * (k % 3), quat[1], quat[2], quat[3])
        h.update(t, q, gyro(t) if callable(gyro) else gyro, cmd_zero)
        t += 1.0 / hz
        k += 1
    return t


# ---------------------------------------------------------------- geometry
def test_raw_axes_isolates_chip_x():
    rx, ry, rz = raw_axes(base_gyro_from_raw_x(-0.2))
    assert rx == pytest.approx(-0.2)
    assert ry == pytest.approx(0.0) and rz == pytest.approx(0.0, abs=1e-12)
    # a pure base yaw rate shows on raw X and raw Z equally: not isolated
    rx, _, rz = raw_axes((0.0, 0.0, 0.3))
    assert abs(rx) == pytest.approx(abs(rz))


def test_tilt_deg():
    assert tilt_deg(LEVEL) == pytest.approx(0.0)
    assert tilt_deg(quat_roll(12)) == pytest.approx(12.0, abs=1e-6)
    yaw = (0.0, 0.0, math.sin(0.6), math.cos(0.6))
    assert tilt_deg(yaw) == pytest.approx(0.0, abs=1e-6)


# ---------------------------------------------------------------- levels
def test_quiet_is_ok():
    h = ImuHealth()
    t = feed(h, 0, 12, LEVEL, (0, 0, 0))
    s = h.summary(now=t)
    assert s["level"] == "ok" and s["bad_cause"] is None
    assert set(s) >= {"tilt_deg", "pulse_density", "bias_x", "stale_ms", "level"}


def test_no_samples_is_warn():
    assert ImuHealth().summary()["level"] == "warn"


def test_dense_raw_x_pulses_warn_not_bad():
    h = ImuHealth()
    # 30 ms pulse every 100 ms on chip X -> ~30% of samples
    t = feed(h, 0, 12, quat_roll(1.0), lambda t: base_gyro_from_raw_x(-0.1 if (t % 0.1) < 0.03 else 0.0))
    s = h.summary(now=t)
    assert s["level"] == "warn"
    assert 0.2 < s["pulse_density"] < 0.4
    assert s["bias_x"] == pytest.approx(0.0, abs=1e-9)  # median still at the quiet level


def test_real_rotation_is_not_counted_as_pulse():
    h = ImuHealth()
    t = feed(h, 0, 12, LEVEL, (0.0, 0.0, -0.2))  # base yaw: all chip axes move
    assert h.summary(now=t)["pulse_density"] == 0.0


def test_held_tilt_at_zero_command_is_bad():
    h = ImuHealth()
    t = feed(h, 0, 5, LEVEL, (0, 0, 0))
    t = feed(h, t, t + 2.5, quat_roll(12), (0, 0, 0))
    s = h.summary(now=t)
    assert s["level"] == "bad" and s["bad_cause"] == "tilt"


def test_short_tilt_like_a_kick_is_not_bad():
    h = ImuHealth()
    t = feed(h, 0, 5, LEVEL, (0, 0, 0))
    t = feed(h, t, t + 1.3, quat_roll(22), (0, 0, 0))
    t = feed(h, t, t + 1.0, LEVEL, (0, 0, 0))
    assert h.summary(now=t)["level"] != "bad"


def test_tilt_while_commanded_is_not_bad():
    h = ImuHealth()
    t = feed(h, 0, 5, quat_roll(12), (0, 0, 0), cmd_zero=False)
    assert h.summary(now=t)["level"] == "ok"


def test_bad_holds_then_clears():
    h = ImuHealth({"hold_s": 3.0})
    t = feed(h, 0, 3, quat_roll(15), (0, 0, 0))
    t = feed(h, t, t + 1.0, LEVEL, (0, 0, 0))
    assert h.summary(now=t)["level"] == "bad"
    t = feed(h, t, t + 3.0, LEVEL, (0, 0, 0))
    assert h.summary(now=t)["level"] == "ok"


def test_frozen_but_arriving_stream_is_bad():
    """RCA 2026-11-24: messages kept arriving while the values froze ~0.5 s, before the robot seized."""
    h = ImuHealth()
    t = feed(h, 0, 2, LEVEL, (0, 0, 0))
    t = feed(h, t, t + 0.3, quat_roll(1), (0.0, 0, 0), jitter=False)
    s = h.summary(now=t)
    assert s["stale_ms"] < 10 and s["level"] == "warn" and "frozen" in s["reasons"][0]
    t = feed(h, t, t + 0.3, quat_roll(1), (0.0, 0, 0), jitter=False)   # identical, at zero command
    s = h.summary(now=t)
    assert s["frozen_ms"] > 500 and s["level"] == "bad" and s["bad_cause"] == "frozen"


def test_changing_accel_means_not_frozen():
    h = ImuHealth()
    t, k = 0.0, 0
    while t < 1.0:                                              # quat+gyro identical, accel noise
        h.update(t, LEVEL, (0.0, 0.0, 0.0), True, (0.0, 0.0, 9.81 + 1e-3 * (k % 2)))
        t += 0.004
        k += 1
    assert h.summary(now=t)["level"] == "ok"
    t2 = t
    while t < t2 + 0.6:                                          # accel freezes too -> bad
        h.update(t, LEVEL, (0.0, 0.0, 0.0), True, (0.0, 0.0, 9.81))
        t += 0.004
    assert h.summary(now=t)["bad_cause"] == "frozen"


def test_staleness_is_time_since_arrival():
    h = ImuHealth()
    t = feed(h, 0, 1, LEVEL, (0, 0, 0))
    assert h.summary(now=t + 0.1)["level"] == "ok"
    assert h.summary(now=t + 0.3)["level"] == "warn"
    s = h.summary(now=t + 0.6)
    assert s["level"] == "bad" and s["bad_cause"] == "stale" and s["stale_ms"] > 500
    assert "no IMU message" in s["reasons"][0]


def test_arrival_gap_is_reported():
    """A starved callback (late burst after a pause) shows as gap_max_ms, then ages out."""
    h = ImuHealth()
    t = feed(h, 0, 1, LEVEL, (0, 0, 0))
    t = feed(h, t + 0.3, t + 1.3, LEVEL, (0, 0, 0))
    assert h.summary(now=t)["gap_max_ms"] == pytest.approx(300, abs=10)
    t = feed(h, t, t + 11, LEVEL, (0, 0, 0))
    assert h.summary(now=t)["gap_max_ms"] == 0.0


def test_missing_updates_go_stale_on_wall_clock():
    clock = [100.0]
    h = ImuHealth(clock=lambda: clock[0])
    feed(h, 0, 1, LEVEL, (0, 0, 0))
    clock[0] += 0.3
    assert h.summary()["level"] == "warn"
    clock[0] += 0.5
    assert h.summary()["level"] == "bad"


def test_params_prefix_override():
    h = ImuHealth({"imu_bad_tilt_deg": 20.0})
    t = feed(h, 0, 3, quat_roll(15), (0, 0, 0))
    assert h.summary(now=t)["level"] == "warn"


# ---------------------------------------------------------------- recordings
def _npz(name):
    path = os.path.join(NPZ_DIR, name)
    if not os.path.exists(path):
        pytest.skip("recording %s not available" % name)
    pytest.importorskip("numpy")
    return path


def test_ks4_bad_stretch_flagged_and_ks6_mild_episode_not_bad():
    from replay_health import replay
    ks4 = replay(_npz("ks4.npz"))
    bad4 = [r["t"] for r in ks4 if r["level"] == "bad"]
    assert bad4 and 500 < bad4[0] < 560            # first held 10+ deg zero-command tilt
    assert all(r["level"] != "bad" for r in ks4 if r["t"] < 500)
    assert any(r["level"] == "warn" for r in ks4 if r["t"] < 60)   # pulses present from boot
    ks6 = replay(_npz("ks6.npz"))
    assert not [r for r in ks6 if r["bad_cause"] == "tilt"]
    assert all(r["level"] != "bad" for r in ks6 if 450 < r["t"] < 1020)  # the mild episode
    assert sum(r["level"] == "warn" for r in ks6 if 480 < r["t"] < 1020) > 600  # flagged as warn


def test_imu_test_replay_prints_one_json_line():
    path = _npz("ks4.npz")
    out = subprocess.run([sys.executable, os.path.join(ROBOT, "imu_test.py"), "--mode", "replay",
                          "--npz", path, "--start", "480", "--seconds", "60"],
                         capture_output=True, text=True, check=True).stdout.strip().splitlines()
    assert len(out) == 1
    s = json.loads(out[0])
    assert s["verdict"] == "pulsing_unseparated"
    assert s["gyro_calibrated"]["x"]["neg_per_min"] > 100
    assert s["gyro_calibrated"]["y"]["neg_per_min"] < 10


# ---------------------------------------------------------------- SH-2 parsing
def rec(rid, seq, vals, q, size):
    raw = [int(round(v * 2 ** q)) for v in vals]
    body = bytes([rid, seq, 3, 0]) + struct.pack("<%dh" % len(raw), *raw)
    return body + bytes(size - len(body))


def test_parse_batch_with_uncal_gyro_and_accel():
    payload = (bytes([0xFB, 0, 0, 0, 0]) + rec(0x07, 5, [-0.1, 0.0, 0.01, -0.02, 0.0, 0.0], 9, 16)
               + rec(0x01, 6, [0.0, 0.0, 9.81], 8, 10))
    out = bno_reader.parse_input_batch(payload)
    assert [r[0] for r in out] == [0x07, 0x01]
    assert out[0][3][0] == pytest.approx(-0.1, abs=2 ** -9) and out[0][3][3] == pytest.approx(-0.02, abs=2 ** -9)
    assert out[1][3][2] == pytest.approx(9.81, abs=2 ** -8)


def test_parse_batch_stops_on_unknown():
    out = bno_reader.parse_input_batch(bytes([0xFB, 0, 0, 0, 0, 0x77, 1, 2, 3]))
    assert out == [("unknown", 0x77)]


def test_read_packet_follows_continuation_protocol():
    payload = bytes([0xFB, 0, 0, 0, 0]) + rec(0x07, 1, [0.1, 0, 0, 0, 0, 0], 9, 16) * 3  # 53 bytes
    length = len(payload) + 4
    reads = [bytes([length & 0xFF, length >> 8, 3, 9])]  # header-only read
    remaining, seq, off = len(payload), 9, 0
    while remaining:
        n = min(remaining, 28)
        seq += 1
        reads.append(bytes([(remaining + 4) & 0xFF, 0x80 | ((remaining + 4) >> 8), 3, seq]) + payload[off:off + n])
        off += n
        remaining -= n
    rd = bno_reader.BNOReader()
    it = iter(reads)
    rd._read = lambda n: next(it)
    ch, got = rd.read_packet()
    assert ch == 3 and got == payload


def test_set_feature_refuses_unknown_reports():
    rd = bno_reader.BNOReader()
    with pytest.raises(ValueError):
        rd.set_feature(0xF9, 1000)


# ---------------------------------------------------------------- verdicts
def _synthetic(kind, seconds=60, hz=100):
    ts, vs, acc = [], [], []
    for k in range(seconds * hz):
        t = k / hz
        pulse = -0.15 if (t % 0.25) < 0.03 else 0.0
        bias_x = -0.004
        if kind == "hardware":
            raw_x = bias_x + pulse
        elif kind == "calibration":
            bias_x = -0.004 + 0.03 * t / seconds   # chip's bias estimate wanders
            raw_x = -0.004
        else:
            raw_x = bias_x
        ts.append(t)
        vs.append((raw_x, 0.0, 0.002, bias_x, 0.0, 0.002))
        acc.append((0.01 * (k % 2), 0.0, 9.81))
    return {0x07: (ts, vs), 0x01: (ts, acc)}


@pytest.mark.parametrize("kind,expected", [("hardware", "hardware_or_electrical"),
                                           ("calibration", "calibration"), ("clean", "clean_this_run")])
def test_bno_verdicts(kind, expected):
    s = imu_test.summarize_bno(_synthetic(kind), 60)
    assert s["verdict"] == expected, s["why"]
    assert s["accel"]["still"] is True


# ---------------------------------------------------------------- heading-hold overlay
def test_policy_hh0_edits_only_kp(tmp_path):
    import heading_hold
    src = tmp_path / "p.json"
    src.write_text('{"use_imu": true, "gait_reference": {"a": [1, 2]}, "weights": [0.1, 2.5e-07],\n'
                   ' "heading_hold": {"kp": 1.5, "clip": 1.0, "yaw_threshold": 0.1, "walk_threshold": 0.05}}\n')
    r = heading_hold.policy_hh0(str(src), str(tmp_path / "o.json"))
    out = (tmp_path / "o.json").read_text()
    assert json.loads(out)["heading_hold"] == {"kp": 0.0, "clip": 1.0, "yaw_threshold": 0.1, "walk_threshold": 0.05}
    assert out.startswith('{"use_imu": true, "gait_reference": {"a": [1, 2]}, "weights": [0.1, 2.5e-07],')
    assert r["sha256"] != r["source_sha256"]


def test_policy_hh0_adds_block_when_defaulted(tmp_path):
    import heading_hold
    src = tmp_path / "p.json"
    src.write_text('{"gait_reference": {}, "x": 1}')
    heading_hold.policy_hh0(str(src), str(tmp_path / "o.json"))
    assert json.loads((tmp_path / "o.json").read_text())["heading_hold"]["kp"] == 0.0
