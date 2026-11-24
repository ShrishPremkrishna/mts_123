"""SH-2 calibration command bytes and the bias/frozen analysis in imu_test."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "imu"))

import bno_reader  # noqa: E402
import imu_test  # noqa: E402


@pytest.fixture
def rd(monkeypatch):
    written = []
    monkeypatch.setattr(bno_reader.os, "write", lambda fd, b: written.append(bytes(b)) or len(b))
    r = bno_reader.BNOReader(opener=lambda: 99)
    r.open()
    r.written = written
    return r


def test_configure_me_cal_bytes(rd):
    rd.configure_me_calibration(accel=1, gyro=False, mag=1)
    pkt = rd.written[-1]
    assert pkt[:3] == bytes([16, 0, 2])  # length 16, channel 2
    assert pkt[4:] == bytes([0xF2, 0, 0x07, 1, 0, 1, 0x00, 0, 0, 0, 0, 0])
    assert rd.sent[-1][1:] == (2, pkt.hex())


def test_get_me_cal_and_clear_dcd_bytes(rd, monkeypatch):
    monkeypatch.setattr(bno_reader.time, "sleep", lambda s: None)
    monkeypatch.setattr(rd, "drain", lambda s: None)
    rd.query_me_calibration()
    assert rd.written[-1][4:] == bytes([0xF2, 0, 0x07, 0, 0, 0, 0x01, 0, 0, 0, 0, 0])
    rd.clear_dcd_and_reset()
    assert rd.written[-1][4:] == bytes([0xF2, 1, 0x0B] + [0] * 9)


def test_refuses_other_commands(rd):
    for cmd in (0x03, 0x04, 0x06, 0x09):  # tare, initialize, save DCD, DCD auto-save
        with pytest.raises(ValueError):
            rd.send_command(cmd)
    assert rd.written == []


def test_parse_me_cal_response(rd):
    payload = bytes([0xF1, 3, 0x07, 0, 0, 0, 1, 0, 1, 0] + [0] * 6)
    rd.handle_control(1.0, payload)
    t, kind, d = rd.events[-1]
    assert kind == "me_calibration"
    assert (d["status"], d["accel_dcal"], d["gyro_dcal"], d["mag_dcal"]) == (0, 1, 0, 1)


def test_bias_dither_is_not_calibration_fault():
    lsb = imu_test.GYRO_LSB
    ts = [i * 0.01 for i in range(18000)]  # 3 min at 100 Hz
    # bias dithers +-3 LSB around 1 LSB, per-minute median stable
    vs = [(0.0, 0.0, 0.0, lsb * (1 + (3 if i % 50 == 0 else 0) - (3 if i % 70 == 0 else 0)), 0.0, 0.0)
          for i in range(18000)]
    s = imu_test.summarize_bno({0x07: (ts, vs)}, 180)
    assert s["gyro_bias_estimate"]["x"]["span"] > 0.01
    assert s["gyro_bias_estimate"]["x"]["drift"] == 0
    assert s["verdict"] == "clean_this_run"
    # a real walk of the bias (8 LSB over the run) is still flagged
    vs2 = [(0.0, 0.0, 0.0, lsb * int(8 * i / 18000), 0.0, 0.0) for i in range(18000)]
    assert imu_test.summarize_bno({0x07: (ts, vs2)}, 180)["verdict"] == "calibration"


def test_identical_runs():
    ts = [i * 0.01 for i in range(10)]
    vs = [(1,), (2,), (2,), (2,), (3,), (3,), (4,), (5,), (5,), (5,)]
    assert imu_test.identical_runs(ts, vs) == {"max_samples": 3, "max_ms": 20.0}


def test_rv_vs_accel_agrees_for_level_identity():
    ts = [i * 0.1 for i in range(200)]
    acc = (ts, [(0.0, 0.0, 9.81)] * 200)
    rv = (ts, [(0.0, 0.0, 0.0, 1.0)] * 200)
    assert imu_test.rv_vs_accel(acc, rv)["all"] == 0.0
