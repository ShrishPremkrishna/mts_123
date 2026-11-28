"""imu_level.py: accel-bias calibration from a still capture, refusals, the at-rest check, the driver log line."""
import json
import math
import os
import random
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "imu"))

import imu_level as L  # noqa: E402

# mts 2026-11-27, level on the stand, published (base-frame) accel median and orientation (10.7 deg tilt)
MTS_BASE_ACC = (-2.096, -0.336, 12.035)


def quat_from_up(u):
    """A quaternion whose body-frame world-up is u (unit): rotation about u x z."""
    ux, uy, uz = u
    ang = math.acos(max(-1.0, min(1.0, uz)))
    ax, ay = uy, -ux
    n = math.hypot(ax, ay) or 1.0
    s = math.sin(ang / 2)
    return (s * ax / n, s * ay / n, 0.0, math.cos(ang / 2))


def make(acc=MTS_BASE_ACC, n=2500, acc_noise=0.012, gyro=0.0015, seed=1, up=None):
    r = random.Random(seed)
    if up is None:
        g = math.sqrt(sum(a * a for a in acc))
        up = tuple(a / g for a in acc)
    q = quat_from_up(up)
    out = []
    for _ in range(n):
        a = [acc[i] + r.gauss(0, acc_noise) for i in range(3)]
        w = [r.gauss(0, gyro) for _ in range(3)]
        out.append(q + tuple(a) + tuple(w))
    return out


def test_mts_case_bias_on_chip_x_and_corrects_to_level():
    rec = L.analyse(make())
    b = rec["accel_bias_chip"]
    assert b[0] == pytest.approx(3.05, abs=0.05)          # chip X carries the fault
    assert abs(b[1] + 0.336) < 0.02 and abs(b[2]) < 0.15
    assert rec["raw_accel_norm"] == pytest.approx(12.22, abs=0.02)
    assert rec["raw_tilt_deg"] == pytest.approx(10.0, abs=0.2)
    assert "sensor bias" in rec["diagnosis"]
    # what the v4 driver publishes after subtracting the bias: level, |a| = g
    chip = L.chip_from_base(MTS_BASE_ACC)
    corr = L.base_from_chip([chip[i] - b[i] for i in range(3)])
    assert L.tilt_from_vec(corr) < 0.2
    assert math.sqrt(sum(c * c for c in corr)) == pytest.approx(L.G, abs=0.02)


def test_frames_round_trip():
    v = (1.0, -2.0, 3.0)
    assert L.base_from_chip(L.chip_from_base(v)) == pytest.approx(v)
    # level body: gravity up along base z = (+0.707, 0, -0.707) g in the chip frame (Nov 24 stand ~ (6.66, -0.14, -7.09))
    assert L.chip_from_base((0, 0, 9.81)) == pytest.approx((6.9367, 0.0, -6.9367), abs=1e-3)


def test_recalibration_adds_to_the_applied_bias():
    """Running with bias b applied, the published accel is already corrected; the new bias must still be b."""
    first = L.analyse(make())["accel_bias_chip"]
    chip = L.chip_from_base(MTS_BASE_ACC)
    published = L.base_from_chip([chip[i] - first[i] for i in range(3)])
    again = L.analyse(make(acc=published), applied_bias=tuple(first))["accel_bias_chip"]
    assert again == pytest.approx(first, abs=0.01)


def test_refuses_motion():
    with pytest.raises(L.Refused, match="moving"):
        L.analyse(make(gyro=0.08))


def test_refuses_noisy_accel():
    with pytest.raises(L.Refused, match="noisy"):
        L.analyse(make(acc_noise=0.4))


def test_refuses_outliers():
    s = make()
    for i in range(0, len(s), 10):          # 10 % glitch samples
        s[i] = s[i][:4] + (5.0, 0.0, 1.0) + s[i][7:]
    with pytest.raises(L.Refused, match="outliers"):
        L.analyse(s)


def test_refuses_stall_marker_short_capture_and_not_upright():
    s = make()
    s[5] = s[5][:4] + (0.0, 0.0, 0.0) + s[5][7:]
    with pytest.raises(L.Refused, match="stall"):
        L.analyse(s)
    with pytest.raises(L.Refused, match="samples"):
        L.analyse(make(n=50))
    with pytest.raises(L.Refused, match="upright"):
        L.analyse(make(acc=(6.0, 0.0, 7.7)))          # ~38 deg


def test_rotation_like_offset_is_diagnosed_but_still_zeroed():
    th = math.radians(10)
    rec = L.analyse(make(acc=(-9.81 * math.sin(th), 0.0, 9.81 * math.cos(th))))
    assert "mount rotation" in rec["diagnosis"]


def test_check_flags_tilt_over_limit():
    bad = L.check(make(), 6.0)
    assert not bad["ok"] and bad["tilt_deg"] == pytest.approx(10.0, abs=0.3) and "imu-level" in bad["error"]
    good = L.check(make(acc=(0.1, 0.0, 9.81)), 6.0)
    assert good["ok"] and good["tilt_deg"] < 1.0
    assert L.check([], 6.0)["ok"] is None          # no verdict without samples


def test_driver_log_line():
    assert L.applied_bias_from_log("FC_IMU_FILTER on: kp=2\n") == (None, None)
    text = ("FC_IMU_CAL accel_bias_chip=(0.0000,0.0000,0.0000) m/s^2 not applied; source none (x not found)\n"
            "FC_IMU_CAL accel_bias_chip=(3.0530,-0.3360,-0.0900) m/s^2 APPLIED; source /home/pi/fc-v1/run/imu_cal.json\n")
    b, applied = L.applied_bias_from_log(text)
    assert applied and b == pytest.approx((3.053, -0.336, -0.09))
    b, applied = L.applied_bias_from_log(text.splitlines()[0])
    assert applied is False and b == (0.0, 0.0, 0.0)


def test_write_cal_keeps_previous(tmp_path):
    p = tmp_path / "imu_cal.json"
    L.write_cal(str(p), {"accel_bias_chip": [1, 0, 0]})
    L.write_cal(str(p), {"accel_bias_chip": [2, 0, 0]})
    assert json.loads(p.read_text())["accel_bias_chip"] == [2, 0, 0]
    assert json.loads((tmp_path / "imu_cal.prev.json").read_text())["accel_bias_chip"] == [1, 0, 0]


def test_file_format_matches_the_driver_parser():
    """The driver's parser (fc_imu_cal.hpp) finds '"accel_bias_chip": [' then 3 numbers before ']'."""
    rec = L.analyse(make())
    text = json.dumps(rec, indent=1)
    seg = text[text.index('"accel_bias_chip"'):]
    body = seg[seg.index("[") + 1:seg.index("]")]
    assert len([float(x) for x in body.split(",")]) == 3


def test_stage_sh_keeps_operator_imu_params():
    """imu_params.json (ImuHealth overrides) lives in ~/fc-v1/robot; stage.sh's rsync --delete wiped it."""
    stage = open(os.path.join(HERE, "..", "stage.sh")).read()
    assert "--exclude /imu_params.json" in stage
