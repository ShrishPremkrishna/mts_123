#!/usr/bin/env python3
"""imu_level.py — measure and store the BNO085 accelerometer bias (fcctl.sh imu-level / ready's IMU check).

Why (fc/notes/rca-20261127-tilt.md): after service, mts read ~10 deg tilt while sitting level on its stand.
The published accel was |a| = 12.2 m/s^2 with only the chip-X component off (+3.3 m/s^2 against Nov 24).
A rotation cannot change |a|, so the fix is a chip-frame accel bias, applied in the v4 driver before the
attitude filter (fc_imu_cal.hpp). The walker's projected gravity and the bridge's tilt checks then both
see level.

  imu_level.py level [--seconds 10] [--dry-run] [--out PATH]
      Robot sitting level on the stand, legs off, stack running. Captures the published IMU, refuses if
      the robot moves or the accel is noisy, computes bias = raw_chip - R_mount^T (0, 0, g), and writes
      ~/fc-v1/run/imu_cal.json (outside the rsync'd tree: stage.sh never deletes it). The previous file
      is kept as imu_cal.prev.json. The driver loads it at stack START: restart the stack to apply.
  imu_level.py check [--seconds 2] [--max-deg 6]
      Tilt from the published orientation (what the bridge and walker see). ok=false above --max-deg.
  imu_level.py show
      The stored file and the bias the running driver logged as applied.

One JSON line on stdout. Pure helpers are tested in tests/test_imu_level.py.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import sys
import time

G = 9.81
MOUNT_PITCH_DEG = -135.0        # URDF imu_sensor pitch -2.35619 (pupper_v3_description components.xacro)
# Same file the v4 driver reads ($FC_IMU_CAL, else $HOME/fc-v1/run/imu_cal.json); FC_ROOT relocates it for tests.
DEFAULT_CAL = os.path.expanduser(os.environ.get("FC_IMU_CAL") or
                                 os.path.join(os.environ.get("FC_ROOT") or "~/fc-v1", "run", "imu_cal.json"))
MAX_BIAS = 6.0                  # m/s^2 per component; the driver rejects larger (fc_imu_cal.hpp)
LIMITS = {
    "min_samples": 200,         # ~250 Hz published; 10 s gives ~2500
    "gyro_median_max": 0.03,    # rad/s |w| median: above = the robot is moving
    "gyro_p95_max": 0.10,       # rad/s |w| 95th percentile
    "accel_sigma_max": 0.15,    # m/s^2 robust spread (1.4826 * MAD) per axis
    "outlier_frac_max": 0.05,   # samples > 1 m/s^2 from the median
    "raw_tilt_max_deg": 30.0,   # uncorrected tilt above this: not upright on the stand
}
_CAL_LINE = re.compile(r"FC_IMU_CAL accel_bias_chip=\(([-0-9.eE+]+),([-0-9.eE+]+),([-0-9.eE+]+)\) m/s\^2 (APPLIED|not applied)")


class Refused(Exception):
    pass


# ---------------------------------------------------------------- pure helpers
def _ry(p_deg):
    p = math.radians(p_deg)
    return math.cos(p), math.sin(p)


def chip_from_base(v, mount_pitch_deg=MOUNT_PITCH_DEG):
    """Published (base) vector -> chip frame. The driver publishes base = Ry(p) chip."""
    c, s = _ry(mount_pitch_deg)
    x, y, z = v
    return (c * x - s * z, y, s * x + c * z)


def base_from_chip(v, mount_pitch_deg=MOUNT_PITCH_DEG):
    c, s = _ry(mount_pitch_deg)
    x, y, z = v
    return (c * x + s * z, y, -s * x + c * z)


def tilt_from_vec(v):
    n = math.sqrt(sum(a * a for a in v))
    return float("nan") if n <= 0 else math.degrees(math.acos(max(-1.0, min(1.0, v[2] / n))))


def quat_tilt_deg(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n <= 0:
        return float("nan")
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y) / n))))


def applied_bias_from_log(text):
    """Bias the running driver logged at start: (bias tuple, applied bool), or (None, None) when the driver
    printed no FC_IMU_CAL line (pre-v4 driver: no bias applied, a stored file has no effect)."""
    found = None
    for m in _CAL_LINE.finditer(text):
        found = m
    if found is None:
        return None, None
    b = tuple(float(found.group(i)) for i in (1, 2, 3))
    applied = found.group(4) == "APPLIED"
    return (b if applied else (0.0, 0.0, 0.0)), applied


def _robust_sigma(xs):
    med = statistics.median(xs)
    return 1.4826 * statistics.median(abs(x - med) for x in xs), med


def analyse(samples, applied_bias=(0.0, 0.0, 0.0), limits=None, g=G):
    """samples: [(qx,qy,qz,qw, ax,ay,az, wx,wy,wz)] from the published topic (base frame).
    Returns the calibration record; raises Refused(reason) when the capture is not a clean still level one."""
    lim = dict(LIMITS, **(limits or {}))
    if len(samples) < lim["min_samples"]:
        raise Refused("only %d IMU samples (need %d): is the stack running?" % (len(samples), lim["min_samples"]))
    if any(s[4] == 0.0 and s[5] == 0.0 and s[6] == 0.0 for s in samples):
        raise Refused("driver stall marker (accel exactly 0) in the capture: IMU not streaming cleanly")
    w = [math.sqrt(s[7] ** 2 + s[8] ** 2 + s[9] ** 2) for s in samples]
    w_med = statistics.median(w)
    w_p95 = sorted(w)[int(0.95 * (len(w) - 1))]
    if w_med > lim["gyro_median_max"] or w_p95 > lim["gyro_p95_max"]:
        raise Refused("robot moving (|gyro| median %.3f, p95 %.3f rad/s): keep it still on the stand" % (w_med, w_p95))
    chip = [chip_from_base(s[4:7]) for s in samples]
    raw = [tuple(c[i] + applied_bias[i] for i in range(3)) for c in chip]   # undo what the driver subtracted
    med, sig = [], []
    for i in range(3):
        sd, m = _robust_sigma([r[i] for r in raw])
        med.append(m)
        sig.append(sd)
    if max(sig) > lim["accel_sigma_max"]:
        raise Refused("accel noisy (robust sigma %s m/s^2): robot touched or vibrating" % [round(x, 3) for x in sig])
    out = sum(1 for r in raw if math.sqrt(sum((r[i] - med[i]) ** 2 for i in range(3))) > 1.0) / len(raw)
    if out > lim["outlier_frac_max"]:
        raise Refused("%.0f%% accel outliers (> 1 m/s^2 off the median): robot bumped or sensor glitching" % (100 * out))
    raw_base = base_from_chip(med)
    raw_tilt = tilt_from_vec(raw_base)
    if raw_tilt > lim["raw_tilt_max_deg"]:
        raise Refused("uncorrected tilt %.1f deg: robot not upright on the stand" % raw_tilt)
    expected = chip_from_base((0.0, 0.0, g))
    bias = [med[i] - expected[i] for i in range(3)]
    if max(abs(b) for b in bias) > MAX_BIAS:
        raise Refused("bias %s m/s^2 exceeds %.0f: not plausible, check the robot is level" % ([round(b, 3) for b in bias], MAX_BIAS))
    norm = math.sqrt(sum(m * m for m in med))
    diag = ("|a| %.2f m/s^2 is off g by %.2f: a sensor bias (a mount rotation keeps |a| = g)" % (norm, norm - g)
            if abs(norm - g) > 0.3 else
            "|a| %.2f ~ g: the offset could also be a mount rotation; the bias correction zeroes it on the stand" % norm)
    q_tilt = statistics.median(quat_tilt_deg(s[:4]) for s in samples)
    return {
        "accel_bias_chip": [round(b, 4) for b in bias],
        "raw_accel_chip_median": [round(m, 4) for m in med],
        "raw_accel_norm": round(norm, 4),
        "raw_tilt_deg": round(raw_tilt, 2),
        "published_tilt_deg": round(q_tilt, 2),
        "accel_sigma": [round(x, 4) for x in sig],
        "gyro_median": round(w_med, 4),
        "applied_bias_during_capture": list(applied_bias),
        "n": len(samples),
        "mount_pitch_deg": MOUNT_PITCH_DEG,
        "g": g,
        "diagnosis": diag,
    }


def check(samples, max_deg):
    if not samples:   # ok None: no verdict (a missing stream is the bridge's stale guard's job)
        return {"ok": None, "error": "no IMU samples: is the stack running?", "n": 0}
    tilts = sorted(quat_tilt_deg(s[:4]) for s in samples)
    med = statistics.median(tilts)
    ok = med <= max_deg
    rec = {"ok": ok, "tilt_deg": round(med, 2), "tilt_max_deg": round(tilts[-1], 2), "max_deg": max_deg, "n": len(samples)}
    if not ok:
        rec["error"] = ("IMU reads %.1f deg at rest (limit %.1f): IMU calibration off? Put the robot level on the "
                        "stand, legs off, run 'fcctl.sh imu-level', then restart the stack" % (med, max_deg))
    return rec


def write_cal(path, rec):
    d = os.path.dirname(path) or "."
    os.makedirs(d, exist_ok=True)
    if os.path.exists(path):
        os.replace(path, os.path.join(d, "imu_cal.prev.json"))
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(rec, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def active_robot_log(run_dir_file=None):
    run_dir_file = run_dir_file or os.path.join(os.path.dirname(DEFAULT_CAL), "active")
    try:
        with open(run_dir_file) as f:
            lines = f.read().split()
        run = next(x for x in lines if x.startswith("/"))
        with open(os.path.join(run, "robot.log"), errors="replace") as f:
            return f.read()
    except (OSError, StopIteration):
        return ""


# ---------------------------------------------------------------- ROS capture (read-only subscriber)
def capture(seconds, topic):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    rclpy.init()
    node = rclpy.create_node("fc_imu_level")
    got = []

    def cb(m):
        got.append((m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w,
                    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))

    node.create_subscription(Imu, topic, cb, qos_profile_sensor_data)
    end = time.monotonic() + seconds + 2.0     # + discovery
    t_first = None
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if got and t_first is None:
            t_first = time.monotonic()
            end = t_first + seconds
    node.destroy_node()
    rclpy.shutdown()
    return got


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("mode", choices=("level", "check", "show"))
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--max-deg", type=float, default=6.0)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--out", default=DEFAULT_CAL)
    ap.add_argument("--topic", default=os.environ.get("FC_IMU_TOPIC", "/imu_sensor_broadcaster/imu"))
    a = ap.parse_args(argv)
    log_bias, log_applied = applied_bias_from_log(active_robot_log())
    driver = {"driver_logged_cal": log_applied is not None, "driver_bias_applied": log_bias}
    if a.mode == "show":
        try:
            with open(a.out) as f:
                stored = json.load(f)
        except (OSError, ValueError) as e:
            stored = {"error": str(e)}
        print(json.dumps({"ok": True, "file": a.out, "stored": stored, **driver}))
        return 0
    samples = capture(a.seconds or (10.0 if a.mode == "level" else 2.0), a.topic)
    if a.mode == "check":
        rec = check(samples, a.max_deg)
        rec.update(driver)
        print(json.dumps(rec))
        return 0 if rec["ok"] else (2 if rec["ok"] is None else 1)
    try:
        rec = analyse(samples, log_bias or (0.0, 0.0, 0.0))
    except Refused as e:
        print(json.dumps({"ok": False, "action": "refused", "error": str(e), "n": len(samples), **driver}))
        return 1
    rec["measured_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    if not a.dry_run:
        write_cal(a.out, rec)
    note = ("dry run: nothing written" if a.dry_run else
            "saved %s; the driver loads it at stack START: fcctl.sh stop, then start/ready (robot on the stand)" % a.out)
    if log_applied is None:
        note += "; WARNING the running driver logged no FC_IMU_CAL line (pre-v4): install the v4 driver or the bias is ignored"
    print(json.dumps({"ok": True, "file": a.out, "written": not a.dry_run, "note": note, **rec, **driver}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
