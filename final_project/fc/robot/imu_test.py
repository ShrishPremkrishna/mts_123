#!/usr/bin/env python3
"""Discriminating IMU test: is the raw-X gyro fault in the MEMS channel, in the
BNO085's dynamic calibration, or electrical noise from the motors?

Prints exactly one JSON summary line on stdout (fcctl `imu-test <s>` relies on
that); progress goes to stderr; the raw log and summary.json go to --out.

Modes (--mode, default auto):
  bno    Standalone, read-only BNO085 reader on /dev/i2c-1 (imu/bno_reader.py).
         Needs: NO robot stack running (it refuses if ros2_control_node runs,
         if anything holds /dev/i2c-1, or if the driver lock is taken; it holds
         the driver lock while running so no stack can start). Motors are never
         touched (no SPI), so legs are limp: robot on the stand. Logs the
         uncalibrated gyro (0x07: raw rate + the chip's bias estimate),
         accelerometer (0x01), rotation vector (0x05) and raw gyro with
         temperature (0x15). This is the discriminating mode.
  ros    Subscribes to the running stack's /imu_sensor_broadcaster/imu (needs a
         stack up and the profile's ROS env, e.g. ROS_DOMAIN_ID=168 for u6).
         Only calibrated gyro + fused quaternion exist there, so it measures
         the fault but cannot separate calibration from hardware. Use it for the
         "motors holding" condition (legs homed by the stack).
  replay Same analysis as ros on a recorded .npz (audit extract.py format).
  auto   ros if a ros2_control_node is running, else bno.

Tomorrow's sequence (see fc/notes/lane-D-gyro.md):
  python3 imu_test.py --mode bno --seconds 600 --condition unpowered
  python3 imu_test.py --mode bno --seconds 600 --condition battery_on_motors_off
  (start the stack, legs homed, standing on the stand)
  python3 imu_test.py --mode ros --seconds 600 --condition holding
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "imu"))

from imu_health import ImuHealth, raw_axes, tilt_deg  # noqa: E402

DRIVER_LOCK = "/tmp/control_board_hardware.lock"
DEFAULT_REPORTS = "07:100,01:50,05:25,15:25"
PULSE_THR = 0.02            # rad/s below the axis median = pulse sample
PULSING_EVENTS_PER_MIN = 10  # healthy axes show 0-4; kickseen4 boot showed 105
STILL_ACCEL_STD = 0.15      # m/s^2; above this the robot moved during the test


def log(*a):
    print(*a, file=sys.stderr, flush=True)


# ---------------------------------------------------------------- statistics
def axis_stats(ts, xs, thr=PULSE_THR):
    """Pulse statistics of one axis relative to its quiet baseline.

    The baseline is the most frequent value (2 mrad/s bins, about one LSB): the
    pulses are one-sided and spread over many values, so the quiet level stays
    the mode even when pulses dominate the median.
    """
    if not xs:
        return None
    med = statistics.median(xs)
    bins = {}
    for x in xs:
        k = round(x / 0.002)
        bins[k] = bins.get(k, 0) + 1
    base = max(bins.items(), key=lambda kv: kv[1])[0] * 0.002
    neg = [x - base < -thr for x in xs]
    pos = [x - base > thr for x in xs]

    def events(flags):
        return sum(1 for i, f in enumerate(flags) if f and (i == 0 or not flags[i - 1]))

    dur_min = max((ts[-1] - ts[0]) / 60.0, 1e-9)
    s = sorted(xs)
    ne, pe = events(neg), events(pos)
    return {"baseline": round(base, 4), "median": round(med, 4), "p1": round(s[len(s) // 100], 4), "p99": round(s[-1 - len(s) // 100], 4),
            "std": round(statistics.pstdev(xs), 4), "neg_events": ne, "pos_events": pe,
            "neg_per_min": round(ne / dur_min, 1), "pos_per_min": round(pe / dur_min, 1),
            "neg_frac": round(sum(neg) / len(xs), 4)}


def per_minute_median(ts, xs):
    out, t0 = [], ts[0] if ts else 0
    buckets = {}
    for t, x in zip(ts, xs):
        buckets.setdefault(int((t - t0) // 60), []).append(x)
    for k in sorted(buckets):
        out.append(round(statistics.median(buckets[k]), 4))
    return out


def stream_stats(ts):
    if len(ts) < 2:
        return {"n": len(ts)}
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    return {"n": len(ts), "hz": round((len(ts) - 1) / (ts[-1] - ts[0]), 1),
            "max_gap_ms": round(max(gaps) * 1000, 1), "gaps_over_100ms": sum(g > 0.1 for g in gaps)}


def gravity_from_quat(q):
    """Gravity direction in the sensor/body frame for quaternion (x, y, z, w)."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))


def angle_deg(a, b):
    d = sum(i * j for i, j in zip(a, b))
    na = math.sqrt(sum(i * i for i in a)) or 1.0
    nb = math.sqrt(sum(i * i for i in b)) or 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, d / (na * nb)))))


def quat_drift(ts, qs, head_s=5.0):
    """Change of the fused gravity direction from the first head_s to the worst later point."""
    if not qs:
        return None
    g0 = [gravity_from_quat(q) for t, q in zip(ts, qs) if t - ts[0] < head_s]
    ref = tuple(statistics.mean(c) for c in zip(*g0))
    drifts = [angle_deg(ref, gravity_from_quat(q)) for q in qs]
    return {"max_deg": round(max(drifts), 2), "end_deg": round(drifts[-1], 2)}


def verdict(ux, others_pulsing, cal_x, bias_x, still):
    """Decide from the uncalibrated (ux) and calibrated (cal_x) raw-X stats."""
    if still is False:
        return "inconclusive_moved", "accelerometer shows the robot moved; repeat on the stand"
    u_puls = ux is not None and ux["neg_per_min"] >= PULSING_EVENTS_PER_MIN
    c_puls = cal_x is not None and cal_x["neg_per_min"] >= PULSING_EVENTS_PER_MIN
    bias_moving = bias_x is not None and (bias_x["span"] > 0.01 or abs(bias_x["last"]) > 0.02)
    note = " Other axes pulse too: suspect board-level noise or mounting." if others_pulsing else ""
    if u_puls:
        return ("hardware_or_electrical",
                "Raw (uncalibrated) X pulses before any calibration is applied. Compare the "
                "unpowered vs motors-holding runs: pulses also when unpowered = in-chip fault "
                "(replace the IMU/board); only when powered/holding = electrical coupling." + note)
    if ux is None:
        if c_puls:
            return "pulsing_unseparated", "Calibrated X pulses; this mode cannot separate calibration from hardware." + note
        return "clean_this_run", "No calibrated raw-X pulses now." + note
    if c_puls or bias_moving:
        return ("calibration",
                "Raw X is clean but the chip's bias estimate / calibrated output moves: dynamic "
                "calibration fault, fixable in software (re-zero, disable gyro dynamic cal, clear DCD)." + note)
    return "clean_this_run", "No raw-X pulses or bias movement now; the fault is intermittent: repeat warm or longer." + note


def pi_temp_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


# ---------------------------------------------------------------- bno mode
def preflight_bno(bus):
    """Refuse when any robot stack or other I2C user is present."""
    problems = []
    dev = "/dev/i2c-%d" % bus
    me = os.getpid()
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open("/proc/%s/cmdline" % pid, "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "ros2_control_node" in cmd or "persistent_session.py" in cmd:
            problems.append("robot stack running: pid %s %s" % (pid, cmd[:80]))
        try:
            for fd in os.listdir("/proc/%s/fd" % pid):
                if os.readlink("/proc/%s/fd/%s" % (pid, fd)) == dev:
                    problems.append("%s held open by pid %s" % (dev, pid))
                    break
        except OSError:
            pass
    # Same path and flock protocol as the native driver, so the two exclude each
    # other. Never follow a symlink; never unlink (unlinking while another process
    # opens the path would let both "hold" different inodes).
    created = False
    lock = os.open(DRIVER_LOCK, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o644)
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        problems.append("driver lock %s is held (a control_board driver is running)" % DRIVER_LOCK)
        os.close(lock)
        lock = None
    return problems, (lock, created)


def run_bno(a, out_dir):
    from bno_reader import BNOReader, REPORTS, NAME_TO_ID  # noqa: F401
    problems, (lock, created_lock) = preflight_bno(a.bus)
    if problems:
        if lock is not None:
            fcntl.flock(lock, fcntl.LOCK_UN)
            os.close(lock)
        return {"ok": False, "mode": "bno", "error": "refused: " + "; ".join(problems)}
    wanted = []
    for item in a.reports.split(","):
        rid, hz = item.split(":")
        wanted.append((int(rid, 16), float(hz)))
    rd = BNOReader(bus=a.bus)
    data = {rid: ([], []) for rid, _ in wanted}
    unknown = {}
    temps = []
    csv = open(os.path.join(out_dir, "bno.csv"), "w") if out_dir else None
    if csv:
        csv.write("t,report,seq,status,v0,v1,v2,v3,v4,v5\n")
    try:
        rd.open()
        if not a.no_reset:
            log("soft-resetting BNO (as the driver does on start)")
            rd.soft_reset()
        rd.request_product_id()
        time.sleep(0.01)
        rd.drain(0.05)
        for rid, hz in wanted:
            rd.set_feature(rid, int(1e6 / hz))
            time.sleep(0.005)
        if a.query_calib:
            rd.query_me_calibration()
        t_start = time.monotonic()
        rd.t0 = t_start
        t_end = t_start + a.seconds
        next_temp = t_start
        while time.monotonic() < t_end:
            now = time.monotonic()
            if now >= next_temp:
                temps.append((round(now - t_start, 1), pi_temp_c()))
                next_temp += 15.0
            got = rd.poll()
            if got is None:
                time.sleep(0.001)
                continue
            for r in got:
                if r[1] in ("unknown", "truncated"):
                    unknown["0x%02X" % r[2]] = unknown.get("0x%02X" % r[2], 0) + 1
                    continue
                t, rid, seq, status, vals = r
                if rid in data:
                    data[rid][0].append(t - t_start)
                    data[rid][1].append(vals)
                if csv:
                    csv.write("%.4f,%d,%d,%d,%s\n" % (t - t_start, rid, seq, status,
                                                      ",".join("%.6g" % v for v in vals)))
    finally:
        rd.close()
        if csv:
            csv.close()
        if lock is not None:
            fcntl.flock(lock, fcntl.LOCK_UN)
            os.close(lock)
    temps.append((round(a.seconds, 1), pi_temp_c()))
    s = summarize_bno(data, a.seconds)
    s.update({"mode": "bno", "condition": a.condition, "pi_temp_c": temps,
              "unknown_reports": unknown, "invalid_reads": rd.invalid_reads,
              "chip_events": [{"t": round(t - getattr(rd, "t0", 0.0), 2), "kind": k, **d}
                              for t, k, d in rd.events[:20]]})
    return s


def summarize_bno(data, seconds):
    s = {"ok": True, "seconds": seconds, "streams": {}}
    for rid, (ts, _) in data.items():
        s["streams"]["0x%02X" % rid] = stream_stats(ts)
    ux = cal_x = bias_x = None
    others = False
    if data.get(0x07) and data[0x07][0]:
        ts, vs = data[0x07]
        raw = {ax: axis_stats(ts, [v[i] for v in vs]) for i, ax in enumerate("xyz")}
        cal = {ax: axis_stats(ts, [v[i] - v[i + 3] for v in vs]) for i, ax in enumerate("xyz")}
        bias = {}
        for i, ax in enumerate("xyz"):
            b = [v[i + 3] for v in vs]
            bias[ax] = {"first": round(b[0], 4), "last": round(b[-1], 4), "min": round(min(b), 4),
                        "max": round(max(b), 4), "span": round(max(b) - min(b), 4),
                        "changes": sum(1 for p, q in zip(b, b[1:]) if p != q),
                        "per_min": per_minute_median(ts, b)}
        ux, cal_x, bias_x = raw["x"], cal["x"], bias["x"]
        others = any(raw[ax]["neg_per_min"] + raw[ax]["pos_per_min"] >= PULSING_EVENTS_PER_MIN for ax in "yz")
        s["gyro_uncal_raw"] = raw
        s["gyro_calibrated"] = cal
        s["gyro_bias_estimate"] = bias
        s["raw_x_per_min_median"] = per_minute_median(ts, [v[0] for v in vs])
    still = None
    if data.get(0x01) and data[0x01][0]:
        ts, vs = data[0x01]
        std = [statistics.pstdev([v[i] for v in vs]) for i in range(3)]
        mean = [statistics.mean([v[i] for v in vs]) for i in range(3)]
        still = max(std) < STILL_ACCEL_STD
        s["accel"] = {"std": [round(x, 3) for x in std], "mean": [round(x, 3) for x in mean], "still": still}
    if data.get(0x05) and data[0x05][0]:
        ts, vs = data[0x05]
        s["rv_gravity_drift"] = quat_drift(ts, vs)
    if data.get(0x15) and data[0x15][0]:
        ts, vs = data[0x15]
        temp = [v[3] for v in vs]
        s["gyro_raw_counts"] = {ax: {"median": statistics.median([v[i] for v in vs]),
                                     "std": round(statistics.pstdev([v[i] for v in vs]), 2)}
                                for i, ax in enumerate("xyz")}
        s["chip_temp_raw"] = {"first": temp[0], "last": temp[-1], "min": min(temp), "max": max(temp),
                              "per_min": per_minute_median(ts, temp)}
    v, why = verdict(ux, others, cal_x, bias_x, still)
    s["verdict"], s["why"] = v, why
    return s


# ---------------------------------------------------------------- ros / replay
def summarize_base_frame(ts, quats, gyros, cmd_zero=None, seconds=None):
    """Analysis of base-frame IMU samples (ROS topic or recording)."""
    h = ImuHealth({}, clock=lambda: 0.0)
    rx, ry, rz, keep_t = [], [], [], []
    last = None
    for i, (t, q, g) in enumerate(zip(ts, quats, gyros)):
        h.update(t, q, g, True if cmd_zero is None else cmd_zero[i])
        if (q, g) == last:
            continue  # the broadcaster repeats samples; analyse distinct ones
        last = (q, g)
        a = raw_axes(g)
        keep_t.append(t)
        rx.append(a[0]); ry.append(a[1]); rz.append(a[2])
    raw = {"x": axis_stats(keep_t, rx), "y": axis_stats(keep_t, ry), "z": axis_stats(keep_t, rz)}
    others = any(raw[ax]["neg_per_min"] + raw[ax]["pos_per_min"] >= PULSING_EVENTS_PER_MIN for ax in "yz")
    tilts = [tilt_deg(q) for q in quats]
    pulsing = raw["x"]["neg_per_min"] >= PULSING_EVENTS_PER_MIN
    s = {"ok": True, "seconds": seconds if seconds is not None else round(ts[-1] - ts[0], 1),
         "streams": {"imu": stream_stats(keep_t)},
         "gyro_calibrated": raw, "raw_x_per_min_median": per_minute_median(keep_t, rx),
         "tilt_deg": {"first": round(tilts[0], 2), "max": round(max(tilts), 2), "last": round(tilts[-1], 2)},
         "rv_gravity_drift": quat_drift(ts, quats), "health": h.summary(now=ts[-1])}
    if pulsing:
        s["verdict"], s["why"] = ("pulsing_unseparated",
                                  "Calibrated raw-X pulses with the stack running; compare with the bno-mode "
                                  "unpowered run to separate electrical coupling from an in-chip fault.")
    else:
        s["verdict"], s["why"] = "clean_this_run", "No raw-X pulses in this window."
    if others:
        s["why"] += " Other axes pulse too (real motion or board-level noise)."
    return s


def run_ros(a):
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
    except ImportError as e:
        return {"ok": False, "mode": "ros", "error": "rclpy unavailable (source the profile's ROS env): %s" % e}
    rclpy.init()
    node = rclpy.create_node("fc_imu_test")
    ts, qs, gs = [], [], []

    def cb(m):
        o, w = m.orientation, m.angular_velocity
        ts.append(time.monotonic())
        qs.append((o.x, o.y, o.z, o.w))
        gs.append((w.x, w.y, w.z))

    node.create_subscription(Imu, a.topic, cb, qos_profile_sensor_data)
    temps, t0 = [], time.monotonic()
    next_temp = t0
    try:
        while time.monotonic() - t0 < a.seconds:
            if time.monotonic() >= next_temp:
                temps.append((round(time.monotonic() - t0, 1), pi_temp_c()))
                next_temp += 15.0
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if len(ts) < 10:
        return {"ok": False, "mode": "ros", "error": "only %d messages on %s (wrong ROS_DOMAIN_ID?)" % (len(ts), a.topic)}
    ts = [t - t0 for t in ts]
    s = summarize_base_frame(ts, qs, gs, seconds=a.seconds)
    s.update({"mode": "ros", "condition": a.condition, "topic": a.topic, "pi_temp_c": temps})
    return s


def run_replay(a):
    import numpy as np  # replay is a laptop/analysis path; numpy is fine here
    d = np.load(a.npz, allow_pickle=True)  # trusted: our own extract.py output
    imu = d["imu"]
    t = imu[:, 0] - imu[0, 0]
    m = (t >= a.start) & (t < a.start + a.seconds)
    ts = t[m].tolist()
    qs = [tuple(r) for r in imu[m, 1:5].tolist()]
    gs = [tuple(r) for r in imu[m, 5:8].tolist()]
    s = summarize_base_frame(ts, qs, gs)
    s.update({"mode": "replay", "file": os.path.basename(a.npz), "start": a.start, "condition": a.condition})
    return s


def stack_running():
    for pid in os.listdir("/proc") if os.path.isdir("/proc") else []:
        if pid.isdigit():
            try:
                with open("/proc/%s/cmdline" % pid, "rb") as f:
                    if b"ros2_control_node" in f.read():
                        return True
            except OSError:
                pass
    return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--mode", choices=("auto", "bno", "ros", "replay"), default="auto")
    ap.add_argument("--condition", default="unlabeled",
                    help="what was physically true: unpowered | battery_on_motors_off | holding | warm | cold ...")
    ap.add_argument("--out", default=os.path.expanduser("~/fc-v1/imu/runs"),
                    help="parent dir for <UTC>-<mode>-<condition>/ (bno.csv, summary.json); '' disables")
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--reports", default=DEFAULT_REPORTS, help="hexid:Hz list for bno mode")
    ap.add_argument("--no-reset", action="store_true", help="bno: skip the soft reset (keep current chip state)")
    ap.add_argument("--query-calib", action="store_true",
                    help="bno: also ask the chip which dynamic calibrations are enabled (SH-2 get ME cal)")
    ap.add_argument("--topic", default="/imu_sensor_broadcaster/imu")
    ap.add_argument("--npz", help="replay: recording .npz")
    ap.add_argument("--start", type=float, default=0.0, help="replay: start offset, s")
    a = ap.parse_args(argv)

    mode = a.mode
    if mode == "auto":
        mode = "ros" if stack_running() else "bno"
    out_dir = None
    if a.out and mode != "replay":
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        out_dir = os.path.join(a.out, "%s-%s-%s" % (stamp, mode, a.condition))
        try:
            os.makedirs(out_dir, exist_ok=True)
        except OSError as e:
            log("cannot create %s: %s; not logging raw data" % (out_dir, e))
            out_dir = None
    log("imu_test mode=%s seconds=%s condition=%s out=%s" % (mode, a.seconds, a.condition, out_dir))
    try:
        if mode == "bno":
            s = run_bno(a, out_dir)
        elif mode == "ros":
            s = run_ros(a)
        else:
            s = run_replay(a)
    except Exception as e:  # one JSON line no matter what
        s = {"ok": False, "mode": mode, "error": "%s: %s" % (type(e).__name__, e)}
    s["out"] = out_dir
    if out_dir:
        with open(os.path.join(out_dir, "summary.json"), "w") as f:
            json.dump(s, f, indent=1)
    print(json.dumps(s, separators=(",", ":")))
    return 0 if s.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
