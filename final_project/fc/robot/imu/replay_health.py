#!/usr/bin/env python3
"""Replay a recorded session through ImuHealth and print its timeline.

Input: an .npz made by the audit's analysis/extract.py from a session MCAP
(arrays imu [t, qx,qy,qz,qw, wx,wy,wz, stamp] and cmd [t, vx, vy, wz]).
cmd_zero = no non-zero /cmd_vel in the last 0.5 s (the mux timeout).

  python3 replay_health.py ks4.npz [--every 30] [--param bad_tilt_deg=10 ...]

Prints level transitions, a per-window table and one JSON line of totals.
Needs numpy only to load the .npz.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from imu_health import ImuHealth  # noqa: E402


def replay(path, params=None, step=0.5):
    d = np.load(path, allow_pickle=True)
    imu = d["imu"]
    cmd = d["cmd"] if "cmd" in d.files else np.zeros((0, 4))
    t0 = imu[0, 0]
    tc = cmd[:, 0] - t0
    nz = np.abs(cmd[:, 1:4]).sum(1) > 1e-4
    nz_t = tc[nz]
    h = ImuHealth(params or {}, clock=lambda: 0.0)
    rows, next_s, j = [], 0.0, 0
    for r in imu:
        t = r[0] - t0
        while j < len(nz_t) and nz_t[j] <= t:
            j += 1
        cmd_zero = not (j > 0 and t - nz_t[j - 1] < 0.5)
        h.update(t, r[1:5], r[5:8], cmd_zero)
        if t >= next_s:
            s = h.summary(now=t)
            s["t"] = round(t, 2)
            s["cmd_zero"] = cmd_zero
            rows.append(s)
            next_s += step
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--every", type=float, default=30.0, help="table row period, s")
    ap.add_argument("--param", action="append", default=[], help="key=value override")
    a = ap.parse_args()
    params = {k: float(v) for k, v in (kv.split("=", 1) for kv in a.param)}
    rows = replay(a.npz, params)
    print("transitions:")
    prev = None
    for s in rows:
        if s["level"] != prev:
            print("  %7.1f s  %-4s  %s" % (s["t"], s["level"], "; ".join(s["reasons"])))
            prev = s["level"]
    print("window   worst  tilt_max  density_max  bias_min   (per %.0f s)" % a.every)
    k = 0
    while k < len(rows):
        w0 = rows[k]["t"]
        chunk = [s for s in rows[k:] if s["t"] < w0 + a.every]
        k += len(chunk)
        worst = max(chunk, key=lambda s: ("ok", "warn", "bad").index(s["level"]))["level"]
        print("%6.0f   %-4s   %6.1f     %5.2f      %+.3f" % (
            w0, worst, max(s["tilt_deg"] for s in chunk),
            max(s["pulse_density"] for s in chunk), min(s["bias_x"] for s in chunk)))
    tot = {lv: round(sum(0.5 for s in rows if s["level"] == lv), 1) for lv in ("ok", "warn", "bad")}
    first = {lv: next((s["t"] for s in rows if s["level"] == lv), None) for lv in ("warn", "bad")}
    print(json.dumps({"file": os.path.basename(a.npz), "seconds_at_level": tot, "first": first}))


if __name__ == "__main__":
    main()
