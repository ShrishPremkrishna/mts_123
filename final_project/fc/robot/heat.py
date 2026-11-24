#!/usr/bin/env python3
"""Heat sources on the Pi, read-only: CPU governor/frequency, fan, top processes by CPU.

    python3 heat.py [--top N] [--interval S]   -> one JSON line {"cpu":..., "fan":..., "top":[...]}

Reads /proc and /sys only (no device opened, nothing written). The top list samples every process's
utime+stime twice, INTERVAL apart (default 0.5 s), so a busy process shows its current rate, not a
lifetime average (what `ps -o pcpu` reports). pct is per core, like top: 100 = one full core.
Used by `fcctl.sh status` ("heat") and shown by `python -m fc.ctl pi`.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

CPU0 = "devices/system/cpu/cpu0/cpufreq"
FAN_DEV = "class/thermal/cooling_device0"
FAN_HWMON = "devices/platform/cooling_fan/hwmon"
INTERPRETERS = ("python", "python3", "bash", "sh", "node", "ruby", "perl")


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def _int(text):
    try:
        return int(text)
    except (TypeError, ValueError):
        return None


def cpu_info(sys_root="/sys"):
    d = os.path.join(sys_root, CPU0)
    khz = {k: _int(_read(os.path.join(d, "scaling_%s_freq" % k))) for k in ("cur", "min", "max")}
    return {"governor": _read(os.path.join(d, "scaling_governor")),
            **{k + "_mhz": (v // 1000 if v is not None else None) for k, v in khz.items()},
            "cores": os.cpu_count()}


def fan_info(sys_root="/sys"):
    d = os.path.join(sys_root, FAN_DEV)
    if _read(os.path.join(d, "type")) is None:
        return None
    rpm = None
    hw = os.path.join(sys_root, FAN_HWMON)
    try:
        for name in sorted(os.listdir(hw)):
            rpm = _int(_read(os.path.join(hw, name, "fan1_input")))
            if rpm is not None:
                break
    except OSError:
        pass
    return {"type": _read(os.path.join(d, "type")), "state": _int(_read(os.path.join(d, "cur_state"))),
            "max_state": _int(_read(os.path.join(d, "max_state"))), "rpm": rpm}


def parse_stat(text):
    """/proc/PID/stat -> (comm, utime+stime ticks). comm may contain spaces and parens."""
    a, b = text.index("("), text.rindex(")")
    rest = text[b + 2:].split()
    return text[a + 1:b], int(rest[11]) + int(rest[12])      # fields 14, 15 (utime, stime)


def label(comm, argv):
    """Short process name: argv[0] basename, plus the script for interpreters (python3 x.py -> python3 x.py)."""
    if not argv:
        return comm
    base = os.path.basename(argv[0])
    if base.startswith(INTERPRETERS) and len(argv) > 1:
        script = next((a for a in argv[1:] if not a.startswith("-")), None)
        if script:
            return ("%s %s" % (base, os.path.basename(script)))[:48]
    return (base or comm)[:48]


def snapshot(proc="/proc"):
    """{pid: (ticks, label)} for every readable process."""
    out = {}
    for pid in os.listdir(proc):
        if not pid.isdigit():
            continue
        st = _read(os.path.join(proc, pid, "stat"))
        if not st:
            continue
        try:
            comm, ticks = parse_stat(st)
        except (ValueError, IndexError):
            continue
        try:
            with open(os.path.join(proc, pid, "cmdline"), "rb") as f:
                argv = [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
        except OSError:
            argv = []
        out[int(pid)] = (ticks, label(comm, argv))
    return out


def top(before, after, dt, n=5, clk=100):
    """Busiest processes between two snapshots, pct per core. Pure."""
    rows = []
    for pid, (t1, name) in after.items():
        t0 = before.get(pid, (None,))[0]
        if t0 is None or dt <= 0:
            continue
        pct = 100.0 * (t1 - t0) / (clk * dt)
        if pct > 0:
            rows.append({"pid": pid, "pct": round(pct, 1), "cmd": name})
    rows.sort(key=lambda r: (-r["pct"], r["pid"]))
    return rows[:n]


def sample(n=5, interval=0.5, proc="/proc", sys_root="/sys"):
    clk = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
    t0, a = time.monotonic(), snapshot(proc)
    time.sleep(interval)
    t1, b = time.monotonic(), snapshot(proc)
    me = os.getpid()
    b.pop(me, None)
    return {"cpu": cpu_info(sys_root), "fan": fan_info(sys_root), "top": top(a, b, t1 - t0, n, clk),
            "top_interval_s": interval}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--top", type=int, default=5)
    ap.add_argument("--interval", type=float, default=0.5)
    a = ap.parse_args(argv)
    print(json.dumps(sample(a.top, a.interval), separators=(",", ":")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
