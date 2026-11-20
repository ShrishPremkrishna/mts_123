#!/usr/bin/env python3
"""Record Pi host thermals every ten seconds for one owned controller manager."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import select
import subprocess
import time


def sysfs_value(path):
    try:
        return path.read_text().strip()
    except OSError:
        return None


def sensor(pattern, attribute, name):
    return next((p for p in Path("/sys/class").glob(pattern)
                 if sysfs_value(p / attribute) == name), None)


def number(directory, attribute, scale=1):
    raw = sysfs_value(directory / attribute) if directory else None
    if raw is None:
        return None
    try:
        value = int(raw)
    except ValueError:
        return None
    return value if scale == 1 else value / scale


def firmware_flags():
    try:
        value = subprocess.check_output(
            ["vcgencmd", "get_throttled"], text=True, timeout=1,
            stderr=subprocess.DEVNULL,
        ).strip()
        label, flags = value.split("=", 1)
        return hex(int(flags, 16)) if label == "throttled" else None
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def record(run):
    session = json.loads((run / "owned_session.json").read_text())
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if session["boot_id"] != boot or session["status"] != "running":
        raise SystemExit("This session is not running on the current boot.")
    manager = session["processes"]["manager"]
    pid = manager["pid"]
    fd = os.pidfd_open(pid)
    try:
        poll = select.poll()
        poll.register(fd, select.POLLIN)
        proc = Path("/proc") / str(pid)
        ticks = (proc / "stat").read_text().rsplit(")", 1)[1].split()[19]
        exe = os.readlink(proc / "exe")
        if (ticks != manager["start_ticks"] or exe != manager["executable"]
                or Path(exe).name != "ros2_control_node" or poll.poll(0)):
            raise SystemExit("Controller manager identity does not match this session.")
        cpu = sensor("thermal/thermal_zone*", "type", "cpu-thermal")
        fan = sensor("hwmon/hwmon*", "name", "pwmfan")
        cooling = sensor("thermal/cooling_device*", "type", "pwm-fan")
        path = run / f"pi-thermal-{pid}-{ticks}.jsonl"
        with path.open("x", buffering=1) as output:
            while not poll.poll(0):
                started = time.monotonic_ns()
                row = {
                    "utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                    "monotonic_ns": started, "boot_id": boot,
                    "manager_pid": pid, "manager_start_ticks": ticks,
                    "cpu_celsius": number(cpu, "temp", 1000),
                    "firmware_throttled_flags": firmware_flags(),
                    "fan_rpm": number(fan, "fan1_input"),
                    "fan_state": number(cooling, "cur_state"),
                    "fan_max_state": number(cooling, "max_state"),
                }
                if poll.poll(0):
                    break
                output.write(json.dumps(row, separators=(",", ":")) + "\n")
                remaining_ns = started + 10_000_000_000 - time.monotonic_ns()
                if poll.poll(max(0, (remaining_ns + 999_999) // 1_000_000)):
                    break
    finally:
        os.close(fd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="Directory containing owned_session.json")
    record(parser.parse_args().run)
