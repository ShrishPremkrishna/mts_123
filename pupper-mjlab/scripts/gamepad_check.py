#!/usr/bin/env python3
"""Live terminal check of a gamepad's sticks, triggers, d-pad and buttons.

Reads the Linux joystick device (/dev/input/jsN) directly, using only the
Python standard library, so it runs on the Pupper Pi without ROS or extra
packages. Axis and button names come from the kernel's axis/button maps,
so they are right for a DualSense and sensible for most other pads.

Usage:
  python3 gamepad_check.py               # auto-pick the first gamepad
  python3 gamepad_check.py /dev/input/js0
  python3 gamepad_check.py --once        # print one snapshot and exit

If the controller disconnects, the screen says so and the tool reconnects
by itself when the controller comes back (press the PS button).
"""

from __future__ import annotations

import argparse
import array
import errno
import fcntl
import glob
import os
import select
import struct
import sys
import time

# --- Linux joystick API (linux/joystick.h) ---------------------------------

JS_EVENT_FMT = "IhBB"  # u32 time (ms), s16 value, u8 type, u8 number
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

ABS_CNT = 0x40
BTN_MAP_LEN = 0x2FF - 0x100 + 1  # KEY_MAX - BTN_MISC + 1


def _ioc(direction: int, nr: int, size: int) -> int:
    return (direction << 30) | (size << 16) | (ord("j") << 8) | nr


_IOC_READ = 2
JSIOCGAXES = _ioc(_IOC_READ, 0x11, 1)
JSIOCGBUTTONS = _ioc(_IOC_READ, 0x12, 1)
JSIOCGAXMAP = _ioc(_IOC_READ, 0x32, ABS_CNT)
JSIOCGBTNMAP = _ioc(_IOC_READ, 0x34, BTN_MAP_LEN * 2)


def JSIOCGNAME(length: int) -> int:
    return _ioc(_IOC_READ, 0x13, length)


# Axis codes (linux/input-event-codes.h) -> label, and whether it is a trigger.
AXIS_NAMES = {
    0x00: ("LX", False),
    0x01: ("LY", False),
    0x02: ("L2", True),
    0x03: ("RX", False),
    0x04: ("RY", False),
    0x05: ("R2", True),
    0x09: ("Gas", True),
    0x0A: ("Brake", True),
    0x10: ("DpadX", False),
    0x11: ("DpadY", False),
}

# Button codes -> PlayStation-style labels.
BUTTON_NAMES = {
    0x130: "Cross",
    0x131: "Circle",
    0x132: "C",
    0x133: "Triangle",
    0x134: "Square",
    0x135: "Z",
    0x136: "L1",
    0x137: "R1",
    0x138: "L2",
    0x139: "R2",
    0x13A: "Create",
    0x13B: "Options",
    0x13C: "PS",
    0x13D: "L3",
    0x13E: "R3",
    0x220: "DpadUp",
    0x221: "DpadDown",
    0x222: "DpadLeft",
    0x223: "DpadRight",
}

BAR_WIDTH = 20


# --- Device handling ---------------------------------------------------------


class Pad:
    def __init__(self, path: str):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        try:
            self.name = self._name()
            self.n_axes = self._u8(JSIOCGAXES)
            self.n_buttons = self._u8(JSIOCGBUTTONS)
            axmap = self._map(JSIOCGAXMAP, "B", ABS_CNT)
            btnmap = self._map(JSIOCGBTNMAP, "H", BTN_MAP_LEN)
        except OSError:
            os.close(self.fd)
            raise
        self.axis_labels = []
        self.axis_trigger = []
        for i in range(self.n_axes):
            code = axmap[i] if axmap else -1
            label, trig = AXIS_NAMES.get(code, (f"A{i}", False))
            self.axis_labels.append(label)
            self.axis_trigger.append(trig)
        self.button_labels = []
        for i in range(self.n_buttons):
            code = btnmap[i] if btnmap else -1
            self.button_labels.append(BUTTON_NAMES.get(code, f"B{i}"))
        self.axes = [0] * self.n_axes
        self.buttons = [0] * self.n_buttons

    def _name(self) -> str:
        buf = array.array("B", [0] * 128)
        fcntl.ioctl(self.fd, JSIOCGNAME(len(buf)), buf, True)
        return buf.tobytes().split(b"\0", 1)[0].decode(errors="replace") or "Unknown"

    def _u8(self, req: int) -> int:
        buf = array.array("B", [0])
        fcntl.ioctl(self.fd, req, buf, True)
        return buf[0]

    def _map(self, req: int, typecode: str, n: int):
        buf = array.array(typecode, [0] * n)
        try:
            fcntl.ioctl(self.fd, req, buf, True)
        except OSError:
            return None
        return buf

    def read_events(self) -> int:
        """Drain pending events. Returns count of non-init events.

        Raises OSError (e.g. ENODEV) if the device went away.
        """
        count = 0
        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE * 64)
            except BlockingIOError:
                return count
            if not data:
                raise OSError(errno.ENODEV, "device closed")
            for off in range(0, len(data) - JS_EVENT_SIZE + 1, JS_EVENT_SIZE):
                _, value, etype, number = struct.unpack_from(JS_EVENT_FMT, data, off)
                if not etype & JS_EVENT_INIT:
                    count += 1
                kind = etype & ~JS_EVENT_INIT
                if kind == JS_EVENT_AXIS and number < self.n_axes:
                    self.axes[number] = value
                elif kind == JS_EVENT_BUTTON and number < self.n_buttons:
                    self.buttons[number] = value

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass


def device_name(path: str) -> str:
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return ""
    try:
        buf = array.array("B", [0] * 128)
        fcntl.ioctl(fd, JSIOCGNAME(len(buf)), buf, True)
        return buf.tobytes().split(b"\0", 1)[0].decode(errors="replace")
    except OSError:
        return ""
    finally:
        os.close(fd)


def find_device() -> str | None:
    """First /dev/input/js* that is a real pad (not motion sensors / touchpad)."""
    paths = sorted(glob.glob("/dev/input/js*"), key=lambda p: int(p[13:] or 0))
    for path in paths:
        name = device_name(path).lower()
        if name and "motion" not in name and "touchpad" not in name:
            return path
    return paths[0] if paths else None


# --- Rendering ---------------------------------------------------------------


def centered_bar(value: int) -> str:
    pos = round((value + 32767) / 65534 * BAR_WIDTH)
    pos = max(0, min(BAR_WIDTH, pos))
    cells = ["-"] * (BAR_WIDTH + 1)
    cells[BAR_WIDTH // 2] = "|"
    cells[pos] = "#"
    return "[" + "".join(cells) + "]"


def trigger_bar(value: int) -> str:
    fill = round((value + 32767) / 65534 * BAR_WIDTH)
    fill = max(0, min(BAR_WIDTH, fill))
    return "[" + "#" * fill + " " * (BAR_WIDTH - fill) + "]"


def render(pad: Pad, rate: float) -> list[str]:
    lines = [
        f"Connected: {pad.name} at {pad.path} — {pad.n_axes} axes, {pad.n_buttons} buttons",
        "",
        f"{pad.name}  ({pad.path})   events/s: {rate:5.0f}   Ctrl-C to quit",
        "",
    ]
    for label, trig, value in zip(pad.axis_labels, pad.axis_trigger, pad.axes):
        bar = trigger_bar(value) if trig else centered_bar(value)
        lines.append(f"{label:>8} {value:+7d}  {bar}")
    pressed = [l for l, v in zip(pad.button_labels, pad.buttons) if v]
    lines.append("")
    lines.append("  pressed: " + (" ".join(pressed) if pressed else "-"))
    lines.append(
        "  " + " ".join(f"{l}:{'X' if v else '.'}" for l, v in zip(pad.button_labels, pad.buttons))
    )
    return lines


class Screen:
    def __init__(self, tty: bool):
        self.tty = tty

    def __enter__(self):
        if self.tty:
            sys.stdout.write("\033[?25l\033[2J")  # hide cursor, clear
        return self

    def __exit__(self, *exc):
        if self.tty:
            sys.stdout.write("\033[?25h\n")  # show cursor
            sys.stdout.flush()

    def draw(self, lines: list[str]) -> None:
        if self.tty:
            out = "\033[H" + "".join(line + "\033[K\n" for line in lines) + "\033[J"
        else:
            out = "\n".join(lines) + "\n"
        sys.stdout.write(out)
        sys.stdout.flush()


# --- Main loop ---------------------------------------------------------------


def open_pad(requested: str | None) -> Pad | None:
    path = requested or find_device()
    if not path or not os.path.exists(path):
        return None
    try:
        return Pad(path)
    except OSError:
        return None


def run(requested: str | None, once: bool, fps: float) -> int:
    if once:
        pad = open_pad(requested)
        if pad is None:
            print(f"No gamepad found at {requested or '/dev/input/js*'}", file=sys.stderr)
            return 1
        time.sleep(0.2)  # let the kernel deliver the initial-state events
        pad.read_events()
        print("\n".join(render(pad, 0)))
        pad.close()
        return 0

    pad: Pad | None = None
    period = 1.0 / fps
    events_in_window = 0
    window_start = time.monotonic()
    rate = 0.0
    with Screen(sys.stdout.isatty()) as screen:
        while True:
            if pad is None:
                pad = open_pad(requested)
                if pad is None:
                    where = requested or "/dev/input/js*"
                    screen.draw([
                        f"Waiting for gamepad at {where} ...   Ctrl-C to quit",
                        "",
                        "  Turn the controller on (press PS). Paired pads reconnect on their own.",
                    ])
                    time.sleep(0.5)
                    continue
                events_in_window, window_start, rate = 0, time.monotonic(), 0.0

            try:
                select.select([pad.fd], [], [], period)
                events_in_window += pad.read_events()
            except OSError:
                pad.close()
                pad = None
                continue

            now = time.monotonic()
            if now - window_start >= 1.0:
                rate = events_in_window / (now - window_start)
                events_in_window, window_start = 0, now
            screen.draw(render(pad, rate))
            if not screen.tty:
                time.sleep(1.0)  # piped output: one snapshot per second


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("device", nargs="?", help="joystick device, e.g. /dev/input/js0 (default: auto)")
    parser.add_argument("--once", action="store_true", help="print one snapshot and exit")
    parser.add_argument("--fps", type=float, default=30.0, help="screen refresh rate (default 30)")
    args = parser.parse_args()
    try:
        return run(args.device, args.once, max(1.0, args.fps))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
