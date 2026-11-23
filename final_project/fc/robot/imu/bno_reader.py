"""Read-only standalone BNO085 reader over Linux i2c-dev (stdlib only).

Talks to the BNO085 at /dev/i2c-1, address 0x4B, the same bus/address the
control_board_hardware_interface driver uses (imu_manager.hpp
IMU_I2C_DEVICE_NUMBER=1, rt_bno055.h I2C_ADDR=0x4B). It never opens SPI, so the
motor board is never addressed: motors stay unpowered/unhomed.

Writes go only to the BNO and only these SH-2 messages:
  - soft reset (channel 1, value 1)            the driver does this on every start
  - product ID request (0xF9)                  identity, read-only
  - set feature (0xFD) for sensor reports      what the driver does for 0x05/0x02
  - optional "get ME calibration" (0xF2/0x07)  only with query_calibration=True
It never sends tare, DCD save/clear, calibration configure, initialize or FRS writes.

Packet reads mirror pupper_bno::read_packet in bno_protocol.hpp: a 4-byte
header read (does not consume the packet) followed by continuation reads of at
most 28 payload bytes, each with its own 4-byte continuation header.
"""
from __future__ import annotations

import fcntl
import os
import struct
import time

I2C_SLAVE = 0x0703
BNO_ADDR = 0x4B
CH_EXECUTABLE, CH_CONTROL, CH_REPORTS = 1, 2, 3

# SH-2 input report id -> (record length in bytes, Q point, number of int16 values, name)
REPORTS = {
    0x01: (10, 8, 3, "accel"),            # m/s^2, calibrated, includes gravity
    0x02: (10, 9, 3, "gyro"),             # rad/s, calibrated (= uncal - bias)
    0x03: (10, 4, 3, "mag"),
    0x04: (10, 8, 3, "linacc"),
    0x05: (14, 14, 4, "rv"),              # rotation vector quaternion i,j,k,real (+accuracy)
    0x06: (10, 8, 3, "gravity"),
    0x07: (16, 9, 6, "gyro_uncal"),       # rad/s: raw x,y,z then the chip's bias estimate x,y,z
    0x08: (12, 14, 4, "grv"),             # game rotation vector (no magnetometer)
    0x09: (14, 14, 4, "geomag_rv"),
    0x13: (6, 0, 0, "stability"),
    0x14: (16, 0, 3, "accel_raw"),        # ADC counts (+temperature, timestamp)
    0x15: (16, 0, 4, "gyro_raw"),         # ADC counts x,y,z + temperature (sensor units)
    0x16: (14, 0, 3, "mag_raw"),
}
NAME_TO_ID = {v[3]: k for k, v in REPORTS.items()}
TIMEBASE = {0xFB: 5, 0xFA: 5}


def parse_input_batch(payload: bytes):
    """Parse one channel-3 payload into [(id, seq, status, values tuple)].

    Unknown ids stop parsing (their length is unknown) and are returned as
    ('unknown', id) so the caller can count them.
    """
    out, i = [], 0
    while i < len(payload):
        rid = payload[i]
        if rid in TIMEBASE:
            i += TIMEBASE[rid]
            continue
        spec = REPORTS.get(rid)
        if spec is None:
            out.append(("unknown", rid))
            break
        size, q, nvals, _name = spec
        if i + size > len(payload):
            out.append(("truncated", rid))
            break
        rec = payload[i:i + size]
        seq, status = rec[1], rec[2] & 3
        raw = struct.unpack_from("<%dh" % nvals, rec, 4) if nvals else ()
        scale = 2.0 ** -q if q else 1.0
        vals = tuple(v * scale for v in raw)
        out.append((rid, seq, status, vals))
        i += size
    return out


class BNOReader:
    def __init__(self, bus: int = 1, addr: int = BNO_ADDR, opener=None):
        self.path = "/dev/i2c-%d" % bus
        self.addr = addr
        self._opener = opener
        self.fd = None
        self.seq = [0] * 6
        self.events = []          # (t, kind, detail) for control-channel responses
        self.invalid_reads = 0

    # -- low level ---------------------------------------------------------
    def open(self):
        if self._opener:
            self.fd = self._opener()
            return
        self.fd = os.open(self.path, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, self.addr)

    def close(self):
        if self.fd is not None and not self._opener:
            os.close(self.fd)
        self.fd = None

    def _read(self, n: int) -> bytes:
        try:
            return os.read(self.fd, n)
        except OSError:
            return b""

    def _write_packet(self, channel: int, data: bytes) -> None:
        n = len(data) + 4
        pkt = bytes([n & 0xFF, n >> 8, channel, self.seq[channel] & 0xFF]) + data
        self.seq[channel] = (self.seq[channel] + 1) & 0xFF
        os.write(self.fd, pkt)

    def read_packet(self, budget_s: float = 0.025):
        """Return (channel, payload) or None. Mirrors bno_protocol.hpp read_packet."""
        start = time.monotonic()
        first = self._read(4)
        if len(first) != 4:
            self.invalid_reads += 1
            return None
        length = (first[0] | first[1] << 8) & 0x7FFF
        if length == 0:
            return None
        if length < 4 or length - 4 > 4096 or first[2] > 5:
            self.invalid_reads += 1
            return None
        orphan = bool(first[1] & 0x80)
        remaining, last_seq, payload = length - 4, first[3], bytearray()
        while remaining:
            if time.monotonic() - start > budget_s:
                self.invalid_reads += 1
                return None
            count = min(remaining, 28)
            chunk = self._read(count + 4)
            if len(chunk) != count + 4:
                self.invalid_reads += 1
                return None
            clen = (chunk[0] | chunk[1] << 8) & 0x7FFF
            if clen == 0:
                time.sleep(0.001)
                continue  # wait for the continuation, like the driver
            if clen != remaining + 4 or chunk[2] != first[2] or not (chunk[1] & 0x80) \
                    or chunk[3] != ((last_seq + 1) & 0xFF):
                self.invalid_reads += 1
                return None
            last_seq = chunk[3]
            payload += chunk[4:4 + count]
            remaining -= count
        if orphan:
            self.invalid_reads += 1
            return None
        return first[2], bytes(payload)

    # -- SH-2 operations ---------------------------------------------------
    def drain(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            got = self.read_packet()
            if got is None:
                time.sleep(0.002)
            elif got[0] == CH_CONTROL:
                self.handle_control(time.monotonic(), got[1])

    def soft_reset(self) -> None:
        self._write_packet(CH_EXECUTABLE, bytes([1]))
        time.sleep(0.2)
        self.drain(0.1)
        time.sleep(0.2)
        self.drain(0.1)

    def request_product_id(self) -> None:
        self._write_packet(CH_CONTROL, bytes([0xF9, 0]))

    def set_feature(self, report_id: int, interval_us: int) -> None:
        if report_id not in REPORTS:
            raise ValueError("refusing to enable unknown report 0x%02X" % report_id)
        d = bytearray(17)
        d[0], d[1] = 0xFD, report_id
        struct.pack_into("<I", d, 5, interval_us)
        self._write_packet(CH_CONTROL, bytes(d))

    def query_me_calibration(self) -> None:
        """SH-2 command 0x07, subcommand 0x01 = get ME calibration config (R1..R3 =
        accel/gyro/mag dynamic calibration enabled). Opt-in: see module doc."""
        d = bytearray(12)
        d[0], d[1], d[2] = 0xF2, 0, 0x07
        d[3 + 3] = 0x01  # P3 subcommand: get
        self._write_packet(CH_CONTROL, bytes(d))

    def handle_control(self, t: float, payload: bytes) -> None:
        if not payload:
            return
        rid = payload[0]
        if rid == 0xF8:
            for k in range(0, len(payload) - 15, 16):
                r = payload[k:k + 16]
                part, build = struct.unpack_from("<II", r, 4)
                self.events.append((t, "product_id", {
                    "reset_cause": r[1], "sw": "%d.%d.%d" % (r[2], r[3], struct.unpack_from("<H", r, 12)[0]),
                    "part": part, "build": build}))
        elif rid == 0xF1 and len(payload) >= 16 and payload[2] == 0x07:
            r = payload[5:]
            self.events.append((t, "me_calibration", {
                "status": r[0], "accel_dcal": r[1], "gyro_dcal": r[2], "mag_dcal": r[3]}))
        elif rid == 0xFC and len(payload) >= 17:
            interval = struct.unpack_from("<I", payload, 5)[0]
            self.events.append((t, "feature", {"report": "0x%02X" % payload[1], "interval_us": interval}))

    def poll(self):
        """Read one packet. Returns list of (t, rid, seq, status, values) sensor reports."""
        got = self.read_packet()
        if got is None:
            return None
        t = time.monotonic()
        channel, payload = got
        if channel == CH_CONTROL:
            self.handle_control(t, payload)
            return []
        if channel != CH_REPORTS:
            return []
        # Entries are (t, rid, seq, status, values) or (t, "unknown"|"truncated", rid).
        return [(t,) + r for r in parse_input_batch(payload)]
