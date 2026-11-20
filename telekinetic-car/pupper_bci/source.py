"""Read stock Cyton packets or replay a recording of those packets."""

from collections import deque
from contextlib import suppress
import csv
from dataclasses import dataclass
import math
import os
from pathlib import Path
from threading import Event, Lock, Thread
import time


RECORDING_COLUMNS = ("t", "sequence", "eeg1_uv", "eeg2_uv", "ax_g", "ay_g", "az_g")
SAMPLE_RATE = 250
_PACKET_START = 0xA0
_PACKET_BYTES = 33
_ACCEL_FOOTER = 0xC0
_EEG_UV_PER_COUNT = 4.5e6 / (24 * (2 ** 23 - 1))
_ACCEL_G_PER_COUNT = 0.002 / 16
_READ_TIMEOUT = 0.05
_WRITE_TIMEOUT = 0.25
_RESET_TIMEOUT = 4.0


@dataclass(frozen=True)
class Sample:
    t: float
    sequence: int
    eeg_uv: tuple[float, float]
    accel_g: tuple[float, float, float] | None


class CytonSource:
    """Read default 8-channel, 250 Hz Cyton packets from a USB dongle."""

    def __init__(self, port: str, channels: tuple[int, int] = (1, 2)):
        if not port:
            raise ValueError("a Cyton serial port is required")
        if len(channels) != 2 or any(type(ch) is not int or not 1 <= ch <= 8 for ch in channels):
            raise ValueError("select two Cyton EEG channels numbered 1–8")
        if channels[0] == channels[1]:
            raise ValueError("select two different Cyton EEG channels")
        self._port = port
        self._eeg_offsets = [2 + 3 * (channel - 1) for channel in channels]
        self._serial = None
        self._lock = Lock()
        self._samples = deque(maxlen=SAMPLE_RATE * 5)
        self._reader_error = None

    def __enter__(self):
        if self._serial is not None:
            raise RuntimeError("Cyton stream is already open")
        from serial import Serial

        port = Serial(
            self._port, baudrate=115200, timeout=_READ_TIMEOUT,
            write_timeout=_WRITE_TIMEOUT, exclusive=True if os.name == "posix" else None,
        )
        try:
            self._reset(port)
            with self._lock:
                self._samples.clear()
                self._reader_error = None
            self._stop = Event()
            self._thread = Thread(
                target=self._receive, args=(port, self._stop), name="Cyton serial reader", daemon=True,
            )
            self._thread.start()
            try:
                port.write(b"b")
            except BaseException:
                self._stop.set()
                with suppress(OSError):
                    port.write(b"s")
                self._thread.join(_WRITE_TIMEOUT)
                raise
        except BaseException:
            with suppress(OSError):
                port.close()
            raise
        self._serial = port
        return self

    def _reset(self, port):
        # Soft reset restores the stock acquisition mode and gain of 24.
        port.reset_input_buffer()
        port.write(b"v")
        deadline = time.monotonic() + _RESET_TIMEOUT
        banner = bytearray()
        while time.monotonic() < deadline and len(banner) < 4096:
            banner.extend(port.read(min(port.in_waiting or 1, 512)))
            if b"$$$" in banner:
                break
        else:
            raise RuntimeError(f"no Cyton startup response on {self._port}")
        welcome, _, _ = banner.partition(b"$$$")
        if b"On Daisy ADS1299" in welcome:
            raise RuntimeError("Cyton Daisy is attached; use the stock 8-channel Cyton")
        if not all(marker in welcome for marker in (b"OpenBCI V3", b"ADS1299", b"LIS3DH")):
            raise RuntimeError(f"unexpected Cyton startup response on {self._port}")
        port.reset_input_buffer()

    def __exit__(self, exc_type, exc_value, traceback):
        port, self._serial = self._serial, None
        self._stop.set()
        try:
            port.write(b"s")
        except OSError:
            if exc_type is None:
                raise
        finally:
            try:
                self._thread.join(_WRITE_TIMEOUT)
            finally:
                port.close()

    def _receive(self, port, stop: Event):
        pending = bytearray()
        aligned = True
        completed_at = None
        try:
            while not stop.is_set():
                chunk = port.read(min(port.in_waiting or 1, 4096))
                received_at = time.monotonic()
                pending.extend(chunk)
                samples = []
                while len(pending) >= _PACKET_BYTES:
                    start = pending.find(_PACKET_START)
                    if start < 0:
                        pending.clear()
                        aligned = False
                        break
                    if start:
                        del pending[:start]
                        aligned = False
                    if len(pending) < _PACKET_BYTES:
                        break
                    if not _ACCEL_FOOTER <= pending[_PACKET_BYTES - 1] <= 0xC6:
                        del pending[0]
                        aligned = False
                        completed_at = None
                        continue
                    if not aligned:
                        # A data byte can equal the header. Confirm the next frame
                        # boundary after losing alignment and keep the first frame's time.
                        if len(pending) == _PACKET_BYTES:
                            if completed_at is None:
                                completed_at = received_at
                            break
                        if pending[_PACKET_BYTES] != _PACKET_START:
                            del pending[0]
                            completed_at = None
                            continue
                        aligned = True
                    t = received_at if completed_at is None else completed_at
                    samples.append(self._decode(pending[:_PACKET_BYTES], t))
                    del pending[:_PACKET_BYTES]
                    completed_at = None
                with self._lock:
                    if not stop.is_set():
                        self._samples.extend(samples)
        except Exception as error:
            with self._lock:
                if not stop.is_set():
                    self._reader_error = error

    def _decode(self, frame: bytearray, t: float) -> Sample:
        eeg1, eeg2 = (
            int.from_bytes(frame[offset:offset + 3], "big", signed=True) * _EEG_UV_PER_COUNT
            for offset in self._eeg_offsets
        )
        accel = None
        # Standard packets omit a new accel reading by sending six zero AUX bytes.
        if frame[32] == _ACCEL_FOOTER and any(frame[26:32]):
            ax, ay, az = (
                int.from_bytes(frame[offset:offset + 2], "big", signed=True) * _ACCEL_G_PER_COUNT
                for offset in (26, 28, 30)
            )
            accel = (ax, ay, az)
        return Sample(t, frame[1], (eeg1, eeg2), accel)

    def read(self) -> list[Sample]:
        if self._serial is None:
            raise RuntimeError("open the Cyton stream before reading")
        with self._lock:
            if self._reader_error is not None:
                raise RuntimeError(
                    f"could not read Cyton on {self._port}: {self._reader_error}"
                ) from self._reader_error
            samples = list(self._samples)
            self._samples.clear()
        return samples


def read_recording(path: str | Path) -> list[Sample]:
    """Load a CSV with elapsed seconds and optional accelerometer triples."""
    samples = []
    with open(path, newline="", encoding="utf-8-sig") as stream:
        rows = csv.reader(stream, strict=True)
        if tuple(next(rows, ())) != RECORDING_COLUMNS:
            raise ValueError(f"{path}: expected CSV header {','.join(RECORDING_COLUMNS)}")
        for row in rows:
            try:
                raw_t, raw_sequence, raw_eeg1, raw_eeg2, *raw_accel = row
                if len(raw_accel) != 3:
                    raise ValueError("expected seven columns")
                t, eeg1, eeg2 = map(float, (raw_t, raw_eeg1, raw_eeg2))
                sequence = int(raw_sequence)
                if not 0 <= sequence <= 255:
                    raise ValueError("sequence must be a byte")
                if not all(map(math.isfinite, (t, eeg1, eeg2))):
                    raise ValueError("time and EEG values must be finite")
                if t < 0 or (samples and t < samples[-1].t):
                    raise ValueError("time must be nonnegative and nondecreasing")
                accel = None
                if any(field.strip() for field in raw_accel):
                    ax, ay, az = map(float, raw_accel)
                    if not all(map(math.isfinite, (ax, ay, az))):
                        raise ValueError("acceleration must be finite")
                    accel = ax, ay, az
            except ValueError as exc:
                raise ValueError(f"{path}:{rows.line_num}: {exc}") from exc
            samples.append(Sample(t, sequence, (eeg1, eeg2), accel))
    if not samples:
        raise ValueError(f"{path}: recording contains no samples")
    return samples
