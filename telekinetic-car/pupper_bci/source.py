"""Read stock Cyton packets or replay a recording of those packets."""

import csv
from dataclasses import dataclass
import math
from pathlib import Path
import time


RECORDING_COLUMNS = ("t", "sequence", "eeg1_uv", "eeg2_uv", "ax_g", "ay_g", "az_g")
SAMPLE_RATE = 250
_CYTON_ACCEL_FOOTER = 0xC0
_CYTON_ACCEL_G_PER_COUNT = 0.002 / 16


@dataclass(frozen=True)
class Sample:
    t: float
    sequence: int
    eeg_uv: tuple[float, float]
    accel_g: tuple[float, float, float] | None


def _cyton_acceleration(footer, data):
    # Cyton sends six zero AUX bytes when there is no new accelerometer sample.
    # BrainFlow's processed accel rows can repeat old values when raw X is zero.
    if footer != _CYTON_ACCEL_FOOTER or not any(data):
        return None
    raw = bytes(map(int, data))
    x, y, z = (
        int.from_bytes(raw[offset:offset + 2], "big", signed=True) * _CYTON_ACCEL_G_PER_COUNT
        for offset in (0, 2, 4)
    )
    return x, y, z


class CytonSource:
    """Read an eight-channel Cyton over its serial dongle in default accel mode."""

    def __init__(self, port: str, channels: tuple[int, int] = (1, 2)):
        if not port:
            raise ValueError("a Cyton serial port is required")
        if len(channels) != 2 or any(type(ch) is not int or not 1 <= ch <= 8 for ch in channels):
            raise ValueError("select two Cyton EEG channels numbered 1–8")
        if channels[0] == channels[1]:
            raise ValueError("select two different Cyton EEG channels")
        self._port = port
        self._channels = tuple(channels)
        self._board = None

    def __enter__(self):
        if self._board is not None:
            raise RuntimeError("Cyton stream is already open")
        from brainflow.board_shim import BoardIds, BoardShim, BrainFlowError, BrainFlowInputParams

        params = BrainFlowInputParams()
        params.serial_port = self._port
        board_id = BoardIds.CYTON_BOARD.value
        board = BoardShim(board_id, params)
        eeg_rows = board.get_eeg_channels(board_id)
        other_rows = board.get_other_channels(board_id)
        if len(eeg_rows) != 8 or len(other_rows) != 7:
            raise RuntimeError("expected the eight-channel BrainFlow Cyton with raw AUX data")
        self._eeg_rows = [eeg_rows[channel - 1] for channel in self._channels]
        self._sequence_row = board.get_package_num_channel(board_id)
        self._timestamp_row = board.get_timestamp_channel(board_id)
        self._footer_row, *self._aux_rows = other_rows

        self._board_error = BrainFlowError
        try:
            board.prepare_session()
        except BrainFlowError as error:
            raise RuntimeError(f"could not open Cyton on {self._port}: {error}") from error
        try:
            if board.get_board_sampling_rate() != SAMPLE_RATE:
                raise RuntimeError(f"expected a {SAMPLE_RATE} Hz Cyton stream")
            # BrainFlow timestamps serial packets as they arrive in host Unix time.
            # A fixed anchor preserves the age of packets waiting in the ring buffer.
            before = time.monotonic()
            wall_time = time.time()
            self._clock_offset = (before + time.monotonic()) / 2 - wall_time
            board.start_stream(SAMPLE_RATE * 5)
        except BrainFlowError as error:
            board.release_session()
            raise RuntimeError(f"could not start Cyton on {self._port}: {error}") from error
        except BaseException:
            board.release_session()
            raise
        self._board = board
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        board, self._board = self._board, None
        if board is not None:
            # BrainFlow's Cyton release stops the stream and closes the serial port.
            try:
                board.release_session()
            except self._board_error as error:
                if exc_type is None:
                    raise RuntimeError(f"could not close Cyton on {self._port}: {error}") from error

    def read(self) -> list[Sample]:
        if self._board is None:
            raise RuntimeError("open the Cyton stream before reading")
        try:
            packets = self._board.get_board_data().T
        except self._board_error as error:
            raise RuntimeError(f"could not read Cyton on {self._port}: {error}") from error
        samples = []
        for packet in packets:
            eeg1, eeg2 = (float(packet[row]) for row in self._eeg_rows)
            samples.append(Sample(
                t=float(packet[self._timestamp_row]) + self._clock_offset,
                sequence=int(packet[self._sequence_row]),
                eeg_uv=(eeg1, eeg2),
                accel_g=_cyton_acceleration(packet[self._footer_row], packet[self._aux_rows]),
            ))
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
