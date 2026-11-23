"""Same-Pi board health evidence for soccer authority. This reader cannot command the board.

The driver publishes one mmap'd record (board_health.hpp, FC_BOARD_HEALTH_V1):
24 little-endian uint64 words guarded by a seqlock in word 0. Only the strict
profile is selected: a primary IMU fault deactivates the board (status 4) and
requires a supported Home. Backup-IMU and supported-rehome states are not
accepted. Health is never inferred from an absent, stale or malformed record.
"""
from __future__ import annotations

import math
import os
import secrets
import stat
import struct
import time
from pathlib import Path


RECORD = struct.Struct("<24Q")
SEQUENCE = struct.Struct("<Q")
MAGIC = int.from_bytes(b"PUPFCIM1", "little")
AGE_NS = 100_000_000  # Same upper bound as the board's feedback_error().
PROFILE = {"strict": 1}
# Word indices.
SEQ, IDENT, PID, INSTANCE, HOMES, FAULTS, INCIDENTS, STATUS, PUBLISHED = range(9)
QUAT_RX, GYRO_RX, QUAT_AGE, GYRO_AGE, QUAT_REPORT, GYRO_REPORT, QUAT_SOURCE, GYRO_SOURCE = range(9, 17)
RAW_QUAT_RX, RAW_GYRO_RX, RAW_QUAT_AGE, RAW_GYRO_AGE, SPI, BOARD_PROFILE, RESERVED = range(17, 24)
# Status values 0-5 are wire-valid. 2/3 belong to the unselected redundant-IMU
# profile; MatchImu never treats them as healthy.
NOT_HOMED, HEALTHY, TERMINAL, HOMING = 0, 1, 4, 5
PRIMARY_SOURCES = (5, 2)  # Primary BNO orientation report 5, gyro report 2.


class BoardReceipt:
    """Read the single writer's seqlock, pinned to the prepared manager process."""

    def __init__(self, path, manager_pid, profile="strict"):
        if profile not in PROFILE:
            raise ValueError("this candidate reads strict board health only")
        self.path, self.manager_pid = Path(path), manager_pid
        self.profile = PROFILE[profile]
        self.fd = None

    def read(self):
        if self.fd is None:
            self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
        info = os.fstat(self.fd)
        if not stat.S_ISREG(info.st_mode) or info.st_size != RECORD.size or info.st_nlink != 1:
            raise ValueError("Board receipt is unavailable or was replaced")
        for _ in range(3):
            before = os.pread(self.fd, SEQUENCE.size, 0)
            data = os.pread(self.fd, RECORD.size, 0)
            after = os.pread(self.fd, SEQUENCE.size, 0)
            if len(data) == RECORD.size and before == data[:8] == after:
                row = RECORD.unpack(data)
                if row[SEQ] and not row[SEQ] & 1:
                    if (row[IDENT] != MAGIC or row[PID] != self.manager_pid or not row[INSTANCE]
                            or row[BOARD_PROFILE] != self.profile or row[RESERVED]
                            or row[STATUS] > HOMING or row[SPI] > 3
                            or max(row[HOMES:INCIDENTS + 1]) >= 2**53
                            or max(row[QUAT_REPORT:GYRO_REPORT + 1]) > 256):
                        raise ValueError("Board receipt identity, schema or enrolled profile differs")
                    return row
        raise ValueError("Board receipt could not be read consistently")

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class MatchImu:
    """Board health for one match epoch, bound to its original board session and Home.

    States: unknown (no current evidence), healthy, paused (a hold for this
    epoch), ready_to_resume (evidence recovered; an explicit operator resume
    with a new epoch is still required) and home_required (terminal).
    """

    def __init__(self, reader, ros_delivery, *, clock_ns=time.monotonic_ns):
        self.reader, self.ros_delivery, self.clock_ns = reader, ros_delivery, clock_ns
        self.epoch = self.anchor = self.hold_id = self.walk_generation = None
        self.hold_reason = None
        self.applied_hold = self.reported_hold_state = None
        self.pitch = self.stop_generation = None
        self.hold_ns = self.owner_sample_ns = self.owner_query_ns = 0
        self.walker_lost = False
        self.last = self.session = self.generation = self.incident = None
        self.terminal = None
        self.state, self.reason = "unknown", "Waiting for board evidence"
        self.deadline_ns = 0
        self._reset_samples()

    def _reset_samples(self):
        self.pairs, self.first_pair, self.pair_count = None, None, 0

    def _healthy_pairs(self, row):
        """Require three advancing orientation/gyro receipt pairs spanning 20 ms."""
        pair = (row[QUAT_RX], row[GYRO_RX])
        if self.pairs is None or all(t > last for t, last in zip(pair, self.pairs)):
            self.pairs = pair
            if self.first_pair is None:
                self.first_pair = min(pair)
            self.pair_count += 1
        return self.pair_count >= 3 and min(pair) - self.first_pair >= 20_000_000

    def _evidence(self, row, ros, now):
        if self.last is not None:
            if (row[PID], row[INSTANCE], row[HOMES], row[FAULTS]) != (
                    self.last[PID], self.last[INSTANCE], self.last[HOMES], self.last[FAULTS]):
                self.terminal = "Board session or Home/fault generation changed; verify supported Home"
            if (row[SEQ] < self.last[SEQ] or row[PUBLISHED] < self.last[PUBLISHED]
                    or row[INCIDENTS] < self.last[INCIDENTS] or row[SEQ] == self.last[SEQ] and row != self.last):
                self.terminal = "Board receipt regressed; verify the supported session"
        self.last = row
        self.session, self.generation, self.incident = f"{row[PID]}-{row[INSTANCE]}", row[HOMES], row[INCIDENTS]
        if row[STATUS] == TERMINAL or (self.epoch and row[STATUS] in (NOT_HOMED, HOMING)):
            self.terminal = "Board deactivated, faulted or entered Home"
        if self.terminal:
            raise ValueError(self.terminal)
        if row[SPI] != 3:
            raise ValueError("Both current board SPI replies are required")
        if not row[HOMES] or row[STATUS] != HEALTHY:
            raise ValueError("Board has no completed active Home")
        if not row[INSTANCE] <= row[PUBLISHED] <= now or now - row[PUBLISHED] >= AGE_NS:
            raise ValueError("Board health receipt expired")
        if (row[QUAT_SOURCE:GYRO_SOURCE + 1] != PRIMARY_SOURCES
                or row[QUAT_RX:GYRO_RX + 1] != row[RAW_QUAT_RX:RAW_GYRO_RX + 1]
                or row[QUAT_AGE:GYRO_AGE + 1] != row[RAW_QUAT_AGE:RAW_GYRO_AGE + 1]):
            raise ValueError("Current primary orientation and gyro evidence is unavailable")
        for receipt, minimum_age in zip(row[QUAT_RX:GYRO_RX + 1], row[QUAT_AGE:GYRO_AGE + 1]):
            if not 0 < receipt <= row[PUBLISHED] or not 0 <= now - receipt + minimum_age < AGE_NS:
                raise ValueError("Original primary orientation or gyro evidence expired")
        if (not ros or len(ros) != 3 or not all(math.isfinite(v) for v in ros)
                or not 0 <= now - round(ros[0] * 1e9) < AGE_NS):
            raise ValueError("Local ROS IMU delivery is missing, stale or invalid")
        # Motion may not outlive the oldest of the board, sensor and ROS evidence.
        self.deadline_ns = min(row[PUBLISHED], row[QUAT_RX] - row[QUAT_AGE],
                               row[GYRO_RX] - row[GYRO_AGE], round(ros[0] * 1e9)) + AGE_NS

    def _new_hold(self, why):
        self.hold_id = secrets.token_hex(16)
        self.hold_reason = why
        self.hold_ns, self.owner_sample_ns, self.owner_query_ns = self.clock_ns(), 0, 0
        self._reset_samples()

    def refresh(self):
        previous_state, previous_incident = self.state, self.incident
        previous_deadline = self.deadline_ns
        problem = None
        self.deadline_ns = 0
        try:
            row = self.reader.read()
            ros = self.ros_delivery()
            self._evidence(row, ros, self.clock_ns())
        except (OSError, ValueError, OverflowError) as exc:
            problem = self.terminal or str(exc)[:180]
            self._reset_samples()
        missed = previous_incident is not None and self.incident != previous_incident
        expired = previous_deadline and self.clock_ns() >= previous_deadline and not self.hold_id
        if self.epoch and (missed or expired or (problem and (not self.hold_id or previous_state == "ready_to_resume"))):
            self._new_hold(problem or ("Board recorded a new IMU incident" if missed else
                                       "Previous balance evidence expired before renewal"))
        if problem:
            self.state = "home_required" if self.terminal else "paused" if self.hold_id else "unknown"
            self.reason = problem
        elif not self._healthy_pairs(row):
            self.state = "paused" if self.hold_id else "unknown"
            self.reason = "Waiting for three advancing primary orientation and gyro pairs over 20 ms"
        elif self.hold_id:
            self.state, self.reason = "ready_to_resume", self.hold_reason
        else:
            self.state, self.reason = "healthy", "Primary board receipts and ROS IMU delivery are current"
        return self.snapshot()

    def snapshot(self):
        return dict(state=self.state, reason=self.reason, board_session=self.session, generation=self.generation,
                    incident=self.incident, hold_id=self.hold_id, epoch=self.epoch)

    def begin(self, epoch, pitch, stop_generation):
        self.epoch, self.anchor = epoch, (self.session, self.generation)
        self.pitch, self.stop_generation = pitch, stop_generation
        self.hold_id = self.walk_generation = None
        self.walker_lost = False
        self.state, self.reason = "healthy", "New match authority on the verified board session"

    def end(self):
        self.epoch = self.anchor = self.hold_id = self.walk_generation = None
        self._reset_samples()

    def observe_owners(self, hold_id, started_ns, owners, leg_controllers):
        """Record a controller inventory sampled after this hold began."""
        if not hold_id or hold_id != self.hold_id or started_ns < max(self.hold_ns, self.owner_query_ns):
            return
        self.owner_query_ns = started_ns
        self.owner_sample_ns = 0
        if not owners:
            return
        active = {name for name in leg_controllers if owners.get(name) == "active"}
        if active != {"gemini_walk"} or owners.get("trick_leg_kick") != "inactive":
            self.walker_lost = True
        if not self.walker_lost:
            self.owner_sample_ns = started_ns

    def owner_current(self):
        return bool(self.owner_sample_ns) and 0 <= self.clock_ns() - self.owner_sample_ns < 200_000_000

    def pause(self, epoch):
        if not self.epoch or epoch != self.epoch:
            raise ValueError("IMU pause does not match the current match epoch")
        if not self.hold_id:
            self._new_hold("Match paused for participant IMU recovery")
            self.state, self.reason = "paused", self.hold_reason

    def check_resume(self, message):
        if (self.state != "ready_to_resume" or not self.hold_id
                or message.get("hold_id") != self.hold_id or message.get("previous_epoch") != self.epoch
                or not isinstance(message.get("epoch"), str) or message["epoch"] == self.epoch
                or message.get("pitch") != self.pitch
                or type(message.get("generation")) is not int
                or (message.get("board_session"), message["generation"]) != self.anchor
                or (self.session, self.generation) != self.anchor):
            raise ValueError("IMU resume requires fresh evidence from the original board, Home and held epoch")
