"""Same-Pi evidence for soccer authority. This reader cannot command the board."""
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
PROFILE = {"strict": 1, "supported_rehome": 2, "field_redundant_imu": 3}


class BoardReceipt:
    """Read the single writer's seqlock, pinned to the prepared manager process."""

    def __init__(self, path, manager_pid, profile):
        self.path, self.manager_pid = Path(path), manager_pid
        self.profile = PROFILE.get(profile)
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
                if row[0] and not row[0] & 1:
                    if (row[1] != MAGIC or row[2] != self.manager_pid or not row[3]
                            or row[22] != self.profile or row[23] or row[7] > 5 or row[21] > 3
                            or max(row[4:7]) >= 2**53 or max(row[13:15]) > 256):
                        raise ValueError("Board receipt identity, schema or enrolled profile differs")
                    return row
        raise ValueError("Board receipt could not be read consistently")

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class MatchImu:
    """One match epoch and its original board/Home binding, retained through pause."""

    def __init__(self, reader, ros_delivery, *, clock_ns=time.monotonic_ns):
        self.reader, self.ros_delivery, self.clock_ns = reader, ros_delivery, clock_ns
        self.epoch = self.anchor = self.hold_id = self.walk_generation = None
        self.applied_hold = None
        self.reported_hold_state = None
        self.pitch = None
        self.stop_generation = None
        self.hold_ns = self.owner_sample_ns = self.owner_query_ns = 0
        self.walker_lost = False
        self.last = self.session = self.generation = self.incident = None
        self.terminal = None
        self.released = None
        self.state, self.reason = "unknown", "Waiting for board evidence"
        self.deadline_ns = 0
        self._reset_samples()

    def _reset_samples(self):
        self.pairs, self.first_pair, self.pair_count = None, None, 0

    def _healthy_pairs(self, row):
        pair = (row[9], row[10])
        if self.pairs is None or all(t > last for t, last in zip(pair, self.pairs)):
            self.pairs = pair
            if self.first_pair is None:
                self.first_pair = min(pair)
            self.pair_count += 1
        return self.pair_count >= 3 and min(pair) - self.first_pair >= 20_000_000

    def _evidence(self, row, ros, now):
        if self.last is not None:
            if ((row[2], row[3]) != (self.last[2], self.last[3]) or row[4] != self.last[4]
                    or row[5] != self.last[5]):
                self.terminal = "Board session or Home/fault generation changed; verify supported Home"
            if (row[0] < self.last[0] or row[8] < self.last[8] or row[6] < self.last[6]
                    or row[0] == self.last[0] and row != self.last):
                self.terminal = "Board receipt regressed; verify the supported session"
        self.last = row
        self.session, self.generation, self.incident = f"{row[2]}-{row[3]}", row[4], row[6]
        if row[7] == 4 or (self.epoch and row[7] in (0, 5)):
            self.terminal = "Board deactivated, faulted or entered Home"
        if self.terminal:
            raise ValueError(self.terminal)
        if not row[4] or row[7] in (0, 5):
            raise ValueError("Board has no completed active Home")
        if not row[3] <= row[8] <= now or now - row[8] >= AGE_NS:
            raise ValueError("Board health receipt expired")
        if row[7] == 2:
            raise ValueError("Primary IMU lost; board is using backup reports")
        if (row[7] not in (1, 3) or row[15:17] != (5, 2)
                or row[9:11] != row[17:19] or row[11:13] != row[19:21]):
            raise ValueError("Current primary orientation and gyro evidence is unavailable")
        for receipt, minimum_age in zip(row[9:11], row[11:13]):
            if not 0 < receipt <= row[8] or not 0 <= now - receipt + minimum_age < AGE_NS:
                raise ValueError("Original primary orientation or gyro evidence expired")
        if (not ros or len(ros) != 3 or not all(math.isfinite(v) for v in ros)
                or not 0 <= now - round(ros[0] * 1e9) < AGE_NS):
            raise ValueError("Local ROS IMU delivery is missing, stale or invalid")
        self.deadline_ns = min(row[8], row[9] - row[11], row[10] - row[12],
                               round(ros[0] * 1e9)) + AGE_NS

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
            self._new_hold(problem or ("Board recorded a new primary IMU incident" if missed else
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
        return self.owner_sample_ns and 0 <= self.clock_ns() - self.owner_sample_ns < 200_000_000

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

    @property
    def native_release(self):
        return self.last is not None and self.last[7] == 3

    @property
    def native_binding(self):
        return self.session, self.generation, self.incident
