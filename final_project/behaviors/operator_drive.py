"""Per-robot keyboard controls for the existing combined runner."""
from __future__ import annotations

import math
import secrets

from final_project.pupper_fc.model import Command, zero


class OperatorDrive:
    interval = .1
    lifetime = .25

    def __init__(self, core):
        self.core = core
        self.mode = "autonomous"
        self.match_epoch = None
        # JSON number, including in browsers. A new runner starts in a fresh ticket range.
        self.sequence = secrets.randbits(48)
        self.next_issue = 0.
        self.tickets = {}
        self.pending = None
        self.motion_until = -math.inf
        self.ready = False
        self.needs_neutral = True
        self.generation = None
        self.authority_revision = 0

    def invalidate(self):
        self.authority_revision += 1
        self.tickets.clear()
        self.pending = None
        self.needs_neutral = True

    def observe_path(self, ready, generation):
        if self.ready != ready or self.generation != generation:
            self.invalidate()
        self.ready = ready
        self.generation = generation

    def offer(self, now, resume_cycle=None):
        if now < self.next_issue:
            return None
        self.next_issue = now + self.interval
        self.sequence += 1
        self.tickets = {seq: row for seq, row in self.tickets.items() if row[0] > now}
        self.tickets[self.sequence] = (now + self.lifetime, resume_cycle)
        return {"event": "controls", "robot_id": self.core.robot_id, "mode": self.mode,
                "ticket": self.sequence, "ready": self.ready, "resume_ready": resume_cycle is not None, "needs_neutral": self.needs_neutral}

    def consume_ticket(self, message, now, resume_cycle=None):
        if message.get("robot_id") != self.core.robot_id:
            raise ValueError("control addresses a different robot")
        seq = message.get("ticket")
        if type(seq) is not int or now >= self.tickets.get(seq, (-math.inf, None))[0]:
            raise ValueError("control ticket expired or already consumed")
        deadline, bound_cycle = self.tickets[seq]
        # Even if the two latest replies cross, an older one cannot override the newer.
        self.tickets = {key: end for key, end in self.tickets.items() if key > seq}
        self.authority_revision += 1
        if message.get("command") == "resume" and (resume_cycle is None or bound_cycle != resume_cycle):
            raise ValueError("this ticket was not issued ready for the current goal reset")
        return deadline

    def accept(self, message, *, now, stable):
        deadline = self.consume_ticket(message, now)
        action = message.get("command")
        if action == "mode":
            mode = message.get("mode")
            if mode not in ("autonomous", "teleop"):
                raise ValueError("unknown control mode")
            if not stable or now < self.motion_until or self.pending is not None:
                raise ValueError("wait for zero walking output and a completed kick before changing mode")
            if mode == "teleop" and not self.ready:
                raise ValueError("walker, camera or Stop channel is not ready")
            self.core.executor.stop()
            self.invalidate()
            self.match_epoch = None
            self.mode = mode
            return
        if action != "drive" or self.mode != "teleop":
            raise ValueError("select teleop mode before driving")
        axes = [message.get(key) for key in ("forward", "left", "turn")]
        if not all(type(v) in (int, float) and -1 <= v <= 1 for v in axes):
            raise ValueError("drive axes must be finite numbers from -1 to 1")
        forward, left, turn = axes
        if math.hypot(forward, left) > 1 + 1e-12:
            raise ValueError("diagonal drive input must be normalized")
        self.pending = None
        if self.match_epoch is None or message.get("epoch") != self.match_epoch:
            raise ValueError("drive needs the current accepted kickoff")
        if not any(axes):
            self.needs_neutral = not self.ready
            return
        if not self.ready or self.needs_neutral:
            raise ValueError("wait for walking clearance, then release the keys before driving")
        limits = self.core.limits
        self.pending = Command(forward * limits.vx, left * limits.vy, turn * limits.wz,
                               deadline, self.core.robot_id)

    def command(self, now):
        if self.mode == "autonomous":
            return None
        if self.pending is not None and self.ready and now < self.pending.expires_at:
            return self.pending
        self.pending = None
        return zero(now, self.core.robot_id)

    def observe_output(self, command):
        if any((command.vx, command.vy, command.wz)):
            self.motion_until = command.expires_at
