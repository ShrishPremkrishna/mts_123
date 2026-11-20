"""Bounded wire format for the opt-in, same-host FC command deadline path.

This module does not grant motion authority or convert clocks. Callers retain
original issue/deadline values, enforce sequence/veto ordering, and compare the
deadline again at the point of use. Legacy Twist paths do not use this format.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path
import re

SCHEMA = "pupper_body_command_deadline_v1"
CONTRACT = "fc_deadline_v1"
REQUEST_TOPIC = "/fc/body_command/request"
ADMITTED_TOPIC = "/fc/body_command/admitted"
MAX_PAYLOAD_BYTES = 1024
MAX_LIFETIME_NS = 150_000_000
MAX_INT = 2**63 - 1
# _drive rotates bounded vectors in floating point. This accommodates arithmetic
# roundoff, not a wider motion envelope; the native parser uses the same margin.
NORM_EPSILON = 1e-12
KEYS = frozenset(("schema", "clock_id", "source_instance", "seq",
                  "issued_mono_ns", "expires_mono_ns", "vx", "vy", "wz", "allowed"))


def local_clock_id() -> str:
    """Identify Linux boot and CLOCK_MONOTONIC time namespace; no fallback."""
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if re.fullmatch(r"[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}", boot) is None:
        raise ValueError("invalid Linux boot identity")
    inode = os.stat("/proc/self/ns/time").st_ino
    if inode <= 0:
        raise ValueError("missing Linux time namespace")
    return f"linux-monotonic:{boot}:{inode}"


def _integer(value, name):
    if type(value) is not int or not 1 <= value <= MAX_INT:
        raise ValueError(f"{name} must be a positive int64 JSON integer")
    return value


def _number(value, name):
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be numeric")
    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _unique(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError("duplicate envelope key")
        value[key] = item
    return value


def _constant(value):
    raise ValueError("nonfinite JSON constant")


@dataclass(frozen=True)
class DeadlineCommand:
    clock_id: str
    source_instance: str
    seq: int
    issued_mono_ns: int
    expires_mono_ns: int
    vx: float
    vy: float
    wz: float
    allowed: bool = True

    @property
    def origin(self):
        """Fields a relay must preserve, including genuine source velocities."""
        return (self.clock_id, self.source_instance, self.seq, self.issued_mono_ns,
                self.expires_mono_ns, self.vx, self.vy, self.wz)

    @property
    def is_zero(self):
        return self.vx == self.vy == self.wz == 0.0

    def veto(self):
        return replace(self, allowed=False)

    def fresh(self, now_ns):
        return self.issued_mono_ns <= now_ns < self.expires_mono_ns

    def encode(self):
        # Validate locally constructed messages too, before they become wire data.
        payload = {"schema": SCHEMA, **vars(self)}
        return _from_mapping(payload)._encode_validated()

    def _encode_validated(self):
        encoded = json.dumps({"schema": SCHEMA, **vars(self)}, separators=(",", ":"),
                             allow_nan=False)
        if len(encoded.encode("utf-8")) > MAX_PAYLOAD_BYTES:
            raise ValueError("command envelope oversized after JSON encoding")
        return encoded


def _from_mapping(data):
    if type(data) is not dict or data.keys() != KEYS:
        raise ValueError("envelope requires exactly the ten contract fields")
    if data["schema"] != SCHEMA:
        raise ValueError("unknown command schema")
    clock = data["clock_id"]
    if type(clock) is not str or not clock or len(clock) > 128 or not clock.isascii():
        raise ValueError("invalid clock identity")
    instance = data["source_instance"]
    if type(instance) is not str or re.fullmatch(r"[0-9a-f]{32}", instance) is None:
        raise ValueError("invalid source instance")
    seq = _integer(data["seq"], "seq")
    issued = _integer(data["issued_mono_ns"], "issued_mono_ns")
    expires = _integer(data["expires_mono_ns"], "expires_mono_ns")
    if not issued < expires or expires - issued > MAX_LIFETIME_NS:
        raise ValueError("command lifetime must be positive and at most 150 ms")
    vx, vy, wz = (_number(data[name], name) for name in ("vx", "vy", "wz"))
    if (abs(vx) > .3 or abs(vy) > .3 or abs(wz) > 1.0 or
            math.hypot(vx, vy) > .3 + NORM_EPSILON):
        raise ValueError("command exceeds FC velocity bounds")
    if type(data["allowed"]) is not bool:
        raise ValueError("allowed must be boolean")
    return DeadlineCommand(clock, instance, seq, issued, expires, vx, vy, wz,
                           data["allowed"])


def decode(payload: str) -> DeadlineCommand:
    """Validate structure only; receiver owns time, identity and ordering checks."""
    if type(payload) is not str or len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError("command envelope missing or oversized")
    try:
        if len(payload.encode("utf-8")) > MAX_PAYLOAD_BYTES:
            raise ValueError("command envelope oversized")
        data = json.loads(payload, object_pairs_hook=_unique, parse_constant=_constant)
        return _from_mapping(data)
    except (UnicodeError, RecursionError) as exc:
        raise ValueError("invalid command JSON encoding or nesting") from exc
