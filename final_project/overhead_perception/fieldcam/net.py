"""Field state UDP transport, with optional measured cross-host clock probes.

Existing viewers can ignore ``_transport``. FC ingress requires this envelope,
explicit live-source/session/calibration pins and a bounded clock round trip.
Relative ages alone cannot account for network delay. Identity is an operational
binding on a trusted network, not authentication. No ROS or motion authority.
"""
from __future__ import annotations

import json
import math
import select
import socket
import threading
import time
import uuid

STATE_PROTOCOL = "fieldcam_state_v1"
PROBE_PROTOCOL = "fieldcam_clock_probe_v1"
REPLY_PROTOCOL = "fieldcam_clock_reply_v1"
MAX_DRAIN = 32
# Declared engineering bounds, not a precision-clock certification. Exceeding
# the measured bound invalidates this publisher session until explicit restart.
WALL_STEP_S = .002
CLOCK_DRIFT = .0005


class StatePublisher:
    def __init__(self, host: str, port: int, *, source_id="fieldcam",
                 source_session=None, live=False, monotonic=time.monotonic, wall=time.time,
                 clock_worker=True):
        self.addr = (host, port)
        self.source_id = source_id
        self.source_session = source_session or str(uuid.uuid4())
        if not all(isinstance(v, str) and 0 < len(v) <= 128
                   for v in (self.source_id, self.source_session)):
            raise ValueError("source ID and session must be nonempty bounded strings")
        if type(live) is not bool:
            raise ValueError("live must be a boolean")
        self.live = live
        self.clock_id = "monotonic:" + self.source_session
        self.monotonic, self.wall = monotonic, wall
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setblocking(False)
        self.wire_seq = 0
        self.clock_valid = True
        self._last_pair = self._pair()
        self.clock_start_wall = self._last_pair[1]
        self._closing = threading.Event()
        self._worker = None
        if clock_worker:
            self._worker = threading.Thread(target=self._serve, name="fieldcam-clock", daemon=True)
            self._worker.start()

    def _serve(self):
        while not self._closing.is_set():
            try:
                readable, _, _ = select.select([self.sock], [], [], .05)
                if readable:
                    self.service_clock()
            except OSError:
                break

    def _pair(self):
        before = self.monotonic()
        wall = self.wall()
        after = self.monotonic()
        if (not all(math.isfinite(v) for v in (before, wall, after))
                or before > after or after - before > WALL_STEP_S):
            self.clock_valid = False
        return before, wall, max(0., after - before)

    def _identity(self):
        return {"source_id": self.source_id, "source_session": self.source_session,
                "clock_id": self.clock_id}

    def service_clock(self):
        """Answer at most 32 small probes; worker-owned, or called by tests."""
        for _ in range(MAX_DRAIN):
            try:
                raw, addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            received = self.monotonic()
            try:
                packet = json.loads(raw)
                if (not isinstance(packet, dict) or packet.get("protocol") != PROBE_PROTOCOL
                        or any(packet.get(k) != v for k, v in self._identity().items())
                        or not isinstance(packet.get("nonce"), str)
                        or not 0 < len(packet["nonce"]) <= 64):
                    continue
                reply = {**self._identity(), "protocol": REPLY_PROTOCOL,
                         "nonce": packet["nonce"], "received_mono": received,
                         "sent_mono": self.monotonic()}
                self.sock.sendto(json.dumps(reply, allow_nan=False).encode(), addr)
            except (ValueError, TypeError, OSError):
                continue

    def send(self, state: dict):
        before, wall, span = self._pair()
        last_mono, last_wall, last_span = self._last_pair
        elapsed = before - last_mono
        if (elapsed < 0 or abs((wall - last_wall) - elapsed)
                > WALL_STEP_S + span + last_span + max(0., elapsed) * CLOCK_DRIFT):
            self.clock_valid = False  # A wall step must never silently re-date a frame.
        self._last_pair = before, wall, span
        self.wire_seq += 1
        packet = {**state, "_transport": {**self._identity(), "protocol": STATE_PROTOCOL,
                  "live": self.live, "wire_seq": self.wire_seq, "sent_mono": before,
                  "sent_wall": wall, "pair_span_s": span,
                  "clock_start_wall": self.clock_start_wall, "clock_valid": self.clock_valid}}
        self.sock.sendto(json.dumps(packet, separators=(",", ":"), allow_nan=False).encode(), self.addr)

    def close(self):
        self._closing.set()
        if self._worker is not None:
            self._worker.join(timeout=.2)
        self.sock.close()


class StateSubscriber:
    def __init__(self, port: int, bind: str = "0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.setblocking(False)
        self._latest: dict | None = None
        self._rx_time = 0.0

    def latest(self) -> dict | None:
        """Viewer-only latest state. rx_age_s excludes network transit time."""
        for _ in range(MAX_DRAIN):
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                packet = json.loads(data)
            except (ValueError, UnicodeError):
                continue
            if not isinstance(packet, dict) or "stamp" not in packet:
                continue
            self._latest = packet
            self._rx_time = time.monotonic()
        if self._latest is None:
            return None
        return {**self._latest, "rx_age_s": time.monotonic() - self._rx_time}

    def close(self):
        self.sock.close()
