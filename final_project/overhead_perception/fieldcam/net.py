"""Field state over UDP as one JSON datagram per frame.

Deliberately not ROS: the perception laptop (a Mac) doesn't need a ROS install,
and anything can listen. ros2/field_state_bridge.py republishes it into ROS 2.
Ages are relative, so the two machines' clocks do not need to be synchronised.
"""
from __future__ import annotations

import json
import socket
import time


class StatePublisher:
    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def send(self, state: dict):
        self.sock.sendto(json.dumps(state, separators=(",", ":")).encode(), self.addr)


class StateSubscriber:
    def __init__(self, port: int, bind: str = "0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.setblocking(False)
        self._latest: dict | None = None
        self._rx_time = 0.0

    def latest(self) -> dict | None:
        """Newest state received so far (drains the socket), with rx_age_s added.

        Total staleness of an object = state[obj]["age_s"] + state["rx_age_s"].
        """
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            self._latest = json.loads(data)
            self._rx_time = time.monotonic()
        if self._latest is None:
            return None
        self._latest["rx_age_s"] = time.monotonic() - self._rx_time
        return self._latest
