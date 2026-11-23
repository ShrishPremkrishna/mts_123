"""Link: packet format, ttl, hard ceilings, ack parsing against a local UDP echo."""
import json
import math
import socket
import threading
import time

from fc.core import Cmd, RobotCfg
from fc.link import CEIL, Link, clamp_cmd, packet


def test_packet_format_and_ttl():
    msg = json.loads(packet(7, Cmd(0.1, -0.05, 0.3), {"max_vx": 0.3, "max_vy": 0.15, "max_wz": 1.0, "ttl_s": 0.4}, t=12.5))
    assert msg == {"v": 1, "seq": 7, "t": 12.5, "vx": 0.1, "vy": -0.05, "wz": 0.3, "ttl": 0.4}
    assert json.loads(packet(1, Cmd(), {"ttl_s": 99}))["ttl"] == 0.5      # ttl clamped
    assert json.loads(packet(1, Cmd(), {"ttl_s": 0.0}))["ttl"] == 0.25    # missing/zero -> default


def test_hard_ceilings_override_params():
    wild = {"max_vx": 9, "max_vy": 9, "max_wz": 9}
    assert clamp_cmd(Cmd(3, -3, 30), wild) == (CEIL["vx"], -CEIL["vy"], CEIL["wz"])
    assert clamp_cmd(Cmd(0.4, 0.2, 1.5), {"max_vx": 0.3, "max_vy": 0.15, "max_wz": 1.0}) == (0.3, 0.15, 1.0)
    assert clamp_cmd(Cmd(math.nan, math.inf, -math.inf), wild) == (0.0, 0.0, 0.0)
    assert clamp_cmd(Cmd(0.2, 0, 0), {"max_vx": 0.0})[0] == 0.0


class Echo:
    """Fake Pi bridge: records packets, acks each one."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port, self.got, self.run = self.sock.getsockname()[1], [], True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.run:
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            msg = json.loads(data)
            self.got.append(msg)
            reply = {"v": 1, "ack": msg["seq"], "imu": {"level": "ok", "tilt_deg": 1.5, "pulse_density": 0.1}, "profile": "stock"}
            self.sock.sendto(json.dumps(reply).encode(), addr)
            self.sock.sendto(b"not json", addr)  # garbage must be ignored

    def close(self):
        self.run = False
        self.sock.close()


def _wait(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_send_ack_status_and_stop_all():
    echo = Echo()
    robots = [RobotCfg("blue", "blue", 10, "127.0.0.1", "stock"), RobotCfg("red", "red", 11, "", "stock")]
    params = {"max_vx": 0.3, "max_vy": 0.15, "max_wz": 1.0, "ttl_s": 0.25}
    link = Link(robots, params, port=echo.port, resolve=lambda h: h)
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "127.0.0.1")
        for _ in range(3):
            link.send({"blue": Cmd(1.0, 0.0, 0.5), "red": Cmd(0.2, 0, 0)})
        assert _wait(lambda: link.status()["blue"]["acks"] >= 3)
        st = link.status()
        b = st["blue"]
        assert b["connected"] and b["profile"] == "stock" and b["imu"]["level"] == "ok"
        assert b["rtt_ms"] is not None and b["rtt_ms"] < 500 and b["ack_age"] < 1.0
        assert [m["seq"] for m in echo.got[:3]] == [1, 2, 3]
        assert echo.got[0]["vx"] == 0.3            # params cap applied at the link
        assert st["red"]["error"] == "no host" and st["red"]["sent"] == 0
        params["ttl_s"] = 0.5                     # live param edits reach the link
        link.stop_all()
        assert _wait(lambda: len(echo.got) >= 6)
        assert all(m["vx"] == m["vy"] == m["wz"] == 0 for m in echo.got[3:6])
        assert echo.got[-1]["ttl"] == 0.5
        assert link.drain_acks() and not link.drain_acks()
    finally:
        link.close()
        echo.close()


def test_unknown_sender_ignored():
    link = Link([RobotCfg("blue", "blue", 10, "10.9.9.9", "stock")], {}, resolve=lambda h: h)
    try:
        assert link.handle_ack(b'{"ack": 1}', "127.0.0.2", time.monotonic()) is None
        assert link.handle_ack(b'{"nope": 1}', "10.9.9.9", time.monotonic()) is None
    finally:
        link.close()
