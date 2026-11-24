"""Link: signed packet format, ttl, hard ceilings, ack auth against a local UDP bridge."""
import json
import math
import socket
import threading
import time

import pytest

from fc.core import Cmd, RobotCfg
from fc.link import CEIL, Link, clamp_cmd, packet
from fc.robot import drive_bridge as db
from fc.robot.keygen import write_key

KEY = bytes(range(32))


@pytest.fixture
def key_file(tmp_path):
    path = tmp_path / "link.key"
    path.write_text(KEY.hex() + "\n")
    path.chmod(0o600)
    return path


def test_packet_format_and_ttl():
    raw = packet(KEY, "blue", "abc", 7, Cmd(0.1, -0.05, 0.3), {"max_vx": 0.3, "max_vy": 0.15, "max_wz": 1.0, "ttl_s": 0.4}, t=12.5)
    msg = json.loads(raw)
    mac = msg.pop("mac")
    assert msg == {"v": 2, "robot": "blue", "sid": "abc", "seq": 7, "t": 12.5, "vx": 0.1, "vy": -0.05, "wz": 0.3, "ttl": 0.4}
    assert db.verify(KEY, {**msg, "mac": mac})
    assert db.parse_packet(raw, KEY).seq == 7                               # the bridge accepts it as-is
    assert json.loads(packet(KEY, "b", "s", 1, Cmd(), {"ttl_s": 99}))["ttl"] == 0.5      # ttl clamped
    assert json.loads(packet(KEY, "b", "s", 1, Cmd(), {"ttl_s": 0.0}))["ttl"] == 0.25    # missing/zero -> default
    assert abs(json.loads(packet(KEY, "b", "s", 1, Cmd(), {}))["t"] - time.time()) < 5   # laptop wall clock


def test_hard_ceilings_override_params():
    wild = {"max_vx": 9, "max_vy": 9, "max_wz": 9}
    assert clamp_cmd(Cmd(3, -3, 30), wild) == (CEIL["vx"], -CEIL["vy"], CEIL["wz"])
    assert clamp_cmd(Cmd(0.4, 0.2, 1.5), {"max_vx": 0.3, "max_vy": 0.15, "max_wz": 1.0}) == (0.3, 0.15, 1.0)
    assert clamp_cmd(Cmd(math.nan, math.inf, -math.inf), wild) == (0.0, 0.0, 0.0)
    assert clamp_cmd(Cmd(0.2, 0, 0), {"max_vx": 0.0})[0] == 0.0


class FakeBridge:
    """Pi stand-in running the real BridgeCore admission: acks only accepted packets,
    then sends garbage and a forged (unsigned) ack that must both be ignored."""

    def __init__(self, robot_id="blue", key=KEY, forge=True):
        self.core = db.BridgeCore(key, robot_id)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port, self.got, self.run, self.forge = self.sock.getsockname()[1], [], True, forge
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.run:
            try:
                data, addr = self.sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            ok, seq, _err = self.core.on_packet(data, time.monotonic(), addr[0])
            if not ok:
                continue
            self.got.append(json.loads(data))
            imu = {"level": "ok", "tilt_deg": 1.5, "pulse_density": 0.1}
            self.sock.sendto(self.core.make_ack(seq, "stock", imu, self.core.snapshot(time.monotonic())), addr)
            self.sock.sendto(b"not json", addr)  # garbage must be ignored
            if self.forge:
                self.sock.sendto(json.dumps({"v": 2, "ack": seq + 1000, "profile": "evil"}).encode(), addr)

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


def test_send_ack_status_and_stop_all(key_file):
    echo = FakeBridge(forge=False)
    robots = [RobotCfg("blue", "blue", 10, "127.0.0.1", "stock"), RobotCfg("red", "red", 11, "", "stock")]
    params = {"max_vx": 0.3, "max_vy": 0.15, "max_wz": 1.0, "ttl_s": 0.25}
    link = Link(robots, params, port=echo.port, resolve=lambda h: h, key_file=key_file)
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "127.0.0.1")
        for _ in range(3):
            link.send({"blue": Cmd(1.0, 0.0, 0.5), "red": Cmd(0.2, 0, 0)})
        assert _wait(lambda: link.status()["blue"]["acks"] >= 3)
        st = link.status()
        b = st["blue"]
        assert b["connected"] and b["profile"] == "stock" and b["imu"]["level"] == "ok" and b["auth"] == "ok"
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


def test_forged_ack_flags_bad_ack_and_good_ack_clears_it(key_file):
    echo = FakeBridge(forge=True)
    link = Link([RobotCfg("blue", "blue", 10, "127.0.0.1", "stock")], {}, port=echo.port, resolve=lambda h: h,
                key_file=key_file)
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "127.0.0.1")
        link.send({"blue": Cmd(0.1, 0, 0)})
        assert _wait(lambda: link.status()["blue"]["bad_acks"] >= 1)
        st = link.status()["blue"]
        assert st["auth"] == "bad_ack" and st["acks"] == 1 and st["profile"] == "stock"  # forged "evil" ignored
        # A newer genuine ack clears the flag.
        now = time.monotonic() + 1
        good = echo.core.make_ack(2, "stock", None, {})
        assert link.handle_ack(good, "127.0.0.1", now) == "blue" and link.status()["blue"]["auth"] == "ok"
        # Replaying that ack gives no liveness credit.
        assert link.handle_ack(good, "127.0.0.1", now + 1) is None and link.status()["blue"]["acks"] == 2
    finally:
        link.close()
        echo.close()


def test_ack_for_other_session_or_robot_rejected(key_file):
    link = Link([RobotCfg("blue", "blue", 10, "10.9.9.9", "stock")], {}, resolve=lambda h: h, key_file=key_file)
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "10.9.9.9")
        base = {"v": 2, "robot": "blue", "sid": link.sid, "ack": 1, "ok": True}
        assert link.handle_ack(db.sign(KEY, {**base, "sid": "oldsession"}), "10.9.9.9", 1.0) is None
        assert link.handle_ack(db.sign(KEY, {**base, "robot": "red"}), "10.9.9.9", 1.0) is None
        assert link.handle_ack(db.sign(b"w" * 32, base), "10.9.9.9", 1.0) is None
        assert link.status()["blue"]["bad_acks"] == 3 and link.status()["blue"]["auth"] == "bad_ack"
        assert link.handle_ack(db.sign(KEY, base), "10.9.9.9", 2.0) == "blue"
    finally:
        link.close()


def test_wrong_robot_and_old_session_not_acked(key_file):
    """End to end: a link addressed to 'blue' gets nothing from a bridge configured as 'red';
    a restarted link (new sid) resumes on the same bridge."""
    red = FakeBridge(robot_id="red", forge=False)
    cfg = [RobotCfg("blue", "blue", 10, "127.0.0.1", "stock")]
    link = Link(cfg, {}, port=red.port, resolve=lambda h: h, key_file=key_file)
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "127.0.0.1")
        link.send({"blue": Cmd(0.1, 0, 0)})
        assert _wait(lambda: red.core.stats["robot"] >= 1)
        time.sleep(0.1)
        assert link.status()["blue"]["acks"] == 0 and not red.got
    finally:
        link.close()
        red.close()
    blue = FakeBridge(robot_id="blue", forge=False)
    try:
        first = Link(cfg, {}, port=blue.port, resolve=lambda h: h, key_file=key_file)
        assert _wait(lambda: first.status()["blue"]["addr"] == "127.0.0.1")
        first.send({"blue": Cmd(0.1, 0, 0)})
        assert _wait(lambda: first.status()["blue"]["acks"] == 1)
        first.close()
        second = Link(cfg, {}, port=blue.port, resolve=lambda h: h, key_file=key_file)   # laptop restart
        assert _wait(lambda: second.status()["blue"]["addr"] == "127.0.0.1")
        second.send({"blue": Cmd(0.2, 0, 0)})
        assert _wait(lambda: second.status()["blue"]["acks"] == 1)
        assert blue.core.sid == second.sid != first.sid and blue.core.stats["sessions"] == 2
        second.close()
    finally:
        blue.close()


def test_missing_key_sends_nothing(tmp_path):
    echo = FakeBridge(forge=False)
    link = Link([RobotCfg("blue", "blue", 10, "127.0.0.1", "stock")], {}, port=echo.port, resolve=lambda h: h,
                key_file=tmp_path / "nope.key")
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "127.0.0.1")
        link.send({"blue": Cmd(0.1, 0, 0)})
        st = link.status()["blue"]
        assert st["auth"] == "missing_key" and "missing" in st["auth_error"] and st["sent"] == 0
        assert echo.core.stats["rx"] == 0
    finally:
        link.close()
        echo.close()


def test_keygen_writes_0600_and_refuses_overwrite(tmp_path):
    path = write_key(tmp_path / "cfg" / "link.key")
    assert (path.stat().st_mode & 0o777) == 0o600 and len(db.load_key(str(path))) == 32
    with pytest.raises(FileExistsError):
        write_key(path)
    old = path.read_text()
    write_key(path, force=True)
    assert path.read_text() != old and (path.stat().st_mode & 0o777) == 0o600


def test_unknown_sender_ignored(key_file):
    link = Link([RobotCfg("blue", "blue", 10, "10.9.9.9", "stock")], {}, resolve=lambda h: h, key_file=key_file)
    try:
        assert link.handle_ack(b'{"ack": 1}', "127.0.0.2", time.monotonic()) is None
        assert link.handle_ack(b'{"nope": 1}', "10.9.9.9", time.monotonic()) is None
    finally:
        link.close()


def test_udp_host_overrides_ssh_host_and_status_fields(key_file):
    """udp_host (LAN) wins over the ssh alias; RTT p95, loss and new ack fields reach status."""
    echo = FakeBridge(forge=False)
    resolved = []

    def resolve(h):
        resolved.append(h)
        return "127.0.0.1" if h == "lan-pi" else "100.102.191.86"

    link = Link([RobotCfg("blue", "blue", 10, "mts", "stock")], {}, port=echo.port, resolve=resolve,
                key_file=key_file, udp_hosts={"blue": "lan-pi"})
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "127.0.0.1")
        st = link.status()["blue"]
        assert resolved == ["lan-pi"] and st["via"] == "udp_host" and st["host"] == "lan-pi" and not st["tailscale"]
        for _ in range(12):
            link.send({"blue": Cmd(0.1, 0, 0)})
        assert _wait(lambda: link.status()["blue"]["acks"] >= 12)
        time.sleep(0.6)                                   # past LOSS_AFTER_S
        st = link.status()["blue"]
        assert st["rtt_p95_ms"] is not None and st["loss"] == 0.0
        good = db.sign(KEY, {"v": 2, "robot": "blue", "sid": link.sid, "ack": 999, "ok": True,
                             "hold": "imu", "imu_hold": True, "mux_ok": False})
        assert link.handle_ack(good, "127.0.0.1", time.monotonic()) == "blue"
        st = link.status()["blue"]
        assert st["imu_hold"] is True and st["mux_ok"] is False and st["hold"] == "imu" and st["extra"]["imu_hold"]
    finally:
        link.close()
        echo.close()


def test_ssh_host_used_without_udp_host_and_tailscale_flagged(key_file):
    link = Link([RobotCfg("blue", "blue", 10, "mts", "stock")], {}, resolve=lambda h: "100.102.191.86",
                key_file=key_file)
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "100.102.191.86")
        st = link.status()["blue"]
        assert st["via"] == "ssh" and st["tailscale"] and st["loss"] is None
    finally:
        link.close()


def test_loss_counts_unacked_packets(key_file):
    link = Link([RobotCfg("blue", "blue", 10, "10.9.9.9", "stock")], {}, port=9, resolve=lambda h: "127.0.0.1",
                key_file=key_file)
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "127.0.0.1")
        for _ in range(10):
            link.send({"blue": Cmd()})
        time.sleep(0.6)
        assert link.status()["blue"]["loss"] == 1.0
    finally:
        link.close()


def test_load_udp_hosts(tmp_path):
    from fc.link import load_udp_hosts
    p = tmp_path / "robots.json"
    p.write_text(json.dumps([{"id": "blue", "udp_host": "192.168.1.5"}, {"id": "red"}]))
    assert load_udp_hosts(p) == {"blue": "192.168.1.5", "red": ""}
    assert load_udp_hosts(tmp_path / "nope.json") == {}


# ---- v1.1: e-stop over the link, fast-play ceiling (2026-11-23) --------------------------
def test_ceiling_allows_fast_play_but_stays_a_hard_cap():
    assert CEIL == {"vx": 0.75, "vy": 0.3, "wz": 2.0}
    assert clamp_cmd(Cmd(0.75, 0, 0), {"max_vx": 0.75})[0] == 0.75
    assert clamp_cmd(Cmd(5.0, 0, 0), {"max_vx": 3.0})[0] == 0.75


def test_estop_packet_is_signed_and_motionless():
    raw = packet(KEY, "blue", "abc", 9, Cmd(0.5, 0.1, 1.0), {"max_vx": 0.75}, t=3.0, estop=True)
    msg = json.loads(raw)
    assert msg["estop"] is True and msg["vx"] == msg["vy"] == msg["wz"] == 0.0
    assert db.verify(KEY, msg)                                      # the flag is under the MAC
    assert db.parse_packet(raw, KEY).vx == 0.0                      # an older bridge reads it as a zero command
    assert "estop" not in json.loads(packet(KEY, "blue", "abc", 10, Cmd(), {}, t=3.1))


def test_estop_burst_and_estopped_ack(key_file):
    echo = FakeBridge(forge=False)
    link = Link([RobotCfg("blue", "blue", 10, "127.0.0.1", "stock")], {"max_vx": 0.3}, port=echo.port,
                resolve=lambda h: h, key_file=key_file)
    try:
        assert _wait(lambda: link.status()["blue"]["addr"] == "127.0.0.1")
        t0 = time.monotonic()
        assert link.estop() == ["blue"]
        assert time.monotonic() - t0 >= 0.07                        # 5 packets, 20 ms apart
        assert _wait(lambda: len(echo.got) >= 5)
        assert all(m.get("estop") is True and m["vx"] == 0 for m in echo.got[:5])
        assert [m["seq"] for m in echo.got[:5]] == [1, 2, 3, 4, 5]
        assert link.status()["blue"]["estop_sent"] == 5
        assert link.status()["blue"]["estopped"] is not False   # True from a v1.1 BridgeCore, None from an older one
        ack = db.sign(KEY, {"v": 2, "robot": "blue", "sid": link.sid, "ack": 99, "ok": True, "estopped": True})
        assert link.handle_ack(ack, "127.0.0.1", time.monotonic()) == "blue"
        assert link.status()["blue"]["estopped"] is True
    finally:
        link.close()
        echo.close()
