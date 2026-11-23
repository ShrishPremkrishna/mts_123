"""Pure-logic tests for drive_bridge.py (no ROS needed)."""
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import drive_bridge as db  # noqa: E402


def pkt(seq=1, vx=0.1, vy=0.0, wz=0.0, **extra):
    msg = {"v": 1, "seq": seq, "t": 12.5, "vx": vx, "vy": vy, "wz": wz}
    msg.update(extra)
    return json.dumps(msg).encode()


# ---------------------------------------------------------------- parsing

def test_parse_valid_defaults_ttl():
    cmd = db.parse_packet(pkt(seq=7, vx=0.2, vy=-0.1, wz=0.5))
    assert (cmd.seq, cmd.vx, cmd.vy, cmd.wz, cmd.ttl, cmd.clamped) == (7, 0.2, -0.1, 0.5, 0.25, False)


@pytest.mark.parametrize("vx,vy,wz,expect", [
    (1.0, 0.0, 0.0, (0.5, 0.0, 0.0)),
    (-9.0, 9.0, -9.0, (-0.5, 0.3, -2.0)),
    (0.0, -0.31, 2.01, (0.0, -0.3, 2.0)),
])
def test_parse_clamps_to_hard_ceilings(vx, vy, wz, expect):
    cmd = db.parse_packet(pkt(vx=vx, vy=vy, wz=wz))
    assert (cmd.vx, cmd.vy, cmd.wz) == expect and cmd.clamped


def test_ttl_is_capped_and_must_be_positive():
    assert db.parse_packet(pkt(ttl=5.0)).ttl == db.MAX_TTL
    assert db.parse_packet(pkt(ttl=0.1)).ttl == 0.1
    for bad in (0, -1, "x", None, True):
        with pytest.raises(db.PacketError) as err:
            db.parse_packet(pkt(ttl=bad))
        assert err.value.reason == "bad_ttl"


@pytest.mark.parametrize("data,reason", [
    (b"not json", "not_json"),
    (b"\xff\xfe", "not_json"),
    (b"[1,2]", "not_object"),
    (json.dumps({"v": 2, "seq": 1, "vx": 0, "vy": 0, "wz": 0}).encode(), "bad_version"),
    (json.dumps({"v": 1, "vx": 0, "vy": 0, "wz": 0}).encode(), "bad_seq"),
    (json.dumps({"v": 1, "seq": -1, "vx": 0, "vy": 0, "wz": 0}).encode(), "bad_seq"),
    (json.dumps({"v": 1, "seq": True, "vx": 0, "vy": 0, "wz": 0}).encode(), "bad_seq"),
    (json.dumps({"v": 1, "seq": 1.5, "vx": 0, "vy": 0, "wz": 0}).encode(), "bad_seq"),
    (json.dumps({"v": 1, "seq": 1, "vy": 0, "wz": 0}).encode(), "bad_vx"),
    (json.dumps({"v": 1, "seq": 1, "vx": "1", "vy": 0, "wz": 0}).encode(), "bad_vx"),
    (b'{"v":1,"seq":1,"vx":NaN,"vy":0,"wz":0}', "bad_vx"),
    (b'{"v":1,"seq":1,"vx":0,"vy":Infinity,"wz":0}', "bad_vy"),
    (json.dumps({"v": 1, "seq": 1, "vx": 0, "vy": 0, "wz": False}).encode(), "bad_wz"),
    (json.dumps({"v": 1, "seq": 1, "vx": 0, "vy": 0, "wz": 0, "t": "now"}).encode(), "bad_t"),
    (b"{" + b" " * 2000 + b"}", "too_large"),
])
def test_parse_rejects_malformed(data, reason):
    with pytest.raises(db.PacketError) as err:
        db.parse_packet(data)
    assert err.value.reason == reason


def test_error_keeps_seq_when_known():
    with pytest.raises(db.PacketError) as err:
        db.parse_packet(json.dumps({"v": 1, "seq": 42, "vx": "fast", "vy": 0, "wz": 0}).encode())
    assert err.value.seq == 42


# ---------------------------------------------------------------- admission

def test_out_of_order_and_duplicate_rejected():
    core = db.BridgeCore()
    assert core.on_packet(pkt(seq=5), 0.00) == (True, 5, None)
    assert core.on_packet(pkt(seq=5), 0.01) == (False, 5, "out_of_order")
    assert core.on_packet(pkt(seq=3, vx=0.4), 0.02) == (False, 3, "out_of_order")
    assert core.cmd.seq == 5 and core.cmd.vx == 0.1
    assert core.on_packet(pkt(seq=6), 0.03)[0]
    assert core.stats["ooo"] == 2 and core.stats["ok"] == 2 and core.stats["rx"] == 4


def test_sender_restart_after_gap_accepts_lower_seq():
    core = db.BridgeCore()
    core.on_packet(pkt(seq=900), 0.0)
    assert not core.on_packet(pkt(seq=0), 0.5)[0]
    assert core.on_packet(pkt(seq=0), 0.0 + db.RESTART_GAP_S + 0.01)[0]


def test_bad_packet_does_not_replace_command():
    core = db.BridgeCore()
    core.on_packet(pkt(seq=1, vx=0.2), 0.0)
    ok, seq, err = core.on_packet(b"garbage", 0.01)
    assert not ok and err == "not_json" and core.cmd.vx == 0.2 and core.stats["bad"] == 1


# ---------------------------------------------------------------- dead-man

def test_idle_bridge_is_silent():
    core = db.BridgeCore()
    assert all(core.tick(t * 0.02) is None for t in range(50))
    assert core.stats["pub"] == 0 and core.stats["zero"] == 0


def test_publishes_until_ttl_then_one_zero_then_silent():
    core = db.BridgeCore()
    core.on_packet(pkt(seq=1, vx=0.3, wz=-1.0, ttl=0.25), 10.0)
    outs = [core.tick(10.0 + i * 0.02) for i in range(30)]  # 0 .. 0.58 s
    live = [o for o in outs if o not in (None, (0.0, 0.0, 0.0))]
    assert len(live) == 13 and all(o == (0.3, 0.0, -1.0) for o in live)  # t < 0.25 s
    assert outs[13] == (0.0, 0.0, 0.0)
    assert all(o is None for o in outs[14:])
    assert core.stats["zero"] == 1 and not core.publishing


def test_fresh_packets_keep_it_alive_and_restart_after_expiry():
    core = db.BridgeCore()
    t = 0.0
    for seq in range(1, 50):  # 30 Hz sender for ~1.6 s
        core.on_packet(pkt(seq=seq, vx=0.2), t)
        for k in range(2):
            assert core.tick(t + k * 0.016) == (0.2, 0.0, 0.0)
        t += 1 / 30
    assert core.tick(t + 0.3) == (0.0, 0.0, 0.0)
    assert core.tick(t + 0.32) is None
    core.on_packet(pkt(seq=100, vx=-0.1), t + 0.4)
    assert core.tick(t + 0.41) == (-0.1, 0.0, 0.0)


def test_shutdown_zero_only_when_driving():
    core = db.BridgeCore()
    assert core.shutdown() is None
    core.on_packet(pkt(seq=1), 0.0)
    core.tick(0.01)
    assert core.shutdown() == (0.0, 0.0, 0.0)
    assert core.shutdown() is None and core.tick(0.02) is None


def test_commanding_zero():
    core = db.BridgeCore()
    assert core.commanding_zero(0.0)
    core.on_packet(pkt(seq=1, vx=0.0, vy=0.0, wz=0.0), 0.0)
    assert core.commanding_zero(0.1)
    core.on_packet(pkt(seq=2, wz=0.5, vx=0.0), 0.1)
    assert not core.commanding_zero(0.2)
    assert core.commanding_zero(0.1 + 0.26)


# ---------------------------------------------------------------- ack + rate

def test_ack_shape():
    core = db.BridgeCore()
    ok, seq, err = core.on_packet(pkt(seq=9), 1.0)
    ack = json.loads(db.make_ack(seq, ok, err, "stock", {"level": "ok"}, core.snapshot(1.1)))
    assert ack["v"] == 1 and ack["ack"] == 9 and ack["ok"] is True and ack["profile"] == "stock"
    assert ack["imu"] == {"level": "ok"} and ack["bridge"]["last_seq"] == 9 and "error" not in ack
    assert math.isclose(ack["bridge"]["age_ms"], 100.0)
    bad = json.loads(db.make_ack(None, False, "not_json", "stock", None, core.snapshot(1.2)))
    assert bad["ok"] is False and bad["error"] == "not_json" and bad["imu"] is None


def test_rate_loop_50hz_no_burst():
    loop = db.RateLoop(50.0, 0.0)
    fires = [t for t in (i * 0.001 for i in range(1000)) if loop.due(t)]
    assert len(fires) == 50
    loop = db.RateLoop(50.0, 0.0)
    assert loop.due(0.0) and loop.due(0.5) and not loop.due(0.505)  # stall: one fire, no catch-up burst


def test_safe_summary_handles_missing_or_broken_health():
    class Broken:
        def summary(self):
            raise RuntimeError("x")

    class Good:
        def summary(self):
            return {"level": "warn", "tilt_deg": 3.0}

    assert db.safe_summary(None) is None
    assert db.safe_summary(Broken())["level"] == "unknown"
    assert db.safe_summary(Good()) == {"level": "warn", "tilt_deg": 3.0}
