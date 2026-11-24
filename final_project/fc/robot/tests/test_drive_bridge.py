"""Pure-logic tests for drive_bridge.py (no ROS needed)."""
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import drive_bridge as db  # noqa: E402

KEY = bytes(range(32))
ROBOT = "blue"


def msg(seq=1, vx=0.1, vy=0.0, wz=0.0, t=None, sid="s1", robot=ROBOT, **extra):
    m = {"v": 2, "robot": robot, "sid": sid, "seq": seq, "t": 1000.0 + seq * 0.01 if t is None else t,
         "vx": vx, "vy": vy, "wz": wz}
    m.update(extra)
    return m


def pkt(seq=1, vx=0.1, vy=0.0, wz=0.0, key=KEY, **extra):
    return db.sign(key, msg(seq, vx, vy, wz, **extra))


def raw(m, key=KEY):
    """Sign an arbitrary (possibly malformed) dict; json.dumps allows NaN here on purpose."""
    try:
        return db.sign(key, m)
    except ValueError:
        return json.dumps(m).encode()


def core(**kw):
    return db.BridgeCore(KEY, ROBOT, **kw)


# ---------------------------------------------------------------- parsing

def test_parse_valid_defaults_ttl():
    cmd = db.parse_packet(pkt(seq=7, vx=0.2, vy=-0.1, wz=0.5), KEY)
    assert (cmd.seq, cmd.vx, cmd.vy, cmd.wz, cmd.ttl, cmd.clamped) == (7, 0.2, -0.1, 0.5, 0.25, False)


@pytest.mark.parametrize("vx,vy,wz,expect", [
    (1.0, 0.0, 0.0, (0.5, 0.0, 0.0)),
    (-9.0, 9.0, -9.0, (-0.5, 0.3, -2.0)),
    (0.0, -0.31, 2.01, (0.0, -0.3, 2.0)),
])
def test_parse_clamps_to_hard_ceilings(vx, vy, wz, expect):
    cmd = db.parse_packet(pkt(vx=vx, vy=vy, wz=wz), KEY)
    assert (cmd.vx, cmd.vy, cmd.wz) == expect and cmd.clamped


def test_ttl_is_capped_and_must_be_positive():
    assert db.parse_packet(pkt(ttl=5.0), KEY).ttl == db.MAX_TTL
    assert db.parse_packet(pkt(ttl=0.1), KEY).ttl == 0.1
    for bad in (0, -1, "x", None, True):
        with pytest.raises(db.PacketError) as err:
            db.parse_packet(pkt(ttl=bad), KEY)
        assert err.value.reason == "bad_ttl"


def _without(key):
    m = msg()
    del m[key]
    return m


@pytest.mark.parametrize("data,reason", [
    (b"not json", "not_json"),
    (b"\xff\xfe", "not_json"),
    (b"[1,2]", "not_object"),
    (b"{" + b" " * 2000 + b"}", "too_large"),
    (json.dumps(msg()).encode(), "bad_mac"),                       # unsigned
    (pkt(key=b"x" * 32), "bad_mac"),                               # wrong key
    (json.dumps({**msg(), "mac": "00" * 32}).encode(), "bad_mac"),
    (json.dumps({**json.loads(pkt()), "vx": 0.4}).encode(), "bad_mac"),  # tampered field
    (b'{"v":2,"seq":1,"t":1,"vx":NaN,"vy":0,"wz":0,"mac":"' + b"0" * 64 + b'"}', "bad_mac"),
    (raw(msg(v=1)), "bad_version"),
    (raw(_without("seq")), "bad_seq"),
    (raw(msg(seq=-1)), "bad_seq"),
    (raw(msg(seq=True)), "bad_seq"),
    (raw(msg(seq=1.5)), "bad_seq"),
    (raw(_without("robot")), "bad_robot"),
    (raw(msg(sid=7)), "bad_sid"),
    (raw(msg(sid="has space")), "bad_sid"),
    (raw(_without("vx")), "bad_vx"),
    (raw(msg(vx="1")), "bad_vx"),
    (raw(msg(wz=False)), "bad_wz"),
    (raw(_without("t")), "bad_t"),
    (raw(msg(t="now")), "bad_t"),
])
def test_parse_rejects_malformed_or_unauthenticated(data, reason):
    with pytest.raises(db.PacketError) as err:
        db.parse_packet(data, KEY)
    assert err.value.reason == reason


def test_error_keeps_seq_when_known():
    with pytest.raises(db.PacketError) as err:
        db.parse_packet(raw(msg(seq=42, vx="fast")), KEY)
    assert err.value.seq == 42


def test_mac_is_over_canonical_json():
    m = msg(seq=3)
    signed = json.loads(pkt(seq=3))
    body = json.dumps(m, sort_keys=True, separators=(",", ":")).encode()
    import hashlib
    import hmac
    assert signed["mac"] == hmac.new(KEY, body, hashlib.sha256).hexdigest()
    # Key order / whitespace on the wire does not matter, only field values.
    shuffled = json.dumps(dict(reversed(list(signed.items()))), indent=1).encode()
    assert db.parse_packet(shuffled, KEY).seq == 3


def test_load_key(tmp_path):
    path = tmp_path / "link.key"
    with pytest.raises(db.KeyFileError, match="missing"):
        db.load_key(str(path))
    path.write_text(KEY.hex() + "\n")
    path.chmod(0o644)
    with pytest.raises(db.KeyFileError, match="chmod 600"):
        db.load_key(str(path))
    path.chmod(0o600)
    assert db.load_key(str(path)) == KEY
    path.write_text("abcd")
    with pytest.raises(db.KeyFileError, match="shorter"):
        db.load_key(str(path))
    path.write_text("zz")
    with pytest.raises(db.KeyFileError, match="not hex"):
        db.load_key(str(path))


# ---------------------------------------------------------------- admission

def test_good_packet_accepted_and_acked_with_valid_mac():
    c = core()
    assert c.on_packet(pkt(seq=1), 0.0) == (True, 1, None)
    ack = json.loads(c.make_ack(1, "stock", None, c.snapshot(0.0)))
    assert db.verify(KEY, ack) and not db.verify(b"y" * 32, ack)
    assert (ack["v"], ack["robot"], ack["sid"], ack["ack"], ack["ok"]) == (2, ROBOT, "s1", 1, True)


def test_unauthenticated_and_misaddressed_packets_rejected_and_counted():
    c = core()
    assert c.on_packet(pkt(seq=1, key=b"z" * 32), 0.0) == (False, None, "bad_mac")
    assert c.on_packet(pkt(seq=2, robot="red"), 0.01) == (False, 2, "wrong_robot")
    assert c.cmd is None and c.sid is None
    assert c.stats["auth"] == 1 and c.stats["robot"] == 1 and c.stats["ok"] == 0


def test_out_of_order_and_duplicate_rejected():
    c = core()
    assert c.on_packet(pkt(seq=5), 0.00) == (True, 5, None)
    assert c.on_packet(pkt(seq=5), 0.01) == (False, 5, "old_seq")
    assert c.on_packet(pkt(seq=3, vx=0.4), 0.02) == (False, 3, "old_seq")
    assert c.cmd.seq == 5 and c.cmd.vx == 0.1
    assert c.on_packet(pkt(seq=6), 0.03)[0]
    assert c.stats["replay"] == 2 and c.stats["ok"] == 2 and c.stats["rx"] == 4


def test_replay_after_silence_still_rejected():
    c = core()
    first = pkt(seq=1)
    assert c.on_packet(first, 0.0)[0]
    assert c.on_packet(pkt(seq=2), 0.1)[0]
    assert c.on_packet(first, 60.0) == (False, 1, "old_seq")   # no time-based restart any more


def test_new_sid_resumes_old_sid_never_again():
    c = core()
    assert c.on_packet(pkt(seq=900, sid="s1", t=1000.0), 0.0)[0]
    assert c.on_packet(pkt(seq=1, sid="s2", t=1005.0), 0.1) == (True, 1, None)   # laptop restarted
    assert c.sid == "s2" and c.stats["sessions"] == 2
    assert c.on_packet(pkt(seq=901, sid="s1", t=1005.1), 0.2) == (False, 901, "old_sid")
    assert c.on_packet(pkt(seq=2, sid="s2", t=1005.2), 0.3)[0]


def test_new_sid_needs_valid_mac():
    c = core()
    assert c.on_packet(pkt(seq=50, sid="s1"), 0.0)[0]
    assert c.on_packet(pkt(seq=1, sid="evil", key=b"q" * 32), 0.1) == (False, None, "bad_mac")
    assert c.sid == "s1" and c.on_packet(pkt(seq=51, sid="s1"), 0.2)[0]


def test_stale_t_rejected_but_forward_jumps_allowed():
    c = core()
    assert c.on_packet(pkt(seq=1, t=1000.0), 0.0)[0]
    assert c.on_packet(pkt(seq=2, t=1030.0), 0.1)[0]                 # laptop paused 30 s: fine
    assert c.on_packet(pkt(seq=1, sid="rec", t=1027.0), 0.2) == (False, 1, "stale_t")   # recorded older session
    assert c.on_packet(pkt(seq=3, t=1028.5), 0.3)[0]                 # small clock step back: inside window


def test_allow_from():
    c = core(allow_from={"10.0.0.5"})
    assert c.on_packet(pkt(seq=1), 0.0, "10.0.0.6") == (False, None, "denied_source")
    assert c.on_packet(pkt(seq=1), 0.1, "10.0.0.5")[0]
    assert c.stats["denied"] == 1


def test_bad_packet_does_not_replace_command():
    c = core()
    c.on_packet(pkt(seq=1, vx=0.2), 0.0)
    ok, seq, err = c.on_packet(b"garbage", 0.01)
    assert not ok and err == "not_json" and c.cmd.vx == 0.2 and c.stats["bad"] == 1


# ---------------------------------------------------------------- dead-man

def test_idle_bridge_is_silent():
    c = core()
    assert all(c.tick(t * 0.02) is None for t in range(50))
    assert c.stats["pub"] == 0 and c.stats["zero"] == 0


def test_publishes_until_ttl_then_one_zero_then_silent():
    c = core()
    c.on_packet(pkt(seq=1, vx=0.3, wz=-1.0, ttl=0.25), 10.0)
    outs = [c.tick(10.0 + i * 0.02) for i in range(30)]  # 0 .. 0.58 s
    live = [o for o in outs if o not in (None, (0.0, 0.0, 0.0))]
    assert len(live) == 13 and all(o == (0.3, 0.0, -1.0) for o in live)  # t < 0.25 s
    assert outs[13] == (0.0, 0.0, 0.0)
    assert all(o is None for o in outs[14:])
    assert c.stats["zero"] == 1 and not c.publishing


def test_fresh_packets_keep_it_alive_and_restart_after_expiry():
    c = core()
    t = 0.0
    for seq in range(1, 50):  # 30 Hz sender for ~1.6 s
        c.on_packet(pkt(seq=seq, vx=0.2), t)
        for k in range(2):
            assert c.tick(t + k * 0.016) == (0.2, 0.0, 0.0)
        t += 1 / 30
    assert c.tick(t + 0.3) == (0.0, 0.0, 0.0)
    assert c.tick(t + 0.32) is None
    c.on_packet(pkt(seq=100, vx=-0.1), t + 0.4)
    assert c.tick(t + 0.41) == (-0.1, 0.0, 0.0)


def test_shutdown_zero_only_when_driving():
    c = core()
    assert c.shutdown() is None
    c.on_packet(pkt(seq=1), 0.0)
    c.tick(0.01)
    assert c.shutdown() == (0.0, 0.0, 0.0)
    assert c.shutdown() is None and c.tick(0.02) is None


def test_commanding_zero():
    c = core()
    assert c.commanding_zero(0.0)
    c.on_packet(pkt(seq=1, vx=0.0, vy=0.0, wz=0.0), 0.0)
    assert c.commanding_zero(0.1)
    c.on_packet(pkt(seq=2, wz=0.5, vx=0.0), 0.1)
    assert not c.commanding_zero(0.2)
    assert c.commanding_zero(0.1 + 0.26)


# ---------------------------------------------------------------- ack + rate

def test_ack_shape():
    c = core()
    ok, seq, err = c.on_packet(pkt(seq=9), 1.0)
    ack = json.loads(c.make_ack(seq, "stock", {"level": "ok"}, c.snapshot(1.1)))
    assert ack["v"] == 2 and ack["ack"] == 9 and ack["ok"] is True and ack["profile"] == "stock"
    assert ack["imu"] == {"level": "ok"} and ack["bridge"]["last_seq"] == 9 and ack["bridge"]["robot_id"] == ROBOT
    assert math.isclose(ack["bridge"]["age_ms"], 100.0)


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


def test_ack_carries_hold_fields():
    c = core()
    c.on_packet(pkt(seq=1), 0.0)
    ack = json.loads(c.make_ack(1, "stock", None, c.snapshot(0.0), hold="imu_stale", imu_hold=True, mux_ok=True))
    assert db.verify(KEY, ack) and (ack["hold"], ack["imu_hold"], ack["mux_ok"]) == ("imu_stale", True, True)
    plain = json.loads(c.make_ack(1, "stock", None, c.snapshot(0.0)))
    assert (plain["hold"], plain["imu_hold"], plain["mux_ok"]) == (None, False, None)


# ---------------------------------------------------------------- hold (IMU / mux) in the publish path

def test_hold_publishes_zero_while_fresh_then_normal_expiry():
    c = core()
    c.on_packet(pkt(seq=1, vx=0.3, ttl=0.25), 0.0)
    assert c.tick(0.02, hold=True) == (0.0, 0.0, 0.0)
    assert c.tick(0.04, hold=False) == (0.3, 0.0, 0.0)       # released: command resumes
    assert c.tick(0.06, hold=True) == (0.0, 0.0, 0.0)
    assert c.tick(0.30, hold=True) == (0.0, 0.0, 0.0)        # expired: one zero
    assert c.tick(0.32, hold=True) is None                   # then silence
    assert c.stats["held"] == 2 and c.stats["pub"] == 1


def test_commanding_zero_true_under_hold():
    c = core()
    c.on_packet(pkt(seq=1, vx=0.3), 0.0)
    assert not c.commanding_zero(0.1) and c.commanding_zero(0.1, hold=True)


# ---------------------------------------------------------------- ImuGuard

def bad(cause):
    return {"level": "bad", "bad_cause": cause}


def test_imu_guard_holds_on_stale_or_tilt_and_releases_after_1s_good():
    for cause in ("stale", "tilt"):
        g = db.ImuGuard(clear_s=1.0)
        assert not g.update(0.0, {"level": "ok"})
        assert g.update(0.1, bad(cause)) and g.cause == cause
        assert g.update(0.2, {"level": "warn"})              # good for 0 s
        assert g.update(1.1, {"level": "ok"})                # 0.9 s
        assert not g.update(1.2, {"level": "ok"})            # 1.0 s -> release
        assert g.cause is None


def test_imu_guard_good_run_restarts_on_bad_or_unknown():
    g = db.ImuGuard(clear_s=1.0)
    g.update(0.0, bad("stale"))
    g.update(0.5, {"level": "ok"})
    g.update(1.0, bad("stale"))                              # relapse: clock restarts
    assert g.update(1.9, {"level": "ok"}) and g.update(2.5, {"level": "ok"})
    g.update(2.6, {"level": "unknown"})                      # broken summary: keeps hold, resets run
    assert g.update(3.0, {"level": "ok"}) and g.update(3.9, {"level": "ok"})
    assert not g.update(4.0, {"level": "ok"})


def test_imu_guard_ignores_other_bad_causes_and_missing_health():
    g = db.ImuGuard()
    assert not g.update(0.0, {"level": "bad", "bad_cause": None})
    assert not g.update(0.1, None) and not g.update(0.2, {"level": "warn", "reasons": ["no IMU samples"]})


def test_imu_guard_with_real_imu_health_stale_stream():
    import imu_health
    clock = [0.0]
    h = imu_health.ImuHealth({}, clock=lambda: clock[0])
    g = db.ImuGuard()
    for i in range(100):                                     # 100 Hz, level, moving gyro values
        clock[0] = i * 0.01
        h.update(clock[0], (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.001 * (i % 3)), True)
    assert not g.update(clock[0], h.summary())
    clock[0] += 0.6                                          # stream froze for 600 ms
    s = h.summary()
    assert s["level"] == "bad" and s["bad_cause"] == "stale" and g.update(clock[0], s)


# ---------------------------------------------------------------- MuxGuard

def test_mux_guard_unseen_until_graph_shows_one_mux():
    m = db.MuxGuard()
    assert m.check(0.0, None) == "mux_unseen"
    m.on_graph(0, 0)
    assert m.check(0.1, None) == "mux_unseen" and not m.emergency(0.1)   # never seen: no emergency
    m.on_graph(1, 1)
    assert m.check(0.2, None) is None


def test_mux_guard_contention_and_missing_subscription_hold_without_emergency():
    m = db.MuxGuard()
    m.on_graph(1, 1)
    m.check(0.0, None)
    m.on_graph(2, 1)
    assert m.check(0.1, None) == "cmd_vel_publishers=2" and not m.emergency(0.1)
    m.on_graph(1, 0)
    assert m.check(0.2, None) == "mux_not_subscribed" and not m.emergency(0.2)


def test_mux_guard_gone_triggers_one_emergency_window():
    m = db.MuxGuard()
    m.on_graph(1, 1)
    assert m.check(0.0, None) is None
    m.on_graph(0, 0)
    assert m.check(1.0, None) == "no_cmd_vel_publisher"
    assert m.emergency(1.0) and m.emergency(1.0 + db.EMERGENCY_S - 0.01) and not m.emergency(1.0 + db.EMERGENCY_S)
    m.check(5.0, None)                                        # still gone: no new window
    assert not m.emergency(5.0)
    m.on_graph(1, 1)
    assert m.check(6.0, None) is None                         # mux back


def test_mux_guard_stall_only_while_publishing():
    m = db.MuxGuard(stall_s=0.5)
    m.on_graph(1, 1)
    m.on_cmd_vel(0.0)
    assert m.check(2.0, None) is None                         # bridge idle: quiet /cmd_vel is normal
    assert m.check(2.0, 1.9) is None                          # just started publishing
    m.on_cmd_vel(2.3)
    assert m.check(2.6, 1.9) is None                          # mux forwarding
    assert m.check(2.8, 1.9) == "mux_stalled" and m.emergency(2.8)
    m.on_cmd_vel(2.9)
    assert m.check(2.95, 1.9) is None                         # traffic resumed (held zeros keep it flowing)


def test_mux_guard_with_core_hold_end_to_end():
    """Mux dies mid-drive: the bridge holds zero, and the emergency window opens once."""
    c, m = core(), db.MuxGuard()
    m.on_graph(1, 1)
    since, outs = None, []
    for i in range(100):                                      # 50 Hz for 2 s, laptop at 50 Hz too
        now = i * 0.02
        c.on_packet(pkt(seq=i + 1, vx=0.2, t=1000.0 + now), now)
        if i < 50:
            m.on_cmd_vel(now)                                 # mux alive for the first second
        hold = m.check(now, since) is not None
        outs.append(c.tick(now, hold=hold))
        since = (since if since is not None else now) if c.publishing else None
    assert all(o == (0.2, 0.0, 0.0) for o in outs[:70])       # ~0.5 s stall detection
    assert outs[-1] == (0.0, 0.0, 0.0) and m.why == "mux_stalled"
    assert m.emergency(1.5) and not m.emergency(1.5 + db.EMERGENCY_S + 0.5)


# ---------------------------------------------------------------- replay state persistence

def test_replay_state_roundtrip_rejects_old_session_after_restart(tmp_path):
    path = str(tmp_path / "state.json")
    c = core()
    assert c.on_packet(pkt(seq=10, sid="s1", t=1000.0), 0.0)[0]
    assert c.on_packet(pkt(seq=1, sid="s2", t=1010.0), 0.1)[0]
    assert c.on_packet(pkt(seq=2, sid="s2", t=1010.1), 0.2)[0]
    db.save_replay_state(path, c.replay_state())
    assert oct(os.stat(path).st_mode & 0o777) == "0o600"

    fresh = core()                                            # bridge restarted
    fresh.restore_replay(db.load_replay_state(path))
    assert fresh.on_packet(pkt(seq=11, sid="s1", t=1010.2), 0.0) == (False, 11, "old_sid")
    assert fresh.on_packet(pkt(seq=2, sid="s2", t=1010.1), 0.0) == (False, 2, "old_seq")
    assert fresh.on_packet(pkt(seq=5, sid="rec", t=1007.0), 0.0) == (False, 5, "stale_t")
    assert fresh.on_packet(pkt(seq=3, sid="s2", t=1010.2), 0.1) == (True, 3, None)   # running laptop continues
    assert fresh.on_packet(pkt(seq=1, sid="s3", t=1020.0), 0.2) == (True, 1, None)   # restarted laptop


def test_without_state_a_restart_would_accept_the_replay():
    c = core()
    c.on_packet(pkt(seq=10, sid="s1", t=1000.0), 0.0)
    c.on_packet(pkt(seq=1, sid="s2", t=1010.0), 0.1)
    assert core().on_packet(pkt(seq=10, sid="s1", t=1000.0), 0.0)[0]     # the hole persistence closes


def test_replay_state_bounded_and_missing_file(tmp_path):
    c = core()
    for i in range(db.MAX_RETIRED_SIDS + 10):
        assert c.on_packet(pkt(seq=1, sid="s%d" % i, t=1000.0 + i), i * 0.1)[0]
    st = c.replay_state()
    assert len(st["retired"]) == db.MAX_RETIRED_SIDS and st["sid"] == "s%d" % (db.MAX_RETIRED_SIDS + 9)
    path = str(tmp_path / "s.json")
    db.save_replay_state(path, st)
    assert len(json.load(open(path))["retired"]) == db.MAX_RETIRED_SIDS
    assert db.load_replay_state(str(tmp_path / "none.json")) is None
    assert core().replay_state() == {"newest_t": None, "sid": None, "last_seq": -1, "retired": []}


@pytest.mark.parametrize("content", [
    "not json", "[]", '{"newest_t": "x", "sid": null, "last_seq": -1, "retired": []}',
    '{"newest_t": 1.0, "sid": "bad sid!", "last_seq": -1, "retired": []}',
    '{"newest_t": 1.0, "sid": null, "last_seq": true, "retired": []}',
    '{"newest_t": 1.0, "sid": null, "last_seq": 3, "retired": [5]}',
    '{"newest_t": 1.0, "sid": null, "last_seq": 3}',
])
def test_replay_state_malformed_fails_closed(tmp_path, content):
    path = tmp_path / "s.json"
    path.write_text(content)
    with pytest.raises(ValueError, match="reset-replay"):
        db.load_replay_state(str(path))


def test_replay_state_overwrite_keeps_0600(tmp_path):
    path = tmp_path / "s.json"
    path.write_text("{}")
    os.chmod(path, 0o644)
    db.save_replay_state(str(path), core().replay_state())
    assert os.stat(path).st_mode & 0o777 == 0o600 and db.load_replay_state(str(path))["last_seq"] == -1


# ---------------------------------------------------------------- SPEC v1.1: e-stop, fallen cutoff, vx ceiling

def test_estop_packet_parses_with_zero_motion_whatever_it_says():
    c = db.parse_packet(pkt(seq=3, vx=0.4, wz=1.0, estop=True), KEY)
    assert c.estop is True and (c.vx, c.vy, c.wz) == (0.0, 0.0, 0.0)
    assert db.parse_packet(pkt(seq=4), KEY).estop is False
    with pytest.raises(db.PacketError) as e:
        db.parse_packet(raw(msg(5, estop="yes")), KEY)
    assert e.value.reason == "bad_estop"


def test_estop_latches_publishes_zero_and_acks_estopped_idempotently():
    c = core()
    assert c.on_packet(pkt(seq=1, vx=0.3), 0.0)[0] and c.tick(0.01) == (0.3, 0.0, 0.0)
    ok, seq, _ = c.on_packet(pkt(seq=2, estop=True), 0.02)
    assert ok and c.estopped and c.estop_reason == "link"
    assert not c.trip("link")                                    # the R3 action fires once
    for s in range(3, 8):                                        # the laptop's resend burst: all admitted, no re-trip
        assert c.on_packet(pkt(seq=s, estop=True), 0.02 + s * 0.02)[0]
    assert c.stats["estop"] == 6
    ack = json.loads(c.make_ack(seq, "stock", None, c.snapshot(0.03)))
    assert ack["estopped"] is True and ack["hold"] == "estopped" and ack["bridge"]["estop_reason"] == "link"
    assert c.on_packet(pkt(seq=20, vx=0.4), 0.5)[0]              # motion afterwards is admitted but never published
    assert c.tick(0.51) == (0.0, 0.0, 0.0) and c.commanding_zero(0.51)
    assert c.tick(0.9) == (0.0, 0.0, 0.0) and c.tick(1.0) is None   # ttl expiry still goes silent
    assert json.loads(core().make_ack(1, "stock", None, {}))["estopped"] is False


def test_park_publishes_zero_but_still_admits_the_estop():
    """fcctl down/stop park the bridge (SIGUSR1) before the gentle lie-down: motion is never published again,
    but a link E-STOP still trips (R3 action deactivates the lie-down controller mid-blend)."""
    c = core()
    assert c.on_packet(pkt(seq=1, vx=0.3), 0.0)[0] and c.tick(0.01) == (0.3, 0.0, 0.0)
    assert c.park() and not c.park()
    assert c.tick(0.02) == (0.0, 0.0, 0.0) and c.commanding_zero(0.02)
    assert c.on_packet(pkt(seq=2, vx=0.4), 0.1)[0] and c.tick(0.11) == (0.0, 0.0, 0.0)
    ack = json.loads(c.make_ack(2, "stock", None, c.snapshot(0.11)))
    assert ack["hold"] == "parked" and ack["bridge"]["parked"] is True and ack["estopped"] is False
    ok, seq, _ = c.on_packet(pkt(seq=3, estop=True), 0.2)
    assert ok and c.estopped and c.estop_reason == "link"
    assert json.loads(c.make_ack(seq, "stock", None, {}))["hold"] == "estopped"


def test_estop_packet_still_needs_auth_robot_and_freshness():
    c = core()
    assert not c.on_packet(pkt(seq=1, estop=True, key=b"x" * 32), 0.0)[0]
    assert not c.on_packet(pkt(seq=1, estop=True, robot="red"), 0.0)[0]
    assert c.on_packet(pkt(seq=5), 0.0)[0] and not c.on_packet(pkt(seq=5, estop=True), 0.1)[0]
    assert not c.estopped


def test_max_vx_follows_stack_speed_cap_with_hard_cap():
    assert db.parse_packet(pkt(vx=0.9), KEY).vx == db.MAX_VX
    c = core(max_vx=0.8)
    assert c.on_packet(pkt(seq=1, vx=0.75), 0.0)[0] and c.cmd.vx == 0.75 and not c.cmd.clamped
    assert c.on_packet(pkt(seq=2, vx=0.95), 0.1)[0] and c.cmd.vx == 0.8 and c.cmd.clamped
    for bad in (0.0, 1.2):
        with pytest.raises(ValueError):
            core(max_vx=bad)


def test_fallen_guard_needs_sustained_tilt():
    g = db.FallenGuard(45.0, 0.5)
    assert not g.update(0.0, 60.0) and not g.update(0.4, 60.0)
    assert not g.update(0.45, 30.0)                              # dip below the limit restarts the run
    assert not g.update(0.5, 50.0) and not g.update(0.9, 50.0) and g.update(1.0, 50.0)
    assert not g.update(1.1, float("nan")) and not g.update(1.7, 50.0)
    assert not db.FallenGuard(0.0).update(9.0, 90.0)             # disabled
    assert math.isclose(db.quat_tilt_deg((0, 0, 0, 1)), 0.0, abs_tol=1e-9)
    s = math.sin(math.radians(30))
    assert math.isclose(db.quat_tilt_deg((s, 0, 0, math.cos(math.radians(30)))), 60.0, abs_tol=1e-6)
    assert math.isnan(db.quat_tilt_deg((0, 0, 0, 0)))


def test_r3_deactivate_list_only_names_profile_legs():
    legs = ["neural_controller_default", "neural_controller_liedown", "trick"]
    st = {"neural_controller_liedown": "active", "joint_state_broadcaster": "active", "trick": "inactive"}
    assert db.r3_deactivate_list(st, "neural_controller_default", legs) == ["neural_controller_liedown",
                                                                            "neural_controller_default"]
    assert db.r3_deactivate_list(None, "neural_controller_default", legs) == ["neural_controller_default"]


def test_r3_tracker_sends_once_retries_and_confirms():
    t = db.R3Tracker(["w", "x"], retry_s=1.0, max_tries=3)
    assert not t.due(0.0) and t.fire(0.0) and not t.fire(0.01)
    assert t.due(0.0)
    t.sent(0.002)
    assert not t.due(0.5)
    assert not t.on_states(0.3, {"w": "active"}) and t.due(1.1)
    t.sent(1.1)
    t.on_response(1.12, True)
    assert t.on_states(1.15, {"w": "inactive", "x": "unconfigured"}) and t.confirmed and not t.due(5.0)
    lat = t.latency_ms()
    assert lat["switch_sent"] == 2.0 and lat["confirmed"] == 1150.0
    t2 = db.R3Tracker(["w"], retry_s=0.1, max_tries=2)
    t2.fire(0.0); t2.sent(0.0); t2.sent(0.2)
    assert not t2.due(9.0)                                       # gives up after max tries (logged by the caller)


def test_latch_file_roundtrip_and_fail_closed(tmp_path):
    p = tmp_path / "estop.json"
    assert db.read_latch(str(p)) is None
    db.write_latch(str(p), {"reason": "fallen", "tilt_deg": 70.0})
    assert db.read_latch(str(p))["reason"] == "fallen" and (p.stat().st_mode & 0o777) == 0o600
    p.write_text("{broken")
    assert db.read_latch(str(p)) == {"reason": "unreadable_latch_file"}
    p.write_text("[1]")
    assert db.read_latch(str(p))["reason"] == "malformed_latch_file"
