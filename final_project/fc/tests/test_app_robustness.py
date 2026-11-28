"""Robustness fixes (Nov 27 audit): halted / link-dead robots out of the Coach and the kickoff wait, field frozen
during a run, ssh keepalives and op cleanup, re-arm pauses a match, default robots / preset, human goals, Link
re-resolve, Space pauses a match."""
import json
import threading
import time

import pytest

import fc.app as fcapp
from fc.app import App, default_robot_ids, restart_argv, start_preset
from fc.core import Ball, Pose, RobotCfg, RobotObs, World, load_field
from fc.play import MatchFlow


def wait(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def running(tmp_path):
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    app.start()
    assert wait(lambda: app.world is not None and len(app.world.robots) == 2, 3)
    yield app
    app.close()


@pytest.fixture
def idle(tmp_path):
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    yield app
    app.close()


# ---- 3: the Coach drops halted and link-dead robots -------------------------------------------------
def test_coach_ignores_a_halted_or_link_dead_robot(running):
    app = running
    with app.lock:
        app.mode, app.playing = "play", ["blue", "red"]
        try:
            assert set(app._coach_tasks(app.world)) == {"blue", "red"}
            app._latch("red", "operator", confirmed=True, reason="test")
            assert app.out_of_play("red") == "estopped" and app._coach_tasks(app.world) == {}   # blue plays alone
            app.estops.clear()
            app.link, app._lstat = object(), {"blue": {"connected": True}, "red": {"connected": False}}
            assert app.out_of_play("red") == "no_link" and app.out_of_play("blue") is None
            assert app._coach_tasks(app.world) == {}
        finally:
            app.link, app._lstat, app.mode = None, {}, "idle"


def test_estopped_robot_does_not_make_the_other_shadow_it(running):
    """Sim regression: red e-stopped while near the ball kept blue in defend for 65 s. Now blue plays alone."""
    app = running
    assert app.action({"action": "play_here"})["ok"]            # a match goes on when one robot is e-stopped
    with app.lock:
        app.flow._set("play", time.monotonic())
    assert wait(lambda: app.tasks, 3)
    app.estop("red")
    assert wait(lambda: app.tasks == {} and (app.debug.get("blue") or {}).get("solo") == 1, 3)
    assert app.mode == "match" and app.flow.phase == "play"
    assert app.cmds["red"].vx == 0 and app.cmds["red"].wz == 0
    app.action({"action": "end"})


# ---- 2b: no field resize while something runs ------------------------------------------------------
def test_field_recalibration_waits_for_idle(idle):
    app = idle

    class Eye:
        def calibrated_size(self):
            return (3.20, 2.10)
    app.eye = Eye()
    try:
        L, W = app.field.length, app.field.width
        app.test = {"name": "match"}
        app._sync_field()
        assert (app.field.length, app.field.width) == (L, W)
        assert any("applied when match ends" in e["msg"] for e in app.events)
        app.test = None
        app._sync_field()
        assert (app.field.length, app.field.width) == (3.20, 2.10) and app.field_source == "camera"
    finally:
        app.eye = None


# ---- 5: kickoff does not wait for halted / absent robots; hostless robots not active by default ----
def _world(now, ball=(0.0, 0.0), robots=None):
    return World(now, Ball(ball[0], ball[1], 0.0, 0.0, now),
                 {k: RobotObs(k, Pose(*p), now) for k, p in (robots or {}).items()})


def test_kickoff_counts_out_of_play_robots_as_in_position():
    f = load_field()
    cfg = [RobotCfg("blue", "blue", 10, "", "stock"), RobotCfg("red", "red", 11, "", "stock")]
    p = {"match_s": 300}
    for out, want in (({}, "kickoff"), ({"red": "estopped"}, "countdown")):
        flow, now = MatchFlow(cfg, f, 0.0), 0.0
        for _ in range(60):      # 2 s with the ball still at the centre, blue at its spot, red never there
            now += 1 / 30
            flow.update(now, 1 / 30, _world(now), {"blue": {"done": True}, "red": {}}, [], p, out=out)
        assert flow.phase == want, out
    assert flow.json(now, p)["out"] == {"red": "estopped"}


def test_app_kickoff_excuses_halted_and_long_unseen_robots(idle):
    app = idle
    now = time.monotonic()
    app.flow = MatchFlow([app.cfg["blue"], app.cfg["red"]], app.field, now)
    w = _world(now, robots={"blue": (-1, 0, 0), "red": (1, 0, 3.14)})
    assert app._out_of_kickoff(w, now) == {}
    app._latch("red", "operator", confirmed=True)
    assert app._out_of_kickoff(w, now) == {"red": "estopped"}
    app.estops.clear()
    gone = _world(now, robots={"blue": (-1, 0, 0)})        # the eye dropped red (after 1 s unseen)
    assert app._out_of_kickoff(gone, now + 1.0) == {}       # a short dropout: still waited for
    later = now + fcapp.KICKOFF_ABSENT_S + 0.1
    assert app._out_of_kickoff(_world(later, robots={"blue": (-1, 0, 0)}), later) == {"red": "not seen"}
    app.flow = None


def test_sim_kickoff_proceeds_when_a_robot_is_estopped(running):
    app = running
    assert app.action({"action": "kickoff"})["ok"]
    app.estop("red")
    assert wait(lambda: app.flow is not None and app.flow.phase in ("countdown", "play"), 20)
    assert app.flow.in_pos["red"] is True and app.flow.out == {"red": "estopped"}
    app.action({"action": "end"})


def test_default_robots_skip_hostless_on_camera():
    blue = RobotCfg("blue", "blue", 10, "mts", "stock")
    red = RobotCfg("red", "red", 11, "", "stock")
    assert default_robot_ids("sim", [blue, red], {}) == ["blue", "red"]
    assert default_robot_ids("camera", [blue, red], {}) == ["blue"]
    assert default_robot_ids("camera", [blue, red], {"red": "10.0.0.9"}) == ["blue", "red"]   # udp_host counts
    nohost = RobotCfg("blue", "blue", 10, "", "stock")
    assert default_robot_ids("camera", [nohost, red], {}) == ["blue", "red"]   # none has one: offline rehearsal


# ---- 6: ssh keepalives; a reader error never leaves an op running ----------------------------------
def test_ssh_has_keepalives_and_reader_errors_end_the_op(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(fcapp.subprocess, "run",
                        lambda cmd, **kw: calls.append(cmd) or fcapp.subprocess.CompletedProcess(cmd, 0, "{}\n", ""))

    class BadProc:
        def __init__(self, cmd, **kw):
            calls.append(cmd)
            self.stdout = self._lines()
            self.killed = False

        def _lines(self):
            yield '{"phase":"starting","ok":true}\n'
            raise UnicodeDecodeError("utf-8", b"\xff", 0, 1, "invalid start byte")

        def wait(self):
            return 0

        def kill(self):
            self.killed = True

    monkeypatch.setattr(fcapp.subprocess, "Popen", BadProc)
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    try:
        assert app.robot_op({"robot": "blue", "op": "status"})["ok"]
        assert wait(lambda: calls and not app.ops["blue"]["running"], 2)
        opts = " ".join(calls[0])
        assert "ServerAliveInterval=5" in opts and "ServerAliveCountMax=3" in opts and "ConnectTimeout=5" in opts
        assert app.robot_op({"robot": "blue", "op": "ready", "confirm": True})["ok"]
        assert wait(lambda: not app.ops["blue"]["running"], 2)
        st = app.ops["blue"]
        assert st["rc"] == -1 and st["progress"]["ok"] is False and "UnicodeDecodeError" in st["progress"]["error"]
        assert app.robot_op({"robot": "blue", "op": "status"})["ok"]          # the next op is not blocked
    finally:
        app.close()


# ---- 7: a re-arm mid-match pauses the match ----------------------------------------------------------
def _fake_rearm_proc(monkeypatch, gate, lines=('{"phase":"ready","elapsed":9.0,"ok":true,"done":true}\n',), rc=0):
    monkeypatch.setattr(fcapp.subprocess, "run",
                        lambda cmd, **kw: fcapp.subprocess.CompletedProcess(cmd, 0, "{}\n", ""))

    class Proc:
        def __init__(self, cmd, **kw):
            self.stdout = self._lines()

        def _lines(self):
            gate.wait(2)
            yield from lines

        def wait(self):
            return rc

        def kill(self):
            pass

    monkeypatch.setattr(fcapp.subprocess, "Popen", Proc)


@pytest.mark.parametrize("op", ["ready", "getup"])
def test_rearm_mid_match_holds_that_robot_briefly_and_the_match_goes_on(tmp_path, monkeypatch, op):
    """Fall recovery (Dec 3: manual Get up): the re-armed robot holds zero for REJOIN_HOLD_S, out of the Coach, then
    plays from a fresh brain; the match is never paused for it (the other robot kept playing all along)."""
    gate = threading.Event()
    _fake_rearm_proc(monkeypatch, gate)
    monkeypatch.setattr(fcapp, "REJOIN_HOLD_S", 0.3)
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    try:
        with app.lock:
            assert app._play_action("play_here", {})["ok"]
            app.flow._set("play", time.monotonic())
        app.estop("blue")
        assert app.flow.phase == "play"                       # the e-stop alone does not pause (the other plays on)
        assert app.robot_op({"robot": "blue", "op": op, "confirm": True})["ok"]
        gate.set()
        assert wait(lambda: "blue" not in app.estops, 2)
        with app.lock:
            assert app.flow.phase == "play" and app.halted("blue") == "rejoining" and app.halted("red") is None
            assert app.out_of_play("blue") == "rejoining"     # the Coach and the kickoff leave it out meanwhile
            cmd, dbg, src = app._decide(app.cfg["blue"], app.world, time.monotonic())
            assert src == "hard" and (cmd.vx, cmd.vy, cmd.wz) == (0, 0, 0) and cmd.tag == "rejoining"
        assert any("rejoins in" in e["msg"] and "match goes on" in e["msg"] for e in app.events)
        assert wait(lambda: app.halted("blue") is None, 2)
        assert app.flow.phase == "play"
    finally:
        app.close()


def test_rearm_while_idle_has_no_hold(idle):
    app = idle
    with app.lock:
        app.estop("red")
        assert app._estop_clear_sim("red")["cleared"] == ["red"]
        assert app.halted("red") is None                     # nothing drives it at once: no hold to wait out


def test_sim_estop_clear_mid_match_rejoins_without_pausing(idle):
    app = idle
    with app.lock:
        assert app._play_action("play_here", {})["ok"]
        app.flow._set("play", time.monotonic())
        app.estop("red")
        assert app._estop_clear_sim("red")["cleared"] == ["red"]
        assert app.flow.phase == "play" and app.halted("red") == "rejoining"
        app.estop("red")                                      # an e-stop during the hold wins at once
        assert app.halted("red") == "estopped" and "red" not in app.rejoin


def test_estop_pressed_during_getup_is_kept_and_resent_to_the_new_bridge(tmp_path, monkeypatch):
    """A fallen robot is latched by the robot. An operator E-STOP pressed while Get up runs must not be absorbed by that
    old latch (the op's success would clear it): it becomes a fresh operator latch, kept after the op, and resent
    while the op restarts the bridge (and ESTOP_RESEND_S after) so the new bridge trips on it."""
    gate = threading.Event()
    _fake_rearm_proc(monkeypatch, gate)
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    try:
        with app.lock:
            app._latch("blue", "robot", confirmed=True, reason="fallen")
        assert app.robot_op({"robot": "blue", "op": "getup", "confirm": True})["ok"]
        t0 = app.ops["blue"]["t0"]
        app.estop("blue")
        e = app.estops["blue"]
        assert e["by"] == "operator" and e["t"] >= t0 and "getup" in e["reason"]
        gate.set()
        assert wait(lambda: not app.ops["blue"]["running"], 2)
        assert "blue" in app.estops and app.halted("blue") == "estopped"      # still latched after a "successful" op
        assert any("pressed during Get up" in ev["msg"] for ev in app.events)
        # resend rule (link path): unconfirmed operator press, op running or just ended -> e-stop packets
        now = time.monotonic()
        with app.lock:
            app.estops["blue"].update(confirmed=False, t=now - 60)           # long ago: only the re-arm keeps it
            assert app._estop_pending(now) == {"blue"}
            app.ops["blue"]["t_end"] = now - fcapp.ESTOP_RESEND_S - 1
            assert app._estop_pending(now) == set()
            app.ops["blue"]["running"] = True
            assert app._estop_pending(now) == {"blue"}
    finally:
        app.close()


def test_getup_failure_keeps_the_latch(tmp_path, monkeypatch):
    gate = threading.Event()
    gate.set()
    _fake_rearm_proc(monkeypatch, gate, lines=(
        '{"phase":"begin","ok":true,"elapsed":0.1,"msg":"profile stock (get up: no homing)","getup":true}\n',
        '{"phase":"error","ok":false,"elapsed":3.2,"msg":"robot not upright (tilt 76.0 deg, limit 25): set it on its '
        'belly or feet, then Get up","at":"upright","upright":false,"tilt_deg":76.0,"exit":6,"done":true}\n'), rc=6)
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    try:
        with app.lock:
            app._latch("blue", "robot", confirmed=True, reason="fallen")
        assert app.robot_op({"robot": "blue", "op": "getup", "confirm": True})["ok"]
        assert wait(lambda: not app.ops["blue"]["running"], 2)
        op = app.ops["blue"]
        assert op["rc"] == 6 and "belly" in op["progress"]["error"] and "blue" in app.estops
        assert app.robot_op({"robot": "blue", "op": "getup"})["error"] == "getup stands the robot up: send confirm=true"
    finally:
        app.close()


# ---- 9: default preset ---------------------------------------------------------------------------------
def test_start_preset_defaults_to_play():
    assert start_preset(None, ["play", "slow"]) == "play"
    assert start_preset(None, ["slow"]) is None                  # no play.json: base
    assert start_preset("base", ["play"]) is None and start_preset("slow", ["play", "slow"]) == "slow"
    assert restart_argv("sim", None, ["--port", "1"]) == ["--source", "sim", "--port", "1", "--preset", "base"]
    assert (fcapp.PRESETS / "play.json").exists()


# ---- 10: a human's goal is a plain goal pause -------------------------------------------------------
def test_goal_by_a_team_without_a_playing_robot_does_not_celebrate():
    f = load_field()
    p = {"match_s": 300, "celebrate_style": "spin"}
    for scorer, want in (("red", "goal"), ("blue", "celebrate")):
        flow = MatchFlow([RobotCfg("blue", "blue", 10, "", "stock")], f, 0.0, kickoff=False)
        flow._set("play", 0.0)
        flow.update(0.1, 1 / 30, _world(0.1), {}, [scorer], p)
        assert flow.phase == want and flow.score[scorer] == 1, scorer


# ---- 11: robot 2 prep ------------------------------------------------------------------------------------
def test_robot_colors_red_placeholder_is_valid_and_ignored():
    path = fcapp.CONFIG / "robot_colors.json"
    data = json.loads(path.read_text())
    assert "red" in data and data["red"]["colors"] == []
    from fc.eye import load_robot_colors
    cols = load_robot_colors(path)
    assert "red" not in cols and "blue" in cols                  # no colour yet: no blob tracking for red


def test_link_re_resolves_after_a_failure(tmp_path):
    from fc.link import Link
    from fc.robot.keygen import write_key
    key = write_key(tmp_path / "link.key")
    answers = [None, "127.0.0.1"]
    asked = []

    def resolve(host):
        asked.append(host)
        return answers.pop(0) if answers else "127.0.0.1"
    link = Link([RobotCfg("blue", "blue", 10, "mts", "stock")], {}, resolve=resolve, key_file=key)
    try:
        assert wait(lambda: link.status()["blue"]["error"] == "cannot resolve mts", 2)
        link._resolve_t["blue"] -= 10                           # past the retry throttle
        link.send({"blue": fcapp.Cmd.stop()})                   # no address: schedules a re-resolve
        assert wait(lambda: link.status()["blue"]["addr"] == "127.0.0.1", 2)
        assert link.status()["blue"]["error"] is None and link._by_addr["127.0.0.1"] == "blue"
        n = len(asked)
        link._st["blue"]["addr"] = None
        link.send({"blue": fcapp.Cmd.stop()})                   # within RESOLVE_RETRY_S: throttled
        time.sleep(0.1)
        assert len(asked) == n
    finally:
        link.close()


# ---- nit: Space pauses a match (zero speed for everyone), the score stays ---------------------------
def test_space_pauses_a_match_and_keeps_the_score(running):
    app = running
    assert app.action({"action": "play_here"})["ok"]
    with app.lock:
        app.flow._set("play", time.monotonic())
        app.flow.score = {"blue": 2, "red": 1}
    r = app.action({"action": "stop"})
    assert r == {"ok": True, "paused": True} and app.mode == "match" and app.flow.phase == "paused"
    time.sleep(0.15)                                            # a few ticks: every robot holds zero
    assert all(c.vx == 0 and c.vy == 0 and c.wz == 0 for c in app.cmds.values())
    assert app.score == {"blue": 2, "red": 1}
    assert app.action({"action": "stop"})["paused"] and app.flow.phase == "paused"   # again: still paused
    assert "End match" in app.busy_reason()
    assert app.action({"action": "end"})["ok"] and app.mode == "idle"
    assert app.last_match["score"] == {"blue": 2, "red": 1}
    assert app.action({"action": "go"})["ok"] and app.action({"action": "stop"}) == {"ok": True}   # non-match: ends
    assert app.mode == "idle"


# ---- eye calibration (self-healing eye, 0571987): recalibrate op, honest suspect state, run log ----------
class FakeEye:
    """The eye's calibration surface only: recalibrate / cal_status / calibrated_size / status."""
    paused = False

    def __init__(self, state="ok", why="", size=(3.20, 2.10), ok=True):
        self.cal = {"state": state, "why": why}
        self.size, self.ok, self.asked = size, ok, []

    def cal_status(self):
        return dict(self.cal)

    def calibrated_size(self):
        return self.size

    def recalibrate(self, timeout=8.0, wait=True):
        self.asked.append(timeout)
        if not self.ok:
            return {"ok": False, "message": "fit err 4.1 px > 3 px: calibration kept", "cal": self.cal_status()}
        self.cal = {"state": "ok", "why": "calibrated (operator)"}
        return {"ok": True, "message": "calibrated (operator): field 3.20 x 2.10 m", "cal": self.cal_status()}

    def note_cmds(self, now, cmds):
        pass

    def status(self):
        return {"fps": 30.0, "calibrated": True, "message": "ok", "paused": False, "picture_age": 0.0,
                "cal": self.cal_status()}


def test_eye_recalibrate_op_idle_only(idle):
    from fc import ops
    app = idle
    assert "eye.recalibrate" in {o["name"] for o in ops.manifest(app)["ops"]}
    assert ops.call(app, "eye.recalibrate")["code"] == "unavailable"            # sim: no camera
    app.eye = FakeEye()
    try:
        app.mode = "match"
        r = ops.call(app, "eye.recalibrate")
        assert r["code"] == "unavailable" and "idle" in r["error"] and not app.eye.asked
        app.mode = "idle"
        r = ops.call(app, "eye.recalibrate")
        assert r["ok"] and app.eye.asked == [8.0] and r["field"] == [3.2, 2.1] and r["field_source"] == "camera"
        assert (app.field.length, app.field.width) == (3.20, 2.10)
        assert any("calibrated (operator)" in e["msg"] for e in app.events)
        app.eye.ok = False
        r = ops.call(app, "eye.recalibrate")
        assert not r["ok"] and r["code"] == "refused" and "calibration kept" in r["error"]
    finally:
        app.eye = None


def test_suspect_calibration_banner_and_health_text(idle):
    app = idle
    app.eye = FakeEye("suspect", "3 tags shifted (px 9, 11, 12); last pose kept")
    try:
        ps = app.play_state(time.monotonic())
        b = [x for x in ps["banners"] if x["kind"] == "calibration"]
        assert len(b) == 1 and "UNVERIFIED" in b[0]["text"] and "3 tags shifted" in b[0]["text"]
        assert b[0]["text"].count("pose kept") == 1          # the eye's "; last pose kept" is not said twice
        assert ps["robots"]["blue"]["health"]["text"].endswith("cal unverified (suspect)")
        app.eye.cal = {"state": "unverified", "why": "loaded eye_cal.json"}
        ps = app.play_state(time.monotonic())
        assert not [x for x in ps["banners"] if x["kind"] == "calibration"]
        assert ps["robots"]["blue"]["health"]["text"].endswith("· cal unverified")
        app.eye.cal = {"state": "ok", "why": ""}
        ps = app.play_state(time.monotonic())
        assert "cal" not in ps["robots"]["blue"]["health"]["text"]
    finally:
        app.eye = None


def test_eye_log_record_carries_cal_state(running):
    app = running
    app.eye = FakeEye("suspect", "camera moved")
    app.params["record"] = False                             # no overhead video from a fake eye
    try:
        assert app.action({"action": "go"})["ok"]
        run_dir = app.test["run_dir"]
        time.sleep(1.2)                                      # > one 2 Hz health record
        app.stop()
        from pathlib import Path
        assert wait(lambda: (Path(run_dir) / "summary.json").exists(), 3)
        recs = [json.loads(x) for x in (Path(run_dir) / "events.jsonl").read_text().splitlines()]
        eyes = [r["eye"] for r in recs if "eye" in r]
        assert eyes and all(e["cal"] == "suspect" for e in eyes)
    finally:
        app.eye = None


def test_ack_tilt_held_high_zeroes_translation_then_releases(tmp_path):
    """Run 20261128T065952Z: the laptop kept ramping vx for 0.7 s after the ack read 39 deg. Tick integration: a held
    high ack tilt zeroes vx/vy (wz kept, brain ramp restarts from zero), and commands pass again once upright."""
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    tilt = {"v": 40.0}
    app.link_status = lambda: {"blue": {"imu": {"tilt_deg": tilt["v"], "level": "warn"}, "ack_age": 0.02, "connected": True}}
    try:
        with app.lock:
            assert app._start_test({"test": "drive", "robot": "blue"})["ok"]

        def run(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                app.teleop["blue"] = (1.0, 0.0, 0.5, time.monotonic())
                app.tick(1 / 30)
                time.sleep(1 / 60)
            return app.cmds["blue"]

        c = run(0.8)
        assert (c.vx, c.vy, c.tag) == (0.0, 0.0, "tilt") and c.wz > 0, c
        assert any("TILT on blue" in e["msg"] for e in app.events)
        tilt["v"] = 4.0
        c = run(0.8)
        assert c.vx > 0 and c.tag == "teleop", c
        assert any("tilt on blue back to 4 deg" in e["msg"] for e in app.events)
    finally:
        app.close()


def test_kickoff_refusal_names_the_halt_and_the_way_out(idle):
    """Rehearsal Nov 28: K with red e-stopped said "red halted by the bridge" (also for an operator e-stop) and not
    what to do. The reason now names the halt and the two ways out; the same text on K, P and W."""
    from fc import ops
    app = idle
    with app.lock:
        app.estop("red")
        why = ops.call(app, "match.kickoff")["error"]
        assert why == "red is e-stopped (Get up / Ready re-arms it), or V to play without it"
        assert ops.availability(app)["show.warmup"]["reason"] == why
        assert app._play_action("play_here", {})["error"] == why
        app.estop("blue")
        assert ops.availability(app)["match.kickoff"]["reason"].endswith("or V to play without them")
        app.estops.clear()
        app.imu_bad = {"red"}
        assert ops.availability(app)["match.kickoff"]["reason"].startswith("red has a bad IMU")
        app.imu_bad = set()
        assert ops.call(app, "match.kickoff")["ok"]
        app.action({"action": "end"})


class _AckLink:
    """Link stand-in for a sim App: only its status matters (sim never sends). connected toggles the link."""

    def __init__(self):
        self.up = True

    def status(self):
        return {rid: {"connected": self.up, "ack_age": 0.02 if self.up else 3.0, "imu": {"level": "ok"}, "mux_ok": True}
                for rid in ("blue", "red")}

    def drain_acks(self):
        return []

    def stop_all(self, burst=3):
        pass

    def close(self):
        pass


def test_link_drop_is_a_hard_stop_and_motion_restarts_from_zero(tmp_path):
    """Rehearsal Nov 28: the brain kept ramping while red had no link, so the first packet after the link came back
    jumped from standstill to 0.45 m/s + 1.17 rad/s. Now: zero while there is no link, zero LINK_BACK_HOLD_S after
    the return, then the normal ramp from zero."""
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    link = app.link = _AckLink()
    app.link_status = link.status
    try:
        with app.lock:
            assert app._start_test({"test": "push_ball", "robot": "blue"})["ok"]

        def run(seconds):
            out, end = [], time.monotonic() + seconds
            while time.monotonic() < end and app.mode == "push_ball":
                app.tick(1 / 30)
                out.append(app.cmds["blue"])
                time.sleep(1 / 60)
            return out

        moving = run(2.0)
        assert max(abs(c.vx) for c in moving) > 0.2, moving[-5:]       # the brain really drives it
        link.up = False
        lost = run(1.0)
        assert lost and all((c.vx, c.vy, c.wz) == (0, 0, 0) for c in lost), lost[-3:]
        assert any("NO LINK to blue" in e["msg"] for e in app.events)
        link.up = True
        back = run(1.2)
        held = [c for c in back if c.tag == "link_back"]
        assert held and all((c.vx, c.vy, c.wz) == (0, 0, 0) for c in held)
        after = back[len(held):]
        assert after and back[0].tag == "link_back"
        first = after[:3]   # ramp from zero: accel_vx 0.6 (keeper 1.5) and accel_wz 2.0 per s, ~30 Hz ticks
        assert all(abs(c.vx) <= 0.16 and abs(c.wz) <= 0.25 for c in first), first
        # healthy link: no gate at all
        assert app._link_gate("blue", time.monotonic()) is None and app._link_gate("red", time.monotonic()) is None
    finally:
        app.close()
