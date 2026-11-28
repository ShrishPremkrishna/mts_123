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
def test_ready_rearm_mid_match_pauses_the_flow(tmp_path, monkeypatch):
    monkeypatch.setattr(fcapp.subprocess, "run",
                        lambda cmd, **kw: fcapp.subprocess.CompletedProcess(cmd, 0, "{}\n", ""))
    gate = threading.Event()

    class ReadyProc:
        def __init__(self, cmd, **kw):
            self.stdout = self._lines()

        def _lines(self):
            gate.wait(2)
            yield '{"phase":"ready","elapsed":9.0,"ok":true,"done":true}\n'

        def wait(self):
            return 0

        def kill(self):
            pass

    monkeypatch.setattr(fcapp.subprocess, "Popen", ReadyProc)
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    try:
        with app.lock:
            assert app._play_action("play_here", {})["ok"]
            assert app.flow.force_start(time.monotonic()) or app.flow.phase == "countdown"
            app.flow._set("play", time.monotonic())
        app.estop("blue")
        assert app.flow.phase == "play"                       # the e-stop alone does not pause (the other plays on)
        assert app.robot_op({"robot": "blue", "op": "ready", "confirm": True})["ok"]
        gate.set()
        assert wait(lambda: "blue" not in app.estops, 2)
        assert app.flow.phase == "paused" and app.flow.paused_from == "play"
        assert any("re-armed mid-match" in e["msg"] for e in app.events)
    finally:
        app.close()


def test_sim_estop_clear_mid_match_pauses_the_flow(idle):
    app = idle
    with app.lock:
        assert app._play_action("play_here", {})["ok"]
        app.flow._set("play", time.monotonic())
        app.estop("red")
        assert app._estop_clear_sim("red")["cleared"] == ["red"]
        assert app.flow.phase == "paused"


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
