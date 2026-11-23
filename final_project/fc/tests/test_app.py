"""App endpoints in-process with the sim source, plus runlog and param validation."""
import json
import time
import urllib.error
import urllib.request

import pytest

from fc.app import App, serve, validate_params
from fc.runlog import RunLog, list_runs


def test_runlog_writes_and_never_blocks(tmp_path):
    run = RunLog("unit", {"a": 1}, {"robot": "blue"}, root=tmp_path)
    t0 = time.perf_counter()
    for i in range(2000):
        run.event({"w": {"x": 1.23456789, "i": i}})
    assert time.perf_counter() - t0 < 0.5          # queue only, no disk wait
    s = run.finish({"result": "ok"})
    lines = (run.dir / "events.jsonl").read_text().splitlines()
    assert len(lines) == 2000 and json.loads(lines[0])["w"]["x"] == 1.235
    assert json.loads((run.dir / "params.json").read_text())["params"] == {"a": 1}
    assert json.loads((run.dir / "summary.json").read_text())["result"] == "ok" == s["result"]
    assert list_runs(tmp_path)[0]["summary"]["test"] == "unit"


def test_validate_params():
    cur = {"max_vx": 0.3, "role_mode": "auto", "loop_hz": 30, "flag": True, "name": "x"}
    ok, err = validate_params(cur, {"max_vx": 0.4, "role_mode": "attack", "loop_hz": 20})
    assert ok == {"max_vx": 0.4, "role_mode": "attack", "loop_hz": 20} and not err
    ok, err = validate_params(cur, {"max_vx": 0.9, "role_mode": "fly", "nope": 1, "flag": 1, "name": 3, "loop_hz": "x"})
    assert ok == {} and len(err) == 6


@pytest.fixture
def server(tmp_path):
    app = App("sim", dry_run_ops=True, runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    app.start()
    srv = serve(app, "127.0.0.1", 0)
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    assert wait(lambda: app.world is not None and len(app.world.robots) == 2, 3)  # sim latency
    yield app, base
    app.close()
    srv.shutdown()


def get(base, path):
    with urllib.request.urlopen(base + path, timeout=5) as r:
        return r.status, r.read()


def post(base, path, body):
    req = urllib.request.Request(base + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def state(base):
    return json.loads(get(base, "/state")[1])


def wait(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_state_ui_and_params(server):
    app, base = server
    code, html = get(base, "/")
    assert code == 200 and b"Pupper FC" in html
    s = state(base)
    assert s["mode"] == "idle" and s["source"] == "sim" and set(s["active"]) == {"blue", "red"}
    assert s["world"]["robots"] and "max_vx" in s["params"] and s["truth"]["robots"]
    assert s["loop"]["hz"] > 10
    code, r = post(base, "/params", {"max_vx": 0.25})
    assert code == 200 and app.params["max_vx"] == 0.25
    code, r = post(base, "/params", {"max_vx": 0.8})                  # above the hard ceiling
    assert code == 400 and app.params["max_vx"] == 0.25
    code, r = post(base, "/preset", {"op": "save", "name": "slow"})
    assert code == 200
    post(base, "/params", {"max_vx": 0.2})
    code, r = post(base, "/preset", {"op": "load", "name": "slow"})
    assert code == 200 and app.params["max_vx"] == 0.25 and "slow" in state(base)["presets"]
    assert post(base, "/preset", {"op": "save", "name": "../x"})[0] == 400


def test_go_stop_and_push_ball_run(server):
    app, base = server
    assert post(base, "/action", {"action": "go"})[0] == 200
    assert wait(lambda: any(abs(c["vx"]) + abs(c["wz"]) > 0 for c in state(base)["cmds"].values()), 3)
    post(base, "/action", {"action": "stop"})
    s = state(base)
    assert s["mode"] == "idle" and s["last_summary"]["result"] == "stopped"
    assert wait(lambda: all(c["vx"] == c["vy"] == c["wz"] == 0 for c in state(base)["cmds"].values()), 1)
    code, r = post(base, "/action", {"action": "test", "test": "push_ball", "robot": "blue"})
    assert code == 200
    run_dir = r["run"]
    time.sleep(1.0)
    assert state(base)["mode"] == "push_ball"
    post(base, "/action", {"action": "stop"})
    assert wait(lambda: (app.runs_root / run_dir.split("/")[-1] / "summary.json").exists(), 6)
    runs = json.loads(get(base, "/runs")[1])
    assert {r["summary"]["test"] for r in runs if r["summary"]} >= {"play", "push_ball"}
    events = (app.runs_root / run_dir.split("/")[-1] / "events.jsonl").read_text().splitlines()
    assert len(events) > 15 and "w" in json.loads(events[1])


def test_goto_reports_error(server):
    app, base = server
    s = state(base)
    p = s["world"]["robots"]["red"]
    target = [p["x"] - 0.4, p["y"] + 0.2]
    post(base, "/params", {"test_timeout_s": 25})
    assert post(base, "/action", {"action": "test", "test": "goto", "robot": "red", "target": target})[0] == 200
    assert wait(lambda: state(base)["mode"] == "idle", 30)
    ls = state(base)["last_summary"]
    assert ls["result"] in ("reached", "timeout") and ls["pos_err_m"] is not None
    assert ls["result"] == "reached", ls


def test_drive_teleop_and_deadman(server):
    app, base = server
    assert post(base, "/action", {"action": "teleop", "robot": "blue", "vx": 1})[0] == 400  # not in drive
    post(base, "/action", {"action": "test", "test": "drive", "robot": "blue"})
    post(base, "/action", {"action": "teleop", "robot": "blue", "vx": 1, "wz": -1})
    time.sleep(0.1)
    c = state(base)["cmds"]["blue"]
    assert 0 < c["vx"] < app.params["max_vx"]                    # rate-limited like the brain (accel_vx)
    keep = lambda: post(base, "/action", {"action": "teleop", "robot": "blue", "vx": 1, "wz": -1})  # noqa: E731
    assert wait(lambda: keep() and state(base)["cmds"]["blue"]["vx"] == pytest.approx(app.params["max_vx"]), 2)
    c = state(base)["cmds"]["blue"]
    assert c["wz"] == pytest.approx(-app.params["max_wz"])
    assert state(base)["cmds"]["red"]["vx"] == 0
    time.sleep(app.params["teleop_timeout_s"] + 0.2)             # no key updates -> zero at once
    assert state(base)["cmds"]["blue"]["vx"] == 0
    post(base, "/action", {"action": "stop"})


def test_robot_ops_dry_run(server):
    app, base = server
    code, r = post(base, "/robot", {"robot": "blue", "op": "start"})
    assert code == 400 and "confirm" in r["error"]                 # homing needs confirmation
    code, r = post(base, "/robot", {"robot": "blue", "op": "start", "confirm": True})
    assert code == 200 and "fcctl.sh start stock --confirm" in r["cmd"] and r["cmd"].startswith("echo DRY-RUN")
    assert wait(lambda: not state(base)["ops"]["blue"]["running"], 3)
    assert "DRY-RUN" in state(base)["ops"]["blue"]["out"]
    assert post(base, "/robot", {"robot": "red", "op": "status"})[0] == 400   # no host
    code, r = post(base, "/robot", {"robot": "blue", "op": "stage"})
    assert code == 200 and "stage.sh mts" in r["cmd"]


def test_bad_requests(server):
    app, base = server
    assert post(base, "/action", {"action": "fly"})[0] == 400
    assert post(base, "/action", {"action": "test", "test": "nope"})[0] == 400
    assert post(base, "/action", {"action": "test", "test": "goto", "robot": "blue"})[0] == 400
    assert post(base, "/nope", {})[0] == 404
    assert get(base, "/state")[0] == 200


# ---- security hardening ------------------------------------------------------------------
def raw_post(base, path, body: bytes, headers: dict):
    req = urllib.request.Request(base + path, body, headers)
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status
    except urllib.error.HTTPError as e:
        return e.code


def test_post_needs_json_content_type(server):
    app, base = server
    before = app.params["max_vx"]
    body = json.dumps({"max_vx": 0.21}).encode()
    assert raw_post(base, "/params", body, {}) == 403                                  # no content type
    assert raw_post(base, "/params", body, {"Content-Type": "text/plain"}) == 403      # CSRF-able "simple" type
    assert app.params["max_vx"] == before
    assert raw_post(base, "/params", body, {"Content-Type": "application/json; charset=utf-8"}) == 200


def test_post_rejects_foreign_origin(server):
    app, base = server
    port = base.rsplit(":", 1)[1]
    body = json.dumps({"action": "go"}).encode()
    ctype = {"Content-Type": "application/json"}
    for bad in ("http://evil.example", f"http://evil.example:{port}", "null", f"http://127.0.0.1:{int(port) + 1}"):
        assert raw_post(base, "/action", body, {**ctype, "Origin": bad}) == 403, bad
    assert app.mode == "idle"
    for good in (f"http://127.0.0.1:{port}", f"http://localhost:{port}"):
        assert raw_post(base, "/action", body, {**ctype, "Origin": good}) == 200
    post(base, "/action", {"action": "stop"})


@pytest.fixture
def live_ops(tmp_path, monkeypatch):
    """App with real (not dry-run) robot ops and subprocess.run recorded instead of executed."""
    import fc.app as fcapp
    calls = []

    def fake_run(cmd, **kw):
        calls.append(cmd)
        return fcapp.subprocess.CompletedProcess(cmd, 0, "{}\n", "")

    monkeypatch.setattr(fcapp.subprocess, "run", fake_run)
    monkeypatch.setattr(fcapp.subprocess, "Popen", lambda *a, **k: calls.append(a) or pytest.fail("Popen"))
    app = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    yield app, calls
    app.close()


def test_bad_robot_op_args_never_spawn(live_ops):
    app, calls = live_ops
    for bad in ("stock; reboot", "$(reboot)", "../stock", "nope", "stock ", 7):
        r = app.robot_op({"robot": "blue", "op": "start", "confirm": True, "profile": bad})
        assert not r["ok"], bad
    for bad in ("5; reboot", 0, 901, 2.5, True, "5"):
        assert not app.robot_op({"robot": "blue", "op": "imu_test", "seconds": bad})["ok"], bad
    assert not app.robot_op({"robot": "blue", "op": "rm -rf"})["ok"]
    assert not app.robot_op({"robot": "blue", "op": "status", "host": "evil"})["cmd"].count("evil")
    assert wait(lambda: calls, 2)
    assert len(calls) == 1 and "evil" not in " ".join(calls[0])                         # host only from robots.json


def test_robot_op_remote_args_are_quoted(live_ops):
    app, calls = live_ops
    from dataclasses import replace
    blue = app.cfg["blue"]
    for bad_host in ("pi; reboot", "-oProxyCommand=x"):                                 # even a bad config is refused
        app.cfg["blue"] = replace(blue, host=bad_host)
        assert not app.robot_op({"robot": "blue", "op": "status"})["ok"]
    app.cfg["blue"] = blue
    assert app.robot_op({"robot": "blue", "op": "imu_test", "seconds": 7})["ok"]
    assert wait(lambda: calls, 2)
    assert calls[0][-3:] == ["--", "mts", "bash ~/fc-v1/robot/fcctl.sh imu-test 7"]
    assert not calls[1:]


# ---- 2026-11-23 review fixes ------------------------------------------------------------
import math  # noqa: E402
from dataclasses import replace  # noqa: E402

import fc.app as fcapp  # noqa: E402
from fc.app import analyze_deadman, analyze_step, load_robot_config  # noqa: E402
from fc.runlog import Metrics  # noqa: E402


def test_no_stub_fallback():
    """A broken brain/sim import must stop the app, never fall back to a stand-in."""
    assert not hasattr(fcapp, "_StubBrain") and not hasattr(fcapp, "_StubSim") and not hasattr(fcapp, "_go_to")
    assert fcapp.shape.__module__ == "fc.brain"


def test_stop_never_refused_and_start_timeout(live_ops, monkeypatch):
    app, calls = live_ops
    seen = []
    monkeypatch.setattr(app, "_run_op", lambda rid, state, cmd, timeout: seen.append((state["op"], timeout)))
    assert app.robot_op({"robot": "blue", "op": "start", "confirm": True})["ok"]
    assert app.ops["blue"]["running"]                                        # start still running (fake never ends)
    assert not app.robot_op({"robot": "blue", "op": "status"})["ok"]          # others wait
    assert app.robot_op({"robot": "blue", "op": "stop"})["ok"]                # stop always goes
    assert app.robot_op({"robot": "blue", "op": "stop"})["ok"]                # even over another stop
    assert seen[0] == ("start", 180) and seen[1][0] == "stop"


def test_imu_test_options_quoted(live_ops):
    app, calls = live_ops
    assert app.robot_op({"robot": "blue", "op": "imu_test", "seconds": 600, "mode": "bno", "condition": "unpowered"})["ok"]
    assert wait(lambda: calls, 2)
    assert calls[0][-1] == "bash ~/fc-v1/robot/fcctl.sh imu-test 600 --mode bno --condition unpowered"
    assert not app.robot_op({"robot": "blue", "op": "imu_test", "seconds": 5, "mode": "x; reboot"})["ok"]
    assert not app.robot_op({"robot": "blue", "op": "imu_test", "seconds": 5, "condition": "a b"})["ok"]


class FakeLink:
    """Stand-in Link for a sim-source App: only status/acks matter (sim mode never sends)."""

    def __init__(self):
        self.st = {"blue": {"imu": {"level": "ok"}, "sent": 0, "acks": 0, "profile": "stock", "imu_hold": False,
                            "hold": None, "mux_ok": True, "auth": "ok"}}
        self.stops = 0

    def status(self):
        return {k: dict(v) for k, v in self.st.items()}

    def drain_acks(self):
        return [{"r": "blue", "ack": 1, "rtt": 9.0}]

    def stop_all(self):
        self.stops += 1

    def send(self, cmds):
        pass

    def close(self):
        pass


def test_imu_bad_stops_robot_and_ends_test(server):
    app, base = server
    fake = app.link = FakeLink()
    code, r = post(base, "/action", {"action": "test", "test": "push_ball", "robot": "blue"})
    assert code == 200
    run = app.runs_root / r["run"].split("/")[-1]
    time.sleep(0.8)
    fake.st["blue"]["imu"] = {"level": "bad", "reasons": ["tilt 12.0 deg at zero command"]}
    assert wait(lambda: state(base)["mode"] == "idle", 2)
    s = state(base)
    assert s["last_summary"]["result"] == "imu_bad" and s["imu_bad"] == ["blue"] and fake.stops >= 1
    assert s["cmds"]["blue"]["vx"] == 0 and s["cmds"]["blue"]["tag"] in ("imu_bad", "idle")
    assert post(base, "/action", {"action": "test", "test": "drive", "robot": "blue"})[0] == 400   # refused while bad
    assert wait(lambda: (run / "summary.json").exists(), 6)
    lines = [json.loads(x) for x in (run / "events.jsonl").read_text().splitlines()]
    assert any("IMU BAD" in str(x.get("ev", "")) for x in lines)
    lk = [x["lk"]["blue"] for x in lines if "lk" in x]
    assert lk and lk[0]["imu_hold"] is False and lk[0]["mux_ok"] is True and lk[0]["profile"] == "stock"
    summ = json.loads((run / "summary.json").read_text())
    assert summ["result"] == "imu_bad" and summ["metrics"]["robots"]["blue"]["imu_worst"] == "bad"
    fake.st["blue"]["imu"] = {"level": "ok"}
    assert wait(lambda: state(base)["imu_bad"] == [], 1)


def test_mux_lost_stops_robot_and_ends_test(server):
    app, base = server
    fake = app.link = FakeLink()
    assert post(base, "/action", {"action": "go"})[0] == 200
    time.sleep(0.3)
    fake.st["blue"].update(mux_ok=False, hold="no_cmd_vel_publisher")
    assert wait(lambda: state(base)["mode"] == "idle", 2)
    s = state(base)
    assert s["last_summary"]["result"] == "mux_lost" and s["mux_lost"] == ["blue"]
    assert s["last_summary"]["hold"] == {"blue": "no_cmd_vel_publisher"} and s["cmds"]["blue"]["vx"] == 0
    assert post(base, "/action", {"action": "test", "test": "step", "robot": "blue", "vx": 0.1})[0] == 400
    assert post(base, "/action", {"action": "test", "test": "step", "robot": "red", "vx": 0.1})[0] == 200
    post(base, "/action", {"action": "stop"})
    fake.st["blue"].update(mux_ok=True, hold=None)
    assert wait(lambda: state(base)["mux_lost"] == [], 1)


def test_step_pipeline_in_sim(server):
    app, base = server
    code, r = post(base, "/action", {"action": "test", "test": "step", "robot": "blue", "vx": 0.2, "seconds": 1.5})
    assert code == 200
    assert wait(lambda: state(base)["mode"] == "idle", 8)
    ls = state(base)["last_summary"]
    assert ls["result"] == "done", ls
    assert ls["cmd"] == [0.2, 0.0, 0.0] and 0.15 < ls["cmd_dist_m"] < 0.35
    assert 0.5 < ls["gain_lin"] < 1.5 and ls["latency_s"] is not None and abs(ls["heading_bias_deg"]) < 20
    m = ls["metrics"]
    assert m["time_s"] > 3 and m["robots"]["blue"]["path_m"] > 0.1 and "fast_reversals" in m["robots"]["blue"]
    assert post(base, "/action", {"action": "test", "test": "step", "robot": "blue"})[0] == 400        # all zero
    assert post(base, "/action", {"action": "test", "test": "step", "robot": "blue", "vx": "x"})[0] == 400


def test_deadman_pipeline_in_sim(server):
    app, base = server
    code, _ = post(base, "/action", {"action": "test", "test": "deadman", "robot": "blue", "vx": 0.15, "mute_s": 1.5})
    assert code == 200
    assert wait(lambda: state(base)["test"] and state(base)["test"].get("phase") == "mute", 4)
    assert wait(lambda: state(base)["mode"] == "idle", 4)
    ls = state(base)["last_summary"]
    assert ls["result"] == "stopped" and ls["emulated"], ls
    assert 0.1 <= ls["stop_time_s"] <= 1.0 and ls["speed_before"] > 0.05


def test_goto_uses_brain_goto_mode(server):
    app, base = server
    p = state(base)["world"]["robots"]["blue"]
    post(base, "/action", {"action": "test", "test": "goto", "robot": "blue", "target": [p["x"] + 0.3, p["y"]]})
    assert wait(lambda: (state(base)["debug"].get("blue") or {}).get("role") == "goto", 2)
    post(base, "/action", {"action": "stop"})


def test_presets_load_on_base_and_are_recorded(server, tmp_path):
    app, base = server
    app.presets_dir.mkdir(parents=True, exist_ok=True)
    (app.presets_dir / "slowx.json").write_text(json.dumps({"_note": "n", "max_vx": 0.15}))
    (app.presets_dir / "turny.json").write_text(json.dumps({"_profile": "stock", "max_wz": 0.5}))
    assert post(base, "/preset", {"op": "load", "name": "slowx"})[0] == 200
    assert post(base, "/preset", {"op": "load", "name": "turny"})[0] == 200
    s = state(base)
    assert s["params"]["max_wz"] == 0.5 and s["params"]["max_vx"] == app.base_params["max_vx"]   # no stacking
    assert s["preset"] == {"name": "turny", "modified": [], "profile": "stock", "note": ""}
    post(base, "/params", {"max_vy": 0.1})
    assert state(base)["preset"]["modified"] == ["max_vy"]
    code, r = post(base, "/action", {"action": "test", "test": "push_ball", "robot": "blue"})
    meta = json.loads((app.runs_root / r["run"].split("/")[-1] / "params.json").read_text())
    assert meta["preset"] == "turny" and meta["preset_modified"] == ["max_vy"] and meta["preset_profile"] == "stock"
    post(base, "/action", {"action": "stop"})
    post(base, "/preset", {"op": "save", "name": "mine"})
    saved = json.loads((app.presets_dir / "mine.json").read_text())
    assert saved == {"_profile": "stock", "max_vy": 0.1, "max_wz": 0.5}                         # only changed keys
    assert post(base, "/preset", {"op": "base"})[0] == 200
    assert state(base)["preset"]["name"] is None and app.params == app.base_params


def test_shipped_presets_validate():
    from fc.brain import DEFAULTS
    from fc.core import load_params
    base = {**fcapp.PARAM_FALLBACKS, **DEFAULTS, **load_params()}
    names = sorted(p.stem for p in fcapp.PRESETS.glob("*.json"))
    assert {"slow", "fast", "stock-hh-on", "stock-hh-off", "u6-hh-on", "push", "headbutt"} <= set(names)
    for n in names:
        data = json.loads((fcapp.PRESETS / f"{n}.json").read_text())
        ok, err = fcapp.validate_params(base, {k: v for k, v in data.items() if not k.startswith("_")})
        assert not err, (n, err)


def test_robot_config_allows_udp_host(tmp_path):
    p = tmp_path / "robots.json"
    p.write_text(json.dumps([{"id": "blue", "team": "blue", "tag": 10, "host": "mts", "profile": "stock",
                              "udp_host": "192.168.1.42"}]))
    robots, udp = load_robot_config(p)
    assert robots[0].host == "mts" and udp == {"blue": "192.168.1.42"}


# ---- pure analysis ----------------------------------------------------------------------
def _walk(gain=0.8, lag=0.25, v=0.2, t_on=1.0, t_off=3.0, yaw=0.3, walk_dir=0.3, fps=30, t_end=4.5):
    """Synthetic step: commands at 30 Hz from t=0; the robot follows gain*cmd delayed by lag,
    walking along walk_dir while its tag reads yaw."""
    sent = [(i / 30, v if t_on <= i / 30 < t_off else 0.0, 0.0, 0.0) for i in range(int(t_end * 30))]
    obs, s = [], 0.0
    for i in range(int(t_end * fps)):
        t = i / fps
        s = gain * v * max(0.0, min(t - lag, t_off) - t_on) if t - lag > t_on else 0.0
        obs.append((t, 1.0 + s * math.cos(walk_dir), 0.5 + s * math.sin(walk_dir), yaw))
    return obs, sent


def test_analyze_step_recovers_gain_lag_and_bias():
    obs, sent = _walk(walk_dir=0.3 + math.radians(7))
    r = analyze_step(obs, [c for c in sent if c[0] >= 1.0], 1.0, 3.0, [0.2, 0.0, 0.0])
    assert r["result"] == "done"
    assert r["gain_lin"] == pytest.approx(0.8, abs=0.03) and r["latency_s"] == pytest.approx(0.25, abs=0.04)
    assert r["heading_bias_deg"] == pytest.approx(7.0, abs=0.5) and r["pass"]["bias"] is False
    assert r["onset_s"] == pytest.approx(0.35, abs=0.1)
    still, sent2 = _walk(gain=0.0)
    assert analyze_step(still, [c for c in sent2 if c[0] >= 1.0], 1.0, 3.0, [0.2, 0, 0])["result"] == "no_motion"


def test_analyze_step_turn():
    sent = [(i / 30, 0.0, 0.0, 0.5 if 1.0 <= i / 30 < 3.0 else 0.0) for i in range(135)]
    obs = []
    for i in range(135):
        t = i / 30
        a = 0.9 * 0.5 * max(0.0, min(t - 0.3, 3.0) - 1.0) if t - 0.3 > 1.0 else 0.0
        obs.append((t, 0.0, 0.0, math.atan2(math.sin(3.0 + a), math.cos(3.0 + a))))   # wraps through pi
    r = analyze_step(obs, [c for c in sent if c[0] >= 1.0], 1.0, 3.0, [0, 0, 0.5])
    assert r["gain_wz"] == pytest.approx(0.9, abs=0.03) and r["latency_s"] == pytest.approx(0.3, abs=0.04)


def test_analyze_deadman():
    obs = []
    for i in range(120):                      # 0.1 m/s until 2.3 s, then rest (mute at 2.0 s)
        t = i / 30
        obs.append((t, 0.1 * min(t, 2.3), 0.0, 0.0))
    r = analyze_deadman(obs, 2.0)
    assert r["result"] == "stopped" and r["stop_time_s"] == pytest.approx(0.3, abs=0.12) and r["pass"]
    assert r["coast_m"] == pytest.approx(0.03, abs=0.005) and r["speed_before"] == pytest.approx(0.1, abs=0.005)
    import random
    rnd = random.Random(1)
    noisy = [(t, x + rnd.gauss(0, 0.01), y + rnd.gauss(0, 0.01), 0.0) for t, x, y, _ in obs]   # sim-like 1 cm noise
    assert analyze_deadman(noisy, 2.0)["stop_time_s"] == pytest.approx(0.3, abs=0.12)
    moving = [(i / 30, 0.1 * i / 30, 0.0, 0.0) for i in range(120)]
    assert analyze_deadman(moving, 2.0)["result"] == "still_moving"
    assert analyze_deadman([(i / 30, 0.0, 0.0, 0.0) for i in range(120)], 2.0)["result"] == "never_moved"


def test_metrics_counts():
    from fc.core import Pose, RobotObs, World
    m = Metrics(["blue"], 0.0, 0, {"blue": {"sent": 10, "acks": 10}})
    for i in range(60):
        t = i / 30
        w = World(t, None, {"blue": RobotObs("blue", Pose(0.01 * i, 0, 0), t)})
        wz = 0.5 if (i // 10) % 2 == 0 else -0.5                      # reversal every 1/3 s
        m.tick(t, w, {"blue": (0.0 if i > 50 else 0.2, 0, wz if i < 40 else 1.0, "stale" if 20 <= i < 25 else "x")}, 0.5,
               {"blue": {"imu": {"level": "warn" if i == 7 else "ok"}}}, [{"r": "blue", "rtt": float(i)}])
    s = m.summary(2.0, 3, {"blue": {"sent": 70, "acks": 67}})["robots"]["blue"]
    assert s["path_m"] == pytest.approx(0.59, abs=1e-6) and s["stale_stops"] == 1 and s["wz_reversals"] == 4
    assert s["fast_reversals"] == 3 and s["imu_worst"] == "warn" and s["loss"] == pytest.approx(0.05)
    assert s["rtt_p95_ms"] == 57.0 and s["spin_frac"] == pytest.approx(9 / 60, abs=1e-3)


def test_eye_reads_mount_knobs_live():
    """eye_* edits apply on the next frame: no restart. Needs OpenCV (.venv-fc); skipped otherwise."""
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    from fc.core import load_field
    from fc.eye import Eye
    robots, _ = load_robot_config()
    params = {**fcapp.PARAM_FALLBACKS}
    eye = Eye(load_field(), robots, params, source="recorded.mov")   # not started: no camera opened
    c = np.array([[0.1, 0.6], [0.1, 0.4], [-0.1, 0.4], [-0.1, 0.6]])  # tag top edge points +x
    x, y, yaw = eye.tag_pose(c, eye.mount())
    assert yaw == pytest.approx(0.0) and x == pytest.approx(-0.065) and y == pytest.approx(0.5)
    params["eye_yaw_offset_deg"] = -180.0
    params["eye_tag_offset_x"] = 0.0
    x, y, yaw = eye.tag_pose(c, eye.mount())
    assert abs(yaw) == pytest.approx(math.pi) and x == pytest.approx(0.0)

    class P:
        reasons = []

        def invalidate_calibration(self, why):
            self.reasons.append(why)

    eye.perc = P()
    eye._apply_floor_size(eye.mount()["floor_size"])
    assert not eye.perc.reasons
    params["eye_floor_size"] = 0.17
    eye._apply_floor_size(eye.mount()["floor_size"])
    assert eye.cfg["tags"]["floor_size"] == 0.17 and eye.perc.reasons == ["eye_floor_size changed"]


def test_loop_error_ends_test(server, monkeypatch):
    app, base = server
    post(base, "/action", {"action": "go"})
    monkeypatch.setattr(app, "_check_test", lambda w, n: 1 / 0)
    assert wait(lambda: state(base)["mode"] == "idle", 2)
    assert state(base)["last_summary"]["result"] == "loop_error"


def test_step_edge_abort_reports(server):
    app, base = server
    app.field = replace(app.field, length=0.2, width=0.2)      # every pose is "outside" -> edge abort
    post(base, "/action", {"action": "test", "test": "step", "robot": "blue", "vx": 0.2, "seconds": 1})
    assert wait(lambda: state(base)["mode"] == "idle", 4)
    assert state(base)["last_summary"]["result"] == "edge"


# ---- Play screen, match flow, ready/down ops, feeds (2026-11-23) --------------------------
def test_play_and_ops_pages(server):
    app, base = server
    code, html = get(base, "/")
    assert code == 200 and b"Pupper FC" in html and b"kickoff" in html.lower()
    code, html = get(base, "/ops")
    assert code == 200 and b"Pupper FC v1" in html
    p = state(base)["play"]
    assert p["flow"] is None and p["playing"] == ["blue", "red"] and p["variants"][0] == ["blue", "red"]
    assert p["robots"]["blue"]["health"]["level"] in ("ok", "warn") and p["robots"]["blue"]["cam"] is None
    with pytest.raises(urllib.error.HTTPError) as e:
        get(base, "/view.jpg")                                   # sim: no camera frame
    assert e.value.code == 404


def test_flow_match_kickoff_pause_touch_variant(server):
    app, base = server
    post(base, "/params", {"countdown_s": 0.3, "ball_settle_s": 0.3})
    assert post(base, "/action", {"action": "pause"})[0] == 400                 # no match yet
    code, r = post(base, "/action", {"action": "kickoff"})
    assert code == 200 and state(base)["play"]["flow"]["phase"] == "kickoff"
    run = app.runs_root / r["run"].split("/")[-1]
    assert wait(lambda: (state(base)["debug"].get("blue") or {}).get("role") == "goto", 2)
    assert wait(lambda: state(base)["play"]["flow"]["phase"] == "play", 25), state(base)["play"]["flow"]
    assert wait(lambda: any(abs(c["vx"]) + abs(c["wz"]) > 0 for c in state(base)["cmds"].values()), 3)
    assert post(base, "/action", {"action": "pause"})[0] == 200
    assert state(base)["play"]["flow"]["phase"] == "paused"
    assert wait(lambda: all(c["vx"] == c["wz"] == 0 for c in state(base)["cmds"].values()), 0.5)
    assert post(base, "/action", {"action": "touch"})[1]["touches"] == 1
    assert post(base, "/action", {"action": "touch", "delta": 5})[0] == 400
    assert post(base, "/action", {"action": "job", "robot": "red", "job": "defend"})[0] == 200
    assert post(base, "/action", {"action": "job", "robot": "red", "job": "fly"})[0] == 400
    assert post(base, "/action", {"action": "pause"})[1]["phase"] == "countdown"   # pause toggles back
    assert state(base)["play"]["ticker"]                                           # decisions narrated
    code, r = post(base, "/action", {"action": "variant"})
    assert code == 200 and r["playing"] == ["blue"]
    s = state(base)
    assert s["play"]["flow"]["playing"] == ["blue"] and s["play"]["flow"]["roles"] == {"blue": "attack"}
    assert wait(lambda: state(base)["cmds"]["red"]["tag"] == "benched", 1)
    assert post(base, "/action", {"action": "variant", "playing": ["nope"]})[0] == 400
    assert post(base, "/action", {"action": "end"})[0] == 200
    s = state(base)
    assert s["play"]["flow"] is None and s["play"]["last_match"]["result"] == "ended"
    assert s["play"]["last_match"]["touches"] == 1 and s["mode"] == "idle"
    assert post(base, "/action", {"action": "variant", "playing": ["blue", "red"]})[1]["playing"] == ["blue", "red"]
    assert wait(lambda: (run / "summary.json").exists(), 6)
    lines = [json.loads(x) for x in (run / "events.jsonl").read_text().splitlines()]
    assert any(x.get("m", {}).get("phase") == "play" for x in lines) and any("tk" in x for x in lines)


def test_ready_and_down_ops(server):
    app, base = server
    code, r = post(base, "/robot", {"robot": "blue", "op": "ready"})
    assert code == 400 and "confirm" in r["error"]
    code, r = post(base, "/robot", {"robot": "blue", "op": "ready", "confirm": True})
    assert code == 200 and "fcctl.sh ready stock" in r["cmd"] and r["cmd"].startswith("echo DRY-RUN")
    assert wait(lambda: not state(base)["ops"]["blue"]["running"], 3)
    code, r = post(base, "/robot", {"robot": "blue", "op": "down"})
    assert code == 200 and r["cmd"].endswith("fcctl.sh down")


def test_ready_streams_progress_and_down_never_refused(live_ops, monkeypatch):
    import threading
    app, calls = live_ops
    lines = ['{"phase":"starting","elapsed":1.0}\n', "plain log line\n", '{"phase":"homed","elapsed":40.2}\n',
             '{"phase":"ready","elapsed":71.0,"ok":true}\n']
    gate = threading.Event()

    class FakeProc:
        def __init__(self, cmd, **kw):
            calls.append(cmd)
            self.stdout = self._lines()

        def _lines(self):
            for i, x in enumerate(lines):
                if i == 2:
                    gate.wait(2)
                yield x

        def wait(self):
            return 0

        def kill(self):
            pass

    monkeypatch.setattr(fcapp.subprocess, "Popen", FakeProc)
    for bad in ("stock; reboot", "nope"):
        assert not app.robot_op({"robot": "blue", "op": "ready", "confirm": True, "profile": bad})["ok"]
    assert app.robot_op({"robot": "blue", "op": "ready", "confirm": True})["ok"]
    assert wait(lambda: app.ops["blue"].get("progress", {}).get("phase") == "starting", 2)
    assert app.play_state(time.monotonic())["robots"]["blue"]["health"]["text"].startswith("getting ready")
    assert not app.robot_op({"robot": "blue", "op": "status"})["ok"]           # ready still running
    ready_state = app.ops["blue"]
    assert app.robot_op({"robot": "blue", "op": "down"})["ok"]                 # down always goes
    gate.set()
    assert wait(lambda: app.robot_phase.get("blue") == "ready" and not ready_state["running"], 2)
    assert ready_state["rc"] == 0 and ready_state["progress"]["ok"] is True
    assert calls[0][-3:] == ["--", "mts", "bash ~/fc-v1/robot/fcctl.sh ready stock"]
    assert wait(lambda: len(calls) > 1, 2) and calls[1][-3:] == ["--", "mts", "bash ~/fc-v1/robot/fcctl.sh down"]


def test_cam_url_config_and_feed_endpoint(server, tmp_path):
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    from fc.media import CamFeed
    app, base = server
    p = tmp_path / "robots.json"
    p.write_text(json.dumps([{"id": "blue", "team": "blue", "tag": 10, "host": "mts", "profile": "stock",
                              "cam_url": "http://10.0.0.5:8090/stream.mjpg"}]))
    robots, _ = load_robot_config(p)
    assert robots[0].cam_url == "http://10.0.0.5:8090/stream.mjpg"
    jpg = b"\xff\xd8" + b"fake-jpeg-body" + b"\xff\xd9"

    class Mjpeg(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):  # noqa: N802
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                for _ in range(60):
                    self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                    self.wfile.flush()
                    time.sleep(0.05)
            except OSError:
                pass

    cam = ThreadingHTTPServer(("127.0.0.1", 0), Mjpeg)
    cam.daemon_threads = True
    threading.Thread(target=cam.serve_forever, daemon=True).start()
    app.cams["blue"] = CamFeed(f"http://127.0.0.1:{cam.server_address[1]}/stream.mjpg")
    try:
        assert wait(lambda: state(base)["play"]["robots"]["blue"]["cam"]["ok"], 3)
        assert get(base, "/cam/blue.jpg") == (200, jpg)
    finally:
        app.cams["blue"].close()
        cam.shutdown()
    dead = CamFeed("http://127.0.0.1:9/none.mjpg", retry_s=0.1)
    assert wait(lambda: dead.error not in (None, "connecting"), 3) and not dead.status()["ok"]
    dead.close()
