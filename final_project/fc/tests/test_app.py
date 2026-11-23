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
    assert c["vx"] == pytest.approx(app.params["max_vx"]) and c["wz"] == pytest.approx(-app.params["max_wz"])
    assert state(base)["cmds"]["red"]["vx"] == 0
    time.sleep(app.params["teleop_timeout_s"] + 0.2)             # no key updates -> zero
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
    for bad in ("5; reboot", 0, 31, 2.5, True, "5"):
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
