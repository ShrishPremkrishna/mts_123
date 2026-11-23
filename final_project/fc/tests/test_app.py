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
