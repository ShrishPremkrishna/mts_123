"""The operation registry: manifest shape, validation, availability, and /api/op over HTTP."""
import json
import time
import urllib.error
import urllib.request

import pytest

from fc import ops
from fc.app import App, serve


@pytest.fixture
def app(tmp_path):
    a = App("sim", dry_run_ops=True, runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    yield a
    a.close()


@pytest.fixture
def server(tmp_path):
    a = App("sim", dry_run_ops=True, runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")
    a.start()
    srv = serve(a, "127.0.0.1", 0)
    t0 = time.monotonic()
    while (a.world is None or len(a.world.robots) < 2) and time.monotonic() - t0 < 3:
        time.sleep(0.02)
    yield a, f"http://127.0.0.1:{srv.server_address[1]}"
    a.close()
    srv.shutdown()


def http(base, path, body=None):
    req = urllib.request.Request(base + path, json.dumps(body).encode() if body is not None else None,
                                 {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_manifest_is_complete_and_json(app):
    m = ops.manifest(app)
    assert m["version"] == ops.VERSION and json.loads(json.dumps(m)) == m
    names = [o["name"] for o in m["ops"]]
    assert len(names) == len(set(names))
    for o in m["ops"]:
        assert o["label"] and o["doc"] and o["group"] and o["safety"] in ops.SAFETY
        for p in o["params"].values():
            assert p["type"] in ("str", "num", "int", "bool", "robot", "robots", "dict")
    by = {o["name"]: o for o in m["ops"]}
    assert by["stop"]["key"] == "Space" and by["estop"]["key"] == "Esc"
    assert by["match.job"]["params"]["robot"]["choices"] == ["blue", "red"]
    assert by["robot.ready"]["ask"]


def test_keys_are_unique_except_mutually_exclusive_groups(app):
    keys = {}
    for o in ops.REGISTRY.values():
        if o.key:
            keys.setdefault(o.key, []).append(o.name)
    shared = {k: v for k, v in keys.items() if len(v) > 1}
    assert set(shared) == {"P"}, shared            # anything else sharing a key is a bug


def test_one_p_op_enabled_in_every_flow_phase(app):
    p_ops = [o.name for o in ops.REGISTRY.values() if o.key == "P"]

    def enabled():
        with app.lock:
            av = ops.availability(app)
        return [n for n in p_ops if av[n]["enabled"]]

    assert enabled() == ["match.play_here"]            # no match
    assert ops.call(app, "match.kickoff")["ok"]
    for phase, want in (("kickoff", "match.start_now"), ("restart", "match.start_now"), ("countdown", "match.pause"),
                        ("play", "match.pause"), ("goal", "match.pause"), ("celebrate", "match.pause"),
                        ("paused", "match.resume")):
        app.flow.phase = phase
        assert enabled() == [want], phase


def test_validation_and_error_codes(app):
    r = ops.call(app, "nope")
    assert not r["ok"] and r["code"] == "unknown_op"
    r = ops.call(app, "match.job", {"robot": "blue"})
    assert r["code"] == "bad_args" and "missing job" in r["error"]
    r = ops.call(app, "match.job", {"robot": "blue", "job": "fly", "extra": 1})
    assert r["code"] == "bad_args" and "job must be one of" in r["error"] and "unknown argument" in r["error"]
    assert ops.call(app, "match.touch", {"delta": True})["code"] == "bad_args"
    assert ops.call(app, "match.pause")["code"] == "unavailable"            # availability is enforced
    assert ops.call(app, "robot.ready", {"robot": "blue"})["code"] == "bad_args"     # confirm required
    assert ops.call(app, "robot.ready", {"robot": "blue", "confirm": False})["code"] == "bad_args"
    assert ops.call(app, "params.set", {"values": {"max_vx": 9}})["code"] == "refused"


def test_explicit_match_ops_are_deterministic(app):
    assert ops.call(app, "match.kickoff")["ok"]
    app.flow.phase = "play"
    assert ops.call(app, "match.pause")["ok"] and app.flow.phase == "paused"
    assert ops.call(app, "match.pause")["code"] == "unavailable"             # never a toggle
    assert ops.call(app, "match.resume")["ok"] and app.flow.phase == "countdown"
    with app.lock:
        nxt = ops.availability(app)["match.players"]
    assert nxt["label"] == "2 players" and nxt["args"] == {"robots": ["blue"]}
    assert ops.call(app, "match.players", nxt["args"])["playing"] == ["blue"]
    with app.lock:
        assert ops.availability(app)["match.players"]["label"] == "BLUE solo"
    assert ops.call(app, "match.end")["ok"] and app.flow is None


def test_estop_unknown_robot_stops_everyone_and_says_so(app):
    r = ops.call(app, "estop", {"robot": "zzz"})
    assert r["ok"] and r["robots"] == ["blue", "red"] and "unknown robot" in r["warnings"][0]
    assert set(app.estops) == {"blue", "red"}
    assert ops.call(app, "estop.clear")["ok"] and not app.estops


def test_stop_and_estop_are_never_refused_for_their_arguments(app):
    """A malformed stop/e-stop acts on every robot (like the legacy /action estop) and says so."""
    for args in ({"robot": 5}, {"robots": ["blue"]}, "all", ["blue"], {"robot": "blue", "now": True}):
        app.estops.clear()
        r = ops.call(app, "estop", args)
        assert r["ok"] and r["robots"] == ["blue", "red"] and "every robot" in r["warnings"][-1], args
        assert set(app.estops) == {"blue", "red"}
    for args in ({"robot": "blue"}, {"x": 1}, "now", None):
        r = ops.call(app, "stop", args)
        assert r["ok"], args
    assert "warnings" not in ops.call(app, "stop")
    app.estops.clear()
    assert ops.call(app, "estop", {"robot": "blue"})["robots"] == ["blue"] and set(app.estops) == {"blue"}
    assert ops.call(app, "match.touch", "all")["code"] == "bad_args"             # other ops stay strict


def test_robot_ops_unavailable_in_plain_sim(tmp_path):
    a = App("sim", runs_root=tmp_path / "runs", presets_dir=tmp_path / "presets")   # no --dry-run-ops
    try:
        with a.lock:
            av = ops.availability(a)
        robot_ops = [n for n in ops.REGISTRY if n.startswith("robot.")]
        assert robot_ops and all(not av[n]["enabled"] and "sim" in av[n]["reason"] for n in robot_ops)
        r = ops.call(a, "robot.ready", {"robot": "blue", "confirm": True})
        assert r["code"] == "unavailable" and not a.ops                       # nothing ran
    finally:
        a.close()


def test_robot_ready_needs_a_fresh_phase_and_home_to_home(app):
    from fc.play import PHASE_FRESH_S
    go = lambda **kw: ops.call(app, "robot.ready", {"robot": "blue", "confirm": True, **kw})  # noqa: E731
    r = go()
    assert r["code"] == "refused" and "robot.status" in r["error"] and not app.ops   # never checked
    app._set_phase("blue", "off")
    assert "HOME" in go()["error"] and not app.ops                           # off needs home=true (on the stand)
    app._set_phase("blue", "homed")
    assert "not off" in go(home=True)["error"] and not app.ops
    app.robot_phase_t["blue"] -= PHASE_FRESH_S + 1                           # Pi may have rebooted since
    assert "old" in go()["error"] and not app.ops
    app._set_phase("blue", "off")
    r = go(home=True)
    assert r["ok"] and "fcctl.sh ready stock --confirm" in r["cmd"] and r["seq"] == app.ops["blue"]["seq"]


def test_api_ops_and_op_over_http(server):
    app, base = server
    code, m = http(base, "/api/ops")
    assert code == 200 and any(o["name"] == "match.kickoff" for o in m["ops"])
    code, r = http(base, "/api/op", {"op": "match.kickoff", "args": {}})
    assert code == 200 and r["ok"] and r["op"] == "match.kickoff"
    assert http(base, "/api/op", {"op": "match.touch", "args": {"delta": 7}})[0] == 400
    assert http(base, "/api/op", {"op": "nope"})[0] == 404
    code, r = http(base, "/api/op", {"op": "match.resume"})
    assert code == 409 and r["code"] == "unavailable"
    code, r = http(base, "/api/op", {"op": "robot.status", "args": {"robot": "blue"}})
    assert code == 200 and r["cmd"].startswith("echo DRY-RUN")
    for args in ({"robot": 5}, "all", None):                                   # e-stop over HTTP: never a 400
        code, r = http(base, "/api/op", {"op": "estop", "args": args})
        assert code == 200 and r["ok"] and r["robots"] == ["blue", "red"], args


def test_lite_state_and_config(server):
    app, base = server
    code, full = http(base, "/state")
    code, lite = http(base, "/state?lite=1")
    assert full["schema"] == lite["schema"] == App.STATE_SCHEMA
    assert "limits" in full and "limits" not in lite and set(lite["params"]) <= set(App.LITE_PARAMS)
    assert lite["avail"]["stop"]["enabled"] is True
    assert len(json.dumps(lite)) < len(json.dumps(full)) * 0.6
    code, cfg = http(base, "/api/config")
    assert code == 200 and cfg["params"]["max_vx"] == app.params["max_vx"] and cfg["presets"] == [] and "step_s" in cfg["params"]


def test_source_switch_restarts_only_when_idle(app):
    from fc.app import _without_source, restart_argv
    assert _without_source(["--source", "sim", "--port", "1", "--source=camera", "--no-link"]) == ["--port", "1", "--no-link"]
    assert restart_argv("camera", "fast", ["--source", "sim", "--preset", "old", "--port", "1"]) == \
        ["--source", "camera", "--port", "1", "--preset", "fast"]
    assert restart_argv("sim", None, ["--preset=old", "--robots", "blue"]) == ["--source", "sim", "--robots", "blue",
                                                                                 "--preset", "base"]  # base stays base
    go = {"source": "camera", "confirm": True}
    assert ops.call(app, "app.source", go)["code"] == "unavailable"      # no launcher hook
    calls = []
    app.restart_hook = lambda src, preset=None: calls.append((src, preset))
    assert ops.call(app, "app.source", {"source": "camera"})["code"] == "bad_args"            # must be confirmed
    assert ops.call(app, "app.source", {"source": "sim", "confirm": True}) == \
        {"ok": True, "source": "sim", "restarting": False, "op": "app.source"}
    assert ops.call(app, "match.kickoff")["ok"]
    r = ops.call(app, "app.source", go)
    assert r["code"] == "unavailable" and "Stop" in r["error"]                           # never mid-match
    assert ops.call(app, "match.end")["ok"]
    app.presets_dir.mkdir(parents=True, exist_ok=True)
    (app.presets_dir / "fast.json").write_text(json.dumps({"max_vx": 0.3}))
    assert ops.call(app, "preset.load", {"name": "fast"})["ok"]
    r = ops.call(app, "app.source", go)
    assert r["restarting"] is True and r["preset"] == "fast"
    t0 = time.monotonic()
    while not calls and time.monotonic() - t0 < 2:
        time.sleep(0.02)
    assert calls == [("camera", "fast")]                                 # the loaded preset survives the restart


def test_show_warmup_toggles_and_refuses_during_a_match(app):
    with app.lock:
        assert ops.availability(app)["show.warmup"] == {"enabled": True, "label": "Warm-up"}
    r = ops.call(app, "show.warmup", {"loops": 1})
    assert r["ok"] and app.mode == "warmup" and app.warm.loops == 1 and r["robots"] == ["blue", "red"]
    with app.lock:
        assert ops.availability(app)["show.warmup"]["label"] == "End warm-up"
    r = ops.call(app, "show.warmup")
    assert r["ok"] and r["finishing"] and app.warm.finishing
    assert ops.call(app, "show.warmup", {"loops": 99})["code"] == "bad_args"
    assert ops.call(app, "match.kickoff")["ok"] and app.warm is None and app.flow is not None   # K replaces it
    r = ops.call(app, "show.warmup")
    assert r["code"] == "unavailable" and "match" in r["error"]


def test_robot_getup_needs_confirm_and_runs_fcctl_getup_without_a_fresh_phase(app):
    by = {o["name"]: o for o in ops.manifest(app)["ops"]}
    assert by["robot.getup"]["safety"] == "confirm" and "belly or feet" in by["robot.getup"]["ask"]
    assert ops.call(app, "robot.getup", {"robot": "blue"})["code"] == "bad_args"            # confirm required
    r = ops.call(app, "robot.getup", {"robot": "blue", "confirm": True})   # no phase known: getup never homes
    assert r["ok"] and r["cmd"].endswith("fcctl.sh getup") and app.ops["blue"]["op"] == "getup"
