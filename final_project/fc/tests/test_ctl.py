"""fc.ctl: parsing, formatting, batch control flow, and a live run against the sim app (dry-run robot ops)."""
import json
import math

import pytest

from fc import ctl
from fc.app import App, serve


# ---- pure parsing ---------------------------------------------------------------------------
def test_parse_kv_and_values():
    assert ctl.parse_kv(["vx=0.2", "robot=blue", "flag=true", "t=[1,2]"]) == {"vx": 0.2, "robot": "blue", "flag": True,
                                                                           "t": [1, 2]}
    for bad in (["vx"], ["=1"]):
        with pytest.raises(ctl.Usage):
            ctl.parse_kv(bad)


def test_test_body_translates_and_refuses_typos():
    b = ctl.test_body("goto", {"x": 0.5, "y": -0.4, "yaw": 90})
    assert b["action"] == "test" and b["test"] == "goto" and b["target"][:2] == [0.5, -0.4]
    assert math.isclose(b["target"][2], math.pi / 2, abs_tol=1e-3) and "x" not in b and "yaw" not in b
    assert ctl.test_body("goto", {"target": "0.1,0.2"})["target"] == [0.1, 0.2]
    assert ctl.test_body("push", {})["test"] == "push_ball"
    assert ctl.test_body("step", {"vx": 0.2, "seconds": 1.5, "robot": "blue"}) == {
        "action": "test", "test": "step", "vx": 0.2, "seconds": 1.5, "robot": "blue"}
    assert ctl.test_body("sim_eval", {"args": "--help"})["args"] == "--help"
    for name, kv in (("step", {"vz": 1}), ("goto", {"x": 1}), ("nope", {}), ("goto", {"x": 1, "y": 2, "yaw": "east"})):
        with pytest.raises(ctl.Usage):
            ctl.test_body(name, kv)


def test_parse_batch_and_pause():
    text = "# comment\n\nstatus\n  test step vx=0.1   # trailing\npause 12 place ball at centre\npause hello there\n"
    steps = ctl.parse_batch(text)
    assert steps == [(3, ["status"]), (4, ["test", "step", "vx=0.1"]), (5, ["pause", "12", "place", "ball", "at", "centre"]),
                     (6, ["pause", "hello", "there"])]
    assert ctl.parse_pause(steps[2][1][1:]) == (12.0, "place ball at centre")
    assert ctl.parse_pause(["hello", "there"]) == (0.0, "hello there")
    assert ctl._next_line(steps, 4) == 5 and ctl._next_line(steps, 6) == 7


def test_s1_batch_parses():
    from pathlib import Path
    path = Path(ctl.__file__).parent / "sessions" / "S1.batch"
    ap = ctl.build_parser()
    steps = ctl.parse_batch(path.read_text())
    cmds = [argv[0] for _, argv in steps]
    for argv in (a for _, a in steps if a[0] != "pause"):
        sub = ap.parse_args(argv)
        if sub.fn is ctl.cmd_test:
            ctl.test_body(sub.name, ctl.parse_kv(sub.pairs))
    assert cmds.count("estop") == 1 and cmds[-3:] == ["pause", "down", "status"]
    tests = [a[1] for _, a in steps if a[0] == "test"]
    assert tests.count("push_ball") == 10 and tests.count("goalie") == 5 and tests.count("deadman") == 1
    # the estop step needs a standing robot with the bridge up: a ready comes after the last down --keep-stack
    i_estop = cmds.index("estop")
    assert "ready" in cmds[cmds.index("down", cmds.index("set", len(cmds) - 30)):i_estop]


# ---- formatting -----------------------------------------------------------------------------------
STEP = {"run": "20261124T003016Z-step", "result": "done", "robot": "blue", "preset": "fast-play-robot", "time_s": 3.71,
        "cmd": [0.75, 0.0, 0.0], "gain_lin": 0.967, "gain_wz": None, "latency_s": 0.14, "onset_s": 0.3,
        "heading_bias_deg": -9.8, "lateral_m": -0.192, "yaw_drift_deg_per_m": -12.5, "dist_m": 0.874,
        "cmd_dist_m": 0.904, "pass": {"latency": True, "bias": False},
        "metrics": {"time_s": 3.72, "loop_overruns": 0, "robots": {
            "blue": {"seen_frac": 1.0, "path_m": 2.028, "stale_stops": 0, "wz_reversals": 0, "fast_reversals": 0,
                     "spin_frac": 0.0, "imu_worst": "ok", "loss": 0.0, "rtt_p95_ms": 8.2, "final": [0.191, -0.182, -0.253]},
            "red": {"seen_frac": 0.98, "path_m": 1.4}}}}


def test_fmt_summary_step_is_short_and_has_the_numbers():
    lines = ctl.fmt_summary(STEP, run_dir="/x/fc/runs/20261124T003016Z-step")
    assert 3 <= len(lines) <= 6 and all(len(x) < 220 for x in lines)
    assert lines[0].startswith("DONE step blue 3.7s preset fast-play-robot")
    assert "gain_lin 0.97" in lines[1] and "latency 0.14s" in lines[1] and "latency=ok bias=FAIL" in lines[1]
    assert lines[2].startswith("blue: seen 1 ") and "final (0.19,-0.18,-14°)" in lines[2] and "red" not in "".join(lines)
    assert lines[-1] == "dir /x/fc/runs/20261124T003016Z-step"
    assert len(ctl.fmt_summary(STEP, compact=True)) == 2


def test_fmt_summary_other_tests_and_problems():
    dm = {"result": "stopped", "robot": "blue", "stop_time_s": 0.32, "coast_m": 0.029, "speed_before": 0.108,
          "vx": 0.1, "mute_s": 2.0, "pass": True, "emulated": True}
    assert "stop_time 0.32s" in ctl.fmt_summary(dm, "R-deadman")[1]
    goto = {"result": "reached", "robot": "blue", "target": [0.2, 0.3], "final": [0.16, 0.212, 0.821],
            "pos_err_m": 0.096, "heading_err_deg": -11.2, "pass": False}
    assert "pos_err 0.096m" in ctl.fmt_summary(goto, "R-goto")[1] and "pass FAIL" in ctl.fmt_summary(goto, "R-goto")[1]
    assert ctl.fmt_summary({"result": "done", "conceded": 1, "saved": 2, "shots": 3}, "R-goalie")[1] == \
        "conceded 1 saved 2 shots 3"
    assert "blue 2 - red 1" in ctl.fmt_summary({"result": "full_time", "score": {"blue": 2, "red": 1}}, "R-match")[1]
    est = ctl.fmt_summary({"result": "estopped", "robot": "blue", "estop": {"blue": {"by": "operator"}}}, "R-step")
    assert est[0].startswith("ESTOPPED") and any(x.startswith("! estop") for x in est)
    assert ctl.fmt_summary(None, "R-step") == ["R-step: no summary (still running, or the app was stopped)"]
    sim_eval = ctl.fmt_summary({"result": "ok", "returncode": 0, "output_tail": "a\nb\nc\nd\n"}, "R-sim_eval")
    assert sim_eval[1].splitlines()[1:] == ["  | b", "  | c", "  | d"]
    assert ctl.fmt_summary({"result": "ok", "returncode": 0, "output_tail": "a\nb\n"}, "R-sim_eval", compact=True)[1] == "rc 0"


def test_fmt_pass():
    assert ctl.fmt_pass({"latency": True, "bias": None}) == "latency=ok bias=-"   # a turn step has no walk bias
    assert ctl.fmt_pass(False) == "FAIL" and ctl.fmt_pass(None) == "-"


def test_run_test_name():
    assert ctl.run_test_name({}, "20261124T011715Z-step-2") == "step"
    assert ctl.run_test_name({}, "20261124T011715Z-push_ball") == "push_ball"
    assert ctl.run_test_name({"test": "goto"}, "x") == "goto" and ctl.run_test_name({}, None) == "?"


def test_health_stop_classification():
    assert ctl.is_health_stop({"result": "stopped"})                          # operator STOP
    assert not ctl.is_health_stop({"result": "stopped", "stop_time_s": 0.3})  # deadman came to rest
    for r in ("estopped", "imu_bad", "mux_lost", "edge", "replaced", "loop_error"):
        assert ctl.is_health_stop({"result": r})
    for r in ("done", "goal", "timeout", "reached", "still_moving", "full_time", "own_goal", "lost_tag"):
        assert not ctl.is_health_stop({"result": r})
    assert ctl.is_health_stop(None)


CAMERA_STATE = {
    "source": "camera", "mode": "step", "dry_run_ops": False, "active": ["blue"],
    "test": {"name": "step", "robot": "blue", "elapsed": 1.24, "phase": "cmd"},
    "preset": {"name": "fast-play-robot", "modified": ["max_vx"], "stack": {"speed_cap": 0.8}},
    "loop": {"hz": 30.04, "overruns": 2}, "params": {"stale_stop_s": 0.5},
    "world": {"ball": {"x": 0.1, "y": -0.2, "age": 0.05}, "robots": {"blue": {"x": -0.8, "y": 0.0, "yaw": 0.05, "age": 0.06}}},
    "eye": {"fps": 29.6, "calibrated": True, "calib_err_px": 0.84, "message": "4 corner tags"},
    "link": {"blue": {"auth": "ok", "connected": True, "rtt_p95_ms": 9.4, "loss": 0.0, "ack_age": 0.03,
                      "imu": {"level": "ok", "tilt_deg": 2.1}, "hold": None}},
    "stack": {"blue": {"speed_cap": 0.8, "heading_hold": "on", "warn": []}},
    "estop": {}, "imu_bad": [], "mux_lost": [],
    "ops": {"blue": {"op": "ready", "running": False, "rc": 0}},
    "play": {"robots": {"blue": {"health": {"level": "ok", "text": "ready"}, "phase": "ready"}}, "banners": []},
}


def test_fmt_status_camera():
    lines = ctl.fmt_status(CAMERA_STATE)
    assert len(lines) == 3
    assert lines[0] == ("camera · mode step · test step(blue) 1.2s cmd · preset fast-play-robot* [speed_cap=0.8]"
                        " · loop 30Hz ovr 2 · ball (0.1,-0.2) 0.1s")
    assert lines[1].startswith("eye 29.6fps calib ok err 0.84px")
    b = lines[2]
    for part in ("blue: OK 'ready'", "phase ready", "link ok p95 9.4ms loss 0 ack 0", "imu ok 2.1°", "cap 0.8 hh on",
                 "pose (-0.8,0,3°) 0.1s", "op ready rc 0"):
        assert part in b, part


def test_fmt_status_estopped_and_unseen():
    st = {**CAMERA_STATE, "estop": {"blue": {"confirmed": False, "by": "operator", "text": "x"}}, "imu_bad": ["blue"],
          "world": {"ball": None, "robots": {}}, "eye": None, "test": None,
          "stack": {"blue": {"speed_cap": None, "warn": ["max_vx 0.75 > stock walker cap 0.49"]}},
          "play": {"robots": {"blue": {"health": {"level": "bad", "text": "E-STOPPED"}}},
                   "banners": [{"kind": "estop", "text": "Blue E-STOP sent: waiting"}]}}
    lines = ctl.fmt_status(st)
    assert "ball NOT SEEN" in lines[0] and "test -" in lines[0]
    assert "ESTOPPED(UNCONFIRMED by operator)" in lines[1] and "IMU-BAD" in lines[1] and "pose NOT SEEN" in lines[1]
    assert lines[2].startswith("  warn: max_vx") and lines[3].startswith("  banner: Blue E-STOP")


def test_check_robot_gates():
    ok, notes = ctl.check_robot(CAMERA_STATE, "blue", ball=True, yaw=0)
    assert ok and notes[0].startswith("ok: ")
    bad = {**CAMERA_STATE, "eye": {**CAMERA_STATE["eye"], "fps": 9, "calib_err_px": 2.0},
           "link": {"blue": {**CAMERA_STATE["link"]["blue"], "rtt_p95_ms": 31.0, "loss": 0.05}}}
    ok, notes = ctl.check_robot(bad, "blue", yaw=90)
    assert not ok and all(k in notes[0] for k in ("eye 9fps", "calib 2px", "p95 31ms", "loss 0.05", "yaw 3°"))
    assert not ctl.check_robot(CAMERA_STATE, "red")[0]
    sim = {**CAMERA_STATE, "source": "sim", "eye": None, "link": None}
    assert ctl.check_robot(sim, "blue", yaw=90)[0]       # sim: the yaw gate is about the camera, not checked


def test_progress_line():
    s = ctl.progress_line({"phase": "waiting_operator", "ok": True, "elapsed": 12.04, "msg": "waiting",
                           "action": "press Options"})
    assert s == "    12.0s waiting_operator: waiting [action press Options]"
    s = ctl.progress_line({"phase": "error", "ok": False, "elapsed": 2.1, "msg": "still standing", "at": "lying_down"})
    assert "error FAIL: still standing [at lying_down]" in s


# ---- batch control flow with a fake client ----------------------------------------------------------
class FakeClient:
    def __init__(self, fail_on=None):
        self.posts, self.fail_on = [], fail_on

    def state(self):
        return {"source": "sim", "active": ["blue"], "params": {}, "preset": {}, "estop": {}}

    def get(self, path):
        return []

    def post(self, path, body):
        self.posts.append((path, body))
        if self.fail_on and body.get(self.fail_on[0]) == self.fail_on[1]:
            return {"ok": False, "error": "nope"}
        return {"ok": True, "applied": body, "errors": []}


def _batch(tmp_path, text, *flags):
    f = tmp_path / "b.batch"
    f.write_text(text)
    ap = ctl.build_parser()
    return ap, ap.parse_args(["batch", str(f), *flags])


def test_batch_stops_at_pause_with_resume_line(tmp_path, capsys):
    ap, a = _batch(tmp_path, "set max_vx=0.2\npause 3 place ball at centre\nset max_vx=0.3\n", "--stop-at-pause")
    c = FakeClient()
    with pytest.raises(ctl.Paused):
        ctl.run_batch(c, a, ap)
    out = capsys.readouterr().out
    assert "PAUSED at line 2: place ball at centre" in out and "--from 3" in out
    assert c.posts == [("/params", {"max_vx": 0.2})]
    a.start = 3
    ctl.run_batch(c, a, ap)                                   # resume
    assert c.posts[-1] == ("/params", {"max_vx": 0.3}) and "BATCH DONE" in capsys.readouterr().out


def test_batch_no_pause_sleeps_capped(tmp_path, capsys, monkeypatch):
    slept = []
    monkeypatch.setattr(ctl.time, "sleep", lambda s: slept.append(s))
    ap, a = _batch(tmp_path, "pause 300 cool down\nsleep 2\n", "--no-pause", "--max-pause", "1")
    ctl.run_batch(FakeClient(), a, ap)
    assert slept == [1.0, 2.0]


def test_batch_interactive_pause_q_stops(tmp_path, capsys, monkeypatch):
    answers = iter(["", "q"])
    monkeypatch.setattr("builtins.input", lambda prompt="": next(answers))
    ap, a = _batch(tmp_path, "pause one\nstop\npause two\nstop\n")
    c = FakeClient()
    with pytest.raises(ctl.Paused):
        ctl.run_batch(c, a, ap, interactive=True)
    assert len(c.posts) == 1 and "--from 4" in capsys.readouterr().out


def test_batch_stops_at_first_failure_and_validates_first(tmp_path, capsys):
    ap, a = _batch(tmp_path, "preset slow\npreset nope\nstop\n")
    c = FakeClient(fail_on=("name", "nope"))
    with pytest.raises(ctl.Fail):
        ctl.run_batch(c, a, ap)
    out = capsys.readouterr().out
    assert "BATCH STOPPED at line 2 (FAILED)" in out and "--from 2" in out
    assert [b for _, b in c.posts] == [{"op": "load", "name": "slow"}, {"op": "load", "name": "nope"}]
    ap, a = _batch(tmp_path, "stop\ntest step vz=1\n")         # typo on a later line: nothing runs
    c = FakeClient()
    with pytest.raises(ctl.Usage):
        ctl.run_batch(c, a, ap)
    assert not c.posts
    ap, a = _batch(tmp_path, "batch other.batch\n")
    with pytest.raises(ctl.Usage):
        ctl.run_batch(FakeClient(), a, ap)


def test_main_exit_codes(capsys):
    assert ctl.main([]) == 2
    assert ctl.main(["test", "nope"]) == 2
    assert ctl.main(["--url", "http://127.0.0.1:9", "status"]) == 1
    assert "app not reachable" in capsys.readouterr().out


# ---- live: the real app in sim, robot ops dry-run ----------------------------------------------------
@pytest.fixture
def live(tmp_path):
    app = App("sim", dry_run_ops=True, runs_root=tmp_path / "runs", presets_dir=ctl_presets(tmp_path))
    app.start()
    srv = serve(app, "127.0.0.1", 0)
    url = f"http://127.0.0.1:{srv.server_address[1]}"
    import time
    end = time.monotonic() + 3
    while not (app.world and len(app.world.robots) == 2) and time.monotonic() < end:
        time.sleep(0.05)
    yield app, url
    app.close()
    srv.shutdown()


def ctl_presets(tmp_path):
    d = tmp_path / "presets"
    d.mkdir()
    (d / "capped.json").write_text('{"_stack": {"speed_cap": 0.8}, "max_vx": 0.2}\n')
    return d


def test_live_status_step_and_ops(live, capsys):
    app, url = live
    run = lambda *a: ctl.main(["--url", url, *a])  # noqa: E731
    assert run("status") == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0].startswith("sim dry-run-ops · mode idle") and out[1].startswith("blue: OK 'simulated'")
    assert run("test", "step", "vx=0.2", "seconds=0.6", "--wait") == 0
    out = capsys.readouterr().out.splitlines()
    assert 3 <= len(out) <= 6 and out[0].startswith("DONE step blue") and "gain_lin" in out[1]
    assert run("last", "1") == 0 and capsys.readouterr().out.startswith("DONE step blue")
    assert run("ready", "blue", "--preset", "capped") == 0
    out = capsys.readouterr().out
    assert "preset capped: 1 keys, stack speed_cap=0.8" in out and "--speed-cap 0.80" in out and "READY blue (dry run" in out
    assert run("down", "blue", "--keep-stack") == 0
    assert "fcctl.sh down --keep-stack" in capsys.readouterr().out
    assert run("estop", "blue") == 0 and "E-STOP blue: confirmed" in capsys.readouterr().out
    assert run("test", "step", "vx=0.1", "--wait") == 1             # refused while latched
    assert "estopped" in capsys.readouterr().out
    assert run("ready", "blue") == 0 and "blue" not in app.estops     # dry-run Ready re-arms
    assert run("set", "max_vx=0.1") == 0 and app.params["max_vx"] == 0.1
    assert run("set", "max_vx=9") == 1
    assert run("drive", "blue", "0.2", "0", "0", "0.5") == 0
    out = capsys.readouterr().out
    assert "DRIVE blue vx 0.2" in out and "clipped to max_vx 0.1" in out and app.mode == "idle"


def test_live_health_stop_and_sim_skips_robot_ops(live, capsys, tmp_path):
    app, url = live
    run = lambda *a: ctl.main(["--url", url, *a])  # noqa: E731
    import threading
    import time
    threading.Timer(0.8, app.stop).start()                          # operator presses space mid-test
    assert run("test", "push_ball", "--wait") == 3
    assert "HEALTH STOP" in capsys.readouterr().out
    app.dry_run_ops = False                                         # plain sim: ops would ssh the robot
    app.estop("blue")
    assert run("ready", "blue", "--home") == 0 and "blue" not in app.estops and not app.ops
    assert run("down", "blue") == 0 and not app.ops
    assert "skipped" in capsys.readouterr().out
    time.sleep(0.1)


class RealOpsClient:
    """Camera source, real robot ops: /robot posts finish at once with scripted fcctl output."""

    def __init__(self, status_phase, ready_lines, connected=False, rc=0):
        self.posts, self.ops = [], {}
        self.out = {"status": json.dumps({"phase": status_phase}) + "\n" if status_phase else "ssh: timeout\n",
                    "ready": "".join(json.dumps(x) + "\n" for x in ready_lines), "down": ready_lines and ""}
        self.connected, self.rc = connected, rc

    def state(self):
        return {"source": "camera", "dry_run_ops": False, "active": ["blue"], "ops": self.ops, "preset": {"stack": {}},
                "link": {"blue": {"connected": self.connected}}, "estop": {}}

    def get(self, path):
        return {}

    def post(self, path, body):
        self.posts.append((path, body))
        if path == "/robot":
            op = body["op"]
            self.ops["blue"] = {"op": op, "cmd": f"ssh mts {op}", "running": False, "out": self.out.get(op, ""),
                                "rc": self.rc if op == "ready" else 0}
            return {"ok": True, "cmd": f"ssh mts {op}"}
        return {"ok": True}


READY_LINES = [{"phase": "begin", "ok": True, "elapsed": 0.0, "msg": "profile stock"},
               {"phase": "homed", "ok": True, "elapsed": 0.9, "msg": "homed", "speed_cap": 0.49, "heading_hold": "on"},
               {"phase": "ready", "ok": True, "elapsed": 11.2, "msg": "ready to play", "done": True}]


def test_ready_refuses_to_home_without_flag(capsys):
    ap = ctl.build_parser()
    c = RealOpsClient("off", READY_LINES)
    with pytest.raises(ctl.Fail, match="HOME the legs"):
        ctl.cmd_ready(c, ap.parse_args(["ready", "blue"]))
    assert [b["op"] for p, b in c.posts if p == "/robot"] == ["status"]
    c = RealOpsClient(None, READY_LINES)                       # status unreadable: same refusal
    with pytest.raises(ctl.Fail, match="phase unknown"):
        ctl.cmd_ready(c, ap.parse_args(["ready", "blue"]))
    c = RealOpsClient("off", READY_LINES)
    ctl.cmd_ready(c, ap.parse_args(["ready", "blue", "--home"]))
    assert c.posts[-1] == ("/robot", {"robot": "blue", "op": "ready", "confirm": True})
    out = capsys.readouterr().out
    assert "homed [speed_cap 0.49] [heading_hold on]" in out and out.rstrip().endswith("READY blue · cap 0.49 hh on")


def test_ready_without_status_when_bridge_acks_and_homed_next(capsys):
    ap = ctl.build_parser()
    c = RealOpsClient("off", READY_LINES[:1] + [{"phase": "homed", "ok": True, "next": "ready", "done": True}],
                      connected=True)
    ctl.cmd_ready(c, ap.parse_args(["ready", "blue"]))         # bridge acks: stack runs, cannot home
    assert [b["op"] for p, b in c.posts if p == "/robot"] == ["ready"]
    assert "HOMED blue: move it to the floor" in capsys.readouterr().out
    c = RealOpsClient("homed", [{"phase": "error", "ok": False, "msg": "another profile running", "at": "profile"}], rc=1)
    with pytest.raises(ctl.Fail, match="another profile running"):
        ctl.cmd_ready(c, ap.parse_args(["ready", "blue"]))
