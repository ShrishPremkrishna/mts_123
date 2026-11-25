"""Play screen logic without the app: match flow, ticker, banners, health lights, recorder."""
import time

import pytest

from fc.core import Ball, Cmd, RobotCfg, World, load_field
from fc.play import FLOW_DEFAULTS, MatchFlow, Ticker, action_label, banners, health

F = load_field()
BLUE = RobotCfg("blue", "blue", 10, "mts", "stock")
RED = RobotCfg("red", "red", 11, "", "stock")
P = {**FLOW_DEFAULTS, "match_s": 10.0, "countdown_s": 1.0, "goal_pause_s": 2.0, "ball_settle_s": 1.0}


def world(t, ball=None):
    return World(t, Ball(*ball, t=t) if ball else None, {})


def run(flow, t0, t1, ball=None, debug=None, goals_at=None, p=P):
    """Step the flow at 30 Hz from t0 to t1; returns the events and the end time."""
    evs, t = [], t0
    while t < t1:
        goals = goals_at.pop(0)[1:] if goals_at and goals_at[0][0] <= t else []
        evs += flow.update(t, 1 / 30, world(t, ball), debug or {}, list(goals), p)
        t += 1 / 30
    return evs, t


def test_kickoff_waits_for_spots_and_centred_still_ball():
    f = MatchFlow([BLUE, RED], F, 0.0)
    assert f.phase == "kickoff" and f.command("blue")[0] == "goto" and f.command("blue")[1][0] < 0
    assert f.command("red")[1][0] > 0 and f.command("red")[1][2] == pytest.approx(3.14159, abs=1e-4)
    done = {"blue": {"done": True}, "red": {"done": True}}
    run(f, 0, 2, ball=(0.5, 0.0), debug=done)                 # ball off centre: wait
    assert f.phase == "kickoff" and "ball at the centre spot" in f.waiting()
    run(f, 2, 4, ball=(0.0, 0.0), debug={"blue": {"done": True}})
    assert f.phase == "kickoff" and f.waiting() == ["Red to its spot"]
    evs, t = run(f, 4, 6.5, ball=(0.0, 0.0), debug=done)      # settle 1 s + countdown 1 s
    assert evs == ["play"] and f.phase == "play" and f.command("red") == ("brain", "auto")


def test_goal_pause_then_resume_on_still_ball_and_full_time():
    f = MatchFlow([BLUE, RED], F, 0.0, kickoff=False)
    assert f.phase == "countdown" and f.command("blue") == ("stop", "countdown")
    evs, t = run(f, 0, 2, ball=(0.3, 0.2), goals_at=[(1.5, "blue")])
    assert evs == ["play", "goal:blue"] and f.phase == "goal" and f.score == {"blue": 1, "red": 0}
    assert f.command("red") == ("stop", "goal")
    _, t = run(f, t, t + 2.1, ball=(1.4, 0.0))                 # ball in the net: not "in the field"
    assert f.phase == "restart" and f.waiting() == ["ball placed and still"]
    _, t = run(f, t, t + 2, ball=None)                         # hand over it: not seen
    assert f.phase == "restart"
    evs, t = run(f, t, t + 2.2, ball=(-0.4, 0.5))              # referee drops it anywhere, still
    assert evs == ["play"] and f.phase == "play"
    assert f.pause(t) and f.command("blue") == ("stop", "paused") and not f.pause(t)
    clock = f.clock
    run(f, t, t + 3, ball=(0, 0))
    assert f.clock == clock                                    # clock stops while paused
    assert f.resume(t + 3) and f.phase == "countdown"
    evs, _ = run(f, t + 3, t + 15, ball=(0, 0))
    assert "full_time" in evs and f.phase == "full_time" and f.clock >= 10.0


def test_restart_mode_kickoff_and_force_start():
    f = MatchFlow([BLUE], F, 0.0, kickoff=False)
    assert f.roles == {"blue": "attack"}                       # one robot: attacker by default
    _, t = run(f, 0, 1.5, ball=(0, 0), goals_at=[(1.2, "blue")], p={**P, "restart_mode": "kickoff"})
    _, t = run(f, t, t + 2.1, ball=(1.4, 0), p={**P, "restart_mode": "kickoff"})
    assert f.phase == "kickoff" and f.force_start(t) and f.phase == "countdown"
    assert MatchFlow([BLUE, RED], F, 0).command("ghost") == ("stop", "benched")


def test_ticker_narrates_roles_and_debounced_phases():
    tk = Ticker(debounce_s=0.2, gap_s=0.5)
    cfg = {"blue": BLUE, "red": RED}
    d = {"blue": {"role": "attack", "phase": "route", "my_cost": 2.0, "opp_cost": 3.2},
         "red": {"role": "defend", "phase": "guard", "my_cost": 3.2, "opp_cost": 2.0}}
    tk.update(0.0, d, cfg, ["blue", "red"])
    texts = [x["text"] for x in tk.items]
    assert texts == ["Blue attacks: reaches the ball 1.2 s sooner", "Red drops back to guard: the other robot is 1.2 s closer"]
    tk.update(0.1, d, cfg, ["blue", "red"])
    assert len(tk.items) == 2                                   # phase not stable long enough yet
    tk.update(0.3, d, cfg, ["blue", "red"])
    assert [x["text"] for x in list(tk.items)[2:]] == ["Blue: circling round the ball to get behind it",
                                                       "Red: guarding the goal mouth"]
    d["blue"]["phase"] = "push"
    tk.update(0.4, d, cfg, ["blue"])
    tk.update(0.7, d, cfg, ["blue"])
    assert len(tk.items) == 4                                   # within gap_s of the last blue line
    tk.update(0.9, d, cfg, ["blue"])
    assert tk.items[-1]["text"] == "Blue: pushing the ball at the goal" and tk.items[-1]["team"] == "blue"


def test_labels_banners_and_health_never_fake_ready():
    assert action_label({"phase": "approach"}) == "chasing the ball"
    assert action_label({"phase": "clear_guard"}) == "clearing" and action_label({}, "stale") == "can't see itself"
    cfg = {"blue": BLUE, "red": RED}
    cmds = {"blue": Cmd.stop("stale"), "red": Cmd(0, 0, 0, "ball_lost")}
    b = banners(cfg, ["blue", "red"], True, cmds, set(), set(), None, False)
    assert [x["text"] for x in b] == ["Blue paused: lost sight of its tag", "Ball out of sight: robots holding still"]
    b = banners(cfg, ["blue"], True, cmds, {"blue"}, set(), {"blue": {"connected": True}}, True)
    assert b[0]["level"] == "bad" and "IMU" in b[0]["text"]
    assert banners(cfg, ["blue"], False, cmds, set(), set(), {"blue": {"connected": False}}, True)[0]["text"] == \
        "Blue: no link to the robot"
    assert health("camera", True, True, None, True, None, None, False, False) == {"level": "off", "text": "not connected"}
    L = {"connected": True, "imu": {"level": "ok"}}
    assert health("camera", True, True, L, True, None, None, False, False)["text"] == "link ok"
    assert health("camera", True, True, L, True, None, "ready", False, False) == {"level": "ok", "text": "ready"}
    assert health("camera", True, False, L, True, None, "ready", False, False)["text"] == "not seen by camera"
    assert health("camera", True, True, L, True, None, "standing", False, False)["level"] == "warn"
    failed = {"op": "ready", "running": False, "rc": 1, "progress": {"ok": False, "error": "homing timeout"}}
    assert health("camera", True, True, L, True, failed, None, False, False)["text"] == "ready failed: homing timeout"
    assert health("sim", True, True, None, False, None, None, False, False)["text"] == "simulated"
    assert health("sim", False, True, None, False, None, None, False, False)["level"] == "off"


def test_recorder_writes_mp4(tmp_path):
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    from fc.media import Recorder
    frame = np.zeros((360, 640, 3), np.uint8)
    n = [0]

    def grab():
        n[0] += 1
        frame[:, : n[0] % 640] = 200
        return frame, n[0]

    rec = Recorder(tmp_path / "v.mp4", grab, fps=20)
    time.sleep(0.6)
    rec.stop()
    rec.thread.join(3)
    assert rec.error is None and rec.frames >= 8 and (tmp_path / "v.mp4").stat().st_size > 1000
    import cv2
    cap = cv2.VideoCapture(str(tmp_path / "v.mp4"))
    assert cap.isOpened() and int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == rec.frames


def test_camstream_serves_what_camfeed_reads():
    """Pi streamer (fc/robot/camstream.py) and laptop reader (fc/media.CamFeed) speak the same MJPEG."""
    import threading
    from http.server import ThreadingHTTPServer

    from fc.media import CamFeed
    from fc.robot.camstream import Latest, make_handler
    latest = Latest()
    srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(latest))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    stop = threading.Event()

    def frames():
        i = 0
        while not stop.is_set():
            latest.put(b"\xff\xd8" + str(i).encode() + b"\xff\xd9")
            i += 1
            time.sleep(0.03)

    threading.Thread(target=frames, daemon=True).start()
    feed = CamFeed(f"http://127.0.0.1:{srv.server_address[1]}/stream.mjpg")
    try:
        end = time.monotonic() + 3
        while time.monotonic() < end and feed.frames < 5:
            time.sleep(0.05)
        assert feed.frames >= 5 and feed.status()["ok"] and feed.latest().startswith(b"\xff\xd8")
    finally:
        stop.set()
        feed.close()
        srv.shutdown()


def test_safety_banners_merge_when_every_robot_is_estopped():
    from fc.core import RobotCfg
    from fc.play import safety_banners
    cfg = {r: RobotCfg(id=r, team=r, tag=t, host="", profile="stock") for r, t in (("blue", 10), ("red", 11))}
    one = {"blue": {"t": 0.0, "confirmed": True, "by": "operator"}}
    both = {**one, "red": {"t": 0.0, "confirmed": True, "by": "operator"}}
    assert [b["robot"] for b in safety_banners(cfg, ["blue", "red"], one, {}, 1.0, True, False, 3.0)] == ["blue"]
    merged = safety_banners(cfg, ["blue", "red"], both, {}, 1.0, True, False, 3.0)
    assert len(merged) == 1 and merged[0]["text"].startswith("ALL ROBOTS E-STOPPED")
    unconfirmed = {**one, "red": {"t": 0.9, "confirmed": False, "by": "operator"}}   # safety text must stay per robot
    assert len(safety_banners(cfg, ["blue", "red"], unconfirmed, {}, 1.0, True, False, 3.0)) == 2


def test_setup_step_walks_the_power_on_order_and_never_homes_blind():
    from fc.play import PHASE_FRESH_S, setup_step
    st = lambda phase, op=None, es=False, host="mts", age=1.0, linked=False: setup_step(  # noqa: E731
        "camera", host, False, phase, op, es, age=age, linked=linked)
    assert setup_step("sim", "", False, None, None, False)["step"] == "sim"
    assert st(None, host="")["step"] == "none" and st(None, host="")["op"] is None
    assert st(None, age=None)["op"] == "robot.status"                       # unknown: check first, never Ready
    assert [st(p, linked=p == "ready")["label"] for p in ("off", "homed", "standing", "ready")] == \
        ["Home legs", "Stand up", "Start link", "Ready"]
    assert [st(p, linked=True)["done"] for p in ("off", "homed", "standing", "ready")] == [0, 1, 2, 3]
    home = st("off")
    assert home["op"] == "robot.ready" and home["args"] == {"home": True} and "ON THE STAND" in home["ask"]
    assert all(st(p)["args"] == {} for p in ("homed", "standing"))          # only "off" may home
    assert st("ready", linked=True)["op"] is None
    assert st("homed", {"op": "ready", "running": True})["busy"] and st("homed", {"op": "ready", "running": True})["op"] is None
    assert st(None, {"op": "status", "running": False, "rc": 255})["step"] == "unreachable"
    assert st("ready", es=True, linked=True)["step"] == "rearm"
    assert st("fault")["op"] == "robot.stop_stack"


def test_setup_step_never_trusts_an_old_phase():
    """After a Pi reboot or battery swap a cached homed/standing would offer Stand up, but fcctl ready would
    HOME the legs: every robot.ready step needs a fresh phase, and a "ready" with no link acks is not Ready."""
    from fc.play import PHASE_FRESH_S, setup_step
    st = lambda phase, age, linked=False, es=False: setup_step("camera", "mts", False, phase, None, es,  # noqa: E731
                                                               age=age, linked=linked)
    old = PHASE_FRESH_S + 1
    for phase in ("off", "homed", "standing"):
        assert st(phase, old)["op"] == "robot.status" and st(phase, None)["op"] == "robot.status", phase
    assert st("homed", old, es=True)["op"] == "robot.status"                # re-arm too
    gone = st("ready", old)                                                  # stale ready, link down
    assert gone["step"] == "disconnected" and gone["op"] == "robot.status" and gone["label"] != "Ready"
    assert st("ready", 1.0)["step"] == "nolink" and st("ready", 1.0)["op"] is None   # Pi says up, no acks here
    assert st("ready", old, linked=True)["step"] == "ready"                  # live acks are evidence enough


def test_ready_refusal_needs_fresh_phase_and_home_flag():
    from fc.play import PHASE_FRESH_S, ready_refusal
    assert "robot.status" in ready_refusal(None, None, False)
    assert "robot.status" in ready_refusal("homed", PHASE_FRESH_S + 1, False)
    assert "HOME" in ready_refusal("off", 1.0, False)                       # homing needs home=true
    assert ready_refusal("off", 1.0, True) is None
    assert "not off" in ready_refusal("homed", 1.0, True)                   # confirmed for a stand, not this
    assert all(ready_refusal(p, 1.0, False) is None for p in ("homed", "standing", "ready"))
    assert ready_refusal("starting", 1.0, False) and ready_refusal("fault", 1.0, False)