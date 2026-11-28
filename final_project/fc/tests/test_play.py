"""Play screen logic without the app: match flow, ticker, banners, health lights, recorder."""
import math
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

# ---- autoplay: the hands-off solo loop ------------------------------------------------------------
from fc.core import Pose, RobotObs  # noqa: E402
from fc.play import AUTO_DEFAULTS, AutoPlay  # noqa: E402


def aw(t, ball=None, pose=None):
    return World(t, Ball(*ball, t=t) if ball else None, {"blue": RobotObs("blue", Pose(*pose), t)} if pose else {})


def arun(a, t0, t1, ball=None, pose=None, goals_at=None):
    evs, t = [], t0
    while t < t1:
        goals = [goals_at.pop(0)[1]] if goals_at and goals_at[0][0] <= t else []
        evs += a.update(t, aw(t, ball, pose), goals)
        t += 1 / 30
    return evs, t


def test_autoplay_full_cycle_goal_net_roll_out_dribble_back_alternate():
    a = AutoPlay(BLUE, F, 0.0, minutes=0)
    assert a.phase == "countdown" and a.want == ("stop", "countdown")
    pose = (0.5, 0.3, 0.0)
    evs, t = arun(a, 0, 2, ball=(0.8, 0.2), pose=pose)
    assert "attack" in evs and a.want == ("attack", "blue")                 # blue attacks east, ball anywhere
    evs, t = arun(a, t, t + 0.2, ball=(1.3, 0.1), pose=pose, goals_at=[(t, "blue")])
    assert "goal:east" in evs and a.phase == "goal" and a.want == ("stop", "goal")
    _, t = arun(a, t, t + 3, ball=(1.45, 0.1), pose=pose)                  # resting in the net
    assert a.phase == "wait_ball" and a.banner["text"] == "Waiting for the ball…"
    _, t = arun(a, t, t + AUTO_DEFAULTS["out_s"], ball=(1.45, 0.1), pose=pose)
    assert a.banner["level"] == "bad" and "east goal: please roll it out" in a.banner["text"]
    _, t = arun(a, t, t + 30, ball=(1.45, 0.1), pose=pose)                 # never times out
    assert a.phase == "wait_ball"
    evs, t = arun(a, t, t + 0.1, ball=(0.6, 0.0), pose=pose)               # a person rolled it out
    assert "rolled" in evs and a.interventions == 1 and a.phase == "settle"
    _, t = arun(a, t, t + 1.5, ball=(0.6, 0.0), pose=pose)                 # at rest -> line up behind it
    assert a.phase == "stage" and a.want[0] == "goto"
    # robot in front of the ball (between it and the centre): it walks round, never straight through
    x, y, yaw, _v = a.want[1]
    assert yaw is None and abs(y) > 0.4
    stage = (0.6 + 0.269 + AUTO_DEFAULTS["stage_gap_m"], 0.0, math.pi)     # behind it, facing the centre
    _, t = arun(a, t, t + 0.2, ball=(0.6, 0.0), pose=stage)
    assert a.phase == "push" and a.want[0] == "goto" and a.want[1][3] == AUTO_DEFAULTS["reset_vx"]
    assert a.want[1][0] == pytest.approx(0.269, abs=0.01)                  # stops with the ball on the spot
    _, t = arun(a, t, t + 0.2, ball=(0.05, 0.0), pose=(0.32, 0.0, math.pi))
    assert a.phase == "settle"
    _, t = arun(a, t, t + 1.5, ball=(0.05, 0.0), pose=(0.32, 0.0, math.pi))
    assert a.phase == "backoff" and a.want[0] == "app" and a.want[1].vx < 0  # backs away from the ball
    evs, t = arun(a, t, t + 0.2, ball=(0.05, 0.0), pose=(0.75, 0.0, math.pi))
    assert a.phase == "countdown"
    evs, t = arun(a, t, t + 2, ball=(0.05, 0.0), pose=(0.75, 0.0, math.pi))
    assert "attack" in evs and a.want == ("attack", "red")                  # alternate: the west goal, straight ahead
    s = a.summary(t)
    assert s["goals"] == 1 and s["cycles"] == 1 and s["interventions"] == 1 and s["own_goals"] == 0
    c = s["cycle_log"][0]
    assert c["goal"] == "east" and c["reset"] == "centre" and c["interventions"] == 1 and c["reset_s"] > 30
    assert c["wait_s"] > 30 and s["time_per_goal_s"] == c["attack_s"]


def test_autoplay_push_direction_and_unreachable_balls():
    a = AutoPlay(BLUE, F, 0.0)
    u = a.push_dir(0.6, 0.3)
    assert u[0] < 0 and u[1] < 0 and math.hypot(*u) == pytest.approx(1)
    hl, hw = F.length / 2, F.width / 2
    assert a.push_dir(hl - 0.1, hw - 0.1) is None                          # corner
    assert a.push_dir(hl - 0.05, 0.0) is None                              # on the goal line in the mouth
    assert a.push_dir(0.3, hw - 0.12) is None                              # on a wall: the brain's attack handles it
    u = a.push_dir(0.3, hw - 0.3)                                          # near a wall: rotated, still toward the centre
    assert u is not None and u[1] < 0.05 and u[0] * -0.3 + u[1] * -(hw - 0.3) > 0


def test_autoplay_unreachable_attacks_nearer_goal_then_asks_for_help():
    a = AutoPlay(BLUE, F, 0.0, minutes=0, alternate=True, attack_max_s=5.0)
    pose, corner = (0.0, 0.0, 0.0), (-(F.length / 2 - 0.1), F.width / 2 - 0.1)
    _, t = arun(a, 0, 2, ball=corner, pose=pose)
    _, t = arun(a, t, t + 6, ball=corner, pose=pose)                       # attack times out
    assert a.attack_timeouts == 1
    evs, t = arun(a, t, t + 3.5, ball=corner, pose=pose)
    assert "attack" in evs and a.team == "red"                              # west goal is nearer the ball
    _, t = arun(a, t, t + 6, ball=corner, pose=pose)
    _, t = arun(a, t, t + 1.5, ball=corner, pose=pose)
    assert a.phase == "wait_ball" and "can't get the ball out" in a.banner["text"]
    _, t = arun(a, t, t + 10, ball=corner, pose=pose)
    assert a.phase == "wait_ball"                                           # still there: keep asking
    evs, t = arun(a, t, t + 0.1, ball=(-0.5, 0.2), pose=pose)
    assert "rolled" in evs and a.phase == "settle"


def test_autoplay_waits_for_robot_and_ends_on_minutes():
    a = AutoPlay(BLUE, F, 0.0, minutes=0.1, reset="none")
    _, t = arun(a, 0, 2, ball=(0.5, 0.0), pose=(0.0, 0.0, 0.0))
    _, t = arun(a, t, t + 0.1, ball=(0.5, 0.0), pose=(0.0, 0.0, 0.0), goals_at=[(t, "red")])
    assert a.cycles == [] and a.cur["own"] is True                          # west goal while attacking east
    _, t = arun(a, t, t + 3.5, ball=(0.5, 0.0), pose=None)                  # reset none: attack from there
    assert a.phase == "countdown"
    evs, _ = arun(a, t, 6.1, ball=(0.5, 0.0), pose=None)
    assert evs[-1] == "done" and a.summary(6.1)["own_goals"] == 1
    b = AutoPlay(BLUE, F, 0.0, minutes=0)
    b._set("stage", 0.0)
    b.update(0.1, aw(0.1, (0.5, 0.0), None), [])
    assert b.want == ("stop", "stale") and b.banner["text"] == "Robot not seen: waiting"


# ---- show: celebration after a goal, warm-up ----------------------------------------------------
from fc.core import Pose, RobotObs  # noqa: E402
from fc.play import CEL_STEP_IN_S, WARM, Warmup, kickoff_spot, show_margin  # noqa: E402

SHOW = {**P, "celebrate_style": "spin", "celebrate_s": 2.0, "celebrate_wz": 1.8, "celebrate_other": "walk",
        "restart_mode": "kickoff", "kickoff_x_frac": 0.25, "wall_margin_m": 0.28}


def robots_world(t, poses, ball=None):
    return World(t, Ball(*ball, t=t) if ball else None,
                 {rid: RobotObs(rid, Pose(*p), t) for rid, p in poses.items()})


def test_spots_and_margin():
    assert kickoff_spot("blue", F, 0.25) == (-0.25 * F.length, 0.0, 0.0)
    assert kickoff_spot("red", F, 0.25)[0] == 0.25 * F.length and kickoff_spot("red", F, 0.25)[2] == pytest.approx(math.pi)
    assert show_margin({"wall_margin_m": 0.0}) == 0.28 and show_margin({"wall_margin_m": 0.35}) == 0.35


def test_celebrate_steps_inside_the_margin_spins_then_kickoff_spots():
    f = MatchFlow([BLUE, RED], F, 0.0, kickoff=False)
    hl = F.length / 2
    poses = {"blue": (hl - 0.1, 0.1, 0.0), "red": (0.3, 0.0, math.pi)}      # blue scored right at the east fence
    t = 0.0
    while f.phase != "play":
        f.update(t, 1 / 30, robots_world(t, poses), {}, [], SHOW)
        t += 1 / 30
    assert f.update(t, 1 / 30, robots_world(t, poses), {}, ["blue"], SHOW) == ["goal:blue"]
    assert f.phase == "celebrate" and f.scorer == "blue" and f.score["blue"] == 1
    f.update(t, 1 / 30, robots_world(t, poses), {}, [], SHOW)
    kind, arg = f.command("blue")
    assert kind == "goto" and arg[0] <= hl - 0.28 - 0.15 + 1e-9 and arg[3] == 0.3    # step back inside first
    assert f.command("red") == ("goto", (0.25 * F.length, 0.0, math.pi, 0.15))      # slow walk to its spot
    poses["blue"] = (hl - 0.5, 0.1, 0.0)                                            # now inside
    t += 0.5
    f.update(t, 1 / 30, robots_world(t, poses), {}, [], SHOW)
    t += 0.5
    f.update(t, 1 / 30, robots_world(t, poses), {}, [], SHOW)
    kind, cmd = f.command("blue")
    assert kind == "app" and (cmd.vx, cmd.vy, cmd.wz, cmd.tag) == (0.0, 0.0, 1.8, "celebrate")
    assert f.json(t, SHOW)["celebrate"] == {"blue": "party", "red": "walk"} and f.clock < 5
    clock = f.clock
    while f.phase == "celebrate":
        t += 1 / 30
        f.update(t, 1 / 30, robots_world(t, poses), {}, [], SHOW)
    assert f.phase == "kickoff" and f.clock == clock and f.command("blue")[0] == "goto"   # clock stood still


def test_celebrate_wiggle_flips_and_is_capped_without_a_pose():
    f = MatchFlow([BLUE, RED], F, 0.0, kickoff=False)
    f.phase = "play"
    p = {**SHOW, "celebrate_style": "wiggle", "celebrate_other": "still"}
    poses = {"blue": (0.5, 0.0, 0.0), "red": (0.8, 0.0, math.pi)}
    f.update(0.0, 1 / 30, robots_world(0.0, poses), {}, ["red"], p)
    signs = set()
    t = 0.0
    while t < 1.5:
        t += 1 / 30
        f.update(t, 1 / 30, robots_world(t, poses), {}, [], p)
        k, c = f.command("red")
        if k == "app":
            signs.add(c.wz > 0)
    assert signs == {True, False} and f.command("blue") == ("stop", "goal")       # red wiggles, blue holds
    g = MatchFlow([BLUE, RED], F, 0.0, kickoff=False)                               # scorer never seen: time cap
    g.phase = "play"
    g.update(0.0, 1 / 30, robots_world(0.0, {}), {}, ["blue"], p)
    t = 0.0
    while g.phase == "celebrate" and t < 20:
        t += 1 / 30
        g.update(t, 1 / 30, robots_world(t, {}), {}, [], p)
    assert g.phase == "kickoff" and t <= p["celebrate_s"] + CEL_STEP_IN_S + 1.2


def test_celebrate_none_keeps_the_old_goal_pause():
    f = MatchFlow([BLUE, RED], F, 0.0, kickoff=False)
    f.phase = "play"
    f.update(0.0, 1 / 30, world(0.0), {}, ["blue"], P)
    assert f.phase == "goal" and f.command("blue") == ("stop", "goal")


def _drive(w, poses, now, dt):
    """Toy kinematics: app commands integrate in the body frame; gotos walk straight at vmax, then turn."""
    for rid, (kind, arg) in w.want.items():
        x, y, yaw = poses[rid]
        if kind == "app":
            c, s = math.cos(yaw), math.sin(yaw)
            x, y, yaw = x + dt * (c * arg.vx - s * arg.vy), y + dt * (s * arg.vx + c * arg.vy), yaw + dt * arg.wz
        elif kind == "goto":
            tx, ty, tyaw, v = arg
            d = math.hypot(tx - x, ty - y)
            if d > 0.02:
                k = min(1.0, v * dt / d)
                x, y = x + k * (tx - x), y + k * (ty - y)
            elif tyaw is not None:
                yaw = tyaw
        poses[rid] = (x, y, yaw)


def test_warmup_loops_the_move_table_and_finishes_at_the_spots():
    w = Warmup([BLUE, RED], F, 0.0, loops=1)
    p = {"kickoff_x_frac": 0.25, "wall_margin_m": 0.28}
    poses = {"blue": (-0.7, 0.2, 0.3), "red": (0.7, -0.1, 3.0)}
    seen, t, dt, evs = set(), 0.0, 1 / 30, []
    margin = show_margin(p)
    while t < 120:
        evs = w.update(t, robots_world(t, poses), {}, p)
        if "done" in evs:
            break
        seen |= {st["step"] for st in w.st.values()}
        _drive(w, poses, t, dt)
        for rid, (x, y, _) in poses.items():
            assert abs(x) <= F.length / 2 - margin + 0.02 and abs(y) <= F.width / 2 - margin + 0.02
        assert math.hypot(poses["blue"][0] - poses["red"][0], poses["blue"][1] - poses["red"][1]) > WARM["gap_m"] - 0.1
        t += dt
    assert "done" in evs and seen >= {"home", "trot", "circle", "shuffle", "spin"}
    for rid, team in (("blue", "blue"), ("red", "red")):
        sx, sy, syaw = kickoff_spot(team, F, 0.25)
        assert math.hypot(poses[rid][0] - sx, poses[rid][1] - sy) < 0.12 and w.want[rid] == ("stop", "warmup")
    assert w.summary(t)["loops"] == {"blue": 1, "red": 1}


def test_warmup_contains_moves_and_finish_goes_home():
    w = Warmup([BLUE, RED], F, 0.0)
    p = {"kickoff_x_frac": 0.25, "wall_margin_m": 0.28}
    sx = kickoff_spot("blue", F, 0.25)[0]
    poses = {"blue": (sx, 0.0, 0.0), "red": (-sx, 0.0, math.pi)}
    w.update(0.0, robots_world(0.0, poses), {}, p)
    assert w.st["blue"]["step"] == "home"                              # on the spot, but not for home_hold_s yet
    w.update(0.7, robots_world(0.7, poses), {}, p)                     # held: starts trotting
    assert w.st["blue"]["step"] == "trot" and w.want["blue"][0] == "app"
    poses["blue"] = (sx + 0.48, 0.0, 0.0)                              # drifted to the end of its leash
    evs = w.update(0.8, robots_world(0.8, poses), {}, p)
    assert "recentre:blue" in evs and w.st["blue"]["step"] == "home" and w.want["blue"][0] == "goto"
    poses["red"] = (sx + 0.6, 0.3, math.pi)                            # red wanders next to blue's spot
    poses["blue"] = (sx, 0.0, 0.0)
    for t in (0.9, 1.6, 1.7, 2.4, 2.5):
        w.update(t, robots_world(t, poses), {}, p)
    assert w.st["blue"]["step"] == "home"                              # too close to red: never moves off
    w.update(2.6, robots_world(2.6, {"blue": poses["blue"]}), {}, p)
    assert w.want["red"] == ("stop", "stale")                          # not seen: holds
    w.finish()
    poses["red"] = (-sx, 0.0, math.pi)
    t = 2.7
    while t < 30 and "done" not in w.update(t, robots_world(t, poses), {}, p):
        _drive(w, poses, t, 1 / 30)
        t += 1 / 30
    assert all(st["step"] == "done" for st in w.st.values()) and t < 30
