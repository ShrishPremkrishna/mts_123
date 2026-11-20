"""The ball and Pupper state machines, the route descriptions, and the tracker's robustness. No camera."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.ballistics import RollModel, RollTracker, predict_roll  # noqa: E402
from fieldcam.intercept import RobotModel, kick_pose, plan_guard, plan_kick, plan_robot, plan_strike, travel  # noqa: E402
from fieldcam.states import BALL_STATES, ROBOT_STATES, BallMachine, RobotMachine  # noqa: E402
from fieldcam.topdown import render, render_strip  # noqa: E402

L, W, GW = 3.66, 2.44, 0.82
EAST = {"x": L / 2, "y_min": -GW / 2, "y_max": GW / 2}
WEST = {"x": -L / 2, "y_min": -GW / 2, "y_max": GW / 2}


def run_roll(hz=30.0, v0=1.1, decel=0.4, direction=0.2, start=(-0.8, 0.0), noise=0.005, push=0.3,
             hide=None, bend=None, seed=0):
    """A ball that stands 1 s, is pushed up to speed over `push` s, then rolls. hide = (from, to) seconds
    since release with no measurements; bend = (seconds since release, radians). Yields (t, prediction)."""
    rng = np.random.default_rng(seed)
    tracker = RollTracker(RollModel(decel=0.35))
    u = np.array([math.cos(direction), math.sin(direction)])
    stop_t = v0 / decel

    def truth(t):                                   # t = seconds since the push began
        if t <= 0:
            return np.array(start, float)
        if t < push:
            return np.array(start) + u * 0.5 * (v0 / push) * t * t
        p0, tt = np.array(start) + u * 0.5 * v0 * push, min(t - push, stop_t)
        if bend and tt > bend[0]:
            at = p0 + u * (v0 * bend[0] - 0.5 * decel * bend[0] ** 2)
            u2 = np.array([math.cos(direction + bend[1]), math.sin(direction + bend[1])])
            vb, rest = v0 - decel * bend[0], tt - bend[0]
            return at + u2 * (vb * rest - 0.5 * decel * rest * rest)
        return p0 + u * (v0 * tt - 0.5 * decel * tt * tt)

    for k in range(int((1.0 + push + stop_t + 2.0) * hz)):
        t = k / hz
        since = t - 1.0 - push
        if not (hide and hide[0] <= since <= hide[1]):
            tracker.update(truth(t - 1.0) + rng.normal(0, noise, 2), t)
        got = tracker.predict(t, 50, 50, 1)
        yield t, since, (got[0] if got else None)


def test_ball_goes_still_hit_rolling_still():
    machine, seen = BallMachine(), []
    for t, since, pred in run_roll():
        state = machine.update(pred, t)
        assert state["state"] in BALL_STATES
        if not seen or seen[-1] != state["state"]:
            seen.append(state["state"])
        if state["state"] == "ROLLING" and state["moving"]:
            assert "will stop at" in state["going"] and state["speed"] > 0
    assert seen == ["NOT_SEEN", "STILL", "HIT", "ROLLING", "STILL"]


def test_ball_knocked_off_course_is_named_and_recovers():
    machine, seen = BallMachine(), []
    for t, since, pred in run_roll(bend=(1.0, math.radians(25))):
        state = machine.update(pred, t)["state"]
        if not seen or seen[-1] != state:
            seen.append(state)
    assert seen == ["NOT_SEEN", "STILL", "HIT", "ROLLING", "KNOCKED", "ROLLING", "STILL"]


def test_ball_hidden_mid_roll_keeps_its_prediction():
    """A robot walks in front of the ball for 0.8 s: the prediction carries on, marked as not measured."""
    machine, hidden_frames, errors = BallMachine(), 0, []
    stop = np.array([-0.8, 0.0]) + np.array([math.cos(0.2), math.sin(0.2)]) * (0.5 * 1.1 * 0.3 + 1.1 ** 2 / 0.8)
    for t, since, pred in run_roll(hide=(1.0, 1.8)):
        state = machine.update(pred, t)
        if 1.5 <= since <= 1.8:
            assert pred is not None and pred["hidden"] and state["state"] == "HIDDEN"
            hidden_frames += 1
            errors.append(math.hypot(pred["end"][0] - stop[0], pred["end"][1] - stop[1]))
    assert hidden_frames > 6 and max(errors) < 0.25          # no worse than it was when the ball went out of sight
    # ... but a ball that was standing still and vanishes (picked up) is dropped quickly.
    tracker = RollTracker()
    for k in range(20):
        tracker.update((0.3, 0.3), k / 30)
    assert tracker.predict(20 / 30 + 1.0, 50, 50, 1) is not None and tracker.predict(20 / 30 + 1.5, 50, 50, 1) is None


def test_tracker_measures_its_own_noise_and_ignores_garbage():
    rng, tracker = np.random.default_rng(0), RollTracker(RollModel(meas_sigma=0.01))
    for k in range(300):                            # a still ball that really jitters by 2.5 cm
        tracker.update(np.array([0.2, 0.1]) + rng.normal(0, 0.025, 2), k / 30)
        tracker.update((float("nan"), 0.0), k / 30 + 0.001)       # a broken measurement
        got = tracker.predict(k / 30 + 0.002, 50, 50, 1)
    assert tracker.noise == pytest.approx(0.025, abs=0.006)        # learned, not the configured 1 cm
    assert not got[0]["moving"]                                    # and the jitter is not mistaken for motion


def test_route_is_spelled_out_and_does_not_flip_between_frames():
    go = travel((0, 0, 0.3), (2.0, 0.1), 0.0, avoid=(1.0, 0.05))
    assert go["route"] in ("round_left", "round_right") and "the ball is in the way" in go["why"]
    assert go["steps"][0].startswith("get moving") and any(step.startswith("walk") for step in go["steps"])
    assert sum(leg["s"] for leg in go["legs"]) == pytest.approx(go["time_s"])
    assert [c["route"] for c in go["considered"]][0] == go["route"]           # quickest first
    # Almost symmetric: whichever side was chosen last time is kept.
    for side in ("round_left", "round_right"):
        assert travel((0, 0, 0.0), (2.0, 0.0), 0.0, avoid=(1.0, 0.001), prefer=side)["route"] == side
    # ... unless the other side is clearly quicker.
    assert travel((0, 0, 0.3), (2.0, 0.1), 0.0, avoid=(1.0, 0.3), prefer="round_left")["route"] == "round_right"


def test_kick_plan_explains_itself_and_keeps_its_paw():
    m = RobotModel()
    still = predict_roll((0.4, -0.3), (0, 0), L, W, GW)
    plan = plan_kick((-1.0, 0.5, 0.0), still, EAST, L, W, m)
    assert [t["what"] for t in plan["timeline"]] == ["walk to the pose", "settle", "wind-up"]
    assert sum(t["s"] for t in plan["timeline"]) == pytest.approx(plan["contact_in_s"])
    assert plan["limit"] == "robot" and plan["other_paw"]["paw"] != plan["paw"] and plan["steps"] and plan["warnings"] == []
    other = plan["other_paw"]["paw"]
    assert plan_kick((-1.0, 0.5, 0.0), still, EAST, L, W, m, prefer_paw=other)["paw"] == other     # nearly equal: keep
    # A ball against the back line: the pose behind it is off the field, and the plan says so.
    edge = predict_roll((-L / 2 + 0.02, 0.9), (0, 0), L, W, GW)
    assert any("outside the field" in text for text in plan_strike((0, 0, 0), edge, EAST, L, W, m)["warnings"])
    # Waiting where the ball is about to roll: flagged.
    rolling = predict_roll((-1.0, 0.0), (1.0, 0.0), L, W, GW, RollModel(decel=0.4))
    area = {"x": rolling.end[0], "y": rolling.end[1], "along": 0.5, "across": 0.1, "heading": 0.0}
    waiting = plan_kick((0.0, 0.5, 0.0), rolling, EAST, L, W, m, area=area)
    assert not waiting["sure"] and "50 cm" in waiting["unsure_why"]
    assert any("ball's path" in text for text in waiting["warnings"])


def test_guard_stands_between_ball_and_own_goal():
    guard = plan_guard((0.5, 0.8, 0.0), (0.2, 0.6), WEST, RobotModel())
    x, y, yaw = guard["pose"]
    assert x == pytest.approx(-L / 2 + 0.45 * math.cos(yaw)) and abs(yaw - math.atan2(0.6, 0.2 + L / 2)) < 1e-9
    assert guard["steps"] and not guard["at_pose"]


def machines_for(striker, keeper, ball_xy, ball_v, t=0.0, machines=None, seconds=2.0, area=None):
    """Run both robots' machines for `seconds` on a fixed scene; returns their last outputs."""
    traj = predict_roll(ball_xy, ball_v, L, W, GW, RollModel(decel=0.4))
    pred = traj.as_dict(ball_xy, ball_v)
    pred.update(hidden=False, seen_ago_s=0.0, fit={}, area=area)
    m = RobotModel()
    robots = {"striker": (striker, EAST, WEST), "keeper": (keeper, WEST, EAST)}
    machines = machines or {"ball": BallMachine(), "striker": RobotMachine("striker"), "keeper": RobotMachine("keeper")}
    out = None
    for k in range(int(seconds * 20)):
        now = t + k / 20
        ball = machines["ball"].update(pred, now)
        plans = {n: plan_robot({"x": p[0], "y": p[1], "yaw": p[2], "age_s": 0.0}, traj, attack, own, L, W, m, area=area)
                 for n, (p, attack, own) in robots.items() if p is not None}
        out = {n: machines[n].update(now, True, None if p is None else {"visible": True}, ball, plans.get(n),
                                     {o: q for o, q in plans.items() if o != n}, own)
               for n, (p, attack, own) in robots.items()}
    return out, machines


def test_roles_one_robot_attacks_alone_and_defends_its_goal():
    out, _ = machines_for((-1.0, 0.3, 0.0), None, (0.2, 0.1), (0, 0))
    assert out["striker"]["role"] == "OFFENSE" and out["striker"]["state"] == "GO_TO_KICK_POSE"
    assert out["striker"]["state"] in ROBOT_STATES and out["keeper"]["state"] == "HOLD"
    # A ball rolling into the striker's own (west) goal: defend at once, whatever else is going on.
    out, _ = machines_for((-1.2, 0.3, 0.0), None, (0.0, 0.0), (-1.4, 0.05), seconds=0.05)     # on the very first frame
    assert out["striker"]["role"] == "DEFENSE" and out["striker"]["state"] == "BLOCK"


def test_roles_between_two_robots():
    # Ball near the west goal: the keeper (who attacks west) is right there, the striker is far.
    out, _ = machines_for((1.2, 0.5, 0.0), (-0.9, 0.3, math.pi), (-1.2, 0.2), (0, 0))
    assert out["keeper"]["role"] == "OFFENSE" and out["keeper"]["first_to_ball"]
    assert out["striker"]["role"] == "DEFENSE" and out["striker"]["state"] == "GUARD"      # ball is in its own half
    # Same robots, ball in the east half: the striker is behind again, but it is not its half, so it presses.
    out, _ = machines_for((-1.5, 0.5, 0.0), (1.6, 0.2, math.pi), (1.2, 0.2), (0, 0))
    assert out["keeper"]["role"] == "OFFENSE" and out["striker"]["role"] == "OFFENSE"
    assert "press" in out["striker"]["role_why"]
    # Being nearest is not being first: a robot on the wrong side of the ball, facing away, gets there later.
    out, _ = machines_for((-1.5, 0.5, 0.0), (0.6, 0.3, math.pi), (1.2, 0.2), (0, 0))
    assert out["striker"]["first_to_ball"] and out["keeper"]["role"] == "DEFENSE"


def test_decision_and_the_kick_sequence():
    m = RobotModel()
    ball_xy, v = (-0.6, 0.0), (0.56, 0.0)                         # stops at about (-0.2, 0) in 1.4 s
    stop = predict_roll(ball_xy, v, L, W, GW, RollModel(decel=0.4)).end
    pose = kick_pose(tuple(stop), EAST, "right", m)
    big = {"x": stop[0], "y": stop[1], "along": 0.3, "across": 0.05, "heading": 0.0}
    small = dict(big, along=0.03, across=0.03)
    # Far away, stop uncertain: go to the waiting point, decision "wait for stop".
    out, _ = machines_for((-1.5, 0.6, 0.0), None, ball_xy, v, area=big, seconds=1.0)
    assert out["striker"]["state"] == "GO_TO_KICK_POSE" and out["striker"]["decision"] == "wait_for_stop"
    # On the pose, stop known: decision "intercept", and after the settle the wind-up starts, then KICKING.
    out, machines = machines_for(pose, None, ball_xy, v, area=small, seconds=0.15)
    assert out["striker"]["decision"] == "intercept" and out["striker"]["state"] != "WIND_UP"     # still settling
    out, machines = machines_for(pose, None, ball_xy, v, t=0.15, machines=machines, area=small, seconds=0.2)
    assert out["striker"]["state"] in ("WIND_UP", "KICKING")
    out, machines = machines_for(pose, None, ball_xy, v, t=0.4, machines=machines, area=small, seconds=1.0)
    assert out["striker"]["state"] == "KICKING" and "paw reaches the ball" in out["striker"]["why"]
    # The kick lands and the ball is somewhere else: the kick is over and it sets off after the ball again.
    out, machines = machines_for(pose, None, (1.0, 0.5), (0, 0), t=1.4, machines=machines, seconds=2.5)
    assert out["striker"]["state"] == "GO_TO_KICK_POSE"


def test_pipeline_publishes_machines_diagnostics_and_survives_a_planning_fault(monkeypatch):
    from fieldcam import pipeline
    from fieldcam.camera_model import default_intrinsics
    from fieldcam.config import SIM_CONFIG, load_config
    from fieldcam.sim import SimArena

    size = (1920, 1080)
    cfg = load_config(SIM_CONFIG)
    cfg["camera"]["intrinsics"] = "config/_none_.yaml"
    arena = SimArena(cfg, default_intrinsics(*size, 69.0)[0], cam_pos=(0.0, 0.0, 2.6), size=size)
    perc = pipeline.Perception(cfg, *size)
    assert perc.calibrate(arena.render({}, None))
    robots = {"striker": (-0.8, -0.4, 0.3), "keeper": (1.0, 0.1, math.pi)}
    for k in range(12):
        state = perc.process(arena.render(robots, (-0.3, 0.2)), k / 10)
    machines, diag = state["machines"], state["diagnostics"]
    assert machines["ball"]["state"] == "STILL" and set(machines["robots"]) == {"striker", "keeper"}
    assert {m["role"] for m in machines["robots"].values()} <= {"OFFENSE", "DEFENSE"}
    assert diag["fps"] == pytest.approx(10.0, abs=0.5) and diag["problem"] is None
    assert diag["robot_tags_seen"] == ["keeper", "striker"] and diag["floor_tags_seen"] == [0, 1, 2, 3, 4, 5]
    assert diag["camera_height"] == pytest.approx(2.6, abs=0.05) and diag["ball_measured"]
    for detail in (True, False):                                   # both dashboards draw it
        assert render(state, detail=detail).shape[1] == 1280
    assert render_strip(state, detail=True).shape[0] > render_strip(state, detail=False).shape[0]

    def broken(*args, **kwargs):
        raise RuntimeError("planner fell over")
    monkeypatch.setattr(pipeline, "plan_robot", broken)
    hurt = perc.process(arena.render(robots, (-0.3, 0.2)), 1.3)
    assert hurt["ball"]["visible"] and hurt["robots"]["striker"]["visible"]       # tracking carries on
    assert hurt["intercepts"] == {} and "planner fell over" in hurt["diagnostics"]["problem"]
    assert render(hurt).shape[1] == 1280                                           # and the fault is shown, not fatal


def test_published_state_always_fits_one_datagram():
    """The full state outgrew a UDP datagram (macOS refuses more than 9216 bytes). What is sent is a
    compact copy; the core fields other code relies on must come through untouched."""
    import json
    from fieldcam import pipeline
    from fieldcam.camera_model import default_intrinsics
    from fieldcam.config import SIM_CONFIG, load_config
    from fieldcam.sim import SimArena
    from fieldcam.wire import LIMIT, wire_state

    size = (1920, 1080)
    cfg = load_config(SIM_CONFIG)
    cfg["camera"]["intrinsics"] = "config/_none_.yaml"
    arena = SimArena(cfg, default_intrinsics(*size, 69.0)[0], cam_pos=(0.0, -0.2, 2.6), size=size)
    perc = pipeline.Perception(cfg, *size)
    assert perc.calibrate(arena.render({}, None))
    robots = {"striker": (-1.0, -0.55, 0.5), "keeper": (1.0, 0.5, math.pi)}
    biggest, full_sizes = None, []
    for k in range(60):                              # a slow roll: long path, long trail, two robots planning
        t = k / 20
        tt = max(t - 0.5, 0.0)
        ball = (-0.9 + 0.9 * tt - 0.5 * 0.15 * tt * tt, 0.45 - 0.2 * tt)
        state = perc.process(arena.render(robots, ball), t)
        full_sizes.append(len(json.dumps(state, separators=(",", ":"))))
        if biggest is None or full_sizes[-1] >= max(full_sizes):
            biggest = state
    assert max(full_sizes) > 9216                    # the problem is real: the full state does not fit
    wire = wire_state(biggest)
    assert len(json.dumps(wire, separators=(",", ":"))) <= LIMIT
    for key in ("seq", "stamp", "captured_at", "frame_age_s", "calibration_valid", "ball", "robots", "goals", "relative"):
        assert wire[key] == biggest[key]             # exactly, not rounded
    assert wire["machines"]["ball"]["state"] == biggest["machines"]["ball"]["state"]
    assert wire["prediction"]["end"] == pytest.approx(biggest["prediction"]["end"], abs=1e-3)
    assert render(json.loads(json.dumps(wire))).shape[1] == 1280          # the stand-alone dashboard still draws it
    # Squeezed much harder, it still fits and still says what the ball and robots are doing.
    tiny = wire_state(biggest, limit=6500)             # (the untouched core fields alone are about 5 KB)
    assert len(json.dumps(tiny, separators=(",", ":"))) <= 6500 and tiny["_wire"]["trimmed"] >= 1
    assert tiny["machines"] and tiny["ball"] == biggest["ball"]
    assert "steps" in biggest["intercepts"]["keeper"]["kick"]             # and the input was left alone
