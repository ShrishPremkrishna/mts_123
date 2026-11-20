"""Ball roll prediction and where the robot should go because of it. No camera."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.ballistics import (RollModel, RollTracker, fit_decel, predict_roll, roll_distance,  # noqa: E402
                                 time_to_roll)
from fieldcam.config import SIM_CONFIG, load_config  # noqa: E402
from fieldcam.fakeworld import FakeWorld  # noqa: E402
from fieldcam.intercept import RobotModel, plan_block, plan_strike, strike_pose, travel  # noqa: E402
from fieldcam.topdown import render  # noqa: E402

L, W, GW = 3.66, 2.44, 0.82
EAST = {"x": L / 2, "y_min": -GW / 2, "y_max": GW / 2}
WEST = {"x": -L / 2, "y_min": -GW / 2, "y_max": GW / 2}


def roll(pos, speed, direction, **model):
    v = (speed * math.cos(direction), speed * math.sin(direction))
    return predict_roll(pos, v, L, W, GW, RollModel(**model), radius=0.08)


# ---- the roll model -------------------------------------------------------------------

def test_roll_arithmetic():
    assert roll_distance(1.0, 99.0, 0.5) == pytest.approx(1.0)            # v^2 / 2a
    assert roll_distance(1.0, 1.0, 0.5) == pytest.approx(0.75)
    assert time_to_roll(1.0, 0.75, 0.5) == pytest.approx(1.0)
    assert time_to_roll(1.0, 1.01, 0.5) is None                           # stops first


def test_still_ball_has_no_path():
    t = roll((0.3, 0.2), 0.05, 0.0)
    assert not t.moving and t.ends == "still" and t.samples() == []
    assert t.at(3.0) == (0.3, 0.2, 0.0)


def test_free_roll_stops_where_the_formula_says():
    t = roll((-1.0, -0.5), 1.0, 0.4, decel=0.35)
    d = 1.0 / (2 * 0.35)
    assert t.ends == "stops" and t.end_in_s == pytest.approx(1.0 / 0.35)
    assert t.end == pytest.approx([-1.0 + d * math.cos(0.4), -0.5 + d * math.sin(0.4)])
    x, y, speed = t.at(1.0)
    assert speed == pytest.approx(0.65) and math.hypot(x + 1.0, y + 0.5) == pytest.approx(0.825)
    path = t.samples()
    assert path[0][0] == 0.0 and path[-1][0] == pytest.approx(t.end_in_s, abs=1e-3)


def test_goal_out_and_bounce():
    assert roll((1.0, 0.1), 1.2, 0.0).ends == "goal_east"
    assert roll((-1.0, 0.1), 1.2, math.pi).ends == "goal_west"
    wide = roll((1.0, 0.9), 1.2, 0.0)                                      # crosses the end line outside the posts
    assert wide.ends == "out" and wide.end[0] == pytest.approx(L / 2)
    side = roll((0.0, 0.8), 1.2, math.pi / 2)
    assert side.ends == "out" and side.end[1] == pytest.approx(W / 2)
    back = roll((0.0, 0.8), 1.2, math.pi / 2, walls=True)                  # same roll with boards
    assert back.ends == "stops" and back.bounces == 1 and back.end[1] < W / 2 - 0.08
    assert roll((1.0, 0.1), 0.5, 0.0).ends == "stops"                      # too slow to reach the goal


@pytest.mark.parametrize("speed,direction", [(1.4, 0.3), (1.0, 2.0), (1.5, -1.2), (0.8, 3.0), (1.3, 1.5)])
def test_prediction_matches_the_fake_field(speed, direction):
    """The fake field steps the same rules in small time steps; the closed-form path must agree with it."""
    world = FakeWorld(L, W, GW, 0.08, robot=(-1.7, -1.1, 0.0), ball=(0.2, -0.1))
    world.roll_ball(speed, direction)
    t = predict_roll(world.ball, world.ball_vel, L, W, GW,
                     RollModel(decel=world.ball_friction, walls=True, restitution=0.4), world.ball_radius)
    checks = [0.5 * t.end_in_s, t.end_in_s] if t.ends == "stops" else [0.5 * t.end_in_s]
    clock = 0.0
    for when in checks:
        while clock < when:
            world.step(None, 1 / 240)
            clock += 1 / 240
        x, y, _ = t.at(clock)
        assert math.hypot(x - world.ball[0], y - world.ball[1]) < 0.03
    if t.ends.startswith("goal"):
        while not world.event and clock < t.end_in_s + 0.5:
            world.step(None, 1 / 240)
            clock += 1 / 240
        assert world.event.startswith("GOAL") and clock == pytest.approx(t.end_in_s, abs=0.05)


# ---- the velocity estimate --------------------------------------------------------------

def kicked_ball(t, kick_at=1.0, v0=1.0, direction=0.4, decel=0.35):
    tau = min(max(t - kick_at, 0.0), v0 / decel)
    d = v0 * tau - 0.5 * decel * tau * tau
    speed = max(v0 - decel * (t - kick_at), 0.0) if t >= kick_at else 0.0
    return np.array([-1.0 + d * math.cos(direction), -0.5 + d * math.sin(direction)]), speed


@pytest.mark.parametrize("hz", [7.5, 30.0])
def test_tracker_follows_a_kick_and_predicts_the_stop(hz):
    """1 cm measurement noise, at the frame rate seen on the mounted phone (7.5/s) and at 30/s."""
    stop = kicked_ball(99.0)[0]
    errors, false_moving, still_frames, noticed = [], 0, 0, []
    for seed in range(40):
        rng = np.random.default_rng(seed)
        tracker, first = RollTracker(RollModel(decel=0.35)), None
        for k in range(int(5.5 * hz)):
            t = k / hz
            truth, speed = kicked_ball(t)
            tracker.update(truth + rng.normal(0, 0.01, 2), t)
            e = tracker.estimate(t)
            if e is None:
                continue
            if t < 0.99 or t > 1.0 + 1.0 / 0.35 + 0.8:
                still_frames += 1
                false_moving += e["moving"]
            if first is None and t >= 1.0 and e["moving"]:
                first = t - 1.0
            if abs(t - 2.0) < 0.5 / hz:                                    # one second after the kick
                end = predict_roll((e["x"], e["y"]), (e["vx"], e["vy"]), 20, 20, 1, RollModel(decel=0.35)).end
                errors.append(float(np.linalg.norm(end - stop)))
        noticed.append(first if first is not None else 9.0)
    assert np.median(errors) < 0.08 and np.percentile(errors, 90) < 0.15   # where it stops, known 1.9 s ahead
    assert false_moving / still_frames < 0.01                              # a still ball stays still
    assert np.median(noticed) < 0.4                                        # the kick is noticed quickly


def test_tracker_ignores_one_stray_detection():
    tracker = RollTracker()
    for k in range(10):
        tracker.update((0.5, 0.5), k / 10)
    tracker.update((1.5, -0.4), 1.0)                                       # something pink that is not the ball
    tracker.update((0.5, 0.5), 1.1)
    e = tracker.estimate(1.1)
    assert not e["moving"] and math.hypot(e["x"] - 0.5, e["y"] - 0.5) < 0.01


def test_fit_decel_recovers_the_deceleration():
    rng = np.random.default_rng(1)
    samples = []
    for k in range(18):                                                    # 2.4 s of a roll at 7.5 frames/s
        t = 1.0 + k / 7.5
        samples.append((t, *(kicked_ball(t)[0] + rng.normal(0, 0.01, 2))))
    fit = fit_decel(samples)
    assert fit["decel"] == pytest.approx(0.35, abs=0.04) and fit["v0"] == pytest.approx(1.0, abs=0.05)
    assert fit_decel(samples[:3]) is None                                  # too short to say anything


# ---- where the robot should go ----------------------------------------------------------

def test_travel_never_plans_a_backward_shuffle_and_goes_round_the_ball():
    m = RobotModel()
    ahead = travel((0, 0, 0), (0.4, 0.1), None, m=m)
    behind = travel((0, 0, 0), (-0.4, 0.1), None, m=m)
    assert ahead["how"] == "shuffle" and behind["how"] == "walk"
    assert behind["time_s"] > ahead["time_s"] + 2.0                        # it has to turn round first
    direct = travel((0, 0, 0), (2.0, 0.0), None, m=m)
    round_ball = travel((0, 0, 0), (2.0, 0.0), None, avoid=(1.0, 0.05), m=m)
    assert len(round_ball["path"]) == 3 and round_ball["time_s"] > direct["time_s"]
    assert min(math.hypot(1.0 - x, 0.05 - y) for x, y in round_ball["path"]) >= m.clearance


def test_strike_pose_is_behind_the_ball_facing_the_goal():
    x, y, yaw = strike_pose((0.5, 0.0), EAST, RobotModel(standoff=0.6))
    assert (x, y, yaw) == pytest.approx((-0.1, 0.0, 0.0))


def test_strike_lines_up_where_the_ball_will_stop():
    t = roll((-1.0, -0.6), 1.2, 0.5, decel=0.35)
    plan = plan_strike((-1.5, 0.8, 0.0), t, EAST, L, W, RobotModel())
    assert plan["feasible"] and plan["kind"] == "at_rest"
    assert plan["ball"] == pytest.approx(list(t.end))                      # not where the ball is now
    assert plan["contact_in_s"] >= max(plan["travel_s"], t.end_in_s) - 1e-9
    still = plan_strike((-1.5, 0.8, 0.0), roll((0.2, 0.1), 0.0, 0.0), EAST, L, W, RobotModel())
    assert still["feasible"] and still["ball"] == pytest.approx([0.2, 0.1]) and still["wait_s"] == 0.0


def test_strike_meets_a_slow_ball_that_comes_to_the_robot():
    """A ball crawling toward a robot that is already nearly lined up: meet it before it stops."""
    t = roll((-0.5, 0.0), 0.3, 0.0, decel=0.05, min_speed=0.05)            # rolls 0.9 m over 6 s
    plan = plan_strike((-0.9, 0.0, 0.0), t, EAST, L, W, RobotModel(max_contact_speed=0.2))
    assert plan["feasible"] and plan["kind"] == "moving" and plan["contact_in_s"] < t.end_in_s


def test_no_strike_plan_when_the_ball_is_leaving():
    plan = plan_strike((0, 0, 0), roll((1.0, 0.9), 1.2, 0.0), EAST, L, W, RobotModel())
    assert not plan["feasible"] and "out" in plan["note"]


def test_block_only_when_own_goal_is_threatened():
    shot = roll((0.6, -0.5), 1.5, math.atan2(0.65, -2.43), decel=0.35)     # at the west goal
    assert shot.ends == "goal_west"
    assert plan_block((-1.45, 0.25, 0.0), shot, EAST, RobotModel()) is None    # not this robot's goal
    near = plan_block((-1.45, 0.05, 0.0), shot, WEST, RobotModel())
    far = plan_block((1.0, 1.0, 0.0), shot, WEST, RobotModel())
    assert near["feasible"] and near["spare_s"] > 0.3
    assert not far["feasible"] and far["spare_s"] < 0
    # The block point puts the body across the ball's path.
    x, y, _ = shot.at(near["ball_there_in_s"])
    assert math.hypot(near["point"][0] - x, near["point"][1] - y) <= RobotModel().block_cover + 1e-9
    assert plan_block((0, 0, 0), roll((0.6, -0.5), 0.6, math.pi), WEST, RobotModel()) is None   # stops short


def test_the_shipped_config_loads_into_both_models():
    """Every key in the live config's prediction and intercept sections must be one the code knows."""
    cfg = load_config()
    assert set(cfg["prediction"]) >= {"roll_decel", "min_speed", "walls"} and "walk_speed" in cfg["intercept"]
    roll_model, robot = RollModel.from_config(cfg), RobotModel.from_config(cfg)
    assert roll_model.decel == cfg["prediction"]["roll_decel"] and roll_model.walls is False
    assert robot.walk_speed == cfg["intercept"]["walk_speed"] and robot.can_reverse is False
    with pytest.raises(ValueError):
        RollModel.from_config({"prediction": {"rol_decel": 0.3}})          # a typo is an error, not ignored
    with pytest.raises(ValueError):
        RobotModel.from_config({"intercept": {"walkspeed": 0.3}})


# ---- in the pipeline and on the dashboard -------------------------------------------------

def test_pipeline_publishes_prediction_and_intercepts():
    from fieldcam.camera_model import default_intrinsics
    from fieldcam.pipeline import Perception
    from fieldcam.sim import SimArena

    size = (1920, 1080)
    cfg = load_config(SIM_CONFIG)
    cfg["camera"]["intrinsics"] = "config/_none_.yaml"
    cfg["camera"]["hfov_deg"] = 69.0
    arena = SimArena(cfg, default_intrinsics(*size, 69.0)[0], cam_pos=(0.0, 0.0, 2.6), size=size)
    perc = Perception(cfg, *size)
    assert perc.calibrate(arena.render({}, None))
    robots = {"striker": (-0.8, -0.4, 0.3), "keeper": (1.0, 0.1, math.pi)}
    state = None
    for k in range(12):                                                    # the ball rolls at 0.6 m/s along +y
        t = k / 10
        state = perc.process(arena.render(robots, (-0.3, -0.5 + 0.6 * t)), t)
        if k == 0:
            assert state["prediction"] is None or not state["prediction"]["moving"]
    pred = state["prediction"]
    assert pred["moving"] and pred["speed"] == pytest.approx(0.6, abs=0.1)
    assert abs(math.degrees(pred["heading"]) - 90) < 10 and pred["ends"] in ("stops", "out")
    assert state["intercepts"]["striker"]["strike"]["feasible"] == (pred["ends"] == "stops")
    perc.invalidate_calibration("test")
    held = perc.process(arena.render(robots, (-0.3, 0.3)), 1.3)
    assert held["prediction"] is None and held["intercepts"] == {}         # nothing predicted on a withheld calibration
    assert render(state).shape == (800, 1280, 3)                           # and the dashboard draws it


# ---- every roll fitted on its own, with a stopping area and a trail ------------------------

def simulate_roll(tracker, decel, v0=1.2, direction=0.4, hz=30.0, noise=0.01, seed=0, bend=None, start=(0.0, 0.0), t0=0.0):
    """A ball standing for 1 s, then rolling and slowing at `decel`; bend = (fraction of the roll, radians)
    knocks it onto a new direction part-way. Returns [(seconds since release, prediction dict, true stop)]."""
    rng = np.random.default_rng(seed)
    u = np.array([math.cos(direction), math.sin(direction)])
    duration = v0 / decel
    t_bend = bend[0] * duration if bend else None

    def truth(t):
        if t <= 0:
            return np.array(start, float)
        t = min(t, duration)
        if t_bend is None or t <= t_bend:
            return np.array(start) + u * (v0 * t - 0.5 * decel * t * t)
        at_bend = np.array(start) + u * (v0 * t_bend - 0.5 * decel * t_bend ** 2)
        u2 = np.array([math.cos(direction + bend[1]), math.sin(direction + bend[1])])
        vb, tt = v0 - decel * t_bend, t - t_bend
        return at_bend + u2 * (vb * tt - 0.5 * decel * tt * tt)

    out, stop = [], truth(1e9)
    for k in range(int((duration + 2.5) * hz)):
        since = k / hz - 1.0
        tracker.update(truth(since) + rng.normal(0, noise, 2), t0 + k / hz)
        got = tracker.predict(t0 + k / hz, 50, 50, 1)
        if got is not None:
            out.append((since, got[0], stop))
    return out


def miss(pred, stop):
    return math.hypot(pred["end"][0] - stop[0], pred["end"][1] - stop[1])


@pytest.mark.parametrize("decel", [0.2, 0.6])
def test_each_roll_is_fitted_on_its_own(decel):
    """The config says 0.35; this ball slows at 0.2 or 0.6. A fixed rate would stay wrong all the way."""
    late, final_decel = [], []
    for seed in range(12):
        frames = simulate_roll(RollTracker(RollModel(decel=0.35)), decel, seed=seed)
        duration = 1.2 / decel
        moving = [(s, p, stop) for s, p, stop in frames if p["moving"] and 0 < s < duration]
        late += [miss(p, stop) for s, p, stop in moving if s > 0.7 * duration]
        final_decel.append(moving[-1][1]["decel"])
    wrong = abs(1.2 ** 2 / (2 * 0.35) - 1.2 ** 2 / (2 * decel)) * 0.3 ** 2    # the fixed rate's error with 30% of the time left
    assert np.median(late) < 0.03 < wrong
    assert np.median(final_decel) == pytest.approx(decel, abs=0.06)


def test_stopping_area_shrinks_and_contains_the_stop():
    inside, first, last = [], [], []
    for seed in range(15):
        frames = simulate_roll(RollTracker(RollModel(decel=0.35)), 0.3 + 0.02 * seed, seed=seed)
        areas = [(p["area"], p, stop) for s, p, stop in frames if p["moving"] and p["area"]]
        for area, p, stop in areas:
            c, s_ = math.cos(area["heading"]), math.sin(area["heading"])
            dx, dy = p["end"][0] - stop[0], p["end"][1] - stop[1]
            inside.append(((c * dx + s_ * dy) / area["along"]) ** 2 + ((-s_ * dx + c * dy) / area["across"]) ** 2 <= 1.0)
        first.append(max(areas[2][0]["along"], areas[2][0]["across"]))
        last.append(max(areas[-1][0]["along"], areas[-1][0]["across"]))
    assert np.mean(inside) > 0.8                    # it is an honest area
    assert np.median(first) > 0.25 and np.median(last) < 0.05   # a general area at first, a spot at the end


def test_trail_during_the_roll_and_summary_after():
    frames = simulate_roll(RollTracker(RollModel(decel=0.35)), 0.4, seed=3)
    moving = [p for s, p, _ in frames if p["moving"] and s > 0.4]            # once the kick has been recognised
    lengths = [len(p["trail"]) for p in moving[:10]]
    assert lengths == list(range(lengths[0], lengths[0] + 10))                # one more prediction each frame
    assert len(moving[-1]["trail"]) > 20 and moving[-1]["trail"][-1][0] == 0.0                    # newest last, age 0
    after = [p for s, p, _ in frames if not p["moving"] and p["last_roll"]]
    assert after, "no summary after the roll"
    done = after[-1]["last_roll"]
    stop = frames[-1][2]
    assert math.hypot(done["stopped"][0] - stop[0], done["stopped"][1] - stop[1]) < 0.02
    assert done["first_guess_error"] == pytest.approx(math.hypot(done["trail"][0][1] - done["stopped"][0],
                                                                 done["trail"][0][2] - done["stopped"][1]), abs=0.002)
    assert done["decel"] == pytest.approx(0.4, abs=0.05)


@pytest.mark.parametrize("hz", [7.5, 30.0])
def test_ball_knocked_off_course_mid_roll(hz):
    """A divot turns the ball 20 degrees a third of the way through. The prediction has to follow it."""
    late, kept_trail = [], []
    for seed in range(12):
        turn = math.radians(20) * (1 if seed % 2 else -1)
        frames = simulate_roll(RollTracker(RollModel(decel=0.35)), 0.4, hz=hz, seed=seed, bend=(0.35, turn))
        moving = [(s, p, stop) for s, p, stop in frames if p["moving"] and 0 < s < 3.0]
        late += [miss(p, stop) for s, p, stop in moving if s > 0.75 * 3.0]
        kept_trail.append(len(moving[-1][1]["trail"]) > 0.5 * len(moving))     # one roll, one trail: not started again
    assert np.median(late) < 0.05 and np.percentile(late, 90) < 0.15
    assert all(kept_trail)


def test_off_course_rolls_widen_the_area_for_the_next_ones():
    tracker, t0 = RollTracker(RollModel(decel=0.35)), 0.0
    straight = simulate_roll(tracker, 0.4, seed=1, t0=t0)
    width_before = np.median([p["area"]["across"] for s, p, _ in straight if p["moving"] and p["area"] and 0.3 < s < 1.0])
    for k in range(4):                              # then four rolls that each get knocked off line
        t0 += 20.0
        simulate_roll(tracker, 0.4, seed=10 + k, bend=(0.4, math.radians(20)), t0=t0)
    after = simulate_roll(tracker, 0.4, seed=2, t0=t0 + 20.0)
    width_after = np.median([p["area"]["across"] for s, p, _ in after if p["moving"] and p["area"] and 0.3 < s < 1.0])
    assert max(tracker.turns) > math.radians(12) and width_after > 1.5 * width_before


def test_slow_down_is_learned_from_finished_rolls():
    tracker = RollTracker(RollModel(decel=0.35))
    for k in range(5):
        simulate_roll(tracker, 0.55, seed=k, t0=20.0 * k)
    guess, spread = tracker.decel_prior()
    assert guess == pytest.approx(0.55, abs=0.05) and spread <= 0.2


# ---- lining up while it rolls, and the leg kick ----------------------------------------------

def test_strike_waits_further_back_while_the_stop_is_uncertain():
    t = roll((-1.0, -0.6), 1.2, 0.5, decel=0.35)
    sure = plan_strike((-1.5, 0.8, 0.0), t, EAST, L, W, RobotModel())
    area = {"x": t.end[0], "y": t.end[1], "along": 0.4, "across": 0.1, "heading": 0.5}
    unsure = plan_strike((-1.5, 0.8, 0.0), t, EAST, L, W, RobotModel(), area=area)
    back = math.hypot(unsure["pose"][0] - unsure["ball"][0], unsure["pose"][1] - unsure["ball"][1])
    assert back == pytest.approx(RobotModel().standoff + 0.4) and unsure["hold"] and sure["hold"]
    still = plan_strike((-1.5, 0.8, 0.0), roll((0.2, 0.1), 0.0, 0.0), EAST, L, W, RobotModel(), area=None)
    assert not still["hold"]


def test_kick_pose_puts_the_ball_in_front_of_the_paw():
    from fieldcam.intercept import kick_pose
    from fieldcam.relative import body_point
    m = RobotModel()
    for paw, left in (("left", m.kick_left_paw), ("right", m.kick_right_paw)):
        x, y, yaw = kick_pose((0.4, -0.3), EAST, paw, m)
        seen = body_point({"x": x, "y": y, "yaw": yaw}, (0.4, -0.3))
        assert (seen["x"], seen["y"]) == pytest.approx((m.kick_forward, left), abs=1e-9)
        assert yaw == pytest.approx(math.atan2(0.0 + 0.3, L / 2 + m.aim_depth - 0.4))    # swing aimed through the goal


def test_kick_on_a_still_ball_is_walk_settle_wind_up():
    from fieldcam.intercept import kick_pose, plan_kick
    m = RobotModel()
    still = roll((0.4, -0.3), 0.0, 0.0)
    plan = plan_kick((-1.0, 0.5, 0.0), still, EAST, L, W, m)
    assert plan["feasible"] and plan["action"] == "go_to_pose" and plan["sure"] and plan["stage"] == plan["pose"]
    assert plan["contact_in_s"] == pytest.approx(plan["travel_s"] + m.kick_settle_s + m.kick_windup_s)
    assert plan["commit_in_s"] == pytest.approx(plan["contact_in_s"] - m.kick_windup_s) and plan["saves_s"] == 0.0
    # Standing on the pose already: wind up now.
    pose = kick_pose((0.4, -0.3), EAST, plan["paw"], m)
    there = plan_kick(pose, still, EAST, L, W, RobotModel(react_s=0.0), )
    assert there["at_pose"] and there["action"] == "wind_up"
    assert not plan_kick((0, 0, 0), roll((1.0, 0.9), 1.2, 0.0), EAST, L, W, m)["feasible"]     # rolling out


def test_kick_wind_up_starts_before_the_ball_has_stopped():
    """Robot already at the pose where the ball will stop; the ball is still 1.5 s from stopping. The wind-up
    takes 2.3 s, so it should start now, not after the ball has stopped: that is the time prediction saves."""
    from fieldcam.intercept import kick_pose, plan_kick
    m = RobotModel(react_s=0.0)
    t = roll((-0.6, 0.0), 0.525, 0.0, decel=0.35, min_speed=0.05)          # stops in 1.5 s
    assert t.end_in_s == pytest.approx(1.5)
    pose = kick_pose(tuple(t.end), EAST, "right", m)
    small = {"x": t.end[0], "y": t.end[1], "along": 0.03, "across": 0.03, "heading": 0.0}
    plan = plan_kick(pose, t, EAST, L, W, m, area=small)
    assert plan["sure"] and plan["at_pose"] and plan["action"] == "wind_up"
    assert plan["contact_in_s"] == pytest.approx(m.kick_settle_s + m.kick_windup_s) and plan["contact_in_s"] > t.end_in_s
    assert plan["saves_s"] == pytest.approx(t.end_in_s, abs=0.05)
    assert plan["ball_speed_at_contact"] <= m.kick_max_ball_speed
    # Same moment, but the stopping point is still only known to 30 cm: wait clear of it instead.
    big = dict(small, along=0.30)
    unsure = plan_kick(pose, t, EAST, L, W, m, area=big)
    assert not unsure["sure"] and unsure["action"] != "wind_up"
    assert math.hypot(unsure["stage"][0] - unsure["pose"][0], unsure["stage"][1] - unsure["pose"][1]) == pytest.approx(0.30)
