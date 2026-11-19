"""The approach logic (get behind the ball, converge onto the shot line going forward, run through
the ball), driven only by a robot-relative ball and goal, on the fake field."""
import json
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.approach import ApproachController, Params, plan  # noqa: E402
from fieldcam.fakeworld import FakeWorld  # noqa: E402
from fieldcam.filters import wrap  # noqa: E402
from fieldcam.topdown import render  # noqa: E402


# ---- geometry -------------------------------------------------------------------
def test_plan_geometry_straight_ahead():
    """Ball 2 m ahead, goal 4 m ahead: everything lies on the robot's own axis."""
    p = plan((2.0, 0.0), (4.0, 0.0))
    assert p.line_up == pytest.approx((1.4, 0.0))            # run_up 0.60 behind the ball
    assert p.entry == pytest.approx((1.05, 0.0))             # 0.35 further back still
    assert p.shot_heading == pytest.approx(0.0) and p.left_of_line == pytest.approx(0.0)
    assert p.behind == pytest.approx(2.0) and not p.detour
    assert p.run_end == pytest.approx((2.0, 0.0))            # contact 0.25 short of the ball, 0.25 follow-through
    assert plan((1.0, 0.0), (1.0, 0.0)) is None              # ball on the aim point: no line to follow


def test_plan_signs():
    """Goal off to the left of the ball: the shot heading is a left turn; a robot standing to the
    left of the shot line (looking at the goal) is told so, with the right sign."""
    p = plan((1.0, 0.0), (1.0, 2.0))                         # shot line runs straight "left" (+y)
    assert p.shot_heading == pytest.approx(math.pi / 2)
    assert p.line_up == pytest.approx((1.0, -0.6))
    assert p.left_of_line == pytest.approx(1.0)              # robot is 1 m to the left of that line
    assert p.behind == pytest.approx(0.0)                    # level with the ball along it
    q = plan((1.0, 0.3), (5.0, 0.3))                         # line parallel to the robot's axis, 0.3 m to its left
    assert q.left_of_line == pytest.approx(-0.3)             # so the robot is RIGHT of the line
    assert any("shuffle 30 cm left onto the shot line" in s for s in q.steps)


def test_plan_goes_round_a_ball_in_the_way():
    """Robot on the goal side: the straight path to the entry point runs through the ball, so it detours."""
    p = plan((1.0, 0.05), (-2.0, 0.05))
    assert p.detour and len(p.path) == 3 and p.behind < 0
    way = p.path[1]
    assert math.hypot(way[0] - 1.0, way[1] - 0.05) == pytest.approx(0.35 * 1.3)
    assert way[1] < 0.05                                     # round the side the robot is nearer to
    assert any("go round the ball" in s for s in p.steps)
    for a, b in zip(p.path, p.path[1:]):                     # neither leg clips the ball
        seg = np.array(b) - np.array(a)
        t = np.clip((np.array([1.0, 0.05]) - a) @ seg / (seg @ seg), 0, 1)
        assert np.linalg.norm(np.array(a) + t * seg - (1.0, 0.05)) > 0.30


def test_aim_point_behind_the_goal_line_is_steady_near_the_goal():
    """Ball 10 cm in front of the goal centre: a 1 cm sideways wobble must not swing the shot heading."""
    a = plan((1.0, 0.00), (1.1, 0.0, 0.0)).shot_heading
    b = plan((1.0, 0.01), (1.1, 0.0, 0.0)).shot_heading
    assert abs(a - b) < math.radians(2)
    centre_only = abs(plan((1.0, 0.00), (1.1, 0.0)).shot_heading - plan((1.0, 0.01), (1.1, 0.0)).shot_heading)
    assert centre_only > math.radians(5)                     # aiming at the centre itself really is that touchy


# ---- closed loop on the fake field ------------------------------------------------
def aim_direction(world):
    """Field direction the ball should be sent in: from the ball to the point behind the goal centre."""
    gx = world.goals[world.attacks]["x"]
    return math.atan2(-world.ball[1], gx + math.copysign(0.4, gx) - world.ball[0])


def run_fake(world, seconds, delay_s=0.0, params=None):
    """Drive the fake field with the controller, which sees positions `delay_s` late.
    Returns per-tick rows and, for each run, the TRUE state when it started and when it hit the ball."""
    ctrl, dt, late = ApproachController(params or Params()), 1 / 30, deque()
    ticks, runs, prev, kicks, current = [], [], None, 0, None
    while world.t < seconds:
        late.append(world.state())
        seen = late[0] if len(late) <= round(delay_s / dt) else late.popleft()
        rel = seen["relative"]["striker"]
        ball = (rel["ball"]["x"], rel["ball"]["y"]) if "ball" in rel else None
        out = ctrl.tick(ball, (rel["goal"]["x"], rel["goal"]["y"], rel["goal"]["facing"]), world.t)  # two relative points in
        aim = aim_direction(world)
        offset = world.robot[:2] - world.ball
        behind = -(offset[0] * math.cos(aim) + offset[1] * math.sin(aim))
        left = -offset[0] * math.sin(aim) + offset[1] * math.cos(aim)
        if out["phase"] == "RUN" and prev != "RUN":
            current = {"start_behind": behind, "start_left": abs(left), "start_yaw": abs(wrap(world.robot[2] - aim))}
        prev = out["phase"]
        ticks.append((world.t, out["phase"], float(np.linalg.norm(offset)), out["cmd"]))
        world.step(out["cmd"], dt)
        if world.kicks > kicks and current is not None:
            c, s = math.cos(world.robot[2]), math.sin(world.robot[2])
            d = world.ball - world.robot[:2]
            current.update(hit_yaw=abs(wrap(world.robot[2] - aim)), hit_offcentre=abs(-s * d[0] + c * d[1]))
            runs.append(current)
            current = None
        kicks = world.kicks
    return ticks, runs


@pytest.mark.parametrize("robot,ball,attacks", [
    ((-1.0, -0.5, 0.3), (0.3, 0.2), "east"),      # behind the ball, off to one side
    ((1.2, 0.3, 2.8), (0.0, 0.0), "east"),        # on the goal side: has to go round
    ((0.2, 0.9, -1.5), (-0.4, -0.3), "west"),     # attacking the other way
    ((-0.5, 0.0, 0.0), (0.4, 0.0), "east"),       # already on the line
])
def test_gets_on_the_line_then_runs_through_the_ball(robot, ball, attacks):
    p = Params()
    world = FakeWorld(robot=robot, ball=ball, attacks=attacks)
    ticks, runs = run_fake(world, 45.0)
    phases = [row[1] for row in ticks]
    first_run = phases.index("RUN")
    r = runs[0]
    # When the run started it was on the shot line, facing along it, somewhere in the allowed window.
    assert p.run_min - 0.03 <= r["start_behind"] <= p.run_max + 0.03
    assert r["start_left"] < p.lat_tol + 0.01 and r["start_yaw"] < p.yaw_tol + math.radians(1)
    # While getting there it kept clear of the ball, and the first contact came during the run.
    if first_run:
        assert min(row[2] for row in ticks[:first_run]) > p.ball_clearance - 0.08
    assert r["hit_yaw"] < math.radians(4) and r["hit_offcentre"] < 0.02
    # It never asked the robot to walk backwards.
    assert min(row[3]["vx"] for row in ticks) >= 0.0
    print(f"\\n[{attacks}, robot {robot}] first run from {r['start_behind']:.2f} m behind, {r['start_left'] * 100:.1f} cm off line, "
          f"{math.degrees(r['start_yaw']):.1f} deg off; {world.kicks} hits, {world.goals_scored} goals in 45 s")
    assert world.goals_scored >= 1


def summarise(runs):
    med = lambda k, scale: float(np.median([r[k] for r in runs]) * scale)  # noqa: E731
    p90 = lambda k, scale: float(np.percentile([r[k] for r in runs], 90) * scale)  # noqa: E731
    return med, p90


def test_sloppy_robot_with_delay_still_lines_up():
    """The robot the team actually has to work with, as far as we know: 80% of the commanded speed,
    veers left, creeps forward while strafing, cannot reverse; and the planner sees positions 0.15 s
    late with 1 cm of jitter. It must still get onto the line, hit the ball squarely, and never stall."""
    all_runs, goals, no_hit = [], 0, 0
    for seed in range(1, 9):
        world = FakeWorld(seed=seed, noise=0.01, sloppy=True)
        world.random_scene()
        ticks, runs = run_fake(world, 60.0, delay_s=0.15)
        assert min(row[3]["vx"] for row in ticks) >= 0.0
        all_runs += runs
        goals += world.goals_scored
        no_hit += world.kicks == 0
    med, p90 = summarise(all_runs)
    deg = math.degrees(1)
    print(f"\\n[sloppy robot, 0.15 s late, 8 scenes x 60 s] {len(all_runs)} runs, {goals} goals, scenes with no hit: {no_hit}; "
          f"at contact: heading {med('hit_yaw', deg):.1f} deg (90th pct {p90('hit_yaw', deg):.1f}), "
          f"ball off-centre {med('hit_offcentre', 100):.1f} cm (90th pct {p90('hit_offcentre', 100):.1f})")
    assert no_hit == 0 and len(all_runs) >= 30
    assert p90("hit_yaw", deg) < 5.0 and p90("hit_offcentre", 100) < 3.0
    assert goals >= 4


def test_perfect_robot_with_delay():
    all_runs, goals = [], 0
    for seed in range(1, 9):
        world = FakeWorld(seed=seed, noise=0.01)
        world.random_scene()
        ticks, runs = run_fake(world, 60.0, delay_s=0.15)
        all_runs += runs
        goals += world.goals_scored
        phases = [row[1] for row in ticks]
        assert sum(a != b for a, b in zip(phases, phases[1:])) < 80    # no flapping between phases
    med, p90 = summarise(all_runs)
    print(f"\\n[perfect robot, 0.15 s late, 8 scenes x 60 s] {len(all_runs)} runs, {goals} goals; at contact: heading "
          f"{med('hit_yaw', math.degrees(1)):.1f} deg, ball off-centre {med('hit_offcentre', 100):.1f} cm")
    assert goals >= 12 and p90("hit_offcentre", 100) < 2.2


# ---- behaviour at the edges -------------------------------------------------------
def test_no_ball_no_motion_and_run_abort():
    ctrl = ApproachController()
    out = ctrl.tick(None, (2.0, 0.0, 0.0), 0.0)
    assert out["cmd"] == {"vx": 0.0, "vy": 0.0, "wz": 0.0} and "not seen" in out["note"]

    # On the line and facing along it: it runs at once, with no pause.
    out = ctrl.tick((0.8, 0.0), (3.0, 0.0, 0.0), 0.1)
    assert out["phase"] == "RUN" and out["cmd"]["vx"] == pytest.approx(0.45)
    # The ball is then moved well off the line before contact: stop running, line up again.
    out = ctrl.tick((0.7, 0.4), (3.0, 0.0, 0.0), 0.2)
    assert out["phase"] != "RUN" and "lining up again" in out["note"] and out["cmd"]["vx"] == 0.0


def test_too_close_does_not_reverse():
    """On the line but only 30 cm behind the ball, facing the wrong way to run: it must not back up.
    It turns or walks round instead."""
    ctrl = ApproachController()
    for k in range(60):
        out = ctrl.tick((0.30, 0.10), (3.0, 0.1, 0.0), k / 30)
        assert out["cmd"]["vx"] >= 0.0
    assert out["phase"] in ("APPROACH", "ALIGN")


def test_fake_state_feeds_the_dashboard():
    world = FakeWorld()
    ctrl = ApproachController()
    st = world.state()
    rel = st["relative"]["striker"]
    st["approach"] = {"striker": ctrl.tick((rel["ball"]["x"], rel["ball"]["y"]),
                                           (rel["goal"]["x"], rel["goal"]["y"], rel["goal"]["facing"]), 0.0)}
    wire = json.loads(json.dumps(st))                     # goes over UDP like the real state
    assert wire["calib_mode"] == "fake" and wire["approach"]["striker"]["plan"]["steps"]
    with_plan = render(wire)
    wire.pop("approach")
    assert np.abs(with_plan.astype(int) - render(wire)).sum() > 0     # the plan is drawn on the field view
