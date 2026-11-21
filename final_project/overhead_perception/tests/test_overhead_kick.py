"""The job selection and the learning of scripts/overhead_kick.py against a kinematic Pupper with
wrong walker gains, a camera bias and a ball that the chest can push."""
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
from overhead_kick import (Job, Params, Striker, command, dist, move_name, seg_dist, to_body,  # noqa: E402
                           to_field, true_pose, wrap)

P = Params()
L, W = 2.72, 1.97
TRUE_X, TRUE_Y = 0.248, {"left": 0.037, "right": -0.042}   # the real kick point
CHEST_X = 0.23                                             # ball centre x at which the chest touches the ball


class World:
    """True robot pose and ball. The camera sees the robot shifted by `bias` in the body frame."""

    def __init__(self, pose, ball, seed=1, gains=None, bias=(0.0, 0.0), noise=0.002, turn_gain=1.0, kick_range=1.0,
                 tilt=(10.0, 0.0)):
        self.pose, self.ball, self.rng = tuple(pose), tuple(ball), random.Random(seed)
        self.gains = {"forward": (0.84, 0.0), "backward": (0.84, 0.0), "left": (0.7, 0.0), "right": (0.7, 0.0),
                      "forward_fast": (0.9, 0.0), **(gains or {})}
        self.tilt = tilt  # the camera reads the heading with an error: true = read + a cos(read) + b sin(read)
        self.bias, self.noise, self.turn_gain, self.kick_range = bias, noise, turn_gain, kick_range
        self.nearest = 9.0  # nearest approach of the Pupper centre to the ball centre

    def measure(self):
        n = lambda: self.rng.gauss(0.0, self.noise)  # noqa: E731
        x, y = to_field(self.pose, self.pose[2], -self.bias[0], -self.bias[1])
        read = self.pose[2]
        for _ in range(20):  # the heading that the camera reads: true = read + a cos(read) + b sin(read)
            read = self.pose[2] - math.radians(self.tilt[0] * math.cos(read) + self.tilt[1] * math.sin(read))
        return (x + n(), y + n(), wrap(read + n())), (self.ball[0] + n(), self.ball[1] + n())

    def do(self, job: Job, striker: Striker) -> bool:
        """Play one job with the true walker. Returns True if a kick moved the ball."""
        x, y, yaw = self.pose
        if job.kind == "kick":
            bx, by = to_body(self.pose, self.ball)
            off = by - TRUE_Y[job.value]
            if abs(bx - TRUE_X) > 0.025 or abs(off) > 0.035:
                return False
            heading = yaw + math.radians(6.0 * off * 100.0) + self.rng.gauss(0.0, math.radians(2.0))
            self.ball = (self.ball[0] + self.kick_range * math.cos(heading),
                         self.ball[1] + self.kick_range * math.sin(heading))
            return True
        speed = P.fast_speed if job.fast else P.speed
        value = command(job, striker.model(move_name(job)), speed)
        if job.kind == "turn":
            self.pose = (x, y, wrap(yaw + math.copysign(math.radians(value) * self.turn_gain, job.value)))
            return False
        gain, dead = self.gains[move_name(job)]
        d = math.copysign(gain * speed * max(0.0, value - dead), job.value)
        steps = max(1, int(abs(d) / 0.01))
        for _ in range(steps):
            x, y = to_field((x, y), yaw, d / steps, 0.0) if job.kind == "walk" else to_field((x, y), yaw, 0.0, d / steps)
            bx, by = to_body((x, y, yaw), self.ball)
            self.nearest = min(self.nearest, math.hypot(bx, by))
            if 0.0 < bx < CHEST_X and abs(by) < 0.13:  # the chest pushes the ball ahead
                self.ball = to_field((x, y), yaw, CHEST_X, by)
        self.pose = (x, y, yaw)
        return False


def play(world: World, striker: Striker, kicks=1, limit=80):
    """Run the bridge loop. Returns the kick records and the jobs."""
    records, jobs, pending = [], [], None
    for _ in range(limit):
        raw, ball = world.measure()
        pose = true_pose(raw, striker.model("heading"))
        if pending:
            striker.observe(pending[0], pending[1], pending[2], pose, ball, pending[3])
            pending = None
        if not striker.in_play(ball):
            break
        job = striker.next_job(pose, ball)
        jobs.append(job)
        if job.kind != "kick":
            world.do(job, striker)
            pending = (job, pose, ball, raw[2])
            continue
        true_ball = to_body(world.pose, world.ball)
        world.do(job, striker)
        rec = striker.kick_result(pose, ball, world.measure()[1])
        rec["true_ball"], rec["true_yaw"] = true_ball, world.pose[2]
        records.append(rec)
        if len(records) == kicks:
            break
    return records, jobs


def test_geometry_helpers():
    assert to_body((1.0, 1.0, math.pi / 2), (1.0, 2.0)) == (1.0, 0.0) or abs(to_body((1.0, 1.0, math.pi / 2), (1.0, 2.0))[1]) < 1e-9
    x, y = to_field((1.0, 1.0), math.pi / 2, 0.5, 0.2)
    assert abs(x - 0.8) < 1e-9 and abs(y - 1.5) < 1e-9
    assert abs(seg_dist((0, 0), (2, 0), (1, 0.4)) - 0.4) < 1e-9 and abs(seg_dist((0, 0), (2, 0), (3, 0)) - 1.0) < 1e-9


def test_shot_heading_points_into_each_goal():
    s = Striker(P, L, W)
    east, half = s.shot((0.0, 0.0), "east")
    west, _ = s.shot((0.0, 0.0), "west")
    assert abs(east) < 1e-9 and abs(abs(west) - math.pi) < 1e-9
    assert abs(half - math.atan2(P.goal_width / 2 - P.post_margin, L / 2)) < 1e-9
    assert s.shot((0.5, 0.6), "east")[0] < 0 and s.shot((0.5, -0.6), "east")[0] > 0   # aim back to the middle
    assert abs(wrap(s.shot((-0.5, 0.6), "west")[0] - math.pi)) > 0.1


def test_stage_point_is_behind_the_ball_on_the_shot_line():
    s = Striker(P, L, W)
    for goal in ("east", "west"):
        for side in ("left", "right"):
            ball = (0.3, -0.2)
            spot, (psi, _) = s.stage(ball, goal, side), s.shot(ball, goal)
            bx, by = to_body((spot[0], spot[1], psi), ball)
            k = s.kick_point(goal, side)
            assert abs(bx - (k["x"] + P.standoff)) < 1e-9 and abs(by - k["y"]) < 1e-9


def test_signs_of_one_job():
    s = Striker(P, L, W, goal="east")
    far = s.next_job((-1.0, 0.0, 0.0), (0.5, 0.0))
    assert far.kind == "walk" and far.value == P.max_step                 # behind the ball, facing it: walk
    assert s.next_job((-1.0, 0.0, math.pi / 2), (0.5, 0.0)).kind == "turn"
    assert s.next_job((-1.0, 0.0, math.pi / 2), (0.5, 0.0)).value < 0     # stage point to the right: turn right
    k = s.kick_point("east", "left")
    def at(ex, ey, yaw=0.0):  # the job for a ball at the kick point plus (ex, ey)
        s.pulses = 0
        return s.next_job((0.5 - k["x"] - ex, -k["y"] - ey, yaw), (0.5, 0.0))

    assert at(0.0, 0.0).kind == "kick" and at(0.0, 0.0).value == "left"
    assert at(0.0, 0.02).kind == "turn" and at(0.0, 0.02).value > 0        # ball left of the paw line: turn left
    assert at(0.0, -0.02).kind == "turn" and at(0.0, -0.02).value < 0
    assert at(0.15, 0.03).kind == "turn" and at(0.15, 0.03).value > 0
    assert at(0.03, 0.0).kind == "walk" and abs(at(0.03, 0.0).value - 0.03) < 1e-6    # too far: one walk to the ball
    assert at(-0.03, 0.0).kind == "walk" and at(-0.03, 0.0).value < 0      # too near: step back
    assert at(0.02, 0.08).kind == "walk" and at(0.02, 0.08).value < 0      # large side step at the ball: back first
    s.lining = False
    assert at(0.15, 0.0, math.radians(20)).kind == "turn"                  # at the stage point, wrong heading
    assert at(0.15, 0.0, math.radians(20)).value < 0


def test_side_step_only_if_a_turn_leaves_the_heading_window():
    s = Striker(P, L, W, goal="east")
    k = s.kick_point("east", "left")
    job = s.next_job((0.5 - k["x"] - 0.15, -k["y"] - 0.11, 0.0), (0.5, 0.0))   # 11 cm to the side at the standoff
    assert job.kind == "side" and 0 < job.value <= P.max_side
    s.lining = False
    job = s.next_job((0.5 - k["x"] - 0.15, -k["y"] - 0.04, 0.0), (0.5, 0.0))   # 4 cm: a turn of approximately 6 degrees
    assert job.kind == "turn" and 3.0 < job.value < 9.0


def test_no_turn_with_the_ball_at_the_chest():
    s = Striker(P, L, W, goal="east")
    job = s.next_job((0.2, 0.0, math.radians(40)), (0.45, 0.0))
    assert job.kind == "walk" and job.value < 0


def test_kick_from_each_side_of_the_ball_into_each_goal():
    starts = [(-1.0, -0.5, 0.0), (1.0, 0.6, 2.0), (0.0, -0.8, -2.5), (0.9, -0.2, math.pi), (0.6, 0.1, math.pi)]
    for goal in ("east", "west", "auto"):
        for start in starts:
            world, s = World(start, (0.1, 0.1)), Striker(P, L, W, goal=goal)
            (rec,), jobs = play(world, s)
            assert rec["result"] in ("goal", "hit"), (goal, start, rec)
            bx, by = rec["true_ball"]
            assert abs(bx - TRUE_X) <= 0.02 and abs(by - TRUE_Y["left"]) <= 0.02, (goal, start, bx, by)
            psi, half = s.shot((0.1, 0.1), rec["goal"])
            assert abs(wrap(rec["true_yaw"] - psi)) <= half + math.radians(P.aim_margin_deg), (goal, start)
            assert len(jobs) <= 25, (goal, start, len(jobs))
            if goal != "auto":
                assert rec["goal"] == goal


def test_the_walk_goes_around_the_ball():
    # The Pupper starts on the goal side of the ball. Before the line-up it stays clear of the ball.
    for start in [(0.9, 0.1, math.pi), (0.7, -0.3, 2.0), (0.6, 0.12, 0.0)]:
        world, s = World(start, (0.1, 0.1)), Striker(P, L, W, goal="east")
        pending = None
        for _ in range(40):
            raw, ball = world.measure()
            pose = true_pose(raw, s.model("heading"))
            if pending:
                s.observe(pending[0], pending[1], pending[2], pose, ball, pending[3])
            job = s.next_job(pose, ball)
            if s.lining:
                break
            world.do(job, s)
            pending = (job, pose, ball, raw[2])
        assert s.lining, start
        assert world.nearest >= 0.24, (start, world.nearest)
        assert dist(world.ball, (0.1, 0.1)) < 1e-9, start


def test_auto_selects_the_goal_with_the_short_walk():
    s = Striker(P, L, W)
    s.choose((-0.8, 0.0, 0.0), (0.0, 0.0))
    assert s.goal == "east"
    s.choose((0.8, 0.0, 0.0), (0.0, 0.0))
    assert s.goal == "west"
    s.choose((0.0, 0.5, 0.0), (-L / 2 + 0.2, 0.0))   # ball at the west goal line: no room behind it for east
    assert s.goal == "west"


def test_converges_with_wrong_gains_and_noise():
    gains = {"forward": (1.2, 0.08), "backward": (0.6, 0.05), "left": (0.45, 0.1), "right": (0.9, 0.0)}
    for seed in range(12):
        world = World((-1.1, 0.7, 1.0), (0.4, -0.3), seed=seed, gains=gains, turn_gain=0.8, noise=0.003)
        (rec,), jobs = play(world, Striker(P, L, W, cal={}))
        bx, by = rec["true_ball"]
        assert abs(bx - TRUE_X) <= 0.022 and abs(by - TRUE_Y["left"]) <= 0.022, (seed, bx, by)
        assert len(jobs) <= 40, (seed, len(jobs))


def test_the_heading_correction_is_learned_from_walks():
    # The tag heading is wrong by a cos + b sin. Walks to the east and to the west correct both signs.
    for tilt in ((0.0, 0.0), (16.0, -8.0)):
        s = Striker(P, L, W, cal={})
        jobs = []
        for seed, (start, ball) in enumerate([((-1.0, 0.4, 0.5), (0.3, -0.1)), ((1.0, -0.3, 2.5), (-0.2, 0.2)),
                                              ((-0.9, -0.5, 0.0), (0.2, 0.3)), ((0.9, 0.5, 3.0), (-0.3, -0.2)),
                                              ((-1.0, 0.4, 0.5), (0.3, -0.1)), ((1.0, -0.3, 2.5), (-0.2, 0.2))]):
            world = World(start, ball, seed=seed, tilt=tilt)
            (rec,), j = play(world, s)
            jobs.append(len(j))
            assert rec["result"] in ("goal", "hit"), (tilt, seed, rec)
        a, b = s.model("heading")
        assert abs(a - tilt[0]) < 5.0, (tilt, a, b)
        assert max(jobs[-2:]) <= 10, (tilt, jobs)


def test_a_far_walk_is_fast_and_the_line_up_is_slow():
    s = Striker(P, L, W, goal="east")
    far = s.next_job((-1.0, 0.0, 0.0), (0.5, 0.0))
    assert far.kind == "walk" and far.fast and move_name(far) == "forward_fast"
    k = s.kick_point("east", "left")
    near = s.next_job((0.5 - k["x"] - 0.10, -k["y"], 0.0), (0.5, 0.0))
    assert near.kind == "walk" and not near.fast and abs(near.value - 0.10) < 1e-6      # one walk to the kick point


def test_observe_corrects_the_walker_model():
    s = Striker(P, L, W, cal={})
    s.observe(Job("walk", 0.30), (0, 0, 0), None, (0.45, 0, 0), None)         # 1.5 times the plan
    assert abs(s.model("forward")[0] - 0.84 * 1.25) < 1e-3
    s.observe(Job("side", -0.20), (0, 0, 0), None, (0, -0.10, 0), None)       # half the plan
    assert abs(s.model("right")[0] - 0.70 * 0.75) < 1e-3
    s.observe(Job("walk", 0.02), (0, 0, 0), None, (0, 0, 0), None)            # a short pulse with no travel
    assert s.model("forward")[1] > 0
    s.observe(Job("turn", 60.0), (0, 0, 0), None, (0, 0, math.radians(90)), None)
    assert abs(s.model("turn")[0] - 1.25) < 1e-3
    s.observe(Job("turn", -10.0), (0, 0, 0), None, (0, 0, 0), None)
    assert s.model("turn")[1] > 0
    before = (list(s.model("forward")), list(s.model("turn")), s.odd)
    s.observe(Job("walk", 0.30), (0, 0, 0), None, (0.30, 0, math.radians(40)), None)   # the Pupper turned in a walk
    s.observe(Job("turn", -20.0), (0, 0, 0), None, (0, 0, math.radians(15)), None)     # a turn to the wrong side
    s.observe(Job("walk", 0.30), (0, 0, 0), None, (0.02, 0, 0), None)                  # no travel
    assert (s.model("forward"), s.model("turn")) == before[:2] and s.odd == before[2] + 3
    assert command(Job("walk", 0.168), [0.84, 0.1], 0.2) == 0.1 + 1.0
    assert command(Job("turn", -30.0), [1.5, 4.0], 0.2) == 24.0


def test_a_touch_moves_the_kick_point_and_starts_the_kick():
    # The camera sees the ball 7 cm farther than it is: the walk to the kick point pushes the ball.
    world, s = World((-0.6, 0.0, 0.0), (0.2, 0.04), bias=(0.07, 0.0)), Striker(P, L, W, cal={}, goal="east")
    (rec,), jobs = play(world, s)
    assert rec["touched"] and rec["result"] in ("goal", "hit"), rec
    assert rec["kick_point_next"][0] > P.kick_x + 0.01


def test_a_miss_moves_the_kick_point_nearer_until_the_kick_hits():
    # The camera sees the ball 4 cm nearer than it is: the first kicks do not get to the ball.
    world, s = World((-0.6, 0.0, 0.0), (0.2, 0.04), bias=(-0.04, 0.0), kick_range=0.0), Striker(P, L, W, cal={}, goal="east")
    recs, _ = play(world, s, kicks=3, limit=200)
    assert [r["result"] for r in recs] == ["miss", "miss", "miss"]
    assert s.kick_point("east", "left")["x"] <= P.kick_x - 2.9 * P.miss_step
    bx, _ = recs[-1]["true_ball"]
    assert abs(bx - TRUE_X) < abs(recs[0]["true_ball"][0] - TRUE_X)


def test_the_ball_direction_corrects_the_aim_or_kick_y():
    # The camera sees the ball to the right of where it is: the ball leaves to the left of the heading.
    # A small error (9 degrees) turns the aim; a large one (15 degrees) moves the kick point.
    for bias, small in ((-0.015, True), (-0.025, False)):
        s = Striker(P, L, W, cal={}, goal="east")
        errs, offs = [], []
        for i in range(6):
            world = World((-0.9, 0.1, 0.0), (0.0, 0.0), seed=i, bias=(0.0, bias), kick_range=0.8)
            (rec,), _ = play(world, s)
            assert rec["dir_err_deg"] is not None, (bias, i, rec)
            errs.append(rec["dir_err_deg"])
            offs.append(math.degrees(wrap(rec["true_yaw"] + math.radians(rec["dir_err_deg"]) - s.shot((0.0, 0.0), "east")[0])))
        mid = lambda v: sorted(abs(e) for e in v)[len(v) // 2]  # noqa: E731
        assert errs[0] > 5.0, errs
        assert mid(offs[-3:]) < 5.0, (bias, offs)                  # the ball goes to the middle of the goal
        k = s.kick_point("east", "left")
        if small:
            assert k["dir"] > 2.0 or k["y"] < P.kick_y_left - 0.003, k
        else:
            assert k["y"] < P.kick_y_left - 0.005 and mid(errs[-3:]) < abs(errs[0]), (k, errs)


def test_the_onboard_shift_moves_the_line_up_and_not_the_walk():
    s = Striker(P, L, W, goal="east")
    k = s.kick_point("east", "left")
    pose, ball = (0.5 - k["x"], -k["y"], 0.0), (0.5, 0.0)
    assert s.next_job(pose, ball).kind == "kick"
    job = s.next_job(pose, ball, shift=(0.05, 0.0))        # the onboard camera sees the ball 5 cm farther
    assert job.kind == "walk" and job.value > 0
    job = s.next_job(pose, ball, shift=(0.0, -0.03))       # ... 3 cm to the right
    assert job.kind == "turn" and job.value < 0
    far = (-1.0, 0.0, 0.0)
    assert s.next_job(far, ball) == s.next_job(far, ball, shift=(0.1, 0.1))
    rec = s.kick_result(pose, ball, (0.51, 0.0))
    assert rec["shift"] == [0.1, 0.1]


def test_a_small_direction_error_turns_the_aim():
    s = Striker(P, L, W, cal={}, goal="east")
    s.choose((-0.5, 0.0, 0.0), (0.0, 0.0))
    psi0 = s.aim((0.0, 0.0))[0]
    # The ball leaves 6 degrees to the right of the heading.
    rec = s.kick_result((-0.25, -0.037, 0.0), (0.0, 0.0), (0.8, -0.084))
    assert -6.5 < rec["dir_err_deg"] < -5.5 and rec["kick_point_next"][1] == P.kick_y_left
    s.choose((-0.5, 0.0, 0.0), (0.0, 0.0))
    assert abs(math.degrees(s.aim((0.0, 0.0))[0] - psi0) - 3.0) < 0.3      # aim half of the error to the left
    spot = s.stage((0.0, 0.0), "east", "left")
    assert abs(math.degrees(math.atan2(-spot[1], -spot[0])) - 3.0) < 12.0  # the stage point turns with the aim


def test_kick_result_names_the_goal():
    s = Striker(P, L, W, goal="east")
    s.choose((0.5, 0.0, 0.0), (0.9, 0.0))
    rec = s.kick_result((0.5, 0.0, 0.0), (0.9, 0.0), None, trace=[(1.1, 0.02), (L / 2 + 0.05, 0.05)])
    assert rec["result"] == "goal" and rec["scored"] == "east" and s.goal is None
    s.choose((0.5, 0.0, 0.0), (0.9, 0.0))
    assert s.kick_result((0.5, 0.0, 0.0), (0.9, 0.0), (L / 2 + 0.1, 0.6))["result"] == "hit"     # wide of the post
    s.choose((0.5, 0.0, 0.0), (0.9, 0.0))
    assert s.kick_result((0.5, 0.0, 0.0), (0.9, 0.0), None)["result"] == "lost"
    s.choose((0.5, 0.0, 0.0), (0.9, 0.0))
    assert s.kick_result((0.5, 0.0, 0.0), (0.9, 0.0), (0.91, 0.0))["result"] == "miss"


def test_repeat_kicks_bring_the_ball_into_a_goal():
    goals = 0
    for seed in range(8):
        world = World((-0.9, 0.5, 0.3), (-0.3, 0.2), seed=seed, kick_range=0.7, noise=0.003)
        recs, _ = play(world, Striker(P, L, W, cal={}), kicks=6, limit=300)
        goals += recs[-1]["result"] == "goal"
        assert len(recs) <= 4, (seed, [r["result"] for r in recs])
    assert goals >= 7, goals


def test_ball_out_of_play():
    s = Striker(P, L, W)
    assert s.in_play((0.0, 0.0)) and s.in_play((L / 2 - 0.1, W / 2 + 0.05))
    assert not s.in_play((L / 2 + 0.1, 0.0)) and not s.in_play((0.0, W / 2 + 0.3))


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
