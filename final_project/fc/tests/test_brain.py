"""Brain unit tests: geometry, roles, safety, shaping. Run from final_project/: pytest fc/tests -q"""
import math

from fc.brain import DEFAULTS, TUNE, Brain, Coach, merged, seg_dist, shape, to_body, unit
from fc.core import Ball, Cmd, Field, Pose, RobotCfg, RobotObs, World

F = Field(length=2.63, width=1.93, goal_width=0.61, goal_depth=0.30, ball_radius=0.09, robot_radius=0.18)
WF = Field(length=3.36, width=2.70, goal_width=1.22, goal_depth=0.30, ball_radius=0.089, robot_radius=0.18)
BLUE = RobotCfg("blue", "blue", 10, "", "stock")
RED = RobotCfg("red", "red", 11, "", "stock")
P = merged({"fast_yaw_bias_deg": 0.0})   # base speeds; no fast-gait heading correction (geometry tests)
TASK = {"coach_task": "attack"}         # a Coach task: the vs-human features are off


def world(t, me=None, ball=(0.0, 0.0), other=None, me_t=None):
    robots = {}
    if me is not None:
        robots["blue"] = RobotObs("blue", Pose(*me), t if me_t is None else me_t)
    if other is not None:
        robots["red"] = RobotObs("red", Pose(*other), t)
    b = None if ball is None else Ball(ball[0], ball[1], 0.0, 0.0, t)
    return World(t, b, robots)


def run(brain, w_fn, n, params=P, dt=1 / 30):
    out = []
    for i in range(n):
        out.append(brain.decide(w_fn(i * dt), params))
    return out


def test_settings_are_few_and_everything_else_is_fixed_tuning():
    assert len(DEFAULTS) <= 15 and not set(DEFAULTS) & set(TUNE)
    assert merged({"max_vx": 0.9})["max_vx"] == 0.9 and merged({})["dribble_close"] == 0.55


def test_geometry_helpers():
    assert abs(seg_dist((0, 0), (1, 0), (0.5, 0.3)) - 0.3) < 1e-9
    assert abs(seg_dist((0, 0), (1, 0), (2, 0)) - 1.0) < 1e-9
    assert unit(3, 4) == (0.6, 0.8)
    bx, by = to_body(Pose(0, 0, math.pi / 2), 0.0, 1.0)  # world +y is body forward when facing north
    assert abs(bx - 1) < 1e-9 and abs(by) < 1e-9


def test_frame_behind_ball_and_goal_sides():
    b = Brain(BLUE, F)
    assert b.att_x > 0 and b.own_x < 0
    u, n, a, l, phi = b._frame(Pose(-0.5, 0.0, 0.0), (0.0, 0.0), P)
    assert u[0] > 0.99 and a < 0 and abs(l) < 1e-6 and abs(abs(phi) - math.pi) < 1e-6
    r = Brain(RED, F)
    u, *_ = r._frame(Pose(0.5, 0, math.pi), (0.0, 0.0), P)
    assert u[0] < -0.99


def test_wall_ball_push_direction_keeps_stage_inside_field():
    b = Brain(BLUE, F)
    B = (0.0, -F.width / 2 + F.ball_radius)  # ball touching the south wall
    u, *_ = b._frame(Pose(-0.5, -0.5, 0.0), B, P)
    c = F.ball_radius + F.robot_radius + 0.03
    assert B[1] - u[1] * c >= -F.width / 2 + P["field_margin"] - 1e-6


def test_clear_push_walks_through_ball():
    b = Brain(BLUE, F)
    for i in range(10):
        cmd, dbg = b._clear_push(Pose(-0.30, 0.0, 0.0), (0.0, 0.0), [], P, i / 30)
    assert dbg["phase"] == "push" and cmd.vx > 0.25 and abs(cmd.wz) < 0.1
    assert dbg["target"][0] > 0.0  # aim point is beyond the ball, toward the east goal


def test_clear_push_squares_up_and_backs_off_before_pushing():
    b = Brain(BLUE, F)
    cmd, dbg = b._clear_push(Pose(-0.40, 0.0, math.pi / 2), (0.0, 0.0), [], P, 0.0)
    assert dbg["phase"] == "square" and cmd.vx == 0.0 and cmd.wz < -0.3  # turn right toward +x, no walking
    # body within clearance and the ball ahead of it: step back while turning
    cmd, dbg = Brain(BLUE, F)._clear_push(Pose(-0.26, 0.0, math.radians(60)), (0.0, 0.0), [], P, 0.0)
    assert dbg["phase"] == "square" and cmd.vx < 0.0 and cmd.wz < 0.0
    # within push_max_heading_deg: push
    cmd, dbg = Brain(BLUE, F)._clear_push(Pose(-0.30, 0.0, math.radians(30)), (0.0, 0.0), [], P, 0.0)
    assert dbg["phase"] == "push"


def test_wrong_side_routes_around_with_clearance():
    b = Brain(BLUE, F)
    cmd, dbg = b._clear_push(Pose(0.40, 0.0, math.pi), (0.0, 0.0), [], P, 0.0)
    assert dbg["phase"] == "route"
    tx, ty = dbg["target"]
    clear = F.ball_radius + F.robot_radius + P["clear_margin"]
    assert math.hypot(tx, ty) >= clear  # waypoint is outside the ball clearance circle
    assert seg_dist((0.40, 0.0), (tx, ty), (0.0, 0.0)) > F.robot_radius
    _, dbg = Brain(BLUE, F).decide(world(0.0, (0.40, 0.0, math.pi), (0.0, 0.0)), P)
    assert dbg["role"] == "attack" and dbg["phase"] == "dr_route"   # the dribble attack orbits the same way


def test_stale_pose_stops():
    b = Brain(BLUE, F)
    run(b, lambda t: world(t, (-0.3, 0.0, 0.0)), 30)
    # vs a human the robot keeps going on its dead-reckoned pose up to occl_pose_s; with a Coach task to pose_stale_s
    cmd, dbg = b.decide(world(2.0, (-0.3, 0.0, 0.0), me_t=2.0 - P["occl_pose_s"] - 0.05), P)
    assert cmd == Cmd.stop("pose_stale") and dbg["role"] == "idle"
    cmd, dbg = Brain(BLUE, F).decide(world(2.0, (-0.3, 0.0, 0.0), me_t=2.0 - P["pose_stale_s"] - 0.05), {**P, **TASK})
    assert cmd == Cmd.stop("pose_stale")
    cmd, dbg = b.decide(World(2.1, Ball(0, 0, t=2.1), {}), P)  # robot missing entirely
    assert (cmd.vx, cmd.vy, cmd.wz) == (0.0, 0.0, 0.0)


def test_ball_lost_holds_still():
    s = Brain(BLUE, F)   # never seen a ball: stand still
    res = run(s, lambda t: world(t, (-0.6, 0.0, 0.0), ball=None), 30)
    assert res[-1][1]["phase"] == "ball_lost_hold" and abs(res[-1][0].vx) < 1e-9
    b = Brain(BLUE, F)   # with a Coach task a lost ball is a stop (vs a human: guard the goal centre, below)
    run(b, lambda t: world(t, (-0.6, 0.0, 0.0)), 30, {**P, **TASK})
    res = run(b, lambda t: world(1.0 + t, (-0.6, 0.0, 0.0), ball=None), 60, {**P, **TASK})
    assert res[-1][1]["phase"] == "ball_lost_hold" and abs(res[-1][0].vx) < 1e-9


def test_clamps_and_rate_limits():
    q = {**P, "max_vx": 0.1, "max_wz": 0.4}
    b = Brain(BLUE, F)
    res = run(b, lambda t: world(t, (-0.8, 0.5, 2.5), (0.3, -0.4)), 90, q)
    prev = Cmd()
    for cmd, _ in res:
        assert abs(cmd.vx) <= 0.1 + 1e-9 and abs(cmd.wz) <= 0.4 + 1e-9 and abs(cmd.vy) <= q["max_vy"] + 1e-9
        assert abs(cmd.vx - prev.vx) <= q["accel_vx"] / 30 + 1e-9
        assert abs(cmd.wz - prev.wz) <= q["accel_wz"] / 30 + 1e-9
        prev = cmd


def test_auto_one_robot_means_attack():
    b = Brain(BLUE, F)
    _, dbg = b.decide(world(0.0, (-1.0, 0.0, math.pi), (0.5, 0.5)), P)
    assert dbg["role"] == "attack"


def test_defend_goalie_line_and_mouth_clamp():
    b = Brain(BLUE, F)
    q = {**P, "role_mode": "defend"}
    gx = -F.length / 2 + q["goalie_dist"]
    for cmd, d in run(b, lambda t: world(t, (gx, 0.0, math.pi / 2), (gx + 0.5, 0.9)), 10, q):
        assert d["role"] == "defend" and d["phase"] == "guard"
        assert abs(d["target"][0] - gx) < 1e-6 and abs(d["target"][1]) <= F.goal_width / 2 - q["gk_edge_m"] + 1e-6
    assert cmd.vx > 0  # sideways body faces north: moving north toward the ball's side is forward walking


def test_defend_clears_ball_in_front_of_goal():
    b = Brain(BLUE, F)
    q = {**P, "role_mode": "defend"}
    _, d = b.decide(world(0.0, (-1.0, 0.0, 0.0), (-0.95, 0.1)), q)
    assert d["phase"].startswith("clear_")


def test_opponent_repulsion_steers_away():
    b = Brain(BLUE, F)
    rx, ry = b._repel(Pose(0, 0, 0), [Pose(0.3, 0.0, 0.0)], P)
    assert rx < 0  # pushed away from an opponent ahead


def test_shape_rate_limits_and_stops_at_once():
    q = {**P, "accel_vx": 0.6, "accel_wz": 2.0, "max_vx": 0.3}
    c = shape(Cmd(1.0, 0.0, 1.0, "go"), Cmd(), 0.1, q)
    assert abs(c.vx - 0.06) < 1e-9 and abs(c.wz - 0.2) < 1e-9 and c.tag == "go"
    c = shape(Cmd(1.0, 0.0, 0.0), Cmd(0.29, 0.0, 0.0), 1.0, q)
    assert abs(c.vx - 0.3) < 1e-9  # clamped to max_vx
    assert shape(Cmd.stop("stale"), Cmd(0.3, 0.1, 0.8), 1 / 30, q) == Cmd.stop("stale")
    assert shape(Cmd(0.2, 0, 0), Cmd(), 0.1, {}).vx > 0  # missing keys fall back to DEFAULTS
    assert abs(shape(Cmd(-1.0, 0, 0), Cmd(-0.29, 0, 0), 1.0, {**q, "max_vx": 1.0}).vx + TUNE["max_vx_back"]) < 1e-9


def test_sent_resets_the_ramp():
    b = Brain(BLUE, F)
    run(b, lambda t: world(t, (-0.6, 0.0, 0.0), (0.0, 0.0)), 60)
    assert b.out.vx > 0.2
    b.sent(Cmd.stop("stale"))
    cmd, _ = b.decide(world(2.1, (-0.6, 0.0, 0.0), (0.0, 0.0)), P)
    assert cmd.vx <= P["accel_vx"] * 0.25 + 1e-9  # ramps up from the stop, no step back to speed


def test_orbit_side_is_held_near_wall():
    b = Brain(BLUE, F)
    B = (0.3, F.width / 2 - 0.25)  # ball near the north wall, robot in front of it (wrong side)
    sides = set()
    for i in range(45):  # 1.5 s < orbit_hold_s: no flip-flop even when the waypoint grazes the margin
        t = i / 30
        b.decide(world(t, (0.62, B[1] + 0.02 * (-1) ** i, math.pi), B), P)
        sides.add(b.orbit_side)
    assert len(sides) == 1


def test_route_waypoint_clamped_inside_the_fence_margin():
    b = Brain(BLUE, F)
    B = (0.2, F.width / 2 - 0.12)
    _, d = b.decide(world(0.0, (0.5, F.width / 2 - 0.2, math.pi), B), P)
    assert d["phase"] == "dr_route"
    assert abs(d["target"][1]) <= F.width / 2 - P["wall_margin_m"] + 1e-6


def test_goto_mode_reaches_then_turns_then_stops():
    b = Brain(BLUE, F)
    q = {**P, "role_mode": "goto", "goto_x": 0.5, "goto_y": 0.2, "goto_yaw": math.pi / 2}
    cmd, d = b.decide(world(0.0, (0.0, 0.0, 0.0), ball=None), q)  # works without a ball
    assert d["role"] == "goto" and d["phase"] == "goto" and not d["done"] and cmd.vx > 0
    _, d = b.decide(world(0.1, (0.5, 0.2, 0.0), ball=None), q)
    assert d["phase"] == "goto_turn"
    cmd, d = b.decide(world(0.2, (0.5, 0.21, math.pi / 2), ball=None), q)
    assert d["done"] and cmd == Cmd.stop("goto_done")
    _, d = b.decide(world(0.3, (0.5, 0.21, 0.0), ball=None), {**q, "goto_yaw": None})
    assert d["done"]  # no heading requested


FAST = {**P, "max_vx": 0.75, "max_wz": 2.0, "accel_vx": 1.2, "decel_vx": 2.0}


def test_fast_mode_skips_the_gait_band_and_cruises():
    b = Brain(BLUE, F)
    q = {**FAST, "role_mode": "goto", "goto_x": 1.1, "goto_y": 0.0}
    res = run(b, lambda t: world(t, (-1.1, 0.0, 0.0), ball=None), 60, q)
    vxs = [c.vx for c, _ in res]
    assert not [v for v in vxs if 0.45 + 1e-9 < v < 0.6 - 1e-9]  # stepped across, never inside
    assert max(vxs) > 0.74
    # a want hovering inside the band holds the side it is on
    c = Brain(BLUE, F)
    c.fast = False
    assert c._gait(Cmd(0.55, 0, 0), 0.0, FAST).vx == 0.45
    c.fast = True
    assert c._gait(Cmd(0.55, 0, 0), 0.0, FAST).vx == 0.6
    assert c._gait(Cmd(0.5, 0, 0), 0.0, FAST).vx == 0.45 and not c.fast  # below the middle: down-shift
    assert c._gait(Cmd(0.7, 0, 0), 0.1, FAST).vx == 0.45  # up-shift waits gait_hold_s
    assert c._gait(Cmd(0.7, 0, 0), 0.7, FAST).vx == 0.7


def test_pace_caps_the_closing_speed_near_the_ball():
    b = Brain(BLUE, F)
    c = F.ball_radius + F.robot_radius
    assert b._pace(Pose(-c - 0.01, 0, 0), (0.0, 0.0), Cmd(0.75, 0, 0), FAST).vx < FAST["contact_vx"] + 0.02
    assert b._pace(Pose(-c - 0.6, 0, 0), (0.0, 0.0), Cmd(0.75, 0, 0), FAST).vx == 0.75  # outside the zone
    assert b._pace(Pose(-c - 0.1, 0, math.pi), (0.0, 0.0), Cmd(0.75, 0, 0), FAST).vx == 0.75  # ball behind


def test_decel_brakes_faster_than_accel():
    q = {**P, "max_vx": 0.75, "accel_vx": 1.0, "decel_vx": 3.0, "accel_wz": 4.0, "decel_wz": 2.0, "max_wz": 2.0}
    assert abs(shape(Cmd(0.3, 0, 0), Cmd(0.7, 0, 0), 0.1, q).vx - 0.4) < 1e-9
    assert abs(shape(Cmd(0.75, 0, 0), Cmd(0.3, 0, 0), 0.1, q).vx - 0.4) < 1e-9
    assert abs(shape(Cmd(0.1, 0, 2.0), Cmd(0.1, 0, 0.5), 0.1, q).wz - 0.9) < 1e-9  # turning harder: accel_wz
    assert abs(shape(Cmd(0.1, 0, -2.0), Cmd(0.1, 0, 0.5), 0.1, q).wz - 0.3) < 1e-9  # reversing: decel_wz


def test_goalie_speed_cap():
    b = Brain(BLUE, WF)
    q = {**FAST, "role_mode": "defend", "goalie_vmax": 0.45}
    gx = -WF.length / 2 + q["goalie_dist"]
    for cmd, d in run(b, lambda t: world(t, (gx, -0.15, math.pi / 2), (gx + 0.6, 0.5)), 30, q):
        assert d["phase"] == "guard" and cmd.vx <= 0.45 + 1e-9
    assert cmd.vx > 0.44  # k_goalie * 0.6 m wants 3.6 m/s


def test_goalie_ramps_faster_on_its_line():
    gx = -WF.length / 2 + FAST["goalie_dist"]
    w = lambda t: world(t, (gx, -0.15, math.pi / 2), (gx + 0.6, 0.5))  # noqa: E731
    q = {**FAST, "role_mode": "defend"}
    fast = run(Brain(BLUE, WF), w, 5, q)
    slow = run(Brain(BLUE, WF), w, 5, {**q, "gk_accel_vx": 0.0})
    assert all(d["phase"] == "guard" for _, d in fast)
    assert fast[-1][0].vx > slow[-1][0].vx + 0.1 and abs(slow[-1][0].vx - 5 * q["accel_vx"] / 30) < 1e-6


def mworld(t, me, ball, bv=(0.0, 0.0)):
    """One robot (blue) and a ball with a velocity."""
    return World(t, Ball(ball[0], ball[1], bv[0], bv[1], t), {"blue": RobotObs("blue", Pose(*me), t)})


def test_solo_defends_a_ball_rolling_at_goal_and_attacks_otherwise():
    q = {**P, "solo_hold_s": 2.0}   # (observations > 1 s apart: the contested-ball model restarts, no "touches")
    b = Brain(BLUE, F)
    me = (0.6, 0.0, 0.0)  # upfield, facing the attack goal
    _, d = b.decide(mworld(0.0, me, (0.3, 0.0)), q)
    assert d["role"] == "attack"  # resting ball in midfield
    _, d = b.decide(mworld(2.1, me, (0.3, 0.0), (-1.5, 0.05)), q)
    assert d["role"] == "defend"  # reaches our goal line well inside the horizon
    _, d = b.decide(mworld(3.2, me, (-0.2, 0.0)), q)
    assert d["role"] == "defend"  # ball stopped, but the hold has not passed
    _, d = b.decide(mworld(4.3, me, (-0.2, 0.0)), q)
    assert d["role"] == "attack" and "rule" not in d   # no pressure: the plain solo rule
    for bv in ((0.8, 0.0), (-0.6, 0.9)):  # rolling away; rolling toward our end but well wide of the mouth
        b = Brain(BLUE, F)
        _, d = b.decide(mworld(0.0, me, (0.3, 0.0), bv), q)
        assert d["role"] == "attack", bv
    b = Brain(BLUE, F)  # slow but moving inside our third: defend
    _, d = b.decide(mworld(0.0, me, (-0.9, 0.3), (-0.2, 0.0)), q)
    assert d["role"] == "defend"


def test_ball_prediction_decelerates_and_stops():
    b = Brain(BLUE, F)
    q = {**P, "ball_decel": 0.5}
    assert b._ball_at((0.0, 0.0), (1.0, 0.0), 1.0, q)[0] == 0.75
    assert b._ball_at((0.0, 0.0), (1.0, 0.0), 9.0, q)[0] == 1.0  # stopped after 2 s, 1 m
    T, y = b._arrival((0.0, 0.0), (-1.0, 0.0), -0.75, q)
    assert abs(T - 1.0) < 1e-9 and y == 0.0
    assert b._arrival((0.0, 0.0), (-1.0, 0.0), -1.2, q)[0] == math.inf  # stops 1 m out


def test_return_goes_round_a_slow_ball_and_backs_while_turning():
    q = {**P, "role_mode": "defend", "return_back_vx": 0.25}
    b = Brain(BLUE, F)
    res = run(b, lambda t: mworld(t, (0.2, 0.0, 0.0), (-0.3, 0.02)), 10, q)
    cmd, d = res[-1]
    assert d["phase"] == "return" and "via" in d
    clear = F.ball_radius + F.robot_radius + q["clear_margin"]
    assert math.hypot(d["via"][0] + 0.3, d["via"][1] - 0.02) >= clear
    assert cmd.vx < 0  # goal is behind: backing toward it while turning round
    b = Brain(BLUE, F)  # a ball coming at goal is worth touching: no detour
    _, d = b.decide(mworld(0.0, (0.2, 0.0, 0.0), (-0.3, 0.02), (-0.8, 0.0)), q)
    assert d["phase"] in ("return", "block") and "via" not in d


def test_block_steps_into_the_ball_path_when_the_line_is_too_far():
    q = {**P, "max_vx": 0.75, "return_vx": 1.5, "max_wz": 1.0}
    b = Brain(BLUE, F)
    K = b._block(Pose(-0.5, 0.2, -math.pi / 2), (0.8, 0.0), (-1.6, 0.0), q)
    assert K is not None and abs(K[1]) < 0.02 and -1.065 < K[0] < 0.8
    assert b._block(Pose(-1.0, 0.0, math.pi / 2), (0.8, 0.0), (-1.6, 0.0), q) is None  # line is reachable


def test_fast_yaw_bias_applies_only_in_the_fast_gait():
    q = {**P, "act_lag_s": 0.0, "fast_yaw_bias_deg": -8.0}
    b = Brain(BLUE, F)
    for vx, want in ((0.3, 0.0), (0.75, -8.0)):
        b.hist = [(0.0, vx, 0.0, 0.0)]
        b.out = Cmd(vx, 0.0, 0.0)
        pose = b._predict(Pose(0.0, 0.0, 0.0), 0.0, 0.0, q)
        assert abs(math.degrees(pose.yaw) - want) < 1e-6
    b.hist = [(0.0, 0.75, 0.0, 0.0)]  # dead reckoning walks along the corrected heading
    pose = b._predict(Pose(0.0, 0.0, 0.0), 0.0, 1.0, q)
    assert pose.y < -0.09 and abs(math.degrees(pose.yaw) + 8.0) < 1e-6
    assert DEFAULTS["fast_yaw_bias_deg"] == -8.0   # the floor step test: the fast gait walks ~8 deg right


DR = {**P, "max_vx": 0.75, "max_wz": 2.0, "accel_vx": 1.2, "decel_vx": 2.0, "max_vy": 0.3, "role_mode": "attack"}
DR3 = DR


def test_sim_ball_curve_bends_slow_balls_more_than_fast_ones():
    from fc.sim import Sim
    turn = {}
    for v in (0.3, 1.0):
        sim = Sim(F, [BLUE], {"sim_errors": 0, "sim_ball_curve": 0.08, "sim_ball_decel": 0.0}, 1)
        sim.reset("attack_solo", robots=[])
        sim.curve = 0.08
        sim.kick(0.0, 0.0, v, 0.0)
        sim.curve = 0.08
        sim.step({}, 0.5)
        turn[v] = math.atan2(sim.ball[3], sim.ball[2])
    assert turn[0.3] > 2.5 * turn[1.0] > 0


def test_dribble_meets_a_rolling_ball_ahead_and_carries_it_near_centre():
    b = Brain(BLUE, F)
    w = World(0.0, Ball(0.0, -0.4, 0.0, 0.4, 0.0), {"blue": RobotObs("blue", Pose(-0.9, -0.4, 0.0), 0.0)})
    _, d = b.decide(w, DR)
    assert d["intercept"][1] > -0.3  # aims where the ball will be, not where it is
    b = Brain(BLUE, F)  # behind the ball on the line: carry dead centre, firm hit, gentle turning
    res = run(b, lambda t: world(t, (-0.29, 0.0, 0.0), (0.0, 0.0)), 30, DR)
    cmd, d = res[-1]
    assert d["phase"] in ("dr_carry", "dr_chase", "dr_shot") and cmd.vx >= 0.45 and abs(cmd.wz) < 0.5
    assert abs(d["lat"]) < 0.01


def test_dribble_turning_is_bounded_by_lateral_acceleration():
    b = Brain(BLUE, F)
    b.out = Cmd(0.7, 0.0, 0.0)
    for i in range(10):
        cmd, _ = b.decide(world(i / 30, (-1.0, 0.5, -0.6), (0.3, 0.0)), DR)
        v = max(cmd.vx, 0.2)
        assert abs(cmd.wz) <= TUNE["dribble_alat"] / v + 0.15 or cmd.vx < 0.05


def test_dribble_snaps_to_a_deflected_ball():
    b = Brain(BLUE, F)
    mk = lambda t, vx, vy: World(t, Ball(0.0, 0.0, vx, vy, t), {"blue": RobotObs("blue", Pose(-0.4, 0.0, 0.0), t)})
    for i in range(5):
        b.decide(mk(i / 30, 0.4, 0.0), DR)
    _, d = b.decide(mk(6 / 30, 0.0, 0.5), DR)  # knocked sideways
    assert d.get("deflect") == 1 and b.deflects == 1 and d["intercept"][1] > 0.1


def test_dribble_line_mode_slides_off_lateral_error_without_turning():
    b = Brain(BLUE, F)
    cmd, d = b.decide(world(0.0, (-0.45, 0.06, 0.0), (0.0, 0.0)), DR3)
    assert b.dr_line and d["phase"].startswith("dr_")
    assert cmd.vy < 0 and abs(cmd.wz) < 0.05  # robot left of the line: slide right, small error inside the deadband


def test_dribble_commits_to_a_turn_direction_before_reversing():
    b = Brain(BLUE, F)
    b.dr_sg = 1.0  # turning left
    w = lambda t: world(t, (-0.6, 0.0, math.radians(25)), (0.0, 0.0))  # heading 25 deg left of the line: turn right
    b.decide(w(0.0), DR3)
    assert b.dr_sg == 1.0 and b.dr_flip_t is not None  # the reversal waits
    for i in range(1, 12):
        b.decide(w(i / 30), DR3)
    assert b.dr_sg == -1.0  # ... at most dribble_commit_s


def test_dribble_shoots_when_centred_and_in_range():
    b = Brain(BLUE, F)
    cmd, d = b.decide(world(0.0, (0.6, 0.0, 0.0), (0.9, 0.0)), DR3)
    assert d["phase"] == "dr_shot" and b.pushing
    b = Brain(BLUE, F)  # far from goal: no shot
    _, d = b.decide(world(0.0, (-0.9, 0.0, 0.0), (-0.6, 0.0)), DR3)
    assert d["phase"] != "dr_shot"


def test_solo_reach_attacks_a_shot_it_can_meet_first():
    q = {**DR3, "role_mode": "auto"}
    w = mworld(0.0, (-0.8, 0.0, 0.0), (0.3, 0.0), (-0.8, 0.0))  # between the ball and our goal, facing it
    _, d = Brain(BLUE, F).decide(w, q)
    assert d["role"] == "attack"
    w = mworld(0.0, (0.9, 0.0, 0.0), (0.3, 0.0), (-0.8, 0.0))  # upfield of the ball: cannot get there first
    _, d = Brain(BLUE, F).decide(w, q)
    assert d["role"] == "defend"


VH = P


def feed(b, q, t0, n, B0, v0, me, decel=0.149, dt=1 / 30):
    """Ball rolling from B0 at v0 under the rolling model, n frames from t0 -> (last (cmd, dbg), t, B, v)."""
    (x, y), (vx, vy), out = B0, v0, None
    for i in range(n):
        out = b.decide(mworld(t0 + i * dt, me, (x, y), (vx, vy)), q)
        sp = math.hypot(vx, vy)
        if sp > 0:
            ns = max(0.0, sp - decel * dt)
            x, y, vx, vy = x + vx * dt, y + vy * dt, vx * ns / sp, vy * ns / sp
    return out, t0 + n * dt, (x, y), (vx, vy)


def test_contest_flags_touches_the_rolling_model_cannot_explain():
    b, me = Brain(BLUE, F), (1.0, 0.6, math.pi)
    (_c, d), t, B, _v = feed(b, VH, 0.0, 30, (-0.3, 0.0), (0.4, 0.0), me)
    assert d["ball_state"] == "free" and b.touches == 0  # decelerating as modelled: trusted
    (_c, d), t, B, _v = feed(b, VH, t, 8, B, (-0.2, 0.45), me)  # someone touches it
    assert d["ball_state"] == "contested" and b.touches == 1
    (_c, d), t, B, _v = feed(b, VH, t, 45, B, _v, me)  # rolls on untouched
    assert d["ball_state"] == "free" and b.touches == 1


def test_contest_ignores_our_own_push_but_not_a_sideways_kick():
    b, me = Brain(BLUE, F), (-0.28, 0.0, 0.0)  # right behind the ball
    q = {**VH, "role_mode": "attack"}
    (_c, d), t, B, _v = feed(b, q, 0.0, 10, (0.0, 0.0), (0.0, 0.0), me)
    (_c, d), t, B, _v = feed(b, q, t, 8, B, (0.5, 0.0), me)  # pushed away from us: our touch
    assert b.touches == 0 and d["ball_state"] == "free"
    (_c, d), t, B, _v = feed(b, q, t, 8, B, (0.1, 0.5), me)  # kicked sideways: someone else
    assert b.touches == 1 and d["ball_state"] == "contested"


def test_hard_roles_commit_to_defending_until_the_ball_is_clearly_away():
    b, me = Brain(BLUE, F), (0.8, 0.0, math.pi)  # upfield
    q = VH
    t, B = 0.0, (-0.2, 0.3)
    for v in ((-0.3, -0.3), (-0.3, 0.3), (-0.3, -0.3)):  # someone dribbles it in our half
        (_c, d), t, B, _v = feed(b, q, t, 12, B, v, me)
    assert d["role"] == "defend" and d["ball_state"] == "contested" and "rule" in d
    t_def = b.role_t
    (_c, d), t, B, _v = feed(b, q, t, 30, B, (0.0, 0.0), me)  # stops in our half, free: still defending
    assert d["role"] == "defend" and b.role_t == t_def
    (_c, d), t, B, _v = feed(b, q, t, 60, (0.7, 0.0), (0.0, 0.0), me)  # ball in the opponent half
    assert d["role"] == "attack" and t - b.role_t < 2.0 and b.role_t - t_def >= q["hr_hold_s"]


def test_a_coach_task_turns_the_vs_human_features_off():
    b, me = Brain(BLUE, F), (0.8, 0.0, math.pi)
    q = {**VH, "coach_task": "jockey"}
    t, B = 0.0, (-0.2, 0.3)
    for v in ((-0.3, -0.3), (-0.3, 0.3), (-0.3, -0.3)):
        (_c, d), t, B, _v = feed(b, q, t, 12, B, v, me)
    assert not b.solo and "rule" not in d and "ball_state" not in d and b.touches == 0 and d["task"] == "jockey"


def test_occluded_pose_dead_reckons_slower_then_stops():
    q = {**DR3, "pose_stale_s": 0.6}
    b, me = Brain(BLUE, F), (-0.8, 0.0, 0.0)
    for i in range(30):
        c, d = b.decide(world(i / 30, me=me, ball=(0.0, 0.0)), q)
    v_seen = c.vx
    for i in range(30):  # tag hidden from t = 1.0
        c, d = b.decide(world(1.0 + i / 30, me=me, ball=(0.0, 0.0), me_t=29 / 30), q)
    assert d["occluded"] == "pose" and 0 < c.vx < v_seen
    c, d = b.decide(world(2.6, me=me, ball=(0.0, 0.0), me_t=29 / 30), q)
    assert d["phase"] == "pose_stale" and c.vx == 0.0
    c, d = Brain(BLUE, F).decide(world(2.0, me=me, ball=(0.0, 0.0), me_t=1.0), {**q, **TASK})
    assert d["phase"] == "pose_stale"


def test_hidden_ball_keeps_the_estimate_then_guards_the_goal_centre():
    q = P
    me = (-0.5, 0.3, 0.0)

    def w(t, tb):
        return World(t, Ball(0.2, 0.0, 0.3, 0.0, tb), {"blue": RobotObs("blue", Pose(*me), t)})
    b = Brain(BLUE, F)
    for i in range(10):
        b.decide(w(i / 30, i / 30), q)
    tl = 9 / 30
    _c, d = b.decide(w(tl + 1.5, tl), q)
    assert d["ball_state"] == "occluded" and d["occluded"] == "ball" and d["ball"][0] > 0.4  # rolled on
    _c, d = b.decide(w(tl + 2.5, tl), q)
    assert d["phase"] == "hold_goal" and d["role"] == "defend" and abs(d["target"][1]) <= q["gk_shuffle_m"] + 1e-9
    _c, d = Brain(BLUE, F).decide(w(1.5, 0.0), {**q, **TASK})
    assert d["phase"].startswith("ball_lost")


def test_fence_clamp_with_mouth_exception():
    b = Brain(BLUE, F)
    x, y = b._clamp_field((F.length / 2 + 1, F.width / 2 + 1), P)
    assert abs(x - (F.length / 2 - 0.28)) < 1e-9 and abs(y - (F.width / 2 - 0.28)) < 1e-9
    x, y = b._clamp_field((F.length, 0.0), P)  # in the goal mouth: up to wall_mouth_m short of the line
    assert abs(x - (F.length / 2 - P["wall_mouth_m"])) < 1e-9


def test_fence_guard_stops_walking_into_a_fence_and_pushes_away():
    b = Brain(BLUE, F)
    pw = {**P, "wall_margin_m": 0.28, "max_vx": 0.75, "max_vy": 0.3}
    hw = F.width / 2
    near = Pose(0.0, hw - 0.20, math.pi / 2)  # 20 cm from the north fence, facing it
    cmd, hit = b._fence_guard(near, Cmd(0.75, 0.0, 0.0, "x"), pw)
    assert hit and cmd.vx < 0.05  # can no longer close in (backs off slightly)
    far = Pose(0.0, 0.0, math.pi / 2)
    cmd, hit = b._fence_guard(far, Cmd(0.75, 0.0, 0.0, "x"), pw)
    assert not hit and cmd.vx == 0.75
    along = Pose(0.0, hw - 0.20, 0.0)  # walking along the fence is allowed, with a small push away
    cmd, hit = b._fence_guard(along, Cmd(0.5, 0.0, 0.0, "x"), pw)
    assert hit and abs(cmd.vx - 0.5) < 1e-9 and cmd.vy < 0


def test_wall_ball_banks_toward_goal_and_keeps_the_contact_point_off_the_fence():
    b = Brain(BLUE, F)
    pw = {**P, "wall_margin_m": 0.28, "wall_ball_m": 0.32}
    B = (0.0, F.width / 2 - F.ball_radius - 0.01)  # ball on the north fence
    u = b._dr_dir(B, pw)
    assert u[0] > 0.3 and u[1] > 0  # toward the attacked (east) goal, into the fence: comes off it into the field
    c = F.ball_radius + F.robot_radius + 0.03
    assert b._out(B[0] - u[0] * c, B[1] - u[1] * c, pw) == 0.0
    assert b._wall_aim((0.0, 0.0), (F.length / 2, 0.0), pw) is None  # open field: unchanged


def test_fence_guard_in_decide_with_dribble_preset_near_wall():
    b = Brain(BLUE, F)
    pw = {**P, "dribble": 1, "wall_margin_m": 0.28, "wall_ball_m": 0.32, "max_vx": 0.75}
    hw = F.width / 2
    res = run(b, lambda t: world(t, (-0.4, hw - 0.22, math.pi / 2), (0.3, hw - 0.1)), 30, pw)
    assert all(c.vx * math.sin(math.pi / 2) < 0.2 for c, _ in res[5:])
    assert res[-1][1].get("fence") == 1


def test_gk_track_shuffles_in_the_middle_and_aligns_with_a_near_ball():
    q = {**P, "role_mode": "defend", "gk_track": 1, "guard_line_m": 0.2, "gk_near_m": 1.0, "gk_far_m": 2.2}
    gx = -WF.length / 2 + q["goalie_dist"]
    for yaw in (math.pi / 2, -math.pi / 2):  # facing +y (forward = +y) or -y
        ys = []
        b = Brain(BLUE, WF)
        mid = -math.copysign(q["gk_asym_m"], math.sin(yaw))
        for i in range(90):  # 3 s, ball far upfield and still: shuffling +- gk_shuffle_m about the middle
            t = i / 30       # moved gk_asym_m toward the backward (slower) side
            _, d = b.decide(world(t, (gx, mid, yaw), (1.2, 0.8)), q)
            assert d["phase"] == "guard"
            ys.append(d["target"][1])
        assert max(ys) > mid + 0.04 and min(ys) < mid - 0.04
        assert max(abs(y - mid) for y in ys) <= q["gk_shuffle_m"] + 0.02
    b = Brain(BLUE, WF)
    _, d = b.decide(world(0.0, (gx, 0.0, math.pi / 2), (gx + 0.6, 0.3)), q)  # ball close: aligned with its y
    assert abs(d["target"][1] - 0.3) < 0.02
    _, d = b.decide(world(0.1, (gx, 0.0, math.pi / 2), (gx + 0.6, 0.9)), q)  # clamped to the mouth less the edge
    assert abs(d["target"][1] - (WF.goal_width / 2 - q["gk_edge_m"])) < 1e-6


def test_gk_track_follows_a_shot_to_its_crossing_point():
    q = {**P, "role_mode": "defend", "gk_track": 1, "goalie_dist": 0.25}
    gx = -WF.length / 2 + q["goalie_dist"]
    b = Brain(BLUE, WF)
    w = World(0.0, Ball(0.5, 0.0, -1.5, -0.3, 0.0), {"blue": RobotObs("blue", Pose(gx, 0.0, math.pi / 2), 0.0)})
    _, d = b.decide(w, q)
    T, y = b._arrival((0.5, 0.0), (-1.5, -0.3), gx, q)
    ty = d["target"][1]
    assert T < 3 and ty * y > 0 and 0.2 * abs(y) < abs(ty) < 0.8 * abs(y)  # far, coming: part of the way
    w = World(0.1, Ball(-0.6, -0.2, -1.5, -0.3, 0.1), {"blue": RobotObs("blue", Pose(gx, 0.0, math.pi / 2), 0.1)})
    _, d = b.decide(w, q)
    T, y = b._arrival((-0.6, -0.2), (-1.5, -0.3), gx, q)
    assert T < 0.8 and abs(d["target"][1] - y) < 0.02  # about to arrive: aligned with the crossing


def test_long_shot_releases_the_ball_then_half_turns():
    q = {**P, "max_vx": 0.75, "accel_vx": 1.2}
    b = Brain(BLUE, WF)
    c = WF.ball_radius + WF.robot_radius
    bx = WF.length / 2 - 0.8
    b.out = Cmd(0.7, 0.0, 0.0)
    b.dr_line = True
    cmd, d = b.decide(world(0.0, (bx - c + 0.01, 0.1, 0.0), (bx, 0.1)), q)
    assert d["phase"] == "dr_follow" and d.get("shot") == 1 and cmd.vx > 0.5
    phases, wz = [], []
    for i in range(1, 40):
        cmd, d = b.decide(world(i / 30, (bx - c + 0.02 * i, 0.1, 0.0), (bx + 0.05 * i, 0.1)), q)
        phases.append(d["phase"])
        wz.append(cmd.wz)
    assert phases[:20].count("dr_follow") == 20  # drives through, then turns (toward the field centre: right)
    assert abs(wz[2]) < 0.3 and min(wz[10:20]) < -0.5


def test_post_wall_stops_sliding_into_the_end_fence_beside_the_post():
    b = Brain(BLUE, WF)
    pw = {**P, "max_vx": 0.75, "max_vy": 0.3}
    deep = Pose(-WF.length / 2 + 0.1, 0.5, math.pi / 2)  # deep in the own mouth, walking toward the post
    cmd, hit = b._fence_guard(deep, Cmd(0.6, 0.0, 0.0, "x"), pw)
    assert hit and cmd.vx < 0.2


# ---------------------------------------------------------------- 1v1 Coach
COACH = P


def world2(t, blue, red, ball=(0.0, 0.0), bv=(0.0, 0.0)):
    return World(t, Ball(ball[0], ball[1], bv[0], bv[1], t),
                 {"blue": RobotObs("blue", Pose(*blue), t), "red": RobotObs("red", Pose(*red), t)})


def test_coach_needs_two_tracked_robots_and_a_ball():
    a, b = Brain(BLUE, WF), Brain(RED, WF)
    c = Coach({"blue": a, "red": b}, WF)
    assert c.update(world(0.0, (-0.8, 0.3, 0.2), (0.1, -0.2)), P) == {}                       # one robot: no coach
    assert c.update(world(0.0, (-0.8, 0.3, 0.2), None, other=(0.9, 0.1, math.pi)), P) == {}   # no ball
    out = c.update(world(0.0, (-0.8, 0.3, 0.2), (0.1, -0.2), other=(0.9, 0.1, math.pi)), P)
    assert set(out) == {"blue", "red"} and all("coach_task" in o for o in out.values())


def test_coach_possession_needs_dwell_and_goes_through_loose():
    c = Coach({"blue": Brain(BLUE, WF), "red": Brain(RED, WF)}, WF)
    near_b, near_r, far = Pose(-0.33, 0.0, 0.0), Pose(0.33, 0.0, math.pi), Pose(1.0, 0.8, math.pi)
    poss = [c.possession(i / 30, {"blue": near_b, "red": far}, (0.0, 0.0), (0.0, 0.0), COACH) for i in range(12)]
    assert poss[0] == "loose" and poss[-1] == "blue"  # gained after 0.3 s
    t0 = 12 / 30
    seq = [c.possession(t0 + i / 30, {"blue": far, "red": near_r}, (0.0, 0.0), (0.0, 0.0), COACH) for i in range(30)]
    assert "red" in seq and seq[seq.index("red") - 1] == "loose"  # never blue -> red directly
    i_l, i_r = seq.index("loose"), seq.index("red")
    assert (i_r - i_l) / 30 >= COACH["poss_dwell_s"] - 0.04 and c.changes == 2


def test_coach_one_robot_goes_for_the_ball_the_other_shadows_goal_side():
    bb, rb = Brain(BLUE, WF), Brain(RED, WF)
    c = Coach({"blue": bb, "red": rb}, WF)
    out = c.update(world2(0.0, (-0.7, 0.0, 0.0), (0.7, 0.0, math.pi)), COACH)  # kickoff tie: exactly one attacks
    tasks = sorted(o["coach_task"] for o in out.values())
    assert tasks[0] == "attack" and tasks[1] in ("jockey", "recover")
    out = c.update(world2(0.1, (-0.3, 0.0, 0.0), (1.2, 0.6, math.pi), ball=(0.6, 0.0)), COACH)
    assert out["blue"]["coach_task"] == "attack" and out["red"]["coach_task"] in ("jockey", "recover")
    J, on_line = rb._shadow((0.6, 0.0), COACH)  # 0.75 m goal-side on the ball -> red goal line
    assert not on_line and abs(J[0] - (0.6 + COACH["jockey_m"])) < 1e-6
    J, on_line = rb._shadow((WF.length / 2 - 0.8, 0.2), COACH)  # within 1 m of the goal line: the keeper
    assert on_line and abs(J[0] - (WF.length / 2 - COACH["goalie_dist"])) < 1e-6


def test_coach_shadow_jockeys_facing_the_ball_and_reports_debug():
    b = Brain(RED, WF)
    q = {**COACH, "coach_task": "jockey", "coach_why": "goal-side", "coach_adv": -0.8, "coach_poss": "blue"}
    J, _ = b._shadow((0.2, 0.3), q)
    _, d = b.decide(World(0.0, Ball(0.2, 0.3, 0, 0, 0.0), {"red": RobotObs("red", Pose(J[0], J[1], math.pi), 0.0),
                                                           "blue": RobotObs("blue", Pose(-0.2, 0.3, 0.0), 0.0)}), q)
    assert d["role"] == "defend" and d["phase"] == "jockey" and d["task"] == "jockey"
    assert d["possession"] == "blue" and d["intent"] == "goal-side" and len(d["shadow"]) == 2


def test_opp_guard_brakes_before_the_other_robot():
    b = Brain(BLUE, WF)
    q = {**COACH, "coach_task": "jockey", "aggression": 0.0}
    want = Cmd(0.6, 0.0, 0.0)
    far = b._opp_guard(Pose(0.0, 0.0, 0.0), want, [Pose(1.5, 0.0, 0.0)], q)
    near = b._opp_guard(Pose(0.0, 0.0, 0.0), want, [Pose(0.46, 0.0, 0.0)], q)
    side = b._opp_guard(Pose(0.0, 0.0, 0.0), Cmd(0.0, 0.3, 0.0), [Pose(0.40, 0.0, 0.0)], q)
    assert far.vx == 0.6 and near.vx < 0.15 and abs(side.vy - 0.3) < 1e-9  # passing sideways is not braked
    att = b._opp_guard(Pose(0.0, 0.0, 0.0), want, [Pose(0.36, 0.0, 0.0)], {**q, "coach_task": "attack", "aggression": 1.0})
    assert abs(att.vx - 0.3) < 1e-6  # the ball player may brush past at a walking pace (aggression 1: 0.3 m/s)


def test_blind_walk_stops_after_blind_max_m_in_every_mode():
    """Own tag hidden: the dead-reckoned walk since the last fix is bounded by distance (blind_max_m), not only by
    time, in every mode (vs a human, with a Coach task, goto); stopped until a newer fix."""
    goto = {"role_mode": "goto", "goto_x": 1.2, "goto_y": 0.0}
    for extra in ({}, TASK, goto):
        for v, stops in ((1.4, True), (0.5, False)):    # 0.9 s blind: 1.26 m vs 0.45 m walked
            b = Brain(BLUE, WF)
            b.hist = [(i / 30, v, 0.0, 0.0) for i in range(60)]
            b.out, b.last_t = Cmd(v, 0.0, 0.0), 59 / 30
            q = {**P, **extra, "max_vx": 1.5}
            cmd, dbg = b.decide(world(2.0, (-1.0, 0.0, 0.0), ball=(0.6, 0.0), me_t=1.1), q)
            assert (dbg["phase"] == "pose_stale") == stops, (extra, v, dbg)
            if stops:
                assert cmd == Cmd.stop("pose_stale") and dbg["blind"] == 1
                _, dbg = b.decide(world(2.05, (-1.0, 0.0, 0.0), ball=(0.6, 0.0), me_t=1.1), q)
                assert dbg["phase"] == "pose_stale"          # the same old fix: stays stopped
                _, dbg = b.decide(world(2.1, (0.2, 0.0, 0.0), ball=(0.6, 0.0)), q)
                assert dbg["phase"] != "pose_stale"          # a new fix: plays again


def test_opp_guard_runs_without_a_coach_task_and_sees_closing_speed():
    b = Brain(BLUE, WF)
    want = Cmd(0.6, 0.0, 0.0)
    near = b._opp_guard(Pose(0.0, 0.0, 0.0), want, [Pose(0.6, 0.0, 0.0)], P)   # no coach_task key at all
    assert near.vx < 0.6
    coming = b._opp_guard(Pose(0.0, 0.0, 0.0), want, [Pose(0.6, 0.0, 0.0)], P, [(-0.4, 0.0)])
    assert coming.vx < near.vx - 0.2                    # the other robot walking at us: brake for the closing speed
    # decide: the guard runs whenever another robot is tracked (the Coach gives {} on a stale tag / unseen ball)
    calls = []
    b._opp_guard = lambda pose, w, others, p, vel=None: calls.append(len(others)) or w
    b.decide(world(0.0, (-0.6, 0.0, 0.0), ball=(0.3, 0.0), other=(0.0, 0.0, math.pi)), P)
    assert calls == [1]
    # a lost robot stops itself within blind_max_m: it stays an obstacle up to opp_keep_s, rolled by its velocity
    m = Brain(BLUE, WF)
    m._movers(World(0.0, None, {"red": RobotObs("red", Pose(0.0, 0.0, 0.0), 0.0)}), 0.0, P)
    poses, vels = m._movers(World(0.1, None, {"red": RobotObs("red", Pose(0.05, 0.0, 0.0), 0.1)}), 0.1, P)
    assert 0.1 < vels[0][0] < 0.5 and abs(poses[0].x - 0.05) < 1e-9
    poses, _ = m._movers(World(2.5, None, {"red": RobotObs("red", Pose(0.05, 0.0, 0.0), 0.1)}), 2.5, P)
    assert len(poses) == 1 and poses[0].x > 0.05      # lost for 2.4 s (> other_stale_s): still guarded
    assert m._movers(World(5.0, None, {"red": RobotObs("red", Pose(0.05, 0.0, 0.0), 0.1)}), 5.0, P)[0] == []


# ---- tilt throttle (ack IMU tilt held high -> zero translation) ---------------------------------------------
def _feed(thr, series, dt=1 / 30, age=0.02):
    """series: tilt per 30 Hz ack; returns the active flag after each."""
    out, t = [], 100.0
    for v in series:
        t += dt
        out.append(thr.update(t, v, age))
    return out


def test_tilt_throttle_ignores_normal_play_spikes():
    from fc.brain import TiltThrottle
    # worst normal-play events in the physics sim at max_vx 1.0: 51 deg peak, > 25 deg for 0.35 s, recovered
    assert not any(_feed(TiltThrottle(), [5] * 30 + [30, 40, 51, 45, 38, 33, 30, 28, 27, 26, 26] + [8] * 30))
    # repeated sub-hold spikes with short dips do not add up
    assert not any(_feed(TiltThrottle(), ([28] * 12 + [10] * 2) * 6))
    # stale acks (Wi-Fi gap) are no evidence: never start it
    assert not any(_feed(TiltThrottle(), [60] * 40, age=1.0))
    assert not any(_feed(TiltThrottle(), [None, float("nan"), "x", True] * 10))


def test_tilt_throttle_catches_a_robot_going_over_and_releases_when_upright():
    from fc.brain import TILT_STOP_HOLD_S, Cmd, TiltThrottle
    thr = TiltThrottle()
    # 20261128T065952Z-step: 39 deg at t=2.66, 78 at 3.19 while vx kept ramping to 1.4
    act = _feed(thr, [3] * 10 + [39, 45, 52, 60, 66, 70, 74, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78])
    first = act.index(True)
    assert abs(first - 10 - TILT_STOP_HOLD_S * 30) <= 1                                 # ~0.5 s of acks above 25
    assert thr.apply(Cmd(1.2, 0.1, 0.4, "carry")) == Cmd(0.0, 0.0, 0.4, "tilt")         # translation zero, wz kept
    assert thr.apply(Cmd(0.0, 0.0, 0.3, "x")).tag == "x"
    assert _feed(thr, [70] * 5, age=2.0)[-1] is True                                      # stale: stays on
    rec = _feed(thr, [10] * 20)
    assert rec[:14] == [True] * 14 and rec[-1] is False                                   # 0.5 s upright -> off
    assert thr.apply(Cmd(1.2, 0.0, 0.0, "carry")).vx == 1.2
