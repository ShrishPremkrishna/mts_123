"""Brain unit tests: geometry, roles, safety, shaping. Run from final_project/: pytest fc/tests -q"""
import math

from fc.brain import Brain, DEFAULTS, seg_dist, shape, to_body, unit
from fc.core import Ball, Cmd, Field, Pose, RobotCfg, RobotObs, World

F = Field(length=2.63, width=1.93, goal_width=0.61, goal_depth=0.30, ball_radius=0.09, robot_radius=0.18)
BLUE = RobotCfg("blue", "blue", 10, "", "stock")
RED = RobotCfg("red", "red", 11, "", "stock")
P = {**DEFAULTS, "yaw_learn": 0, "predict_self": 0}


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


def test_push_from_behind_walks_through_ball_and_never_stops():
    b = Brain(BLUE, F)
    res = run(b, lambda t: world(t, (-0.30, 0.0, 0.0), (0.0, 0.0)), 40)
    cmd, dbg = res[-1]
    assert dbg["role"] == "attack" and dbg["phase"] == "push"
    assert cmd.vx > 0.25 and abs(cmd.wz) < 0.1
    assert dbg["target"][0] > 0.0  # aim point is beyond the ball, toward the east goal


def test_wrong_side_routes_around_with_clearance():
    b = Brain(BLUE, F)
    cmd, dbg = run(b, lambda t: world(t, (0.40, 0.0, math.pi), (0.0, 0.0)), 5)[-1]
    assert dbg["phase"] == "route"
    tx, ty = dbg["target"]
    clear = F.ball_radius + F.robot_radius + P["clear_margin"]
    assert math.hypot(tx, ty) >= clear  # waypoint is outside the ball clearance circle
    assert seg_dist((0.40, 0.0), (tx, ty), (0.0, 0.0)) > F.robot_radius


def test_stale_pose_stops_immediately():
    b = Brain(BLUE, F)
    run(b, lambda t: world(t, (-0.3, 0.0, 0.0)), 30)
    cmd, dbg = b.decide(world(2.0, (-0.3, 0.0, 0.0), me_t=2.0 - P["pose_stale_s"] - 0.05), P)
    assert cmd == Cmd.stop("pose_stale") and dbg["role"] == "idle"
    cmd, dbg = b.decide(World(2.1, Ball(0, 0, t=2.1), {}), P)  # robot missing entirely
    assert (cmd.vx, cmd.vy, cmd.wz) == (0.0, 0.0, 0.0)


def test_ball_lost_hold_and_search():
    b = Brain(BLUE, F)
    run(b, lambda t: world(t, (-0.6, 0.0, 0.0)), 30)
    res = run(b, lambda t: world(1.0 + t, (-0.6, 0.0, 0.0), ball=None), 60)
    assert res[-1][1]["phase"] == "ball_lost_hold" and abs(res[-1][0].vx) < 1e-9
    s = Brain(BLUE, F)
    res = run(s, lambda t: world(t, (-0.6, 0.0, 0.0), ball=None), 60, {**P, "ball_lost": "search"})
    assert abs(res[-1][0].wz - P["search_wz"]) < 1e-9


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


def test_role_hysteresis_and_hold():
    q = {**P, "role_tau_s": 0.01, "role_hold_s": 2.0, "role_hyst_s": 1.0}
    b = Brain(BLUE, F)
    ball = (0.3, 0.0)
    # I am right behind the ball; the opponent is far and facing away -> attack
    _, d = b.decide(world(0.0, (0.0, 0.0, 0.0), ball, other=(1.2, 0.8, 0.0)), q)
    assert d["role"] == "attack"
    # opponent now right behind the ball for its own attack, I am far away facing the wrong way,
    # but the hold time has not passed -> still attack
    far = (-1.0, 0.8, math.pi)
    _, d = b.decide(world(0.5, far, ball, other=(0.62, 0.0, math.pi)), q)
    assert d["role"] == "attack"
    _, d = b.decide(world(2.6, far, ball, other=(0.62, 0.0, math.pi)), q)
    assert d["role"] == "defend"
    # inside the hysteresis band a defender returns to attack only when it is not slower
    my, opp = b._cost(Pose(*far), ball, b.att_x, q), b._cost(Pose(0.62, 0.0, math.pi), ball, b.own_x, q)
    assert my > opp


def test_defend_goalie_line_and_clamp():
    b = Brain(BLUE, F)
    q = {**P, "role_mode": "defend"}
    gx = -F.length / 2 + q["goalie_dist"]
    res = run(b, lambda t: world(t, (gx, 0.2, math.pi / 2), (0.3, 0.9)), 10, q)
    cmd, d = res[-1]
    assert d["role"] == "defend" and d["phase"] == "guard"
    assert abs(d["target"][0] - gx) < 1e-6
    assert abs(d["target"][1] - (F.goal_width / 2 + q["goalie_margin"])) < 1e-6  # clamped to the mouth
    assert cmd.vx > 0  # sideways style: body faces north, so moving north is forward walking


def test_defend_clears_ball_in_front_of_goal():
    b = Brain(BLUE, F)
    q = {**P, "role_mode": "defend"}
    _, d = b.decide(world(0.0, (-1.0, 0.0, 0.0), (-0.95, 0.1)), q)
    assert d["phase"].startswith("clear_")


def test_headbutt_lines_up_then_charges():
    b = Brain(BLUE, F)
    q = {**P, "contact_mode": "headbutt"}
    sd = F.ball_radius + F.robot_radius + q["hb_standoff"]
    phases = [d["phase"] for _, d in run(b, lambda t: world(t, (-sd - 0.02, 0.0, 0.0), (0.0, 0.0)), 5, q)]
    assert "charge" in phases


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


def test_route_waypoint_clamped_inside_field():
    b = Brain(BLUE, F)
    B = (0.2, F.width / 2 - 0.12)
    _, d = b.decide(world(0.0, (0.5, F.width / 2 - 0.2, math.pi), B), P)
    assert d["phase"] == "route"
    assert abs(d["target"][1]) <= F.width / 2 - P["field_margin"] + 1e-6


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


def test_yaw_learn_off_by_default_and_bounded():
    assert DEFAULTS["yaw_learn"] == 0 and DEFAULTS["yaw_learn_rate"] == 0.02
    b = Brain(BLUE, F)
    b.yaw_off, b.yaw_bias = math.radians(30), [math.radians(30)] * 8
    assert abs(b._bias_at(0.3, math.radians(DEFAULTS["yaw_learn_max_deg"]))) <= math.radians(20) + 1e-9
    b.reset(forget_yaw=True)
    assert b.yaw_off == 0.0 and not any(b.yaw_bias)


def test_yaw_learn_skips_push_and_learns_clean_walk():
    q = {**P, "yaw_learn": 1, "yaw_learn_rate": 0.05, "predict_self": 0}
    bias = math.radians(10)
    # pushing the ball: never learns
    b = Brain(BLUE, F)
    run(b, lambda t: world(t, (-0.3 + 0.27 * t, 0.0, bias), (0.0 + 0.27 * t, 0.0)), 90, q)
    assert b.yaw_off == 0.0 and not any(b.yaw_bias)
    # clean straight walk east (ball far away, nobody close), tag reads 10 deg left: learns toward -10
    c = Brain(BLUE, F)
    q2 = {**q, "role_mode": "goto", "goto_x": 1.2, "goto_y": 2.2 * math.tan(bias)}  # straight ahead by the tag
    run(c, lambda t: world(t, (-1.0 + 0.25 * t, 0.0, bias), (0.9, 0.8)), 90, q2)
    assert -math.radians(20) <= c._bias_at(bias) < -math.radians(1)


FAST = {**P, "max_vx": 0.75, "max_wz": 2.0, "accel_vx": 1.2, "decel_vx": 2.0, "push_vx": 0.75,
        "contact_vx": 0.4, "contact_slow_m": 0.35}


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


def test_fast_mode_slows_to_contact_speed_near_the_ball():
    b = Brain(BLUE, F)
    c = F.ball_radius + F.robot_radius
    res = run(b, lambda t: world(t, (-c - 0.02, 0.0, 0.0), (0.0, 0.0)), 60, FAST)
    cmd, dbg = res[-1]
    assert dbg["phase"] == "push" and 0.3 < cmd.vx <= FAST["contact_vx"] + 0.02 * 0.35 / 0.35 + 1e-6
    assert b._pace(Pose(-c - 0.5, 0, 0), (0.0, 0.0), Cmd(0.75, 0, 0), FAST).vx == 0.75  # outside the zone
    assert b._pace(Pose(-c - 0.1, 0, math.pi), (0.0, 0.0), Cmd(0.75, 0, 0), FAST).vx == 0.75  # ball behind


def test_decel_brakes_faster_than_accel():
    q = {**P, "max_vx": 0.75, "accel_vx": 1.0, "decel_vx": 3.0, "accel_wz": 4.0, "decel_wz": 2.0, "max_wz": 2.0}
    assert abs(shape(Cmd(0.3, 0, 0), Cmd(0.7, 0, 0), 0.1, q).vx - 0.4) < 1e-9
    assert abs(shape(Cmd(0.75, 0, 0), Cmd(0.3, 0, 0), 0.1, q).vx - 0.4) < 1e-9
    assert abs(shape(Cmd(0.1, 0, 2.0), Cmd(0.1, 0, 0.5), 0.1, q).wz - 0.9) < 1e-9  # turning harder: accel_wz
    assert abs(shape(Cmd(0.1, 0, -2.0), Cmd(0.1, 0, 0.5), 0.1, q).wz - 0.3) < 1e-9  # reversing: decel_wz


def test_goalie_speed_cap():
    b = Brain(BLUE, F)
    q = {**FAST, "role_mode": "defend", "k_goalie": 6.0, "goalie_vmax": 0.45}
    gx = -F.length / 2 + q["goalie_dist"]
    for cmd, d in run(b, lambda t: world(t, (gx, 0.1, math.pi / 2), (0.3, 0.9)), 30, q):
        assert d["phase"] == "guard" and cmd.vx <= 0.45 + 1e-9
    assert cmd.vx > 0.44  # k_goalie * 0.255 m wants 1.5 m/s
