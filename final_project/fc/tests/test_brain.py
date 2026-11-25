"""Brain unit tests: geometry, roles, safety, shaping. Run from final_project/: pytest fc/tests -q"""
import math

from fc.brain import Brain, DEFAULTS, dubins_points, dubins_words, seg_dist, shape, to_body, unit
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


def test_push_max_heading_off_by_default_pushes_sideways_as_before():
    assert DEFAULTS["push_max_heading_deg"] == 0
    cmd, dbg = run(Brain(BLUE, F), lambda t: world(t, (-0.30, 0.0, math.pi / 2), (0.0, 0.0)), 10)[-1]
    assert dbg["phase"] == "push"


def test_push_max_heading_squares_up_and_backs_off_before_pushing():
    q = {**P, "push_max_heading_deg": 45}
    cmd, dbg = run(Brain(BLUE, F), lambda t: world(t, (-0.40, 0.0, math.pi / 2), (0.0, 0.0)), 10, q)[-1]
    assert dbg["phase"] == "square" and cmd.vx == 0.0 and cmd.wz < -0.3  # turn right toward +x, no walking
    # body within clearance and the ball ahead of it: step back while turning
    cmd, dbg = run(Brain(BLUE, F), lambda t: world(t, (-0.26, 0.0, math.radians(60)), (0.0, 0.0)), 10, q)[-1]
    assert dbg["phase"] == "square" and cmd.vx < 0.0 and cmd.wz < 0.0
    # within the limit: push as before
    cmd, dbg = run(Brain(BLUE, F), lambda t: world(t, (-0.30, 0.0, math.radians(30)), (0.0, 0.0)), 10, q)[-1]
    assert dbg["phase"] == "push"


def test_push_side_off_by_default_is_centre_line():
    assert DEFAULTS["push_side_m"] == 0
    b = Brain(BLUE, F)
    cmd, dbg = run(b, lambda t: world(t, (-0.30, 0.05, 0.0), (0.0, 0.0)), 10)[-1]
    assert "side" not in dbg and b.push_side == 0.0 and abs(dbg["lat"] - 0.05) < 1e-6


def test_push_side_auto_picks_current_side_offsets_target_and_latches():
    q = {**P, "push_side_m": 0.06, "push_side": "auto"}
    b = Brain(BLUE, F)  # attacks +x: n = +y, robot at y=+0.03 is left of the shot line (ball on its right)
    cmd, dbg = run(b, lambda t: world(t, (-0.30, 0.03, 0.0), (0.0, 0.0)), 10, q)[-1]
    assert b.push_side == 1.0 and dbg["side"] == 0.06
    assert abs(dbg["lat"] - (0.03 - 0.06)) < 1e-6  # lateral error measured from the offset line
    assert abs(dbg["target"][1] - 0.06) < 1e-6 and dbg["phase"] == "push"
    # sitting exactly on the offset line: no lateral slide, keeps pushing
    cmd, dbg = run(b, lambda t: world(t, (-0.25, 0.06, 0.0), (0.0, 0.0)), 5, q)[-1]
    assert dbg["phase"] == "push" and abs(dbg["lat"]) < 1e-6 and abs(cmd.vy) < 1e-6
    # hysteresis: drifting across the centre line mid-push (still within 2*tol of the offset line) keeps the side
    cmd, dbg = run(b, lambda t: world(t, (-0.25, -0.05, 0.0), (0.0, 0.0)), 5, q)[-1]
    assert b.push_side == 1.0 and dbg["phase"] == "push"
    # ball lost resets; the next approach from the other side picks the other side
    b.decide(world(10.0, (-0.3, -0.05, 0.0), None), q)
    assert b.push_side == 0.0
    cmd, dbg = run(b, lambda t: world(20 + t, (-0.30, -0.03, 0.0), (0.0, 0.0)), 5, q)[-1]
    assert b.push_side == -1.0 and abs(dbg["target"][1] + 0.06) < 1e-6


def test_push_side_auto_on_line_uses_heading_and_fixed_sides():
    q = {**P, "push_side_m": 0.06}
    b = Brain(BLUE, F)
    b.decide(world(0.0, (-0.30, 0.0, 0.2), (0.0, 0.0)), q)  # facing left of +x: drifts to +y
    assert b.push_side == 1.0
    b = Brain(BLUE, F)
    b.decide(world(0.0, (-0.30, 0.0, -0.2), (0.0, 0.0)), q)
    assert b.push_side == -1.0
    for side, want in (("left", -1.0), ("right", 1.0)):
        b = Brain(BLUE, F)
        b.decide(world(0.0, (-0.30, 0.04, 0.0), (0.0, 0.0)), {**q, "push_side": side})
        assert b.push_side == want


def test_push_side_route_phase_unlatches():
    q = {**P, "push_side_m": 0.06}
    b = Brain(BLUE, F)
    b.decide(world(0.0, (-0.30, 0.03, 0.0), (0.0, 0.0)), q)
    assert b.push_side == 1.0
    cmd, dbg = run(b, lambda t: world(1 + t, (0.4, 0.0, math.pi), (0.0, 0.0)), 3, q)[-1]
    assert dbg["phase"] == "route" and b.push_side == 0.0


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


def mworld(t, me, ball, bv=(0.0, 0.0)):
    """One robot (blue) and a ball with a velocity."""
    return World(t, Ball(ball[0], ball[1], bv[0], bv[1], t), {"blue": RobotObs("blue", Pose(*me), t)})


def test_solo_defends_a_ball_rolling_at_goal_and_attacks_otherwise():
    q = {**P, "solo_hold_s": 1.0}
    b = Brain(BLUE, F)
    me = (0.6, 0.0, 0.0)  # upfield, facing the attack goal
    _, d = b.decide(mworld(0.0, me, (0.3, 0.0)), q)
    assert d["role"] == "attack"  # resting ball in midfield
    _, d = b.decide(mworld(1.1, me, (0.3, 0.0), (-1.5, 0.05)), q)
    assert d["role"] == "defend"  # reaches our goal line well inside the horizon
    _, d = b.decide(mworld(1.5, me, (-0.2, 0.0)), q)
    assert d["role"] == "defend"  # ball stopped, but the hold has not passed
    _, d = b.decide(mworld(2.2, me, (-0.2, 0.0)), q)
    assert d["role"] == "attack"
    for bv in ((0.8, 0.0), (-0.6, 0.9)):  # rolling away; rolling toward our end but well wide of the mouth
        b = Brain(BLUE, F)
        _, d = b.decide(mworld(0.0, me, (0.3, 0.0), bv), q)
        assert d["role"] == "attack", bv
    b = Brain(BLUE, F)  # slow but moving inside our third: defend
    _, d = b.decide(mworld(0.0, me, (-0.9, 0.3), (-0.2, 0.0)), q)
    assert d["role"] == "defend"


def test_solo_roles_off_always_attacks():
    b = Brain(BLUE, F)
    _, d = b.decide(mworld(0.0, (0.6, 0.0, 0.0), (0.3, 0.0), (-1.5, 0.0)), {**P, "solo_roles": 0})
    assert d["role"] == "attack"


def test_ball_prediction_decelerates_and_stops():
    b = Brain(BLUE, F)
    q = {**P, "ball_decel": 0.5}
    assert b._ball_at((0.0, 0.0), (1.0, 0.0), 1.0, q)[0] == 0.75
    assert b._ball_at((0.0, 0.0), (1.0, 0.0), 9.0, q)[0] == 1.0  # stopped after 2 s, 1 m
    T, y = b._arrival((0.0, 0.0), (-1.0, 0.0), -0.75, q)
    assert abs(T - 1.0) < 1e-9 and y == 0.0
    assert b._arrival((0.0, 0.0), (-1.0, 0.0), -1.2, q)[0] == math.inf  # stops 1 m out


def test_attacker_aims_where_a_moving_ball_will_be():
    q = {**P, "intercept_max_s": 2.0}
    b = Brain(BLUE, F)
    _, d = b.decide(mworld(0.0, (-0.8, -0.3, 0.0), (0.0, -0.6), (0.0, 0.7)), q)
    assert d["role"] == "attack" and d["intercept"][1] > -0.6 + 0.3  # well ahead of the ball, capped by its stop
    assert d["intercept"][1] <= -0.6 + 0.7 ** 2 / (2 * q["ball_decel"]) + 1e-6
    b = Brain(BLUE, F)
    _, d = b.decide(mworld(0.0, (-0.8, -0.3, 0.0), (0.0, -0.6), (0.0, 0.05)), q)
    assert "intercept" not in d  # slow ball: aim at the ball
    b = Brain(BLUE, F)
    _, d = b.decide(mworld(0.0, (-0.35, 0.0, 0.0), (0.0, 0.0), (0.5, 0.0)), q)
    assert "intercept" not in d  # at the ball (dribbling): play the ball itself
    b = Brain(BLUE, F)
    _, d = b.decide(mworld(0.0, (-0.8, -0.3, 0.0), (0.0, -0.6), (0.0, 0.7)), {**q, "intercept_max_s": 0})
    assert "intercept" not in d


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


# ---------------------------------------------------------------- plan mode, coast, fast-gait heading bias
PL = {**P, "plan": 1, "max_vx": 0.75, "push_vx": 0.75, "contact_vx": 0.3, "contact_slow_m": 0.5, "max_wz": 2.0}


def test_dubins_paths_reach_the_goal_pose():
    q0, q1 = (0.1, -0.2, 2.5), (-0.6, 0.4, -0.3)
    words = dubins_words(q0, q1, 0.3)
    assert words and sum(words[0][1]) <= sum(words[-1][1])
    for word, seg in words:
        pts = dubins_points(q0, word, seg, 0.3)
        assert math.hypot(pts[-1][0] - q1[0], pts[-1][1] - q1[1]) < 0.01
        hx, hy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
        assert abs(math.atan2(math.sin(math.atan2(hy, hx) - q1[2]), math.cos(math.atan2(hy, hx) - q1[2]))) < 0.2


def test_plan_off_by_default():
    assert DEFAULTS["plan"] == 0 and DEFAULTS["coast_s"] == 0 and DEFAULTS["fast_yaw_bias_deg"] == 0
    _, d = Brain(BLUE, F).decide(mworld(0.0, (0.6, 0.3, 0.0), (0.0, 0.0)), P)
    assert d["phase"] == "route" and "plan" not in d


def test_plan_from_the_wrong_side_goes_round_and_ends_on_the_shot_line():
    b = Brain(BLUE, F)
    _, d = b.decide(mworld(0.0, (0.6, 0.1, math.pi), (0.0, 0.0)), PL)
    assert d["phase"] in ("plan", "plan_turn") and b.replans == 1
    pts = b.plan["pts"]
    c = F.ball_radius + F.robot_radius
    clear = c + PL["clear_margin"]
    for x, y in pts:  # off the shot line (the run-up and the push), clear of the ball
        assert math.hypot(x, y) >= clear - 1e-6 or x < -0.5 * c or abs(y) < 0.01
    stage = [q for q in pts if abs(q[1]) < 0.01 and q[0] < -c]
    assert stage and min(q[0] for q in stage) <= -(c + PL["plan_stage_m"]) + 0.01  # run-up along the shot line
    assert pts[-1][0] > 0.2 and abs(pts[-1][1]) < 0.01  # and on through the ball toward the east goal


def test_plan_commits_through_noise_and_replans_when_the_ball_moves():
    b = Brain(BLUE, F)
    for i in range(45):  # 1.5 s of tag noise (+-2 cm, +-3 deg) and a jittering ball (+-1 cm)
        n = (-1) ** i
        b.decide(mworld(i / 30, (-0.9 + 0.02 * n, 0.5, -0.4 + 0.05 * n), (0.0, 0.01 * n)), PL)
    assert b.replans == 1
    for i in range(45, 70):  # the ball is moved 0.4 m
        b.decide(mworld(i / 30, (-0.9, 0.5, -0.4), (0.0, -0.4)), PL)
    assert b.replans == 2 and abs(b.plan["P"][1] + 0.4) < 0.02


def test_plan_runs_to_where_a_rolling_ball_will_be():
    b = Brain(BLUE, F)
    _, d = b.decide(mworld(0.0, (-0.9, -0.5, 0.5), (-0.2, 0.6), (0.0, -0.6)), PL)
    P = d["intercept"]
    assert P[1] < 0.6 - 0.2 and abs(P[0] + 0.2) < 0.02  # well ahead of the ball along its path
    assert 0.6 - P[1] <= 0.6 ** 2 / (2 * PL["ball_decel"]) + 1e-6  # no further than where it stops rolling


def test_plan_line_zone_slides_off_a_small_lateral_error():
    b = Brain(BLUE, F)
    cmd, d = run(b, lambda t: mworld(t, (-0.45, 0.06, 0.0), (0.0, 0.0)), 10, PL)[-1]
    assert d["phase"] in ("line", "push") and cmd.vy < -0.05 and abs(cmd.wz) < 0.3 and cmd.vx > 0.1


def test_coast_keeps_going_slower_on_a_stale_pose_then_stops():
    q = {**P, "coast_s": 0.3, "coast_vx": 0.25, "pose_stale_s": 1.0, "max_vx": 0.75, "gait_band": 0}
    b = Brain(BLUE, F)
    b.out = Cmd(0.6, 0.0, 0.8)
    outs = [b.decide(world(0.4 + i / 30, (-0.8, 0.0, 0.0), me_t=0.0), q) for i in range(12)]
    assert all(d["phase"] == "coast" for _, d in outs)
    vx = [c.vx for c, _ in outs]
    wz = [c.wz for c, _ in outs]
    assert vx[0] < 0.6 and vx[-1] < vx[0] and vx[-1] > 0.25 and wz[-1] < wz[0] < 0.8
    cmd, d = b.decide(world(1.1, (-0.8, 0.0, 0.0), me_t=0.0), q)
    assert d["phase"] == "pose_stale" and cmd.vx == 0.0
    b = Brain(BLUE, F)  # coast off: steers on the old pose as before
    _, d = b.decide(world(0.4, (-0.8, 0.0, 0.0), me_t=0.0), {**q, "coast_s": 0})
    assert d["phase"] != "coast"


def test_fast_yaw_bias_applies_only_in_the_fast_gait():
    q = {**P, "predict_self": 1, "act_lag_s": 0.0, "fast_yaw_bias_deg": -8.0}
    b = Brain(BLUE, F)
    for vx, want in ((0.3, 0.0), (0.75, -8.0)):
        b.hist = [(0.0, vx, 0.0, 0.0, True)]
        b.out = Cmd(vx, 0.0, 0.0)
        pose = b._predict(Pose(0.0, 0.0, 0.0), 0.0, 0.0, q)
        assert abs(math.degrees(pose.yaw) - want) < 1e-6
    b.hist = [(0.0, 0.75, 0.0, 0.0, True)]  # dead reckoning walks along the corrected heading
    pose = b._predict(Pose(0.0, 0.0, 0.0), 0.0, 1.0, q)
    assert pose.y < -0.09 and abs(math.degrees(pose.yaw) + 8.0) < 1e-6


def test_plan_debug_carries_the_committed_path_for_the_ui():
    b = Brain(BLUE, F)
    _, d = b.decide(mworld(0.0, (-0.9, 0.5, -0.4), (0.0, 0.0)), PL)
    pts = d["plan"]
    assert 2 <= len(pts) <= 20 and all(len(q) == 2 for q in pts)
    assert pts[-1] == [round(b.plan["pts"][-1][0], 3), round(b.plan["pts"][-1][1], 3)]


DR = {**P, "dribble": 1, "max_vx": 0.75, "max_wz": 2.0, "accel_vx": 1.2, "decel_vx": 2.0, "push_side_m": 0.03,
      "role_mode": "attack"}


def test_dribble_off_by_default_and_never_touches_other_presets():
    b = Brain(BLUE, F)
    _, d = b.decide(world(0.0, (-0.6, 0.0, 0.0), (0.0, 0.0)), P)
    assert not d["phase"].startswith("dr_")


def test_dribble_meets_a_rolling_ball_ahead_and_carries_it_near_centre():
    b = Brain(BLUE, F)
    w = World(0.0, Ball(0.0, -0.4, 0.0, 0.4, 0.0), {"blue": RobotObs("blue", Pose(-0.9, -0.4, 0.0), 0.0)})
    _, d = b.decide(w, DR)
    assert d["intercept"][1] > -0.3  # aims where the ball will be, not where it is
    b = Brain(BLUE, F)  # behind the ball on the line: carry, ~3 cm bias, gentle turning
    res = run(b, lambda t: world(t, (-0.29, 0.0, 0.0), (0.0, 0.0)), 30, DR)
    cmd, d = res[-1]
    assert d["phase"] == "dr_carry" and abs(d["side"]) == 0.03 and cmd.vx > 0.1 and abs(cmd.wz) < 0.5


def test_dribble_turning_is_bounded_by_lateral_acceleration():
    b = Brain(BLUE, F)
    b.out = Cmd(0.7, 0.0, 0.0)
    for i in range(10):
        cmd, _ = b.decide(world(i / 30, (-1.0, 0.5, -0.6), (0.3, 0.0)), DR)
        v = max(cmd.vx, 0.2)
        assert abs(cmd.wz) <= DR.get("dribble_alat", DEFAULTS["dribble_alat"]) / v + 0.15 or cmd.vx < 0.05


def test_dribble_snaps_to_a_deflected_ball():
    b = Brain(BLUE, F)
    mk = lambda t, vx, vy: World(t, Ball(0.0, 0.0, vx, vy, t), {"blue": RobotObs("blue", Pose(-0.4, 0.0, 0.0), t)})
    for i in range(5):
        b.decide(mk(i / 30, 0.4, 0.0), DR)
    _, d = b.decide(mk(6 / 30, 0.0, 0.5), DR)  # knocked sideways
    assert d.get("deflect") == 1 and b.deflects == 1 and d["intercept"][1] > 0.1
