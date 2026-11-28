"""Sim tests: determinism, goals, walls, error model plumbing. Run from final_project/: pytest fc/tests -q"""
import math

from fc.core import Cmd, Field, RobotCfg
from fc.sim import SIM_DEFAULTS, Sim

F = Field(length=2.63, width=1.93, goal_width=0.61, goal_depth=0.30, ball_radius=0.09, robot_radius=0.18)
ROBOTS = [RobotCfg("blue", "blue", 10, "", "stock"), RobotCfg("red", "red", 11, "", "stock")]
OFF = {"sim_errors": 0}


def drive(sim, cmds, seconds, dt=1 / 30):
    for _ in range(int(round(seconds / dt))):
        sim.step(cmds, dt)


def test_deterministic_per_seed():
    a, b, c = Sim(F, ROBOTS, {}, 7), Sim(F, ROBOTS, {}, 7), Sim(F, ROBOTS, {}, 8)
    cmds = {"blue": Cmd(0.2, 0.05, 0.3), "red": Cmd(0.1, -0.05, -0.2)}
    for s in (a, b, c):
        s.reset("random")
        drive(s, cmds, 3.0)
    assert a.truth() == b.truth() and a.observe() == b.observe()
    assert a.truth() != c.truth()


def test_scenarios():
    s = Sim(F, ROBOTS, {}, 1)
    for sc in ("kickoff", "random"):
        s.reset(sc)
        assert set(s.truth()["robots"]) == {"blue", "red"}
    s.reset("attack_solo")
    assert list(s.truth()["robots"]) == ["blue"]
    s.reset("shot_on_goal")
    t = s.truth()
    assert t["ball"]["vx"] < 0  # blue's goal is west


def test_goal_detection_and_kickoff_reset():
    s = Sim(F, ROBOTS, OFF, 0)
    s.reset("shot_on_goal", robots=[], team="red")  # empty net, ball rolling east
    drive(s, {}, 6.0)
    goals = [e for e in s.events if e["type"] == "goal"]
    assert goals and goals[0]["goal"] == "east" and goals[0]["team"] == "blue"
    assert s.score["blue"] == 1
    tb = s.truth()["ball"]
    assert math.hypot(tb["x"], tb["y"]) < 0.1  # auto reset to kickoff after the pause


def test_keep_ball_after_goal_rests_in_net_or_bounces_back():
    """Autoplay: no kickoff reset after a goal; the ball stays in the net (unreachable) until a person rolls it out,
    or bounces back into the field."""
    seen = set()
    for bounce in (0.0, 1.0):
        s = Sim(F, ROBOTS, {**OFF, "sim_goal_bounce": bounce}, 0)
        s.keep_ball = True
        s.reset("shot_on_goal", robots=[], team="red")
        drive(s, {}, 8.0)
        goals = [e for e in s.events if e["type"] == "goal"]
        tb = s.truth()["ball"]
        assert len(goals) == 1 and not s.truth()["paused"]
        if bounce:
            assert tb["x"] < F.length / 2 and not s.netted                 # back in the field
        else:
            assert tb["x"] > F.length / 2 and s.netted and tb["vx"] == 0   # resting in the net
            assert s.kick(1.0, 0.0, -0.5, 0.0) and not s.netted            # a person rolls it out
            drive(s, {}, 1.0)
            assert s.truth()["ball"]["x"] < 1.0
        seen.add(bounce)
    assert seen == {0.0, 1.0}


def test_ball_bounces_off_sideline_not_goal_mouth_edges():
    s = Sim(F, ROBOTS, OFF, 0)
    s.reset("kickoff", robots=[])
    s.ball = [0.0, 0.5, 0.0, 0.8]
    drive(s, {}, 1.0)
    assert s.truth()["ball"]["vy"] < 0 and abs(s.truth()["ball"]["y"]) <= F.width / 2
    s.ball = [1.0, 0.6, 0.8, 0.0]  # end line outside the mouth: bounce, no goal
    drive(s, {}, 1.5)
    assert not [e for e in s.events if e["type"] == "goal"] and s.truth()["ball"]["vx"] <= 0


def test_robot_pushes_ball():
    s = Sim(F, ROBOTS, OFF, 0)
    s.reset("kickoff", robots=["blue"])
    b = s.bots["blue"]
    b.x, b.y, b.yaw = -0.4, 0.0, 0.0
    s.ball = [0.0, 0.0, 0.0, 0.0]
    drive(s, {"blue": Cmd(0.3, 0, 0)}, 2.0)
    assert s.truth()["ball"]["x"] > 0.2


def test_command_gains_and_lag():
    on = Sim(F, ROBOTS, {"sim_drift_deg_per_m": 0, "sim_yaw_walk_deg": 0}, 0)
    on.reset("kickoff", robots=["blue"])
    b = on.bots["blue"]
    b.x, b.y, b.yaw = -1.0, 0.0, 0.0
    on.ball = [1.0, 0.8, 0.0, 0.0]
    drive(on, {"blue": Cmd(0.2, 0, 0)}, 3.0)
    expected = SIM_DEFAULTS["sim_gain_vx"] * 0.2 * (3.0 - SIM_DEFAULTS["sim_lag_s"])
    assert abs((b.x + 1.0) - expected) < 0.02


def test_observation_latency_and_errors_off_exact():
    s = Sim(F, ROBOTS, OFF, 0)
    s.reset("kickoff")
    drive(s, {"blue": Cmd(0.2, 0, 0.1)}, 1.0)
    w, tr = s.observe(), s.truth()
    assert abs(w.robots["blue"].pose.x - tr["robots"]["blue"]["x"]) < 0.01  # at most one camera frame old
    e = Sim(F, ROBOTS, {}, 0)
    e.reset("kickoff")
    drive(e, {}, 1.0)
    w = e.observe(now=100.0)  # caller clock: capture times shift consistently
    assert w.t == 100.0
    for o in w.robots.values():
        assert w.t - o.t >= SIM_DEFAULTS["sim_latency_s"] - 1e-6


def test_far_tags_drop_more():
    s = Sim(F, ROBOTS, {"sim_burst_per_s": 0}, 3)
    s.reset("kickoff")
    for b, y in zip(s.bots.values(), (0.8, -0.8)):  # blue near the camera (north), red far
        b.y = y
    seen = {"blue": 0, "red": 0}
    for _ in range(600):
        s._capture()
        _, _, robots = s.frames.pop()
        for k in robots:
            seen[k] += 1
    assert seen["blue"] > seen["red"]


def test_ball_rattling_in_corner_counts_as_trapped():
    s = Sim(F, ROBOTS, {**OFF, "sim_trap_s": 2.0}, 0)
    s.reset("kickoff", robots=["blue"])
    b = s.bots["blue"]
    cx, cy = -F.length / 2 + F.ball_radius, F.width / 2 - F.ball_radius
    b.x, b.y = cx + 0.2, cy - 0.2
    for i in range(90):  # 3 s of the ball jiggling 7 cm back and forth next to the robot
        s.ball = [cx + (0.07 if i % 2 else 0.0), cy, 0.0, 0.0]
        drive(s, {}, 1 / 30)
        if any(e["type"] == "trapped" for e in s.events):
            break
    assert any(e["type"] == "trapped" for e in s.events)


def test_speed_curve_matches_the_measured_walker():
    from fc.sim import vx_actual
    p = SIM_DEFAULTS
    assert abs(vx_actual(0.2, p) - 0.92 * 0.2) < 1e-9  # floor gain in the slow trot
    assert abs(vx_actual(0.49, p) - 0.36) < 1e-6        # trot saturates below the switch
    assert vx_actual(0.5, p) > 0.5                      # fast gait from 0.5
    assert abs(vx_actual(0.75, p) - 0.73) < 1e-6 and abs(vx_actual(1.0, p) - 0.86) < 1e-6
    assert vx_actual(2.0, p) == vx_actual(1.5, p)       # policy clip
    assert vx_actual(-0.5, p) == -0.92 * 0.3            # backward cap
    assert vx_actual(0.75, {**p, "sim_speed_curve": 0}) == 0.92 * 0.75


def test_fast_run_brakes_in_about_11_cm_and_turns_clip_at_2():
    s = Sim(F, ROBOTS, {"sim_drift_deg_per_m": 0, "sim_yaw_walk_deg": 0, "sim_fast_wobble": 0, "sim_gait_kick": 0}, 0)
    s.reset("kickoff", robots=["blue"])
    b = s.bots["blue"]
    b.x, b.y, b.yaw = -1.2, 0.0, 0.0
    s.ball = [1.0, 0.8, 0.0, 0.0]
    drive(s, {"blue": Cmd(0.75, 0, 0)}, 2.0)
    assert abs(b.vx - 0.73) < 0.01 and b.switches == 1
    x0 = b.x
    drive(s, {"blue": Cmd(0, 0, 0)}, 1.5)
    assert 0.08 < b.x - x0 < 0.13 and b.switches == 2
    drive(s, {"blue": Cmd(0, 0, 3.0)}, 1.0)
    assert abs(b.wz - SIM_DEFAULTS["sim_gain_wz"] * 2.0) < 0.01


def test_contacts_record_robot_speed_at_first_touch():
    s = Sim(F, ROBOTS, OFF, 0)
    s.reset("kickoff", robots=["blue"])
    b = s.bots["blue"]
    b.x, b.y, b.yaw = -0.6, 0.0, 0.0
    s.ball = [0.0, 0.0, 0.0, 0.0]
    drive(s, {"blue": Cmd(0.3, 0, 0)}, 3.0)
    assert s.contacts and s.contacts[0][1] == "blue" and abs(s.contacts[0][2] - 0.3) < 0.01


def test_shoot_reaches_target_at_end_speed():
    s = Sim(F, ROBOTS, OFF, 0)
    s.reset("kickoff", robots=[])
    assert s.shoot(-0.8, 0.3, 0.2, x=0.4, y=-0.1)
    assert s.events[-1]["type"] == "kick"
    best = (9.0, 0.0)
    for _ in range(400):
        s.step({}, 0.01)
        d = math.hypot(s.ball[0] + 0.8, s.ball[1] - 0.3)
        if d < best[0]:
            best = (d, math.hypot(s.ball[2], s.ball[3]))
    assert best[0] < 0.01 and abs(best[1] - 0.2) < 0.02


def test_return_and_rolling_ball_scenarios():
    s = Sim(F, ROBOTS, OFF, 3)
    s.reset("return")
    t = s.truth()
    assert list(t["robots"]) == ["blue"] and t["robots"]["blue"]["x"] > 0  # upfield in blue's attack half
    assert t["ball"]["vx"] == 0 and s.scheduled
    drive(s, {}, 4.5)
    k = [e for e in s.events if e["type"] == "kick"]
    assert len(k) == 1 and k[0]["vx"] < 0  # the human shot at blue's (west) goal
    s.reset("rolling_ball")
    t = s.truth()
    assert t["robots"]["blue"]["x"] < 0 and abs(t["ball"]["vy"]) > abs(t["ball"]["vx"])


def test_sim_eval_new_scenarios_smoke():
    from fc import sim_eval as se
    p = {**OFF, "max_vx": 0.75, "push_vx": 0.75, "contact_vx": 0.4}
    h = se.human_solo(F, ROBOTS, p, 1, minutes=0.25)
    assert h["ev"] and {"scored", "conceded", "flips_pm"} <= set(h)
    r = se.return_test(F, ROBOTS, p, 3, forced=True)
    assert r is None or r["arrive"] > 0
    m = se.moving_attack(F, ROBOTS, p, 4)
    assert set(m) == {"hit", "toward", "goal"}


def test_fast_gait_walks_off_its_heading_and_bursts_have_a_length_range():
    from fc.sim import OFF as NO_ERRORS
    q = {**NO_ERRORS, "sim_fast_dir_deg": -8.0, "sim_speed_curve": 1}  # errors on, all but this one zeroed
    s = Sim(F, ROBOTS, q, 0)
    s.reset("kickoff", robots=["blue"])
    b = s.bots["blue"]
    b.x, b.y, b.yaw = -0.8, 0.0, 0.0
    drive(s, {"blue": Cmd(0.75, 0.0, 0.0)}, 1.0)
    assert abs(b.yaw) < 1e-9 and math.degrees(math.atan2(b.y, b.x + 0.8)) < -7.0  # walks right of its heading
    b.x, b.y = -0.8, 0.0
    drive(s, {"blue": Cmd(0.3, 0.0, 0.0)}, 1.0)
    assert abs(b.y) < 1e-9  # the trot walks straight
    s = Sim(F, ROBOTS, {"sim_burst_per_s": 30.0, "sim_burst_s": 0.3, "sim_burst_s_max": 1.5}, 2)
    s.reset("kickoff")
    lens = set()
    for _ in range(40):
        s._capture()
        lens.add(round(s.bots["blue"].burst_until - s.t, 2))
    assert min(lens) >= 0.3 - 1e-9 and max(lens) <= 1.5 + 1e-9 and len(lens) > 5


def test_sim_eval_solo_reports_jitter_and_floor_knobs_exist():
    from fc import sim_eval as se
    assert set(se.FLOOR) <= set(SIM_DEFAULTS)
    p = {**OFF, "max_vx": 0.75}
    t, own, st = se.solo(F, ROBOTS, p, 3, limit=10.0)
    assert {"rev", "fast", "sq", "n", "speed", "touch", "min", "align", "talign", "near_vy", "shot"} <= set(st)
    assert st["n"] > 0


def test_hide_blinds_the_camera_and_vs_human_smoke():
    s = Sim(F, ROBOTS, OFF, 0)
    s.reset("kickoff")
    s.hide["blue"], s.hide["ball"] = s.t + 0.5, s.t + 0.5
    t0 = s.t
    drive(s, {}, 0.3)
    w = s.observe()
    assert w.robots["blue"].t <= t0 + 1e-9 and w.ball.t <= t0 + 1e-9 and w.robots["red"].t > t0
    from fc import sim_eval as se
    p = {**OFF, "max_vx": 0.75}
    h = se.vs_human(F, ROBOTS, p, 1, minutes=0.25)
    assert {"scored", "conceded", "flips_pm", "rev_pm", "exposed", "occ_stall", "rec", "touch_pm"} <= set(h)
    assert h["touch_pm"] > 0


def test_fence_contacts_and_tips_and_wall_balls_scenario():
    s = Sim(F, ROBOTS, OFF, 0)
    s.reset("attack_solo")
    b = s.bots["blue"]
    b.x, b.y, b.yaw = 0.0, F.width / 2 - 0.4, math.pi / 2
    s.ball = [1.0, -0.5, 0.0, 0.0]
    drive(s, {"blue": Cmd(0.5, 0.0, 0.0)}, 1.5)  # walks into the north fence
    ev = [e["type"] for e in s.events]
    assert ev.count("wall") == 1 and ev.count("tip") == 1
    s2 = Sim(F, ROBOTS, OFF, 0)
    s2.reset("attack_solo")
    b = s2.bots["blue"]
    b.x, b.y, b.yaw = 0.0, F.width / 2 - 0.4, 0.0
    drive(s2, {"blue": Cmd(0.0, 0.0, 0.0)}, 0.5)  # standing in the open: nothing
    assert not [e for e in s2.events if e["type"] in ("wall", "tip")]
    for seed in range(20):
        s3 = Sim(F, ROBOTS, {}, seed)
        s3.reset("wall_balls")
        bx, by = s3.ball[0], s3.ball[1]
        assert list(s3.bots) == ["blue"]
        assert min(F.length / 2 - abs(bx), F.width / 2 - abs(by)) < 0.26


def test_sim_eval_wall_rows_smoke():
    from fc.sim_eval import evaluate
    out = evaluate(2, True, 1.0, 1, {}, scen=("wall",))
    assert "wall_balls: fence contacts / tips / traps" in out


def test_sim_eval_coached_match_smoke():
    import dataclasses

    from fc import sim_eval
    from fc.core import load_field, load_params, load_robots
    field = dataclasses.replace(load_field(), length=3.36, width=2.70)
    p = {**load_params(), **sim_eval.load_preset("play"), "sim_errors": 0}
    m = sim_eval.match(field, load_robots(), p, 1, minutes=0.5)
    assert 0.0 <= m["both"] <= 1.0 and m["tasksw_pm"] > 0 and "poss_pm" in m
