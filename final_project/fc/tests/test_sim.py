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
