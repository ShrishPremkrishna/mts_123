import math

import pytest

from final_project.pupper_fc import paw_approach
from final_project.pupper_fc.controller import Controller
from final_project.pupper_fc.model import Ball, Command, Field, Limits, Observation, Pose
from final_project.pupper_fc.paw_approach import PawApproach, contact_window, paw_pose


FIELD = Field(ball_radius=.11)
LIMITS = Limits(.22, .18, .65)
GOAL = (FIELD.length / 2, 0.)
FAR = Pose(-.85, -.04, 0.)


def controller():
    return Controller(FIELD, LIMITS, kick_side="left", paw_forward_m=.258)


def roll_through(ctrl, velocity, *, robot=FAR, end=(0., 0.), age=.123):
    for i in range(11):
        stamp = i * .034
        dt = (i - 10) * .034
        ball = Ball(end[0] + dt * velocity[0], end[1] + dt * velocity[1], *velocity)
        obs = Observation(stamp, robot, ball)
        decision = ctrl.tick(obs, stamp + age, shot_target=GOAL)
    return obs, decision


@pytest.mark.parametrize("ux,uy", [(1., 0.), (-.6, .8)])
def test_online_drag_from_measured_positions_and_speed_stops_without_reversing(ux, uy):
    paw = PawApproach("left")
    for i in range(11):
        t = i * .034
        speed = .40 - .7 * t
        distance = .40 * t - .35 * t*t
        ball = Ball(ux * distance, uy * distance, ux * speed, uy * speed)
        roll = paw._roll(t, ball)
    assert roll.deceleration == pytest.approx(.7)
    stop = roll.speed / roll.deceleration
    for t in (stop, stop + .1, roll.horizon):
        point = roll.position(ball, t)
        assert point.x == pytest.approx(ux * (.4**2 / (2 * .7)))
        assert point.y == pytest.approx(uy * (.4**2 / (2 * .7)))
    # The same positions with conflicting velocity reports cannot supply drag.
    paw.reset()
    for i in range(11):
        t = i * .034
        distance = .40 * t - .35 * t*t
        roll = paw._roll(t, Ball(ux * distance, uy * distance, ux * .4, uy * .4))
    assert roll.deceleration == 0
    assert roll.horizon <= .35


@pytest.mark.parametrize("vel", [(0.18, 0), (-.18, 0), (0, .18), (0, -.18)])
def test_real_attack_leads_in_each_measured_direction_for_a_distant_pass(vel):
    obs, d = roll_through(controller(), vel)
    current = paw_pose(obs.ball, GOAL, "left")
    axis = 0 if vel[0] else 1
    displacement = d.target[axis] - (current.x, current.y)[axis]
    assert displacement * vel[axis] > .005
    assert "horizon-limited" in d.reason
    assert d.phase == "approach"


def test_earliest_reachable_point_and_original_age_change_arrival():
    robot = Pose(-.3, -.04, 0.)
    early_obs, early = roll_through(controller(), (-.2, 0.), robot=robot, age=.03)
    _, delayed = roll_through(controller(), (-.2, 0.), robot=robot, age=.151)
    assert "estimated moving-ball intercept" in early.reason
    assert "estimated moving-ball intercept" in delayed.reason
    reactive = paw_pose(early_obs.ball, GOAL, "left")
    assert reactive.x > early.target[0] > delayed.target[0]
    assert delayed.target[0] > reactive.x - .2 * .34  # earlier than the full supported horizon
    assert delayed.phase != "kick_ready"


def test_eta_includes_body_axis_caps_signed_speed_yaw_and_acceleration():
    arrival = paw_approach._arrival
    robot = Pose(0., 0., 0.)
    forward, side = Pose(.4, 0., 0.), Pose(0., .4, 0.)
    at_rest = arrival(robot, forward, LIMITS, Command())
    assert arrival(robot, side, LIMITS, Command()) > at_rest
    assert arrival(robot, forward, LIMITS, Command(vx=.22)) < at_rest
    assert arrival(robot, forward, LIMITS, Command(vx=-.22)) > at_rest
    assert arrival(robot, forward, Limits(.22, .18, .65, .5), Command()) > at_rest
    assert arrival(robot, Pose(.4, 0., math.pi/2), LIMITS, Command()) > at_rest


@pytest.mark.parametrize("duration,reached", [(.6, False), (3., True)])
def test_reachable_claim_matches_control_ticks_with_proportional_braking(duration, reached):
    ctrl = controller()
    ball, robot = Ball(0., .04), Pose(-.38, -.05, .12)
    stance = paw_pose(ball, GOAL, "left")
    initial = Command(expires_at=.15)
    assert ctrl._paw._can_intercept(robot, ball, stance, LIMITS, initial, duration) == reached
    ctrl._paw.last_command, ctrl._paw.last_control_time = initial, 0.
    target = stance.x, stance.y
    for i in range(1, round(duration * 50) + 1):
        cmd = ctrl._drive("align", "", robot, target, ctrl._travel_heading(robot, target, stance.yaw),
                          LIMITS.vx, i / 50, "pupper-1").command
        yaw = robot.yaw + cmd.wz / 100
        robot = Pose(robot.x + (cmd.vx * math.cos(yaw) - cmd.vy * math.sin(yaw)) / 50,
                     robot.y + (cmd.vx * math.sin(yaw) + cmd.vy * math.cos(yaw)) / 50, robot.yaw + cmd.wz / 50)
    assert contact_window(robot, ball, stance.yaw - robot.yaw, "left", scale=.55) == reached


def test_camera_delay_longer_than_supported_horizon_does_not_choose_a_past_intercept():
    paw = PawApproach("left")
    for stamp in (0., .05, .10):
        b = Ball(stamp * .20, 0., .20, 0.)
        plan, _ = paw.intercept(FAR, b, GOAL, FIELD, LIMITS, stamp, stamp + .123)
    assert paw.roll is not None and paw.roll.horizon == pytest.approx(.10)
    assert plan is None


def test_redirect_withdraws_old_lead_and_reacquires_from_new_measurements():
    ctrl = controller()
    obs, initial = roll_through(ctrl, (0, .20))
    assert initial.target[1] > -.04 + .03
    for i in range(1, 12):
        stamp = obs.captured_at + i * .034
        b = Ball(0., -.2 * i * .034, 0., -.20)
        d = ctrl.tick(Observation(stamp, FAR, b), stamp + .123, shot_target=GOAL)
        if i == 1:
            # A longitudinal clearance offset may project sideways by a
            # fraction of a millimetre at this small angle to goal.
            assert abs(d.target[1] - paw_pose(b, GOAL, "left").y) < .001
            assert ctrl._paw.plan[0] is None
            assert "intercept" not in d.reason and "horizon-limited" not in d.reason
    assert d.target[1] < paw_pose(b, GOAL, "left").y - .03
    assert "horizon-limited" in d.reason


def test_changed_measured_position_with_lagging_filtered_velocity_withdraws_old_lead():
    ctrl = controller()
    obs, d = roll_through(ctrl, (0., .28))
    assert "horizon-limited" in d.reason
    stamp = obs.captured_at + .055
    ball = Ball(obs.ball.x, obs.ball.y - .016, 0., .28)
    change = ctrl.tick(Observation(stamp, FAR, ball), stamp + .123, shot_target=GOAL)
    assert ctrl._paw.plan[0] is None
    assert "horizon-limited" not in change.reason


def test_cache_at_camera_rate_with_repeated_control_ticks_and_gate_at_true_source_age(monkeypatch):
    ctrl = controller()
    obs, d = roll_through(ctrl, (.18, 0.), age=.123)
    calls = 0
    actual_arrival = paw_approach._arrival
    def counted(*args):
        nonlocal calls
        calls += 1
        return actual_arrival(*args)
    monkeypatch.setattr(paw_approach, "_arrival", counted)
    for age in (.143, .163, .183):
        duplicate = ctrl.tick(obs, obs.captured_at + age, shot_target=GOAL)
        assert duplicate.target == d.target and duplicate.reason == d.reason
    assert calls == 0
    redirect = (1.5, -.1)
    ctrl.tick(obs, obs.captured_at + .193, shot_target=redirect)
    assert calls > 0
    calls = 0
    t = obs.captured_at + .20
    # Exposure gap exceeded the roll-history bound. A new capture cannot borrow the old motion.
    b = Ball(.18 * .20, 0., .18, 0.)
    fresh = ctrl.tick(Observation(t, FAR, b), t + .151, shot_target=GOAL)
    assert fresh.target == pytest.approx((b.x - .258, -.04))
    assert calls == 0
    stale = ctrl.tick(Observation(t, FAR, b), t + .351, shot_target=GOAL)
    assert stale.phase == "tracking_lost"
    assert (stale.command.vx, stale.command.vy, stale.command.wz) == (0., 0., 0.)


def test_noise_boundary_and_disappearing_ball_do_not_create_a_forecast():
    ctrl = controller()
    for i in range(14):
        stamp = i * .034
        ball = Ball(.002 * (-1)**i, -.002 * (-1)**i, .18, -.12)
        d = ctrl.tick(Observation(stamp, FAR, ball), stamp + .123, shot_target=GOAL)
        actual = paw_pose(ball, GOAL, "left")
        assert d.target == pytest.approx((actual.x, actual.y))
    obs, edge = roll_through(ctrl, (.25, 0), end=(FIELD.length / 2 - FIELD.ball_radius - .005, 0.))
    assert ctrl._paw.plan[0] is None
    assert abs(edge.target[1]) + FIELD.robot_radius <= FIELD.width / 2
    lost = ctrl.tick(Observation(obs.captured_at + .034, FAR, None), obs.captured_at + .157, shot_target=GOAL)
    assert lost.phase == "tracking_lost" and lost.command.vx == 0.
    assert ctrl._paw.plan_key is None and not ctrl._paw.roll_samples


def test_crossing_ball_near_nose_requires_measured_clearance_before_sidestep():
    ctrl = controller()
    obs, d = roll_through(ctrl, (0., .20), robot=Pose(-.31, -.04, 0.))
    assert "backing clear" in d.reason
    assert d.target[0] < obs.robot.x and d.target[1] == pytest.approx(obs.robot.y)
    assert d.phase != "kick_ready"
    # A duplicate capture cannot satisfy measured dwell.
    for age in (.14, .16):
        d = ctrl.tick(obs, obs.captured_at + age, shot_target=GOAL)
        assert "backing clear" in d.reason
    t = obs.captured_at
    for i in range(1, 15):
        stamp = t + .034 * i
        b = Ball(0., 0.)
        r = paw_pose(b, GOAL, "left")
        d = ctrl.tick(Observation(stamp, r, b), stamp + .123, shot_target=GOAL)
        if i < 5:
            assert d.phase != "kick_ready"
    assert d.phase == "kick_ready"


def test_27hz_partial_age_capture_and_50hz_ticks_permit_only_fresh_settled_kicks():
    ctrl = controller()
    ball = Ball(0., .04)
    robot = paw_pose(ball, GOAL, "left")
    ready = expired = 0
    for i in range(60):
        now = .123 + i / 50
        stamp = math.floor(((now - .123) * 27) + 1e-9) / 27
        d = ctrl.tick(Observation(stamp, robot, ball), now, shot_target=GOAL)
        if now - stamp >= .150:
            assert d.phase == "align"
            expired += 1
        elif stamp >= .20:
            assert d.phase == "kick_ready"
            assert d.command.expires_at <= stamp + .150
            ready += 1
    assert ready > 15 and expired > 5
    # Loss of the source never extends a previously granted kick receipt.
    for age in (.151, .205, .34, .36):
        late = ctrl.tick(Observation(stamp, robot, ball), stamp + age, shot_target=GOAL)
        assert late.phase != "kick_ready" and late.command.vx == 0.
