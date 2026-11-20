import math

import pytest

from final_project.pupper_fc.controller import Controller
from final_project.pupper_fc.model import Ball, Field, Limits, Observation, Pose
from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M, PawApproach, paw_pose
from final_project.pupper_fc.simulation import Simulator


@pytest.mark.parametrize("side,lateral", [("left", .040), ("right", -.046)])
def test_rotated_field_pose_puts_the_ball_on_the_requested_paw(side, lateral):
    ball = Ball(.5, -.3)
    pose = paw_pose(ball, (.5, 1.0), side)
    assert pose == Pose(ball.x + lateral, ball.y - .258, math.pi / 2)
    with pytest.raises(ValueError, match="kick_side"):
        Controller(kick_side="center")


def test_docking_assist_requires_measured_stall_and_fresh_stationary_ball():
    paw = PawApproach("left", .258)
    robot, ball = Pose(0, 0, 0), Ball(.278, .040)

    def observe(t, pose=robot, observed_ball=ball):
        return paw.docking_speed_floor(pose, observed_ball, .020, 0., t, t + .06)

    assert all(observe(i * .05) == 0. for i in range(11))
    assert observe(.56) == .075
    assert all(observe(t) == .075 for t in (.60, .65, .70))
    # An unchanged base never receives a second pulse until it actually moves.
    assert all(observe(i * .05) == 0. for i in range(15, 42))
    assert observe(2.10, Pose(.006, 0, 0)) == 0.
    for i in range(43, 56):
        assert observe(i * .05, Pose(.006, 0, 0), Ball(.278, .04, .08, 0.)) == 0.
    assert observe(4.) == 0.  # Capture gap starts a new measurement window.


@pytest.mark.parametrize("side", ["left", "right"])
def test_combined_standoff_guides_steering_and_kick_readiness(side):
    # Stand near the forward edge of the release-specific window. A right
    # foot using the common historical offset would reject this placement.
    forward = COMBINED_PAW_FORWARD_M[side]
    ball = Ball(.4, 0.)
    goal = (1., 0.)
    controller = Controller(kick_side=side, paw_forward_m=forward)
    pose = paw_pose(ball, goal, side, forward_m=forward)
    for tick in range(12):
        t = tick * .05
        obs = Observation(t, pose, ball)
        decision = controller.tick(obs, t, shot_target=goal)
    assert decision.phase == "kick_ready"
    assert decision.target == pytest.approx((ball.x - forward, pose.y))
    near_edge = Pose(pose.x - .014, pose.y, pose.yaw)
    t += .05
    assert controller.tick(Observation(t, near_edge, ball), t, shot_target=goal).command.vx == 0.


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("speed", [.2, .45])
def test_closed_loop_routes_around_ball_and_brakes_with_bounded_commands(side, speed):
    limits = Limits(vx=speed, vy=speed)
    sim = Simulator(limits=limits, scenario="wrong_side")
    controller = Controller(sim.field, limits, kick_side=side)
    initial = sim.ball
    previous = None
    speeds = []
    for _ in range(500):
        d = controller.tick(sim.observe(delay=.05), sim.time)
        cmd = d.command
        assert d.phase in ("approach", "align", "kick_ready")
        if previous is not None:
            assert math.hypot(cmd.vx - previous.vx, cmd.vy - previous.vy) <= limits.acceleration * .05 + 1e-9
            assert abs(cmd.wz - previous.wz) <= limits.yaw_acceleration * .05 + 1e-9
        previous = cmd
        speeds.append(math.hypot(cmd.vx, cmd.vy))
        assert speeds[-1] <= speed + 1e-9
        if d.phase == "kick_ready":
            break
        sim.step(cmd, .05)
    assert d.phase == "kick_ready"
    assert (cmd.vx, cmd.vy, cmd.wz) == (0, 0, 0)
    assert max(speeds) > .1
    assert max(speeds[-4:]) < .04
    assert sim.ball == initial
    assert math.hypot(d.target[0] - sim.robot.x, d.target[1] - sim.robot.y) < .025


def test_ready_requires_current_distinct_steady_measurements_and_stationary_ball():
    controller = Controller(kick_side="left")
    ball = Ball(.1, .2)
    goal = (Field().length / 2, 0.)
    pose = paw_pose(ball, goal, "left")

    def phase(capture, now, sample=ball):
        return controller.tick(Observation(capture, pose, sample), now, shot_target=goal).phase

    assert all(phase(0., now) == "align" for now in (0., .04, .08, .12, .18, .24))
    assert phase(0., .40) == "tracking_lost"
    # A true velocity report that puts the ball outside its window at swing time vetoes release.
    rolling = Ball(ball.x, ball.y, 0., .04)
    assert all(phase(t / 20, t / 20, rolling) == "align" for t in range(9, 20))
    controller.cancel()
    assert phase(1., 1.) == "align"
    assert all(phase(t / 20, t / 20) == "align" for t in range(21, 24))
    assert phase(1.21, 1.21) == "kick_ready"
    late = controller.tick(Observation(1.21, pose, ball), 1.33, shot_target=goal)
    assert late.phase == "kick_ready"
    assert late.command.expires_at == pytest.approx(1.21 + .15)
    assert phase(1.35, 1.35, rolling) == "align"


def test_pose_mode_steers_wrapped_heading_and_stops_at_the_metric_target():
    sim = Simulator()
    sim.move_robot(-.7, .4, -math.pi + .15)
    controller = Controller(sim.field, sim.limits, kick_side="right")
    target = Pose(.4, -.3, math.pi - .10)
    for _ in range(500):
        decision = controller.tick(sim.observe(delay=.06), sim.time, target_pose=target)
        if decision.phase == "target_reached":
            break
        sim.step(decision.command, .04)
    assert decision.phase == "target_reached"
    assert math.hypot(sim.robot.x - target.x, sim.robot.y - target.y) <= .06
    assert abs(math.remainder(sim.robot.yaw - target.yaw, math.tau)) <= .10


def test_continuously_replaced_field_target_preserves_ramp_but_cancel_restarts():
    ctrl = Controller(limits=Limits(vx=.45), kick_side="right")
    robot = Pose(-.6, -.1, .2)
    speeds = []
    for i in range(12):
        now = i * .04
        d = ctrl.tick(Observation(now, robot, None), now, target_pose=Pose(.4, i * .002, .3))
        speeds.append(math.hypot(d.command.vx, d.command.vy))
    assert speeds[-1] > .35
    assert all(0 <= b - a <= .04 + 1e-9 for a, b in zip(speeds, speeds[1:]))
    ctrl.cancel()
    d = ctrl.tick(Observation(.50, robot, None), .50, target_pose=Pose(.4, .024, .3))
    assert (d.command.vx, d.command.vy, d.command.wz) == (0, 0, 0)


def test_stationary_ball_with_noisy_observations_and_wrong_reported_zero_velocity():
    sim = Simulator(scenario="offset", seed=7)
    ctrl = Controller(sim.field, sim.limits, kick_side="left")
    for _ in range(240):
        d = ctrl.tick(sim.observe(delay=.06, noise=.002), sim.time)
        if d.phase == "kick_ready":
            break
        sim.step(d.command, .05)
    assert d.phase == "kick_ready"

    # Even a slow physical drift traverses the paw window during the long wind-up.
    ctrl.cancel()
    origin = Ball(.1, 0.)
    goal = (sim.field.length / 2, 0)
    pose = paw_pose(origin, goal, "left")
    for i in range(16):
        t = i * .04
        drifting = Ball(origin.x, origin.y + .025 * t, 0, 0)
        d = ctrl.tick(Observation(t, pose, drifting), t, shot_target=goal)
        assert d.phase != "kick_ready"
