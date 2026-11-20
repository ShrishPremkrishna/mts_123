"""Selected-config defensive outcomes in the existing approximate Simulator."""
from dataclasses import asdict, replace
import math

import pytest

from final_project.pupper_fc.controller import Controller, defensive_clearance
from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.model import Ball, Field, Limits, Observation, Pose
from final_project.pupper_fc.simulation import Simulator
from final_project.pupper_fc.tests.test_integration import arm, frame, intent, command, is_zero


LIMITS = Limits(vx=.3, vy=.3, wz=1)
GEOMETRY = dict(length=3.66, width=2.44, goal_width=.6096,
                robot_radius=.16, ball_radius=.11, bumper_offset=.19)


def run_defense(sign, robot_xy, ball_xy, seconds=15, ball_jitter=0):
    """Public Executor input, .05 s camera/.02 s control/.01 s model steps."""
    field = Field(**GEOMETRY, attack_sign=sign)
    sim = Simulator(field=field, limits=LIMITS)
    yaw = robot_xy[2] if len(robot_xy) == 3 else 0
    sim.move_robot(sign * robot_xy[0], robot_xy[1], yaw if sign == 1 else math.pi - yaw)
    sim.move_ball(sign * ball_xy[0], ball_xy[1])
    initial_ball = sim.ball
    ex = Executor(fields={"pupper-1": field}, limits=LIMITS)
    arm(ex)
    trace, first_contact, reached = [], None, None
    for n in range(round(seconds / .01) + 1):
        now = sim.time
        if n % 5 == 0:
            obs = sim.observe()
            reported_ball = replace(obs.ball, y=obs.ball.y + ball_jitter * (-1 if (n // 5) % 2 else 1))
            result = ex.handle(frame(now, n + 1, robots={"pupper-1":
                dict(**asdict(obs.robot), goal=obs.goal)}, ball=asdict(reported_ball)))
            assert result["accepted"], result["reason"]
        if n % 10 == 0:
            assert ex.handle(intent(now, n + 1, kind="auto"))["accepted"]
        if n % 2 == 0:
            decision = ex.tick(now)["pupper-1"]
            cmd = decision.command
            if decision.phase == "target_reached" and reached is None:
                reached = now
        if n % 5 == 0:
            trace.append(dict(now=now, robot=asdict(sim.robot), ball=asdict(sim.ball),
                              decision=asdict(decision)))
        if n == round(seconds / .01):
            break
        sim.step(cmd, .01)
        if sim.ball != initial_ball and first_contact is None:
            first_contact = sim.time
    return dict(sign=sign, initial_robot=robot_xy, initial_ball=ball_xy,
                first_contact_s=first_contact, first_target_reached_s=reached,
                final_phase=decision.phase, final_reason=decision.reason, goal=sim.goal,
                final_robot=asdict(sim.robot), final_ball=asdict(sim.ball), trace=trace)


@pytest.mark.parametrize("sign", [1, -1])
@pytest.mark.parametrize("robot,ball", [((-.5, 0), (-.9, 0)),
                                        ((-1, -.4), (-1, .55)),
                                        ((-.5, 0), (-1.25, 0)),
                                        ((-.05, 0), (-1.25, .4)),
                                        ((-.8, 1.04, -math.pi / 2), (-1.15, .65)),
                                        ((-.8, -1.04, math.pi / 2), (-1.15, -.65)),
                                        ((-.8, 1.04), (-1, .75)),
                                        ((-.8, -1.04), (-1, -.75)),
                                        ((-1.501, 0), (-1.2, 0)),
                                        ((-1.28, 0, .45), (-1, 0))])
def test_reachable_guard_converges_without_ball_contact(sign, robot, ball):
    result = run_defense(sign, robot, ball)
    assert result["first_contact_s"] is None
    assert result["first_target_reached_s"] is not None
    assert result["first_target_reached_s"] < 15
    assert result["final_phase"] == "target_reached" and result["goal"] == 0


@pytest.mark.parametrize("sign", [1, -1])
def test_small_ball_jitter_does_not_restart_guard_route(sign):
    result = run_defense(sign, (-.05, 0), (-1, .14), ball_jitter=.01)
    assert result["first_contact_s"] is None and result["first_target_reached_s"] is not None
    assert result["final_phase"] == "target_reached"


@pytest.mark.parametrize("sign", [1, -1])
@pytest.mark.parametrize("ball", [(-1.35, .08), (-1.48, 0), (-1.60, -.08)])
def test_unreachable_deep_guard_explicitly_holds_without_touching_ball(sign, ball):
    result = run_defense(sign, (-1, 0), ball, seconds=2)
    assert result["first_contact_s"] is None and result["first_target_reached_s"] is None
    assert result["final_phase"] == "tactical_hold"
    assert "no ball-clear goalward guard pose" in result["final_reason"]
    assert all(is_zero(row["decision"]["command"]) for row in result["trace"])


def test_explicit_defend_requires_ball_while_move_and_goal_return_do_not():
    ex = Executor(limits=LIMITS)
    arm(ex)
    ex.handle(frame(ball=None))
    target = dict(x=-1.48, y=0, yaw=0)
    defended = ex.handle(intent(kind="defend", target=target))
    assert defended["decisions"]["pupper-1"]["phase"] == "tracking_lost"
    assert is_zero(command(defended))
    moved = ex.handle(intent(.01, 2, kind="move", target=target))
    assert moved["decisions"]["pupper-1"]["phase"] == "move"
    assert not is_zero(command(moved))
    ex.handle(intent(.02, 3, kind="defend", target=target))
    returned = ex.handle(frame(.03, 2, ball=None,
        robots={"pupper-1": dict(x=0, y=0, yaw=0, goal=1)}))
    assert returned["decisions"]["pupper-1"]["phase"] == "return_home"


def test_guard_waypoint_survives_target_yaw_updates_and_each_segment_is_clear():
    field = Field(**GEOMETRY)
    ctrl = Controller(field, LIMITS)
    obs = Observation(0, Pose(-.2, 0, 0), Ball(-.9, 0))
    target = Pose(-1.48, 0, 0)
    first = ctrl.tick(obs, 0, target_pose=target, preserve_target_heading=True, avoid_ball=True)
    waypoint = ctrl._guard_waypoint
    assert waypoint is not None and "circling" in first.reason
    following = ctrl.tick(replace(obs, captured_at=.05), .05,
                          target_pose=replace(target, yaw=.01), preserve_target_heading=True, avoid_ball=True)
    assert ctrl._guard_waypoint == waypoint and following.target == first.target
    assert following.target_heading == .01
    from final_project.pupper_fc.controller import _segment_distance
    assert _segment_distance(obs.robot.x, obs.robot.y, *first.target, obs.ball.x, obs.ball.y) >= defensive_clearance(field)


@pytest.mark.parametrize("options", [{"avoid_ball": 1}, {"avoid_ball": True}])
def test_avoidance_requires_boolean_and_pose(options):
    result = Controller().tick(Observation(0, Pose(-1, 0, 0), Ball(0, 0)), 0, **options)
    assert result.phase == "invalid_intent" and is_zero(asdict(result.command))
