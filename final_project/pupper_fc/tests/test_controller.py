"""Controller behaviour checked through observations and commands only.

The helpers below integrate commands as ideal body-frame velocities with a
ball that never moves on its own. That is enough to test geometry and phase
logic; it says nothing about contact, acceleration or real Pupper motion.
"""
import math
import random

import pytest

from final_project.pupper_fc import controller as ctl
from final_project.pupper_fc.controller import Controller
from final_project.pupper_fc.model import Ball, Field, Limits, Observation, Pose

DT = 0.05
ACTIVE = {"approach", "align", "strike"}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def step(robot, command, dt=DT):
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    return Pose(robot.x + (command.vx * c - command.vy * s) * dt,
                robot.y + (command.vx * s + command.vy * c) * dt,
                wrap(robot.yaw + command.wz * dt))


def run(controller, robot, ball, seconds, start=0.0, goal=0, until=None):
    """Closed loop; returns [(time, robot pose seen, decision)] and the final pose."""
    trace, now = [], start
    for _ in range(round(seconds / DT)):
        decision = controller.tick(Observation(now, robot, ball, goal=goal), now)
        trace.append((now, robot, decision))
        if until is not None and until(decision):
            break
        robot = step(robot, decision.command)
        now += DT
    return trace, robot


def is_zero(command):
    return command.vx == 0 and command.vy == 0 and command.wz == 0


def staged(field=Field(), ball=Ball(0.0, 0.0), yaw_offset=0.0, back=0.0):
    """Robot pose at the staging point for a ball on the field's x axis."""
    sign = field.attack_sign
    standoff = field.bumper_offset + field.ball_radius + ctl.STRIKE_GAP + back
    heading = 0.0 if sign == 1 else math.pi
    return Pose(ball.x - sign * standoff, ball.y, heading + yaw_offset)


def strike_started(controller, field=Field(), ball=Ball(0.0, 0.0)):
    """Drive an already-staged robot through the dwell; returns (time, pose)."""
    trace, _ = run(controller, staged(field, ball), ball, 2.0,
                   until=lambda d: d.phase == "strike")
    now, robot, decision = trace[-1]
    assert decision.phase == "strike"
    return now, robot


# --- approach geometry -------------------------------------------------------

@pytest.mark.parametrize("sign", [1, -1])
@pytest.mark.parametrize("start", [(0.8, 0.05), (0.8, -0.3), (0.4, 0.0), (0.3, 0.25), (0.3, 0.0), (0.29, -0.05)])
def test_goal_side_start_circles_without_touching_then_strikes_aligned(sign, start):
    field = Field(attack_sign=sign)
    ball = Ball(0.0, 0.0)
    robot = Pose(sign * start[0], start[1], 1.0)  # between the ball and the opponent goal
    trace, _ = run(Controller(field), robot, ball, 30.0, until=lambda d: d.phase == "strike")
    assert trace[-1][2].phase == "strike", "never reached a strike"

    clearance = field.robot_radius + field.ball_radius + ctl.CLEARANCE_MARGIN
    start_distance = math.hypot(*start)
    assert min(math.hypot(r.x - ball.x, r.y - ball.y) for _, r, _ in trace) >= min(clearance, start_distance) - 1e-9

    detour = [d for _, _, d in trace if "circling" in d.reason or "backing" in d.reason]
    backing = [d for d in detour if "backing" in d.reason]
    circling = [d for d in detour if "circling" in d.reason]
    assert detour and trace[0][2] is detour[0]
    assert all(d.phase == "approach" for d in detour)
    assert bool(backing) == (start_distance < ctl.SIDE_OFFSET)
    if backing:  # back straight away first, then the abeam waypoint
        assert detour[:len(backing)] == backing
        assert len({d.target for d in backing}) == 1
    # One persistent waypoint, abeam of the ball on one side of the shot line.
    assert len({d.target for d in circling}) == 1
    assert len({d.reason for d in circling}) == 1
    waypoint = circling[0].target
    assert abs(waypoint[0]) < 1e-9 and abs(waypoint[1]) == pytest.approx(ctl.SIDE_OFFSET)
    assert waypoint[1] * start[1] >= 0  # the robot's own side

    # At the first strike tick the bumper is behind the ball and on the shot line.
    _, robot, strike = trace[-1]
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    forward = (ball.x - robot.x) * c + (ball.y - robot.y) * s
    left = -(ball.x - robot.x) * s + (ball.y - robot.y) * c
    goal_heading = math.atan2(-ball.y, sign * field.length / 2 - ball.x)
    assert forward > field.bumper_offset + field.ball_radius
    assert abs(left) <= ctl.LATERAL_TOLERANCE
    assert abs(wrap(goal_heading - robot.yaw)) <= ctl.HEADING_TOLERANCE
    assert (ball.x - robot.x) * sign > 0  # robot is on its own-goal side of the ball


def test_start_behind_ball_goes_straight_to_staging_point():
    field = Field()
    ball = Ball(0.3, 0.4)
    decision = Controller(field).tick(Observation(0.0, Pose(-1.0, 0.2, 0.0), ball), 0.0)
    assert decision.phase == "approach" and "circling" not in decision.reason
    gx, gy = field.length / 2 - ball.x, -ball.y
    distance = math.hypot(gx, gy)
    standoff = field.bumper_offset + field.ball_radius + ctl.STRIKE_GAP
    assert decision.target == pytest.approx((ball.x - gx / distance * standoff,
                                             ball.y - gy / distance * standoff))


def test_robot_too_close_behind_ball_backs_away_instead_of_circling():
    ball = Ball(0.0, 0.0)
    robot = staged(back=-0.17)  # 3 cm bumper gap, still on the shot line
    decision = Controller().tick(Observation(0.0, robot, ball), 0.0)
    assert decision.phase == "align"
    assert decision.command.vx < 0 and decision.command.vy == pytest.approx(0.0, abs=1e-9)


def test_close_goal_side_robot_backs_straight_away_before_circling():
    field, ball = Field(), Ball(0.0, 0.0)
    robot = Pose(0.3, 0.0, math.pi)  # nose 11 cm from the ball, on the goal side
    controller = Controller(field)
    first = controller.tick(Observation(0.0, robot, ball), 0.0)
    assert first.phase == "approach" and "backing" in first.reason
    assert first.target == pytest.approx((ctl.BACK_OFF_RADIUS, 0.0))
    assert first.command.vx < 0 and first.command.vy == pytest.approx(0.0, abs=1e-9)
    moved = step(robot, first.command)
    assert math.hypot(moved.x, moved.y) > math.hypot(robot.x, robot.y)


def test_ball_against_wall_is_an_explicit_blocked_stop_until_the_ball_moves():
    """The near_wall sandbox scenario: no goal-directed staging point fits."""
    field = Field()
    ball = Ball(0.1, field.width / 2 - field.ball_radius)  # against the upper touchline
    controller = Controller(field)
    trace, _ = run(controller, Pose(-0.9, 0.5, 0.4), ball, 5.0)
    for _, _, decision in trace:
        assert decision.phase == "blocked" and is_zero(decision.command)
        assert "move the ball" in decision.reason and "reset" in decision.reason
    # Once the ball is playable again, normal approach resumes with no reset.
    resumed = controller.tick(Observation(5.0, Pose(-0.9, 0.5, 0.4), Ball(0.1, 0.3)), 5.0)
    assert resumed.phase == "approach"


def test_near_wall_ball_shortens_the_standoff_but_keeps_the_shot_line():
    field = Field()
    ball = Ball(0.1, 0.9)
    decision = Controller(field).tick(Observation(0.0, Pose(-0.9, 0.5, 0.4), ball), 0.0)
    assert decision.phase == "approach"
    ymax = field.width / 2 - field.robot_radius
    assert abs(decision.target[1]) <= ymax + 1e-9
    gx, gy = field.length / 2 - ball.x, -ball.y
    bx, by = ball.x - decision.target[0], ball.y - decision.target[1]
    assert gx * by - gy * bx == pytest.approx(0.0, abs=1e-9)  # on the shot line
    gap = math.hypot(bx, by) - field.bumper_offset - field.ball_radius
    assert ctl.MIN_STAGING_GAP - 1e-9 <= gap < ctl.STRIKE_GAP
    # The full standoff would have put the body through the wall.
    full_standoff = field.bumper_offset + field.ball_radius + ctl.STRIKE_GAP
    assert ball.y - gy / math.hypot(gx, gy) * full_standoff > ymax


# --- alignment gate ----------------------------------------------------------

def test_no_strike_until_heading_is_aligned():
    ball = Ball(0.0, 0.0)
    trace, _ = run(Controller(), staged(yaw_offset=0.6), ball, 5.0,
                   until=lambda d: d.phase == "strike")
    phases = [d.phase for _, _, d in trace]
    assert phases[0] == "align" and phases[-1] == "strike"
    assert set(phases[:-1]) == {"align"}
    first_aligned = next(t for t, r, _ in trace if abs(wrap(-r.yaw)) <= ctl.HEADING_TOLERANCE)
    assert trace[-1][0] - first_aligned >= ctl.ALIGN_DWELL - 1e-9


@pytest.mark.parametrize("robot", [
    staged(yaw_offset=2 * ctl.HEADING_TOLERANCE),
    Pose(staged().x, 2 * ctl.LATERAL_TOLERANCE, 0.0),
    staged(back=2 * ctl.GAP_TOLERANCE),
])
def test_misaligned_pose_held_in_place_never_strikes(robot):
    controller, ball = Controller(), Ball(0.0, 0.0)
    for i in range(100):  # pose never improves, e.g. the robot is not responding
        decision = controller.tick(Observation(i * DT, robot, ball), i * DT)
        assert decision.phase == "align"


def test_aligned_pose_waits_for_dwell_before_striking():
    controller, ball, robot = Controller(), Ball(0.0, 0.0), staged()
    first = controller.tick(Observation(0.0, robot, ball), 0.0)
    assert first.phase == "align"
    early = controller.tick(Observation(ctl.ALIGN_DWELL / 2, robot, ball), ctl.ALIGN_DWELL / 2)
    assert early.phase == "align"
    later = controller.tick(Observation(ctl.ALIGN_DWELL, robot, ball), ctl.ALIGN_DWELL)
    assert later.phase == "strike"


# --- strike, reassess, retry -------------------------------------------------

def test_strike_is_forward_only_bounded_and_followed_by_brake_then_retry():
    controller, ball, limits = Controller(), Ball(0.0, 0.0), Limits()
    trace, _ = run(controller, staged(), ball, 12.0)
    phases = [d.phase for _, _, d in trace]
    first = phases.index("strike")
    end = next(i for i in range(first, len(phases)) if phases[i] != "strike")
    strike = trace[first:end]

    for _, _, d in strike:
        assert 0 < d.command.vx <= limits.vx
        assert d.command.vy == 0 and d.command.wz == 0
    travelled = strike[-1][1].x - strike[0][1].x
    assert 0 < travelled <= ctl.STRIKE_DISTANCE + limits.vx * DT
    assert strike[-1][0] - strike[0][0] <= ctl.STRIKE_TIMEOUT

    # Brake with zero commands, then plan again because the ball is still there.
    resume = next(i for i in range(end, len(phases)) if phases[i] != "reassess")
    assert all(is_zero(d.command) for _, _, d in trace[end:resume])
    assert trace[resume][0] - trace[end][0] >= ctl.REASSESS_TIME - 1e-9
    assert phases[resume] == "approach"
    assert "strike" in phases[resume:], "missed ball was not retried"


def test_strike_times_out_when_robot_makes_no_progress():
    controller, ball = Controller(), Ball(0.0, 0.0)
    started, robot = strike_started(controller)
    phases = []
    for i in range(1, 60):
        now = started + i * DT
        phases.append(controller.tick(Observation(now, robot, ball), now).phase)
    stopped = phases.index("reassess")
    assert set(phases[:stopped]) == {"strike"}
    assert (stopped + 1) * DT <= ctl.STRIKE_TIMEOUT + DT


@pytest.mark.parametrize("change", ["ball_behind", "heading_drift"])
def test_strike_aborts_to_zero_when_it_can_no_longer_work(change):
    controller, ball = Controller(), Ball(0.0, 0.0)
    now, robot = strike_started(controller)
    now += DT
    if change == "ball_behind":
        ball = Ball(robot.x - 0.3, robot.y)
    else:
        robot = Pose(robot.x, robot.y, robot.yaw + 2 * ctl.STRIKE_ABORT_HEADING)
    decision = controller.tick(Observation(now, robot, ball), now)
    assert decision.phase == "reassess" and is_zero(decision.command)


def test_ball_jumping_sideways_during_strike_stops_the_run():
    controller = Controller()
    now, robot = strike_started(controller)
    now += DT
    sideways = Ball(robot.x + 0.5, robot.y + 2 * ctl.STRIKE_CORRIDOR)  # still ahead
    decision = controller.tick(Observation(now, robot, sideways), now)
    assert decision.phase == "reassess" and is_zero(decision.command)
    assert "corridor" in decision.reason
    # Not a blind continuation: the next plan is an approach to the new ball.
    later = controller.tick(Observation(now + 1.0, robot, sideways), now + 1.0)
    assert later.phase == "approach"


def test_struck_ball_rolling_away_ahead_keeps_the_strike_valid():
    controller = Controller()
    now, robot = strike_started(controller)
    for i in range(1, 4):
        now += DT
        rolling = Ball(robot.x + 0.4 + 0.3 * i, robot.y + 0.01 * i, vx=1.0)
        decision = controller.tick(Observation(now, robot, rolling), now)
        assert decision.phase == "strike" and decision.command.vx > 0


# --- replanning --------------------------------------------------------------

def test_ball_moved_during_alignment_replans_without_striking():
    controller, ball, robot = Controller(), Ball(0.0, 0.0), staged()
    before = controller.tick(Observation(0.0, robot, ball), 0.0)
    moved = Ball(0.6, -0.5)
    after = controller.tick(Observation(DT, robot, moved), DT)
    assert before.phase == "align"
    assert after.phase == "approach" and "replanned" in after.reason
    assert math.hypot(after.target[0] - before.target[0], after.target[1] - before.target[1]) > 0.5
    # The new staging point is behind the new ball position on its line to goal.
    field = Field()
    to_goal = (field.length / 2 - moved.x, -moved.y)
    to_ball = (moved.x - after.target[0], moved.y - after.target[1])
    assert to_goal[0] * to_ball[1] - to_goal[1] * to_ball[0] == pytest.approx(0.0, abs=1e-9)
    assert to_goal[0] * to_ball[0] + to_goal[1] * to_ball[1] > 0


def test_small_ball_shifts_keep_restarting_the_dwell():
    controller, robot = Controller(), staged()
    for i in range(60):  # ball creeps forward 6 cm per tick: never steady
        ball = Ball(0.06 * (i % 2), 0.0)
        decision = controller.tick(Observation(i * DT, robot, ball), i * DT)
        assert decision.phase != "strike"


def test_ball_moved_while_circling_picks_a_new_side_waypoint():
    controller = Controller()
    robot = Pose(0.8, 0.05, math.pi)
    first = controller.tick(Observation(0.0, robot, Ball(0.0, 0.0)), 0.0)
    second = controller.tick(Observation(DT, robot, Ball(0.0, 0.4)), DT)
    assert "left" in first.reason and first.target[1] > 0
    assert "replanned" in second.reason and "right" in second.reason
    assert second.target != first.target


# --- command contract --------------------------------------------------------

@pytest.mark.parametrize("limits", [Limits(), Limits(vx=0.1, vy=0.05, wz=0.3)])
@pytest.mark.parametrize("sign", [1, -1])
def test_every_command_is_bounded_and_expires_after_150_ms(limits, sign):
    field = Field(attack_sign=sign)
    controller = Controller(field, limits)
    rng = random.Random(123)
    phases = set()
    for i in range(4000):
        now = i * DT
        robot = Pose(rng.uniform(-1.8, 1.8), rng.uniform(-1.2, 1.2), rng.uniform(-7, 7))
        ball = Ball(rng.uniform(-1.8, 1.8), rng.uniform(-1.2, 1.2))
        if i % 25 < 15:  # hold a staged pose for 0.75 s, longer than ALIGN_DWELL
            ball = Ball(0.0, 0.0)
            robot = staged(field, ball)
        observation = Observation(now - rng.uniform(0, 0.4), robot,
                                  None if rng.random() < 0.05 else ball,
                                  fallen=rng.random() < 0.03,
                                  goal=rng.choice([0] * 199 + [rng.choice([1, -1])]))
        if rng.random() < 0.05:
            controller.reset()
        decision = controller.tick(observation, now)
        command = decision.command
        phases.add(decision.phase)
        assert abs(command.vx) <= limits.vx + 1e-12
        assert abs(command.vy) <= limits.vy + 1e-12
        assert abs(command.wz) <= limits.wz + 1e-12
        assert command.expires_at == pytest.approx(now + 0.15)
        assert command.robot_id == observation.robot_id
    assert {"approach", "align", "strike", "reassess", "blocked", "return_home",
            "tracking_lost", "recovery_required"} <= phases


def test_command_carries_the_observed_robot_id():
    observation = Observation(0.0, Pose(-1.0, 0.0, 0.0), Ball(0.0, 0.0), robot_id="pupper-7")
    assert Controller().tick(observation, 0.0).command.robot_id == "pupper-7"


# --- unsafe or unusable input ------------------------------------------------

def test_freshness_limit_is_about_350_ms():
    assert ctl.MAX_OBSERVATION_AGE == pytest.approx(0.35)
    robot, ball = Pose(-1.0, 0.0, 0.0), Ball(0.0, 0.0)
    fresh = Controller().tick(Observation(0.0, robot, ball), ctl.MAX_OBSERVATION_AGE - 0.01)
    stale = Controller().tick(Observation(0.0, robot, ball), ctl.MAX_OBSERVATION_AGE + 0.01)
    assert fresh.phase == "approach" and not is_zero(fresh.command)
    assert stale.phase == "tracking_lost" and is_zero(stale.command)


def test_frozen_camera_packet_stops_the_robot_and_recovers_when_live():
    controller = Controller()
    frozen = Observation(1.0, Pose(-1.0, 0.0, 0.0), Ball(0.0, 0.0))
    phases = {}
    for i in range(20):
        now = 1.0 + i * DT
        decision = controller.tick(frozen, now)
        phases[i] = decision.phase
        if now - 1.0 > ctl.MAX_OBSERVATION_AGE:
            assert decision.phase == "tracking_lost" and is_zero(decision.command)
            assert decision.command.expires_at == pytest.approx(now + 0.15)
    assert phases[0] == "approach" and phases[19] == "tracking_lost"
    live = controller.tick(Observation(2.0, frozen.robot, frozen.ball), 2.0)
    assert live.phase == "approach"


def test_stale_packet_cancels_a_strike_in_progress():
    controller, ball = Controller(), Ball(0.0, 0.0)
    now, robot = strike_started(controller)
    stale = controller.tick(Observation(now, robot, ball), now + 1.0)
    assert stale.phase == "tracking_lost" and is_zero(stale.command)
    resumed = controller.tick(Observation(now + 1.05, robot, ball), now + 1.05)
    assert resumed.phase != "strike"


NAN, INF = float("nan"), float("inf")
GOOD_ROBOT, GOOD_BALL = Pose(-1.0, 0.0, 0.0), Ball(0.0, 0.0)


@pytest.mark.parametrize("observation", [
    None,
    "not an observation",
    Observation(NAN, GOOD_ROBOT, GOOD_BALL),
    Observation(INF, GOOD_ROBOT, GOOD_BALL),
    Observation(None, GOOD_ROBOT, GOOD_BALL),
    Observation(1.0, Pose(NAN, 0.0, 0.0), GOOD_BALL),
    Observation(1.0, Pose(0.0, INF, 0.0), GOOD_BALL),
    Observation(1.0, Pose(0.0, 0.0, NAN), GOOD_BALL),
    Observation(1.0, Pose(0.0, 0.0, None), GOOD_BALL),
    Observation(1.0, (0.0, 0.0, 0.0), GOOD_BALL),
    Observation(1.0, GOOD_ROBOT, Ball(NAN, 0.0)),
    Observation(1.0, GOOD_ROBOT, Ball(0.0, -INF)),
    Observation(1.0, GOOD_ROBOT, Ball(0.0, 0.0, NAN, 0.0)),
    Observation(1.0, GOOD_ROBOT, Ball(0.0, 0.0, 0.0, INF)),
    Observation(1.0, GOOD_ROBOT, Ball(0.0, 0.0, 50.0, 0.0)),   # absurd ball speed
    Observation(1.0, Pose(9.0, 0.0, 0.0), GOOD_BALL),          # far outside the field
    Observation(1.0, Pose(0.0, -4.0, 0.0), GOOD_BALL),
    Observation(1.0, GOOD_ROBOT, Ball(-7.0, 0.0)),
    Observation(1.0, GOOD_ROBOT, Ball(0.0, 3.0)),
    Observation(1.0, Pose(NAN, 0.0, 0.0), None),
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, goal=2),
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, goal=NAN),
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, goal=True),        # bool is not a goal flag
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, goal=1.0),
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, goal=None),
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, fallen="false"),   # truthy string
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, fallen=1),
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, fallen=0),
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, fallen=None),
    Observation(1.0, GOOD_ROBOT, GOOD_BALL, robot_id=None),
    Observation(1.5, GOOD_ROBOT, GOOD_BALL),                   # captured in the future
])
def test_invalid_or_implausible_input_gives_zero_command(observation):
    decision = Controller().tick(observation, 1.0)
    assert decision.phase == "invalid_observation"
    assert is_zero(decision.command)
    assert decision.command.expires_at == pytest.approx(1.15)
    assert not decision.recovery_requested


@pytest.mark.parametrize("now", [NAN, INF, -INF, None])
def test_non_finite_clock_gives_an_already_expired_zero_command(now):
    decision = Controller().tick(Observation(1.0, GOOD_ROBOT, GOOD_BALL), now)
    assert decision.phase == "invalid_observation" and is_zero(decision.command)
    assert decision.command.expires_at == 0.0


@pytest.mark.parametrize("robot, ball", [(None, GOOD_BALL), (GOOD_ROBOT, None), (None, None)])
def test_missing_robot_or_ball_is_tracking_lost_with_zero_command(robot, ball):
    decision = Controller().tick(Observation(1.0, robot, ball), 1.0)
    assert decision.phase == "tracking_lost" and is_zero(decision.command)


def test_slightly_outside_the_lines_is_plausible_but_blocked():
    field = Field()
    ball = Ball(field.length / 2 + 0.05, 0.9)  # just over the goal line, outside the mouth
    decision = Controller(field).tick(Observation(1.0, GOOD_ROBOT, ball), 1.0)
    assert decision.phase == "blocked" and is_zero(decision.command)  # real ball, no shot from inside


@pytest.mark.parametrize("robot, ball", [
    (GOOD_ROBOT, GOOD_BALL),
    (None, None),
    (Pose(NAN, NAN, NAN), GOOD_BALL),  # a fallen robot may report a garbage pose
])
def test_fallen_requests_recovery_and_commands_nothing(robot, ball):
    decision = Controller().tick(Observation(1.0, robot, ball, fallen=True), 1.0)
    assert decision.phase == "recovery_required"
    assert decision.recovery_requested is True
    assert is_zero(decision.command)


def test_fall_cancels_a_strike_and_play_resumes_from_planning():
    controller, ball = Controller(), Ball(0.0, 0.0)
    now, robot = strike_started(controller)
    for i in range(1, 40):  # stays zero for as long as the robot is down
        fallen = controller.tick(Observation(now + i * DT, robot, ball, fallen=True), now + i * DT)
        assert fallen.phase == "recovery_required" and is_zero(fallen.command)
    upright = controller.tick(Observation(now + 2.0, robot, ball), now + 2.0)
    assert upright.phase in {"approach", "align"} and not upright.recovery_requested


# --- mirrored attack direction -----------------------------------------------

def test_goal_centre_follows_attack_sign():
    for sign in (1, -1):
        field = Field(attack_sign=sign)
        ball = Ball(0.2, 0.5)
        decision = Controller(field).tick(Observation(0.0, Pose(-sign * 1.2, 0.5, 0.0), ball), 0.0)
        # The staging point lies on the far side of the ball from the attacked goal.
        gx, gy = sign * field.length / 2 - ball.x, -ball.y
        bx, by = ball.x - decision.target[0], ball.y - decision.target[1]
        assert gx * by - gy * bx == pytest.approx(0.0, abs=1e-9)
        assert gx * bx + gy * by > 0


def test_field_rotated_half_a_turn_gives_identical_commands():
    """Attacking -x from a 180-degree rotated layout is the same game."""
    ball, robot = Ball(0.2, -0.4), Pose(0.9, 0.3, 2.0)
    plus, _ = run(Controller(Field(attack_sign=1)), robot, ball, 15.0)
    minus, _ = run(Controller(Field(attack_sign=-1)),
                   Pose(-robot.x, -robot.y, wrap(robot.yaw + math.pi)), Ball(-ball.x, -ball.y), 15.0)
    assert [d.phase for _, _, d in plus] == [d.phase for _, _, d in minus]
    assert {"approach", "align", "strike", "reassess"} <= {d.phase for _, _, d in plus}
    for (_, _, a), (_, _, b) in zip(plus, minus):
        assert (a.command.vx, a.command.vy, a.command.wz) == pytest.approx(
            (b.command.vx, b.command.vy, b.command.wz), abs=1e-6)
        if a.target is not None:
            assert a.target == pytest.approx((-b.target[0], -b.target[1]), abs=1e-6)


# --- goal latch --------------------------------------------------------------

@pytest.mark.parametrize("sign", [1, -1])
@pytest.mark.parametrize("goal", [1, -1])
def test_goal_latches_return_home_facing_opponent_goal_without_a_ball(sign, goal):
    field = Field(attack_sign=sign)
    controller = Controller(field)
    home = (-sign * field.length * 0.3, 0.0)
    trace, robot = run(controller, Pose(sign * 1.2, 0.6, 1.0), None, 30.0, goal=goal,
                       until=lambda d: d.phase == "await_reset")
    phases = [d.phase for _, _, d in trace]
    assert set(phases[:-1]) == {"return_home"} and phases[-1] == "await_reset"
    assert all(d.target == pytest.approx(home) for _, _, d in trace)
    assert is_zero(trace[-1][2].command)
    assert math.hypot(robot.x - home[0], robot.y - home[1]) <= ctl.HOME_TOLERANCE
    facing = 0.0 if sign == 1 else math.pi
    assert abs(wrap(facing - robot.yaw)) <= ctl.HOME_HEADING_TOLERANCE


def home_pose(field=Field()):
    return Pose(-field.attack_sign * field.length * 0.3, 0.0, 0.0 if field.attack_sign == 1 else math.pi)


def test_latch_needs_both_reset_and_cleared_goal_flag():
    controller, ball, robot = Controller(), Ball(0.0, 0.0), home_pose()

    def phase(now, goal):
        return controller.tick(Observation(now, robot, ball, goal=goal), now).phase

    assert phase(0.0, 1) == "await_reset"
    assert phase(0.1, 0) == "await_reset"      # flag cleared, no reset yet
    assert phase(0.2, 1) == "await_reset"
    controller.reset()
    assert phase(0.3, 1) == "await_reset"      # reset, flag still raised
    assert phase(0.4, -1) == "await_reset"
    assert phase(0.5, 0) in ACTIVE             # both: play resumes
    assert phase(0.6, 0) in ACTIVE


def test_reset_before_a_goal_does_not_pre_release_the_latch():
    controller, ball, robot = Controller(), Ball(0.0, 0.0), Pose(0.5, 0.5, 0.0)
    controller.reset()
    assert controller.tick(Observation(0.0, robot, ball), 0.0).phase in ACTIVE
    assert controller.tick(Observation(0.1, robot, ball, goal=1), 0.1).phase == "return_home"
    assert controller.tick(Observation(0.2, robot, ball), 0.2).phase == "return_home"


def test_goal_seen_once_keeps_robot_out_of_play_until_released():
    controller, ball = Controller(), Ball(0.0, 0.0)
    now, robot = strike_started(controller)
    scored = controller.tick(Observation(now + DT, robot, ball, goal=1), now + DT)
    assert scored.phase == "return_home"       # the strike is abandoned at once
    trace, robot = run(controller, robot, ball, 30.0, start=now + 2 * DT)
    assert {d.phase for _, _, d in trace} == {"return_home", "await_reset"}
    assert trace[-1][2].phase == "await_reset"


def test_await_reset_stays_zero_even_if_the_robot_is_moved():
    controller = Controller()
    assert controller.tick(Observation(0.0, home_pose(), None, goal=1), 0.0).phase == "await_reset"
    moved = controller.tick(Observation(0.1, Pose(0.8, 0.8, 2.0), None, goal=1), 0.1)
    assert moved.phase == "await_reset" and is_zero(moved.command)


def test_latch_survives_lost_tracking_falls_and_bad_packets():
    controller, ball = Controller(), Ball(0.0, 0.0)
    robot = Pose(0.5, 0.5, 0.0)
    assert controller.tick(Observation(0.0, robot, ball, goal=1), 0.0).phase == "return_home"
    assert controller.tick(Observation(0.0, robot, ball), 1.0).phase == "tracking_lost"
    assert controller.tick(Observation(1.1, None, ball), 1.1).phase == "tracking_lost"
    assert controller.tick(Observation(1.2, robot, ball, fallen=True), 1.2).phase == "recovery_required"
    assert controller.tick(Observation(1.3, Pose(NAN, 0.0, 0.0), ball), 1.3).phase == "invalid_observation"
    assert controller.tick(Observation(1.4, robot, ball), 1.4).phase == "return_home"


def test_goal_flag_on_a_corrupt_packet_is_not_latched():
    controller, ball = Controller(), Ball(0.0, 0.0)
    bad = controller.tick(Observation(0.0, Pose(NAN, 0.0, 0.0), ball, goal=1), 0.0)
    assert bad.phase == "invalid_observation"
    assert controller.tick(Observation(0.1, Pose(-1.0, 0.0, 0.0), ball), 0.1).phase in ACTIVE
