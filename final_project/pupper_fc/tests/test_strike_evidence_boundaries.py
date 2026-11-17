"""Strike evidence and kinematic field bounds; no physical braking claim."""
import math

import pytest

from final_project.pupper_fc import controller as ctl
from final_project.pupper_fc.controller import Controller
from final_project.pupper_fc.model import Ball, Field, Observation, Pose


STAGED = Pose(-0.45, 0, 0)
BALL = Ball(0, 0)


def tick(controller, captured_at, *, now=None, robot=STAGED, ball=BALL, target=None):
    return controller.tick(Observation(captured_at, robot, ball),
                           captured_at if now is None else now, shot_target=target)


def is_zero(decision):
    return (decision.command.vx, decision.command.vy, decision.command.wz) == (0, 0, 0)


def start(controller, **kwargs):
    assert tick(controller, 0, **kwargs).phase == "align"
    return tick(controller, ctl.ALIGN_DWELL, **kwargs)


def test_repeated_ticks_on_one_aligned_capture_cannot_authorize_strike():
    controller = Controller()
    for now in (0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.34, 0.35, 0.4):
        decision = tick(controller, 0, now=now)
        assert decision.phase != "strike"
        if now >= ctl.MAX_STRIKE_OBSERVATION_AGE:
            assert is_zero(decision)


def test_distinct_capture_times_must_span_alignment_dwell():
    controller = Controller()
    assert tick(controller, 0).phase == "align"
    assert tick(controller, 0.1).phase == "align"
    assert tick(controller, 0.1, now=0.24).phase == "align"
    assert tick(controller, 0.25).phase == "strike"


@pytest.mark.parametrize("interruption", ["unaligned", "invalid", "cancel", "expired"])
def test_interruption_requires_a_new_alignment_evidence_span(interruption):
    controller = Controller()
    tick(controller, 0)
    if interruption == "unaligned":
        tick(controller, 0.2, robot=Pose(-0.45, 0, 0.2))
    elif interruption == "invalid":
        tick(controller, 0.2, ball=Ball(float("nan"), 0))
    elif interruption == "cancel":
        controller.cancel()
    else:
        assert is_zero(tick(controller, 0, now=ctl.MAX_OBSERVATION_AGE))
    assert tick(controller, 0.4).phase == "align"
    assert tick(controller, 0.65).phase == "strike"


def test_reordered_capture_restarts_alignment_evidence():
    controller = Controller()
    tick(controller, 0)
    tick(controller, 0.1)
    assert tick(controller, 0.05, now=0.11).phase == "align"
    assert tick(controller, 0.25).phase == "align"
    assert tick(controller, 0.3).phase == "strike"


@pytest.mark.parametrize("age", [ctl.MAX_OBSERVATION_AGE, 0.4, 0.5])
def test_delayed_new_capture_cannot_authorize_strike(age):
    controller = Controller()
    tick(controller, 0)
    decision = tick(controller, 0.25, now=0.25 + age)
    assert decision.phase == "tracking_lost" and is_zero(decision)
    assert tick(controller, 1.0).phase == "align"


def test_cached_strike_evidence_cannot_extend_evidence_deadline():
    controller = Controller()
    initial = start(controller)
    assert initial.phase == "strike"
    deadline = 0.25 + ctl.MAX_OBSERVATION_AGE
    for now in (0.5, 0.55):
        repeated = tick(controller, 0.25, now=now)
        assert repeated.phase == "strike"
        assert repeated.command.expires_at == pytest.approx(deadline)
        assert now < repeated.command.expires_at <= now + 0.15
    expired = tick(controller, 0.25, now=deadline)
    assert expired.phase == "tracking_lost" and is_zero(expired)
    assert tick(controller, deadline + 0.01).phase == "align"


@pytest.mark.parametrize("age", [0.15, 0.3])
def test_usable_delayed_evidence_still_produces_a_live_strike_command(age):
    controller = Controller()
    assert tick(controller, 0, now=age).phase == "align"
    now = ctl.ALIGN_DWELL + age
    decision = tick(controller, ctl.ALIGN_DWELL, now=now)
    assert decision.phase == "strike" and decision.command.vx > 0
    assert now < decision.command.expires_at <= now + 0.15
    assert decision.command.expires_at <= ctl.ALIGN_DWELL + ctl.MAX_OBSERVATION_AGE


@pytest.mark.parametrize("capture", [0.2, 0.25, 123.4])
def test_exact_capture_deadline_cannot_authorize_nonzero_strike(capture):
    controller = Controller()
    tick(controller, capture - ctl.ALIGN_DWELL)
    assert tick(controller, capture).phase == "strike"
    decision = tick(controller, capture, now=capture + ctl.MAX_STRIKE_OBSERVATION_AGE)
    assert decision.phase == "tracking_lost" and is_zero(decision)


@pytest.mark.parametrize("sx", [-1, 1])
@pytest.mark.parametrize("sy", [-1, 1])
@pytest.mark.parametrize("position,heading", [((1.3, 0), 0),
                                              ((0, 0.75), math.pi / 2),
                                              ((1.0, 0.7), math.pi / 4)])
def test_strike_swept_body_and_allowance_fit_all_sides_and_diagonals(sx, sy, position, heading):
    ux, uy = sx * math.cos(heading), sy * math.sin(heading)
    robot = Pose(sx * position[0], sy * position[1], math.atan2(uy, ux))
    ball = Ball(robot.x + 0.45 * ux, robot.y + 0.45 * uy)
    target = (robot.x + 0.9 * ux, robot.y + 0.9 * uy)
    decision = start(Controller(Field(attack_sign=sx)), robot=robot, ball=ball, target=target)
    assert decision.phase == "reassess" and is_zero(decision)
    assert "leave the field" in decision.reason


@pytest.mark.parametrize("sign", [-1, 1])
def test_explicit_stopping_allowance_changes_only_borderline_strike_admission(sign):
    field = Field(attack_sign=sign)
    robot = Pose(sign * 1.1, 0, 0 if sign == 1 else math.pi)
    ball = Ball(sign * 1.55, 0)
    with_allowance = start(Controller(field), robot=robot, ball=ball)
    without_allowance = start(Controller(field, strike_stopping_allowance=0), robot=robot, ball=ball)
    assert is_zero(with_allowance) and "leave the field" in with_allowance.reason
    assert without_allowance.phase == "strike"


@pytest.mark.parametrize("y,yaw", [(1.08, 0), (1.05, 0.25)])
def test_active_strike_rechecks_current_position_and_heading(y, yaw):
    controller = Controller()
    target = (1.83, 0.7)
    assert start(controller, robot=Pose(1.0, 0.7, 0), ball=Ball(1.45, 0.7), target=target).phase == "strike"
    robot = Pose(1.05, y, yaw)
    ball = Ball(robot.x + 0.45 * math.cos(yaw), robot.y + 0.45 * math.sin(yaw))
    decision = tick(controller, 0.3, robot=robot, ball=ball, target=target)
    assert decision.phase == "reassess" and is_zero(decision)
    assert "leave the field" in decision.reason


def test_boundary_projection_includes_observation_age_travel():
    robot, ball = Pose(1.0, 0, 0), Ball(1.45, 0)
    assert start(Controller(), robot=robot, ball=ball).phase == "strike"
    controller = Controller()
    tick(controller, 0, robot=robot, ball=ball)
    decision = tick(controller, 0.25, now=0.37, robot=robot, ball=ball)
    assert decision.phase == "reassess" and is_zero(decision)
    assert "leave the field" in decision.reason


@pytest.mark.parametrize("allowance", [-0.1, float("nan"), float("inf"), True, "0.1"])
def test_stopping_allowance_must_be_finite_and_nonnegative(allowance):
    with pytest.raises(ValueError, match="stopping allowance"):
        Controller(strike_stopping_allowance=allowance)
