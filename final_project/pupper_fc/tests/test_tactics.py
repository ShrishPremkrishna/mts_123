"""Role/geometry contracts only; these tests do not validate robot behavior."""
import math
from dataclasses import replace

import pytest

from final_project.pupper_fc.model import Ball, Field, Observation, Pose
from final_project.pupper_fc.tactics import Tactics, TacticsConfig


def observe(x=0.0, y=0.0, now=0.0, robot_id="one", **kwargs):
    return Observation(now, Pose(0, 0, 0), Ball(x, y), robot_id, **kwargs)


def at_fraction(fraction, field=Field()):
    return field.attack_sign * (fraction * field.length - field.length / 2)


def test_opponents_are_independent_and_may_both_attack():
    one = Tactics(Field(), "one")
    two = Tactics(Field(attack_sign=-1), "two")
    assert one.tick(observe(), 0).action == "attack"
    assert two.tick(observe(robot_id="two"), 0).action == "attack"
    assert one.tick(observe(-1.5, now=.1), .1).role == "defense"
    assert two.role == "offense"
    assert two.tick(observe(-1.5, now=.1, robot_id="two"), .1).action == "attack"


def test_reflection_exchanges_goals_and_preserves_decisions():
    one = Tactics(Field(), "one")
    two = Tactics(Field(attack_sign=-1), "two")
    for now, x, y in [(0, 0, .2), (.1, -1.5, .9), (1.2, -.4, -.8), (2.4, 0, 0)]:
        left = one.tick(observe(x, y, now), now)
        right = two.tick(observe(-x, y, now, "two"), now)
        assert (left.role, left.action, left.reason) == (right.role, right.action, right.reason)
        if left.target is not None:
            assert right.target.x == pytest.approx(-left.target.x)
            assert right.target.y == pytest.approx(left.target.y)
            assert math.cos(right.target.yaw) == pytest.approx(-math.cos(left.target.yaw))
            assert math.sin(right.target.yaw) == pytest.approx(math.sin(left.target.yaw))


def test_hysteresis_rejects_jitter_at_both_thresholds():
    tactics = Tactics(Field(), "one", TacticsConfig(minimum_dwell=0))
    for now, fraction, expected in [(0, .5, "offense"), (1, .299, "defense"),
                                    (2, .301, "defense"), (3, .299, "defense"),
                                    (4, .451, "offense"), (5, .449, "offense"),
                                    (6, .451, "offense")]:
        assert tactics.tick(observe(at_fraction(fraction), now=now), now).role == expected


def test_dwell_delays_normal_change_but_emergency_enters_immediately():
    tactics = Tactics(Field(), "one", TacticsConfig(minimum_dwell=2))
    assert tactics.tick(observe(), 0).role == "offense"
    early = tactics.tick(observe(at_fraction(.25), now=.1), .1)
    assert early.role == "offense" and "dwell" in early.reason
    emergency = tactics.tick(observe(at_fraction(.10), now=.2), .2)
    assert emergency.role == "defense" and "emergency" in emergency.reason
    assert tactics.tick(observe(now=2), 2).role == "defense"
    assert tactics.tick(observe(now=2.3), 2.3).role == "offense"


def test_first_valid_frame_can_select_defense_without_waiting():
    tactics = Tactics(Field(), "one")
    assert tactics.tick(observe(at_fraction(.25)), 0).action == "defend"


def test_hold_does_not_restart_dwell_deadline():
    tactics = Tactics(Field(), "one")
    assert tactics.tick(observe(-1.5), 0).role == "defense"
    assert tactics.tick(replace(observe(now=.99), ball=None), .99).action == "hold"
    assert tactics.tick(observe(now=1.01), 1.01).role == "offense"


@pytest.mark.parametrize("sign", [1, -1])
def test_ball_at_guard_target_keeps_finite_attack_heading(sign):
    field = Field(attack_sign=sign)
    config = TacticsConfig()
    x = sign * (-field.length / 2 + config.guard_depth)
    target = Tactics(field, "one", config).tick(observe(x), 0).target
    assert (target.x, target.y) == (x, 0)
    assert math.cos(target.yaw) == pytest.approx(sign)
    assert math.sin(target.yaw) == pytest.approx(0)


@pytest.mark.parametrize("sign", [1, -1])
@pytest.mark.parametrize("y", [-1.2, 0, 1.2])
@pytest.mark.parametrize("depth", [0, .35, 100])
def test_guard_pose_stays_inside_field_and_goal_span_and_faces_ball(sign, y, depth):
    field = Field(attack_sign=sign)
    config = TacticsConfig(guard_depth=depth)
    ball_x = at_fraction(.1, field)
    result = Tactics(field, "one", config).tick(observe(ball_x, y), 0)
    target = result.target
    assert result.action == "defend"
    assert abs(target.x) <= field.length / 2 - field.robot_radius - config.boundary_clearance + 1e-9
    assert abs(target.y) <= field.width / 2 - field.robot_radius - config.boundary_clearance
    assert abs(target.y) <= max(0, field.goal_width / 2 - field.robot_radius)
    assert target.yaw == pytest.approx(math.atan2(y - target.y, ball_x - target.x))


@pytest.mark.parametrize("sign", [1, -1])
def test_ball_behind_goal_holds_instead_of_requesting_unreachable_guard(sign):
    tactics = Tactics(Field(attack_sign=sign), "one")
    tactics.tick(observe(-sign * 1.5), 0)
    decision = tactics.tick(observe(-sign * 2, now=.1), .1)
    assert decision.role == "defense"
    assert decision.action == "hold" and decision.target is None


@pytest.mark.parametrize("invalid", [
    None,
    replace(observe(), robot=None),
    replace(observe(), ball=None),
    replace(observe(), robot=Pose(float("nan"), 0, 0)),
    replace(observe(), ball=Ball(0, 0, float("inf"), 0)),
    replace(observe(), robot_id="other"),
    replace(observe(), fallen=True),
    replace(observe(), fallen=1),
    replace(observe(), captured_at=-1),
    replace(observe(), captured_at=1),
    replace(observe(), captured_at=float("nan")),
    replace(observe(), robot=Pose(0, 5, 0)),
    replace(observe(), ball=Ball(0, 5)),
])
def test_missing_stale_or_invalid_state_holds_without_target_and_preserves_role(invalid):
    tactics = Tactics(Field(), "one")
    assert tactics.tick(observe(-1.5), 0).role == "defense"
    result = tactics.tick(invalid, 0)
    assert (result.role, result.action, result.target) == ("defense", "hold", None)


def test_frozen_image_becomes_stale_and_recovery_uses_fresh_state():
    tactics = Tactics(Field(), "one")
    frame = observe(-1.5)
    assert tactics.tick(frame, 0).role == "defense"
    assert tactics.tick(frame, .2).action == "defend"
    assert tactics.tick(frame, .36).action == "hold"
    result = tactics.tick(observe(now=2), 2)
    assert result.role == "offense" and result.action == "attack"


def test_clock_rollback_and_reordered_capture_hold():
    tactics = Tactics(Field(), "one")
    tactics.tick(observe(now=1), 1)
    assert tactics.tick(observe(now=.9), .9).action == "hold"
    assert tactics.tick(observe(now=.95), 1.1).action == "hold"
    assert tactics.tick(observe(now=1.2), 1.2).action == "attack"


@pytest.mark.parametrize("now", [float("nan"), float("inf"), None, True, 10 ** 1000])
def test_invalid_control_clock_holds(now):
    result = Tactics(Field(), "one").tick(observe(), now)
    assert result.action == "hold" and result.target is None


@pytest.mark.parametrize("field,config", [
    (Field(attack_sign=0), TacticsConfig()),
    (Field(width=0), TacticsConfig()),
    (Field(goal_width=4), TacticsConfig()),
    (Field(robot_radius=2), TacticsConfig()),
    (Field(length=float("nan")), TacticsConfig()),
    (Field(), TacticsConfig(enter_defense=.5)),
    (Field(), TacticsConfig(emergency_defense=.35)),
    (Field(), TacticsConfig(minimum_dwell=-1)),
    (Field(), TacticsConfig(max_observation_age=0)),
    (Field(), TacticsConfig(boundary_clearance=float("inf"))),
])
def test_invalid_configuration_rejected(field, config):
    with pytest.raises(ValueError):
        Tactics(field, "one", config)
