"""Leased role selection and execution together; ideal motion, no hardware."""
from dataclasses import asdict
import math

import pytest

from final_project.pupper_fc.controller import Controller
from final_project.pupper_fc.integration import Executor, replay
from final_project.pupper_fc.model import Ball, Field, Observation, Pose
from final_project.pupper_fc.obstacles import ObstacleConfig
from final_project.pupper_fc.tests.test_controller import step
from final_project.pupper_fc.tests.test_integration import arm, frame, intent, command, is_zero
from final_project.pupper_fc.tests.test_obstacle_integration import camera


def row(output):
    return output["decisions"]["pupper-1"]


def auto(*, staged=False, obstacles=False):
    executor = Executor(obstacle_config=ObstacleConfig(clear_frames=1) if obstacles else None)
    arm(executor)
    if obstacles:
        executor.handle(camera(0, 1))
    executor.handle(frame(robots={"pupper-1": dict(x=-.45 if staged else -1, y=0, yaw=0)}))
    executor.handle(intent(kind="auto"))
    return executor


def strike(executor):
    for seq, now in enumerate((.1, .25), 2):
        output = executor.handle(frame(now, seq, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
    assert row(output)["phase"] == "strike"


@pytest.mark.parametrize("sign", [1, -1])
def test_distant_defend_retains_guard_heading_while_move_faces_travel(sign):
    field = Field(attack_sign=sign)
    target = Pose(-sign * 1.48, .25, math.atan2(.45, sign * .28))
    robot = Pose(sign * -.2, -.4, target.yaw)
    observation = Observation(0, robot, Ball(sign * -1.2, .7))
    ordinary = Controller(field).tick(observation, 0, target_pose=target)
    guarding = Controller(field).tick(observation, 0, target_pose=target, preserve_target_heading=True)
    assert ordinary.target_heading == pytest.approx(math.atan2(target.y - robot.y, target.x - robot.x))
    assert guarding.target_heading == target.yaw
    assert guarding.command.wz == 0
    assert (guarding.command.vx, guarding.command.vy) == (ordinary.command.vx, ordinary.command.vy)

    executor = Executor(fields={"pupper-1": field})
    arm(executor)
    executor.handle(frame(robots={"pupper-1": asdict(robot)}, ball=asdict(observation.ball)))
    result = executor.handle(intent(kind="auto"))
    assert row(result)["phase"] == "defend" and row(result)["role"] == "defense"
    assert row(result)["target_heading"] == pytest.approx(target.yaw)
    assert command(result)["wz"] == pytest.approx(0)
    result = executor.handle(intent(.01, 2, kind="defend", target=asdict(target)))
    assert row(result)["role"] is None and row(result)["target_heading"] == target.yaw


@pytest.mark.parametrize("options", [{"preserve_target_heading": 1},
                                      {"preserve_target_heading": True},
                                      {"preserve_target_heading": True, "shot_target": (1, 0)}])
def test_heading_option_requires_boolean_and_pose_target(options):
    result = Controller().tick(Observation(0, Pose(-1, 0, 0), Ball(0, 0)), 0, **options)
    assert result.phase == "invalid_intent" and is_zero(asdict(result.command))


@pytest.mark.parametrize("sign", [1, -1])
def test_guard_closed_loop_converges_with_mirrored_motion_and_ball_jitter(sign):
    executor = Executor(fields={"pupper-1": Field(attack_sign=sign)})
    arm(executor)
    robot = Pose(sign * -.1, -.6, 0 if sign == 1 else math.pi)
    positions, roles = [], set()
    for n in range(121):
        now = n * .05
        ball = Ball(sign * -1.2, .6 + (-1 if n % 2 else 1) * .01)
        executor.handle(frame(now, n + 1, robots={"pupper-1": asdict(robot)}, ball=asdict(ball)))
        if n % 4 == 0:
            executor.handle(intent(now, n + 1, kind="auto"))
        decision = executor.tick(now)["pupper-1"]
        roles.add(decision.role)
        assert decision.command.expires_at <= min(now + .15, now + .35) + 1e-12
        robot = step(robot, decision.command)
        positions.append((robot.x, robot.y))
    assert roles == {"defense"}
    assert math.hypot(robot.x + sign * 1.48, robot.y - .25) < .065
    assert abs(math.atan2(math.sin(decision.target_heading - robot.yaw),
                          math.cos(decision.target_heading - robot.yaw))) < .1
    assert all(abs(x) <= 1.67 and abs(y) <= 1.06 for x, y in positions)


def test_auto_hysteresis_survives_renewals_jitter_and_missing_pose():
    executor = auto()
    for seq in range(2, 18):
        now = (seq - 1) * .1
        # Enter defense, then jitter inside its hysteresis band.
        fraction = .13 if seq == 2 else .37 + .01 * (-1 if seq % 2 else 1)
        executor.handle(intent(now, seq, kind="auto"))
        output = executor.handle(frame(now, seq, ball=dict(x=fraction * 3.66 - 1.83, y=0),
                                       robots={"pupper-1": None if seq == 6 else dict(x=-1, y=0, yaw=0)}))
        assert row(output)["role"] == "defense"
        if seq == 6:
            assert row(output)["phase"] == "tactical_hold" and is_zero(command(output))
    result = executor.handle(frame(1.7, 18, ball=dict(x=0, y=0)))
    assert row(result)["role"] == "offense"


def test_explicit_owner_preempts_auto_and_never_falls_back_after_expiry():
    executor = auto(staged=True)
    strike(executor)
    result = executor.handle(intent(.26, 2, kind="move", target=dict(x=-.45, y=.2, yaw=0)))
    assert row(result)["phase"] == "move" and row(result)["role"] is None
    assert executor.controllers["pupper-1"]._strike is None
    executor.handle(frame(.75, 4))
    result = executor.handle(dict(event="tick", now=.76))
    assert row(result)["phase"] == "intent_lost" and row(result)["role"] is None
    assert is_zero(command(executor.handle(frame(.77, 5))))


def test_emergency_role_change_preempts_strike_and_returns_to_fresh_alignment():
    executor = auto(staged=True)
    strike(executor)
    result = executor.handle(frame(.26, 4, ball=dict(x=-1.5, y=.2)))
    assert row(result)["role"] == "defense" and row(result)["phase"] == "defend"
    assert executor.controllers["pupper-1"]._strike is None
    for seq in range(5, 17):
        now = .26 + (seq - 4) * .1
        executor.handle(intent(now, seq, kind="auto"))
        result = executor.handle(frame(now, seq, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
        if seq < 14:
            assert row(result)["role"] == "defense"
        if row(result)["role"] == "offense":
            assert row(result)["phase"] == "align"
            assert executor.controllers["pupper-1"]._strike is None
            break
    assert row(result)["role"] == "offense"


def test_opponents_select_independent_roles_from_the_same_frame():
    executor = auto()
    executor.handle(intent(robot_id="pupper-2", kind="auto"))
    result = executor.handle(frame(.1, 2, ball=dict(x=-1.5, y=0)))
    assert row(result)["role"] == "defense"
    assert result["decisions"]["pupper-2"]["role"] == "offense"
    assert command(result)["robot_id"] == "pupper-1"
    assert command(result, "pupper-2")["robot_id"] == "pupper-2"


def test_auto_lease_and_capture_expiry_cannot_revive_strike():
    executor = auto(staged=True)
    strike(executor)
    assert executor.tick(.3)["pupper-1"].command.expires_at == pytest.approx(.45)
    result = executor.handle(dict(event="tick", now=.5))
    assert row(result)["phase"] == "intent_lost" and is_zero(command(result))
    executor.handle(frame(.51, 4, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
    result = executor.handle(intent(.51, 2, kind="auto"))
    assert row(result)["phase"] == "align"
    result = executor.handle(dict(event="tick", now=.86))
    assert is_zero(command(result))
    assert executor.controllers["pupper-1"]._strike is None
    result = executor.handle(frame(.87, 5, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
    assert row(result)["phase"] == "align"


def test_repeated_capture_ticks_cannot_supply_auto_alignment_dwell():
    executor = auto(staged=True)
    for now in (.1, .2, .3):
        assert executor.tick(now)["pupper-1"].phase == "align"
    result = executor.handle(frame(.31, 2, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
    assert row(result)["phase"] == "strike"


def test_camera_hold_preserves_role_and_resumption_needs_new_pose_and_alignment():
    executor = auto(staged=True, obstacles=True)
    strike(executor)
    result = executor.handle(camera(.26, 2, blocked=True))
    assert row(result)["role"] == "offense" and row(result)["phase"] == "obstacle_hold"
    assert is_zero(command(result)) and executor.controllers["pupper-1"]._strike is None
    result = executor.handle(camera(.27, 3))
    assert row(result)["phase"] == "tracking_lost" and is_zero(command(result))
    result = executor.handle(frame(.28, 4, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
    assert row(result)["phase"] == "align"


def test_defense_role_survives_camera_and_tracking_holds_until_new_owner():
    executor = auto(obstacles=True)
    result = executor.handle(frame(.1, 2, ball=dict(x=-1.5, y=0)))
    assert row(result)["role"] == "defense"
    result = executor.handle(camera(.11, 2, blocked=True))
    assert row(result)["role"] == "defense" and is_zero(command(result))
    result = executor.handle(camera(.12, 3))
    assert row(result)["role"] == "defense" and is_zero(command(result))
    result = executor.handle(frame(.13, 3, ball=dict(x=-.4, y=0)))
    assert row(result)["role"] == "defense"
    result = executor.handle(intent(.14, 2, kind="auto", intent_id="new-owner"))
    assert row(result)["role"] == "offense"


def test_goal_latch_owns_return_even_without_ball_or_goals_and_reset_is_bounded():
    executor = auto()
    home = dict(x=-.3 * 3.66, y=0, yaw=0, goal=1)
    result = executor.handle(frame(.1, 2, robots={"pupper-1": home}, ball=None, goals={}))
    assert row(result)["phase"] == "await_reset" and is_zero(command(result))
    home["goal"] = 0
    result = executor.handle(frame(.2, 3, robots={"pupper-1": home}))
    assert row(result)["phase"] == "await_reset"
    executor.controllers["pupper-1"].reset()
    result = executor.handle(frame(.3, 4, robots={"pupper-1": home}, goals={}))
    assert is_zero(command(result))
    result = executor.handle(dict(event="tick", now=.31))
    assert row(result)["phase"] == "tracking_lost" and is_zero(command(result))


def test_referee_return_can_recover_body_partly_outside_field_without_tactical_target():
    executor = auto()
    result = executor.handle(frame(.1, 2, robots={"pupper-1": dict(x=-1.8, y=0, yaw=0, goal=1)},
                                   ball=None, goals={}))
    assert row(result)["phase"] == "return_home"
    assert command(result)["vx"] > 0


def test_stop_and_new_epoch_require_fresh_authority_and_reset_tactics():
    executor = auto()
    executor.handle(frame(.1, 2, ball=dict(x=-1.5, y=0)))
    result = executor.handle(dict(event="stop", now=.11))
    assert row(result)["phase"] == "stopped" and row(result)["role"] is None
    assert not executor.handle(intent(.12, 2, kind="auto"))["accepted"]
    assert arm(executor, .13, "next")["accepted"]
    assert executor.tactics["pupper-1"].role == "offense"
    assert not executor.handle(frame(999, 3))["accepted"]
    assert executor.last_now == .13
    result = executor.handle(frame(.14, epoch="next"))
    assert is_zero(command(result)) and row(result)["phase"] == "intent_lost"
    result = executor.handle(intent(.14, kind="auto", epoch="next"))
    assert row(result)["role"] == "offense" and not is_zero(command(result))


def test_auto_requires_goal_for_attack_but_can_guard_without_it_and_fall_holds():
    executor = auto()
    result = executor.handle(frame(.1, 2, goals={}))
    assert row(result)["phase"] == "tracking_lost" and is_zero(command(result))
    result = executor.handle(frame(.2, 3, ball=dict(x=-1.5, y=0), goals={}))
    assert row(result)["role"] == "defense" and row(result)["phase"] == "defend"
    result = executor.handle(frame(.3, 4, robots={"pupper-1": dict(x=-1, y=0, yaw=0, fallen=True)}))
    assert row(result)["recovery_requested"] and is_zero(command(result))


def test_auto_target_rejection_revokes_lease():
    executor = auto()
    result = executor.handle(intent(.1, 2, kind="auto", target=dict(x=1, y=0)))
    assert not result["accepted"] and is_zero(command(result))


def test_auto_replay_is_deterministic_and_default_frame_cannot_self_arm():
    events = [dict(event="arm", now=0, epoch="test", clock="replay"), frame(),
              intent(kind="auto"), frame(.1, 2, ball=dict(x=-1.5, y=0)),
              dict(event="stop", now=.2)]
    assert list(replay(events)) == list(replay(events))
    executor = Executor()
    assert not executor.handle(frame())["accepted"] and not executor.armed
    arm(executor)
    assert row(executor.handle(frame()))["phase"] == "intent_lost"
