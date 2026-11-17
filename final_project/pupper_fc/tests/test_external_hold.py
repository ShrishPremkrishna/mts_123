"""Local downstream veto cancels planning; no ROS or physical behavior implied."""
import math
import pickle

import pytest

from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.obstacles import ObstacleConfig
from final_project.pupper_fc.tests.test_integration import arm, frame, intent, command, is_zero
from final_project.pupper_fc.tests.test_obstacle_integration import camera
from final_project.pupper_fc.tests.test_resume_integration import home, resume, waiting


ROBOT = "pupper-1"
STAGED = {ROBOT: dict(x=-.45, y=0, yaw=0),
          "pupper-2": dict(x=.45, y=0, yaw=math.pi)}


def playing(kind="auto", *, both=False):
    executor = Executor()
    arm(executor)
    executor.handle(frame(robots=STAGED))
    extra = {"target": dict(x=1.83, y=0)} if kind == "shoot" else {}
    executor.handle(intent(kind=kind, **extra))
    if both:
        executor.handle(intent(robot_id="pupper-2"))
    for seq, now in enumerate((.1, .25), 2):
        output = executor.handle(frame(now, seq, robots=STAGED))
    assert output["decisions"][ROBOT]["phase"] == "strike"
    return executor


def held_and_released(executor):
    executor.set_external_hold(ROBOT, True, .26, "camera blocked")
    executor.set_external_hold(ROBOT, False, .3)


def test_external_hold_immediately_cancels_strike_and_only_its_robot():
    executor = playing(both=True)
    executor.set_external_hold(ROBOT, True, .26, "camera blocked")
    controller = executor.controllers[ROBOT]
    assert controller._strike is None and controller._aligned_since is None
    decisions = executor.tick(.27)
    assert decisions[ROBOT].phase == "external_hold"
    assert decisions[ROBOT].command.vx == 0
    assert decisions["pupper-2"].phase == "strike"
    assert decisions["pupper-2"].command.vx > 0


@pytest.mark.parametrize("kind", ["auto", "attack"])
def test_queued_inputs_cannot_recover_and_fresh_renewal_restarts_alignment(kind):
    executor = playing(kind)
    executor.set_external_hold(ROBOT, True, .26)
    executor.handle(frame(.27, 4, robots=STAGED))
    executor.handle(intent(.28, 2, kind=kind))
    assert executor.controllers[ROBOT]._aligned_since is None
    executor.set_external_hold(ROBOT, False, .3)
    assert executor.external_resume_after[ROBOT] == .3
    assert executor.tick(.3)[ROBOT].phase == "external_hold"
    executor.handle(frame(.31, 5, robots=STAGED))
    delayed = executor.handle(intent(.32, 3, kind=kind, issued_at=.29, valid_until=.79))
    assert is_zero(command(delayed)) and "freshly issued" in delayed["decisions"][ROBOT]["reason"]
    fresh = executor.handle(intent(.33, 4, kind=kind))
    assert fresh["decisions"][ROBOT]["phase"] == "align"
    assert executor.controllers[ROBOT]._aligned_since == .31  # First post-release source capture.
    # Repeated clear status must not move the watermark or reset alignment.
    executor.set_external_hold(ROBOT, False, .34)
    assert executor.external_resume_after[ROBOT] == .3
    assert executor.controllers[ROBOT]._aligned_since == .31
    assert executor.handle(frame(.59, 6, robots=STAGED))["decisions"][ROBOT]["phase"] == "strike"


@pytest.mark.parametrize("stale_field", [True, False])
def test_recovery_requires_both_source_times_strictly_after_release(stale_field):
    executor = playing()
    held_and_released(executor)
    executor.handle(frame(.31, 4, captured_at=.3 if stale_field else .31, robots=STAGED))
    output = executor.handle(intent(.32, 2, kind="auto", issued_at=.32 if stale_field else .3,
                                    valid_until=.8))
    assert is_zero(command(output)) and output["decisions"][ROBOT]["phase"] == "external_hold"
    if stale_field:
        output = executor.handle(frame(.33, 5, captured_at=.300001, robots=STAGED))
    else:
        output = executor.handle(intent(.33, 3, kind="auto", issued_at=.300001, valid_until=.8))
    assert output["decisions"][ROBOT]["phase"] == "align"


def test_held_ticks_do_not_accumulate_alignment_and_advance_watermark():
    executor = playing()
    executor.set_external_hold(ROBOT, True, .26)
    for seq, now in enumerate((.3, .4, .5), 4):
        executor.handle(frame(now, seq, robots=STAGED))
        executor.handle(intent(now, seq, kind="auto"))
        assert executor.controllers[ROBOT]._aligned_since is None
        assert executor.external_resume_after[ROBOT] == now
    executor.set_external_hold(ROBOT, False, .51)
    executor.handle(frame(.52, 7, robots=STAGED))
    assert executor.handle(intent(.53, 7, kind="auto"))["decisions"][ROBOT]["phase"] == "align"


def test_interrupted_shoot_id_cannot_be_reused_even_by_changing_target_or_kind():
    executor = playing("shoot")
    held_and_released(executor)
    executor.handle(frame(.31, 4, robots=STAGED))
    for seq, changes in enumerate((dict(kind="shoot", target=dict(x=1.83, y=.1)), dict(kind="attack")), 2):
        output = executor.handle(intent(.32, seq, **changes))
        assert not output["accepted"] and "new intent_id" in output["reason"]
        assert is_zero(command(output))
    output = executor.handle(intent(.33, 4, kind="shoot", intent_id="new-shot", target=dict(x=1.83, y=0)))
    assert output["accepted"] and output["decisions"][ROBOT]["phase"] == "align"
    output = executor.handle(intent(.34, 5, kind="shoot", target=dict(x=1.83, y=0)))
    assert not output["accepted"]  # Retirement survives intervening ownership.
    arm(executor, .35, "next")
    executor.handle(frame(.36, 1, epoch="next", robots=STAGED))
    output = executor.handle(intent(.37, 1, epoch="next", kind="shoot", target=dict(x=1.83, y=0)))
    assert output["accepted"]  # Explicit new gameplay epoch retires the old ID fence.


def test_shoot_submitted_while_held_is_refused_without_retiring_its_unused_id():
    executor = playing()
    executor.set_external_hold(ROBOT, True, .26)
    output = executor.handle(intent(.27, 2, kind="shoot", intent_id="queued-shot", target=dict(x=1.83, y=0)))
    assert not output["accepted"] and "refused during external hold" in output["reason"]
    assert executor.intent_seq[ROBOT] == 2 and executor.intents[ROBOT] is None
    assert "queued-shot" not in executor.external_retired_ids[ROBOT]
    executor.set_external_hold(ROBOT, False, .3)
    executor.handle(frame(.31, 4, robots=STAGED))
    assert is_zero(command(executor.handle(intent(.32, 2, kind="shoot", intent_id="queued-shot",
                                                 target=dict(x=1.83, y=0)))))  # Consumed sequence cannot retry.
    output = executor.handle(intent(.32, 3, kind="shoot", intent_id="queued-shot", target=dict(x=1.83, y=0)))
    assert output["accepted"] and output["decisions"][ROBOT]["phase"] == "align"


def test_expiry_clears_owner_during_hold_but_does_not_unretire_shoot():
    executor = playing("shoot")
    executor.set_external_hold(ROBOT, True, .26)
    assert executor.tick(.5)[ROBOT].phase == "external_hold"
    assert executor.intents[ROBOT] is None
    executor.set_external_hold(ROBOT, False, .51)
    executor.handle(frame(.52, 4, robots=STAGED))
    output = executor.handle(intent(.53, 2, kind="shoot", target=dict(x=1.83, y=0)))
    assert not output["accepted"] and is_zero(command(output))


def test_live_auto_role_survives_hold_and_renewal_but_not_expired_authority():
    executor = Executor()
    arm(executor)
    executor.handle(frame(ball=dict(x=-1.5, y=0)))
    assert executor.handle(intent(kind="auto"))["decisions"][ROBOT]["role"] == "defense"
    executor.set_external_hold(ROBOT, True, .1)
    executor.handle(frame(.11, 2, ball=dict(x=-.4, y=0)))
    executor.handle(intent(.12, 2, kind="auto"))
    executor.set_external_hold(ROBOT, False, .13)
    executor.handle(frame(.14, 3, ball=dict(x=-.4, y=0)))
    output = executor.handle(intent(.15, 3, kind="auto"))
    assert output["decisions"][ROBOT]["role"] == "defense"
    executor.set_external_hold(ROBOT, True, .2)
    assert executor.tick(.7)[ROBOT].role is None
    assert executor.intents[ROBOT] is None
    executor.set_external_hold(ROBOT, False, .71)
    executor.handle(frame(.72, 4, ball=dict(x=-.4, y=0)))
    assert executor.handle(intent(.73, 4, kind="auto"))["decisions"][ROBOT]["role"] == "offense"


def test_goal_resume_refuses_hold_and_recovery_without_queuing_reset():
    executor = waiting()
    executor.set_external_hold(ROBOT, True, .15)
    output = executor.handle(resume(.16))
    assert not output["accepted"] and is_zero(command(output))
    assert executor.controllers[ROBOT]._goal_latched
    executor.set_external_hold(ROBOT, False, .2)
    executor.handle(frame(.21, 3, robots=home()))
    output = executor.handle(resume(.22, frame_seq=3))
    assert not output["accepted"] and "freshly issued" in output["reason"]
    assert not executor.controllers[ROBOT]._reset_requested
    executor.handle(intent(.23, 2, kind="auto"))
    assert executor.handle(resume(.24, frame_seq=3))["accepted"]


def test_goal_pulse_during_hold_remains_latched_after_clearance():
    executor = playing()
    executor.set_external_hold(ROBOT, True, .26)
    executor.handle(frame(.27, 4, robots=home(goal=1)))
    assert executor.controllers[ROBOT]._goal_latched
    executor.set_external_hold(ROBOT, False, .3)
    executor.handle(frame(.31, 5, robots=home()))
    output = executor.handle(intent(.32, 2, kind="auto"))
    assert output["decisions"][ROBOT]["phase"] == "await_reset"


def test_stop_and_new_epoch_cannot_release_active_external_veto():
    executor = playing()
    executor.set_external_hold(ROBOT, True, .26)
    executor.handle(dict(event="stop", now=.27))
    assert not executor.armed and executor.external_holds[ROBOT]
    arm(executor, .28, "next")
    executor.handle(frame(.29, 1, epoch="next", robots=STAGED))
    output = executor.handle(intent(.29, 1, epoch="next", kind="auto"))
    assert output["decisions"][ROBOT]["phase"] == "external_hold"
    before = executor.last_now
    assert not executor.handle(intent(999, 9))["accepted"]
    assert executor.last_now == before and executor.external_holds[ROBOT]
    executor.set_external_hold(ROBOT, False, .3)
    executor.handle(frame(.31, 2, epoch="next", robots=STAGED))
    assert executor.handle(intent(.32, 2, epoch="next", kind="auto"))["decisions"][ROBOT]["phase"] == "align"


@pytest.mark.parametrize("new_arm", [10.31, 1.0])
def test_release_then_arm_requires_new_sources_even_when_new_clock_changes_epoch(new_arm):
    executor = Executor()
    arm(executor, 9.9)
    executor.set_external_hold(ROBOT, True, 10.0)
    executor.set_external_hold(ROBOT, False, 10.3)
    executor.arm("next", "new-local-clock", new_arm)
    assert executor.external_resume_after[ROBOT] == new_arm
    executor.handle(frame(new_arm, 1, epoch="next", clock="new-local-clock",
                          captured_at=new_arm-.06, robots=STAGED))
    output = executor.handle(intent(new_arm+.01, 1, epoch="next", clock="new-local-clock",
                                    issued_at=new_arm-.11, valid_until=new_arm+.39))
    assert output["accepted"] and is_zero(command(output))
    assert output["decisions"][ROBOT]["phase"] == "external_hold"
    executor.handle(frame(new_arm+.02, 2, epoch="next", clock="new-local-clock", robots=STAGED))
    output = executor.handle(intent(new_arm+.03, 2, epoch="next", clock="new-local-clock"))
    assert output["decisions"][ROBOT]["phase"] == "align"


def test_expired_shoot_is_not_newly_retired_when_hold_begins():
    executor = playing("shoot")
    executor.set_external_hold(ROBOT, True, .5)
    assert "play" not in executor.external_retired_ids[ROBOT]
    executor.set_external_hold(ROBOT, False, .51)
    executor.handle(frame(.52, 4, robots=STAGED))
    output = executor.handle(intent(.53, 2, kind="shoot", target=dict(x=1.83, y=0)))
    assert output["accepted"] and output["decisions"][ROBOT]["phase"] == "align"


def test_clear_notification_cannot_arm_stopped_executor_and_json_cannot_clear_hold():
    executor = Executor()
    executor.set_external_hold(ROBOT, True, 0)
    executor.set_external_hold(ROBOT, False, .1)
    assert not executor.armed and executor.tick(.1)[ROBOT].phase == "stopped"
    arm(executor, .2)
    executor.set_external_hold(ROBOT, True, .21)
    output = executor.handle(dict(event="external_hold", now=.22, robot_id=ROBOT, held=False))
    assert not output["accepted"] and not executor.armed
    assert executor.external_holds[ROBOT]


@pytest.mark.parametrize("changes", [
    {"robot_id": "missing"}, {"robot_id": []}, {"held": 1}, {"held": None},
    {"now": float("nan")}, {"now": float("inf")}, {"now": True}, {"now": -.1}, {"now": .1},
    {"reason": ""}, {"reason": None}, {"reason": "x" * 129},
])
def test_malformed_external_hold_calls_are_atomic(changes):
    executor = playing()
    arguments = dict(robot_id=ROBOT, held=True, now=.26, reason="camera blocked")
    arguments.update(changes)
    before = pickle.dumps(executor)
    with pytest.raises(ValueError):
        executor.set_external_hold(**arguments)
    assert pickle.dumps(executor) == before
    assert executor.controllers[ROBOT]._strike is not None


def test_external_release_does_not_bypass_optional_internal_camera_gate():
    executor = Executor(obstacle_config=ObstacleConfig(clear_frames=1))
    arm(executor)
    executor.handle(camera(0, 1, blocked=True))
    executor.handle(frame(robots=STAGED))
    executor.handle(intent(kind="auto"))
    executor.set_external_hold(ROBOT, True, .1)
    executor.set_external_hold(ROBOT, False, .2)
    executor.handle(frame(.21, 2, robots=STAGED))
    assert executor.handle(intent(.22, 2, kind="auto"))["decisions"][ROBOT]["phase"] == "obstacle_hold"
    executor.handle(camera(.23, 2))
    assert executor.handle(frame(.24, 3, robots=STAGED))["decisions"][ROBOT]["phase"] == "align"
