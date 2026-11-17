"""A deliberate public resume cue consumes one completed goal reset."""
from copy import deepcopy
import math

import pytest

from final_project.pupper_fc.integration import Executor, replay
from final_project.pupper_fc.obstacles import ObstacleConfig
from final_project.pupper_fc.tests.test_integration import arm, frame, intent, command, is_zero
from final_project.pupper_fc.tests.test_obstacle_integration import camera


def resume(now=.2, frame_seq=2, robot_id="pupper-1", **changes):
    event = dict(event="resume", now=now, epoch="test", clock="replay", robot_id=robot_id,
                 frame_seq=frame_seq)
    event.update(changes)
    return event


def home(goal=0, **changes):
    row = dict(x=-.3 * 3.66, y=0, yaw=0, goal=goal)
    row.update(changes)
    return {"pupper-1": row}


def waiting(*, obstacles=False, kind="auto"):
    executor = Executor(obstacle_config=ObstacleConfig(clear_frames=1) if obstacles else None)
    arm(executor)
    if obstacles:
        executor.handle(camera(0, 1))
    executor.handle(intent(kind=kind))
    result = executor.handle(frame(0, 1, robots=home(goal=1)))
    assert result["decisions"]["pupper-1"]["phase"] == "await_reset"
    executor.handle(frame(.1, 2, robots=home()))
    return executor


@pytest.mark.parametrize("kind", ["auto", "attack"])
def test_public_resume_retains_intent_and_consumes_reset_without_new_epoch(kind):
    executor = waiting(kind=kind)
    previous = executor.intents["pupper-1"]
    result = executor.handle(resume())
    assert result["accepted"] and result["armed"] and result["epoch"] == "test"
    assert result["decisions"]["pupper-1"]["phase"] == "approach"
    assert executor.intents["pupper-1"] == previous
    assert executor.resume_frame_seq["pupper-1"] == 2
    controller = executor.controllers["pupper-1"]
    assert not controller._goal_latched and not controller._reset_requested
    assert controller._strike is None and controller._aligned_since is None
    assert command(result)["expires_at"] <= previous.valid_until


def test_later_goal_latches_normally_and_old_or_repeated_cues_cannot_clear_it():
    executor = waiting()
    assert executor.handle(resume())["accepted"]
    assert not executor.handle(resume(.21))["accepted"]
    executor.handle(frame(.22, 3, robots=home(goal=-1)))
    executor.handle(frame(.23, 4, robots=home()))
    result = executor.handle(resume(.24, frame_seq=2))
    assert not result["accepted"] and "current admitted" in result["reason"]
    assert is_zero(command(result)) and executor.controllers["pupper-1"]._goal_latched
    assert executor.handle(resume(.25, frame_seq=4))["accepted"]
    assert not executor.controllers["pupper-1"]._goal_latched


@pytest.mark.parametrize("changes,reason", [
    (dict(goal=1), "cleared goal"), (dict(goal=-1), "cleared goal"),
    (dict(yaw=.2), "position and heading"), (dict(x=-.8), "position and heading"),
    (dict(fallen=True), "unfallen"),
])
def test_resume_refuses_uncleared_goal_fall_and_displaced_home_pose(changes, reason):
    executor = waiting()
    executor.handle(frame(.2, 3, robots=home(**changes)))
    result = executor.handle(resume(.21, frame_seq=3))
    assert not result["accepted"] and reason in result["reason"]
    assert executor.controllers["pupper-1"]._goal_latched
    assert not executor.controllers["pupper-1"]._reset_requested


def test_resume_refuses_pose_at_home_before_goal_return_was_authorized():
    executor = Executor()
    arm(executor)
    executor.handle(frame(robots=home(goal=1)))
    executor.handle(frame(.1, 2, robots=home()))
    result = executor.handle(resume(.1))
    assert not result["accepted"] and "unexpired intent" in result["reason"]
    assert not executor.controllers["pupper-1"]._home_reached


def test_expired_intent_cannot_be_renewed_or_reset_by_resume():
    executor = waiting()
    executor.handle(frame(.5, 3, robots=home()))
    result = executor.handle(resume(.51, frame_seq=3))
    assert not result["accepted"] and "unexpired intent" in result["reason"]
    assert is_zero(command(result)) and executor.intents["pupper-1"] is None
    assert executor.controllers["pupper-1"]._goal_latched


def test_camera_hold_and_pre_hold_pose_refuse_resume_without_queuing_reset():
    executor = waiting(obstacles=True)
    executor.handle(camera(.2, 2, blocked=True))
    result = executor.handle(resume(.21))
    assert not result["accepted"] and "clear front-camera" in result["reason"]
    assert is_zero(command(result))
    executor.handle(camera(.22, 3))
    result = executor.handle(resume(.23))
    assert not result["accepted"] and "new field observation" in result["reason"]
    assert not executor.controllers["pupper-1"]._reset_requested
    executor.handle(frame(.24, 3, robots=home()))
    assert executor.handle(resume(.25, frame_seq=3))["accepted"]


@pytest.mark.parametrize("frame_seq", [1, 3, -1, True, "2", None])
def test_resume_requires_exact_current_integer_frame_reference(frame_seq):
    executor = waiting()
    result = executor.handle(resume(frame_seq=frame_seq))
    assert not result["accepted"] and executor.controllers["pupper-1"]._goal_latched


def test_stale_frame_refuses_resume_before_lease_expiry():
    executor = waiting()
    result = executor.handle(resume(.451))
    assert not result["accepted"] and "fresh field" in result["reason"]
    assert is_zero(command(result)) and executor.controllers["pupper-1"]._goal_latched


@pytest.mark.parametrize("changes", [dict(epoch="old"), dict(clock="other")])
def test_wrong_session_resume_is_atomic_even_with_extreme_arrival_time(changes):
    executor = waiting()
    before = deepcopy(vars(executor))
    result = executor.handle(resume(999, **changes))
    assert not result["accepted"] and result["decisions"] == {}
    assert executor.last_now == before["last_now"]
    assert executor.intents == before["intents"] and executor.resume_frame_seq == before["resume_frame_seq"]
    assert executor.controllers["pupper-1"]._goal_latched


def test_stop_cannot_be_released_by_resume_and_new_epoch_rejects_old_cue():
    executor = waiting()
    executor.handle(dict(event="stop", now=.2))
    assert not executor.handle(resume(.21))["accepted"] and not executor.armed
    arm(executor, .22, "new")
    assert not executor.handle(resume(999))["accepted"]
    assert executor.last_now == .22 and executor.intents["pupper-1"] is None


def test_each_robot_requires_its_own_resume_and_frame_use_is_independent():
    executor = Executor()
    arm(executor)
    executor.handle(intent(kind="auto"))
    executor.handle(intent(kind="auto", robot_id="pupper-2"))
    robots = {"pupper-1": dict(x=-1.098, y=0, yaw=0, goal=1),
              "pupper-2": dict(x=1.098, y=0, yaw=math.pi, goal=-1)}
    executor.handle(frame(robots=robots))
    for robot in robots.values():
        robot["goal"] = 0
    executor.handle(frame(.1, 2, robots=robots))
    result = executor.handle(resume())
    assert result["accepted"] and not is_zero(command(result))
    assert result["decisions"]["pupper-2"]["phase"] == "await_reset"
    assert executor.handle(resume(.21, robot_id="pupper-2"))["accepted"]


def test_resume_without_goal_is_refused_and_replay_is_deterministic():
    events = [dict(event="arm", now=0, epoch="test", clock="replay"), intent(kind="auto"),
              frame(robots=home(goal=1)), frame(.1, 2, robots=home()), resume(), resume(.21)]
    outputs = list(replay(events))
    assert outputs == list(replay(events))
    assert outputs[-2]["accepted"] and not outputs[-1]["accepted"]
    executor = Executor()
    arm(executor)
    executor.handle(intent())
    executor.handle(frame(.1, 2, robots=home()))
    result = executor.handle(resume())
    assert not result["accepted"] and "return is not complete" in result["reason"]
