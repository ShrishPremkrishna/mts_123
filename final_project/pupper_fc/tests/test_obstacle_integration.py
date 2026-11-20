"""Camera authority must interrupt the actual executor, not a lower-priority stream."""
import pytest

from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.obstacles import ObstacleConfig
from final_project.pupper_fc.tests.test_integration import arm, frame, intent, command, is_zero


def camera(now, sequence, robot="pupper-1", blocked=False, **changes):
    row = dict(event="obstacles", epoch="test", clock="replay", now=now,
               seq=sequence, captured_at=now, robot_id=robot, valid=True, healthy=True,
               format="normalized_xyxy_v1", detections=[])
    if blocked:
        row["detections"] = [dict(box=[.2, .2, .8, .95], confidence=.9, class_id="0")]
    row.update(changes)
    return row


def ready(*, staged=False):
    executor = Executor(obstacle_config=ObstacleConfig())
    arm(executor)
    for seq in range(1, 4):
        assert executor.handle(camera(seq * .01, seq))["accepted"]
    robots = {"pupper-1": dict(x=-.45 if staged else -1, y=0, yaw=0)}
    executor.handle(frame(.03, robots=robots))
    executor.handle(intent(.03))
    return executor


def test_configured_camera_fails_closed_before_any_detection():
    executor = Executor(obstacle_config=ObstacleConfig())
    arm(executor)
    executor.handle(frame())
    result = executor.handle(intent())
    assert result["decisions"]["pupper-1"]["phase"] == "obstacle_hold"
    assert is_zero(command(result))


@pytest.mark.parametrize("kind", ["attack", "move", "defend", "shoot"])
def test_hazard_preempts_each_action_kind_on_same_tick(kind):
    executor = ready()
    target = (dict(x=0, y=.3, yaw=0) if kind in ("move", "defend") else dict(x=1.83, y=0))
    changes = {} if kind == "attack" else dict(target=target)
    executor.handle(intent(.04, 2, kind=kind, **changes))
    result = executor.handle(camera(.05, 4, blocked=True))
    assert result["accepted"]
    assert is_zero(command(result))
    assert result["decisions"]["pupper-1"]["phase"] == "obstacle_hold"


def test_clearance_cannot_revive_old_strike_or_use_pre_hold_pose():
    executor = ready(staged=True)
    for seq, now in enumerate((.13, .23, .29), 4):
        executor.handle(camera(now, seq))
        result = executor.handle(frame(now, seq, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
    assert result["decisions"]["pupper-1"]["phase"] == "strike"
    executor.handle(camera(.30, 7, blocked=True))
    assert executor.controllers["pupper-1"]._strike is None
    for seq, now in enumerate((.31, .32, .33), 8):
        result = executor.handle(camera(now, seq))
    assert is_zero(command(result))
    assert result["decisions"]["pupper-1"]["phase"] == "tracking_lost"
    result = executor.handle(frame(.34, 10, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
    assert result["decisions"]["pupper-1"]["phase"] == "align"
    assert executor.controllers["pupper-1"]._strike is None


def test_camera_expiry_limits_command_and_cancels_while_field_is_fresh():
    executor = ready()
    executor.handle(frame(.30, 2))
    result = executor.handle(dict(event="tick", now=.30))
    assert command(result)["expires_at"] <= .03 + ObstacleConfig().max_frame_age_s
    result = executor.handle(dict(event="tick", now=.39))
    assert is_zero(command(result))
    assert result["decisions"]["pupper-1"]["phase"] == "obstacle_hold"


@pytest.mark.parametrize("changes", [dict(detections=None), dict(healthy=None),
                                      dict(detections=[dict(box=[0, 1], confidence=.9)]),
                                      dict(captured_at=float("nan")), dict(format="pixels"),
                                      dict(seq=2)])
def test_malformed_or_replayed_camera_packet_cannot_leave_motion_alive(changes):
    executor = ready()
    result = executor.handle(camera(.04, 4, **changes))
    assert not result["accepted"]
    assert is_zero(command(result))


def test_old_epoch_camera_packet_has_no_effect_on_current_execution():
    executor = ready()
    before = executor.tick(.04)
    result = executor.handle(camera(999, 4, blocked=True, epoch="old"))
    assert not result["accepted"] and not result["decisions"]
    assert executor.last_now == .04
    assert executor.tick(.04) == before


def test_obstacle_clear_cannot_override_stop_or_resurrect_expired_intent():
    executor = ready()
    executor.handle(camera(.04, 4, blocked=True))
    for seq, now in enumerate((.54, .55, .56), 5):
        executor.handle(camera(now, seq))
    result = executor.handle(frame(.56, 2))
    assert result["decisions"]["pupper-1"]["phase"] == "intent_lost"
    executor.handle(dict(event="stop", now=.57))
    result = executor.handle(camera(.58, 8))
    assert not result["accepted"]
    assert not executor.armed
    assert all(is_zero(vars(d.command)) for d in executor.tick(.58).values())


def test_each_robot_has_independent_camera_authority():
    executor = ready()
    for seq, now in enumerate((.04, .05, .06), 1):
        executor.handle(camera(now, seq, robot="pupper-2"))
    executor.handle(frame(.06, 2))
    executor.handle(intent(.06, robot_id="pupper-2"))
    result = executor.handle(camera(.07, 4, blocked=True))
    assert is_zero(command(result))
    assert not is_zero(command(result, "pupper-2"))


def test_camera_gap_cancels_strike_even_if_one_new_frame_reopens_gate():
    executor = Executor(obstacle_config=ObstacleConfig(clear_frames=1))
    arm(executor)
    executor.handle(camera(.01, 1))
    executor.handle(frame(.01, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
    executor.handle(intent(.01))
    for seq, now in enumerate((.1, .2, .3), 2):
        result = executor.handle(frame(now, seq, robots={"pupper-1": dict(x=-.45, y=0, yaw=0)}))
    assert result["decisions"]["pupper-1"]["phase"] == "strike"
    result = executor.handle(camera(.37, 2))
    assert is_zero(command(result))
    assert executor.controllers["pupper-1"]._strike is None
    assert result["decisions"]["pupper-1"]["phase"] == "tracking_lost"


@pytest.mark.parametrize("target", [dict(x=-1.3,y=0,yaw=0), dict(x=-1,y=.3,yaw=0),
                                   dict(x=-1,y=0,yaw=.5), dict(x=-.7,y=.2,yaw=.5)])
def test_obstacle_hold_zeroes_reverse_lateral_turn_and_combined_motion(target):
    executor = ready()
    result = executor.handle(intent(.04, 2, kind="move", target=target))
    assert not is_zero(command(result))
    result = executor.handle(camera(.05, 4, blocked=True))
    assert is_zero(command(result))
