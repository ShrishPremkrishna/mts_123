"""Shadow/replay integration: no ROS, network, or robot dependencies."""
from dataclasses import asdict
from copy import deepcopy
import json
import math

import pytest

from final_project.pupper_fc.controller import Controller
from final_project.pupper_fc.integration import Executor, main, replay
from final_project.pupper_fc.kick_handoff import KickHandoff
from final_project.pupper_fc.model import Ball, Limits, Observation, Pose


def arm(executor, now=0, epoch="test"):
    return executor.handle(dict(event="arm", now=now, epoch=epoch, clock="replay"))


def frame(now=0, seq=1, **changes):
    result = dict(event="frame", now=now, epoch="test", clock="replay", seq=seq,
                  captured_at=now, robots={"pupper-1": dict(x=-1, y=0, yaw=0),
                                           "pupper-2": dict(x=1, y=0, yaw=math.pi)},
                  ball=dict(x=0, y=0), goals={"left": dict(x=-1.83, y=0), "right": dict(x=1.83, y=0)})
    result.update(changes)
    return result


def intent(now=0, seq=1, robot_id="pupper-1", kind="attack", **changes):
    result = dict(event="intent", now=now, epoch="test", clock="replay", seq=seq,
                  issued_at=now, valid_until=now + 0.5, robot_id=robot_id,
                  intent_id="play", kind=kind)
    result.update(changes)
    return result


def command(output, robot="pupper-1"):
    return output["decisions"][robot]["command"]


def is_zero(cmd):
    return (cmd["vx"], cmd["vy"], cmd["wz"]) == (0, 0, 0)


def active():
    executor = Executor()
    assert arm(executor)["accepted"]
    assert executor.handle(frame())["accepted"]
    assert not is_zero(command(executor.handle(intent())))
    return executor


@pytest.mark.parametrize("kind", ["move", "shoot"])
def test_foot_mode_continuously_refines_one_target_without_resetting_the_walker(kind):
    executor = Executor(kick_side="right")
    arm(executor)
    owner = KickHandoff("pupper-1")
    commands = []
    for seq in range(1, 12):
        now = seq * 0.04
        target = dict(x=1.2, y=seq * 0.002)
        if kind == "move":
            target["yaw"] = 0.15
        executor.handle(frame(now, seq))
        preview = executor.handle(intent(now, seq, kind=kind, target=target))
        assert preview["accepted"] and is_zero(command(preview))
        commands.append(executor.kick_step(owner, now).decision.command.vx)
    assert commands[0] == 0.0 and commands[-1] >= 0.20
    # A different owner's motion starts from a fresh ramp.
    executor.handle(intent(now, seq + 1, kind=kind, target=target, intent_id="replacement"))
    assert executor.kick_step(owner, now).decision.command.vx == 0.0
    with pytest.raises(ValueError, match="another kick handoff"):
        executor.kick_step(KickHandoff("pupper-1"), now)


def striking():
    executor = Executor()
    arm(executor)
    executor.handle(frame(robots={"pupper-1": dict(x=-0.45, y=0, yaw=0)}))
    executor.handle(intent())
    for seq, now in enumerate((0.1, 0.25), 2):
        output = executor.handle(frame(now, seq, robots={"pupper-1": dict(x=-0.45, y=0, yaw=0)}))
    assert output["decisions"]["pupper-1"]["phase"] == "strike"
    return executor


def test_attack_reuses_baseline_and_robot_ids_are_isolated():
    executor = active()
    baseline = Controller().tick(Observation(0, Pose(-1, 0, 0), Ball(0, 0)), 0)
    assert executor.tick(0)["pupper-1"] == baseline
    assert is_zero(asdict(executor.tick(0)["pupper-2"].command))
    result = executor.handle(intent(robot_id="pupper-2"))
    for robot in ("pupper-1", "pupper-2"):
        assert command(result, robot)["robot_id"] == robot
        assert command(result, robot)["vx"] > 0
    result = executor.handle(intent(0.1, 2, kind="defend", target=dict(x=-1, y=0.3, yaw=0)))
    assert result["decisions"]["pupper-1"]["phase"] == "defend"
    assert result["decisions"]["pupper-2"]["phase"] == "approach"


def test_pose_target_uses_body_coordinates_and_configured_limits_without_ball():
    executor = Executor(limits=Limits(vx=0.2, vy=0.1, wz=0.3))
    arm(executor)
    executor.handle(frame(robots={"pupper-1": dict(x=0, y=0, yaw=math.pi / 2)}, ball=None))
    result = executor.handle(intent(kind="move", target=dict(x=0.2, y=0, yaw=math.pi / 2)))
    cmd = command(result)
    assert cmd["vx"] == pytest.approx(0, abs=1e-12)
    assert cmd["vy"] == pytest.approx(-0.1)
    assert cmd["wz"] == 0
    assert cmd["expires_at"] <= 0.15


def test_supplied_shot_destination_changes_existing_alignment_geometry():
    executor = Executor()
    arm(executor)
    executor.handle(frame(robots={"pupper-1": dict(x=0, y=-0.6, yaw=math.pi / 2)}))
    result = executor.handle(intent(kind="shoot", target=dict(x=0, y=1)))
    decision = result["decisions"]["pupper-1"]
    assert decision["phase"] == "align"
    assert decision["target"] == pytest.approx((0, -0.45))
    assert decision["target_heading"] == pytest.approx(math.pi / 2)


def test_new_frames_and_same_intent_renewal_preserve_alignment_dwell():
    executor = striking()
    result = executor.handle(intent(0.26, 2))
    assert result["decisions"]["pupper-1"]["phase"] == "strike"


@pytest.mark.parametrize("replacement", [
    dict(kind="move", target=dict(x=-0.45, y=0, yaw=math.pi)),
    dict(kind="shoot", target=dict(x=1.83, y=0.5)),
    dict(kind="attack", intent_id="new-play"),
])
def test_replacement_cancels_old_strike_on_same_tick(replacement):
    executor = striking()
    result = executor.handle(intent(0.26, 2, **replacement))
    assert result["decisions"]["pupper-1"]["phase"] != "strike"
    assert executor.controllers["pupper-1"]._strike is None


def test_expired_intent_never_falls_back_to_autonomous_attack():
    executor = active()
    executor.handle(frame(0.49, 2))
    result = executor.handle(dict(event="tick", now=0.5))
    assert result["decisions"]["pupper-1"]["phase"] == "intent_lost"
    assert is_zero(command(result))
    result = executor.handle(frame(0.51, 3))
    assert is_zero(command(result))


def test_silent_scheduler_gap_then_renewal_cannot_resume_old_strike():
    executor = striking()
    executor.handle(frame(0.51, 4, robots={"pupper-1": dict(x=-0.45, y=0, yaw=0)}))
    result = executor.handle(intent(0.51, 2))
    assert result["decisions"]["pupper-1"]["phase"] == "align"


def test_stop_preempts_strike_and_requires_new_epoch_and_fresh_inputs():
    executor = striking()
    stopped = executor.handle(dict(event="stop", now=0.26))
    assert all(is_zero(row["command"]) for row in stopped["decisions"].values())
    assert not executor.handle(intent(0.27, 2))["accepted"]
    assert not arm(executor, 0.28)["accepted"]
    assert arm(executor, 0.29, "new")["accepted"]
    assert not executor.handle(intent(0.3, 3))["accepted"]
    assert is_zero(command(executor.handle(dict(event="tick", now=0.3))))
    assert executor.handle(frame(0.31, epoch="new"))["accepted"]
    assert executor.handle(intent(0.31, epoch="new"))["accepted"]


@pytest.mark.parametrize("event_factory", [frame, intent])
@pytest.mark.parametrize("packet_now", [0, 999, pytest.param(10**1000, id="huge-time")])
def test_old_epoch_packets_cannot_touch_active_clock_or_execution(event_factory, packet_now):
    executor = Executor()
    arm(executor, 10, "current")
    executor.handle(frame(10, epoch="current"))
    executor.handle(intent(10, epoch="current"))

    def state():
        return (executor.last_now, executor.armed, executor.frame, executor.frame_seq,
                executor.capture_highwater, executor.intents, executor.intent_seq,
                {r: vars(c) for r, c in executor.controllers.items()})

    before = deepcopy(state())
    event = event_factory(0, epoch="old")
    event["now"] = packet_now
    result = executor.handle(event)
    assert not result["accepted"] and result["decisions"] == {}
    assert result["now"] == 10
    assert state() == before
    assert not is_zero(asdict(executor.tick(10.2)["pupper-1"].command))


def test_wrong_clock_packet_does_not_advance_current_session():
    executor = active()
    result = executor.handle(frame(999, 2, clock="unmapped-source"))
    assert not result["accepted"] and result["decisions"] == {}
    assert executor.last_now == 0
    assert not is_zero(asdict(executor.tick(0.1)["pupper-1"].command))


@pytest.mark.parametrize("packet_now", [0, 999, pytest.param(10**1000, id="huge-time")])
def test_rejected_reused_epoch_arm_leaves_active_session_untouched(packet_now):
    executor = Executor()
    arm(executor, 10, "current")
    executor.handle(frame(10, epoch="current"))
    executor.handle(intent(10, epoch="current"))

    def state():
        return (executor.last_now, executor.armed, executor.frame, executor.intents,
                executor.intent_seq, executor.used_epochs,
                {r: vars(c) for r, c in executor.controllers.items()})

    before = deepcopy(state())
    result = arm(executor, packet_now, "current")
    assert not result["accepted"] and result["decisions"] == {}
    assert result["now"] == 10 and state() == before
    assert not is_zero(asdict(executor.tick(10.2)["pupper-1"].command))


def test_half_second_lease_tolerates_subtraction_roundoff_without_extra_time():
    executor = Executor()
    arm(executor)
    now = 0.5000000000000003
    assert (now + 0.5) - now > 0.5  # the exact previously rejected reproduction
    assert executor.handle(intent(now))["accepted"]
    assert executor.intents["pupper-1"].valid_until == now + 0.5
    result = executor.handle(dict(event="tick", now=now + 0.5))
    assert result["decisions"]["pupper-1"]["phase"] == "intent_lost"
    assert is_zero(command(result))


def test_lease_tolerance_is_clamped_and_does_not_extend_expiry_or_allow_longer_lease():
    executor = active()
    assert executor.handle(intent(0, 2, valid_until=0.5 + 5e-13))["accepted"]
    assert executor.intents["pupper-1"].valid_until == 0.5
    result = executor.handle(intent(0.5, 3, issued_at=0, valid_until=0.5 + 5e-13))
    assert not result["accepted"] and is_zero(command(result))
    result = executor.handle(intent(0.5, 4, valid_until=1.0 + 1e-8))
    assert not result["accepted"]


def test_changed_fixed_goal_geometry_preempts_current_attack_including_small_jitter():
    executor = striking()
    result = executor.handle(frame(0.26, 4, robots={"pupper-1": dict(x=-0.45, y=0, yaw=0)},
                                   goals={"right": dict(x=1.83, y=1e-7)}))
    assert result["decisions"]["pupper-1"]["phase"] == "align"
    assert executor.controllers["pupper-1"]._strike is None


def test_intent_and_frame_expiry_cap_output_deadline():
    executor = Executor()
    arm(executor)
    executor.handle(frame())
    result = executor.handle(intent(0.3, valid_until=0.34))
    assert command(result)["expires_at"] == pytest.approx(0.34)
    result = executor.handle(intent(0.31, 2, valid_until=0.8))
    assert command(result)["expires_at"] == pytest.approx(0.35)
    assert is_zero(command(executor.handle(dict(event="tick", now=0.35))))


@pytest.mark.parametrize("changes", [dict(captured_at=-1), dict(captured_at=1),
    dict(captured_at=float("nan")), dict(robots={"pupper-1": dict(x=float("inf"), y=0, yaw=0)}),
    dict(ball=dict(x=0, y=float("nan"))), dict(seq="bad")])
def test_bad_new_frame_revokes_shared_motion(changes):
    executor = active()
    event = frame(0.1, 2)
    event.update(changes)
    result = executor.handle(event)
    assert not result["accepted"]
    assert is_zero(command(result))


@pytest.mark.parametrize("changes", [dict(valid_until=0.05), dict(valid_until=5),
    dict(issued_at=1), dict(valid_until=float("inf")), dict(kind="fly"),
    dict(kind="move", target=dict(x=0, y=0, yaw=float("nan"))), dict(seq="bad")])
def test_bad_new_intent_revokes_only_its_robot(changes):
    executor = active()
    executor.handle(intent(robot_id="pupper-2"))
    event = intent(0.1, 2)
    event.update(changes)
    result = executor.handle(event)
    assert not result["accepted"]
    assert is_zero(command(result))
    assert not is_zero(command(result, "pupper-2"))


def test_duplicate_and_reordered_packets_cannot_refresh_lifetimes():
    executor = active()
    assert not executor.handle(frame(0.2, 1))["accepted"]
    assert not executor.handle(intent(0.2, 1))["accepted"]
    result = executor.handle(dict(event="tick", now=0.36))
    assert is_zero(command(result))
    assert executor.frame[0] == 0
    assert executor.intents["pupper-1"].valid_until == 0.5


def test_new_sequence_with_repeated_capture_cannot_restart_tracking():
    executor = active()
    result = executor.handle(frame(0.1, 2, captured_at=0))
    assert not result["accepted"] and is_zero(command(result))


def test_missing_goal_robot_and_fall_stop_without_fallback():
    executor = active()
    assert is_zero(command(executor.handle(frame(0.1, 2, goals={}))))
    assert is_zero(command(executor.handle(frame(0.2, 3, robots={"pupper-1": None}))))
    result = executor.handle(frame(0.3, 4, robots={"pupper-1": dict(x=-1, y=0, yaw=0, fallen=True)}))
    assert result["decisions"]["pupper-1"]["recovery_requested"]
    assert is_zero(command(result))


@pytest.mark.parametrize("now", [-0.1, float("nan"), True])
def test_bad_or_backward_control_clock_latches_stop(now):
    executor = active()
    output = executor.handle(dict(event="tick", now=now))
    assert not output["armed"]
    assert all(is_zero(row["command"]) for row in output["decisions"].values())


def test_json_huge_integer_is_rejected_without_overflow():
    huge = 10**1000
    executor = active()
    event = json.loads(json.dumps(frame(0.1, 2, ball=dict(x=huge, y=0))))
    result = executor.handle(event)
    assert not result["accepted"] and is_zero(command(result))
    result = executor.handle(json.loads(json.dumps(dict(event="tick", now=huge))))
    assert not result["accepted"] and not result["armed"]
    assert all(is_zero(row["command"]) for row in result["decisions"].values())


@pytest.mark.parametrize("kwargs", [dict(shot_target=(10**1000, 0)),
                                    dict(target_pose=Pose(10**1000, 0, 0))])
def test_direct_controller_huge_target_is_zero_instead_of_overflow(kwargs):
    decision = Controller().tick(Observation(0, Pose(-1, 0, 0), Ball(0, 0)), 0, **kwargs)
    assert decision.phase == "invalid_intent"
    assert is_zero(asdict(decision.command))


def test_direct_controller_target_replacement_cancels_strike_and_rejects_bad_target():
    controller = Controller()
    for now in (0, 0.25):
        decision = controller.tick(Observation(now, Pose(-0.45, 0, 0), Ball(0, 0)), now)
    assert decision.phase == "strike"
    decision = controller.tick(Observation(0.26, Pose(-0.45, 0, 0), Ball(0, 0)), 0.26,
                               target_pose=Pose(-0.45, 0, math.pi))
    assert decision.phase == "move" and controller._strike is None
    for kwargs in (dict(target_pose=Pose(100, 0, 0)), dict(shot_target=(float("nan"), 0)),
                   dict(shot_target=(0, 0)), dict(target_pose=Pose(0, 0, 0), shot_target=(1, 0))):
        decision = controller.tick(Observation(0.3, Pose(-0.45, 0, 0), Ball(0, 0)), 0.3, **kwargs)
        assert is_zero(asdict(decision.command))


def test_jsonl_cli_is_deterministic_shadow_replay(tmp_path, capsys):
    events = [dict(event="arm", now=0, epoch="test", clock="replay"), frame(), intent(),
              dict(event="tick", now=0.05), dict(event="stop", now=0.1)]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(row) for row in events))
    assert main(["--replay", str(path)]) == 0
    outputs = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert outputs == json.loads(json.dumps(list(replay(events))))
    assert not outputs[-1]["armed"]
    assert is_zero(command(outputs[-1]))


@pytest.mark.parametrize("kind", ["attack", "auto"])
@pytest.mark.parametrize("goal", [-1, 1])
def test_goal_pulse_during_expired_lease_survives_clear_before_renewal(kind, goal):
    executor = Executor()
    arm(executor)
    executor.handle(frame())
    executor.handle(intent(kind=kind))
    result = executor.handle(frame(.51, 2, robots={"pupper-1": dict(x=-1, y=0, yaw=0, goal=goal)}))
    assert result["decisions"]["pupper-1"]["phase"] == "intent_lost"
    assert is_zero(command(result))
    assert executor.controllers["pupper-1"]._goal_latched
    result = executor.handle(frame(.52, 3))  # referee pulse ends before authority returns
    assert is_zero(command(result))
    result = executor.handle(intent(.53, 2, kind=kind))
    assert result["decisions"]["pupper-1"]["phase"] == "return_home"
    assert executor.controllers["pupper-1"]._goal_latched


@pytest.mark.parametrize("goal", [-1, 1])
def test_goal_pulse_during_camera_hold_survives_clear_and_waits_for_fresh_pose(goal):
    from final_project.pupper_fc.obstacles import ObstacleConfig
    from final_project.pupper_fc.tests.test_obstacle_integration import camera

    executor = Executor(obstacle_config=ObstacleConfig(clear_frames=1))
    arm(executor)
    executor.handle(camera(0, 1))
    executor.handle(frame())
    executor.handle(intent())
    executor.handle(camera(.1, 2, blocked=True))
    pulse = frame(.11, 2, robots={"pupper-1": dict(x=-1, y=0, yaw=0, goal=goal)})
    for event in (pulse, frame(.12, 3)):
        result = executor.handle(event)
        assert result["decisions"]["pupper-1"]["phase"] == "obstacle_hold"
        assert is_zero(command(result))
        assert executor.controllers["pupper-1"]._goal_latched
    result = executor.handle(camera(.13, 3))
    assert result["decisions"]["pupper-1"]["phase"] == "tracking_lost"
    assert is_zero(command(result))
    result = executor.handle(frame(.14, 4))
    assert result["decisions"]["pupper-1"]["phase"] == "return_home"


@pytest.mark.parametrize("kind", ["attack", "auto", "move", "defend", "shoot"])
def test_referee_preempts_every_action_before_target_or_goal_geometry(kind):
    executor = Executor()
    arm(executor)
    result = executor.handle(frame(robots={"pupper-1": dict(x=-1, y=0, yaw=0, goal=1)},
                                   ball=None, goals={}))
    assert is_zero(command(result))  # bookkeeping does not grant motion authority
    executor.handle(frame(.1, 2, ball=None, goals={}))  # clear flag cannot erase latch
    kwargs = (dict(target=dict(x=100, y=100, yaw=0)) if kind in ("move", "defend")
              else dict(target=dict(x=100, y=100)) if kind == "shoot" else {})
    result = executor.handle(intent(.1, kind=kind, **kwargs))
    assert result["accepted"]
    assert result["decisions"]["pupper-1"]["phase"] == "return_home"


@pytest.mark.parametrize("second_changes", [dict(x=float("nan")), dict(fallen="yes"),
                                             dict(goal=True), dict(x=100)])
def test_later_invalid_robot_row_cannot_partially_latch_first_robot_goal(second_changes):
    executor = Executor()
    arm(executor)
    executor.handle(frame())
    event = frame(.1, 2)
    event["robots"]["pupper-1"]["goal"] = 1
    event["robots"]["pupper-2"].update(second_changes)
    result = executor.handle(event)
    assert not result["accepted"] and is_zero(command(result))
    assert not executor.controllers["pupper-1"]._goal_latched
    executor.handle(frame(.2, 3))
    result = executor.handle(intent(.2))
    assert result["decisions"]["pupper-1"]["phase"] == "approach"


@pytest.mark.parametrize("changes", [dict(epoch="old"), dict(clock="unmapped"), dict(seq=1),
                                      dict(captured_at=0), dict(captured_at=-.4), dict(captured_at=.2),
                                      dict(ball=dict(x=0, y=0, vx=11))])
def test_unadmitted_goal_pulse_cannot_change_referee_state(changes):
    executor = active()
    event = frame(.1, 2, robots={"pupper-1": dict(x=-1, y=0, yaw=0, goal=1)})
    event.update(changes)
    result = executor.handle(event)
    assert not result["accepted"]
    assert not executor.controllers["pupper-1"]._goal_latched


def test_stop_rejects_goal_bookkeeping_and_rearm_starts_fresh_referee_state():
    executor = active()
    executor.handle(dict(event="stop", now=.1))
    pulse = frame(.2, 2, robots={"pupper-1": dict(x=-1, y=0, yaw=0, goal=1)})
    assert not executor.handle(pulse)["accepted"]
    assert not executor.controllers["pupper-1"]._goal_latched
    arm(executor, .3, "new")
    executor.handle(frame(.31, epoch="new"))
    result = executor.handle(intent(.31, epoch="new"))
    assert result["decisions"]["pupper-1"]["phase"] == "approach"


def test_active_explicit_attack_handles_goal_without_ball_or_goal_points():
    executor = active()
    result = executor.handle(frame(.1, 2, robots={"pupper-1": dict(x=-1, y=0, yaw=0, goal=1)},
                                   ball=None, goals={}))
    assert result["accepted"]
    assert result["decisions"]["pupper-1"]["phase"] == "return_home"
