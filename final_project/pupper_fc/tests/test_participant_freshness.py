"""Individual track age fences; these do not establish opponent clearance."""
from copy import deepcopy
import math

import pytest

from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.obstacles import ObstacleConfig
from final_project.pupper_fc.tests.test_integration import arm, frame, intent, command, is_zero
from final_project.pupper_fc.tests.test_obstacle_integration import camera
from final_project.pupper_fc.tests.test_resume_integration import resume


OWN, PEER = "pupper-1", "pupper-2"
PARTICIPANTS = (OWN, PEER)


def tracked(now=0., seq=1, *, captures=None, robots=None, **changes):
    packet = frame(now, seq, **changes)
    if robots is not None:
        packet["robots"] = deepcopy(robots)
    for robot, row in packet["robots"].items():
        if row is not None:
            row["captured_at"] = (captures or {}).get(robot, now)
    return packet


def action(kind="move", now=0., seq=1, **changes):
    target = {}
    if kind in ("move", "defend"):
        target = dict(target=dict(x=-.8, y=.4, yaw=0.))
    elif kind == "shoot":
        target = dict(target=dict(x=1.83, y=0.))
    return intent(now, seq, kind=kind, **target, **changes)


def active(kind="move", *, required=PARTICIPANTS):
    executor = Executor(required_participants=required)
    arm(executor)
    assert executor.handle(tracked())["accepted"]
    assert executor.handle(action(kind))["accepted"]
    return executor


@pytest.mark.parametrize("participants", [None, "pupper-1", {OWN}, [OWN, OWN], ["unknown"], [True]])
def test_required_participants_reject_ambiguous_or_unknown_configuration(participants):
    with pytest.raises(ValueError, match="participant"):
        Executor(required_participants=participants)


@pytest.mark.parametrize("participants", [(), [], PARTICIPANTS, list(PARTICIPANTS), (PEER,)])
def test_required_participants_accept_explicit_distinct_configured_ids(participants):
    executor = Executor(required_participants=participants)
    assert executor.required_participants == tuple(participants)


def test_default_contract_keeps_synchronous_solo_inputs_and_optional_capture_is_honored():
    executor = Executor()
    arm(executor)
    solo = {OWN: dict(x=-1., y=0., yaw=0.)}
    assert executor.handle(frame(robots=solo))["accepted"]
    assert not is_zero(command(executor.handle(action())))
    result = executor.handle(tracked(.36, 2, robots=solo, captures={OWN: 0.}))
    assert result["accepted"] and is_zero(command(result))
    assert executor.frame[1][OWN].captured_at == 0.
    assert executor.frame_evidence_deadline(OWN) == pytest.approx(.35)


@pytest.mark.parametrize("robot", PARTICIPANTS)
@pytest.mark.parametrize("capture", [None, True, "0", math.nan, math.inf, .00001])
def test_present_required_track_requires_finite_nonfuture_capture(robot, capture):
    executor = Executor(required_participants=PARTICIPANTS)
    arm(executor)
    packet = tracked()
    packet["robots"][robot]["captured_at"] = capture
    result = executor.handle(packet)
    assert not result["accepted"] and executor.frame is None
    assert all(stamp == -math.inf for stamp in executor.track_capture_highwater.values())


def test_missing_required_capture_rejected_but_unrequired_row_may_use_frame_stamp():
    executor = Executor(required_participants=(PEER,))
    arm(executor)
    packet = frame()
    assert not executor.handle(packet)["accepted"]
    packet = frame(.1, 2)
    packet["robots"][PEER]["captured_at"] = .1
    assert executor.handle(packet)["accepted"]
    assert executor.frame[1][OWN].captured_at == .1


@pytest.mark.parametrize("absent", ["omitted", "null"])
@pytest.mark.parametrize("kind", ["move", "defend", "attack", "shoot", "auto"])
def test_missing_required_peer_holds_every_action_without_renewing_lease(absent, kind):
    executor = active(kind)
    previous_intent = executor.intents[OWN]
    previous_role = executor.tactics[OWN].role
    packet = tracked(.1, 2)
    if absent == "omitted":
        del packet["robots"][PEER]
    else:
        packet["robots"][PEER] = None
    result = executor.handle(packet)
    assert result["accepted"] and is_zero(command(result))
    assert result["decisions"][OWN]["phase"] == "participant_hold"
    assert PEER in result["decisions"][OWN]["reason"]
    assert executor.intents[OWN] == previous_intent
    assert executor.frame_evidence_deadline(OWN) is None
    assert result["decisions"][OWN]["role"] == (previous_role if kind == "auto" else None)
    assert executor.controllers[OWN]._strike is None and executor.controllers[OWN]._aligned_since is None
    executor.tick(.5)
    assert executor.intents[OWN] is None  # Frames/holds never extend the action.


@pytest.mark.parametrize("stale_robot", PARTICIPANTS)
def test_new_envelopes_cannot_refresh_old_track_or_extend_command_deadline(stale_robot):
    executor = active()
    result = executor.handle(tracked(.3, 2, captures={stale_robot: 0.}))
    assert result["accepted"] and not is_zero(command(result))
    assert executor.frame[1][stale_robot].captured_at == 0.
    assert command(result)["expires_at"] == pytest.approx(.35)
    assert executor.frame_evidence_deadline(OWN) == pytest.approx(.35)
    result = executor.handle(tracked(.35, 3, captures={stale_robot: 0.}))
    assert result["accepted"] and is_zero(command(result))
    assert result["decisions"][OWN]["phase"] == "participant_hold"


def test_observations_retain_original_deferred_track_captures():
    executor = Executor(required_participants=PARTICIPANTS)
    arm(executor)
    packet = tracked(.2, captures={OWN: .15, PEER: .1})
    assert executor.handle(packet)["accepted"]
    assert executor.frame[1][OWN].captured_at == .15
    assert executor.frame[1][PEER].captured_at == .1
    assert executor.frame_evidence_deadline(OWN) == pytest.approx(.45)


@pytest.mark.parametrize("required", [(), PARTICIPANTS])
@pytest.mark.parametrize("robot", PARTICIPANTS)
def test_enclosing_frame_cannot_predate_any_supplied_pose_capture(required, robot):
    executor = Executor(required_participants=required)
    arm(executor)
    packet = tracked(.2, captures={OWN: .15, PEER: .15})
    packet["captured_at"] = .15
    packet["robots"][robot]["captured_at"] = .2
    result = executor.handle(packet)
    assert not result["accepted"] and "enclosing frame" in result["reason"]
    assert executor.frame is None
    assert all(stamp == -math.inf for stamp in executor.track_capture_highwater.values())


@pytest.mark.parametrize("required", [(), PARTICIPANTS])
def test_one_cached_pose_cannot_gain_alignment_dwell_from_older_envelopes(required):
    executor = Executor(required_participants=required)
    arm(executor)
    staged = {OWN: dict(x=-.45, y=0., yaw=0.), PEER: dict(x=1., y=0., yaw=math.pi)}
    first = tracked(.3, robots=staged, captures={OWN: .3, PEER: 0.})
    first["captured_at"] = 0.
    assert not executor.handle(first)["accepted"]
    executor.handle(action("attack", now=.3))
    second = tracked(.31, 2, robots=staged, captures={OWN: .3, PEER: .25})
    second["captured_at"] = .25
    result = executor.handle(second)
    assert not result["accepted"] and is_zero(command(result))
    assert executor.controllers[OWN]._aligned_since is None
    assert executor.controllers[OWN]._strike is None


@pytest.mark.parametrize("change", ["backwards", "pose"])
def test_invalid_peer_sample_atomically_invalidates_frame_without_advancing_other_tracks(change):
    executor = active()
    assert executor.handle(tracked(.1, 2))["accepted"]
    captures, identities = deepcopy(executor.track_capture_highwater), deepcopy(executor.track_identity)
    packet = tracked(.2, 3, captures={PEER: .05 if change == "backwards" else .1})
    packet["robots"][OWN]["goal"] = 1
    if change == "pose":
        packet["robots"][PEER]["x"] = .4
    result = executor.handle(packet)
    assert not result["accepted"] and executor.frame is None and is_zero(command(result))
    assert executor.track_capture_highwater == captures and executor.track_identity == identities
    assert not executor.controllers[OWN]._goal_latched
    assert not executor.controllers[PEER]._goal_latched


def test_new_frame_goal_and_fall_flags_are_admitted_with_repeated_pose_capture():
    executor = active()
    packet = tracked(.36, 2, captures={OWN: 0.})
    packet["robots"][OWN].update(goal=1, fallen=True)
    result = executor.handle(packet)
    assert result["accepted"] and is_zero(command(result))
    assert result["decisions"][OWN]["phase"] == "participant_hold"
    assert result["decisions"][OWN]["recovery_requested"]
    assert executor.frame[1][OWN].captured_at == 0.
    assert executor.frame[1][OWN].fallen
    assert executor.controllers[OWN]._goal_latched
    packet = tracked(.37, 3)
    packet["robots"][OWN]["fallen"] = True
    result = executor.handle(packet)
    assert result["accepted"] and result["decisions"][OWN]["recovery_requested"]
    assert executor.controllers[OWN]._goal_latched


@pytest.mark.parametrize("peer_fault", ["missing", "stale"])
def test_participant_hold_preserves_own_fall_recovery_request(peer_fault):
    executor = active()
    now = .1 if peer_fault == "missing" else .36
    packet = tracked(now, 2, captures={PEER: 0.})
    packet["robots"][OWN]["fallen"] = True
    if peer_fault == "missing":
        del packet["robots"][PEER]
    result = executor.handle(packet)
    assert result["accepted"] and result["decisions"][OWN]["phase"] == "participant_hold"
    assert result["decisions"][OWN]["recovery_requested"]
    assert is_zero(command(result))


def test_missing_track_does_not_erase_highwater_but_new_epoch_does():
    executor = active()
    packet = tracked(.1, 2)
    packet["robots"][PEER] = None
    assert executor.handle(packet)["accepted"]
    assert executor.track_capture_highwater[PEER] == 0.
    assert not executor.handle(tracked(.2, 3, captures={PEER: -.01}))["accepted"]
    arm(executor, .2, "next")
    assert all(stamp == -math.inf for stamp in executor.track_capture_highwater.values())
    assert executor.handle(tracked(.21, epoch="next"))["accepted"]


def test_silent_expiry_then_fresh_frame_cancels_old_strike_before_replanning():
    executor = Executor(required_participants=PARTICIPANTS)
    arm(executor)
    staged = {OWN: dict(x=-.45, y=0., yaw=0.), PEER: dict(x=1., y=0., yaw=math.pi)}
    executor.handle(tracked(robots=staged))
    executor.handle(action("attack"))
    for seq, now in enumerate((.1, .25), 2):
        result = executor.handle(tracked(now, seq, robots=staged, captures={PEER: 0.}))
    assert result["decisions"][OWN]["phase"] == "strike"
    result = executor.handle(tracked(.36, 4, robots=staged))
    assert result["decisions"][OWN]["phase"] == "align"
    assert executor.controllers[OWN]._strike is None


@pytest.mark.parametrize("stale_robot", PARTICIPANTS)
def test_post_external_hold_requires_all_participant_captures_after_release(stale_robot):
    executor = active()
    executor.set_external_hold(OWN, True, .1)
    executor.set_external_hold(OWN, False, .2)
    executor.handle(action(now=.21, seq=2))
    result = executor.handle(tracked(.22, 2, captures={stale_robot: .2}))
    assert result["accepted"] and is_zero(command(result))
    assert result["decisions"][OWN]["phase"] == "external_hold"
    result = executor.handle(tracked(.23, 3))
    assert not is_zero(command(result))


@pytest.mark.parametrize("stale_robot", PARTICIPANTS)
def test_camera_hold_cannot_release_using_a_pre_hold_track_under_new_envelope(stale_robot):
    executor = Executor(required_participants=PARTICIPANTS, obstacle_config=ObstacleConfig(clear_frames=1))
    arm(executor)
    executor.handle(camera(0., 1))
    executor.handle(tracked())
    assert not is_zero(command(executor.handle(action())))
    executor.handle(camera(.1, 2, blocked=True))
    executor.handle(camera(.2, 3))
    result = executor.handle(tracked(.21, 2, captures={stale_robot: .19}))
    assert result["accepted"] and is_zero(command(result))
    assert result["decisions"][OWN]["phase"] == "tracking_lost"
    assert "after obstacle hold" in result["decisions"][OWN]["reason"]
    result = executor.handle(tracked(.22, 3))
    assert not is_zero(command(result))


def test_goal_return_hold_retains_latch_and_resumes_return_with_fresh_peer():
    executor = active()
    packet = tracked(.1, 2)
    packet["robots"][OWN].update(x=0., goal=1)
    packet["robots"][PEER] = None
    result = executor.handle(packet)
    assert result["decisions"][OWN]["phase"] == "participant_hold"
    assert executor.controllers[OWN]._goal_latched
    packet = tracked(.2, 3)
    packet["robots"][OWN]["x"] = 0.
    result = executor.handle(packet)
    assert result["decisions"][OWN]["phase"] == "return_home"
    assert executor.controllers[OWN]._goal_latched


@pytest.mark.parametrize("stale_robot", PARTICIPANTS)
def test_resume_refuses_old_participant_under_fresh_envelope_without_consuming_latch(stale_robot):
    executor = Executor(required_participants=PARTICIPANTS)
    arm(executor)
    executor.handle(action("auto"))
    robots = {OWN: dict(x=-1.098, y=0., yaw=0., goal=1),
              PEER: dict(x=1., y=0., yaw=math.pi)}
    assert executor.handle(tracked(robots=robots))["decisions"][OWN]["phase"] == "await_reset"
    robots[OWN]["goal"] = 0
    assert executor.handle(tracked(.01, 2, robots=robots))["accepted"]
    executor.handle(tracked(.36, 3, robots=robots, captures={stale_robot: .01}))
    result = executor.handle(resume(.36, 3))
    assert not result["accepted"] and "participant" in result["reason"]
    assert executor.controllers[OWN]._goal_latched
    assert executor.resume_frame_seq[OWN] == -1


def test_deadline_query_is_pure_and_missing_own_pose_also_has_no_deadline():
    executor = active(required=())
    before = deepcopy(vars(executor))
    assert executor.frame_evidence_deadline(OWN) == pytest.approx(.35)
    assert executor.frame == before["frame"]
    assert executor.track_capture_highwater == before["track_capture_highwater"]
    packet = tracked(.1, 2)
    packet["robots"][OWN] = None
    executor.handle(packet)
    assert executor.frame_evidence_deadline(OWN) is None
    with pytest.raises(ValueError, match="unknown"):
        executor.frame_evidence_deadline("missing")
