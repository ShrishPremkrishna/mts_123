import json

import pytest

from final_project.behaviors import combined_fc_runner as runner
from final_project.behaviors.fc_executor_bridge import BridgeCore
from final_project.behaviors.test_command_path_evidence import (
    CLOCK, SOURCE, authority, graph, status)
from final_project.pupper_fc.model import Command, Field


def path_fixture():
    observed = []
    now = [10.]
    fields = {"pupper-1": Field(), "pupper-2": Field(attack_sign=-1)}
    core = BridgeCore("pupper-1", "camera", "test", fields, clock=lambda: now[0],
                      command_contract=runner.DEADLINE_CONTRACT, command_clock_id=CLOCK,
                      monotonic_ns=lambda: round(now[0]*1e9), source_instance=SOURCE, kick_side="left")
    node, pubs, subs, endpoint = graph()
    pubs[runner.protocol.REQUEST_TOPIC] = [endpoint(runner.NAME)]
    pubs[runner.evidence.NATIVE_AUTHORITY_TOPIC] = [endpoint("gemini_walk")]
    pubs["/fc/obstacle_detections"] = [endpoint("viser_camera")]
    subs[runner.protocol.ADMITTED_TOPIC] = [endpoint("gemini_walk")]
    path = runner.GuardedPath(core, node, observed.append)
    path.guard_status(json.dumps(status(frame_age_s=.34)))
    path.native_status(json.dumps(authority()))
    assert path.check() is None
    return path, now, observed, pubs, subs, endpoint


def test_publication_expires_with_original_camera_evidence():
    path, _, out, *_ = path_fixture()
    path.write(Command(.2, 0., 0., 10.14))
    cmd = runner.protocol.decode(out[-1])
    assert cmd.vx == .2 and cmd.expires_mono_ns <= 10_010_000_000


def test_missing_detector_holds_and_competing_detector_revokes():
    path, now, _, pubs, _, endpoint = path_fixture()
    pubs["/fc/obstacle_detections"].clear()
    assert path.check(True)[0] == "camera"
    pubs["/fc/obstacle_detections"] = [endpoint("viser_camera"), endpoint("second_detector")]
    now[0] += .11
    assert path.check(True)[0] == "authority"


def test_kick_may_release_walker_and_requires_new_native_binding_before_motion():
    path, now, _, pubs, subs, endpoint = path_fixture()
    path.core.executor.arm("play", path.core.clock_name, now[0])
    path.clip_started()
    pubs[runner.evidence.NATIVE_AUTHORITY_TOPIC] = []
    subs[runner.protocol.ADMITTED_TOPIC] = []
    assert path.check(True) is None
    assert path.check(False)
    pubs[runner.evidence.NATIVE_AUTHORITY_TOPIC] = [endpoint("gemini_walk")]
    subs[runner.protocol.ADMITTED_TOPIC] = [endpoint("gemini_walk")]
    path.check(True)  # change discovery phase
    assert path.check(False)[0] == "native"
    now[0] += .001
    path.native_status(json.dumps(authority(generation=3, activation_mono_ns=10_000_000_000,
                                           sampled_mono_ns=10_001_000_000)))
    assert path.check(False) is None
    now[0] += .001
    path.native_status(json.dumps(authority(generation=4, activation_mono_ns=10_001_000_000,
                                           sampled_mono_ns=10_002_000_000)))
    assert path.check(False)[0] == "authority"


def test_wrong_source_cannot_own_velocity_even_while_kick_hides_walker():
    path, _, out, pubs, _, endpoint = path_fixture()
    pubs[runner.protocol.REQUEST_TOPIC].append(endpoint("another_owner"))
    path.write(Command(expires_at=10.1))
    assert not out
    with pytest.raises(RuntimeError, match="unexpected publishers"):
        path.write(Command(vx=.2, expires_at=10.1))


def test_return_waits_for_discovery_but_refuses_a_foreign_source():
    path, now, _, pubs, _, endpoint = path_fixture()
    path.clip_started()
    assert not path.restored_walker_ready()
    pubs[runner.evidence.NATIVE_AUTHORITY_TOPIC] = []
    assert not path.restored_walker_ready()
    pubs[runner.evidence.NATIVE_AUTHORITY_TOPIC] = [endpoint("gemini_walk")]
    now[0] += .001
    path.native_status(json.dumps(authority(generation=3, bound_source_instance="b" * 32,
        activation_mono_ns=10_000_000_000, sampled_mono_ns=10_001_000_000)))
    with pytest.raises(RuntimeError, match="another source"):
        path.restored_walker_ready()


@pytest.mark.parametrize("clip_complete,goal,retain_ball", [
    (False, 1, False), (True, 0, False), (False, 0, True)])
def test_actual_shot_can_finish_before_planning_the_next_approach(clip_complete, goal, retain_ball):
    from final_project.pupper_fc.integration import Executor
    from final_project.pupper_fc.kick_handoff import KickHandoff
    from final_project.pupper_fc.tests.test_integration import arm, frame, intent
    robot_id = "pupper-1"
    e, h = Executor(kick_side="left"), KickHandoff(robot_id)
    arm(e)
    e.handle(intent())
    robot = dict(x=-.258, y=-.040, yaw=0.)
    job = None
    for seq in range(7):
        t = seq * .04
        e.handle(frame(t, seq+1, robots={robot_id:robot}))
        job = e.kick_step(h, t).request or job
    assert h.receipt(job, "clip_started", clip="kick_left")
    if clip_complete:
        assert h.receipt(job, "clip_confirmed", clip="kick_left")
    displaced = {"x":-1.70, "y":0} if retain_ball else None
    e.handle(frame(.27, 8, robots={robot_id:{**robot, "goal":goal}}, ball=displaced))
    step = e.kick_step(h, .27)
    assert step.decision.phase == "kick_in_flight" and step.cancel is None
    # Source and pose must still be fresh while recovering, even with clip proof.
    e.handle(frame(.28, 9, robots={}, ball=displaced))
    assert e.kick_step(h, .28).cancel == job
