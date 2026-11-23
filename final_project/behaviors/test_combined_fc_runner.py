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


@pytest.mark.parametrize("newer_same_gateway", [True, False])
def test_hard_stop_result_is_bounded_and_tied_to_owned_gateway(newer_same_gateway):
    import asyncio
    from types import SimpleNamespace
    from foundation.hardware.software_start import ResidentStart, STOP_TOPIC

    clock = [3.]
    sent = []
    caller = ResidentStart(None, None, clock=lambda: clock[0], gateway_stop=lambda: sent.append("gateway"),
                           emergency_stop=lambda: sent.append("native"))
    status = dict(protocol="pupper_stop_v1", instance_id="a" * 32, sequence=10, epoch=2,
                  latched=False, fault=False, stop_required=False, in_flight=True,
                  unknown_remote_submission=False, shutting_down=False, selected_controller="gemini_walk",
                  software_stop_topic=STOP_TOPIC)
    assert caller.observe(status)
    anchor = {key: status[key] for key in ("instance_id", "epoch", "sequence")}
    caller.stop_publisher = SimpleNamespace(get_subscription_count=lambda: 1)
    assert runner.gateway_stop_ready(caller, anchor)
    # A restart cannot replace the authority the robot client opened against.
    assert caller.observe({**status, "instance_id":"b" * 32})
    assert not runner.gateway_stop_ready(caller, anchor)
    caller.request_stop("gateway has a pending activation", authority=anchor)
    caller.request_stop("repeated local fault", authority=anchor)
    assert sent == ["native", "gateway"]
    sequence = [10]

    def receive(timeout_sec):
        clock[0] += .1
        sequence[0] += 1
        caller.observe({**status, "instance_id": anchor["instance_id"] if newer_same_gateway else "b" * 32,
                        "epoch":3, "latched":True, "sequence":sequence[0]})

    result = asyncio.run(runner.await_gateway_stop(caller, receive, lambda: True))
    assert result["confirmed"] == newer_same_gateway and result["uncertain"] != newer_same_gateway
    assert sent == ["native", "gateway"]


def test_motion_expiry_at_encoder_publishes_neutral_without_poisoning_session():
    path, now, out, *_ = path_fixture()
    original_ns = path.core.deadline.monotonic_ns

    def publication_time():
        now[0] = 10.006  # command was live at the path check, expires before encoding
        return original_ns()

    path.core.deadline.monotonic_ns = publication_time
    reason = path.write(Command(.2, 0., 0., 10.005))
    neutral = runner.protocol.decode(out[-1])
    assert reason == "motion command expired before publication"
    assert neutral.is_zero and neutral.issued_mono_ns == 10_006_000_000
    assert not path.core.executor.external_holds[path.core.robot_id]
    path.core.deadline.monotonic_ns = original_ns
    for _ in range(3):
        now[0] += .0005
        assert path.write(Command(.2, 0., 0., 10.005)) == reason
        assert runner.protocol.decode(out[-1]).is_zero
        assert not path.core.executor.external_holds[path.core.robot_id]
    now[0] += .001
    assert path.write(Command(.2, 0., 0., 10.009)) is None
    fresh = runner.protocol.decode(out[-1])
    assert fresh.vx == .2 and fresh.expires_mono_ns == 10_009_000_000


def test_expired_motion_does_not_hide_a_failed_neutral_publication():
    path, now, _, *_ = path_fixture()

    def publication_time():
        now[0] = 10.006
        return 10_006_000_000

    def broken_transport(payload):
        assert runner.protocol.decode(payload).is_zero
        raise OSError("neutral transport failed")

    path.core.deadline.monotonic_ns = publication_time
    path.publish = broken_transport
    with pytest.raises(OSError, match="neutral transport failed"):
        path.write(Command(.2, 0., 0., 10.005))
