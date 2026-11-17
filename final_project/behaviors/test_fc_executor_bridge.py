"""Offline transport/admission tests; these do not establish robot behavior."""
from dataclasses import replace
import hashlib
import importlib
import io
import json
import math
import pickle
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest

import fc_executor_bridge as bridge
from final_project.pupper_fc.model import Command, Decision

CONFIG = {"field": {"length": 3.66, "width": 2.44, "goal_width": .6096,
                     "robot_radius": .16, "ball_radius": .11, "bumper_offset": .19},
          "attack_signs": {"pupper-1": 1, "pupper-2": -1}}
ZERO = (0., 0., 0.)


def core_fixture(robot="pupper-1"):
    now = [10.]
    core = bridge.BridgeCore(robot, "fixture", "boot", bridge.load_fields(CONFIG), clock=lambda: now[0])
    assert core.operator({"command": "arm", "epoch": "game-1"})["accepted"]
    return core, now


def frame(core, now, seq=1, **changes):
    row = {"event": "frame", "producer_id": core.producer_id, "clock": core.clock_name,
           "epoch": core.executor.epoch, "seq": seq, "captured_at": now,
           "robots": {"pupper-1": {"x": -1., "y": 0., "yaw": 0.},
                      "pupper-2": {"x": 1., "y": 0., "yaw": math.pi}},
           "ball": {"x": 0., "y": 0.},
           "goals": {"left": {"x": -1.83, "y": 0}, "right": {"x": 1.83, "y": 0}}}
    row.update(changes)
    return row


def intent(core, now, seq=1, **changes):
    row = {"event": "intent", "producer_id": core.producer_id, "clock": core.clock_name,
           "epoch": core.executor.epoch, "seq": seq, "robot_id": core.robot_id,
           "intent_id": "move-1", "kind": "move", "target": {"x": 0., "y": 0., "yaw": 0.},
           "issued_at": now, "valid_until": now + .3}
    row.update(changes)
    return row


def moving(core, now):
    assert core.ingest(frame(core, now))["accepted"]
    assert core.ingest(intent(core, now))["accepted"]
    result = core.tick()
    assert abs(result.command.vx) > 0
    return result


def xyz(command):
    return command.vx, command.vy, command.wz


@pytest.mark.parametrize("robot", bridge.ROBOT_IDS)
def test_pinned_robot_only_and_lease_expires_without_any_new_ingress(robot):
    core, now = core_fixture(robot)
    moving(core, now[0])
    for t in (10.02, 10.1, 10.299):
        now[0] = t
        output = core.output(core.tick())
        assert output.robot_id == robot and any(xyz(output))
        assert abs(output.vx) <= .3 and abs(output.vy) <= .3 and abs(output.wz) <= 1
    now[0] = 10.3
    assert xyz(core.output(core.tick())) == ZERO
    assert core.executor.intents[robot] is None


def test_capture_expiry_is_independent_of_longer_lease_and_incoming_envelope_now():
    core, now = core_fixture()
    assert core.ingest({**frame(core, 9.8), "now": 10**9})["accepted"]
    assert core.ingest({**intent(core, 10., valid_until=10.5), "now": -999})["accepted"]
    assert core.executor.frame[0] == 9.8
    now[0] = 10.151
    assert xyz(core.output(core.tick())) == ZERO


@pytest.mark.parametrize("changes", [{"captured_at": 10.001}, {"captured_at": 9.6},
    {"captured_at": float("nan")}, {"robots": {"pupper-1": {"x": 500, "y": 0, "yaw": 0}}}])
def test_bad_capture_or_geometry_invalidates_current_frame(changes):
    core, now = core_fixture()
    moving(core, now[0])
    result = core.ingest(frame(core, now[0], seq=2, **changes))
    assert not result["accepted"]
    assert xyz(core.output(core.tick())) == ZERO


@pytest.mark.parametrize("changes", [{"producer_id": "stranger"}, {"clock": "linux-monotonic:other-boot"},
    {"event": "arm"}, {"event": "stop"}, {"event": "resume"}, {"robot_id": "pupper-2"}])
def test_perception_cannot_change_operator_authority_or_other_robot(changes):
    core, now = core_fixture()
    with pytest.raises(ValueError):
        core.ingest(intent(core, now[0], **changes))
    assert core.executor.armed and core.executor.intents[core.robot_id] is None


def test_reordered_and_old_epoch_packets_cannot_refresh_current_evidence():
    core, now = core_fixture()
    moving(core, now[0])
    now[0] += .1
    assert not core.ingest(frame(core, now[0], seq=1))["accepted"]
    assert not core.ingest(intent(core, now[0], epoch="old"))["accepted"]
    assert core.executor.frame[0] == 10.
    assert core.executor.intents[core.robot_id].valid_until == 10.3


def test_output_rechecks_expiry_after_computation_and_fences_recovery():
    core, now = core_fixture()
    decision = moving(core, now[0])
    now[0] = decision.command.expires_at
    assert xyz(core.output(decision)) == ZERO
    now[0] = 10.1
    assert xyz(core.output(replace(core.tick(), recovery_requested=True))) == ZERO


@pytest.mark.parametrize("changes", [{"vx": .30001}, {"vy": -.30001}, {"wz": 1.001},
    {"vx": float("nan")}, {"expires_at": float("inf")}, {"robot_id": "pupper-2"}])
def test_final_publication_rejects_invalid_commands_and_latches_stop(changes):
    core, now = core_fixture()
    decision = moving(core, now[0])
    with pytest.raises(ValueError):
        core.output(replace(decision, command=replace(decision.command, **changes)))
    assert core.stopped and not core.executor.armed


def test_external_hold_cancels_controller_and_requires_post_clear_field_and_issue():
    core, now = core_fixture()
    moving(core, now[0])
    now[0] += .02
    core.hold(True)
    assert xyz(core.output(core.tick())) == ZERO
    now[0] += .02
    assert core.ingest(frame(core, now[0], seq=2))["accepted"]
    assert core.ingest(intent(core, now[0], seq=2))["accepted"]
    now[0] += .02
    core.hold(False)
    assert xyz(core.output(core.tick())) == ZERO
    now[0] += .01
    core.ingest(frame(core, now[0], seq=3))
    assert xyz(core.output(core.tick())) == ZERO
    core.ingest(intent(core, now[0], seq=3))
    assert any(xyz(core.output(core.tick())))


def test_stop_cannot_be_rearmed_and_publication_clock_regression_stops():
    core, now = core_fixture()
    decision = moving(core, now[0])
    now[0] -= .01
    with pytest.raises(ValueError, match="backwards"):
        core.output(decision)
    with pytest.raises(ValueError, match="latched"):
        core.operator({"command": "arm", "epoch": "new"})


def test_fallen_frame_requests_recovery_and_never_getup_motion():
    core, now = core_fixture()
    row = frame(core, now[0])
    row["robots"][core.robot_id]["fallen"] = True
    core.ingest(row)
    core.ingest(intent(core, now[0]))
    decision = core.tick()
    assert decision.recovery_requested and xyz(core.output(decision)) == ZERO


def test_explicit_geometry_is_required_and_dry_run_never_imports_ros(capsys):
    assert bridge.main(["--robot-id", "pupper-1", "--producer-id", "fixture"]) == 0
    assert json.loads(capsys.readouterr().out)["ros_started"] is False
    assert bridge.main(["--execute", "--robot-id", "pupper-1", "--producer-id", "fixture"]) == 2
    for missing in bridge.FIELD_KEYS:
        config = json.loads(json.dumps(CONFIG))
        del config["field"][missing]
        with pytest.raises(ValueError):
            bridge.load_fields(config)


def test_blocked_notification_writer_is_bounded_and_does_not_delay_expiry():
    blocked, release = threading.Event(), threading.Event()
    def writer(row):
        blocked.set()
        release.wait(2)
    sink = bridge.EventSink(writer=writer, capacity=2)
    sink({"first": True})
    assert blocked.wait(1)
    core, now = core_fixture()
    moving(core, now[0])
    began = time.monotonic()
    for i in range(1000):
        sink({"status": i})
    now[0] += .31
    assert xyz(core.output(core.tick())) == ZERO
    assert time.monotonic() - began < .2
    assert sink.queue.qsize() <= 2 and sink.dropped > 0
    release.set()


@pytest.fixture
def ros_node(tmp_path, monkeypatch):
    import _fake_ros
    roots = {"rclpy", "rcl_interfaces", "geometry_msgs", "std_msgs", "std_srvs", "sensor_msgs",
             "controller_manager_msgs", "run_through_kick_node", "joy_utils"}
    saved = {n: m for n, m in sys.modules.items() if n.split(".")[0] in roots}
    ros = _fake_ros.install()
    sys.modules["std_msgs.msg"].Float32MultiArray = SimpleNamespace
    clock_module = ModuleType("rclpy.clock")
    clock_module.Clock = lambda **kwargs: SimpleNamespace(**kwargs)
    clock_module.ClockType = SimpleNamespace(STEADY_TIME="steady")
    sys.modules["rclpy.clock"] = clock_module
    sys.modules["rclpy.qos"] = ModuleType("rclpy.qos")
    sys.modules["rclpy.qos"].QoSProfile = lambda **kwargs: SimpleNamespace(**kwargs)
    sys.modules["rclpy.qos"].ReliabilityPolicy = SimpleNamespace(BEST_EFFORT="best_effort")
    timer_clocks = []
    def timer(self, period, callback, *, clock):
        timer_clocks.append(clock.clock_type)
        ros.timers.append((period, callback))
    monkeypatch.setattr(_fake_ros.Node, "create_timer", timer)
    stock_module = importlib.reload(importlib.import_module("run_through_kick_node"))
    policy = tmp_path / "fixture.json"
    policy.write_bytes(b"offline policy fixture only")
    stock_module.STOCK_POLICY_SHA256 = hashlib.sha256(policy.read_bytes()).hexdigest()
    ros.controller_params["model_path"] = str(policy)
    core = bridge.BridgeCore("pupper-1", "fixture", "boot", bridge.load_fields(CONFIG), clock=lambda: ros.t)
    identities = {"native": True, "path": True}
    events = []
    node = bridge._node_class()(core, same_manager=lambda: identities["native"],
        same_command_path=lambda: identities["path"], emit=events.append)
    names = {"/cmd_vel": ["fc_body_command_guard", bridge.NODE_NAME],
             "/kick_cmd_vel": [bridge.NODE_NAME], "/fc/requested_cmd_vel": ["cmd_vel_mux"],
             bridge.GUARD_STATUS_TOPIC: ["fc_body_command_guard"]}
    node.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(node_name=name, node_namespace="/")
                                                      for name in names.get(topic, [])]
    ros.publishers.update({topic: len(rows) for topic, rows in names.items()})
    state = {"protocol": "pupper_stop_v1", "instance_id": "gateway", "epoch": 1,
             "latched": False, "fault": False, "stop_required": False,
             "unknown_remote_submission": False, "shutting_down": False}
    guard = {"state": "ready", "camera_state": "clear", "camera_permitted": True,
             "input_state": "missing", "requires_post_hold_zero": False, "clear_frames": 3,
             "reason": "clear", "frame_age_s": .05, "max_frame_age_s": .35,
             "producer_instance": "camera", "sequence": 1}
    def update(*, applied=ZERO, camera=True):
        node.on_gateway(_fake_ros.String(json.dumps(state)))
        data = [0.] * 960
        data[6:9] = applied
        node.on_observation(SimpleNamespace(data=data))
        if camera:
            guard["sequence"] += 1
            guard["sampled_at_monotonic_ns"] = round(ros.t * 1e9)
            node.on_guard(_fake_ros.String(json.dumps(guard)))
        node.check_identity()
        for future in list(ros.pending):
            future.resolve()
        ros.pending.clear()
    update()
    node.stock.poll()
    update()
    node.stock.poll()
    assert node.native_problem() is None and node.guard_problem() is None
    assert timer_clocks == ["steady"] * 3
    yield SimpleNamespace(ros=ros, node=node, core=core, events=events, names=names,
                          identities=identities, guard=guard, state=state, update=update)
    for name in list(sys.modules):
        if name.split(".")[0] in roots:
            del sys.modules[name]
    sys.modules.update(saved)


def arm_and_move(env):
    env.node.inbox.put({"command": "arm", "epoch": "game-1"})
    env.node.on_tick()
    assert env.core.executor.armed
    env.core.ingest(frame(env.core, env.ros.t))
    env.core.ingest(intent(env.core, env.ros.t))
    env.node.on_tick()
    assert any(last_twist(env))


def last_twist(env, topic="/kick_cmd_vel"):
    msg = env.ros.published[topic][-1]
    return msg.linear.x, msg.linear.y, msg.angular.z


def step(env, dt=.02, **kwargs):
    env.ros.t += dt
    env.update(**kwargs)
    env.node.on_tick()


def test_ros_wiring_idle_does_not_arm_and_independent_tick_expires_lease(ros_node):
    e = ros_node
    assert not e.ros.published["/kick_cmd_vel"]
    arm_and_move(e)
    for _ in range(16):
        step(e)
    assert last_twist(e) == ZERO
    assert not e.ros.published["/emergency_stop"]


@pytest.mark.parametrize("cause", ["wrong_publisher", "missing_status", "stale_camera", "missing_native", "bad_observation"])
def test_arm_refuses_missing_current_evidence(ros_node, cause):
    e = ros_node
    if cause == "wrong_publisher":
        e.names["/cmd_vel"][0] = "rogue"
    elif cause == "missing_status":
        e.node.guard_at = None
    elif cause == "stale_camera":
        e.node.guard_deadline = e.ros.t
    elif cause == "missing_native":
        e.node.native_valid = False
    else:
        e.node.observation = None
    e.node.inbox.put({"command": "arm", "epoch": "game-1"})
    e.node.on_tick()
    assert not e.core.executor.armed and not e.ros.published["/kick_cmd_vel"]


def test_guard_hold_zero_confirmation_and_fresh_post_clear_inputs(ros_node):
    e = ros_node
    arm_and_move(e)
    e.guard.update(camera_state="blocked", camera_permitted=False)
    step(e, applied=(.2, 0., 0.))
    assert last_twist(e) == ZERO and e.core.executor.external_holds[e.core.robot_id]
    e.guard.update(camera_state="clear", camera_permitted=True)
    step(e, applied=(.2, 0., 0.))
    assert e.core.executor.external_holds[e.core.robot_id]  # still awaiting consumed zero
    step(e)
    assert not e.core.executor.external_holds[e.core.robot_id]
    assert last_twist(e) == ZERO
    e.ros.t += .01
    e.core.ingest(frame(e.core, e.ros.t, seq=2))
    e.core.ingest(intent(e.core, e.ros.t, seq=2))
    e.node.on_tick()
    assert any(last_twist(e)) and not e.ros.published["/emergency_stop"]


def test_unconfirmed_camera_zero_escalates_to_emergency_stop(ros_node):
    e = ros_node
    arm_and_move(e)
    e.guard.update(camera_state="blocked", camera_permitted=False)
    for _ in range(28):
        step(e, applied=(.2, 0., 0.))
    assert e.node.hard_stopped and len(e.ros.published["/emergency_stop"]) == 1


@pytest.mark.parametrize("cause", ["guard", "native", "publisher", "stale_observation"])
def test_guard_only_loss_preserves_standing_but_native_faults_stop(ros_node, cause):
    e = ros_node
    arm_and_move(e)
    if cause == "guard":
        e.identities["path"] = False
    elif cause == "native":
        e.identities["native"] = False
    elif cause == "publisher":
        e.names["/cmd_vel"][0] = "rogue"
    else:
        e.node.observation = None
    e.node.check_identity()
    e.node.on_tick()
    assert e.node.closing and last_twist(e) == ZERO
    assert bool(e.ros.published["/emergency_stop"]) is (cause != "guard")
    step(e)
    assert e.node.closed


@pytest.mark.parametrize("late", [False, True])
def test_stop_outranks_guard_loss_through_final_cleanup(ros_node, late):
    e = ros_node
    arm_and_move(e)
    e.identities["path"] = False
    e.node.check_identity()
    if late:
        e.node.on_tick()
        assert e.node.guard_only_exit
    e.node.inbox.put({"command": "stop", "reason": "operator Stop"})
    e.node.inbox.put({"command": "arm", "epoch": "unwanted"})
    step(e)
    assert e.node.hard_stopped and not e.node.guard_only_exit
    assert len(e.ros.published["/emergency_stop"]) == 1
    assert not e.core.executor.armed and e.node.inbox.normal.qsize() == 1


def test_guard_loss_during_pending_start_always_requests_emergency(ros_node):
    e = ros_node
    e.node.start_client = SimpleNamespace(pending=object(), uncertain=False, poll=lambda: None)
    e.node.submitted_start = True
    e.identities["path"] = False
    e.node.check_identity()
    e.node.on_tick()
    assert e.node.hard_stopped and len(e.ros.published["/emergency_stop"]) == 1


def test_gateway_new_epoch_ends_initial_latched_session_without_rearm(ros_node):
    e = ros_node
    e.node.gateway_released = False
    e.state.update(latched=True, epoch=2)
    e.update()
    assert e.node.closing and e.node.hard_stopped


def test_operator_stop_receipt_fences_nonzero_before_tick(ros_node):
    e = ros_node
    arm_and_move(e)
    before = len(e.ros.published["/kick_cmd_vel"])
    e.node.inbox.put({"command": "stop"})
    e.node.on_tick()
    new = e.ros.published["/kick_cmd_vel"][before:]
    assert new and all((m.linear.x, m.linear.y, m.angular.z) == ZERO for m in new)


def test_source_roles_are_required_and_hash_checked():
    with pytest.raises(ValueError, match="bridge_script"):
        bridge.verify_bridge_sources({"files": {}})


@pytest.mark.parametrize("stamp", [None, -1, True, 10.0, float("nan"), 10_300_000_001, 9_999_999_999])
def test_guard_sampling_timestamp_refuses_absent_invalid_future_or_stale(ros_node, stamp):
    status = dict(ros_node.guard, sampled_at_monotonic_ns=stamp)
    with pytest.raises(ValueError, match="timestamp"):
        bridge.decode_guard_status(json.dumps(status), 10.3)


def test_delayed_first_status_cannot_rejuvenate_camera_capture(ros_node):
    status = dict(ros_node.guard, sampled_at_monotonic_ns=10_000_000_000, frame_age_s=.1)
    parsed, _, deadline = bridge.decode_guard_status(json.dumps(status), 10.3)
    assert deadline == pytest.approx(10.25)
    assert bridge.guard_status_problem(parsed, 10.3, None, deadline, 10.3)[0] == "camera"
    assert bridge.guard_status_problem(parsed, 10.3, None, deadline, 10.31)[0] == "path"


def test_competing_publisher_outranks_missing_guard_graph_and_process(ros_node):
    e = ros_node
    arm_and_move(e)
    e.names["/cmd_vel"] = [bridge.NODE_NAME]
    e.ros.publishers["/cmd_vel"] = 1
    e.names["/kick_cmd_vel"].append("competing")
    e.ros.publishers["/kick_cmd_vel"] = 2
    e.identities["path"] = False
    e.node.check_identity()
    assert e.node.guard_problem()[0] == "authority"
    e.node.on_tick()
    assert e.node.hard_stopped and len(e.ros.published["/emergency_stop"]) == 1


def attach_start(env):
    from foundation.hardware.software_start import ResidentStart
    calls = []
    future = SimpleNamespace(done=lambda: False)
    def submit(request):
        calls.append(request)
        return future
    caller = ResidentStart(submit, lambda: True, clock=lambda: env.ros.t)
    status = {**env.state, "sequence": 1, "selected_controller": "neural_controller_default", "in_flight": False}
    caller.observe(status)
    env.node.start_client = caller
    return caller, calls, future, status


def test_explicit_start_uses_receipt_ticket_once_and_never_arms_fc(ros_node):
    e = ros_node
    caller, calls, future, _ = attach_start(e)
    e.node.inbox.put({"command": "start", "_ticket": "forged"})
    e.node.on_tick()
    assert len(calls) == 1 and not e.core.executor.armed
    future.done = lambda: True
    future.result = lambda: SimpleNamespace(success=True, message="activated")
    e.node.on_tick()
    assert not e.core.executor.armed and not e.ros.published["/kick_cmd_vel"]
    e.node.inbox.put({"command": "start"})
    e.node.on_tick()
    assert len(calls) == 1  # the consumed gateway challenge cannot be submitted twice


def test_start_capture_precedes_node_lock_and_delay_cannot_refresh_press(ros_node):
    e = ros_node
    caller, calls, _, status = attach_start(e)
    captured = threading.Event()
    original_capture = caller.capture
    def capture(*, pressed_at):
        ticket = original_capture(pressed_at=pressed_at)
        captured.set()
        return ticket
    caller.capture = capture
    e.node.lock.acquire()
    worker = threading.Thread(target=e.node.inbox.put, args=({"command": "start"},))
    worker.start()
    assert captured.wait(1)
    e.ros.t += .31
    caller.observe({**status, "sequence": 2})
    e.node.lock.release()
    worker.join(1)
    assert not worker.is_alive()
    e.update()
    e.node.on_tick()
    assert not calls and not e.node.submitted_start


@pytest.mark.parametrize("shutdown", ["stop", "quit", "native", "guard", "uncertain"])
def test_pending_start_cleanup_strong_stop_has_precedence(ros_node, shutdown):
    e = ros_node
    caller, calls, _, _ = attach_start(e)
    e.node.inbox.put({"command": "start"})
    e.node.on_tick()
    assert len(calls) == 1
    if shutdown in ("stop", "quit"):
        e.node.inbox.put({"command": shutdown})
    elif shutdown in ("native", "guard"):
        e.identities["native" if shutdown == "native" else "path"] = False
        e.node.check_identity()
    else:
        caller.pending = None
        caller.uncertain = True
    e.node.on_tick()
    assert e.node.hard_stopped and len(e.ros.published["/emergency_stop"]) == 1
    assert not e.core.executor.armed


def test_start_ack_does_not_replace_stock_readiness(ros_node):
    e = ros_node
    caller, calls, future, _ = attach_start(e)
    e.node.inbox.put({"command": "start"})
    e.node.on_tick()
    future.done = lambda: True
    future.result = lambda: SimpleNamespace(success=True, message="ack")
    e.node.on_tick()
    e.node.stock.bounds, e.node.stock.error = None, "controller still unavailable"
    e.node.inbox.put({"command": "arm", "epoch": "game-1"})
    e.node.on_tick()
    assert len(calls) == 1 and not e.core.executor.armed


def test_guard_only_loss_after_successful_start_keeps_native_activation(ros_node):
    e = ros_node
    caller, calls, future, _ = attach_start(e)
    e.node.inbox.put({"command": "start"})
    e.node.on_tick()
    future.done = lambda: True
    future.result = lambda: SimpleNamespace(success=True, message="ack")
    e.node.on_tick()
    e.identities["path"] = False
    e.node.check_identity()
    e.node.on_tick()
    step(e)
    assert e.node.closed and e.node.guard_only_exit and not e.ros.published["/emergency_stop"]


def test_maximum_detector_label_reason_still_cancels_to_ordinary_camera_hold(ros_node):
    e = ros_node
    arm_and_move(e)
    reason = "near detection overlaps forward corridor (" + "x" * 128 + ")"
    e.guard.update(camera_state="blocked", camera_permitted=False, reason=reason)
    step(e, applied=(.3, 0., 0.))
    assert last_twist(e) == ZERO
    assert e.core.executor.external_holds[e.core.robot_id]
    assert len(e.core.executor.external_hold_reasons[e.core.robot_id]) == 128
    assert e.core.executor.controllers[e.core.robot_id]._strike is None
    assert not e.node.closing and not e.ros.published["/emergency_stop"]


@pytest.mark.parametrize("late_action", ["stop", "expiry"])
def test_final_stock_check_cannot_publish_after_received_stop_or_expiry(ros_node, late_action):
    e = ros_node
    arm_and_move(e)
    original = e.node.stock.problem
    before = len(e.ros.published["/kick_cmd_vel"])
    def final_check(*args, **kwargs):
        result = original(*args, **kwargs)
        command = kwargs.get("command")
        if command is not None:
            if late_action == "stop":
                e.node.inbox.put({"command": "stop", "reason": "Stop during final stock check"})
            else:
                e.ros.t = command.expires_at
        return result
    e.node.stock.problem = final_check
    e.node.on_tick()
    new = e.ros.published["/kick_cmd_vel"][before:]
    assert new and all((m.linear.x, m.linear.y, m.angular.z) == ZERO for m in new)
    if late_action == "stop":
        assert e.node.hard_stopped and e.node.closing
        assert len(e.ros.published["/emergency_stop"]) == 1


def test_publication_boundary_itself_fences_a_received_stop(ros_node):
    e = ros_node
    arm_and_move(e)
    command = e.core.output(e.core.tick())
    before = len(e.ros.published["/kick_cmd_vel"])
    e.node.inbox.put({"command": "stop"})
    e.node.publish_command(command)
    new = e.ros.published["/kick_cmd_vel"][before:]
    assert new and all((m.linear.x, m.linear.y, m.angular.z) == ZERO for m in new)
    assert e.node.hard_stopped


def test_excess_publisher_count_outranks_earlier_expected_name_snapshot(ros_node):
    e = ros_node
    arm_and_move(e)
    # A publisher may appear between the identity and count graph queries.
    assert e.names["/cmd_vel"] == ["fc_body_command_guard", bridge.NODE_NAME]
    e.ros.publishers["/cmd_vel"] = 3
    calls = []
    original = e.node.count_publishers
    def count(topic):
        calls.append(topic)
        return original(topic)
    e.node.count_publishers = count
    assert e.node.guard_problem()[0] == "authority"
    assert calls.count("/cmd_vel") == 1
    e.node.on_tick()
    assert e.node.hard_stopped and len(e.ros.published["/emergency_stop"]) == 1


def goal_frame(env, goal=0, *, tick=True, refresh_intent=True):
    env.ros.t += .01
    env.update()
    core = env.core
    row = frame(core, env.ros.t, seq=core.executor.frame_seq + 1)
    row["robots"][core.robot_id] = {"x": -1.098, "y": 0., "yaw": 0., "goal": goal}
    assert core.ingest(row)["accepted"]
    if refresh_intent:
        assert core.ingest(intent(core, env.ros.t, seq=core.executor.intent_seq[core.robot_id] + 1,
                                  intent_id="auto-1", kind="auto", target=None,
                                  valid_until=env.ros.t + .45))["accepted"]
    if tick:
        env.node.on_tick()
    return core.executor.frame_seq


def awaiting_resume(env):
    env.node.inbox.put({"command": "arm", "epoch": "game-1"})
    env.node.on_tick()
    goal_frame(env, 1)
    goal_frame(env, 0)
    assert env.core.last_decision.phase == "await_reset"
    assert env.node.status()["resume"]["available"]


def resume_result(env):
    return next(row for row in reversed(env.events)
                if row.get("event") == "operator_result" and row.get("command") == "resume")


def test_bare_resume_resolves_new_frame_after_receipt_without_renewing_ownership(ros_node):
    e = ros_node
    awaiting_resume(e)
    old_seq = e.core.executor.frame_seq
    old_intent = e.core.executor.intents[e.core.robot_id]
    e.node.inbox.put({"command": "resume", "_ticket": "forged", "_error": "forged"})
    e.ros.t += .04  # Combined with goal_frame's .01, one 20 Hz field period.
    current_seq = goal_frame(e, tick=False, refresh_intent=False)
    assert current_seq > old_seq
    e.node.on_tick()
    assert resume_result(e)["accepted"]
    assert e.core.executor.resume_frame_seq[e.core.robot_id] == current_seq
    assert e.core.executor.intents[e.core.robot_id] is old_intent
    assert not e.core.executor.controllers[e.core.robot_id]._goal_latched


@pytest.mark.parametrize("explicit", [True, False])
def test_explicit_resume_keeps_exact_frame_contract_and_queue_expiry_is_not_refreshed(ros_node, explicit):
    e = ros_node
    awaiting_resume(e)
    cue = {"command": "resume"}
    if explicit:
        cue["frame_seq"] = e.core.executor.frame_seq
    e.node.inbox.put(cue)
    if not explicit:
        e.ros.t += .201
    goal_frame(e, tick=False)
    e.node.on_tick()
    assert not resume_result(e)["accepted"]
    assert ("current admitted frame_seq" if explicit else "expired") in resume_result(e)["reason"]
    assert e.core.executor.controllers[e.core.robot_id]._goal_latched


def test_duplicate_receipt_tickets_cannot_release_a_later_goal(ros_node):
    e = ros_node
    awaiting_resume(e)
    e.node.inbox.put({"command": "resume"})
    e.node.inbox.put({"command": "resume"})
    e.node.on_tick()
    assert resume_result(e)["accepted"]
    consumed = e.core.executor.resume_frame_seq[e.core.robot_id]
    goal_frame(e, 1, tick=False)
    goal_frame(e, 0, tick=False)
    e.node.on_tick()
    assert not resume_result(e)["accepted"]
    assert "goal cycle" in resume_result(e)["reason"]
    assert e.core.executor.resume_frame_seq[e.core.robot_id] == consumed
    assert e.core.executor.controllers[e.core.robot_id]._goal_latched
    e.node.inbox.put({"command": "resume"})
    e.node.on_tick()
    assert resume_result(e)["accepted"]


@pytest.mark.parametrize("fault", ["hold", "stale_frame", "expired_intent", "missing_frame", "newarm", "stop"])
def test_resume_ticket_rechecks_evidence_and_session_at_processing(ros_node, fault):
    e = ros_node
    awaiting_resume(e)
    e.node.inbox.put({"command": "resume"})
    if fault == "hold":
        e.guard.update(camera_state="blocked", camera_permitted=False)
        e.update()
    elif fault == "stale_frame":
        stamp, observations, goals = e.core.executor.frame
        e.core.executor.frame = (e.ros.t - .36, observations, goals)
    elif fault == "expired_intent":
        old = e.core.executor.intents[e.core.robot_id]
        e.core.executor.intents[e.core.robot_id] = replace(old, valid_until=e.ros.t)
    elif fault == "missing_frame":
        e.core.executor.frame = None
    elif fault == "newarm":
        assert e.core.operator({"command": "arm", "epoch": "game-2"})["accepted"]
        goal_frame(e, 1, tick=False)
        goal_frame(e, 0, tick=False)
    else:
        e.node.inbox.put({"command": "stop"})
    e.node.on_tick()
    assert not any(row.get("command") == "resume" and row.get("accepted") for row in e.events)
    if fault == "stop":
        assert e.node.hard_stopped and not e.core.executor.armed
        assert len(e.ros.published["/emergency_stop"]) == 1
    else:
        assert e.core.executor.controllers[e.core.robot_id]._goal_latched


@pytest.mark.parametrize("fault", ["not_armed", "goal_flag", "hold", "old_frame", "pending_start", "uncertain_start", "native"])
def test_unavailable_current_evidence_refuses_resume_at_processing(ros_node, fault):
    e = ros_node
    if fault != "not_armed":
        awaiting_resume(e)
    if fault == "goal_flag":
        goal_frame(e, 1)
    elif fault == "hold":
        e.core.hold(True)
    elif fault == "old_frame":
        stamp, observations, goals = e.core.executor.frame
        e.core.executor.frame = (e.ros.t - .36, observations, goals)
    elif fault in ("pending_start", "uncertain_start"):
        e.node.start_client = SimpleNamespace(pending=object() if fault == "pending_start" else None,
                                             uncertain=fault == "uncertain_start")
    elif fault == "native":
        e.node.native_valid = False
    status = e.node.status()
    assert not status["resume"]["available"] and status["resume"]["reason"]
    e.node.inbox.put({"command": "resume"})
    with e.node.lock:
        e.node.operator(*e.node.inbox.get())
    assert not resume_result(e)["accepted"]


def test_status_readiness_is_descriptive_and_does_not_mutate_executor_or_internal_gate(ros_node):
    from final_project.pupper_fc.obstacles import DetectionFrame, ForwardObstacleGate, ObstacleConfig
    e = ros_node
    awaiting_resume(e)
    gate = ForwardObstacleGate(ObstacleConfig(clear_frames=1))
    gate.update(DetectionFrame(1, e.ros.t - .1), e.ros.t - .1)
    e.core.executor.obstacle_gates[e.core.robot_id] = gate
    before = pickle.dumps(e.core.executor)
    for _ in range(5):
        status = e.node.status()
        e.core.executor.check_resume(e.core.robot_id, e.core.executor.frame_seq, e.ros.t)
        assert status["resume"]["available"]
        assert status["frame_seq"] == e.core.executor.frame_seq
        assert status["capture_age_s"] == 0
        assert status["intent"] == {"id": "auto-1", "kind": "auto", "age_s": 0.,
                                    "time_left_s": pytest.approx(.45)}
        assert status["decision"]["phase"] == "await_reset"
        assert status["decision"]["role"] == "offense"
        assert status["sampled_at_monotonic"] == e.ros.t
    assert pickle.dumps(e.core.executor) == before
    # A read-only availability query must not expire even a stale live gate.
    stale = ForwardObstacleGate(ObstacleConfig(clear_frames=1))
    stale.update(DetectionFrame(1, e.ros.t - .4), e.ros.t - .4)
    e.core.executor.obstacle_gates[e.core.robot_id] = stale
    before = pickle.dumps(e.core.executor)
    assert not e.node.status()["resume"]["available"]
    with pytest.raises(ValueError, match="clear front-camera"):
        e.core.executor.check_resume(e.core.robot_id, e.core.executor.frame_seq, e.ros.t)
    assert pickle.dumps(e.core.executor) == before


def test_resume_capture_does_not_block_reader_from_following_stop_under_control_lock(ros_node):
    e = ros_node
    awaiting_resume(e)
    e.node.lock.acquire()
    try:
        stream = io.StringIO('{"command":"resume"}\n{"command":"stop"}\n')
        worker = threading.Thread(target=e.node.inbox.read, args=(stream,))
        worker.start()
        worker.join(1)
        assert not worker.is_alive() and e.node.inbox.stop_seen.is_set()
        assert e.node.inbox.normal.qsize() == 1  # Resume is queued, not consumed.
    finally:
        e.node.lock.release()
    e.node.on_tick()
    assert e.node.hard_stopped and not e.core.executor.armed
    assert len(e.ros.published["/emergency_stop"]) == 1
    assert not any(row.get("command") == "resume" and row.get("accepted") for row in e.events)


@pytest.mark.parametrize("prior_state", ["before_goal", "returning_home", "previous_cycle"])
def test_cue_before_completed_current_goal_cannot_gain_authority_while_lock_is_held(ros_node, prior_state):
    e = ros_node
    if prior_state == "previous_cycle":
        awaiting_resume(e)
        e.node.inbox.put({"command": "resume"})
        e.node.on_tick()
        assert resume_result(e)["accepted"]
    else:
        e.node.inbox.put({"command": "arm", "epoch": "game-1"})
        e.node.on_tick()
        if prior_state == "returning_home":
            row = frame(e.core, e.ros.t)
            row["robots"][e.core.robot_id] = {"x": 0., "y": 0., "yaw": 0., "goal": 1}
            assert e.core.ingest(row)["accepted"]
            assert e.core.ingest(intent(e.core, e.ros.t))["accepted"]
            e.node.on_tick()
            assert e.core.last_decision.phase == "return_home"
    assert not e.node.status()["resume"]["available"]
    e.node.lock.acquire()
    try:
        worker = threading.Thread(target=e.node.inbox.put, args=({"command": "resume"},))
        worker.start()
        worker.join(1)  # No completed marker: refusal need not wait for the lock.
        assert not worker.is_alive()
        goal_frame(e, 1, tick=False)
        goal_frame(e, 0, tick=False)
        e.core.tick()  # Now this cycle genuinely reaches await_reset.
        assert e.node.status()["resume"]["available"]
    finally:
        e.node.lock.release()
    e.node.on_tick()
    assert not resume_result(e)["accepted"]
    expected = "goal cycle" if prior_state == "previous_cycle" else "not completed before resume receipt"
    assert expected in resume_result(e)["reason"]
    assert e.core.executor.controllers[e.core.robot_id]._goal_latched


def test_completion_after_stamped_receipt_before_marker_read_cannot_authorize_cue(ros_node):
    e = ros_node
    e.node.inbox.put({"command": "arm", "epoch": "game-1"})
    e.node.on_tick()
    stamped, release = threading.Event(), threading.Event()
    original_capture = e.node.inbox.capture_resume
    def delayed_capture(*, pressed_at):
        stamped.set()
        assert release.wait(1)
        return original_capture(pressed_at=pressed_at)
    e.node.inbox.capture_resume = delayed_capture
    pressed_at = e.ros.t
    worker = threading.Thread(target=e.node.inbox.put, args=({"command": "resume"},))
    worker.start()
    assert stamped.wait(1)
    goal_frame(e, 1)
    goal_frame(e, 0)
    assert e.core.completed_goal_cycle[2] > pressed_at
    release.set()
    worker.join(1)
    assert not worker.is_alive()
    e.node.on_tick()
    assert not resume_result(e)["accepted"]
    assert "not completed before resume receipt" in resume_result(e)["reason"]


def test_completed_cycle_cue_may_use_cleared_goal_evidence_that_arrives_before_processing(ros_node):
    e = ros_node
    awaiting_resume(e)
    marker = e.core.completed_goal_cycle
    goal_frame(e, 1)
    assert not e.node.status()["resume"]["available"]
    e.node.lock.acquire()
    try:
        worker = threading.Thread(target=e.node.inbox.put, args=({"command": "resume"},))
        worker.start()
        worker.join(1)
        assert not worker.is_alive()
        goal_frame(e, 0, tick=False)
        e.core.tick()
        assert e.node.status()["resume"]["available"]
        assert e.core.completed_goal_cycle == marker  # Repeated ticks retain FIRST completion time.
    finally:
        e.node.lock.release()
    e.node.on_tick()
    assert resume_result(e)["accepted"]


def test_status_requires_control_tick_completion_marker_and_cannot_create_it(ros_node):
    e = ros_node
    e.node.inbox.put({"command": "arm", "epoch": "game-1"})
    e.node.on_tick()
    goal_frame(e, 1, tick=False)
    goal_frame(e, 0, tick=False)
    e.core.executor.check_resume(e.core.robot_id, e.core.executor.frame_seq, e.ros.t)
    assert e.core.completed_goal_cycle is None
    for _ in range(3):
        assert not e.node.status()["resume"]["available"]
        assert e.core.completed_goal_cycle is None
    e.node.on_tick()
    assert e.node.status()["resume"]["available"]


@pytest.mark.parametrize("late", ["deadline", "stop", "new_cycle"])
def test_resume_final_consumption_fence_after_slow_readiness_query(ros_node, late):
    e = ros_node
    awaiting_resume(e)
    e.node.inbox.put({"command": "resume"})
    original_guard = e.node.guard_problem
    calls = []
    def delayed_guard(*, camera=True):
        calls.append(camera)
        if len(calls) == 2:  # Last readiness query, after initial age/native checks.
            if late == "deadline":
                e.ros.t += .201
            elif late == "stop":
                e.node.inbox.put({"command": "stop"})
            else:
                assert e.core.operator({"command": "resume", "frame_seq": e.core.executor.frame_seq})["accepted"]
                goal_frame(e, 1, tick=False)
                goal_frame(e, 0, tick=False)
        return original_guard(camera=camera)
    e.node.guard_problem = delayed_guard
    with e.node.lock:
        e.node.operator(*e.node.inbox.get())
    assert len(calls) == 2
    assert not resume_result(e)["accepted"]
    assert ("expired" if late == "deadline" else "stopping" if late == "stop" else "goal cycle changed") in resume_result(e)["reason"]
    assert e.core.executor.controllers[e.core.robot_id]._goal_latched
    if late == "stop":
        e.node.on_tick()
        assert e.node.hard_stopped and len(e.ros.published["/emergency_stop"]) == 1
