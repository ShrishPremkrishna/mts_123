"""No ROS/hardware: raw clock/frame admission and existing Executor acceptance."""
from copy import deepcopy
import json
import math
import select
import socket
import time

import pytest

import overhead_ingress as ingress
import fc_executor_bridge as bridge
from final_project.overhead_perception.fieldcam.net import StatePublisher, REPLY_PROTOCOL, STATE_PROTOCOL

FIELD = {"length": 3.66, "width": 2.44, "goal_width": .6096, "ball_radius": .11,
         "robot_radius": .16, "bumper_offset": .19}
CONFIG = {"field": FIELD, "attack_signs": {"pupper-1": 1, "pupper-2": -1}}


def core_fixture():
    core = ingress.IngressCore(source_id="camera", source_session="session-1", producer_id="perception",
        epoch="trial-1", boot_id="boot", robot_map={"striker": "pupper-1", "keeper": "pupper-2"},
        calibration_revision=1, field=FIELD)
    probe = core.make_probe(9.98)
    core.accept_probe({**core.identity(), "protocol": REPLY_PROTOCOL, "nonce": probe["nonce"],
                       "received_mono": 19.985, "sent_mono": 19.986}, 9.992)
    return core


def state(seq=1, capture=1000., *, wall=None, mono=None):
    stamp = capture + .02
    wall = capture + .03 if wall is None else wall
    mono = wall - 980 if mono is None else mono
    robot = {"visible": True, "measured_this_frame": True, "measured_at": capture,
             "x": -.98, "y": 0., "yaw": .01, "vx": 1., "vy": 0., "wz": .5,
             "age_s": stamp - capture}
    return {"seq": seq, "stamp": stamp, "captured_at": capture, "frame_age_s": stamp - capture,
            "calibration_valid": True, "calibration_revision": 1,
            "robots": {"striker": robot, "keeper": {**robot, "x": 1., "yaw": math.pi}},
            "ball": {"visible": False}, "field": {k: FIELD[k] for k in ("length", "width", "ball_radius")},
            "goals": {"west": {"x": -1.83, "y_min": -.3048, "y_max": .3048},
                      "east": {"x": 1.83, "y_min": -.3048, "y_max": .3048}},
            "_transport": {"protocol": STATE_PROTOCOL, "source_id": "camera", "source_session": "session-1",
                           "clock_id": "monotonic:session-1", "live": True, "wire_seq": seq,
                           "clock_valid": True, "sent_mono": mono, "sent_wall": wall,
                           "pair_span_s": .0001, "clock_start_wall": 999.}}


def test_actual_bridge_accepts_original_capture_and_cancels_on_calibration_loss():
    core = core_fixture()
    now = [10.04]
    fc = bridge.BridgeCore("pupper-1", "perception", "boot", bridge.load_fields(CONFIG),
                           clock=lambda: now[0], required_participants=("pupper-1", "pupper-2"))
    assert not fc.ingest(core.normalize(state(), now[0]))["accepted"]  # Does not auto-arm.
    assert not fc.executor.armed
    assert fc.operator({"command": "arm", "epoch": "trial-1"})["accepted"]
    frame = core.normalize(state(2, 1000.02), now[0] + .02)
    now[0] += .02
    assert fc.ingest(frame)["accepted"]
    assert frame["captured_at"] < now[0] - .04  # Clock bound + capture + transit, never receipt.
    pose = frame["robots"]["pupper-1"]
    assert pose["x"] == pytest.approx(-1.) and pose["yaw"] == pytest.approx(0.)
    assert pose["captured_at"] == frame["captured_at"]
    assert fc.ingest({"event": "intent", "producer_id": "perception", "clock": fc.clock_name,
                     "epoch": "trial-1", "seq": 1, "robot_id": "pupper-1", "kind": "move",
                     "intent_id": "fixed", "target": {"x": 0., "y": 0., "yaw": 0.},
                     "issued_at": now[0], "valid_until": now[0] + .2})["accepted"]
    assert fc.tick().command.vx > 0
    bad = state(3, 1000.04)
    bad["calibration_valid"] = False
    with pytest.raises(ValueError, match="calibration invalid"):
        core.normalize(bad, 10.08)
    invalidation = core.revoke("calibration invalid")
    assert invalidation["captured_at"] is None
    assert not fc.ingest(invalidation)["accepted"]
    assert fc.executor.frame is None and fc.tick().command.vx == 0
    assert fc.executor.armed  # Only the operator owns arm/Stop.
    assert core.revoke("still invalid") is None


def test_missing_and_coasting_tracks_never_get_new_capture():
    core = core_fixture()
    packet = state()
    packet["robots"]["keeper"].update(measured_at=999.99, measured_this_frame=False)
    packet["ball"] = {**packet["robots"]["striker"], "measured_at": 999.999999}
    frame = core.normalize(packet, 10.04)
    assert set(frame["robots"]) == {"pupper-1"} and frame["ball"] is None


@pytest.mark.parametrize("change", [
    {"source_id": "wrong"}, {"source_session": "restarted"}, {"clock_id": "other"},
    {"live": False}, {"clock_valid": False}, {"pair_span_s": .1},
    {"sent_wall": 2000.}, {"sent_mono": 5000.}, {"clock_start_wall": 1001.},
])
def test_bad_source_identity_clock_and_non_live_inputs_rejected(change):
    core = core_fixture()
    packet = state()
    packet["_transport"].update(change)
    with pytest.raises(ValueError):
        core.normalize(packet, 10.04)


@pytest.mark.parametrize("change", [{"calibration_revision": 2}, {"calibration_valid": False},
    {"captured_at": 998.}, {"frame_age_s": 0.}, {"captured_at": float("nan")}])
def test_bad_health_capture_and_age_rejected(change):
    core = core_fixture()
    packet = state()
    packet.update(change)
    with pytest.raises(ValueError):
        core.normalize(packet, 10.04)


def test_delayed_reordered_and_silent_expiry_fail_closed():
    core = core_fixture()
    core.normalize(state(), 10.04)
    with pytest.raises(ValueError, match="reordered"):
        core.normalize(state(), 10.05)
    with pytest.raises(ValueError, match="stale"):
        core.normalize(state(2, 1000.02), 10.5)
    invalid = core.expire(10.5)
    assert invalid["captured_at"] is None
    assert core.expire(10.6) is None


def test_wall_step_and_calibration_revision_require_new_binding_even_after_recovery():
    for defect in ("clock", "calibration"):
        core = core_fixture()
        core.normalize(state(), 10.04)
        packet = state(2, 1000.02)
        if defect == "clock":
            packet["_transport"]["sent_wall"] += .2
        else:
            packet["calibration_revision"] = 2
        with pytest.raises(ValueError):
            core.normalize(packet, 10.06)
        with pytest.raises(ValueError, match="explicitly rebind"):
            core.normalize(state(3, 1000.04), 10.08)


def test_geometry_change_and_omitted_measurement_contract_rejected():
    packet = state()
    packet["field"]["length"] += .2
    with pytest.raises(ValueError, match="geometry"):
        core_fixture().normalize(packet, 10.04)
    packet = state()
    del packet["robots"]["striker"]["measured_this_frame"]
    assert "pupper-1" not in core_fixture().normalize(packet, 10.04)["robots"]


@pytest.mark.parametrize("key,value", [("field", None), ("field", []), ("goals", []),
    ("goals", {"west": None}), ("stamp", 10**1000), ("robots", [])])
def test_malformed_packet_containers_and_huge_numbers_are_admission_errors(key, value):
    packet = state()
    packet[key] = value
    with pytest.raises(ValueError):
        core_fixture().normalize(packet, 10.04)


def test_combined_capture_uncertainty_includes_pair_wall_tolerance_and_drift():
    core = core_fixture()
    core.sync_samples.clear()
    core.sync_samples.append((-10.01, -9.992, 10.03))  # RTT alone fits, full bound does not.
    with pytest.raises(ValueError, match="combined capture clock uncertainty"):
        core.normalize(state(), 10.04)


def test_clock_intersects_fresh_bounds_and_each_sample_expires_independently():
    core = core_fixture()
    old_lower, old_upper = core._offset(10.)
    probe = core.make_probe(11.)
    core.accept_probe({**core.identity(), "protocol": REPLY_PROTOCOL, "nonce": probe["nonce"],
                       "received_mono": 21.009, "sent_mono": 21.009}, 11.018)
    tight = core._offset(11.02)
    assert tight[0] > old_lower - .001 and tight[1] < old_upper + .001
    # Beyond the first sample's TTL the broad second measurement stands alone.
    broad = core._offset(12.)
    assert broad[0] < tight[0] and broad[1] > tight[1]
    with pytest.raises(ValueError, match="absent or expired"):
        core._offset(13.019)


def test_slow_reply_keeps_current_frame_until_its_original_expiry():
    core = core_fixture()
    core.normalize(state(), 10.04)
    now = [10.05]
    receiver = ingress.UDPIngress(core, source_host="127.0.0.1", bind="127.0.0.1", clock=lambda: now[0])
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.bind(("127.0.0.1", 0))
        receiver.peer = sender.getsockname()
        probe = core.make_probe(10.02)
        reply = {**core.identity(), "protocol": REPLY_PROTOCOL, "nonce": probe["nonce"],
                 "received_mono": 20.04, "sent_mono": 20.04}
        sender.sendto(json.dumps(reply).encode(), receiver.sock.getsockname())
        try:
            assert not receiver.poll() and not core.held
            now[0] = 10.5
            assert receiver.poll()[0]["captured_at"] is None and core.held
        finally:
            receiver.close()


@pytest.mark.parametrize("condition", ["wide", "expired", "nonce", "replay", "reverse", "jump"])
def test_sync_failures(condition):
    core = core_fixture()
    if condition == "expired":
        with pytest.raises(ValueError, match="expired"):
            core.normalize(state(), 12.1)
        return
    probe = core.make_probe(10.)
    reply = {**core.identity(), "protocol": REPLY_PROTOCOL, "nonce": probe["nonce"],
             "received_mono": 20.001, "sent_mono": 20.002}
    received = 10.004
    if condition == "wide":
        received = 10.1
    if condition == "nonce":
        reply["nonce"] = "wrong"
    if condition == "reverse":
        reply["sent_mono"] = 20.
    if condition == "jump":
        reply.update(received_mono=30.001, sent_mono=30.002)
    if condition == "replay":
        core.accept_probe(reply, received)
    with pytest.raises(ValueError):
        core.accept_probe(reply, received)


def test_real_udp_loopback_to_normalized_frame_without_ros_or_motor_api():
    core = ingress.IngressCore(source_id="camera", source_session="session-1", producer_id="perception",
        epoch="trial-1", boot_id="boot", robot_map={"striker": "pupper-1"}, calibration_revision=1, field=FIELD)
    receiver = ingress.UDPIngress(core, source_host="127.0.0.1", bind="127.0.0.1")
    publisher = StatePublisher("127.0.0.1", receiver.sock.getsockname()[1], source_id="camera",
                               source_session="session-1", live=True)
    try:
        frames = []
        end = time.monotonic() + 2.
        seq = 0
        while time.monotonic() < end and not frames:
            capture = time.time()
            packet = state(seq, capture, wall=capture + .03)
            packet["stamp"], packet["frame_age_s"] = capture, 0.
            for row in packet["robots"].values():
                row["age_s"] = 0.
            publisher.send(packet)
            frames.extend(e for e in receiver.poll() if e["captured_at"] is not None)
            seq += 1
            time.sleep(.002)
        assert frames, core.last_reason
        frame = frames[-1]
        assert 0 <= time.monotonic() - frame["captured_at"] < .3
        assert frame["clock"] == "linux-monotonic:boot" and frame["event"] == "frame"
        assert set(frame["robots"]) == {"pupper-1"}
        # Replaying the raw packet at a new receipt time cannot refresh it.
        publisher.send(packet)
        assert select.select([receiver.sock], [], [], .2)[0]
        assert any(e["captured_at"] is None for e in receiver.poll())
    finally:
        publisher.close()
        receiver.close()


def test_publish_missing_pins_fails_before_ros_import():
    with pytest.raises(SystemExit) as exc:
        ingress.main(["--publish"])
    assert exc.value.code == 2
