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


def core_fixture(pitch=None):
    core = ingress.IngressCore(source_id="camera", source_session="session-1", producer_id="perception",
        epoch="trial-1", boot_id="boot", robot_map={"striker": "pupper-1", "keeper": "pupper-2"},
        calibration_revision=1, field=FIELD, pitch=pitch)
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


@pytest.mark.parametrize("tracking", ["tags", "colors"])
@pytest.mark.parametrize("capture_yaw", [.5, math.pi - .02, -math.pi + .02])
def test_capture_heading_agrees_across_both_control_adapters(tracking, capture_yaw):
    from final_project.overhead_perception.fieldcam.gameplay import to_observation
    packet = state()
    packet["tracking"] = tracking
    row = packet["robots"]["striker"]
    row.update(yaw_at_capture=capture_yaw, wz=5.)
    # Display heading is deliberately different; capture heading must survive
    # delay, wrap and reacquisition without subtracting angular speed twice.
    row["yaw"] = capture_yaw if tracking == "colors" else capture_yaw + row["wz"] * row["age_s"]
    normalized = core_fixture().normalize(packet, 10.04)
    observation = to_observation(packet)
    assert normalized["robots"]["pupper-1"]["yaw"] == pytest.approx(capture_yaw)
    assert observation.robot.yaw == pytest.approx(capture_yaw)
    assert observation.captured_at == packet["captured_at"]
    # Frozen pre-upgrade streams retain their actual color/tag convention.
    del row["yaw_at_capture"]
    assert core_fixture().normalize(packet, 10.04)["robots"]["pupper-1"]["yaw"] == pytest.approx(capture_yaw)
    assert to_observation(packet).robot.yaw == pytest.approx(capture_yaw)


def test_invalid_explicit_capture_heading_is_not_repaired_from_display():
    from final_project.overhead_perception.fieldcam.gameplay import to_observation
    packet = state()
    packet["robots"]["striker"]["yaw_at_capture"] = None
    with pytest.raises(ValueError, match="capture yaw"):
        core_fixture().normalize(packet, 10.04)
    assert to_observation(packet).robot is None


def test_gameplay_does_not_repair_missing_capture_from_display_stamp():
    from final_project.overhead_perception.fieldcam.gameplay import to_observation
    packet = state()
    del packet["captured_at"]
    observation = to_observation(packet)
    assert math.isnan(observation.captured_at)
    assert observation.robot is None and observation.ball is None


@pytest.mark.parametrize("change,accepted", [
    ({}, True),
    ({"measured_this_frame": False}, False),
    ({"measured_at": 999.99}, False),
    ({"age_s": .08}, False),
    ({"x": float("nan")}, False),
    ({"vx": float("inf")}, False),
])
def test_control_and_referee_share_capture_row_admission(change, accepted):
    from final_project.overhead_perception.fieldcam.gameplay import to_observation
    from final_project.pupper_fc.match_director import measured
    packet = state()
    packet["ball"] = {**packet["robots"]["striker"], **change}
    try:
        control = core_fixture().normalize(packet, 10.04)["ball"]
    except ValueError:
        control = None  # Ingress revokes the frame; the referee drops this row.
    referee = measured(packet["ball"], packet["captured_at"], packet["frame_age_s"])
    gameplay = to_observation(packet).ball
    assert (control is not None, referee is not None, gameplay is not None) == (accepted,) * 3
    if accepted:
        assert (control["x"], control["y"]) == pytest.approx(referee)
        assert (gameplay.x, gameplay.y) == pytest.approx(referee)


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
    core.sync = (-10.01, -9.992, 10.03)  # 18 ms RTT alone fits, full bound does not.
    with pytest.raises(ValueError, match="combined capture clock uncertainty"):
        core.normalize(state(), 10.04)


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


def test_selected_pitch_preserves_marker_coordinates_and_admits_real_goal_fronts():
    from final_project.overhead_perception.fieldcam.pitch import Pitch
    goals = {"west": {"x": -2.13, "y_min": -.3048, "y_max": .3048},
             "east": {"x": 2.13, "y_min": -.2048, "y_max": .4048}}
    pitch = Pitch.from_field(FIELD, goals).record()
    packet = state()
    packet.update(pitch=pitch, goals=goals)
    row = packet["robots"]["striker"]
    packet["ball"] = {**row, "x": 2.0, "vx": 0.}
    event = core_fixture(pitch).normalize(packet, 10.04)
    assert event["ball"]["x"] == 2.0  # playable strip beyond the marker rectangle
    assert event["goals"]["right"] == {"x": 2.13, "y": pytest.approx(.1)}
    changed = deepcopy(packet)
    changed["pitch"]["goals"]["east"]["y_max"] += .01
    with pytest.raises(ValueError, match="pitch geometry differs"):
        core_fixture(pitch).normalize(changed, 10.04)
    with pytest.raises(ValueError, match="goal geometry differs"):
        core_fixture().normalize(packet, 10.04)  # legacy peers cannot silently adopt v2
