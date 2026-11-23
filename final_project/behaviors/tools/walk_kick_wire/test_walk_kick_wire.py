"""Host checks for the walk_kick_wire harness logic; no ROS, devices or native code."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time
from xml.etree import ElementTree as ET

import pytest
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def load(name):
    spec = importlib.util.spec_from_file_location(f"walk_kick_wire_{name}", HERE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


run = load("run")
client = load("client")


def recipe_output(tmp_path, package="neural_controller"):
    build = tmp_path / package
    (build / "prefix/lib").mkdir(parents=True)
    (build / "prefix/lib/libneural_controller.so").write_bytes(b"\x7fELF selected")
    files = {"lib/libneural_controller.so": hashlib.sha256(b"\x7fELF selected").hexdigest()}
    (build / "installed-files.json").write_text(json.dumps(files))
    (build / "source-closure.json").write_text(json.dumps({"package": package, "source_commit": "abc", "files": {"a": "1"}}))
    (build / "recipe.sha256").write_text("feed  /recipe.sh\n")
    (build / "exit-code").write_text("0\n")
    return build, run.digest(build / "installed-files.json")


def test_build_is_accepted_only_with_its_receipt_and_unchanged_prefix(tmp_path):
    build, receipt = recipe_output(tmp_path)
    accepted = run.verify_build(build, receipt, "neural_controller")
    assert accepted["installed_files_sha256"] == receipt and accepted["source_commit"] == "abc"
    with pytest.raises(ValueError, match="receipt"):
        run.verify_build(build, "0" * 64, "neural_controller")
    with pytest.raises(ValueError, match="not a joy_utils build"):
        run.verify_build(build, receipt, "joy_utils")
    (build / "prefix/lib/libneural_controller.so").write_bytes(b"\x7fELF rebuilt")
    with pytest.raises(ValueError, match="prefix differs"):
        run.verify_build(build, receipt, "neural_controller")


def test_incomplete_build_is_refused(tmp_path):
    build, receipt = recipe_output(tmp_path)
    (build / "exit-code").write_text("1\n")
    with pytest.raises(ValueError, match="did not complete"):
        run.verify_build(build, receipt, "neural_controller")


def test_checkout_must_equal_the_compiled_closure():
    path = "pupper_gait_deploy/src/neural_controller.cpp"
    actual = run.digest(ROOT / path)
    assert run.checkout_matches({"src/neural_controller.cpp": actual}, {path: "src/neural_controller.cpp"}) == {path: actual}
    with pytest.raises(ValueError, match="differs from the selected build closure"):
        run.checkout_matches({"src/neural_controller.cpp": "0" * 64}, {path: "src/neural_controller.cpp"})


def test_controller_overlay_map_covers_the_maintained_native_sources():
    rows = run.controller_overlays()
    assert rows["pupper_gait_deploy/src/neural_controller.cpp"] == "src/neural_controller.cpp"
    assert rows["pupper_gait_deploy/src/command_deadline.hpp"] == "include/neural_controller/command_deadline.hpp"
    assert rows["pupper_gait_deploy/test/test_neural_controller_safety.cpp"] == "test/test_neural_controller_safety.cpp"


@pytest.mark.parametrize("change, message", [
    (dict(domain=168), "reserved"),
    (dict(env={"PUPPER_PROBE_SESSION_ID": "live"}), "robot session"),
    (dict(env={"ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET"}), "discovery"),
    (dict(env={"AMENT_PREFIX_PATH": "/home/pi/pupperv3-monorepo/ros2_ws/install/neural_controller:/opt/ros/jazzy"}),
     "overlay"),
])
def test_device_context_is_refused_before_any_process_starts(monkeypatch, change, message):
    monkeypatch.setattr(run.sys, "platform", "linux")
    for key in run.SESSION_MARKERS:
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv("AMENT_PREFIX_PATH", "/opt/ros/jazzy")
    for key, value in change.get("env", {}).items():
        monkeypatch.setenv(key, value)
    with pytest.raises(ValueError, match=message):
        run.refuse_device_context(change.get("domain", 94), "ros2-control")


def test_memory_description_replaces_only_the_hardware_selection():
    text = (ROOT / "foundation/hardware/floor_stock/baseline-expanded.urdf").read_text()
    config = yaml.safe_load((ROOT / "foundation/hardware/lab6-session.yaml").read_text())
    xml = client.memory_urdf(text, config)
    block = next(ET.fromstring(xml).iter("ros2_control"))
    original = next(ET.fromstring(text).iter("ros2_control"))
    assert [p.text for p in block.iter("plugin")] == ["mock_components/GenericSystem"]
    assert block.get("name") == "walk_kick_wire_memory" and block.get("type") == "system"
    assert "control_board" not in xml and "can_id" not in xml and "homing" not in xml
    # Same joints, sensor and interfaces as the selected description.
    shape = lambda b: [(e.tag, e.get("name"), [(i.tag, i.get("name")) for i in e if i.tag != "param"])
                       for e in b if e.tag in ("joint", "sensor")]
    assert shape(block) == shape(original)
    walk = config["gemini_walk"]["ros__parameters"]
    first = block.find("joint")
    position = first.find("state_interface[@name='position']/param").text
    assert float(position) == walk["default_joint_pos"][walk["joint_names"].index(first.get("name"))]
    imu = block.find("sensor")
    assert imu.find("state_interface[@name='orientation.w']/param").text == "1.0"


def test_applied_command_offset_comes_from_the_selected_walker_policy():
    policy = json.loads((ROOT / "pupper_gait_deploy/policies/default_policy.json").read_text())
    assert client.command_offset(policy) == 6
    assert client.command_offset({}) == 6
    with pytest.raises(client.Failure):
        client.command_offset({"obs_spec": {"frame_order": "oldest_first", "components": []}})


def test_synthetic_board_receipt_is_read_as_healthy_by_the_production_reader(tmp_path):
    from final_project.behaviors import board_imu as imu
    path = tmp_path / "fc-board-health.bin"
    writer = client.BoardReceipt(path, os.getpid())
    reader = imu.BoardReceipt(path, os.getpid(), "strict")
    gate = imu.MatchImu(reader, lambda: (time.monotonic(), 0., 0.))
    try:
        for _ in range(60):
            if gate.refresh()["state"] == "healthy":
                break
            time.sleep(.01)
        assert gate.state == "healthy", gate.reason
        other = imu.BoardReceipt(path, os.getpid() + 1)  # pinned to a different manager
        with pytest.raises(ValueError, match="identity"):
            other.read()
        other.close()
    finally:
        writer.close()
        reader.close()


def test_trace_audit_reports_only_unexpected_device_paths(tmp_path):
    (tmp_path / "syscalls.1").write_text('openat(AT_FDCWD, "/dev/null", O_RDWR) = 3\n'
                                         'openat(AT_FDCWD, "/dev/shm/fastrtps_1", O_RDWR) = 4\n'
                                         'openat(AT_FDCWD, "/dev/urandom", O_RDONLY) = 5\n')
    (tmp_path / "syscalls.2").write_text('openat(AT_FDCWD, "/dev/spidev0.0", O_RDWR) = -1 EACCES\n')
    assert run.audit_trace(tmp_path) == {"trace_files": 2, "unexpected_devices": ["/dev/spidev0.0"]}


def test_field_configuration_is_accepted_by_the_runner_loader():
    from final_project.behaviors.fc_executor_bridge import load_fields
    fields = load_fields(client.field_config())
    assert fields["pupper-1"].attack_sign == 1 and fields["pupper-2"].attack_sign == -1


def test_clip_receipt_pattern_matches_the_native_format():
    match = client.CLIP.match("kick_right playing frame 186 queued 0 execution " + "c" * 32 + " request none")
    assert match.groups() == ("kick_right", "186", "0", "c" * 32, "none")
    assert client.CLIP.match("kick_right holding frame 0 queued 0 execution " + "c" * 32 + " request none") is None


def test_latency_scenario_uses_the_selected_arena_camera_latency():
    dating = run.capture_dating("kick-entry-arena-latency")
    arena = json.loads((ROOT / run.ARENA_CONFIG).read_text())
    assert dating["capture_latency_s"] == arena["camera"]["latency_s"]
    assert dating["capture_latency_source"]["sha256"] == run.digest(ROOT / run.ARENA_CONFIG)
    assert run.capture_dating("walk-kick-restore-stop") == {"capture_latency_s": 0.}


def timing_fixture(backend, switch_response={"ok": True}):
    """One kick request 120 ms after its capture: 30 ms of start_before budget, then service rounds."""
    t0 = 1_000_000_000_000  # request issue, ns
    s = lambda ms: t0 + round(ms * 1e6)
    captured = s(-120)
    rows = [
        {"kind": "ingest", "id": 0, "t_ns": s(-5), "before_ns": s(-5.1), "event": "frame", "seq": 7, "accepted": True},
        {"kind": "kick_request", "id": 1, "t_ns": s(.2), "attempt_id": 1, "epoch": "e", "side": "right",
         "issued_at": t0 / 1e9, "start_before": (captured + 150_000_000) / 1e9,
         "evidence": {"frame_seq": 7, "frame_captured_at": captured / 1e9, "own_captured_at": captured / 1e9,
                      "intent": {"issued_at": captured / 1e9, "valid_until": captured / 1e9 + .3}}},
        {"kind": "submit", "id": 2, "t_ns": s(1), "attempt_id": 1, "exit_ns": s(3000), "outcome": "bound"},
        {"kind": "gate", "id": 3, "t_ns": s(1.1), "before_ns": s(1), "entry": True, "call": None, "attempt_id": 1,
         "reason": None},
        {"kind": "switch_to", "id": 4, "t_ns": s(8), "controller": "trick_leg_kick", "start_clip": "kick_right"},
        {"kind": "call", "id": 5, "t_ns": s(9), "service": "/trick_leg_kick/set_parameters", "switch": 4,
         "exit_ns": s(31), "outcome": "response"},
        {"kind": "rpc_send", "id": 6, "t_ns": s(9.5), "service": "/trick_leg_kick/set_parameters", "call": 5,
         "request": {"type": "SetParameters_Request", "parameters": {"execution_request_id": "a" * 32}},
         "send_returned_ns": s(9.6), "response_ns": s(12)},
        {"kind": "call", "id": 7, "t_ns": s(32), "service": "/controller_manager/switch_controller", "switch": 4,
         "exit_ns": s(60), "outcome": "CancelledError: "},
        {"kind": "gate", "id": 8, "t_ns": s(32.1), "before_ns": s(32), "entry": True, "call": 7, "attempt_id": 1,
         "reason": "field kick authority expired or was revoked before controller activation"},
        {"kind": "rpc_send", "id": 9, "t_ns": s(33), "service": "/controller_manager/switch_controller", "call": 7,
         "request": {"type": "SwitchController_Request", "activate": ["trick_leg_kick"], "deactivate": ["gemini_walk"]},
         "send_returned_ns": s(33.1), "response_ns": s(45), "response": switch_response},
        {"kind": "clip_receipt", "id": 10, "t_ns": s(70), "controller": "trick_leg_kick", "received_ns": s(70),
         "frame": 50},
    ]
    scenario = {"capture_latency_s": .12, "marks_ns": {},
                "field_frames": [{"publish_ns": s(-6), "seq": 7, "captured_at_ns": captured, "kind": "attack"}],
                "clip_status": [{"receipt_ns": s(68), "text": "kick_right playing frame 50 queued 0 execution "
                                 + "a" * 32 + " request none"}],
                "native_authority": [], "gateway_status": [], "emergency_stop_receipts_ns": [], "runner_events": []}
    manager = [{"event": "switch", "activate": ["trick_leg_kick"], "entry_ns": s(40), "response_ns": s(41),
                "activations": [{"controller": "trick_leg_kick", "begin_ns": s(40.2), "end_ns": s(40.9), "ok": True}],
                "deactivations": []},
               {"event": "first_update", "controller": "trick_leg_kick", "begin_ns": s(42), "end_ns": s(42.5)}]
    return client.derive_timing({"rows": rows}, scenario, manager if backend == "fixture" else [], backend)


def test_timing_derives_budget_gates_and_exact_fixture_entry():
    entry = timing_fixture("fixture")["kick_entries"][0]
    assert entry["budget_ms"] == pytest.approx(30, abs=1e-3)
    assert entry["start_before_basis"] == "capture + MAX_CAPTURE_GAP_S"
    assert entry["observation"]["capture_to_issue_ms"] == pytest.approx(120, abs=1e-3)
    assert entry["observation"]["publish_to_runner_ingest_ms"] == pytest.approx(1, abs=1e-3)
    gate = entry["last_activation_gate"]
    assert gate["where"] == "/controller_manager/switch_controller" and gate["reason"].startswith("field kick")
    assert gate["remaining_to_start_before_ms"] == pytest.approx(-2.1, abs=1e-3)
    params = entry["service_rounds"][0]
    assert params["context"] == "switch_to(trick_leg_kick, start_clip='kick_right')"
    assert params["round_trip_ms"] == pytest.approx(2.5, abs=1e-3) and params["poll_delay_ms"] == pytest.approx(19, abs=1e-3)
    native = entry["native_entry"]
    assert native["basis"].startswith("exact") and native["on_activate_begin_ms"] == pytest.approx(40.2, abs=1e-3)
    assert native["first_update_begin_ms"] == pytest.approx(42, abs=1e-3)
    assert entry["first_clip"]["observer_frame"] == 50
    switch = entry["kick_switch"]
    assert switch["call_outcome"].startswith("CancelledError") and switch["poll_delay_ms"] is None


def test_real_manager_timing_reports_bounds_never_rpc_completion_as_entry():
    native = timing_fixture("ros2-control")["kick_entries"][0]["native_entry"]
    assert native["basis"].startswith("bounds only")
    assert native["not_before_ms"] == pytest.approx(33, abs=1e-3)
    assert native["not_after_ms"] == pytest.approx(45, abs=1e-3)  # earliest of response and first clip receipt
    assert "on_activate_begin_ms" not in native


def test_decision_ages_count_ticks_at_or_over_the_capture_gap():
    ages = client.decision_ages({"decision_ages": [[1, 128., "align"], [2, 151., "align"], [3, 20., "approach"]]}, .15)
    assert ages["align"]["ticks"] == 2 and ages["align"]["at_or_over_capture_gap"] == 1
    assert ages["approach"]["max_ms"] == 20.


def test_rpc_probe_is_pass_through_and_never_records_a_cancel_as_a_response(monkeypatch):
    import sys
    import types

    class Future:
        def __init__(self):
            self.callbacks, self.state = [], None

        def add_done_callback(self, callback):
            self.callbacks.append(callback)

        def finish(self, state):
            self.state = state
            for callback in self.callbacks:
                callback(self)

        def cancelled(self):
            return self.state == "cancelled"

        def result(self):
            return None if self.state == "cancelled" else types.SimpleNamespace(ok=True)

    class Client:
        srv_name = "/controller_manager/switch_controller"

        def call_async(self, request):
            if request == "boom":
                raise RuntimeError("transport")
            return Future()

    rclpy = types.ModuleType("rclpy")
    rclpy.client = types.ModuleType("rclpy.client")
    rclpy.client.Client = Client
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.client", rclpy.client)
    probe = client.Probe("runner")
    client.install_rpc_probe(probe)
    answered, cancelled = Client().call_async(object()), Client().call_async(object())
    answered.finish("done")
    cancelled.finish("cancelled")
    with pytest.raises(RuntimeError, match="transport"):
        Client().call_async("boom")
    first, second, third = probe.rows
    assert first["response"] == {"ok": True} and "response_ns" in first
    assert "cancelled_ns" in second and "response_ns" not in second
    assert third["error"] == "RuntimeError: transport"


def test_refused_kick_switch_is_not_a_native_entry_bound():
    native = timing_fixture("ros2-control", switch_response={"ok": False})["kick_entries"][0]["native_entry"]
    assert native["basis"].startswith("kick switch refused") and "not_after_ms" not in native
