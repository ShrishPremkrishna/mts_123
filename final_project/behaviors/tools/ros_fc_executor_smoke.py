#!/usr/bin/env python3
"""Isolated real-ROS FC bridge/mux/guard test; no devices or robot access.

The native release identity, stock services and observation feedback are fake.
The production bridge CLI, executor, ROS transports, C++ mux and camera guard
are real. Test-only timer/publication tracing is collected inside the bridge
process; observer receipt timing is kept separate. Native receipt expiry has a
separate memory-interface test and is not inferred from this fake consumer.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / "final_project/behaviors")]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def child(args, policy):
    from foundation.hardware import runtime_identity as identity
    from final_project.behaviors import fc_executor_bridge as bridge
    from run_through_kick_node import STOCK_POLICY_SHA256
    assert digest(policy) == STOCK_POLICY_SHA256
    runtime = {"command_topology": "guarded", "manager_pid": -1,
               "manager_start_ticks": -1,
               "files": {"policy": {"path": str(policy), "sha256": digest(policy)}}}
    roles = {"bridge_script": "final_project/behaviors/fc_executor_bridge.py",
             "command_path_evidence": "final_project/behaviors/command_path_evidence.py",
             "fc_integration": "final_project/pupper_fc/integration.py",
             "fc_controller": "final_project/pupper_fc/controller.py",
             "fc_model": "final_project/pupper_fc/model.py",
             "fc_tactics": "final_project/pupper_fc/tactics.py",
             "guard_obstacles": "final_project/pupper_fc/obstacles.py"}
    for role, relative in roles.items():
        path = ROOT / relative
        runtime["files"][role] = {"path": str(path), "sha256": digest(path)}
    identity.verify_loaded = lambda _: runtime
    identity.same_manager = lambda _: True
    guard_pid = int(os.environ["PUPPER_TEST_GUARD_PID"])
    def guard_alive(_):
        try:
            state = Path(f"/proc/{guard_pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
            return state not in ("Z", "X", "T", "t")
        except (FileNotFoundError, ProcessLookupError):
            return False
    identity.same_command_path = guard_alive
    manifest = args.out / "synthetic-manifest.json"
    manifest.write_text(json.dumps({"trial_profile": "walk"}) + "\n")
    receipt = args.out / "synthetic-preflight.json"
    receipt.write_text(json.dumps({"manifest_path": str(manifest)}) + "\n")
    ticks, publications, writes, injections = [], [], [], []
    original_sink = bridge.EventSink
    class TracedSink(original_sink):
        def __init__(self, writer=None, capacity=64):
            actual = writer or (lambda row: print(json.dumps(row, allow_nan=False), flush=True))
            def write(row):
                entry = {"before": time.monotonic(), "after": None}
                writes.append(entry)
                try:
                    return actual(row)
                finally:
                    entry["after"] = time.monotonic()
            super().__init__(write, capacity)
    bridge.EventSink = TracedSink
    original_factory = bridge._node_class
    def instrumented_factory():
        cls = original_factory()
        original_tick, original_publish, original_init = cls.on_tick, cls.publish_command, cls.__init__
        def initialize(node, *a, **kw):
            original_init(node, *a, **kw)
            original_problem = node.stock.problem
            def problem(*a, **kw):
                result = original_problem(*a, **kw)
                command = kw.get("command")
                if (not injections and command is not None and
                        any(abs(getattr(command, key)) > .02 for key in ("vx", "vy", "wz")) and
                        (args.out / "inject-publication-stop").exists()):
                    injections.append({"at": time.monotonic(),
                        "type": "test_only_urgent_Stop_inside_final_stock_check"})
                    node.inbox.put({"command": "stop", "reason": "test-only final-check Stop injection"})
                return result
            node.stock.problem = problem
        def tick(node):
            began = time.monotonic()
            try:
                return original_tick(node)
            finally:
                decision = node.core.last_decision
                ticks.append({"before": began, "after": time.monotonic(),
                              "phase": decision.phase if decision else None,
                              "armed": node.core.executor.armed, "closing": node.closing,
                              "stdout_write_pending_since": writes[-1]["before"]
                                  if writes and writes[-1]["after"] is None else None})
        def publish(node, command):
            intent = node.core.executor.intents[node.core.robot_id]
            frame = node.core.executor.frame
            publications.append({"at": time.monotonic(), "command": asdict(command),
                                 "intent_valid_until": intent.valid_until if intent else None,
                                 "field_captured_at": frame[0] if frame else None})
            return original_publish(node, command)
        cls.__init__, cls.on_tick, cls.publish_command = initialize, tick, publish
        return cls
    bridge._node_class = instrumented_factory
    try:
        return bridge.main(["--execute", "--runtime-receipt", str(receipt),
                            "--robot-id", "pupper-1", "--producer-id", "fixture-1",
                            "--field-config", str(args.out / "field-config.json")])
    finally:
        (args.out / "bridge-timing.json").write_text(json.dumps({
            "ticks": ticks, "publications": publications, "stdout_writes": writes,
            "fault_injections": injections,
            "native_identity_stubbed": True, "guard_pid_observed": guard_pid,
            "scope": "Test-only in-process monotonic entry/exit and command publication tracing"}, indent=2) + "\n")


def main():
    if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1":
        raise SystemExit("Only run in the isolated device-free ROS container.")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=("lifecycle", "guard_death", "publication_stop", "_child"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--mux", default="/mux")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    policy = Path(os.environ["PUPPER_TEST_STOCK_POLICY"]).resolve(strict=True)
    if args.case == "_child":
        return child(args, policy)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import ListControllers
    from rcl_interfaces.srv import GetParameters
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Empty, Float32MultiArray, String
    from sensor_msgs.msg import Joy
    rclpy.init()
    executor = SingleThreadedExecutor()
    processes, streams, events = [], [], []
    reader_enabled = threading.Event(); reader_enabled.set()
    report = {"case": args.case, "passed": False, "groups": [], "hardware": False,
              "native_identity_stubbed": True, "native_watchdog_retested": False}
    began = time.monotonic()

    class Fixture(Node):
        def __init__(self):
            super().__init__("fc_executor_smoke_fixture")
            self.clock_name = None; self.epoch = "executor-smoke-1"
            self.frame_seq = self.intent_seq = self.camera_seq = self.gateway_seq = 0
            self.fields_enabled = self.intents_enabled = False
            self.camera_enabled = True; self.near = False; self.camera_label = "tower"
            self.kind = "move"; self.intent_id = "move-1"; self.pose_x = 0.
            self.latest = (0., 0., 0.); self.commands = []; self.raw = []; self.final = []
            self.statuses = []; self.guard_statuses = []; self.sent = []
            self.estops = self.inventory_reads = self.feedback_count = 0
            self.latched = False; self.last_capture = self.last_lease_deadline = None
            self.ingress = self.create_publisher(String, "/fc/normalized_events", 1)
            self.camera = self.create_publisher(String, "/fc/obstacle_detections", 1)
            self.gateway = self.create_publisher(String, "/estop_controller/stop_status", 10)
            self.feedback = self.create_publisher(Float32MultiArray, "/neural_controller_default/observation", 1)
            self.joy = self.create_publisher(Joy, "/joy", 1)
            self.create_service(ListControllers, "/controller_manager/list_controllers", self.inventory)
            self.create_service(GetParameters, "/neural_controller_default/get_parameters", self.parameters)
            self.create_subscription(Twist, "/kick_cmd_vel", lambda m: self.record(self.commands, m), 1)
            self.create_subscription(Twist, "/fc/requested_cmd_vel", lambda m: self.record(self.raw, m), 1)
            self.create_subscription(Twist, "/cmd_vel", self.consume, 1)
            self.create_subscription(String, "/fc/executor_bridge/status", lambda m: self.statuses.append((time.monotonic(), json.loads(m.data))), QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(String, "/fc/body_command_guard/status", lambda m: self.guard_statuses.append((time.monotonic(), json.loads(m.data))), 1)
            self.create_subscription(Empty, "/emergency_stop", self.emergency, 10)
            self.create_timer(.02, self.authority)
            self.create_timer(.05, self.frame)
            self.create_timer(.10, self.intent)
            self.create_timer(.05, self.camera_frame)
        def record(self, rows, message):
            value = (message.linear.x, message.linear.y, message.angular.z)
            rows.append((time.monotonic(), value))
            return value
        def consume(self, message):
            self.latest = self.record(self.final, message)
        def emergency(self, _):
            self.estops += 1; self.latched = True
        def inventory(self, _, response):
            self.inventory_reads += 1
            row = ControllerState(); row.name = "neural_controller_default"
            row.type = "neural_controller/NeuralController"; row.state = "active"
            row.claimed_interfaces = ["leg_front_l_1/position"]
            response.controller.append(row)
            return response
        def parameters(self, request, response):
            values = {"model_path": str(policy), "max_vx_forward": .49,
                      "max_vx_backward": .3, "speed_cap": .49, "game_buttons": False}
            for name in request.names:
                response.values.append(Parameter(name, value=values[name]).get_parameter_value()
                                       if name in values else Parameter(name).get_parameter_value())
            return response
        def authority(self):
            self.gateway_seq += 1
            payload = {"protocol": "pupper_stop_v1", "instance_id": "fake-stock-gateway",
                       "epoch": 1, "sequence": self.gateway_seq,
                       "selected_controller": "neural_controller_default", "latched": self.latched,
                       **{key: False for key in ("fault", "stop_required", "in_flight", "unknown_remote_submission", "shutting_down")}}
            self.gateway.publish(String(data=json.dumps(payload)))
            values = [0.] * 960; values[6:9] = self.latest
            self.feedback.publish(Float32MultiArray(data=values)); self.feedback_count += 1
            joy = Joy(); joy.buttons = [0] * 13; self.joy.publish(joy)
        def send(self, payload):
            payload = {"producer_id": "fixture-1", "clock": self.clock_name,
                       "epoch": self.epoch, "now": -999., **payload}
            self.sent.append({"at": time.monotonic(), "packet": payload})
            self.ingress.publish(String(data=json.dumps(payload)))
        def frame(self):
            if self.fields_enabled and self.clock_name:
                self.frame_seq += 1; now = time.monotonic(); self.last_capture = now
                self.send({"event": "frame", "seq": self.frame_seq, "captured_at": now,
                           "robots": {"pupper-1": {"x": self.pose_x, "y": 0., "yaw": 0.},
                                      "pupper-2": {"x": 1., "y": 0., "yaw": 3.141592653589793}},
                           "ball": {"x": 0., "y": 0.},
                           "goals": {"left": {"x": -1.83, "y": 0.}, "right": {"x": 1.83, "y": 0.}}})
        def intent(self, *, force=False, robot_id="pupper-1"):
            if (self.intents_enabled or force) and self.clock_name:
                self.intent_seq += 1; now = time.monotonic(); self.last_lease_deadline = now + .45
                packet = {"event": "intent", "seq": self.intent_seq, "issued_at": now,
                          "valid_until": self.last_lease_deadline, "robot_id": robot_id,
                          "intent_id": self.intent_id, "kind": self.kind}
                if self.kind == "move": packet["target"] = {"x": 1., "y": 0., "yaw": 0.}
                elif self.kind == "shoot": packet["target"] = {"x": 1.83, "y": 0.}
                self.send(packet)
        def camera_frame(self):
            if not self.camera_enabled: return
            self.camera_seq += 1
            self.camera.publish(String(data=json.dumps({"schema": "fc_camera_obstacles_v1",
                "producer_instance": "executor-smoke-camera", "sequence": self.camera_seq,
                "captured_at_ros_ns": self.get_clock().now().nanoseconds - 5_000_000,
                "valid": True, "healthy": True, "format": "normalized_xyxy_v1",
                "image_width": 700, "image_height": 572, "horizontal_fov_deg": 220,
                "detections": [{"box": [.4, .3, .6, .85], "confidence": .9, "class_id": self.camera_label}] if self.near else []})))

    node = Fixture(); executor.add_node(node)
    def pump(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=min(.003, max(0., until-time.monotonic())))
            for _ in range(12): executor.spin_once(timeout_sec=0.)
    def wait(test, reason, seconds=4.):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            pump(.01)
            if test(): return
        raise AssertionError(reason)
    def launch(name, command, *, interactive=False, env=None):
        stream = (args.out / (name + ".log")).open("w"); streams.append(stream)
        process = subprocess.Popen(command, stdin=subprocess.PIPE if interactive else None,
                                   stdout=subprocess.PIPE if interactive else stream,
                                   stderr=stream, text=True, bufsize=1, env=env)
        processes.append(process)
        return process
    def nonzero(rows, since):
        return any(at >= since and any(abs(x) > .02 for x in value) for at, value in rows)
    def zero_after(rows, since):
        return any(at >= since and all(abs(x) < 1e-6 for x in value) for at, value in rows)
    def status_matches(test, since=0.):
        return any(at >= since and test(row) for at, row in node.statuses)
    def operator(message):
        index = len(events)
        bridge.stdin.write(json.dumps(message) + "\n"); bridge.stdin.flush()
        wait(lambda: any(e.get("event") == "operator_result" and e.get("command") == message["command"]
                         for e in events[index:]), "operator response " + message["command"])
        result = next(e for e in events[index:] if e.get("event") == "operator_result" and e.get("command") == message["command"])
        assert result["accepted"], result
        return result
    def flowing():
        node.fields_enabled = node.intents_enabled = True
        at = time.monotonic(); node.frame(); pump(.015); node.intent(force=True)
        wait(lambda: nonzero(node.commands, at) and nonzero(node.final, at), "nonzero through bridge/mux/guard")
    try:
        (args.out / "field-config.json").write_text(json.dumps({"field": {
            "length": 3.66, "width": 2.44, "goal_width": .6096,
            "robot_radius": .16, "ball_radius": .11, "bumper_offset": .19},
            "attack_signs": {"pupper-1": 1, "pupper-2": -1}}) + "\n")
        launch("mux", [args.mux, "--ros-args", "-p", "inputs:=[/kick_cmd_vel]", "-p",
                       "timeout_ms:=150", "-r", "/cmd_vel:=/fc/requested_cmd_vel"])
        guard = launch("guard", [sys.executable, "-m", "final_project.behaviors.body_command_guard"])
        env = dict(os.environ, PUPPER_TEST_GUARD_PID=str(guard.pid))
        bridge = launch("bridge", [sys.executable, str(Path(__file__).resolve()), "_child", "--out", str(args.out)], interactive=True, env=env)
        def reader():
            with (args.out / "bridge-stdout.jsonl").open("w") as transcript:
                while True:
                    reader_enabled.wait()
                    line = bridge.stdout.readline()
                    if not line: break
                    transcript.write(line); transcript.flush()
                    try: events.append(json.loads(line))
                    except ValueError: pass
        thread = threading.Thread(target=reader, daemon=True); thread.start()
        wait(lambda: any(e.get("event") == "bridge_ready" for e in events), "bridge startup")
        node.clock_name = next(e["clock"] for e in events if e.get("event") == "bridge_ready")
        wait(lambda: status_matches(lambda s: s.get("native_problem") is None and s.get("guard_problem") is None), "bridge authority and camera readiness")
        pump(.12); assert not nonzero(node.commands, began)
        report["groups"].append("explicit_arm_required")
        operator({"command": "arm", "epoch": node.epoch})
        # Pause periodic input so the refusal remains visible in the latest
        # ingress result reported on the production ROS status topic.
        at = time.monotonic(); node.intent(force=True, robot_id="pupper-2")
        wait(lambda: status_matches(lambda s: (s.get("ingress_result") or {}).get("accepted") is False and
                                    "pinned robot" in s["ingress_result"].get("reason", ""), at), "other robot refusal")
        assert not nonzero(node.commands, at)
        report["groups"].append("other_robot_intent_refused")
        flowing(); report["groups"].append("actual_bridge_mux_guard_nonzero")
        if args.case == "guard_death":
            at = time.monotonic(); count = node.estops; feedback = node.feedback_count
            guard.kill(); guard.wait(timeout=3)
            wait(lambda: bridge.poll() is not None, "bridge closes on guard loss")
            pump(.1)
            assert bridge.returncode == 0 and node.estops == count and node.feedback_count > feedback
            assert zero_after(node.commands, at) and zero_after(node.final, at)
            report["groups"].append("guard_death_closes_bridge_with_zero_and_retains_fake_native_authority")
        elif args.case == "publication_stop":
            at = time.monotonic()
            (args.out / "inject-publication-stop").write_text(str(at) + "\n")
            wait(lambda: bridge.poll() is not None, "final-check Stop closes bridge")
            assert bridge.returncode == 3 and node.estops >= 1 and zero_after(node.commands, at)
            report["groups"].append("test_hook_Stop_inside_final_stock_check_prevents_nonzero_publication")
        else:
            node.intents_enabled = False; deadline = node.last_lease_deadline
            wait(lambda: zero_after(node.commands, deadline) and zero_after(node.final, deadline), "lease expires independently", .9)
            pump(.12); assert not nonzero(node.commands, deadline+.08)
            report.setdefault("expiry", {})["lease"] = {"deadline": deadline}
            report["groups"].append("lease_expiry_zero_with_fresh_field_and_camera")
            flowing()
            node.fields_enabled = False; deadline = node.last_capture + .35
            wait(lambda: zero_after(node.commands, deadline) and zero_after(node.final, deadline), "field expires independently", .8)
            pump(.12); assert not nonzero(node.commands, deadline+.08)
            report["expiry"]["field"] = {"deadline": deadline}
            report["groups"].append("field_expiry_zero_with_fresh_lease_and_camera")
            # Interrupt an actual aligned shooting action, then prove the old
            # explicit shoot ID cannot be replayed even with fresh packets.
            node.kind = "shoot"; node.intent_id = "shot-1"; node.pose_x = -.45
            flowing()
            wait(lambda: status_matches(lambda s: (s.get("decision") or {}).get("phase") == "strike"), "shoot reaches strike")
            node.camera_label = "x" * 128
            at = time.monotonic(); node.near = True
            wait(lambda: status_matches(lambda s: (s.get("decision") or {}).get("phase") == "external_hold", at), "camera cancels action")
            wait(lambda: zero_after(node.commands, at) and zero_after(node.final, at), "hold zeros entire chain")
            node.fields_enabled = node.intents_enabled = False
            node.near = False; at = time.monotonic()
            wait(lambda: node.guard_statuses and node.guard_statuses[-1][1].get("camera_permitted"), "camera clears")
            pump(.12); assert not nonzero(node.commands, at+.05)
            node.fields_enabled = True
            at = time.monotonic(); node.frame(); pump(.015)
            node.fields_enabled = False; node.intent(force=True)
            wait(lambda: status_matches(lambda s: "new intent_id" in
                       (s.get("ingress_result") or {}).get("reason", ""), at), "old shoot ID refused after clearance")
            assert not nonzero(node.commands, at+.05)
            node.intent_id = "shot-2"; flowing()
            report["groups"].append("long_label_hold_cancels_shoot_old_id_cannot_revive_fresh_id_replans")
            # Fill a deliberately small stdout pipe while status replies are
            # unread. Timer/output evidence comes from inside the bridge.
            import array
            import fcntl
            import termios
            fcntl.fcntl(bridge.stdout.fileno(), fcntl.F_SETPIPE_SZ, 4096)
            capacity = fcntl.fcntl(bridge.stdout.fileno(), fcntl.F_GETPIPE_SZ)
            reader_enabled.clear(); pump(.03)
            for _ in range(120): bridge.stdin.write('{"command":"status"}\n')
            bridge.stdin.flush()
            node.intents_enabled = False; deadline = node.last_lease_deadline
            wait(lambda: zero_after(node.commands, deadline), "lease expiry with blocked stdout", 1.)
            pump(.2); assert not nonzero(node.commands, deadline+.08)
            unread = array.array("i", [0])
            fcntl.ioctl(bridge.stdout.fileno(), termios.FIONREAD, unread, True)
            assert unread[0] > 0, "No unread stdout accumulated"
            report["backpressure"] = {"pipe_capacity_bytes": capacity, "unread_pipe_bytes": unread[0], "status_requests": 120,
                                      "lease_deadline": deadline, "reader_resumed_at": time.monotonic()}
            reader_enabled.set()
            report["groups"].append("unread_stdout_does_not_block_lease_expiry")
            # Let the bounded status backlog drain before deliberate Stop.
            pump(.2); at = time.monotonic()
            bridge.stdin.write('{"command":"stop","reason":"smoke explicit Stop"}\n'); bridge.stdin.flush()
            wait(lambda: bridge.poll() is not None, "explicit Stop closes bridge")
            assert bridge.returncode == 3 and node.estops >= 1 and zero_after(node.commands, at)
            report["groups"].append("explicit_Stop_zero_and_session_latch")
        thread.join(timeout=1)
        timing = json.loads((args.out / "bridge-timing.json").read_text())
        ticks, publications = timing["ticks"], timing["publications"]
        spacing = [b["before"]-a["before"] for a, b in zip(ticks, ticks[1:])]
        report["bridge_tick_timing"] = {"samples": len(ticks),
            "max_spacing_s": max(spacing, default=0.),
            "p95_spacing_s": sorted(spacing)[int(.95*(len(spacing)-1))] if spacing else None,
            "max_callback_duration_s": max((row["after"]-row["before"] for row in ticks), default=0.),
            "source": "in-process on_tick tracing, not observer callback intervals"}
        if args.case == "publication_stop":
            assert len(timing["fault_injections"]) == 1
            injected = timing["fault_injections"][0]["at"]
            assert not any(p["at"] >= injected and any(abs(p["command"][key]) > .02
                           for key in ("vx", "vy", "wz")) for p in publications)
            report["publication_stop"] = {"injected_at": injected,
                "nonzero_publications_after_injection": 0,
                "scope": "Test-only Stop queued inside final stock check; separate from natural timing and not an atomic-instant or physical guarantee."}
        if args.case == "lifecycle":
            for kind, row in report["expiry"].items():
                cutoff = row["deadline"]
                first = next(p for p in publications if p["at"] >= cutoff and
                             all(abs(p["command"][key]) < 1e-6 for key in ("vx", "vy", "wz")))
                row["first_zero_publication_at_or_after_last_sent_deadline"] = first["at"]
                row["delay_from_last_sent_deadline_s"] = first["at"] - cutoff
            blocked = [row for row in ticks if row["stdout_write_pending_since"] is not None]
            longest = max((row["before"]-row["stdout_write_pending_since"] for row in blocked), default=0.)
            assert longest >= .1, "stdout writer was not observably blocked during timer ticks"
            cutoff = report["backpressure"]["lease_deadline"]
            first = next(p for p in publications if p["at"] >= cutoff and
                         all(abs(p["command"][key]) < 1e-6 for key in ("vx", "vy", "wz")))
            assert first["at"] < report["backpressure"]["reader_resumed_at"]
            report["backpressure"].update(longest_pending_stdout_write_observed_by_timer_s=longest,
                first_zero_publication=first["at"], delay_from_last_sent_lease_deadline_s=first["at"]-cutoff)
        report["passed"] = True
    except Exception as exc:
        import traceback
        report.update(error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc())
    finally:
        reader_enabled.set()
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=3)
        report.update(elapsed_seconds=time.monotonic()-began, child_exit_codes=[p.returncode for p in processes],
                      all_children_exited=all(p.poll() is not None for p in processes),
                      emergency_stop_requests=node.estops, inventory_reads=node.inventory_reads,
                      feedback_samples=node.feedback_count, events=events,
                      bridge_output_samples=node.commands, mux_output_samples=node.raw,
                      final_output_samples=node.final, bridge_status=node.statuses, guard_status=node.guard_statuses,
                      normalized_packets=node.sent,
                      synthetic_camera={"hz": 20, "declared_source_age_s": .005, "actual_inference": False})
        (args.out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        executor.shutdown(); node.destroy_node(); rclpy.try_shutdown()
        for stream in streams: stream.close()
    print(json.dumps({k: report.get(k) for k in ("case", "passed", "groups", "error", "all_children_exited", "elapsed_seconds")}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
