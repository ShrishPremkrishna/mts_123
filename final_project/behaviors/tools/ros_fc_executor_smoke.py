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
import math
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
                            "--robot-id", args.robot_id, "--producer-id", "fixture-" + args.robot_id,
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
    parser.add_argument("case", choices=("lifecycle", "guard_death", "publication_stop",
                                        "domain_lease", "domain_stop", "domain_peer",
                                        "arena_ball", "strict_ball", "closed_loop", "_child"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--mux", default="/mux")
    parser.add_argument("--robot-id", choices=("pupper-1", "pupper-2"), default="pupper-1")
    parser.add_argument("--coord-dir", type=Path)
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
    report = {"case": args.case, "robot_id": args.robot_id,
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
              "passed": False, "groups": [], "hardware": False,
              "native_identity_stubbed": True, "native_watchdog_retested": False}
    began = time.monotonic()

    class Fixture(Node):
        def __init__(self):
            super().__init__("fc_executor_smoke_fixture")
            self.clock_name = None; self.epoch = "executor-smoke-1"
            self.frame_seq = self.intent_seq = self.camera_seq = self.gateway_seq = 0
            self.fields_enabled = self.intents_enabled = False
            self.camera_enabled = True; self.near = False; self.camera_label = "tower"
            self.camera_identity = None; self.camera_detections = None; self.camera_packets = []
            self.kind = "move"; self.intent_id = "move-1"; self.pose_x = 0.
            self.pose_y = 0.; self.pose_yaw = 0.; self.target = {"x": 1., "y": 0., "yaw": 0.}
            self.simulator = None; self.simulation_trace = []; self.simulation_events = []
            self.simulation_input = (0., 0., 0.); self.simulation_received_at = None
            self.simulation_receipt_index = 0
            self.simulation_active_input = (0., 0., 0.); self.simulation_active_receipt = None
            self.simulation_steps = 0; self.simulation_max_lag = 0.
            if args.case == "closed_loop":
                from final_project.behaviors.fc_executor_bridge import LIMITS, load_fields
                from final_project.pupper_fc.simulation import Simulator
                fields = load_fields({"field": {"length": 3.66, "width": 2.44, "goal_width": .6096,
                    "robot_radius": .16, "ball_radius": .11, "bumper_offset": .19},
                    "attack_signs": {"pupper-1": 1, "pupper-2": -1}})
                self.simulator = Simulator(field=fields[args.robot_id], limits=LIMITS, scenario="center")
                self.simulation_origin = time.monotonic()
                self.kind = "auto"; self.intent_id = "auto-1"
                self.simulation_events.append({"event": "initial", "at": self.simulation_origin,
                                               "snapshot": self.simulator.snapshot()})
            if args.case in ("arena_ball", "strict_ball"):
                from final_project.behaviors.body_command_guard import ARENA_DETECTOR_IDENTITY
                self.camera_identity = dict(ARENA_DETECTOR_IDENTITY)
                self.kind = "shoot"; self.intent_id = "approach-1"; self.pose_x = -.9
                if args.case == "arena_ball":
                    self.camera_detections = [{"box": [.4, .3, .6, .85], "confidence": .9, "class_id": "32"}]
            if args.coord_dir:
                if args.robot_id == "pupper-1":
                    self.pose_x, self.pose_y = -.7, -.2
                    self.target = {"x": .6, "y": -.2, "yaw": 0.}
                else:
                    self.pose_x, self.pose_y, self.pose_yaw = .7, .2, math.pi
                    self.target = {"x": -.6, "y": .6, "yaw": math.pi}
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
            if self.simulator is not None:
                self.create_timer(.01, self.physics)
        def record(self, rows, message):
            value = (message.linear.x, message.linear.y, message.angular.z)
            rows.append((time.monotonic(), value))
            return value
        def consume(self, message):
            if self.simulator is not None:
                self.physics()  # Previous received command owns elapsed time.
            self.latest = self.record(self.final, message)
            if self.simulator is not None:
                self.simulation_input = self.latest
                self.simulation_received_at = self.final[-1][0]
        def physics(self):
            if self.simulator is None: return
            from final_project.pupper_fc.model import Command
            now = time.monotonic(); elapsed = now - self.simulation_origin
            lag = elapsed - self.simulator.time
            self.simulation_max_lag = max(self.simulation_max_lag, lag)
            # Fixed10ms steps never advance beyond elapsed wall time. Command
            # numbers come ONLY from final /cmd_vel DDS receipts. The receipt
            # TTL is a simulator boundary rule, not native-watchdog evidence.
            while self.simulator.time + .01 <= elapsed:
                step_started = self.simulation_origin + self.simulator.time
                while (self.simulation_receipt_index < len(self.final) and
                       self.final[self.simulation_receipt_index][0] <= step_started):
                    self.simulation_active_receipt, self.simulation_active_input = self.final[self.simulation_receipt_index]
                    self.simulation_receipt_index += 1
                expires = (self.simulation_active_receipt - self.simulation_origin + .25
                           if self.simulation_active_receipt is not None else 0.)
                command = Command(*self.simulation_active_input, expires_at=expires,
                                  robot_id="pupper-1")
                previous_goal = self.simulator.goal
                before_ball = asdict(self.simulator.ball)
                self.simulator.step(command, .01); self.simulation_steps += 1
                if self.simulator.goal != previous_goal:
                    self.simulation_events.append({"event": "natural_goal_crossing", "at": now,
                        "simulation_time": self.simulator.time, "goal": self.simulator.goal,
                        "ball_before": before_ball, "ball_after": asdict(self.simulator.ball),
                        "applied_final_cmd_vel": list(self.simulation_active_input),
                        "applied_final_received_at": self.simulation_active_receipt})
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
            payload = {"producer_id": "fixture-" + args.robot_id, "clock": self.clock_name,
                       "epoch": self.epoch, "now": -999., **payload}
            self.sent.append({"at": time.monotonic(), "packet": payload})
            self.ingress.publish(String(data=json.dumps(payload)))
        def frame(self):
            if self.fields_enabled and self.clock_name:
                self.frame_seq += 1; now = time.monotonic(); self.last_capture = now
                robots = {"pupper-1": {"x": -.7, "y": -.2, "yaw": 0.},
                          "pupper-2": {"x": .7, "y": .2, "yaw": math.pi}}
                robots[args.robot_id] = {"x": self.pose_x, "y": self.pose_y, "yaw": self.pose_yaw}
                ball = {"x": 0., "y": 0.}
                if self.simulator is not None:
                    self.physics(); observation = self.simulator.observe()
                    self.last_capture = self.simulation_origin + observation.captured_at
                    other = "pupper-2" if args.robot_id == "pupper-1" else "pupper-1"
                    sign = self.simulator.field.attack_sign
                    robots = {other: {"x": sign, "y": .8, "yaw": math.pi if sign == 1 else 0.},
                              args.robot_id: {**asdict(observation.robot), "goal": observation.goal,
                                              "fallen": observation.fallen}}
                    ball = asdict(observation.ball) if observation.ball else None
                    self.simulation_trace.append({"at": now, "captured_at": self.last_capture,
                        "frame_seq": self.frame_seq, "snapshot": self.simulator.snapshot(),
                        "received_final_command": list(self.simulation_input),
                        "received_final_at": self.simulation_received_at,
                        "applied_final_command": list(self.simulation_active_input),
                        "applied_final_at": self.simulation_active_receipt})
                self.send({"event": "frame", "seq": self.frame_seq, "captured_at": self.last_capture,
                           "robots": robots,
                           "ball": ball,
                           "goals": {"left": {"x": -1.83, "y": 0.}, "right": {"x": 1.83, "y": 0.}}})
        def intent(self, *, force=False, robot_id=None):
            if (self.intents_enabled or force) and self.clock_name:
                self.intent_seq += 1; now = time.monotonic(); self.last_lease_deadline = now + .45
                packet = {"event": "intent", "seq": self.intent_seq, "issued_at": now,
                          "valid_until": self.last_lease_deadline, "robot_id": robot_id or args.robot_id,
                          "intent_id": self.intent_id, "kind": self.kind}
                if self.kind == "move": packet["target"] = self.target
                elif self.kind == "shoot": packet["target"] = {"x": 1.83, "y": 0.}
                self.send(packet)
        def camera_frame(self):
            if not self.camera_enabled: return
            self.camera_seq += 1
            packet = {"schema": "fc_camera_obstacles_v1",
                "producer_instance": "executor-smoke-camera", "sequence": self.camera_seq,
                "captured_at_ros_ns": self.get_clock().now().nanoseconds - 5_000_000,
                "valid": True, "healthy": True, "format": "normalized_xyxy_v1",
                "image_width": 700, "image_height": 572, "horizontal_fov_deg": 220,
                "detections": self.camera_detections if self.camera_detections is not None else (
                    [{"box": [.4, .3, .6, .85], "confidence": .9, "class_id": self.camera_label}] if self.near else [])}
            if self.camera_identity is not None:
                packet["detector_identity"] = self.camera_identity
            if args.case in ("arena_ball", "strict_ball"):
                self.camera_packets.append({"at": time.monotonic(), "packet": packet})
            self.camera.publish(String(data=json.dumps(packet)))

    node = Fixture(); executor.add_node(node)
    def pump(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if args.case == "closed_loop" and time.monotonic()-began >= 60.:
                raise TimeoutError("closed-loop case exceeded60seconds")
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
        guard = launch("guard", [sys.executable, "-m", "final_project.behaviors.body_command_guard",
                                 *(["--arena-ball-contact"] if args.case == "arena_ball" else [])])
        env = dict(os.environ, PUPPER_TEST_GUARD_PID=str(guard.pid))
        bridge = launch("bridge", [sys.executable, str(Path(__file__).resolve()), "_child", "--out", str(args.out), "--robot-id", args.robot_id], interactive=True, env=env)
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
        at = time.monotonic(); node.intent(force=True, robot_id="pupper-2" if args.robot_id == "pupper-1" else "pupper-1")
        wait(lambda: status_matches(lambda s: (s.get("ingress_result") or {}).get("accepted") is False and
                                    "pinned robot" in s["ingress_result"].get("reason", ""), at), "other robot refusal")
        assert not nonzero(node.commands, at)
        report["groups"].append("other_robot_intent_refused")
        flowing(); report["groups"].append("actual_bridge_mux_guard_nonzero")
        if args.coord_dir:
            args.coord_dir.mkdir(parents=True, exist_ok=True)
            (args.coord_dir / ("ready-" + args.robot_id)).write_text(str(time.monotonic()) + "\n")
            wait(lambda: all((args.coord_dir / ("ready-"+r)).exists() for r in ("pupper-1", "pupper-2")),
                 "both ROS domains passing commands", 8.)
            report["orientation"] = {"yaw": node.pose_yaw, "pose": [node.pose_x, node.pose_y],
                "field_target": node.target, "attack_sign": 1 if args.robot_id == "pupper-1" else -1}
            if args.case != "domain_peer":
                (args.coord_dir / "fault-start.json").write_text(json.dumps({"at": time.monotonic(), "case": args.case}) + "\n")
        if args.case == "closed_loop":
            wait(lambda: node.simulator.goal != 0, "simulator generates a natural goal", 40.)
            assert node.simulator.goal == 1, "simulator scored an own goal"
            natural_goal = next(e for e in node.simulation_events if e["event"] == "natural_goal_crossing")
            field = node.simulator.field
            assert abs(natural_goal["ball_after"]["x"]) >= field.length/2 + field.ball_radius - 1e-9
            assert abs(natural_goal["ball_after"]["y"]) + field.ball_radius <= field.goal_width/2
            assert field.attack_sign * natural_goal["ball_after"]["x"] > 0
            wait(lambda: status_matches(lambda s: (s.get("decision") or {}).get("phase") == "await_reset"),
                 "scoring robot returns home and waits", 30.)
            await_at = time.monotonic(); pump(.1); zero_start = time.monotonic(); pump(.5)
            zero_end = time.monotonic()
            samples = [value for at, value in node.final if zero_start <= at <= zero_end]
            assert len(samples) >= 10 and all(all(abs(x) < 1e-6 for x in v) for v in samples)
            assert not nonzero(node.commands, zero_start)
            assert math.hypot(node.simulator.velocity.vx, node.simulator.velocity.vy) < .02
            report["groups"].append("actual_final_cmd_vel_drives_natural_goal_and_return_home_then_sustained_zero")
            node.physics(); restart_at = time.monotonic()
            node.simulator.restart_play()
            node.simulation_events.append({"event": "explicit_simulated_manual_restart_play", "at": restart_at,
                                           "snapshot": node.simulator.snapshot()})
            wait(lambda: node.last_capture is not None and node.last_capture > restart_at,
                 "newer post-replacement field frame")
            # Keep20Hz frames streaming: the human cue must not race a moving
            # sequence number or require stopping the perception producer.
            wait(lambda: status_matches(lambda s: (s.get("resume") or {}).get("available") is True,
                                        restart_at), "descriptive resume readiness after fresh kickoff frame")
            resume_ready = next(row for at, row in reversed(node.statuses)
                                if at >= restart_at and (row.get("resume") or {}).get("available") is True)
            resume_at = time.monotonic()
            assert not nonzero(node.commands, zero_start), "ball replacement resumed motion without a cue"
            resume_result = operator({"command": "resume"})
            wait(lambda: status_matches(lambda s: (s.get("decision") or {}).get("phase") == "strike", resume_at),
                 "one human cue permits next strike", 30.)
            second_strike_at = time.monotonic()
            assert nonzero(node.final, resume_at)
            phases = {}
            for at, status in node.statuses:
                phase = (status.get("decision") or {}).get("phase")
                if phase is not None: phases.setdefault(phase, at)
            assert {"approach", "align", "strike", "return_home", "await_reset"} <= phases.keys(), phases
            assert phases["approach"] < phases["align"] < phases["strike"] < phases["return_home"] < phases["await_reset"]
            assert len([e for e in events if e.get("event") == "operator_result" and e.get("command") == "resume"]) == 1
            report["closed_loop"] = {"scenario": "center", "attack_sign": field.attack_sign,
                "field": asdict(field), "limits": asdict(node.simulator.limits),
                "physics_step_s": .01, "simulation_clock_origin_monotonic": node.simulation_origin,
                "physics_steps": node.simulation_steps, "simulated_time": node.simulator.time,
                "real_elapsed_since_simulation_origin": time.monotonic()-node.simulation_origin,
                "max_physics_catchup_lag_s": node.simulation_max_lag,
                "command_source": "Only actual final /cmd_vel DDS callback receipts, applied no earlier than next10ms substep",
                "simulator_receipt_expiry_s": .25, "simulated_receipt_expiry_is_native_evidence": False,
                "simulator_internal_robot_id": "pupper-1", "selected_bridge_robot_id": args.robot_id,
                "shared_ball_two_robot_simulation": False,
                "first_phase_observer_times": phases, "natural_goal": natural_goal,
                "await_reset_observed_at": await_at, "sustained_zero": {"from": zero_start, "through": zero_end, "samples": len(samples)},
                "manual_restart_at": restart_at, "resume_cue_at": resume_at, "resume_result": resume_result,
                "descriptive_resume_readiness": {k: resume_ready.get(k) for k in
                    ("resume", "frame_seq", "capture_age_s", "intent", "sampled_at_monotonic")},
                "second_strike_observed_at": second_strike_at,
                "auto_intent_id": node.intent_id, "scripted_goal_or_ball_nudge": False}
            report["groups"].append("one_human_resume_after_manual_ball_replacement_and_fresh_frame_reaches_next_strike")
            bridge.stdin.write('{"command":"stop","reason":"closed-loop smoke cleanup"}\n'); bridge.stdin.flush()
            wait(lambda: bridge.poll() is not None, "closed-loop cleanup")
            assert bridge.returncode == 3
        elif args.case in ("arena_ball", "strict_ball"):
            from final_project.behaviors.body_command_guard import ARENA_DETECTOR_IDENTITY
            ball = {"box": [.4, .3, .6, .85], "confidence": .9, "class_id": "32"}
            person = {**ball, "class_id": "0"}
            def guard_status(test, since):
                return any(at >= since and test(row) for at, row in node.guard_statuses)
            def held(reason, since):
                wait(lambda: guard_status(lambda s: not s["camera_permitted"] and reason in s["reason"], since),
                     "camera hold: " + reason)
                wait(lambda: status_matches(lambda s: (s.get("decision") or {}).get("phase") == "external_hold", since),
                     "bridge observes camera hold")
                wait(lambda: zero_after(node.commands, since) and zero_after(node.final, since), "camera hold zeros chain")
                pump(.08)
                assert not nonzero(node.final, time.monotonic()-.05)
            def cleared():
                at = time.monotonic()
                wait(lambda: guard_status(lambda s: s["camera_permitted"], at), "fresh camera clearance")
                # The bridge receives guard status at 5 Hz. Wait for its own
                # release observation before issuing post-release evidence.
                wait(lambda: status_matches(lambda s: s.get("guard_problem") is None and
                     (s.get("decision") or {}).get("phase") != "external_hold", at), "bridge hold releases")
                pump(.08)
            if args.case == "strict_ball":
                at = time.monotonic(); node.camera_detections = [ball]
                held("(32)", at)
                assert guard_status(lambda s: s["contact_policy"] == "all_classes" and s["ignored_ball_count"] == 0, at)
                report["groups"].append("strict_default_verified_ball_still_holds_and_zeros")
            else:
                wait(lambda: status_matches(lambda s: (s.get("decision") or {}).get("phase") == "approach"),
                     "verified ball permits pre-strike approach")
                assert guard_status(lambda s: s["contact_policy"] == "arena_ball_contact" and s["ignored_ball_count"] == 1,
                                    began)
                assert not status_matches(lambda s: (s.get("decision") or {}).get("phase") == "strike")
                report["groups"].append("verified_ball_earns_clearance_and_approach_before_strike")
                at = time.monotonic(); node.camera_detections = [ball, person]
                held("(0)", at)
                node.fields_enabled = node.intents_enabled = False
                node.camera_detections = [ball]; at = time.monotonic(); cleared()
                assert not nonzero(node.commands, at+.05)
                node.fields_enabled = True; node.frame(); pump(.08)
                assert not nonzero(node.commands, at+.05), "fresh frame alone revived interrupted shot"
                node.intent(force=True)
                wait(lambda: status_matches(lambda s: "new intent_id" in
                     (s.get("ingress_result") or {}).get("reason", ""), at), "interrupted shot ID refused")
                assert not nonzero(node.commands, at+.05)
                node.intent_id = "approach-2"; flowing()
                report["groups"].append("person_with_ball_holds_old_shot_cannot_revive_fresh_id_and_frame_replan")
                invalids = [("missing_identity", None, [ball], "detector identity"),
                            ("wrong_model", {**ARENA_DETECTOR_IDENTITY, "model_sha256": "0"*64}, [ball], "detector identity"),
                            ("malformed_ball", dict(ARENA_DETECTOR_IDENTITY), [{**ball, "box": [.4, .3, .6, 1.1]}], "box")]
                for number, (label, identity, detections, reason) in enumerate(invalids, 3):
                    at = time.monotonic(); node.camera_identity = identity; node.camera_detections = detections
                    held(reason, at)
                    assert guard_status(lambda s: s["camera_state"] == "invalid" and s["ignored_ball_count"] == 0, at)
                    node.fields_enabled = node.intents_enabled = False
                    node.camera_identity = dict(ARENA_DETECTOR_IDENTITY); node.camera_detections = [ball]
                    cleared()
                    node.intent_id = "approach-" + str(number); flowing()
                    report["groups"].append(label + "_fails_closed_then_fresh_evidence_recovers")
            bridge.stdin.write('{"command":"stop","reason":"ball policy smoke cleanup"}\n'); bridge.stdin.flush()
            wait(lambda: bridge.poll() is not None, "ball fixture cleanup")
            assert bridge.returncode == 3
        elif args.case == "guard_death":
            at = time.monotonic(); count = node.estops; feedback = node.feedback_count
            guard.kill(); guard.wait(timeout=3)
            wait(lambda: bridge.poll() is not None, "bridge closes on guard loss")
            pump(.1)
            assert bridge.returncode == 0 and node.estops == count and node.feedback_count > feedback
            assert zero_after(node.commands, at) and zero_after(node.final, at)
            report["groups"].append("guard_death_closes_bridge_with_zero_and_retains_fake_native_authority")
        elif args.case == "domain_lease":
            node.intents_enabled = False; deadline = node.last_lease_deadline
            wait(lambda: zero_after(node.commands, deadline) and zero_after(node.final, deadline), "own lease expires", .9)
            pump(.3)
            assert not nonzero(node.commands, deadline+.08) and not nonzero(node.final, deadline+.08)
            report["domain_fault"] = {"lease_deadline": deadline, "zero_held_until": time.monotonic(),
                "other_domain_did_not_feed_this_robot": True}
            flowing()
            report["groups"].append("own_lease_loss_zeros_despite_other_domain_motion_then_fresh_renewal_recovers")
        elif args.case == "domain_stop":
            at = time.monotonic()
            bridge.stdin.write('{"command":"stop","reason":"domain isolation Stop"}\n'); bridge.stdin.flush()
            wait(lambda: bridge.poll() is not None, "own Stop closes bridge")
            pump(.2)
            assert bridge.returncode == 3 and node.estops == 1 and zero_after(node.final, at)
            assert not nonzero(node.final, at+.1)
            report["groups"].append("own_Stop_zeros_and_latches_without_other_domain_feed")
        elif args.case == "domain_peer":
            wait(lambda: (args.coord_dir / "fault-done.json").exists(), "other domain completes fault", 8.)
            pump(.3)
            start = json.loads((args.coord_dir / "fault-start.json").read_text())["at"]
            ended = time.monotonic()
            samples = [(at, value) for at, value in node.final if start <= at <= ended]
            # This peer faces -x and commands toward (-.6,+.6). Its body-frame
            # lateral/yaw signs differ from pupper-1's straight +x command.
            assert len(samples) >= 10 and all(value[0] > .2 and value[1] < -.05 and value[2] < -.1
                                              for _, value in samples), samples[-10:]
            assert bridge.poll() is None and node.estops == 0
            gaps = [b[0]-a[0] for a, b in zip(samples, samples[1:])]
            assert max(gaps, default=0.) < .15, "peer command stream paused"
            assert all(row["robot_id"] == args.robot_id for _, row in node.statuses)
            report["peer_isolation"] = {"fault_start": start, "verified_through": ended,
                "nonzero_samples": len(samples), "max_observer_sample_gap_s": max(gaps, default=0.),
                "unexpected_zeros_or_other_command_signatures": 0, "emergency_stops_before_own_cleanup": node.estops,
                "bridge_alive_after_other_fault": True, "sample_signature": samples[-1][1]}
            report["groups"].append("opposite_orientation_peer_keeps_own_stream_and_authority_through_other_domain_fault")
            (args.coord_dir / "peer-verified").write_text(str(ended) + "\n")
            bridge.stdin.write('{"command":"stop","reason":"peer own explicit cleanup"}\n'); bridge.stdin.flush()
            wait(lambda: bridge.poll() is not None, "peer own cleanup")
            assert bridge.returncode == 3
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
        if args.coord_dir and args.case != "domain_peer":
            (args.coord_dir / "fault-done.json").write_text(json.dumps({"at": time.monotonic(), "case": args.case}) + "\n")
            wait(lambda: (args.coord_dir / "peer-verified").exists(), "peer confirms isolation", 5.)
            if bridge.poll() is None:
                bridge.stdin.write('{"command":"stop","reason":"fault fixture own cleanup"}\n'); bridge.stdin.flush()
                wait(lambda: bridge.poll() is not None, "fault fixture cleanup")
        thread.join(timeout=1)
        timing = json.loads((args.out / "bridge-timing.json").read_text())
        ticks, publications = timing["ticks"], timing["publications"]
        spacing = [b["before"]-a["before"] for a, b in zip(ticks, ticks[1:])]
        report["bridge_tick_timing"] = {"samples": len(ticks),
            "max_spacing_s": max(spacing, default=0.),
            "p95_spacing_s": sorted(spacing)[int(.95*(len(spacing)-1))] if spacing else None,
            "max_callback_duration_s": max((row["after"]-row["before"] for row in ticks), default=0.),
            "source": "in-process on_tick tracing, not observer callback intervals"}
        assert all(row["command"]["robot_id"] == args.robot_id for row in publications)
        report["all_publications_address_pinned_robot"] = True
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
                      camera_packets=node.camera_packets,
                      simulation_trace=node.simulation_trace, simulation_events=node.simulation_events,
                      synthetic_camera={"hz": 20, "declared_source_age_s": .005, "actual_inference": False})
        (args.out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        executor.shutdown(); node.destroy_node(); rclpy.try_shutdown()
        for stream in streams: stream.close()
    print(json.dumps({k: report.get(k) for k in ("case", "passed", "groups", "error", "all_children_exited", "elapsed_seconds")}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
