#!/usr/bin/env python3
"""Isolated real-ROS FC bridge/guard test; no devices or robot access.

The native release identity, stock services and observation feedback are fake.
The production bridge CLI, executor, ROS transports, C++ mux and camera guard
are real. Test-only timer/publication tracing is collected inside the bridge
process; observer receipt timing is kept separate. Native receipt expiry has a
separate memory-interface test and is not inferred from this fake consumer.
Explicit fc_deadline_v1 uses the direct String path without a mux. Its native
services and deadline-aware feedback remain synthetic; the independent real
native chain test establishes the native implementation's expiry behavior.
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
    strict = args.command_contract == "fc_deadline_v1"
    runtime = {"command_topology": "fc_deadline_v1" if strict else "guarded", "manager_pid": -1,
               "manager_start_ticks": -1,
               "files": {"policy": {"path": str(policy), "sha256": digest(policy)}}}
    roles = {"bridge_script": "final_project/behaviors/fc_executor_bridge.py",
             "command_path_evidence": "final_project/behaviors/command_path_evidence.py",
             "fc_integration": "final_project/pupper_fc/integration.py",
             "fc_controller": "final_project/pupper_fc/controller.py",
             "fc_model": "final_project/pupper_fc/model.py",
             "fc_tactics": "final_project/pupper_fc/tactics.py",
             "guard_obstacles": "final_project/pupper_fc/obstacles.py"}
    if strict:
        roles["command_deadline"] = "final_project/behaviors/command_deadline.py"
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
    ticks, publications, writes, injections, envelope_publications = [], [], [], [], []
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
            if strict:
                publisher = node.command_pub
                class RecordingPublisher:
                    def publish(self, message):
                        envelope_publications.append({"at_ns": time.monotonic_ns(),
                                                       "envelope": json.loads(message.data)})
                        return publisher.publish(message)
                node.command_pub = RecordingPublisher()
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
                            "--field-config", str(args.out / "field-config.json"),
                            "--command-contract", args.command_contract,
                            *(["--require-participants", "pupper-1", "pupper-2"] if args.participant_tracks else [])])
    finally:
        (args.out / "bridge-timing.json").write_text(json.dumps({
            "ticks": ticks, "publications": publications, "stdout_writes": writes,
            "original_envelope_publications": envelope_publications,
            "fault_injections": injections,
            "native_identity_stubbed": True, "guard_pid_observed": guard_pid,
            "scope": "Test-only in-process monotonic entry/exit and command publication tracing"}, indent=2) + "\n")


def main():
    if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1":
        raise SystemExit("Only run in the isolated device-free ROS container.")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=("lifecycle", "software_start", "guard_death", "publication_stop",
                                        "domain_lease", "domain_stop", "domain_peer",
                                        "arena_ball", "strict_ball", "closed_loop", "defense_loop", "participant_freshness", "_child"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--mux", default="/mux")
    parser.add_argument("--robot-id", choices=("pupper-1", "pupper-2"), default="pupper-1")
    parser.add_argument("--coord-dir", type=Path)
    parser.add_argument("--participant-tracks", action="store_true")
    parser.add_argument("--command-contract", choices=("legacy_twist", "fc_deadline_v1"),
                        default="legacy_twist")
    args = parser.parse_args()
    strict = args.command_contract == "fc_deadline_v1"
    if strict and args.case not in ("lifecycle", "software_start", "participant_freshness", "publication_stop", "_child"):
        parser.error("strict fixture covers lifecycle, software_start, participant_freshness and publication_stop only")
    if args.case == "software_start" and not strict:
        parser.error("software_start fixture requires fc_deadline_v1")
    shared_tracks = args.participant_tracks or args.case == "participant_freshness"
    args.out.mkdir(parents=True, exist_ok=True)
    policy = Path(os.environ["PUPPER_TEST_STOCK_POLICY"]).resolve(strict=True)
    if args.case == "_child":
        return child(args, policy)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import ListControllers
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterDescriptor
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Empty, Float32MultiArray, String
    from sensor_msgs.msg import Joy
    if strict:
        from final_project.behaviors import command_deadline as deadline_protocol
    rclpy.init()
    executor = SingleThreadedExecutor()
    processes, streams, events = [], [], []
    reader_enabled = threading.Event(); reader_enabled.set()
    report = {"case": args.case, "robot_id": args.robot_id,
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
              "required_participants": ["pupper-1", "pupper-2"] if shared_tracks else [],
              "command_contract": args.command_contract,
              "passed": False, "groups": [], "hardware": False,
              "native_identity_stubbed": True, "native_watchdog_retested": False}
    began = time.monotonic()

    class Fixture(Node):
        def __init__(self):
            # The strict graph requires the selected native receiver's name.
            # Disable builtin parameter services before creating our explicit
            # synthetic GetParameters endpoint at that same name.
            super().__init__("neural_controller_default" if strict else "fc_executor_smoke_fixture",
                             start_parameter_services=not strict)
            self.clock_name = None; self.epoch = "executor-smoke-1"
            self.frame_seq = self.intent_seq = self.camera_seq = self.gateway_seq = 0
            self.gateway_epoch = 1
            self.native_emergency = False
            self.fields_enabled = self.intents_enabled = False
            self.camera_enabled = True; self.near = False; self.camera_label = "tower"
            self.camera_identity = None; self.camera_detections = None; self.camera_packets = []
            self.kind = "move"; self.intent_id = "move-1"; self.pose_x = 0.
            self.frozen_track_captures = {}; self.missing_tracks = set(); self.last_track_captures = {}
            self.pose_y = 0.; self.pose_yaw = 0.; self.target = {"x": 1., "y": 0., "yaw": 0.}
            self.simulator = None; self.simulation_trace = []; self.simulation_events = []
            self.simulation_input = (0., 0., 0.); self.simulation_received_at = None
            self.simulation_receipt_index = 0
            self.simulation_active_input = (0., 0., 0.); self.simulation_active_receipt = None
            self.simulation_steps = 0; self.simulation_max_lag = 0.
            self.simulation_ball_origin = None; self.simulation_ball_max_displacement = 0.
            self.simulation_min_ball_distance = math.inf
            if args.case in ("closed_loop", "defense_loop"):
                from final_project.behaviors.fc_executor_bridge import LIMITS, load_fields
                from final_project.pupper_fc.simulation import Simulator
                fields = load_fields({"field": {"length": 3.66, "width": 2.44, "goal_width": .6096,
                    "robot_radius": .16, "ball_radius": .11, "bumper_offset": .19},
                    "attack_signs": {"pupper-1": 1, "pupper-2": -1}})
                self.simulator = Simulator(field=fields[args.robot_id], limits=LIMITS, scenario="center")
                if args.case == "defense_loop":
                    sign = self.simulator.field.attack_sign
                    self.simulator.move_robot(-sign*.5, 0., 0. if sign == 1 else math.pi)
                    self.simulator.move_ball(-sign*.9, 0.)
                    self.simulation_ball_origin = (self.simulator.ball.x, self.simulator.ball.y)
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
            self.source_envelopes = []; self.admitted_envelopes = []; self.feedback_commands = []
            self.native_envelope = None; self.native_veto = False
            self.native_activation_ns = time.monotonic_ns(); self.native_generation = 1
            self.native_bound_source = ""; self.native_authority_last_ns = None
            self.native_authority_samples = []
            self.native_active = args.case != "software_start"
            self.native_authority_pub = self.native_subscription = None
            self.start_requests = []; self.activation_timer = None
            self.activation_observed_at = None
            self.statuses = []; self.guard_statuses = []; self.sent = []
            self.estops = self.inventory_reads = self.feedback_count = 0
            self.latched = not self.native_active; self.last_capture = self.last_lease_deadline = None
            self.ingress = self.create_publisher(String, "/fc/normalized_events", 1)
            self.camera = self.create_publisher(String, "/fc/obstacle_detections", 1)
            self.gateway = self.create_publisher(String, "/estop_controller/stop_status",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.feedback = self.create_publisher(Float32MultiArray, "/neural_controller_default/observation", 1)
            self.joy = self.create_publisher(Joy, "/joy", 1)
            self.create_service(ListControllers, "/controller_manager/list_controllers", self.inventory)
            self.create_service(GetParameters, "/neural_controller_default/get_parameters", self.parameters)
            if strict:
                for name, value in (("cmd_vel_loaded_contract", "fc_deadline_v1"),
                                    ("cmd_vel_original_deadline_supported", True),
                                    ("cmd_vel_source_clock_id", deadline_protocol.local_clock_id())):
                    self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
                self.create_subscription(String, deadline_protocol.REQUEST_TOPIC, self.record_source_envelope, 1)
                if self.native_active:
                    self.activate_native()
                else:
                    from joy_utils.srv import StartMotion
                    self.create_service(StartMotion, "/estop_controller/start_motion", self.start_motion)
            else:
                self.create_subscription(Twist, "/kick_cmd_vel", lambda m: self.record(self.commands, m), 1)
                self.create_subscription(Twist, "/fc/requested_cmd_vel", lambda m: self.record(self.raw, m), 1)
                self.create_subscription(Twist, "/cmd_vel", self.consume, 1)
            self.create_subscription(String, "/fc/executor_bridge/status", lambda m: self.statuses.append((time.monotonic(), json.loads(m.data))), QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(String, "/fc/body_command_guard/status", lambda m: self.guard_statuses.append((time.monotonic(), json.loads(m.data))), 1)
            self.create_subscription(Empty, "/emergency_stop", self.emergency, 10)
            self.create_subscription(Empty, "/estop_controller/request_stop", self.gateway_stop, 10)
            self.create_timer(.02, self.authority)
            self.create_timer(.05, self.frame)
            self.create_timer(.10, self.intent)
            self.create_timer(.05, self.camera_frame)
            if self.simulator is not None:
                self.create_timer(.01, self.physics)
        def activate_native(self):
            if self.activation_timer is not None:
                self.activation_timer.cancel()
            assert self.native_subscription is None and self.native_authority_pub is None
            self.native_activation_ns = time.monotonic_ns()
            self.activation_observed_at = self.native_activation_ns / 1e9
            self.native_authority_pub = self.create_publisher(String,
                "/neural_controller_default/command_authority",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.native_subscription = self.create_subscription(String,
                deadline_protocol.ADMITTED_TOPIC, self.consume_envelope, 1)
            self.native_active = True
            self.native_emergency = False
            self.latched = False
        def start_motion(self, request, response):
            # Real StartMotion DDS transport; this backend only schedules fake
            # native endpoints, and neither initializes hardware nor runs motors.
            received = time.monotonic()
            self.start_requests.append({"received_at": received, **{
                k: getattr(request, k) for k in ("instance_id", "epoch", "sequence", "selected_controller")}})
            assert len(self.start_requests) == 1 and not self.native_active
            assert request.instance_id == "fake-stock-gateway" and request.epoch == self.gateway_epoch
            assert request.selected_controller == "neural_controller_default"
            assert 0 < request.sequence <= self.gateway_seq
            # Endpoint discovery deliberately follows the acknowledgement.
            self.activation_timer = self.create_timer(.12, self.activate_native)
            response.success = True
            response.message = "Synthetic backend accepted one Start; activation endpoints follow"
            return response
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
        def record_source_envelope(self, message):
            command = deadline_protocol.decode(message.data)
            now = time.monotonic()
            self.source_envelopes.append({"received_at": now, "envelope": json.loads(message.data)})
            self.commands.append((now, (command.vx, command.vy, command.wz)))
        def consume_envelope(self, message):
            command = deadline_protocol.decode(message.data)
            now_ns = time.monotonic_ns()
            self.admitted_envelopes.append({"received_at": time.monotonic(),
                                            "envelope": json.loads(message.data)})
            binding_changed = False
            if (command.clock_id != deadline_protocol.local_clock_id() or
                    command.issued_mono_ns < self.native_activation_ns):
                self.publish_synthetic_authority()
                return
            if not self.native_bound_source:
                if not command.is_zero or not command.fresh(now_ns):
                    self.publish_synthetic_authority()
                    return
                self.native_bound_source = command.source_instance
                binding_changed = True
            if command.source_instance != self.native_bound_source:
                self.publish_synthetic_authority()
                return
            previous = self.native_envelope
            if previous is not None and command.seq < previous.seq:
                return
            if previous is not None and command.seq == previous.seq:
                self.native_veto = self.native_veto or not command.allowed or command.origin != previous.origin
            else:
                self.native_veto = not command.allowed
            self.native_envelope = command
            self.apply_synthetic_deadline()
            self.publish_synthetic_authority(force=binding_changed)
        def publish_synthetic_authority(self, *, force=False):
            now_ns = time.monotonic_ns()
            if not force and self.native_authority_last_ns is not None and now_ns-self.native_authority_last_ns < 100_000_000:
                return
            payload = {"schema": "pupper_command_authority_v1",
                "clock_id": deadline_protocol.local_clock_id(), "generation": self.native_generation,
                "activation_mono_ns": self.native_activation_ns,
                "bound_source_instance": self.native_bound_source, "sampled_mono_ns": now_ns}
            self.native_authority_pub.publish(String(data=json.dumps(payload)))
            self.native_authority_samples.append(payload)
            self.native_authority_last_ns = now_ns
        def apply_synthetic_deadline(self):
            now_ns = time.monotonic_ns()
            command = self.native_envelope
            self.latest = ((command.vx, command.vy, command.wz)
                if command is not None and command.allowed and not self.native_veto and not self.native_emergency and
                command.fresh(now_ns) else (0., 0., 0.))
            # These are explicitly synthetic native input/feedback samples;
            # raw admitted DDS envelopes are retained separately above.
            self.final.append((now_ns / 1e9, self.latest))
            return now_ns
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
                if self.simulation_ball_origin is not None:
                    b, r = self.simulator.ball, self.simulator.robot
                    self.simulation_ball_max_displacement = max(self.simulation_ball_max_displacement,
                        math.hypot(b.x-self.simulation_ball_origin[0], b.y-self.simulation_ball_origin[1]))
                    self.simulation_min_ball_distance = min(self.simulation_min_ball_distance,
                        math.hypot(b.x-r.x, b.y-r.y))
                if self.simulator.goal != previous_goal:
                    self.simulation_events.append({"event": "natural_goal_crossing", "at": now,
                        "simulation_time": self.simulator.time, "goal": self.simulator.goal,
                        "ball_before": before_ball, "ball_after": asdict(self.simulator.ball),
                        "applied_final_cmd_vel": list(self.simulation_active_input),
                        "applied_final_received_at": self.simulation_active_receipt})
        def emergency(self, _):
            # A native emergency does not itself latch the real gateway.
            self.estops += 1; self.native_emergency = True
            self.latest = (0., 0., 0.)
        def gateway_stop(self, _):
            # Synthetic gateway receipt only; actual gateway cancellation is
            # exercised separately against delayed controller-manager replies.
            self.gateway_epoch += 1
            self.latched = True
            if self.activation_timer is not None:
                self.activation_timer.cancel()
        def inventory(self, _, response):
            self.inventory_reads += 1
            row = ControllerState(); row.name = "neural_controller_default"
            row.type = "neural_controller/NeuralController"
            row.state = "active" if self.native_active else "inactive"
            row.claimed_interfaces = ["leg_front_l_1/position"] if self.native_active else []
            response.controller.append(row)
            return response
        def parameters(self, request, response):
            values = {"model_path": str(policy), "max_vx_forward": .49,
                      "max_vx_backward": .3, "speed_cap": .49, "game_buttons": False}
            if strict:
                values.update(cmd_vel_loaded_contract="fc_deadline_v1",
                              cmd_vel_original_deadline_supported=True,
                              cmd_vel_source_clock_id=deadline_protocol.local_clock_id())
            for name in request.names:
                response.values.append(Parameter(name, value=values[name]).get_parameter_value()
                                       if name in values else Parameter(name).get_parameter_value())
            return response
        def authority(self):
            if strict and self.native_active:
                sampled_ns = self.apply_synthetic_deadline()  # Original expiry applies even with silent input.
                self.feedback_commands.append({"at_ns": sampled_ns, "command": self.latest,
                    "original_expires_mono_ns": self.native_envelope.expires_mono_ns if self.native_envelope else None})
            self.gateway_seq += 1
            payload = {"protocol": "pupper_stop_v1", "instance_id": "fake-stock-gateway",
                       "epoch": self.gateway_epoch, "sequence": self.gateway_seq,
                       "software_stop_topic": "/estop_controller/request_stop",
                       "selected_controller": "neural_controller_default", "latched": self.latched,
                       **{key: False for key in ("fault", "stop_required", "in_flight", "unknown_remote_submission", "shutting_down")}}
            self.gateway.publish(String(data=json.dumps(payload)))
            values = [0.] * 960; values[6:9] = self.latest
            if self.native_active:
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
                if shared_tracks:
                    for robot in robots:
                        robots[robot]["captured_at"] = self.frozen_track_captures.get(robot, self.last_capture)
                        self.last_track_captures[robot] = robots[robot]["captured_at"]
                    robots = {r: row for r, row in robots.items() if r not in self.missing_tracks}
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
            if args.case in ("closed_loop", "defense_loop") and time.monotonic()-began >= 60.:
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
        wait(lambda: nonzero(node.commands, at) and nonzero(node.final, at), "nonzero through selected bridge/guard path")
    try:
        (args.out / "field-config.json").write_text(json.dumps({"field": {
            "length": 3.66, "width": 2.44, "goal_width": .6096,
            "robot_radius": .16, "ball_radius": .11, "bumper_offset": .19},
            "attack_signs": {"pupper-1": 1, "pupper-2": -1}}) + "\n")
        if not strict:
            launch("mux", [args.mux, "--ros-args", "-p", "inputs:=[/kick_cmd_vel]", "-p",
                           "timeout_ms:=150", "-r", "/cmd_vel:=/fc/requested_cmd_vel"])
        guard = launch("guard", [sys.executable, "-m", "final_project.behaviors.body_command_guard",
                                 "--command-contract", args.command_contract,
                                 *(["--arena-ball-contact"] if args.case == "arena_ball" else [])])
        env = dict(os.environ, PUPPER_TEST_GUARD_PID=str(guard.pid))
        bridge = launch("bridge", [sys.executable, str(Path(__file__).resolve()), "_child", "--out", str(args.out), "--robot-id", args.robot_id,
                                  "--command-contract", args.command_contract,
                                  *(["--participant-tracks"] if shared_tracks else [])], interactive=True, env=env)
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
        assert next(e["required_participants"] for e in events if e.get("event") == "bridge_ready") == report["required_participants"]
        if args.case == "software_start":
            wait(lambda: node.statuses and node.guard_statuses and node.inventory_reads >= 1,
                 "inactive native inventory and current guard status")
            pump(.12)
            assert not node.native_active and node.native_subscription is None and node.native_authority_pub is None
            assert not nonzero(node.commands, began) and not nonzero(node.final, began)
            assert bridge.poll() is None and node.estops == 0
            assert node.statuses[-1][1]["native_problem"] is not None
            start_at = time.monotonic()
            operator({"command": "start"})
            wait(lambda: any(e.get("event") == "start_result" and e.get("success") for e in events),
                 "correlated software Start acknowledgement")
            assert len(node.start_requests) == 1
            wait(lambda: node.native_active and bool(node.native_bound_source),
                 "activation endpoints and original source-zero binding")
            report["software_start"] = {"requested_at": start_at,
                "activation_at": node.activation_observed_at,
                "endpoint_delay_after_service_receipt_s": node.activation_observed_at-node.start_requests[0]["received_at"],
                "bound_source_instance": node.native_bound_source,
                "start_requests": list(node.start_requests), "emergency_stops_before_arm": node.estops,
                "scope": "Real software Start client/service transport; fake gateway backend and native activation"}
            assert bridge.poll() is None and node.estops == 0
            assert not nonzero(node.commands, began) and not nonzero(node.final, began)
            report["groups"].append("one_software_Start_from_inactive_native_binds_without_emergency_Stop")
        wait(lambda: status_matches(lambda s: s.get("native_problem") is None and s.get("guard_problem") is None), "bridge authority and camera readiness")
        pump(.12); assert not nonzero(node.commands, began)
        if strict:
            assert node.source_envelopes and not nonzero(node.final, began)
        report["groups"].append("explicit_arm_required")
        operator({"command": "arm", "epoch": node.epoch})
        if args.case == "software_start":
            report["software_start"]["arm_accepted_at"] = time.monotonic()
            assert len(node.start_requests) == 1 and node.estops == 0
        # Pause periodic input so the refusal remains visible in the latest
        # ingress result reported on the production ROS status topic.
        at = time.monotonic(); node.intent(force=True, robot_id="pupper-2" if args.robot_id == "pupper-1" else "pupper-1")
        wait(lambda: status_matches(lambda s: (s.get("ingress_result") or {}).get("accepted") is False and
                                    "pinned robot" in s["ingress_result"].get("reason", ""), at), "other robot refusal")
        assert not nonzero(node.commands, at)
        report["groups"].append("other_robot_intent_refused")
        flowing(); report["groups"].append("actual_bridge_guard_deadline_nonzero" if strict else "actual_bridge_mux_guard_nonzero")
        if args.coord_dir:
            args.coord_dir.mkdir(parents=True, exist_ok=True)
            (args.coord_dir / ("ready-" + args.robot_id)).write_text(str(time.monotonic()) + "\n")
            wait(lambda: all((args.coord_dir / ("ready-"+r)).exists() for r in ("pupper-1", "pupper-2")),
                 "both ROS domains passing commands", 8.)
            report["orientation"] = {"yaw": node.pose_yaw, "pose": [node.pose_x, node.pose_y],
                "field_target": node.target, "attack_sign": 1 if args.robot_id == "pupper-1" else -1}
            if args.case != "domain_peer":
                (args.coord_dir / "fault-start.json").write_text(json.dumps({"at": time.monotonic(), "case": args.case}) + "\n")
        if args.case == "participant_freshness":
            peer = "pupper-2" if args.robot_id == "pupper-1" else "pupper-1"
            report["participant_freshness"] = {}
            for label, robot in (("peer_stale", peer), ("own_stale", args.robot_id)):
                frozen = node.last_track_captures[robot]
                node.frozen_track_captures[robot] = frozen
                deadline = frozen + .35
                wait(lambda: status_matches(lambda s: (s.get("decision") or {}).get("phase") == "participant_hold"
                                            and s.get("participant_capture_age_s", {}).get(robot, 0.) >= .35,
                                            frozen), "individual track expiry: " + label)
                wait(lambda: zero_after(node.final, deadline), "track expiry reaches final output")
                pump(.12)
                assert not nonzero(node.final, time.monotonic()-.08)
                stale_status = next(s for at, s in reversed(node.statuses)
                                    if (s.get("decision") or {}).get("phase") == "participant_hold")
                assert stale_status["field_capture_age_s"] < .12
                if robot == peer:
                    assert stale_status["capture_age_s"] < .12
                first_zero = next(at for at, v in node.final if at >= deadline and all(abs(x) < 1e-6 for x in v))
                assert first_zero - deadline < .15, "fresh envelopes prolonged expired track motion"
                report["participant_freshness"][label] = {"robot": robot,
                    "original_capture": frozen, "deadline": deadline, "first_final_zero": first_zero,
                    "zero_delay_s": first_zero-deadline, "status": stale_status}
                node.frozen_track_captures.pop(robot)
                recovered = time.monotonic()
                wait(lambda: nonzero(node.final, recovered), "fresh original track permits current leased target")
            at = time.monotonic(); node.missing_tracks.add(peer)
            wait(lambda: status_matches(lambda s: (s.get("decision") or {}).get("phase") == "participant_hold"
                                         and "missing" in s["decision"]["reason"], at), "missing participant hold")
            wait(lambda: zero_after(node.final, at), "missing participant zeros final command")
            pump(.1); assert not nonzero(node.final, time.monotonic()-.07)
            report["participant_freshness"]["missing_peer"] = {"robot": peer, "held": True}
            report["groups"].append("own_and_peer_original_capture_expiry_and_missing_peer_hold_despite_fresh_envelopes")
            bridge.stdin.write('{"command":"stop","reason":"participant freshness cleanup"}\n'); bridge.stdin.flush()
            wait(lambda: bridge.poll() is not None, "participant cleanup")
            assert bridge.returncode == 3
        elif args.case == "defense_loop":
            wait(lambda: status_matches(lambda s: (s.get("decision") or {}).get("phase") == "target_reached"),
                 "defender reaches guard around stationary ball", 20.)
            reached_at = time.monotonic(); pump(.15)
            zero_start = time.monotonic(); pump(.5); zero_end = time.monotonic()
            samples = [value for at, value in node.final if zero_start <= at <= zero_end]
            assert len(samples) >= 10 and all(all(abs(x) < 1e-6 for x in v) for v in samples)
            assert node.simulation_ball_max_displacement <= 1e-9, "defensive transit moved the ball"
            assert node.simulator.goal == 0
            field = node.simulator.field
            guard = (-field.attack_sign*(field.length/2-.35), 0.)
            assert math.hypot(node.simulator.robot.x-guard[0], node.simulator.robot.y-guard[1]) <= .06
            decisions = [s.get("decision") or {} for _, s in node.statuses]
            assert any(d.get("phase") == "defend" and d.get("role") == "defense" for d in decisions)
            assert any(d.get("phase") == "target_reached" and d.get("role") == "defense" for d in decisions)
            report["defense_loop"] = {"attack_sign": field.attack_sign, "field": asdict(field),
                "limits": asdict(node.simulator.limits), "physics_step_s": .01,
                "simulation_clock_origin_monotonic": node.simulation_origin,
                "physics_steps": node.simulation_steps, "simulated_time": node.simulator.time,
                "real_elapsed_since_simulation_origin": time.monotonic()-node.simulation_origin,
                "max_physics_catchup_lag_s": node.simulation_max_lag,
                "command_source": "Only actual final /cmd_vel DDS callback receipts, applied no earlier than next10ms substep",
                "simulator_receipt_expiry_s": .25, "simulated_receipt_expiry_is_native_evidence": False,
                "ball_max_displacement_m": node.simulation_ball_max_displacement,
                "minimum_center_to_ball_distance_m": node.simulation_min_ball_distance,
                "guard": list(guard), "final_snapshot": node.simulator.snapshot(),
                "reached_at": reached_at, "sustained_zero": {"from": zero_start, "through": zero_end, "samples": len(samples)},
                "physical_or_opponent_collision_acceptance": False}
            report["groups"].append("actual_final_cmd_vel_reaches_guard_without_ball_contact_then_sustained_zero")
            bridge.stdin.write('{"command":"stop","reason":"defense-loop smoke cleanup"}\n'); bridge.stdin.flush()
            wait(lambda: bridge.poll() is not None, "defense-loop cleanup")
            assert bridge.returncode == 3
        elif args.case == "closed_loop":
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
        elif args.case == "software_start":
            pump(.2)
            assert len(node.start_requests) == 1 and node.estops == 0
            assert sum(e.get("event") == "operator_result" and e.get("command") == "arm" and
                       e.get("accepted") is True for e in events) == 1
            report["software_start"].update(start_request_count_before_cleanup=1,
                accepted_arm_count=1, emergency_stops_before_cleanup=node.estops,
                matching_native_binding_before_arm=True)
            at = time.monotonic()
            bridge.stdin.write('{"command":"stop","reason":"software Start fixture own cleanup"}\n'); bridge.stdin.flush()
            wait(lambda: bridge.poll() is not None, "software Start own cleanup")
            assert bridge.returncode == 3 and node.estops == 1 and zero_after(node.commands, at)
            report["groups"].append("one_explicit_arm_then_motion_then_one_deliberate_cleanup_Stop")
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
        if strict:
            original = timing["original_envelope_publications"]
            assert original
            by_seq = {row["envelope"]["seq"]: row["envelope"] for row in original}
            assert len(by_seq) == len(original), "source sequence repeated"
            for row in node.admitted_envelopes:
                accepted = row["envelope"]
                source = by_seq[accepted["seq"]]
                assert {k:v for k,v in accepted.items() if k != "allowed"} == {
                    k:v for k,v in source.items() if k != "allowed"}, "guard changed original command"
            for row in node.feedback_commands:
                deadline = row["original_expires_mono_ns"]
                if deadline is not None and row["at_ns"] >= deadline:
                    assert all(value == 0. for value in row["command"]), "synthetic feedback extended source expiry"
            report["deadline_transport"] = {"original_publications": len(original),
                "observed_admitted_envelopes": len(node.admitted_envelopes),
                "immutable_origins_verified": True, "synthetic_feedback_respects_original_expiry": True,
                "scope": "Real bridge/guard String transport; deadline feedback and native services are synthetic"}
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
                      source_envelopes=node.source_envelopes, admitted_envelopes=node.admitted_envelopes,
                      synthetic_deadline_feedback=node.feedback_commands,
                      synthetic_native_authority=node.native_authority_samples,
                      synthetic_start_requests=node.start_requests,
                      final_output_semantics=("synthetic native applied feedback; raw admitted DDS envelopes recorded separately"
                                              if strict else "final Twist DDS observer receipts"),
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
