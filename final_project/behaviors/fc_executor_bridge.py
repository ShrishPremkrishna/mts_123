#!/usr/bin/env python3
"""Resident, explicitly armed FC transport for a selected guarded walk release.

The default CLI validates configuration and exits without ROS. Execution needs
an explicit field-config, a runtime receipt, a producer ID and operator input.
ROS String /fc/normalized_events admits only normalized frame/intent packets;
producer_id and clock='linux-monotonic:<this Linux boot UUID>' are operational
bindings, not authentication. Only a same-host normalized fixture is supported.
Incoming now is replaced by local receipt time; capture/issue/expiry never are.
Operator stdin JSON commands: arm (new epoch), resume (optional strict frame_seq),
start (optional gateway activation), status, stop, quit. Start never arms FC.
Resume requires a goal return completed before receipt; a bare cue resolves the
latest fresh admitted frame when processed. Current native/camera/field/intent
readiness is rechecked then. Queued cues expire after 0.2 seconds.
Neither camera clearance nor reconnect arms or renews an action. No homing.
Optional --require-participants pins the robot tracks needed for motion. Each
present required track needs its own original captured_at; stale/missing peers
hold motion. This checks tracking evidence, not geometric collision clearance.
Explicit --command-contract fc_deadline_v1 requires the matching native/guard
topology and carries the original source deadline to native. It emits genuine
zero envelopes while waiting for arm so receiver startup never grants motion.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue, SimpleQueue
import sys
import threading
import time
import uuid

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for path in (ROOT, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from final_project.pupper_fc.integration import Executor, number, token
from final_project.pupper_fc.model import Command, Field, Limits, zero
from command_path_evidence import (GUARD_STATUS_TOPIC, GUARD_STATUS_MAX_AGE_S,
    NATIVE_AUTHORITY_TOPIC, decode_guard_status, decode_native_authority,
    guard_publishers_problem, guard_status_problem, native_authority_problem)

TICK_S = 0.02
IDENTITY_S = 0.2
OBS_MAX_AGE_S = 0.2
NATIVE_DISCOVERY_S = 0.3
ROBOT_IDS = ("pupper-1", "pupper-2")
LIMITS = Limits(vx=0.3, vy=0.3, wz=1.0)
EVENT_TOPIC = "/fc/normalized_events"
STATUS_TOPIC = "/fc/executor_bridge/status"
NODE_NAME = "fc_executor_bridge"
FIELD_KEYS = {"length", "width", "goal_width", "robot_radius", "ball_radius", "bumper_offset"}
LEGACY_CONTRACT = "legacy_twist"
DEADLINE_CONTRACT = "fc_deadline_v1"


class EventSink:
    """Bounded notifications; an absent stdout reader cannot block control."""
    def __init__(self, writer=None, capacity=64):
        self.writer = writer or (lambda row: print(json.dumps(row, allow_nan=False), flush=True))
        self.queue, self.dropped = Queue(maxsize=capacity), 0
        threading.Thread(target=self._write, daemon=True).start()

    def __call__(self, row):
        try:
            self.queue.put_nowait(row)
        except Full:
            self.dropped += 1

    def _write(self):
        while True:
            row = self.queue.get()
            try:
                self.writer(row)
            except (OSError, ValueError):
                pass  # notifications never grant motion authority


def verify_bridge_sources(runtime):
    """Bind executing imports to explicit selected source roles before ROS."""
    import command_path_evidence
    from final_project.pupper_fc import integration, controller, model, tactics, obstacles
    modules = {"bridge_script": __file__, "command_path_evidence": command_path_evidence.__file__,
               "fc_integration": integration.__file__, "fc_controller": controller.__file__,
               "fc_model": model.__file__, "fc_tactics": tactics.__file__,
               "guard_obstacles": obstacles.__file__}
    if runtime.get("command_topology") == DEADLINE_CONTRACT:
        from final_project.behaviors import command_deadline
        modules["command_deadline"] = command_deadline.__file__
    for role, origin in modules.items():
        selected = runtime.get("files", {}).get(role)
        path = Path(origin).resolve(strict=True)
        if (not selected or str(path) != selected.get("path") or
                hashlib.sha256(path.read_bytes()).hexdigest() != selected.get("sha256")):
            raise ValueError("executing source differs from selected role: " + role)


def load_fields(config):
    if not isinstance(config, dict) or set(config) != {"field", "attack_signs"}:
        raise ValueError("field config requires field and attack_signs")
    raw, signs = config["field"], config["attack_signs"]
    if not isinstance(raw, dict) or set(raw) != FIELD_KEYS:
        raise ValueError("all six calibrated field geometry values must be explicit")
    values = {key: number(raw[key], key) for key in FIELD_KEYS}
    if any(value <= 0 for value in values.values()):
        raise ValueError("field geometry must be positive")
    if (values["goal_width"] > values["width"] or
            any(values[key] >= min(values["length"], values["width"]) / 2
                for key in ("robot_radius", "ball_radius", "bumper_offset"))):
        raise ValueError("field geometry is inconsistent")
    if (not isinstance(signs, dict) or set(signs) != set(ROBOT_IDS) or
            any(type(v) is not int or v not in (-1, 1) for v in signs.values()) or
            signs[ROBOT_IDS[0]] == signs[ROBOT_IDS[1]]):
        raise ValueError("both robot attack signs must be explicit and opposite")
    return {robot: Field(**values, attack_sign=signs[robot]) for robot in ROBOT_IDS}


class DeadlineEncoder:
    """One process-owned source identity and sequence across every FC arm epoch."""

    def __init__(self, *, monotonic_ns=time.monotonic_ns, clock_id=None, source_instance=None):
        from final_project.behaviors import command_deadline
        self.protocol = command_deadline
        self.clock_id = command_deadline.local_clock_id() if clock_id is None else clock_id
        self.source_instance = uuid.uuid4().hex if source_instance is None else source_instance
        self.monotonic_ns = monotonic_ns
        self.sequence = self.last_issued_ns = 0
        # Fail startup on invalid local identity rather than discovering that
        # even a fallback zero cannot be encoded at the publication boundary.
        command_deadline.DeadlineCommand(self.clock_id, self.source_instance, 1, 1, 2,
                                        0., 0., 0.).encode()

    def encode(self, command):
        issued = self.monotonic_ns()  # Exactly one original issue timestamp per publication.
        if (type(issued) is not int or not self.last_issued_ns < issued < self.protocol.MAX_INT or
                self.sequence >= self.protocol.MAX_INT):
            return None, False, "source monotonic clock did not advance or sequence exhausted"
        self.last_issued_ns = issued
        self.sequence += 1
        diagnostic = None
        latest = min(issued + self.protocol.MAX_LIFETIME_NS, self.protocol.MAX_INT)
        try:
            expiry = min(math.floor(number(command.expires_at, "command expiry") * 1e9), latest)
            if expiry <= issued:
                raise ValueError("original command expired before publication")
            envelope = self.protocol.DeadlineCommand(self.clock_id, self.source_instance,
                self.sequence, issued, expiry, command.vx, command.vy, command.wz)
            payload = envelope.encode()
        except (ValueError, TypeError, AttributeError, OverflowError) as exc:
            diagnostic = str(exc)[:160]
            envelope = self.protocol.DeadlineCommand(self.clock_id, self.source_instance,
                self.sequence, issued, latest, 0., 0., 0.)
            payload = envelope.encode()  # Only the genuine zero receives a fresh bounded deadline.
        return payload, not envelope.is_zero, diagnostic


class BridgeCore:
    """Transport-free admission and output fence; callbacks never publish."""

    def __init__(self, robot_id, producer_id, boot_id, fields, *, clock=time.monotonic,
                 required_participants=(), command_contract=LEGACY_CONTRACT,
                 monotonic_ns=time.monotonic_ns, command_clock_id=None, source_instance=None):
        if robot_id not in ROBOT_IDS or set(fields) != set(ROBOT_IDS):
            raise ValueError("configure both robots and pin one output robot")
        if command_contract not in (LEGACY_CONTRACT, DEADLINE_CONTRACT):
            raise ValueError("unknown bridge command contract")
        self.command_contract = command_contract
        self.deadline = (DeadlineEncoder(monotonic_ns=monotonic_ns, clock_id=command_clock_id,
                                        source_instance=source_instance)
                         if command_contract == DEADLINE_CONTRACT else None)
        self.robot_id, self.producer_id = robot_id, token(producer_id, "producer_id")
        self.clock_name = "linux-monotonic:" + token(boot_id, "boot_id")
        self.clock = clock
        self.executor = Executor(fields, LIMITS, required_participants=required_participants)
        if self.executor.required_participants and robot_id not in self.executor.required_participants:
            raise ValueError("required participants must include this bridge's robot")
        self.stopped = False
        self.last_decision = None
        self.completed_goal_cycle = None  # Immutable (epoch, prior resume seq, first completion time).

    def stop(self):
        self.stopped = True
        self.executor.stop()

    def operator(self, message):
        command = message.get("command")
        if command == "stop":
            self.stop()
            return {"accepted": True, "reason": "Stop latched"}
        if self.stopped:
            raise ValueError("bridge Stop is latched; create a new explicit session")
        now = self.clock()
        if command == "arm":
            event = {"event": "arm", "epoch": message.get("epoch"), "clock": self.clock_name, "now": now}
        elif command == "resume":
            event = {"event": "resume", "robot_id": self.robot_id, "frame_seq": message.get("frame_seq"),
                     "epoch": self.executor.epoch, "clock": self.clock_name, "now": now}
        else:
            raise ValueError("not a core operator action")
        result = self.executor.handle(event)
        return {key: result[key] for key in ("accepted", "reason")}

    def ingest(self, message):
        if not isinstance(message, dict):
            raise ValueError("normalized event must be an object")
        if message.get("event") not in ("frame", "intent"):
            raise ValueError("perception ingress accepts frame/intent only; operator authority is separate")
        if message.get("producer_id") != self.producer_id or message.get("clock") != self.clock_name:
            raise ValueError("producer or same-host Linux clock binding differs")
        if message["event"] == "intent" and message.get("robot_id") != self.robot_id:
            raise ValueError("intent does not address this bridge's pinned robot")
        event = dict(message)
        event["now"] = self.clock()  # NEVER substitute capture, issue or expiry
        result = self.executor.handle(event)
        return {key: result[key] for key in ("accepted", "reason")}

    def hold(self, held, reason="external command guard hold"):
        # Detector labels plus guard prefixes can exceed the Executor's token
        # limit. Diagnostics must never prevent the actual hold transition.
        diagnostic = " ".join(reason.split())[:128] if isinstance(reason, str) else ""
        self.executor.set_external_hold(self.robot_id, held, self.clock(),
                                        diagnostic or "external command guard hold")

    def tick(self):
        self.last_decision = self.executor.tick(self.clock())[self.robot_id]
        if self.last_decision.phase == "await_reset":
            binding = (self.executor.epoch, self.executor.resume_frame_seq[self.robot_id])
            if self.completed_goal_cycle is None or self.completed_goal_cycle[:2] != binding:
                self.completed_goal_cycle = (*binding, self.clock())
        return self.last_decision

    def output(self, decision):
        """Check again immediately before publication, after any slow checks."""
        now = number(self.clock(), "publication clock")
        cmd = decision.command
        values = (cmd.vx, cmd.vy, cmd.wz, cmd.expires_at)
        if (cmd.robot_id != self.robot_id or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values) or
                abs(cmd.vx) > LIMITS.vx or abs(cmd.vy) > LIMITS.vy or abs(cmd.wz) > LIMITS.wz):
            self.stop()
            raise ValueError("invalid, misrouted or out-of-bounds Executor command")
        intent = self.executor.intents[self.robot_id]
        frame = self.executor.frame
        if self.executor.last_now is not None and now < self.executor.last_now:
            self.stop()
            raise ValueError("publication clock moved backwards")
        if (decision.recovery_requested or self.stopped or not self.executor.armed or now >= cmd.expires_at or
                self.executor.last_now is None or now < self.executor.last_now or
                intent is None or now >= intent.valid_until or frame is None):
            return zero(now, self.robot_id)
        deadline = self.executor.frame_evidence_deadline(self.robot_id)
        if now < frame[0] or deadline is None or now >= deadline:
            return zero(now, self.robot_id)
        return cmd


@dataclass(frozen=True)
class ResumeTicket:
    epoch: str
    prior_resume_frame_seq: int
    pressed_at: float


class OperatorInbox:
    def __init__(self, clock, capture_start, capture_resume):
        self.clock, self.capture_start, self.capture_resume = clock, capture_start, capture_resume
        self.urgent, self.normal = SimpleQueue(), SimpleQueue()
        self.stop_seen = threading.Event()

    def put(self, message):
        at = self.clock()
        message = dict(message)
        message.pop("_ticket", None)
        message.pop("_error", None)
        if message.get("command") in ("start", "resume"):
            try:
                capture = self.capture_start if message["command"] == "start" else self.capture_resume
                message["_ticket"] = capture(pressed_at=at)
            except ValueError as exc:
                message["_error"] = str(exc)
        urgent = message.get("command") in ("stop", "quit")
        if urgent:
            self.stop_seen.set()
        (self.urgent if urgent else self.normal).put((at, message))

    def get(self, *, urgent_only=False):
        for queue in ((self.urgent,) if urgent_only else (self.urgent, self.normal)):
            try:
                return queue.get_nowait()
            except Empty:
                pass
        return None

    def read(self, stream):
        try:
            for line in stream:
                try:
                    if len(line) > 8192:
                        raise ValueError("oversized operator command")
                    row = json.loads(line)
                    if not isinstance(row, dict):
                        raise ValueError("operator command must be an object")
                    self.put(row)
                except (ValueError, TypeError) as exc:
                    self.put({"command": "invalid", "reason": str(exc)})
        finally:
            self.put({"command": "quit", "reason": "operator input closed"})


def _node_class():
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Empty as EmptyMessage, Float32MultiArray, String
    from run_through_kick_node import StockWalkerEvidence, STOP_BUTTONS

    class FCExecutorBridge(Node):
        def __init__(self, core, *, same_manager, same_command_path, emit=None):
            super().__init__(NODE_NAME)
            self.core, self.clock = core, core.clock
            self.same_manager, self.same_command_path = same_manager, same_command_path
            self.emit = emit or EventSink()
            self.lock = threading.RLock()
            self.native_valid = self.path_valid = False
            self.identity_at = None
            self.guard_status = self.guard_at = self.guard_source = self.guard_deadline = None
            self.guard_error = "no guard status received"
            self.authority_status = self.authority_at = None
            self.authority_error = "no native command authority received"
            self.gateway_at = self.gateway_authority = None
            self.gateway_released = False
            self.gateway_blocked = True
            self.observation = None
            self.closing = self.closed = False
            self.hard_stopped = self.ever_armed = self.submitted_start = False
            self.native_discovery_until = None
            self.guard_only_exit = False
            self.close_at = self.zero_seen_at = None
            self.reason = "waiting for explicit arm"
            self.last_hold = None
            self.hold_zero_at = self.hold_zero_seen_at = None
            self.ingress_result = None
            self.command_error = None
            strict = core.deadline is not None
            self.command_pub = self.create_publisher(String if strict else Twist,
                core.deadline.protocol.REQUEST_TOPIC if strict else "/kick_cmd_vel", 1)
            self.zero_pub = None if strict else self.create_publisher(Twist, "/cmd_vel", 1)
            self.stop_pub = self.create_publisher(EmptyMessage, "/emergency_stop", 10)
            self.status_pub = self.create_publisher(String, STATUS_TOPIC,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.stock = StockWalkerEvidence(self, forward_ceiling=.3, lateral_ceiling=.3,
                **({"command_contract": DEADLINE_CONTRACT, "command_clock_id": core.deadline.clock_id}
                   if strict else {}))
            self.stock.clock = self.clock
            self.start_client = None
            try:
                from foundation.hardware.software_start import attach
                self.start_client = attach(self, clock=self.clock)
            except ImportError:
                pass
            self.inbox = OperatorInbox(self.clock, self.capture_start, self.capture_resume)
            self.create_subscription(String, EVENT_TOPIC, self.on_event, 1)
            self.create_subscription(String, GUARD_STATUS_TOPIC, self.on_guard, 1)
            if strict:
                self.create_subscription(String, NATIVE_AUTHORITY_TOPIC, self.on_native_authority,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(String, "/estop_controller/stop_status", self.on_gateway, 10)
            self.create_subscription(Float32MultiArray, "/neural_controller_default/observation", self.on_observation, 1)
            self.create_subscription(EmptyMessage, "/emergency_stop", self.on_emergency, 10)
            self.create_subscription(Joy, "/joy", self.on_joy, 10)
            self.create_subscription(String, "/run_through_kick/state", self.on_shot, 1)
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(TICK_S, self.on_tick, clock=steady)
            self.create_timer(IDENTITY_S, self.check_identity, clock=steady)
            self.create_timer(.2, self.publish_status, clock=steady)

        def capture_start(self, *, pressed_at):
            if self.start_client is None:
                raise ValueError("optional Start service interface unavailable")
            ticket = self.start_client.capture(pressed_at=pressed_at)
            with self.lock:
                if self.closing or self.core.executor.armed or self.inbox.stop_seen.is_set():
                    raise ValueError("Start requires an idle, unstopped bridge")
                if (ticket.instance_id, ticket.epoch) != self.gateway_authority:
                    raise ValueError("Start differs from observed gateway authority")
            return ticket

        def check_identity(self):
            try:
                native, path = self.same_manager() is True, self.same_command_path() is True
            except (OSError, ValueError, KeyError, TypeError):
                native, path = False, False
            with self.lock:
                self.native_valid, self.path_valid, self.identity_at = native, path, self.clock()

        def resume_binding(self):
            return (self.core.executor.epoch,
                    self.core.executor.resume_frame_seq[self.core.robot_id])

        def resume_problem(self):
            if self.closing or self.core.stopped or self.inbox.stop_seen.is_set():
                return "bridge is stopping"
            completed = self.core.completed_goal_cycle
            if completed is None or completed[:2] != self.resume_binding():
                return "current goal return has not completed on a control tick"
            if self.start_client is not None and (self.start_client.pending is not None or self.start_client.uncertain):
                return "Start pending or uncertain"
            problem = self.native_problem() or self.guard_problem()
            if problem:
                return str(problem)
            try:
                self.core.executor.check_resume(self.core.robot_id, self.core.executor.frame_seq, self.clock())
            except (ValueError, TypeError) as exc:
                return str(exc)
            return None

        def capture_resume(self, *, pressed_at):
            # Immutable proof only: never block the stdin reader from a later
            # Stop. Mutable readiness is checked in serialized processing.
            completed = self.core.completed_goal_cycle
            if completed is None or completed[2] > pressed_at:
                raise ValueError("goal return was not completed before resume receipt")
            if not 0 <= self.clock() - pressed_at <= .2:
                raise ValueError("operator action expired")
            return ResumeTicket(*completed[:2], pressed_at)

        def native_problem(self):
            now = self.clock()
            if not self.native_valid or self.identity_at is None or not 0 <= now - self.identity_at <= .3:
                return "selected native runtime identity unavailable"
            if self.gateway_blocked or self.gateway_at is None or not 0 <= now - self.gateway_at <= .6:
                return "gateway authority blocked or stale"
            if self.observation is None or not 0 <= now - self.observation[1] <= OBS_MAX_AGE_S:
                return "stock policy observation unavailable or stale"
            problem = self.stock.problem()
            if problem or self.core.deadline is None:
                return problem
            return native_authority_problem(self.authority_status, self.authority_at,
                self.authority_error, now, self.core.deadline.source_instance)

        def on_native_authority(self, message):
            with self.lock:
                now = self.clock()
                try:
                    self.authority_status = decode_native_authority(message.data, now,
                        self.core.deadline.clock_id, self.authority_status)
                    self.authority_error = None
                except (ValueError, TypeError, KeyError, OverflowError) as exc:
                    self.authority_error = "invalid native command authority: " + str(exc)
                self.authority_at = now

        def guard_problem(self, *, camera=True, allow_absent_native=False):
            graph = guard_publishers_problem(self, "/" + NODE_NAME,
                command_contract=self.core.command_contract, allow_absent_native=allow_absent_native)
            if graph and graph[0] == "authority":
                return graph
            if not self.path_valid or self.identity_at is None or not 0 <= self.clock() - self.identity_at <= .3:
                return "path", "selected guard process identity unavailable"
            return (graph or
                    guard_status_problem(self.guard_status, self.guard_at, self.guard_error,
                                         self.guard_deadline, self.clock(), camera=camera,
                                         command_contract=self.core.command_contract))

        def on_guard(self, message):
            with self.lock:
                now = self.clock()
                try:
                    self.guard_status, self.guard_source, self.guard_deadline = decode_guard_status(
                        message.data, now, self.guard_source, self.guard_deadline,
                        command_contract=self.core.command_contract,
                        clock_id=self.core.deadline.clock_id if self.core.deadline else None,
                        source_instance=self.core.deadline.source_instance if self.core.deadline else None)
                    self.guard_error = None
                except (ValueError, TypeError, KeyError, OverflowError) as exc:
                    self.guard_error = "invalid guard status: " + str(exc)
                self.guard_at = now

        def on_gateway(self, message):
            with self.lock:
                try:
                    state = json.loads(message.data)
                    if (state.get("protocol") != "pupper_stop_v1" or
                            not isinstance(state.get("instance_id"), str) or not state["instance_id"] or
                            type(state.get("epoch")) is not int or not 0 <= state["epoch"] < 2**64 or
                            any(type(state.get(k)) is not bool for k in
                                ("latched", "fault", "stop_required", "unknown_remote_submission", "shutting_down"))):
                        raise ValueError("invalid gateway status")
                    authority = (state["instance_id"], state["epoch"])
                    if self.gateway_authority is not None and authority != self.gateway_authority:
                        raise ValueError("gateway authority epoch or instance changed")
                    self.gateway_authority = authority
                    unsafe = any(state[k] for k in ("fault", "stop_required", "unknown_remote_submission", "shutting_down"))
                    self.gateway_blocked = state["latched"] or unsafe
                    self.gateway_at = self.clock()
                    if not self.gateway_blocked:
                        self.gateway_released = True
                    elif unsafe or self.gateway_released or self.ever_armed:
                        self.shutdown("gateway Stop or fault", hard=True, publish_stop=False)
                except (ValueError, TypeError, AttributeError, KeyError):
                    self.gateway_blocked = True
                    if self.gateway_authority is not None or self.ever_armed:
                        self.shutdown("gateway authority invalid", hard=True)

        def on_observation(self, message):
            with self.lock:
                values = message.data
                if len(values) != 48 * 20 or not all(math.isfinite(v) for v in values):
                    self.observation = None
                    if self.ever_armed:
                        self.shutdown("invalid stock observation", hard=True)
                    return
                applied = tuple(values[6:9])
                self.observation = applied, self.clock()
                if self.closing and all(abs(v) <= .02 for v in applied):
                    self.zero_seen_at = self.clock()
                if self.hold_zero_at is not None and all(abs(v) <= .02 for v in applied):
                    self.hold_zero_seen_at = self.clock()

        def on_event(self, message):
            with self.lock:
                try:
                    if self.closing or self.inbox.stop_seen.is_set():
                        raise ValueError("bridge is stopping")
                    if len(message.data) > 32768:
                        raise ValueError("oversized normalized event")
                    result = self.core.ingest(json.loads(message.data))
                except (ValueError, TypeError) as exc:
                    result = {"accepted": False, "reason": str(exc)}
                self.ingress_result = {"received_at": self.clock(), **result}

        def on_emergency(self, message):
            with self.lock:
                self.shutdown("emergency Stop", hard=True, publish_stop=False)

        def on_joy(self, message):
            if any(i < len(message.buttons) and message.buttons[i] for i in STOP_BUTTONS):
                with self.lock:
                    self.shutdown("physical Stop button", hard=True)

        def on_shot(self, message):
            try:
                state = json.loads(message.data).get("state")
            except (ValueError, AttributeError):
                state = "invalid"
            if state not in ("idle", "done", "aborted"):
                with self.lock:
                    self.shutdown("competing shot adapter active", hard=True)

        def publish_command(self, command):
            if self.core.deadline is not None:
                if any((command.vx, command.vy, command.wz)):
                    if self.inbox.stop_seen.is_set() or self.closing:
                        self.drain_operator(urgent_only=True)
                        return
                    if self.core.stopped or not self.core.executor.armed:
                        command = zero(self.clock(), self.core.robot_id)
                payload, nonzero, self.command_error = self.core.deadline.encode(command)
                if self.command_error:
                    self.emit({"event": "command_zero", "reason": self.command_error})
                if payload is None:
                    return  # Never invent a future issue timestamp; native expiry still applies.
                message = String(data=payload)
                if nonzero and (self.inbox.stop_seen.is_set() or self.closing):
                    self.drain_operator(urgent_only=True)
                    return
                self.command_pub.publish(message)
                return
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = command.vx, command.vy, command.wz
            if any((command.vx, command.vy, command.wz)):
                # This is the last boundary, after evidence checks and message
                # construction. A received operator Stop wins over this output.
                if self.inbox.stop_seen.is_set() or self.closing:
                    self.drain_operator(urgent_only=True)
                    return
                if self.clock() >= command.expires_at:
                    message = Twist()
            self.command_pub.publish(message)

        def shutdown(self, reason, *, hard, publish_stop=True):
            pending = self.start_client is not None and (self.start_client.pending is not None or self.start_client.uncertain)
            hard = hard or pending
            if hard and not self.hard_stopped:
                self.hard_stopped = True
                if publish_stop:
                    self.stop_pub.publish(EmptyMessage())
            self.guard_only_exit = not hard and not self.hard_stopped
            self.core.stop()
            if not self.closing:
                self.close_at = self.clock()
            self.closing, self.reason = True, reason
            self.publish_command(zero(self.clock(), self.core.robot_id))
            if self.zero_pub is not None:
                self.zero_pub.publish(Twist())

        def operator(self, at, message):
            command = message.get("command")
            try:
                if command in ("stop", "quit"):
                    hard = command == "stop" or self.ever_armed or self.submitted_start
                    self.shutdown(message.get("reason", "operator " + command), hard=hard)
                    result = {"accepted": True, "reason": self.reason}
                elif self.closing or self.inbox.stop_seen.is_set():
                    raise ValueError("bridge is stopping")
                elif command == "status":
                    result = {"accepted": True, "reason": "status", "status": self.status()}
                elif command == "start":
                    if message.get("_error"):
                        raise ValueError(message["_error"])
                    if self.core.executor.armed or self.start_client is None or "_ticket" not in message:
                        raise ValueError("Start requires idle bridge and receipt-time ticket")
                    if (not self.native_valid or not self.path_valid or self.identity_at is None or
                            not 0 <= self.clock() - self.identity_at <= .3):
                        raise ValueError("selected runtime is unavailable")
                    self.start_client.send(message["_ticket"])
                    self.submitted_start = True
                    self.native_discovery_until = None
                    result = {"accepted": True, "reason": "one Start submitted; FC remains unarmed"}
                elif command in ("arm", "resume"):
                    if not 0 <= self.clock() - at <= .2:
                        raise ValueError("operator action expired")
                    if self.start_client is not None and (self.start_client.pending is not None or self.start_client.uncertain):
                        raise ValueError("Start pending or uncertain")
                    problem = self.native_problem() or self.guard_problem()
                    if problem:
                        raise ValueError(str(problem))
                    if command == "arm" and any(abs(v) > .02 for v in self.observation[0]):
                        raise ValueError("arm requires current zero controller feedback")
                    if command == "resume":
                        if message.get("_error"):
                            raise ValueError(message["_error"])
                        ticket = message.get("_ticket")
                        if (not isinstance(ticket, ResumeTicket) or ticket.pressed_at != at or
                                (ticket.epoch, ticket.prior_resume_frame_seq) != self.resume_binding()):
                            raise ValueError("resume requires an unconsumed receipt ticket for this goal cycle")
                        problem = self.resume_problem()
                        if problem:
                            raise ValueError(problem)
                        message = dict(message)
                        message.setdefault("frame_seq", self.core.executor.frame_seq)
                        # Readiness/graph queries can take time or receive Stop.
                        # Fence semantic consumption as well as later publication.
                        if not 0 <= self.clock() - at <= .2:
                            raise ValueError("operator action expired")
                        if self.closing or self.inbox.stop_seen.is_set():
                            raise ValueError("bridge is stopping")
                        if (ticket.epoch, ticket.prior_resume_frame_seq) != self.resume_binding():
                            raise ValueError("goal cycle changed before resume consumption")
                    result = self.core.operator(message)
                    self.ever_armed |= self.core.executor.armed
                else:
                    raise ValueError(message.get("reason", "unknown operator command"))
            except (ValueError, TypeError, KeyError) as exc:
                result = {"accepted": False, "reason": str(exc)}
            self.emit({"event": "operator_result", "command": command, **result})

        def drain_operator(self, *, urgent_only=False):
            # Drain a bounded snapshot of shutdown input, including Stop behind
            # quit. Ordinary input is limited to one action per timer tick.
            for _ in range(self.inbox.urgent.qsize() if urgent_only else 1):
                item = self.inbox.get(urgent_only=urgent_only)
                if item is not None:
                    self.operator(*item)

        def on_tick(self):
            with self.lock:
                self.drain_operator(urgent_only=self.closing)
                self.stock.poll()
                if self.start_client is not None:
                    result = self.start_client.poll()
                    if result:
                        self.emit(result)
                        if self.core.deadline is not None and result.get("success") is True:
                            self.native_discovery_until = self.clock() + NATIVE_DISCOVERY_S
                    if self.start_client.uncertain:
                        self.shutdown("Start result uncertain", hard=True)
                if self.submitted_start and not self.closing:
                    if (not self.native_valid or self.identity_at is None or
                            not 0 <= self.clock() - self.identity_at <= .3):
                        self.shutdown("native runtime lost after Start", hard=True)
                    else:
                        # Native endpoints are activation-scoped. Permit their
                        # absence only while the explicit Start is pending and
                        # during bounded DDS discovery after its acknowledgement.
                        native_pending = (self.core.deadline is not None and not self.ever_armed and
                            self.start_client is not None and (self.start_client.pending is not None or
                            (self.native_discovery_until is not None and self.clock() < self.native_discovery_until)))
                        path_problem = self.guard_problem(camera=False, allow_absent_native=native_pending)
                        if path_problem:
                            self.shutdown(path_problem[1], hard=path_problem[0] == "authority")
                if self.ever_armed and not self.closing:
                    problem = self.native_problem()
                    if problem:
                        self.shutdown(problem, hard=True)
                if self.core.executor.armed and not self.closing:
                    problem = self.guard_problem()
                    if problem and problem[0] != "camera":
                        self.shutdown(problem[1], hard=problem[0] == "authority")
                    else:
                        held = problem is not None
                        if held and self.last_hold is not True:
                            self.hold_zero_at, self.hold_zero_seen_at = self.clock(), None
                        if self.last_hold is True and self.hold_zero_seen_at is None:
                            held = True
                            problem = problem or ("camera", "waiting for post-hold zero controller feedback")
                        self.core.hold(held, problem[1] if held else "guard clear")
                        if held != self.last_hold:
                            self.emit({"event": "guard_hold", "held": held, "reason": problem[1] if held else "guard clear"})
                            self.last_hold = held
                        if (held and self.hold_zero_seen_at is None and self.hold_zero_at is not None and
                                self.clock() - self.hold_zero_at >= .5):
                            self.shutdown("camera hold zero unconfirmed", hard=True)
                        decision = self.core.tick()
                        self.drain_operator(urgent_only=True)
                        if not self.closing and not self.inbox.stop_seen.is_set():
                            try:
                                command = self.core.output(decision)
                                problem = self.stock.problem(command=command)
                                if problem:
                                    raise ValueError(problem)
                                self.drain_operator(urgent_only=True)
                                if not self.closing and not self.inbox.stop_seen.is_set():
                                    self.publish_command(self.core.output(decision))
                            except ValueError as exc:
                                self.shutdown(str(exc), hard=True)
                elif self.core.deadline is not None and not self.closing:
                    # Native/guard source binding requires harmless genuine
                    # zeros before arm; it must not depend on guard readiness.
                    self.publish_command(zero(self.clock(), self.core.robot_id))
                if self.closing:
                    self.drain_operator(urgent_only=True)
                    if self.guard_only_exit and self.native_problem():
                        self.shutdown("native authority lost during guard cleanup", hard=True)
                    if self.zero_seen_at is not None or self.clock() - self.close_at >= .5:
                        if self.zero_seen_at is None and (self.ever_armed or self.submitted_start):
                            self.shutdown("cleanup zero unconfirmed", hard=True)
                        self.drain_operator(urgent_only=True)
                        self.closed = True

        def status(self):
            with self.lock:
                return self._status()

        def _status(self):
            now = self.clock()
            frame = self.core.executor.frame
            own = frame[1][self.core.robot_id] if frame else None
            participants = self.core.executor.required_participants
            intent = self.core.executor.intents[self.core.robot_id]
            resume_problem = self.resume_problem()
            return {"event": "bridge_status", "robot_id": self.core.robot_id,
                    "command_contract": self.core.command_contract,
                    "command_clock_id": self.core.deadline.clock_id if self.core.deadline else None,
                    "source_instance": self.core.deadline.source_instance if self.core.deadline else None,
                    "source_sequence": self.core.deadline.sequence if self.core.deadline else None,
                    "command_error": self.command_error,
                    "native_command_authority": self.authority_status if self.core.deadline else None,
                    "sampled_at_monotonic": now,
                    "clock": self.core.clock_name, "epoch": self.core.executor.epoch,
                    "frame_seq": self.core.executor.frame_seq if frame else None,
                    "capture_age_s": now - own.captured_at if own and own.robot is not None else None,
                    "field_capture_age_s": now - frame[0] if frame else None,
                    "required_participants": list(participants),
                    "participant_capture_age_s": {r: now - frame[1][r].captured_at
                        if frame and frame[1][r].robot is not None else None for r in participants},
                    "field_evidence_deadline": self.core.executor.frame_evidence_deadline(self.core.robot_id),
                    "intent": {"id": intent.identity[0], "kind": intent.kind,
                               "age_s": now - intent.issued_at,
                               "time_left_s": intent.valid_until - now} if intent else None,
                    "resume": {"available": resume_problem is None,
                               "reason": resume_problem or "current goal return and evidence permit resume"},
                    "armed": self.core.executor.armed, "closing": self.closing,
                    "hard_stopped": self.hard_stopped, "guard_only_exit": self.guard_only_exit,
                    "reason": self.reason, "native_problem": self.native_problem(),
                    "guard_problem": self.guard_problem(), "zero_confirmed": self.zero_seen_at is not None,
                    "applied_command": list(self.observation[0]) if self.observation else None,
                    "applied_age_s": self.clock() - self.observation[1] if self.observation else None,
                    "hold_zero_confirmed": self.hold_zero_seen_at is not None,
                    "ingress_result": self.ingress_result,
                    "notification_drops": getattr(self.emit, "dropped", 0),
                    "decision": asdict(self.core.last_decision) if self.core.last_decision else None}

        def publish_status(self):
            self.status_pub.publish(String(data=json.dumps(self.status(), allow_nan=False)))

    return FCExecutorBridge


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--runtime-receipt", type=Path)
    parser.add_argument("--field-config", type=Path)
    parser.add_argument("--robot-id", choices=ROBOT_IDS, required=True)
    parser.add_argument("--producer-id", required=True)
    parser.add_argument("--command-contract", choices=(LEGACY_CONTRACT, DEADLINE_CONTRACT),
                        default=LEGACY_CONTRACT)
    parser.add_argument("--require-participants", nargs="+", choices=ROBOT_IDS, default=(),
                        help="Tracks required for motion, including this robot; requires original per-row captured_at")
    args = parser.parse_args(argv)
    try:
        token(args.producer_id, "producer_id")
        if (len(set(args.require_participants)) != len(args.require_participants) or
                (args.require_participants and args.robot_id not in args.require_participants)):
            raise ValueError("required participants must be distinct and include this robot")
        fields = load_fields(json.loads(args.field_config.read_text())) if args.field_config else None
        if not args.execute:
            print(json.dumps({"event": "bridge_dry_run", "robot_id": args.robot_id,
                              "producer_id": args.producer_id, "limits": asdict(LIMITS),
                              "required_participants": list(args.require_participants),
                              "command_contract": args.command_contract,
                              "fields": {r: asdict(f) for r, f in fields.items()} if fields else None,
                              "execute_requires": ["explicit field-config", "guarded runtime receipt", "operator arm"],
                              "ros_started": False}))
            return 0
        if fields is None or args.runtime_receipt is None:
            raise ValueError("execution requires explicit --field-config and --runtime-receipt")
        from foundation.hardware import runtime_identity
        preflight = runtime_identity.read_json(args.runtime_receipt)
        runtime = runtime_identity.verify_loaded(preflight)
        topology = DEADLINE_CONTRACT if args.command_contract == DEADLINE_CONTRACT else "guarded"
        if runtime.get("command_topology") != topology:
            raise ValueError("FC bridge command contract differs from selected runtime topology")
        if runtime_identity.read_json(preflight["manifest_path"]).get("trial_profile") != "walk":
            raise ValueError("FC bridge requires the walk profile")
        verify_bridge_sources(runtime)
        from run_through_kick_node import STOCK_POLICY_SHA256
        if runtime["files"]["policy"]["sha256"] != STOCK_POLICY_SHA256:
            raise ValueError("selected policy differs from the stock evidence guard")
        boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        core = BridgeCore(args.robot_id, args.producer_id, boot, fields,
                          required_participants=args.require_participants, command_contract=args.command_contract)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print("bridge refused: " + str(exc), file=sys.stderr)
        return 2
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    import signal
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = _node_class()(core, same_manager=lambda: runtime_identity.same_manager(runtime),
                         same_command_path=lambda: runtime_identity.same_command_path(runtime))
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[sig] = signal.signal(sig, lambda *_: node.inbox.put({"command": "stop", "reason": "process signal"}))
    threading.Thread(target=node.inbox.read, args=(sys.stdin,), daemon=True).start()
    node.emit({"event": "bridge_ready", "robot_id": core.robot_id, "producer_id": core.producer_id,
               "clock": core.clock_name, "fields": {r: asdict(f) for r, f in fields.items()},
               "required_participants": list(core.executor.required_participants),
               "command_contract": core.command_contract,
               "source_instance": core.deadline.source_instance if core.deadline else None,
               "limits": asdict(LIMITS), "armed": False, "software_start_supported": node.start_client is not None,
               "identity_scope": "operational evidence; not authentication"})
    try:
        while rclpy.ok() and not node.closed:
            rclpy.spin_once(node, timeout_sec=TICK_S)
        return 3 if node.hard_stopped else 0
    finally:
        if not node.closed:
            node.shutdown("bridge execution ended", hard=True)
        node.drain_operator(urgent_only=True)
        node.emit({**node.status(), "event": "bridge_closed"})
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
