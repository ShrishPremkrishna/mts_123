#!/usr/bin/env python3
"""Candidate actual-command camera gate; no arming, homing or controller calls.

Selected path: /kick_cmd_vel -> existing mux -> /fc/requested_cmd_vel -> this
guard -> /cmd_vel. Twist has no source stamp: its 150 ms limit bounds *receipt*
age only. Upstream still owns finite action expiry and cancellation. Following a
camera hold that vetoed motion, upstream must send zero, then a fresh command
after clearance; repeated mux copies of a pre-hold command cannot resume it.

Detection String JSON uses schema fc_camera_obstacles_v1, producer_instance,
sequence, captured_at_ros_ns (original image stamp, integer), valid, healthy,
format normalized_xyxy_v1, image_width 700, image_height 572,
horizontal_fov_deg 220, and detections [{box:[l,t,r,b], confidence, class_id}].
Coordinates refer to that fixed projection. Default policy gates all classes.
Explicit --arena-ball-contact exempts only class_id "32" from the selected HEF
and canonical COCO labels, identified by detector_identity SHA256 fields. This
permits approach/alignment as well as contact; it does not establish that the
intended pink ball is recognized. Every other detection retains the same gate.
Healthy describes inference, not unobscured visibility or proven free space.

ROS and monotonic clocks are compared using integer nanoseconds. ROS clock
jumps or producer identity changes latch a fault until explicit process restart;
restart earns fresh clearance. Sequence/capture highwaters never reset in place.
Capture age includes a conservative 50 ms clock-offset reserve, mapping against
the initial clock anchor rather than arrival-time offset. Stable clocks therefore
have at most 300 ms usable capture age under the provisional 350 ms threshold.
Captures whose earliest plausible time predates that anchor are rejected. This
assumes source/local ROS offsets remain in the admitted +/-50 ms window; sampled
clock checks cannot establish the absence of arbitrary unsampled clock jumps.
The steady timer expires evidence even when no messages arrive or ROS time stops.
Defaults are provisional camera thresholds, not hardware stopping acceptance.
Guard death relies on the selected native 250 ms receipt watchdog; this process
does not authenticate other publishers or establish native controller authority.

Explicit --command-contract fc_deadline_v1 selects a separate String-only path
from /fc/body_command/request to /fc/body_command/admitted. It preserves the
source's same-host monotonic deadline and can only veto its immutable origin.
This mode starts closed and requires a genuine source zero after camera
clearance before a newer motion command; receipt of old input cannot release it.
"""
from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
import math
from pathlib import Path
import sys
import time

# Support both `python -m ...` and the deployment's direct script invocation.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from final_project.pupper_fc.obstacles import (  # noqa: E402
    BOX_FORMAT, Detection, DetectionFrame, ForwardObstacleGate, ObstacleConfig,
)

SCHEMA = "fc_camera_obstacles_v1"
INPUT_TOPIC = "/fc/requested_cmd_vel"
FRAME_TOPIC = "/fc/obstacle_detections"
OUTPUT_TOPIC = "/cmd_vel"
STATUS_TOPIC = "/fc/body_command_guard/status"
TIMER_S = 0.02
INPUT_TIMEOUT_NS = 150_000_000
CLOCK_JUMP_NS = 50_000_000
MAX_PAYLOAD_BYTES = 65536
MAX_DETECTIONS = 256
LEGACY_CONTRACT = "legacy_twist"
DEADLINE_CONTRACT = "fc_deadline_v1"  # Its parser is imported only in explicit strict mode.
ARENA_DETECTOR_IDENTITY = {
    "model_sha256": "3e91fd4c8be77a560169185bd8cba7e6a6b9ab8be14ea118d5e9d054df51ca36",
    "labels_sha256": "bd17f1ee35d5f3c862a4894605855abbb9dda4b0621fdb0ac4c2c8c7bb7e730a",
}


@dataclass(frozen=True)
class BodyCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0


ZERO = BodyCommand()


@dataclass(frozen=True)
class GuardResult:
    command: BodyCommand
    status: dict
    envelope: object = None  # Strict mode's immutable DeadlineCommand; absent in legacy mode.


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError("expected a finite number")
    try:
        value = float(value)
    except OverflowError:
        raise ValueError("expected a finite number") from None
    if not math.isfinite(value):
        raise ValueError("expected a finite number")
    return value


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON field")
        result[key] = value
    return result


class BodyCommandGuard:
    """Synchronous pure core. The caller serializes callbacks and owns publishing."""

    def __init__(self, obstacle_config=ObstacleConfig(), *, arena_ball_contact=False):
        if type(arena_ball_contact) is not bool:
            raise ValueError("arena_ball_contact must be an explicit boolean")
        self.gate = ForwardObstacleGate(obstacle_config)
        self._arena_ball_contact = arena_ball_contact
        self._ignored_ball_count = 0
        self._anchor = None
        self._last_clocks = None
        self._fault = ""
        self._producer = None
        self._sequence = -1
        self._capture_ros_ns = 0
        self._frame_mono_ns = None
        self._input_ns = None
        self._command = ZERO
        self._last_valid_command = ZERO  # Receipt timeout is not an upstream cancellation.
        self._input_error = "no body command received"
        self._input_state = "missing"
        self._camera_clear = False
        self._clear_since_ns = None
        self._hold_since_ns = None
        self._needs_zero = False

    def _clock(self, ros_ns, mono_ns):
        if self._fault:
            return False
        if (type(ros_ns) is not int or not 0 < ros_ns < 2**63 or
                type(mono_ns) is not int or not 0 <= mono_ns < 2**63):
            self._fault = "invalid clock; restart guard"
        elif self._last_clocks and (mono_ns < self._last_clocks[1] or ros_ns < self._last_clocks[0]):
            self._fault = "clock regression; restart guard"
        else:
            if self._anchor is None:
                self._anchor = (ros_ns, mono_ns)
                self._hold_since_ns = mono_ns
            ros_delta = ros_ns - self._anchor[0]
            mono_delta = mono_ns - self._anchor[1]
            if abs(ros_delta - mono_delta) > CLOCK_JUMP_NS:
                self._fault = "ROS/monotonic clock divergence; restart guard"
            self._last_clocks = (ros_ns, mono_ns)
        if self._fault:
            self._command = ZERO
        return not self._fault

    def _observe(self, decision, mono_ns):
        clear = decision.permitted
        if not clear:
            if self._camera_clear:
                self._hold_since_ns = mono_ns
                self._needs_zero = self._needs_zero or self._last_valid_command != ZERO
            self._command = ZERO
            self._clear_since_ns = None
        elif not self._camera_clear:
            self._clear_since_ns = mono_ns
            self._command = ZERO
        self._camera_clear = clear

    def _begin(self, ros_ns, mono_ns):
        if not self._clock(ros_ns, mono_ns):
            return False
        self._observe(self.gate.evaluate(mono_ns / 1e9), mono_ns)
        if (self._frame_mono_ns is not None and
                max(mono_ns - self._frame_mono_ns, ros_ns - self._capture_ros_ns) >=
                int(self.gate.config.max_frame_age_s * 1e9)):
            decision = self.gate.invalidate("source detection frame expired", mono_ns / 1e9)
            self._observe(decision, mono_ns)
        if (self._input_state == "valid" and self._input_ns is not None and
                mono_ns - self._input_ns >= INPUT_TIMEOUT_NS):
            self._command = ZERO
            self._input_error = "body command receipt expired"
            self._input_state = "expired"
        return True

    def receive_command(self, command, *, ros_ns, mono_ns):
        if self._begin(ros_ns, mono_ns):
            self._input_ns = mono_ns
            try:
                if not isinstance(command, BodyCommand):
                    raise ValueError("expected BodyCommand")
                values = tuple(_finite_number(getattr(command, name))
                               for name in ("vx", "vy", "wz", "vz", "wx", "wy"))
                vx, vy, wz, vz, wx, wy = values
                if not (-0.3 <= vx <= 0.49 and abs(vy) <= 0.3 and abs(wz) <= 1.0):
                    raise ValueError("command exceeds selected walk bounds")
                if any(value != 0 for value in (vz, wx, wy)):
                    raise ValueError("unused Twist axes must be zero")
                command = BodyCommand(*values)
                self._last_valid_command = command
                self._input_error = ""
                self._input_state = "valid"
                if command == ZERO:
                    if mono_ns > self._hold_since_ns:
                        self._needs_zero = False
                    self._command = ZERO
                elif not self._camera_clear:
                    self._needs_zero = True
                    self._command = ZERO
                elif not self._needs_zero and mono_ns > self._clear_since_ns:
                    self._command = command
                else:
                    self._command = ZERO
            except (TypeError, ValueError, AttributeError) as exc:
                self._command = ZERO
                self._input_error = str(exc)[:160]
                self._input_state = "invalid"
                if not self._camera_clear:
                    self._needs_zero = True  # Invalid/uncertain input cannot count as a stop.
        return self._result(mono_ns)

    def _decode_frame(self, payload):
        if not isinstance(payload, str) or len(payload) > MAX_PAYLOAD_BYTES:
            raise ValueError("detection JSON missing or oversized")
        if len(payload.encode("utf-8")) > MAX_PAYLOAD_BYTES:
            raise ValueError("detection JSON oversized")
        frame = json.loads(payload, object_pairs_hook=_unique_object)
        if not isinstance(frame, dict) or frame.get("schema") != SCHEMA:
            raise ValueError("unexpected detection schema")
        producer = frame.get("producer_instance")
        if (not isinstance(producer, str) or not 1 <= len(producer) <= 128 or
                not all(c.isascii() and (c.isalnum() or c in "-_") for c in producer)):
            raise ValueError("invalid producer instance")
        if self._producer is not None and producer != self._producer:
            self._fault = "detector producer changed; restart guard"
            raise ValueError(self._fault)
        if (frame.get("format") != BOX_FORMAT or type(frame.get("image_width")) is not int or
                frame["image_width"] != 700 or type(frame.get("image_height")) is not int or
                frame["image_height"] != 572 or
                _finite_number(frame.get("horizontal_fov_deg")) != 220):
            raise ValueError("unexpected detection projection")
        if type(frame.get("valid")) is not bool or type(frame.get("healthy")) is not bool:
            raise ValueError("valid and healthy must be explicit booleans")
        seq, stamp = frame.get("sequence"), frame.get("captured_at_ros_ns")
        if type(seq) is not int or not 0 <= seq < 2**63 or type(stamp) is not int or not 0 < stamp < 2**63:
            raise ValueError("invalid detection sequence or integer capture timestamp")
        items = frame.get("detections")
        if not isinstance(items, list) or len(items) > MAX_DETECTIONS:
            raise ValueError("invalid or oversized detections")
        detections = []
        for item in items:
            if not isinstance(item, dict) or not isinstance(item.get("box"), list):
                raise ValueError("invalid detection entry")
            label = item.get("class_id")
            if not isinstance(label, str) or len(label) > 128:
                raise ValueError("invalid or oversized class label")
            detection = Detection(tuple(item["box"]), item.get("confidence"), label)
            ForwardObstacleGate._validate_detection(detection)
            detections.append(detection)
        return frame, producer, seq, stamp, tuple(detections)

    def receive_frame(self, payload, *, ros_ns, mono_ns):
        self._ignored_ball_count = 0
        if self._begin(ros_ns, mono_ns):
            now = mono_ns / 1e9
            try:
                frame, producer, seq, stamp, detections = self._decode_frame(payload)
                if stamp > ros_ns:
                    raise ValueError("capture timestamp is in the future")
                if seq <= self._sequence or stamp <= self._capture_ros_ns:
                    raise ValueError("repeated or regressed detection identity")
                # Consume even stale/unhealthy metadata; later edits to the same
                # identity are not distinct clearance evidence.
                self._producer, self._sequence, self._capture_ros_ns = producer, seq, stamp
                # ROS-minus-monotonic offset can range from anchor-50 ms to
                # anchor+50 ms before a sampled fault. Map against the maximum
                # admitted offset, not arrival offset: a +40 -> -40 ms change
                # must not make a delayed source capture 80 ms younger.
                captured_mono_ns = self._anchor[1] + (stamp - self._anchor[0]) - CLOCK_JUMP_NS
                if captured_mono_ns < self._anchor[1]:
                    raise ValueError("capture predates guard clock history; require newer frame")
                self._frame_mono_ns = captured_mono_ns
                if self._arena_ball_contact:
                    if frame.get("detector_identity") != ARENA_DETECTOR_IDENTITY:
                        raise ValueError("arena contact requires selected detector identity")
                    # Validate the complete frame before applying the exception.
                    # Non-ball overlap is retained; no phase or target is inferred.
                    if not frame["valid"] or not frame["healthy"]:
                        raise ValueError("detector reports an invalid or unhealthy frame")
                    if max(mono_ns - captured_mono_ns, ros_ns - stamp) >= int(
                            self.gate.config.max_frame_age_s * 1e9):
                        raise ValueError("source detection frame expired on arrival")
                    self._ignored_ball_count = sum(d.class_id == "32" for d in detections)
                    detections = tuple(d for d in detections if d.class_id != "32")
                decision = self.gate.update(DetectionFrame(
                    seq, captured_mono_ns / 1e9, detections,
                    frame["valid"], frame["healthy"], frame["format"]), now)
            except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
                decision = self.gate.invalidate(str(exc)[:160], now)
            self._observe(decision, mono_ns)
        return self._result(mono_ns)

    def evaluate(self, *, ros_ns, mono_ns):
        self._begin(ros_ns, mono_ns)
        return self._result(mono_ns)

    def _result(self, mono_ns):
        # Invalid clocks can be arbitrary Python values in a pure-core caller.
        safe_now = mono_ns if type(mono_ns) is int and 0 <= mono_ns < 2**63 else (
            self._last_clocks[1] if self._last_clocks else 0)
        decision = self.gate.evaluate(safe_now / 1e9)
        command = ZERO
        if self._fault:
            state, reason = "fault", self._fault
        elif not decision.permitted:
            state, reason = "hold", decision.reason
        elif self._needs_zero:
            state, reason = "hold", "post-hold upstream zero required before fresh command"
        elif self._input_error:
            state, reason = "hold", self._input_error
        elif self._command == ZERO:
            state, reason = "ready", "clear; zero or awaiting command received after clearance"
        else:
            state, reason, command = "passing", "fresh bounded command and camera clearance", self._command

        def age(stamp):
            return None if stamp is None else round(max(0, safe_now - stamp) / 1e9, 6)

        frame_age = None if self._frame_mono_ns is None else round(max(
            0, safe_now - self._frame_mono_ns,
            (self._last_clocks[0] if self._last_clocks else 0) - self._capture_ros_ns,
        ) / 1e9, 6)
        return GuardResult(command, {
            "sampled_at_monotonic_ns": safe_now,
            "contact_policy": "arena_ball_contact" if self._arena_ball_contact else "all_classes",
            "ignored_ball_count": self._ignored_ball_count if (
                decision.state in ("clear", "blocked") and not self._fault) else 0,
            "state": state, "reason": reason[:200], "input_age_s": age(self._input_ns),
            "input_state": self._input_state, "camera_state": decision.state,
            "camera_permitted": decision.permitted and not self._fault,
            "max_frame_age_s": self.gate.config.max_frame_age_s,
            "frame_age_s": frame_age, "producer_instance": self._producer,
            "sequence": None if self._sequence < 0 else self._sequence,
            "clear_frames": decision.clear_frames, "requires_post_hold_zero": self._needs_zero,
            "command": [command.vx, command.vy, command.wz],
        })


class DeadlineBodyCommandGuard:
    """Original-source ordering around the existing camera/clock gate.

    The caller serializes callbacks. Camera parsing and projection remain owned
    by BodyCommandGuard; no receipt-based Twist admission is used in this mode.
    """

    def __init__(self, obstacle_config=ObstacleConfig(), *, arena_ball_contact=False,
                 clock_id=None, started_mono_ns):
        # Keep legacy source subsets independent of the strict-mode helper.
        from final_project.behaviors import command_deadline
        self.protocol = command_deadline
        self.clock_id = command_deadline.local_clock_id() if clock_id is None else clock_id
        if (type(self.clock_id) is not str or not self.clock_id or
                len(self.clock_id) > 128 or not self.clock_id.isascii()):
            raise ValueError("invalid local command clock identity")
        if type(started_mono_ns) is not int or not 0 <= started_mono_ns < 2**63:
            raise ValueError("invalid command guard startup timestamp")
        self.started_mono_ns = started_mono_ns
        self.camera = BodyCommandGuard(obstacle_config, arena_ball_contact=arena_ball_contact)
        self.source_instance = None
        self._seq = self._issued_ns = -1
        self._origin = None
        self._vetoed = True
        self._receipt_ns = None
        self._camera_clear = False
        self._clearance_ns = None
        self._zero_seq = self._zero_issued_ns = None
        self._input_state = "missing"
        self._input_error = "fresh source zero required after camera clearance"
        self._last_rejection = None

    def _observe_camera(self, result, mono_ns):
        clear = result.status["camera_permitted"]
        if not clear or not self._camera_clear:
            self._zero_seq = self._zero_issued_ns = None
            self._vetoed = True
            self._clearance_ns = mono_ns if clear else None
        self._camera_clear = clear

    def _invalidate(self, reason, state="invalid"):
        self._vetoed = True
        self._input_state, self._input_error = state, reason[:160]

    def _finish(self, camera_result, mono_ns):
        status = dict(camera_result.status)
        now = status["sampled_at_monotonic_ns"]
        origin = self._origin
        if origin is not None and (not origin.fresh(now) or
                self._receipt_ns is None or now - self._receipt_ns >= INPUT_TIMEOUT_NS):
            self._invalidate("original source command expired", "expired")
        envelope = origin.veto() if origin is not None else None
        command = ZERO
        if status["state"] == "fault":
            state, reason = "fault", status["reason"]
        elif not self._camera_clear:
            state, reason = "hold", status["reason"]
        elif self._input_error:
            state, reason = "hold", self._input_error
        elif self._zero_seq is None:
            state, reason = "hold", "fresh source zero required after camera clearance"
        elif origin is None or self._vetoed:
            state, reason = "hold", "source command vetoed; require a newer source sequence"
        else:
            envelope = origin
            command = BodyCommand(origin.vx, origin.vy, origin.wz)
            state = "ready" if origin.is_zero else "passing"
            reason = "fresh source zero and camera clearance" if origin.is_zero else "original deadline and camera clearance"
        status.update(
            state=state, reason=reason, command=[command.vx, command.vy, command.wz],
            command_contract=DEADLINE_CONTRACT, clock_id=self.clock_id,
            input_state=self._input_state,
            input_age_s=None if self._receipt_ns is None else round(max(0, now - self._receipt_ns) / 1e9, 6),
            requires_post_hold_zero=self._zero_seq is None,
            source_bound=self.source_instance is not None, source_instance=self.source_instance,
            source_seq=None if self._seq < 0 else self._seq,
            source_issued_mono_ns=None if origin is None else origin.issued_mono_ns,
            source_expires_mono_ns=None if origin is None else origin.expires_mono_ns,
            source_vetoed=envelope is None or not envelope.allowed,
            source_age_s=None if origin is None else round(max(0, now - origin.issued_mono_ns) / 1e9, 6),
            clearance_mono_ns=self._clearance_ns, post_clear_zero_seq=self._zero_seq,
            last_rejection=self._last_rejection,
        )
        return GuardResult(command, status, envelope)

    def receive_command(self, payload, *, ros_ns, mono_ns):
        camera_result = self.camera.evaluate(ros_ns=ros_ns, mono_ns=mono_ns)
        self._observe_camera(camera_result, mono_ns)
        if camera_result.status["state"] == "fault":
            return self._finish(camera_result, mono_ns)
        try:
            candidate = self.protocol.decode(payload)
        except (ValueError, TypeError) as exc:
            self._invalidate(str(exc))
            return self._finish(camera_result, mono_ns)
        if candidate.clock_id != self.clock_id:
            self._last_rejection = "unrecognized command clock"
        elif self.source_instance is not None and candidate.source_instance != self.source_instance:
            self._last_rejection = "unrecognized source instance; restart guard"
        elif candidate.issued_mono_ns <= self.started_mono_ns:
            self._last_rejection = "source command predates guard startup"
        elif self.source_instance is not None and candidate.seq < self._seq:
            self._last_rejection = "older source sequence"
        elif candidate.issued_mono_ns > mono_ns:
            self._invalidate("source command issue time is future")
        elif self.source_instance is not None and candidate.seq == self._seq:
            if candidate.origin != self._origin.origin or not candidate.allowed:
                self._invalidate("conflicting or vetoed source request at the same sequence")
            # Identical duplicates cannot refresh receipt, undo a veto, or
            # repeat a consumed source-zero/clearance handshake.
        elif not candidate.allowed:
            self._invalidate("source request must have allowed=true")
        elif self.source_instance is None and (not candidate.is_zero or not candidate.fresh(mono_ns)):
            self._last_rejection = "first source command must be a fresh genuine zero"
        elif candidate.issued_mono_ns <= self._issued_ns:
            # Retire the newer sequence without lowering the issue highwater.
            # Forward its veto so downstream ordering cannot retain an older
            # positive or accept an edited retry of this retired identity.
            self._seq = candidate.seq
            self._origin, self._receipt_ns = candidate, mono_ns
            self._invalidate("source issue time did not increase")
        else:
            self.source_instance = candidate.source_instance
            self._seq, self._issued_ns = candidate.seq, candidate.issued_mono_ns
            self._origin, self._receipt_ns = candidate, mono_ns
            self._vetoed = True
            self._input_state, self._input_error = "valid", ""
            self._last_rejection = None
            if not candidate.fresh(mono_ns):
                self._invalidate("original source command expired", "expired")
            elif self._camera_clear and candidate.issued_mono_ns > self._clearance_ns:
                if candidate.is_zero:
                    self._zero_seq, self._zero_issued_ns = candidate.seq, candidate.issued_mono_ns
                    self._vetoed = False
                elif self._zero_seq is not None and candidate.issued_mono_ns > self._zero_issued_ns:
                    self._vetoed = False
        return self._finish(camera_result, mono_ns)

    def receive_frame(self, payload, *, ros_ns, mono_ns):
        result = self.camera.receive_frame(payload, ros_ns=ros_ns, mono_ns=mono_ns)
        self._observe_camera(result, mono_ns)
        return self._finish(result, mono_ns)

    def evaluate(self, *, ros_ns, mono_ns):
        result = self.camera.evaluate(ros_ns=ros_ns, mono_ns=mono_ns)
        self._observe_camera(result, mono_ns)
        return self._finish(result, mono_ns)

    def veto(self, *, ros_ns, mono_ns):
        result = self.camera.evaluate(ros_ns=ros_ns, mono_ns=mono_ns)
        self._observe_camera(result, mono_ns)
        self._invalidate("guard shutdown")
        return self._finish(result, mono_ns)


def create_ros_node(*, monotonic_ns=time.monotonic_ns, arena_ball_contact=False,
                    command_contract=LEGACY_CONTRACT, clock_id=None):
    """Import ROS only here; default executor serializes all guard callbacks."""
    if command_contract not in (LEGACY_CONTRACT, DEADLINE_CONTRACT):
        raise ValueError("unknown command contract")
    strict = command_contract == DEADLINE_CONTRACT
    if not strict:
        from geometry_msgs.msg import Twist
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.node import Node
    from std_msgs.msg import String

    class BodyCommandGuardNode(Node):
        def __init__(self):
            super().__init__("fc_body_command_guard")
            self.guard = (DeadlineBodyCommandGuard(arena_ball_contact=arena_ball_contact,
                          clock_id=clock_id, started_mono_ns=monotonic_ns()) if strict else
                          BodyCommandGuard(arena_ball_contact=arena_ball_contact))
            self.command_pub = self.create_publisher(
                String if strict else Twist,
                self.guard.protocol.ADMITTED_TOPIC if strict else OUTPUT_TOPIC, 1)
            self.status_pub = self.create_publisher(String, STATUS_TOPIC, 1)
            self._last_status_ns = None
            self.guard.evaluate(**self.clocks())  # Establish this process's clock-history boundary.
            self.create_subscription(String if strict else Twist,
                self.guard.protocol.REQUEST_TOPIC if strict else INPUT_TOPIC, self.on_command, 1)
            self.create_subscription(String, FRAME_TOPIC, self.on_frame, 1)
            self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(TIMER_S, self.on_tick, clock=self._steady_clock)

        def clocks(self):
            return {"ros_ns": self.get_clock().now().nanoseconds, "mono_ns": monotonic_ns()}

        def publish_result(self, result, mono_ns):
            if strict:
                if result.envelope is not None:
                    self.command_pub.publish(String(data=result.envelope.encode()))
            else:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = result.command.vx, result.command.vy, result.command.wz
                self.command_pub.publish(msg)
            if self._last_status_ns is None or mono_ns - self._last_status_ns >= 200_000_000:
                self.status_pub.publish(String(data=json.dumps(result.status, separators=(",", ":"), allow_nan=False)))
                self._last_status_ns = mono_ns

        def on_command(self, msg):
            clocks = self.clocks()
            cmd = msg.data if strict else BodyCommand(msg.linear.x, msg.linear.y, msg.angular.z,
                                                    msg.linear.z, msg.angular.x, msg.angular.y)
            self.publish_result(self.guard.receive_command(cmd, **clocks), clocks["mono_ns"])

        def on_frame(self, msg):
            clocks = self.clocks()
            self.publish_result(self.guard.receive_frame(msg.data, **clocks), clocks["mono_ns"])

        def on_tick(self):
            clocks = self.clocks()
            self.publish_result(self.guard.evaluate(**clocks), clocks["mono_ns"])

        def publish_zero(self):
            if strict:
                clocks = self.clocks()
                self.publish_result(self.guard.veto(**clocks), clocks["mono_ns"])
            else:
                self.command_pub.publish(Twist())

    return BodyCommandGuardNode()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arena-ball-contact", action="store_true",
                        help="allow selected COCO sports-ball contact; every other class still gates")
    parser.add_argument("--command-contract", choices=(LEGACY_CONTRACT, DEADLINE_CONTRACT),
                        default=LEGACY_CONTRACT, help="explicit source-deadline transport; default preserves Twist")
    options, ros_args = parser.parse_known_args(argv)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation

    rclpy.init(args=ros_args)
    selection = {"arena_ball_contact": options.arena_ball_contact}
    if options.command_contract != LEGACY_CONTRACT:
        selection["command_contract"] = options.command_contract
    node = create_ros_node(**selection)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # SIGTERM can invalidate the context during a callback's publication.
        # An RCL failure while the context is live remains an application error.
        if rclpy.ok():
            raise
    finally:
        primary_error = sys.exc_info()[1]
        try:
            try:
                if rclpy.ok():
                    node.publish_zero()
            except rclpy_implementation.RCLError:
                # The context may shut down after the check above. No delivery
                # is claimed when this final publication cannot complete.
                if rclpy.ok():
                    raise
            finally:
                try:
                    node.destroy_node()
                finally:
                    rclpy.try_shutdown()
        except BaseException as cleanup_error:
            if primary_error is not None:
                # Keep the original failure visible if cleanup also fails.
                raise primary_error from cleanup_error
            raise


if __name__ == "__main__":
    main()
