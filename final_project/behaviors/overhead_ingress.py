#!/usr/bin/env python3
"""Measured fieldcam UDP -> same-Pi /fc/normalized_events (ROS String).

Default: bounded observation only. --publish requires explicit source process,
calibration, geometry, robot names, producer and operator epoch. It emits frames
only; no arm, intent, activation, motor initialization or command publisher.

Clock bounds use a nonce-bound four-timestamp round trip. The conservative lower
bound dates each original capture, including network delay, pairing uncertainty
and a declared 500 ppm drift allowance. Expired/wide samples fail closed. The
source's capture is its host camera-delivery timestamp minus the configured
latency adjustment, not a measured sensor exposure timestamp; this adapter
cannot measure camera/driver buffering.
Source IDs are operational pins on a trusted network, not authentication.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import socket
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from final_project.overhead_perception.fieldcam.net import (
    CLOCK_DRIFT, WALL_STEP_S, MAX_DRAIN, STATE_PROTOCOL, PROBE_PROTOCOL, REPLY_PROTOCOL)

EVENT_TOPIC = "/fc/normalized_events"
MAX_SYNC_WIDTH_S = .020
SYNC_TTL_S = 2.
PROBE_PERIOD_S = .5
MAX_FRAME_AGE_S = .30
ROBOT_IDS = {"pupper-1", "pupper-2"}


def number(value, label):
    if type(value) not in (int, float):
        raise ValueError(label + " must be finite")
    try:
        result = float(value)
    except OverflowError:
        raise ValueError(label + " must be finite") from None
    if not math.isfinite(result):
        raise ValueError(label + " must be finite")
    return result


def token(value, label):
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ValueError(label + " must be a nonempty bounded string")
    return value


class IngressCore:
    """Pure admission. ``now`` always belongs to this Pi's monotonic clock."""

    def __init__(self, *, source_id, source_session, producer_id, epoch, boot_id,
                 robot_map, calibration_revision, field):
        self.source_id = token(source_id, "source_id")
        self.source_session = token(source_session, "source_session")
        self.source_clock = "monotonic:" + self.source_session
        self.producer_id = token(producer_id, "producer_id")
        self.epoch = token(epoch, "epoch")
        self.clock_name = "linux-monotonic:" + token(boot_id, "boot_id")
        if (not isinstance(robot_map, dict) or not robot_map
                or not set(robot_map.values()) <= ROBOT_IDS
                or len(set(robot_map.values())) != len(robot_map)):
            raise ValueError("robot map must bind distinct source names to configured FC robot IDs")
        for name in robot_map:
            token(name, "robot name")
        if type(calibration_revision) is not int or calibration_revision < 1:
            raise ValueError("pin a positive calibration revision")
        self.robot_map = dict(robot_map)
        self.calibration_revision = calibration_revision
        self.field = {k: number(field[k], k) for k in ("length", "width", "goal_width", "ball_radius")}
        if any(v <= 0 for v in self.field.values()):
            raise ValueError("field dimensions must be positive")
        self.pending = None
        self.sync = None
        self.wire_seq = self.source_seq = self.output_seq = -1
        self.last_pair = None
        self.last_capture = None
        self.poisoned = False
        self.held = True
        self.last_reason = "waiting for measured clock sync and a live frame"

    def identity(self):
        return {"source_id": self.source_id, "source_session": self.source_session,
                "clock_id": self.source_clock}

    def _identity(self, packet):
        if not isinstance(packet, dict) or any(packet.get(k) != v for k, v in self.identity().items()):
            raise ValueError("source identity/session/clock differs")

    def make_probe(self, now):
        now = number(now, "probe clock")
        nonce = uuid.uuid4().hex
        self.pending = nonce, now
        return {"protocol": PROBE_PROTOCOL, **self.identity(), "nonce": nonce}

    def accept_probe(self, reply, now):
        self._identity(reply)
        now = number(now, "reply clock")
        if (reply.get("protocol") != REPLY_PROTOCOL or self.pending is None
                or reply.get("nonce") != self.pending[0]):
            raise ValueError("unsolicited/replayed clock reply")
        _, sent = self.pending
        self.pending = None
        received = number(reply.get("received_mono"), "remote receive clock")
        remote_sent = number(reply.get("sent_mono"), "remote send clock")
        if not sent <= now or not received <= remote_sent:
            raise ValueError("clock probe timestamps reversed")
        lower, upper = sent - received, now - remote_sent
        if not 0 <= upper - lower <= MAX_SYNC_WIDTH_S or now - sent > MAX_SYNC_WIDTH_S:
            raise ValueError("clock round trip uncertainty exceeds 20 ms")
        if self.sync is not None:
            old_lower, old_upper, at = self.sync
            drift = max(0., now - at) * CLOCK_DRIFT
            if upper < old_lower - drift or lower > old_upper + drift:
                self.poisoned = True
                raise ValueError("source clock mapping discontinuity; explicitly rebind session")
        self.sync = lower, upper, now

    def _offset(self, now):
        if self.poisoned:
            raise ValueError("source clock or calibration changed; explicitly rebind session")
        if self.sync is None or not 0 <= now - self.sync[2] <= SYNC_TTL_S:
            raise ValueError("clock sync absent or expired")
        lower, upper, at = self.sync
        drift = (now - at) * CLOCK_DRIFT
        if upper - lower + 2 * drift > MAX_SYNC_WIDTH_S:
            raise ValueError("clock sync uncertainty exceeded")
        return lower - drift, upper + drift

    def _geometry(self, state):
        incoming = state.get("field", {})
        if not isinstance(incoming, dict):
            raise ValueError("field must be an object")
        for key in ("length", "width", "ball_radius"):
            if abs(number(incoming.get(key), "field." + key) - self.field[key]) > 1e-6:
                raise ValueError("field geometry differs from pinned executor configuration")
        goals = state.get("goals", {})
        if not isinstance(goals, dict):
            raise ValueError("goals must be an object")
        for name, sign in (("west", -1), ("east", 1)):
            goal = goals.get(name, {})
            if not isinstance(goal, dict):
                raise ValueError("each goal must be an object")
            for key, expected in (("x", sign * self.field["length"] / 2),
                                  ("y_min", -self.field["goal_width"] / 2),
                                  ("y_max", self.field["goal_width"] / 2)):
                if abs(number(goal.get(key), "goal." + key) - expected) > 1e-6:
                    raise ValueError("goal geometry differs from pinned executor configuration")
        return {"left": {"x": -self.field["length"] / 2, "y": 0.},
                "right": {"x": self.field["length"] / 2, "y": 0.}}

    def _event(self, capture, robots, ball, goals):
        self.output_seq += 1
        return {"event": "frame", "producer_id": self.producer_id,
                "clock": self.clock_name, "epoch": self.epoch, "seq": self.output_seq,
                "captured_at": capture, "robots": robots, "ball": ball, "goals": goals}

    def revoke(self, reason):
        """Deliberately invalid capture clears Executor's frame before validation.

        Never invent a receipt-time capture. Rejection is expected and leaves
        operator arm/Stop authority untouched. Emit once per contiguous fault.
        """
        self.last_reason = str(reason)[:200]
        if self.held:
            return None
        self.held = True
        return self._event(None, {}, None, {})

    def expire(self, now):
        try:
            self._offset(now)
            if self.last_capture is None or now - self.last_capture >= MAX_FRAME_AGE_S:
                raise ValueError("source frame expired")
        except ValueError as exc:
            return self.revoke(exc)
        return None

    def normalize(self, state, now):
        if not isinstance(state, dict):
            raise ValueError("state must be an object")
        now = number(now, "local clock")
        transport = state.get("_transport")
        self._identity(transport)
        if transport.get("protocol") != STATE_PROTOCOL or transport.get("live") is not True:
            raise ValueError("only explicitly live fieldcam state can enter FC")
        if transport.get("clock_valid") is not True:
            self.poisoned = True
            raise ValueError("source wall clock stepped or sampling failed")
        lower, upper = self._offset(now)
        wire, seq = transport.get("wire_seq"), state.get("seq")
        if type(wire) is not int or type(seq) is not int or wire <= self.wire_seq or seq <= self.source_seq:
            raise ValueError("duplicate/reordered source frame")
        self.wire_seq, self.source_seq = wire, seq
        mono = number(transport.get("sent_mono"), "source monotonic")
        wall = number(transport.get("sent_wall"), "source wall")
        span = number(transport.get("pair_span_s"), "clock pair span")
        if not 0 <= span <= WALL_STEP_S:
            raise ValueError("source clock pair uncertainty exceeded")
        if self.last_pair is not None:
            old_mono, old_wall, old_span = self.last_pair
            elapsed = mono - old_mono
            if (elapsed <= 0 or abs((wall - old_wall) - elapsed)
                    > WALL_STEP_S + span + old_span + max(0., elapsed) * CLOCK_DRIFT):
                self.poisoned = True
                raise ValueError("source clock pair stepped/backtracked")
        self.last_pair = mono, wall, span
        capture = number(state.get("captured_at"), "original capture")
        stamp = number(state.get("stamp"), "state prediction timestamp")
        age = number(state.get("frame_age_s"), "frame age")
        start_wall = number(transport.get("clock_start_wall"), "session clock start")
        if not start_wall <= capture <= stamp <= wall + span or not 0 <= age <= MAX_FRAME_AGE_S:
            raise ValueError("source capture predates session, is future or stale")
        if abs((stamp - capture) - age) > 1e-6:
            raise ValueError("capture and frame age disagree")
        # Wall sample occurs within [mono, mono+span]; subtract uncertainty and
        # drift from the oldest possible source capture and network offset.
        capture_mono = mono + capture - wall
        mapped = capture_mono + lower - WALL_STEP_S - (wall - capture) * CLOCK_DRIFT
        uncertainty = upper - lower + span + 2 * WALL_STEP_S + 2 * (wall - capture) * CLOCK_DRIFT
        if uncertainty > MAX_SYNC_WIDTH_S:
            raise ValueError("combined capture clock uncertainty exceeds 20 ms")
        if (not 0 <= now - mapped <= MAX_FRAME_AGE_S
                or (self.last_capture is not None and mapped <= self.last_capture)):
            raise ValueError("mapped capture is stale, future or reordered")
        revision = state.get("calibration_revision")
        if type(revision) is not int or revision != self.calibration_revision:
            self.poisoned = True
            raise ValueError("calibration revision changed; explicitly rebind session")
        if state.get("calibration_valid") is not True:
            raise ValueError("calibration invalid")
        goals = self._geometry(state)

        def measured(row, heading=False):
            if not isinstance(row, dict) or row.get("visible") is not True or row.get("measured_this_frame") is not True:
                return None
            if number(row.get("measured_at"), "track measurement") != capture:
                return None  # Never promote a coasting track to this frame.
            if abs(number(row.get("age_s"), "track age") - age) > 1e-6:
                raise ValueError("track age disagrees with original measurement")
            result = {}
            for axis, velocity in (("x", "vx"), ("y", "vy")):
                speed = number(row.get(velocity), velocity)
                result[axis] = number(row.get(axis), axis) - speed * age
                if not heading:
                    result[velocity] = speed
            if heading:
                # Current producers retain capture heading explicitly. Legacy
                # color pairs already reported capture yaw; only legacy tags
                # extrapolated it to the publication time.
                if "yaw_at_capture" in row:
                    yaw = number(row["yaw_at_capture"], "capture yaw")
                else:
                    yaw = number(row.get("yaw"), "yaw")
                    tracking = state.get("tracking", "tags")
                    if tracking not in ("tags", "colors"):
                        raise ValueError("legacy heading convention is unknown")
                    if tracking == "tags":
                        yaw -= number(row.get("wz"), "wz") * age
                result.update(yaw=math.atan2(math.sin(yaw), math.cos(yaw)), captured_at=mapped)
            return result

        source_robots = state.get("robots")
        if not isinstance(source_robots, dict):
            raise ValueError("robots must be an object")
        robots = {}
        for source, target in self.robot_map.items():
            pose = measured(source_robots.get(source), heading=True)
            if pose is not None:
                robots[target] = pose
        ball = measured(state.get("ball"))
        event = self._event(mapped, robots, ball, goals)
        event["source"] = {**self.identity(), "seq": seq, "calibration_revision": revision,
                           "clock_uncertainty_s": uncertainty}
        self.last_capture, self.held, self.last_reason = mapped, False, "admitted live frame"
        return event


class UDPIngress:
    """Bounded socket service; source address and port are pinned after identity."""

    def __init__(self, core, *, port=0, bind="0.0.0.0", source_host, clock=time.monotonic):
        self.core, self.clock = core, clock
        self.source_host = socket.gethostbyname(source_host)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, port))
        self.sock.setblocking(False)
        self.peer = None
        self.next_probe = 0.

    def poll(self):
        events = []
        for _ in range(MAX_DRAIN):
            try:
                raw, address = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            now = self.clock()
            if address[0] != self.source_host:
                continue
            try:
                packet = json.loads(raw)
                if not isinstance(packet, dict):
                    raise ValueError("packet must be an object")
                if packet.get("protocol") == REPLY_PROTOCOL:
                    if address == self.peer:
                        self.core.accept_probe(packet, now)
                    continue
                self.core._identity(packet.get("_transport"))
                if self.peer is None:
                    self.peer = address
                if address != self.peer:
                    raise ValueError("source UDP address changed; explicitly rebind")
                events.append(self.core.normalize(packet, now))
            except (ValueError, TypeError, KeyError, UnicodeError) as exc:
                event = self.core.revoke(exc)
                if event is not None:
                    events.append(event)
        now = self.clock()
        if self.peer is not None and now >= self.next_probe:
            self.sock.sendto(json.dumps(self.core.make_probe(now)).encode(), self.peer)
            self.next_probe = now + PROBE_PERIOD_S
        event = self.core.expire(now)
        if event is not None:
            events.append(event)
        return events

    def close(self):
        self.sock.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--publish", action="store_true")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--source-host")
    parser.add_argument("--source-id", default="fieldcam")
    parser.add_argument("--source-session")
    parser.add_argument("--producer-id")
    parser.add_argument("--epoch")
    parser.add_argument("--calibration-revision", type=int)
    parser.add_argument("--field-config", type=Path)
    parser.add_argument("--robot-map", action="append", default=[], metavar="SOURCE=FC_ID")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3600:
        parser.error("duration must be between 0 and 3600 seconds")
    if not args.publish:
        # Discovery cannot accidentally load ROS, activate hardware or mint frames.
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((args.bind, args.port))
        sock.settimeout(.2)
        end, report_at = time.monotonic() + args.duration, 0.
        try:
            while time.monotonic() < end:
                try:
                    raw, address = sock.recvfrom(65535)
                    state = json.loads(raw)
                    if isinstance(state, dict) and time.monotonic() >= report_at:
                        print(json.dumps({"mode": "observe", "address": address,
                            "transport": state.get("_transport"),
                            "calibration_revision": state.get("calibration_revision"),
                            "calibration_valid": state.get("calibration_valid"),
                            "field": state.get("field"),
                            "robots": list(state["robots"]) if isinstance(state.get("robots"), dict) else []}), flush=True)
                        report_at = time.monotonic() + 1.
                except (socket.timeout, ValueError, TypeError):
                    pass
        finally:
            sock.close()
        return 0
    required = (args.source_host, args.source_session, args.producer_id, args.epoch,
                args.calibration_revision, args.field_config, args.robot_map)
    if not all(required):
        parser.error("publish requires source-host/session, producer-id, epoch, calibration-revision, field-config and robot-map")
    try:
        mapping = dict(item.split("=", 1) for item in args.robot_map)
        if len(mapping) != len(args.robot_map):
            raise ValueError("duplicate source robot mapping")
        core = IngressCore(source_id=args.source_id, source_session=args.source_session,
            producer_id=args.producer_id, epoch=args.epoch,
            boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(), robot_map=mapping,
            calibration_revision=args.calibration_revision,
            field=json.loads(args.field_config.read_text())["field"])
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    import rclpy  # Deliberately lazy: offline observation/core need only stdlib.
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    rclpy.init(args=[])
    node = rclpy.create_node("fc_overhead_ingress")
    pub = node.create_publisher(String, EVENT_TOPIC, QoSProfile(depth=1,
        history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))
    ingress = UDPIngress(core, bind=args.bind, port=args.port, source_host=args.source_host)
    end, report_at = time.monotonic() + args.duration, 0.
    try:
        while rclpy.ok() and time.monotonic() < end:
            for event in ingress.poll():
                pub.publish(String(data=json.dumps(event, allow_nan=False)))
            rclpy.spin_once(node, timeout_sec=.005)
            if time.monotonic() >= report_at:
                print(json.dumps({"mode": "publish_frames_only", "reason": core.last_reason,
                                  "held": core.held, "seq": core.output_seq}), flush=True)
                report_at = time.monotonic() + 1.
    finally:
        event = core.revoke("ingress shutdown")
        if event is not None:
            pub.publish(String(data=json.dumps(event)))
        ingress.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
