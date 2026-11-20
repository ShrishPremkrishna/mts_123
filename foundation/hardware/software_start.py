"""Resident caller for the existing deliberate Start gateway; never homes.

Capture the already-observed challenge at operator-input receipt, then submit it
once. Discovery, reconnects and newer status cannot refresh an earlier press.
The gateway still owns the authoritative epoch, Joy freshness and controller
inventory checks. Acknowledgement is not physical readiness.
"""
from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass

STATUS_MAX_AGE_S = 0.2
ACK_TIMEOUT_S = 4.0
STOP_TOPIC = "/estop_controller/request_stop"
STOP_CONFIRM_TIMEOUT_S = 1.0
BLOCKED = ("fault", "stop_required", "in_flight", "unknown_remote_submission", "shutting_down")


@dataclass(frozen=True)
class StartTicket:
    instance_id: str
    epoch: int
    sequence: int
    selected_controller: str
    observed_at: float
    pressed_at: float


class ResidentStart:
    """Transport-injected state; no calls happen on status, capture or poll retry."""

    def __init__(self, submit, ready, *, clock=time.monotonic,
                 selected_controller="neural_controller_default", gateway_stop=None,
                 emergency_stop=None):
        self.submit, self.ready, self.clock = submit, ready, clock
        self.selected_controller = selected_controller
        self.lock = threading.RLock()
        self.state = None
        self.observed_at = None
        self.highwater = None
        self.consumed = set()
        self.pending = None
        self.uncertain = False
        self.gateway_stop, self.emergency_stop = gateway_stop, emergency_stop
        self.last_request = None
        self.stop_latched = False
        self.stop_request = self.stop_result = None
        self._stop_pending = None

    @property
    def stop_pending(self):
        return self._stop_pending is not None

    @property
    def stop_capable(self):
        return (self.state is not None and self.state.get("software_stop_topic") == STOP_TOPIC and
                callable(self.gateway_stop) and callable(self.emergency_stop))

    def observe(self, raw):
        """Malformed/reordered status invalidates readiness, never refreshes it."""
        with self.lock:
            try:
                state = json.loads(raw) if isinstance(raw, str) else dict(raw)
                if not isinstance(state, dict):
                    raise ValueError("gateway status must be an object")
                if state.get("protocol") != "pupper_stop_v1":
                    raise ValueError("unsupported gateway protocol")
                for key in ("instance_id", "selected_controller"):
                    if not isinstance(state.get(key), str) or not state[key]:
                        raise ValueError("missing gateway identity")
                for key in ("epoch", "sequence"):
                    if type(state.get(key)) is not int or not 0 <= state[key] < 2**64:
                        raise ValueError("invalid gateway counter")
                for key in (*BLOCKED, "latched"):
                    if type(state.get(key)) is not bool:
                        raise ValueError("invalid gateway flag")
                if self.highwater and state["instance_id"] == self.highwater[0]:
                    if state["sequence"] <= self.highwater[1] or state["epoch"] < self.highwater[2]:
                        raise ValueError("reordered gateway status")
                now = self.clock()
                if not math.isfinite(now):
                    raise ValueError("invalid local clock")
            except Exception:
                self.state, self.observed_at = None, None
                return False
            self.state, self.observed_at = state, now
            self.highwater = (state["instance_id"], state["sequence"], state["epoch"])
            return True

    def _problem(self, now):
        if self.stop_latched:
            return "local Stop is latched; no further Start is permitted"
        if self.uncertain:
            return "previous Start result uncertain; inspect gateway, do not retry automatically"
        if self.pending is not None:
            return "Start acknowledgement pending"
        try:
            available = self.ready()
        except Exception:
            available = False
        if not available:
            return "Start service unavailable; no action queued"
        if (self.state is None or not math.isfinite(now) or
                not 0 <= now - self.observed_at <= STATUS_MAX_AGE_S):
            return "no fresh observed gateway status"
        if not self.stop_capable:
            return "gateway lacks supported software Stop cancellation; use Options only"
        if any(self.state[key] for key in BLOCKED):
            return "gateway has pending work, Stop cleanup or a fault"
        if self.state["selected_controller"] != self.selected_controller:
            return "selected controller differs from this session"
        return None

    def capture(self, *, pressed_at=None):
        """Call at input receipt, not after queueing/discovery or on reconnect."""
        with self.lock:
            now = self.clock()
            pressed_at = now if pressed_at is None else pressed_at
            if (isinstance(pressed_at, bool) or not isinstance(pressed_at, (int, float))
                    or not math.isfinite(pressed_at) or not 0 <= now - pressed_at <= STATUS_MAX_AGE_S):
                raise ValueError("Start input receipt expired or invalid")
            problem = self._problem(now)
            if problem:
                raise ValueError(problem)
            if self.observed_at > pressed_at:
                raise ValueError("gateway status was not observed at Start input receipt")
            state = self.state
            return StartTicket(*(state[k] for k in
                ("instance_id", "epoch", "sequence", "selected_controller")), self.observed_at, pressed_at)

    def send(self, ticket):
        """Submit the frozen ticket once; a failure never submits another call."""
        with self.lock:
            now = self.clock()
            problem = self._problem(now)
            if problem:
                raise ValueError(problem)
            if (not isinstance(ticket, StartTicket) or
                    not 0 <= now - ticket.pressed_at <= STATUS_MAX_AGE_S or
                    not 0 <= now - ticket.observed_at <= STATUS_MAX_AGE_S):
                raise ValueError("Start press expired; no action queued")
            if any(getattr(ticket, k) != self.state[k] for k in
                   ("instance_id", "epoch", "selected_controller")):
                raise ValueError("gateway changed after Start press")
            identity = (ticket.instance_id, ticket.sequence)
            if identity in self.consumed:
                raise ValueError("Start press already submitted")
            self.consumed.add(identity)
            # History can be bounded after a new gateway instance: instance IDs
            # and the current identity fence make older tickets unusable.
            self.consumed = {key for key in self.consumed if key[0] == ticket.instance_id}
            payload = {k: getattr(ticket, k) for k in
                       ("instance_id", "epoch", "sequence", "selected_controller")}
            self.last_request = dict(payload)  # retain the anchor even if transport raises
            try:
                self.pending = (self.submit(payload), now, payload)
            except Exception as exc:
                self.uncertain = True  # the transport may have submitted first
                raise ValueError("Start submission uncertain; inspect gateway before further action") from exc
            return payload

    def request_stop(self, reason, *, authority=None):
        """Latch locally and send native plus gateway Stop once; neither is an ACK."""
        with self.lock:
            if self.stop_latched:
                return self.stop_request
            self.stop_latched = True
            now = self.clock()
            # A bad/new gateway status must not replace the identity of a Start
            # that may already be queued remotely. For a no-Start Stop, use the
            # already-observed identity; a missing anchor cannot be confirmed.
            anchor = authority if authority is not None else self.last_request or self.state
            identity = anchor.get("instance_id") if anchor else None
            epoch = anchor.get("epoch") if anchor else None
            sequence = anchor.get("sequence") if anchor else None
            if self.state is not None and self.state["instance_id"] == identity:
                epoch = max(epoch, self.state["epoch"])
                sequence = max(sequence, self.state["sequence"])
            self.stop_request = {"instance_id": identity, "epoch": epoch, "sequence": sequence,
                                 "requested_at": now if math.isfinite(now) else None,
                                 "reason": str(reason)[:200]}
            errors = []
            # Keep native damping prompt even when the gateway path fails.
            for label, send in (("native", self.emergency_stop), ("gateway", self.gateway_stop)):
                try:
                    if not callable(send):
                        raise ValueError("publisher unavailable")
                    send()
                except Exception:
                    errors.append(label + " Stop publication uncertain")
            self._stop_pending = (now, self.stop_request, errors)
            return self.stop_request

    def poll_stop(self, *, force_unconfirmed=False):
        """One bounded gateway-latch result, never physical release or zero feedback."""
        with self.lock:
            if self._stop_pending is None:
                return None
            began, request, errors = self._stop_pending
            now = self.clock()
            valid_clock = math.isfinite(now) and math.isfinite(began) and now >= began
            state = self.state
            confirmed = bool(valid_clock and state is not None and request["instance_id"] is not None and
                self.observed_at is not None and began <= self.observed_at <= now and
                now - self.observed_at <= STATUS_MAX_AGE_S and
                state["instance_id"] == request["instance_id"] and state["epoch"] > request["epoch"] and
                state["sequence"] > request["sequence"] and state["latched"] is True and
                state.get("software_stop_topic") == STOP_TOPIC)
            if not confirmed and valid_clock and now - began < STOP_CONFIRM_TIMEOUT_S and not force_unconfirmed:
                return None
            self._stop_pending = None
            self.stop_result = {"event": "stop_result", "confirmed": confirmed,
                "uncertain": not confirmed or bool(errors),
                "request": dict(request), "publication_errors": errors,
                "gateway_epoch": state["epoch"] if confirmed else None,
                "message": ("matching gateway Stop latch and newer epoch observed" if confirmed else
                            "gateway Stop unconfirmed at deadline or local shutdown; inspect gateway before further action"),
                "scope": "gateway latch acknowledgement only; not backend deactivation, policy zero or physical stopping"}
            return self.stop_result

    def poll(self):
        """Return one correlated result, or None; timeout never retries/cancels Stop."""
        with self.lock:
            if self.pending is None:
                return None
            future, began, payload = self.pending
            try:
                # A correlated response already received is an acknowledgement
                # even if this consumer was scheduled late. Once a prior poll
                # declared uncertainty, pending is cleared and late replies
                # cannot undo it. Success never proves current motion readiness.
                if not future.done():
                    now = self.clock()
                    if math.isfinite(now) and 0 <= now - began < ACK_TIMEOUT_S:
                        return None
                    raise TimeoutError("Start acknowledgement unavailable at deadline")
                response = future.result()
                if type(response.success) is not bool or not isinstance(response.message, str):
                    raise ValueError("malformed Start acknowledgement")
                self.pending = None
                return {"event": "start_result", "success": response.success,
                        "uncertain": False, "message": response.message, "request": payload}
            except Exception:
                self.pending = None
                self.uncertain = True
                return {"event": "start_result", "success": False, "uncertain": True,
                        "message": "Start result uncertain; inspect gateway; no automatic retry",
                        "request": payload}


def attach(node, *, clock=time.monotonic, selected_controller="neural_controller_default", emergency_stop=None):
    """Attach to an already resident ROS node; absence of the service is inert."""
    from joy_utils.srv import StartMotion
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Empty, String
    client = node.create_client(StartMotion, "/estop_controller/start_motion")

    def submit(payload):
        request = StartMotion.Request()
        for key, value in payload.items():
            setattr(request, key, value)
        return client.call_async(request)

    stop_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    gateway = node.create_publisher(Empty, STOP_TOPIC, stop_qos)
    native = None if emergency_stop is not None else node.create_publisher(Empty, "/emergency_stop", stop_qos)
    caller = ResidentStart(submit, client.service_is_ready, clock=clock,
        selected_controller=selected_controller, gateway_stop=lambda: gateway.publish(Empty()),
        emergency_stop=emergency_stop if emergency_stop is not None else lambda: native.publish(Empty()))
    caller.stop_publisher, caller.emergency_publisher = gateway, native
    caller.client = client
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    caller.subscription = node.create_subscription(String, "/estop_controller/stop_status",
                                                    lambda message: caller.observe(message.data), qos)
    return caller
