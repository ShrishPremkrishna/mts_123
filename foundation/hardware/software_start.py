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
                 selected_controller="neural_controller_default"):
        self.submit, self.ready, self.clock = submit, ready, clock
        self.selected_controller = selected_controller
        self.lock = threading.RLock()
        self.state = None
        self.observed_at = None
        self.highwater = None
        self.consumed = set()
        self.pending = None
        self.uncertain = False

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
            try:
                self.pending = (self.submit(payload), now, payload)
            except Exception as exc:
                self.uncertain = True  # the transport may have submitted first
                raise ValueError("Start submission uncertain; inspect gateway before further action") from exc
            return payload

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


def attach(node, *, clock=time.monotonic, selected_controller="neural_controller_default"):
    """Attach to an already resident ROS node; absence of the service is inert."""
    from joy_utils.srv import StartMotion
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    client = node.create_client(StartMotion, "/estop_controller/start_motion")

    def submit(payload):
        request = StartMotion.Request()
        for key, value in payload.items():
            setattr(request, key, value)
        return client.call_async(request)

    caller = ResidentStart(submit, client.service_is_ready, clock=clock,
                           selected_controller=selected_controller)
    caller.client = client
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    caller.subscription = node.create_subscription(String, "/estop_controller/stop_status",
                                                    lambda message: caller.observe(message.data), qos)
    return caller
