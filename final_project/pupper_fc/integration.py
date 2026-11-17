"""Provisional, transport-free FC state/intent executor and JSONL replay.

This is an INTERNAL normalized contract, not Ryan's required wire format. A live
adapter must map capture/issue times into the executor's named monotonic clock
before admission; arrival time must never replace capture time. Event ``now``
is the replay/adapter's local receive time, never an upstream payload clock.
Geometry is in
the existing metric field frame. Goal points are FIXED calibrated field geometry,
not per-frame estimates: keep their values identical across frames. Any change,
even small measurement jitter, deliberately replaces the attack target and
cancels its current alignment/strike. This slice does not smooth goal geometry.
Each frame supplies synchronous robot/ball state; absent tracks stay absent.
The default dimensions/limits are sandbox assumptions, not hardware approval.

Example input (one JSON object per line; times share the explicit replay clock):
  {"event":"arm","now":0,"epoch":"trial-1","clock":"replay"}
  {"event":"frame","now":0,"epoch":"trial-1","clock":"replay","seq":1,"captured_at":0,"robots":{"pupper-1":{"x":-1,"y":0,"yaw":0},"pupper-2":{"x":1,"y":0,"yaw":3.14159}},"ball":{"x":0,"y":0},"goals":{"left":{"x":-1.83,"y":0},"right":{"x":1.83,"y":0}}}
  {"event":"intent","now":0,"epoch":"trial-1","clock":"replay","robot_id":"pupper-1","seq":1,"intent_id":"play-1","issued_at":0,"valid_until":0.3,"kind":"attack"}
  {"event":"tick","now":0.05}
  {"event":"stop","now":0.1}

Every output contains accepted/reason and decisions keyed by robot ID, e.g.
  {"accepted":true,"reason":"intent replaced","now":0.0,"armed":true,"epoch":"trial-1","decisions":{"pupper-1":{"phase":"approach","command":{"vx":0.65,"vy":0.0,"wz":0.0,"expires_at":0.15,"robot_id":"pupper-1"},"...":"other Decision fields"}}}
The example output is abbreviated; the CLI emits full dataclass fields.

``move`` and ``defend`` require target={x,y,yaw}; ``shoot`` requires target={x,y}.
``attack`` uses the configured opponent goal from the latest frame. It NEVER
substitutes a fixed goal when that observation is missing. Referee events are
optional integer ``goal`` (-1/0/+1) on each robot; the baseline goal latch stays
in force. ``fallen`` is an optional boolean. This slice has no physical get-up.

Intent seq increases independently per robot. Renew the same intent_id/kind/
target to extend its lease (at most 0.5 s from issued_at); renewal does not reset
alignment. A changed ID/kind/target cancels the old action before the next tick.
Frames do not renew intent. Duplicate/reordered packets cannot refresh clocks.
Malformed or stale NEW current-session packets revoke their affected intent or
shared frame; old-session packets cannot alter a new session. Stop cancels all
actions; only an explicit arm with a never-used epoch releases it. Rejected
session/clock packets return no decisions and do not advance the control clock;
the caller continues its independently scheduled tick(now). Arm is an
operator action, not permission for an upstream perception client to self-arm.

Run: .venv-api/bin/python -m final_project.pupper_fc.integration --replay INPUT.jsonl
This only reads input and writes JSONL decisions to stdout; no ROS, network,
robot process, files, policy activation or background task is created.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path
import sys

from .controller import Controller, MAX_OBSERVATION_AGE
from .model import Ball, Command, Decision, Field, Limits, Observation, Pose, zero
from .obstacles import Detection, DetectionFrame, ForwardObstacleGate

MAX_INTENT_LEASE = 0.5
LEASE_ROUNDING_TOLERANCE_S = 1e-12  # subtraction roundoff only, never extra lease time


def number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be finite")
    try:
        result = float(value)
    except OverflowError:
        raise ValueError(f"{label} must be finite") from None
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def token(value, label):
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ValueError(f"{label} must be a nonempty string of at most 128 characters")
    return value


def sequence(value):
    if type(value) is not int or value < 0:
        raise ValueError("seq must be a nonnegative integer")
    return value


def point(value, *, heading=False):
    if not isinstance(value, dict):
        raise ValueError("target/point must be an object")
    result = tuple(number(value.get(k), k) for k in ("x", "y", "yaw") if heading or k != "yaw")
    return Pose(*result) if heading else result


@dataclass(frozen=True)
class Intent:
    identity: tuple
    kind: str
    target: Pose | tuple | None
    valid_until: float


class Executor:
    """One latest shared frame, one latest intent/controller per configured robot.

    Call handle() for admitted events and tick(now) at the control cadence. The
    caller owns scheduling and output publication. A decision is not a hardware
    receipt; a future robot adapter must enforce expiry again before publishing.
    """

    def __init__(self, fields=None, limits=Limits(), *, obstacle_config=None):
        fields = fields or {"pupper-1": Field(), "pupper-2": Field(attack_sign=-1)}
        self.controllers = {token(r, "robot_id"): Controller(f, limits) for r, f in fields.items()}
        # None preserves the camera-free sandbox. A live front-camera adapter
        # must supply a calibrated config; configured gates start closed.
        self.obstacle_config = obstacle_config
        self.used_epochs = set()
        self.epoch = self.clock = None
        self.armed = False
        self.last_now = None
        self._clear()

    def _clear(self):
        self.frame = None
        self.frame_seq = -1
        self.capture_highwater = -math.inf
        self.intents = {r: None for r in self.controllers}
        self.intent_seq = {r: -1 for r in self.controllers}
        self.obstacle_gates = ({r: ForwardObstacleGate(self.obstacle_config) for r in self.controllers}
                               if self.obstacle_config is not None else {})
        self.obstacle_resume_after = {r: -math.inf for r in self.controllers}
        for controller in self.controllers.values():
            controller.cancel()

    def stop(self):
        self.armed = False
        self._clear()

    def arm(self, epoch, clock, now):
        epoch, clock = token(epoch, "epoch"), token(clock, "clock")
        now = number(now, "now")
        if epoch in self.used_epochs:
            raise ValueError("arm requires a new epoch; old actions cannot be resumed")
        self.stop()
        self.epoch, self.clock, self.last_now = epoch, clock, now
        self.used_epochs.add(epoch)
        # A deliberate new session gets fresh referee/controller state too.
        self.controllers = {r: Controller(c.field, c.limits) for r, c in self.controllers.items()}
        self.armed = True

    def _session(self, event):
        if not self.armed:
            raise ValueError("Stop is latched; deliberately arm with a new epoch")
        if event.get("epoch") != self.epoch or event.get("clock") != self.clock:
            raise ValueError("packet has an unrecognized session epoch or clock")

    def _frame(self, event, now):
        self._session(event)
        try:
            seq = sequence(event.get("seq"))
        except ValueError:
            self.frame = None
            raise
        if seq <= self.frame_seq:
            raise ValueError("duplicate/reordered frame")
        if self.frame is None or now - self.frame[0] >= MAX_OBSERVATION_AGE:
            for controller in self.controllers.values():
                controller.cancel()
        self.frame_seq = seq
        self.frame = None  # a new invalid observation cannot leave old motion alive
        stamp = number(event.get("captured_at"), "captured_at")
        if stamp <= self.capture_highwater or not 0 <= now - stamp <= MAX_OBSERVATION_AGE:
            raise ValueError("frame capture is duplicate, reordered, future or stale")
        self.capture_highwater = stamp
        robots, raw_ball, goals = event.get("robots"), event.get("ball"), event.get("goals", {})
        if not isinstance(robots, dict) or set(robots) - set(self.controllers):
            raise ValueError("robots must contain only configured robot IDs")
        ball = None
        if raw_ball is not None:
            x, y = point(raw_ball)
            ball = Ball(x, y, number(raw_ball.get("vx", 0), "ball.vx"),
                        number(raw_ball.get("vy", 0), "ball.vy"))
        if not isinstance(goals, dict) or set(goals) - {"left", "right"}:
            raise ValueError("goals must map left/right to field points or null")
        goals = {key: point(value) for key, value in goals.items() if value is not None}
        observations = {}
        for robot_id in self.controllers:
            row = robots.get(robot_id)
            pose, fallen, goal = None, False, 0
            if row is not None:
                pose = point(row, heading=True)
                fallen, goal = row.get("fallen", False), row.get("goal", 0)
                if type(fallen) is not bool or type(goal) is not int or goal not in (-1, 0, 1):
                    raise ValueError("fallen/goal must be boolean and integer -1/0/+1")
            observations[robot_id] = Observation(stamp, pose, ball, robot_id, fallen, goal)
        self.frame = (stamp, observations, goals)

    def _intent(self, event, now):
        self._session(event)
        robot_id = event.get("robot_id")
        if robot_id not in self.controllers:
            raise ValueError("unknown robot_id")
        try:
            seq = sequence(event.get("seq"))
        except ValueError:
            self.intents[robot_id] = None
            self.controllers[robot_id].cancel()
            raise
        if seq <= self.intent_seq[robot_id]:
            raise ValueError("duplicate/reordered intent")
        previous = self.intents[robot_id]
        self.intent_seq[robot_id] = seq
        self.intents[robot_id] = None
        try:
            issued = number(event.get("issued_at"), "issued_at")
            expiry = number(event.get("valid_until"), "valid_until")
            if (not issued <= now < expiry
                    or not 0 < expiry - issued <= MAX_INTENT_LEASE + LEASE_ROUNDING_TOLERANCE_S):
                raise ValueError("intent is future, expired or exceeds the 0.5-second lease")
            expiry = min(expiry, issued + MAX_INTENT_LEASE)
            if now >= expiry:
                raise ValueError("intent is expired")
            identity = token(event.get("intent_id"), "intent_id")
            kind = event.get("kind")
            if kind not in ("attack", "move", "defend", "shoot"):
                raise ValueError("kind must be attack, move, defend or shoot")
            target = None if kind == "attack" else point(event.get("target"), heading=kind in ("move", "defend"))
            if kind == "attack" and event.get("target") is not None:
                raise ValueError("attack uses the observed opponent goal; use shoot for an explicit target")
            intent = Intent((identity, kind, target), kind, target, expiry)
        except (ValueError, TypeError):
            self.controllers[robot_id].cancel()
            raise
        if previous is None or previous.valid_until <= now or previous.identity != intent.identity:
            self.controllers[robot_id].cancel()
        self.intents[robot_id] = intent

    def _obstacles(self, event, now):
        """One robot's normalized camera input, independently timed from field state."""
        self._session(event)
        robot_id = event.get("robot_id")
        if robot_id not in self.controllers:
            raise ValueError("unknown robot_id")
        gate = self.obstacle_gates.get(robot_id)
        if gate is None:
            raise ValueError("front-camera gate is not configured")
        # Observe expiry before a new frame can clear it, even when the
        # scheduler slept and a one-frame clearance policy is configured.
        if not gate.evaluate(now).permitted:
            self.controllers[robot_id].cancel()
            self.obstacle_resume_after[robot_id] = now
        try:
            if type(event.get("valid")) is not bool or type(event.get("healthy")) is not bool:
                raise ValueError("camera valid/healthy must be explicit booleans")
            rows = event.get("detections")
            if not isinstance(rows, list):
                raise ValueError("detections must be a list; empty is distinct from missing")
            detections = []
            for row in rows:
                if not isinstance(row, dict) or not isinstance(row.get("box"), (list, tuple)) or len(row["box"]) != 4:
                    raise ValueError("detection requires normalized xyxy box")
                detections.append(Detection(tuple(number(v, "box") for v in row["box"]),
                                            number(row.get("confidence"), "confidence"),
                                            row.get("class_id", "")))
            frame = DetectionFrame(sequence(event.get("seq")), number(event.get("captured_at"), "captured_at"),
                                   tuple(detections), event["valid"], event["healthy"], event.get("format"))
            decision = gate.update(frame, now)
            if decision.state in ("invalid", "stale"):
                raise ValueError(decision.reason)
        except (ValueError, TypeError) as exc:
            gate.invalidate(str(exc), now)
            self.controllers[robot_id].cancel()
            raise ValueError(str(exc)) from None

    def tick(self, now):
        try:
            now = number(now, "now")
        except ValueError:
            self.stop()
            return {r: Decision("stopped", "invalid control clock", Command(robot_id=r)) for r in self.controllers}
        if self.last_now is not None and now < self.last_now:
            self.stop()
        self.last_now = now
        decisions = {}
        for robot_id, controller in self.controllers.items():
            intent = self.intents[robot_id]
            obstacle = self.obstacle_gates[robot_id].evaluate(now) if robot_id in self.obstacle_gates else None
            if not self.armed:
                phase, reason = "stopped", "Stop latched; arm a new epoch"
            elif obstacle is not None and not obstacle.permitted:
                phase, reason = "obstacle_hold", f"front camera {obstacle.state}: {obstacle.reason}"
                self.obstacle_resume_after[robot_id] = now
            elif intent is None or now >= intent.valid_until:
                phase, reason = "intent_lost", "no current unexpired intent"
                self.intents[robot_id] = None
            elif self.frame is None or not 0 <= now - self.frame[0] <= MAX_OBSERVATION_AGE:
                phase, reason = "tracking_lost", "no current synchronized field frame"
            elif self.frame[0] < self.obstacle_resume_after[robot_id]:
                phase, reason = "tracking_lost", "new field observation required after obstacle hold"
            else:
                stamp, observations, goals = self.frame
                kwargs = {}
                if intent.kind in ("move", "defend"):
                    kwargs["target_pose"] = intent.target
                elif intent.kind == "shoot":
                    kwargs["shot_target"] = intent.target
                else:
                    destination = goals.get("right" if controller.field.attack_sign == 1 else "left")
                    if destination is None:
                        controller.cancel()
                        decisions[robot_id] = Decision("tracking_lost", "opponent goal is missing", zero(now, robot_id))
                        continue
                    kwargs["shot_target"] = destination
                decision = controller.tick(observations[robot_id], now, **kwargs)
                deadline = min(decision.command.expires_at, intent.valid_until, stamp + MAX_OBSERVATION_AGE)
                if obstacle is not None:
                    deadline = min(deadline, obstacle.valid_until)
                if deadline <= now:
                    controller.cancel()
                    decisions[robot_id] = Decision("tracking_lost", "evidence expired at this tick", zero(now, robot_id))
                    continue
                decisions[robot_id] = replace(decision,
                    phase="defend" if intent.kind == "defend" and decision.phase == "move" else decision.phase,
                    command=replace(decision.command, expires_at=deadline))
                continue
            controller.cancel()
            decisions[robot_id] = Decision(phase, reason, zero(now, robot_id))
        return decisions

    def handle(self, event):
        """Apply one normalized event and return a fully serializable shadow tick."""
        accepted, reason = True, "tick"
        raw_now = event.get("now") if isinstance(event, dict) else None
        kind = event.get("event") if isinstance(event, dict) else None
        if kind == "arm":
            try:
                self.arm(event.get("epoch"), event.get("clock"), raw_now)
            except (ValueError, TypeError) as exc:
                # Rejected operator requests are atomic too: their timestamp
                # cannot rewind/advance an otherwise valid active session.
                return dict(accepted=False, reason=str(exc), now=self.last_now,
                            armed=self.armed, epoch=self.epoch, decisions={})
        if kind in ("frame", "intent", "obstacles"):
            try:
                self._session(event)
            except ValueError as exc:
                # A delayed old-session packet cannot advance or rewind the
                # active session's clock, expire its intents, or tick its plan.
                return dict(accepted=False, reason=str(exc), now=self.last_now,
                            armed=self.armed, epoch=self.epoch, decisions={})
        if kind == "stop":
            self.stop()  # Stop wins even when its timestamp is malformed.
        try:
            now = number(raw_now, "now")
            if kind == "arm":
                reason = "new epoch armed"
            else:
                if self.last_now is not None and now < self.last_now:
                    self.stop()
                    raise ValueError("control clock moved backwards; arm a new epoch")
                if kind == "frame":
                    self._frame(event, now)
                    reason = "frame replaced"
                elif kind == "intent":
                    self._intent(event, now)
                    reason = "intent replaced"
                elif kind == "obstacles":
                    self._obstacles(event, now)
                    reason = "front-camera observation replaced"
                elif kind == "stop":
                    reason = "Stop latched"
                elif kind != "tick":
                    self.stop()
                    raise ValueError("unknown event; stopped")
        except (ValueError, TypeError) as exc:
            accepted, reason = False, str(exc)
        decisions = self.tick(raw_now)
        return dict(accepted=accepted, reason=reason,
                    now=self.last_now, armed=self.armed, epoch=self.epoch,
                    decisions={r: asdict(d) for r, d in decisions.items()})


def replay(events, executor=None):
    executor = executor or Executor()
    for event in events:
        yield executor.handle(event)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--replay", type=Path, required=True, help="normalized input JSONL; output is shadow decisions only")
    args = parser.parse_args(argv)
    executor = Executor()
    try:
        with args.replay.open() as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    event = json.loads(line)
                except ValueError as exc:
                    executor.stop()
                    print(f"line {line_number}: invalid JSON; stopped: {exc}", file=sys.stderr)
                    return 2
                print(json.dumps(executor.handle(event), allow_nan=False))
    except OSError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
