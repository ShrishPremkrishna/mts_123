#!/usr/bin/env python3
"""One field executor, guarded walker and leg-kick owner in the fc_walk_kick release.

Opening this runner requires a previously selected, supervised native session.
It does not home, start a controller, enable the HTTP server, or arm field play.
Normalized field/intent input uses /fc/normalized_events; local stdin owns arm,
resume, status and Stop. EOF ends the session. Times are Linux monotonic seconds.

With --operator-controls, every stdin request must consume a robot-issued
ticket, and per-robot teleop shares this process's single CombinedRuntime with
autonomous play. Board health comes from the strict driver receipt
(board_imu); absent or stale evidence never grants authority.

Kick receipts on /fc/kick_result and stdout separate what each one proves:
engaged (job requested), clip_started (native reported the requested clip
active), completed (confirmed clip and restored walker), retired (walker
restored without a successful clip) and aborted (runtime stopped with the job
unresolved). None of them observes ball contact.
"""
from __future__ import annotations

import argparse
import asyncio
from dataclasses import replace
import fcntl
import json
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
for directory in (ROOT, ROOT / "gemini_pupper/robot", ROOT / "gemini_pupper/lab"):
    sys.path.insert(0, str(directory))

from final_project.behaviors import command_deadline as protocol
from final_project.behaviors import command_path_evidence as evidence
from final_project.behaviors.board_imu import BoardReceipt, MatchImu
from final_project.behaviors.fc_executor_bridge import (
    BridgeCore, DEADLINE_CONTRACT, EVENT_TOPIC, EventSink, TICK_S, load_fields)
from final_project.behaviors.operator_drive import OperatorDrive
from final_project.pupper_fc.model import zero
from follow_me_pupper.robot import onboard_ball

NAME = "fc_walk_kick_runner"
WALKER = "/gemini_walk"
KICK_TOPIC = "/fc/kick_result"
NO_CONTACT = "no ball contact or goal is observed by this receipt"


class KickReporter:
    """Report each kick attempt's edges once, from the handoff's own evidence."""

    def __init__(self, handoff):
        self.handoff = handoff
        self.request = None
        self.started = False

    def _edge(self, state, request, **fields):
        return {"state": state, "robot_id": request.robot_id, "epoch": request.epoch,
                "attempt_id": request.attempt_id, "side": request.side, "physical": NO_CONTACT, **fields}

    def observe(self, step, now):
        edges = []
        retired = self.handoff.last_retired
        if self.request is not None and retired is not None and retired[0] == self.request:
            if retired[1] == "kick_complete":
                edges.append(self._edge("completed", self.request, completed_at=now,
                                        evidence="native clip confirmed and walker restored"))
            else:
                edges.append(self._edge("retired", self.request, ended_at=now, outcome=retired[1],
                                        evidence="walker restored without a successful clip"))
            self.request, self.started = None, False
        if step.request is not None:
            self.request, self.started = step.request, False
            edges.append(self._edge("engaged", step.request, issued_at=step.request.issued_at,
                                    evidence="kick job requested from the native runtime"))
        if (self.request is not None and not self.started and self.handoff.active == self.request
                and self.handoff.clip_started):
            self.started = True
            edges.append(self._edge("clip_started", self.request, started_at=now,
                                    evidence="native reported the requested clip active"))
        return edges

    def abort(self, reason, now):
        if self.request is None or self.handoff.active != self.request:
            return None
        edge = self._edge("aborted", self.request, ended_at=now, reason=str(reason)[:180],
                          evidence="runtime stopped before walker restoration was confirmed")
        self.request = None
        return edge


def native_authority_qos():
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class GuardedPath:
    """Check the running guard, native source and graph before issuing velocity."""

    def __init__(self, core, node, publish):
        self.core, self.node, self.publish = core, node, publish
        self.guard = self.guard_at = self.guard_source = self.frame_deadline = None
        self.native = self.native_at = None
        self.guard_error = self.native_error = None
        self.graph_checks = {}
        self.graph_binding = None
        self.last_walk_generation = 0
        self.return_after_generation = 0
        self.balance = self.sync_balance = None

    def guard_status(self, raw):
        now = self.core.clock()
        try:
            self.guard, self.guard_source, self.frame_deadline = evidence.decode_guard_status(
                raw, now, self.guard_source, self.frame_deadline,
                command_contract=DEADLINE_CONTRACT, clock_id=self.core.deadline.clock_id,
                source_instance=self.core.deadline.source_instance)
            self.guard_error = None
        except (ValueError, TypeError, KeyError) as exc:
            self.guard_error = str(exc)
        self.guard_at = now

    def native_status(self, raw):
        now = self.core.clock()
        try:
            self.native = evidence.decode_native_authority(raw, now, self.core.deadline.clock_id, self.native)
            self.native_error = None
        except (ValueError, TypeError, KeyError) as exc:
            self.native_error = str(exc)
        self.native_at = now

    def check(self, during_kick=False):
        now = self.core.clock()
        # Strict (walking) and permissive (zero/kick) graph checks alternate on
        # every tick; cache each for 100 ms so idle ticks do not rescan the graph twice.
        generation = self.native["generation"] if self.native else None
        binding = (self.return_after_generation, generation)
        if self.graph_binding != binding:
            self.graph_checks.clear()
            self.graph_binding = binding
        cached = self.graph_checks.get(during_kick)
        if cached is None or not 0 <= now - cached[0] < .10:
            graph = evidence.guard_publishers_problem(
                self.node, "/" + NAME, command_contract=DEADLINE_CONTRACT,
                native_node=WALKER, detector_node="/viser_camera", allow_absent_native=during_kick)
            # A fault discards the other mode's cached success. During a kick
            # transfer, recheck on every mode change until the walker returns.
            if graph or (self.return_after_generation and self.last_walk_generation <= self.return_after_generation):
                self.graph_checks.clear()
            cached = self.graph_checks[during_kick] = (now, graph)
        if cached[1]:
            return cached[1]
        guard = evidence.guard_status_problem(self.guard, self.guard_at, self.guard_error,
            self.frame_deadline, now, command_contract=DEADLINE_CONTRACT)
        if guard:
            return guard
        if not during_kick:
            reason = evidence.native_authority_problem(self.native, self.native_at, self.native_error,
                                                       now, self.core.deadline.source_instance)
            if reason:
                return "native", reason
            if self.native["generation"] <= self.return_after_generation:
                return "native", "waiting for the restored walker's new activation"
            if (self.core.executor.armed and self.last_walk_generation
                    and self.last_walk_generation > self.return_after_generation
                    and self.native["generation"] != self.last_walk_generation):
                return "authority", "walker activation changed outside a kick handoff"
            self.last_walk_generation = self.native["generation"]
        return None

    def clip_started(self):
        self.return_after_generation = max(self.return_after_generation, self.last_walk_generation)

    def restored_walker_ready(self):
        """Allow discovery of the returned walker while all velocity stays zero."""
        problem = self.check(True)
        if problem:
            raise RuntimeError(problem[1])
        if self.native_error:
            raise RuntimeError(self.native_error)
        if self.native and self.native["bound_source_instance"] not in ("", self.core.deadline.source_instance):
            raise RuntimeError("restored walker is bound to another source")
        problem = self.check(False)
        if problem is None:
            return True
        kind, reason = problem
        if kind == "native" or (kind == "path" and reason in (
                "missing selected publisher on " + evidence.NATIVE_AUTHORITY_TOPIC,
                "missing selected subscriber on " + protocol.ADMITTED_TOPIC)):
            return False
        raise RuntimeError(reason)

    def walking_problem(self):
        problem = self.check(False)
        if not problem or problem[0] != "camera":
            return problem
        # A camera hold may keep a healthy walker standing. It cannot excuse
        # loss of the native command owner in the same interval.
        reason = evidence.native_authority_problem(self.native, self.native_at, self.native_error,
                                                  self.core.clock(), self.core.deadline.source_instance)
        if reason:
            return "native", reason
        if self.last_walk_generation and self.native["generation"] != self.last_walk_generation:
            return "authority", "walker activation changed during camera hold"
        return problem

    def write(self, command):
        """Publish one command; return a hold reason if camera evidence lapsed at the send boundary."""
        motion_deadline = command.expires_at
        moving = any((command.vx, command.vy, command.wz))
        if moving and self.balance is not None:
            self.sync_balance()
            if self.balance.state != "healthy":
                command = zero(self.core.clock(), self.core.robot_id)
                moving = False
            else:
                command = replace(command, expires_at=min(command.expires_at, self.balance.deadline_ns / 1e9))
        problem = self.walking_problem() if moving else self.check(True)
        hold = None
        if problem and (moving or problem[0] == "authority"):
            if moving and problem[0] == "camera":
                hold = problem[1]
                command = zero(self.core.clock(), self.core.robot_id)
                moving = False
            elif moving:
                raise RuntimeError(problem[1])
            else:
                return None  # Never attempt a competing source handshake.
        if moving:
            native_deadline = self.native["sampled_mono_ns"] / 1e9 + evidence.NATIVE_AUTHORITY_MAX_AGE_S
            command = replace(command, expires_at=min(command.expires_at, self.frame_deadline, native_deadline))
        payload, actuating, reason = self.core.deadline.encode(command)
        if payload is not None:
            self.publish(payload)
        if hold and payload is None:
            raise RuntimeError(reason or "camera hold could not publish zero")
        if moving and (reason or not actuating):
            encoded_at = self.core.deadline.last_issued_ns / 1e9
            if (payload is not None and reason == "original command expired before publication"
                    and command.expires_at == motion_deadline
                    and encoded_at < min(self.frame_deadline, native_deadline)
                    and (self.balance is None or encoded_at < self.balance.deadline_ns / 1e9)):
                # The encoder already published genuine neutral. An ordinary
                # motion lease can expire between the check and encoding; it
                # neither renews movement nor poisons the standing session.
                return "motion command expired before publication"
            # The encoder already substituted and published zero if sensor
            # evidence expired in the instructions since check(). Other faults propagate.
            if (payload is not None and reason == "original command expired before publication"
                    and (command.expires_at == self.frame_deadline < native_deadline
                         or self.balance is not None and command.expires_at == self.balance.deadline_ns / 1e9)):
                hold = "sensor evidence expired at velocity publication"
            else:
                raise RuntimeError(reason or "walk deadline expired at publication")
        if hold:
            self.core.hold(True, hold)
        return hold


async def await_gateway_stop(caller, spin_once, ros_ok):
    """Keep receiving the one-shot Stop acknowledgement through bounded shutdown."""
    while caller.stop_pending:
        available = ros_ok()
        if available:
            try:
                spin_once(timeout_sec=.01)
            except RuntimeError:
                available = False
        result = caller.poll_stop(force_unconfirmed=not available)
        if result is not None:
            return result
        await asyncio.sleep(0)
    return caller.stop_result


def gateway_stop_ready(gateway, anchor):
    return (anchor is not None and gateway.stop_capable
            and gateway.state["instance_id"] == anchor["instance_id"]
            and gateway.stop_publisher.get_subscription_count() > 0)


def resume_cycle(core, now):
    executor = core.executor
    if not executor.armed:
        return None
    try:
        executor.check_resume(core.robot_id, executor.frame_seq, now)
    except ValueError:
        return None
    return executor.epoch, executor.resume_frame_seq[core.robot_id]


def native_job_active(runtime):
    robot = runtime.robot
    return runtime.handoff.active is not None or robot.current is not None or not robot.commands.empty()


def sync_match_balance(balance, core, path, runtime, controls, emit):
    """Refresh board health; a new hold ends this epoch's field and teleop authority."""
    balance.refresh()
    if not balance.hold_id:
        balance.applied_hold = None
        balance.reported_hold_state = None
        return
    if balance.walk_generation is None:
        balance.walk_generation = path.last_walk_generation or None
    if (balance.state == "ready_to_resume" and
            (not path.native or path.native["generation"] != balance.walk_generation)):
        balance.state, balance.reason = "paused", "Walker activation changed; end the match and verify standing"
    if (balance.state == "ready_to_resume" and (runtime.stop_reason or runtime.robot.fallen
            or runtime.robot.stops != balance.stop_generation or runtime.robot.motion_block_reason())):
        balance.state, balance.reason = "paused", "Stop, fall or native fault prevents resuming this match"
    if balance.state == "ready_to_resume":
        if balance.walker_lost:
            balance.state, balance.reason = "paused", "Walker lost leg ownership; end the match and verify standing"
        elif (not balance.owner_current() or path.native.get("sampled_mono_ns", 0) <= balance.hold_ns
              or path.check(False)):
            balance.state, balance.reason = "paused", "Waiting for current sole walker ownership and guarded path"
    if balance.hold_id != balance.applied_hold:
        balance.applied_hold = balance.hold_id
        core.executor.stop()
        if controls is not None:
            controls.invalidate()
            controls.match_epoch = None
        if native_job_active(runtime):
            runtime.stop("IMU pause during a native job: " + balance.reason)
        else:
            runtime.velocity_sink(zero(core.clock(), core.robot_id))
    state = balance.hold_id, balance.state
    if balance.reported_hold_state != state:
        balance.reported_hold_state = state
        emit({"event": "imu", "robot_id": core.robot_id, "imu": balance.snapshot()})


async def sample_walker(balance, robot):
    hold_id, started_ns = balance.hold_id, balance.clock_ns()
    if not hold_id:
        return
    try:
        owners = await asyncio.wait_for(robot.controller_states(), .15)
    except (OSError, RuntimeError, TimeoutError):
        owners = {}
    balance.observe_owners(hold_id, started_ns, owners, robot.leg_controllers)


def check_balance_resume(message, core, path, runtime, gateway, controls, balance):
    path.sync_balance()
    balance.check_resume(message)
    robot = runtime.robot
    if (controls is None or runtime.stop_reason or robot.fallen or robot.motion_block_reason()
            or robot.stops != balance.stop_generation or native_job_active(runtime)
            or core.executor.armed or controls.match_epoch is not None):
        raise ValueError("IMU resume requires an idle, stopped match owner")
    if not gateway_stop_ready(gateway, robot.gateway_stop_anchor()):
        raise ValueError("Waiting for the selected Stop channel")
    if (not path.native or not balance.walk_generation or path.native["generation"] != balance.walk_generation
            or message.get("mode", controls.mode) != controls.mode):
        raise ValueError("Walker activation or operator mode changed during the IMU pause")
    problem = path.check(False)
    if problem:
        raise ValueError(problem[1])
    epoch = message.get("epoch")
    if not epoch.strip() or len(epoch) > 128 or epoch in core.executor.used_epochs:
        raise ValueError("Resume requires a fresh, unused soccer epoch")


async def imu_resume_action(message, core, path, runtime, gateway, controls, balance):
    """Re-arm after a recovered hold: consume the ticket, then confirm sole walker ownership."""
    check_balance_resume(message, core, path, runtime, gateway, controls, balance)
    expires = controls.consume_ticket(message, core.clock())
    revision = controls.authority_revision
    anchor = (balance.session, balance.generation, balance.incident)
    remaining = expires - core.clock()
    if remaining <= 0:
        raise ValueError("IMU resume authorization expired before the walker check")
    await asyncio.wait_for(sample_walker(balance, runtime.robot), remaining)
    check_balance_resume(message, core, path, runtime, gateway, controls, balance)
    if (core.clock() >= expires or controls.authority_revision != revision
            or (balance.session, balance.generation, balance.incident) != anchor):
        raise ValueError("IMU resume authorization expired or changed during the walker check")
    return operator_action(message, core, path, runtime, gateway, controls, balance=balance,
                           resume_confirmed=(revision, expires))


def operator_action(message, core, path, runtime, gateway, controls, *, balance, set_ball_mode=None,
                    resume_confirmed=None):
    """Apply one operator request. Every authority-granting action requires healthy board evidence."""
    action = message.get("command")
    robot = runtime.robot
    if balance is None:
        raise ValueError("Board health proof is unavailable")
    path.sync_balance()
    if action == "imu_pause":
        if controls is None:
            raise ValueError("IMU pause requires the match operator")
        controls.consume_ticket(message, core.clock())
        balance.pause(message.get("epoch"))
        path.sync_balance()
        return {"accepted": True, "state": "hard_stop" if runtime.stop_reason else "paused",
                "epoch": balance.epoch, "hold_id": balance.hold_id, "imu": balance.snapshot()}
    if action == "imu_resume":
        check_balance_resume(message, core, path, runtime, gateway, controls, balance)
        if (not resume_confirmed or resume_confirmed[0] != controls.authority_revision
                or core.clock() >= resume_confirmed[1]):
            raise ValueError("IMU resume requires an explicit current operator and walker check")
    elif action not in ("pause", "onboard_ball_mode") and balance.state != "healthy":
        raise ValueError(balance.reason)
    if action == "onboard_ball_mode":
        if controls is None or set_ball_mode is None:
            raise ValueError("onboard settings require an operator connection")
        mode = message.get("mode")
        if mode not in onboard_ball.MODES:
            raise ValueError("unknown onboard ball mode")
        controls.consume_ticket(message, core.clock())
        set_ball_mode(mode)
        return {"accepted": True, "requested_mode": mode}
    if action in ("mode", "drive"):
        if controls is None:
            raise ValueError("keyboard controls are not enabled")
        if not gateway_stop_ready(gateway, robot.gateway_stop_anchor()):
            controls.observe_path(False, controls.generation)
        stable = (not native_job_active(runtime)
                  and path.guard is not None and path.guard["state"] in ("ready", "hold"))
        controls.accept(message, now=core.clock(), stable=stable)
        return {"accepted": True, "mode": controls.mode}
    if controls is not None:
        now = core.clock()
        cycle = resume_cycle(core, now) if action == "resume" and controls.ready else None
        if action != "imu_resume":
            controls.consume_ticket(message, now, resume_cycle=cycle)
        if action == "resume":
            message = {**message, "frame_seq": core.executor.frame_seq}
        if controls.mode != "autonomous" and action not in ("pause", "arm", "imu_resume"):
            raise ValueError("select autonomous mode before starting field play")
    if action == "resume" and controls is None and "frame_seq" not in message:
        if resume_cycle(core, core.clock()) is None:
            raise ValueError("the robot and reset ball are not ready")
        message = {**message, "frame_seq": core.executor.frame_seq}
    if action == "pause":
        balance.end()
        if native_job_active(runtime):
            runtime.stop("operator ended match during a native job")
            return {"accepted": True, "state": "hard_stop", "reason": runtime.stop_reason}
        core.executor.stop()
        if controls is not None:
            controls.invalidate()
            controls.match_epoch = None
        runtime.velocity_sink(zero(core.clock(), core.robot_id))
        return {"accepted": True, "state": "standby", "armed": False}
    if action in ("arm", "imu_resume"):
        if core.executor.armed or (controls is not None and controls.match_epoch is not None):
            raise ValueError("field play is already armed")
        if controls is not None and message.get("mode", controls.mode) != controls.mode:
            raise ValueError("control mode changed before kickoff")
        if not gateway_stop_ready(gateway, robot.gateway_stop_anchor()):
            raise ValueError("waiting for the selected gateway Stop channel")
        problem = path.check(False)
        if problem:
            raise ValueError(problem[1])
        path.sync_balance()
        if runtime.stop_reason or robot.fallen or robot.motion_block_reason():
            raise ValueError("Stop, fall or native fault prevents field authority")
        if action == "imu_resume":
            balance.check_resume(message)
        elif balance.state != "healthy":
            raise ValueError(balance.reason)
        hold_id = balance.hold_id
        reply = core.operator({**message, "command": "arm"})
        if reply["accepted"]:
            balance.begin(message["epoch"], message.get("pitch"), robot.stops)
            if action == "imu_resume":
                reply.update(state="armed", epoch=message["epoch"], hold_id=hold_id, imu=balance.snapshot())
            if controls is not None and controls.mode == "teleop":
                # Manual drive owns the walker exclusively. The executor keeps
                # the epoch's replay fence without arming its autonomous path.
                core.executor.stop()
                controls.invalidate()
                controls.match_epoch = message["epoch"]
        return reply
    return core.operator(message)


class RunnerSession:
    """One runner's control tick and operator protocol, independent of ROS transport.

    run() supplies the ROS node, publishers and stdin; offline fixtures supply
    fakes. Both drive this object, so autonomy, teleop, kicks and board holds
    share one CombinedRuntime. Call attach() with that runtime before tick().
    """

    def __init__(self, core, path, balance, gateway, emit, *, controls=None, publish_kick=None,
                 set_ball_mode=None, ball_snapshot=None, clock=time.monotonic):
        self.core, self.path, self.balance, self.gateway, self.emit = core, path, balance, gateway, emit
        self.controls = controls
        self.publish_kick = publish_kick or (lambda edge: None)
        self.set_ball_mode, self.ball_snapshot = set_ball_mode, ball_snapshot
        self.clock = clock
        self.runtime = self.kicks = self.resume_task = None
        self.last_phase = None
        self.next_ball_report = 0.
        path.balance, path.sync_balance = balance, self.sync_balance

    def attach(self, runtime):
        self.runtime = runtime
        self.kicks = KickReporter(runtime.handoff)

    def sync_balance(self):
        sync_match_balance(self.balance, self.core, self.path, self.runtime, self.controls, self.emit)

    def runtime_problem(self, during_kick):
        core, runtime = self.core, self.runtime
        self.sync_balance()
        if self.balance.state != "healthy" and (during_kick or runtime.handoff.active is not None):
            return self.balance.reason
        problem = self.path.check(True) if during_kick else self.path.walking_problem()
        if problem and problem[0] == "camera" and not during_kick and runtime.handoff.active is None:
            if not core.executor.external_holds[core.robot_id]:
                core.hold(True, problem[1])
            return None  # Normal walking pauses at once. An in-progress kick still requires its evidence.
        return problem[1] if problem else None

    def sync_camera_hold(self):
        core, path, controls = self.core, self.path, self.controls
        active_clip = self.runtime.handoff.active is not None
        problem = path.check(active_clip)
        held = bool(problem and problem[0] == "camera")
        if (held or problem is None) and held != core.executor.external_holds[core.robot_id]:
            core.hold(held, problem[1] if held else "camera clear")
        if controls is not None:
            controls.observe_path(problem is None and not active_clip
                                  and self.balance.state in ("healthy", "ready_to_resume"),
                                  path.native.get("generation") if path.native else None)

    def on_guard(self, raw):
        self.path.guard_status(raw)
        armed = self.core.executor.armed or (self.controls is not None and self.controls.match_epoch is not None)
        if armed and self.path.guard_error:
            self.runtime.stop("invalid guard evidence: " + self.path.guard_error)
        self.sync_camera_hold()

    def report_kick(self, edge):
        self.publish_kick(edge)
        self.emit({"event": "kick", **edge})

    def tick(self):
        """One control period: refresh evidence, publish one command, report edges and offers."""
        core, runtime, controls = self.core, self.runtime, self.controls
        self.sync_balance()
        self.sync_camera_hold()
        if runtime.handoff.active is not None and runtime.handoff.clip_started:
            self.path.clip_started()
        now = self.clock()
        manual = controls.command(now) if controls is not None and self.balance.state == "healthy" else None
        step = runtime.tick(manual_command=manual)
        for edge in self.kicks.observe(step, self.clock()):
            self.report_kick(edge)
        if controls is not None:
            if self.ball_snapshot is not None and now >= self.next_ball_report:
                self.next_ball_report = now + .10
                self.emit({"event": "onboard_ball", "robot_id": core.robot_id, "onboard_ball": self.ball_snapshot(now)})
            if core.executor.external_holds[core.robot_id]:
                controls.observe_path(False, controls.generation)
            controls.observe_output(step.decision.command)
            # Observe completed runtime work with a fresh time, not the earlier
            # command-sampling time (the executor advances its clock in tick).
            now = self.clock()
            offer = (controls.offer(now, resume_cycle(core, now) if controls.ready else None)
                     if now >= controls.next_issue else None)
            if offer is not None:
                self.emit({**offer, "imu": self.balance.snapshot()})
        if step.decision.phase != self.last_phase:
            self.last_phase = step.decision.phase
            self.emit({"event": "phase", "phase": self.last_phase, "reason": step.decision.reason})
        return step

    def reply(self, message, reply):
        context = message if isinstance(message, dict) else {}
        guard = self.path.guard
        self.emit({**reply, "event": "operator", "command": context.get("command"),
                   "ticket": context.get("ticket"), "robot_id": self.core.robot_id,
                   "decision_mono_ns": time.monotonic_ns(), "decision_wall_ns": time.time_ns(),
                   "guard_sample_mono_ns": guard.get("sampled_at_monotonic_ns") if guard else None,
                   "frame_sequence": guard.get("sequence") if guard else None})

    async def _resume(self, message):
        try:
            reply = await imu_resume_action(message, self.core, self.path, self.runtime, self.gateway,
                                            self.controls, self.balance)
        except (OSError, ValueError, TypeError, OverflowError, RuntimeError, TimeoutError) as exc:
            reply = {"accepted": False, "reason": str(exc)[:180] or "Walker check timed out; match held"}
        self.reply(message, reply)

    def handle(self, message):
        """Apply one decoded operator line. Returns True when the line ends the session."""
        try:
            if not isinstance(message, dict):
                raise ValueError("expected a JSON object")
            action = message.get("command")
            if action in ("stop", "quit"):
                return True
            if action == "status":
                self.sync_balance()
                self.emit({"event": "status", "armed": self.core.executor.armed,
                           "path_problem": self.path.check(self.runtime.handoff.active is not None),
                           "stop": self.runtime.stop_reason, "imu": self.balance.snapshot(),
                           "teleop_epoch": self.controls.match_epoch if self.controls else None})
                return False
            if self.controls is not None:
                self.sync_camera_hold()
            if action == "imu_resume":
                if self.resume_task is not None and not self.resume_task.done():
                    raise ValueError("A balance resume is already pending")
                self.resume_task = asyncio.get_running_loop().create_task(self._resume(message))
                return False
            reply = operator_action(message, self.core, self.path, self.runtime, self.gateway, self.controls,
                                    balance=self.balance, set_ball_mode=self.set_ball_mode)
        except (ValueError, TypeError, OverflowError) as exc:
            reply = {"accepted": False, "reason": str(exc)[:180]}
        self.reply(message, reply)
        return False

    async def close(self, now):
        """Cancel pending operator work and report a kick left without walker restoration."""
        if self.resume_task is not None:
            self.resume_task.cancel()
            await asyncio.gather(self.resume_task, return_exceptions=True)
        if self.kicks is not None:
            aborted = self.kicks.abort(self.runtime.stop_reason, now)
            if aborted is not None:
                self.report_kick(aborted)


async def run(args):
    from foundation.hardware.fc_walk_kick_release import verify_running_session
    from foundation.hardware.software_start import attach
    # Refuse stale installs or an unowned manager before constructing a robot client.
    manifest = verify_running_session(args.run_dir, ROOT, operator_controls=args.operator_controls)
    from robot_api import Robot
    from fc_walk_kick import CombinedRuntime
    from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    clock = time.monotonic
    fields = load_fields(json.loads(args.field_config.read_text()))
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    core = BridgeCore(args.robot_id, args.producer_id, boot, fields,
                      command_contract=DEADLINE_CONTRACT, kick_side=args.side,
                      paw_forward_m=COMBINED_PAW_FORWARD_M[args.side])
    robot = Robot(ros_signal_handlers=False, observe_only=True)
    robot.follow_available = False
    node = rclpy.create_node(NAME)
    spin = SingleThreadedExecutor()
    spin.add_node(node)
    publish = node.create_publisher(String, protocol.REQUEST_TOPIC, 1)
    path = GuardedPath(core, node, lambda data: publish.publish(String(data=data)))
    controls = OperatorDrive(core) if args.operator_controls else None
    kick_events = node.create_publisher(String, KICK_TOPIC, 4)
    emit = EventSink()
    owner = json.loads((args.run_dir / "owned_session.json").read_text())["processes"]["manager"]
    board_reader = BoardReceipt(args.run_dir / "fc-board-health.bin", owner["pid"],
                                manifest.get("fc_imu_recovery", "strict"))

    def latest_ros_imu():
        try:
            return robot.imu[-1]
        except IndexError:
            return None

    balance = MatchImu(board_reader, latest_ros_imu)
    set_ball_mode = ball_snapshot = None
    if controls is not None:
        ball_receiver = onboard_ball.OnboardBallReceiver()
        ball_modes = node.create_publisher(String, onboard_ball.MODE_TOPIC, 1)

        def set_ball_mode(mode):
            ball_modes.publish(String(data=mode))

        def ball_snapshot(now):
            return ball_receiver.snapshot(node.get_clock().now().nanoseconds, now)

        node.create_subscription(String, onboard_ball.TOPIC, lambda msg: ball_receiver.accept(
            msg.data, node.get_clock().now().nanoseconds, clock()), 1)
    quit_event = asyncio.Event()
    gateway = attach(node, clock=clock, selected_controller="gemini_walk")
    session = RunnerSession(core, path, balance, gateway, emit, controls=controls, clock=clock,
                            publish_kick=lambda edge: kick_events.publish(String(data=json.dumps(edge, allow_nan=False))),
                            set_ball_mode=set_ball_mode, ball_snapshot=ball_snapshot)
    runtime = CombinedRuntime(robot, core.executor, core.robot_id, velocity_sink=path.write,
                              path_problem=session.runtime_problem, walker_ready=path.restored_walker_ready,
                              hard_stop=lambda reason: gateway.request_stop(reason, authority=robot.gateway_stop_anchor()))
    session.attach(runtime)

    def ingest(msg):
        try:
            if len(msg.data) > 65536:
                raise ValueError("field event exceeds the size limit")
            core.ingest(json.loads(msg.data))
        except (ValueError, TypeError, OverflowError):
            return  # Invalid input cannot renew the last valid authority.

    node.create_subscription(String, EVENT_TOPIC, ingest, 8)
    node.create_subscription(String, evidence.GUARD_STATUS_TOPIC, lambda msg: session.on_guard(msg.data), 1)
    node.create_subscription(String, evidence.NATIVE_AUTHORITY_TOPIC, lambda msg: path.native_status(msg.data),
                             native_authority_qos())
    loop = asyncio.get_running_loop()
    pending = bytearray()
    inventory_task = None

    async def watch_walk_owner():
        while True:
            await sample_walker(balance, robot)
            await asyncio.sleep(.05)

    def terminate():
        try:
            runtime.stop("operator Stop or input closed")
        finally:
            quit_event.set()

    def read_operator():
        block = os.read(sys.stdin.fileno(), 4096)
        if not block or len(pending) + len(block) > 16384:
            terminate()
            return
        pending.extend(block)
        while b"\n" in pending:
            line, _, rest = pending.partition(b"\n")
            pending[:] = rest
            try:
                message = json.loads(line)
            except (ValueError, TypeError) as exc:
                session.reply({}, {"accepted": False, "reason": str(exc)[:180]})
                continue
            if session.handle(message):
                terminate()
                return

    reader = opened = False
    exit_code = 0
    try:
        await asyncio.wait_for(runtime.open(), timeout=10)
        opened = True
        inventory_task = asyncio.create_task(watch_walk_owner())
        loop.add_reader(sys.stdin.fileno(), read_operator)
        reader = True
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, terminate)
        emit({"event": "ready", "armed": False, "side": args.side, "robot_id": args.robot_id})
        while not quit_event.is_set() and rclpy.ok():
            started = clock()
            for _ in range(10):
                spin.spin_once(timeout_sec=0)
            session.tick()
            if runtime.stop_reason:
                break
            await asyncio.sleep(max(0, TICK_S - (clock() - started)))
        if runtime.stop_reason and not quit_event.is_set():
            exit_code = 3
    finally:
        if reader:
            loop.remove_reader(sys.stdin.fileno())
        if inventory_task is not None:
            inventory_task.cancel()
            await asyncio.gather(inventory_task, return_exceptions=True)
        try:
            try:
                if opened:
                    runtime.stop("combined runtime closed")
                    await session.close(clock())
                    stop_result = await await_gateway_stop(gateway, spin.spin_once, rclpy.ok)
                    emit(stop_result)
                    if stop_result["uncertain"]:
                        exit_code = 3
            finally:
                await runtime.close()
        finally:
            board_reader.close()
            spin.shutdown()
            node.destroy_node()
            if opened:
                await robot.shutdown()
            else:
                await robot.disconnect()
    return exit_code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True, help="prepared and supervised combined native session")
    parser.add_argument("--robot-id", choices=("pupper-1", "pupper-2"), required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--producer-id", required=True)
    parser.add_argument("--field-config", required=True, type=Path)
    parser.add_argument("--operator-controls", action="store_true",
                        help="require robot-issued tickets on stdin and enable per-robot teleop")
    args = parser.parse_args(argv)
    if os.environ.get("PUPPER_SESSION_MOTION_MODE") != "fc_walk_kick":
        parser.error("select the fc_walk_kick release environment first")
    # Two launch attempts cannot share one robot runtime even during DDS discovery.
    with (args.run_dir / "fc-runner.lock").open("a") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
