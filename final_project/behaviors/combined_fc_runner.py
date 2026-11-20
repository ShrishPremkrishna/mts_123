#!/usr/bin/env python3
"""One field executor, guarded walker and leg-kick owner in the fc_walk_kick release.

Opening this runner requires a previously selected, supervised native session.
It does not home, start a controller, enable the HTTP server, or arm field play.
Normalized field/intent input uses /fc/normalized_events; local stdin owns arm,
resume, status and Stop. EOF ends the session. Times are Linux monotonic seconds.
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
from final_project.behaviors.board_imu import BoardReceipt, MatchImu
from final_project.behaviors.operator_drive import OperatorDrive
from follow_me_pupper.robot import onboard_ball
from final_project.pupper_fc.model import zero
from final_project.behaviors import command_path_evidence as evidence
from final_project.behaviors.fc_executor_bridge import (
    BridgeCore, DEADLINE_CONTRACT, EVENT_TOPIC, EventSink, TICK_S, load_fields)

NAME = "fc_walk_kick_runner"
WALKER = "/gemini_walk"


def kick_event(prior, handoff, step, now):
    """Report dispatch once; success requires the active clip to confirm and yield the walker."""
    if step.request is not None:
        return {"state": "engaged", "robot_id": step.request.robot_id, "epoch": step.request.epoch,
                "attempt_id": step.request.attempt_id, "issued_at": step.request.issued_at}
    if prior is not None and handoff.active is None and step.decision.phase == "kick_complete":
        return {"state": "completed", "robot_id": prior.robot_id, "epoch": prior.epoch,
                "attempt_id": prior.attempt_id, "completed_at": now}
    return None


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
        # A zero command may tolerate an absent walker even during ordinary
        # standby. Alternating that check with the strict camera-sync check
        # must not enumerate the graph twice on every control tick.
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
            # Discard any success from the other check after a fault. During
            # a kick transfer, recheck discovery on each change of mode until
            # the replacement walker takes ownership.
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
                return  # Never attempt a competing source handshake.
        if moving:
            native_deadline = self.native["sampled_mono_ns"] / 1e9 + evidence.NATIVE_AUTHORITY_MAX_AGE_S
            command = replace(command, expires_at=min(command.expires_at, self.frame_deadline, native_deadline))
        payload, actuating, reason = self.core.deadline.encode(command)
        if payload is not None:
            self.publish(payload)
        if hold and payload is None:
            raise RuntimeError(reason or "camera hold could not publish zero")
        if moving and (reason or not actuating):
            # Encoder already substituted and published a zero if the camera
            # expired in the few instructions since check(). Other faults propagate.
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


def sync_match_balance(balance, core, path, runtime, controls, emit):
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
        robot = runtime.robot
        if runtime.handoff.active is not None or robot.current is not None or not robot.commands.empty():
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
    if (controls is None or runtime.stop_reason or runtime.robot.fallen or runtime.robot.motion_block_reason()
            or runtime.robot.stops != balance.stop_generation or runtime.handoff.active is not None
            or runtime.robot.current is not None or not runtime.robot.commands.empty()
            or core.executor.armed or controls.match_epoch is not None):
        raise ValueError("IMU resume requires an idle, stopped match owner")
    if not gateway_stop_ready(gateway, runtime.robot.gateway_stop_anchor()):
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


async def request_native_resume(node, resume_service, generation, incident):
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParametersAtomically
    name = WALKER + "/resume_balance"
    owned = dict(node.get_service_names_and_types_by_node(WALKER.lstrip("/"), "/"))
    if (node.count_services(name) != 1 or owned.get(name) != ["rcl_interfaces/srv/SetParametersAtomically"]
            or not resume_service.service_is_ready()):
        raise ValueError("Expected one balance resume service on the selected walker")
    request = SetParametersAtomically.Request()
    for name, value in (("generation", generation), ("incident", incident), ("issued_mono_ns", time.monotonic_ns())):
        request.parameters.append(Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=value)))
    future = resume_service.call_async(request)
    try:
        while not future.done():
            await asyncio.sleep(.002)
        response = future.result().result
        if not response.successful:
            raise ValueError("Walker refused balance resume: " + response.reason)
    finally:
        if not future.done():
            resume_service.remove_pending_request(future)


async def imu_resume_action(message, core, path, runtime, gateway, controls, balance, release_native):
    check_balance_resume(message, core, path, runtime, gateway, controls, balance)
    expires = controls.consume_ticket(message, core.clock())
    revision = controls.authority_revision
    binding = balance.native_binding
    if balance.native_release:
        # No soccer owner while the non-actuating native permission is in flight.
        # A timeout leaves authority paused, including when the response was lost.
        await asyncio.wait_for(release_native(balance.generation, balance.incident), max(0., expires - core.clock()))
    remaining = expires - core.clock()
    if remaining <= 0 or controls.authority_revision != revision:
        raise ValueError("IMU resume authorization expired or changed while awaiting native confirmation")
    await asyncio.wait_for(sample_walker(balance, runtime.robot), remaining)
    check_balance_resume(message, core, path, runtime, gateway, controls, balance)
    if core.clock() >= expires or controls.authority_revision != revision or balance.native_binding != binding:
        raise ValueError("IMU resume authorization expired or changed while awaiting native confirmation")
    return operator_action(message, core, path, runtime, gateway, controls, balance=balance,
                           resume_confirmed=(binding, revision, expires))


def operator_action(message, core, path, runtime, gateway, controls, *, set_ball_mode=None, balance=None,
                    resume_confirmed=None):
    action = message.get("command")
    robot = runtime.robot
    if action in ("imu_pause", "imu_resume") and balance is None:
        raise ValueError("Board health proof is unavailable")
    if balance is not None:
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
            if (not resume_confirmed or resume_confirmed[:2] != (balance.native_binding, controls.authority_revision)
                    or core.clock() >= resume_confirmed[2]):
                raise ValueError("IMU resume requires an explicit current operator and native check")
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
        stable = (runtime.handoff.active is None and robot.current is None and robot.commands.empty()
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
        if balance is not None:
            balance.end()
        if runtime.handoff.active is not None or robot.current is not None or not robot.commands.empty():
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
        if balance is not None:
            path.sync_balance()
            if runtime.stop_reason or robot.fallen or robot.motion_block_reason():
                raise ValueError("Stop, fall or native fault prevents field authority")
            if action == "imu_resume":
                balance.check_resume(message)
                if balance.native_binding != resume_confirmed[0] or core.clock() >= resume_confirmed[2]:
                    raise ValueError("IMU resume evidence changed before arm")
            elif balance.state != "healthy":
                raise ValueError(balance.reason)
        hold_id = balance.hold_id if balance is not None else None
        reply = core.operator({**message, "command": "arm"})
        if reply["accepted"] and balance is not None:
            balance.begin(message["epoch"], message.get("pitch"), robot.stops)
            if action == "imu_resume":
                if balance.native_release:
                    balance.released = balance.native_binding
                reply.update(state="armed", epoch=message["epoch"], hold_id=hold_id, imu=balance.snapshot())
            elif balance.native_release and balance.released != balance.native_binding:
                # A new connection cannot inherit permission from an earlier match.
                balance.pause(message["epoch"])
                path.sync_balance()
                return {**reply, "state": "paused", "epoch": balance.epoch,
                        "hold_id": balance.hold_id, "imu": balance.snapshot()}
        if controls is not None and controls.mode == "teleop":
            # Manual drive owns the walker exclusively. Reuse the executor's epoch replay fence
            # without arming its autonomous control path.
            if reply["accepted"]:
                core.executor.stop()
                controls.invalidate()
                controls.match_epoch = message["epoch"]
        return reply
    return core.operator(message)


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
    from rcl_interfaces.srv import SetParametersAtomically
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
    kick_events = node.create_publisher(String, "/fc/kick_result", 4)
    resume_service = node.create_client(SetParametersAtomically, WALKER + "/resume_balance")
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
    ball_receiver = onboard_ball.OnboardBallReceiver()
    set_ball_mode = None
    if controls is not None:
        ball_modes = node.create_publisher(String, onboard_ball.MODE_TOPIC, 1)

        def set_ball_mode(mode):
            ball_modes.publish(String(data=mode))

        def accept_ball(msg):
            ball_receiver.accept(msg.data, node.get_clock().now().nanoseconds, clock())

        node.create_subscription(String, onboard_ball.TOPIC, accept_ball, 1)
    quit_event = asyncio.Event()
    gateway = attach(node, clock=clock, selected_controller="gemini_walk")

    def runtime_problem(during_kick):
        sync_balance()
        if balance.state != "healthy" and (during_kick or runtime.handoff.active is not None):
            return balance.reason
        problem = path.check(True) if during_kick else path.walking_problem()
        if problem and problem[0] == "camera" and not during_kick and runtime.handoff.active is None:
            if not core.executor.external_holds[core.robot_id]:
                core.hold(True, problem[1])
            return None  # Normal walk pauses immediately. An in-progress kick still requires its evidence.
        return problem[1] if problem else None

    runtime = CombinedRuntime(robot, core.executor, core.robot_id, velocity_sink=path.write,
                              path_problem=runtime_problem, walker_ready=path.restored_walker_ready,
                              hard_stop=lambda reason: gateway.request_stop(reason, authority=robot.gateway_stop_anchor()))

    def sync_balance():
        sync_match_balance(balance, core, path, runtime, controls, emit)

    path.balance, path.sync_balance = balance, sync_balance

    def sync_camera_hold():
        active_clip = runtime.handoff.active is not None
        problem = path.check(active_clip)
        held = bool(problem and problem[0] == "camera")
        if (held or problem is None) and held != core.executor.external_holds[core.robot_id]:
            core.hold(held, problem[1] if held else "camera clear")
        if controls is not None:
            controls.observe_path(problem is None and not active_clip and balance.state in ("healthy", "ready_to_resume"),
                                  path.native.get("generation") if path.native else None)

    def on_guard(msg):
        path.guard_status(msg.data)
        if (core.executor.armed or (controls is not None and controls.mode == "teleop")) and path.guard_error:
            runtime.stop("invalid guard evidence: " + path.guard_error)
        sync_camera_hold()

    def ingest(msg):
        try:
            if len(msg.data) > 65536:
                raise ValueError("field event exceeds the size limit")
            core.ingest(json.loads(msg.data))
        except (ValueError, TypeError, OverflowError):
            return  # Invalid input cannot renew the last valid authority.

    node.create_subscription(String, EVENT_TOPIC, ingest, 8)
    node.create_subscription(String, evidence.GUARD_STATUS_TOPIC, on_guard, 1)
    node.create_subscription(String, evidence.NATIVE_AUTHORITY_TOPIC, lambda msg: path.native_status(msg.data),
                             native_authority_qos())
    loop = asyncio.get_running_loop()
    pending = bytearray()
    resume_task = inventory_task = None

    async def watch_walk_owner():
        while True:
            await sample_walker(balance, robot)
            await asyncio.sleep(.05)

    def operator_reply(message, reply):
        context = message if isinstance(message, dict) else {}
        emit({**reply, "event": "operator", "command": context.get("command"),
              "ticket": context.get("ticket"), "robot_id": core.robot_id,
              "decision_mono_ns": time.monotonic_ns(), "decision_wall_ns": time.time_ns(),
              "guard_sample_mono_ns": path.guard.get("sampled_at_monotonic_ns") if path.guard else None,
              "frame_sequence": path.guard.get("sequence") if path.guard else None})

    async def resume_match(message):
        try:
            reply = await imu_resume_action(message, core, path, runtime, gateway, controls, balance,
                                            lambda g, i: request_native_resume(node, resume_service, g, i))
        except (OSError, ValueError, TypeError, OverflowError, RuntimeError, TimeoutError) as exc:
            reply = {"accepted": False, "reason": str(exc)[:180] or "Walker balance resume timed out; match held"}
        operator_reply(message, reply)

    def terminate():
        try:
            runtime.stop("operator Stop or input closed")
        finally:
            quit_event.set()

    def read_operator():
        nonlocal resume_task
        block = os.read(sys.stdin.fileno(), 4096)
        if not block or len(pending) + len(block) > 16384:
            terminate()
            return
        pending.extend(block)
        while b"\n" in pending:
            line, _, rest = pending.partition(b"\n")
            pending[:] = rest
            message = {}
            try:
                message = json.loads(line)
                if not isinstance(message, dict):
                    raise ValueError("expected a JSON object")
                action = message.get("command")
                if action in ("stop", "quit"):
                    terminate()
                    return
                if action == "status":
                    sync_balance()
                    problem = path.check(runtime.handoff.active is not None)
                    emit({"event": "status", "armed": core.executor.armed, "path_problem": problem,
                          "stop": runtime.stop_reason, "imu": balance.snapshot()})
                    continue
                if controls is not None:
                    sync_camera_hold()
                if action == "imu_resume":
                    if resume_task is not None and not resume_task.done():
                        raise ValueError("A balance resume is already pending")
                    resume_task = asyncio.create_task(resume_match(message))
                    continue
                reply = operator_action(message, core, path, runtime, gateway, controls,
                                        set_ball_mode=set_ball_mode, balance=balance)
            except (ValueError, TypeError, OverflowError) as exc:
                reply = {"accepted": False, "reason": str(exc)[:180]}
            operator_reply(message, reply)

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
        last_phase = None
        next_ball_report = 0.0
        while not quit_event.is_set() and rclpy.ok():
            started = clock()
            for _ in range(10):
                spin.spin_once(timeout_sec=0)
            sync_balance()
            sync_camera_hold()
            if runtime.handoff.active is not None and runtime.handoff.clip_started:
                path.clip_started()
            now = clock()
            prior_job = runtime.handoff.active
            step = runtime.tick(manual_command=controls.command(now) if controls and balance.state == "healthy" else None)
            edge = kick_event(prior_job, runtime.handoff, step, clock())
            if edge is not None:
                kick_events.publish(String(data=json.dumps(edge, allow_nan=False)))
                emit({"event": "kick", **edge})
            if controls is not None:
                if now >= next_ball_report:
                    next_ball_report = now + .10
                    emit({"event": "onboard_ball", "robot_id": core.robot_id,
                          "onboard_ball": ball_receiver.snapshot(node.get_clock().now().nanoseconds, now)})
                if core.executor.external_holds[core.robot_id]:
                    controls.observe_path(False, controls.generation)
                controls.observe_output(step.decision.command)
                offer = (controls.offer(now, resume_cycle(core, now) if controls.ready else None)
                         if now >= controls.next_issue else None)
                if offer is not None:
                    emit({**offer, "imu": balance.snapshot()})
            if step.decision.phase != last_phase:
                last_phase = step.decision.phase
                emit({"event": "phase", "phase": last_phase, "reason": step.decision.reason})
            if runtime.stop_reason:
                break
            await asyncio.sleep(max(0, TICK_S - (clock() - started)))
        if runtime.stop_reason and not quit_event.is_set():
            exit_code = 3
    finally:
        if reader:
            loop.remove_reader(sys.stdin.fileno())
        if resume_task is not None:
            resume_task.cancel()
            await asyncio.gather(resume_task, return_exceptions=True)
        if inventory_task is not None:
            inventory_task.cancel()
            await asyncio.gather(inventory_task, return_exceptions=True)
        try:
            try:
                if opened:
                    runtime.stop("combined runtime closed")
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
                        help="enable robot-scoped keyboard tickets on the operator stream")
    args = parser.parse_args(argv)
    if os.environ.get("PUPPER_SESSION_MOTION_MODE") != "fc_walk_kick":
        parser.error("select the fc_walk_kick release environment first")
    # Two launch attempts cannot share one robot runtime even during DDS discovery.
    with (args.run_dir / "fc-runner.lock").open("a") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
