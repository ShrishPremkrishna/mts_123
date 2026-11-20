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
from final_project.behaviors import command_path_evidence as evidence
from final_project.behaviors.fc_executor_bridge import (
    BridgeCore, DEADLINE_CONTRACT, EVENT_TOPIC, EventSink, TICK_S, load_fields)

NAME = "fc_walk_kick_runner"
WALKER = "/gemini_walk"


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
        self.graph_at = -float("inf")
        self.graph_mode = self.graph = None
        self.last_walk_generation = 0
        self.return_after_generation = 0

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
        if self.graph_mode != during_kick or now - self.graph_at >= .10:
            self.graph = evidence.guard_publishers_problem(
                self.node, "/" + NAME, command_contract=DEADLINE_CONTRACT,
                native_node=WALKER, detector_node="/viser_camera", allow_absent_native=during_kick)
            self.graph_mode, self.graph_at = during_kick, now
        if self.graph:
            return self.graph
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

    def write(self, command):
        moving = any((command.vx, command.vy, command.wz))
        problem = self.check(not moving)
        if problem and (moving or problem[0] == "authority"):
            if moving:
                raise RuntimeError(problem[1])
            return  # Never attempt a competing source handshake.
        if moving:
            command = replace(command, expires_at=min(command.expires_at, self.frame_deadline,
                self.native["sampled_mono_ns"] / 1e9 + evidence.NATIVE_AUTHORITY_MAX_AGE_S))
        payload, actuating, reason = self.core.deadline.encode(command)
        if payload is not None:
            self.publish(payload)
        if moving and (reason or not actuating):
            raise RuntimeError(reason or "walk deadline expired at publication")


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


async def run(args):
    from foundation.hardware.fc_walk_kick_release import verify_running_session
    from foundation.hardware.software_start import attach
    # Refuse stale installs or an unowned manager before constructing a robot client.
    verify_running_session(args.run_dir, ROOT)
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
    emit = EventSink()
    quit_event = asyncio.Event()
    gateway = attach(node, clock=clock, selected_controller="gemini_walk")

    def runtime_problem(during_kick):
        problem = path.check(during_kick)
        if problem and problem[0] == "camera" and not during_kick and core.executor.external_holds[core.robot_id]:
            return None  # Executor is held; the sink independently fences positive output.
        return problem[1] if problem else None

    runtime = CombinedRuntime(robot, core.executor, core.robot_id, velocity_sink=path.write,
                              path_problem=runtime_problem, walker_ready=path.restored_walker_ready,
                              hard_stop=lambda reason: gateway.request_stop(reason, authority=robot.gateway_stop_anchor()))

    def sync_camera_hold():
        active_clip = runtime.handoff.active is not None
        problem = path.check(active_clip)
        held = bool(problem and problem[0] == "camera")
        if (held or problem is None) and held != core.executor.external_holds[core.robot_id]:
            core.hold(held, problem[1] if held else "camera clear")

    def on_guard(msg):
        path.guard_status(msg.data)
        if core.executor.armed and path.guard_error:
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
                if not isinstance(message, dict):
                    raise ValueError("expected a JSON object")
                action = message.get("command")
                if action in ("stop", "quit"):
                    terminate()
                    return
                if action == "status":
                    problem = path.check(runtime.handoff.active is not None)
                    emit({"event": "status", "armed": core.executor.armed, "path_problem": problem,
                          "stop": runtime.stop_reason})
                else:
                    if action == "arm":
                        if core.executor.armed:
                            raise ValueError("field play is already armed")
                        if not gateway_stop_ready(gateway, robot.gateway_stop_anchor()):
                            raise ValueError("waiting for the selected gateway Stop channel")
                        problem = path.check(False)
                        if problem:
                            raise ValueError(problem[1])
                    emit(core.operator(message))
            except (ValueError, TypeError, OverflowError) as exc:
                emit({"accepted": False, "reason": str(exc)[:180]})

    reader = opened = False
    exit_code = 0
    try:
        await asyncio.wait_for(runtime.open(), timeout=10)
        opened = True
        loop.add_reader(sys.stdin.fileno(), read_operator)
        reader = True
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, terminate)
        emit({"event": "ready", "armed": False, "side": args.side, "robot_id": args.robot_id})
        last_phase = None
        while not quit_event.is_set() and rclpy.ok():
            started = clock()
            for _ in range(10):
                spin.spin_once(timeout_sec=0)
            sync_camera_hold()
            if runtime.handoff.active is not None and runtime.handoff.clip_started:
                path.clip_started()
            step = runtime.tick()
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
    args = parser.parse_args(argv)
    if os.environ.get("PUPPER_SESSION_MOTION_MODE") != "fc_walk_kick":
        parser.error("select the fc_walk_kick release environment first")
    # Two launch attempts cannot share one robot runtime even during DDS discovery.
    with (args.run_dir / "fc-runner.lock").open("a") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
