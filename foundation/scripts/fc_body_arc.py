#!/usr/bin/env python3
"""One resident, walk-only trial owner for the verified FC release.

Dry run: fc_body_arc.py plan --speed .20 --radius 1.4 --sweep-degrees 35
On the Pi: fc_body_arc.py session --release "$FC_RELEASE" --run-dir "$FC_RUN"
Launch after supervised walker activation. Send JSON lines on stdin: status;
prepare with speed, radius, sweep_degrees; run with the returned token; stop.
Stop or EOF latches the gateway. A finished arc stays in standing walk at zero.
Distances and heading printed by 'plan' are nominal body-command integrals.
"""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
from dataclasses import asdict, dataclass
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace
import uuid


@dataclass(frozen=True)
class Arc:
    speed: float
    radius: float
    sweep_degrees: float

    def __post_init__(self):
        if any(
            type(v) not in (float, int) or not math.isfinite(v)
            for v in (self.speed, self.radius, self.sweep_degrees)
        ):
            raise ValueError("arc parameters must be finite numbers")
        if not (0 < self.speed <= 0.30 and self.radius > 0 and 0 < abs(self.sweep_degrees) <= 90):
            raise ValueError("require 0 < speed <= 0.30 m/s, positive radius and sweep up to 90 degrees")
        if not 0 < self.length <= 2.5 or self.peak == 0 or self.speed / self.radius > 1 or self.duration > 15:
            raise ValueError("arc exceeds 1 rad/s yaw, 2.5 m path length or 15 s duration")

    @property
    def length(self):
        return self.radius * abs(math.radians(self.sweep_degrees))

    @property
    def peak(self):
        # A short arc is triangular. Longer arcs cruise between matched ramps.
        return min(self.speed, math.sqrt(self.length * 0.4))

    @property
    def duration(self):
        return self.length / self.peak + self.peak / 0.4

    def velocity(self, elapsed):
        vx = max(0.0, min(self.peak, 0.4 * elapsed, 0.4 * (self.duration - elapsed)))
        return vx, math.copysign(vx / self.radius, self.sweep_degrees)

    def describe(self):
        angle = math.radians(abs(self.sweep_degrees))
        return dict(
            **asdict(self),
            duration_s=self.duration,
            path_m=self.length,
            nominal_forward_m=self.radius * math.sin(angle),
            nominal_left_m=math.copysign(2 * self.radius * math.sin(angle / 2) ** 2, self.sweep_degrees),
        )


class Trials:
    """Operator commands create a single bounded attempt. Clearance cannot restart it."""

    def __init__(self):
        self.prepared = self.running = None
        self.token = self.prepared_until = self.started = self.previous_tick = None
        self.await_zero_since = None
        self.latest_feedback = None
        self.saw_curved_feedback = False
        self.peak_observed = [0.0, 0.0]
        self.recent_commands = deque(maxlen=16)
        self.guard_stamp = self.clearance = None

    @property
    def armed(self):
        return self.running is not None

    def observe(self, now, values):
        if len(values) != 960 or not all(math.isfinite(v) for v in values[6:9]):
            self.latest_feedback = None
            return
        self.latest_feedback = (now, tuple(values[6:9]))
        if self.running and now >= self.started:
            vx, vy, wz = self.latest_feedback[1]
            if (
                abs(vy) <= 0.025
                and wz * self.running.sweep_degrees > 0
                and any(
                    0 <= now - at <= 0.20
                    and forward >= 0.5 * self.running.peak
                    and vx >= 0.5 * forward
                    and abs(vx - forward) <= 0.03
                    and abs(wz) >= 0.5 * abs(yaw)
                    and abs(wz - yaw) <= 0.06
                    for at, forward, yaw in self.recent_commands
                )
            ):
                self.saw_curved_feedback = True
            self.peak_observed[0] = max(self.peak_observed[0], vx)
            self.peak_observed[1] = max(self.peak_observed[1], abs(wz))

    def feedback_ready(self, now, *, zero=False):
        if self.latest_feedback is None:
            return False
        at, body = self.latest_feedback
        return 0 <= now - at <= 0.20 and (not zero or all(abs(v) <= 0.02 for v in body))

    def retire(self):
        self.prepared = self.running = self.token = None

    def guard_sample(self, status, max_gap_s):
        """The current clearance began after the last hold, even across a dropped status."""
        stamp, clearance = status["sampled_at_monotonic_ns"], status["clearance_mono_ns"]
        changed = self.guard_stamp is not None and (
            self.clearance != clearance or not 0 <= stamp - self.guard_stamp <= max_gap_s * 1e9
        )
        self.guard_stamp, self.clearance = stamp, clearance
        return "guard clearance changed or status gap exceeded freshness" if changed else None

    def operator(self, msg, now, problem):
        if problem or not self.feedback_ready(now, zero=True):
            self.prepared = self.token = None
            raise ValueError(problem or "waiting for fresh zero walker feedback")
        if self.armed or self.await_zero_since is not None:
            raise ValueError("previous attempt is active or awaiting zero")
        if msg.get("command") == "prepare":
            self.prepared = self.token = None
            arc = Arc(msg["speed"], msg["radius"], msg["sweep_degrees"])
            self.prepared, self.token, self.prepared_until = arc, uuid.uuid4().hex, now + 15
            return dict(event="prepared", token=self.token, **arc.describe())
        if msg.get("command") != "run":
            raise ValueError("expected prepare, run, status or stop")
        if self.token is None or msg.get("token") != self.token or now >= self.prepared_until:
            raise ValueError("stale or incorrect attempt token; prepare again")
        self.running, self.prepared, self.token = self.prepared, None, None
        self.started = self.previous_tick = now
        self.saw_curved_feedback = False
        self.peak_observed = [0.0, 0.0]
        self.recent_commands.clear()
        return dict(event="started", **self.running.describe())

    def end(self, now, reason=None):
        result = dict(
            event="cancelled" if reason else "profile_ended",
            reason=reason,
            curved_walker_feedback_seen=self.saw_curved_feedback,
            peak_policy_forward_mps=self.peak_observed[0],
            peak_policy_abs_yaw_rps=self.peak_observed[1],
        )
        self.retire()
        self.await_zero_since = now
        return result

    def tick(self, now, problem):
        if problem:
            self.prepared = self.token = None
        if self.armed:
            elapsed = now - self.started
            if not 0 <= now - self.previous_tick < 0.15:
                problem = "control loop missed a command lifetime"
            if not self.feedback_ready(now):
                problem = "walker feedback unavailable"
            if not problem and not self.saw_curved_feedback and elapsed >= min(1.0, self.running.duration):
                problem = "walker never acknowledged forward and signed yaw together"
            self.previous_tick = now
            if problem or elapsed >= self.running.duration:
                return 0.0, 0.0, now + 0.15, self.end(now, problem)
            vx, wz = self.running.velocity(elapsed)
            self.recent_commands.append((now, vx, wz))
            return vx, wz, min(now + 0.15, self.started + self.running.duration), None
        if self.await_zero_since is not None:
            if self.feedback_ready(now, zero=True) and self.latest_feedback[0] > self.await_zero_since:
                self.await_zero_since = None
                return 0.0, 0.0, now + 0.15, dict(event="walker_zero_observed")
            if now - self.await_zero_since >= 1.0:
                raise RuntimeError("walker zero unconfirmed within 1 s")
        return 0.0, 0.0, now + 0.15, None


async def serve(run_dir, release):
    # Import the frozen release's graph/clock checks, camera veto and Stop gateway.
    sys.path[:0] = [str(release), str(release / "gemini_pupper/robot"), str(release / "gemini_pupper/lab")]
    from final_project.behaviors import combined_fc_runner as fc
    from final_project.behaviors.fc_executor_bridge import DeadlineEncoder
    from final_project.pupper_fc.model import Command
    from foundation.hardware.fc_walk_kick_release import verify_running_session
    from foundation.hardware.software_start import attach

    if Path(fc.__file__).resolve() != release / "final_project/behaviors/combined_fc_runner.py":
        raise RuntimeError("combined runner import is outside the selected release")
    verify_running_session(run_dir, release)
    os.environ["PUPPER_SESSION_MANIFEST"] = str(run_dir / "runtime-manifest.json")
    from robot_api import Robot
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Float32MultiArray, String

    owner = json.loads((run_dir / "owned_session.json").read_text())["session_id"]
    trial = Trials()
    clock = time.monotonic
    core = SimpleNamespace(clock=clock, deadline=DeadlineEncoder(), executor=trial)
    robot = Robot(ros_signal_handlers=False, observe_only=True)
    node = rclpy.create_node(fc.NAME)
    spin = SingleThreadedExecutor()
    spin.add_node(node)
    publisher = node.create_publisher(String, fc.protocol.REQUEST_TOPIC, 1)
    path = fc.GuardedPath(core, node, lambda v: publisher.publish(String(data=v)))
    gateway = attach(node, clock=clock, selected_controller="gemini_walk")
    stop = asyncio.Event()
    buffer = bytearray()
    reasons = []

    def quit(reason):
        reasons.append(reason)
        stop.set()

    def send(event):
        print(json.dumps(dict(event, reported_mono_ns=time.monotonic_ns())), flush=True)

    def on_guard(msg):
        path.guard_status(msg.data)
        reason = path.guard_error or trial.guard_sample(path.guard, fc.evidence.GUARD_STATUS_MAX_AGE_S)
        if trial.armed or trial.prepared is not None:
            issue = path.check()
            reason = reason or (issue[1] if issue else None)
        if reason:
            trial.prepared = trial.token = None
            if trial.armed:
                send(trial.end(clock(), reason))

    observation_topic = "/gemini_walk/observation"
    feedback_owner = None
    owner_check_at = -math.inf
    owner_ok = False

    def walker_publisher_ok():
        nonlocal feedback_owner, owner_check_at, owner_ok
        if clock() - owner_check_at >= 0.10:
            owner_check_at = clock()
            endpoints = node.get_publishers_info_by_topic(observation_topic)
            owner_ok = (
                len(endpoints) == 1
                and endpoints[0].node_name == "gemini_walk"
                and (
                    endpoints[0].node_namespace == "/"
                    and endpoints[0].topic_type == "std_msgs/msg/Float32MultiArray"
                )
            )
            if owner_ok:
                gid = tuple(endpoints[0].endpoint_gid)
                feedback_owner = feedback_owner or gid
                owner_ok = gid == feedback_owner
        return owner_ok

    def on_observation(msg, info):
        # Native observations have no header. DDS stamps the publication on the same Pi.
        stamp = info.get("source_timestamp")
        age = (time.time_ns() - stamp) * 1e-9 if type(stamp) is int else math.inf
        if not 0 <= age <= 0.20:
            trial.latest_feedback = None
            return
        trial.observe(clock() - age, msg.data)

    def readiness():
        if not opened:
            return "waiting for walker acknowledgement of this source's first zero"
        if robot.fallen or robot.stops != generation or robot.motion_block_reason():
            return "robot Stop, fall or controller fault"
        imu = robot.imu[-1] if robot.imu else None
        if imu is None or not 0 <= clock() - imu[0] < 0.25 or not all(math.isfinite(v) for v in imu[1:]):
            return "waiting for fresh IMU feedback"
        if not fc.gateway_stop_ready(gateway, robot.gateway_stop_anchor()):
            return "gateway Stop channel not ready"
        if not walker_publisher_ok():
            return "walker feedback has a missing, competing or replaced publisher"
        issue = path.check()
        return issue[1] if issue else None

    def read_operator():
        block = os.read(sys.stdin.fileno(), 4096)
        if not block or len(buffer) + len(block) > 16384:
            quit("operator input closed or oversized")
            return
        buffer.extend(block)
        while b"\n" in buffer:
            line, _, remainder = buffer.partition(b"\n")
            buffer[:] = remainder
            try:
                msg = json.loads(line)
                if not isinstance(msg, dict):
                    raise ValueError("expected JSON object")
                action = msg.get("command")
                if action in ("stop", "quit"):
                    quit("operator Stop")
                    return
                if action == "status":
                    send(
                        dict(
                            event="status",
                            moving=trial.armed,
                            problem=readiness(),
                            walker_zero=trial.feedback_ready(clock(), zero=True),
                        )
                    )
                else:
                    send(trial.operator(msg, clock(), readiness()))
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                send(dict(event="refused", reason=str(exc)))

    node.create_subscription(String, fc.evidence.GUARD_STATUS_TOPIC, on_guard, 1)
    node.create_subscription(
        String,
        fc.evidence.NATIVE_AUTHORITY_TOPIC,
        lambda m: path.native_status(m.data),
        fc.native_authority_qos(),
    )
    node.create_subscription(Float32MultiArray, observation_topic, on_observation, 1)
    loop = asyncio.get_running_loop()
    opened = False
    reader = False
    try:
        await asyncio.wait_for(robot.ready.wait(), 8)
        generation = robot.stops
        await asyncio.wait_for(robot.wait_for_initial_stop_observation(), 3)
        discovery_end = clock() + 4
        while clock() < discovery_end:
            spin.spin_once(timeout_sec=0.01)
            path.check()  # Refresh graph ownership; source binding needs the first genuine zero.
            if path.guard_error == "guard is bound to another bridge source instance":
                raise RuntimeError("guard belongs to the previous client; coordinate a guard restart")
            if path.native and path.native["bound_source_instance"]:
                raise RuntimeError("walker already belongs to another source")
            if (
                path.graph is None
                and path.guard
                and path.guard_error is None
                and path.native_error is None
                and walker_publisher_ok()
                and trial.feedback_ready(clock(), zero=True)
                and fc.gateway_stop_ready(gateway, robot.gateway_stop_anchor())
            ):
                break
            await asyncio.sleep(0)
        else:
            facts = dict(
                graph=path.graph,
                guard=path.guard_error or bool(path.guard),
                native=path.native_error or bool(path.native),
                walker_feedback_owner=walker_publisher_ok(),
                stop_channel=fc.gateway_stop_ready(gateway, robot.gateway_stop_anchor()),
            )
            raise RuntimeError("unconfirmed read-only startup checks: " + json.dumps(facts))
        states = await robot.controller_states()
        if (
            not states
            or {n for n in robot.leg_controllers if states.get(n) == "active"} != {"gemini_walk"}
            or states.get("trick_leg_kick") != "inactive"
            or robot.motion_block_reason()
            or robot.fallen
            or robot.stops != generation
        ):
            raise RuntimeError("requires a healthy, deliberately activated walker and inactive kick")
        # The native controller reports its binding only in response to a command.
        # Probe with genuine zero after the read-only checks; it may reveal an older owner.
        loop.add_reader(sys.stdin.fileno(), read_operator)
        reader = True
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, quit, "process interrupted")
        send(dict(event="binding", session=owner, source_instance=core.deadline.source_instance))
        while not stop.is_set() and rclpy.ok():
            started = clock()
            for _ in range(10):
                spin.spin_once(timeout_sec=0)
            if robot.fallen or robot.stops != generation or robot.motion_block_reason():
                quit("robot Stop, fall or controller fault")
                break
            if not opened and path.native and path.native["bound_source_instance"]:
                if path.native["bound_source_instance"] != core.deadline.source_instance:
                    raise RuntimeError("walker is bound to a different owner; arc refused")
                if path.native_error is None and path.check() is None:
                    opened = True
                    send(dict(event="idle", session=owner, source_instance=core.deadline.source_instance))
            vx, wz, expiry, event = trial.tick(clock(), readiness())
            path.write(Command(vx=vx, wz=wz, expires_at=expiry))
            if event:
                send(event)
            await asyncio.sleep(max(0, fc.TICK_S - (clock() - started)))
    finally:
        if reader:
            loop.remove_reader(sys.stdin.fileno())
        try:
            if opened:
                gateway.request_stop(
                    reasons[0] if reasons else "arc tool closed", authority=robot.gateway_stop_anchor()
                )
                result = await fc.await_gateway_stop(gateway, spin.spin_once, rclpy.ok)
                send(result)
                if not result.get("confirmed"):
                    raise RuntimeError("gateway Stop unconfirmed; inspect before further trials")
            elif reader:
                send(
                    dict(
                        event="observer_closed",
                        reason="walker ownership never confirmed; verify physical Stop",
                    )
                )
        finally:
            spin.shutdown()
            node.destroy_node()
            await robot.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    plan = modes.add_parser("plan")
    plan.add_argument("--speed", type=float, required=True)
    plan.add_argument("--radius", type=float, required=True)
    plan.add_argument("--sweep-degrees", type=float, required=True)
    session = modes.add_parser("session")
    session.add_argument("--release", type=Path, required=True)
    session.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.mode == "plan":
        print(json.dumps(Arc(args.speed, args.radius, args.sweep_degrees).describe(), indent=2))
        return
    run_dir = args.run_dir.resolve(strict=True)
    if (
        os.environ.get("PUPPER_SESSION_MOTION_MODE") != "fc_walk_kick"
        or os.environ.get("ROS_DOMAIN_ID") != "168"
    ):
        parser.error("source the selected combined release environment first")
    with (run_dir / "fc-runner.lock").open("a") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        asyncio.run(serve(run_dir, args.release.resolve(strict=True)))


if __name__ == "__main__":
    main()
