#!/usr/bin/env python3
"""Scenario driver and production-role entries for walk_kick_wire (started by run.py).

scenario SPEC        Start the manager backend, selected Stop gateway, production guard,
                     operator relay and field runner in this isolated domain. Act as the
                     browser, overhead field, detector and strict board receipt. Assert the
                     native model input, kick frames/execution, restored binding and Stop.
runner SPEC          combined_fc_runner.main() as the release runs it with --operator-controls.
operator SPEC OWNER  fc_operator.serve(), the relay behind the browser's Stand and heartbeat.

Both roles replace only persistent-session supervision (verify_running_session and
check_manager, which require a prepared, homed board session) with this run's own
manager identity check. Waits poll while every required child stays alive; motion
phases end when their finite synthetic input is exhausted, not on elapsed time.

Timing: the runner and relay install pass-through probes (PROBES) that record
CLOCK_MONOTONIC times around existing calls without changing arguments, results or
exceptions. The scenario keeps its own observation receipt times and joins them with
the probes and the fixture manager's native stamps in timing.json. Every time there
is on the one clock named by `clock_id`; no wall-clock alignment is assumed.
"""
from collections import deque
import contextvars
import hashlib
import math
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
ROBOT = "pupper-1"
PRODUCER = "walk-kick-wire"
SPAWNER = "/opt/ros/jazzy/lib/controller_manager/spawner"
MANAGER = "/opt/ros/jazzy/lib/controller_manager/ros2_control_node"
# Same as lab6_session.launch.py: the manager's own switch service sits behind the gateway.
BACKEND_REMAP = "controller_manager:/controller_manager/switch_controller:=/controller_manager/switch_controller_backend"
SPAWN_ARGS = ("--controller-manager", "/controller_manager", "--controller-manager-timeout", "60",
              "--service-call-timeout", "10")
ALLOWED_DEVICES = {"/dev/null", "/dev/random", "/dev/urandom"}
KICK_CONTROLLER, WALK_CONTROLLER = "trick_leg_kick", "gemini_walk"
FIELD_PERIOD_S, CAMERA_PERIOD_S, BOARD_PERIOD_S = .035, .025, .010
# Finite synthetic field frames per motion phase (one frame and one intent per period).
APPROACH_FRAMES, KICK_FRAMES, RESTORE_FRAMES = 400, 700, 400
MOVING_VX = .03
CLIP = re.compile(r"^(kick_(?:left|right)) playing frame (\d+) queued (\d+) execution ([0-9a-f]{32}) request (\S+)$")


class Failure(RuntimeError):
    pass


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def start_ticks(pid):
    return (Path("/proc") / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()[19]


def record_imports(path):
    """Record every executed module file from this checkout with its digest."""
    rows = {}
    for module in list(sys.modules.values()):
        origin = getattr(module, "__file__", None)
        if origin and Path(origin).resolve().is_relative_to(ROOT):
            rows[str(Path(origin).resolve().relative_to(ROOT))] = digest(origin)
    Path(path).write_text(json.dumps(dict(sorted(rows.items())), indent=2) + "\n")


# ----------------------------------------------------------------- timing probes

TIMING_SCHEMA = "walk_kick_wire_timing_v1"
# What each probe row means, and what it does not establish.
PROBES = {
    "rpc_send": "t_ns immediately before rclpy Client.call_async; send_returned_ns after it returned (the "
                "request was handed to rcl inside). response_ns: when the future's done callback ran in this "
                "process, at or after the response was taken. Not a server-side time. cancelled_ns instead: "
                "the caller cancelled the future locally (no response; the remote request is not cancelled).",
    "call": "Robot._call entry (t_ns) and return to its caller (exit_ns). Robot._call polls future.done() "
            "every 20 ms, so exit_ns - response_ns is local polling delay, not service time.",
    "switch_to": "Robot.switch_to entry and result; call rows carry the enclosing switch_to id.",
    "gate": "CombinedRuntime._gate evaluated between before_ns and t_ns (its own now lies inside). "
            "call=None is the check at _submit entry; a call id is the ACTIVATION_GATE check in Robot._call.",
    "submit": "CombinedRuntime._submit entry (t_ns) and return (exit_ns). It returns once robot.queue accepts "
              "the kick job; the job's switches run afterwards (see switch_to rows).",
    "kick_request": "KickRequest returned by CombinedRuntime.tick, with the executor frame and intent that "
                    "authorized it. issued_at/start_before are the request's own monotonic seconds.",
    "kick_cancel": "CombinedRuntime.tick returned a cancel for the in-flight request.",
    "phase": "runner decision phase change observed after CombinedRuntime.tick.",
    "decision_ages": "per CombinedRuntime.tick: [t_ns, executor now - own captured_at in ms, phase]. This is "
                     "the capture age the paw settle and kick checks received (paw_approach MAX_CAPTURE_GAP_S).",
    "ingest": "BridgeCore.ingest of one field frame/intent, bracketed by before_ns and t_ns; the "
              "executor's receipt now lies inside.",
    "clip_receipt": "first and last accepted clip_status per activation token, with Robot's own receipt time. "
                    "A receipt is a bound on native playback, not the native activation time.",
    "native_authority": "runner receipt of a changed native command_authority binding; activation_mono_ns and "
                        "sampled_mono_ns are the native controller's own CLOCK_MONOTONIC values.",
    "runtime_stop": "first CombinedRuntime.stop call: before_ns at entry, t_ns after it returned.",
    "immediate_stop": "Robot.immediate_stop entry and result.",
    "gateway_stop_request": "ResidentStart.request_stop (local latch plus native and gateway Stop publication).",
    "gateway_stop_result": "ResidentStart.poll_stop returned a result (gateway latch acknowledgement only).",
    "imu_gap": "Robot._on_imu receipt gap of at least IMU_GAP_MS. The runner's ROS IMU freshness limit is "
               "board_imu.AGE_NS (100 ms). Receipt time in the runner, not the broadcaster's publication time.",
}
IMU_GAP_MS = 20


def num(value):
    """JSON-safe number: non-finite values become strings."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def to_ns(seconds):
    return None if seconds is None or not math.isfinite(seconds) else round(seconds * 1e9)


def clock_identity():
    """This process's monotonic clock, formatted like command_deadline.local_clock_id (boot and time namespace)."""
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    return {"clock_id": f"linux-monotonic:{boot}:{os.stat('/proc/self/ns/time').st_ino}",
            "python_monotonic": time.get_clock_info("monotonic").implementation, "pid": os.getpid()}


def service_name(client):
    return getattr(client, "srv_name", None) or getattr(client, "service_name", None)


def request_summary(request):
    row = {"type": type(request).__name__}
    if hasattr(request, "activate_controllers"):
        row.update(activate=list(request.activate_controllers), deactivate=list(request.deactivate_controllers),
                   strictness=request.strictness)
    elif hasattr(request, "parameters"):
        row["parameters"] = {p.name: p.value.string_value for p in request.parameters}
    elif hasattr(request, "selected_controller"):
        row.update(selected_controller=request.selected_controller, epoch=getattr(request, "epoch", None),
                   sequence=getattr(request, "sequence", None))
    return row


def response_summary(future):
    try:
        response = future.result()
    except Exception as exc:  # recorded, never raised from a probe
        return {"error": f"{type(exc).__name__}: {exc}"[:160]}
    if response is None:
        return None
    if hasattr(response, "ok"):
        return {"ok": response.ok}
    if hasattr(response, "results"):
        return {"successful": [r.successful for r in response.results]}
    if hasattr(response, "success"):
        return {"success": response.success, "message": str(response.message)[:160]}
    if hasattr(response, "controller"):
        return {"states": {c.name: c.state for c in response.controller}}
    return {"type": type(response).__name__}


class Probe:
    """CLOCK_MONOTONIC rows recorded inside one production role process."""

    def __init__(self, role):
        self.role, self.rows, self.lock = role, [], threading.Lock()
        self.ticks = []
        self.call = contextvars.ContextVar("walk_kick_wire_call", default=None)
        self.switch = contextvars.ContextVar("walk_kick_wire_switch", default=None)

    def mark(self, kind, **fields):
        row = {"kind": kind, "t_ns": time.monotonic_ns(), **{k: num(v) for k, v in fields.items()}}
        with self.lock:
            row["id"] = len(self.rows)
            self.rows.append(row)
        return row

    def update(self, row, **fields):
        with self.lock:
            row.update({k: num(v) for k, v in fields.items()})

    def dump(self, path):
        with self.lock:
            rows = [dict(row) for row in self.rows]
        Path(path).write_text(json.dumps({"schema": TIMING_SCHEMA, "role": self.role, "clock": clock_identity(),
                                          "probes": PROBES, "rows": rows, "decision_ages": list(self.ticks)},
                                         default=str) + "\n")


def install_rpc_probe(probe):
    from rclpy.client import Client
    original = Client.call_async

    def call_async(client, request):
        row = probe.mark("rpc_send", service=service_name(client), call=probe.call.get(),
                         request=request_summary(request))
        try:
            future = original(client, request)
        except BaseException as exc:
            probe.update(row, send_returned_ns=time.monotonic_ns(), error=f"{type(exc).__name__}: {exc}"[:160])
            raise
        probe.update(row, send_returned_ns=time.monotonic_ns())
        def done(future):
            if future.cancelled():
                probe.update(row, cancelled_ns=time.monotonic_ns())
            else:
                probe.update(row, response_ns=time.monotonic_ns(), response=response_summary(future))
        future.add_done_callback(done)
        return future
    Client.call_async = call_async


def outcome(exc):
    return f"{type(exc).__name__}: {exc}"[:200]


def install_runner_probes(probe, runner):
    """Pass-through wrappers on the runner's own classes; results and exceptions are unchanged."""
    from robot_api import Robot
    from fc_walk_kick import CombinedRuntime
    from foundation.hardware.software_start import ResidentStart
    install_rpc_probe(probe)

    original_call = Robot._call

    async def _call(robot, client, request, *args, **kwargs):
        row = probe.mark("call", service=service_name(client), switch=probe.switch.get())
        token = probe.call.set(row["id"])
        try:
            result = await original_call(robot, client, request, *args, **kwargs)
        except BaseException as exc:
            probe.update(row, exit_ns=time.monotonic_ns(), outcome=outcome(exc))
            raise
        finally:
            probe.call.reset(token)
        probe.update(row, exit_ns=time.monotonic_ns(), outcome="no response" if result is None else "response")
        return result
    Robot._call = _call

    original_switch = Robot.switch_to

    async def switch_to(robot, controller, start_clip="", **kwargs):
        row = probe.mark("switch_to", controller=controller, start_clip=start_clip,
                         expected_active=kwargs.get("expected_active"))
        token = probe.switch.set(row["id"])
        try:
            result = await original_switch(robot, controller, start_clip, **kwargs)
        except BaseException as exc:
            probe.update(row, exit_ns=time.monotonic_ns(), outcome=outcome(exc))
            raise
        finally:
            probe.switch.reset(token)
        probe.update(row, exit_ns=time.monotonic_ns(), ok=result.ok, outcome=str(result.message)[:200])
        return result
    Robot.switch_to = switch_to

    original_immediate = Robot.immediate_stop

    async def immediate_stop(robot, *args, **kwargs):
        row = probe.mark("immediate_stop")
        try:
            result = await original_immediate(robot, *args, **kwargs)
        except BaseException as exc:
            probe.update(row, exit_ns=time.monotonic_ns(), outcome=outcome(exc))
            raise
        probe.update(row, exit_ns=time.monotonic_ns(), ok=result.ok, outcome=str(result.message)[:200])
        return result
    Robot.immediate_stop = immediate_stop

    original_imu = Robot._on_imu
    last_imu = [None]

    def _on_imu(robot, msg):
        now = time.monotonic_ns()
        original_imu(robot, msg)
        if last_imu[0] is not None and now - last_imu[0] >= IMU_GAP_MS * 1_000_000:
            probe.mark("imu_gap", previous_ns=last_imu[0], gap_ms=round((now - last_imu[0]) / 1e6, 3))
        last_imu[0] = now
    Robot._on_imu = _on_imu

    original_clip = Robot._on_clip_status
    first = {}

    def _on_clip_status(robot, controller, msg):
        before = robot.clip_receipts.get(controller)
        original_clip(robot, controller, msg)
        after = robot.clip_receipts.get(controller)
        if after is None or after is before:
            return
        name, holding, frame, received, activation, _, _ = after
        row = first.get((controller, activation))
        if row is None:
            first[controller, activation] = probe.mark(
                "clip_receipt", controller=controller, clip=name, activation=activation, holding=holding,
                frame=frame, received_ns=to_ns(received), receipts=1)
        else:
            probe.update(row, last_frame=frame, last_holding=holding, last_received_ns=to_ns(received),
                         receipts=row["receipts"] + 1)
    Robot._on_clip_status = _on_clip_status

    original_gate = CombinedRuntime._gate

    def _gate(runtime, job, *, entry):
        before = time.monotonic_ns()
        reason = original_gate(runtime, job, entry=entry)
        probe.mark("gate", before_ns=before, entry=entry, call=probe.call.get(), attempt_id=job.attempt_id,
                   issued_at=job.issued_at, start_before=job.start_before,
                   authorized_until=runtime._authorized_until, reason=None if reason is None else str(reason))
        return reason
    CombinedRuntime._gate = _gate

    original_submit = CombinedRuntime._submit

    async def _submit(runtime, job):
        row = probe.mark("submit", attempt_id=job.attempt_id)
        try:
            await original_submit(runtime, job)
        except BaseException as exc:
            probe.update(row, exit_ns=time.monotonic_ns(), outcome=outcome(exc))
            raise
        probe.update(row, exit_ns=time.monotonic_ns(), outcome="bound")
    CombinedRuntime._submit = _submit

    original_tick = CombinedRuntime.tick
    last_phase = [None]

    def tick(runtime, *args, **kwargs):
        step = original_tick(runtime, *args, **kwargs)
        if step.request is not None:
            request = step.request
            probe.mark("kick_request", attempt_id=request.attempt_id, epoch=request.epoch, side=request.side,
                       issued_at=request.issued_at, start_before=request.start_before,
                       evidence=executor_evidence(runtime))
        if step.cancel is not None:
            probe.mark("kick_cancel", attempt_id=step.cancel.attempt_id)
        executor = runtime.executor
        own = executor.frame[1].get(runtime.robot_id) if executor.frame is not None else None
        if own is not None and executor.last_now is not None:
            probe.ticks.append([time.monotonic_ns(), round((executor.last_now - own.captured_at) * 1e3, 3),
                                step.decision.phase])
        if step.decision.phase != last_phase[0]:
            last_phase[0] = step.decision.phase
            probe.mark("phase", phase=step.decision.phase, reason=str(step.decision.reason)[:200])
        return step
    CombinedRuntime.tick = tick

    original_stop = CombinedRuntime.stop

    def stop(runtime, reason="operator Stop"):
        first_stop, before = not runtime.stop_reason, time.monotonic_ns()
        original_stop(runtime, reason)
        if first_stop:
            probe.mark("runtime_stop", before_ns=before, reason=str(reason)[:200],
                       kick_active=runtime.handoff.active is not None)
    CombinedRuntime.stop = stop

    original_ingest = runner.BridgeCore.ingest

    def ingest(core, message):
        before = time.monotonic_ns()
        fields = {}
        if isinstance(message, dict):
            own = (message.get("robots") or {}).get(core.robot_id) if isinstance(message.get("robots"), dict) else None
            fields = {"event": message.get("event"), "seq": message.get("seq"),
                      "captured_at": message.get("captured_at"), "issued_at": message.get("issued_at"),
                      "valid_until": message.get("valid_until"),
                      "own_captured_at": own.get("captured_at") if isinstance(own, dict) else None}
        try:
            result = original_ingest(core, message)
        except Exception as exc:
            probe.mark("ingest", before_ns=before, accepted=False, reason=outcome(exc), **fields)
            raise
        probe.mark("ingest", before_ns=before, accepted=result["accepted"], reason=str(result["reason"])[:160],
                   **fields)
        return result
    runner.BridgeCore.ingest = ingest

    original_native = runner.GuardedPath.native_status
    binding = [None]

    def native_status(path, raw):
        original_native(path, raw)
        try:
            value = json.loads(raw)
            key = (value.get("generation"), value.get("activation_mono_ns"), value.get("bound_source_instance"))
        except (ValueError, TypeError, AttributeError):
            return
        if key != binding[0]:
            binding[0] = key
            probe.mark("native_authority", generation=key[0], activation_mono_ns=key[1], bound_source_instance=key[2],
                       sampled_mono_ns=value.get("sampled_mono_ns"), native_clock_id=value.get("clock_id"),
                       error=path.native_error)
    runner.GuardedPath.native_status = native_status

    original_request_stop = ResidentStart.request_stop

    def request_stop(caller, reason, **kwargs):
        before = time.monotonic_ns()
        request = original_request_stop(caller, reason, **kwargs)
        probe.mark("gateway_stop_request", before_ns=before, request=request)
        return request
    ResidentStart.request_stop = request_stop

    original_poll_stop = ResidentStart.poll_stop

    def poll_stop(caller, **kwargs):
        result = original_poll_stop(caller, **kwargs)
        if result is not None:
            probe.mark("gateway_stop_result", confirmed=result["confirmed"], uncertain=result["uncertain"],
                       gateway_epoch=result["gateway_epoch"])
        return result
    ResidentStart.poll_stop = poll_stop


def executor_evidence(runtime):
    """The executor frame and intent current when a kick request is issued."""
    executor = runtime.executor
    frame, intent = executor.frame, executor.intents.get(runtime.robot_id)
    row = {"frame_seq": executor.frame_seq,
           "intent": None if intent is None else {"issued_at": intent.issued_at, "valid_until": intent.valid_until}}
    if frame is not None:
        own = frame[1].get(runtime.robot_id)
        row.update(frame_captured_at=frame[0], own_captured_at=None if own is None else own.captured_at)
    return row


# ----------------------------------------------------------------- production roles

def harness_supervision(run):
    """This run's manager identity, in place of the prepared/homed persistent session checks."""
    def verify(run_dir, root, *, operator_controls=False):
        if Path(run_dir).resolve() != run or Path(root).resolve() != ROOT or not operator_controls:
            raise ValueError("harness runner must use its own run, checkout and operator controls")
        manager = json.loads((run / "owned_session.json").read_text())["processes"]["manager"]
        if (start_ticks(manager["pid"]) != manager["start_ticks"]
                or os.readlink(f"/proc/{manager['pid']}/exe") != manager["executable"]):
            raise ValueError("harness manager identity changed")
        return json.loads((run / "runtime-manifest.json").read_text())
    return verify


def runner_role(spec):
    run = Path(spec["run"])
    sys.path.insert(0, str(ROOT))
    import foundation.hardware.fc_walk_kick_release as release
    from final_project.behaviors import combined_fc_runner as runner
    release.verify_running_session = harness_supervision(run)
    probe = Probe("runner")
    install_runner_probes(probe, runner)
    try:
        return runner.main(["--run-dir", str(run), "--robot-id", ROBOT, "--side", spec["side"],
                            "--producer-id", PRODUCER, "--field-config", str(run / "field.json"), "--operator-controls"])
    finally:
        probe.dump(run / "runner-timing.json")
        record_imports(run / "runner-imports.json")


def operator_role(spec, owner):
    run = Path(spec["run"])
    sys.path.insert(0, str(ROOT / "foundation/hardware"))
    import fc_operator
    harness_supervision(run)(run, ROOT, operator_controls=True)
    probe = Probe("operator")
    install_rpc_probe(probe)
    try:
        return fc_operator.serve(run, ROBOT, owner)
    finally:
        probe.dump(run / "operator-timing.json")
        record_imports(run / "operator-imports.json")


def board_role(spec, manager_pid):
    """Synthetic strict board receipt until interrupted (the driver is never loaded)."""
    sys.path.insert(0, str(ROOT))
    board = BoardReceipt(Path(spec["run"]) / "fc-board-health.bin", int(manager_pid))
    print(json.dumps({"event": "ready", "manager_pid": board.pid, "instance": board.instance}), flush=True)
    try:
        board.thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        board.close()
    return 0


# ----------------------------------------------------------------- scenario fixtures

def command_offset(policy):
    """Index of the commanded vx in the newest published observation frame."""
    spec = policy.get("obs_spec")
    if spec is None:
        return 6  # obs_spec.hpp legacy_obs_layout
    if spec.get("frame_order") != "newest_first":
        raise Failure("selected walker policy does not publish its newest frame first")
    offset = 0
    for component in spec["components"]:
        if component["name"] == "command":
            return offset
        offset += component["size"]
    raise Failure("selected walker policy does not observe a velocity command")


def memory_urdf(text, config):
    """The selected robot description with only its hardware plugin and driver parameters replaced."""
    root = ET.fromstring(text)
    blocks = list(root.iter("ros2_control"))
    if len(blocks) != 1:
        raise Failure("selected URDF must have one ros2_control system")
    walk = config["gemini_walk"]["ros__parameters"]
    defaults = dict(zip(walk["joint_names"], walk["default_joint_pos"]))
    block = blocks[0]
    # Never present memory hardware under the board driver's component name.
    block.set("name", "walk_kick_wire_memory")
    hardware = block.find("hardware")
    for child in list(hardware):
        hardware.remove(child)
    ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"
    joints = block.findall("joint")
    if sorted(j.get("name") for j in joints) != sorted(defaults):
        raise Failure("selected URDF joints differ from the walker configuration")
    for element in [*joints, *block.findall("sensor")]:
        for child in list(element):
            if child.tag == "param":
                element.remove(child)
        for state in element.findall("state_interface"):
            for param in state.findall("param"):
                state.remove(param)
            name = state.get("name")
            value = (defaults[element.get("name")] if name == "position" and element.tag == "joint"
                     else 1. if name == "orientation.w" else 0.)
            ET.SubElement(state, "param", name="initial_value").text = repr(float(value))
    xml = ET.tostring(root, encoding="unicode")
    plugins = [p.text for b in root.iter("ros2_control") for p in b.iter("plugin")]
    if plugins != ["mock_components/GenericSystem"] or "control_board" in xml or "/dev/" in xml:
        raise Failure("memory robot description still selects device hardware")
    return xml


def field_config():
    from dataclasses import asdict
    from final_project.pupper_fc.model import Field
    values = asdict(Field())
    keys = ("length", "width", "goal_width", "robot_radius", "ball_radius", "bumper_offset")
    return {"field": {k: values[k] for k in keys}, "attack_signs": {"pupper-1": 1, "pupper-2": -1}}


class BoardReceipt:
    """Synthetic FC_BOARD_HEALTH_V1 writer: healthy strict profile, homed once, both SPI replies.

    Each update is one complete 192-byte pwrite whose sequence word is even, so a
    reader never observes a half-written record. Run it in its own process: a
    writer thread sharing the scenario's interpreter can stall past the reader's retries.
    """

    def __init__(self, path, manager_pid):
        from final_project.behaviors import board_imu as imu
        self.imu, self.path, self.pid = imu, Path(path), manager_pid
        self.fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        self.instance = time.monotonic_ns()
        self.seq = 0
        self.stop = threading.Event()
        self.write()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def write(self):
        imu, now = self.imu, time.monotonic_ns()
        self.seq += 2
        sample = now - 1_000_000
        row = [self.seq, imu.MAGIC, self.pid, self.instance, 1, 0, 0, imu.HEALTHY, now, sample, sample, 0, 0,
               (self.seq // 2) % 256, (self.seq // 2) % 256, *imu.PRIMARY_SOURCES, sample, sample, 0, 0, 3,
               imu.PROFILE["strict"], 0]
        os.pwrite(self.fd, imu.RECORD.pack(*row), 0)

    def loop(self):
        while not self.stop.wait(BOARD_PERIOD_S):
            self.write()

    def close(self):
        self.stop.set()
        self.thread.join()
        os.close(self.fd)


class Children:
    """Owned child processes, each in its own process group; JSON stdout lines become events."""

    def __init__(self, run):
        self.run, self.rows, self.exits = run, {}, {}

    def spawn(self, tag, argv, env, *, events=False, stdin=False):
        log = (self.run / f"{tag}.log").open("w")
        proc = subprocess.Popen(argv, env=env, cwd=ROOT, stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
                                stdout=subprocess.PIPE if events else log, stderr=log, text=True, bufsize=1,
                                start_new_session=True)
        row = self.rows[tag] = {"proc": proc, "log": log, "events": [], "argv": argv}
        if events:
            def read():
                for line in proc.stdout:
                    log.write(line)
                    log.flush()
                    try:
                        value = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(value, dict):
                        row["events"].append((time.monotonic(), value))
            threading.Thread(target=read, daemon=True).start()
        return proc

    def pid(self, tag):
        return self.rows[tag]["proc"].pid

    def events(self, tag, name=None):
        return [(t, e) for t, e in list(self.rows[tag]["events"]) if name is None or e.get("event") == name]

    def alive(self, tag):
        return tag in self.rows and self.rows[tag]["proc"].poll() is None

    def check(self, tags):
        for tag in tags:
            code = self.rows[tag]["proc"].poll()
            if code is not None:
                self.rows[tag]["log"].flush()
                tail = (self.run / f"{tag}.log").read_text(errors="replace")[-1500:]
                raise Failure(f"{tag} exited with {code}:\n{tail}")

    def send(self, tag, message):
        proc = self.rows[tag]["proc"]
        proc.stdin.write(json.dumps(message) + "\n")
        proc.stdin.flush()

    def wait(self, tag, why):
        proc = self.rows[tag]["proc"]
        last = time.monotonic()
        while proc.poll() is None:
            time.sleep(.02)
            if time.monotonic() - last >= 5:
                last = time.monotonic()
                print(f"  still waiting for {tag} to exit ({why})", flush=True)
        self.exits[tag] = proc.returncode
        return proc.returncode

    def reap(self):
        """Interrupt every live child, newest first, and wait for each actual exit."""
        try:
            for tag in reversed(list(self.rows)):
                proc = self.rows[tag]["proc"]
                if proc.poll() is None:
                    if proc.stdin and not proc.stdin.closed:
                        try:
                            proc.stdin.close()
                        except OSError:
                            pass
                    os.killpg(proc.pid, signal.SIGINT)
                self.wait(tag, "cleanup")
        except KeyboardInterrupt:
            # A second operator interrupt during cleanup ends the remaining groups.
            for tag, row in self.rows.items():
                if row["proc"].poll() is None:
                    os.killpg(row["proc"].pid, signal.SIGKILL)
                    self.exits[tag] = row["proc"].wait()
            raise
        finally:
            for row in self.rows.values():
                row["log"].close()
        return dict(self.exits)


class Observer:
    """Isolated-domain ROS publishers and observers for the scenario."""

    def __init__(self, offset):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from controller_manager_msgs.srv import ListControllers
        from joy_utils.srv import FcStartMotion
        from rcl_interfaces.msg import ParameterEvent
        from std_msgs.msg import Empty, Float32MultiArray, String
        from final_project.behaviors import body_command_guard as guard
        from final_project.behaviors import command_path_evidence as evidence
        from final_project.behaviors.fc_executor_bridge import EVENT_TOPIC
        self.rclpy, self.String, self.guard = rclpy, String, guard
        self.ListControllers, self.FcStartMotion = ListControllers, FcStartMotion
        rclpy.init(args=[])
        self.node = rclpy.create_node("walk_kick_wire_observer", start_parameter_services=False)
        # The runner's graph check names the detector node; this node publishes only synthetic frames.
        self.camera_node = rclpy.create_node("viser_camera", start_parameter_services=False)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.description = self.node.create_publisher(String, "/robot_description", latched)
        self.camera = self.camera_node.create_publisher(String, guard.FRAME_TOPIC, 1)
        self.field = self.node.create_publisher(String, EVENT_TOPIC, 8)
        self.offset = offset
        self.moving, self.observations = deque(maxlen=20000), [0]
        self.clips, self.authority, self.estops, self.params, self.statuses = [], [], [], [], []
        self.frames = deque(maxlen=20000)  # (publish time, seq, captured_at, intent kind) per field frame
        self.status = {}

        def observation(msg):
            self.observations[0] += 1
            vx = float(msg.data[offset])
            if abs(vx) > MOVING_VX:
                self.moving.append((time.monotonic(), vx))

        def status(msg):
            value = json.loads(msg.data)
            key = tuple(value.get(k) for k in ("instance_id", "epoch", "latched", "fault", "in_flight",
                                                 "stop_required", "fc_operator_owner", "fc_operator_armed"))
            if not self.statuses or self.statuses[-1][1] != key:
                self.statuses.append((time.monotonic(), key, value))
            self.status = value

        def parameters(msg):
            if msg.node == "/trick_leg_kick":
                for row in (*msg.new_parameters, *msg.changed_parameters):
                    if row.name in ("execution_request_id", "start_clip"):
                        # stamp is the controller node's ROS clock (system time), kept only as reported.
                        self.params.append((time.monotonic(), row.name, row.value.string_value,
                                            msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec))

        self.node.create_subscription(Float32MultiArray, "/gemini_walk/observation", observation, 10)
        self.node.create_subscription(String, "/trick_leg_kick/clip_status",
                                      lambda m: self.clips.append((time.monotonic(), m.data)), 50)
        self.node.create_subscription(String, evidence.NATIVE_AUTHORITY_TOPIC,
                                      lambda m: self.authority.append((time.monotonic(), json.loads(m.data))),
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(String, "/estop_controller/stop_status", status, latched)
        self.node.create_subscription(Empty, "/emergency_stop", lambda _: self.estops.append(time.monotonic()), 10)
        self.node.create_subscription(ParameterEvent, "/parameter_events", parameters, 50)
        self.inventory_client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.start_client = self.node.create_client(FcStartMotion, "/estop_controller/fc_start_motion")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.camera_node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.frame_seq = 0

    def publish_frame(self):
        self.frame_seq += 1
        self.camera.publish(self.String(data=json.dumps(dict(
            schema=self.guard.SCHEMA, producer_instance="walk-kick-wire-synthetic",
            detector_identity=dict(self.guard.ARENA_DETECTOR_IDENTITY), sequence=self.frame_seq,
            captured_at_ros_ns=self.camera_node.get_clock().now().nanoseconds, valid=True, healthy=True,
            format="normalized_xyxy_v1", image_width=700, image_height=572, horizontal_fov_deg=220,
            detections=[]))))

    def close(self):
        self.executor.shutdown()
        self.thread.join()
        self.camera_node.destroy_node()
        self.node.destroy_node()
        self.rclpy.try_shutdown()


class Scenario:
    def __init__(self, spec):
        self.spec, self.run = spec, Path(spec["run"])
        self.env, self.side = spec["env"], spec["side"]
        self.manifest = json.loads((self.run / "runtime-manifest.json").read_text())
        import yaml
        self.config_path = self.run / "persistent_controllers.yaml"
        self.config = yaml.safe_load(self.config_path.read_text())
        walk = json.loads(Path(self.config["gemini_walk"]["ros__parameters"]["model_path"]).read_text())
        kick = json.loads(Path(self.config["trick_leg_kick"]["ros__parameters"]["model_path"]).read_text())
        clip = next(c for c in kick["trick"]["clips"] if c["name"] == f"kick_{self.side}")
        # operator_kick.confirm_kick_progress: the clip's motion completes 2.6 s after its start.
        self.kick_motion_frame = clip["start"] + round(2.6 * kick["trick"]["fps"])
        self.kick_midpoint = (clip["start"] + self.kick_motion_frame) // 2  # Stop-during-kick point
        self.clip_end = clip["end"]
        self.children = Children(self.run)
        self.obs = Observer(command_offset(walk))
        self.checks, self.live = {}, []
        self.owner = uuid.uuid4().hex
        self.challenges_sent = 0
        self.stood = False
        self.field_on = None  # (pose, kind) while the synthetic field is publishing
        self.field_frames = self.field_seq = 0
        self.epoch = "wire-" + uuid.uuid4().hex[:12]
        self.clock = "linux-monotonic:" + Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.next_field = self.next_camera = 0.
        self.camera_on = False
        self.armed_at = None  # unexpected runner stops after arm end the scenario at once
        self.goal_x = field_config()["field"]["length"] / 2
        # Seconds subtracted from publication time to date each field capture (0: dated at publication).
        self.capture_latency = float(spec.get("capture_latency_s", 0.))
        if self.capture_latency:
            from final_project.behaviors.overhead_ingress import MAX_FRAME_AGE_S
            self.intent_lease = MAX_FRAME_AGE_S
        # kick-entry-arena-latency records a runtime Stop as its outcome instead of failing on it.
        self.tolerate_stop = spec["scenario"] == "kick-entry-arena-latency"
        self.runner_stopped = None
        self.status_at_arm = None
        self.marks = {}

    # -- one pass of every synthetic producer and liveness check
    def pump(self):
        now = time.monotonic()
        if self.tolerate_stop and self.armed_at is not None and self.runner_stopped is None:
            stopped = [(t, e) for t, e in self.children.events("runner", "phase")
                       if t > self.armed_at and e["phase"] == "stopped"]
            if stopped or not self.children.alive("runner"):
                self.runner_stopped = {"observed_ns": to_ns(stopped[0][0] if stopped else now),
                                       "reason": stopped[0][1].get("reason") if stopped else "runner exited"}
                # The runtime's own Stop latches the gateway; the relay then ends laptop control.
                self.live = [tag for tag in self.live if tag not in ("runner", "operator")]
        self.children.check(self.live)
        if self.armed_at is not None and not self.tolerate_stop:
            stopped = [e for t, e in self.children.events("runner", "phase")
                       if t > self.armed_at and e["phase"] == "stopped"]
            if stopped:
                raise Failure("runner stopped field play: " + str(stopped[0].get("reason")))
        if self.camera_on and now >= self.next_camera:
            self.next_camera = now + CAMERA_PERIOD_S
            self.obs.publish_frame()
        if self.field_on and now >= self.next_field:
            self.next_field = now + FIELD_PERIOD_S
            self.publish_field(*self.field_on, now)
        if self.children.alive("operator"):
            self.browser()
        time.sleep(.004)

    def until(self, predicate, why, budget=None, *, exhausted_ok=False):
        """Poll while required children live; a motion phase fails when its finite input is used up.

        exhausted_ok returns None instead, for phases whose exhaustion is itself the observation.
        """
        start_frames, last = self.field_frames, time.monotonic()
        while True:
            value = predicate()
            if value:
                return value
            if budget is not None and self.field_frames - start_frames >= budget:
                if exhausted_ok:
                    return None
                raise Failure(f"{why}: finite synthetic input exhausted after {budget} field frames")
            if time.monotonic() - last >= 5:
                last = time.monotonic()
                print(f"  waiting: {why}", flush=True)
            self.pump()

    def pause(self, seconds):
        end = time.monotonic() + seconds
        self.until(lambda: time.monotonic() >= end, f"{seconds}s observation interval")

    def publish_field(self, pose, kind, now):
        self.field_seq += 1
        self.field_frames += 1
        common = dict(producer_id=PRODUCER, clock=self.clock, epoch=self.epoch, seq=self.field_seq)
        captured = now - self.capture_latency
        if self.capture_latency:
            # As live_field_attack leases each capture: issued at the capture, valid for the ingress frame age.
            issued, valid = captured, captured + self.intent_lease
        else:
            issued, valid = now, now + .3
        intent = dict(common, event="intent", robot_id=ROBOT, kind=kind, intent_id=f"{kind}-1",
                      issued_at=issued, valid_until=valid)
        if kind == "move":
            intent["target"] = {"x": pose[0] + .3, "y": pose[1], "yaw": 0.}
        frame = dict(common, event="frame", captured_at=captured, ball={"x": 0., "y": 0.},
                     robots={ROBOT: {"x": pose[0], "y": pose[1], "yaw": 0., "captured_at": captured}},
                     goals={"right": {"x": self.goal_x, "y": 0.}})
        for row in (intent, frame):
            self.obs.field.publish(self.obs.String(data=json.dumps(row)))
        self.obs.frames.append((time.monotonic(), self.field_seq, captured, kind))

    def browser(self):
        """Echo each fresh relay challenge as a heartbeat; press Stand once when the relay is ready."""
        challenges = self.children.events("operator", "operator_challenge")
        for _, event in challenges[self.challenges_sent:]:
            self.children.send("operator", {"action": "heartbeat", "challenge": event["challenge"]})
        self.challenges_sent = len(challenges)
        status = self.children.events("operator", "operator_status")
        if not self.stood and challenges and status and status[-1][1]["ready"]:
            challenge = challenges[-1][1]["challenge"]
            instance, epoch, sequence = challenge.split()
            self.checks["pre_start_challenge"] = {"instance_id": instance, "epoch": int(epoch), "sequence": int(sequence)}
            self.children.send("operator", {"action": "stand", "challenge": challenge, "supported": True})
            self.stood = True

    def inventory(self, why):
        future = self.obs.inventory_client.call_async(self.obs.ListControllers.Request())
        self.until(future.done, why)
        return {row.name: row.state for row in future.result().controller}

    # -- startup
    def start_manager(self):
        controllers = self.manifest["motion_controllers"]
        if self.spec["manager"] == "ros2-control":
            description = memory_urdf(Path(self.manifest["robot_description"]).read_text(), self.config)
            (self.run / "memory-robot.urdf").write_text(description)
            self.obs.description.publish(self.obs.String(data=description))
            self.children.spawn("manager", [MANAGER, "--ros-args", "--params-file", str(self.config_path),
                                            "-r", BACKEND_REMAP], self.env)
            self.live = ["manager"]
            # Same two spawners and arguments as lab6_session.launch.py.
            spawners = {"spawn-feedback": [SPAWNER, *self.manifest["feedback_broadcasters"], *SPAWN_ARGS],
                        "spawn-motion": [SPAWNER, *controllers, "--inactive", "--param-file",
                                         str(self.config_path), *SPAWN_ARGS]}
            for tag, argv in spawners.items():
                self.children.spawn(tag, argv, self.env)
            for tag in spawners:
                self.until(lambda tag=tag: not self.children.alive(tag), f"{tag} loads controllers")
                if self.children.wait(tag, "spawner") != 0:
                    raise Failure(f"{tag} failed; see {tag}.log")
        else:
            manager = self.config["controller_manager"]["ros__parameters"]
            env = dict(self.env, WIRE_PARAMS_FILE=str(self.config_path), WIRE_CONTROLLERS=",".join(controllers),
                       WIRE_MANAGER_RATE=str(manager["update_rate"]),
                       WIRE_IMU_RATE=str(self.config["imu_sensor_broadcaster"]["ros__parameters"]["update_rate"]))
            self.children.spawn("manager", [self.spec["fixture"], "--gtest_filter=ProductionRuntime.FixtureManager"],
                                env, events=True)
            self.live = ["manager"]
            self.until(lambda: self.children.events("manager", "ready"), "fixture manager ready")
        states = self.until(lambda: self.obs.inventory_client.service_is_ready() and self.inventory("inventory"),
                            "controller inventory")
        expected = {name: "inactive" for name in controllers} | {"imu_sensor_broadcaster": "active"}
        if {k: states.get(k) for k in expected} != expected:
            raise Failure(f"unexpected initial controller states {states}")
        self.checks["manager"] = {"backend": self.spec["manager"], "initial_states": states,
                                  **self.inspect_manager()}

    def inspect_manager(self):
        pid = self.children.pid("manager")
        maps = Path(f"/proc/{pid}/maps").read_text()
        libraries = sorted({line.split()[-1] for line in maps.splitlines() if line.endswith("libneural_controller.so")})
        selected = str(Path(self.spec["controller_prefix"], "lib/libneural_controller.so").resolve())
        if libraries != [selected] or digest(selected) != self.spec["native_library_sha256"]:
            raise Failure(f"manager mapped {libraries}, not the selected controller library {selected}")
        if any(marker in maps for marker in ("control_board", "bno0", "libi2c")):
            raise Failure("manager mapped a hardware driver library")
        mock = "libmock_components.so" in maps
        if mock != (self.spec["manager"] == "ros2-control"):
            raise Failure("memory hardware backend differs from the selected manager mode")
        return {"pid": pid, "executable": os.readlink(f"/proc/{pid}/exe"), "native_library": selected,
                "mock_components_mapped": mock, "device_fds": self.device_fds(pid)}

    def device_fds(self, pid):
        found = []
        for fd in Path(f"/proc/{pid}/fd").iterdir():
            try:
                target = os.readlink(fd)
            except OSError:
                continue
            if target.startswith("/dev/") and target not in ALLOWED_DEVICES and not target.startswith("/dev/shm/"):
                found.append(target)
        if found:
            raise Failure(f"pid {pid} holds device descriptors {found}")
        return found

    def start_gateway(self):
        executable = str(Path(self.spec["gateway_prefix"], "lib/joy_utils/estop_controller").resolve())
        self.children.spawn("gateway", [executable, "--ros-args", "--params-file", str(self.config_path)], self.env)
        self.live.append("gateway")
        pid = self.children.pid("gateway")
        self.until(lambda: os.path.exists(f"/proc/{pid}/exe"), "gateway process")
        if os.readlink(f"/proc/{pid}/exe") != executable or digest(executable) != self.spec["gateway_executable_sha256"]:
            raise Failure("running gateway is not the selected executable")
        status = self.until(lambda: (lambda s: s if s.get("latched") and s.get("fc_operator_control") is True
                                     and not any(s.get(k) for k in ("fault", "in_flight", "stop_required")) else None)(
                                         self.obs.status), "gateway starts latched with laptop Stop control")
        self.checks["gateway"] = {"executable": executable, "initial": {k: status[k] for k in (
            "instance_id", "epoch", "latched", "fc_operator_control", "fc_operator_lease_ms")}}

    def write_session_files(self):
        manager = self.children.pid("manager")
        (self.run / "selected-session.json").write_text(json.dumps({"session_id": uuid.uuid4().hex}) + "\n")
        (self.run / "owned_session.json").write_text(json.dumps({"processes": {"manager": {
            "pid": manager, "start_ticks": start_ticks(manager), "executable": os.readlink(f"/proc/{manager}/exe")}}}) + "\n")
        (self.run / "field.json").write_text(json.dumps(field_config()) + "\n")
        self.python_role("board", str(manager))
        self.until(lambda: self.children.events("board", "ready"), "synthetic board receipt")

    def python_role(self, tag, *args):
        argv = [sys.executable, "-B", "-s", str(HERE / "client.py"), tag, str(self.run / "spec.json"), *args]
        self.children.spawn(tag, argv, self.env, events=True, stdin=True)
        self.live.append(tag)

    def stand(self):
        self.python_role("operator", self.owner)
        self.until(lambda: self.children.events("operator", "operator_standing"), "operator Stand acknowledged")
        states = self.inventory("standing inventory")
        if states.get("gemini_walk") != "active" or states.get("trick_leg_kick") != "inactive":
            raise Failure(f"Stand did not select only the walker: {states}")
        self.checks["stand"] = {"owner_armed": self.obs.status.get("fc_operator_armed"), "states": states}

    def open_and_arm(self):
        self.python_role("runner")
        self.until(lambda: self.children.events("runner", "ready"), "field runner opened against the walker")
        replies = 0
        for attempt in range(5):
            offers = self.until(lambda: [e for _, e in self.children.events("runner", "controls") if e["ready"]],
                                "ready control ticket")
            self.children.send("runner", {"command": "arm", "robot_id": ROBOT, "ticket": offers[-1]["ticket"],
                                          "epoch": self.epoch})
            reply = self.until(lambda: self.children.events("runner", "operator")[replies:], "arm reply")[-1][1]
            replies = len(self.children.events("runner", "operator"))
            if reply.get("accepted"):
                self.armed_at = self.marks["armed"] = time.monotonic()
                self.status_at_arm = dict(self.obs.status)
                self.checks["arm"] = {"attempts": attempt + 1, "epoch": self.epoch}
                return
            if "ticket" not in str(reply.get("reason")):
                raise Failure(f"arm refused: {reply}")
        raise Failure("arm tickets kept expiring")

    # -- motion phases
    def bound(self, after):
        return [a for t, a in self.obs.authority if t > after and a.get("bound_source_instance")]

    def walking_ready(self, after, generation_above=0):
        moving = [vx for t, vx in self.obs.moving if t > after]
        bound = [a for a in self.bound(after) if a["generation"] > generation_above]
        if not (moving and bound):
            return None
        return {"applied_vx": round(moving[-1], 4), "generation": bound[-1]["generation"],
                "bound_source_instance": bound[-1]["bound_source_instance"]}

    def walking(self, after, why, budget, generation_above=0):
        return self.until(lambda: self.walking_ready(after, generation_above), why, budget)

    def kick_edges(self):
        return [e for _, e in self.children.events("runner", "kick")]

    def clip_receipts(self, after):
        rows = []
        for t, text in self.obs.clips:
            match = CLIP.match(text)
            if t > after and match and match.group(1) == f"kick_{self.side}":
                rows.append((t, int(match.group(2)), match.group(4)))
        return rows

    def execution_request(self):
        values = [v for _, name, v, _ in self.obs.params if name == "execution_request_id" and len(v) == 32]
        return values[-1] if values else None

    def run_scenario(self):
        side = self.side
        from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M, PAW_LEFT_M
        staged = (-COMBINED_PAW_FORWARD_M[side], -PAW_LEFT_M[side])
        self.start_manager()
        self.start_gateway()
        from fc_release_profile import GUARD, GUARD_ARGS
        self.children.spawn("guard", [sys.executable, "-B", "-s", str(ROOT / GUARD), *GUARD_ARGS], self.env)
        self.live.append("guard")
        self.camera_on = True
        self.write_session_files()
        self.stand()
        self.open_and_arm()

        began = self.marks["approach"] = time.monotonic()
        self.field_on = ((-.65, staged[1]), "attack")
        if self.tolerate_stop:
            return self.latency_scenario(began, staged)
        first = self.walking(began, "approach: native applies the guarded field command", APPROACH_FRAMES)
        self.checks["approach"] = first

        docked = self.marks["docked"] = time.monotonic()
        self.field_on = (staged, "attack")
        if self.spec["scenario"] == "stop-during-kick":
            self.until(lambda: any(f >= self.kick_midpoint for _, f, _ in self.clip_receipts(docked)),
                       "kick clip plays to its motion midpoint", KICK_FRAMES)
            self.marks["kick_midpoint_seen"] = time.monotonic()
            self.checks["kick_interrupted"] = self.kick_evidence(docked, first, complete=False)
            self.stop_phase(expect_kick_aborted=True)
            return
        self.until(lambda: any(e["state"] == "completed" for e in self.kick_edges()), "kick completes and walker restores",
                   KICK_FRAMES)
        completed = self.marks["kick_completed"] = time.monotonic()
        self.checks["kick"] = self.kick_evidence(docked, first, complete=True)
        self.field_on = (staged, "move")
        restored = self.walking(completed - .5, "restored walker applies a new guarded command", RESTORE_FRAMES,
                                generation_above=first["generation"])
        if restored["bound_source_instance"] != first["bound_source_instance"]:
            raise Failure("restored walker bound a different command source")
        self.checks["restored"] = restored
        self.stop_phase(expect_kick_aborted=False)

    def latency_scenario(self, began, staged):
        """Field captures dated by the selected arena camera latency; the observed outcome is the result.

        Contract: every phase ends at its finite input or a terminal observation, and the robot ends in a
        verified Stop (gateway latched, both leg controllers inactive, no walking after the acknowledgement).
        A refused or late kick entry is a legitimate recorded outcome, not a harness failure. Any runtime
        Stop passes this contract, whatever caused it: read checks.runtime_stop.reason before attributing it.
        """
        def stopped():
            return self.runner_stopped is not None

        first = self.until(lambda: stopped() or self.walking_ready(began), "approach under capture latency",
                           APPROACH_FRAMES, exhausted_ok=True)
        if not isinstance(first, dict):
            return self.latency_end("approach: " + ("runtime stopped" if stopped() else "no applied walking command "
                                                    "within its finite field input"))
        self.checks["approach"] = first
        docked = self.marks["docked"] = time.monotonic()
        self.field_on = (staged, "attack")
        edge = self.until(lambda: stopped() or next((e["state"] for e in self.kick_edges()
                                                      if e["state"] in ("completed", "retired")), None),
                          "kick under capture latency", KICK_FRAMES, exhausted_ok=True)
        if edge != "completed":
            requested = bool(self.kick_edges())
            return self.latency_end(("kick requested; " if requested else "no kick request; ")
                                    + ("runtime stopped" if stopped() else f"kick {edge}" if edge
                                       else "finite field input exhausted"))
        completed = self.marks["kick_completed"] = time.monotonic()
        self.checks["kick"] = self.kick_evidence(docked, first, complete=True)
        self.field_on = (staged, "move")
        restored = self.until(lambda: stopped() or self.walking_ready(completed - .5, first["generation"]),
                              "restored walker under capture latency", RESTORE_FRAMES, exhausted_ok=True)
        if not isinstance(restored, dict):
            return self.latency_end("kick completed; restored walking " + ("ended by runtime Stop" if stopped()
                                                                          else "not applied within finite input"))
        self.checks["restored"] = restored
        return self.latency_end("kick completed and walker restored")

    def latency_end(self, outcome):
        self.checks["outcome"] = outcome
        self.field_on = None
        if self.runner_stopped is None:
            self.stop_phase(expect_kick_aborted=None)
        else:
            self.checks["runtime_stop"] = self.runner_stopped
            self.stop_phase(expect_kick_aborted=None, runtime_initiated=True)

    def kick_evidence(self, docked, first, *, complete):
        execution = self.execution_request()
        clips = self.clip_receipts(docked)
        if not execution or not clips or any(token != execution for _, _, token in clips):
            raise Failure(f"clip receipts do not carry this kick's execution request {execution}: {clips[-2:]}")
        starts = [v for t, name, v, _ in self.obs.params if name == "start_clip" and t > docked]
        if f"kick_{self.side}" not in starts:
            raise Failure(f"kick controller was not started at kick_{self.side}: {starts}")
        edges = self.kick_edges()
        states = [e["state"] for e in edges]
        frames = [f for _, f, _ in clips]
        evidence = {"execution_request_id": execution, "start_clip": f"kick_{self.side}", "edges": states,
                    "first_receipt_frame": frames[0], "last_receipt_frame": max(frames), "receipts": len(clips),
                    "walker_generation_before": first["generation"]}
        first_clip = clips[0][0]
        if any(a["generation"] == first["generation"] for t, a in self.obs.authority if t > first_clip + .25):
            raise Failure("walker command authority stayed live while the kick owned the legs")
        if complete:
            if states[:3] != ["engaged", "clip_started", "completed"] or len({e["attempt_id"] for e in edges}) != 1:
                raise Failure(f"kick receipts out of order: {states}")
            if not self.kick_motion_frame <= max(frames) < self.clip_end:
                raise Failure(f"kick frames {frames[0]}..{max(frames)} never reached motion end {self.kick_motion_frame}")
            evidence["motion_end_frame"] = self.kick_motion_frame
        return evidence

    def stop_phase(self, *, expect_kick_aborted, runtime_initiated=False):
        """Scenario Stop, or (runtime_initiated) verification of a Stop the runtime already issued itself."""
        armed = self.armed_at
        if self.tolerate_stop:
            # The runtime may stop itself at any moment, even just before a scenario Stop arrives.
            # Gateway state at arm is unlatched; the first newer latched epoch is the Stop either way.
            before, stop_sent = self.status_at_arm, armed
        else:
            before = dict(self.obs.status)
        if not runtime_initiated:
            self.marks["stop_sent"] = time.monotonic()
            stop_sent = stop_sent if self.tolerate_stop else self.marks["stop_sent"]
            try:
                self.children.send("runner", {"command": "stop"})
            except OSError:
                if not self.tolerate_stop:
                    raise  # the runner's exit and Stop result are verified below either way
        self.armed_at = None
        # The relay ends laptop control itself once the gateway latches.
        self.live = [tag for tag in self.live if tag not in ("runner", "operator")]
        self.until(lambda: not self.children.alive("runner"), "runner Stop and exit")
        code = self.children.wait("runner", "Stop")
        stopped = [e.get("reason") for t, e in self.children.events("runner", "phase")
                   if t > armed and e["phase"] == "stopped"]
        if self.tolerate_stop and not runtime_initiated and code == 3 and stopped:
            runtime_initiated = True  # the runtime's own Stop won the race with the scenario Stop
            self.checks["runtime_stop"] = {"reason": stopped[0], "raced_scenario_stop": True}
        expected_exit = 3 if runtime_initiated else 0  # 3: a runtime Stop the operator did not request
        results = [e for _, e in self.children.events("runner", "stop_result")]
        if code != expected_exit or not results or not results[-1]["confirmed"] or results[-1]["uncertain"]:
            raise Failure(f"runner Stop not confirmed (exit {code}): {results[-1:]}")
        after = self.until(lambda: (lambda s: s if s.get("epoch", 0) > before["epoch"] and s.get("latched") and not any(
            s.get(k) for k in ("in_flight", "stop_required")) else None)(self.obs.status), "gateway latch settles")
        if after["instance_id"] != before["instance_id"] or after.get("fault"):
            raise Failure(f"gateway instance changed or faulted during Stop: {after}")
        if not [t for t in self.obs.estops if t >= stop_sent]:
            raise Failure("gateway did not publish /emergency_stop")
        first = self.inventory("post-Stop inventory")
        self.pause(.55)  # A delayed backend completion would reactivate within this window.
        second = self.inventory("late post-Stop inventory")
        for states in (first, second):
            if states.get("gemini_walk") != "inactive" or states.get("trick_leg_kick") != "inactive":
                raise Failure(f"a leg controller stayed active after Stop: {states}")
        acknowledged = self.children.events("runner", "stop_result")[-1][0]
        late = [vx for t, vx in self.obs.moving if t > acknowledged + .05]
        if late:
            raise Failure(f"native applied walking commands after the Stop acknowledgement: {late[:3]}")
        edges = [e["state"] for e in self.kick_edges()]
        if expect_kick_aborted and ("aborted" not in edges or "completed" in edges):
            raise Failure(f"Stop during the kick was not reported as aborted: {edges}")
        self.until(lambda: not self.children.alive("operator"), "relay ends laptop control")
        relay = self.children.events("operator", "operator_stop")
        self.checks["stop"] = {"initiated_by": "runtime" if runtime_initiated else "scenario",
                               "runner_exit": code, "stop_result_epoch": results[-1]["gateway_epoch"],
                               "gateway_epoch_before_after": [before["epoch"], after["epoch"]],
                               "same_gateway_instance": True, "estop_published": True,
                               "states_now_and_after_0_55s": [first, second], "kick_edges": edges,
                               "relay_exit": self.children.wait("operator", "relay"),
                               "relay_stop": relay[-1][1] if relay else None}
        self.stale_authority()

    def stale_authority(self):
        """The consumed pre-Stop Stand challenge, replayed by its owner, cannot reactivate the walker."""
        old = self.checks["pre_start_challenge"]
        request = self.obs.FcStartMotion.Request(instance_id=old["instance_id"], epoch=old["epoch"],
                                                 sequence=old["sequence"], selected_controller="gemini_walk",
                                                 owner=self.owner)
        future = self.obs.start_client.call_async(request)
        self.until(future.done, "replayed Stand reply")
        response = future.result()
        self.pause(.3)
        states = self.inventory("inventory after replayed Stand")
        if response.success or states.get("gemini_walk") != "inactive":
            raise Failure(f"replayed pre-Stop Stand reactivated the walker: {response.message} {states}")
        self.checks["pre_stop_stand_replay_refused"] = {"gateway_reason": response.message, "states": states}

    def finish(self, result):
        self.field_on, self.camera_on = None, False
        for tag in ("runner", "operator"):
            if self.children.alive(tag):
                try:
                    self.children.send(tag, {"command": "stop"} if tag == "runner" else {"action": "stop"})
                except OSError:
                    pass
        try:
            if self.children.alive("manager"):
                self.checks.setdefault("manager_final_device_fds", self.device_fds(self.children.pid("manager")))
        except Failure as exc:
            result["passed"], result["failure"] = False, str(exc)
        result["children"] = self.children.reap()
        result["native_observations"] = self.obs.observations[0]
        result["gateway_transitions"] = [list(k) for _, k, _ in self.obs.statuses][-12:]
        self.obs.close()
        try:
            timing = self.timing()
            (self.run / "timing.json").write_text(json.dumps(timing, indent=1, default=str) + "\n")
            result["timing"] = {"file": "timing.json", **timing["derived"]}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            # Missing timing never turns a run into a pass; it is reported as missing.
            result["timing"] = {"error": f"{type(exc).__name__}: {exc}"}

    def timing(self):
        """Raw CLOCK_MONOTONIC observations from every process plus labeled derivations."""
        roles = {}
        for role in ("runner", "operator"):
            path = self.run / f"{role}-timing.json"
            roles[role] = json.loads(path.read_text()) if path.is_file() else None
        obs = self.obs
        scenario = {
            "clock": clock_identity(),
            "marks_ns": {k: to_ns(v) for k, v in self.marks.items()},
            "capture_latency_s": self.capture_latency,
            "capture_latency_source": self.spec.get("capture_latency_source"),
            "field_frames": [{"publish_ns": to_ns(t), "seq": q, "captured_at_ns": to_ns(c), "kind": k}
                             for t, q, c, k in obs.frames],
            "native_authority": [{"receipt_ns": to_ns(t), **a} for t, a in obs.authority],
            "clip_status": [{"receipt_ns": to_ns(t), "text": text} for t, text in obs.clips],
            "parameter_events": [{"receipt_ns": to_ns(t), "name": n, "value": v, "node_stamp_system_ns": stamp}
                                 for t, n, v, stamp in obs.params],
            "gateway_status": [{"receipt_ns": to_ns(t), "status": value} for t, _, value in obs.statuses],
            "emergency_stop_receipts_ns": [to_ns(t) for t in obs.estops],
            "runner_events": [{"receipt_ns": to_ns(t), **e} for t, e in self.children.events("runner")
                              if e.get("event") in ("kick", "phase", "stop_result", "ready", "operator")],
        }
        manager = [{"receipt_ns": to_ns(t), **e} for t, e in self.children.events("manager")] \
            if self.spec["manager"] == "fixture" else []
        return {"schema": TIMING_SCHEMA, "scenario": self.spec["scenario"], "manager": self.spec["manager"],
                "clock_note": "every *_ns value and every monotonic-seconds field (captured_at, issued_at, "
                              "start_before, valid_until) is CLOCK_MONOTONIC on the clock_id recorded by each "
                              "process; node_stamp_system_ns is the controller node's ROS system clock and is "
                              "not aligned to it",
                "probes": PROBES, "native_events": manager, "scenario_observer": scenario, "roles": roles,
                "derived": derive_timing(roles.get("runner"), scenario, manager, self.spec["manager"])}


# ----------------------------------------------------------------- timing derivation

def ms(t, origin):
    return None if t is None or origin is None else round((t - origin) / 1e6, 3)


def first(rows, test=lambda row: True):
    return next((row for row in rows if test(row)), None)


def clip_frame(text, clip=None):
    match = CLIP.match(text)
    return int(match.group(2)) if match and (clip is None or match.group(1) == clip) else None


def derive_timing(runner, scenario, manager, backend):
    """Per kick request: budget, observation age, entry gates, service rounds and native entry evidence.

    Times are milliseconds from the request's own issued_at unless named *_ns. Native entry is exact
    only for the fixture manager (in-process on_activate); the real manager yields bounds.
    """
    from final_project.pupper_fc.paw_approach import MAX_CAPTURE_GAP_S
    rows = runner["rows"] if runner else []
    kind = lambda name: [r for r in rows if r["kind"] == name]
    calls = {r["id"]: r for r in kind("call")}
    switches = {r["id"]: r for r in kind("switch_to")}
    ingests = {r["seq"]: r for r in kind("ingest") if r.get("event") == "frame"}
    published = {f["seq"]: f for f in scenario["field_frames"]}

    def context(rpc):
        call = calls.get(rpc.get("call"))
        if call is None:
            return "outside Robot._call"
        switch = switches.get(call.get("switch"))
        if switch is None:
            return "Robot._call outside switch_to (inventory watch or stop)"
        return f"switch_to({switch['controller']}, start_clip={switch['start_clip']!r})"

    def rpc_row(rpc, origin):
        call = calls.get(rpc.get("call"), {})
        # A call that ended without taking its response (cancelled, no response) has no polling delay.
        answered = call.get("outcome") == "response"
        return {"service": rpc["service"], "context": context(rpc), "request": rpc["request"],
                "send_ms": ms(rpc["t_ns"], origin), "send_returned_ms": ms(rpc.get("send_returned_ns"), origin),
                "response_ms": ms(rpc.get("response_ns"), origin), "cancelled_ms": ms(rpc.get("cancelled_ns"), origin),
                "call_exit_ms": ms(call.get("exit_ns"), origin), "call_outcome": call.get("outcome"), "round_trip_ms": ms(rpc.get("response_ns"), rpc["t_ns"]),
                "poll_delay_ms": ms(call.get("exit_ns"), rpc.get("response_ns")) if answered else None,
                "response": rpc.get("response")}

    entries = []
    requests = kind("kick_request")
    for index, request in enumerate(requests):
        issued, deadline = to_ns(request["issued_at"]), to_ns(request["start_before"])
        # The job runs after _submit returns (queue acceptance); its evidence lasts until the next request.
        end = requests[index + 1]["t_ns"] if index + 1 < len(requests) else math.inf
        submit = first(kind("submit"), lambda r: r["attempt_id"] == request["attempt_id"] and r["t_ns"] >= request["t_ns"])
        window = lambda r: request["t_ns"] <= r["t_ns"] < end
        evidence = request["evidence"]
        captured = to_ns(evidence.get("own_captured_at") if evidence.get("own_captured_at") is not None
                         else evidence.get("frame_captured_at"))
        ingest, publish = ingests.get(evidence.get("frame_seq")), published.get(evidence.get("frame_seq"))
        rpcs = [r for r in kind("rpc_send") if window(r)]
        gates = [{"t_ms": ms(g["t_ns"], issued), "before_ms": ms(g["before_ns"], issued),
                  "remaining_to_start_before_ms": ms(deadline, g["t_ns"]), "entry": g["entry"],
                  "where": "_submit entry" if g["call"] is None else calls.get(g["call"], {}).get("service"),
                  "reason": g["reason"]} for g in kind("gate")
                 if window(g) and g["attempt_id"] == request["attempt_id"]]
        kick_switch = first(rpcs, lambda r: KICK_CONTROLLER in r["request"].get("activate", []))
        restore = first(rpcs, lambda r: kick_switch is not None and r["t_ns"] > kick_switch["t_ns"]
                        and WALK_CONTROLLER in r["request"].get("activate", []))
        entry_gates = [g for g in gates if g["entry"] and g["where"] != "_submit entry"]
        entry = {"issued_at_ns": issued, "start_before_ns": deadline, "attempt_id": request["attempt_id"],
                 "budget_ms": ms(deadline, issued),
                 "start_before_basis": ("capture + MAX_CAPTURE_GAP_S" if captured is not None and abs(
                     deadline - captured - round(MAX_CAPTURE_GAP_S * 1e9)) <= 1000 else "decision command expiry"),
                 "observation": {
                     "frame_seq": evidence.get("frame_seq"), "captured_at_ns": captured,
                     "capture_to_issue_ms": ms(issued, captured),
                     "declared_capture_latency_s": scenario["capture_latency_s"],
                     "scenario_publish_ns": publish and publish["publish_ns"],
                     "runner_ingest_ns": ingest and ingest["t_ns"],
                     "publish_to_runner_ingest_ms": ms(ingest and ingest["t_ns"], publish and publish["publish_ns"]),
                     "runner_ingest_to_issue_ms": ms(issued, ingest and ingest["t_ns"]),
                     "intent": evidence.get("intent")},
                 "submit": submit and {"enter_ms": ms(submit["t_ns"], issued), "exit_ms": ms(submit.get("exit_ns"), issued),
                                       "outcome": submit.get("outcome")},
                 "gates": gates,
                 "last_activation_gate": entry_gates[-1] if entry_gates else None,
                 "service_rounds": [rpc_row(r, issued) for r in rpcs if kick_switch is None or restore is None
                                    or r["t_ns"] <= restore["t_ns"]],
                 "kick_switch": kick_switch and {**rpc_row(kick_switch, issued),
                                                 "remaining_to_start_before_ms": ms(deadline, kick_switch["t_ns"])}}
        receipts = [r for r in kind("clip_receipt") if r["controller"] == KICK_CONTROLLER
                    and kick_switch is not None and r["t_ns"] >= kick_switch["t_ns"]]
        observed = first(scenario["clip_status"], lambda c: kick_switch is not None and c["receipt_ns"] >= kick_switch["t_ns"]
                         and clip_frame(c["text"], f"kick_{request['side']}") is not None)
        entry["first_clip"] = {
            "runner_received_ms": ms(receipts[0]["received_ns"], issued) if receipts else None,
            "runner_frame": receipts[0]["frame"] if receipts else None,
            "observer_received_ms": ms(observed and observed["receipt_ns"], issued),
            "observer_frame": clip_frame(observed["text"]) if observed else None,
            "meaning": "first received clip_status sample: bounds native playback, not physical motion"}
        if backend == "fixture" and kick_switch is not None:
            switch = first(manager, lambda e: e.get("event") == "switch" and e.get("entry_ns", 0) >= kick_switch["t_ns"]
                           and KICK_CONTROLLER in e.get("activate", []))
            activation = first(switch.get("activations", []) if switch else [],
                               lambda a: a["controller"] == KICK_CONTROLLER)
            update = first(manager, lambda e: e.get("event") == "first_update" and e["controller"] == KICK_CONTROLLER
                           and switch is not None and e["begin_ns"] >= switch["entry_ns"])
            entry["native_entry"] = {
                "basis": "exact: fixture switch callback ran the selected controller's on_activate in-process",
                "switch_callback_entry_ms": ms(switch and switch["entry_ns"], issued),
                "on_activate_begin_ms": ms(activation and activation["begin_ns"], issued),
                "on_activate_end_ms": ms(activation and activation["end_ns"], issued),
                "on_activate_ok": activation and activation["ok"],
                "switch_response_ms": ms(switch and switch["response_ns"], issued),
                "first_update_begin_ms": ms(update and update["begin_ns"], issued),
                "parameter_commits": [{"ms": ms(e["mono_ns"], issued), **e["parameters"]} for e in manager
                                      if e.get("event") == "param_set" and e["controller"] == KICK_CONTROLLER
                                      and request["t_ns"] <= e["mono_ns"] <= kick_switch["t_ns"]]}
            entry["native_entry"]["remaining_to_start_before_at_on_activate_ms"] = ms(
                deadline, activation and activation["begin_ns"])
        elif kick_switch is not None and kick_switch.get("response") not in (None, {"ok": True}):
            entry["native_entry"] = {"basis": "kick switch refused; no activation is implied",
                                     "response": kick_switch["response"],
                                     "response_ms": ms(kick_switch.get("response_ns"), issued)}
        elif kick_switch is not None:
            # An accepted switch response bounds activation; an unanswered one does not.
            accepted = kick_switch.get("response_ns") if kick_switch.get("response") == {"ok": True} else None
            bounds = [t for t in (accepted, receipts[0]["received_ns"] if receipts else None,
                                  observed and observed["receipt_ns"]) if t is not None]
            entry["native_entry"] = {
                "basis": "bounds only: the real manager exposes no activation hook to this harness",
                "not_before_ms": ms(kick_switch["t_ns"], issued),
                "not_after_ms": ms(min(bounds), issued) if bounds else None,
                "not_after_source": "earliest of an accepted switch response receipt and first clip_status receipt"}
        else:
            entry["native_entry"] = {"basis": "no kick activation request was sent"}
        if restore is not None:
            walker = first(scenario["native_authority"], lambda a: (a.get("activation_mono_ns") or 0) >= restore["t_ns"])
            bound = first(scenario["native_authority"], lambda a: walker is not None and a.get("bound_source_instance")
                          and a.get("generation") == walker.get("generation"))
            restore_end = restore.get("response_ns") or math.inf
            entry["restore"] = {**rpc_row(restore, issued),
                                "max_imu_gap_during_switch_ms": max(
                                    (g["gap_ms"] for g in kind("imu_gap")
                                     if restore["t_ns"] <= g["t_ns"] and g["previous_ns"] <= restore_end), default=None),
                                "native_walker_activation_ms": ms(walker and walker["activation_mono_ns"], issued),
                                "native_walker_activation_basis": "exact: native command_authority activation_mono_ns",
                                "walker_generation": walker and walker.get("generation"),
                                "first_bound_authority_receipt_ms": ms(bound and bound["receipt_ns"], issued)}
        entries.append(entry)
    gaps = kind("imu_gap")
    return {"kick_entries": entries, "decision_ages": decision_ages(runner, MAX_CAPTURE_GAP_S),
            "imu_gaps": {"threshold_ms": IMU_GAP_MS, "count": len(gaps),
                         "max_ms": max((g["gap_ms"] for g in gaps), default=None),
                         "gaps": [{"end_ns": g["t_ns"], "gap_ms": g["gap_ms"]} for g in gaps]},
            "stop": derive_stop(rows, scenario, manager, backend)}


def decision_ages(runner, gap_s):
    """Capture age at each decision tick, per phase, against the paw capture gap."""
    by_phase = {}
    for _, age, phase in (runner or {}).get("decision_ages", []):
        by_phase.setdefault(phase, []).append(age)
    limit = gap_s * 1e3
    return {phase: {"ticks": len(ages), "min_ms": min(ages), "median_ms": sorted(ages)[len(ages) // 2],
                    "max_ms": max(ages), "at_or_over_capture_gap": sum(a >= limit for a in ages),
                    "capture_gap_ms": limit} for phase, ages in by_phase.items()}


def derive_stop(rows, scenario, manager, backend):
    """Stop request to latch, Stop publication and deactivation, in ms from the Stop origin."""
    kind = lambda name: [r for r in rows if r["kind"] == name]
    runtime = first(kind("runtime_stop"))
    sent = scenario["marks_ns"].get("stop_sent")
    origin, basis = (sent, "scenario Stop sent to the runner") if sent else (
        runtime and runtime["before_ns"], "runtime-initiated Stop (CombinedRuntime.stop entry)")
    if origin is None:
        return None
    request = first(kind("gateway_stop_request"))
    result = first(kind("gateway_stop_result"))
    prior = [s for s in scenario["gateway_status"] if s["receipt_ns"] < origin]
    epoch = prior[-1]["status"].get("epoch", 0) if prior else 0
    latched = first(scenario["gateway_status"], lambda s: s["receipt_ns"] >= origin and s["status"].get("latched")
                    and s["status"].get("epoch", 0) > epoch)
    estop = first(scenario["emergency_stop_receipts_ns"], lambda t: t >= origin)
    stop_result = first(scenario["runner_events"], lambda e: e.get("event") == "stop_result")
    row = {"origin": basis, "origin_ns": origin,
           "runtime_stop_ms": ms(runtime and runtime["before_ns"], origin), "runtime_reason": runtime and runtime["reason"],
           "gateway_stop_published_ms": ms(request and request["t_ns"], origin),
           "gateway_stop_requested_at_ns": request and to_ns(request["request"].get("requested_at")),
           "emergency_stop_observer_receipt_ms": ms(estop, origin),
           "gateway_latched_status_receipt_ms": ms(latched and latched["receipt_ns"], origin),
           "runner_stop_confirmed_ms": ms(result and result["t_ns"], origin),
           "stop_result_scenario_receipt_ms": ms(stop_result and stop_result["receipt_ns"], origin),
           "deactivation_requests": [{"send_ms": ms(r["t_ns"], origin), "response_ms": ms(r.get("response_ns"), origin),
                                      "cancelled_ms": ms(r.get("cancelled_ns"), origin),
                                      "service": r["service"], "request": r["request"], "response": r.get("response")}
                                     for r in kind("rpc_send") if r["t_ns"] >= origin and r["request"].get("deactivate")]}
    if backend == "fixture":
        row["native_deactivations"] = [{"switch_callback_entry_ms": ms(e["entry_ns"], origin),
                                        "on_deactivate": [{**d, "begin_ms": ms(d["begin_ns"], origin),
                                                           "end_ms": ms(d["end_ns"], origin)} for d in e["deactivations"]]}
                                       for e in manager if e.get("event") == "switch" and e.get("entry_ns", 0) >= origin
                                       and e.get("deactivations")]
    return row


def scenario(spec_path):
    spec = json.loads(Path(spec_path).read_text())
    if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("scenario requires the isolated loopback environment from run.py")
    sys.path[:0] = [str(ROOT), str(ROOT / "gemini_pupper/robot")]
    run = Path(spec["run"])
    result = {"scenario": spec["scenario"], "side": spec["side"], "manager": spec["manager"], "passed": False,
              "synthetic": ["browser heartbeat/Stand", "overhead field frames and intents", "detector frames",
                            "strict board health receipt"],
              "field_capture_dating": (f"captured_at = publication - {spec['capture_latency_s']} s "
                                       f"({spec['capture_latency_source']['key']} in "
                                       f"{spec['capture_latency_source']['path']}); intents leased from the capture"
                                       if spec.get("capture_latency_s") else
                                       "captured_at = publication time (no camera latency adjustment)")}
    signal.signal(signal.SIGINT, signal.default_int_handler)
    current = None
    try:
        current = Scenario(spec)
        result["checks"] = current.checks
        current.run_scenario()
        result["passed"] = True
    except (Failure, OSError, ValueError, KeyError) as exc:
        result["failure"] = f"{type(exc).__name__}: {exc}"
    except KeyboardInterrupt:
        result["failure"] = "interrupted by the operator"
    finally:
        if current is not None:
            current.finish(result)
        record_imports(run / "scenario-imports.json")
        (run / "result.json").write_text(json.dumps(result, indent=2, default=str) + "\n")
    print(json.dumps({k: result.get(k) for k in ("scenario", "passed", "failure")}), flush=True)
    return 0 if result["passed"] else 1


def main(argv):
    if len(argv) >= 2 and argv[0] == "scenario":
        return scenario(argv[1])
    spec = json.loads(Path(argv[1]).read_text())
    if argv[0] == "runner" and len(argv) == 2:
        return runner_role(spec)
    if argv[0] == "operator" and len(argv) == 3:
        return operator_role(spec, argv[2])
    if argv[0] == "board" and len(argv) == 3:
        return board_role(spec, argv[2])
    raise SystemExit(__doc__)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
