"""One synthetic replay robot: the production field runner over substituted native I/O.

The laptop starts one of these interpreters per connected replay team, in place of
``fc_session.py match --runner-only --connect`` on an enrolled Pi. It speaks the
same stdin/stdout protocol. Everything that decides or authorizes motion is the
selected code: RunnerSession (ticketed offers, operator actions, board holds, kick
receipts), OperatorDrive, GuardedPath, CombinedRuntime, BridgeCore/Executor,
MatchImu/BoardReceipt, the software_start gateway caller, robot_api.Robot and
operator_kick.kick_in_place.

Only what lies beyond the Pi's ROS boundary is substituted, in this process only:
rclpy and its message types; the controller manager and leg_kick clip playback;
the Stop gateway latch; the body-command guard and native command-authority
reports; the board-health record and ROS IMU; and the walker, which forwards each
command it would execute to the laptop's kinematic plant. Home and standing are
not rehearsed: the stand-in starts with the walker active. Every stdout event and
bus message is marked synthetic and non-physical. Nothing here observes a robot.

fd ``--bus-fd`` stands in for the Pi's ROS topics shared with the field producer:
``/fc/normalized_events`` arrive, ``/fc/kick_result`` and plant messages leave.
"""
from __future__ import annotations

import argparse
import asyncio
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import tempfile
import time
import types
import uuid

ROOT = Path(__file__).resolve().parents[2]
SYNTHETIC = {"synthetic": True, "physical": False}
WALKER = "gemini_walk"
GUARD = "fc_body_command_guard"
STOP_STATUS_TOPIC = "/estop_controller/stop_status"
STOP_TOPIC = "/estop_controller/request_stop"
IMU_TOPIC = "/imu_sensor_broadcaster/imu"
# Synthetic: the plant applies the kick impulse this far into the native kick clip.
KICK_CONTACT_S = 1.2
TELEMETRY_S = .2
LABELS = {"planner": "Executed by the selected RunnerSession, CombinedRuntime and integration.Executor",
          "native": "Synthetic controller manager, clip playback, guard, board and walker; no contact model"}
NATIVE = None  # The stand-in every substituted ROS endpoint addresses.


# ------------------------------------------------------------------ ROS boundary
class _Msg:
    def __init__(self, **values):
        self.__dict__.update(values)


class _Data(_Msg):
    def __init__(self, data=None):
        super().__init__(data=data)


class _Vector(_Msg):
    def __init__(self, x=0., y=0., z=0.):
        super().__init__(x=x, y=y, z=z)


class _Twist(_Msg):
    def __init__(self):
        super().__init__(linear=_Vector(), angular=_Vector())


class _Service:
    Request = _Msg


class _SwitchController:
    class Request(_Msg):
        STRICT, BEST_EFFORT = 2, 1

        def __init__(self):
            super().__init__(activate_controllers=[], deactivate_controllers=[], strictness=1)


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result

    def cancel(self):
        return False


class _Client:
    def __init__(self, name):
        self.name = name

    def wait_for_service(self, timeout_sec=None):
        return True

    def service_is_ready(self):
        return self.name.endswith(("list_controllers", "switch_controller", "set_parameters"))

    def call_async(self, request):
        return _Future(NATIVE.call(self.name, request))


class _Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, message):
        NATIVE.publish(self.topic, message)

    def get_subscription_count(self):
        return NATIVE.subscriber_count(self.topic)


class _Node:
    def __init__(self, name="replay", **_):
        self.name = name

    def create_client(self, kind, name):
        return _Client(name)

    def create_publisher(self, kind, topic, qos):
        return _Publisher(topic)

    def create_subscription(self, kind, topic, callback, qos):
        return NATIVE.subscribe(topic, callback)

    def create_timer(self, period, callback):
        return NATIVE.timer(period, callback)

    def destroy_node(self):
        pass


class _Executor:
    def add_node(self, node):
        pass

    def spin(self):
        pass

    def shutdown(self, timeout_sec=None):
        pass


def install_ros_stand_ins():
    """Module stand-ins for this synthetic interpreter only; never imported by the laptop app."""
    if "rclpy" in sys.modules and not getattr(sys.modules["rclpy"], "replay_stand_in", False):
        raise RuntimeError("A real ROS client is loaded; the replay robot refuses to substitute it")

    def module(name, **values):
        item = types.ModuleType(name)
        item.__dict__.update(values, replay_stand_in=True)
        sys.modules[name] = item

    module("rclpy", ok=lambda: True, init=lambda **_: None, shutdown=lambda: None, try_shutdown=lambda: None)
    module("rclpy.node", Node=_Node)
    module("rclpy.signals", SignalHandlerOptions=types.SimpleNamespace(NO=0))
    module("rclpy.qos", QoSProfile=lambda **values: _Msg(**values),
           ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2),
           DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2))
    module("rclpy.executors", SingleThreadedExecutor=_Executor)
    for package in ("controller_manager_msgs", "geometry_msgs", "rcl_interfaces", "sensor_msgs", "std_msgs",
                    "std_srvs", "joy_utils"):
        module(package)
    module("controller_manager_msgs.srv", ListControllers=_Service, SwitchController=_SwitchController)
    module("geometry_msgs.msg", Twist=_Twist)
    module("rcl_interfaces.msg", Log=_Msg, Parameter=_Msg, ParameterValue=_Msg,
           ParameterType=types.SimpleNamespace(PARAMETER_STRING=4))
    module("rcl_interfaces.srv", SetParameters=_Service)
    module("sensor_msgs.msg", CompressedImage=_Msg, Imu=_Msg, JointState=_Msg)
    module("std_msgs.msg", Empty=_Data, Float32MultiArray=_Data, Float64MultiArray=_Data, String=_Data,
           MultiArrayDimension=_Msg, MultiArrayLayout=_Msg)
    module("std_srvs.srv", Trigger=_Service)
    module("joy_utils.srv", StartMotion=_Service, FcStartMotion=_Service)


# ------------------------------------------------------------- beyond the Pi's ROS
class NativeStandIn:
    """The Pi's native side, synthetic: controllers, leg_kick clips, Stop gateway, IMU and walker.

    Controller switching and interactive clip status follow the course fake robot
    that robot_api's tests use. The gateway latches on request_stop, raises its
    epoch and deactivates every leg controller. The walker forwards commands only
    while it is the active controller and the gateway is released.
    """

    def __init__(self, tricks, leg_controllers, plant, clock=time.monotonic):
        self.tricks = {t.controller: t for t in tricks.values()}
        self.clock, self.plant = clock, plant
        self.subs = defaultdict(list)
        self.timers = []
        self.controllers = {name: "inactive" for name in set(leg_controllers) | set(self.tricks)}
        self.controllers[WALKER] = "active"
        self.activations = defaultdict(int)
        self.params = defaultdict(dict)
        self.clips = {}
        self.contacts = set()
        self.status = dict(protocol="pupper_stop_v1", instance_id=uuid.uuid4().hex, sequence=0, epoch=1,
                           latched=False, fault=False, stop_required=False, in_flight=False,
                           unknown_remote_submission=False, shutting_down=False, selected_controller=WALKER,
                           start_freshness_ms=500, software_stop_topic=STOP_TOPIC)
        self.walk_source = None   # Source bound to the current walker activation.
        self.admitted = 0

    # ROS endpoints
    def subscribe(self, topic, callback):
        self.subs[topic].append(callback)
        if topic == STOP_STATUS_TOPIC:
            callback(_Data(json.dumps(self.status)))  # Transient-local initial state.
        return _Msg(topic=topic)

    def subscriber_count(self, topic):
        return 1 if topic == STOP_TOPIC else len(self.subs.get(topic, ()))

    def timer(self, period, callback):
        timer = _Msg(period=period, callback=callback, next_at=self.clock() + period, cancelled=False)
        timer.cancel = lambda: setattr(timer, "cancelled", True)
        self.timers.append(timer)
        return timer

    def deliver(self, topic, message):
        for callback in list(self.subs.get(topic, ())):
            callback(message)

    def publish(self, topic, message):
        if topic == STOP_TOPIC:
            self.latch()
        elif topic == "/emergency_stop":
            self.latch()
            self.deliver(topic, message)

    def call(self, name, request):
        if name.endswith("list_controllers"):
            return _Msg(controller=[_Msg(name=n, state=s) for n, s in self.controllers.items()])
        if name.endswith("switch_controller"):
            blocked = any(self.status[key] for key in ("latched", "fault", "stop_required",
                                                       "unknown_remote_submission", "shutting_down"))
            if request.activate_controllers and blocked:
                return _Msg(ok=False)
            for controller in request.deactivate_controllers:
                self.controllers[controller] = "inactive"
                self.clips.pop(controller, None)
            for controller in request.activate_controllers:
                self.controllers[controller] = "active"
                self.activations[controller] += 1
                if controller == WALKER:
                    self.walk_source = None
                trick = self.tricks.get(controller)
                if trick is not None and trick.kind == "policy" and trick.interactive:
                    params = self.params[controller]
                    from tricks import TRICK_FADE_S, TRICK_INIT_S
                    self.clips[controller] = dict(clip=params.get("start_clip") or trick.clips[0]["name"],
                                                  t0=self.clock() + TRICK_INIT_S + TRICK_FADE_S, holding=False,
                                                  token=params.get("execution_request_id", "none"),
                                                  activation=self.activations[controller])
            return _Msg(ok=True)
        if name.endswith("/set_parameters"):
            controller = name.split("/")[1]
            for parameter in request.parameters:
                self.params[controller][parameter.name] = parameter.value.string_value
            return _Msg(results=[_Msg(successful=True)])
        raise RuntimeError("the replay native stand-in does not provide " + name)

    def latch(self):
        if not self.status["latched"]:
            self.status.update(latched=True, epoch=self.status["epoch"] + 1)
        for controller in self.controllers:
            self.controllers[controller] = "inactive"
        self.clips.clear()
        self.plant("walk", vx=0., vy=0., wz=0., expires_mono_ns=time.monotonic_ns())

    def walker_active(self):
        return self.controllers.get(WALKER) == "active" and not self.status["latched"]

    def admit(self, payload, decode):
        """The synthetic guard admits the runner's deadline command; the walker executes it."""
        command = decode(payload)
        if not self.walker_active():
            return
        self.walk_source = command.source_instance
        self.admitted += 1
        self.plant("walk", vx=command.vx, vy=command.vy, wz=command.wz, expires_mono_ns=command.expires_mono_ns)

    def advance(self):
        """Publish one round of native status, as the Pi's controllers would."""
        now = self.clock()
        self.status["sequence"] += 1
        self.deliver(STOP_STATUS_TOPIC, _Data(json.dumps(self.status)))
        self.deliver(IMU_TOPIC, _Msg(orientation=_Msg(x=0., y=0., z=0., w=1.),
                                     angular_velocity=_Vector(), orientation_covariance=[0.]))
        for controller, state in list(self.clips.items()):
            if self.controllers.get(controller) != "active" or now < state["t0"]:
                continue
            trick = self.tricks[controller]
            for _ in range(8):
                spec = trick.clip(state["clip"])
                length = trick.clip_length(state["clip"])
                if state["holding"] or now - state["t0"] < length - 1e-6:
                    break
                if spec.get("loop"):
                    state["t0"] += length
                elif spec.get("next"):
                    state["clip"], state["t0"] = spec["next"], state["t0"] + length
                else:
                    state["holding"] = True
            spec = trick.clip(state["clip"])
            elapsed = now - state["t0"]
            if state["clip"].startswith("kick_") and elapsed >= KICK_CONTACT_S:
                key = (controller, state["activation"])
                if key not in self.contacts:
                    self.contacts.add(key)
                    self.plant("kick_contact", clip=state["clip"])
            frame = spec["end"] - 1 if state["holding"] else min(spec["start"] + int(elapsed * trick.fps + 1e-6),
                                                                 spec["end"] - 1)
            phase = "holding" if state["holding"] else "playing"
            self.deliver(f"/{controller}/clip_status", _Data(
                f"{state['clip']} {phase} frame {frame} queued 0 execution {state['token']} request none"))
        for timer in self.timers:
            if not timer.cancelled and now >= timer.next_at:
                timer.next_at = now + timer.period
                timer.callback()


class SyntheticEvidence:
    """Guard status, native command authority and the board-health record, all synthetic.

    The shapes are the strict contracts GuardedPath and BoardReceipt decode. Native
    authority is published only while the walker is active, with a new generation for
    each activation, bound once the walker admits a command from this runner.
    """

    def __init__(self, core, native, board_path):
        from final_project.behaviors import board_imu as imu
        from final_project.behaviors.command_deadline import ADMITTED_TOPIC, CONTRACT, REQUEST_TOPIC
        from final_project.behaviors.command_path_evidence import GUARD_STATUS_TOPIC, NATIVE_AUTHORITY_TOPIC
        self.imu, self.core, self.native = imu, core, native
        self.contract, self.admitted_topic, self.native_topic = CONTRACT, ADMITTED_TOPIC, NATIVE_AUTHORITY_TOPIC
        self.start_ns = time.monotonic_ns()
        self.activation = (0, self.start_ns)  # (walker activations seen, latest activation time)
        self.board_path = Path(board_path)
        self.board_fd = os.open(self.board_path, os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        self.board_seq = self.guard_seq = 0
        endpoint = lambda name: types.SimpleNamespace(node_name=name, node_namespace="/", topic_type="std_msgs/msg/String")
        self.endpoint = endpoint
        self.pubs = {REQUEST_TOPIC: [endpoint("fc_walk_kick_runner")], ADMITTED_TOPIC: [endpoint(GUARD)],
                     GUARD_STATUS_TOPIC: [endpoint(GUARD)], "/fc/obstacle_detections": [endpoint("viser_camera")]}
        self.subs = {REQUEST_TOPIC: [endpoint(GUARD)]}
        self.node = types.SimpleNamespace(get_publishers_info_by_topic=lambda topic: list(self.pubs.get(topic, [])),
                                          count_publishers=lambda topic: len(self.pubs.get(topic, [])),
                                          get_subscriptions_info_by_topic=lambda topic: list(self.subs.get(topic, [])))
        self.write_board()

    def write_board(self):
        imu, n = self.imu, time.monotonic_ns()
        self.board_seq += 2
        received = n - 1_000_000
        # One Home, no faults or incidents, healthy, both SPI replies, primary IMU sources.
        record = imu.RECORD.pack(self.board_seq, imu.MAGIC, os.getpid(), self.start_ns, 1, 0, 0, imu.HEALTHY,
                                 n, received, received, 0, 0, 1, 1, *imu.PRIMARY_SOURCES, received, received,
                                 0, 0, 3, imu.PROFILE["strict"], 0)
        os.pwrite(self.board_fd, record, 0)

    def publish(self, session, path):
        """One round of evidence on the shared clock, before the runner's tick."""
        n = time.monotonic_ns()
        self.write_board()
        walking = self.native.walker_active()
        owner = [self.endpoint(WALKER)] if walking else []
        self.pubs[self.native_topic] = owner
        self.subs[self.admitted_topic] = owner
        self.guard_seq += 1
        session.on_guard(json.dumps(dict(
            state="ready", camera_state="clear", camera_permitted=True, input_state="valid",
            requires_post_hold_zero=False, clear_frames=3, reason="synthetic guard ready", frame_age_s=.05,
            max_frame_age_s=.35, producer_instance="replay-obstacle-camera", sequence=self.guard_seq,
            sampled_at_monotonic_ns=n, command_contract=self.contract, clock_id=self.core.deadline.clock_id,
            source_bound=True, source_instance=self.core.deadline.source_instance, source_seq=self.guard_seq + 1,
            source_issued_mono_ns=n - 5_000_000, source_expires_mono_ns=n + 140_000_000, source_vetoed=False,
            clearance_mono_ns=self.start_ns - 1, post_clear_zero_seq=1)))
        if walking:
            bound = self.native.walk_source if self.native.walk_source == self.core.deadline.source_instance else ""
            path.native_status(json.dumps(dict(
                schema="pupper_command_authority_v1", clock_id=self.core.deadline.clock_id,
                generation=1 + self.native.activations[WALKER], activation_mono_ns=self.activation_ns(n),
                bound_source_instance=bound, sampled_mono_ns=n)))

    def activation_ns(self, now_ns):
        generation = self.native.activations[WALKER]
        if self.activation[0] != generation:
            self.activation = (generation, now_ns)
        return self.activation[1]

    def close(self):
        os.close(self.board_fd)


def telemetry(session, step, kick, ingress, native, contacts):
    """Observation of the production objects for the laptop's Play view. It decides nothing."""
    core, runtime, controls = session.core, session.runtime, session.controls
    executor, now = core.executor, core.clock()
    planner = {"phase": "idle", "reason": "Not armed. Start a match to arm this runner's executor."}
    if step is not None and not (step.decision.phase == "stopped" and not executor.armed and not runtime.stop_reason):
        decision, command = step.decision, step.decision.command
        planner = {"phase": decision.phase, "reason": decision.reason, "role": getattr(decision, "role", None),
                   "command": {k: _round(getattr(command, k)) for k in ("vx", "vy", "wz")},
                   "target": [_round(v) for v in decision.target] if getattr(decision, "target", None) else None,
                   "target_heading": _round(getattr(decision, "target_heading", None)),
                   "position_error_m": _round(getattr(decision, "position_error", None)),
                   "heading_error_rad": _round(getattr(decision, "heading_error", None))}
    observed = None
    if executor.frame is not None:
        capture, observations, _ = executor.frame
        row = observations.get(core.robot_id)
        if row is not None:
            observed = {"capture_age_ms": _round((now - capture) * 1000, 1),
                        "pose": None if row.robot is None else [_round(row.robot.x), _round(row.robot.y), _round(row.robot.yaw)],
                        "ball": None if row.ball is None else [_round(row.ball.x), _round(row.ball.y)]}
    resume = None
    if executor.armed and planner["phase"] == "await_reset":
        try:  # The executor's read-only availability query; it consumes nothing.
            executor.check_resume(core.robot_id, executor.frame_seq, now)
            resume = "ready"
        except ValueError as exc:
            resume = str(exc)[:160]
        planner["resume"] = resume
    since = None if ingress["at"] is None else _round((now - ingress["at"]) * 1000, 1)
    return dict(event="fc_status", robot_id=core.robot_id, source="replay_runner_session",
                armed=executor.armed, epoch=executor.epoch, stopped=bool(runtime.stop_reason),
                mode=controls.mode if controls else None, planner=planner, observed=observed,
                ingress={k: ingress[k] for k in ("frames", "intents", "rejected", "last_reject")} | {"since_frame_ms": since},
                output_error=runtime.stop_reason,
                kick=dict(kick, receipts=list(kick.get("receipts", ())), impulse=contacts.get(kick.get("attempt_id"))),
                native=dict(active=sorted(k for k, v in native.controllers.items() if v == "active"),
                            gateway_latched=native.status["latched"], walker_commands=native.admitted),
                implementation={"session": _name(session), "runtime": _name(runtime), "drive": _name(controls),
                                "executor": _name(executor), "gateway": _name(session.gateway),
                                "robot": _name(runtime.robot)},
                labels=LABELS)


def _name(item):
    kind = type(item)
    return kind.__module__ + "." + kind.__qualname__


def _round(value, digits=3):
    return round(value, digits) if isinstance(value, (int, float)) and math.isfinite(value) else None


async def serve(args):
    global NATIVE
    info = json.loads(args.profile)
    if info.get("robot_id") != args.robot_id or info.get("synthetic") is not True:
        raise ValueError("The replay robot needs its own synthetic profile")
    install_ros_stand_ins()
    for directory in (ROOT, ROOT / "gemini_pupper/robot", ROOT / "gemini_pupper/lab"):
        if str(directory) not in sys.path:
            sys.path.insert(0, str(directory))
    os.environ.update(PUPPER_SESSION_SKILLS="leg_kick", PUPPER_SESSION_MOTION_MODE="fc_walk_kick")
    from final_project.behaviors import combined_fc_runner as runner
    from final_project.behaviors.board_imu import BoardReceipt, MatchImu
    from final_project.behaviors.command_deadline import decode
    from final_project.behaviors.fc_executor_bridge import (
        BridgeCore, DEADLINE_CONTRACT, EVENT_TOPIC, EventSink, TICK_S, load_fields)
    from final_project.behaviors.operator_drive import OperatorDrive
    from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M
    from foundation.hardware.software_start import attach
    from follow_me_pupper.robot import onboard_ball
    from fc_walk_kick import CombinedRuntime
    from robot_api import LEG_CONTROLLERS, Robot
    from tricks import load_tricks

    loop = asyncio.get_running_loop()
    bus_reader, bus_writer = await asyncio.open_unix_connection(sock=socket.socket(fileno=args.bus_fd))
    runtime = None

    def to_bus(topic, **data):
        if topic == "kick_contact":
            active = runtime.handoff.active if runtime is not None else None
            data["attempt_id"] = active.attempt_id if active is not None else None
        if not bus_writer.is_closing():
            bus_writer.write((json.dumps(dict(topic=topic, **data, **SYNTHETIC), allow_nan=False) + "\n").encode())

    # Mark every event synthetic. A production receipt's own physical statement is kept verbatim.
    emit = EventSink(writer=lambda row: print(json.dumps({**SYNTHETIC, **row, "synthetic": True}, allow_nan=False),
                                              flush=True))
    emit({"event": "profile", **info})
    workspace = tempfile.TemporaryDirectory(prefix="pupper-fc-replay-robot-")
    native = NATIVE = NativeStandIn(load_tricks(), LEG_CONTROLLERS, to_bus)
    robot = Robot(validate_session_manifest=False, ros_signal_handlers=False, observe_only=True)
    robot.follow_available = False
    core = BridgeCore(args.robot_id, info["producer_id"], args.boot_id, load_fields(info["field_config"]),
                      command_contract=DEADLINE_CONTRACT, command_clock_id="replay-monotonic:" + args.boot_id,
                      kick_side=args.side, paw_forward_m=COMBINED_PAW_FORWARD_M[args.side])
    evidence = SyntheticEvidence(core, native, Path(workspace.name) / "fc-board-health.bin")
    path = runner.GuardedPath(core, evidence.node, lambda payload: native.admit(payload, decode))
    controls = OperatorDrive(core)
    board_reader = BoardReceipt(evidence.board_path, os.getpid(), "strict")

    def latest_ros_imu():
        try:
            return robot.imu[-1]
        except IndexError:
            return None

    balance = MatchImu(board_reader, latest_ros_imu)
    ball_receiver = onboard_ball.OnboardBallReceiver()
    gateway = attach(_Node("fc_walk_kick_runner"), clock=time.monotonic, selected_controller=WALKER)
    kick, contacts, requested_ball = {}, {}, {}

    def publish_kick(edge):
        to_bus("/fc/kick_result", data=edge)
        if edge["attempt_id"] != kick.get("attempt_id"):
            kick.clear()
            kick.update(attempt_id=edge["attempt_id"], receipts=[])
        source = "runner" if edge["state"] == "engaged" else "synthetic native receipt"
        kick.update(state=edge["state"], epoch=edge.get("epoch"), synthetic=True,
                    outcome=edge.get("outcome") or (None if edge["state"] == "engaged" else edge["state"]))
        kick["receipts"] = (kick["receipts"] + [dict(phase=edge["state"], source=source)])[-6:]

    session = runner.RunnerSession(core, path, balance, gateway, emit, controls=controls, clock=time.monotonic,
                                   publish_kick=publish_kick, set_ball_mode=lambda mode: requested_ball.update(mode=mode),
                                   ball_snapshot=lambda now: ball_receiver.snapshot(time.time_ns(), now))
    runtime = CombinedRuntime(robot, core.executor, core.robot_id, velocity_sink=path.write,
                              path_problem=session.runtime_problem, walker_ready=path.restored_walker_ready,
                              hard_stop=lambda reason: gateway.request_stop(reason, authority=robot.gateway_stop_anchor()))
    session.attach(runtime)
    ingress = {"frames": 0, "intents": 0, "rejected": 0, "last_reject": None, "at": None}

    def ingest(data):
        # As run()'s /fc/normalized_events subscription: invalid input cannot renew authority.
        try:
            if len(json.dumps(data)) > 65536:
                raise ValueError("field event exceeds the size limit")
            core.ingest(data)
            kind = "frames" if data.get("event") == "frame" else "intents"
            ingress[kind] += 1
            if kind == "frames":
                ingress["at"] = core.clock()
        except (ValueError, TypeError, OverflowError, AttributeError) as exc:
            ingress["rejected"] += 1
            ingress["last_reject"] = str(exc)[:160]

    async def read_bus():
        while line := await bus_reader.readline():
            try:
                message = json.loads(line)
            except ValueError:
                continue
            if not isinstance(message, dict):
                continue
            if message.get("topic") == EVENT_TOPIC:
                ingest(message.get("data"))
            elif message.get("topic") == "kick_contact_result" and isinstance(message.get("attempt_id"), str):
                contacts[message["attempt_id"]] = message.get("hit") is True

    quit_event = asyncio.Event()
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
            except (ValueError, TypeError) as exc:
                session.reply({}, {"accepted": False, "reason": str(exc)[:180]})
                continue
            if session.handle(message):
                terminate()
                return

    async def watch_walk_owner():
        while True:
            await runner.sample_walker(balance, robot)
            await asyncio.sleep(.05)

    reader = opened = False
    exit_code = 0
    step = None
    bus_task = asyncio.create_task(read_bus())
    inventory_task = None
    try:
        native.advance()
        await asyncio.wait_for(runtime.open(), timeout=10)
        opened = True
        inventory_task = asyncio.create_task(watch_walk_owner())
        loop.add_reader(sys.stdin.fileno(), read_operator)
        reader = True
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, terminate)
        emit({"event": "ready", "armed": False, "side": args.side, "robot_id": args.robot_id})
        next_report = 0.
        while not quit_event.is_set():
            started = time.monotonic()
            native.advance()
            evidence.publish(session, path)
            step = session.tick()
            if started >= next_report:
                next_report = started + TELEMETRY_S
                emit(telemetry(session, step, kick, ingress, native, contacts))
            if runtime.stop_reason:
                break
            await asyncio.sleep(max(0, TICK_S - (time.monotonic() - started)))
        if runtime.stop_reason and not quit_event.is_set():
            exit_code = 3
    except Exception as exc:
        # Keep a failed synthetic I/O boundary inspectable through the same UI
        # stream; the Stop receipt alone names the boundary, not its exception.
        emit({"event": "error", "robot_id": args.robot_id,
              "reason": f"Replay runner: {type(exc).__name__}: {exc}"[:500]})
        raise
    finally:
        if reader:
            loop.remove_reader(sys.stdin.fileno())
        for task in (inventory_task, bus_task):
            if task is not None:
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
        try:
            try:
                if opened:
                    runtime.stop("combined runtime closed")
                    await session.close(time.monotonic())
                    stop_result = await runner.await_gateway_stop(gateway, lambda timeout_sec=0: native.advance(),
                                                                  lambda: True)
                    emit(stop_result)
                    if stop_result["uncertain"]:
                        exit_code = 3
                    emit(telemetry(session, step, kick, ingress, native, contacts))
            finally:
                await runtime.close()
        finally:
            board_reader.close()
            evidence.close()
            if opened:
                await robot.shutdown()
            else:
                await robot.disconnect()
            bus_writer.close()
            workspace.cleanup()
            await asyncio.sleep(.05)  # Let the event writer thread flush the terminal receipts.
    return exit_code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", choices=("pupper-1", "pupper-2"), required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--boot-id", required=True)
    parser.add_argument("--profile", required=True, help="the synthetic profile this robot reports, as JSON")
    parser.add_argument("--bus-fd", type=int, required=True)
    args = parser.parse_args(argv)
    return asyncio.run(serve(args))


if __name__ == "__main__":
    raise SystemExit(main())
