"""The robot: a command queue, the leg controllers, falls and the e-stop (provided).

You don't change this file. Your code in behaviors.py uses the Robot API below;
the table in the lab handout lists it too.

  await robot.queue(name, job, seconds, then=state)
                                  put a job in line; returns what Gemini hears now
  await robot.switch_to(controller or None)
                                  give the legs to one controller, or relax them
  robot.state_now                 the state Pupper is in right now
  robot.state_after_queue         ... once everything already queued has run
  robot.send_speed(vx, vy, wz_deg)  one speed command to the policy that walks now
  robot.walking_controller(state)  the controller that walks in a state (locomotion,
                                  or a walking trick's own walking policy)
  robot.speeds()                  default speeds and limits where Pupper will be
                                  once the queue has run (move, turn, arc use them)
  await robot.sleep(seconds), robot.now()
                                  wait without freezing the server; the time now
  robot.fallen                    which controller fell over (None if upright)
  robot.following, await robot.stop_following()
  await robot.is_active(controller)
  await robot.play_clip(trick, clip)
  await robot.play_reference_trick(trick)
  robot.tricks, robot.action_ids()

The robot is always in exactly one state, and each state is whatever holds the
legs: LOCOMOTION (the locomotion policy, the gemini_walk controller; follow mode
is on or off inside it), one trick (its trick_<name> controller: trick mode),
MOTORS_IDLE (nothing), or FALLEN. Changing state is one controller switch,
switch_to, and only switch_to (and a fall or the e-stop) changes robot.state_now.

Adapted from the Pupper monorepo's ros_tool_server.py, without the LiveKit voice
agent it was written for. Topics and controllers used (all started by
launch/gemini_pupper.launch.py):
  /llm_cmd_vel                      speed commands; cmd_vel_mux forwards them to
                                    /cmd_vel, and a joystick always overrides them
  trick_<name> controllers          trick policies (loaded by the launch file)
  /trick_<name>/clip, /clip_status  interactive tricks: request a clip, follow playback
  /camera/image_raw/compressed      latest camera frame
  activate/deactivate_person_following        person_follower services
  /imu_sensor_broadcaster/imu       body tilt and rotation, to check it is safe to stand up
  /rosout                           the controllers' "Emergency stop triggered" (tilt e-stop)

Falls. A leg controller that tilts past its max_body_angle latches an e-stop: it
stays "active" but only damps the joints until it is activated again. The robot
notices from /rosout, cancels the queue, and refuses motion until standing up
again, which first checks that the robot is upright and still, then restarts
the controller (its on_activate clears the latch and eases the legs to standing).
"""

from __future__ import annotations

import asyncio
import collections
import contextvars
import logging
import math
import queue
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Awaitable, Callable

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import Empty, Float32MultiArray, MultiArrayDimension, MultiArrayLayout, String
from std_srvs.srv import Trigger

from spawn_controllers import LOADED, spawn
from secret_gate import SecretGate
from tricks import TRICK_FADE_S, TRICK_INIT_S, WALK_CONTROLLER, WALK_READY_S, Trick, load_tricks, reference_message_data

log = logging.getLogger("robot_api")

# The locomotion controller Gemini drives: gemini_walk, the course default policy
# (the joystick's X button) loaded by our launch file with running allowed and
# reference tricks enabled. See tricks.py.
LOCOMOTION_CONTROLLER = WALK_CONTROLLER

# The states that aren't a trick (a trick's state is its Trick).
LOCOMOTION = "locomotion"  # the locomotion policy holds the legs (following or not)
MOTORS_IDLE = "motors idle"  # no controller holds the legs: Pupper lies down
FALLEN = "fallen"  # a tilt e-stop latched: the motors only damp until a safe stand-up
# gemini_walk holds the default pose for init_duration, then fades the policy in
# over fade_in_duration (the course default controller's 2 s + 2 s); reference
# tricks are refused until then. WALK_READY_S (tricks.py) is that wait.
# Everything that may hold the leg joints; switching modes deactivates the others.
LEG_CONTROLLERS = {
    WALK_CONTROLLER,
    "neural_controller",
    "neural_controller_default",
    "neural_controller_default_alt",
    "neural_controller_three_legged",
    "neural_controller_student",
}

LLM_CMD_RATE = 20.0  # Hz; cmd_vel_mux drops a source that is quiet for 500 ms

# Standing up again after a tilt e-stop: the body must be this level and this still
# (all IMU samples over SAFE_STILL_S) before a controller is restarted.
SAFE_TILT_DEG = 20.0
SAFE_GYRO_RAD_S = 0.5
SAFE_STILL_S = 1.0
SAFE_WAIT_S = 4.0  # how long standing up waits for the robot to settle
ESTOP_LOG = "Emergency stop triggered"
FELL_MESSAGE = (
    "the robot fell over: its tilt e-stop tripped and the motors only damp now. "
    "A person should set it upright on its feet; then call stand_up"
)


@dataclass
class MoveLimits:
    """Speeds below the minimums barely move the robot; above the maximums are unsafe."""

    vx_min: float = 0.2  # m/s
    vy_min: float = 0.2  # m/s
    wz_min: float = 20.0  # deg/s
    vx_max: float = 1.5  # m/s; above about 0.5 the default policy runs
    vy_max: float = 0.5  # m/s, the policy's trained sideways range
    wz_max: float = 114.0  # deg/s, the policy's trained 2 rad/s
    duration_max: float = 20.0  # s: long enough for a 1 m circle at a walk
    distance_max: float = 7.0  # m in one move (a 1 m circle is 6.3 m), so a run can't cross the room


LIMITS = MoveLimits()


@dataclass
class Speeds:
    """How Pupper walks in a state: default speeds, and the limits check_move enforces."""

    walk: float = 0.4  # m/s: move's and arc's speed when Gemini gives none
    turn: float = 60.0  # deg/s: turn's speed (and arc's, for a sharp corner)
    limits: MoveLimits = field(default_factory=lambda: LIMITS)
    where: str = ""  # "" in locomotion; " (in <trick>)" in a walking trick, for refusals


SPEEDS = Speeds()  # locomotion mode


@dataclass
class Result:
    """What a tool tells Gemini: did it work, and what happened (or why not)."""

    ok: bool
    message: str

    def as_dict(self) -> dict:
        return {"success": self.ok, "message": self.message}


def with_note(note: str, res: Result) -> Result:
    """``res`` with ``note`` (e.g. "Stopped following the person first.") in front."""
    return Result(res.ok, f"{note.strip()} {res.message}") if note.strip() else res


def not_written(todo: str, where: str) -> Result:
    """What an unfinished TODO tells Gemini, so it says so instead of pretending."""
    return Result(False, f"{todo} is not written yet ({where})")


# then= for a queued job that doesn't move the legs (robot.wait): see Robot.queue.
UNCHANGED = object()

# How many stops had happened when the tool call being handled began (set by
# tool_registry.call). A request that was still in its *now* part when a stop
# came in must not queue its job afterwards: stop clears everything.
CALL_STOPS: contextvars.ContextVar[int | None] = contextvars.ContextVar("call_stops", default=None)


def state_name(state) -> str:
    """A state in words: "locomotion", "motors idle", "fallen", or the trick's name."""
    return state.name if isinstance(state, Trick) else str(state)

Job = Callable[[], Awaitable[Result]]


class Robot:
    """Owns a ROS node and runs queued jobs one at a time."""

    def __init__(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("gemini_pupper_tools")
        self.switch_client = self.node.create_client(SwitchController, "/controller_manager/switch_controller")
        self.list_client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.follow_on = self.node.create_client(Trigger, "activate_person_following")
        self.follow_off = self.node.create_client(Trigger, "deactivate_person_following")
        self.cmd_pub = self.node.create_publisher(Twist, "/llm_cmd_vel", 10)
        self.ref_pub = self.node.create_publisher(Float32MultiArray, f"/{WALK_CONTROLLER}/reference_trick", 10)
        self.reference_request_id: str | None = None
        self.reference_status: tuple[str, str] | None = None
        self.node.create_subscription(String, f"/{WALK_CONTROLLER}/reference_status", self._on_reference_status, 10)
        self.walk_since = None  # monotonic time gemini_walk was last activated

        self.latest_image: queue.Queue[CompressedImage] = queue.Queue(maxsize=1)
        self.node.create_subscription(CompressedImage, "/camera/image_raw/compressed", self._on_image, 1)

        # The state Pupper is in right now (set by switch_to, a fall, the e-stop),
        # and the one it will be in once everything queued so far has run (set by
        # queue's then=): what a request made *now* can rely on.
        self.state_now: str | Trick = MOTORS_IDLE
        self.state_after_queue: str | Trick = MOTORS_IDLE
        self._running_job_then: str | Trick = MOTORS_IDLE  # ... once the running job ends
        # Interactive tricks: request clips and follow what the controller is playing.
        self.clip_status: dict[str, tuple[str, bool]] = {}  # controller -> (clip, holding)
        self.clip_receipts: dict[str, tuple[str, bool, int, float, str]] = {}
        self.finite_execution_requests: dict[str, str] = {}
        self.finite_activation_since: dict[str, float] = {}
        self.finite_playing_seen: set[str] = set()
        self.clip_pubs: dict[str, object] = {}
        for t in load_tricks().values():
            if t.interactive:
                self.clip_pubs[t.controller] = self.node.create_publisher(String, f"/{t.controller}/clip", 10)
            if t.kind == "policy":
                self.node.create_subscription(
                    String, f"/{t.controller}/clip_status", lambda m, c=t.controller: self._on_clip_status(c, m), 10
                )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

        self.tricks: dict[str, Trick] = load_tricks()
        self.secret_gate = SecretGate()
        self.action_lock = asyncio.Lock()  # serialize request acceptance; stop bypasses it
        self.last_outcome: dict | None = None
        self.leg_controllers = (LEG_CONTROLLERS | {t.controller for t in self.tricks.values() if t.kind == "policy"}
                                | {t.walk_controller for t in self.tricks.values() if t.walks})
        self.param_clients: dict[str, object] = {}  # controller -> its set_parameters client
        self.following = False
        self.follow_available = True  # server fills from explicit startup configuration
        self.follow_state_uncertain = False
        self.startup_error: str | None = None
        self.controller_fault: str | None = None
        self.activation_cleanup_tasks: set[asyncio.Task] = set()
        self.reference_active = False
        self.closing = False
        self.ready = asyncio.Event()
        self.commands: asyncio.Queue = asyncio.Queue()
        self.current: asyncio.Task | None = None
        self.busy_until = 0.0  # monotonic time when the queued work should finish
        self.stops = 0  # how many times stop (or the e-stop) has run
        self.worker = asyncio.get_event_loop().create_task(self._run_queue())
        # The gamepad e-stop (right-stick press) relaxes the course's joystick
        # controllers and publishes /emergency_stop. The controllers used here
        # (walking, tricks) are not on its list, so relax them too.
        self.loop = asyncio.get_event_loop()
        self.node.create_subscription(Empty, "/emergency_stop", self._on_estop, 10)
        self.gamepad_estop_at = 0.0  # monotonic time of the last gamepad e-stop
        # Tilt e-stops: the controller that latched one, until a safe stand-up clears it.
        self.fallen: str | None = None
        self.imu: collections.deque = collections.deque()  # (monotonic t, tilt deg, gyro rad/s)
        self.node.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self._on_imu, 10)
        self.node.create_subscription(Log, "/rosout", self._on_rosout, 100)
        log.info("tool server ready: %d tricks, locomotion controller %s", len(self.tricks), LOCOMOTION_CONTROLLER)

    # ---------------------------------------------------------------- time
    def now(self) -> float:
        """Seconds on a clock that only goes forward (time.monotonic)."""
        return time.monotonic()

    async def sleep(self, seconds: float) -> None:
        """Wait without freezing the server. Never use time.sleep in here: it would
        stop everything else, the stop tool included."""
        await asyncio.sleep(seconds)

    # ---------------------------------------------------------------- ROS helpers
    def _on_reference_status(self, msg: String) -> None:
        parts = str(msg.data).split()
        if (len(parts) != 4 or parts[0] != "pupper_reference_v1"
                or parts[1] != self.reference_request_id
                or parts[2] not in ("playing", "completed", "rejected", "interrupted")):
            return
        # Repeated/out-of-order playing receipts cannot undo a terminal outcome.
        if self.reference_status and self.reference_status[0] in ("completed", "rejected", "interrupted"):
            return
        self.reference_status = (parts[2], parts[3])

    def _on_clip_status(self, controller: str, msg: String) -> None:
        # "<clip> playing|holding frame N queued K [execution <activation-token>]"
        parts = msg.data.split()
        if len(parts) >= 2 and parts[1] in ("playing", "holding"):
            self.clip_status[controller] = (parts[0], parts[1] == "holding")
            # Preserve the legacy interactive status above. Finite completion
            # requires the token snapshotted by the controller for this activation.
            expected = self.finite_execution_requests.get(controller)
            if (len(parts) != 8 or parts[2] != "frame" or parts[4] != "queued"
                    or parts[6] != "execution" or not expected or parts[7] != expected):
                return
            try:
                frame, queued = int(parts[3]), int(parts[5])
            except ValueError:
                return
            if frame < 0 or queued < 0:
                return
            if queued:
                # A queued restart is not a settled finite endpoint. Do not
                # reuse a recently held receipt after this newer status.
                self.clip_receipts.pop(controller, None)
                return
            received = time.monotonic()
            self.clip_receipts[controller] = (parts[0], parts[1] == "holding", frame, received, expected)
            if (parts[0] == "all" and parts[1] == "playing"
                    and controller in self.finite_activation_since
                    and received >= self.finite_activation_since[controller]):
                self.finite_playing_seen.add(controller)

    def _on_image(self, msg: CompressedImage) -> None:
        try:
            self.latest_image.get_nowait()
        except queue.Empty:
            pass
        self.latest_image.put_nowait(msg)

    def _on_imu(self, msg: Imu) -> None:
        q, w = msg.orientation, msg.angular_velocity
        now = time.monotonic()
        while self.imu and self.imu[0][0] < now - SAFE_STILL_S - 0.5:
            self.imu.popleft()
        values = (q.x, q.y, q.z, q.w, w.x, w.y, w.z)
        norm = math.sqrt(sum(v * v for v in (q.x, q.y, q.z, q.w)))
        covariance = getattr(msg, "orientation_covariance", [])
        if (any(not math.isfinite(v) for v in values) or not math.isfinite(norm) or norm < 1e-8
                or (len(covariance) and covariance[0] < 0)):
            self.imu.append((now, float("inf"), float("inf")))
            return  # invalid/missing orientation must never authorize recovery
        # Angle between the body's up axis and the world's: the controllers' tilt check.
        up = max(-1.0, min(1.0, 1.0 - 2.0 * ((q.x / norm) ** 2 + (q.y / norm) ** 2)))
        self.imu.append((now, math.degrees(math.acos(up)), math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)))
        while self.imu and self.imu[0][0] < now - SAFE_STILL_S - 0.5:
            self.imu.popleft()

    def _on_rosout(self, msg: Log) -> None:
        name = msg.name.rsplit(".", 1)[-1]
        if msg.msg != ESTOP_LOG or name not in self.leg_controllers:
            return
        if time.monotonic() - self.gamepad_estop_at < 2.0:
            return  # the gamepad e-stop, which relaxes everything itself
        asyncio.run_coroutine_threadsafe(self._on_fall(name), self.loop)

    async def _on_fall(self, controller: str) -> None:
        if self.fallen:
            return
        self.fallen = controller
        await self.immediate_stop()
        self.state_now = self.state_after_queue = FALLEN
        log.warning("tilt e-stop in %s: the robot fell over; waiting for a safe stand-up", controller)

    def upright(self) -> tuple[bool, str]:
        """Whether the robot is level and still enough to restart a controller."""
        now = time.monotonic()
        samples = list(self.imu)  # the ROS thread keeps appending
        recent = [s for s in samples if s[0] >= now - SAFE_STILL_S]
        if not recent or now - recent[-1][0] > 0.2:
            return False, "no IMU data, so it can't check that the robot is upright"
        if now - samples[0][0] < SAFE_STILL_S:
            return False, "not enough IMU data yet"
        tilt = max(s[1] for s in recent)
        gyro = max(s[2] for s in recent)
        if tilt > SAFE_TILT_DEG:
            return False, f"the robot is tilted {tilt:.0f} degrees (needs under {SAFE_TILT_DEG:.0f}); set it upright on its feet"
        if gyro > SAFE_GYRO_RAD_S:
            return False, "the robot is still moving or being handled; let it rest on the ground"
        return True, f"upright ({tilt:.0f} degrees) and still"

    async def _wait_upright(self) -> tuple[bool, str]:
        generation = self.stops
        deadline = time.monotonic() + SAFE_WAIT_S
        while True:
            if self.stops != generation or self.stopped_during_call():
                return False, "a stop interrupted the request"
            ok, why = self.upright()
            if ok or time.monotonic() > deadline:
                return ok, why
            await asyncio.sleep(0.1)

    async def _call(self, client, request, timeout: float = 3.0):
        # A blocking wait here would prevent the stop tool from being handled.
        deadline = time.monotonic() + timeout
        while not client.wait_for_service(timeout_sec=0.0):
            if time.monotonic() >= deadline:
                return None
            await asyncio.sleep(0.02)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        follower_activation = client is self.follow_on
        activation = follower_activation or (client is self.switch_client and bool(getattr(request, "activate_controllers", [])))
        try:
            while not future.done() and time.monotonic() < deadline:
                await asyncio.sleep(0.02)
            if future.done():
                return future.result()
        except (asyncio.CancelledError, Exception):
            if activation:
                self._watch_uncertain_activation(future, follower=follower_activation)
            raise
        if activation:
            # Cancelling a ROS client future does NOT cancel the remote request.
            # Keep its eventual response so a late activation gets cleaned up.
            self._watch_uncertain_activation(future, follower=follower_activation)
            return None
        if hasattr(future, "cancel"):
            future.cancel()
        return None

    def _watch_uncertain_activation(self, future, *, follower: bool = False) -> None:
        source = "follower" if follower else "controller"
        self.controller_fault = (f"{source} activation outcome was unknown; motion is quarantined. "
                                 "Use the physical e-stop and inspect/restart the controller stack "
                                 "before restarting this tool server")
        async def cleanup():
            try:
                while not future.done():
                    await asyncio.sleep(0.1)
                if follower:
                    self.following = True  # lack of an earlier reply was not proof it stayed off
                    result = await self.stop_following()
                    self.send_speed(0, 0, 0)
                    if not result.ok:
                        relaxed = await self._relax_controllers()
                        log.warning("late follower cleanup failed: %s; %s", result.message, relaxed.message)
                else:
                    result = await self._relax_controllers()
                log.warning("late %s activation cleanup: %s", source, result.message)
            except Exception:
                log.exception("late controller cleanup failed; use the physical e-stop")
        task = asyncio.create_task(cleanup())
        self.activation_cleanup_tasks.add(task)
        task.add_done_callback(self.activation_cleanup_tasks.discard)

    async def controller_states(self) -> dict[str, str]:
        res = await self._call(self.list_client, ListControllers.Request())
        return {c.name: c.state for c in res.controller} if res else {}

    async def is_active(self, controller: str) -> bool:
        """Is this controller driving the legs right now?"""
        return (await self.controller_states()).get(controller) == "active"

    # ---------------------------------------------------------------- the legs
    async def switch_to(self, controller: str | None, start_clip: str = "") -> Result:
        """Hand the legs to one controller, or to none (relax the motors). Every other
        leg controller is deactivated in the same switch, so exactly one policy (or
        nothing) drives the legs.

        It also takes care of: standing up safely after a fall (only once the robot
        is upright and still), loading a controller the launch file missed,
        leaving an interactive trick through its exit clip before another motion
        takes the legs, waiting until the locomotion policy is ready to walk, and
        keeping robot.state_now up to date.
        """
        generation = self.stops
        if controller and self.controller_fault:
            return Result(False, self.controller_fault)
        if controller and self.stopped_during_call():
            return Result(False, "controller change cancelled by a stop")
        activate = [controller] if controller else []
        deactivate = sorted(self.leg_controllers - set(activate))
        states = await self.controller_states()
        if not states:
            return Result(False, "controller_manager is not running; is the robot stack launched?")
        recovered = ""
        if self.fallen and activate:
            ok, why = await self._wait_upright()
            if not ok:
                return Result(False, f"won't stand up: {why}. ({FELL_MESSAGE})")
            # Relax every latched controller first: activating one clears its e-stop.
            latched = [c for c in self.leg_controllers if states.get(c) == "active"]
            if latched:
                req = SwitchController.Request()
                req.deactivate_controllers = latched
                req.strictness = SwitchController.Request.BEST_EFFORT
                reset = await self._call(self.switch_client, req)
                if reset is None:
                    self.controller_fault = ("fall recovery deactivation outcome unknown; motion quarantined. "
                                             "Use the physical e-stop and inspect/restart the controller stack before restarting this tool server")
                    return Result(False, self.controller_fault)
                if not reset.ok:
                    return Result(False, "could not deactivate the fall-latched controllers; fall remains latched")
                states = await self.controller_states()
                if not states or any(states.get(c) == "active" for c in latched):
                    return Result(False, "could not confirm fall-latched controllers inactive; fall remains latched")
            self.walk_since = None
            log.info("recovering from the fall in %s: %s", self.fallen, why)
            recovered = "recovered from the fall and stood back up; "
        not_ready = [c for c in activate if states.get(c) not in LOADED]
        if not_ready:  # the launch's spawner missed it: load it now (a few seconds)
            log.warning("loading %s (state %s)", not_ready, [states.get(c, "not loaded") for c in not_ready])
            await asyncio.to_thread(spawn, not_ready)
            states = await self.controller_states()
            not_ready = [c for c in activate if states.get(c) not in LOADED]
            if not_ready:
                return Result(False, f"could not load controllers {not_ready}; see /tmp/gemini-pupper/robot.log")
        leaving = self.state_now
        if (isinstance(leaving, Trick) and leaving.interactive and leaving.exit_clip and activate
                and controller not in (leaving.controller, leaving.walk_controller if leaving.walks else None)):
            # Leave an interactive trick through its own exit clip before another motion
            # takes the legs (not before relaxing the motors: lying down from a sit is fine).
            if states.get(leaving.controller) == "active":
                res = await self.play_clip(leaving, leaving.exit_clip)
                if not res.ok:
                    return Result(False, f"cannot leave {leaving.name}: {res.message}")
            elif leaving.walks and states.get(leaving.walk_controller) == "active":
                # Walking in its stance: give the legs back to the trick's own policy,
                # started at its exit clip, and let it play.
                res = await self.switch_to(leaving.controller, start_clip=leaving.exit_clip)
                if res.ok:
                    await asyncio.sleep(TRICK_INIT_S + TRICK_FADE_S + leaving.clip_length(leaving.exit_clip) + 0.3)
                    res = await self.confirm_clip_endpoint(leaving, leaving.exit_clip)
                    if not res.ok:
                        return Result(False, f"cannot leave {leaving.name}: {res.message}")
                else:
                    return Result(False, f"cannot leave {leaving.name}: {res.message}")
            states = await self.controller_states()
            deactivate = sorted(self.leg_controllers - set(activate))
        trick = self.state_of(controller)
        if isinstance(trick, Trick) and trick.interactive and controller == trick.controller and states.get(controller) != "active":
            configured = await self._set_start_clip(controller, start_clip)
            if not configured.ok:
                return configured  # never replay an entry motion instead of an exit
            # Completion feedback from a previous activation is not evidence that
            # this entry/exit motion has reached its endpoint.
            self.clip_status[controller] = ("", False)
        if activate and (generation != self.stops or self.stopped_during_call()):
            return Result(False, "controller change cancelled by a stop")
        req = SwitchController.Request()
        req.activate_controllers = [c for c in activate if states[c] != "active"]
        req.deactivate_controllers = [c for c in deactivate if states.get(c) == "active"]
        if states.get(LOCOMOTION_CONTROLLER) == "active" and self.walk_since is None:
            self.walk_since = time.monotonic() - WALK_READY_S  # active before we started
        if not req.activate_controllers and not req.deactivate_controllers:
            self.state_now = self.state_of(controller)
            if controller == LOCOMOTION_CONTROLLER:
                await self._wait_walk_ready()
            if activate and generation != self.stops:
                return Result(False, "controller change cancelled by a stop")
            if recovered:
                self.fallen = None
            return Result(True, recovered + "already in that state")
        req.strictness = SwitchController.Request.STRICT
        if controller in req.activate_controllers and isinstance(trick, Trick) and not trick.interactive:
            configured = await self._set_finite_execution_request(controller)
            if not configured.ok:
                return configured
            if generation != self.stops or self.stopped_during_call():
                self.finite_execution_requests.pop(controller, None)
                return Result(False, "controller change cancelled by a stop")
            self.clip_receipts.pop(controller, None)
            self.clip_status.pop(controller, None)
            self.finite_playing_seen.discard(controller)
            self.finite_activation_since[controller] = time.monotonic()
        res = await self._call(self.switch_client, req)
        if activate and generation != self.stops:
            # A ROS request already in flight can finish after a stop; explicitly
            # remove that late activation before accepting another request.
            relaxed = await self._relax_controllers()
            return Result(False, f"controller activation overlapped a stop; {relaxed.message}"
                          + (f". {self.controller_fault}" if self.controller_fault else ""))
        if res is None or not res.ok:
            why = "no answer from controller_manager" if res is None else "refused by controller_manager"
            log.warning("switch %s -> %s: %s", req.deactivate_controllers, req.activate_controllers, why)
            if res is None:
                # Even without a retained future (e.g. transport failure), a
                # missing response is not evidence that activation was refused.
                self.controller_fault = self.controller_fault or (
                    "controller switch outcome unknown; motion quarantined. Use the physical e-stop "
                    "and inspect/restart the controller stack before restarting this tool server")
                relaxed = await self._relax_controllers()
                return Result(False, f"{self.controller_fault}. Cleanup: {relaxed.message}")
            return Result(False, f"controller switch failed (activate {activate}): {why}")
        self.state_now = self.state_of(controller)
        if recovered:
            self.fallen = None
        if LOCOMOTION_CONTROLLER in req.activate_controllers:
            self.walk_since = time.monotonic()
        if LOCOMOTION_CONTROLLER in req.deactivate_controllers:
            self.walk_since = None
        if controller == LOCOMOTION_CONTROLLER:
            await self._wait_walk_ready()
        if activate and generation != self.stops:
            return Result(False, "controller change cancelled by a stop")
        return Result(True, recovered + "ok")

    def state_of(self, controller: str | None) -> str | Trick:
        """The state a controller holding the legs means (None: MOTORS_IDLE). A walking
        trick's walking policy is that trick's state too."""
        if controller is None:
            return MOTORS_IDLE
        for t in self.tricks.values():
            if t.kind == "policy" and (t.controller == controller or (t.walks and t.walk_controller == controller)):
                return t
        return LOCOMOTION

    def walking_controller(self, state) -> str:
        """The controller move, turn and arc drive in a state: a walking trick's own
        walking policy, else the locomotion policy (which they switch to first)."""
        return state.walk_controller if isinstance(state, Trick) and state.walks else LOCOMOTION_CONTROLLER

    def speeds(self, state=None) -> Speeds:
        """Default speeds and limits in a state (default: robot.state_after_queue, where
        Pupper will be when a move asked for now runs). A walking trick has its own,
        from its trick.yaml; everywhere else, the locomotion policy's."""
        state = self.state_after_queue if state is None else state
        w = state.walk if isinstance(state, Trick) and state.walks else None
        if w is None:
            return SPEEDS
        limits = MoveLimits(vx_min=min(0.05, w.max_forward), vy_min=min(0.05, w.max_sideways), wz_min=min(10.0, w.turn),
                            vx_max=w.max_forward, vy_max=w.max_sideways, wz_max=w.turn,
                            duration_max=LIMITS.duration_max, distance_max=LIMITS.distance_max)
        return Speeds(walk=w.speed, turn=w.turn, limits=limits, where=f" (in {state.name})")

    async def _set_start_clip(self, controller: str, clip: str) -> Result:
        """Set an interactive trick's start_clip parameter ("" = its first clip)."""
        client = self.param_clients.get(controller)
        if client is None:
            client = self.param_clients[controller] = self.node.create_client(SetParameters, f"/{controller}/set_parameters")
        req = SetParameters.Request()
        req.parameters = [Parameter(name="start_clip", value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=clip))]
        result = await self._call(client, req)
        if result is None:
            self.controller_fault = ("start-clip parameter update outcome unknown; motion quarantined. "
                                     "Inspect/restart the controller stack before restarting this tool server")
            return Result(False, self.controller_fault)
        if result is None or not result.results or not all(r.successful for r in result.results):
            return Result(False, f"could not set {controller} start clip; controller not activated")
        return Result(True, "start clip configured")

    async def _set_finite_execution_request(self, controller: str) -> Result:
        """Set a fresh token before activation; the controller snapshots it once."""
        request_id = uuid.uuid4().hex
        client = self.param_clients.get(controller)
        if client is None:
            client = self.param_clients[controller] = self.node.create_client(SetParameters, f"/{controller}/set_parameters")
        req = SetParameters.Request()
        req.parameters = [Parameter(name="execution_request_id", value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=request_id))]
        result = await self._call(client, req)
        if result is None:
            self.finite_execution_requests.pop(controller, None)
            self.controller_fault = ("finite execution token update outcome unknown; motion quarantined. "
                                     "Inspect/restart the controller stack before restarting this tool server")
            return Result(False, self.controller_fault)
        if len(result.results) != 1 or not result.results[0].successful:
            self.finite_execution_requests.pop(controller, None)
            return Result(False, f"could not set {controller} execution token; controller not activated")
        self.finite_execution_requests[controller] = request_id
        return Result(True, "finite execution token configured")

    async def _wait_walk_ready(self) -> None:
        """Wait until gemini_walk has eased the legs to standing and faded its
        policy in. A walk sent before that runs on a half-started controller."""
        generation = self.stops
        deadline = (self.walk_since or time.monotonic()) + WALK_READY_S
        while time.monotonic() < deadline and self.stops == generation:
            await asyncio.sleep(min(0.1, deadline - time.monotonic()))

    def send_speed(self, vx: float, vy: float, wz_deg: float) -> None:
        """One speed command to the locomotion policy: vx forward m/s, vy left m/s,
        wz_deg counter-clockwise deg/s. cmd_vel_mux forgets it after 0.5 s."""
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz_deg) / 57.2958
        self.cmd_pub.publish(t)

    async def _adopt_running_state(self) -> None:
        """A restarted server finds the legs as the last one left them: start from
        that state. An interactive trick still running (the robot sits) is taken up
        again, so the next motion stands up through its exit clip instead of
        jumping out of it."""
        states = await self.controller_states()
        active = [c for c in sorted(self.leg_controllers) if states.get(c) == "active"]
        if active:
            self.state_now = self.state_of(active[0])
            if self.commands.empty():  # a request may already be queued: it set the state after it
                self.state_after_queue = self.state_now
            log.info("found %s holding the legs; starting in state %s", active[0], state_name(self.state_now))
        if self.follow_available:
            # The ROS follower can survive a tool-server restart. Establish that it
            # is off before accepting motion, rather than trusting a fresh False.
            response = await self._call(self.follow_off, Trigger.Request())
            self.follow_state_uncertain = response is None or not response.success
            self.following = self.follow_state_uncertain
            if self.follow_state_uncertain:
                log.warning("cannot confirm follower stopped at startup; motion remains blocked")

    # ---------------------------------------------------------------- the queue
    async def _run_queue(self) -> None:
        try:
            await self._adopt_running_state()
        except Exception:
            log.exception("could not establish the robot state at startup")
            self.startup_error = "robot state could not be established at startup; check the services and restart the tool server"
        finally:
            self.ready.set()
        while True:
            name, job, then = await self.commands.get()
            self._running_job_then = then
            token = CALL_STOPS.set(self.stops)
            self.current = asyncio.create_task(job())
            CALL_STOPS.reset(token)
            try:
                res = await self.current
                (log.info if res.ok else log.warning)("%s: %s", name, res.message)
                self.last_outcome = {"name": name, "success": res.ok, "message": res.message}
                if not res.ok:
                    self.send_speed(0, 0, 0)
                    await self.clear_queue()  # later requests relied on this one succeeding
                    self.state_after_queue = self.state_now
            except asyncio.CancelledError:
                self.send_speed(0, 0, 0)
                log.info("%s: cancelled", name)
                self.last_outcome = {"name": name, "success": False, "message": "cancelled"}
                if self.closing:
                    raise
            except Exception as exc:
                log.exception("%s failed", name)
                self.send_speed(0, 0, 0)
                await self.clear_queue()
                self.state_after_queue = self.state_now
                self.last_outcome = {"name": name, "success": False, "message": f"job failed: {type(exc).__name__}"}
            finally:
                self.current = None
                if self.commands.empty():
                    self.state_after_queue = self.state_now
                    self.busy_until = time.monotonic()

    async def queue(self, name: str, job: Job, seconds: float, *, then) -> Result:
        """Put ``job`` in line behind everything already queued, and return at once.

        ``job`` is an async function with no arguments. It runs *later*, when the
        queue reaches it, and what it returns (the outcome) goes to the debug log.
        What this returns now (the job was accepted) is what Gemini hears.

        ``seconds``: about how long the job takes, for the estimate Gemini hears.
        ``then``: the state Pupper is in once this job is done (LOCOMOTION,
        MOTORS_IDLE, or a trick); it becomes robot.state_after_queue. Jobs that
        don't move the legs (wait) pass UNCHANGED.
        """
        await self.ready.wait()
        if self.startup_error:
            return Result(False, self.startup_error)
        if self.controller_fault:
            return Result(False, self.controller_fault)
        if self.stopped_during_call():
            return Result(False, f"{name} not queued: a stop came in while this request was being handled")
        if self.follow_state_uncertain:
            return Result(False, "motion blocked: could not confirm that the follower stopped; use stop and check the follower service")
        if not math.isfinite(seconds) or seconds < 0:
            return Result(False, "queue duration estimate must be finite and nonnegative")
        if then is not UNCHANGED:
            self.state_after_queue = then
        now = time.monotonic()
        self.busy_until = max(self.busy_until, now) + seconds
        await self.commands.put((name, job, self.state_after_queue))
        return Result(True, f"{name} queued; the queue should finish in {self.busy_until - now:.1f} s")

    def stopped_during_call(self) -> bool:
        """Did a stop come in after the tool call being handled began?"""
        began = CALL_STOPS.get()
        return began is not None and began != self.stops

    def status(self) -> dict:
        return {
            "queued": self.commands.qsize(),
            "running": self.current is not None,
            "seconds_left": round(max(0.0, self.busy_until - time.monotonic()), 1),
            "following": self.following,
            "controller_fault": self.controller_fault,
            "state": self.public_state_name(self.state_now),
            "state_after_queue": self.public_state_name(self.state_after_queue),
            "last_outcome": self.public_outcome(),
        }

    async def wait(self, duration: float) -> Result:
        if isinstance(duration, bool) or not math.isfinite(duration) or not 0 < duration <= 60:
            return Result(False, "wait duration must be finite, more than 0 and at most 60 seconds")
        async def job():
            await asyncio.sleep(duration)
            return Result(True, f"waited {duration} s")

        return await self.queue(f"wait {duration}s", job, duration, then=UNCHANGED)

    async def clear_queue(self) -> Result:
        n = 0
        while not self.commands.empty():
            self.commands.get_nowait()
            n += 1
        self.busy_until = time.monotonic()
        running = self.current is not None and not self.current.done()
        self.state_after_queue = self._running_job_then if running else self.state_now
        return Result(True, f"cleared {n} queued commands")

    # ---------------------------------------------------------------- tricks
    def action_ids(self) -> dict[str, tuple[Trick, str]]:
        """Interactive action ids Gemini uses -> (trick, clip). A clip name, prefixed with the
        trick's name only when two tricks share it."""
        pairs = [(t, clip) for t in self.available_tricks().values() for clip in t.actions]
        names = [clip for t in self.tricks.values() for clip in t.actions]
        return {(clip if names.count(clip) == 1 else f"{t.name}.{clip}"): (t, clip) for t, clip in pairs}

    def trick_available(self, trick: Trick) -> bool:
        return not trick.secret or self.secret_gate.unlocked

    def available_tricks(self) -> dict[str, Trick]:
        return {name: t for name, t in self.tricks.items() if self.trick_available(t)}

    def public_state_name(self, state) -> str:
        return "locked trick" if isinstance(state, Trick) and not self.trick_available(state) else state_name(state)

    def public_outcome(self) -> dict | None:
        outcome = self.last_outcome
        if outcome and not self.secret_gate.unlocked:
            text = outcome["name"] + " " + outcome["message"]
            if any(t.secret and (t.name in text or any(clip in text for clip in t.actions)) for t in self.tricks.values()):
                return {"name": "locked trick", "success": outcome["success"], "message": "details hidden while locked"}
        return outcome

    async def play_clip(self, trick: Trick, clip: str) -> Result:
        """Request a clip and wait until the controller has played it through."""
        if self.fallen or self.stopped_during_call() or self.controller_fault:
            return Result(False, self.controller_fault or "clip refused after a fall or stop")
        async def unconfirmed(message):
            # A missing topic response does not mean the autonomous policy stopped.
            self.controller_fault = message + "; motion quarantined. Inspect/restart the controller stack before restarting this tool server"
            relaxed = await self._relax_controllers()
            return Result(False, f"{message}; {relaxed.message}. {self.controller_fault}")
        self.clip_status[trick.controller] = ("", False)  # forget an earlier activation's status
        self.clip_pubs[trick.controller].publish(String(data=clip))
        length = trick.clip_length(clip)
        longest = max((trick.clip_length(c["name"]) for c in trick.clips), default=0.0)
        deadline = time.monotonic() + longest + 1.0  # the current clip finishes first
        while self.clip_status.get(trick.controller, ("", False))[0] != clip:
            if self.fallen or self.stopped_during_call():
                return Result(False, "clip interrupted by a fall or stop")
            if time.monotonic() > deadline:
                return await unconfirmed(f"{trick.name} did not confirm starting {clip}")
            await asyncio.sleep(0.05)
        deadline = time.monotonic() + length + 1.0
        while True:
            name, holding = self.clip_status.get(trick.controller, ("", False))
            if self.fallen or self.stopped_during_call():
                return Result(False, "clip interrupted by a fall or stop")
            if name != clip or holding:
                break
            if time.monotonic() > deadline:
                return await unconfirmed(f"{trick.name} did not confirm completion of {clip}")
            await asyncio.sleep(0.05)
        return Result(True, f"played {clip}")

    async def confirm_finite_endpoint(self, trick: Trick) -> Result:
        """Confirm a fresh terminal frame after this activation's playing feedback.

        This is controller playback feedback, not measured physical posture.
        Only feedback carrying this activation's controller-snapshotted token
        can complete it; delayed receipts from prior activations are ignored.
        """
        if trick.kind != "policy" or trick.interactive or getattr(trick, "frame_count", 0) < 2:
            return Result(False, "finite playback endpoint metadata unavailable")
        deadline = time.monotonic() + 1.0
        while True:
            if self.fallen or self.stopped_during_call() or self.controller_fault:
                return Result(False, self.controller_fault or "finite playback interrupted by a fall or stop")
            receipt = self.clip_receipts.get(trick.controller)
            began = self.finite_activation_since.get(trick.controller)
            if receipt and began is not None and trick.controller in self.finite_playing_seen:
                name, holding, frame, received, request_id = receipt
                earliest = began + TRICK_INIT_S + TRICK_FADE_S + (trick.frame_count - 1) / trick.fps
                if (request_id == self.finite_execution_requests.get(trick.controller)
                        and name == "all" and holding and frame == trick.frame_count - 1
                        and received >= earliest and 0 <= time.monotonic() - received <= 0.25):
                    return Result(True, f"controller confirmed final frame of {trick.name}")
            if time.monotonic() >= deadline:
                self.controller_fault = (f"{trick.name} did not confirm a fresh final playback frame; motion quarantined. "
                                         "Inspect/restart the controller stack before restarting this tool server")
                relaxed = await self._relax_controllers()
                return Result(False, f"{self.controller_fault}. Cleanup: {relaxed.message}")
            await asyncio.sleep(0.02)

    async def confirm_clip_endpoint(self, trick: Trick, start_clip: str = "") -> Result:
        """Confirm the loop/hold reached by an automatically started clip chain.

        Call after its nominal playback time. A timer cannot establish that an
        entry/exit actually completed; the controller must report its endpoint.
        """
        clip = start_clip or trick.clips[0]["name"]
        seen = set()
        while clip not in seen:
            seen.add(clip)
            endpoint = trick.clip(clip)
            if endpoint.get("loop") or not endpoint.get("next"):
                break
            clip = endpoint["next"]
        else:
            return Result(False, "clip chain has no stable endpoint")
        deadline = time.monotonic() + 1.0
        while True:
            if self.fallen or self.stopped_during_call() or self.controller_fault:
                return Result(False, self.controller_fault or "clip transition interrupted by a fall or stop")
            name, holding = self.clip_status.get(trick.controller, ("", False))
            if name == clip and (holding or endpoint.get("loop")):
                return Result(True, f"controller confirmed {clip}")
            if time.monotonic() >= deadline:
                self.controller_fault = (f"{trick.name} did not confirm its {clip} endpoint; motion quarantined. "
                                         "Inspect/restart the controller stack before restarting this tool server")
                relaxed = await self._relax_controllers()
                return Result(False, f"{self.controller_fault}. Cleanup: {relaxed.message}")
            await asyncio.sleep(0.05)

    async def play_reference_trick(self, trick: Trick) -> Result:
        """Play a correlated reference and require the controller's playback receipt.

        A receipt confirms the reference clock completed, not physical flight,
        landing or posture. Missing receipts leave motion outcome uncertain.
        """
        if self.controller_fault:
            return Result(False, self.controller_fault)
        reference = reference_message_data(trick)
        res = await self.switch_to(LOCOMOTION_CONTROLLER)
        if not res.ok:
            return res
        self.send_speed(0, 0, 0)
        await asyncio.sleep(0.5)
        if self.controller_fault:
            return Result(False, self.controller_fault)
        if self.fallen or self.stopped_during_call():
            return Result(False, "reference motion cancelled by a fall or stop")
        generation = self.stops
        request_id = uuid.uuid4().hex
        self.reference_request_id = request_id
        self.reference_status = None
        request = Float32MultiArray(data=reference)
        request.layout = MultiArrayLayout(dim=[MultiArrayDimension(
            label="pupper_reference_v1:" + request_id, size=len(reference), stride=len(reference))], data_offset=0)
        self.reference_active = True
        async def unconfirmed(message):
            self.controller_fault = message + "; motion outcome unknown; inspect/restart the controller stack before restarting this tool server"
            relaxed = await self._relax_controllers()
            return Result(False, f"{self.controller_fault}. Cleanup: {relaxed.message}")
        try:
            self.ref_pub.publish(request)
            acknowledgement_deadline = time.monotonic() + 1.0
            completion_deadline = time.monotonic() + float(reference[1]) + 2.0
            while True:
                if self.controller_fault:
                    return Result(False, self.controller_fault)
                if generation != self.stops or self.stopped_during_call() or self.fallen:
                    return Result(False, "reference playback interrupted by a fall or stop")
                receipt = self.reference_status
                if receipt:
                    phase, reason = receipt
                    if phase == "rejected":
                        return Result(False, f"controller refused {trick.name} playback: {reason}")
                    if phase == "interrupted":
                        return Result(False, f"controller interrupted {trick.name} playback: {reason}")
                    if phase == "completed":
                        break
                elif time.monotonic() >= acknowledgement_deadline:
                    return await unconfirmed(f"controller did not acknowledge {trick.name}")
                if time.monotonic() >= completion_deadline:
                    return await unconfirmed(f"controller did not confirm completing {trick.name}")
                await asyncio.sleep(0.02)
            # Preserve the class's post-reference settle interval. It is a wait,
            # not evidence of measured posture or a successful physical landing.
            await asyncio.sleep(max(0.0, trick.total_time - float(reference[1])))
            if self.controller_fault:
                return Result(False, self.controller_fault)
            if generation != self.stops or self.stopped_during_call() or self.fallen:
                return Result(False, "reference playback interrupted by a fall or stop")
            return Result(True, f"controller completed {trick.name} reference playback")
        finally:
            self.reference_active = False
            self.reference_request_id = None

    # ---------------------------------------------------------------- follow mode
    async def stop_following(self) -> Result:
        """Turn the person follower off (if it is on)."""
        if self.following:
            result = await self._call(self.follow_off, Trigger.Request())
            if result is None or not result.success:
                self.follow_state_uncertain = True
                return Result(False, "could not confirm that following stopped; new motion refused")
            self.following = False
            self.follow_state_uncertain = False
        return Result(True, "following is off")

    async def set_following(self, enabled: bool) -> Result:
        await self.ready.wait()
        if enabled and self.startup_error:
            return Result(False, self.startup_error)
        if enabled and self.controller_fault:
            return Result(False, self.controller_fault)
        if not enabled:
            result = await self.stop_following()
            self.send_speed(0, 0, 0)
            if not result.ok:
                return result
            return Result(True, "stopped following")
        if self.fallen:
            return Result(False, FELL_MESSAGE)
        if not self.follow_available:
            return Result(False, "following is disabled in this robot's startup configuration")
        if any(isinstance(s, Trick) and s.walks for s in (self.state_now, self.state_after_queue)):
            return Result(False, "following needs the normal locomotion stance; stand_up to leave the walking trick first")
        if self.follow_state_uncertain:
            return Result(False, "following state is unknown; use stop and check the follower service first")
        if self.following:
            return Result(True, "already following the nearest person")
        await self.clear_queue()
        if self.current and not self.current.done():
            self.current.cancel()
            try:
                await self.current
            except asyncio.CancelledError:
                pass
        res = await self.switch_to(LOCOMOTION_CONTROLLER)
        if not res.ok:
            self.state_after_queue = self.state_now
            return res
        self.state_after_queue = LOCOMOTION
        if self.stopped_during_call():  # stop came in while the legs were switching
            return Result(False, "stopped before follow mode started; not following")
        r = await self._call(self.follow_on, Trigger.Request())
        if r is None or not r.success:
            # A timed-out activation might still have reached the service. Confirm
            # deactivation before future movement; don't equate no reply with off.
            self.following = True
            stopped = await self.stop_following()
            if not stopped.ok:
                await self._relax_controllers()
            return Result(False, "person_follower did not confirm startup; " + stopped.message
                          + (f". {self.controller_fault}" if self.controller_fault else ""))
        self.following = True
        if self.stopped_during_call():  # ... or while the follower was starting
            stopped = await self.stop_following()
            if not stopped.ok:
                relaxed = await self._relax_controllers()
                return Result(False, f"stop interrupted follower startup; {stopped.message}; {relaxed.message}")
            return Result(False, "stopped before follow mode started; not following")
        return Result(True, "following the nearest person. Any new motion (walk, turn, trick, stand, lie down) will end follow mode.")

    # ---------------------------------------------------------------- stopping
    def _on_estop(self, _msg: Empty) -> None:
        self.gamepad_estop_at = time.monotonic()
        # Stopping is not evidence that a previously fallen robot is upright.
        asyncio.run_coroutine_threadsafe(self.emergency_stop(), self.loop)

    async def emergency_stop(self) -> Result:
        """Gamepad e-stop: cancel everything and relax every leg controller."""
        await self.immediate_stop()
        return await self._relax_controllers()

    async def _relax_controllers(self) -> Result:
        """Deactivate directly, without an exit motion; used by emergency/failure handling."""
        states = await self.controller_states()
        if not states:
            self.controller_fault = ("could not reach controller_manager to confirm motors relaxed; "
                                     "motion quarantined. Use the physical e-stop and inspect/restart "
                                     "the controller stack before restarting this tool server")
            return Result(False, self.controller_fault)
        active = [c for c in sorted(self.leg_controllers) if states.get(c) == "active"]
        if active:
            req = SwitchController.Request()
            req.deactivate_controllers = active
            req.strictness = SwitchController.Request.BEST_EFFORT
            result = await self._call(self.switch_client, req)
            if result is None or not result.ok:
                self.controller_fault = ("could not confirm motors relaxed; motion quarantined. "
                                         "Use the physical e-stop and inspect/restart the controller "
                                         "stack before restarting this tool server")
                return Result(False, self.controller_fault)
            observed = await self.controller_states()
            if not observed or any(observed.get(c) == "active" for c in self.leg_controllers):
                self.controller_fault = ("controller deactivation response did not confirm inactive controllers; "
                                         "motion quarantined. Use the physical e-stop and inspect/restart "
                                         "the controller stack before restarting this tool server")
                return Result(False, self.controller_fault)
        self.state_now = self.state_after_queue = MOTORS_IDLE
        self.walk_since = None
        log.warning("e-stop: relaxed %s", active or "nothing (no leg controller was active)")
        return Result(True, "e-stop: motors relaxed")

    async def immediate_stop(self, relax_playing_trick: bool = False) -> Result:
        """Clear the queue, cancel the running job, stop following and stand still.

        Cancelling a walk is enough to stop it: the locomotion policy only moves while
        it is sent speeds. A trick policy plays its motion by itself, so with
        relax_playing_trick (the stop tool) a trick that was still playing is
        stopped the only way it can be: by relaxing the motors, like the gamepad
        e-stop. A trick that finished and only holds its pose is left as it is.
        """
        self.stops += 1  # requests still being handled won't queue anything now
        self.secret_gate.lock()
        self.send_speed(0, 0, 0)  # stand still first, before anything that waits
        running = self.current is not None and not self.current.done()
        reference_was_playing = self.reference_active
        cleared = await self.clear_queue()
        if running:
            self.current.cancel()
        self.state_after_queue = self.state_now  # nothing else is coming
        follower = await self.stop_following()
        self.send_speed(0, 0, 0)
        if not follower.ok:
            relaxed = await self._relax_controllers()
            return Result(False, f"{follower.message}. {relaxed.message}")
        if self.controller_fault:
            relaxed = await self._relax_controllers()
            return Result(False, f"{relaxed.message}. {self.controller_fault}")
        if relax_playing_trick and reference_was_playing:
            # A reference jump runs autonomously inside the locomotion controller;
            # a zero velocity does not cancel it. Disable that controller too.
            relaxed = await self._relax_controllers()
            return Result(relaxed.ok, f"reference motion stopped; {relaxed.message}")
        if not (relax_playing_trick and running):
            return Result(True, f"stopped; {cleared.message}")
        await asyncio.sleep(0.1)  # a job cancelled while switching to a trick may still finish the switch
        states = await self.controller_states()
        playing = [t for t in self.tricks.values() if t.kind == "policy" and states.get(t.controller) == "active"]
        if not playing:
            return Result(True, f"stopped; {cleared.message}")
        relaxed = await self._relax_controllers()
        if not relaxed.ok:
            return relaxed
        log.warning("stop: %s was still playing; relaxed the motors", playing[0].name)
        return Result(True, f"stopped; {cleared.message}. {playing[0].name} was still playing, so the motors"
                            " relaxed and the robot is lying down; call stand_up to go on")

    async def shutdown(self) -> Result:
        """Stop autonomous activity before a graceful tool-server exit.

        A completed pose can keep holding; active tricks and reference motions
        use the same stop path as the stop tool. Unresolved remote activation
        remains a fault requiring the physical stop and controller inspection.
        """
        self.closing = True
        self.startup_error = "tool server is shutting down; no new motion is accepted"
        try:
            result = await self.immediate_stop(relax_playing_trick=True)
            pending = set(self.activation_cleanup_tasks)
            if pending:
                _, unresolved = await asyncio.wait(pending, timeout=3.0)
                if unresolved:
                    result = Result(False, "remote activation still pending at shutdown; use the physical e-stop and restart the controller stack")
            return result
        finally:
            self.worker.cancel()
            try:
                await self.worker
            except asyncio.CancelledError:
                pass
            if hasattr(self.executor, "shutdown"):
                self.executor.shutdown(timeout_sec=1.0)
            if hasattr(self.node, "destroy_node"):
                self.node.destroy_node()
            if hasattr(rclpy, "try_shutdown"):
                rclpy.try_shutdown()

    async def mode(self) -> Result:
        """What the robot is doing right now, in words (the robot_status tool)."""
        if self.controller_fault:
            return Result(False, self.controller_fault)
        states = await self.controller_states()
        if not states:
            return Result(False, "controller_manager is not running")
        trick = next((t for t in self.tricks.values() if t.kind == "policy" and states.get(t.controller) == "active"), None)
        walking = next((t for t in self.tricks.values() if t.walks and states.get(t.walk_controller) == "active"), None)
        if self.fallen:
            return Result(True, f"mode: fallen over (tilt e-stop in {self.fallen}). {FELL_MESSAGE}")
        mode = (
            "locomotion, following a person" if self.following
            else "locomotion" if states.get(LOCOMOTION_CONTROLLER) == "active"
            else (f"trick {walking.name}, walking in its {walking.end_pose} stance (move, turn and arc walk slowly in it; "
                  f"actions: {', '.join(walking.actions)})") if walking
            else f"trick {trick.name} ({trick.end_pose}; actions: {', '.join(trick.actions)})" if trick and trick.actions
            else f"trick {trick.name}, holding its end pose ({trick.end_pose})" if trick
            else "motors idle (lying down)"
        )
        if (trick and not self.trick_available(trick)) or (walking and not self.trick_available(walking)):
            mode = "holding a locked trick pose; stand_up leaves it, stop halts movement"
        outcome = self.public_outcome()
        detail = (f" Last queued action {outcome['name']}: "
                  f"{'completed' if outcome['success'] else 'failed'}: {outcome['message']}.") if outcome else ""
        return Result(True, f"mode: {mode}. {self.commands.qsize()} queued; running: {self.current is not None}.{detail}")


# The old name, for staff scripts written against tool_server.ToolServer.
ToolServer = Robot
