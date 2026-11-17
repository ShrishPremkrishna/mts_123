#!/usr/bin/env python3
"""One bounded velocity pulse on an already homed stock session, with a JSON receipt.

Purpose: tell a command-path failure from a gait failure quickly. The tool
publishes one axis pulse on ``/kick_cmd_vel`` (the kick trial mux's only input,
which zeroes ``/cmd_vel`` 150 ms after this process goes quiet) and records, per
tick, the requested command next to the command the native controller reports in
its observation (``/neural_controller_default/observation``, newest frame first,
command at indices 6:9 of the 48-float frame). If applied never follows
requested, the command path (topic, mux, caps, controller input) is at fault; if
it follows but nothing moves, look at the gait.

    movement_trial.py forward                 # dry run: validate, write receipt, no ROS
    movement_trial.py turn_left --duration 1.5 --runtime-receipt RUN/preflight.json --execute

Supervised lateral calibration may explicitly opt into a continuous pulse up to
20 s at at most 0.15 m/s with ``--lateral-calibration`` (left/right only). This
does not measure distance or establish gait stability; use a clear marked lane
and an operator ready to Stop. All ordinary pulses remain limited to 3 s.

What the receipt does NOT claim: the command integral is not achieved distance,
IMU yaw is sensor output rather than external truth, and operator observation is
left pending for the person watching the robot.

``--execute`` moves the robot. It requires the selected stock session (mux only; shot adapter stopped) on top of the plain course stack with teleop off, the pinned stock
controller active with verified caps (``StockWalkerEvidence`` from the shot
adapter, reused unchanged), a released e-stop gateway, exactly the mux on
``/cmd_vel`` and no competing traffic on ``/kick_cmd_vel``. It never arms, homes
or switches controllers. R3/face buttons, ``/emergency_stop`` and a latched
gateway stop the pulse at once; loss of stock authority requests
``/emergency_stop``. Cleanup sends zeros only after a pulse started. If this
process dies, the mux times out. The selected movement release also expires
silent velocity input in the native controller after 250 ms. These are zero
requests, not proof of physical braking or protection against a hung controller.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
BEHAVIORS = HERE.parent
if str(BEHAVIORS) not in sys.path:
  sys.path.insert(0, str(BEHAVIORS))  # run_through_kick_node guard reuse, imported only under ROS

SCHEMA = "pupper_movement_trial_v1"
MAX_PULSE_S = 3.0
MAX_LATERAL_CALIBRATION_S = 20.0
MAX_LATERAL_CALIBRATION_SPEED = 0.15
DEFAULT_SPEED = {"forward": 0.2, "backward": 0.2, "turn": 0.35, "lateral": 0.15}
# Tool ceiling pending a measured policy envelope. The live guard additionally applies the
# verified native caps; the stock node's forward limit is 0.49 and native backward 0.3.
SPEED_CEILING = {"forward": 0.49, "backward": 0.3, "lateral": 0.3, "turn": 1.0}
SPEED_PROFILES = {"walk": SPEED_CEILING,
                  "sprint": {**SPEED_CEILING, "forward": 1.0, "lateral": 0.5}}
AXES = {"forward": ("forward", (1, 0, 0)), "backward": ("backward", (-1, 0, 0)),
        "left": ("lateral", (0, 1, 0)), "right": ("lateral", (0, -1, 0)),
        "turn_left": ("turn", (0, 0, 1)), "turn_right": ("turn", (0, 0, -1))}
OBS_FRAME_SIZE, OBS_COMMAND_OFFSET, OBS_HISTORY = 48, 6, 20  # pinned stock obs_spec
MATCH_TOL = 0.02  # m/s or rad/s: requested vs native filtered command
SETTLE_S = 0.2  # ignore the first ticks after onset when judging agreement
OBS_MAX_AGE_S = 0.2
OWNER_CHECK_S = 0.2
RUN_TRIGGER_MAX_AGE_S = 0.2
ZERO = (0.0, 0.0, 0.0)
CLIENT_IDENTITY = {"path": str(Path(__file__).resolve()),
                   "sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


# ---------------------------------------------------------------- pure helpers


@dataclass(frozen=True)
class PulseRequest:
  axis: str
  kind: str
  speed: float
  duration_s: float
  twist: tuple
  profile: str = "walk"
  lateral_calibration: bool = False

  @property
  def max_duration_s(self) -> float:
    return MAX_LATERAL_CALIBRATION_S if self.lateral_calibration else MAX_PULSE_S

  @classmethod
  def build(cls, axis: str, speed: float | None, duration_s: float, profile="walk",
            *, lateral_calibration=False) -> "PulseRequest":
    if axis not in AXES:
      raise ValueError(f"axis must be one of {sorted(AXES)}")
    if profile not in SPEED_PROFILES:
      raise ValueError("unknown trial profile")
    ceilings = SPEED_PROFILES[profile]
    kind, sign = AXES[axis]
    if speed is None:
      speed = DEFAULT_SPEED[kind]
    speed, duration_s = float(speed), float(duration_s)
    if not math.isfinite(speed) or not 0 < speed <= ceilings[kind]:
      raise ValueError(f"{axis} speed {speed} outside the tool envelope (0, {ceilings[kind]}]")
    if type(lateral_calibration) is not bool:
      raise ValueError("lateral_calibration must be a boolean")
    if lateral_calibration and (kind != "lateral" or speed > MAX_LATERAL_CALIBRATION_SPEED):
      raise ValueError(f"lateral calibration requires left/right at at most {MAX_LATERAL_CALIBRATION_SPEED} m/s")
    limit = MAX_LATERAL_CALIBRATION_S if lateral_calibration else MAX_PULSE_S
    if not math.isfinite(duration_s) or not 0 < duration_s <= limit:
      raise ValueError(f"duration {duration_s} outside the tool envelope (0, {limit}] s")
    return cls(axis, kind, speed, duration_s, tuple(s * speed for s in sign), profile, lateral_calibration)


@dataclass(frozen=True)
class Sample:
  t: float  # seconds since the first published command
  requested: tuple
  applied: tuple | None  # native observation command, or None if no fresh observation


def applied_command(data, frame_size: int = OBS_FRAME_SIZE, offset: int = OBS_COMMAND_OFFSET) -> tuple:
  """Command the controller fed its policy, from the newest observation frame."""
  n = len(data)
  if n != frame_size * OBS_HISTORY:
    raise ValueError(f"observation length {n} does not match {OBS_HISTORY} x {frame_size} stock observations")
  values = tuple(float(v) for v in data[offset:offset + 3])
  if len(values) != 3 or not all(math.isfinite(v) for v in values):
    raise ValueError("observation command slot is missing or not finite")
  return values


def _integral(samples, key):
  total = [0.0, 0.0, 0.0]
  for a, b in zip(samples, samples[1:]):
    value = getattr(a, key)
    if value is None:
      continue
    dt = b.t - a.t
    for i in range(3):
      total[i] += value[i] * dt
  return total


def summarize(samples, started, zero_sent_at, zero_confirmed_at, compare_indices=(0, 1, 2)):
  requested = _integral(samples, "requested")
  observed = [s for s in samples if s.applied is not None]
  compare = [s for s in observed if s.requested != ZERO and s.t >= SETTLE_S]
  diffs = None
  if compare:
    diffs = {k: max(abs(s.requested[i] - s.applied[i]) for s in compare)
             for i, k in enumerate(("vx", "vy", "wz"))}
  onset = next((s.t for s in observed if s.requested != ZERO and
                all(abs(s.requested[i] - s.applied[i]) <= MATCH_TOL for i in compare_indices)), None)
  return {
    "elapsed_s": None if zero_sent_at is None else zero_sent_at - started,
    "ticks": len(samples),
    "observation_samples": len(observed),
    "requested_command_integral": dict(zip(("vx_m", "vy_m", "wz_rad"), requested)),
    "applied_command_integral": (dict(zip(("vx_m", "vy_m", "wz_rad"), _integral(samples, "applied")))
                                 if observed else None),
    "max_abs_requested_minus_applied": diffs,
    "applied_matched_requested": None if diffs is None else all(diffs[("vx", "vy", "wz")[i]] <= MATCH_TOL for i in compare_indices),
    "comparison_axes": [("vx", "vy", "wz")[i] for i in compare_indices],
    "applied_onset_s": onset,
    "zero_confirmed_in_observation_s": (None if zero_confirmed_at is None or zero_sent_at is None
                                        else zero_confirmed_at - zero_sent_at),
  }


IMU_NOTE = ("sensor output, not external truth; yaw from the IMU quaternion on the robot, "
            "not a calibrated heading or translation measurement")


def build_record(*, request, mode, outcome, reason, samples, imu, summary, stock, started_wall,
                 published_commands=0, obs_error=None, path=None):
  if imu is None:
    imu_block = {"status": "no IMU sample received", "note": IMU_NOTE}
  else:
    imu_block = dict(imu, status="sampled", note=IMU_NOTE)
  return {
    "schema": SCHEMA,
    "tool": "final_project/behaviors/tools/movement_trial.py",
    "client_identity": dict(CLIENT_IDENTITY),
    "mode": mode,
    "outcome": outcome,
    "reason": reason,
    "started_wall": started_wall,
    "request": asdict(request),
    "published_commands": published_commands,
    "command_topic": "/kick_cmd_vel",
    "stock_evidence": stock,
    "summary": summary,
    "observation_schema": {
      "topic": "/neural_controller_default/observation",
      "frame_size": OBS_FRAME_SIZE, "command_slice": [OBS_COMMAND_OFFSET, OBS_COMMAND_OFFSET + 3],
      "frame_order": "newest_first", "error": obs_error,
      "meaning": "the controller's filtered command input (after its clip/caps), not achieved motion",
      "heading_hold": "forward/lateral comparison checks X/Y; native heading hold may intentionally change yaw, which remains recorded",
    },
    "imu": imu_block,
    "operator_observation": "pending: fill in observed motion/stability by hand",
    "interpretation": {
      "command_integral_is_distance": False,
      "note": ("requested/applied integrals are integrated commands; actual travel, braking and "
               "stability need floor marks, video or the operator's note"),
    },
    "samples": [asdict(s) for s in samples],
    "file": None if path is None else str(path),
  }


def write_record(record, out_dir) -> Path:
  out_dir = Path(out_dir)
  out_dir.mkdir(parents=True, exist_ok=True)
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
  path = out_dir / f"movement-{stamp}-{record['request']['axis']}.json"
  record["file"] = str(path)
  path.write_text(json.dumps(record, indent=1, default=str) + "\n")
  return path


def _fmt(values, keys):
  return "none" if values is None else ",".join(f"{values[k]:+.3f}" for k in keys)


def finalize_verification(record, runtime_identity_held):
  """A timer finishing is distinct from verifying command delivery and zero."""
  problems = []
  summary = record.get("summary") or {}
  if record["outcome"] != "completed":
    problems.append("pulse did not complete")
  if summary.get("applied_matched_requested") is not True:
    problems.append("requested command was not confirmed at the policy input")
  if summary.get("zero_confirmed_in_observation_s") is None:
    problems.append("zero was not confirmed in controller feedback")
  if not runtime_identity_held:
    problems.append("selected runtime identity did not remain valid")
  record["command_verification"] = {
    "passed": not problems,
    "problems": problems,
    "scope": "command input and selected process identities; physical outcome remains unverified",
  }


def result_exit_code(record):
  if record["outcome"] == "completed":
    return 0 if record.get("command_verification", {}).get("passed") is True else 5
  return 4 if record["outcome"] == "stopped" else 3


def receipt_line(record) -> str:
  req, summary, imu = record["request"], record["summary"], record["imu"]
  parts = [f"movement_trial {req['axis']} {req['speed']:g} {req['duration_s']:g}s -> {record['outcome']}"]
  if record["reason"]:
    parts.append(f"({record['reason']})")
  if summary:
    integral_keys = ("vx_m", "vy_m", "wz_rad")
    parts.append(f"elapsed={summary['elapsed_s']:.2f}s" if summary["elapsed_s"] is not None else "elapsed=none")
    parts.append(f"req_int={_fmt(summary['requested_command_integral'], integral_keys)}")
    parts.append(f"applied_int={_fmt(summary['applied_command_integral'], integral_keys)}")
    parts.append(f"applied_match={summary['applied_matched_requested']}")
    parts.append(f"obs={summary['observation_samples']}")
    zero = summary["zero_confirmed_in_observation_s"]
    parts.append("zero_seen=" + ("none" if zero is None else f"{zero:.2f}s"))
  if record.get("command_verification") is not None:
    check = record["command_verification"]
    parts.append("command_verification=" + ("passed" if check["passed"] else "UNVERIFIED"))
    if check["problems"]:
      parts.append("issues=" + "; ".join(check["problems"]))
  if imu.get("status") == "sampled":
    yaw = imu.get("yaw_change_rad")
    parts.append("imu_yaw_change=" + ("unavailable" if yaw is None else f"{yaw:+.3f}rad"))
  else:
    parts.append("imu=none")
  parts.append("operator_observation=pending")
  if record.get("file"):
    parts.append(f"file={record['file']}")
  return " ".join(parts)


# ---------------------------------------------------------------- ROS node (imported lazily)


def _node_class():
  import threading

  from geometry_msgs.msg import Twist
  from rclpy.node import Node
  from rclpy.qos import qos_profile_sensor_data
  from sensor_msgs.msg import Imu, Joy
  from std_msgs.msg import Empty, Float32MultiArray, String

  from run_through_kick import Command
  from run_through_kick_node import STOP_BUTTONS, StockWalkerEvidence, yaw_from_quaternion

  class MovementTrialNode(Node):
    """Single pulse; phases waiting -> pulsing -> finished. Nothing restarts."""

    def __init__(self, request: PulseRequest, out_dir, *, kick_topic="/kick_cmd_vel",
                 robot_topic="/cmd_vel", obs_topic="/neural_controller_default/observation",
                 imu_topic="/imu_sensor_broadcaster/imu", shot_state_topic="/run_through_kick/state",
                 rate_hz=20.0, require_gateway=True, quiet_s=1.0, clock=time.monotonic):
      super().__init__("movement_trial")
      self.clock = clock
      self.request, self.out_dir = request, Path(out_dir)
      self.kick_topic, self.robot_topic = kick_topic, robot_topic
      self.require_gateway, self.quiet_s = require_gateway, quiet_s
      self.lock = threading.Lock()
      self.phase = "waiting"
      self.record = None
      self.last_problem = ""
      self.stop_requested = None  # session lifetime; prepare never rearms a stop
      self.started = self.zero_sent_at = self.zero_confirmed_at = None
      self.started_wall = None
      self.published = 0
      self.samples = []
      self.recent_own = []  # our own recent twists, to discount loopback on the kick topic
      self.gateway_blocked = None
      self.gateway_seen_at = None
      self.shot_state = None
      self.foreign_cmd_at = None
      self.created_at = self.now()
      self.last_owner_check = None
      self.latest_obs = None  # (applied, time)
      self.obs_error = None
      self.imu_latest = None  # (yaw, time)
      self.imu_count = 0
      self.imu_start = None
      self.imu_end = None
      self.cmd_pub = self.create_publisher(Twist, kick_topic, 10)
      self.robot_pub = self.create_publisher(Twist, robot_topic, 10)  # shutdown zeros only
      self.estop_pub = self.create_publisher(Empty, "/emergency_stop", 10)
      limits = SPEED_PROFILES[request.profile]
      self.stock = StockWalkerEvidence(self, forward_ceiling=limits["forward"],
                                       lateral_ceiling=limits["lateral"])
      self.create_subscription(Empty, "/emergency_stop", self.on_emergency, 10)
      self.create_subscription(Joy, "/joy", self.on_joy, 10)
      self.create_subscription(String, "/estop_controller/stop_status", self.on_stop_status, 10)
      self.create_subscription(String, shot_state_topic, self.on_shot_state, 10)
      self.create_subscription(Twist, kick_topic, self.on_kick_cmd, 10)
      self.create_subscription(Float32MultiArray, obs_topic, self.on_observation, 1)
      self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
      self.create_timer(1.0 / rate_hz, self.on_tick)

    def now(self) -> float:
      return self.clock()

    # -------------------------------------------------------- inputs
    def on_emergency(self, msg) -> None:
      with self.lock:
        self.stop("emergency stop", fault=False)

    def on_joy(self, msg) -> None:
      pressed = [i for i in STOP_BUTTONS if 0 <= i < len(msg.buttons) and msg.buttons[i]]
      if pressed:
        with self.lock:
          self.stop(f"joystick button {pressed[0]}", fault=True)

    def on_stop_status(self, msg) -> None:
      try:
        status = json.loads(msg.data)
      except ValueError:
        return
      blocked = bool(status.get("latched")) or bool(status.get("fault"))
      with self.lock:
        self.gateway_blocked = blocked
        self.gateway_seen_at = self.now()
        if blocked:
          self.stop("e-stop gateway latched", fault=False)

    def on_shot_state(self, msg) -> None:
      try:
        state = json.loads(msg.data).get("state")
      except (ValueError, AttributeError):
        state = "unreadable"
      with self.lock:
        self.shot_state = (state, self.now())
        if state not in ("idle", "done", "aborted"):
          self.stop(f"shot adapter became active ({state})", fault=True)

    def on_kick_cmd(self, msg) -> None:
      twist = (msg.linear.x, msg.linear.y, msg.angular.z)
      with self.lock:
        if twist in self.recent_own:
          self.recent_own.remove(twist)  # loopback of our own publication
          return
        self.foreign_cmd_at = self.now()
        self.stop("another producer published on the kick input", fault=True)

    def on_observation(self, msg) -> None:
      try:
        applied = applied_command(msg.data)
      except ValueError as exc:
        with self.lock:
          self.obs_error, self.latest_obs = str(exc), None
          self.stop("invalid stock policy observation: " + str(exc), fault=True)
        return
      with self.lock:
        now = self.now()
        self.latest_obs = (applied, now)
        self.obs_error = None
        if (self.zero_sent_at is not None and self.zero_confirmed_at is None
            and all(abs(v) <= MATCH_TOL for v in applied)):
          self.zero_confirmed_at = now

    def on_imu(self, msg) -> None:
      yaw = yaw_from_quaternion(msg.orientation)
      if math.isfinite(yaw):
        with self.lock:
          self.imu_latest = (yaw, self.now())
          self.imu_count += 1

    # -------------------------------------------------------- control
    def start_problem(self) -> str | None:
      now = self.now()
      if self.phase != "waiting":
        return f"trial already {self.phase}"
      if self.require_gateway and self.gateway_blocked is not False:
        return ("e-stop gateway latched or faulted" if self.gateway_blocked
                else "no /estop_controller/stop_status yet")
      others = self.count_publishers(self.robot_topic) - 1
      if others != 1:
        return f"{others} publisher(s) on {self.robot_topic} besides this tool; need exactly the kick mux"
      kick_others = self.count_publishers(self.kick_topic) - 1
      if kick_others != 0:
        return f"{kick_others} other publishers on {self.kick_topic}; expected no other command producers"
      if self.shot_state and self.shot_state[0] not in ("idle", "done", "aborted"):
        return f"shot adapter state is {self.shot_state[0]}"
      if now - self.created_at < self.quiet_s:
        return f"listening for competing commands on {self.kick_topic} ({self.quiet_s:g} s)"
      if self.foreign_cmd_at is not None and now - self.foreign_cmd_at < self.quiet_s:
        return f"another producer published on {self.kick_topic} within {self.quiet_s:g} s"
      if not self.latest_obs or not 0 <= now - self.latest_obs[1] <= OBS_MAX_AGE_S:
        return "no fresh stock policy observation; check schema/topic/active controller"
      return self.stock.problem(command=Command(*self.request.twist))

    def try_start(self) -> str | None:
      with self.lock:
        problem = self.start_problem()
        if problem:
          self.last_problem = problem
          return problem
        self.phase = "pulsing"
        self.started = self.now()
        self.started_wall = datetime.now(timezone.utc).isoformat()
        self.imu_start = self.imu_latest[0] if self.imu_latest else None
        self.last_owner_check = self.started
        self.publish(self.request.twist)
        return None

    def prepare(self, request) -> None:
      """Reset only trial data; subscriptions and own-message accounting persist."""
      if self.phase == "pulsing" or (self.phase == "finished" and not self.record.get("file")):
        raise ValueError("previous trial is still running or finalizing")
      if request.profile != self.request.profile:
        raise ValueError("session profile cannot change")
      self.request, self.phase, self.record = request, "waiting", None
      self.last_problem, self.samples, self.published = "", [], 0
      self.started = self.started_wall = self.zero_sent_at = self.zero_confirmed_at = None
      self.imu_start = self.imu_end = self.imu_latest = None
      self.imu_count = 0

    def publish(self, twist, *, record=True) -> None:
      msg = Twist()
      msg.linear.x, msg.linear.y, msg.angular.z = twist
      self.recent_own.append(tuple(twist))
      del self.recent_own[:-20]
      self.cmd_pub.publish(msg)
      if not record:
        return
      self.published += 1
      now = self.now()
      fresh = self.latest_obs if self.latest_obs and now - self.latest_obs[1] <= OBS_MAX_AGE_S else None
      self.samples.append(Sample(t=now - self.started, requested=tuple(twist),
                                 applied=fresh[0] if fresh else None))

    def on_tick(self) -> None:
      with self.lock:
        now = self.now()
        self.stock.poll()
        if self.phase != "pulsing":
          return
        if not self.latest_obs or not 0 <= now - self.latest_obs[1] <= OBS_MAX_AGE_S:
          self.stop("stock policy observation stale", fault=True)
          return
        problem = self.stock.problem(command=Command(*self.request.twist))
        if problem:
          self.stop(problem, fault=True)
          return
        if now - self.last_owner_check >= OWNER_CHECK_S:
          self.last_owner_check = now
          if self.count_publishers(self.robot_topic) != 2:  # the mux and our shutdown publisher
            self.stop(f"another node publishes {self.robot_topic}", fault=True)
            return
          if self.count_publishers(self.kick_topic) != 1:
            self.stop(f"another node publishes {self.kick_topic}", fault=True)
            return
        elapsed = now - self.started
        if not 0 <= elapsed < min(self.request.duration_s, self.request.max_duration_s):
          self.complete("completed", "")
          return
        self.publish(self.request.twist)

    def stop(self, reason: str, *, fault: bool) -> None:
      self.stop_requested = reason
      if self.phase != "pulsing":
        return
      if fault:
        self.estop_pub.publish(Empty())  # zero alone is not a stop for a target-conditioned owner
      self.complete("stopped", reason)

    def complete(self, outcome: str, reason: str) -> None:
      self.publish(ZERO)
      self.zero_sent_at = self.now()
      self.imu_end = self.imu_latest[0] if self.imu_latest else None
      self.phase = "finished"
      self.record = self.build(outcome, reason)

    def build(self, outcome, reason):
      imu = None
      if self.imu_count:
        change = None
        if self.imu_start is not None and self.imu_end is not None:
          change = math.atan2(math.sin(self.imu_end - self.imu_start), math.cos(self.imu_end - self.imu_start))
        imu = {"yaw_start_rad": self.imu_start, "yaw_end_rad": self.imu_end, "yaw_change_rad": change,
               "samples": self.imu_count}
      summary = (summarize(self.samples, self.started, self.zero_sent_at, self.zero_confirmed_at,
                           (0, 1, 2) if self.request.kind == "turn" else (0, 1))
                 if self.started is not None else None)
      stock = {"bounds_forward_backward_lateral_yaw": self.stock.bounds, "error": self.stock.error,
               "note": "configured values and file hash sampled by the shared guard; not a loaded-model proof"}
      return build_record(request=self.request, mode="execute", outcome=outcome, reason=reason,
                          samples=self.samples, imu=imu, summary=summary, stock=stock,
                          started_wall=self.started_wall, published_commands=self.published,
                          obs_error=self.obs_error)

    def finish(self, sleep=time.sleep) -> Path:
      """Zero-on-finally. Safe to call once from the shutdown path."""
      with self.lock:
        if self.phase == "pulsing":
          self.estop_pub.publish(Empty())
          self.complete("stopped", "process shutdown during pulse")
        stock_known = self.stock.problem() is None
        for _ in range(3):
          if self.published:
            self.publish(ZERO, record=False)
          if stock_known and self.published:
            self.robot_pub.publish(Twist())
          sleep(0.05)
        if self.record is None:
          self.record = self.build("not_started", self.last_problem)
        else:
          self.record = self.build(self.record["outcome"], self.record["reason"])
        return write_record(self.record, self.out_dir)

  return MovementTrialNode


def __getattr__(name):
  if name == "MovementTrialNode":
    return _node_class()
  raise AttributeError(name)


# ---------------------------------------------------------------- CLI


def finish_trial(node, runtime, same_manager, *, session=False):
  # A persistent loop has already spun for zero feedback. Do not pause callback
  # handling for another 150 ms while sending its redundant cleanup zeros.
  path = node.finish(sleep=lambda _: None) if session else node.finish()
  node.record["runtime_identity"] = runtime
  node.record["runtime_identity_held"] = same_manager(runtime)
  finalize_verification(node.record, node.record["runtime_identity_held"])
  path.write_text(json.dumps(node.record, indent=1, allow_nan=False) + "\n")
  return path


class SessionInbox:
  """The input thread only parses messages; the ROS loop owns every action."""

  def __init__(self, clock=time.monotonic):
    from queue import SimpleQueue
    self.clock = clock
    self.urgent, self.normal = SimpleQueue(), SimpleQueue()

  def put(self, message):
    queue = self.urgent if message.get("command") in ("stop", "quit") else self.normal
    queue.put((self.clock(), message))

  def read(self, stream):
    try:
      for line in stream:
        try:
          message = json.loads(line)
          if not isinstance(message, dict):
            raise ValueError("command must be a JSON object")
        except (ValueError, TypeError) as exc:
          message = {"command": "invalid", "error": str(exc)}
        self.put(message)
    finally:
      self.put({"command": "quit", "reason": "session stdin closed"})

  def get(self):
    from queue import Empty
    for queue in (self.urgent, self.normal):
      try:
        return queue.get_nowait()
      except Empty:
        pass
    return None


def run_session(node, runtime, same_manager, *, inbox=None, emit=None, spin_once=None):
  """One-use prepare tokens; no deferred start or automatic trial sequence.

  JSON lines: {"command":"list"}, {"command":"status"},
  {"command":"prepare","axis":"forward","speed":0.2,"duration":1},
  {"command":"run","token":"<prepared response token>"},
  {"command":"stop"}, {"command":"quit"}. EOF also stops and exits.
  Each extended left/right prepare must explicitly include
  "lateral_calibration":true (at most 20 s and 0.15 m/s); it is never inherited.
  """
  import threading
  import uuid
  if emit is None:
    emit = lambda message: print(json.dumps(message, allow_nan=False), flush=True)
  if spin_once is None:
    import rclpy
    spin_once = rclpy.spin_once
  if inbox is None:
    inbox = SessionInbox(node.clock)
    threading.Thread(target=inbox.read, args=(sys.stdin,), daemon=True).start()
  token, last_receipt = None, None
  accept_after, next_identity_check = -math.inf, 0.0
  exiting, code, has_run = False, 0, False
  emit({"event": "session_ready", "profile": node.request.profile,
        "client_identity": dict(CLIENT_IDENTITY), "commands": ["list", "prepare", "run", "stop", "status", "quit"]})
  while True:
    spin_once(node, timeout_sec=0.02)
    if node.stop_requested and not exiting:
      exiting, token, code = True, None, 3
      emit({"event": "fault", "reason": node.stop_requested})
    item = None if exiting else inbox.get()  # urgent stop/EOF bypass ordinary backlog
    if item:
      received_at, message = item
      command = message.get("command")
      try:
        if command in ("stop", "quit"):
          token = None
          exiting = True  # a stop is latched until a new session
          if node.phase == "pulsing":
            with node.lock:
              node.stop(message.get("reason", "session " + command), fault=True)
          emit({"event": "stopping", "reason": message.get("reason", command)})
        elif command == "status":
          emit({"event": "status", "phase": node.phase, "prepared": token is not None,
                "problem": node.start_problem() if node.phase == "waiting" else None,
                "last_receipt": last_receipt})
        elif command == "list":
          emit({"event": "trials", "axes": sorted(AXES), "defaults": DEFAULT_SPEED,
                "ceilings": SPEED_PROFILES[node.request.profile], "max_duration_s": MAX_PULSE_S,
                "lateral_calibration": {"axes": ["left", "right"],
                  "max_duration_s": MAX_LATERAL_CALIBRATION_S,
                  "max_speed": MAX_LATERAL_CALIBRATION_SPEED, "requires_per_prepare_opt_in": True}})
        elif command in ("prepare", "run"):
          if exiting or node.phase == "pulsing" or (node.phase == "finished" and not node.record.get("file")):
            raise ValueError("trial is running or finalizing; command was not queued")
          if received_at < accept_after:
            raise ValueError("command predates the last completed trial; send a fresh command")
          if command == "prepare":
            if message.get("profile", node.request.profile) != node.request.profile:
              raise ValueError("session profile cannot change")
            request = PulseRequest.build(message["axis"], message.get("speed"),
                                         message.get("duration", 1.0), node.request.profile,
                                         lateral_calibration=message.get("lateral_calibration", False))
            node.prepare(request)
            token = uuid.uuid4().hex
            emit({"event": "prepared", "token": token, "request": asdict(request),
                  "problem": node.start_problem()})
          else:
            if token is None or message.get("token") != token:
              raise ValueError("run requires the current unused prepare token")
            if not 0 <= node.now() - received_at <= RUN_TRIGGER_MAX_AGE_S:
              raise ValueError("run trigger expired; send a fresh run command")
            next_identity_check = node.now() + OWNER_CHECK_S
            if not same_manager(runtime):
              emit({"event": "fault", "reason": "native manager changed before session trial"})
              return 3
            if (node.require_gateway and (node.gateway_seen_at is None or
                node.now() - node.gateway_seen_at > 0.6)):
              raise ValueError("no fresh e-stop gateway status")
            if (not node.latest_obs or node.now() - node.latest_obs[1] > OBS_MAX_AGE_S or
                any(abs(v) > MATCH_TOL for v in node.latest_obs[0])):
              raise ValueError("start requires a fresh zero command in controller feedback")
            problem = node.try_start()
            if problem:
              raise ValueError(problem)  # never retry a trigger later
            token = None
            has_run = True
            emit({"event": "started", "request": asdict(node.request)})
        else:
          raise ValueError(message.get("error", "unknown command"))
      except (ValueError, KeyError, TypeError) as exc:
        emit({"event": "refused", "command": command, "reason": str(exc)})
    if (has_run and not exiting and (node.phase == "waiting" or
        (node.phase == "finished" and node.record.get("file")))):
      problem = node.stock.problem()
      if not node.latest_obs or node.now() - node.latest_obs[1] > OBS_MAX_AGE_S:
        problem = "stock policy observation stale"
      if node.require_gateway and (node.gateway_blocked is not False or
          node.gateway_seen_at is None or node.now() - node.gateway_seen_at > 0.6):
        problem = "e-stop gateway status unavailable or blocked"
      if problem:
        emit({"event": "fault", "reason": "session authority interrupted: " + problem})
        return 3
    if not exiting and node.now() >= next_identity_check:
      next_identity_check = node.now() + OWNER_CHECK_S
      if not same_manager(runtime):
        exiting, token, code = True, None, 3
        if node.phase == "pulsing":
          with node.lock:
            node.stop("native manager changed during session", fault=True)
        emit({"event": "fault", "reason": "native manager changed during session"})
    if node.phase == "pulsing" and node.now() - node.started >= node.request.duration_s:
      with node.lock:
        node.complete("completed", "wall-clock duration reached")
    if node.phase == "finished" and not node.record.get("file"):
      if node.zero_confirmed_at is not None or node.now() - node.zero_sent_at >= 0.5:
        last_receipt = str(finish_trial(node, runtime, same_manager, session=True))
        accept_after = node.now()
        trial_code = result_exit_code(node.record)
        emit({"event": "trial_complete", "file": last_receipt, "outcome": node.record["outcome"],
              "reason": node.record["reason"], "summary": node.record["summary"],
              "command_verification": node.record["command_verification"]})
        if trial_code:
          exiting, code = True, trial_code  # fault/unconfirmed zero cannot be rearmed
    if exiting and (node.phase == "waiting" or node.record.get("file")):
      return code


def parse_args(argv=None):
  parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                   epilog="Without --execute nothing is imported from ROS and nothing is published.")
  parser.add_argument("axis", nargs="?", choices=sorted(AXES))
  parser.add_argument("--session", action="store_true",
                      help="keep one verified ROS session; accept list/prepare/run/stop/status/quit JSON lines on stdin")
  parser.add_argument("--profile", choices=sorted(SPEED_PROFILES), default="walk",
                      help="must match selected release; sprint is a physically unaccepted candidate")
  parser.add_argument("--speed", type=float, default=None,
                      help=f"m/s or rad/s; defaults {DEFAULT_SPEED}, ceilings {SPEED_CEILING}")
  parser.add_argument("--duration", type=float, default=1.0,
                      help=f"seconds, at most {MAX_PULSE_S} unless --lateral-calibration is explicit")
  parser.add_argument("--lateral-calibration", action="store_true",
                      help="one-shot left/right calibration: up to 20 s at <=0.15 m/s; in sessions opt in on each prepare JSON")
  parser.add_argument("--out", default=str(BEHAVIORS / "logs" / "movement"), help="receipt directory")
  parser.add_argument("--execute", action="store_true",
                      help="publish the pulse; needs the running homed stock session described in ROBOT_RUNBOOK.md")
  parser.add_argument("--runtime-receipt", type=Path, help="required for execution: verified pre-start release receipt")
  parser.add_argument("--wait", type=float, default=6.0, help="seconds to wait for authority before giving up")
  parser.add_argument("--no-gateway", action="store_true", help="older course stacks without /estop_controller")
  return parser.parse_args(argv)


def run_execute(request, args) -> int:
  import signal
  root = HERE.parents[2]
  if str(root) not in sys.path:
    sys.path.insert(0, str(root))
  from foundation.hardware.runtime_identity import read_json, verify_loaded, same_manager
  try:
    if args.runtime_receipt is None:
      raise ValueError("--runtime-receipt is required before motion")
    preflight = read_json(args.runtime_receipt)
    runtime = verify_loaded(preflight)
    if read_json(preflight["manifest_path"]).get("trial_profile") != request.profile:
      raise ValueError("trial profile differs from the selected release")
    Path(args.out).mkdir(parents=True, exist_ok=True)
    import tempfile
    with tempfile.TemporaryFile(dir=args.out) as probe:
      probe.write(b"receipt destination ready")
      probe.flush()
      import os
      os.fsync(probe.fileno())
  except (OSError, ValueError, KeyError, TypeError) as exc:
    print(f"runtime refused: {exc}", file=sys.stderr)
    return 2

  import rclpy
  from rclpy.executors import ExternalShutdownException
  from rclpy.signals import SignalHandlerOptions

  from run_through_kick_node import STOCK_POLICY_SHA256
  if runtime["files"]["policy"]["sha256"] != STOCK_POLICY_SHA256:
    print("runtime refused: release policy differs from the stock guard", file=sys.stderr)
    return 2
  rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

  def interrupt(signum, frame):
    raise KeyboardInterrupt

  previous = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
  node = _node_class()(request, args.out, require_gateway=not args.no_gateway)
  code = 3
  try:
    if args.session:
      code = run_session(node, runtime, same_manager)
    else:
      deadline = time.monotonic() + args.wait
      last_problem, next_try = None, 0.0
      while node.phase == "waiting":
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() >= next_try:
          next_try = time.monotonic() + 0.2
          if node.start_problem() is None:
            current_runtime = verify_loaded(preflight)
            if ((current_runtime["manager_pid"], current_runtime["manager_start_ticks"]) !=
                (runtime["manager_pid"], runtime["manager_start_ticks"])):
              raise ValueError("manager changed while waiting for the trial")
          problem = node.try_start()
          if problem and problem != last_problem:
            print(f"waiting: {problem}", file=sys.stderr)
            last_problem = problem
          if problem and time.monotonic() > deadline:
            print(f"not started: {problem}", file=sys.stderr)
            break
      next_identity_check = 0.0
      while node.phase == "pulsing":
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        # Full process scans can take ~20 ms on the Pi. Running one after every
        # callback starves service replies and the command timer under sensor load.
        if node.phase == "pulsing" and now >= next_identity_check:
          next_identity_check = now + OWNER_CHECK_S
          if not same_manager(runtime):
            with node.lock:
              node.stop("native manager changed during pulse", fault=True)
        if node.phase == "pulsing" and time.monotonic() - node.started >= request.duration_s:
          with node.lock:
            node.complete("completed", "wall-clock duration reached")
      settle = time.monotonic() + 0.5  # give the observation a chance to confirm the zero
      while node.phase == "finished" and node.zero_confirmed_at is None and time.monotonic() < settle:
        rclpy.spin_once(node, timeout_sec=0.05)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  except (OSError, ValueError, KeyError, TypeError) as exc:
    node.last_problem = "runtime refused: " + str(exc)
    if node.phase == "pulsing":
      with node.lock:
        node.stop(node.last_problem, fault=True)
  finally:
    for s in previous:
      signal.signal(s, signal.SIG_IGN)  # a second Ctrl-C must not skip the zeros
    if (not args.session or node.phase == "pulsing" or
        (node.phase == "finished" and not node.record.get("file"))):
      path = finish_trial(node, runtime, same_manager)
      if args.session:
        print(json.dumps({"event": "trial_complete", "file": str(path),
                          "outcome": node.record["outcome"], "reason": node.record["reason"]}), flush=True)
      else:
        print(receipt_line(node.record))
      code = result_exit_code(node.record)
    node.destroy_node()
    rclpy.try_shutdown()
    for signum, handler in previous.items():
      signal.signal(signum, handler)
  return code


def main(argv=None) -> int:
  args = parse_args(argv)
  if args.session and args.lateral_calibration:
    print("refused: sessions require lateral_calibration:true on each prepare JSON", file=sys.stderr)
    return 2
  if args.session and not args.execute:
    print("refused: --session requires --execute; it never starts a trial without prepare/run", file=sys.stderr)
    return 2
  if args.axis is None and not args.session:
    print("refused: an axis is required outside --session", file=sys.stderr)
    return 2
  try:
    request = PulseRequest.build(args.axis or "forward", args.speed, args.duration, args.profile,
                                 lateral_calibration=args.lateral_calibration)
  except ValueError as exc:
    print(f"refused: {exc}", file=sys.stderr)
    return 2
  if not args.execute:
    record = build_record(request=request, mode="dry-run", outcome="dry_run",
                          reason="no --execute: nothing published", samples=[], imu=None,
                          summary=None, stock=None, started_wall=datetime.now(timezone.utc).isoformat())
    write_record(record, args.out)
    print(receipt_line(record))
    return 0
  if not math.isfinite(args.wait) or not 0 < args.wait <= 30:
    print("refused: --wait must be finite and within (0, 30] seconds", file=sys.stderr)
    return 2
  return run_execute(request, args)


if __name__ == "__main__":
  sys.exit(main())
