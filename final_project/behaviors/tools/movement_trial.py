#!/usr/bin/env python3
"""Bounded movement trials on an already homed stock session, with a JSON receipt.

Purpose: tell a command-path failure from a gait failure quickly. The tool
publishes one axis pulse on ``/kick_cmd_vel`` (the trial mux's only input,
which expires after 150 ms without input) and records, per
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

Persistent-session prepare also accepts ``turn_angle_rad`` (relative to yaw at
execution) or ``heading_rad`` (absolute IMU reference), each within [-pi, pi].
Feedback turns require fresh timestamped IMU orientation, use the shortest turn
with exact antipodes choosing left, and stop after stable tolerance or timeout.
Relative angles select a fixed target orientation, not a signed travel path;
feedback may reverse the turn if a disturbance changes the shortest route.
These are software-tested candidates; IMU completion is not physical acceptance.

What the receipt does NOT claim: the command integral is not achieved distance,
IMU yaw is sensor output rather than external truth, and operator observation is
left pending for the person watching the robot.

``--execute`` moves the robot. It requires the selected session (shot adapter
stopped) on top of the plain course stack with teleop off, the pinned stock
controller active with verified caps (``StockWalkerEvidence`` from the shot
adapter, reused unchanged), a released e-stop gateway, and no competing traffic
on ``/kick_cmd_vel``. The default stock topology has the mux on ``/cmd_vel``.
An explicitly selected guarded walk topology routes the mux through
``/fc/requested_cmd_vel`` and the camera guard to ``/cmd_vel``; only that guard
and this client's cleanup-zero publisher may publish on the final topic.
Guarded movement requires fresh camera clearance and current guard evidence.
A camera hold cancels the finite action and revokes its prepare token. After
verified zero and continuing authority, the client remains resident but requires
a fresh prepare/run after clearance. Guard process/status loss ends the client
without automatically restarting or rearming; native authority faults and
explicit Stop retain their emergency-stop behavior. Movement commands never
arm, home or switch controllers. The optional explicit session ``start`` delegates
one activation request to the existing gateway; it never homes. R3/face buttons, ``/emergency_stop`` and a latched
gateway stop the pulse at once; loss of stock authority requests
``/emergency_stop``. Cleanup sends zeros only after a pulse started. If this
process dies, the mux times out. The selected movement release also expires
silent velocity input in the native controller after 250 ms. These are zero
requests, not proof of physical braking or protection against a hung controller.

An explicitly selected ``fc_deadline_v1`` release instead uses only original-
deadline String envelopes through the camera guard. It requires current native
source binding and sends harmless binding zeros while idle, without starting a
trial. Strict qualification is walk-only, at <=0.3 m/s and <=1 rad/s, with every
nonzero deadline capped by the finite trial end. Keep this client resident across
trials; changing to another source process requires deliberate native Stop/Start.
No field observations or synthetic robot poses are needed for these body pulses
and IMU turns. Command feedback still does not establish physical movement.
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
STRICT_SPEED_CEILING = {**SPEED_CEILING, "forward": 0.3}
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
HEADING_MAX_SPEED = 0.35  # initial feedback-turn candidate; native limits still apply
HEADING_MAX_TIMEOUT_S = 20.0
HEADING_TOLERANCE_RAD = 0.05
HEADING_DWELL_S = 0.3
from command_path_evidence import (GUARD_STATUS_TOPIC, GUARD_INPUT_TOPIC,
    GUARD_STATUS_MAX_AGE_S, GUARD_FRAME_MAX_AGE_S, decode_guard_status,
    guard_publishers_problem, guard_status_problem, NATIVE_AUTHORITY_TOPIC,
    decode_native_authority, native_authority_problem)
STRICT_CONTRACT = "fc_deadline_v1"
NATIVE_DISCOVERY_S = 0.3
ZERO = (0.0, 0.0, 0.0)
CLIENT_IDENTITY = {"path": str(Path(__file__).resolve()),
                   "sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def validate_topology_request(topology, request, *, require_gateway=True):
  if topology not in ("stock", "guarded", STRICT_CONTRACT):
    raise ValueError("unsupported command topology")
  if topology != "stock" and request.profile != "walk":
    raise ValueError("guarded topology requires the walk profile")
  if topology == STRICT_CONTRACT:
    if not require_gateway:
      raise ValueError("strict qualification requires the selected stop gateway")
    vx, vy, wz = request.twist
    if (not all(math.isfinite(v) for v in request.twist) or
        math.hypot(vx, vy) > .3 + 1e-12 or abs(wz) > 1.):
      raise ValueError("strict qualification exceeds 0.3 m/s translation or 1 rad/s yaw")


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


def wrap_heading(angle):
  """Shortest signed angle in (-pi, pi]; the exact antipode turns left."""
  value = (angle + math.pi) % (2 * math.pi) - math.pi
  # Adding/subtracting a nonzero initial yaw can round an exact antipode to
  # either side of the boundary. This tie band is only floating-point noise.
  return math.pi if math.pi - abs(value) <= 4 * math.ulp(math.pi) else value


@dataclass(frozen=True)
class HeadingRequest:
  axis: str
  kind: str
  speed: float
  duration_s: float  # hard timeout, never a successful completion criterion
  twist: tuple  # maximum requested turn, used by the unchanged stock guard
  profile: str
  target_rad: float
  relative: bool

  @property
  def max_duration_s(self):
    return HEADING_MAX_TIMEOUT_S

  @classmethod
  def build(cls, target_rad, *, relative, speed=None, timeout_s=15.0, profile="walk"):
    if profile not in SPEED_PROFILES or type(relative) is not bool:
      raise ValueError("invalid heading profile or mode")
    target_rad, speed, timeout_s = (float(target_rad),
      HEADING_MAX_SPEED if speed is None else float(speed), float(timeout_s))
    if not math.isfinite(target_rad) or not -math.pi <= target_rad <= math.pi:
      raise ValueError("heading/turn angle must be finite and within [-pi, pi] radians")
    if not math.isfinite(speed) or not 0 < speed <= min(HEADING_MAX_SPEED, SPEED_PROFILES[profile]["turn"]):
      raise ValueError(f"heading speed outside the tool envelope (0, {HEADING_MAX_SPEED}] rad/s")
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= HEADING_MAX_TIMEOUT_S:
      raise ValueError(f"heading timeout outside (0, {HEADING_MAX_TIMEOUT_S}] seconds")
    return cls("turn_angle" if relative else "heading", "turn", speed, timeout_s,
               (0.0, 0.0, speed), profile, target_rad, relative)


@dataclass
class HeadingControl:
  request: HeadingRequest
  target_rad: float
  started_ros_ns: int = 0
  within_since_ros_ns: int | None = None
  error_rad: float | None = None
  completed: bool = False
  feedback_samples: int = 0
  dwell_feedback_samples: int = 0
  last_capture_ros_ns: int | None = None

  @classmethod
  def start(cls, request, yaw, started_ros_ns=0):
    return cls(request, wrap_heading(yaw + request.target_rad if request.relative else request.target_rad),
               started_ros_ns)

  def update(self, yaw, capture_ros_ns):
    """Dwell measures distinct capture times, never message delivery times."""
    fresh = self.last_capture_ros_ns != capture_ros_ns
    if fresh:
      self.feedback_samples += 1
      self.last_capture_ros_ns = capture_ros_ns
    self.error_rad = wrap_heading(self.target_rad - yaw)
    if abs(self.error_rad) <= HEADING_TOLERANCE_RAD:
      if fresh:
        self.dwell_feedback_samples += 1
      if self.within_since_ros_ns is None:
        self.within_since_ros_ns = max(capture_ros_ns, self.started_ros_ns)
      self.completed = capture_ros_ns - self.within_since_ros_ns >= round(HEADING_DWELL_S * 1e9)
      return ZERO
    self.within_since_ros_ns, self.completed, self.dwell_feedback_samples = None, False, 0
    # No integral term, rate floor, gains or native limits are changed.
    return (0.0, 0.0, max(-self.request.speed, min(self.request.speed, 0.8 * self.error_rad)))


def session_request(message, profile):
  targets = [key for key in ("turn_angle_rad", "heading_rad") if key in message]
  if targets:
    if len(targets) != 1 or any(key in message for key in ("axis", "duration", "lateral_calibration")):
      raise ValueError("use exactly one heading_rad/turn_angle_rad, without pulse/calibration fields")
    return HeadingRequest.build(message[targets[0]], relative=targets[0] == "turn_angle_rad",
                                speed=message.get("speed"), timeout_s=message.get("timeout_s", 15),
                                profile=profile)
  if "timeout_s" in message:
    raise ValueError("timeout_s requires a heading request")
  return PulseRequest.build(message["axis"], message.get("speed"), message.get("duration", 1.0),
                            profile, lateral_calibration=message.get("lateral_calibration", False))


@dataclass(frozen=True)
class Sample:
  t: float  # seconds since the first published command
  requested: tuple
  applied: tuple | None  # native observation command, or None if no fresh observation
  applied_t: float | None = None  # observation receipt, monotonic seconds since first publication


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


def heading_command_verification(samples):
  """Match fresh feedback to a previous publication within the bounded window.

  A matching command's age bounds compatibility; repeated command values cannot
  identify actual transport latency. Observation messages have no capture stamp.
  """
  matched, unmatched, missing, stale, skipped = [], 0, 0, 0, 0
  nonzero_requested = any(any(abs(v) > MATCH_TOL for v in s.requested) for s in samples)
  nonzero_feedback, startup_matched = 0, 0
  seen = set()
  for index, sample in enumerate(samples):
    startup = sample.t < SETTLE_S or (sample.applied_t is not None and sample.applied_t < SETTLE_S)
    if startup:
      skipped += 1
    if sample.applied_t is not None and not 0 <= sample.t - sample.applied_t <= OBS_MAX_AGE_S:
      if sample.t >= SETTLE_S:
        stale += 1
      continue
    if sample.applied is None or sample.applied_t is None:
      if sample.t >= SETTLE_S:
        missing += 1
      continue
    if sample.applied_t in seen:
      continue  # one observation reused by several command ticks is one witness
    seen.add(sample.applied_t)
    candidates = []
    for command in samples[:index]:  # current publication occurs after this observation
      age = sample.applied_t - command.t
      if command.t >= 0 and 0 <= age <= OBS_MAX_AGE_S:
        error = max(abs(a - b) for a, b in zip(sample.applied, command.requested))
        if error <= MATCH_TOL:
          candidates.append((error, age))
    if candidates:
      if startup:
        startup_matched += 1
      else:
        matched.append(min(candidates))
      if any(abs(v) > MATCH_TOL for v in sample.applied):
        nonzero_feedback += 1
    elif not startup:
      unmatched += 1
  return {
    "passed": bool(matched) and not (unmatched or missing or stale) and
              (not nonzero_requested or nonzero_feedback > 0),
    "method": "fresh observation matched to an earlier published command",
    "feedback_window_s": OBS_MAX_AGE_S, "component_tolerance": MATCH_TOL,
    "startup_settle_s": SETTLE_S,
    "matched_observations": len(matched), "unmatched_observations": unmatched,
    "startup_matched_observations": startup_matched,
    "nonzero_command_requested": nonzero_requested, "nonzero_feedback_observations": nonzero_feedback,
    "missing_observation_ticks": missing, "stale_observation_ticks": stale,
    "excluded_startup_ticks": skipped,
    "accepted_command_age_s": None if not matched else {
      "min": min(age for _, age in matched), "max": max(age for _, age in matched)},
    "max_matched_component_error": None if not matched else max(error for error, _ in matched),
    "note": ("bounded compatibility, not measured latency; observation receipt time is not capture time; "
             "nonzero evidence may include matching startup observations"),
  }


def summarize(samples, started, zero_sent_at, zero_confirmed_at, compare_indices=(0, 1, 2),
              *, heading_feedback=False):
  requested = _integral(samples, "requested")
  observed = [s for s in samples if s.applied is not None]
  compare = [s for s in observed if s.requested != ZERO and s.t >= SETTLE_S]
  diffs = None
  if compare:
    diffs = {k: max(abs(s.requested[i] - s.applied[i]) for s in compare)
             for i, k in enumerate(("vx", "vy", "wz"))}
  onset = next((s.t for s in observed if s.requested != ZERO and
                all(abs(s.requested[i] - s.applied[i]) <= MATCH_TOL for i in compare_indices)), None)
  summary = {
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
  if heading_feedback:
    alignment = heading_command_verification(samples)
    summary["feedback_alignment"] = alignment
    summary["applied_matched_requested"] = alignment["passed"]
    summary["max_abs_requested_minus_applied_meaning"] = "instantaneous snapshot difference; not the heading verification criterion"
  return summary


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
  return 4 if record["outcome"] in ("stopped", "interrupted") else 3


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
                 rate_hz=20.0, require_gateway=True, quiet_s=1.0, clock=time.monotonic,
                 command_topology="stock", command_path_check=None,
                 monotonic_ns=time.monotonic_ns, command_clock_id=None, source_instance=None):
      super().__init__("movement_trial")
      self.clock = clock
      self.request, self.out_dir = request, Path(out_dir)
      self.kick_topic, self.robot_topic = kick_topic, robot_topic
      self.require_gateway, self.quiet_s = require_gateway, quiet_s
      validate_topology_request(command_topology, request, require_gateway=require_gateway)
      self.command_topology, self.command_path_check = command_topology, command_path_check
      self.deadline = None
      if command_topology == STRICT_CONTRACT:
        from fc_executor_bridge import DeadlineEncoder, Command as ExpiringCommand
        self.deadline = DeadlineEncoder(monotonic_ns=monotonic_ns, clock_id=command_clock_id,
                                        source_instance=source_instance)
        self.source_command = ExpiringCommand
        self.kick_topic = self.deadline.protocol.REQUEST_TOPIC
      self.authority_status = self.authority_at = None
      self.authority_error = "native source binding unavailable"
      self.command_error = None
      self.stop_received = threading.Event()
      self.operator_stop_received = None
      self.estop_sent = self.ever_run = False
      self.software_start_client = None
      self.guard_status = self.guard_status_at = None
      self.guard_error = "no guard status received"
      self.guard_source = self.guard_frame_deadline = None
      self.guard_fault = None
      self.guard_ready_once = False
      self.command_path_valid, self.command_path_checked_at = None, None
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
      self.gateway_released_once = False
      self.gateway_authority = None  # immutable (instance, Stop epoch) once observed
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
      self.imu_source_stamp_ns = None
      self.heading = None
      self.cmd_pub = self.create_publisher(String if self.deadline else Twist, self.kick_topic, 10)
      self.robot_pub = None if self.deadline else self.create_publisher(Twist, robot_topic, 10)
      self.estop_pub = self.create_publisher(Empty, "/emergency_stop", 10)
      limits = STRICT_SPEED_CEILING if self.deadline else SPEED_PROFILES[request.profile]
      self.stock = StockWalkerEvidence(self, forward_ceiling=limits["forward"],
          lateral_ceiling=limits["lateral"], **({"command_contract": STRICT_CONTRACT,
          "command_clock_id": self.deadline.clock_id} if self.deadline else {}))
      self.create_subscription(Empty, "/emergency_stop", self.on_emergency, 10)
      self.create_subscription(Joy, "/joy", self.on_joy, 10)
      self.create_subscription(String, "/estop_controller/stop_status", self.on_stop_status, 10)
      self.create_subscription(String, shot_state_topic, self.on_shot_state, 10)
      if self.deadline:
        self.create_subscription(String, NATIVE_AUTHORITY_TOPIC, self.on_native_authority,
                                 qos_profile_sensor_data)
      else:
        self.create_subscription(Twist, kick_topic, self.on_kick_cmd, 10)
      self.create_subscription(Float32MultiArray, obs_topic, self.on_observation, 1)
      self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
      if self.command_topology != "stock":
        self.create_subscription(String, GUARD_STATUS_TOPIC, self.on_guard_status, 1)
      self.create_timer(1.0 / rate_hz, self.on_tick)

    def now(self) -> float:
      return self.clock()

    def check_command_path(self):
      if self.command_topology == "stock":
        return True
      try:
        valid = self.command_path_check is not None and self.command_path_check() is True
      except (OSError, ValueError, KeyError, TypeError):
        valid = False
      self.command_path_valid, self.command_path_checked_at = valid, self.now()
      if not valid:
        with self.lock:
          self.interrupt_guard("selected guard process identity lost", fatal=True)
      return valid

    def guard_publishers_problem(self, *, allow_absent_native=False):
      return guard_publishers_problem(self, "/movement_trial", robot_topic=self.robot_topic,
          kick_topic=self.kick_topic, command_contract=STRICT_CONTRACT if self.deadline else "legacy_twist",
          allow_absent_native=allow_absent_native)

    def guard_problem(self, *, camera=True, allow_absent_native=False):
      if self.command_topology == "stock":
        return None
      problem = self.guard_publishers_problem(allow_absent_native=allow_absent_native)
      if problem and problem[0] == "authority":
        return problem
      if self.guard_fault:
        return "path", self.guard_fault
      if (self.command_path_valid is not True or self.command_path_checked_at is None or
          not 0 <= self.now() - self.command_path_checked_at <= GUARD_STATUS_MAX_AGE_S):
        return "path", "no fresh selected guard process identity"
      if problem:
        return problem
      return guard_status_problem(self.guard_status, self.guard_status_at, self.guard_error,
          self.guard_frame_deadline, self.now(), camera=camera,
          command_contract=STRICT_CONTRACT if self.deadline else "legacy_twist")

    def native_binding_problem(self):
      if self.deadline:
        return native_authority_problem(self.authority_status, self.authority_at,
            self.authority_error, self.now(), self.deadline.source_instance)
      return None

    def on_native_authority(self, msg):
      with self.lock:
        now = self.now()
        try:
          self.authority_status = decode_native_authority(msg.data, now,
              self.deadline.clock_id, self.authority_status)
          self.authority_error = None
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
          self.authority_error = "invalid native command authority: " + str(exc)
        self.authority_at = now
        if self.ever_run and self.native_binding_problem():
          self.stop(self.native_binding_problem(), fault=True)

    def on_guard_status(self, msg):
      with self.lock:
        now = self.now()
        try:
          status, self.guard_source, self.guard_frame_deadline = decode_guard_status(
            msg.data, now, self.guard_source, self.guard_frame_deadline,
            **({"command_contract": STRICT_CONTRACT, "clock_id": self.deadline.clock_id,
                "source_instance": self.deadline.source_instance} if self.deadline else {}))
          self.guard_status, self.guard_status_at, self.guard_error = status, now, None
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
          self.guard_error = "invalid guard status: " + str(exc)
          self.guard_status_at = now
        if self.phase == "pulsing":
          problem = self.guard_problem()
          if problem:
            if problem[0] == "authority":
              self.stop(problem[1], fault=True)
            else:
              self.interrupt_guard(problem[1], fatal=problem[0] != "camera")

    def interrupt_guard(self, reason, *, fatal):
      if fatal:
        self.guard_fault = reason
      self.last_problem = reason
      if self.phase == "pulsing":
        self.complete("interrupted", reason)

    # -------------------------------------------------------- inputs
    def on_emergency(self, msg) -> None:
      self.stop_received.set()  # receipt fence precedes the control lock
      with self.lock:
        self.stop("emergency stop", fault=False)

    def on_joy(self, msg) -> None:
      pressed = [i for i in STOP_BUTTONS if 0 <= i < len(msg.buttons) and msg.buttons[i]]
      if pressed:
        self.stop_received.set()
        with self.lock:
          self.stop(f"joystick button {pressed[0]}", fault=True)

    def on_stop_status(self, msg) -> None:
      try:
        status = json.loads(msg.data)
      except ValueError:
        return
      if not isinstance(status, dict):
        return
      unsafe = any(bool(status.get(key)) for key in
                   ("fault", "stop_required", "unknown_remote_submission", "shutting_down"))
      blocked = bool(status.get("latched")) or unsafe
      if blocked and (unsafe or self.gateway_released_once or self.phase == "pulsing"):
        self.stop_received.set()
      with self.lock:
        has_identity = "instance_id" in status or "epoch" in status
        if self.deadline or has_identity or self.gateway_authority is not None:
          if (status.get("protocol") != "pupper_stop_v1" or
              not isinstance(status.get("instance_id"), str) or not status["instance_id"] or
              type(status.get("epoch")) is not int or not 0 <= status["epoch"] < 2**64):
            self.gateway_blocked = True
            self.stop("gateway authority identity unavailable", fault=False)
            return
          identity = (status["instance_id"], status["epoch"])
          if self.gateway_authority is None:
            self.gateway_authority = identity
          elif identity != self.gateway_authority:
            self.gateway_blocked = True
            self.stop("gateway authority epoch or instance changed", fault=False)
            return
        self.gateway_blocked = blocked
        self.gateway_seen_at = self.now()
        if not blocked:
          self.gateway_released_once = True
        if blocked and (unsafe or self.gateway_released_once or self.phase == "pulsing"):
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
      with self.lock:
        yaw = yaw_from_quaternion(msg.orientation)
        if isinstance(self.request, HeadingRequest):
          q = msg.orientation
          values = (q.x, q.y, q.z, q.w)
          problem = None
          stamp = None
          if (not all(math.isfinite(v) for v in values) or
              not 0.95 <= sum(v * v for v in values) <= 1.05 or
              getattr(msg, "orientation_covariance", [0])[0] == -1):
            problem = "invalid IMU orientation"
          else:
            try:
              sec, nanosec = msg.header.stamp.sec, msg.header.stamp.nanosec
              if type(sec) is not int or type(nanosec) is not int or not 0 <= nanosec < 1_000_000_000:
                raise ValueError("invalid IMU capture timestamp")
              stamp = sec * 1_000_000_000 + nanosec
              age = self.get_clock().now().nanoseconds - stamp
              if not 0 <= age <= round(OBS_MAX_AGE_S * 1e9):
                problem = "IMU capture timestamp stale or in the future"
              elif self.imu_source_stamp_ns is not None and stamp < self.imu_source_stamp_ns:
                problem = "IMU capture timestamp moved backwards"
            except (AttributeError, TypeError, ValueError):
              problem = "IMU capture timestamp unavailable"
          if problem:
            self.imu_latest = None
            if self.phase == "pulsing":
              self.stop(problem, fault=True)
            return
          if stamp == self.imu_source_stamp_ns:
            return  # repeated capture cannot refresh freshness or complete dwell
          self.imu_source_stamp_ns = stamp
        if math.isfinite(yaw):
          self.imu_latest = (yaw, self.now())
          self.imu_count += 1

    # -------------------------------------------------------- control
    def start_problem(self) -> str | None:
      now = self.now()
      if self.deadline and (self.stop_requested or self.shutdown_received()):
        return "session Stop is latched"
      if self.phase != "waiting":
        return f"trial already {self.phase}"
      if self.require_gateway and self.gateway_blocked is not False:
        return ("e-stop gateway latched or faulted" if self.gateway_blocked
                else "no /estop_controller/stop_status yet")
      if not self.deadline:
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
      if self.deadline:
        if self.gateway_seen_at is None or not 0 <= now - self.gateway_seen_at <= .6:
          return "no fresh e-stop gateway status"
        if any(abs(v) > MATCH_TOL for v in self.latest_obs[0]):
          return "start requires a fresh zero command in controller feedback"
        if self.native_binding_problem():
          return self.native_binding_problem()
      if isinstance(self.request, HeadingRequest) and self.heading_problem():
        return self.heading_problem()
      problem = self.stock.problem(command=Command(*self.request.twist))
      if problem:
        return problem
      problem = self.guard_problem()
      return problem[1] if problem else None

    def heading_problem(self):
      if not self.imu_latest or not 0 <= self.now() - self.imu_latest[1] <= OBS_MAX_AGE_S:
        return "no fresh IMU yaw for feedback turn"
      if (self.imu_source_stamp_ns is None or not 0 <=
          self.get_clock().now().nanoseconds - self.imu_source_stamp_ns <= round(OBS_MAX_AGE_S * 1e9)):
        return "IMU capture timestamp stale or in the future"
      return None

    def try_start(self) -> str | None:
      if not self.check_command_path():
        return self.last_problem
      with self.lock:
        problem = self.start_problem()
        if problem:
          self.last_problem = problem
          return problem
        self.phase = "pulsing"
        self.ever_run = True
        self.started = self.now()
        self.started_wall = datetime.now(timezone.utc).isoformat()
        self.imu_start = self.imu_latest[0] if self.imu_latest else None
        self.last_owner_check = self.started
        self.guard_ready_once = self.command_topology != "stock"
        if isinstance(self.request, HeadingRequest):
          self.heading = HeadingControl.start(self.request, self.imu_start,
                                              self.get_clock().now().nanoseconds)
        self.publish(self.heading.update(self.imu_latest[0], self.imu_source_stamp_ns)
                     if self.heading else self.request.twist)
        return None

    def prepare(self, request) -> None:
      """Reset only trial data; subscriptions and own-message accounting persist."""
      validate_topology_request(self.command_topology, request, require_gateway=self.require_gateway)
      if self.phase == "pulsing" or (self.phase == "finished" and not self.record.get("file")):
        raise ValueError("previous trial is still running or finalizing")
      if request.profile != self.request.profile:
        raise ValueError("session profile cannot change")
      self.request, self.phase, self.record = request, "waiting", None
      self.last_problem, self.samples, self.published = "", [], 0
      self.started = self.started_wall = self.zero_sent_at = self.zero_confirmed_at = None
      self.imu_start = self.imu_end = self.imu_latest = None
      self.imu_count = 0
      self.imu_source_stamp_ns = self.heading = None

    def shutdown_received(self):
      return self.stop_received.is_set() or (self.operator_stop_received is not None and
                                             self.operator_stop_received.is_set())

    def publish(self, twist, *, record=True) -> None:
      if self.deadline:
        now = self.now()
        if any(twist) and (self.phase != "pulsing" or self.stop_requested or self.shutdown_received()):
          twist = ZERO
        expiry = now + .15
        if any(twist):
          expiry = min(expiry, self.started + min(self.request.duration_s, self.request.max_duration_s))
        payload, nonzero, self.command_error = self.deadline.encode(self.source_command(*twist, expires_at=expiry))
        if payload is None:
          return
        msg = String(data=payload)
        if not nonzero:
          twist = ZERO  # expired/invalid originals are recorded as the zero actually sent
      else:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = twist
        self.recent_own.append(tuple(twist))
        del self.recent_own[:-20]
      # Check after message construction as well as source serialization.
      if self.deadline and nonzero and (self.stop_requested or self.shutdown_received() or self.phase != "pulsing"):
        return
      self.cmd_pub.publish(msg)
      if not record or self.started is None:
        return
      self.published += 1
      now = self.now()
      fresh = self.latest_obs if self.latest_obs and 0 <= now - self.latest_obs[1] <= OBS_MAX_AGE_S else None
      self.samples.append(Sample(t=now - self.started, requested=tuple(twist),
                                 applied=fresh[0] if fresh else None,
                                 applied_t=self.latest_obs[1] - self.started if self.latest_obs else None))

    def on_tick(self) -> None:
      with self.lock:
        now = self.now()
        self.stock.poll()
        if self.deadline:
          if self.stop_requested or self.shutdown_received():
            if self.phase == "pulsing":
              self.stop("Stop received before publication", fault=True)
            return
          if self.ever_run and self.native_binding_problem():
            self.stop(self.native_binding_problem(), fault=True)
            return
        if self.phase != "pulsing":
          if self.deadline:
            self.publish(ZERO, record=False)  # binding never starts or renews a finite trial
          return
        if not self.latest_obs or not 0 <= now - self.latest_obs[1] <= OBS_MAX_AGE_S:
          self.stop("stock policy observation stale", fault=True)
          return
        problem = self.stock.problem(command=Command(*self.request.twist))
        if self.deadline and (self.gateway_blocked is not False or self.gateway_seen_at is None or
                             not 0 <= now - self.gateway_seen_at <= .6):
          problem = "e-stop gateway status unavailable or blocked"
        if problem:
          self.stop(problem, fault=True)
          return
        if now - self.last_owner_check >= OWNER_CHECK_S:
          self.last_owner_check = now
          if self.command_topology == "stock" and self.count_publishers(self.robot_topic) != 2:
            self.stop(f"another node publishes {self.robot_topic}", fault=True)
            return
          if self.count_publishers(self.kick_topic) != 1:
            self.stop(f"another node publishes {self.kick_topic}", fault=True)
            return
        if self.command_topology != "stock":
          problem = self.guard_problem()
          if problem:
            if problem[0] == "authority":
              self.stop(problem[1], fault=True)
            else:
              self.interrupt_guard(problem[1], fatal=problem[0] != "camera")
            return
        elapsed = now - self.started
        if not 0 <= elapsed < min(self.request.duration_s, self.request.max_duration_s):
          self.expire()
          return
        if self.heading:
          problem = self.heading_problem()
          if problem:
            self.stop(problem, fault=True)
            return
          twist = self.heading.update(self.imu_latest[0], self.imu_source_stamp_ns)
          if self.heading.completed:
            self.complete("completed", "IMU heading held within tolerance")
          else:
            self.publish(twist)
          return
        self.publish(self.request.twist)

    def expire(self):
      if self.heading:
        self.stop("heading timeout before stable completion", fault=True)
      else:
        self.complete("completed", "wall-clock duration reached")

    def stop(self, reason: str, *, fault: bool) -> None:
      self.stop_requested = reason
      self.stop_received.set()
      if fault and (self.phase == "pulsing" or self.deadline):
        self.request_cancel(reason)
      if self.phase != "pulsing":
        return
      self.complete("stopped", reason)

    def request_emergency(self):
      if not self.deadline or not self.estop_sent:
        self.estop_sent = True
        self.estop_pub.publish(Empty())

    def request_cancel(self, reason):
      caller = self.software_start_client
      if caller is not None and (caller.last_request is not None or caller.stop_capable):
        caller.request_stop(reason)
      else:
        self.request_emergency()  # old gateways remain Options-only

    def cancel_start(self, reason):
      """A Stop during pending/uncertain activation must reach the gateway too."""
      with self.lock:
        self.request_cancel(reason)
        self.stop(reason, fault=False)

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
                           (0, 1, 2) if self.request.kind == "turn" else (0, 1),
                           heading_feedback=self.heading is not None)
                 if self.started is not None else None)
      stock = {"bounds_forward_backward_lateral_yaw": self.stock.bounds, "error": self.stock.error,
               "note": "configured values and file hash sampled by the shared guard; not a loaded-model proof"}
      record = build_record(request=self.request, mode="execute", outcome=outcome, reason=reason,
                          samples=self.samples, imu=imu, summary=summary, stock=stock,
                          started_wall=self.started_wall, published_commands=self.published,
                          obs_error=self.obs_error)
      record["command_topology"] = self.command_topology
      if self.software_start_client is not None:
        record["software_stop_result"] = self.software_start_client.stop_result
        record["software_stop_pending"] = self.software_start_client.stop_pending
      if self.deadline:
        record.update(command_topic=self.kick_topic, command_contract=STRICT_CONTRACT,
            command_clock_id=self.deadline.clock_id, source_instance=self.deadline.source_instance,
            source_sequence=self.deadline.sequence, command_error=self.command_error,
            native_authority=self.authority_status, native_authority_error=self.native_binding_problem())
      if self.command_topology != "stock":
        record["camera_guard"] = {
          "status": self.guard_status, "status_error": self.guard_error,
          "status_receipt_age_s": None if self.guard_status_at is None else self.now() - self.guard_status_at,
          "capture_valid_until_monotonic": self.guard_frame_deadline,
          "process_identity_valid": self.command_path_valid, "fault": self.guard_fault,
          "hard_stop_reason": self.stop_requested,
          "scope": "selected process and named ROS publishers are operational evidence, not authentication",
        }
      if self.heading:
        record["heading_feedback"] = {
          "reference": "IMU yaw; not field localization or externally verified heading",
          "shortest_turn_range": "(-pi, pi]; exact antipode turns left",
          "relative_semantics": "fixed target orientation at execution; shortest route may reverse after disturbance",
          "initial_yaw_rad": self.imu_start, "target_yaw_rad": self.heading.target_rad,
          "final_yaw_rad": self.imu_end, "final_error_rad": (
            None if self.imu_end is None else wrap_heading(self.heading.target_rad - self.imu_end)),
          "tolerance_rad": HEADING_TOLERANCE_RAD, "required_dwell_s": HEADING_DWELL_S,
          "dwell_clock": "distinct IMU capture timestamps in ROS time; starts no earlier than execution",
          "execution_ros_ns": self.heading.started_ros_ns,
          "within_since_ros_ns": self.heading.within_since_ros_ns,
          "last_capture_ros_ns": self.heading.last_capture_ros_ns,
          "capture_dwell_s": (None if self.heading.within_since_ros_ns is None else max(
            0, self.heading.last_capture_ros_ns - self.heading.within_since_ros_ns) * 1e-9),
          "feedback_samples": self.heading.feedback_samples,
          "dwell_feedback_samples": self.heading.dwell_feedback_samples,
          "stable_completion": self.heading.completed,
        }
      return record

    def finish(self, sleep=time.sleep) -> Path:
      """Zero-on-finally. Safe to call once from the shutdown path."""
      with self.lock:
        if self.phase == "pulsing":
          self.request_cancel("process shutdown during pulse")
          self.complete("stopped", "process shutdown during pulse")
        stock_known = self.stock.problem() is None
        for _ in range(3):
          if self.published:
            self.publish(ZERO, record=False)
          if stock_known and self.published and self.robot_pub is not None:
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
  if node.command_topology != "stock":
    node.record["command_path_identity_held"] = node.check_command_path()
    node.record["camera_guard"]["process_identity_valid"] = node.record["command_path_identity_held"]
    node.record["camera_guard"]["fault"] = node.guard_fault
    if node.record["outcome"] == "interrupted":
      zero = node.record["summary"].get("zero_confirmed_in_observation_s") is not None
      native = (node.record["runtime_identity_held"] and node.stock.problem() is None and
                node.native_binding_problem() is None and not node.stop_requested)
      guard = node.record["command_path_identity_held"] and node.guard_problem(camera=False) is None
      node.record["interruption_verification"] = {
        "passed": bool(zero and native and guard), "zero_confirmed": zero,
        "native_authority_held": native, "guard_path_held": guard,
        "scope": "finite trial cancelled; only zero and continuing authority verified, requested action not completed",
      }
      if not node.stop_requested and (not zero or not native):
        node.cancel_start("guard cancellation zero unconfirmed" if not zero else
                          "native authority lost during guard cancellation")
        node.record["camera_guard"]["hard_stop_reason"] = node.stop_requested
  path.write_text(json.dumps(node.record, indent=1, allow_nan=False) + "\n")
  return path


class SessionInbox:
  """The input thread only parses messages; the ROS loop owns every action."""

  def __init__(self, clock=time.monotonic, *, capture_start=None):
    from queue import SimpleQueue
    import threading
    self.clock = clock
    self.capture_start = capture_start
    self.stop_seen = threading.Event()
    self.urgent, self.normal = SimpleQueue(), SimpleQueue()

  def put(self, message):
    received_at = self.clock()
    message = dict(message)
    message.pop("_start_ticket", None)
    message.pop("_start_error", None)
    if message.get("command") in ("stop", "quit"):
      self.stop_seen.set()
    if message.get("command") == "start":
      try:
        if self.capture_start is None:
          raise ValueError("Start capability unavailable at input receipt; no action queued")
        message["_start_ticket"] = self.capture_start(pressed_at=received_at)
      except ValueError as exc:
        message["_start_error"] = str(exc)
    queue = self.urgent if message.get("command") in ("stop", "quit") else self.normal
    queue.put((received_at, message))

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

  def get(self, *, urgent_only=False):
    from queue import Empty
    for queue in ((self.urgent,) if urgent_only else (self.urgent, self.normal)):
      try:
        return queue.get_nowait()
      except Empty:
        pass
    return None


def run_session(node, runtime, same_manager, *, inbox=None, emit=None, spin_once=None, start_client=None):
  """One-use prepare tokens; no deferred start or automatic trial sequence.

  JSON lines: {"command":"list"}, {"command":"status"},
  {"command":"prepare","axis":"forward","speed":0.2,"duration":1},
  {"command":"run","token":"<prepared response token>"},
  {"command":"stop"}, {"command":"quit"}. EOF also stops and exits.
  Each extended left/right prepare must explicitly include
  "lateral_calibration":true (at most 20 s and 0.15 m/s); it is never inherited.
  Feedback turns use {"command":"prepare","turn_angle_rad":1.57,"timeout_s":15}
  or heading_rad for an absolute IMU heading. Both accept [-pi,pi], take the
  shortest turn (exact +/-pi chooses left), and default to <=0.35 rad/s.
  Relative targets are anchored to fresh measured yaw at run, never at prepare.
  {"command":"start"} makes one explicit gateway Start request using status
  already observed at input receipt. It never homes, retries, or runs a trial.
  """
  import threading
  import uuid
  if emit is None:
    emit = lambda message: print(json.dumps(message, allow_nan=False), flush=True)
  if spin_once is None:
    import rclpy
    spin_once = rclpy.spin_once
  if start_client is None:
    try:
      from foundation.hardware.software_start import attach
      start_client = attach(node, clock=node.clock, emergency_stop=node.request_emergency)
    except ImportError:
      pass  # legacy releases lack StartMotion; existing Options/movement still work

  def capture_start(*, pressed_at):
    if start_client is None:
      raise ValueError("Start service interface unavailable; no action queued")
    # Freeze already-observed gateway state before waiting for the ROS node's
    # lock. Queueing/lock contention must not refresh the operator's press.
    ticket = start_client.capture(pressed_at=pressed_at)
    with node.lock:
      if node.stop_requested or node.shutdown_received():
        raise ValueError("session Stop is latched")
      if node.phase == "pulsing" or (node.phase == "finished" and not node.record.get("file")):
        raise ValueError("trial is running or finalizing; Start was not queued")
      if (ticket.instance_id, ticket.epoch) != node.gateway_authority:
        raise ValueError("Start status does not match this session's gateway authority")
      return ticket

  if inbox is None:
    inbox = SessionInbox(node.clock, capture_start=capture_start)
    threading.Thread(target=inbox.read, args=(sys.stdin,), daemon=True).start()
  else:
    inbox.capture_start = capture_start
  node.operator_stop_received = inbox.stop_seen
  node.software_start_client = start_client
  token, last_receipt = None, None
  accept_after, next_identity_check = -math.inf, 0.0
  exiting, code, has_run, submitted_start = False, 0, False, False
  guard_only_exit = False
  native_discovery_until = None

  def report_stop(result):
    nonlocal code
    if result is None:
      return
    emit(result)
    if result["uncertain"]:
      code = code or 3
    if node.record is not None:
      node.record["software_stop_result"] = result
      node.record["software_stop_pending"] = False
      if node.record.get("file"):
        Path(node.record["file"]).write_text(json.dumps(node.record, indent=1, allow_nan=False) + "\n")

  def request_exit(message):
    nonlocal token, exiting, guard_only_exit
    command = message["command"]
    reason = message.get("reason", "session " + command)
    token, exiting = None, True
    guard_stop = command == "stop" and node.command_topology != "stock" and (
      guard_only_exit or (node.record and node.record["outcome"] == "interrupted"))
    if command == "stop":
      guard_only_exit = False  # explicit Stop overrides a guard-only shutdown, even during cleanup
    if submitted_start or guard_stop or (node.deadline and command == "stop"):
      node.cancel_start(reason)
    if node.phase == "pulsing":
      with node.lock:
        node.stop(reason, fault=True)
    emit({"event": "stopping", "reason": message.get("reason", command)})

  emit({"event": "session_ready", "profile": node.request.profile,
        "command_topology": node.command_topology,
        "source_instance": node.deadline.source_instance if node.deadline else None,
        "client_identity": dict(CLIENT_IDENTITY), "commands": ["list", "prepare", "start", "run", "stop", "status", "quit"],
        "software_start_interface_available": start_client is not None,
        "software_start_supported": bool(start_client is not None and start_client.stop_capable)})
  try:
    while True:
      spin_once(node, timeout_sec=0.02)
      if node.stop_requested and not exiting:
        if start_client is not None and (start_client.pending is not None or start_client.last_request is not None):
          node.cancel_start(node.stop_requested)
        exiting, token, code = True, None, 3
        emit({"event": "fault", "reason": node.stop_requested})
      item = inbox.get(urgent_only=exiting)  # Stop remains live throughout finalization
      urgent = item is not None and item[1].get("command") in ("stop", "quit")
      if node.command_topology != "stock" and not exiting and not urgent:
        if (node.command_path_checked_at is None or
            node.now() - node.command_path_checked_at >= OWNER_CHECK_S):
          node.check_command_path()
        native_pending = (node.deadline and not node.ever_run and start_client is not None and
            (start_client.pending is not None or (native_discovery_until is not None and
                                                 node.now() < native_discovery_until)))
        problem = node.guard_problem(camera=not native_pending, allow_absent_native=bool(native_pending))
        if problem is None:
          node.guard_ready_once = True
        elif problem[0] == "camera":
          token = None  # an interrupted/prepared action never revives after clearance
        elif node.guard_ready_once or node.guard_fault or (node.deadline and submitted_start):
          token = None
          native_held = (same_manager(runtime) and node.stock.problem() is None and
                         node.native_binding_problem() is None)
          if (problem[0] == "authority" or not native_held or
              (start_client is not None and (start_client.pending is not None or start_client.uncertain))):
            node.cancel_start("command authority lost: " + problem[1])
          else:
            with node.lock:
              node.interrupt_guard(problem[1], fatal=True)
            guard_only_exit = True
          exiting, code = True, 3
          emit({"event": "fault", "reason": problem[1], "guard_only_exit": guard_only_exit,
                "native_deactivation_requested": not guard_only_exit})
      if exiting and not urgent:
        item = inbox.get(urgent_only=True)  # a guard-fault notice may itself prompt Stop
      if item:
        received_at, message = item
        command = message.get("command")
        try:
          if command in ("stop", "quit"):
            request_exit(message)
          elif command == "status":
            emit({"event": "status", "phase": node.phase, "prepared": token is not None,
                  "problem": node.start_problem() if node.phase == "waiting" else None,
                  "software_start_interface_available": start_client is not None,
                  "software_start_supported": bool(start_client is not None and start_client.stop_capable),
                  "software_stop_result": start_client.stop_result if start_client is not None else None,
                  "last_receipt": last_receipt})
          elif command == "list":
            emit({"event": "trials", "axes": sorted(AXES), "defaults": DEFAULT_SPEED,
                  "ceilings": STRICT_SPEED_CEILING if node.deadline else SPEED_PROFILES[node.request.profile],
                  "max_duration_s": MAX_PULSE_S,
                  "lateral_calibration": {"axes": ["left", "right"],
                    "max_duration_s": MAX_LATERAL_CALIBRATION_S,
                    "max_speed": MAX_LATERAL_CALIBRATION_SPEED, "requires_per_prepare_opt_in": True},
                  "heading": {"fields": ["turn_angle_rad", "heading_rad"],
                    "range_rad": [-math.pi, math.pi], "max_speed_rad_s": HEADING_MAX_SPEED,
                    "max_timeout_s": HEADING_MAX_TIMEOUT_S, "reference": "IMU yaw"}})
          elif command == "start":
            if message.get("_start_error"):
              raise ValueError(message["_start_error"])
            if start_client is None or "_start_ticket" not in message:
              raise ValueError("Start requires status captured at input receipt")
            if node.phase == "pulsing" or (node.phase == "finished" and not node.record.get("file")):
              raise ValueError("trial is running or finalizing; Start was not queued")
            if received_at < accept_after:
              raise ValueError("Start predates the last completed trial; send a fresh command")
            if not same_manager(runtime):
              raise ValueError("native manager changed before Start")
            if not node.check_command_path():
              raise ValueError("selected command path is unavailable before Start")
            if node.deadline:
              problem = node.guard_problem(camera=False, allow_absent_native=not node.ever_run)
              if problem:
                raise ValueError(problem[1])
            ticket = message["_start_ticket"]
            if (ticket.instance_id, ticket.epoch) != node.gateway_authority:
              raise ValueError("Start ticket differs from this session's gateway authority")
            # Receipt-time Stop must also win if it arrives during the final
            # process/graph checks, after this Start was dequeued.
            if node.stop_requested or node.shutdown_received():
              raise ValueError("session Stop received before Start dispatch")
            token = None
            request = start_client.send(ticket)
            submitted_start = True
            native_discovery_until = None
            emit({"event": "start_submitted", "request": request})
          elif command in ("prepare", "run"):
            if start_client is not None and (start_client.pending is not None or start_client.uncertain):
              raise ValueError("Start acknowledgement pending or uncertain; no trial queued")
            if exiting or node.phase == "pulsing" or (node.phase == "finished" and not node.record.get("file")):
              raise ValueError("trial is running or finalizing; command was not queued")
            if received_at < accept_after:
              raise ValueError("command predates the last completed trial; send a fresh command")
            if command == "prepare":
              if node.command_topology != "stock":
                problem = node.guard_problem()
                if problem:
                  raise ValueError(problem[1])
              if message.get("profile", node.request.profile) != node.request.profile:
                raise ValueError("session profile cannot change")
              request = session_request(message, node.request.profile)
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
      if start_client is not None:
        result = start_client.poll()
        if result is not None:
          emit(result)
          if node.deadline and result.get("success") is True:
            native_discovery_until = node.now() + NATIVE_DISCOVERY_S
        if start_client.uncertain and not exiting:
          node.cancel_start("Start result uncertain")
          exiting, token, code = True, None, 3
        report_stop(start_client.poll_stop())
      if (has_run and not exiting and (node.phase == "waiting" or
          (node.phase == "finished" and node.record.get("file")))):
        problem = node.stock.problem() or node.native_binding_problem()
        if not node.latest_obs or node.now() - node.latest_obs[1] > OBS_MAX_AGE_S:
          problem = "stock policy observation stale"
        if node.require_gateway and (node.gateway_blocked is not False or
            node.gateway_seen_at is None or node.now() - node.gateway_seen_at > 0.6):
          problem = "e-stop gateway status unavailable or blocked"
        if problem:
          if node.deadline:
            node.cancel_start("session authority interrupted: " + problem)
          emit({"event": "fault", "reason": "session authority interrupted: " + problem})
          return 3
      if not exiting and node.now() >= next_identity_check:
        next_identity_check = node.now() + OWNER_CHECK_S
        if not same_manager(runtime):
          exiting, token, code = True, None, 3
          if start_client is not None and start_client.pending is not None:
            node.cancel_start("native manager changed during Start")
          if node.phase == "pulsing":
            with node.lock:
              node.stop("native manager changed during session", fault=True)
          elif node.command_topology != "stock" and node.record and node.record["outcome"] == "interrupted":
            node.cancel_start("native manager changed during guard cancellation")
          emit({"event": "fault", "reason": "native manager changed during session"})
      if guard_only_exit and not same_manager(runtime):
        guard_only_exit = False
        node.cancel_start("native manager changed during guard cancellation")
      if node.phase == "pulsing" and node.now() - node.started >= node.request.duration_s:
        with node.lock:
          node.expire()
      if node.phase == "finished" and not node.record.get("file"):
        if node.zero_confirmed_at is not None or node.now() - node.zero_sent_at >= 0.5:
          last_receipt = str(finish_trial(node, runtime, same_manager, session=True))
          accept_after = node.now()
          trial_code = result_exit_code(node.record)
          emit({"event": "trial_complete", "file": last_receipt, "outcome": node.record["outcome"],
                "reason": node.record["reason"], "summary": node.record["summary"],
                "command_verification": node.record["command_verification"],
                "interruption_verification": node.record.get("interruption_verification")})
          if (node.record["outcome"] == "interrupted" and
              node.record.get("interruption_verification", {}).get("passed") and not node.guard_fault):
            trial_code = 0  # keep resident after verified cancellation; require fresh prepare/run
          if trial_code:
            exiting, code = True, trial_code  # fault/unconfirmed zero cannot be rearmed
      if exiting and (node.phase == "waiting" or node.record.get("file")):
        # Receipt/fault emission can enqueue Stop after the loop's first read.
        # Drain only shutdown requests; queued Start/prepare/run remain disabled.
        item = inbox.get(urgent_only=True)
        while item is not None:
          request_exit(item[1])
          item = inbox.get(urgent_only=True)
        if start_client is None or not start_client.stop_pending:
          return code
  finally:
    if (start_client is not None and not node.stop_requested and
        (start_client.pending is not None or start_client.uncertain or (submitted_start and not guard_only_exit))):
      node.cancel_start("session exit after Start request")
    if start_client is not None and start_client.stop_pending:
      # Exceptional shutdown still observes a bounded gateway-latch outcome.
      # Native zero/deactivation remains separate and is never synthesized here.
      from foundation.hardware.software_start import STOP_CONFIRM_TIMEOUT_S
      until = time.monotonic() + STOP_CONFIRM_TIMEOUT_S
      while start_client.stop_pending and time.monotonic() < until:
        try:
          spin_once(node, timeout_sec=.02)
        except (Exception, KeyboardInterrupt):
          break  # transport/shutdown cannot be reported as a gateway ACK
        report_stop(start_client.poll_stop())
      report_stop(start_client.poll_stop(force_unconfirmed=True))


def parse_args(argv=None):
  parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                   epilog="Without --execute nothing is imported from ROS and nothing is published.")
  parser.add_argument("axis", nargs="?", choices=sorted(AXES))
  parser.add_argument("--session", action="store_true",
                      help="keep one verified ROS session; accept list/prepare/start/run/stop/status/quit JSON lines on stdin")
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
  from foundation.hardware import runtime_identity
  read_json, verify_loaded, same_manager = runtime_identity.read_json, runtime_identity.verify_loaded, runtime_identity.same_manager
  same_command_path = getattr(runtime_identity, "same_command_path",
                              lambda evidence: evidence.get("command_topology", "stock") == "stock")
  try:
    if args.runtime_receipt is None:
      raise ValueError("--runtime-receipt is required before motion")
    preflight = read_json(args.runtime_receipt)
    runtime = verify_loaded(preflight)
    validate_topology_request(runtime.get("command_topology", "stock"), request,
                              require_gateway=not args.no_gateway)
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
  node = _node_class()(request, args.out, require_gateway=not args.no_gateway,
                      command_topology=runtime.get("command_topology", "stock"),
                      command_path_check=lambda: same_command_path(runtime))
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
          elif not node.check_command_path():
            break
        if node.phase == "pulsing" and time.monotonic() - node.started >= request.duration_s:
          with node.lock:
            node.expire()
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
