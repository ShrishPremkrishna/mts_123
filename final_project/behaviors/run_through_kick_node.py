#!/usr/bin/env python3
"""ROS 2 service around ``run_through_kick``: one run-through kick per activation.

Same shape as follow_me_pupper/robot/follow_service.py:

    services   activate_run_through_kick, deactivate_run_through_kick (std_srvs/Trigger)
    publishes  /kick_cmd_vel geometry_msgs/Twist, 20 Hz while active, into
                             cmd_vel_mux (the mux sends zero when this node goes
                             quiet); one zero Twist when the kick ends
               /cmd_vel      zero only, three times at shutdown (a killed mux
                             sends nothing, and the walker keeps its last command)
               /run_through_kick/state  std_msgs/String  JSON {state, reason}
    stops on   /emergency_stop (std_msgs/Empty), a face button or R3 on /joy, a
               latched e-stop gateway, or a second /cmd_vel publisher; loss of
               stock authority during a shot requests /emergency_stop and aborts

Activation refuses unless exactly one other node (the mux) publishes /cmd_vel
(joystick teleop or a missing mux would make the kick unsafe) and the e-stop
gateway is released. Ball messages with a 0 or future stamp, or non-finite
values, are dropped. KickParams are re-read at each activation, so
`ros2 param set` changes the next kick.

Ball input, parameter ``input_mode``:
    manual  no perception. Before each activation set the measured ball centre
            relative to base_link (manual_ball_x/y [m], x forward, y left) and
            optionally manual_aim_deg (NaN = straight through the ball):
              ros2 param set /run_through_kick manual_ball_x 1.0
    robot   /soccer/ball and /soccer/goal (geometry_msgs/PointStamped) in the
            base_link yaw-only frame; header.stamp = image capture time
    field   /soccer/ball and /soccer/goal in the field frame, plus the robot
            pose /soccer/robot_pose (geometry_msgs/PoseStamped, field frame)

Every KickParams field is also a ROS parameter (for example charge_speed).
This node currently admits only the pinned stock walk profile, at most 0.49 m/s.

Walker guard (offline-tested, first physical baseline = stock walker at 0.45 m/s):
the node polls /controller_manager/list_controllers and
/neural_controller_default/get_parameters in the background (one bounded request
per service, POLL_S apart, replies older than REPLY_TIMEOUT_S or from a superseded
request are dropped, evidence older than FRESH_S is stale). Activation refuses, and
a running kick aborts with a zero command, unless the fresh evidence shows
  * exactly one active leg owner (broadcasters tolerated): neural_controller_default
    of the pinned controller type, so a target-conditioned student, a gemini_walk
    or any custom controller is refused;
  * game_buttons false, speed_cap/max_vx_forward/max_vx_backward finite and >= 0
    (0 disables an extra cap, > 0 narrows the envelope below the 0.49 walk cap);
  * model_path naming an existing regular file whose SHA-256 is the pinned stock
    actor. ROS parameters and the file hash describe configured values, NOT
    proof of the weights or parameters snapshotted by the running controller.
    A fresh session started from the verified configuration, unchanged for the
    session, is required; live native parameter changes are not preparation.
Requested KickParams must fit the envelope (charge_speed, align_speed, max_vy,
max_wz); an unsupported request is refused, never clamped. Every non-zero command
is checked against the envelope before publishing, including backward alignment
against max_vx_backward; a command outside it aborts the kick instead of letting
the walker clamp it silently (which would break dead reckoning). Guard timing uses
time.monotonic(); completed futures are polled by the timer, never used to start.
Loss of stock authority during a shot requests the shared emergency stop: zero
alone cannot stop a target-conditioned policy. This is a request, not proof of
physical stopping. Receiver-side expiry and fresh-session pinning remain needed.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import signal
import stat
import sys
import threading
import time
from dataclasses import fields, replace
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Joy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_through_kick import (  # noqa: E402
  BallObs,
  KickParams,
  RunThroughKick,
  State,
  aim_from_goal,
  field_to_robot,
)

INPUT_MODES = ("manual", "robot", "field")
STOP_BUTTONS = (0, 1, 2, 3, 12)  # the four face buttons and R3 (e-stop) in the course config
MANUAL_X = (0.9, 1.5)  # m: no reversing toward the operator, short dead reckoning
MANUAL_Y = 0.5  # m
STOCK_CONTROLLER = "neural_controller_default"
STOCK_POLICY_SHA256 = "ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712"
POLL_S, REPLY_TIMEOUT_S, FRESH_S = 0.2, 0.4, 0.6
PARAM_NAMES = ("model_path", "max_vx_forward", "max_vx_backward", "speed_cap", "game_buttons")


class StockWalkerEvidence:
  """Bounded read-only polling; no callback can start motion or rearm a stop."""

  def __init__(self, node):
    self.clock = time.monotonic
    self.clients = (node.create_client(ListControllers, "/controller_manager/list_controllers"),
                    node.create_client(GetParameters, f"/{STOCK_CONTROLLER}/get_parameters"))
    self.pending = None
    self.last_poll = -math.inf
    self.checked_at = -math.inf
    self.bounds = None
    self.error = "stock controller evidence not ready"
    self.model_cache = None

  def invalidate(self, reason):
    pending, self.pending = self.pending, None
    if pending:
      for client, future in zip(self.clients, pending[1]):
        client.remove_pending_request(future)
        future.cancel()
    self.bounds, self.checked_at, self.error = None, -math.inf, reason

  def _model_matches(self, filename):
    path = Path(filename).resolve(strict=True)
    before = path.stat()
    if not stat.S_ISREG(before.st_mode) or not 0 < before.st_size <= 32 * 1024 * 1024:
      raise ValueError("stock policy is not a bounded regular file")
    def identity(s):
      return (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
    key = (str(path), identity(before))
    if not self.model_cache or self.model_cache[0] != key:
      digest = hashlib.sha256()
      with path.open("rb") as source:
        if identity(os.fstat(source.fileno())) != identity(before):
          raise ValueError("stock policy changed while opening")
        remaining = before.st_size
        while remaining:
          chunk = source.read(min(65536, remaining))
          if not chunk:
            raise ValueError("stock policy changed while reading")
          digest.update(chunk)
          remaining -= len(chunk)
        if source.read(1) or identity(os.fstat(source.fileno())) != identity(before):
          raise ValueError("stock policy changed while reading")
      if identity(path.stat()) != identity(before) or Path(filename).resolve(strict=True) != path:
        raise ValueError("stock policy changed while reading")
      self.model_cache = (key, digest.hexdigest())
    if self.model_cache[1] != STOCK_POLICY_SHA256:
      raise ValueError("configured policy does not match the pinned stock actor")

  def _decode(self, inventory, parameters):
    active = []
    seen = set()
    for c in inventory.controller:
      if not c.name or c.name in seen:
        raise ValueError("malformed controller inventory")
      seen.add(c.name)
      if c.state != "active":
        continue
      # Read-only broadcasters are the only other active controllers admitted.
      if (c.type in ("joint_state_broadcaster/JointStateBroadcaster",
                     "imu_sensor_broadcaster/IMUSensorBroadcaster") and not c.claimed_interfaces):
        continue
      active.append((c.name, c.type))
    if active != [(STOCK_CONTROLLER, "neural_controller/NeuralController")]:
      raise ValueError("expected only the stock walker active; controller ownership changed or missing")
    values = parameters.values
    if len(values) != len(PARAM_NAMES):
      raise ValueError("stock cap/model parameters missing")
    if values[0].type != ParameterType.PARAMETER_STRING or not values[0].string_value:
      raise ValueError("stock model_path missing")
    caps = []
    for value in values[1:4]:
      if value.type != ParameterType.PARAMETER_DOUBLE or not math.isfinite(value.double_value) or value.double_value < 0:
        raise ValueError("stock cap parameter missing or invalid")
      caps.append(value.double_value or math.inf)  # native zero disables the extra cap
    if values[4].type != ParameterType.PARAMETER_BOOL or values[4].bool_value:
      raise ValueError("stock baseline requires game_buttons=false")
    self._model_matches(values[0].string_value)
    forward, backward, speed = caps
    return (min(0.49, forward, speed), min(0.49, backward, speed), min(0.49, speed), 2.0)

  def poll(self):
    now = self.clock()
    if self.pending:
      started, futures = self.pending
      if now - started > REPLY_TIMEOUT_S:
        self.invalidate("stock controller evidence request timed out")
      elif all(f.done() for f in futures):
        self.pending = None
        try:
          bounds = self._decode(*(f.result() for f in futures))
          if self.clock() - started > REPLY_TIMEOUT_S:
            raise ValueError("stock controller evidence arrived too late")
          self.bounds, self.checked_at, self.error = bounds, started, ""
        except Exception as exc:
          self.invalidate("stock controller evidence refused: " + str(exc))
    if self.pending is None and now - self.last_poll >= POLL_S:
      self.last_poll = now
      if not all(c.service_is_ready() for c in self.clients):
        self.invalidate("stock controller status/parameter service unavailable")
        return
      futures = []
      try:
        request = GetParameters.Request()
        request.names = list(PARAM_NAMES)
        for client, req in zip(self.clients, (ListControllers.Request(), request)):
          futures.append(client.call_async(req))
        self.pending = (now, futures)
      except Exception:
        self.pending = (now, futures)
        self.invalidate("stock controller evidence request failed")

  def problem(self, params=None, command=None):
    if self.bounds is None:
      return self.error
    if not 0 <= self.clock() - self.checked_at <= FRESH_S:
      return "stock controller evidence stale"
    forward, backward, lateral, yaw = self.bounds
    if params:
      for name, cap in (("charge_speed", forward), ("align_speed", min(forward, backward)),
                        ("max_vy", lateral), ("max_wz", yaw)):
        value = getattr(params, name)
        if not math.isfinite(value) or not 0 < value <= cap:
          return f"{name}={value} exceeds supported stock bounds (0, {cap}]"
    if command and not (-backward <= command.vx <= forward and abs(command.vy) <= lateral
                        and abs(command.wz) <= yaw):
      return "command exceeds verified stock bounds; refusing silent clipping"
    return None


def stamp_seconds(header) -> float:
  return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_from_quaternion(q) -> float:
  return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RunThroughKickNode(Node):
  def __init__(self):
    super().__init__("run_through_kick")
    # False on older course stacks with no e-stop gateway (no /estop_controller/stop_status):
    # then the gateway check is skipped and switches go to controller_manager directly.
    self.declare_parameter("require_gateway", True)
    self.declare_parameter("input_mode", "manual")
    self.declare_parameter("cmd_vel_topic", "/kick_cmd_vel")
    self.declare_parameter("robot_cmd_vel_topic", "/cmd_vel")  # what the walker reads
    self.declare_parameter("ball_topic", "/soccer/ball")
    self.declare_parameter("goal_topic", "/soccer/goal")
    self.declare_parameter("robot_pose_topic", "/soccer/robot_pose")
    self.declare_parameter("rate_hz", 20.0)  # cmd_vel_mux drops a source quiet for 0.5 s
    self.declare_parameter("goal_max_age", 1.0)  # s; older goal -> kick straight
    self.declare_parameter("manual_ball_x", float("nan"))
    self.declare_parameter("manual_ball_y", 0.0)
    self.declare_parameter("manual_aim_deg", float("nan"))
    defaults = replace(KickParams(), charge_speed=0.45, align_speed=0.3)
    for f in fields(KickParams):
      self.declare_parameter(f.name, float(getattr(defaults, f.name)))
    params = replace(
      defaults, **{f.name: float(self.get_parameter(f.name).value) for f in fields(KickParams)}
    )
    self.mode = str(self.get_parameter("input_mode").value)
    if self.mode not in INPUT_MODES:
      raise ValueError(f"input_mode must be one of {INPUT_MODES}")
    rate = float(self.get_parameter("rate_hz").value)
    if not 5.0 <= rate <= 100.0:
      raise ValueError("rate_hz must be in [5, 100]")

    self.lock = threading.Lock()
    self.kick = RunThroughKick(params)
    self.gateway_blocked = None  # None until /estop_controller/stop_status arrives
    self.last_owner_check = 0.0
    self.ball = None  # latest ball message: (x, y, stamp), frame per input_mode
    self.goal = None  # (x, y, receive time)
    self.pose = None  # (x, y, yaw, stamp), field mode only
    self.last_state = None
    self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
    self.robot_topic = str(self.get_parameter("robot_cmd_vel_topic").value)
    self.robot_pub = self.create_publisher(Twist, self.robot_topic, 10)  # shutdown zeros only
    self.estop_pub = self.create_publisher(Empty, "/emergency_stop", 10)
    self.state_pub = self.create_publisher(String, "run_through_kick/state", 10)
    self.stock = StockWalkerEvidence(self)
    if self.mode != "manual":
      self.create_subscription(PointStamped, self.get_parameter("ball_topic").value, self.on_ball, 10)
      self.create_subscription(PointStamped, self.get_parameter("goal_topic").value, self.on_goal, 10)
    if self.mode == "field":
      self.create_subscription(
        PoseStamped, self.get_parameter("robot_pose_topic").value, self.on_pose, 10
      )
    self.create_subscription(Empty, "/emergency_stop", self.on_emergency, 10)
    self.create_subscription(Joy, "/joy", self.on_joy, 10)
    self.create_subscription(String, "/estop_controller/stop_status", self.on_stop_status, 10)
    self.create_service(Trigger, "activate_run_through_kick", self.on_activate)
    self.create_service(Trigger, "deactivate_run_through_kick", self.on_deactivate)
    self.create_timer(1.0 / rate, self.on_tick)
    self.get_logger().info(f"run_through_kick ready, input_mode={self.mode}: {params}")

  def now(self) -> float:
    return self.get_clock().now().nanoseconds * 1e-9

  # ------------------------------------------------------------ inputs
  def usable_stamp(self, header):
    """Capture time, or None if it is 0 or more than 50 ms in the future (clock fault)."""
    stamp = stamp_seconds(header)
    return stamp if stamp > 0.0 and stamp <= self.now() + 0.05 else None

  def on_ball(self, msg: PointStamped) -> None:
    x, y, stamp = float(msg.point.x), float(msg.point.y), self.usable_stamp(msg.header)
    if stamp is not None and math.isfinite(x) and math.isfinite(y):
      with self.lock:
        self.ball = (x, y, stamp)

  def on_goal(self, msg: PointStamped) -> None:
    x, y = float(msg.point.x), float(msg.point.y)
    if math.isfinite(x) and math.isfinite(y):
      with self.lock:
        self.goal = (x, y, self.now())

  def on_pose(self, msg: PoseStamped) -> None:
    p, stamp = msg.pose.position, self.usable_stamp(msg.header)
    yaw = yaw_from_quaternion(msg.pose.orientation)
    if stamp is not None and all(math.isfinite(v) for v in (p.x, p.y, yaw)):
      with self.lock:
        self.pose = (float(p.x), float(p.y), yaw, stamp)

  def robot_frame_inputs(self, now: float) -> tuple[BallObs | None, float | None]:
    """Latest ball observation and aim, both in the robot's yaw-only frame."""
    if self.ball is None:
      return None, None
    bx, by, stamp = self.ball
    goal = self.goal
    if goal is not None and now - goal[2] > float(self.get_parameter("goal_max_age").value):
      goal = None
    if self.mode == "field":
      if self.pose is None:
        return None, None
      rx, ry, ryaw, pose_stamp = self.pose
      bx, by = field_to_robot(rx, ry, ryaw, bx, by)
      stamp = min(stamp, pose_stamp)  # as old as the older of the two
      if goal is not None:
        gx, gy = field_to_robot(rx, ry, ryaw, goal[0], goal[1])
        goal = (gx, gy, goal[2])
    aim = None if goal is None else aim_from_goal(bx, by, goal[0], goal[1])
    return BallObs(bx, by, stamp), aim

  def stop_kick(self, reason: str, *, fault=False) -> None:
    if self.kick.active:
      if fault:
        self.estop_pub.publish(Empty())
      self.kick.stop(self.now(), reason)
      self.cmd_pub.publish(Twist())
    self.stock.invalidate(reason)

  def on_emergency(self, msg) -> None:
    with self.lock:
      self.stop_kick("emergency stop")

  def on_joy(self, msg) -> None:
    pressed = [i for i in STOP_BUTTONS if 0 <= i < len(msg.buttons) and msg.buttons[i]]
    if pressed:
      with self.lock:
        self.stop_kick(f"joystick button {pressed[0]}", fault=True)

  def on_stop_status(self, msg) -> None:
    try:
      status = json.loads(msg.data)
    except ValueError:
      return
    blocked = bool(status.get("latched")) or bool(status.get("fault"))
    with self.lock:
      self.gateway_blocked = blocked
      if blocked:
        self.stop_kick("e-stop gateway latched")

  # ------------------------------------------------------------ services
  def on_activate(self, request, response):
    with self.lock:
      if self.kick.active:
        response.success, response.message = True, "already kicking"
        return response
      if bool(self.get_parameter("require_gateway").value) and self.gateway_blocked is not False:
        response.success = False
        response.message = ("e-stop gateway latched or faulted (press Options so the walker stands)"
                            if self.gateway_blocked else "no /estop_controller/stop_status yet")
        return response
      others = self.count_publishers(self.robot_topic) - 1  # minus our shutdown publisher
      if others != 1:
        response.success = False
        response.message = (f"{others} publisher(s) on {self.robot_topic} besides this node; need exactly "
                            "the cmd_vel_mux (course stack with teleop:=False plus the kick_trial launch)")
        return response
      # Parameters set since the last kick (for example speed_efficiency) apply now.
      self.kick.params = replace(
        self.kick.params,
        **{f.name: float(self.get_parameter(f.name).value) for f in fields(KickParams)},
      )
      problem = self.stock.problem(self.kick.params)
      if problem:
        response.success, response.message = False, problem
        return response
      if self.mode == "manual":
        x = float(self.get_parameter("manual_ball_x").value)
        y = float(self.get_parameter("manual_ball_y").value)
        aim_deg = float(self.get_parameter("manual_aim_deg").value)
        if not (math.isfinite(x) and math.isfinite(y)) or not (
          MANUAL_X[0] <= x <= MANUAL_X[1] and abs(y) <= MANUAL_Y
        ):
          response.success = False
          response.message = (f"set manual_ball_x ({MANUAL_X[0]} to {MANUAL_X[1]} m) and "
                              f"manual_ball_y (|y| <= {MANUAL_Y} m) first")
          return response
        aim = None if math.isnan(aim_deg) else math.radians(aim_deg)
        self.kick.start(self.now(), static_ball=(x, y), static_aim=aim)
        # Consume the measurement: the next kick needs a new one.
        self.set_parameters([
          Parameter("manual_ball_x", value=float("nan")),
          Parameter("manual_ball_y", value=0.0),
          Parameter("manual_aim_deg", value=float("nan")),
        ])
      else:
        self.kick.start(self.now())
    response.success, response.message = True, f"run-through kick started ({self.mode})"
    self.get_logger().warning(response.message)
    return response

  def on_deactivate(self, request, response):
    with self.lock:
      was_active = self.kick.active
      self.stop_kick("operator stopped the kick; fresh evidence required",
                     fault=self.stock.problem() is not None)
    response.success = True
    response.message = "stopped" if was_active else "was not kicking"
    return response

  # ------------------------------------------------------------ tick
  def on_tick(self) -> None:
    with self.lock:
      now = self.now()
      self.stock.poll()
      if self.kick.active:
        problem = self.stock.problem(self.kick.params)
        if problem:
          self.stop_kick(problem, fault=True)
      if self.kick.active and now - self.last_owner_check >= 1.0:
        self.last_owner_check = now
        if self.count_publishers(self.robot_topic) != 2:  # the mux and our shutdown publisher
          self.stop_kick("another node publishes /cmd_vel", fault=True)
      if self.kick.active:
        ball, aim = (None, None) if self.mode == "manual" else self.robot_frame_inputs(now)
        cmd = self.kick.step(now, ball, aim)
        problem = self.stock.problem(command=cmd)
        if problem:
          self.stop_kick(problem, fault=True)
          self.publish_state()
          return
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = cmd.vx, cmd.vy, cmd.wz
        self.cmd_pub.publish(twist)  # a finished kick publishes its final zero here
      self.publish_state()

  def publish_state(self) -> None:
    state = (self.kick.state.value, self.kick.reason)
    if state == self.last_state:
      return
    self.last_state = state
    msg = String()
    msg.data = json.dumps({"state": state[0], "reason": state[1]})
    self.state_pub.publish(msg)
    if self.kick.state in (State.DONE, State.ABORTED):
      self.get_logger().warning(f"kick {state[0]} {state[1]}".strip())


def main():
  rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

  def interrupt(signum, frame):
    raise KeyboardInterrupt

  previous = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
  node = RunThroughKickNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    for s in (signal.SIGINT, signal.SIGTERM):
      signal.signal(s, signal.SIG_IGN)  # a second Ctrl-C must not skip the zeros
    # A target-conditioned owner interprets native zero as a target, not a stop.
    # Shutdown may request a fault stop for our active shot; it never rearms.
    owned_active = node.kick.active
    if owned_active:
      node.estop_pub.publish(Empty())
    for _ in range(3):
      stock_known = node.stock.problem() is None
      if owned_active or stock_known:
        node.cmd_pub.publish(Twist())
      if stock_known:
        node.robot_pub.publish(Twist())
      time.sleep(0.05)
    node.destroy_node()
    rclpy.try_shutdown()
    for signum, handler in previous.items():
      signal.signal(signum, handler)


if __name__ == "__main__":
  main()
