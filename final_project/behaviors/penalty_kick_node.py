#!/usr/bin/env python3
"""ROS 2 node around ``penalty_kick``: one penalty kick per activation.

    services   activate_penalty_kick, deactivate_penalty_kick (std_srvs/Trigger)
    publishes  /kick_cmd_vel  geometry_msgs/Twist at rate_hz while active, into a
                              cmd_vel_mux (launch/penalty_kick.launch.py starts one);
                              zero during the strike, one zero at the end
               /cmd_vel       zero only, three times at shutdown
               /<kick_controller>/clip   std_msgs/String, the kick clip request
               /emergency_stop           std_msgs/Empty, when the switch back fails
               /penalty_kick/state       std_msgs/String JSON (penalty_kick.status())
    switches   /controller_manager/switch_controller, STRICT:
               walker -> kick slot before the kick, kick slot -> walker after it
    reads      /soccer/ball (PointStamped, base_link yaw-only frame, capture stamp),
               /soccer/goal (goal_frame topic only), /imu_sensor_broadcaster/imu,
               /<kick_controller>/clip_status, /joint_states (strike watchdog)
    stops on   /emergency_stop, R3 or a face button on /joy, a latched e-stop
               gateway, a second /cmd_vel publisher, a stale IMU

Activation refuses unless exactly the mux (plus this node's shutdown publisher)
publishes /cmd_vel, the IMU is fresh, the gateway is released (require_gateway)
and controller_manager lists the kick slot inactive and the walker active.
Every PenaltyParams field is a ROS parameter. They are read again at each
activation, so `ros2 param set /penalty_kick creep_efficiency 0.75` changes the
next attempt. R1/R2 select the right kick and L1/L2 the left kick for the next
attempt (side "auto" until then).
"""

from __future__ import annotations

import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import fields, replace

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from penalty_kick import PenaltyKick, PenaltyParams, State, check_params  # noqa: E402
from run_through_kick import BallObs  # noqa: E402

ABORT_BUTTONS = [0, 1, 2, 3, 12]  # face buttons and R3 (e-stop) in the course config
RIGHT_BUTTONS, LEFT_BUTTONS = [5, 7], [4, 6]  # R1, R2 / L1, L2 (ds4)
IDLE, CHECKING, RUNNING = "idle", "checking", "running"


def stamp_seconds(header) -> float:
  return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_from_quaternion(q) -> float:
  return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PenaltyKickNode(Node):
  def __init__(self):
    super().__init__("penalty_kick")
    d = self.declare_parameter
    d("require_gateway", True)  # False on the older course stack (no e-stop gateway)
    d("use_imu", True)
    d("ball_topic", "/soccer/ball")
    d("goal_topic", "/soccer/goal")
    d("imu_topic", "/imu_sensor_broadcaster/imu")
    d("cmd_vel_topic", "/kick_cmd_vel")
    d("robot_cmd_vel_topic", "/cmd_vel")
    d("kick_controller", "neural_controller_student")
    d("walker_controller", "neural_controller_default")
    d("rate_hz", 50.0)
    d("imu_max_age", 0.5)
    d("goal_max_age", 1.0)
    d("check_timeout_s", 3.0)
    d("abort_buttons", ABORT_BUTTONS)
    d("start_button", -1)  # a free joystick button that starts an attempt; -1 = service only
    defaults = PenaltyParams()
    for f in fields(PenaltyParams):
      value = getattr(defaults, f.name)
      d(f.name, value if isinstance(value, str) else float(value))
    rate = float(self.param("rate_hz"))
    if not 20.0 <= rate <= 100.0:
      raise ValueError("rate_hz must be in [20, 100]")

    self.lock = threading.Lock()
    self.kick = PenaltyKick(self.read_params())
    self.stage, self.stage_since, self.message = IDLE, 0.0, ""
    self.ball = None  # BallObs, robot frame
    self.goal = None  # (x, y, receive time)
    self.imu_yaw, self.imu_time = None, -math.inf
    self.gateway_blocked = None
    self.last_owner_check = 0.0
    self.buttons: list = []
    self.last_state = None
    self.slot = str(self.param("kick_controller"))
    self.walker = str(self.param("walker_controller"))
    self.cmd_topic = str(self.param("cmd_vel_topic"))
    self.robot_topic = str(self.param("robot_cmd_vel_topic"))

    self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
    self.robot_pub = self.create_publisher(Twist, self.robot_topic, 10)  # shutdown zeros only
    self.clip_pub = self.create_publisher(String, f"/{self.slot}/clip", 10)
    self.estop_pub = self.create_publisher(Empty, "/emergency_stop", 10)
    self.state_pub = self.create_publisher(String, "penalty_kick/state", 10)
    self.create_subscription(PointStamped, self.param("ball_topic"), self.on_ball, 10)
    self.create_subscription(PointStamped, self.param("goal_topic"), self.on_goal, 10)
    self.create_subscription(Imu, self.param("imu_topic"), self.on_imu, 10)
    self.create_subscription(String, f"/{self.slot}/clip_status", self.on_clip_status, 10)
    self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
    self.create_subscription(Joy, "/joy", self.on_joy, 10)
    self.create_subscription(Empty, "/emergency_stop", self.on_emergency, 10)
    self.create_subscription(String, "/estop_controller/stop_status", self.on_stop_status, 10)
    self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
    self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
    self.create_service(Trigger, "activate_penalty_kick", self.on_activate)
    self.create_service(Trigger, "deactivate_penalty_kick", self.on_deactivate)
    self.create_timer(1.0 / rate, self.on_tick)
    self.get_logger().info(f"penalty_kick ready: {self.kick.params}")

  def param(self, name):
    return self.get_parameter(name).value

  def now(self) -> float:
    return self.get_clock().now().nanoseconds * 1e-9

  def read_params(self) -> PenaltyParams:
    values = {}
    for f in fields(PenaltyParams):
      v = self.param(f.name)
      values[f.name] = str(v) if isinstance(getattr(PenaltyParams, f.name), str) else float(v)
    return replace(PenaltyParams(), **values)

  # ------------------------------------------------------------ inputs
  def usable_stamp(self, header):
    stamp = stamp_seconds(header)
    return stamp if stamp > 0.0 and stamp <= self.now() + 0.05 else None

  def on_ball(self, msg) -> None:
    x, y, stamp = float(msg.point.x), float(msg.point.y), self.usable_stamp(msg.header)
    if stamp is not None and math.isfinite(x) and math.isfinite(y):
      with self.lock:
        self.ball = BallObs(x, y, stamp)

  def on_goal(self, msg) -> None:
    x, y = float(msg.point.x), float(msg.point.y)
    if math.isfinite(x) and math.isfinite(y):
      with self.lock:
        self.goal = (x, y, self.now())

  def on_imu(self, msg) -> None:
    yaw = yaw_from_quaternion(msg.orientation)
    if math.isfinite(yaw):
      with self.lock:
        self.imu_yaw, self.imu_time = yaw, self.now()

  def on_clip_status(self, msg) -> None:
    clip_name = msg.data.split(" ")[0] if msg.data else ""
    with self.lock:
      self.kick.on_clip_status(self.now(), clip_name)

  def on_joints(self, msg) -> None:
    if msg.velocity:
      speed = max(abs(v) for v in msg.velocity)
      with self.lock:
        self.kick.on_joint_speed(self.now(), speed)

  def on_joy(self, msg) -> None:
    pressed = [i for i, b in enumerate(msg.buttons)
               if b and (i >= len(self.buttons) or not self.buttons[i])]
    held = [i for i in [int(b) for b in self.param("abort_buttons")]
            if 0 <= i < len(msg.buttons) and msg.buttons[i]]
    self.buttons = list(msg.buttons)
    with self.lock:
      if held and self.stage != IDLE:
        self.stop(f"joystick button {held[0]}", switch=False)  # the operator took over
        return
      if self.stage != IDLE:
        return
      for i in pressed:
        if i in RIGHT_BUTTONS + LEFT_BUTTONS:
          side = "right" if i in RIGHT_BUTTONS else "left"
          self.set_parameters([Parameter("side", value=side)])
          self.get_logger().info(f"next kick: {side}")
    start = int(self.param("start_button"))
    if start >= 0 and start in pressed:
      ok, message = self.activate()
      self.get_logger().warning(f"start button: {message}")

  def on_emergency(self, msg) -> None:
    with self.lock:
      self.stop("emergency stop", switch=False)

  def on_stop_status(self, msg) -> None:
    try:
      status = json.loads(msg.data)
    except ValueError:
      return
    blocked = bool(status.get("latched")) or bool(status.get("fault"))
    with self.lock:
      self.gateway_blocked = blocked
      if blocked:
        self.stop("e-stop gateway latched", switch=False)

  # ------------------------------------------------------------ activation
  def on_activate(self, request, response):
    response.success, response.message = self.activate()
    return response

  def activate(self) -> tuple[bool, str]:
    with self.lock:
      now = self.now()
      if self.stage != IDLE:
        return False, f"busy ({self.stage}, {self.kick.state.value})"
      problem = ""
      if bool(self.param("use_imu")) and now - self.imu_time > float(self.param("imu_max_age")):
        problem = "no recent IMU yaw"
      elif bool(self.param("require_gateway")) and self.gateway_blocked is not False:
        problem = ("e-stop gateway latched or faulted (press Options so the walker stands)"
                   if self.gateway_blocked else "no /estop_controller/stop_status yet")
      else:
        others = self.count_publishers(self.robot_topic) - 1  # minus our shutdown publisher
        if others != 1:
          problem = (f"{others} publisher(s) on {self.robot_topic} besides this node; need exactly the "
                     "cmd_vel_mux (course stack with teleop:=False plus the penalty_kick launch)")
      if not problem and not (self.list_client.service_is_ready() and self.switch_client.service_is_ready()):
        problem = "controller_manager services unavailable"
      if not problem:
        try:
          params = self.read_params()
          check_params(params)
          self.kick = PenaltyKick(params)
        except ValueError as exc:
          problem = str(exc)
      if problem:
        self.message = f"not starting: {problem}"
        self.publish_state(force=True)
        return False, self.message
      self.set_stage(CHECKING, "checking controllers")
      self.list_client.call_async(ListControllers.Request()).add_done_callback(self.on_list)
    return True, "starting: checking controllers"

  def on_list(self, future) -> None:
    with self.lock:
      if self.stage != CHECKING:
        return
      res = future.result()
      if res is None:
        return self.set_stage(IDLE, "not starting: controller list failed")
      states = {c.name: c.state for c in res.controller}
      if self.kick.params.strike != "none" and states.get(self.slot) != "inactive":
        return self.set_stage(IDLE, f"not starting: {self.slot} is {states.get(self.slot, 'not loaded')} "
                                    "(install the leg kick policy in that slot)")
      if states.get(self.walker) != "active":
        return self.set_stage(IDLE, f"not starting: {self.walker} must be active (push X)")
      others = sorted(n for n, s in states.items()
                      if s == "active" and n.startswith("neural_controller") and n != self.walker)
      if others:
        return self.set_stage(IDLE, f"not starting: other active controllers {others}")
      self.kick.start(self.now())
      self.set_stage(RUNNING, "running")
      self.get_logger().warning(f"penalty kick started: {self.kick.params}")

  def on_deactivate(self, request, response):
    with self.lock:
      was = self.stage
      self.stop("stopped by service", switch=True)
    response.success = True
    response.message = f"stopping ({was})" if was != IDLE else "was not running"
    return response

  def stop(self, reason: str, switch: bool) -> None:
    """Caller holds the lock."""
    if self.stage == CHECKING:
      self.set_stage(IDLE, f"cancelled: {reason}")
    elif self.stage == RUNNING:
      self.kick.stop(self.now(), reason, switch)
      self.cmd_pub.publish(Twist())

  def set_stage(self, stage: str, message: str) -> None:
    self.stage, self.stage_since, self.message = stage, self.now(), message
    self.publish_state(force=True)

  # ------------------------------------------------------------ actions
  def run_actions(self) -> None:
    for action in self.kick.pop_actions():
      if action[0] == "switch":
        self.request_switch(action[1])
      elif action[0] == "clip":
        self.clip_pub.publish(String(data=action[1]))
        self.get_logger().warning(f"requested clip {action[1]}")
      elif action[0] == "estop":
        self.estop_pub.publish(Empty())
        self.get_logger().error("could not switch back to the walker: emergency stop sent")

  def request_switch(self, which: str) -> None:
    req = SwitchController.Request()
    if which == "in":
      req.activate_controllers, req.deactivate_controllers = [self.slot], [self.walker]
    else:
      req.activate_controllers, req.deactivate_controllers = [self.walker], [self.slot]
    req.strictness = SwitchController.Request.STRICT
    if not self.switch_client.service_is_ready():
      self.kick.on_switch(self.now(), which, False)
      return
    self.switch_client.call_async(req).add_done_callback(
      lambda future, which=which: self.on_switched(which, future))

  def on_switched(self, which: str, future) -> None:
    res = future.result()
    ok = res is not None and bool(res.ok)
    with self.lock:
      self.get_logger().info(f"switch {which}: {'ok' if ok else 'refused'}")
      self.kick.on_switch(self.now(), which, ok)
      self.run_actions()

  # ------------------------------------------------------------ tick
  def on_tick(self) -> None:
    with self.lock:
      now = self.now()
      if self.stage == CHECKING and now - self.stage_since > float(self.param("check_timeout_s")):
        self.set_stage(IDLE, "not starting: no answer from controller_manager")
      if self.stage != RUNNING:
        return
      if now - self.last_owner_check >= 1.0:
        self.last_owner_check = now
        if self.count_publishers(self.robot_topic) != 2:  # the mux and our shutdown publisher
          self.kick.stop(now, "another node publishes /cmd_vel", switch=True)
      if bool(self.param("use_imu")) and now - self.imu_time > float(self.param("imu_max_age")):
        self.kick.stop(now, "IMU lost", switch=True)
      goal = None
      if self.goal is not None and now - self.goal[2] <= float(self.param("goal_max_age")):
        goal = self.goal[:2]
      yaw = self.imu_yaw if bool(self.param("use_imu")) else None
      cmd = self.kick.step(now, self.ball, yaw, goal)
      self.run_actions()
      twist = Twist()
      twist.linear.x, twist.linear.y, twist.angular.z = cmd.vx, cmd.vy, cmd.wz
      self.cmd_pub.publish(twist)  # the final zero of a finished attempt too
      if not self.kick.active:
        self.stage = IDLE
        summary = json.dumps(self.kick.status())
        self.message = "finished"
        self.get_logger().warning(f"penalty kick {self.kick.state.value}: {summary}")
      self.publish_state()

  def publish_state(self, force: bool = False) -> None:
    status = self.kick.status()
    status.update(node=self.stage, message=self.message)
    key = (self.stage, status["state"], status["phase"], status["reason"], self.message)
    if key == self.last_state and not force:
      return
    self.last_state = key
    self.state_pub.publish(String(data=json.dumps(status)))
    if not force:
      self.get_logger().info(json.dumps(status))


def main():
  rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

  def interrupt(signum, frame):
    raise KeyboardInterrupt

  previous = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
  node = PenaltyKickNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    for s in (signal.SIGINT, signal.SIGTERM):
      signal.signal(s, signal.SIG_IGN)  # a second Ctrl-C must not skip the cleanup
    # The switch back needs the executor: if the kick slot may be active, damp
    # every controller instead of leaving the trick policy running.
    if node.kick.state == State.STRIKE and node.kick.strike.slot_may_be_active:
      node.estop_pub.publish(Empty())
    for _ in range(3):
      node.cmd_pub.publish(Twist())
      node.robot_pub.publish(Twist())
      time.sleep(0.05)
    node.destroy_node()
    rclpy.try_shutdown()
    for signum, handler in previous.items():
      signal.signal(signum, handler)


if __name__ == "__main__":
  main()
