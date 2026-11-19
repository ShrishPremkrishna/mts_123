#!/usr/bin/env python3
"""ROS 2 service that runs one learned header kick (see learned_kick.py).

    services   activate_learned_kick, deactivate_learned_kick (std_srvs/Trigger)
    publishes  /cmd_vel  geometry_msgs/Twist at 50 Hz: zero while switching, then
                         the policy's command slot (ball x, ball y, aim), NOT a velocity
               /learned_kick/state  std_msgs/String  JSON {stage, phase, reason}
    switches   through /controller_manager/switch_controller (the e-stop gateway)
    aborts on  /emergency_stop (std_msgs/Empty) or a controller face button on /joy

Sequence (each step must succeed, or the attempt ends with zero commands):
  1. activate: fresh ball inside the trained window, IMU yaw (robot mode), and no
     other /cmd_vel publisher (joystick teleop must be off).
  2. list_controllers: the kick controller is loaded and inactive, the walker is active.
  3. switch: kick controller in, every active neural_controller out (STRICT).
     Zero is published until the gateway confirms. A controller only reads
     /cmd_vel after its activation, so no target ever reaches the walker.
  4. run: targets at 50 Hz until the bridge finishes.
  5. exit: keep sending the newest target (a state the policy trained on) until
     the gateway confirms the walker is back, then zero. Each try waits
     exit_timeout_s; after exit_retries the node publishes /emergency_stop
     (every neural controller then damps).
Also: refuses to start while the e-stop gateway is latched or faulted
(/estop_controller/stop_status); drops non-finite messages and stamps that are 0
or in the future; aborts if the IMU goes quiet or another node starts publishing
/cmd_vel; square (the kick controller's own button) ends the attempt normally.

Ball input, parameter ``input_mode``:
    robot  /soccer/ball, /soccer/goal (PointStamped, base_link yaw-only frame) and
           the IMU (/imu_sensor_broadcaster/imu) for the yaw change since start
    field  /soccer/ball, /soccer/goal and /soccer/robot_pose in the field frame
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
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from learned_kick import BridgeParams, LearnedKickBridge, Phase, Target, entry_check  # noqa: E402
from run_through_kick import aim_from_goal, field_to_robot  # noqa: E402

# Joystick buttons that end an attempt: the four controller face buttons and the
# e-stop (right stick press) in the course estop_controller config.
DEFAULT_ABORT_BUTTONS = [0, 1, 2, 3, 12]
IDLE, CHECKING, SWITCHING_IN, RUNNING, EXITING, ZERO_HOLD = (
  "idle", "checking", "switching_in", "running", "exiting", "zero_hold")
ZERO_STAGES = (CHECKING, SWITCHING_IN, ZERO_HOLD)


def stamp_seconds(header) -> float:
  return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_from_quaternion(q) -> float:
  return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LearnedKickNode(Node):
  def __init__(self):
    super().__init__("learned_kick")
    # False on older course stacks with no e-stop gateway (no /estop_controller/stop_status):
    # then the gateway check is skipped and switches go to controller_manager directly.
    self.declare_parameter("require_gateway", True)
    self.declare_parameter("input_mode", "robot")
    self.declare_parameter("cmd_vel_topic", "/cmd_vel")
    self.declare_parameter("ball_topic", "/soccer/ball")
    self.declare_parameter("goal_topic", "/soccer/goal")
    self.declare_parameter("robot_pose_topic", "/soccer/robot_pose")
    self.declare_parameter("imu_topic", "/imu_sensor_broadcaster/imu")
    self.declare_parameter("rate_hz", 50.0)  # the policy runs at 50 Hz
    self.declare_parameter("goal_max_age", 1.0)
    self.declare_parameter("kick_controller", "neural_controller_student")
    self.declare_parameter("exit_controller", "neural_controller_default")
    self.declare_parameter("zero_hold_s", 1.0)
    self.declare_parameter("switch_timeout_s", 3.0)  # list + switch in
    self.declare_parameter("exit_timeout_s", 1.0)  # each try of the switch back
    self.declare_parameter("exit_retries", 2)
    self.declare_parameter("imu_max_age", 0.5)
    self.declare_parameter("abort_buttons", DEFAULT_ABORT_BUTTONS)
    self.declare_parameter("kick_button", 3)  # square: ends the attempt with the switch back
    defaults = BridgeParams()
    scalar = [f for f in fields(BridgeParams) if isinstance(getattr(defaults, f.name), float)]
    for f in scalar:
      self.declare_parameter(f.name, float(getattr(defaults, f.name)))
    params = replace(defaults, **{f.name: float(self.get_parameter(f.name).value) for f in scalar})
    self.mode = str(self.get_parameter("input_mode").value)
    if self.mode not in ("robot", "field"):
      raise ValueError("input_mode must be robot or field (the learned kick needs live ball updates)")
    rate = float(self.get_parameter("rate_hz").value)
    if not 20.0 <= rate <= 100.0:
      raise ValueError("rate_hz must be in [20, 100]")
    self.abort_buttons = [int(b) for b in self.get_parameter("abort_buttons").value]
    self.cmd_topic = str(self.get_parameter("cmd_vel_topic").value)

    self.lock = threading.Lock()
    self.bridge = LearnedKickBridge(params)
    self.stage, self.reason, self.stage_since = IDLE, "", 0.0
    self.cancel = False
    self.exit_tries = 0
    self.zero_until = 0.0
    self.ball = None  # (x, y, stamp), frame per input_mode
    self.goal = None  # (x, y, receive time)
    self.pose = None  # (x, y, yaw, stamp), field mode
    self.imu_yaw = None  # robot mode
    self.imu_time = 0.0
    self.gateway_blocked = None  # None until /estop_controller/stop_status arrives
    self.last_owner_check = 0.0
    self.yaw0 = None
    self.last_state = None
    self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
    self.state_pub = self.create_publisher(String, "learned_kick/state", 10)
    self.estop_pub = self.create_publisher(Empty, "/emergency_stop", 10)
    self.create_subscription(PointStamped, self.get_parameter("ball_topic").value, self.on_ball, 10)
    self.create_subscription(PointStamped, self.get_parameter("goal_topic").value, self.on_goal, 10)
    if self.mode == "field":
      self.create_subscription(
        PoseStamped, self.get_parameter("robot_pose_topic").value, self.on_pose, 10
      )
    else:
      self.create_subscription(Imu, self.get_parameter("imu_topic").value, self.on_imu, 10)
    self.create_subscription(Empty, "/emergency_stop", self.on_emergency, 10)
    self.create_subscription(Joy, "/joy", self.on_joy, 10)
    self.create_subscription(String, "/estop_controller/stop_status", self.on_stop_status, 10)
    self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
    self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
    self.create_service(Trigger, "activate_learned_kick", self.on_activate)
    self.create_service(Trigger, "deactivate_learned_kick", self.on_deactivate)
    self.create_timer(1.0 / rate, self.on_tick)
    self.get_logger().info(f"learned_kick ready, input_mode={self.mode}: {params}")

  def now(self) -> float:
    return self.get_clock().now().nanoseconds * 1e-9

  def param(self, name):
    return self.get_parameter(name).value

  # ------------------------------------------------------------ inputs
  def usable_stamp(self, header):
    """Capture time, or None if it is 0 or more than 50 ms in the future (clock fault)."""
    stamp = stamp_seconds(header)
    return stamp if stamp > 0.0 and stamp <= self.now() + 0.05 else None

  def on_ball(self, msg):
    x, y, stamp = float(msg.point.x), float(msg.point.y), self.usable_stamp(msg.header)
    if stamp is not None and math.isfinite(x) and math.isfinite(y):
      with self.lock:
        self.ball = (x, y, stamp)

  def on_goal(self, msg):
    x, y = float(msg.point.x), float(msg.point.y)
    if math.isfinite(x) and math.isfinite(y):
      with self.lock:
        self.goal = (x, y, self.now())

  def on_pose(self, msg):
    p, stamp = msg.pose.position, self.usable_stamp(msg.header)
    yaw = yaw_from_quaternion(msg.pose.orientation)
    if stamp is not None and all(math.isfinite(v) for v in (p.x, p.y, yaw)):
      with self.lock:
        self.pose = (float(p.x), float(p.y), yaw, stamp)

  def on_imu(self, msg):
    yaw = yaw_from_quaternion(msg.orientation)
    if math.isfinite(yaw):
      with self.lock:
        self.imu_yaw, self.imu_time = yaw, self.now()

  def on_stop_status(self, msg):
    try:
      status = json.loads(msg.data)
    except ValueError:
      return
    blocked = bool(status.get("latched")) or bool(status.get("fault"))
    with self.lock:
      self.gateway_blocked = blocked
      if blocked and self.stage not in (IDLE, ZERO_HOLD):
        self.finish("e-stop gateway latched", switch=False)

  def on_emergency(self, msg):
    with self.lock:
      if self.stage not in (IDLE, ZERO_HOLD):
        self.finish("emergency stop", switch=False)  # the e-stop owns the controllers

  def on_joy(self, msg):
    pressed = [i for i in self.abort_buttons if 0 <= i < len(msg.buttons) and msg.buttons[i]]
    if not pressed:
      return
    with self.lock:
      if self.stage in (IDLE, ZERO_HOLD, EXITING):
        return
      if pressed == [int(self.param("kick_button"))] and self.stage == RUNNING:
        # The arbiter "activates" the already active kick controller: end normally.
        self.begin_exit("square pressed")
      else:
        self.finish(f"joystick button {pressed[0]}", switch=False)  # the operator took over

  def current_yaw(self):
    return self.pose[2] if self.mode == "field" and self.pose else self.imu_yaw

  def inputs(self, now):
    """(ball Target, aim) in the robot's yaw-only frame now; (None, None) if unknown."""
    if self.ball is None:
      return None, None
    bx, by, stamp = self.ball
    goal = self.goal
    if goal is not None and now - goal[2] > float(self.param("goal_max_age")):
      goal = None
    if self.mode == "field":
      if self.pose is None:
        return None, None
      rx, ry, ryaw, pose_stamp = self.pose
      bx, by = field_to_robot(rx, ry, ryaw, bx, by)
      stamp = min(stamp, pose_stamp)
      if goal is not None:
        gx, gy = field_to_robot(rx, ry, ryaw, goal[0], goal[1])
        goal = (gx, gy, goal[2])
    aim = None if goal is None else aim_from_goal(bx, by, goal[0], goal[1])
    return Target(bx, by, stamp), aim

  def entry_problem(self, now) -> str:
    ball, aim = self.inputs(now)
    p = self.bridge.params
    if ball is None or now - ball.stamp > p.start_max_age:
      return "no fresh ball estimate"
    return entry_check(p, ball.x, ball.y, math.atan2(ball.y, ball.x) if aim is None else aim)

  # ------------------------------------------------------------ sequence
  def set_stage(self, stage, reason=None):
    self.stage, self.stage_since = stage, self.now()
    if reason is not None:
      self.reason = reason

  def on_activate(self, request, response):
    with self.lock:
      now = self.now()
      if self.stage != IDLE:
        response.success, response.message = False, f"busy ({self.stage})"
        return response
      problem = self.entry_problem(now)
      if not problem and self.mode == "robot" and now - self.imu_time > float(self.param("imu_max_age")):
        problem = "no recent IMU yaw"
      if not problem and bool(self.param("require_gateway")) and self.gateway_blocked is not False:
        problem = ("e-stop gateway latched or faulted (press Options so the walker stands)"
                   if self.gateway_blocked else "no /estop_controller/stop_status yet")
      others = self.count_publishers(self.cmd_topic) - 1
      if not problem and others != 0:
        problem = f"{others} other publisher(s) on {self.cmd_topic}: launch the course stack with teleop:=False"
      if not problem and not (self.list_client.service_is_ready() and self.switch_client.service_is_ready()):
        problem = "controller_manager services unavailable"
      if problem:
        response.success, response.message = False, f"not starting: {problem}"
        return response
      self.cancel = False
      self.bridge.phase, self.bridge.reason = Phase.IDLE, ""
      self.set_stage(CHECKING, "")
      self.cmd_pub.publish(Twist())
      self.list_client.call_async(ListControllers.Request()).add_done_callback(self.on_list)
    response.success, response.message = True, "starting: checking controllers"
    return response

  def on_list(self, future):
    with self.lock:
      if self.stage != CHECKING:
        return
      res = future.result()
      if res is None:
        return self.fail("controller list failed")
      states = {c.name: c.state for c in res.controller}
      kick, walker = str(self.param("kick_controller")), str(self.param("exit_controller"))
      if self.cancel:
        return self.fail("stopped")
      if states.get(kick) != "inactive":
        return self.fail(f"{kick} is {states.get(kick, 'not loaded')}; install the policy and restart the stack")
      if states.get(walker) != "active":
        return self.fail(f"{walker} must be active (press Options so the walker stands)")
      req = SwitchController.Request()
      req.activate_controllers = [kick]
      req.deactivate_controllers = sorted(
        n for n, s in states.items() if s == "active" and n.startswith("neural_controller") and n != kick)
      req.strictness = SwitchController.Request.STRICT
      self.set_stage(SWITCHING_IN)
      self.switch_client.call_async(req).add_done_callback(self.on_switch_in)

  def on_switch_in(self, future):
    with self.lock:
      if self.stage != SWITCHING_IN:
        return
      res = future.result()
      if res is None or not res.ok:
        return self.fail("the gateway refused the switch to the kick controller")
      now = self.now()
      ball, aim = self.inputs(now)
      why = "stopped" if self.cancel else self.bridge.start(now, ball, aim)
      if why:
        return self.begin_exit(why)
      self.yaw0 = self.current_yaw()
      self.set_stage(RUNNING, "")

  def fail(self, reason):
    """Before the kick controller is active: nothing to switch back."""
    self.cmd_pub.publish(Twist())
    self.bridge.phase = Phase.ABORTED
    self.zero_until = self.now() + float(self.param("zero_hold_s"))
    self.set_stage(ZERO_HOLD, reason)

  def finish(self, reason, switch=True):
    self.bridge.stop(reason)
    if self.stage in (CHECKING, SWITCHING_IN) and not switch:
      self.cancel = True
      return self.fail(reason)
    if self.stage == RUNNING and switch:
      return self.begin_exit(reason)
    self.fail(reason)  # e-stop/operator: the controllers are theirs now

  def begin_exit(self, reason):
    self.bridge.stop(reason)
    self.publish_target()  # keep a trained-on target until the walker is confirmed back
    self.exit_tries = 0
    self.set_stage(EXITING, reason or self.bridge.reason)
    self.send_exit()

  def send_exit(self):
    req = SwitchController.Request()
    req.activate_controllers = [str(self.param("exit_controller"))]
    req.deactivate_controllers = [str(self.param("kick_controller"))]
    req.strictness = SwitchController.Request.STRICT
    self.stage_since = self.now()
    if self.switch_client.service_is_ready():
      self.switch_client.call_async(req).add_done_callback(self.on_switch_out)

  def on_switch_out(self, future):
    with self.lock:
      if self.stage != EXITING:
        return
      res = future.result()
      if res is not None and res.ok:
        self.zero_until = self.now() + float(self.param("zero_hold_s"))
        self.set_stage(ZERO_HOLD)
      else:
        self.retry_exit()

  def retry_exit(self):
    self.exit_tries += 1
    if self.exit_tries < int(self.param("exit_retries")):
      self.send_exit()
      return
    self.estop_pub.publish(Empty())  # last resort: every neural controller damps
    self.get_logger().error("could not switch back to the walker: emergency stop sent; press R3")
    self.zero_until = self.now() + float(self.param("zero_hold_s"))
    self.set_stage(ZERO_HOLD, self.reason + "; exit refused, emergency stop sent")

  # ------------------------------------------------------------ services and tick
  def on_deactivate(self, request, response):
    with self.lock:
      was = self.stage
      if self.stage in (CHECKING, SWITCHING_IN):
        self.cancel = True  # handled when the pending call returns
        self.cmd_pub.publish(Twist())
      elif self.stage == RUNNING:
        self.begin_exit("stopped")
    response.success = True
    response.message = f"stopping ({was})" if was not in (IDLE, ZERO_HOLD) else "was not kicking"
    return response

  def on_tick(self):
    with self.lock:
      now = self.now()
      timeout = float(self.param("switch_timeout_s"))
      if self.stage == RUNNING:
        ball, aim = self.inputs(now)
        yaw = self.current_yaw()
        yaw_delta = None if yaw is None or self.yaw0 is None else yaw - self.yaw0
        x, y, wz = self.bridge.step(now, ball, aim, yaw_delta)
        if self.bridge.active:
          twist = Twist()
          twist.linear.x, twist.linear.y, twist.angular.z = x, y, wz
          self.cmd_pub.publish(twist)
        else:
          self.begin_exit(self.bridge.reason)
      elif self.stage in (CHECKING, SWITCHING_IN) and now - self.stage_since > timeout:
        self.fail(f"no answer from controller_manager in {self.stage}")
      elif self.stage == EXITING and now - self.stage_since > float(self.param("exit_timeout_s")):
        self.retry_exit()
      if self.stage == RUNNING and self.mode == "robot" and now - self.imu_time > float(self.param("imu_max_age")):
        self.begin_exit("IMU lost")
      if self.stage not in (IDLE, ZERO_HOLD) and now - self.last_owner_check >= 1.0:
        self.last_owner_check = now
        if self.count_publishers(self.cmd_topic) != 1:
          if self.stage == RUNNING:
            self.begin_exit("another node publishes /cmd_vel")
          elif self.stage in (CHECKING, SWITCHING_IN):
            self.cancel = True
      if self.stage == EXITING:
        self.publish_target()
      elif self.stage in ZERO_STAGES:
        self.cmd_pub.publish(Twist())
      if self.stage == ZERO_HOLD and now >= self.zero_until:
        self.set_stage(IDLE)
      self.publish_state()

  def publish_target(self):
    ball, aim = self.inputs(self.now())
    yaw = self.current_yaw()
    yaw_delta = None if yaw is None or self.yaw0 is None else yaw - self.yaw0
    x, y, wz = self.bridge.target_now(ball, aim, yaw_delta)
    twist = Twist()
    twist.linear.x, twist.linear.y, twist.angular.z = x, y, wz
    self.cmd_pub.publish(twist)

  def publish_state(self):
    state = (self.stage, self.bridge.phase.value, self.reason)
    if state == self.last_state:
      return
    self.last_state = state
    msg = String()
    msg.data = json.dumps({"stage": state[0], "phase": state[1], "reason": state[2]})
    self.state_pub.publish(msg)


def main():
  rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

  def interrupt(signum, frame):
    raise KeyboardInterrupt

  previous = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
  node = LearnedKickNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    for s in (signal.SIGINT, signal.SIGTERM):
      signal.signal(s, signal.SIG_IGN)  # a second Ctrl-C must not skip the cleanup
    # The switch back cannot complete without the executor: if the kick controller
    # may be active, damp every controller instead of leaving a target behind.
    if node.stage in (SWITCHING_IN, RUNNING, EXITING):
      node.estop_pub.publish(Empty())
    for _ in range(3):
      node.cmd_pub.publish(Twist())
      time.sleep(0.05)
    node.destroy_node()
    rclpy.try_shutdown()
    for signum, handler in previous.items():
      signal.signal(signum, handler)


if __name__ == "__main__":
  main()
