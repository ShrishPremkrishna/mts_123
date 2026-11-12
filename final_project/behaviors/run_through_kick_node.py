#!/usr/bin/env python3
"""ROS 2 service around ``run_through_kick``: one run-through kick per activation.

Same shape as follow_me_pupper/robot/follow_service.py, so the harness can
drive it the same way:

    services   activate_run_through_kick, deactivate_run_through_kick (std_srvs/Trigger)
    subscribes /soccer/ball  geometry_msgs/PointStamped  ball centre, base_link
                             yaw-only frame (x forward, y left) [m]; header.stamp =
                             image capture time
               /soccer/goal  geometry_msgs/PointStamped  goal target, same frame
                             (optional; without it the kick goes straight through)
    publishes  /kick_cmd_vel geometry_msgs/Twist at 20 Hz while active, into
                             cmd_vel_mux (add it to the mux inputs, below the
                             joystick); one zero Twist when the kick ends
               /run_through_kick/state  std_msgs/String  JSON {state, reason}

The walker must accept the run speed: charge_speed above 0.49 m/s needs
speed_cap raised (follow_me_pupper/robot/speed_cap.yaml). Run by hand:

    python3 final_project/behaviors/run_through_kick_node.py --ros-args -p charge_speed:=0.8
    ros2 service call /activate_run_through_kick std_srvs/srv/Trigger
"""

from __future__ import annotations

import json
import os
import signal
import sys
import threading
from dataclasses import fields, replace

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_through_kick import BallObs, KickParams, RunThroughKick, State, aim_from_goal  # noqa: E402


class RunThroughKickNode(Node):
  def __init__(self):
    super().__init__("run_through_kick")
    self.declare_parameter("cmd_vel_topic", "/kick_cmd_vel")
    self.declare_parameter("ball_topic", "/soccer/ball")
    self.declare_parameter("goal_topic", "/soccer/goal")
    self.declare_parameter("rate_hz", 20.0)  # cmd_vel_mux drops a source quiet for 0.5 s
    self.declare_parameter("goal_max_age", 1.0)  # s; older goal -> kick straight
    # Every KickParams field is a ROS parameter of the same name.
    defaults = KickParams()
    for f in fields(KickParams):
      self.declare_parameter(f.name, float(getattr(defaults, f.name)))
    params = replace(
      defaults, **{f.name: float(self.get_parameter(f.name).value) for f in fields(KickParams)}
    )

    self.lock = threading.Lock()
    self.kick = RunThroughKick(params)
    self.ball: BallObs | None = None
    self.goal: tuple[float, float, float] | None = None  # x, y, receive time
    self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
    self.state_pub = self.create_publisher(String, "run_through_kick/state", 10)
    self.create_subscription(PointStamped, self.get_parameter("ball_topic").value, self.on_ball, 10)
    self.create_subscription(PointStamped, self.get_parameter("goal_topic").value, self.on_goal, 10)
    self.create_service(Trigger, "activate_run_through_kick", self.on_activate)
    self.create_service(Trigger, "deactivate_run_through_kick", self.on_deactivate)
    rate = float(self.get_parameter("rate_hz").value)
    if not 5.0 <= rate <= 100.0:
      raise ValueError("rate_hz must be in [5, 100]")
    self.create_timer(1.0 / rate, self.on_tick)
    self.last_state = None
    self.get_logger().info(f"run_through_kick ready: {params}")

  def now(self) -> float:
    return self.get_clock().now().nanoseconds * 1e-9

  # ------------------------------------------------------------ inputs
  def on_ball(self, msg: PointStamped) -> None:
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    with self.lock:
      self.ball = BallObs(float(msg.point.x), float(msg.point.y), stamp or self.now())

  def on_goal(self, msg: PointStamped) -> None:
    with self.lock:
      self.goal = (float(msg.point.x), float(msg.point.y), self.now())

  # ------------------------------------------------------------ services
  def on_activate(self, request, response):
    with self.lock:
      if self.kick.active:
        response.success, response.message = True, "already kicking"
        return response
      self.kick.start(self.now())
    response.success, response.message = True, "run-through kick started"
    self.get_logger().warning(response.message)
    return response

  def on_deactivate(self, request, response):
    with self.lock:
      was_active = self.kick.active
      self.kick.stop(self.now())
    self.cmd_pub.publish(Twist())  # zero travel immediately
    response.success = True
    response.message = "stopped" if was_active else "was not kicking"
    return response

  # ------------------------------------------------------------ tick
  def on_tick(self) -> None:
    with self.lock:
      if not self.kick.active:
        self.publish_state()
        return
      now = self.now()
      aim = None
      if self.goal is not None and self.ball is not None and now - self.goal[2] <= float(
        self.get_parameter("goal_max_age").value
      ):
        aim = aim_from_goal(self.ball.x, self.ball.y, self.goal[0], self.goal[1])
      cmd = self.kick.step(now, self.ball, aim)
      twist = Twist()
      twist.linear.x, twist.linear.y, twist.angular.z = cmd.vx, cmd.vy, cmd.wz
      self.cmd_pub.publish(twist)  # a finished kick publishes its final zero here
      self.publish_state()

  def publish_state(self) -> None:
    state = (self.kick.state.value, self.kick.reason)
    if state != self.last_state:
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
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()
    for signum, handler in previous.items():
      signal.signal(signum, handler)


if __name__ == "__main__":
  main()
