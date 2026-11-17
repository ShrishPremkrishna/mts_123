"""Exercise the stock-shot guard through real ROS services against a fake robot.

Run only in the networking-disabled ROS test container, with no hardware mounts.
No controller manager, motor driver, camera or learned policy is started. The
fake services describe stock/controller parameters; published commands stay in
the isolated ROS graph. This checks transport/callback wiring, not movement.
"""

import json
import os
from pathlib import Path
import time


def main():
  if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1":
    raise SystemExit("Use the isolated ROS container; this test publishes synthetic velocity messages.")
  import rclpy
  from rclpy.executors import SingleThreadedExecutor
  from rclpy.node import Node
  from rclpy.parameter import Parameter
  from controller_manager_msgs.msg import ControllerState
  from controller_manager_msgs.srv import ListControllers
  from rcl_interfaces.srv import GetParameters
  from geometry_msgs.msg import Twist
  from std_msgs.msg import Empty, String
  from std_srvs.srv import Trigger
  from run_through_kick_node import RunThroughKickNode

  policy = str(Path(os.environ["PUPPER_TEST_STOCK_POLICY"]).resolve(strict=True))
  rclpy.init(args=["--ros-args", "-p", "input_mode:=manual", "-p", "manual_ball_x:=1.0"])

  class FakeRobot(Node):
    def __init__(self):
      super().__init__("cmd_vel_mux")
      self.active = "neural_controller_default"
      self.latched, self.emergency_stops = False, 0
      self.create_service(ListControllers, "/controller_manager/list_controllers", self.controllers)
      self.create_service(GetParameters, "/neural_controller_default/get_parameters", self.parameters)
      self.mux_pub = self.create_publisher(Twist, "/cmd_vel", 10)
      self.stop_pub = self.create_publisher(String, "/estop_controller/stop_status", 10)
      self.create_timer(0.05, lambda: self.stop_pub.publish(
        String(data=json.dumps({"latched": self.latched, "fault": False}))))
      self.create_subscription(Empty, "/emergency_stop", self.stop, 10)
      self.commands, self.states = [], []
      self.create_subscription(Twist, "/kick_cmd_vel", self.commands.append, 10)
      self.create_subscription(String, "/run_through_kick/state",
                               lambda msg: self.states.append(json.loads(msg.data)), 10)

    def stop(self, message):
      self.emergency_stops += 1
      self.latched = True

    def controllers(self, request, response):
      for name in ("neural_controller_default", "neural_controller_student"):
        c = ControllerState()
        c.name, c.type = name, "neural_controller/NeuralController"
        c.state = "active" if name == self.active else "inactive"
        c.claimed_interfaces = ["leg_front_l_1/position"] if c.state == "active" else []
        response.controller.append(c)
      return response

    def parameters(self, request, response):
      values = {"model_path": policy, "max_vx_forward": 0.49, "max_vx_backward": 0.3,
                "speed_cap": 0.49, "game_buttons": False}
      for name in request.names:
        response.values.append(Parameter(name, value=values[name]).get_parameter_value()
                               if name in values else Parameter(name).get_parameter_value())
      return response

  fake, shot = FakeRobot(), RunThroughKickNode()
  executor = SingleThreadedExecutor()
  executor.add_node(fake)
  executor.add_node(shot)

  def until(predicate, seconds=5.0):
    deadline = time.monotonic() + seconds
    while not predicate() and time.monotonic() < deadline:
      executor.spin_once(timeout_sec=0.02)
    if not predicate():
      raise AssertionError("ROS smoke deadline expired")

  def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
      executor.spin_once(timeout_sec=0.02)

  client = fake.create_client(Trigger, "/activate_run_through_kick")

  def activate():
    future = client.call_async(Trigger.Request())
    until(future.done)
    return future.result()

  try:
    until(client.service_is_ready)
    # Wait for DDS discovery and the node's asynchronous evidence polling.
    spin(2.0)
    accepted = activate()
    assert accepted.success, accepted.message
    until(lambda: any(abs(m.linear.x) > 0 for m in fake.commands))

    fake.active = "neural_controller_student"
    until(lambda: any(s.get("state") == "aborted" for s in fake.states))
    spin(0.1)
    assert fake.emergency_stops, "Zero is not a stop for a target-conditioned policy"
    fake.latched = False  # simulate deliberate operator rearm, not automatic recovery
    spin(1.0)
    shot.set_parameters([Parameter("manual_ball_x", value=1.0)])
    refused = activate()
    assert not refused.success, "A target-conditioned student slot must not accept velocity shots"

    fake.active = "neural_controller_default"
    spin(2.0)
    shot.set_parameters([Parameter("charge_speed", value=0.7)])
    capped = activate()
    assert not capped.success, "A request above the verified stock cap must be refused"
    print(json.dumps({"scope": "real ROS transport with fake robot in isolated container",
                      "stock_045_accepted": True, "controller_takeover_aborted": True,
                      "controller_takeover_requested_emergency_stop": True,
                      "student_refused": True, "over_cap_refused": True,
                      "physical_validation": False}, indent=2))
  finally:
    executor.remove_node(shot)
    executor.remove_node(fake)
    shot.destroy_node()
    fake.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
