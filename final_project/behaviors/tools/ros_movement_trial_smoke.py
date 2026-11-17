#!/usr/bin/env python3
"""Real ROS transport smoke for movement_trial, exclusively in an isolated container.

No manager, driver, camera, controller lifecycle or hardware is started. The
actual lazy trial node talks to synthetic services/topics. CLI runtime identity
remains mandatory: this harness exercises the node directly, while fake-/proc
tests cover that separate gate. Run each case under an external 15 s timeout.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def digest(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
  if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1":
    raise SystemExit("Use the networking-disabled ROS test container without devices.")
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("case", choices=("pulse", "sprint", "wrong_stock", "lost_observation", "competing_sender"))
  args = parser.parse_args()
  here = Path(__file__).resolve().parent
  sys.path.insert(0, str(here.parent))
  import movement_trial as trial_module
  import rclpy
  from controller_manager_msgs.msg import ControllerState
  from controller_manager_msgs.srv import ListControllers
  from geometry_msgs.msg import Twist
  from rcl_interfaces.srv import GetParameters
  from rclpy.executors import SingleThreadedExecutor
  from rclpy.node import Node
  from rclpy.parameter import Parameter
  from std_msgs.msg import Empty, Float32MultiArray, String
  from run_through_kick_node import STOCK_POLICY_SHA256

  policy = Path(os.environ["PUPPER_TEST_STOCK_POLICY"]).resolve(strict=True)
  assert digest(policy) == STOCK_POLICY_SHA256, "Use the actual pinned stock actor, not a hash override"
  metadata = json.loads(policy.read_text())
  assert metadata["single_observation_size"] == 48 and metadata["observation_history"] == 20
  assert metadata["obs_spec"]["frame_order"] == "newest_first"
  offset = 0
  for component in metadata["obs_spec"]["components"]:
    if component["name"] == "command":
      assert offset == 6 and component["size"] == 3
      break
    offset += component["size"]
  else:
    raise AssertionError("Stock observation has no command component")

  # No hidden --skip-preflight option: the public CLI must refuse before ROS.
  with tempfile.TemporaryDirectory(prefix="movement-smoke-") as output:
    if args.case == "pulse":
      refused = subprocess.run(
          [sys.executable, str(here / "movement_trial.py"), "forward", "--execute", "--out", output],
          text=True, capture_output=True, timeout=3)
      assert refused.returncode == 2 and "runtime-receipt is required" in refused.stderr, refused.stderr
      assert not list(Path(output).glob("*.json"))

    rclpy.init()

    class FakeRobot(Node):
      def __init__(self):
        super().__init__("movement_smoke_mux")
        self.active = "neural_controller_student" if args.case == "wrong_stock" else "neural_controller_default"
        self.latched, self.emergency_stops = False, 0
        self.emit_observations = True
        self.latest = (0.0, 0.0, 0.0)
        self.commands, self.robot_commands = [], []
        self.observation_messages = 0
        self.create_service(ListControllers, "/controller_manager/list_controllers", self.controllers)
        self.create_service(GetParameters, "/neural_controller_default/get_parameters", self.parameters)
        self.mux_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.stop_pub = self.create_publisher(String, "/estop_controller/stop_status", 10)
        self.obs_pub = self.create_publisher(Float32MultiArray, "/neural_controller_default/observation", 1)
        self.create_subscription(Twist, "/kick_cmd_vel", self.command, 10)
        self.create_subscription(Twist, "/cmd_vel", lambda msg: self.robot_commands.append(
            (msg.linear.x, msg.linear.y, msg.angular.z)), 10)
        self.create_subscription(Empty, "/emergency_stop", self.stop, 10)
        self.create_timer(0.025, self.feedback)

      def command(self, msg):
        self.latest = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.commands.append((time.monotonic(), self.latest))
        self.mux_pub.publish(msg)

      def stop(self, msg):
        self.emergency_stops += 1
        self.latched = True

      def feedback(self):
        self.stop_pub.publish(String(data=json.dumps({"latched": self.latched, "fault": False})))
        if self.emit_observations:
          values = [0.0] * 960
          values[6:9] = (0.0, 0.0, 0.0) if self.latched else self.latest
          self.obs_pub.publish(Float32MultiArray(data=values))
          self.observation_messages += 1

      def controllers(self, request, response):
        for name in ("neural_controller_default", "neural_controller_student"):
          controller = ControllerState()
          controller.name, controller.type = name, "neural_controller/NeuralController"
          controller.state = "active" if name == self.active else "inactive"
          controller.claimed_interfaces = ["leg_front_l_1/position"] if controller.state == "active" else []
          response.controller.append(controller)
        return response

      def parameters(self, request, response):
        cap = 1.0 if args.case == "sprint" else 0.49
        values = {"model_path": str(policy), "max_vx_forward": cap, "max_vx_backward": 0.3,
                  "speed_cap": cap, "game_buttons": False}
        for name in request.names:
          response.values.append(Parameter(name, value=values[name]).get_parameter_value()
                                 if name in values else Parameter(name).get_parameter_value())
        return response

    fake = FakeRobot()
    duration = 2.0 if args.case in ("lost_observation", "competing_sender") else 0.5
    profile = "sprint" if args.case == "sprint" else "walk"
    speed = 0.7 if args.case == "sprint" else 0.2
    node = trial_module.MovementTrialNode(
        trial_module.PulseRequest.build("forward", speed, duration, profile), output)
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    executor.add_node(node)
    deadline = time.monotonic() + 12.0

    def until(predicate, seconds=4.0):
      end = min(deadline, time.monotonic() + seconds)
      while not predicate() and time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
      assert predicate(), f"ROS deadline: phase={node.phase}, problem={node.last_problem}, stock={node.stock.error}"

    def spin(seconds):
      end = min(deadline, time.monotonic() + seconds)
      while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)

    try:
      if args.case == "wrong_stock":
        spin(1.5)
        assert node.try_start() is not None
        assert node.phase == "waiting" and node.stock.problem() is not None
        assert not fake.commands and not fake.robot_commands and fake.emergency_stops == 0
      else:
        until(lambda: node.start_problem() is None)
        assert node.try_start() is None
        until(lambda: any(command != trial_module.ZERO for _, command in fake.commands))
        if args.case == "lost_observation":
          fake.emit_observations = False
        if args.case == "competing_sender":
          # An idle competing writer must be noticed through the actual ROS
          # graph, without relying on different command payloads arriving.
          competing = fake.create_publisher(Twist, "/kick_cmd_vel", 10)
        until(lambda: node.phase == "finished")
        until(lambda: fake.commands[-1][1] == trial_module.ZERO)
        if args.case in ("pulse", "sprint"):
          until(lambda: node.zero_confirmed_at is not None)
          assert node.record["outcome"] == "completed"
          assert 0.45 <= node.zero_sent_at - node.started <= 0.75
          assert fake.emergency_stops == 0
        else:
          until(lambda: fake.emergency_stops > 0)
          assert node.record["outcome"] == "stopped"
          assert ("observation stale" in node.record["reason"] if args.case == "lost_observation"
                  else "another node publishes /kick_cmd_vel" in node.record["reason"]), node.record["reason"]
          assert node.zero_sent_at - node.started < (0.6 if args.case == "lost_observation" else duration)

      receipt = node.finish()
      spin(0.15)  # deliver finally's zeros through actual DDS callbacks
      record = json.loads(receipt.read_text())
      if args.case == "wrong_stock":
        assert record["outcome"] == "not_started" and record["published_commands"] == 0
        assert not fake.commands and not fake.robot_commands and fake.emergency_stops == 0
      else:
        assert fake.commands[-1][1] == trial_module.ZERO and fake.robot_commands[-1] == trial_module.ZERO
        if args.case in ("pulse", "sprint"):
          assert record["summary"]["applied_matched_requested"] is True
          assert record["summary"]["zero_confirmed_in_observation_s"] is not None
      print(json.dumps({"case": args.case, "profile": profile, "speed": speed,
                        "passed": True, "outcome": record["outcome"],
                        "reason": record["reason"], "summary": record["summary"],
                        "emergency_stop_receipts": fake.emergency_stops,
                        "synthetic_observations": fake.observation_messages,
                        "stock_policy_sha256": digest(policy),
                        "source_sha256": {str(p): digest(p) for p in (
                            Path(__file__), here / "movement_trial.py", here.parent / "run_through_kick_node.py")},
                        "scope": "Real ROS callbacks/services with synthetic robot feedback; no physical validation"}, indent=2))
    finally:
      executor.remove_node(node)
      executor.remove_node(fake)
      node.destroy_node()
      fake.destroy_node()
      executor.shutdown()
      rclpy.shutdown()


if __name__ == "__main__":
  main()
