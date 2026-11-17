#!/usr/bin/env python3
"""Exercise the persistent movement CLI through real ROS and JSON stdin.

Only run in a networking-disabled, device-free ROS test container with
PUPPER_ISOLATED_ROS_TEST=1 and PUPPER_TEST_STOCK_POLICY pointing to the pinned
actor. Native process verification is explicitly stubbed inside the child:
there is no real controller manager, loaded actor, driver or physical robot.
The stock policy hash, services, graph ownership, sensor callbacks, session
loop, command transport, receipt writing and cleanup use production code.
Run each case under an external 20 s timeout. Output and receipts are retained.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(HERE), str(HERE.parent), str(ROOT)]


def digest(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
  if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1":
    raise SystemExit("Use the networking-disabled ROS test container without devices.")
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("case", choices=("two_pulses_eof", "stop", "eof", "_child"))
  parser.add_argument("--out", type=Path, required=True)
  args = parser.parse_args()
  args.out.mkdir(parents=True, exist_ok=True)

  import movement_trial as trial
  from run_through_kick_node import STOCK_POLICY_SHA256

  policy = Path(os.environ["PUPPER_TEST_STOCK_POLICY"]).resolve(strict=True)
  assert digest(policy) == STOCK_POLICY_SHA256, "Use the pinned actor without a hash override"
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

  if args.case == "_child":
    import foundation.hardware.runtime_identity as identity
    runtime = {"manager_pid": -1, "manager_start_ticks": -1,
               "files": {"policy": {"sha256": digest(policy)}},
               "test_only": "native identity stub; no real manager or loaded-model proof"}
    identity.verify_loaded = lambda preflight: runtime

    def same_manager(receipt):
      time.sleep(0.020)  # measured-order Pi scan cost, without a fake ROS executor
      return True

    identity.same_manager = same_manager
    manifest = args.out / "synthetic-manifest.json"
    manifest.write_text(json.dumps({"trial_profile": "walk"}))
    preflight = args.out / "synthetic-preflight.json"
    preflight.write_text(json.dumps({"manifest_path": str(manifest)}))
    return trial.main(["--session", "--execute", "--runtime-receipt", str(preflight),
                       "--out", str(args.out)])

  import rclpy
  from controller_manager_msgs.msg import ControllerState
  from controller_manager_msgs.srv import ListControllers
  from geometry_msgs.msg import Twist
  from rcl_interfaces.srv import GetParameters
  from rclpy.executors import SingleThreadedExecutor
  from rclpy.node import Node
  from rclpy.parameter import Parameter
  from rclpy.qos import qos_profile_sensor_data
  from sensor_msgs.msg import Imu
  from std_msgs.msg import Empty, Float32MultiArray, String

  rclpy.init()

  class FakeRobot(Node):
    def __init__(self):
      super().__init__("movement_session_smoke_mux")
      self.latest = trial.ZERO
      self.commands, self.robot_commands = [], []
      self.observations = self.imu_samples = self.emergency_stops = 0
      self.latched = False
      self.create_service(ListControllers, "/controller_manager/list_controllers", self.controllers)
      self.create_service(GetParameters, "/neural_controller_default/get_parameters", self.parameters)
      self.mux_pub = self.create_publisher(Twist, "/cmd_vel", 10)
      self.stop_pub = self.create_publisher(String, "/estop_controller/stop_status", 10)
      self.obs_pub = self.create_publisher(Float32MultiArray, "/neural_controller_default/observation", 1)
      self.imu_pub = self.create_publisher(Imu, "/imu_sensor_broadcaster/imu", qos_profile_sensor_data)
      self.create_subscription(Twist, "/kick_cmd_vel", self.command, 10)
      self.create_subscription(Twist, "/cmd_vel", lambda msg: self.robot_commands.append(
          (time.monotonic(), (msg.linear.x, msg.linear.y, msg.angular.z))), 10)
      self.create_subscription(Empty, "/emergency_stop", self.stop, 10)
      self.create_timer(0.020, self.feedback)

    def command(self, msg):
      self.latest = (msg.linear.x, msg.linear.y, msg.angular.z)
      self.commands.append((time.monotonic(), self.latest))
      self.mux_pub.publish(msg)

    def stop(self, msg):
      self.latched = True
      self.emergency_stops += 1

    def feedback(self):
      self.stop_pub.publish(String(data=json.dumps({"latched": self.latched, "fault": False})))
      values = [0.0] * 960
      values[6:9] = trial.ZERO if self.latched else self.latest
      self.obs_pub.publish(Float32MultiArray(data=values))
      self.observations += 1
      imu = Imu()
      imu.orientation.w = 1.0
      self.imu_pub.publish(imu)
      self.imu_samples += 1

    def controllers(self, request, response):
      for name in ("neural_controller_default", "neural_controller_student"):
        controller = ControllerState()
        controller.name, controller.type = name, "neural_controller/NeuralController"
        controller.state = "active" if name == "neural_controller_default" else "inactive"
        controller.claimed_interfaces = ["leg_front_l_1/position"] if controller.state == "active" else []
        response.controller.append(controller)
      return response

    def parameters(self, request, response):
      values = {"model_path": str(policy), "max_vx_forward": 0.49, "max_vx_backward": 0.3,
                "speed_cap": 0.49, "game_buttons": False}
      for name in request.names:
        response.values.append(Parameter(name, value=values[name]).get_parameter_value()
                               if name in values else Parameter(name).get_parameter_value())
      return response

  fake = FakeRobot()
  executor = SingleThreadedExecutor()
  executor.add_node(fake)
  spin_thread = threading.Thread(target=executor.spin, daemon=True)
  spin_thread.start()
  events, event_queue = [], queue.Queue()
  stderr_path = args.out / "client-stderr.log"
  stderr = stderr_path.open("w")
  child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "_child", "--out", str(args.out)],
                           stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=stderr, text=True, bufsize=1)

  def read_output():
    with (args.out / "events.jsonl").open("w") as transcript:
      for line in child.stdout:
        transcript.write(line)
        transcript.flush()
        event = json.loads(line)
        events.append(event)
        event_queue.put(event)

  reader = threading.Thread(target=read_output, daemon=True)
  reader.start()

  def send(message):
    child.stdin.write(json.dumps(message) + "\n")
    child.stdin.flush()

  def wait_event(name, seconds=5.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
      try:
        event = event_queue.get(timeout=min(0.1, deadline - time.monotonic()))
      except queue.Empty:
        assert child.poll() is None, ("CLI exited", child.returncode, stderr_path.read_text(), events)
        continue
      assert event["event"] not in ("fault", "refused"), event
      if event["event"] == name:
        return event
    raise AssertionError(("Event deadline", name, events, stderr_path.read_text()))

  receipts = []
  stop_requested_at = None
  try:
    wait_event("session_ready")
    send({"command": "list"})
    assert "forward" in wait_event("trials")["axes"]
    deadline = time.monotonic() + 5.0
    while True:
      send({"command": "status"})
      status = wait_event("status")
      if status["problem"] is None:
        break
      assert time.monotonic() < deadline, status
      time.sleep(0.1)
    assert not fake.commands, "Idle session published a command before prepare/run"
    axes = ("forward", "turn_left") if args.case == "two_pulses_eof" else ("forward",)
    for axis in axes:
      send({"command": "prepare", "axis": axis, "speed": 0.2 if axis == "forward" else 0.35,
            "duration": 0.5 if args.case == "two_pulses_eof" else 2.0})
      prepared = wait_event("prepared")
      assert prepared["problem"] is None, prepared
      before = len(fake.commands)
      time.sleep(0.06)  # real callbacks remain live after preparation
      assert len(fake.commands) == before, "Prepare started motion"
      send({"command": "run", "token": prepared["token"]})
      wait_event("started")
      if args.case != "two_pulses_eof":
        time.sleep(0.15)
        stop_requested_at = time.monotonic()
        if args.case == "stop":
          send({"command": "stop", "reason": "synthetic operator stop"})
        else:
          child.stdin.close()
        wait_event("stopping")
      completed = wait_event("trial_complete")
      record = json.loads(Path(completed["file"]).read_text())
      receipts.append(record)
      assert record["client_identity"]["sha256"] == digest(HERE / "movement_trial.py")
      assert record["summary"]["zero_confirmed_in_observation_s"] is not None
      assert record["imu"]["status"] == "sampled" and record["imu"]["samples"] > 5
      if args.case == "two_pulses_eof":
        assert record["outcome"] == "completed" and record["command_verification"]["passed"], record
        assert 0.45 <= record["summary"]["elapsed_s"] <= 0.8
        before = len(fake.commands)
        time.sleep(0.25)
        assert len(fake.commands) == before, "Finished trial automatically published another command"
      else:
        assert record["outcome"] == "stopped", record
        assert record["summary"]["elapsed_s"] < 0.6
    if args.case == "two_pulses_eof":
      child.stdin.close()
      assert wait_event("stopping")["reason"] == "session stdin closed"
    exit_code = child.wait(timeout=3)
    reader.join(timeout=1)
    time.sleep(0.1)  # allow final zero deliveries through the real subscriber
    assert exit_code == (0 if args.case == "two_pulses_eof" else 4), (exit_code, stderr_path.read_text())
    assert len(list(args.out.glob("movement-*.json"))) == len(axes)
    assert fake.commands[-1][1] == trial.ZERO and fake.robot_commands[-1][1] == trial.ZERO
    assert fake.emergency_stops == (0 if args.case == "two_pulses_eof" else 1)
    stop_latency = None
    if stop_requested_at is not None:
      stop_latency = next(t - stop_requested_at for t, command in fake.commands
                          if t >= stop_requested_at and command == trial.ZERO)
      assert stop_latency < 0.2, stop_latency
    runs = []
    current = []
    for stamp, command in fake.commands:
      if command != trial.ZERO:
        current.append(stamp)
      elif current:
        runs.append(current)
        current = []
    assert len(runs) == len(axes)
    gaps = [max((b - a for a, b in zip(run, run[1:])), default=0) for run in runs]
    assert max(gaps) < 0.15, gaps
    result = {"case": args.case, "passed": True, "exit_code": exit_code,
              "trial_count": len(receipts), "emergency_stop_receipts": fake.emergency_stops,
              "stop_input_to_zero_s": stop_latency, "max_nonzero_command_gap_s": gaps,
              "synthetic_observations": fake.observations, "synthetic_imu_samples": fake.imu_samples,
              "feedback_hz": 50, "identity_check_delay_s": 0.02,
              "receipts": [{"file": record["file"], "outcome": record["outcome"],
                            "reason": record["reason"], "summary": record["summary"],
                            "imu": record["imu"], "command_verification": record["command_verification"]}
                           for record in receipts],
              "source_sha256": {str(p.relative_to(ROOT)): digest(p) for p in (
                  Path(__file__), HERE / "movement_trial.py", HERE.parent / "run_through_kick.py",
                  HERE.parent / "run_through_kick_node.py", ROOT / "foundation/hardware/runtime_identity.py")},
              "stock_policy_sha256": digest(policy),
              "scope": "Actual persistent CLI/JSON stdin/ROS callbacks, synthetic 50 Hz feedback; no hardware or real native manager",
              "limitation": "verify_loaded and same_manager stubbed only in isolated child; native identity and loaded-model evidence not tested"}
    (args.out / "events.json").write_text(json.dumps(events, indent=2) + "\n")
    (args.out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0
  finally:
    if child.poll() is None:
      child.kill()
      child.wait(timeout=3)
    stderr.close()
    executor.shutdown(timeout_sec=2)
    spin_thread.join(timeout=2)
    fake.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  sys.exit(main())
