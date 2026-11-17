"""Offline checks for movement_trial: pure pulse/extraction logic and the fake-ROS control boundary.

    .venv-api/bin/python -m pytest -q final_project/behaviors/tools/test_movement_trial.py

Not a ROS run; it checks that the tool refuses to move without verified stock
authority, stops and zeros on every stop path, and records what it saw.
"""

from __future__ import annotations

import hashlib
import importlib
import json
import math
import os
import subprocess
import sys
import types
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
BEHAVIORS = HERE.parent
for p in (str(HERE), str(BEHAVIORS)):
  if p not in sys.path:
    sys.path.insert(0, p)

import _fake_ros  # noqa: E402

ZERO = (0.0, 0.0, 0.0)
ROOTS = {"rclpy", "rcl_interfaces", "geometry_msgs", "std_msgs", "std_srvs", "sensor_msgs",
         "controller_manager_msgs", "run_through_kick_node", "movement_trial"}


@pytest.fixture(autouse=True)
def restore_modules():
  saved = {n: m for n, m in sys.modules.items() if n.split(".")[0] in ROOTS}
  yield
  for name in list(sys.modules):
    if name.split(".")[0] in ROOTS:
      del sys.modules[name]
  sys.modules.update(saved)


def pure():
  """The module imported without any ROS stand-ins: pure helpers must not need them."""
  for name in list(sys.modules):
    if name.split(".")[0] in ROOTS:
      del sys.modules[name]
  return importlib.import_module("movement_trial")


# ---------------------------------------------------------------- pure: request validation


def test_default_pulse_speeds_are_the_bounded_prototype_defaults():
  mt = pure()
  assert mt.DEFAULT_SPEED == {"forward": 0.2, "backward": 0.2, "turn": 0.35, "lateral": 0.15}
  assert mt.MAX_PULSE_S == 3.0


@pytest.mark.parametrize("axis,speed,duration,sign", [
  ("forward", 0.2, 1.0, (1, 0, 0)), ("backward", 0.2, 1.0, (-1, 0, 0)),
  ("left", 0.15, 1.0, (0, 1, 0)), ("right", 0.15, 1.0, (0, -1, 0)),
  ("turn_left", 0.35, 1.0, (0, 0, 1)), ("turn_right", 0.35, 1.0, (0, 0, -1)),
])
def test_request_maps_axis_to_a_single_signed_twist_component(axis, speed, duration, sign):
  mt = pure()
  req = mt.PulseRequest.build(axis, speed, duration)
  assert req.twist == tuple(s * speed for s in sign)
  assert req.duration_s == duration


@pytest.mark.parametrize("axis,speed,duration", [
  ("forward", 0.2, 3.5), ("forward", 0.0, 1.0), ("forward", -0.2, 1.0),
  ("forward", float("nan"), 1.0), ("forward", 0.2, 0.0), ("sideways", 0.2, 1.0),
  ("forward", 0.5, 1.0), ("left", 0.4, 1.0), ("turn_left", 2.5, 1.0),
])
def test_request_rejects_out_of_envelope_or_malformed_pulses(axis, speed, duration):
  mt = pure()
  with pytest.raises(ValueError):
    mt.PulseRequest.build(axis, speed, duration)


# ---------------------------------------------------------------- pure: observation extraction


def test_applied_command_comes_from_the_newest_frame_only():
  mt = pure()
  newest = [0.0] * 48
  newest[6:9] = [0.19, 0.01, -0.02]
  older = [0.0] * 48
  older[6:9] = [0.45, 0.0, 0.0]
  assert mt.applied_command(newest + older * 19) == pytest.approx((0.19, 0.01, -0.02))


@pytest.mark.parametrize("length", [0, 36, 720, 959, 961])
def test_applied_command_refuses_a_frame_count_that_does_not_fit_the_schema(length):
  mt = pure()
  with pytest.raises(ValueError):
    mt.applied_command([0.0] * length)


def test_applied_command_refuses_non_finite_values():
  mt = pure()
  data = [0.0] * 960
  data[7] = float("nan")
  with pytest.raises(ValueError):
    mt.applied_command(data)


# ---------------------------------------------------------------- pure: pulse timing/summary


def test_summary_reports_requested_integral_as_command_integral_not_distance():
  mt = pure()
  samples = [mt.Sample(t=0.0, requested=(0.2, 0.0, 0.0), applied=(0.2, 0.0, 0.0)),
             mt.Sample(t=0.5, requested=(0.2, 0.0, 0.0), applied=(0.2, 0.0, 0.0)),
             mt.Sample(t=1.0, requested=(0.0, 0.0, 0.0), applied=(0.0, 0.0, 0.0))]
  summary = mt.summarize(samples, started=0.0, zero_sent_at=1.0, zero_confirmed_at=1.1)
  assert summary["elapsed_s"] == pytest.approx(1.0)
  assert summary["requested_command_integral"]["vx_m"] == pytest.approx(0.2)
  assert summary["applied_command_integral"]["vx_m"] == pytest.approx(0.2)
  assert "distance" not in json.dumps(summary).lower()
  assert summary["zero_confirmed_in_observation_s"] == pytest.approx(0.1)


def test_summary_flags_mismatch_between_requested_and_applied():
  mt = pure()
  samples = [mt.Sample(t=0.0, requested=(0.4, 0.0, 0.0), applied=(0.2, 0.0, 0.0)),
             mt.Sample(t=0.5, requested=(0.4, 0.0, 0.0), applied=(0.2, 0.0, 0.0)),
             mt.Sample(t=1.0, requested=(0.0, 0.0, 0.0), applied=(0.0, 0.0, 0.0))]
  summary = mt.summarize(samples, started=0.0, zero_sent_at=1.0, zero_confirmed_at=None)
  assert summary["max_abs_requested_minus_applied"]["vx"] == pytest.approx(0.2)
  assert summary["applied_matched_requested"] is False
  assert summary["zero_confirmed_in_observation_s"] is None


def test_summary_without_observations_is_explicit():
  mt = pure()
  samples = [mt.Sample(t=0.0, requested=(0.2, 0.0, 0.0), applied=None),
             mt.Sample(t=1.0, requested=(0.0, 0.0, 0.0), applied=None)]
  summary = mt.summarize(samples, started=0.0, zero_sent_at=1.0, zero_confirmed_at=None)
  assert summary["applied_command_integral"] is None
  assert summary["applied_matched_requested"] is None
  assert summary["observation_samples"] == 0


@pytest.mark.parametrize("matched,zero,runtime,expected", [
  (True, 0.1, True, 0),
  (False, 0.1, True, 5),
  (None, 0.1, True, 5),
  (True, None, True, 5),
  (True, 0.1, False, 5),
])
def test_completed_timer_requires_command_zero_and_runtime_evidence(matched, zero, runtime, expected):
  mt = pure()
  record = {"outcome": "completed", "summary": {
    "applied_matched_requested": matched,
    "zero_confirmed_in_observation_s": zero,
  }}
  mt.finalize_verification(record, runtime)
  assert mt.result_exit_code(record) == expected
  assert record["command_verification"]["passed"] == (expected == 0)
  assert bool(record["command_verification"]["problems"]) == (expected != 0)
  assert "physical outcome remains unverified" in record["command_verification"]["scope"]


@pytest.mark.parametrize("outcome,code", [("stopped", 4), ("not_started", 3)])
def test_stop_and_refusal_keep_their_distinct_exit_status(outcome, code):
  mt = pure()
  record = {"outcome": outcome, "summary": None}
  mt.finalize_verification(record, False)
  assert not record["command_verification"]["passed"]
  assert mt.result_exit_code(record) == code


# ---------------------------------------------------------------- pure: receipt


def test_receipt_keeps_imu_and_operator_observation_pending(tmp_path):
  mt = pure()
  record = mt.build_record(request=mt.PulseRequest.build("forward", 0.2, 1.0), mode="dry-run",
                           outcome="dry_run", reason="", samples=[], imu=None,
                           summary=None, stock=None, started_wall="2026-11-16T00:00:00+00:00")
  assert record["imu"]["status"] == "no IMU sample received"
  assert record["imu"]["note"].startswith("sensor output, not external truth")
  assert record["operator_observation"] == "pending: fill in observed motion/stability by hand"
  assert record["interpretation"]["command_integral_is_distance"] is False
  path = mt.write_record(record, tmp_path)
  assert json.loads(path.read_text())["request"]["axis"] == "forward"
  line = mt.receipt_line(record)
  assert "forward" in line and "dry_run" in line and "distance" not in line.lower()


# ---------------------------------------------------------------- CLI without ROS


def run_cli(*args):
  env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
  return subprocess.run([sys.executable, str(HERE / "movement_trial.py"), *args],
                        capture_output=True, text=True, env=env, timeout=60)


def test_help_and_dry_run_work_without_ros_and_publish_nothing(tmp_path):
  helped = run_cli("--help")
  assert helped.returncode == 0 and "--execute" in helped.stdout
  dry = run_cli("forward", "--duration", "1.0", "--out", str(tmp_path))
  assert dry.returncode == 0, dry.stderr
  receipt = dry.stdout.strip().splitlines()[-1]
  assert "dry_run" in receipt
  files = list(tmp_path.glob("*.json"))
  assert len(files) == 1
  record = json.loads(files[0].read_text())
  assert record["mode"] == "dry-run" and record["published_commands"] == 0


def test_dry_run_rejects_over_envelope_before_touching_ros(tmp_path):
  bad = run_cli("forward", "--speed", "0.5", "--duration", "1.0", "--out", str(tmp_path))
  assert bad.returncode != 0 and "envelope" in (bad.stderr + bad.stdout).lower()
  assert not list(tmp_path.glob("*.json"))


def test_importing_the_module_does_not_import_rclpy():
  pure()
  assert "rclpy" not in sys.modules


# ---------------------------------------------------------------- fake ROS control boundary

OBS_TOPIC = "/neural_controller_default/observation"
STATUS_SERVICE = "/controller_manager/list_controllers"
PARAM_SERVICE = "/neural_controller_default/get_parameters"


class FloatArray(_fake_ros._Msg):
  def __init__(self, data):
    super().__init__(data=list(data), layout=None)


def make_trial(tmp_path, axis="forward", speed=0.2, duration=1.0, kick_mux=True, overrides=None):
  ros = _fake_ros.install(overrides)
  qos = types.ModuleType("rclpy.qos")
  qos.qos_profile_sensor_data = object()
  sys.modules["rclpy.qos"] = qos
  sys.modules["std_msgs.msg"].Float32MultiArray = FloatArray  # the observation message type
  if "run_through_kick_node" in sys.modules:
    del sys.modules["run_through_kick_node"]
  if "movement_trial" in sys.modules:
    del sys.modules["movement_trial"]
  mt = importlib.import_module("movement_trial")
  guard = importlib.import_module("run_through_kick_node")
  fixture = tmp_path / "stock.json"
  fixture.write_bytes(b"test-only stock policy fixture")
  guard.STOCK_POLICY_SHA256 = hashlib.sha256(fixture.read_bytes()).hexdigest()
  ros.controller_params["model_path"] = str(fixture)
  if kick_mux:
    ros.publishers["/cmd_vel"] += 1  # the kick trial mux
  node = mt.MovementTrialNode(mt.PulseRequest.build(axis, speed, duration), out_dir=tmp_path, clock=lambda: ros.t)
  node.stock.clock = lambda: ros.t
  node._test_emit_obs = True
  def emit():
    if node._test_emit_obs:
      observe(ros, *(node.request.twist if node.phase == "pulsing" else ZERO))
  node.create_timer(0.05, emit)
  return ros, node, mt


def ready(ros, node, idle_shot=True):
  ros.publish("/estop_controller/stop_status",
              _fake_ros.String(json.dumps({"latched": False, "fault": False})))
  if idle_shot:
    ros.publish("/run_through_kick/state", _fake_ros.String(json.dumps({"state": "idle", "reason": ""})))
  ros.tick(25)  # inventory/parameters answer; 1 s of listening for competing kick commands


def observe(ros, vx=0.0, vy=0.0, wz=0.0):
  data = [0.0] * 960
  data[6:9] = [vx, vy, wz]
  ros.publish(OBS_TOPIC, FloatArray(data))


def twists(ros, topic="/kick_cmd_vel"):
  return [(t.linear.x, t.linear.y, t.angular.z) for t in ros.published[topic]]


def test_constructing_the_node_publishes_nothing_and_does_not_start(tmp_path):
  ros, node, mt = make_trial(tmp_path)
  ready(ros, node)
  assert not ros.published["/kick_cmd_vel"] and not ros.published["/cmd_vel"]
  assert not ros.published["/emergency_stop"]
  assert node.phase == "waiting"


def test_pulse_runs_for_its_duration_then_zeros_and_finishes(tmp_path):
  ros, node, mt = make_trial(tmp_path, duration=1.0)
  ready(ros, node)
  assert node.try_start() is None
  for _ in range(30):  # 1.5 s at 20 Hz
    observe(ros, vx=node.request.twist[0] if node.phase == "pulsing" else 0.0)
    ros.tick(1)
  cmds = twists(ros)
  assert cmds[0] == (0.2, 0.0, 0.0)
  assert cmds[-1] == ZERO
  pulsing = [c for c in cmds if c != ZERO]
  assert 19 <= len(pulsing) <= 21
  assert node.phase == "finished" and node.record["outcome"] == "completed"
  assert node.record["summary"]["applied_matched_requested"] is True
  assert not ros.published["/emergency_stop"]


@pytest.mark.parametrize("why", ["no_gateway", "latched", "no_mux", "teleop_too", "stock_inactive",
                                 "student_active", "cap_too_low", "no_evidence", "shot_active",
                                 "other_kick_publisher"])
def test_start_is_refused_without_full_authority_and_publishes_nothing(tmp_path, why):
  ros, node, mt = make_trial(tmp_path, kick_mux=(why != "no_mux"))
  if why == "no_gateway":
    ros.tick(4)
  elif why == "latched":
    ready(ros, node)
    ros.publish("/estop_controller/stop_status",
                _fake_ros.String(json.dumps({"latched": True, "fault": False})))
  elif why == "teleop_too":
    ros.publishers["/cmd_vel"] += 1
    ready(ros, node)
  elif why == "stock_inactive":
    ros.controllers["neural_controller_default"] = "inactive"
    ready(ros, node)
  elif why == "student_active":
    ros.controllers["neural_controller_default"] = "inactive"
    ros.controllers["neural_controller_student"] = "active"
    ready(ros, node)
  elif why == "cap_too_low":
    ros.controller_params["max_vx_forward"] = 0.1
    ready(ros, node)
  elif why == "no_evidence":
    ros.unavailable.add(PARAM_SERVICE)
    ready(ros, node)
  elif why == "shot_active":
    ready(ros, node, idle_shot=False)
    ros.publish("/run_through_kick/state", _fake_ros.String(json.dumps({"state": "charge", "reason": ""})))
  elif why == "other_kick_publisher":
    ros.publishers["/kick_cmd_vel"] += 2  # the idle shot adapter plus something else
    ready(ros, node)
  else:
    ready(ros, node)
  problem = node.try_start()
  assert problem, why
  ros.tick(5)
  assert not ros.published["/kick_cmd_vel"], why
  assert not ros.published["/emergency_stop"], why
  assert node.phase == "waiting"


def test_recent_foreign_kick_command_refuses_start_until_quiet_again(tmp_path):
  # The shot adapter publishes its state only on change, so traffic on the kick input is
  # the direct evidence of a competing producer.
  ros, node, mt = make_trial(tmp_path)
  ready(ros, node)
  foreign = _fake_ros.Twist()
  foreign.linear.x = 0.3
  ros.publish("/kick_cmd_vel", foreign)
  assert "producer" in node.try_start()
  ros.tick(25)
  assert node.try_start() is None
  assert not ros.published["/emergency_stop"]  # a refusal while waiting is not a fault


def test_competing_same_command_publisher_is_detected_during_pulse(tmp_path):
  ros, node, mt = make_trial(tmp_path)
  ready(ros, node)
  assert node.try_start() is None
  # Equal payloads cannot be distinguished from our loopback by content alone.
  ros.publishers["/kick_cmd_vel"] += 1
  foreign = _fake_ros.Twist()
  foreign.linear.x = node.request.speed
  ros.publish("/kick_cmd_vel", foreign)
  ros.tick(5)
  assert node.phase == "finished"
  assert node.record["outcome"] == "stopped"
  assert "another node publishes /kick_cmd_vel" == node.record["reason"]
  assert twists(ros)[-1] == ZERO
  assert ros.published["/emergency_stop"]


def test_start_waits_a_full_listening_window_before_trusting_silence(tmp_path):
  ros, node, mt = make_trial(tmp_path)
  ros.publish("/estop_controller/stop_status",
              _fake_ros.String(json.dumps({"latched": False, "fault": False})))
  ros.tick(5)  # evidence is in, but only 0.25 s of listening
  assert "listening" in node.try_start()


@pytest.mark.parametrize("how", ["estop", "r3", "face_button", "latched", "teleop_appears",
                                 "stock_lost", "evidence_stale", "foreign_kick_cmd", "shot_activates"])
def test_active_pulse_stops_and_zeros_on_every_stop_path(tmp_path, how):
  ros, node, mt = make_trial(tmp_path, duration=3.0)
  ready(ros, node)
  assert node.try_start() is None
  for _ in range(5):
    observe(ros, vx=0.2)
    ros.tick(1)
  assert twists(ros)[-1] == (0.2, 0.0, 0.0)
  if how == "estop":
    ros.publish("/emergency_stop", _fake_ros.Empty())
  elif how == "r3":
    ros.publish("/joy", _fake_ros.Joy(buttons=[0] * 12 + [1]))
  elif how == "face_button":
    ros.publish("/joy", _fake_ros.Joy(buttons=[0, 1, 0, 0]))
  elif how == "latched":
    ros.publish("/estop_controller/stop_status",
                _fake_ros.String(json.dumps({"latched": True, "fault": False})))
  elif how == "teleop_appears":
    ros.publishers["/cmd_vel"] += 1
    ros.tick(25)
  elif how == "stock_lost":
    ros.controllers["neural_controller_default"] = "inactive"
    ros.tick(15)
  elif how == "foreign_kick_cmd":
    foreign = _fake_ros.Twist()
    foreign.linear.x = 0.45
    ros.publish("/kick_cmd_vel", foreign)
  elif how == "shot_activates":
    ros.publish("/run_through_kick/state", _fake_ros.String(json.dumps({"state": "align", "reason": ""})))
  else:
    ros.held_services.add(STATUS_SERVICE)
    ros.tick(15)
  assert twists(ros)[-1] == ZERO
  assert node.phase == "finished" and node.record["outcome"] == "stopped"
  count = len(ros.published["/kick_cmd_vel"])
  ros.tick(20)
  assert len(ros.published["/kick_cmd_vel"]) == count  # never resumes
  if how in ("r3", "face_button", "teleop_appears", "stock_lost", "evidence_stale",
             "foreign_kick_cmd", "shot_activates"):
    assert len(ros.published["/emergency_stop"]) == 1  # authority loss requests the shared stop
  else:
    assert not ros.published["/emergency_stop"]  # the operator/gateway already owns the stop


def test_pulse_never_exceeds_max_duration_even_if_clock_jumps(tmp_path):
  ros, node, mt = make_trial(tmp_path, duration=3.0)
  ready(ros, node)
  assert node.try_start() is None
  ros.tick(2)
  ros.t += 10.0
  ros.tick(1)
  assert twists(ros)[-1] == ZERO and node.phase == "finished"


def test_finish_zeroes_kick_input_and_robot_topic_only_when_stock_is_known(tmp_path):
  ros, node, mt = make_trial(tmp_path, duration=0.5)
  ready(ros, node)
  assert node.try_start() is None
  ros.tick(15)
  assert node.phase == "finished"
  node.finish(sleep=lambda s: None)
  assert twists(ros)[-3:] == [ZERO, ZERO, ZERO]
  assert twists(ros, "/cmd_vel")[-3:] == [ZERO, ZERO, ZERO]
  (tmp_path / "b").mkdir()
  ros2, node2, _ = make_trial(tmp_path / "b")
  ros2.tick(1)  # never had evidence
  node2.finish(sleep=lambda s: None)
  assert not ros2.published["/cmd_vel"] and not ros2.published["/kick_cmd_vel"]


def test_finish_during_an_active_pulse_requests_emergency_stop_once(tmp_path):
  ros, node, mt = make_trial(tmp_path, duration=3.0)
  ready(ros, node)
  assert node.try_start() is None
  ros.tick(3)
  node.finish(sleep=lambda s: None)
  assert len(ros.published["/emergency_stop"]) == 1
  assert twists(ros)[-1] == ZERO and node.record["outcome"] == "stopped"


def test_record_written_on_finish_includes_imu_when_seen(tmp_path):
  ros, node, mt = make_trial(tmp_path, duration=0.5)
  ready(ros, node)
  ros.publish("/imu_sensor_broadcaster/imu", _fake_ros.Imu(yaw=0.3))
  assert node.try_start() is None
  ros.publish("/imu_sensor_broadcaster/imu", _fake_ros.Imu(yaw=0.4))
  ros.tick(15)
  node.finish(sleep=lambda s: None)
  files = list(tmp_path.glob("movement-*.json"))
  assert len(files) == 1
  record = json.loads(files[0].read_text())
  assert record["imu"]["yaw_start_rad"] == pytest.approx(0.3)
  assert record["imu"]["yaw_end_rad"] == pytest.approx(0.4)
  assert record["imu"]["note"].startswith("sensor output, not external truth")
  assert record["operator_observation"].startswith("pending")


def test_no_fresh_observation_refuses_start(tmp_path):
  ros, node, mt = make_trial(tmp_path)
  node._test_emit_obs = False
  ready(ros, node)
  assert "observation" in node.try_start()
  assert not ros.published["/kick_cmd_vel"]


def test_lost_observation_stops_the_diagnostic_pulse(tmp_path):
  ros, node, mt = make_trial(tmp_path, duration=3.0)
  ready(ros, node)
  assert node.try_start() is None
  node._test_emit_obs = False
  ros.tick(10)
  assert node.phase == "finished"
  assert "observation stale" in node.record["reason"]
  assert twists(ros)[-1] == ZERO


def test_execute_without_runtime_identity_refuses_before_ros_import(tmp_path):
  mt = pure()
  assert mt.main(["forward", "--execute", "--out", str(tmp_path)]) == 2
  assert "rclpy" not in sys.modules


def test_refused_competition_does_not_send_shutdown_commands(tmp_path):
  ros, node, mt = make_trial(tmp_path)
  ready(ros, node)
  ros.publishers["/kick_cmd_vel"] += 1
  assert node.try_start()
  node.finish(sleep=lambda _: None)
  assert not ros.published["/kick_cmd_vel"]
  assert not ros.published["/cmd_vel"]


def test_sprint_candidate_profile_is_explicit_and_bounded():
  mt = pure()
  with pytest.raises(ValueError):
    mt.PulseRequest.build("forward", 0.7, 1)
  assert mt.PulseRequest.build("forward", 0.7, 1, "sprint").twist == (0.7, 0.0, 0.0)
  assert mt.PulseRequest.build("left", 0.5, 1, "sprint").twist == (0.0, 0.5, 0.0)
  with pytest.raises(ValueError):
    mt.PulseRequest.build("forward", 1.01, 1, "sprint")


def test_native_heading_correction_is_reported_without_falsely_failing_translation():
  mt = pure()
  samples = [mt.Sample(t=0.5, requested=(0.2,0,0), applied=(0.2,0,0.1))]
  report = mt.summarize(samples, 0, 1, 1.1, compare_indices=(0,1))
  assert report["applied_matched_requested"] is True
  assert report["max_abs_requested_minus_applied"]["wz"] == 0.1
  assert report["comparison_axes"] == ["vx", "vy"]
