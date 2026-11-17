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


@pytest.mark.parametrize("axis", ["left", "right"])
@pytest.mark.parametrize("profile", ["walk", "sprint"])
def test_lateral_calibration_requires_explicit_opt_in_and_keeps_normal_limit(axis, profile):
  mt = pure()
  assert mt.PulseRequest.build(axis, None, 3, profile).max_duration_s == 3
  with pytest.raises(ValueError, match="envelope"):
    mt.PulseRequest.build(axis, None, 3.001, profile)
  request = mt.PulseRequest.build(axis, None, 20, profile, lateral_calibration=True)
  assert request.duration_s == request.max_duration_s == 20
  assert request.speed == 0.15 and request.lateral_calibration is True


@pytest.mark.parametrize("axis,speed,duration,opt_in", [
  ("left", 0.15, 20.001, True), ("right", 0.15, float("inf"), True),
  ("left", 0.15, float("nan"), True), ("left", 0.15, 0, True),
  ("left", 0.151, 20, True), ("forward", 0.15, 20, True),
  ("turn_right", 0.15, 20, True), ("left", 0.15, 20, "true"),
  ("left", 0.15, 20, 1), ("left", 0.15, 20, False),
])
def test_lateral_calibration_rejects_unbounded_or_malformed_requests(axis, speed, duration, opt_in):
  mt = pure()
  with pytest.raises(ValueError):
    mt.PulseRequest.build(axis, speed, duration, "sprint", lateral_calibration=opt_in)


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


def test_one_shot_lateral_calibration_cli_is_explicit_and_recorded(tmp_path):
  args = ["left", "--profile", "sprint", "--duration", "12.2", "--out", str(tmp_path)]
  assert run_cli(*args).returncode == 2
  assert not list(tmp_path.glob("*.json"))
  dry = run_cli(*args, "--lateral-calibration")
  assert dry.returncode == 0, dry.stderr
  record = json.loads(next(tmp_path.glob("*.json")).read_text())
  assert record["request"]["lateral_calibration"] is True
  assert record["request"]["duration_s"] == 12.2
  assert record["request"]["profile"] == "sprint"
  assert record["published_commands"] == 0
  assert record["interpretation"]["command_integral_is_distance"] is False
  bad = run_cli("--session", "--execute", "--lateral-calibration")
  assert bad.returncode == 2 and "each prepare" in bad.stderr


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


def make_trial(tmp_path, axis="forward", speed=0.2, duration=1.0, kick_mux=True, overrides=None,
               lateral_calibration=False):
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
  node = mt.MovementTrialNode(mt.PulseRequest.build(axis, speed, duration,
                              lateral_calibration=lateral_calibration), out_dir=tmp_path, clock=lambda: ros.t)
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


@pytest.mark.parametrize("axis,duration", [("left", 12.2), ("right", 20.0)])
def test_lateral_calibration_runs_continuously_then_confirms_zero(tmp_path, axis, duration):
  ros, node, mt = make_trial(tmp_path, axis=axis, speed=0.15, duration=duration, lateral_calibration=True)
  ready(ros, node)
  assert node.try_start() is None
  ros.tick(80)
  assert node.phase == "pulsing" and ros.t - node.started > 3
  ros.tick(int(duration / 0.05) + 10)
  assert node.phase == "finished" and node.record["outcome"] == "completed"
  assert duration <= node.zero_sent_at - node.started <= duration + 0.051
  mt.finish_trial(node, {}, lambda _: True, session=True)
  assert node.record["command_verification"]["passed"]
  assert node.record["summary"]["zero_confirmed_in_observation_s"] is not None
  commands = twists(ros)
  first_zero = commands.index(ZERO)
  assert all(command == node.request.twist for command in commands[:first_zero])
  assert all(command == ZERO for command in commands[first_zero:])
  assert not ros.published["/emergency_stop"]


@pytest.mark.parametrize("cause", ["feedback_stale", "emergency", "joystick", "stock_lost"])
def test_lateral_calibration_still_stops_early_after_three_seconds(tmp_path, cause):
  ros, node, mt = make_trial(tmp_path, axis="left", speed=0.15, duration=20, lateral_calibration=True)
  ready(ros, node)
  assert node.try_start() is None
  ros.tick(80)
  assert node.phase == "pulsing"
  interrupted_at = ros.t
  if cause == "feedback_stale":
    assert mt.OBS_MAX_AGE_S == 0.2
    node._test_emit_obs = False
  elif cause == "emergency":
    ros.publish("/emergency_stop", _fake_ros.Empty())
  elif cause == "joystick":
    ros.publish("/joy", _fake_ros.Joy(buttons=[0] * 12 + [1]))
  else:
    ros.controllers["neural_controller_default"] = "inactive"
  ros.tick(25)
  assert node.phase == "finished" and node.record["outcome"] == "stopped"
  assert node.zero_sent_at - node.started < 20
  assert twists(ros)[-1] == ZERO
  if cause == "feedback_stale":
    assert node.record["reason"] == "stock policy observation stale"
    assert node.zero_sent_at - interrupted_at <= 0.251
  if cause in ("emergency", "joystick"):
    assert node.zero_sent_at == interrupted_at
  assert len(ros.published["/emergency_stop"]) == (0 if cause == "emergency" else 1)
  count = len(twists(ros))
  ros.tick(20)
  assert len(twists(ros)) == count


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


@pytest.mark.parametrize("identity_change_after", [None, 0.23])
def test_execute_bounds_process_scans_without_delaying_identity_loss_stop(
    tmp_path, monkeypatch, identity_change_after):
  ros, node, mt = make_trial(tmp_path, duration=1.0)
  ready(ros, node)
  monkeypatch.syspath_prepend(str(HERE.parents[2]))
  from foundation.hardware import runtime_identity
  guard = sys.modules["run_through_kick_node"]
  runtime = {"manager_pid": 123, "manager_start_ticks": 456,
             "files": {"policy": {"sha256": guard.STOCK_POLICY_SHA256}}}
  preflight = {"manifest_path": "selected.json"}
  full_checks, identity_checks, callbacks = [], [], []
  next_tick = ros.t + 0.05

  def verify_loaded(receipt):
    full_checks.append(receipt)
    return runtime

  def same_manager(evidence):
    assert evidence is runtime
    identity_checks.append((node.phase, ros.t))
    ros.t += 0.02  # measured Pi scan cost; this blocks callback processing
    return (identity_change_after is None or node.started is None or
            ros.t - node.started < identity_change_after)

  def spin_once(actual_node, timeout_sec):
    nonlocal next_tick
    assert actual_node is node
    ros.t += 0.001  # many ready sensor callbacks between command ticks
    callbacks.append(ros.t)
    pending, ros.pending = ros.pending, []
    for future in pending:
      future.resolve()
    observe(ros, *(node.request.twist if node.phase == "pulsing" else ZERO))
    if ros.t >= next_tick:
      node.on_tick()
      next_tick = ros.t + 0.05

  monkeypatch.setattr(runtime_identity, "read_json", lambda path:
                      {"trial_profile": "walk"} if str(path) == "selected.json" else preflight)
  monkeypatch.setattr(runtime_identity, "verify_loaded", verify_loaded)
  monkeypatch.setattr(runtime_identity, "same_manager", same_manager)
  monkeypatch.setattr(mt.time, "monotonic", lambda: ros.t)
  monkeypatch.setattr(sys.modules["rclpy"], "spin_once", spin_once, raising=False)
  monkeypatch.setattr(mt, "_node_class", lambda: lambda *a, **kw: node)
  finish = node.finish
  monkeypatch.setattr(node, "finish", lambda: finish(sleep=lambda _: None))
  args = mt.parse_args(["forward", "--execute", "--duration", "1",
                        "--out", str(tmp_path), "--runtime-receipt", "preflight.json"])

  code = mt.run_execute(node.request, args)
  checks = [t for phase, t in identity_checks if phase == "pulsing"]
  assert len(full_checks) == 2  # initial and immediately before starting
  assert identity_checks[-1][0] == "finished"  # final check is never throttled
  assert 2 <= len(checks) <= 5
  assert all(b - a >= mt.OWNER_CHECK_S for a, b in zip(checks, checks[1:]))
  assert len(callbacks) > 10 * len(checks)
  assert twists(ros)[-1] == ZERO
  if identity_change_after is None:
    assert code == 0
    assert node.record["command_verification"]["passed"]
    assert not ros.published["/emergency_stop"]
  else:
    assert code == 4
    assert node.record["reason"] == "native manager changed during pulse"
    # Cadence plus one callback and the bounded check's own execution time.
    delay = node.zero_sent_at - node.started - identity_change_after
    assert 0 <= delay <= mt.OWNER_CHECK_S + 0.001 + 0.02
    assert len(ros.published["/emergency_stop"]) == 1


def test_refused_competition_does_not_send_shutdown_commands(tmp_path):
  ros, node, mt = make_trial(tmp_path)
  ready(ros, node)
  ros.publishers["/kick_cmd_vel"] += 1
  assert node.try_start()
  node.finish(sleep=lambda _: None)
  assert not ros.published["/kick_cmd_vel"]
  assert not ros.published["/cmd_vel"]


def session_fixture(tmp_path, monkeypatch):
  ros, node, mt = make_trial(tmp_path, duration=1.0)
  ready(ros, node)
  inbox = mt.SessionInbox(clock=lambda: ros.t)
  next_tick = ros.t + 0.05

  def spin_once(actual_node, timeout_sec):
    nonlocal next_tick
    assert actual_node is node
    ros.t += 0.01
    pending, ros.pending = ros.pending, []
    for future in pending:
      future.resolve()
    ros.publish("/estop_controller/stop_status", _fake_ros.String(
      json.dumps({"latched": False, "fault": False})))
    observe(ros, *(node.request.twist if node.phase == "pulsing" else ZERO))
    if ros.t >= next_tick:
      node.on_tick()
      next_tick = ros.t + 0.05

  return ros, node, mt, inbox, spin_once


def test_session_reuses_node_for_two_explicit_trials_and_drains_old_own_loopbacks(tmp_path, monkeypatch):
  ros, node, mt, inbox, spin_once = session_fixture(tmp_path, monkeypatch)
  events, paths, pending_loopbacks = [], [], []
  starts = 0
  publish = node.cmd_pub.publish

  def capture(message):
    publish(message)
    pending_loopbacks.append(message)

  node.cmd_pub.publish = capture

  def emit(event):
    nonlocal starts
    events.append(event)
    if event["event"] == "prepared":
      inbox.put({"command": "run", "token": event["token"]})
      inbox.put({"command": "run", "token": event["token"]})  # consumed token cannot replay
    elif event["event"] == "started":
      starts += 1
      inbox.put({"command": "prepare", "axis": "forward", "speed": 999})
    elif event["event"] == "trial_complete":
      paths.append(Path(event["file"]))
      if len(paths) == 1:
        assert len(pending_loopbacks) >= 5  # last pulse, completion zero, three cleanup zeros
        inbox.put({"command": "prepare", "axis": "turn_left", "duration": 0.35})
      else:
        inbox.put({"command": "quit"})

  def spin(actual_node, timeout_sec):
    spin_once(actual_node, timeout_sec)
    if node.phase == "pulsing":
      # Preserve trial one's final messages until trial two is already active.
      count = len(pending_loopbacks) if starts == 2 else max(0, len(pending_loopbacks) - 1)
      for message in pending_loopbacks[:count]:
        node.on_kick_cmd(message)
      del pending_loopbacks[:count]

  inbox.put({"command": "prepare", "axis": "forward", "duration": 0.3})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin) == 0
  assert starts == 2 and len(paths) == 2
  assert node.foreign_cmd_at is None
  assert not ros.published["/emergency_stop"]
  records = [json.loads(path.read_text()) for path in paths]
  assert [record["request"]["axis"] for record in records] == ["forward", "turn_left"]
  for record in records:
    assert record["command_verification"]["passed"]
    assert record["samples"][0]["t"] == pytest.approx(0)
    assert record["client_identity"]["sha256"] == hashlib.sha256(Path(mt.__file__).read_bytes()).hexdigest()
  assert sum(event["event"] == "refused" for event in events) >= 4


def test_session_lateral_calibration_requires_fresh_opt_in_and_one_use_token(tmp_path, monkeypatch):
  ros, node, mt, inbox, spin_once = session_fixture(tmp_path, monkeypatch)
  events = []
  completed = 0

  def emit(event):
    nonlocal completed
    events.append(event)
    if event["event"] == "prepared":
      inbox.put({"command": "run", "token": event["token"]})
      inbox.put({"command": "run", "token": event["token"]})
    elif event["event"] == "trial_complete":
      completed += 1
      assert event["command_verification"]["passed"]
      if completed == 1:
        assert event["summary"]["elapsed_s"] >= 4
        inbox.put({"command": "prepare", "axis": "right", "duration": 4})
      else:
        inbox.put({"command": "quit"})
    elif event["event"] == "refused" and event["command"] == "prepare":
      assert "envelope" in event["reason"]
      inbox.put({"command": "prepare", "axis": "right", "duration": 0.3})

  inbox.put({"command": "prepare", "axis": "left", "duration": 4, "lateral_calibration": True})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin_once) == 0
  started = [event["request"] for event in events if event["event"] == "started"]
  assert [request["lateral_calibration"] for request in started] == [True, False]
  assert completed == 2
  assert sum(event["event"] == "refused" and event["command"] == "run" for event in events) == 2
  assert twists(ros)[-1] == ZERO


@pytest.mark.parametrize("cause", ["stop", "eof", "identity", "authority"])
def test_session_latches_stop_disconnect_or_fault_and_does_not_play_queued_runs(tmp_path, monkeypatch, cause):
  import io
  ros, node, mt, inbox, spin_once = session_fixture(tmp_path, monkeypatch)
  events, identity_valid = [], True
  started_at = None

  def emit(event):
    nonlocal identity_valid, started_at
    events.append(event)
    if event["event"] == "prepared":
      inbox.put({"command": "run", "token": event["token"]})
    elif event["event"] == "started":
      started_at = ros.t
      for _ in range(100):
        inbox.put({"command": "status"})
      inbox.put({"command": "prepare", "axis": "forward"})
      if cause == "eof":
        inbox.read(io.StringIO(""))
      elif cause == "stop":
        inbox.put({"command": "stop"})
      elif cause == "identity":
        identity_valid = False
      else:
        ros.controllers["neural_controller_default"] = "inactive"

  inbox.put({"command": "prepare", "axis": "forward", "duration": 1})
  code = mt.run_session(node, {}, lambda _: identity_valid, inbox=inbox, emit=emit, spin_once=spin_once)
  assert code == 4
  assert sum(event["event"] == "started" for event in events) == 1
  assert sum(event["event"] == "trial_complete" for event in events) == 1
  assert node.record["outcome"] == "stopped"
  assert twists(ros)[-1] == ZERO
  assert len(ros.published["/emergency_stop"]) == 1
  if cause in ("stop", "eof"):
    assert node.zero_sent_at - started_at <= 0.011
    assert not any(event["event"] == "status" for event in events)
  if cause == "eof":
    assert node.record["reason"] == "session stdin closed"


def test_session_idle_disconnect_publishes_nothing(tmp_path, monkeypatch):
  ros, node, mt, inbox, spin_once = session_fixture(tmp_path, monkeypatch)
  inbox.put({"command": "quit", "reason": "session stdin closed"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=lambda _: None,
                        spin_once=spin_once) == 0
  assert not ros.published["/kick_cmd_vel"]
  assert not ros.published["/cmd_vel"]
  assert not list(tmp_path.glob("movement-*.json"))


@pytest.mark.parametrize("stop_kind", ["emergency", "joystick"])
def test_session_idle_stop_cannot_be_rearmed_by_prepare_and_run(tmp_path, monkeypatch, stop_kind):
  ros, node, mt, inbox, spin_once = session_fixture(tmp_path, monkeypatch)
  events = []

  def emit(event):
    events.append(event)
    if event["event"] == "prepared":
      inbox.put({"command": "run", "token": event["token"]})
    elif event["event"] == "trial_complete":
      if stop_kind == "emergency":
        node.on_emergency(None)
      else:
        node.on_joy(_fake_ros.Joy(buttons=[1]))
      inbox.put({"command": "prepare", "axis": "forward", "duration": 0.3})

  inbox.put({"command": "prepare", "axis": "forward", "duration": 0.3})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin_once) == 3
  assert sum(event["event"] == "started" for event in events) == 1
  assert sum(event["event"] == "trial_complete" for event in events) == 1
  assert any(event["event"] == "fault" for event in events)


def test_session_run_trigger_expires_behind_backlog_without_moving(tmp_path, monkeypatch):
  ros, node, mt, inbox, spin_once = session_fixture(tmp_path, monkeypatch)
  events = []

  def emit(event):
    events.append(event)
    if event["event"] == "prepared":
      for _ in range(30):
        inbox.put({"command": "status"})
      inbox.put({"command": "run", "token": event["token"]})
    elif event["event"] == "refused":
      inbox.put({"command": "quit"})

  inbox.put({"command": "prepare", "axis": "forward", "duration": 0.3})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin_once) == 0
  assert any("expired" in event.get("reason", "") for event in events)
  assert not ros.published["/kick_cmd_vel"]


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


# ---------------------------------------------------------------- feedback turns

@pytest.mark.parametrize("fields", [
  {"turn_angle_rad": 4 * math.pi}, {"heading_rad": float("nan")},
  {"heading_rad": 0, "turn_angle_rad": 1}, {"heading_rad": 0, "axis": "turn_left"},
  {"heading_rad": 0, "lateral_calibration": False}, {"heading_rad": 0, "duration": 1},
  {"turn_angle_rad": 1, "speed": 0.351}, {"turn_angle_rad": 1, "speed": 0},
  {"turn_angle_rad": 1, "timeout_s": 20.01}, {"turn_angle_rad": 1, "timeout_s": float("inf")},
])
def test_heading_request_rejects_ambiguous_or_unbounded_inputs(fields):
  mt = pure()
  with pytest.raises(ValueError):
    mt.session_request(fields, "walk")


def test_heading_shortest_turn_wrap_and_exact_antipodes_are_deterministic():
  mt = pure()
  assert mt.wrap_heading(-math.pi) == mt.wrap_heading(math.pi) == math.pi
  req = mt.HeadingRequest.build(-3.1, relative=False)
  control = mt.HeadingControl.start(req, 3.1)
  assert 0 < control.update(3.1, 0)[2] < 0.35
  assert control.update(-2.8, 100_000_000)[2] < 0  # shortest correction after overshoot
  for angle in (-math.pi, math.pi):
    control = mt.HeadingControl.start(mt.HeadingRequest.build(angle, relative=True), 0)
    assert control.update(0, 0) == (0, 0, 0.35)


def test_heading_antipodes_choose_left_at_nonzero_yaw_without_widening_nearby_ties():
  import random
  mt = pure()
  rng = random.Random(1)
  for _ in range(10_000):
    yaw = rng.uniform(-math.pi, math.pi)
    for angle in (-math.pi, math.pi):
      control = mt.HeadingControl.start(mt.HeadingRequest.build(angle, relative=True), yaw)
      assert control.update(yaw, 0)[2] == 0.35
    for angle, sign in ((math.pi - 1e-12, 1), (-math.pi + 1e-12, -1)):
      control = mt.HeadingControl.start(mt.HeadingRequest.build(angle, relative=True), yaw)
      assert control.update(yaw, 0)[2] * sign > 0


def test_relative_heading_keeps_target_but_uses_new_shortest_route_after_disturbance():
  mt = pure()
  control = mt.HeadingControl.start(mt.HeadingRequest.build(3.1, relative=True), 0)
  assert control.update(0, 0)[2] > 0
  assert control.update(-0.1, 50_000_000)[2] < 0
  assert control.target_rad == pytest.approx(3.1)
  assert abs(control.error_rad) < math.pi
  # Continuing left would now travel 3.2 rad, so reversing is the shorter route.
  assert abs(control.error_rad) < 3.1 - (-0.1)


@pytest.mark.parametrize("delay", [0.02, 0.05, 0.10])
@pytest.mark.parametrize("disturbance", [False, True])
def test_heading_verification_aligns_healthy_delayed_varying_commands(delay, disturbance):
  mt = pure()
  request = mt.HeadingRequest.build(1, relative=True)
  control = mt.HeadingControl.start(request, 0)
  yaw, disturbed = 0.0, False
  command = control.update(yaw, 0)
  samples = [mt.Sample(0, command, ZERO, -0.01)]
  for tick in range(1, 1500):
    t = tick / 100
    yaw += command[2] * 0.01
    if tick % 5:
      continue
    if disturbance and not disturbed and control.within_since_ros_ns is not None:
      yaw += 0.12
      disturbed = True
    command = control.update(yaw, tick * 10_000_000)
    receipt_at = t - 0.005
    prior = [s for s in samples if s.t <= receipt_at - delay]
    applied = prior[-1].requested if prior else ZERO
    samples.append(mt.Sample(t, command, applied, receipt_at))
    if control.completed:
      break
  assert control.completed and (disturbed or not disturbance)
  result = mt.summarize(samples, 0, t, t + delay, heading_feedback=True)
  assert result["applied_matched_requested"] is True
  alignment = result["feedback_alignment"]
  assert alignment["matched_observations"] > 10
  assert alignment["unmatched_observations"] == 0
  assert alignment["feedback_window_s"] == 0.2 and alignment["component_tolerance"] == 0.02
  assert 0 <= alignment["accepted_command_age_s"]["min"] <= alignment["accepted_command_age_s"]["max"] <= 0.2
  if disturbance or delay == 0.1:
    assert result["max_abs_requested_minus_applied"]["wz"] > mt.MATCH_TOL
    assert mt.summarize(samples, 0, t, t + delay)["applied_matched_requested"] is False


@pytest.mark.parametrize("wrong", [(0.1, 0, 0.3), (0, 0.1, 0.3), (0, 0, -0.3),
                                   (0, 0, 0.1), ZERO])
def test_heading_verification_rejects_wrong_axis_magnitude_reverse_and_persistent_zero(wrong):
  mt = pure()
  samples = [mt.Sample(i / 20, (0, 0, 0.3), wrong, i / 20 - 0.01) for i in range(11)]
  result = mt.summarize(samples, 0, 0.5, 0.55, heading_feedback=True)
  assert result["applied_matched_requested"] is False
  assert result["feedback_alignment"]["unmatched_observations"] > 0


@pytest.mark.parametrize("cause", ["stale_receipt", "missing_receipt", "missing_observation", "old_command"])
def test_heading_verification_rejects_stale_missing_or_overage_command_evidence(cause):
  mt = pure()
  samples = [mt.Sample(i / 20, (0, 0, 0.3), (0, 0, 0.3), i / 20 - 0.01) for i in range(11)]
  if cause == "stale_receipt":
    samples[-1] = mt.Sample(0.5, (0, 0, 0.3), (0, 0, 0.3), 0.29)
  elif cause == "missing_receipt":
    samples[-1] = mt.Sample(0.5, (0, 0, 0.3), (0, 0, 0.3))
  elif cause == "missing_observation":
    samples[-1] = mt.Sample(0.5, (0, 0, 0.3), None)
  else:
    samples[0] = mt.Sample(0, (0, 0, -0.3), ZERO, -0.01)
    samples[-1] = mt.Sample(0.5, (0, 0, 0.3), (0, 0, -0.3), 0.49)
  result = mt.summarize(samples, 0, 0.5, 0.55, heading_feedback=True)
  assert result["applied_matched_requested"] is False
  alignment = result["feedback_alignment"]
  key = ("stale_observation_ticks" if cause == "stale_receipt" else
         "unmatched_observations" if cause == "old_command" else "missing_observation_ticks")
  assert alignment[key] == 1


def test_heading_verification_cannot_use_future_stop_zero_or_reused_feedback_as_new_evidence():
  mt = pure()
  samples = [mt.Sample(i / 20, (0, 0, 0.3), (0, 0, 0.3), i / 20 - 0.01) for i in range(10)]
  samples.append(mt.Sample(0.5, ZERO, ZERO, 0.49))  # zero observed before the first zero request
  result = mt.summarize(samples, 0, 0.5, 0.55, heading_feedback=True)
  assert result["feedback_alignment"]["unmatched_observations"] == 1
  assert result["applied_matched_requested"] is False
  samples = samples[:-1]
  samples.append(mt.Sample(0.5, ZERO, (0, 0, 0.3), samples[-1].applied_t))
  result = mt.summarize(samples, 0, 0.5, None, heading_feedback=True)
  assert result["applied_matched_requested"] is True
  assert result["feedback_alignment"]["matched_observations"] == 5
  record = {"outcome": "completed", "summary": result}
  mt.finalize_verification(record, True)
  assert record["command_verification"]["passed"] is False  # stop zero still unconfirmed
  result["zero_confirmed_in_observation_s"] = 0.05
  mt.finalize_verification(record, True)
  assert record["command_verification"]["passed"] is True
  record["outcome"] = "stopped"
  mt.finalize_verification(record, True)
  assert record["command_verification"]["passed"] is False


def test_heading_verification_persistent_zero_cannot_hide_brief_motion_inside_hold():
  mt = pure()
  samples = [mt.Sample(i / 20, ZERO, ZERO, i / 20 - 0.01) for i in range(15)]
  samples[8] = mt.Sample(0.4, (0, 0, 0.1), ZERO, 0.39)
  result = mt.summarize(samples, 0, 0.7, 0.75, heading_feedback=True)
  alignment = result["feedback_alignment"]
  assert alignment["unmatched_observations"] == 0  # zeros match nearby hold commands
  assert alignment["nonzero_command_requested"] is True
  assert alignment["nonzero_feedback_observations"] == 0
  assert result["applied_matched_requested"] is False


def test_heading_verification_accepts_short_confirmed_motion_before_settle_then_zero_hold():
  mt = pure()
  samples = [mt.Sample(0, (0, 0, 0.05), ZERO, -0.01),
             mt.Sample(0.05, (0, 0, 0.05), (0, 0, 0.05), 0.04),
             mt.Sample(0.1, ZERO, (0, 0, 0.05), 0.09)]
  samples.extend(mt.Sample(i / 20, ZERO, ZERO, i / 20 - 0.01) for i in range(3, 10))
  result = mt.summarize(samples, 0, 0.45, 0.5, heading_feedback=True)
  assert result["applied_matched_requested"] is True
  assert result["feedback_alignment"]["nonzero_feedback_observations"] == 2


def test_heading_dwell_requires_new_feedback_and_restarts_after_leaving_tolerance():
  mt = pure()
  control = mt.HeadingControl.start(mt.HeadingRequest.build(0, relative=False), 0)
  assert control.update(0.04, 1_000_000_000) == ZERO
  for _ in range(100):
    control.update(0.04, 1_000_000_000)
  assert not control.completed and control.feedback_samples == 1
  control.update(0.06, 1_290_000_000)
  assert control.within_since_ros_ns is None
  control.update(0.02, 1_300_000_000)
  control.update(-0.02, 1_590_000_000)
  assert not control.completed
  control.update(0.01, 1_610_000_000)
  assert control.completed


def test_heading_dwell_never_counts_time_before_execution():
  mt = pure()
  control = mt.HeadingControl.start(mt.HeadingRequest.build(0, relative=False), 0,
                                   started_ros_ns=1_000_000_000)
  control.update(0, 810_000_000)
  control.update(0, 1_150_000_000)
  assert not control.completed
  assert control.within_since_ros_ns == 1_000_000_000
  control.update(0, 1_310_000_000)
  assert control.completed


def heading_imu(ros, yaw, *, capture=None):
  msg = _fake_ros.Imu(yaw=yaw)
  captured_ns = round((ros.t if capture is None else capture) * 1e9)
  msg.header = _fake_ros.Header()
  msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(captured_ns, 1_000_000_000)
  ros.publish("/imu_sensor_broadcaster/imu", msg)
  return msg


def heading_trial(tmp_path, **fields):
  ros, node, mt = make_trial(tmp_path)
  ready(ros, node)
  node.prepare(mt.session_request(fields or {"turn_angle_rad": 1}, "walk"))
  node._test_emit_obs = False
  observe(ros)
  return ros, node, mt


def test_heading_anchors_relative_target_at_execution_not_prepare(tmp_path):
  ros, node, mt = heading_trial(tmp_path, turn_angle_rad=0.5)
  heading_imu(ros, 0.2)
  heading_imu(ros, 0.7, capture=ros.t + 0.01)  # future captures cannot supply start yaw
  assert node.try_start() is not None
  ros.t += 0.02
  heading_imu(ros, 0.9)
  observe(ros)
  assert node.try_start() is None
  assert node.heading.target_rad == pytest.approx(1.4)
  assert twists(ros)[0] == (0, 0, 0.35)


@pytest.mark.parametrize("phase", ["waiting", "finished"])
@pytest.mark.parametrize("bad_sample", ["invalid", "reordered"])
def test_bad_idle_imu_refuses_readiness_without_latching_session_stop(tmp_path, phase, bad_sample):
  ros, node, mt = heading_trial(tmp_path, heading_rad=0)
  heading_imu(ros, 0)
  if phase == "finished":
    assert node.try_start() is None
    for _ in range(8):
      ros.t += 0.05
      pending, ros.pending = ros.pending, []
      for future in pending:
        future.resolve()
      heading_imu(ros, 0)
      observe(ros)
      node.on_tick()
    assert node.record["outcome"] == "completed"
    mt.finish_trial(node, {}, lambda _: True, session=True)
  commands_before = len(twists(ros))
  if bad_sample == "reordered":
    heading_imu(ros, 0, capture=ros.t - 0.01)
  else:
    msg = _fake_ros.Imu()
    msg.orientation.w = 0
    node.on_imu(msg)
  assert node.imu_latest is None and node.stop_requested is None
  assert not ros.published["/emergency_stop"]
  assert len(twists(ros)) == commands_before
  if phase == "finished":
    node.prepare(mt.HeadingRequest.build(0.2, relative=True))
  assert "IMU" in node.try_start()
  ros.t += 0.02
  heading_imu(ros, 0)
  observe(ros)
  assert node.try_start() is None


def test_imu_snapshot_is_consistent_whenever_the_callback_releases_its_lock(tmp_path):
  ros, node, mt = heading_trial(tmp_path)
  heading_imu(ros, 0)
  snapshots, original = [], node.lock
  class ObservedLock:
    def __enter__(self):
      original.acquire()
    def __exit__(self, *args):
      snapshots.append((node.imu_source_stamp_ns, node.imu_latest))
      original.release()
  node.lock = ObservedLock()
  ros.t += 0.05
  heading_imu(ros, 0.5)
  assert snapshots
  assert all(stamp == round(ros.t * 1e9) and latest[0] == pytest.approx(0.5)
             for stamp, latest in snapshots)


@pytest.mark.parametrize("cause", ["absent", "old_arrival", "old_capture", "invalid_quaternion"])
def test_heading_requires_fresh_valid_captured_yaw_before_any_publication(tmp_path, cause):
  ros, node, mt = heading_trial(tmp_path)
  if cause == "old_arrival":
    heading_imu(ros, 0)
    ros.t += 0.21
  elif cause == "old_capture":
    heading_imu(ros, 0, capture=ros.t - 0.3)
  elif cause == "invalid_quaternion":
    msg = heading_imu(ros, 0)
    msg.orientation.w = 0
    node.on_imu(msg)
  observe(ros)
  assert "IMU" in node.try_start()
  assert not twists(ros)


@pytest.mark.parametrize("cause", ["stale", "old_capture", "invalid", "stop", "owner", "stock", "timeout"])
def test_heading_stop_paths_zero_and_never_report_timeout_as_success(tmp_path, cause):
  ros, node, mt = heading_trial(tmp_path, turn_angle_rad=1, timeout_s=0.4)
  heading_imu(ros, 0)
  assert node.try_start() is None
  if cause == "old_capture":
    heading_imu(ros, 0, capture=ros.t - 0.3)
  elif cause == "invalid":
    msg = _fake_ros.Imu()
    msg.orientation.z = float("nan")
    node.on_imu(msg)
  elif cause == "stop":
    node.on_emergency(None)
  elif cause == "owner":
    ros.publishers["/kick_cmd_vel"] += 1
  elif cause == "stock":
    node.stock.bounds = (0.49, 0.3, 0.3, 0.2)
  for _ in range(10):
    ros.t += 0.05
    pending, ros.pending = ros.pending, []
    for future in pending:
      future.resolve()
    if cause != "stale":
      heading_imu(ros, 0)
    observe(ros, wz=twists(ros)[-1][2])
    node.on_tick()
  assert node.phase == "finished" and node.record["outcome"] == "stopped"
  assert twists(ros)[-1] == ZERO
  assert not node.record["heading_feedback"]["stable_completion"]
  if cause == "timeout":
    assert "timeout" in node.record["reason"]
  assert max(abs(command[2]) for command in twists(ros)) <= 0.35


def test_heading_complete_requires_dwell_and_records_sensor_evidence(tmp_path):
  ros, node, mt = heading_trial(tmp_path, heading_rad=0.2)
  heading_imu(ros, 0)
  assert node.try_start() is None
  for i in range(15):
    ros.t += 0.05
    pending, ros.pending = ros.pending, []
    for future in pending:
      future.resolve()
    heading_imu(ros, min(0.2, i * 0.04))
    observe(ros, wz=twists(ros)[-1][2])
    node.on_tick()
  assert node.record["outcome"] == "completed"
  feedback = node.record["heading_feedback"]
  assert feedback["initial_yaw_rad"] == 0
  assert feedback["target_yaw_rad"] == pytest.approx(0.2)
  assert abs(feedback["final_error_rad"]) <= mt.HEADING_TOLERANCE_RAD
  assert feedback["stable_completion"] and feedback["feedback_samples"] > 5
  assert node.zero_sent_at - node.started >= 0.5
  assert twists(ros)[-1] == ZERO
  assert node.zero_confirmed_at is not None


def test_repeated_imu_captures_cannot_extend_freshness_or_complete_dwell(tmp_path):
  ros, node, mt = heading_trial(tmp_path, heading_rad=0)
  capture = ros.t
  heading_imu(ros, 0, capture=capture)
  assert node.try_start() is None
  for _ in range(6):
    ros.t += 0.05
    pending, ros.pending = ros.pending, []
    for future in pending:
      future.resolve()
    heading_imu(ros, 0, capture=capture)
    observe(ros)
    node.on_tick()
  assert node.record["outcome"] == "stopped"
  assert not node.record["heading_feedback"]["stable_completion"]
  assert node.heading.feedback_samples == 1
  assert twists(ros)[-1] == ZERO


def test_heading_delayed_captures_cannot_stretch_dwell_then_fresh_captures_complete(tmp_path):
  ros, node, mt = heading_trial(tmp_path, heading_rad=0)
  started = ros.t
  heading_imu(ros, 0)
  assert node.try_start() is None
  # Arrival spans .31 s, but the four distinct captures span only .12 s.
  # Each capture is still within the .2 s freshness window at delivery.
  for arrival, captured in ((0.10, 0.05), (0.20, 0.10), (0.31, 0.12)):
    ros.t = started + arrival
    pending, ros.pending = ros.pending, []
    for future in pending:
      future.resolve()
    heading_imu(ros, 0, capture=started + captured)
    observe(ros)
    node.on_tick()
    assert node.phase == "pulsing" and not node.heading.completed
  assert node.heading.feedback_samples == 4
  assert node.heading.last_capture_ros_ns - node.heading.within_since_ros_ns == 120_000_000
  for arrival, captured in ((0.36, 0.20), (0.41, 0.30)):
    ros.t = started + arrival
    pending, ros.pending = ros.pending, []
    for future in pending:
      future.resolve()
    heading_imu(ros, 0, capture=started + captured)
    observe(ros)
    node.on_tick()
  assert node.phase == "finished" and node.record["outcome"] == "completed"
  assert node.record["heading_feedback"]["capture_dwell_s"] == pytest.approx(0.3)
  assert node.record["heading_feedback"]["dwell_feedback_samples"] == 6


def test_heading_dwell_uses_ros_capture_clock_while_timeout_uses_monotonic_clock(tmp_path):
  ros, node, mt = heading_trial(tmp_path, heading_rad=1, timeout_s=0.4)
  started = ros.t
  ros_stamp_ns = 1_800_000_000_000_000_000  # unrelated ROS and monotonic origins
  node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=ros_stamp_ns))
  def captured_yaw():
    msg = _fake_ros.Imu(yaw=0)
    msg.header = _fake_ros.Header()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ros_stamp_ns, 1_000_000_000)
    node.on_imu(msg)
  captured_yaw()
  assert node.try_start() is None
  assert node.heading.started_ros_ns == ros_stamp_ns
  for _ in range(9):
    ros.t += 0.05
    ros_stamp_ns += 10_000_000  # .01 s of captured time per .05 s of monotonic runtime
    pending, ros.pending = ros.pending, []
    for future in pending:
      future.resolve()
    captured_yaw()
    observe(ros, wz=twists(ros)[-1][2])
    node.on_tick()
  assert node.record["outcome"] == "stopped" and "timeout" in node.record["reason"]
  assert 0.4 <= node.zero_sent_at - started <= 0.451
  assert node.heading.last_capture_ros_ns - node.heading.started_ros_ns < 100_000_000
  assert twists(ros)[-1] == ZERO


def test_session_heading_timeout_fallback_cannot_mark_timer_completion_success(tmp_path, monkeypatch):
  ros, node, mt, inbox, spin_once = session_fixture(tmp_path, monkeypatch)
  events = []
  def spin(actual_node, timeout_sec):
    spin_once(actual_node, timeout_sec)
    heading_imu(ros, 0)
  def emit(event):
    events.append(event)
    if event["event"] == "prepared":
      inbox.put({"command": "run", "token": event["token"]})
  inbox.put({"command": "prepare", "turn_angle_rad": 1, "timeout_s": 0.2})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin) == 4
  assert node.record["outcome"] == "stopped"
  assert "timeout" in node.record["reason"]
  assert sum(event["event"] == "started" for event in events) == 1


def start_session_fixture(tmp_path, *, initial_latch=True, service_ready=True, acknowledge=True):
  from foundation.hardware.software_start import ResidentStart
  ros, node, mt = make_trial(tmp_path)
  ros.tick(25)  # stock evidence, but no gateway release has been observed
  inbox = mt.SessionInbox(node.clock)
  submissions = []
  state = {"protocol": "pupper_stop_v1", "instance_id": "test-gateway", "epoch": 1,
           "sequence": 0, "selected_controller": "neural_controller_default", "latched": initial_latch,
           "fault": False, "stop_required": False, "in_flight": False,
           "unknown_remote_submission": False, "shutting_down": False}
  future = types.SimpleNamespace(complete=False, done=lambda: future.complete,
                                result=lambda: types.SimpleNamespace(success=True, message="activated"))
  availability = [service_ready]
  def submit(payload):
    submissions.append(dict(payload))
    return future
  caller = ResidentStart(submit, lambda: availability[0], clock=node.clock)
  def status():
    state["sequence"] += 1
    caller.observe(state)
    node.on_stop_status(_fake_ros.String(json.dumps(state)))
  status()
  def spin(actual, timeout_sec):
    assert actual is node
    ros.t += 0.01
    pending, ros.pending = ros.pending, []
    for f in pending:
      f.resolve()
    if caller.pending is not None and acknowledge:
      future.complete, state["latched"] = True, False
    status()
    observe(ros, *(node.request.twist if node.phase == "pulsing" else ZERO))
    node.on_tick()
  return ros, node, mt, inbox, caller, submissions, state, availability, spin


def test_session_initial_latch_explicit_start_consumes_token_and_subsequent_stop_ends_session(tmp_path):
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(tmp_path)
  events, captured, prepared = [], [], []
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready":
      assert node.stop_requested is None and node.gateway_blocked
      inbox.put({"command": "prepare", "axis": "forward"})
    elif event["event"] == "prepared":
      prepared.append(event["token"])
      captured.append(state["sequence"])
      inbox.put({"command": "start", "_start_ticket": "forged", "ticket": "ignored"})
    elif event["event"] == "start_result":
      assert event["success"]
      inbox.put({"command": "run", "token": prepared[0]})
    elif event["event"] == "refused":
      assert event["command"] == "run" and "token" in event["reason"]
      state["latched"] = True
      inbox.put({"command": "start"})  # cannot rearm the subsequent Stop
  code = mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin, start_client=caller)
  assert code == 3 and len(submissions) == 1
  assert submissions[0]["sequence"] == captured[0] < state["sequence"]
  assert not twists(ros)
  assert node.stop_requested == "e-stop gateway latched"


@pytest.mark.parametrize("changed", [{"epoch": 2}, {"instance_id": "restarted"}])
def test_initial_latch_cannot_hide_later_cleaned_stop_or_gateway_restart(tmp_path, changed):
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(tmp_path)
  events = []
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready":
      assert node.gateway_authority == ("test-gateway", 1)
      state.update(changed)  # transient Stop/cleanup was missed; ordinary latch remains
      state["sequence"] += 1
      caller.observe(state)
      node.on_stop_status(_fake_ros.String(json.dumps(state)))
      inbox.put({"command": "start"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin,
                        start_client=caller) == 3
  assert node.stop_requested == "gateway authority epoch or instance changed"
  assert node.gateway_authority == ("test-gateway", 1)
  assert not submissions and not twists(ros)


def test_start_status_arriving_before_node_authority_cannot_bypass_epoch_fence(tmp_path):
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(tmp_path)
  events = []
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready":
      state["epoch"], state["sequence"] = 2, state["sequence"] + 1
      caller.observe(state)  # helper subscription ran before the node's status callback
      inbox.put({"command": "start"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin,
                        start_client=caller) == 3
  assert not submissions


def test_start_press_time_is_frozen_before_node_lock_contention(tmp_path):
  import threading
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(tmp_path)
  captured, tickets, events = threading.Event(), [], []
  original_capture = caller.capture
  def capture(*, pressed_at):
    ticket = original_capture(pressed_at=pressed_at)
    tickets.append(ticket)
    captured.set()
    return ticket
  caller.capture = capture
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready":
      ingress = ros.t
      node.lock.acquire()
      thread = threading.Thread(target=lambda: inbox.put({"command": "start"}))
      thread.start()
      try:
        frozen_before_unlock = captured.wait(timeout=1)
        ros.t += 0.30
        state["sequence"] += 1
        caller.observe(state)  # later status must not repair the blocked original press
      finally:
        node.lock.release()
        thread.join(timeout=1)
      assert frozen_before_unlock and not thread.is_alive()
      assert tickets[0].pressed_at == ingress
    elif event["event"] == "refused":
      assert "expired" in event["reason"]
      inbox.put({"command": "quit"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin,
                        start_client=caller) == 0
  assert not submissions and not twists(ros)


def test_start_capture_cannot_use_status_that_arrived_after_ingress_during_helper_lock_wait(tmp_path):
  import threading
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(tmp_path)
  entered = threading.Event()
  original_capture = caller.capture
  def capture(*, pressed_at):
    entered.set()
    return original_capture(pressed_at=pressed_at)
  caller.capture = capture
  def emit(event):
    if event["event"] == "session_ready":
      caller.lock.acquire()
      thread = threading.Thread(target=lambda: inbox.put({"command": "start"}))
      thread.start()
      try:
        waiting = entered.wait(timeout=1)
        ros.t += 0.05
        state["sequence"] += 1
        caller.observe(state)
      finally:
        caller.lock.release()
        thread.join(timeout=1)
      assert waiting and not thread.is_alive()
    elif event["event"] == "refused":
      assert "not observed at Start input receipt" in event["reason"]
      inbox.put({"command": "quit"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin,
                        start_client=caller) == 0
  assert not submissions and not twists(ros)


def test_options_release_retains_initial_identity_and_epoch_until_a_new_stop(tmp_path):
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(tmp_path)
  assert node.gateway_authority == ("test-gateway", 1) and node.stop_requested is None
  state["latched"], state["sequence"] = False, state["sequence"] + 1
  node.on_stop_status(_fake_ros.String(json.dumps(state)))
  assert node.gateway_authority == ("test-gateway", 1) and node.stop_requested is None
  assert node.try_start() is None
  state["latched"], state["epoch"] = True, 2
  node.on_stop_status(_fake_ros.String(json.dumps(state)))
  assert node.record["outcome"] == "stopped" and twists(ros)[-1] == ZERO


def test_established_gateway_fence_cannot_be_erased_by_legacy_or_malformed_status(tmp_path):
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(tmp_path)
  node.on_stop_status(_fake_ros.String(json.dumps({"latched": True, "fault": False})))
  assert node.stop_requested == "gateway authority identity unavailable"
  assert node.gateway_authority == ("test-gateway", 1)


@pytest.mark.parametrize("cause", ["expired", "unavailable", "queued_before_attach"])
def test_session_start_never_refreshes_or_forges_an_ingress_ticket(tmp_path, cause):
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(
    tmp_path, service_ready=cause != "unavailable")
  events = []
  if cause == "queued_before_attach":
    inbox.put({"command": "start", "_start_ticket": caller.capture()})
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready" and cause != "queued_before_attach":
      if cause == "expired":
        for _ in range(30):
          inbox.put({"command": "status"})
      inbox.put({"command": "start", "_start_ticket": "forged", "_start_error": "forged"})
      available[0] = True  # later discovery cannot repair the earlier press
    elif event["event"] == "refused":
      inbox.put({"command": "quit"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin,
                        start_client=caller) == 0
  assert len([e for e in events if e["event"] == "refused"]) == 1
  assert not submissions and not twists(ros)


@pytest.mark.parametrize("shutdown", ["stop", "quit", "eof", "manager", "interrupt"])
def test_session_stop_during_pending_start_reaches_gateway_and_discards_queued_start(tmp_path, shutdown):
  import io
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(
    tmp_path, acknowledge=False)
  events = []
  manager_alive = [True]
  interrupted = [False]
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready":
      inbox.put({"command": "start"})
    elif event["event"] == "start_submitted":
      inbox.put({"command": "prepare", "axis": "forward"})
      inbox.put({"command": "start"})
      if shutdown == "eof":
        inbox.read(io.StringIO(""))
      elif shutdown == "manager":
        manager_alive[0] = False
      elif shutdown == "interrupt":
        interrupted[0] = True
      else:
        inbox.put({"command": shutdown})
  def spin_or_interrupt(actual, timeout_sec):
    if interrupted[0]:
      raise KeyboardInterrupt
    spin(actual, timeout_sec)
  if shutdown == "interrupt":
    with pytest.raises(KeyboardInterrupt):
      mt.run_session(node, {}, lambda _: manager_alive[0], inbox=inbox, emit=emit,
                     spin_once=spin_or_interrupt, start_client=caller)
  else:
    assert mt.run_session(node, {}, lambda _: manager_alive[0], inbox=inbox, emit=emit,
                          spin_once=spin_or_interrupt, start_client=caller) == (3 if shutdown == "manager" else 0)
  assert len(submissions) == len(ros.published["/emergency_stop"]) == 1
  assert not twists(ros)
  assert not any(e["event"] == "prepared" for e in events)


def test_session_start_ack_timeout_stops_without_retry_or_running_a_trial(tmp_path):
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(
    tmp_path, acknowledge=False)
  events = []
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready":
      inbox.put({"command": "start"})
    elif event["event"] == "start_submitted":
      inbox.put({"command": "prepare", "axis": "forward"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin,
                        start_client=caller) == 3
  assert len(submissions) == len(ros.published["/emergency_stop"]) == 1
  assert any(e["event"] == "refused" and "pending" in e["reason"] for e in events)
  assert any(e["event"] == "start_result" and e["uncertain"] for e in events)
  assert not twists(ros)


def test_session_refuses_start_during_motion_without_submitting_service_call(tmp_path):
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(
    tmp_path, initial_latch=False)
  events = []
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready":
      inbox.put({"command": "prepare", "axis": "forward", "duration": 1})
    elif event["event"] == "prepared":
      inbox.put({"command": "run", "token": event["token"]})
    elif event["event"] == "started":
      inbox.put({"command": "start"})
    elif event["event"] == "refused":
      assert event["command"] == "start" and "running" in event["reason"]
      inbox.put({"command": "stop"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin,
                        start_client=caller) == 4
  assert not submissions and twists(ros)[-1] == ZERO


def test_missing_start_interface_is_optional_and_explicit_start_is_refused(tmp_path, monkeypatch):
  from foundation.hardware import software_start
  def missing(*args, **kwargs):
    raise ImportError("legacy release has no StartMotion")
  monkeypatch.setattr(software_start, "attach", missing)
  ros, node, mt, inbox, spin = session_fixture(tmp_path, monkeypatch)
  events = []
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready":
      assert not event["software_start_supported"]
      inbox.put({"command": "start"})
    elif event["event"] == "refused":
      inbox.put({"command": "prepare", "axis": "forward", "duration": 0.3})
    elif event["event"] == "prepared":
      inbox.put({"command": "run", "token": event["token"]})
    elif event["event"] == "trial_complete":
      inbox.put({"command": "quit"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin) == 0
  assert sum(e["event"] == "trial_complete" for e in events) == 1
  assert not ros.published["/emergency_stop"]


def test_start_acknowledgement_does_not_replace_stock_authority_checks(tmp_path):
  ros, node, mt, inbox, caller, submissions, state, available, spin = start_session_fixture(tmp_path)
  events = []
  def emit(event):
    events.append(event)
    if event["event"] == "session_ready":
      inbox.put({"command": "start"})
    elif event["event"] == "start_result":
      ros.controllers["neural_controller_default"] = "inactive"
      node.stock.bounds, node.stock.error = None, "stock controller inactive"
      inbox.put({"command": "prepare", "axis": "forward"})
    elif event["event"] == "prepared":
      inbox.put({"command": "run", "token": event["token"]})
    elif event["event"] == "refused":
      assert event["command"] == "run" and "stock" in event["reason"]
      inbox.put({"command": "quit"})
  assert mt.run_session(node, {}, lambda _: True, inbox=inbox, emit=emit, spin_once=spin,
                        start_client=caller) == 0
  assert len(submissions) == 1 and not twists(ros)


def test_legacy_initial_latch_can_release_via_options_but_never_rearms_after_stop(tmp_path):
  ros, node, mt = make_trial(tmp_path)
  ros.tick(25)
  node.on_stop_status(_fake_ros.String(json.dumps({"latched": True, "fault": False})))
  assert node.stop_requested is None and "latched" in node.try_start()
  node.on_stop_status(_fake_ros.String(json.dumps({"latched": False, "fault": False})))
  assert node.try_start() is None
  node.on_stop_status(_fake_ros.String(json.dumps({"latched": True, "fault": False})))
  assert node.record["outcome"] == "stopped" and node.stop_requested
  assert twists(ros)[-1] == ZERO


@pytest.mark.parametrize("flag", ["fault", "stop_required", "unknown_remote_submission", "shutting_down"])
def test_initial_gateway_latch_exception_never_hides_unsafe_state(tmp_path, flag):
  ros, node, mt = make_trial(tmp_path)
  node.on_stop_status(_fake_ros.String(json.dumps({"latched": True, flag: True})))
  assert node.stop_requested and node.gateway_blocked
