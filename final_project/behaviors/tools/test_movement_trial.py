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
