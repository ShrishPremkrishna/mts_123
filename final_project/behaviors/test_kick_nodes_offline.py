"""Offline replay of both kick nodes against stand-in rclpy (see _fake_ros.py).

Checks services, published commands, stale/lost handling, cancellation, the
/cmd_vel ownership checks, and the learned kick's confirmed controller switches
and zero-command exit. Not a ROS run.

    python3 -m pytest -q final_project/behaviors/test_kick_nodes_offline.py
"""

from __future__ import annotations

import importlib
import hashlib
import json
import math
import sys

import pytest

import _fake_ros

ZERO = (0.0, 0.0, 0.0)
POLICY_FIXTURE = None


@pytest.fixture(autouse=True)
def isolate_ros_modules(tmp_path):
  """A tiny explicit stock fixture; restore modules after every fake-ROS test."""
  global POLICY_FIXTURE
  POLICY_FIXTURE = tmp_path / "stock.json"
  POLICY_FIXTURE.write_bytes(b"test-only stock policy fixture")
  roots = {"rclpy", "rcl_interfaces", "geometry_msgs", "std_msgs", "std_srvs", "sensor_msgs",
           "controller_manager_msgs", "run_through_kick_node", "learned_kick_node"}
  saved = {n: m for n, m in sys.modules.items() if n.split(".")[0] in roots}
  yield
  for name in list(sys.modules):
    if name.split(".")[0] in roots:
      del sys.modules[name]
  sys.modules.update(saved)


def make_node(module_name, overrides=None, other_cmd_vel_publishers=None):
  ros = _fake_ros.install(overrides)
  module = importlib.reload(importlib.import_module(module_name))  # bind to fresh fakes
  if other_cmd_vel_publishers is None:  # the mux for the run-through kick, nothing for the learned one
    other_cmd_vel_publishers = 1 if module_name == "run_through_kick_node" else 0
  ros.publishers["/cmd_vel"] += other_cmd_vel_publishers
  cls = module.RunThroughKickNode if module_name == "run_through_kick_node" else module.LearnedKickNode
  node = cls()
  if module_name == "run_through_kick_node":
    module.STOCK_POLICY_SHA256 = hashlib.sha256(POLICY_FIXTURE.read_bytes()).hexdigest()
    ros.controller_params["model_path"] = str(POLICY_FIXTURE)
    node.stock.clock = lambda: ros.t
  gateway(ros, latched=False)  # the operator pressed Options: the walker stands
  if module_name == "run_through_kick_node":
    ros.tick(3)  # asynchronous inventory/parameters, without any activation
  return ros, node


def gateway(ros, latched):
  ros.publish("/estop_controller/stop_status",
              _fake_ros.String(json.dumps({"latched": latched, "fault": False})))


def last_state(ros, topic):
  msgs = ros.published[topic]
  return json.loads(msgs[-1].data) if msgs else None


def twists(ros, topic):
  return [(t.linear.x, t.linear.y, t.angular.z) for t in ros.published[topic]]


# ---------------------------------------------------------------- run-through


def test_manual_kick_runs_to_done_and_consumes_the_measurement():
  ros, node = make_node("run_through_kick_node", {"input_mode": "manual"})
  assert not ros.call("activate_run_through_kick").success  # no measurement yet
  node.set_parameters([_fake_ros.Parameter("manual_ball_x", value=1.0),
                       _fake_ros.Parameter("manual_aim_deg", value=10.0)])
  assert ros.call("activate_run_through_kick").success
  ros.tick(200)  # 10 s at 20 Hz
  cmds = twists(ros, "/kick_cmd_vel")
  assert max(c[0] for c in cmds) == pytest.approx(0.45) and cmds[-1] == ZERO
  assert last_state(ros, "/run_through_kick/state")["state"] == "done"
  assert math.isnan(node.get_parameter("manual_ball_x").value)
  assert math.isnan(node.get_parameter("manual_aim_deg").value)
  assert not ros.call("activate_run_through_kick").success


@pytest.mark.parametrize("others", [0, 2])  # no mux, or the mux plus joystick teleop
def test_run_through_refuses_without_exactly_the_mux_on_cmd_vel(others):
  ros, node = make_node("run_through_kick_node", {"input_mode": "manual"}, other_cmd_vel_publishers=others)
  node.set_parameters([_fake_ros.Parameter("manual_ball_x", value=1.0)])
  res = ros.call("activate_run_through_kick")
  assert not res.success and "publisher" in res.message


def test_run_through_refuses_while_the_gateway_is_latched_and_manual_bounds():
  ros, node = make_node("run_through_kick_node", {"input_mode": "manual"})
  gateway(ros, latched=True)
  node.set_parameters([_fake_ros.Parameter("manual_ball_x", value=1.0)])
  assert "latched" in ros.call("activate_run_through_kick").message
  gateway(ros, latched=False)
  for x, y in ((0.5, 0.0), (2.0, 0.0), (1.0, 0.8)):  # reverse, too far, too wide
    node.set_parameters([_fake_ros.Parameter("manual_ball_x", value=x),
                         _fake_ros.Parameter("manual_ball_y", value=y)])
    assert not ros.call("activate_run_through_kick").success


def test_run_through_rereads_parameters_at_activation():
  ros, node = make_node("run_through_kick_node", {"input_mode": "manual"})
  node.set_parameters([_fake_ros.Parameter("speed_efficiency", value=0.7),
                       _fake_ros.Parameter("manual_ball_x", value=1.0)])
  assert ros.call("activate_run_through_kick").success
  assert node.kick.params.speed_efficiency == 0.7


def test_restamped_stale_ball_hits_the_travel_budget():
  # Perception republishes an old detection with fresh stamps: it looks fresh but
  # never comes closer. The travel budget ends the walk before the lane runs out.
  ros, node = make_node("run_through_kick_node", {"input_mode": "robot", "align_timeout": 12.0})
  ros.publish("/soccer/ball", _fake_ros.PointStamped(2.0, 0.0, ros.t))
  assert ros.call("activate_run_through_kick").success
  for _ in range(220):  # 11 s
    ros.publish("/soccer/ball", _fake_ros.PointStamped(2.0, 0.0, ros.t))
    ros.tick(1)
  state = last_state(ros, "/run_through_kick/state")
  assert state["state"] == "aborted" and "budget" in state["reason"], state
  travelled = sum(c[0] for c in twists(ros, "/kick_cmd_vel")) * 0.05 * 0.8
  assert travelled <= node.kick.params.align_travel_max + 0.05


def test_bad_ball_messages_are_dropped():
  ros, node = make_node("run_through_kick_node", {"input_mode": "robot"})
  ros.publish("/soccer/ball", _fake_ros.PointStamped(float("nan"), 0.0, ros.t))
  ros.publish("/soccer/ball", _fake_ros.PointStamped(1.0, 0.0, 0.0))  # stamp 0
  ros.publish("/soccer/ball", _fake_ros.PointStamped(1.0, 0.0, ros.t + 5.0))  # future clock
  assert node.ball is None


def test_robot_mode_without_a_ball_sends_zero_then_aborts():
  ros, node = make_node("run_through_kick_node", {"input_mode": "robot"})
  assert ros.call("activate_run_through_kick").success
  ros.tick(40)  # 2 s, past lost_timeout (1.5 s)
  assert all(c == ZERO for c in twists(ros, "/kick_cmd_vel"))
  state = last_state(ros, "/run_through_kick/state")
  assert state["state"] == "aborted" and "lost" in state["reason"]


@pytest.mark.parametrize("how", ["service", "estop", "button", "latched", "teleop"])
def test_run_through_stop_sends_zero_at_once(how):
  ros, node = make_node("run_through_kick_node", {"input_mode": "robot"})
  ros.publish("/soccer/ball", _fake_ros.PointStamped(2.0, 0.0, ros.t))
  ros.call("activate_run_through_kick")
  ros.tick(5)
  assert ros.published["/kick_cmd_vel"][-1].linear.x > 0.0
  if how == "service":
    ros.call("deactivate_run_through_kick")
  elif how == "estop":
    ros.publish("/emergency_stop", _fake_ros.Empty())
  elif how == "button":
    ros.publish("/joy", _fake_ros.Joy(buttons=[0, 1, 0, 0]))
  elif how == "latched":
    gateway(ros, latched=True)
  else:  # someone starts joystick teleop: caught by the 1 s ownership check
    ros.publishers["/cmd_vel"] += 1
    ros.tick(20)
  assert twists(ros, "/kick_cmd_vel")[-1] == ZERO
  ros.tick(1)
  assert last_state(ros, "/run_through_kick/state")["state"] == "aborted"


def test_field_mode_converts_to_the_robot_frame():
  ros, node = make_node("run_through_kick_node", {"input_mode": "field"})
  # Robot at (2, 1) facing +y; ball at (2, 3): 2 m dead ahead in the robot frame.
  ros.publish("/soccer/robot_pose", _fake_ros.PoseStamped(2.0, 1.0, math.pi / 2, ros.t))
  ros.publish("/soccer/ball", _fake_ros.PointStamped(2.0, 3.0, ros.t))
  ros.call("activate_run_through_kick")
  ros.tick(1)
  vx, vy, wz = twists(ros, "/kick_cmd_vel")[-1]
  assert vx == pytest.approx(0.3) and abs(vy) < 1e-6 and abs(wz) < 1e-6


# ---------------------------------------------------------------- stock authority/caps

STATUS_SERVICE = "/controller_manager/list_controllers"
PARAM_SERVICE = "/neural_controller_default/get_parameters"


def manual_ready(ros, node):
  node.set_parameters([_fake_ros.Parameter("manual_ball_x", value=1.0)])
  return ros.call("activate_run_through_kick")


@pytest.mark.parametrize("owner", ["neural_controller_student", "neural_controller_default_alt", "gemini_walk", "custom"])
def test_stock_guard_refuses_wrong_active_owner_without_motion(owner):
  ros, node = make_node("run_through_kick_node")
  ros.controllers["neural_controller_default"] = "inactive"
  ros.controllers[owner] = "active"
  ros.tick(7)
  assert not manual_ready(ros, node).success
  assert not ros.published["/kick_cmd_vel"]
  assert not ros.published["/emergency_stop"]


def test_stock_guard_refuses_competing_controller_but_allows_read_only_broadcaster():
  ros, node = make_node("run_through_kick_node")
  ros.controllers["joint_states"] = "active"
  ros.controller_types["joint_states"] = "joint_state_broadcaster/JointStateBroadcaster"
  ros.tick(7)
  assert node.stock.problem() is None
  ros.controllers["gemini_walk"] = "active"
  ros.tick(7)
  assert not manual_ready(ros, node).success


@pytest.mark.parametrize("name,value", [("charge_speed", 0.7), ("charge_speed", 1.0),
                                       ("align_speed", 0.35), ("max_vy", 0.5), ("max_wz", 2.1)])
def test_stock_guard_refuses_requests_that_native_would_clip(name, value):
  ros, node = make_node("run_through_kick_node", {name: value})
  res = manual_ready(ros, node)
  assert not res.success and name in res.message
  assert not ros.published["/kick_cmd_vel"]


@pytest.mark.parametrize("name,value", [("max_vx_forward", None), ("max_vx_backward", None),
                                       ("speed_cap", None), ("game_buttons", None),
                                       ("max_vx_forward", -1.0), ("speed_cap", float("nan")),
                                       ("max_vx_backward", float("inf")), ("game_buttons", True),
                                       ("speed_cap", 0.2), ("max_vx_forward", 0.2)])
def test_stock_guard_requires_valid_caps_and_respects_tighter_caps(name, value):
  ros, node = make_node("run_through_kick_node")
  ros.controller_params[name] = value
  ros.tick(7)
  assert not manual_ready(ros, node).success


@pytest.mark.parametrize("service", [STATUS_SERVICE, PARAM_SERVICE])
def test_stock_guard_missing_or_hung_service_refuses_and_bounds_requests(service):
  ros, node = make_node("run_through_kick_node")
  ros.unavailable.add(service)
  ros.tick(7)
  assert not manual_ready(ros, node).success
  ros.unavailable.clear()
  ros.held_services.add(service)
  ros.tick(50)
  assert not manual_ready(ros, node).success
  assert sum(not f.cancelled for f in ros.held) <= 1
  assert not ros.published["/kick_cmd_vel"]


@pytest.mark.parametrize("change", ["missing", "mismatch", "empty", "directory"])
def test_stock_guard_requires_matching_regular_model_file(change, tmp_path):
  ros, node = make_node("run_through_kick_node")
  if change == "missing":
    POLICY_FIXTURE.unlink()
  elif change == "mismatch":
    POLICY_FIXTURE.write_bytes(b"same slot, different actor")
  elif change == "empty":
    ros.controller_params["model_path"] = ""
  else:
    ros.controller_params["model_path"] = str(tmp_path)
  ros.tick(7)
  assert not manual_ready(ros, node).success


@pytest.mark.parametrize("fault", ["student", "lost", "caps", "model", "status_timeout", "params_timeout"])
def test_active_shot_authority_loss_requests_emergency_stop_and_latches_abort(fault):
  ros, node = make_node("run_through_kick_node")
  assert manual_ready(ros, node).success
  ros.tick(3)
  assert any(c[0] > 0 for c in twists(ros, "/kick_cmd_vel"))
  if fault in ("student", "lost"):
    ros.controllers["neural_controller_default"] = "inactive"
    if fault == "student":
      ros.controllers["neural_controller_student"] = "active"
  elif fault == "caps":
    ros.controller_params["max_vx_forward"] = 0.2
  elif fault == "model":
    POLICY_FIXTURE.write_bytes(b"policy replaced")
  else:
    ros.held_services.add(STATUS_SERVICE if fault == "status_timeout" else PARAM_SERVICE)
  ros.tick(15)
  assert last_state(ros, "/run_through_kick/state")["state"] == "aborted"
  assert len(ros.published["/emergency_stop"]) == 1
  assert twists(ros, "/kick_cmd_vel")[-1] == ZERO
  count = len(twists(ros, "/kick_cmd_vel"))
  ros.tick(20)
  assert len(twists(ros, "/kick_cmd_vel")) == count  # never auto-resume


def test_late_evidence_after_stop_cannot_start_or_authorize():
  ros, node = make_node("run_through_kick_node")
  ros.held_services.update([STATUS_SERVICE, PARAM_SERVICE])
  ros.tick(5)
  old = list(ros.held)
  ros.call("deactivate_run_through_kick")
  for future in old:
    future.resolve()
  assert not manual_ready(ros, node).success
  ros.tick(15)
  for future in old:
    future.resolve()
  assert not manual_ready(ros, node).success
  assert not ros.published["/kick_cmd_vel"]
  ros.held_services.clear()
  ros.tick(10)
  assert node.stock.problem() is None
  assert not node.kick.active  # fresh data is readiness, never a motion request


def test_failed_and_stale_reads_do_not_refresh_evidence():
  ros, node = make_node("run_through_kick_node")
  node.stock.invalidate("new read required")
  ros.tick(5)
  # Force an old, otherwise successful result; age is measured from request start.
  if node.stock.pending is None:
    ros.t += 0.21
    node.stock.poll()
  started, futures = node.stock.pending
  for future in futures:
    future.resolve()
  ros.t = started + 0.41
  node.stock.poll()
  assert node.stock.problem() is not None
  # A service future raising is also an explicit refusal, not stale success.
  if node.stock.pending is None:
    ros.t += 0.21
    node.stock.poll()
  for future in node.stock.pending[1]:
    future.resolve()
  node.stock.pending[1][0]._error = RuntimeError("read failed")
  node.stock.poll()
  assert "read failed" in node.stock.problem()


def test_per_command_guard_rejects_reverse_clipping_even_if_controller_changes_output(monkeypatch):
  ros, node = make_node("run_through_kick_node")
  assert manual_ready(ros, node).success
  module = importlib.import_module("run_through_kick")
  monkeypatch.setattr(node.kick, "step", lambda *args: module.Command(-0.31, 0, 0))
  ros.tick(1)
  assert twists(ros, "/kick_cmd_vel")[-1] == ZERO
  assert ros.published["/emergency_stop"]
  assert "bounds" in last_state(ros, "/run_through_kick/state")["reason"]


# ---------------------------------------------------------------- learned kick


def feed(ros, x, y, yaw=0.0):
  ros.publish("/imu_sensor_broadcaster/imu", _fake_ros.Imu(yaw))
  ros.publish("/soccer/ball", _fake_ros.PointStamped(x, y, ros.t))


def stage(ros):
  return last_state(ros, "/learned_kick/state")["stage"]


def start(ros, x=1.0, y=0.1):
  feed(ros, x, y)
  res = ros.call("activate_learned_kick")
  assert res.success, res.message
  feed(ros, x, y)
  ros.tick(1)  # list_controllers answers
  feed(ros, x, y)
  ros.tick(1)  # the switch answers


def test_learned_kick_refusals():
  ros, node = make_node("learned_kick_node")
  feed(ros, 2.0, 0.0)
  assert "away" in ros.call("activate_learned_kick").message
  feed(ros, 0.8, 0.8)  # bearing 45 deg
  assert not ros.call("activate_learned_kick").success
  ros2, _ = make_node("learned_kick_node", other_cmd_vel_publishers=1)  # joystick teleop running
  feed(ros2, 1.0, 0.0)
  res = ros2.call("activate_learned_kick")
  assert not res.success and "teleop:=False" in res.message
  assert ros.switch_calls == [] and ros.published["/cmd_vel"] == []


def test_learned_kick_full_attempt():
  ros, node = make_node("learned_kick_node")
  feed(ros, 1.0, 0.1)
  ros.call("activate_learned_kick")
  feed(ros, 1.0, 0.1)
  ros.tick(1)  # list_controllers answers; the switch request is in flight
  assert stage(ros) == "switching_in" and ros.switch_calls == []
  before = twists(ros, "/cmd_vel")
  assert before and all(c == ZERO for c in before), "only zero until the switch is confirmed"
  feed(ros, 1.0, 0.1)
  ros.tick(1)  # the gateway confirms; the first target follows in the same tick
  first = ros.switch_calls[0]
  assert first.activate_controllers == ["neural_controller_student"]
  assert first.deactivate_controllers == ["neural_controller_default"]
  assert first.strictness == _fake_ros.SwitchController.Request.STRICT
  assert stage(ros) == "running"
  assert twists(ros, "/cmd_vel")[-1][:2] == pytest.approx((1.0, 0.1))
  x = 1.0
  for _ in range(60):  # the robot closes in at about 1 m/s
    x = max(0.2, x - 0.02)
    feed(ros, x, 0.0)
    ros.tick(1)
  assert twists(ros, "/cmd_vel")[-1][0] == pytest.approx(0.2)
  during_exit = []
  for _ in range(55):  # the kick window ends 1.25 s after the ball enters 0.45 m
    feed(ros, 0.2, 0.0)
    ros.tick(1)
    if stage(ros) == "exiting":
      during_exit.append(twists(ros, "/cmd_vel")[-1])
  assert during_exit and all(c != ZERO for c in during_exit), "trained-on target until the walker is back"
  ros.tick(1)
  back = ros.switch_calls[-1]
  assert back.activate_controllers == ["neural_controller_default"]
  assert back.deactivate_controllers == ["neural_controller_student"]
  assert ros.controllers["neural_controller_default"] == "active"
  n = len(ros.published["/cmd_vel"])
  ros.tick(100)
  tail = twists(ros, "/cmd_vel")[n:]
  assert tail and all(c == ZERO for c in tail)  # zero hold, then silence
  assert len(tail) < 60 and stage(ros) == "idle"
  assert last_state(ros, "/learned_kick/state")["phase"] == "done"


def test_learned_kick_holds_the_aim_in_the_world():
  ros, node = make_node("learned_kick_node")
  start(ros, 1.0, 0.0)
  feed(ros, 0.9, 0.0, yaw=0.2)  # the robot turned 0.2 rad left
  ros.tick(1)
  assert ros.published["/cmd_vel"][-1].angular.z == pytest.approx(-0.2)


@pytest.mark.parametrize("problem", ["not_loaded", "walker_off", "refused"])
def test_learned_kick_never_sends_targets_if_the_switch_cannot_happen(problem):
  ros, node = make_node("learned_kick_node")
  if problem == "not_loaded":
    del ros.controllers["neural_controller_student"]
  elif problem == "walker_off":
    ros.controllers["neural_controller_default"] = "inactive"
  else:
    ros.switch_ok = False
  feed(ros, 1.0, 0.0)
  assert ros.call("activate_learned_kick").success
  for _ in range(10):
    feed(ros, 1.0, 0.0)
    ros.tick(1)
  assert all(c == ZERO for c in twists(ros, "/cmd_vel"))
  assert last_state(ros, "/learned_kick/state")["phase"] == "aborted"
  assert len(ros.switch_calls) == (1 if problem == "refused" else 0)


def test_learned_kick_exit_refused_sends_an_emergency_stop():
  ros, node = make_node("learned_kick_node")
  start(ros)
  ros.switch_ok = False  # the gateway refuses the way back
  ros.call("deactivate_learned_kick")
  for _ in range(10):  # 2 tries, each refused at once
    ros.tick(1)
  assert len(ros.published["/emergency_stop"]) == 1
  assert twists(ros, "/cmd_vel")[-1] == ZERO


def test_learned_kick_exit_unanswered_sends_an_emergency_stop_within_2_s():
  ros, node = make_node("learned_kick_node")
  start(ros)
  ros.switch_answers = False  # the gateway never answers the way back
  ros.call("deactivate_learned_kick")
  for _ in range(110):  # 2.2 s: two tries of exit_timeout_s (1 s)
    feed(ros, 0.9, 0.0)
    ros.tick(1)
  assert len(ros.published["/emergency_stop"]) == 1


def test_learned_kick_square_ends_normally_and_other_buttons_abort():
  ros, node = make_node("learned_kick_node")
  start(ros)
  ros.publish("/joy", _fake_ros.Joy(buttons=[0, 0, 0, 1]))  # square
  ros.tick(1)
  assert ros.switch_calls[-1].activate_controllers == ["neural_controller_default"]


def test_learned_kick_refuses_while_latched_and_stops_on_imu_loss():
  ros, node = make_node("learned_kick_node")
  gateway(ros, latched=True)
  feed(ros, 1.0, 0.0)
  assert "latched" in ros.call("activate_learned_kick").message
  gateway(ros, latched=False)
  start(ros)
  for _ in range(30):  # ball updates continue, IMU stops
    ros.publish("/soccer/ball", _fake_ros.PointStamped(0.9, 0.0, ros.t))
    ros.tick(1)
  assert "IMU" in last_state(ros, "/learned_kick/state")["reason"]


def test_learned_kick_frozen_target_after_fade_aborts():
  ros, node = make_node("learned_kick_node")
  start(ros)
  for _ in range(240):  # 4.8 s: fade-in (4 s) and then 0.8 s without the target moving
    feed(ros, 1.0, 0.1)
    ros.tick(1)
  assert "frozen" in last_state(ros, "/learned_kick/state")["reason"]


@pytest.mark.parametrize("event", ["estop", "button", "stale"])
def test_learned_kick_aborts_safely(event):
  ros, node = make_node("learned_kick_node")
  start(ros)
  ros.tick(5)
  calls_before = len(ros.switch_calls)
  if event == "estop":
    ros.publish("/emergency_stop", _fake_ros.Empty())
  elif event == "button":
    ros.publish("/joy", _fake_ros.Joy(buttons=[1, 0, 0, 0]))  # X: the operator takes over
  else:
    ros.tick(60)  # no ball updates for 1.2 s
  ros.tick(2)
  assert last_state(ros, "/learned_kick/state")["phase"] == "aborted"
  assert twists(ros, "/cmd_vel")[-1] == ZERO
  if event in ("estop", "button"):
    assert len(ros.switch_calls) == calls_before  # the e-stop or operator owns the switch
  else:
    assert ros.switch_calls[-1].activate_controllers == ["neural_controller_default"]
