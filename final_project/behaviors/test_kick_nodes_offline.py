"""Offline replay of both kick nodes against stand-in rclpy (see _fake_ros.py).

Checks services, published commands, stale/lost handling, cancellation, the
/cmd_vel ownership checks, and the learned kick's confirmed controller switches
and zero-command exit. Not a ROS run.

    python3 -m pytest -q final_project/behaviors/test_kick_nodes_offline.py
"""

from __future__ import annotations

import importlib
import json
import math

import pytest

import _fake_ros

ZERO = (0.0, 0.0, 0.0)


def make_node(module_name, overrides=None, other_cmd_vel_publishers=None):
  ros = _fake_ros.install(overrides)
  module = importlib.reload(importlib.import_module(module_name))  # bind to fresh fakes
  if other_cmd_vel_publishers is None:  # the mux for the run-through kick, nothing for the learned one
    other_cmd_vel_publishers = 1 if module_name == "run_through_kick_node" else 0
  ros.publishers["/cmd_vel"] += other_cmd_vel_publishers
  cls = module.RunThroughKickNode if module_name == "run_through_kick_node" else module.LearnedKickNode
  node = cls()
  gateway(ros, latched=False)  # the operator pressed Options: the walker stands
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
  assert max(c[0] for c in cmds) > 0.9 and cmds[-1] == ZERO
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
  ros, node = make_node("run_through_kick_node", {"input_mode": "robot"})
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
  assert vx > 0.3 and abs(vy) < 1e-6 and abs(wz) < 1e-6


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
