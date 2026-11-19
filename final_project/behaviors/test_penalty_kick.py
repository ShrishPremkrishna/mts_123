"""Offline checks of the penalty kick: logic, kinematic sweep, strike sequencer, node wiring.

    python3 -m pytest -q final_project/behaviors/test_penalty_kick.py

The sweep uses penalty_sim.World (walker lag, efficiency and drift; camera
latency, noise, dropouts and blind zone; IMU drift). The node tests replay
penalty_kick_node against stand-in rclpy (_fake_ros.py). Neither is a robot or
ROS run.
"""

from __future__ import annotations

import importlib
import json
import math
import os
import sys
from dataclasses import replace

import pytest

import _fake_ros
import penalty_sim as ps
from penalty_kick import PenaltyKick, PenaltyParams, State, StrikeSequencer, check_params
from run_through_kick import Command

P = PenaltyParams()


def attempt(robot=(0.0, 0.0, 0.0), ball=(1.5, 0.0), target=(3.5, 0.0), params=P, seed=0, stop_at=None,
            **cfg):
  return ps.run_attempt(params, ps.WorldCfg(**cfg), robot, ball, target, seed=seed, stop_at=stop_at)


# ---------------------------------------------------------------- whole attempt (kinematic world)


def test_sweep_places_the_ball_at_the_kicking_paw():
  s = ps.summary(ps.sweep(60, seed=7, calibrated=True))
  assert s["kicked"] == s["runs"], s["states"]
  assert s["ey_cm"][1] <= 1.2  # p90 lateral placement error [cm]
  assert s["ex_cm"][1] <= 2.5
  assert s["shot_err_deg"][1] <= 9.0
  assert s["min_ball_dist_m"] >= 0.40  # never walks into the ball before the creep


def test_straight_penalty_kicks_right_and_reports_the_shot():
  r = attempt()
  assert r["state"] == "done", r["reason"]
  assert r["side"] == "right" and abs(r["ey"]) < 0.01 and abs(r["shot_err_deg"]) < 5.0
  info = r["info"]
  assert info["iters"] <= 4 and "placement" in info and "shot_err_deg" in info
  assert abs(info["shot_err_deg"] - r["shot_err_deg"]) < 3.0  # the robot's own estimate of the shot
  assert not r["slot_active"] and not r["estopped"]  # walker back, no e-stop


def test_ball_behind_the_robot_is_found_by_the_scan():
  r = attempt(ball=(-1.2, 0.4), target=(-3.2, 0.4))
  assert r["state"] == "done", r["reason"]


def test_robot_on_the_goal_side_walks_around_the_ball():
  r = attempt(ball=(1.2, 0.0), target=(-1.0, 0.0))  # the goal is behind the robot
  assert r["state"] == "done", r["reason"]
  assert r["min_ball_dist"] >= 0.40 and abs(r["shot_err_deg"]) < 8.0


def test_no_ball_aborts_after_a_full_scan_without_travel():
  r = attempt(ball_visible=False)
  assert r["state"] == "aborted" and "no ball found" in r["reason"]
  w = r["world"]
  assert math.hypot(w.x, w.y) < 0.02  # turned in place only


def test_explicit_left_kick_and_auto_side():
  r = attempt(params=replace(P, side="left"))
  assert r["state"] == "done" and r["side"] == "left" and r["world"].kick["side"] == "left"
  r = attempt(ball=(1.5, 0.3), target=(3.5, 0.3))
  assert r["side"] in ("right", "left") and r["state"] == "done"


@pytest.mark.parametrize("frame", ["start", "topic"])
def test_other_goal_frames(frame):
  r = attempt(robot=(0.0, 0.0, 0.4), ball=(1.0, 1.0), target=(2.5, 2.2), params=replace(P, goal_frame=frame))
  assert r["state"] == "done", r["reason"]
  assert abs(r["shot_err_deg"]) < 8.0


def test_imu_drift_is_removed_and_large_drift_aborts():
  r = attempt(imu_drift_dps=1.5)
  assert r["state"] == "done" and abs(r["info"]["imu_drift_dps"] - 1.5) < 0.3
  heading_err = r["shot_err_deg"] - 7.0 * 100.0 * r["ey"]
  assert abs(heading_err) < 1.5
  r = attempt(imu_drift_dps=5.0)
  assert r["state"] == "aborted" and "IMU yaw drifts" in r["reason"]


def test_stop_during_the_approach_gives_zero_at_once():
  r = attempt(ball=(2.0, 0.5), target=(4.0, 0.5), stop_at=5.0)
  assert r["state"] == "aborted" and r["reason"] == "test stop" and r["t"] < 5.1


def test_stop_during_the_strike_switches_back_to_the_walker():
  r = attempt()
  t_stop = r["t_strike"] + 2.0  # slot active, waiting for its init and fade-in
  r = attempt(stop_at=t_stop)
  assert r["state"] == "aborted" and r["reason"] == "test stop"
  assert not r["slot_active"] and not r["estopped"] and r["world"].kick is None


def test_lost_ball_gives_zero_travel_then_searches_then_aborts():
  world = ps.World(ps.WorldCfg(), ball=(2.0, 0.3))
  kick = PenaltyKick(replace(P, target_x=2.0))
  kick.start(0.0)
  cmds = []
  while kick.active and world.t < 60.0:
    if world.t > 3.0:
      world.cfg.ball_visible = False  # the detector dies during the approach
    cmd = kick.step(world.t, world.sense(), world.imu_yaw())
    cmds.append((world.t, kick.state, cmd))
    world.advance(cmd)
  # one re-search (a full scan), then the attempt ends
  assert kick.state == State.ABORTED and "no ball found" in kick.reason and kick.info["reacquired"] == 1
  last_seen = max(t for t, st, c in cmds if st == State.APPROACH and c.vx != 0.0)
  assert last_seen < 3.0 + P.max_obs_age + 0.3  # zero travel once the track is stale
  assert all(c.vx == 0.0 and c.vy == 0.0 for t, st, c in cmds if st == State.SEARCH)


def test_dry_run_stops_at_the_strike_point_without_the_kick_slot():
  r = attempt(params=replace(P, strike="none"))
  assert r["state"] == "done" and r["info"]["dry_run"] and "placement" in r["info"]
  assert r["t_strike"] is None and r["world"].kick is None and not r["slot_active"]


def test_walk_calibration_arithmetic():
  sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools"))
  from walk_calibration import walk_result
  r = walk_result((1.40, 0.0), (1.04, -0.01), 0.0, 0.15, 0.0, 3.0)
  assert r["efficiency"] == pytest.approx(0.8, abs=1e-3) and r["drift_per_m"] == pytest.approx(0.028, abs=1e-3)
  r = walk_result((1.40, 0.0), (1.40, -0.12), 0.0, 0.0, 0.08, 2.0)  # sidestep left 0.12 m of 0.16
  assert r["efficiency"] == pytest.approx(0.75, abs=1e-3)


def test_check_params_rejects_unsafe_settings():
  check_params(P)
  for bad in (dict(goal_frame="x"), dict(side="middle"), dict(target_x=0.1), dict(stage_x=0.4),
              dict(keep_out=0.3), dict(creep_efficiency=0.1), dict(approach_speed=0.8),
              dict(strike="run_up")):
    with pytest.raises(ValueError):
      check_params(replace(P, **bad))


# ---------------------------------------------------------------- strike sequencer


def run_strike(params=P, status=True, plays=True, end=True, out_ok=True, in_ok=True, speed=0.2,
               stop=None, t_max=20.0):
  s = StrikeSequencer(params)
  s.start(0.0, "right")
  t, sent = 0.0, []
  active, clip_t = False, None
  while not s.finished and t < t_max:
    for a in s.actions:
      sent.append(a)
      if a == ("switch", "in"):
        s.on_switch(t, "in", in_ok)
        active = in_ok
      elif a == ("switch", "out"):
        s.on_switch(t, "out", out_ok)
        active = active and not out_ok
      elif a[0] == "clip":
        clip_t = t + 0.5 if plays else None
    s.actions = []
    if stop and t >= stop[0]:
      s.abort(t, "stop", switch=stop[1])
      stop = None
    if active and status:
      playing = clip_t is not None and t >= clip_t and (not end or t < clip_t + 3.0)
      s.on_clip_status(t, "kick_right" if playing else "stand_idle")
      s.on_joint_speed(t, speed if not playing else 14.0)
    s.tick(t)
    t += 0.02
  return s, sent


def test_strike_normal_sequence():
  s, sent = run_strike()
  assert s.ok and s.phase == s.DONE
  assert sent == [("switch", "in"), ("clip", "kick_right"), ("switch", "out")]


def test_strike_switch_in_refused_needs_no_switch_back():
  s, sent = run_strike(in_ok=False)
  assert not s.ok and "refused" in s.reason and sent == [("switch", "in")]


@pytest.mark.parametrize("kw, why", [
  (dict(status=False), "no clip status"),
  (dict(plays=False), "did not start"),
  (dict(end=False), "did not end"),
  (dict(speed=3.0), "watchdog"),
])
def test_strike_faults_switch_back(kw, why):
  s, sent = run_strike(**kw)
  assert not s.ok and why in s.reason and sent[-1] == ("switch", "out") and ("estop",) not in sent


def test_strike_switch_back_refused_sends_the_emergency_stop():
  s, sent = run_strike(out_ok=False)
  assert not s.ok and sent.count(("switch", "out")) == 1 + int(P.exit_retries) and sent[-1] == ("estop",)


def test_strike_abort_with_and_without_switch():
  s, sent = run_strike(stop=(3.0, True))
  assert not s.ok and s.reason == "stop" and sent[-1] == ("switch", "out")
  s, sent = run_strike(stop=(3.0, False))  # e-stop: the controllers are not ours
  assert not s.ok and sent == [("switch", "in")]


# ---------------------------------------------------------------- node (stand-in rclpy)


class JointState(_fake_ros._Msg):
  def __init__(self, velocity=()):
    super().__init__(velocity=list(velocity))


def make_node(overrides=None, mux=1):
  ros = _fake_ros.install(overrides)
  sys.modules["sensor_msgs.msg"].JointState = JointState
  module = importlib.reload(importlib.import_module("penalty_kick_node"))
  ros.publishers["/cmd_vel"] += mux
  node = module.PenaltyKickNode()
  ros.publish("/estop_controller/stop_status", _fake_ros.String(json.dumps({"latched": False, "fault": False})))
  return ros, node


def drive(ros, world, seconds, t0=1000.0):
  """Feed the node from a penalty_sim.World and move the world with its commands."""
  slot = "neural_controller_student"
  n_clip = len(ros.published[f"/{slot}/clip"])
  for _ in range(int(round(seconds / 0.02))):
    world.slot_active = ros.controllers[slot] == "active"
    clips = ros.published[f"/{slot}/clip"]
    for msg in clips[n_clip:]:
      world.clip_req = (msg.data, world.t + 0.5)
    n_clip = len(clips)
    ros.publish("/imu_sensor_broadcaster/imu", _fake_ros.Imu(world.imu_yaw()))
    obs = world.sense()
    if obs is not None:
      ros.publish("/soccer/ball", _fake_ros.PointStamped(obs.x, obs.y, obs.stamp + t0))
    for ev in world.events():
      if ev[0] == "status":
        ros.publish(f"/{slot}/clip_status", _fake_ros.String(ev[1] + " 0"))
      elif ev[0] == "joints":
        ros.publish("/joint_states", JointState([ev[1], -ev[1]]))
    ros.tick()
    twists = ros.published["/kick_cmd_vel"]
    t = twists[-1] if twists else None
    cmd = Command(t.linear.x, t.linear.y, t.angular.z) if t is not None else Command()
    world.advance(cmd)


def node_state(ros):
  return json.loads(ros.published["/penalty_kick/state"][-1].data)


def test_node_runs_a_whole_penalty_kick():
  ros, node = make_node({"target_x": 2.0, "target_y": 0.0})
  world = ps.World(ps.WorldCfg(), ball=(1.4, 0.2))
  drive(ros, world, 0.1)
  assert ros.call("activate_penalty_kick").success
  drive(ros, world, 60.0)
  st = node_state(ros)
  assert st["state"] == "done" and node.stage == "idle", st
  assert [m.data for m in ros.published["/neural_controller_student/clip"]] == ["kick_right"]
  sw = [(r.activate_controllers, r.deactivate_controllers, r.strictness) for r in ros.switch_calls]
  assert sw == [(["neural_controller_student"], ["neural_controller_default"], 2),
                (["neural_controller_default"], ["neural_controller_student"], 2)]
  assert ros.controllers["neural_controller_default"] == "active"
  last = ros.published["/kick_cmd_vel"][-1]
  assert (last.linear.x, last.linear.y, last.angular.z) == (0.0, 0.0, 0.0)
  assert world.kick is not None and abs(world.kick["ey"]) < 0.015


@pytest.mark.parametrize("setup, why", [
  ("no_imu", "IMU"), ("teleop", "publisher"), ("no_mux", "publisher"), ("latched", "latched"),
  ("slot_active", "neural_controller_student is active"), ("bad_param", "side"),
])
def test_node_refuses_to_start(setup, why):
  ros, node = make_node({"side": "middle"} if setup == "bad_param" else None,
                        mux={"teleop": 2, "no_mux": 0}.get(setup, 1))
  world = ps.World(ps.WorldCfg(), ball=(1.4, 0.0))
  if setup != "no_imu":
    drive(ros, world, 0.1)
  if setup == "latched":
    ros.publish("/estop_controller/stop_status", _fake_ros.String(json.dumps({"latched": True})))
  if setup == "slot_active":
    ros.controllers["neural_controller_student"] = "active"
  res = ros.call("activate_penalty_kick")
  if res.success:  # the controller check answers on the next tick
    drive(ros, world, 0.1)
    res.message = node_state(ros)["message"]
  assert why in res.message and node.stage == "idle"
  assert not ros.switch_calls


def test_node_r3_stops_without_switching_and_side_buttons_set_the_next_kick():
  ros, node = make_node()
  world = ps.World(ps.WorldCfg(), ball=(2.0, 0.0))
  drive(ros, world, 0.1)
  ros.publish("/joy", _fake_ros.Joy([0, 0, 0, 0, 1]))  # L1
  ros.publish("/joy", _fake_ros.Joy([0, 0, 0, 0, 0]))
  assert node.get_parameter("side").value == "left"
  assert ros.call("activate_penalty_kick").success
  drive(ros, world, 5.0)
  assert node_state(ros)["state"] in ("search", "approach")
  ros.publish("/joy", _fake_ros.Joy([0] * 12 + [1]))  # R3
  drive(ros, world, 0.1)
  st = node_state(ros)
  assert st["state"] == "aborted" and "button 12" in st["reason"] and not ros.switch_calls
  last = ros.published["/kick_cmd_vel"][-1]
  assert (last.linear.x, last.linear.y, last.angular.z) == (0.0, 0.0, 0.0)


def test_node_deactivate_during_the_strike_switches_back():
  ros, node = make_node({"target_x": 2.0})
  world = ps.World(ps.WorldCfg(), ball=(1.2, 0.0))
  drive(ros, world, 0.1)
  assert ros.call("activate_penalty_kick").success
  for _ in range(600):
    drive(ros, world, 0.1)
    if node_state(ros)["state"] == "strike" and ros.controllers["neural_controller_student"] == "active":
      break
  assert ros.call("deactivate_penalty_kick").success
  drive(ros, world, 3.0)
  st = node_state(ros)
  assert st["state"] == "aborted" and ros.controllers["neural_controller_default"] == "active"
  assert not ros.published["/neural_controller_student/clip"] and not ros.published["/emergency_stop"]
