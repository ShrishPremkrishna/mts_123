#!/usr/bin/env python3
"""Measure the walker's efficiency and sideways drift, with the ball as a landmark.

Put the ball 1.2 to 1.5 m straight ahead of the standing robot (X walker
active). Run penalty_kick.launch.py with detector:=true (its mux reads
/kick_cmd_vel) and keep the penalty kick idle. The script:
  1. averages /soccer/ball for --measure-s with the robot still,
  2. publishes (--vx, --vy) on /kick_cmd_vel at 20 Hz for --seconds, then zero,
  3. waits --settle-s and averages again (IMU yaw corrects a turn),
  4. prints efficiency = achieved / commanded travel, and the drift.

    python3 tools/walk_calibration.py --vx 0.15 --seconds 3   # creep_efficiency, walk_drift
    python3 tools/walk_calibration.py --vx 0.30 --seconds 2   # speed_efficiency
    python3 tools/walk_calibration.py --vy 0.08 --seconds 2   # correction steps

Repeat each 3 times and use the mean. R3 or a face button on /joy stops the walk.
"""

from __future__ import annotations

import argparse
import math
import sys
import time


def walk_result(before, after, dyaw: float, vx: float, vy: float, seconds: float) -> dict:
  """before/after: mean ball (x, y) in the robot frame; dyaw: robot turn [rad]."""
  c, s = math.cos(dyaw), math.sin(dyaw)
  ax, ay = c * after[0] - s * after[1], s * after[0] + c * after[1]  # in the start frame
  dx, dy = before[0] - ax, before[1] - ay  # robot travel = opposite of the ball's
  cx, cy = vx * seconds, vy * seconds
  cc = cx * cx + cy * cy
  along = (dx * cx + dy * cy) / math.sqrt(cc)
  across = (-dx * cy + dy * cx) / math.sqrt(cc)  # + = left of the commanded direction
  return dict(travel=[round(dx, 3), round(dy, 3)], efficiency=round(along / math.sqrt(cc), 3),
              drift_per_m=round(across / along, 3) if abs(along) > 1e-3 else float("nan"),
              turn_deg=round(math.degrees(dyaw), 1))


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  ap.add_argument("--vx", type=float, default=0.15)
  ap.add_argument("--vy", type=float, default=0.0)
  ap.add_argument("--seconds", type=float, default=3.0)
  ap.add_argument("--measure-s", type=float, default=3.0)
  ap.add_argument("--settle-s", type=float, default=1.0)
  ap.add_argument("--topic", default="/kick_cmd_vel")
  args = ap.parse_args(argv)
  if math.hypot(args.vx, args.vy) < 0.03 or abs(args.vx) > 0.35 or abs(args.vy) > 0.2 \
      or not 0.5 <= args.seconds <= 5.0 or args.vx < 0:
    ap.error("need 0 <= vx <= 0.35, |vy| <= 0.2, speed >= 0.03 m/s and 0.5 <= seconds <= 5")

  import rclpy
  from geometry_msgs.msg import PointStamped, Twist
  from sensor_msgs.msg import Imu, Joy

  rclpy.init()
  node = rclpy.create_node("walk_calibration")
  state = {"ball": [], "yaw": None, "stop": False, "record": False}

  def on_ball(m):
    if state["record"]:
      state["ball"].append((m.point.x, m.point.y))

  def on_imu(m):
    q = m.orientation
    state["yaw"] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

  def on_joy(m):
    if any(i < len(m.buttons) and m.buttons[i] for i in (0, 1, 2, 3, 12)):
      state["stop"] = True

  node.create_subscription(PointStamped, "/soccer/ball", on_ball, 10)
  node.create_subscription(Imu, "/imu_sensor_broadcaster/imu", on_imu, 10)
  node.create_subscription(Joy, "/joy", on_joy, 10)
  pub = node.create_publisher(Twist, args.topic, 10)

  def spin_for(seconds, twist=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not state["stop"]:
      if twist is not None:
        pub.publish(twist)
      rclpy.spin_once(node, timeout_sec=0.05)

  def measure():
    state["ball"], state["record"] = [], True
    spin_for(args.measure_s)
    state["record"] = False
    pts = state["ball"]
    if len(pts) < 5:
      return None
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)

  try:
    spin_for(1.0)
    before, yaw0 = measure(), state["yaw"]
    if before is None or yaw0 is None:
      print("no ball or no IMU: is the detector running and the ball in view?")
      return 1
    travel = math.hypot(args.vx, args.vy) * args.seconds
    if before[0] - args.vx * args.seconds < 0.55 or abs(before[1]) > 0.3:
      print(f"ball at {before[0]:.2f}, {before[1]:.2f} m: put it 1.2 to 1.5 m straight ahead")
      return 1
    print(f"before: ball {before[0]:.3f}, {before[1]:.3f} m; walking {travel:.2f} m commanded")
    twist = Twist()
    twist.linear.x, twist.linear.y = args.vx, args.vy
    end = time.monotonic() + args.seconds
    while time.monotonic() < end and not state["stop"]:
      pub.publish(twist)
      spin_for(0.05)
    for _ in range(5):
      pub.publish(Twist())
      spin_for(0.02)
    if state["stop"]:
      print("stopped by the joystick")
      return 1
    spin_for(args.settle_s)
    after, yaw1 = measure(), state["yaw"]
    if after is None:
      print("ball not seen after the walk")
      return 1
    dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    print(f"after:  ball {after[0]:.3f}, {after[1]:.3f} m")
    print(walk_result(before, after, dyaw, args.vx, args.vy, args.seconds))
    return 0
  finally:
    for _ in range(3):
      pub.publish(Twist())
      time.sleep(0.05)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
  sys.exit(main())
