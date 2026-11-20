#!/usr/bin/env python3
"""Standing check vs sim: per-leg offset from the policy's default pose, left
load share (|effort| proxy) and IMU roll/pitch, averaged over a window.
  ./stand_check.py <policy.json> [--seconds 5]      (source ROS first; read-only)
"""
import argparse, json, math
import numpy as np
import rclpy
from sensor_msgs.msg import Imu, JointState

JOINTS = [f"leg_{leg}_{i}" for leg in ("front_r", "front_l", "back_r", "back_l") for i in (1, 2, 3)]
LEGS = {"FR": [0, 1, 2], "FL": [3, 4, 5], "BR": [6, 7, 8], "BL": [9, 10, 11]}

ap = argparse.ArgumentParser()
ap.add_argument("policy")
ap.add_argument("--seconds", type=float, default=5.0)
a = ap.parse_args()
default = np.array(json.load(open(a.policy))["default_joint_pos"])

pos, eff, rp = [], [], []
def on_joints(m):
    idx = [m.name.index(j) for j in JOINTS]
    pos.append([m.position[i] for i in idx])
    if len(m.effort) == len(m.name):
        eff.append([abs(m.effort[i]) for i in idx])
def on_imu(m):
    q = m.orientation
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    rp.append((math.degrees(roll), math.degrees(pitch)))

rclpy.init()
node = rclpy.create_node("stand_check")
node.create_subscription(JointState, "/joint_states", on_joints, 10)
node.create_subscription(Imu, "/imu_sensor_broadcaster/imu", on_imu, 10)
end = node.get_clock().now().nanoseconds + a.seconds * 1e9
while node.get_clock().now().nanoseconds < end:
    rclpy.spin_once(node, timeout_sec=0.1)
assert pos, "no /joint_states received: is the stack running?"

off = np.mean(pos, axis=0) - default
print(f"{len(pos)} joint samples over {a.seconds:.0f} s")
for leg, ix in LEGS.items():
    tag = "   <- sim bumpy: ~0.38 rad" if leg == "BL" else ""
    print(f"  {leg} offset [hip_fa hip_side knee] = {np.round(off[ix], 3)}  |norm| {np.linalg.norm(off[ix]):.3f} rad{tag}")
if eff:
    e = np.mean(eff, axis=0)
    left = e[LEGS["FL"] + LEGS["BL"]].sum() / e.sum()
    print(f"  left share of |effort|: {100 * left:.1f}%   (sim bumpy ~59%; ~0 effort = feet off ground)")
if rp:
    r, p = np.mean(rp, axis=0)
    print(f"  IMU roll {r:+.2f} deg, pitch {p:+.2f} deg")
