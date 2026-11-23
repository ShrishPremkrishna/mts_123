#!/usr/bin/env python3
"""Print the IMU tilt and gyro rate over one second as JSON: {"tilt_deg", "gyro_rad_s", "n"}.
A Pupper that stands still has a tilt below approximately 3 degrees and a gyro rate below 0.03 rad/s.
Larger values at rest mean that the IMU orientation drifts (handoff 2026-11-16-fr-leg-jiggle.md)."""
import json
import math
import statistics
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

rclpy.init()
node = rclpy.create_node("imu_health")
tilt, gyro = [], []


def on_imu(m):
    q, g = m.orientation, m.angular_velocity
    tilt.append(math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y))))))
    gyro.append(math.sqrt(g.x ** 2 + g.y ** 2 + g.z ** 2))


node.create_subscription(Imu, "/imu_sensor_broadcaster/imu", on_imu, qos_profile_sensor_data)
end = time.monotonic() + 1.5
while time.monotonic() < end and len(tilt) < 400:
    rclpy.spin_once(node, timeout_sec=0.05)
print(json.dumps({"tilt_deg": round(statistics.median(tilt), 1) if tilt else None,
                  "gyro_rad_s": round(statistics.median(gyro), 3) if gyro else None, "n": len(tilt)}))
