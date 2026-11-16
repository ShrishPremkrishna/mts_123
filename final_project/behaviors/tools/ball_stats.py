#!/usr/bin/env python3
"""Mean and spread of /soccer/ball for a few seconds, for calibration.

    python3 ball_stats.py [seconds]

Place the ball at a measured position (ball centre from the body centre, x
forward, y left), keep the robot still, and compare.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped


def main():
  seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
  rclpy.init()
  node = rclpy.create_node("ball_stats")
  got = []
  node.create_subscription(PointStamped, "/soccer/ball", lambda m: got.append(
    (time.time(), m.point.x, m.point.y, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), 50)
  end = time.time() + seconds
  while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
  if not got:
    print("no /soccer/ball messages")
    return
  n = len(got)
  mx = sum(g[1] for g in got) / n
  my = sum(g[2] for g in got) / n
  sx = math.sqrt(sum((g[1] - mx) ** 2 for g in got) / n)
  sy = math.sqrt(sum((g[2] - my) ** 2 for g in got) / n)
  age = sum(g[0] - g[3] for g in got) / n
  print(f"{n} msgs ({n / seconds:.1f} Hz)  x {mx:.3f} m (sd {sx:.3f})  y {my:.3f} m (sd {sy:.3f})  "
        f"range {math.hypot(mx, my):.3f} m  bearing {math.degrees(math.atan2(my, mx)):+.1f} deg  "
        f"mean age {age * 1000:.0f} ms")


if __name__ == "__main__":
  main()
