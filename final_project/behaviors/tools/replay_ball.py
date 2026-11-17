#!/usr/bin/env python3
"""Publish a scripted ball for stand tests (robot lifted, feet off the ground).

The ball starts at (x0, y) in the base_link yaw-only frame and approaches at
--speed (as if the robot ran at it), then stays at --stop-x. This exercises a
kick node's full cycle on /soccer/ball with no camera. Never use it with the
robot on the floor.

    python3 tools/replay_ball.py --x0 1.0 --speed 0.8
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import PointStamped


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--x0", type=float, default=1.0)
  ap.add_argument("--y", type=float, default=0.0)
  ap.add_argument("--speed", type=float, default=0.8, help="approach speed [m/s]; 0 = still")
  ap.add_argument("--stop-x", type=float, default=0.2)
  ap.add_argument("--rate", type=float, default=20.0)
  ap.add_argument("--seconds", type=float, default=15.0)
  args = ap.parse_args()
  rclpy.init()
  node = rclpy.create_node("replay_ball")
  pub = node.create_publisher(PointStamped, "/soccer/ball", 10)
  start = time.monotonic()
  try:
    while time.monotonic() - start < args.seconds:
      t = time.monotonic() - start
      msg = PointStamped()
      msg.header.stamp = node.get_clock().now().to_msg()
      msg.header.frame_id = "base_link_yaw"
      msg.point.x = max(args.stop_x, args.x0 - args.speed * t)
      msg.point.y = args.y
      pub.publish(msg)
      time.sleep(1.0 / args.rate)
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
  main()
