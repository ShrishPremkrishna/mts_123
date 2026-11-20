#!/usr/bin/env python3
"""ROS 2 bridge: UDP field state -> topics, and (optionally) drive one robot to its shot.

UNTESTED SKETCH - written without a ROS install to hand. Topic names and the
strike trigger are placeholders to agree with the movement/shooting owners.

Run on the Pupper (or any machine with rclpy on the robot's ROS domain):
  python3 field_state_bridge.py --robot striker --port 5005 [--drive]

Publishes:
  /field_state            std_msgs/String   the whole state as JSON
  /<robot>/field_pose     geometry_msgs/Pose2D
  /ball/field_pose        geometry_msgs/Pose2D   (theta unused)
With --drive:
  /cmd_vel                geometry_msgs/Twist    approach command while not ready
  /<robot>/strike         std_msgs/Empty         once, when the shot gate opens
"""
import argparse
import json
import socket

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Empty, String

MAX_AGE_S = 0.3  # stop the robot if perception goes quiet


class Bridge(Node):
    def __init__(self, robot: str, port: int, drive: bool):
        super().__init__("field_state_bridge")
        self.robot, self.drive = robot, drive
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.pub_state = self.create_publisher(String, "/field_state", 10)
        self.pub_robot = self.create_publisher(Pose2D, f"/{robot}/field_pose", 10)
        self.pub_ball = self.create_publisher(Pose2D, "/ball/field_pose", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_strike = self.create_publisher(Empty, f"/{robot}/strike", 1)
        self.last_rx = self.get_clock().now()
        self.struck = False
        self.create_timer(0.02, self.tick)

    def tick(self):
        state = None
        while True:  # drain to the newest datagram
            try:
                data, _ = self.sock.recvfrom(65535)
                state = json.loads(data)
            except BlockingIOError:
                break
        now = self.get_clock().now()
        if state is None:
            if self.drive and (now - self.last_rx).nanoseconds * 1e-9 > MAX_AGE_S:
                self.pub_cmd.publish(Twist())  # watchdog: perception lost -> stand still
            return
        self.last_rx = now
        self.pub_state.publish(String(data=json.dumps(state)))
        r, b = state["robots"].get(self.robot, {}), state["ball"]
        if r.get("visible"):
            self.pub_robot.publish(Pose2D(x=r["x"], y=r["y"], theta=r["yaw"]))
        if b.get("visible"):
            self.pub_ball.publish(Pose2D(x=b["x"], y=b["y"], theta=0.0))
        if not self.drive:
            return
        shot = state["shots"].get(self.robot, {})
        cmd = Twist()
        if shot.get("ready") and not self.struck:
            self.pub_strike.publish(Empty())  # hand over to the strike policy
            self.struck = True
        elif "cmd" in shot and not shot.get("ready"):
            self.struck = False
            cmd.linear.x, cmd.linear.y, cmd.angular.z = shot["cmd"]["vx"], shot["cmd"]["vy"], shot["cmd"]["wz"]
        self.pub_cmd.publish(cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="striker")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--drive", action="store_true")
    args = ap.parse_args()
    rclpy.init()
    rclpy.spin(Bridge(args.robot, args.port, args.drive))


if __name__ == "__main__":
    main()
