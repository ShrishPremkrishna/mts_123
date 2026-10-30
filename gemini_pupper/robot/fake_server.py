#!/usr/bin/env python3
"""Run the real web/API routes against the fake robot, with real queue timing.

    python robot/fake_server.py --port 8001

Always binds localhost; no robot, ROS installation, camera or Gemini key is
required. Browser voice can use its separately configured Gemini connection.
This exercises tools and cancellation, never physics or native controllers.
"""
from __future__ import annotations

import argparse
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8001)
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    # Never silently replace or fall back to an already imported native runtime.
    if "rclpy" in sys.modules or "robot_api" in sys.modules:
        raise RuntimeError("fake_server requires a fresh process; ROS/robot_api is already imported")
    import test_tools  # installs fake ROS BEFORE server imports robot_api
    import server
    if server.Robot is not test_tools.Robot or server.Robot.__module__ != "robot_api":
        raise RuntimeError("fake robot bindings were not established")
    server.FAKE_MODE = True
    server.uvicorn.run(server.app, host="127.0.0.1", port=args.port)


if __name__ == "__main__":
    main()
