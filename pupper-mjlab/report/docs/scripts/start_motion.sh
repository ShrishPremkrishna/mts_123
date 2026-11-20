#!/usr/bin/env bash
# Software "Options": release the e-stop and re-activate the selected controller.
# The robot MOVES. Needs the gamepad connected; cannot clear a fault; no retries.
#   ./start_motion.sh [--dry-run]
MONO=${MONO:-$HOME/pupperv3-monorepo}
source /opt/ros/jazzy/setup.bash
source "$MONO/ros2_ws/install/setup.bash"
exec python3 - "$@" <<'PY'
import json, sys, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from joy_utils.srv import StartMotion

rclpy.init()
node = rclpy.create_node("operator_start_once")
cli = node.create_client(StartMotion, "/estop_controller/start_motion")
assert cli.wait_for_service(timeout_sec=5.0), "start_motion not available: is launch.sh running?"
s = {}
# Volatile QoS skips the latched sample: the challenge must be < 500 ms old.
node.create_subscription(String, "/estop_controller/stop_status",
                         lambda m: s or s.update(json.loads(m.data)),
                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
while not s:
    rclpy.spin_once(node, timeout_sec=0.05)
print({k: s[k] for k in ("latched", "fault", "selected_controller")})
if "--dry-run" in sys.argv:
    sys.exit(0)
req = StartMotion.Request(instance_id=s["instance_id"], epoch=int(s["epoch"]),
                          sequence=int(s["sequence"]), selected_controller=s["selected_controller"])
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
print(("STARTED: " if fut.result().success else "REFUSED: ") + fut.result().message)
PY
