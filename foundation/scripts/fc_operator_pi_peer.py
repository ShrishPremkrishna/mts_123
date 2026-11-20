#!/usr/bin/env python3
"""Exercise the shipped operator stdin/stdout worker on a device-free Pi with a memory manager."""
import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", choices=("pupper-1", "pupper-2"), required=True)
    parser.add_argument("--owner", required=True)
    parser.add_argument("--trial", required=True)
    args = parser.parse_args()
    if (os.environ.get("PUPPER_FC_ISOLATED_FIXTURE") != "1" or set(dict(socket.if_nameindex()).values()) != {"lo"}
            or not re.fullmatch("[a-f0-9]{32}", args.owner) or not re.fullmatch("[a-zA-Z0-9_-]{1,64}", args.trial)):
        parser.error("requires a new isolated fixture with a valid trial and owner")
    root = Path(__file__).resolve().parents[2]
    run = root / "trials" / args.trial
    run.mkdir(mode=0o700, parents=True, exist_ok=False)
    (run / "selected-session.json").write_text(json.dumps({"session_id":args.trial}))
    sys.path.insert(0, str(root / "foundation/hardware"))
    from fc_operator import serve
    import rclpy
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import ListControllers, SwitchController
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Empty, String

    # The worker initializes its own default ROS context. Keep this independent observer and
    # the memory manager alive briefly after worker EOF so the last back-end receipt is recorded.
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx)
    with ExitStack() as stack:
        stack.callback(ctx.try_shutdown)
        manager = rclpy.create_node("controller_manager", context=ctx)
        stack.callback(manager.destroy_node)
        observations = stack.enter_context((run / "observer.jsonl").open("x", buffering=1))
        active = set()
        last_state = None
        def record(event, **data):
            print(json.dumps({"event":event, "pi_monotonic_ns":time.monotonic_ns(), **data}), file=observations, flush=True)
        def inventory(request, response):
            for name in ("gemini_walk", "trick_leg_kick", "imu_sensor_broadcaster"):
                feedback = name == "imu_sensor_broadcaster"
                response.controller.append(ControllerState(name=name, state="active" if feedback or name in active else "inactive",
                    type="imu_sensor_broadcaster/IMUSensorBroadcaster" if feedback else "neural_controller/NeuralController"))
            return response
        def switch(request, response):
            active.difference_update(request.deactivate_controllers)
            active.update(request.activate_controllers)
            record("backend_switch", activate=list(request.activate_controllers), deactivate=list(request.deactivate_controllers), active=sorted(active))
            response.ok = True
            return response
        def status(message):
            nonlocal last_state
            row = json.loads(message.data)
            state = {key:row[key] for key in ("instance_id", "epoch", "latched", "fault", "stop_required", "in_flight", "fc_operator_owner", "fc_operator_armed")}
            if state != last_state:
                record("gateway_state", sequence=row["sequence"], **state)
                last_state = state
        manager.create_service(ListControllers, "/controller_manager/list_controllers", inventory)
        manager.create_service(SwitchController, "/controller_manager/switch_controller_backend", switch)
        manager.create_subscription(String, "/estop_controller/stop_status", status, QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        manager.create_subscription(Empty, "/emergency_stop", lambda _: record("native_stop"), 10)
        manager.create_subscription(String, "/estop_controller/fc_operator_heartbeat", lambda m:record("heartbeat", proof=m.data),
                                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
        executor.add_node(manager)
        watcher = threading.Thread(target=executor.spin)
        watcher.start()
        stack.callback(watcher.join)
        stack.callback(executor.shutdown)
        binary = Path(os.environ["PUPPER_FC_FIXTURE_GATEWAY_PREFIX"]) / "lib/joy_utils/estop_controller"
        native_log = stack.enter_context((run / "gateway.log").open("x"))
        gateway = subprocess.Popen([str(binary), "--ros-args", "-p", "fc_operator_control:=true",
                                    "-p", "controller_names:=[gemini_walk,trick_leg_kick]", "-p", "switch_button_indices:=[0]"],
                                   stdout=native_log, stderr=subprocess.STDOUT)
        def stop_gateway():
            gateway.send_signal(signal.SIGINT)
            gateway.wait(timeout=5)
        stack.callback(stop_gateway)
        record("fixture", pid_namespace=os.readlink("/proc/self/ns/pid"), net_namespace=os.readlink("/proc/self/ns/net"))
        try:
            return serve(run, args.robot_id, args.owner)
        finally:
            time.sleep(1.2)
            record("fixture_finished", active=sorted(active))


if __name__ == "__main__":
    sys.exit(main())
