#!/usr/bin/env python3
"""Real ROS wiring checks, only in a network-disabled, device-free container.

Uses the actual Start gateway and cmd_vel_mux executables, production resident
Start attachment/session queue and body guard node. The controller-manager
backend and velocity consumer are explicitly fake; no hardware is initialized.
Guard death proves publication silence only, not the native watchdog or braking.
The guard source is synthetic at 20 Hz with 5 ms declared capture age; passing
three clear frames here does not establish clearance at actual Pi model latency.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace


def main():
    if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1":
        raise SystemExit("Requires isolated ROS test container; never run on a robot.")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=("start", "guard"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--gateway", default="/gateway/build/estop_controller")
    parser.add_argument("--mux", default="/evidence/mux-build/cmd_vel_mux_node")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rclpy.init()
    executor = SingleThreadedExecutor()
    processes, streams = [], []
    report = {"case": args.case, "passed": False, "hardware": False,
              "native_watchdog_observed": False, "groups": []}
    began = time.monotonic()

    def launch(name, command):
        stream = (args.out / (name + ".log")).open("w")
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def pump(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=min(0.01, until-time.monotonic()))

    def wait(test, why, seconds=4.0):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.01)
            if test():
                return
        raise AssertionError(why)

    try:
        if args.case == "start":
            from controller_manager_msgs.msg import ControllerState
            from controller_manager_msgs.srv import ListControllers, SwitchController
            from sensor_msgs.msg import Joy
            from foundation.hardware.software_start import attach
            from final_project.behaviors.tools import movement_trial as movement

            class FakeManager(Node):
                def __init__(self):
                    super().__init__("fc_start_fake_manager_and_client")
                    self.active, self.switches, self.reads = set(), [], 0
                    self.joy_count = 0
                    self.create_service(SwitchController, "/controller_manager/switch_controller_backend", self.switch)
                    self.create_service(ListControllers, "/controller_manager/list_controllers", self.inventory)
                    self.joy = self.create_publisher(Joy, "/joy", 10)
                    self.create_timer(0.02, self.neutral)
                    # Idle session surface only; no movement/native identity is simulated as proven.
                    self.clock = time.monotonic
                    self.now = self.clock
                    self.lock = threading.RLock()
                    self.request = SimpleNamespace(profile="walk")
                    self.phase, self.stop_requested, self.record = "waiting", "", {}
                    self.gateway_authority = None
                def neutral(self):
                    msg = Joy(); msg.buttons = [0] * 13
                    self.joy.publish(msg); self.joy_count += 1
                def switch(self, req, res):
                    row = {"activate": list(req.activate_controllers), "deactivate": list(req.deactivate_controllers),
                           "strictness": req.strictness, "time": time.monotonic()}
                    self.switches.append(row)
                    res.ok = not (req.strictness == req.STRICT and any(n in self.active for n in req.activate_controllers))
                    if res.ok:
                        self.active.difference_update(req.deactivate_controllers)
                        self.active.update(req.activate_controllers)
                    return res
                def inventory(self, req, res):
                    self.reads += 1
                    for name, kind in [("neural_controller_default", "neural_controller/NeuralController"),
                                       ("neural_controller_student", "neural_controller/NeuralController"),
                                       ("joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster"),
                                       ("imu_sensor_broadcaster", "imu_sensor_broadcaster/IMUSensorBroadcaster")]:
                        state = ControllerState(); state.name, state.type = name, kind
                        state.state = "active" if name in self.active else "inactive"
                        res.controller.append(state)
                    return res
                def cancel_start(self, reason):
                    raise AssertionError("Queued Stop must not submit or cancel a never-dispatched Start: " + reason)

            node = FakeManager(); executor.add_node(node)
            caller = attach(node)
            wire_requests = []
            original_call = caller.client.call_async
            def observe_call(request):
                wire_requests.append({k: getattr(request, k) for k in
                                      ("instance_id", "epoch", "sequence", "selected_controller")})
                return original_call(request)
            caller.client.call_async = observe_call
            launch("gateway", [args.gateway, "--ros-args", "-p", "switch_button_indices:=[0,1]", "-p",
                               "controller_names:=[neural_controller_default,neural_controller_student]"])
            def ready():
                return node.joy_count >= 3 and caller._problem(time.monotonic()) is None
            wait(ready, "fresh gateway/neutral Joy/service discovery")
            ticket = caller.capture(); base = len(node.switches)
            payload = caller.send(ticket)
            try:
                caller.send(ticket)
            except ValueError:
                pass
            else:
                raise AssertionError("Duplicate ticket was submitted")
            results = []
            def poll():
                value = caller.poll()
                if value is not None: results.append(value)
                return bool(results)
            wait(poll, "Start response")
            assert results[-1]["success"] and len(wire_requests) == 1, results
            assert wire_requests[0] == payload == {k: getattr(ticket, k) for k in wire_requests[0]}
            # Deliberate Start inherits the gateway's Options/BEST_EFFORT path.
            # Generic STRICT requests are a distinct service contract.
            assert len(node.switches) == base + 1, node.switches
            assert node.switches[-1]["strictness"] == SwitchController.Request.BEST_EFFORT, node.switches
            assert node.active == {"neural_controller_default"}
            report["groups"].append("real_StartMotion_fields_single_call_and_selected_activation")
            wait(ready, "settled active gateway")
            base, reads = len(node.switches), node.reads
            caller.send(caller.capture()); results.clear(); wait(poll, "already-active response")
            assert results[-1]["success"] and len(wire_requests) == 2
            assert len(node.switches) == base and node.reads > reads
            report["groups"].append("already_active_fresh_inventory_no_second_STRICT")
            wait(ready, "gateway ready for queued input")
            node.gateway_authority = (caller.state["instance_id"], caller.state["epoch"])
            inbox = movement.SessionInbox(node.clock)
            events, injected = [], False
            def spin_once(_node, timeout_sec):
                nonlocal injected
                executor.spin_once(timeout_sec=timeout_sec)
                if not injected:
                    injected = True
                    inbox.put({"command": "start"})
                    inbox.put({"command": "stop", "reason": "test urgent queued Stop"})
            calls = len(wire_requests)
            rc = movement.run_session(node, {"test_only": True}, lambda _: True,
                                      inbox=inbox, emit=events.append, spin_once=spin_once, start_client=caller)
            assert rc == 0 and len(wire_requests) == calls and len(node.switches) == base
            assert any(e["event"] == "stopping" for e in events)
            assert not any(e["event"] == "start_submitted" for e in events)
            queued = inbox.get()
            assert queued[1]["command"] == "start" and "_start_ticket" in queued[1], queued
            report["groups"].append("actual_session_queue_Stop_preempts_captured_Start_without_dispatch")
            report.update(wire_requests=wire_requests, backend_switches=node.switches,
                          inventory_reads=node.reads, session_events=events,
                          limitation="Idle session node surface is fake; actual attach, ROS client, message type, gateway, backend DDS and run_session/SessionInbox are exercised. Native identity and robot readiness are not proven.")
        else:
            class Driver(Node):
                def __init__(self):
                    super().__init__("fc_guard_smoke_fake_source_consumer")
                    self.desired = 0.0; self.commands_enabled = True; self.frames_enabled = True
                    self.near = False; self.seq = 0; self.received = []; self.statuses = []
                    self.raw = []; self.last_frame = None; self.last_command = None
                    self.command_pub = self.create_publisher(Twist, "/kick_cmd_vel", 10)
                    self.frame_pub = self.create_publisher(String, "/fc/obstacle_detections", 10)
                    self.create_subscription(Twist, "/cmd_vel", lambda msg: self.received.append((time.monotonic(), msg.linear.x)), 10)
                    self.create_subscription(Twist, "/fc/requested_cmd_vel", lambda msg: self.raw.append((time.monotonic(), msg.linear.x)), 10)
                    self.create_subscription(String, "/fc/body_command_guard/status", lambda msg: self.statuses.append((time.monotonic(), json.loads(msg.data))), 10)
                    self.create_timer(0.02, self.commands)
                    self.create_timer(0.05, self.frames)
                def commands(self):
                    if self.commands_enabled:
                        msg = Twist(); msg.linear.x = self.desired; self.command_pub.publish(msg)
                        self.last_command = time.monotonic()
                def frames(self):
                    if self.frames_enabled:
                        self.seq += 1
                        stamp = self.get_clock().now().nanoseconds - 5_000_000
                        payload = {"schema": "fc_camera_obstacles_v1", "producer_instance": "isolated_smoke",
                                   "sequence": self.seq, "captured_at_ros_ns": stamp, "valid": True, "healthy": True,
                                   "format": "normalized_xyxy_v1", "image_width": 700, "image_height": 572,
                                   "horizontal_fov_deg": 220,
                                   "detections": [{"box": [0.4, 0.3, 0.6, 0.85], "confidence": 0.9, "class_id": "tower"}] if self.near else []}
                        self.frame_pub.publish(String(data=json.dumps(payload))); self.last_frame = time.monotonic()
            node = Driver(); executor.add_node(node)
            def mux_start(suffix):
                return launch("mux-"+suffix, [args.mux, "--ros-args", "-p", "inputs:=[/kick_cmd_vel]", "-p", "timeout_ms:=150", "-r", "/cmd_vel:=/fc/requested_cmd_vel"])
            mux = mux_start("initial")
            guard = launch("guard", [sys.executable, "-m", "final_project.behaviors.body_command_guard"])
            def output(value, since):
                return any(at >= since and abs(x-value) < 1e-6 for at,x in node.received)
            def cleared():
                return node.statuses and node.statuses[-1][1]["state"] == "ready"
            wait(cleared, "camera fresh clearance with zero input")
            changed = time.monotonic(); node.desired = 0.2
            wait(lambda: output(.2, changed), "real mux/guard nonzero transport")
            report["groups"].append("actual_mux_guard_nonzero_transport")
            changed = time.monotonic(); node.near = True
            wait(lambda: output(0., changed), "near obstacle zero", .5)
            report["groups"].append("near_obstacle_zero")
            node.near = False; pump(.5); changed = time.monotonic(); pump(.2)
            assert not output(.2, changed), "Pre-hold mux command revived"
            report["groups"].append("clearance_cannot_revive_prehold_nonzero")
            def resume():
                node.desired = 0.; node.commands_enabled = True; node.frames_enabled = True; node.near = False
                wait(cleared, "post-hold zero and fresh clearance")
                pump(.08); changed = time.monotonic(); node.desired = .2
                wait(lambda: output(.2, changed), "fresh deliberate post-hold motion")
            resume()
            changed = time.monotonic(); node.frames_enabled = False
            wait(lambda: output(0., changed+.30), "silent camera expires to zero", .7)
            report["groups"].append("stale_camera_zero_while_upstream_continues")
            resume()
            changed = time.monotonic(); node.commands_enabled = False
            wait(lambda: output(0., changed+.10), "actual mux upstream timeout zero", .5)
            assert any(at >= changed and x == 0 for at,x in node.raw)
            report["groups"].append("actual_mux_upstream_silence_zero")
            resume()
            changed = time.monotonic(); mux.terminate(); mux.wait(timeout=3)
            wait(lambda: output(0., changed+.10), "guard receipt watchdog after mux death", .5)
            report["groups"].append("mux_death_guard_input_expiry_zero")
            node.desired = 0.; mux = mux_start("restart"); pump(.3); resume()
            guard.kill(); guard.wait(timeout=3); pump(.08)
            last_count = len(node.received); last_value = node.received[-1][1]
            pump(.4)
            assert len(node.received) == last_count and last_value == .2
            report["groups"].append("guard_death_publication_silence_observed_native_watchdog_unverified")
            report.update(guard_death_last_received_command=last_value,
                          raw_mux_samples=node.raw, command_samples=node.received, guard_status=node.statuses,
                          synthetic_camera={"frequency_hz": 20, "declared_capture_age_s": .005,
                                            "production_max_frame_age_s": .35, "clock_offset_reserve_s": .05,
                                            "usable_capture_age_at_stable_clocks_s": .30,
                                            "actual_model_inference_exercised": False,
                                            "pi_5hz_clearance_established": False},
                          limitation="Fake consumer has no native watchdog. Abrupt guard death leaves its last received nonzero command unchanged; actual selected native watchdog response is explicitly unverified here.")
        report["passed"] = True
    except Exception as exc:
        import traceback
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["traceback"] = traceback.format_exc()
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=3)
        report["child_exit_codes"] = [p.returncode for p in processes]
        report["all_children_exited"] = all(p.poll() is not None for p in processes)
        if args.case == "start" and "node" in locals():
            report["backend_switches"] = node.switches
            report["inventory_reads"] = node.reads
            report["wire_requests"] = wire_requests
        executor.shutdown()
        for node in list(executor.get_nodes()): node.destroy_node()
        rclpy.try_shutdown()
        for stream in streams: stream.close()
        report["elapsed_seconds"] = time.monotonic()-began
        (args.out/"result.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({k: report[k] for k in ("case", "passed", "groups", "all_children_exited", "elapsed_seconds")}))
    if not report["passed"]: print(report.get("error"), file=sys.stderr)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
