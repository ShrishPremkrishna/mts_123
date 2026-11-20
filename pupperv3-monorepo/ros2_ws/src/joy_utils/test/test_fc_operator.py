"""Exercise the compiled FC gateway with real DDS and a memory-only manager."""
import json
import signal
import subprocess
import sys
import threading
import time

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from joy_utils.srv import FcStartMotion, StartMotion
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String


def wait(predicate, why, seconds=3):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if value := predicate():
            return value
        time.sleep(.005)
    raise AssertionError(why)


def main(gateway):
    rclpy.init()
    manager = rclpy.create_node("fc_test_manager")
    client = rclpy.create_node("fc_test_operator")
    executors = []
    for node in (manager, client):
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executors.append(executor)
    status, active, activations, stops = {}, set(), [], []
    inventory_gate, switch_gate = threading.Event(), threading.Event()
    inventory_gate.set()
    switch_gate.set()

    def inventory(request, response):
        assert inventory_gate.wait(3), "inventory gate timed out"
        for name in ("gemini_walk", "trick_leg_kick"):
            response.controller.append(ControllerState(name=name, state="active" if name in active else "inactive",
                                                       type="neural_controller/NeuralController"))
        return response

    def switch(request, response):
        if request.activate_controllers:
            activations.append(time.monotonic())
            assert switch_gate.wait(3), "switch gate timed out"
        active.difference_update(request.deactivate_controllers)
        active.update(request.activate_controllers)
        response.ok = True
        return response

    manager.create_service(ListControllers, "/controller_manager/list_controllers", inventory)
    manager.create_service(SwitchController, "/controller_manager/switch_controller_backend", switch)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def observe(msg):
        nonlocal status
        status = json.loads(msg.data)
    client.create_subscription(String, "/estop_controller/stop_status", observe, qos)
    client.create_subscription(Empty, "/emergency_stop", lambda _: stops.append(time.monotonic()), 10)
    beat_pub = client.create_publisher(String, "/estop_controller/fc_operator_heartbeat",
                                       QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    stop_pub = client.create_publisher(Empty, "/estop_controller/request_stop", 10)
    joy = client.create_publisher(Joy, "/joy", 10)
    start = client.create_client(FcStartMotion, "/estop_controller/fc_start_motion")
    legacy = client.create_client(StartMotion, "/estop_controller/start_motion")
    threads = [threading.Thread(target=e.spin) for e in executors]
    for thread in threads:
        thread.start()
    process = subprocess.Popen([gateway, "--ros-args", "-p", "fc_operator_control:=true",
                                "-p", "controller_names:=[gemini_walk,trick_leg_kick]",
                                "-p", "switch_button_indices:=[0]"])
    owner, other = "a"*32, "b"*32

    def stopped():
        current = status
        return (current.get("latched") and not current["stop_required"] and not current["in_flight"]
                and not current["fc_operator_armed"] and not active)

    def beat(state, identity=owner):
        beat_pub.publish(String(data=f"{state['instance_id']} {state['epoch']} {state['sequence']} {identity}"))

    def acquire():
        first = status["sequence"]
        def owned():
            current = status
            assert not current["fault"], f"native Stop fault during acquisition: {current}"
            beat(current)
            if current["fc_operator_owner"] == owner and current["sequence"] >= first + 2:
                return current
        return wait(owned, "operator proof missing")

    def request(state, identity=owner):
        return start.call_async(FcStartMotion.Request(
            instance_id=state["instance_id"], epoch=state["epoch"], sequence=state["sequence"],
            selected_controller=state["selected_controller"], owner=identity))

    def result(future):
        wait(future.done, "missing Start response")
        return future.result().success

    def stand_with_heartbeat(state):
        future = request(state)
        def keep_alive():
            beat(status)
            return future.done()
        wait(keep_alive, "healthy Start acknowledgement")
        response = future.result()
        assert response.success, response.message

    try:
        wait(lambda: stopped() and start.service_is_ready() and legacy.service_is_ready() and beat_pub.get_subscription_count()
             and joy.get_subscription_count() and stop_pub.get_subscription_count(), "startup")
        assert status["fc_operator_control"] and status["fc_operator_lease_ms"] == 750
        # Exercise every pre-lease activation path with a connected controller.
        # Both Options and a face selection leave initialization latched, including
        # when a valid legacy Start transaction cites a fresh physical Joy report.
        def buttons(*pressed):
            values = [0] * 13
            for index in pressed:
                values[index] = 1
            joy.publish(Joy(buttons=values))

        buttons(9, 0)
        time.sleep(.15)
        buttons()
        state = status
        old_start = legacy.call_async(StartMotion.Request(
            instance_id=state["instance_id"], epoch=state["epoch"], sequence=state["sequence"],
            selected_controller=state["selected_controller"]))
        assert not result(old_start) and "dashboard" in old_start.result().message
        assert stopped() and not activations and not status["fc_operator_owner"], "gamepad bypassed dashboard ownership"
        assert not result(request(status)), "FC Start without browser proof"
        ready = acquire()
        assert not result(request(ready, other)), "wrong owner captured Start"
        stand_with_heartbeat(ready)
        assert active == {"gemini_walk"}
        assert not result(request(ready)), "replayed Start admitted"
        # Sustain walking with successive, current challenges, including malformed Joy after unplug.
        until = time.monotonic() + 1.
        while time.monotonic() < until:
            beat(status)
            joy.publish(Joy())
            time.sleep(.075)
        assert status["fc_operator_armed"] and not status["latched"]
        # Face/Options edges during a lease cannot reactivate or switch the walker.
        buttons(0, 9)
        time.sleep(.10)
        buttons()
        assert len(activations) == 1 and active == {"gemini_walk"}
        frozen = status
        previous = len(stops)
        began = time.monotonic()
        while time.monotonic() - began < 1.3 and not stopped():
            beat(frozen)         # replay cannot extend its original Pi deadline
            beat(status, other) # new sequences from a second owner cannot renew
            time.sleep(.04)
        assert stopped() and len(stops) > previous and status["epoch"] > frozen["epoch"]
        print(f"Replayed/foreign heartbeat to native Stop: {stops[previous]-began:.3f}s", flush=True)
        beat(frozen)
        assert not result(request(frozen)) and len(activations) == 1

        # Explicit software Stop fences both an old click and its queued heartbeat.
        ready = acquire()
        stop_pub.publish(Empty())
        wait(lambda: stopped() and status["epoch"] > ready["epoch"], "software Stop cleanup")
        beat(ready)
        assert not result(request(ready)) and len(activations) == 1

        # When the heartbeat is old at admission, lease expiry beats pending inventory.
        acquire()
        beat(status)
        time.sleep(.35)
        inventory_gate.clear()
        queued_status = status
        queued = request(queued_status)
        wait(lambda: status["fc_operator_armed"] or queued.done(), "Start admission while waiting for inventory")
        if queued.done():
            raise AssertionError(f"expected pending Start: {queued.result()} {queued_status} {status}")
        assert not result(queued), "Start dispatched after loss while pending inventory"
        wait(lambda: status["epoch"] > queued_status["epoch"], "pending expiry did not fence old epoch")
        inventory_gate.set()
        wait(stopped, "queued expiration cleanup")
        assert len(activations) == 1

        # A remote switch may finish after expiry. Wait for its corrective deactivation.
        ready = acquire()
        switch_gate.clear()
        late = request(ready)
        wait(lambda: len(activations) == 2, "backend did not begin")
        wait(lambda: status["latched"] and status["epoch"] > ready["epoch"], "no Stop during delayed backend")
        switch_gate.set()
        assert not result(late)
        wait(stopped, "late activation left a motion owner")

        # Physical Stop is effective during a healthy leased run. Empty Joy cannot release a held Stop.
        ready = acquire()
        stand_with_heartbeat(ready)
        joy.publish(Joy(buttons=[0]*12+[1]))
        wait(lambda: stopped() and status["epoch"] > ready["epoch"], "physical Stop cleanup")
        joy.publish(Joy())
        ready = acquire()
        assert not result(request(ready)), "malformed Joy released physical Stop"
        joy.publish(Joy(buttons=[0]*13))
        time.sleep(.06)
        stand_with_heartbeat(acquire())
        stop_pub.publish(Empty())
        wait(stopped, "final cleanup")
        assert not status["fault"]
        print("FC DDS passed with two controller inventories, owned browser proof and physical Stop.", flush=True)
    except BaseException:
        print("Last gateway state:", status, "covered active:", active, flush=True)
        raise
    finally:
        inventory_gate.set()
        switch_gate.set()
        process.send_signal(signal.SIGINT)
        process.wait(4)
        for executor in executors:
            executor.shutdown()
        for thread in threads:
            thread.join()
        client.destroy_node()
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1])
