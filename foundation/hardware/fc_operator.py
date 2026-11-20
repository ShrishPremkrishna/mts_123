"""Relay fresh browser responses to the Pi gateway. The gateway owns the watchdog."""
import collections
import fcntl
import json
import os
from pathlib import Path
import re
import select
import sys
import time


def valid_owner(owner):
    return isinstance(owner, str) and re.fullmatch(r"[0-9a-f]{32}", owner) is not None


def attached(run, session_id, owner):
    """An exited SSH relay cannot authorize a match with its old receipt."""
    if not valid_owner(owner):
        return False
    try:
        with (run / "fc-operator.lock").open("r") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return False
            except BlockingIOError:
                receipt = json.loads((run / "fc-operator-standing.json").read_text())
                ticks = Path("/proc", str(receipt["pid"]), "stat").read_text().rsplit(")", 1)[1].split()[19]
                return (receipt["owner"] == owner and receipt["session_id"] == session_id
                        and receipt["start_ticks"] == ticks)
    except (OSError, KeyError, ValueError, IndexError):
        return False


def serve(run, robot_id, owner):
    """Called with verified local ROS environment and a homed, owned board."""
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from joy_utils.srv import FcStartMotion
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from fc_session import read, write_new
    from software_start import attach, BLOCKED, STATUS_MAX_AGE_S

    if not valid_owner(owner):
        raise ValueError("invalid browser owner")
    run = Path(run)
    session_id = read(run / "selected-session.json")["session_id"]
    receipt = run / "fc-operator-standing.json"
    with (run / "fc-operator.lock").open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        receipt.unlink(missing_ok=True)
        rclpy.init()
        node = rclpy.create_node("fc_laptop_operator", enable_rosout=False, start_parameter_services=False)
        gateway = attach(node, selected_controller="gemini_walk")
        heart = node.create_publisher(String, "/estop_controller/fc_operator_heartbeat",
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        start = node.create_client(FcStartMotion, "/estop_controller/fc_start_motion")
        inventory = node.create_client(ListControllers, "/controller_manager/list_controllers")
        offers = collections.OrderedDict()
        echoed = set()
        pending = inventory_request = None
        touched = standing = False
        seen = previous_ready = None
        buffer = bytearray()
        started = time.monotonic()
        ack_deadline = None

        def emit(event, **values):
            print(json.dumps(dict(event=event, robot_id=robot_id, **values)), flush=True)

        def stop(reason):
            receipt.unlink(missing_ok=True)
            if gateway.stop_result is not None:
                return
            gateway.request_stop(reason)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.02)
                result = gateway.poll_stop()
                if result is not None:
                    emit("operator_stop", confirmed=result.get("confirmed") is True and not result.get("uncertain"), reason=reason)
                    break

        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.01)
                now = time.monotonic()
                state = gateway.state
                fresh = state is not None and gateway.observed_at is not None and 0 <= now-gateway.observed_at < STATUS_MAX_AGE_S
                if fresh:
                    if state.get("fc_operator_control") is not True:
                        raise ValueError("Selected gateway does not support laptop Stop")
                    challenge = f'{state["instance_id"]} {state["epoch"]} {state["sequence"]}'
                    if challenge != seen:
                        seen = challenge
                        offers[challenge] = now
                        while len(offers) > 24:
                            expired, _ = offers.popitem(last=False)
                            echoed.discard(expired)
                        emit("operator_challenge", challenge=challenge)
                elif now - (gateway.observed_at or started) > 4:
                    raise ValueError("Pi Stop gateway unavailable")
                ready = bool(fresh and state.get("fc_operator_owner") == owner and state["latched"]
                             and not any(state[key] for key in BLOCKED) and start.service_is_ready()
                             and inventory.service_is_ready() and heart.get_subscription_count()
                             and gateway.stop_publisher.get_subscription_count() and not touched)
                if ready != previous_ready:
                    previous_ready = ready
                    emit("operator_status", ready=ready, standing=standing)
                if standing and fresh and (state["latched"] or state.get("fc_operator_owner") != owner
                                           or not state.get("fc_operator_armed")):
                    stop("Pi revoked laptop control")
                    return
                if (pending is not None or inventory_request is not None) and now >= ack_deadline:
                    raise ValueError("Standing acknowledgement unavailable")
                if pending is not None and pending.done():
                    response = pending.result()
                    pending = None
                    if not response.success:
                        raise ValueError("Standing refused: " + response.message)
                    inventory_request = inventory.call_async(ListControllers.Request())
                if inventory_request is not None and inventory_request.done() and fresh and state.get("fc_operator_armed"):
                    rows = {c.name: c.state for c in inventory_request.result().controller}
                    if rows.get("gemini_walk") != "active" or rows.get("trick_leg_kick") != "inactive" or state.get("fc_operator_owner") != owner:
                        raise ValueError("Standing controller or laptop ownership unconfirmed")
                    inventory_request = None
                    standing = True
                    ticks = Path("/proc/self/stat").read_text().rsplit(")", 1)[1].split()[19]
                    write_new(receipt, dict(pid=os.getpid(), start_ticks=ticks, owner=owner, session_id=session_id))
                    (run / "awaiting-standing.json").unlink(missing_ok=True)
                    emit("operator_standing")
                if not select.select([sys.stdin], [], [], 0)[0]:
                    continue
                chunk = os.read(sys.stdin.fileno(), 4096)
                if not chunk:
                    if touched:
                        stop("Laptop operator disconnected")
                    return
                buffer.extend(chunk)
                if len(buffer) > 8192:
                    raise ValueError("Invalid operator message size")
                lines = buffer.split(b"\n")
                buffer = lines.pop()
                packets = [json.loads(line) for line in lines]
                if {"action":"stop"} in packets:
                    stop("Operator pressed Stop")
                    return
                for packet in packets:
                    action = packet.get("action")
                    challenge = packet.get("challenge")
                    is_stand = action == "stand" and packet == {"action":"stand", "challenge":challenge, "supported":True} and packet["supported"] is True
                    if not is_stand and packet != {"action":"heartbeat", "challenge":challenge}:
                        raise ValueError("Unknown or unsupported operator input")
                    issued = offers.get(challenge)
                    if issued is None or time.monotonic()-issued >= .30:
                        if is_stand:
                            raise ValueError("Standing input expired")
                        continue
                    if is_stand:
                        if not ready or touched:
                            raise ValueError("Standing requires current laptop Stop authority")
                        instance, epoch, sequence = challenge.split()
                        request = FcStartMotion.Request(instance_id=instance, epoch=int(epoch), sequence=int(sequence),
                                                        selected_controller="gemini_walk", owner=owner)
                        offers.clear()
                        echoed.clear()
                        touched = True
                        ack_deadline = time.monotonic()+4
                        pending = start.call_async(request)
                    else:
                        # A Pi timer or a buffered copy must never manufacture a browser heartbeat.
                        if challenge not in echoed:
                            echoed.add(challenge)
                            heart.publish(String(data=challenge + " " + owner))
        except BaseException:
            if touched:
                try:
                    stop("Standing or operator connection interrupted")
                except (OSError, ValueError, RuntimeError):
                    pass
            raise
        finally:
            receipt.unlink(missing_ok=True)
            node.destroy_node()
            rclpy.try_shutdown()
