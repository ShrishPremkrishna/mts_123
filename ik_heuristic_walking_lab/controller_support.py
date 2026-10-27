"""Sensor validation and position-controller shutdown shared by the Lab 3 nodes.

No ROS imports or activity occur on import. Shutdown deactivates the controller;
publishing zero angles would command a new pose, not turn off torque.
"""
import math

import numpy as np


def joint_sample(msg, names):
    """Return aligned finite positions/velocities, or None for a malformed sample."""
    try:
        if len(msg.name) != len(set(msg.name)):
            return None
        if len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
            return None
        indices = [msg.name.index(name) for name in names]
        positions = np.asarray([msg.position[i] for i in indices], dtype=float)
        velocities = np.asarray([msg.velocity[i] for i in indices], dtype=float)
        if positions.shape != (len(names),) or not np.isfinite([positions, velocities]).all():
            return None
        return positions, velocities
    except (AttributeError, TypeError, ValueError, IndexError, OverflowError):
        return None


def state_fresh(last_state_at, now, timeout=0.25):
    """Use receipt time on a monotonic clock; stale or invalid times are unsafe."""
    return (last_state_at is not None
            and all(math.isfinite(t) for t in (last_state_at, now, timeout))
            and timeout > 0 and 0 <= now - last_state_at <= timeout)


def request_position_stop(node, timers):
    """Cancel command timers and request deactivation without nested spinning.

    Safe to call from a timer callback. The caller latches its fault; repairing
    sensors must not silently resume movement. A missing service is reported.
    """
    for timer in timers:
        if timer is not None:
            timer.cancel()
    try:
        from controller_manager_msgs.srv import SwitchController
        client = node.create_client(SwitchController, '/controller_manager/switch_controller')
        if not client.wait_for_service(timeout_sec=0.0):
            node.get_logger().error('Controller stop unavailable: use the gamepad emergency stop.')
            return None
        request = SwitchController.Request()
        request.deactivate_controllers = ['forward_command_controller']
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.timeout.sec = 2
        return client.call_async(request)
    except Exception as exc:
        node.get_logger().error(f'Could not request controller stop ({type(exc).__name__}); use the gamepad emergency stop.')
        return None


def confirm_position_stop(node, future, timeout=3.0):
    """Wait after the main spin loop exits, and report whether ROS confirmed stop."""
    try:
        import rclpy
        if future is not None and not future.done() and rclpy.ok():
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        ok = (future is not None and future.done() and not future.cancelled()
              and future.exception() is None and bool(future.result().ok))
    except Exception:
        ok = False
    if not ok:
        node.get_logger().error('Controller did not confirm stop: use the gamepad emergency stop.')
    return ok
