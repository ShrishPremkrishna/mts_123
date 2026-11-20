"""ROS output for the supervised, already-running private stock walk session."""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

from .limits import COMMAND_PERIOD, FORWARD_SPEED, MAX_YAW_RATE

NODE = "pupper_bci"
INPUT = "/kick_cmd_vel"
OUTPUT = "/cmd_vel"
WALKER = "neural_controller_default"
OBSERVATION = f"/{WALKER}/observation"
JOY = "/joy"
GATEWAY = "/estop_controller/stop_status"
STOP = "/emergency_stop"
TWIST_TYPE = "geometry_msgs/msg/Twist"
RESERVED_BUTTONS = (0, 1, 2, 3, 9, 12)  # policy buttons, Options and R3
JOY_AGE = 0.15
FEEDBACK_AGE = 0.2
GATEWAY_AGE = 0.2
RUNTIME_CHECK_PERIOD = 0.2
QUIET_PERIOD = 0.5  # allow previously selected velocity to clear before admission
ZERO_TOLERANCE = 0.02


def _check_enable_button(button):
    if type(button) is not int or not 0 <= button < 32 or button in RESERVED_BUTTONS:
        raise ValueError("choose a gamepad button index other than a face button, Options or R3")


def _check_config(config):
    """Require the walk profile's cached mux/native timeouts and Stop mapping."""
    def params(name):
        return config[name]["ros__parameters"]

    mux, gateway, stock, joy = map(params, ("cmd_vel_mux", "estop_controller", WALKER, "joy_node"))
    if ("/**" in config or any(p.get("use_sim_time", False) is not False
                               for p in (mux, gateway, stock, joy))):
        raise ValueError("BCI requires the private stock profile with real-time clocks")
    if mux.get("inputs") != [INPUT] or mux.get("timeout_ms") != 150:
        raise ValueError(f"BCI requires private mux input {INPUT} with a 150 ms timeout")
    if (gateway.get("switch_button_indices") != [0] or gateway.get("controller_names") != [WALKER]
            or gateway.get("estop_index") != 12 or gateway.get("estop_release_index") != 9):
        raise ValueError("BCI requires the stock Stop gateway and gamepad mapping")
    if (stock.get("game_buttons") is not False or stock.get("reference_tricks", False) is not False
            or stock.get("cmd_vel_contract", "legacy_twist") != "legacy_twist"
            or stock.get("cmd_vel_timeout_s") != 0.25 or stock.get("speed_cap") != 0.49):
        raise ValueError("BCI requires the legacy stock walk profile and 250 ms native expiry")
    rate = joy.get("autorepeat_rate")
    if type(rate) not in (float, int) or not 20 <= rate <= 100:
        raise ValueError("BCI requires gamepad autorepeat between 20 and 100 Hz")


def _check_selected_process_args(proc, identity, config_sha, sha):
    args = [os.fsdecode(item) for item in (proc / str(identity["pid"]) / "cmdline").read_bytes()
            .split(b"\0") if item]
    if (len(args) != 4 or args[0] != identity["path"] or args[1:3] != ["--ros-args", "--params-file"]
            or sha(args[3]) != config_sha):
        raise ValueError("mux and Stop gateway must use the selected profile with no runtime overrides")


def _check_runtime(receipt_path: Path, enable_button: int):
    """Return the maintained runtime continuity check, after read-only admission."""
    from foundation.hardware import runtime_identity as runtime
    from final_project.behaviors.run_through_kick_node import STOCK_POLICY_SHA256
    import yaml

    _check_enable_button(enable_button)
    try:
        receipt = runtime.read_json(receipt_path)
        files = receipt["files"]
        # Historical stock preflights predate this derived metadata. Recover it
        # from the hashed config; verify_loaded still checks the full receipt.
        if "command_contract" not in files["config"]:
            files["config"]["command_contract"] = runtime.command_contract(files)
        if (runtime.command_contract(files) != "legacy_twist" or "guard_script" in files
                or runtime.read_json(receipt["manifest_path"]).get("trial_profile") != "walk"):
            raise ValueError("BCI requires a plain legacy stock walk release")
        evidence = runtime.verify_loaded(receipt)
        selected = evidence["files"]
        if (evidence["command_topology"] != "stock" or selected["policy"]["sha256"] != STOCK_POLICY_SHA256
                or Path(selected["launch"]["path"]).name != "stock_session.launch.py"):
            raise ValueError("BCI requires the selected stock launcher and stock actor")
        config_sha = selected["config"]["sha256"]
        config_bytes = Path(selected["config"]["path"]).read_bytes()
        if hashlib.sha256(config_bytes).hexdigest() != config_sha:
            raise ValueError("stock profile changed during admission")
        _check_config(yaml.safe_load(config_bytes))
        for role in ("mux", "gateway"):
            _check_selected_process_args(Path("/proc"), evidence["executables"][role], config_sha, runtime.sha)
    except (OSError, KeyError, ValueError, TypeError, yaml.YAMLError) as exc:
        raise RuntimeError(f"stock runtime refused: {exc}") from exc
    return lambda: runtime.same_manager(evidence)


def _stock_evidence(node, clock):
    from final_project.behaviors.run_through_kick_node import StockWalkerEvidence
    from rcl_interfaces.msg import ParameterType

    class LegacyStockEvidence(StockWalkerEvidence):
        def _decode(self, inventory, parameters):
            bounds = super()._decode(inventory, parameters)
            enabled, timeout = parameters.values[-2:]
            if (enabled.type != ParameterType.PARAMETER_BOOL or not enabled.bool_value
                    or timeout.type != ParameterType.PARAMETER_DOUBLE or timeout.double_value != 0.25):
                raise ValueError("native controller has no verified 250 ms velocity expiry")
            return bounds

    evidence = LegacyStockEvidence(node, forward_ceiling=FORWARD_SPEED, lateral_ceiling=0.3)
    evidence.param_names += ("cmd_vel_expiry_supported", "cmd_vel_expiry_timeout_s")
    evidence.clock = clock
    return evidence


def _fresh(now, received, age):
    return received is not None and 0 <= now - received <= age


def _endpoint(rows, name, message_type):
    if len(rows) > 1:
        raise RuntimeError(f"competing ROS endpoints, expected only /{name}")
    if not rows:
        return None
    row = rows[0]
    if row.node_name == "_NODE_NAME_UNKNOWN_":
        return None  # wait for discovery to resolve a newly found endpoint
    if (row.node_namespace, row.node_name, row.topic_type) != ("/", name, message_type):
        raise RuntimeError(f"unexpected ROS endpoint /{row.node_name}, expected /{name}")
    gid = bytes(row.endpoint_gid)
    return gid if any(gid) else None


class RobotSink:
    def __init__(self, node, executor, publisher, twist, stock, stop_status, decode_observation,
                 same_runtime, enable_button, *, clock=time.monotonic):
        self.node, self.executor, self.publisher, self.twist = node, executor, publisher, twist
        self.stock, self.stop = stock, stop_status
        self.decode_observation, self.same_runtime = decode_observation, same_runtime
        self.enable_button, self.clock = enable_button, clock
        self.graph = None
        self.graph_since = None
        self.episode = None
        self.next_runtime_check = -math.inf
        self.joy_at = self.joy_stamp = self.observation_at = self.authorized_at = None
        self.held = self.released = self.moving = self.sent_motion = self.closed = False
        self.observation = None
        self.fault = None
        self.waiting_reason = "discovering the private stock command path"

    def on_joy(self, message):
        buttons, stamp = message.buttons, message.header.stamp
        now = self.clock()
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age_ns = self.node.get_clock().now().nanoseconds - stamp_ns
        if (len(buttons) <= max(self.enable_button, *RESERVED_BUTTONS)
                or any(button not in (0, 1) for button in buttons)
                or stamp_ns <= 0 or not 0 <= stamp.nanosec < 1_000_000_000
                or not 0 <= age_ns <= round(JOY_AGE * 1_000_000_000)):
            self.joy_at, self.released, self.held = None, False, False
            if self.moving:
                self._zero_if_owned()
            return
        if self.joy_stamp is not None and stamp_ns <= self.joy_stamp:
            return  # duplicate or old samples never renew a held enable
        if not _fresh(now, self.joy_at, JOY_AGE):
            self.released = False
        self.joy_at, self.joy_stamp = now - age_ns / 1_000_000_000, stamp_ns
        if any(buttons[index] for index in RESERVED_BUTTONS):
            self.held, self.released = False, False
            if self.episode is not None or buttons[12]:
                self.fault = "operator Stop or controller switch received"
            return
        self.held = bool(buttons[self.enable_button])
        if self.episode is not None and not self.held:
            self.released = True
            self.authorized_at = None
            if self.moving:
                self._zero_if_owned()

    def on_gateway(self, message):
        if len(message.data) > 2048:
            self.fault = "oversized Stop gateway status"
            return
        self.stop.accept(message.data, self.clock())
        if self.stop.permanent_error:
            self.fault = self.stop.permanent_error
        elif self.episode is not None:
            reason = self._gateway_problem(self.clock())
            if reason:
                self.fault = reason

    def on_emergency(self, _message):
        self.fault = "emergency Stop received"

    def on_observation(self, message):
        try:
            self.observation = self.decode_observation(message.data)
            self.observation_at = self.clock()
        except ValueError:
            self.observation_at = None

    def _gateway_problem(self, now):
        if self.episode is not None and self.episode != self.stop.episode():
            return "Stop gateway instance or epoch changed"
        return (self.stop.reason(now)
                or ("Stop gateway status is stale" if not _fresh(now, self.stop.received, GATEWAY_AGE) else None)
                or ("Stop gateway has a controller operation pending" if self.stop.state["in_flight"] else None))

    def _command_endpoints(self):
        return (_endpoint(self.node.get_publishers_info_by_topic(INPUT), NODE, TWIST_TYPE),
                _endpoint(self.node.get_subscriptions_info_by_topic(INPUT), "cmd_vel_mux", TWIST_TYPE))

    def _graph_endpoints(self):
        publishers = self.node.get_publishers_info_by_topic
        controller_subscriptions = [row for row in self.node.get_subscriptions_info_by_topic(OUTPUT)
                                    if row.node_name == WALKER]
        return (*self._command_endpoints(),
                _endpoint(publishers(OUTPUT), "cmd_vel_mux", TWIST_TYPE),
                _endpoint(controller_subscriptions, WALKER, TWIST_TYPE),
                _endpoint(publishers(JOY), "joy_node", "sensor_msgs/msg/Joy"),
                _endpoint(publishers(GATEWAY), "estop_controller", "std_msgs/msg/String"),
                _endpoint(publishers(OBSERVATION), WALKER, "std_msgs/msg/Float32MultiArray"))

    def _authority_problem(self, now):
        return (self._gateway_problem(now)
                or self.stock.problem()
                or ("stock policy observation is missing or stale"
                    if not _fresh(now, self.observation_at, FEEDBACK_AGE) else None))

    def _zero_if_owned(self):
        if not self.sent_motion or self.graph is None:
            return False
        try:
            if self._command_endpoints() == self.graph[:2]:
                self.publisher.publish(self.twist())
                self.moving = False
                return True
        except RuntimeError:
            pass  # the mux/native expiry still applies if we lost the path
        return False

    def _fail(self, reason):
        self.fault = reason
        self.authorized_at = None
        self._zero_if_owned()
        raise RuntimeError(reason)

    def poll(self) -> bool:
        """Process bounded ROS work and return whether a held-enable tick is admissible."""
        self.authorized_at = None
        if self.closed or self.fault:
            self._fail(self.fault or "BCI output is closed")
        try:
            # Latest-only subscriptions bound backlog. A finite budget also keeps
            # authority checks responsive under unrelated traffic.
            until = self.clock() + 0.012
            for _ in range(12):
                self.executor.spin_once(timeout_sec=0)
                if self.clock() >= until:
                    break
            self.stock.poll()
            if self.clock() >= self.next_runtime_check:
                if not self.same_runtime():
                    raise RuntimeError("selected stock runtime changed or stopped")
                self.next_runtime_check = self.clock() + RUNTIME_CHECK_PERIOD
            graph = self._graph_endpoints()
        except RuntimeError as exc:
            self._fail(str(exc))
        if self.fault:
            self._fail(self.fault)

        now = self.clock()
        if not all(graph) or (self.graph and graph != self.graph):
            if self.episode is not None:
                self._fail("private stock command graph changed")
            self.graph, self.graph_since = None, None
        if all(graph) and self.graph is None:
            self.graph, self.graph_since = graph, now
        reason = self._authority_problem(now)
        if self.episode is None:
            if self.graph_since is None or now - self.graph_since < QUIET_PERIOD:
                self.waiting_reason = "discovering the private stock command path"
                return False
            if reason or any(abs(value) > ZERO_TOLERANCE for value in self.observation):
                self.waiting_reason = reason or "waiting for zero stock policy command feedback"
                return False
            self.episode = self.stop.episode()
            self.released = _fresh(now, self.joy_at, JOY_AGE) and not self.held
        elif reason:
            self._fail(reason)

        if not _fresh(now, self.joy_at, JOY_AGE):
            self.released = False
        if self.released and self.held:
            if self.moving or all(abs(value) <= ZERO_TOLERANCE for value in self.observation):
                self.authorized_at = now
                self.waiting_reason = None
                return True
            self.waiting_reason = "waiting for zero stock policy command feedback"
        else:
            self.waiting_reason = "release then hold the enable button" if not self.released else "hold the enable button"
        if self.moving:
            self._zero_if_owned()
        return False

    def publish(self, forward: float, yaw: float):
        """Publish one bounded command through the mux, with a recent poll ticket."""
        if (type(forward) not in (int, float) or type(yaw) not in (int, float)
                or not 0 <= forward <= FORWARD_SPEED or not -MAX_YAW_RATE <= yaw <= MAX_YAW_RATE):
            self._fail("velocity request exceeds the BCI walk limits")
        if forward == yaw == 0:
            if self.moving:
                self._zero_if_owned()
            self.authorized_at = None
            return
        now = self.clock()
        if (self.closed or self.fault or not _fresh(now, self.authorized_at, COMMAND_PERIOD)
                or not _fresh(now, self.joy_at, JOY_AGE) or not self.held):
            self._fail(self.fault or "motion request lacks a recent held-enable poll")
        reason = self._authority_problem(now) or self.stock.problem(command=SimpleNamespace(vx=forward, vy=0, wz=yaw))
        if reason:
            self._fail(reason)
        try:
            endpoints = self._command_endpoints()
        except RuntimeError as exc:
            self._fail(str(exc))
        if endpoints != self.graph[:2]:
            self._fail("private mux input ownership changed")
        self.authorized_at = None
        command = self.twist()
        command.linear.x, command.angular.z = float(forward), float(yaw)
        self.moving = self.sent_motion = True
        self.publisher.publish(command)

    def close(self):
        if self.closed:
            return False
        self.closed = True
        self.authorized_at = None
        return self._zero_if_owned()


@contextmanager
def open_robot(receipt_path: Path, enable_button: int):
    """Bind to the selected Pi runtime without starting or switching any controller."""
    _check_enable_button(enable_button)
    root = str(Path(__file__).resolve().parents[2])
    if root not in sys.path:
        sys.path.insert(0, root)

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Empty, Float32MultiArray, String

    from final_project.behaviors.tools.movement_trial import applied_command
    from gemini_pupper.robot.stop_status import StopStatus

    same_runtime = _check_runtime(receipt_path, enable_button)
    context = Context()
    rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = executor = sink = None
    try:
        node = rclpy.create_node(NODE, namespace="/", context=context, use_global_arguments=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        publisher = node.create_publisher(Twist, INPUT, 1)
        clock = time.monotonic
        sink = RobotSink(node, executor, publisher, Twist, _stock_evidence(node, clock),
                         StopStatus(), applied_command, same_runtime, enable_button, clock=clock)
        node.create_subscription(Joy, JOY, sink.on_joy, 1)
        node.create_subscription(String, GATEWAY, sink.on_gateway, 1)
        node.create_subscription(Empty, STOP, sink.on_emergency, 1)
        node.create_subscription(Float32MultiArray, OBSERVATION, sink.on_observation, 1)
        yield sink
    finally:
        try:
            if sink and sink.close():
                publisher.wait_for_all_acked(Duration(seconds=COMMAND_PERIOD))
        finally:
            if executor:
                executor.shutdown(timeout_sec=COMMAND_PERIOD)
            if node:
                node.destroy_node()
            rclpy.try_shutdown(context=context)
