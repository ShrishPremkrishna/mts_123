#!/usr/bin/env python3
"""Pupper FC drive bridge: laptop UDP velocity packets -> geometry_msgs/Twist.

Wire format (one JSON object per datagram, SPEC.md):
    {"v":1,"seq":int,"t":float,"vx":m/s,"vy":m/s,"wz":rad/s,"ttl":s}

Behaviour:
- Publishes the latest accepted command at --rate Hz (default 50) while it is
  younger than its ttl (default 0.25 s, measured on the Pi's monotonic clock).
- When the command expires it publishes zero ONCE and goes silent, so the
  cmd_vel_mux timeout and the gamepad keep authority.
- Rejects malformed and out-of-order packets; clamps to hard ceilings.
- Replies to each datagram with {"v":1,"ack":seq,"ok":..,"imu":..,"profile":..,"bridge":{stats}}.

Everything except `main()` / `RosBridge` is ROS-free so it can be unit tested
on the laptop (tests/test_drive_bridge.py).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass

# Hard ceilings (never relaxed here; softer caps live in the laptop params).
MAX_VX = 0.5
MAX_VY = 0.3
MAX_WZ = 2.0
DEFAULT_TTL = 0.25
MAX_TTL = 0.5          # a packet can never hold motion longer than the mux timeout
MAX_DATAGRAM = 1024
RESTART_GAP_S = 1.0    # after this much silence a lower seq is a new sender stream


@dataclass(frozen=True)
class Cmd:
    seq: int
    vx: float
    vy: float
    wz: float
    ttl: float
    t: float | None
    clamped: bool


class PacketError(ValueError):
    def __init__(self, reason: str, seq: int | None = None):
        super().__init__(reason)
        self.reason = reason
        self.seq = seq


def _number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


def parse_packet(data: bytes, default_ttl: float = DEFAULT_TTL, max_ttl: float = MAX_TTL) -> Cmd:
    """Validate one datagram and return a clamped Cmd, or raise PacketError."""
    if len(data) > MAX_DATAGRAM:
        raise PacketError("too_large")
    try:
        msg = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        raise PacketError("not_json") from None
    if not isinstance(msg, dict):
        raise PacketError("not_object")
    seq = msg.get("seq")
    seq = seq if isinstance(seq, int) and not isinstance(seq, bool) and seq >= 0 else None
    if msg.get("v") != 1:
        raise PacketError("bad_version", seq)
    if seq is None:
        raise PacketError("bad_seq")
    for key in ("vx", "vy", "wz"):
        if not _number(msg.get(key)):
            raise PacketError("bad_" + key, seq)
    ttl = msg.get("ttl", default_ttl)
    if not _number(ttl) or ttl <= 0:
        raise PacketError("bad_ttl", seq)
    t = msg.get("t")
    if t is not None and not _number(t):
        raise PacketError("bad_t", seq)
    vx, vy, wz = (_clamp(msg["vx"], MAX_VX), _clamp(msg["vy"], MAX_VY), _clamp(msg["wz"], MAX_WZ))
    clamped = (vx, vy, wz) != (float(msg["vx"]), float(msg["vy"]), float(msg["wz"]))
    return Cmd(seq, vx, vy, wz, min(float(ttl), max_ttl), None if t is None else float(t), clamped)


class BridgeCore:
    """Packet admission + ttl dead-man. Pure logic; the caller supplies `now`."""

    def __init__(self, default_ttl: float = DEFAULT_TTL, max_ttl: float = MAX_TTL,
                 restart_gap_s: float = RESTART_GAP_S):
        self.default_ttl = default_ttl
        self.max_ttl = max_ttl
        self.restart_gap_s = restart_gap_s
        self.cmd: Cmd | None = None
        self.cmd_time = -math.inf
        self.publishing = False
        self.stats = {"rx": 0, "ok": 0, "bad": 0, "ooo": 0, "clamped": 0, "pub": 0, "zero": 0}
        self.last_error: str | None = None

    def on_packet(self, data: bytes, now: float) -> tuple[bool, int | None, str | None]:
        """Admit one datagram. Returns (accepted, seq, error_reason)."""
        self.stats["rx"] += 1
        try:
            cmd = parse_packet(data, self.default_ttl, self.max_ttl)
        except PacketError as error:
            self.stats["bad"] += 1
            self.last_error = error.reason
            return False, error.seq, error.reason
        if self.cmd is not None and cmd.seq <= self.cmd.seq and now - self.cmd_time < self.restart_gap_s:
            self.stats["ooo"] += 1
            self.last_error = "out_of_order"
            return False, cmd.seq, "out_of_order"
        self.cmd, self.cmd_time = cmd, now
        self.stats["ok"] += 1
        self.stats["clamped"] += int(cmd.clamped)
        return True, cmd.seq, None

    def tick(self, now: float) -> tuple[float, float, float] | None:
        """Called at the publish rate. Returns the Twist to publish, or None for silence."""
        if self.cmd is not None and now - self.cmd_time < self.cmd.ttl:
            self.publishing = True
            self.stats["pub"] += 1
            return (self.cmd.vx, self.cmd.vy, self.cmd.wz)
        if self.publishing:
            self.publishing = False
            self.stats["zero"] += 1
            return (0.0, 0.0, 0.0)
        return None

    def shutdown(self) -> tuple[float, float, float] | None:
        """Final output on exit: zero once if we were driving, else nothing."""
        if self.publishing:
            self.publishing = False
            self.cmd = None
            self.stats["zero"] += 1
            return (0.0, 0.0, 0.0)
        return None

    def commanding_zero(self, now: float) -> bool:
        """True when the robot is not being asked to move by this bridge."""
        if self.cmd is None or now - self.cmd_time >= self.cmd.ttl:
            return True
        return self.cmd.vx == 0.0 and self.cmd.vy == 0.0 and self.cmd.wz == 0.0

    def snapshot(self, now: float) -> dict:
        age = None if self.cmd is None else round((now - self.cmd_time) * 1000.0, 1)
        return {**self.stats, "active": self.publishing, "last_seq": None if self.cmd is None else self.cmd.seq,
                "age_ms": age, "err": self.last_error}


def make_ack(seq, ok: bool, error: str | None, profile: str, imu: dict | None, bridge: dict) -> bytes:
    reply = {"v": 1, "ack": seq, "ok": ok, "profile": profile, "imu": imu, "bridge": bridge}
    if error:
        reply["error"] = error
    return json.dumps(reply, separators=(",", ":"), allow_nan=False).encode()


class RateLoop:
    """Fixed-rate scheduler without drift: next deadline = previous + period."""

    def __init__(self, hz: float, start: float):
        self.period = 1.0 / hz
        self.next = start

    def due(self, now: float) -> bool:
        if now < self.next:
            return False
        # Skip missed periods instead of bursting to catch up.
        self.next += self.period * max(1, math.floor((now - self.next) / self.period) + 1)
        return True


def load_imu_health():
    """Import the IMU lane's ImuHealth if it exists next to this file or as fc.robot."""
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    for name in ("imu_health", "fc.robot.imu_health"):
        try:
            module = __import__(name, fromlist=["ImuHealth"])
            return module.ImuHealth
        except Exception:  # missing or broken module: run without health
            continue
    return None


def safe_summary(health) -> dict | None:
    if health is None:
        return None
    try:
        summary = health.summary()
        json.dumps(summary, allow_nan=False)
        return summary
    except Exception as error:
        return {"level": "unknown", "error": type(error).__name__}


# --------------------------------------------------------------------------- ROS


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9870)
    parser.add_argument("--topic", default="/llm_cmd_vel", help="Twist output (a cmd_vel_mux input)")
    parser.add_argument("--imu-topic", default="/imu_sensor_broadcaster/imu", help="'' disables IMU health")
    parser.add_argument("--profile", default=os.environ.get("FC_PROFILE", "unknown"))
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--default-ttl", type=float, default=DEFAULT_TTL)
    args = parser.parse_args(argv)
    if not (1.0 <= args.rate <= 200.0) or not (0.0 < args.default_ttl <= MAX_TTL):
        parser.error("rate must be 1..200 Hz and default ttl in (0, %.2f] s" % MAX_TTL)

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))  # EADDRINUSE => another bridge owns the port
    sock.settimeout(0.2)

    # Own SIGINT/SIGTERM ourselves so the context is still alive for the final zero Twist.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("fc_drive_bridge")
    pub = node.create_publisher(Twist, args.topic, 10)
    core = BridgeCore(default_ttl=args.default_ttl)
    lock = threading.Lock()
    started = time.monotonic()
    running = threading.Event()
    running.set()

    health = None
    health_cls = load_imu_health() if args.imu_topic else None
    if health_cls is not None:
        try:
            health = health_cls({})
        except Exception as error:
            node.get_logger().warning(f"ImuHealth init failed: {error!r}; running without IMU health")
    if health is not None:
        from sensor_msgs.msg import Imu

        def on_imu(msg):
            q, w = msg.orientation, msg.angular_velocity
            now = time.monotonic()
            with lock:
                zero = core.commanding_zero(now)
                try:
                    health.update(now, (q.x, q.y, q.z, q.w), (w.x, w.y, w.z), zero)
                except Exception:
                    pass

        node.create_subscription(Imu, args.imu_topic, on_imu, qos_profile_sensor_data)

    def publish(values):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = values
        pub.publish(msg)

    def on_timer():
        with lock:
            out = core.tick(time.monotonic())
        if out is not None:
            publish(out)

    def bridge_stats(now):
        snap = core.snapshot(now)
        snap.update(topic=args.topic, up_s=round(now - started, 1), pid=os.getpid())
        return snap

    def udp_loop():
        last_log = 0.0
        while running.is_set():
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            now = time.monotonic()
            with lock:
                was = core.publishing
                ok, seq, error = core.on_packet(data, now)
                reply = make_ack(seq, ok, error, args.profile, safe_summary(health), bridge_stats(now))
            try:
                sock.sendto(reply, addr)
            except OSError:
                pass
            if ok and not was:
                print(f"[bridge] commands from {addr[0]}:{addr[1]} seq={seq}", flush=True)
            if now - last_log > 5.0:
                last_log = now
                print("[bridge] " + json.dumps(bridge_stats(now)), flush=True)

    thread = threading.Thread(target=udp_loop, daemon=True)
    thread.start()
    print(f"[bridge] udp {args.bind}:{args.port} -> {args.topic} @ {args.rate:g} Hz, ttl {args.default_ttl}s, "
          f"profile {args.profile}, imu {'on ' + args.imu_topic if health else 'off'}, "
          f"domain {os.environ.get('ROS_DOMAIN_ID', '0')}", flush=True)
    rate = RateLoop(args.rate, time.monotonic())
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(0.005, rate.period / 4))
            if rate.due(time.monotonic()):
                on_timer()
    except (KeyboardInterrupt, Exception) as error:
        if not isinstance(error, KeyboardInterrupt):
            print(f"[bridge] spin error {error!r}", file=sys.stderr, flush=True)
    finally:
        running.clear()
        with lock:
            out = core.shutdown()
        if out is not None:
            try:
                publish(out)
                time.sleep(0.05)
            except Exception:
                pass
        sock.close()
        print("[bridge] exit " + json.dumps(bridge_stats(time.monotonic())), flush=True)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass
    return 0


def _sigterm(*_):
    raise KeyboardInterrupt


if __name__ == "__main__":
    import signal

    signal.signal(signal.SIGTERM, _sigterm)
    sys.exit(main())
