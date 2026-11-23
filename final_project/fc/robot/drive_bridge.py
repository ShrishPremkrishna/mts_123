#!/usr/bin/env python3
"""Pupper FC drive bridge: laptop UDP velocity packets -> geometry_msgs/Twist.

Wire format (one JSON object per datagram, SPEC.md):
    {"v":2,"robot":id,"sid":hex,"seq":int,"t":float,"vx":m/s,"vy":m/s,"wz":rad/s,"ttl":s,"mac":hex}
    mac = HMAC-SHA256(shared key, canonical JSON of every other field)

Behaviour:
- Admits a packet only if its MAC is valid, `robot` is this bridge's --robot-id, the
  sender address is allowed (--allow-from), and it is not a replay: within a sender
  session (`sid`) seq must increase; `t` (laptop clock) may not fall more than
  REPLAY_WINDOW_S behind the newest accepted `t`; a retired sid is never accepted again.
  A new sid with a valid MAC starts a new session (laptop restart).
- Rejected packets are counted and never acked. Accepted packets get a signed ack
  {"v":2,"robot":..,"sid":..,"ack":seq,"ok":true,"imu":..,"profile":..,"bridge":{stats},"mac":..}.
- Publishes the latest accepted command at --rate Hz (default 50) while it is
  younger than its ttl (default 0.25 s, measured on the Pi's monotonic clock).
- When the command expires it publishes zero ONCE and goes silent, so the
  cmd_vel_mux timeout and the gamepad keep authority. Clamps to hard ceilings.
- Replay state (newest accepted `t`, current sid + seq, retired sids) is saved to a
  0600 --state-file at most once a second and on exit, and reloaded on start, so a
  bridge restart does not reopen old sessions. Only laptop `t` values are compared.
- Holds (publishes zero instead of the command, ack `hold`) while:
  * --imu-stop (default on): ImuHealth is "bad" with bad_cause stale/tilt, until it has
    been ok/warn for IMU_CLEAR_S (ack `imu_hold: true`);
  * --mux-guard (default on): the graph does not show exactly one /cmd_vel publisher and
    the mux on our topic, or /cmd_vel went quiet while we publish (ack `mux_ok: false`).
    If the mux disappears after it was seen, the bridge also publishes zero straight to
    /cmd_vel for EMERGENCY_S (a dead mux otherwise leaves the walker on its last command).

Everything except `main()` is ROS-free so it can be unit tested on the laptop
(tests/test_drive_bridge.py). fc/link.py imports the signing helpers from here.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
import re
import socket
import stat
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
VERSION = 2
REPLAY_WINDOW_S = 2.0  # packet t may trail the newest accepted t by at most this (laptop clock)
MAX_RETIRED_SIDS = 64
STATE_SAVE_PERIOD_S = 1.0
IMU_CLEAR_S = 1.0      # ImuHealth must read ok/warn this long before an IMU hold releases
MUX_STALL_S = 0.5      # publishing this long with no /cmd_vel traffic => mux stalled (it republishes at 20 Hz)
EMERGENCY_S = 2.0      # direct zero on /cmd_vel for this long after the mux is lost
DEFAULT_KEY_FILE = "~/fc-v1/robot/link.key"
DEFAULT_STATE_FILE = "~/fc-v1/run/bridge_state.json"
_ID_RE = re.compile(r"^[A-Za-z0-9_.-]{1,32}$")
_MAC_RE = re.compile(r"^[0-9a-f]{64}$")


# --------------------------------------------------------------------------- auth

class KeyFileError(RuntimeError):
    pass


def load_key(path: str) -> bytes:
    """Read a hex key file. Refuses missing, short, or group/other-accessible files."""
    path = os.path.expanduser(path)
    try:
        st = os.stat(path)
    except OSError:
        raise KeyFileError(f"key file {path} missing (python -m fc.robot.keygen on the laptop, then stage.sh)") from None
    if stat.S_IMODE(st.st_mode) & 0o077:
        raise KeyFileError(f"key file {path} has mode {stat.S_IMODE(st.st_mode):o}; chmod 600 it")
    try:
        key = bytes.fromhex(open(path).read().strip())
    except ValueError:
        raise KeyFileError(f"key file {path} is not hex") from None
    if len(key) < 16:
        raise KeyFileError(f"key file {path} is shorter than 16 bytes")
    return key


def canonical(msg: dict) -> bytes:
    """Bytes that the MAC covers: every field except "mac", sorted keys, compact."""
    body = {k: v for k, v in msg.items() if k != "mac"}
    return json.dumps(body, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def mac_of(key: bytes, msg: dict) -> str:
    return hmac.new(key, canonical(msg), hashlib.sha256).hexdigest()


def sign(key: bytes, msg: dict) -> bytes:
    """Serialize msg with its "mac" field added."""
    return json.dumps({**msg, "mac": mac_of(key, msg)}, separators=(",", ":"), allow_nan=False).encode()


def verify(key: bytes, msg: dict) -> bool:
    mac = msg.get("mac")
    if not isinstance(mac, str) or not _MAC_RE.match(mac):
        return False
    try:
        return hmac.compare_digest(mac, mac_of(key, msg))
    except ValueError:  # NaN/inf cannot be canonicalized
        return False


@dataclass(frozen=True)
class Cmd:
    robot: str
    sid: str
    seq: int
    vx: float
    vy: float
    wz: float
    ttl: float
    t: float
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


def parse_packet(data: bytes, key: bytes, default_ttl: float = DEFAULT_TTL, max_ttl: float = MAX_TTL) -> Cmd:
    """Authenticate and validate one datagram; return a clamped Cmd or raise PacketError.

    The MAC is checked before any field is trusted. Robot id and replay checks are
    BridgeCore's job (they need session state)."""
    if len(data) > MAX_DATAGRAM:
        raise PacketError("too_large")
    try:
        msg = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        raise PacketError("not_json") from None
    if not isinstance(msg, dict):
        raise PacketError("not_object")
    if not verify(key, msg):
        raise PacketError("bad_mac")
    seq = msg.get("seq")
    seq = seq if isinstance(seq, int) and not isinstance(seq, bool) and seq >= 0 else None
    if msg.get("v") != VERSION:
        raise PacketError("bad_version", seq)
    if seq is None:
        raise PacketError("bad_seq")
    for key_ in ("robot", "sid"):
        if not isinstance(msg.get(key_), str) or not _ID_RE.match(msg[key_]):
            raise PacketError("bad_" + key_, seq)
    for key_ in ("vx", "vy", "wz", "t"):
        if not _number(msg.get(key_)):
            raise PacketError("bad_" + key_, seq)
    ttl = msg.get("ttl", default_ttl)
    if not _number(ttl) or ttl <= 0:
        raise PacketError("bad_ttl", seq)
    vx, vy, wz = (_clamp(msg["vx"], MAX_VX), _clamp(msg["vy"], MAX_VY), _clamp(msg["wz"], MAX_WZ))
    clamped = (vx, vy, wz) != (float(msg["vx"]), float(msg["vy"]), float(msg["wz"]))
    return Cmd(msg["robot"], msg["sid"], seq, vx, vy, wz, min(float(ttl), max_ttl), float(msg["t"]), clamped)


class BridgeCore:
    """Packet admission (auth, robot id, replay) + ttl dead-man. Pure logic; the caller supplies `now`."""

    def __init__(self, key: bytes, robot_id: str, default_ttl: float = DEFAULT_TTL, max_ttl: float = MAX_TTL,
                 allow_from: frozenset[str] | None = None, replay_window_s: float = REPLAY_WINDOW_S):
        self.key, self.robot_id = key, robot_id
        self.default_ttl = default_ttl
        self.max_ttl = max_ttl
        self.allow_from = frozenset(allow_from) if allow_from else None
        self.replay_window_s = replay_window_s
        self.cmd: Cmd | None = None
        self.cmd_time = -math.inf
        self.publishing = False
        self.sid: str | None = None          # current sender session
        self.last_seq = -1                   # newest accepted seq in that session
        self.newest_t = -math.inf            # newest accepted packet t (sender clock)
        self.retired: list[str] = []         # earlier sids, never accepted again
        self.stats = {"rx": 0, "ok": 0, "bad": 0, "auth": 0, "robot": 0, "replay": 0, "denied": 0,
                      "sessions": 0, "clamped": 0, "pub": 0, "zero": 0, "held": 0}
        self.last_error: str | None = None

    def _reject(self, stat_key: str, reason: str, seq: int | None):
        self.stats[stat_key] += 1
        self.last_error = reason
        return False, seq, reason

    def on_packet(self, data: bytes, now: float, src_ip: str | None = None) -> tuple[bool, int | None, str | None]:
        """Admit one datagram. Returns (accepted, seq, error_reason). Ack only when accepted."""
        self.stats["rx"] += 1
        if self.allow_from is not None and src_ip not in self.allow_from:
            return self._reject("denied", "denied_source", None)
        try:
            cmd = parse_packet(data, self.key, self.default_ttl, self.max_ttl)
        except PacketError as error:
            return self._reject("auth" if error.reason == "bad_mac" else "bad", error.reason,
                                None if error.reason == "bad_mac" else error.seq)
        if cmd.robot != self.robot_id:
            return self._reject("robot", "wrong_robot", cmd.seq)
        if cmd.t < self.newest_t - self.replay_window_s:
            return self._reject("replay", "stale_t", cmd.seq)
        if cmd.sid == self.sid:
            if cmd.seq <= self.last_seq:
                return self._reject("replay", "old_seq", cmd.seq)
        elif cmd.sid in self.retired:
            return self._reject("replay", "old_sid", cmd.seq)
        else:  # authenticated new sender session (laptop restart)
            if self.sid is not None:
                self.retired = (self.retired + [self.sid])[-MAX_RETIRED_SIDS:]
            self.sid = cmd.sid
            self.stats["sessions"] += 1
        self.cmd, self.cmd_time, self.last_seq = cmd, now, cmd.seq
        self.newest_t = max(self.newest_t, cmd.t)
        self.stats["ok"] += 1
        self.stats["clamped"] += int(cmd.clamped)
        return True, cmd.seq, None

    # -- replay state persistence (bridge restart must not reopen old sessions)
    def replay_state(self) -> dict:
        return {"newest_t": self.newest_t if math.isfinite(self.newest_t) else None,
                "sid": self.sid, "last_seq": self.last_seq, "retired": list(self.retired)}

    def restore_replay(self, state: dict) -> None:
        """Resume from a saved replay_state(). The saved sid stays current (a laptop that kept
        running continues with higher seqs); its seq <= last_seq and every retired sid stay dead."""
        t = state.get("newest_t")
        if t is not None:
            self.newest_t = float(t)
        self.sid = state.get("sid")
        self.last_seq = int(state.get("last_seq", -1))
        self.retired = list(state.get("retired") or [])[-MAX_RETIRED_SIDS:]

    def make_ack(self, seq: int, profile: str, imu: dict | None, bridge: dict,
                 hold: str | None = None, imu_hold: bool = False, mux_ok: bool | None = None) -> bytes:
        """Signed reply for an accepted packet. hold: why commands are being replaced by zero."""
        return sign(self.key, {"v": VERSION, "robot": self.robot_id, "sid": self.sid, "ack": seq, "ok": True,
                               "profile": profile, "imu": imu, "bridge": bridge,
                               "hold": hold, "imu_hold": imu_hold, "mux_ok": mux_ok})

    def tick(self, now: float, hold: bool = False) -> tuple[float, float, float] | None:
        """Called at the publish rate. Returns the Twist to publish, or None for silence.
        hold: publish zero instead of the fresh command (keeps the mux input alive, refuses motion)."""
        if self.cmd is not None and now - self.cmd_time < self.cmd.ttl:
            self.publishing = True
            if hold:
                self.stats["held"] += 1
                return (0.0, 0.0, 0.0)
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

    def commanding_zero(self, now: float, hold: bool = False) -> bool:
        """True when the robot is not being asked to move by this bridge (a hold publishes zero)."""
        if hold or self.cmd is None or now - self.cmd_time >= self.cmd.ttl:
            return True
        return self.cmd.vx == 0.0 and self.cmd.vy == 0.0 and self.cmd.wz == 0.0

    def snapshot(self, now: float) -> dict:
        age = None if self.cmd is None else round((now - self.cmd_time) * 1000.0, 1)
        return {**self.stats, "active": self.publishing, "last_seq": None if self.cmd is None else self.cmd.seq,
                "age_ms": age, "err": self.last_error, "robot_id": self.robot_id}


def load_replay_state(path: str) -> dict | None:
    """Saved BridgeCore.replay_state(), or None if there is no file. Raises ValueError if unusable
    (fail closed: the operator decides with --reset-replay)."""
    path = os.path.expanduser(path)
    try:
        with open(path) as f:
            state = json.load(f)
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as error:
        raise ValueError(f"replay state {path} unreadable ({error}); restart with --reset-replay") from None
    ok = (isinstance(state, dict)
          and (state.get("newest_t") is None or _number(state.get("newest_t")))
          and (state.get("sid") is None or (isinstance(state.get("sid"), str) and _ID_RE.match(state["sid"])))
          and isinstance(state.get("last_seq"), int) and not isinstance(state.get("last_seq"), bool)
          and isinstance(state.get("retired"), list)
          and all(isinstance(x, str) and _ID_RE.match(x) for x in state["retired"]))
    if not ok:
        raise ValueError(f"replay state {path} malformed; restart with --reset-replay")
    return state


def save_replay_state(path: str, state: dict) -> None:
    """Atomic 0600 write (tmp + rename)."""
    path = os.path.expanduser(path)
    tmp = f"{path}.tmp{os.getpid()}"
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        os.fchmod(fd, 0o600)
        os.write(fd, json.dumps(state, separators=(",", ":"), allow_nan=False).encode())
    finally:
        os.close(fd)
    os.replace(tmp, path)


class ImuGuard:
    """IMU hold: set when ImuHealth is bad for a stale stream or a held tilt; released only
    after clear_s of continuous ok/warn. Anything else ("unknown", None) keeps the current state."""

    def __init__(self, clear_s: float = IMU_CLEAR_S):
        self.clear_s = clear_s
        self.hold = False
        self.cause: str | None = None
        self._good_since: float | None = None

    def update(self, now: float, summary: dict | None) -> bool:
        level = summary.get("level") if isinstance(summary, dict) else None
        if level == "bad" and summary.get("bad_cause") in ("stale", "tilt"):
            self.hold, self.cause, self._good_since = True, summary["bad_cause"], None
        elif level in ("ok", "warn"):
            if self._good_since is None:
                self._good_since = now
            if self.hold and now - self._good_since >= self.clear_s:
                self.hold, self.cause = False, None
        else:
            self._good_since = None
        return self.hold


class MuxGuard:
    """Is the one cmd_vel_mux there to forward our topic? Inputs: graph counts (polled), /cmd_vel
    receipts, and how long the bridge has been publishing. Pure; the caller supplies `now`."""

    EMERGENCY = ("no_cmd_vel_publisher", "mux_stalled")   # mux lost: walker would hold its last command

    def __init__(self, stall_s: float = MUX_STALL_S):
        self.stall_s = stall_s
        self.pubs = 0                 # /cmd_vel publishers other than ourselves
        self.subs = 0                 # subscribers on our topic
        self.seen = False             # mux observed at least once
        self.last_cmd_vel = -math.inf
        self.why: str | None = "mux_unseen"
        self._emergency_until = -math.inf

    def on_graph(self, cmd_vel_pubs: int, topic_subs: int) -> None:
        self.pubs, self.subs = cmd_vel_pubs, topic_subs
        if cmd_vel_pubs == 1 and topic_subs >= 1:
            self.seen = True

    def on_cmd_vel(self, now: float) -> None:
        self.last_cmd_vel = now

    def check(self, now: float, publishing_since: float | None) -> str | None:
        """Returns None when ok, else the reason. Starts an emergency window on entering a mux-lost state."""
        if not self.seen:
            why = "mux_unseen"
        elif self.pubs == 0:
            why = "no_cmd_vel_publisher"
        elif self.pubs > 1:
            why = "cmd_vel_publishers=%d" % self.pubs
        elif self.subs == 0:
            why = "mux_not_subscribed"
        elif (publishing_since is not None and now - publishing_since >= self.stall_s
              and now - self.last_cmd_vel >= self.stall_s):
            why = "mux_stalled"
        else:
            why = None
        if why in self.EMERGENCY and self.why not in self.EMERGENCY:
            self._emergency_until = now + EMERGENCY_S
        self.why = why
        return why

    def emergency(self, now: float) -> bool:
        """True while the bridge should publish zero directly on /cmd_vel."""
        return now < self._emergency_until


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
    parser.add_argument("--robot-id", required=True, help="only packets addressed to this id are accepted")
    parser.add_argument("--key-file", default=DEFAULT_KEY_FILE, help="hex HMAC key, mode 0600")
    parser.add_argument("--allow-from", default="", help="comma-separated sender IPs (default: any)")
    parser.add_argument("--state-file", default=DEFAULT_STATE_FILE, help="replay state (0600); '' disables")
    parser.add_argument("--reset-replay", action="store_true", help="ignore (and overwrite) the saved replay state")
    parser.add_argument("--imu-stop", action=argparse.BooleanOptionalAction, default=True,
                        help="hold zero while ImuHealth is bad (stale/tilt) (default on)")
    parser.add_argument("--mux-guard", action=argparse.BooleanOptionalAction, default=True,
                        help="hold zero unless exactly one mux forwards our topic to /cmd_vel (default on)")
    args = parser.parse_args(argv)
    if not (1.0 <= args.rate <= 200.0) or not (0.0 < args.default_ttl <= MAX_TTL):
        parser.error("rate must be 1..200 Hz and default ttl in (0, %.2f] s" % MAX_TTL)
    if not _ID_RE.match(args.robot_id):
        parser.error("bad --robot-id")
    try:
        key = load_key(args.key_file)
    except KeyFileError as error:
        parser.error(str(error))
    allow = frozenset(ip.strip() for ip in args.allow_from.split(",") if ip.strip()) or None
    core = BridgeCore(key, args.robot_id, default_ttl=args.default_ttl, allow_from=allow)
    if args.state_file and not args.reset_replay:
        try:
            saved = load_replay_state(args.state_file)
        except ValueError as error:
            parser.error(str(error))
        if saved:
            core.restore_replay(saved)
    if args.state_file:
        try:
            save_replay_state(args.state_file, core.replay_state())   # fail now if the path is unwritable
        except OSError as error:
            parser.error(f"cannot write replay state {args.state_file}: {error}")

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
    imu_guard = ImuGuard() if (health is not None and args.imu_stop) else None
    mux = MuxGuard() if args.mux_guard else None
    emergency_pub = None            # /cmd_vel publisher, exists only while the mux is lost
    state = {"imu": safe_summary(health), "hold": None, "since": None}   # guarded by lock

    def hold_reason():
        if imu_guard is not None and imu_guard.hold:
            return "imu_" + (imu_guard.cause or "bad")
        if mux is not None and mux.why is not None:
            return mux.why
        return None

    if health is not None:
        from sensor_msgs.msg import Imu

        def on_imu(msg):
            q, w = msg.orientation, msg.angular_velocity
            now = time.monotonic()
            with lock:
                zero = core.commanding_zero(now, hold=state["hold"] is not None)
                try:
                    health.update(now, (q.x, q.y, q.z, q.w), (w.x, w.y, w.z), zero)
                except Exception:
                    pass

        node.create_subscription(Imu, args.imu_topic, on_imu, qos_profile_sensor_data)

    if mux is not None:
        def on_cmd_vel(_msg):
            if emergency_pub is None:          # ignore our own emergency zeros
                with lock:
                    mux.on_cmd_vel(time.monotonic())

        node.create_subscription(Twist, "/cmd_vel", on_cmd_vel, 10)

    def twist(values):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = values
        return msg

    def bridge_stats(now):
        snap = core.snapshot(now)
        snap.update(topic=args.topic, up_s=round(now - started, 1), pid=os.getpid(), hold=state["hold"],
                    imu_hold=bool(imu_guard and imu_guard.hold),
                    mux=None if mux is None else {"ok": mux.why is None, "why": mux.why,
                                                  "cmd_vel_pubs": mux.pubs, "topic_subs": mux.subs})
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
                was, sid = core.publishing, core.sid
                ok, seq, error = core.on_packet(data, now, addr[0])
                reply = core.make_ack(seq, args.profile, state["imu"], bridge_stats(now), hold=state["hold"],
                                      imu_hold=bool(imu_guard and imu_guard.hold),
                                      mux_ok=None if mux is None else mux.why is None) if ok else None
            if reply is not None:  # never answer unauthenticated or replayed packets
                try:
                    sock.sendto(reply, addr)
                except OSError:
                    pass
            if ok and (not was or core.sid != sid):
                print(f"[bridge] commands from {addr[0]}:{addr[1]} sid={core.sid} seq={seq}", flush=True)
            if now - last_log > 5.0:
                last_log = now
                print("[bridge] " + json.dumps(bridge_stats(now)), flush=True)

    def save_state():
        if args.state_file:
            with lock:
                snapshot = core.replay_state()
            try:
                save_replay_state(args.state_file, snapshot)
            except OSError as error:
                print(f"[bridge] replay state save failed: {error!r}", file=sys.stderr, flush=True)

    thread = threading.Thread(target=udp_loop, daemon=True)
    thread.start()
    print(f"[bridge] udp {args.bind}:{args.port} -> {args.topic} @ {args.rate:g} Hz, ttl {args.default_ttl}s, "
          f"robot {args.robot_id}, allow {','.join(sorted(allow)) if allow else 'any'}, profile {args.profile}, "
          f"imu {'on ' + args.imu_topic if health else 'off'}, imu-stop {'on' if imu_guard else 'off'}, "
          f"mux-guard {'on' if mux else 'off'}, replay state {args.state_file or 'off'}"
          f"{' (reset)' if args.reset_replay else ''} newest_t {core.replay_state()['newest_t']}, "
          f"domain {os.environ.get('ROS_DOMAIN_ID', '0')}", flush=True)
    rate = RateLoop(args.rate, time.monotonic())
    guard_loop = RateLoop(20.0, time.monotonic())
    graph_loop = RateLoop(5.0, time.monotonic())
    save_loop = RateLoop(1.0 / STATE_SAVE_PERIOD_S, time.monotonic())
    saved_state = core.replay_state()
    last_hold = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(0.005, rate.period / 4))
            now = time.monotonic()
            if mux is not None and graph_loop.due(now):
                own = 1 if emergency_pub is not None else 0
                pubs = node.count_publishers("/cmd_vel") - own
                subs = node.count_subscribers(args.topic)
                with lock:
                    mux.on_graph(pubs, subs)
            if guard_loop.due(now):
                imu = safe_summary(health)       # one ImuHealth summary per 50 ms, shared with the acks
                with lock:
                    state["imu"] = imu
                    if imu_guard is not None:
                        imu_guard.update(now, imu)
                    if mux is not None:
                        mux.check(now, state["since"])
                    state["hold"] = hold_reason()
                    hold = state["hold"]
                if hold != last_hold:
                    print(f"[bridge] hold {last_hold} -> {hold}", flush=True)
                    last_hold = hold
                if mux is not None:
                    if mux.emergency(now) and emergency_pub is None:
                        print(f"[bridge] MUX LOST ({mux.why}): publishing zero on /cmd_vel for {EMERGENCY_S}s",
                              flush=True)
                        emergency_pub = node.create_publisher(Twist, "/cmd_vel", 10)
                    if emergency_pub is not None:
                        if mux.emergency(now):
                            emergency_pub.publish(twist((0.0, 0.0, 0.0)))   # 20 Hz while the window lasts
                        else:
                            node.destroy_publisher(emergency_pub)
                            emergency_pub = None
            if rate.due(now):
                with lock:
                    out = core.tick(now, hold=state["hold"] is not None)
                    if core.publishing:
                        state["since"] = state["since"] if state["since"] is not None else now
                    else:
                        state["since"] = None
                if out is not None:
                    pub.publish(twist(out))
            if args.state_file and save_loop.due(now):
                with lock:
                    current = core.replay_state()
                if current != saved_state:
                    save_state()
                    saved_state = current
    except (KeyboardInterrupt, Exception) as error:
        if not isinstance(error, KeyboardInterrupt):
            print(f"[bridge] spin error {error!r}", file=sys.stderr, flush=True)
    finally:
        running.clear()
        with lock:
            out = core.shutdown()
        if out is not None:
            try:
                pub.publish(twist(out))
                time.sleep(0.05)
            except Exception:
                pass
        sock.close()
        save_state()
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
