"""Cmd -> UDP datagram to each robot's Pi (SPEC wire protocol), plus ack/health status.

Stdlib only. One socket for all robots; a listener thread matches acks to robots by
the sender's address. Hard speed ceilings apply here no matter what params say.
Packets and acks are HMAC-signed with the shared key in config/link.key
(`python -m fc.robot.keygen`); without a key nothing is sent.
"""
from __future__ import annotations

import ipaddress
import json
import math
import secrets
import socket
import subprocess
import threading
import time
from collections import deque

from .core import CONFIG, Cmd, RobotCfg
from .robot.drive_bridge import VERSION, KeyFileError, load_key, sign, verify

PORT = 9870
CEIL = {"vx": 1.5, "vy": 0.3, "wz": 2.0}  # never exceeded, whatever params say (vx 1.5 = top walker speed cap)
TTL_RANGE = (0.05, 0.5)   # drive_bridge MAX_TTL is 0.5 s (mux timeout)
CONNECTED_AGE_S = 1.0                     # ack younger than this counts as connected
KEY_FILE = CONFIG / "link.key"
TAILSCALE_NET = ipaddress.ip_network("100.64.0.0/10")   # CGNAT range Tailscale uses
LOSS_AFTER_S = 0.5                        # a packet not acked this long after sending counts as lost
ACK_CORE = {"v", "robot", "sid", "ack", "mac", "ok", "imu", "profile", "bridge", "gz"}
GYRO_KEEP = 150                           # measured yaw-rate samples kept per robot (~5 s of acks)
ESTOP_BURST = (5, 0.02)                   # SPEC v1.1: e-stop burst of 5 packets, 20 ms apart
RESOLVE_RETRY_S = 5.0                     # unresolved address / failed send: resolve the host again at most this often


def is_tailscale(addr: str | None) -> bool:
    try:
        return addr is not None and ipaddress.ip_address(addr) in TAILSCALE_NET
    except ValueError:
        return False


def resolve_host(host: str) -> str | None:
    """IP for a host or ssh alias. `ssh -G` only evaluates local ssh config (no network)."""
    if not host:
        return None
    name = host
    try:
        out = subprocess.run(["ssh", "-G", host], capture_output=True, text=True, timeout=3).stdout
        for line in out.splitlines():
            if line.startswith("hostname "):
                name = line.split(None, 1)[1].strip()
                break
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        return socket.gethostbyname(name)
    except OSError:
        return None


def _num(v) -> float:
    try:
        v = float(v)
    except (TypeError, ValueError):
        return 0.0
    return v if math.isfinite(v) else 0.0


def clamp_cmd(cmd: Cmd, params: dict) -> tuple[float, float, float]:
    """Clamp to min(params cap, hard ceiling). NaN/inf -> 0."""
    out = []
    for axis, key in (("vx", "max_vx"), ("vy", "max_vy"), ("wz", "max_wz")):
        cap = min(abs(_num(params.get(key, CEIL[axis]))), CEIL[axis])
        out.append(max(-cap, min(cap, _num(getattr(cmd, axis)))))
    return out[0], out[1], out[2]


def packet(key: bytes, robot: str, sid: str, seq: int, cmd: Cmd, params: dict, t: float | None = None,
           estop: bool = False) -> bytes:
    """Signed datagram. `t` is laptop wall-clock time: the Pi only compares it with earlier
    packets' t (replay window), and wall time keeps increasing across laptop restarts.
    `estop` (SPEC v1.1): all motion fields zero plus "estop": true; the bridge does the R3
    action (leg controllers off, damped drop) and latches until `fcctl.sh ready ... --confirm`."""
    vx, vy, wz = (0.0, 0.0, 0.0) if estop else clamp_cmd(cmd, params)
    ttl = max(TTL_RANGE[0], min(TTL_RANGE[1], _num(params.get("ttl_s", 0.25)) or 0.25))
    msg = {"v": VERSION, "robot": robot, "sid": sid, "seq": seq, "t": round(time.time() if t is None else t, 4),
           "vx": round(vx, 4), "vy": round(vy, 4), "wz": round(wz, 4), "ttl": ttl}
    if estop:
        msg["estop"] = True
    return sign(key, msg)


def load_link_key(path=KEY_FILE) -> tuple[bytes | None, str | None]:
    try:
        return load_key(str(path)), None
    except KeyFileError as e:
        return None, str(e)


class Link:
    def __init__(self, robots: list[RobotCfg], params: dict, port: int = PORT, resolve=resolve_host,
                 key_file=KEY_FILE, udp_hosts: dict[str, str] | None = None):
        """`udp_hosts` (robots.json "udp_host"): optional per-robot address for control packets,
        e.g. the Pi's LAN IP, so UDP does not ride Tailscale when the ssh alias resolves there."""
        self.params, self.port = params, port
        self.robots = {r.id: r for r in robots}
        self.udp_hosts = {k: v for k, v in (udp_hosts or {}).items() if v}
        self.key, self.key_error = load_link_key(key_file)
        self.sid = secrets.token_hex(8)   # new sender session per process; the bridge resumes on it
        self._lock = threading.Lock()
        self._st = {r.id: {"host": self.udp_hosts.get(r.id) or r.host, "via": "udp_host" if r.id in self.udp_hosts else "ssh",
                           "addr": None, "error": None if (r.host or r.id in self.udp_hosts) else "no host", "extra": {},
                           "rtts": deque(maxlen=200),
                           "seq": 0, "sent": 0, "acks": 0, "last_ack_t": None, "rtt_ms": None, "last_ack_seq": 0,
                           "bad_acks": 0, "bad_ack_t": None, "estop_sent": 0, "estopped": None,
                           "imu": None, "profile": None, "bridge_ok": None, "bridge": None, "last_cmd": [0.0, 0.0, 0.0],
                           "gz": deque(maxlen=GYRO_KEEP), "gz_n": -1} for r in robots}
        self._sent_t: dict[str, dict[int, list]] = {r.id: {} for r in robots}   # seq -> [t_sent, acked]
        self._by_addr: dict[str, str] = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.2)
        self._stop = False
        self.ack_log: list[dict] = []  # drained by the app into the run log
        self._resolve = resolve
        self._resolving: set[str] = set()       # robots with a resolve in flight
        self._resolve_t: dict[str, float] = {}  # monotonic time of each robot's last resolve attempt
        threading.Thread(target=self._listen, daemon=True, name="link-acks").start()
        threading.Thread(target=self._resolve_all, args=(resolve,), daemon=True, name="link-resolve").start()

    def _resolve_all(self, resolve) -> None:
        for rid in self.robots:
            self._resolve_one(rid, resolve)

    def _resolve_one(self, rid: str, resolve=None) -> None:
        """Resolve one robot's host (blocking: ssh -G + DNS). A new address replaces the old one for acks too."""
        host = self.udp_hosts.get(rid) or self.robots[rid].host
        if not host:
            return
        with self._lock:
            self._resolving.add(rid)
            self._resolve_t[rid] = time.monotonic()
        try:
            ip = (resolve or self._resolve)(host)
        except Exception:  # noqa: BLE001 - a resolver bug must not kill the thread; retried later
            ip = None
        with self._lock:
            self._resolving.discard(rid)
            st = self._st[rid]
            if ip:
                old = st["addr"]
                if old and old != ip and self._by_addr.get(old) == rid:
                    del self._by_addr[old]
                st["addr"], st["error"] = ip, None
                self._by_addr[ip] = rid
            elif not st["addr"]:
                st["error"] = f"cannot resolve {host}"

    def _retry_resolve(self, rid: str, now: float) -> None:
        """Re-resolve in the background (never on the 30 Hz loop) after a failure: an address that did not
        resolve at start (Wi-Fi not up yet) or a send error. Throttled to RESOLVE_RETRY_S per robot."""
        with self._lock:
            if rid in self._resolving or now - self._resolve_t.get(rid, -math.inf) < RESOLVE_RETRY_S:
                return
            self._resolving.add(rid)
            self._resolve_t[rid] = now
        threading.Thread(target=self._resolve_one, args=(rid,), daemon=True, name=f"link-resolve-{rid}").start()

    # ---- sending -----------------------------------------------------------
    def send(self, cmds: dict[str, Cmd], estop: set[str] | frozenset = frozenset()) -> None:
        """One packet per robot in `cmds`; robots in `estop` get an e-stop packet instead."""
        if self.key is None:   # unsigned packets would be dropped by the bridge
            return
        now, wall = time.monotonic(), time.time()
        for rid, cmd in cmds.items():
            st = self._st.get(rid)
            if st is None:
                continue
            if not st["addr"]:
                if st["host"]:
                    self._retry_resolve(rid, now)
                continue
            with self._lock:
                st["seq"] += 1
                seq = st["seq"]
                sent = self._sent_t[rid]
                sent[seq] = [now, False]
                if len(sent) > 256:
                    for k in sorted(sent)[:-128]:
                        del sent[k]
            stop = rid in estop
            data = packet(self.key, rid, self.sid, seq, cmd, self.params, wall, estop=stop)
            try:
                self.sock.sendto(data, (st["addr"], self.port))
                st["sent"] += 1
                st["last_cmd"] = [0.0, 0.0, 0.0] if stop else list(clamp_cmd(cmd, self.params))
                if stop:
                    st["estop_sent"] += 1
            except OSError as e:
                st["error"] = f"send: {e}"
                self._retry_resolve(rid, now)   # the address may have changed (DHCP, Wi-Fi rejoin)

    def stop_all(self, burst: int = 3) -> None:
        zero = {rid: Cmd.stop() for rid in self.robots}
        for _ in range(burst):
            self.send(zero)

    def estop(self, robots=None, burst: int = ESTOP_BURST[0], gap_s: float = ESTOP_BURST[1]) -> list[str]:
        """Signed e-stop burst to `robots` (default: all). Blocks for (burst-1)*gap_s. Returns the
        robots it was addressed to (sending needs the key and a resolved address)."""
        ids = [r for r in (robots or list(self.robots)) if r in self._st]
        for i in range(burst):
            self.send({rid: Cmd.stop("estop") for rid in ids}, estop=set(ids))
            if i + 1 < burst:
                time.sleep(gap_s)
        return ids

    # ---- acks --------------------------------------------------------------
    def _listen(self) -> None:
        while not self._stop:
            try:
                data, (ip, _port) = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if self._stop:
                    return
                time.sleep(0.05)
                continue
            self.handle_ack(data, ip, time.monotonic())

    def handle_ack(self, data: bytes, ip: str, now: float) -> str | None:
        """Verify and parse one reply; returns the robot id it belonged to (or None)."""
        rid = self._by_addr.get(ip)
        if rid is None or self.key is None:
            return None
        try:
            msg = json.loads(data)
            seq = int(msg["ack"])
        except (ValueError, KeyError, TypeError):
            return None
        with self._lock:
            st = self._st[rid]
            if not (verify(self.key, msg) and msg.get("robot") == rid and msg.get("sid") == self.sid):
                st["bad_acks"] += 1
                st["bad_ack_t"] = now
                return None
            if seq <= st["last_ack_seq"]:   # duplicate/replayed ack: no liveness credit
                return None
            st["last_ack_seq"] = seq
            st["acks"] += 1
            st["last_ack_t"] = now
            rec = self._sent_t[rid].get(seq)
            if rec is not None:
                rec[1] = True
                st["rtt_ms"] = round((now - rec[0]) * 1000, 1)
                st["rtts"].append(st["rtt_ms"])
            if isinstance(msg.get("imu"), dict):
                st["imu"] = msg["imu"]
            if msg.get("profile"):
                st["profile"] = str(msg["profile"])
            st["bridge_ok"] = msg.get("ok")
            if isinstance(msg.get("estopped"), bool):   # SPEC v1.1; older bridges omit it (None)
                st["estopped"] = msg["estopped"]
            if isinstance(msg.get("bridge"), dict):
                st["bridge"] = msg["bridge"]
            self._note_gyro(st, msg.get("gz"), now)
            # Fields newer bridges add (imu_hold, mux, ...) are kept as-is for the UI and run log.
            st["extra"] = {k: v for k, v in msg.items() if k not in ACK_CORE}
            self.ack_log.append({"r": rid, "ack": seq, "rtt": st["rtt_ms"]})
            if len(self.ack_log) > 1000:
                del self.ack_log[:500]
        return rid

    @staticmethod
    def _note_gyro(st: dict, gz, now: float) -> None:
        """Ack "gz" (drive_bridge.YawRate; bridges before Nov 25 omit it) -> (t, yaw, wz) on this clock.
        Sample time = ack receipt - sample age at the ack - half the round trip (the ack's one-way trip)."""
        if not isinstance(gz, dict):
            return
        try:
            yaw, wz, n, ms = float(gz["yaw"]), float(gz["wz"]), int(gz["n"]), float(gz["ms"])
        except (KeyError, TypeError, ValueError):
            return
        if not (math.isfinite(yaw) and math.isfinite(wz) and math.isfinite(ms)) or not 0 <= ms < 1000:
            return
        if n < st["gz_n"]:          # bridge restarted: its integral starts over
            st["gz"].clear()
        elif n == st["gz_n"]:       # no new IMU sample since the previous ack
            return
        st["gz_n"] = n
        t = now - ms / 1000 - (st["rtt_ms"] or 0.0) / 2000
        if st["gz"] and t <= st["gz"][-1][0]:
            return
        st["gz"].append((t, yaw, wz))

    def gyro(self, rid: str) -> list[tuple[float, float, float]]:
        """Measured yaw-rate samples (t monotonic, integrated world yaw rad, body wz rad/s), oldest first."""
        with self._lock:
            st = self._st.get(rid)
            return list(st["gz"]) if st else []

    def drain_acks(self) -> list[dict]:
        with self._lock:
            out, self.ack_log = self.ack_log, []
        return out

    def status(self) -> dict[str, dict]:
        now = time.monotonic()
        out = {}
        with self._lock:
            for rid, st in self._st.items():
                age = None if st["last_ack_t"] is None else round(now - st["last_ack_t"], 3)
                if self.key is None:
                    auth = "missing_key"
                elif st["bad_ack_t"] is not None and (st["last_ack_t"] is None or st["bad_ack_t"] > st["last_ack_t"]):
                    auth = "bad_ack"
                else:
                    auth = "ok"
                rtts = sorted(st["rtts"])
                window = [a for t, a in self._sent_t[rid].values() if now - t > LOSS_AFTER_S]
                bridge = st["bridge"] if isinstance(st["bridge"], dict) else {}
                extra = st["extra"]
                out[rid] = {"host": st["host"], "via": st["via"], "addr": st["addr"], "error": st["error"],
                            "tailscale": is_tailscale(st["addr"]),
                            "rtt_p95_ms": rtts[min(len(rtts) - 1, int(0.95 * len(rtts)))] if rtts else None,
                            "loss": round(1 - sum(window) / len(window), 3) if len(window) >= 10 else None,
                            "hold": extra.get("hold", bridge.get("hold")),           # why the bridge sends zero
                            "imu_hold": extra.get("imu_hold", bridge.get("imu_hold")),
                            "mux_ok": extra.get("mux_ok"), "extra": extra,
                            "estopped": st["estopped"], "estop_sent": st["estop_sent"],
                            "auth": auth, "auth_error": self.key_error, "bad_acks": st["bad_acks"],
                            "connected": age is not None and age < CONNECTED_AGE_S,
                            "ack_age": age, "rtt_ms": st["rtt_ms"], "imu": st["imu"],
                            "profile": st["profile"], "bridge_ok": st["bridge_ok"], "bridge": st["bridge"], "seq": st["seq"], "sent": st["sent"],
                            "acks": st["acks"], "last_cmd": st["last_cmd"],
                            "gyro_age": round(now - st["gz"][-1][0], 3) if st["gz"] else None}
        return out

    def close(self) -> None:
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass


def load_udp_hosts(path=CONFIG / "robots.json") -> dict[str, str]:
    """Optional per-robot "udp_host" from robots.json (LAN address for control packets)."""
    try:
        return {str(r["id"]): str(r.get("udp_host") or "") for r in json.loads(path.read_text())}
    except (OSError, ValueError, KeyError, TypeError):
        return {}
