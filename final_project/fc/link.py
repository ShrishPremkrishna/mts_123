"""Cmd -> UDP datagram to each robot's Pi (SPEC wire protocol), plus ack/health status.

Stdlib only. One socket for all robots; a listener thread matches acks to robots by
the sender's address. Hard speed ceilings apply here no matter what params say.
Packets and acks are HMAC-signed with the shared key in config/link.key
(`python -m fc.robot.keygen`); without a key nothing is sent.
"""
from __future__ import annotations

import json
import math
import secrets
import socket
import subprocess
import threading
import time

from .core import CONFIG, Cmd, RobotCfg
from .robot.drive_bridge import VERSION, KeyFileError, load_key, sign, verify

PORT = 9870
CEIL = {"vx": 0.5, "vy": 0.3, "wz": 2.0}   # never exceeded, whatever params say
TTL_RANGE = (0.05, 0.5)   # drive_bridge MAX_TTL is 0.5 s (mux timeout)
CONNECTED_AGE_S = 1.0                     # ack younger than this counts as connected
KEY_FILE = CONFIG / "link.key"


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


def packet(key: bytes, robot: str, sid: str, seq: int, cmd: Cmd, params: dict, t: float | None = None) -> bytes:
    """Signed datagram. `t` is laptop wall-clock time: the Pi only compares it with earlier
    packets' t (replay window), and wall time keeps increasing across laptop restarts."""
    vx, vy, wz = clamp_cmd(cmd, params)
    ttl = max(TTL_RANGE[0], min(TTL_RANGE[1], _num(params.get("ttl_s", 0.25)) or 0.25))
    msg = {"v": VERSION, "robot": robot, "sid": sid, "seq": seq, "t": round(time.time() if t is None else t, 4),
           "vx": round(vx, 4), "vy": round(vy, 4), "wz": round(wz, 4), "ttl": ttl}
    return sign(key, msg)


def load_link_key(path=KEY_FILE) -> tuple[bytes | None, str | None]:
    try:
        return load_key(str(path)), None
    except KeyFileError as e:
        return None, str(e)


class Link:
    def __init__(self, robots: list[RobotCfg], params: dict, port: int = PORT, resolve=resolve_host,
                 key_file=KEY_FILE):
        self.params, self.port = params, port
        self.robots = {r.id: r for r in robots}
        self.key, self.key_error = load_link_key(key_file)
        self.sid = secrets.token_hex(8)   # new sender session per process; the bridge resumes on it
        self._lock = threading.Lock()
        self._st = {r.id: {"host": r.host, "addr": None, "error": None if r.host else "no host",
                           "seq": 0, "sent": 0, "acks": 0, "last_ack_t": None, "rtt_ms": None, "last_ack_seq": 0,
                           "bad_acks": 0, "bad_ack_t": None,
                           "imu": None, "profile": None, "bridge_ok": None, "bridge": None, "last_cmd": [0.0, 0.0, 0.0]} for r in robots}
        self._sent_t: dict[str, dict[int, float]] = {r.id: {} for r in robots}
        self._by_addr: dict[str, str] = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.2)
        self._stop = False
        self.ack_log: list[dict] = []  # drained by the app into the run log
        threading.Thread(target=self._listen, daemon=True, name="link-acks").start()
        threading.Thread(target=self._resolve_all, args=(resolve,), daemon=True, name="link-resolve").start()

    def _resolve_all(self, resolve) -> None:
        for rid, r in self.robots.items():
            if not r.host:
                continue
            ip = resolve(r.host)
            with self._lock:
                st = self._st[rid]
                st["addr"], st["error"] = ip, None if ip else f"cannot resolve {r.host}"
                if ip:
                    self._by_addr[ip] = rid

    # ---- sending -----------------------------------------------------------
    def send(self, cmds: dict[str, Cmd]) -> None:
        if self.key is None:   # unsigned packets would be dropped by the bridge
            return
        now, wall = time.monotonic(), time.time()
        for rid, cmd in cmds.items():
            st = self._st.get(rid)
            if st is None or not st["addr"]:
                continue
            with self._lock:
                st["seq"] += 1
                seq = st["seq"]
                sent = self._sent_t[rid]
                sent[seq] = now
                if len(sent) > 128:
                    for k in sorted(sent)[:-64]:
                        del sent[k]
            data = packet(self.key, rid, self.sid, seq, cmd, self.params, wall)
            try:
                self.sock.sendto(data, (st["addr"], self.port))
                st["sent"] += 1
                st["last_cmd"] = list(clamp_cmd(cmd, self.params))
            except OSError as e:
                st["error"] = f"send: {e}"

    def stop_all(self, burst: int = 3) -> None:
        zero = {rid: Cmd.stop() for rid in self.robots}
        for _ in range(burst):
            self.send(zero)

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
            t0 = self._sent_t[rid].get(seq)
            if t0 is not None:
                st["rtt_ms"] = round((now - t0) * 1000, 1)
            if isinstance(msg.get("imu"), dict):
                st["imu"] = msg["imu"]
            if msg.get("profile"):
                st["profile"] = str(msg["profile"])
            st["bridge_ok"] = msg.get("ok")
            if isinstance(msg.get("bridge"), dict):
                st["bridge"] = msg["bridge"]
            self.ack_log.append({"r": rid, "ack": seq, "rtt": st["rtt_ms"]})
            if len(self.ack_log) > 1000:
                del self.ack_log[:500]
        return rid

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
                out[rid] = {"host": st["host"], "addr": st["addr"], "error": st["error"],
                            "auth": auth, "auth_error": self.key_error, "bad_acks": st["bad_acks"],
                            "connected": age is not None and age < CONNECTED_AGE_S,
                            "ack_age": age, "rtt_ms": st["rtt_ms"], "imu": st["imu"],
                            "profile": st["profile"], "bridge_ok": st["bridge_ok"], "bridge": st["bridge"], "seq": st["seq"], "sent": st["sent"],
                            "acks": st["acks"], "last_cmd": st["last_cmd"]}
        return out

    def close(self) -> None:
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass
