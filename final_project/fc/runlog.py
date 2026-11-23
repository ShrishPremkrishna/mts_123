"""Per-run log folder: params.json, events.jsonl (compact, loop rate), summary.json.

Writes go through a queue to one writer thread, so the control loop never waits on disk.
Stdlib only.
"""
from __future__ import annotations

import json
import queue
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

RUNS = Path(__file__).parent / "runs"


def _r(v, nd=3):
    """Round floats recursively so events.jsonl stays small."""
    if isinstance(v, float):
        return round(v, nd)
    if isinstance(v, dict):
        return {k: _r(x, nd) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_r(x, nd) for x in v]
    return v


class RunLog:
    def __init__(self, test: str, params: dict, meta: dict | None = None, root: Path = RUNS):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.dir = root / f"{stamp}-{test}"
        n = 1
        while self.dir.exists():
            n += 1
            self.dir = root / f"{stamp}-{test}-{n}"
        self.dir.mkdir(parents=True)
        self.test, self.t0 = test, time.monotonic()
        self.n_events = 0
        self._q: queue.Queue = queue.Queue(maxsize=20000)
        self.dropped = 0
        (self.dir / "params.json").write_text(json.dumps({"params": params, **(meta or {})}, indent=1))
        self._f = open(self.dir / "events.jsonl", "a", buffering=1 << 16)
        self._thread = threading.Thread(target=self._writer, daemon=True, name=f"runlog-{test}")
        self._thread.start()
        self.closed = False

    def event(self, rec: dict) -> None:
        """Queue one record (never blocks). `dt` = seconds since run start is added."""
        if self.closed:
            return
        rec = {"dt": round(time.monotonic() - self.t0, 3), **rec}
        try:
            self._q.put_nowait(rec)
            self.n_events += 1
        except queue.Full:
            self.dropped += 1

    def _writer(self) -> None:
        while True:
            rec = self._q.get()
            if rec is None:
                break
            try:
                self._f.write(json.dumps(_r(rec), separators=(",", ":")) + "\n")
            except (TypeError, ValueError) as e:
                self._f.write(json.dumps({"log_error": str(e)}) + "\n")
        self._f.flush()
        self._f.close()

    def finish(self, summary: dict) -> dict:
        """Flush events and write summary.json. Called off the control loop."""
        if self.closed:
            return summary
        self.closed = True
        summary = {"test": self.test, "duration_s": round(time.monotonic() - self.t0, 2),
                   "events": self.n_events, "dropped": self.dropped, **summary}
        self._q.put(None)
        self._thread.join(timeout=5)
        (self.dir / "summary.json").write_text(json.dumps(_r(summary), indent=1))
        return summary


def list_runs(root: Path = RUNS, limit: int = 30) -> list[dict]:
    if not root.exists():
        return []
    out = []
    for d in sorted((p for p in root.iterdir() if p.is_dir()), reverse=True)[:limit]:
        s = d / "summary.json"
        summary = None
        if s.exists():
            try:
                summary = json.loads(s.read_text())
            except ValueError:
                pass
        out.append({"name": d.name, "summary": summary})
    return out


LEVELS = ("ok", "warn", "bad")


class Metrics:
    """Per-run numbers every summary.json carries, accumulated at loop rate (cheap, no disk).

    Per robot: seen fraction, path length (overhead camera), stale stops, wz reversals and fast
    ones (< 1 s after the previous), spin fraction (|wz| > 0.8 with |vx| < 0.05), worst IMU
    level, packets sent/acked, loss, RTT p95, final pose. Plus run time and loop overruns."""

    def __init__(self, robots: list[str], t0: float, overruns0: int = 0, link0: dict | None = None):
        self.t0, self.overruns0 = t0, overruns0
        link0 = link0 or {}
        self.r = {rid: {"ticks": 0, "seen": 0, "path": 0.0, "last": None, "stale": 0, "was_stale": False,
                        "rev": 0, "fast_rev": 0, "sign": 0, "rev_t": None, "spin": 0, "imu": None, "rtts": [],
                        "sent0": (link0.get(rid) or {}).get("sent", 0), "acks0": (link0.get(rid) or {}).get("acks", 0)}
                  for rid in robots}

    def tick(self, now: float, world, cmds: dict, stale_s: float, link: dict | None = None, acks=None) -> None:
        for rid, m in self.r.items():
            m["ticks"] += 1
            obs = world.robots.get(rid) if world is not None else None
            if obs is not None and now - obs.t <= stale_s:
                m["seen"] += 1
            if obs is not None:
                p = (obs.t, obs.pose.x, obs.pose.y, obs.pose.yaw)
                last = m["last"]
                if last is None or p[0] > last[0]:
                    if last is not None:
                        d = ((p[1] - last[1]) ** 2 + (p[2] - last[2]) ** 2) ** 0.5
                        if d < 0.5:   # bigger jumps are tag swaps / sim resets, not travel
                            m["path"] += d
                    m["last"] = p
            c = cmds.get(rid)
            if c is not None:
                vx, _vy, wz, tag = c
                stale = tag == "stale"
                m["stale"] += stale and not m["was_stale"]
                m["was_stale"] = stale
                if abs(wz) > 0.1:
                    s = 1 if wz > 0 else -1
                    if m["sign"] and s != m["sign"]:
                        m["rev"] += 1
                        if m["rev_t"] is not None and now - m["rev_t"] < 1.0:
                            m["fast_rev"] += 1
                        m["rev_t"] = now
                    m["sign"] = s
                m["spin"] += abs(wz) > 0.8 and abs(vx) < 0.05
            lvl = (((link or {}).get(rid) or {}).get("imu") or {}).get("level")
            if lvl in LEVELS and (m["imu"] is None or LEVELS.index(lvl) > LEVELS.index(m["imu"])):
                m["imu"] = lvl
        for a in acks or ():
            m = self.r.get(a.get("r"))
            if m is not None and isinstance(a.get("rtt"), (int, float)):
                m["rtts"].append(a["rtt"])

    def summary(self, now: float, overruns: int = 0, link: dict | None = None) -> dict:
        el = max(now - self.t0, 1e-6)
        out = {"time_s": round(el, 2), "loop_overruns": overruns - self.overruns0, "robots": {}}
        for rid, m in self.r.items():
            st = (link or {}).get(rid) or {}
            sent = st.get("sent", 0) - m["sent0"] if st else None
            acked = st.get("acks", 0) - m["acks0"] if st else None
            rtts = sorted(m["rtts"])
            n = max(m["ticks"], 1)
            out["robots"][rid] = {
                "seen_frac": round(m["seen"] / n, 3), "path_m": round(m["path"], 3), "stale_stops": m["stale"],
                "wz_reversals": m["rev"], "fast_reversals": m["fast_rev"],
                "fast_rev_per_min": round(m["fast_rev"] * 60 / el, 2), "spin_frac": round(m["spin"] / n, 3),
                "imu_worst": m["imu"], "sent": sent, "acked": acked,
                "loss": round(1 - acked / sent, 3) if sent else None,
                "rtt_p95_ms": rtts[min(len(rtts) - 1, int(0.95 * len(rtts)))] if rtts else None,
                "final": [round(v, 3) for v in m["last"][1:]] if m["last"] else None}
        return out
