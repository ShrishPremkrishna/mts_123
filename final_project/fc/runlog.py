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
