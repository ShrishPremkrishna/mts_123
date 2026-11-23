"""Side threads for pictures: onboard camera feeds (MJPEG in) and run recording (MP4 out).

Neither ever takes the app lock or blocks the control loop: each owns a daemon thread and the
loop / HTTP handlers only read the latest result.

- CamFeed(url): reads a robot's MJPEG stream (fc/robot/camstream.py), keeps the newest JPEG.
  Frames are cut on JPEG start/end markers, so any multipart boundary works. Stdlib only.
- Recorder(path, grab): writes the overhead camera to an MP4 at a fixed rate. `grab()` returns
  (BGR frame, frame number); the latest frame is repeated when the camera is slower, so video
  time stays real time. Needs OpenCV (only used with the camera source).
"""
from __future__ import annotations

import threading
import time
import urllib.request

SOI, EOI = b"\xff\xd8", b"\xff\xd9"


class CamFeed:
    def __init__(self, url: str, stale_s: float = 2.0, retry_s: float = 2.0, timeout_s: float = 3.0):
        self.url, self.stale_s, self.retry_s, self.timeout_s = url, stale_s, retry_s, timeout_s
        self.jpeg: bytes | None = None
        self.t = 0.0                 # monotonic time of the newest frame
        self.frames = 0
        self.fps = 0.0
        self.error = "connecting"
        self._stop = False
        threading.Thread(target=self._run, daemon=True, name="fc-camfeed").start()

    def status(self) -> dict:
        age = time.monotonic() - self.t if self.t else None
        ok = age is not None and age < self.stale_s
        return {"ok": ok, "age": None if age is None else round(age, 2), "fps": round(self.fps, 1),
                "frames": self.frames, "error": None if ok else (self.error or "stalled")}

    def latest(self) -> bytes | None:
        return self.jpeg if self.status()["ok"] else None

    def close(self) -> None:
        self._stop = True

    def _run(self) -> None:
        while not self._stop:
            try:
                with urllib.request.urlopen(self.url, timeout=self.timeout_s) as r:
                    self.error = None
                    self._read(r)
                self.error = "stream ended"
            except Exception as e:  # noqa: BLE001 - keep retrying; the UI shows why
                self.error = f"{type(e).__name__}: {e}"[:120]
            time.sleep(self.retry_s)

    def _read(self, r) -> None:
        buf, prev = b"", None
        while not self._stop:
            chunk = r.read1(65536) if hasattr(r, "read1") else r.read(16384)
            if not chunk:
                return
            buf += chunk
            while True:
                a = buf.find(SOI)
                b = buf.find(EOI, a + 2) if a >= 0 else -1
                if a < 0 or b < 0:
                    break
                self.jpeg, buf = buf[a:b + 2], buf[b + 2:]
                now = time.monotonic()
                if prev is not None:
                    self.fps = 0.8 * self.fps + 0.2 / max(now - prev, 1e-3)
                prev, self.t = now, now
                self.frames += 1
            if len(buf) > 4_000_000:     # garbage without markers: drop it
                buf = b""


class Recorder:
    def __init__(self, path, grab, fps: float = 15.0, width: int = 1280):
        self.path, self.grab, self.fps, self.width = str(path), grab, float(fps), int(width)
        self.frames = 0
        self.error: str | None = None
        self._stop = False
        self.thread = threading.Thread(target=self._run, daemon=True, name="fc-recorder")
        self.thread.start()

    def stop(self) -> None:
        """Ask the thread to finish the file; returns at once."""
        self._stop = True

    def _run(self) -> None:
        try:
            import cv2
        except ImportError as e:
            self.error = f"no OpenCV: {e}"
            return
        writer, last, dt = None, None, 1.0 / self.fps
        nxt = time.monotonic()
        try:
            while not self._stop:
                frame, _n = self.grab()
                if frame is not None:
                    h, w = frame.shape[:2]
                    if w > self.width:
                        frame = cv2.resize(frame, (self.width, int(h * self.width / w) // 2 * 2),
                                           interpolation=cv2.INTER_AREA)
                    last = frame
                if last is not None:
                    if writer is None:
                        size = (last.shape[1], last.shape[0])
                        writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, size)
                        if not writer.isOpened():
                            self.error, writer = "VideoWriter did not open", None
                            return
                    # one write per tick missed so far (capped): video time stays real time
                    now = time.monotonic()
                    nxt = max(nxt, now - 1.0)      # a slow disk drops time rather than piling up
                    n = max(1, min(5, int((now - nxt) / dt) + 1))
                    for _ in range(n):
                        writer.write(last)
                    self.frames += n
                    nxt += n * dt
                else:
                    nxt += dt
                time.sleep(max(0.0, nxt - time.monotonic()))
        except Exception as e:  # noqa: BLE001 - recording must never take the app down
            self.error = f"{type(e).__name__}: {e}"
        finally:
            if writer is not None:
                writer.release()
