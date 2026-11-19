"""Camera capture on a background thread: always hand out the newest frame.

Built to survive a wireless phone camera (Continuity Camera over Wi-Fi), which
can be missing at start and can drop out mid-run:

- at start it waits for the camera instead of failing,
- if frames stop it says so (`status == "lost"`), lets go of the camera and
  keeps trying to reopen it, and carries on when it is back,
- `close()` always releases the camera, so the phone is not left held.

A lost feed hands out NO frames. Callers must treat that as "nothing is known
right now", never reuse the last frame as if it were current.
"""
from __future__ import annotations

import atexit
import os
import subprocess
import sys
import threading
import time
from contextlib import contextmanager

import cv2

HELP = ("If the iPhone is not listed: lock it, leave it still and near the Mac, Wi-Fi and Bluetooth on for both, "
        "same Apple ID. Plugging it in by USB also works.")


@contextmanager
def _quiet():
    """OpenCV prints a native warning on every failed open; hide it while we retry."""
    try:
        sys.stderr.flush()
        saved, null = os.dup(2), os.open(os.devnull, os.O_WRONLY)
    except OSError:
        yield
        return
    os.dup2(null, 2)
    try:
        yield
    finally:
        os.dup2(saved, 2)
        os.close(null)
        os.close(saved)


def camera_names() -> list[str]:
    """Names of the cameras macOS currently offers (for messages only; empty elsewhere)."""
    if sys.platform != "darwin":
        return []
    try:
        out = subprocess.run(["system_profiler", "SPCameraDataType"], capture_output=True, text=True, timeout=8).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [ln.strip().rstrip(":") for ln in out.splitlines()
            if ln.startswith("    ") and not ln.startswith("     ") and ln.strip().endswith(":")]


def is_live(source) -> bool:
    """A camera index or a network stream URL is live; any other string is a recorded file."""
    return isinstance(source, int) or (isinstance(source, str) and "://" in source)


class FrameGrabber:
    def __init__(self, source, width: int, height: int, fps: int, wait: float | None = 10.0,
                 stall_s: float = 2.0, say=print, open_fn=None):
        """wait: seconds to keep trying to open the camera (None = for ever).
        stall_s: no frame for this long counts as a lost feed.
        say: where status messages go. open_fn: replaces cv2.VideoCapture in tests."""
        self.source, self._want, self.stall_s, self._say = source, (width, height, fps), stall_s, say
        self._open_fn = open_fn
        self.is_file = not is_live(source)
        self._lock = threading.Lock()
        self._frame, self._t, self._n = None, 0.0, 0
        self._stop, self._gen = False, 0
        self._t_ok = self._t_alive = time.time()
        self.reconnects = 0

        self.cap = self._open()
        t0, told = time.time(), 0.0
        while self.cap is None:
            if self.is_file or (wait is not None and time.time() - t0 >= wait):
                raise RuntimeError(f"could not open camera source {source!r}. " + ("" if self.is_file else HELP))
            if time.time() - told > 6.0:
                told = time.time()
                names = camera_names()
                self._say(f"waiting for camera {source!r}..." + (f" macOS sees: {', '.join(names)}." if names else "")
                          + " " + HELP)
            time.sleep(1.0)
            self.cap = self._open()
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._t_ok = self._t_alive = time.time()
        # Whatever script uses this, and however it ends (normally, an error, Ctrl-C), let go
        # of the camera. A wireless phone left held by a dead program stayed unavailable to the next run.
        atexit.register(self.close)
        if not self.is_file:
            threading.Thread(target=self._loop, args=(self._gen, self.cap), daemon=True).start()

    # ---- opening -----------------------------------------------------------
    def _open(self):
        with _quiet():
            if self._open_fn is not None:
                cap = self._open_fn(self.source)
            elif isinstance(self.source, int) and sys.platform == "darwin":
                cap = cv2.VideoCapture(self.source, cv2.CAP_AVFOUNDATION)
            else:
                cap = cv2.VideoCapture(self.source)
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()
                return None
            if not self.is_file:
                w, h, fps = self._want
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                cap.set(cv2.CAP_PROP_FPS, fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    # ---- background reader -------------------------------------------------
    def _loop(self, gen: int, cap):
        while not self._stop and gen == self._gen:
            ok, frame = cap.read()
            self._t_alive = time.time()
            if gen != self._gen:   # replaced while this read was blocked: leave everything alone
                return
            if ok and frame is not None:
                with self._lock:
                    self._frame, self._t, self._n = frame, self._t_alive, self._n + 1
                self._t_ok = self._t_alive
                continue
            if time.time() - self._t_ok < self.stall_s:
                time.sleep(0.005)
                continue
            # The feed is gone. Let go of the camera and keep trying to get it back.
            cap.release()
            cap = None
            while cap is None and not self._stop and gen == self._gen:
                time.sleep(1.0)
                self._t_alive = time.time()
                cap = self._open()
            if cap is None:
                return
            if gen != self._gen:
                cap.release()
                return
            self.cap = cap
            self.reconnects += 1
            self._t_ok = time.time()  # give the reopened camera a moment to deliver

    def _unstick(self):
        """The reader has been inside one read() for far too long: start a fresh reader on a
        fresh camera handle and leave the stuck one alone (touching it could hang too)."""
        self._gen += 1
        self._t_alive = time.time()

        def restart(gen):
            cap = None
            while cap is None and not self._stop and gen == self._gen:
                cap = self._open()
                if cap is None:
                    time.sleep(1.0)
            if cap is not None and gen == self._gen:
                self.cap = cap
                self.reconnects += 1
                self._t_ok = self._t_alive = time.time()
                self._loop(gen, cap)

        threading.Thread(target=restart, args=(self._gen,), daemon=True).start()

    # ---- for callers ---------------------------------------------------------
    @property
    def lost_for(self) -> float:
        """Seconds since the last good frame, once that exceeds stall_s; otherwise 0."""
        if self.is_file:
            return 0.0
        gap = time.time() - self._t_ok
        return gap if gap > self.stall_s else 0.0

    @property
    def status(self) -> str:
        return "lost" if self.lost_for else "ok"

    def read(self, last_n: int = -1, timeout: float = 1.0):
        """Returns (frame, t_received, frame_number); blocks until a frame newer than last_n.
        (None, 0.0, last_n) on timeout, which is what a lost feed looks like."""
        if self.is_file:  # recorded video: step through every frame
            ok, frame = self.cap.read()
            self._n += 1
            return (frame, time.time(), self._n) if ok else (None, 0.0, self._n)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._frame is not None and self._n != last_n:
                    return self._frame, self._t, self._n
            time.sleep(0.001)
        if time.time() - self._t_alive > self.stall_s + 10.0:
            self._unstick()
        return None, 0.0, last_n

    def close(self):
        """Stop reading and release the camera (so a wireless phone is not left held)."""
        self._stop = True
        self._gen += 1
        try:
            self.cap.release()
        except Exception:  # noqa: BLE001 - closing must never raise
            pass

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
