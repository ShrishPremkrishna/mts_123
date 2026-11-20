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
import json
import os
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import cv2

HELP = ("If the phone is not listed: plug it in by USB and unlock it once to trust this computer "
        "(iPhone: then lock it and leave it still; wireless also works with Wi-Fi and Bluetooth on, same Apple ID).")
VIDEO_SUFFIXES = {".mov", ".mp4", ".m4v", ".avi", ".mkv", ".webm"}
BUILT_IN = ("macbook", "facetime", "built-in", "imac", "studio display", "isight")
PHONES = ("iphone", "ipad", "android", "pixel", "galaxy", "oneplus", "camo", "iriun", "droidcam", "epoccam")


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


def list_cameras() -> list[dict]:
    """The cameras macOS offers right now, in the order OpenCV numbers them.

    OpenCV's macOS backend sorts capture devices by their unique ID (see
    orderedVideoCaptureDevices in cap_avfoundation_mac.mm), so sorting the system's
    own list the same way gives the index OpenCV will use for each one. The numbers
    shift whenever a camera is plugged in or goes away; the names do not.
    Returns [{"index", "name", "model", "built_in"}]; empty off macOS.
    """
    if sys.platform != "darwin":
        return []
    try:
        out = subprocess.run(["system_profiler", "SPCameraDataType", "-json"], capture_output=True, text=True, timeout=8).stdout
        found = json.loads(out).get("SPCameraDataType", [])
    except (OSError, subprocess.SubprocessError, ValueError):
        return []
    found.sort(key=lambda c: c.get("spcamera_unique-id", ""))
    return [{"index": i, "name": c.get("_name", "?"), "model": c.get("spcamera_model-id", ""),
             "built_in": any(w in (c.get("_name", "") + " " + c.get("spcamera_model-id", "")).lower() for w in BUILT_IN)}
            for i, c in enumerate(found)]


def phone_on_cable() -> bool | None:
    """True if a phone is plugged into this Mac by USB, False if not, None if that cannot be told."""
    if sys.platform != "darwin":
        return None
    try:
        out = subprocess.run(["system_profiler", "SPUSBDataType"], capture_output=True, text=True, timeout=8).stdout.lower()
    except (OSError, subprocess.SubprocessError):
        return None
    return any(w in out for w in ("iphone", "ipad", "android", "pixel", "galaxy"))


def camera_names() -> list[str]:
    """Names of the cameras macOS currently offers (for messages)."""
    return [c["name"] for c in list_cameras()]


def is_camera_name(source) -> bool:
    """True for a camera given by NAME ("phone", "iPhone", "Pixel 8"...), as opposed to an
    index, the simulator, a stream URL or a recorded video file."""
    if not isinstance(source, str) or source.isdigit() or source == "sim" or "://" in source:
        return False
    return Path(source).suffix.lower() not in VIDEO_SUFFIXES and not Path(source).exists()


def find_camera(name: str, cameras: list[dict] | None = None) -> dict | None:
    """The camera a name refers to, or None if it is not connected right now.

    "phone" (or "external") = a phone-looking camera if there is one, otherwise any
    camera that is not the computer's own. Anything else matches part of a camera's
    name or model, ignoring case and the kind of apostrophe.
    """
    cameras = list_cameras() if cameras is None else cameras
    plain = lambda t: t.lower().replace("\u2019", "'")  # noqa: E731
    want = plain(name.strip())
    if want in ("phone", "external"):
        phones = [c for c in cameras if any(w in plain(c["name"] + " " + c["model"]) for w in PHONES)]
        others = [c for c in cameras if not c["built_in"]]
        return (phones or others or [None])[0]
    for c in cameras:
        if want in plain(c["name"]) or want in plain(c["model"]):
            return c
    return None


def is_live(source) -> bool:
    """A camera (index or name) or a network stream URL is live; a video file is not."""
    return isinstance(source, int) or (isinstance(source, str) and "://" in source) or is_camera_name(source)


class FrameGrabber:
    def __init__(self, source, width: int, height: int, fps: int, wait: float | None = 10.0,
                 stall_s: float = 2.0, say=print, open_fn=None, cameras_fn=None):
        """source: a camera index, a camera NAME ("phone", "iPhone"...), a stream URL or a video file.
        A name is looked up again on every (re)connect, because the index of a phone changes
        as cameras come and go.
        wait: seconds to keep trying to open the camera (None = for ever).
        stall_s: no frame for this long counts as a lost feed.
        say: where status messages go. open_fn / cameras_fn: stand-ins for tests."""
        self.source, self._want, self.stall_s, self._say = source, (width, height, fps), stall_s, say
        self._open_fn, self._cameras_fn = open_fn, cameras_fn
        self._by_name = is_camera_name(source)
        self.device = None  # {"index", "name", ...} of the camera in use, when chosen by name
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
        target = self.source
        if self._by_name:  # find which number that camera has right now
            found = find_camera(self.source, self._cameras_fn() if self._cameras_fn else None)
            if found is None:
                return None
            target = found["index"]
            if self.device is None or self.device["name"] != found["name"] or self.device["index"] != found["index"]:
                self._say(f"camera {self.source!r} -> {found['name']} (number {found['index']} right now)")
            self.device = found
        with _quiet():
            if self._open_fn is not None:
                cap = self._open_fn(target)
            elif isinstance(target, int) and sys.platform == "darwin":
                cap = cv2.VideoCapture(target, cv2.CAP_AVFOUNDATION)
            else:
                cap = cv2.VideoCapture(target)
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

    def file_time_s(self, previous: float | None, constant_fps: float | None = None) -> float:
        """Presentation time of the file frame just read, independent of decode speed."""
        if not self.is_file or self._n < 1:
            raise ValueError("Read a video-file frame before requesting its presentation time")
        if constant_fps is not None:
            if not (0 < constant_fps < float("inf")):
                raise ValueError("Replay frame rate must be finite and positive")
            return (self._n - 1) / constant_fps
        stamp = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000
        if not (0 <= stamp < float("inf")) or (previous is not None and stamp <= previous):
            raise ValueError(f"No increasing presentation timestamp at video frame {self._n}. "
                             "For footage recorded at a known constant rate, supply --replay-fps.")
        return stamp

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
