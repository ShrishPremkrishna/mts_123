"""Camera capture on a background thread: always hand out the newest frame."""
from __future__ import annotations

import sys
import threading
import time

import cv2


class FrameGrabber:
    def __init__(self, source, width: int, height: int, fps: int):
        if isinstance(source, int) and sys.platform == "darwin":
            self.cap = cv2.VideoCapture(source, cv2.CAP_AVFOUNDATION)
        else:
            self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise RuntimeError(f"could not open camera source {source!r}")
        self.is_file = isinstance(source, str)
        if not self.is_file:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._lock = threading.Lock()
        self._frame, self._t, self._n = None, 0.0, 0
        self._stop = False
        if not self.is_file:
            threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self._stop:
            ok, frame = self.cap.read()
            t = time.time()
            if not ok:
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame, self._t, self._n = frame, t, self._n + 1

    def read(self, last_n: int = -1, timeout: float = 1.0):
        """Returns (frame, t_received, frame_number); blocks until a frame newer than last_n."""
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
        return None, 0.0, last_n

    def close(self):
        self._stop = True
        self.cap.release()
