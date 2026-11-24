"""Eye speed benchmark on a recorded overhead video (no camera needed).

    .venv-fc/bin/python -m fc.tools.eye_bench fc/runs/<run>/overhead.mp4            # this checkout
    .venv-fc/bin/python -m fc.tools.eye_bench VIDEO --ref HEAD~1                   # + an older eye.py
    .venv-fc/bin/python -m fc.tools.eye_bench VIDEO --mode live --seconds 20

Recorded runs are 1280 px (the recorder downsizes); frames are upscaled to --width (default
1920, what the phone delivers live) so detector costs match the live picture. Upscaled frames
are softer than real ones, so detection counts are a pessimistic comparison, not absolute.

sync: every frame through Eye.process() in turn; per-stage ms (tags = everything behind
      perc.tag_det.detect, ball = perc.ball_det.detect, process = whole frame) plus the
      annotated-JPEG, Play-view JPEG and recorder costs, and how many frames saw each object.
live: the real threads: a 30 fps replay "camera" that keeps only the newest frame, Eye.start(),
      the run recorder, a 15 Hz /view.jpg poller, a 5 Hz /frame.jpg poller and a 30 Hz loop
      sampling eye.latest(). Reports eye fps, observation age (receipt -> use, excluding the
      fixed eye_latency_s) and stalls (no new measurement for > 0.3 s). The background scan
      thread's cost is in its own row (it is not on the hot path).
"""
from __future__ import annotations

import argparse
import importlib.util
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

FC = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(FC.parent))

import numpy as np  # noqa: E402

from fc.app import PARAM_FALLBACKS, load_robot_config  # noqa: E402
from fc.core import load_field, load_params  # noqa: E402


def load_eye_module(ref: str | None):
    if ref is None:
        import fc.eye as m
        return m
    src = subprocess.run(["git", "-C", str(FC), "show", f"{ref}:final_project/fc/eye.py"],
                         capture_output=True, text=True, check=True).stdout
    path = Path(tempfile.mkdtemp()) / "eye_ref.py"
    path.write_text(src)
    spec = importlib.util.spec_from_file_location("fc._eye_ref", path)
    m = importlib.util.module_from_spec(spec)
    sys.modules["fc._eye_ref"] = m
    spec.loader.exec_module(m)
    m.OP = FC.parent / "overhead_perception"   # it was computed from the temp file's location
    return m


class FileFrames:
    """Decoded, resized frames from the video, in order (looping when `loop`)."""

    def __init__(self, path, width: int, loop=False):
        import cv2
        self.cv2, self.path, self.loop = cv2, str(path), loop
        self.cap = cv2.VideoCapture(self.path)
        if not self.cap.isOpened():
            raise SystemExit(f"cannot open {path}")
        w0, h0 = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.width, self.height = width, int(round(h0 * width / w0)) // 2 * 2

    def next(self):
        ok, f = self.cap.read()
        if not ok and self.loop:
            self.cap.set(self.cv2.CAP_PROP_POS_FRAMES, 0)
            ok, f = self.cap.read()
        if not ok:
            return None
        if f.shape[1] != self.width:
            f = self.cv2.resize(f, (self.width, self.height), interpolation=self.cv2.INTER_LINEAR)
        return f


class ReplayCamera:
    """Stands in for LiveSource: a thread 'captures' at fps and keeps only the newest frame;
    read() blocks for a frame newer than the last one returned."""
    latency = False

    def __init__(self, frames: FileFrames, fps: float):
        self.frames, self.dt = frames, 1.0 / fps
        self.width, self.height = frames.width, frames.height
        self._cv, self._slot, self._n, self._got, self._stop = threading.Condition(), None, 0, 0, False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        nxt = time.monotonic()
        while not self._stop:
            f = self.frames.next()
            nxt += self.dt
            time.sleep(max(0.0, nxt - time.monotonic()))
            with self._cv:
                self._n += 1
                self._slot = (f, time.monotonic())
                self._cv.notify_all()

    def read(self):
        with self._cv:
            self._cv.wait_for(lambda: self._n != self._got, timeout=1.0)
            if self._n == self._got:
                return None, None
            self._got = self._n
            return self._slot

    def close(self):
        self._stop = True


def make_eye(mod, source: str):
    return mod.Eye(load_field(), load_robot_config()[0], {**PARAM_FALLBACKS, **load_params()}, source=source)


def stats(v) -> str:
    v = np.asarray(v, float)
    if not len(v):
        return "-"
    return (f"mean {v.mean():6.1f}  p50 {np.median(v):6.1f}  p95 {np.percentile(v, 95):6.1f}  "
            f"max {v.max():7.1f}")


def run_sync(mod, video, width, max_frames):
    import cv2
    frames = FileFrames(video, width)
    eye = make_eye(mod, video)
    eye._open_source = lambda: type("S", (), {"latency": False, "width": frames.width,
                                              "height": frames.height, "close": lambda s: None})()
    eye._setup()
    perc, T, seen = eye.perc, {k: [] for k in ("tags", "ball", "calibrate", "process", "annotate",
                                               "view_jpeg", "recorder")}, {}
    cur = {}

    def timed(fn, key):
        def w(*a, **k):
            t = time.perf_counter()
            try:
                return fn(*a, **k)
            finally:
                cur[key] = cur.get(key, 0.0) + time.perf_counter() - t
        return w
    perc.tag_det.detect = timed(perc.tag_det.detect, "tags")
    perc.ball_det.detect = timed(perc.ball_det.detect, "ball")
    eye._calibrate = timed(eye._calibrate, "calibrate")
    if hasattr(eye, "_scan_det"):   # this version: run the background scan inline, at its rate
        threading.Thread(target=eye._scan_loop, daemon=True).start()
    n, rec_w = 0, None
    tmp = Path(tempfile.mkdtemp())
    while max_frames is None or n < max_frames:
        f = frames.next()
        if f is None:
            break
        cur.clear()
        t_recv = time.monotonic()
        t = time.perf_counter()
        w = eye.process(f, t_recv)
        cur["process"] = time.perf_counter() - t
        if hasattr(eye, "_scan_slot"):
            with eye._scan_cv:
                eye._scan_slot = (f, n, t_recv)
                eye._scan_cv.notify()
        t = time.perf_counter()
        eye._annotate(f, w)
        cur["annotate"] = time.perf_counter() - t
        t = time.perf_counter()
        s = 1280 / f.shape[1]
        small = cv2.resize(f, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s < 1 else f
        cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 75])
        cur["view_jpeg"] = time.perf_counter() - t
        t = time.perf_counter()
        if rec_w is None:
            rec_w = cv2.VideoWriter(str(tmp / "rec.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 15, small.shape[1::-1])
        rec_w.write(small)
        cur["recorder"] = time.perf_counter() - t
        for k in T:
            if k in cur and (k != "calibrate" or cur[k] > 0):
                T[k].append(cur[k] * 1000)
        if w is not None:
            seen["frames"] = seen.get("frames", 0) + 1
            if w.ball and w.ball.t == t_recv:
                seen["ball"] = seen.get("ball", 0) + 1
            for k, o in w.robots.items():
                if o.t == t_recv:
                    seen[k] = seen.get(k, 0) + 1
        n += 1
        if hasattr(eye, "_scan_det"):
            time.sleep(0.001)   # let the scan thread take its frame now and then
    eye._stop = True
    if rec_w is not None:
        rec_w.release()
    print(f"\n[{mod.__name__}] sync, {n} frames at {frames.width}x{frames.height}  ({Path(video).parent.name})")
    print(f"  {eye.message}")
    print(f"  measured: {seen}")
    for k, v in T.items():
        if v:
            print(f"  {k:10s} {stats(v)}")
    p = np.asarray(T["process"][1:])
    print(f"  -> process-limited fps {1000 / p.mean():.1f}; frames > 100 ms: {(p > 100).sum()}, > 300 ms: {(p > 300).sum()}")
    if hasattr(eye, "stage_ms"):
        print(f"  background scan (not on the hot path): {eye.stage_ms}")
    return T, seen


def run_live(mod, video, width, seconds, cam_fps):
    from fc.media import Recorder
    frames = FileFrames(video, width, loop=True)
    cam = ReplayCamera(frames, cam_fps)
    eye = make_eye(mod, video)
    eye._open_source = lambda: cam
    eye.start()
    t0 = time.monotonic()
    while not (eye.perc and eye.perc.calibration_valid) and time.monotonic() - t0 < 20:
        time.sleep(0.05)
    rec = Recorder(Path(tempfile.mkdtemp()) / "overhead.mp4", eye.frame, fps=15.0)
    stop = False

    def poll(fn, hz):
        while not stop:
            fn()
            time.sleep(1 / hz)
    for fn, hz in ((eye.view_jpeg, 15), (eye.jpeg, 5)):
        threading.Thread(target=poll, args=(fn, hz), daemon=True).start()
    ages = {"ball": [], "robot": []}
    last_meas, gaps = {}, {"ball": [], "robot": []}
    f0, t_start = eye.frames, time.monotonic()
    while time.monotonic() - t_start < seconds:
        w = eye.latest()
        now = time.monotonic()
        if w:
            obs = {"ball": w.ball.t if w.ball else None}
            if w.robots:
                obs["robot"] = max(o.t for o in w.robots.values())
            for k, t in obs.items():
                if t is None:
                    continue
                ages[k].append((now - t) * 1000)
                if k in last_meas and t != last_meas[k]:
                    gaps[k].append(t - last_meas[k])
                last_meas[k] = t
        time.sleep(1 / 30)
    elapsed = time.monotonic() - t_start
    fps = (eye.frames - f0) / elapsed
    stop = True
    rec.stop()
    eye.close()
    cam.close()
    print(f"\n[{mod.__name__}] live, {elapsed:.0f} s at {cam.width}x{cam.height}, camera {cam_fps:.0f} fps, "
          f"with recorder + view/frame JPEG pollers  ({Path(video).parent.name})")
    print(f"  eye fps {fps:.1f}")
    for k in ages:
        g = np.asarray(gaps[k])
        stalls = int((g > 0.3).sum()) if len(g) else 0
        print(f"  {k:5s} age ms {stats(ages[k])} | gaps > 0.3 s: {stalls} ({stalls / elapsed * 60:.0f}/min), "
              f"longest {g.max() if len(g) else 0:.2f} s")
    if hasattr(eye, "stage_ms"):
        print(f"  stage_ms (smoothed): {eye.stage_ms}")
    return fps, ages, gaps


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("video", nargs="+")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--mode", choices=("sync", "live", "both"), default="sync")
    ap.add_argument("--frames", type=int, default=None, help="sync: stop after this many frames")
    ap.add_argument("--seconds", type=float, default=20.0, help="live: how long")
    ap.add_argument("--cam-fps", type=float, default=30.0)
    ap.add_argument("--ref", default=None, help="also run fc/eye.py from this git revision (e.g. HEAD~1)")
    ap.add_argument("--threads", type=int, default=None, help="cv2.setNumThreads for this run")
    a = ap.parse_args(argv)
    if a.threads is not None:
        import cv2
        cv2.setNumThreads(a.threads)
    mods = [load_eye_module(a.ref)] if a.ref else []
    mods.append(load_eye_module(None))
    for video in a.video:
        for mod in mods:
            if a.mode in ("sync", "both"):
                run_sync(mod, video, a.width, a.frames)
            if a.mode in ("live", "both"):
                run_live(mod, video, a.width, a.seconds, a.cam_fps)


if __name__ == "__main__":
    main()
