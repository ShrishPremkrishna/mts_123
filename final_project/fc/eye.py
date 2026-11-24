"""Overhead camera -> World. Thin adapter over overhead_perception/fieldcam (not edited).

Ported from Tej's overhead_kick.py Eye (origin/tej/overhead-kick, 2026-11-20):
  * lens-angle fit from the four corner tags when there is no intrinsics file
    (fixed 69 deg gave 9-14 px fit error; the fit gave 0.5-1.0 px),
  * calibration drift limit scaled to the picture width (8 px is for 896 px),
  * a second, relaxed tag detector in an enlarged crop for the small robot tags,
  * ball search area widened 0.55 m behind the goal lines,
  * raw per-frame detections (not the pipeline's lagging Kalman output).

Speed (2026-11-24; floor session ran at 10-11 fps with ~1 s stalls once a second):
the hot thread only looks where things were. Robot tags are found in a small crop around
their last position (standard detector at 1x, then 2x, then a tuned relaxed detector at 2x);
the ball in a crop around its last position (a half-resolution whole-field search finds it
when lost). A background "scan" thread runs the full-frame tag detection ~4x a second for
the calibration drift check and to (re)find robots, and the slow 2x whole-frame search for a
robot not seen at all (e.g. a configured robot that is not on the field) only every few
seconds. The old code ran that 2x search with the relaxed detector on every 10th frame on the
hot path: ~1 s per call on carpet texture (1000+ candidate quads), which was the periodic stall.
The annotated /frame.jpg is only drawn while someone is fetching it. Per-stage timings are
in status()["stage_ms"]; `python -m fc.tools.eye_bench VIDEO` measures them offline.

Sources: a live camera name/index ("phone"), a video file, a folder of images, or
"synthetic" (fieldcam's renderer with moving robots; truth kept for checking).

    python -m fc.eye --source synthetic --seconds 5       # offline self-check
"""
from __future__ import annotations

import collections
import math
import sys
import threading
import time
from pathlib import Path

from .core import Ball, Field, Pose, RobotCfg, RobotObs, World, wrap

OP = Path(__file__).resolve().parent.parent / "overhead_perception"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
KEEP_ROBOT_S = 1.0   # keep reporting a robot's last raw pose (with its old time) this long
BALL_VEL_WINDOW_S = 0.3
# Pixel sizes below are for a 1920 px wide picture and scale with the width.
ROI_HALF_PX = 110        # robot-tag crop half size around the last sighting
ROI_GROW_PX_S = 340      # ...grown by this per second since the last sighting (~0.6 m/s)
ROI_MAX_HALF_PX = 260
SEED_KEEP_S = 0.5        # look around a robot's last sighting this long; then the scan finds it
SCAN_PERIOD_S = 0.25     # background full-frame tag scan (floor-tag drift check, lost robots)
DEEP_AFTER_S = 3.0       # a robot unseen this long gets the slow 2x whole-frame search...
DEEP_PERIOD_S = 3.0      # ...at most this often (background)
BALL_GLOBAL_EVERY = 15   # frames between whole-field ball checks while tracking it
JPEG_WANTED_S = 3.0      # draw /frame.jpg only while it was fetched this recently


def _fieldcam():
    if str(OP) not in sys.path:
        sys.path.insert(0, str(OP))
    import cv2
    import numpy as np
    return cv2, np


# ---------------------------------------------------------------------------------------
# frame sources: read() -> (frame, t_receipt_monotonic) or (None, None)
# ---------------------------------------------------------------------------------------
class LiveSource:
    latency = True

    def __init__(self, source, width, height):
        from fieldcam.capture import FrameGrabber
        src = int(source) if str(source).isdigit() else source
        self.g = FrameGrabber(src, width, height, 30, wait=20, say=lambda m: print("[eye]", m, flush=True))
        self.width, self.height, self.n = self.g.width, self.g.height, -1

    def read(self):
        frame, receipt, self.n = self.g.read(self.n)
        if frame is None:
            return None, None
        return frame, time.monotonic() - (time.time() - receipt)

    def close(self):
        self.g.close()


class VideoSource:
    """A recorded video, paced at its own frame rate, looping at the end."""
    latency = False

    def __init__(self, path):
        cv2, _ = _fieldcam()
        self.cv2, self.cap = cv2, cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open video {path}")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.next = time.monotonic()

    def read(self):
        time.sleep(max(0.0, self.next - time.monotonic()))
        self.next = max(self.next + 1 / self.fps, time.monotonic() - 0.5)
        ok, frame = self.cap.read()
        if not ok:
            self.cap.set(self.cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return (frame, time.monotonic()) if ok else (None, None)

    def close(self):
        self.cap.release()


class FolderSource:
    """Still images (sorted by name), cycled at `fps`."""
    latency = False

    def __init__(self, path, fps=5.0):
        cv2, _ = _fieldcam()
        self.files = sorted(p for p in Path(path).iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
        if not self.files:
            raise RuntimeError(f"no images in {path}")
        self.cv2, self.fps, self.i, self.next = cv2, fps, 0, time.monotonic()
        first = cv2.imread(str(self.files[0]))
        self.height, self.width = first.shape[:2]

    def read(self):
        time.sleep(max(0.0, self.next - time.monotonic()))
        self.next = time.monotonic() + 1 / self.fps
        frame = self.cv2.imread(str(self.files[self.i % len(self.files)]))
        self.i += 1
        return frame, time.monotonic()

    def close(self):
        pass


class SyntheticSource:
    """fieldcam's renderer: corner tags 0-3 on the fc field corners, robots driving known paths.
    `truth` holds the exact poses of the last frame for checking the Eye."""
    latency = False
    HFOV = 64.0

    def __init__(self, cfg: dict, field: Field, robots: list[RobotCfg], width=1920, height=1080):
        import copy
        cv2, np = _fieldcam()
        from fieldcam.camera_model import default_intrinsics
        from fieldcam.sim import SimArena
        c = copy.deepcopy(cfg)
        L, W = field.length / 2, field.width / 2
        c["tags"]["field"] = {0: np.array([-L, -W]), 1: np.array([L, -W]), 2: np.array([L, W]), 3: np.array([-L, W])}
        c["ball"]["sim_bgr"] = (180, 100, 250)  # pink
        c.pop("field_setup", None)
        K, _ = default_intrinsics(width, height, self.HFOV)
        self.arena = SimArena(c, K, cam_pos=(0.0, -0.25, 3.8), size=(width, height), noise_sigma=2.0)
        self.robots, self.width, self.height, self.t0 = robots, width, height, time.monotonic()
        self.truth: dict = {}
        self.next = time.monotonic()

    def poses(self, t: float) -> tuple[dict, tuple]:
        out = {}
        for i, r in enumerate(self.robots):
            if i == 0:   # circle, facing along the path
                a = 0.4 * t
                out[r.id] = (-0.45 + 0.45 * math.cos(a), 0.45 * math.sin(a), wrap(a + math.pi / 2))
            else:        # slide along y near the east goal, facing west
                out[r.id] = (0.75, 0.5 * math.sin(0.5 * t), math.pi)
        ball = (0.3 * math.cos(0.3 * t), 0.25 * math.sin(0.6 * t))
        return out, ball

    def read(self):
        time.sleep(max(0.0, self.next - time.monotonic()))
        self.next = time.monotonic() + 1 / 15
        t = time.monotonic()
        robots, ball = self.poses(t - self.t0)
        frame = self.arena.render(robots, ball)
        self.truth = {"t": t, "robots": robots, "ball": ball}
        return frame, t

    def close(self):
        pass


# ---------------------------------------------------------------------------------------
class Eye:
    def __init__(self, field: Field, robots: list[RobotCfg], params: dict, source="phone",
                 width: int = 1920, height: int = 1080):
        cv2, np = _fieldcam()
        from fieldcam.config import load_config, set_field_size
        self.cv2, self.np = cv2, np
        self.field, self.robots, self.params = field, robots, params
        self.source_name = str(source)
        cfg = load_config(None)
        self.synthetic = self.source_name == "synthetic"
        cfg["ball"]["radius"] = field.ball_radius
        cfg["field"]["goal_width"] = field.goal_width
        cfg.pop("field_setup", None)
        set_field_size(cfg, field.length, field.width)
        mount = self.mount()
        cfg["tags"]["floor_size"] = mount["floor_size"]
        cfg["robots"] = {r.id: {
            "tag_id": r.tag, "attacks": "east" if r.team == "blue" else "west",
            "tag_height": mount["tag_height"], "tag_offset": np.array([mount["tag_offset_x"], 0.0]),
            "yaw_offset": mount["yaw_offset"],
        } for r in robots}
        self.cfg = cfg
        self._lock = threading.Lock()
        self._world: World | None = None
        self._jpeg: bytes | None = None
        self._stop = False
        self.src = None
        self.perc = None
        self.hfov: float | None = None
        self.frames, self.fps, self.message = 0, 0.0, "starting"
        self._last: dict[str, RobotObs] = {}
        self._balls: list[tuple[float, float, float]] = []
        self._next_cal = self._next_jpg = 0.0
        self._jpeg_wanted = -1e9
        # tracking state (hot thread) and what the background scan hands over (under _seed_lock)
        self._seed_lock = threading.Lock()
        self._seeds: dict[int, tuple[float, float, float]] = {}   # tag id -> (u, v, t_recv seen)
        self._scan_floor: dict = {}                               # floor tags from the last scan
        self._scan_cv = threading.Condition()
        self._scan_slot: tuple | None = None                      # (frame, n, t_recv) newest frame
        self._in_cal = False
        self._t_frame = 0.0
        self._ball_seed: tuple[float, float] | None = None
        self._ball_n = 0
        self._tm: dict[str, float] = {}                           # this frame's stage seconds
        self._seen: dict = {}
        self.stage_ms: dict[str, float] = {}                      # smoothed per-stage ms
        self.trace: collections.deque = collections.deque(maxlen=900)  # per-frame timing records
        self.src_size = (width, height)
        self._frame, self._frame_n = None, 0          # latest raw frame (recorder, Play view)
        self._view: tuple | None = None              # (frame_n, width, jpeg) cache for view_jpeg
        self._overlay: tuple | None = None           # (cam id, tag height, homographies) cache

    # ---- setup -------------------------------------------------------------------
    def mount(self) -> dict:
        """Tag mount knobs, read from the live params dict on every frame so UI edits apply at
        once. The synthetic renderer has its own fixed geometry."""
        if self.synthetic:
            return {"floor_size": 0.15, "tag_height": 0.18, "tag_offset_x": 0.0, "yaw_offset": 0.0}
        p = self.params
        return {"floor_size": float(p.get("eye_floor_size", 0.15875)), "tag_height": float(p.get("eye_tag_height", 0.24)),
                "tag_offset_x": float(p.get("eye_tag_offset_x", 0.065)),
                "yaw_offset": math.radians(float(p.get("eye_yaw_offset_deg", 0.0)))}

    def _apply_floor_size(self, size: float) -> None:
        """A changed corner-tag size changes the whole calibration: recalibrate."""
        if self.perc is not None and abs(self.cfg["tags"]["floor_size"] - size) > 1e-6:
            self.cfg["tags"]["floor_size"] = size   # perc.cfg is this same dict
            self.perc.invalidate_calibration("eye_floor_size changed")
            self._next_cal = 0.0

    def _open_source(self):
        s, (w, h) = self.source_name, self.src_size
        if s == "synthetic":
            return SyntheticSource(self.cfg, self.field, self.robots, w, h)
        p = Path(s)
        if p.is_dir():
            return FolderSource(p)
        if p.is_file():
            return VideoSource(p)
        return LiveSource(s, w, h)

    def _setup(self) -> None:
        import fieldcam.pipeline as pipeline
        from fieldcam.pipeline import Perception
        self.src = self._open_source()
        pipeline.CALIBRATION_DRIFT_LIMIT_PX = 8.0 * self.src.width / 896.0
        self.perc = Perception(self.cfg, self.src.width, self.src.height)
        self._install_tracking()
        self.message = f"{self.src.width}x{self.src.height}, calibrating from corner tags 0-3"

    # ---- tracking: robot tags and ball looked for near where they were -----------------
    def _install_tracking(self) -> None:
        """Wrap the pipeline's tag and ball detectors (the fieldcam code is not edited).

        Robot tags are small (~26x10 px at the far side at 1920 px); Tej's relaxed settings find
        them where the standard detector misses. The relaxed detector is slow on carpet (1000+
        candidate quads even in a 240 px crop), so it only runs in a small crop at 2x with fewer
        threshold passes (5..45 px step 8: same hits on the recorded runs, ~1/2 the cost)."""
        cv2 = self.cv2
        from fieldcam.detect import TAG_DICT, TagDetector

        def relaxed(min_perimeter=0.04):
            par = cv2.aruco.DetectorParameters()
            par.adaptiveThreshWinSizeMin, par.adaptiveThreshWinSizeMax, par.adaptiveThreshWinSizeStep = 5, 45, 8
            par.minMarkerPerimeterRate, par.polygonalApproxAccuracyRate, par.errorCorrectionRate = min_perimeter, 0.08, 0.9
            par.perspectiveRemoveIgnoredMarginPerCell, par.perspectiveRemovePixelPerCell = 0.2, 8
            par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            return cv2.aruco.ArucoDetector(TAG_DICT, par)

        def standard(min_perimeter=0.06):
            par = cv2.aruco.DetectorParameters()
            par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            par.adaptiveThreshWinSizeMax, par.minMarkerPerimeterRate = 45, min_perimeter
            return cv2.aruco.ArucoDetector(TAG_DICT, par)

        self._robot_tags = {r.tag for r in self.robots}
        self._px = self.src.width / 1920.0
        self._base_detect = self.perc.tag_det.detect          # full frame (+0.7x pass), hot thread only
        self._roi_det = (standard(), relaxed())               # hot thread only
        self._scan_det = TagDetector(self.perc.tag_det.expected)   # scan thread only
        self._deep_det = (standard(0.01), relaxed(0.02))           # scan thread only (Tej's sizes)
        self.perc.tag_det.detect = self._detect_tags
        self._ball_full = self.perc.ball_det.detect
        self.perc.ball_det.detect = self._detect_ball

    def _tracking(self) -> bool:
        return bool(self.perc is not None and self.perc.calibration_valid and not self._in_cal)

    def _add(self, stage: str, t0: float) -> None:
        self._tm[stage] = self._tm.get(stage, 0.0) + time.perf_counter() - t0

    @staticmethod
    def _find(det, img, want: set[int]) -> dict:
        corners, ids, _ = det.detectMarkers(img)
        return {} if ids is None else {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.ravel()) if int(i) in want}

    def _look(self, gray, centre, half: float, want: set[int], dets, scales=(1, 2)) -> dict:
        """Tags in `want` inside a square crop around `centre`, tried at each scale with each
        detector until all are found. Corners in full-frame pixels."""
        cv2, np = self.cv2, self.np
        h, w = gray.shape
        x0, y0 = max(0, int(centre[0] - half)), max(0, int(centre[1] - half))
        x1, y1 = min(w, int(centre[0] + half)), min(h, int(centre[1] + half))
        found: dict = {}
        if x1 - x0 < 16 or y1 - y0 < 16:
            return found
        crop = gray[y0:y1, x0:x1]
        for s in scales:
            img = crop if s == 1 else cv2.resize(crop, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
            for det in (dets[:1] if s == 1 else dets):
                for tid, c in self._find(det, img, want - found.keys()).items():
                    found[tid] = ((c + 0.5) / s - 0.5 + [x0, y0]).astype(np.float32)
                if not want - found.keys():
                    return found
        return found

    def _detect_tags(self, gray) -> dict:
        """perc.tag_det.detect replacement. Calibrating / not calibrated: the full-frame detector
        (calibration needs every corner tag). Calibrated: robot tags from crops around their last
        sightings, plus the floor tags from the latest background scan (for the drift check)."""
        t0 = time.perf_counter()
        t = self._t_frame
        try:
            if not self._tracking():
                tags = self._base_detect(gray)
            else:
                with self._seed_lock:
                    tags, self._scan_floor = dict(self._scan_floor), {}
                    seeds = dict(self._seeds)
                for tid in self._robot_tags:
                    s = seeds.get(tid)
                    if s is None or t - s[2] > SEED_KEEP_S:
                        continue
                    half = min(ROI_HALF_PX + ROI_GROW_PX_S * max(0.0, t - s[2]), ROI_MAX_HALF_PX) * self._px
                    tags.update(self._look(gray, s[:2], half, {tid}, self._roi_det))
            self._note_seeds(tags, t)
            return tags
        finally:
            self._add("tags", t0)

    def _note_seeds(self, tags: dict, t: float) -> None:
        with self._seed_lock:
            for tid in self._robot_tags & tags.keys():
                old = self._seeds.get(tid)
                if old is None or old[2] <= t:
                    c = tags[tid].mean(axis=0)
                    self._seeds[tid] = (float(c[0]), float(c[1]), t)

    def _scan_loop(self) -> None:
        """Background: full-frame tag detection on the newest frame ~4x a second (floor tags for
        the drift check; robots that the crops lost), and the slow 2x whole-frame search for a
        robot not seen for a while. OpenCV releases the GIL, so this runs beside the hot thread."""
        cv2 = self.cv2
        last_n, next_deep = -1, 0.0
        while not self._stop:
            with self._scan_cv:
                self._scan_cv.wait_for(lambda: self._stop or (self._scan_slot is not None and self._scan_slot[1] != last_n),
                                       timeout=1.0)
                slot = self._scan_slot
            if self._stop or slot is None or slot[1] == last_n:
                continue
            frame, last_n, t_recv = slot
            if not self._tracking():   # the hot thread runs the full detector itself then
                time.sleep(0.1)
                continue
            t0 = time.perf_counter()
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                tags = self._scan_det.detect(gray)
                with self._seed_lock:
                    seen = {tid: s[2] for tid, s in self._seeds.items()}
                lost = {tid for tid in self._robot_tags - tags.keys() if t_recv - seen.get(tid, -1e9) > DEEP_AFTER_S}
                now = time.monotonic()
                if lost and now >= next_deep:
                    next_deep = now + DEEP_PERIOD_S
                    t1 = time.perf_counter()
                    h, w = gray.shape
                    tags.update(self._look(gray, (w / 2, h / 2), max(w, h), lost, self._deep_det, scales=(2,)))
                    self._scan_ms("deep", t1)
                floor = {k: v for k, v in tags.items() if k not in self._robot_tags}
                with self._seed_lock:
                    self._scan_floor = floor
                self._note_seeds(tags, t_recv)
            except Exception as e:  # noqa: BLE001 - the scan must never stop perception
                self.message = f"scan error: {type(e).__name__}: {e}"
            self._scan_ms("scan", t0)
            time.sleep(max(0.0, SCAN_PERIOD_S - (time.perf_counter() - t0)))

    def _scan_ms(self, stage: str, t0: float) -> None:
        ms = (time.perf_counter() - t0) * 1000
        self.stage_ms[stage] = round(ms if stage not in self.stage_ms else 0.8 * self.stage_ms[stage] + 0.2 * ms, 2)

    def _detect_ball(self, bgr, roi_mask=None, expected_radius_px=None, radius_at=None):
        """perc.ball_det.detect replacement: the full-resolution detector on a crop around the
        last ball position; a half-resolution whole-field search to (re)find it, and every
        BALL_GLOBAL_EVERY frames to make sure the crop is not following something else (the
        whole-field answer wins if it disagrees, as the plain detector's would)."""
        t0 = time.perf_counter()
        try:
            if roi_mask is None or not expected_radius_px or radius_at is None:
                return self._ball_full(bgr, roi_mask, expected_radius_px, radius_at)
            r = float(expected_radius_px)
            self._ball_n += 1
            seed = self._ball_seed
            det = self._ball_crop(bgr, roi_mask, r, radius_at, seed) if seed is not None else None
            if det is None or self._ball_n % BALL_GLOBAL_EVERY == 0:
                g = self._ball_locate(bgr, roi_mask, r, radius_at)
                if g is not None and (det is None or math.hypot(g[0] - det[0], g[1] - det[1]) > 2 * r):
                    det = self._ball_crop(bgr, roi_mask, r, radius_at, g) or g
            self._ball_seed = None if det is None else (det[0], det[1])
            return det
        finally:
            self._add("ball", t0)

    def _ball_crop(self, bgr, roi_mask, r, radius_at, centre):
        half = max(4.0 * r, 120.0 * self._px)
        for _ in range(2):   # re-centre once if the ball sits near the crop edge
            h, w = bgr.shape[:2]
            x0, y0 = max(0, int(centre[0] - half)), max(0, int(centre[1] - half))
            x1, y1 = min(w, int(centre[0] + half)), min(h, int(centre[1] + half))
            if x1 - x0 < 16 or y1 - y0 < 16:
                return None
            det = self._ball_full(bgr[y0:y1, x0:x1], roi_mask[y0:y1, x0:x1], r,
                                  lambda u, v: radius_at(u + x0, v + y0))
            if det is None:
                return None
            det = (det[0] + x0, det[1] + y0, det[2])
            if math.hypot(det[0] - centre[0], det[1] - centre[1]) < half - 1.6 * r:
                return det
            centre = det
        return det

    def _ball_locate(self, bgr, roi_mask, r, radius_at):
        cv2, k = self.cv2, 0.5
        small = cv2.resize(bgr, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        key = id(roi_mask)
        if getattr(self, "_roi_small", (None,))[0] != key:
            self._roi_small = (key, cv2.resize(roi_mask, (small.shape[1], small.shape[0]), interpolation=cv2.INTER_NEAREST))

        def radius_small(u, v):
            px = radius_at(u / k, v / k)
            return px * k if px else None
        det = self._ball_full(small, self._roi_small[1], r * k, radius_small)
        return None if det is None else tuple(v / k for v in det)

    def _calibrate(self, frame) -> bool:
        """Corner-tag calibration; with no intrinsics file, fit the lens angle first (Tej)."""
        import fieldcam.pipeline as pipeline
        from fieldcam.camera_model import default_intrinsics
        perc, cv2 = self.perc, self.cv2
        if perc.has_calibrated_intrinsics or self.hfov is not None:
            if perc.calibrate_from_corners(frame):
                return True
            if perc.has_calibrated_intrinsics or "does not fit" not in (perc.calib_fail or ""):
                return False
        tags = perc.tag_det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        missing = [t for t in perc.cfg["tags"]["corner_ids"].values() if t not in tags]
        if missing:
            perc.calib_fail = f"corner tags not completely in view: {missing}"
            return False
        orig, limit, best = perc.tag_det.detect, pipeline.CALIBRATION_DRIFT_LIMIT_PX, None
        perc.tag_det.detect = lambda gray: dict(tags)  # one detection for the whole sweep
        pipeline.CALIBRATION_DRIFT_LIMIT_PX = 1e9
        w, h = self.src.width, self.src.height
        try:
            for angles in ([float(a) for a in range(30, 121, 3)], None):
                for hfov in angles or [best[1] + 0.5 * k for k in range(-5, 6)]:
                    perc.K, perc.dist = default_intrinsics(w, h, hfov)
                    if perc.calibrate_from_corners(frame) and (best is None or perc.cam.reproj_err_px < best[0]):
                        best = (perc.cam.reproj_err_px, hfov)
                if best is None:
                    break
        finally:
            perc.tag_det.detect = orig
            pipeline.CALIBRATION_DRIFT_LIMIT_PX = limit
        perc.invalidate_calibration("lens angle fit")
        if best is None:
            return False
        self.hfov = best[1]
        perc.K, perc.dist = default_intrinsics(w, h, self.hfov)
        perc.tag_det.detect = lambda gray: dict(tags)
        try:
            return perc.calibrate_from_corners(frame)
        finally:
            perc.tag_det.detect = orig

    def _widen_ball_area(self) -> None:
        """Look for the ball up to 0.55 m behind the goal lines, so a ball in a goal is measured."""
        np, cv2, perc = self.np, self.cv2, self.perc
        corners = np.array(perc.cfg["field"]["corners"], float)
        grown = corners + np.sign(corners) * [0.55, 0.25]
        poly = perc.cam.project(np.hstack([grown, np.zeros((4, 1))])).astype(np.int32)
        mask = np.zeros((perc.size[1], perc.size[0]), np.uint8)
        cv2.fillConvexPoly(mask, poly, 255)
        perc.roi_mask = mask
        perc.ball_roi = mask if perc.ball_scale == 1 else cv2.resize(
            mask, None, fx=perc.ball_scale, fy=perc.ball_scale, interpolation=cv2.INTER_NEAREST)

    # ---- per frame ---------------------------------------------------------------------
    def process(self, frame, t_recv: float) -> World | None:
        np, perc = self.np, self.perc
        now = time.monotonic()
        mount = self.mount()
        self._apply_floor_size(mount["floor_size"])
        t_cap = t_recv - (self.params.get("eye_latency_s", 0.12) if self.src.latency else 0.0)
        self._t_frame = t_recv
        if not perc.calibration_valid and now >= self._next_cal:
            self._next_cal = now + 0.5
            self._in_cal, t0 = True, time.perf_counter()
            try:
                calibrated = self._calibrate(frame)
            finally:
                self._in_cal = False
                self._add("calibrate", t0)
            if calibrated:
                self._widen_ball_area()
                f = perc.cfg["field"]
                self.message = (f"calibrated: field {f['length']:.2f} x {f['width']:.2f} m (config "
                                f"{self.field.length:.2f} x {self.field.width:.2f}), lens {self.hfov or 0:.1f} deg, "
                                f"err {perc.calib_err_px:.1f} px")
            else:
                self.message = f"not calibrated: {perc.calib_fail or perc.calibration_reason}"
        self._seen = {}
        state = perc.process(frame, t_cap, t_cap, match_only=True)
        self._state = state
        if not state.get("calibration_valid"):
            if perc.cam is not None and perc.calibration_reason:
                self.message = f"calibration lost: {perc.calibration_reason}"
            return None
        cam, tags = perc.cam, perc.debug.get("tags", {})
        for rid, rc in perc.cfg["robots"].items():
            tag = tags.get(rc["tag_id"])
            if tag is None:
                continue
            x, y, yaw = self.tag_pose(cam.pixel_to_plane(tag, mount["tag_height"]), mount)
            self._last[rid] = RobotObs(rid, Pose(x, y, yaw), t_cap)
        robots = {k: v for k, v in self._last.items() if t_cap - v.t < KEEP_ROBOT_S}
        ball = None
        px = perc.debug.get("ball_px")
        self._seen = {"ball": px is not None, **{k: v.t == t_cap for k, v in robots.items()}}
        if px is not None:
            bx, by = (float(v) for v in cam.pixel_to_plane([px[:2]], perc.cfg["ball"]["radius"])[0])
            self._balls.append((t_cap, bx, by))
        self._balls = [b for b in self._balls if t_cap - b[0] < BALL_VEL_WINDOW_S]
        if self._balls and t_cap - self._balls[-1][0] < KEEP_ROBOT_S:
            t, bx, by = self._balls[-1]
            vx = vy = 0.0
            if len(self._balls) >= 3:
                ts = np.array([b[0] for b in self._balls])
                ts = ts - ts.mean()
                d = float((ts ** 2).sum())
                if d > 1e-6:
                    vx = float((ts * np.array([b[1] for b in self._balls])).sum() / d)
                    vy = float((ts * np.array([b[2] for b in self._balls])).sum() / d)
            ball = Ball(bx, by, vx, vy, t)
        return World(now, ball, robots)

    @staticmethod
    def tag_pose(c, mount: dict) -> tuple[float, float, float]:
        """Robot body pose from the tag's floor-plane corners (TL, TR, BR, BL at tag height):
        tag top edge = nose direction + yaw_offset; body centre tag_offset_x behind the tag."""
        fwd = (c[0] + c[1]) / 2 - (c[3] + c[2]) / 2
        yaw = wrap(math.atan2(fwd[1], fwd[0]) + mount["yaw_offset"])
        ctr, ox = c.mean(axis=0), mount["tag_offset_x"]
        return float(ctr[0] - math.cos(yaw) * ox), float(ctr[1] - math.sin(yaw) * ox), yaw

    def _annotate(self, frame, world: World | None) -> bytes | None:
        cv2 = self.cv2
        try:
            out = self.perc.draw(frame, self._state) if getattr(self, "_state", None) else frame.copy()
        except Exception:  # noqa: BLE001 - drawing must never stop perception
            out = frame.copy()
        cv2.putText(out, self.message[:110], (20, out.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        scale = 1280 / out.shape[1]
        if scale < 1:
            out = cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes() if ok else None

    def _run(self) -> None:
        """Hot thread: newest frame -> World, as fast as frames come. The source keeps only the
        newest frame (LiveSource's grabber thread), so a slow frame never leaves a backlog."""
        while not self._stop and self.src is None:
            try:
                self._setup()
            except Exception as e:  # noqa: BLE001 - keep trying; the UI shows why
                self.message = f"camera not open: {e}"
                time.sleep(2.0)
        threading.Thread(target=self._scan_loop, daemon=True, name="fc-eye-scan").start()
        t_prev = time.monotonic()
        while not self._stop:
            t_wait = time.perf_counter()
            frame, t_recv = self.src.read()
            if frame is None:
                self.message = "no frame (camera lost?)"
                with self._lock:
                    self._world = None
                continue
            self._tm = {"wait": time.perf_counter() - t_wait}
            t0 = time.perf_counter()
            try:
                world = self.process(frame, t_recv)
            except Exception as e:  # noqa: BLE001
                self.message, world = f"perception error: {type(e).__name__}: {e}", None
            self._tm["process"] = time.perf_counter() - t0
            now = time.monotonic()
            self.frames += 1
            self.fps = 0.9 * self.fps + 0.1 / max(now - t_prev, 1e-6)
            t_prev = now
            with self._lock:
                self._world = world
                self._frame, self._frame_n = frame, self._frame_n + 1
            with self._scan_cv:
                self._scan_slot = (frame, self._frame_n, t_recv)
                self._scan_cv.notify()
            self._tm["age"] = now - t_recv   # receipt -> World published (plus eye_latency_s before receipt)
            if now - self._jpeg_wanted < JPEG_WANTED_S and now >= self._next_jpg:
                self._next_jpg = now + 0.2
                t1 = time.perf_counter()
                jpg = self._annotate(frame, world)
                self._tm["annotate"] = time.perf_counter() - t1
                if jpg:
                    with self._lock:
                        self._jpeg = jpg
            rec = {k: round(v * 1000, 2) for k, v in self._tm.items()}
            for k, v in rec.items():
                self.stage_ms[k] = round(v if k not in self.stage_ms else 0.9 * self.stage_ms[k] + 0.1 * v, 2)
            rec["t"], rec["seen"] = now, self._seen   # what this frame measured (for eye_bench)
            self.trace.append(rec)

    # ---- public (SPEC) -------------------------------------------------------------------
    def start(self) -> "Eye":
        threading.Thread(target=self._run, daemon=True, name="fc-eye").start()
        return self

    def latest(self) -> World | None:
        with self._lock:
            return self._world

    def jpeg(self) -> bytes | None:
        """Annotated frame (~5 Hz) for the ops screen. Drawn only while it is being fetched, so
        the first fetch after a pause may return None (the page retries)."""
        self._jpeg_wanted = time.monotonic()
        with self._lock:
            return self._jpeg

    def frame(self):
        """(latest raw BGR frame, frame number) or (None, 0). Never copy-modify the frame."""
        with self._lock:
            return self._frame, self._frame_n

    def view_jpeg(self, width: int = 1280, quality: int = 75) -> bytes | None:
        """Clean (unannotated) downscaled JPEG of the latest frame for the Play screen; encoded on
        the caller's thread, once per frame."""
        frame, n = self.frame()
        if frame is None:
            return None
        v = self._view
        if v and v[0] == n and v[1] == width:
            return v[2]
        cv2 = self.cv2
        scale = width / frame.shape[1]
        out = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else frame
        ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None
        self._view = (n, width, buf.tobytes())
        return self._view[2]

    def overlay(self) -> dict | None:
        """Homographies field (x, y) -> normalised image coords (0..1) for drawing on the view:
        "floor" (z = 0) and "tag" (z = robot tag height). Fitted to the calibrated camera model
        (lens distortion included as well as a plane map allows). None until calibrated."""
        perc = self.perc
        if perc is None or not perc.calibration_valid or perc.cam is None or self.src is None:
            return None
        th = self.mount()["tag_height"]
        key = (id(perc.cam), round(th, 4))
        if self._overlay and self._overlay[0] == key:
            return self._overlay[1]
        np, cv2, f = self.np, self.cv2, self.field
        gx, gy = np.meshgrid(np.linspace(-f.length / 2 - 0.3, f.length / 2 + 0.3, 9),
                             np.linspace(-f.width / 2 - 0.2, f.width / 2 + 0.2, 7))
        pts = np.stack([gx.ravel(), gy.ravel()], axis=1)
        out = {}
        for name, z in (("floor", 0.0), ("tag", th)):
            px = perc.cam.project(np.hstack([pts, np.full((len(pts), 1), z)])).reshape(-1, 2)
            px = px / [self.src.width, self.src.height]
            H, _ = cv2.findHomography(pts.astype(np.float64), px.astype(np.float64))
            if H is None:
                return None
            out[name] = [[round(float(v), 9) for v in row] for row in H]
        out["aspect"] = round(self.src.width / self.src.height, 4)
        self._overlay = (key, out)
        return out

    def status(self) -> dict:
        w = self.latest()
        now = time.monotonic()
        return {"source": self.source_name, "message": self.message, "frames": self.frames,
                "fps": round(self.fps, 1), "calibrated": bool(self.perc and self.perc.calibration_valid),
                "hfov_deg": self.hfov, "calib_err_px": self.perc.calib_err_px if self.perc else None,
                "mount": self.mount(),
                "robots_age": {k: round(now - v.t, 3) for k, v in (w.robots.items() if w else [])},
                "ball_age": round(now - w.ball.t, 3) if w and w.ball else None,
                "stage_ms": dict(self.stage_ms)}

    def close(self) -> None:
        self._stop = True
        with self._scan_cv:
            self._scan_cv.notify_all()
        if self.src:
            self.src.close()


def main(argv=None) -> None:
    """Offline check: run the Eye on a source and print what it sees (and the error vs truth
    for the synthetic source)."""
    import argparse
    from .app import PARAM_FALLBACKS, load_robot_config
    from .core import load_field, load_params
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="synthetic", help="synthetic | video file | image folder | camera name")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--save", default=None, help="write the last annotated frame here (jpg)")
    a = ap.parse_args(argv)
    eye = Eye(load_field(), load_robot_config()[0], {**PARAM_FALLBACKS, **load_params()}, source=a.source).start()
    t_end, errs = time.monotonic() + a.seconds, []
    while time.monotonic() < t_end:
        time.sleep(0.5)
        if a.save:
            eye.jpeg()   # keeps the annotated frame being drawn
        w = eye.latest()
        line = f"[{eye.fps:4.1f} fps] {eye.message}"
        if w:
            line += " | " + " ".join(f"{k}=({o.pose.x:+.3f},{o.pose.y:+.3f},{math.degrees(o.pose.yaw):+.0f})"
                                     for k, o in w.robots.items())
            line += f" ball=({w.ball.x:+.3f},{w.ball.y:+.3f})" if w.ball else " ball=-"
            tr = getattr(eye.src, "truth", None)
            if tr and w.robots:
                for k, o in w.robots.items():
                    if k in tr["robots"]:
                        # truth at the observation's capture time
                        rp, _ = eye.src.poses(o.t - eye.src.t0)
                        x, y, yaw = rp[k]
                        errs.append((math.hypot(o.pose.x - x, o.pose.y - y), abs(math.degrees(wrap(o.pose.yaw - yaw)))))
        print(line, flush=True)
    if errs:
        pe, ye = sorted(e[0] for e in errs), sorted(e[1] for e in errs)
        print(f"synthetic truth check: {len(errs)} robot obs, median pos err {pe[len(pe)//2]*100:.1f} cm "
              f"(max {pe[-1]*100:.1f}), median yaw err {ye[len(ye)//2]:.1f} deg (max {ye[-1]:.1f})")
    if a.save and eye.jpeg():
        Path(a.save).write_bytes(eye.jpeg())
        print("saved", a.save)
    eye.close()


if __name__ == "__main__":
    main()
