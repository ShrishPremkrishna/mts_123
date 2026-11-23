"""Overhead camera -> World. Thin adapter over overhead_perception/fieldcam (not edited).

Ported from Tej's overhead_kick.py Eye (origin/tej/overhead-kick, 2026-11-20):
  * lens-angle fit from the four corner tags when there is no intrinsics file
    (fixed 69 deg gave 9-14 px fit error; the fit gave 0.5-1.0 px),
  * calibration drift limit scaled to the picture width (8 px is for 896 px),
  * a second, relaxed tag detector in an enlarged crop for the small robot tags,
  * ball search area widened 0.55 m behind the goal lines,
  * raw per-frame detections (not the pipeline's lagging Kalman output).

Sources: a live camera name/index ("phone"), a video file, a folder of images, or
"synthetic" (fieldcam's renderer with moving robots; truth kept for checking).

    python -m fc.eye --source synthetic --seconds 5       # offline self-check
"""
from __future__ import annotations

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
        self.arena = SimArena(c, K, cam_pos=(0.0, -0.25, 3.2), size=(width, height), noise_sigma=2.0)
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
        synthetic = self.source_name == "synthetic"
        cfg["tags"]["floor_size"] = 0.15 if synthetic else params.get("eye_floor_size", 0.15875)
        cfg["ball"]["radius"] = field.ball_radius
        cfg["field"]["goal_width"] = field.goal_width
        cfg.pop("field_setup", None)
        set_field_size(cfg, field.length, field.width)
        cfg["robots"] = {r.id: {
            "tag_id": r.tag, "attacks": "east" if r.team == "blue" else "west",
            "tag_height": 0.18 if synthetic else params.get("eye_tag_height", 0.24),
            "tag_offset": np.array([0.0 if synthetic else params.get("eye_tag_offset_x", 0.065), 0.0]),
            "yaw_offset": 0.0 if synthetic else math.radians(params.get("eye_yaw_offset_deg", 0.0)),
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
        self._zoom_n = 0
        self.src_size = (width, height)

    # ---- setup -------------------------------------------------------------------
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
        self._install_robot_zoom([r.tag for r in self.robots])
        self.message = f"{self.src.width}x{self.src.height}, calibrating from corner tags 0-3"

    def _install_robot_zoom(self, tag_ids: list[int], half: int = 280) -> None:
        """Robot tags are small (~26x10 px at the far side). If the standard detector misses one,
        look again with relaxed settings in an enlarged crop near its last position, and every
        10th frame in the whole frame at 2x. (Tej's settings, generalised to several robots.)"""
        cv2, np = self.cv2, self.np
        from fieldcam.detect import TAG_DICT, TagDetector
        base, want_all = self.perc.tag_det.detect, set(tag_ids)
        par = cv2.aruco.DetectorParameters()
        par.adaptiveThreshWinSizeMin, par.adaptiveThreshWinSizeMax, par.adaptiveThreshWinSizeStep = 5, 65, 6
        par.minMarkerPerimeterRate, par.polygonalApproxAccuracyRate, par.errorCorrectionRate = 0.02, 0.08, 0.9
        par.perspectiveRemoveIgnoredMarginPerCell, par.perspectiveRemovePixelPerCell = 0.2, 8
        par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        relaxed, standard = cv2.aruco.ArucoDetector(TAG_DICT, par), TagDetector(())
        last: dict[int, np.ndarray] = {}

        def zoom(gray, want):
            tags = {k: v for k, v in standard.detect(gray).items() if k in want}
            if want - tags.keys():
                corners, ids, _ = relaxed.detectMarkers(gray)
                for c, i in zip(corners, [] if ids is None else ids.ravel()):
                    if int(i) in want and int(i) not in tags:
                        tags[int(i)] = c.reshape(4, 2)
            return tags

        def look(gray, box, scale, want, tags):
            x0, y0, x1, y1 = box
            big = cv2.resize(gray[y0:y1, x0:x1], None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            for tid, c in zoom(big, want).items():
                tags[tid] = ((c + 0.5) / scale - 0.5 + [x0, y0]).astype(np.float32)

        def detect(gray):
            tags = base(gray)
            h, w = gray.shape
            for tid in want_all - tags.keys():
                if tid in last:
                    cx, cy = last[tid]
                    box = (max(0, int(cx - half)), max(0, int(cy - half)), min(w, int(cx + half)), min(h, int(cy + half)))
                    for s in (2, 4, 3):
                        look(gray, box, s, {tid}, tags)
                        if tid in tags:
                            break
            missing = want_all - tags.keys()
            self._zoom_n += 1
            if missing and self._zoom_n % 10 == 0:
                look(gray, (0, 0, w, h), 2, missing, tags)
            for tid in want_all & tags.keys():
                last[tid] = tags[tid].mean(axis=0)
            return tags

        self.perc.tag_det.detect = detect

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
        t_cap = t_recv - (self.params.get("eye_latency_s", 0.12) if self.src.latency else 0.0)
        if not perc.calibration_valid and now >= self._next_cal:
            self._next_cal = now + 0.5
            if self._calibrate(frame):
                self._widen_ball_area()
                f = perc.cfg["field"]
                self.message = (f"calibrated: field {f['length']:.2f} x {f['width']:.2f} m (config "
                                f"{self.field.length:.2f} x {self.field.width:.2f}), lens {self.hfov or 0:.1f} deg, "
                                f"err {perc.calib_err_px:.1f} px")
            else:
                self.message = f"not calibrated: {perc.calib_fail or perc.calibration_reason}"
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
            c = cam.pixel_to_plane(tag, rc["tag_height"])  # TL, TR, BR, BL
            fwd = (c[0] + c[1]) / 2 - (c[3] + c[2]) / 2
            yaw = wrap(math.atan2(fwd[1], fwd[0]) + rc["yaw_offset"])
            cy, sy = math.cos(yaw), math.sin(yaw)
            ctr, (ox, oy) = c.mean(axis=0), rc["tag_offset"]
            x, y = ctr[0] - (cy * ox - sy * oy), ctr[1] - (sy * ox + cy * oy)
            self._last[rid] = RobotObs(rid, Pose(float(x), float(y), yaw), t_cap)
        robots = {k: v for k, v in self._last.items() if t_cap - v.t < KEEP_ROBOT_S}
        ball = None
        px = perc.debug.get("ball_px")
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
        while not self._stop and self.src is None:
            try:
                self._setup()
            except Exception as e:  # noqa: BLE001 - keep trying; the UI shows why
                self.message = f"camera not open: {e}"
                time.sleep(2.0)
        t_prev = time.monotonic()
        while not self._stop:
            frame, t_recv = self.src.read()
            if frame is None:
                self.message = "no frame (camera lost?)"
                with self._lock:
                    self._world = None
                continue
            try:
                world = self.process(frame, t_recv)
            except Exception as e:  # noqa: BLE001
                self.message, world = f"perception error: {type(e).__name__}: {e}", None
            now = time.monotonic()
            self.frames += 1
            self.fps = 0.9 * self.fps + 0.1 / max(now - t_prev, 1e-6)
            t_prev = now
            jpg = None
            if now >= self._next_jpg:
                self._next_jpg = now + 0.2
                jpg = self._annotate(frame, world)
            with self._lock:
                self._world = world
                if jpg:
                    self._jpeg = jpg

    # ---- public (SPEC) -------------------------------------------------------------------
    def start(self) -> "Eye":
        threading.Thread(target=self._run, daemon=True, name="fc-eye").start()
        return self

    def latest(self) -> World | None:
        with self._lock:
            return self._world

    def jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def status(self) -> dict:
        w = self.latest()
        now = time.monotonic()
        return {"source": self.source_name, "message": self.message, "frames": self.frames,
                "fps": round(self.fps, 1), "calibrated": bool(self.perc and self.perc.calibration_valid),
                "hfov_deg": self.hfov, "calib_err_px": self.perc.calib_err_px if self.perc else None,
                "robots_age": {k: round(now - v.t, 3) for k, v in (w.robots.items() if w else [])},
                "ball_age": round(now - w.ball.t, 3) if w and w.ball else None}

    def close(self) -> None:
        self._stop = True
        if self.src:
            self.src.close()


def main(argv=None) -> None:
    """Offline check: run the Eye on a source and print what it sees (and the error vs truth
    for the synthetic source)."""
    import argparse
    from .core import load_field, load_params, load_robots
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="synthetic", help="synthetic | video file | image folder | camera name")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--save", default=None, help="write the last annotated frame here (jpg)")
    a = ap.parse_args(argv)
    eye = Eye(load_field(), load_robots(), load_params(), source=a.source).start()
    t_end, errs = time.monotonic() + a.seconds, []
    while time.monotonic() < t_end:
        time.sleep(0.5)
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
