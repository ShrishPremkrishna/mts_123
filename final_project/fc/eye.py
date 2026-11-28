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

Robot board blob (2026-11-25): when the robot tag is not decoded (motion blur in the fast gait), its
coloured tag board is tracked instead (RobotObs.src "blob", heading carried; see BLOB_* below).
`python -m fc.tools.robot_track_replay RUN` measures coverage and errors on a recorded run.

Field calibration (Nov 28; replaces fieldcam's all-or-nothing drift check). fieldcam dropped the whole
calibration when ANY floor tag showed > 8 px off, and the corner tags sit on a flexible fence: one bumped tag
= no World until all four corner tags decoded in one frame again (warm-up 071320Z: 4922/4924 ticks without a
World), and each recovery re-fitted the lens (53-68.5 deg across restarts, field 3.38 -> 3.21 m mid-match).
Now:
  * a full fit (lens + field size + camera pose, all four corner tags) happens once, on an empty field: at the
    first start, or when the operator asks (recalibrate(), app idle only). It is accepted only below
    eye_cal_max_px (3 px RMS at 1920 px) and saved to config/eye_cal.json (per site, not in git);
  * a restart loads eye_cal.json: poses at once, no tags needed ("unverified" until floor tags confirm it);
  * the eye monitors each floor tag against where it showed at calibration (background scan, ~4 Hz). One or
    two tags off while >= 2 others hold = "tag N moved": that tag is ignored, play goes on (re-measured where
    it now sits once steady for READOPT_S). >= 3 tags off for > MOVED_S = the camera moved: the camera pose
    alone is re-solved from the tags in view (lens and field frozen), accepted below eye_pose_max_px (3 px);
  * until then the last pose keeps serving with cal state "suspect" (status()["cal"], message "CAL SUSPECT ...").
    The field size never changes without the operator.

Sources: a live camera name/index ("phone"), a video file, a folder of images, or
"synthetic" (fieldcam's renderer with moving robots; truth kept for checking).

    python -m fc.eye --source synthetic --seconds 5       # offline self-check
    python -m fc.eye --source RUN/overhead.mp4 --fast     # replay a recording frame by frame (calibration events)
"""
from __future__ import annotations

import atexit
import collections
import dataclasses
import datetime
import json
import math
import os
import select
import subprocess
import sys
import threading
import time
from itertools import combinations
from pathlib import Path

from . import ballcal
from .ballvel import BallVelocity, fit_decel  # noqa: F401 - fit_decel re-exported
from .core import CONFIG, ROBOTS_DIR, Ball, Field, Pose, RobotCfg, RobotObs, World, load_robot_overrides, wrap

OP = Path(__file__).resolve().parent.parent / "overhead_perception"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
KEEP_ROBOT_S = 1.0   # keep reporting a robot's last raw pose (with its old time) this long
BALL_TRACK_S = 60.0  # raw ball sightings kept for `ctl rollcal` (GET /ball_track)
FROZEN_DIFF = 0.05   # mean |grey diff| on a 1/8 subsample below this = the same picture again (live noise ~1)
FROZEN_S = 0.5       # same picture (or no frame) this long = CAMERA PAUSED
# Pixel sizes below are for a 1920 px wide picture and scale with the width.
ROI_HALF_PX = 110        # robot-tag crop half size around the last sighting
ROI_GROW_PX_S = 340      # ...grown by this per second since the last sighting (~0.6 m/s)
ROI_MAX_HALF_PX = 260
SEED_KEEP_S = 1.5        # look around a robot's last sighting this long; then the scan finds it.
# Was 0.5: the 4 Hz full-frame scan finds the small (~30 px) robot tag in only 18-38% of tries,
# so any 0.5 s miss became a 1.2-1.5 s gap (= pose_stale_s stop). Replay of the Nov 24 fast runs
# 074127 / 073726: robot seen 32% -> 56% / 43% -> 63% of frames; hot path +7 ms mean.
SCAN_PERIOD_S = 0.25     # background full-frame tag scan (floor-tag drift check, lost robots)
DEEP_AFTER_S = 3.0       # a robot unseen this long gets the slow 2x whole-frame search...
DEEP_PERIOD_S = 3.0      # ...at most this often (background)
BALL_GLOBAL_EVERY = 15   # frames between whole-field ball checks while tracking it
JPEG_WANTED_S = 3.0      # draw /frame.jpg only while it was fetched this recently

# Robot board blob (Nov 25). In the fast gait the robot tag decoded in ~6% of frames (motion blur), so the
# brain ran on a 1 s old pose and stale-stopped. The tag sits on a board wrapped in coloured tape, and a
# colour blob survives blur. When the tag is not decoded, the board is looked for near where the robot was;
# its centre gives the position (same parallax model as the tag, at the tag height) and the heading is
# carried from the last decoded tag by the commands sent (x measured turn gain) and, when walking fast and
# straight, pulled toward the direction of travel. Params (live dict, defaults here):
#   eye_blob 1/0, eye_blob_max_s (blob-only tracking this long after the last decoded tag), eye_board_m,
#   eye_turn_gain / eye_turn_lag_s (heading change = gain x commanded wz, lag relative to capture time;
#   Nov 25 runs: 0.95 / 0.05 s fit 3042 tag pairs to 6 deg rms), eye_travel_tau_s (0 = no travel pull).
# Measured yaw rate (Nov 25): when the bridge's acks carry the IMU yaw integral (drive_bridge.YawRate ->
# Link.gyro), the heading change between tags is taken from it instead of the commands (the fast gait turns
# 13-23 deg off the command per tag gap). Used only when samples bracket the interval to within eye_gyro_fresh_s;
# the travel direction is then a weak prior (eye_travel_tau_gyro_s). A decoded tag still resets the heading.
ROBOT_COLORS_PATH = CONFIG / "robot_colors.json"
BLOB_MAX_S = 3.0          # blob-only tracking this long after the last decoded tag
BLOB_GATE_M = 0.08        # blob centre within this of the predicted spot...
BLOB_SPEED = 1.2          # ...grown by this (m/s) per second since the last fix...
BLOB_GATE_MAX_M = 0.45    # ...up to this
BOARD_M = 0.14            # side of the visible coloured board (13 x 15 cm on blue)
BLOB_AREA = (0.35, 2.8)   # blob hull area / expected board area (blur stretches it, a broken ring shrinks it)
BLOB_ASPECT = 2.6         # longest / shortest side of the blob's rotated rectangle
CMD_HOLD_S = 0.25         # a command counts this long without a newer one (the bridge's ttl_s)
TURN_GAIN, TURN_LAG_S = 0.95, 0.05
TRAVEL_TAU_S = 0.3        # heading pulled toward the travel direction with this time constant
TRAVEL_WIN_S = 0.3        # travel direction = line fit through the fixes of this window
YAW_CONF_TAU_S = 2.0      # heading confidence decays with this time constant without a decoded tag
GYRO_FRESH_S = 0.1        # measured yaw used only if a sample lies within this of both interval ends
GYRO_GAP_S = 0.3          # ...and no gap between ack samples longer than this around an end
TRAVEL_TAU_GYRO_S = 5.0   # travel-direction pull while the measured yaw carries the heading (weak prior)
YAW_CONF_GYRO_TAU_S = 8.0  # confidence decay while the measured yaw carries the heading

# Per-robot tag mount (Nov 28). These knobs describe one robot's tag (how high, how far ahead of the body centre,
# how it is turned), so fc/config/robots/<id>.json overrides them for that robot; params.json holds the default for
# a robot without its own value. The files are re-read when they change (no restart); Eye.robot_over is live.
ROBOT_MOUNT_KEYS = ("eye_tag_height", "eye_tag_offset_x", "eye_yaw_offset_deg")
ROBOT_OVER_CHECK_S = 1.0  # how often the robots/<id>.json files are checked for edits

# Field calibration (see the module notes). Pixel sizes for a 1920 px wide picture, scaled with the width.
EYE_CAL_PATH = CONFIG / "eye_cal.json"
TAG_MOVED_PX = 6.0     # a floor tag this far from where it showed at calibration has shifted (~2 cm on the floor;
                       # corner tags held within ~3 px over 2 min on the Nov 27 recordings)
MOVED_S = 1.0          # a shift must hold this long: 1-2 tags -> "tag N moved" (ignored), >= 3 -> camera moved
POSE_MAX_PX = 3.0      # pose re-solve accepted below this RMS corner error (param eye_pose_max_px)
FIT_MAX_PX = 3.0       # full fit accepted below this (param eye_cal_max_px); was the 17 px drift limit
READOPT_S = 5.0        # an ignored tag steady this long (pose verified by >= 2 others) is re-measured where it is
READOPT_STILL_PX = 1.5  # ...steady = every sighting within this of their mean
RECENT_S = 0.6         # the monitor judges each floor tag's latest sighting this recent (a scan often decodes
                       # only 1-2 of the edge-of-frame corner tags: Nov 27 recordings)
REFIT_S = 8.0          # an operator re-fit keeps trying this long (all four corner tags in one frame)


@dataclasses.dataclass
class FieldCal:
    """One field calibration: lens (K, dist), camera pose (rvec, tvec: field -> camera), field size, the floor
    tags' poses on the field {id: (x, y, yaw)} and where each tag's centre showed in the picture (`ref`, the
    monitor's reference). Pixel quantities are for a picture of `size` (w, h)."""
    K: object
    dist: object
    rvec: object
    tvec: object
    length: float
    width: float
    floor_size: float
    tags: dict
    ref: dict
    size: tuple
    hfov: float | None = None
    err: float = 0.0
    how: str = ""
    saved: str = ""

    def camera(self):
        _, np = _fieldcam()
        from fieldcam.camera_model import CameraModel
        cam = CameraModel(np.asarray(self.K, float), np.asarray(self.dist, float),
                          np.asarray(self.rvec, float).reshape(3, 1), np.asarray(self.tvec, float).reshape(3, 1))
        cam.reproj_err_px = float(self.err)
        return cam

    def scaled(self, w: int, h: int) -> "FieldCal":
        _, np = _fieldcam()
        sw, sh = w / self.size[0], h / self.size[1]
        K = np.array(self.K, float)
        K[0] *= sw
        K[1] *= sh
        return dataclasses.replace(self, K=K, ref={i: np.asarray(r, float) * [sw, sh] for i, r in self.ref.items()},
                                   size=(int(w), int(h)))

    def to_json(self) -> dict:
        _, np = _fieldcam()
        r = lambda v: [round(float(x), 9) for x in np.asarray(v, float).ravel()]   # noqa: E731
        return {"version": 1, "saved": self.saved, "how": self.how, "width": self.size[0], "height": self.size[1],
                "K": np.asarray(self.K, float).round(9).tolist(), "dist": r(self.dist), "hfov_deg": self.hfov,
                "rvec": r(self.rvec), "tvec": r(self.tvec), "fit_err_px": round(float(self.err), 3),
                "field": {"length": round(self.length, 4), "width": round(self.width, 4)}, "floor_size": self.floor_size,
                "tags": {str(i): [round(float(v), 5) for v in p] for i, p in sorted(self.tags.items())},
                "ref_px": {str(i): r(v) for i, v in sorted(self.ref.items())}}

    @staticmethod
    def from_json(d: dict) -> "FieldCal":
        _, np = _fieldcam()
        return FieldCal(K=np.array(d["K"], float).reshape(3, 3), dist=np.array(d["dist"], float),
                        rvec=np.array(d["rvec"], float).reshape(3, 1), tvec=np.array(d["tvec"], float).reshape(3, 1),
                        length=float(d["field"]["length"]), width=float(d["field"]["width"]),
                        floor_size=float(d["floor_size"]),
                        tags={int(i): tuple(float(v) for v in p) for i, p in d["tags"].items()},
                        ref={int(i): np.array(v, float) for i, v in d["ref_px"].items()},
                        size=(int(d["width"]), int(d["height"])), hfov=d.get("hfov_deg"),
                        err=float(d.get("fit_err_px", 0.0)), how=str(d.get("how", "")), saved=str(d.get("saved", "")))


def load_cal(path: Path, w: int, h: int, floor_size: float) -> tuple[FieldCal | None, str]:
    """eye_cal.json scaled to a w x h picture: (cal, "") or (None, why). No file: (None, "")."""
    p = Path(path)
    if not p.exists():
        return None, ""
    try:
        cal = FieldCal.from_json(json.loads(p.read_text()))
    except (OSError, ValueError, KeyError, TypeError) as e:
        return None, f"{p.name} unreadable ({type(e).__name__}: {e})"
    if abs(w / h - cal.size[0] / cal.size[1]) > 0.01:
        return None, f"{p.name} is for a {cal.size[0]}x{cal.size[1]} picture, the camera gives {w}x{h}"
    if abs(cal.floor_size - floor_size) > 1e-6:
        return None, f"{p.name} was made with eye_floor_size {cal.floor_size}, now {floor_size}"
    if len(cal.tags) < 3:
        return None, f"{p.name} has fewer than 3 floor tags"
    return cal.scaled(w, h), ""


def save_cal(path: Path, cal: FieldCal) -> None:
    """Atomic write (a crash never leaves half a file)."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(cal.to_json(), indent=1) + "\n")
    os.replace(tmp, p)


def _now_iso() -> str:
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def load_robot_colors(path: Path | None = ROBOT_COLORS_PATH) -> dict[str, list[dict]]:
    """{robot id: [{lo, hi}, ...]} from robot_colors.json ({} if missing / unreadable). A robot with no
    entry gets no blob tracking (its board colour is unknown)."""
    try:
        data = json.loads(Path(path).read_text())
    except (OSError, ValueError, TypeError):
        return {}
    out = {}
    for rid, v in (data.items() if isinstance(data, dict) else []):
        try:
            cols = [{"lo": [int(x) for x in c["lo"]], "hi": [int(x) for x in c["hi"]]} for c in v["colors"]]
        except (KeyError, TypeError, ValueError):
            continue
        if cols and all(len(c["lo"]) == 3 and len(c["hi"]) == 3 for c in cols):
            out[rid] = cols
    return out


def save_robot_colors(path: Path, rid: str, colors: list[dict], **meta) -> None:
    """Set one robot's entry in robot_colors.json, keeping the others."""
    try:
        data = json.loads(Path(path).read_text())
        data = data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        data = {}
    data[rid] = {"colors": colors, **meta}
    Path(path).write_text(json.dumps(data, indent=1) + "\n")


def board_ring_pixels(bgr, corners, inner: float = 1.15, outer: float = 1.6, s_min: int = 70):
    """HSV pixels of the board around a decoded tag: between the tag outline grown by `inner` and by
    `outer` (the blue area is ~1.8x the 7.6 cm tag), keeping saturated ones (not carpet, tag or legs)."""
    cv2, np = _fieldcam()
    c = np.asarray(corners, np.float32).reshape(4, 2)
    ctr = c.mean(axis=0)
    h, w = bgr.shape[:2]
    big = (c - ctr) * outer + ctr
    x0, y0 = (max(0, int(v) - 2) for v in big.min(axis=0))
    x1, y1 = (int(v) + 3 for v in big.max(axis=0))
    x1, y1 = min(w, x1), min(h, y1)
    if x1 - x0 < 4 or y1 - y0 < 4:
        raise ValueError("tag outside the frame")
    m = np.zeros((y1 - y0, x1 - x0), np.uint8)
    off = np.array([x0, y0], np.float32)
    cv2.fillConvexPoly(m, np.round(big - off).astype(np.int32), 255)
    cv2.fillConvexPoly(m, np.round((c - ctr) * inner + ctr - off).astype(np.int32), 0)
    hsv = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    px = hsv[m > 0]
    return px[px[:, 1] >= s_min]


def color_mask(hsv, colors: list[dict]):
    cv2, np = _fieldcam()
    m = None
    for c in colors:
        k = cv2.inRange(hsv, np.array(c["lo"], np.uint8), np.array(c["hi"], np.uint8))
        m = k if m is None else m | k
    return m


def find_board(mask, pred, exp_area: float, gate_px: float, offset=(0, 0)):
    """The board blob in a colour mask: (u, v, hull area, score) in full-frame pixels, or None.
    Candidates: outer contours whose convex hull (fills the tag hole and a ring broken by blur) has
    BLOB_AREA x exp_area and aspect <= BLOB_ASPECT, centre within gate_px of `pred`. Best = nearest
    to `pred` and closest to the expected size."""
    cv2, np = _fieldcam()
    best = None
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        if cv2.contourArea(c) < 0.12 * exp_area:
            continue
        hull = cv2.convexHull(c)
        a = cv2.contourArea(hull)
        if not BLOB_AREA[0] * exp_area <= a <= BLOB_AREA[1] * exp_area:
            continue
        (_, _), (rw, rh), _ = cv2.minAreaRect(hull)
        if max(rw, rh) > BLOB_ASPECT * max(min(rw, rh), 1.0):
            continue
        m = cv2.moments(hull)
        u, v = m["m10"] / m["m00"] + offset[0], m["m01"] / m["m00"] + offset[1]
        d = math.hypot(u - pred[0], v - pred[1])
        if d > gate_px:
            continue
        score = d / max(gate_px, 1.0) + abs(math.log(a / exp_area))
        if best is None or score < best[3]:
            best = (u, v, a, score)
    return best


def integrate_cmds(cmds, a: float, b: float, hold: float = CMD_HOLD_S) -> dict:
    """Commanded motion over [a, b] from (t, vx, vy, wz) records sorted by t (zero-order hold, each for at
    most `hold` s: the bridge stops a robot that hears nothing). turn = integral of wz (rad); vx_min, vx,
    vy (time means), wz_max = max |wz|; known = the window is covered by commands."""
    turn = vx_s = vy_s = cover = 0.0
    vx_min, wz_max = math.inf, 0.0
    for i, (t, vx, vy, wz) in enumerate(cmds):
        t_end = min(t + hold, cmds[i + 1][0] if i + 1 < len(cmds) else t + hold)
        lo, hi = max(a, t), min(b, t_end)
        if hi <= lo:
            continue
        d = hi - lo
        turn, vx_s, vy_s, cover = turn + wz * d, vx_s + vx * d, vy_s + vy * d, cover + d
        vx_min, wz_max = min(vx_min, vx), max(wz_max, abs(wz))
    span = max(b - a, 1e-9)
    return {"turn": turn, "vx_min": vx_min if cover else 0.0, "vx": vx_s / span, "vy": vy_s / span,
            "wz_max": wz_max, "known": cover >= 0.8 * span}


def gyro_yaw_at(samples, x: float, fresh: float = GYRO_FRESH_S, gap: float = GYRO_GAP_S) -> float | None:
    """Integrated yaw (rad) at time x from (t, yaw, wz) samples sorted by t: linear between samples at most
    `gap` apart, extrapolated with the end sample's rate up to `fresh` beyond either end, else None."""
    if not samples:
        return None
    t1, y1, w1 = samples[-1]
    if x >= t1:
        return y1 + w1 * (x - t1) if x - t1 <= fresh else None
    t0, y0, w0 = samples[0]
    if x <= t0:
        return y0 - w0 * (t0 - x) if t0 - x <= fresh else None
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid][0] <= x:
            lo = mid
        else:
            hi = mid
    (ta, ya, _), (tb, yb, _) = samples[lo], samples[hi]
    if x == ta:
        return ya
    if tb - ta > gap:
        return None
    return ya + (yb - ya) * (x - ta) / max(tb - ta, 1e-9)


def gyro_turn(samples, a: float, b: float, p: dict | None = None) -> float | None:
    """Measured heading change over [a, b] (rad), or None if the samples do not cover both ends."""
    p = p or {}
    fresh = float(p.get("eye_gyro_fresh_s", GYRO_FRESH_S))
    ya, yb = gyro_yaw_at(samples, a, fresh), gyro_yaw_at(samples, b, fresh)
    return None if ya is None or yb is None else yb - ya


class RobotTrack:
    """One robot between decoded tags (hot thread only): last fix, learned board geometry, heading."""

    def __init__(self):
        self.t_tag: float | None = None      # capture time of the last decoded tag
        self.px: tuple | None = None         # (u, v, t) board / tag centre pixel of the last fix
        self.vpx = (0.0, 0.0)                # pixel velocity (px/s)
        self.off = (0.0, 0.0)                # tag centre - board centre, body frame (m), learned
        self.n_off = 0
        self.area_k = 1.0                    # board hull area / expected (learned while the tag is seen)
        self.yaw: float | None = None        # heading in tag convention (what Eye.tag_pose returns)
        self.t_yaw = 0.0
        self.conf = 0.0
        self.travel_bias: dict[str, float] = {}  # walking dir - tag yaw per speed class, learned
        self.fixes: collections.deque = collections.deque(maxlen=40)   # (t, x, y) tag centre (m)
        self.n = {"tag": 0, "blob": 0, "miss": 0, "old": 0}
        self.last: dict = {}                 # what the last carry_yaw did (debug / replay)

    def note_px(self, u: float, v: float, t: float) -> None:
        if self.px is not None and 0.0 < t - self.px[2] < 0.3:
            dt = t - self.px[2]
            k = 0.5
            self.vpx = (k * (u - self.px[0]) / dt + (1 - k) * self.vpx[0], k * (v - self.px[1]) / dt + (1 - k) * self.vpx[1])
        elif self.px is None or t - self.px[2] >= 0.3:
            self.vpx = (0.0, 0.0)
        self.px = (u, v, t)

    def predict_px(self, t: float) -> tuple[float, float]:
        dt = min(max(0.0, t - self.px[2]), 0.3)
        return self.px[0] + self.vpx[0] * dt, self.px[1] + self.vpx[1] * dt

    def travel(self, t: float, window: float = TRAVEL_WIN_S) -> tuple[float, float] | None:
        """(direction rad, speed m/s) of the tag centre over the last `window` s: least-squares line through
        >= 4 fixes spanning >= 60 % of the window. None if fewer fixes or slower than 0.15 m/s."""
        pts = [f for f in self.fixes if t - window - 1e-6 <= f[0] <= t]
        if len(pts) < 4 or pts[-1][0] - pts[0][0] < 0.6 * window:
            return None
        n = len(pts)
        tm = sum(f[0] for f in pts) / n
        xm, ym = sum(f[1] for f in pts) / n, sum(f[2] for f in pts) / n
        stt = sum((f[0] - tm) ** 2 for f in pts)
        vx = sum((f[0] - tm) * (f[1] - xm) for f in pts) / stt
        vy = sum((f[0] - tm) * (f[2] - ym) for f in pts) / stt
        v = math.hypot(vx, vy)
        return (math.atan2(vy, vx), v) if v >= 0.15 else None

    def walking(self, t: float, cmds, p: dict) -> tuple[float, str] | None:
        """Heading implied by the travel direction (tag convention, before the bias) and the speed class,
        while walking forward (commanded vx >= 0.3 over the window): travel - commanded direction + half the
        commanded turn over the window (a chord points at the heading of its middle)."""
        lag = float(p.get("eye_turn_lag_s", TURN_LAG_S))
        c = integrate_cmds(cmds, t - TRAVEL_WIN_S - lag, t - lag)
        tr = self.travel(t)
        if tr is None or not c["known"] or c["vx_min"] < 0.3:
            return None
        half = 0.5 * float(p.get("eye_turn_gain", TURN_GAIN)) * c["turn"]
        cls = "fast" if c["vx"] > float(p.get("fast_yaw_vx", 0.5)) else "slow"
        return wrap(tr[0] - math.atan2(c["vy"], c["vx"]) + half), cls

    def _bias(self, cls: str, p: dict) -> float:
        """walking direction - tag heading for this speed class: learned at decoded tags, else the brain's
        convention (walking direction = tag heading + fast_yaw_bias above fast_yaw_vx)."""
        b = self.travel_bias.get(cls)
        if b is not None:
            return b
        return math.radians(float(p.get("fast_yaw_bias_deg", 0.0))) if cls == "fast" else 0.0

    def on_tag(self, t: float, yaw: float, ctr: tuple, cmds, p: dict) -> None:
        self.fixes.append((t, ctr[0], ctr[1]))
        wk = self.walking(t, cmds, p)
        if wk is not None:   # learn how the gait walks off its tag heading (per speed class)
            s, cls = wrap(wk[0] - yaw), wk[1]
            old = self.travel_bias.get(cls)
            self.travel_bias[cls] = s if old is None else wrap(old + 0.15 * wrap(s - old))
        self.t_tag, self.yaw, self.t_yaw, self.conf = t, yaw, t, 1.0
        self.n["tag"] += 1

    def carry_yaw(self, t: float, cmds, p: dict, gyro=None) -> float:
        """Heading at t without a decoded tag: + the measured turn since the last update when the bridge's
        yaw-rate samples (`gyro`, Link.gyro) cover it, else gain x commanded turn (good when slow: 6 deg rms;
        in the fast gait the turn rate departs from the command); then pulled toward the heading implied by
        the travel direction while walking forward (eye_travel_tau_s; eye_travel_tau_gyro_s, weaker, when
        measured)."""
        lag = float(p.get("eye_turn_lag_s", TURN_LAG_S))
        dt = max(0.0, t - self.t_yaw)
        g = gyro_turn(gyro, self.t_yaw, t, p) if gyro and float(p.get("eye_gyro", 1)) else None
        c = integrate_cmds(cmds, self.t_yaw - lag, t - lag)
        if g is not None:
            turn = g
            self.conf = max(0.0, self.conf * math.exp(-dt / YAW_CONF_GYRO_TAU_S))
            tau = float(p.get("eye_travel_tau_gyro_s", TRAVEL_TAU_GYRO_S))
        else:
            turn = float(p.get("eye_turn_gain", TURN_GAIN)) * c["turn"]
            self.conf = max(0.0, self.conf * math.exp(-dt / YAW_CONF_TAU_S) - 0.15 * abs(turn))
            tau = float(p.get("eye_travel_tau_s", TRAVEL_TAU_S))
        self.yaw = wrap(self.yaw + turn)
        self.last = {"turn": turn, "travel": None, "cmd_known": c["known"], "src": "gyro" if g is not None else "cmd"}
        wk = self.walking(t, cmds, p) if tau > 0 else None
        if wk is not None:
            target = wrap(wk[0] - self._bias(wk[1], p))
            self.last["travel"] = target
            self.yaw = wrap(self.yaw + (1 - math.exp(-dt / tau)) * wrap(target - self.yaw))
            self.conf = max(self.conf, 0.5)
        self.t_yaw = t
        return self.yaw


def _mount_keys(rid: str, over: dict, where: str) -> dict:
    """The tag-mount knobs (ROBOT_MOUNT_KEYS) of one robot's overrides, as floats. Other keys are for other
    modules (e.g. act_lag_s for the brain) and are left out. A non-number raises ValueError."""
    out = {}
    for k in ROBOT_MOUNT_KEYS:
        if k in over:
            v = over[k]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"{where}: {rid} {k!r} must be a number, got {v!r}")
            out[k] = float(v)
    return out


def _fieldcam():
    if str(OP) not in sys.path:
        sys.path.insert(0, str(OP))
    import cv2
    import numpy as np
    return cv2, np


# ---------------------------------------------------------------------------------------
# frame sources: read() -> (frame, t_receipt_monotonic) or (None, None)
#
# live camera: capture in a child process that the eye watches and respawns (Nov 28)
# Nov 27: the phone camera dropped mid-session and only an app restart brought it back. Why a child process:
#   * the camera list a process sees comes from AVFoundation / CoreMediaIO, which update it through notifications
#     delivered on the main thread's run loop; a Python process never runs that loop, so a phone that went away and
#     came back can stay invisible (or a dead device object stay "openable") to the process that first opened it.
#     A NEW process enumerates afresh, so every connection attempt is a new child (hypothesis from the code and
#     Apple's notification model; not reproduced with the phone);
#   * a read that hangs inside AVFoundation can only be stopped by killing its process (the camera is then freed);
#   * the frozen phone feed of Nov 27 042513Z (27 s of identical pictures at full rate) never counted as lost.
# The child finds the camera, opens it once and writes frames into shared memory (CAM_SLOTS slots at the pinned
# size, one byte on a pipe per frame wakes the reader). The watcher thread kills and respawns it when frames stop
# (CAM_STALL_S), the picture freezes (CAM_FROZEN_S) or a start delivers nothing (CAM_OPEN_S), with back-off while
# attempts fail. The picture size is pinned to the eye's (= the calibration's): a camera that comes back at another
# size of the same aspect is scaled to it (the calibration scales the same way); another aspect is refused (another
# view). A camera chosen by a name ("phone") is locked to the device found first: a reconnect never falls back to
# another camera.
CAM_SLOTS = 3
CAM_STALL_S = 1.0       # no new frame this long -> respawn
CAM_FROZEN_S = 1.0      # identical pictures this long -> respawn (live sensor noise differs every frame)
CAM_OPEN_S = 6.0        # a child that delivers no moving picture this long after its start -> respawn
CAM_BACKOFF_S = (0.0, 0.5, 1.0, 2.0, 3.0)   # wait before attempt n after a failure (last value repeats)
_HI_SEQ, _HI_OWNER, _HI_W, _HI_H = 0, 1, 2, 3        # int64 header: frames published, owner generation, camera size
_HF_FRAME, _HF_CHANGED, _HF_SLOT0 = 0, 1, 2          # float64 header: monotonic t of the last frame / last change
_HDR_BYTES = 256
ROOT = Path(__file__).resolve().parent.parent       # final_project (for the child's imports)
CAPTURE_ENV = "FC_CAPTURE"   # "child" (default) | "inproc" = the pre-Nov 28 in-process grabber (escape hatch; app --capture)
PERMISSION_WORDS = ("not authorized", "not authorised", "permission", "denied", "tcc")
PERMISSION_MSG = "camera permission denied in capture child: run with --capture inproc"


def _cam_views(np, buf, w: int, h: int):
    hi = np.ndarray((8,), np.int64, buf, 0)
    hf = np.ndarray((8 + CAM_SLOTS,), np.float64, buf, 64)
    slots = [np.ndarray((h, w, 3), np.uint8, buf, _HDR_BYTES + i * w * h * 3) for i in range(CAM_SLOTS)]
    return hi, hf, slots


def _plain(t: str) -> str:
    return t.lower().replace("\u2019", "'").strip()


def find_device(source: str, lock: str | None = None, cameras: list[dict] | None = None) -> tuple[dict | None, str]:
    """The camera to open for `source` right now: ({"index", "name", ...}, "") or (None, why).
    lock: the exact device name found at the first connection; a reconnect accepts only that device.
    Without a lock a name is matched as fieldcam.capture.find_camera does ("phone" = a phone-looking camera, else
    any non-built-in one); iPhone "Desk View" cameras are left out unless asked for by name."""
    source = str(source)
    if source.startswith("fake:"):
        return FakeCamera.find(source, lock)
    if source.isdigit():
        return {"index": int(source), "name": f"camera {source}"}, ""
    if "://" in source:
        return {"index": source, "name": source}, ""
    _fieldcam()
    from fieldcam.capture import HELP, find_camera, list_cameras
    cams = list_cameras() if cameras is None else cameras
    if "desk view" not in _plain(source):
        cams = [c for c in cams if "desk view" not in _plain(c["name"])]
    seen = f" macOS sees: {', '.join(c['name'] for c in cams) or 'no camera'}."
    if lock is not None:
        found = next((c for c in cams if _plain(c["name"]) == _plain(lock)), None)
        return (found, "") if found else (None, f"{lock} not connected.{seen} {HELP}")
    found = find_camera(source, cams)
    return (found, "") if found else (None, f"no camera matches {source!r}.{seen} {HELP}")


def _open_device(dev: dict, w: int, h: int, fps: int):
    if "fake" in dev:
        if FakeCamera.control(dev["fake"]).get("mode") == "denied":   # what OpenCV prints without camera access
            print("OpenCV: not authorized to capture video (status 0), requesting...", file=sys.stderr, flush=True)
            return None
        return FakeCamera(dev["fake"])
    cv2, _ = _fieldcam()
    target = dev["index"]
    cap = (cv2.VideoCapture(target, cv2.CAP_AVFOUNDATION) if isinstance(target, int) and sys.platform == "darwin"
           else cv2.VideoCapture(target))
    if cap is None or not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def _capture_child(spec_json: str) -> None:
    """Child process: one camera connection. Exits when the parent dies or replaces it (owner generation)."""
    spec = json.loads(spec_json)
    cv2, np = _fieldcam()
    from multiprocessing import resource_tracker, shared_memory
    shm = shared_memory.SharedMemory(name=spec["shm"])
    try:   # the parent owns and unlinks the memory; this process's tracker must not remove it when we exit
        resource_tracker.unregister(shm._name, "shared_memory")  # noqa: SLF001
    except Exception:  # noqa: BLE001
        pass
    W, H, gen, parent, fd = spec["w"], spec["h"], spec["gen"], spec["parent"], spec["notify_fd"]
    hi, hf, slots = _cam_views(np, shm.buf, W, H)

    def say(**kw):
        print(json.dumps(kw), flush=True)

    def watchdog():   # runs even while the main thread is stuck inside a camera read
        while os.getppid() == parent and int(hi[_HI_OWNER]) == gen:
            time.sleep(0.2)
        os._exit(0)
    threading.Thread(target=watchdog, daemon=True).start()

    dev, why = find_device(spec["source"], spec.get("lock"))
    if dev is None:
        say(state="absent", why=why)
        sys.exit(2)
    say(state="opening", device=dev["name"], index=dev["index"] if isinstance(dev["index"], int) else None)
    cap = _open_device(dev, W, H, spec["fps"])
    if cap is None:
        say(state="open_failed", device=dev["name"], why=f"could not open {dev['name']}")
        sys.exit(3)
    os.set_blocking(fd, False)
    seq, prev, n, t_copy, t_stats, live = int(hi[_HI_SEQ]), None, 0, 0.0, time.monotonic() + 5.0, False
    while True:
        slot = (seq + 1) % CAM_SLOTS
        dst = slots[slot]
        ok, img = cap.read(dst)
        t = time.monotonic()
        if not ok or img is None:
            time.sleep(0.005)
            continue
        t0 = time.perf_counter()
        sh, sw = img.shape[:2]
        if (sw, sh) != (W, H):
            if abs(sw / sh - W / H) > 0.01:
                say(state="wrong_size", device=dev["name"], size=[sw, sh],
                    why=f"{dev['name']} gives {sw}x{sh}; the calibration needs {W}x{H} (or the same aspect)")
                sys.exit(4)
            cv2.resize(img, (W, H), dst=dst, interpolation=cv2.INTER_LINEAR)
        elif not np.shares_memory(img, dst):
            np.copyto(dst, img)
        sig = dst[::8, ::8, 1].astype(np.int16)
        moved = prev is not None and float(np.abs(sig - prev).mean()) >= FROZEN_DIFF
        if moved or prev is None:
            hf[_HF_CHANGED] = t
        prev = sig
        hf[_HF_SLOT0 + slot] = t
        hf[_HF_FRAME] = t
        hi[_HI_W], hi[_HI_H] = sw, sh
        seq += 1
        hi[_HI_SEQ] = seq   # published last: the reader takes the newest seq's slot
        try:
            os.write(fd, b"\0")
        except (BlockingIOError, OSError):
            pass
        t_copy += time.perf_counter() - t0
        n += 1
        if moved and not live:   # a moving picture (a frozen feed also delivers frames)
            live = True
            say(state="streaming", device=dev["name"], size=[sw, sh], scaled=(sw, sh) != (W, H))
        if t >= t_stats:
            say(stats={"frames": n, "store_ms": round(1000 * t_copy / n, 3)})
            t_stats = t + 5.0


class LiveSource:
    """A live camera (name like "phone", index, or stream URL) captured by a child process that is respawned when
    frames stop or freeze. read() -> (newest frame, monotonic receipt time) or (None, None) after 1 s without one."""
    latency = True

    def __init__(self, source, width: int, height: int, fps: int = 30, wait: float | None = 20.0, say=None):
        _, np = _fieldcam()
        from multiprocessing import shared_memory
        self.source, self.width, self.height, self.fps = str(source), int(width), int(height), int(fps)
        self._say = say or (lambda m: print("[eye]", m, flush=True))
        self.by_name = not (self.source.isdigit() or "://" in self.source)   # a fake camera acts as a named one
        self.shm = shared_memory.SharedMemory(create=True, size=_HDR_BYTES + CAM_SLOTS * self.width * self.height * 3)
        self._hi, self._hf, self._slots = _cam_views(np, self.shm.buf, self.width, self.height)
        self._hi[:], self._hf[:] = 0, 0.0
        self._rfd, self._wfd = os.pipe()
        os.set_blocking(self._rfd, False)
        self._mu = threading.Lock()
        self.proc, self.gen, self.lock_name, self.device = None, 0, None, None
        self.state, self._message, self.reason = "starting", f"opening camera {self.source!r}", ""
        self._down_why = ""
        self.reconnects = self.respawns = self.fails = 0
        self.camera_size: list | None = None
        self.child_stats: dict = {}
        self.t_down: float | None = None
        self.last_down_s: float | None = None
        self._why = ""                      # the current child's own reason (not connected, wrong size, ...)
        self._live = False                  # the current child delivers a moving picture
        self._seq_spawn = self._seq_seen = 0
        self._t_spawn = self._next_spawn = self._told = 0.0
        self._closed = False
        self._stderr: collections.deque = collections.deque(maxlen=5)
        atexit.register(self.close)
        threading.Thread(target=self._watch, daemon=True, name="fc-eye-camwatch").start()
        t_end = None if wait is None else time.monotonic() + wait
        while self.state != "ok":   # a moving picture
            if t_end is not None and time.monotonic() > t_end:
                msg = self.message
                self.close()
                raise RuntimeError(f"camera {self.source!r} not open after {wait:.0f} s: {msg}")
            time.sleep(0.02)

    @property
    def message(self) -> str:
        if self.state == "reconnecting" and self.t_down is not None:
            return f"camera reconnecting ({time.monotonic() - self.t_down:.0f} s): {self._down_why}"
        return self._message

    # ---- the watcher ------------------------------------------------------------------
    def _spawn(self) -> None:
        self.gen += 1
        self._hi[_HI_OWNER] = self.gen
        self._seq_spawn = int(self._hi[_HI_SEQ])
        self._why, self._live = "", False
        spec = {"shm": self.shm.name, "w": self.width, "h": self.height, "fps": self.fps, "source": self.source,
                "lock": self.lock_name, "gen": self.gen, "parent": os.getpid(), "notify_fd": self._wfd}
        env = {**os.environ, "PYTHONPATH": os.pathsep.join(filter(None, [str(ROOT), os.environ.get("PYTHONPATH")]))}
        self.proc = subprocess.Popen(
            [sys.executable, "-c", "import sys; from fc.eye import _capture_child; _capture_child(sys.argv[1])",
             json.dumps(spec)], cwd=str(ROOT), env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, pass_fds=(self._wfd,), text=True, bufsize=1)
        self._t_spawn = time.monotonic()
        self.respawns += 1
        threading.Thread(target=self._read_out, args=(self.proc, self.gen), daemon=True).start()
        threading.Thread(target=self._read_err, args=(self.proc,), daemon=True).start()

    def _read_out(self, proc, gen: int) -> None:
        for line in proc.stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if gen == self.gen:
                self._on_child(msg)

    def _read_err(self, proc) -> None:
        for line in proc.stderr:
            if line.strip():
                self._stderr.append(line.strip()[:300])

    def _on_child(self, msg: dict) -> None:
        st = msg.get("state")
        with self._mu:
            if "stats" in msg:
                self.child_stats = msg["stats"]
            elif st == "streaming":
                self._live = True
                self.device, self.camera_size = msg.get("device"), msg.get("size")
                was_down, self.state, self.fails = self.t_down, "ok", 0
                size = "x".join(map(str, self.camera_size or []))
                scaled = f", scaled to {self.width}x{self.height}" if msg.get("scaled") else ""
                if self.by_name and self.lock_name is None:
                    self.lock_name = self.device   # reconnects accept only this device
                if was_down is not None:
                    self.last_down_s = time.monotonic() - was_down
                    self.reconnects += 1
                    self.t_down = None
                    self._message = f"camera back after {self.last_down_s:.1f} s: {self.device} {size}{scaled}"
                else:
                    self._message = f"camera {self.source!r} -> {self.device} {size}{scaled}"
                self._say(self.message)
            elif st in ("absent", "open_failed", "wrong_size"):
                self._why = msg.get("why", st)
            elif st == "opening" and self.state != "ok":
                self._why = f"opening {msg.get('device')}"

    def _permission(self) -> bool:
        """The child's stderr (OpenCV: "not authorized to capture video") or reason says macOS refused the camera."""
        text = " ".join([self._why, *self._stderr]).lower()
        return any(w in text for w in PERMISSION_WORDS)

    def _stall(self, now: float) -> str:
        if self._live:   # this child has delivered a moving picture: watch its frames
            gap, frozen = now - float(self._hf[_HF_FRAME]), now - float(self._hf[_HF_CHANGED])
            if gap > CAM_STALL_S:
                return f"no new frame for {gap:.1f} s"
            if frozen > CAM_FROZEN_S:
                return f"picture frozen for {frozen:.1f} s"
            return ""
        if int(self._hi[_HI_SEQ]) > self._seq_spawn and now - float(self._hf[_HF_CHANGED]) > CAM_FROZEN_S:
            return f"picture frozen for {now - float(self._hf[_HF_CHANGED]):.1f} s"   # frames, never a moving one
        if now - self._t_spawn > CAM_OPEN_S:
            if self._permission():
                return PERMISSION_MSG
            return f"no frame {CAM_OPEN_S:.0f} s after opening{f' ({self._why})' if self._why else ''}"
        return ""

    def _down(self, why: str) -> None:
        if self._closed:
            return
        with self._mu:
            now = time.monotonic()
            self.fails += 1
            self._next_spawn = now + CAM_BACKOFF_S[min(self.fails - 1, len(CAM_BACKOFF_S) - 1)]
            if self.state == "ok":
                self.t_down, self.reason, self._told = now, why, now
                self._say(f"camera lost ({why}): reconnecting")
            self.state = "reconnecting" if self.t_down is not None else "waiting"
            self._down_why = why
            self._message = f"waiting for camera {self.source!r}: {why}"
            if now - self._told > 8.0:
                self._told = now
                self._say(self.message)

    def _end(self, proc, why: str) -> None:
        """Kill a child (if still running) and free the camera; never blocks for long."""
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(0.5)
            except subprocess.TimeoutExpired:
                proc.kill()
                try:
                    proc.wait(1.0)
                except subprocess.TimeoutExpired:
                    pass   # stuck in the kernel; the owner generation already retired it
        if proc is self.proc:
            self.proc = None
            self._down(why)

    def _watch(self) -> None:
        while not self._closed:
            now = time.monotonic()
            p = self.proc
            if p is not None:
                code = p.poll()
                if code is not None:
                    time.sleep(0.05)   # let the output reader catch the child's last words
                    why = (PERMISSION_MSG if self._permission() else
                           self._why or (self._stderr[-1] if self._stderr else "") or f"capture process ended ({code})")
                    self._end(p, why)
                else:
                    why = self._stall(now)
                    if why:
                        self._hi[_HI_OWNER] = -1   # retire it at once (its watchdog exits) ...
                        self._end(p, why)          # ... and make sure it is gone
            if self.proc is None and not self._closed and now >= self._next_spawn:
                try:
                    self._spawn()
                except OSError as e:
                    self._down(f"cannot start the capture process: {e}")
            time.sleep(0.05)

    # ---- for the eye --------------------------------------------------------------------
    def read(self):
        t_end = time.monotonic() + 1.0
        hi, hf, slots = self._hi, self._hf, self._slots
        while not self._closed and slots:
            seq = int(hi[_HI_SEQ])
            if seq > self._seq_seen:
                slot = seq % CAM_SLOTS
                t = float(hf[_HF_SLOT0 + slot])
                frame = slots[slot].copy()
                if int(hi[_HI_SEQ]) - seq <= 1 and float(hf[_HF_SLOT0 + slot]) == t:   # not overwritten meanwhile
                    self._seq_seen = seq
                    return frame, t
                continue
            left = t_end - time.monotonic()
            if left <= 0:
                return None, None
            try:
                select.select([self._rfd], [], [], min(left, 0.05))
                while os.read(self._rfd, 65536):
                    pass
            except BlockingIOError:
                pass
            except (OSError, ValueError):   # closed meanwhile
                break
        return None, None

    def health(self) -> dict:
        now = time.monotonic()
        p = self.proc
        return {"state": self.state, "capture": "child", "message": self.message, "device": self.device,
                "lock": self.lock_name,
                "size": [self.width, self.height], "camera_size": self.camera_size, "reconnects": self.reconnects,
                "respawns": self.respawns, "last_reason": self.reason,
                "down_s": round(now - self.t_down, 1) if self.t_down is not None else 0.0,
                "last_down_s": None if self.last_down_s is None else round(self.last_down_s, 1),
                "pid": p.pid if p is not None else None, "child": dict(self.child_stats)}

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._hi[_HI_OWNER] = -1
        p = self.proc
        if p is not None:
            self._end(p, "closed")
        for fd in (self._rfd, self._wfd):
            try:
                os.close(fd)
            except OSError:
                pass
        try:
            self.shm.unlink()   # the memory goes once nothing maps it any more
        except Exception:  # noqa: BLE001 - closing must never raise
            pass
        self._hi = self._hf = None
        self._slots = []
        try:
            self.shm.close()
        except Exception:  # noqa: BLE001 - a read() still holding a view: freed with the process
            pass


class InprocSource:
    """The pre-Nov 28 capture path, unchanged: fieldcam's FrameGrabber on a thread in this process (no respawn).
    Escape hatch while the child capture has not met the real iPhone / macOS camera permission:
    `fc.app --capture inproc` (or FC_CAPTURE=inproc)."""
    latency = True

    def __init__(self, source, width, height):
        _fieldcam()
        from fieldcam.capture import FrameGrabber
        src = int(source) if str(source).isdigit() else source
        self.g = FrameGrabber(src, width, height, 30, wait=20, say=lambda m: print("[eye]", m, flush=True))
        self.width, self.height, self.n = self.g.width, self.g.height, -1

    def read(self):
        frame, receipt, self.n = self.g.read(self.n)
        if frame is None:
            return None, None
        return frame, time.monotonic() - (time.time() - receipt)

    def health(self) -> dict:
        return {"state": self.g.status, "capture": "inproc", "reconnects": self.g.reconnects}

    def close(self):
        self.g.close()


class FakeCamera:
    """Stand-in camera for tests and measurements: source "fake:<control.json>". The control file is read on every
    frame: {"mode": "ok" | "stall" (reads fail) | "freeze" (same picture) | "hang" (read blocks) | "absent"
    (not connected), "name": device name (default "Fake iPhone Camera"), "size": [w, h] (default 1920x1080),
    "image": picture file (default a grey ramp), "fps": 30}; "denied" = macOS camera permission refused. Sensor noise on every 8th pixel makes frames differ."""

    def __init__(self, ctl: str):
        _, self.np = _fieldcam()
        self.ctl, self.n, self.next, self._base = ctl, 0, time.monotonic(), None

    @staticmethod
    def control(ctl: str) -> dict:
        try:
            return json.loads(Path(ctl).read_text())
        except (OSError, ValueError):
            return {}

    @staticmethod
    def find(source: str, lock: str | None) -> tuple[dict | None, str]:
        ctl = source[len("fake:"):]
        c = FakeCamera.control(ctl)
        name = c.get("name", "Fake iPhone Camera")
        if c.get("mode") == "absent":
            return None, "fake camera not connected"
        if lock is not None and name != lock:
            return None, f"{lock} not connected. macOS sees: {name}."
        return {"index": 0, "name": name, "fake": ctl}, ""

    def isOpened(self):  # noqa: N802 - cv2.VideoCapture interface
        return True

    def set(self, *_a):
        return True

    def release(self):
        pass

    def _picture(self, c: dict):
        np = self.np
        w, h = c.get("size", (1920, 1080))
        key = (c.get("image"), w, h)
        if self._base is None or self._base[0] != key:
            cv2, _ = _fieldcam()
            img = cv2.imread(str(c["image"])) if c.get("image") else None
            if img is None:
                img = np.repeat(np.tile(np.linspace(30, 220, w, dtype=np.uint8), (h, 1))[:, :, None], 3, axis=2)
            elif img.shape[:2] != (h, w):
                img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            img = np.clip(img, 0, 250).astype(np.uint8)
            sub = img[::8, ::8].shape
            self._base = (key, img, [np.random.default_rng(i).integers(0, 4, sub, dtype=np.uint8) for i in range(2)])
        return self._base[1], self._base[2]

    def read(self, out=None):
        c = self.control(self.ctl)
        mode = c.get("mode", "ok")
        time.sleep(max(0.0, self.next - time.monotonic()))
        self.next = max(self.next + 1.0 / float(c.get("fps", 30)), time.monotonic() - 0.1)
        if mode == "hang":
            time.sleep(3600)
        if mode in ("stall", "absent"):
            return False, None
        base, noise = self._picture(c)
        frame = out if out is not None and out.shape == base.shape else self.np.empty_like(base)
        self.np.copyto(frame, base)
        if mode != "freeze":   # frozen: the same picture as the last one again
            self.n += 1
        frame[::8, ::8] += noise[self.n % 2]
        return True, frame


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
                 width: int = 1920, height: int = 1080, ball_color_path: Path | None = CONFIG / "ball_color.json",
                 robot_colors_path: Path | None = ROBOT_COLORS_PATH, cal_path: Path | None = EYE_CAL_PATH,
                 robots_dir: Path | None = ROBOTS_DIR, robot_over: dict | None = None, capture: str | None = None):
        """robots_dir: per-robot override files (<id>.json; None = none). robot_over: {robot id: {knob: value}}
        given by the caller instead of reading robots_dir (only ROBOT_MOUNT_KEYS are used here).
        capture: live camera path, "child" (default) or "inproc"; None = $FC_CAPTURE or "child"."""
        cv2, np = _fieldcam()
        from fieldcam.config import load_config, set_field_size
        self.cv2, self.np = cv2, np
        self.field, self.robots, self.params = field, robots, params
        self.source_name = str(source)
        self.capture = (capture or os.environ.get(CAPTURE_ENV) or "child").strip().lower()
        if self.capture not in ("child", "inproc"):
            raise ValueError(f"capture must be 'child' or 'inproc', got {self.capture!r}")
        cfg = load_config(None)
        self.synthetic = self.source_name == "synthetic"
        self.robots_dir = None if robot_over is not None or robots_dir is None else Path(robots_dir)
        self._over_mtimes: dict[str, float | None] = {}
        self._next_over = 0.0
        self.robot_over: dict[str, dict] = {r.id: {} for r in robots}
        if robot_over is not None:
            self.robot_over.update({rid: _mount_keys(rid, v, "robot_over") for rid, v in robot_over.items()})
        self._reload_robot_over(strict=True)
        cfg["ball"]["radius"] = field.ball_radius   # fc/config/field.json is the one source
        # The synthetic renderer always paints fieldcam's default (pink) ball, so a floor-calibrated
        # ball_color.json (e.g. the purple ball of Nov 25) would hide it: synthetic uses the default.
        saved = ballcal.load(ball_color_path) if ball_color_path and not self.synthetic else None
        self.ball_color_source = (str(ball_color_path) if saved else
                                  "fieldcam default (synthetic renderer)" if self.synthetic else "fieldcam field.yaml default")
        if saved:
            cfg["ball"]["colors"] = saved
        self._cfg_size = (field.length, field.width)   # field.json size (the brain's Field may be resized)
        cfg["field"]["goal_width"] = field.goal_width
        cfg.pop("field_setup", None)
        set_field_size(cfg, field.length, field.width)
        mount = self.mount()
        cfg["tags"]["floor_size"] = mount["floor_size"]
        mounts = {r.id: self.mount(r.id) for r in robots}
        cfg["robots"] = {r.id: {
            "tag_id": r.tag, "attacks": "east" if r.team == "blue" else "west",
            "tag_height": mounts[r.id]["tag_height"], "tag_offset": np.array([mounts[r.id]["tag_offset_x"], 0.0]),
            "yaw_offset": mounts[r.id]["yaw_offset"],
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
        self.ball_vel = BallVelocity()
        self._ball: Ball | None = None
        self.ball_raw: collections.deque = collections.deque(maxlen=int(BALL_TRACK_S * 40))   # (t_cap, x, y)
        # frozen feed: the same picture again (or no frame) for FROZEN_S -> paused, observations not refreshed
        self._sig = None                 # 1/8 grey subsample of the last frame
        self._t_new = time.monotonic()  # receipt time of the last frame that differed from the one before
        self.paused = False
        self.pauses = 0
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
        self._src_reconnects = 0                      # live camera reconnects seen (see _after_reconnect)
        self._frame, self._frame_n = None, 0          # latest raw frame (recorder, Play view)
        self._view: tuple | None = None              # (frame_n, width, jpeg) cache for view_jpeg
        self._overlay: tuple | None = None           # (cam id, tag height, homographies) cache
        # robot board blobs (see BLOB_* above): colours per robot (none on the synthetic renderer, which
        # draws no board), per-robot tracks, commands sent (app thread -> note_cmds) for the heading
        self.robot_colors = load_robot_colors(robot_colors_path) if robot_colors_path and not self.synthetic else {}
        self.robot_colors_source = {rid: str(robot_colors_path) for rid in self.robot_colors}
        self._tracks: dict[str, RobotTrack] = {r.id: RobotTrack() for r in robots}
        self._cmd_lock = threading.Lock()
        self._cmds: dict[str, collections.deque] = {r.id: collections.deque(maxlen=300) for r in robots}
        self.gyro_source = None   # callable(robot id) -> [(t, yaw, wz)] measured yaw rate (app: Link.gyro)
        self._blob_seeds: dict[int, tuple[float, float, float]] = {}   # tag id -> (u, v, t) board seen
        self._cal_lock = threading.Lock()
        # field calibration (see the module notes). The site's eye_cal.json belongs to the live camera: the
        # synthetic renderer and recorded videos use it only when a path is passed explicitly.
        live = not self.synthetic and not Path(self.source_name).exists()
        self.cal_path = None if cal_path is None or (Path(cal_path) == EYE_CAL_PATH and not live) else Path(cal_path)
        self.cal: FieldCal | None = None
        self._refit: dict | None = None     # operator re-fit request (recalibrate -> eye thread)
        self.calmon = {"state": "none", "why": "not calibrated yet", "source": None, "shift": {}, "resid": {},
                       "since": {}, "ignored": {}, "cam_since": None, "hist": {}, "checked": None, "resolves": 0,
                       "events": collections.deque(maxlen=20)}

    # ---- setup -------------------------------------------------------------------
    def mount(self, rid: str | None = None) -> dict:
        """Tag mount knobs, read from the live params dict on every frame so UI edits apply at
        once; with a robot id, that robot's overrides (robots/<id>.json) win over params.
        The synthetic renderer has its own fixed geometry."""
        if self.synthetic:
            return {"floor_size": 0.15, "tag_height": 0.18, "tag_offset_x": 0.0, "yaw_offset": 0.0}
        p = self.params
        over = self.robot_over.get(rid) if rid is not None else None
        if over:
            p = {**p, **over}
        return {"floor_size": float(p.get("eye_floor_size", 0.15875)), "tag_height": float(p.get("eye_tag_height", 0.24)),
                "tag_offset_x": float(p.get("eye_tag_offset_x", 0.065)),
                "yaw_offset": math.radians(float(p.get("eye_yaw_offset_deg", 0.0)))}

    def _reload_robot_over(self, strict: bool = False) -> None:
        """(Re-)read robots/<id>.json for each robot when its file changed. strict (startup): a bad file raises;
        later a bad edit keeps the previous values and says why."""
        if self.robots_dir is None:
            return
        for r in self.robots:
            path = self.robots_dir / f"{r.id}.json"
            try:
                mt = path.stat().st_mtime
            except OSError:
                mt = None
            if r.id in self._over_mtimes and self._over_mtimes[r.id] == mt:
                continue
            self._over_mtimes[r.id] = mt
            try:
                new = _mount_keys(r.id, load_robot_overrides(r.id, self.robots_dir), str(path))
            except ValueError as e:
                if strict:
                    raise
                print(f"[eye] {e}: keeping {r.id}'s previous mount", flush=True)
                continue
            if r.id in self.robot_over and self.robot_over[r.id] != new and not strict:
                print(f"[eye] {r.id} tag mount from {path.name}: {new or 'params.json defaults'}", flush=True)
            self.robot_over[r.id] = new

    def mounts(self) -> dict:
        """Per robot: its mount knobs in params units, and which come from its own file (for status / run meta)."""
        out = {}
        for r in self.robots:
            m = self.mount(r.id)
            out[r.id] = {"tag_height": m["tag_height"], "tag_offset_x": m["tag_offset_x"],
                         "yaw_offset_deg": round(math.degrees(m["yaw_offset"]), 3),
                         "own": sorted(self.robot_over.get(r.id, {}))}
        return out

    def _apply_floor_size(self, size: float) -> None:
        """A changed corner-tag size changes the whole calibration: recalibrate."""
        if self.perc is not None and abs(self.cfg["tags"]["floor_size"] - size) > 1e-6:
            self.cfg["tags"]["floor_size"] = size   # perc.cfg is this same dict
            self.perc.invalidate_calibration("eye_floor_size changed")   # operator action: full re-fit
            self._next_cal = 0.0
            self.cal = None
            self._cal_state("none", "eye_floor_size changed: re-fitting from the four corner tags")

    def _open_source(self):
        s, (w, h) = self.source_name, self.src_size
        if s == "synthetic":
            return SyntheticSource(self.cfg, self.field, self.robots, w, h)
        p = Path(s)
        if p.is_dir():
            return FolderSource(p)
        if p.is_file():
            return VideoSource(p)
        return InprocSource(s, w, h) if self.capture == "inproc" else LiveSource(s, w, h)

    def _setup(self) -> None:
        import fieldcam.pipeline as pipeline
        from fieldcam.pipeline import Perception
        self.src = self._open_source()
        pipeline.CALIBRATION_DRIFT_LIMIT_PX = 8.0 * self.src.width / 896.0
        self.perc = Perception(self.cfg, self.src.width, self.src.height)
        self._install_tracking()
        self.message = f"{self.src.width}x{self.src.height}, calibrating from corner tags 0-3"
        if self.cal_path is not None:
            cal, why = load_cal(self.cal_path, self.src.width, self.src.height, self.cfg["tags"]["floor_size"])
            if cal is not None:
                self._install(cal)
                self.calmon["source"] = str(self.cal_path)
                self._cal_state("unverified", f"loaded {self.cal_path.name} ({cal.how}, {cal.saved}): field "
                                f"{cal.length:.2f} x {cal.width:.2f} m, lens {cal.hfov or 0:.1f} deg; waiting for floor tags to verify")
            elif why:
                self.message = f"{why}; calibrating from corner tags 0-3"

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
        self._cal_det = (standard(0.01), relaxed(0.02))            # robotcal (app thread, under _cal_lock)
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
                    for tid, b in self._blob_seeds.items():   # the board seen more recently: look there
                        if tid not in seeds or b[2] > seeds[tid][2]:
                            seeds[tid] = b
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
            next_deep = self.scan_once(frame, t_recv, next_deep)
            time.sleep(max(0.0, SCAN_PERIOD_S - (time.perf_counter() - t0)))

    def scan_once(self, frame, t_recv: float, next_deep: float = 0.0, now: float | None = None) -> float:
        """One background scan of `frame` (also called inline by the offline replay). Returns when the
        next deep search may run."""
        cv2 = self.cv2
        t0 = time.perf_counter()
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = self._scan_det.detect(gray)
            with self._seed_lock:
                seen = {tid: s[2] for tid, s in self._seeds.items()}
            lost = {tid for tid in self._robot_tags - tags.keys() if t_recv - seen.get(tid, -1e9) > DEEP_AFTER_S}
            now = time.monotonic() if now is None else now
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
        return next_deep

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

    # ---- field calibration: full fit (once / operator), per-tag monitor, pose-only re-solve ----------------
    def _full_fit(self, frame, tags: dict | None = None) -> tuple[FieldCal | None, str]:
        """Lens angle (when there is no intrinsics file; Tej's sweep), field size and camera pose from the four
        corner tags, on a scratch Perception: the live calibration is untouched until the result is accepted.
        `tags` ({id: corners}) skips the detection (tests). Returns (cal, "") or (None, why)."""
        import copy

        import fieldcam.pipeline as pipeline
        from fieldcam.camera_model import default_intrinsics
        from fieldcam.config import set_field_size
        from fieldcam.pipeline import Perception, _quad_center
        cv2, np = self.cv2, self.np
        h, w = frame.shape[:2]
        cfg = copy.deepcopy(self.cfg)
        set_field_size(cfg, *self._cfg_size)
        sp = Perception(cfg, w, h)
        if tags is None:
            tags = sp.tag_det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        floor = {i: c for i, c in tags.items() if i not in self._robot_tag_ids()}
        sp.tag_det.detect = lambda gray: dict(floor)   # one detection for the whole sweep
        limit, pipeline.CALIBRATION_DRIFT_LIMIT_PX = pipeline.CALIBRATION_DRIFT_LIMIT_PX, 1e9   # judged below
        hfov = None
        try:
            if not sp.has_calibrated_intrinsics:
                best = None
                for angles in ([float(a) for a in range(30, 121, 3)], None):
                    for a in angles or [best[1] + 0.5 * k for k in range(-5, 6)]:
                        sp.K, sp.dist = default_intrinsics(w, h, a)
                        if sp.calibrate_from_corners(frame) and (best is None or sp.cam.reproj_err_px < best[0]):
                            best = (sp.cam.reproj_err_px, a)
                    if best is None:
                        return None, sp.calib_fail or "the corner tags do not fit a camera above the floor"
                hfov = best[1]
                sp.K, sp.dist = default_intrinsics(w, h, hfov)
            if not sp.calibrate_from_corners(frame):
                return None, sp.calib_fail or "the corner tags do not fit a camera above the floor"
        finally:
            pipeline.CALIBRATION_DRIFT_LIMIT_PX = limit
        err = float(sp.cam.reproj_err_px)
        lim = float(self.params.get("eye_cal_max_px", FIT_MAX_PX)) * w / 1920.0
        f = cfg["field"]
        if not err <= lim:
            return None, (f"fit error {err:.1f} px > {lim:.1f} px (eye_cal_max_px): rejected (field {f['length']:.2f} x "
                          f"{f['width']:.2f} m, lens {hfov or 0:.1f} deg)")
        return FieldCal(K=sp.K.copy(), dist=sp.dist.copy(), rvec=sp.cam.rvec.reshape(3, 1).copy(),
                        tvec=sp.cam.tvec.reshape(3, 1).copy(), length=float(f["length"]), width=float(f["width"]),
                        floor_size=float(cfg["tags"]["floor_size"]),
                        tags={int(i): (p["x"], p["y"], p["yaw"]) for i, p in sp.survey["tags"].items()},
                        ref={int(i): _quad_center(np.asarray(c, float)) for i, c in floor.items()
                             if int(i) in sp.survey["tags"]},
                        size=(w, h), hfov=hfov, err=err, how="full fit", saved=_now_iso()), ""

    def _robot_tag_ids(self) -> set[int]:
        return {rc["tag_id"] for rc in self.cfg["robots"].values()}

    def _install(self, cal: FieldCal) -> None:
        """Make `cal` the live calibration (eye thread, or before it starts). fieldcam's own floor-tag drift
        check stays off (floor_layout empty): the eye monitors each tag itself."""
        from fieldcam.config import set_field_size
        perc = self.perc
        perc.K, perc.dist = self.np.array(cal.K, float), self.np.array(cal.dist, float)
        set_field_size(self.cfg, cal.length, cal.width)   # perc.cfg is this same dict
        if not perc._use_camera(cal.camera(), "corners", {}):
            raise ValueError(f"calibration not usable: {perc.calib_fail}")
        perc.floor_layout = {}
        perc.survey = {"ref": min(cal.tags), "tags": {i: {"x": p[0], "y": p[1], "yaw": p[2]} for i, p in cal.tags.items()},
                       "field": {"length": cal.length, "width": cal.width}}
        self._widen_ball_area()
        self.hfov = cal.hfov
        self.cal = cal
        self.calmon["recent"] = {}   # sightings judged against the previous calibration

    def _save_cal(self) -> None:
        if self.cal_path is None or self.cal is None:
            return
        try:
            save_cal(self.cal_path, self.cal)
        except OSError as e:
            self._cal_event(f"could not save {self.cal_path}: {e}")

    def _cal_event(self, text: str) -> None:
        self.calmon["events"].append((round(time.time(), 1), text))
        print(f"[eye] {text}", flush=True)

    def _cal_state(self, state: str, why: str, log: bool = False) -> None:
        """Monitor state: none (no calibration) | unverified (loaded, no floor tag checked yet) | ok | suspect
        (the last pose keeps serving; UI: CAL SUSPECT). Message on change; logged when the state changes (or log)."""
        m = self.calmon
        if (state, why) != (m["state"], m["why"]):
            self.message = f"CAL SUSPECT: {why}" if state == "suspect" else why
        if log or state != m["state"]:
            self._cal_event(f"cal {state}: {why}")
        m["state"], m["why"] = state, why

    def _fit(self, frame, how: str, tags: dict | None = None) -> tuple[bool, str]:
        """Full fit; accepted -> installed, saved, monitor reset. Rejected -> the current calibration stays."""
        t0 = time.perf_counter()
        self._in_cal = True
        try:
            cal, why = self._full_fit(frame, tags)
        finally:
            self._in_cal = False
            self._add("calibrate", t0)
        if cal is None:
            return False, why
        old = self.cal
        cal.how = how
        self._install(cal)
        self._save_cal()
        m = self.calmon
        m.update(since={}, ignored={}, cam_since=None, hist={}, source="live fit")
        msg = (f"calibrated ({how}): field {cal.length:.2f} x {cal.width:.2f} m (config {self._cfg_size[0]:.2f} x "
               f"{self._cfg_size[1]:.2f}), lens {cal.hfov or 0:.1f} deg, fit err {cal.err:.1f} px")
        if old is not None:
            msg += (f" (was {old.length:.2f} x {old.width:.2f} m, lens {old.hfov or 0:.1f} deg)")
        self._cal_state("ok", msg, log=True)
        return True, msg

    def recalibrate(self, timeout: float = REFIT_S, wait: bool = True) -> dict:
        """Operator full re-fit (lens + field size + camera pose) from the four corner tags, on an empty field.
        It can change the field size, so the app calls it only while idle. Runs on the eye thread over the next
        frames for up to `timeout` s; the current calibration stays unless a fit passes eye_cal_max_px.
        Returns {"ok", "message", "cal"}; wait=False returns at once ({"ok": None, "pending": True})."""
        if self.perc is None:
            return {"ok": False, "message": "camera not open yet"}
        req = {"until": time.monotonic() + timeout, "done": threading.Event(), "result": None}
        self._refit = req
        self._next_cal = 0.0
        if not wait:
            return {"ok": None, "pending": True, "message": "re-fit requested"}
        req["done"].wait(timeout + 5.0)
        return req["result"] or {"ok": False, "message": "re-fit did not finish (no frames?)", "cal": self.cal_status()}

    def _cal_check(self, floor: dict, t: float) -> None:
        """Per-tag monitor on the floor tags of one detection ({id: corners}; background scan, ~4 Hz)."""
        cal, m, np = self.cal, self.calmon, self.np
        if cal is None:
            return
        from fieldcam.pipeline import _quad_center
        rec = m.setdefault("recent", {})
        rec.update({i: (t, c) for i, c in floor.items() if i in cal.tags})
        for i in [i for i, (tt, _) in rec.items() if t - tt > RECENT_S]:
            del rec[i]
        floor = {i: c for i, (_, c) in rec.items()}
        obs = {i: _quad_center(np.asarray(c, float)) for i, c in floor.items()}
        if not obs:
            return
        k = cal.size[0] / 1920.0
        lim = float(self.params.get("eye_tag_moved_px", TAG_MOVED_PX)) * k
        cam = self.perc.cam
        shift = {i: float(np.hypot(*(obs[i] - cal.ref[i]))) for i in obs}
        proj = cam.project([[cal.tags[i][0], cal.tags[i][1], 0.0] for i in obs])
        m["resid"] = {i: round(float(np.hypot(*(obs[i] - p))), 2) for i, p in zip(obs, proj)}
        m["shift"], m["checked"] = {i: round(v, 1) for i, v in shift.items()}, t
        for i in [i for i in m["ignored"] if shift.get(i, math.inf) <= lim]:   # back where it was
            del m["ignored"][i]
            m["hist"].pop(i, None)
            self._cal_event(f"tag {i} back in place ({shift[i]:.1f} px)")
        live = [i for i in obs if i not in m["ignored"]]
        sh = sorted(i for i in live if shift[i] > lim)
        ok = sorted(i for i in live if shift[i] <= lim)
        since = m["since"]
        for i in sh:
            since[i] = (since.get(i, (t, t))[0], t)
        for i in list(since):
            if (i in obs and i not in sh) or t - since[i][1] > MOVED_S:
                del since[i]
        px = ", ".join(f"{i}: {shift[i]:.0f}" for i in sh)
        if len(sh) >= 3 or (sh and len(ok) < 2):   # the camera moved (or too few steady tags to tell)
            if m["cam_since"] is None:
                m["cam_since"] = t
            what = "camera moved?" if len(sh) >= 3 else f"only {len(ok)} steady floor tag(s) in view and"
            if t - m["cam_since"] < MOVED_S or not self._resolve_pose(floor, t):
                self._cal_state("suspect", f"{what} tags shifted (px {px}); last pose kept"
                                + (f"; {m['resolve_fail']}" if m.get("resolve_fail") and t - m["cam_since"] >= MOVED_S else ""))
            return
        for i in sh:
            if t - since[i][0] >= MOVED_S:
                m["ignored"][i] = round(shift[i], 1)
                self._cal_event(f"tag {i} moved {shift[i]:.0f} px: ignored, tags {ok} verify the camera")
        if len(ok) >= 2:
            m["cam_since"] = None
            ign = sorted(m["ignored"])
            self._cal_state("ok", f"calibrated: tags {ok} verify ({cal.how}, field {cal.length:.2f} x {cal.width:.2f} m)"
                            + (f"; tag{'s' if len(ign) > 1 else ''} {', '.join(map(str, ign))} moved, ignored" if ign else ""))
            self._readopt(floor, obs, t)

    def _readopt(self, floor: dict, obs: dict, t: float) -> None:
        """An ignored tag that has stayed put for READOPT_S (camera verified by >= 2 other tags) is re-measured
        where it now is, so it can verify / re-solve the camera again. The field size is not touched."""
        m, cal, np = self.calmon, self.cal, self.np
        from fieldcam.tagmeasure import _fit_square
        for i in list(m["ignored"]):
            if i not in obs:
                continue
            h = m["hist"].setdefault(i, collections.deque())
            h.append((t, *obs[i]))
            while h and t - h[0][0] > READOPT_S + 1.0:
                h.popleft()
            pts = np.array([p[1:] for p in h])
            if t - h[0][0] < READOPT_S or np.hypot(*(pts - pts.mean(axis=0)).T).max() > READOPT_STILL_PX * cal.size[0] / 1920:
                continue
            on_floor = self.perc.cam.pixel_to_plane(np.asarray(floor[i], float), 0.0)
            ctr, yaw, _ = _fit_square(on_floor, cal.floor_size)
            cal.tags[i] = (float(ctr[0]), float(ctr[1]), float(yaw))
            cal.ref[i] = pts.mean(axis=0)
            del m["ignored"][i]
            m["hist"].pop(i, None)
            self._save_cal()
            self._cal_event(f"tag {i} re-measured where it now sits ({ctr[0]:+.3f}, {ctr[1]:+.3f}) m")

    def _resolve_pose(self, floor: dict, t: float) -> bool:
        """Camera moved: re-solve the camera pose alone (lens and field frozen) from the floor tags in view at
        their calibrated places. Largest tag set that fits within eye_pose_max_px wins (a tag that also moved
        drops out). Accepted -> installed, saved, references re-based."""
        cal, m, np = self.cal, self.calmon, self.np
        from fieldcam.camera_model import solve_camera_pose
        from fieldcam.tagmeasure import tag_corners
        vis = sorted(i for i in floor if i in cal.tags)
        lim = float(self.params.get("eye_pose_max_px", POSE_MAX_PX)) * cal.size[0] / 1920.0
        if len(vis) < 3:
            m["resolve_fail"] = f"re-solve needs 3 floor tags in view, sees {vis}"
            return False
        old = cal.camera()
        best = None
        for n in range(len(vis), 2, -1):
            for sub in combinations(vis, n):
                cam = solve_camera_pose(cal.K, cal.dist, np.vstack([np.asarray(floor[i], float) for i in sub]),
                                        np.vstack([tag_corners(cal.tags[i], cal.floor_size) for i in sub]))
                if cam is None or abs(cam.center[2] - old.center[2]) > 0.5 * old.center[2]:
                    continue
                if best is None or cam.reproj_err_px < best[0]:
                    best = (cam.reproj_err_px, sub, cam)
            if best is not None and best[0] <= lim:
                break
        if best is None or not best[0] <= lim:
            m["resolve_fail"] = ("no camera pose fits the tags in view" if best is None else
                                 f"pose re-solve rejected: best {best[0]:.1f} px > {lim:.1f} px (eye_pose_max_px)")
            return False
        err, sub, cam = best
        ref = {}
        for i, p in cal.tags.items():   # where the new pose puts each tag, plus its own offset from the model
            xyz = [[p[0], p[1], 0.0]]      # (lens residual) at calibration: not one noisy (or stale) sighting
            ref[i] = cam.project(xyz)[0] + (np.asarray(cal.ref[i], float) - old.project(xyz)[0])
        moved = float(np.linalg.norm(cam.center - old.center))
        self._install(dataclasses.replace(cal, rvec=cam.rvec.reshape(3, 1).copy(), tvec=cam.tvec.reshape(3, 1).copy(),
                                          ref=ref, err=float(err), how="pose re-solve", saved=_now_iso()))
        self._save_cal()
        m.update(since={}, cam_since=None, hist={}, resolve_fail="", resolves=m["resolves"] + 1,
                 ignored={i: m["shift"].get(i, 0.0) for i in vis if i not in sub})
        msg = (f"camera moved: pose re-solved from tags {list(sub)}, err {err:.1f} px, camera {moved * 100:.0f} cm "
               f"from before (lens and field kept)")
        self._cal_state("ok", msg, log=True)
        return True

    def cal_status(self) -> dict:
        """Calibration for the UI: state (none / unverified / ok / suspect: show CAL SUSPECT), why, field, lens,
        per-tag shift from the calibrated place (px), ignored (moved) tags, pose re-solves, recent events."""
        cal, m = self.cal, self.calmon
        return {"state": m["state"], "suspect": m["state"] == "suspect", "why": m["why"], "source": m["source"],
                "path": str(self.cal_path) if self.cal_path else None,
                "how": cal.how if cal else None, "saved": cal.saved if cal else None,
                "field": [round(cal.length, 3), round(cal.width, 3)] if cal else None,
                "lens_deg": cal.hfov if cal else None, "fit_err_px": round(cal.err, 2) if cal else None,
                "tag_shift_px": {str(i): v for i, v in dict(m["shift"]).items()},
                "ignored": sorted(dict(m["ignored"])), "pose_resolves": m["resolves"],
                "refit_pending": self._refit is not None, "events": list(m["events"])[-5:]}

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
        perc = self.perc
        now = time.monotonic()
        mount = self.mount()
        self._apply_floor_size(mount["floor_size"])
        if self.robots_dir is not None and now >= self._next_over:
            self._next_over = now + ROBOT_OVER_CHECK_S
            self._reload_robot_over()
        t_cap = t_recv - (self.params.get("eye_latency_s", 0.12) if self.src.latency else 0.0)
        self._t_frame = t_recv
        req = self._refit
        if req is not None and now >= self._next_cal:   # operator re-fit (app idle): the old calibration stays until one passes
            self._next_cal = now + 0.3
            ok, why = self._fit(frame, "operator re-fit")
            if ok or now >= req["until"]:
                self._refit = None
                if not ok:
                    why = f"re-fit failed, calibration kept: {why}"
                    self._cal_event(why)
                    self.message = why
                req["result"] = {"ok": ok, "message": why, "cal": self.cal_status()}
                req["done"].set()
            else:
                self.message = f"re-fitting: {why}"
        elif not perc.calibration_valid and now >= self._next_cal:   # no calibration yet: the first full fit
            self._next_cal = now + 0.5
            ok, why = self._fit(frame, "full fit")
            if not ok:
                self._cal_state("none", f"not calibrated: {why}")   # the real reason (never overwritten below)
        self._seen = {}
        state = perc.process(frame, t_cap, t_cap, match_only=True)
        self._state = state
        if not state.get("calibration_valid"):
            return None
        robot_ids = self._robot_tag_ids()
        floor = {i: c for i, c in perc.debug.get("tags", {}).items() if i not in robot_ids}
        if floor:
            self._cal_check(floor, t_cap)   # may re-solve the camera pose: perc.cam below is the new one
        cam, tags = perc.cam, perc.debug.get("tags", {})
        blob_on = bool(self.params.get("eye_blob", 1))
        t0 = time.perf_counter()
        for rid, rc in perc.cfg["robots"].items():
            tag = tags.get(rc["tag_id"])
            tr = self._tracks.get(rid)
            mount = self.mount(rid)
            if tag is not None:
                x, y, yaw = self.tag_pose(cam.pixel_to_plane(tag, mount["tag_height"]), mount)
                self._last[rid] = RobotObs(rid, Pose(x, y, yaw), t_cap)
                if tr is not None:
                    self._track_tag(rid, tr, frame, tag, yaw, t_cap, mount, blob_on)
            elif tr is not None and blob_on:
                obs = self._track_blob(rid, rc["tag_id"], tr, frame, t_cap, mount)
                if obs is not None:
                    self._last[rid] = obs
        self._add("blob", t0)
        robots = {k: v for k, v in self._last.items() if t_cap - v.t < KEEP_ROBOT_S}
        ball = None
        px = perc.debug.get("ball_px")
        self._seen = {"ball": px is not None, **{k: v.t == t_cap for k, v in robots.items()}}
        if px is not None:
            bx, by = (float(v) for v in cam.pixel_to_plane([px[:2]], perc.cfg["ball"]["radius"])[0])
            self.ball_raw.append((t_cap, bx, by))
            fx, fy, vx, vy = self.ball_vel.update(t_cap, bx, by, float(self.params.get("ball_decel", 0.0) or 0.0))
            self._ball = Ball(fx, fy, vx, vy, t_cap)
        if self._ball is not None and t_cap - self._ball.t < KEEP_ROBOT_S:
            ball = self._ball
        return World(now, ball, robots)

    # ---- robot board blob (between decoded tags) ------------------------------------------
    def note_cmds(self, t: float, cmds: dict) -> None:
        """Commands sent at monotonic time t ({robot id: Cmd or (vx, vy, wz, ...)}), for the heading
        between decoded tags. Called by the app every control tick (any thread)."""
        with self._cmd_lock:
            for rid, c in cmds.items():
                q = self._cmds.get(rid)
                if q is not None:
                    q.append((t, *((c.vx, c.vy, c.wz) if hasattr(c, "vx") else tuple(c[:3]))))

    def _gyro(self, rid: str) -> list:
        src = self.gyro_source
        if src is None:
            return []
        try:
            return src(rid) or []
        except Exception:  # noqa: BLE001 - a broken source falls back to the commands
            return []

    def _cmd_list(self, rid: str) -> list:
        with self._cmd_lock:
            return list(self._cmds.get(rid, ()))

    def _board_px(self, cam, uv, h: float) -> tuple[float, float]:
        """(expected board area px^2, board side px) at pixel uv for a board at height h."""
        np = self.np
        d = 8.0
        q = cam.pixel_to_plane(np.array([uv, (uv[0] + d, uv[1]), (uv[0], uv[1] + d)], float), h)
        a, b = q[1] - q[0], q[2] - q[0]
        m2 = abs(float(a[0] * b[1] - a[1] * b[0])) / (d * d)   # floor m^2 per pixel
        bm = float(self.params.get("eye_board_m", BOARD_M))
        area = bm * bm / max(m2, 1e-12)
        return area, math.sqrt(area)

    def _board_in(self, frame, colors, pred, exp_area: float, gate_px: float):
        cv2 = self.cv2
        side = math.sqrt(exp_area)
        half = gate_px + 0.9 * side
        h, w = frame.shape[:2]
        x0, y0 = max(0, int(pred[0] - half)), max(0, int(pred[1] - half))
        x1, y1 = min(w, int(pred[0] + half) + 1), min(h, int(pred[1] + half) + 1)
        if x1 - x0 < 8 or y1 - y0 < 8:
            return None
        m = color_mask(cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV), colors)
        k = max(3, int(side * 0.1) | 1)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        return find_board(m, pred, exp_area, gate_px, (x0, y0))

    def _track_tag(self, rid, tr: RobotTrack, frame, tag, yaw, t, mount, blob_on) -> None:
        """Decoded tag: reset the heading; learn where the board centre sits relative to the tag and how
        big it looks, so blob-only fixes land where the tag would have."""
        h = mount["tag_height"]
        cam = self.perc.cam
        c = tag.mean(axis=0)
        ctr = cam.pixel_to_plane([c], h)[0]
        tr.on_tag(t, yaw, (float(ctr[0]), float(ctr[1])), self._cmd_list(rid), self.params)
        colors = self.robot_colors.get(rid)
        b = None
        if blob_on and colors:
            exp, side = self._board_px(cam, c, h)
            b = self._board_in(frame, colors, c, exp * tr.area_k, 0.5 * side)
            if b is not None:
                bxy = cam.pixel_to_plane([b[:2]], h)[0]
                dx, dy = float(ctr[0] - bxy[0]), float(ctr[1] - bxy[1])
                cs, sn = math.cos(yaw), math.sin(yaw)
                ob = (cs * dx + sn * dy, -sn * dx + cs * dy)   # into the body (tag heading) frame
                if math.hypot(*ob) < 0.06:
                    k = max(0.1, 1.0 / (tr.n_off + 1))
                    tr.off = (tr.off[0] + k * (ob[0] - tr.off[0]), tr.off[1] + k * (ob[1] - tr.off[1]))
                    tr.n_off += 1
                    tr.area_k = min(2.0, max(0.5, tr.area_k + 0.1 * (b[2] / exp - tr.area_k)))
        tr.note_px(*((b[0], b[1]) if b is not None else (float(c[0]), float(c[1]))), t)

    def _track_blob(self, rid, tid, tr: RobotTrack, frame, t, mount) -> RobotObs | None:
        """No decoded tag: the board near where the robot was -> position (heading carried)."""
        colors = self.robot_colors.get(rid)
        p = self.params
        if not colors or tr.t_tag is None or tr.px is None or tr.yaw is None:
            return None
        if t - tr.t_tag > float(p.get("eye_blob_max_s", BLOB_MAX_S)):
            tr.n["old"] += 1
            return None
        cam, h = self.perc.cam, mount["tag_height"]
        pred = tr.predict_px(t)
        exp, side = self._board_px(cam, pred, h)
        gate_m = min(BLOB_GATE_M + BLOB_SPEED * (t - tr.px[2]), BLOB_GATE_MAX_M)
        b = self._board_in(frame, colors, pred, exp * tr.area_k, gate_m * side / float(p.get("eye_board_m", BOARD_M)))
        if b is None:
            tr.n["miss"] += 1
            return None
        bx, by = (float(v) for v in cam.pixel_to_plane([b[:2]], h)[0])

        def tag_ctr(yaw):
            cs, sn = math.cos(yaw), math.sin(yaw)
            return bx + cs * tr.off[0] - sn * tr.off[1], by + sn * tr.off[0] + cs * tr.off[1]
        tr.fixes.append((t, *tag_ctr(tr.yaw)))
        yaw = tr.carry_yaw(t, self._cmd_list(rid), p, self._gyro(rid))
        cx, cy = tag_ctr(yaw)
        tr.fixes[-1] = (t, cx, cy)
        ox = mount["tag_offset_x"]
        tr.note_px(b[0], b[1], t)
        tr.n["blob"] += 1
        with self._seed_lock:
            self._blob_seeds[tid] = (b[0], b[1], self._t_frame)
        return RobotObs(rid, Pose(cx - math.cos(yaw) * ox, cy - math.sin(yaw) * ox, yaw), t,
                        src="blob", yaw_t=tr.t_tag, yaw_conf=round(tr.conf, 3))

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
                cam = getattr(self.src, "state", "ok")
                self.message = self.src.message if cam != "ok" else "no frame (camera lost?)"
                self._paused_check()
                with self._lock:
                    self._world = None
                continue
            self._tm = {"wait": time.perf_counter() - t_wait}
            self._after_reconnect()
            if self._same_picture(frame, t_recv):
                continue   # nothing new: the last World keeps its capture times, so it goes stale
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

    def _after_reconnect(self) -> None:
        """The live camera came back (new capture process): the same calibration serves at once (same pinned
        picture size), marked unverified until the floor tags confirm it; a camera put back elsewhere shows as
        shifted tags and its pose is re-solved by the monitor."""
        n = getattr(self.src, "reconnects", 0)
        if n == self._src_reconnects:
            return
        self._src_reconnects = n
        back = getattr(self.src, "message", "camera back")
        if self.cal is not None:
            self._cal_state("unverified", f"{back}; same calibration, waiting for floor tags to verify", log=True)
        else:
            self.message = back

    def _same_picture(self, frame, t_recv: float) -> bool:
        """True if this frame is (almost) the picture before it. A live camera's sensor noise
        differs by ~1 grey level per pixel between frames; the frozen phone feed of Nov 24
        (074158Z) delivered byte-identical frames at full rate. Updates `paused`."""
        np = self.np
        sig = frame[::8, ::8].astype(np.int16)
        prev, self._sig = self._sig, sig
        same = prev is not None and prev.shape == sig.shape and float(np.abs(sig - prev).mean()) < FROZEN_DIFF
        if not same:
            self._t_new = t_recv
        self._paused_check()
        return same

    def _paused_check(self) -> bool:
        """paused = no new picture for FROZEN_S (frozen feed or no frames). Eye thread only; logged
        once per pause."""
        age = time.monotonic() - self._t_new
        was, self.paused = self.paused, self.src is not None and age > FROZEN_S
        if self.paused and not was:
            self.pauses += 1
            print(f"[eye] CAMERA PAUSED: no new picture for {age:.1f} s (frozen feed?)", flush=True)
        elif was and not self.paused:
            print("[eye] camera picture moving again", flush=True)
        return self.paused

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
        hs = [self.mount(r.id)["tag_height"] for r in self.robots] or [self.mount()["tag_height"]]
        th = sum(hs) / len(hs)   # one "tag" plane for drawing: the robots' mean tag height
        key = (id(perc.cam), round(th, 4), self.field.length, self.field.width)
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
                "hfov_deg": self.hfov, "calib_err_px": self._calib_err(),
                "cal": self.cal_status(), "cal_suspect": self.calmon["state"] == "suspect",
                "mount": self.mount(), "mounts": self.mounts(),
                "robots_age": {k: round(now - v.t, 3) for k, v in (w.robots.items() if w else [])},
                "ball_age": round(now - w.ball.t, 3) if w and w.ball else None,
                "paused": self.paused or (self.src is not None and now - self._t_new > FROZEN_S), "picture_age": round(now - self._t_new, 2), "pauses": self.pauses,
                "ball_vel": {"resets": self.ball_vel.resets, "outliers": self.ball_vel.outliers},
                "robots_src": {k: v.src for k, v in (w.robots.items() if w else [])},
                "blob": self.blob_status(),
                "camera": self.src.health() if hasattr(self.src, "health") else None,
                "stage_ms": dict(self.stage_ms)}

    def _calib_err(self) -> float | None:
        """Largest distance (px) between a steady floor tag and where the calibration puts it (last check);
        the fit error until a tag has been checked."""
        m = self.calmon
        r = [v for i, v in dict(m["resid"]).items() if i not in m["ignored"]]
        if r:
            return max(r)
        return round(self.cal.err, 2) if self.cal else (self.perc.calib_err_px if self.perc else None)

    def blob_status(self) -> dict:
        """Per robot: board colour known, fixes by source (tag / blob / board missed / too long since a
        tag), heading confidence and age, learned board offset (cm) and size factor."""
        out = {}
        for rid, tr in self._tracks.items():
            out[rid] = {"colors": rid in self.robot_colors, **tr.n, "yaw_conf": round(tr.conf, 2),
                        "off_cm": [round(v * 100, 1) for v in tr.off], "area_k": round(tr.area_k, 2),
                        "travel_bias_deg": {k: round(math.degrees(v), 1) for k, v in tr.travel_bias.items()}}
        return out

    def ball_track(self, since: float = -math.inf) -> list[tuple[float, float, float]]:
        """Raw ball sightings (t_capture, x, y) newer than `since` (for ctl rollcal)."""
        return [p for p in list(self.ball_raw) if p[0] > since]

    # ---- field size and ball colour (read / set from the app thread) -------------------
    def calibrated_size(self) -> tuple[float, float] | None:
        """(length, width) measured from corner tags 0-3, or None while not calibrated."""
        perc = self.perc
        if perc is None or not perc.calibration_valid or self.synthetic:
            return None
        f = perc.cfg["field"]
        return float(f["length"]), float(f["width"])

    def ball_colors(self) -> list[dict]:
        return [{"lo": [int(x) for x in c["lo"]], "hi": [int(x) for x in c["hi"]]} for c in self.cfg["ball"]["colors"]]

    def set_ball_colors(self, colors: list[dict]) -> None:
        """Apply new HSV ranges live: the detector reads .ranges on every frame."""
        np = self.np
        self.cfg["ball"]["colors"] = colors
        if self.perc is not None:
            self.perc.ball_det.ranges = [(np.array(c["lo"], np.uint8), np.array(c["hi"], np.uint8)) for c in colors]

    def _cal_frame(self):
        frame, _ = self.frame()
        if frame is None:
            raise ValueError("no camera frame yet")
        perc = self.perc
        roi = perc.roi_mask if perc is not None and perc.calibration_valid else None
        return frame.copy(), roi

    def ball_show(self) -> dict:
        """Current ranges and how many ball-coloured patches the detector sees in the field."""
        frame, roi = self._cal_frame()
        det = self.perc.ball_det if self.perc is not None else None
        n = ballcal.count_candidates(det.mask(frame, roi), det.min_area) if det else None
        w = self.latest()
        return {"colors": self.ball_colors(), "candidates": n, "in_field_only": roi is not None,
                "source": self.ball_color_source, "ball_radius": self.cfg["ball"]["radius"],
                "ball_seen": bool(w and w.ball)}

    def ball_calibrate(self, u: float | None = None, v: float | None = None, k: float = ballcal.K_MAD) -> dict:
        """Sample the ball at pixel (u, v) (or find the most saturated compact blob in the
        field), compute HSV ranges and apply them live. Returns ranges, where, candidates."""
        frame, roi = self._cal_frame()
        perc = self.perc
        r_px = float(getattr(perc, "ball_radius_px", 0) or 0) if perc is not None else 0.0
        how = "pixel"
        if u is None or v is None:
            blob = ballcal.find_blob(frame, roi)
            if blob is None:
                raise ValueError("no saturated compact blob found; pass the ball pixel: ballcal X Y")
            u, v, r_px = blob
            how = "auto"
        elif perc is not None and perc.calibration_valid and perc.cam is not None:
            r_px = perc._ball_radius_at(u, v) or r_px
        r_px = r_px or 12.0 * frame.shape[1] / 1920.0
        px = ballcal.sample_disk(frame, u, v, 0.7 * r_px)
        colors = ballcal.hsv_ranges(px, k)
        self.set_ball_colors(colors)
        self.ball_color_source = "ballcal (live)"
        out = self.ball_show()
        out.update({"how": how, "u": round(float(u), 1), "v": round(float(v), 1), "r_px": round(r_px, 1),
                    "n_pixels": int(len(px)), "frame": [int(frame.shape[1]), int(frame.shape[0])]})
        return out

    def robot_show(self, rid: str) -> dict:
        """A robot's board colour and how many board-sized blobs of it the field shows (want 1)."""
        if rid not in self._tracks:
            raise ValueError(f"unknown robot {rid!r} (configured: {sorted(self._tracks)})")
        out = {"robot": rid, "colors": self.robot_colors.get(rid), "source": self.robot_colors_source.get(rid),
               "candidates": None, "fixes": self.blob_status()[rid]}
        if out["colors"] and self.perc is not None and self.perc.calibration_valid:
            frame, roi = self._cal_frame()
            out["candidates"] = self._board_candidates(frame, roi, out["colors"], rid)
        return out

    def _board_candidates(self, frame, roi, colors, rid: str | None = None) -> int:
        cv2 = self.cv2
        m = color_mask(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), colors)
        if roi is not None:
            m &= roi
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        cam, h, n = self.perc.cam, self.mount(rid)["tag_height"], 0
        for c in cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            hull = cv2.convexHull(c)
            a = cv2.contourArea(hull)
            if a < 20:
                continue
            mm = cv2.moments(hull)
            exp, _ = self._board_px(cam, (mm["m10"] / mm["m00"], mm["m01"] / mm["m00"]), h)
            n += BLOB_AREA[0] * exp <= a <= BLOB_AREA[1] * exp
        return n

    def robot_calibrate(self, rid: str, k: float = ballcal.K_MAD) -> dict:
        """Board colour of robot `rid` from the live frame: find its tag (crop around where it was, else the
        whole frame), sample the board ring around it, compute HSV ranges and apply them live. The caller
        saves them (save_robot_colors)."""
        rc = self.cfg["robots"].get(rid)
        if rc is None:
            raise ValueError(f"unknown robot {rid!r} (configured: {sorted(self.cfg['robots'])})")
        frame, _ = self._cal_frame()
        gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
        tid = rc["tag_id"]
        with self._seed_lock:
            seed = self._seeds.get(tid)
        with self._cal_lock:
            tags = self._look(gray, seed[:2], 260 * self._px, {tid}, self._cal_det) if seed else {}
            if tid not in tags:
                h, w = gray.shape
                tags = self._look(gray, (w / 2, h / 2), max(w, h), {tid}, self._cal_det, scales=(1, 2))
        if tid not in tags:
            raise ValueError(f"tag {tid} of robot {rid} not found in the current frame (robot still, in view?)")
        px = board_ring_pixels(frame, tags[tid])
        colors = ballcal.hsv_ranges(px, k)
        self.robot_colors[rid] = colors
        self.robot_colors_source[rid] = "robotcal (live)"
        c = tags[tid].mean(axis=0)
        out = self.robot_show(rid)
        out.update({"tag": tid, "u": round(float(c[0]), 1), "v": round(float(c[1]), 1), "n_pixels": int(len(px)),
                    "frame": [int(frame.shape[1]), int(frame.shape[0])]})
        return out

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
    ap.add_argument("--cal", default=None, help="eye_cal.json to load / save (default: config/eye_cal.json for a live "
                    "camera only; a replay or synthetic run touches none)")
    ap.add_argument("--fast", action="store_true", help="video: every frame inline as fast as possible (scan every "
                    "SCAN_PERIOD_S of video time); prints calibration events and a summary")
    ap.add_argument("--start", type=float, default=0.0, help="--fast: skip this many seconds of video")
    a = ap.parse_args(argv)
    params = {**PARAM_FALLBACKS, **load_params()}
    kw = {"cal_path": Path(a.cal)} if a.cal else {}
    if a.fast:
        return replay(Eye(load_field(), load_robot_config()[0], params, source=a.source, **kw), a.start, a.seconds)
    eye = Eye(load_field(), load_robot_config()[0], params, source=a.source, **kw).start()
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


def replay(eye: Eye, start: float = 0.0, seconds: float = 1e9) -> dict:
    """A recorded video frame by frame (video time drives the monitor's timers): calibration events as they
    happen, then how many frames had a World and in which calibration state."""
    eye._setup()
    cap, fps = eye.src.cap, eye.src.fps
    cap.set(eye.cv2.CAP_PROP_POS_FRAMES, int(start * fps))
    n, next_scan, counts, last = 0, -1.0, collections.Counter(), None
    while n < seconds * fps:
        ok, frame = cap.read()
        if not ok:
            break
        t = start + n / fps
        n += 1
        if eye._tracking() and t >= next_scan:
            next_scan = t + SCAN_PERIOD_S
            eye.scan_once(frame, t, now=t)
        w = eye.process(frame, t)
        st = eye.calmon["state"]
        counts[(st, w is not None)] += 1
        if (st, eye.message) != last:
            print(f"t={t:7.2f}s world={'yes' if w is not None else 'NO '} [{st}] {eye.message}", flush=True)
            last = (st, eye.message)
    total = sum(counts.values())
    with_world = sum(v for (st, ok), v in counts.items() if ok)
    out = {"frames": total, "with_world": with_world, "by_state": {f"{st}{'' if ok else ' (no world)'}": v
                                                                    for (st, ok), v in sorted(counts.items())},
           "pose_resolves": eye.calmon["resolves"], "ignored": sorted(eye.calmon["ignored"]),
           "field": eye.cal_status()["field"]}
    print(f"replay: {total} frames, World in {with_world} ({100 * with_world / max(total, 1):.1f} %), {out}")
    eye.close()
    return out


if __name__ == "__main__":
    main()
