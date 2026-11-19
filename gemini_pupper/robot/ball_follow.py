#!/usr/bin/env python3
"""Ball follower: the Lab 5 follow-me, retargeted from a person to the ball, ending in a leg kick.

Copied from follow_me_pupper (Lab 5):
  * follow_service.py's camera pipeline: keep the newest frame, decode it at a fixed rate,
    remap it to the 220 deg equirectangular image (700 x 572) with fisheye_converter.
  * follow_me.py's state machine: a 10 Hz timer, SEARCH <-> TRACK, steer with yaw against
    the target's offset, walk with approach_speed's law (full speed when far, slower near
    the stop distance, back off when too close), on cmd_vel_mux's /person_following_cmd_vel.

Changed for the ball:
  * Detector: colour blobs of the red/white/blue ball (ball_color from the header-kick work)
    instead of Hailo YOLO, because COCO "sports ball" never fires on this ball. One gore of
    one colour is enough; the white panels next to it complete the ball's box.
  * Target lock: the blob nearest to the last fix (target_lock.pick_target's idea), in metres.
  * Distance: the ball's position from its box and the camera model (ball_geometry from the
    header-kick work), instead of the box's bottom row (STOP_BOTTOM).
  * Target: the ball STAND_X ahead on the kicking paw's line, instead of centred.
  * End: lined up for SETTLE_S, then a timed blind walk over the last ~0.23 m (the camera
    cannot see the ball nearer than about 0.45 m), then the API's kick_in_place tool.

Run it on the Pupper while the Gemini Pupper session runs and the robot stands:

    ball_follow.sh right|left                 # follow, line up, kick
    ball_follow.sh right --dry-run            # follow and line up, no blind walk, no kick
    ball_follow.sh right --measure            # no motion: print what the camera measures

Ctrl-C publishes zero speed and ends it. R3 latches the e-stop as always.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field, replace

import numpy as np

try:
    import cv2
except ImportError:  # desk tests without OpenCV use SciPy for the two image operations
    cv2 = None

PAW_Y = {"right": -0.046, "left": 0.040}  # ball centre y at the kick (leg kick handoff, 2026-11-16)
EQ_WIDTH, EQ_HEIGHT, H_FOV_DEG = 700, 572, 220.0  # the Lab 5 follower's reference frame


# ===================================================================== detector
@dataclass(frozen=True)
class ColorParams:
    # Measured on this robot's frames on 2026-11-18 (fixed white balance 1.4/3.3):
    # ball red H 6-10 S 191-226; carpet H 10-13 S 113-138; ball blue H 112-116 S 157-255;
    # jeans H 104-112 S 59-131. OpenCV 8-bit HSV (H 0..179).
    red_s_min: int = 165
    red_v_min: int = 90
    # The lit top of the ball is overexposed: H 4-17, S 70-161, V 230-255 (2026-11-19). Wood
    # furniture: H 8-22, S 49-132, V up to 245; carpet V <= 158. A narrow bright-orange test:
    bright_hue: tuple[int, int] = (12, 170)  # H <= 12 or H >= 170
    bright_s_min: int = 110
    bright_v_min: int = 180
    red_hue: tuple[int, int] = (13, 165)  # red is H <= 13 or H >= 165
    blue_s_min: int = 140
    blue_v_min: int = 60  # 35 caught the shadowed bottom of the ball and dark blue furniture too
    blue_hue: tuple[int, int] = (98, 130)
    white_s_max: int = 45  # 2026-11-19 daylight: carpet S 59-79 V 120-166; ball white S about 20
    white_v_min: int = 140
    row_min_frac: float = 0.45  # only floor rows (the horizon is at 0.5)
    close_px: int = 9
    min_pixels: int = 40  # coloured pixels in a blob
    min_blob_px: int = 8  # a blob's longer side
    max_aspect: float = 3.0  # a grouped ball is at most about 2:1 (its lit top band alone was 3.5:1;
    #                          a chair seat 3.9:1)
    min_fill: float = 0.12  # coloured pixels / blob box area (panels 0.4; the curved top 0.18; noise 0.08)
    two_colour_share: float = 0.10  # each colour's share for a two-colour ball (ball 14 %; a blue chair 7 %)
    max_width_frac: float = 0.7  # wider blobs are floor (a ball at the nose is 0.6 wide)
    join_gap: float = 0.7  # side by side: gap (x size; a white panel between is up to 0.55) ...
    join_size: float = 2.2  # ... and the group stays this many times the largest blob: one ball
    join_overlap: float = 0.5  # ... and the blob shares this much of its rows or columns with it
    join_gap_v: float = 0.3  # stacked panels: vertical gap (x size); a blob 63 px above the ball joined
    min_size_px: float = 14.0  # ball diameter in the image (1.4 m is about 28 px)
    extent_frac: float = 0.35  # a column is ball when this share of its rows are ball pixels
    extent_max: float = 1.4  # the box may grow to this times the coloured panels' size


@dataclass(frozen=True)
class Candidate:
    box: tuple[float, float, float, float]  # full-ball box (x1, y1, x2, y2), px
    pixels: int
    colours: int  # 1 or 2
    x: float = math.nan  # ball centre in base_link (m), set by locate()
    y: float = math.nan
    z: float = math.nan


def hsv(rgb: np.ndarray) -> np.ndarray:
    """OpenCV-convention 8-bit HSV of an RGB uint8 image (H 0..179), in NumPy."""
    f = rgb.astype(np.float32) / 255.0
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    mx, mn = f.max(axis=-1), f.min(axis=-1)
    d = mx - mn
    h = np.zeros_like(mx)
    m = d > 1e-6
    rr = m & (mx == r)
    gg = m & (mx == g) & ~rr
    bb = m & ~rr & ~gg
    h[rr] = (60.0 * (g - b)[rr] / d[rr]) % 360.0
    h[gg] = 60.0 * (b - r)[gg] / d[gg] + 120.0
    h[bb] = 60.0 * (r - g)[bb] / d[bb] + 240.0
    s = np.where(mx > 0, d / np.maximum(mx, 1e-6), 0.0)
    return np.stack([np.round(h / 2.0) % 180, np.round(s * 255), np.round(mx * 255)], axis=-1)


def _ellipse(k: int) -> np.ndarray:
    r = (k - 1) / 2.0
    yy, xx = np.mgrid[:k, :k]
    return ((xx - r) / (r + 0.5)) ** 2 + ((yy - r) / (r + 0.5)) ** 2 <= 1.0


def _close(mask: np.ndarray, k: int) -> np.ndarray:
    if cv2 is not None:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        return cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel) > 0
    from scipy import ndimage
    se = _ellipse(k)
    return ndimage.binary_erosion(ndimage.binary_dilation(mask, se), se, border_value=1)


def _components(mask: np.ndarray):
    """(label image, [(x, y, w, h, label)]) with 8-connectivity."""
    if cv2 is not None:
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
        return labels, [(*map(int, stats[i][:4]), i) for i in range(1, n)]
    from scipy import ndimage
    labels, n = ndimage.label(mask, structure=np.ones((3, 3), bool))
    out = []
    for i, sl in enumerate(ndimage.find_objects(labels), start=1):
        if sl is not None:
            out.append((sl[1].start, sl[0].start, sl[1].stop - sl[1].start, sl[0].stop - sl[0].start, i))
    return labels, out


def _extent(ball_px: np.ndarray, x1: int, y1: int, x2: int, y2: int, frac: float, max_ratio: float):
    """Extend the coloured panels' box sideways through columns that hold ball pixels
    (coloured or white) in at least `frac` of its rows. Returns (x1, y1, x2). The top is not
    raised: white walls and furniture behind the ball leaked in (2026-11-19)."""
    H, W = ball_px.shape
    limit = int(max_ratio * max(x2 - x1, y2 - y1))
    lo = (y1 + y2) // 2  # lower half only: behind the upper half is the horizon (bright windows)
    col = ball_px[lo:y2].sum(axis=0)
    need = max(2.0, frac * (y2 - lo))
    while x1 > 0 and x2 - x1 < limit and col[x1 - 1] >= need:
        x1 -= 1
    while x2 < W and x2 - x1 < limit and col[x2] >= need:
        x2 += 1
    return x1, y1, x2


def find_candidates(rgb: np.ndarray, p: ColorParams = ColorParams()) -> list[Candidate]:
    """Ball-coloured blobs below the horizon, grouped into balls, each with a full-ball box.

    The ball has orange-red, blue and white panels. White panels often separate the
    coloured ones, so nearby blobs are grouped into one ball, and the box then grows
    through the white panels at its sides and top. The bottom is in shadow.
    """
    row0 = int(rgb.shape[0] * p.row_min_frac)
    sub = hsv(rgb[row0:])
    h, s, v = sub[..., 0], sub[..., 1], sub[..., 2]
    red = (((s >= p.red_s_min) & (v >= p.red_v_min) & ((h <= p.red_hue[0]) | (h >= p.red_hue[1])))
           | ((s >= p.bright_s_min) & (v >= p.bright_v_min) & ((h <= p.bright_hue[0]) | (h >= p.bright_hue[1]))))
    blue = (s >= p.blue_s_min) & (v >= p.blue_v_min) & (h >= p.blue_hue[0]) & (h <= p.blue_hue[1])
    white = (s <= p.white_s_max) & (v >= p.white_v_min)
    labels, comps = _components(_close(red | blue, p.close_px))
    H, W = sub.shape[:2]
    blobs = []  # [x1, y1, x2, y2, n_red, n_blue]
    for x, y, w, hh, i in comps:
        blob = labels[y:y + hh, x:x + w] == i
        n_red = int(np.count_nonzero(red[y:y + hh, x:x + w] & blob))
        n_blue = int(np.count_nonzero(blue[y:y + hh, x:x + w] & blob))
        total = n_red + n_blue
        if (total < p.min_pixels or max(w, hh) < p.min_blob_px or total < p.min_fill * w * hh
                or w > p.max_width_frac * W):
            continue
        blobs.append([x, y, x + w, y + hh, n_red, n_blue])
    # Group blobs of one ball: grow each group from its largest blob. A blob joins when it is
    # close to the group and the whole group stays ball-sized (no chains across the room).
    blobs.sort(key=lambda b: -max(b[2] - b[0], b[3] - b[1]))
    groups = []
    used = [False] * len(blobs)
    for i, seed in enumerate(blobs):
        if used[i]:
            continue
        used[i] = True
        g = list(seed)
        size = max(seed[2] - seed[0], seed[3] - seed[1])
        grew = True
        while grew:
            grew = False
            for j, b in enumerate(blobs):
                if used[j]:
                    continue
                union = (min(g[0], b[0]), min(g[1], b[1]), max(g[2], b[2]), max(g[3], b[3]))
                span = max(union[2] - union[0], union[3] - union[1])
                # Panels of one ball sit side by side (shared rows; a white gore between, up to
                # 0.55 of their size) or stacked (shared columns; 0 to 12 px apart).
                ox = min(g[2], b[2]) - max(g[0], b[0])
                oy = min(g[3], b[3]) - max(g[1], b[1])
                beside = oy >= p.join_overlap * min(g[3] - g[1], b[3] - b[1]) and -ox <= p.join_gap * size
                stacked = ox >= p.join_overlap * min(g[2] - g[0], b[2] - b[0]) and -oy <= p.join_gap_v * size
                if (beside or stacked) and span <= p.join_size * size:
                    used[j], grew = True, True
                    g[:4] = union
                    g[4], g[5] = g[4] + b[4], g[5] + b[5]
        groups.append(g)
    ball_px = red | blue | white
    out = []
    for x, y, x2, y2, n_red, n_blue in groups:
        if max(x2 - x, y2 - y) > p.max_aspect * max(1, min(x2 - x, y2 - y)):
            continue
        x1, y1, x2 = _extent(ball_px, x, y, x2, y2, p.extent_frac, p.extent_max)
        width = float(x2 - x1)
        if max(width, y2 - y1) < p.min_size_px:
            continue
        # The widest row is lit and the bottom is in shadow: the width through the white
        # panels is the diameter, and the box hangs from the top edge. An equirectangular
        # image stretches widths by 1 / cos(latitude), so the ball is width * cos(lat) tall.
        top = float(y1 + row0)
        height = float(y2 - y1)
        for _ in range(2):
            lat = math.radians((top + height / 2) / (rgb.shape[0] - 1) * 180.0 - 90.0)
            height = max(float(y2 - y1), width * math.cos(lat))
        width = max(width, height / max(0.2, math.cos(lat)))
        cx = (x1 + x2) / 2.0
        total = n_red + n_blue
        colours = 2 if min(n_red, n_blue) >= p.two_colour_share * total else 1
        box = (cx - width / 2, top, cx + width / 2, top + height)
        out.append(Candidate(box, total, colours))
    return out


# ===================================================================== geometry
# From ball_geometry.py (header-kick work, mts_123-headerkick final_project/behaviors).
@dataclass(frozen=True)
class CameraMount:
    x: float = 0.15  # m, camera from the base_link origin, forward (estimate)
    y: float = 0.0
    z: float = 0.03
    tilt_up_deg: float = 10.0
    yaw_deg: float = 0.0
    range_scale: float = 1.0  # true distance / measured distance (--measure calibrates it)


def pixel_ray(u: float, v: float, width: int = EQ_WIDTH, height: int = EQ_HEIGHT,
              h_fov_deg: float = H_FOV_DEG) -> np.ndarray:
    """Unit ray (optical frame: x right, y down, z forward) through equirectangular pixel (u, v)."""
    hf = math.radians(h_fov_deg)
    lon = u / (width - 1) * hf - hf / 2
    lat = v / (height - 1) * math.pi - math.pi / 2
    return np.array([math.cos(lat) * math.sin(lon), math.sin(lat), math.cos(lat) * math.cos(lon)])


def ray_pixel(ray: np.ndarray, width: int = EQ_WIDTH, height: int = EQ_HEIGHT,
              h_fov_deg: float = H_FOV_DEG) -> tuple[float, float]:
    x, y, z = ray / np.linalg.norm(ray)
    hf = math.radians(h_fov_deg)
    lon = math.atan2(x, z)
    lat = math.asin(max(-1.0, min(1.0, y)))
    return (lon + hf / 2) / hf * (width - 1), (lat + math.pi / 2) / math.pi * (height - 1)


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    return math.atan2(np.linalg.norm(np.cross(a, b)), float(np.dot(a, b)))


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def locate(box, radius: float, mount: CameraMount = CameraMount()) -> tuple[float, float, float] | None:
    """Ball centre (x forward, y left, z up) from base_link, from a full-ball box."""
    x1, y1, x2, y2 = box
    uc, vc = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    left, right = pixel_ray(x1, vc), pixel_ray(x2, vc)
    top, bottom = pixel_ray(uc, y1), pixel_ray(uc, y2)
    rho = max(_angle(left, right), _angle(top, bottom)) / 2.0
    if not 1e-4 < rho < math.pi / 2:
        return None
    direction = left + right + top + bottom
    p_opt = radius / math.sin(rho) * direction / np.linalg.norm(direction)
    p = np.array([p_opt[2], -p_opt[0], -p_opt[1]]) * mount.range_scale
    p = _rot_z(math.radians(mount.yaw_deg)) @ _rot_y(-math.radians(mount.tilt_up_deg)) @ p
    p = p + np.array([mount.x, mount.y, mount.z])
    return float(p[0]), float(p[1]), float(p[2])


def project(x: float, y: float, z: float, radius: float, mount: CameraMount = CameraMount()):
    """Inverse of locate (desk tests): the full-ball box of a ball centred at (x, y, z)."""
    p = np.array([x, y, z]) - np.array([mount.x, mount.y, mount.z])
    p = _rot_y(math.radians(mount.tilt_up_deg)) @ _rot_z(-math.radians(mount.yaw_deg)) @ p
    p = p / mount.range_scale
    opt = np.array([-p[1], -p[2], p[0]])
    d = float(np.linalg.norm(opt))
    rho = math.asin(min(1.0, radius / d))
    u, v = ray_pixel(opt)
    deg_px = (EQ_WIDTH - 1) / H_FOV_DEG
    half = math.degrees(rho) * deg_px
    # Horizontal half-width grows with latitude on an equirectangular image.
    lat = math.radians((v / (EQ_HEIGHT - 1)) * 180.0 - 90.0)
    half_u = half / max(0.2, math.cos(lat))
    return (u - half_u, v - half, u + half_u, v + half)


@dataclass(frozen=True)
class Fix:
    t: float
    x: float
    y: float
    z: float
    colours: int
    size_px: float
    pixels: int = 0


@dataclass
class Lock:
    """The ball we follow: the candidate nearest to the last fix (Lab 5 target_lock, in metres)."""
    last: Fix | None = None


@dataclass(frozen=True)
class LockParams:
    gate_m: float = 0.15  # a candidate this far from the last fix is not our ball
    hold_s: float = 1.0  # the lock forgets the ball after this long without a fix
    min_range: float = 0.20  # nearer fixes still count: TRACK backs off
    max_range: float = 1.8
    z_range: tuple[float, float] = (-0.26, 0.03)  # ball centre height from base_link; robot frames: -0.09 to -0.19


def pick(cands: list[Candidate], lock: Lock, now: float, radius: float, mount: CameraMount,
         lp: LockParams = LockParams()) -> Fix | None:
    """This frame's fix of our ball, or None. Updates the lock."""
    fixes = []
    for c in cands:
        pos = locate(c.box, radius, mount)
        if pos is None:
            continue
        x, y, z = pos
        rng = math.hypot(x, y)
        if not (lp.min_range <= rng <= lp.max_range and lp.z_range[0] <= z <= lp.z_range[1]):
            continue
        fixes.append(Fix(now, x, y, z, c.colours, c.box[2] - c.box[0], c.pixels))
    if lock.last is not None and now - lock.last.t > lp.hold_s:
        lock.last = None
    two = [f for f in fixes if f.colours == 2]
    if lock.last is not None:
        near = [f for f in fixes if math.hypot(f.x - lock.last.x, f.y - lock.last.y) <= lp.gate_m]
        if not near:
            return None
        best = min(near, key=lambda f: math.hypot(f.x - lock.last.x, f.y - lock.last.y))
    else:
        # A lock starts only on a two-colour ball (a blue chair was taken for the ball on
        # 2026-11-19); one colour can only continue a lock, inside the gate.
        if not two:
            return None
        best = max(two, key=lambda f: f.pixels)
    lock.last = best
    return best


# ===================================================================== state machine
@dataclass(frozen=True)
class FollowParams:
    side: str = "right"
    stand_x: float = 0.30  # m: ball centre at the line-up; this detector reads 0.27 m steadily and
    #                        sees the ball at the nose (0.23 m). The blind walk is then about 5 cm
    tol_x: float = 0.03
    tol_y: float = 0.012  # sideways; simulated kicks: sd 0.9 cm, max 1.9 cm (2 cm tolerance: max 3.5 cm)
    quiet_s: float = 0.6  # after the last turn or step: the walker's delay and lag still move it
    settle_s: float = 0.8  # then average the fixes of this window (at least settle_n of them) ...
    settle_max_s: float = 2.0  # ... and extend it to this when the fixes are noisy
    settle_n: int = 4
    settle_se: float = 0.004  # m: standard error of the mean allowed for x and y
    settle_drift: float = 0.006  # m: the window's two halves must agree (beyond their noise)
    kp_yaw: float = 1.2  # rad/s per rad of bearing error to the paw line (Lab 5 KP, in radians)
    max_yaw: float = 0.7  # rad/s
    min_yaw: float = 0.08  # rad/s; it grows while the distance is right and the side is not
    yaw_ramp: float = 0.04  # rad/s per s while the side error does not shrink (a walker that
    #                         ignores small turns); halved at each overshoot
    loose: tuple = ((8.0, 0.02),)  # (s at the right distance, accepted side error)
    max_fwd: float = 0.20  # m/s (TRACK_FORWARD_VEL)
    max_back: float = 0.12  # m/s (MAX_BACKUP_VEL)
    min_speed: float = 0.06  # m/s: the walker does not walk below about this
    slowdown: float = 0.25  # m of distance error at which the speed reaches max_fwd (SLOWDOWN_PX)
    turn_first: float = 0.35  # rad: above this bearing error, only turn
    filter_s: float = 0.4  # s: median of the fixes of this window (one jumping frame cannot move it)
    timeout: float = 1.0  # s without a fix: SEARCH (Lab 5 TIMEOUT)
    search_yaw: float = 0.6  # rad/s (Lab 5 SEARCH_YAW_VEL 1.0; slower for the 12 Hz detector)
    search_s: float = 13.0  # s of one search turn (about a full turn)
    backup_m: float = 0.40  # m to back up after a search turn finds nothing
    max_backups: int = 2
    creep_v: float = 0.20  # m/s for the blind walk
    fwd_gain: float = 1.31  # walker distance / commanded distance at 0.2 m/s (robot, 2026-11-18)
    dead_s: float = 0.14  # walker start delay (robot, 2026-11-18)
    contact_x: float = 0.258  # ball centre at the kick (leg kick handoff)
    margin: float = -0.01  # m short of contact to stop: -0.01 aims at the chest (0.248 m). A long
    #                        walk pushes the ball and still kicks it; a short one can miss it
    pause_s: float = 0.4  # zero speed after the blind walk, before the kick
    max_time_s: float = 150.0
    stale_s: float = 3.0  # no camera frames this long: abort

    @property
    def paw_y(self) -> float:
        return PAW_Y[self.side]


SEARCH, TRACK, SETTLE, CREEP, BACKUP, KICK, DONE, ABORT = (
    "SEARCH", "TRACK", "SETTLE", "CREEP", "BACKUP", "KICK", "DONE", "ABORT")


@dataclass
class Command:
    vx: float = 0.0
    wz: float = 0.0
    publish: bool = True  # False in KICK/DONE/ABORT: leave the mux to the API


def _clip(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class BallFollower:
    """follow_me.py's StateMachineNode without ROS: tick(now) at 10 Hz, see(fix) per frame."""

    def __init__(self, p: FollowParams, dry_run: bool = False, log=print):
        if p.side not in PAW_Y:
            raise ValueError("side must be right or left")
        self.p, self.dry_run, self.log = p, dry_run, log
        self.state = SEARCH
        self.fixes: list[Fix] = []
        self.last_frame: float | None = None
        self.started: float | None = None
        self.phase_start = 0.0
        self.phase_until = 0.0
        self.backups = 0
        self.search_dir = 1.0
        self.creep_x = math.nan
        self.creep_s = 0.0
        self.near_since: float | None = None
        self.ramp, self.ramp_sign, self.ramp_best = 0.0, 0.0, math.inf
        self.settle_tol_y = p.tol_y
        self.result = ""

    # ----------------------------------------------------------------- inputs
    def frame(self, now: float, fix: Fix | None) -> None:
        """One processed camera frame: the ball fix or None."""
        self.last_frame = now
        if fix is not None:
            self.fixes.append(fix)
            self.fixes = [f for f in self.fixes if now - f.t <= 2.0]

    def estimate(self, now: float) -> Fix | None:
        recent = [f for f in self.fixes if now - f.t <= self.p.filter_s]
        if not recent:
            last = self.fixes[-1] if self.fixes else None
            return last if last is not None and now - last.t <= self.p.timeout else None
        return Fix(now, float(np.median([f.x for f in recent])), float(np.median([f.y for f in recent])),
                   float(np.median([f.z for f in recent])), max(f.colours for f in recent), recent[-1].size_px)

    # ----------------------------------------------------------------- one tick
    def tick(self, now: float) -> Command:
        p = self.p
        if self.started is None:
            self.started = now
        if self.state in (CREEP, KICK, DONE, ABORT):
            return Command(publish=False)  # the main loop drives the blind walk and the kick
        if now - self.started > p.max_time_s:
            return self.abort(f"gave up after {p.max_time_s:.0f} s")
        if self.last_frame is None or now - self.last_frame > 1.0:
            if self.last_frame is not None and now - self.last_frame > p.stale_s:
                return self.abort("camera frames stopped")
            return Command()  # Lab 5: a stopped camera holds still
        if self.state == BACKUP:
            return self.timed(now)
        est = self.estimate(now)
        if est is None:
            return self.search(now)
        if self.state == SEARCH:
            self.log(f"ball at x {est.x:.2f} y {est.y:+.2f}: TRACK")
        self.search_dir = 1.0 if est.y >= 0 else -1.0
        ex, ey = est.x - p.stand_x, est.y - p.paw_y
        # Time at the right distance (hysteresis), not reset by sideways overshoots.
        if self.near_since is None and abs(ex) <= p.tol_x:
            self.near_since = now
        elif self.near_since is not None and abs(ex) > 1.5 * p.tol_x:
            self.near_since = None
        near = 0.0 if self.near_since is None else now - self.near_since
        tol_y = p.tol_y
        for after, loose in p.loose:
            if near >= after:
                tol_y = loose
        lined = abs(ex) <= p.tol_x and abs(ey) <= tol_y
        if self.state == SETTLE:
            if abs(ex) > 1.5 * p.tol_x or abs(ey) > 1.5 * self.settle_tol_y:
                self.log(f"line-up lost (x {est.x:.3f} y {est.y:+.3f}): TRACK")
                self.state = TRACK
            else:
                return self.settle(now)
        if lined:
            self.state, self.phase_start, self.settle_tol_y = SETTLE, now, tol_y
            self.log(f"lined up: ball x {est.x:.3f} y {est.y:+.3f}: SETTLE")
            return Command()
        self.state = TRACK
        if near > 0 and abs(ey) > p.tol_y:
            sign = math.copysign(1.0, ey)
            if sign != self.ramp_sign:  # overshoot: halve the extra yaw, start a new reference
                self.ramp, self.ramp_sign, self.ramp_best = 0.5 * self.ramp, sign, abs(ey)
            elif abs(ey) < self.ramp_best - 0.003:  # still closing in: keep the extra yaw
                self.ramp_best = abs(ey)
            else:  # stuck: the walker ignores this yaw rate
                self.ramp += p.yaw_ramp * 0.1
        elif near == 0:
            self.ramp, self.ramp_sign, self.ramp_best = 0.0, 0.0, math.inf
        return self.track(ex, ey, est.x, min(p.min_yaw + self.ramp, 0.5 * p.max_yaw))

    def track(self, ex: float, ey: float, x: float, min_yaw: float | None = None) -> Command:
        """Lab 5 motion_command: yaw against the offset, approach_speed for distance."""
        p = self.p
        bearing = math.atan2(ey, max(x, 0.1))  # turning by this puts the ball on the paw line
        wz = _clip(p.kp_yaw * bearing, -p.max_yaw, p.max_yaw)
        min_yaw = p.min_yaw if min_yaw is None else min(min_yaw, p.max_yaw)
        if abs(ey) <= p.tol_y:
            wz = 0.0
        elif abs(wz) < min_yaw:
            wz = math.copysign(min_yaw, wz)
        if abs(ex) <= p.tol_x:
            vx = 0.0
        elif ex > 0:
            vx = max(p.min_speed, min(p.max_fwd, p.max_fwd * ex / p.slowdown))
        else:
            vx = min(-p.min_speed, max(-p.max_back, p.max_fwd * ex / p.slowdown))
        if abs(bearing) > p.turn_first:
            vx = 0.0
        return Command(vx, wz)

    def settle(self, now: float) -> Command:
        p = self.p
        start = self.phase_start + p.quiet_s
        if now - start < p.settle_s:
            return Command()
        window = [f for f in self.fixes if f.t >= start]
        if len(window) < p.settle_n:
            if now - start > p.settle_max_s:
                self.log("settle: too few fixes: TRACK")
                self.state = TRACK
            return Command()
        n, half = len(window), len(window) // 2
        xs, ys = np.array([f.x for f in window]), np.array([f.y for f in window])
        mx, my = float(np.median(xs)), float(np.median(ys))
        # Robust spread (MAD), so one jumping frame does not fail the settle.
        sd = max(float(np.median(np.abs(xs - mx))), float(np.median(np.abs(ys - my)))) * 1.4826
        se = sd / math.sqrt(n)
        # Drift beyond the noise: the robot still moves.
        drift_tol = min(0.012, max(p.settle_drift, 5.0 * se))  # 2.5 sd of a difference of half-medians
        drift = max(abs(np.median(xs[:half]) - np.median(xs[half:])), abs(np.median(ys[:half]) - np.median(ys[half:])))
        noisy = se > p.settle_se
        if noisy and now - start < p.settle_max_s:
            return Command()  # collect more fixes
        if (abs(mx - p.stand_x) > p.tol_x or abs(my - p.paw_y) > self.settle_tol_y or noisy
                or drift > drift_tol):
            self.log(f"settle failed (x {mx:.3f} y {my:+.3f}, se {se:.3f}, drift {drift:.3f}): TRACK")
            self.state = TRACK
            return Command()
        if self.dry_run:
            self.state, self.result = DONE, f"dry run: lined up, ball x {mx:.3f} y {my:+.3f} ({len(window)} fixes)"
            self.log(self.result)
            return Command()
        self.creep_x = mx
        distance = max(0.0, mx - p.contact_x - p.margin)
        self.creep_s = p.dead_s + distance / (p.fwd_gain * p.creep_v)
        self.state = CREEP
        self.log(f"ball x {mx:.3f} y {my:+.3f} ({len(window)} fixes): blind walk {distance:.3f} m "
                 f"({self.creep_s:.2f} s at {p.creep_v} m/s), then kick with the {p.side} leg")
        return Command(publish=False)

    def timed(self, now: float) -> Command:
        p = self.p
        if now < self.phase_until:
            return Command(-p.max_back, 0.0)
        # BACKUP done: search again from here
        self.fixes.clear()
        self.state, self.phase_start = SEARCH, now
        return Command()

    def search(self, now: float) -> Command:
        p = self.p
        if self.state != SEARCH:
            # No back-up when the ball is lost close ahead: this detector sees the ball at the
            # nose, and that rule walked backward 7 times with a chair locked (2026-11-19).
            self.state, self.phase_start = SEARCH, now
            self.log("no ball: SEARCH")
        if now - self.phase_start > p.search_s:
            return self.backup(now, "no ball after a search turn")
        return Command(0.0, self.search_dir * p.search_yaw)

    def backup(self, now: float, why: str) -> Command:
        p = self.p
        if self.backups >= p.max_backups:
            return self.abort(f"ball not found ({why}; backed up {self.backups} times)")
        self.backups += 1
        seconds = p.dead_s + p.backup_m / (p.fwd_gain * p.max_back)
        self.state, self.phase_until = BACKUP, now + seconds
        self.log(f"{why}: back up {p.backup_m:.2f} m")
        return Command(-p.max_back, 0.0)

    def abort(self, why: str) -> Command:
        self.state, self.result = ABORT, why
        self.log("ABORT: " + why)
        return Command()


# ===================================================================== ROS node
class FollowNode:
    """The ROS side (follow_service.py's pipeline): camera in, Twist out at 10 Hz."""

    def __init__(self, args, follower: BallFollower | None):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from sensor_msgs.msg import CompressedImage

        sys.path.insert(0, args.follow_dir)
        from fisheye_converter import equirectangular_maps, load_camera_model

        self.Twist, self.args, self.follower = Twist, args, follower
        self.node = Node("ball_follower")
        self.pub = self.node.create_publisher(Twist, args.cmd_topic, 10)
        self.maps_fn, self.load_model = equirectangular_maps, load_camera_model
        self.camera_yaml = os.path.join(args.follow_dir, "camera_params.yaml")
        self.model = None
        self.mount = CameraMount(range_scale=args.range_scale)
        self.cp = ColorParams()
        self.lock = Lock()
        self.latest = None
        self.ready = threading.Event()
        self.running = True
        self.mutex = threading.Lock()
        self.frames = 0
        self.last_print = self.last_snapshot = 0.0
        self.node.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.on_image, 1)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        if follower is not None:
            self.node.create_timer(0.1, self.on_tick)

    def on_image(self, msg) -> None:
        self.latest = msg
        self.ready.set()

    def run(self) -> None:
        period = 1.0 / self.args.rate
        while self.running:
            if not self.ready.wait(timeout=0.5):
                continue
            started = time.monotonic()
            self.ready.clear()
            msg, self.latest = self.latest, None
            if msg is not None:
                try:
                    self.process(msg)
                except Exception as exc:  # a bad frame: try the next one
                    self.node.get_logger().error(f"detection failed: {exc}")
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def equirect(self, bgr: np.ndarray) -> np.ndarray:
        h, w = bgr.shape[:2]
        if self.model is None or (self.model.width, self.model.height) != (w, h):
            model = self.load_model(self.camera_yaml, w, h)
            # Half-size decode: pixel i of the half frame covers full pixels 2i and 2i+1.
            model.fx, model.fy = model.fx * 0.5, model.fy * 0.5
            model.cx, model.cy = (model.cx + 0.5) * 0.5 - 0.5, (model.cy + 0.5) * 0.5 - 0.5
            self.model = model
        map_x, map_y, invalid = self.maps_fn(self.model, EQ_WIDTH, EQ_HEIGHT, H_FOV_DEG)
        r = int(EQ_HEIGHT * self.cp.row_min_frac)
        eq = np.zeros((EQ_HEIGHT, EQ_WIDTH, 3), np.uint8)
        eq[r:] = cv2.remap(bgr, map_x[r:], map_y[r:], interpolation=cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
        eq[r:][invalid[r:]] = 0
        return cv2.cvtColor(eq, cv2.COLOR_BGR2RGB)

    def process(self, msg) -> None:
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        if bgr is None:
            return
        rgb = self.equirect(bgr)
        now = time.monotonic()
        cands = find_candidates(rgb, self.cp)
        with self.mutex:
            fix = pick(cands, self.lock, now, self.args.radius, self.mount)
            if self.follower is not None:
                self.follower.frame(now, fix)
        self.frames += 1
        if self.args.snapshot and now - self.last_snapshot > 2.0:
            self.last_snapshot = now
            img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for c in cands:
                x1, y1, x2, y2 = (int(round(v)) for v in c.box)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 1)
            if fix is not None:
                cv2.putText(img, f"x {fix.x:.2f} y {fix.y:+.2f} z {fix.z:+.2f}", (5, 300),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imwrite(self.args.snapshot, img)
        if now - self.last_print > 0.25:  # every fix at 4 Hz (also in follow mode, for the log)
            self.last_print = now
            if fix is None:
                print(f"{time.strftime('%H:%M:%S')}   no ball ({len(cands)} blobs)", flush=True)
            else:
                print(f"{time.strftime('%H:%M:%S')}   ball x {fix.x:.3f} y {fix.y:+.3f} z {fix.z:+.3f} "
                      f"size {fix.size_px:.0f} px colours {fix.colours} ({len(cands)} blobs)", flush=True)

    def on_tick(self) -> None:
        with self.mutex:
            cmd = self.follower.tick(time.monotonic())
        if cmd.publish:
            self.send(cmd.vx, cmd.wz)

    def send(self, vx: float, wz: float) -> None:
        t = self.Twist()
        t.linear.x, t.angular.z = float(vx), float(wz)
        self.pub.publish(t)

    def close(self) -> None:
        self.running = False
        self.ready.set()
        self.worker.join(timeout=2.0)


def drive(node: FollowNode, vx: float, seconds: float) -> None:
    """Publish vx every 0.05 s for exactly `seconds`, then zero."""
    end = time.monotonic() + seconds
    while True:
        left = end - time.monotonic()
        if left <= 0:
            break
        node.send(vx, 0.0)
        time.sleep(min(0.05, left))
    node.send(0.0, 0.0)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("side", choices=["right", "left"])
    ap.add_argument("--dry-run", action="store_true", help="follow and line up; no blind walk, no kick")
    ap.add_argument("--measure", action="store_true", help="no motion: print the ball fixes")
    ap.add_argument("--follow-dir", default=os.environ.get("FOLLOW_ME_DIR", ""), help="Lab 5 follow_me_pupper/robot")
    ap.add_argument("--url", default="http://127.0.0.1:18090", help="Gemini Pupper API (for kick_in_place)")
    ap.add_argument("--cmd-topic", default="/person_following_cmd_vel")
    ap.add_argument("--radius", type=float, default=0.097)
    ap.add_argument("--range-scale", type=float, default=1.0)
    ap.add_argument("--stand", type=float, default=FollowParams.stand_x)
    ap.add_argument("--fwd-gain", type=float, default=FollowParams.fwd_gain)
    ap.add_argument("--rate", type=float, default=12.0, help="detector frames per second")
    ap.add_argument("--snapshot", default="", help="save the annotated detector image here every 2 s")
    args = ap.parse_args(argv)
    if not args.follow_dir or not os.path.isfile(os.path.join(args.follow_dir, "fisheye_converter.py")):
        ap.error("--follow-dir must be the Lab 5 follow_me_pupper/robot folder")

    import signal

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):  # Lab 5 main(): Ctrl-C and SIGTERM end with zero speed
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupt)
    stamp = lambda text: print(time.strftime("%H:%M:%S"), text, flush=True)
    follower = None
    if not args.measure:
        follower = BallFollower(FollowParams(side=args.side, stand_x=args.stand, fwd_gain=args.fwd_gain),
                                dry_run=args.dry_run, log=stamp)
    node = FollowNode(args, follower)
    code = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node.node, timeout_sec=0.05)
            if follower is not None and follower.state == CREEP:
                drive(node, follower.p.creep_v, follower.creep_s)  # exact timing, not the 10 Hz tick
                drive(node, 0.0, follower.p.pause_s)
                follower.state = KICK
            if follower is not None and follower.state == KICK:
                node.send(0.0, 0.0)
                sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
                from ball_seek import Abort, HttpClient
                client = HttpClient(args.url)
                try:
                    stamp(client.kick(args.side))
                    follower.state, follower.result = DONE, f"kicked with the {args.side} leg"
                except Abort as error:
                    follower.state, follower.result = ABORT, f"kick refused: {error}"
                    client.stop()
            if follower is not None and follower.state in (DONE, ABORT):
                stamp(("DONE: " if follower.state == DONE else "ABORT: ") + follower.result)
                code = 0 if follower.state == DONE else 1
                break
    except KeyboardInterrupt:
        stamp("STOPPED (Ctrl-C)")
        code = 130
        if follower is not None and follower.state == KICK:  # the kick job runs in the API
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from ball_seek import HttpClient
            HttpClient(args.url).stop()
    finally:
        for _ in range(0 if args.measure else 3):  # zero speed: the mux holds the last command 0.5 s
            node.send(0.0, 0.0)
            time.sleep(0.03)
        node.close()
        node.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
