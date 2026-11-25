"""Ball colour calibration: pick the ball's pixels, turn them into OpenCV HSV ranges.

Pure functions (numpy + cv2) so they test without a camera. The Eye applies the result live
and the app saves it to config/ball_color.json, which the Eye loads at startup (it overrides
fieldcam's default pink range in overhead_perception/config/field.yaml).

OpenCV HSV: H 0..179 (a circle: 179 is next to 0), S and V 0..255.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np

K_MAD = 3.0          # range = median +- K_MAD * 1.4826 * MAD (MAD scaled to a std-dev estimate)
MIN_HALF = (6, 40, 50)   # never narrower than this either side of the median (H, S, V)
S_FLOOR, V_FLOOR = 50, 40  # never accept greyer / darker than this (carpet, shadows)


def _robust(x: np.ndarray, k: float, min_half: float) -> tuple[float, float]:
    med = float(np.median(x))
    half = max(min_half, k * 1.4826 * float(np.median(np.abs(x - med))))
    return med, half


def hsv_ranges(hsv_pixels: np.ndarray, k: float = K_MAD) -> list[dict]:
    """[{lo: [h, s, v], hi: [h, s, v]}, ...] covering median +- k*MAD of the pixels.

    Hue is circular: the pixels are rotated so their circular mean sits at 90 before the median
    is taken, and a range crossing 0/179 comes back as two ranges. S and V get no upper limit
    (a brighter / more saturated pixel of the same hue is still the ball)."""
    px = np.asarray(hsv_pixels, float).reshape(-1, 3)
    if len(px) < 10:
        raise ValueError(f"only {len(px)} pixels to calibrate from (need >= 10)")
    ang = px[:, 0] * (2 * math.pi / 180.0)
    mean_h = (math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean())) / 2.0) % 180.0
    h_rot = (px[:, 0] - mean_h + 90.0) % 180.0
    hm, hh = _robust(h_rot, k, MIN_HALF[0])
    hh = min(hh, 45.0)                      # a ball is one colour family, not half the wheel
    sm, sh = _robust(px[:, 1], k, MIN_HALF[1])
    vm, vh = _robust(px[:, 2], k, MIN_HALF[2])
    s_lo = int(max(S_FLOOR, min(sm - sh, 230)))
    v_lo = int(max(V_FLOOR, min(vm - vh, 230)))
    lo_h = hm - hh + mean_h - 90.0
    hi_h = hm + hh + mean_h - 90.0
    lo_h, hi_h = math.floor(lo_h), math.ceil(hi_h)
    if lo_h < 0:
        return [{"lo": [0, s_lo, v_lo], "hi": [int(hi_h), 255, 255]},
                {"lo": [int(lo_h + 180), s_lo, v_lo], "hi": [179, 255, 255]}]
    if hi_h > 179:
        return [{"lo": [int(lo_h), s_lo, v_lo], "hi": [179, 255, 255]},
                {"lo": [0, s_lo, v_lo], "hi": [int(hi_h - 180), 255, 255]}]
    return [{"lo": [int(lo_h), s_lo, v_lo], "hi": [int(hi_h), 255, 255]}]


def sample_disk(bgr: np.ndarray, u: float, v: float, r: float) -> np.ndarray:
    """HSV pixels inside a disk of radius r px around (u, v), keeping the more saturated half
    (so a slightly-off click or a too-big radius does not pull carpet into the range)."""
    h, w = bgr.shape[:2]
    r = max(3.0, r)
    x0, x1 = max(0, int(u - r)), min(w, int(u + r) + 1)
    y0, y1 = max(0, int(v - r)), min(h, int(v + r) + 1)
    if x1 <= x0 or y1 <= y0:
        raise ValueError(f"pixel ({u:.0f}, {v:.0f}) is outside the {w}x{h} frame")
    hsv = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    px = hsv[(xx - u) ** 2 + (yy - v) ** 2 <= r * r].astype(float)
    if len(px) >= 20:
        px = px[px[:, 1] >= np.median(px[:, 1])]
    return px


def find_blob(bgr: np.ndarray, roi_mask: np.ndarray | None = None, min_area: float = 30.0,
              max_area: float | None = None) -> tuple[float, float, float] | None:
    """(u, v, radius px) of the most colourful compact blob (inside roi_mask if given), or None.

    Colourful = far from the region's median colour in Lab a/b (so a saturated carpet does not
    count, only things that stand out from it), and not dark. Compact = fills >= 50 % of its
    enclosing circle. Score = mean colour distance of the blob x its saturation."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    region = roi_mask > 0 if roi_mask is not None else np.ones(hsv.shape[:2], bool)
    if not region.any():
        return None
    am, bm = float(np.median(lab[:, :, 1][region])), float(np.median(lab[:, :, 2][region]))
    d = np.hypot(lab[:, :, 1] - am, lab[:, :, 2] - bm)
    thr = max(20.0, float(np.percentile(d[region], 99.5)) * 0.6)
    m = ((d >= thr) & (hsv[:, :, 2] >= 50) & (hsv[:, :, 1] >= 60) & region).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    max_area = max_area or 0.02 * m.size
    best = None
    for c in cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        a = cv2.contourArea(c)
        if not min_area <= a <= max_area:
            continue
        (u, v), r = cv2.minEnclosingCircle(c)
        if a < 0.5 * math.pi * r * r:
            continue
        blob = np.zeros(m.shape, np.uint8)
        cv2.drawContours(blob, [c], -1, 255, -1)
        sel = blob > 0
        score = float(d[sel].mean()) * float(hsv[:, :, 1][sel].mean())
        if best is None or score > best[0]:
            best = (score, (float(u), float(v), float(r)))
    return None if best is None else best[1]


def count_candidates(mask: np.ndarray, min_area: float = 30.0) -> int:
    """Connected ball-coloured patches at least min_area px big in a detector mask."""
    n, _, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8))
    return int(sum(1 for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= min_area))


def load(path: Path) -> list[dict] | None:
    """Colour ranges from a ball_color.json, or None if missing / unreadable / malformed."""
    try:
        cols = json.loads(Path(path).read_text())["colors"]
        ok = [{"lo": [int(x) for x in c["lo"]], "hi": [int(x) for x in c["hi"]]} for c in cols]
        return ok if ok and all(len(c["lo"]) == 3 and len(c["hi"]) == 3 for c in ok) else None
    except (OSError, ValueError, KeyError, TypeError):
        return None


def save(path: Path, colors: list[dict], **meta) -> None:
    Path(path).write_text(json.dumps({"colors": colors, **meta}, indent=1) + "\n")
