"""Set the ball's colour by pointing at it: pixels under the cursor -> HSV ranges for ball.colors.

OpenCV HSV: hue 0-179 (a circle: 179 is next to 0), saturation and value 0-255.
"""
from __future__ import annotations

import cv2
import numpy as np


def suggest_ranges(bgr: np.ndarray, u: float, v: float, radius: int = 10) -> list[dict]:
    """HSV range(s) covering the colour in a small disc around pixel (u, v).

    Returns one {lo, hi} range, or two when the hue straddles the 0/179 wrap
    (reds). Tolerances are deliberately generous: the ball's exact outline comes
    from its edge, the colour only has to say roughly where it is.
    """
    h, w = bgr.shape[:2]
    x0, x1 = int(max(0, u - radius)), int(min(w, u + radius + 1))
    y0, y1 = int(max(0, v - radius)), int(min(h, v + radius + 1))
    if x1 <= x0 or y1 <= y0:
        raise ValueError("cursor is outside the picture")
    yy, xx = np.mgrid[y0:y1, x0:x1]
    disc = (xx - u) ** 2 + (yy - v) ** 2 <= radius ** 2
    px = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)[disc].astype(np.float64)
    if len(px) == 0:
        raise ValueError("no pixels under the cursor")
    # Hue is circular: average it as an angle, then measure the spread around that.
    ang = px[:, 0] * (2 * np.pi / 180.0)
    centre = np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())
    spread = np.abs(np.angle(np.exp(1j * (ang - centre)))) * (180.0 / (2 * np.pi))  # in hue units
    hc = (centre * 180.0 / (2 * np.pi)) % 180.0
    half = float(np.clip(np.percentile(spread, 95) + 8.0, 10.0, 22.0))
    s_lo = int(max(40, np.percentile(px[:, 1], 10) - 45))
    v_lo = int(max(40, np.percentile(px[:, 2], 10) - 60))
    lo_h, hi_h = hc - half, hc + half
    spans = []
    if lo_h < 0:
        spans = [(0, hi_h), (180 + lo_h, 179)]
    elif hi_h > 179:
        spans = [(lo_h, 179), (0, hi_h - 180)]
    else:
        spans = [(lo_h, hi_h)]
    return [{"lo": [int(round(a)), s_lo, v_lo], "hi": [int(round(b)), 255, 255]} for a, b in spans]


def as_yaml(ranges: list[dict]) -> list[str]:
    """Lines to paste under `ball: colors:` in config/field.yaml."""
    return [f"    - {{lo: {r['lo']}, hi: {r['hi']}}}" for r in ranges]
