"""Where on the floor is each pixel?

Once the camera is calibrated, every pixel is a ray from the camera, and the
ray meets the floor at exactly one spot. So every pixel has a field position,
on one condition: that the pixel is showing something ON the floor. A pixel
showing the top of a robot or ball is reported as the floor spot behind it.
Pixels at or above the horizon never meet the floor and have no position (NaN).
"""
from __future__ import annotations

import cv2
import numpy as np

MAX_RANGE_M = 30.0  # past this a pixel covers so much floor that the number is meaningless


def pixel_to_floor(cam, uv, z: float = 0.0) -> np.ndarray:
    """Pixels (N,2) -> field xy (N,2) on the plane at height z; NaN where the ray misses it."""
    uv = np.asarray(uv, np.float64).reshape(-1, 1, 2)
    n = cv2.undistortPoints(uv, cam.K, cam.dist).reshape(-1, 2)
    rays = np.hstack([n, np.ones((len(n), 1))]) @ cam.R
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (z - cam.center[2]) / rays[:, 2]
        xy = cam.center[:2] + s[:, None] * rays[:, :2]
    bad = ~(s > 0) | (np.linalg.norm(xy - cam.center[:2], axis=1) > MAX_RANGE_M)
    xy[bad] = np.nan
    return xy


def floor_map(cam, width: int, height: int, z: float = 0.0) -> np.ndarray:
    """(height, width, 2) float32: field x, y in metres of every pixel. map[v, u] = (x, y)."""
    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    return pixel_to_floor(cam, np.column_stack([u.ravel(), v.ravel()]), z).reshape(height, width, 2).astype(np.float32)


def _spacing(cam, width: int, height: int) -> float:
    """Grid spacing that puts lines roughly 80+ pixels apart at the image centre."""
    a, b = pixel_to_floor(cam, [[width / 2, height / 2], [width / 2 + 80, height / 2]])
    m = float(np.linalg.norm(b - a)) if np.isfinite(a).all() and np.isfinite(b).all() else 0.5
    return next((s for s in (0.05, 0.10, 0.25, 0.50, 1.0, 2.0) if s >= m), 5.0)


def draw_grid(out: np.ndarray, cam, spacing: float | None = None) -> float:
    """Draw a metric grid on the floor, labelled in cm. Returns the spacing used (m)."""
    h, w = out.shape[:2]
    spacing = spacing or _spacing(cam, w, h)
    # How much floor is in view: sample the image border.
    t = np.linspace(0, 1, 25)
    border = np.vstack([np.column_stack([t * (w - 1), np.zeros_like(t)]), np.column_stack([t * (w - 1), np.full_like(t, h - 1)]),
                        np.column_stack([np.zeros_like(t), t * (h - 1)]), np.column_stack([np.full_like(t, w - 1), t * (h - 1)])])
    seen = pixel_to_floor(cam, border)
    seen = seen[np.isfinite(seen).all(axis=1)]
    if len(seen) == 0:
        return spacing
    reach = 40 * spacing  # never draw more than 80 lines each way
    lo = np.maximum(seen.min(axis=0), cam.center[:2] - reach)
    hi = np.minimum(seen.max(axis=0), cam.center[:2] + reach)
    xs = np.arange(np.floor(lo[0] / spacing), np.ceil(hi[0] / spacing) + 1) * spacing
    ys = np.arange(np.floor(lo[1] / spacing), np.ceil(hi[1] / spacing) + 1) * spacing
    frame = np.array([0, 0, w, h])

    def line(p, q, value, axis_colour):
        pts = np.linspace(p, q, 40)
        px = cam.project(np.column_stack([pts, np.zeros(len(pts))]))
        # Only the part in front of the camera and near the picture (projecting points behind it gives nonsense).
        depth = (np.column_stack([pts, np.zeros(len(pts))]) - cam.center) @ cam.R[2]
        ok = (depth > 0.05) & (np.abs(px - (w / 2, h / 2)) < (3 * w, 3 * h)).all(axis=1)
        major = abs(value) < 1e-9
        colour, thick = (axis_colour, 2) if major else ((200, 200, 200), 1)
        seg = []
        for good, pt in zip(ok, px):
            if good:
                seg.append(pt)
            elif seg:
                cv2.polylines(out, [np.int32(seg)], False, colour, thick, cv2.LINE_AA)
                seg = []
        if seg:
            cv2.polylines(out, [np.int32(seg)], False, colour, thick, cv2.LINE_AA)
        return px[ok & (px >= frame[:2] + 4).all(axis=1) & (px <= frame[2:] - 4).all(axis=1)]

    for x in xs:
        vis = line((x, ys[0]), (x, ys[-1]), x, (80, 80, 255))
        if len(vis):
            cv2.putText(out, f"x{x * 100:+.0f}", tuple(np.int32(vis[len(vis) // 2]) + (4, -4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    for y in ys:
        vis = line((xs[0], y), (xs[-1], y), y, (80, 255, 80))
        if len(vis):
            cv2.putText(out, f"y{y * 100:+.0f}", tuple(np.int32(vis[len(vis) // 3]) + (4, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return spacing


class Probe:
    """Mouse interaction for the live window: hover = position, click = pin a point."""

    def __init__(self, scale: float = 1.0):
        self.scale = scale       # window pixels per image pixel
        self.cursor = None       # (u, v) in full-resolution image pixels
        self.pins: list = []     # pinned points, as FIELD (x, y) so they stay put if the camera is recalibrated
        self.grid = False
        self.bounds = None       # (w, h) of the camera picture inside the window; None = the whole window
        self._click = None       # a click waiting to be turned into a pin

    def on_mouse(self, event, x, y, _flags, _param):
        if self.bounds is not None and not (0 <= x < self.bounds[0] and 0 <= y < self.bounds[1]):
            self.cursor = None   # over the field view or the readouts: there is no camera pixel there
            if event == cv2.EVENT_RBUTTONDOWN:
                self.pins.clear()
            return
        self.cursor = (x / self.scale, y / self.scale)
        if event == cv2.EVENT_LBUTTONDOWN:
            self._click = self.cursor
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.pins.clear()

    def draw(self, out: np.ndarray, cam) -> None:
        if cam is None:
            return
        if self.grid:
            spacing = draw_grid(out, cam)
            self._text(out, f"grid: {spacing * 100:.0f} cm squares   red line x = 0, green line y = 0", (20, out.shape[0] - 20))
        if self._click is not None:
            xy = pixel_to_floor(cam, [self._click])[0]
            if np.isfinite(xy).all():
                self.pins.append((float(xy[0]), float(xy[1])))
            self._click = None
        prev = None
        for i, (x, y) in enumerate(self.pins):
            p = tuple(np.int32(cam.project([[x, y, 0.0]])[0]))
            cv2.drawMarker(out, p, (0, 255, 255), cv2.MARKER_TILTED_CROSS, 24, 3)
            self._text(out, f"{i + 1}: ({x * 100:+.1f}, {y * 100:+.1f}) cm", (p[0] + 14, p[1] - 10), (0, 255, 255))
            if prev is not None:
                q, (px_, py_) = prev
                cv2.line(out, q, p, (0, 255, 255), 2, cv2.LINE_AA)
                mid = ((p[0] + q[0]) // 2, (p[1] + q[1]) // 2)
                self._text(out, f"{np.hypot(x - px_, y - py_) * 100:.1f} cm", mid, (0, 255, 255))
            prev = (p, (x, y))
        if self.cursor is not None:
            c = (int(self.cursor[0]), int(self.cursor[1]))
            xy = pixel_to_floor(cam, [self.cursor])[0]
            cv2.drawMarker(out, c, (255, 255, 255), cv2.MARKER_CROSS, 30, 2)
            label = (f"({xy[0] * 100:+.1f}, {xy[1] * 100:+.1f}) cm on the floor" if np.isfinite(xy).all()
                     else "no floor here (above the horizon)")
            self._text(out, label, (c[0] + 18, c[1] + 34))

    @staticmethod
    def _text(out, text, org, colour=(255, 255, 255)):
        org = (int(org[0]), int(org[1]))
        cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 6, cv2.LINE_AA)
        cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2, cv2.LINE_AA)
