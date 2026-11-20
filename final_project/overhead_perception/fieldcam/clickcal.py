"""Calibrate without floor tags: click floor points whose field positions are known.

The points come from `manual_points` in field.yaml (default: the four field
corners). Clicks are in full-resolution image pixels; the window may show the
frame scaled down, so `scale` converts window pixels back.
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

KEY_ESC, KEY_UNDO = 27, ord("u")


class ClickSession:
    """Which point is next, what has been clicked so far."""

    def __init__(self, points: list[dict]):
        self.points = points
        self.clicks: list[tuple[float, float]] = []

    @property
    def done(self) -> bool:
        return len(self.clicks) >= len(self.points)

    @property
    def prompt(self) -> str:
        if self.done:
            return "all points clicked"
        p = self.points[len(self.clicks)]
        return f"click {len(self.clicks) + 1}/{len(self.points)}: {p['name']}"

    def click(self, u: float, v: float):
        if not self.done:
            self.clicks.append((float(u), float(v)))

    def undo(self):
        if self.clicks:
            self.clicks.pop()

    def draw(self, frame: np.ndarray) -> np.ndarray:
        out = frame.copy()
        for i, (u, v) in enumerate(self.clicks):
            c = (int(round(u)), int(round(v)))
            cv2.drawMarker(out, c, (0, 255, 0), cv2.MARKER_CROSS, 40, 2)
            cv2.putText(out, str(i + 1), (c[0] + 12, c[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        for y, text in ((45, self.prompt), (90, "u = undo last click    Esc = cancel")):
            cv2.putText(out, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 6)
            cv2.putText(out, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 2)
        return out


def collect_clicks(window: str, frame: np.ndarray, points: list[dict], scale: float = 1.0):
    """Freeze `frame` in `window` and wait for one click per point.

    Returns the clicked image points (N, 2) in full-resolution pixels, or None
    if cancelled with Esc.
    """
    session = ClickSession(points)

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            session.click(x / scale, y / scale)

    def show():
        vis = session.draw(frame)
        if scale != 1.0:
            vis = cv2.resize(vis, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        cv2.imshow(window, vis)

    show()  # the window must exist before a mouse callback can be attached
    cv2.setMouseCallback(window, on_mouse)
    try:
        while not session.done:
            show()
            key = cv2.waitKey(20) & 0xFF
            if key == KEY_ESC:
                return None
            if key == KEY_UNDO:
                session.undo()
    finally:
        cv2.setMouseCallback(window, lambda *_a: None)
    return np.array(session.clicks, float)


def save_clicks(path: Path, image_pts, points: list[dict], size: tuple[int, int]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({
        "size": list(size),
        "points": [{"name": p["name"], "xy": list(p["xy"]), "pixel": [float(u), float(v)]}
                   for p, (u, v) in zip(points, image_pts)],
    }, indent=2))


def load_clicks(path: Path, points: list[dict], size: tuple[int, int]):
    """Saved clicks, but only if they match the current resolution and point list."""
    if not path.exists():
        return None
    saved = json.loads(path.read_text())
    same = (saved.get("size") == list(size)
            and [(p["name"], list(p["xy"])) for p in saved["points"]]
            == [(p["name"], [float(v) for v in p["xy"]]) for p in points])
    return np.array([p["pixel"] for p in saved["points"]], float) if same else None
