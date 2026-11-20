"""Image-space detectors: AprilTags (via OpenCV's aruco module) and the orange ball."""
from __future__ import annotations

import cv2
import numpy as np

TAG_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


class TagDetector:
    def __init__(self, expected_floor_ids=()):
        self.expected = set(expected_floor_ids)
        p = cv2.aruco.DetectorParameters()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        p.adaptiveThreshWinSizeMax = 45      # copes with uneven arena lighting
        p.minMarkerPerimeterRate = 0.01      # tags are small in a whole-field view
        self.duplicates = set()
        self._det = cv2.aruco.ArucoDetector(TAG_DICT, p)

    def detect(self, gray: np.ndarray) -> dict[int, np.ndarray]:
        """Returns {tag_id: corners (4,2)} in order TL, TR, BR, BL of the printed tag."""
        def decode(image):
            corners, ids, _ = self._det.detectMarkers(image)
            if ids is None:
                return {}, set()
            labels, counts = np.unique(ids, return_counts=True)
            repeats = set(map(int, labels[counts > 1]))
            return {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}, repeats

        tags, self.duplicates = decode(gray)
        # Annotated mounted-camera captures resolve different small tags at the
        # two scales. Keep the native corners when both decodes agree.
        if self.expected - tags.keys() and gray.shape[1] >= 960:
            small = cv2.resize(gray, None, fx=.7, fy=.7, interpolation=cv2.INTER_AREA)
            extra, repeats = decode(small)
            self.duplicates.update(repeats)
            scale = np.array([gray.shape[1] / small.shape[1], gray.shape[0] / small.shape[0]])
            for tid, corners in extra.items():
                corners = ((corners + .5) * scale - .5).astype(np.float32)
                if tid in tags and np.linalg.norm(tags[tid] - corners, axis=1).max() > 5:
                    self.duplicates.add(tid)
                else:
                    tags.setdefault(tid, corners)
        return {tid: corners for tid, corners in tags.items() if tid not in self.duplicates}


class BallDetector:
    """Finds a ball that may have several colours (e.g. a red/blue/grey mini basketball).

    1. Colour: any pixel inside one of the HSV ranges counts as "ball coloured".
       Nearby coloured patches are merged, so panels of other colours in between
       (grey, seams, a logo) do not split the ball.
    2. Shape: the coloured patches only say roughly where the ball is. The exact
       centre and radius come from fitting a circle to the ball's outline in the
       image, so a ball showing mostly one colour on one side is not pulled
       off-centre.
    """

    def __init__(self, colors, min_area_px: float = 30.0, min_edge_support: float = 0.45):
        # colors: [{"lo": [h, s, v], "hi": [h, s, v]}, ...]; a (lo, hi) pair also works.
        if colors and not isinstance(colors[0], dict):
            colors = [{"lo": colors[0], "hi": colors[1]}]
        self.ranges = [(np.array(c["lo"], np.uint8), np.array(c["hi"], np.uint8)) for c in colors]
        self.min_area = min_area_px
        self.min_support = min_edge_support
        self._open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.last_support = 0.0  # how much of the fitted outline had an edge under it (debug)

    def mask(self, bgr: np.ndarray, roi_mask: np.ndarray | None = None) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.ranges:
            m |= cv2.inRange(hsv, lo, hi)
        if roi_mask is not None:
            m = cv2.bitwise_and(m, roi_mask)
        return cv2.morphologyEx(m, cv2.MORPH_OPEN, self._open)

    def detect(self, bgr, roi_mask=None, expected_radius_px: float | None = None, radius_at=None):
        """Returns (u, v, radius_px) of the ball, or None.

        expected_radius_px: roughly how big the ball should look (None = unknown).
        radius_at(u, v): how big it should look at that pixel. In a close or
        slanted view the ball looks much bigger near the camera than far away,
        so one number for the whole picture is not enough.
        """
        m = self.mask(bgr, roi_mask)
        # Work on a reduced copy when the ball is large in the image: same answer, far less work.
        guess = expected_radius_px or 0.0
        if not guess:
            ys, xs = np.nonzero(m)
            if len(xs) < self.min_area:
                return None
            guess = 0.5 * max(np.ptp(xs), np.ptp(ys))
        k = min(1.0, 40.0 / max(guess, 1.0))
        small = cv2.resize(m, None, fx=k, fy=k, interpolation=cv2.INTER_AREA) if k < 1.0 else m
        small = (small > 100).astype(np.uint8) * 255

        # Merge patches that are closer together than about one ball radius.
        gap = max(3, int(round(0.9 * guess * k)) | 1)
        merged = cv2.morphologyEx(small, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (gap, gap)))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(merged)
        best, best_area = None, 0
        for i in range(1, n):
            ys, xs = np.nonzero((labels == i) & (small > 0))
            if len(xs) * (1 / k) ** 2 < self.min_area:
                continue
            (cx, cy), r = cv2.minEnclosingCircle(np.column_stack([xs, ys]).astype(np.float32))
            r_full = r / k
            if radius_at is not None:
                expected_radius_px = radius_at(cx / k, cy / k) or expected_radius_px
            if expected_radius_px and not (0.35 * expected_radius_px < r_full < 1.6 * expected_radius_px):
                continue  # wrong size to be the ball (shirt, shoe, cone...)
            if len(xs) > best_area:
                best, best_area, best_expected = (cx / k, cy / k, r_full), len(xs), expected_radius_px
                step = max(1, len(xs) // 60)  # a sample of the coloured pixels, full-res coordinates
                colour_pts = np.column_stack([xs[::step], ys[::step]]) / k
        if best is None:
            return None
        u0, v0, r0 = best
        expected_radius_px = best_expected
        if expected_radius_px:
            r_lo, r_hi, reach = 0.85 * expected_radius_px, 1.15 * expected_radius_px, 1.0 * expected_radius_px
        else:
            r_lo, r_hi, reach = 0.8 * r0, 1.5 * r0, 0.7 * r0
        fit = self._fit_outline(bgr, u0, v0, r_lo, r_hi, reach, colour_pts)
        if fit is not None:
            return fit
        # Outline not clear enough (ball partly hidden?): fall back to the coloured patch itself.
        self.last_support = 0.0
        return (u0, v0, expected_radius_px or r0)

    def _fit_outline(self, bgr, u0, v0, r_lo, r_hi, reach, colour_pts):
        """Circle (radius within [r_lo, r_hi], centre within `reach` of the guess)
        whose outline lies on the most image edges AND which contains the ball-coloured
        pixels. The second condition stops the fit from settling on a seam or panel
        edge inside the ball, which is also a nice arc but leaves colour outside it."""
        pad = reach + r_hi + 6
        x0, y0 = int(max(0, u0 - pad)), int(max(0, v0 - pad))
        x1, y1 = int(min(bgr.shape[1], u0 + pad)), int(min(bgr.shape[0], v0 + pad))
        if x1 - x0 < 8 or y1 - y0 < 8:
            return None
        k = min(1.0, 32.0 / r_hi)  # search at ~32 px radius, refine at full resolution after
        crop = bgr[y0:y1, x0:x1]
        if k < 1.0:
            crop = cv2.resize(crop, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        gray = cv2.GaussianBlur(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        med = float(np.median(gray))
        edges = cv2.Canny(gray, 0.4 * med, 0.9 * med)
        # Distance to the nearest edge: a soft score, so a circle that is off by a pixel still counts.
        near = np.exp(-(cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3) / 1.5) ** 2)
        h, w = near.shape
        cx0, cy0 = (u0 - x0) * k, (v0 - y0) * k
        cp = (np.asarray(colour_pts, float) - (x0, y0)) * k

        def search(cxs, cys, radii, n_ang):
            ang = np.linspace(0, 2 * np.pi, n_ang, endpoint=False)
            ca, sa = np.cos(ang), np.sin(ang)
            CX, CY = np.meshgrid(cxs, cys)
            # Distance from every candidate centre to its farthest-out coloured pixels, sorted
            # once, so "what fraction lies outside radius r" is a lookup per radius.
            d = np.sort(np.sqrt((CX[..., None] - cp[:, 0]) ** 2 + (CY[..., None] - cp[:, 1]) ** 2), axis=-1)
            top = (0.0, None)
            for r in radii:
                px = np.rint(CX[..., None] + r * ca).astype(int)
                py = np.rint(CY[..., None] + r * sa).astype(int)
                inside = (px >= 0) & (px < w) & (py >= 0) & (py < h)
                score = np.where(inside, near[np.clip(py, 0, h - 1), np.clip(px, 0, w - 1)], 0.0).mean(axis=-1)
                score = score - 1.5 * (d > r + 1.5).mean(axis=-1)
                j = int(np.argmax(score))
                if score.flat[j] > top[0]:
                    top = (float(score.flat[j]), (float(CX.flat[j]), float(CY.flat[j]), float(r)))
            return top

        span = reach * k
        best = search(np.arange(cx0 - span, cx0 + span + 1e-6, 3.0), np.arange(cy0 - span, cy0 + span + 1e-6, 3.0),
                      np.arange(r_lo * k, r_hi * k + 1e-6, 1.5), 48)
        if best[1] is not None:
            cx, cy, r = best[1]
            fine = search(np.arange(cx - 3.0, cx + 3.0 + 1e-6, 0.5), np.arange(cy - 3.0, cy + 3.0 + 1e-6, 0.5),
                          np.arange(max(r - 1.5, 1.0), r + 1.5 + 1e-6, 0.5), 96)
            if fine[0] >= best[0]:
                best = fine
        self.last_support = best[0]
        if best[1] is None or best[0] < self.min_support:
            return None
        cx, cy, r = best[1]
        return (x0 + cx / k, y0 + cy / k, r / k)
