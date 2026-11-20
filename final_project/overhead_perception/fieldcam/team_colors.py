"""Tag-free overhead team localization for Pupper FC from coloured back patches.

Physical recipe
---------------
All sizes are relative to L = ``expected_length_px``, the robot's nose-to-tail
body length as the overhead camera sees it at that spot in the frame.

* Team patch: matte red (keeper) or matte blue (striker) square of side
  0.30 L, or a disk of diameter 0.30 L, centred on the robot's back. Its
  centroid is what ``center_px`` reports.
* Nose patch: matte fluorescent lime / chartreuse square of side 0.12 L, or a
  disk of diameter 0.12 L, on the body centre line with its centre 0.30 L
  ahead of the team patch centre. That leaves a 0.09 L gap between the two
  patches so they never touch.
* Nothing else on the robot, its harness, the ball or the field may be
  saturated red, blue or lime at patch size. Keep the patches flat and matte
  so glare does not punch holes in them.

For example, with a 30 cm body: a 9 cm team square, and a 3.6 cm nose square
whose centre is 9 cm ahead of the team square's centre.

Detection
---------
Each pixel is thresholded with fixed HSV ranges (no learning, no adaptation).
Then 8-connected components are measured against size and shape bounds
scaled by L. We report a team only when exactly one patch-like blob of its
colour exists (inside ``roi_mask``, if one is given) and no near-miss rival
exists. We report a heading only when exactly one nose-like lime blob sits at
the right distance from that team patch, no other lime blob competes, and no
other possible robot back is close enough to claim the same nose. If any of
those checks fail, ``front_px`` is None and the pose is a centre-only preview.
We never infer heading from the patch's own axis or from the nearest clutter.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

__all__ = [
    "TeamColorPose",
    "detect_team_colors",
    "TEAM_PATCH_SIDE_L",
    "NOSE_PATCH_SIDE_L",
    "NOSE_OFFSET_L",
]

Point = Tuple[float, float]

# Physical recipe, in units of the expected body length L.
TEAM_PATCH_SIDE_L = 0.30  # square side or disk diameter
NOSE_PATCH_SIDE_L = 0.12  # square side or disk diameter
NOSE_OFFSET_L = 0.30  # team patch centre -> nose patch centre

# Fixed HSV thresholds, OpenCV scale (H 0-179, S and V 0-255).
# Each entry is (hue_lo, hue_hi, sat_min, val_min) with an inclusive hue range.
# Red stops at H 175 on the magenta side so hot/deep pink (H ~160-170) stays
# out, and needs S >= 150 so skin and pink edge blends stay out.
_RED_HSV = ((0, 8, 150, 70), (175, 179, 150, 70))
_BLUE_HSV = ((100, 128, 120, 50),)
# Fluorescent yellow-green (hue ~70-100 degrees). It must be brighter and more
# saturated than ordinary grass or turf.
_LIME_HSV = ((26, 50, 150, 150),)

_TEAM_HSV = {"striker": _BLUE_HSV, "keeper": _RED_HSV}


@dataclass(frozen=True)
class _Bounds:
    min_area: float  # in L^2
    max_area: float  # in L^2
    max_aspect: float  # long / short side of the min-area rectangle
    min_solidity: float  # pixel area / pixels inside the convex hull
    min_extent: float  # pixel area / min-area rectangle area


# Nominal team patch area: square 0.090 L^2, disk 0.071 L^2. The bounds allow
# roughly +/-22% linear scale error in L.
_TEAM = _Bounds(0.042, 0.135, 1.5, 0.85, 0.60)
# Anything this patch-like still counts as a possible second robot back.
_TEAM_RIVAL = _Bounds(0.021, 0.200, 2.2, 0.70, 0.45)

# Nominal nose patch area: square 0.0144 L^2, disk 0.0113 L^2.
_NOSE = _Bounds(0.0068, 0.023, 1.6, 0.80, 0.55)
_NOSE_RIVAL = _Bounds(0.0034, 0.040, 2.5, 0.60, 0.35)
_NOSE_DIST_L = (0.20, 0.42)  # accepted team-centre -> nose-centre distance
_NOSE_RIVAL_DIST_L = 0.55  # lime blobs or other backs this close compete

_MIN_BLOB_PIXELS = 12  # below this, shape measurements are meaningless


@dataclass(frozen=True)
class TeamColorPose:
    """One robot localized from its patches, in image pixel coordinates.

    ``score`` in [0, 1] measures how well the matched patches fit the recipe
    (size, squareness, solidity, nose placement). It is a shape score, not a
    calibrated probability. Ambiguous scenes produce no pose, or no
    ``front_px``; they never produce a low score.
    """

    center_px: Point
    front_px: Optional[Point]
    score: float

    @property
    def heading_rad(self) -> Optional[float]:
        """Image-frame heading atan2(dy, dx) with y pointing down, or None."""
        if self.front_px is None:
            return None
        return math.atan2(
            self.front_px[1] - self.center_px[1],
            self.front_px[0] - self.center_px[0],
        )


@dataclass(frozen=True, eq=False)
class _Blob:
    area: float
    center: Point
    aspect: float
    solidity: float
    extent: float
    touches_border: bool


def detect_team_colors(
    bgr: np.ndarray,
    expected_length_px: float,
    *,
    roi_mask: Optional[np.ndarray] = None,
) -> Dict[str, TeamColorPose]:
    """Find the striker (blue) and keeper (red) in an overhead BGR frame.

    Returns a dict with only the teams found unambiguously, so it may be
    empty. ``roi_mask`` (HxW, nonzero = field) decides which team patches
    count, using each patch's centroid. Patches are measured on the full
    image, so a patch straddling the ROI edge keeps its true centre instead
    of being clipped. Patches touching the image border may be truncated, so
    they are never accepted, though they still block a second candidate.
    """
    image = _check_image(bgr)
    length = _check_length(expected_length_px)
    roi = _check_roi(roi_mask, image.shape[:2])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    team_blobs = {
        team: _blobs(_color_mask(hsv, ranges), _TEAM_RIVAL, length)
        for team, ranges in _TEAM_HSV.items()
    }
    noses = _blobs(_color_mask(hsv, _LIME_HSV), _NOSE_RIVAL, length)
    # Every red or blue blob that could be a robot back, ignoring the ROI.
    # Any of them can claim a nearby nose.
    possible_backs = [
        blob
        for blobs in team_blobs.values()
        for blob in blobs
        if _fits(blob, _TEAM_RIVAL, length)
    ]

    poses: Dict[str, TeamColorPose] = {}
    for team, blobs in team_blobs.items():
        back = _unique_back(blobs, roi, length)
        if back is None:
            continue
        back_score = _shape_score(back, _TEAM, length)
        nose, nose_score = _unique_nose(back, noses, possible_backs, length)
        if nose is None:
            poses[team] = TeamColorPose(back.center, None, back_score)
        else:
            score = math.sqrt(back_score * nose_score)
            poses[team] = TeamColorPose(back.center, nose.center, score)
    return poses


def _unique_back(
    blobs: List[_Blob], roi: Optional[np.ndarray], length: float
) -> Optional[_Blob]:
    candidates = [blob for blob in blobs if _in_roi(blob, roi)]
    winners = [
        blob
        for blob in candidates
        if not blob.touches_border and _fits(blob, _TEAM, length)
    ]
    if len(winners) != 1:
        return None
    winner = winners[0]
    for blob in candidates:
        if blob is not winner and _fits(blob, _TEAM_RIVAL, length):
            return None
    return winner


def _unique_nose(
    back: _Blob, noses: List[_Blob], possible_backs: List[_Blob], length: float
) -> Tuple[Optional[_Blob], float]:
    near = [(nose, _distance(back.center, nose.center) / length) for nose in noses]
    lo, hi = _NOSE_DIST_L
    matches = [
        (nose, d)
        for nose, d in near
        if lo <= d <= hi and not nose.touches_border and _fits(nose, _NOSE, length)
    ]
    if len(matches) != 1:
        return None, 0.0
    nose, d = matches[0]
    for other, other_d in near:
        if (
            other is not nose
            and other_d <= _NOSE_RIVAL_DIST_L
            and _fits(other, _NOSE_RIVAL, length)
        ):
            return None, 0.0
    # A nose that another possible back could also own (a crossing, or a nose
    # shared between two robots) gives no trustworthy heading.
    for other in possible_backs:
        if other is not back and (
            _distance(other.center, nose.center) <= _NOSE_RIVAL_DIST_L * length
        ):
            return None, 0.0
    placement = 1.0 - abs(d - NOSE_OFFSET_L) / (hi - NOSE_OFFSET_L)
    return nose, _shape_score(nose, _NOSE, length) * _clamp01(placement)


def _color_mask(hsv: np.ndarray, ranges) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for h_lo, h_hi, s_min, v_min in ranges:
        lower = np.array([h_lo, s_min, v_min], np.uint8)
        upper = np.array([h_hi, 255, 255], np.uint8)
        mask |= cv2.inRange(hsv, lower, upper)
    return mask


def _blobs(mask: np.ndarray, bounds: _Bounds, length: float) -> List[_Blob]:
    """Measure every component whose pixel area could fit ``bounds``."""
    height, width = mask.shape
    min_pixels = max(_MIN_BLOB_PIXELS, bounds.min_area * length * length)
    max_pixels = bounds.max_area * length * length
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask, connectivity=8
    )
    blobs = []
    for i in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[i])
        if not min_pixels <= area <= max_pixels:
            continue
        component = (labels[y : y + h, x : x + w] == i).astype(np.uint8)
        contours = cv2.findContours(
            component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )[-2]
        contour = max(contours, key=len)
        hull_pixels = _lattice_pixels(cv2.convexHull(contour))
        (_, _), (rect_w, rect_h), _ = cv2.minAreaRect(contour)
        # Contours run through pixel centres, so add one pixel to each side
        # of the rectangle to compare it with a pixel count.
        long_side = max(rect_w, rect_h) + 1.0
        short_side = min(rect_w, rect_h) + 1.0
        blobs.append(
            _Blob(
                area=float(area),
                center=(float(centroids[i][0]), float(centroids[i][1])),
                aspect=long_side / short_side,
                solidity=min(1.0, area / hull_pixels),
                extent=min(1.0, area / (long_side * short_side)),
                touches_border=x == 0 or y == 0 or x + w >= width or y + h >= height,
            )
        )
    return blobs


def _lattice_pixels(polygon: np.ndarray) -> float:
    """Pixel centres inside or on an integer polygon, by Pick's theorem."""
    pts = polygon.reshape(-1, 2).astype(np.int64)
    nxt = np.roll(pts, -1, axis=0)
    twice_area = abs(int(np.sum(pts[:, 0] * nxt[:, 1] - nxt[:, 0] * pts[:, 1])))
    step = np.abs(nxt - pts)
    boundary = int(np.sum(np.gcd(step[:, 0], step[:, 1])))
    return twice_area / 2.0 + boundary / 2.0 + 1.0


def _fits(blob: _Blob, bounds: _Bounds, length: float) -> bool:
    area_l2 = blob.area / (length * length)
    return (
        bounds.min_area <= area_l2 <= bounds.max_area
        and blob.aspect <= bounds.max_aspect
        and blob.solidity >= bounds.min_solidity
        and blob.extent >= bounds.min_extent
    )


def _shape_score(blob: _Blob, bounds: _Bounds, length: float) -> float:
    """1 for an ideal patch at mid size, falling to 0 at any bound."""
    area_l2 = blob.area / (length * length)
    mid = math.sqrt(bounds.min_area * bounds.max_area)
    half_span = 0.5 * math.log(bounds.max_area / bounds.min_area)
    size = 1.0 - abs(math.log(area_l2 / mid)) / half_span
    aspect = 1.0 - (blob.aspect - 1.0) / (bounds.max_aspect - 1.0)
    solidity = (blob.solidity - bounds.min_solidity) / (1.0 - bounds.min_solidity)
    product = _clamp01(size) * _clamp01(aspect) * _clamp01(solidity)
    return product ** (1.0 / 3.0)


def _in_roi(blob: _Blob, roi: Optional[np.ndarray]) -> bool:
    if roi is None:
        return True
    height, width = roi.shape
    col = min(max(int(round(blob.center[0])), 0), width - 1)
    row = min(max(int(round(blob.center[1])), 0), height - 1)
    return bool(roi[row, col])


def _distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _clamp01(value: float) -> float:
    return min(1.0, max(0.0, value))


def _check_image(bgr) -> np.ndarray:
    if not isinstance(bgr, np.ndarray) or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("bgr must be an HxWx3 BGR image array")
    if bgr.dtype != np.uint8:
        raise ValueError("bgr must have dtype uint8")
    return np.ascontiguousarray(bgr)


def _check_length(expected_length_px) -> float:
    length = float(expected_length_px)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("expected_length_px must be a positive finite number")
    return length


def _check_roi(roi_mask, shape) -> Optional[np.ndarray]:
    if roi_mask is None:
        return None
    roi = np.asarray(roi_mask)
    if roi.shape != tuple(shape):
        raise ValueError(f"roi_mask shape {roi.shape} does not match image {shape}")
    return roi != 0
