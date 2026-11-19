"""Ball detection for kick_ball: one camera frame -> ball centre in the body frame.

Reduced copy of the onboard detector in ``final_project/behaviors`` (``ball_color.py``,
``ball_geometry.py``, ``ball_detector_node.py``; main 4110d82), with a silhouette fit
in place of the colour box. The deployed Gemini package does not import
``final_project``, and the kick behavior runs detection only while its job runs,
in the tool server, so no detector node is necessary.

Pipeline: JPEG decoded at half size -> the floor rows of the Lab 5 220 deg
equirectangular image -> red and blue saturated pixels (the red, white and blue
beach ball; YOLO COCO "sports ball" does not detect it) -> 3 x 3 majority filter
-> blobs (close, then 8-connected components) -> silhouette fit -> plausibility.

Silhouette fit: the limb of a sphere is a circle on the unit sphere of camera
rays. The edge of the coloured pixels gives that circle (direction of the centre
and angular radius rho; range R / sin(rho)), but the white gores are not
detected: where a white gore is at the limb, the coloured edge is inside the
circle. So the fit finds the circle that has the most edge points on it and no
edge points outside it (candidates through 3 convex-hull vertices), then refits
it by weighted least squares, where edge points inside the circle lose weight.
Blobs inside the fitted circle (red and blue gores that a white gore separates)
join the fit. The colour box (the trial detector's method) misses the white
gores, so its centre moved 3 to 4 cm when the ball turned; it is now only the
fallback when the fit is not reliable (``BallFix.method`` 'box', ``box_scale``).

Plausibility: the ball centre is on the floor, at ``ball_z`` on base_link. A
red and blue object above the floor (a person) gives a centre above it.

No IMU levelling: the robot walks on a flat floor, the range comes from the
angular size (no camera height or tilt), and a false IMU tilt (seen on
2026-11-16) would move the estimate. Pitch moves x and z only by cos(pitch).

Frames: optical (x right, y down, z forward) -> body (x forward, y left, z up),
origin at base_link. NumPy, and OpenCV where it is installed. The pixel steps
have NumPy versions, so the geometry, the blobs and the fit run without OpenCV
(the tool server's API environment and the offline tests); the robot has
OpenCV from ROS for the JPEG decode, the remap and the HSV conversion.
"""

from __future__ import annotations

import itertools
import logging
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

LOG = logging.getLogger(__name__)

CAMERA_YAML = Path(__file__).resolve().parent / "camera_params.yaml"
EQ_WIDTH = 700  # the Lab 5 follower's equirectangular width
H_FOV_DEG = 220.0
# fisheye_converter.equirect_height(EQ_WIDTH, H_FOV_DEG): equal degrees per pixel in x and y.
EQ_HEIGHT = max(2, int(round(EQ_WIDTH * 180.0 / H_FOV_DEG)) // 2 * 2)
PIX_RAD = math.radians(H_FOV_DEG) / (EQ_WIDTH - 1)  # one pixel [rad]; a row step is 0.2 % larger


@dataclass(frozen=True)
class ColorParams:
    """Colour thresholds (OpenCV HSV), set on a Pupper frame on 2026-11-16."""

    s_min: int = 140
    v_min: int = 70
    red_hue: tuple[int, int] = (13, 165)  # red is h <= 13 or h >= 165 (hue 0..179)
    blue_hue: tuple[int, int] = (98, 130)
    # First processed row, as a fraction of the image height. 0.50 is the optical
    # axis: 10 deg above the horizon with tilt_up_deg 10. A ball at 0.45 m has its
    # top at +3 deg and its centre at -16 deg (base_link 0.15 m above the floor).
    row_min_frac: float = 0.50
    close_px: int = 9
    min_pixels: int = 30
    min_each_frac: float = 0.05  # colour-box fallback: each colour's share of the coloured pixels
    denoise: int = 1  # 1: 3 x 3 majority filter on each colour mask before the blobs


@dataclass(frozen=True)
class CameraMount:
    """Camera pose on base_link. Estimates, not measured (2026-11-16 handoff)."""

    x: float = 0.15  # m forward
    y: float = 0.0  # m left
    z: float = 0.03  # m up
    tilt_up_deg: float = 10.0
    yaw_deg: float = 0.0  # optical axis to the left of the body x axis
    range_scale: float = 1.0  # true / measured range of the silhouette fit


@dataclass(frozen=True)
class VisionParams:
    ball_radius: float = 0.097  # m, beach ball, circumference approximately 24 in
    mount: CameraMount = field(default_factory=CameraMount)
    color: ColorParams = field(default_factory=ColorParams)
    y_bias: float = 0.0  # m, subtracted from y (calibrate with a ball on the body axis)
    min_range: float = 0.15
    max_range: float = 3.0
    # Silhouette fit (distances in equirectangular pixels, PIX_RAD each).
    edge_px: float = 0.0  # added to the fitted radius (if the colour edge is not at the limb)
    fit_inner_px: float = 0.5  # an edge point at most this far inside the circle is on it
    fit_outside_px: float = 1.5  # an edge point farther outside the circle is an outlier
    fit_min_points: int = 8  # edge points on the circle
    fit_min_span_deg: float = 120.0  # their spread around the centre
    fit_max_resid_px: float = 0.6  # RMS distance of those points from the circle
    fit_max_outside: float = 0.03  # largest fraction of outlier edge points
    fit_min_fill: float = 0.15  # coloured pixels / pixels in the circle (processed rows)
    fit_min_each_frac: float = 0.01  # each colour's share for a fit (a thin gore at the limb)
    single_colour: int = 0  # 1: a red-only or blue-only blob with a reliable fit is a ball too
    # Colour-box fallback (fit not reliable): two colours (color.min_each_frac), box
    # aspect at most box_max_aspect, range times box_scale (range_scale not applied).
    # The box misses the white gores; 0.616 is the trial detector's empirical value.
    box_scale: float = 0.616
    box_max_aspect: float = 2.0  # larger / smaller angular side of the box
    # A box fix is also refused when the box touches the image side edges (a cut
    # object), when its colour fill is low (a ball's colour box is approximately 45 %
    # filled; the floor mat seen on 2026-11-18 was 15 %), or when it is very near.
    box_min_fill: float = 0.20  # coloured pixels / box area
    box_min_range: float = 0.40  # m
    # Plausibility: |z - ball_z| <= z_tol + z_tol_per_m * range, else no fix.
    ball_z: float = -0.05  # m, ball centre on base_link: ball radius minus base_link height (0.15 m)
    z_tol: float = 0.04  # base_link height 0.11 to 0.19 m
    z_tol_per_m: float = 0.03  # pitch and tilt error, approximately 1.7 deg
    max_blobs: int = 8  # blobs examined per frame, largest first (clutter can be larger than the ball)
    near_range: float = 0.30  # m, nearest ball centre the camera can see (sets the largest blob fitted)


@dataclass(frozen=True)
class BallBox:
    """One blob of red and blue pixels."""

    box: tuple[float, float, float, float]  # x1, y1, x2, y2 in the equirectangular image
    pixels: int  # red and blue pixels
    n_red: int = 0
    n_blue: int = 0
    uv: np.ndarray | None = field(default=None, compare=False, repr=False)  # (N, 2) pixels (u, v)


@dataclass(frozen=True)
class BallFix:
    """One detection: ball centre in the body frame [m]."""

    x: float
    y: float
    z: float
    box: tuple[float, float, float, float]
    pixels: int
    method: str = "fit"  # 'fit': silhouette fit; 'box': colour-box fallback (larger error)
    rho: float = 0.0  # angular radius used for the range [rad]

    @property
    def range(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def bearing(self) -> float:
        return math.atan2(self.y, self.x)


@dataclass(frozen=True)
class SilhouetteFit:
    """A circle on the unit sphere of rays through the outer edge of a blob."""

    centre: np.ndarray  # unit ray to the ball centre, optical frame
    rho: float  # angular radius [rad], without edge_px
    points: int  # edge points on the circle
    span_deg: float  # their spread around the centre
    resid_px: float  # RMS distance of those points from the circle
    outside: float  # fraction of edge points farther outside than fit_outside_px
    fill: float  # coloured pixels / pixels in the circle (processed rows)

    def problem(self, p: VisionParams) -> str | None:
        """Why the fit is not reliable, or None."""
        if self.points < p.fit_min_points:
            return f"{self.points} edge points"
        if self.span_deg < p.fit_min_span_deg:
            return f"edge span {self.span_deg:.0f} deg"
        if self.resid_px > p.fit_max_resid_px:
            return f"residual {self.resid_px:.2f} px"
        if self.outside > p.fit_max_outside:
            return f"{self.outside:.0%} of edge outside"
        if self.fill < p.fit_min_fill:
            return f"fill {self.fill:.2f}"
        return None


# --- pixels and rays ---------------------------------------------------------------


def pixel_ray(u: float, v: float, width: int, height: int, h_fov_deg: float = H_FOV_DEG) -> np.ndarray:
    """Unit ray (optical frame) through equirectangular pixel (u, v)."""
    hf = math.radians(h_fov_deg)
    lon = u / (width - 1) * hf - hf / 2
    lat = v / (height - 1) * math.pi - math.pi / 2
    return np.array([math.cos(lat) * math.sin(lon), math.sin(lat), math.cos(lat) * math.cos(lon)])


def pixel_rays(u, v, width: int = EQ_WIDTH, height: int = EQ_HEIGHT, h_fov_deg: float = H_FOV_DEG) -> np.ndarray:
    """pixel_ray for arrays: (..., 3)."""
    hf = math.radians(h_fov_deg)
    lon = np.asarray(u, float) / (width - 1) * hf - hf / 2
    lat = np.asarray(v, float) / (height - 1) * math.pi - math.pi / 2
    return np.stack([np.cos(lat) * np.sin(lon), np.sin(lat), np.cos(lat) * np.cos(lon)], -1)


def ray_pixel(ray: np.ndarray, width: int, height: int, h_fov_deg: float = H_FOV_DEG) -> tuple[float, float]:
    """Inverse of pixel_ray."""
    x, y, z = ray / np.linalg.norm(ray)
    hf = math.radians(h_fov_deg)
    lon = math.atan2(x, z)
    lat = math.asin(max(-1.0, min(1.0, y)))
    return (lon + hf / 2) / hf * (width - 1), (lat + math.pi / 2) / math.pi * (height - 1)


def ray_pixels(rays: np.ndarray, width: int = EQ_WIDTH, height: int = EQ_HEIGHT,
               h_fov_deg: float = H_FOV_DEG) -> np.ndarray:
    """ray_pixel for an (N, 3) array: (N, 2) (u, v)."""
    r = rays / np.linalg.norm(rays, axis=-1, keepdims=True)
    hf = math.radians(h_fov_deg)
    lon = np.arctan2(r[..., 0], r[..., 2])
    lat = np.arcsin(np.clip(r[..., 1], -1.0, 1.0))
    return np.stack([(lon + hf / 2) / hf * (width - 1), (lat + math.pi / 2) / math.pi * (height - 1)], -1)


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    return math.atan2(np.linalg.norm(np.cross(a, b)), float(np.dot(a, b)))


def _angles(rays: np.ndarray, c: np.ndarray) -> np.ndarray:
    """Angle [rad] from each unit ray to unit vector c (arccos: exact enough above 1e-6 rad)."""
    return np.arccos(np.clip(rays @ c, -1.0, 1.0))


def _angles_many(cs: np.ndarray, rays: np.ndarray) -> np.ndarray:
    """(len(cs), len(rays)) angles [rad] between unit vectors."""
    return np.arccos(np.clip(cs @ rays.T, -1.0, 1.0))


# --- colour masks and blobs ----------------------------------------------------------


def _cv2():
    try:
        import cv2
    except ImportError:
        return None
    return cv2


def classify(rgb: np.ndarray, p: ColorParams = ColorParams()) -> tuple[np.ndarray, np.ndarray]:
    """(red, blue) masks of an RGB image (OpenCV HSV); rows above row_min_frac are False."""
    import cv2

    row0 = int(rgb.shape[0] * p.row_min_frac)
    red = np.zeros(rgb.shape[:2], bool)
    blue = np.zeros(rgb.shape[:2], bool)
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb[row0:]), cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    strong = (s >= p.s_min) & (v >= p.v_min)
    red[row0:] = strong & ((h <= p.red_hue[0]) | (h >= p.red_hue[1]))
    blue[row0:] = strong & (h >= p.blue_hue[0]) & (h <= p.blue_hue[1])
    return red, blue


def _ellipse_offsets(k: int) -> list[tuple[int, int]]:
    """(dy, dx) of cv2.getStructuringElement(MORPH_ELLIPSE, (k, k)) about its centre."""
    r = k // 2
    out = []
    for i in range(k):
        dy = i - r
        dx = int(round(r * math.sqrt((r * r - dy * dy) / (r * r)))) if r else 0
        out += [(dy, j - r) for j in range(max(r - dx, 0), min(r + dx + 1, k))]
    return out


def _dilate(mask: np.ndarray, offsets) -> np.ndarray:
    """Binary dilation; pixels outside the image are False."""
    h, w = mask.shape
    out = np.zeros_like(mask)
    for dy, dx in offsets:
        out[max(dy, 0):h + min(dy, 0), max(dx, 0):w + min(dx, 0)] |= \
            mask[max(-dy, 0):h - max(dy, 0), max(-dx, 0):w - max(dx, 0)]
    return out


def close_mask(mask: np.ndarray, k: int, use_cv2: bool | None = None) -> np.ndarray:
    """Morphological close with OpenCV's k x k ellipse (OpenCV if found, else NumPy)."""
    if k <= 1:
        return mask.copy()
    cv2 = _cv2() if use_cv2 is not False else None
    if cv2 is not None:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        return cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel).astype(bool)
    offsets = _ellipse_offsets(k)
    return ~_dilate(~_dilate(mask, offsets), offsets)  # the erode treats outside pixels as True, as OpenCV does


def label_mask(mask: np.ndarray, use_cv2: bool | None = None) -> tuple[int, np.ndarray]:
    """(count, labels): 8-connected components, labels 1..count, 0 is background."""
    cv2 = _cv2() if use_cv2 is not False else None
    if cv2 is not None:
        n, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
        return n - 1, labels
    h, w = mask.shape
    edges = np.diff(np.pad(mask.astype(np.int8), ((0, 0), (1, 1))), axis=1)
    rows, starts = np.nonzero(edges == 1)
    _, ends = np.nonzero(edges == -1)  # first column after the run
    labels = np.zeros((h, w), np.int32)
    if len(rows) == 0:
        return 0, labels
    parent = list(range(len(rows)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    first = np.searchsorted(rows, np.arange(h + 1))
    for r in range(1, h):
        i, i_end, j, j_end = first[r], first[r + 1], first[r - 1], first[r]
        while i < i_end and j < j_end:
            if starts[i] <= ends[j] and starts[j] <= ends[i]:  # touch, diagonals included
                a, b = root(i), root(j)
                if a != b:
                    parent[max(a, b)] = min(a, b)
            if ends[i] < ends[j]:
                i += 1
            else:
                j += 1
    roots = [root(i) for i in range(len(rows))]
    ids = {}
    for k, rt in enumerate(roots):
        labels[rows[k], starts[k]:ends[k]] = ids.setdefault(rt, len(ids) + 1)
    return len(ids), labels


def majority3(mask: np.ndarray) -> np.ndarray:
    """3 x 3 majority filter (median of a binary mask): removes single-pixel noise."""
    h, w = mask.shape
    pad = np.pad(mask.astype(np.uint8), 1)
    count = sum(pad[dy:dy + h, dx:dx + w] for dy in range(3) for dx in range(3))
    return count >= 5


def find_blobs(red: np.ndarray, blue: np.ndarray, p: ColorParams, row0: int,
               use_cv2: bool | None = None) -> list[BallBox]:
    """Blobs of red and blue pixels in rows >= row0, largest first, each >= min_pixels.
    A close joins pixels across small gaps; one-colour blobs are included."""
    red, blue = red[row0:], blue[row0:]
    colored = red | blue
    ys, xs = np.nonzero(colored)
    if len(ys) < p.min_pixels:
        return []
    n, labels = label_mask(close_mask(colored, p.close_px, use_cv2), use_cv2)
    lab = labels[ys, xs]  # the close keeps every coloured pixel, so each has a label
    order = np.argsort(lab, kind="stable")
    lab, ys, xs = lab[order], ys[order], xs[order]
    is_red = red[ys, xs]
    first = np.searchsorted(lab, np.arange(1, n + 2))
    blobs = []
    for a, b in zip(first[:-1], first[1:]):
        if b - a < p.min_pixels:
            continue
        u, v = xs[a:b], ys[a:b] + row0
        n_red = int(np.count_nonzero(is_red[a:b]))
        box = (float(u.min()), float(v.min()), float(u.max() + 1), float(v.max() + 1))
        blobs.append(BallBox(box, int(b - a), n_red, int(b - a) - n_red, np.stack([u, v], 1)))
    blobs.sort(key=lambda blob: -blob.pixels)
    return blobs


def _merge(blobs: list[BallBox]) -> BallBox:
    if len(blobs) == 1:
        return blobs[0]
    box = (min(b.box[0] for b in blobs), min(b.box[1] for b in blobs),
           max(b.box[2] for b in blobs), max(b.box[3] for b in blobs))
    return BallBox(box, sum(b.pixels for b in blobs), sum(b.n_red for b in blobs),
                   sum(b.n_blue for b in blobs), np.concatenate([b.uv for b in blobs]))


# --- silhouette fit -------------------------------------------------------------------


def _edge_points(uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Edge crossings of the blob: (M, 2) points (u, v) and (M, 2) source pixels.

    For each row, the crossing is half a pixel outside its outermost pixel (between
    the last pixel centre in the blob and the first outside it); the same for each
    column. The mean error of a crossing is zero on a straight edge."""
    points, pixels = [], []
    for axis in (0, 1):  # 0: rows (u moves), 1: columns (v moves)
        major, minor = uv[:, 1 - axis], uv[:, axis]
        order = np.lexsort((minor, major))
        m = major[order]
        start = np.r_[0, np.nonzero(np.diff(m))[0] + 1]
        stop = np.r_[start[1:] - 1, len(m) - 1]
        for ends, step in ((start, -0.5), (stop, 0.5)):
            px = uv[order[ends]].astype(float)
            pt = px.copy()
            pt[:, axis] += step
            points.append(pt)
            pixels.append(px)
    return np.concatenate(points), np.concatenate(pixels)


def _hull(pts: np.ndarray) -> np.ndarray:
    """Indices of the convex hull vertices of 2-D points (monotone chain)."""
    order = np.lexsort((pts[:, 1], pts[:, 0]))
    q = pts[order].tolist()

    def chain(seq):
        out = []
        for i in seq:
            while len(out) >= 2:
                (ax, ay), (bx, by), (cx, cy) = q[out[-2]], q[out[-1]], q[i]
                if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) > 0:
                    break
                out.pop()
            out.append(i)
        return out

    idx = list(range(len(q)))
    hull = chain(idx)[:-1] + chain(idx[::-1])[:-1]
    return order[np.array(hull, int)]


def _tangent_basis(c: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    a = np.array([0.0, 1.0, 0.0]) if abs(c[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(a, c)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(c, e1)


def _plane_fit(rays: np.ndarray, w: np.ndarray) -> tuple[np.ndarray, float]:
    """Weighted least-squares plane n . p = d through unit rays: (centre n, angular radius).
    The plane cuts the unit sphere in a circle about n at angle acos(d)."""
    mean = (w[:, None] * rays).sum(0) / w.sum()
    a = np.sqrt(w)[:, None] * (rays - mean)
    # Normal = the least-variance direction: eigenvector of the 3 x 3 scatter matrix.
    # (An SVD of the N x 3 matrix computed an N x N factor: 72 ms per call on the Pi.)
    n = np.linalg.eigh(a.T @ a)[1][:, 0]
    d = float(n @ mean)
    if d < 0:
        n, d = -n, -d
    return n, math.acos(min(1.0, d))


def _span_deg(rays: np.ndarray, c: np.ndarray) -> float:
    """360 deg minus the largest gap between the rays' azimuths about c."""
    if len(rays) < 2:
        return 0.0
    e1, e2 = _tangent_basis(c)
    az = np.sort(np.arctan2(rays @ e2, rays @ e1))
    gaps = np.diff(np.r_[az, az[0] + 2 * math.pi])
    return math.degrees(2 * math.pi - gaps.max())


_MAX_VERTICES = 24  # hull vertices for candidate circles (C(24, 3) = 2024 triples)
_MAX_TEST = 160  # edge points that score each candidate
_OUT_FULL_PX = 0.75  # an edge point outside the circle has full weight up to this, none at fit_outside_px
_FADE_PX = 0.3  # weight of an edge point farther inside than fit_inner_px: Gaussian, this SD
_SCORE_SD_PX = 0.5  # candidate score: sum of exp(-0.5 (d / SD)^2) over the edge points on it (d <= 0.5 px) ...
_OUTSIDE_COST = 4.0  # ... minus this for each edge point outside: 0 at 0.5 px, all of it at fit_outside_px


def fit_silhouette(uv: np.ndarray, row0: int, p: VisionParams = VisionParams(),
                   invalid: np.ndarray | None = None, height: int = EQ_HEIGHT) -> SilhouetteFit | None:
    """Fit the limb circle to a blob's pixels (u, v), or None if no circle is found.

    Edge points on the first processed row, on the image border or next to
    invalid (outside the lens) pixels are cut edges, not the limb: they only
    count against a circle that they are outside. Candidate circles go through 3
    convex-hull vertices; the best has the most edge points on it and the fewest
    outside it. A weighted plane fit then refines it: edge points inside the
    circle (a white gore at the limb) lose weight, edge points far outside
    (noise) have none."""
    pts, src = _edge_points(uv)
    su, sv = src[:, 0].astype(int), src[:, 1].astype(int)
    cut = (sv <= row0) | (su <= 0) | (su >= EQ_WIDTH - 1) | (sv >= height - 1)
    if invalid is not None:
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                cut |= invalid[np.clip(sv + dy, 0, height - 1), np.clip(su + dx, 0, EQ_WIDTH - 1)]
    rays = pixel_rays(pts[:, 0], pts[:, 1], EQ_WIDTH, height)
    c0 = rays.mean(0)
    c0 /= np.linalg.norm(c0)
    if (rays @ c0).min() < 0.1:  # wider than 168 deg: not a ball
        return None
    e1, e2 = _tangent_basis(c0)
    hull = _hull(np.stack([rays @ e1, rays @ e2], 1) / (rays @ c0)[:, None])  # gnomonic plane: the limb is convex
    limb = hull[~cut[hull]]
    if len(limb) < 3:
        return None
    if len(limb) > _MAX_VERTICES:
        limb = limb[np.linspace(0, len(limb) - 1, _MAX_VERTICES).astype(int)]
    tri = np.array(list(itertools.combinations(range(len(limb)), 3)))
    a, b, c = (rays[limb[tri[:, k]]] for k in range(3))
    n = np.cross(b - a, c - a)
    norm = np.linalg.norm(n, axis=1)
    keep = norm > 1e-12
    n = n[keep] / norm[keep, None]
    d = np.einsum("ij,ij->i", n, a[keep])
    n[d < 0] *= -1
    rho = np.arccos(np.clip(np.abs(d), -1.0, 1.0))
    test = np.arange(len(rays))
    if len(test) > _MAX_TEST:
        test = np.union1d(test[np.linspace(0, len(test) - 1, _MAX_TEST).astype(int)], hull)
    dev = (_angles_many(n, rays[test]) - rho[:, None]) / PIX_RAD
    # reward for an edge point on the circle; cost for a point outside (cut edges get the cost only)
    reward = np.where(cut[test], 0.0, np.exp(-0.5 * (dev / _SCORE_SD_PX) ** 2))
    cost = _OUTSIDE_COST * np.clip((dev - 0.5) / (p.fit_outside_px - 0.5), 0.0, 1.0)
    score = (np.where(dev <= 0.5, reward, 0.0) - cost).sum(1)
    score[rho > math.radians(80.0)] = -math.inf
    best = int(np.argmax(score)) if len(score) else 0
    if not len(score) or score[best] < 3:
        return None
    centre, r = n[best], float(rho[best])
    for _ in range(8):  # weighted refit: points inside fade out, points far outside are outliers
        dev = (_angles(rays, centre) - r) / PIX_RAD
        w = np.where(dev >= -p.fit_inner_px, 1.0, np.exp(-0.5 * ((dev + p.fit_inner_px) / _FADE_PX) ** 2))
        w *= np.clip((p.fit_outside_px - dev) / (p.fit_outside_px - _OUT_FULL_PX), 0.0, 1.0)
        w[cut] = 0.0
        if w.sum() < 3:
            break
        centre, r = _plane_fit(rays, w)
    dev = (_angles(rays, centre) - r) / PIX_RAD
    on = (dev >= -p.fit_inner_px) & (dev <= p.fit_outside_px) & ~cut
    resid = float(np.sqrt(np.mean(dev[on] ** 2))) if on.any() else math.inf
    outside = float(np.mean(dev > p.fit_outside_px))
    return SilhouetteFit(centre, r, int(on.sum()), _span_deg(rays[on], centre), resid, outside,
                         len(uv) / max(1, _circle_area(centre, r, row0, height)))


def circle_points(centre: np.ndarray, rho: float, n: int = 72) -> np.ndarray:
    """(n, 3) unit rays on the circle at angle rho about centre."""
    e1, e2 = _tangent_basis(centre)
    t = np.linspace(0.0, 2 * math.pi, n, endpoint=False)[:, None]
    return math.cos(rho) * centre + math.sin(rho) * (np.cos(t) * e1 + np.sin(t) * e2)


def _circle_area(centre: np.ndarray, rho: float, row0: int, height: int) -> int:
    """Pixels inside the circle, in rows >= row0."""
    uv = ray_pixels(circle_points(centre, rho, 36), EQ_WIDTH, height)
    u0, u1 = max(0, int(uv[:, 0].min()) - 1), min(EQ_WIDTH - 1, int(uv[:, 0].max()) + 2)
    v0, v1 = max(row0, int(uv[:, 1].min()) - 1), min(height - 1, int(uv[:, 1].max()) + 2)
    if u1 < u0 or v1 < v0:
        return 0
    vv, uu = np.mgrid[v0:v1 + 1, u0:u1 + 1]
    return int(np.count_nonzero(pixel_rays(uu, vv, EQ_WIDTH, height) @ centre >= math.cos(rho)))


# --- geometry -------------------------------------------------------------------------


def ball_in_optical(box, width: int, height: int, radius: float, h_fov_deg: float = H_FOV_DEG):
    """Colour-box method: (ball centre in the optical frame [m], angular radius [rad]); centre None if invalid."""
    x1, y1, x2, y2 = box
    uc, vc = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    left, right = pixel_ray(x1, vc, width, height, h_fov_deg), pixel_ray(x2, vc, width, height, h_fov_deg)
    top, bottom = pixel_ray(uc, y1, width, height, h_fov_deg), pixel_ray(uc, y2, width, height, h_fov_deg)
    rho = max(_angle(left, right), _angle(top, bottom)) / 2.0
    if not 1e-4 < rho < math.pi / 2:
        return None, rho
    direction = left + right + top + bottom
    direction /= np.linalg.norm(direction)
    return radius / math.sin(rho) * direction, rho


def optical_to_body(p_optical: np.ndarray, mount: CameraMount, scale: float | None = None) -> np.ndarray:
    """Point from the optical frame to the body frame (x forward, y left, z up).
    The range from the camera is multiplied by ``scale`` (default mount.range_scale)."""
    s = mount.range_scale if scale is None else scale
    p = np.array([p_optical[2], -p_optical[0], -p_optical[1]]) * s
    a = math.radians(mount.tilt_up_deg)  # optical axis tilted up: rotate about -y
    p = np.array([math.cos(a) * p[0] - math.sin(a) * p[2], p[1], math.sin(a) * p[0] + math.cos(a) * p[2]])
    b = math.radians(mount.yaw_deg)
    p = np.array([math.cos(b) * p[0] - math.sin(b) * p[1], math.sin(b) * p[0] + math.cos(b) * p[1], p[2]])
    return p + np.array([mount.x, mount.y, mount.z])


def body_to_optical(p_body: np.ndarray, mount: CameraMount, scale: float | None = None) -> np.ndarray:
    """Inverse of optical_to_body."""
    s = mount.range_scale if scale is None else scale
    p = np.asarray(p_body, float) - np.array([mount.x, mount.y, mount.z])
    b = -math.radians(mount.yaw_deg)
    p = np.array([math.cos(b) * p[0] - math.sin(b) * p[1], math.sin(b) * p[0] + math.cos(b) * p[1], p[2]])
    a = -math.radians(mount.tilt_up_deg)
    p = np.array([math.cos(a) * p[0] - math.sin(a) * p[2], p[1], math.sin(a) * p[0] + math.cos(a) * p[2]]) / s
    return np.array([-p[1], -p[2], p[0]])


# --- detector -------------------------------------------------------------------------


class BallVision:
    """Decodes one camera JPEG and returns a BallFix or None. Not thread-safe:
    one caller at a time (kick_ball runs one frame at a time in a worker thread).

    After each frame: ``last_note`` tells the result or why a blob was refused,
    ``last_fit`` and ``last_blob`` hold the fit and the blob for ``save_snapshot``."""

    def __init__(self, params: VisionParams = VisionParams(), camera_yaml: Path = CAMERA_YAML):
        self.params = params
        self.camera_yaml = camera_yaml
        self.eq_height = EQ_HEIGHT
        self.row0 = int(self.eq_height * params.color.row_min_frac)
        self._model = None
        self._size: tuple[int, int] | None = None
        self._invalid: np.ndarray | None = None
        self.last_image: np.ndarray | None = None  # RGB floor image of the last decoded frame
        self.last_blob: BallBox | None = None
        self.last_fit: SilhouetteFit | None = None
        self.last_fix: BallFix | None = None
        self.last_note = ""

    def _camera_model(self, w: int, h: int):
        """Lens model for a half-size frame (camera_params.yaml is for the full frame)."""
        if self._size != (w, h):
            from fisheye_converter import load_camera_model

            model = load_camera_model(str(self.camera_yaml), w, h)
            model.fx, model.fy = model.fx * 0.5, model.fy * 0.5
            model.cx, model.cy = (model.cx + 0.5) * 0.5 - 0.5, (model.cy + 0.5) * 0.5 - 0.5
            self._model, self._size = model, (w, h)
        return self._model

    def floor_image(self, bgr_half: np.ndarray) -> np.ndarray:
        """RGB equirectangular image; only the floor rows (>= row0) hold pixels."""
        import cv2
        from fisheye_converter import equirectangular_maps

        h, w = bgr_half.shape[:2]
        map_x, map_y, invalid = equirectangular_maps(self._camera_model(w, h), EQ_WIDTH, self.eq_height, H_FOV_DEG)
        self._invalid = invalid
        r = self.row0
        eq = np.zeros((self.eq_height, EQ_WIDTH, 3), np.uint8)
        eq[r:] = cv2.remap(bgr_half, map_x[r:], map_y[r:], interpolation=cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
        eq[r:][invalid[r:]] = 0
        return cv2.cvtColor(eq, cv2.COLOR_BGR2RGB)

    def _group(self, blob: BallBox, blobs, invalid) -> tuple[BallBox, SilhouetteFit | None]:
        """The blob, joined by the other blobs inside its fitted circle, and the fit of the group."""
        p = self.params
        fit = fit_silhouette(blob.uv, self.row0, p, invalid, self.eq_height) if blob.uv is not None else None
        if fit is None:
            return blob, None
        cos_out = math.cos(fit.rho + p.fit_outside_px * PIX_RAD)
        members = [blob] + [b for b in blobs if b is not blob and b.uv is not None and np.mean(
            pixel_rays(b.uv[:, 0], b.uv[:, 1], EQ_WIDTH, self.eq_height) @ fit.centre >= cos_out) >= 0.9]
        if len(members) == 1:
            return blob, fit
        group = _merge(members)
        return group, fit_silhouette(group.uv, self.row0, p, invalid, self.eq_height)

    def _colours_near(self, blob: BallBox, blobs) -> bool:
        """True if the blob has both colours, or a blob of the other colour is within one
        blob size of it."""
        total = blob.n_red + blob.n_blue
        if total and min(blob.n_red, blob.n_blue) / total >= self.params.fit_min_each_frac:
            return True
        red = blob.n_red >= blob.n_blue
        x1, y1, x2, y2 = blob.box
        m = max(x2 - x1, y2 - y1)
        for b in blobs:
            if b is blob or not (b.n_blue if red else b.n_red):
                continue
            u1, v1, u2, v2 = b.box
            if u1 <= x2 + m and u2 >= x1 - m and v1 <= y2 + m and v2 >= y1 - m:
                return True
        return False

    def _box_aspect(self, box) -> float:
        x1, y1, x2, y2 = box
        uc, vc = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        h = EQ_WIDTH, self.eq_height
        wide = _angle(pixel_ray(x1, vc, *h), pixel_ray(x2, vc, *h))
        tall = _angle(pixel_ray(uc, y1, *h), pixel_ray(uc, y2, *h))
        return max(wide, tall) / max(min(wide, tall), 1e-9)

    def locate(self, blob: BallBox, blobs=(), invalid: np.ndarray | None = None) -> BallFix | None:
        """BallFix for one blob (blobs inside its fitted circle join it), or None.
        ``last_note`` tells the method, or why there is no fix."""
        p = self.params
        blob, fit = self._group(blob, blobs, invalid)
        self.last_blob, self.last_fit = blob, fit
        problem = "no circle" if fit is None else fit.problem(p)
        total = blob.n_red + blob.n_blue
        share = min(blob.n_red, blob.n_blue) / total if total else 0.0
        if problem is None:
            if share < p.fit_min_each_frac and not p.single_colour:
                self.last_note = f"fit: one colour (share {share:.3f})"
                return None
            rho = fit.rho + p.edge_px * PIX_RAD
            p_opt, scale, method = p.ball_radius / math.sin(rho) * fit.centre, p.mount.range_scale, "fit"
        else:
            if share < p.color.min_each_frac:
                self.last_note = f"fit not reliable ({problem}); box: one colour (share {share:.3f})"
                return None
            aspect = self._box_aspect(blob.box)
            if aspect > p.box_max_aspect:
                self.last_note = f"fit not reliable ({problem}); box: aspect {aspect:.1f}"
                return None
            x1, y1, x2, y2 = blob.box
            if x1 <= 1 or x2 >= EQ_WIDTH - 1:
                self.last_note = f"fit not reliable ({problem}); box: touches the image edge"
                return None
            fill = total / max(1.0, (x2 - x1) * (y2 - y1))
            if fill < p.box_min_fill:
                self.last_note = f"fit not reliable ({problem}); box: fill {fill:.2f}"
                return None
            p_opt, rho = ball_in_optical(blob.box, EQ_WIDTH, self.eq_height, p.ball_radius)
            scale, method = p.box_scale, "box"
            if p_opt is None:
                self.last_note = f"fit not reliable ({problem}); box: no size"
                return None
        x, y, z = optical_to_body(p_opt, p.mount, scale)
        y -= p.y_bias
        rng = math.hypot(x, y)
        self.last_note = f"{method}{'' if problem is None else f' ({problem})'}: x {x:.3f} y {y:.3f} z {z:.3f}"
        if not p.min_range <= rng <= p.max_range or (method == "box" and rng < p.box_min_range):
            self.last_note += ": range out of limits"
            return None
        if abs(z - p.ball_z) > p.z_tol + p.z_tol_per_m * rng:
            self.last_note += f": not on the floor (ball_z {p.ball_z:.3f})"
            return None
        return BallFix(float(x), float(y), float(z), blob.box, blob.pixels, method, float(rho))

    def detect_masks(self, red: np.ndarray, blue: np.ndarray, invalid: np.ndarray | None = None) -> BallFix | None:
        """BallFix from red and blue masks of the equirectangular image (rows below
        row0 are used), or None. ``invalid``: pixels outside the lens image."""
        p = self.params
        self.last_blob = self.last_fit = self.last_fix = None
        self.last_note = "no blob"
        if p.color.denoise:
            red, blue = majority3(red), majority3(blue)
        blobs = find_blobs(red, blue, p.color, self.row0)
        notes = []
        # CPU: a one-colour blob (a shirt, a key) is fitted only when a blob of the other
        # colour is near it (a gore of the ball); otherwise it can never give a fix.
        seeds = [b for b in blobs if p.single_colour or self._colours_near(b, blobs)]
        if len(seeds) < len(blobs):
            notes.append(f"{len(blobs) - len(seeds)} blobs of one colour skipped")
        # CPU: a blob larger than the ball at its nearest visible range (a floor mat)
        # is not fitted. 2 x asin(R / near_range) in pixels, with a 30 % margin.
        big_px = 1.3 * 2.0 * math.asin(min(1.0, p.ball_radius / p.near_range)) / PIX_RAD
        sized = [b for b in seeds if max(b.box[2] - b.box[0], b.box[3] - b.box[1]) <= big_px]
        if len(sized) < len(seeds):
            notes.append(f"{len(seeds) - len(sized)} blobs larger than a ball skipped")
        seeds = sized
        for blob in seeds[:p.max_blobs]:
            fix = self.locate(blob, blobs, invalid)
            if fix is not None:
                self.last_fix = fix
                LOG.debug("ball_vision: %s", self.last_note)
                return fix
            notes.append(self.last_note)
        if notes:
            self.last_note = " | ".join(notes)
        LOG.debug("ball_vision: no fix: %s", self.last_note)
        return None

    def detect(self, jpeg: bytes) -> BallFix | None:
        import cv2

        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        if bgr is None:
            self.last_note = "JPEG not decoded"
            return None
        self.last_image = self.floor_image(bgr)
        red, blue = classify(self.last_image, self.params.color)
        return self.detect_masks(red, blue, self._invalid)

    def save_snapshot(self, path) -> bool:
        """Write the last floor image (BGR JPEG) with the blob box (yellow) and the fitted
        circle (green: fix, red: refused) for calibration. Logs and returns False on error."""
        try:
            import cv2

            if self.last_image is None:
                raise ValueError("no frame processed")
            img = cv2.cvtColor(self.last_image, cv2.COLOR_RGB2BGR)
            cv2.line(img, (0, self.row0), (EQ_WIDTH - 1, self.row0), (128, 128, 128), 1)
            if self.last_blob is not None:
                x1, y1, x2, y2 = (int(round(c)) for c in self.last_blob.box)
                cv2.rectangle(img, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 255), 1)
            if self.last_fit is not None:
                rho = self.last_fit.rho + self.params.edge_px * PIX_RAD
                uv = ray_pixels(circle_points(self.last_fit.centre, rho), EQ_WIDTH, self.eq_height)
                colour = (0, 255, 0) if self.last_fix is not None else (0, 0, 255)
                cv2.polylines(img, [np.round(uv).astype(np.int32)], True, colour, 1)
            cv2.putText(img, self.last_note[:90], (4, self.row0 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (255, 255, 255), 1)
            if not cv2.imwrite(str(path), img):
                raise OSError("cv2.imwrite returned False")
            return True
        except Exception as error:  # a snapshot never stops the caller
            LOG.warning("ball_vision: snapshot %s not written: %s", path, error)
            return False

