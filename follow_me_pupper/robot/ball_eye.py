"""Ball Eye: metric body-frame ball measurements from the onboard fisheye camera.

Maintained port of the recovered u6 Ball Eye geometry (source sha256
fd2559408155c9c94c0af3706a88d46af6bb646471d4ebf782ddeba8ed351f73). One frame
-> zero or more ball centres on base_link (x ahead, y left, z up):

  1. Floor view. The fisheye frame (double sphere model) is remapped to a
     stereographic view looking ahead and down. Stereographic projection maps
     every circle on the sphere of camera rays to an image circle, so a ball's
     outline stays round at every position, including next to the nose.
  2. Proposals. Ball Eye's one-class YOLOv8n (CPU, OpenCV DNN; training
     provenance unknown), the maintained pink tracker's panorama box converted
     to a cone of rays, or pink colour components found in the floor view.
  3. Outline fit. Along rays from the proposal centre, the outermost
     ball-coloured pixel is an outline point; colours are learned per frame
     from the proposal centre against a surrounding ring. A robust circle fit
     ignores dents.
  4. Geometry. The circle gives the direction of the ball centre and its
     angular radius rho; range = R / sin(rho). No camera height is needed, but
     range scales linearly with the selected physical radius R.

This module measures only. It attaches no clock: callers pair a result with
the source capture time of the frame they passed in. There is no tracker,
prediction or online calibration; consumers own admission and expiry.

Uncertainty fields are an image-noise model propagated through the geometry.
They exclude radius, mount, lens and exposure-time error, which stay unknown
until measured; ``MetricProfile`` records which inputs were measured.

Frames: optical (x right, y down, z ahead) -> view (optical turned down by
View.pitch_deg) and -> body (Mount). NumPy always, OpenCV for image work.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

PROFILE_SCHEMA = "fc_onboard_metric_profile_v1"
PROFILE_STATUSES = ("synthetic", "estimated", "measured")
RECOVERED_SOURCE_SHA256 = "fd2559408155c9c94c0af3706a88d46af6bb646471d4ebf782ddeba8ed351f73"
RECOVERED_MODEL_SHA256 = "ae3a4592a59b3b30957f5a42b6e1f87495e61c7e2efd9b963670abcd590a827c"
MODEL_INPUT = 320  # the recovered .onnx has a fixed 320 x 320 input


# ---------------------------------------------------------------- camera model


@dataclass(frozen=True)
class Lens:
    """Double sphere fisheye model (Usenko 2018) for a full 1400 x 1050 frame."""

    fx: float
    fy: float
    cx: float
    cy: float
    xi: float
    alpha: float
    width: int = 1400
    height: int = 1050
    max_angle_deg: float = 92.0  # edge of the lit image circle (u6 sample frame, 2026-11-19)

    @classmethod
    def from_camera_params(cls, path) -> "Lens":
        import yaml

        p = (yaml.safe_load(Path(path).read_text()) or {}).get("camera_params", {})
        return cls(**{k: float(p[k]) for k in ("fx", "fy", "cx", "cy", "xi", "alpha")})

    def record(self) -> dict:
        return {k: getattr(self, k) for k in ("fx", "fy", "cx", "cy", "xi", "alpha", "width", "height",
                                              "max_angle_deg")}

    def scaled(self, width: int, height: int) -> "Lens":
        """The same lens for a frame decoded at another size (pixel-edge convention)."""
        sx, sy = width / self.width, height / self.height
        return replace(self, fx=self.fx * sx, fy=self.fy * sy, cx=(self.cx + 0.5) * sx - 0.5,
                       cy=(self.cy + 0.5) * sy - 0.5, width=width, height=height)

    def project(self, rays: np.ndarray):
        """Rays (..., 3) in the optical frame -> pixel u, v and a validity mask."""
        x, y, z = rays[..., 0], rays[..., 1], rays[..., 2]
        d1 = np.sqrt(x * x + y * y + z * z)
        k = self.xi * d1 + z
        d2 = np.sqrt(x * x + y * y + k * k)
        denom = self.alpha * d2 + (1.0 - self.alpha) * k
        valid = (denom > 1e-6) & (z / np.maximum(d1, 1e-12) > math.cos(math.radians(self.max_angle_deg)))
        denom = np.maximum(denom, 1e-6)
        u, v = self.fx * x / denom + self.cx, self.fy * y / denom + self.cy
        valid &= (u >= 0) & (u <= self.width - 1) & (v >= 0) & (v <= self.height - 1)
        return u, v, valid

    def rays(self):
        """Unit optical rays (height, width, 3) of every pixel and the mask the lens sees
        (unprojection: Usenko 2018, equations 46 to 50)."""
        u, v = np.meshgrid(np.arange(self.width, dtype=float), np.arange(self.height, dtype=float))
        mx, my = (u - self.cx) / self.fx, (v - self.cy) / self.fy
        r2 = mx * mx + my * my
        seen = r2 < (1.0 / (2.0 * self.alpha - 1.0) if self.alpha > 0.5 else np.inf)
        mz = (1.0 - self.alpha ** 2 * r2) / (self.alpha * np.sqrt(np.maximum(1.0 - (2.0 * self.alpha - 1.0) * r2, 0.0))
                                             + 1.0 - self.alpha)
        k = (mz * self.xi + np.sqrt(mz * mz + (1.0 - self.xi ** 2) * r2)) / (mz * mz + r2)
        rays = np.stack([k * mx, k * my, k * mz - self.xi], axis=-1)
        return rays, seen & (rays[..., 2] > math.cos(math.radians(self.max_angle_deg)))


@dataclass(frozen=True)
class Circle:
    """A circle in the floor view [pixels]."""

    u: float
    v: float
    r: float


@dataclass(frozen=True)
class View:
    """Stereographic floor view: image radius = 2 f tan(angle from the view axis / 2)."""

    size: int = 640
    half_fov_deg: float = 72.0  # angle from the view axis at the middle of an image side
    pitch_deg: float = 40.0  # view axis below the optical axis

    def record(self) -> dict:
        return {"size": self.size, "half_fov_deg": self.half_fov_deg, "pitch_deg": self.pitch_deg}

    @property
    def f(self) -> float:
        return (self.size / 2.0) / (2.0 * math.tan(math.radians(self.half_fov_deg) / 2.0))

    @property
    def c(self) -> float:
        return (self.size - 1) / 2.0

    def _rot(self) -> np.ndarray:
        """Columns: the view x, y, z axes in the optical frame."""
        s, c = math.sin(math.radians(self.pitch_deg)), math.cos(math.radians(self.pitch_deg))
        return np.array([[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]])

    def ray(self, u, v) -> np.ndarray:
        """View pixels -> unit rays in the optical frame."""
        x, y = (np.asarray(u, float) - self.c) / self.f, (np.asarray(v, float) - self.c) / self.f
        q = x * x + y * y
        d = np.stack([4.0 * x, 4.0 * y, 4.0 - q], axis=-1) / (4.0 + q)[..., None]
        return d @ self._rot().T

    def pixel(self, rays: np.ndarray):
        """Unit rays in the optical frame -> view pixels (u, v)."""
        d = np.asarray(rays, float) @ self._rot()
        scale = 2.0 * self.f / np.maximum(1.0 + d[..., 2], 1e-9)
        return d[..., 0] * scale + self.c, d[..., 1] * scale + self.c

    def maps(self, lens: Lens):
        """cv2.remap tables fisheye -> view, and the mask of view pixels the lens sees."""
        u, v = np.meshgrid(np.arange(self.size), np.arange(self.size))
        fu, fv, valid = lens.project(self.ray(u, v))
        return fu.astype(np.float32), fv.astype(np.float32), valid

    def cone(self, circle: Circle):
        """View circle -> (unit axis in the optical frame, angular radius rho [rad])."""
        mx, my = (circle.u - self.c) / self.f, (circle.v - self.c) / self.f
        m, r = math.hypot(mx, my), circle.r / self.f
        ex, ey = (mx / m, my / m) if m > 1e-9 else (1.0, 0.0)
        near, far = 2.0 * math.atan((m - r) / 2.0), 2.0 * math.atan((m + r) / 2.0)
        mid, rho = (near + far) / 2.0, (far - near) / 2.0
        axis = np.array([math.sin(mid) * ex, math.sin(mid) * ey, math.cos(mid)])
        return axis @ self._rot().T, rho

    def circle(self, axis: np.ndarray, rho: float) -> Circle:
        """Cone of rays (unit axis in the optical frame, angular radius) -> view circle."""
        d = np.asarray(axis, float) @ self._rot()
        mid = math.acos(max(-1.0, min(1.0, float(d[2]))))
        m = math.hypot(d[0], d[1])
        ex, ey = (d[0] / m, d[1] / m) if m > 1e-9 else (1.0, 0.0)
        near, far = 2.0 * math.tan((mid - rho) / 2.0), 2.0 * math.tan((mid + rho) / 2.0)
        centre, r = (near + far) / 2.0, (far - near) / 2.0
        return Circle(centre * ex * self.f + self.c, centre * ey * self.f + self.c, r * self.f)


@dataclass(frozen=True)
class Pano:
    """The FC camera node's 220-degree equirectangular detection image
    (fisheye_converter.create_equirectangular_rays): equal degrees per pixel."""

    width: int = 700
    h_fov_deg: float = 220.0
    round_tol: float = 0.2  # the two spherical radii of a whole ball agree within this fraction

    @property
    def height(self) -> int:
        return max(2, int(round(self.width * 180.0 / self.h_fov_deg)) // 2 * 2)

    def angles(self, x: float, y: float) -> tuple[float, float]:
        """Normalized box coordinates (pixel edges) -> (longitude right, latitude down) [rad]."""
        w, h = self.width, self.height
        return (((x * w - 0.5) / (w - 1) - 0.5) * math.radians(self.h_fov_deg),
                ((y * h - 0.5) / (h - 1) - 0.5) * math.pi)

    def cone(self, box):
        """Normalized box (x1, y1, x2, y2) -> (unit optical axis, rho, whole).

        A ball outline is a circle on the ray sphere: its latitude limits are the
        centre latitude +/- rho and its longitude half-width w obeys
        sin(rho) = sin(w) cos(latitude). This replaces a pixel aspect test, which
        rejects close spherical balls. whole is False for a cut, merged or
        pole-crossing box: a search seed that needs an outline fit, not a position.
        """
        (lon1, lat1), (lon2, lat2) = self.angles(box[0], box[1]), self.angles(box[2], box[3])
        lon0, lat0, half_lon, rho_v = (lon1 + lon2) / 2.0, (lat1 + lat2) / 2.0, (lon2 - lon1) / 2.0, (lat2 - lat1) / 2.0
        rho_h = math.asin(min(1.0, math.sin(min(half_lon, math.pi / 2.0)) * math.cos(lat0)))
        edge = 1.5 / self.width
        whole = (abs(rho_v - rho_h) <= self.round_tol * min(rho_v, rho_h) and half_lon < math.radians(80.0)
                 and lat0 + rho_v < math.radians(85.0) and box[0] > edge and box[2] < 1.0 - edge and box[3] < 1.0 - edge)
        rho = (rho_v + rho_h) / 2.0 if whole else max(rho_v, rho_h)
        axis = np.array([math.cos(lat0) * math.sin(lon0), math.sin(lat0), math.cos(lat0) * math.cos(lon0)])
        return axis, rho, whole

    def box(self, axis: np.ndarray, rho: float) -> tuple[float, float, float, float]:
        """Cone of rays -> its normalized box (synthetic data; a ball short of the pole)."""
        lat0, lon0 = math.asin(float(axis[1])), math.atan2(float(axis[0]), float(axis[2]))
        half_lon = math.asin(min(1.0, math.sin(rho) / math.cos(lat0)))
        w, h = self.width, self.height
        fx = lambda lon: ((lon / math.radians(self.h_fov_deg) + 0.5) * (w - 1) + 0.5) / w  # noqa: E731
        fy = lambda lat: ((lat / math.pi + 0.5) * (h - 1) + 0.5) / h  # noqa: E731
        return fx(lon0 - half_lon), fy(lat0 - rho), fx(lon0 + half_lon), fy(lat0 + rho)


@dataclass(frozen=True)
class Mount:
    """Camera pose on base_link. Values come only from a selected profile."""

    x: float  # m ahead
    y: float  # m left
    z: float  # m up
    tilt_up_deg: float
    yaw_deg: float  # optical axis to the left of the body x axis

    def record(self) -> dict:
        return {k: getattr(self, k) for k in ("x", "y", "z", "tilt_up_deg", "yaw_deg")}

    def rot(self) -> np.ndarray:
        """Columns: the optical x, y, z axes in the body frame."""
        t, w = math.radians(self.tilt_up_deg), math.radians(self.yaw_deg)
        tilt = np.array([[0.0, math.sin(t), math.cos(t)], [-1.0, 0.0, 0.0], [0.0, -math.cos(t), math.sin(t)]])
        yaw = np.array([[math.cos(w), -math.sin(w), 0.0], [math.sin(w), math.cos(w), 0.0], [0.0, 0.0, 1.0]])
        return yaw @ tilt

    def to_body(self, p_optical: np.ndarray) -> np.ndarray:
        return self.rot() @ np.asarray(p_optical, float) + np.array([self.x, self.y, self.z])

    def to_optical(self, p_body: np.ndarray) -> np.ndarray:
        return self.rot().T @ (np.asarray(p_body, float) - np.array([self.x, self.y, self.z]))


# u6 estimate (2026-11-16), never measured. Only an "estimated" profile may use it.
ESTIMATED_MOUNT = Mount(0.15, 0.0, 0.03, 10.0, 0.0)
ESTIMATED_BASE_HEIGHT_M = 0.15  # simulation range 0.14 to 0.165


# ---------------------------------------------------------------- selected profile


def _sha256_json(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _positive(value, label):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be a finite positive number")
    return float(value)


def _finite(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


@dataclass(frozen=True)
class MetricProfile:
    """One selected physical ball radius, camera mount and lens, fixed for a session.

    profile_id hashes the whole selection; calibration_id hashes the geometry
    (lens, mount, view, base height). Consumers compare these identities rather
    than trusting a number. Nothing here is learned or updated during play.
    """

    status: str
    ball_radius_m: float
    radius_source: str
    radius_measured: bool
    mount: Mount
    mount_source: str
    mount_measured: bool
    base_height_m: float
    lens: Lens
    view: View = View()

    def __post_init__(self):
        if self.status not in PROFILE_STATUSES:
            raise ValueError("profile status must be synthetic, estimated or measured")
        _positive(self.ball_radius_m, "ball radius")
        _positive(self.base_height_m, "base height")
        if not 0.02 <= self.ball_radius_m <= 0.25:
            raise ValueError("ball radius is outside the plausible 0.02 to 0.25 m range")
        if type(self.radius_measured) is not bool or type(self.mount_measured) is not bool:
            raise ValueError("measurement flags must be booleans")
        if self.status == "measured" and not (self.radius_measured and self.mount_measured):
            raise ValueError("a measured profile requires a measured radius and mount")
        for value, label in ((self.radius_source, "radius source"), (self.mount_source, "mount source")):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{label} must name where the value came from")

    def geometry_record(self) -> dict:
        return {"lens": self.lens.record(), "mount": self.mount.record(), "view": self.view.record(),
                "base_height_m": self.base_height_m}

    def record(self) -> dict:
        return {"schema": PROFILE_SCHEMA, "status": self.status,
                "ball": {"radius_m": self.ball_radius_m, "source": self.radius_source,
                         "measured": self.radius_measured},
                "mount": {**self.mount.record(), "source": self.mount_source, "measured": self.mount_measured},
                "base_height_m": self.base_height_m, "lens": self.lens.record(), "view": self.view.record()}

    @property
    def calibration_id(self) -> str:
        return _sha256_json(self.geometry_record())

    @property
    def profile_id(self) -> str:
        return _sha256_json(self.record())

    def identity(self) -> dict:
        return {"profile_id": self.profile_id, "calibration_id": self.calibration_id, "status": self.status,
                "ball_radius_m": self.ball_radius_m, "radius_measured": self.radius_measured,
                "mount_measured": self.mount_measured}

    @classmethod
    def read(cls, value, *, camera_params=None) -> "MetricProfile":
        """Parse a profile record. A lens may be inline or read from camera_params."""
        if not isinstance(value, dict) or value.get("schema") != PROFILE_SCHEMA:
            raise ValueError(f"metric profile requires schema {PROFILE_SCHEMA}")
        ball, mount = value.get("ball"), value.get("mount")
        if not isinstance(ball, dict) or not isinstance(mount, dict):
            raise ValueError("metric profile requires ball and mount records")
        lens = value.get("lens")
        if lens is not None:
            lens = Lens(**{k: _finite(lens[k], k) for k in ("fx", "fy", "cx", "cy", "xi", "alpha")},
                        width=int(lens.get("width", 1400)), height=int(lens.get("height", 1050)),
                        max_angle_deg=_positive(lens.get("max_angle_deg", 92.0), "max angle"))
        elif camera_params is not None:
            lens = Lens.from_camera_params(camera_params)
        else:
            raise ValueError("metric profile requires an inline lens or camera_params")
        view = value.get("view")
        view = View(int(view["size"]), _positive(view["half_fov_deg"], "view fov"),
                    _finite(view["pitch_deg"], "view pitch")) if view is not None else View()
        return cls(status=value.get("status"), ball_radius_m=_positive(ball.get("radius_m"), "ball radius"),
                   radius_source=ball.get("source"), radius_measured=ball.get("measured"),
                   mount=Mount(*(_finite(mount.get(k), "mount " + k) for k in ("x", "y", "z", "tilt_up_deg", "yaw_deg"))),
                   mount_source=mount.get("source"), mount_measured=mount.get("measured"),
                   base_height_m=_positive(value.get("base_height_m"), "base height"), lens=lens, view=view)


# ---------------------------------------------------------------- measurement geometry


def circle_to_body(view: View, mount: Mount, circle: Circle, ball_radius: float):
    """View circle -> (ball centre on base_link, camera range, rho)."""
    axis, rho = view.cone(circle)
    rng = ball_radius / math.sin(max(rho, 1e-6))
    return mount.to_body(axis * rng), rng, rho


def body_to_circle(view: View, mount: Mount, p_body, ball_radius: float) -> Circle:
    """Ball centre on base_link -> its outline in the view (tests and synthetic data)."""
    p = mount.to_optical(p_body)
    rng = float(np.linalg.norm(p))
    return view.circle(p / rng, math.asin(min(1.0, ball_radius / rng)))


def image_uncertainty(view: View, mount: Mount, circle: Circle, ball_radius: float, sigma_px: float):
    """1-sigma body-frame spread from independent sigma_px error on circle u, v and r.

    Linearized by central differences. Returns (sigma_xy_m, sigma_range_m): the
    largest horizontal standard deviation and the range standard deviation.
    Radius, mount, lens and exposure errors are deliberately not included.
    """
    h = 0.25
    cols, rcol = [], []
    for du, dv, dr in ((h, 0, 0), (0, h, 0), (0, 0, h)):
        plus = Circle(circle.u + du, circle.v + dv, circle.r + dr)
        minus = Circle(circle.u - du, circle.v - dv, max(circle.r - dr, 1e-3))
        (p1, r1, _), (p2, r2, _) = (circle_to_body(view, mount, c, ball_radius) for c in (plus, minus))
        cols.append((p1 - p2) / (2 * h))
        rcol.append((r1 - r2) / (2 * h))
    jac = np.column_stack(cols)[:2]  # d(x, y) / d(u, v, r)
    cov = jac @ jac.T * sigma_px ** 2
    sigma_xy = math.sqrt(max(float(np.linalg.eigvalsh(cov).max()), 0.0))
    sigma_range = math.sqrt(sum(c * c for c in rcol)) * sigma_px
    return sigma_xy, sigma_range


# ---------------------------------------------------------------- YOLO backend (CPU)


def decode_yolov8(out: np.ndarray, conf: float, iou: float = 0.5, max_det: int = 5):
    """Raw YOLOv8 head (1, 4 + classes, N), cxcywh in input pixels -> [(x1, y1, x2, y2, score, class)]."""
    import cv2

    p = np.asarray(out)[0].T
    scores, classes = p[:, 4:].max(axis=1), p[:, 4:].argmax(axis=1)
    keep = scores >= conf
    p, scores, classes = p[keep], scores[keep], classes[keep]
    if not len(p):
        return []
    xywh = np.stack([p[:, 0] - p[:, 2] / 2, p[:, 1] - p[:, 3] / 2, p[:, 2], p[:, 3]], axis=1)
    idx = np.asarray(cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), conf, iou)).reshape(-1)[:max_det]
    return [(float(xywh[i, 0]), float(xywh[i, 1]), float(xywh[i, 0] + xywh[i, 2]), float(xywh[i, 1] + xywh[i, 3]),
             float(scores[i]), int(classes[i])) for i in idx]


class OnnxBackend:
    """Ball Eye YOLOv8n .onnx on the CPU through OpenCV DNN, identified by content hash.

    The bytes that are hashed are the bytes that are loaded, so a file swapped
    between hashing and loading cannot run under the expected identity.
    """

    def __init__(self, path, *, expected_sha256=None, conf: float = 0.35, size: int = MODEL_INPUT):
        import cv2

        data = Path(path).read_bytes()
        self.sha256 = hashlib.sha256(data).hexdigest()
        if expected_sha256 is not None and self.sha256 != expected_sha256:
            raise ValueError("Ball Eye model content does not match the selected sha256")
        self.net = cv2.dnn.readNetFromONNX(np.frombuffer(data, np.uint8))
        self.conf, self.size = conf, size

    def infer(self, rgb: np.ndarray):
        import cv2

        if rgb.shape[:2] != (self.size, self.size):
            raise ValueError(f"Ball Eye model input must be {self.size} x {self.size}")
        self.net.setInput(cv2.dnn.blobFromImage(rgb, 1.0 / 255.0, (self.size, self.size), swapRB=False))
        return decode_yolov8(self.net.forward(), self.conf)


# ---------------------------------------------------------------- outline fit


@dataclass(frozen=True)
class FitParams:
    rays: int = 96
    inner: float = 0.55  # ball colour samples: inside this fraction of the proposal radius
    ring: tuple[float, float] = (1.2, 1.45)  # background samples and the search limit
    bins: int = 24
    smooth_bins: float = 2.0
    on_px: float = 2.0
    out_px: float = 3.5
    min_points: int = 14
    min_arc_deg: float = 110.0
    max_rms_px: float = 1.3
    trials: int = 300
    min_r_px: float = 30.0  # smaller (far) balls: the proposal circle is as good
    box_agree: float = 0.25  # a whole proposal and its fit agree in centre and radius within this fraction
    seed_agree: float = 0.6  # a cut/seeded fit stays within this fraction of the seed radius (u6 lacked this)


def fit_circle(pts: np.ndarray):
    """Least-squares circle (Kasa) through points (N, 2) -> (u, v, r)."""
    a = np.column_stack([2.0 * pts, np.ones(len(pts))])
    sol, *_ = np.linalg.lstsq(a, (pts * pts).sum(axis=1), rcond=None)
    return sol[0], sol[1], math.sqrt(max(sol[2] + sol[0] ** 2 + sol[1] ** 2, 1e-9))


def outline_points(rgb: np.ndarray, valid: np.ndarray, circle: Circle, p: FitParams = FitParams()):
    """Outermost ball-coloured pixel on each ray from the proposal centre -> points (N, 2)."""
    import cv2

    size = rgb.shape[0]
    reach = circle.r * p.ring[1]
    x0, y0 = max(0, int(circle.u - reach)), max(0, int(circle.v - reach))
    x1, y1 = min(size, int(circle.u + reach) + 2), min(size, int(circle.v + reach) + 2)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return np.empty((0, 2))
    crop, ok = rgb[y0:y1, x0:x1], valid[y0:y1, x0:x1]
    ab = (cv2.cvtColor(np.ascontiguousarray(crop), cv2.COLOR_RGB2LAB)[..., 1:].astype(np.int32) * p.bins) // 256
    yy, xx = np.mgrid[y0:y1, x0:x1]
    dist = np.hypot(xx - circle.u, yy - circle.v) / circle.r
    ball_px, back_px = ok & (dist < p.inner), ok & (dist > p.ring[0]) & (dist < p.ring[1])
    if ball_px.sum() < 20 or back_px.sum() < 20:
        return np.empty((0, 2))
    code = ab[..., 0] * p.bins + ab[..., 1]
    h_ball = np.bincount(code[ball_px], minlength=p.bins * p.bins) / ball_px.sum()
    h_back = np.bincount(code[back_px], minlength=p.bins * p.bins) / back_px.sum()
    # Smoothed histograms assign a blurred edge colour to the nearer side, so the
    # outline lands mid-edge rather than inside it.
    h_ball, h_back = (cv2.GaussianBlur(h.reshape(p.bins, p.bins).astype(np.float32), (0, 0), p.smooth_bins).ravel()
                      for h in (h_ball, h_back))
    is_ball = (h_ball > h_back)[code] & ok
    is_ball = cv2.morphologyEx(is_ball.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0
    ang = np.linspace(0.0, 2.0 * math.pi, p.rays, endpoint=False)
    radii = np.arange(0.4 * circle.r, reach, 0.5)
    px = circle.u + np.cos(ang)[:, None] * radii[None, :]
    py = circle.v + np.sin(ang)[:, None] * radii[None, :]
    ix, iy = np.round(px).astype(int) - x0, np.round(py).astype(int) - y0
    inside = (ix >= 0) & (ix < x1 - x0) & (iy >= 0) & (iy < y1 - y0)
    ixc, iyc = np.clip(ix, 0, x1 - x0 - 1), np.clip(iy, 0, y1 - y0 - 1)
    hit, seen = is_ball[iyc, ixc] & inside, ok[iyc, ixc] & inside
    last = hit.shape[1] - 1 - np.argmax(hit[:, ::-1], axis=1)
    rows = np.arange(p.rays)
    # Drop rays with no hit, hits up to the search limit (colour leak), or whose
    # next pixels leave the lens/image (a cut ball, not its outline).
    after = np.clip(last + 3, 0, hit.shape[1] - 1)
    good = hit.any(axis=1) & (last < hit.shape[1] - 3) & seen[rows, after]
    return np.column_stack([px[rows, last], py[rows, last]])[good]


@dataclass(frozen=True)
class Fit:
    circle: Circle
    rms_px: float
    points: int


def refine_circle(rgb: np.ndarray, valid: np.ndarray, circle: Circle, p: FitParams = FitParams()) -> Fit | None:
    """Outline fit: the circle with the most outline points on it and few outside it.

    Deterministic: the RANSAC sample generator is seeded per call.
    """
    if circle.r < p.min_r_px:
        return None
    pts = outline_points(rgb, valid, circle, p)
    if len(pts) < p.min_points:
        return None
    tri = pts[np.random.default_rng(0).integers(0, len(pts), size=(p.trials, 3))]
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    d = 2.0 * (a[:, 0] * (b[:, 1] - c[:, 1]) + b[:, 0] * (c[:, 1] - a[:, 1]) + c[:, 0] * (a[:, 1] - b[:, 1]))
    okd = np.abs(d) > 1e-6
    d = np.where(okd, d, 1.0)
    a2, b2, c2 = (a * a).sum(1), (b * b).sum(1), (c * c).sum(1)
    cu = (a2 * (b[:, 1] - c[:, 1]) + b2 * (c[:, 1] - a[:, 1]) + c2 * (a[:, 1] - b[:, 1])) / d
    cv_ = (a2 * (c[:, 0] - b[:, 0]) + b2 * (a[:, 0] - c[:, 0]) + c2 * (b[:, 0] - a[:, 0])) / d
    cr = np.hypot(a[:, 0] - cu, a[:, 1] - cv_)
    res = np.hypot(pts[None, :, 0] - cu[:, None], pts[None, :, 1] - cv_[:, None]) - cr[:, None]
    score = (np.abs(res) < p.on_px).sum(1) - 2 * (res > p.out_px).sum(1)
    score[~okd | (cr < 0.6 * circle.r) | (cr > 1.5 * circle.r)] = -10 ** 6
    best = int(score.argmax())
    on = np.abs(res[best]) < p.on_px
    for _ in range(2):
        if on.sum() < p.min_points:
            return None
        u, v, r = fit_circle(pts[on])
        r_all = np.hypot(pts[:, 0] - u, pts[:, 1] - v) - r
        on = np.abs(r_all) < p.on_px
    if on.sum() < p.min_points:
        return None
    ang = np.sort(np.arctan2(pts[on, 1] - v, pts[on, 0] - u))
    gap = np.diff(np.concatenate([ang, ang[:1] + 2.0 * math.pi])).max()
    rms = float(np.sqrt(np.mean(r_all[on] ** 2)))
    if math.degrees(2.0 * math.pi - gap) < p.min_arc_deg or rms > p.max_rms_px:
        return None
    return Fit(Circle(float(u), float(v), float(r)), rms, int(on.sum()))


# ---------------------------------------------------------------- detector


METHODS = ("eye_fit", "eye_box", "pink_fit", "pink_box", "pinkfloor_fit")
FIT_METHODS = ("eye_fit", "pink_fit", "pinkfloor_fit")


@dataclass(frozen=True)
class Measurement:
    """One ball centre on base_link [m] from one proposal method; no capture time."""

    method: str
    x: float
    y: float
    z: float
    range_m: float
    rho: float
    circle: Circle
    score: float
    sigma_px: float
    sigma_xy_m: float  # image-noise model only
    sigma_range_m: float  # image-noise model only
    fit_rms_px: float | None = None
    fit_points: int | None = None

    def record(self) -> dict:
        return {"method": self.method, "x": round(self.x, 5), "y": round(self.y, 5), "z": round(self.z, 5),
                "range_m": round(self.range_m, 5), "rho_rad": round(self.rho, 6),
                "circle": [round(self.circle.u, 2), round(self.circle.v, 2), round(self.circle.r, 2)],
                "score": round(self.score, 4), "sigma_px": round(self.sigma_px, 3),
                "sigma_xy_m": round(self.sigma_xy_m, 5), "sigma_range_m": round(self.sigma_range_m, 5),
                "fit_rms_px": None if self.fit_rms_px is None else round(self.fit_rms_px, 3),
                "fit_points": self.fit_points}


@dataclass(frozen=True)
class FrameResult:
    candidates: tuple  # Measurement, at most one per method
    selected: int | None
    state: str  # measured, searching, conflict
    notes: dict
    ms: dict


@dataclass(frozen=True)
class EyeParams:
    fit: FitParams = FitParams()
    min_range: float = 0.10
    max_range: float = 3.0
    min_box_r_px: float = 6.0
    seed_fit_range: float = 0.45  # nearer than this, a proposal counts only with an outline fit
    z_tol: float = 0.05
    z_tol_per_m: float = 0.06
    box_sigma_px: float = 2.0
    box_sigma_frac: float = 0.10
    fit_sigma_floor_px: float = 0.5
    agree_sigmas: float = 3.0
    agree_floor_m: float = 0.03


class BallEye:
    """RGB fisheye frame -> FrameResult. One instance per thread."""

    def __init__(self, profile: MetricProfile, backend=None, params: EyeParams = EyeParams()):
        self.profile, self.backend, self.p = profile, backend, params
        self.view = profile.view
        self._maps = None
        self.view_rgb = None  # newest floor view (debug/data capture only)

    def floor_view(self, frame_rgb: np.ndarray):
        """Fisheye frame (any decoded size) -> (view image, valid mask)."""
        import cv2

        h, w = frame_rgb.shape[:2]
        if self._maps is None or self._maps[0] != (w, h):
            self._maps = ((w, h), *self.view.maps(self.profile.lens.scaled(w, h)))
        _, mx, my, valid = self._maps
        view = cv2.remap(frame_rgb, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
        view[~valid] = 0
        return view, valid

    def _measure(self, circle: Circle, method: str, score: float, fit: Fit | None, notes: dict):
        """Circle -> Measurement, if the ball is in range and on the floor."""
        p, radius = self.p, self.profile.ball_radius_m
        pos, rng, rho = circle_to_body(self.view, self.profile.mount, circle, radius)
        want_z = radius - self.profile.base_height_m
        if not p.min_range <= rng <= p.max_range:
            notes[method] = f"range {rng:.2f} m out of limits"
            return None
        if abs(pos[2] - want_z) > p.z_tol + p.z_tol_per_m * rng:
            notes[method] = f"not on the floor (z {pos[2]:+.2f} m, want {want_z:+.2f})"
            return None
        sigma_px = (max(fit.rms_px, p.fit_sigma_floor_px) if fit is not None
                    else max(p.box_sigma_px, p.box_sigma_frac * circle.r))
        sigma_xy, sigma_range = image_uncertainty(self.view, self.profile.mount, circle, radius, sigma_px)
        return Measurement(method, float(pos[0]), float(pos[1]), float(pos[2]), float(rng), float(rho), circle,
                           float(score), sigma_px, sigma_xy, sigma_range,
                           None if fit is None else fit.rms_px, None if fit is None else fit.points)

    def _outline(self, view, valid, first: Circle, cut: bool) -> Fit | None:
        """Outline fit from a first circle. cut: the first circle is not itself the outline."""
        fp = self.p.fit
        fitted = refine_circle(view, valid, first, fp)
        if fitted is not None and cut:
            fitted = refine_circle(view, valid, fitted.circle, fp) or fitted
        if fitted is None:
            return None
        c = fitted.circle
        offset, dr = math.hypot(c.u - first.u, c.v - first.v), abs(c.r - first.r)
        # A whole proposal must agree closely. A cut or seeded proposal may be off
        # centre, but the fit cannot wander to another round edge.
        limit = fp.seed_agree if cut else fp.box_agree
        if offset > limit * first.r or dr > limit * first.r:
            return None
        return fitted

    def eye(self, view, valid, notes):
        """Ball Eye YOLO proposals -> best eye Measurement."""
        if self.backend is None:
            return None
        import cv2

        size = getattr(self.backend, "size", MODEL_INPUT)
        k = self.view.size / size
        small = cv2.resize(view, (size, size), interpolation=cv2.INTER_AREA) if size != self.view.size else view
        boxes = [(x1 * k, y1 * k, x2 * k, y2 * k, s) for x1, y1, x2, y2, s, _ in self.backend.infer(small)]
        if not boxes:
            notes["eye"] = "no box"
        for x1, y1, x2, y2, score in boxes:
            cut = (min(x1, y1) < 2 or max(x2, y2) > self.view.size - 3
                   or abs((x2 - x1) - (y2 - y1)) > 0.2 * min(x2 - x1, y2 - y1))
            box = Circle((x1 + x2) / 2.0, (y1 + y2) / 2.0,
                         max(x2 - x1, y2 - y1) / 2.0 if cut else (x2 - x1 + y2 - y1) / 4.0)
            fit = self._outline(view, valid, box, cut)
            if fit is None and (cut or self.profile.ball_radius_m / math.sin(max(self.view.cone(box)[1], 1e-6))
                                < self.p.seed_fit_range):
                notes["eye"] = "near or cut box with no outline fit"
                continue
            circle = fit.circle if fit is not None else box
            if circle.r < self.p.min_box_r_px:
                notes["eye"] = "box too small"
                continue
            found = self._measure(circle, "eye_fit" if fit else "eye_box", score, fit, notes)
            if found is not None:
                return found
        return None

    def seed(self, view, valid, axis, rho, *, whole: bool, method: str, score: float, notes):
        """Measure a ball another detector found, from a cone of camera rays.

        With an outline fit the result is "<method>_fit". With no fit, only a whole
        seed farther than seed_fit_range may become "<method>_box".
        """
        first = self.view.circle(axis, rho)
        in_view = 0 <= first.u < self.view.size and 0 <= first.v < self.view.size
        fit = self._outline(view, valid, first, cut=not whole) if in_view and first.r >= self.p.fit.min_r_px else None
        if fit is not None:
            return self._measure(fit.circle, method + "_fit", score, fit, notes)
        if not whole or self.profile.ball_radius_m / math.sin(max(rho, 1e-6)) < self.p.seed_fit_range:
            notes[method] = "seed with no outline fit"
            return None
        if first.r < self.p.min_box_r_px:
            notes[method] = "box too small"
            return None
        return self._measure(first, method + "_box", score, None, notes)

    def pink_floor(self, view, valid, notes):
        """Pink colour components in the floor view, where a ball is truly round."""
        import cv2

        from pink_ball import pink_masks

        hsv = cv2.cvtColor(view, cv2.COLOR_RGB2HSV)
        ok = valid.astype(np.uint8) * 255
        best = []
        # Like PinkBallTracker, keep the masks separate: a pink white-balance cast
        # turns pale floor and walls into a pale component that swallows the ball.
        for mask in pink_masks(hsv):
            mask = cv2.bitwise_and(mask, ok)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
            for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                area = cv2.contourArea(contour)
                (u, v), r = cv2.minEnclosingCircle(contour)
                if area < 60 or r < self.p.min_box_r_px:
                    continue
                fill = area / max(1.0, math.pi * r * r)
                if fill < 0.45:
                    continue
                best.append((fill * min(1.0, r / 20.0), Circle(float(u), float(v), float(r))))
        best.sort(key=lambda item: -item[0])
        measured = []
        ang = np.linspace(0, 2 * math.pi, 32, endpoint=False)
        for score, circle in best[:3]:
            # A component wholly inside the lens image is a whole proposal and its fit
            # must agree closely; only one cut by the image edge may move further.
            pu = np.round(circle.u + 1.1 * circle.r * np.cos(ang)).astype(int)
            pv = np.round(circle.v + 1.1 * circle.r * np.sin(ang)).astype(int)
            inside = (pu >= 0) & (pu < self.view.size) & (pv >= 0) & (pv < self.view.size)
            cut = not inside.all() or not valid[pv[inside], pu[inside]].all()
            fit = self._outline(view, valid, circle, cut=cut)
            if fit is None:
                continue
            found = self._measure(fit.circle, "pinkfloor_fit", score, fit, notes)
            if found is not None and not any(math.dist((found.x, found.y), (m.x, m.y)) < self.profile.ball_radius_m
                                             for m in measured):
                measured.append(found)
        if len(measured) > 1:
            notes["pinkfloor"] = "ambiguous: several pink balls"
            return None
        if not measured:
            notes.setdefault("pinkfloor", "no fitted pink component")
        return measured[0] if measured else None

    def measure(self, frame_rgb: np.ndarray, *, pink_box=None, pano: Pano = Pano(), methods=("eye", "pink", "pinkfloor")):
        """One frame -> per-method measurements and a deterministic selection.

        pink_box is the maintained PinkBallTracker's normalized panorama box from
        the same capture, or None. Fits beat boxes; when two fits disagree beyond
        their image uncertainty the frame is a conflict with nothing selected.
        """
        t0 = time.perf_counter()
        notes = {}
        view, valid = self.floor_view(frame_rgb)
        self.view_rgb = view
        t1 = time.perf_counter()
        found = []
        if "eye" in methods:
            found.append(self.eye(view, valid, notes))
        t2 = time.perf_counter()
        if "pink" in methods and pink_box is not None:
            axis, rho, whole = pano.cone(pink_box)
            found.append(self.seed(view, valid, axis, rho, whole=whole, method="pink", score=1.0, notes=notes))
        if "pinkfloor" in methods:
            found.append(self.pink_floor(view, valid, notes))
        t3 = time.perf_counter()
        candidates = tuple(m for m in found if m is not None)
        selected, state = self.select(candidates, notes)
        ms = {"view": (t1 - t0) * 1e3, "eye": (t2 - t1) * 1e3, "pink": (t3 - t2) * 1e3}
        return FrameResult(candidates, selected, state, notes, ms)

    def select(self, candidates, notes):
        """Lowest-uncertainty fit unless two disagree. pink and pinkfloor share the
        outline fit, so their agreement is not independent confirmation."""
        fits = [i for i, m in enumerate(candidates) if m.method in FIT_METHODS]
        pool = fits or list(range(len(candidates)))
        if not pool:
            return None, "searching"
        for i in pool:
            for j in pool:
                a, b = candidates[i], candidates[j]
                if i < j and math.dist((a.x, a.y), (b.x, b.y)) > max(
                        self.p.agree_floor_m, self.p.agree_sigmas * math.hypot(a.sigma_xy_m, b.sigma_xy_m)):
                    notes["selection"] = f"{a.method} and {b.method} disagree"
                    return None, "conflict"
        return min(pool, key=lambda i: candidates[i].sigma_xy_m), "measured"


# ---------------------------------------------------------------- non-blocking worker


@dataclass(frozen=True)
class MetricJob:
    frame: np.ndarray  # decoded camera frame, as the decoder returned it
    swap_rb: bool  # True: frame is already in detector RGB order
    captured_at_ros_ns: int
    captured_at_mono_ns: int
    frame_sequence: int
    pink_box: tuple | None
    pano_width: int = 700  # width of the panorama the pink box is normalized to


class MetricWorker:
    """One-slot latest-job mailbox and a background thread for Ball Eye work.

    submit never waits for inference, so obstacle evidence keeps its cadence.
    A job that is no longer current before or after measurement is dropped:
    slow work can delay or skip a ball row but never publish an old capture as
    new, and a result always carries its own frame's capture times.
    """

    def __init__(self, eye: BallEye, publish, current, *, model_sha256=None, producer_instance=None,
                 clock_ns=time.monotonic_ns):
        import uuid

        from onboard_ball import metric_payload

        self.eye, self.publish, self.current = eye, publish, current
        self.payload = metric_payload
        self.model_sha256 = model_sha256
        self.producer_instance = producer_instance or uuid.uuid4().hex
        self.clock_ns = clock_ns
        self.sequence = 0
        self.counts = dict(submitted=0, overwritten=0, expired=0, published=0, failed=0)
        self.last_ms = None
        self.last_error = None
        self._lock = threading.Lock()
        self._pending = None
        self._last_stamp = None
        self._wake = threading.Event()
        self._stop = False
        self.thread = threading.Thread(target=self._run, daemon=True, name="ball-eye")
        self.thread.start()

    def submit(self, job: MetricJob) -> bool:
        """Offer the newest capture. Older or repeated captures are refused."""
        with self._lock:
            if self._last_stamp is not None and job.captured_at_ros_ns <= self._last_stamp:
                return False
            self._last_stamp = job.captured_at_ros_ns
            self.counts["submitted"] += 1
            self.counts["overwritten"] += self._pending is not None
            self._pending = job
        self._wake.set()
        return True

    def process(self, job: MetricJob):
        """Measure one job and publish it if its capture is still current."""
        import cv2

        if not self.current(job):
            self.counts["expired"] += 1
            return None
        started = self.clock_ns()
        rgb = job.frame if job.swap_rb else cv2.cvtColor(job.frame, cv2.COLOR_BGR2RGB)
        result = self.eye.measure(rgb, pink_box=job.pink_box, pano=Pano(job.pano_width))
        self.last_ms = (self.clock_ns() - started) / 1e6
        if not self.current(job):
            self.counts["expired"] += 1
            return None
        payload = self.payload(producer_instance=self.producer_instance, sequence=self.sequence,
                               frame_sequence=job.frame_sequence, captured_at_ros_ns=job.captured_at_ros_ns,
                               captured_at_mono_ns=job.captured_at_mono_ns, identity=self.eye.profile.identity(),
                               model_sha256=self.model_sha256, state=result.state, selected=result.selected,
                               candidates=[m.record() for m in result.candidates],
                               processing_ms=round(self.last_ms, 2))
        self.sequence += 1
        self.publish(payload)
        self.counts["published"] += 1
        return payload

    def _run(self):
        while not self._stop:
            self._wake.wait(timeout=0.5)
            self._wake.clear()
            with self._lock:
                job, self._pending = self._pending, None
            if job is None or self._stop:
                continue
            try:
                self.process(job)
            except Exception as exc:  # a bad frame must not stop later frames or the obstacle loop
                self.counts["failed"] += 1
                self.last_error = f"{type(exc).__name__}: {exc}"[:200]

    def close(self) -> bool:
        self._stop = True
        self._wake.set()
        self.thread.join(timeout=2.0)
        return not self.thread.is_alive()
