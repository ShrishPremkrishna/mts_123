"""Ball Eye: the close-range ball detector for the kick line-up.

One camera frame -> the ball centre in the body frame (base_link: x ahead, y left,
z up), in four steps:

  1. Floor view. The 220 deg fisheye frame (double sphere model, IMX296 global
     shutter) is remapped to a stereographic view that looks ahead and down.
     A stereographic projection maps each circle on the sphere of camera rays to
     a circle in the image. The outline of a ball is such a circle, so the ball
     is round in the view at each position, also 1 cm from the nose.
  2. YOLO. A one-class YOLOv8n (trained on this view) gives the ball box. It runs
     on the Hailo-8 (.hef) or on the CPU through OpenCV DNN (.onnx).
  3. Outline fit. Along rays from the box centre, the outermost ball-colored pixel
     is an outline point. The colors are learned in each frame from the box
     centre and from a ring around the box, so there are no color thresholds.
     A robust circle fit ignores dents (white flowers, a cut ball).
  4. Geometry. The circle gives the direction of the ball centre and its angular
     radius rho. Range = ball radius / sin(rho). No camera height is necessary.

BallTrack then filters the fixes and predicts across the frame delay.

Frames: optical (x right, y down, z ahead) -> view (optical turned down by
View.pitch_deg) and -> body (Mount). NumPy and OpenCV only.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

CAMERA_YAML = Path(__file__).resolve().parent.parent / "robot" / "camera_params.yaml"


# ---------------------------------------------------------------- camera model


@dataclass(frozen=True)
class Lens:
    """Double sphere fisheye model (Usenko 2018); defaults from robot/camera_params.yaml."""

    fx: float = 228.81030
    fy: float = 228.81030
    cx: float = 680.0
    cy: float = 479.0
    xi: float = -0.00092
    alpha: float = 0.62007
    width: int = 1400
    height: int = 1050
    max_angle_deg: float = 92.0  # edge of the lit image circle (sample frame, 2026-11-19)

    def scaled(self, width: int, height: int) -> "Lens":
        """The same lens for a frame decoded at another size."""
        sx, sy = width / self.width, height / self.height
        return replace(self, fx=self.fx * sx, fy=self.fy * sy, cx=(self.cx + 0.5) * sx - 0.5,
                       cy=(self.cy + 0.5) * sy - 0.5, width=width, height=height)

    def project(self, rays: np.ndarray):
        """Unit rays (..., 3) in the optical frame -> pixel u, v and a validity mask."""
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
        """Unit rays (height, width, 3) of the pixels in the optical frame, and the mask of
        pixels that the lens sees (unprojection: Usenko 2018, equations 46 to 50)."""
        u, v = np.meshgrid(np.arange(self.width, dtype=float), np.arange(self.height, dtype=float))
        mx, my = (u - self.cx) / self.fx, (v - self.cy) / self.fy
        r2 = mx * mx + my * my
        seen = r2 < (1.0 / (2.0 * self.alpha - 1.0) if self.alpha > 0.5 else np.inf)
        mz = (1.0 - self.alpha ** 2 * r2) / (self.alpha * np.sqrt(np.maximum(1.0 - (2.0 * self.alpha - 1.0) * r2, 0.0))
                                             + 1.0 - self.alpha)
        k = (mz * self.xi + np.sqrt(mz * mz + (1.0 - self.xi ** 2) * r2)) / (mz * mz + r2)
        rays = np.stack([k * mx, k * my, k * mz - self.xi], axis=-1)
        return rays, seen & (rays[..., 2] > math.cos(math.radians(self.max_angle_deg)))


def load_lens(path: Path = CAMERA_YAML) -> Lens:
    import yaml

    p = (yaml.safe_load(Path(path).read_text()) or {}).get("camera_params", {})
    return Lens(**{k: float(p[k]) for k in ("fx", "fy", "cx", "cy", "xi", "alpha") if k in p})


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

    def grid_rays(self) -> np.ndarray:
        u, v = np.meshgrid(np.arange(self.size), np.arange(self.size))
        return self.ray(u, v)

    def maps(self, lens: Lens):
        """cv2.remap tables fisheye -> view, and the mask of view pixels the lens sees."""
        u, v, valid = lens.project(self.grid_rays())
        return u.astype(np.float32), v.astype(np.float32), valid

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
    """The equirectangular detection image of the Pupper FC camera node
    (follow_me_pupper/robot/fisheye_converter.py): h_fov_deg across the width and
    180 deg down the height, with equal degrees per pixel on the two axes."""

    width: int = 700
    h_fov_deg: float = 220.0
    round_tol: float = 0.2  # the two radii of a whole ball agree within this fraction

    @property
    def height(self) -> int:
        return max(2, int(round(self.width * 180.0 / self.h_fov_deg)) // 2 * 2)

    def angles(self, x: float, y: float) -> tuple[float, float]:
        """Box coordinates (pixel edges / image size) -> (longitude to the right, latitude down) [rad]."""
        w, h = self.width, self.height
        return (((x * w - 0.5) / (w - 1) - 0.5) * math.radians(self.h_fov_deg),
                ((y * h - 0.5) / (h - 1) - 0.5) * math.pi)

    def cone(self, box):
        """Box of a ball (x1, y1, x2, y2) -> (unit axis in the optical frame, angular radius rho, whole).

        The outline of a ball is a circle on the sphere of camera rays. Its latitude
        limits are (centre latitude) +/- rho, and its longitude half-width w obeys
        sin(rho) = sin(w) cos(centre latitude). whole is False when the two values of
        rho differ, or the box touches an image side or comes near the pole: a cut,
        merged or stretched box, which gives a search area but not a position."""
        (lon1, lat1), (lon2, lat2) = self.angles(box[0], box[1]), self.angles(box[2], box[3])
        lon0, lat0, half_lon, rho_v = (lon1 + lon2) / 2.0, (lat1 + lat2) / 2.0, (lon2 - lon1) / 2.0, (lat2 - lat1) / 2.0
        rho_h = math.asin(min(1.0, math.sin(min(half_lon, math.pi / 2.0)) * math.cos(lat0)))
        edge = 1.5 / self.width
        whole = (abs(rho_v - rho_h) <= self.round_tol * min(rho_v, rho_h) and half_lon < math.radians(80.0)
                 and lat0 + rho_v < math.radians(85.0) and box[0] > edge and box[2] < 1.0 - edge and box[3] < 1.0 - edge)
        rho = (rho_v + rho_h) / 2.0 if whole else max(rho_v, rho_h)  # a cut ball: the longer side is the better radius
        axis = np.array([math.cos(lat0) * math.sin(lon0), math.sin(lat0), math.cos(lat0) * math.cos(lon0)])
        return axis, rho, whole

    def box(self, axis: np.ndarray, rho: float) -> tuple[float, float, float, float]:
        """Cone of rays -> its box in the image (for tests; a ball that does not reach the pole)."""
        lat0, lon0 = math.asin(float(axis[1])), math.atan2(float(axis[0]), float(axis[2]))
        half_lon = math.asin(min(1.0, math.sin(rho) / math.cos(lat0)))
        w, h = self.width, self.height
        fx = lambda lon: ((lon / math.radians(self.h_fov_deg) + 0.5) * (w - 1) + 0.5) / w  # noqa: E731
        fy = lambda lat: ((lat / math.pi + 0.5) * (h - 1) + 0.5) / h  # noqa: E731
        return fx(lon0 - half_lon), fy(lat0 - rho), fx(lon0 + half_lon), fy(lat0 + rho)


@dataclass(frozen=True)
class Mount:
    """Camera pose on base_link. Estimates (2026-11-16), not measured."""

    x: float = 0.15  # m ahead
    y: float = 0.0  # m left
    z: float = 0.03  # m up
    tilt_up_deg: float = 10.0
    yaw_deg: float = 0.0  # optical axis to the left of the body x axis

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


# ---------------------------------------------------------------- fix and geometry


@dataclass(frozen=True)
class BallFix:
    """One detection: the ball centre on base_link [m]."""

    x: float
    y: float
    z: float
    range: float  # camera to ball centre
    rho: float  # angular radius [rad]
    score: float  # YOLO confidence
    method: str  # "fit" (outline fit) or "box" (YOLO box only)
    circle: Circle
    stamp: float = 0.0  # capture time, caller's clock
    ms: dict = field(default_factory=dict, compare=False)  # stage times [ms]
    source: str = "eye"  # what found the ball: "eye" (Ball Eye YOLO), "pink+yolo" / "pink" (Pupper FC detector), "track"


def circle_to_body(view: View, mount: Mount, circle: Circle, ball_radius: float):
    """View circle -> (ball centre on base_link, range, rho)."""
    axis, rho = view.cone(circle)
    rng = ball_radius / math.sin(max(rho, 1e-6))
    return mount.to_body(axis * rng), rng, rho


def body_to_circle(view: View, mount: Mount, p_body, ball_radius: float) -> Circle:
    """Ball centre on base_link -> its outline in the view (for tests and synthetic data)."""
    p = mount.to_optical(p_body)
    rng = float(np.linalg.norm(p))
    return view.circle(p / rng, math.asin(min(1.0, ball_radius / rng)))


# ---------------------------------------------------------------- YOLO backends


def decode_yolov8(out: np.ndarray, conf: float, iou: float = 0.5, max_det: int = 5):
    """Raw YOLOv8 head (1, 4 + classes, N), cxcywh in input pixels -> [(x1, y1, x2, y2, score, class)]."""
    import cv2

    p = np.asarray(out)[0].T  # (N, 4 + classes)
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
    """YOLOv8 .onnx on the CPU through OpenCV DNN (no extra package on the Pupper)."""

    def __init__(self, path, conf: float = 0.35, threads: int = 2):
        import cv2

        cv2.setNumThreads(threads)  # leave cores for the controller; the Pi throttles near 85 deg C
        # A path, or the bytes of the model file (pi_look.py sends the model with no file on the Pupper).
        self.net = cv2.dnn.readNetFromONNX(np.frombuffer(path, np.uint8) if isinstance(path, bytes) else str(path))
        self.conf = conf

    def infer(self, rgb: np.ndarray):
        import cv2

        self.net.setInput(cv2.dnn.blobFromImage(rgb, 1.0 / 255.0, (rgb.shape[1], rgb.shape[0]), swapRB=False))
        return decode_yolov8(self.net.forward(), self.conf)


class HailoBackend:
    """YOLOv8 .hef on the Hailo-8 (HailoRT 4.20). The .hef has the NMS step inside
    (Hailo Model Zoo yolov8n.alls): one output, per class an array of
    [y1, x1, y2, x2, score], normalized to 0..1."""

    def __init__(self, path, conf: float = 0.35):
        from hailo_platform import FormatType, HailoSchedulingAlgorithm, VDevice

        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        params.group_id = "SHARED"  # the same group as the course detector node, so both can run
        self.target = VDevice(params)
        self.model = self.target.create_infer_model(str(path))
        self.model.set_batch_size(1)
        self.model.output().set_format_type(FormatType.FLOAT32)
        self.size = int(self.model.input().shape[0])
        self._ctx = self.model.configure()
        self._configured = self._ctx.__enter__()
        self.conf = conf

    def infer(self, rgb: np.ndarray):
        bindings = self._configured.create_bindings()
        bindings.input().set_buffer(np.ascontiguousarray(rgb, dtype=np.uint8))
        bindings.output().set_buffer(np.empty(self.model.output().shape, dtype=np.float32))
        self._configured.run([bindings], 1000)
        out, s, boxes = bindings.output().get_buffer(), rgb.shape[0], []
        for class_id, rows in enumerate(out if isinstance(out, list) else [out]):
            for y1, x1, y2, x2, score in np.asarray(rows, dtype=np.float32).reshape(-1, 5):
                if score >= self.conf:
                    boxes.append((x1 * s, y1 * s, x2 * s, y2 * s, float(score), class_id))
        return sorted(boxes, key=lambda b: -b[4])

    def close(self):
        ctx, self._ctx = self._ctx, None
        if ctx is not None:
            ctx.__exit__(None, None, None)
        self.target.release()


def load_backend(path, conf: float = 0.35):
    return HailoBackend(path, conf) if str(path).endswith(".hef") else OnnxBackend(path, conf)


# ---------------------------------------------------------------- outline fit


@dataclass(frozen=True)
class FitParams:
    rays: int = 96
    inner: float = 0.55  # ball color samples: inside this fraction of the box radius
    ring: tuple[float, float] = (1.2, 1.45)  # background samples and the search limit
    bins: int = 24  # histogram bins per chroma axis
    smooth_bins: float = 2.0  # Gaussian sigma of the histogram smoothing [bins]
    on_px: float = 2.0  # an outline point this near the circle is on it
    out_px: float = 3.5  # farther outside: counts against the circle
    min_points: int = 14
    min_arc_deg: float = 110.0
    max_rms_px: float = 1.3
    trials: int = 300
    min_r_px: float = 30.0  # smaller balls (far): the YOLO box is as good, and one pixel is 3 % of the radius
    box_agree: float = 0.25  # a whole box and the fit must agree: centre and radius within this fraction


def fit_circle(pts: np.ndarray):
    """Least-squares circle (Kasa) through points (N, 2) -> (u, v, r)."""
    a = np.column_stack([2.0 * pts, np.ones(len(pts))])
    sol, *_ = np.linalg.lstsq(a, (pts * pts).sum(axis=1), rcond=None)
    return sol[0], sol[1], math.sqrt(max(sol[2] + sol[0] ** 2 + sol[1] ** 2, 1e-9))


def outline_points(rgb: np.ndarray, valid: np.ndarray, circle: Circle, p: FitParams = FitParams()):
    """Outermost ball-colored pixel on each ray from the box centre -> points (N, 2)."""
    import cv2

    size = rgb.shape[0]
    reach = circle.r * p.ring[1]
    x0, y0 = max(0, int(circle.u - reach)), max(0, int(circle.v - reach))
    x1, y1 = min(size, int(circle.u + reach) + 2), min(size, int(circle.v + reach) + 2)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return np.empty((0, 2))
    crop, ok = rgb[y0:y1, x0:x1], valid[y0:y1, x0:x1]
    ab = (cv2.cvtColor(crop, cv2.COLOR_RGB2LAB)[..., 1:].astype(np.int32) * p.bins) // 256
    yy, xx = np.mgrid[y0:y1, x0:x1]
    dist = np.hypot(xx - circle.u, yy - circle.v) / circle.r
    ball_px, back_px = ok & (dist < p.inner), ok & (dist > p.ring[0]) & (dist < p.ring[1])
    if ball_px.sum() < 20 or back_px.sum() < 20:
        return np.empty((0, 2))
    code = ab[..., 0] * p.bins + ab[..., 1]
    h_ball = np.bincount(code[ball_px], minlength=p.bins * p.bins) / ball_px.sum()
    h_back = np.bincount(code[back_px], minlength=p.bins * p.bins) / back_px.sum()
    # A chroma bin belongs to the ball when the ball samples have it more often. The two
    # histograms are smoothed, so a color between the ball and the background (a blurred
    # edge pixel) goes to the nearer one: the outline is at the middle of the edge, not inside it.
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
    last = hit.shape[1] - 1 - np.argmax(hit[:, ::-1], axis=1)  # outermost hit on each ray
    rows = np.arange(p.rays)
    # Drop a ray with no hit, a ray whose hits go to the search limit (color leak), and
    # a ray whose next pixels are outside the lens or the image (a cut ball, not its outline).
    after = np.clip(last + 3, 0, hit.shape[1] - 1)
    good = hit.any(axis=1) & (last < hit.shape[1] - 3) & seen[rows, after]
    return np.column_stack([px[rows, last], py[rows, last]])[good]


def refine_circle(rgb: np.ndarray, valid: np.ndarray, circle: Circle, p: FitParams = FitParams(),
                  rng: np.random.Generator | None = None) -> Circle | None:
    """Outline fit: the circle with the most outline points on it and few outside it."""
    if circle.r < p.min_r_px:
        return None
    pts = outline_points(rgb, valid, circle, p)
    if len(pts) < p.min_points:
        return None
    rng = rng or np.random.default_rng(0)
    tri = pts[rng.integers(0, len(pts), size=(p.trials, 3))]  # (T, 3, 2)
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
    for _ in range(2):  # least squares on the points of the best circle
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
    return Circle(float(u), float(v), float(r))


# ---------------------------------------------------------------- detector


@dataclass(frozen=True)
class EyeParams:
    ball_radius: float = 0.091  # m; a circumference of 22.5 in. Measure the pink ball.
    view: View = field(default_factory=View)
    mount: Mount = field(default_factory=Mount)
    fit: FitParams = field(default_factory=FitParams)
    conf: float = 0.35
    yolo_size: int = 0  # YOLO input size; 0: the view size. 320 with a 640 view: CPU speed, and the outline fit keeps the 640 precision
    refine: bool = True
    reduce: int = 1  # 2: decode the JPEG at half size (CPU backend with a 320 view)
    min_range: float = 0.10
    max_range: float = 3.0
    min_box_r_px: float = 6.0  # a smaller box is not used: 2 m in the 320 view, 3.8 m in the 640 view
    seed_fit_range: float = 0.45  # m from the camera; nearer, a seed from another detector counts only with an outline fit
    # The ball is on the floor: its centre is ball_z above base_link, within the tolerance.
    base_height: float = 0.15  # m, base_link above the floor (0.14 to 0.165 in simulation)
    z_tol: float = 0.05
    z_tol_per_m: float = 0.06


class BallEye:
    """JPEG frame -> BallFix. One instance per thread."""

    def __init__(self, backend, params: EyeParams = EyeParams(), lens: Lens = Lens()):
        self.backend, self.p, self.lens = backend, params, lens
        self._maps = None  # (frame size, map_x, map_y, valid)
        self.view_rgb = None  # the newest floor view (debug and data capture)
        self.note = ""

    def floor_view(self, frame_rgb: np.ndarray):
        """Fisheye frame (any decoded size) -> (view image, valid mask)."""
        import cv2

        h, w = frame_rgb.shape[:2]
        if self._maps is None or self._maps[0] != (w, h):
            self._maps = ((w, h), *self.p.view.maps(self.lens.scaled(w, h)))
        _, mx, my, valid = self._maps
        view = cv2.remap(frame_rgb, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
        view[~valid] = 0
        return view, valid

    def detect(self, jpeg: bytes, stamp: float = 0.0) -> BallFix | None:
        import cv2

        t0 = time.perf_counter()
        flag = cv2.IMREAD_REDUCED_COLOR_2 if self.p.reduce == 2 else cv2.IMREAD_COLOR
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), flag)
        if bgr is None:
            self.note = "frame did not decode"
            return None
        return self.detect_rgb(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), stamp, t0)

    def _outline(self, view: np.ndarray, valid: np.ndarray, first: Circle, cut: bool) -> Circle | None:
        """Outline fit from a first circle. cut: the first circle is not the outline (a cut box, a predicted position)."""
        fitted = refine_circle(view, valid, first, self.p.fit) if self.p.refine else None
        if fitted is not None and cut:  # second pass from the first fit: the first circle is off centre
            fitted = refine_circle(view, valid, fitted, self.p.fit) or fitted
        if fitted is not None and not cut and (
                math.hypot(fitted.u - first.u, fitted.v - first.v) > self.p.fit.box_agree * first.r
                or abs(fitted.r - first.r) > self.p.fit.box_agree * first.r):
            fitted = None  # the fit took another edge
        return fitted

    def _fix(self, axis: np.ndarray, rho: float, circle: Circle, method: str, score: float, stamp: float,
             source: str) -> BallFix | None:
        """Cone of rays (optical frame) -> BallFix, if the ball is in range and on the floor."""
        rng = self.p.ball_radius / math.sin(max(rho, 1e-6))
        pos = self.p.mount.to_body(axis * rng)
        want_z = self.p.ball_radius - self.p.base_height
        if not self.p.min_range <= rng <= self.p.max_range:
            self.note = f"range {rng:.2f} m out of limits"
            return None
        if abs(pos[2] - want_z) > self.p.z_tol + self.p.z_tol_per_m * rng:
            self.note = f"not on the floor (z {pos[2]:+.2f} m, want {want_z:+.2f})"
            return None
        return BallFix(float(pos[0]), float(pos[1]), float(pos[2]), rng, rho, score, method, circle, stamp, source=source)

    def detect_rgb(self, frame_rgb: np.ndarray, stamp: float = 0.0, t0: float | None = None) -> BallFix | None:
        t0 = time.perf_counter() if t0 is None else t0
        t1 = time.perf_counter()
        view, valid = self.floor_view(frame_rgb)
        self.view_rgb = view
        t2 = time.perf_counter()
        if self.p.yolo_size and self.p.yolo_size != self.p.view.size:
            import cv2

            k = self.p.view.size / self.p.yolo_size
            small = cv2.resize(view, (self.p.yolo_size, self.p.yolo_size), interpolation=cv2.INTER_AREA)
            boxes = [(x1 * k, y1 * k, x2 * k, y2 * k, score, cls) for x1, y1, x2, y2, score, cls in self.backend.infer(small)]
        else:
            boxes = self.backend.infer(view)
        t3 = time.perf_counter()
        self.note = "no box" if not boxes else ""
        for x1, y1, x2, y2, score, _cls in boxes:
            cut = min(x1, y1) < 2 or max(x2, y2) > self.p.view.size - 3 or abs((x2 - x1) - (y2 - y1)) > 0.2 * min(x2 - x1, y2 - y1)
            # A cut ball: the longer box side is the better first radius.
            box = Circle((x1 + x2) / 2.0, (y1 + y2) / 2.0, max(x2 - x1, y2 - y1) / 2.0 if cut else (x2 - x1 + y2 - y1) / 4.0)
            fitted = self._outline(view, valid, box, cut)
            circle, method = (fitted, "fit") if fitted is not None else (box, "box")
            if method == "box" and cut:
                self.note = "cut box with no outline fit"
                continue  # the box of a cut ball is not its outline
            if circle.r < self.p.min_box_r_px:
                self.note = "box too small"
                continue
            fix = self._fix(*self.p.view.cone(circle), circle, method, score, stamp, "eye")
            if fix is not None:
                t4 = time.perf_counter()
                ms = {"decode": (t1 - t0) * 1e3, "view": (t2 - t1) * 1e3, "yolo": (t3 - t2) * 1e3, "fit": (t4 - t3) * 1e3}
                return replace(fix, ms=ms)
        return None

    def detect_seed(self, jpeg: bytes, axis: np.ndarray, rho: float, stamp: float = 0.0, *, whole: bool = True,
                    must_fit: bool = False, score: float = 1.0, source: str = "seed") -> BallFix | None:
        """Measure a ball that another detector found (no YOLO of this module).

        The seed is a cone of camera rays: unit axis in the optical frame and angular
        radius (Pano.cone of a box, or the predicted position of the track). The
        outline fit on the floor view gives the position ("fit"). With no fit, a
        whole seed far from the camera is the position ("box"). A seed that is not
        whole, a seed with must_fit, and a seed nearer than seed_fit_range count
        only with a fit."""
        import cv2

        t0 = time.perf_counter()
        view_p, self.note = self.p.view, ""
        first = view_p.circle(axis, rho)
        fitted, t1, t2 = None, t0, t0
        in_view = 0 <= first.u < view_p.size and 0 <= first.v < view_p.size
        if self.p.refine and in_view and first.r >= self.p.fit.min_r_px:
            flag = cv2.IMREAD_REDUCED_COLOR_2 if self.p.reduce == 2 else cv2.IMREAD_COLOR
            bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), flag) if jpeg else None
            if bgr is None:
                self.note = "frame did not decode"
                return None
            t1 = time.perf_counter()
            view, valid = self.floor_view(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            self.view_rgb = view
            t2 = time.perf_counter()
            fitted = self._outline(view, valid, first, cut=not whole or must_fit)
        if fitted is not None:
            fix = self._fix(*view_p.cone(fitted), fitted, "fit", score, stamp, source)
        elif must_fit or not whole or self.p.ball_radius / math.sin(max(rho, 1e-6)) < self.p.seed_fit_range:
            self.note = f"{source} seed with no outline fit"
            return None
        elif first.r < self.p.min_box_r_px:
            self.note = "box too small"
            return None
        else:
            fix = self._fix(axis, rho, first, "box", score, stamp, source)
        if fix is None:
            return None
        t3 = time.perf_counter()
        return replace(fix, ms={"decode": (t1 - t0) * 1e3, "view": (t2 - t1) * 1e3, "fit": (t3 - t2) * 1e3})


# ---------------------------------------------------------------- tracker


class BallTrack:
    """Constant-velocity Kalman filter of the ball on base_link (x, y, vx, vy).

    The velocity is the ball's apparent velocity in the body frame, so it holds
    the robot's own motion with no walker model. predict(t) gives the position at
    a later time: the frame delay and the walker delay.
    """

    def __init__(self, sigma_xy: float = 0.004, accel: float = 0.6, gate: float = 0.12, max_age: float = 0.6):
        self.sigma, self.accel, self.gate, self.max_age = sigma_xy, accel, gate, max_age
        self.s = None  # state (4,)
        self.P = None
        self.t = None  # time of the state
        self.hits = 0

    def _step(self, dt: float):
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        q = self.accel ** 2 * np.array([[dt ** 4 / 4, 0, dt ** 3 / 2, 0], [0, dt ** 4 / 4, 0, dt ** 3 / 2],
                                        [dt ** 3 / 2, 0, dt ** 2, 0], [0, dt ** 3 / 2, 0, dt ** 2]])
        return f @ self.s, f @ self.P @ f.T + q

    def update(self, fix: BallFix) -> bool:
        """Add one fix (stamp = capture time). False: the fix was refused as a jump."""
        z = np.array([fix.x, fix.y])
        sigma = self.sigma * (1.0 + 4.0 * fix.range) * (1.0 if fix.method == "fit" else 3.0)
        if self.s is None or fix.stamp - self.t > self.max_age:
            self.s, self.P = np.array([fix.x, fix.y, 0.0, 0.0]), np.diag([sigma ** 2, sigma ** 2, 0.25, 0.25])
            self.t, self.hits = fix.stamp, 1
            return True
        dt = fix.stamp - self.t
        if dt <= 0:
            return False
        s, big_p = self._step(dt)
        if np.linalg.norm(z - s[:2]) > self.gate + 0.5 * dt:  # 0.5 m/s: more than the walker moves
            self.hits = 0
            self.s = None  # a jump: start again from the next fix
            return False
        h = np.eye(2, 4)
        k = big_p @ h.T @ np.linalg.inv(h @ big_p @ h.T + np.eye(2) * sigma ** 2)
        self.s, self.P, self.t = s + k @ (z - s[:2]), (np.eye(4) - k @ h) @ big_p, fix.stamp
        self.hits += 1
        return True

    def predict(self, t: float):
        """(x, y, vx, vy) at time t, or None with no fresh track."""
        if self.s is None or t - self.t > self.max_age:
            return None
        s, _ = self._step(max(0.0, t - self.t))
        return tuple(float(v) for v in s)
