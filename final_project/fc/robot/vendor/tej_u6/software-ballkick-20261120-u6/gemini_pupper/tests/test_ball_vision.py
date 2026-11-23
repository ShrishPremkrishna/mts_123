"""ball_vision: blobs, silhouette fit, range and plausibility on ray-traced scenes.

The scenes are ray-traced at the pixel centres of the equirectangular image: a
beach ball with 6 gores (red, white, blue, red, white, blue) on the floor at many
orientations, a person in a red top and blue jeans, and small red objects on the
floor. The NumPy tests give the red and blue masks to the detector directly (no
OpenCV). The tests that call ``pytest.importorskip("cv2")`` run where OpenCV is
installed (the robot): they encode a fisheye camera JPEG and run the full path.

Run with ``-s`` to see the error summary of each sweep.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))

import ball_vision as bv  # noqa: E402
from ball_vision import (  # noqa: E402
    EQ_HEIGHT, EQ_WIDTH, PIX_RAD, BallFix, BallVision, CameraMount, ColorParams, VisionParams, body_to_optical,
    optical_to_body, pixel_ray, pixel_rays, ray_pixel, ray_pixels,
)

RADIUS = 0.097
BASE_H = 0.15  # m, base_link above the floor (simulated walking policies: 0.14 to 0.165 m)
GORES = "RWBRWB"
CODES = {"R": 1, "W": 2, "B": 3, "O": 4}
# colour of code 0 (floor), R, W, B, O
RGB = np.array([(110, 110, 110), (220, 30, 30), (235, 235, 235), (30, 60, 220), (255, 150, 0)], np.uint8)


# --- scene ----------------------------------------------------------------------------------


def rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    i, j = {"x": (1, 2), "y": (2, 0), "z": (0, 1)}[axis]
    m = np.eye(3)
    m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
    return m


def orientations() -> list[tuple[str, np.ndarray]]:
    """108 ball orientations (body <- ball; the ball's z axis is the gore axis)."""
    out = [(f"vertical spin {s}", rot("z", s)) for s in range(0, 360, 15)]
    for yaw in (0, 45, 90):
        out += [(f"horizontal yaw {yaw} spin {s}", rot("z", yaw) @ rot("x", 90) @ rot("z", s))
                for s in range(0, 360, 30)]
    for yaw in (0, 90):
        out += [(f"tilted 45 yaw {yaw} spin {s}", rot("z", yaw) @ rot("x", 45) @ rot("z", s))
                for s in range(0, 360, 30)]
    rng = np.random.default_rng(7)
    for k in range(24):
        w, x, y, z = (q := rng.normal(size=4)) / np.linalg.norm(q)
        out.append((f"random {k}", np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])))
    return out


ORIENTATIONS = orientations()
_EQ_DIRS: dict = {}


def optical_to_body_dirs(rays: np.ndarray, mount: CameraMount) -> np.ndarray:
    """Optical-frame directions (..., 3) to body-frame directions."""
    origin = optical_to_body(np.zeros(3), mount, 1.0)
    rotate = np.stack([optical_to_body(e, mount, 1.0) - origin for e in np.eye(3)], 1)
    return rays @ rotate.T


def eq_dirs(mount: CameraMount) -> np.ndarray:
    """Body-frame ray of every equirectangular pixel, (EQ_HEIGHT * EQ_WIDTH, 3)."""
    if mount not in _EQ_DIRS:
        vv, uu = np.mgrid[0:EQ_HEIGHT, 0:EQ_WIDTH]
        _EQ_DIRS[mount] = optical_to_body_dirs(pixel_rays(uu, vv), mount).reshape(-1, 3)
    return _EQ_DIRS[mount]


class Scene:
    """Colour codes of an image (one ray per pixel), with a depth buffer."""

    def __init__(self, mount: CameraMount = CameraMount(), dirs: np.ndarray | None = None,
                 shape: tuple[int, int] = (EQ_HEIGHT, EQ_WIDTH)):
        self.dirs = eq_dirs(mount) if dirs is None else dirs.reshape(-1, 3)
        self.shape = shape
        self.origin = np.array([mount.x, mount.y, mount.z])
        self.depth = np.full(len(self.dirs), np.inf)
        self.code = np.zeros(len(self.dirs), np.uint8)

    def _put(self, idx: np.ndarray, t: np.ndarray, code: np.ndarray) -> None:
        nearer = t < self.depth[idx]
        self.depth[idx[nearer]] = t[nearer]
        self.code[idx[nearer]] = code[nearer]

    def ball(self, centre, orient: np.ndarray = np.eye(3), radius: float = RADIUS, pattern: str = GORES,
             cap_deg: float = 0.0) -> Scene:
        """A sphere; gore k (of len(pattern)) spans longitudes [k, k + 1) * 360 / len(pattern)."""
        centre = np.asarray(centre, float)
        oc = self.origin - centre
        dist = np.linalg.norm(oc)
        idx = np.nonzero(self.dirs @ (-oc / dist) >= math.cos(math.asin(min(1.0, radius / dist)) + 0.01))[0]
        d = self.dirs[idx]
        b = d @ oc
        disc = b * b - (oc @ oc - radius * radius)
        hit = disc >= 0
        t = -b - np.sqrt(np.where(hit, disc, 0.0))
        hit &= t > 0
        idx, d, t = idx[hit], d[hit], t[hit]
        local = (self.origin + d * t[:, None] - centre) @ orient
        lon = np.degrees(np.arctan2(local[:, 1], local[:, 0])) % 360.0
        polar = np.degrees(np.arccos(np.clip(local[:, 2] / radius, -1.0, 1.0)))
        codes = np.array([CODES[c] for c in pattern], np.uint8)
        code = codes[(lon // (360.0 / len(pattern))).astype(int) % len(pattern)]
        code[(polar < cap_deg) | (polar > 180.0 - cap_deg)] = CODES["W"]
        self._put(idx, t, code)
        return self

    def boxes(self, boxes) -> Scene:
        """Axis-aligned boxes ((x0, y0, z0), (x1, y1, z1), colour) in the body frame."""
        for lo, hi, colour in boxes:
            with np.errstate(divide="ignore", invalid="ignore"):
                t1 = (np.asarray(lo) - self.origin) / self.dirs
                t2 = (np.asarray(hi) - self.origin) / self.dirs
            near = np.nanmax(np.minimum(t1, t2), -1)
            far = np.nanmin(np.maximum(t1, t2), -1)
            idx = np.nonzero((near <= far) & (far > 0))[0]
            t = np.where(near[idx] > 0, near[idx], far[idx])
            self._put(idx, t, np.full(len(idx), CODES[colour], np.uint8))
        return self

    def floor_disc(self, centre_xy, radius: float, colour: str = "R") -> Scene:
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (-BASE_H + 0.002 - self.origin[2]) / self.dirs[:, 2]
        p = self.origin + self.dirs * t[:, None]
        idx = np.nonzero((t > 0) & (np.hypot(p[:, 0] - centre_xy[0], p[:, 1] - centre_xy[1]) <= radius))[0]
        self._put(idx, t[idx], np.full(len(idx), CODES[colour], np.uint8))
        return self

    def person(self, distance: float, y: float = 0.0, shirt: str = "R", jeans: str = "B") -> Scene:
        """Torso, arms and legs as boxes 0.2 m deep, standing on the floor, front at x = distance."""
        f, x0, x1 = -BASE_H, distance, distance + 0.2
        return self.boxes([
            ((x0, y - 0.18, f + 0.82), (x1, y + 0.18, f + 1.45), shirt),  # torso
            ((x0, y - 0.26, f + 0.80), (x1, y - 0.19, f + 1.42), shirt),  # arms
            ((x0, y + 0.19, f + 0.80), (x1, y + 0.26, f + 1.42), shirt),
            ((x0, y - 0.16, f + 0.04), (x1, y - 0.02, f + 0.85), jeans),  # legs
            ((x0, y + 0.02, f + 0.04), (x1, y + 0.16, f + 0.85), jeans),
            ((x0 - 0.08, y - 0.17, f), (x1, y - 0.03, f + 0.07), "W"),  # shoes
            ((x0 - 0.08, y + 0.03, f), (x1, y + 0.17, f + 0.07), "W"),
        ])

    def masks(self) -> tuple[np.ndarray, np.ndarray]:
        return (self.code == CODES["R"]).reshape(self.shape), (self.code == CODES["B"]).reshape(self.shape)

    def rgb(self) -> np.ndarray:
        return RGB[self.code].reshape(self.shape + (3,))


def ball_at(x: float, y: float, base_h: float = BASE_H) -> np.ndarray:
    """Ball centre on base_link for a ball on the floor."""
    return np.array([x, y, RADIUS - base_h])


N4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def pixel_noise(mask: np.ndarray, rng, p_edge: float, specks: float, row0: int) -> np.ndarray:
    """Each pixel next to the mask edge flips with probability p_edge; random specks below row0."""
    grow = bv._dilate(mask, N4) & ~mask
    shrink = mask & bv._dilate(~mask, N4)
    out = mask.copy()
    out[grow & (rng.random(mask.shape) < p_edge)] = True
    out[shrink & (rng.random(mask.shape) < p_edge)] = False
    dots = rng.random(mask.shape) < specks
    dots[:row0] = False
    return out | dots


def detect(scene: Scene, params: VisionParams = VisionParams()) -> tuple[BallFix | None, BallVision]:
    vision = BallVision(params)
    return vision.detect_masks(*scene.masks()), vision


def sweep(pos, params: VisionParams = VisionParams(), step: int = 1, noise: tuple[float, float] | None = None,
          seed: int = 0):
    """(orientation name, fix or None, note) for the ball at pos over ORIENTATIONS[::step]."""
    rng = np.random.default_rng(seed)
    vision = BallVision(params)
    out = []
    for name, orient in ORIENTATIONS[::step]:
        red, blue = Scene().ball(ball_at(*pos), orient).masks()
        if noise is not None:
            red, blue = (pixel_noise(m, rng, *noise, vision.row0) for m in (red, blue))
        out.append((name, vision.detect_masks(red, blue), vision.last_note))
    return out


def errors(pos, results) -> dict:
    truth = ball_at(*pos)
    fits = [f for _, f, _ in results if f is not None and f.method == "fit"]
    dy = np.array([f.y - truth[1] for f in fits])
    dr = np.array([f.range - math.hypot(truth[0], truth[1]) for f in fits])
    db = np.degrees([f.bearing - math.atan2(truth[1], truth[0]) for f in fits])
    found = sum(f is not None for _, f, _ in results)
    return {"n": len(results), "found": found, "fit": len(fits), "dy": dy, "dr": dr, "db": db,
            "missed": [(name, note) for name, f, note in results if f is None]}


def summary(pos, e) -> str:
    return (f"ball at {pos}: found {e['found']}/{e['n']} (fit {e['fit']}); y error {e['dy'].min() * 1000:+.1f}.."
            f"{e['dy'].max() * 1000:+.1f} mm, range error {e['dr'].min() * 1000:+.1f}..{e['dr'].max() * 1000:+.1f} mm, "
            f"bearing error {np.abs(e['db']).max():.2f} deg; missed {e['missed'][:4]}")


# --- geometry and interface -----------------------------------------------------------------


def test_interface_defaults_and_old_constructor():
    fix = BallFix(0.5, -0.04, 0.0, (0.0, 0.0, 1.0, 1.0), 100)  # the old positional form
    assert fix.method == "fit" and fix.rho == 0.0
    assert math.isclose(fix.range, math.hypot(0.5, -0.04)) and math.isclose(fix.bearing, math.atan2(-0.04, 0.5))
    p = VisionParams()
    assert p.mount.range_scale == 1.0 and p.box_scale == 0.616 and p.ball_radius == 0.097
    for name in ("ball_radius", "mount", "color", "y_bias", "min_range", "max_range", "ball_z", "z_tol", "z_tol_per_m"):
        assert hasattr(p, name)
    assert {f for f in CameraMount.__dataclass_fields__} == {"x", "y", "z", "tilt_up_deg", "yaw_deg", "range_scale"}


def test_optical_round_trip():
    p = np.array([0.7, -0.1, 0.02])
    for mount in (CameraMount(), CameraMount(yaw_deg=4.0, tilt_up_deg=13.0, range_scale=0.9)):
        assert np.allclose(optical_to_body(body_to_optical(p, mount), mount), p)


def test_pixel_ray_round_trip():
    rng = np.random.default_rng(1)
    uv = np.stack([rng.uniform(0, EQ_WIDTH - 1, 50), rng.uniform(0, EQ_HEIGHT - 1, 50)], 1)
    rays = pixel_rays(uv[:, 0], uv[:, 1])
    assert np.allclose(ray_pixels(rays), uv)
    assert np.allclose(rays[3], pixel_ray(uv[3, 0], uv[3, 1], EQ_WIDTH, EQ_HEIGHT))
    assert np.allclose(ray_pixel(rays[3], EQ_WIDTH, EQ_HEIGHT), uv[3])


def test_numpy_close_and_labels():
    mask = np.zeros((40, 60), bool)
    mask[10:20, 5:15] = True
    mask[10:20, 22:30] = True  # 7-pixel gap: the 9-pixel close joins it
    mask[10:20, 45:50] = True  # 15-pixel gap: separate
    mask[30, 5] = mask[31, 6] = True  # diagonal neighbours: one component
    closed = bv.close_mask(mask, 9, use_cv2=False)
    assert closed[15, 18] and not closed[15, 40] and (closed >= mask).all()
    n, labels = bv.label_mask(closed, use_cv2=False)
    assert n == 3
    assert labels[15, 6] == labels[15, 25] != labels[15, 46]
    assert labels[30, 5] == labels[31, 6] != 0
    kernel = bv._ellipse_offsets(9)  # rows of 1, 7, 7, 9, 9, 9, 7, 7, 1 pixels, as OpenCV
    assert kernel.count((-4, 0)) == 1 and len(kernel) == 57


def test_fit_of_a_two_colour_sphere():
    """Red and blue halves (no white): the fit gives the true circle within a fraction of a pixel."""
    centre = ball_at(0.6, 0.1)
    red, blue = Scene().ball(centre, pattern="RB").masks()
    vision = BallVision()
    blob = bv.find_blobs(red, blue, vision.params.color, vision.row0)[0]
    fit = bv.fit_silhouette(blob.uv, vision.row0)
    c = body_to_optical(centre, CameraMount())
    assert fit is not None and fit.problem(vision.params) is None
    assert abs(fit.rho - math.asin(RADIUS / np.linalg.norm(c))) < 0.3 * PIX_RAD
    assert bv._angle(fit.centre, c / np.linalg.norm(c)) < 0.3 * PIX_RAD


# --- the gore ball at many orientations ---------------------------------------------------------

# (position, largest |y error| [m], largest |range error| [m]) over the orientations where it is found
CASES = [((0.50, -0.046), 0.005, 0.015), ((0.50, 0.040), 0.005, 0.015), ((0.76, 0.0), 0.005, 0.02),
         ((1.0, 0.3), 0.03, 0.08), ((1.5, -0.4), 0.05, 0.13)]


@pytest.mark.parametrize("pos,y_tol,r_tol", CASES)
def test_gore_ball_at_all_orientations(pos, y_tol, r_tol):
    e = errors(pos, sweep(pos))
    print(summary(pos, e))
    assert e["found"] >= 0.9 * e["n"], e["missed"]
    assert e["fit"] == e["found"]  # no colour-box fallback for the ball
    assert np.abs(e["dy"]).max() <= y_tol and np.abs(e["dr"]).max() <= r_tol, summary(pos, e)
    assert np.abs(e["db"]).max() <= 0.5


def test_gore_ball_with_pixel_noise():
    """Edge pixels flip with probability 0.25; random specks on 0.05 % of the floor pixels."""
    pos = (0.50, -0.046)
    e = errors(pos, sweep(pos, step=2, noise=(0.25, 0.0005), seed=3))
    print(summary(pos, e))
    assert e["found"] >= 0.9 * e["n"], e["missed"]
    assert np.abs(e["dy"]).max() <= 0.005 and np.abs(e["dr"]).max() <= 0.015, summary(pos, e)


def test_ball_cut_by_the_first_row():
    """Edge points on the first processed row are not the limb: the fit uses the rest."""
    pos = (0.45, 0.0)
    params = VisionParams(color=ColorParams(row_min_frac=0.57))  # cuts the top quarter of the ball
    vision = BallVision(params)
    red, blue = Scene().ball(ball_at(*pos), ORIENTATIONS[0][1]).masks()
    assert red[:vision.row0].any() or blue[:vision.row0].any()  # the ball reaches above the cut
    fix = vision.detect_masks(red, blue)
    assert fix is not None and fix.method == "fit", vision.last_note
    assert abs(fix.y) <= 0.005 and abs(fix.range - 0.45) <= 0.015


def test_ball_next_to_invalid_pixels():
    """Pixels outside the lens image cut the blob; the cut edge is not the limb."""
    pos = (0.50, -0.046)
    red, blue = Scene().ball(ball_at(*pos), ORIENTATIONS[3][1]).masks()
    u = np.nonzero((red | blue).any(0))[0]
    invalid = np.zeros_like(red)
    invalid[:, :int(u.min() + 0.3 * (u.max() - u.min()))] = True
    vision = BallVision()
    fix = vision.detect_masks(red & ~invalid, blue & ~invalid, invalid)
    assert fix is not None and fix.method == "fit", vision.last_note
    assert abs(fix.y - pos[1]) <= 0.005 and abs(fix.range - math.hypot(*pos)) <= 0.015


def test_colour_box_fallback():
    """A fit that is not reliable gives the colour box, scaled by box_scale only."""
    scene = Scene().ball(ball_at(0.76, 0.0), ORIENTATIONS[0][1])
    no_fit = VisionParams(fit_min_span_deg=361.0)
    fix, vision = detect(scene, no_fit)
    assert fix is not None and fix.method == "box", vision.last_note
    unscaled, _ = detect(scene, VisionParams(fit_min_span_deg=361.0, box_scale=1.0,
                                             mount=CameraMount(range_scale=0.5)))
    cam = np.array([CameraMount().x, CameraMount().y, CameraMount().z])
    ratio = np.linalg.norm(np.array([fix.x, fix.y, fix.z]) - cam) / np.linalg.norm(
        np.array([unscaled.x, unscaled.y, unscaled.z]) - cam)
    assert math.isclose(ratio, 0.616, rel_tol=1e-6)
    assert abs(fix.y) <= 0.03 and fix.rho > 0


# --- what is not the ball ---------------------------------------------------------------------


@pytest.mark.parametrize("distance", [1.0, 2.0, 3.0, 4.0, 4.5, 5.0, 6.0])
@pytest.mark.parametrize("y", [0.0, 0.6])
def test_person_in_red_and_blue_is_not_a_ball(distance, y):
    for shirt, jeans in (("R", "B"), ("B", "R")):
        for row_min_frac in (0.50, 0.45):
            fix, vision = detect(Scene().person(distance, y, shirt, jeans),
                                 VisionParams(color=ColorParams(row_min_frac=row_min_frac)))
            assert fix is None, (shirt, jeans, row_min_frac, vision.last_note)


@pytest.mark.parametrize("distance", [0.5, 0.76, 1.0])
def test_small_red_objects_on_the_floor_are_not_a_ball(distance):
    f = -BASE_H
    scenes = {
        "disc r 0.05": Scene().floor_disc((distance, 0.0), 0.05),
        "disc r 0.10": Scene().floor_disc((distance, 0.0), 0.10),
        "shoe": Scene().boxes([((distance, -0.05, f), (distance + 0.28, 0.05, f + 0.10), "R")]),
        "shoe sideways": Scene().boxes([((distance, -0.14, f), (distance + 0.10, 0.14, f + 0.10), "R")]),
        "sphere r 0.05": Scene().ball((distance, 0.0, 0.05 - BASE_H), radius=0.05, pattern="R"),
    }
    for name, scene in scenes.items():
        for single in (0, 1):  # also with one-colour blobs allowed: not ball-sized at their range
            fix, vision = detect(scene, VisionParams(single_colour=single))
            assert fix is None, (name, single, vision.last_note)


def test_one_colour_sphere_needs_single_colour_option():
    scene = Scene().ball(ball_at(0.5, 0.0), pattern="B")
    fix, vision = detect(scene)
    assert fix is None and "one colour" in vision.last_note
    fix, _ = detect(scene, VisionParams(single_colour=1))
    assert fix is not None and abs(fix.range - 0.5) < 0.01


def test_ball_found_beside_a_refused_person():
    scene = Scene().person(3.0, 1.2).ball(ball_at(0.76, 0.0), ORIENTATIONS[7][1])
    fix, vision = detect(scene)
    assert fix is not None and fix.method == "fit", vision.last_note
    assert abs(fix.x - 0.76) < 0.02 and abs(fix.y) < 0.005


def test_objects_above_the_first_row_are_ignored():
    scene = Scene().ball((1.0, 0.0, 0.45), pattern="RWB")  # a ball on a table, 0.6 m above the floor
    vision = BallVision()
    red, blue = scene.masks()
    assert (red | blue).any() and not (red | blue)[vision.row0:].any()
    assert vision.detect_masks(red, blue) is None


def test_height_gate():
    scene = Scene().ball(ball_at(0.76, 0.0), ORIENTATIONS[0][1])
    fix, _ = detect(scene)
    assert fix is not None and abs(fix.z - (RADIUS - BASE_H)) < 0.005
    assert detect(scene, VisionParams(ball_z=0.10))[0] is None
    low_body = Scene().ball(ball_at(0.76, 0.0, base_h=0.11), ORIENTATIONS[0][1])  # walker 4 cm lower
    assert detect(low_body)[0] is not None


def test_y_bias_and_range_limits():
    scene = Scene().ball(ball_at(1.0, 0.0), ORIENTATIONS[0][1])
    fix, _ = detect(scene, VisionParams(y_bias=0.02))
    assert abs(fix.y + 0.02) < 0.01
    assert detect(scene, VisionParams(max_range=0.8))[0] is None


# --- OpenCV path (the robot) -----------------------------------------------------------------

DS = {"fx": 228.8103, "fy": 228.8103, "cx": 680.0, "cy": 479.0, "xi": -0.00092, "alpha": 0.62007}  # camera_params.yaml
FRAME = (1050, 1400)  # full camera frame (rows, columns)


def ds_project(rays: np.ndarray, m=DS) -> np.ndarray:
    """fisheye_converter.DoubleSphereModel.project (copy for the NumPy check)."""
    x, y, z = rays[..., 0], rays[..., 1], rays[..., 2]
    d1 = np.sqrt(x * x + y * y + z * z)
    k2 = m["xi"] * d1 + z
    den = m["alpha"] * np.sqrt(x * x + y * y + k2 * k2) + (1 - m["alpha"]) * k2
    return np.stack([m["fx"] * x / den + m["cx"], m["fy"] * y / den + m["cy"]], -1)


def ds_unproject(u: np.ndarray, v: np.ndarray, m=DS) -> tuple[np.ndarray, np.ndarray]:
    """Unit optical rays of fisheye pixels (Usenko et al. 2018), and their validity."""
    a, xi = m["alpha"], m["xi"]
    mx, my = (u - m["cx"]) / m["fx"], (v - m["cy"]) / m["fy"]
    r2 = mx * mx + my * my
    valid = r2 <= 1.0 / (2 * a - 1) if a > 0.5 else np.ones_like(r2, bool)
    r2c = np.where(valid, r2, 0.0)
    mz = (1 - a * a * r2c) / (a * np.sqrt(1 - (2 * a - 1) * r2c) + 1 - a)
    k = (mz * xi + np.sqrt(mz * mz + (1 - xi * xi) * r2c)) / (mz * mz + r2c)
    rays = np.stack([k * mx, k * my, k * mz - xi], -1)
    return rays / np.linalg.norm(rays, axis=-1, keepdims=True), valid


def test_double_sphere_unprojection_inverts_projection():
    rng = np.random.default_rng(2)
    rays = rng.normal(size=(500, 3))
    rays[:, 2] = np.abs(rays[:, 2]) + 0.2
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    uv = ds_project(rays)
    back, valid = ds_unproject(uv[:, 0], uv[:, 1])
    assert valid.all() and np.allclose(back, rays, atol=1e-9)


def camera_jpeg(cv2, build, quality: int = 95) -> bytes:
    """A full-size fisheye camera JPEG of a scene: build(scene) adds the objects."""
    vv, uu = np.mgrid[0:FRAME[0], 0:FRAME[1]].astype(float)
    rays, valid = ds_unproject(uu, vv)
    scene = build(Scene(dirs=optical_to_body_dirs(rays, CameraMount()), shape=FRAME))
    rgb = scene.rgb()
    rgb[~valid] = 0
    ok, jpeg = cv2.imencode(".jpg", np.ascontiguousarray(rgb[..., ::-1]), [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return jpeg.tobytes()


def test_numpy_and_opencv_blobs_agree():
    pytest.importorskip("cv2")
    rng = np.random.default_rng(4)
    scene = Scene().person(3.0, 1.2).ball(ball_at(0.6, 0.1), ORIENTATIONS[30][1])
    red, blue = (pixel_noise(m, rng, 0.3, 0.001, 0) for m in scene.masks())
    for mask in (red, blue, red | blue):
        a = bv.close_mask(mask, 9, use_cv2=False)
        assert np.array_equal(a, bv.close_mask(mask, 9, use_cv2=True))
        na, la = bv.label_mask(a, use_cv2=False)
        nb, lb = bv.label_mask(a, use_cv2=True)
        pairs = set(zip(la[a].tolist(), lb[a].tolist()))
        assert na == nb == len(pairs)  # the same partition into components


def test_classify_scene_colours():
    pytest.importorskip("cv2")
    scene = Scene().ball(ball_at(0.6, 0.0), ORIENTATIONS[5][1]).boxes(
        [((1.5, 0.3, -BASE_H), (1.7, 0.5, 0.05 - BASE_H), "O")])
    red, blue = bv.classify(scene.rgb(), ColorParams())
    want_red, want_blue = scene.masks()
    row0 = int(EQ_HEIGHT * ColorParams().row_min_frac)
    want_red[:row0] = want_blue[:row0] = False
    assert np.array_equal(red, want_red) and np.array_equal(blue, want_blue)  # orange is neither


@pytest.mark.parametrize("pos,k", [((0.50, -0.046), 0), ((0.50, -0.046), 40), ((0.50, 0.040), 84), ((0.76, 0.0), 7)])
def test_detect_from_camera_jpeg(pos, k):
    """Full path on the robot: JPEG -> half-size decode -> floor image -> HSV -> fit."""
    cv2 = pytest.importorskip("cv2")
    jpeg = camera_jpeg(cv2, lambda s: s.ball(ball_at(*pos), ORIENTATIONS[k][1]))
    vision = BallVision()
    fix = vision.detect(jpeg)
    assert fix is not None and fix.method == "fit", vision.last_note
    assert abs(fix.y - pos[1]) <= 0.01 and abs(fix.range - math.hypot(*pos)) <= 0.03, vision.last_note


def test_orange_objects_are_not_a_ball():
    """Orange is neither red nor blue. With R and B swapped (BGR read as RGB), orange is blue and
    blue is red: the orange ball would pass with single_colour, the orange-white-blue ball always.
    White gores keep orange and blue apart (a blend of the two can have a red hue)."""
    cv2 = pytest.importorskip("cv2")
    orange = camera_jpeg(cv2, lambda s: s.ball(ball_at(0.6, 0.0), pattern="O"))
    vision = BallVision(VisionParams(single_colour=1))
    assert vision.detect(orange) is None, vision.last_note
    # the white gore between blue (120-180 deg) and orange (240-300 deg) faces the camera
    orange_blue = camera_jpeg(cv2, lambda s: s.ball(ball_at(0.6, 0.0), rot("z", -30.0), pattern="OWBWOW"))
    vision = BallVision()
    assert vision.detect(orange_blue) is None, vision.last_note


def test_detect_without_ball_and_bad_jpeg():
    cv2 = pytest.importorskip("cv2")
    ok, jpeg = cv2.imencode(".jpg", np.full(FRAME + (3,), 100, np.uint8))
    assert ok
    vision = BallVision()
    assert vision.detect(jpeg.tobytes()) is None
    assert vision.detect(b"not a jpeg") is None


def test_save_snapshot(tmp_path):
    cv2 = pytest.importorskip("cv2")
    vision = BallVision()
    assert vision.save_snapshot(tmp_path / "none.jpg") is False  # no frame yet: logs, no exception
    assert vision.detect(camera_jpeg(cv2, lambda s: s.ball(ball_at(0.6, 0.0), ORIENTATIONS[0][1]))) is not None
    path = tmp_path / "snap.jpg"
    assert vision.save_snapshot(path) is True
    assert cv2.imread(str(path)).shape == (EQ_HEIGHT, EQ_WIDTH, 3)
    assert vision.save_snapshot(tmp_path / "missing" / "snap.jpg") is False


def test_floor_mat_wide_two_colour_blob_is_not_a_ball():
    """2026-11-18 robot frame: an orange and teal floor mat formed one 700 px wide
    blob; the box fallback read it as a ball 0.18 m away. It must give no fix."""
    vision = BallVision(VisionParams())
    red = np.zeros((vision.eq_height, EQ_WIDTH), bool)
    blue = np.zeros_like(red)
    rows = slice(int(vision.eq_height * 0.62), vision.eq_height)
    red[rows, 0:EQ_WIDTH:3] = True  # sparse stripes: low fill, touches both side edges
    blue[rows, 1:EQ_WIDTH:7] = True
    assert vision.detect_masks(red, blue) is None
    centre = np.zeros_like(red)  # the same pattern, away from the edges, still low fill
    centre[rows, 200:500:3] = True
    assert vision.detect_masks(centre, np.roll(centre, 1, axis=1)) is None
