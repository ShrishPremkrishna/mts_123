"""Ray-traced synthetic balls in real onboard fisheye frames, with exact labels.

Every pixel's ray comes from the same double sphere lens the detector uses
(ball_eye.Lens.rays); a sphere of the selected radius is intersected in the
optical frame, shaded, given pale panels and a soft floor shadow, then the
frame is JPEG-encoded like the camera topic. The label is the sphere centre on
base_link, so errors measured on these frames are geometric/pipeline errors
under the same lens and mount, never physical camera accuracy.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

SATURATED_PINK = (222, 46, 150)  # RGB; inside pink_ball's saturated range when lit
PALE_PINK = (246, 182, 204)
_RAYS = {}


@dataclass(frozen=True)
class Label:
    x: float
    y: float
    z: float
    range_m: float
    rho: float


def lens_rays(lens):
    key = lens.record().__repr__()
    if key not in _RAYS:
        rays, seen = lens.rays()
        rays = rays / np.linalg.norm(rays, axis=-1, keepdims=True)
        _RAYS.clear()
        _RAYS[key] = (rays.astype(np.float32), seen)
    return _RAYS[key]


def render_ball(background_rgb, lens, mount, ball_xy, radius, base_height, *, rng=None, noise=2.0,
                jpeg_quality=88, shadow=True, panels=True):
    """Return (JPEG bytes, RGB frame, Label) for a ball resting on the floor at body (x, y)."""
    import cv2

    rng = rng or np.random.default_rng(0)
    h, w = background_rgb.shape[:2]
    if (w, h) != (lens.width, lens.height):
        lens = lens.scaled(w, h)
    rays, seen = lens_rays(lens)
    centre_body = np.array([ball_xy[0], ball_xy[1], radius - base_height])
    centre = mount.to_optical(centre_body)
    dist = float(np.linalg.norm(centre))
    if dist <= radius:
        raise ValueError("camera is inside the ball")
    rho = math.asin(radius / dist)
    out = background_rgb.astype(np.float32).copy()
    along = rays @ centre.astype(np.float32)  # ray parameter of closest approach
    cos_theta = np.clip(along / dist, -1, 1)
    theta = np.arccos(cos_theta)
    # Local angular pixel pitch for antialiasing the silhouette.
    pitch = np.maximum(np.linalg.norm(np.diff(rays, axis=1, append=rays[:, -1:]), axis=-1), 1e-4)
    cover = np.clip((rho - theta) / pitch + 0.5, 0, 1) * seen
    if shadow:
        cam = np.array([mount.x, mount.y, mount.z])
        body_rays = rays @ mount.rot().T.astype(np.float32)
        down = body_rays[..., 2]
        floor = down < -1e-3
        t = np.where(floor, (-base_height - cam[2]) / np.minimum(down, -1e-3), 0)
        fx, fy = cam[0] + t * body_rays[..., 0], cam[1] + t * body_rays[..., 1]
        d = np.hypot(fx - ball_xy[0], fy - ball_xy[1]) / radius
        dark = np.where(floor, 0.45 * np.exp(-((d / 0.9) ** 4)), 0) * (1 - cover) * seen
        out *= (1 - dark)[..., None]
    region = cover > 0
    d = rays[region]
    a = along[region]
    disc = np.maximum(a * a - (dist * dist - radius * radius), 0)
    hit = (a - np.sqrt(disc))[:, None] * d
    normal_opt = (hit - centre) / radius
    normal_body = normal_opt @ mount.rot().T
    light = np.array([0.3, 0.4, 0.87])
    lambert = np.clip(normal_body @ (light / np.linalg.norm(light)), 0, 1)
    shade = 0.58 + 0.42 * lambert
    colour = np.tile(np.array(SATURATED_PINK, np.float32), (len(d), 1))
    if panels:
        # Pale panels around fixed directions on the ball (a pentagon-like pattern).
        dirs = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
                         [.58, .58, .58], [-.58, -.58, .58]], np.float32)
        pale = (normal_body @ (dirs / np.linalg.norm(dirs, axis=1, keepdims=True)).T).max(axis=1) > 0.93
        colour[pale] = PALE_PINK
    ball = colour * shade[:, None]
    c = cover[region][:, None]
    out[region] = out[region] * (1 - c) + ball * c
    if noise:
        out += rng.normal(0, noise, out.shape)
    rgb = np.clip(out, 0, 255).astype(np.uint8)
    ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    decoded = cv2.cvtColor(cv2.imdecode(jpeg, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    return jpeg.tobytes(), decoded, Label(*map(float, centre_body), dist, rho)


def plain_background(lens, value=(150, 140, 135)):
    """Uniform floor-coloured frame for deterministic unit tests."""
    return np.full((lens.height, lens.width, 3), value, np.uint8)
