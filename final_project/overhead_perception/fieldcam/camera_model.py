"""Camera geometry: intrinsics, pose from floor tags, and pixel <-> field mapping.

The whole localisation idea lives here. Everything we track sits on a plane of
known height (floor tags at z=0, ball centre at z=radius, robot tag at
z=tag_height), so one camera is enough: back-project the pixel to a ray and
intersect it with that plane. No stereo triangulation needed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


def default_intrinsics(width: int, height: int, hfov_deg: float):
    """Pinhole guess from field of view. Fine for a first run on an iPhone 1x lens."""
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    K = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], float)
    return K, np.zeros(5)


def load_intrinsics(path: Path, width: int, height: int):
    """Returns (K, dist) scaled to the current frame size, or None if no file."""
    if not path.exists():
        return None
    d = yaml.safe_load(path.read_text())
    K = np.array(d["K"], float)
    sx, sy = width / d["width"], height / d["height"]
    K[0] *= sx
    K[1] *= sy
    return K, np.array(d["dist"], float)


def save_intrinsics(path: Path, K, dist, width: int, height: int, rms: float):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump({
        "width": width, "height": height, "rms_px": float(rms),
        "K": np.asarray(K).tolist(), "dist": np.asarray(dist).ravel().tolist(),
    }))


@dataclass
class CameraModel:
    K: np.ndarray
    dist: np.ndarray
    rvec: np.ndarray  # field -> camera
    tvec: np.ndarray
    reproj_err_px: float = 0.0

    def __post_init__(self):
        self.R, _ = cv2.Rodrigues(self.rvec)
        self.center = (-self.R.T @ self.tvec.reshape(3)).ravel()  # camera position in field frame

    def pixel_to_plane(self, uv, z: float) -> np.ndarray:
        """Pixels (N,2) -> field xy (N,2) on the horizontal plane at height z."""
        uv = np.asarray(uv, float).reshape(-1, 1, 2)
        n = cv2.undistortPoints(uv, self.K, self.dist).reshape(-1, 2)
        rays_cam = np.hstack([n, np.ones((len(n), 1))])
        rays = rays_cam @ self.R  # == (R.T @ ray) for each row
        s = (z - self.center[2]) / rays[:, 2]
        return self.center[:2] + s[:, None] * rays[:, :2]

    def project(self, pts_xyz) -> np.ndarray:
        """Field points (N,3) -> pixels (N,2)."""
        pts = np.asarray(pts_xyz, float).reshape(-1, 1, 3)
        px, _ = cv2.projectPoints(pts, self.rvec, self.tvec, self.K, self.dist)
        return px.reshape(-1, 2)


def solve_camera_pose(K, dist, image_pts, field_xy) -> CameraModel | None:
    """Camera pose from >= 4 floor points (z = 0) with known field coordinates."""
    image_pts = np.asarray(image_pts, float).reshape(-1, 1, 2)
    obj = np.hstack([np.asarray(field_xy, float), np.zeros((len(field_xy), 1))]).reshape(-1, 1, 3)
    if len(obj) < 4:
        return None
    ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, image_pts, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    best = None
    for rvec, tvec in zip(rvecs, tvecs):  # planar PnP has two candidate poses
        rvec, tvec = cv2.solvePnPRefineLM(obj, image_pts, K, dist, rvec, tvec)
        cam = CameraModel(K, dist, rvec, tvec)
        if cam.center[2] <= 0:  # camera must be above the floor
            continue
        err = float(np.sqrt(np.mean(np.sum((cam.project(obj) - image_pts.reshape(-1, 2)) ** 2, axis=1))))
        cam.reproj_err_px = err
        if best is None or err < best.reproj_err_px:
            best = cam
    return best
