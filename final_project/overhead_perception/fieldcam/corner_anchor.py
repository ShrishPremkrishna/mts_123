"""Estimate the camera from one surveyed floor square and bound extrapolation."""
import math

import cv2
import numpy as np

from .camera_model import camera_pose_candidates
from .tagmeasure import tag_corners

# Software acceptance budget in metres; verify the resulting overlay against the field.
FIELD_TOLERANCE_M = .075
CORNER_NOISE_PX = .35


def anchor_geometry(settings):
    side, end = settings["corner"].split("_")
    x = (settings["length"] / 2 - settings["inset_x"]) * (1 if side == "east" else -1)
    y = (settings["width"] / 2 - settings["inset_y"]) * (1 if end == "north" else -1)
    return tag_corners((x, y, math.radians(settings["rotation"])), settings["tag_size"])


def corner_fit(K, dist, observed, anchor, field_corners, size, far_reference=None):
    """Check both planar poses, the view footprint, and far-field pixel sensitivity."""
    field_xy = np.asarray(field_corners, float)
    field_xyz = np.column_stack((field_xy, np.zeros(4)))
    if min(np.linalg.norm(observed - np.roll(observed, 1, axis=0), axis=1)) < 20:
        return None, "The corner tag is too small in this view. Enlarge the printed tag or use the six-tag layout.", {}
    if far_reference is not None:
        far_xy, far_px = far_reference
        anchor = np.vstack((anchor, far_xy))
        observed = np.vstack((observed, far_px))
    anchor_xyz = np.column_stack((anchor, np.zeros(len(anchor))))
    candidates = camera_pose_candidates(K, dist, observed, anchor)
    valid = []
    for cam in candidates:
        uv = cam.project(field_xyz)
        if (np.isfinite(uv).all() and (field_xyz @ cam.R[2] + cam.tvec[2, 0] > 0).all()
                and (uv >= 0).all() and (uv < size).all()):
            valid.append(cam)
    if not valid:
        return None, "The predicted field leaves the picture. Check the tag corner, rotation and offsets, then frame both end lines.", {}
    cam = valid[0]
    metrics = {"tag_fit_px": round(cam.reproj_err_px, 3)}
    if cam.reproj_err_px > 1.:
        return None, "The printed square does not fit within 1 pixel. Flatten the tag and check focus and lens settings.", metrics
    uv = cam.project(field_xyz)
    for other in valid[1:]:
        if other.reproj_err_px <= cam.reproj_err_px + 2 * CORNER_NOISE_PX:
            spread = np.linalg.norm(other.pixel_to_plane(uv, 0) - field_xy, axis=1).max()
            if not np.isfinite(spread) or spread > FIELD_TOLERANCE_M:
                return None, "Two camera poses fit this square but disagree across the field. Enlarge the tag or use distributed tags.", metrics

    # Propagate independent corner-localization noise through the six pose parameters.
    _, J = cv2.projectPoints(anchor_xyz, cam.rvec, cam.tvec, K, dist)
    _, F = cv2.projectPoints(field_xyz, cam.rvec, cam.tvec, K, dist)
    dop = J[:, :6]
    if np.linalg.matrix_rank(dop) < 6:
        return None, "The single-tag view cannot resolve camera tilt. Use a larger tag or the six-tag layout.", metrics
    pixel_axes = (cam.pixel_to_plane(uv + [1, 0], 0) - field_xy, cam.pixel_to_plane(uv + [0, 1], 0) - field_xy)
    jac = F[:, :6].reshape(4, 2, 6) @ np.linalg.pinv(dop)
    noise = np.full(2 * len(observed), CORNER_NOISE_PX)
    if far_reference is not None:
        noise[-2:] = 2.0  # one manually identified line intersection
    field_jac = (np.stack(pixel_axes, axis=-1) @ jac) * noise
    sensitivity = 2 * np.linalg.svd(field_jac, compute_uv=False).max()
    if not np.isfinite(sensitivity):
        return None, "The single-tag fit is unstable. Enlarge the tag or use the six-tag layout.", metrics
    metrics["far_sensitivity_m"] = round(float(sensitivity), 4)
    if sensitivity > FIELD_TOLERANCE_M:
        return None, (f"Far-corner sensitivity is {sensitivity * 100:.0f} cm (limit 7.5 cm). "
                      "Enlarge the printed tag or use distributed tags."), metrics
    return cam, None, metrics


class FarCorner:
    """Track the explicitly marked projected line intersection on the fixed mount."""
    def __init__(self, gray, pixel):
        height, width = gray.shape
        self.uv = np.array(pixel, float)
        self.x, self.y = map(round, self.uv)
        if not (24 <= self.x < width-24 and 24 <= self.y < height-24):
            raise ValueError("Frame the opposite corner with space around it, then take a new snapshot.")
        self.patch = gray[self.y-10:self.y+11, self.x-10:self.x+11].copy()
        if self.patch.std() < 12 or self.locate(gray) is None:
            raise ValueError("Choose a distinct floor-line intersection at the opposite corner. Add a small tape cross if needed.")

    def locate(self, gray):
        view = gray[self.y-24:self.y+25, self.x-24:self.x+25]
        scores = cv2.matchTemplate(view, self.patch, cv2.TM_CCOEFF_NORMED)
        _, best, _, (x, y) = cv2.minMaxLoc(scores)
        if best < .80 or math.hypot(x-14, y-14) > 3:
            return None
        # A straight edge matches many points along itself; require an isolated peak.
        yy, xx = np.ogrid[:scores.shape[0], :scores.shape[1]]
        if scores[(xx-x)**2 + (yy-y)**2 >= 25].max() > best-.055:
            return None
        return self.uv + [x-14, y-14]
