"""Measure where tags are relative to each other with no setup at all.

Any tags lying flat on the same surface, anywhere. One tag is the reference:
its centre is the origin, +x points along its right edge, +y along its top
edge. Because every tag is a square of known printed size, that is enough to
turn pixels into metres on that surface, and no lens calibration is needed
(the surface-to-image mapping is found directly from the tag corners).

Only valid for things lying ON that surface.
"""
from __future__ import annotations

import math

import cv2
import numpy as np


def _square(size: float) -> np.ndarray:
    """Corners TL, TR, BR, BL of a tag in its own frame (x right, y up, seen from above)."""
    h = size / 2
    return np.array([[-h, h], [h, h], [h, -h], [-h, -h]], float)


def _fit_square(pts: np.ndarray, size: float):
    """Best rigid placement (centre, yaw) of a `size` square onto 4 measured points."""
    model = _square(size)
    centre = pts.mean(axis=0)
    d = pts - centre
    # 2D Procrustes without scale: the rotation that best maps model -> d.
    yaw = math.atan2(float(np.sum(model[:, 0] * d[:, 1] - model[:, 1] * d[:, 0])),
                     float(np.sum(model[:, 0] * d[:, 0] + model[:, 1] * d[:, 1])))
    c, s = math.cos(yaw), math.sin(yaw)
    return centre, yaw, model @ np.array([[c, s], [-s, c]]) + centre


def tag_corners(pose, size: float) -> np.ndarray:
    """Corners TL, TR, BR, BL on the surface of a tag at pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return _square(size) @ np.array([[c, s], [-s, c]]) + (x, y)


def _refine(img: dict, sizes: dict, ids: list, ref_id: int, H_img_to_plane, poses: dict, iters: int = 40):
    """Joint least squares: one surface-to-image mapping plus (x, y, yaw) of every
    non-reference tag, minimising how far the predicted corners land from the
    detected ones, in pixels. Small problem, so plain Levenberg-Marquardt with a
    numerical Jacobian is plenty.
    """
    others = [i for i in ids if i != ref_id]
    # Work in centred, scaled pixel units so the unknowns have similar magnitudes.
    all_px = np.vstack([img[i] for i in ids])
    mu, sc = all_px.mean(axis=0), float(np.abs(all_px - all_px.mean(axis=0)).max()) or 1.0
    T = np.array([[1 / sc, 0, -mu[0] / sc], [0, 1 / sc, -mu[1] / sc], [0, 0, 1]])
    obs = np.concatenate([((img[i] - mu) / sc).ravel() for i in ids])
    G = T @ np.linalg.inv(H_img_to_plane)  # surface (m) -> normalised pixels
    G = G / G[2, 2]

    def unpack(p):
        Gp = np.append(p[:8], 1.0).reshape(3, 3)
        tag = {ref_id: (0.0, 0.0, 0.0)}
        tag.update({i: tuple(p[8 + 3 * k: 11 + 3 * k]) for k, i in enumerate(others)})
        return Gp, tag

    def residual(p):
        Gp, tag = unpack(p)
        out = []
        for i in ids:
            pts = np.hstack([tag_corners(tag[i], sizes[i]), np.ones((4, 1))]) @ Gp.T
            out.append((pts[:, :2] / pts[:, 2:3]).ravel())
        return np.concatenate(out) - obs

    p = np.concatenate([G.ravel()[:8]] + [np.array(poses[i], float) for i in others])
    r = residual(p)
    cost, lam = float(r @ r), 1e-3
    for _ in range(iters):
        J = np.empty((len(r), len(p)))
        for k in range(len(p)):
            step = 1e-6 * max(1.0, abs(p[k]))
            dp = p.copy()
            dp[k] += step
            J[:, k] = (residual(dp) - r) / step
        A, g = J.T @ J, J.T @ r
        improved = False
        for _try in range(8):
            try:
                delta = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            r_new = residual(p + delta)
            cost_new = float(r_new @ r_new)
            if cost_new < cost:
                gain = cost - cost_new
                p, r, cost, lam, improved = p + delta, r_new, cost_new, lam / 3, True
                break
            lam *= 10
        if not improved or gain < 1e-14:
            break
    Gp, tag = unpack(p)
    H = np.linalg.inv(np.linalg.inv(T) @ Gp)  # pixels -> surface (m)
    return H / H[2, 2], tag, math.sqrt(cost / len(r)) * sc


def survey_tags(tags: dict[int, np.ndarray], sizes: dict[int, float], ref_id: int, refine: bool = True):
    """tags: {id: image corners (4,2) TL,TR,BR,BL}. sizes: {id: side of the black square, m}.

    Returns ({id: {"x", "y", "yaw"}} in the reference tag's frame, H) where H
    maps image pixels to metres on the surface, or (None, None) if the
    reference tag is not in `tags`. refine=False uses the reference tag alone
    (kept for comparison: one small tag extrapolates badly).
    """
    if ref_id not in tags:
        return None, None
    ids = [i for i in tags if i in sizes]
    img = {i: np.asarray(tags[i], np.float64) for i in ids}
    # First guess from the reference tag alone...
    H = cv2.getPerspectiveTransform(img[ref_id].astype(np.float32), _square(sizes[ref_id]).astype(np.float32))
    poses = {ref_id: (0.0, 0.0, 0.0)}
    for i in ids:
        if i != ref_id:
            on_plane = cv2.perspectiveTransform(img[i].reshape(-1, 1, 2), H).reshape(-1, 2)
            centre, yaw, _ = _fit_square(on_plane, sizes[i])
            poses[i] = (float(centre[0]), float(centre[1]), yaw)
    # ...then let every tag's known shape and size pull the mapping into agreement.
    if refine and len(ids) > 1:
        H, poses, _rms_px = _refine(img, sizes, ids, ref_id, H, poses)
    return {i: {"x": float(x), "y": float(y), "yaw": float(yaw)} for i, (x, y, yaw) in poses.items()}, H


def pair_distances(poses: dict[int, dict]) -> dict[tuple[int, int], float]:
    ids = sorted(poses)
    return {(a, b): math.hypot(poses[a]["x"] - poses[b]["x"], poses[a]["y"] - poses[b]["y"])
            for n, a in enumerate(ids) for b in ids[n + 1:]}
