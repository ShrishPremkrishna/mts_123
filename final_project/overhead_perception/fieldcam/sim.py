"""Synthetic arena: renders what the god camera would see for known ground truth.

Used by the tests, and by `run_perception.py --source sim` so the gameplay code
can be developed before the real arena exists.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from .camera_model import CameraModel
from .detect import TAG_DICT


def look_at(cam_pos, target, up=(0.0, 1.0, 0.0)):
    """rvec, tvec for a camera at cam_pos looking at target (field frame)."""
    cam_pos, target = np.asarray(cam_pos, float), np.asarray(target, float)
    z = target - cam_pos
    z /= np.linalg.norm(z)
    down = -np.asarray(up, float)
    y = down - (down @ z) * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.stack([x, y, z])
    rvec, _ = cv2.Rodrigues(R)
    return rvec, (-R @ cam_pos).reshape(3, 1)


def _tag_image(tag_id: int, px: int = 160) -> np.ndarray:
    """Tag with the white quiet zone a printed tag has."""
    inner = cv2.aruco.generateImageMarker(TAG_DICT, tag_id, px)
    return cv2.copyMakeBorder(inner, px // 4, px // 4, px // 4, px // 4, cv2.BORDER_CONSTANT, value=255)


class SimArena:
    def __init__(self, cfg: dict, K, cam_pos=(0.0, -0.2, 2.6), cam_target=(0.0, 0.0, 0.0),
                 size=(1920, 1080), noise_sigma: float = 3.0, seed: int = 0,
                 robot_tag_size: float = 0.08):
        self.cfg, self.size, self.noise = cfg, size, noise_sigma
        self.robot_tag_size = robot_tag_size  # side of the black square, m
        rvec, tvec = look_at(cam_pos, cam_target)
        self.cam = CameraModel(np.asarray(K, float), np.zeros(5), rvec, tvec)
        self.rng = np.random.default_rng(seed)
        self._floor = self._render_floor()

    def _render_floor(self) -> np.ndarray:
        f = self.cfg["field"]
        ppm, m = 400, 0.4
        L, W = f["length"], f["width"]
        cw, ch = int((L + 2 * m) * ppm), int((W + 2 * m) * ppm)
        to_px = lambda x, y: (int((x + L / 2 + m) * ppm), int((W / 2 + m - y) * ppm))  # noqa: E731
        canvas = np.full((ch, cw, 3), (95, 105, 100), np.uint8)
        cv2.rectangle(canvas, to_px(-L / 2, W / 2), to_px(L / 2, -W / 2), (70, 120, 60), -1)
        cv2.rectangle(canvas, to_px(-L / 2, W / 2), to_px(L / 2, -W / 2), (235, 235, 235), 4)
        cv2.line(canvas, to_px(0, W / 2), to_px(0, -W / 2), (235, 235, 235), 4)
        side = int(0.15 * ppm * 1.5)  # 15 cm tag plus quiet zone
        single = self.cfg.get("field_setup", {}).get("mode") == "corner"
        for tid, (x, y) in self.cfg["tags"]["field"].items():
            if single:
                continue
            tag = cv2.cvtColor(cv2.resize(_tag_image(tid), (side, side), interpolation=cv2.INTER_AREA),
                               cv2.COLOR_GRAY2BGR)
            cx, cy = to_px(x, y)
            canvas[cy - side // 2: cy - side // 2 + side, cx - side // 2: cx - side // 2 + side] = tag
        # canvas px -> field plane -> image
        A = np.array([[1 / ppm, 0, -(L / 2 + m)], [0, -1 / ppm, W / 2 + m], [0, 0, 1]])
        Hf = self.cam.K @ np.column_stack([self.cam.R[:, 0], self.cam.R[:, 1], self.cam.tvec.ravel()])
        view = cv2.warpPerspective(canvas, Hf @ A, self.size, borderValue=(60, 60, 60))
        if single:
            from .corner_anchor import anchor_geometry
            setup = self.cfg["field_setup"]
            square = anchor_geometry(setup)
            center = square.mean(axis=0)
            quiet = center + (square - center) * 1.5
            sheet = cv2.cvtColor(_tag_image(setup["tag_id"]), cv2.COLOR_GRAY2BGR)
            self._paste_quad(view, sheet, np.column_stack((quiet, np.zeros(4))))
        return view

    def _paste_quad(self, img, texture, corners_xyz):
        h, w = texture.shape[:2]
        dst = self.cam.project(corners_xyz).astype(np.float32)
        M = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), dst)
        warped = cv2.warpPerspective(texture, M, self.size)
        mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), M, self.size)
        img[mask > 127] = warped[mask > 127]

    def _draw_ball(self, img, c, rad):
        """A mini basketball seen from above: grey with a red side panel, a blue band and
        black seams, turned to a random angle each frame (the real one rolls)."""
        n = 2 * rad + 3
        plain = self.cfg["ball"].get("sim_bgr")
        if plain is not None:  # a single-colour ball (e.g. pink), slightly darker toward the rim
            o = n // 2
            yy, xx = np.mgrid[0:n, 0:n]
            shade = 1.0 - 0.25 * np.clip(np.hypot(xx - o, yy - o) / max(rad, 1), 0, 1) ** 2
            patch = np.clip(np.array(plain, np.float32) * shade[..., None], 0, 255).astype(np.uint8)
            disc = np.zeros((n, n), np.uint8)
            cv2.circle(disc, (o, o), rad, 255, -1, cv2.LINE_AA)
            return self._blend(img, patch, disc, c, o, n)
        patch = np.full((n, n, 3), (190, 200, 195), np.uint8)
        o = n // 2
        cv2.rectangle(patch, (0, 0), (int(o - 0.55 * rad), n), (60, 50, 215), -1)                     # red
        cv2.ellipse(patch, (int(o + 0.25 * rad), o), (int(0.38 * rad), int(1.2 * rad)), 0, 0, 360, (190, 80, 40), -1)  # blue
        t = max(1, rad // 14)
        cv2.line(patch, (int(o - 0.55 * rad), 0), (int(o - 0.55 * rad), n), (25, 25, 25), t)
        cv2.ellipse(patch, (int(o + 0.25 * rad), o), (int(0.38 * rad), int(1.2 * rad)), 0, 0, 360, (25, 25, 25), t)
        M = cv2.getRotationMatrix2D((o, o), float(self.rng.uniform(0, 360)), 1.0)
        patch = cv2.warpAffine(patch, M, (n, n), borderMode=cv2.BORDER_REPLICATE)
        disc = np.zeros((n, n), np.uint8)
        cv2.circle(disc, (o, o), rad, 255, -1, cv2.LINE_AA)
        self._blend(img, patch, disc, c, o, n)

    @staticmethod
    def _blend(img, patch, disc, c, o, n):
        x0, y0 = c[0] - o, c[1] - o
        xs, ys = slice(max(x0, 0), min(x0 + n, img.shape[1])), slice(max(y0, 0), min(y0 + n, img.shape[0]))
        if xs.start >= xs.stop or ys.start >= ys.stop:
            return
        ps = patch[ys.start - y0: ys.stop - y0, xs.start - x0: xs.stop - x0].astype(np.float32)
        al = disc[ys.start - y0: ys.stop - y0, xs.start - x0: xs.stop - x0].astype(np.float32)[..., None] / 255
        img[ys, xs] = (al * ps + (1 - al) * img[ys, xs]).astype(np.uint8)

    def render(self, robots: dict, ball_xy=None) -> np.ndarray:
        """robots: {name: (x, y, yaw)} body poses. ball_xy: (x, y) or None."""
        img = self._floor.copy()
        for name, (x, y, yaw) in robots.items():
            rc = self.cfg["robots"][name]
            c, s = math.cos(yaw), math.sin(yaw)
            fwd, left = np.array([c, s]), np.array([-s, c])
            h = rc["tag_height"]

            def quad(center, half_f, half_l):
                # TL, TR, BR, BL with "top" = robot forward, as seen from above
                pts = [center + half_f * fwd + half_l * left, center + half_f * fwd - half_l * left,
                       center - half_f * fwd - half_l * left, center - half_f * fwd + half_l * left]
                return [[p[0], p[1], h] for p in pts]

            body = np.array([x, y])
            self._paste_quad(img, np.full((40, 70, 3), (40, 40, 40), np.uint8), quad(body, 0.17, 0.10))
            if self.cfg.get("robot_tracking") == "colors":
                for point, radius, color in ((body, .045, (225, 56, 35) if name == "striker" else (32, 40, 226)),
                                              (body + .09 * fwd, .018, (25, 235, 152))):
                    theta = np.linspace(0, 2*math.pi, 32, endpoint=False)
                    ring = point + radius * np.column_stack((np.cos(theta), np.sin(theta)))
                    pixels = self.cam.project(np.column_stack((ring, np.full(len(ring), h))))
                    cv2.fillConvexPoly(img, np.int32(np.round(pixels)), color)
                continue
            tag_c = body + rc["tag_offset"][0] * fwd + rc["tag_offset"][1] * left
            # Undo yaw_offset so a tag mounted rotated on the robot is simulated rotated too.
            c2, s2 = math.cos(yaw - rc["yaw_offset"]), math.sin(yaw - rc["yaw_offset"])
            fwd, left = np.array([c2, s2]), np.array([-s2, c2])
            tag = cv2.cvtColor(_tag_image(rc["tag_id"]), cv2.COLOR_GRAY2BGR)
            half = 0.75 * self.robot_tag_size  # tag plus its white quiet zone
            self._paste_quad(img, tag, quad(tag_c, half, half))
        if ball_xy is not None:
            r = self.cfg["ball"]["radius"]
            center = np.array([ball_xy[0], ball_xy[1], r])
            px = self.cam.project([center, center + self.cam.R[0] * r])
            rad = float(np.linalg.norm(px[1] - px[0]))
            self._draw_ball(img, (int(round(px[0][0])), int(round(px[0][1]))), int(round(rad)))
        img = cv2.GaussianBlur(img, (3, 3), 0)
        if self.noise > 0:
            img = np.clip(img + self.rng.normal(0, self.noise, img.shape), 0, 255).astype(np.uint8)
        return img
