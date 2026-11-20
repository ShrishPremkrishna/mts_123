"""Rectify a fixed horizontal body-height plane for metric patch sizing and yaw."""
import cv2
import numpy as np


class PatchPlane:
    def __init__(self, cam, height, length, width, image_size):
        self.cam, self.height = cam, height
        self.origin = np.array([-length / 2, width / 2])
        px = cam.project([[0, 0, height], [.3, 0, height], [0, .3, height]])
        nominal_ppm = np.linalg.norm(px[1:] - px[0], axis=1).mean() / .3
        self.ppm = min(nominal_ppm, 1000 / length, 800 / width)
        shape = np.ceil(np.array([width, length]) * self.ppm).astype(int) + 1
        self.map = np.empty((*shape, 2), np.float32)
        # Chunk projection so OpenCV only creates a small temporary Jacobian.
        for first in range(0, shape[0], 16):
            last = min(first + 16, shape[0])
            rows, cols = np.mgrid[first:last, :shape[1]]
            xy = self.to_xy(np.column_stack((cols.ravel(), rows.ravel())))
            xyz = np.column_stack((xy, np.full(len(xy), height)))
            self.map[first:last] = cam.project(xyz).reshape(last-first, shape[1], 2)
        self.roi = np.uint8(((self.map >= 1) & (self.map < np.array(image_size)-1)).all(axis=-1)) * 255

    def to_xy(self, points):
        return self.origin + np.asarray(points) * [1/self.ppm, -1/self.ppm]

    def image_pixels(self, xy):
        coords = np.asarray(xy).reshape(-1, 2)
        return self.cam.project(np.column_stack((coords, np.full(len(coords), self.height))))

    def detect(self, frame):
        from .team_colors import detect_team_colors
        overhead = cv2.remap(frame, self.map, None, cv2.INTER_LINEAR)
        return detect_team_colors(overhead, .30 * self.ppm, roi_mask=self.roi)
