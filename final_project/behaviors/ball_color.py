"""Colour detector for a multicolour ball (the red, white and blue beach ball).

YOLOv8's COCO "sports ball" does not fire on the inflatable beach ball, so this
finds it by colour in the equirectangular image: strongly saturated red/orange and
blue pixels below the horizon, joined across the white panels by a morphological
close, kept only if one blob holds both colours. Thresholds were set on a frame
from the Pupper's camera on 2026-11-16 (ball 184 px at 1 m, scattered background
pixels mostly above the floor). The box misses white panels at the edge, so the
range needs the ``range_scale`` calibration.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ColorParams:
  s_min: int = 140  # OpenCV HSV, 0..255
  v_min: int = 70
  red_hue: tuple[int, int] = (13, 165)  # red is h <= 13 or h >= 165 (OpenCV hue 0..179)
  blue_hue: tuple[int, int] = (98, 130)
  row_min_frac: float = 0.45  # ignore rows above this fraction of the image (above the floor)
  close_px: int = 9
  min_pixels: int = 30  # coloured pixels in the blob
  min_each_frac: float = 0.05  # each colour's share of the blob's coloured pixels


@dataclass(frozen=True)
class ColorBall:
  box: tuple[float, float, float, float]
  score: float  # coloured pixels / box area
  pixels: int


def find_ball(rgb: np.ndarray, p: ColorParams = ColorParams()) -> ColorBall | None:
  """The largest two-colour blob in an RGB image, or None."""
  row0 = int(rgb.shape[0] * p.row_min_frac)  # only the floor rows are processed
  hsv = cv2.cvtColor(rgb[row0:], cv2.COLOR_RGB2HSV)
  h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
  strong = (s >= p.s_min) & (v >= p.v_min)
  red = strong & ((h <= p.red_hue[0]) | (h >= p.red_hue[1]))
  blue = strong & (h >= p.blue_hue[0]) & (h <= p.blue_hue[1])
  colored = (red | blue).astype(np.uint8)
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p.close_px, p.close_px))
  joined = cv2.morphologyEx(colored, cv2.MORPH_CLOSE, kernel)
  n, labels, stats, _ = cv2.connectedComponentsWithStats(joined, connectivity=8)
  best = None
  for i in range(1, n):
    x, y, w, hgt, _ = stats[i]
    blob = labels[y:y + hgt, x:x + w] == i
    n_red = int(np.count_nonzero(red[y:y + hgt, x:x + w] & blob))
    n_blue = int(np.count_nonzero(blue[y:y + hgt, x:x + w] & blob))
    total = n_red + n_blue
    if total < p.min_pixels or min(n_red, n_blue) < p.min_each_frac * total:
      continue
    if best is None or total > best.pixels:
      box = (float(x), float(y + row0), float(x + w), float(y + row0 + hgt))
      best = ColorBall(box, total / float(w * hgt), total)
  return best
