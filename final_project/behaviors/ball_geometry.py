"""Ball position from one detection box in the Lab 5 equirectangular image.

The follower's pipeline remaps the fisheye frame to an equirectangular image that
spans ``h_fov_deg`` horizontally and 180 deg vertically, so every pixel is a known
direction (``fisheye_converter.create_equirectangular_rays``). For a ball of known
radius R the box gives:

    angular radius rho   half the angle between the rays through opposite box
                         edges (the larger of the horizontal and vertical pairs,
                         because the image edge or the robot's nose can cut one)
    camera distance      d = R / sin(rho)
    direction            the normalised sum of the four edge rays

The range therefore does not depend on camera height or tilt, which change while
the robot runs. Pure NumPy, no ROS; ``ball_detector_node`` wraps it.

Frames: optical (x right, y down, z forward) -> body (x forward, y left, z up) ->
yaw-only (body levelled with the IMU roll and pitch), the frame the learned kick
policy reads the ball in.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraMount:
  x: float = 0.15  # m, camera from the base_link origin, forward
  y: float = 0.0  # m, left
  z: float = 0.03  # m, up
  tilt_up_deg: float = 10.0  # optical axis above the body x axis
  yaw_deg: float = 0.0  # optical axis to the left of the body x axis
  range_scale: float = 1.0  # calibration: true distance / measured distance


def pixel_ray(u: float, v: float, width: int, height: int, h_fov_deg: float) -> np.ndarray:
  """Unit ray (optical frame) through equirectangular pixel (u, v)."""
  hf = math.radians(h_fov_deg)
  lon = u / (width - 1) * hf - hf / 2
  lat = v / (height - 1) * math.pi - math.pi / 2
  return np.array([math.cos(lat) * math.sin(lon), math.sin(lat), math.cos(lat) * math.cos(lon)])


def ray_pixel(ray: np.ndarray, width: int, height: int, h_fov_deg: float) -> tuple[float, float]:
  """Inverse of pixel_ray (used by the tests)."""
  x, y, z = ray / np.linalg.norm(ray)
  hf = math.radians(h_fov_deg)
  lon = math.atan2(x, z)
  lat = math.asin(max(-1.0, min(1.0, y)))
  return (lon + hf / 2) / hf * (width - 1), (lat + math.pi / 2) / math.pi * (height - 1)


def _angle(a: np.ndarray, b: np.ndarray) -> float:
  return math.atan2(np.linalg.norm(np.cross(a, b)), float(np.dot(a, b)))


def ball_in_optical(box, width: int, height: int, h_fov_deg: float, radius: float):
  """(ball centre in the optical frame [m], angular radius [rad]) from a box (x1, y1, x2, y2)."""
  x1, y1, x2, y2 = box
  uc, vc = (x1 + x2) / 2.0, (y1 + y2) / 2.0
  left, right = pixel_ray(x1, vc, width, height, h_fov_deg), pixel_ray(x2, vc, width, height, h_fov_deg)
  top, bottom = pixel_ray(uc, y1, width, height, h_fov_deg), pixel_ray(uc, y2, width, height, h_fov_deg)
  rho = max(_angle(left, right), _angle(top, bottom)) / 2.0
  if not 1e-4 < rho < math.pi / 2:
    return None, rho
  direction = left + right + top + bottom
  direction /= np.linalg.norm(direction)
  return radius / math.sin(rho) * direction, rho


def rot_y(a: float) -> np.ndarray:
  c, s = math.cos(a), math.sin(a)
  return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
  c, s = math.cos(a), math.sin(a)
  return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def level_from_quaternion(w: float, x: float, y: float, z: float) -> np.ndarray:
  """Rotation from the body frame to its yaw-only frame (roll and pitch removed)."""
  r = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ])
  return rot_z(-math.atan2(r[1, 0], r[0, 0])) @ r


def optical_to_yaw_frame(p_optical: np.ndarray, mount: CameraMount,
                         level: np.ndarray | None = None) -> np.ndarray:
  """Point from the optical frame to the base_link yaw-only frame."""
  p = np.array([p_optical[2], -p_optical[0], -p_optical[1]]) * mount.range_scale
  p = rot_z(math.radians(mount.yaw_deg)) @ rot_y(-math.radians(mount.tilt_up_deg)) @ p
  p = p + np.array([mount.x, mount.y, mount.z])
  return p if level is None else level @ p
