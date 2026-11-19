"""Offline checks of ball_geometry: synthetic balls are projected into the Lab 5
equirectangular image and recovered from their boxes."""

import math

import numpy as np
import pytest

from ball_geometry import (CameraMount, ball_in_optical, level_from_quaternion,
                           optical_to_yaw_frame, pixel_ray, ray_pixel, rot_y, rot_z)

W, H, FOV, R = 700, 572, 220.0, 0.075


def box_of_ball(centre_optical, radius, samples=720):
  """Tight box around the ball's silhouette (a cone of half-angle asin(R/d))."""
  d = np.linalg.norm(centre_optical)
  axis = centre_optical / d
  rho = math.asin(radius / d)
  a = np.cross(axis, [0.0, 1.0, 0.0])
  a /= np.linalg.norm(a)
  b = np.cross(axis, a)
  us, vs = [], []
  for t in np.linspace(0, 2 * math.pi, samples, endpoint=False):
    ray = math.cos(rho) * axis + math.sin(rho) * (math.cos(t) * a + math.sin(t) * b)
    u, v = ray_pixel(ray, W, H, FOV)
    us.append(u)
    vs.append(v)
  return min(us), min(vs), max(us), max(vs)


def yaw_to_optical(p_yaw, mount):
  """Inverse of optical_to_yaw_frame for a level body."""
  p = np.asarray(p_yaw, float) - [mount.x, mount.y, mount.z]
  p = rot_y(math.radians(mount.tilt_up_deg)) @ rot_z(-math.radians(mount.yaw_deg)) @ p
  return np.array([-p[1], -p[2], p[0]])


def test_pixel_ray_round_trip():
  for u, v in [(0, 0), (350, 286), (123.4, 400.2), (699, 571)]:
    assert ray_pixel(pixel_ray(u, v, W, H, FOV), W, H, FOV) == pytest.approx((u, v), abs=1e-6)


@pytest.mark.parametrize("x,y", [(0.45, 0.0), (0.6, 0.2), (1.0, 0.0), (1.0, -0.4), (1.5, 0.3), (2.5, 0.0)])
def test_recovers_ball_on_floor(x, y):
  mount = CameraMount()
  ball = [x, y, R - 0.12]  # centre R above the floor, base_link 0.12 m up
  box = box_of_ball(yaw_to_optical(ball, mount), R)
  p_opt, rho = ball_in_optical(box, W, H, FOV, R)
  got = optical_to_yaw_frame(p_opt, mount)
  assert got[:2] == pytest.approx([x, y], abs=0.01 * math.hypot(x, y) + 0.005)


def test_one_pixel_box_error_at_1p5_m_is_small():
  mount = CameraMount()
  box = np.array(box_of_ball(yaw_to_optical([1.5, 0.0, -0.045], mount), R))
  p_opt, _ = ball_in_optical(box + [0.5, 0.5, -0.5, -0.5], W, H, FOV, R)  # 1 px smaller
  assert optical_to_yaw_frame(p_opt, mount)[0] == pytest.approx(1.5, rel=0.08)


def test_levelling_undoes_body_pitch():
  pitch = math.radians(12.0)
  q = (math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0)  # nose down
  level = level_from_quaternion(*q)
  assert level @ np.array([1.0, 0.0, 0.0]) == pytest.approx([math.cos(pitch), 0.0, -math.sin(pitch)])
  yawed = (math.cos(0.6), 0.0, 0.0, math.sin(0.6))  # yaw alone is removed
  assert level_from_quaternion(*yawed) == pytest.approx(np.eye(3))


def test_degenerate_box_is_rejected():
  p_opt, _ = ball_in_optical((100, 100, 100, 100), W, H, FOV, R)
  assert p_opt is None
