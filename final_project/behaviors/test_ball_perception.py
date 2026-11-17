"""Offline coordinate/time regressions; no ROS, camera, OpenCV or model execution.

The node is imported with test-local dependency stubs. Its real callbacks and
publication gate run against synthetic detection geometry and source stamps.
"""

import importlib.util
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest

from ball_geometry import (
  CameraMount, capture_seconds, level_from_quaternion, optical_to_yaw_frame,
)


def stamp(t):
  sec, nanosec = divmod(round(t * 1_000_000_000), 1_000_000_000)
  return NS(sec=sec, nanosec=nanosec)


def imu(t, q=(1.0, 0.0, 0.0, 0.0), frame="base_link", available=True):
  return NS(header=NS(stamp=stamp(t), frame_id=frame),
            orientation=NS(w=q[0], x=q[1], y=q[2], z=q[3]),
            orientation_covariance=[0.0 if available else -1.0] + [0.0] * 8)


def frame(t):
  return NS(header=NS(stamp=stamp(t)), data=b"synthetic JPEG dependency stub")


@pytest.fixture
def detector(monkeypatch):
  def stub(name, **members):
    module = ModuleType(name)
    module.__dict__.update(members)
    monkeypatch.setitem(sys.modules, name, module)
    return module

  class PointStamped:
    def __init__(self):
      self.header = NS(stamp=None, frame_id="")
      self.point = NS(x=0.0, y=0.0, z=0.0)

  stub("rclpy")
  stub("rclpy.executors", ExternalShutdownException=RuntimeError)
  stub("rclpy.node", Node=object)
  stub("rclpy.qos", qos_profile_sensor_data=object())
  for name in ("geometry_msgs", "sensor_msgs", "std_msgs"):
    stub(name)
  stub("geometry_msgs.msg", PointStamped=PointStamped)
  stub("sensor_msgs.msg", CompressedImage=object, Imu=object)
  stub("std_msgs.msg", String=NS)
  stub("cv2", IMREAD_REDUCED_COLOR_2=1, IMREAD_COLOR=2, COLOR_BGR2RGB=3,
       imdecode=lambda *_: np.zeros((4, 4, 3), dtype=np.uint8),
       cvtColor=lambda value, _: value)
  stub("ball_color", ColorParams=object,
       find_ball=lambda _: NS(box=(300.0, 300.0, 400.0, 400.0), score=0.9))
  spec = importlib.util.spec_from_file_location(
    "_isolated_ball_detector", Path(__file__).with_name("ball_detector_node.py"))
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  node = module.BallDetector.__new__(module.BallDetector)
  node.params = {"use_imu": True, "imu_max_age": 0.2, "max_image_age": 0.5,
                 "frame_id": "base_link_yaw",
                 "min_range": 0.15, "max_range": 3.0}
  node.clock = 20.0
  node.get_parameter = lambda name: NS(value=node.params[name])
  node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round(node.clock * 1e9)))
  node.imu_history, node.last_image_stamp = (), 0.0
  node.last_imu_stamp = 0.0
  node.decode_half, node.mode, node.detector = True, "color", None
  node.camera_model = NS(width=4, height=4)
  node.eq_height, node.radius, node.mount = 572, 0.075, CameraMount()
  node.equirect = lambda image: image
  node.maybe_snapshot = lambda *_: None
  node.balls, node.debug = [], []
  node.ball_pub = NS(publish=node.balls.append)
  node.debug_pub = NS(publish=lambda msg: node.debug.append(json.loads(msg.data)))
  return node


def test_capture_stamp_is_preserved_when_leveling_is_valid(detector):
  detector.on_imu(imu(19.9))
  image = frame(19.95)
  detector.process(image)
  assert len(detector.balls) == 1
  ball = detector.balls[0]
  assert ball.header.frame_id == "base_link_yaw"
  assert ball.header.stamp is image.header.stamp
  assert capture_seconds(ball.header.stamp.sec, ball.header.stamp.nanosec) == 19.95
  assert detector.debug[-1]["age_ms"] == pytest.approx(50.0)


def test_history_covers_a_delayed_frame_at_the_configured_imu_rate(detector):
  for i in range(260):
    detector.on_imu(imu(19.0 + i / 260))
  detector.process(frame(19.501))
  assert len(detector.balls) == 1
  assert len(detector.imu_history) == 256
  assert detector.debug[-1]["age_ms"] == pytest.approx(499.0)


@pytest.mark.parametrize("sample", [
  None, imu(19.7), imu(20.1), imu(19.99), imu(0.0),
  imu(19.9, frame="imu_link"), imu(19.9, frame=""), imu(19.9, available=False),
  imu(19.9, q=(0.0, 0.0, 0.0, 0.0)), imu(19.9, q=(math.nan, 0.0, 0.0, 0.0)),
])
def test_unusable_imu_never_publishes_body_coordinates_as_yaw_frame(detector, sample):
  if sample is not None:
    detector.on_imu(sample)
  detector.process(frame(19.95))
  assert not detector.balls
  assert detector.debug[-1]["published"] is False
  assert detector.debug[-1]["levelled"] is False
  assert detector.debug[-1]["reason"]


def test_delayed_image_uses_preceding_orientation_instead_of_newest_arrival(detector):
  detector.on_imu(imu(19.9))
  detector.process(frame(19.91))
  expected = vars(detector.balls.pop().point)
  detector.on_imu(imu(19.95, q=(math.cos(0.3), math.sin(0.3), 0.0, 0.0)))
  detector.process(frame(19.92))  # arrives after the newer IMU sample
  assert vars(detector.balls[0].point) == pytest.approx(expected)


def test_reordered_imu_does_not_replace_the_valid_preceding_sample(detector):
  detector.on_imu(imu(19.9))
  detector.on_imu(imu(19.8, q=(math.cos(0.3), math.sin(0.3), 0.0, 0.0)))
  detector.on_imu(imu(19.9))  # duplicate receipt is not a fresh sample
  assert [item[0] for item in detector.imu_history] == [19.9]
  detector.clock = 20.2
  detector.process(frame(20.15))
  assert not detector.balls
  assert "stale" in detector.debug[-1]["reason"]


def test_new_invalid_orientation_clears_old_valid_leveling(detector):
  detector.on_imu(imu(19.9))
  detector.on_imu(imu(19.94, available=False))
  detector.process(frame(19.95))
  assert not detector.balls
  assert detector.imu_history == ()


def test_delayed_imu_cannot_restore_orientation_after_a_newer_invalid_sample(detector):
  detector.on_imu(imu(19.9))
  detector.on_imu(imu(19.94, available=False))
  detector.on_imu(imu(19.92))
  detector.process(frame(19.95))
  assert not detector.balls
  detector.on_imu(imu(19.96))
  detector.process(frame(19.97))
  assert len(detector.balls) == 1  # a genuinely later valid sample can recover


def test_processing_delay_preserves_valid_alignment_and_original_image_age(detector):
  detector.on_imu(imu(19.94))
  detector.maybe_snapshot = lambda *_: setattr(detector, "clock", 20.2)
  image = frame(19.95)  # 250 ms old after processing; IMU is only 10 ms before capture
  detector.process(image)
  assert len(detector.balls) == 1
  assert detector.balls[0].header.stamp is image.header.stamp
  assert detector.debug[-1]["age_ms"] == pytest.approx(250.0)


def test_inference_delay_past_image_budget_withholds_even_aligned_estimate(detector):
  detector.on_imu(imu(19.94))
  detector.maybe_snapshot = lambda *_: setattr(detector, "clock", 20.5)
  detector.process(frame(19.95))  # 550 ms old after processing
  assert not detector.balls
  assert detector.debug[-1]["reason"] == "image capture is stale"


def test_fresh_image_cannot_use_an_imu_sample_outside_alignment_budget(detector):
  detector.on_imu(imu(19.7))
  detector.process(frame(19.95))  # image 50 ms old, but IMU 250 ms before capture
  assert not detector.balls
  assert detector.debug[-1]["reason"] == "IMU source sample is stale for image capture"


@pytest.mark.parametrize("name,value", [("use_imu", False), ("frame_id", "field"),
                                        ("imu_max_age", math.nan), ("imu_max_age", -1.0),
                                        ("max_image_age", math.nan), ("max_image_age", -1.0)])
def test_incompatible_configuration_refuses_the_estimate(detector, name, value):
  detector.params[name] = value
  detector.on_imu(imu(19.9))
  detector.process(frame(19.95))
  assert not detector.balls
  assert detector.debug[-1]["reason"]


@pytest.mark.parametrize("captured_at", [0.0, 20.1])
def test_zero_and_future_image_stamps_do_not_publish(detector, captured_at):
  detector.on_imu(imu(19.9))
  detector.process(frame(captured_at))
  assert not detector.balls


def test_duplicate_and_reordered_images_cannot_refresh_ball_estimates(detector):
  detector.on_imu(imu(19.9))
  detector.process(frame(19.95))
  detector.process(frame(19.95))
  detector.process(frame(19.94))
  assert len(detector.balls) == 1
  assert "reordered" in detector.debug[-1]["reason"]


@pytest.mark.parametrize("sec,nanosec", [(0, 0), (-1, 0), (1, -1), (1, 1_000_000_000),
                                         (math.inf, 0), (math.nan, 0), (True, 0)])
def test_invalid_source_stamp_is_not_replaced_with_current_time(sec, nanosec):
  assert capture_seconds(sec, nanosec) is None


@pytest.mark.parametrize("q", [(0, 0, 0, 0), (2, 0, 0, 0), (math.inf, 0, 0, 0),
                               (math.nan, 0, 0, 0)])
def test_invalid_quaternion_is_not_a_level_pose(q):
  with pytest.raises(ValueError):
    level_from_quaternion(*q)


def test_leveling_removes_yaw_but_preserves_roll_transform():
  # Body yaw 90 degrees, roll 30 degrees. A one-metre body-forward point stays
  # forward in yaw coordinates; a body-left point tilts up by 30 degrees.
  a, b = math.pi / 4, math.pi / 12
  q = (math.cos(a) * math.cos(b), math.cos(a) * math.sin(b),
       math.sin(a) * math.sin(b), math.sin(a) * math.cos(b))
  level = level_from_quaternion(*(v * 1.001 for v in q))
  assert level @ [1, 0, 0] == pytest.approx([1, 0, 0])
  assert level @ [0, 1, 0] == pytest.approx([0, math.cos(2 * b), math.sin(2 * b)])


def test_optical_transform_requires_leveling_and_applies_it_to_mount_offset():
  mount = CameraMount(x=0.15, y=0.0, z=0.03, tilt_up_deg=0.0)
  with pytest.raises(ValueError):
    optical_to_yaw_frame(np.array([0, 0, 1]), mount, None)
  pitch = math.pi / 6
  level = level_from_quaternion(math.cos(pitch / 2), 0, math.sin(pitch / 2), 0)
  result = optical_to_yaw_frame(np.array([0, 0, 1]), mount, level)
  assert result == pytest.approx([1.15 * math.cos(pitch) + 0.03 * math.sin(pitch),
                                 0, -1.15 * math.sin(pitch) + 0.03 * math.cos(pitch)])
