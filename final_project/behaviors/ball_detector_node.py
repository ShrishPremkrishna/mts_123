#!/usr/bin/env python3
"""Ball detector for the learned header kick: camera -> /soccer/ball.

Pipeline, reused from the Lab 5 follower (``follow_me_pupper/robot``):
/camera/image_raw/compressed -> 220 deg equirectangular image (700 x 572) ->
a ball box -> ``ball_geometry`` (range from angular size, IMU levelling). The box
comes from ``ball_color`` (mode color, the default: the red/blue beach ball, which
YOLO misses) or from Hailo YOLOv8n-seg's COCO "sports ball" (mode yolo or both).

    publishes  /soccer/ball        geometry_msgs/PointStamped, ball centre in the
                                   base_link yaw-only frame (x forward, y left),
                                   stamped with the image capture time
               /soccer/ball_debug  std_msgs/String JSON per processed frame
    subscribes /camera/image_raw/compressed, /imu_sensor_broadcaster/imu

No detection, no message: the kick bridge measures loss by message age. The
Hailo is opened at start and released on exit, so the Lab 5 follower cannot run
at the same time.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String

from ball_color import ColorParams, find_ball
from ball_geometry import CameraMount, ball_in_optical, level_from_quaternion, optical_to_yaw_frame

EQ_WIDTH = 700  # the follower's IMAGE_WIDTH; height from equirect_height()
H_FOV_DEG = 220.0


class BallDetector(Node):
  def __init__(self):
    super().__init__("ball_detector")
    p = self.declare_parameter
    p("follow_dir", os.path.expanduser("~/mts_123/follow_me_pupper/robot"))
    p("ball_radius", 0.075)  # m; measure the real ball
    p("score_threshold", 0.3)
    p("min_range", 0.15)
    p("max_range", 3.0)
    for name, value in vars(CameraMount()).items():
      p(f"cam_{name}" if not name.startswith("range") else name, value)
    p("use_imu", True)
    p("imu_max_age", 0.2)  # s; older IMU -> no levelling
    p("ball_topic", "/soccer/ball")
    p("frame_id", "base_link_yaw")
    p("snapshot_every_s", 0.0)  # > 0: save the annotated detector input to snapshot_path
    p("snapshot_path", os.path.expanduser("~/kick/logs/ball_snapshot.jpg"))
    # color: ball_color (the red/blue beach ball, no Hailo); yolo: COCO "sports ball";
    # both: YOLO first, colour when YOLO finds nothing.
    p("mode", "color")
    # CPU (1.2 cores at full rate overheated the Pi on 2026-11-16): decode the JPEG
    # at half size (the 700 px wide output barely changes), cap the frame rate,
    # and in colour mode remap only the floor rows that ball_color reads.
    p("decode_half", True)
    p("max_rate_hz", 15.0)  # 0 = every frame

    follow_dir = self.get_parameter("follow_dir").value
    sys.path.insert(0, follow_dir)
    from fisheye_converter import equirect_height, fisheye_to_equirectangular, load_camera_model
    try:
      from fisheye_converter import equirectangular_maps
    except ImportError:  # an older Lab 5 copy without the map cache: remap every row
      equirectangular_maps = None

    self.mode = self.get_parameter("mode").value
    if self.mode not in ("color", "yolo", "both"):
      raise ValueError(f"mode must be color, yolo or both, not {self.mode!r}")
    self._to_equirect, self._load_model = fisheye_to_equirectangular, load_camera_model
    self._maps = equirectangular_maps
    self.eq_height = equirect_height(EQ_WIDTH, H_FOV_DEG)
    self.decode_half = bool(self.get_parameter("decode_half").value)
    rate = float(self.get_parameter("max_rate_hz").value)
    self.period = 1.0 / rate if rate > 0 else 0.0
    self.row0 = (int(self.eq_height * ColorParams().row_min_frac)
                 if self.mode == "color" and equirectangular_maps is not None else 0)
    self.camera_yaml = os.path.join(follow_dir, "camera_params.yaml")
    self.camera_model = None
    self.detector = None
    if self.mode != "color":
      from yolo_seg import HailoYoloSeg
      started = time.monotonic()
      self.detector = HailoYoloSeg(model_path=os.path.join(follow_dir, "yolov8n_seg.hef"),
                                   score_threshold=float(self.get_parameter("score_threshold").value),
                                   want_masks=False)
      self.ball_class = self.detector.labels.index("sports ball")
      self.get_logger().info(f"Hailo ready in {time.monotonic() - started:.1f} s, class id {self.ball_class}")

    self.mount = CameraMount(**{
      name: float(self.get_parameter(f"cam_{name}" if not name.startswith("range") else name).value)
      for name in vars(CameraMount())})
    self.radius = float(self.get_parameter("ball_radius").value)
    self.ball_pub = self.create_publisher(PointStamped, self.get_parameter("ball_topic").value, 10)
    self.debug_pub = self.create_publisher(String, "/soccer/ball_debug", 10)
    self.latest = None
    self.imu = None  # (receive time, quaternion w, x, y, z)
    self.ready = threading.Event()
    self.running = True
    self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.on_image, 1)
    if self.get_parameter("use_imu").value:
      self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self.on_imu, qos_profile_sensor_data)
    self.worker = threading.Thread(target=self.run, daemon=True)
    self.worker.start()
    self.get_logger().info(f"ball detector up: radius {self.radius} m, mount {self.mount}")

  def on_image(self, msg) -> None:
    self.latest = msg
    self.ready.set()

  def on_imu(self, msg) -> None:
    q = msg.orientation
    self.imu = (time.monotonic(), q.w, q.x, q.y, q.z)

  def run(self) -> None:
    next_t = 0.0
    while self.running:
      if not self.ready.wait(timeout=0.5):
        continue
      delay = next_t - time.monotonic()
      if delay > 0:  # rate cap: wait, then take the newest frame
        time.sleep(delay)
      self.ready.clear()
      msg, self.latest = self.latest, None
      if msg is None:
        continue
      next_t = time.monotonic() + self.period
      try:
        self.process(msg)
      except Exception as exc:  # a bad frame or a chip hiccup: try the next one
        self.get_logger().error(f"detection failed: {exc}")
        time.sleep(0.2)

  def process(self, msg) -> None:
    t0 = time.monotonic()
    flag = cv2.IMREAD_REDUCED_COLOR_2 if self.decode_half else cv2.IMREAD_COLOR
    bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), flag)
    if bgr is None:
      return
    h, w = bgr.shape[:2]
    if self.camera_model is None or (self.camera_model.width, self.camera_model.height) != (w, h):
      self.camera_model = self.load_model(w, h)
    eq = cv2.cvtColor(self.equirect(bgr), cv2.COLOR_BGR2RGB)
    detections, box, score, debug = [], None, 0.0, {}
    if self.detector is not None:
      detections = self.detector.infer(eq)
      balls = [d for d in detections if d.class_id == self.ball_class]
      debug.update(infer_ms=round(self.detector.last_inference_ms, 1),
                   top=[[d.class_name, round(float(d.score), 2)]
                        for d in sorted(detections, key=lambda d: -d.score)[:4]])
      if balls:
        best = max(balls, key=lambda d: d.score)
        box, score, debug["src"] = best.box, float(best.score), "yolo"
    if box is None and self.mode in ("color", "both"):
      found = find_ball(eq)
      if found is not None:
        box, score, debug["src"] = found.box, found.score, "color"
    self.maybe_snapshot(eq, detections, box)
    if box is not None:
      p_opt, rho = ball_in_optical(box, EQ_WIDTH, self.eq_height, H_FOV_DEG, self.radius)
      level = None
      if self.imu is not None and time.monotonic() - self.imu[0] < self.get_parameter("imu_max_age").value:
        level = level_from_quaternion(*self.imu[1:])
      debug.update(score=round(score, 2), box=[round(float(v), 1) for v in box],
                   rho_deg=round(math.degrees(rho), 2), levelled=level is not None)
      if p_opt is not None:
        x, y, z = optical_to_yaw_frame(p_opt, self.mount, level)
        rng = math.hypot(x, y)
        debug.update(x=round(float(x), 3), y=round(float(y), 3), z=round(float(z), 3),
                     cam_dist=round(float(np.linalg.norm(p_opt)), 3))
        if self.get_parameter("min_range").value <= rng <= self.get_parameter("max_range").value:
          out = PointStamped()
          out.header.stamp = msg.header.stamp  # capture time, robot clock
          out.header.frame_id = self.get_parameter("frame_id").value
          out.point.x, out.point.y, out.point.z = float(x), float(y), float(z)
          self.ball_pub.publish(out)
          debug["published"] = True
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    debug.update(age_ms=round((self.get_clock().now().nanoseconds * 1e-9 - stamp) * 1000.0, 1),
                 proc_ms=round((time.monotonic() - t0) * 1000.0, 1), img=[w, h])
    self.debug_pub.publish(String(data=json.dumps(debug)))

  def load_model(self, w: int, h: int):
    """The lens model for a decoded w x h frame; camera_params.yaml is for the full frame."""
    model = self._load_model(self.camera_yaml, w, h)
    if self.decode_half:  # pixel i of the half frame covers full pixels 2i and 2i+1
      model.fx, model.fy = model.fx * 0.5, model.fy * 0.5
      model.cx, model.cy = (model.cx + 0.5) * 0.5 - 0.5, (model.cy + 0.5) * 0.5 - 0.5
    return model

  def equirect(self, bgr: np.ndarray) -> np.ndarray:
    """The equirectangular image; in colour mode only rows >= row0 hold pixels."""
    if not self.row0:
      return self._to_equirect(bgr, self.camera_model, EQ_WIDTH, self.eq_height, h_fov_deg=H_FOV_DEG)
    map_x, map_y, invalid = self._maps(self.camera_model, EQ_WIDTH, self.eq_height, H_FOV_DEG)
    r = self.row0
    eq = np.zeros((self.eq_height, EQ_WIDTH, 3), np.uint8)
    eq[r:] = cv2.remap(bgr, map_x[r:], map_y[r:], interpolation=cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    eq[r:][invalid[r:]] = 0
    return eq

  def maybe_snapshot(self, eq, detections, box) -> None:
    every = float(self.get_parameter("snapshot_every_s").value)
    now = time.monotonic()
    if every <= 0 or now - getattr(self, "_last_snapshot", -1e9) < every:
      return
    self._last_snapshot = now
    img = cv2.cvtColor(eq, cv2.COLOR_RGB2BGR)
    for d in detections:
      x1, y1, x2, y2 = (int(round(v)) for v in d.box)
      cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 1)
      cv2.putText(img, f"{d.class_name} {d.score:.2f}", (x1, max(10, y1 - 3)),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    if box is not None:  # the box used for the estimate
      x1, y1, x2, y2 = (int(round(v)) for v in box)
      cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
    cv2.imwrite(self.get_parameter("snapshot_path").value, img)

  def close(self) -> None:
    self.running = False
    self.ready.set()
    self.worker.join(timeout=2.0)
    if self.detector is not None:
      self.detector.close()  # frees the Hailo


def main():
  rclpy.init()
  node = BallDetector()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.close()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == "__main__":
  main()
