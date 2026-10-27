#!/usr/bin/env python3
"""Your follow-me, as a service other programs can switch on and off.

The robot foundation model lab (Gemini Pupper) has a "follow me" tool. Instead
of the monorepo's simple person follower, it runs *this*: your state machine
from lab/follow_me.py, with your target lock and stop distance, behind the same
interface the monorepo follower has, so nothing else needs to change:

    services  activate_person_following, deactivate_person_following  (std_srvs/Trigger)
    publishes /person_following_cmd_vel  (geometry_msgs/Twist, into cmd_vel_mux)

It is built to sit quietly on the robot all day:

  * The Hailo is only opened while following. Activate loads YOLOv8n-seg and
    subscribes to the camera; deactivate releases both, so between follows the
    accelerator is idle and no camera frames are decoded. (-p always_detect:=true
    keeps detecting at a low rate between follows instead, so the eyes below
    track people all the time -- at the cost of a warmer robot.)
  * No viewer, no /detections topic, no per-frame logging. The viewer from
    follow_me.launch.py is how you tune; this runs what you tuned.
  * Obstacle avoidance is off unless asked for (-p avoid_obstacles:=true).
  * While following it also publishes where people are on ZMQ port 5556, the
    format Gemini Pupper's on-screen eyes read, so the dog face keeps looking
    at the person it follows.

Run it by hand (Gemini Pupper's launch file normally starts it):

    python3 robot/follow_service.py
    ros2 service call /activate_person_following std_srvs/srv/Trigger
    ros2 service call /deactivate_person_following std_srvs/srv/Trigger

It needs camera frames on /camera/image_raw/compressed (the course launch's
camera node, or follow_me.launch.py) and yolov8n_seg.hef
(scripts/download_model.sh).
"""

import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab'))

import follow_me  # noqa: E402  (lab/follow_me.py: your state machine)
from boxes import IMAGE_HEIGHT, IMAGE_WIDTH  # noqa: E402
from fisheye_converter import equirect_height, fisheye_to_equirectangular, load_camera_model
from follow_me import StateMachineNode

HERE = os.path.dirname(os.path.abspath(__file__))
# The same projection viser_camera.py hands the model by default, so boxes come
# out in the 700 x 572 reference frame everything in this lab measures in.
H_FOV_DEG = 220.0
assert equirect_height(IMAGE_WIDTH, H_FOV_DEG) == IMAGE_HEIGHT


def mean_color(mask, box, frame):
    """Mean RGB inside a mask (or else the box), as "r,g,b", or "" if empty."""
    if mask is not None and mask.any():
        pixels = frame[mask]
    else:
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = (int(round(v)) for v in box)
        pixels = frame[max(y1, 0):min(y2, h), max(x1, 0):min(x2, w)].reshape(-1, 3)
    if pixels.size == 0:
        return ""
    r, g, b = pixels.mean(axis=0)
    return f"{r:.0f},{g:.0f},{b:.0f}"


class FollowService(StateMachineNode):
    """follow_me.py's state machine, fed by an on-demand camera + Hailo pipeline."""

    def __init__(self):
        super().__init__()
        self.declare_parameter('follow_class', 'person')
        self.declare_parameter('cmd_vel_topic', '/person_following_cmd_vel')
        self.declare_parameter('avoid_obstacles', False)
        self.declare_parameter('use_color', False)
        self.declare_parameter('detect_hz', 10.0)       # while following
        self.declare_parameter('always_detect', False)  # keep detecting between follows
        self.declare_parameter('idle_detect_hz', 3.0)   # ... at this rate (for the eyes)
        self.declare_parameter('score_threshold', 0.4)
        self.declare_parameter('eyes_port', 5556)  # 0 turns the eye feed off
        self.declare_parameter('verbose', False)

        # follow_me.py logs every frame and every tick, which is what you want
        # while tuning and not on a robot that follows people all afternoon.
        if not self.get_parameter('verbose').value:
            self.get_logger().set_level(LoggingSeverity.WARN)
        follow_me.ENABLE_OBSTACLE_AVOIDANCE = bool(self.get_parameter('avoid_obstacles').value)
        follow_me.target_lock.USE_COLOR = bool(self.get_parameter('use_color').value)

        # Frames come from our own pipeline and on/off from the services, so
        # drop follow_me.py's /detections and /tracking_control subscriptions
        # (a viewer running alongside must not drive this node).
        self.destroy_subscription(self.detection_subscription)
        self.destroy_subscription(self.tracking_control_subscription)

        # Same commands, different topic: the mux decides who drives.
        self.destroy_publisher(self.command_publisher)
        self.command_publisher = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        # Detections arrive on the pipeline thread, ticks on the executor's.
        self.state_lock = threading.Lock()
        self.detector = None
        self.image_sub = None
        self.latest_image = None
        self.image_ready = threading.Event()
        self.wake = threading.Event()  # cuts the wait between frames short
        self.pipeline = None
        self.running = False
        self.camera_model = None

        self.eyes = None
        port = int(self.get_parameter('eyes_port').value)
        if port:
            try:
                import zmq
                self.eyes = zmq.Context.instance().socket(zmq.PUB)
                self.eyes.bind(f"tcp://127.0.0.1:{port}")
            except Exception as exc:  # the eyes are a nicety: never block following
                self.eyes = None
                self.get_logger().warning(f'No eye feed on port {port}: {exc}')

        self.always_detect = bool(self.get_parameter('always_detect').value)
        self.create_service(Trigger, 'activate_person_following', self.on_activate)
        self.create_service(Trigger, 'deactivate_person_following', self.on_deactivate)
        if self.always_detect:
            error = self.start_pipeline()  # if the Hailo is busy, activate tries again
            hailo = (f'detecting at {self.get_parameter("idle_detect_hz").value:g} Hz between follows'
                     if error is None else f'detector not started yet: {error}')
        else:
            hailo = 'Hailo idle until activate_person_following'
        self.get_logger().warning(
            f'follow_service ready ({hailo}; obstacle avoidance '
            f'{"on" if follow_me.ENABLE_OBSTACLE_AVOIDANCE else "off"})')

    # ------------------------------------------------------------ services
    def on_activate(self, request, response):
        if self.tracking_enabled:
            response.success, response.message = True, 'already following'
            return response
        error = self.start_pipeline()
        if error is not None:
            response.success, response.message = False, f'could not start the detector: {error}'
            self.get_logger().error(response.message)
            return response
        target = self.get_parameter('follow_class').value
        with self.state_lock:
            self.start_tracking(target)
        self.wake.set()  # switch to the following rate right away
        response.success, response.message = True, f'following the nearest {target}'
        self.get_logger().warning(response.message)
        return response

    def on_deactivate(self, request, response):
        with self.state_lock:
            if self.tracking_enabled:
                self.stop_tracking()  # publishes a zero command
        if not self.always_detect:
            self.stop_pipeline()
        response.success, response.message = True, 'stopped following'
        self.get_logger().warning(response.message)
        return response

    def start_pipeline(self):
        """Open the Hailo and start detecting. None on success, else why not."""
        if self.running:
            return None
        try:
            from yolo_seg import HailoYoloSeg
            self.detector = HailoYoloSeg(
                model_path=os.path.join(HERE, 'yolov8n_seg.hef'),
                score_threshold=float(self.get_parameter('score_threshold').value))
        except Exception as exc:
            self.detector = None
            return str(exc)
        if not follow_me.ENABLE_OBSTACLE_AVOIDANCE:
            # Only the target's color is ever read; skip the other classes' masks.
            target = self.get_parameter('follow_class').value
            self.detector.mask_classes = {self.class_name_to_id.get(target, 0)}
        self.latest_image = None
        self.image_ready.clear()
        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.on_image, 1)
        self.running = True
        self.pipeline = threading.Thread(target=self.run_pipeline, daemon=True)
        self.pipeline.start()
        return None

    def stop_pipeline(self):
        """Stop detecting and release the Hailo and the camera subscription."""
        if not self.running:
            return
        self.running = False
        self.image_ready.set()
        self.wake.set()
        self.pipeline.join(timeout=3.0)
        self.destroy_subscription(self.image_sub)
        self.image_sub = None
        self.detector.close()  # frees the Hailo for anyone else
        self.detector = None
        self.send_eyes([])

    # ------------------------------------------------------------ pipeline
    def on_image(self, msg):
        # Just keep the newest frame; decoding happens at detect_hz, not 30 Hz.
        self.latest_image = msg
        self.image_ready.set()

    def run_pipeline(self):
        follow_hz = float(self.get_parameter('detect_hz').value)
        idle_hz = float(self.get_parameter('idle_detect_hz').value)
        while self.running:
            started = time.monotonic()
            period = 1.0 / max(follow_hz if self.tracking_enabled else idle_hz, 0.5)
            self.image_ready.wait(timeout=1.0)
            self.image_ready.clear()
            msg, self.latest_image = self.latest_image, None
            if msg is None or not self.running:
                continue
            try:
                self.process(msg)
            except Exception as exc:  # a bad frame or a chip hiccup: try the next one
                self.get_logger().error(f'detection failed: {exc}')
                time.sleep(0.5)
            self.wake.wait(max(0.0, period - (time.monotonic() - started)))
            self.wake.clear()

    def process(self, image_msg):
        bgr = cv2.imdecode(np.frombuffer(image_msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = frame.shape[:2]
        if self.camera_model is None or (self.camera_model.width, self.camera_model.height) != (w, h):
            self.camera_model = load_camera_model(os.path.join(HERE, 'camera_params.yaml'), w, h)
        equirect = fisheye_to_equirectangular(
            frame, self.camera_model, IMAGE_WIDTH, IMAGE_HEIGHT, h_fov_deg=H_FOV_DEG)
        detections = self.detector.infer(equirect)

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        for det in detections:
            x1, y1, x2, y2 = det.box
            d = Detection2D()
            d.header = msg.header
            if det.class_id == self.target_class_id:
                d.id = mean_color(det.mask, det.box, equirect)
            d.bbox.center.position.x = float((x1 + x2) / 2)
            d.bbox.center.position.y = float((y1 + y2) / 2)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det.class_id)
            hyp.hypothesis.score = float(det.score)
            d.results.append(hyp)
            msg.detections.append(d)

        with self.state_lock:
            if self.tracking_enabled:
                self.detection_callback(msg)
        self.send_eyes([d for d in detections if d.class_id == self.target_class_id])

    def send_eyes(self, people):
        """Where people are, in the JSON the monorepo detector put on port 5556."""
        if self.eyes is None:
            return
        out = []
        for i, det in enumerate(people):
            x1, y1, x2, y2 = det.box
            cx, cy = (x1 + x2) / 2 / IMAGE_WIDTH, (y1 + y2) / 2 / IMAGE_HEIGHT
            out.append({
                'x': cx, 'y': cy,
                'width': (x2 - x1) / IMAGE_WIDTH, 'height': (y2 - y1) / IMAGE_HEIGHT,
                'heading': (cx - 0.5) * H_FOV_DEG, 'elevation': (0.5 - cy) * 180.0,
                'id': i,
            })
        try:
            self.eyes.send_string(json.dumps({'people': out, 'timestamp': time.time()}))
        except Exception:
            pass

    # ------------------------------------------------------------ ticks
    def timer_callback(self):
        with self.state_lock:
            super().timer_callback()


def main():
    rclpy.init()
    node = FollowService()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.tracking_enabled:
            node.stop_tracking()
        node.stop_pipeline()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
