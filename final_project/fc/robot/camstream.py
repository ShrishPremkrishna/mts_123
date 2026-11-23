#!/usr/bin/env python3
"""Pi side, optional: the robot's front camera as a small MJPEG stream for the laptop Play screen.

    python3 ~/fc-v1/robot/camstream.py                     # ROS: piggyback /camera/image_raw/compressed
    python3 ~/fc-v1/robot/camstream.py --source picamera2  # no camera node running: open the sensor here

Serves http://<pi>:8090/stream.mjpg (multipart MJPEG), /frame.jpg and /health. Put the URL in
fc/config/robots.json as "cam_url". Low resolution (320 px wide) and capped at --fps (default 8) to
keep Pi load and Wi-Fi small. Never in the control path: the laptop reads it on a side thread.

Read-only camera use:
- `ros` (default) only subscribes (best effort, depth 1) to the topic camera_ros publishes; it never
  opens the sensor or changes a control. Source the same ROS environment and ROS_DOMAIN_ID as the
  running profile (stock: domain 0; tej_u6: 168).
- `picamera2` opens the sensor itself (the sensor can only be opened by one process, see
  follow_me_pupper/robot/viser_camera.py PiCamSource) with a preview configuration and no control
  changes. Use it only when no camera node is running.
Without OpenCV the ROS frames are passed through unscaled.
"""
from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BOUNDARY = "fcframe"


class Latest:
    """Newest JPEG + a condition so stream clients wait for the next frame."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg: bytes | None = None
        self.n, self.t = 0, 0.0

    def put(self, jpeg: bytes) -> None:
        with self.cond:
            self.jpeg, self.n, self.t = jpeg, self.n + 1, time.monotonic()
            self.cond.notify_all()


def make_encoder(width: int, quality: int):
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None, None

    def from_jpeg(data: bytes) -> bytes | None:
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        return None if img is None else from_bgr(img)

    def from_bgr(img) -> bytes | None:
        h, w = img.shape[:2]
        if w > width:
            img = cv2.resize(img, (width, int(h * width / w)), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else None

    return from_jpeg, from_bgr


def run_ros(latest: Latest, topic: str, fps: float, from_jpeg) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage

    rclpy.init()
    node = Node("fc_camstream")
    last = [0.0]

    def on_image(msg):
        now = time.monotonic()
        if now - last[0] < 1.0 / fps:
            return
        last[0] = now
        data = bytes(msg.data)
        out = from_jpeg(data) if from_jpeg else data
        if out:
            latest.put(out)

    node.create_subscription(CompressedImage, topic, on_image, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    rclpy.spin(node)


def run_picamera2(latest: Latest, width: int, fps: float, from_bgr) -> None:
    from picamera2 import Picamera2
    if from_bgr is None:
        raise SystemExit("picamera2 source needs OpenCV")
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": (width, width * 3 // 4)}))
    cam.start()
    try:
        while True:
            t0 = time.monotonic()
            out = from_bgr(cam.capture_array())   # picamera2 "RGB888" arrays are BGR-ordered
            if out:
                latest.put(out)
            time.sleep(max(0.0, 1.0 / fps - (time.monotonic() - t0)))
    finally:
        cam.stop()
        cam.close()


def make_handler(latest: Latest):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):  # noqa: N802
            path = self.path.split("?")[0]
            if path == "/health":
                age = time.monotonic() - latest.t if latest.t else None
                body = (f'{{"frames":{latest.n},"age":{"null" if age is None else round(age, 2)}}}').encode()
                return self._send(200, body, "application/json")
            if path == "/frame.jpg":
                return self._send(200, latest.jpeg, "image/jpeg") if latest.jpeg else self._send(503, b"no frame", "text/plain")
            if path != "/stream.mjpg":
                return self._send(404, b"not found", "text/plain")
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seen = 0
            try:
                while True:
                    with latest.cond:
                        latest.cond.wait_for(lambda: latest.n != seen, timeout=5.0)
                        jpeg, seen = latest.jpeg, latest.n
                    if jpeg:
                        self.wfile.write(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode()
                                         + jpeg + b"\r\n")
                        self.wfile.flush()
            except OSError:
                pass   # client went away

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

    return H


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["ros", "picamera2"], default="ros")
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--quality", type=int, default=60)
    a = ap.parse_args(argv)
    latest = Latest()
    srv = ThreadingHTTPServer((a.bind, a.port), make_handler(latest))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"camstream: http://{a.bind}:{a.port}/stream.mjpg ({a.source})", flush=True)
    from_jpeg, from_bgr = make_encoder(a.width, a.quality)
    try:
        if a.source == "ros":
            run_ros(latest, a.topic, a.fps, from_jpeg)
        else:
            run_picamera2(latest, a.width, a.fps, from_bgr)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()


if __name__ == "__main__":
    main()
