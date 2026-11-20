"""Replay camera-only FC ball recognition on archived Pi JPEGs and an optical texture fixture."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time
from types import SimpleNamespace

import cv2
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "follow_me_pupper/robot"))
from fisheye_converter import load_camera_model, fisheye_to_equirectangular
from pink_ball import PinkBallTracker


def detail(values):
    if not values:
        return None
    return {key: round(float(np.percentile(values, q)), 3) for key, q in (("p50", 50), ("p95", 95), ("max", 100))}


def replay(bag, start, end, patch):
    rows = dict(negative=Counter(), composited=Counter(), mocked_confirmation=Counter())
    trackers = [PinkBallTracker() for _ in rows]
    centers, times, pre, pixel_equal = [], [], [], 0
    missed, max_missed = 0, 0
    model = None
    digest = hashlib.sha256()
    with bag.open("rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        for _, _, _, msg in reader.iter_decoded_messages(topics=["/camera/image_raw/compressed"],
                         starting_at=start - 500_000_000, ending_before=end + 500_000_000):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if not start <= stamp < end:
                continue
            jpeg = bytes(msg.data)
            digest.update(hashlib.sha256(jpeg).digest())
            tick = time.perf_counter()
            raw = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if model is None:
                h, w = raw.shape[:2]
                model = load_camera_model(ROOT / "follow_me_pupper/robot/camera_params.yaml", w, h)
            projected = fisheye_to_equirectangular(raw, model, 700, 572)
            rgb = cv2.cvtColor(projected, cv2.COLOR_BGR2RGB)
            detection_start = time.perf_counter()
            before = trackers[0].observe(rgb, [], stamp, "pink")
            ended = time.perf_counter()
            rows["negative"][before["state"]] += 1
            index = sum(rows["negative"].values()) - 1
            if index:  # report steady-state CPU time, excluding first remap table build
                times.append((ended - detection_start) * 1000)
                pre.append((detection_start - tick) * 1000)
            previous = fisheye_to_equirectangular(cv2.cvtColor(raw, cv2.COLOR_BGR2RGB), model, 700, 572)
            pixel_equal += np.array_equal(previous, rgb)
            x = round(350 + 230 * math.sin(index / 41))
            y = round(377 + 62 * math.cos(index / 27))
            r = (6, 12, 24, 42)[(index // 30) % 4]
            size = 2 * r + 1
            circle = np.zeros((size, size), np.uint8)
            cv2.circle(circle, (r, r), r, 255, -1)
            region = rgb[y - r:y + r + 1, x - r:x + r + 1]
            region[circle > 0] = cv2.resize(patch, (size, size))[circle > 0]
            box = SimpleNamespace(class_name="sports ball", box=(x - r, y - r, x + r + 1, y + r + 1))
            for tracker, mode, boxes, key in ((trackers[1], "pink", [], "composited"),
                                               (trackers[2], "confirmed", [box], "mocked_confirmation")):
                row = tracker.observe(rgb, boxes, stamp, mode)
                outcome = row["state"]
                if outcome == "tracked":
                    left, top, right, bottom = row["box"]
                    error = math.hypot((left + right) * 350 - x, (top + bottom) * 286 - y)
                    outcome = "correct" if error <= r else "wrong_location"
                    if key == "composited" and outcome == "correct":
                        centers.append(error)
                rows[key][outcome] += 1
                if key == "composited":
                    missed = 0 if outcome == "correct" else missed + 1
                    max_missed = max(missed, max_missed)
    return dict(frames=sum(rows["negative"].values()), jpeg_sequence_sha256=digest.hexdigest(), results=rows,
                same_projected_pixels=int(pixel_equal), tracked_center_error_px=detail(centers),
                longest_consecutive_composite_misses=max_missed, cpu_color_stage_ms=detail(times),
                cpu_jpeg_and_projection_ms=detail(pre), hardware=platform.machine(),
                opencv=cv2.__version__, python=platform.python_version())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("photo", type=Path, help="Nov 20 original 2524x1582 setup-preview.png")
    # The Nov 17 source-clock window documented in the archive's visual-analysis-local/extract.py.
    parser.add_argument("--start-ns", type=int, default=int((1794905684.8667839 + 64) * 1e9))
    parser.add_argument("--end-ns", type=int, default=int((1794905684.8667839 + 113) * 1e9))
    args = parser.parse_args()
    image = cv2.imread(str(args.photo))
    if image is None or image.shape[:2] != (1582, 2524):
        parser.error("Expected the original field preview with the pink ball at (862, 530)")
    texture = cv2.cvtColor(image[506:555, 838:887], cv2.COLOR_BGR2RGB)
    result = replay(args.bag, args.start_ns, args.end_ns, texture)
    print(json.dumps(result, indent=2))
