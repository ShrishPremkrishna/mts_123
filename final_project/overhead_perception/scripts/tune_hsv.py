#!/usr/bin/env python3
"""Tune the ball's HSV range under the real arena lights.

The ball has several colours, each with its own range in ball.colors; this shows
one at a time (--range 0, 1, 2...). Drag the sliders until only that colour of the ball is white in the mask, everywhere on the
field (centre, corners, in a robot's shadow). p = print values for field.yaml, q = quit.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.capture import FrameGrabber  # noqa: E402
from fieldcam.config import load_config  # noqa: E402

NAMES = ["H lo", "S lo", "V lo", "H hi", "S hi", "V hi"]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", default=None)
    ap.add_argument("--range", type=int, default=0, help="which entry of ball.colors to start from")
    args = ap.parse_args()
    cfg = load_config()
    c = cfg["camera"]
    source = args.source if args.source is not None else c["source"]
    source = int(source) if str(source).isdigit() else source
    cap = FrameGrabber(source, c["width"], c["height"], c["fps"])
    cv2.namedWindow("mask")
    for name, val, top in zip(NAMES, list(cfg["ball"]["colors"][args.range]["lo"]) + list(cfg["ball"]["colors"][args.range]["hi"]),
                              [179, 255, 255] * 2):
        cv2.createTrackbar(name, "mask", val, top, lambda _v: None)
    n = -1
    while True:
        frame, _, n = cap.read(n)
        if frame is None:
            continue
        v = [cv2.getTrackbarPos(name, "mask") for name in NAMES]
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), np.array(v[:3], np.uint8), np.array(v[3:], np.uint8))
        both = np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        cv2.imshow("mask", cv2.resize(both, None, fx=0.5, fy=0.5))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("p"):
            print(f"    - {{lo: {v[:3]}, hi: {v[3:]}}}")
        if key == ord("q"):
            break


if __name__ == "__main__":
    main()
