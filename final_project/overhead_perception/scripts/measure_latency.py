#!/usr/bin/env python3
"""Measure glass-to-Python camera latency.

Point the camera at this laptop's screen. The window flips black <-> white;
we time how long until the camera frames show the flip. Put the median in
field.yaml as camera.latency_s. (Includes a few ms of display lag, so the true
camera latency is slightly lower; good enough for prediction.)
"""
import argparse
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.capture import FrameGrabber  # noqa: E402
from fieldcam.config import load_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", default=None)
    ap.add_argument("--trials", type=int, default=20)
    args = ap.parse_args()
    c = load_config()["camera"]
    source = args.source if args.source is not None else c["source"]
    source = int(source) if str(source).isdigit() else source
    cap = FrameGrabber(source, c["width"], c["height"], c["fps"])

    cv2.namedWindow("flash", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("flash", 900, 700)
    screen = np.zeros((700, 900), np.uint8)
    white, n, results = False, -1, []

    def brightness():
        nonlocal n
        frame, t, n = cap.read(n)
        return (float(frame.mean()), t) if frame is not None else (None, 0.0)

    for trial in range(args.trials + 1):
        cv2.imshow("flash", screen)
        cv2.waitKey(700)  # let exposure settle
        base = statistics.mean(b for b, _ in (brightness() for _ in range(5)) if b is not None)
        white = not white
        screen[:] = 255 if white else 0
        cv2.imshow("flash", screen)
        cv2.waitKey(1)
        t_flip = time.time()
        while time.time() - t_flip < 2.0:
            b, t = brightness()
            if b is not None and abs(b - base) > 12:
                if trial > 0:  # first flip is a warm-up
                    results.append(t - t_flip)
                    print(f"trial {trial}: {1000 * (t - t_flip):.0f} ms")
                break
    if results:
        print(f"\nmedian {1000 * statistics.median(results):.0f} ms, "
              f"min {1000 * min(results):.0f}, max {1000 * max(results):.0f}  ->  camera.latency_s")
    else:
        print("never saw the flash: is the camera pointed at the 'flash' window?")


if __name__ == "__main__":
    main()
