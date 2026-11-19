#!/usr/bin/env python3
"""Save one raw, full-resolution frame from the camera (for debugging detection offline).

  python scripts/grab_frame.py --out out/live_frame.png
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.capture import FrameGrabber  # noqa: E402
from fieldcam.config import load_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", default=None)
    ap.add_argument("--out", default="out/live_frame.png")
    ap.add_argument("--settle", type=float, default=1.5, help="seconds to let exposure settle first")
    args = ap.parse_args()
    cam = load_config()["camera"]
    source = args.source if args.source is not None else cam["source"]
    source = int(source) if str(source).isdigit() else source
    grab = FrameGrabber(source, cam["width"], cam["height"], cam["fps"])
    n, frame, t0 = -1, None, time.time()
    while frame is None or time.time() - t0 < args.settle:
        f, _, n = grab.read(n)
        frame = f if f is not None else frame
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(args.out, frame)
    print(f"wrote {args.out} {frame.shape[1]}x{frame.shape[0]}")
    grab.close()


if __name__ == "__main__":
    main()
