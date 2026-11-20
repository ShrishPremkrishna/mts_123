#!/usr/bin/env python3
"""Lens calibration from a chessboard. Do once per camera + lens + resolution.

Show a printed (or on-screen) chessboard to the camera from ~15 different
angles and distances, covering the image corners too. SPACE = capture,
ENTER = solve and save, q = quit. Aim for an RMS error under ~0.5 px.

  python scripts/calibrate_intrinsics.py --cols 9 --rows 6 --square 0.024
(cols/rows are INNER corners; square is the side of one square in metres.)
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.camera_model import save_intrinsics  # noqa: E402
from fieldcam.capture import FrameGrabber  # noqa: E402
from fieldcam.config import load_config, resolve  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default=None)
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.024)
    args = ap.parse_args()
    cfg = load_config()
    c = cfg["camera"]
    source = args.source if args.source is not None else c["source"]
    source = int(source) if str(source).isdigit() else source
    cap = FrameGrabber(source, c["width"], c["height"], c["fps"])

    pattern = (args.cols, args.rows)
    obj = np.zeros((args.cols * args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_pts, img_pts, n = [], [], -1
    while True:
        frame, _, n = cap.read(n)
        if frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, None, fx=0.5, fy=0.5)
        found, _ = cv2.findChessboardCorners(small, pattern, flags=cv2.CALIB_CB_FAST_CHECK)
        vis = frame.copy()
        cv2.putText(vis, f"{len(img_pts)} views  board {'FOUND' if found else 'not found'}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.imshow("calibrate", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            return
        if key == ord(" ") and found:
            ok, corners = cv2.findChessboardCornersSB(gray, pattern)
            if ok:
                obj_pts.append(obj)
                img_pts.append(corners)
        if key in (10, 13) and len(img_pts) >= 8:
            break
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, (cap.width, cap.height), None, None)
    path = resolve(c["intrinsics"])
    save_intrinsics(path, K, dist, cap.width, cap.height, rms)
    print(f"RMS {rms:.3f} px over {len(img_pts)} views -> {path}")


if __name__ == "__main__":
    main()
