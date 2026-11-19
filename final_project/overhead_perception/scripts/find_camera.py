#!/usr/bin/env python3
"""Find which camera index is the phone.

Opens each camera index, reports resolution and frame rate, and saves one
frame per camera to out/probe/camN.jpg so you can see which is which.

The first time, macOS asks whether this app may use the camera. Click Allow;
this script keeps retrying for --wait seconds so you have time to answer.
"""
import argparse
import os
import sys
import time
from contextlib import contextmanager
from pathlib import Path

import cv2

OUT = Path(__file__).resolve().parent.parent / "out" / "probe"


@contextmanager
def quiet():
    """OpenCV prints a native warning on every refused open; hide it while we retry."""
    sys.stderr.flush()
    saved, null = os.dup(2), os.open(os.devnull, os.O_WRONLY)
    os.dup2(null, 2)
    try:
        yield
    finally:
        os.dup2(saved, 2)
        os.close(null)
        os.close(saved)


def open_camera(index: int):
    with quiet():
        cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY)
    return cap if cap.isOpened() else None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--max-index", type=int, default=3)
    ap.add_argument("--wait", type=float, default=60.0, help="seconds to wait for camera permission")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    # Wait until at least one camera opens (i.e. permission has been granted).
    deadline, cap = time.time() + args.wait, None
    while cap is None:
        cap = open_camera(0)
        if cap is None:
            if time.time() > deadline:
                print("No camera could be opened. If macOS showed no prompt, allow this app under\n"
                      "System Settings > Privacy & Security > Camera, then run this again.")
                return 1
            print("waiting for camera permission... (click Allow on the macOS prompt)")
            time.sleep(3)
    cap.release()

    found = 0
    for i in range(args.max_index + 1):
        cap = open_camera(i)
        if cap is None:
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        frame, t0 = None, time.time()
        while time.time() - t0 < 8:  # phone cameras take a moment to wake up
            ok, f = cap.read()
            if ok and f is not None and f.mean() > 1:
                frame = f
                break
            time.sleep(0.05)
        if frame is None:
            print(f"index {i}: opened but gave no frames")
            cap.release()
            continue
        n, t1 = 0, time.time()
        while time.time() - t1 < 2:
            n += bool(cap.read()[0])
        path = OUT / f"cam{i}.jpg"
        cv2.imwrite(str(path), cv2.resize(frame, None, fx=0.4, fy=0.4))
        print(f"index {i}: {frame.shape[1]}x{frame.shape[0]} at ~{n / 2:.0f} fps -> {path}")
        found += 1
        cap.release()
    print(f"\n{found} camera(s). Use the phone's index as:  run_perception.py --source <index>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
