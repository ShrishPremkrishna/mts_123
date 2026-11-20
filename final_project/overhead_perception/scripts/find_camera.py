#!/usr/bin/env python3
"""List the cameras this Mac can use, by name, and say which one is the phone.

  python scripts/find_camera.py

For each camera it prints its name and today's number, opens it, reports the
picture size and frame rate, and saves one frame to out/probe/camN.jpg so you
can see which is which. It also says whether a phone is plugged in by USB.

You normally do not need the number: `camera.source: phone` in the config (the
default) picks the phone by name every time, even if its number changes. Use a
name fragment (`--source iphone`) or a number (`--source 1`) to override.

The first time, macOS asks whether this app may use the camera. Click Allow;
this script keeps retrying for --wait seconds so you have time to answer.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.capture import HELP, _quiet, find_camera, list_cameras, phone_on_cable  # noqa: E402

OUT = Path(__file__).resolve().parent.parent / "out" / "probe"


def open_camera(index: int):
    with _quiet():
        cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY)
    return cap if cap.isOpened() else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max-index", type=int, default=3, help="how many numbers to try when names are unavailable")
    ap.add_argument("--wait", type=float, default=60.0, help="seconds to wait for camera permission")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    cameras = list_cameras()
    names = {c["index"]: c for c in cameras}
    phone = find_camera("phone", cameras)
    cable = phone_on_cable()
    print("phone on a USB cable: " + {True: "yes", False: "no", None: "cannot tell on this system"}[cable])
    if cameras:
        print("cameras macOS offers:")
        for c in cameras:
            kind = "built in" if c["built_in"] else ("PHONE" if phone and c["index"] == phone["index"] else "external")
            print(f"  number {c['index']}: {c['name']}  [{kind}]")
        if phone is None:
            print("  no phone camera among them. " + HELP)
    indices = [c["index"] for c in cameras] or list(range(args.max_index + 1))

    # Wait until at least one camera opens (i.e. permission has been granted).
    deadline, cap = time.time() + args.wait, None
    while cap is None:
        cap = open_camera(indices[0])
        if cap is None:
            if time.time() > deadline:
                print("No camera could be opened. If macOS showed no prompt, allow this app under\n"
                      "System Settings > Privacy & Security > Camera, then run this again.")
                return 1
            print("waiting for camera permission... (click Allow on the macOS prompt)")
            time.sleep(3)
    cap.release()

    found = 0
    for i in indices:
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
        label = names[i]["name"] if i in names else f"camera {i}"
        if frame is None:
            print(f"number {i} ({label}): opened but gave no picture (a phone lying face down gives a black one)")
            cap.release()
            continue
        n, t1 = 0, time.time()
        while time.time() - t1 < 2:
            n += bool(cap.read()[0])
        path = OUT / f"cam{i}.jpg"
        cv2.imwrite(str(path), cv2.resize(frame, None, fx=0.4, fy=0.4))
        print(f"number {i} ({label}): {frame.shape[1]}x{frame.shape[0]} at ~{n / 2:.0f} fps -> out/probe/{path.name}")
        found += 1
        cap.release()
    print(f"\n{found} camera(s) gave a picture.")
    if phone is not None:
        print(f"The config's default `source: phone` will use: {phone['name']}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
