#!/usr/bin/env python3
"""Top-down dashboard: field, goals, floor tags, ball and Puppers, drawn from the live state.

Run it next to scripts/run_perception.py (same machine, or any machine that
receives the UDP stream). It needs no camera: it only listens.

  python scripts/dashboard.py
  python scripts/dashboard.py --port 5005

Keys: z = zoom between the whole field and "fit to what is visible"
      i = more / less detail in the readouts   q = quit
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.config import load_config  # noqa: E402
from fieldcam.net import StateSubscriber  # noqa: E402
from fieldcam.topdown import render  # noqa: E402

WINDOW = "pupper fc field view"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=None, help="UDP port (default: net.port in the config)")
    ap.add_argument("--zoom", choices=["field", "fit"], default="field")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = run until q)")
    ap.add_argument("--snapshot", default=None, help="write the last picture to this image file")
    args = ap.parse_args()

    port = args.port or load_config()["net"]["port"]
    sub = StateSubscriber(port)
    print(f"listening for perception state on UDP port {port} (start scripts/run_perception.py if nothing shows)")
    zoom, img, t0, detail = args.zoom, None, time.time(), True
    while True:
        state = sub.latest()
        img = render(state, zoom=zoom, stale_s=state["rx_age_s"] if state else 0.0, detail=detail)
        if not args.no_gui:
            cv2.imshow(WINDOW, img)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            if key == ord("z"):
                zoom = "fit" if zoom == "field" else "field"
            if key == ord("i"):
                detail = not detail
        else:
            time.sleep(0.03)
        if args.seconds and time.time() - t0 > args.seconds:
            break
    if args.snapshot and img is not None:
        cv2.imwrite(args.snapshot, img)
        print("wrote", args.snapshot)


if __name__ == "__main__":
    main()
