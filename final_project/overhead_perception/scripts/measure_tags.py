#!/usr/bin/env python3
"""Quick try with no setup: measure the distances between tags lying on the floor.

Lay any printed tags flat on the same surface, point the camera at them, run
this. One tag is the reference (lowest id unless --ref); every other tag is
reported as x, y and straight-line distance from it, and each pair gets a
distance. Check the numbers against a tape measure, centre to centre.

  python scripts/measure_tags.py                  # live camera from config
  python scripts/measure_tags.py --floor-size 0.15 --robot-size 0.10
  python scripts/measure_tags.py --source sim     # simulated arena, 15 cm tags

Sizes are the side of the BLACK square as printed; measure it with a ruler.
Only things lying on that surface are measured correctly.  q = quit.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.config import SIM_CONFIG, load_config  # noqa: E402
from fieldcam.detect import TagDetector  # noqa: E402
from fieldcam.tagmeasure import pair_distances, survey_tags  # noqa: E402

WINDOW = "tag distances"


def centre_px(c: np.ndarray):
    return tuple(int(v) for v in c.mean(axis=0))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default=None, help="camera index, video path, or 'sim'")
    ap.add_argument("--floor-size", type=float, default=None, help="black square of floor tags, m (default: config)")
    ap.add_argument("--robot-size", type=float, default=None, help="black square of robot tags, m (default: config)")
    ap.add_argument("--size", type=float, default=None, help="use this size for every tag")
    ap.add_argument("--ref", type=int, default=None, help="reference tag id (default: lowest visible)")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = run forever)")
    ap.add_argument("--snapshot", default=None, help="write the last annotated frame to this image file")
    ap.add_argument("--display-width", type=int, default=1280)
    args = ap.parse_args()

    cfg = load_config(SIM_CONFIG if args.source == "sim" else None)
    cam = cfg["camera"]
    args.floor_size = args.floor_size or cfg["tags"]["floor_size"]
    args.robot_size = args.robot_size or cfg["tags"]["robot_size"]
    source = args.source if args.source is not None else cam["source"]
    source = int(source) if str(source).isdigit() else source
    if source == "sim":
        from fieldcam.camera_model import default_intrinsics
        from fieldcam.sim import SimArena
        K, _ = default_intrinsics(cam["width"], cam["height"], cam["hfov_deg"])
        arena = SimArena(cfg, K, size=(cam["width"], cam["height"]))
        read = lambda _n: (arena.render({}, None), 0.0, _n + 1)  # noqa: E731
        width = cam["width"]
        if args.size is None:
            args.size = 0.15  # the simulator draws 15 cm floor tags
    else:
        from fieldcam.capture import FrameGrabber
        grab = FrameGrabber(source, cam["width"], cam["height"], cam["fps"])
        read, width = grab.read, grab.width

    robot_ids = {rc["tag_id"] for rc in cfg["robots"].values()}
    size_of = lambda i: args.size or (args.robot_size if i in robot_ids else args.floor_size)  # noqa: E731
    print(f"assuming black squares of {(args.size or args.floor_size) * 100:.1f} cm (floor tags) and "
          f"{(args.size or args.robot_size) * 100:.1f} cm (robot tags {sorted(robot_ids)}). "
          "If a ruler disagrees, pass --floor-size / --robot-size.")

    det = TagDetector()
    scale = min(1.0, args.display_width / width)
    smooth: dict = {}  # (a, b) -> smoothed distance, so the readout does not flicker
    n, count, t_print, vis = -1, 0, 0.0, None
    while True:
        frame, _, n = read(n)
        if frame is None:
            continue
        count += 1
        tags = det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        ref = args.ref if args.ref is not None else (min(tags) if tags else None)
        poses, _H = survey_tags(tags, {i: size_of(i) for i in tags}, ref) if tags else (None, None)

        vis = frame.copy()
        lines = []
        if not tags:
            lines.append("no tags in view")
        elif poses is None:
            lines.append(f"reference tag {ref} not in view (seeing {sorted(tags)})")
        else:
            dist = pair_distances(poses)
            for key in list(smooth):
                if key not in dist:
                    del smooth[key]
            for key, d in dist.items():
                smooth[key] = d if key not in smooth else 0.8 * smooth[key] + 0.2 * d
            for tid, c in tags.items():
                cv2.polylines(vis, [c.astype(np.int32)], True, (0, 255, 0) if tid == ref else (255, 200, 0), 3)
                cv2.putText(vis, f"id {tid}", tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 3)
            for (a, b), d in smooth.items():
                pa, pb = centre_px(tags[a]), centre_px(tags[b])
                cv2.line(vis, pa, pb, (0, 255, 255), 2)
                mid = ((pa[0] + pb[0]) // 2, (pa[1] + pb[1]) // 2)
                cv2.putText(vis, f"{d * 100:.1f} cm", mid, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 7)
                cv2.putText(vis, f"{d * 100:.1f} cm", mid, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            lines.append(f"reference: tag {ref} (green)")
            for tid in sorted(poses):
                if tid != ref:
                    p = poses[tid]
                    lines.append(f"tag {tid}: x {p['x'] * 100:+.1f} cm, y {p['y'] * 100:+.1f} cm, "
                                 f"{smooth[tuple(sorted((ref, tid)))] * 100:.1f} cm from tag {ref}")
            if len(poses) == 1:
                lines.append("only one tag in view: add another to measure a distance")
        for k, text in enumerate(lines):
            cv2.putText(vis, text, (20, 45 + 42 * k), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 6)
            cv2.putText(vis, text, (20, 45 + 42 * k), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        if time.time() - t_print > 1.0:  # once a second in the terminal too
            t_print = time.time()
            pairs = "  ".join(f"{a}-{b}: {d * 100:.1f} cm" for (a, b), d in smooth.items()) if poses else ""
            print(f"tags {sorted(tags)}  {pairs or lines[0]}", flush=True)

        if not args.no_gui:
            cv2.imshow(WINDOW, cv2.resize(vis, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                       if scale != 1.0 else vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if args.frames and count >= args.frames:
            break

    if args.snapshot and vis is not None:
        cv2.imwrite(args.snapshot, vis)
        print("wrote", args.snapshot)


if __name__ == "__main__":
    main()
