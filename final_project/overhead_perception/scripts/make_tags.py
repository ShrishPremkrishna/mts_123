#!/usr/bin/env python3
"""Generate printable AprilTag (36h11) pages for every tag in the config.

Each PNG is a US-Letter page at 300 dpi; print at 100% / "actual size".
Robot tags: cut out at the corner marks (keeps the white border and the small
FRONT note), mount flat with the FRONT edge toward the robot's nose. Use matte
paper (glossy paper glares under overhead lights and kills detection).
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.config import load_config  # noqa: E402
from fieldcam.detect import TAG_DICT  # noqa: E402

DPI = 300
CUT_BORDER_M = 0.014  # white border left around a robot tag when it is cut out


def page(tag_id: int, size_m: float, label: str, front: str | None = None) -> np.ndarray:
    """One US-Letter page. `front`: robot tags only - a small note that stays on the tag after cutting."""
    img = np.full((int(11 * DPI), int(8.5 * DPI)), 255, np.uint8)
    side = int(round(size_m / 0.0254 * DPI))
    side -= side % 8  # whole pixels per tag cell
    tag = cv2.resize(cv2.aruco.generateImageMarker(TAG_DICT, tag_id, 8), (side, side),
                     interpolation=cv2.INTER_NEAREST)
    y0, x0 = (img.shape[0] - side) // 2, (img.shape[1] - side) // 2
    img[y0:y0 + side, x0:x0 + side] = tag
    # Keep the label clear of the tag's white quiet zone (one cell) and of the printer's top margin.
    cv2.putText(img, f"{label}   id {tag_id}   {size_m * 100:.0f} cm", (x0, max(y0 - side // 4 - 30, 170)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, 0, 4)
    if front:
        # Robot tags get cut out, so the front marker must sit inside the cut:
        # beyond the one-cell white quiet zone, inside corner cut marks at CUT_BORDER.
        cell, b = side // 8, int(round(CUT_BORDER_M / 0.0254 * DPI))
        assert b > cell + 45, "tag too large for a front marker inside the cut border"
        (tw, _), _ = cv2.getTextSize(front, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
        cv2.putText(img, front, (x0 + (side - tw) // 2, y0 - cell - 8), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
        for cx, sx in ((x0 - b, 1), (x0 + side + b, -1)):
            for cy, sy in ((y0 - b, 1), (y0 + side + b, -1)):
                cv2.line(img, (cx, cy), (cx + sx * 60, cy), 0, 2)
                cv2.line(img, (cx, cy), (cx, cy + sy * 60), 0, 2)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="tags")
    ap.add_argument("--field-size", type=float, default=None,
                    help="floor tag side, m (default: config; 0.17 is the largest that keeps a white border on US Letter)")
    ap.add_argument("--robot-size", type=float, default=None, help="robot tag side, m (default: config)")
    args = ap.parse_args()
    cfg = load_config()
    args.field_size = args.field_size or cfg["tags"]["floor_size"]
    args.robot_size = args.robot_size or cfg["tags"]["robot_size"]
    out = Path(args.out)
    out.mkdir(exist_ok=True)
    for tid in cfg["tags"]["field"]:
        cv2.imwrite(str(out / f"field_{tid:02d}.png"), page(tid, args.field_size, "floor tag (any rotation)"))
    for name, rc in cfg["robots"].items():
        cv2.imwrite(str(out / f"robot_{name}_{rc['tag_id']:02d}.png"), page(rc["tag_id"], args.robot_size, f"{name}: cut at the corner marks",
                                 front=f"^ FRONT   {name}   id {rc['tag_id']} ^"))
    print(f"wrote {len(list(out.glob('*.png')))} pages to {out}/")


if __name__ == "__main__":
    main()
