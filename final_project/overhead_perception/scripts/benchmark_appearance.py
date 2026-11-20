"""Replay the November 20 mounted-phone screenshots with body tags and UI overlays removed.

Run with --phone-dir /path/to/phone-field-dtrknxnz --pose-window /path/to/pose-window.png.
Prints only measurements and image hashes. The imported detector receives pixels and scale.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fieldcam.appearance import detect_puppers

# Independent manual body centres and undirected body-axis angles on the full screenshots.
# Multiple capture times at the same placement are intentionally reported as such.
LABELS = {
    "preview-window": ((443, 452), 35, "untagged placement"),
    "adjustment": ((444, 570), 35, "untagged placement"),
    "three-corners": (None, None, "no visible robot"),
    "recheck": ((910, 613), 90, "central placement"),
    "measured-preview": ((911, 613), 90, "central placement"),
    "live-calibrated": ((907, 614), 90, "central placement with person"),
    "pose-window": ((449, 542), 160, "oblique goal-side placement"),
    "framing-check": (None, None, "defocused; position unresolved"),
}
# Whole printed sheets, including their white borders, used only to erase from evaluation inputs.
PAGES = {
    "recheck": [(874, 552), (951, 552), (952, 599), (872, 600)],
    "measured-preview": [(874, 552), (951, 552), (952, 599), (872, 600)],
    "live-calibrated": [(870, 556), (940, 542), (953, 595), (876, 606)],
    "pose-window": [(409, 495), (453, 477), (472, 526), (424, 548)],
}


def prepare(original, name, *, carpet=False, erase_body=False):
    frame = original.copy()
    center = LABELS[name][0]
    if erase_body:
        x, y = center
        frame[y-97:y+97, x-97:x+97] = frame[y-97:y+97, x+123:x+317]
    # Camera panel only. Exclude desktop title, setup banners and rendered dashboard.
    width, top, bottom = (1426, 65, 854) if name == "pose-window" else (1800, 246, 1066)
    frame = frame[top:bottom, :width].copy()
    if name in PAGES:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color = ((hsv[:, :, 1] > 125) & (hsv[:, :, 2] > 125)).astype(np.uint8) * 255
        color = cv2.dilate(color, np.ones((5, 5), np.uint8))
        frame = cv2.inpaint(frame, color, 5, cv2.INPAINT_TELEA)
        card = np.zeros(frame.shape[:2], np.uint8)
        cv2.fillConvexPoly(card, np.int32(PAGES[name]) - np.int32([0, top]), 255)
        card = cv2.dilate(card, np.ones((7, 7), np.uint8))
        if carpet:
            floor = np.roll(frame, 150, axis=1)
            frame[card != 0] = floor[card != 0]
        else:
            frame = cv2.inpaint(frame, card, 8, cv2.INPAINT_TELEA)
    target = None if center is None or erase_body else (center[0], center[1] - top)
    return frame, target, width * .075


def baseline(frame, length):
    """Simple alternative: compact dark object near a patch of pale material."""
    scale = 40 / length
    gray = cv2.cvtColor(cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA),
                        cv2.COLOR_BGR2GRAY)
    local = cv2.blur(gray, (41, 41)).astype(np.float32)
    dark = ((gray < 76) & (gray < local - 19)).astype(np.uint8)
    bright = (gray > 152) & (gray > local + 28)
    joined = cv2.morphologyEx(dark, cv2.MORPH_CLOSE,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    _, _, regions, centers = cv2.connectedComponentsWithStats(joined)
    candidates = []
    for (_, _, w, h, area), (x, y) in zip(regions[1:], centers[1:]):
        if 34 <= area <= 1250 and 8 <= max(w, h) <= 56 and min(w, h) >= 5:
            u, v = round(x), round(y)
            crop = bright[max(0, v-19):v+20, max(0, u-19):u+20]
            if crop.sum() >= 30:
                candidates.append((x/scale, y/scale))
    return candidates


def score(points, target, length):
    errors = [] if target is None else [math.dist(p, target)/length for p in points]
    hit = bool(errors and min(errors) <= .35)
    return {"match": hit, "extra": len(points) - int(hit),
            "nearest_error_body_lengths": round(min(errors), 3) if errors else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phone-dir", required=True, type=Path)
    parser.add_argument("--pose-window", required=True, type=Path)
    args = parser.parse_args()
    rows, times = [], []
    controls = {"erased_robot_proposals": [], "alternate_fill": [], "scale": [],
                "quarter_occlusion": [], "heavy_occlusion": [], "width_896": [],
                "dark_exposure": [], "two_body_composites": []}
    for name, (center, axis, placement) in LABELS.items():
        source = args.pose_window if name == "pose-window" else args.phone_dir / (name + ".png")
        original = cv2.imread(str(source))
        if original is None:
            raise FileNotFoundError(source)
        frame, target, length = prepare(original, name)
        candidates = detect_puppers(frame, length)
        points = [p.center_px for p in candidates]
        row = {"frame": name, "sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "placement": placement,
               "truth": target, "expected_length_px": length, "paired_limbs": score(points, target, length),
               "brightness_baseline": score(baseline(frame, length), target, length)}
        if target is not None and candidates:
            nearest = min(candidates, key=lambda p: math.dist(p.center_px, target))
            row["shape_score"] = round(nearest.score, 3)
            row["axis_error_mod_180_degrees"] = (None if nearest.axis_rad is None else
                round(abs((math.degrees(nearest.axis_rad) - axis + 90) % 180 - 90), 1))
        rows.append(row)
        # Exclude I/O, screenshot cleanup and resizing to the deployed processing resolution.
        ratio = 896 / frame.shape[1]
        small = cv2.resize(frame, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)
        points = [p.center_px for p in detect_puppers(small, length * ratio)]
        truth = tuple(ratio * v for v in target) if target is not None else None
        controls["width_896"].append({"frame": name, **score(points, truth, length * ratio)})
        for _ in range(10):
            start = time.perf_counter()
            detect_puppers(small, length * ratio)
            times.append(1000 * (time.perf_counter() - start))
        if center is None:
            continue
        darker = np.uint8(frame * .65)
        points = [p.center_px for p in detect_puppers(darker, length)]
        controls["dark_exposure"].append({"frame": name, **score(points, target, length)})
        if name == "preview-window":
            # Identical visual appearance. Distinct locations can be counted,
            # but a copy confers no second robot identity or real two-body evidence.
            x, y = target
            radius = round(.70 * length)
            body = frame[y-radius:y+radius, x-radius:x+radius].copy()
            for separation in (2., 1., .4):
                duplicate = frame.copy()
                other = (x + round(separation * length), y)
                u, v = other
                duplicate[v-radius:v+radius, u-radius:u+radius] = body
                points = [c.center_px for c in detect_puppers(duplicate, length)]
                left = {i for i, p in enumerate(points) if math.dist(p, target) <= .35 * length}
                right = {i for i, p in enumerate(points) if math.dist(p, other) <= .35 * length}
                matched = 2 if any(i != j for i in left for j in right) else int(bool(left | right))
                controls["two_body_composites"].append({"separation_body_lengths": separation,
                    "count": len(points), "matched_centers": matched})
        for fraction in (.25, .8):
            occluded = frame.copy()
            x, y = target
            half = round(.5 * length)
            covered = round(2 * half * fraction)
            occluded[y-half:y+half, x-half:x-half+covered] = frame[
                y-half:y+half, x+half+20:x+half+20+covered]
            points = [c.center_px for c in detect_puppers(occluded, length)]
            key = "quarter_occlusion" if fraction == .25 else "heavy_occlusion"
            controls[key].append({"frame": name, **score(points, target, length)})
        erased, _, _ = prepare(original, name, erase_body=True)
        controls["erased_robot_proposals"].append({"frame": name, "count": len(detect_puppers(erased, length))})
        if name in PAGES:
            changed, _, _ = prepare(original, name, carpet=True)
            controls["alternate_fill"].append({"frame": name,
                **score([p.center_px for p in detect_puppers(changed, length)], target, length)})
        for multiplier in (.8, 1.2):
            points = [p.center_px for p in detect_puppers(frame, length * multiplier)]
            controls["scale"].append({"frame": name, "scale": multiplier, **score(points, target, length)})
    positives = [r for r in rows if r["truth"] is not None]
    report = {
        "method": "Local offline comparison. Independent manual center tolerance is 0.35 nominal body lengths.",
        "coverage": "8 stored phone screenshots in one room, 3 physical placements, 1 real robot; not independent sessions.",
        "input": "Full camera panels. Robot paper and colored overlays erased on four tagged images. Earlier two positive images show an untagged Pupper. Resized copies of each screenshot excluded.",
        "limits": "One mount family and colorway. No physical robot pair, ground-truth signed yaw, calibrated score, surveyed body center, physical tracking latency or motor test. No on-field identity inferred.",
        "summary": {method: {"positives_matched": sum(r[method]["match"] for r in positives),
                             "positives": len(positives), "total_extra_proposals": sum(r[method]["extra"] for r in rows),
                             "median_matched_center_error_body_lengths": round(statistics.median(
                                 r[method]["nearest_error_body_lengths"] for r in positives if r[method]["match"]), 3)}
                    for method in ("paired_limbs", "brightness_baseline")},
        "timing": {"computer": "development Mac", "width_px": 896, "frames": len(times),
                   "opencv": cv2.__version__, "median_ms": round(statistics.median(times), 2),
                   "p95_ms": round(float(np.percentile(times, 95)), 2),
                   "scope": "appearance detector only, warm, cropped camera panels"},
        "screenshots": rows, "controls": controls,
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
