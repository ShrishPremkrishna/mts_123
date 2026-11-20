"""Evaluate team-color proposals on the eight archived overhead views at the laptop's working width."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import cv2

from benchmark_appearance import LABELS
from fieldcam.team_colors import detect_team_colors


def poses(frame, body):
    return {key: {"center": value.center_px, "front": value.front_px}
            for key, value in detect_team_colors(frame, body).items()}


def run(phone_dir, pose_window):
    rows, latency = [], []
    for name, (center, axis, placement) in LABELS.items():
        path = pose_window if name == "pose-window" else phone_dir / (name + ".png")
        image = cv2.imread(str(path))
        if image is None:
            raise FileNotFoundError(path)
        width, top, bottom = (1426, 65, 854) if name == "pose-window" else (1800, 246, 1066)
        scale = 896 / width
        # Preserve all recorded scene colors, including old display annotations.
        panel = cv2.resize(image[top:bottom, :width], (896, round((bottom-top)*scale)), interpolation=cv2.INTER_AREA)
        body = 896 * .075
        raw = poses(panel, body)
        for _ in range(5):
            start = time.perf_counter_ns()
            detect_team_colors(panel, body)
            latency.append((time.perf_counter_ns() - start) / 1e6)
        row = dict(frame=name, placement=placement, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                   working_size=list(panel.shape[1::-1]), recorded_proposals=raw)
        if center is not None:
            x, y = round(center[0]*scale), round((center[1]-top)*scale)
            a = math.radians(axis)
            front = round(x + .30*body*math.cos(a)), round(y + .30*body*math.sin(a))
            controls = {}
            for team, color in (("striker", (225,52,25)), ("keeper",(25,25,220))):
                overlay = panel.copy()
                cv2.circle(overlay, (x,y), round(.15*body), color, -1)
                cv2.circle(overlay, front, round(.06*body), (10,245,155), -1)
                result = poses(overlay, body)
                p = result.get(team)
                controls[team] = bool(p and p["front"] and math.dist(p["center"], (x,y)) <= 2
                                      and math.dist(p["front"], front) <= 2)
            row["digital_patch_controls"] = controls
        rows.append(row)
    signed = sum(p["front"] is not None for r in rows for p in r["recorded_proposals"].values())
    controls = [item for r in rows for item in r.get("digital_patch_controls", {}).values()]
    return dict(schema=1, method="Recorded 896px camera-panel crops, no inpainting or field mask. "
                "No recorded robot wore the proposed patches. Digital patches on six labeled bodies are a separate software control.",
                frames=rows, recorded_signed_poses=signed,
                recorded_team_centers=sum(len(r["recorded_proposals"]) for r in rows),
                digital_controls_passed=sum(controls), digital_controls_total=len(controls),
                detector_ms_median=round(statistics.median(latency),2),
                detector_ms_p95=round(sorted(latency)[math.ceil(len(latency)*.95)-1],2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phone-dir", required=True, type=Path)
    parser.add_argument("--pose-window", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.phone_dir, args.pose_window), indent=2))
