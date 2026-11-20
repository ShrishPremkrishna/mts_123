"""Offline comparison on original camera-dashboard screenshots. No device or network.

python -m final_project.overhead_perception.scripts.evaluate_field_setup --phone-dir DIR --pose-window PNG
"""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

from final_project.overhead_perception.fieldcam.camera_model import default_intrinsics, solve_camera_pose
from final_project.overhead_perception.fieldcam.config import load_config, set_field_size
from final_project.overhead_perception.fieldcam.corner_anchor import FarCorner, anchor_geometry, corner_fit
from final_project.overhead_perception.fieldcam.detect import TagDetector
from final_project.overhead_perception.fieldcam.pipeline import Perception, _quad_center
from final_project.overhead_perception.scripts.rehearse_match_camera import NAMES, PROFILE, dashboard_panel, measure, synthetic_control


def setup_for_tag_zero(profile, training_panel, K, dist):
    # Independent IDs in one training image provide an orientation proxy.
    # The archive contains no tape-measured orientation for the printed square.
    tags = TagDetector().detect(cv2.cvtColor(training_panel, cv2.COLOR_BGR2GRAY))
    others = sorted(set(tags) & {1, 2, 3, 4, 5})
    cam = solve_camera_pose(K, dist, [_quad_center(tags[i]) for i in others], [profile["tags"][str(i)] for i in others])
    square = cam.pixel_to_plane(tags[0], 0)
    right = (square[1] + square[2] - square[0] - square[3]) / 2
    x, y = profile["tags"]["0"]
    f = profile["field"]
    return dict(length=f["length"], width=f["width"], goal_width=f["goal_width"], tag_id=0, corner="west_north",
                rotation=math.degrees(math.atan2(right[1], right[0])), tag_size=profile["floor_size"],
                inset_x=x+f["length"]/2, inset_y=f["width"]/2-y)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--phone-dir", type=Path, required=True)
    args.add_argument("--pose-window", type=Path, required=True)
    args = args.parse_args()
    profile = json.loads(PROFILE.read_text())
    paths = {name: args.pose_window if name == "pose-window" else args.phone_dir / (name+".png") for name in NAMES}
    panels = {name: dashboard_panel(path) for name, path in paths.items()}
    size = (1280, 720)
    full = {name: cv2.resize(panel, size, interpolation=cv2.INTER_AREA) for name, panel in panels.items()}
    cfg = load_config()
    K, dist = default_intrinsics(*size, cfg["camera"]["hfov_deg"])
    anchor_setup = setup_for_tag_zero(profile, full["recheck"], K, dist)
    anchor = anchor_geometry(anchor_setup)
    cfg["field"]["goal_width"] = profile["field"]["goal_width"]
    cfg["ball"]["radius"] = profile["field"]["ball_radius"]
    set_field_size(cfg, anchor_setup["length"], anchor_setup["width"])
    cfg["tags"]["field"] = {int(k): np.array(v) for k, v in profile["tags"].items()}
    cfg["field_setup"] = {"mode": "layout"}
    pipeline = Perception(cfg, *size)
    records = []
    for name, panel in panels.items():
        path = paths[name]
        raw_shape = cv2.imread(str(path)).shape[:2]
        frame = full[name]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        reduced = cv2.resize(panel, (896, 504), interpolation=cv2.INTER_AREA)
        old = TagDetector().detect(cv2.cvtColor(reduced, cv2.COLOR_BGR2GRAY))
        tags = TagDetector().detect(gray)
        state, ids = measure(panel, profile)
        row = dict(capture=name, sha256=digest(path), screenshot_wh=list(reversed(raw_shape)),
                   crop_xywh=[0, 56, panel.shape[1], panel.shape[0]], baseline_896_ids=sorted(old),
                   native_1280_ids=sorted(tags), shipped_ids=ids, layout_valid=state["calibration_valid"],
                   layout_reference_error_px=state["calib_err_px"], diagnostics=state["setup_metrics"])
        if 0 in tags:
            chosen, reason, metrics = corner_fit(K, dist, tags[0], anchor, cfg["field"]["corners"], size)
            unchecked = solve_camera_pose(K, dist, tags[0], anchor)
            reference = {i: np.asarray(profile["tags"][str(i)]) for i in sorted(tags) if 0 < i <= 5}
            if unchecked is not None and reference:
                recovered = unchecked.pixel_to_plane([_quad_center(tags[i]) for i in reference], 0)
                holdout = {str(i):round(float(np.linalg.norm(v-xy)), 3) for (i,v),xy in zip(reference.items(), recovered)}
            else:
                holdout = {}
            row["one_tag"] = dict(fit_eligible=chosen is not None, reason=reason, metrics=metrics,
                                  unchecked_other_id_errors_m=holdout)
            if 2 in tags:
                # Proxy for a human click on a distant physical floor mark. Tag 2
                # is 0.7 cm from the configured intersection and is withheld below.
                far_px = _quad_center(tags[2])
                field_corners = np.array(cfg["field"]["corners"])
                far_xy = field_corners[1]  # southeast, opposite the northwest anchor
                tracked = FarCorner(gray, far_px)
                improved, why, uncertainty = corner_fit(K, dist, tags[0], anchor, field_corners, size, (far_xy, tracked.locate(gray)))
                remaining = {i: xy for i, xy in reference.items() if i != 2}
                check = improved.pixel_to_plane([_quad_center(tags[i]) for i in remaining], 0) if improved and remaining else []
                row["one_tag_plus_opposite_mark"] = dict(fit_eligible=improved is not None, reason=why, metrics=uncertainty,
                    click_proxy="Tag 2 center only, used as a point and texture patch; no ID/corners used in the fit.",
                    other_id_errors_m={str(i):round(float(np.linalg.norm(xy-observed)),3) for (i,xy),observed in zip(remaining.items(),check)})
        records.append(row)
    pipeline.calibrate(full["recheck"])
    timings = []
    for n in range(60):
        frame = full[("recheck", "measured-preview", "live-calibrated")[n % 3]]
        start = time.perf_counter_ns()
        pipeline.process(frame, 1+n/30, match_only=True)
        timings.append((time.perf_counter_ns()-start)/1e6)
    print(json.dumps(dict(
        method="Eight annotated views from one mounted room. Crop the camera panel using its measured 62% content width below the 56 px window title. Older single-scale 896 px comparison re-decodes the same pixels. The shipped worker receives each panel independently as a one-frame offline file. Repeated placements are correlated.",
        sources={"phone_dir":args.phone_dir.name, "pose_window_archive":"2026-11-20/field-live-loopback/pose-window.png",
                 "profile_sha256":digest(PROFILE), "opencv":cv2.__version__},
        one_tag_assumptions={"training_view":"recheck", "tag_rotation_source":"Tag 0 orientation estimated from a camera fit to IDs 1–5 in the training view. Rotation and offsets have no independent tape survey.",
           "tag_size_source":"Operator's archived 6.25 inch black-square measurement.", "settings":anchor_setup,
           "held_out_targets":"Other individually decoded floor centers compared with the archived metric field profile. The reference map is camera-derived. No independent field accuracy or physical post survey is available."},
        views=records, processing_ms={"samples":60,"p50":round(float(np.median(timings)),2),
                    "p95":round(float(np.percentile(timings,95)),2),"scope":"1280 x 720 match-only pipeline on development Mac, tags and ball; excludes capture, JPEG, network and Pi."},
        known_pose_render=synthetic_control(profile, args.phone_dir / "field-measured-pi-720.yaml"),
        limits="Static dashboard graphics can hide floor and body tags or look like a ball. Different placements, motion blur, lens changes, physical sign and ground truth at both posts need independent field measurements.",
        ), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
