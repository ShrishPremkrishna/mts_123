"""Replay annotated dashboard captures and an archived field log through laptop match vision.

--phone-dir points to phone-field-dtrknxnz, --pose-window to the mounted-phone capture.
Run from the checkout with python -m final_project.overhead_perception.scripts.rehearse_match_camera.
Optional --raw-frame inputs must be unannotated overhead images. No camera or socket is opened.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import shutil
import statistics
from unittest.mock import patch

import cv2
import numpy as np

from final_project.overhead_perception.fieldcam import capture, pipeline
from final_project.overhead_perception.fieldcam.camera_model import default_intrinsics
from final_project.overhead_perception.fieldcam.config import load_config, set_field_size
from final_project.overhead_perception.fieldcam.pipeline import Perception
from final_project.overhead_perception.fieldcam.sim import SimArena
from final_project.pupper_fc import arena_camera
from final_project.pupper_fc.arena_camera import ArenaCamera

PROFILE = Path(arena_camera.__file__).with_name("arena.json")
NAMES = ("preview-window", "adjustment", "three-corners", "recheck", "measured-preview",
         "live-calibrated", "framing-check", "pose-window")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def measure(frame, profile):
    """Use the actual file branch, resize, calibration and match-only publication state."""
    camera = ArenaCamera(profile, "archive.avi")
    result = {}

    class OneFrameFile:
        height, width = frame.shape[:2]
        is_file = True
        def read(self, index):
            if index == -1:
                return frame, 1., 0
            result.update(camera.snapshot())
            return None, None, index

        def file_time_s(self, *_):
            return 1.

        def close(self):
            pass

    with patch.object(capture, "FrameGrabber", return_value=OneFrameFile()):
        camera._run()
    if not result.get("frame"):
        raise RuntimeError(camera.snapshot()["status"])
    assert not result["live"] and not camera.publishers
    return result["frame"], result["visible"]["tags"]


def summarize(state):
    def position(obj):
        if not obj["measured_this_frame"]:
            return None
        return {k: round(obj[k], 4) for k in ("x", "y", "yaw") if k in obj}
    return {"calibrated": state["calibration_valid"], "ball": position(state["ball"]),
            "robots": {name: position(value) for name, value in state["robots"].items()},
            "fit_px": state["calib_err_px"]}


def dashboard_panel(path):
    screen = cv2.imread(str(path))
    if screen is None or (screen.shape[1], screen.shape[0]) not in ((2900, 1568), (2900, 1572), (2300, 1358)):
        raise ValueError("Expected the original Retina camera-dashboard window " + str(path))
    # render_combined allocates 62% of the content width. Preserve the 16:9 camera
    # aspect and the optical centre, excluding the 56-pixel macOS title bar only.
    width = int(screen.shape[1] * .62)
    return screen[56:56+round(width * 9 / 16), :width]


def original_log(path, profile):
    rows, errors, first = Counter(), [], None
    for line in path.open():
        if not line.startswith("{"):
            continue
        state = json.loads(line)
        if "floor_tags" not in state:
            continue
        rows["frames"] += 1
        if not state["calibration_valid"]:
            continue
        rows["calibrated"] += 1
        first = state if first is None else first
        ids = sorted(int(k) for k, v in state["floor_tags"].items() if v["visible"])
        rows["all_six_floor"] += ids == [0, 1, 2, 3, 4, 5]
        b = state["ball"]["measured_this_frame"]
        a = state["robots"]["striker"]["measured_this_frame"]
        k = state["robots"]["keeper"]["measured_this_frame"]
        for label, flag in (("ball", b), ("striker", a), ("same_frame_striker_ball", a and b), ("keeper", k)):
            rows[label] += bool(flag)
        errors.append(state["calib_err_px"])
    tags = first["floor_tags"]
    layout = {i: [v["x"], v["y"]] for i, v in tags.items()}
    dim_error = [profile["field"][k] - first["field"][k] for k in ("length", "width")]
    return {"sha256": digest(path), "counts": dict(rows), "first_calibrated_sequence": first["seq"],
            "field_axis_length_width_delta_m": dim_error,
            "max_floor_center_delta_m": max(math.dist(v, profile["tags"][i]) for i, v in layout.items()),
            "tag_0_to_1_distance_m": round(math.dist(layout["0"], layout["1"]), 4),
            "comparison_to_rough_nine_feet_m": round(math.dist(layout["0"], layout["1"]) - 2.7432, 4),
            "median_fit_px": round(statistics.median(errors), 3),
            "positive_x_corner_pair": sorted(int(i) for i, v in layout.items() if int(i) < 4 and v[0] > 0),
            "positive_y_corner_pair": sorted(int(i) for i, v in layout.items() if int(i) < 4 and v[1] > 0)}


def synthetic_control(profile, archived_cfg):
    # This camera uses the archived playing surface with known, distinguishable poses
    # and opposite headings. It checks names and signs independently of screenshot UI.
    cfg = load_config(archived_cfg)
    field = profile["field"]
    cfg["ball"]["radius"] = field["ball_radius"]
    cfg["ball"]["sim_bgr"] = [168, 60, 220]  # the mounted field uses a pink ball
    set_field_size(cfg, field["length"], field["width"])
    cfg["tags"]["field"] = {int(i): np.array(p) for i, p in profile["tags"].items()}
    k, _ = default_intrinsics(1280, 720, cfg["camera"]["hfov_deg"])
    render = SimArena(cfg, k, cam_pos=(.1,-.6,2.65), size=(1280,720), noise_sigma=1, seed=5)
    poses = {"striker": (-.43, .21, .35), "keeper": (.58,-.23, .35-math.pi)}
    ball = (-.06,-.09)
    state, tags = measure(render.render(poses, ball), profile)
    state = summarize(state)
    errors = {}
    for name, (x, y, yaw) in poses.items():
        result = state["robots"][name]
        if result is not None:
            delta = math.remainder(result["yaw"] - yaw, math.tau)
            errors[name] = {"center_m": round(math.dist((x, y), (result["x"], result["y"])), 4),
                            "signed_yaw_error_deg": round(math.degrees(delta), 2)}
    ball_error = (math.dist(ball, (state["ball"]["x"], state["ball"]["y"])) if state["ball"] else None)
    passed = (state["calibrated"] and len(errors) == len(poses) and ball_error is not None and ball_error < .02
              and all(v["center_m"] < .02 and abs(v["signed_yaw_error_deg"]) < 5 for v in errors.values()))
    return {"passes_2cm_and_5deg": passed, "body_errors": errors,
            "ball_error_m": round(ball_error, 4) if ball_error is not None else None,
            "kind": "synthetic rendered pixels with known body and ball coordinates", "detected_ids": tags,
            "synthetic_ball_bgr": cfg["ball"]["sim_bgr"], "reference_poses_xy_yaw": poses, "reference_ball_xy": ball, "result":state}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--phone-dir", type=Path, required=True)
    ap.add_argument("--pose-window", type=Path, required=True)
    ap.add_argument("--raw-frame", type=Path, action="append", default=[])
    args = ap.parse_args()
    profile = json.loads(PROFILE.read_text())
    cfg_path = args.phone_dir / "field-measured-pi-720.yaml"
    screenshots = []
    for name in NAMES:
        path = args.pose_window if name == "pose-window" else args.phone_dir / (name + ".png")
        image = dashboard_panel(path)
        current, ids = measure(image, profile)
        old = Perception(load_config(cfg_path), 896, 504)
        small = cv2.resize(image, (896, 504), interpolation=cv2.INTER_AREA)
        old.calibrate_from_corners(small)
        original = old.process(small, 1., 1.)
        screenshots.append({"screen":name, "sha256":digest(path), "opencv_ids":ids,
                            "lobby":summarize(current), "archived_corner_path":summarize(original)})
    raw = []
    for path in args.raw_frame:
        frame = cv2.imread(str(path))
        if frame is None:
            raise ValueError("Unreadable raw frame " + str(path))
        state, ids = measure(frame,profile)
        raw.append({"sha256":digest(path),"opencv_ids":ids,"lobby":summarize(state)})
    report = {
        "inputs": "Physical session log contains measurements computed from original camera frames. Dashboard PNGs include colored boxes, generated robot headings, ball outlines and text. Those overlays can obscure AprilTags or be detected as a pink ball.",
        "method": "Each image starts a fresh ArenaCamera on the file branch with a one-frame offline capture. No saved calibration or previous detections. Reconstruction of the older four-corner method uses the same camera panel and 896px width. Archive logs and screenshots are not paired exposures.",
        "software": {p.name: digest(p) for p in (PROFILE, Path(arena_camera.__file__), Path(pipeline.__file__))}, "historical_wire":original_log(args.phone_dir/"live-pi-896.log",profile),
        "raw_physical_frames":raw, "annotated_physical_camera_screens":screenshots,
        "positive_control":synthetic_control(profile,cfg_path),
        "camera_dependencies": {"opencv": cv2.__version__, "apriltag_36h11": hasattr(cv2.aruco, "DICT_APRILTAG_36h11"),
            "avfoundation_in_installed_wheel": cv2.videoio_registry.hasBackend(cv2.CAP_AVFOUNDATION),
            "macos_device_enumerator_available": bool(shutil.which("system_profiler"))},
        "unresolved": "No original overhead images were present in the located field archive. No independently measured nose direction, second tagged robot or goal posts in these records. The printed generic floor layout has tags 0 and 1 on the south, while this measured profile has them on the north. Retain the measured placements. The tag's printed top edge must correspond to the actual robot nose or have a measured yaw offset."
    }
    print(json.dumps(report, indent=2, allow_nan=False))
    if not report["positive_control"]["passes_2cm_and_5deg"]:
        raise SystemExit("Known-pose reference scene failed its position or yaw tolerance")


if __name__ == "__main__":
    main()
