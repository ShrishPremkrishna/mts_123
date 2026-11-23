#!/usr/bin/env python3
"""Compare Ball Eye, maintained-pink-seeded and floor-view pink measurements on identical images.

Three sections, each labelled with what it can and cannot show:

  synthetic    ray-traced balls in saved real fisheye frames (and a plain floor);
               exact labels, so errors are pipeline/geometry errors under an
               assumed lens and mount, never physical accuracy.
  negatives    saved real frames with no ball: false measurements per method.
  sensitivity  frames rendered with the selected profile, measured with another
               radius/mount: the error an unmeasured parameter can cause.
  real         optional saved onboard frames; errors only for rows with
               independently measured labels (labels CSV: file,x_m,y_m).

No camera, robot or service is touched. Usage:

    .venv-fc/bin/python final_project/perception/compare_onboard_ball.py \\
        --model ~/.config/gemini-pupper/ball_eye.onnx --model-sha256 <hex> --out result.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
ROBOT = ROOT / "follow_me_pupper/robot"
sys.path[:0] = [str(ROOT), str(ROBOT)]

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import ball_eye as be  # noqa: E402
from fisheye_converter import equirect_height, fisheye_to_equirectangular, load_camera_model  # noqa: E402
from pink_ball import PinkBallTracker  # noqa: E402
from final_project.perception.synthetic_ball import plain_background, render_ball  # noqa: E402

REAL_BACKGROUNDS = sorted((ROOT / "pupperv3-monorepo/ai/playground/image-description-benchmark/src/"
                           "image_description_benchmark/images").glob("camera_image_raw_compressed-*.jpg"))
NEGATIVES = [*REAL_BACKGROUNDS, *sorted((ROOT / "pupperv3-monorepo/ai/playground/image-description-benchmark/src/"
                                         "image_description_benchmark/images").glob("rfdetr_image_*.jpg")),
             ROOT / "pupperv3-monorepo/ai/playground/undistory/image_01758907493568318505.jpg",
             ROOT / "pupperv3-monorepo/ai/playground/undistory/fisheye.png"]
# Paw contact (combined left/right candidates), near docking, then approach ranges.
POSITIONS = [(.258, .040), (.270, -.046), (.30, 0.), (.30, .10), (.35, -.12), (.40, .05), (.45, -.20),
             (.55, .15), (.60, 0.), (.75, -.25), (.80, .20), (1.0, 0.), (1.0, .30), (1.3, -.20), (1.6, 0.)]
STREAMS = ("eye", "pink", "pinkfloor")
SENSITIVITY = {"radius 0.091 (u6 donor)": dict(radius=.091), "radius 0.06 (simulator)": dict(radius=.06),
               "tilt +3 deg": dict(tilt=3.), "tilt -3 deg": dict(tilt=-3.), "camera z +1 cm": dict(z=.01),
               "camera x +1 cm": dict(x=.01), "yaw +3 deg": dict(yaw=3.)}


def read_rgb(path):
    image = cv2.imread(str(path))
    if image is None:
        raise ValueError(f"cannot read {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


class Runner:
    def __init__(self, profile, backend):
        self.profile, self.eye = profile, be.BallEye(profile, backend)
        self.pano_model = load_camera_model(ROBOT / "camera_params.yaml", 1400, 1050)

    def pink_box(self, rgb):
        """The maintained tracker on the FC panorama; two identical captures, as a still ball gives."""
        h, w = rgb.shape[:2]
        if (w, h) != (1400, 1050):
            return None, "unsupported frame size"
        pano = fisheye_to_equirectangular(rgb, self.pano_model, 700, equirect_height(700))
        tracker = PinkBallTracker()
        tracker.observe(pano, [], 1, "pink")
        row = tracker.observe(pano, [], 100_000_001, "pink")
        return (tuple(row["box"]) if row["state"] == "tracked" else None), row["state"]

    def run(self, rgb):
        box, pink_state = self.pink_box(rgb)
        started = time.perf_counter()
        result = self.eye.measure(rgb, pink_box=box)
        return result, pink_state, (time.perf_counter() - started) * 1e3


def stream_of(method):
    return method.split("_")[0]


def summarize(rows, key):
    """Per stream: measured fraction, fit/box split and error statistics against labels."""
    out = {}
    for stream in STREAMS:
        found = [r for r in rows if r.get(stream)]
        errors = [r[stream]["error_m"] for r in found if r[stream].get("error_m") is not None]
        out[stream] = {"measured": f"{len(found)}/{len(rows)}",
                       "fit": sum(r[stream]["method"].endswith("_fit") for r in found)}
        if errors:
            out[stream].update(median_error_m=round(statistics.median(errors), 4), max_error_m=round(max(errors), 4),
                               p90_error_m=round(sorted(errors)[int(.9 * (len(errors) - 1))], 4))
            range_errors = [r[stream]["range_error_m"] for r in found]
            out[stream]["median_range_error_m"] = round(statistics.median(range_errors), 4)
            within = [r[stream]["error_m"] <= max(.006, 3 * r[stream]["sigma_xy_m"]) for r in found]
            out[stream]["within_max_6mm_or_3sigma"] = f"{sum(within)}/{len(within)}"
    selected = [r for r in rows if r.get("selected")]
    out["selected"] = {"measured": f"{len(selected)}/{len(rows)}",
                       "conflicts": sum(r["state"] == "conflict" for r in rows)}
    sel_errors = [r["selected"]["error_m"] for r in selected if r["selected"].get("error_m") is not None]
    if sel_errors:
        out["selected"].update(median_error_m=round(statistics.median(sel_errors), 4),
                               max_error_m=round(max(sel_errors), 4))
    return {key: out}


def describe(result, label=None):
    row = {"state": result.state, "notes": result.notes}
    for i, m in enumerate(result.candidates):
        item = {"method": m.method, "x": round(m.x, 4), "y": round(m.y, 4), "range_m": round(m.range_m, 4),
                "sigma_xy_m": round(m.sigma_xy_m, 5)}
        if label is not None:
            item.update(error_m=round(math.hypot(m.x - label[0], m.y - label[1]), 5),
                        range_error_m=round(m.range_m - label[2], 5) if len(label) > 2 else None)
        row[stream_of(m.method)] = item
        if i == result.selected:
            row["selected"] = item
    return row


def synthetic(runner, backgrounds, rng):
    rows = []
    for background_name, background in backgrounds:
        for xy in POSITIONS:
            _, rgb, label = render_ball(background, runner.profile.lens, runner.profile.mount, xy,
                                        runner.profile.ball_radius_m, runner.profile.base_height_m, rng=rng)
            result, pink_state, ms = runner.run(rgb)
            rows.append({"background": background_name, "x": xy[0], "y": xy[1], "range_m": round(label.range_m, 4),
                         "pink_tracker": pink_state, "ms": round(ms, 1),
                         **describe(result, (label.x, label.y, label.range_m))})
    return rows


def by_range(rows):
    bins = (("contact <0.3 m", 0, .3), ("near 0.3-0.5 m", .3, .5), ("mid 0.5-0.9 m", .5, .9), ("far >=0.9 m", .9, 9))
    return {name: summarize([r for r in rows if lo <= r["x"] < hi], name)[name] for name, lo, hi in bins}


def sensitivity(profile, backend, background, rng):
    out = {}
    for name, change in SENSITIVITY.items():
        mount = profile.mount
        mount = be.Mount(mount.x + change.get("x", 0), mount.y, mount.z + change.get("z", 0),
                         mount.tilt_up_deg + change.get("tilt", 0), mount.yaw_deg + change.get("yaw", 0))
        wrong = be.MetricProfile(profile.status, change.get("radius", profile.ball_radius_m), "sensitivity", False,
                                 mount, "sensitivity", False, profile.base_height_m, profile.lens, profile.view)
        # The floor gate would hide the error being measured; widen it for this study only.
        runner = Runner(wrong, backend)
        runner.eye.p = be.EyeParams(z_tol=.5)
        errors = []
        for xy in ((.30, 0.), (.40, .05), (.60, 0.), (.80, .20)):
            _, rgb, label = render_ball(background, profile.lens, profile.mount, xy, profile.ball_radius_m,
                                        profile.base_height_m, rng=rng)
            result, _, _ = runner.run(rgb)
            if result.selected is not None:
                m = result.candidates[result.selected]
                errors.append(math.hypot(m.x - label.x, m.y - label.y))
        out[name] = {"measured": len(errors), "median_error_m": round(statistics.median(errors), 4) if errors else None,
                     "max_error_m": round(max(errors), 4) if errors else None}
    return out


def negatives(runner, paths):
    rows = []
    for path in paths:
        if path.is_file():
            result, pink_state, ms = runner.run(read_rgb(path))
            rows.append({"file": path.name, "pink_tracker": pink_state, "ms": round(ms, 1), **describe(result)})
    return {"frames": len(rows), "false_measurements": {s: sum(1 for r in rows if r.get(s)) for s in STREAMS},
            "rows": rows}


def real(runner, directory, labels_path):
    labels = {}
    if labels_path:
        with open(labels_path, newline="") as f:
            for row in csv.DictReader(f):
                labels[row["file"]] = (float(row["x_m"]), float(row["y_m"]))
    rows = []
    for path in sorted(p for p in Path(directory).iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png")):
        result, pink_state, ms = runner.run(read_rgb(path))
        label = labels.get(path.name)
        rows.append({"file": path.name, "labelled": label is not None, "pink_tracker": pink_state, "ms": round(ms, 1),
                     **describe(result, label)})
    labelled = [r for r in rows if r["labelled"]]
    return {"frames": len(rows), "labelled": len(labelled),
            "summary": summarize(labelled, "labelled")["labelled"] if labelled else
            "no independent labels: detections only, no accuracy claim", "rows": rows}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", type=Path, help="Ball Eye .onnx (omit to compare pink streams only)")
    parser.add_argument("--model-sha256", help="required content hash of --model")
    parser.add_argument("--radius", type=float, default=.11,
                        help="selected physical ball radius in metres (standard field profile: 0.11, unmeasured)")
    parser.add_argument("--real-dir", type=Path, help="saved onboard frames to compare")
    parser.add_argument("--labels", type=Path, help="CSV file,x_m,y_m of independently measured ball positions")
    parser.add_argument("--out", type=Path, help="write the full JSON result here")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    if args.model and not args.model_sha256:
        parser.error("--model requires --model-sha256")
    lens = be.Lens.from_camera_params(ROBOT / "camera_params.yaml")
    profile = be.MetricProfile("synthetic", args.radius, "comparison argument", False, be.ESTIMATED_MOUNT,
                               "u6 estimate 2026-11-16 (unmeasured)", False, be.ESTIMATED_BASE_HEIGHT_M, lens)
    backend = be.OnnxBackend(args.model, expected_sha256=args.model_sha256) if args.model else None
    runner = Runner(profile, backend)
    rng = np.random.default_rng(args.seed)
    backgrounds = [("plain floor", plain_background(lens))] + [(p.name, read_rgb(p)) for p in REAL_BACKGROUNDS]
    rows = synthetic(runner, backgrounds, rng)
    result = {
        "kind": "offline Ball Eye comparison; synthetic errors are not physical accuracy",
        "profile": profile.identity(), "model_sha256": backend.sha256 if backend else None,
        "ball_eye_source_sha256": be.RECOVERED_SOURCE_SHA256,
        "backgrounds": [name for name, _ in backgrounds], "positions": POSITIONS,
        "synthetic": {**summarize(rows, "all")["all"], "by_range": by_range(rows),
                      "mac_cpu_ms_median": round(statistics.median(r["ms"] for r in rows), 1)},
        "negatives": negatives(runner, NEGATIVES),
        "sensitivity": sensitivity(profile, backend, backgrounds[1][1] if len(backgrounds) > 1 else backgrounds[0][1],
                                   rng),
        "real": real(runner, args.real_dir, args.labels) if args.real_dir else
        "no saved onboard ball footage: prepared capture task, no real accuracy claim",
        "synthetic_rows": rows,
    }
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=1))
    printable = {k: v for k, v in result.items() if k not in ("synthetic_rows",)}
    printable["negatives"] = {k: v for k, v in result["negatives"].items() if k != "rows"}
    if isinstance(printable["real"], dict):
        printable["real"] = {k: v for k, v in printable["real"].items() if k != "rows"}
    print(json.dumps(printable, indent=1))


if __name__ == "__main__":
    main()
