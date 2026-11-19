"""Load config/field.yaml into a plain namespace-ish dict with a few derived values."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
SIM_CONFIG = ROOT / "config" / "sim_field.yaml"  # frozen: simulator and tests


def load_config(path: str | Path | None = None) -> dict:
    path = Path(path) if path else ROOT / "config" / "field.yaml"
    cfg = yaml.safe_load(path.read_text())
    cfg["_path"] = str(path)

    f = cfg["field"]
    half_l, half_w, half_g = f["length"] / 2, f["width"] / 2, f["goal_width"] / 2
    # Goal = segment on a goal line; "east" is at +x.
    cfg["goals"] = {
        "east": {"x": half_l, "y_min": -half_g, "y_max": half_g},
        "west": {"x": -half_l, "y_min": -half_g, "y_max": half_g},
    }
    cfg["field"]["corners"] = [
        [-half_l, -half_w], [half_l, -half_w], [half_l, half_w], [-half_l, half_w]
    ]
    cfg["tags"].setdefault("floor_size", 0.17)
    cfg["tags"].setdefault("robot_size", 0.08)
    cfg["tags"]["field"] = {int(k): np.array(v, float) for k, v in cfg["tags"]["field"].items()}
    # Points to click when calibrating without floor tags. Default: the field corners.
    names = ["west goal line, -y end", "east goal line, -y end", "east goal line, +y end", "west goal line, +y end"]
    manual = cfg.get("manual_points") or [
        {"name": f"field corner: {n}", "xy": c} for n, c in zip(names, cfg["field"]["corners"])
    ]
    cfg["manual_points"] = [{"name": str(m["name"]), "xy": [float(v) for v in m["xy"]]} for m in manual]
    for r in cfg["robots"].values():
        r["yaw_offset"] = math.radians(r.get("yaw_offset_deg", 0.0))
        r["tag_offset"] = np.array(r.get("tag_offset", [0.0, 0.0]), float)
    b = cfg["ball"]
    if "colors" not in b:  # older configs: one orange range
        b["colors"] = [{"lo": b["hsv_lo"], "hi": b["hsv_hi"]}]
    cfg["shot"]["yaw_tol"] = math.radians(cfg["shot"]["yaw_tol_deg"])
    return cfg


def resolve(path: str) -> Path:
    """Config-relative paths are relative to the project root."""
    p = Path(path)
    return p if p.is_absolute() else ROOT / p
