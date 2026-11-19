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

    cfg["_manual_default"] = not cfg.get("manual_points")
    set_field_size(cfg, cfg["field"]["length"], cfg["field"]["width"])
    cfg["tags"].setdefault("floor_size", 0.17)
    # Which floor tag sits on which field corner, for the corner-tag calibration.
    cfg["tags"].setdefault("corner_ids", {"west_south": 0, "east_south": 1, "east_north": 2, "west_north": 3})
    cfg["tags"]["corner_ids"] = {k: int(v) for k, v in cfg["tags"]["corner_ids"].items()}
    cfg["tags"]["corner_inset"] = [float(v) for v in cfg["tags"].get("corner_inset", [0.0, 0.0])]
    cfg["tags"].setdefault("robot_size", 0.08)
    cfg["tags"]["field"] = {int(k): np.array(v, float) for k, v in cfg["tags"]["field"].items()}
    cfg["manual_points"] = [{"name": str(m["name"]), "xy": [float(v) for v in m["xy"]]} for m in cfg["manual_points"]]
    for r in cfg["robots"].values():
        r["yaw_offset"] = math.radians(r.get("yaw_offset_deg", 0.0))
        r["tag_offset"] = np.array(r.get("tag_offset", [0.0, 0.0]), float)
    b = cfg["ball"]
    if "colors" not in b:  # older configs: one orange range
        b["colors"] = [{"lo": b["hsv_lo"], "hi": b["hsv_hi"]}]
    cfg["shot"]["yaw_tol"] = math.radians(cfg["shot"]["yaw_tol_deg"])
    return cfg


def set_field_size(cfg: dict, length: float, width: float) -> None:
    """Set the field's size and everything derived from it (goals, corners, default click points).

    Called on load with the config's numbers, and again by the corner-tag
    calibration, which measures the size from the tags instead.
    """
    f = cfg["field"]
    f["length"], f["width"] = float(length), float(width)
    half_l, half_w = f["length"] / 2, f["width"] / 2
    half_g = min(f["goal_width"], f["width"]) / 2
    # Goal = segment on a goal line; "east" is at +x.
    cfg["goals"] = {
        "east": {"x": half_l, "y_min": -half_g, "y_max": half_g},
        "west": {"x": -half_l, "y_min": -half_g, "y_max": half_g},
    }
    f["corners"] = [[-half_l, -half_w], [half_l, -half_w], [half_l, half_w], [-half_l, half_w]]
    if cfg.get("_manual_default", True):  # points to click when calibrating by hand: the field corners
        names = ["west goal line, -y end", "east goal line, -y end", "east goal line, +y end", "west goal line, +y end"]
        cfg["manual_points"] = [{"name": f"field corner: {n}", "xy": list(c)} for n, c in zip(names, f["corners"])]


def resolve(path: str) -> Path:
    """Config-relative paths are relative to the project root."""
    p = Path(path)
    return p if p.is_absolute() else ROOT / p
