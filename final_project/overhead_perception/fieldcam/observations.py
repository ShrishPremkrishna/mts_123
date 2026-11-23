"""Capture estimates from fieldcam rows; no clocks, transport, prediction or authority.

The producer predicts accepted xy estimates linearly for display. Control and
referee consumers recover that estimate with its original capture time. A row
which was not measured in this frame stays missing. Callers decide whether a
malformed row drops one entity or revokes their whole frame.
"""
import math


def finite_number(value, label):
    if type(value) not in (int, float):
        raise ValueError(label + " must be finite")
    try:
        result = float(value)
    except OverflowError:
        raise ValueError(label + " must be finite") from None
    if not math.isfinite(result):
        raise ValueError(label + " must be finite")
    return result


def capture_measurement(row, capture, age, *, heading=False, tracking="tags"):
    """Return one measured capture estimate, None for missing/coasting, or raise.

    This does not map clocks or establish freshness. ``capture`` and ``age``
    belong to the admitted producer frame. There is deliberately no replacement
    receipt time or default coordinate for invalid evidence.
    """
    if not isinstance(row, dict) or row.get("visible") is not True or row.get("measured_this_frame") is not True:
        return None
    capture = finite_number(capture, "capture time")
    age = finite_number(age, "frame age")
    if age < 0:
        raise ValueError("frame age must not be negative")
    if finite_number(row.get("measured_at"), "track measurement") != capture:
        return None
    if abs(finite_number(row.get("age_s"), "track age") - age) > 1e-6:
        raise ValueError("track age disagrees with original measurement")
    result = {}
    for axis, velocity in (("x", "vx"), ("y", "vy")):
        speed = finite_number(row.get(velocity), velocity)
        result[velocity] = speed
        result[axis] = finite_number(finite_number(row.get(axis), axis) - speed * age,
                                     "capture " + axis)
    if heading:
        if "yaw_at_capture" in row:
            yaw = finite_number(row["yaw_at_capture"], "capture yaw")
        else:
            # Frozen pre-upgrade streams have two documented conventions.
            # Unknown modes cannot guess which timestamp their yaw describes.
            if tracking not in ("tags", "colors"):
                raise ValueError("legacy heading convention is unknown")
            yaw = finite_number(row.get("yaw"), "yaw")
            if tracking == "tags":
                yaw -= finite_number(row.get("wz"), "wz") * age
            yaw = finite_number(yaw, "capture yaw")
        result["yaw"] = math.atan2(math.sin(yaw), math.cos(yaw))
    return result
