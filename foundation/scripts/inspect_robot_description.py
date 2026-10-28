"""Extract motor-limit evidence from XML without executing xacro or ROS code."""
from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET


def inspect_description(source: str) -> dict:
    result = {"sha256": hashlib.sha256(source.encode()).hexdigest(),
              "driver_joints": {}, "errors": [],
              "scope": "Declared parameters only; not proof that a running driver loaded them."}
    try:
        root = ET.fromstring(source)
    except ET.ParseError as error:
        result["errors"].append(f"Invalid XML: {error}")
        return result
    numeric = ("position_min", "position_max", "velocity_max", "effort_max", "kp_max", "kd_max",
               "homed_position", "post_homing_position", "homing_kp", "homing_kd")
    for joint in root.iter("joint"):
        params = {item.get("name"): item.text for item in joint.findall("param")}
        if "position_min" not in params and "position_max" not in params:
            continue  # URDF mechanical limits are not driver command limits.
        name = joint.get("name")
        entry = {"parameters": {}, "state_interfaces": [item.get("name") for item in joint.findall("state_interface")],
                 "command_interfaces": [item.get("name") for item in joint.findall("command_interface")]}
        for key in numeric:
            if key not in params:
                continue
            try:
                value = float(params[key])
                if not math.isfinite(value):
                    raise ValueError("nonfinite")
                entry["parameters"][key] = value
            except (TypeError, ValueError):
                result["errors"].append(f"{name}: {key} is not a finite literal; xacro is not evaluated")
        limits = entry["parameters"]
        if not ("position_min" in limits and "position_max" in limits and limits["position_min"] < limits["position_max"]):
            result["errors"].append(f"{name}: missing or invalid position interval")
        if name in result["driver_joints"]:
            result["errors"].append(f"{name}: duplicate driver declaration")
        else:
            result["driver_joints"][name] = entry
    return result
