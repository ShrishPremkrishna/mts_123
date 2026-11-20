#!/usr/bin/env python3
"""Prepare a small, repeatable destination trial; never publish or activate motors.

The target is a metric field pose, shared with Executor's ``move`` intent.
Without tracking, the operator supplies the marked starting pose. Translation
is a bounded timed pulse, NOT position feedback; turns reuse movement_trial's
IMU feedback request. Execute the returned prepare requests through the existing
resident movement client, one at a time, with fresh prepare/run tokens and a
passing receipt between steps. Do not feed invented poses to the FC executor.

The plan contains no success claim. Score independently measured endpoints with
``--measurements FILE``; missing measurements stay missing. One invocation makes
one plan, never an unattended sequence of repeated hardware trials.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from final_project.behaviors.tools.movement_trial import (
    HeadingRequest, PulseRequest, MAX_PULSE_S, wrap_heading,
)
from final_project.pupper_fc.integration import number, point
from final_project.pupper_fc.model import Field, Pose


def _inside(pose, field):
    return (abs(pose.x) <= field.length / 2 - field.robot_radius and
            abs(pose.y) <= field.width / 2 - field.robot_radius)


def plan(start: Pose, target: Pose, *, speed=0.2, distance_scale=1.0,
         field=Field(), calibration_note="nominal speed; physical distance uncalibrated"):
    """Return existing movement-client requests plus the future tracked intent.

    distance_scale = measured distance / (commanded speed * pulse duration),
    measured at THIS speed with the same stop/start sequence and floor surface.
    A model of constant speed cannot remove acceleration or slip error.
    """
    start = point(asdict(start), heading=True)
    target = point(asdict(target), heading=True)
    start = Pose(start.x, start.y, wrap_heading(start.yaw))
    target = Pose(target.x, target.y, wrap_heading(target.yaw))
    speed, distance_scale = number(speed, "speed"), number(distance_scale, "distance_scale")
    if not 0 < speed <= .3 or not .25 <= distance_scale <= 2:
        raise ValueError("speed must be in (0, 0.3] m/s and distance_scale in [0.25, 2]")
    if not _inside(start, field) or not _inside(target, field):
        raise ValueError("start and target must leave a robot-radius margin inside the field")
    if not isinstance(calibration_note, str) or not calibration_note.strip():
        raise ValueError("record the distance calibration evidence or explicitly label it nominal")
    dx, dy = target.x - start.x, target.y - start.y
    distance = math.hypot(dx, dy)
    duration = distance / (speed * distance_scale)
    if duration > MAX_PULSE_S:
        raise ValueError("target exceeds one bounded three-second translation; choose a closer target")
    heading = math.atan2(dy, dx) if distance > 1e-9 else start.yaw
    steps = []

    def turn(angle):
        angle = wrap_heading(angle)
        if abs(angle) > 1e-9:
            HeadingRequest.build(angle, relative=True, timeout_s=20)
            steps.append({"command": "prepare", "turn_angle_rad": angle, "timeout_s": 20})

    turn(heading - start.yaw)
    if distance > 1e-9:
        PulseRequest.build("forward", speed, duration, "walk")
        steps.append({"command": "prepare", "axis": "forward", "speed": speed,
                      "duration": duration})
    turn(target.yaw - heading)
    return {
        "schema": "pupper_pose_trial_plan_v1", "mode": "estimated_translation_imu_turns",
        "start": asdict(start), "target": asdict(target), "field": asdict(field),
        "speed_m_s": speed, "distance_scale": distance_scale,
        "calibration_note": calibration_note, "translation_m": distance,
        "steps": steps,
        "tracked_action": {"kind": "move", "target": asdict(target)},
        "preconditions": ["marked starting pose restored for each trial",
                          "supervised clear route and selected ready movement release",
                          "IMU-turn capability qualified before combined trials"],
        "between_steps": "require completed outcome, passed command verification and confirmed zero; abort otherwise",
        "position_feedback": False, "hardware_executed": False,
        "physical_arrival": "unmeasured", "repeat_automatically": False,
    }


def score(target: Pose, measurements: list[dict]):
    """Separate endpoint spread (precision) from target error (accuracy).

    Each trial row records x/y/yaw in the same independently marked field frame,
    plus a unique trial_id. Include failed or incomplete trials with outcome and
    x/y/yaw=null: they count as incomplete, never a zero-error success.
    """
    target = point(asdict(target), heading=True)
    if not isinstance(measurements, list) or not measurements:
        raise ValueError("measurements must be a nonempty list")
    rows, endpoints, ids = [], [], set()
    for row in measurements:
        if not isinstance(row, dict):
            raise ValueError("measurement must be an object")
        trial_id = row.get("trial_id")
        if not isinstance(trial_id, str) or not trial_id or trial_id in ids:
            raise ValueError("measurement trial_id must be unique and nonempty")
        ids.add(trial_id)
        outcome = row.get("outcome")
        if not isinstance(outcome, str) or not outcome:
            raise ValueError("every attempted trial needs an outcome")
        if any(row.get(k) is None for k in ("x", "y", "yaw")):
            rows.append({"trial_id": trial_id, "outcome": outcome, "measured": False})
            continue
        pose = point(row, heading=True)
        endpoints.append(pose)
        rows.append({"trial_id": trial_id, "outcome": outcome, "measured": True,
                     "position_error_m": math.hypot(pose.x-target.x, pose.y-target.y),
                     "heading_error_deg": math.degrees(wrap_heading(pose.yaw-target.yaw))})
    result = {"attempts": len(rows), "measured": len(endpoints),
              "unmeasured": len(rows)-len(endpoints), "trials": rows,
              "accuracy_rms_m": None, "repeatability_rms_m": None,
              "mean_endpoint": None,
              "scope": "Independent endpoint measurements; verify identical marked starts separately."}
    if endpoints:
        x = sum(p.x for p in endpoints)/len(endpoints)
        y = sum(p.y for p in endpoints)/len(endpoints)
        c = sum(math.cos(p.yaw) for p in endpoints)
        s = sum(math.sin(p.yaw) for p in endpoints)
        result.update(mean_endpoint={"x": x, "y": y,
                                     "yaw": math.atan2(s, c) if math.hypot(s, c)>1e-9 else None},
                      accuracy_rms_m=math.sqrt(sum((p.x-target.x)**2+(p.y-target.y)**2 for p in endpoints)/len(endpoints)),
                      repeatability_rms_m=math.sqrt(sum((p.x-x)**2+(p.y-y)**2 for p in endpoints)/len(endpoints)))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"), default=(0, 0, 0))
    parser.add_argument("--target", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"), required=True)
    parser.add_argument("--speed", type=float, default=.2)
    parser.add_argument("--distance-scale", type=float, default=1.)
    parser.add_argument("--calibration-note", default="nominal speed; physical distance uncalibrated")
    parser.add_argument("--measurements", type=Path)
    args = parser.parse_args(argv)
    try:
        target = Pose(*args.target[:2], math.radians(args.target[2]))
        if args.measurements:
            result = score(target, json.loads(args.measurements.read_text()))
        else:
            start = Pose(*args.start[:2], math.radians(args.start[2]))
            result = plan(start, target, speed=args.speed, distance_scale=args.distance_scale,
                          calibration_note=args.calibration_note)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print("pose trial refused: " + str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
