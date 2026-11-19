"""Shooting geometry and the walk-to-the-shot controller.

Pure functions of the field state, so they can be unit-tested and reused by
the gameplay layer or run on the robot instead of the laptop.
"""
from __future__ import annotations

import math

import numpy as np

from .filters import wrap


def pick_target(ball_xy, goal: dict, post_margin: float, keeper_xy=None) -> np.ndarray:
    """Point on the goal line to aim at: centre, or the bigger gap beside the keeper."""
    lo, hi = goal["y_min"] + post_margin, goal["y_max"] - post_margin
    y = 0.5 * (lo + hi)
    if keeper_xy is not None and abs(keeper_xy[0] - goal["x"]) < 0.6:
        ky = float(np.clip(keeper_xy[1], lo, hi))
        y = 0.5 * (lo + ky) if (ky - lo) > (hi - ky) else 0.5 * (ky + hi)
    return np.array([goal["x"], y])


def shot_geometry(ball_xy, target_xy, standoff: float) -> dict:
    """Staging pose: `standoff` behind the ball on the ball->target line, facing the target."""
    ball_xy, target_xy = np.asarray(ball_xy, float), np.asarray(target_xy, float)
    d = target_xy - ball_xy
    dist = float(np.linalg.norm(d))
    u = d / dist if dist > 1e-6 else np.array([1.0, 0.0])
    staging = ball_xy - u * standoff
    return {
        "target": target_xy.tolist(),
        "aim_yaw": math.atan2(u[1], u[0]),
        "staging": [float(staging[0]), float(staging[1]), math.atan2(u[1], u[0])],
        "ball_to_target": dist,
    }


def shot_errors(robot: dict, shot: dict) -> dict:
    """How far the robot is from the staging pose, expressed in the staging frame."""
    sx, sy, syaw = shot["staging"]
    dx, dy = robot["x"] - sx, robot["y"] - sy
    c, s = math.cos(syaw), math.sin(syaw)
    return {
        "along": c * dx + s * dy,      # + = too close to the ball
        "lateral": -s * dx + c * dy,   # + = left of the aim line
        "dist": math.hypot(dx, dy),
        "yaw": wrap(robot["yaw"] - syaw),
    }


def shot_ready(robot: dict, ball: dict, shot: dict, cfg_shot: dict) -> bool:
    """Gate for triggering the strike policy. Deliberately conservative."""
    if not (robot["visible"] and ball["visible"]):
        return False
    if max(robot["age_s"], ball["age_s"]) > cfg_shot["max_state_age_s"]:
        return False
    if math.hypot(ball["vx"], ball["vy"]) > cfg_shot["ball_still_speed"]:
        return False
    e = shot_errors(robot, shot)
    return e["dist"] < cfg_shot["pos_tol"] and abs(e["yaw"]) < cfg_shot["yaw_tol"]


def approach_cmd(robot: dict, ball: dict, shot: dict, cfg_app: dict) -> dict:
    """Body-frame velocity command (vx fwd, vy left, wz CCW) that walks to the staging pose.

    If the straight path would clip the ball, go via a waypoint beside it first.
    Far away: turn to face where we're going and walk forward (fastest gait).
    Close: hold the shot heading and strafe into place.
    """
    p = np.array([robot["x"], robot["y"]])
    b = np.array([ball["x"], ball["y"]])
    goal = np.array(shot["staging"][:2])
    clear = cfg_app["ball_clearance"]

    waypoint = goal
    seg = goal - p
    seg_len = float(np.linalg.norm(seg))
    if seg_len > 1e-6:
        t = float(np.clip((b - p) @ seg / seg_len ** 2, 0.0, 1.0))
        closest = p + t * seg
        if np.linalg.norm(b - closest) < clear and 0.0 < t < 1.0:
            # Step sideways around the ball, on whichever side we're already on.
            n = np.array([-seg[1], seg[0]]) / seg_len
            side = 1.0 if (p - b) @ n >= 0 else -1.0
            if abs((p - b) @ n) < 1e-3:
                side = 1.0
            waypoint = b + side * n * clear * 1.3

    err_world = waypoint - p
    dist = float(np.linalg.norm(err_world))
    final = waypoint is goal
    if final and dist < cfg_app["turn_first_dist"]:
        yaw_ref = shot["staging"][2]
    else:
        yaw_ref = math.atan2(err_world[1], err_world[0])
    yaw_err = wrap(yaw_ref - robot["yaw"])

    c, s = math.cos(robot["yaw"]), math.sin(robot["yaw"])
    ex, ey = c * err_world[0] + s * err_world[1], -s * err_world[0] + c * err_world[1]
    slow = 1.0 if abs(yaw_err) < 0.6 else 0.3   # don't run off sideways while still turning
    vx = float(np.clip(cfg_app["kp_pos"] * ex * slow, -cfg_app["max_vx"], cfg_app["max_vx"]))
    vy = float(np.clip(cfg_app["kp_pos"] * ey * slow, -cfg_app["max_vy"], cfg_app["max_vy"]))
    wz = float(np.clip(cfg_app["kp_yaw"] * yaw_err, -cfg_app["max_wz"], cfg_app["max_wz"]))
    return {"vx": vx, "vy": vy, "wz": wz, "via_waypoint": not final,
            "waypoint": [float(waypoint[0]), float(waypoint[1])]}
