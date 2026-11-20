"""Where things are from the robot's point of view, and the angles for lining up a shot.

Conventions (the same ones as the rest of the project):

- Robot body frame: x forward (out of the nose), y to the robot's LEFT, metres.
  This is the yaw-only body frame the shooting code calls `base_link_yaw`.
- Angles are radians, counter-clockwise positive, wrapped to (-pi, pi].
- `bearing` of a point = atan2(y, x) in the body frame: how far the robot must
  turn to face it. Positive = turn LEFT, negative = turn right, 0 = dead ahead.
- The shot line runs from the ball to the centre of the goal this robot attacks.

Everything is computed from the same field-frame state, so it inherits that
state's timing: `age_s` here is the age of the oldest measurement used.
"""
from __future__ import annotations

import math

from .filters import wrap


def body_point(robot: dict, xy) -> dict:
    """A field point seen from the robot: {x ahead, y left, distance, bearing}."""
    dx, dy = xy[0] - robot["x"], xy[1] - robot["y"]
    c, s = math.cos(robot["yaw"]), math.sin(robot["yaw"])
    x, y = c * dx + s * dy, -s * dx + c * dy
    return {"x": x, "y": y, "distance": math.hypot(x, y), "bearing": math.atan2(y, x)}


def relative_geometry(robot: dict, ball: dict | None, goal: dict) -> dict:
    """robot/ball: visible entries of the perception state. goal: {x, y_min, y_max} being attacked.

    Returns
      goal:  the goal centre in the robot's body frame, plus `facing`: the direction a ball
             must travel to go INTO that goal, as a turn from the robot's heading (+ = left)
      ball:  the ball in the robot's body frame                         (only if the ball is seen)
      shot_line:                                                         (only if the ball is seen)
        ball_to_goal_yaw   field direction of the shot line, ball -> goal centre
        turn_to_shot_line  turn that makes the robot face along the shot line (+ = left)
        behind_ball        how far the robot is behind the ball along the shot line, m
                           (+ = ball is between robot and goal; - = robot is on the goal side)
        left_of_line       how far the robot is to the left of the shot line, looking at the goal, m
      age_s: age of the oldest measurement behind these numbers
    """
    goal_xy = (goal["x"], 0.5 * (goal["y_min"] + goal["y_max"]))
    out = {"goal": body_point(robot, goal_xy), "age_s": robot["age_s"]}
    # The goal line runs along y, so "into the goal" is +x for the east goal and -x for the west one.
    out["goal"]["facing"] = wrap((0.0 if goal["x"] >= 0 else math.pi) - robot["yaw"])
    if ball is None:
        return out
    out["ball"] = body_point(robot, (ball["x"], ball["y"]))
    out["age_s"] = max(robot["age_s"], ball["age_s"])
    gx, gy = goal_xy[0] - ball["x"], goal_xy[1] - ball["y"]
    dist = math.hypot(gx, gy)
    if dist > 1e-6:
        ux, uy = gx / dist, gy / dist                          # unit vector along the shot line
        rx, ry = robot["x"] - ball["x"], robot["y"] - ball["y"]  # robot relative to the ball
        line_yaw = math.atan2(uy, ux)
        out["shot_line"] = {
            "ball_to_goal_yaw": line_yaw,
            "turn_to_shot_line": wrap(line_yaw - robot["yaw"]),
            "behind_ball": -(rx * ux + ry * uy),
            "left_of_line": -rx * uy + ry * ux,
        }
    return out
