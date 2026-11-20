"""Where a Pupper should go, given where the ball is going.

Input: the robot's pose, the ball's predicted Trajectory (ballistics.py) and a
goal. Output: a target pose, the route to it and the timing, chosen as the
fastest among a small set of candidate routes and meeting times.

Two plans:

  plan_strike   get behind the ball on the line to the goal being attacked, at the
                place the ball WILL be rather than where it is now. A walking Pupper
                is slower than a rolling ball, so this is nearly always "be lined up
                where the ball comes to rest"; if the robot can be in place while the
                ball is still crawling (max_contact_speed), it meets it there.
  plan_block    when the ball is predicted to roll into this robot's own goal: the
                point on the ball's path the robot can reach with the most time to
                spare, or how late it would be.

The robot is modelled the way the team describes the real one: it walks forward,
turns, shuffles sideways slowly and does not walk backwards. Travel time is the
quicker of
  - turn to face the next point, walk, repeat, then turn to the final heading, and
  - for short moves that are not behind it, shuffle there while turning.
Speeds are config values (the `intercept:` section); nothing here has been
measured on a robot, and turning on the spot before every leg is pessimistic for
a robot that can curve while it walks.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .ballistics import Trajectory
from .filters import wrap


@dataclass
class RobotModel:
    walk_speed: float = 0.22       # m/s forward
    strafe_speed: float = 0.18     # m/s sideways
    turn_rate: float = 0.65        # rad/s
    can_reverse: bool = False
    react_s: float = 0.3           # s before the robot is actually moving (camera delay + gait start)
    clearance: float = 0.35        # m, keep the body centre this far from the ball when walking round it
    shuffle_max: float = 0.6       # m, shuffle sideways only for moves shorter than this
    standoff: float = 0.60         # m, strike pose: this far behind the ball on the line to the goal
    aim_depth: float = 0.40        # m, aim through the goal centre at a point this far behind the goal line
    max_contact_speed: float = 0.10  # m/s, only strike a ball moving slower than this
    setup_s: float = 0.0           # s the robot must be in place before contact (a kick's wind-up)
    block_margin_s: float = 0.3    # s to spare required before a block counts as reachable
    body_radius: float = 0.20      # m, a block point is kept at least this far in front of the goal line
    block_cover: float = 0.10      # m, the body blocks a ball passing this close to its centre (half its width)

    @classmethod
    def from_config(cls, cfg: dict) -> "RobotModel":
        """Settings from the optional `intercept:` section of the config; anything missing keeps its default."""
        m = cls()
        for key, value in (cfg.get("intercept") or {}).items():
            if not hasattr(m, key):
                raise ValueError(f"config intercept: unknown setting {key!r}")
            setattr(m, key, bool(value) if key == "can_reverse" else float(value))
        return m


def _route_time(points, yaw0: float, yaw_final: float | None, m: RobotModel) -> float:
    """Turn, walk, turn along a list of points, starting with heading yaw0."""
    t, heading = 0.0, yaw0
    for a, b in zip(points, points[1:]):
        leg = math.hypot(b[0] - a[0], b[1] - a[1])
        if leg < 0.03:
            continue
        direction = math.atan2(b[1] - a[1], b[0] - a[0])
        t += abs(wrap(direction - heading)) / m.turn_rate + leg / m.walk_speed
        heading = direction
    if yaw_final is not None:
        t += abs(wrap(yaw_final - heading)) / m.turn_rate
    return t


def travel(robot, target, yaw_final: float | None = None, avoid=None, m: RobotModel = RobotModel()) -> dict:
    """Fastest candidate route from robot (x, y, yaw) to target (x, y) [ending at heading yaw_final].

    avoid: (x, y) of the ball; a walking route that would pass within m.clearance of it
    goes round, on whichever side is quicker. Returns {"time_s", "path": [[x, y], ...], "how"}.
    """
    start = (float(robot[0]), float(robot[1]))
    yaw0 = float(robot[2])
    target = (float(target[0]), float(target[1]))
    routes = [[start, target]]
    if avoid is not None:
        seg = (target[0] - start[0], target[1] - start[1])
        seg_len = math.hypot(*seg)
        if seg_len > 1e-6:
            rel = (avoid[0] - start[0], avoid[1] - start[1])
            along = max(0.0, min(1.0, (rel[0] * seg[0] + rel[1] * seg[1]) / seg_len ** 2))
            miss = math.hypot(rel[0] - along * seg[0], rel[1] - along * seg[1])
            if miss < m.clearance and 0.0 < along < 1.0:
                n = (-seg[1] / seg_len, seg[0] / seg_len)
                routes = [[start, (avoid[0] + side * n[0] * m.clearance * 1.3,
                                   avoid[1] + side * n[1] * m.clearance * 1.3), target] for side in (1.0, -1.0)]
    best = min(routes, key=lambda r: _route_time(r, yaw0, yaw_final, m))
    out = {"time_s": m.react_s + _route_time(best, yaw0, yaw_final, m), "path": [list(p) for p in best], "how": "walk"}

    # Short move, not behind the robot, nothing in the way: shuffle there while turning.
    dx, dy = target[0] - start[0], target[1] - start[1]
    dist = math.hypot(dx, dy)
    ahead = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    left = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    if dist <= m.shuffle_max and len(best) == 2 and (ahead >= -0.02 or m.can_reverse):
        turn = abs(wrap(yaw_final - yaw0)) / m.turn_rate if yaw_final is not None else 0.0
        shuffle = m.react_s + max(abs(ahead) / m.walk_speed, abs(left) / m.strafe_speed, turn)
        if shuffle < out["time_s"]:
            out = {"time_s": shuffle, "path": [list(start), list(target)], "how": "shuffle"}
    return out


def strike_pose(ball_xy, goal: dict, m: RobotModel = RobotModel()) -> tuple[float, float, float]:
    """(x, y, yaw): m.standoff behind the ball on the line through it to the goal, facing along that line."""
    gy = 0.5 * (goal["y_min"] + goal["y_max"])
    ax = goal["x"] + math.copysign(m.aim_depth, goal["x"])     # behind the goal line: steady close to the goal
    d = math.hypot(ax - ball_xy[0], gy - ball_xy[1])
    ux, uy = ((ax - ball_xy[0]) / d, (gy - ball_xy[1]) / d) if d > 1e-6 else (math.copysign(1.0, goal["x"]), 0.0)
    return ball_xy[0] - m.standoff * ux, ball_xy[1] - m.standoff * uy, math.atan2(uy, ux)


def _in_field(xy, length: float, width: float) -> bool:
    return abs(xy[0]) <= length / 2 and abs(xy[1]) <= width / 2


def plan_strike(robot, traj: Trajectory, goal: dict, length: float, width: float,
                m: RobotModel = RobotModel(), step: float = 0.1) -> dict:
    """Where to line up to strike the ball toward `goal`, and when contact can happen.

    {"feasible", "kind": "at_rest" | "moving", "ball": [x, y] at contact, "pose": [x, y, yaw],
     "path": [[x, y], ...], "travel_s", "ball_there_in_s", "contact_in_s", "wait_s", "note"}
    feasible is False when the ball is predicted to leave the field or go in a goal first.
    """
    def candidate(t):
        bx, by, speed = traj.at(t)
        pose = strike_pose((bx, by), goal, m)
        go = travel(robot, pose[:2], pose[2], avoid=(bx, by), m=m)
        return (bx, by), speed, pose, go

    best = None
    if traj.moving:
        # Meet it while it is still crawling: be in place (plus any wind-up) by the time it arrives.
        for t in np.arange(0.0, traj.end_in_s, step):
            (bx, by), speed, pose, go = candidate(float(t))
            if speed <= m.max_contact_speed and go["time_s"] + m.setup_s <= t and _in_field((bx, by), length, width):
                best = ("moving", float(t), float(t), (bx, by), pose, go)
                break
    rests = traj.ends in ("still", "stops")
    if best is None and rests:
        (bx, by), _, pose, go = candidate(traj.end_in_s)
        ready = go["time_s"] + m.setup_s
        best = ("at_rest", traj.end_in_s, max(ready, traj.end_in_s), (bx, by), pose, go)
    if best is None:
        why = {"out": "the ball is rolling out of the field", "horizon": "the ball does not stop within the prediction"}
        return {"feasible": False, "note": why.get(traj.ends, "the ball is rolling into a goal")}
    kind, ball_t, contact_t, ball_xy, pose, go = best
    wait = max(0.0, contact_t - go["time_s"] - m.setup_s)
    note = ("the ball is still: line up behind it" if not traj.moving else
            f"line up where the ball {'will stop' if kind == 'at_rest' else 'will be'} in {ball_t:.1f} s")
    return {"feasible": True, "kind": kind, "ball": [float(ball_xy[0]), float(ball_xy[1])],
            "pose": [float(pose[0]), float(pose[1]), float(pose[2])], "path": go["path"], "how": go["how"],
            "travel_s": float(go["time_s"]), "ball_there_in_s": float(ball_t), "contact_in_s": float(contact_t),
            "wait_s": float(wait), "note": note}


def plan_block(robot, traj: Trajectory, own_goal: dict, m: RobotModel = RobotModel(), step: float = 0.1) -> dict | None:
    """If the ball is predicted to roll into `own_goal`: where to stand in its way.

    None when there is no such threat. Otherwise
    {"feasible", "point": [x, y], "path", "travel_s", "ball_there_in_s", "spare_s", "enters_in_s", "enters_at_y"}
    `point` is where the robot's centre should stand (within block_cover of the ball's path),
    chosen to leave the most time to spare. feasible needs block_margin_s to spare; if no
    point is reachable in time, spare_s is negative and the point is the least-late one.
    """
    side = "goal_east" if own_goal["x"] > 0 else "goal_west"
    if traj.ends != side:
        return None
    best = None
    for t in np.arange(step, traj.end_in_s + 1e-9, step):
        bx, by, _ = traj.at(float(t))
        if abs(own_goal["x"] - bx) < m.body_radius:
            break                                           # do not stand on the goal line itself
        # The body is wide: its centre only has to get within block_cover of the ball's path.
        gap = math.hypot(bx - robot[0], by - robot[1])
        if gap <= m.block_cover:
            stand = (float(robot[0]), float(robot[1]))
            go = {"time_s": 0.0, "path": [list(stand)], "how": "stay"}
        else:
            k = 1.0 - m.block_cover / gap
            stand = (robot[0] + k * (bx - robot[0]), robot[1] + k * (by - robot[1]))
            go = travel(robot, stand, None, m=m)
        spare = float(t) - go["time_s"]
        if best is None or spare > best[0]:
            best = (spare, float(t), stand, go)
    if best is None:
        return {"feasible": False, "point": None, "path": [], "travel_s": None, "ball_there_in_s": None,
                "spare_s": None, "enters_in_s": float(traj.end_in_s), "enters_at_y": float(traj.end[1])}
    spare, t, point, go = best
    return {"feasible": spare >= m.block_margin_s, "point": [float(point[0]), float(point[1])], "path": go["path"],
            "how": go["how"], "travel_s": float(go["time_s"]), "ball_there_in_s": t, "spare_s": spare,
            "enters_in_s": float(traj.end_in_s), "enters_at_y": float(traj.end[1])}


def plan_robot(robot_state: dict, traj: Trajectory, attack_goal: dict, own_goal: dict, length: float, width: float,
               m: RobotModel = RobotModel()) -> dict:
    """Both plans for one robot of the perception state ({"x", "y", "yaw", ...})."""
    robot = (robot_state["x"], robot_state["y"], robot_state["yaw"])
    out = {"strike": plan_strike(robot, traj, attack_goal, length, width, m)}
    block = plan_block(robot, traj, own_goal, m)
    if block is not None:
        out["block"] = block
    return out
