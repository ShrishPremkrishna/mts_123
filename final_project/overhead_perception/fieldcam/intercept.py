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
    area_standoff_max: float = 0.5 # m, while the stopping area is uncertain, wait up to this much further back
    # The leg kick (figures from the team's paw approach and kick notes; none measured with the match ball).
    kick_forward: float = 0.258    # m, ball centre ahead of the robot centre at contact
    kick_left_paw: float = 0.040   # m, ball to the LEFT of the body centre line for a left-paw kick
    kick_right_paw: float = -0.046 # m, the same for the right paw (negative = to the right)
    kick_settle_s: float = 0.2     # s standing still at the pose before the wind-up starts
    kick_windup_s: float = 2.3     # s from the start of the wind-up to the paw reaching the ball
    kick_max_ball_speed: float = 0.05  # m/s, the ball must be this slow when the paw arrives
    kick_window: float = 0.04      # m, the wind-up may start once the stopping area is this small
    pose_tol: float = 0.04         # m, this close to the pose counts as being there
    yaw_tol_deg: float = 5.0       # degrees
    field_margin: float = 0.25     # m, a pose further outside the field lines than this is flagged
    switch_s: float = 0.5          # s, a different route or paw must be this much quicker to replace the current one
    guard_depth: float = 0.45      # m, a guarding robot stands this far in front of its goal line
    stale_s: float = 0.35          # s, a robot position older than this is flagged

    @classmethod
    def from_config(cls, cfg: dict) -> "RobotModel":
        """Settings from the optional `intercept:` section of the config; anything missing keeps its default."""
        m = cls()
        for key, value in (cfg.get("intercept") or {}).items():
            if not hasattr(m, key):
                raise ValueError(f"config intercept: unknown setting {key!r}")
            setattr(m, key, bool(value) if key == "can_reverse" else float(value))
        return m


def _side(angle: float) -> str:
    return f"{abs(math.degrees(angle)):.0f} deg {'left' if angle >= 0 else 'right'}"


def _route_legs(points, yaw0: float, yaw_final: float | None, m: RobotModel):
    """Turn, walk, turn along a list of points, starting with heading yaw0: (seconds, legs).

    A leg is {"do": "turn", "deg" (+ = left), "s"} or {"do": "walk", "m", "s", "to": [x, y]}.
    """
    t, heading, legs = 0.0, yaw0, []

    def turn(to):
        nonlocal t, heading
        angle = wrap(to - heading)
        if abs(angle) > math.radians(1.0):
            legs.append({"do": "turn", "deg": math.degrees(angle), "s": abs(angle) / m.turn_rate})
            t += abs(angle) / m.turn_rate
        heading = to

    for a, b in zip(points, points[1:]):
        leg = math.hypot(b[0] - a[0], b[1] - a[1])
        if leg < 0.03:
            continue
        turn(math.atan2(b[1] - a[1], b[0] - a[0]))
        legs.append({"do": "walk", "m": leg, "s": leg / m.walk_speed, "to": [float(b[0]), float(b[1])]})
        t += leg / m.walk_speed
    if yaw_final is not None:
        turn(yaw_final)
    return t, legs


def describe(legs) -> list[str]:
    """The legs of a route as short lines a person can read."""
    out = []
    for leg in legs:
        if leg["do"] == "start":
            out.append(f"get moving ({leg['s']:.1f} s)")
        elif leg["do"] == "turn":
            out.append(f"turn {_side(math.radians(leg['deg']))} ({leg['s']:.1f} s)")
        elif leg["do"] == "walk":
            out.append(f"walk {leg['m']:.2f} m ({leg['s']:.1f} s)")
        elif leg["do"] == "shuffle":
            parts = [f"{abs(leg['ahead']) * 100:.0f} cm forward"] if abs(leg["ahead"]) > 0.02 else []
            if abs(leg["left"]) > 0.02:
                parts.append(f"{abs(leg['left']) * 100:.0f} cm {'left' if leg['left'] > 0 else 'right'}")
            if abs(leg["deg"]) > 1.0:
                parts.append(f"turning {_side(math.radians(leg['deg']))}")
            out.append("shuffle " + (", ".join(parts) or "in place") + f" ({leg['s']:.1f} s)")
        elif leg["do"] == "stay":
            out.append("already there")
        else:
            out.append(f"{leg['do']} ({leg['s']:.1f} s)")
    return out


def travel(robot, target, yaw_final: float | None = None, avoid=None, m: RobotModel = RobotModel(),
           prefer: str | None = None) -> dict:
    """Fastest candidate route from robot (x, y, yaw) to target (x, y) [ending at heading yaw_final].

    avoid: (x, y) of the ball; a walking route that would pass within m.clearance of it goes
    round instead. prefer: the route chosen last time ("direct", "round_left", "round_right",
    "shuffle"); another route replaces it only if it is m.switch_s quicker, so the choice does
    not flip back and forth between frames.

    {"time_s", "path": [[x, y], ...], "how": "walk" | "shuffle", "route", "legs": [...],
     "steps": [text, ...], "considered": [{"route", "time_s"}, ...], "why"}
    """
    start = (float(robot[0]), float(robot[1]))
    yaw0 = float(robot[2])
    target = (float(target[0]), float(target[1]))
    options = {}          # route name -> (seconds, path, legs, how)
    routes = {"direct": [start, target]}
    blocked = False
    if avoid is not None:
        seg = (target[0] - start[0], target[1] - start[1])
        seg_len = math.hypot(*seg)
        if seg_len > 1e-6:
            rel = (avoid[0] - start[0], avoid[1] - start[1])
            along = max(0.0, min(1.0, (rel[0] * seg[0] + rel[1] * seg[1]) / seg_len ** 2))
            miss = math.hypot(rel[0] - along * seg[0], rel[1] - along * seg[1])
            if miss < m.clearance and 0.0 < along < 1.0:
                blocked = True
                n = (-seg[1] / seg_len, seg[0] / seg_len)
                routes = {name: [start, (avoid[0] + side * n[0] * m.clearance * 1.3,
                                         avoid[1] + side * n[1] * m.clearance * 1.3), target]
                          for name, side in (("round_left", 1.0), ("round_right", -1.0))}
    for name, pts in routes.items():
        seconds, legs = _route_legs(pts, yaw0, yaw_final, m)
        options[name] = (m.react_s + seconds, [list(q) for q in pts], [{"do": "start", "s": m.react_s}] + legs, "walk")

    # Short move, not behind the robot, nothing in the way: shuffle there while turning.
    dx, dy = target[0] - start[0], target[1] - start[1]
    dist = math.hypot(dx, dy)
    ahead = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    left = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    if dist <= m.shuffle_max and not blocked and (ahead >= -0.02 or m.can_reverse):
        angle = wrap(yaw_final - yaw0) if yaw_final is not None else 0.0
        seconds = max(abs(ahead) / m.walk_speed, abs(left) / m.strafe_speed, abs(angle) / m.turn_rate)
        options["shuffle"] = (m.react_s + seconds, [list(start), list(target)],
                              [{"do": "start", "s": m.react_s},
                               {"do": "shuffle", "ahead": ahead, "left": left, "deg": math.degrees(angle), "s": seconds}],
                              "shuffle")

    quickest = min(options, key=lambda k: options[k][0])
    chosen, why = quickest, ""
    if prefer in options and prefer != quickest and options[prefer][0] - options[quickest][0] < m.switch_s:
        chosen = prefer
        why = (f"keeping {prefer.replace('_', ' ')}: {quickest.replace('_', ' ')} is only "
               f"{options[prefer][0] - options[quickest][0]:.1f} s quicker")
    elif len(options) > 1:
        other = min((k for k in options if k != chosen), key=lambda k: options[k][0])
        why = (f"{chosen.replace('_', ' ')} is {options[other][0] - options[chosen][0]:.1f} s quicker than "
               f"{other.replace('_', ' ')}")
    if blocked:
        why = ("the ball is in the way: " + why) if why else "the ball is in the way"
    seconds, path, legs, how = options[chosen]
    return {"time_s": seconds, "path": path, "how": how, "route": chosen, "legs": legs, "steps": describe(legs),
            "considered": [{"route": k, "time_s": float(v[0])} for k, v in sorted(options.items(), key=lambda kv: kv[1][0])],
            "why": why}


def _outside(pose, length: float, width: float) -> float:
    """How far (m) a point is outside the field lines; 0 if inside."""
    return max(abs(pose[0]) - length / 2, abs(pose[1]) - width / 2, 0.0)


def _path_clearance(traj: Trajectory, point) -> float | None:
    """Closest the ball's predicted path comes to a point while it is still rolling, m (None if it is not rolling)."""
    if not traj.moving:
        return None
    return min(math.hypot(x - point[0], y - point[1]) for _, x, y in traj.samples(0.1)[:-1] or [[0, *traj.end]])


def _shot_direction(ball_xy, goal: dict, m: RobotModel) -> tuple[float, float]:
    """Unit vector from the ball toward a point m.aim_depth behind the goal centre (steady close to the goal)."""
    gy = 0.5 * (goal["y_min"] + goal["y_max"])
    ax = goal["x"] + math.copysign(m.aim_depth, goal["x"])
    d = math.hypot(ax - ball_xy[0], gy - ball_xy[1])
    return ((ax - ball_xy[0]) / d, (gy - ball_xy[1]) / d) if d > 1e-6 else (math.copysign(1.0, goal["x"]), 0.0)


def strike_pose(ball_xy, goal: dict, m: RobotModel = RobotModel(), extra: float = 0.0) -> tuple[float, float, float]:
    """(x, y, yaw): m.standoff (+ extra) behind the ball on the line through it to the goal, facing along that line."""
    ux, uy = _shot_direction(ball_xy, goal, m)
    back = m.standoff + extra
    return ball_xy[0] - back * ux, ball_xy[1] - back * uy, math.atan2(uy, ux)


def kick_pose(ball_xy, goal: dict, paw: str, m: RobotModel = RobotModel(), extra: float = 0.0) -> tuple[float, float, float]:
    """(x, y, yaw) of the robot centre that puts the ball in front of the chosen paw, the swing aimed at the goal.

    extra > 0 backs the robot off along the shot direction (for waiting while the ball is still rolling).
    """
    ux, uy = _shot_direction(ball_xy, goal, m)
    left = m.kick_left_paw if paw == "left" else m.kick_right_paw
    forward = m.kick_forward + extra
    return (ball_xy[0] - ux * forward + uy * left, ball_xy[1] - uy * forward - ux * left, math.atan2(uy, ux))


def _area_radius(area: dict | None, m: RobotModel) -> float:
    return 0.0 if not area else min(max(area["along"], area["across"]), m.area_standoff_max)


def _in_field(xy, length: float, width: float) -> bool:
    return abs(xy[0]) <= length / 2 and abs(xy[1]) <= width / 2


def _warnings(pose, traj: Trajectory, length: float, width: float, m: RobotModel, what: str) -> list[str]:
    """Things about a waiting pose a person should know: off the field, or in the rolling ball's way."""
    out = []
    outside = _outside(pose, length, width)
    if outside > m.field_margin:
        out.append(f"{what} is {outside * 100:.0f} cm outside the field: no room behind the ball there")
    near = _path_clearance(traj, pose)
    if near is not None and near < m.body_radius + 0.10:
        out.append(f"the ball's path passes {near * 100:.0f} cm from {what}: the robot may be hit while it waits")
    return out


def plan_strike(robot, traj: Trajectory, goal: dict, length: float, width: float,
                m: RobotModel = RobotModel(), step: float = 0.1, area: dict | None = None,
                prefer: str | None = None) -> dict:
    """Where to line up to strike the ball toward `goal`, and when contact can happen.

    {"feasible", "kind": "at_rest" | "moving", "ball": [x, y] at contact, "pose": [x, y, yaw],
     "path": [[x, y], ...], "travel_s", "ball_there_in_s", "contact_in_s", "wait_s", "hold", "note"}
    feasible is False when the ball is predicted to leave the field or go in a goal first.
    While the ball rolls, `area` (the predicted stopping area) pushes the line-up point further
    back by the area's size, so the robot waits clear of wherever the ball may end up, and
    `hold` is True: line up now, run only once the ball has stopped.
    """
    extra = _area_radius(area, m) if traj.moving else 0.0

    def candidate(t):
        bx, by, speed = traj.at(t)
        pose = strike_pose((bx, by), goal, m, extra)
        if extra and _outside(pose, length, width) > m.field_margin:
            pose = strike_pose((bx, by), goal, m, 0.0)      # no room to wait further back: use the plain line-up point
        go = travel(robot, pose[:2], pose[2], avoid=(bx, by), m=m, prefer=prefer)
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
            f"line up where the ball {'will stop' if kind == 'at_rest' else 'will be'} in {ball_t:.1f} s"
            + (f", {extra * 100:.0f} cm further back until that is certain" if extra > 0.05 else "")
            + "; run when it stops")
    steps = go["steps"] + ([f"wait for the ball ({wait:.1f} s)"] if wait > 0.05 else []) + ["run through the ball"]
    return {"feasible": True, "kind": kind, "ball": [float(ball_xy[0]), float(ball_xy[1])],
            "pose": [float(pose[0]), float(pose[1]), float(pose[2])], "path": go["path"], "how": go["how"],
            "route": go["route"], "steps": steps, "why": go["why"], "considered": go["considered"],
            "travel_s": float(go["time_s"]), "ball_there_in_s": float(ball_t), "contact_in_s": float(contact_t),
            "wait_s": float(wait), "hold": bool(traj.moving), "note": note,
            "warnings": _warnings(pose, traj, length, width, m, "the line-up point")}


def plan_kick(robot, traj: Trajectory, goal: dict, length: float, width: float,
              m: RobotModel = RobotModel(), area: dict | None = None,
              prefer: str | None = None, prefer_paw: str | None = None) -> dict:
    """Meet the ball with a LEG KICK: be at the paw pose, wind up, and have the paw arrive as the ball does.

    The kick needs the robot standing still at an exact pose (ball in front of one paw) for
    kick_settle_s, then kick_windup_s of wind-up before the paw reaches the ball. The ball must
    be nearly stopped (kick_max_ball_speed) when the paw arrives. So the timeline is

        walk to the pose -> settle -> wind-up starts ("commit") -> paw contact

    and the wind-up may start while the ball is still rolling, as long as it will have
    arrived by the time the paw does. That needs the stopping point known to within
    kick_window, which is what `area` (the predicted stopping area) says.

    {"feasible", "paw", "ball": [x, y] at contact, "pose": [x, y, yaw], "stage": [x, y, yaw],
     "path", "travel_s", "commit_in_s", "contact_in_s", "ball_ready_in_s", "ball_speed_at_contact",
     "sure": bool, "at_pose": bool, "action": "go_to_pose" | "wait" | "wind_up", "saves_s", "note"}
    stage is where to go now: the pose itself once the stopping point is sure, otherwise the same
    pose backed off by the size of the stopping area. saves_s is how much earlier the paw reaches
    the ball than if the robot only started moving in once the ball had stopped.

    For reading the decision: "timeline" lists the stages with their seconds, "limit" says whether
    the robot or the ball decides when contact can happen, "other_paw" gives the paw not chosen,
    "steps" spells the route out and "warnings" lists anything wrong with the pose. prefer /
    prefer_paw are the route and paw chosen last time: they are kept unless another is clearly quicker.
    """
    if traj.ends not in ("still", "stops"):
        why = {"out": "the ball is rolling out of the field", "horizon": "the ball does not stop within the prediction"}
        return {"feasible": False, "note": why.get(traj.ends, "the ball is rolling into a goal")}
    # When is the ball slow enough for the paw? (Solve speed(t) = kick_max_ball_speed on the last piece.)
    ready = 0.0
    if traj.moving:
        t0, _, _, s0, t1 = traj.pieces[-1]
        ready = min(max(t0 + (s0 - m.kick_max_ball_speed) / traj.decel, 0.0), t1) if traj.decel > 0 else t1
    sure = (not traj.moving) or (area is not None and max(area["along"], area["across"]) <= m.kick_window)
    extra = 0.0 if sure else _area_radius(area, m)

    options = {}
    for paw in ("right", "left"):                           # right first: the paw with robot kicks on record
        contact = max(ready, 0.0)
        for _ in range(3):                                  # the pose depends on where the ball is at contact
            bx, by, speed = traj.at(contact)
            pose = kick_pose((bx, by), goal, paw, m)
            go = travel(robot, pose[:2], pose[2], avoid=(bx, by), m=m, prefer=prefer)
            contact = max(ready, go["time_s"] + m.kick_settle_s + m.kick_windup_s)
        if _in_field((bx, by), length, width):
            options[paw] = (contact, paw, (bx, by), speed, pose, go)
    if not options:
        return {"feasible": False, "note": "the ball ends up outside the field"}
    quickest = min(options, key=lambda k: options[k][0])   # ties go to the right paw (listed first)
    chosen = quickest
    if prefer_paw in options and options[prefer_paw][0] - options[quickest][0] < m.switch_s:
        chosen = prefer_paw                                 # do not swap paws for a fraction of a second
    contact, paw, ball_xy, speed, pose, go = options[chosen]
    other = next((options[k] for k in options if k != chosen), None)
    stage = kick_pose(ball_xy, goal, paw, m, extra)
    off = math.hypot(robot[0] - pose[0], robot[1] - pose[1])
    at_pose = off <= m.pose_tol and abs(wrap(robot[2] - pose[2])) <= math.radians(m.yaw_tol_deg)
    if at_pose:
        # Already standing on the pose: nothing to walk, only the settle and the wind-up are left.
        go = {**go, "time_s": 0.0, "steps": ["already at the pose"], "path": [[float(robot[0]), float(robot[1])]]}
        contact = max(ready, m.kick_settle_s + m.kick_windup_s)
    commit = contact - m.kick_windup_s
    if at_pose and sure and commit <= m.kick_settle_s + 1e-6:
        action, note = "wind_up", f"start the {paw}-paw kick now: the ball will be there when the paw is"
    elif at_pose or (not sure and math.hypot(robot[0] - stage[0], robot[1] - stage[1]) <= m.pose_tol):
        action = "wait"
        note = ("in place: waiting for the ball to slow down" if sure else
                "waiting clear of the ball until its stopping point is certain")
    else:
        action = "go_to_pose"
        note = (f"go to the {paw}-paw kick pose" if sure else
                f"go to {extra * 100:.0f} cm behind the {paw}-paw kick pose until the stopping point is certain")
    # Without prediction the robot would see the ball stop, and only then walk in, settle and wind up.
    stop = traj.end_in_s if traj.moving else 0.0
    robot_ready = go["time_s"] + m.kick_settle_s            # standing settled at the pose
    wait = max(commit - robot_ready, 0.0)
    timeline = [{"what": "walk to the pose", "s": float(go["time_s"])}, {"what": "settle", "s": m.kick_settle_s}]
    if wait > 0.05:
        timeline.append({"what": "wait for the ball", "s": float(wait)})
    timeline.append({"what": "wind-up", "s": m.kick_windup_s})
    if sure:
        unsure_why = ""
    elif area is None:
        unsure_why = "no stopping area yet"
    else:
        unsure_why = (f"stopping area is {max(area['along'], area['across']) * 100:.0f} cm, "
                      f"needs {m.kick_window * 100:.0f} cm")
    return {"feasible": True, "paw": paw, "ball": [float(ball_xy[0]), float(ball_xy[1])],
            "pose": [float(v) for v in pose], "stage": [float(v) for v in stage], "path": go["path"], "how": go["how"],
            "route": go["route"], "steps": go["steps"], "why": go["why"], "considered": go["considered"],
            "travel_s": float(go["time_s"]), "commit_in_s": float(max(commit, 0.0)), "contact_in_s": float(contact),
            "ball_ready_in_s": float(ready), "ball_speed_at_contact": float(speed), "sure": bool(sure),
            "unsure_why": unsure_why, "at_pose": bool(at_pose), "action": action, "timeline": timeline,
            "limit": "ball" if ready > robot_ready + m.kick_windup_s + 1e-6 else "robot",
            "other_paw": None if other is None else {"paw": other[1], "contact_in_s": float(other[0])},
            "saves_s": float(max(stop + go["time_s"] + m.kick_settle_s + m.kick_windup_s - contact, 0.0)), "note": note,
            "warnings": _warnings(stage, traj, length, width, m, "the kick pose" if sure else "the waiting point")}


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
            go = {"time_s": 0.0, "path": [list(stand)], "how": "stay", "steps": ["already in the ball's path"]}
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
            "how": go["how"], "steps": go["steps"], "travel_s": float(go["time_s"]), "ball_there_in_s": t, "spare_s": spare,
            "enters_in_s": float(traj.end_in_s), "enters_at_y": float(traj.end[1])}


def plan_guard(robot, ball_xy, own_goal: dict, m: RobotModel = RobotModel(), prefer: str | None = None) -> dict:
    """Stand between the ball and this robot's own goal: on the line from the goal centre to the ball,
    m.guard_depth in front of the goal line, facing the ball.

    {"pose": [x, y, yaw], "path", "steps", "travel_s", "at_pose"}
    """
    gx, gy = own_goal["x"], 0.5 * (own_goal["y_min"] + own_goal["y_max"])
    dx, dy = ball_xy[0] - gx, ball_xy[1] - gy
    d = math.hypot(dx, dy)
    ux, uy = (dx / d, dy / d) if d > 1e-6 else (-math.copysign(1.0, gx), 0.0)
    depth = min(m.guard_depth, max(d - m.clearance, 0.1))  # never further out than just short of the ball
    pose = (gx + ux * depth, gy + uy * depth, math.atan2(uy, ux))
    go = travel(robot, pose[:2], pose[2], avoid=ball_xy, m=m, prefer=prefer)
    at_pose = (math.hypot(robot[0] - pose[0], robot[1] - pose[1]) <= 2 * m.pose_tol
               and abs(wrap(robot[2] - pose[2])) <= math.radians(2 * m.yaw_tol_deg))
    return {"pose": [float(v) for v in pose], "path": go["path"], "route": go["route"], "steps": go["steps"],
            "travel_s": float(go["time_s"]), "at_pose": bool(at_pose)}


def plan_robot(robot_state: dict, traj: Trajectory, attack_goal: dict, own_goal: dict, length: float, width: float,
               m: RobotModel = RobotModel(), area: dict | None = None, previous: dict | None = None) -> dict:
    """The plans for one robot of the perception state ({"x", "y", "yaw", ...}):
    strike (run through the ball), kick (leg kick), guard (stand between the ball and its own
    goal) and, when its own goal is threatened, block.
    area: the prediction's stopping area, if any. previous: this robot's plans from the frame
    before, so routes and the kicking paw are not swapped for a marginal gain."""
    robot = (robot_state["x"], robot_state["y"], robot_state["yaw"])
    before = previous or {}
    kept = lambda plan, key: (before.get(plan) or {}).get(key)  # noqa: E731
    out = {"strike": plan_strike(robot, traj, attack_goal, length, width, m, area=area, prefer=kept("strike", "route")),
           "kick": plan_kick(robot, traj, attack_goal, length, width, m, area=area,
                             prefer=kept("kick", "route"), prefer_paw=kept("kick", "paw")),
           "guard": plan_guard(robot, traj.at(0.0)[:2], own_goal, m, prefer=kept("guard", "route"))}
    block = plan_block(robot, traj, own_goal, m)
    if block is not None:
        out["block"] = block
    age = robot_state.get("age_s")
    if age is not None and age > m.stale_s:
        out["stale"] = f"robot position is {age:.1f} s old: plans are from where it was"
    return out
