#!/usr/bin/env python3
"""Overhead camera to kick session bridge: walk the Pupper to the ball and kick it into a goal.

The phone on the tripod measures the Pupper (robot tag 10) and the ball in the field frame
(x to the east goal, y left of x, metres). This program sends finite motion jobs to the
Gemini Pupper tool server of the kick session (release u6, port 18090 on the Pupper, reached
through an SSH tunnel) and measures again after each job:

  far from the ball   turn and walk to the stage point: behind the ball on the line goal-ball,
                      around the ball if the Pupper starts on the goal side   (tools turn, move)
  at the stage point  turn to the shot heading
  line-up             the onboard camera (tool look_ball) gives the ball in the body frame. A turn
                      puts the ball on the line of one front paw, one walk puts it at the kick
                      point, then kick_in_place

After each kick the program measures where the ball went, writes one row to <out>/kicks.jsonl,
and corrects the kick point for that goal and paw in <out>/calibration.json. The walker models
(travel per commanded metre, start delay, heading error of the tag) are corrected from each job
in the same file. If the camera does not read the robot tag, the program does some jobs from
the pose that the last job gives. It reads the IMU of the Pupper one time for each kick and ends
the run if the IMU reports a tilt (the IMU of this Pupper drifts).

  python scripts/overhead_kick.py --look            # camera only: print the measurement and the plan
  python scripts/overhead_kick.py --yaw-offset DEG  # walk to the ball and kick one time
  python scripts/overhead_kick.py --kicks 5 --goal east --side left
  python scripts/overhead_kick.py --no-kick         # line up, no kick

--yaw-offset is the angle from the top edge of the robot tag to the nose (-180 on 2026-11-20).
The program ends the run if the first long walk goes sideways or back.

Stop: Ctrl-C, or make the file <out>/STOP. Each of them sends Stop to the Pupper.
macOS gives the camera only to a process that Terminal.app starts.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import NamedTuple

HERE = Path(__file__).resolve().parent
ROBOT = "striker"  # robot tag 10 in config/field.yaml
GOALS = ("east", "west")
SIDES = ("left", "right")
# Walker models. Moves: [travel per commanded metre, start delay s]. Turn: [degrees per commanded
# degree, lost degrees]. Start values: 0.84 from leg odometry at 0.2 m/s (2026-11-19).
WALKER0 = {"forward": [0.84, 0.0], "backward": [0.84, 0.0], "left": [0.70, 0.0], "right": [0.70, 0.0],
           "turn": [1.0, 0.0], "forward_fast": [0.90, 0.0],
           # Heading correction [a, b], degrees: true heading = tag heading + a cos(heading) + b sin(heading).
           # The paper tag on the Pupper is not level, so its heading is wrong by an amount that changes with
           # the facing direction. Forward walks on 2026-11-20 went 9 to 19 degrees to the left of the tag
           # heading when the Pupper faced east and 9 to 15 degrees to the right when it faced west.
           "heading": [10.0, 0.0]}


@dataclass(frozen=True)
class Params:
    # Kick point: the ball centre in the body frame (x ahead, y left) when the kick clip starts.
    # Start values from lab/ball_lineup.py (contact_x - 0.005, AIM_Y). The camera numbers have a
    # constant bias, so the program corrects them for each goal and paw from the kick results.
    kick_x: float = 0.250
    kick_y_left: float = 0.037
    kick_y_right: float = -0.042
    tol_x: float = 0.020            # kick window half-widths, m (kicks at x 0.242 and 0.259 moved the ball 1.0 to 1.6 m)
    tol_y: float = 0.010
    loose: float = 2.0              # after max_pulses, kick inside loose x the window
    max_pulses: int = 8
    standoff: float = 0.15          # stage point: the ball is this far beyond the kick point
    final_x: float = 0.0            # the walk to the ball stops this far short (0: one walk to the kick point)
    box_back: float = 0.08          # line-up box around the kick point, in the body frame
    box_ahead: float = 0.20         # beyond the standoff
    box_side: float = 0.25
    stage_near: float = 0.30        # nearer than this to the stage point: correct the heading, then line up
    side_clear: float = 0.06        # a side step larger than side_small needs the ball this far beyond the kick point
    side_small: float = 0.03
    max_side: float = 0.12
    turn_clear_r: float = 0.30      # no turn in place with the ball centre nearer than this ...
    small_turn_deg: float = 10.0    # ... but a turn to this size is safe to 0.22 m
    small_turn_r: float = 0.22
    turn_tol_deg: float = 8.0       # walk to a point: turn first if its bearing is larger
    max_turn_deg: float = 150.0
    max_step: float = 1.00
    min_step: float = 0.03
    clear: float = 0.30             # the walk path keeps the Pupper centre this far from the ball centre
    ring: float = 0.55              # distance of the go-around points from the ball
    goal_width: float = 0.61
    post_margin: float = 0.10       # aim this far inside each post
    aim_margin_deg: float = 2.0
    yaw_tol_min_deg: float = 6.0    # heading window to start the line-up: half of the half-angle of the goal mouth,
    yaw_tol_max_deg: float = 8.0    # in these limits (a small turn of the walker has an error of 3 to 5 degrees)
    yaw_stay_min_deg: float = 10.0  # after the line-up started: the half-angle less aim_margin, in these limits
    yaw_stay_max_deg: float = 14.0
    min_turn_deg: float = 3.0       # a smaller turn onto the paw line is not commanded
    eye_min_x: float = 0.18         # the onboard camera is read with the ball in this box of the body frame
    eye_max_x: float = 0.60
    eye_max_y: float = 0.30
    eye_max_diff: float = 0.15      # an onboard reading is used if its range agrees with the overhead range to this ...
    eye_max_bearing_deg: float = 30.0  # ... and its bearing to this (the tag heading can be 10 to 20 degrees off)
    blind_jobs: int = 6             # jobs in a row from dead reckoning when the camera does not read the robot tag
    blind_wait_s: float = 1.5       # wait this long for the tag before a job from dead reckoning
    rechoose_m: float = 0.25        # a ball that moved this far gets a new goal selection
    play_margin_x: float = 0.05     # the ball is in play this far inside the goal lines ...
    play_margin_y: float = 0.10     # ... and this far outside the side lines
    walk_margin_x: float = 0.10     # the Pupper centre can go this far outside the goal lines ...
    walk_margin_y: float = 0.30     # ... and this far outside the side lines
    speed: float = 0.2              # m/s command for each move (the walker barely moves below 0.2)
    fast_speed: float = 0.35        # m/s command for a walk to a far point ...
    fast_from: float = 0.30         # ... that is longer than this, m
    learn_min_m: float = 0.06       # a longer move corrects the gain, a shorter one the start delay
    contact_m: float = 0.015        # a forward pulse that moved the ball this far touched it
    contact_gap: float = 0.012      # kick point = ball position after a touch + this
    miss_m: float = 0.05            # a kick that moved the ball less than this missed it
    miss_step: float = 0.015        # a miss moves the kick point this much nearer
    weak_m: float = 0.30
    dir_big_deg: float = 12.0       # a ball that left more than this off the heading corrects kick_y; less corrects the aim
    deg_per_cm: float = 6.0         # ball direction per centimetre of sideways offset (lab/ball_lineup.py)
    dir_learn: float = 0.5
    max_shift: float = 0.07         # largest correction of the kick point from its start value, m


class Job(NamedTuple):
    kind: str        # "turn" (degrees, + is left) | "walk" (m, + is ahead) | "side" (m, + is left) | "kick" (paw)
    value: object
    why: str = ""
    fast: bool = False  # a walk at Params.fast_speed

    def text(self) -> str:
        if self.kind == "kick":
            return f"kick {self.value} ({self.why})"
        unit = " deg" if self.kind == "turn" else " m"
        return f"{self.kind} {self.value:+.{1 if self.kind == 'turn' else 3}f}{unit} ({self.why})"


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def to_body(pose, point) -> tuple:
    """A field point in the body frame of pose (x, y, yaw): (ahead, left)."""
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


def to_field(origin, yaw: float, ahead: float, left: float) -> tuple:
    c, s = math.cos(yaw), math.sin(yaw)
    return origin[0] + c * ahead - s * left, origin[1] + s * ahead + c * left


def seg_dist(a, b, c) -> float:
    """Distance from point c to the segment a-b."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    n = vx * vx + vy * vy
    t = 0.0 if n < 1e-12 else clamp(((c[0] - a[0]) * vx + (c[1] - a[1]) * vy) / n, 0.0, 1.0)
    return math.hypot(a[0] + t * vx - c[0], a[1] + t * vy - c[1])


def true_pose(pose, fix) -> tuple:
    """The pose with the heading correction fix = [a, b] (degrees) of the walker model "heading"."""
    yaw = pose[2]
    return pose[0], pose[1], wrap(yaw + math.radians(fix[0] * math.cos(yaw) + fix[1] * math.sin(yaw)))


def predict(pose, job: Job) -> tuple:
    """The pose after a job that goes as commanded (dead reckoning for a time with no tag reading)."""
    x, y, yaw = pose
    if job.kind == "turn":
        return x, y, wrap(yaw + math.radians(job.value))
    if job.kind == "walk":
        return (*to_field((x, y), yaw, job.value, 0.0), yaw)
    if job.kind == "side":
        return (*to_field((x, y), yaw, 0.0, job.value), yaw)
    return pose


def move_name(job: Job) -> str:
    if job.kind == "turn":
        return "turn"
    if job.kind == "walk":
        return ("forward_fast" if job.fast else "forward") if job.value > 0 else "backward"
    return "left" if job.value > 0 else "right"


def command(job: Job, model, speed: float) -> float:
    """The number for the tool: seconds for a move, degrees for a turn. model = [gain, loss]."""
    if job.kind == "turn":
        return abs(job.value) / model[0] + model[1]
    return model[1] + abs(job.value) / (model[0] * speed)


class Striker:
    """Job selection and learning. Pure logic: poses and points in the field frame, no camera, no robot.

    cal (a dict that the caller saves): {"kick": {"east/left": {"x", "y"}, ...}, "walker": {name: [gain, loss]}}.
    """

    def __init__(self, p: Params, length: float, width: float, cal: dict | None = None, goal: str = "auto",
                 side: str = "left", camera_xy=(0.0, 5.0)):
        self.p, self.half_l, self.half_w = p, length / 2.0, width / 2.0
        self.cal = cal if cal is not None else {}
        self.want_goal, self.want_side, self.camera_xy = goal, side, camera_xy
        self.goal = self.side = self.ball0 = None
        self.lining, self.pulses, self.touched = False, 0, False
        self.shift = (0.0, 0.0)
        self.odd = 0  # number of moves whose measured result did not agree with the command

    # ---- calibration ----
    def model(self, name: str) -> list:
        return self.cal.setdefault("walker", {}).setdefault(name, list(WALKER0[name]))

    def kick_point(self, goal: str | None = None, side: str | None = None) -> dict:
        goal, side = goal or self.goal, side or self.side
        start = {"x": self.p.kick_x, "y": self.p.kick_y_left if side == "left" else self.p.kick_y_right, "dir": 0.0}
        k = self.cal.setdefault("kick", {}).setdefault(f"{goal}/{side}", start)
        k.setdefault("dir", 0.0)  # degrees that the ball leaves off the heading, + is left
        return k

    def aim(self, ball) -> tuple:
        """(heading for the Pupper, half the angle of the goal mouth): the shot heading less the
        direction that the ball leaves off the heading with this paw."""
        psi, half = self.shot(ball, self.goal)
        return wrap(psi - math.radians(self.kick_point()["dir"])), half

    # ---- geometry ----
    def goal_x(self, goal: str) -> float:
        return self.half_l if goal == "east" else -self.half_l

    def shot(self, ball, goal: str) -> tuple:
        """(heading to the middle of the goal mouth, half the angle of the mouth inside the post margin)."""
        gx, h = self.goal_x(goal), max(0.02, self.p.goal_width / 2.0 - self.p.post_margin)
        a, b = math.atan2(-h - ball[1], gx - ball[0]), math.atan2(h - ball[1], gx - ball[0])
        return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b)), abs(wrap(b - a)) / 2.0

    def stage(self, ball, goal: str, side: str) -> tuple:
        """Where the Pupper centre stands, with the shot heading, for the ball at the standoff."""
        k, (psi, _) = self.kick_point(goal, side), self.shot(ball, goal)
        psi = wrap(psi - math.radians(k["dir"]))
        return to_field(ball, psi, -(k["x"] + self.p.standoff), -k["y"])

    def can_walk(self, point) -> bool:
        return (abs(point[0]) <= self.half_l + self.p.walk_margin_x
                and abs(point[1]) <= self.half_w + self.p.walk_margin_y)

    def in_play(self, ball) -> bool:
        return (abs(ball[0]) <= self.half_l - self.p.play_margin_x
                and abs(ball[1]) <= self.half_w + self.p.play_margin_y)

    def route(self, pose, ball, target, psi: float) -> tuple:
        """The next point of the walk to target: the target, or a point around the ball."""
        p = self.p
        clear = min(p.clear, dist(pose, ball) - 0.02)  # a Pupper inside the clear zone must not go nearer
        if seg_dist(pose, target, ball) >= clear:
            return target, "stage point"
        left = to_body((ball[0], ball[1], psi), pose)[1]  # + is left of the shot line, looking at the goal
        if abs(left) >= 0.10:
            first = math.copysign(1.0, left)
        else:  # on the line: go around on the camera side, where the robot tag is larger
            first = math.copysign(1.0, to_body((ball[0], ball[1], psi), self.camera_xy)[1] or 1.0)
        options = []
        for s in (first, -first):
            options += [(to_field(ball, psi, -p.ring, s * p.ring), "behind the ball, to the side"),
                        (to_field(ball, psi, 0.0, s * p.ring), "beside the ball")]
        for point, name in options:
            if self.can_walk(point) and seg_dist(pose, point, ball) >= clear:
                return point, name
        return options[1]

    # ---- selection ----
    def choose(self, pose, ball) -> None:
        """Lock the goal and the paw for this kick: the shortest walk, the shortest shot."""
        best = None
        for goal in ([self.want_goal] if self.want_goal in GOALS else GOALS):
            psi, half = self.shot(ball, goal)
            for side in ([self.want_side] if self.want_side in SIDES else SIDES):
                spot = self.stage(ball, goal, side)
                cost = dist(pose, spot) + 0.6 * dist(ball, (self.goal_x(goal), 0.0))
                cost += 0.6 if seg_dist(pose, spot, ball) < self.p.clear else 0.0      # go around
                cost += 5.0 if not self.can_walk(spot) else 0.0
                cost += 1.5 if math.degrees(half) < self.p.yaw_tol_min_deg else 0.0   # steep angle
                if best is None or cost < best[0]:
                    best = (cost, goal, side)
        _, self.goal, self.side = best
        self.ball0, self.lining, self.pulses, self.touched = tuple(ball), False, 0, False

    def next_job(self, pose, ball, shift=(0.0, 0.0)) -> Job:
        """shift: onboard camera reading less overhead reading of the ball in the body frame. The
        line-up uses the overhead reading plus shift; the walk to the stage point does not."""
        p = self.p
        if self.goal is None or dist(ball, self.ball0) > p.rechoose_m:
            self.choose(pose, ball)
        self.shift = shift
        psi, half = self.aim(ball)
        tol = clamp(math.degrees(half) / 2.0, p.yaw_tol_min_deg, p.yaw_tol_max_deg)
        stay = clamp(math.degrees(half) - p.aim_margin_deg, p.yaw_stay_min_deg, p.yaw_stay_max_deg)
        k = self.kick_point()
        bx, by = to_body(pose, ball)
        bx, by = bx + shift[0], by + shift[1]
        ex, ey = bx - k["x"], by - k["y"]
        turn = math.degrees(wrap(psi - pose[2]))
        spot = self.stage(ball, self.goal, self.side)
        in_box = -p.box_back <= ex <= p.standoff + p.box_ahead and abs(ey) <= p.box_side
        if in_box or dist(pose, spot) <= p.stage_near:
            if abs(turn) <= (stay if self.lining else tol):
                self.lining = True
                return self._line_up(ex, ey, bx, by, turn, stay)
            self.lining = False
            r = math.hypot(bx, by)
            if r < p.turn_clear_r:
                return Job("walk", -math.copysign(p.turn_clear_r + 0.06 - r, bx), "room for a turn")
            return Job("turn", clamp(turn, -p.max_turn_deg, p.max_turn_deg), "to the shot heading")
        self.lining = False
        point, name = self.route(pose, ball, spot, psi)
        tx, ty = to_body(pose, point)
        d = math.hypot(tx, ty)
        bearing = math.degrees(math.atan2(ty, tx))
        if abs(bearing) > p.turn_tol_deg:
            return Job("turn", clamp(bearing, -p.max_turn_deg, p.max_turn_deg), "face the " + name)
        return Job("walk", clamp(d, p.min_step, p.max_step), "to the " + name, d >= p.fast_from)

    def _line_up(self, ex: float, ey: float, bx: float, by: float, turn: float, stay: float) -> Job:
        """One step of the line-up. bx, by: the ball in the body frame. turn: degrees to the shot
        heading. stay: the heading window. A sideways error is corrected with a turn onto the paw line
        if the heading stays in the window (a side step is less accurate and turns the Pupper)."""
        p = self.p
        if self.touched and ex <= p.tol_x and abs(ey) <= p.loose * p.tol_y:
            return Job("kick", self.side, "the ball is at the chest")
        if abs(ex) <= p.tol_x and abs(ey) <= p.tol_y:
            return Job("kick", self.side, "in the window")
        if self.pulses >= p.max_pulses and abs(ex) <= p.loose * p.tol_x and abs(ey) <= p.loose * p.tol_y:
            return Job("kick", self.side, f"loose window after {self.pulses} pulses")
        self.pulses += 1
        if abs(ey) > p.tol_y:
            r = math.hypot(bx, by)
            onto = math.degrees(math.atan2(by, bx) - math.asin(clamp((by - ey) / max(r, 1e-6), -1.0, 1.0)))
            if abs(turn - onto) <= stay and abs(onto) >= p.min_turn_deg:
                if r >= p.turn_clear_r or (abs(onto) <= p.small_turn_deg and r >= p.small_turn_r):
                    return Job("turn", onto, "line-up, onto the paw line")
                return Job("walk", -(p.turn_clear_r + 0.04 - r), "back for a turn")
            if ex < p.side_clear and abs(ey) > p.side_small:
                return Job("walk", ex - 0.6 * p.standoff, "back for a side step")
            return Job("side", clamp(ey, -p.max_side, p.max_side), "line-up")
        if ex > p.final_x + 0.03:
            return Job("walk", min(ex - p.final_x, p.max_step), "line-up, to the ball")
        return Job("walk", ex, "line-up")

    # ---- learning ----
    def observe(self, job: Job, pose0, ball0, pose1, ball1, raw_yaw: float | None = None) -> list:
        """Correct the walker model from the measured result of a move or a turn. Returns log notes.
        raw_yaw: the tag heading before the job, with no heading correction (for the model "heading")."""
        p, notes = self.p, []
        if job.kind == "kick":
            return notes
        m, want = self.model(move_name(job)), abs(job.value)
        sign = math.copysign(1.0, job.value)
        turned = math.degrees(wrap(pose1[2] - pose0[2]))
        if job.kind == "turn":
            got = sign * turned
            if abs(got - want) > max(12.0, 0.5 * want):
                self.odd += 1
                notes.append(f"turn {sign * want:+.1f} deg gave {sign * got:+.1f}: not as commanded, model not changed")
                return notes
            if want >= 20.0:
                m[0] = round(clamp(0.5 * m[0] + 0.5 * m[0] * got / want, 0.4, 1.8), 3)
            else:
                m[1] = round(clamp(m[1] + 0.5 * (want - got) / m[0], 0.0, 6.0), 2)
            notes.append(f"turn {sign * want:+.1f} deg gave {sign * got:+.1f}: model {m}")
            return notes
        ahead, left = to_body((pose0[0], pose0[1], pose0[2]), pose1)
        got, other = sign * (ahead if job.kind == "walk" else left), (left if job.kind == "walk" else ahead)
        text = f"{move_name(job)} {want:.3f} m gave {got:.3f} (other axis {other:+.3f}, turn {turned:+.0f} deg)"
        if abs(turned) > 20.0 or abs(got - want) > max(0.06, 0.6 * want):
            self.odd += 1
            notes.append(text + ": not as commanded, model not changed")
        else:
            if want >= p.learn_min_m:
                m[0] = round(clamp(0.5 * m[0] + 0.5 * m[0] * got / want, 0.3, 1.6), 3)
            else:
                m[1] = round(clamp(m[1] + 0.5 * (want - got) / (m[0] * p.speed), 0.0, 0.3), 3)
            if job.kind == "walk" and job.value >= 0.25 and got > 0.1 and raw_yaw is not None:
                # The walk went "off" degrees to the left of the corrected heading: correct the correction.
                off, fix = math.degrees(math.atan2(other, got)), self.model("heading")
                fix[0] = round(clamp(fix[0] + 0.4 * off * math.cos(raw_yaw), -25.0, 25.0), 1)
                fix[1] = round(clamp(fix[1] + 0.4 * off * math.sin(raw_yaw), -25.0, 25.0), 1)
                text += f", walk {off:+.0f} deg off the heading, heading correction {fix}"
            notes.append(text + f": model {m}")
        if job.kind == "walk" and job.value > 0 and self.lining and ball0 is not None and ball1 is not None:
            push = to_body((ball0[0], ball0[1], pose0[2]), ball1)[0]
            if p.contact_m < push < 0.25:
                self.touched = True
                k, x = self.kick_point(), to_body(pose1, ball1)[0] + self.shift[0]
                k["x"] = round(clamp(0.5 * k["x"] + 0.5 * (x + p.contact_gap), p.kick_x - p.max_shift,
                                     p.kick_x + p.max_shift), 4)
                notes.append(f"the walk moved the ball {push * 100:.1f} cm: touch at x {x:.3f}, kick_x {k['x']:.3f}")
        return notes

    def kick_result(self, pose, ball0, ball1, trace=()) -> dict:
        """Classify a kick from the ball before (ball0), at rest after (ball1 or None), and the points
        seen between. Corrects the kick point, and ends the lock on the goal and the paw."""
        p, k = self.p, self.kick_point()
        points = [tuple(q) for q in trace] + ([tuple(ball1)] if ball1 is not None else [])
        scored = next((g for g in GOALS for q in points
                       if (q[0] - self.goal_x(g)) * self.goal_x(g) > 0 and abs(q[1]) <= p.goal_width / 2.0), None)
        last = points[-1] if points else None
        moved = dist(ball0, last) if last else None
        launch = next((q for q in points if dist(ball0, q) >= 0.15), None)
        err = None if launch is None else math.degrees(wrap(math.atan2(launch[1] - ball0[1], launch[0] - ball0[0]) - pose[2]))
        if scored:
            result = "goal"
        elif ball1 is None:
            result = "lost"
        elif moved < p.miss_m:
            result = "miss"
        else:
            result = "weak" if moved < p.weak_m else "hit"
        bx, by = to_body(pose, ball0)
        bx, by = bx + self.shift[0], by + self.shift[1]
        rec = {"goal": self.goal, "side": self.side, "result": result, "scored": scored,
               "robot": [round(pose[0], 3), round(pose[1], 3), round(math.degrees(pose[2]), 1)],
               "ball_before": [round(ball0[0], 3), round(ball0[1], 3)], "ball_body": [round(bx, 3), round(by, 3)],
               "shift": [round(self.shift[0], 3), round(self.shift[1], 3)],
               "kick_point": [k["x"], k["y"], k["dir"]], "pulses": self.pulses, "touched": self.touched,
               "ball_after": None if last is None else [round(last[0], 3), round(last[1], 3)],
               "moved_m": None if moved is None else round(moved, 3),
               "dir_err_deg": None if err is None else round(err, 1)}
        y0 = p.kick_y_left if self.side == "left" else p.kick_y_right
        if result == "miss":
            k["x"] = round(max(p.kick_x - p.max_shift, k["x"] - p.miss_step), 4)
        elif err is not None and abs(err) > p.dir_big_deg:
            # A ball left of the paw line leaves to the left: move the kick point against the error.
            step = clamp(p.dir_learn * err / p.deg_per_cm / 100.0, -0.012, 0.012)
            k["y"] = round(clamp(k["y"] - step, y0 - p.max_shift / 2.0, y0 + p.max_shift / 2.0), 4)
        elif err is not None:  # a small constant error: turn the aim
            k["dir"] = round(clamp(k["dir"] + p.dir_learn * (err - k["dir"]), -p.dir_big_deg, p.dir_big_deg), 1)
        rec["kick_point_next"] = [k["x"], k["y"], k["dir"]]
        rec["trace"] = [[round(q[0], 2), round(q[1], 2)] for q in points[::max(1, len(points) // 25)]]
        self.goal = None
        return rec


class Abort(RuntimeError):
    pass


class Robot:
    """Client of the Gemini Pupper tool server (same calls as robot/ball_seek.py HttpClient)."""

    def __init__(self, url: str, job_timeout_s: float = 60.0):
        self.url, self.job_timeout_s = url.rstrip("/"), job_timeout_s

    def _request(self, method: str, path: str, body: dict | None = None, epoch: str | None = None) -> dict:
        headers = {"Content-Type": "application/json"}
        if epoch:
            headers["X-Pupper-Command-Epoch"] = epoch
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + path, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=30) as reply:
                return json.load(reply)
        except urllib.error.HTTPError as error:
            try:
                return json.loads(error.read() or b"{}")
            except ValueError:
                return {"success": False, "message": f"HTTP {error.code}"}

    def health(self) -> dict:
        return self._request("GET", "/health")

    def status(self) -> dict:
        return self._request("POST", "/tools/robot_status", {})

    def run(self, tool: str, args: dict, tick=None) -> dict:
        """Start one job and wait until the queue is idle again. tick() runs while the job runs."""
        res = self._request("POST", "/tools/" + tool, args, self.health().get("command_epoch"))
        if not res.get("success"):
            raise Abort(f"{tool} refused: {res.get('message')}")
        job = ((res.get("details") or {}).get("submission") or {}).get("job_id")
        deadline = time.monotonic() + self.job_timeout_s
        while time.monotonic() < deadline:
            poll = time.monotonic() + 0.2
            while tick and time.monotonic() < poll:
                tick()
            time.sleep(max(0.0, poll - time.monotonic()))
            st = self.status().get("details") or {}
            if st.get("current_action") is None and not st.get("queued"):
                last = st.get("last_outcome") or {}
                if job and last.get("job_id") != job:
                    continue  # the outcome of this job is not recorded yet
                if not last.get("success"):
                    raise Abort(f"{tool} failed: {last.get('message')}")
                return last
        raise Abort(f"{tool} did not finish in {self.job_timeout_s:.0f} s")

    def look_ball(self) -> tuple | None:
        """The ball in the body frame from the onboard camera (tool look_ball, no motion), or None."""
        try:
            res = self._request("POST", "/tools/look_ball", {}, self.health().get("command_epoch"))
        except OSError:
            return None
        eye = (res.get("details") or {}).get("ball_eye") or {}
        if res.get("success") and eye.get("seen") and eye.get("x") is not None and eye.get("y") is not None:
            return float(eye["x"]), float(eye["y"])
        return None

    def do(self, job: Job, model, speed: float, tick=None) -> None:
        if job.kind == "kick":
            self.run("kick_in_place", {"side": job.value}, tick)
        elif job.kind == "turn":
            self.run("turn", {"direction": "left" if job.value > 0 else "right",
                              "angle": round(command(job, model, speed), 1)}, tick)
        else:
            self.run("move", {"direction": move_name(job).split("_")[0], "speed": speed,
                              "duration": round(max(0.05, command(job, model, speed)), 2)}, tick)

    def stop(self) -> None:
        try:
            self._request("POST", "/tools/stop", {})
        except Exception:  # noqa: BLE001 - Stop must not raise
            pass


@dataclass
class Obs:
    robot: tuple | None   # field x, y, yaw: medians of the frames (None: the tag was not read)
    ball: tuple | None    # field x, y
    spread: float         # largest standard deviation of a robot or ball coordinate, m
    t: float              # receipt time of the last frame


class Eye:
    """The phone camera and the field perception. The four corner tags define the field."""

    def __init__(self, args, log):
        import cv2
        import numpy as np
        sys.path.insert(0, str(HERE.parent))
        from fieldcam.capture import FrameGrabber
        from fieldcam.config import load_config
        from fieldcam.pipeline import Perception
        self.cv2, self.np, self.log, self.out = cv2, np, log, Path(args.out)
        cfg = load_config(None)
        cfg["tags"]["floor_size"] = args.floor_size
        cfg["ball"]["radius"] = args.ball_radius
        cfg["field"]["goal_width"] = args.goal_width
        self.rc = rc = cfg["robots"][ROBOT]
        rc["tag_height"] = args.tag_height
        rc["tag_offset"][:] = args.tag_offset
        rc["yaw_offset"] = math.radians(args.yaw_offset)
        self.cfg, self.latency = cfg, cfg["camera"]["latency_s"]
        source = int(args.source) if args.source.isdigit() else args.source
        self.grab = FrameGrabber(source, args.width, args.height, 30, wait=15, say=log)
        # The 8 px drift limit of the pipeline is for the 896 px match picture. Keep the same angle here.
        import fieldcam.pipeline as pipeline
        pipeline.CALIBRATION_DRIFT_LIMIT_PX = 8.0 * self.grab.width / 896.0
        self.perc = Perception(cfg, self.grab.width, self.grab.height)
        self._zoom_for_robot_tag(rc["tag_id"])
        self.n, self.state, self.next_shot, self.next_cal = -1, None, 0.0, 0.0
        self.trace, self.marks, self.hfov = [], [], None
        log(f"camera {self.grab.width} x {self.grab.height}")

    def _zoom_for_robot_tag(self, tag_id: int, half: int = 280) -> None:
        """The robot tag is small at the far side of the field. If the detector does not find it,
        look again in an enlarged picture: first near its last position, then in the full frame."""
        import numpy as np
        from fieldcam.detect import TagDetector
        cv2, base, last = self.cv2, self.perc.tag_det.detect, [None]
        # At the far side the tag is approximately 26 x 10 pixels. The standard detector settings do not
        # read it at any enlargement; these settings do (wider threshold windows, more tolerant outline).
        from fieldcam.detect import TAG_DICT
        par = cv2.aruco.DetectorParameters()
        par.adaptiveThreshWinSizeMin, par.adaptiveThreshWinSizeMax, par.adaptiveThreshWinSizeStep = 5, 65, 6
        par.minMarkerPerimeterRate, par.polygonalApproxAccuracyRate, par.errorCorrectionRate = 0.02, 0.08, 0.9
        par.perspectiveRemoveIgnoredMarginPerCell, par.perspectiveRemovePixelPerCell = 0.2, 8
        par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        relaxed = cv2.aruco.ArucoDetector(TAG_DICT, par)

        class zoom:  # same call as TagDetector.detect, standard settings first
            standard = TagDetector(())

            @staticmethod
            def detect(gray):
                tags = zoom.standard.detect(gray)
                if tag_id not in tags:
                    corners, ids, _ = relaxed.detectMarkers(gray)
                    for c, i in zip(corners, [] if ids is None else ids.ravel()):
                        if int(i) == tag_id:
                            tags[tag_id] = c.reshape(4, 2)
                return tags

        def detect(gray):
            tags = base(gray)
            if tag_id not in tags:
                h, w = gray.shape
                boxes = [(0, 0, w, h, (2,))]
                if last[0] is not None:
                    cx, cy = last[0]
                    boxes.insert(0, (max(0, int(cx - half)), max(0, int(cy - half)),
                                     min(w, int(cx + half)), min(h, int(cy + half)), (2, 4, 3)))
                for x0, y0, x1, y1, scales in boxes:
                    for s in scales:
                        big = cv2.resize(gray[y0:y1, x0:x1], None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
                        found = zoom.detect(big).get(tag_id)
                        if found is not None:
                            tags[tag_id] = ((found + 0.5) / s - 0.5 + [x0, y0]).astype(np.float32)
                            break
                    if tag_id in tags:
                        break
            if tag_id in tags:
                last[0] = tags[tag_id].mean(axis=0)
            return tags

        self.perc.tag_det.detect = detect

    def close(self):
        self.grab.close()

    def field(self) -> tuple:
        f = self.perc.cfg["field"]
        return f["length"], f["width"]

    def camera_xy(self) -> tuple:
        return tuple(float(v) for v in self.perc.cam.center[:2])

    def _widen_ball_area(self) -> None:
        """The pipeline looks for the ball to 0.25 m outside the field. Look to 0.55 m behind the
        goal lines, so that a ball in a goal is measured."""
        np, cv2, perc = self.np, self.cv2, self.perc
        corners = np.array(perc.cfg["field"]["corners"], float)
        grown = corners + np.sign(corners) * [0.55, 0.25]
        poly = perc.cam.project(np.hstack([grown, np.zeros((4, 1))])).astype(np.int32)
        mask = np.zeros((perc.size[1], perc.size[0]), np.uint8)
        cv2.fillConvexPoly(mask, poly, 255)
        perc.roi_mask = perc.ball_roi = mask

    def _calibrate(self, frame) -> bool:
        """Field calibration from the corner tags. With no lens calibration file, the lens angle is
        fitted first: the angle for which the corner tags fit a camera above the floor best. The fixed
        angle of the config (69 degrees) is wrong after a change of the phone zoom."""
        perc = self.perc
        if perc.has_calibrated_intrinsics or self.hfov is not None:
            if perc.calibrate_from_corners(frame):
                return True
            if perc.has_calibrated_intrinsics or "does not fit" not in (perc.calib_fail or ""):
                return False
        import fieldcam.pipeline as pipeline
        from fieldcam.camera_model import default_intrinsics
        limit, best = pipeline.CALIBRATION_DRIFT_LIMIT_PX, None
        pipeline.CALIBRATION_DRIFT_LIMIT_PX = 1e9
        try:
            for angles in ([float(a) for a in range(30, 121, 3)], None):
                for hfov in angles or [best[1] + 0.5 * k for k in range(-5, 6)]:
                    perc.K, perc.dist = default_intrinsics(self.grab.width, self.grab.height, hfov)
                    if perc.calibrate_from_corners(frame) and (best is None or perc.cam.reproj_err_px < best[0]):
                        best = (perc.cam.reproj_err_px, hfov)
                if best is None:
                    break
        finally:
            pipeline.CALIBRATION_DRIFT_LIMIT_PX = limit
        perc.invalidate_calibration("lens angle fit")
        if best is None:
            return False
        if self.hfov is None or abs(best[1] - self.hfov) > 1.0:
            self.log(f"lens angle from the corner tags: {best[1]:.1f} degrees (fit error {best[0]:.1f} px)")
        self.hfov = best[1]
        perc.K, perc.dist = default_intrinsics(self.grab.width, self.grab.height, self.hfov)
        return perc.calibrate_from_corners(frame)

    def frame(self) -> dict | None:
        """One processed frame: {"t", "robot": (x, y, yaw) | None, "ball": (x, y) | None}, not filtered."""
        frame, receipt, self.n = self.grab.read(self.n)
        if frame is None:
            return None
        now, perc = time.monotonic(), self.perc
        if not perc.calibration_valid and now >= self.next_cal:
            self.next_cal = now + 0.5
            if self._calibrate(frame):
                self._widen_ball_area()
                self.log("field calibrated from the corner tags: {:.2f} x {:.2f} m".format(*self.field()))
            elif now >= self.next_shot:
                self.log(f"no calibration: {perc.calib_fail}")
        self.state = state = perc.process(frame, receipt - self.latency, time.time())
        robot = ball = None
        if state.get("calibration_valid"):
            cam, rc = perc.cam, self.rc
            tag = perc.debug.get("tags", {}).get(rc["tag_id"])
            if tag is not None:
                c = cam.pixel_to_plane(tag, rc["tag_height"])  # TL, TR, BR, BL
                fwd = (c[0] + c[1]) / 2 - (c[3] + c[2]) / 2
                yaw = wrap(math.atan2(fwd[1], fwd[0]) + rc["yaw_offset"])
                x, y = to_field(c.mean(axis=0), yaw, -rc["tag_offset"][0], -rc["tag_offset"][1])
                robot = (float(x), float(y), yaw)
            px = perc.debug.get("ball_px")
            if px is not None:
                ball = tuple(float(v) for v in cam.pixel_to_plane([px[:2]], self.cfg["ball"]["radius"])[0])
        if now >= self.next_shot:
            self.next_shot = now + 0.5
            self.cv2.imwrite(str(self.out / "raw.jpg"), frame, [self.cv2.IMWRITE_JPEG_QUALITY, 92])  # for checks off line
            self.cv2.imwrite(str(self.out / "latest.jpg"), self._draw(frame, state), [self.cv2.IMWRITE_JPEG_QUALITY, 80])
        return {"t": receipt, "robot": robot, "ball": ball}

    def _draw(self, frame, state):
        """The pipeline picture, plus the plan: marks = [(field x, y, text)], joined by a line."""
        cv2, out = self.cv2, self.perc.draw(frame, state)
        if self.perc.cam is not None and self.marks:
            px = self.perc.cam.project([[x, y, 0.0] for x, y, _ in self.marks]).astype(int)
            for a, b in zip(px[:-1], px[1:]):
                cv2.line(out, tuple(map(int, a)), tuple(map(int, b)), (0, 220, 255), 2)
            for (u, v), (_, _, text) in zip(px, self.marks):
                cv2.circle(out, (int(u), int(v)), 7, (0, 220, 255), 2)
                cv2.putText(out, text, (int(u) + 10, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
        return out

    def track(self) -> None:
        """One frame while a job runs: keep the ball position (the path of a kicked ball)."""
        f = self.frame()
        if f and f["ball"]:
            self.trace.append(f["ball"])

    def sample(self, n: int = 10, timeout: float = 5.0, after: float = 0.0) -> Obs | None:
        """Medians of the frames taken after `after`: until n frames have the robot tag, or the timeout.
        The robot is None with fewer than 3 tag readings, the ball is None with fewer than 3 ball readings."""
        rows, tagged, end = [], 0, time.time() + timeout
        while tagged < n and time.time() < end:
            f = self.frame()
            if f and f["t"] - self.latency >= after and (f["robot"] or f["ball"]):
                rows.append(f)
                tagged += bool(f["robot"])
        med, dev = statistics.median, statistics.pstdev
        robots, balls = [f["robot"] for f in rows if f["robot"]], [f["ball"] for f in rows if f["ball"]][-n:]
        robot = ball = None
        spread = 0.0
        if len(robots) >= min(3, n):
            rx, ry, yaws = [r[0] for r in robots], [r[1] for r in robots], [r[2] for r in robots]
            yaw = math.atan2(statistics.fmean(map(math.sin, yaws)), statistics.fmean(map(math.cos, yaws)))
            robot, spread = (med(rx), med(ry), yaw), max(dev(rx), dev(ry))
        if len(balls) >= 3:
            bx, by = [b[0] for b in balls], [b[1] for b in balls]
            ball, spread = (med(bx), med(by)), max(spread, dev(bx), dev(by))
        return Obs(robot, ball, spread, rows[-1]["t"]) if robot or ball else None

    def ball_at_rest(self, timeout: float = 8.0, n: int = 4, tol: float = 0.008) -> tuple | None:
        """The ball position when two medians of n frames agree. None if the ball is not seen."""
        end, last, seen = time.time() + timeout, None, []
        while time.time() < end:
            f = self.frame()
            if not (f and f["ball"]):
                continue
            self.trace.append(f["ball"])
            seen.append(f["ball"])
            if len(seen) == n:
                now = (statistics.median(b[0] for b in seen), statistics.median(b[1] for b in seen))
                if last is not None and dist(now, last) <= tol:
                    return now
                last, seen = now, []
        return None

    def missing(self) -> str:
        s = self.state
        if s is None:
            return "no camera frame"
        if not s.get("calibration_valid"):
            return f"no field calibration ({self.perc.calib_fail or s.get('calibration_reason')})"
        tags = self.perc.debug.get("tags", {})
        return "not measured: robot tag 10" if self.rc["tag_id"] not in tags else "measurements not steady"


KIT = "/home/pi/pupper-foundation-session-20261029/kickseen-kit-20261120"


def imu_health(host: str) -> dict:
    """IMU tilt and gyro rate of the Pupper over one second (imu-health.py in the kit), {} if not read.
    The IMU orientation of this Pupper drifts after some minutes, and the walker then moves the legs
    against a tilt that is not there (handoffs/2026-11-16-fr-leg-jiggle.md)."""
    cmd = f'eval "$(grep "^export " {KIT}/kickseen-session.sh)"; python3 {KIT}/imu-health.py'
    try:
        out = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, cmd],
                             capture_output=True, text=True, timeout=20).stdout.strip().splitlines()
        return json.loads(out[-1]) if out else {}
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return {}


def open_tunnel(host: str, port: int, log):
    proc = subprocess.Popen(["ssh", "-N", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes",
                             "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
                             "-L", f"{port}:127.0.0.1:18090", host],
                            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    robot, end = Robot(f"http://127.0.0.1:{port}"), time.time() + 10
    while time.time() < end:
        if proc.poll() is not None:
            raise Abort("SSH tunnel closed: " + proc.stderr.read().decode(errors="replace").strip()[-200:])
        try:
            if robot.health():
                log(f"tool server of {host} answers")
                return proc, robot
        except OSError:
            time.sleep(0.3)
    proc.terminate()
    raise Abort("the tool server does not answer: is the kick session up (start-api)?")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--look", action="store_true", help="camera only: print the measurement and the plan, send nothing")
    ap.add_argument("--no-kick", action="store_true", help="line up, then count the kick and do not play it")
    ap.add_argument("--kicks", type=int, default=1)
    ap.add_argument("--goal", default="auto", choices=["auto", *GOALS], help="east is +x of the field frame")
    ap.add_argument("--side", default="left", choices=["auto", *SIDES], help="the front paw that kicks")
    ap.add_argument("--until-goal", action="store_true", help="end the run at the first goal")
    ap.add_argument("--max-jobs", type=int, default=120)
    ap.add_argument("--seconds", type=float, default=300.0, help="end the run after this time")
    ap.add_argument("--host", default="mts-pi", help="SSH alias of the Pupper")
    ap.add_argument("--port", type=int, default=18190, help="local port of the SSH tunnel")
    ap.add_argument("--source", default="phone")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--floor-size", type=float, default=0.15875, help="side of the black square of a floor tag, m")
    ap.add_argument("--ball-radius", type=float, default=0.09)
    ap.add_argument("--goal-width", type=float, default=Params.goal_width, help="goal mouth, m")
    ap.add_argument("--tag-height", type=float, default=0.24, help="floor to the robot tag while the Pupper stands, m"
                    " (0.24 and 0.065: from the feet in the picture and the onboard camera, 2026-11-20)")
    ap.add_argument("--tag-offset", type=float, nargs=2, default=[0.065, 0.0], metavar=("AHEAD", "LEFT"),
                    help="robot tag centre in the body frame, m")
    ap.add_argument("--no-eye", action="store_true", help="do not read the onboard camera in the line-up")
    ap.add_argument("--max-tilt", type=float, default=8.0,
                    help="end the run if the IMU reports more tilt than this while the Pupper stands, degrees (0: no check)")
    ap.add_argument("--yaw-offset", type=float, default=0.0, help="degrees to add if the tag top does not point to the nose")
    ap.add_argument("--settle", type=float, default=0.3, help="wait after a job before a measurement, s")
    ap.add_argument("--out", default=str(HERE.parent / "out" / "overhead_kick"))
    args = ap.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stop_file, cal_file = out / "STOP", out / "calibration.json"
    stop_file.unlink(missing_ok=True)
    logfile = (out / "bridge.log").open("a", buffering=1)

    def log(text: str) -> None:
        line = time.strftime("%H:%M:%S ") + str(text)
        print(line, flush=True)
        logfile.write(line + "\n")

    p = Params(goal_width=args.goal_width)
    cal = json.loads(cal_file.read_text()) if cal_file.exists() else {}
    robot = tunnel = eye = striker = None
    try:
        eye = Eye(args, log)
        if not args.look:
            tunnel, robot = open_tunnel(args.host, args.port, log)
            st = robot.status()
            log("robot: " + str(st.get("message"))[:160])
            if not st.get("success") or "locomotion controller active" not in str(st.get("message")):
                raise Abort("the Pupper does not stand on the walker: release Stop (Options), then stand_up")
        kicks = goals = jobs = misses = 0
        shift, fix, shifted, odd, rest, steady, imu_read = (0.0, 0.0), None, False, 0, None, 0, None
        guess, last_ball, blind = None, None, 0
        ready, pending, end = 0.0, None, time.time() + args.seconds
        while kicks < args.kicks and jobs < args.max_jobs and time.time() < end:
            if stop_file.exists():
                raise KeyboardInterrupt
            can_guess = guess is not None and blind < p.blind_jobs
            obs = eye.sample(n=8 if striker and striker.lining else 4, timeout=p.blind_wait_s if can_guess else 5.0,
                             after=ready)
            if obs is None or obs.spread > 0.02 or (obs.robot is None and not can_guess):
                misses += 1
                if misses % 3 == 1:
                    log("waiting: " + (eye.missing() if obs is None or obs.robot is None
                                       else f"measurement spread {obs.spread * 100:.1f} cm"))
                continue
            if striker is None:
                striker = Striker(p, *eye.field(), cal, args.goal, args.side, eye.camera_xy())
            if obs.robot is None:  # the tag is not read: go on from the pose that the last job gives
                blind, pending, raw_yaw = blind + 1, None, None
                obs.robot, obs.ball = guess, obs.ball or last_ball
                log(f"  robot tag not read: job {blind} of {p.blind_jobs} from dead reckoning")
            else:
                blind = 0
                raw_yaw, obs.robot = obs.robot[2], true_pose(obs.robot, striker.model("heading"))
            if pending is not None:  # the result of the last job
                job, before, before_yaw = pending
                pending = None
                for note in striker.observe(job, before.robot, before.ball, obs.robot, obs.ball, before_yaw):
                    log("  " + note)
                cal_file.write_text(json.dumps(cal, indent=1))
                step = to_body(before.robot, obs.robot)
                if job.kind == "walk" and abs(job.value) >= 0.2 and (abs(step[1]) > max(0.12, abs(step[0]))
                                                                     or step[0] * job.value < -0.05):
                    raise Abort(f"a walk of {job.value:+.2f} m moved the Pupper {step[0]:+.2f} m ahead and "
                                f"{step[1]:+.2f} m to the left: the robot tag direction is wrong (see --yaw-offset)")
            if obs.ball is None or not striker.in_play(obs.ball):
                misses += 1
                if misses % 5 == 1:
                    log("waiting: the ball is not seen" if obs.ball is None else
                        f"waiting: the ball ({obs.ball[0]:+.2f}, {obs.ball[1]:+.2f}) is not in play")
                continue
            misses = 0
            if striker.goal is None:  # a new kick starts only on a ball that rests (a person can be in the field)
                steady = steady + 1 if rest and dist(rest, obs.ball) < 0.03 else 0
                rest = obs.ball
                if steady < 2:
                    continue
            if robot and args.max_tilt > 0 and imu_read is None and (striker.goal is None or striker.odd > odd):
                # One time for each kick, and after each move that did not go as commanded. The read takes
                # some seconds, so it runs beside the first jobs. It reads the IMU while the Pupper moves
                # too: the tilt limit is for a Pupper on its feet, the gyro limit is used at rest only.
                odd, imu_read = striker.odd, {}
                threading.Thread(target=lambda box=imu_read: box.update(imu_health(args.host) or {"none": 1}),
                                 daemon=True).start()
            if imu_read:
                imu, imu_read = imu_read, None
                log(f"IMU: tilt {imu.get('tilt_deg')} deg, gyro {imu.get('gyro_rad_s')} rad/s")
                if (imu.get("tilt_deg") or 0.0) > args.max_tilt:
                    raise Abort(f"the IMU reports a tilt of {imu.get('tilt_deg')} deg: the IMU drifts, "
                                "start the kick session again")
            ox, oy = to_body(obs.robot, obs.ball)
            if p.eye_min_x <= ox <= p.eye_max_x and abs(oy) <= p.eye_max_y:
                fix = robot.look_ball() if robot and not args.no_eye else None
                if fix and abs(math.hypot(*fix) - math.hypot(ox, oy)) <= p.eye_max_diff and abs(math.degrees(
                        wrap(math.atan2(fix[1], fix[0]) - math.atan2(oy, ox)))) <= p.eye_max_bearing_deg:  # else: last shift
                    # The newest reading, not a mean: the difference changes with the range (run 6).
                    shift, shifted = (fix[0] - ox, fix[1] - oy), True
            else:
                shift, fix, shifted = (0.0, 0.0), None, False
            job = striker.next_job(obs.robot, obs.ball, shift)
            k, (bx, by) = striker.kick_point(), (ox + shift[0], oy + shift[1])
            psi, _ = striker.aim(obs.ball)
            spot = striker.stage(obs.ball, striker.goal, striker.side)
            eye.marks = [(spot[0], spot[1], "stage"), (obs.ball[0], obs.ball[1], ""),
                         (striker.goal_x(striker.goal), obs.ball[1] + math.tan(psi) * (striker.goal_x(striker.goal) - obs.ball[0]), striker.goal)]
            log(f"robot ({obs.robot[0]:+.2f}, {obs.robot[1]:+.2f}) yaw {math.degrees(obs.robot[2]):+.0f} | "
                f"ball ({obs.ball[0]:+.2f}, {obs.ball[1]:+.2f}) overhead x {ox:+.3f} y {oy:+.3f}"
                + (f" onboard x {fix[0]:+.3f} y {fix[1]:+.3f}" if fix else "") + f" | {striker.goal} {striker.side}: "
                f"error x {bx - k['x']:+.3f} y {by - k['y']:+.3f} yaw {math.degrees(wrap(psi - obs.robot[2])):+.0f} -> {job.text()}")
            if args.look:
                time.sleep(0.5)
                continue
            jobs += 1
            if job.kind != "kick":
                robot.do(job, striker.model(move_name(job)), p.fast_speed if job.fast else p.speed)
                pending, ready = (job, obs, raw_yaw), time.time() + args.settle
                guess, last_ball = predict(obs.robot, job), obs.ball
                continue
            eye.trace = []
            saved = json.dumps(cal.get("kick", {}))
            if not args.no_kick:
                robot.do(job, None, p.speed, eye.track)
            after = eye.ball_at_rest()
            rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "dry_run": args.no_kick,
                   "overhead": [round(ox, 3), round(oy, 3)], "onboard": fix and [round(fix[0], 3), round(fix[1], 3)],
                   "tag": [args.tag_height, *args.tag_offset],
                   **striker.kick_result(obs.robot, obs.ball, after, eye.trace)}
            if blind:  # the pose at the kick was not measured: no correction of the kick point from this kick
                cal["kick"] = json.loads(saved)
                rec["blind"] = blind
            shift, fix, shifted, guess, blind = (0.0, 0.0), None, False, None, 0
            with (out / "kicks.jsonl").open("a") as f:
                f.write(json.dumps(rec) + "\n")
            cal_file.write_text(json.dumps(cal, indent=1))
            kicks += 1
            goals += rec["result"] == "goal"
            log(f"KICK {kicks}: {rec['result']}" + (f" in the {rec['scored']} goal" if rec["scored"] else "")
                + f" | ball moved {rec['moved_m']} m, {rec['dir_err_deg']} deg off the heading | "
                f"ball at the kick: body x {rec['ball_body'][0]} y {rec['ball_body'][1]} | next kick point {rec['kick_point_next']}")
            ready = time.time() + args.settle
            if args.until_goal and rec["result"] == "goal":
                break
        log(f"end: {kicks} kick(s), {goals} goal(s), {jobs} job(s)")
        return 0
    except KeyboardInterrupt:
        if robot:
            robot.stop()
        log("STOPPED: Stop sent" if robot else "stopped")
        return 130
    except Abort as error:
        if robot:
            robot.stop()
        log(f"ABORT: {error}" + (" (Stop sent)" if robot else ""))
        return 1
    except Exception as error:  # an unexpected fault must still stop the robot
        if robot:
            robot.stop()
        log(f"ERROR: {type(error).__name__} {error}" + (" (Stop sent)" if robot else ""))
        return 1
    finally:
        if eye:
            eye.close()
        if tunnel:
            tunnel.terminate()


if __name__ == "__main__":
    sys.exit(main())
