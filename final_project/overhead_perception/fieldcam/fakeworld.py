"""A fake field: one Pupper and a ball moving on a flat floor, with no camera involved.

It stands in for the whole perception pipeline. Each step it produces a state
dict with the same shape the real pipeline publishes (so the dashboard and any
other consumer work unchanged), including the robot-relative ball and goal that
the approach planner needs.

This is a cartoon for trying the approach LOGIC, not physics: the robot is a
point that follows its velocity command under simple acceleration limits, the
ball is a disc that slows at a constant rate, bounces off the field edge and is
knocked along by the robot's front bumper (roll_ball sends it rolling). It says nothing about what a real Pupper or ball will do.

The fake robot cannot walk backwards (like the real one). With sloppy=True it
also follows commands badly, in ways the team has seen or assumed: it reaches
only 80% of the forward speed asked for (the shooting code's speed_efficiency),
70% sideways, veers left while walking and creeps forward while strafing. The
veer and strafe figures are guesses to make the logic prove itself, not
measurements.
"""
from __future__ import annotations

import math

import numpy as np

from .filters import wrap
from .relative import relative_geometry


class FakeWorld:
    def __init__(self, length=3.66, width=2.44, goal_width=0.82, ball_radius=0.08, attacks="east",
                 robot=(-1.0, -0.5, 0.3), ball=(0.3, 0.2), seed=0, noise=0.0,
                 bumper_offset=0.17, max_acc=1.2, max_yaw_acc=4.0, ball_friction=0.35,
                 sloppy=False, can_reverse=False):
        # How the robot really responds to a command. Perfect unless sloppy.
        self.can_reverse = can_reverse
        self.forward_gain, self.strafe_gain = (0.8, 0.7) if sloppy else (1.0, 1.0)
        self.veer = 0.3 if sloppy else 0.0           # rad of unwanted left turn per metre walked forward
        self.strafe_creep = 0.2 if sloppy else 0.0   # m of unwanted forward travel per metre strafed
        self.length, self.width, self.goal_width, self.ball_radius = length, width, goal_width, ball_radius
        self.attacks, self.bumper_offset = attacks, bumper_offset
        self.max_acc, self.max_yaw_acc, self.ball_friction = max_acc, max_yaw_acc, ball_friction
        self.rng, self.noise = np.random.default_rng(seed), noise
        self.t, self.seq, self.goals_scored, self.kicks = 0.0, 0, 0, 0
        self.robot = np.array(robot, float)          # x, y, yaw in the field frame
        self.robot_vel = np.zeros(3)                 # body-frame vx, vy and wz actually achieved
        self.ball = np.array(ball, float)            # x, y
        self.ball_vel = np.zeros(2)
        self.last_goal_t = None
        self.event = ""

    # ---- geometry the consumer sees -------------------------------------------
    @property
    def goals(self) -> dict:
        hl, hg = self.length / 2, self.goal_width / 2
        return {"east": {"x": hl, "y_min": -hg, "y_max": hg}, "west": {"x": -hl, "y_min": -hg, "y_max": hg}}

    def place_ball(self, x, y):
        self.ball[:] = (np.clip(x, -self.length / 2 + 0.1, self.length / 2 - 0.1),
                        np.clip(y, -self.width / 2 + 0.1, self.width / 2 - 0.1))
        self.ball_vel[:] = 0.0

    def place_robot(self, x, y, yaw=None):
        self.robot[:2] = (np.clip(x, -self.length / 2, self.length / 2), np.clip(y, -self.width / 2, self.width / 2))
        if yaw is not None:
            self.robot[2] = yaw
        self.robot_vel[:] = 0.0

    def roll_ball(self, speed: float | None = None, direction: float | None = None):
        """Send the ball rolling: speed in m/s and direction in radians (field frame); random if not given."""
        speed = self.rng.uniform(0.6, 1.5) if speed is None else speed
        direction = self.rng.uniform(-math.pi, math.pi) if direction is None else direction
        self.ball_vel[:] = (speed * math.cos(direction), speed * math.sin(direction))

    def random_scene(self):
        hl, hw = self.length / 2, self.width / 2
        self.place_ball(self.rng.uniform(-0.5 * hl, 0.6 * hl), self.rng.uniform(-0.7 * hw, 0.7 * hw))
        for _ in range(50):
            x, y = self.rng.uniform(-0.9 * hl, 0.9 * hl), self.rng.uniform(-0.85 * hw, 0.85 * hw)
            if math.hypot(x - self.ball[0], y - self.ball[1]) > 0.5:
                break
        self.place_robot(x, y, self.rng.uniform(-math.pi, math.pi))

    # ---- one step of the cartoon ------------------------------------------------
    def step(self, cmd: dict | None, dt: float):
        """cmd: body-frame {vx, vy, wz} (None = stand still). Advances time by dt seconds."""
        self.t += dt
        self.event = ""
        want = np.zeros(3)
        if cmd:
            vx = cmd["vx"] if self.can_reverse else max(cmd["vx"], 0.0)
            vx *= self.forward_gain
            want = np.array([vx + self.strafe_creep * abs(cmd["vy"]), cmd["vy"] * self.strafe_gain,
                             cmd["wz"] + self.veer * vx])
        lim = np.array([self.max_acc, self.max_acc, self.max_yaw_acc]) * dt
        self.robot_vel += np.clip(want - self.robot_vel, -lim, lim)
        c, s = math.cos(self.robot[2]), math.sin(self.robot[2])
        vx, vy, wz = self.robot_vel
        world_v = np.array([c * vx - s * vy, s * vx + c * vy])
        self.robot[:2] += world_v * dt
        self.robot[2] = wrap(self.robot[2] + wz * dt)

        # Ball: roll, slow down, bounce softly off the boundary.
        speed = float(np.linalg.norm(self.ball_vel))
        if speed > 0:
            self.ball_vel *= max(0.0, speed - self.ball_friction * dt) / speed
        self.ball += self.ball_vel * dt
        hl, hw, r = self.length / 2, self.width / 2, self.ball_radius
        in_mouth = abs(self.ball[1]) < self.goal_width / 2
        for axis, half in ((0, hl), (1, hw)):
            if abs(self.ball[axis]) > half - r and not (axis == 0 and in_mouth):
                self.ball[axis] = math.copysign(half - r, self.ball[axis])
                self.ball_vel[axis] *= -0.4
        if abs(self.ball[0]) > hl and in_mouth and self.last_goal_t is None:
            scored_in = "east" if self.ball[0] > 0 else "west"
            self.goals_scored += scored_in == self.attacks
            self.event = f"GOAL in the {scored_in} goal" + ("" if scored_in == self.attacks else " (own goal)")
            self.last_goal_t = self.t
            self.ball_vel[:] = 0.0

        # Bumper: a small disc at the nose pushes the ball along.
        nose = self.robot[:2] + self.bumper_offset * np.array([c, s])
        gap = self.ball - nose
        dist = float(np.linalg.norm(gap))
        reach = self.ball_radius + 0.05
        if 1e-9 < dist < reach and self.last_goal_t is None:
            n = gap / dist
            self.ball = nose + n * reach
            closing = float(world_v @ n)
            if closing > float(self.ball_vel @ n):
                self.ball_vel += (1.5 * closing - float(self.ball_vel @ n)) * n  # leaves faster than the nose
                if closing > 0.2:
                    self.kicks += 1
                    self.event = "bumper hit the ball"

        # After a goal the ball is put back at the centre, as a person would.
        if self.last_goal_t is not None and self.t - self.last_goal_t > 2.0:
            self.place_ball(0.0, 0.0)
            self.last_goal_t = None
            self.event = "ball replaced at the centre"

    # ---- what perception would publish -----------------------------------------
    def state(self, name: str = "striker") -> dict:
        self.seq += 1
        jitter = lambda: self.rng.normal(0.0, self.noise) if self.noise else 0.0  # noqa: E731
        c, s = math.cos(self.robot[2]), math.sin(self.robot[2])
        vx, vy, wz = self.robot_vel
        robot = {"visible": True, "x": float(self.robot[0] + jitter()), "y": float(self.robot[1] + jitter()),
                 "yaw": float(self.robot[2] + jitter()), "vx": float(c * vx - s * vy), "vy": float(s * vx + c * vy),
                 "wz": float(wz), "age_s": 0.0, "measured_this_frame": True, "measured_at": self.t}
        ball_seen = self.last_goal_t is None  # the ball is "out of play" while it sits in the goal
        ball = ({"visible": True, "x": float(self.ball[0] + jitter()), "y": float(self.ball[1] + jitter()),
                 "vx": float(self.ball_vel[0]), "vy": float(self.ball_vel[1]), "age_s": 0.0,
                 "measured_this_frame": True, "measured_at": self.t}
                if ball_seen else {"visible": False, "measured_this_frame": False, "measured_at": None})
        goals = self.goals
        return {
            "seq": self.seq, "stamp": self.t, "captured_at": self.t, "frame_age_s": 0.0,
            "calibrated": True, "calibration_valid": True, "calib_mode": "fake",
            "calib_err_px": 0.0, "ball": ball, "robots": {name: robot}, "goals": goals, "shots": {},
            "field": {"length": self.length, "width": self.width, "ball_radius": self.ball_radius},
            "floor_tags": {},
            "relative": {name: relative_geometry(robot, ball if ball_seen else None, goals[self.attacks])},
        }
