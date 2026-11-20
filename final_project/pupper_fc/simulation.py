"""Deterministic, approximate 2D soccer sandbox; not a robot/contact model.

The robot is a bounded disc with a short frontal bumper. Body-frame commands
accelerate toward their target, and expired/invalid commands brake toward zero.
The ball rolls with friction and bounces at walls outside the goal mouth. These
assumptions make gameplay inspectable; none are measured Pupper performance.
"""
from collections import deque
from dataclasses import asdict
import math
import random

from .model import Ball, Command, Field, Limits, Observation, Pose, Velocity


SCENARIOS = {
    "center": {"name": "Straight shot", "description": "Start behind a stationary ball at midfield."},
    "offset": {"name": "Offset approach", "description": "Approach a stationary ball from below the shooting line."},
    "wrong_side": {"name": "Wrong side", "description": "Get behind the ball before shooting."},
    "moving_ball": {"name": "Moving ball", "description": "Track a slowly rolling ball before lining up."},
    "near_wall": {"name": "Near wall", "description": "Work a ball near the upper sideline."},
}


def _clip(value, low, high):
    return max(low, min(high, value))


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


class SimulatedKick:
    """Post synthetic policy receipts on simulation time, without a ball impulse.

    The fast profile leaves the clip at 2.1 s. Tej measured about 2.7 s from
    controller activation to walker activation, then allows 0.7 s for readiness.
    These event times exercise ownership; they do not simulate the moving foot.
    """

    def __init__(self, handoff):
        self.handoff = handoff
        self.pending = None
        self.clip_confirmed = False

    def update(self, step, now):
        if step.cancel is not None:
            if self.pending == step.cancel:
                self.handoff.receipt(self.pending, "walker_restored", walker_active=True)
                self.pending = None
            return
        if step.request is not None:
            if self.pending is not None:
                raise RuntimeError("simulated kick already owns the legs")
            if not step.request.issued_at <= now < step.request.start_before:
                self.handoff.receipt(step.request, "failed")
                self.handoff.receipt(step.request, "walker_restored", walker_active=True)
                return
            self.pending = step.request
            self.clip_confirmed = False
        if self.pending is None:
            return
        elapsed = now - self.pending.issued_at
        if elapsed >= 2.1 and not self.clip_confirmed:
            self.clip_confirmed = self.handoff.receipt(
                self.pending, "clip_confirmed", clip=f"kick_{self.pending.side}")
        if elapsed >= 3.4 and self.clip_confirmed:
            self.handoff.receipt(self.pending, "walker_restored", walker_active=True)
            self.pending = None


class Simulator:
    """One robot, one ball, no networking, hardware access or recovery physics.

    ``snapshot`` is operator-only truth. ``observe`` samples buffered frames and
    estimates ball velocity from delivered positions, never current truth.
    ``noise`` is position standard deviation in metres (heading uses 1.5 times
    that value in radians). A frozen camera repeats its entire previous packet.
    """

    def __init__(self, field=Field(), limits=Limits(), seed=7, scenario="center",
                 *, bumper_contact=True):
        field_values = (field.length, field.width, field.goal_width,
                        field.robot_radius, field.ball_radius, field.bumper_offset)
        limit_values = (limits.vx, limits.vy, limits.wz,
                        limits.acceleration, limits.yaw_acceleration)
        if not all(math.isfinite(v) and v > 0 for v in field_values):
            raise ValueError("field dimensions must be finite and positive")
        if (field.attack_sign not in (-1, 1) or field.goal_width > field.width
                or min(field.length, field.width) <= 2 * max(field.robot_radius, field.ball_radius)
                or field.goal_width <= 2 * field.ball_radius):
            raise ValueError("invalid field geometry or attack sign")
        if not all(math.isfinite(v) and v >= 0 for v in limit_values) or min(limit_values[-2:]) <= 0:
            raise ValueError("limits must be finite and nonnegative, with positive acceleration")
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario: {scenario}")
        if not isinstance(seed, int):
            raise ValueError("seed must be an integer")
        self.field, self.limits = field, limits
        self.bumper_contact = bumper_contact
        self.scenario = scenario
        self.time = 0.0
        self.velocity = Velocity()
        self.goal = 0
        self.fallen = False
        self._random = random.Random(seed)
        self._history = deque(maxlen=4001)
        self._last_observation = None
        self._observation_key = None
        self._last_ball_sample = None
        setups = {
            "center": ((-0.95, 0.0, 0.0), (-0.1, 0.0, 0.0, 0.0)),
            "offset": ((-1.05, -0.65, 0.25), (0.0, 0.30, 0.0, 0.0)),
            "wrong_side": ((0.75, 0.15, math.pi), (0.0, 0.0, 0.0, 0.0)),
            "moving_ball": ((-1.05, -0.45, 0.2), (-0.2, 0.30, 0.25, -0.18)),
            "near_wall": ((-0.9, 0.55, 0.4), (0.1, 1.08, 0.0, 0.0)),
        }
        robot, ball = setups[scenario]
        sx, sy, sign = field.length / 3.66, field.width / 2.44, field.attack_sign
        self.robot = Pose(sign * robot[0] * sx, robot[1] * sy,
                          _angle(robot[2] if sign == 1 else math.pi - robot[2]))
        self.ball = Ball(sign * ball[0] * sx, ball[1] * sy, sign * ball[2], ball[3])
        self.move_robot(self.robot.x, self.robot.y, self.robot.yaw)
        self.move_ball(self.ball.x, self.ball.y)
        self.ball = Ball(self.ball.x, self.ball.y, sign * ball[2], ball[3])
        self._record()

    def _record(self):
        frame = Observation(self.time, self.robot, self.ball, fallen=self.fallen, goal=self.goal)
        if self._history and self._history[-1].captured_at == self.time:
            self._history[-1] = frame
        else:
            self._history.append(frame)

    def step(self, command: Command, dt=0.05):
        """Advance at most 0.2 seconds with small internal collision steps."""
        if not math.isfinite(dt) or not 0 < dt <= 0.2:
            raise ValueError("dt must be finite, positive and at most 0.2 seconds")
        valid = (isinstance(command, Command) and command.robot_id == "pupper-1"
                 and all(_finite_number(v) for v in
                         (command.vx, command.vy, command.wz, command.expires_at)))
        end = self.time + dt
        while self.time < end - 1e-12:
            h = min(0.01, end - self.time)
            if valid and self.time < command.expires_at < self.time + h:
                h = command.expires_at - self.time
            if valid and command.expires_at > self.time and not self.fallen:
                target = Velocity(_clip(command.vx, -self.limits.vx, self.limits.vx),
                                  _clip(command.vy, -self.limits.vy, self.limits.vy),
                                  _clip(command.wz, -self.limits.wz, self.limits.wz))
            else:
                target = Velocity()
            self._move(h, target)
            if not self.goal:
                self._roll_ball(h)
            self.time += h
        self.time = end
        self._record()

    def _move(self, dt, target):
        if self.fallen:
            self.velocity = Velocity()
            return
        old = self.velocity
        dx, dy = target.vx - old.vx, target.vy - old.vy
        distance = math.hypot(dx, dy)
        scale = min(1.0, self.limits.acceleration * dt / distance) if distance else 0.0
        velocity = Velocity(old.vx + dx * scale, old.vy + dy * scale,
                            old.wz + _clip(target.wz - old.wz,
                                           -self.limits.yaw_acceleration * dt,
                                           self.limits.yaw_acceleration * dt))
        yaw_mid = self.robot.yaw + (old.wz + velocity.wz) * dt / 4
        c, s = math.cos(yaw_mid), math.sin(yaw_mid)
        wx = ((old.vx + velocity.vx) * c - (old.vy + velocity.vy) * s) / 2
        wy = ((old.vx + velocity.vx) * s + (old.vy + velocity.vy) * c) / 2
        xmax = self.field.length / 2 - self.field.robot_radius
        ymax = self.field.width / 2 - self.field.robot_radius
        x = _clip(self.robot.x + wx * dt, -xmax, xmax)
        y = _clip(self.robot.y + wy * dt, -ymax, ymax)
        # Wall contact removes velocity into the wall, including in body coordinates.
        world_vx, world_vy = velocity.vx * c - velocity.vy * s, velocity.vx * s + velocity.vy * c
        if abs(x) >= xmax and x * world_vx > 0:
            world_vx = 0.0
        if abs(y) >= ymax and y * world_vy > 0:
            world_vy = 0.0
        body_vx, body_vy = world_vx * c + world_vy * s, -world_vx * s + world_vy * c
        # Projection onto a wall can increase an individual body component.
        # Scale both together to preserve wall tangency and the speed limits.
        scale = min(1.0, self.limits.vx / abs(body_vx) if body_vx else 1.0,
                    self.limits.vy / abs(body_vy) if body_vy else 1.0)
        self.velocity = Velocity(body_vx * scale, body_vy * scale, velocity.wz)
        self.robot = Pose(x, y, _angle(self.robot.yaw + (old.wz + velocity.wz) * dt / 2))

    def _roll_ball(self, dt):
        ball, field = self.ball, self.field
        speed = math.hypot(ball.vx, ball.vy)
        slowed = max(0.0, speed - 0.18 * dt)
        factor = slowed / speed if speed else 0.0
        vx, vy = ball.vx * factor, ball.vy * factor
        x, y = ball.x + (ball.vx + vx) * dt / 2, ball.y + (ball.vy + vy) * dt / 2
        x, y, vx, vy = self._ball_contact(x, y, vx, vy)
        ymax = field.width / 2 - field.ball_radius
        if abs(y) > ymax:
            y = math.copysign(ymax, y)
            if y * vy > 0:
                vy *= -0.65
        in_mouth = abs(y) + field.ball_radius <= field.goal_width / 2
        if in_mouth and abs(x) >= field.length / 2 + field.ball_radius:
            self.goal = (1 if x > 0 else -1) * field.attack_sign
            self.ball = Ball(math.copysign(field.length / 2 + field.ball_radius, x), y)
            return
        xmax = field.length / 2 - field.ball_radius
        if not in_mouth and abs(x) > xmax:
            x = math.copysign(xmax, x)
            if x * vx > 0:
                vx *= -0.65
        self.ball = Ball(x, y, vx, vy)

    def _ball_contact(self, x, y, vx, vy):
        robot, field = self.robot, self.field
        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        dx, dy = x - robot.x, y - robot.y
        forward, left = dx * c + dy * s, -dx * s + dy * c
        bumper_half_width = field.robot_radius * 0.75
        bumper = (self.bumper_contact and 0 <= forward < field.bumper_offset + field.ball_radius
                  and abs(left) <= bumper_half_width)
        distance = math.hypot(dx, dy)
        if bumper:
            nx, ny = c, s
            depth = field.bumper_offset + field.ball_radius - forward
        elif distance < field.robot_radius + field.ball_radius:
            nx, ny = (dx / distance, dy / distance) if distance > 1e-9 else (c, s)
            depth = field.robot_radius + field.ball_radius - distance
        else:
            return x, y, vx, vy
        x, y = x + nx * depth, y + ny * depth
        # Tangential bumper movement from yaw contributes to contact speed.
        rvx = self.velocity.vx * c - self.velocity.vy * s - self.velocity.wz * (y - robot.y)
        rvy = self.velocity.vx * s + self.velocity.vy * c + self.velocity.wz * (x - robot.x)
        closing = (rvx - vx) * nx + (rvy - vy) * ny
        if closing > 0:
            impulse = closing * (1.65 if bumper else 1.15)
            vx, vy = vx + nx * impulse, vy + ny * impulse
        return x, y, vx, vy

    def observe(self, delay=0.0, noise=0.0, ball_visible=True, frozen=False):
        """Return a camera packet; repeated/frozen captures retain their timestamp.

        Delays older than the finite history return its earliest available frame.
        Freeze holds all previous fields, including visibility and referee state.
        """
        if not all(math.isfinite(v) and v >= 0 for v in (delay, noise)):
            raise ValueError("delay and noise must be finite and nonnegative")
        if frozen and self._last_observation is not None:
            return self._last_observation
        cutoff = self.time - delay
        frame = next((frame for frame in reversed(self._history)
                      if frame.captured_at <= cutoff + 1e-9), self._history[0])
        key = (frame, noise, bool(ball_visible))
        if key == self._observation_key:
            return self._last_observation
        def jitter(value, scale=1.0):
            return value + self._random.gauss(0, noise * scale) if noise else value
        robot = Pose(jitter(frame.robot.x), jitter(frame.robot.y), _angle(jitter(frame.robot.yaw, 1.5)))
        ball = None
        if ball_visible:
            x, y = jitter(frame.ball.x), jitter(frame.ball.y)
            vx = vy = 0.0
            previous = self._last_ball_sample
            if previous and frame.captured_at > previous[0] + 1e-9:
                elapsed = frame.captured_at - previous[0]
                vx, vy = (x - previous[1]) / elapsed, (y - previous[2]) / elapsed
            ball = Ball(x, y, vx, vy)
            self._last_ball_sample = (frame.captured_at, x, y)
        else:
            self._last_ball_sample = None
        packet = Observation(frame.captured_at, robot, ball, fallen=frame.fallen, goal=frame.goal)
        self._observation_key = key
        self._last_observation = packet
        return packet

    def move_ball(self, x, y):
        """Operator placement inside the field; a latched goal needs explicit reset."""
        if not all(math.isfinite(value) for value in (x, y)):
            raise ValueError("ball position must be finite")
        xmax = self.field.length / 2 - self.field.ball_radius
        ymax = self.field.width / 2 - self.field.ball_radius
        self.ball = Ball(_clip(x, -xmax, xmax), _clip(y, -ymax, ymax))
        self._last_ball_sample = None
        self._record()

    def restart_play(self, x=0.0, y=0.0):
        """Place the kickoff ball and clear the referee event atomically.

        Explicit kickoff starts a new observation epoch: prior-play delayed and
        frozen packets are discarded so a past goal cannot latch the next play.
        The robot's pose, velocity and simulation clock are preserved. Ordinary
        ``move_ball`` does not clear the goal or replace the observation epoch.
        """
        self.move_ball(x, y)  # Validate before changing the goal or observation epoch.
        self.goal = 0
        self._history.clear()
        self._last_observation = None
        self._observation_key = None
        self._last_ball_sample = None
        self._record()

    def set_fallen(self, fallen):
        """Record an operator fall/reposition report on the current capture."""
        if not isinstance(fallen, bool):
            raise ValueError("fallen must be true or false")
        self.fallen = fallen
        self._record()

    def move_robot(self, x, y, yaw=None):
        if yaw is None:
            yaw = self.robot.yaw
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("robot position and heading must be finite")
        xmax = self.field.length / 2 - self.field.robot_radius
        ymax = self.field.width / 2 - self.field.robot_radius
        self.robot = Pose(_clip(x, -xmax, xmax), _clip(y, -ymax, ymax), _angle(yaw))
        self.velocity = Velocity()
        self._record()

    def nudge_ball(self, vx, vy):
        """Set an operator ball velocity, capped at 3 m/s for this simple model."""
        if not all(math.isfinite(value) for value in (vx, vy)):
            raise ValueError("ball velocity must be finite")
        speed = math.hypot(vx, vy)
        scale = min(1.0, 3.0 / speed) if speed else 0.0
        if not self.goal:
            self.ball = Ball(self.ball.x, self.ball.y, vx * scale, vy * scale)
            self._record()

    def snapshot(self):
        """JSON-ready operator truth; do not pass this to gameplay decisions."""
        return {"time": self.time, "robot": asdict(self.robot), "ball": asdict(self.ball),
                "velocity": asdict(self.velocity), "goal": self.goal, "fallen": self.fallen,
                "field": asdict(self.field), "limits": asdict(self.limits), "scenario": self.scenario}
