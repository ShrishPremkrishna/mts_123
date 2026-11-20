"""Steer to a foot-specific ball pose and measure when the walker can yield.

The common ball offset comes from Tej's u5 tool. The tolerances, braking envelope and
wind-up allowance are provisional until measured with the intended ball.
"""
import math
from collections import deque
from dataclasses import dataclass

from .controller import KP_POSITION, KP_YAW, MAX_OBSERVATION_AGE, _paw_travel_heading, _walk_velocity
from .model import Ball, Command, Pose


PAW_FORWARD_M = 0.258
# Combined-release candidates from a rigid size-5 ball in MuJoCo. Tej owns floor validation.
COMBINED_PAW_FORWARD_M = {"left": 0.258, "right": 0.270}
PAW_LEFT_M = {"left": 0.040, "right": -0.046}
FORWARD_WINDOW_M = 0.020
LATERAL_WINDOW_M = 0.015
YAW_WINDOW_RAD = 0.075
WINDUP_S = 2.3
SETTLE_S = 0.20
MAX_CAPTURE_GAP_S = 0.15
COMMAND_LEASE_S = 0.15
# Provisional steering noise floor. The camera rehearsal exercises 2 mm noise.
ROLL_HISTORY_S = .40
ROLL_POSITION_FLOOR_M = .003
INTERCEPT_STEP_S = .020


@dataclass(frozen=True)
class BallRoll:
    speed: float
    ux: float
    uy: float
    deceleration: float
    horizon: float
    position_error: float
    speed_error: float

    def position(self, ball, t):
        if self.deceleration > 0:
            t = min(t, self.speed / self.deceleration)
        distance = t * (self.speed - .5 * self.deceleration * t)
        return Ball(ball.x + self.ux * distance, ball.y + self.uy * distance)


def _travel_time(distance, velocity, cap, acceleration):
    """First arrival along one axis with a signed initial speed and bounded acceleration."""
    if distance <= 0:
        return 0.
    if cap <= 0 or acceleration <= 0:
        return math.inf
    velocity = max(-cap, min(cap, velocity))
    ramp_distance = (cap * cap - velocity * velocity) / (2 * acceleration)
    if distance <= ramp_distance:
        return (math.sqrt(velocity * velocity + 2 * acceleration * distance) - velocity) / acceleration
    return (cap - velocity) / acceleration + (distance - ramp_distance) / cap


def _arrival(robot, pose, limits, command):
    """Estimated time to enter the contact tolerances under the normal steering law."""
    def time_to_window(distance, tolerance, velocity, cap, acceleration, gain):
        if distance <= tolerance:
            return 0.
        fastest = _travel_time(distance - tolerance, velocity, cap, acceleration)
        if not math.isfinite(fastest):
            return fastest
        # Below cap/gain the actual command reduces speed in proportion to
        # the remaining error. Account for that travel as well as acceleration.
        near = min(distance, cap / gain)
        if near > tolerance:
            fastest += math.log(near / tolerance) / gain - (near - tolerance) / cap
        return fastest

    dx, dy = pose.x - robot.x, pose.y - robot.y
    distance = math.hypot(dx, dy)
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    fx, fy = ((dx * c + dy * s) / distance, (-dx * s + dy * c) / distance) if distance else (0., 0.)
    speed = min(limits.vx, limits.vy / abs(fy) if fy else math.inf)
    translation = time_to_window(distance, min(FORWARD_WINDOW_M, LATERAL_WINDOW_M),
                                 command.vx * fx + command.vy * fy, speed, limits.acceleration, KP_POSITION)
    angle = math.remainder(pose.yaw - robot.yaw, math.tau)
    turn = time_to_window(abs(angle), YAW_WINDOW_RAD, command.wz * (1 if angle >= 0 else -1),
                          limits.wz, limits.yaw_acceleration, KP_YAW)
    # Turning and walking overlap. The steering law slows translation on a curve.
    lost_travel_time = turn * (1 - max(.05, math.cos(.5 * angle)))
    return max(turn, translation + lost_travel_time)


def paw_pose(ball, goal, side, *, forward_m=PAW_FORWARD_M):
    """Place the observed ball on the chosen paw, with the swing aimed at goal."""
    yaw = math.atan2(goal[1] - ball.y, goal[0] - ball.x)
    c, s = math.cos(yaw), math.sin(yaw)
    lateral = PAW_LEFT_M[side]
    return Pose(ball.x - c * forward_m + s * lateral,
                ball.y - s * forward_m - c * lateral, yaw)


def contact_window(robot, ball, heading_error, side, lookahead=0.0, *, scale=1.0, forward_m=PAW_FORWARD_M):
    """Test the ball-centre offset in the robot's observed body frame."""
    dx = ball.x + ball.vx * lookahead - robot.x
    dy = ball.y + ball.vy * lookahead - robot.y
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    return (abs(heading_error) <= scale * YAW_WINDOW_RAD
            and abs(dx * c + dy * s - forward_m) <= scale * FORWARD_WINDOW_M
            and abs(-dx * s + dy * c - PAW_LEFT_M[side]) <= scale * LATERAL_WINDOW_M)


def _ramped_velocity(previous, desired, limits, dt):
    dx, dy = desired[0] - previous[0], desired[1] - previous[1]
    step = math.hypot(dx, dy)
    scale = min(1., limits.acceleration * dt / step) if step else 1.
    yaw_step = limits.yaw_acceleration * dt
    wz = previous[2] + max(-yaw_step, min(yaw_step, desired[2] - previous[2]))
    return previous[0] + scale * dx, previous[1] + scale * dy, wz


class PawApproach:
    def __init__(self, side, forward_m=None):
        if side not in ("left", "right"):
            raise ValueError("kick_side must be left, right or None")
        self.forward_m = PAW_FORWARD_M if forward_m is None else forward_m
        if not math.isfinite(self.forward_m) or self.forward_m <= 0:
            raise ValueError("paw forward offset must be finite and positive")
        self.assist_enabled = forward_m is not None
        self.side = side
        self.samples = deque(maxlen=128)
        self.reset()

    def reset(self):
        self.last_command = Command()
        self.last_control_time = None
        self.docked = False
        self.roll_samples = deque(maxlen=64)
        self.plan_key, self.plan, self.plan_until = None, (None, ""), -math.inf
        self.roll = None
        self.crossing = False
        self.aligned_capture = None
        self.crossing_capture = None
        self.reset_dock_progress()
        self.reset_settle()

    def _roll(self, captured, ball):
        samples = self.roll_samples
        speed = math.hypot(ball.vx, ball.vy)
        if samples:
            previous_t, previous = samples[-1]
            gap = captured - previous_t
            # A deflection ends the previous roll on its first credible new
            # velocity. Do not fit across an occlusion, reversal or kick.
            dot = previous.vx * ball.vx + previous.vy * ball.vy
            lengths = math.hypot(previous.vx, previous.vy) * speed
            reversed_position = False
            for stamp, prior in reversed(samples):
                if captured - stamp > MAX_CAPTURE_GAP_S:
                    break
                dx, dy = ball.x - prior.x, ball.y - prior.y
                moved = math.hypot(dx, dy)
                if moved >= 3 * ROLL_POSITION_FLOOR_M:
                    reversed_position = dx * ball.vx + dy * ball.vy < .5 * moved * speed
                    break
            if (not 0 <= gap <= MAX_CAPTURE_GAP_S or gap == 0 and ball != previous
                    or lengths > 0 and dot < .70 * lengths or reversed_position):
                samples.clear()
        if not samples or captured > samples[-1][0]:
            samples.append((captured, ball))
        while captured - samples[0][0] > ROLL_HISTORY_S:
            samples.popleft()
        first_t, first = samples[0]
        span = captured - first_t
        if len(samples) < 3 or span < .08 or speed < .025:
            return None
        dx, dy = ball.x - first.x, ball.y - first.y
        moved = math.hypot(dx, dy)
        if moved < 3 * ROLL_POSITION_FLOOR_M or dx * ball.vx + dy * ball.vy < .85 * moved * speed:
            return None
        ux, uy = ball.vx / speed, ball.vy / speed
        # Estimate acceleration twice. Secants of position in the first and
        # second half have midpoints span/2 apart. Velocity trend comes from
        # the camera's reports. Only corroborated slowing supplies friction.
        middle_t, middle = samples[len(samples)//2]
        def along(x, y):
            return x * ux + y * uy
        early = along(middle.x-first.x, middle.y-first.y) / (middle_t - first_t)
        late = along(ball.x-middle.x, ball.y-middle.y) / (captured - middle_t)
        position_drag = 2 * (early - late) / span
        reported_drag = (along(first.vx, first.vy) - speed) / span
        drag = 0.
        curvature = .5 * min(position_drag, reported_drag) * (middle_t - first_t) * (captured - middle_t)
        if (len(samples) >= 4 and span >= .15 and curvature >= ROLL_POSITION_FLOOR_M
                and abs(position_drag - reported_drag) <= .5 * max(position_drag, reported_drag)):
            drag = (position_drag + reported_drag) / 2
        position_error, speed_error = ROLL_POSITION_FLOOR_M, ROLL_POSITION_FLOOR_M / span
        for stamp, observed in samples:
            past = captured - stamp
            backward = past * (speed + .5 * drag * past)
            position_error = max(position_error, math.hypot(ball.x - backward * ux - observed.x,
                                                            ball.y - backward * uy - observed.y))
            speed_error = max(speed_error, math.hypot(ball.vx + drag * past * ux - observed.vx,
                                                     ball.vy + drag * past * uy - observed.vy))
        # Without an observed deceleration there is no surface coefficient to
        # assume. Limit constant-velocity pursuit to the measured interval.
        horizon = min(.8, 2 * span) if drag else min(.35, span)
        return BallRoll(speed, ux, uy, drag, horizon, position_error, speed_error)

    def _can_intercept(self, robot, ball, stance, limits, command, duration):
        """Run the actual discrete steering and ramp to the proposed encounter time."""
        yaw = abs(math.remainder(stance.yaw - robot.yaw, math.tau))
        reach_margin = math.hypot(FORWARD_WINDOW_M, LATERAL_WINDOW_M) + self.forward_m * YAW_WINDOW_RAD
        if (math.hypot(stance.x - robot.x, stance.y - robot.y) > limits.vx * duration + reach_margin
                or yaw > limits.wz * duration + YAW_WINDOW_RAD):
            return False
        velocity = command.vx, command.vy, command.wz
        for step in range(math.ceil(duration / INTERCEPT_STEP_S)):
            dt = min(INTERCEPT_STEP_S, duration - step * INTERCEPT_STEP_S)
            dx, dy = stance.x - robot.x, stance.y - robot.y
            distance = math.hypot(dx, dy)
            heading = _paw_travel_heading(dx, dy, distance, stance.yaw)
            desired = _walk_velocity(dx, dy, distance, robot.yaw, math.remainder(heading - robot.yaw, math.tau),
                                     limits, limits.vx, 0., True)
            velocity = _ramped_velocity(velocity, desired, limits, dt)
            vx, vy, wz = velocity
            c, s = math.cos(robot.yaw + wz * dt / 2), math.sin(robot.yaw + wz * dt / 2)
            robot = Pose(robot.x + dt * (vx * c - vy * s), robot.y + dt * (vx * s + vy * c),
                         robot.yaw + dt * wz)
        return contact_window(robot, ball, math.remainder(stance.yaw - robot.yaw, math.tau), self.side,
                              scale=.55, forward_m=self.forward_m)

    def intercept(self, robot, ball, goal, field, limits, captured, now):
        """Find the first kinematically reachable paw stance on a supported roll.

        Times in the roll model start at capture. The runner still grants
        motion only against the original frame. Duplicate control ticks use
        the same plan and independently ramp and expire commands.
        """
        if now < captured:
            return None, ""
        key = (captured, robot, ball, tuple(goal), field, limits)
        if self.plan_key == key:
            return self.plan if now <= self.plan_until else (None, "")
        self.plan_key = key
        roll = self.roll = self._roll(captured, ball)
        self.plan = None, ""
        self.plan_until = captured + roll.horizon if roll else captured
        age = now - captured
        if roll is None or age > roll.horizon:
            return self.plan
        command = self.last_command if now < self.last_command.expires_at else Command()
        best_gap = math.inf
        for step in range(math.ceil((roll.horizon - age) / INTERCEPT_STEP_S) + 1):
            t = min(roll.horizon, age + step * INTERCEPT_STEP_S)
            spread = roll.position_error + roll.speed_error * t
            predicted = roll.position(ball, t)
            if (spread > .5 * field.ball_radius
                    or abs(predicted.x) + field.ball_radius + spread > field.length / 2
                    or abs(predicted.y) + field.ball_radius + spread > field.width / 2
                    or math.hypot(goal[0] - predicted.x, goal[1] - predicted.y) <= spread):
                break
            pose = paw_pose(predicted, goal, self.side, forward_m=self.forward_m)
            if (abs(pose.x) + field.robot_radius + spread > field.length / 2
                    or abs(pose.y) + field.robot_radius + spread > field.width / 2):
                break
            if (spread <= min(FORWARD_WINDOW_M, LATERAL_WINDOW_M)
                    and self._can_intercept(robot, predicted, pose, limits, command, t - age)):
                self.plan = pose, "estimated moving-ball intercept"
                break
            gap = _arrival(robot, pose, limits, command) - (t - age)
            if gap < best_gap:
                best_gap = gap
                self.plan = pose, "horizon-limited moving-ball pursuit"
        if self.plan[0] is None:
            self.plan = None, "measured ball pursuit outside forecast limits"
        return self.plan

    def crossing_clearance(self, robot, ball, captured, now, yaw_error):
        """Keep a moving pass clear of the nose until it stays on the correct paw line."""
        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        lateral_error = -(ball.x - robot.x) * s + (ball.y - robot.y) * c - PAW_LEFT_M[self.side]
        lateral_speed = -ball.vx * s + ball.vy * c
        was_crossing = self.crossing
        if (self.roll is not None and now <= self.plan_until and abs(lateral_speed) >= .025
                and abs(lateral_error) > 2 * LATERAL_WINDOW_M):
            self.crossing = True
        if self.crossing:
            if (self.crossing_capture is None or not 0 <= captured - self.crossing_capture <= MAX_CAPTURE_GAP_S
                    or abs(lateral_error) > LATERAL_WINDOW_M or abs(yaw_error) > YAW_WINDOW_RAD):
                self.aligned_capture = None
            elif self.aligned_capture is None:
                self.aligned_capture = captured
            elif captured - self.aligned_capture >= MAX_CAPTURE_GAP_S:
                self.crossing = False
                self.aligned_capture = None
            self.crossing_capture = captured
        if was_crossing != self.crossing:
            self.reset_dock_progress()
        return self.crossing

    def reset_dock_progress(self):
        self.dock_anchor = None
        self.last_dock_capture = None
        self.assist_until = -math.inf

    def docking_speed_floor(self, robot, ball, distance, heading_error, captured_at, now):
        """Briefly step out of the stock gait's near-zero response after a measured stall."""
        if (not self.assist_enabled or not .006 < distance < .040 or abs(heading_error) > YAW_WINDOW_RAD
                or not 0 <= now - captured_at < MAX_CAPTURE_GAP_S or math.hypot(ball.vx, ball.vy) > .02):
            self.reset_dock_progress()
            return 0.
        if self.last_dock_capture is not None and not 0 <= captured_at - self.last_dock_capture < MAX_CAPTURE_GAP_S:
            self.reset_dock_progress()
        self.last_dock_capture = captured_at
        if self.dock_anchor is not None:
            stamp, pose = self.dock_anchor
            if (captured_at < stamp or math.hypot(robot.x - pose.x, robot.y - pose.y) >= .004
                    or abs(math.remainder(robot.yaw - pose.yaw, math.tau)) > .03):
                self.dock_anchor = None
            elif captured_at - stamp >= .55 and self.assist_until < stamp:
                # Allow one brief pulse per measured standstill. A new pulse
                # needs new motion. The speed, acceleration and expiry limits still apply.
                self.assist_until = captured_at + .16
        if self.dock_anchor is None:
            self.dock_anchor = captured_at, robot
        return .075 if captured_at < self.assist_until else 0.

    def reset_settle(self):
        self.samples.clear()

    def ramp(self, vx, vy, wz, limits, now, robot_id):
        """Bound per-second changes on the control clock, including repeated ticks."""
        previous = self.last_command
        dt = 0.0 if self.last_control_time is None else now - self.last_control_time
        if dt < 0 or now >= previous.expires_at:
            previous = Command()
            dt = 0.0
        velocity = _ramped_velocity((previous.vx, previous.vy, previous.wz), (vx, vy, wz), limits, dt)
        cmd = Command(*velocity, now + COMMAND_LEASE_S, robot_id)
        self.last_command, self.last_control_time = cmd, now
        return cmd

    def settled(self, robot, ball, captured_at, now, heading_error):
        """Check independent captures of body stability and estimated ball motion.

        A short baseline suppresses the amplified noise in frame-to-frame velocity.
        A new velocity outside the recent spread still vetoes on its first frame.
        """
        age = now - captured_at
        if not 0 <= age < MAX_OBSERVATION_AGE:
            self.reset_settle()
            return False
        sample = (captured_at, robot, ball)
        if self.samples:
            gap = captured_at - self.samples[-1][0]
            if gap < 0 or gap > MAX_CAPTURE_GAP_S or (gap == 0 and sample != self.samples[-1]):
                self.samples.clear()
        if not self.samples or captured_at > self.samples[-1][0]:
            self.samples.append(sample)
        while captured_at - self.samples[0][0] > .45:
            self.samples.popleft()
        span = captured_at - self.samples[0][0]
        # An intervening control tick can see the same exposure just after
        # its 150 ms kick deadline. It cannot authorize a strike or erase
        # the original measured history ahead of the next camera capture.
        if age >= MAX_CAPTURE_GAP_S or span < SETTLE_S:
            return False
        for _, pose, b in self.samples:
            if (math.hypot(robot.x - pose.x, robot.y - pose.y) > 0.009
                    or abs(math.remainder(robot.yaw - pose.yaw, math.tau)) > .03
                    or math.hypot(ball.x - b.x, ball.y - b.y) > 0.008):
                return False

        horizon = now - captured_at + WINDUP_S

        def will_stay(vx, vy):
            return contact_window(robot, Ball(ball.x, ball.y, vx, vy), heading_error, self.side, horizon,
                                  forward_m=self.forward_m)

        # Fit a line to positions and average the camera track's velocity reports
        # over elapsed capture time. Both forecasts must stay on the paw.
        mean_t = sum(t - captured_at for t, _, _ in self.samples) / len(self.samples)
        total = vx = vy = reported_vx = reported_vy = 0.
        previous_t = self.samples[0][0]
        for t, _, b in self.samples:
            offset = (t - captured_at) - mean_t
            total += offset * offset
            vx += offset * (b.x - ball.x)
            vy += offset * (b.y - ball.y)
            reported_vx += (t - previous_t) * b.vx
            reported_vy += (t - previous_t) * b.vy
            previous_t = t
        if not will_stay(vx / total, vy / total) or not will_stay(reported_vx / span, reported_vy / span):
            return False
        if will_stay(ball.vx, ball.vy):
            return True
        # Differentiate jitter already present in finite-difference velocities
        # from an abrupt roll on the latest observation.
        prior = [b for t, _, b in self.samples if t < captured_at]
        mean_x = sum(b.vx for b in prior) / len(prior)
        mean_y = sum(b.vy for b in prior) / len(prior)
        spread_sq = sum((b.vx - mean_x) ** 2 + (b.vy - mean_y) ** 2 for b in prior) / len(prior)
        return (ball.vx - mean_x) ** 2 + (ball.vy - mean_y) ** 2 <= 4 * spread_sq
