"""Steer to a foot-specific ball pose and measure when the walker can yield.

The common ball offset comes from Tej's u5 tool. The tolerances, braking envelope and
wind-up allowance are provisional until measured with the intended ball.
"""
import math
from collections import deque

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
        self.reset_dock_progress()
        self.reset_settle()

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
        dx, dy = vx - previous.vx, vy - previous.vy
        step = math.hypot(dx, dy)
        scale = min(1.0, limits.acceleration * dt / step) if step else 1.0
        yaw_step = limits.yaw_acceleration * dt
        cmd = Command(previous.vx + scale * dx, previous.vy + scale * dy,
                      previous.wz + max(-yaw_step, min(yaw_step, wz - previous.wz)),
                      now + 0.15, robot_id)
        self.last_command, self.last_control_time = cmd, now
        return cmd

    def settled(self, robot, ball, captured_at, now, heading_error):
        """Check independent captures of body stability and estimated ball motion.

        A short baseline suppresses the amplified noise in frame-to-frame velocity.
        A new velocity outside the recent spread still vetoes on its first frame.
        """
        if not 0 <= now - captured_at < MAX_CAPTURE_GAP_S:
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
        if span < SETTLE_S:
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

        # Fit a line to positions and average the independent velocity reports
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
