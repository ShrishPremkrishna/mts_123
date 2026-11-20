"""Offline approach-and-shot gameplay for one robot; observation in, command out.

Each ``tick`` reads one ``Observation`` and returns a ``Decision`` whose command
holds body-frame velocities inside ``Limits`` and expires at most 0.15 s after
``now``. Strike commands also expire with their supporting capture.
The controller never reads simulator truth and never performs I/O. Stop and
emergency handling belong to the runtime, not here.

Plan, recomputed from the observed ball every tick:

1. Shot line: from the ball to the opponent goal centre
   ``(attack_sign * length / 2, 0)``. The staging point sits on that line
   behind the ball with the bumper ``STRIKE_GAP`` short of contact. Near a
   wall the gap shrinks, never below ``MIN_STAGING_GAP``, until the robot body
   fits inside the field. If no such point exists the controller reports
   ``blocked`` with a zero command: there is no wall-extraction planner, so an
   operator must move the ball or reset the scenario.
2. approach: drive to the staging point. If the straight path would brush the
   ball (robot on the goal side), use one persistent waypoint first: straight
   away from the ball when closer than ``SIDE_OFFSET``, otherwise abeam of the
   ball on the robot's own side of the shot line. Both keep the body outside
   the clearance circle. The waypoint persists until reached, until the path
   clears or until the ball moves.
3. align: near the staging point, hold position and face the shot heading until
   distinct capture timestamps confirm alignment over ``ALIGN_DWELL`` seconds.
4. strike: forward only (vy = wz = 0), ended by distance, time, heading drift,
   the ball no longer being ahead, or the ball leaving the frontal corridor
   (a sideways jump; a struck ball rolling away ahead stays valid). Commands
   expire with their evidence, and the projected run plus a configurable
   stopping allowance must fit the robot body inside the field.
5. reassess: zero command for ``REASSESS_TIME``, then plan again. A miss is
   retried simply because the ball is still there.

Phases: approach, align, strike, reassess, blocked, return_home, await_reset,
tracking_lost (stale packet, or no robot/ball in view), recovery_required
(fallen; zero command and ``recovery_requested``; no get-up motion is sent),
invalid_observation (non-finite, wrongly typed or implausible input).

A referee goal (+1 or -1) latches return_home to
``(-attack_sign * 0.3 * length, 0)``, facing the opponent goal, then
await_reset. The latch releases only after ``reset()`` has been called and the
observation reports ``goal == 0``. Returning home needs the robot pose only.

Every tunable below is a sandbox assumption, not a measured Pupper value.
"""
import math
from dataclasses import replace

from .model import Ball, Command, Decision, Field, Limits, Observation, Pose, zero

# Freshness: a packet older than this (now - captured_at) yields a zero command.
# A frozen camera repeats its timestamp, so it trips the same check.
MAX_OBSERVATION_AGE = 0.35
MAX_STRIKE_OBSERVATION_AGE = MAX_OBSERVATION_AGE  # same evidence contract for every phase
FUTURE_TOLERANCE = 0.05     # s a capture time may lead the controller clock
FIELD_MARGIN = 0.5          # m outside the field lines still accepted as plausible
MAX_BALL_SPEED = 10.0       # m/s; faster reports are treated as corrupt

KP_POSITION = 1.5           # 1/s; slows inside ~0.43 m at the default speed limit
KP_YAW = 2.0                # 1/s
FACE_TRAVEL_DISTANCE = 0.5  # m; farther than this, face the direction of travel

STRIKE_GAP = 0.20           # m between bumper and ball surface at the staging point
MIN_STAGING_GAP = 0.05      # m; shortest gap accepted when a wall is in the way
STANDOFF_STEPS = 3          # gap values tried between STRIKE_GAP and MIN_STAGING_GAP
CLEARANCE_MARGIN = 0.06     # m kept between robot body and ball while approaching
SIDE_OFFSET = 0.50          # m from ball centre to the abeam waypoint
BACK_OFF_RADIUS = SIDE_OFFSET + 0.10  # m from the ball when backing straight away
WAYPOINT_RADIUS = 0.10      # m; a waypoint this close counts as reached
REPLAN_DISTANCE = 0.05      # m of ball movement that resets waypoint and dwell

ALIGN_RADIUS = 0.25         # m from the staging point where approach becomes align
ALIGN_SPEED = 0.25          # m/s translation cap while aligning
HEADING_TOLERANCE = 0.08    # rad
LATERAL_TOLERANCE = 0.035   # m ball offset from the bumper centre line
GAP_TOLERANCE = 0.08        # m the bumper gap may exceed STRIKE_GAP
MIN_GAP = 0.01              # m; never start a strike already touching the ball
ALIGN_DWELL = 0.25          # s the alignment must hold before a strike

FOLLOW_THROUGH = 0.20       # m driven past the contact point
STRIKE_DISTANCE = STRIKE_GAP + FOLLOW_THROUGH
STRIKE_TIMEOUT = 1.5        # s
STRIKE_ABORT_HEADING = 0.35  # rad of drift that ends a strike
STRIKE_CORRIDOR = 0.15      # m half-width ahead of the bumper the ball must stay in
REASSESS_TIME = 0.4         # s of zero command after a strike
STRIKE_STOPPING_ALLOWANCE = 0.10  # m; sandbox allowance, not measured braking

HOME_FRACTION = 0.3         # home x = -attack_sign * HOME_FRACTION * length
HOME_TOLERANCE = 0.06       # m
HOME_HEADING_TOLERANCE = 0.1  # rad


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _clip(value, low, high):
    return max(low, min(high, value))


def _finite(*values):
    try:
        return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                   for v in values)
    except OverflowError:  # JSON permits integers too large for a float.
        return False


def _segment_distance(ax, ay, bx, by, px, py):
    """Distance from point p to the segment a-b."""
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    t = _clip(((px - ax) * dx + (py - ay) * dy) / length_squared, 0.0, 1.0) if length_squared else 0.0
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def defensive_clearance(field):
    """Ball exclusion radius enclosing the body and frontal bumper, plus margin."""
    return max(field.robot_radius, field.bumper_offset) + field.ball_radius + CLEARANCE_MARGIN


class Controller:
    """Steer to a field pose, with an optional foot-specific approach and handoff."""

    def __init__(self, field: Field = Field(), limits: Limits = Limits(), *,
                 strike_stopping_allowance: float = STRIKE_STOPPING_ALLOWANCE,
                 kick_side: str | None = None):
        if not _finite(strike_stopping_allowance) or strike_stopping_allowance < 0:
            raise ValueError("strike stopping allowance must be finite and nonnegative")
        self.field, self.limits = field, limits
        self.strike_stopping_allowance = strike_stopping_allowance
        self.kick_side = kick_side
        self._paw = None
        if kick_side is not None:
            # Stock releases intentionally package only their pinned walking sources.
            from .paw_approach import PawApproach
            self._paw = PawApproach(kick_side)
        self._goal_latched = False
        self._home_reached = False
        self._reset_requested = False
        self._target_key = None
        self._clear_play()

    def reset(self):
        """Forget the current plan and allow a goal latch to release.

        The latch itself releases on the next valid tick that reports goal == 0.
        """
        self._clear_play()
        self._reset_requested = True

    def cancel(self):
        """Cancel current motion intent without releasing the referee goal latch."""
        self._clear_play()
        self._target_key = None

    def referee_active(self, goal):
        """Whether this goal flag requires return/reset instead of a new action.

        A requested reset with a cleared flag permits ordinary action selection;
        tick still validates the observation before actually releasing the latch.
        """
        return bool(goal or (self._goal_latched and not self._reset_requested))

    def record_goal(self, goal):
        """Remember an admitted referee pulse without issuing motion or reset.

        Packet admission owns observation validation. Cleared flags do not
        release the latch here: reset still requires a valid controller tick.
        """
        if type(goal) is not int or goal not in (-1, 0, 1):
            raise ValueError("goal flag must be the integer -1, 0 or 1")
        if goal:
            self._update_goal_latch(goal)

    def resume_after_goal(self, observation, now):
        """Consume one deliberate resume cue at home without producing motion.

        The runtime must separately validate session and command authority.
        Unlike reset(), this operation cannot leave a pending reset request.
        """
        self.check_resume_after_goal(observation, now)
        self.reset()
        self._update_goal_latch(0)

    def check_resume_after_goal(self, observation, now):
        """Raise when goal resume is unavailable, without changing control state."""
        if not self._goal_latched or not self._home_reached:
            raise ValueError("goal return is not complete")
        if (not isinstance(observation, Observation) or not _finite(now, observation.captured_at)
                or not 0 <= now - observation.captured_at < MAX_OBSERVATION_AGE):
            raise ValueError("resume requires a fresh observation")
        if type(observation.goal) is not int or observation.goal != 0:
            raise ValueError("resume requires a cleared goal flag")
        if type(observation.fallen) is not bool or observation.fallen:
            raise ValueError("resume requires an unfallen robot")
        if observation.robot is None or self.observation_problem(observation):
            raise ValueError("resume requires a valid robot pose and state")
        robot = observation.robot
        home_x = -self.field.attack_sign * HOME_FRACTION * self.field.length
        heading = math.atan2(0.0, self.field.attack_sign)
        if (math.hypot(robot.x - home_x, robot.y) > HOME_TOLERANCE
                or abs(_wrap(robot.yaw - heading)) > HOME_HEADING_TOLERANCE):
            raise ValueError("resume requires the observed home position and heading")

    def observation_problem(self, observation):
        """Shared state plausibility check for admission and controller ticks."""
        return self._pose_problem(observation.robot) or self._ball_problem(observation.ball)

    def _clear_play(self):
        if self._paw is not None:
            self._paw.reset()
        self._ball_anchor = None    # ball position the waypoint and dwell refer to
        self._waypoint = None       # ((x, y), description) while getting around the ball
        self._aligned_since = None
        self._aligned_through = None
        self._strike = None         # (start x, start y, heading, start time)
        self._reassess_until = None
        self._guard_waypoint = None
        self._guard_side = None

    def _halt(self, phase, reason, now, robot_id, recovery=False):
        self._clear_play()
        return Decision(phase, reason, zero(now, robot_id), recovery_requested=recovery)

    def tick(self, observation: Observation, now: float, *, target_pose: Pose | None = None,
             shot_target: tuple[float, float] | None = None,
             preserve_target_heading: bool = False, avoid_ball: bool = False) -> Decision:
        """Choose a command; optional field targets replace the ordinary attack.

        ``target_pose`` navigates without requiring a visible ball. ``shot_target``
        changes the shot destination, retaining the existing approach/strike.
        ``preserve_target_heading`` keeps a pose target's yaw even on long moves
        (e.g. facing the ball while guarding); ordinary moves face their travel.
        ``avoid_ball`` explicitly selects ball-clear defensive transit and requires
        a visible ball. Ordinary pose movement remains independent of the ball.
        Changing either target cancels an old strike. Stop/arming remain runtime
        responsibilities; these arguments do not provide transport or authority.
        """
        if not _finite(now):
            # No finite expiry exists; 0.0 is already expired on the simulation clock.
            self._clear_play()
            return Decision("invalid_observation", "controller clock is not finite", Command())
        if not isinstance(observation, Observation) or not isinstance(observation.robot_id, str):
            return self._halt("invalid_observation", "observation is missing or malformed", now, "pupper-1")
        robot_id = observation.robot_id
        if (type(preserve_target_heading) is not bool
                or (preserve_target_heading and target_pose is None)):
            return self._halt("invalid_intent", "heading preservation requires a pose target and boolean option",
                              now, robot_id)
        if type(avoid_ball) is not bool or (avoid_ball and target_pose is None):
            return self._halt("invalid_intent", "ball avoidance requires a pose target and boolean option",
                              now, robot_id)
        if target_pose is not None and shot_target is not None:
            return self._halt("invalid_intent", "choose a pose or a shot target", now, robot_id)
        if target_pose is not None:
            if (not isinstance(target_pose, Pose)
                    or not _finite(target_pose.x, target_pose.y, target_pose.yaw)):
                return self._halt("invalid_intent", "target pose is not finite", now, robot_id)
            if self._reachable(target_pose.x, target_pose.y) != (target_pose.x, target_pose.y):
                return self._halt("blocked", "target pose leaves the robot outside the field", now, robot_id)
            # A reactive guard keeps its route while its facing yaw changes.
            # Ball movement and segment validation govern defensive replanning;
            # explicit intent replacement still cancels through the Executor.
            if avoid_ball:
                target_key = ("guard", preserve_target_heading)
            elif self._paw is not None:
                # Moving waypoints replace the current steering reference in place.
                # Cancellation or changing action owners separately resets the ramp.
                target_key = ("pose", preserve_target_heading)
            else:
                target_key = ("pose", target_pose, preserve_target_heading)
        elif shot_target is not None:
            if (not isinstance(shot_target, (tuple, list)) or len(shot_target) != 2
                    or not _finite(*shot_target) or not self._plausible(*shot_target)):
                return self._halt("invalid_intent", "shot target is invalid or outside the field", now, robot_id)
            target_key = ("paw_shot",) if self._paw else ("shot", tuple(shot_target))
        else:
            target_key = None
        if target_key != self._target_key:
            self._clear_play()
            self._target_key = target_key
        if not _finite(observation.captured_at):
            return self._halt("invalid_observation", "capture time is not finite", now, robot_id)
        age = now - observation.captured_at
        if age < -FUTURE_TOLERANCE:
            return self._halt("invalid_observation", "capture time is ahead of the clock", now, robot_id)
        if age > MAX_OBSERVATION_AGE:
            return self._halt("tracking_lost", f"observation is {age:.2f} s old", now, robot_id)
        if type(observation.goal) is not int or observation.goal not in (-1, 0, 1):
            return self._halt("invalid_observation", "goal flag is not the integer -1, 0 or 1", now, robot_id)
        if type(observation.fallen) is not bool:
            return self._halt("invalid_observation", "fallen flag is not a boolean", now, robot_id)

        robot, ball = observation.robot, observation.ball
        problem = self.observation_problem(observation)
        if not problem:
            self._update_goal_latch(observation.goal)
        if observation.fallen:
            return self._halt("recovery_required", "robot reported fallen", now, robot_id, recovery=True)
        if problem:
            return self._halt("invalid_observation", problem, now, robot_id)
        if robot is None:
            return self._halt("tracking_lost", "robot is not in view", now, robot_id)
        if self._goal_latched:
            return self._return_home(robot, now, robot_id)
        if target_pose is not None:
            if avoid_ball:
                if ball is None:
                    return self._halt("tracking_lost", "defense requires a visible ball", now, robot_id)
                return self._guard_transit(robot, ball, target_pose, now, robot_id)
            target = (target_pose.x, target_pose.y)
            if (math.hypot(target_pose.x - robot.x, target_pose.y - robot.y) <= HOME_TOLERANCE
                    and abs(_wrap(target_pose.yaw - robot.yaw)) <= HOME_HEADING_TOLERANCE):
                if self._paw:
                    cmd = self._paw.ramp(0, 0, 0, self.limits, now, robot_id)
                    if cmd.vx or cmd.vy or cmd.wz:
                        return Decision("move", "braking at the requested field pose", cmd, target, target_pose.yaw)
                return Decision("target_reached", "at the requested field pose", zero(now, robot_id),
                                target, target_pose.yaw)
            return self._drive("move", "moving to the requested field pose", robot, target,
                               target_pose.yaw if preserve_target_heading else
                               self._travel_heading(robot, target, target_pose.yaw),
                               self.limits.vx, now, robot_id)
        if ball is None:
            return self._halt("tracking_lost", "ball is not in view", now, robot_id)
        if shot_target is not None and math.hypot(shot_target[0] - ball.x, shot_target[1] - ball.y) <= 1e-6:
            return self._halt("invalid_intent", "shot destination coincides with the ball", now, robot_id)
        if self._paw is not None:
            goal = shot_target or (self.field.attack_sign * self.field.length / 2, 0.0)
            if math.hypot(goal[0] - ball.x, goal[1] - ball.y) <= 1e-6:
                return self._halt("blocked", "ball is already at the shot destination", now, robot_id)
            return self._paw_attack(robot, ball, goal, observation.captured_at, now, robot_id)
        return self._play(robot, ball, now, robot_id, observation.captured_at, shot_target)

    def _guard_transit(self, robot, ball, target_pose, now, robot_id):
        """Reach a clear guard pose using validated radial/circular segments.

        This is deliberately separate from attack's fixed abeam waypoint: its
        chord guarantee does not enclose the larger defensive bumper clearance.
        There is no obstacle map or deep-ball extraction policy here.
        """
        clearance = defensive_clearance(self.field)
        target = (target_pose.x, target_pose.y)
        distance = math.hypot(robot.x - ball.x, robot.y - ball.y)
        if math.hypot(target[0] - ball.x, target[1] - ball.y) < clearance - 1e-9:
            return self._halt("blocked", "guard target is occupied by the ball clearance", now, robot_id)
        if (self._ball_anchor is None or
                math.hypot(ball.x - self._ball_anchor[0], ball.y - self._ball_anchor[1]) > REPLAN_DISTANCE):
            self._ball_anchor = (ball.x, ball.y)
            self._guard_waypoint = self._guard_side = None

        def segment_clear(point):
            return (_segment_distance(robot.x, robot.y, point[0], point[1], ball.x, ball.y)
                    >= clearance - 1e-9)

        if segment_clear(target):
            self._guard_waypoint = None
            if (math.hypot(target[0] - robot.x, target[1] - robot.y) <= HOME_TOLERANCE
                    and abs(_wrap(target_pose.yaw - robot.yaw)) <= HOME_HEADING_TOLERANCE):
                return Decision("target_reached", "at the ball-clear guard pose", zero(now, robot_id),
                                target, target_pose.yaw)
            return self._drive("move", "moving to the ball-clear guard pose", robot, target,
                               target_pose.yaw, self.limits.vx, now, robot_id)

        # A radial back-off may begin inside the margin, but must never get
        # closer to the ball. All circular segments require full clearance.
        # A small circumscribed route fits behind reachable deep guard poses.
        # Attack's .50 m waypoint and .10 m arrival tolerance do not provide
        # this clearance; validate every actual segment even on this ring.
        waypoint_tolerance = 0.01
        angular_step = math.pi / 6
        ring = (clearance + 2 * waypoint_tolerance) / math.cos(angular_step / 2)
        if self._guard_waypoint is not None:
            point, backing = self._guard_waypoint
            reached = math.hypot(point[0] - robot.x, point[1] - robot.y) <= waypoint_tolerance
            safe = segment_clear(point)
            if backing:
                safe = (_segment_distance(robot.x, robot.y, *point, ball.x, ball.y)
                        >= distance - 1e-9 and math.hypot(point[0] - ball.x, point[1] - ball.y) > distance)
            if reached or not safe or self._reachable(*point) != point:
                self._guard_waypoint = None
        if self._guard_waypoint is None:
            if distance < ring - waypoint_tolerance:
                if distance <= 1e-9:
                    return self._halt("blocked", "ball and robot coincide; no clear defense departure", now, robot_id)
                dx, dy = robot.x - ball.x, robot.y - ball.y
                candidates = [(ball.x + dx * ring / distance, ball.y + dy * ring / distance)]
                # Near a boundary the radial endpoint may not fit. A bounded
                # field-axis departure can still increase clearance; never
                # clip an unchecked segment toward the ball.
                xoff = math.sqrt(max(0.0, ring * ring - dy * dy))
                yoff = math.sqrt(max(0.0, ring * ring - dx * dx))
                candidates.extend((ball.x + side * xoff, robot.y)
                                  for side in (self.field.attack_sign, -self.field.attack_sign))
                candidates.extend((robot.x, ball.y + side * yoff) for side in (1, -1))
                point = next((candidate for candidate in candidates
                              if self._reachable(*candidate) == candidate
                              and math.hypot(candidate[0] - ball.x, candidate[1] - ball.y) >= ring - 1e-9
                              and _segment_distance(robot.x, robot.y, *candidate, ball.x, ball.y)
                              >= distance - 1e-9), None)
                if point is None:
                    return self._halt("blocked", "no ball-clear defensive back-off fits inside the field", now, robot_id)
                self._guard_waypoint = (point, True)
            else:
                angle = math.atan2(robot.y - ball.y, robot.x - ball.x)
                destination = math.atan2(target[1] - ball.y, target[0] - ball.x)
                delta = _wrap(destination - angle)
                preferred = (1 if delta > 0 else -1) if abs(abs(delta) - math.pi) > 1e-9 else self.field.attack_sign
                sides = (self._guard_side,) if self._guard_side is not None else (preferred, -preferred)
                for side in sides:
                    next_angle = angle + side * min(angular_step, abs(delta))
                    point = (ball.x + ring * math.cos(next_angle), ball.y + ring * math.sin(next_angle))
                    if self._reachable(*point) == point and segment_clear(point):
                        self._guard_waypoint, self._guard_side = (point, False), side
                        break
                if self._guard_waypoint is None:
                    return self._halt("blocked", "no ball-clear defensive waypoint fits inside the field", now, robot_id)
        point, backing = self._guard_waypoint
        # A safe initial side/back contact gap can be smaller than the bumper's
        # swept radius. Retreat without turning until that whole radius clears.
        heading = robot.yaw if backing and distance < clearance else target_pose.yaw
        return self._drive("move", "backing away from the ball for defense" if backing else
                           "circling the ball to the guard pose", robot, point, heading,
                           self.limits.vx, now, robot_id)

    def _pose_problem(self, robot):
        if robot is None:
            return None
        if not isinstance(robot, Pose) or not _finite(robot.x, robot.y, robot.yaw):
            return "robot pose is not finite"
        if not self._plausible(robot.x, robot.y):
            return "robot position is implausibly far outside the field"
        return None

    def _ball_problem(self, ball):
        if ball is None:
            return None
        if not isinstance(ball, Ball) or not _finite(ball.x, ball.y, ball.vx, ball.vy):
            return "ball state is not finite"
        if not self._plausible(ball.x, ball.y):
            return "ball position is implausibly far outside the field"
        if math.hypot(ball.vx, ball.vy) > MAX_BALL_SPEED:
            return "ball speed is implausible"
        return None

    def _plausible(self, x, y):
        return (abs(x) <= self.field.length / 2 + FIELD_MARGIN
                and abs(y) <= self.field.width / 2 + FIELD_MARGIN)

    def _update_goal_latch(self, goal):
        if goal:
            if not self._goal_latched:
                self._goal_latched = True
                self._home_reached = False
                self._reset_requested = False
                self._clear_play()
        elif self._goal_latched and self._reset_requested:
            self._goal_latched = False
            self._home_reached = False
        if not self._goal_latched:
            self._reset_requested = False

    def _reachable(self, x, y):
        """Clamp a robot-centre target to where the body stays inside the field."""
        xmax = self.field.length / 2 - self.field.robot_radius
        ymax = self.field.width / 2 - self.field.robot_radius
        return _clip(x, -xmax, xmax), _clip(y, -ymax, ymax)

    def _drive(self, phase, reason, robot, target, heading, max_speed, now, robot_id):
        """Proportional move to a field point while turning to a field heading."""
        dx, dy = target[0] - robot.x, target[1] - robot.y
        distance = math.hypot(dx, dy)
        heading_error = _wrap(heading - robot.yaw)
        vx = vy = 0.0
        if distance > 1e-9:
            speed = min(max_speed, KP_POSITION * distance)
            if self._paw is not None:
                # Leave room to turn on a tight curve; full speed is available on line.
                speed *= max(0.05, math.cos(heading_error))
            c, s = math.cos(robot.yaw), math.sin(robot.yaw)
            vx = speed * (dx * c + dy * s) / distance
            vy = speed * (-dx * s + dy * c) / distance
            # Scale both axes together so the direction of travel is preserved.
            scale = min(1.0, self.limits.vx / abs(vx) if vx else 1.0,
                        self.limits.vy / abs(vy) if vy else 1.0)
            vx, vy = vx * scale, vy * scale
        wz = _clip(KP_YAW * heading_error, -self.limits.wz, self.limits.wz)
        cmd = (self._paw.ramp(vx, vy, wz, self.limits, now, robot_id) if self._paw is not None else
               Command(vx, vy, wz, now + 0.15, robot_id))
        return Decision(phase, reason, cmd, target, heading, distance, heading_error)

    def _travel_heading(self, robot, target, near_heading):
        dx, dy = target[0] - robot.x, target[1] - robot.y
        if self._paw is not None:
            # Blend over a band of remaining distance instead of switching yaw at one radius.
            w = _clip((math.hypot(dx, dy) - 0.14) / FACE_TRAVEL_DISTANCE, 0.0, 1.0)
            w *= w * (3.0 - 2.0 * w)
            return _wrap(near_heading + w * _wrap(math.atan2(dy, dx) - near_heading))
        if math.hypot(dx, dy) > FACE_TRAVEL_DISTANCE:
            return math.atan2(dy, dx)
        return near_heading

    def _return_home(self, robot, now, robot_id):
        field = self.field
        home = (-field.attack_sign * HOME_FRACTION * field.length, 0.0)
        facing = math.atan2(0.0, field.attack_sign * field.length / 2 - home[0])
        distance = math.hypot(home[0] - robot.x, home[1] - robot.y)
        heading_error = _wrap(facing - robot.yaw)
        if distance <= HOME_TOLERANCE and abs(heading_error) <= HOME_HEADING_TOLERANCE:
            self._home_reached = True
        if self._home_reached:
            return Decision("await_reset", "home after goal; waiting for reset and a cleared goal flag",
                            zero(now, robot_id), home, facing, distance, heading_error)
        return self._drive("return_home", "goal recorded; returning to home position", robot, home,
                           self._travel_heading(robot, home, facing), self.limits.vx, now, robot_id)

    def _paw_attack(self, robot, ball, goal, captured_at, now, robot_id):
        from .paw_approach import MAX_CAPTURE_GAP_S, PAW_FORWARD_M, contact_window, paw_pose
        pose = paw_pose(ball, goal, self.kick_side)
        target = (pose.x, pose.y)
        if self._reachable(*target) != target:
            return self._halt("blocked", "selected paw position lies outside the field", now, robot_id)
        heading_error = _wrap(pose.yaw - robot.yaw)
        # Enter near the middle of the contact window; hysteresis leaves room for
        # capture noise while braking and waiting for evidence of steady placement.
        self._paw.docked = contact_window(robot, ball, heading_error, self.kick_side,
                                          scale=1.0 if self._paw.docked else 0.55)
        if self._paw.docked:
            cmd = self._paw.ramp(0, 0, 0, self.limits, now, robot_id)
            # The swing is committed for the wind-up. A rolling ball that will leave
            # the paw line cannot authorize a kick, even when aligned at this instant.
            ready = False
            if cmd.vx or cmd.vy or cmd.wz:
                self._paw.reset_settle()
            else:
                ready = self._paw.settled(robot, ball, captured_at, now, heading_error)
            if ready:
                cmd = replace(cmd, expires_at=min(cmd.expires_at, captured_at + MAX_CAPTURE_GAP_S))
            return Decision("kick_ready" if ready else "align",
                            f"{self.kick_side} paw ready" if ready else "confirming a steady paw and ball",
                            cmd, target, pose.yaw, math.hypot(pose.x - robot.x, pose.y - robot.y), heading_error)
        self._paw.reset_settle()
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        dx, dy = ball.x - robot.x, ball.y - robot.y
        along = dx * c + dy * s
        across = -dx * s + dy * c
        # Route around the ball to a point safely behind the stance. From the
        # back half-plane, close only on the paw's side of the ball.
        if (along < PAW_FORWARD_M - 0.025
                or (along < PAW_FORWARD_M + 0.09 and abs(across) > 0.12)
                or (along < PAW_FORWARD_M + 0.16 and abs(heading_error) > 0.30)):
            margin = defensive_clearance(self.field)
            staging = Pose(pose.x - c * margin, pose.y - s * margin, pose.yaw)
            if self._reachable(staging.x, staging.y) != (staging.x, staging.y):
                return self._halt("blocked", "no ball-clear staging pose fits behind the ball", now, robot_id)
            routed = self._guard_transit(robot, ball, staging, now, robot_id)
            if routed.phase == "move":
                return replace(routed, phase="approach", reason="routing behind the ball for the selected paw")
            return routed
        self._guard_waypoint = self._guard_side = None
        distance = math.hypot(pose.x - robot.x, pose.y - robot.y)
        return self._drive("align" if distance < ALIGN_RADIUS else "approach",
                           f"steering to the {self.kick_side} paw position", robot, target,
                           self._travel_heading(robot, target, pose.yaw), self.limits.vx, now, robot_id)

    def _play(self, robot, ball, now, robot_id, captured_at, shot_target=None):
        if self._strike is not None:
            return self._continue_strike(robot, ball, now, robot_id, captured_at)
        if self._reassess_until is not None:
            if now < self._reassess_until:
                return Decision("reassess", "braking after strike", zero(now, robot_id))
            self._reassess_until = None

        note = ""
        if (self._ball_anchor is None
                or math.hypot(ball.x - self._ball_anchor[0], ball.y - self._ball_anchor[1]) > REPLAN_DISTANCE):
            if self._ball_anchor is not None:
                note = "ball moved, replanned; "
            self._ball_anchor = (ball.x, ball.y)
            self._waypoint = None
            self._aligned_since = None

        field = self.field
        plan = self._staging_point(ball, shot_target)
        if plan is None:
            self._clear_play()
            return Decision("blocked", "no goal-directed spot behind the ball fits inside the field; "
                            "move the ball or reset", zero(now, robot_id))
        stage, heading, ux, uy = plan

        # The straight path would bring the body closer to the ball than the
        # clearance, and closer than it already is (backing away is allowed).
        ball_distance = math.hypot(ball.x - robot.x, ball.y - robot.y)
        clearance = field.robot_radius + field.ball_radius + CLEARANCE_MARGIN
        path_distance = _segment_distance(robot.x, robot.y, stage[0], stage[1], ball.x, ball.y)
        if path_distance < clearance and path_distance < ball_distance - 0.01:
            self._aligned_since = None
            if (self._waypoint is not None and math.hypot(self._waypoint[0][0] - robot.x,
                                                          self._waypoint[0][1] - robot.y) <= WAYPOINT_RADIUS):
                self._waypoint = None
            if self._waypoint is None:
                self._waypoint = self._choose_waypoint(robot, ball, ball_distance, ux, uy)
            waypoint, description = self._waypoint
            return self._drive("approach", note + description, robot, waypoint,
                               self._travel_heading(robot, waypoint, heading),
                               self.limits.vx, now, robot_id)
        self._waypoint = None

        stage_distance = math.hypot(stage[0] - robot.x, stage[1] - robot.y)
        if stage_distance > ALIGN_RADIUS:
            self._aligned_since = None
            return self._drive("approach", f"{note}driving to the staging point behind the ball", robot,
                               stage, self._travel_heading(robot, stage, heading),
                               self.limits.vx, now, robot_id)

        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        forward = (ball.x - robot.x) * c + (ball.y - robot.y) * s
        left = -(ball.x - robot.x) * s + (ball.y - robot.y) * c
        gap = forward - field.bumper_offset - field.ball_radius
        aligned = (abs(_wrap(heading - robot.yaw)) <= HEADING_TOLERANCE
                   and abs(left) <= LATERAL_TOLERANCE
                   and MIN_GAP <= gap <= STRIKE_GAP + GAP_TOLERANCE)
        if not aligned:
            self._aligned_since = None
            return self._drive("align", f"{note}lining the bumper up behind the ball", robot, stage,
                               heading, ALIGN_SPEED, now, robot_id)
        if not captured_at <= now < captured_at + MAX_STRIKE_OBSERVATION_AGE:
            return self._halt("tracking_lost", "alignment needs fresh strike evidence", now, robot_id)
        if self._aligned_since is None or captured_at < self._aligned_through:
            self._aligned_since = captured_at
        self._aligned_through = captured_at
        if captured_at - self._aligned_since < ALIGN_DWELL:
            return self._drive("align", f"{note}aligned; confirming the ball is steady", robot, stage,
                               heading, ALIGN_SPEED, now, robot_id)
        self._aligned_since = None
        self._strike = (robot.x, robot.y, heading, now)
        return self._continue_strike(robot, ball, now, robot_id, captured_at)

    def _staging_point(self, ball, shot_target=None):
        """Point on the shot line behind the ball, or None when a wall leaves none.

        Returns ((x, y), heading, ux, uy) with (ux, uy) the unit shot direction.
        """
        field = self.field
        destination = shot_target if shot_target is not None else (field.attack_sign * field.length / 2, 0.0)
        gx, gy = destination[0] - ball.x, destination[1] - ball.y
        goal_distance = math.hypot(gx, gy)
        ux, uy = (gx / goal_distance, gy / goal_distance) if goal_distance > 1e-6 else (field.attack_sign, 0.0)
        contact = field.bumper_offset + field.ball_radius
        for i in range(STANDOFF_STEPS + 1):
            gap = STRIKE_GAP - (STRIKE_GAP - MIN_STAGING_GAP) * i / STANDOFF_STEPS
            x, y = ball.x - ux * (contact + gap), ball.y - uy * (contact + gap)
            if self._reachable(x, y) == (x, y):
                return (x, y), math.atan2(uy, ux), ux, uy
        return None

    def _choose_waypoint(self, robot, ball, ball_distance, ux, uy):
        """One point that leads the body around the ball without touching it.

        Close in, back straight away: that path only increases the distance.
        From SIDE_OFFSET or farther, the chord to a point abeam of the ball on
        the robot's own side stays at least SIDE_OFFSET / sqrt(2) from the ball.
        """
        if ball_distance < SIDE_OFFSET:
            if ball_distance > 1e-6:
                rx, ry = (robot.x - ball.x) / ball_distance, (robot.y - ball.y) / ball_distance
            else:
                rx, ry = -ux, -uy
            point = self._reachable(ball.x + rx * BACK_OFF_RADIUS, ball.y + ry * BACK_OFF_RADIUS)
            return point, "backing away from the ball"
        lateral = -(robot.x - ball.x) * uy + (robot.y - ball.y) * ux
        side = 1 if lateral >= 0 else -1

        def abeam(candidate):
            return ball.x - uy * SIDE_OFFSET * candidate, ball.y + ux * SIDE_OFFSET * candidate

        # Stay on the robot's own side unless a wall leaves room only on the other.
        if self._reachable(*abeam(side)) != abeam(side) and self._reachable(*abeam(-side)) == abeam(-side):
            side = -side
        return self._reachable(*abeam(side)), f"circling to the {'left' if side > 0 else 'right'} of the shot line"

    def _strike_path_fits(self, robot, remaining, heading_error, evidence_age):
        """Bound a straight run of the disc footprint inside the rectangular field.

        Include travel during observation age and one whole command lifetime,
        then the explicit stopping allowance. This assumes bounded commanded
        speed and the observed heading; it is not a physical braking model.
        """
        if self._reachable(robot.x, robot.y) != (robot.x, robot.y):
            return False
        travel = (remaining / math.cos(heading_error)
                  + self.limits.vx * (evidence_age + 0.15)
                  + self.strike_stopping_allowance)
        end = (robot.x + math.cos(robot.yaw) * travel,
               robot.y + math.sin(robot.yaw) * travel)
        return self._reachable(*end) == end

    def _continue_strike(self, robot, ball, now, robot_id, captured_at):
        evidence_age = now - captured_at
        if not captured_at <= now < captured_at + MAX_STRIKE_OBSERVATION_AGE:
            return self._halt("tracking_lost", "strike evidence expired", now, robot_id)
        start_x, start_y, heading, started_at = self._strike
        ux, uy = math.cos(heading), math.sin(heading)
        progress = (robot.x - start_x) * ux + (robot.y - start_y) * uy
        heading_error = _wrap(heading - robot.yaw)
        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        ball_forward = (ball.x - robot.x) * c + (ball.y - robot.y) * s
        ball_left = -(ball.x - robot.x) * s + (ball.y - robot.y) * c
        if progress >= STRIKE_DISTANCE:
            ended = "run-through complete"
        elif now - started_at >= STRIKE_TIMEOUT:
            ended = "strike timed out"
        elif abs(heading_error) > STRIKE_ABORT_HEADING:
            ended = "heading drifted during strike"
        elif ball_forward <= 0:
            ended = "ball is no longer ahead"
        elif abs(ball_left) > STRIKE_CORRIDOR:
            ended = "ball left the strike corridor"
        elif not self._strike_path_fits(robot, STRIKE_DISTANCE - progress,
                                       heading_error, evidence_age):
            ended = "strike and stopping allowance leave the field"
        else:
            target = (start_x + ux * STRIKE_DISTANCE, start_y + uy * STRIKE_DISTANCE)
            expiry = min(now + 0.15, captured_at + MAX_STRIKE_OBSERVATION_AGE)
            return Decision("strike", "running through the ball toward goal",
                            Command(self.limits.vx, 0.0, 0.0, expiry, robot_id),
                            target, heading, STRIKE_DISTANCE - progress, heading_error)
        self._clear_play()
        self._reassess_until = now + REASSESS_TIME
        return Decision("reassess", f"{ended}; braking before replanning", zero(now, robot_id))
