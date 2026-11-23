"""Per-robot role selection, defense and possession clearance, with no I/O.

Inputs use model.py's field frame and one caller-supplied monotonic clock.
Create a separate Tactics instance for each opponent; nothing is shared or
communicated between them. A hold preserves the role but supplies no action
target; the caller must stop/cancel current execution. Geometry comes from the
field's playable bounds and physical goal fronts, never a recentred substitute.

Role policies: ``match`` selects offense/defense with hysteresis from the
ball's position between the two physical goal fronts, plus a confirmed rolling
threat to the own mouth. ``offense`` (normal solo play) never defends.
``defense`` is the deliberate defensive experiment.

Actions:
- attack: the controller approaches and kicks the current ball. Stationary
  contact remains the only physical contact candidate.
- intercept: walk to a forecast meeting pose. Offense targets a ball-clear
  staging pose behind the predicted ball; defense stands on the incoming path.
  Requires a complete motion profile; otherwise this capability is absent.
- block: stand in front of the own mouth at the forecast crossing.
- defend: conservative goal-side guard facing the current ball.
- yield: wait at a ball-clear standoff while a peer has possession.
- hold: no target.

Possession clearance applies to offense. A peer within ``contest_radius`` of
the ball contests it. The robot nearer the ball by more than
``contest_margin`` keeps it; otherwise the lower robot ID does, so both
observers of one frame reach the same answer without communicating. A robot
that yields keeps yielding for ``yield_dwell_s`` so noise cannot alternate it.
A peer with an unresolved referee kick near the ball keeps priority. A peer
that stays static for ``peer_stall_s`` without a kick forfeits priority (it may
be held or stopped); the executor's keep-out still prevents approaching its
body, and a ball pinned against a static peer is an explicit hold. Peers are
only those present in the frame: callers must hold when a required
participant is missing rather than pass an empty peer set.

Forecasts (first section below) start at one measured capture and keep that
time, sample count, fit residual and model assumptions. ``MotionProfile``
names where each robot/floor value came from: synthetic profiles make offline
play deterministic, measured profiles require real trial evidence. A capability
whose inputs are absent is unavailable and tactics fall back to conservative
positions. The ball rolls straight and slows at a constant rate; without a
deceleration value the forecast assumes constant velocity, which over-predicts
travel and is the conservative case for defense. Robot arrival uses a
trapezoidal speed profile from rest plus a reaction delay, turning concurrently.
Both are planning approximations, not measured behavior.
"""
from collections import deque
from dataclasses import asdict, dataclass
import math

from .controller import defensive_clearance, peer_clearance
from .model import Ball, Field, Observation, Pose

ROLE_POLICIES = ("match", "offense", "defense")


def _finite(*values):
    try:
        return all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in values)
    except OverflowError:
        return False

# ---- forecast and reachability (pure) -----------------------------------------

ROBOT_INPUTS = ("speed", "turn_rate", "acceleration", "braking", "reaction_s")
CAPABILITY_INPUTS = {
    "intercept": ROBOT_INPUTS,
    "moving_contact": ROBOT_INPUTS + ("kick_delay_s",),
    "ball_deceleration": ("ball_deceleration",),
}
FORECAST_HORIZON_S = 3.0          # never extrapolate a roll further than this
CONSTANT_VELOCITY_HORIZON_S = 1.5  # without a deceleration value, trust the fit less far
INTERCEPT_MARGIN_S = 0.25         # robot must be in place this long before the ball (provisional)
INTERCEPT_STEP_S = 0.05
ARRIVED_M, ARRIVED_RAD = .06, .10  # controller.HOME_TOLERANCE / HOME_HEADING_TOLERANCE


@dataclass(frozen=True)
class MotionProfile:
    """Robot/floor values with one named source.

    ``synthetic`` values are invented for deterministic offline play; they never
    qualify a live capability. Missing values stay None rather than defaulting.
    """
    source: str
    synthetic: bool
    speed: float | None = None              # sustained planar walking speed, m/s
    turn_rate: float | None = None          # sustained yaw rate, rad/s
    acceleration: float | None = None       # planar speed increase, m/s^2
    braking: float | None = None            # planar speed decrease to a stop, m/s^2
    reaction_s: float | None = None         # command issue to measurable motion, s
    kick_delay_s: float | None = None       # kick request to paw contact, s
    ball_deceleration: float | None = None  # rolling ball slowdown on this floor, m/s^2

    def __post_init__(self):
        if not isinstance(self.source, str) or not self.source.strip() or len(self.source) > 128:
            raise ValueError("motion profile requires a named source")
        if type(self.synthetic) is not bool:
            raise ValueError("motion profile synthetic flag must be boolean")
        for key in ROBOT_INPUTS + ("kick_delay_s", "ball_deceleration"):
            value = getattr(self, key)
            if value is None:
                continue
            low_ok = value >= 0 if key in ("reaction_s", "kick_delay_s") else value > 0
            if not _finite(value) or not low_ok:
                raise ValueError(f"motion profile {key} must be finite and physically bounded")

    def missing(self, capability):
        return tuple(key for key in CAPABILITY_INPUTS[capability] if getattr(self, key) is None)

    def record(self):
        return asdict(self)


# Offline sandbox values matching kick_rehearsal's mid-speed plant and the
# simulator's 0.18 m/s^2 rolling slowdown. Invented, not measured.
SYNTHETIC_PROFILE = MotionProfile(
    "synthetic:offline-sandbox-v1", True, speed=.30, turn_rate=1.0, acceleration=.65,
    braking=.65, reaction_s=.15, kick_delay_s=2.1, ball_deceleration=.18)


@dataclass(frozen=True)
class BallForecast:
    """A straight constant-deceleration roll fitted to recent measured captures.

    Times are seconds after ``captured_at``. ``x, y`` is the fitted centre at
    that capture. ``spread(t)`` grows from the fit residual and velocity error;
    it is a fit statistic, not a validated physical error bound.
    """
    captured_at: float
    x: float
    y: float
    vx: float
    vy: float
    deceleration: float
    deceleration_source: str
    horizon_s: float
    position_sigma: float
    speed_sigma: float
    samples: int
    span_s: float

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    def stop_time(self):
        return math.inf if self.deceleration <= 0 else self.speed / self.deceleration

    def distance(self, t):
        t = max(0.0, min(t, self.stop_time()))
        return self.speed * t - .5 * self.deceleration * t * t

    def position(self, t):
        speed = self.speed
        if speed <= 0:
            return self.x, self.y
        travelled = self.distance(t)
        return self.x + self.vx / speed * travelled, self.y + self.vy / speed * travelled

    def spread(self, t):
        return self.position_sigma + self.speed_sigma * max(0.0, t)

    def crossing(self, x):
        """First (t, y) when the centre reaches the plane at x, or None if it stops first."""
        speed = self.speed
        if speed <= 0 or abs(self.vx) < 1e-9 or (x - self.x) * self.vx <= 0:
            return None
        needed = (x - self.x) * speed / self.vx  # distance along the roll
        if self.deceleration > 0:
            if needed > speed * speed / (2 * self.deceleration):
                return None
            t = (speed - math.sqrt(max(0.0, speed * speed - 2 * self.deceleration * needed))) / self.deceleration
        else:
            t = needed / speed
        return t, self.y + self.vy / speed * needed

    def record(self, now=None):
        result = {"captured_at": self.captured_at, "samples": self.samples, "span_s": round(self.span_s, 4),
                  "x": round(self.x, 4), "y": round(self.y, 4), "vx": round(self.vx, 4), "vy": round(self.vy, 4),
                  "deceleration": self.deceleration, "deceleration_source": self.deceleration_source,
                  "horizon_s": round(self.horizon_s, 4), "position_sigma": round(self.position_sigma, 4),
                  "speed_sigma": round(self.speed_sigma, 4),
                  "model": "straight roll, constant deceleration" if self.deceleration > 0
                  else "straight roll, constant velocity (deceleration unmeasured)"}
        if now is not None:
            result["age_s"] = round(now - self.captured_at, 4)
        return result


class BallTrack:
    """Least-squares fit of recent measured ball centres for one observer.

    A single velocity report never establishes motion: the camera filter's
    velocity is noisy for a still ball. Motion needs several captures, a
    minimum span and displacement well above the measurement floor. A sample
    far from the current fit (kick, bounce, relocation) restarts the history.
    Moving state has hysteresis so noise near the threshold cannot flicker.
    """

    def __init__(self, *, window_s=.50, min_samples=4, min_span_s=.15, enter_speed=.10,
                 exit_speed=.06, noise_m=.01, max_gap_s=.20, jump_m=.08):
        self.window_s, self.min_samples, self.min_span_s = window_s, min_samples, min_span_s
        self.enter_speed, self.exit_speed, self.noise_m = enter_speed, exit_speed, noise_m
        self.max_gap_s, self.jump_m = max_gap_s, jump_m
        self.samples = deque()
        self.moving = False

    def reset(self):
        self.samples.clear()
        self.moving = False

    def update(self, captured_at, x, y):
        if not all(_finite(v) for v in (captured_at, x, y)):
            self.reset()
            return
        if self.samples:
            last_t, last_x, last_y = self.samples[-1]
            if captured_at < last_t or captured_at - last_t > self.max_gap_s:
                self.reset()
            elif captured_at == last_t:
                if (x, y) != (last_x, last_y):
                    self.reset()
                else:
                    return
            elif len(self.samples) >= 3:
                fit = self._fit()
                if fit is not None:
                    fx, fy, fvx, fvy, _, _ = fit
                    dt = captured_at - last_t
                    if math.hypot(fx + fvx * dt - x, fy + fvy * dt - y) > self.jump_m:
                        self.reset()
        self.samples.append((captured_at, x, y))
        while captured_at - self.samples[0][0] > self.window_s:
            self.samples.popleft()

    def _fit(self):
        """(x, y, vx, vy, residual, Stt) at the latest capture, or None."""
        n = len(self.samples)
        if n < 2:
            return None
        latest = self.samples[-1][0]
        ts = [t - latest for t, _, _ in self.samples]
        mean_t = sum(ts) / n
        stt = sum((t - mean_t) ** 2 for t in ts)
        if stt <= 0:
            return None
        mean_x = sum(p[1] for p in self.samples) / n
        mean_y = sum(p[2] for p in self.samples) / n
        vx = sum((t - mean_t) * (p[1] - mean_x) for t, p in zip(ts, self.samples)) / stt
        vy = sum((t - mean_t) * (p[2] - mean_y) for t, p in zip(ts, self.samples)) / stt
        x0, y0 = mean_x - vx * mean_t, mean_y - vy * mean_t
        squares = sum((x0 + vx * t - p[1]) ** 2 + (y0 + vy * t - p[2]) ** 2 for t, p in zip(ts, self.samples))
        residual = math.sqrt(squares / max(1, n - 2))
        return x0, y0, vx, vy, residual, stt

    def forecast(self, profile=None):
        """A BallForecast while confidently rolling, else None (treat as stationary)."""
        n = len(self.samples)
        fit = self._fit() if n >= self.min_samples else None
        span = self.samples[-1][0] - self.samples[0][0] if self.samples else 0.
        if fit is None or span < self.min_span_s:
            self.moving = False
            return None
        x, y, vx, vy, residual, stt = fit
        sigma = max(self.noise_m, residual)
        speed_sigma = sigma / math.sqrt(stt)
        speed = math.hypot(vx, vy)
        threshold = self.exit_speed if self.moving else self.enter_speed
        self.moving = (speed >= threshold and speed >= 3 * speed_sigma
                       and speed * span >= 4 * self.noise_m)
        if not self.moving:
            return None
        decel = profile.ball_deceleration if profile is not None else None
        if decel is not None:
            source = ("synthetic " if profile.synthetic else "measured ") + profile.source
            horizon = min(FORECAST_HORIZON_S, speed / decel)
        else:
            decel, source = 0.0, "unmeasured: constant velocity"
            horizon = CONSTANT_VELOCITY_HORIZON_S
        return BallForecast(self.samples[-1][0], x, y, vx, vy, decel, source, horizon,
                            sigma, speed_sigma, n, span)


def travel_time(distance, speed, acceleration, braking):
    """Trapezoidal rest-to-rest time over a straight distance."""
    if distance <= 0:
        return 0.0
    ramp = speed * speed / (2 * acceleration) + speed * speed / (2 * braking)
    if distance >= ramp:
        return speed / acceleration + speed / braking + (distance - ramp) / speed
    peak = math.sqrt(2 * distance * acceleration * braking / (acceleration + braking))
    return peak / acceleration + peak / braking


def arrival_time(profile, robot, target):
    """Planning estimate of seconds for a robot pose to reach and settle at target.

    Omnidirectional walking translates and turns concurrently. Starting from
    rest is conservative while the robot already walks toward the target.
    """
    if profile is None or profile.missing("intercept"):
        raise ValueError("arrival estimate requires a complete intercept profile")
    distance = math.hypot(target.x - robot.x, target.y - robot.y)
    angle = abs(math.remainder(target.yaw - robot.yaw, math.tau))
    if distance <= ARRIVED_M and angle <= ARRIVED_RAD:
        return 0.0  # already within the controller's arrival tolerance; it holds still
    turn = angle / profile.turn_rate
    return profile.reaction_s + max(travel_time(distance, profile.speed, profile.acceleration, profile.braking), turn)


@dataclass(frozen=True)
class Intercept:
    t: float           # seconds after the forecast capture when the ball reaches the point
    point: tuple       # predicted ball centre
    pose: object       # robot pose that meets it
    arrival_s: float   # estimated robot arrival after now
    spread: float      # forecast spread at t

    def record(self):
        return {"t_after_capture_s": round(self.t, 4), "ball": [round(v, 4) for v in self.point],
                "pose": [round(self.pose.x, 4), round(self.pose.y, 4), round(self.pose.yaw, 4)],
                "arrival_s": round(self.arrival_s, 4), "spread_m": round(self.spread, 4)}


def first_intercept(forecast, robot, now, profile, candidate, *, until=None, max_spread=.08,
                    lead=lambda t: 0.0):
    """Earliest forecast time whose candidate pose the robot reaches in time.

    ``candidate(point, t)`` returns a pose or None for an unusable point.
    ``lead(t)`` subtracts any earlier contact time (e.g. the body radius).
    The search stops once the forecast spread exceeds ``max_spread``.
    """
    age = now - forecast.captured_at
    if age < 0:
        return None
    end = min(forecast.horizon_s, forecast.stop_time(), math.inf if until is None else until)
    steps = math.floor(max(0.0, end - age) / INTERCEPT_STEP_S + 1e-9)
    for k in range(steps + 1):
        t = age + k * INTERCEPT_STEP_S
        spread = forecast.spread(t)
        if spread > max_spread:
            return None
        point = forecast.position(t)
        pose = candidate(point, t)
        if pose is None:
            continue
        arrival = arrival_time(profile, robot, pose)
        if age + arrival + INTERCEPT_MARGIN_S <= t - lead(t):
            return Intercept(t, point, pose, arrival, spread)
    return None


# ---- tactics -------------------------------------------------------------------


@dataclass(frozen=True)
class TacticsConfig:
    # Fractions of the distance between goal fronts, measured from our own goal.
    enter_defense: float = 0.30
    exit_defense: float = 0.45
    emergency_defense: float = 0.14
    minimum_dwell: float = 1.0
    max_observation_age: float = 0.35
    guard_depth: float = 0.35  # metres in front of own goal front
    boundary_clearance: float = 0.04  # beyond robot radius, metres
    threat_horizon_s: float = 3.0  # later forecast crossings do not force defense
    threat_confirmations: int = 2  # distinct captures forecasting a mouth crossing
    contest_radius: float = 0.60  # metres from the ball where a peer contests possession
    contest_margin: float = 0.10  # metres of advantage that decides possession outright
    yield_dwell_s: float = 1.0
    peer_stall_s: float = 3.0
    peer_stall_motion: float = 0.03  # metres; less motion than this counts as static
    intercept_max_spread: float = 0.08  # metres of forecast spread an intercept may use


@dataclass(frozen=True)
class TacticalDecision:
    role: str
    action: str
    target: Pose | None
    reason: str
    prediction: dict | None = None


@dataclass(frozen=True)
class Threat:
    t: float          # seconds after forecast capture
    y: float          # forecast crossing ordinate at the own goal front
    spread: float
    certain: bool     # spread small relative to the physical mouth
    confirmed: bool   # forecast on enough distinct captures


def _pose_ok(pose):
    return isinstance(pose, Pose) and _finite(pose.x, pose.y, pose.yaw)


class Tactics:
    """tick(observation, now, peers) returns an action for this robot only.

    Defense enters near our own goal and exits farther away. Minimum dwell
    applies to ordinary changes; an emergency can enter defense immediately.
    The first valid observation chooses a role without an artificial dwell.
    Config defaults are provisional decisions, not hardware operating limits.
    Clock rollback holds until the previous time is reached; create a new
    instance for a new clock/session. No past motion is replayed by this class.
    """

    def __init__(self, field: Field, robot_id: str, config=TacticsConfig(), *,
                 roles="match", profile=None):
        if not isinstance(field, Field) or not isinstance(config, TacticsConfig):
            raise ValueError("field and config must be Field and TacticsConfig")
        if not isinstance(robot_id, str) or not robot_id.strip():
            raise ValueError("robot_id must be nonempty")
        if roles not in ROLE_POLICIES:
            raise ValueError("roles must be match, offense or defense")
        if profile is not None and not isinstance(profile, MotionProfile):
            raise ValueError("profile must be a MotionProfile or None")
        if (type(field.attack_sign) is not int or field.attack_sign not in (-1, 1)
                or not _finite(field.length, field.width, field.goal_width,
                               field.robot_radius, field.ball_radius, field.bumper_offset)
                or min(field.length, field.width, field.goal_width, field.robot_radius, field.bumper_offset) <= 0
                or not 0 <= field.ball_radius < min(field.length, field.width) / 2
                or field.goal_width > field.width):
            raise ValueError("field geometry must be finite and physically bounded")
        bounds = field.bounds
        if (not _finite(*vars(config).values()) or type(config.threat_confirmations) is not int
                or not 0 <= config.emergency_defense < config.enter_defense
                < config.exit_defense < 1
                or config.minimum_dwell < 0 or config.max_observation_age <= 0
                or config.guard_depth < 0 or config.boundary_clearance < 0
                or config.threat_horizon_s <= 0 or config.threat_confirmations < 1
                or min(config.contest_radius, config.peer_stall_s, config.intercept_max_spread) <= 0
                or min(config.contest_margin, config.yield_dwell_s, config.peer_stall_motion) < 0
                or field.robot_radius + config.boundary_clearance
                >= min(bounds.length, bounds.width) / 2):
            raise ValueError("invalid tactic thresholds, timing or clearance")
        self.field, self.robot_id, self.config = field, robot_id, config
        self.roles, self.profile = roles, profile
        self.role = "defense" if roles == "defense" else "offense"
        self._changed_at = self._last_now = self._last_capture = None
        self.track = BallTrack()
        self._threat_count, self._threat_capture = 0, None
        self._peer_anchor = {}   # peer id -> (capture, pose) at its last observed motion
        self._yield_until = {}   # peer id -> control time a yield must persist until

    def _hold(self, reason):
        return TacticalDecision(self.role, "hold", None, reason)

    def tick(self, observation: Observation, now: float, peers=None) -> TacticalDecision:
        if not _finite(now):
            return self._hold("invalid control clock")
        if self._last_now is not None and now < self._last_now:
            return self._hold("control clock moved backwards")
        self._last_now = now
        if not isinstance(observation, Observation) or observation.robot_id != self.robot_id:
            return self._hold("missing observation or wrong robot identity")
        stamp = observation.captured_at
        if not _finite(stamp) or not 0 <= now - stamp <= self.config.max_observation_age:
            return self._hold("observation is stale, future or has an invalid timestamp")
        if self._last_capture is not None and stamp < self._last_capture:
            return self._hold("observation capture moved backwards")
        self._last_capture = stamp
        if type(observation.fallen) is not bool or observation.fallen:
            return self._hold("robot is fallen or fall state is invalid")
        robot, ball = observation.robot, observation.ball
        if not _pose_ok(robot):
            return self._hold("robot pose is missing or invalid")
        if not isinstance(ball, Ball) or not _finite(ball.x, ball.y, ball.vx, ball.vy):
            self.track.reset()
            return self._hold("ball state is missing or invalid")
        f, c = self.field, self.config
        bounds = f.bounds
        if not bounds.contains(robot.x, robot.y):
            return self._hold("robot is outside the field")
        if not bounds.contains(ball.x, ball.y):
            self.track.reset()
            return self._hold("ball is outside the field; reassess/reset is required")
        peers = {} if peers is None else peers
        if not isinstance(peers, dict):
            return self._hold("peer observations must be a mapping")
        for pid, peer in peers.items():
            if (pid == self.robot_id or not isinstance(peer, Observation) or not _pose_ok(peer.robot)
                    or type(peer.kick_active) is not bool or not _finite(peer.captured_at)
                    or not 0 <= now - peer.captured_at <= c.max_observation_age):
                return self._hold(f"peer {pid} observation is missing, stale or invalid")
            self._observe_peer(pid, peer)
        for pid in set(self._peer_anchor) - set(peers):
            self._peer_anchor.pop(pid)
            self._yield_until.pop(pid, None)

        self.track.update(stamp, ball.x, ball.y)
        forecast = self.track.forecast(self.profile)
        threat = self._threat(forecast)
        reason = self._select_role(ball, threat, now)
        if self.role == "offense":
            return self._offense(robot, ball, forecast, peers, now, reason)
        return self._defense(robot, ball, forecast, threat, now, reason)

    # ---- roles -------------------------------------------------------------
    def _select_role(self, ball, threat, now):
        c = self.config
        if self.roles != "match":
            if self._changed_at is None:
                self._changed_at = now
            return "solo offense policy" if self.roles == "offense" else "defensive experiment policy"
        own, opponent = self.field.own_goal, self.field.opponent_goal
        distance_fraction = (ball.x - own.x) / (opponent.x - own.x)
        rolling = threat is not None and threat.confirmed
        emergency = distance_fraction <= c.emergency_defense or rolling
        desired = self.role
        if emergency or (self.role == "offense" and distance_fraction <= c.enter_defense):
            desired = "defense"
        elif self.role == "defense" and distance_fraction >= c.exit_defense:
            desired = "offense"
        dwell_complete = self._changed_at is None or now - self._changed_at >= c.minimum_dwell
        cause = ("rolling ball forecast to cross the own goal (emergency own-goal threat)"
                 if rolling and distance_fraction > c.emergency_defense else "emergency own-goal threat")
        reason = "ball position retains current role"
        if desired != self.role:
            if dwell_complete or emergency:
                self.role = desired
                self._changed_at = now
                reason = cause if emergency else "ball crossed role threshold"
            else:
                reason = "minimum role dwell"
        elif emergency:
            reason = cause
        if self._changed_at is None:
            self._changed_at = now
        return reason

    def _threat(self, forecast):
        """Forecast own-mouth crossing, counted once per distinct capture."""
        if forecast is None:
            self._threat_count, self._threat_capture = 0, None
            return None
        f, c = self.field, self.config
        goal = f.own_goal
        crossing = forecast.crossing(goal.x)
        if crossing is None or crossing[0] > c.threat_horizon_s:
            self._threat_count, self._threat_capture = 0, None
            return None
        t, y = crossing
        spread = forecast.spread(t)
        if y + spread < goal.y_min - f.ball_radius or y - spread > goal.y_max + f.ball_radius:
            self._threat_count, self._threat_capture = 0, None
            return None
        if self._threat_capture is None or forecast.captured_at > self._threat_capture:
            self._threat_count += 1
            self._threat_capture = forecast.captured_at
        return Threat(t, y, spread, 2 * spread <= goal.width, self._threat_count >= c.threat_confirmations)

    # ---- prediction bookkeeping -------------------------------------------
    def _intercept_ready(self):
        return self.profile is not None and not self.profile.missing("intercept")

    def _prediction(self, forecast, now, **extra):
        record = forecast.record(now)
        profile = self.profile
        record["profile"] = None if profile is None else {"source": profile.source, "synthetic": profile.synthetic}
        record["intercept_inputs_missing"] = (list(profile.missing("intercept")) if profile is not None
                                              else ["motion profile"])
        record.update(extra)
        return record

    # ---- offense -----------------------------------------------------------
    def _offense(self, robot, ball, forecast, peers, now, reason):
        f = self.field
        yielded = self._contest(robot, ball, peers, now)
        if yielded is not None:
            target = self._standoff(ball, peers)
            if target is None:
                return self._hold(f"{reason}; {yielded}; no ball-clear standoff fits")
            return TacticalDecision(self.role, "yield", target, f"{reason}; {yielded}")
        keepout = peer_clearance(f)
        goal = f.opponent_goal.center
        heading = math.atan2(goal[1] - ball.y, goal[0] - ball.x)
        contact = f.bumper_offset + f.ball_radius
        stance = (ball.x - math.cos(heading) * contact, ball.y - math.sin(heading) * contact)
        for pid, peer in sorted(peers.items()):
            if (self._peer_static(pid, peer) and not peer.kick_active
                    and math.hypot(stance[0] - peer.robot.x, stance[1] - peer.robot.y) < keepout):
                return self._hold(f"{reason}; ball is pinned beside static peer {pid}; reset is required")
        if forecast is None:
            return TacticalDecision(self.role, "attack", None, reason)
        if not self._intercept_ready():
            return TacticalDecision(self.role, "attack", None,
                                    f"{reason}; rolling ball; intercept unavailable without a complete motion profile",
                                    self._prediction(forecast, now, intercept=None))
        clearance = defensive_clearance(f)

        def staging(point, t):
            yaw = math.atan2(goal[1] - point[1], goal[0] - point[0])
            back = clearance + .02
            x, y = point[0] - math.cos(yaw) * back, point[1] - math.sin(yaw) * back
            if (not f.bounds.contains(x, y, f.robot_radius + self.config.boundary_clearance)
                    or math.hypot(x - ball.x, y - ball.y) < clearance
                    or any(math.hypot(x - p.robot.x, y - p.robot.y) < keepout for p in peers.values())):
                return None
            return Pose(x, y, yaw)

        found = first_intercept(forecast, robot, now, self.profile, staging,
                                max_spread=self.config.intercept_max_spread)
        if found is None:
            return TacticalDecision(self.role, "attack", None,
                                    f"{reason}; no reachable forecast intercept; pursuing the measured ball",
                                    self._prediction(forecast, now, intercept=None))
        return TacticalDecision(self.role, "intercept", found.pose,
                                f"{reason}; forecast intercept behind the rolling ball",
                                self._prediction(forecast, now, intercept=found.record()))

    def _observe_peer(self, pid, peer):
        anchor = self._peer_anchor.get(pid)
        pose = peer.robot
        if (anchor is None or peer.captured_at < anchor[0]
                or math.hypot(pose.x - anchor[1].x, pose.y - anchor[1].y) > self.config.peer_stall_motion
                or abs(math.remainder(pose.yaw - anchor[1].yaw, math.tau)) > .15):
            self._peer_anchor[pid] = (peer.captured_at, pose)

    def _peer_static(self, pid, peer):
        anchor = self._peer_anchor.get(pid)
        return anchor is not None and peer.captured_at - anchor[0] >= self.config.peer_stall_s

    def _contest(self, robot, ball, peers, now):
        """Reason to yield possession to a peer, or None."""
        c = self.config
        for pid, peer in sorted(peers.items()):
            p = peer.robot
            d_peer = math.hypot(p.x - ball.x, p.y - ball.y)
            if d_peer > c.contest_radius:
                self._yield_until.pop(pid, None)
                continue
            if peer.kick_active:
                return f"peer {pid} has an unresolved kick near the ball"
            if self._peer_static(pid, peer):
                self._yield_until.pop(pid, None)
                continue
            d_self = math.hypot(robot.x - ball.x, robot.y - ball.y)
            if abs(d_self - d_peer) <= c.contest_margin:
                winner = min(self.robot_id, pid)  # symmetric tie: identical answer for both observers
            else:
                winner = self.robot_id if d_self < d_peer else pid
            if winner == pid:
                self._yield_until[pid] = now + c.yield_dwell_s
            if winner == pid or now < self._yield_until.get(pid, -math.inf):
                return f"peer {pid} has possession of the contested ball"
        return None

    def _standoff(self, ball, peers):
        f, c = self.field, self.config
        own = f.own_goal.center
        base = math.atan2(own[1] - ball.y, own[0] - ball.x)
        distance = c.contest_radius + .10
        margin = f.robot_radius + c.boundary_clearance
        keepout = peer_clearance(f)
        for delta in (0., .5, -.5, 1., -1., 1.5, -1.5):
            angle = base + delta
            x, y = ball.x + distance * math.cos(angle), ball.y + distance * math.sin(angle)
            if (f.bounds.contains(x, y, margin)
                    and all(math.hypot(x - p.robot.x, y - p.robot.y) >= keepout for p in peers.values())):
                return Pose(x, y, math.atan2(ball.y - y, ball.x - x))
        return None

    # ---- defense -----------------------------------------------------------
    def _defense(self, robot, ball, forecast, threat, now, reason):
        f = self.field
        if threat is not None and threat.certain:
            crossing = {"t_after_capture_s": round(threat.t, 4), "y": round(threat.y, 4),
                        "spread_m": round(threat.spread, 4), "confirmed": threat.confirmed}
            if self._intercept_ready():
                found = self._defensive_intercept(robot, ball, forecast, threat, now)
                if found is not None:
                    return TacticalDecision(self.role, "intercept", found.pose,
                                            f"{reason}; reachable intercept of the forecast own-goal shot",
                                            self._prediction(forecast, now, crossing=crossing,
                                                             intercept=found.record()))
            block = self.guard_pose(ball, aim_y=threat.y)
            if block is not None:
                return TacticalDecision(self.role, "block", block,
                                        f"{reason}; retreat to block the forecast own-goal crossing",
                                        self._prediction(forecast, now, crossing=crossing, intercept=None))
        target = self.guard_pose(ball)
        if target is None:
            return self._hold(reason + "; no ball-clear goalward guard pose fits inside the field and goal span")
        if threat is not None:
            return TacticalDecision(self.role, "defend", target,
                                    f"{reason}; uncertain rolling ball; conservative goal-side guard",
                                    self._prediction(forecast, now, crossing=None, intercept=None))
        return TacticalDecision(self.role, "defend", target, reason)

    def _defensive_intercept(self, robot, ball, forecast, threat, now):
        f, c = self.field, self.config
        clearance = defensive_clearance(f)
        facing = math.atan2(-forecast.vy, -forecast.vx)
        margin = f.robot_radius + c.boundary_clearance
        contact = f.robot_radius + f.ball_radius

        def on_path(point, t):
            if (not f.bounds.contains(point[0], point[1], margin)
                    or math.hypot(point[0] - ball.x, point[1] - ball.y) < clearance):
                return None
            return Pose(point[0], point[1], facing)

        def lead(t):
            return contact / max(.05, forecast.speed - forecast.deceleration * t)

        return first_intercept(forecast, robot, now, self.profile, on_path, until=threat.t,
                               max_spread=c.intercept_max_spread, lead=lead)

    def guard_pose(self, ball, own_goal=None, aim_y=None):
        """Ball-facing guard pose in front of the own goal mouth, or None.

        own_goal is a measured goal front ({"x", "y_min", "y_max"} or Goal);
        None uses this field's physical own goal. ``aim_y`` replaces the ball
        ordinate as the lateral aim, e.g. a forecast crossing to block.
        """
        f, c = self.field, self.config
        bounds = f.bounds
        margin = f.robot_radius + c.boundary_clearance
        depth = min(max(c.guard_depth, margin), bounds.length / 2)
        if own_goal is None:
            own_goal = f.own_goal
        goal = own_goal.record() if hasattr(own_goal, "record") else own_goal
        x = goal["x"] + f.attack_sign * depth
        centre = (goal["y_min"] + goal["y_max"]) / 2
        lower = max(bounds.y_min + margin, min(centre, goal["y_min"] + f.robot_radius))
        upper = min(bounds.y_max - margin, max(centre, goal["y_max"] - f.robot_radius))
        if lower > upper or not bounds.x_min + margin <= x <= bounds.x_max - margin:
            return None
        y = min(max(ball.y if aim_y is None else aim_y, lower), upper)
        clearance = defensive_clearance(f)
        if math.hypot(ball.x - x, ball.y - y) < clearance:
            # Preserve the nominal goal-mouth lane; only move goalward until
            # the bumper-inclusive exclusion circle clears that lane.
            goalward = math.sqrt(max(0.0, clearance ** 2 - (ball.y - y) ** 2))
            x = ball.x - f.attack_sign * goalward
            if not bounds.x_min + margin <= x <= bounds.x_max - margin:
                return None
        dx, dy = ball.x - x, ball.y - y
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else math.atan2(0.0, f.attack_sign)
        return Pose(x, y, yaw)
