"""Separately timed onboard ball evidence for paw docking; never global pose.

Overhead tracking keeps route, global pose and goal heading. A Pi-local Ball
Eye row (follow_me_pupper/robot/onboard_ball.py, fc_onboard_ball_metric_v1)
gives the ball centre on base_link at its own capture time. ``LocalDocking``
decides whether that measurement may replace the overhead ball-minus-pose
offset in the paw contact window. It never produces a Pose or a field Ball,
and its evidence expires no later than the overhead pose it depends on.

Evidence status:

- ``overhead``: no enabled profile; existing overhead docking is unchanged.
- ``blocked``: an enabled profile without current agreed local evidence. The
  overhead path may still steer, but it cannot authorize a kick. Remembering
  nothing across cancels is deliberate: authority comes only from current evidence.
- ``local``: agreed local evidence; dock and decide readiness in the body frame.

Rules, all on the Pi monotonic clock in seconds:

- Admission uses the row's source capture time. Future, stale, duplicate,
  reordered and pre-reset captures are refused, never repaired from arrival.
- Profile, calibration and physical radius must equal the selected docking
  profile, whose radius is the runner Field's radius. Synthetic docking accepts
  only synthetic camera rows; live docking only measured ones.
- Outliers are gated against the track. A refused fix never seeds or replaces
  it, but it does suspend the track's evidence until a consistent fix arrives.
  A newer row with no measurement (searching/conflict) ends the track.
- Body-frame values describe the body at capture. Commanded motion, overhead
  motion since the local capture, unknown motion (kick clip, hold) or overhead
  evidence that the ball moved clear the track and refuse older captures.
- Agreement with overhead uses a pose and ball captured within the pairing gap
  of the local capture; a track that agreed may continue while the overhead
  ball is hidden, until it resets.
- Nothing here learns or updates calibration. ``evidence`` updates track state
  (expiry and resets) as it evaluates; call it from the control owner only.

Live use is off: ``live_profile`` stays unqualified until a measured radius,
mount and independent local error exist. ``synthetic_profile`` exercises the
same contract offline and is labelled synthetic in every result.
"""
from collections import deque
from dataclasses import dataclass
import math

from .model import Ball, Pose
from .paw_approach import (FORWARD_WINDOW_M, LATERAL_WINDOW_M, MAX_CAPTURE_GAP_S, PAW_LEFT_M, SETTLE_S, WINDUP_S,
                           YAW_WINDOW_RAD)

FIT_METHODS = ("eye_fit", "pink_fit", "pinkfloor_fit")
PROFILE_STATUSES = ("synthetic", "live")


def _finite(*values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class LocalBallObservation:
    """One measured ball centre on base_link at its source capture time."""

    captured_at: float  # Pi CLOCK_MONOTONIC seconds of the camera capture
    x: float
    y: float
    sigma_xy_m: float  # image-noise model only
    method: str
    profile_id: str
    calibration_id: str
    ball_radius_m: float
    profile_status: str
    producer: str
    sequence: int
    radius_measured: bool = False
    mount_measured: bool = False

    @classmethod
    def from_metric_row(cls, row):
        """A receiver row -> the selected measurement, or None when nothing was measured."""
        if row is None or row["state"] != "measured":
            return None
        chosen, identity = row["candidates"][row["selected"]], row["identity"]
        return cls(row["captured_at_mono_ns"] / 1e9, chosen["x"], chosen["y"], chosen["sigma_xy_m"],
                   chosen["method"], identity["profile_id"], identity["calibration_id"],
                   identity["ball_radius_m"], identity["status"], row["producer_instance"], row["sequence"],
                   identity["radius_measured"], identity["mount_measured"])


@dataclass(frozen=True)
class DockingProfile:
    """Budgets and identity for local docking. Unmeasured values keep it disabled."""

    name: str
    status: str
    ball_radius_m: float
    profile_id: str | None = None  # required camera profile; None accepts any with the same radius
    calibration_id: str | None = None
    measured_error_m: float | None = None  # independent local-vs-truth error; None is unqualified
    radius_measured: bool = False
    mount_measured: bool = False
    allowed_methods: tuple = FIT_METHODS
    max_age_s: float = 0.25
    max_pose_age_s: float = MAX_CAPTURE_GAP_S  # same pose freshness as overhead kick readiness
    max_pair_gap_s: float = 0.10
    agreement_m: float = 0.15  # Tej's overhead/local budget; it cannot detect a radius-sized range error
    max_sigma_m: float = 0.010
    gate_m: float = 0.05
    gate_speed_mps: float = 0.5
    confirm: int = 2
    max_gap_s: float = 0.30
    motion_m: float = 0.010
    motion_rad: float = 0.03
    pose_motion_m: float = 0.03
    pose_motion_rad: float = 0.06

    def __post_init__(self):
        if self.status not in PROFILE_STATUSES:
            raise ValueError("docking profile status must be synthetic or live")
        if not _finite(self.ball_radius_m) or self.ball_radius_m <= 0:
            raise ValueError("docking profile needs the selected positive ball radius")
        numbers = (self.max_age_s, self.max_pose_age_s, self.max_pair_gap_s, self.agreement_m, self.max_sigma_m,
                   self.gate_m, self.gate_speed_mps, self.max_gap_s, self.motion_m, self.motion_rad,
                   self.pose_motion_m, self.pose_motion_rad)
        if not _finite(*numbers) or min(numbers) <= 0 or type(self.confirm) is not int or self.confirm < 1:
            raise ValueError("docking budgets must be finite and positive")
        if self.measured_error_m is not None and (not _finite(self.measured_error_m) or self.measured_error_m < 0):
            raise ValueError("measured error must be finite and nonnegative")
        if not set(self.allowed_methods) <= set(FIT_METHODS):
            raise ValueError("docking accepts outline-fitted methods only")

    def problem(self):
        """Why this profile cannot refine docking, or None."""
        if self.status == "synthetic":
            return None
        missing = [label for label, ok in (("measured ball radius", self.radius_measured),
                                           ("measured camera mount", self.mount_measured),
                                           ("selected camera profile", self.profile_id is not None),
                                           ("independent local error", self.measured_error_m is not None)) if not ok]
        if missing:
            return "local docking unqualified: missing " + ", ".join(missing)
        if self.measured_error_m > LATERAL_WINDOW_M / 2:
            return "local docking unqualified: measured error exceeds half the lateral contact window"
        return None

    def identity_problem(self, obs):
        if obs.ball_radius_m != self.ball_radius_m:
            return "ball radius differs from the selected field radius"
        if self.profile_id is not None and obs.profile_id != self.profile_id:
            return "camera profile differs from the selected profile"
        if self.calibration_id is not None and obs.calibration_id != self.calibration_id:
            return "camera calibration differs from the selected calibration"
        if self.status == "synthetic" and obs.profile_status != "synthetic":
            return "synthetic docking accepts only synthetic camera rows"
        if self.status == "live" and not (obs.profile_status == "measured" and obs.radius_measured
                                          and obs.mount_measured):
            return "live docking requires a measured camera profile"
        return None


def live_profile(field, *, profile_id=None, calibration_id=None):
    """Default live profile: the runner's selected radius, disabled until measured."""
    return DockingProfile("live-unqualified", "live", field.ball_radius, profile_id, calibration_id)


def synthetic_profile(ball_radius_m, **overrides):
    """Offline profile that exercises the consumer contract; never physical evidence."""
    return DockingProfile(overrides.pop("name", "synthetic"), "synthetic", ball_radius_m, **overrides)


class MotionLedger:
    """Published body velocity commands as piecewise-constant segments."""

    def __init__(self, horizon_s=2.0):
        self.horizon_s = horizon_s
        self.segments = deque()  # (start time, vx, vy, wz)

    def record(self, now, vx, vy, wz):
        if not _finite(now, vx, vy, wz):
            self.segments.clear()  # unknown motion: nothing before this can be trusted
            return
        if self.segments and now < self.segments[-1][0]:
            self.segments.clear()
        if not self.segments or self.segments[-1][1:] != (vx, vy, wz):
            self.segments.append((now, vx, vy, wz))
        while len(self.segments) > 1 and self.segments[1][0] <= now - self.horizon_s:
            self.segments.popleft()

    def bound(self, t0, t1):
        """(translation m, rotation rad) commanded over [t0, t1], or None if not covered."""
        if not self.segments or t0 < self.segments[0][0] or t1 < t0:
            return None
        moved = turned = 0.
        for i, (start, vx, vy, wz) in enumerate(self.segments):
            end = self.segments[i + 1][0] if i + 1 < len(self.segments) else math.inf
            span = min(end, t1) - max(start, t0)
            if span > 0:
                moved += math.hypot(vx, vy) * span
                turned += abs(wz) * span
        return moved, turned


@dataclass(frozen=True)
class DockingEvidence:
    status: str  # "overhead", "blocked" or "local" (see the module docstring)
    reason: str
    profile: str | None = None
    synthetic: bool = False
    forward: float | None = None  # ball centre ahead of base_link at capture [m]
    lateral: float | None = None  # ball centre left of base_link at capture [m]
    velocity: tuple | None = None  # body-frame ball velocity over the still track [m/s], if measurable
    captured_at: float | None = None
    expires_at: float | None = None
    sigma_xy_m: float | None = None
    agreement_m: float | None = None
    steady: bool = False
    method: str | None = None
    profile_id: str | None = None
    calibration_id: str | None = None


class LocalBallTrack:
    """Admitted local fixes of one ball; measurements only, no prediction."""

    def __init__(self, profile: DockingProfile):
        self.profile = profile
        self.highwater = None
        self.cutoff = -math.inf  # captures at or before the last reset never re-enter
        self.generation = 0
        self.rejected = 0
        self.reset("new track")

    def reset(self, reason, now=None):
        self.fixes = deque(maxlen=32)
        self.agreed = None  # agreement distance earned by this track
        self.contradicted = False
        self.reason = reason
        self.generation += 1
        boundary = now if _finite(now) else self.highwater
        if boundary is not None:
            self.cutoff = max(self.cutoff, boundary)

    @property
    def confirmed(self):
        return len(self.fixes) >= self.profile.confirm

    def _refuse(self, reason):
        self.rejected += 1
        self.reason = reason
        return False, reason

    def _time_problem(self, captured_at, now):
        if not _finite(now, captured_at):
            return "observation or capture time is missing or not finite"
        if captured_at > now:
            return "capture time is in the future"
        if now - captured_at > self.profile.max_age_s:
            return "capture is stale"
        if self.highwater is not None and captured_at <= self.highwater:
            return "duplicate or reordered capture"
        if captured_at <= self.cutoff:
            return "capture precedes the last reset"
        return None

    def admit_absence(self, captured_at, now):
        """A newer capture measured no ball (searching or conflict): the track ends."""
        if (problem := self._time_problem(captured_at, now)) is not None:
            return self._refuse(problem)
        self.highwater = captured_at
        self.reset("a newer capture measured no ball", captured_at)
        return True, self.reason

    def admit(self, obs, now, ledger=None):
        p = self.profile
        if not isinstance(obs, LocalBallObservation) or not _finite(obs.x, obs.y, obs.sigma_xy_m, obs.ball_radius_m):
            return self._refuse("observation or capture time is missing or not finite")
        if (problem := self._time_problem(obs.captured_at, now)) is not None:
            return self._refuse(problem)
        self.highwater = obs.captured_at
        if (problem := p.identity_problem(obs)) or (
                self.fixes and (obs.profile_id, obs.calibration_id) != (self.fixes[-1].profile_id,
                                                                         self.fixes[-1].calibration_id)):
            self.reset("profile, calibration or radius changed", obs.captured_at)
            return self._refuse(problem or "camera profile changed within the track")
        if obs.method not in p.allowed_methods:
            return self._refuse(f"method {obs.method} is not allowed for docking")
        if obs.sigma_xy_m > p.max_sigma_m:
            return self._refuse("image uncertainty exceeds the docking limit")
        if self.fixes:
            last = self.fixes[-1]
            dt = obs.captured_at - last.captured_at
            motion = ledger.bound(last.captured_at, obs.captured_at) if ledger is not None else (0., 0.)
            if dt > p.max_gap_s:
                self.reset("track lapsed")
            elif motion is None or motion[0] > p.motion_m or motion[1] > p.motion_rad:
                self.reset("body moved between captures")
            elif math.hypot(obs.x - last.x, obs.y - last.y) > (
                    p.gate_m + p.gate_speed_mps * dt + 3 * math.hypot(obs.sigma_xy_m, last.sigma_xy_m)):
                if self.confirmed:
                    self.contradicted = True  # newer evidence disagrees: suspend, never reseed
                else:
                    self.reset("tentative seed contradicted")
                return self._refuse("outlier against the current track")
        self.fixes.append(obs)
        self.contradicted = False
        self.reason = "confirmed" if self.confirmed else "tentative"
        return True, self.reason

    def current(self, now, ledger=None):
        """Newest fix of a confirmed, uncontradicted track, if fresh and the body has not moved since."""
        if not self.confirmed or self.contradicted or not _finite(now):
            return None
        last = self.fixes[-1]
        if not 0 <= now - last.captured_at <= self.profile.max_age_s:
            return None
        motion = ledger.bound(last.captured_at, now) if ledger is not None else (0., 0.)
        if motion is None or motion[0] > self.profile.motion_m or motion[1] > self.profile.motion_rad:
            self.reset("body moved since the local capture", now)
            return None
        return last

    def recent(self):
        last = self.fixes[-1]
        return [f for f in self.fixes if last.captured_at - f.captured_at <= .45]

    def steady(self):
        """Fixes spanning SETTLE_S all within 8 mm of the newest: a body-relative still ball."""
        if not self.confirmed:
            return False
        recent, last = self.recent(), self.fixes[-1]
        return (last.captured_at - recent[0].captured_at >= SETTLE_S
                and all(math.hypot(f.x - last.x, f.y - last.y) <= 0.008 for f in recent))

    def velocity(self):
        """Body-frame ball velocity over the recent still-body fixes, or None.

        Least squares, shrunk toward zero by two standard errors of the reported
        image noise, so stationary jitter is not forecast as a roll.
        """
        if not self.confirmed:
            return None
        recent = self.recent()
        if len(recent) < 3 or recent[-1].captured_at - recent[0].captured_at < SETTLE_S:
            return None
        n = len(recent)
        mean_t = sum(f.captured_at for f in recent) / n
        total = sum((f.captured_at - mean_t) ** 2 for f in recent)
        sigma = sum(f.sigma_xy_m for f in recent) / n
        margin = 2 * sigma / math.sqrt(total)

        def slope(values):
            mean = sum(values) / n
            v = sum((f.captured_at - mean_t) * (value - mean) for f, value in zip(recent, values)) / total
            return math.copysign(max(0., abs(v) - margin), v)

        return slope([f.x for f in recent]), slope([f.y for f in recent])


class LocalDocking:
    """One robot's local docking consumer: track, command ledger and overhead history.

    The owner (executor) calls observe_row for each new local receiver row,
    note_command for each planned command, note_overhead for each admitted
    field frame, unknown_motion when the body may move without a velocity
    command (kick clip, external hold), reset on Stop/new epoch, and evidence
    when the paw controller docks.
    """

    def __init__(self, profile: DockingProfile | None):
        self.profile = profile
        self.ledger = MotionLedger()
        self.poses = deque(maxlen=64)  # (capture time, Pose)
        self.balls = deque(maxlen=64)  # (capture time, Ball)
        self.track = LocalBallTrack(profile) if profile is not None else None

    def reset(self, reason, now=None):
        if self.track is not None:
            self.track.reset(reason, now)

    def unknown_motion(self, now, reason):
        """The body may move without a commanded velocity: forget motion history and fixes."""
        self.ledger.segments.clear()
        self.reset(reason, now)

    def observe(self, obs, now):
        if self.track is None:
            return False, "local docking is not configured"
        return self.track.admit(obs, now, self.ledger)

    def observe_row(self, row, now):
        """A MetricBallReceiver row: a measurement or a newer capture without one."""
        if self.track is None:
            return False, "local docking is not configured"
        obs = LocalBallObservation.from_metric_row(row)
        if obs is not None:
            return self.observe(obs, now)
        return self.track.admit_absence(row["captured_at_mono_ns"] / 1e9, now)

    def note_command(self, now, vx, vy, wz):
        self.ledger.record(now, vx, vy, wz)

    def note_overhead(self, robot, robot_captured_at, ball=None, ball_captured_at=None):
        if isinstance(robot, Pose) and _finite(robot_captured_at, robot.x, robot.y, robot.yaw):
            if self.poses and robot_captured_at <= self.poses[-1][0]:
                return
            if self.poses and self.profile is not None and self._moved(self.poses[-1][1], robot):
                self.reset("overhead observed body motion", robot_captured_at)
            self.poses.append((robot_captured_at, robot))
        if isinstance(ball, Ball) and _finite(ball_captured_at, ball.x, ball.y) and (
                not self.balls or ball_captured_at > self.balls[-1][0]):
            self.balls.append((ball_captured_at, ball))

    def _moved(self, old, new):
        return (math.hypot(new.x - old.x, new.y - old.y) > self.profile.pose_motion_m
                or abs(_wrap(new.yaw - old.yaw)) > self.profile.pose_motion_rad)

    @staticmethod
    def _nearest(history, t, gap):
        best = min(history, key=lambda item: abs(item[0] - t), default=None)
        return best if best is not None and abs(best[0] - t) <= gap else None

    def evidence(self, now, robot, robot_captured_at):
        """Docking evidence for this tick. Requires a fresh overhead pose; never renews it."""
        p = self.profile
        if p is None:
            return DockingEvidence("overhead", "local docking is not configured")
        base = dict(profile=p.name, synthetic=p.status == "synthetic")
        if (problem := p.problem()) is not None:
            return DockingEvidence("overhead", problem, **base)

        def blocked(reason, **fields):
            return DockingEvidence("blocked", reason, **base, **fields)

        if not isinstance(robot, Pose) or not _finite(now, robot_captured_at) or not (
                0 <= now - robot_captured_at <= p.max_pose_age_s):
            return blocked("overhead pose is stale; local ball cannot renew global pose")
        local = self.track.current(now, self.ledger)
        if local is None:
            return blocked(f"no current confirmed local ball ({self.track.reason})")
        pose = self._nearest(self.poses, local.captured_at, p.max_pair_gap_s)
        if pose is not None and self._moved(pose[1], robot):
            self.track.reset("overhead pose moved since the local capture", robot_captured_at)
            return blocked("overhead pose moved since the local capture")
        ball = self._nearest(self.balls, local.captured_at, p.max_pair_gap_s)
        newest = self.balls[-1] if self.balls else None
        if (ball is not None and newest is not None and newest[0] > local.captured_at
                and math.hypot(newest[1].x - ball[1].x, newest[1].y - ball[1].y) > p.gate_m):
            self.track.reset("overhead saw the ball move after the local capture", newest[0])
            return blocked("overhead saw the ball move after the local capture")
        if pose is not None and ball is not None:
            t0, t1 = sorted((pose[0], local.captured_at))
            motion = self.ledger.bound(t0, t1)
            if motion is not None and motion[0] <= p.motion_m and motion[1] <= p.motion_rad:
                c, s = math.cos(pose[1].yaw), math.sin(pose[1].yaw)
                bx = pose[1].x + c * local.x - s * local.y
                by = pose[1].y + s * local.x + c * local.y
                distance = math.hypot(bx - ball[1].x, by - ball[1].y)
                if distance > p.agreement_m:
                    self.track.reset("local and overhead balls disagree", local.captured_at)
                    return blocked(f"local and overhead balls disagree by {distance:.3f} m", agreement_m=distance)
                self.track.agreed = distance
        if self.track.agreed is None:
            return blocked("local track has not agreed with an overhead ball")
        return DockingEvidence(
            "local", "agreed local ball refines docking", forward=local.x, lateral=local.y,
            velocity=self.track.velocity(), captured_at=local.captured_at,
            expires_at=min(local.captured_at + p.max_age_s, robot_captured_at + p.max_pose_age_s),
            sigma_xy_m=local.sigma_xy_m, agreement_m=self.track.agreed, steady=self.track.steady(),
            method=local.method, profile_id=local.profile_id, calibration_id=local.calibration_id, **base)


def body_contact_window(forward, lateral, heading_error, side, *, forward_m, scale=1.0):
    """paw_approach.contact_window with a ball offset already in the body frame."""
    return (abs(heading_error) <= scale * YAW_WINDOW_RAD
            and abs(forward - forward_m) <= scale * FORWARD_WINDOW_M
            and abs(lateral - PAW_LEFT_M[side]) <= scale * LATERAL_WINDOW_M)


def local_kick_ready(evidence, now, heading_error, side, *, forward_m):
    """Local counterpart of PawApproach.settled: steady, in the window, and forecast to stay
    there through the wind-up using the measured body-frame ball velocity."""
    if evidence.status != "local" or not evidence.steady or evidence.velocity is None or now >= evidence.expires_at:
        return False
    horizon = now - evidence.captured_at + WINDUP_S
    vx, vy = evidence.velocity
    return all(body_contact_window(evidence.forward + vx * h, evidence.lateral + vy * h, heading_error, side,
                                   forward_m=forward_m) for h in (0., horizon))


def overhead_body_offset(robot, ball):
    """The overhead ball expressed in the observed body frame (what contact_window uses)."""
    dx, dy = ball.x - robot.x, ball.y - robot.y
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    return dx * c + dy * s, -dx * s + dy * c


def evidence_record(evidence):
    """JSON-safe telemetry of one docking decision."""
    return {k: (round(v, 5) if isinstance(v, float) else v) for k, v in evidence.__dict__.items()}


__all__ = ["DockingEvidence", "DockingProfile", "LocalBallObservation", "LocalBallTrack", "LocalDocking",
           "MotionLedger", "body_contact_window", "evidence_record", "live_profile", "local_kick_ready",
           "overhead_body_offset", "synthetic_profile"]
