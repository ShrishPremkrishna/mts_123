"""Pupper FC v1 brain: one robot's decision on a World snapshot -> body-frame Cmd.

Pure logic (stdlib only). The same object drives the sim and the real robots.
Memory per robot is small: role + hold timer, attack sub-state (behind/orbit side,
push latch, headbutt stage), goalie return latch, stuck detector, last command (for
rate limits), a short command history (to predict our own pose through camera latency)
and a learned tag-heading correction per facing (kept across reset()).

Geometry, in the field frame: B = ball, u = unit push direction (ball -> aim point in
the attack goal), n = u rotated +90 deg. For the robot R, a = (R-B).u (negative =
behind the ball) and l = (R-B).n (lateral offset from the shot line); phi = atan2(l, a)
is the robot's bearing around the ball (+-pi = directly behind).

Knobs (params.json; DEFAULTS below apply when a key is missing):
  role_mode attack|defend|auto   role_hold_s  min seconds between role flips
  role_tau_s    low-pass time constant on the travel-time gap
  role_hyst_s   extra travel-time (s) the opponent must win by before an attacker
                yields; a defender attacks as soon as it is not slower (no both-defend stall)
  contact_mode push|headbutt
  pose_stale_s  own pose older than this -> stop      other_stale_s  ignore older opponents
  ball_stale_s  ball older than this -> ball_lost (hold = stop, search = turn at search_wz)
  ball_lead_max_s  cap on extrapolating the ball by its velocity to "now"
  predict_self, act_lag_s  roll own pose forward by the commands not yet visible to the camera
  yaw_learn, yaw_learn_rate, yaw_learn_max_deg  learn the tag heading bias per facing from
                clean straight walking (off by default; total correction clamped to +-max)
  k_turn        heading P gain (rad/s per rad)    k_vy  lateral P gain (m/s per m)
  k_curve       approach curvature: staging distance behind the ball per metre of |l|
  behind_cone_deg  |phi| above this counts as "behind" (else orbit around the ball)
  clear_margin  extra clearance (m) beyond ball_radius+robot_radius when orbiting
  push_vx, push_ahead, push_lat_tol  push speed, aim point beyond the ball, |l| latch (m)
  push_turn_scale  heading gain multiplier while pushing (lower = calmer dribble)
  push_side_m   0 = off (push dead-centre). >0 (m): approach and push along a line parallel to the shot line,
                shifted sideways so the ball sits push_side_m off the robot's centre line (a front leg/shoulder
                carries it; Nov 25 floor: off-centre hits scored, tight head-on alignment did not). push_lat_tol
                is then measured from that offset line. Push mode only, not plan mode.
  push_side     auto|left|right: which side of the robot the ball sits on. auto = the side the robot is
                already on (by heading when within 2 cm of the line). Latched once behind the ball; kept until
                the push ends, the robot loses the behind cone, the role changes or the ball is lost
  push_max_heading_deg  0 = off. >0: on the shot line next to the ball but facing more than this
                off the push direction -> "square" (turn in place toward the shot line, backing off
                to hold the body clear of the ball) instead of pushing sideways; +15 deg hysteresis
  aim_y_frac    aim at y = frac * ball_y inside the mouth (0 = centre)
  hb_*          headbutt: standoff, align tolerances, charge speed/distance/accel, back-off,
                hb_square_r = radius around the stage where it squares up holonomically
  goalie_style sideways|facing   goalie_dist (m in front of goal line)  goalie_margin
  goalie_reach  beyond this distance from the guard point the goalie walks back first
  k_goalie      guard position gain       clear_depth / clear_width_extra  clear zone
  clear_max_speed  only clear a ball approaching slower than this
  avoid_radius, avoid_gain  opponent repulsion (centre distance, strength)
  stuck_s, stuck_dist, unstick_s, unstick_vx  forward command but < stuck_dist progress
                for stuck_s -> reverse at unstick_vx for unstick_s
  orbit_hold_s  keep the chosen way round the ball at least this long
  orbit_slack   orbit waypoints up to this far outside the margin are clamped in, not rejected
  field_margin  keep walking targets this far inside the field edges
  accel_vx/vy/wz  rate limits (see shape())   decel_vx  vx rate limit toward zero (0 = accel_vx)
  decel_wz      wz rate limit toward zero (0 = accel_wz): below accel_wz, turns start quickly but
                reversals are damped
  contact_vx, contact_slow_m  near the ball the closing speed toward it is capped, ramping from
                max_vx at contact_slow_m (surface gap, m) down to contact_vx at touch
  gait_band, gait_lo, gait_hi, gait_hold_s  the walker switches gait at vx 0.5: forward commands
                avoid (gait_lo, gait_hi). Below the band unless the want reaches gait_hi; once
                fast, stay >= gait_hi while the want is above the band middle. Down-shifts are
                immediate; an up-shift waits gait_hold_s after the last down-shift
  goalie_vmax   speed cap while guarding the goal line (keep it in the trot: <= gait_lo)
  return_vx     speed cap while a defender walks back to its goal (> max_vx = no extra cap)
  solo_roles    1 = with no opponent robot seen (one robot vs a human), auto mode defends while the
                ball threatens our goal and attacks otherwise (0 = always attack, the v1 rule)
  solo_hold_s   min seconds between solo role flips (role_hold_s is for 1v1)
  solo_def_vmin ball speed toward our goal (m/s) that counts as moving in (halved to stay defending)
  solo_def_horizon_s  defend if the ball reaches our goal line within this many seconds
  solo_def_margin  ...and crosses it within this far outside the mouth (m)
  solo_third    our defensive zone as a fraction of the field length: a ball moving in here (not
                resting, not moving away) is defended even beyond the horizon
  solo_zone     0 = off; else a solo defender goes back to attack only for a ball within this fraction
                of the field length from our goal line (keeper first, forays into the home zone)
  ball_decel    rolling deceleration (m/s^2) for ball prediction (sim_ball_decel matches it)
  intercept_max_s  attacker aims at the ball's predicted position at its estimated arrival time,
                capped at this horizon (0 = off: aim at the ball now)
  intercept_vmin   below this ball speed (m/s) the attacker aims at the ball now
  intercept_near_m ...and also while pushing or within this surface gap (m) of the ball (a dribbled
                ball rolls ahead; chasing its prediction broke the push)
  return_detour 1 = a defender walking back to goal is not slowed to contact speed, and passes a ball
                that is not coming at the goal at clearance instead of walking into it
  return_back_vx  a returning defender facing away from its target backs toward it at up to this
                speed while it turns round (0 = turn in place first)
  return_block  1 = a returning defender that cannot reach its guard line before the ball steps into
                the ball's path at the first point it can reach in time (straight-line ball model)
  plan          1 = planned attack: a Dubins path (min turning radius plan_radius) to a stage plan_stage_m
                behind the predicted intercept point on the ball->goal line, then straight through the ball;
                followed with pure pursuit (plan_lookahead, turn rate cap plan_wz: speed drops on tight arcs).
                Replans only when the intercept point moves > plan_replan_m (scaled down to 30% over the last
                metre of path), the push direction turns >
                plan_replan_deg, the robot is > plan_dev_m off the path or the path is used up (at most once per
                plan_min_s). From the wrong side the path goes via a tangent point beside the ball. No collision-
                free path -> the old curved approach (0 = always the old approach)
  plan_horizon_s  intercept horizon for the plan (ETA from the path length at plan_eta_frac * max_vx)
  plan_near_m, plan_near_lat  behind the ball within plan_near_m surface gap, within 2*push_lat_tol of the
                line and facing within 45 deg of the push: pure pursuit along the shot line through the ball, sliding
                off the lateral error (turns scaled by push_turn_scale inside push_lat_tol), until the gap exceeds
                plan_near_m + 0.4 (no limit while on the line and facing the push within 30 deg: a rolled-on ball is
                chased along the line), the offset plan_near_lat or the heading error 70 deg. Only for a ball crossing
                the line slower than 0.25 m/s (0.4 to stay): a crossing ball is intercepted by a plan
  plan_flip_s   planning cost (s) of each left/right switch in a path (a spin the other way than the first arc
                counts): paths that keep turning one way look deliberate and reverse wz less
  plan_look_s   pure-pursuit lookahead grows by this many seconds of the current speed (a short lookahead at
                speed is a high heading gain: wobble and tag noise become turn reversals)
  plan_spin_deg pure-pursuit point further off the nose than this: turn in place
  coast_s       0 = off. Own pose older than this (but younger than pose_stale_s): do not steer on it; keep the
                last command, vx decaying toward coast_vx and vy/wz toward 0 (time constant coast_tau_s)
  fast_yaw_bias_deg  added to the robot's heading while the command executing is faster than fast_yaw_vx
                (the fast gait walks off its tag heading). Set it to the step test's heading_bias_deg at speed
  role_mode goto: walk to (goto_x, goto_y) and, if goto_yaw is a number, face it. Per-call
                params, not knobs; done within goto_tol_m and goto_yaw_tol_deg (dbg["done"]).
"""
from __future__ import annotations

import math

from .core import Cmd, Field, Pose, RobotCfg, World, wrap

DEFAULTS = {
    "loop_hz": 30, "role_mode": "auto", "contact_mode": "push",
    "max_vx": 0.30, "max_vy": 0.15, "max_wz": 1.0,
    "accel_vx": 0.6, "accel_vy": 0.5, "accel_wz": 2.0,
    "pose_stale_s": 0.6, "ball_stale_s": 1.0, "other_stale_s": 1.5,
    "ball_lost": "hold", "search_wz": 0.5, "ball_lead_max_s": 0.4,
    "predict_self": 1, "act_lag_s": 0.15, "yaw_learn": 0, "yaw_learn_rate": 0.02, "yaw_learn_max_deg": 20,
    "role_hold_s": 2.5, "role_hyst_s": 1.0, "role_tau_s": 0.5,
    "k_turn": 1.5, "k_vy": 1.5, "k_curve": 1.2,
    "behind_cone_deg": 120, "clear_margin": 0.06, "orbit_hold_s": 2.0, "orbit_slack": 0.10,
    "push_vx": 0.30, "push_ahead": 0.4, "push_turn_scale": 0.6, "push_lat_tol": 0.09, "aim_y_frac": 0.5,
    "push_max_heading_deg": 0,
    "push_side_m": 0.0, "push_side": "auto",
    "hb_standoff": 0.12, "hb_square_r": 0.2, "hb_lat_tol": 0.06, "hb_align_deg": 15, "hb_charge_vx": 0.30,
    "hb_charge_dist": 0.50, "hb_accel": 1.2, "hb_backoff_vx": 0.15, "hb_backoff_s": 0.8,
    "goalie_style": "sideways", "goalie_dist": 0.25, "goalie_margin": 0.05,
    "goalie_reach": 0.35, "k_goalie": 2.5,
    "clear_depth": 0.55, "clear_width_extra": 0.25, "clear_max_speed": 0.25,
    "stuck_s": 2.5, "stuck_dist": 0.06, "unstick_s": 1.0, "unstick_vx": 0.15,
    "avoid_radius": 0.55, "avoid_gain": 1.0, "field_margin": 0.12,
    "goto_tol_m": 0.08, "goto_yaw_tol_deg": 8,
    "decel_vx": 0.0, "decel_wz": 0.0, "contact_vx": 0.30, "contact_slow_m": 0.35,
    "gait_band": 1, "gait_lo": 0.45, "gait_hi": 0.60, "gait_hold_s": 0.5, "goalie_vmax": 0.45,
    "return_vx": 1.5,
    "solo_roles": 1, "solo_hold_s": 1.0, "solo_def_vmin": 0.15, "solo_def_horizon_s": 3.0,
    "solo_def_margin": 0.3, "solo_third": 0.333, "ball_decel": 0.45,
    "intercept_max_s": 2.0, "intercept_vmin": 0.15, "intercept_near_m": 0.4, "return_detour": 1, "return_block": 1,
    "solo_zone": 0.0, "return_back_vx": 0.25,
    "plan": 0, "plan_radius": 0.30, "plan_stage_m": 0.12, "plan_lookahead": 0.35, "plan_wz": 1.8,
    "plan_replan_m": 0.20, "plan_replan_deg": 20, "plan_dev_m": 0.30, "plan_min_s": 0.5, "plan_horizon_s": 2.5,
    "plan_eta_frac": 0.7, "plan_near_m": 0.50, "plan_near_lat": 0.35, "plan_spin_deg": 80, "plan_look_s": 0.5,
    "plan_flip_s": 0.4,
    "coast_s": 0.0, "coast_vx": 0.25, "coast_tau_s": 0.5, "fast_yaw_bias_deg": 0.0, "fast_yaw_vx": 0.5,
}


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def unit(x: float, y: float) -> tuple[float, float]:
    d = math.hypot(x, y)
    return (x / d, y / d) if d > 1e-9 else (1.0, 0.0)


def seg_dist(p, q, c) -> float:
    """Distance from point c to segment p-q."""
    vx, vy = q[0] - p[0], q[1] - p[1]
    L2 = vx * vx + vy * vy
    s = 0.0 if L2 < 1e-12 else clamp(((c[0] - p[0]) * vx + (c[1] - p[1]) * vy) / L2, 0.0, 1.0)
    return math.hypot(p[0] + s * vx - c[0], p[1] + s * vy - c[1])


_TAU = 2 * math.pi


def dubins_words(q0, q1, R: float) -> list:
    """Dubins paths (word, (t, p, q)) from pose q0 = (x, y, yaw) to q1 with turning radius R, shortest
    first. Arc lengths in radians, the straight part in units of R."""
    dx, dy = (q1[0] - q0[0]) / R, (q1[1] - q0[1]) / R
    d = math.hypot(dx, dy)
    th = math.atan2(dy, dx) if d > 1e-9 else 0.0
    a, b = (q0[2] - th) % _TAU, (q1[2] - th) % _TAU
    sa, sb, ca, cb, cab = math.sin(a), math.sin(b), math.cos(a), math.cos(b), math.cos(a - b)
    out = []
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if p2 >= 0:
        t1 = math.atan2(cb - ca, d + sa - sb)
        out.append(("LSL", ((-a + t1) % _TAU, math.sqrt(p2), (b - t1) % _TAU)))
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
    if p2 >= 0:
        t1 = math.atan2(ca - cb, d - sa + sb)
        out.append(("RSR", ((a - t1) % _TAU, math.sqrt(p2), (-b + t1) % _TAU)))
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if p2 >= 0:
        pp = math.sqrt(p2)
        t2 = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, pp)
        out.append(("LSR", ((-a + t2) % _TAU, pp, (-b + t2) % _TAU)))
    p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
    if p2 >= 0:
        pp = math.sqrt(p2)
        t2 = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, pp)
        out.append(("RSL", ((a - t2) % _TAU, pp, (b - t2) % _TAU)))
    c = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8
    if abs(c) <= 1:
        pp = (_TAU - math.acos(c)) % _TAU
        t = (a - math.atan2(ca - cb, d - sa + sb) + pp / 2) % _TAU
        out.append(("RLR", (t, pp, (a - b - t + pp) % _TAU)))
    c = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8
    if abs(c) <= 1:
        pp = (_TAU - math.acos(c)) % _TAU
        t = (-a - math.atan2(ca - cb, d + sa - sb) + pp / 2) % _TAU
        out.append(("LRL", (t, pp, (b - a - t + pp) % _TAU)))
    return sorted(out, key=lambda w: sum(w[1]))


def dubins_points(q0, word: str, seg, R: float, ds: float = 0.05) -> list:
    """Sample a Dubins path every ~ds metres: [(x, y), ...] starting at q0."""
    x, y, h = q0
    pts = [(x, y)]
    for typ, L in zip(word, seg):
        n = max(1, int(math.ceil(L * R / ds)))
        st = L / n
        for _ in range(n):
            if typ == "S":
                x, y = x + R * st * math.cos(h), y + R * st * math.sin(h)
            else:
                k = 1.0 if typ == "L" else -1.0
                nh = h + k * st
                x, y, h = x + k * R * (math.sin(nh) - math.sin(h)), y - k * R * (math.cos(nh) - math.cos(h)), nh
            pts.append((x, y))
    return pts


def to_body(pose: Pose, wx: float, wy: float) -> tuple[float, float]:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return c * wx + s * wy, -s * wx + c * wy


def shape(cmd: Cmd, prev: Cmd, dt: float, params: dict, accel_vx: float | None = None) -> Cmd:
    """The one output shaper: clamp to max_vx/vy/wz, then rate-limit each axis from `prev`
    (accel_vx/vy/wz per second; vx and wz toward zero at decel_vx / decel_wz if set). An
    all-zero command is a stop and passes through at once.
    The caller keeps the returned Cmd as the next `prev`; missing keys fall back to DEFAULTS."""
    if cmd.vx == 0.0 and cmd.vy == 0.0 and cmd.wz == 0.0:
        return cmd
    q = lambda k: params.get(k, DEFAULTS[k])  # noqa: E731
    dt = clamp(dt, 0.0, 0.25)
    ax, ay, aw = (accel_vx or q("accel_vx")) * dt, q("accel_vy") * dt, q("accel_wz") * dt
    if q("decel_vx") > 0 and accel_vx is None and prev.vx * (cmd.vx - prev.vx) < 0:
        ax = q("decel_vx") * dt  # slowing down: the walker stops in ~0.4 s, so braking may be quicker
    if q("decel_wz") > 0 and prev.wz * (cmd.wz - prev.wz) < 0:
        aw = q("decel_wz") * dt
    vx = clamp(cmd.vx, -q("max_vx"), q("max_vx"))
    vy = clamp(cmd.vy, -q("max_vy"), q("max_vy"))
    wz = clamp(cmd.wz, -q("max_wz"), q("max_wz"))
    return Cmd(prev.vx + clamp(vx - prev.vx, -ax, ax), prev.vy + clamp(vy - prev.vy, -ay, ay),
               prev.wz + clamp(wz - prev.wz, -aw, aw), cmd.tag)


UNCLEAN_TAGS = ("push", "square", "charge", "backoff", "unstick", "line_up", "ball_lost")  # never yaw_learn from these


class Brain:
    def __init__(self, cfg: RobotCfg, field: Field):
        self.cfg, self.f = cfg, field
        self.reset()

    # goal x read from the live Field on every use (the app resizes it from the camera calibration)
    @property
    def own_x(self) -> float:
        return self.f.goal_center(self.cfg.team, own=True)[0]

    @property
    def att_x(self) -> float:
        return self.f.goal_center(self.cfg.team, own=False)[0]

    def reset(self, forget_yaw: bool = False) -> None:
        self.role: str | None = None
        self.role_t = -1e9
        self.gap: float | None = None  # filtered (my - opponent) travel-time gap, s
        self.role_cost_t = 0.0
        self.out = Cmd()
        self.last_t: float | None = None
        self.hist: list[tuple[float, float, float, float, bool]] = []  # (t, vx, vy, wz, clean)
        self.track: list[tuple[float, float, float, float]] = []  # own (t, x, y, tag yaw) observations
        if forget_yaw or not hasattr(self, "yaw_bias"):
            self.yaw_off = 0.0
            self.yaw_bias = [0.0] * 8  # learned tag-heading correction per 45-deg facing bin (kept across resets)
        self.clean = False  # this tick's command is a yaw_learn candidate (no contact, nobody close)
        self.behind = False
        self.orbit_side = 0.0
        self.side_until = -1e9
        self.goto_done = False
        self.pushing = False
        self.push_side = 0.0      # latched push offset side: +1 robot left of the shot line (ball on its right)
        self.hb: dict | None = None
        self.returning = False
        self.clearing = False
        self.stuck_ref: tuple | None = None  # (t, x, y) where forward progress was last seen
        self.unstick_until = -1.0
        self.fast = False         # commanding the walker's fast gait (vx >= gait_hi)
        self.upshift_t = -1e9     # earliest time the next up-shift is allowed
        self.plan: dict | None = None  # committed attack path (plan mode)
        self.near = False         # plan mode: close behind the ball, the push controller has it
        self.plan_fail_t = -1e9
        self.replans = 0          # paths planned since reset (sim_eval: replans/min)
        self.replan_why = "new"   # why the current plan was made (dbg)

    # ------------------------------------------------------------------ entry
    def decide(self, world: World, params: dict) -> tuple[Cmd, dict]:
        p = {**DEFAULTS, **params}
        t = world.t
        dt = 1.0 / p["loop_hz"] if self.last_t is None else clamp(t - self.last_t, 0.0, 0.25)
        self.last_t = t
        me = world.robots.get(self.cfg.id)
        if me is None or t - me.t > p["pose_stale_s"]:
            self.out, self.hb, self.pushing, self.push_side = Cmd.stop("pose_stale"), None, False, 0.0
            self.hist.clear()
            return self.out, {"role": "idle", "phase": "pose_stale", "target": None}
        if 0 < p["coast_s"] < t - me.t and p["role_mode"] != "goto" and (self.out.vx or self.out.vy or self.out.wz):
            # pose too old to steer on: keep going the way we were, slowing down (dead reckoning)
            k = math.exp(-dt / max(1e-3, p["coast_tau_s"]))
            vx = self.out.vx if self.out.vx <= p["coast_vx"] else p["coast_vx"] + k * (self.out.vx - p["coast_vx"])
            cmd = self._smooth(Cmd(vx, k * self.out.vy, k * self.out.wz, "coast"), dt, p, t)
            return cmd, {"role": self.role or "idle", "phase": "coast", "target": None,
                         "pose_age": round(t - me.t, 2)}
        pose = self._predict(self._unbias(me.pose, me.t, p), me.t, t, p)
        others = [o.pose for k, o in world.robots.items()
                  if k != self.cfg.id and t - o.t < p["other_stale_s"]]
        ball = world.ball
        f = self.f
        near_r = f.ball_radius + f.robot_radius + 0.08
        self.clean = not any(math.hypot(o.x - pose.x, o.y - pose.y) < 2 * f.robot_radius + 0.15 for o in others)
        if p["role_mode"] == "goto":
            self.role, self.hb, self.pushing = "goto", None, False
            want, dbg = self._goto_test(pose, others, p)
            cmd = self._smooth(want, dt, p, t)
            dbg.update(role="goto", pose=[round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 3)])
            return cmd, dbg
        if ball is None or t - ball.t > p["ball_stale_s"]:
            self.hb, self.pushing, self.push_side = None, False, 0.0
            want = Cmd(0.0, 0.0, p["search_wz"] if p["ball_lost"] == "search" else 0.0, "ball_lost")
            cmd = self._smooth(want, dt, p, t)
            return cmd, {"role": self.role or "idle", "phase": "ball_lost_" + p["ball_lost"], "target": None}
        h = clamp(t - ball.t, 0.0, p["ball_lead_max_s"])
        B = (clamp(ball.x + ball.vx * h, -f.length / 2, f.length / 2),
             clamp(ball.y + ball.vy * h, -f.width / 2, f.width / 2))
        self.clean = self.clean and math.hypot(pose.x - B[0], pose.y - B[1]) > near_r
        bv = (ball.vx, ball.vy)
        role, costs = self._role(pose, B, others, t, p, bv)
        if role != "attack":
            self.plan = None
            if not self.clearing:
                self.push_side = 0.0
        if role == "attack" and p["plan"] and p["contact_mode"] == "push" and t - self.plan_fail_t >= p["plan_min_s"]:
            self.returning = self.clearing = False
            want, dbg = self._attack_plan(pose, B, bv, others, p, t)
        elif role == "attack":
            self.returning = self.clearing = False
            Bi = self._intercept(pose, B, bv, p)
            want, dbg = self._attack(pose, Bi, others, p, t)
            if Bi is not B:
                dbg["intercept"] = [round(Bi[0], 3), round(Bi[1], 3)]
        else:
            want, dbg = self._defend(pose, B, bv, others, p, t)
        want, dbg = self._unstick(pose, want, dbg, t, p)
        if not (p["return_detour"] and dbg["phase"] == "return"):
            want = self._pace(pose, B, want, p)
        cmd = self._smooth(want, dt, p, t, accel_vx=p["hb_accel"] if dbg["phase"] in ("charge", "backoff") else None)
        dbg.update(role=role, pose=[round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 3)],
                   ball=[round(B[0], 3), round(B[1], 3)], **costs)
        if p["yaw_learn"]:
            dbg["yaw_corr_deg"] = round(math.degrees(self._bias_at(pose.yaw, math.radians(p["yaw_learn_max_deg"]))), 1)
        return cmd, dbg

    def sent(self, cmd: Cmd) -> None:
        """Tell the brain what actually went to the robot when the caller overrode it (stale stop,
        teleop, shaping): the next ramp starts from there and pose prediction uses it."""
        self.out = cmd
        if self.hist:
            t0, *_rest = self.hist[-1]
            self.hist[-1] = (t0, cmd.vx, cmd.vy, cmd.wz, False)

    def _goto_test(self, pose: Pose, others, p: dict):
        """role_mode goto: walk to (goto_x, goto_y) with the play controller, then face goto_yaw."""
        tgt = (float(p.get("goto_x", 0.0)), float(p.get("goto_y", 0.0)))
        yaw = p.get("goto_yaw")
        yaw = float(yaw) if isinstance(yaw, (int, float)) and math.isfinite(yaw) else None
        d = math.hypot(tgt[0] - pose.x, tgt[1] - pose.y)
        tol = p["goto_tol_m"]
        self.goto_done = d < (2 * tol if self.goto_done else tol)
        dbg = {"phase": "goto", "target": [round(tgt[0], 3), round(tgt[1], 3)], "dist": round(d, 3), "done": False}
        if not self.goto_done:
            return self._goto(pose, tgt, p, p["max_vx"], others, slow_r=0.3, allow_back=True, tag="goto"), dbg
        err = 0.0 if yaw is None else wrap(yaw - pose.yaw)
        if abs(err) > math.radians(p["goto_yaw_tol_deg"]):
            dbg["phase"] = "goto_turn"
            return Cmd(0.0, 0.0, p["k_turn"] * err, "goto_turn"), dbg
        dbg.update(phase="goto_done", done=True)
        return Cmd.stop("goto_done"), dbg

    # ------------------------------------------------------------------ state estimation
    def _bias_at(self, yaw: float, lim: float = math.radians(20)) -> float:
        x = (yaw % (2 * math.pi)) / (math.pi / 4)
        i = int(x) % 8
        w = x - int(x)
        return clamp(self.yaw_off + (1 - w) * self.yaw_bias[i] + w * self.yaw_bias[(i + 1) % 8], -lim, lim)

    def _unbias(self, pose: Pose, t_obs: float, p: dict) -> Pose:
        """Learn the overhead tag's heading bias from clean straight walking: positions are
        unbiased, so the direction of travel is the true heading. Only 0.6 s windows where every
        command was forward, straight (wz ~ 0, vy ~ 0) and clean (no ball/robot contact, no
        push/charge/unstick) count. The total correction is clamped to +-yaw_learn_max_deg."""
        if not p["yaw_learn"]:
            return pose
        lim = math.radians(p["yaw_learn_max_deg"])
        if not self.track or t_obs > self.track[-1][0]:
            self.track.append((t_obs, pose.x, pose.y, pose.yaw))
            self.track = [o for o in self.track if o[0] > t_obs - 1.0]
            old = next((o for o in self.track if o[0] <= t_obs - 0.6), None)
            cmds = [h for h in self.hist if old and old[0] - p["act_lag_s"] <= h[0] <= t_obs - p["act_lag_s"]]
            n = len(cmds)
            if (old and n >= 5 and all(h[4] for h in cmds) and min(h[1] for h in cmds) > 0.08
                    and max(abs(h[3]) for h in cmds) < 0.25 and max(abs(h[2]) for h in cmds) < 0.03):
                dx, dy = pose.x - old[1], pose.y - old[2]
                mvx = sum(h[1] for h in cmds) / n
                expect = mvx * (t_obs - old[0])
                if 0.07 < math.hypot(dx, dy) < 0.35 and math.hypot(dx, dy) > 0.5 * expect:  # moved freely, no teleport
                    tag = math.atan2(math.sin(pose.yaw) + math.sin(old[3]), math.cos(pose.yaw) + math.cos(old[3]))
                    err = wrap(math.atan2(dy, dx) - tag)
                    if abs(err) < math.radians(35):
                        r = p["yaw_learn_rate"]
                        resid = err - self._bias_at(tag, lim)
                        self.yaw_off = clamp(self.yaw_off + 0.5 * r * resid, -lim, lim)  # common offset
                        x = (tag % (2 * math.pi)) / (math.pi / 4)
                        i, w = int(x) % 8, x - int(x)
                        for j, wj in ((i, 1 - w), ((i + 1) % 8, w)):  # facing-dependent part
                            self.yaw_bias[j] = clamp(self.yaw_bias[j] + r * wj * resid, -lim, lim)
        return Pose(pose.x, pose.y, wrap(pose.yaw + self._bias_at(pose.yaw, lim)))

    def _predict(self, pose: Pose, t_obs: float, t: float, p: dict) -> Pose:
        """Roll the observed pose forward by commands executed after the camera saw it."""
        lag = p["act_lag_s"]
        self.hist = [hc for hc in self.hist if hc[0] > t - 2.0]
        fb, fv = math.radians(p["fast_yaw_bias_deg"]), p["fast_yaw_vx"]

        def bias(vx):  # the fast gait walks off its tag heading: steer by the walking direction
            return fb if fb and vx > fv else 0.0

        if not p["predict_self"] or not self.hist:
            return Pose(pose.x, pose.y, wrap(pose.yaw + bias(self.out.vx))) if fb else pose
        x, y, yaw = pose.x, pose.y, pose.yaw
        lo, hi = t_obs - lag, t - lag
        v_now = self.hist[0][1]
        for i, (ti, vx, vy, wz, _clean) in enumerate(self.hist):
            if ti <= hi:
                v_now = vx
            tj = self.hist[i + 1][0] if i + 1 < len(self.hist) else t
            seg = min(tj, hi) - max(ti, lo)
            if seg <= 0:
                continue
            ym = yaw + 0.5 * wz * seg + bias(vx)
            c, s = math.cos(ym), math.sin(ym)
            x += (c * vx - s * vy) * seg
            y += (s * vx + c * vy) * seg
            yaw += wz * seg
        return Pose(x, y, wrap(yaw + bias(v_now)))

    # ------------------------------------------------------------------ roles
    def _cost(self, pose: Pose, B, goal_x: float, p: dict) -> float:
        """Rough travel time (s) until this robot can push the ball toward goal_x."""
        f = self.f
        ux, uy = self._feasible_dir(B, unit(goal_x - B[0], -B[1]), p)
        c = f.ball_radius + f.robot_radius
        rx, ry = pose.x - B[0], pose.y - B[1]
        a, l = rx * ux + ry * uy, -rx * uy + ry * ux
        if a < 0 and abs(l) < 0.5 * c:  # already behind the ball, on the line: walk into it
            d, head = max(0.0, -a - c), math.atan2(uy, ux)
        else:
            S = (B[0] - ux * (c + 0.05), B[1] - uy * (c + 0.05))
            d = math.hypot(S[0] - pose.x, S[1] - pose.y)
            head = math.atan2(S[1] - pose.y, S[0] - pose.x) if d > 0.15 else math.atan2(uy, ux)
            d += 1.5 * max(0.0, a + 0.5 * c)  # must walk around the ball
        return d / p["max_vx"] + abs(wrap(head - pose.yaw)) / p["max_wz"]

    # ------------------------------------------------------------------ ball prediction
    def _ball_at(self, B, bv, h: float, p: dict):
        """Ball position after h seconds: straight line, constant rolling deceleration, kept on the field."""
        sp = math.hypot(bv[0], bv[1])
        if sp < 1e-6 or h <= 0:
            return B
        a = max(1e-6, p["ball_decel"])
        h = min(h, sp / a)
        s = (sp * h - 0.5 * a * h * h) / sp
        f, r = self.f, self.f.ball_radius
        return (clamp(B[0] + bv[0] * s, -f.length / 2 + r, f.length / 2 - r),
                clamp(B[1] + bv[1] * s, -f.width / 2 + r, f.width / 2 - r))

    def _arrival(self, B, bv, x_line: float, p: dict):
        """(seconds, y) when the decelerating ball crosses x = x_line, or (inf, None) if it stops first."""
        if abs(bv[0]) < 1e-6 or (x_line - B[0]) * bv[0] <= 0:
            return math.inf, None
        sp = math.hypot(bv[0], bv[1])
        L = (x_line - B[0]) / bv[0] * sp  # path length to the line
        a = max(1e-6, p["ball_decel"])
        disc = sp * sp - 2 * a * L
        if disc < 0:
            return math.inf, None
        return (sp - math.sqrt(disc)) / a, B[1] + bv[1] * (x_line - B[0]) / bv[0]

    def _intercept(self, pose: Pose, B, bv, p: dict):
        """Aim point for the attacker: where a moving ball will be when we get there (fixed point of
        our travel time, capped at intercept_max_s). Slow balls: the ball itself."""
        f = self.f
        gap = math.hypot(pose.x - B[0], pose.y - B[1]) - f.ball_radius - f.robot_radius
        if (p["intercept_max_s"] <= 0 or math.hypot(bv[0], bv[1]) < p["intercept_vmin"]
                or self.pushing or gap < p["intercept_near_m"]):  # dribbling / at the ball: play the ball itself
            return B
        P = B
        for _ in range(3):
            P = self._ball_at(B, bv, min(p["intercept_max_s"], self._cost(pose, P, self.att_x, p)), p)
        return P

    def _solo_role(self, B, bv, p: dict) -> str:
        """One robot vs a human: defend while the ball threatens our goal, attack otherwise."""
        s = 1.0 if self.own_x > 0 else -1.0
        depth = abs(self.own_x) - s * B[0]
        approach = s * bv[0]
        vmin = p["solo_def_vmin"] * (0.5 if self.role == "defend" else 1.0)
        in_third = depth < p["solo_third"] * self.f.length
        if approach > vmin:
            T, y = self._arrival(B, bv, self.own_x, p)
            on_target = y is not None and abs(y) < self.f.goal_width / 2 + p["solo_def_margin"]
            if on_target and (T <= p["solo_def_horizon_s"] or in_third):
                return "defend"
        if in_third and approach > -vmin and math.hypot(bv[0], bv[1]) > vmin:
            return "defend"  # moving (not away) inside our third
        if self.role == "defend" and p["solo_zone"] > 0 and depth > p["solo_zone"] * self.f.length:
            return "defend"  # keeper first: leave the goal only for a ball inside the home zone
        return "attack"

    def _role(self, pose: Pose, B, others, t: float, p: dict, bv=(0.0, 0.0)):
        mode = p["role_mode"]
        if mode in ("attack", "defend"):
            self.role = mode
            return mode, {}
        hold = p["role_hold_s"]
        if not others:
            want, costs = (self._solo_role(B, bv, p), {"solo": 1}) if p["solo_roles"] else ("attack", {})
            hold = p["solo_hold_s"] if p["solo_roles"] else 0.0
        else:
            my = self._cost(pose, B, self.att_x, p)
            opp = min(self._cost(o, B, self.own_x, p) for o in others)
            # low-pass the cost gap so tag noise and turning-in-place do not flip roles
            k = min(1.0, (t - self.role_cost_t) / max(1e-3, p["role_tau_s"])) if self.role_cost_t else 1.0
            self.role_cost_t = t
            self.gap = (my - opp) if self.gap is None else self.gap + k * ((my - opp) - self.gap)
            my, opp = opp + self.gap, opp
            s = 1.0 if self.own_x > 0 else -1.0
            in_third = abs(self.own_x) - s * B[0] < self.f.length / 3
            near = math.hypot(pose.x - B[0], pose.y - B[1]) <= min(math.hypot(o.x - B[0], o.y - B[1]) for o in others)
            if in_third and near:
                want = "attack"
            elif self.role == "attack":
                want = "defend" if my > opp + p["role_hyst_s"] else "attack"
            else:
                want = "attack" if my <= opp else "defend"
            costs = {"my_cost": round(my, 2), "opp_cost": round(opp, 2)}
        if want != self.role and (self.role is None or t - self.role_t >= hold):
            self.role, self.role_t = want, t
        return self.role, costs

    # ------------------------------------------------------------------ motion primitives
    def _clamp_field(self, q, p: dict):
        m, f = p["field_margin"], self.f
        return (clamp(q[0], -f.length / 2 + m, f.length / 2 - m), clamp(q[1], -f.width / 2 + m, f.width / 2 - m))

    def _repel(self, pose: Pose, others, p: dict):
        rx = ry = 0.0
        r0, rmin = p["avoid_radius"], 2 * self.f.robot_radius
        for o in others:
            dx, dy = pose.x - o.x, pose.y - o.y
            d = math.hypot(dx, dy)
            if d >= r0 or d < 1e-6:
                continue
            k = p["avoid_gain"] * min(2.0, (r0 - d) / max(1e-3, r0 - rmin))
            ux, uy = dx / d, dy / d
            # rotate the push-away 30 deg clockwise: both robots veer to their right, so no head-on stall
            c, s = math.cos(-0.5), math.sin(-0.5)
            rx += k * (c * ux - s * uy)
            ry += k * (s * ux + c * uy)
        return rx, ry

    def _goto(self, pose: Pose, tgt, p: dict, vmax: float, others=(), avoid: float = 1.0,
              slow_r: float = 0.0, allow_back: bool = False, lateral: bool = True, tag: str = "",
              back_blend: float = 0.0) -> Cmd:
        dx, dy = tgt[0] - pose.x, tgt[1] - pose.y
        d = math.hypot(dx, dy)
        ux, uy = (dx / d, dy / d) if d > 1e-6 else (math.cos(pose.yaw), math.sin(pose.yaw))
        rx, ry = self._repel(pose, others, p)
        hx, hy = ux + avoid * rx, uy + avoid * ry
        err = wrap(math.atan2(hy, hx) - pose.yaw)
        speed = vmax * (min(1.0, d / slow_r) if slow_r > 0 else 1.0)
        sign = 1.0
        if allow_back and abs(err) > 2.0 and d < 0.4:
            err, sign = wrap(err + math.pi), -1.0
        c = max(0.0, math.cos(err))
        vx = sign * speed * c * c
        if back_blend > 0 and sign > 0 and c == 0.0:  # target behind: back toward it while turning round
            vx = -min(speed, back_blend) * math.cos(err) ** 2
        vy = 0.0
        if lateral and d < 0.25:
            vy = p["k_vy"] * to_body(pose, dx, dy)[1]
        return Cmd(vx, vy, p["k_turn"] * err, tag)

    # ------------------------------------------------------------------ attack
    def _frame(self, pose: Pose, B, p: dict, stage_d: float = 0.0):
        f = self.f
        hw = f.goal_width / 2 - f.ball_radius
        aim = (self.att_x, clamp(B[1] * p["aim_y_frac"], -hw, hw))
        ux, uy = self._feasible_dir(B, unit(aim[0] - B[0], aim[1] - B[1]), p, stage_d)
        nx, ny = -uy, ux
        rx, ry = pose.x - B[0], pose.y - B[1]
        a, l = rx * ux + ry * uy, rx * nx + ry * ny
        return (ux, uy), (nx, ny), a, l, math.atan2(l, a)

    def _feasible_dir(self, B, u0, p: dict, stage_d: float = 0.0):
        """Smallest rotation of the push direction whose stage point behind the ball is reachable.
        A ball on a wall gets pushed along it (slightly into it) instead of an unreachable shot line."""
        f, m = self.f, p["field_margin"]
        d = stage_d or f.ball_radius + f.robot_radius + 0.03
        hl, hw = f.length / 2 - m, f.width / 2 - m
        best, best_out = u0, 1e9
        for k in range(0, 37):
            rot = math.radians(5 * ((k + 1) // 2) * (1 if k % 2 else -1))
            c, s = math.cos(rot), math.sin(rot)
            u = (c * u0[0] - s * u0[1], s * u0[0] + c * u0[1])
            sx, sy = B[0] - u[0] * d, B[1] - u[1] * d
            out = max(0.0, abs(sx) - hl) + max(0.0, abs(sy) - hw)
            if out <= 0.0:
                return u
            if out < best_out - 1e-9:
                best, best_out = u, out
        return best

    def _attack(self, pose: Pose, B, others, p: dict, t: float):
        f = self.f
        c = f.ball_radius + f.robot_radius
        hb_mode = p["contact_mode"] == "headbutt"
        u, n, a, l, phi = self._frame(pose, B, p, c + p["hb_standoff"] if hb_mode else 0.0)
        clear = c + p["clear_margin"]
        cone = math.radians(p["behind_cone_deg"])
        self.behind = abs(phi) > (cone - math.radians(15) if self.behind else cone)
        if hb_mode:
            return self._headbutt(pose, B, u, n, a, l, phi, clear, others, p, t)
        self.hb = None
        if not self.behind:
            self.pushing, self.push_side = False, 0.0
            tgt = self._route(pose, B, u, n, phi, clear, p, t)
            return self._goto(pose, tgt, p, p["max_vx"], others, tag="route"), \
                {"phase": "route", "target": [round(tgt[0], 3), round(tgt[1], 3)], "lat": round(l, 3)}
        herr = wrap(math.atan2(u[1], u[0]) - pose.yaw)
        d_side = self._side(l, herr, p)
        l0, l = l, l - d_side  # lateral error from the (offset) push line
        tol = p["push_lat_tol"]
        on_line = abs(l) < (2 * tol if self.pushing else tol)
        dB = math.hypot(pose.x - B[0], pose.y - B[1])
        close = dB < c + 0.15
        lim = p["push_max_heading_deg"]
        if lim > 0 and on_line and close and abs(herr) > math.radians(lim + (15 if self.pushing else 0)):
            # pushes that began 50-130 deg off the shot line knocked the ball sideways: square up first,
            # stepping back while the body is within clear_margin of the ball so turning can't clip it
            self.pushing = False  # (the side stays latched: squaring is part of the same push)
            bx, _ = to_body(pose, B[0] - pose.x, B[1] - pose.y)
            vx = -p["unstick_vx"] if dB < clear and bx > 0 else 0.0
            return Cmd(vx, clamp(-p["k_vy"] * l, -p["max_vy"], p["max_vy"]), p["k_turn"] * herr, "square"), \
                {"phase": "square", "target": [round(B[0], 3), round(B[1], 3)], "lat": round(l, 3)}
        if self.pushing and not on_line and p["push_side_m"] > 0:  # push ended: choose the side afresh
            self.push_side = 0.0
            d_side = self._side(l0, herr, p)
            l = l0 - d_side
            on_line = abs(l) < tol
        self.pushing = on_line
        # unified curved approach: far off the line -> stage behind the ball (k_curve*|l|);
        # on the line -> aim push_ahead beyond the ball and keep walking through it
        w = min(1.0, abs(l) / (2 * tol))
        back = p["k_curve"] * abs(l) * w - p["push_ahead"] * (1.0 - w)
        tgt = (B[0] - u[0] * back + n[0] * d_side, B[1] - u[1] * back + n[1] * d_side)
        if back > 0:
            tgt = self._clamp_field(tgt, p)
        phase = "push" if self.pushing and close else "approach"
        cmd = self._goto(pose, tgt, p, p["push_vx"] if self.pushing else p["max_vx"], others,
                         avoid=0.3 if phase == "push" else 1.0, lateral=False, tag=phase)
        if self.pushing:  # slide onto the shot line with the weak lateral axis
            cmd = Cmd(cmd.vx, clamp(-p["k_vy"] * l, -p["max_vy"], p["max_vy"]) * (1 - w),
                      cmd.wz * p["push_turn_scale"], phase)
        dbg = {"phase": phase, "target": [round(tgt[0], 3), round(tgt[1], 3)], "lat": round(l, 3)}
        if d_side:
            dbg["side"] = round(d_side, 3)
        return cmd, dbg

    def _side(self, l: float, herr: float, p: dict) -> float:
        """Signed lateral offset (m, along n) of the push line; 0 when push_side_m is off. The side is latched
        in self.push_side (+1 = robot left of the shot line, ball on its right) until reset."""
        d = p["push_side_m"]
        if d <= 0:
            self.push_side = 0.0
            return 0.0
        if not self.push_side:
            mode = p["push_side"]
            if mode == "left":     # ball on the robot's left -> robot right of the line
                self.push_side = -1.0
            elif mode == "right":
                self.push_side = 1.0
            elif abs(l) > 0.02:    # auto: the side we are already on
                self.push_side = 1.0 if l > 0 else -1.0
            else:                  # on the line: the side the heading already drifts to (yaw left of u -> +n)
                self.push_side = 1.0 if herr <= 0 else -1.0
        return self.push_side * d

    # ------------------------------------------------------------------ planned attack (plan mode)
    def _plan_goal(self, pose: Pose, B, bv, p: dict, eta: float | None = None):
        """(P, u): predicted intercept point (ball after our ETA, capped at plan_horizon_s) and push direction."""
        f = self.f
        P = B
        if math.hypot(bv[0], bv[1]) >= p["intercept_vmin"] and p["plan_horizon_s"] > 0:
            if eta is None:
                for _ in range(3):
                    P = self._ball_at(B, bv, min(p["plan_horizon_s"], self._cost(pose, P, self.att_x, p)), p)
            else:
                P = self._ball_at(B, bv, min(p["plan_horizon_s"], eta), p)
        hw = f.goal_width / 2 - f.ball_radius
        # aim inside the goal: a ball on the goal line still has a stable push direction
        aim = (self.att_x * (1 + f.goal_depth / max(1e-3, abs(self.att_x))), clamp(P[1] * p["aim_y_frac"], -hw, hw))
        stage = f.ball_radius + f.robot_radius + p["plan_stage_m"]
        return P, self._feasible_dir(P, unit(aim[0] - P[0], aim[1] - P[1]), p, stage)

    def _plan_path(self, pose: Pose, P, u, B, p: dict, t: float) -> dict | None:
        """Fastest collision-free path to the stage behind P (heading u), directly or via a tangent point beside
        the ball, then straight through P: Dubins arcs at plan_radius or 0.6x, optionally after a turn on the
        spot, or turn-straight-turn. None if nothing fits in the field."""
        f = self.f
        c = f.ball_radius + f.robot_radius
        R, d_st = p["plan_radius"], c + p["plan_stage_m"]
        h = math.atan2(u[1], u[0])
        n = (-u[1], u[0])
        S = (P[0] - u[0] * d_st, P[1] - u[1] * d_st, h)
        clear = c + p["clear_margin"]
        hl, hw = f.length / 2 - p["field_margin"], f.width / 2 - p["field_margin"]
        q0 = (pose.x, pose.y, pose.yaw)
        balls = [B] if math.hypot(P[0] - B[0], P[1] - B[1]) < 0.05 else [B, P]

        def out_of(x, y):
            return max(0.0, abs(x) - hl, abs(y) - hw)

        def near(x, y):  # (distance, clearance needed) to each ball point: behind the ball only no contact,
            res = []    # elsewhere the full clearance (a touch there can knock it sideways or back)
            for Q in balls:
                rx, ry = x - Q[0], y - Q[1]
                behind = rx * u[0] + ry * u[1] < -0.5 * c
                res.append((math.hypot(rx, ry), c + 0.03 if behind else clear))
            return res

        o0, n0 = out_of(*q0[:2]), near(*q0[:2])

        def ok(pts):  # never closer to the ball than clear (or than the start), never further out of the field
            for x, y in pts:
                if out_of(x, y) > o0 + 0.06:  # path points may use 6 cm of the field margin
                    return False
                for (d, need), (d0, _n) in zip(near(x, y), n0):
                    if d < min(need, d0 - 0.02):
                        return False
            return True

        vmax, wmax = p["max_vx"], p["plan_wz"]

        def turns(word, seg):  # signs of the real arcs, in order
            return [1 if ch == "L" else -1 for ch, x in zip(word, seg) if ch != "S" and x > 0.1]

        def flips(sg):
            return sum(x != y for x, y in zip(sg, sg[1:]))

        def leg(a, b, r, spin=0.0):  # fastest collision-free Dubins leg a -> b at radius r: (seconds, points);
            va = min(vmax, wmax * r)  # each planned left/right switch (spin included) costs plan_flip_s
            words = []
            for w, sg in dubins_words(a, b, r):
                tn = ([1 if spin > 0 else -1] if abs(spin) > 0.1 else []) + turns(w, sg)
                words.append(((sg[0] + sg[2]) * r / va + sg[1] * r / vmax + p["plan_flip_s"] * flips(tn), w, sg))
            for sec, word, seg in sorted(words):
                pts = dubins_points(a, word, seg, r)
                if math.hypot(pts[-1][0] - b[0], pts[-1][1] - b[1]) < 0.02 and ok(pts):
                    return sec, pts
            return None

        def line(a, b, spin0):  # already facing b: walk straight, turn on the spot at the end (we are not a car)
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            k = max(1, int(L / 0.05))
            pts = [(a[0] + (b[0] - a[0]) * j / k, a[1] + (b[1] - a[1]) * j / k) for j in range(k + 1)]
            end = wrap(b[2] - a[2])
            flip = p["plan_flip_s"] if abs(end) > 0.1 and abs(spin0) > 0.1 and end * spin0 < 0 else 0.0
            return (L / vmax + abs(end) / wmax + flip, pts) if ok(pts) else None

        rr = 1.15 * clear
        sd = math.copysign(1.0, (pose.x - P[0]) * n[0] + (pose.y - P[1]) * n[1])
        vias = [None]
        for side in (sd, -sd):
            for psi in (math.radians(110), math.radians(150)):
                tx = -math.sin(psi) * u[0] + side * math.cos(psi) * n[0]
                ty = -math.sin(psi) * u[1] + side * math.cos(psi) * n[1]
                vias.append((P[0] + rr * (math.cos(psi) * u[0] + side * math.sin(psi) * n[0]),
                             P[1] + rr * (math.cos(psi) * u[1] + side * math.sin(psi) * n[1]), math.atan2(ty, tx)))
        best, cache = None, {}
        for r in (R, 0.6 * R):
            for W in vias:
                first = W or S
                face = math.atan2(first[1] - pose.y, first[0] - pose.x)
                for yaw0 in {pose.yaw, face}:  # as we are, or spin in place to face the first target
                    spin = abs(wrap(yaw0 - pose.yaw)) / wmax
                    if best and spin >= best[0]:
                        continue
                    a = leg((pose.x, pose.y, yaw0), first, r, wrap(yaw0 - pose.yaw))
                    if yaw0 == face and r == R:
                        b2 = line((pose.x, pose.y, yaw0), first, wrap(yaw0 - pose.yaw))
                        a = b2 if a is None or (b2 and b2[0] < a[0]) else a
                    if a is None or (best and spin + a[0] >= best[0]):
                        continue
                    if W is not None:
                        if (W, r) not in cache:
                            cache[(W, r)] = leg(W, S, r)
                        b2 = cache[(W, r)]
                        if b2 is None:
                            continue
                        a = (a[0] + b2[0], a[1] + b2[1][1:])
                    if best is None or spin + a[0] < best[0]:
                        best = (spin + a[0], a[1], yaw0)
        if best is None:
            return None
        _sec, best, yaw0 = best
        tail = d_st + p["push_ahead"]
        best += [(S[0] + u[0] * k * 0.05, S[1] + u[1] * k * 0.05) for k in range(1, int(tail / 0.05) + 1)]
        s = [0.0]
        for (x0, y0), (x1, y1) in zip(best, best[1:]):
            s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
        self.replans += 1
        return {"P": P, "u": u, "pts": best, "s": s, "i": 0, "sP": s[-1] - p["push_ahead"], "t": t,
                "yaw0": yaw0, "spun": abs(wrap(yaw0 - pose.yaw)) < 0.3}

    def _attack_plan(self, pose: Pose, B, bv, others, p: dict, t: float):
        """Plan mode attack: commit to a path, follow it with pure pursuit, drive through the ball."""
        f = self.f
        c = f.ball_radius + f.robot_radius
        self.hb = None
        u, n, a, l, _phi = self._frame(pose, B, p)
        gap = math.hypot(pose.x - B[0], pose.y - B[1]) - c
        herr = abs(wrap(math.atan2(u[1], u[0]) - pose.yaw))
        side_v = abs(bv[1] * u[0] - bv[0] * u[1])  # ball speed across the shot line: the line itself is moving
        if a < 0 and side_v < (0.4 if self.near else 0.25) and ((gap < p["plan_near_m"] + 0.4 or (abs(l) < 2 * p["push_lat_tol"] and herr < math.radians(30)))
                       and abs(l) < p["plan_near_lat"] and herr < math.radians(70) if self.near else
                      gap < p["plan_near_m"] and abs(l) < 2 * p["push_lat_tol"] and herr < math.radians(45)):
            # behind the ball on the shot line: pure pursuit along the line through the ball (the target stays a
            # lookahead ahead, never inside the turning circle); the lateral error is mostly slid off
            self.near, self.plan = True, None
            self.pushing = gap < 0.15
            L = p["plan_lookahead"] + p["plan_look_s"] * max(0.0, self.out.vx)
            tgt = (B[0] + u[0] * (a + L), B[1] + u[1] * (a + L))
            bx, by = to_body(pose, tgt[0] - pose.x, tgt[1] - pose.y)
            alpha = math.atan2(by, bx)
            v = p["push_vx"] * max(0.0, math.cos(alpha))  # (_pace caps vx later; turning from the paced speed
            tol = p["push_lat_tol"]                        # was too weak to hold the line: 35/40 vs 40/40 goals)
            if gap < 0.25 and abs(l) > 0.5 * tol:  # off-centre at the ball: slide over before walking into it
                v *= clamp(1.0 - (abs(l) - 0.5 * tol) / tol, 0.0, 1.0)
            wz = 2.0 * v * math.sin(alpha) / max(0.1, math.hypot(bx, by))
            if abs(l) < tol:
                wz *= p["push_turn_scale"]
            phase = "push" if self.pushing else "line"
            cmd = Cmd(v, clamp(-p["k_vy"] * l, -p["max_vy"], p["max_vy"]), clamp(wz, -p["plan_wz"], p["plan_wz"]), phase)
            return cmd, {"phase": phase, "target": [round(tgt[0], 3), round(tgt[1], 3)], "lat": round(l, 3)}
        self.near = self.pushing = False
        pl = self.plan
        if pl is not None:
            tgt, dev, i = self._pursuit_point(pose, pl, p)
            eta = p["act_lag_s"] + max(0.0, pl["sP"] - pl["s"][i]) / max(0.05, p["plan_eta_frac"] * p["max_vx"])
            P, uu = self._plan_goal(pose, B, bv, p, eta)
            moved = math.hypot(P[0] - pl["P"][0], P[1] - pl["P"][1])
            turned = math.degrees(abs(wrap(math.atan2(uu[1], uu[0]) - math.atan2(pl["u"][1], pl["u"][0]))))
            done = i >= len(pl["pts"]) - 2
            rem = max(0.0, pl["sP"] - pl["s"][i])  # path left to the ball: near it, a smaller miss matters
            why = "moved" if moved > p["plan_replan_m"] * clamp(rem, 0.3, 1.0) else "turned" if turned > p["plan_replan_deg"] else \
                "off_path" if dev > p["plan_dev_m"] else "done" if done else ""
            if why and t - pl["t"] >= p["plan_min_s"]:
                pl, self.replan_why = None, why
        if pl is None:
            P, uu = self._plan_goal(pose, B, bv, p)
            pl = self._plan_path(pose, P, uu, B, p, t)
            if pl is not None and P is not B:  # one refinement with the path's own ETA
                eta = p["act_lag_s"] + pl["sP"] / max(0.05, p["plan_eta_frac"] * p["max_vx"])
                P2, u2 = self._plan_goal(pose, B, bv, p, eta)
                if math.hypot(P2[0] - P[0], P2[1] - P[1]) > 0.5 * p["plan_replan_m"]:
                    pl = self._plan_path(pose, P2, u2, B, p, t) or pl
            self.plan, pl_why, self.replan_why = pl, self.replan_why, "new"
            if pl is not None:
                pl["why"] = pl_why
            if pl is None:  # nothing fits: the old approach for a while
                self.plan_fail_t = t
                return self._attack(pose, B, others, p, t)
            tgt, dev, i = self._pursuit_point(pose, pl, p)
        rx, ry = self._repel(pose, others, p)
        L = p["plan_lookahead"] + p["plan_look_s"] * max(0.0, self.out.vx)
        bx, by = to_body(pose, tgt[0] - pose.x + L * rx, tgt[1] - pose.y + L * ry)
        alpha = math.atan2(by, bx)
        if not pl["spun"]:  # the plan starts with a turn on the spot
            err = wrap(pl["yaw0"] - pose.yaw)
            pl["spun"] = abs(err) < 0.25
            alpha = err if not pl["spun"] else alpha
        if not pl["spun"] or abs(alpha) > math.radians(p["plan_spin_deg"]):
            cmd = Cmd(0.0, 0.0, clamp(2.5 * alpha, -p["plan_wz"], p["plan_wz"]), "plan_turn")
        else:
            kap = 2.0 * math.sin(alpha) / max(0.1, math.hypot(bx, by))
            v = min(p["max_vx"], p["plan_wz"] / max(1e-6, abs(kap))) * math.cos(alpha)
            wz = math.copysign(max(abs(v * kap), p["k_turn"] * abs(alpha)), alpha)
            cmd = Cmd(v, 0.0, clamp(wz, -p["plan_wz"], p["plan_wz"]), "plan")
        pts = pl["pts"][i::max(1, -(-(len(pl["pts"]) - i) // 19))] + [pl["pts"][-1]]  # <= 20 points for the UI
        return cmd, {"phase": "plan", "target": [round(tgt[0], 3), round(tgt[1], 3)], "lat": round(l, 3),
                     "intercept": [round(pl["P"][0], 3), round(pl["P"][1], 3)], "dev": round(dev, 3), "why": pl["why"],
                     "plan": [[round(x, 3), round(y, 3)] for x, y in pts]}

    def _pursuit_point(self, pose: Pose, pl: dict, p: dict):
        """(lookahead point, distance off the path, progress index). Progress only moves forward."""
        pts, s, i0 = pl["pts"], pl["s"], pl["i"]
        dev, i = 1e9, i0
        for j in range(i0, min(len(pts), i0 + 30)):
            d = math.hypot(pts[j][0] - pose.x, pts[j][1] - pose.y)
            if d < dev:
                dev, i = d, j
        pl["i"] = i
        look = s[i] + p["plan_lookahead"] + p["plan_look_s"] * max(0.0, self.out.vx)
        j = i
        while j + 1 < len(pts) and s[j + 1] < look:
            j += 1
        if j + 1 < len(pts):
            w = (look - s[j]) / max(1e-9, s[j + 1] - s[j])
            tgt = (pts[j][0] + w * (pts[j + 1][0] - pts[j][0]), pts[j][1] + w * (pts[j + 1][1] - pts[j][1]))
        else:
            u = pl["u"]
            tgt = (pts[-1][0] + u[0] * (look - s[-1]), pts[-1][1] + u[1] * (look - s[-1]))
        return tgt, dev, i

    def _route(self, pose: Pose, B, u, n, phi: float, clear: float, p: dict, t: float):
        """Next waypoint around the ball toward the stage behind it. The way round is kept for
        orbit_hold_s; a waypoint slightly outside the field margin is clamped in, not rejected
        (rejecting it used to flip the side every tick next to a wall)."""
        rr = clear * 1.15
        S = self._clamp_field((B[0] - u[0] * rr, B[1] - u[1] * rr), p)
        R = (pose.x, pose.y)
        if math.hypot(R[0] - B[0], R[1] - B[1]) > clear and seg_dist(R, S, B) >= clear:
            return S
        f, m, step = self.f, p["field_margin"], math.radians(70)

        def wp(s):
            pw = phi + s * step
            pw = clamp(pw, -math.pi, math.pi) if s * phi >= 0 else pw
            return (B[0] + rr * (math.cos(pw) * u[0] + math.sin(pw) * n[0]),
                    B[1] + rr * (math.cos(pw) * u[1] + math.sin(pw) * n[1]))

        def inside(W, slack):
            return abs(W[0]) <= f.length / 2 - m + slack and abs(W[1]) <= f.width / 2 - m + slack

        if not self.orbit_side or t >= self.side_until:
            if abs(phi) > 0.35:
                pref = math.copysign(1.0, phi)
            else:
                pref = self.orbit_side or (-1.0 if B[1] * n[1] + B[0] * n[0] > 0 else 1.0)
            side = pref if inside(wp(pref), p["orbit_slack"]) or not inside(wp(-pref), 0.0) else -pref
            if side != self.orbit_side:
                self.side_until = t + p["orbit_hold_s"]
            self.orbit_side = side
        return self._clamp_field(wp(self.orbit_side), p)

    def _headbutt(self, pose, B, u, n, a, l, phi, clear, others, p, t):
        f = self.f
        c = f.ball_radius + f.robot_radius
        hb = self.hb or {"stage": "line"}
        self.hb = hb
        if hb["stage"] == "charge":
            moved = math.hypot(pose.x - hb["x0"], pose.y - hb["y0"])
            if moved >= p["hb_standoff"] + p["hb_charge_dist"] or t - hb["t0"] > hb["tmax"]:
                hb.update(stage="back", t0=t)
            else:  # pursue a point on the shot line locked at the start (position feedback beats tag-heading bias)
                aim = hb["aim"]
                err = wrap(math.atan2(aim[1] - pose.y, aim[0] - pose.x) - pose.yaw)
                return Cmd(p["hb_charge_vx"], 0.0, p["k_turn"] * err, "charge"), \
                    {"phase": "charge", "target": [round(B[0], 3), round(B[1], 3)], "lat": round(l, 3)}
        if hb["stage"] == "back":
            if t - hb["t0"] < p["hb_backoff_s"]:
                return Cmd(-p["hb_backoff_vx"], 0.0, 0.0, "backoff"), {"phase": "backoff", "target": None, "lat": round(l, 3)}
            hb["stage"] = "line"
        if not self.behind:
            tgt = self._route(pose, B, u, n, phi, clear, p, t)
            return self._goto(pose, tgt, p, p["max_vx"], others, tag="route"), \
                {"phase": "route", "target": [round(tgt[0], 3), round(tgt[1], 3)], "lat": round(l, 3)}
        sd = c + p["hb_standoff"]
        back = sd + p["k_curve"] * abs(l)
        tgt = self._clamp_field((B[0] - u[0] * back, B[1] - u[1] * back), p)
        dist = math.hypot(tgt[0] - pose.x, tgt[1] - pose.y)
        yaw_err = wrap(math.atan2(u[1], u[0]) - pose.yaw)
        if (abs(l) < p["hb_lat_tol"] and -(sd + 0.15) < a < -(c + 0.03)
                and abs(yaw_err) < math.radians(p["hb_align_deg"]) and abs(self.out.wz) < 0.4):
            hb.update(stage="charge", t0=t, x0=pose.x, y0=pose.y, aim=(B[0] + u[0] * 0.6, B[1] + u[1] * 0.6),
                      tmax=1.5 + (p["hb_standoff"] + p["hb_charge_dist"]) / max(0.05, p["hb_charge_vx"]))
            return Cmd(p["hb_charge_vx"], 0.0, 0.0, "charge"), \
                {"phase": "charge", "target": [round(B[0], 3), round(B[1], 3)], "lat": round(l, 3)}
        if dist < p["hb_square_r"] and abs(yaw_err) < 1.2:  # near the stage: face the line, fix position holonomically
            bx, by = to_body(pose, 1.5 * (tgt[0] - pose.x), 1.5 * (tgt[1] - pose.y))
            cmd = Cmd(clamp(bx, -0.15, 0.15), by, p["k_turn"] * yaw_err, "line_up")
        else:
            cmd = self._goto(pose, tgt, p, p["max_vx"], others, slow_r=p["hb_square_r"], tag="line_up")
        return cmd, {"phase": "line_up", "target": [round(tgt[0], 3), round(tgt[1], 3)], "lat": round(l, 3)}

    # ------------------------------------------------------------------ defend
    def _defend(self, pose: Pose, B, bv, others, p: dict, t: float):
        f = self.f
        s = 1.0 if self.own_x > 0 else -1.0
        gl = abs(self.own_x)
        depth = gl - s * B[0]  # ball distance in front of own goal line
        hyst = 0.12 if self.clearing else 0.0
        approach = s * bv[0]
        if (depth < p["clear_depth"] + hyst and abs(B[1]) < f.goal_width / 2 + p["clear_width_extra"] + hyst
                and approach < p["clear_max_speed"]):
            self.clearing, self.returning = True, False
            cmd, dbg = self._attack(pose, B, others, p, t)
            dbg["phase"] = "clear_" + dbg["phase"]
            return cmd, dbg
        self.clearing, self.pushing, self.hb = False, False, None
        x_line = self.own_x - s * p["goalie_dist"]
        y_t = B[1]
        if approach > 0.05:
            ty = (x_line - B[0]) / bv[0]
            if 0.0 < ty < 3.0:
                y_t = B[1] + bv[1] * ty
        lim = f.goal_width / 2 + p["goalie_margin"]
        G = (x_line, clamp(y_t, -lim, lim))
        dx, dy = G[0] - pose.x, G[1] - pose.y
        dist = math.hypot(dx, dy)
        self.returning = dist > (0.12 if self.returning else p["goalie_reach"])
        tgt = [round(G[0], 3), round(G[1], 3)]
        if self.returning:
            dbg = {"phase": "return", "target": tgt}
            K = self._block(pose, B, bv, p) if p["return_block"] and approach > p["solo_def_vmin"] else None
            if K is not None:  # cannot reach the line in time: step into the ball's path where we can
                dbg.update(phase="block", target=[round(K[0], 3), round(K[1], 3)])
                return self._goto(pose, K, p, min(p["max_vx"], p["return_vx"]), others, slow_r=0.2,
                                  allow_back=True, tag="block", back_blend=p["return_back_vx"]), dbg
            # a ball coming at goal is worth touching (a block); a slow one we might knock in: go round it
            W = self._detour(pose, G, B, p) if p["return_detour"] and approach < p["solo_def_vmin"] else G
            if W is not G:
                dbg["via"] = [round(W[0], 3), round(W[1], 3)]
            return self._goto(pose, W, p, min(p["max_vx"], p["return_vx"]), others,
                              slow_r=0.2 if W is G else 0.0, allow_back=True, tag="return",
                              back_blend=p["return_back_vx"]), dbg
        if p["goalie_style"] == "facing":
            yaw_des = math.atan2(B[1] - pose.y, B[0] - pose.x)
        else:  # body parallel to the goal line: forward/back walking slides along it
            yaw_des = math.pi / 2 if abs(wrap(pose.yaw - math.pi / 2)) <= math.pi / 2 else -math.pi / 2
        bx, by = to_body(pose, p["k_goalie"] * dx, p["k_goalie"] * dy)
        bx = clamp(bx, -p["goalie_vmax"], p["goalie_vmax"])
        return Cmd(bx, by, p["k_turn"] * wrap(yaw_des - pose.yaw), "guard"), {"phase": "guard", "target": tgt}

    def _block(self, pose: Pose, B, bv, p: dict):
        """Earliest point on the ball's path to our goal that we reach before the ball, if the guard line
        is not reachable in time; None when the line is (or nothing is) reachable."""
        T, _y = self._arrival(B, bv, self.own_x, p)
        if T == math.inf:
            return None
        v = 0.9 * min(p["max_vx"], p["return_vx"])

        def eta(Q):
            d = math.hypot(Q[0] - pose.x, Q[1] - pose.y)
            turn = abs(wrap(math.atan2(Q[1] - pose.y, Q[0] - pose.x) - pose.yaw)) if d > 0.15 else 0.0
            return p["act_lag_s"] + d / v + turn / p["max_wz"]

        s = 1.0 if self.own_x > 0 else -1.0
        line = self._ball_at(B, bv, self._arrival(B, bv, self.own_x - s * p["goalie_dist"], p)[0], p)
        if eta(line) <= T:
            return None
        tau = 0.2
        while tau < T:
            Q = self._ball_at(B, bv, tau, p)
            if eta(Q) <= tau:
                return self._clamp_field(Q, p)
            tau += 0.1
        return None

    def _detour(self, pose: Pose, G, B, p: dict):
        """Walking back to G with the ball in the way: pass it at clearance on our side of the path."""
        f = self.f
        clear = f.ball_radius + f.robot_radius + p["clear_margin"]
        R = (pose.x, pose.y)
        ux, uy = unit(G[0] - R[0], G[1] - R[1])
        along = (B[0] - R[0]) * ux + (B[1] - R[1]) * uy
        if seg_dist(R, G, B) >= clear or along <= 0:
            return G
        lat = -(B[0] - R[0]) * uy + (B[1] - R[1]) * ux  # ball left (+) or right of the path
        side = -1.0 if lat > 0 else 1.0
        for sd in (side, -side):
            W = (B[0] - uy * sd * 1.3 * clear, B[1] + ux * sd * 1.3 * clear)
            m = p["field_margin"]
            if abs(W[0]) <= f.length / 2 - m and abs(W[1]) <= f.width / 2 - m:
                return W
        return self._clamp_field(W, p)

    def _unstick(self, pose: Pose, want: Cmd, dbg: dict, t: float, p: dict):
        """Walking forward but not moving for stuck_s (pinned ball, wall, opponent) -> back off briefly."""
        if t < self.unstick_until:
            dbg["phase"] = "unstick"
            return Cmd(-p["unstick_vx"], 0.0, 0.0, "unstick"), dbg
        ref = self.stuck_ref
        if abs(want.vx) < 0.08 or ref is None or math.hypot(pose.x - ref[1], pose.y - ref[2]) > p["stuck_dist"]:
            self.stuck_ref = (t, pose.x, pose.y)
        elif t - ref[0] > p["stuck_s"]:
            self.unstick_until, self.stuck_ref = t + p["unstick_s"], None
            self.pushing, self.hb, self.plan = False, None, None
        return want, dbg

    # ------------------------------------------------------------------ output shaping
    def _pace(self, pose: Pose, B, want: Cmd, p: dict) -> Cmd:
        """Cap the closing speed toward the ball: max_vx at contact_slow_m surface gap, falling
        linearly to contact_vx at touch. A pass at a wide angle gets up to twice the cap."""
        if want.vx <= p["contact_vx"] or p["contact_slow_m"] <= 0:
            return want
        f = self.f
        dx, dy = B[0] - pose.x, B[1] - pose.y
        gap = math.hypot(dx, dy) - f.ball_radius - f.robot_radius
        bear = math.cos(wrap(math.atan2(dy, dx) - pose.yaw))
        if gap >= p["contact_slow_m"] or bear <= 0.0:  # far enough, or the ball is behind us
            return want
        cap = p["contact_vx"] + max(0.0, gap) / p["contact_slow_m"] * max(0.0, p["max_vx"] - p["contact_vx"])
        cap /= max(bear, 0.5)
        return want if want.vx <= cap else Cmd(cap, want.vy, want.wz, want.tag)

    def _gait(self, want: Cmd, t: float, p: dict) -> Cmd:
        """Keep the forward target out of (gait_lo, gait_hi) with hysteresis (see DEFAULTS doc)."""
        lo, hi = p["gait_lo"], p["gait_hi"]
        if not p["gait_band"] or hi <= lo:
            return want
        v = min(want.vx, p["max_vx"])
        if self.fast and v < 0.5 * (lo + hi):
            self.fast, self.upshift_t = False, t + p["gait_hold_s"]
        elif not self.fast and v >= hi and t >= self.upshift_t and p["max_vx"] >= hi:
            self.fast = True
        if self.fast:
            v = max(v, hi)
        elif v > lo:
            v = lo
        return want if v == want.vx else Cmd(v, want.vy, want.wz, want.tag)

    def _smooth(self, want: Cmd, dt: float, p: dict, t: float, accel_vx: float | None = None) -> Cmd:
        want = self._gait(want, t, p)
        self.out = shape(want, self.out, dt, p, accel_vx)
        lo, hi = p["gait_lo"], p["gait_hi"]
        if p["gait_band"] and lo < self.out.vx < hi:  # step across the band instead of ramping through it
            v = hi if want.vx >= hi else lo if want.vx <= lo else self.out.vx
            self.out = Cmd(v, self.out.vy, self.out.wz, self.out.tag)
        clean = self.clean and not want.tag.startswith(UNCLEAN_TAGS)
        self.hist.append((t, self.out.vx, self.out.vy, self.out.wz, clean))
        return self.out
