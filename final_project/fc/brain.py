"""Pupper FC v1 brain: one robot's decision on a World snapshot -> body-frame Cmd. Pure logic (stdlib only); the
same object drives the sim and the real robots.

The system, from the top:
  World (camera: robots + ball, with capture times) -> Brain.decide -> Cmd (vx, vy, wz), once per loop tick (30 Hz).
  1. Where am I? The camera sees the past (~0.1-0.2 s): roll our pose forward by the commands sent since (_predict).
     Own tag hidden: keep playing on that dead-reckoned pose for a while, slower (vs a human only, see 4); every
     mode stops once that blind walk reaches blind_max_m (1 m) since the last fix.
  2. Where is the ball? The last sighting led by its velocity; a constant-deceleration rolling model predicts it.
  3. Who does what (role)? With two robots tracked the Coach (bottom of this file) assigns a task to each robot
     (attack / challenge = play the ball; jockey / goalie / recover = shadow goal-side). Alone (vs a human) the robot
     defends while the ball threatens its goal and attacks otherwise (_solo_role, _hard_role under pressure).
  4. Vs a human only (no Coach task): the ball model also spots foreign touches and then smooths and commits to the
     ball ("contested"), the role rules commit harder under pressure, the keeper slides along a wide mouth, and the
     robot keeps going through short tag dropouts (own tag or ball hidden under the human).
  5. Attack = dribble (_attack_dribble): meet the ball where it will be (fixed point of "ball after our ETA"), steer by
     pure pursuit bounded by lateral acceleration, line up behind it, hit it dead-centre and hard (closing 0.55 m/s)
     and keep driving through; a lined-up hard touch near the goal is a long shot (then a half turn away).
     Defend = keeper (_defend): shuffle in the middle of the mouth, shift toward a threatening ball; walk back, block
     or clear a slow ball in front of goal.
  6. Safety on every command: fences (targets kept wall_margin_m inside, speed toward a fence capped to what can
     brake, a ball on a fence is banked off it), any other tracked robot (no rams, with or without a Coach task),
     contact speed near the ball, the gait band, rate limits (shape).

Geometry: B = ball, u = unit push direction (ball -> aim point in the attack goal), n = u rotated +90 deg. For the robot
R, a = (R-B).u (negative = behind the ball), l = (R-B).n (offset from the shot line), phi = atan2(l, a) (+-pi = behind).

Settings that matter (DEFAULTS; params.json / a preset / `ctl set` change them; every other number is a fixed tuning
constant in TUNE, measured or found in the sim; notes/brain.md has the experiments):
  role_mode         auto (Coach / solo rules) | attack | defend | goto (per-call goto_x, goto_y, goto_yaw)
  goto_tol_m        goto: close enough to the target, m
  max_vx, max_vy, max_wz   speed caps, m/s and rad/s (the walker itself is capped by the preset's _stack speed_cap)
  accel_vx, accel_wz       acceleration limits, m/s^2 and rad/s^2
  decel_vx, decel_wz       braking limits toward zero (0 = same as accel)
  ball_decel        ball rolling deceleration, m/s^2 (measure with `ctl rollcal`)
  act_lag_s         command -> motion delay, s (pose prediction, fence braking)
  fast_yaw_bias_deg the fast gait walks this far off its tag heading, deg (step test; floor: -8 = to the right)
  wall_margin_m     robot centre keeps this far from every fence, m (in a goal mouth: up to 5 cm short of the line)
  wall_brake        deceleration the fence guard allows for, m/s^2 (lower = brakes earlier)
  aggression        Coach, 0..1: how close the ball player may come to the other robot (stop at 0.45 - 0.09a m,
                    walking pace 0.3a m/s allowed there) and how soon the shadow dashes for an opening
"""
from __future__ import annotations

import math

from .core import Cmd, Field, Pose, RobotCfg, World, wrap

DEFAULTS = {
    "role_mode": "auto",
    "max_vx": 0.30, "max_vy": 0.15, "max_wz": 1.0,
    "accel_vx": 0.6, "accel_wz": 2.0, "decel_vx": 0.0, "decel_wz": 0.0,
    "ball_decel": 0.149, "act_lag_s": 0.15, "fast_yaw_bias_deg": -8.0,
    "wall_margin_m": 0.28, "wall_brake": 0.8, "aggression": 0.75, "goto_tol_m": 0.08,
}

TUNE = {
    # timing and staleness (s)
    "pose_stale_s": 1.0, "ball_stale_s": 1.0, "other_stale_s": 1.5, "ball_lead_max_s": 0.4,
    "blind_max_m": 1.0,     # own tag hidden: stop once the dead-reckoned walk since the last fix is this long (m)
    "accel_vy": 0.5, "fast_yaw_vx": 0.5,            # fast_yaw_bias_deg applies above this commanded vx
    # steering gains (rad/s per rad, m/s per m) and the walker's gait switch: forward commands avoid (gait_lo, gait_hi)
    "k_turn": 1.0, "k_vy": 1.5, "gait_lo": 0.45, "gait_hi": 0.60, "gait_hold_s": 0.5,
    # near the ball: closing speed capped from max_vx at contact_slow_m (surface gap) down to contact_vx at touch
    "contact_vx": 0.3, "contact_slow_m": 0.5,
    # push controller (a defender clearing a ball): stage k_curve*|l| behind the ball, walk push_ahead through it
    "k_curve": 1.2, "behind_cone_deg": 120, "clear_margin": 0.12, "orbit_hold_s": 2.0, "orbit_slack": 0.10,
    "push_ahead": 0.4, "push_turn_scale": 0.5, "push_lat_tol": 0.12, "aim_y_frac": 0.0, "push_max_heading_deg": 40,
    # keeper: guard line goalie_dist in front of the goal line; clear a slow ball inside the clear zone
    "goalie_dist": 0.25, "goalie_margin": 0.05, "goalie_reach": 0.35, "k_goalie": 6.0, "goalie_vmax": 0.75,
    "return_vx": 0.75, "return_back_vx": 0.25,
    "clear_depth": 0.55, "clear_width_extra": 0.25, "clear_max_speed": 0.25,
    "gk_shuffle_m": 0.06, "gk_shuffle_hz": 0.7, "gk_edge_m": 0.15, "gk_far_m": 2.2, "gk_near_m": 1.0, "gk_tgoal_s": 2.5,
    # stuck: forward command but < stuck_dist progress for stuck_s -> reverse for unstick_s
    "stuck_s": 2.5, "stuck_dist": 0.06, "unstick_s": 1.0, "unstick_vx": 0.15,
    "avoid_radius": 0.55, "avoid_gain": 1.0, "field_margin": 0.12, "goto_yaw_tol_deg": 8,
    # solo roles (no Coach): defend a ball reaching our goal line within the horizon (or moving in our third)
    "solo_hold_s": 1.0, "solo_def_vmin": 0.15, "solo_def_horizon_s": 3.0, "solo_def_margin": 0.3, "solo_third": 0.333,
    "solo_reach_s": 0.4,
    # dribble: lateral acceleration (m/s^2), jerk (m/s^3), pursuit lookahead (m + s*speed), closing speed at touch
    "dribble_alat": 1.0, "dribble_jerk": 4.0, "dribble_look_m": 0.5, "dribble_look_s": 0.5, "dribble_close": 0.55,
    "dribble_margin": 0.65, "dribble_horizon_s": 3.0, "dribble_gait_gain": 0.88, "dribble_onset_s": 0.4,
    "dribble_cone_deg": 30, "dribble_near_m": 0.15, "dribble_line_m": 0.9, "dribble_line_lat": 0.3,
    "dribble_line_deg": 70, "dribble_line_look": 0.8, "dribble_db_deg": 7, "dribble_k_vy": 3.0, "dribble_k_head": 2.0,
    "dribble_commit_s": 0.3, "dribble_commit_wz": 0.8, "dribble_shot_m": 1.0, "dribble_shot_close": 1.0,
    "dribble_shot_deg": 15, "shot_follow_s": 1.8, "shot_through_s": 0.25, "shot_min_m": 0.4,
    # vs a human: contested ball, hard roles, wide-mouth keeper, occlusion
    "contest_dv": 0.3, "contest_hold_s": 1.0, "contest_self_m": 0.15, "contest_wall_m": 0.12, "contest_tau_p": 0.05,
    "contest_near_m": 0.4, "contest_tau_s": 0.4, "contest_horizon_s": 0.5, "contest_commit_s": 0.6,
    "contest_replan_m": 0.3, "contest_replan_frac": 0.3, "press_s": 8.0,
    "hr_zone": 0.5, "hr_vin": 0.1, "hr_hold_s": 2.5, "hr_exit_v": 0.3, "hr_exit_eta": 2.0, "hr_stick_m": 0.35,
    "guard_line_m": 0.2,
    "occl_pose_s": 1.5, "occl_slow_s": 0.3, "occl_speed": 0.6, "occl_gain": 0.9, "occl_ball_s": 2.0,
    # Coach (1v1)
    "poss_in_m": 0.12, "poss_out_m": 0.35, "poss_dwell_s": 0.3, "chal_hold_s": 1.0, "chal_cool_s": 1.5,
    "first_hyst_s": 0.4, "first_hold_s": 1.5, "jockey_m": 0.75, "jockey_sway_m": 0.10, "jockey_sway_s": 2.5,
    # any other robot: no rams (stop distance m, braking assumed m/s^2, a lost robot stays an obstacle this long s)
    "opp_stop_m": 0.45, "opp_brake": 0.8, "opp_keep_s": 4.0,
    # fences: end limit in a goal mouth, push-away speed inside the margin, banking a ball this close to a fence
    "wall_mouth_m": 0.05, "wall_rep_v": 0.2, "wall_ball_m": 0.32,
}


def merged(params: dict) -> dict:
    """The full parameter set the brain uses: fixed tuning, settings, then the caller's params. TUNE keys are not
    settings: the app refuses them (`ctl set` / presets: unknown parameter), so only `sim_eval --set` overrides them."""
    return {**TUNE, **DEFAULTS, **params}


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


def to_body(pose: Pose, wx: float, wy: float) -> tuple[float, float]:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return c * wx + s * wy, -s * wx + c * wy


def shape(cmd: Cmd, prev: Cmd, dt: float, params: dict) -> Cmd:
    """The one output shaper: clamp to max_vx/vy/wz, then rate-limit each axis from `prev` (accel_vx/vy/wz per
    second; vx and wz toward zero at decel_vx / decel_wz if set). An all-zero command is a stop and passes at once.
    The caller keeps the returned Cmd as the next `prev`."""
    if cmd.vx == 0.0 and cmd.vy == 0.0 and cmd.wz == 0.0:
        return cmd
    q = lambda k: params[k] if k in params else DEFAULTS[k] if k in DEFAULTS else TUNE[k]  # noqa: E731
    dt = clamp(dt, 0.0, 0.25)
    ax, ay, aw = q("accel_vx") * dt, q("accel_vy") * dt, q("accel_wz") * dt
    if q("decel_vx") > 0 and prev.vx * (cmd.vx - prev.vx) < 0:
        ax = q("decel_vx") * dt  # slowing down: the walker stops in ~0.4 s, so braking may be quicker
    if q("decel_wz") > 0 and prev.wz * (cmd.wz - prev.wz) < 0:
        aw = q("decel_wz") * dt
    vx = clamp(cmd.vx, -q("max_vx"), q("max_vx"))
    vy = clamp(cmd.vy, -q("max_vy"), q("max_vy"))
    wz = clamp(cmd.wz, -q("max_wz"), q("max_wz"))
    return Cmd(prev.vx + clamp(vx - prev.vx, -ax, ax), prev.vy + clamp(vy - prev.vy, -ay, ay),
               prev.wz + clamp(wz - prev.wz, -aw, aw), cmd.tag)


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

    def reset(self) -> None:
        self.role: str | None = None
        self.role_t = -1e9
        self.solo = True          # no Coach task this tick: play alone (vs a human)
        self.out = Cmd()
        self.last_t: float | None = None
        self.hist: list[tuple[float, float, float, float]] = []  # commands sent (t, vx, vy, wz), for _predict
        self.blind_m = 0.0        # distance _predict rolled the pose forward from the last fix (m)
        self.blind_t = -1e9       # capture time of the fix whose blind walk hit blind_max_m (stopped until a newer one)
        self.behind = False
        self.orbit_side = 0.0
        self.side_until = -1e9
        self.goto_done = False
        self.pushing = False
        self.returning = False
        self.clearing = False
        self.stuck_ref: tuple | None = None  # (t, x, y) where forward progress was last seen
        self.unstick_until = -1.0
        self.fast = False         # commanding the walker's fast gait (vx >= gait_hi)
        self.upshift_t = -1e9     # earliest time the next up-shift is allowed
        self.dr_a = 0.0           # dribble: commanded vx acceleration (jerk-limited)
        self.dr_bv: tuple | None = None  # dribble: filtered ball velocity
        self.dr_P: tuple | None = None   # dribble: filtered meeting point
        self.dr_t = -1e9          # dribble: time of the last update
        self.deflects = 0         # dribble: ball deflections seen since reset (sim_eval)
        self.dr_line = False      # dribble: line mode (heading on the push direction, lateral error by vy)
        self.dr_shot = False      # dribble: committed shot
        self.dr_sg, self.dr_flip_t = 0.0, None  # turn commitment: committed direction, start of a pending reversal
        self.dr_shot_t = -1e9     # dribble: time of the last long shot
        self.dr_follow: tuple | None = None  # long shot released: (since, turn sign, heading at release)
        self.bs: list | None = None  # contest: smoothed ball (x, y, vx, vy) at observation time bs_tobs
        self.bs_tobs = -1e9
        self.bs_hist: list = []      # contest: recent raw observations (t, x, y, vx, vy)
        self.bs_odd = 0              # contest: consecutive frames the rolling model could not explain
        self.bs_touch_t = -1e9       # contest: last foreign touch (brain time)
        self.bs_self_t = -1e9        # contest: last time our robot was at the ball ...
        self.bs_self_u = (1.0, 0.0)  # ... and the direction from it to the ball then
        self.touches = 0             # contest: foreign touches seen since reset (sim_eval)
        self.bc: tuple | None = None  # contest: committed target point, set at bc_t
        self.bc_t = -1e9
        self.b_state, self.b_press = "free", False  # this tick's ball state, opponent pressure
        self.hold_goal = False       # defend at the goal centre (ball not visible)
        self.b_raw: tuple | None = None  # contested: the raw (B, bv), for the goalie
        self.guard_yaw = 0.0         # wide-mouth goalie: the sideways heading chosen (+-pi/2)
        self.clear_sw_t = -1e9       # wide-mouth goalie: last clear <-> guard switch
        self.jk: tuple | None = None  # coach: low-passed shadow point J (x, y, t)
        self.ot: dict = {}           # other robots: id -> (t, x, y, vx, vy) last fix and low-passed velocity

    # ------------------------------------------------------------------ entry
    def decide(self, world: World, params: dict) -> tuple[Cmd, dict]:
        p = merged(params)
        t = world.t
        dt = 1.0 / p.get("loop_hz", 30) if self.last_t is None else clamp(t - self.last_t, 0.0, 0.25)
        self.last_t = t
        task = p.get("coach_task")
        self.solo = not task
        me = world.robots.get(self.cfg.id)
        dr_s = p["occl_pose_s"] if self.solo else 0.0  # own tag hidden: keep playing this long (vs a human)
        if me is None or t - me.t > max(p["pose_stale_s"], dr_s):
            self.out, self.pushing = Cmd.stop("pose_stale"), False
            self.hist.clear()
            dbg = {"role": "idle", "phase": "pose_stale", "target": None}
            if self.solo:
                dbg["occluded"] = "pose"
            return self.out, dbg
        # own tag hidden: under pressure (a human next to us) slow down from occl_slow_s; otherwise keep the
        # full-speed prediction up to pose_stale_s and slow only after that
        pose_hid = dr_s > 0 and t - me.t > p["occl_slow_s"] and p["role_mode"] != "goto"
        pose_occ = pose_hid and (self.b_press or t - me.t > p["pose_stale_s"])
        pose = self._predict(me.pose, me.t, t, p, p["occl_gain"] if pose_occ else 1.0)
        if self.blind_m > p["blind_max_m"] or me.t <= self.blind_t:  # walked blind far enough: stop until a new fix
            self.blind_t = max(self.blind_t, me.t)
            self.out, self.pushing = Cmd.stop("pose_stale"), False
            self.hist.clear()
            return self.out, {"role": "idle", "phase": "pose_stale", "target": None, "occluded": "pose", "blind": 1}
        others = [o.pose for k, o in world.robots.items()
                  if k != self.cfg.id and t - o.t < p["other_stale_s"]]
        if p["role_mode"] == "goto":
            self.role, self.pushing = "goto", False
            want, dbg = self._goto_test(pose, others, p)
            cmd = self._smooth(want, dt, p, t)
            dbg.update(role="goto", pose=[round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 3)])
            return cmd, dbg
        est = self._ball_est(world.ball, pose, t, p)
        occ = "+".join(k for k, v in (("pose", pose_hid), ("ball", self.b_state in ("occluded", "lost"))) if v)
        self.hold_goal = False
        if est is None and self.solo and p["role_mode"] in ("auto", "defend") and self.bs is not None:
            # ball unseen too long (under the human?): never stand still, guard the goal centre
            if self.role != "defend":
                self.role_t = t
            self.role, self.hold_goal = "defend", True
            want, dbg = self._defend(pose, (self.bs[0], self.bs[1]), (0.0, 0.0), others, p, t)
            want = self._occ_scale(want, pose_occ, p)
            cmd = self._smooth(want, dt, p, t)
            dbg.update(role="defend", phase="hold_goal", occluded=occ, ball_state="lost",
                       pose=[round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 3)])
            return cmd, dbg
        if est is None:
            self.pushing = False
            cmd = self._smooth(Cmd(0.0, 0.0, 0.0, "ball_lost"), dt, p, t)
            dbg = {"role": self.role or "idle", "phase": "ball_lost_hold", "target": None}
            if self.solo:
                dbg["occluded"] = occ
            return cmd, dbg
        B, bv = est
        if self.b_state == "contested":  # someone else is playing it: short predictions, no deflection snaps
            p = {**p, "dribble_horizon_s": min(p["dribble_horizon_s"], p["contest_horizon_s"]), "dribble_cone_deg": 180}
        role, info = self._role(pose, B, t, p, bv)
        self.hold_goal = role == "defend" and self.b_state == "occluded"
        if role == "attack":
            self.returning = self.clearing = False
            want, dbg = self._attack_dribble(pose, B, bv, others, p, t, dt)
        else:
            Bd, bvd = self.b_raw or (B, bv)
            want, dbg = self._defend(pose, Bd, bvd, others, p, t)
            if self.solo and not dbg["phase"].startswith("clear"):  # the dribble turn commitment, also
                want = Cmd(want.vx, want.vy, self._turn_commit(want.wz, False, t, p), want.tag)  # for the goalie
        want, dbg = self._unstick(pose, want, dbg, t, p)
        if dbg["phase"] != "return" and not dbg["phase"].startswith("dr_"):
            want = self._pace(pose, B, want, p)
        want, fenced = self._fence_guard(pose, want, p)
        if fenced:
            dbg["fence"] = 1
        near, vel = self._movers(world, t, p)
        if near:  # no rams, with or without a Coach task (the Coach gives {} on a stale tag or an unseen ball)
            want = self._opp_guard(pose, want, near, p, vel)
        want = self._occ_scale(want, pose_occ, p)
        cmd = self._smooth(want, dt, p, t)
        dbg.update(role=role, pose=[round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 3)],
                   ball=[round(B[0], 3), round(B[1], 3)], **info)
        if role == "defend" and task and self.jk:  # Play screen: the shadow line ball -> J
            dbg["shadow"] = [[round(B[0], 3), round(B[1], 3)], [round(self.jk[0], 3), round(self.jk[1], 3)]]
        if self.solo:
            dbg["occluded"] = occ
            dbg["ball_state"] = self.b_state
        return cmd, dbg

    def sent(self, cmd: Cmd) -> None:
        """Tell the brain what actually went to the robot when the caller overrode it (stale stop,
        teleop, shaping): the next ramp starts from there and pose prediction uses it."""
        self.out = cmd
        if self.hist:
            self.hist[-1] = (self.hist[-1][0], cmd.vx, cmd.vy, cmd.wz)

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
    def _predict(self, pose: Pose, t_obs: float, t: float, p: dict, gain: float = 1.0) -> Pose:
        """Roll the observed pose forward by commands executed after the camera saw it (translation x gain:
        dead reckoning through a long tag dropout uses the walker's measured gain). The fast gait walks
        fast_yaw_bias_deg off its tag heading: steer by the walking direction. Sets blind_m = the path walked."""
        lag = p["act_lag_s"]
        self.hist = [hc for hc in self.hist if hc[0] > t - 2.0]
        fb, fv = math.radians(p["fast_yaw_bias_deg"]), p["fast_yaw_vx"]

        def bias(vx):
            return fb if fb and vx > fv else 0.0

        self.blind_m = 0.0
        if not self.hist:
            return Pose(pose.x, pose.y, wrap(pose.yaw + bias(self.out.vx))) if fb else pose
        x, y, yaw = pose.x, pose.y, pose.yaw
        lo, hi = t_obs - lag, t - lag
        v_now = self.hist[0][1]
        for i, (ti, vx, vy, wz) in enumerate(self.hist):
            if ti <= hi:
                v_now = vx
            tj = self.hist[i + 1][0] if i + 1 < len(self.hist) else t
            seg = min(tj, hi) - max(ti, lo)
            if seg <= 0:
                continue
            ym = yaw + 0.5 * wz * seg + bias(vx)
            c, s = math.cos(ym), math.sin(ym)
            x += (c * vx - s * vy) * seg * gain
            y += (s * vx + c * vy) * seg * gain
            yaw += wz * seg
            self.blind_m += math.hypot(vx, vy) * seg * gain
        return Pose(x, y, wrap(yaw + bias(v_now)))

    # ------------------------------------------------------------------ ball: contested model, occlusion
    def _ball_est(self, ball, pose: Pose, t: float, p: dict):
        """(B, bv) for this tick, or None when the ball is lost; sets self.b_state (free / contested / occluded /
        lost) and self.b_press. With a Coach task this is the plain estimate: the last observation led linearly by
        its velocity for up to ball_lead_max_s.
        Vs a human (solo): the ball is contested when someone else touches it, i.e. over ~0.2 s its velocity changed
        by more than contest_dv (or it moved 0.25 m off the prediction) on two frames in a row, not a wall bounce and
        not explained by our own touch; also a ball back from > 0.3 s hidden far off the model. Contested lasts
        contest_hold_s after the last touch: alpha-beta smoothed ball, short horizons, committed target (_commit).
        "Pressure" = a foreign touch within press_s. Ball hidden: kept (frozen if contested, rolled on if free) up
        to occl_ball_s."""
        f = self.f
        self.b_raw = None
        if ball is None:
            self.b_state = "lost"
            return None
        age = t - ball.t
        soft = p["ball_lead_max_s"]
        h = clamp(age, 0.0, soft)
        plain = ((clamp(ball.x + ball.vx * h, -f.length / 2, f.length / 2),
                  clamp(ball.y + ball.vy * h, -f.width / 2, f.width / 2)), (ball.vx, ball.vy))
        if not self.solo:
            self.b_state = "free" if age <= p["ball_stale_s"] else "lost"
            return plain if age <= p["ball_stale_s"] else None
        if math.hypot(pose.x - ball.x, pose.y - ball.y) - f.ball_radius - f.robot_radius < p["contest_self_m"]:
            self.bs_self_t = t  # our own touches are not the opponent's: they push the ball away from us
            self.bs_self_u = unit(ball.x - pose.x, ball.y - pose.y)
        if self.bs is None or ball.t > self.bs_tobs + 1e-9:
            self._ball_obs(ball, t, p)
        # state as of the last observation (frozen while the ball is hidden)
        t_ref = min(t, ball.t + soft + 0.2)
        contested = t_ref - self.bs_touch_t < p["contest_hold_s"]
        self.b_press = t_ref - self.bs_touch_t < p["press_s"]
        x, y, vx, vy = self.bs
        if age <= soft:
            if not contested:
                self.b_state = "free"
                return plain
            self.b_state = "contested"
            self.b_raw = plain  # the goalie tracks the raw ball: lag costs saves, jitter on the line costs nothing
            return self._commit(self._ball_at((x, y), (vx, vy), h, p), pose, t, p), (vx, vy)
        if age > p["occl_ball_s"]:
            self.b_state = "lost"
            return None
        self.b_state = "occluded"
        if contested:  # hidden while someone plays it: keep the committed point, no velocity
            return (self.bc or (x, y)), (0.0, 0.0)
        sp = math.hypot(vx, vy)  # free: it keeps rolling as the model says
        ns = max(0.0, sp - p["ball_decel"] * age)
        return self._ball_at((x, y), (vx, vy), age, p), ((vx * ns / sp, vy * ns / sp) if sp > 1e-6 else (0.0, 0.0))

    def _ball_obs(self, ball, t: float, p: dict) -> None:
        """New ball observation: alpha-beta smoothing and the foreign-touch test (velocity change over ~0.2 s that
        the rolling model can't explain, two frames in a row, away from walls and our own touches)."""
        f = self.f
        m = (ball.t, ball.x, ball.y, ball.vx, ball.vy)
        if self.bs is None or ball.t - self.bs_tobs > 1.0:
            self.bs, self.bs_tobs, self.bs_hist, self.bs_odd = [ball.x, ball.y, ball.vx, ball.vy], ball.t, [m], 0
            return
        dt = max(1e-3, ball.t - self.bs_tobs)
        x, y, vx, vy = self.bs
        px, py = self._ball_at((x, y), (vx, vy), dt, p)
        sp = math.hypot(vx, vy)
        k = max(0.0, sp - p["ball_decel"] * dt) / sp if sp > 1e-6 else 0.0
        kp, kv = dt / (p["contest_tau_p"] + dt), dt / (p["contest_tau_s"] + dt)
        self.bs = [px + kp * (ball.x - px), py + kp * (ball.y - py),
                   k * vx + kv * (ball.vx - k * vx), k * vy + kv * (ball.vy - k * vy)]
        self.bs_tobs = ball.t
        self.bs_hist = [o for o in self.bs_hist if o[0] > ball.t - 0.5] + [m]
        if dt > 0.3:  # back after a gap (hidden): somewhere the rolling model can't explain -> it was played
            if (math.hypot(ball.x - px, ball.y - py) > 0.3 or math.hypot(ball.vx - k * vx, ball.vy - k * vy) > p["contest_dv"]) \
                    and min(f.length / 2 - abs(ball.x), f.width / 2 - abs(ball.y)) > f.ball_radius + p["contest_wall_m"]:
                self.touches += 1
                self.bs_touch_t, self.bs_odd = t, 2
            return
        old = None
        for o in self.bs_hist:
            if o[0] <= ball.t - 0.15:
                old = o
        if old is None:
            return
        to, ox, oy, ovx, ovy = old
        d = ball.t - to
        osp = math.hypot(ovx, ovy)
        ko = max(0.0, osp - p["ball_decel"] * d) / osp if osp > 1e-6 else 0.0
        res = math.hypot(ball.vx - ko * ovx, ball.vy - ko * ovy)
        ex, ey = self._ball_at((ox, oy), (ovx, ovy), d, p)
        pres = math.hypot(ball.x - ex, ball.y - ey)
        wm = f.ball_radius + p["contest_wall_m"]
        dvx, dvy = ball.vx - ko * ovx, ball.vy - ko * ovy
        wall = False  # a bounce: near a wall, and the change is along that wall's normal
        for o in self.bs_hist:
            if o[0] >= to:
                if f.length / 2 - abs(o[1]) < wm and abs(dvx) > 0.8 * res:
                    wall = True
                if f.width / 2 - abs(o[2]) < wm and abs(dvy) > 0.8 * res:
                    wall = True
        mine = t - self.bs_self_t <= 0.4 and dvx * self.bs_self_u[0] + dvy * self.bs_self_u[1] > 0.5 * res
        odd = (res > p["contest_dv"] or pres > 0.25) and not wall and not mine
        self.bs_odd = self.bs_odd + 1 if odd else 0
        if self.bs_odd >= 2:
            self.touches += self.bs_odd == 2
            self.bs_touch_t = t

    def _commit(self, Bs, pose: Pose, t: float, p: dict):
        """Contested target point: moves only on a large change (scaled by our distance), at most every
        contest_commit_s unless the change is twice the threshold."""
        gap = math.hypot(pose.x - Bs[0], pose.y - Bs[1]) - self.f.ball_radius - self.f.robot_radius
        thr = clamp(p["contest_replan_frac"] * gap, 0.05, p["contest_replan_m"])
        jump = math.inf if self.bc is None else math.hypot(Bs[0] - self.bc[0], Bs[1] - self.bc[1])
        if (gap < p["contest_near_m"] or jump > 2 * thr or (jump > thr and t - self.bc_t >= p["contest_commit_s"])
                or t - self.bc_t > 3.0):  # at the ball: play it live
            self.bc, self.bc_t = Bs, t
        return self.bc

    def _occ_scale(self, want: Cmd, pose_occ: bool, p: dict) -> Cmd:
        """Own tag missing: carry on with the dead-reckoned plan at occl_speed."""
        if not pose_occ:
            return want
        k = p["occl_speed"]
        return Cmd(want.vx * k, want.vy * k, want.wz * k, want.tag)

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

    # ------------------------------------------------------------------ roles
    def _reach_first(self, pose: Pose, B, bv, T: float, p: dict) -> bool:
        """Can we stand behind the ball on its way to our goal at least solo_reach_s (0 once we are already
        attacking) before it gets there, and >= 0.5 s before it reaches our goal line?"""
        if pose is None or T == math.inf:
            return False
        margin = 0.0 if self.role == "attack" else p["solo_reach_s"]
        tau = 0.1
        while tau <= T - 0.5:
            Q = self._ball_at(B, bv, tau, p)
            if self._dr_eta(pose, Q, self._dr_dir(Q, p), p) + margin <= tau:
                return True
            tau += 0.1
        return False

    def _solo_role(self, B, bv, p: dict, pose: Pose | None = None) -> str:
        """One robot vs a human: defend while the ball threatens our goal, attack otherwise (also attack a
        threatening ball we can reach first)."""
        s = 1.0 if self.own_x > 0 else -1.0
        depth = abs(self.own_x) - s * B[0]
        approach = s * bv[0]
        vmin = p["solo_def_vmin"] * (0.5 if self.role == "defend" else 1.0)
        in_third = depth < p["solo_third"] * self.f.length
        if approach > vmin:
            T, y = self._arrival(B, bv, self.own_x, p)
            on_target = y is not None and abs(y) < self.f.goal_width / 2 + p["solo_def_margin"]
            if on_target and (T <= p["solo_def_horizon_s"] or in_third):
                return "attack" if self._reach_first(pose, B, bv, T, p) else "defend"
        if in_third and approach > -vmin and math.hypot(bv[0], bv[1]) > vmin:
            T, _y = self._arrival(B, bv, self.own_x, p)
            return "attack" if self._reach_first(pose, B, bv, T, p) else "defend"  # moving (not away) in our third
        return "attack"

    def _hard_role(self, pose: Pose, B, bv, t: float, p: dict) -> tuple[str, str]:
        """Solo roles under opponent pressure, ball-only signals, with commitment (table in notes/brain.md):
        defend when a shot is predicted at our mouth, or the ball is in our zone (hr_zone x length from our goal
        line) and contested or moving at us (> hr_vin); stay defending >= hr_hold_s and until the ball is in the
        opponent half, or free, slow and reachable. A contested ball we are fighting for is not left.
        Returns (role, reason)."""
        f = self.f
        s = 1.0 if self.own_x > 0 else -1.0
        gl = abs(self.own_x)
        depth, rdepth = gl - s * B[0], gl - s * pose.x  # ball / robot distance from our goal line
        approach, sp = s * bv[0], math.hypot(bv[0], bv[1])
        contested = self.b_state == "contested" or (self.b_state == "occluded" and t - self.bs_touch_t < p["contest_hold_s"] + 1.0)
        T, y = self._arrival(B, bv, self.own_x, p)
        shot = (approach > p["solo_def_vmin"] and y is not None and abs(y) < f.goal_width / 2 + p["solo_def_margin"]
                and T <= p["solo_def_horizon_s"])
        if self.role == "defend":
            if t - self.role_t < p["hr_hold_s"]:
                return "defend", "def_hold"
            if depth > f.length / 2 + 0.1 and not shot:
                return "attack", "opp_half"
            if (not contested and sp < p["hr_exit_v"] and approach < 0.5 * p["solo_def_vmin"]
                    and self._dr_eta(pose, B, self._dr_dir(B, p), p) <= p["hr_exit_eta"]):
                return "attack", "free_reach"
            return "defend", "def_stay"
        if shot:
            if not contested and self._reach_first(pose, B, bv, T, p):
                return "attack", "meet_shot"
            return "defend", "shot"
        if depth < p["hr_zone"] * f.length and (contested or approach > p["hr_vin"]):
            gap = math.hypot(pose.x - B[0], pose.y - B[1]) - f.ball_radius - f.robot_radius
            if self.role == "attack" and gap < p["hr_stick_m"] and rdepth < depth + 0.1:
                return "attack", "stick"  # on the ball and goal-side of it: keep fighting for it
            if not contested and (self._reach_first(pose, B, bv, T, p) or (
                    rdepth < depth - 0.1 and self._dr_eta(pose, B, self._dr_dir(B, p), p) <= p["hr_exit_eta"])):
                return "attack", "reach"  # a free ball we get to first (or meet head-on from goal-side)
            if self.role == "attack" and t - self.role_t < p["solo_hold_s"]:
                return "attack", "att_hold"
            return "defend", "zone"
        return "attack", "attack"

    def _role(self, pose: Pose, B, t: float, p: dict, bv=(0.0, 0.0)):
        mode = p["role_mode"]
        if mode in ("attack", "defend"):
            self.role = mode
            return mode, {}
        task = p.get("coach_task")
        if task:  # the Coach decided for both robots: attack / challenge play the ball, the rest shadow
            want = "attack" if task in ("attack", "challenge") else "defend"
            if want != self.role:
                self.role, self.role_t = want, t
            return self.role, {"task": task, "intent": p.get("coach_why", ""), "adv": p.get("coach_adv"),
                               "possession": p.get("coach_poss")}
        if self.b_press:
            want, why = self._hard_role(pose, B, bv, t, p)
            if want != self.role:
                self.role, self.role_t = want, t
            return self.role, {"solo": 1, "rule": why}
        want = self._solo_role(B, bv, p, pose)
        if want != self.role and (self.role is None or t - self.role_t >= p["solo_hold_s"]):
            self.role, self.role_t = want, t
        return self.role, {"solo": 1}

    # ------------------------------------------------------------------ fences
    def _box(self, y: float, p: dict):
        """(x limit, y limit) for a robot centre at height y: the field shrunk by the walking margin; in a goal
        mouth the end limit is wall_mouth_m inside the goal line."""
        f = self.f
        m = max(p["field_margin"], p["wall_margin_m"])
        mouth = abs(y) < f.goal_width / 2 - 0.5 * f.robot_radius
        return f.length / 2 - (min(m, p["wall_mouth_m"]) if mouth else m), f.width / 2 - m

    def _out(self, x: float, y: float, p: dict) -> float:
        hl, hw = self._box(y, p)
        return max(0.0, abs(x) - hl) + max(0.0, abs(y) - hw)

    def _clamp_field(self, q, p: dict):
        hl, hw = self._box(q[1], p)
        y = clamp(q[1], -hw, hw)
        hl = self._box(y, p)[0]
        return (clamp(q[0], -hl, hl), y)

    def _fences(self, x: float, y: float, p: dict, post: bool = True):
        """[(distance, inward normal)] to each fence; in the mouth band the end 'fence' is virtual, placed so
        the margin ends wall_mouth_m inside the goal line. post=False leaves out the post walls."""
        f, wm = self.f, p["wall_margin_m"]
        hl, hw = f.length / 2, f.width / 2
        edge = f.goal_width / 2 - 0.5 * f.robot_radius
        out = self._post_walls(x, y) if post else []
        if abs(y) < edge:
            hl += max(0.0, wm - p["wall_mouth_m"])
        return out + [(hw - y, (0.0, -1.0)), (hw + y, (0.0, 1.0)), (hl - x, (-1.0, 0.0)), (hl + x, (1.0, 0.0))]

    def _post_walls(self, x: float, y: float):
        """Deep in a goal mouth (centre within robot_radius + 2 cm of the goal line) the end fence beside each post
        is a side wall: stop at the mouth band edge instead of sliding into it."""
        f = self.f
        edge = f.goal_width / 2 - 0.5 * f.robot_radius
        if abs(y) < edge and f.length / 2 - abs(x) < f.robot_radius + 0.02:
            return [(edge + f.robot_radius - abs(y), (0.0, -math.copysign(1.0, y)))]
        return []

    def _fence_guard(self, pose: Pose, want: Cmd, p: dict) -> tuple[Cmd, bool]:
        """Fence guard on the final command, every role: the world velocity toward a fence is capped at what can
        still brake (wall_brake, after act_lag_s of travel) before robot_radius, and within wall_margin_m a push
        away is added (0 at the margin, wall_rep_v at robot_radius)."""
        wm = p["wall_margin_m"]
        stop = self.f.robot_radius
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        vx, vy = c * want.vx - s * want.vy, s * want.vx + c * want.vy
        hit = False
        ox, oy = c * self.out.vx - s * self.out.vy, s * self.out.vx + c * self.out.vy  # what we are doing now
        post = self._post_walls(pose.x, pose.y)
        for i, (d, (nx, ny)) in enumerate(post + self._fences(pose.x, pose.y, p, post=False)):
            vin = -(vx * nx + vy * ny)  # commanded speed toward the fence
            d_eff = d - max(0.0, -(ox * nx + oy * ny)) * p["act_lag_s"]
            cap = math.sqrt(2 * p["wall_brake"] * max(0.0, d_eff - stop))
            if vin > cap:
                hit = True
                vx, vy = vx + (vin - cap) * nx, vy + (vin - cap) * ny
            if d < wm and i >= len(post):  # the post wall only caps: its push-away cancelled a robot's
                # first metres per second leaving the mouth past the post (1v1 deadlocks 6 -> 64 at 6c3f8ee)
                hit = True
                k = clamp((wm - d) / max(1e-3, wm - stop), 0.0, 1.0)  # 0 at the margin, 1 at the stop line
                vx, vy = vx + p["wall_rep_v"] * k * nx, vy + p["wall_rep_v"] * k * ny
        if not hit:
            return want, False
        bx, by = c * vx + s * vy, -s * vx + c * vy
        return Cmd(clamp(bx, -0.3, max(p["max_vx"], 0.3)), clamp(by, -p["max_vy"], p["max_vy"]), want.wz, want.tag), True

    def _wall_aim(self, B, aim, p: dict):
        """Push direction for a ball within wall_ball_m of a fence (None = not a wall ball): the robot cannot get
        behind it, so it banks: at the goal mirrored through a side fence (the ball comes off it toward the goal),
        into the corner for two fences (it comes out toward the centre). _feasible_dir then steepens it as needed."""
        wb = p["wall_ball_m"]
        f = self.f
        hl, hw = f.length / 2, f.width / 2
        near = []
        if hw - abs(B[1]) < wb:
            near.append("y")
        if hl - abs(B[0]) < wb and abs(B[1]) >= f.goal_width / 2:
            near.append("x")
        if not near:
            return None
        if len(near) == 2:
            return unit(math.copysign(1.0, B[0]), math.copysign(1.0, B[1]))
        if near[0] == "y":
            yw = math.copysign(hw, B[1])
            return unit(aim[0] - B[0], 2 * yw - aim[1] - B[1])
        return None  # end fence beside a goal: the goal is in that fence, aim straight (it slides along into the mouth)

    # ------------------------------------------------------------------ other robots
    def _opp_guard(self, pose: Pose, want: Cmd, others, p: dict, vel=None) -> Cmd:
        """Any other robot (Coach task or not): the closing speed is capped at what brakes at opp_brake (after
        act_lag_s) before opp_stop_m centre distance, and our share is that cap minus the other robot's own speed at
        us (vel, from _movers): no body rams; a ball between the two (0.54 m) still gets played. The ball player
        (Coach task attack / challenge, or role attack without a task) may come closer at a walking pace."""
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        vx, vy = c * want.vx - s * want.vy, s * want.vx + c * want.vy
        ox, oy = c * self.out.vx - s * self.out.vy, s * self.out.vx + c * self.out.vy
        hit, stop, vt = False, p["opp_stop_m"], 0.0
        task = p.get("coach_task")
        if task in ("attack", "challenge") or (not task and self.role == "attack"):  # may brush past at a walk
            a = clamp(p["aggression"], 0.0, 1.0)
            stop, vt = stop - 0.09 * a, 0.3 * a
        for i, o in enumerate(others):
            d = math.hypot(pose.x - o.x, pose.y - o.y)
            nx, ny = unit(pose.x - o.x, pose.y - o.y)  # away from the other robot
            vin = -(vx * nx + vy * ny)
            va = max(0.0, vel[i][0] * nx + vel[i][1] * ny) if vel else 0.0  # the other robot coming at us
            room = d - (max(0.0, -(ox * nx + oy * ny)) + va) * p["act_lag_s"] - stop
            cap = max(0.0, math.sqrt(vt * vt + 2 * p["opp_brake"] * max(0.0, room)) - va)
            if vin > cap:
                hit, vx, vy = True, vx + (vin - cap) * nx, vy + (vin - cap) * ny
        if not hit:
            return want
        return Cmd(c * vx + s * vy, clamp(-s * vx + c * vy, -p["max_vy"], p["max_vy"]), want.wz, want.tag)

    def _movers(self, world: World, t: float, p: dict):
        """The other robots now: ([Pose], [(vx, vy)]), each last fix rolled forward (<= 0.5 s) by its low-passed
        velocity (time constant 0.2 s), so the guard sees closing speed, not only our own. A robot seen within
        opp_keep_s counts: a lost robot stops itself within blind_max_m, so its last fix stays a fair obstacle
        (sim 1v1 at max_vx 1.4: bumps over 0.5 m/s 20 -> 12 per 72 matches)."""
        poses, vels = [], []
        for k, o in world.robots.items():
            if k == self.cfg.id or t - o.t >= p["opp_keep_s"]:
                continue
            last = self.ot.get(k)
            if last is None or o.t > last[0] + 1e-6:
                vx = vy = 0.0
                if last is not None and o.t - last[0] < 0.5:
                    h = o.t - last[0]
                    a = min(1.0, h / 0.2)
                    vx = last[3] + a * ((o.pose.x - last[1]) / h - last[3])
                    vy = last[4] + a * ((o.pose.y - last[2]) / h - last[4])
                last = self.ot[k] = (o.t, o.pose.x, o.pose.y, vx, vy)
            h = min(0.5, t - o.t)
            poses.append(Pose(o.pose.x + last[3] * h, o.pose.y + last[4] * h, o.pose.yaw))
            vels.append((last[3], last[4]))
        return poses, vels

    def _shadow(self, B, p: dict):
        """Coach shadow point J: jockey_m goal-side of the ball on the line ball -> own goal centre, never behind
        the guard line (clamped to it = the keeper). Returns (J, on_line)."""
        s = 1.0 if self.own_x > 0 else -1.0
        x_line = self.own_x - s * p["goalie_dist"]
        jm = p["jockey_m"]
        ux, uy = unit(self.own_x - B[0], -B[1])
        J = (B[0] + ux * jm, B[1] + uy * jm)
        if s * J[0] >= s * x_line:
            return (x_line, J[1]), True
        return self._clamp_field(J, p), False

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

    # ------------------------------------------------------------------ motion primitives
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

    def _frame(self, pose: Pose, B, p: dict):
        f = self.f
        hw = f.goal_width / 2 - f.ball_radius
        aim = (self.att_x, clamp(B[1] * p["aim_y_frac"], -hw, hw))
        ux, uy = self._feasible_dir(B, self._wall_aim(B, aim, p) or unit(aim[0] - B[0], aim[1] - B[1]), p)
        nx, ny = -uy, ux
        rx, ry = pose.x - B[0], pose.y - B[1]
        a, l = rx * ux + ry * uy, rx * nx + ry * ny
        return (ux, uy), (nx, ny), a, l, math.atan2(l, a)

    def _feasible_dir(self, B, u0, p: dict):
        """Smallest rotation of the push direction whose stage point behind the ball is reachable.
        A ball on a wall gets pushed along it (slightly into it) instead of an unreachable shot line."""
        f = self.f
        d = f.ball_radius + f.robot_radius + 0.03
        best, best_out = u0, 1e9
        for k in range(0, 73):
            rot = math.radians(5 * ((k + 1) // 2) * (1 if k % 2 else -1))
            c, s = math.cos(rot), math.sin(rot)
            u = (c * u0[0] - s * u0[1], s * u0[0] + c * u0[1])
            sx, sy = B[0] - u[0] * d, B[1] - u[1] * d
            out = self._out(sx, sy, p)
            if out <= 0.0:
                return u
            if out < best_out - 1e-9:
                best, best_out = u, out
        return best

    def _route(self, pose: Pose, B, u, n, phi: float, clear: float, p: dict, t: float):
        """Next waypoint around the ball toward the stage behind it. The way round is kept for
        orbit_hold_s; a waypoint slightly outside the field margin is clamped in, not rejected
        (rejecting it used to flip the side every tick next to a wall)."""
        rr = clear * 1.15
        S = self._clamp_field((B[0] - u[0] * rr, B[1] - u[1] * rr), p)
        R = (pose.x, pose.y)
        if math.hypot(R[0] - B[0], R[1] - B[1]) > clear and seg_dist(R, S, B) >= clear:
            return S
        step = math.radians(70)

        def wp(s):
            pw = phi + s * step
            pw = clamp(pw, -math.pi, math.pi) if s * phi >= 0 else pw
            return (B[0] + rr * (math.cos(pw) * u[0] + math.sin(pw) * n[0]),
                    B[1] + rr * (math.cos(pw) * u[1] + math.sin(pw) * n[1]))

        def inside(W, slack):
            bl, bw = self._box(W[1], p)
            return abs(W[0]) <= bl + slack and abs(W[1]) <= bw + slack

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

    # ------------------------------------------------------------------ push (a defender clearing the ball)
    def _clear_push(self, pose: Pose, B, others, p: dict, t: float):
        """Walk round to the stage behind the ball (k_curve*|l| back), then on the shot line walk push_ahead
        through it; on the line next to the ball but facing > push_max_heading_deg off: square up first."""
        f = self.f
        c = f.ball_radius + f.robot_radius
        u, n, a, l, phi = self._frame(pose, B, p)
        clear = c + p["clear_margin"]
        cone = math.radians(p["behind_cone_deg"])
        self.behind = abs(phi) > (cone - math.radians(15) if self.behind else cone)
        if not self.behind:
            self.pushing = False
            tgt = self._route(pose, B, u, n, phi, clear, p, t)
            return self._goto(pose, tgt, p, p["max_vx"], others, tag="route"), \
                {"phase": "route", "target": [round(tgt[0], 3), round(tgt[1], 3)], "lat": round(l, 3)}
        herr = wrap(math.atan2(u[1], u[0]) - pose.yaw)
        tol = p["push_lat_tol"]
        on_line = abs(l) < (2 * tol if self.pushing else tol)
        dB = math.hypot(pose.x - B[0], pose.y - B[1])
        close = dB < c + 0.15
        lim = p["push_max_heading_deg"]
        if on_line and close and abs(herr) > math.radians(lim + (15 if self.pushing else 0)):
            # pushes that began 50-130 deg off the shot line knocked the ball sideways: square up first,
            # stepping back while the body is within clear_margin of the ball so turning can't clip it
            self.pushing = False
            bx, _ = to_body(pose, B[0] - pose.x, B[1] - pose.y)
            vx = -p["unstick_vx"] if dB < clear and bx > 0 else 0.0
            return Cmd(vx, clamp(-p["k_vy"] * l, -p["max_vy"], p["max_vy"]), p["k_turn"] * herr, "square"), \
                {"phase": "square", "target": [round(B[0], 3), round(B[1], 3)], "lat": round(l, 3)}
        self.pushing = on_line
        # far off the line -> stage behind the ball (k_curve*|l|); on the line -> aim push_ahead beyond the ball
        w = min(1.0, abs(l) / (2 * tol))
        back = p["k_curve"] * abs(l) * w - p["push_ahead"] * (1.0 - w)
        tgt = (B[0] - u[0] * back, B[1] - u[1] * back)
        if back > 0:
            tgt = self._clamp_field(tgt, p)
        phase = "push" if self.pushing and close else "approach"
        cmd = self._goto(pose, tgt, p, p["max_vx"], others, avoid=0.3 if phase == "push" else 1.0, lateral=False, tag=phase)
        if self.pushing:  # slide onto the shot line with the weak lateral axis
            cmd = Cmd(cmd.vx, clamp(-p["k_vy"] * l, -p["max_vy"], p["max_vy"]) * (1 - w),
                      cmd.wz * p["push_turn_scale"], phase)
        return cmd, {"phase": phase, "target": [round(tgt[0], 3), round(tgt[1], 3)], "lat": round(l, 3)}

    # ------------------------------------------------------------------ attack = dribble
    def _dr_eta(self, pose: Pose, Q, u, p: dict) -> float:
        """Seconds until we stand at the contact point behind Q (push direction u): start-up (less if already
        moving that way) + distance at the measured gait speed (+ the walk round the ball) + half the turn."""
        c = self.f.ball_radius + self.f.robot_radius
        S = (Q[0] - u[0] * c, Q[1] - u[1] * c)
        d = math.hypot(S[0] - pose.x, S[1] - pose.y)
        a = (pose.x - Q[0]) * u[0] + (pose.y - Q[1]) * u[1]
        d += 1.5 * max(0.0, a + 0.5 * c)  # in front of the ball: walk round it
        v = max(0.1, p["dribble_gait_gain"] * p["max_vx"])
        head = math.atan2(S[1] - pose.y, S[0] - pose.x) if d > 0.15 else math.atan2(u[1], u[0])
        turn = abs(wrap(head - pose.yaw))
        v0 = max(0.0, self.out.vx) * max(0.0, math.cos(turn))
        return p["dribble_onset_s"] * (1.0 - min(1.0, v0 / v)) + d / v + 0.5 * turn / max(0.1, p["max_wz"])

    def _dr_dir(self, P, p: dict):
        f = self.f
        hw = f.goal_width / 2 - f.ball_radius
        ay = p.get("coach_aim_y")  # coach: the side of the mouth away from the opponent
        aim = (self.att_x * (1 + f.goal_depth / max(1e-3, abs(self.att_x))),
               clamp(P[1] * p["aim_y_frac"] if ay is None else ay, -hw, hw))
        return self._feasible_dir(P, self._wall_aim(P, aim, p) or unit(aim[0] - P[0], aim[1] - P[1]), p)

    def _attack_dribble(self, pose: Pose, B, bv, others, p: dict, t: float, dt: float):
        """Meet the ball where it will be, then keep touching it toward the goal (see the module doc):
        - meeting point: fixed point of "ball after our ETA" (dribble_horizon_s cap); at the ball (dribble_near_m)
          the ball itself. A velocity change > dribble_cone_deg or 0.3 m/s is a deflection: filters snap.
        - steering: pure pursuit, lookahead dribble_look_m + dribble_look_s * speed, wz = v * kappa with v cut to
          sqrt(dribble_alat / |kappa|). In front of the ball: orbit round it (_route).
        - line mode (behind the ball within dribble_line_m, |lateral| < dribble_line_lat, heading within
          dribble_line_deg): heading on the push direction with a dribble_db_deg deadband, lateral error by vy.
        - speed: closing speed dribble_close at the touch (hit hard and straight), dribble_margin one step out,
          braking-limited beyond; jerk-limited; off centre at the ball it keeps pace without closing in.
        - shot: lined up and centred within dribble_shot_m of the goal -> closing dribble_shot_close. Long shot: a
          hard aligned touch at fast-gait speed that goes in is released, then a half turn (shot_follow_s)."""
        f = self.f
        c = f.ball_radius + f.robot_radius
        clear = c + p["clear_margin"]
        fresh = t - self.dr_t > 0.5
        self.dr_t = t
        if self.dr_follow is not None:  # long shot hit: drive through it, then a half turn, then play on
            t0, sg, y0 = self.dr_follow
            if fresh or t >= t0 + p["shot_follow_s"] or abs(wrap(pose.yaw - y0)) > 0.9 * math.pi:
                self.dr_follow = None
            else:
                self.pushing = False
                v = min(p["max_vx"], p["gait_hi"])
                wz = sg * p["max_wz"] if t >= t0 + p["shot_through_s"] else 0.0
                return Cmd(v, 0.0, wz, "dr_follow"), {"phase": "dr_follow", "target": None}
        # ball velocity: light low-pass; a big change of direction/speed is a deflection -> snap and re-solve
        sp = math.hypot(bv[0], bv[1])
        deflect = False
        if fresh or self.dr_bv is None:
            self.dr_bv = bv
        else:
            fv = self.dr_bv
            fs = math.hypot(fv[0], fv[1])
            ang = abs(wrap(math.atan2(bv[1], bv[0]) - math.atan2(fv[1], fv[0]))) if min(sp, fs) > 0.15 else 0.0
            if ang > math.radians(p["dribble_cone_deg"]) or abs(sp - fs) > 0.3:
                deflect = True
                self.deflects += 1
                self.dr_bv = bv
            else:
                k = min(1.0, dt / 0.1)
                self.dr_bv = (fv[0] + k * (bv[0] - fv[0]), fv[1] + k * (bv[1] - fv[1]))
        bvf = self.dr_bv
        # meeting point: fixed point of "ball after our ETA", capped at the horizon
        P, u = B, self._dr_dir(B, p)
        if math.hypot(bvf[0], bvf[1]) > 0.05:
            for _ in range(4):
                P = self._ball_at(B, bvf, min(p["dribble_horizon_s"], self._dr_eta(pose, P, u, p)), p)
                u = self._dr_dir(P, p)
            # at the ball: play the ball itself (a far meeting point swings the line under a carried ball)
            w = clamp((math.hypot(pose.x - B[0], pose.y - B[1]) - c) / p["dribble_near_m"], 0.0, 1.0)
            P = (B[0] + w * (P[0] - B[0]), B[1] + w * (P[1] - B[1]))
        if fresh or deflect or self.dr_P is None or math.hypot(P[0] - self.dr_P[0], P[1] - self.dr_P[1]) > 0.25:
            self.dr_P = P
        else:  # 3 s of prediction multiplies velocity noise: smooth the point (tau 0.2 s)
            k = min(1.0, dt / 0.2)
            self.dr_P = (self.dr_P[0] + k * (P[0] - self.dr_P[0]), self.dr_P[1] + k * (P[1] - self.dr_P[1]))
        P = self.dr_P
        u = self._dr_dir(P, p)
        n = (-u[1], u[0])
        rx, ry = pose.x - P[0], pose.y - P[1]
        a, le = rx * u[0] + ry * u[1], rx * n[0] + ry * n[1]
        phi = math.atan2(le, a)
        cone = math.radians(p["behind_cone_deg"])
        self.behind = abs(phi) > (cone - math.radians(15) if self.behind else cone)
        v_now = max(0.0, self.out.vx)
        L = p["dribble_look_m"] + p["dribble_look_s"] * v_now
        gap = math.hypot(pose.x - B[0], pose.y - B[1]) - c
        if not self.behind:
            tgt = self._route(pose, P, u, n, phi, clear, p, t)
            phase = "dr_route"
            ex, ey = unit(tgt[0] - pose.x, tgt[1] - pose.y)
            back = c + 1.2 * abs(le)  # a far-off-line robot stages further back: it joins the line at a shallow angle
            Sf = self._clamp_field((P[0] - u[0] * back, P[1] - u[1] * back), p)
            if seg_dist((pose.x, pose.y), Sf, P) >= clear:
                tgt, (ex, ey) = Sf, u  # ... and the carrot runs on along the line, not past the stage
            d = math.hypot(tgt[0] - pose.x, tgt[1] - pose.y)
            carrot = tgt
            if d < L:  # never chase a point inside the turning circle: extend it
                carrot = (tgt[0] + ex * (L - d), tgt[1] + ey * (L - d))
        else:
            # pure pursuit on the ball line; far off the line the carrot stays behind the ball so the robot joins
            # the line before it reaches the ball; the carrot is always >= L/2 ahead of us
            along = max(a + 0.5 * L, min(a + L, -(c + 1.2 * max(0.0, abs(le) - 0.03))))
            carrot = (P[0] + u[0] * along, P[1] + u[1] * along)
            phase = "dr_carry" if gap < 0.15 and abs(le) < 0.08 else "dr_chase"
        carrot = self._clamp_field(carrot, p)
        rpx, rpy = self._repel(pose, others, p)
        bx, by = to_body(pose, carrot[0] - pose.x + L * rpx, carrot[1] - pose.y + L * rpy)
        Ld = max(0.05, math.hypot(bx, by))
        alpha = math.atan2(by, bx)
        kap = 2.0 * math.sin(alpha) / Ld
        alat = p["dribble_alat"]
        vmax = p["max_vx"]
        v_des = min(vmax, math.sqrt(alat / max(1e-6, abs(kap))))
        # closing speed toward the ball: braking-limited far out, dribble_margin one step out, dribble_close at touch
        dxb, dyb = B[0] - pose.x, B[1] - pose.y
        bear = math.cos(wrap(math.atan2(dyb, dxb) - pose.yaw))
        if bear > 0.5:
            vb = bvf[0] * math.cos(pose.yaw) + bvf[1] * math.sin(pose.yaw)
            g = max(0.0, gap)
            rel = p["dribble_close"] + (p["dribble_margin"] - p["dribble_close"]) * min(1.0, g / 0.15)
            rel += math.sqrt(2 * 0.5 * max(0.1, p["decel_vx"] or p["accel_vx"]) * max(0.0, g - 0.15))
            if gap < 0.25 and abs(le) > 0.06:  # off-centre at the ball: keep pace, but don't close in until lined up
                rel = min(rel, max(0.15, rel * clamp(1.0 - (abs(le) - 0.06) / 0.08, 0.0, 1.0)))  # (never a stall)
            v_des = min(v_des, max(0.0, vb) + rel)
        # line mode: close behind the ball and roughly lined up -> the heading tracks the push direction (plus a
        # gentle pull toward the line), a deadband ignores small errors and vy slides off the lateral error
        herr = wrap(math.atan2(u[1], u[0]) - pose.yaw)
        if self.behind and a < 0:
            k_in = 0.0 if not self.dr_line else 1.0
            self.dr_line = (gap < p["dribble_line_m"] + 0.2 * k_in and abs(le) < p["dribble_line_lat"] * (1 + 0.5 * k_in)
                            and abs(herr) < math.radians(p["dribble_line_deg"]) * (1 + 0.4 * k_in))
        else:
            self.dr_line = False
        # shot: in range of the goal, lined up and centred -> commit: full speed through the ball
        sm = p["dribble_shot_m"]
        if self.dr_line and gap < 0.3:
            gx = self.att_x
            ray = (gx - B[0]) / math.cos(pose.yaw) if math.cos(pose.yaw) * (gx - B[0]) > 0 else math.inf
            y_hit = B[1] + math.sin(pose.yaw) * ray if ray < math.inf else math.inf
            mouth = f.goal_width / 2 - f.ball_radius
            k_in = 1.0 if self.dr_shot else 0.0
            self.dr_shot = (abs(gx - B[0]) < sm * (1 + 0.3 * k_in) and abs(y_hit) < mouth * (1 + 0.3 * k_in)
                            and abs(herr) < math.radians(p["dribble_shot_deg"]) * (1 + k_in)
                            and abs(le) < 0.04 * (1 + 0.5 * k_in))  # dead centre: an off-centre hard hit flies wide
        elif not (self.dr_shot and gap < 0.4 and self.dr_line):
            self.dr_shot = False
        if (self.dr_line and gap < 0.03 and v_now >= p["gait_hi"]
                and p["shot_min_m"] < abs(self.att_x - B[0]) < sm and t - self.dr_shot_t > 3.0
                and self._goes_in(pose, B)
                and all(seg_dist(B, (self.att_x, B[1] + math.tan(pose.yaw) * (self.att_x - B[0])), (o.x, o.y))
                        > f.robot_radius + f.ball_radius + 0.1 for o in others)):  # (lane open: not into a keeper)
            # long shot: one hard touch at speed that goes in -> let the ball go (no dribbling it into the goal)
            lx, ly = -math.sin(pose.yaw), math.cos(pose.yaw)
            sg = 1.0 if lx * -pose.x + ly * -pose.y >= 0 else -1.0  # the half turn goes toward the field centre
            self.dr_follow, self.dr_shot, self.pushing, self.dr_shot_t = (t, sg, pose.yaw), False, False, t
            return Cmd(v_now, 0.0, 0.0, "dr_follow"), {"phase": "dr_follow", "target": None, "shot": 1}
        if self.dr_line:
            phase = "dr_shot" if self.dr_shot else phase
            v_des = vmax
            if bear > 0.5:
                vb = bvf[0] * math.cos(pose.yaw) + bvf[1] * math.sin(pose.yaw)
                g = max(0.0, gap)
                close = p["dribble_shot_close"] if self.dr_shot else p["dribble_close"]
                rel = close + (max(close, p["dribble_margin"]) - close) * min(1.0, g / 0.15)
                rel += math.sqrt(2 * 0.5 * max(0.1, p["decel_vx"] or p["accel_vx"]) * max(0.0, g - 0.15))
                if gap < 0.25 and abs(le) > 0.06 and not self.dr_shot:  # off-centre at the ball: keep pace, slide
                    wall = min(f.length / 2 - abs(B[0]), f.width / 2 - abs(B[1])) < c + 0.05  # (a wall ball may
                    rel = max(0.15 if wall else 0.0, rel * clamp(1.0 - (abs(le) - 0.06) / 0.08, 0.0, 1.0))  # not let us)
                v_des = min(v_des, max(0.0, vb) + rel)
        spin = abs(alpha) > math.radians(75) and not self.dr_line
        if spin:
            v_des = 0.0
        # jerk-limited speed (time constant 0.25 s toward v_des)
        dec = p["decel_vx"] or p["accel_vx"]
        a_t = clamp((v_des - v_now) / 0.25, -dec, p["accel_vx"])
        j = p["dribble_jerk"] * dt
        self.dr_a = clamp(self.dr_a + clamp(a_t - self.dr_a, -j, j), -dec, p["accel_vx"])
        v = clamp(v_now + self.dr_a * dt, 0.0, vmax)
        if (v - v_des) * self.dr_a > 0 and abs(v - v_des) < abs(self.dr_a) * dt:
            v = v_des
        lo, hi = p["gait_lo"], p["gait_hi"]
        if lo < v < hi:  # the walker's gait switch: step across the band, don't stall in it
            v = hi if v_des >= hi else lo if v_des <= lo else v
        self.dr_a = (v - v_now) / max(1e-3, dt)  # back-calculated: no wind-up at a clamp
        if spin:
            wz = clamp(1.5 * alpha, -p["max_wz"], p["max_wz"])
        elif self.dr_line:
            ah = wrap(herr - math.atan2(le, p["dribble_line_look"]))
            ah = math.copysign(max(0.0, abs(ah) - math.radians(p["dribble_db_deg"])), ah)
            wz = clamp(p["dribble_k_head"] * ah, -alat / max(v, 0.2), alat / max(v, 0.2))
        else:  # curvature times speed, bounded by the lateral-acceleration limit
            wz = kap * max(v, 0.2)
            wz = clamp(wz, -alat / max(v, 0.2), alat / max(v, 0.2))
        wz = self._turn_commit(wz, spin, t, p)
        vy = 0.0
        if self.dr_line:  # world lateral correction along -n, expressed in the body frame
            vy = clamp(-p["dribble_k_vy"] * le * (-math.sin(pose.yaw) * n[0] + math.cos(pose.yaw) * n[1]),
                       -p["max_vy"], p["max_vy"])
        elif self.behind and gap < 0.3 and abs(le) < 0.2:  # slide the last centimetres with the lateral axis
            vy = clamp(-p["k_vy"] * le, -p["max_vy"], p["max_vy"])
        self.pushing = phase in ("dr_carry", "dr_shot")
        dbg = {"phase": phase, "target": [round(carrot[0], 3), round(carrot[1], 3)], "lat": round(le, 3),
               "intercept": [round(P[0], 3), round(P[1], 3)], "kappa": round(kap, 2), "v_des": round(v_des, 2)}
        if deflect:
            dbg["deflect"] = 1
        return Cmd(v, vy, wz, phase), dbg

    def _goes_in(self, pose: Pose, B) -> bool:
        """A hit now sends the ball into the attacked mouth: the ray from the ball along the contact line (robot
        centre -> ball) and along our heading both cross the goal line inside it (less a ball radius)."""
        f = self.f
        gx, hw = self.att_x, f.goal_width / 2 - 2 * f.ball_radius
        for ux, uy in (unit(B[0] - pose.x, B[1] - pose.y), (math.cos(pose.yaw), math.sin(pose.yaw))):
            if ux * (gx - B[0]) <= 0:
                return False
            if abs(B[1] + uy * (gx - B[0]) / ux) > hw:
                return False
        return True

    def _turn_commit(self, wz: float, spin: bool, t: float, p: dict) -> float:
        """Minimum turn commitment: a turn the other way waits dribble_commit_s unless it is big
        (dribble_commit_wz) or a turn on the spot."""
        sg = 1.0 if wz > 0 else -1.0
        if abs(wz) > 0.2 and self.dr_sg and sg != self.dr_sg and not spin:
            if self.dr_flip_t is None:
                self.dr_flip_t = t
            if t - self.dr_flip_t < p["dribble_commit_s"] and abs(wz) < p["dribble_commit_wz"]:
                return sg * 0.2
            self.dr_sg, self.dr_flip_t = sg, None
        else:
            self.dr_flip_t = None
            if abs(wz) > 0.2:
                self.dr_sg = sg
        return wz

    # ------------------------------------------------------------------ defend
    def _defend(self, pose: Pose, B, bv, others, p: dict, t: float):
        """Keeper on the guard line (goalie_dist in front of the goal line), body sideways (forward/back walking
        slides along it): a shuffle in the middle of the mouth shifted toward the ball as it threatens (_gk_target).
        Away from the line: walk back (round a slow ball, _detour), or block the ball's path when the line can't be
        reached in time (_block). A slow ball inside the clear zone is cleared (push controller). Coach shadow
        tasks (jockey / goalie / recover) hold the shadow point J instead, facing the ball.
        Vs a human the mouth is wide (guard_line_m): on the line it slides instead of turning to walk."""
        f = self.f
        s = 1.0 if self.own_x > 0 else -1.0
        gl = abs(self.own_x)
        depth = gl - s * B[0]  # ball distance in front of own goal line
        hyst = 0.12 if self.clearing else 0.0
        approach = s * bv[0]
        # under pressure a contested ball is cleared only from goal-side: walking round it opens the goal
        side_ok = not (self.solo and self.b_press and self.b_state == "contested") or \
            gl - s * pose.x < depth - (0.0 if self.clearing else 0.05)
        coach = p.get("coach_task") in ("jockey", "goalie", "recover")
        gm = p["guard_line_m"] if self.solo else 0.0
        clear = (not coach and not self.hold_goal and side_ok and depth < p["clear_depth"] + hyst
                 and abs(B[1]) < f.goal_width / 2 + p["clear_width_extra"] + hyst and approach < p["clear_max_speed"])
        if gm > 0 and not self.hold_goal and clear != self.clearing:
            if t - self.clear_sw_t < 0.8:  # wide-mouth goalie: clear <-> guard flips want opposite headings
                clear = self.clearing
            else:
                self.clear_sw_t = t
        if clear:
            self.clearing, self.returning = True, False
            cmd, dbg = self._clear_push(pose, B, others, p, t)
            dbg["phase"] = "clear_" + dbg["phase"]
            return cmd, dbg
        self.clearing, self.pushing = False, False
        x_line = self.own_x - s * p["goalie_dist"]
        lim = f.goal_width / 2 + p["goalie_margin"]
        G, ff = self._gk_target(B, bv, x_line, approach, t, p)  # ff: feed-forward world vy of the shuffle
        jockey = False
        if coach:
            J, on_line = self._shadow(B, p)
            if self.jk is None or t - self.jk[2] > 0.5 or math.hypot(J[0] - self.jk[0], J[1] - self.jk[1]) > 0.5:
                self.jk = (J[0], J[1], t)
            else:  # low-pass ~0.3 s: ball jitter is not mirrored
                k = min(1.0, (t - self.jk[2]) / 0.3)
                self.jk = (self.jk[0] + k * (J[0] - self.jk[0]), self.jk[1] + k * (J[1] - self.jk[1]), t)
            if not on_line:  # jockey: hold J facing the ball, swaying across the ball-goal line (looks alive)
                jockey = True
                ux, uy = unit(self.own_x - B[0], -B[1])
                sw = p["jockey_sway_m"] * math.sin(2 * math.pi * t / max(0.5, p["jockey_sway_s"]))
                G = self._clamp_field((self.jk[0] - uy * sw, self.jk[1] + ux * sw), p)
                x_line = G[0]
        dx, dy = G[0] - pose.x, G[1] - pose.y
        dist = math.hypot(dx, dy)
        self.returning = dist > (0.12 if self.returning else p["goalie_reach"])
        if gm > 0 and abs(pose.x - x_line) < (gm if self.returning else 1.5 * gm) and abs(pose.y) < lim + 0.1:
            self.returning = False  # on the line already: slide along it (guard), don't turn and walk
        tgt = [round(G[0], 3), round(G[1], 3)]
        if self.returning and gm > 0:  # from afar aim at the mouth centre first: a target sliding along a wide mouth
            w = clamp(1.0 - (dist - 0.4) / 0.8, 0.0, 1.0)  # swings the heading (turn reversals)
            G = (G[0], G[1] * w)
            dx, dy = G[0] - pose.x, G[1] - pose.y
            tgt = [round(G[0], 3), round(G[1], 3)]
        vret = min(p["max_vx"], p["return_vx"])
        if self.returning:
            dbg = {"phase": "return", "target": tgt}
            K = self._block(pose, B, bv, p) if approach > p["solo_def_vmin"] else None
            if K is not None:  # cannot reach the line in time: step into the ball's path where we can
                dbg.update(phase="block", target=[round(K[0], 3), round(K[1], 3)])
                return self._goto(pose, K, p, vret, others, slow_r=0.2,
                                  allow_back=True, tag="block", back_blend=p["return_back_vx"]), dbg
            # a ball coming at goal is worth touching (a block); a slow one we might knock in: go round it
            W = self._detour(pose, G, B, p) if approach < p["solo_def_vmin"] else G
            if W is not G:
                dbg["via"] = [round(W[0], 3), round(W[1], 3)]
            return self._goto(pose, W, p, vret, others, slow_r=0.2 if W is G else 0.0, allow_back=True, tag="return",
                              back_blend=p["return_back_vx"]), dbg
        if jockey:  # face the ball, mirror it sideways, stay in the trot; a 10 deg deadband
            herr = wrap(math.atan2(B[1] - pose.y, B[0] - pose.x) - pose.yaw)
            herr = math.copysign(max(0.0, abs(herr) - math.radians(10)), herr)
            bx, by = to_body(pose, p["k_goalie"] * dx, p["k_goalie"] * dy)
            vm = min(p["goalie_vmax"], p["gait_lo"])
            return Cmd(clamp(bx, -vm, vm), by, p["k_turn"] * herr, "jockey"), {"phase": "jockey", "target": tgt}
        # body parallel to the goal line: forward/back walking slides along it
        yaw_des = math.pi / 2 if abs(wrap(pose.yaw - math.pi / 2)) <= math.pi / 2 else -math.pi / 2
        if gm > 0:  # wide mouth: keep the chosen way round unless clearly the other (no flip-flop at the tie)
            if self.guard_yaw and abs(wrap(pose.yaw - self.guard_yaw)) < math.pi / 2 + 0.5:
                yaw_des = self.guard_yaw
            self.guard_yaw = yaw_des
        bx, by = to_body(pose, p["k_goalie"] * dx, p["k_goalie"] * dy + ff)
        bx = clamp(bx, -p["goalie_vmax"], p["goalie_vmax"])
        herr = wrap(yaw_des - pose.yaw)
        if gm > 0:  # wide mouth: a heading deadband (15 deg) so gait wobble is not chased with turn reversals
            herr = math.copysign(max(0.0, abs(herr) - math.radians(15)), herr)
        return Cmd(bx, by, p["k_turn"] * herr, "guard"), {"phase": "guard", "target": tgt}

    def _gk_target(self, B, bv, x_line: float, approach: float, t: float, p: dict):
        """Guard point (x_line, y) and the shuffle's feed-forward world vy. At rest in the middle of the mouth,
        shuffling +-gk_shuffle_m at gk_shuffle_hz (looks alive); shifted toward the ball's y (its predicted crossing
        of the guard line when it comes at goal) by a weight that grows as the ball gets closer (gk_far_m ->
        gk_near_m from the goal centre) or sooner (time to the line gk_tgoal_s -> 0.8 s); the shuffle fades out by
        the same weight. Clamped to the mouth less gk_edge_m."""
        f = self.f
        y_aim, T = B[1], math.inf
        if approach > 0.05 and not self.hold_goal:
            T, y_c = self._arrival(B, bv, x_line, p)
            if T < 3.0:
                y_aim = y_c
        d = math.hypot(B[0] - self.own_x, B[1])
        far, near = p["gk_far_m"], p["gk_near_m"]
        w = clamp((far - d) / max(1e-3, far - near), 0.0, 1.0)
        if T < math.inf:
            w = max(w, clamp((p["gk_tgoal_s"] - T) / max(1e-3, p["gk_tgoal_s"] - 0.8), 0.0, 1.0))
        if self.hold_goal:
            w = 0.0
        ph = 2 * math.pi * p["gk_shuffle_hz"] * t
        A = p["gk_shuffle_m"] * (1.0 - w)
        hw = f.goal_width / 2 - p["gk_edge_m"]
        y = clamp(w * y_aim + A * math.sin(ph), -hw, hw)
        ff = A * 2 * math.pi * p["gk_shuffle_hz"] * math.cos(ph) if abs(y) < hw else 0.0
        return (x_line, y), ff

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
            if self._out(W[0], W[1], p) <= 0.0:
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
            self.pushing = False
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
        """The walker switches gait at vx 0.5: keep the forward target out of (gait_lo, gait_hi). Below the band
        unless the want reaches gait_hi; once fast, stay >= gait_hi while the want is above the band middle.
        Down-shifts are immediate; an up-shift waits gait_hold_s after the last down-shift."""
        lo, hi = p["gait_lo"], p["gait_hi"]
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

    def _smooth(self, want: Cmd, dt: float, p: dict, t: float) -> Cmd:
        want = self._gait(want, t, p)
        self.out = shape(want, self.out, dt, p)
        lo, hi = p["gait_lo"], p["gait_hi"]
        if lo < self.out.vx < hi:  # step across the band instead of ramping through it
            v = hi if want.vx >= hi else lo if want.vx <= lo else self.out.vx
            self.out = Cmd(v, self.out.vy, self.out.wz, self.out.tag)
        self.hist.append((t, self.out.vx, self.out.vy, self.out.wz))
        return self.out


class Coach:
    """1v1: one decision per tick for both robots (notes/1v1-design.md). update(world, params) -> {robot id:
    per-call params for Brain.decide}; {} (not two fresh robots, or no ball) leaves each brain to play alone.
    - Possession (blue / red / loose, with hysteresis): gained after gap < poss_in_m with the ball within 50 deg of
      the nose for poss_dwell_s (the other robot not as close); lost (always to loose) after 0.2 s of gap > poss_out_m
      (not while a just-touched ball runs ahead), ball > 90 deg off the nose, the other robot as close, or the ball
      receding > 0.7 m/s.
    - The owner, or "first" on a loose ball (lower ETA, with hysteresis), attacks (coach_aim_y: the side of the mouth
      away from the other robot). The other shadows: J = jockey_m goal-side of the ball on the ball -> own goal line,
      clamped to the guard line (= goalie); > 0.35 m from J = recover. It dashes (challenge) when its ETA beats the
      opponent's by 0.8 - 0.6 aggression s for 0.2 s with the ball clear of the opponent and itself goal-side.
    Uses the brains' own ETA (_dr_eta at their last command) and shadow point."""

    def __init__(self, brains: dict, field: Field):
        self.b, self.f = brains, field
        self.reset()

    def reset(self) -> None:
        self.poss, self.last_owner, self.changes = "loose", None, 0
        self.cand: dict = {}       # id -> since when it meets the gain condition
        self.lose_t = None         # since when the owner meets a lose condition
        self.touch: dict = {}      # id -> last touch
        self.first, self.first_t = None, -1e9
        self.open: dict = {}       # id -> since when an opening holds
        self.chal: dict = {}       # id -> challenge start
        self.cool: dict = {}       # id -> no dash before
        self.task: dict = {}       # id -> last task
        self.aim: dict = {}        # id -> aim y in the attacked mouth

    def possession(self, t: float, poses: dict, B, bv, p: dict) -> str:
        """Ball owner id or "loose", with hysteresis; never owner -> owner directly."""
        c = self.f.ball_radius + self.f.robot_radius
        g = {k: math.hypot(q.x - B[0], q.y - B[1]) - c for k, q in poses.items()}
        off = {k: abs(wrap(math.atan2(B[1] - q.y, B[0] - q.x) - q.yaw)) for k, q in poses.items()}
        for k in poses:
            if g[k] < 0.03:
                self.touch[k] = t
        if self.poss in poses:
            k, q = self.poss, poses[self.poss]
            other = [j for j in poses if j != k]
            sp = math.hypot(bv[0], bv[1])
            ahead = (t - self.touch.get(k, -1e9) < 0.6 and sp > 0.05
                     and abs(wrap(math.atan2(bv[1], bv[0]) - q.yaw)) < math.radians(40))  # a dribbled ball runs ahead
            ux, uy = unit(B[0] - q.x, B[1] - q.y)
            lose = ((g[k] > p["poss_out_m"] and not ahead) or off[k] > math.pi / 2 or bv[0] * ux + bv[1] * uy > 0.7
                    or any(g[j] < p["poss_in_m"] for j in other))
            self.lose_t = (self.lose_t if self.lose_t is not None else t) if lose else None
            if self.lose_t is not None and t - self.lose_t >= 0.2:
                self.poss, self.lose_t, self.cand = "loose", None, {}
            return self.poss
        for k in poses:
            ok = g[k] < p["poss_in_m"] and off[k] < math.radians(50) and all(
                g[j] >= p["poss_in_m"] for j in poses if j != k)
            self.cand[k] = (t if self.cand.get(k) is None else self.cand[k]) if ok else None
            if ok and t - self.cand[k] >= p["poss_dwell_s"]:
                self.changes += k != self.last_owner
                self.poss = self.last_owner = k
                self.cand = {}
                break
        return self.poss

    def _eta(self, k, q: Pose, B, bv, p: dict) -> tuple[float, tuple]:
        """(seconds, meeting point) for robot k: fixed point of 'ball after our ETA'."""
        br, P, e = self.b[k], B, 0.0
        for _ in range(3):
            e = br._dr_eta(q, P, br._dr_dir(P, p), p)
            P = br._ball_at(B, bv, min(3.0, e), p)
        return e, P

    def _aim_y(self, k, br: Brain, q: Pose, B) -> float:
        """Aim y in the attacked mouth: the side whose line from the ball passes the opponent widest (kept unless the
        other side is 0.1 m wider): the dribble goes round a shadow standing on the ball-goal line."""
        f = self.f
        a = f.goal_width / 2 - f.ball_radius - 0.08
        gx = br.att_x
        w = {y: seg_dist(B, (gx, y), (q.x, q.y)) for y in (-a, a)}
        side = self.aim.get(k, a if w[a] >= w[-a] else -a)
        if w[-side] > w[side] + 0.1:
            side = -side
        self.aim[k] = side
        return side

    def update(self, world: World, params: dict) -> dict:
        p = merged(params)
        t, ball = world.t, world.ball
        ids = [k for k in self.b if k in world.robots and t - world.robots[k].t < p["other_stale_s"]]  # (a dropout
        # does not end the coaching: the stale brain stops on its own, the other keeps its task)
        if len(ids) != 2 or ball is None or t - ball.t > p["ball_stale_s"]:
            self.poss, self.chal = "loose", {}
            return {}
        poses = {k: world.robots[k].pose for k in ids}
        B, bv = (ball.x, ball.y), (ball.vx, ball.vy)
        poss = self.possession(t, poses, B, bv, p)
        eta = {k: self._eta(k, poses[k], B, bv, p) for k in ids}
        a, o = ids
        if poss in ids:
            self.first = poss
        elif self.first not in ids or t - world.robots[self.first].t > p["pose_stale_s"]:
            d = eta[a][0] - eta[o][0]
            self.first = (o if a == self.last_owner else a) if abs(d) < 0.2 else (a if d < 0 else o)
            self.first_t = t
        else:
            j = o if self.first == a else a
            if eta[j][0] < eta[self.first][0] - p["first_hyst_s"] and t - self.first_t >= p["first_hold_s"]:
                self.first, self.first_t = j, t
        c = self.f.ball_radius + self.f.robot_radius
        g = {k: math.hypot(poses[k].x - B[0], poses[k].y - B[1]) - c for k in ids}
        out = {}
        for k in ids:
            j, br, q = (o if k == a else a), self.b[k], poses[k]
            adv = eta[j][0] - eta[k][0]
            task, why = "attack", "on the ball" if poss == k else "first to the loose ball"
            if poss != k and self.first != k:
                s = 1.0 if br.own_x > 0 else -1.0
                goal_side = s * q.x > s * B[0]
                P = eta[k][1]
                ok = (adv >= 0.8 - 0.6 * clamp(p["aggression"], 0.0, 1.0) and g[j] > 0.20 and goal_side
                      and br._out(P[0], P[1], p) <= 0.0)
                self.open[k] = (t if self.open.get(k) is None else self.open[k]) if ok else None
                t0 = self.chal.get(k)
                if t0 is not None and (t - t0 > 2.5 or (t - t0 >= p["chal_hold_s"] and g[j] < 0.10 and g[k] > 0.40)):
                    self.chal.pop(k)
                    self.cool[k], t0 = t + p["chal_cool_s"], None
                if t0 is None and ok and t - self.open[k] >= 0.2 and t >= self.cool.get(k, -1e9):
                    self.chal[k] = t0 = t
                if t0 is not None:
                    task, why = "challenge", f"opening: gets there {adv:.1f} s first"
                else:
                    J, on_line = br._shadow(B, p)
                    if on_line:
                        task, why = "goalie", "in shooting range: on the line"
                    elif math.hypot(q.x - J[0], q.y - J[1]) > (0.15 if self.task.get(k) == "recover" else 0.35):
                        task, why = "recover", "beaten: back goal-side"
                    else:
                        task, why = "jockey", "goal-side, waiting for an opening"
            else:
                self.chal.pop(k, None)
            self.task[k] = task
            out[k] = {"coach_task": task, "coach_why": why, "coach_adv": round(adv, 2), "coach_poss": poss}
            if task in ("attack", "challenge"):
                out[k]["coach_aim_y"] = self._aim_y(k, br, poses[j], B)
        return out
