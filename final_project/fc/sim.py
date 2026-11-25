"""Pupper FC v1 simulator: 2D kinematic robots + rolling ball, with the measured error model.

Deterministic per seed (stdlib `random`). Time is the sim's own clock `self.t` (s);
`observe(now)` shifts capture times into the caller's clock so World.t and RobotObs.t
stay consistent whatever clock the caller uses.

Error knobs (params.json, `sim_*`; `sim_errors: 0` turns every error off):
  sim_dt                internal step (s)
  sim_gain_vx/vy/wz     measured travel per commanded (0.92 / 0.70 / 1.08); vx gain holds up to
                        sim_trot_knee, then the speed curve below takes over
  sim_lag_s             first-order actuation lag (0.15 s also gives the measured stop: 0.11 m from 0.73 m/s)
Walker speed model (the deployed policy, fc/notes/fast-gait.md; sim_speed_curve: 0 = plain gains):
  sim_trot_knee, sim_trot_top   trot: gain up to the knee (0.3), then linear to sim_trot_top at cmd 0.49
  sim_gait_switch       cmd vx at or above this runs the fast gait (policy gallop_speed 0.5)
  sim_fast_v060/v075/v100/v150  fast-gait speed at cmd 0.60/0.75/1.00/1.50 (piecewise linear)
  sim_vx_clip, sim_vx_back_max  policy/controller clips on forward and backward cmd vx
  sim_wz_clip           policy clip on cmd wz (turns track to 2.0 rad/s)
  sim_fast_wobble, sim_fast_wobble_hz  fast-gait yaw-rate wobble (amplitude rad/s, frequency)
  sim_gait_kick, sim_gait_kick_s  yaw-rate kick of random sign for this long after each gait switch
  sim_drift_deg_per_m   yaw drift per metre of lateral travel (random sign/scale per episode)
  sim_yaw_walk_deg      yaw random walk while moving (deg/sqrt(s))
  sim_cam_hz, sim_latency_s, sim_pos_noise (m), sim_yaw_noise_deg
  sim_yaw_bias_a/b/c_deg  tag heading bias c + a*cos(yaw) + b*sin(yaw)
  sim_drop_near/far     tag dropout probability at the camera (north, +y) / far (south) edge
  sim_burst_per_s, sim_burst_s  random dropout bursts per robot; sim_burst_s_max > sim_burst_s draws each
                        burst's length uniformly from [sim_burst_s, sim_burst_s_max] (floor: 0.3-1.5 s)
  sim_fast_dir_deg      the fast gait walks this many degrees off its heading (floor: about -8, to the right)
  sim_ball_drop, sim_ball_noise, sim_ball_vel_noise
Ball curve (sim_ball_curve, rad/s at 1 m/s; 0 = off): a slightly non-spherical ball on carpet curves on its own,
  more the slower it rolls: heading rate = k / max(speed, 0.1) with k = sim_ball_curve * U(-1, 1), redrawn at every
  robot touch or kick (Nov 25 floor: soft touches let the ball curve away; hard, straight hits stayed straight).
Physics knobs (always on): sim_ball_decel (m/s^2), sim_ball_e_robot, sim_ball_e_wall,
  sim_goal_pause_s, sim_goal_bounce (with Sim.keep_ball, autoplay: the chance a goal bounces back out of the net
  instead of resting in it; no kickoff reset), sim_walls (1 = ball bounces off every edge except goal mouths,
  0 = ball leaving the field is an "out" event and a kickoff reset), sim_trap_s (a ball
  near a wall that stays within 10 cm for this long is a "trapped" event: the referee nudges it
  0.35 m inward, like the accepted manual ball replacement).

Scripted human (for one-robot play): `kick(x, y, vx, vy)` places the ball and rolls it now;
`shoot(tx, ty, v_end, x=None, y=None)` rolls it from (x, y) (default: where it is) so it reaches
(tx, ty) still moving at v_end; `schedule(t, **shoot_kwargs)` does that at sim time t (with
`spot=(x, y)`: from the ball if no robot is within 0.45 m of it, else from the spot). Each logs a
"kick" event. Scenarios "return" (first robot upfield, ball at rest in its attack half, a shot at our
goal at a random 1-4 s) and "rolling_ball" (first robot in its own half, ball rolled across the field
from a sideline) use them.
"""
from __future__ import annotations

import math
import random
from collections import deque

from .core import Ball, Field, Pose, RobotCfg, RobotObs, World, wrap

SIM_DEFAULTS = {
    "sim_errors": 1, "sim_dt": 0.01,
    "sim_gain_vx": 0.92, "sim_gain_vy": 0.70, "sim_gain_wz": 1.08, "sim_lag_s": 0.15,
    "sim_drift_deg_per_m": 60.0, "sim_yaw_walk_deg": 2.0,
    "sim_cam_hz": 30.0, "sim_latency_s": 0.15, "sim_pos_noise": 0.01, "sim_yaw_noise_deg": 1.5,
    "sim_yaw_bias_a_deg": 10.0, "sim_yaw_bias_b_deg": 5.0, "sim_yaw_bias_c_deg": 0.0,
    "sim_drop_near": 0.02, "sim_drop_far": 0.25, "sim_burst_per_s": 0.1, "sim_burst_s": 0.4, "sim_burst_s_max": 0.0,
    "sim_ball_drop": 0.03, "sim_ball_noise": 0.01, "sim_ball_vel_noise": 0.03,
    "sim_ball_decel": 0.45, "sim_ball_e_robot": 0.4, "sim_ball_e_wall": 0.6,
    "sim_goal_pause_s": 1.0, "sim_walls": 1, "sim_trap_s": 6.0,
    "sim_speed_curve": 1, "sim_trot_knee": 0.30, "sim_trot_top": 0.36, "sim_gait_switch": 0.5,
    "sim_fast_v060": 0.62, "sim_fast_v075": 0.73, "sim_fast_v100": 0.86, "sim_fast_v150": 1.14,
    "sim_vx_clip": 1.5, "sim_vx_back_max": 0.3, "sim_wz_clip": 2.0,
    "sim_fast_wobble": 0.5, "sim_fast_wobble_hz": 2.5, "sim_gait_kick": 0.3, "sim_gait_kick_s": 0.3,
    "sim_fast_dir_deg": 0.0, "sim_ball_curve": 0.0, "sim_goal_bounce": 0.35,
}
OFF = {"sim_gain_vx": 1.0, "sim_gain_vy": 1.0, "sim_gain_wz": 1.0, "sim_lag_s": 0.0,
       "sim_drift_deg_per_m": 0.0, "sim_yaw_walk_deg": 0.0, "sim_latency_s": 0.0,
       "sim_pos_noise": 0.0, "sim_yaw_noise_deg": 0.0, "sim_yaw_bias_a_deg": 0.0,
       "sim_yaw_bias_b_deg": 0.0, "sim_yaw_bias_c_deg": 0.0, "sim_drop_near": 0.0,
       "sim_drop_far": 0.0, "sim_burst_per_s": 0.0, "sim_ball_drop": 0.0,
       "sim_ball_noise": 0.0, "sim_ball_vel_noise": 0.0,
       "sim_speed_curve": 0, "sim_fast_wobble": 0.0, "sim_gait_kick": 0.0, "sim_fast_dir_deg": 0.0,
       "sim_ball_curve": 0.0}
SCENARIOS = ("kickoff", "attack_solo", "shot_on_goal", "random", "return", "rolling_ball")


class _Bot:
    __slots__ = ("cfg", "x", "y", "yaw", "vx", "vy", "wz", "cmd", "drift", "burst_until",
                 "fast", "switches", "kick", "kick_until", "phase", "touch")

    def __init__(self, cfg: RobotCfg):
        self.cfg = cfg
        self.x = self.y = self.yaw = self.vx = self.vy = self.wz = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.drift = 0.0
        self.burst_until = -1.0
        self.fast = False         # running the fast gait (cmd vx >= sim_gait_switch)
        self.switches = 0         # gait switches this episode
        self.kick, self.kick_until, self.phase = 0.0, -1.0, 0.0
        self.touch = set()        # what this robot is touching now: "ball", other robot ids


def vx_actual(c: float, p: dict) -> float:
    """Steady-state forward speed (m/s) the walker reaches for a commanded vx (the measured curve)."""
    g = p["sim_gain_vx"]
    if not p["sim_speed_curve"]:
        return g * c
    if c < 0:
        return -g * min(-c, p["sim_vx_back_max"])
    c = min(c, p["sim_vx_clip"])
    if c < p["sim_gait_switch"]:  # trot: floor gain to the knee, then it saturates toward the switch
        k = p["sim_trot_knee"]
        return g * c if c <= k else g * k + (p["sim_trot_top"] - g * k) * (c - k) / (0.49 - k)
    pts = ((0.60, p["sim_fast_v060"]), (0.75, p["sim_fast_v075"]), (1.00, p["sim_fast_v100"]),
           (1.50, p["sim_fast_v150"]))
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if c <= x1 or x1 == pts[-1][0]:
            return y0 + (y1 - y0) * (c - x0) / (x1 - x0)
    return pts[-1][1]


class Sim:
    def __init__(self, field: Field, robots: list[RobotCfg], params: dict, seed: int = 0):
        self.f, self.cfgs = field, list(robots)
        self.rng = random.Random(seed)
        self.set_params(params)
        self.events: list[dict] = []
        # first touches, for contact-speed stats: (t, robot, body speed, closing speed), m/s
        self.contacts: deque = deque(maxlen=5000)
        self.bumps: deque = deque(maxlen=5000)  # robot-robot first touches: (t, a, b, closing speed)
        self.score = {"blue": 0, "red": 0}
        self.keep_ball = False  # after a goal: no kickoff reset; the ball rests in the net or bounces back out (autoplay)
        self.reset("kickoff")

    def set_params(self, params: dict) -> None:
        p = {**SIM_DEFAULTS, **params}
        if not p["sim_errors"]:
            p.update(OFF)
        self.p = p

    # ------------------------------------------------------------------ scenarios
    def reset(self, scenario: str = "kickoff", **opts) -> None:
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario {scenario!r}")
        self.scenario = scenario
        r, f = self.rng, self.f
        hl, hw = f.length / 2, f.width / 2
        self.t = getattr(self, "t", 0.0)
        ids = opts.get("robots")
        cfgs = [c for c in self.cfgs if ids is None or c.id in ids]
        if ids is None and scenario in ("attack_solo", "shot_on_goal", "return", "rolling_ball"):
            cfgs = cfgs[:1]
        self.bots = {c.id: _Bot(c) for c in cfgs}
        bx = by = bvx = bvy = 0.0
        self.scheduled: list[tuple[float, dict]] = []
        if scenario in ("return", "rolling_ball"):
            g = next(iter(self.bots.values()), None)
            s = -1.0 if (opts.get("team") or (g.cfg.team if g else "blue")) == "blue" else 1.0  # own goal side
            if scenario == "return":  # upfield in the attack half, ball resting near it, a shot comes later
                if g:
                    g.x, g.y = -s * r.uniform(0.2, 0.9), r.uniform(-0.6, 0.6)
                    g.yaw = wrap((0.0 if s < 0 else math.pi) + r.uniform(-0.8, 0.8))
                while True:
                    bx, by = -s * r.uniform(0.3, 1.0), r.uniform(-0.6, 0.6)
                    if not g or math.hypot(bx - g.x, by - g.y) > 0.5:
                        break
                ty = r.uniform(-(f.goal_width / 2 - f.ball_radius), f.goal_width / 2 - f.ball_radius)
                self.scheduled.append((self.t + r.uniform(1.0, 4.0),
                                       {"tx": s * (hl + f.ball_radius + 0.03), "ty": ty, "v_end": r.uniform(0.1, 0.5),
                                        "spot": (bx, by)}))
            else:  # rolling_ball: robot in its own half, ball rolled across from a sideline
                if g:
                    g.x, g.y = s * r.uniform(0.3, 1.0), r.uniform(-0.6, 0.6)
                    g.yaw = r.uniform(-math.pi, math.pi)
                side = r.choice((-1.0, 1.0))
                bx, by = -s * r.uniform(-0.3, 0.7), side * (hw - 0.15)
                sp, ang = r.uniform(0.5, 1.1), r.uniform(-0.5, 0.5)
                bvx, bvy = sp * math.sin(ang), -side * sp * math.cos(ang)
        elif scenario == "kickoff":
            for b in self.bots.values():
                west = b.cfg.team == "blue"
                b.x = (-0.7 if west else 0.7) + r.uniform(-0.05, 0.05)
                b.y = r.uniform(-0.1, 0.1)
                b.yaw = wrap((0.0 if west else math.pi) + r.uniform(-0.2, 0.2))
            bx, by = r.uniform(-0.03, 0.03), r.uniform(-0.03, 0.03)
        elif scenario in ("attack_solo", "random"):
            for b in self.bots.values():
                b.x, b.y = r.uniform(-hl + 0.3, hl - 0.3), r.uniform(-hw + 0.3, hw - 0.3)
                b.yaw = r.uniform(-math.pi, math.pi)
            while True:
                bx, by = r.uniform(-hl + 0.35, hl - 0.35), r.uniform(-hw + 0.25, hw - 0.25)
                if all(math.hypot(bx - b.x, by - b.y) > 0.5 for b in self.bots.values()):
                    break
        else:  # shot_on_goal: first robot (if any) keeps goal, ball rolls at a point inside its mouth
            g = next(iter(self.bots.values()), None)
            team = opts.get("team") or (g.cfg.team if g else "blue")
            s = -1.0 if team == "blue" else 1.0  # side of own goal
            gx = s * hl
            bx, by = gx - s * r.uniform(0.9, 1.4), r.uniform(-0.6, 0.6)
            ty = r.uniform(-(f.goal_width / 2 - f.ball_radius), f.goal_width / 2 - f.ball_radius)
            # residual speed counted fully past the line
            bvx, bvy = self.aim(bx, by, gx + s * (f.ball_radius + 0.03), ty, r.uniform(0.2, 0.6))
            if g:
                g.x, g.y = gx - s * (0.25 + r.uniform(-0.05, 0.05)), r.uniform(-0.25, 0.25)
                g.yaw = wrap(math.copysign(math.pi / 2, r.random() - 0.5) + r.uniform(-0.4, 0.4))
        for b in self.bots.values():
            k = math.radians(self.p["sim_drift_deg_per_m"]) * r.uniform(0.5, 1.5)
            b.drift = k if r.random() < 0.5 else -k
        self.ball = [bx, by, bvx, bvy]
        self.netted = False
        self.curve = self.p["sim_ball_curve"] * r.uniform(-1.0, 1.0)
        self.pause_until = -1.0
        self.trap_ref = (self.t, bx, by)
        self.frames: list[tuple] = []
        self.next_frame = self.t
        self.seen_ball: Ball | None = None
        self.seen: dict[str, RobotObs] = {}
        self._capture()
        self._deliver()

    # ------------------------------------------------------------------ scripted human
    def aim(self, bx: float, by: float, tx: float, ty: float, v_end: float) -> tuple[float, float]:
        """Ball velocity at (bx, by) that reaches (tx, ty) still rolling at v_end (rolling decel)."""
        dx, dy = tx - bx, ty - by
        d = max(1e-6, math.hypot(dx, dy))
        v0 = math.sqrt(v_end ** 2 + 2 * self.p["sim_ball_decel"] * d)
        return v0 * dx / d, v0 * dy / d

    def kick(self, x: float, y: float, vx: float, vy: float) -> bool:
        """Place the ball at (x, y) rolling at (vx, vy), like a human rolling it. No-op after a goal."""
        if self.pause_until > 0:
            return False
        self.ball = [x, y, vx, vy]
        self.netted = False
        self.curve = self.p["sim_ball_curve"] * self.rng.uniform(-1.0, 1.0)
        self.trap_ref = (self.t, x, y)
        self.events.append({"t": round(self.t, 3), "type": "kick", "x": round(x, 3), "y": round(y, 3),
                            "vx": round(vx, 3), "vy": round(vy, 3)})
        return True

    def shoot(self, tx: float, ty: float, v_end: float, x: float | None = None, y: float | None = None) -> bool:
        x = self.ball[0] if x is None else x
        y = self.ball[1] if y is None else y
        return self.kick(x, y, *self.aim(x, y, tx, ty, v_end))

    def schedule(self, t: float, **shot) -> None:
        self.scheduled.append((t, shot))
        self.scheduled.sort(key=lambda e: e[0])

    # ------------------------------------------------------------------ stepping
    def step(self, cmds: dict, dt: float) -> None:
        h = self.p["sim_dt"]
        n = max(1, int(round(dt / h)))
        h = dt / n
        for k, c in cmds.items():
            if k in self.bots:
                self.bots[k].cmd = (c.vx, c.vy, c.wz)
        for _ in range(n):
            self._substep(h)

    def _substep(self, h: float) -> None:
        p, f, r = self.p, self.f, self.rng
        self.t += h
        if self.pause_until > 0 and self.t >= self.pause_until:
            self.reset("kickoff", robots=list(self.bots))
            return
        while self.scheduled and self.t >= self.scheduled[0][0]:
            shot = dict(self.scheduled.pop(0)[1])
            spot = shot.pop("spot", None)
            near = any(math.hypot(self.ball[0] - b.x, self.ball[1] - b.y) < 0.45 for b in self.bots.values())
            if spot and near:  # a robot is on the ball: the human picks it up and shoots from the spot
                shot["x"], shot["y"] = spot
            self.shoot(**shot)
        alpha = 1.0 if p["sim_lag_s"] <= 0 else min(1.0, h / p["sim_lag_s"])
        walk = math.radians(p["sim_yaw_walk_deg"])
        for b in self.bots.values():
            cvx, cvy, cwz = b.cmd
            fast = cvx >= p["sim_gait_switch"]
            if fast != b.fast:
                b.fast, b.switches = fast, b.switches + 1
                b.kick, b.kick_until = r.choice((-1.0, 1.0)) * p["sim_gait_kick"], self.t + p["sim_gait_kick_s"]
            wz_t = p["sim_gain_wz"] * max(-p["sim_wz_clip"], min(p["sim_wz_clip"], cwz))
            if b.fast and p["sim_fast_wobble"]:
                b.phase += 2 * math.pi * p["sim_fast_wobble_hz"] * h
                wz_t += p["sim_fast_wobble"] * math.sin(b.phase)
            if self.t < b.kick_until:
                wz_t += b.kick
            b.vx += (vx_actual(cvx, p) - b.vx) * alpha
            b.vy += (p["sim_gain_vy"] * cvy - b.vy) * alpha
            b.wz += (wz_t - b.wz) * alpha
            hd = b.yaw + (math.radians(p["sim_fast_dir_deg"]) if b.fast else 0.0)  # walking direction
            c, s = math.cos(hd), math.sin(hd)
            b.x += (c * b.vx - s * b.vy) * h
            b.y += (s * b.vx + c * b.vy) * h
            moving = abs(b.vx) + abs(b.vy) + 0.2 * abs(b.wz) > 0.02
            b.yaw = wrap(b.yaw + b.wz * h + b.drift * abs(b.vy) * h
                         + (r.gauss(0.0, walk * math.sqrt(h)) if moving and walk else 0.0))
        self._collide_robots()
        if self.pause_until < 0:
            self._ball_step(h)
        if self.t >= self.next_frame - 1e-9:
            self._capture()
            self.next_frame += 1.0 / p["sim_cam_hz"]
        self._deliver()

    def _collide_robots(self) -> None:
        f = self.f
        bots = list(self.bots.values())
        rr = f.robot_radius
        for i in range(len(bots)):
            for j in range(i + 1, len(bots)):
                a, b = bots[i], bots[j]
                dx, dy = b.x - a.x, b.y - a.y
                d = math.hypot(dx, dy)
                if d < 2 * rr and b.cfg.id not in a.touch and d > 1e-9:
                    ux, uy = dx / d, dy / d
                    (avx, avy), (bvx, bvy) = self._vel(a), self._vel(b)
                    self.bumps.append((round(self.t, 3), a.cfg.id, b.cfg.id,
                                       round((avx - bvx) * ux + (avy - bvy) * uy, 3)))
                    a.touch.add(b.cfg.id)
                elif d > 2 * rr + 0.02:
                    a.touch.discard(b.cfg.id)
                if 1e-9 < d < 2 * rr:
                    push = (2 * rr - d) / 2
                    a.x, a.y = a.x - dx / d * push, a.y - dy / d * push
                    b.x, b.y = b.x + dx / d * push, b.y + dy / d * push
        lx, ly = f.length / 2 - 0.08, f.width / 2 - 0.08  # body is narrower than its clearance circle
        for b in bots:
            b.x, b.y = max(-lx, min(lx, b.x)), max(-ly, min(ly, b.y))

    @staticmethod
    def _vel(b: _Bot) -> tuple[float, float]:
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        return c * b.vx - s * b.vy, s * b.vx + c * b.vy

    def _ball_step(self, h: float) -> None:
        p, f = self.p, self.f
        if self.netted:   # resting in the net: the robot cannot reach it, a person rolls it out (kick)
            return
        bx, by, vx, vy = self.ball
        sp = math.hypot(vx, vy)
        if sp > 0:
            ns = max(0.0, sp - p["sim_ball_decel"] * h)
            vx, vy = vx * ns / sp, vy * ns / sp
            if self.curve and ns > 0:  # curves on its own, more the slower it rolls
                q = self.curve / max(ns, 0.1) * h
                vx, vy = vx * math.cos(q) - vy * math.sin(q), vx * math.sin(q) + vy * math.cos(q)
        bx, by = bx + vx * h, by + vy * h
        br, rr = f.ball_radius, f.robot_radius
        for b in self.bots.values():
            dx, dy = bx - b.x, by - b.y
            d = math.hypot(dx, dy)
            if d > br + rr + 0.02:
                b.touch.discard("ball")
            if 1e-9 < d < br + rr:
                nx, ny = dx / d, dy / d
                bx, by = b.x + nx * (br + rr), b.y + ny * (br + rr)
                c, s = math.cos(b.yaw), math.sin(b.yaw)
                pvx = c * b.vx - s * b.vy - b.wz * ny * rr  # robot surface velocity at contact
                pvy = s * b.vx + c * b.vy + b.wz * nx * rr
                rel = (vx - pvx) * nx + (vy - pvy) * ny
                if "ball" not in b.touch:
                    b.touch.add("ball")
                    self.curve = p["sim_ball_curve"] * self.rng.uniform(-1.0, 1.0)
                    self.contacts.append((round(self.t, 3), b.cfg.id, round(math.hypot(b.vx, b.vy), 3),
                                          round(max(0.0, -rel), 3)))
                if rel < 0:
                    vx -= (1 + p["sim_ball_e_robot"]) * rel * nx
                    vy -= (1 + p["sim_ball_e_robot"]) * rel * ny
        hl, hw, gw = f.length / 2, f.width / 2, f.goal_width / 2
        e = p["sim_ball_e_wall"]
        if abs(bx) >= hl + br and abs(by) < gw:
            side = "east" if bx > 0 else "west"
            team = "blue" if side == "east" else "red"
            self.score[team] += 1
            self.events.append({"t": round(self.t, 3), "type": "goal", "goal": side, "team": team})
            if self.keep_ball:
                s = 1.0 if bx > 0 else -1.0
                if self.rng.random() < p["sim_goal_bounce"]:   # hits the back of the net, rolls back into the field
                    self.ball = [s * (hl - br - 0.01), by, -s * self.rng.uniform(0.2, 0.6), self.rng.uniform(-0.2, 0.2)]
                else:                                          # stays in the net
                    self.ball = [s * (hl + min(f.goal_depth / 2, 0.15)), by, 0.0, 0.0]
                    self.netted = True
                return
            self.ball = [bx, by, 0.0, 0.0]
            self.pause_until = self.t + p["sim_goal_pause_s"]
            return
        if p["sim_walls"]:
            if abs(by) > hw - br:
                by = math.copysign(hw - br, by)
                vy = -e * vy if vy * by > 0 else vy
            if abs(bx) > hl - br and abs(by) >= gw:
                bx = math.copysign(hl - br, bx)
                vx = -e * vx if vx * bx > 0 else vx
            for b in self.bots.values():  # ball pinned on a wall pushes the robot back
                dx, dy = bx - b.x, by - b.y
                d = math.hypot(dx, dy)
                if 1e-9 < d < br + rr:
                    b.x, b.y = bx - dx / d * (br + rr), by - dy / d * (br + rr)
        elif abs(by) > hw or (abs(bx) > hl and abs(by) >= gw):
            self.events.append({"t": round(self.t, 3), "type": "out", "x": round(bx, 3), "y": round(by, 3)})
            self.ball = [bx, by, 0.0, 0.0]
            self.pause_until = self.t + p["sim_goal_pause_s"]
            return
        self.ball = [bx, by, vx, vy]
        tt, tx, ty = self.trap_ref
        if math.hypot(bx - tx, by - ty) > 0.10:  # a ball rattling inside 10 cm is still trapped
            self.trap_ref = (self.t, bx, by)
        elif (self.t - tt > p["sim_trap_s"] and (abs(bx) > hl - 0.25 or abs(by) > hw - 0.25)
              and any(math.hypot(bx - b.x, by - b.y) < br + rr + 0.03 for b in self.bots.values())):
            nx = -math.copysign(0.35, bx) if abs(bx) > hl - 0.25 else 0.0
            ny = -math.copysign(0.35, by) if abs(by) > hw - 0.25 else 0.0
            self.events.append({"t": round(self.t, 3), "type": "trapped", "x": round(bx, 3), "y": round(by, 3)})
            self.ball = [bx + nx, by + ny, 0.0, 0.0]
            self.trap_ref = (self.t, bx + nx, by + ny)

    # ------------------------------------------------------------------ camera
    def _capture(self) -> None:
        """Take one noisy camera frame of the current truth; it becomes visible after the latency."""
        p, f, r = self.p, self.f, self.rng
        frame_r = {}
        for k, b in self.bots.items():
            if r.random() < p["sim_burst_per_s"] / p["sim_cam_hz"]:
                lo, hi = p["sim_burst_s"], p["sim_burst_s_max"]
                b.burst_until = self.t + (r.uniform(lo, hi) if hi > lo else lo)
            far = min(1.0, max(0.0, (f.width / 2 - b.y) / f.width))
            pdrop = p["sim_drop_near"] + (p["sim_drop_far"] - p["sim_drop_near"]) * far
            if self.t < b.burst_until or r.random() < pdrop:
                continue
            bias = math.radians(p["sim_yaw_bias_c_deg"] + p["sim_yaw_bias_a_deg"] * math.cos(b.yaw)
                                + p["sim_yaw_bias_b_deg"] * math.sin(b.yaw))
            frame_r[k] = Pose(b.x + r.gauss(0, p["sim_pos_noise"]), b.y + r.gauss(0, p["sim_pos_noise"]),
                              wrap(b.yaw + bias + r.gauss(0, math.radians(p["sim_yaw_noise_deg"]))))
        ball = None
        if r.random() >= p["sim_ball_drop"]:
            bx, by, vx, vy = self.ball
            n, vn = p["sim_ball_noise"], p["sim_ball_vel_noise"]
            ball = (bx + r.gauss(0, n), by + r.gauss(0, n), vx + r.gauss(0, vn), vy + r.gauss(0, vn))
        self.frames.append((self.t, ball, frame_r))

    def _deliver(self) -> None:
        lat = self.p["sim_latency_s"]
        while self.frames and self.frames[0][0] + lat <= self.t + 1e-9:
            tc, ball, robots = self.frames.pop(0)
            if ball is not None:
                self.seen_ball = Ball(ball[0], ball[1], ball[2], ball[3], tc)
            for k, pose in robots.items():
                self.seen[k] = RobotObs(k, pose, tc)

    def observe(self, now: float | None = None) -> World:
        """Latest delivered camera view (delayed, noisy). Missing tags keep their last capture time."""
        off = 0.0 if now is None else now - self.t
        t = self.t + off
        ball = self.seen_ball
        if ball is not None and off:
            ball = Ball(ball.x, ball.y, ball.vx, ball.vy, ball.t + off)
        robots = {k: RobotObs(k, o.pose, o.t + off) for k, o in self.seen.items() if k in self.bots}
        return World(t, ball, robots)

    def truth(self) -> dict:
        bx, by, vx, vy = self.ball
        return {"t": self.t, "ball": {"x": bx, "y": by, "vx": vx, "vy": vy},
                "robots": {k: {"x": b.x, "y": b.y, "yaw": b.yaw, "team": b.cfg.team} for k, b in self.bots.items()},
                "score": dict(self.score), "paused": self.pause_until > 0}
