"""Pupper FC v1 simulator: 2D kinematic robots + rolling ball, with the measured error model.

Deterministic per seed (stdlib `random`). Time is the sim's own clock `self.t` (s);
`observe(now)` shifts capture times into the caller's clock so World.t and RobotObs.t
stay consistent whatever clock the caller uses.

Error knobs (params.json, `sim_*`; `sim_errors: 0` turns every error off):
  sim_dt                internal step (s)
  sim_gain_vx/vy/wz     measured travel per commanded (0.92 / 0.70 / 1.08)
  sim_lag_s             first-order actuation lag
  sim_drift_deg_per_m   yaw drift per metre of lateral travel (random sign/scale per episode)
  sim_yaw_walk_deg      yaw random walk while moving (deg/sqrt(s))
  sim_cam_hz, sim_latency_s, sim_pos_noise (m), sim_yaw_noise_deg
  sim_yaw_bias_a/b/c_deg  tag heading bias c + a*cos(yaw) + b*sin(yaw)
  sim_drop_near/far     tag dropout probability at the camera (north, +y) / far (south) edge
  sim_burst_per_s, sim_burst_s  random dropout bursts per robot
  sim_ball_drop, sim_ball_noise, sim_ball_vel_noise
Physics knobs (always on): sim_ball_decel (m/s^2), sim_ball_e_robot, sim_ball_e_wall,
  sim_goal_pause_s, sim_walls (1 = ball bounces off every edge except goal mouths,
  0 = ball leaving the field is an "out" event and a kickoff reset), sim_trap_s (a ball
  near a wall that moves < 5 cm for this long is a "trapped" event: the referee nudges it
  0.35 m inward, like the accepted manual ball replacement).
"""
from __future__ import annotations

import math
import random

from .core import Ball, Field, Pose, RobotCfg, RobotObs, World, wrap

SIM_DEFAULTS = {
    "sim_errors": 1, "sim_dt": 0.01,
    "sim_gain_vx": 0.92, "sim_gain_vy": 0.70, "sim_gain_wz": 1.08, "sim_lag_s": 0.15,
    "sim_drift_deg_per_m": 60.0, "sim_yaw_walk_deg": 2.0,
    "sim_cam_hz": 30.0, "sim_latency_s": 0.15, "sim_pos_noise": 0.01, "sim_yaw_noise_deg": 1.5,
    "sim_yaw_bias_a_deg": 10.0, "sim_yaw_bias_b_deg": 5.0, "sim_yaw_bias_c_deg": 0.0,
    "sim_drop_near": 0.02, "sim_drop_far": 0.25, "sim_burst_per_s": 0.1, "sim_burst_s": 0.4,
    "sim_ball_drop": 0.03, "sim_ball_noise": 0.01, "sim_ball_vel_noise": 0.03,
    "sim_ball_decel": 0.45, "sim_ball_e_robot": 0.4, "sim_ball_e_wall": 0.6,
    "sim_goal_pause_s": 1.0, "sim_walls": 1, "sim_trap_s": 6.0,
}
OFF = {"sim_gain_vx": 1.0, "sim_gain_vy": 1.0, "sim_gain_wz": 1.0, "sim_lag_s": 0.0,
       "sim_drift_deg_per_m": 0.0, "sim_yaw_walk_deg": 0.0, "sim_latency_s": 0.0,
       "sim_pos_noise": 0.0, "sim_yaw_noise_deg": 0.0, "sim_yaw_bias_a_deg": 0.0,
       "sim_yaw_bias_b_deg": 0.0, "sim_yaw_bias_c_deg": 0.0, "sim_drop_near": 0.0,
       "sim_drop_far": 0.0, "sim_burst_per_s": 0.0, "sim_ball_drop": 0.0,
       "sim_ball_noise": 0.0, "sim_ball_vel_noise": 0.0}
SCENARIOS = ("kickoff", "attack_solo", "shot_on_goal", "random")


class _Bot:
    __slots__ = ("cfg", "x", "y", "yaw", "vx", "vy", "wz", "cmd", "drift", "burst_until")

    def __init__(self, cfg: RobotCfg):
        self.cfg = cfg
        self.x = self.y = self.yaw = self.vx = self.vy = self.wz = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.drift = 0.0
        self.burst_until = -1.0


class Sim:
    def __init__(self, field: Field, robots: list[RobotCfg], params: dict, seed: int = 0):
        self.f, self.cfgs = field, list(robots)
        self.rng = random.Random(seed)
        self.set_params(params)
        self.events: list[dict] = []
        self.score = {"blue": 0, "red": 0}
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
        if ids is None and scenario in ("attack_solo", "shot_on_goal"):
            cfgs = cfgs[:1]
        self.bots = {c.id: _Bot(c) for c in cfgs}
        bx = by = bvx = bvy = 0.0
        if scenario == "kickoff":
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
            dx, dy = gx + s * (f.ball_radius + 0.03) - bx, ty - by  # residual speed counted fully past the line
            d = math.hypot(dx, dy)
            v0 = math.sqrt(r.uniform(0.2, 0.6) ** 2 + 2 * self.p["sim_ball_decel"] * d)
            bvx, bvy = v0 * dx / d, v0 * dy / d
            if g:
                g.x, g.y = gx - s * (0.25 + r.uniform(-0.05, 0.05)), r.uniform(-0.25, 0.25)
                g.yaw = wrap(math.copysign(math.pi / 2, r.random() - 0.5) + r.uniform(-0.4, 0.4))
        for b in self.bots.values():
            k = math.radians(self.p["sim_drift_deg_per_m"]) * r.uniform(0.5, 1.5)
            b.drift = k if r.random() < 0.5 else -k
        self.ball = [bx, by, bvx, bvy]
        self.pause_until = -1.0
        self.trap_ref = (self.t, bx, by)
        self.frames: list[tuple] = []
        self.next_frame = self.t
        self.seen_ball: Ball | None = None
        self.seen: dict[str, RobotObs] = {}
        self._capture()
        self._deliver()

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
        alpha = 1.0 if p["sim_lag_s"] <= 0 else min(1.0, h / p["sim_lag_s"])
        walk = math.radians(p["sim_yaw_walk_deg"])
        for b in self.bots.values():
            cvx, cvy, cwz = b.cmd
            b.vx += (p["sim_gain_vx"] * cvx - b.vx) * alpha
            b.vy += (p["sim_gain_vy"] * cvy - b.vy) * alpha
            b.wz += (p["sim_gain_wz"] * cwz - b.wz) * alpha
            c, s = math.cos(b.yaw), math.sin(b.yaw)
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
                if 1e-9 < d < 2 * rr:
                    push = (2 * rr - d) / 2
                    a.x, a.y = a.x - dx / d * push, a.y - dy / d * push
                    b.x, b.y = b.x + dx / d * push, b.y + dy / d * push
        lx, ly = f.length / 2 - 0.08, f.width / 2 - 0.08  # body is narrower than its clearance circle
        for b in bots:
            b.x, b.y = max(-lx, min(lx, b.x)), max(-ly, min(ly, b.y))

    def _ball_step(self, h: float) -> None:
        p, f = self.p, self.f
        bx, by, vx, vy = self.ball
        sp = math.hypot(vx, vy)
        if sp > 0:
            ns = max(0.0, sp - p["sim_ball_decel"] * h)
            vx, vy = vx * ns / sp, vy * ns / sp
        bx, by = bx + vx * h, by + vy * h
        br, rr = f.ball_radius, f.robot_radius
        for b in self.bots.values():
            dx, dy = bx - b.x, by - b.y
            d = math.hypot(dx, dy)
            if 1e-9 < d < br + rr:
                nx, ny = dx / d, dy / d
                bx, by = b.x + nx * (br + rr), b.y + ny * (br + rr)
                c, s = math.cos(b.yaw), math.sin(b.yaw)
                pvx = c * b.vx - s * b.vy - b.wz * ny * rr  # robot surface velocity at contact
                pvy = s * b.vx + c * b.vy + b.wz * nx * rr
                rel = (vx - pvx) * nx + (vy - pvy) * ny
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
        if math.hypot(bx - tx, by - ty) > 0.05:
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
                b.burst_until = self.t + p["sim_burst_s"]
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
