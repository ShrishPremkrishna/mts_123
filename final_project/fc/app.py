"""Pupper FC v1 laptop app: one 30 Hz loop + a stdlib HTTP UI.

    cd final_project && python -m fc.app --source sim
    python -m fc.app --source camera [--video FILE|DIR|synthetic] [--robots blue] [--port 8800]

World (Sim or Eye) -> Brain.decide per active robot -> Cmd -> Sim.step or Link.send -> run log.
Every test is a named mode with its own run folder (see runlog.py). Stop is always immediate.
Core (this file, link.py, runlog.py) is stdlib only; the camera source needs OpenCV.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import shlex
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, is_dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .link import Link
from .core import CONFIG, Cmd, Field, RobotCfg, World, load_field, load_params, load_robots, wrap
from .runlog import RUNS, RunLog, list_runs

HERE = Path(__file__).parent
UI = HERE / "ui"
PRESETS = CONFIG / "presets"
FINAL_PROJECT = HERE.parent

# Knobs the app (tests, eye) needs that params.json does not carry yet. params.json wins.
APP_DEFAULTS = {
    "test_timeout_s": 45.0,     # push_ball / goalie / goto give up after this
    "match_s": 300.0,           # match length
    "goalie_shot_s": 5.0,       # sim goalie: a new shot this often (no goal by then = saved)
    "goto_tol_m": 0.08,         # goto: close enough
    "goto_kp_lin": 1.0,
    "goto_kp_yaw": 2.0,
    "stale_stop_s": 0.5,        # robot observation older than this -> stop that robot (not in drive)
    "teleop_timeout_s": 0.4,    # drive: no key update for this long -> stop
    "eye_floor_size": 0.15875,  # corner tag black square side, m (Tej, 2026-11-20)
    "eye_tag_height": 0.24,     # floor -> robot tag while standing, m (Tej, 2026-11-20)
    "eye_tag_offset_x": 0.065,  # robot tag centre ahead of body centre, m
    "eye_yaw_offset_deg": 0.0,  # add if the tag top does not point at the nose (Tej used -180)
    "eye_latency_s": 0.12,      # glass-to-python delay
}
# Hard limits for live edits. Speed ceilings mirror link.CEIL.
LIMITS = {
    "loop_hz": (5, 60), "max_vx": (0.0, 0.5), "max_vy": (0.0, 0.3), "max_wz": (0.0, 2.0),
    "accel_vx": (0.05, 5.0), "accel_wz": (0.1, 20.0), "ttl_s": (0.05, 0.5),
    "test_timeout_s": (1, 1800), "goalie_shot_s": (1, 30), "match_s": (10, 3600), "goto_tol_m": (0.02, 1.0),
    "goto_kp_lin": (0.0, 5.0), "goto_kp_yaw": (0.0, 10.0), "stale_stop_s": (0.1, 2.0),
    "teleop_timeout_s": (0.1, 2.0), "eye_floor_size": (0.03, 0.5), "eye_tag_height": (0.0, 0.5),
    "eye_tag_offset_x": (-0.3, 0.3), "eye_yaw_offset_deg": (-360, 360), "eye_latency_s": (0.0, 1.0),
}
LIMITS.update({k: (0.0, 0.5) for k in ("push_vx", "hb_charge_vx", "hb_backoff_vx")})
LIMITS.update({"search_wz": (0.0, 2.0), "accel_vy": (0.05, 5.0), "predict_self": (0, 1)})
CHOICES = {"role_mode": ["auto", "attack", "defend"], "contact_mode": ["push", "headbutt"],
           "ball_lost": ["hold", "search"], "goalie_style": ["sideways", "facing"]}
TESTS = ["drive", "goto", "push_ball", "goalie", "match", "sim_eval"]
# Sim scenario each test starts from (sim source only; the UI can override).
SIM_SCENARIO = {"push_ball": "attack_solo", "goalie": "shot_on_goal", "match": "kickoff"}
ROBOT_OPS = ["status", "start", "stop", "bridge_start", "bridge_stop", "imu_test", "stage"]

# ---------------------------------------------------------------------------------------
# Brain / Sim: the real ones (lane A) if importable, else tiny stand-ins so the app runs.
# ---------------------------------------------------------------------------------------
def _go_to(pose, target, params, yaw_goal=None):
    """Turn toward the target, walk, stop. Returns (Cmd, done)."""
    dx, dy = target[0] - pose.x, target[1] - pose.y
    dist = math.hypot(dx, dy)
    kp_l, kp_y = params.get("goto_kp_lin", 1.0), params.get("goto_kp_yaw", 2.0)
    mvx, mwz = params.get("max_vx", 0.3), params.get("max_wz", 1.0)
    clip = lambda v, m: max(-m, min(m, v))  # noqa: E731
    if dist < params.get("goto_tol_m", 0.08):
        if yaw_goal is None:
            return Cmd.stop("goto:there"), True
        e = wrap(yaw_goal - pose.yaw)
        if abs(e) < 0.1:
            return Cmd.stop("goto:there"), True
        return Cmd(0.0, 0.0, clip(kp_y * e, mwz), "goto:turn"), False
    e = wrap(math.atan2(dy, dx) - pose.yaw)
    vx = 0.0 if abs(e) > 0.6 else clip(kp_l * dist, mvx) * math.cos(e)
    return Cmd(vx, 0.0, clip(kp_y * e, mwz), "goto:walk"), False


class _StubBrain:
    """Stand-in until fc/brain.py exists: attack = get behind the ball and walk through it."""

    def __init__(self, cfg: RobotCfg, field: Field):
        self.cfg, self.field = cfg, field

    def decide(self, world: World, params: dict):
        me = world.robots.get(self.cfg.id)
        if me is None or world.ball is None:
            return Cmd.stop("blind"), {"role": "idle", "phase": "blind", "target": None}
        role = params.get("role_mode", "auto")
        b, p = world.ball, me.pose
        if role == "auto":
            others = [r for r in world.robots.values() if r.id != self.cfg.id]
            d = lambda r: math.hypot(r.pose.x - b.x, r.pose.y - b.y)  # noqa: E731
            role = "attack" if not others or d(me) <= min(d(o) for o in others) else "defend"
        if role == "attack":
            gx, gy = self.field.goal_center(self.cfg.team, own=False)
            ux, uy = gx - b.x, gy - b.y
            n = math.hypot(ux, uy) or 1.0
            ux, uy = ux / n, uy / n
            back = self.field.robot_radius + self.field.ball_radius + 0.05
            behind = (b.x - ux * back, b.y - uy * back)
            aligned = math.hypot(p.x - behind[0], p.y - behind[1]) < 0.12
            phase, target = ("push", (b.x + ux * 0.4, b.y + uy * 0.4)) if aligned else ("approach", behind)
        elif role == "defend":
            gx, gy = self.field.goal_center(self.cfg.team, own=True)
            ux, uy = b.x - gx, b.y - gy
            n = math.hypot(ux, uy) or 1.0
            phase, target = "guard", (gx + ux / n * 0.35, gy + uy / n * 0.35)
        else:
            return Cmd.stop("idle"), {"role": "idle", "phase": "idle", "target": None}
        p2 = dict(params, goto_tol_m=0.03)
        cmd, _ = _go_to(p, target, p2)
        return Cmd(cmd.vx, cmd.vy, cmd.wz, f"stub:{phase}"), {"role": role, "phase": phase, "target": list(target)}


class _StubSim:
    """Stand-in until fc/sim.py exists: exact holonomic kinematics, ball pushed on contact."""

    def __init__(self, field: Field, robots: list[RobotCfg], params: dict, seed: int = 0):
        from .core import Ball, Pose
        self.field, self.robots, self.params, self.t = field, robots, params, 0.0
        self._Pose, self._Ball = Pose, Ball
        self.events: list[dict] = []
        self.reset()

    def reset(self, scenario: str = "kickoff"):
        L = self.field.length
        self.pose = {r.id: [(-0.6 if r.team == "blue" else 0.6) * L / 2, 0.0, 0.0 if r.team == "blue" else math.pi]
                     for r in self.robots}
        self.ball = [0.0, 0.0, 0.0, 0.0]

    def step(self, cmds: dict, dt: float):
        self.t += dt
        f = self.field
        for rid, c in cmds.items():
            if rid not in self.pose:
                continue
            x, y, yaw = self.pose[rid]
            x += (c.vx * math.cos(yaw) - c.vy * math.sin(yaw)) * dt
            y += (c.vx * math.sin(yaw) + c.vy * math.cos(yaw)) * dt
            self.pose[rid] = [x, y, wrap(yaw + c.wz * dt)]
            dx, dy = self.ball[0] - x, self.ball[1] - y
            d, reach = math.hypot(dx, dy), f.robot_radius + f.ball_radius
            if 1e-6 < d < reach:
                self.ball[0], self.ball[1] = x + dx / d * reach, y + dy / d * reach
                sp = max(0.15, abs(c.vx))
                self.ball[2], self.ball[3] = dx / d * sp * 1.3, dy / d * sp * 1.3
        b = self.ball
        b[0] += b[2] * dt
        b[1] += b[3] * dt
        b[2] *= 0.96
        b[3] *= 0.96
        if abs(b[1]) > f.width / 2:
            b[1], b[3] = math.copysign(f.width / 2, b[1]), -b[3]
        if abs(b[0]) > f.length / 2:
            if abs(b[1]) < f.goal_width / 2:
                self.events.append({"type": "goal", "team": "blue" if b[0] > 0 else "red", "t": self.t})
                self.reset()
            else:
                b[0], b[2] = math.copysign(f.length / 2, b[0]), -b[2]

    def observe(self, now: float) -> World:
        from .core import RobotObs
        return World(now, self._Ball(self.ball[0], self.ball[1], self.ball[2], self.ball[3], now),
                     {rid: RobotObs(rid, self._Pose(*p), now) for rid, p in self.pose.items()})

    def truth(self) -> dict:
        return {"ball": {"x": self.ball[0], "y": self.ball[1]},
                "robots": {rid: {"x": p[0], "y": p[1], "yaw": p[2]} for rid, p in self.pose.items()}}


def load_brain_sim():
    """(Brain, Sim, note). Imported lazily so lane A's files are picked up whenever they land."""
    note = []
    try:
        from .brain import Brain
    except Exception as e:  # noqa: BLE001 - any failure falls back, visibly
        Brain = _StubBrain
        note.append(f"brain: STUB ({type(e).__name__}: {e})")
    try:
        from .sim import Sim
    except Exception as e:  # noqa: BLE001
        Sim = _StubSim
        note.append(f"sim: STUB ({type(e).__name__}: {e})")
    return Brain, Sim, "; ".join(note) or "brain+sim: real"


# ---------------------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------------------
def _xyyaw(o):
    """Pose-like thing (dataclass, dict, list) -> dict x/y[/yaw] or None."""
    if o is None:
        return None
    if hasattr(o, "pose"):
        o = o.pose
    if is_dataclass(o):
        o = asdict(o)
    if isinstance(o, dict):
        if "pose" in o and isinstance(o["pose"], dict):
            o = o["pose"]
        out = {k: float(o[k]) for k in ("x", "y", "yaw", "vx", "vy") if isinstance(o.get(k), (int, float))}
        return out if "x" in out else None
    if isinstance(o, (list, tuple)) and len(o) >= 2:
        return dict(zip(("x", "y", "yaw"), map(float, o[:3])))
    return None


def world_json(w: World | None, now: float) -> dict | None:
    if w is None:
        return None
    ball = None
    if w.ball is not None:
        ball = {"x": w.ball.x, "y": w.ball.y, "vx": w.ball.vx, "vy": w.ball.vy, "age": now - w.ball.t}
    robots = {rid: {"x": r.pose.x, "y": r.pose.y, "yaw": r.pose.yaw, "age": now - r.t} for rid, r in w.robots.items()}
    return {"t": w.t, "ball": ball, "robots": robots}


def truth_json(tr) -> dict | None:
    if not tr:
        return None
    robots = tr.get("robots", {}) if isinstance(tr, dict) else {}
    return {"ball": _xyyaw(tr.get("ball")), "robots": {k: _xyyaw(v) for k, v in robots.items()}}


def clean(o):
    """JSON-safe copy (brain debug may hold tuples, numpy numbers, dataclasses)."""
    if isinstance(o, float):
        return round(o, 4) if math.isfinite(o) else None
    if isinstance(o, (str, int, bool)) or o is None:
        return o
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if is_dataclass(o):
        return clean(asdict(o))
    try:
        return clean(float(o))
    except (TypeError, ValueError):
        return str(o)


def validate_params(current: dict, changes: dict) -> tuple[dict, list[str]]:
    """Accepted subset of changes + list of errors. Only known keys; types and limits enforced."""
    ok, errors = {}, []
    for k, v in changes.items():
        if k not in current:
            errors.append(f"{k}: unknown parameter")
            continue
        old = current[k]
        if k in CHOICES:
            if v not in CHOICES[k]:
                errors.append(f"{k}: must be one of {CHOICES[k]}")
                continue
        elif isinstance(old, bool):
            if not isinstance(v, bool):
                errors.append(f"{k}: must be true/false")
                continue
        elif isinstance(old, (int, float)):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                errors.append(f"{k}: must be a number")
                continue
            lo, hi = LIMITS.get(k, (-1e9, 1e9))
            if not lo <= v <= hi:
                errors.append(f"{k}: {v} outside [{lo}, {hi}]")
                continue
            v = int(v) if isinstance(old, int) and float(v).is_integer() else float(v)
        elif isinstance(old, str):
            if not isinstance(v, str):
                errors.append(f"{k}: must be text")
                continue
        ok[k] = v
    return ok, errors


# ---------------------------------------------------------------------------------------
# the app
# ---------------------------------------------------------------------------------------
class App:
    def __init__(self, source: str = "sim", robot_ids: list[str] | None = None, video: str | None = None,
                 camera: str = "phone", dry_run_ops: bool = False, seed: int = 0, no_link: bool = False,
                 runs_root: Path = RUNS, presets_dir: Path = PRESETS):
        self.source, self.dry_run_ops = source, dry_run_ops
        self.runs_root, self.presets_dir = runs_root, presets_dir
        self.field = load_field()
        self.all_robots = load_robots()
        ids = robot_ids or [r.id for r in self.all_robots]
        unknown = set(ids) - {r.id for r in self.all_robots}
        if unknown:
            raise SystemExit(f"unknown robots {sorted(unknown)}; robots.json has {[r.id for r in self.all_robots]}")
        self.active = [r for r in self.all_robots if r.id in ids]
        self.cfg = {r.id: r for r in self.all_robots}
        Brain, Sim, self.impl_note = load_brain_sim()
        # Every knob in one editable dict: app defaults < brain defaults < params.json.
        brain_defaults = getattr(sys.modules.get(Brain.__module__), "DEFAULTS", {})
        sim_defaults = getattr(sys.modules.get(Sim.__module__), "SIM_DEFAULTS", {}) if source == "sim" else {}
        self.params = {**APP_DEFAULTS, **brain_defaults, **sim_defaults, **load_params()}
        self.brains = {r.id: Brain(r, self.field) for r in self.active}
        self.sim = self.eye = self.link = None
        if source == "sim":
            self.sim = Sim(self.field, self.all_robots, self.params, seed=seed)
            self.sim.reset("kickoff")
        else:
            from .eye import Eye  # needs OpenCV (.venv-fc)
            self.eye = Eye(self.field, self.all_robots, self.params, source=video or camera)
            self.eye.start()
            self.link = None if no_link else Link(self.active, self.params)
        self.lock = threading.RLock()
        self.mode = "idle"
        self.test: dict | None = None
        self.run: RunLog | None = None
        self.last_summary: dict | None = None
        self.teleop: dict[str, tuple] = {}
        self.events: deque = deque(maxlen=60)
        self.ops: dict[str, dict] = {}
        self.robot_status: dict[str, dict] = {}
        self.score = {"blue": 0, "red": 0}
        self.world: World | None = None
        self.cmds: dict[str, Cmd] = {}
        self.debug: dict[str, dict] = {}
        self._sim_ev_n = 0
        self._goal_cooldown = 0.0
        self._err_t: dict[str, float] = {}
        self.loop_stats = {"hz": 0.0, "overruns": 0, "tick_ms": 0.0}
        self._stop = False
        self.say(f"started: source={source}, robots={[r.id for r in self.active]}, {self.impl_note}"
                 + (", LINK OFF (nothing sent)" if source != "sim" and self.link is None else ""))

    # ---- small utilities -----------------------------------------------------
    def say(self, text: str, **extra) -> None:
        self.events.append({"t": time.strftime("%H:%M:%S"), "msg": text})
        if self.run:
            self.run.event({"ev": text, **extra})

    def presets(self) -> list[str]:
        return sorted(p.stem for p in self.presets_dir.glob("*.json")) if self.presets_dir.exists() else []

    # ---- loop ------------------------------------------------------------------
    def start(self) -> None:
        threading.Thread(target=self._loop, daemon=True, name="fc-loop").start()

    def _loop(self) -> None:
        nxt, last = time.monotonic(), time.monotonic()
        while not self._stop:
            dt = 1.0 / float(self.params.get("loop_hz", 30))
            t0 = time.monotonic()
            try:
                self.tick(dt)
            except Exception as e:  # noqa: BLE001 - the loop must survive; stop the robots
                self._throttled("loop", f"loop error: {type(e).__name__}: {e}")
                if self.link:
                    self.link.stop_all()
            t1 = time.monotonic()
            self.loop_stats["tick_ms"] = round((t1 - t0) * 1000, 2)
            self.loop_stats["hz"] = round(0.9 * self.loop_stats["hz"] + 0.1 / max(t1 - last, 1e-6), 1)
            last = t1
            nxt += dt
            if nxt < t1:
                self.loop_stats["overruns"] += 1
                nxt = t1
            time.sleep(max(0.0, nxt - time.monotonic()))

    def _throttled(self, key: str, msg: str, every: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._err_t.get(key, 0) > every:
            self._err_t[key] = now
            self.say(msg)

    def tick(self, dt: float) -> None:
        with self.lock:
            now = time.monotonic()
            world = self.sim.observe(now) if self.sim else self.eye.latest()
            self.world = world
            cmds, debug = {}, {}
            for r in self.active:
                cmd, dbg = self._decide(r, world, now)
                cmds[r.id], debug[r.id] = cmd, dbg
            if self.sim:
                self.sim.step({r.id: cmds.get(r.id, Cmd.stop()) for r in self.all_robots}, dt)
            elif self.link:
                self.link.send(cmds)
            self.cmds, self.debug = cmds, debug
            self._check_test(world, now)
            if self.run:
                rec = {"w": world_json(world, now),
                       "c": {k: [c.vx, c.vy, c.wz, c.tag] for k, c in cmds.items()},
                       "d": {k: {kk: d.get(kk) for kk in ("role", "phase", "target")} for k, d in debug.items()}}
                if self.sim:
                    rec["tr"] = truth_json(self.sim.truth())
                if self.link:
                    acks = self.link.drain_acks()
                    if acks:
                        rec["a"] = acks
                self.run.event(rec)

    def _decide(self, r: RobotCfg, world: World | None, now: float) -> tuple[Cmd, dict]:
        mode, test = self.mode, self.test or {}
        idle = (Cmd.stop("idle"), {"role": "idle", "phase": mode, "target": None})
        if mode == "idle" or mode == "sim_eval":
            return idle
        if mode == "drive":
            if r.id != test.get("robot"):
                return idle
            vx, vy, wz, t = self.teleop.get(r.id, (0, 0, 0, 0))
            if now - t > self.params["teleop_timeout_s"]:
                return Cmd.stop("teleop:timeout"), {"role": "drive", "phase": "keys idle", "target": None}
            p = self.params
            return (Cmd(vx * p["max_vx"], vy * p["max_vy"], wz * p["max_wz"], "teleop"),
                    {"role": "drive", "phase": "keys", "target": None})
        me = world.robots.get(r.id) if world else None
        if me is None or now - me.t > self.params["stale_stop_s"]:
            return Cmd.stop("stale"), {"role": "idle", "phase": "not seen", "target": None}
        if mode == "goto":
            if r.id != test.get("robot") or test.get("target") is None:
                return idle
            tgt = test["target"]
            cmd, done = _go_to(me.pose, tgt[:2], self.params, tgt[2] if len(tgt) > 2 else None)
            test["done_since"] = (test.get("done_since") or now) if done else None
            return cmd, {"role": "goto", "phase": cmd.tag, "target": list(tgt[:2])}
        # brain-driven modes
        params = self.params
        if mode == "push_ball":
            if r.id != test.get("robot"):
                return idle
            params = {**params, "role_mode": "attack"}
        elif mode == "goalie":
            if r.id != test.get("robot"):
                return idle
            params = {**params, "role_mode": "defend"}
        elif mode == "match":
            params = {**params, "role_mode": "auto"}
        try:
            cmd, dbg = self.brains[r.id].decide(world, params)
        except Exception as e:  # noqa: BLE001 - a brain bug must not move the robot
            self._throttled(f"brain-{r.id}", f"brain error {r.id}: {type(e).__name__}: {e}")
            return Cmd.stop("brain_error"), {"role": "idle", "phase": "brain error", "target": None}
        return cmd, clean(dbg or {})

    # ---- test bookkeeping --------------------------------------------------------
    def _goal_events(self, world: World | None, now: float) -> list[str]:
        """Teams that scored since last tick."""
        if self.sim is not None and hasattr(self.sim, "events"):
            new = self.sim.events[self._sim_ev_n:]
            self._sim_ev_n = len(self.sim.events)
            out = []
            for ev in new:
                if not isinstance(ev, dict) or "goal" not in str(ev.get("type", ev.get("kind", ""))):
                    continue
                team = ev.get("team") or ev.get("scorer")
                if team not in ("blue", "red"):
                    x = ev.get("x", ev.get("ball_x"))
                    side = ev.get("goal") or ev.get("side")
                    team = "blue" if (side == "east" or (isinstance(x, (int, float)) and x > 0)) else "red"
                out.append(team)
            return out
        b = world.ball if world else None
        if b is None or now < self._goal_cooldown:
            return []
        f = self.field
        if abs(b.x) > f.length / 2 + f.ball_radius and abs(b.y) < f.goal_width / 2:
            self._goal_cooldown = now + 3.0
            return ["blue" if b.x > 0 else "red"]
        return []

    def _check_test(self, world: World | None, now: float) -> None:
        goals = self._goal_events(world, now)
        for team in goals:
            self.score[team] += 1
            self.say(f"GOAL {team}  ({self.score['blue']}-{self.score['red']})", team=team)
        t = self.test
        if not t:
            return
        el = now - t["t0"]
        name = t["name"]
        if name == "push_ball":
            team = self.cfg[t["robot"]].team
            if goals:
                res = "goal" if team in goals else "own_goal"
                self._finish(res, time_s=round(el, 2), ball=world_json(world, now)["ball"] if world else None)
            elif el > self.params["test_timeout_s"]:
                self._finish("timeout", time_s=round(el, 2))
        elif name == "goalie":
            team = self.cfg[t["robot"]].team
            t.setdefault("conceded", 0)
            t.setdefault("saved", 0)
            t.setdefault("shot_t0", t["t0"])
            conceded = sum(1 for g in goals if g != team)
            t["conceded"] += conceded
            if self.sim is not None and not isinstance(self.sim, _StubSim):
                # sim: a fresh shot every goalie_shot_s; no goal by then = saved
                if conceded or now - t["shot_t0"] > self.params["goalie_shot_s"]:
                    t["saved"] += 0 if conceded else 1
                    t["shot_t0"] = now
                    self._reset_sim("shot_on_goal", [t["robot"]])
            if el > self.params["test_timeout_s"]:
                shots = t["conceded"] + t["saved"] if self.sim else None
                self._finish("done", conceded=t["conceded"], saved=t["saved"] if self.sim else None,
                             shots=shots, time_s=round(el, 2))
        elif name == "match":
            if el > self.params["match_s"]:
                self._finish("full_time", score={k: self.score[k] - t["score0"][k] for k in self.score},
                             time_s=round(el, 1))
        elif name == "goto":
            me = world.robots.get(t["robot"]) if world else None
            if me is not None:
                t["last_pose"] = [me.pose.x, me.pose.y, me.pose.yaw]
                if t.get("start") is None:  # e.g. the sim was just reset
                    t["start"] = t["last_pose"][:2]
            if t.get("done_since") and now - t["done_since"] > 0.5:
                self._finish("reached", **self._goto_report(t, el))
            elif el > self.params["test_timeout_s"]:
                self._finish("timeout", **self._goto_report(t, el))

    def _goto_report(self, t: dict, el: float) -> dict:
        p, tgt = t.get("last_pose"), t["target"]
        if p is None:
            return {"time_s": round(el, 2), "pos_err_m": None}
        want_yaw = tgt[2] if len(tgt) > 2 and tgt[2] is not None else math.atan2(tgt[1] - t["start"][1], tgt[0] - t["start"][0]) if t.get("start") else None
        return {"time_s": round(el, 2), "target": tgt, "final": p,
                "pos_err_m": round(math.hypot(p[0] - tgt[0], p[1] - tgt[1]), 3),
                "heading_err_deg": None if want_yaw is None else round(math.degrees(wrap(p[2] - want_yaw)), 1)}

    def _reset_sim(self, scenario: str, robots: list[str] | None = None) -> None:
        try:
            if robots and not isinstance(self.sim, _StubSim):
                self.sim.reset(scenario, robots=robots)
            else:
                self.sim.reset(scenario)
        except Exception as e:  # noqa: BLE001
            self.say(f"sim reset {scenario!r} failed ({e}); using kickoff")
            self.sim.reset("kickoff")
        self._sim_ev_n = len(getattr(self.sim, "events", []))

    def _begin(self, name: str, robot: str | None, **extra) -> dict:
        """Start a test/mode with its own run log. Caller holds the lock."""
        if self.test:
            self._finish("replaced")
        self.test = {"name": name, "robot": robot, "t0": time.monotonic(), **extra}
        self.mode = name
        for b in self.brains.values():  # fresh role/latch memory for every run
            if hasattr(b, "reset"):
                b.reset()
        meta = {"test": name, "robot": robot, "source": self.source, "impl": self.impl_note,
                "active": [r.id for r in self.active], "extra": clean(extra)}
        self.run = RunLog(name, dict(self.params), meta, root=self.runs_root)
        self.test["run_dir"] = str(self.run.dir)
        self.say(f"start {name}" + (f" ({robot})" if robot else "") + f" -> {self.run.dir.name}")
        return self.test

    def _finish(self, result: str, **summary) -> None:
        """End the current test; summary.json written on a side thread. Caller holds the lock."""
        t, run = self.test, self.run
        self.test, self.run, self.mode = None, None, "idle"
        self.teleop.clear()
        if self.link:
            self.link.stop_all()
        if not t:
            return
        summary = clean({"result": result, "robot": t.get("robot"), **summary})
        self.last_summary = {"run": run.dir.name if run else None, **summary}
        self.events.append({"t": time.strftime("%H:%M:%S"), "msg": f"end {t['name']}: {result}"})
        if run:
            run.event({"ev": f"end {result}"})
            threading.Thread(target=run.finish, args=(summary,), daemon=True).start()

    # ---- commands from the UI ------------------------------------------------------
    def action(self, body: dict) -> dict:
        a = body.get("action")
        if a == "stop":
            return self.stop()
        with self.lock:
            if a == "go":
                self._begin("play", None)
                return {"ok": True}
            if a == "teleop":
                rid = body.get("robot")
                if self.mode != "drive" or rid != (self.test or {}).get("robot"):
                    return {"ok": False, "error": "not in drive mode for that robot"}
                c = lambda k: max(-1.0, min(1.0, float(body.get(k, 0) or 0)))  # noqa: E731
                self.teleop[rid] = (c("vx"), c("vy"), c("wz"), time.monotonic())
                return {"ok": True}
            if a == "reset":
                if not self.sim:
                    return {"ok": False, "error": "reset is sim only"}
                self._reset_sim(body.get("scenario") or "kickoff")
                return {"ok": True}
            if a == "score_reset":
                self.score = {"blue": 0, "red": 0}
                return {"ok": True}
            if a == "test":
                return self._start_test(body)
        return {"ok": False, "error": f"unknown action {a!r}"}

    def _start_test(self, body: dict) -> dict:
        name = body.get("test")
        if name not in TESTS:
            return {"ok": False, "error": f"unknown test {name!r}; one of {TESTS}"}
        active = [r.id for r in self.active]
        robot = body.get("robot") or active[0]
        if name != "sim_eval" and name != "match" and robot not in active:
            return {"ok": False, "error": f"robot {robot!r} not active ({active})"}
        if self.sim:
            scen = body.get("scenario") or SIM_SCENARIO.get(name)
            in_sim = set((truth_json(self.sim.truth()) or {}).get("robots", {}))
            if scen:
                self._reset_sim(scen, [robot] if name in ("push_ball", "goalie") else None)
            elif name in ("drive", "goto") and robot not in in_sim:
                self._reset_sim("kickoff")  # bring every robot back onto the field
        if name == "goto":
            tgt = body.get("target")
            if not (isinstance(tgt, list) and len(tgt) >= 2 and all(isinstance(v, (int, float)) for v in tgt[:2])):
                return {"ok": False, "error": "goto needs target [x, y] or [x, y, yaw]"}
            me = self.world.robots.get(robot) if self.world else None
            start = [me.pose.x, me.pose.y] if me else None
            self._begin("goto", robot, target=[float(v) for v in tgt[:3] if v is not None], start=start)
        elif name == "match":
            self._begin("match", None, score0=dict(self.score))
        elif name == "sim_eval":
            self._begin("sim_eval", None)
            threading.Thread(target=self._sim_eval, args=(self.test, str(body.get("args", ""))), daemon=True).start()
        else:
            self._begin(name, robot)
        return {"ok": True, "run": self.test["run_dir"] if self.test else None}

    def _sim_eval(self, test: dict, args: str) -> None:
        cmd = [sys.executable, "-m", "fc.sim_eval", *shlex.split(args)]
        test["output"] = f"$ {' '.join(cmd)}\n"
        try:
            p = subprocess.Popen(cmd, cwd=FINAL_PROJECT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            test["pid"] = p.pid
            for line in p.stdout:
                test["output"] += line
                if self.test is not test:  # stopped from the UI
                    p.kill()
                    break
            rc = p.wait()
        except OSError as e:
            test["output"] += f"failed: {e}\n"
            rc = -1
        Path(test["run_dir"], "sim_eval.txt").write_text(test["output"])
        with self.lock:
            if self.test is test:
                self._finish("ok" if rc == 0 else f"exit {rc}", returncode=rc, output_tail=test["output"][-4000:])

    def stop(self) -> dict:
        """Immediate: zero bursts first, then bookkeeping."""
        if self.link:
            self.link.stop_all()
        with self.lock:
            if self.test:
                self._finish("stopped")
            self.mode = "idle"
            self.teleop.clear()
        return {"ok": True}

    def set_params(self, changes: dict) -> dict:
        with self.lock:
            ok, errors = validate_params(self.params, changes)
            self.params.update(ok)  # in place: Link/Eye hold this same dict
            if ok and self.sim is not None and hasattr(self.sim, "set_params"):
                self.sim.set_params(self.params)  # the sim copies its params
            if ok:
                self.say("params: " + ", ".join(f"{k}={v}" for k, v in ok.items()), params=ok)
        return {"ok": not errors, "applied": ok, "errors": errors}

    def preset(self, body: dict) -> dict:
        name = str(body.get("name", ""))
        if not re.fullmatch(r"[A-Za-z0-9_\-]{1,40}", name):
            return {"ok": False, "error": "preset name: letters, digits, - and _ only"}
        path = self.presets_dir / f"{name}.json"
        if body.get("op") == "save":
            self.presets_dir.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(self.params, indent=1, sort_keys=True))
            self.say(f"preset saved: {name}")
            return {"ok": True}
        if body.get("op") == "load":
            if not path.exists():
                return {"ok": False, "error": f"no preset {name}"}
            res = self.set_params(json.loads(path.read_text()))
            self.say(f"preset loaded: {name}" + (f" ({len(res['errors'])} keys rejected)" if res["errors"] else ""))
            return {"ok": True, "applied": len(res["applied"]), "errors": res["errors"]}
        return {"ok": False, "error": "op must be save or load"}

    # ---- robot ops over ssh ----------------------------------------------------------
    def robot_op(self, body: dict) -> dict:
        rid, op = body.get("robot"), body.get("op")
        r = self.cfg.get(rid)
        if r is None or op not in ROBOT_OPS:
            return {"ok": False, "error": f"need robot in {list(self.cfg)} and op in {ROBOT_OPS}"}
        if not r.host:
            return {"ok": False, "error": f"{rid} has no host in robots.json"}
        if op == "start" and body.get("confirm") is not True:
            return {"ok": False, "error": "start homes the legs: send confirm=true"}
        cur = self.ops.get(rid)
        if cur and cur.get("running"):
            return {"ok": False, "error": f"{cur['op']} still running on {rid}"}
        host = r.host
        ctl = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, "bash", "~/fc-v1/robot/fcctl.sh"]
        cmd = {"status": ctl + ["status"],
               "start": ctl + ["start", str(body.get("profile") or r.profile), "--confirm"],
               "stop": ctl + ["stop"],
               "bridge_start": ctl + ["bridge", "start"], "bridge_stop": ctl + ["bridge", "stop"],
               "imu_test": ctl + ["imu-test", str(int(body.get("seconds", 5)))],
               "stage": ["bash", str(HERE / "robot" / "stage.sh"), host]}[op]
        if self.dry_run_ops:
            cmd = ["echo", "DRY-RUN:", *cmd]
        state = {"op": op, "cmd": " ".join(cmd), "running": True, "rc": None, "out": "", "t": time.strftime("%H:%M:%S")}
        self.ops[rid] = state
        self.say(f"robot {rid}: {op}" + (" (dry run)" if self.dry_run_ops else ""))
        threading.Thread(target=self._run_op, args=(rid, state, cmd, 120 if op == "stage" else 60), daemon=True).start()
        return {"ok": True, "cmd": state["cmd"]}

    def _run_op(self, rid: str, state: dict, cmd: list[str], timeout: float) -> None:
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=FINAL_PROJECT)
            state["out"], state["rc"] = (p.stdout + p.stderr)[-4000:], p.returncode
        except subprocess.TimeoutExpired as e:
            state["out"], state["rc"] = f"timeout after {timeout}s\n{(e.stdout or '')}"[-4000:], -1
        except OSError as e:
            state["out"], state["rc"] = str(e), -1
        state["running"] = False
        if state["op"] == "status":
            for line in reversed(state["out"].strip().splitlines()):
                try:
                    self.robot_status[rid] = json.loads(line)
                    break
                except ValueError:
                    continue
        self.say(f"robot {rid}: {state['op']} -> rc {state['rc']}")

    # ---- state for the UI ----------------------------------------------------------------
    def state(self) -> dict:
        with self.lock:
            now = time.monotonic()
            test = None
            if self.test:
                test = {k: v for k, v in self.test.items() if k not in ("t0", "output")}
                test["elapsed"] = round(now - self.test["t0"], 1)
                if "output" in self.test:
                    test["output"] = self.test["output"][-6000:]
            return clean({
                "t": now, "source": self.source, "mode": self.mode, "test": test, "impl": self.impl_note,
                "run_dir": self.run.dir.name if self.run else None, "last_summary": self.last_summary,
                "field": asdict(self.field), "robots": [asdict(r) for r in self.all_robots],
                "active": [r.id for r in self.active], "world": world_json(self.world, now),
                "truth": truth_json(self.sim.truth()) if self.sim else None,
                "cmds": {k: {"vx": c.vx, "vy": c.vy, "wz": c.wz, "tag": c.tag} for k, c in self.cmds.items()},
                "debug": self.debug, "link": self.link.status() if self.link else None,
                "eye": self.eye.status() if self.eye else None, "params": self.params,
                "choices": CHOICES, "limits": LIMITS, "tests": TESTS, "presets": self.presets(),
                "ops": self.ops, "robot_status": self.robot_status, "dry_run_ops": self.dry_run_ops,
                "events": list(self.events)[-30:], "score": self.score, "loop": self.loop_stats,
            })

    def close(self) -> None:
        self.stop()
        self._stop = True
        if self.link:
            self.link.stop_all()
            self.link.close()
        if self.eye:
            self.eye.close()


# ---------------------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------------------
def make_handler(app: App):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):  # quiet
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode(), "application/json")

        def do_GET(self):  # noqa: N802
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                return self._send(200, (UI / "index.html").read_bytes(), "text/html; charset=utf-8")
            if path == "/state":
                return self._json(app.state())
            if path == "/runs":
                return self._json(list_runs(app.runs_root))
            if path == "/frame.jpg":
                jpg = app.eye.jpeg() if app.eye else None
                return self._send(200, jpg, "image/jpeg") if jpg else self._send(404, b"no frame", "text/plain")
            m = re.fullmatch(r"/runs/([\w\-]+)/(summary\.json|params\.json|sim_eval\.txt)", path)
            if m and (app.runs_root / m[1] / m[2]).exists():
                return self._send(200, (app.runs_root / m[1] / m[2]).read_bytes(), "text/plain; charset=utf-8")
            self._send(404, b"not found", "text/plain")

        def do_POST(self):  # noqa: N802
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(body, dict):
                    raise ValueError("body must be a JSON object")
            except ValueError as e:
                return self._json({"ok": False, "error": f"bad JSON: {e}"}, 400)
            route = {"/action": app.action, "/params": app.set_params, "/robot": app.robot_op,
                     "/preset": app.preset}.get(self.path)
            if route is None:
                return self._json({"ok": False, "error": "not found"}, 404)
            res = route(body)
            self._json(res, 200 if res.get("ok") else 400)

    return H


def serve(app: App, host: str, port: int) -> ThreadingHTTPServer:
    srv = ThreadingHTTPServer((host, port), make_handler(app))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True, name="fc-http").start()
    return srv


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["sim", "camera"], default="sim")
    ap.add_argument("--robots", default=None, help="comma list of robot ids to control (default: all in robots.json)")
    ap.add_argument("--port", type=int, default=8800)
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to reach the UI from another device")
    ap.add_argument("--video", default=None, help="camera source: video file, image folder, or 'synthetic'")
    ap.add_argument("--camera", default="phone", help="live camera name or index (fieldcam capture)")
    ap.add_argument("--dry-run-ops", action="store_true", help="robot ops print their ssh command instead of running it")
    ap.add_argument("--no-link", action="store_true", help="camera source: compute commands but send nothing")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(argv)
    app = App(a.source, a.robots.split(",") if a.robots else None, video=a.video, camera=a.camera,
              dry_run_ops=a.dry_run_ops, seed=a.seed, no_link=a.no_link)
    app.start()
    srv = serve(app, a.host, a.port)
    print(f"Pupper FC v1: http://{a.host}:{a.port}/   ({app.impl_note})   Ctrl-C to quit", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        app.close()
        srv.shutdown()


if __name__ == "__main__":
    main()
