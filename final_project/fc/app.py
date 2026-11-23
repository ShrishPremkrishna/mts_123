"""Pupper FC v1 laptop app: one 30 Hz loop + a stdlib HTTP UI.

    cd final_project && python -m fc.app --source sim
    python -m fc.app --source camera [--video FILE|DIR|synthetic] [--robots blue] [--port 8800]

World (Sim or Eye) -> Brain.decide per active robot -> Cmd -> brain.shape -> Sim.step or Link.send
-> run log. Every test is a named mode with its own run folder (see runlog.py). Stop is always
immediate. Core (this file, link.py, runlog.py) is stdlib only; the camera source needs OpenCV.
A broken fc.brain / fc.sim import stops the app: there is no fallback controller.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import re
import shlex
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, fields, is_dataclass, replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .brain import Brain, shape
from .brain import DEFAULTS as BRAIN_DEFAULTS
from .core import CONFIG, Cmd, RobotCfg, World, load_field, load_params, wrap
from .link import Link
from .runlog import RUNS, Metrics, RunLog, list_runs
from .sim import Sim
from .sim import SIM_DEFAULTS

HERE = Path(__file__).parent
UI = HERE / "ui"
PRESETS = CONFIG / "presets"
FINAL_PROJECT = HERE.parent

# Fallbacks for knobs params.json should carry (proposed for merge 2026-11-23). params.json wins;
# delete entries here once they are in params.json.
PARAM_FALLBACKS = {
    "test_timeout_s": 45.0,     # push_ball / goalie / goto give up after this
    "match_s": 300.0,           # match length
    "goalie_shot_s": 5.0,       # sim goalie: a new shot this often (no goal by then = saved)
    "goto_tol_m": 0.08,         # goto: close enough (brain uses it too)
    "stale_stop_s": 0.5,        # robot observation older than this -> stop that robot (not in drive)
    "teleop_timeout_s": 0.4,    # drive: no key update for this long -> stop
    "step_s": 2.0,              # step: how long the open-loop command is held
    "step_settle_s": 1.5,       # step: zero command afterwards before the final pose is read
    "deadman_vx": 0.10,         # deadman: walking speed before the app goes silent
    "deadman_drive_s": 2.0,     # deadman: walk this long first
    "deadman_mute_s": 2.0,      # deadman: send nothing for this long (bridge ttl must stop the robot)
    "eye_floor_size": 0.15875,  # corner tag black square side, m (Tej, 2026-11-20)
    "eye_tag_height": 0.24,     # floor -> robot tag while standing, m (Tej, 2026-11-20)
    "eye_tag_offset_x": 0.065,  # robot tag centre ahead of body centre, m
    "eye_yaw_offset_deg": 0.0,  # add if the tag top does not point at the nose (Tej used -180)
    "eye_latency_s": 0.12,      # glass-to-python delay
}
# Hard limits for live edits. Speed ceilings mirror link.CEIL.
LIMITS = {
    "loop_hz": (5, 60), "max_vx": (0.0, 0.5), "max_vy": (0.0, 0.3), "max_wz": (0.0, 2.0),
    "accel_vx": (0.05, 5.0), "accel_vy": (0.05, 5.0), "accel_wz": (0.1, 20.0), "ttl_s": (0.05, 0.5),
    "test_timeout_s": (1, 1800), "goalie_shot_s": (1, 30), "match_s": (10, 3600), "goto_tol_m": (0.02, 1.0),
    "stale_stop_s": (0.1, 2.0), "teleop_timeout_s": (0.1, 2.0),
    "step_s": (0.2, 10.0), "step_settle_s": (0.5, 5.0),
    "deadman_vx": (0.03, 0.2), "deadman_drive_s": (0.5, 5.0), "deadman_mute_s": (0.5, 5.0),
    "eye_floor_size": (0.03, 0.5), "eye_tag_height": (0.0, 0.5),
    "eye_tag_offset_x": (-0.3, 0.3), "eye_yaw_offset_deg": (-360, 360), "eye_latency_s": (0.0, 1.0),
    "k_turn": (0.0, 6.0), "k_vy": (0.0, 5.0), "search_wz": (0.0, 2.0), "predict_self": (0, 1),
    "yaw_learn": (0, 1), "yaw_learn_rate": (0.0, 0.2), "act_lag_s": (0.0, 1.0),
}
LIMITS.update({k: (0.0, 0.5) for k in ("push_vx", "hb_charge_vx", "hb_backoff_vx")})
CHOICES = {"role_mode": ["auto", "attack", "defend"], "contact_mode": ["push", "headbutt"],
           "ball_lost": ["hold", "search"], "goalie_style": ["sideways", "facing"]}
TESTS = ["drive", "goto", "step", "deadman", "push_ball", "goalie", "match", "sim_eval"]
ONE_ROBOT_TESTS = {"drive", "goto", "step", "deadman", "push_ball", "goalie"}
BRAIN_MODES = {"play", "goto", "push_ball", "goalie", "match"}
# Sim scenario each test starts from (sim source only; the UI can override).
SIM_SCENARIO = {"push_ball": "attack_solo", "goalie": "shot_on_goal", "match": "kickoff"}
ROBOT_OPS = ["status", "start", "stop", "bridge_start", "bridge_stop", "imu_test", "stage"]
PROFILES_DIR = HERE / "robot" / "profiles"
HOST_RE = re.compile(r"[A-Za-z0-9_.@-]{1,253}")   # robots.json host: [user@]name, no shell or ssh-option chars
IMU_TEST_S = (1, 900)
IMU_MODES = ("auto", "bno", "ros")
IMU_CONDITION_RE = re.compile(r"[a-z0-9_]{1,24}")
OP_TIMEOUT_S = {"start": 180, "stage": 120}       # fcctl start waits up to 160 s for READY
STEP_PRE_S = 0.7                                   # step: still before the command (start pose)
DEADMAN_PRE_S = 0.5
PRESET_NAME_RE = re.compile(r"[A-Za-z0-9_\-]{1,40}")


def load_robot_config(path: Path = CONFIG / "robots.json") -> tuple[list[RobotCfg], dict[str, str]]:
    """robots.json -> (RobotCfg list, {id: udp_host}). Extra keys such as "udp_host" are allowed
    even while core.RobotCfg does not declare them."""
    raw = json.loads(path.read_text())
    names = {f.name for f in fields(RobotCfg)}
    robots = [RobotCfg(**{k: v for k, v in r.items() if k in names}) for r in raw]
    return robots, {str(r["id"]): str(r.get("udp_host") or "") for r in raw}


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
    if isinstance(o, (list, tuple, deque)):
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
# pipeline analysis (pure functions: obs = [(t_capture, x, y, yaw)], sent = [(t, vx, vy, wz)])
# ---------------------------------------------------------------------------------------
def _mean(v):
    return sum(v) / len(v) if v else None


def _circmean(a):
    return math.atan2(sum(math.sin(x) for x in a), sum(math.cos(x) for x in a))


def _unwrap(yaws):
    out = []
    for y in yaws:
        out.append(y if not out else out[-1] + wrap(y - out[-1]))
    return out


def _interp(t, ts, vs):
    if not ts:
        return 0.0
    if t <= ts[0]:
        return vs[0]
    if t >= ts[-1]:
        return vs[-1]
    i = bisect.bisect_right(ts, t)
    t0, t1 = ts[i - 1], ts[i]
    return vs[i - 1] + (vs[i] - vs[i - 1]) * (t - t0) / (t1 - t0) if t1 > t0 else vs[i]


def _cumulative(sent, rate):
    """Times and running integral of rate(cmd) for commands held from each send to the next."""
    ts, acc, out = [], 0.0, []
    for i, c in enumerate(sent):
        ts.append(c[0])
        out.append(acc)
        if i + 1 < len(sent):
            acc += rate(c) * (sent[i + 1][0] - c[0])
    return ts, out


def fit_lag(obs_t, obs_s, cmd_t, cmd_s, max_lag=1.0, step=0.01):
    """Delay tau and gain g that best fit obs_s(t) ~ g * cmd_s(t - tau) (least squares)."""
    best = None
    for k in range(int(max_lag / step) + 1):
        tau = k * step
        pred = [_interp(t - tau, cmd_t, cmd_s) for t in obs_t]
        pp = sum(p * p for p in pred)
        if pp < 1e-9:
            continue
        g = sum(p * o for p, o in zip(pred, obs_s)) / pp
        res = sum((o - g * p) ** 2 for p, o in zip(pred, obs_s))
        if best is None or res < best[0] - 1e-12:
            best = (res, tau, g)
    return None if best is None else (best[1], best[2])


def analyze_step(obs, sent, t_cmd0, t_cmd1, cmd):
    """Open-loop step: actual vs commanded travel, response delay, walk direction vs tag yaw."""
    out = {"cmd": [round(v, 3) for v in cmd], "obs": len(obs)}
    pre = [o for o in obs if o[0] < t_cmd0][-15:]
    if len(pre) < 2 or len(obs) - len(pre) < 5 or not sent:
        return {**out, "result": "no_data"}
    end = obs[-8:]
    x0, y0 = _mean([o[1] for o in pre]), _mean([o[2] for o in pre])
    x1, y1 = _mean([o[1] for o in end]), _mean([o[2] for o in end])
    un = _unwrap([o[3] for o in obs])
    n_pre = sum(1 for o in obs if o[0] < t_cmd0)
    yaw0 = _circmean([o[3] for o in pre])
    dyaw = _mean(un[-8:]) - _mean(un[max(0, n_pre - 15):n_pre])
    # commanded displacement: unicycle integration from the start heading
    cx = cy = th = 0.0
    for i, (t, vx, vy, wz) in enumerate(sent):
        dt = sent[i + 1][0] - t if i + 1 < len(sent) else 0.0
        c, s = math.cos(yaw0 + th), math.sin(yaw0 + th)
        cx, cy, th = cx + (c * vx - s * vy) * dt, cy + (s * vx + c * vy) * dt, th + wz * dt
    dx, dy = x1 - x0, y1 - y0
    dist, cmd_dist = math.hypot(dx, dy), math.hypot(cx, cy)
    out.update(start=[round(x0, 3), round(y0, 3), round(math.degrees(yaw0), 1)], end=[round(x1, 3), round(y1, 3)],
               cmd_dist_m=round(cmd_dist, 3), dist_m=round(dist, 3),
               cmd_yaw_deg=round(math.degrees(th), 1), yaw_deg=round(math.degrees(dyaw), 1))
    linear = cmd_dist > 0.03
    t_on = next((c[0] for c in sent if abs(c[1]) + abs(c[2]) + abs(c[3]) > 1e-6), t_cmd0)
    if linear:
        ux, uy = cx / cmd_dist, cy / cmd_dist
        during = [o[3] for o in obs if t_cmd0 <= o[0] <= t_cmd1 + 0.3] or [yaw0]
        expect = _circmean(during) + math.atan2(cmd[1], cmd[0])
        travel = math.atan2(dy, dx)
        out.update(gain_lin=round(dist / cmd_dist, 3),
                   heading_bias_deg=round(math.degrees(wrap(travel - expect)), 1) if dist > 0.05 else None,
                   lateral_m=round(ux * dy - uy * dx, 3),
                   yaw_drift_deg_per_m=round(math.degrees(dyaw) / dist, 1) if dist > 0.05 else None)
        sig = [(o[0], (o[1] - x0) * ux + (o[2] - y0) * uy) for o in obs if o[0] >= t_cmd0 - 0.3]
        rate = lambda c: math.hypot(c[1], c[2])  # noqa: E731
        thresh = 0.02
    else:
        sig = [(o[0], u - _mean(un[max(0, n_pre - 15):n_pre])) for o, u in zip(obs, un) if o[0] >= t_cmd0 - 0.3]
        sign = 1.0 if th >= 0 else -1.0
        sig = [(t, sign * s) for t, s in sig]
        rate = lambda c: sign * c[3]  # noqa: E731
        thresh = math.radians(4)
    if abs(th) > 0.15:
        out["gain_wz"] = round(dyaw / th, 3)
    ct, cs = _cumulative(sent, rate)
    fit = fit_lag([s[0] for s in sig], [s[1] for s in sig], ct, cs)
    out["latency_s"] = round(fit[0], 2) if fit else None
    onset = next((t for t, s in sig if t >= t_on and s > thresh), None)
    out["onset_s"] = round(onset - t_on, 2) if onset is not None else None
    moved = dist > 0.02 if linear else abs(dyaw) > math.radians(3)
    out["result"] = "done" if moved else "no_motion"
    bias = out.get("heading_bias_deg")
    out["pass"] = {"latency": out["latency_s"] is not None and out["latency_s"] < 0.35,
                   "bias": None if bias is None else abs(bias) < 5.0}
    return out


def _linfit(xs, ys):
    """Least-squares y = a + b x -> (a, b, residual sum of squares)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx > 1e-12 else 0.0
    a = my - b * mx
    return a, b, sum((y - a - b * x) ** 2 for x, y in zip(xs, ys))


def analyze_deadman(obs, t_mute, rest_speed=0.03, tail_s=0.3, lead_s=0.8):
    """Time from the last packet to the robot at rest, from overhead positions only.
    Progress along the walking direction is fitted as a + b*min(t, tau) (walk, then stand);
    the best tau over all frames is robust to centimetre-level camera noise."""
    out = {"obs": len(obs)}
    seg = [o for o in obs if t_mute is not None and o[0] >= t_mute - lead_s]
    before = [o for o in seg if o[0] <= t_mute]
    if t_mute is None or len(before) < 5 or len(seg) - len(before) < 5:
        return {**out, "result": "no_data"}
    _, vx, _ = _linfit([o[0] for o in before], [o[1] for o in before])
    _, vy, _ = _linfit([o[0] for o in before], [o[2] for o in before])
    v = math.hypot(vx, vy)
    out["speed_before"] = round(v, 3)
    if v < rest_speed:
        return {**out, "result": "never_moved", "stop_time_s": None, "pass": None}
    ux, uy = vx / v, vy / v
    ts = [o[0] for o in seg]
    ss = [(o[1] - seg[0][1]) * ux + (o[2] - seg[0][2]) * uy for o in seg]
    best = None
    for k in range(int((ts[-1] - t_mute) / 0.01) + 1):
        tau = t_mute + k * 0.01
        _a, b, res = _linfit([min(t, tau) for t in ts], ss)
        if best is None or res < best[0]:
            best = (res, tau, b)
    _res, tau, b = best
    out["coast_m"] = round(max(0.0, b) * (tau - t_mute), 3)
    if tau > ts[-1] - tail_s:
        return {**out, "result": "still_moving", "stop_time_s": None, "pass": False}
    st = round(tau - t_mute, 2)
    return {**out, "result": "stopped", "stop_time_s": st, "pass": st <= 0.5}


# ---------------------------------------------------------------------------------------
# the app
# ---------------------------------------------------------------------------------------
class App:
    def __init__(self, source: str = "sim", robot_ids: list[str] | None = None, video: str | None = None,
                 camera: str = "phone", dry_run_ops: bool = False, seed: int = 0, no_link: bool = False,
                 runs_root: Path = RUNS, presets_dir: Path = PRESETS, udp_hosts: dict[str, str] | None = None):
        self.source, self.dry_run_ops = source, dry_run_ops
        self.runs_root, self.presets_dir = runs_root, presets_dir
        self.field = load_field()
        self.all_robots, cfg_udp = load_robot_config()
        self.udp_hosts = {**cfg_udp, **(udp_hosts or {})}
        ids = robot_ids or [r.id for r in self.all_robots]
        unknown = set(ids) - {r.id for r in self.all_robots}
        if unknown:
            raise SystemExit(f"unknown robots {sorted(unknown)}; robots.json has {[r.id for r in self.all_robots]}")
        self.active = [r for r in self.all_robots if r.id in ids]
        self.cfg = {r.id: r for r in self.all_robots}
        self.impl_note = "brain+sim: real"
        # Every knob in one editable dict: fallbacks < brain defaults < sim defaults < params.json.
        sim_defaults = SIM_DEFAULTS if source == "sim" else {}
        self.params = {**PARAM_FALLBACKS, **BRAIN_DEFAULTS, **sim_defaults, **load_params()}
        self.base_params = dict(self.params)   # what "base" / preset loads start from
        self.preset_name: str | None = None
        self.preset_vals: dict = {}
        self.preset_profile: str | None = None
        self.preset_note = ""
        self.brains = {r.id: Brain(r, self.field) for r in self.active}
        self.sim = self.eye = self.link = None
        if source == "sim":
            self.sim = Sim(self.field, self.all_robots, self.params, seed=seed)
            self.sim.reset("kickoff")
        else:
            from .eye import Eye  # needs OpenCV (.venv-fc)
            self.eye = Eye(self.field, self.all_robots, self.params, source=video or camera)
            self.eye.start()
            self.link = None if no_link else Link(self.active, self.params, udp_hosts=self.udp_hosts)
        self.lock = threading.RLock()
        self.mode = "idle"
        self.test: dict | None = None
        self.run: RunLog | None = None
        self.metrics: Metrics | None = None
        self.last_summary: dict | None = None
        self.teleop: dict[str, tuple] = {}
        self.events: deque = deque(maxlen=60)
        self.ops: dict[str, dict] = {}
        self.robot_status: dict[str, dict] = {}
        self.score = {"blue": 0, "red": 0}
        self.world: World | None = None
        self.cmds: dict[str, Cmd] = {}
        self.prev: dict[str, Cmd] = {}       # last command actually sent per robot (shaper memory)
        self.debug: dict[str, dict] = {}
        self.imu_bad: set[str] = set()      # bridge IMU level bad -> zero + end test (imu_bad)
        self.mux_lost: set[str] = set()     # bridge mux_ok false -> zero + end test (mux_lost)
        self._lstat: dict[str, dict] = {}
        self._sim_ev_n = 0
        self._goal_cooldown = 0.0
        self._err_t: dict[str, float] = {}
        self._lk_t = 0.0
        self.loop_stats = {"hz": 0.0, "overruns": 0, "tick_ms": 0.0}
        self._stop = False
        self.say(f"started: source={source}, robots={[r.id for r in self.active]}"
                 + (", LINK OFF (nothing sent)" if source != "sim" and self.link is None else ""))

    # ---- small utilities -----------------------------------------------------
    def say(self, text: str, **extra) -> None:
        self.events.append({"t": time.strftime("%H:%M:%S"), "msg": text})
        if self.run:
            self.run.event({"ev": text, **clean(extra)})

    def presets(self) -> list[str]:
        return sorted(p.stem for p in self.presets_dir.glob("*.json")) if self.presets_dir.exists() else []

    def preset_modified(self) -> list[str]:
        want = {**self.base_params, **self.preset_vals}
        return sorted(k for k, v in self.params.items() if want.get(k) != v)

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
            except Exception as e:  # noqa: BLE001 - the loop must survive; stop the robots and the test
                self._throttled("loop", f"loop error: {type(e).__name__}: {e}")
                if self.link:
                    self.link.stop_all()
                try:
                    with self.lock:
                        if self.test:
                            self._finish("loop_error", error=f"{type(e).__name__}: {e}")
                        self.mode = "idle"
                        self.prev.clear()
                except Exception as e2:  # noqa: BLE001
                    self._throttled("loop2", f"loop error while stopping: {type(e2).__name__}: {e2}")
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

    def link_status(self) -> dict[str, dict]:
        return self.link.status() if self.link else {}

    def tick(self, dt: float) -> None:
        with self.lock:
            now = time.monotonic()
            world = self.sim.observe(now) if self.sim else self.eye.latest()
            if world is not None and world.t != now:
                world = replace(world, t=now)   # brain measures observation age against "now"
            self.world = world
            lstat = self._lstat = self.link_status()
            self._health_watch(lstat)
            cmds, debug = {}, {}
            for r in self.active:
                want, dbg, src = self._decide(r, world, now)
                if src == "brain":
                    cmd = want
                elif src == "hard":
                    cmd = Cmd.stop(want.tag)
                else:   # every app-made command gets the brain's clamp + rate limit
                    cmd = shape(want, self.prev.get(r.id, Cmd()), dt, self.params)
                if src != "brain" and self.mode in BRAIN_MODES and r.id in self.brains:
                    self.brains[r.id].sent(cmd)   # next brain ramp starts from what really went out
                self.prev[r.id] = cmd
                cmds[r.id], debug[r.id] = cmd, dbg
            acks = self.link.drain_acks() if (self.link and self.run) else []
            if self.metrics:
                self.metrics.tick(now, world, {k: (c.vx, c.vy, c.wz, c.tag) for k, c in cmds.items()},
                                  self.params["stale_stop_s"], lstat, acks)
            self._health_end_test()
            t = self.test
            muted = {t["robot"]} if t and t["name"] == "deadman" and t.get("phase") == "mute" else set()
            if self.sim:
                out = {r.id: cmds.get(r.id, Cmd.stop()) for r in self.all_robots}
                for rid in muted:   # emulate the bridge: last packet held for ttl_s, then zero
                    held = now - t.get("last_sent_t", now) < float(self.params.get("ttl_s", 0.25))
                    out[rid] = t.get("last_sent", Cmd()) if held else Cmd.stop("ttl")
                self.sim.step(out, dt)
            elif self.link:
                self.link.send({k: v for k, v in cmds.items() if k not in muted})
            if t and t["name"] in ("step", "deadman") and t["robot"] not in muted:
                c = cmds[t["robot"]]
                t["sent"].append((now, c.vx, c.vy, c.wz))
                t["last_sent_t"], t["last_sent"] = now, c
            self.cmds, self.debug = cmds, debug
            if self.run:
                rec = {"w": world_json(world, now),
                       "c": {k: [c.vx, c.vy, c.wz, c.tag] for k, c in cmds.items()},
                       "d": clean(debug)}
                if muted:
                    rec["mute"] = sorted(muted)
                if self.sim:
                    rec["tr"] = truth_json(self.sim.truth())
                if acks:
                    rec["a"] = acks
                if now - self._lk_t >= 0.5:   # link/IMU/eye health at 2 Hz
                    self._lk_t = now
                    if lstat:
                        rec["lk"] = {rid: {k: st.get(k) for k in (
                            "imu", "hold", "imu_hold", "mux_ok", "auth", "connected", "ack_age", "rtt_ms", "rtt_p95_ms", "loss",
                            "profile", "bridge_ok", "bridge", "extra", "sent", "acks", "addr")} for rid, st in lstat.items()}
                    if self.eye:
                        es = self.eye.status()
                        rec["eye"] = {"fps": es["fps"], "calibrated": es["calibrated"], "msg": es["message"][:80]}
                self.run.event(rec)
            self._check_test(world, now)

    def _health_watch(self, lstat: dict) -> None:
        """Robot health from the bridge acks: IMU level bad, or the mux no longer forwarding."""
        bad = {rid for rid, st in lstat.items() if ((st or {}).get("imu") or {}).get("level") == "bad"}
        for rid in sorted(bad - self.imu_bad):
            imu = lstat[rid].get("imu")
            self.say(f"IMU BAD on {rid}: sending zero ({', '.join((imu or {}).get('reasons') or []) or 'level bad'})"
                     " - zero speed does not fix a false tilt: R3 if it misbehaves", robot=rid, imu=imu)
        for rid in sorted(self.imu_bad - bad):
            self.say(f"IMU on {rid} no longer bad", robot=rid, imu=lstat.get(rid, {}).get("imu"))
        self.imu_bad = bad
        lost = {rid for rid, st in lstat.items() if (st or {}).get("mux_ok") is False}
        for rid in sorted(lost - self.mux_lost):
            self.say(f"MUX LOST on {rid} (bridge hold: {lstat[rid].get('hold')}): sending zero. The walker keeps its"
                     " last command without the mux - R3 if it is still walking", robot=rid, hold=lstat[rid].get("hold"))
        for rid in sorted(self.mux_lost - lost):
            self.say(f"mux on {rid} ok again", robot=rid)
        self.mux_lost = lost

    def halted(self, rid: str) -> str | None:
        return "imu_bad" if rid in self.imu_bad else "mux_lost" if rid in self.mux_lost else None

    def _health_end_test(self) -> None:
        t = self.test
        if not t or t["name"] == "sim_eval" or not (self.imu_bad or self.mux_lost):
            return
        involved = {t["robot"]} if t.get("robot") else {r.id for r in self.active}
        for result, ids, key in (("imu_bad", self.imu_bad, "imu"), ("mux_lost", self.mux_lost, "hold")):
            hit = sorted(involved & ids)
            if hit:
                self._finish(result, **{key: {rid: self._lstat.get(rid, {}).get(key) for rid in hit}})
                return

    def _decide(self, r: RobotCfg, world: World | None, now: float) -> tuple[Cmd, dict, str]:
        """(want, debug, src): src "brain" = already shaped by the brain, "hard" = immediate
        zero, "app" = app-made command that goes through brain.shape."""
        mode, test = self.mode, self.test or {}
        idle = (Cmd.stop("idle"), {"role": "idle", "phase": mode, "target": None}, "hard")
        if mode in ("idle", "sim_eval"):
            return idle
        halt = self.halted(r.id)
        if halt:
            return Cmd.stop(halt), {"role": "idle", "phase": halt, "target": None}, "hard"
        if mode in ONE_ROBOT_TESTS and r.id != test.get("robot"):
            return idle
        if mode == "drive":
            vx, vy, wz, t = self.teleop.get(r.id, (0, 0, 0, 0))
            if now - t > self.params["teleop_timeout_s"]:
                return Cmd.stop("teleop:timeout"), {"role": "drive", "phase": "keys idle", "target": None}, "app"
            p = self.params
            return (Cmd(vx * p["max_vx"], vy * p["max_vy"], wz * p["max_wz"], "teleop"),
                    {"role": "drive", "phase": "keys", "target": None}, "app")
        me = world.robots.get(r.id) if world else None
        fresh = me is not None and now - me.t <= self.params["stale_stop_s"]
        if mode in ("step", "deadman"):
            return self._pipeline_cmd(test, fresh, now)
        if not fresh:
            return Cmd.stop("stale"), {"role": "idle", "phase": "not seen", "target": None}, "app"
        params = self.params
        if mode == "goto":
            tgt = test["target"]
            params = {**params, "role_mode": "goto", "goto_x": tgt[0], "goto_y": tgt[1],
                      "goto_yaw": tgt[2] if len(tgt) > 2 else None}
        elif mode == "push_ball":
            params = {**params, "role_mode": "attack"}
        elif mode == "goalie":
            params = {**params, "role_mode": "defend"}
        elif mode == "match":
            params = {**params, "role_mode": "auto"}
        try:
            cmd, dbg = self.brains[r.id].decide(world, params)
        except Exception as e:  # noqa: BLE001 - a brain bug must not move the robot
            self._throttled(f"brain-{r.id}", f"brain error {r.id}: {type(e).__name__}: {e}")
            return Cmd.stop("brain_error"), {"role": "idle", "phase": "brain error", "target": None}, "hard"
        dbg = clean(dbg or {})
        if mode == "goto":
            test["done_since"] = (test.get("done_since") or now) if dbg.get("done") else None
        return cmd, dbg, "brain"

    def _pipeline_cmd(self, test: dict, fresh: bool, now: float) -> tuple[Cmd, dict, str]:
        """step / deadman phases by elapsed time. Losing the tag aborts (see _check_test)."""
        el, name = now - test["t0"], test["name"]
        dbg = {"role": name, "target": None}
        if name == "step":
            secs = test["secs"]
            ph = "pre" if el < STEP_PRE_S else "cmd" if el < STEP_PRE_S + secs else "settle"
            if ph == "cmd" and "t_cmd0" not in test:
                test["t_cmd0"] = now
            if ph == "settle" and "t_cmd1" not in test:
                test["t_cmd1"] = now
            want = Cmd(*test["cmd"], "step") if ph == "cmd" else Cmd.stop(f"step:{ph}")
        else:
            drive = self.params["deadman_drive_s"]
            ph = "pre" if el < DEADMAN_PRE_S else "drive" if el < DEADMAN_PRE_S + drive else "mute"
            if ph == "mute" and "t_mute" not in test:
                test["t_mute"] = test.get("last_sent_t", now)   # time of the last packet that went out
            want = Cmd(test["vx"], 0.0, 0.0, "deadman") if ph == "drive" else Cmd.stop(f"deadman:{ph}")
        test["phase"] = ph
        dbg["phase"] = ph
        if not fresh and ph != "mute" and ph != "settle":
            test.setdefault("abort", "lost_tag")
            return Cmd.stop("stale"), dbg, "hard"
        return want, dbg, "app"

    # ---- test bookkeeping --------------------------------------------------------
    def _goal_events(self, world: World | None, now: float) -> list[str]:
        """Teams that scored since last tick."""
        if self.sim is not None:
            new = self.sim.events[self._sim_ev_n:]
            self._sim_ev_n = len(self.sim.events)
            return [ev["team"] for ev in new if ev.get("type") == "goal" and ev.get("team") in ("blue", "red")]
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
            if self.sim is not None:
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
        elif name in ("step", "deadman"):
            me = world.robots.get(t["robot"]) if world else None
            if me is not None and (not t["obs"] or me.t > t["obs"][-1][0]):
                t["obs"].append((me.t, me.pose.x, me.pose.y, me.pose.yaw))
                f = self.field
                if t.get("phase") in ("cmd", "drive") and (abs(me.pose.x) > f.length / 2 + 0.05
                                                         or abs(me.pose.y) > f.width / 2 + 0.05):
                    t.setdefault("abort", "edge")
            if name == "step":
                over = el > STEP_PRE_S + t["secs"] + self.params["step_settle_s"]
                if t.get("abort") or over:
                    rep = analyze_step(t["obs"], [s for s in t["sent"] if s[0] >= t.get("t_cmd0", now)],
                                       t.get("t_cmd0", now), t.get("t_cmd1", now), t["cmd"])
                    rep["cmd_asked"] = t["cmd_asked"]
                    res = rep.pop("result")
                    self._finish(t.get("abort") or res, time_s=round(el, 2), **rep)
            else:
                over = el > DEADMAN_PRE_S + self.params["deadman_drive_s"] + t["mute_s"]
                if t.get("abort") or over:
                    rep = analyze_deadman(t["obs"], t.get("t_mute"))
                    rep.update(vx=t["vx"], mute_s=t["mute_s"], ttl_s=self.params.get("ttl_s"),
                               emulated=self.sim is not None)
                    res = rep.pop("result")
                    self._finish(t.get("abort") or res, time_s=round(el, 2), **rep)

    def _goto_report(self, t: dict, el: float) -> dict:
        p, tgt = t.get("last_pose"), t["target"]
        if p is None:
            return {"time_s": round(el, 2), "pos_err_m": None}
        want_yaw = tgt[2] if len(tgt) > 2 and tgt[2] is not None else math.atan2(tgt[1] - t["start"][1], tgt[0] - t["start"][0]) if t.get("start") else None
        err = round(math.hypot(p[0] - tgt[0], p[1] - tgt[1]), 3)
        return {"time_s": round(el, 2), "target": tgt, "final": p, "pos_err_m": err,
                "heading_err_deg": None if want_yaw is None else round(math.degrees(wrap(p[2] - want_yaw)), 1),
                "pass": err < 0.08}

    def _reset_sim(self, scenario: str, robots: list[str] | None = None) -> None:
        try:
            if robots:
                self.sim.reset(scenario, robots=robots)
            else:
                self.sim.reset(scenario)
        except Exception as e:  # noqa: BLE001
            self.say(f"sim reset {scenario!r} failed ({e}); using kickoff")
            self.sim.reset("kickoff")
        self._sim_ev_n = len(self.sim.events)

    def _begin(self, name: str, robot: str | None, **extra) -> dict:
        """Start a test/mode with its own run log. Caller holds the lock."""
        if self.test:
            self._finish("replaced")
        self.test = {"name": name, "robot": robot, "t0": time.monotonic(), **extra}
        self.mode = name
        for b in self.brains.values():  # fresh role/latch memory for every run
            b.reset()
        lstat = self.link_status()
        meta = {"test": name, "robot": robot, "source": self.source, "impl": self.impl_note,
                "active": [r.id for r in self.active], "extra": clean(extra),
                "preset": self.preset_name, "preset_modified": self.preset_modified(),
                "preset_profile": self.preset_profile,
                "profile_running": {rid: st.get("profile") for rid, st in lstat.items()},
                "link": {rid: {"addr": st.get("addr"), "via": st.get("via"), "rtt_p95_ms": st.get("rtt_p95_ms")}
                         for rid, st in lstat.items()}}
        self.run = RunLog(name, dict(self.params), meta, root=self.runs_root)
        self.metrics = Metrics([r.id for r in self.active], time.monotonic(), self.loop_stats["overruns"], lstat)
        self.test["run_dir"] = str(self.run.dir)
        self.say(f"start {name}" + (f" ({robot})" if robot else "") + f" -> {self.run.dir.name}"
                 + (f" [preset {self.preset_name}]" if self.preset_name else ""))
        return self.test

    def _finish(self, result: str, **summary) -> None:
        """End the current test; summary.json written on a side thread. Caller holds the lock."""
        t, run, metrics = self.test, self.run, self.metrics
        self.test, self.run, self.metrics, self.mode = None, None, None, "idle"
        self.teleop.clear()
        if self.link:
            self.link.stop_all()
        self.prev.clear()
        if not t:
            return
        m = metrics.summary(time.monotonic(), self.loop_stats["overruns"], self.link_status()) if metrics else None
        summary = clean({"result": result, "robot": t.get("robot"), "preset": self.preset_name, **summary,
                         "metrics": m})
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

    @staticmethod
    def _num(body: dict, key: str, default: float, lo: float, hi: float) -> float:
        v = body.get(key, default)
        if v is None or v == "":
            v = default
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError(f"{key} must be a number in [{lo}, {hi}]")
        return float(v)

    def _start_test(self, body: dict) -> dict:
        name = body.get("test")
        if name not in TESTS:
            return {"ok": False, "error": f"unknown test {name!r}; one of {TESTS}"}
        active = [r.id for r in self.active]
        robot = body.get("robot") or active[0]
        if name in ONE_ROBOT_TESTS and robot not in active:
            return {"ok": False, "error": f"robot {robot!r} not active ({active})"}
        if name in ONE_ROBOT_TESTS and self.halted(robot):
            return {"ok": False, "error": f"{robot}: {self.halted(robot)} reported by the bridge; fix that first"
                                         " (IMU test / restart the stack; after a mux crash wait ~20 s)"}
        extra: dict = {}
        try:
            if name == "step":
                p = self.params
                asked = [self._num(body, "vx", 0.0, -0.5, 0.5), self._num(body, "vy", 0.0, -0.3, 0.3),
                         self._num(body, "wz", 0.0, -2.0, 2.0)]
                cmd = [max(-p[k], min(p[k], v)) for v, k in zip(asked, ("max_vx", "max_vy", "max_wz"))]
                if not any(cmd):
                    return {"ok": False, "error": "step needs a non-zero vx, vy or wz (within max_*)"}
                extra = {"cmd": cmd, "cmd_asked": asked, "secs": self._num(body, "seconds", p["step_s"], 0.2, 10.0),
                         "obs": [], "sent": []}
            elif name == "deadman":
                extra = {"vx": self._num(body, "vx", self.params["deadman_vx"], 0.03, 0.2),
                         "mute_s": self._num(body, "mute_s", self.params["deadman_mute_s"], 0.5, 5.0),
                         "obs": [], "sent": []}
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        if self.sim:
            scen = body.get("scenario") or SIM_SCENARIO.get(name)
            in_sim = set((truth_json(self.sim.truth()) or {}).get("robots", {}))
            if scen:
                self._reset_sim(scen, [robot] if name in ("push_ball", "goalie") else None)
            elif name in ("drive", "goto", "step", "deadman") and robot not in in_sim:
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
            self._begin(name, robot, **extra)
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
            self.prev.clear()
        return {"ok": True}

    def _apply_params(self, new: dict) -> None:
        """Replace every value in place: Link and Eye hold this same dict."""
        self.params.clear()
        self.params.update(new)
        if self.sim is not None:
            self.sim.set_params(self.params)  # the sim copies its params

    def set_params(self, changes: dict) -> dict:
        with self.lock:
            ok, errors = validate_params(self.params, changes)
            if ok:
                self._apply_params({**self.params, **ok})
                self.say("params: " + ", ".join(f"{k}={v}" for k, v in ok.items()), params=ok)
        return {"ok": not errors, "applied": ok, "errors": errors}

    def preset(self, body: dict) -> dict:
        """op load: base params + the preset's keys (presets never stack). op save: only the keys
        that differ from base (plus the preset's robot profile). op base: back to base params.
        Keys starting with "_" are metadata: _profile (robot profile for start), _note."""
        op = body.get("op")
        if op == "base":
            with self.lock:
                self._apply_params(dict(self.base_params))
                self.preset_name, self.preset_vals, self.preset_profile, self.preset_note = None, {}, None, ""
                self.say("params reset to base (params.json + defaults)")
            return {"ok": True}
        name = str(body.get("name", ""))
        if not PRESET_NAME_RE.fullmatch(name):
            return {"ok": False, "error": "preset name: letters, digits, - and _ only"}
        path = self.presets_dir / f"{name}.json"
        if op == "save":
            with self.lock:
                diff = {k: v for k, v in sorted(self.params.items()) if self.base_params.get(k) != v}
                if self.preset_profile:
                    diff = {"_profile": self.preset_profile, **diff}
                self.presets_dir.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(diff, indent=1) + "\n")
                self.preset_name, self.preset_vals = name, {k: v for k, v in diff.items() if not k.startswith("_")}
                self.say(f"preset saved: {name} ({len(self.preset_vals)} keys differ from base)")
            return {"ok": True, "keys": sorted(diff)}
        if op == "load":
            if not path.exists():
                return {"ok": False, "error": f"no preset {name}"}
            try:
                data = json.loads(path.read_text())
                if not isinstance(data, dict):
                    raise ValueError("not a JSON object")
            except ValueError as e:
                return {"ok": False, "error": f"preset {name}: {e}"}
            vals = {k: v for k, v in data.items() if not k.startswith("_")}
            with self.lock:
                ok, errors = validate_params(self.base_params, vals)
                prof = data.get("_profile")
                known = self.profiles()
                if prof is not None and prof not in known:
                    errors.append(f"_profile {prof!r} is not one of {known} (start will refuse it)")
                self._apply_params({**self.base_params, **ok})
                self.preset_name, self.preset_vals = name, ok
                self.preset_profile = prof if isinstance(prof, str) else None
                self.preset_note = str(data.get("_note", ""))
                self.say(f"preset loaded: {name} ({len(ok)} keys"
                         + (f", profile {self.preset_profile}" if self.preset_profile else "")
                         + (f"; {len(errors)} rejected: {'; '.join(errors)}" if errors else "") + ")",
                         preset=name, params=ok)
            return {"ok": True, "applied": len(ok), "errors": errors}
        return {"ok": False, "error": "op must be save, load or base"}

    # ---- robot ops over ssh ----------------------------------------------------------
    @staticmethod
    def profiles() -> list[str]:
        return sorted(p.stem for p in PROFILES_DIR.glob("*.sh"))

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
        if cur and cur.get("running") and op != "stop":   # stop is never refused here
            return {"ok": False, "error": f"{cur['op']} still running on {rid}"}
        host = r.host  # only ever from robots.json, never from the request
        if not HOST_RE.fullmatch(host) or host.startswith("-"):
            return {"ok": False, "error": f"{rid}: bad host {host!r} in robots.json"}
        timeout = OP_TIMEOUT_S.get(op, 60)
        # ssh joins remote args into one string for the Pi's shell, so every argument is
        # allowlisted here and shlex-quoted below.
        if op == "start":
            profile = body.get("profile") or self.preset_profile or r.profile
            known = self.profiles()
            if profile not in known:
                return {"ok": False, "error": f"unknown profile {profile!r}; one of {known}"}
            args = ["start", profile, "--confirm"]
        elif op == "imu_test":
            secs = body.get("seconds", 5)
            lo, hi = IMU_TEST_S
            if isinstance(secs, bool) or not isinstance(secs, int) or not lo <= secs <= hi:
                return {"ok": False, "error": f"seconds must be an integer in [{lo}, {hi}]"}
            args = ["imu-test", str(secs)]
            mode, cond = body.get("mode"), body.get("condition")
            if mode is not None:
                if mode not in IMU_MODES:
                    return {"ok": False, "error": f"mode must be one of {IMU_MODES}"}
                args += ["--mode", mode]
            if cond is not None:
                if not isinstance(cond, str) or not IMU_CONDITION_RE.fullmatch(cond):
                    return {"ok": False, "error": "condition: 1-24 of a-z, 0-9, _"}
                args += ["--condition", cond]
            timeout = secs + 90
        else:
            args = {"status": ["status"], "stop": ["stop"], "bridge_start": ["bridge", "start", "--robot-id", rid],
                    "bridge_stop": ["bridge", "stop"], "stage": []}[op]
        if op == "stage":
            cmd = ["bash", str(HERE / "robot" / "stage.sh"), host]  # local script, no shell
        else:
            remote = "bash ~/fc-v1/robot/fcctl.sh " + " ".join(shlex.quote(a) for a in args)
            cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "--", host, remote]
        if self.dry_run_ops:
            cmd = ["echo", "DRY-RUN:", *cmd]
        state = {"op": op, "cmd": " ".join(cmd), "running": True, "rc": None, "out": "", "t": time.strftime("%H:%M:%S"),
                 "timeout_s": timeout}
        self.ops[rid] = state
        self.say(f"robot {rid}: {op}" + (" (dry run)" if self.dry_run_ops else ""), cmd=state["cmd"])
        threading.Thread(target=self._run_op, args=(rid, state, cmd, timeout), daemon=True).start()
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
        self.say(f"robot {rid}: {state['op']} -> rc {state['rc']}", out=state["out"][-1500:])

    # ---- state for the UI ----------------------------------------------------------------
    def state(self) -> dict:
        with self.lock:
            now = time.monotonic()
            test = None
            if self.test:
                test = {k: v for k, v in self.test.items() if k not in ("t0", "output", "obs", "sent", "last_sent")}
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
                "debug": self.debug, "link": self._lstat if self.link else None, "imu_bad": sorted(self.imu_bad),
                "mux_lost": sorted(self.mux_lost),
                "eye": self.eye.status() if self.eye else None, "params": self.params,
                "choices": CHOICES, "limits": LIMITS, "tests": TESTS, "presets": self.presets(),
                "preset": {"name": self.preset_name, "modified": self.preset_modified() if self.preset_name else [],
                           "profile": self.preset_profile, "note": self.preset_note},
                "profiles": self.profiles(), "udp_hosts": self.udp_hosts,
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

        def _post_allowed(self) -> bool:
            """Block cross-site POSTs: JSON content type forces a CORS preflight (which we never
            answer), and a browser-sent Origin must be this server's own."""
            if self.headers.get_content_type() != "application/json":
                return False
            origin = self.headers.get("Origin")
            return origin is None or origin in self.server.allowed_origins

        def do_POST(self):  # noqa: N802
            if not self._post_allowed():
                return self._json({"ok": False, "error": "forbidden: JSON from this page only"}, 403)
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
    p = srv.server_address[1]
    names = {"127.0.0.1", "localhost"} | ({host} if host not in ("", "0.0.0.0", "::") else set())
    srv.allowed_origins = {f"http://{h}:{p}" for h in names}  # POSTs from other origins get 403
    threading.Thread(target=srv.serve_forever, daemon=True, name="fc-http").start()
    return srv


def _udp_arg(s: str) -> tuple[str, str]:
    rid, sep, addr = s.partition("=")
    if not sep or not rid or not HOST_RE.fullmatch(addr):
        raise argparse.ArgumentTypeError("use ID=ADDRESS, e.g. blue=192.168.1.42")
    return rid, addr


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["sim", "camera"], default="sim")
    ap.add_argument("--robots", default=None, help="comma list of robot ids to control (default: all in robots.json)")
    ap.add_argument("--port", type=int, default=8800)
    ap.add_argument("--host", default="127.0.0.1", help="default loopback only; a LAN IP serves the UI to other devices (POSTs must come from that same address)")
    ap.add_argument("--video", default=None, help="camera source: video file, image folder, or 'synthetic'")
    ap.add_argument("--camera", default="phone", help="live camera name or index (fieldcam capture)")
    ap.add_argument("--dry-run-ops", action="store_true", help="robot ops print their ssh command instead of running it")
    ap.add_argument("--no-link", action="store_true", help="camera source: compute commands but send nothing")
    ap.add_argument("--udp-host", action="append", type=_udp_arg, default=[], metavar="ID=ADDR",
                    help="control packets to this address instead of the ssh host (overrides robots.json udp_host)")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(argv)
    app = App(a.source, a.robots.split(",") if a.robots else None, video=a.video, camera=a.camera,
              dry_run_ops=a.dry_run_ops, seed=a.seed, no_link=a.no_link, udp_hosts=dict(a.udp_host))
    app.start()
    srv = serve(app, a.host, a.port)
    print(f"Pupper FC v1: http://{a.host}:{a.port}/   Ctrl-C to quit", flush=True)
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
