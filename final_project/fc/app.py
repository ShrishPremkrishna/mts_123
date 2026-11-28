"""Pupper FC v1 laptop app: one 30 Hz loop + a stdlib HTTP UI.

    cd final_project && python -m fc.app --source sim
    python -m fc.app --source camera [--video FILE|DIR|synthetic] [--robots blue] [--port 8800]

World (Sim or Eye) -> Coach (1v1 tasks) -> Brain.decide per active robot -> Cmd -> brain.shape -> Sim.step or Link.send
-> run log. Every test is a named mode with its own run folder (see runlog.py). Stop is always
immediate. Core (this file, link.py, runlog.py) is stdlib only; the camera source needs OpenCV.
A broken fc.brain / fc.sim import stops the app: there is no fallback controller.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
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

from .brain import Brain, Coach, shape
from .brain import DEFAULTS as BRAIN_DEFAULTS
from .core import CONFIG, Cmd, RobotCfg, World, load_field, load_params, wrap
from .link import CEIL, Link
from .media import CamFeed, Recorder
from . import ops as OPS
from .play import (AUTO_AFTER_ROLL, AUTO_RESETS, FLOW_CHOICES, FLOW_DEFAULTS, FLOW_LIMITS, JOBS, RESTART_MODES, AutoPlay,
                   MatchFlow, Ticker, Warmup, action_label, banners, down_failed, estop_view, health, safety_banners,
                   setup_step)
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
    **FLOW_DEFAULTS,            # Play screen match flow (see play.py)
    "restart_mode": "ball_rest",  # after a goal: resume when the ball rests anywhere | walk to kickoff spots
    "record": 1,                # camera source: write each run's overhead video to <run>/overhead.mp4
    "rec_fps": 15.0,
    "shot_vmin": 0.3,           # camera goalie: ball speed toward our goal that can be a shot (ShotCounter)
    "shot_horizon_s": 4.0,      # ...predicted to reach the goal line within this
    "shot_mouth_margin": 0.1,   # ...inside the mouth widened by this on each side
    "shot_end_s": 0.3,          # ...and over once off target this long (saved / missed)
}
# Hard limits for live edits. Speed ceilings mirror link.CEIL (the hard cap on every packet).
LIMITS = {
    "loop_hz": (5, 60), "max_vx": (0.0, CEIL["vx"]), "max_vy": (0.0, CEIL["vy"]), "max_wz": (0.0, CEIL["wz"]),
    "accel_vx": (0.05, 5.0), "accel_wz": (0.1, 20.0), "ttl_s": (0.05, 0.5),
    "test_timeout_s": (1, 1800), "goalie_shot_s": (1, 30), "match_s": (10, 3600), "goto_tol_m": (0.02, 1.0),
    "stale_stop_s": (0.1, 2.0), "teleop_timeout_s": (0.1, 2.0),
    "step_s": (0.2, 10.0), "step_settle_s": (0.5, 5.0),
    "deadman_vx": (0.03, 0.2), "deadman_drive_s": (0.5, 5.0), "deadman_mute_s": (0.5, 5.0),
    "eye_floor_size": (0.03, 0.5), "eye_tag_height": (0.0, 0.5),
    "eye_tag_offset_x": (-0.3, 0.3), "eye_yaw_offset_deg": (-360, 360), "eye_latency_s": (0.0, 1.0),
    "act_lag_s": (0.0, 1.0), "decel_vx": (0.0, 5.0), "decel_wz": (0.0, 20.0), "fast_yaw_bias_deg": (-30.0, 30.0),
    "wall_margin_m": (0.18, 0.6), "wall_brake": (0.2, 3.0), "aggression": (0.0, 1.0),
}
LIMITS.update(FLOW_LIMITS, record=(0, 1), rec_fps=(2, 30))
LIMITS.update(shot_vmin=(0.05, 3.0), shot_horizon_s=(0.5, 10.0), shot_mouth_margin=(0.0, 0.5), shot_end_s=(0.1, 2.0),
              ball_decel=(0.0, 3.0))
CHOICES = {"role_mode": ["auto", "attack", "defend"], "restart_mode": RESTART_MODES, **FLOW_CHOICES}
TESTS = ["drive", "goto", "step", "deadman", "push_ball", "goalie", "match", "sim_eval", "autoplay"]
ONE_ROBOT_TESTS = {"drive", "goto", "step", "deadman", "push_ball", "goalie", "autoplay"}
BRAIN_MODES = {"play", "goto", "push_ball", "goalie", "match", "autoplay", "warmup"}
# Sim scenario each test starts from (sim source only; the UI can override).
SIM_SCENARIO = {"push_ball": "attack_solo", "goalie": "shot_on_goal", "match": "kickoff", "autoplay": "attack_solo"}
SIM_HUMAN_S = 3.0      # sim autoplay: the "please roll the ball out" banner is up this long before the sim person does it
ROBOT_OPS = ["status", "ready", "down", "start", "stop", "bridge_start", "bridge_stop", "imu_test", "stage"]
CONFIRM_OPS = {"start", "ready"}                   # may home the legs / stand the robot up
ALWAYS_OPS = {"stop", "down"}                      # never refused by a running op
PROFILES_DIR = HERE / "robot" / "profiles"
HOST_RE = re.compile(r"[A-Za-z0-9_.@-]{1,253}")   # robots.json host: [user@]name, no shell or ssh-option chars
IMU_TEST_S = (1, 900)
IMU_MODES = ("auto", "bno", "ros")
IMU_CONDITION_RE = re.compile(r"[a-z0-9_]{1,24}")
OP_TIMEOUT_S = {"start": 180, "stage": 120, "ready": 300, "down": 90}  # fcctl start waits up to 160 s for READY
CAM_URL_RE = re.compile(r"https?://[A-Za-z0-9_.\-]+(:\d{1,5})?(/[A-Za-z0-9_./\-]*)?")
STEP_PRE_S = 0.7                                   # step: still before the command (start pose)
DEADMAN_PRE_S = 0.5
PRESET_NAME_RE = re.compile(r"[A-Za-z0-9_\-]{1,40}")
# Stack options a preset's "_stack" passes to fcctl ready/start (SPEC v1.1), allowlisted.
STACK_SPEED_CAP = (0.30, 1.50)                    # = robot/stack_overlay.py SPEED_CAP_RANGE
STACK_HEADING_HOLD = ("on", "off")
STOCK_SPEED_CAP = 0.49                             # walker cap without --speed-cap
ESTOP_RESEND_S = 3.0                               # after the burst, resend e-stop each tick until an ack confirms it


def auto_args(body: dict, num) -> dict:
    """autoplay test arguments (UI / ctl / op) -> AutoPlay keywords; ValueError on a bad one."""
    out = {"minutes": num(body, "minutes", 10.0, 0.0, 240.0),
           "reset_vx": num(body, "reset_vx", 0.15, 0.05, 0.3)}
    for k, choices, default in (("reset", AUTO_RESETS, "centre"), ("after_roll", AUTO_AFTER_ROLL, "centre")):
        v = body.get(k) or default
        if v not in choices:
            raise ValueError(f"{k} must be one of {list(choices)}")
        out[k] = v
    alt = body.get("alternate", True)
    if not isinstance(alt, bool):
        raise ValueError("alternate must be true or false")
    out["alternate"] = alt
    return out


def validate_stack(raw) -> tuple[dict, list[str]]:
    """Preset "_stack" -> ({speed_cap: float, heading_hold: on|off} subset, errors)."""
    if raw is None:
        return {}, []
    if not isinstance(raw, dict):
        return {}, ["_stack must be an object like {\"speed_cap\": 0.8, \"heading_hold\": \"on\"}"]
    ok, errors = {}, []
    for k, v in raw.items():
        if k == "speed_cap":
            lo, hi = STACK_SPEED_CAP
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
                errors.append(f"_stack.speed_cap must be a number in [{lo}, {hi}]")
            else:
                ok[k] = round(float(v), 2)
        elif k == "heading_hold":
            if v not in STACK_HEADING_HOLD:
                errors.append(f"_stack.heading_hold must be one of {list(STACK_HEADING_HOLD)}")
            else:
                ok[k] = v
        else:
            errors.append(f"_stack.{k}: unknown (speed_cap, heading_hold)")
    return ok, errors


def stack_args(stack: dict) -> list[str]:
    """Validated stack dict -> fcctl options."""
    out = []
    if "speed_cap" in stack:
        out += ["--speed-cap", f"{stack['speed_cap']:.2f}"]
    if "heading_hold" in stack:
        out += ["--heading-hold", stack["heading_hold"]]
    return out


def load_robot_config(path: Path = CONFIG / "robots.json") -> tuple[list[RobotCfg], dict[str, str]]:
    """robots.json -> (RobotCfg list, {id: udp_host}). Extra keys such as "udp_host" are allowed
    even while core.RobotCfg does not declare them."""
    raw = json.loads(path.read_text())
    names = {f.name for f in fields(RobotCfg)}
    robots = [RobotCfg(**{k: v for k, v in r.items() if k in names}) for r in raw]
    return robots, {str(r["id"]): str(r.get("udp_host") or "") for r in raw}


def pov_feeds_allowed(source: str, video: str | None, camera: str, linked: bool) -> bool:
    """Onboard (robot POV) feeds only for real robots: the camera source on a live camera (not a recorded
    video or the synthetic renderer) with the link on. In sim or the test server a feed would be a stray
    HTTP client that nobody watches."""
    return source == "camera" and not video and camera != "synthetic" and linked


def variants(active: list[str]) -> list[list[str]]:
    """Demo variants the V key cycles through: every active robot, then each one alone."""
    return [list(active)] + ([[r] for r in active] if len(active) > 1 else [])


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
    for rid, r in w.robots.items():   # eye board fix (tag blurred): say so, with the heading confidence
        if getattr(r, "src", "tag") not in ("tag", "sim"):
            robots[rid].update(src=r.src, yc=r.yaw_conf)
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


def ball_crossing(b, x_line: float, decel: float) -> tuple[float, float | None]:
    """(seconds, y) when ball b (x, y, vx, vy), rolling straight and slowing at `decel`, reaches
    x = x_line; (inf, None) if it rolls away or stops first. Same model as the brain's _arrival."""
    if abs(b.vx) < 1e-6 or (x_line - b.x) * b.vx <= 0:
        return math.inf, None
    sp = math.hypot(b.vx, b.vy)
    path = (x_line - b.x) / b.vx * sp
    a = max(1e-6, decel)
    disc = sp * sp - 2 * a * path
    if disc < 0:
        return math.inf, None
    return (sp - math.sqrt(disc)) / a, b.y + b.vy * (x_line - b.x) / b.vx


class ShotCounter:
    """Shots on one team's goal from the overhead ball track (camera goalie test; the sim test
    counts its own scripted shots).

    A shot starts when the ball rolls toward our goal line at >= shot_vmin and the rolling model
    (ball_decel) has it cross the line inside the mouth (+ shot_mouth_margin) within
    shot_horizon_s. It ends as
      conceded  a goal for the other team while it is on (or within GRACE_S after it ended),
      saved     no longer on target (stopped, reversed or deflected) for shot_end_s and our robot
                came within reach of the ball (robot + ball radius + 0.12 m) during the shot,
      missed    the same without our robot near the ball (stopped short, wide),
      unclear   ball not seen for 1 s.
    A goal with no shot seen first counts as a conceded shot. Known gap: a ball carried through
    the air toward the goal looks like a shot."""
    GRACE_S = 1.5
    LOST_S = 1.0

    def __init__(self, team: str, field, params: dict):
        self.team, self.f, self.p = team, field, params
        self.side = -1.0 if team == "blue" else 1.0    # blue keeps the west goal
        self.shot: dict | None = None
        self.log: list[dict] = []
        self.quiet_until = -math.inf

    def counts(self) -> dict:
        c = {k: sum(1 for s in self.log if s["result"] == k) for k in ("conceded", "saved", "missed", "unclear")}
        return {**c, "shots": len(self.log)}

    def on_target(self, b) -> tuple[bool, float, float | None]:
        p, f = self.p, self.f
        sp = math.hypot(b.vx, b.vy)
        if sp < float(p.get("shot_vmin", 0.3)):
            return False, math.inf, None
        t, y = ball_crossing(b, self.side * f.length / 2, float(p.get("ball_decel", 0.45)))
        ok = (t <= float(p.get("shot_horizon_s", 4.0))
              and abs(y) <= f.goal_width / 2 + float(p.get("shot_mouth_margin", 0.1)))
        return ok, t, y

    def update(self, now: float, world, goals: list[str], robot_id: str | None) -> list[dict]:
        """One tick. Returns the shots that ended this tick (each also kept in self.log)."""
        ended = []
        conceded = any(g != self.team for g in goals)
        b = world.ball if world else None
        fresh = b is not None and now - b.t <= 0.3
        me = world.robots.get(robot_id) if (world and robot_id) else None
        s = self.shot
        if conceded:
            if s is not None:
                ended.append(self._end(s, now, "conceded"))
            else:
                last = self.log[-1] if self.log else None
                if last is not None and now - last["t1"] <= self.GRACE_S and last["result"] != "conceded":
                    last["result"], last["late"] = "conceded", True   # it rolled in after all
                    ended.append(last)
                else:
                    ended.append(self._end({"t0": now, "v": None, "y": None, "near": False, "off_t": None}, now,
                                           "conceded", unseen=True))
            self.quiet_until = now + 3.0   # the ball sits in the goal / is carried back
            return ended
        if now < self.quiet_until:
            return ended
        if s is None:
            if fresh:
                ok, t, y = self.on_target(b)
                if ok:
                    self.shot = {"t0": now, "v": round(math.hypot(b.vx, b.vy), 2), "y": round(y, 2),
                                 "eta_s": round(t, 2), "from": [round(b.x, 2), round(b.y, 2)], "near": False,
                                 "off_t": None, "seen_t": now}
            return ended
        if fresh:
            s["seen_t"] = now
            if me is not None:
                reach = self.f.robot_radius + self.f.ball_radius + 0.12
                s["near"] = s["near"] or math.hypot(me.pose.x - b.x, me.pose.y - b.y) <= reach
            ok = self.on_target(b)[0]
            s["off_t"] = None if ok else (s["off_t"] or now)
            if s["off_t"] is not None and now - s["off_t"] >= float(self.p.get("shot_end_s", 0.3)):
                ended.append(self._end(s, now, "saved" if s["near"] else "missed"))
        elif now - s["seen_t"] > self.LOST_S:
            ended.append(self._end(s, now, "unclear"))
        return ended

    def _end(self, s: dict, now: float, result: str, unseen: bool = False) -> dict:
        rec = {"t0": s["t0"], "t1": now, "result": result, "v": s.get("v"), "y": s.get("y"),
               "eta_s": s.get("eta_s"), "from": s.get("from"), "near": s.get("near", False)}
        if unseen:
            rec["unseen"] = True
        self.log.append(rec)
        self.shot = None
        return rec


# ---------------------------------------------------------------------------------------
# the app
# ---------------------------------------------------------------------------------------
class App:
    def __init__(self, source: str = "sim", robot_ids: list[str] | None = None, video: str | None = None,
                 camera: str = "phone", dry_run_ops: bool = False, seed: int = 0, no_link: bool = False,
                 runs_root: Path = RUNS, presets_dir: Path = PRESETS, udp_hosts: dict[str, str] | None = None):
        self.source, self.dry_run_ops = source, dry_run_ops
        self.runs_root, self.presets_dir = runs_root, presets_dir
        self.field = load_field()   # ONE live object: brains, referee, match flow, sim, eye, UI
        self.field_json = (self.field.length, self.field.width)
        self.field_source = "field.json"
        self.ball_color_path = CONFIG / "ball_color.json"
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
        self.preset_stack: dict = {}            # "_stack": speed_cap / heading_hold passed to fcctl ready/start
        self.brains = {r.id: Brain(r, self.field) for r in self.active}
        self.coach = Coach(self.brains, self.field)   # 1v1: one decision per tick for the playing robots
        self.tasks: dict[str, dict] = {}              # this tick's Coach tasks (per-robot params)
        self.sim = self.eye = self.link = None
        if source == "sim":
            self.sim = Sim(self.field, self.all_robots, self.params, seed=seed)
            self.sim.reset("kickoff")
        else:
            from .eye import Eye  # needs OpenCV (.venv-fc)
            self.eye = Eye(self.field, self.all_robots, self.params, source=video or camera)
            self.eye.start()
            self.link = None if no_link else Link(self.active, self.params, udp_hosts=self.udp_hosts)
            if self.link is not None:
                self.eye.gyro_source = self.link.gyro   # measured yaw rate from the acks: heading between tags
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
        # E-stop latch per robot: {t, wall, by: operator|robot, confirmed, reason}. Cleared only by a
        # successful Ready on that robot (or estop_clear in sim). Never by time or by acks.
        self.estops: dict[str, dict] = {}
        self.estop_cleared: dict[str, float] = {}
        self._lstat: dict[str, dict] = {}
        self._sim_ev_n = 0
        self._goal_cooldown = 0.0
        self._goal_armed = True    # camera referee: False after a goal until the ball is back in the field
        self._shots: ShotCounter | None = None    # camera goalie test: shots on the robot's goal
        self._paused = False                       # eye reported CAMERA PAUSED last tick
        self._ball_track: deque = deque(maxlen=2400)   # sim: ball sightings for /ball_track (the eye keeps its own)
        self._err_t: dict[str, float] = {}
        self._coach_errs: set[str] = set()   # Coach error kinds already logged
        self._lk_t = 0.0
        self.loop_stats = {"hz": 0.0, "overruns": 0, "tick_ms": 0.0}
        self._dt = 1.0 / 30
        # Play screen: who plays (V key), the match flow, decision ticker, robot phases, feeds, video
        self.playing: list[str] = [r.id for r in self.active]
        self.flow: MatchFlow | None = None
        self.last_match: dict | None = None
        self.auto: AutoPlay | None = None          # autoplay test: the solo demo loop (play.AutoPlay)
        self.auto_brain: Brain | None = None       # ...its own brain, whose team picks the attacked goal
        self.warm: Warmup | None = None            # show warm-up (play.Warmup), op show.warmup
        self.ticker = Ticker()
        self.robot_phase: dict[str, str] = {}
        self._op_seq = 0                            # robot op counter (ops[rid].seq)
        self.robot_phase_t: dict[str, float] = {}   # monotonic time each phase was reported (setup needs a fresh one)
        # Robot POV feeds: real robots only (camera source with a live camera and the link on), never sim/tests.
        real = pov_feeds_allowed(source, video, camera, self.link is not None)
        self.cams = {r.id: CamFeed(r.cam_url) for r in self.active
                     if real and r.cam_url and CAM_URL_RE.fullmatch(r.cam_url) and (r.host or self.udp_hosts.get(r.id))}
        self.recorder: Recorder | None = None
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
        if self.source == "camera":   # Play: one read-only fcctl status per robot, so each card shows a real next step
            for r in self.active:
                if r.host:
                    self.robot_op({"robot": r.id, "op": "status"})

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
            self._dt = dt
            now = time.monotonic()
            world = self.sim.observe(now) if self.sim else self.eye.latest()
            if world is not None and world.t != now:
                world = replace(world, t=now)   # brain measures observation age against "now"
            self.world = world
            if self.eye is not None:
                self._sync_field()
                self._watch_eye()
            elif world is not None and world.ball is not None and (
                    not self._ball_track or world.ball.t > self._ball_track[-1][0] + 1e-6):
                b = world.ball
                self._ball_track.append((b.t, b.x, b.y))
            lstat = self._lstat = self.link_status()
            self._health_watch(lstat)
            cmds, debug = {}, {}
            self.tasks = self._coach_tasks(world)
            for r in self.active:
                want, dbg, src = self._decide(r, world, now)
                if src == "brain":
                    cmd = want
                elif src == "hard":
                    cmd = Cmd.stop(want.tag)
                else:   # every app-made command gets the brain's clamp + rate limit
                    cmd = shape(want, self.prev.get(r.id, Cmd()), dt, self.params)
                if src != "brain" and self.mode in BRAIN_MODES and r.id in self.brains:
                    self._brain(r.id).sent(cmd)   # next brain ramp starts from what really went out
                self.prev[r.id] = cmd
                cmds[r.id], debug[r.id] = cmd, dbg
            if self.eye is not None and cmds:
                self.eye.note_cmds(now, cmds)   # heading between decoded tags (eye board tracking)
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
                pending = {rid for rid, e in self.estops.items()
                           if not e["confirmed"] and e["by"] == "operator" and now - e["t"] < ESTOP_RESEND_S}
                self.link.send({k: v for k, v in cmds.items() if k not in muted}, estop=pending)
            if t and t["name"] in ("step", "deadman") and t["robot"] not in muted:
                c = cmds[t["robot"]]
                t["sent"].append((now, c.vx, c.vy, c.wz))
                t["last_sent_t"], t["last_sent"] = now, c
            self.cmds, self.debug = cmds, debug
            said = self.ticker.update(now, debug, self.cfg, self.playing) if self.mode in ("play", "match") else []
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
                if said:
                    rec["tk"] = [x["text"] for x in said]
                if self.flow:
                    rec["m"] = {"phase": self.flow.phase, "score": self.flow.score, "clock": round(self.flow.clock, 2),
                                "touches": self.flow.touches}
                if now - self._lk_t >= 0.5:   # link/IMU/eye health at 2 Hz
                    self._lk_t = now
                    if lstat:
                        rec["lk"] = {rid: {k: st.get(k) for k in (
                            "imu", "hold", "imu_hold", "mux_ok", "auth", "connected", "ack_age", "rtt_ms", "rtt_p95_ms", "loss",
                            "profile", "bridge_ok", "bridge", "extra", "sent", "acks", "addr")} for rid, st in lstat.items()}
                    if self.eye:
                        es = self.eye.status()
                        rec["eye"] = {"fps": es["fps"], "calibrated": es["calibrated"], "msg": es["message"][:80],
                                      "paused": es.get("paused"), "picture_age": es.get("picture_age")}
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
        now = time.monotonic()
        for rid, st in lstat.items():   # SPEC v1.1: acks carry "estopped"
            if (st or {}).get("estopped") is not True or st.get("ack_age") is None:
                continue
            ack_t, e = now - st["ack_age"], self.estops.get(rid)
            if e is not None:
                if not e["confirmed"] and ack_t >= e["t"]:
                    e["confirmed"] = True
                    self.say(f"E-STOP confirmed by {rid}: legs off, latched until Ready", robot=rid)
            elif ack_t > self.estop_cleared.get(rid, -math.inf) + 0.5:
                extra = st.get("extra") or {}
                bridge = st.get("bridge") if isinstance(st.get("bridge"), dict) else {}
                reason = str(extra.get("estop_reason") or bridge.get("estop_reason") or "robot reports estopped")[:80]
                self._latch(rid, "robot", confirmed=True, reason=reason)

    def _latch(self, rid: str, by: str, confirmed: bool, reason: str = "", t: float | None = None) -> None:
        """E-stop latch for one robot. Caller holds the lock (or is the loop)."""
        if rid in self.estops:
            return
        self.estops[rid] = {"t": time.monotonic() if t is None else t, "wall": time.strftime("%H:%M:%S"), "by": by,
                            "confirmed": confirmed, "reason": reason}
        self.say(f"E-STOP {rid} ({by}{': ' + reason if reason else ''}): zero commands until Ready re-arms it",
                 robot=rid, estop=self.estops[rid])

    def halted(self, rid: str) -> str | None:
        if rid in self.estops:
            return "estopped"
        return "imu_bad" if rid in self.imu_bad else "mux_lost" if rid in self.mux_lost else None

    def _health_end_test(self) -> None:
        t = self.test
        if not t or t["name"] == "sim_eval" or t.get("flow") or not (self.imu_bad or self.mux_lost or self.estops):
            return
        involved = {t["robot"]} if t.get("robot") else {r.id for r in self.active}
        for result, ids, key in (("estopped", set(self.estops), "estop"), ("imu_bad", self.imu_bad, "imu"),
                                 ("mux_lost", self.mux_lost, "hold")):
            if key == "estop":
                hit = sorted(involved & ids)
                if hit:
                    self._finish(result, estop={rid: self.estops[rid] for rid in hit})
                    return
                continue
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
        if mode in ("play", "match") and r.id not in self.playing:
            return Cmd.stop("benched"), {"role": "idle", "phase": "benched", "target": None}, "hard"
        flow_goto = flow_role = flow_app = None
        if mode == "match" and test.get("flow") and self.flow:
            kind, arg = self.flow.command(r.id)
            if kind == "stop":
                return Cmd.stop(arg), {"role": "idle", "phase": arg, "target": None}, "hard"
            if kind == "app":
                flow_app = arg
            else:
                flow_goto, flow_role = (arg, None) if kind == "goto" else (None, arg)
        if mode == "warmup":
            if not self.warm or r.id not in self.warm.cfg:
                return idle
            kind, arg = self.warm.want.get(r.id, ("stop", "warmup"))
            if kind == "stop":
                return Cmd.stop(arg), {"role": "warmup", "phase": arg, "target": None}, "hard"
            flow_goto, flow_app = (arg, None) if kind == "goto" else (None, arg)
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
        if flow_app is not None:     # show moves: celebration turn, warm-up moves (smoothed by brain.shape)
            return flow_app, {"role": "show", "phase": flow_app.tag, "target": None}, "app"
        params = self.params
        if mode == "autoplay" and self.auto:
            kind, arg = self.auto.want
            if kind == "stop":
                return Cmd.stop(arg), {"role": "idle", "phase": arg, "target": None}, "hard"
            if kind == "app":
                return arg, {"role": "autoplay", "phase": "backoff", "target": None}, "app"
            if kind == "goto":
                x, y, yaw, vmax = arg
                params = {**params, "role_mode": "goto", "goto_x": x, "goto_y": y, "goto_yaw": yaw,
                          "max_vx": min(params["max_vx"], vmax), "goto_tol_m": 0.05}
            else:
                params = {**params, "role_mode": "attack"}
        elif mode == "goto" or flow_goto:
            tgt = flow_goto or test["target"]
            params = {**params, "role_mode": "goto", "goto_x": tgt[0], "goto_y": tgt[1],
                      "goto_yaw": tgt[2] if len(tgt) > 2 else None}
            if len(tgt) > 3 and tgt[3] is not None:     # show walks: a calmer speed cap
                params["max_vx"] = min(params["max_vx"], tgt[3])
        elif flow_role:
            params = {**params, "role_mode": flow_role}
        elif mode == "push_ball":
            params = {**params, "role_mode": "attack"}
        elif mode == "goalie":
            params = {**params, "role_mode": "defend"}
        elif mode == "match":
            params = {**params, "role_mode": "auto"}
        if params.get("role_mode") == "auto" and r.id in self.tasks:   # 1v1: the Coach's task for this robot
            params = {**params, **self.tasks[r.id]}
        try:
            cmd, dbg = self._brain(r.id).decide(world, params)
        except Exception as e:  # noqa: BLE001 - a brain bug must not move the robot
            self._throttled(f"brain-{r.id}", f"brain error {r.id}: {type(e).__name__}: {e}")
            return Cmd.stop("brain_error"), {"role": "idle", "phase": "brain error", "target": None}, "hard"
        dbg = clean(dbg or {})
        if mode == "goto":
            test["done_since"] = (test.get("done_since") or now) if dbg.get("done") else None
        return cmd, dbg, "brain"

    def _coach_tasks(self, world: World | None) -> dict[str, dict]:
        """Play / match: the Coach over the playing robots. It assigns tasks only while every playing robot is in
        auto (a forced attack / defend job means no Coach), both are tracked and the ball is seen ({} otherwise:
        each brain plays alone, vs a human). A Coach error is logged once per kind and gives {}: the match goes on."""
        if world is None or self.mode not in ("play", "match"):
            return {}
        flow = self.flow if self.mode == "match" and (self.test or {}).get("flow") else None
        play_job = self.params.get("role_mode") if self.mode == "play" else "auto"   # (a match runs auto)
        if any((flow.roles.get(k, "auto") if flow else play_job) != "auto" for k in self.playing):
            return {}
        seen = {k: o for k, o in world.robots.items() if k in self.playing and k in self.brains}
        try:
            return self.coach.update(replace(world, robots=seen), self.params)
        except Exception as e:  # noqa: BLE001 - a Coach bug must not stop the loop: both brains play alone
            if type(e).__name__ not in self._coach_errs:
                self._coach_errs.add(type(e).__name__)
                self.say(f"coach error (each robot plays alone): {type(e).__name__}: {e}")
            return {}

    def _brain(self, rid: str) -> Brain:
        """The brain driving this robot now: autoplay's own (its team = the goal it attacks), else the robot's."""
        if self.auto_brain is not None and self.mode == "autoplay" and rid == (self.test or {}).get("robot"):
            return self.auto_brain
        return self.brains[rid]

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
    def _sync_field(self) -> None:
        """field_from_camera: length/width from the corner-tag calibration (resizes the shared
        Field in place). Calibration lost: keep the last measured size. Param off: field.json."""
        want, src = self.field_json, "field.json"
        if self.params.get("field_from_camera", True):
            size = self.eye.calibrated_size()
            if size and 1.0 <= size[0] <= 10.0 and 0.8 <= size[1] <= 8.0:
                want, src = size, "camera"
            elif self.field_source == "camera":
                return
        if abs(want[0] - self.field.length) > 0.005 or abs(want[1] - self.field.width) > 0.005 or src != self.field_source:
            self.field.resize(*want)
            self.field_source = src
            self.say(f"field {self.field.length:.2f} x {self.field.width:.2f} m from {src}"
                     f" (goal mouth {self.field.goal_width:.2f} m from field.json)")

    def _watch_eye(self) -> None:
        """CAMERA PAUSED (frozen feed / no frames): the eye stops refreshing observations, so robots
        go stale and stop by themselves; here it is logged, once per pause."""
        paused = bool(getattr(self.eye, "paused", False))
        if paused and not self._paused:
            self.say("CAMERA PAUSED: the overhead picture stopped changing (frozen phone feed?) - observations"
                     " are stale, robots stop; check the phone / camera app", camera_paused=True)
        elif self._paused and not paused:
            self.say("camera picture moving again", camera_paused=False)
        self._paused = paused

    def ball_track(self, since: float = -math.inf) -> list:
        """Raw ball sightings [(t_capture, x, y)] newer than `since` (for ctl rollcal)."""
        if self.eye is not None:
            return self.eye.ball_track(since)
        with self.lock:
            return [p for p in self._ball_track if p[0] > since]

    def ballcal(self, body: dict) -> dict:
        """Ball colour calibration on the live camera frame. body: {} auto-find, {x, y} pixel,
        {u, v} 0..1 of the frame, {show: true} report only; optional k (MAD multiplier)."""
        if self.eye is None:
            return {"ok": False, "error": "no camera (sim source)"}
        try:
            if body.get("show"):
                return {"ok": True, **self.eye.ball_show()}
            x, y = body.get("x"), body.get("y")
            if body.get("u") is not None and body.get("v") is not None:
                frame, _ = self.eye.frame()
                if frame is None:
                    raise ValueError("no camera frame yet")
                x, y = float(body["u"]) * frame.shape[1], float(body["v"]) * frame.shape[0]
            k = float(body.get("k", 3.0))
            res = self.eye.ball_calibrate(None if x is None else float(x), None if y is None else float(y), k)
            if getattr(self.eye, "synthetic", False):   # never overwrite the floor calibration from a rendered ball
                return {"ok": True, **res, "saved": None, "warnings": ["synthetic video: range not saved"]}
            from . import ballcal
            ballcal.save(self.ball_color_path, res["colors"], how=res["how"], u=res["u"], v=res["v"], k=k,
                         saved=time.strftime("%Y-%m-%d %H:%M:%S"))
            res["saved"] = str(self.ball_color_path)
            self.say(f"ball colour calibrated ({res['how']} at {res['u']:.0f},{res['v']:.0f}): {res['colors']}"
                     f", {res['candidates']} candidate(s)")
            return {"ok": True, **res}
        except (ValueError, TypeError) as e:
            return {"ok": False, "error": str(e)}

    def robotcal(self, body: dict) -> dict:
        """Robot tag-board colour (eye blob tracking) from the live frame. body: {robot, k?} calibrate
        (robot still and its tag in view), {robot, show: true} report only. Saved to robot_colors.json."""
        if self.eye is None:
            return {"ok": False, "error": "no camera (sim source)"}
        try:
            rid = str(body.get("robot") or "")
            if body.get("show"):
                return {"ok": True, **self.eye.robot_show(rid)}
            k = float(body.get("k", 3.0))
            res = self.eye.robot_calibrate(rid, k)
            if getattr(self.eye, "synthetic", False):
                return {"ok": True, **res, "saved": None, "warnings": ["synthetic video: colour not saved"]}
            from .eye import ROBOT_COLORS_PATH, save_robot_colors
            path = getattr(self, "robot_colors_path", ROBOT_COLORS_PATH)
            save_robot_colors(path, rid, res["colors"], how=f"robotcal tag {res['tag']} at {res['u']:.0f},{res['v']:.0f}",
                              k=k, saved=time.strftime("%Y-%m-%d %H:%M:%S"))
            res["saved"] = str(path)
            self.say(f"robot {rid} board colour calibrated: {res['colors']}, {res['candidates']} candidate(s)")
            return {"ok": True, **res}
        except (ValueError, TypeError) as e:
            return {"ok": False, "error": str(e)}

    def _goal_events(self, world: World | None, now: float) -> list[str]:
        """Teams that scored since last tick."""
        if self.sim is not None:
            new = self.sim.events[self._sim_ev_n:]
            self._sim_ev_n = len(self.sim.events)
            return [ev["team"] for ev in new if ev.get("type") == "goal" and ev.get("team") in ("blue", "red")]
        b = world.ball if world else None
        if b is None:
            return []
        f = self.field
        # Goal when the ball centre reaches the goal line (minus an optional margin) inside the mouth.
        # Re-armed once the ball is back in the field: a ball left lying in the goal scores once.
        line = f.length / 2 - float(self.params.get("goal_line_margin_m", 0.0))
        if abs(b.x) < line - 0.15:
            self._goal_armed = True
        if now < self._goal_cooldown or not self._goal_armed:
            return []
        if abs(b.x) >= line and abs(b.y) < f.goal_width / 2:
            self._goal_cooldown = now + 3.0
            self._goal_armed = False
            return ["blue" if b.x > 0 else "red"]
        return []

    def _check_test(self, world: World | None, now: float) -> None:
        goals = self._goal_events(world, now)
        if self.test and self.test.get("flow") and self.flow:
            return self._flow_tick(world, now, goals)
        for team in goals:
            self.score[team] += 1
            self.say(f"GOAL {team}  ({self.score['blue']}-{self.score['red']})", team=team)
        t = self.test
        if not t:
            return
        el = now - t["t0"]
        name = t["name"]
        if name == "autoplay" and self.auto:
            return self._auto_tick(t, world, now, goals)
        if name == "warmup" and self.warm:
            return self._warm_tick(t, world, now)
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
            else:
                # camera: shots seen in the ball track (ShotCounter); goals come from the same referee
                sc = self._shots if self._shots is not None and self._shots.team == team else None
                if sc is None:
                    sc = self._shots = ShotCounter(team, self.field, self.params)
                for shot in sc.update(now, world, goals, t["robot"]):
                    self.say(f"shot on {team}'s goal: {shot['result'].upper()}"
                             + (f" ({shot['v']} m/s at y {shot['y']})" if shot.get("v") is not None else " (not seen coming)")
                             + (" - rolled in late" if shot.get("late") else ""), shot=shot)
                t.update(sc.counts())
            if el > self.params["test_timeout_s"]:
                if self.sim is not None:
                    self._finish("done", conceded=t["conceded"], saved=t["saved"], shots=t["conceded"] + t["saved"],
                                 time_s=round(el, 2))
                else:
                    c = self._shots.counts() if self._shots else {}
                    self._finish("done", conceded=t["conceded"], saved=c.get("saved", 0), shots=c.get("shots", 0),
                                 missed=c.get("missed", 0), unclear=c.get("unclear", 0),
                                 shot_log=self._shots.log if self._shots else [], time_s=round(el, 2))
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

    def _flow_tick(self, world: World | None, now: float, goals: list[str]) -> None:
        f, t = self.flow, self.test
        before = f.phase
        for ev in f.update(now, self._dt, world, self.debug, goals, self.params):
            if ev.startswith("goal:"):
                team = ev[5:]
                self.say(f"GOAL {team}  ({f.score['blue']}-{f.score['red']})", team=team, clock=f.clock)
                self.ticker.add(None, team, f"GOAL! {team.capitalize()} scores  {f.score['blue']}-{f.score['red']}", "goal")
            elif ev == "play":
                for b in self.brains.values():
                    b.reset()
                self.coach.reset()
                self.ticker.clear()
            elif ev == "full_time":
                self.ticker.add(None, None, f"Full time  {f.score['blue']}-{f.score['red']}", "goal")
                self._finish("full_time", score=dict(f.score), touches=f.touches, goals=f.goals,
                             time_s=round(now - t["t0"], 1))
                return
        self.score = dict(f.score)
        if f.phase != before:
            if f.phase == "celebrate":
                g = f.goals[-1] if f.goals else {}
                self.say(f"celebrate: {g.get('team')} {self.params.get('celebrate_style')}", scorer=g.get("team"))
            if f.phase in ("celebrate", "kickoff"):    # fresh goto memory (goto_done latch), ramp from what went out
                self._reset_brains()
        # sim referee: robots in position but the ball is not at the centre -> put it back (kickoff reset). With
        # keep_ball (show matches: robots must not teleport during a celebration) it only places the ball, like the
        # operator; a ball left in a net / off the field waiting for a restart is placed at the centre too.
        sim_ref = self.sim and ((f.phase == "kickoff" and f.in_pos and all(f.in_pos.values()) and not f.ball_ok)
                                or (f.phase == "restart" and self.sim.keep_ball and not f.ball_ok))
        if sim_ref:
            if now - t.setdefault("ref_wait", now) > (1.0 if f.phase == "kickoff" else 3.0):
                if self.sim.keep_ball:
                    self.sim.kick(0.0, 0.0, 0.0, 0.0)
                    self.say("sim: the operator puts the ball on the centre spot")
                else:
                    self._reset_sim("kickoff", list(f.cfg))
                t.pop("ref_wait", None)
        else:
            t.pop("ref_wait", None)

    def _reset_brains(self) -> None:
        for rid, b in self.brains.items():
            b.reset()
            b.out = self.prev.get(rid, Cmd())
        self.coach.reset()

    def _warm_tick(self, t: dict, world: World | None, now: float) -> None:
        w = self.warm
        evs = w.update(now, world, self.debug, self.params)
        for ev in evs:
            if ev.startswith("step:") and ev.endswith(":home"):    # fresh goto memory for each walk back
                rid = ev.split(":")[1]
                self.brains[rid].reset()
                self.brains[rid].out = self.prev.get(rid, Cmd())
            elif ev.startswith("recentre:"):
                self.say(f"warm-up: {ev[9:]} back to its spot early (fence margin / leash / other robot)")
        if "done" in evs:
            self.ticker.add(None, None, "Warm-up done: robots on their kickoff spots")
            return self._finish("done", **w.summary(now))
        t.update(steps={rid: st["step"] for rid, st in w.st.items()}, finishing=w.finishing)

    def _auto_tick(self, t: dict, world: World | None, now: float, goals: list[str]) -> None:
        a, rid = self.auto, t["robot"]
        evs = a.update(now, world, goals)
        if "done" in evs:
            return self._finish("done", **a.summary(now))
        for ev in evs:
            if ev.startswith("goal:"):
                g = a.cur
                self.say(f"autoplay GOAL in the {ev[5:]} goal{' (own goal)' if g.get('own') else ''}"
                         f" after {g.get('attack_s')} s", goal=ev[5:], own=g.get("own"), attack_s=g.get("attack_s"))
            elif ev == "rolled":
                self.say("autoplay: ball back in the field (intervention)", interventions=a.interventions)
            elif ev == "attack":
                last = a.cycles[-1] if a.cycles else None
                self.say(f"autoplay: attack the {'east' if a.att_sign() > 0 else 'west'} goal"
                         + (f" (reset {last['reset']} in {last['reset_s']} s)" if last else ""), cycle=last)
            if ev.startswith("phase:") or ev == "attack":    # fresh brain memory per phase, ramp from what went out
                b = self.auto_brain
                if ev == "attack":
                    b.cfg = replace(self.cfg[rid], team=a.team)
                b.reset()
                b.out = self.prev.get(rid, Cmd())
        # sim: the person who rolls the ball out of the net once the banner has been up a while
        if (self.sim is not None and a.phase == "wait_ball" and a.out_long and a.wait_t is not None
                and now - a.wait_t >= a.p["out_s"] + SIM_HUMAN_S and not t.get("sim_rolled_t")):
            self._sim_roll_out(a)
            t["sim_rolled_t"] = now
            t["sim_rolls"] = t.get("sim_rolls", 0) + 1
        if a.phase != "wait_ball":
            t.pop("sim_rolled_t", None)
        s = a.json(now)
        t.update(phase=a.phase, words=s["words"], goals=s["goals"], own_goals=s["own_goals"], cycles=s["cycles"],
                 interventions=s["interventions"], banner=(a.banner or {}).get("text"), attacking=s["attacking"])

    def _sim_roll_out(self, a: AutoPlay) -> None:
        """Sim stand-in for the person: roll the ball from the goal mouth (or a random spot) back into the field."""
        r, f = self.sim.rng, self.field
        bx = self.sim.ball[0]
        s = (1.0 if bx > 0 else -1.0) if abs(bx) > 0.5 else r.choice((-1.0, 1.0))
        x, y = s * (f.length / 2 - 0.25), r.uniform(-0.3, 0.3)
        tx, ty = s * r.uniform(0.0, 0.5), r.uniform(-0.4, 0.4)
        self.sim.shoot(tx, ty, 0.0, x=x, y=y)
        self.say("sim: a person rolls the ball back onto the field")

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
        self._shots = None
        self.mode = name
        for b in self.brains.values():  # fresh role/latch memory for every run
            b.reset()
        self.coach.reset()
        lstat = self.link_status()
        meta = {"test": name, "robot": robot, "source": self.source, "impl": self.impl_note,
                "field": asdict(self.field), "field_source": self.field_source,
                "active": [r.id for r in self.active], "extra": clean(extra),
                "preset": self.preset_name, "preset_modified": self.preset_modified(),
                "preset_profile": self.preset_profile, "preset_stack": self.preset_stack,
                "profile_running": {rid: st.get("profile") for rid, st in lstat.items()},
                "stack_running": {rid: {k: (self.robot_status.get(rid) or {}).get(k) for k in ("speed_cap", "heading_hold")}
                                  for rid in self.robot_status},
                "estops": clean(self.estops),
                "link": {rid: {"addr": st.get("addr"), "via": st.get("via"), "rtt_p95_ms": st.get("rtt_p95_ms")}
                         for rid, st in lstat.items()}}
        self.run = RunLog(name, dict(self.params), meta, root=self.runs_root)
        self.metrics = Metrics([r.id for r in self.active], time.monotonic(), self.loop_stats["overruns"], lstat)
        self.test["run_dir"] = str(self.run.dir)
        if self.eye is not None and self.params.get("record"):
            self.recorder = Recorder(self.run.dir / "overhead.mp4", self.eye.frame, fps=float(self.params["rec_fps"]))
        self.say(f"start {name}" + (f" ({robot})" if robot else "") + f" -> {self.run.dir.name}"
                 + (f" [preset {self.preset_name}]" if self.preset_name else ""))
        return self.test

    def _finish(self, result: str, **summary) -> None:
        """End the current test; summary.json written on a side thread. Caller holds the lock."""
        t, run, metrics = self.test, self.run, self.metrics
        self.test, self.run, self.metrics, self.mode = None, None, None, "idle"
        if self.recorder:
            self.recorder.stop()
            summary.setdefault("video", {"file": "overhead.mp4", "frames": self.recorder.frames,
                                         "error": self.recorder.error})
            self.recorder = None
        if self.flow is not None:
            self.last_match = {**self.flow.json(time.monotonic(), self.params), "result": result}
            self.flow = None
        if self.warm is not None:
            if t and t.get("name") == "warmup" and "recentres" not in summary:
                summary = {**self.warm.summary(time.monotonic()), **summary}
            self.warm = None
        if self.auto is not None:
            if t and t.get("name") == "autoplay" and "goals" not in summary:
                summary = {**self.auto.summary(time.monotonic()), **summary}
            self.auto = self.auto_brain = None
        if self.sim is not None and self.sim.keep_ball:
            self.sim.keep_ball = False
            if self.sim.netted:       # a ball left lying in the net would stay frozen: back to kickoff
                self._reset_sim("kickoff")
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
        if a == "estop":
            return self.estop(body.get("robot"))
        with self.lock:
            if a == "estop_clear":
                return self._estop_clear_sim(body.get("robot"))
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
            if a in ("kickoff", "play", "play_here", "start_now", "pause", "resume", "reposition", "touch", "variant",
                     "job", "end"):
                return self._play_action(a, body)
        return {"ok": False, "error": f"unknown action {a!r}"}

    def _play_action(self, a: str, body: dict) -> dict:
        """Play screen controls. Caller holds the lock."""
        now, f = time.monotonic(), self.flow
        active = [r.id for r in self.active]
        if a == "variant":
            opts = variants(active)
            want = body.get("playing")
            if want is None:
                i = next((k for k, v in enumerate(opts) if v == self.playing), -1)
                want = opts[(i + 1) % len(opts)]
            if not (isinstance(want, list) and want and all(w in active for w in want)):
                return {"ok": False, "error": f"playing must be a non-empty list of active robots, e.g. {opts}"}
            self.playing = [rid for rid in active if rid in want]
            if self.sim:
                self._reset_sim("kickoff", self.playing if len(self.playing) < len(self.all_robots) else None)
            if f:
                solo = len(self.playing) == 1
                f.cfg = {rid: self.cfg[rid] for rid in self.playing}
                f.roles = {rid: ("attack" if solo and f.roles.get(rid, "auto") == "auto" else f.roles.get(rid, "auto"))
                           for rid in self.playing}
                f.reposition(now)
            self.say(f"playing: {', '.join(self.playing)}")
            return {"ok": True, "playing": self.playing}
        if a == "touch":
            d = body.get("delta", 1)
            if d not in (1, -1) or isinstance(d, bool):
                return {"ok": False, "error": "delta must be 1 or -1"}
            if not f:
                return {"ok": False, "error": "no match running"}
            f.touches = max(0, f.touches + d)
            self.say(f"referee touch {'+1' if d > 0 else '-1'} -> {f.touches}", touches=f.touches)
            return {"ok": True, "touches": f.touches}
        if a in ("kickoff", "play_here") or (a == "play" and not f):
            halted = [r for r in self.playing if self.halted(r)]
            if halted:
                return {"ok": False, "error": f"{', '.join(halted)} halted by the bridge ({self.halted(halted[0])})"}
            self._begin("match", None, flow=True, score0={"blue": 0, "red": 0}, playing=list(self.playing))
            self.score = {"blue": 0, "red": 0}
            self.ticker.clear()
            self.flow = MatchFlow([self.cfg[r] for r in self.playing], self.field, now, kickoff=a == "kickoff",
                                  restart_mode=self.params.get("restart_mode", "ball_rest"))
            if self.sim is not None:   # celebration: a goal must not teleport the robots back (sim kickoff reset)
                self.sim.keep_ball = self.params.get("celebrate_style", "none") != "none"
            self.ticker.add(None, None, "Kickoff: robots walk to their spots" if a == "kickoff" else "Play from here")
            return {"ok": True, "run": self.test["run_dir"]}
        if not f:
            return {"ok": False, "error": "no match running: press Kickoff"}
        if a == "play":               # legacy one-key play: resume, else start now
            ok = f.resume(now) or f.force_start(now)
            return {"ok": True, "phase": f.phase} if ok else {"ok": False, "error": f"already {f.phase}"}
        if a == "start_now":
            ok = f.force_start(now)
            return {"ok": True, "phase": f.phase} if ok else {"ok": False, "error": f"not waiting (phase {f.phase})"}
        if a == "pause":              # strict: pausing twice is refused (resume is its own action)
            if f.pause(now):
                if self.link:
                    self.link.stop_all()      # zero now; the next tick holds it (hard stop per robot)
                return {"ok": True, "phase": f.phase}
            return {"ok": False, "error": f"cannot pause in phase {f.phase}"}
        if a == "resume":
            ok = f.resume(now)
            return {"ok": True, "phase": f.phase} if ok else {"ok": False, "error": "not paused"}
        if a == "reposition":
            f.reposition(now)
            return {"ok": True}
        if a == "job":
            rid, job = body.get("robot"), body.get("job")
            if rid not in f.cfg or job not in JOBS:
                return {"ok": False, "error": f"robot in {list(f.cfg)} and job in {JOBS}"}
            f.roles[rid] = job
            self.ticker.add(rid, self.cfg[rid].team, f"Operator: {rid.capitalize()} plays "
                            + {"auto": "either job (auto)", "attack": "attack", "defend": "goalie"}[job])
            return {"ok": True}
        self._finish("ended", score=dict(f.score), touches=f.touches, goals=f.goals)   # a == "end"
        return {"ok": True}

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
                asked = [self._num(body, k, 0.0, -CEIL[k], CEIL[k]) for k in ("vx", "vy", "wz")]
                cmd = [max(-p[k], min(p[k], v)) for v, k in zip(asked, ("max_vx", "max_vy", "max_wz"))]
                if not any(cmd):
                    return {"ok": False, "error": "step needs a non-zero vx, vy or wz (within max_*)"}
                extra = {"cmd": cmd, "cmd_asked": asked, "secs": self._num(body, "seconds", p["step_s"], 0.2, 10.0),
                         "obs": [], "sent": []}
            elif name == "deadman":
                extra = {"vx": self._num(body, "vx", self.params["deadman_vx"], 0.03, 0.2),
                         "mute_s": self._num(body, "mute_s", self.params["deadman_mute_s"], 0.5, 5.0),
                         "obs": [], "sent": []}
            elif name == "autoplay":
                extra = auto_args(body, self._num)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        if self.sim:
            scen = body.get("scenario") or SIM_SCENARIO.get(name)
            in_sim = set((truth_json(self.sim.truth()) or {}).get("robots", {}))
            if scen:
                self._reset_sim(scen, [robot] if name in ("push_ball", "goalie", "autoplay") else None)
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
        elif name == "autoplay":
            self._begin("autoplay", robot, **extra)
            self.auto = AutoPlay(self.cfg[robot], self.field, time.monotonic(), **extra)
            self.auto_brain = Brain(self.cfg[robot], self.field)
            if self.sim is not None:
                self.sim.keep_ball = True     # a goal leaves the ball in the net (or bouncing back), no kickoff reset
        elif name == "sim_eval":
            self._begin("sim_eval", None)
            threading.Thread(target=self._sim_eval, args=(self.test, str(body.get("args", ""))), daemon=True).start()
        else:
            self._begin(name, robot, **extra)
        return {"ok": True, "run": self.test["run_dir"] if self.test else None}

    def warmup(self, loops: int = 0) -> dict:
        """show.warmup: start the warm-up for the playing robots, or (running) end it at the kickoff spots."""
        with self.lock:
            if self.test and self.test.get("name") == "warmup" and self.warm:
                self.warm.finish()
                self.say("warm-up: finishing, robots walk to their kickoff spots")
                return {"ok": True, "finishing": True}
            if self.flow:
                return {"ok": False, "error": "a match is running"}
            if self.test:
                return {"ok": False, "error": f"{self.test.get('name')} is running: Stop first"}
            halted = [r for r in self.playing if self.halted(r)]
            if halted:
                return {"ok": False, "error": f"{', '.join(halted)} halted ({self.halted(halted[0])})"}
            if self.sim is not None:
                in_sim = set((truth_json(self.sim.truth()) or {}).get("robots", {}))
                if not set(self.playing) <= in_sim:
                    self._reset_sim("kickoff")
            self._begin("warmup", None, loops=int(loops), playing=list(self.playing))
            self.warm = Warmup([self.cfg[r] for r in self.playing], self.field, time.monotonic(), loops=int(loops))
            self.ticker.add(None, None, "Warm-up: robots stretch their legs at their kickoff spots")
            return {"ok": True, "run": self.test["run_dir"], "robots": list(self.playing)}

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

    def estop(self, robot: str | None = None) -> dict:
        """E-STOP (SPEC v1.1): signed e-stop burst first, then bookkeeping. robot None/"all" = every
        active robot; an unknown id also e-stops everyone (fail safe). The robot latches with its legs
        off (damped drop); the app holds zero for it until a successful Ready re-arms it."""
        active = [r.id for r in self.active]
        targets = [robot] if robot in active else active
        t_press = time.monotonic()   # before the burst: acks to its first packets confirm it
        sent = self.link.estop(targets) if self.link else []
        with self.lock:
            whole = len(targets) == len(active)
            if self.test and (whole or self.test.get("robot") in targets):
                self._finish("estop", estop=targets)
            if whole:
                self.teleop.clear()
            for rid in targets:
                self.prev.pop(rid, None)
                reason = "sim: emulated latch" if self.sim is not None else "" if self.link else "link off: nothing sent"
                self._latch(rid, "operator", confirmed=self.sim is not None, reason=reason, t=t_press)   # no-op if latched
        return {"ok": True, "robots": targets, "sent": sent}

    def _estop_clear_sim(self, robot) -> dict:
        """Sim only: stands in for the Ready that re-arms a real robot. Caller holds the lock."""
        if self.sim is None:
            return {"ok": False, "error": "a real robot is re-armed only by Ready (fcctl ready --confirm)"}
        ids = [robot] if robot in self.estops else list(self.estops) if robot in (None, "", "all") else []
        for rid in ids:
            self.estops.pop(rid, None)
            self.estop_cleared[rid] = time.monotonic()
        self.say(f"sim: e-stop cleared for {', '.join(ids) or 'nobody'}")
        return {"ok": True, "cleared": ids}

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
        """op load: base params + the preset's keys (presets never stack). op save: only the keys that differ
        from base (plus the preset's robot profile). op base: back to base params.
        Keys starting with "_" are metadata: _profile (robot profile for start), _note, _stack."""
        op = body.get("op")
        if op == "base":
            with self.lock:
                self._apply_params(dict(self.base_params))
                self.preset_name, self.preset_vals, self.preset_profile, self.preset_note = None, {}, None, ""
                self.preset_stack = {}
                self.say("params reset to base (params.json + defaults)")
            return {"ok": True}
        name = str(body.get("name", ""))
        if not PRESET_NAME_RE.fullmatch(name):
            return {"ok": False, "error": "preset name: letters, digits, - and _ only"}
        path = self.presets_dir / f"{name}.json"
        if op == "save":
            with self.lock:
                diff = {k: v for k, v in sorted(self.params.items()) if self.base_params.get(k) != v}
                if self.preset_stack:
                    diff = {"_stack": dict(self.preset_stack), **diff}
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
                stack, serr = validate_stack(data.get("_stack"))
                errors += serr
                if isinstance(prof, str) and prof.endswith("_hh0") and stack.get("heading_hold") == "on":
                    errors.append(f"_profile {prof} turns heading hold off but _stack says on: use the base profile")
                self._apply_params({**self.base_params, **ok})
                self.preset_name, self.preset_vals = name, ok
                self.preset_profile = prof if isinstance(prof, str) else None
                self.preset_stack = stack
                self.preset_note = str(data.get("_note", ""))
                self.say(f"preset loaded: {name} ({len(ok)} keys"
                         + (f", profile {self.preset_profile}" if self.preset_profile else "")
                         + (f", stack {' '.join(stack_args(stack))}" if stack else "")
                         + (f"; {len(errors)} rejected: {'; '.join(errors)}" if errors else "") + ")",
                         preset=name, params=ok)
            return {"ok": True, "applied": len(ok), "errors": errors, "stack": stack}
        return {"ok": False, "error": "op must be save, load or base"}

    # ---- robot ops over ssh ----------------------------------------------------------
    def _set_phase(self, rid: str, phase: str | None) -> None:
        """Record an fcctl phase with its time (None = unknown: changed on the Pi)."""
        if phase is None:
            self.robot_phase.pop(rid, None)
            self.robot_phase_t.pop(rid, None)
        else:
            self.robot_phase[rid], self.robot_phase_t[rid] = phase, time.monotonic()

    def phase_age(self, rid: str, now: float | None = None) -> float | None:
        t = self.robot_phase_t.get(rid)
        return None if t is None or rid not in self.robot_phase else (time.monotonic() if now is None else now) - t

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
        if op in CONFIRM_OPS and body.get("confirm") is not True:
            return {"ok": False, "error": f"{op} homes the legs: send confirm=true"}
        cur = self.ops.get(rid)
        if cur and cur.get("running") and op not in ALWAYS_OPS:   # stop/down are never refused here
            return {"ok": False, "error": f"{cur['op']} still running on {rid}"}
        host = r.host  # only ever from robots.json, never from the request
        if not HOST_RE.fullmatch(host) or host.startswith("-"):
            return {"ok": False, "error": f"{rid}: bad host {host!r} in robots.json"}
        timeout = OP_TIMEOUT_S.get(op, 60)
        # ssh joins remote args into one string for the Pi's shell, so every argument is
        # allowlisted here and shlex-quoted below.
        if op in ("start", "ready"):
            profile = body.get("profile") or self.preset_profile or r.profile
            known = self.profiles()
            if profile not in known:
                return {"ok": False, "error": f"unknown profile {profile!r}; one of {known}"}
            stack, serr = validate_stack(self.preset_stack)
            if serr:
                return {"ok": False, "error": "; ".join(serr)}
            # The operator confirmed in the UI: ready homes (stack not running) or stands (homed), and
            # re-arms a latched e-stop; fcctl needs --confirm for each. Stack options come last.
            args = [op, profile, "--confirm", *stack_args(stack)]
            pov = body.get("pov")
            if pov is not None and not isinstance(pov, bool):
                return {"ok": False, "error": "pov must be true or false"}
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
            # down/stop never get --force here: a robot not confirmed lying stays up (use R3).
            # down keep_stack=true: lie down, keep the stack homed (next Ready needs no stand).
            args = {"status": ["status"], "stop": ["stop"],
                    "down": ["down", "--keep-stack"] if body.get("keep_stack") is True else ["down"],
                    "bridge_start": ["bridge", "start", "--robot-id", rid],
                    "bridge_stop": ["bridge", "stop"], "stage": []}[op]
        if op == "stage":
            cmd = ["bash", str(HERE / "robot" / "stage.sh"), host]  # local script, no shell
        else:
            remote = "bash ~/fc-v1/robot/fcctl.sh " + " ".join(shlex.quote(a) for a in args)
            cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "--", host, remote]
        if self.dry_run_ops:
            cmd = ["echo", "DRY-RUN:", *cmd]
        self._op_seq += 1     # lets a client tell this op's result from an older one
        state = {"op": op, "cmd": " ".join(cmd), "running": True, "rc": None, "out": "", "t": time.strftime("%H:%M:%S"),
                 "timeout_s": timeout, "t0": time.monotonic(), "seq": self._op_seq}
        if op == "ready":   # POV feed after a successful Ready: default on when robots.json has a cam_url
            pov = body.get("pov")
            state["pov"] = bool(r.cam_url) if pov is None else pov
        self.ops[rid] = state
        self.say(f"robot {rid}: {op}" + (" (dry run)" if self.dry_run_ops else ""), cmd=state["cmd"])
        target = self._run_stream if op in ("ready", "down") else self._run_op
        threading.Thread(target=target, args=(rid, state, cmd, timeout), daemon=True).start()
        return {"ok": True, "cmd": state["cmd"], "seq": state["seq"]}

    def _run_stream(self, rid: str, state: dict, cmd: list[str], timeout: float) -> None:
        """Long op whose output is JSON progress lines (fcctl ready: phase, elapsed, ok, error):
        each line shows as it arrives; the newest JSON object is state["progress"]."""
        state["progress"] = {}
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                 cwd=FINAL_PROJECT)
        except OSError as e:
            state.update(out=str(e), rc=-1, running=False, progress={"ok": False, "error": str(e)})
            self.say(f"robot {rid}: {state['op']} -> {e}")
            return
        timed_out = []
        timer = threading.Timer(timeout, lambda: (timed_out.append(1), p.kill()))
        timer.start()
        try:
            for line in p.stdout:
                state["out"] = (state["out"] + line)[-4000:]
                try:
                    obj = json.loads(line)
                except ValueError:
                    continue
                if isinstance(obj, dict):
                    if obj.get("ok") is False and obj.get("msg") and not obj.get("error"):
                        obj = {**obj, "error": obj["msg"]}   # fcctl puts the reason in msg
                    state["progress"] = obj
                    if state["op"] == "ready" and isinstance(obj.get("phase"), str):
                        self._set_phase(rid, obj["phase"])
            state["rc"] = p.wait()
        finally:
            timer.cancel()
        if timed_out:
            state["progress"] = {**state["progress"], "ok": False, "error": f"timeout after {timeout:.0f} s"}
        elif state["rc"] != 0 and not state["progress"].get("error"):
            state["progress"] = {**state["progress"], "ok": False, "error": f"exit {state['rc']}"}
        state["running"] = False
        self.say(f"robot {rid}: {state['op']} -> rc {state['rc']}", out=state["out"][-1500:])
        prog = state["progress"]
        ok = state["rc"] == 0 and not timed_out
        if state["op"] == "down":
            if ok:
                self._set_phase(rid, "off")
            elif down_failed(state):
                self.say(f"robot {rid}: DOWN REFUSED - still standing? use R3 or support the robot ({prog.get('error')})")
            return
        armed = ok and ((prog.get("phase") == "ready" and prog.get("ok") is True) or self.dry_run_ops)
        if armed:
            with self.lock:
                e = self.estops.get(rid)
                if e is not None and e["t"] < state["t0"]:
                    self.estops.pop(rid)
                    self.estop_cleared[rid] = time.monotonic()
                    self.say(f"E-STOP on {rid} cleared: Ready re-armed it", robot=rid)
                elif e is not None:
                    self.say(f"E-STOP on {rid} was pressed during Ready: still latched, press Ready again", robot=rid)
            if state.get("pov"):
                self._pov_start(rid)
            if not self.dry_run_ops:
                self._refresh_status(rid)   # speed_cap / heading_hold of the stack that is now running

    def _pov_start(self, rid: str) -> None:
        """fcctl cam start (160x120 @ 5 fps MJPEG, measured 1.3 % of one Pi core, 0.13 Mbit/s) after a
        successful Ready. Runs on the op's worker thread, never the 30 Hz loop; an already running feed is fine."""
        host = self.cfg[rid].host
        if not host or not HOST_RE.fullmatch(host) or host.startswith("-"):
            return
        cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "--", host, "bash ~/fc-v1/robot/fcctl.sh cam start"]
        if self.dry_run_ops:
            cmd = ["echo", "DRY-RUN:", *cmd]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=30, cwd=FINAL_PROJECT)
        except (subprocess.TimeoutExpired, OSError) as e:
            self.say(f"robot {rid}: POV feed not started ({type(e).__name__})")
            return
        out = (p.stdout + p.stderr).strip()
        if p.returncode == 0 or "already running" in out:
            self.say(f"robot {rid}: POV feed on" + (" (dry run)" if self.dry_run_ops else ""), cmd=" ".join(cmd))
        else:
            self.say(f"robot {rid}: POV feed not started (rc {p.returncode}): {out[-200:]}")

    def _refresh_status(self, rid: str) -> None:
        """fcctl status into robot_status without touching the op shown on the cards."""
        host = self.cfg[rid].host
        if not host or not HOST_RE.fullmatch(host) or host.startswith("-"):
            return
        cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "--", host, "bash ~/fc-v1/robot/fcctl.sh status"]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=60, cwd=FINAL_PROJECT)
        except (subprocess.TimeoutExpired, OSError):
            return
        for line in reversed(p.stdout.strip().splitlines()):
            try:
                st = json.loads(line)
            except ValueError:
                continue
            if isinstance(st, dict):
                self.robot_status[rid] = st
                if isinstance(st.get("phase"), str):
                    self._set_phase(rid, st["phase"])
            break

    def _run_op(self, rid: str, state: dict, cmd: list[str], timeout: float) -> None:
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=FINAL_PROJECT)
            state["out"], state["rc"] = (p.stdout + p.stderr)[-4000:], p.returncode
        except subprocess.TimeoutExpired as e:
            state["out"], state["rc"] = f"timeout after {timeout}s\n{(e.stdout or '')}"[-4000:], -1
        except OSError as e:
            state["out"], state["rc"] = str(e), -1
        state["running"] = False
        if state["op"] in ("stop", "bridge_start", "bridge_stop") and not self.dry_run_ops:
            self._set_phase(rid, None)        # changed on the Pi: unknown until the next status
        if state["op"] == "status":
            for line in reversed(state["out"].strip().splitlines()):
                try:
                    self.robot_status[rid] = json.loads(line)
                    if isinstance(self.robot_status[rid].get("phase"), str):
                        self._set_phase(rid, self.robot_status[rid]["phase"])
                    break
                except ValueError:
                    continue
        self.say(f"robot {rid}: {state['op']} -> rc {state['rc']}", out=state["out"][-1500:])

    # ---- state for the UI ----------------------------------------------------------------
    STATE_SCHEMA = 2
    STATIC_KEYS = ("choices", "limits", "tests", "presets", "profiles", "events")   # left out of lite states
    LITE_PARAMS = ("match_s", "ball_decel", "stale_stop_s", "kickoff_x_frac", "countdown_s")

    def config(self) -> dict:
        """What changes rarely: everything the lite /state leaves out (GET /api/config)."""
        with self.lock:
            return clean({"schema": self.STATE_SCHEMA, "params": self.params, "choices": CHOICES, "limits": LIMITS,
                          "tests": TESTS, "presets": self.presets(), "profiles": self.profiles(),
                          "events": list(self.events)[-30:]})

    def state(self, lite: bool = False) -> dict:
        with self.lock:
            now = time.monotonic()
            test = None
            if self.test:
                test = {k: v for k, v in self.test.items() if k not in ("t0", "output", "obs", "sent", "last_sent")}
                test["elapsed"] = round(now - self.test["t0"], 1)
                if "output" in self.test:
                    test["output"] = self.test["output"][-6000:]
            out = clean({
                "t": now, "source": self.source, "mode": self.mode, "test": test, "impl": self.impl_note,
                "run_dir": self.run.dir.name if self.run else None, "last_summary": self.last_summary,
                "field": asdict(self.field), "field_source": self.field_source, "robots": [asdict(r) for r in self.all_robots],
                "active": [r.id for r in self.active], "world": world_json(self.world, now),
                "truth": truth_json(self.sim.truth()) if self.sim else None,
                "cmds": {k: {"vx": c.vx, "vy": c.vy, "wz": c.wz, "tag": c.tag} for k, c in self.cmds.items()},
                "debug": self.debug, "link": self._lstat if self.link else None, "imu_bad": sorted(self.imu_bad),
                "mux_lost": sorted(self.mux_lost),
                "eye": self.eye.status() if self.eye else None, "params": self.params,
                "choices": CHOICES, "limits": LIMITS, "tests": TESTS, "presets": self.presets(),
                "preset": {"name": self.preset_name, "modified": self.preset_modified() if self.preset_name else [],
                           "profile": self.preset_profile, "note": self.preset_note, "stack": self.preset_stack},
                "estop": {rid: estop_view(rid, e, now, self.link is not None, self.sim is not None, ESTOP_RESEND_S)
                          for rid, e in self.estops.items()},
                "stack": {r.id: self.stack_info(r.id) for r in self.active},
                "profiles": self.profiles(), "udp_hosts": self.udp_hosts,
                "ops": self.ops, "robot_status": self.robot_status, "dry_run_ops": self.dry_run_ops,
                "events": list(self.events)[-30:], "score": self.score, "loop": self.loop_stats,
                "play": self.play_state(now),
            })
            # homographies need more than clean()'s 4 decimals (perspective terms are ~1e-4)
            out["play"]["overlay"] = self.eye.overlay() if self.eye else None
            out["schema"] = self.STATE_SCHEMA
            out["avail"] = OPS.availability(self)
            if lite:
                for k in self.STATIC_KEYS:
                    out.pop(k, None)
                out["params"] = {k: self.params[k] for k in self.LITE_PARAMS if k in self.params}
            return out

    def stack_info(self, rid: str) -> dict:
        """The robot's running stack options (from fcctl status) vs the preset, with warnings."""
        st = self.robot_status.get(rid) or {}
        cap = st.get("speed_cap")
        cap = float(cap) if isinstance(cap, (int, float)) and not isinstance(cap, bool) else None
        hh = st.get("heading_hold") if st.get("heading_hold") in STACK_HEADING_HOLD else None
        want, warns = self.preset_stack, []
        if self.link is not None or self.dry_run_ops:   # the sim has no walker cap
            mx = float(self.params.get("max_vx", 0))
            if cap is not None and mx > cap + 1e-9:
                warns.append(f"max_vx {mx:g} > robot speed cap {cap:g}: the walker clips it"
                             " (preset with a higher _stack speed_cap, then down + Ready)")
            elif cap is None and mx > STOCK_SPEED_CAP + 1e-9:
                warns.append(f"max_vx {mx:g} > stock walker cap {STOCK_SPEED_CAP}; robot cap unknown (press status)")
            if "speed_cap" in want and cap is not None and abs(want["speed_cap"] - cap) > 1e-6:
                warns.append(f"preset wants speed cap {want['speed_cap']:g}, robot runs {cap:g}: down, then Ready")
            if "heading_hold" in want and hh is not None and want["heading_hold"] != hh:
                warns.append(f"preset wants heading hold {want['heading_hold']}, robot runs {hh}: down, then Ready")
        return {"speed_cap": cap, "heading_hold": hh, "known": bool(st), "want": want, "warn": warns}

    def play_state(self, now: float) -> dict:
        """What the Play screen shows beyond the world: flow, labels, ticker, banners, lights.
        Caller holds the lock."""
        moving = self.mode in ("play", "match", "warmup") and (self.flow is None or self.flow.phase in ("play", "kickoff",
                                                                                                    "celebrate"))
        lstat = self._lstat if self.link else None
        seen = {rid for rid, o in (self.world.robots.items() if self.world else [])
                if now - o.t <= self.params["stale_stop_s"]}
        active = {r.id for r in self.active}
        robots = {}
        for r in self.all_robots:
            dbg = self.debug.get(r.id) or {}
            hl = health(self.source, r.id in active, r.id in seen, (lstat or {}).get(r.id), self.link is not None,
                        self.ops.get(r.id), self.robot_phase.get(r.id), r.id in self.imu_bad, r.id in self.mux_lost,
                        estopped=r.id in self.estops)
            auto_me = self.auto is not None and r.id == (self.test or {}).get("robot")
            warm_me = self.warm is not None and r.id in self.warm.cfg
            robots[r.id] = {
                "action": self.auto.json(now)["words"] if auto_me else
                self.warm.json(now)["robots"][r.id]["words"] if warm_me else
                action_label(dbg, getattr(self.cmds.get(r.id), "tag", "")),
                "role": dbg.get("role"),
                "health": hl,
                # ready to play: green light AND (sim, or fcctl reported phase ready). "link ok" alone is not ready.
                "ready": hl["level"] == "ok" and (self.sim is not None or self.robot_phase.get(r.id) == "ready"),
                "estop": estop_view(r.id, self.estops[r.id], now, self.link is not None, self.sim is not None,
                                    ESTOP_RESEND_S) if r.id in self.estops else None,
                "phase": self.robot_phase.get(r.id), "playing": r.id in self.playing,
                "phase_age": None if self.phase_age(r.id, now) is None else round(self.phase_age(r.id, now), 1),
                "setup": setup_step(self.source, r.host, self.dry_run_ops, self.robot_phase.get(r.id), self.ops.get(r.id),
                                    r.id in self.estops, str((self.robot_status.get(r.id) or {}).get("detail") or ""),
                                    age=self.phase_age(r.id), linked=bool(((lstat or {}).get(r.id) or {}).get("connected"))),
                "cam": self.cams[r.id].status() if r.id in self.cams else None,
            }
        cam = []
        if self.eye is not None and getattr(self.eye, "paused", False):
            cam = [{"robot": None, "team": None, "level": "bad", "kind": "camera",
                    "text": "CAMERA PAUSED: the overhead picture is frozen - robots stop (stale). Check the phone"}]
        auto = self.auto.json(now) if self.auto else None
        auto_b = [{"robot": self.auto.rid, "team": self.cfg[self.auto.rid].team, "kind": "autoplay", **self.auto.banner}] \
            if self.auto and self.auto.banner else []
        return {"flow": self.flow.json(now, self.params) if self.flow else None, "last_match": self.last_match,
                "auto": auto, "warmup": self.warm.json(now) if self.warm else None,
                "playing": self.playing, "variants": variants([r.id for r in self.active]), "robots": robots,
                "ticker": list(self.ticker.items)[-12:],
                "banners": cam + safety_banners(self.cfg, [r.id for r in self.active], self.estops, self.ops, now,
                                          self.link is not None, self.sim is not None, ESTOP_RESEND_S)
                + auto_b
                + banners(self.cfg, [p for p in self.playing if p not in self.estops], moving, self.cmds, self.imu_bad,
                          self.mux_lost, lstat, self.link is not None),
                "overlay": None,   # set by state() at full precision
                "recording": bool(self.recorder)}

    # ---- mode switch (home screen: Simulation / Play) ----------------------------------------
    restart_hook = None   # main() sets it: callable(source, preset) that replaces this process with the other source

    def busy_reason(self) -> str:
        """Why the app should not restart now ("" = free). Caller holds the lock."""
        if self.mode != "idle":
            return f"{self.mode} is running: Stop (Space) first"
        running = [f"{rid} {o.get('op')}" for rid, o in self.ops.items() if o.get("running")]
        return f"wait for {', '.join(running)}" if running else ""

    def switch_source(self, source: str) -> dict:
        """Restart as the other source (same flags otherwise). Replies first; the restart zeroes every robot,
        closes camera/link cleanly, then replaces the process. Robots on a running bridge also stop by the
        dead-man while the laptop restarts."""
        if source == self.source:
            return {"ok": True, "source": source, "restarting": False}
        if self.restart_hook is None:
            return {"ok": False, "error": "this app was not started by fc.app main(): restart it with the other --source"}
        with self.lock:
            why = self.busy_reason()
            preset = self.preset_name     # reloaded after the restart (unsaved live params are not kept)
        if why:
            return {"ok": False, "error": why}
        self.say(f"switching to {source}: restarting the app" + (f" with preset {preset}" if preset else ""))
        threading.Thread(target=self._restart, args=(source, preset), daemon=True, name="fc-restart").start()
        return {"ok": True, "source": source, "restarting": True, "preset": preset}

    def _restart(self, source: str, preset: str | None = None) -> None:
        time.sleep(0.25)          # let the HTTP reply leave first
        self.close()
        self.restart_hook(source, preset)

    def close(self) -> None:
        self.stop()
        self._stop = True
        for c in self.cams.values():
            c.close()
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
            if path in ("/", "/home"):
                return self._send(200, (UI / "home.html").read_bytes(), "text/html; charset=utf-8")
            if path == "/play":
                return self._send(200, (UI / "play.html").read_bytes(), "text/html; charset=utf-8")
            if path == "/play-old":     # the fc-v1 Play screen (legacy routes), kept until the new one is floor-verified
                return self._send(200, (UI / "play_old.html").read_bytes(), "text/html; charset=utf-8")
            if path in ("/ops", "/index.html"):
                return self._send(200, (UI / "index.html").read_bytes(), "text/html; charset=utf-8")
            m = re.fullmatch(r"/kit/([\w\-]+)\.(css|js)", path)
            if m and (UI / "kit" / f"{m[1]}.{m[2]}").is_file():     # the UI kit (fc/ui/kit): tokens, primitives
                ctype = "text/css; charset=utf-8" if m[2] == "css" else "text/javascript; charset=utf-8"
                return self._send(200, (UI / "kit" / f"{m[1]}.{m[2]}").read_bytes(), ctype)
            if path == "/view.jpg":     # clean camera frame for the Play screen (encoded here, not in the loop)
                jpg = app.eye.view_jpeg() if app.eye else None
                return self._send(200, jpg, "image/jpeg") if jpg else self._send(404, b"no frame", "text/plain")
            m = re.fullmatch(r"/cam/([\w\-]+)\.jpg", path)
            if m:
                feed = app.cams.get(m[1])
                jpg = feed.latest() if feed else None
                return self._send(200, jpg, "image/jpeg") if jpg else self._send(404, b"no feed", "text/plain")
            if path == "/state":
                return self._json(app.state(lite="lite=1" in self.path))
            if path == "/api/ops":
                return self._json(OPS.manifest(app))
            if path == "/api/config":
                return self._json(app.config())
            if path == "/runs":
                return self._json(list_runs(app.runs_root))
            if path == "/ball_track":
                q = dict(p.split("=", 1) for p in self.path.partition("?")[2].split("&") if "=" in p)
                try:
                    since = float(q.get("since", "-inf"))
                except ValueError:
                    since = -math.inf
                pts = [[round(v, 4) for v in p] for p in app.ball_track(since - 1e-3)]
                return self._json({"now": time.monotonic(), "points": [p for p in pts if p[0] > since]})
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
            if self.path == "/api/op":
                res = OPS.call(app, body.get("op"), body.get("args", {}))
                code = 200 if res.get("ok") else {"unknown_op": 404, "bad_args": 400}.get(res.get("code"), 409)
                return self._json(res, code)
            route = {"/action": app.action, "/params": app.set_params, "/robot": app.robot_op,
                     "/preset": app.preset, "/ballcal": app.ballcal, "/robotcal": app.robotcal}.get(self.path)
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


def _without_source(argv: list[str], flags: tuple[str, ...] = ("--source",)) -> list[str]:
    """argv minus --source X / --source=X (a mode switch keeps every other flag); flags: which options to drop."""
    out, skip = [], False
    for x in argv:
        if skip:
            skip = False
        elif x in flags:
            skip = True
        elif not any(x.startswith(f + "=") for f in flags):
            out.append(x)
    return out


def restart_argv(source: str, preset: str | None, argv: list[str]) -> list[str]:
    """fc.app arguments for a mode switch: the other source, the same flags, the loaded preset."""
    return ["--source", source, *_without_source(argv, ("--source", "--preset")), *(["--preset", preset] if preset else [])]


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
    ap.add_argument("--preset", default=None, help="load this preset at start (a mode switch passes the loaded one on)")
    argv = list(sys.argv[1:] if argv is None else argv)
    a = ap.parse_args(argv)
    app = App(a.source, a.robots.split(",") if a.robots else None, video=a.video, camera=a.camera,
              dry_run_ops=a.dry_run_ops, seed=a.seed, no_link=a.no_link, udp_hosts=dict(a.udp_host))
    if a.preset:
        res = app.preset({"op": "load", "name": a.preset})
        if not res.get("ok") or res.get("errors"):
            print(f"preset {a.preset}: {res.get('error') or '; '.join(res.get('errors') or [])}", flush=True)
    app.restart_hook = lambda src, preset=None: os.execv(sys.executable, [sys.executable, "-m", "fc.app",
                                                                         *restart_argv(src, preset, argv)])
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
