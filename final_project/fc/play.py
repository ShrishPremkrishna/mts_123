"""Play screen logic (stdlib): match flow, plain-words labels, decision ticker, stop banners and
per-robot health lights. app.py owns the loop and calls these; nothing here moves a robot.

Match flow (one `match` run with flow=True):

    kickoff --robots at their spots + ball at rest at centre--> countdown --> play
    play --goal--> goal (pause) --> restart (ball seen at rest anywhere in the field) --> countdown
                                    (or kickoff again when restart_mode = kickoff)
    any --pause--> paused --resume--> countdown (if it was play) / where it was
    play --clock out--> full_time

The clock runs only in play. Robots hold still (hard zero) in countdown, goal, restart, paused.
"""
from __future__ import annotations

import math
import time
from collections import deque

from .core import Field, RobotCfg, World

FLOW_DEFAULTS = {
    "countdown_s": 3.0,          # still before play starts / resumes
    "goal_pause_s": 2.0,         # goal flash; robots stop
    "ball_settle_s": 1.0,        # ball seen at rest this long -> play may (re)start
    "ball_rest_m": 0.04,         # "at rest": stays within this distance
    "ball_centre_tol_m": 0.15,   # kickoff: ball this close to the centre spot
    "kickoff_x_frac": 0.27,      # kickoff spots at x = -/+ frac * length (blue west, red east)
}
FLOW_LIMITS = {"countdown_s": (0.0, 10.0), "goal_pause_s": (0.5, 10.0), "ball_settle_s": (0.2, 5.0),
               "ball_rest_m": (0.01, 0.3), "ball_centre_tol_m": (0.05, 0.5), "kickoff_x_frac": (0.1, 0.45)}
RESTART_MODES = ["ball_rest", "kickoff"]
JOBS = ["auto", "attack", "defend"]

# brain debug phase -> short action label (robot card, field chip)
ACTION = {
    "route": "circling the ball", "approach": "chasing the ball", "push": "pushing to goal",
    "line_up": "lining up", "charge": "headbutt!", "backoff": "backing off",
    "guard": "blocking", "return": "back to goal", "clear": "clearing", "unstick": "unsticking",
    "ball_lost": "looking for ball", "pose_stale": "can't see itself",
    "goto": "to kickoff spot", "goto_turn": "facing the field", "goto_done": "in position",
}
# app-made stop tags -> action label
STOP_ACTION = {
    "stale": "can't see itself", "imu_bad": "stopped: IMU", "mux_lost": "stopped: motion link",
    "brain_error": "stopped: error", "benched": "benched", "countdown": "get ready", "goal": "goal!",
    "wait_ball": "waiting for ball", "paused": "paused", "full_time": "full time", "idle": "idle",
}
# brain phase -> ticker sentence (what it decided, in plain words)
TICKER = {
    "route": "circling round the ball to get behind it", "approach": "lining up behind the ball",
    "push": "pushing the ball at the goal", "line_up": "squaring up for a headbutt",
    "charge": "headbutt!", "guard": "guarding the goal mouth", "return": "running back to its goal",
    "clear": "clearing the ball away from its goal", "unstick": "stuck: backing out",
    "ball_lost": "lost the ball: holding still", "pose_stale": "can't see its own tag: stopped",
    "goto": "walking to its kickoff spot", "goto_done": "in position",
}


def name(rid: str) -> str:
    return rid[:1].upper() + rid[1:]


def phase_key(phase) -> str:
    p = str(phase or "")
    if p.startswith("clear"):
        return "clear"
    if p.startswith("ball_lost"):
        return "ball_lost"
    return p


def action_label(dbg: dict | None, tag: str = "") -> str:
    """Short label for what a robot is doing right now."""
    head = (tag or "").split(":")[0]
    if head in STOP_ACTION:
        return STOP_ACTION[head]
    return ACTION.get(phase_key((dbg or {}).get("phase")), "idle")


class MatchFlow:
    def __init__(self, robots: list[RobotCfg], field: Field, now: float, kickoff: bool = True,
                 restart_mode: str = "ball_rest"):
        self.cfg = {r.id: r for r in robots}
        self.field = field
        self.restart_mode = restart_mode
        self.score = {"blue": 0, "red": 0}
        self.touches = 0
        self.clock = 0.0
        self.goals: list[dict] = []
        solo = len(robots) == 1
        self.roles = {r.id: "attack" if solo else "auto" for r in robots}
        self.in_pos: dict[str, bool] = {}
        self.ball_ok = False
        self._ball: deque = deque()
        self.phase = ""
        self.paused_from = ""
        self.t_phase = now
        self._set("kickoff" if kickoff else "countdown", now)

    # ---- phase control ----------------------------------------------------------------
    def _set(self, phase: str, now: float) -> None:
        self.phase, self.t_phase = phase, now
        self._ball.clear()
        self.ball_ok = False
        if phase == "kickoff":
            self.in_pos = {}

    def pause(self, now: float) -> bool:
        if self.phase in ("paused", "full_time"):
            return False
        self.paused_from = self.phase
        self._set("paused", now)
        return True

    def resume(self, now: float) -> bool:
        if self.phase != "paused":
            return False
        self._set("countdown" if self.paused_from in ("play", "countdown") else self.paused_from, now)
        return True

    def reposition(self, now: float) -> None:
        self._set("kickoff", now)

    def force_start(self, now: float) -> bool:
        """Operator: start now without waiting for positions / the ball."""
        if self.phase in ("kickoff", "restart"):
            self._set("countdown", now)
            return True
        return False

    def spot(self, rid: str) -> tuple[float, float, float]:
        L = self.field.length
        frac = self.frac
        blue = self.cfg[rid].team == "blue"
        return (-frac * L if blue else frac * L, 0.0, 0.0 if blue else math.pi)

    frac = FLOW_DEFAULTS["kickoff_x_frac"]

    def command(self, rid: str) -> tuple[str, object]:
        """("stop", tag) | ("goto", (x, y, yaw)) | ("brain", role_mode) for a playing robot."""
        if rid not in self.cfg:
            return "stop", "benched"
        ph = self.phase
        if ph == "kickoff":
            return "goto", self.spot(rid)
        if ph == "play":
            return "brain", self.roles.get(rid, "auto")
        return "stop", {"countdown": "countdown", "goal": "goal", "restart": "wait_ball",
                        "paused": "paused", "full_time": "full_time"}.get(ph, "idle")

    # ---- per tick -------------------------------------------------------------------------
    def _ball_at_rest(self, world: World | None, now: float, p: dict, centre: bool) -> bool:
        b = world.ball if world else None
        f = self.field
        if (b is None or now - b.t > 0.4 or abs(b.x) > f.length / 2 - f.ball_radius
                or abs(b.y) > f.width / 2 - f.ball_radius
                or (centre and math.hypot(b.x, b.y) > p["ball_centre_tol_m"])):
            self._ball.clear()
            return False
        self._ball.append((now, b.x, b.y))
        while math.hypot(self._ball[0][1] - b.x, self._ball[0][2] - b.y) > p["ball_rest_m"]:
            self._ball.popleft()
        return now - self._ball[0][0] >= p["ball_settle_s"]

    def update(self, now: float, dt: float, world: World | None, debug: dict, goals: list[str],
               params: dict) -> list[str]:
        """Advance the flow. Returns event names: "goal:<team>", "play", "full_time"."""
        p = {**FLOW_DEFAULTS, **params}
        self.frac = p["kickoff_x_frac"]
        out = []
        ph, el = self.phase, now - self.t_phase
        if ph == "play":
            for team in goals:
                self.score[team] += 1
                self.goals.append({"team": team, "clock": round(self.clock, 1), "n": len(self.goals) + 1})
                out.append(f"goal:{team}")
            if goals:
                self._set("goal", now)
                return out
            self.clock += dt
            if self.clock >= p["match_s"]:
                self._set("full_time", now)
                out.append("full_time")
        elif ph == "goal":
            if el >= p["goal_pause_s"]:
                self._set("kickoff" if p.get("restart_mode", self.restart_mode) == "kickoff" else "restart", now)
        elif ph == "kickoff":
            for rid in self.cfg:
                self.in_pos[rid] = bool((debug.get(rid) or {}).get("done"))
            self.ball_ok = self._ball_at_rest(world, now, p, centre=True)
            if self.ball_ok and all(self.in_pos.values()):
                self._set("countdown", now)
        elif ph == "restart":
            self.ball_ok = self._ball_at_rest(world, now, p, centre=False)
            if self.ball_ok:
                self._set("countdown", now)
        elif ph == "countdown":
            if el >= p["countdown_s"]:
                self._set("play", now)
                out.append("play")
        return out

    def waiting(self) -> list[str]:
        """What the flow is waiting for, in plain words."""
        if self.phase == "kickoff":
            w = [f"{name(r)} to its spot" for r, ok in self.in_pos.items() if not ok]
            return w + ([] if self.ball_ok else ["ball at the centre spot"])
        if self.phase == "restart":
            return [] if self.ball_ok else ["ball placed and still"]
        return []

    def json(self, now: float, params: dict) -> dict:
        p = {**FLOW_DEFAULTS, **params}
        el = now - self.t_phase
        return {"phase": self.phase, "phase_s": round(el, 2), "paused_from": self.paused_from,
                "clock": round(self.clock, 2), "clock_left": round(max(0.0, p["match_s"] - self.clock), 2),
                "match_s": p["match_s"], "score": dict(self.score), "touches": self.touches,
                "goals": self.goals[-10:], "roles": dict(self.roles), "playing": list(self.cfg),
                "in_pos": dict(self.in_pos), "ball_ok": self.ball_ok, "waiting": self.waiting(),
                "countdown_left": round(max(0.0, p["countdown_s"] - el), 2) if self.phase == "countdown" else None,
                "goal_left": round(max(0.0, p["goal_pause_s"] - el), 2) if self.phase == "goal" else None,
                "restart_mode": p.get("restart_mode", self.restart_mode)}


class Ticker:
    """Plain-words feed of real brain decisions: role flips (with the travel-time reason) and
    debounced phase changes."""

    def __init__(self, debounce_s: float = 0.4, gap_s: float = 1.2, maxlen: int = 24):
        self.items: deque = deque(maxlen=maxlen)
        self.debounce_s, self.gap_s = debounce_s, gap_s
        self.role: dict[str, str] = {}
        self.cand: dict[str, tuple[str, float]] = {}
        self.said: dict[str, tuple[str, float]] = {}
        self.n = 0

    def clear(self) -> None:
        self.role.clear()
        self.cand.clear()
        self.said.clear()

    def add(self, rid: str | None, team: str | None, text: str, kind: str = "info") -> dict:
        self.n += 1
        item = {"n": self.n, "t": time.strftime("%H:%M:%S"), "robot": rid, "team": team, "text": text, "kind": kind}
        self.items.append(item)
        return item

    def update(self, now: float, debug: dict, cfg: dict[str, RobotCfg], playing) -> list[dict]:
        new = []
        for rid in playing:
            d = debug.get(rid) or {}
            team = cfg[rid].team if rid in cfg else None
            role = d.get("role")
            if role in ("attack", "defend") and role != self.role.get(rid):
                new.append(self.add(rid, team, self._role_text(rid, role, d), "role"))
                self.role[rid] = role
            key = phase_key(d.get("phase"))
            if key not in TICKER:
                continue
            c = self.cand.get(rid)
            if not c or c[0] != key:
                self.cand[rid] = (key, now)
                continue
            said = self.said.get(rid)
            if (now - c[1] >= self.debounce_s and (not said or said[0] != key)
                    and (not said or now - said[1] >= self.gap_s)):
                self.said[rid] = (key, now)
                new.append(self.add(rid, team, f"{name(rid)}: {TICKER[key]}", "phase"))
        return new

    @staticmethod
    def _role_text(rid: str, role: str, d: dict) -> str:
        my, opp = d.get("my_cost"), d.get("opp_cost")
        n = name(rid)
        if isinstance(my, (int, float)) and isinstance(opp, (int, float)):
            gap = opp - my
            if role == "attack":
                return (f"{n} attacks: reaches the ball {gap:.1f} s sooner" if gap > 0.05
                        else f"{n} attacks: nearest to the ball by its own goal")
            return f"{n} drops back to guard: the other robot is {-gap:.1f} s closer" if gap < -0.05 \
                else f"{n} drops back to guard the goal"
        return f"{n} attacks" if role == "attack" else f"{n} guards the goal"


HOLD_WORDS = {"imu_stale": "balance sensor data stopped", "imu_tilt": "balance sensor reports a tilt",
              "mux_unseen": "motion channel not found", "mux_not_subscribed": "motion channel not listening",
              "no_cmd_vel_publisher": "motion channel dropped", "mux_stalled": "motion channel stalled"}


def banners(cfg: dict[str, RobotCfg], playing, moving: bool, cmds: dict, imu_bad, mux_lost,
            lstat: dict | None, has_link: bool) -> list[dict]:
    """Plain-words reasons a robot that should be playing is stopped or held."""
    out = []
    for rid in playing:
        n, team = name(rid), cfg[rid].team
        L = (lstat or {}).get(rid) or {}
        tag = (getattr(cmds.get(rid), "tag", "") or "").split(":")[0]
        hold = L.get("hold")
        if rid in imu_bad:
            text, lvl = f"{n} stopped: its balance sensor (IMU) reports bad data", "bad"
        elif rid in mux_lost:
            text, lvl = f"{n} stopped: the robot's motion channel dropped out", "bad"
        elif has_link and L and not L.get("connected"):
            text, lvl = f"{n}: no link to the robot", "bad"
        elif hold:
            text, lvl = f"{n} held by the robot: {HOLD_WORDS.get(str(hold).split('=')[0], hold)}", "warn"
        elif moving and tag in ("stale", "pose_stale"):
            text, lvl = f"{n} paused: lost sight of its tag", "warn"
        elif moving and tag == "brain_error":
            text, lvl = f"{n} stopped: software error (see Ops)", "bad"
        else:
            continue
        out.append({"robot": rid, "team": team, "level": lvl, "text": text})
    if moving and any((getattr(cmds.get(r), "tag", "") or "") == "ball_lost" for r in playing):
        out.append({"robot": None, "team": None, "level": "warn", "text": "Ball out of sight: robots holding still"})
    return out


def health(source: str, active: bool, seen: bool, L: dict | None, has_link: bool, op: dict | None,
           phase: str | None, imu_bad: bool, mux_lost: bool) -> dict:
    """One traffic light per robot: level ok|warn|bad|off and a short text. Never claims ready
    without evidence: "Ready" needs fcctl phase ready; otherwise the best observed state."""
    if not active:
        return {"level": "off", "text": "not in this session"}
    if imu_bad:
        return {"level": "bad", "text": "IMU bad"}
    if mux_lost:
        return {"level": "bad", "text": "motion link lost"}
    if op and op.get("op") == "ready":
        prog = op.get("progress") or {}
        if op.get("running"):
            el = prog.get("elapsed")
            return {"level": "warn", "text": f"getting ready: {prog.get('phase') or 'starting'}"
                    + (f" {el:.0f} s" if isinstance(el, (int, float)) else "")}
        if op.get("rc") not in (0, None) or prog.get("ok") is False or prog.get("error"):
            return {"level": "bad", "text": f"ready failed: {prog.get('error') or 'rc ' + str(op.get('rc'))}"[:60]}
    if phase == "fault":
        return {"level": "bad", "text": "robot fault"}
    if source == "sim":
        return {"level": "ok" if seen else "warn", "text": "simulated" if seen else "not on the field"}
    if not has_link:
        return {"level": "off", "text": "link off (--no-link)"}
    if not L or not L.get("connected"):
        return {"level": "off", "text": "not connected"}
    if L.get("hold"):
        return {"level": "warn", "text": f"held: {L['hold']}"[:40]}
    if not seen:
        return {"level": "warn", "text": "not seen by camera"}
    if ((L.get("imu") or {}).get("level")) == "warn":
        return {"level": "warn", "text": "IMU warning"}
    if phase and phase != "ready":
        return {"level": "warn", "text": phase}
    return {"level": "ok", "text": "ready" if phase == "ready" else "link ok"}
