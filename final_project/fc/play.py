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

from .core import Cmd, Field, RobotCfg, World

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


AUTO_DEFAULTS = {
    "reset_vx": 0.15,        # walking speed while bringing the ball back (slow, controlled push)
    "stage_gap_m": 0.12,     # stand this surface gap behind the ball before pushing it
    "centre_tol_m": 0.15,    # ball this close to the centre spot = back at the centre
    "backoff_m": 0.40,       # then back off to this surface gap before attacking again
    "backoff_vx": 0.12,
    "line_lat_m": 0.09,      # pushing: ball this far off the robot's push line -> line up again
    "edge_m": 0.12,          # staging points stay this far inside the field (the brain's field_margin)
    "lost_s": 1.0,           # ball older than this = not seen
    "out_s": 4.0,            # ball in a goal / off the field / not seen this long -> "please roll it out"
    "goal_pause_s": 2.0,     # after a goal: stand still while the ball settles
    "countdown_s": 1.5,      # still before each attack
    "settle_s": 0.8,         # ball at rest = within rest_m this long
    "settle_max_s": 4.0,     # ...or give up waiting for rest after this
    "rest_m": 0.04,
    "attack_max_s": 90.0,    # no goal by then: bring the ball back and start again
    "reset_max_s": 60.0,     # ball not at the centre by then: attack from where it is
}
FAR_VX = 9.9                              # goto speed cap meaning "the live max_vx" (the app takes the smaller)
AUTO_RESETS = ("centre", "none")          # after a goal: dribble the ball to the centre spot | attack from where it is
AUTO_AFTER_ROLL = ("centre", "attack")    # ball rolled out of a goal by a person: to the centre first | attack at once
# autoplay phase -> plain words (Play screen, ctl)
AUTO_WORDS = {"countdown": "get ready", "attack": "attacking", "goal": "goal!", "settle": "waiting for the ball to stop",
              "stage": "lining up behind the ball", "push": "bringing the ball to the centre",
              "backoff": "backing off", "wait_ball": "waiting for the ball"}


def _unit(x: float, y: float) -> tuple[float, float]:
    d = math.hypot(x, y)
    return (x / d, y / d) if d > 1e-9 else (1.0, 0.0)


def _median(v: list[float]) -> float | None:
    s = sorted(v)
    n = len(s)
    return None if not n else s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class AutoPlay:
    """Hands-off solo demo loop for one robot: attack -> goal -> bring the ball back -> attack ...

        countdown -> attack --goal--> goal (still) -> settle --ball at the centre--> backoff -> countdown
                                                       |                  ^
                                                       +--> stage -> push +   (dribble to the centre, slowly)
        ball in a goal / off the field / not seen (any reset phase) -> wait_ball ("please roll it out" after out_s,
        no time-out) -> settle (or straight to countdown with after_roll=attack)

    Pure logic: update() advances the phases from the World and the goal events; `want` is what the robot does
    now: ("attack", team) = the brain attacks the goal that `team` attacks; ("goto", (x, y, yaw, vmax)) = the
    brain's goto; ("app", Cmd) = an app-made command (back off); ("stop", tag) = hold still. The app turns it
    into brain params, so this needs no new brain capability. With alternate, each cycle attacks the goal away
    from the one just scored on: the robot finishes the reset behind the ball facing that goal (no walk round).
    """

    def __init__(self, cfg: RobotCfg, field: Field, now: float, minutes: float = 10.0, alternate: bool = True,
                 reset: str = "centre", after_roll: str = "centre", **opts):
        self.rid, self.home = cfg.id, cfg.team
        self.f = field
        self.p = {**AUTO_DEFAULTS, **{k: v for k, v in opts.items() if k in AUTO_DEFAULTS}}
        self.minutes, self.alternate, self.reset, self.after_roll = minutes, alternate, reset, after_roll
        self.t0 = now
        self.team = cfg.team          # whose attack goal this cycle targets (blue attacks east, +x)
        self.next_team = cfg.team
        self.cycles: list[dict] = []
        self.cur: dict = {"wait_s": 0.0, "interventions": 0, "how": "start"}
        self.interventions = self.attack_timeouts = 0
        self.unreachable = 0          # stagings in a row that found no reachable point behind the ball
        self.banner: dict | None = None
        self.u: tuple[float, float] | None = None     # push direction (frozen during a push)
        self.side = 0.0                               # staging: which side of the ball to walk round
        self.wait_t: float | None = None              # wait_ball: since when
        self.wait_why = ""
        self.out_long = False                         # the "please roll it out" banner has been shown
        self.reset_t: float | None = None             # start of the current dribble (reset_max_s)
        self.stuck_at: tuple[float, float] = (0.0, 0.0)   # where the ball was when the robot gave up on it
        self._rest: deque = deque()
        self.want: tuple[str, object] = ("stop", "countdown")
        self.phase, self.t_phase = "", now
        self._set("countdown", now)

    # ---- helpers ----------------------------------------------------------------------------
    def _set(self, phase: str, now: float) -> None:
        self.phase, self.t_phase = phase, now
        self._rest.clear()

    def att_sign(self, team: str | None = None) -> float:
        return 1.0 if (team or self.team) == "blue" else -1.0

    def _ball(self, world: World | None, now: float):
        b = world.ball if world else None
        return b if b is not None and now - b.t <= self.p["lost_s"] else None

    def _where(self, b) -> str:
        """field | goal_east | goal_west | off | unseen."""
        if b is None:
            return "unseen"
        hl, hw, gw = self.f.length / 2, self.f.width / 2, self.f.goal_width / 2
        if abs(b.x) > hl - 0.02 and abs(b.y) < gw + 0.05:
            return "goal_east" if b.x > 0 else "goal_west"
        if abs(b.x) > hl + 0.05 or abs(b.y) > hw + 0.05:
            return "off"
        return "field"

    def _at_rest(self, b, now: float) -> bool:
        if b is None:
            self._rest.clear()
            return False
        self._rest.append((now, b.x, b.y))
        while math.hypot(self._rest[0][1] - b.x, self._rest[0][2] - b.y) > self.p["rest_m"]:
            self._rest.popleft()
        return now - self._rest[0][0] >= self.p["settle_s"]

    def _contact(self) -> float:
        return self.f.ball_radius + self.f.robot_radius

    def _inside(self, x: float, y: float) -> bool:
        m = self.p["edge_m"]
        return abs(x) <= self.f.length / 2 - m and abs(y) <= self.f.width / 2 - m

    def _clamp(self, x: float, y: float) -> tuple[float, float]:
        hx, hy = self.f.length / 2 - self.p["edge_m"], self.f.width / 2 - self.p["edge_m"]
        return max(-hx, min(hx, x)), max(-hy, min(hy, y))

    def push_dir(self, bx: float, by: float) -> tuple[float, float] | None:
        """Direction to push the ball toward the centre spot whose staging point is inside the field (the
        straight line first, then up to 75 deg either way: still toward the centre). None: no reachable point behind
        the ball (a ball on a wall, in a corner or on the goal line)."""
        u0 = _unit(-bx, -by)
        d = self._contact() + self.p["stage_gap_m"]
        for deg in (0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75):
            c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            u = (c * u0[0] - s * u0[1], s * u0[0] + c * u0[1])
            away = (abs(bx) > 0.1 and u[0] * bx > 0.02) or (abs(by) > 0.1 and u[1] * by > 0.02)
            if not away and self._inside(bx - u[0] * d, by - u[1] * d):   # never pushes it further out on an axis
                return u
        return None

    def centred(self, b, slack: float = 0.0) -> bool:
        return b is not None and math.hypot(b.x, b.y) <= self.p["centre_tol_m"] + slack

    def _start_attack_soon(self, now: float, how: str) -> None:
        self.cur["how"] = how
        self._set("countdown", now)

    def _wait(self, now: float, why: str) -> None:
        if self.phase != "wait_ball":
            self.wait_t, self.out_long = now, False
            self._set("wait_ball", now)
        self.wait_why = why

    # ---- per tick ---------------------------------------------------------------------------------
    def update(self, now: float, world: World | None, goals: list[str]) -> list[str]:
        """Advance. Returns events: "goal:<east|west>", "attack", "phase:<name>", "rolled", "done"."""
        out: list[str] = []
        before = self.phase
        p, b = self.p, self._ball(world, now)
        me = world.robots.get(self.rid) if world else None
        pose = me.pose if me is not None and now - me.t <= 0.6 else None
        where = self._where(b)
        el = now - self.t_phase
        self.banner = None
        if self.minutes and now - self.t0 >= self.minutes * 60:
            return ["done"]
        if self.phase == "attack":
            if goals:
                side = "east" if goals[0] == "blue" else "west"   # the referee names the team the goal counts for
                own = (side == "east") != (self.team == "blue")
                self.cur.update(goal=side, own=own, attack_s=round(el, 1), t_goal=now)
                self.next_team = ("red" if side == "east" else "blue") if self.alternate else self.home
                self.unreachable = 0
                out.append(f"goal:{side}")
                self._set("goal", now)
            elif el > p["attack_max_s"]:
                self.attack_timeouts += 1
                self.cur.update(goal=None, attack_s=round(el, 1), t_goal=now)
                self._set("settle", now)
            else:
                self.want = ("attack", self.team)
                if b is None:
                    self.banner = {"level": "warn", "text": "Ball not seen: robot holding still"}
        elif self.phase == "goal":
            self.want = ("stop", "goal")
            if el >= p["goal_pause_s"]:
                self._set("settle", now)
        elif self.phase == "countdown":
            self.want = ("stop", "countdown")
            if el >= p["countdown_s"]:
                self._begin_attack(now)
                out.append("attack")
        elif where != "field" and self.phase in ("settle", "stage", "push", "backoff", "wait_ball"):
            self.want = ("stop", "wait_ball")
            self._wait(now, where)
        elif self.phase == "wait_ball" and self.wait_why == "stuck" and not self._unstuck(b):
            self.want = ("stop", "wait_ball")
        elif self.phase == "wait_ball":    # the ball is back in the field
            if self.out_long:
                self.interventions += 1
                self.cur["interventions"] += 1
                out.append("rolled")
            self.cur["wait_s"] += now - (self.wait_t or now)
            self.wait_t = None
            self.reset_t = None
            if self.out_long and self.after_roll == "attack":
                self._start_attack_soon(now, "rolled")
            else:
                self._set("settle", now)
            self.want = ("stop", "wait_ball")
        elif self.phase == "settle":
            self.want = ("stop", "settle")
            if self._at_rest(b, now) or el > p["settle_max_s"]:
                self._after_settle(now, b, pose)
        elif pose is None:
            self.want = ("stop", "stale")
            self.banner = {"level": "warn", "text": "Robot not seen: waiting"}
        elif self.phase == "stage":
            self._stage(now, b, pose)
        elif self.phase == "push":
            self._push(now, b, pose)
        elif self.phase == "backoff":
            self._backoff(now, b, pose, el)
        if self.phase == "wait_ball":
            self._wait_banner(now)
        if self.phase != before:
            out.append(f"phase:{self.phase}")
        return out

    def _begin_attack(self, now: float) -> None:
        c = self.cur
        team = c.pop("force_team", None) or self.next_team
        if "attack_s" in c:          # the previous attack ended (goal or time-out): its cycle is complete
            self.cycles.append(self._cycle(c, now))
        self.cur = {"wait_s": 0.0, "interventions": 0, "how": "", "team": team, "t_attack": now}
        self.team = team
        self.reset_t = None
        self.want = ("attack", team)
        self._set("attack", now)

    def _unstuck(self, b) -> bool:
        """A ball the robot gave up on counts as back once someone moved it or it became reachable."""
        return b is not None and (math.hypot(b.x - self.stuck_at[0], b.y - self.stuck_at[1]) > 0.25
                                  or self.push_dir(b.x, b.y) is not None)

    def _cycle(self, c: dict, now: float | None) -> dict:
        """One finished (or, with now=None, unfinished) cycle: its attack, the goal and the reset after it."""
        return {"n": len(self.cycles) + 1, "team": c.get("team"), "goal": c.get("goal"), "own": bool(c.get("own")),
                "attack_s": c.get("attack_s"),
                "reset_s": None if now is None or c.get("t_goal") is None else round(now - c["t_goal"], 1),
                "wait_s": round(c.get("wait_s", 0.0), 1), "interventions": c.get("interventions", 0),
                "reset": c.get("how") or None}

    def _after_settle(self, now: float, b, pose) -> None:
        if b is None:
            return
        if self.reset == "none":
            return self._start_attack_soon(now, "none")
        if self.centred(b, 0.05):    # a little slack here: no push-settle dithering at the tolerance
            return self._go_backoff(now, "centre")
        if self.reset_t is None:
            self.reset_t = now
        elif now - self.reset_t > self.p["reset_max_s"]:
            self.reset_t = None
            return self._start_attack_soon(now, "reset_timeout")
        if self.push_dir(b.x, b.y) is None:
            self.unreachable += 1
            if self.unreachable >= 2:       # the attack from there did not score either: ask for help
                self.unreachable = 0
                self.wait_t, self.out_long = now - self.p["out_s"], False
                self._set("wait_ball", now)
                self.wait_why, self.stuck_at = "stuck", (b.x, b.y)
                return
            # can't get behind it toward the centre (wall / goal mouth): attack the nearer goal from where it is
            self.cur["force_team"] = "blue" if b.x > 0 else "red"
            return self._start_attack_soon(now, "unreachable")
        self.side = 0.0
        self._set("stage", now)

    def _stage(self, now: float, b, pose) -> None:
        u = self.push_dir(b.x, b.y)
        if self.centred(b):
            return self._go_backoff(now, "centre")
        if u is None or now - (self.reset_t or now) > self.p["reset_max_s"]:
            return self._set("settle", now)
        c = self._contact()
        n = (-u[1], u[0])
        rx, ry = pose.x - b.x, pose.y - b.y
        a, l = rx * u[0] + ry * u[1], rx * n[0] + ry * n[1]
        if not self.side or abs(l) > c + 0.25:
            self.side = 1.0 if l >= 0 else -1.0
        yaw = math.atan2(u[1], u[0])
        d = c + self.p["stage_gap_m"]
        vmax = FAR_VX if math.hypot(rx, ry) > c + 0.45 else self.p["reset_vx"] * 1.5   # slow only near the ball
        if a > -0.5 * c:      # not behind the ball yet: walk round it (beside it, then behind it), never through it
            off = c + 0.2
            fwd = max(0.0, min(a, c)) if abs(l) < c + 0.12 else -d
            tgt = self._clamp(b.x + n[0] * self.side * off + u[0] * fwd, b.y + n[1] * self.side * off + u[1] * fwd)
            self.want = ("goto", (tgt[0], tgt[1], None, vmax))
            return
        sx, sy = b.x - u[0] * d, b.y - u[1] * d
        self.want = ("goto", (sx, sy, yaw, vmax))
        if math.hypot(pose.x - sx, pose.y - sy) < 0.08 and abs(_wrap(pose.yaw - yaw)) < math.radians(12):
            self.u, self.unreachable = u, 0
            self._set("push", now)

    def _push(self, now: float, b, pose) -> None:
        # follow the ball -> centre line as the ball drifts; a staging rotated off it (wall) keeps its own line
        u = _unit(-b.x, -b.y)
        if self.u and u[0] * self.u[0] + u[1] * self.u[1] < math.cos(math.radians(20)):
            u = self.u
        n = (-u[1], u[0])
        c = self._contact()
        if self.centred(b):
            return self._set("settle", now)
        rx, ry = b.x - pose.x, b.y - pose.y
        ahead, lat = rx * u[0] + ry * u[1], rx * n[0] + ry * n[1]
        left = -(b.x * u[0] + b.y * u[1])            # how far the ball still is from the centre's foot point
        if abs(lat) > self.p["line_lat_m"] or ahead < 0.5 * c or ahead > c + 0.5:
            return self._set("stage", now)           # lost the ball off the line: line up again
        if left < 0.03:
            return self._set("settle", now)          # passed the point nearest the centre
        tx, ty = b.x + u[0] * (left - c), b.y + u[1] * (left - c)   # where the robot stands with the ball at the foot point
        self.want = ("goto", (tx, ty, math.atan2(u[1], u[0]), self.p["reset_vx"]))
        if math.hypot(pose.x - tx, pose.y - ty) < 0.05:
            self._set("settle", now)

    def _go_backoff(self, now: float, how: str) -> None:
        self.cur["how"] = how
        self.reset_t = None
        self._set("backoff", now)

    def _backoff(self, now: float, b, pose, el: float) -> None:
        if b is None:
            return
        gap = math.hypot(pose.x - b.x, pose.y - b.y) - self._contact()
        fwd = (b.x - pose.x) * math.cos(pose.yaw) + (b.y - pose.y) * math.sin(pose.yaw)
        sign = -1.0 if fwd >= 0 else 1.0             # step away from the ball: back if it is in front
        hx, hy = pose.x + sign * 0.2 * math.cos(pose.yaw), pose.y + sign * 0.2 * math.sin(pose.yaw)
        if gap >= self.p["backoff_m"] or el > 6.0 or not self._inside(hx, hy):
            self.want = ("stop", "backoff")
            return self._start_attack_soon(now, self.cur.get("how") or "centre")
        self.want = ("app", Cmd(sign * self.p["backoff_vx"], 0.0, 0.0, "ap_backoff"))

    def _wait_banner(self, now: float) -> None:
        long = now - (self.wait_t or now) >= self.p["out_s"]
        self.out_long = self.out_long or long
        why = self.wait_why
        if not long:
            self.banner = {"level": "warn", "text": "Waiting for the ball…"}
        elif why.startswith("goal_"):
            self.banner = {"level": "bad", "text": f"Ball is in the {why[5:]} goal: please roll it out onto the field"}
        elif why == "off":
            self.banner = {"level": "bad", "text": "Ball is off the field: please roll it back onto the field"}
        elif why == "stuck":
            self.banner = {"level": "bad", "text": "Robot can't get the ball out: please roll it onto the field"}
        else:
            self.banner = {"level": "bad", "text": "Ball not seen: please put it on the field"}

    # ---- reporting ----------------------------------------------------------------------------------
    def summary(self, now: float) -> dict:
        log = self.cycles + ([self._cycle(self.cur, None)] if "attack_s" in self.cur else [])
        goals = [c for c in log if c.get("goal")]
        att = [c["attack_s"] for c in goals if isinstance(c.get("attack_s"), (int, float))]
        res = [c["reset_s"] for c in log if isinstance(c.get("reset_s"), (int, float))]
        return {"goals": len([c for c in goals if not c["own"]]), "own_goals": len([c for c in goals if c["own"]]),
                "cycles": len(log), "interventions": self.interventions,
                "attack_timeouts": self.attack_timeouts,
                "time_per_goal_s": _median(att), "reset_s": _median(res),
                "time_s": round(now - self.t0, 1), "cycle_log": log[-50:],
                "alternate": self.alternate, "reset": self.reset, "after_roll": self.after_roll,
                "minutes": self.minutes}

    def json(self, now: float) -> dict:
        s = self.summary(now)
        last = s.pop("cycle_log")[-1:]
        return {**s, "phase": self.phase, "words": AUTO_WORDS.get(self.phase, self.phase),
                "phase_s": round(now - self.t_phase, 1), "attacking": self.team, "banner": self.banner,
                "last": last[0] if last else None,
                "left_s": round(max(0.0, self.minutes * 60 - (now - self.t0)), 0) if self.minutes else None}


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


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
              "imu_frozen": "balance sensor values frozen",
              "imu_stall": "balance sensor paused (standing, resumes by itself)",
              "imu_stall_long": "balance sensor stopped",
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


def down_failed(op: dict | None) -> bool:
    """fcctl down finished without success (e.g. "still standing": lie-down not confirmed)."""
    return bool(op and op.get("op") == "down" and not op.get("running") and op.get("rc") not in (0, None))


DOWN_FAIL_TEXT = "STILL STANDING? down refused: use R3 or support the robot"


def estop_view(rid: str, e: dict, now: float, has_link: bool, sim: bool, resend_s: float) -> dict:
    """E-stop latch -> {level, text, confirmed, by, wall} for the UI. Never clears itself."""
    n, why = name(rid), e.get("reason") or ""
    if e.get("by") == "robot":
        text, lvl = f"{n} E-STOPPED by the robot ({why}): legs off. Re-arm = Ready", "bad"
    elif e.get("confirmed"):
        text, lvl = f"{n} E-STOPPED: legs off, latched{' (sim)' if sim else ''}. Re-arm = Ready", "bad"
    elif not has_link:
        text, lvl = f"{n} E-STOP: link off, nothing was sent. Use R3", "bad"
    elif now - e["t"] < resend_s:
        text, lvl = f"{n} E-STOP sent: waiting for the robot to confirm", "bad"
    else:
        text, lvl = f"{n} E-STOP NOT CONFIRMED by the robot: use R3 now", "bad"
    return {"level": lvl, "text": text, "confirmed": bool(e.get("confirmed")), "by": e.get("by"), "wall": e.get("wall")}


def safety_banners(cfg: dict[str, RobotCfg], active, estops: dict, ops: dict, now: float, has_link: bool, sim: bool,
                   resend_s: float) -> list[dict]:
    """Banners that outrank play: latched e-stops and a refused down, for every active robot."""
    out = []
    views = {rid: estop_view(rid, estops[rid], now, has_link, sim, resend_s) for rid in active if rid in estops}
    # every robot latched and confirmed the same way: one banner instead of one per robot
    same = len(views) > 1 and len(views) == len(active) and all(v["confirmed"] and v["by"] != "robot" for v in views.values())
    if same:
        out.append({"robot": None, "team": None, "level": "bad", "kind": "estop",
                    "text": f"ALL ROBOTS E-STOPPED: legs off, latched{' (sim)' if sim else ''}. Re-arm = Ready"})
    for rid in active:
        if rid in views and not same:
            out.append({"robot": rid, "team": cfg[rid].team, "level": "bad", "kind": "estop", "text": views[rid]["text"]})
        if down_failed(ops.get(rid)):
            out.append({"robot": rid, "team": cfg[rid].team, "level": "bad", "kind": "down",
                        "text": f"{name(rid)}: {DOWN_FAIL_TEXT}"})
    return out


SETUP_STEPS = ("off", "homed", "standing", "ready")      # power-on order of fcctl phases
READY_PHASES = ("homed", "standing", "ready")             # stack running: fcctl ready stands / links / re-arms, never homes
PHASE_FRESH_S = 15.0     # a robot.ready decision needs an fcctl phase at most this old (Pi reboot / battery swap)
HOME_ASK = ("Is the robot ON THE STAND? The stack is off, so this HOMES the legs (they move to their zero pose). "
            "Then it stops, ready to stand.")


def phase_fresh(phase: str | None, age: float | None) -> bool:
    return phase is not None and age is not None and 0 <= age <= PHASE_FRESH_S


def ready_refusal(phase: str | None, age: float | None, home: bool) -> str | None:
    """Why robot.ready must not run now (None = go). fcctl ready HOMES the legs when the stack is off, so the
    decision needs a fresh phase: home=True only from a fresh "off" (operator confirmed: robot on the stand),
    home=False only from a fresh running-stack phase."""
    if not phase_fresh(phase, age):
        old = "unknown" if phase is None or age is None else f"{phase!r} is {age:.0f} s old"
        return f"phase {old}: run robot.status first (Ready may HOME the legs)"
    if home and phase != "off":
        return f"phase is {phase!r}, not off: nothing to home (status changed; press again)"
    if not home and phase == "off":
        return "stack is off: Ready would HOME the legs; robot on the stand, then robot.ready with home=true"
    if not home and phase not in READY_PHASES:
        return f"phase {phase!r}: check again (robot.status) or stop the stack"
    return None


def setup_step(source: str, host: str, dry_run: bool, phase: str | None, op: dict | None, estopped: bool,
               detail: str = "", age: float | None = None, linked: bool = False) -> dict:
    """The robot's next setup step, for the one setup button and for agents:
    {step, label, op (registry name or None), args (extra op args), ask (confirm question or None), reason, busy,
    done (0-3 of off -> homed -> standing -> ready)}. Never trusts an old phase: a robot.ready step needs an fcctl
    phase younger than PHASE_FRESH_S (the UI runs robot.status first, then asks), and a "ready" phase with no
    link acks is shown as disconnected, never Ready."""
    fresh = phase_fresh(phase, age)
    done = SETUP_STEPS.index(phase) if phase in SETUP_STEPS else 0     # last known progress (display only)
    out = {"done": done, "busy": False, "ask": None, "reason": "", "args": {}}
    if source == "sim":
        return {**out, "step": "sim", "label": "Simulated", "op": None, "done": 3}
    if not host and not dry_run:
        return {**out, "step": "none", "label": "No host", "op": None, "reason": "no host in robots.json"}
    if op and op.get("running"):
        label = {"ready": {"homed": "Standing up…", "standing": "Starting link…"}.get(phase, "Homing…"),
                 "down": "Lying down…", "status": "Checking…", "stop": "Stopping…"}.get(op.get("op"), f"{op.get('op')}…")
        return {**out, "step": "busy", "label": label, "op": None, "busy": True}
    if op and op.get("op") == "status" and op.get("rc") not in (0, None):
        return {**out, "step": "unreachable", "label": "Retry", "op": "robot.status", "reason": "not reachable over ssh"}
    if phase == "ready" and linked and not estopped:
        return {**out, "step": "ready", "label": "Ready", "op": None, "reason": "Standing, linked"}
    if phase == "ready" and not linked:
        if fresh:   # the Pi says bridge up, the laptop gets no acks: checking again will not fix it
            return {**out, "step": "nolink", "label": "No link", "op": None,
                    "reason": "Pi bridge up, no acks here: wifi, udp_host or --no-link"}
        return {**out, "step": "disconnected", "label": "Check", "op": "robot.status",
                "reason": "Link down: state unknown, check"}
    if phase == "fault" or (fresh and phase not in SETUP_STEPS and phase != "starting"):
        return {**out, "step": "fault", "label": "Stop stack", "op": "robot.stop_stack", "reason": detail or "robot fault",
                "ask": "Stops the robot stack: legs go limp. Support the robot first."}
    if not fresh:
        why = "not checked yet" if phase is None or age is None else f"last check {age:.0f} s ago"
        return {**out, "step": "check", "label": "Check", "op": "robot.status", "reason": why}
    if phase == "off":
        return {**out, "step": "home", "label": "Home legs", "op": "robot.ready", "args": {"home": True},
                "reason": "Put it on the stand", "ask": HOME_ASK}
    if phase == "starting":
        return {**out, "step": "check", "label": "Check", "op": "robot.status", "reason": "homing, loading controllers"}
    if estopped:
        return {**out, "step": "rearm", "label": "Re-arm", "op": "robot.ready", "reason": "E-stopped: legs off",
                "ask": "Re-arm: stands it up again and clears the E-stop. Area clear, R3 in reach?"}
    if phase == "homed":
        return {**out, "step": "stand", "label": "Stand up", "op": "robot.ready", "reason": "Put it on the floor",
                "ask": "Stands it up and starts the link. Robot on the floor, area clear, R3 in reach?"}
    if phase == "standing":
        return {**out, "step": "link", "label": "Start link", "op": "robot.ready", "reason": "Standing, link off",
                "ask": "Starts the link so the laptop can drive it. Area clear, R3 in reach?"}
    return {**out, "step": "check", "label": "Check", "op": "robot.status", "reason": f"phase {phase}"}


def health(source: str, active: bool, seen: bool, L: dict | None, has_link: bool, op: dict | None,
           phase: str | None, imu_bad: bool, mux_lost: bool, estopped: bool = False) -> dict:
    """One traffic light per robot: level ok|warn|bad|off and a short text. Never claims ready
    without evidence: "Ready" needs fcctl phase ready; otherwise the best observed state."""
    if not active:
        return {"level": "off", "text": "not in this session"}
    if estopped:
        return {"level": "bad", "text": "E-STOPPED · re-arm = Ready"}
    if down_failed(op):
        return {"level": "bad", "text": "still standing? use R3"}
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
