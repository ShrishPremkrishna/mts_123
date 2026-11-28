"""The one list of things anyone can tell the app to do (UI, fc.ctl, agents). Stdlib only.

Each operation has a dotted name, a label, an optional keyboard key, a safety class, typed parameters
and, optionally, an availability rule. `GET /api/ops` publishes the list (manifest); `POST /api/op
{"op": name, "args": {...}}` validates the arguments against it and runs it; `/state` carries
`avail[name] = {enabled, reason?, label?, args?}` computed here, so every client shows the same
enabled/disabled state and the same next arguments (e.g. what V = "who plays" will set).

Replies: {"ok": true, "op": name, ...} or {"ok": false, "op": name, "code": <code>, "error": <text>}.
Codes: unknown_op, bad_args, refused (the app said no, e.g. "no match running"), unavailable.

Operations that share a key are mutually exclusive by their availability rules (P is play-here /
resume / start-now / pause; at most one is enabled at any time). Handlers only translate arguments into
the App's existing methods; no robot logic lives here. Availability rules run under the app lock and
must stay cheap (they are evaluated on every /state).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from .play import GETUP_ASK

VERSION = 1
SAFETY = ("info", "match", "params", "motion", "confirm", "stop")   # what kind of effect the op has


@dataclass(frozen=True)
class P:
    """A parameter. type: str | num | int | bool | robot | robots | dict. choices may be a callable(app)."""
    type: str
    doc: str = ""
    required: bool = False
    choices: Any = None
    lo: float | None = None
    hi: float | None = None
    default: Any = None


@dataclass(frozen=True)
class Op:
    name: str
    label: str
    doc: str
    group: str
    run: Callable[[Any, dict], dict]
    params: dict[str, P] = field(default_factory=dict)
    key: str | None = None
    safety: str = "info"
    extra: bool = False                          # pass unknown args through (tests take per-test keys)
    when: Callable[[Any], dict] | None = None    # availability: {} = enabled; {"reason": ...} = disabled
    ask: str | None = None                       # the question a UI asks before running it


REGISTRY: dict[str, Op] = {}


def op(name, label, doc, group, /, *, key=None, safety="info", extra=False, when=None, ask=None, **params):
    def deco(fn):
        assert name not in REGISTRY and safety in SAFETY, name
        REGISTRY[name] = Op(name, label, doc, group, fn, params, key, safety, extra, when, ask)
        return fn
    return deco


# ---- manifest, validation, dispatch --------------------------------------------------------------
def _choices(p: P, app) -> list | None:
    c = p.choices(app) if callable(p.choices) else p.choices
    return list(c) if c is not None else None


def manifest(app) -> dict:
    out = []
    for o in REGISTRY.values():
        ps = {}
        for k, p in o.params.items():
            d = {"type": p.type, "required": p.required}
            for a in ("doc", "lo", "hi", "default"):
                if getattr(p, a) not in (None, ""):
                    d[a] = getattr(p, a)
            ch = _choices(p, app)
            if ch is not None:
                d["choices"] = ch
            ps[k] = d
        out.append({"name": o.name, "label": o.label, "doc": o.doc, "group": o.group, "key": o.key,
                    "safety": o.safety, "params": ps, "extra": o.extra, "ask": o.ask})
    return {"version": VERSION, "ops": out}


def _check(name: str, p: P, v, app) -> str | None:
    t = p.type
    if t == "num" and (isinstance(v, bool) or not isinstance(v, (int, float))):
        return f"{name} must be a number"
    if t == "int" and (isinstance(v, bool) or not isinstance(v, int)):
        return f"{name} must be an integer"
    if t == "bool" and not isinstance(v, bool):
        return f"{name} must be true or false"
    if t in ("str", "robot") and not isinstance(v, str):
        return f"{name} must be a string"
    if t == "dict" and not isinstance(v, dict):
        return f"{name} must be an object"
    if t == "robots" and not (isinstance(v, list) and v and all(isinstance(x, str) for x in v)):
        return f"{name} must be a non-empty list of robot ids"
    if t in ("num", "int") and ((p.lo is not None and v < p.lo) or (p.hi is not None and v > p.hi)):
        return f"{name} must be in [{p.lo}, {p.hi}]"
    ch = _choices(p, app)
    if ch is not None:
        bad = [x for x in (v if t == "robots" else [v]) if x not in ch]
        if bad:
            return f"{name} must be one of {ch}"
    return None


def validate(o: Op, args: dict, app) -> tuple[dict, list[str]]:
    errors, out = [], {}
    for k, p in o.params.items():
        if k not in args:
            if p.required:
                errors.append(f"missing {k}")
            elif p.default is not None:
                out[k] = p.default
            continue
        e = _check(k, p, args[k], app)
        if e:
            errors.append(e)
        else:
            out[k] = args[k]
    unknown = [k for k in args if k not in o.params]
    if unknown and not o.extra:
        errors.append(f"unknown argument(s) {unknown}; {o.name} takes {list(o.params) or 'none'}")
    elif o.extra:
        out.update({k: args[k] for k in unknown})
    return out, errors


FAIL_SAFE = ("stop", "estop")    # never refused for their arguments: bad or missing args act on every robot


def call(app, name: str, args: dict | None = None) -> dict:
    args = {} if args is None else args
    o = REGISTRY.get(name)
    if o is None:
        return {"ok": False, "op": name, "code": "unknown_op", "error": f"unknown op {name!r}: GET /api/ops lists them"}
    if name in FAIL_SAFE:
        clean, errors = validate(o, args, app) if isinstance(args, dict) else ({}, ["args must be a JSON object"])
        if errors:   # like the legacy /action estop: a malformed stop/e-stop still stops everyone, and says so
            res = dict(o.run(app, {}) or {"ok": True})
            res["op"] = name
            res["warnings"] = res.get("warnings", []) + [f"bad arguments ({'; '.join(errors)}): applied to every robot"]
            return res
    elif not isinstance(args, dict):
        return {"ok": False, "op": name, "code": "bad_args", "error": "args must be a JSON object"}
    else:
        clean, errors = validate(o, args, app)
        if errors:
            return {"ok": False, "op": name, "code": "bad_args", "error": "; ".join(errors)}
    if o.when:
        with app.lock:
            why = o.when(app).get("reason")
        if why:
            return {"ok": False, "op": name, "code": "unavailable", "error": why}
    res = dict(o.run(app, clean) or {"ok": True})
    res["op"] = name
    if not res.get("ok"):
        res.setdefault("code", "refused")
        if "error" not in res:
            res["error"] = "; ".join(res.get("errors") or []) or "refused"
    return res


def availability(app) -> dict:
    """Caller holds app.lock. {name: {"enabled": bool, "reason"?, "label"?, "args"?}} for every op."""
    out = {}
    for o in REGISTRY.values():
        a = o.when(app) if o.when else {}
        out[o.name] = {"enabled": not a.get("reason"), **a}
    return out


# ---- availability rules ---------------------------------------------------------------------------
def _flow_phase(app):
    return app.flow.phase if app.flow else None


def _no_match(app):
    return {} if app.flow is None else {"reason": "a match is running"}


def _need_match(app):
    return {} if app.flow else {"reason": "no match running"}


def _sim_only(app):
    return {} if app.sim is not None else {"reason": "sim only"}


def _active(app):
    return [r.id for r in app.active]


def _robots(app):
    return list(app.cfg)


def _tests():
    from .app import TESTS           # local: app imports this module
    return TESTS


def _players_next(app):
    from .app import variants        # local: app imports this module
    opts = variants(_active(app))
    i = next((k for k, v in enumerate(opts) if v == app.playing), -1)
    nxt = opts[(i + 1) % len(opts)]
    cur = app.playing
    label = f"{len(cur)} players" if len(cur) > 1 else f"{cur[0].upper()} solo" if cur else "players"
    return {"label": label, "args": {"robots": nxt}} if len(opts) > 1 else {"label": label, "reason": "one robot"}


def _when_kickoff(app):
    halted = [r for r in app.playing if app.halted(r)]
    if halted:
        return {"reason": f"{', '.join(halted)} halted by the bridge"}
    return {"label": "Restart" if app.flow else "Kickoff"}


def _when_play_here(app):
    if app.flow:
        return {"reason": "a match is running"}
    k = _when_kickoff(app)
    return {"reason": k["reason"]} if "reason" in k else {}


def _when_start_now(app):
    return {} if _flow_phase(app) in ("kickoff", "restart") else {"reason": "only while waiting for kickoff or a restart"}


def _when_pause(app):
    return {} if _flow_phase(app) in ("countdown", "play", "goal", "celebrate") else {"reason": "nothing to pause"}


def _when_resume(app):
    return {} if _flow_phase(app) == "paused" else {"reason": "not paused"}


def _when_estop_clear(app):
    return _sim_only(app) or ({} if app.estops else {"reason": "no robot is e-stopped"})


# ---- safety ---------------------------------------------------------------------------------------
@op("stop", "Stop", "Zero speed to every robot now; they stand still with legs on. Ends a running test; a match "
    "pauses instead (score kept: Resume, or End match).",
    "safety", key="Space", safety="stop")
def _stop(app, a):
    return app.stop()


@op("estop", "E-stop", "Legs off (damped drop) for one robot or all; latched until Ready re-arms it. "
    "An unknown robot id e-stops every robot (fail safe) and says so.", "safety", key="Esc", safety="stop",
    robot=P("str", "robot id; omit for all"))
def _estop(app, a):
    rid = a.get("robot")
    res = app.estop(rid)
    if rid not in (None, "", "all") and rid not in [r.id for r in app.active]:
        res["warnings"] = [f"unknown robot {rid!r}: e-stopped every robot"]
    return res


@op("estop.clear", "Clear e-stop", "Sim only: stands in for the Ready that re-arms a real robot.", "safety",
    when=_when_estop_clear, robot=P("robot", "omit for all", choices=_robots))
def _estop_clear(app, a):
    with app.lock:
        return app._estop_clear_sim(a.get("robot"))


# ---- match ----------------------------------------------------------------------------------------
def _play(app, action, **body):
    with app.lock:
        return app._play_action(action, body)


@op("match.kickoff", "Kickoff", "New match: robots walk to their kickoff spots, countdown, play.", "match",
    key="K", safety="match", when=_when_kickoff)
def _kickoff(app, a):
    return _play(app, "kickoff")


@op("match.play_here", "Play", "New match from where everything is now (no kickoff walk).", "match",
    key="P", safety="match", when=_when_play_here)
def _play_here(app, a):
    return _play(app, "play_here")


@op("match.start_now", "Start now", "Skip the wait for kickoff positions or a still ball.", "match",
    key="P", safety="match", when=_when_start_now)
def _start_now(app, a):
    return _play(app, "start_now")


@op("match.pause", "Pause", "Everyone holds still; the clock stops.", "match", key="P", safety="match", when=_when_pause)
def _pause(app, a):
    return _play(app, "pause")


@op("match.resume", "Resume", "Countdown, then play continues.", "match", key="P", safety="match", when=_when_resume)
def _resume(app, a):
    return _play(app, "resume")


@op("match.reposition", "Reset", "Robots walk back to their kickoff spots.", "match", key="R", safety="match",
    when=_need_match)
def _reposition(app, a):
    return _play(app, "reposition")


@op("match.end", "End match", "End the match now and show the final score.", "match", safety="match", when=_need_match)
def _end(app, a):
    return _play(app, "end")


@op("match.touch", "Touches", "Referee touch counter +1 (or -1 to undo).", "match", key="T", safety="info",
    when=_need_match, delta=P("int", "+1 or -1", choices=[1, -1], default=1))
def _touch(app, a):
    return _play(app, "touch", delta=a["delta"])


@op("match.players", "Players", "Who plays: both robots or one alone. Resets the sim to kickoff spots.", "match",
    key="V", safety="match", when=_players_next, robots=P("robots", "robot ids that play", required=True, choices=_active))
def _players(app, a):
    return _play(app, "variant", playing=a["robots"])


@op("match.job", "Job", "A robot's job in the match: auto (by the ball), attack or defend (goalie).", "match",
    safety="match", when=_need_match, robot=P("robot", required=True, choices=_robots),
    job=P("str", required=True, choices=["auto", "attack", "defend"]))
def _job(app, a):
    return _play(app, "job", robot=a["robot"], job=a["job"])


# ---- show ------------------------------------------------------------------------------------------
def _when_warmup(app):
    t = app.test or {}
    if t.get("name") == "warmup":
        return {"label": "End warm-up", "state": "on"}
    if app.flow:
        return {"reason": "a match is running", "label": "Warm-up"}
    if t:
        return {"reason": f"{t.get('name')} is running: Stop first", "label": "Warm-up"}
    halted = [r for r in app.playing if app.halted(r)]
    if halted:
        return {"reason": f"{', '.join(halted)} halted by the bridge", "label": "Warm-up"}
    return {"label": "Warm-up"}


@op("show.warmup", "Warm-up", "Show warm-up while the owner talks: each playing robot loops trot in place, a small circle, "
    "a side shuffle and a quick spin at its kickoff spot (inside the fence margin, away from the other robot), smoothed. "
    "Pressed again: robots walk to their kickoff spots facing the field and it ends. Stop / E-stop / a health hold "
    "end it at once. Then K = kickoff.", "match", key="W", safety="motion", when=_when_warmup,
    loops=P("int", "move-table loops, then to the spots and end; 0 = until W again (or K)", lo=0, hi=20, default=0))
def _warmup(app, a):
    return app.warmup(a["loops"])


# ---- robots (ssh to the Pi; results stream into /state ops[robot]) --------------------------------
def _robot(app, op_name, a, **extra):
    return app.robot_op({"robot": a["robot"], "op": op_name, **extra})


def _real_robots(app):
    """Robot ops ssh the real Pi: in the sim that would move a robot nobody is watching (dry-run only prints)."""
    if app.sim is not None and not app.dry_run_ops:
        return {"reason": "sim: no real robot (start with --dry-run-ops to see the commands)"}
    return {}


@op("robot.ready", "Ready", "fcctl ready: stack off -> homes the legs (robot on the stand) and stops; homed -> stands, "
    "starts the link and re-arms an e-stop. Run it again for the next step. Needs a robot.status at most 15 s old; "
    "homing (phase off) also needs home=true.", "robot", safety="confirm",
    ask="Stands it up if homed, starts the link, re-arms an E-stop; with home=true (stack off) it HOMES the legs: "
        "robot on the stand. Area clear, R3 in reach?",
    when=_real_robots, robot=P("robot", required=True, choices=_robots), confirm=P("bool", "must be true", required=True, choices=[True]),
    home=P("bool", "true only to HOME the legs from phase off (robot on the stand)", default=False),
    pov=P("bool", "start the robot POV feed (fcctl cam start) once ready; default: on if robots.json has a cam_url"))
def _ready(app, a):
    from .play import ready_refusal   # local: play is imported by app, which imports this module
    rid = a["robot"]
    why = ready_refusal(app.robot_phase.get(rid), app.phase_age(rid), a["home"])
    if why:
        return {"ok": False, "error": why}
    return _robot(app, "ready", a, confirm=True, **({"pov": a["pov"]} if "pov" in a else {}))


@op("robot.getup", "Get up", "fcctl getup: a fallen / e-stopped robot back into play WITHOUT homing (~20 s): tilt gate "
    "(refuses unless the IMU says upright: set it on its belly or feet), re-arm, stand, link. The stack must be running "
    "(else it needs Ready with homing, on the stand); an IMU-fault latch (imu_*) stays down (check the IMU, then Ready). "
    "In a match the robot rejoins after a short hold; the match is not paused. E-stop / R3 still win.", "robot",
    safety="confirm", ask=GETUP_ASK, when=_real_robots, robot=P("robot", required=True, choices=_robots),
    confirm=P("bool", "must be true", required=True, choices=[True]))
def _getup(app, a):
    return _robot(app, "getup", a, confirm=True)


@op("robot.down", "Lie down", "fcctl down: gentle lie-down, then the stack stops (keep_stack keeps it homed).", "robot",
    safety="motion", when=_real_robots, robot=P("robot", required=True, choices=_robots), keep_stack=P("bool", default=False))
def _down(app, a):
    return _robot(app, "down", a, keep_stack=a["keep_stack"])


@op("robot.status", "Status", "fcctl status: phase, bridge, latch, Pi health.", "robot",
    when=_real_robots, robot=P("robot", required=True, choices=_robots))
def _status(app, a):
    return _robot(app, "status", a)


@op("robot.stop_stack", "Stop stack", "fcctl stop: stop the robot stack (legs off).", "robot", safety="stop",
    when=_real_robots, robot=P("robot", required=True, choices=_robots))
def _stop_stack(app, a):
    return _robot(app, "stop", a)


@op("robot.bridge", "Bridge", "Start or stop the link bridge on the Pi.", "robot", safety="motion",
    when=_real_robots, robot=P("robot", required=True, choices=_robots), on=P("bool", required=True))
def _bridge(app, a):
    return _robot(app, "bridge_start" if a["on"] else "bridge_stop", a)


@op("robot.stage", "Stage", "Copy fc/robot to the Pi (rsync) with the link key.", "robot",
    when=_real_robots, robot=P("robot", required=True, choices=_robots))
def _stage(app, a):
    return _robot(app, "stage", a)


@op("robot.imu_test", "IMU test", "Record the IMU for some seconds (robot powered, still).", "robot",
    when=_real_robots, robot=P("robot", required=True, choices=_robots), seconds=P("int", lo=1, hi=900, default=600),
    mode=P("str", choices=["bno", "ros", "auto"]), condition=P("str"))
def _imu(app, a):
    return _robot(app, "imu_test", a, **{k: a[k] for k in ("seconds", "mode", "condition") if k in a})


# ---- app ------------------------------------------------------------------------------------------
def _when_source(app):
    if app.restart_hook is None:
        return {"reason": "started without the launcher: restart with the other --source"}
    why = app.busy_reason()
    return {"reason": why} if why else {}


@op("app.source", "Mode", "Switch between Simulation (sim) and Play (camera: field camera, real robots). Restarts the app "
    "with the same flags and the loaded preset (unsaved live params are lost); refused while a match, test or robot "
    "operation runs.", "app", safety="motion", when=_when_source,
    ask="Restarts the app in the other mode: link, camera and robots stop for a few seconds. The loaded preset is kept; "
        "unsaved live param changes are lost.",
    source=P("str", required=True, choices=["sim", "camera"]), confirm=P("bool", "must be true", required=True, choices=[True]))
def _source(app, a):
    return app.switch_source(a["source"])


# ---- tests, params, presets, sim ------------------------------------------------------------------
@op("test.run", "Run test", "Start a test run (drive, goto, step, deadman, push_ball, goalie, match, sim_eval, autoplay). "
    "Per-test keys pass through (step: vx vy wz seconds; goto: target; deadman: vx mute_s; sim_eval: args; "
    "autoplay: minutes alternate reset after_roll reset_vx).",
    "test", safety="motion", extra=True, test=P("str", required=True, choices=lambda app: _tests()),
    robot=P("robot", choices=_robots), scenario=P("str"))
def _test(app, a):
    with app.lock:
        return app._start_test(a)


def _auto_robot(app):
    """Autoplay's robot: the one playing solo, else the first active robot."""
    act = _active(app)
    return app.playing[0] if len(app.playing) == 1 and app.playing[0] in act else (act or [None])[0]


def _when_autoplay(app):
    if app.test:
        return {"reason": "autoplay is running: Space stops it" if app.test.get("name") == "autoplay"
                else f"{app.test.get('name')} is running: Stop first", "state": "on" if app.test.get("name") == "autoplay" else None}
    rid = _auto_robot(app)
    if rid is None:
        return {"reason": "no active robot"}
    if app.halted(rid):
        return {"reason": f"{rid} halted ({app.halted(rid)})"}
    return {"label": f"Autoplay {rid.upper()}", "args": {"robot": rid}}


@op("autoplay.start", "Autoplay", "Hands-off solo demo loop: attack the ball wherever it is, score, bring the ball back "
    "to the centre spot slowly, back off, attack again (alternating goals); a ball left in a net shows \"please roll the "
    "ball out\" and waits. Ends after `minutes` (0 = until Stop) or on Stop / E-stop / IMU / motion-link loss.", "test",
    key="A", safety="motion", when=_when_autoplay,
    ask="Starts the hands-off loop: the robot keeps attacking, scoring and bringing the ball back by itself. Area clear, "
        "Space / R3 in reach?",
    robot=P("robot", choices=_robots), minutes=P("num", "0 = until Stop", lo=0, hi=240, default=10),
    alternate=P("bool", "attack the other goal each cycle", default=True),
    reset=P("str", "after a goal: centre = dribble the ball to the centre spot, none = attack from where it is",
            choices=["centre", "none"], default="centre"),
    after_roll=P("str", "ball rolled out of a goal by a person: centre first, or attack at once",
                 choices=["centre", "attack"], default="centre"),
    reset_vx=P("num", "walking speed while bringing the ball back, m/s", lo=0.05, hi=0.3, default=0.15),
    confirm=P("bool", "set by the UI's question", choices=[True]))
def _autoplay(app, a):
    a = {k: v for k, v in a.items() if k != "confirm"}
    with app.lock:
        return app._start_test({"test": "autoplay", "robot": a.pop("robot", None) or _auto_robot(app), **a})


@op("freeplay.start", "Free play", "Brain on every active robot with the current params (no match flow).", "test",
    safety="motion")
def _freeplay(app, a):
    with app.lock:
        app._begin("play", None)
    return {"ok": True}


@op("teleop", "Drive", "Drive test only: fractions of max_* in [-1, 1].", "test", safety="motion",
    robot=P("robot", required=True, choices=_robots), vx=P("num", lo=-1, hi=1, default=0),
    vy=P("num", lo=-1, hi=1, default=0), wz=P("num", lo=-1, hi=1, default=0))
def _teleop(app, a):
    return app.action({"action": "teleop", **a})


@op("params.set", "Set params", "Live params (validated; unknown keys refused). Kept until restart or preset load.",
    "params", safety="params", values=P("dict", required=True))
def _params(app, a):
    return app.set_params(a["values"])


@op("preset.load", "Load preset", "Base params + the preset's keys (presets never stack).", "params", safety="params",
    name=P("str", required=True, choices=lambda app: app.presets()))
def _preset_load(app, a):
    res = app.preset({"op": "load", "name": a["name"]})
    if res.get("ok") and res.get("errors"):
        res["warnings"] = res.pop("errors")
    return res


@op("preset.base", "Base params", "Back to params.json + defaults.", "params", safety="params")
def _preset_base(app, a):
    return app.preset({"op": "base"})


@op("preset.save", "Save preset", "Save the keys that differ from base under a name.", "params", safety="params",
    name=P("str", required=True))
def _preset_save(app, a):
    return app.preset({"op": "save", "name": a["name"]})


@op("sim.reset", "Reset sim", "Sim only: put robots and ball at a scenario.", "test", when=_sim_only,
    scenario=P("str", default="kickoff"))
def _sim_reset(app, a):
    return app.action({"action": "reset", "scenario": a["scenario"]})


@op("score.reset", "Reset score", "Free-play score back to 0-0.", "test")
def _score_reset(app, a):
    return app.action({"action": "score_reset"})


def _when_recalibrate(app):
    if app.eye is None:
        return {"reason": "camera only (sim has no calibration)"}
    return {} if app.mode == "idle" else {"reason": f"{app.mode} is running: recalibrate only while idle"}


@op("eye.recalibrate", "Recalibrate", "Camera only, idle only: full re-fit of lens, field size and camera pose from the "
    "four corner tags (empty field, up to ~8 s); the field size follows. The old calibration stays if the fit fails.",
    "camera", safety="params", when=_when_recalibrate,
    ask="Re-fits the camera from the four corner tags: clear the field (robots and ball off the corner tags) first.")
def _recalibrate(app, a):
    return app.recalibrate(timeout=8.0)


@op("ballcal", "Ball colour", "Camera only: ball HSV range at pixel (x, y) or (u, v) in [0, 1], else the most likely "
    "blob; show=true only reports.", "camera", extra=True)
def _ballcal(app, a):
    return app.ballcal(a)
