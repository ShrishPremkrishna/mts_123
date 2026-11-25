"""Batch validation of brain + sim. Run from final_project/:
    python -m fc.sim_eval [--seeds N] [--matches M] [--quick] [--off] [--preset NAME ...] [--set key=value ...]
                          [--field key=value ...] [--scen core human return moving] [--lag 0.3 0.7] [--floor]

Prints one table per error setting (errors ON = measured model, OFF = ideal), one column per
preset: `base` is params.json, any other name is params.json + fc/config/presets/NAME.json
(e.g. `--preset base fast-play` compares them side by side). `--set` overrides on top of every
column (to try a value before committing it). `--field` overrides field.json geometry the same way
(e.g. `--field goal_width=0.8` to try a wider marked goal). `--lag L ...` repeats every column at
a total lag L (camera latency L/2 + walker lag L/2; the brain's act_lag_s set to L/2, as if measured).
`--scen` picks row groups (default all): core (rows below), human, return, moving; solo = the solo push
rows only (fast). `--floor25` = FLOOR updated with the Nov 25 measurements (FLOOR25 below, cap 0.8). `--floor` adds the
Nov 24 floor errors on top of the sim's (FLOOR below: 0.1 s camera latency, tag dropouts of 0.3-1.5 s,
fast gait at ~0.83 of the command and walking 8 deg right of its heading, slower walker onset, a ball
rolling at 0.18 m/s^2 deceleration: hand rolls of the old ball fit 0.12-0.29, median 0.15).

Rows:
- solo push/headbutt: goals within 40 s from a random start, median time to goal (s).
- solo jitter (push runs, per robot-minute of play): wz reversals / reversals < 1 s apart, RMS of the
  commanded wz (rad/s), replans (plan mode: paths planned), coast share (ticks steering blind).
- athletic (push runs): mean true speed (m/s), median time to the first touch of the ball (s).
- goalie: saves of the same shots that score on an empty net.
- 1v1 (auto roles, kickoff): goals per 3 min each side; deadlocks = the ball stayed within 10 cm
  for 10 s; traps = the sim referee nudged a wall ball.
- smoothness per robot-minute: wz reversals (|wz| > 0.25 one way, then the other); fast wz rev =
  reversals less than 1 s apart; side flips = changes of the way round the ball while orbiting.
- contact: true robot body speed at each first touch of the ball (solo push + 1v1); hard =
  touches above 0.45 m/s. bumps = robot-robot first touches (closing speed).
- pace: mean true robot speed in 1v1; gait switches /robot-min; "in band" = share of ticks with a
  forward command inside (0.45, 0.6), where the walker hovers at its gait switch.
- human (one robot in auto vs a scripted human, `--matches` x `--minutes`): every 4-9 s the human
  picks the ball up and either shoots at the robot's goal from its own half (60%) or rolls it to a
  spot outside the robot's third (40%). Robot goals / conceded per 3 min; saves of shots; role
  switches per min; reaction = shot -> brain role defend, shot -> true robot velocity toward its goal
  >= 0.1 m/s (median s, only when not already defending / on the line); right role = defend within
  1.5 s of a shot / no defend within 2 s of a roll.
- return: robot upfield, ball resting near it, a human shot at our goal at a random 1-4 s (from the
  human's spot if the robot is on the ball). auto: saves. forced (attack until the shot, then defend:
  the return run alone): reached the guard line (+-0.1 m, inside the mouth) within 8 s, median time
  from the shot, and the ball's median time to our goal line for comparison.
- moving: ball rolled across the field (0.5-1.1 m/s), robot role attack from its own half: first
  touch within 6 s, that touch sends the ball toward the attack goal, goal within 15 s (median s).
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import random
import statistics
import time

from .brain import Brain
from .core import CONFIG, load_field, load_params, load_robots
from .sim import Sim

DT = 1.0 / 30
PRESETS = CONFIG / "presets"
BAND = (0.45, 0.6)
FLOOR = {"sim_latency_s": 0.10, "sim_burst_per_s": 0.15, "sim_burst_s": 0.3, "sim_burst_s_max": 1.5,
         "sim_fast_v060": 0.50, "sim_fast_v075": 0.62, "sim_fast_v100": 0.75, "sim_fast_v150": 1.0,
         "sim_fast_dir_deg": -8.0, "sim_lag_s": 0.2, "sim_ball_decel": 0.18}
# Nov 25 floor: new evenly rolling ball (rollcal 0.149 m/s^2), fast gait 0.83-0.93 of the command (0.88 used),
# camera 31 fps at ~50 ms age (+ eye processing), walker speed cap 0.8 (sim_vx_clip)
FLOOR25 = {**FLOOR, "sim_ball_decel": 0.149, "sim_latency_s": 0.07, "sim_fast_v060": 0.53, "sim_fast_v075": 0.66,
           "sim_fast_v100": 0.88, "sim_fast_v150": 1.25, "sim_vx_clip": 0.8}


class Jitter:
    """Command smoothness per robot: wz reversals (|wz| > 0.25 one way, then the other), reversals less
    than 1 s apart, sum of wz^2 for the RMS."""

    def __init__(self):
        self.sg, self.last, self.rev, self.fast, self.sq, self.n = {}, {}, 0, 0, 0.0, 0

    def add(self, k, c, t: float) -> None:
        if c is None:
            return
        self.n += 1
        self.sq += c.wz * c.wz
        if abs(c.wz) > 0.25:
            sg = 1 if c.wz > 0 else -1
            if self.sg.get(k, sg) != sg:
                self.rev += 1
                self.fast += t - self.last.get(k, -9.0) < 1.0
                self.last[k] = t
            self.sg[k] = sg


def load_preset(name: str) -> dict:
    if name == "base":
        return {}
    data = json.loads((PRESETS / f"{name}.json").read_text())
    return {k: v for k, v in data.items() if not k.startswith("_")}


def _loop(sim: Sim, brains: dict, params: dict, seconds: float, on_tick=None) -> None:
    for _ in range(int(seconds / DT)):
        w = sim.observe()
        cmds, dbgs = {}, {}
        for k, b in brains.items():
            if k in sim.bots:
                cmds[k], dbgs[k] = b.decide(w, params)
        sim.step(cmds, DT)
        if on_tick and on_tick(cmds, dbgs):
            return


def _goals(sim):
    return [e for e in sim.events if e["type"] == "goal"]


def solo(field, robots, params, seed, mode, limit=40.0, contacts=None):
    """One robot attacks a resting ball from a random start. Returns (goal time or None, own goal, stats)."""
    p = {**params, "contact_mode": mode, "role_mode": "auto"}
    sim = Sim(field, robots, p, seed)
    sim.reset("attack_solo")
    me = robots[0]
    brain = Brain(me, field)
    jit, near, st = Jitter(), Jitter(), {"speed": 0.0, "coast": 0}
    cc = field.ball_radius + field.robot_radius
    dr = {"poss": 0, "touch": 0, "in": False, "gap": 0.0, "ng": 0, "nc": 0, "check": [], "defl": [], "rec": []}

    def tick(c, d):
        jit.add(me.id, c.get(me.id), sim.t)
        bot = sim.bots.get(me.id)
        st["speed"] += math.hypot(bot.vx, bot.vy) if bot else 0.0
        st["coast"] += d.get(me.id, {}).get("phase") == "coast"
        if bot:
            gap = math.hypot(sim.ball[0] - bot.x, sim.ball[1] - bot.y) - cc
            if gap < 0.6:
                near.add(me.id, c.get(me.id), sim.t)
            new = list(sim.contacts)[dr["nc"]:]
            dr["nc"] = len(sim.contacts)
            for ct in new:
                dr["touch"] += 1
                if not dr["in"]:
                    dr["in"], dr["poss"] = True, dr["poss"] + 1
                if dr["defl"] and dr["defl"][-1][1] is None:
                    dr["defl"][-1][1] = ct[0] - dr["defl"][-1][0]
                dr["check"].append(ct[0] + 0.15)
            if dr["in"] and gap > 0.6:
                dr["in"] = False
            if dr["in"]:
                dr["gap"] += max(0.0, gap)
                dr["ng"] += 1
            while dr["check"] and sim.t >= dr["check"][0]:
                t0c = dr["check"].pop(0) - 0.15
                bvx, bvy = sim.ball[2], sim.ball[3]
                if math.hypot(bvx, bvy) > 0.1 and abs(math.atan2(math.sin(math.atan2(bvy, bvx) - bot.yaw),
                                                                math.cos(math.atan2(bvy, bvx) - bot.yaw))) > math.radians(30):
                    dr["defl"].append([t0c, None])
        return bool(_goals(sim))

    t0 = sim.t
    _loop(sim, {me.id: brain}, p, limit, tick)
    if contacts is not None:
        contacts.extend(c[2] for c in sim.contacts)
    ev = _goals(sim)[0] if _goals(sim) else None
    n = max(1, jit.n)
    stats = {"min": (sim.t - t0) / 60, "rev": jit.rev, "fast": jit.fast, "sq": jit.sq, "n": jit.n,
             "replans": brain.replans, "coast": st["coast"] / n, "speed": st["speed"] / n,
             "touch": (sim.contacts[0][0] - t0) if sim.contacts else None,
             "dr_touch": dr["touch"], "dr_poss": dr["poss"], "dr_gap": dr["gap"], "dr_ng": dr["ng"],
             "near_sq": near.sq, "near_n": near.n, "near_rev": near.rev,
             "defl": len(dr["defl"]), "rec": [x[1] for x in dr["defl"] if x[1] is not None and x[1] <= 8.0]}
    return (ev["t"] - t0 if ev and ev["team"] == me.team else None), bool(ev and ev["team"] != me.team), stats


def goalie(field, robots, params, seed, style, empty=False):
    p = {**params, "role_mode": "defend", "goalie_style": style}
    sim = Sim(field, robots, p, seed)
    sim.reset("shot_on_goal", **({"robots": [], "team": robots[0].team} if empty else {}))
    me = robots[0]
    brains = {me.id: Brain(me, field)}
    _loop(sim, brains, p, 6.0, lambda c, d: bool(_goals(sim)) or math.hypot(sim.ball[2], sim.ball[3]) < 0.02)
    return not _goals(sim)


def match(field, robots, params, seed, minutes=3.0):
    p = {**params, "role_mode": "auto"}
    sim = Sim(field, robots, p, seed)
    sim.reset("kickoff")
    brains = {r.id: Brain(r, field) for r in robots}
    st = {"flips": 0, "roles": {}, "wz": {}, "flipwz": 0, "fastwz": 0, "lastrev": {}, "jerk": [], "last": {},
          "stall": 0, "anchor": (0.0, 0.0, 0.0), "side": {}, "sideflip": 0, "band": 0, "ticks": 0,
          "speed": [], "switch": 0, "bots": {}}
    jit = Jitter()

    def tick(cmds, dbgs):
        for k, b in brains.items():
            side = b.orbit_side
            if side and st["side"].get(k) and side != st["side"][k] and dbgs.get(k, {}).get("phase", "").endswith("route"):
                st["sideflip"] += 1
            if side:
                st["side"][k] = side
            r = b.role
            if st["roles"].get(k) not in (None, r):
                st["flips"] += 1
            st["roles"][k] = r
            c = cmds.get(k)
            jit.add(k, c, sim.t)
            prev = st["last"].get(k)
            if c:
                st["ticks"] += 1
                st["band"] += BAND[0] < c.vx < BAND[1]
            if c and prev:
                if abs(c.wz) > 0.25:  # a turn reversal: |wz| > 0.25 one way, then the other
                    sg = 1 if c.wz > 0 else -1
                    if st["wz"].get(k, sg) != sg:
                        st["flipwz"] += 1
                        if sim.t - st["lastrev"].get(k, -9.0) < 1.0:
                            st["fastwz"] += 1
                        st["lastrev"][k] = sim.t
                    st["wz"][k] = sg
                st["jerk"].append(abs(c.vx - prev.vx) / DT + abs(c.wz - prev.wz) / DT * 0.3)
            if c:
                st["last"][k] = c
        for k, bot in sim.bots.items():  # bots are rebuilt at each kickoff: sum switches per object
            st["speed"].append(math.hypot(bot.vx, bot.vy))
            st["bots"][id(bot)] = bot.switches
        bx, by = sim.ball[0], sim.ball[1]
        ax, ay, at = st["anchor"]
        if math.hypot(bx - ax, by - ay) > 0.1 or sim.pause_until > 0:
            st["anchor"] = (bx, by, sim.t)
        elif sim.t - at > 10.0:
            st["stall"] += 1
            st["anchor"] = (bx, by, sim.t)
        return False

    _loop(sim, brains, p, minutes * 60, tick)
    n = len(robots)
    return {"goals": dict(sim.score), "flips_pm": st["flips"] / minutes / n,
            "wzflip_pm": st["flipwz"] / minutes / n, "fastwz_pm": st["fastwz"] / minutes / n,
            "side_pm": st["sideflip"] / minutes / n, "stalls": st["stall"],
            "jerk": statistics.mean(st["jerk"]) if st["jerk"] else 0.0,
            "traps": sum(e["type"] == "trapped" for e in sim.events),
            "contacts": [c[2] for c in sim.contacts], "bumps": [b[3] for b in sim.bumps],
            "speed": statistics.mean(st["speed"]) if st["speed"] else 0.0,
            "switch_pm": sum(st["bots"].values()) / minutes / n,
            "band": st["band"] / max(1, st["ticks"]), "wzrms": math.sqrt(jit.sq / max(1, jit.n)),
            "replans_pm": sum(b.replans for b in brains.values()) / minutes / n}


def _fastwz(c, k, st, t):
    """Count turn reversals less than 1 s apart (|wz| > 0.25 one way, then the other)."""
    if c and abs(c.wz) > 0.25:
        sg = 1 if c.wz > 0 else -1
        if st.setdefault("wz", {}).get(k, sg) != sg:
            if t - st.setdefault("lastrev", {}).get(k, -9.0) < 1.0:
                st["fastwz"] = st.get("fastwz", 0) + 1
            st["lastrev"][k] = t
        st["wz"][k] = sg


def _guard_line(field, team, params):
    s = -1.0 if team == "blue" else 1.0  # own goal side
    return s, s * (field.length / 2 - params.get("goalie_dist", 0.25))


def human_solo(field, robots, params, seed, minutes=3.0):
    """One robot (auto unless params say otherwise, e.g. role_mode=defend for a pure keeper) vs a
    scripted human who shoots at its goal or rolls the ball around."""
    p = {"role_mode": "auto", **params}
    me = robots[0]
    sim = Sim(field, robots, p, seed)
    sim.reset("kickoff", robots=[me.id])
    brain = Brain(me, field)
    r = random.Random(seed + 7)  # the human's own dice: brain changes do not reshuffle the script
    s, xl = _guard_line(field, me.team, p)
    hl, hw, gw = field.length / 2, field.width / 2, field.goal_width / 2
    third = params.get("solo_third", 0.333) * field.length
    st = {"next": 3.0, "ev": [], "flips": 0, "role": None}

    def close(e, t):
        e["end"] = t

    def tick(cmds, dbgs):
        t, bot = sim.t, sim.bots[me.id]
        role = brain.role
        if st["role"] not in (None, role):
            st["flips"] += 1
        st["role"] = role
        _fastwz(cmds.get(me.id), me.id, st, t)
        e = st["ev"][-1] if st["ev"] else None
        if e and "end" not in e:
            el = t - e["t"]
            if role == "defend" and e["defend"] is None:
                e["defend"] = el
            c, sn = math.cos(bot.yaw), math.sin(bot.yaw)
            vx_w = c * bot.vx - sn * bot.vy
            if e["move"] is None and (s * vx_w >= 0.1 or abs(bot.x - xl) < 0.15):
                e["move"] = el
            goals = [g for g in sim.events[e["n"]:] if g["type"] == "goal"]
            if goals:
                e["conceded"] = goals[0]["team"] != me.team
                close(e, t)
        if t >= st["next"] and sim.pause_until < 0:
            if e and "end" not in e:
                close(e, t)
            shot = r.random() < 0.6
            x, y = -s * r.uniform(0.2, 1.0), r.uniform(-0.7, 0.7)
            if shot:
                ty = r.uniform(-(gw - field.ball_radius), gw - field.ball_radius)
                sim.shoot(s * (hl + field.ball_radius + 0.03), ty, r.uniform(0.1, 0.5), x, y)
            else:  # roll to a spot outside the robot's third
                tx = s * (hl - third - r.uniform(0.2, 1.0))
                tx = max(-hl + 0.2, min(hl - 0.2, tx))
                sim.shoot(tx, r.uniform(-hw + 0.2, hw - 0.2), 0.05, x, y)
            st["ev"].append({"t": t, "n": len(sim.events), "shot": shot, "role0": role,
                             "online0": abs(bot.x - xl) < 0.15, "defend": None if role != "defend" else 0.0,
                             "move": None, "conceded": False})
            st["next"] = t + r.uniform(4.0, 9.0)
        return False

    _loop(sim, {me.id: brain}, p, minutes * 60, tick)
    goals = [g for g in sim.events if g["type"] == "goal"]
    return {"scored": sum(g["team"] == me.team for g in goals), "conceded": sum(g["team"] != me.team for g in goals),
            "ev": st["ev"], "flips_pm": st["flips"] / minutes, "fastwz_pm": st.get("fastwz", 0) / minutes}


def return_test(field, robots, params, seed, window=8.0, forced=False):
    """Robot upfield in auto; a human shot at its goal comes at a random 1-4 s. forced: attack until
    the shot, defend from the shot on (the return itself, without the role rule)."""
    p = {**params, "role_mode": "attack" if forced else "auto"}
    me = robots[0]
    sim = Sim(field, robots, p, seed)
    sim.reset("return")
    brain = Brain(me, field)
    s, xl = _guard_line(field, me.team, p)
    lim = field.goal_width / 2 + 0.1
    st = {"shot": None, "line": None, "arrive": None}

    def tick(cmds, dbgs):
        t, bot = sim.t, sim.bots[me.id]
        if st["shot"] is None:
            k = [e for e in sim.events if e["type"] == "kick"]
            if k:
                st["shot"] = k[0]["t"]
                if forced:
                    p["role_mode"] = "defend"
                bv = (k[0]["vx"], k[0]["vy"])
                st["arrive"] = brain._arrival((k[0]["x"], k[0]["y"]), bv, brain.own_x, {"ball_decel": sim.p["sim_ball_decel"]})[0]
            elif _goals(sim) or not sim.scheduled:
                return True
            return False
        el = t - st["shot"]
        if st["line"] is None and abs(bot.x - xl) < 0.1 and abs(bot.y) < lim:
            st["line"] = el
        return bool(_goals(sim)) and not forced or el > window or (forced and st["line"] is not None)
    _loop(sim, {me.id: brain}, p, 4.5 + window, tick)
    g = [e for e in _goals(sim) if st["shot"] is not None and e["t"] >= st["shot"]]
    if st["shot"] is None:
        return None  # the robot scored before the human shot
    return {"line": st["line"], "save": not (g and g[0]["team"] != me.team), "arrive": st["arrive"]}


def moving_attack(field, robots, params, seed):
    """Ball rolled across the field; the robot (role attack) must reach it and hit it toward goal."""
    p = {**params, "role_mode": "attack"}
    me = robots[0]
    sim = Sim(field, robots, p, seed)
    sim.reset("rolling_ball")
    brain = Brain(me, field)
    att = 1.0 if me.team == "blue" else -1.0
    st = {"touch": None, "dir": None, "goal": None}

    def tick(cmds, dbgs):
        if st["touch"] is None and sim.contacts:
            st["touch"] = sim.contacts[0][0]
        if st["touch"] is not None and st["dir"] is None and sim.t - st["touch"] >= 0.2:
            st["dir"] = att * sim.ball[2] > 0.05
        g = _goals(sim)
        if g:
            st["goal"] = g[0]["t"] if g[0]["team"] == me.team else None
            return True
        return False
    _loop(sim, {me.id: brain}, p, 15.0, tick)
    return {"hit": st["touch"] is not None and st["touch"] <= 6.0, "toward": bool(st["dir"]), "goal": st["goal"]}


def lag_over(total: float) -> dict:
    """Total loop lag split evenly between camera latency and walker lag; the brain told the walker part."""
    return {"sim_latency_s": total / 2, "sim_lag_s": total / 2, "act_lag_s": total / 2}


def _med(xs):
    return statistics.median(xs) if xs else float("nan")


def _pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else 0.0


def evaluate(seeds: int, errors: bool, minutes: float, matches: int, overrides: dict | None = None,
             headbutt: bool = True, field_over: dict | None = None, scen=("core",)) -> dict:
    """Metrics for one parameter set: {row label: formatted value} plus raw numbers under '_raw'."""
    field, robots, params = load_field(), load_robots(), load_params()
    if field_over:
        field = dataclasses.replace(field, **field_over)
    params = {**params, **(overrides or {}), "sim_errors": int(errors)}
    out, raw = {}, {}
    if "core" in scen:
        _core(out, raw, field, robots, params, seeds, minutes, matches, headbutt)
    elif "solo" in scen:
        _core(out, raw, field, robots, params, seeds, minutes, matches, False, solo_only=True)
    if "human" in scen:
        _human(out, raw, field, robots, params, matches, minutes)
    if "return" in scen:
        res = [x for x in (return_test(field, robots, params, 3000 + s) for s in range(seeds)) if x]
        n = max(1, len(res))
        raw["ret_save"] = sum(x["save"] for x in res) / n
        out["return (auto): saves"] = f"{sum(x['save'] for x in res)}/{len(res)} ({100 * raw['ret_save']:.0f}%)"
        fr = [x for x in (return_test(field, robots, params, 3000 + s, forced=True) for s in range(seeds)) if x]
        line = [x["line"] for x in fr if x["line"] is not None]
        raw.update(ret_line=len(line) / max(1, len(fr)), ret_med=_med(line),
                   ret_ball=_med([x["arrive"] for x in fr if x["arrive"] < math.inf]))
        out["return (forced): on line / median s, ball s"] = (f"{len(line)}/{len(fr)}, {raw['ret_med']:.1f}, "
                                                              f"{raw['ret_ball']:.1f}")
    if "moving" in scen:
        res = [moving_attack(field, robots, params, 4000 + s) for s in range(seeds)]
        g = [x["goal"] for x in res if x["goal"] is not None]
        raw.update(mov_hit=sum(x["hit"] for x in res) / seeds, mov_toward=sum(x["toward"] for x in res) / seeds,
                   mov_goal=len(g) / seeds, mov_med=_med(g))
        out["moving: hit <=6 s / toward goal / goal <=15 s"] = (f"{sum(x['hit'] for x in res)}/"
                                                             f"{sum(x['toward'] for x in res)}/{len(g)} of {seeds}")
        out["moving: median s to goal"] = f"{raw['mov_med']:.1f}"
    out["_raw"] = raw
    return out


def _human(out, raw, field, robots, params, matches, minutes):
    ms = [human_solo(field, robots, params, 5000 + s, minutes) for s in range(matches)]
    k = 3 / minutes / matches
    ev = [e for m in ms for e in m["ev"]]
    shots = [e for e in ev if e["shot"]]
    rolls = [e for e in ev if not e["shot"]]
    saves = sum(not e["conceded"] for e in shots)
    rd = [e["defend"] for e in shots if e["role0"] != "defend" and e["defend"] is not None]
    rm = [e["move"] for e in shots if not e["online0"] and e["move"] is not None]
    right_s = sum(e["defend"] is not None and e["defend"] <= 1.5 for e in shots)
    right_r = sum(e["defend"] is None or e["defend"] > 2.0 for e in rolls)
    raw.update(h_scored=sum(m["scored"] for m in ms) * k, h_conceded=sum(m["conceded"] for m in ms) * k,
               h_saves=saves / max(1, len(shots)), h_flips=statistics.mean(m["flips_pm"] for m in ms),
               h_react_def=_med(rd), h_react_move=_med(rm), h_right_shot=right_s / max(1, len(shots)),
               h_right_roll=right_r / max(1, len(rolls)), h_fastwz=statistics.mean(m["fastwz_pm"] for m in ms))
    out[f"human ({matches}x{minutes:g} min): robot goals / conceded /3min"] = (f"{raw['h_scored']:.1f} / "
                                                                         f"{raw['h_conceded']:.1f}")
    out["human: saves of shots"] = f"{saves}/{len(shots)} ({100 * raw['h_saves']:.0f}%)"
    out["human: role switches /min"] = f"{raw['h_flips']:.1f}"
    out["human: react s, ->defend / ->moving back"] = f"{raw['h_react_def']:.2f} / {raw['h_react_move']:.2f}"
    out["human: right role, shots / rolls"] = (f"{100 * raw['h_right_shot']:.0f}% / "
                                               f"{100 * raw['h_right_roll']:.0f}%")
    out["human: fast wz rev /min"] = f"{raw['h_fastwz']:.1f}"


def _core(out, raw, field, robots, params, seeds, minutes, matches, headbutt, solo_only=False):
    contacts = []
    for mode in ("push", "headbutt") if headbutt else ("push",):
        res = [solo(field, robots, params, s, mode, contacts=contacts if mode == "push" else None)
               for s in range(seeds)]
        times = [t for t, _, _ in res if t is not None]
        med = statistics.median(times) if times else float("nan")
        raw[f"solo_{mode}_rate"], raw[f"solo_{mode}_med"] = len(times) / seeds, med
        out[f"solo {mode}: goals (<=40 s)"] = f"{len(times)}/{seeds} ({100 * len(times) / seeds:.0f}%)"
        out[f"solo {mode}: median s / own goals"] = f"{med:.1f} / {sum(o for _, o, _ in res)}"
        if mode == "push":
            ss = [x for _, _, x in res]
            mins = sum(x["min"] for x in ss) or 1.0
            raw.update(solo_rev=sum(x["rev"] for x in ss) / mins, solo_fastrev=sum(x["fast"] for x in ss) / mins,
                       solo_wzrms=math.sqrt(sum(x["sq"] for x in ss) / max(1, sum(x["n"] for x in ss))),
                       solo_replans=sum(x["replans"] for x in ss) / mins,
                       solo_coast=statistics.mean(x["coast"] for x in ss),
                       solo_speed=statistics.mean(x["speed"] for x in ss),
                       solo_touch=_med([x["touch"] for x in ss if x["touch"] is not None]),
                       solo_p90=_pct(times, 0.9))
            out["solo push: p90 s"] = f"{raw['solo_p90']:.1f}"
            out["solo jitter: wz rev / fast rev /min, wz rms"] = (f"{raw['solo_rev']:.1f} / {raw['solo_fastrev']:.1f}, "
                                                                 f"{raw['solo_wzrms']:.2f}")
            out["solo jitter: replans /min, coast %"] = f"{raw['solo_replans']:.1f}, {100 * raw['solo_coast']:.0f}%"
            out["athletic: mean speed m/s, s to ball"] = f"{raw['solo_speed']:.2f}, {raw['solo_touch']:.1f}"
            nn = sum(x["near_n"] for x in ss)
            rec = [r for x in ss for r in x["rec"]]
            nd = sum(x["defl"] for x in ss)
            raw.update(dr_tpp=sum(x["dr_touch"] for x in ss) / max(1, sum(x["dr_poss"] for x in ss)),
                       dr_gap=sum(x["dr_gap"] for x in ss) / max(1, sum(x["dr_ng"] for x in ss)),
                       near_rms=math.sqrt(sum(x["near_sq"] for x in ss) / max(1, nn)),
                       near_rev=sum(x["near_rev"] for x in ss) / max(1e-6, nn * DT / 60),
                       defl=nd, rec_med=_med(rec), rec_ok=len(rec) / max(1, nd))
            out["dribble: touches/possession, gap m"] = f"{raw['dr_tpp']:.1f}, {raw['dr_gap']:.2f}"
            out["near ball (<0.6 m): wz rms, rev/min"] = f"{raw['near_rms']:.2f}, {raw['near_rev']:.1f}"
            out["deflections: n, re-touch <=8 s, median s"] = (f"{nd}, {100 * raw['rec_ok']:.0f}%, "
                                                               f"{raw['rec_med']:.1f}")
    if solo_only:
        return
    empty = seeds - sum(goalie(field, robots, params, 1000 + s, "sideways", empty=True) for s in range(seeds))
    for style in ("sideways", "facing"):
        saves = sum(goalie(field, robots, params, 1000 + s, style) for s in range(seeds))
        raw[f"saves_{style}"] = saves / seeds
        out[f"goalie {style}: saves"] = f"{saves}/{seeds} ({100 * saves / seeds:.0f}%)"
    out["goalie: empty net scores"] = f"{empty}/{seeds}"
    ms = [match(field, robots, params, 2000 + s, minutes) for s in range(matches)]
    k = 3 / minutes / matches
    gb, gr = sum(m["goals"]["blue"] for m in ms) * k, sum(m["goals"]["red"] for m in ms) * k
    raw.update(goals=gb + gr, stalls=sum(m["stalls"] for m in ms),
               fastwz=statistics.mean(m["fastwz_pm"] for m in ms), side=statistics.mean(m["side_pm"] for m in ms))
    out[f"1v1 ({matches}x{minutes:g} min): goals/3 min blue-red (total)"] = f"{gb:.1f}-{gr:.1f} ({gb + gr:.1f})"
    out["1v1: deadlocks / traps"] = f"{raw['stalls']} / {sum(m['traps'] for m in ms)}"
    out["1v1: role flips /robot-min"] = f"{statistics.mean(m['flips_pm'] for m in ms):.1f}"
    out["smooth: wz rev / fast wz rev /min"] = (f"{statistics.mean(m['wzflip_pm'] for m in ms):.1f} / "
                                               f"{raw['fastwz']:.1f}")
    out["smooth: side flips /min, jerk"] = f"{raw['side']:.1f}, {statistics.mean(m['jerk'] for m in ms):.2f}"
    out["smooth: 1v1 wz rms, replans /min"] = (f"{statistics.mean(m['wzrms'] for m in ms):.2f}, "
                                              f"{statistics.mean(m['replans_pm'] for m in ms):.1f}")
    contacts += [c for m in ms for c in m["contacts"]]
    bumps = [b for m in ms for b in m["bumps"]]
    raw["contact_mean"] = statistics.mean(contacts) if contacts else 0.0
    out["contact m/s: mean / p90 / hard>0.45"] = (f"{raw['contact_mean']:.2f} / {_pct(contacts, 0.9):.2f} / "
                                                 f"{100 * sum(c > 0.45 for c in contacts) / max(1, len(contacts)):.0f}%")
    out["bumps /3 min, closing m/s mean / max"] = (f"{len(bumps) * k:.1f}, "
                                                   f"{statistics.mean(bumps) if bumps else 0:.2f} / {max(bumps, default=0):.2f}")
    out["pace: 1v1 mean speed m/s"] = f"{statistics.mean(m['speed'] for m in ms):.2f}"
    out["pace: gait switches /min, in band"] = (f"{statistics.mean(m['switch_pm'] for m in ms):.1f}, "
                                                f"{100 * statistics.mean(m['band'] for m in ms):.1f}%")


def run(seeds: int, errors: bool, minutes: float, matches: int, overrides: dict | None = None) -> list[str]:
    """One parameter set as text lines (kept for callers of the old interface)."""
    res = evaluate(seeds, errors, minutes, matches, overrides)
    return [f"{k:44s} {v}" for k, v in res.items() if not k.startswith("_")]


def table(cols: dict[str, dict]) -> list[str]:
    names = list(cols)
    rows = [k for k in next(iter(cols.values())) if not k.startswith("_")]
    w = max(18, *(len(n) + 2 for n in names))
    lines = [f"{'':44s}" + "".join(f"{n:>{w}s}" for n in names)]
    for r in rows:
        lines.append(f"{r:44s}" + "".join(f"{cols[n].get(r, '-'):>{w}s}" for n in names))
    return lines


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--matches", type=int, default=4)
    ap.add_argument("--minutes", type=float, default=3.0)
    ap.add_argument("--quick", action="store_true", help="10 seeds, 2 matches")
    ap.add_argument("--off", action="store_true", help="also run with errors OFF")
    ap.add_argument("--preset", nargs="*", default=["base"], metavar="NAME",
                    help="columns: 'base' (params.json) or fc/config/presets/NAME.json on top of it")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE", help="override params.json (JSON values)")
    ap.add_argument("--field", nargs="*", default=[], metavar="KEY=VALUE", help="override field.json (numbers)")
    ap.add_argument("--scen", nargs="*", default=["core", "human", "return", "moving"],
                    help="row groups: core human return moving")
    ap.add_argument("--floor", action="store_true", help="add the Nov 24 floor errors (FLOOR) to the sim's")
    ap.add_argument("--floor25", action="store_true", help="Nov 25 floor (FLOOR25: new ball, gait 0.88, cap 0.8)")
    ap.add_argument("--lag", nargs="*", type=float, default=[], metavar="S",
                    help="total lag(s) in s; one column per preset and lag")
    a = ap.parse_args()
    over = {}
    for kv in a.set:
        k, _, v = kv.partition("=")
        try:
            over[k] = json.loads(v)
        except json.JSONDecodeError:
            over[k] = v
    fover = {k: float(v) for k, _, v in (kv.partition("=") for kv in a.field)}
    if a.floor:
        over = {**FLOOR, **over}
    if a.floor25:
        over = {**FLOOR25, **over}
    if over:
        print("overrides:", over)
    if fover:
        print("field:", fover)
    if a.quick:
        a.seeds, a.matches = 10, 2
    t0 = time.time()
    names = a.preset or ["base"]
    presets = {n: load_preset(n) for n in names}
    if a.lag:
        presets = {f"{n}@{L:g}": {**pv, **lag_over(L)} for n, pv in presets.items() for L in a.lag}
    for errors in ((True, False) if a.off else (True,)):
        print(f"== errors {'ON ' if errors else 'OFF'} ==")
        cols = {n: evaluate(a.seeds, errors, a.minutes, a.matches, {**pv, **over}, field_over=fover,
                            scen=tuple(a.scen)) for n, pv in presets.items()}
        for line in table(cols):
            print(line)
    print(f"({time.time() - t0:.0f}s)")


if __name__ == "__main__":
    main()
