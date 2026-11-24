"""Batch validation of brain + sim. Run from final_project/:
    python -m fc.sim_eval [--seeds N] [--matches M] [--quick] [--off] [--preset NAME ...] [--set key=value ...]
                          [--field key=value ...]

Prints one table per error setting (errors ON = measured model, OFF = ideal), one column per
preset: `base` is params.json, any other name is params.json + fc/config/presets/NAME.json
(e.g. `--preset base fast-play` compares them side by side). `--set` overrides on top of every
column (to try a value before committing it). `--field` overrides field.json geometry the same way
(e.g. `--field goal_width=0.8` to try a wider marked goal).

Rows:
- solo push/headbutt: goals within 40 s from a random start, median time to goal (s).
- goalie: saves of the same shots that score on an empty net.
- 1v1 (auto roles, kickoff): goals per 3 min each side; deadlocks = the ball stayed within 10 cm
  for 10 s; traps = the sim referee nudged a wall ball.
- smoothness per robot-minute: wz reversals (|wz| > 0.25 one way, then the other); fast wz rev =
  reversals less than 1 s apart; side flips = changes of the way round the ball while orbiting.
- contact: true robot body speed at each first touch of the ball (solo push + 1v1); hard =
  touches above 0.45 m/s. bumps = robot-robot first touches (closing speed).
- pace: mean true robot speed in 1v1; gait switches /robot-min; "in band" = share of ticks with a
  forward command inside (0.45, 0.6), where the walker hovers at its gait switch.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import statistics
import time

from .brain import Brain
from .core import CONFIG, load_field, load_params, load_robots
from .sim import Sim

DT = 1.0 / 30
PRESETS = CONFIG / "presets"
BAND = (0.45, 0.6)


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
    p = {**params, "contact_mode": mode, "role_mode": "auto"}
    sim = Sim(field, robots, p, seed)
    sim.reset("attack_solo")
    me = robots[0]
    brains = {me.id: Brain(me, field)}
    _loop(sim, brains, p, limit, lambda c, d: bool(_goals(sim)))
    if contacts is not None:
        contacts.extend(c[2] for c in sim.contacts)
    ev = _goals(sim)[0] if _goals(sim) else None
    return (ev["t"] if ev and ev["team"] == me.team else None), bool(ev and ev["team"] != me.team)


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
            "band": st["band"] / max(1, st["ticks"])}


def _pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else 0.0


def evaluate(seeds: int, errors: bool, minutes: float, matches: int, overrides: dict | None = None,
             headbutt: bool = True, field_over: dict | None = None) -> dict:
    """Metrics for one parameter set: {row label: formatted value} plus raw numbers under '_raw'."""
    field, robots, params = load_field(), load_robots(), load_params()
    if field_over:
        field = dataclasses.replace(field, **field_over)
    params = {**params, **(overrides or {}), "sim_errors": int(errors)}
    out, raw, contacts = {}, {}, []
    for mode in ("push", "headbutt") if headbutt else ("push",):
        res = [solo(field, robots, params, s, mode, contacts=contacts if mode == "push" else None)
               for s in range(seeds)]
        times = [t for t, _ in res if t is not None]
        med = statistics.median(times) if times else float("nan")
        raw[f"solo_{mode}_rate"], raw[f"solo_{mode}_med"] = len(times) / seeds, med
        out[f"solo {mode}: goals (<=40 s)"] = f"{len(times)}/{seeds} ({100 * len(times) / seeds:.0f}%)"
        out[f"solo {mode}: median s / own goals"] = f"{med:.1f} / {sum(o for _, o in res)}"
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
    out["_raw"] = raw
    return out


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
    a = ap.parse_args()
    over = {}
    for kv in a.set:
        k, _, v = kv.partition("=")
        try:
            over[k] = json.loads(v)
        except json.JSONDecodeError:
            over[k] = v
    fover = {k: float(v) for k, _, v in (kv.partition("=") for kv in a.field)}
    if over:
        print("overrides:", over)
    if fover:
        print("field:", fover)
    if a.quick:
        a.seeds, a.matches = 10, 2
    t0 = time.time()
    names = a.preset or ["base"]
    presets = {n: load_preset(n) for n in names}
    for errors in ((True, False) if a.off else (True,)):
        print(f"== errors {'ON ' if errors else 'OFF'} ==")
        cols = {n: evaluate(a.seeds, errors, a.minutes, a.matches, {**pv, **over}, field_over=fover)
                for n, pv in presets.items()}
        for line in table(cols):
            print(line)
    print(f"({time.time() - t0:.0f}s)")


if __name__ == "__main__":
    main()
