"""Batch validation of brain + sim. Run from final_project/:
    python -m fc.sim_eval [--seeds N] [--quick] [--off] [--set key=value ...]

Prints one compact table per error setting (errors ON = measured model, OFF = ideal).
`--set` overrides params.json for this run (e.g. to try a value before committing it).
1v1 smoothness: "fast wz rev" = turn reversals (|wz| > 0.25 one way, then the other) less
than 1 s apart; "side flips" = changes of the way round the ball while orbiting. Both per
robot-minute.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time

from .brain import Brain
from .core import load_field, load_params, load_robots
from .sim import Sim

DT = 1.0 / 30


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


def solo(field, robots, params, seed, mode, limit=40.0):
    p = {**params, "contact_mode": mode, "role_mode": "auto"}
    sim = Sim(field, robots, p, seed)
    sim.reset("attack_solo")
    me = robots[0]
    brains = {me.id: Brain(me, field)}
    _loop(sim, brains, p, limit, lambda c, d: bool(_goals(sim)))
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
          "stall": 0, "anchor": (0.0, 0.0, 0.0), "side": {}, "sideflip": 0}

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
            "traps": sum(e["type"] == "trapped" for e in sim.events)}


def run(seeds: int, errors: bool, minutes: float, matches: int, overrides: dict | None = None) -> list[str]:
    field, robots, params = load_field(), load_robots(), load_params()
    params = {**params, **(overrides or {}), "sim_errors": int(errors)}
    out = []
    for mode in ("push", "headbutt"):
        res = [solo(field, robots, params, s, mode) for s in range(seeds)]
        times = [t for t, _ in res if t is not None]
        own = sum(o for _, o in res)
        med = f"{statistics.median(times):5.1f}s" if times else "   - "
        out.append(f"solo {mode:9s} goal {len(times):3d}/{seeds} ({100 * len(times) / seeds:3.0f}%)  "
                   f"median {med}  own-goals {own}")
    empty = seeds - sum(goalie(field, robots, params, 1000 + s, "sideways", empty=True) for s in range(seeds))
    for style in ("sideways", "facing"):
        saves = sum(goalie(field, robots, params, 1000 + s, style) for s in range(seeds))
        out.append(f"goalie {style:8s} saves {saves:3d}/{seeds} ({100 * saves / seeds:3.0f}%)  "
                   f"[same shots, empty net: {empty}/{seeds} score]")
    ms = [match(field, robots, params, 2000 + s, minutes) for s in range(matches)]
    gb = sum(m["goals"]["blue"] for m in ms) / matches
    gr = sum(m["goals"]["red"] for m in ms) / matches
    out.append(f"1v1 auto ({matches}x{minutes:g}min) goals/3min blue {gb * 3 / minutes:4.1f} red {gr * 3 / minutes:4.1f}  "
               f"role flips/min {statistics.mean(m['flips_pm'] for m in ms):4.1f}  "
               f"stalls {sum(m['stalls'] for m in ms)} traps {sum(m['traps'] for m in ms)}")
    out.append(f"1v1 smoothness /robot-min: wz reversals {statistics.mean(m['wzflip_pm'] for m in ms):4.1f}  "
               f"fast wz rev (<1 s) {statistics.mean(m['fastwz_pm'] for m in ms):4.1f}  "
               f"side flips {statistics.mean(m['side_pm'] for m in ms):4.1f}  jerk {statistics.mean(m['jerk'] for m in ms):4.2f}")
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--matches", type=int, default=4)
    ap.add_argument("--minutes", type=float, default=3.0)
    ap.add_argument("--quick", action="store_true", help="10 seeds, 2 matches")
    ap.add_argument("--off", action="store_true", help="also run with errors OFF")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE", help="override params.json (JSON values)")
    a = ap.parse_args()
    over = {}
    for kv in a.set:
        k, _, v = kv.partition("=")
        try:
            over[k] = json.loads(v)
        except json.JSONDecodeError:
            over[k] = v
    if over:
        print("overrides:", over)
    if a.quick:
        a.seeds, a.matches = 10, 2
    t0 = time.time()
    for errors in ((True, False) if a.off else (True,)):
        print(f"== errors {'ON ' if errors else 'OFF'} ==")
        for line in run(a.seeds, errors, a.minutes, a.matches, over):
            print(line)
    print(f"({time.time() - t0:.0f}s)")


if __name__ == "__main__":
    main()
