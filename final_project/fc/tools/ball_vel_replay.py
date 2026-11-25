"""Replay ball sightings from run logs through fc.eye.BallVelocity (no camera, stdlib only).

    cd final_project && python -m fc.tools.ball_vel_replay fc/runs/20261124T07*      # run folders
    python -m fc.tools.ball_vel_replay --synthetic                                    # known rolls

Run logs hold the Eye's ball at each 30 Hz tick: position, capture time (w.t - age) and the
velocity the app then had. Before BallVelocity the position was the raw sighting, so those logs
give raw tracks. Reported per run:
  * at rest: stretches of >= 1 s where the raw ball stays within 2 cm. Speed reported by the old
    estimator (logged) and by BallVelocity: median / p95 / max and the fraction reported non-zero.
  * rolls: free rolls found by fc.eye.fit_decel and their fitted deceleration.
--synthetic: rolls with known speed and deceleration at 31 fps: lag of the speed
estimate after release (time to 50 % / 90 % of the true speed) and error while rolling.
Noise: 3 mm plus 3 % single-frame 2 cm flickers (what the Nov 24 runs show at rest).
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

from ..ballvel import BallVelocity, fit_decel


def load_track(run: Path) -> tuple[list[tuple], list[tuple]]:
    """events.jsonl -> (raw [(t_cap, x, y)], logged [(t_cap, speed)]) with one entry per sighting."""
    raw, logged, seen = [], [], set()
    with open(run / "events.jsonl") as f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            w = r.get("w") or {}
            b = w.get("ball")
            if not b or w.get("t") is None or b.get("age") is None:
                continue
            t = round(w["t"] - b["age"], 4)
            if t in seen:
                continue
            seen.add(t)
            raw.append((t, b["x"], b["y"]))
            logged.append((t, math.hypot(b.get("vx") or 0.0, b.get("vy") or 0.0)))
    raw.sort()
    logged.sort()
    return raw, logged


def rest_spans(raw, min_s: float = 1.0, tol: float = 0.02) -> list[tuple[float, float]]:
    """(t0, t1) stretches where every sighting stays within tol of the stretch's first one."""
    spans, i = [], 0
    while i < len(raw):
        j = i
        while j + 1 < len(raw) and math.hypot(raw[j + 1][1] - raw[i][1], raw[j + 1][2] - raw[i][2]) < tol \
                and raw[j + 1][0] - raw[j][0] < 0.3:
            j += 1
        if raw[j][0] - raw[i][0] >= min_s:
            spans.append((raw[i][0], raw[j][0]))
        i = j + 1
    return spans


def stats(v: list[float]) -> str:
    if not v:
        return "-"
    v = sorted(v)
    q = lambda p: v[min(len(v) - 1, int(p * len(v)))]  # noqa: E731
    nz = sum(1 for x in v if x > 1e-9) / len(v)
    return f"med {q(0.5):.3f} p95 {q(0.95):.3f} max {v[-1]:.3f} m/s, non-zero {nz:.0%}"


def replay(run: Path, decel: float) -> dict:
    raw, logged = load_track(run)
    f = BallVelocity()
    est = [(p[0], math.hypot(*f.update(p[0], p[1], p[2], decel)[2:])) for p in raw]
    spans = rest_spans(raw)
    inside = lambda t: any(a + 0.4 <= t <= b for a, b in spans)  # noqa: E731 - skip the window's warm-up
    old = [s for t, s in logged if inside(t)]
    new = [s for t, s in est if inside(t)]
    roll = fit_decel(raw)
    return {"run": run.name, "sightings": len(raw), "rest_s": round(sum(b - a for a, b in spans), 1),
            "old": old, "new": new, "rolls": roll, "resets": f.resets, "outliers": f.outliers}


def synthetic(seed: int = 1, fps: float = 31.0, noise: float = 0.003, flicker: float = 0.03) -> list[dict]:
    """Ball at rest 1 s, released at v0 (instantly), rolling with decel a, until it stops. Sightings:
    `noise` m Gaussian plus, with probability `flicker`, a 2 cm jump (as seen at rest on Nov 24)."""
    rng = random.Random(seed)
    out = []
    for v0, a in ((0.5, 0.3), (1.0, 0.45), (1.5, 0.45), (2.0, 0.6)):
        f = BallVelocity()
        t, rows = 0.0, []
        while t < 1.0 + v0 / a + 1.0:
            tr = max(0.0, t - 1.0)
            tr = min(tr, v0 / a)
            s = v0 * tr - 0.5 * a * tr * tr
            v_true = max(0.0, v0 - a * max(0.0, t - 1.0)) if t >= 1.0 else 0.0
            x, y = -1.0 + 0.8 * s, 0.6 * s   # direction (0.8, 0.6)
            jx, jy = (0.02 * math.cos(k := rng.uniform(0, 6.283)), 0.02 * math.sin(k)) if rng.random() < flicker else (0, 0)
            ex, ey, vx, vy = f.update(t, x + jx + rng.gauss(0, noise), y + jy + rng.gauss(0, noise), a)
            rows.append((t, v_true, math.hypot(vx, vy), math.hypot(ex - x, ey - y)))
            t += 1.0 / fps
        rest = [r[2] for r in rows if r[0] < 1.0]
        t50 = next((r[0] - 1.0 for r in rows if r[0] >= 1.0 and r[2] >= 0.5 * r[1]), None)
        t90 = next((r[0] - 1.0 for r in rows if r[0] >= 1.0 and r[2] >= 0.9 * r[1]), None)
        rolling = [r for r in rows if 1.0 + 0.5 <= r[0] <= 1.0 + v0 / a - 0.3]
        err = sorted(abs(r[2] - r[1]) for r in rolling)
        perr = sorted(r[3] for r in rolling)
        stop = next((r[0] - (1.0 + v0 / a) for r in rows if r[0] > 1.0 + 0.3 and r[2] == 0.0), None)
        out.append({"v0": v0, "decel": a, "rest_max": max(rest), "t50": t50, "t90": t90,
                    "speed_err_med": err[len(err) // 2] if err else None, "speed_err_max": err[-1] if err else None,
                    "pos_err_med": perr[len(perr) // 2] if perr else None, "zero_after_stop_s": stop})
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", type=Path)
    ap.add_argument("--decel", type=float, default=0.45, help="ball_decel used for the slope correction")
    ap.add_argument("--synthetic", action="store_true")
    a = ap.parse_args(argv)
    if a.synthetic:
        for r in synthetic():
            print(f"v0 {r['v0']:.1f} m/s decel {r['decel']:.2f}: at rest max {r['rest_max']:.3f} m/s · lag to 50% "
                  f"{r['t50']:.2f} s, 90% {r['t90']:.2f} s · rolling speed err med {r['speed_err_med']:.3f} "
                  f"max {r['speed_err_max']:.3f} m/s · pos err med {r['pos_err_med'] * 100:.1f} cm"
                  f" · zero {r['zero_after_stop_s']:+.2f} s after the stop")
    allold, allnew = [], []
    for run in a.runs:
        if not (run / "events.jsonl").exists():
            continue
        r = replay(run, a.decel)
        allold += r["old"]
        allnew += r["new"]
        rolls = r["rolls"]
        print(f"{r['run']}: {r['sightings']} sightings, at rest {r['rest_s']} s, resets {r['resets']} outliers"
              f" {r['outliers']}, rolls {len(rolls['rolls'])} (used {rolls['n_used']}, decel {rolls['decel']})")
        for x in rolls["rolls"]:
            print(f"    roll v0 {x['v0']:.2f} m/s {x['secs']} s {x['dist_m']} m decel {x['decel']:.3f} rms "
                  f"{x['rms_m'] * 100:.1f} cm{'' if x['used'] else ' (not used)'}")
    if a.runs:
        print(f"AT REST old estimator: {stats(allold)}")
        print(f"AT REST BallVelocity : {stats(allnew)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
