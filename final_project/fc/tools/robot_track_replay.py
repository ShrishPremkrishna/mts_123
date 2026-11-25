"""Robot tracking replay on recorded runs: how often the Eye has a fresh robot position (decoded tag, or the
coloured tag board when the tag is blurred), and how good the board fixes and carried heading are.

    .venv-fc/bin/python -m fc.tools.robot_track_replay fc/runs/20261125T09*-push_ball fc/runs/20261125T090717Z-match
    .venv-fc/bin/python -m fc.tools.robot_track_replay RUN --no-blob          # tag only (before)
    .venv-fc/bin/python -m fc.tools.robot_track_replay RUN --colors auto      # board colour from the first tag fix

Each frame of overhead.mp4 (15 fps, 1280 px; upscaled to --width like eye_bench) goes through Eye.process();
the background scan runs inline every 4th frame. The commands in events.jsonl are fed to Eye.note_cmds() as the
app does live. Video time is matched to the run log by delta (s): frame i was captured at run time
i/15 - delta, fitted from the replay's tag fixes against the live tag observations (fallback 0.16).

Reference ("truth") is the live tag observations in events.jsonl (the 1920 px stream decoded the tag far more
often than this recording does), interpolated between observations at most 0.12 s apart.
  coverage   share of calibrated frames with a fresh fix (tag / board), overall and while steadily fast
             (commanded vx >= 0.59 over the 0.5..0.15 s before capture); "live tag" = the live run's own
             tag rate in 1/30 s slots (the before number of the floor session)
  pos err    board fix vs the live tag pose (same capture time) | vs the replay tag on the same frame
  yaw err    carried heading at board fixes vs the live tag heading | vs the next replay tag fix (<= 0.2 s on)
  outliers   board fixes more than 8 cm from the live tag track; distractors = board-sized blobs of the robot's
             colour in the field besides the robot's own (checked every 15th frame)
Recorded video is softer than the live picture, so the replay's tag rate is a pessimistic floor.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys
import time
from pathlib import Path

FC = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(FC.parent))

import numpy as np  # noqa: E402

from fc import eye as E  # noqa: E402
from fc.app import PARAM_FALLBACKS, load_robot_config  # noqa: E402
from fc.core import load_field, load_params, wrap  # noqa: E402
from fc.tools.eye_bench import FileFrames  # noqa: E402

FPS = 15.0
T0 = 1000.0
PARAMS_EXTRA: dict = {}   # --set k=v (eye params to try)


def load_run(run: Path, rid: str):
    """(commands [(dt, vx, vy, wz)], live obs [(dt_cap, x, y, yaw)], run params)."""
    cmds, obs = [], {}
    for line in open(run / "events.jsonl"):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        c = (d.get("c") or {}).get(rid)
        if c:
            cmds.append((d["dt"], float(c[0]), float(c[1]), float(c[2])))
        w = d.get("w") or {}
        r = (w.get("robots") or {}).get(rid)
        if r:
            obs[round(d["dt"] - r["age"], 3)] = (r["x"], r["y"], r["yaw"])
    pj = json.loads((run / "params.json").read_text())
    lt = np.array(sorted(obs))
    return cmds, (lt, np.array([obs[k] for k in lt]).reshape(-1, 3)), pj.get("params", {})


def interp_live(live, t, max_gap=0.12):
    """Live tag pose at run time t, interpolated between observations <= max_gap apart, else None."""
    lt, lp = live
    j = int(np.searchsorted(lt, t))
    if j <= 0 or j >= len(lt) or lt[j] - lt[j - 1] > max_gap:
        return None
    a = (t - lt[j - 1]) / (lt[j] - lt[j - 1])
    yaw = lp[j - 1, 2] + a * wrap(lp[j, 2] - lp[j - 1, 2])
    return lp[j - 1, 0] + a * (lp[j, 0] - lp[j - 1, 0]), lp[j - 1, 1] + a * (lp[j, 1] - lp[j - 1, 1]), wrap(yaw)


def motion(cmds_arr, t):
    """'fast' / 'spin' / 'slow' / 'still' from the commands 0.5..0.15 s before run time t."""
    if not len(cmds_arr):
        return "?"
    m = (cmds_arr[:, 0] >= t - 0.5) & (cmds_arr[:, 0] <= t - 0.15)
    if m.sum() < 5:
        return "?"
    vx, wz = cmds_arr[m, 1], np.abs(cmds_arr[m, 3])
    if vx.min() >= 0.59:
        return "fast"
    if vx.max() < 0.1 and wz.min() > 1.0:
        return "spin"
    if vx.max() < 0.05 and wz.max() < 0.3:
        return "still"
    return "other"


def live_tag_rate(live, cmds_arr, t0, t1):
    """Before: the live run's share of 1/30 s slots with a tag observation, by motion class."""
    out = {}
    lt = live[0]
    for t in np.arange(t0 + 0.6, t1 - 0.2, 1 / 30):
        k = motion(cmds_arr, t)
        seen = len(lt) and np.min(np.abs(lt - t)) < 1 / 60
        a = out.setdefault(k, [0, 0])
        a[0] += int(bool(seen))
        a[1] += 1
    return out


def make_eye(params, colors_path, video):
    eye = E.Eye(load_field(), load_robot_config()[0], params, source=str(video), robot_colors_path=colors_path)
    return eye


def replay(run: Path, args) -> dict:
    rid = "blue"
    cmds, live, rparams = load_run(run, rid)
    params = {**PARAM_FALLBACKS, **load_params(), **rparams}
    params["eye_blob"] = 0 if args.no_blob else 1
    params.update(PARAMS_EXTRA)
    frames = FileFrames(run / "overhead.mp4", args.width)
    colors_path = None if args.colors == "auto" else Path(args.colors)
    eye = make_eye(params, colors_path, run / "overhead.mp4")
    eye._open_source = lambda: type("S", (), {"latency": False, "width": frames.width,
                                              "height": frames.height, "close": lambda s: None})()
    eye._setup()
    recs, n, ci, next_deep, ms = [], 0, 0, 0.0, []
    delta = args.delta if args.delta is not None else 0.16
    cmds_arr = np.array(cmds) if cmds else np.zeros((0, 4))
    distract = []
    while args.max_frames is None or n < args.max_frames:
        f = frames.next()
        if f is None:
            break
        t = T0 + n / FPS
        while ci < len(cmds) and T0 + cmds[ci][0] + delta <= t + 0.15:   # what the app had sent by now
            c = cmds[ci]
            eye.note_cmds(T0 + c[0] + delta, {rid: c[1:]})
            ci += 1
        t1 = time.perf_counter()
        eye._tm = {}
        w = eye.process(f, t)
        ms.append(((time.perf_counter() - t1) * 1000, eye._tm.get("blob", 0.0) * 1000))
        if n % 4 == 0 and eye._tracking():
            next_deep = eye.scan_once(f, t, next_deep, now=t)
        if args.colors == "auto" and rid not in eye.robot_colors and w is not None:
            tag = eye.perc.debug.get("tags", {}).get(10)
            if tag is not None:
                from fc import ballcal
                eye.robot_colors[rid] = ballcal.hsv_ranges(E.board_ring_pixels(f, tag))
                print(f"  auto colour at frame {n}: {eye.robot_colors[rid]}")
        o = w.robots.get(rid) if w is not None else None
        fresh = o is not None and o.t == t
        rec = {"n": n, "cal": w is not None, "src": o.src if fresh else None,
               "pose": (o.pose.x, o.pose.y, o.pose.yaw) if fresh else None, "conf": o.yaw_conf if fresh else None,
               "tdt": n / FPS - delta}
        if fresh and o.src == "blob":
            tr = eye._tracks[rid]
            rec["dbg"] = {**tr.last, "since_tag": t - tr.t_tag, "yaw_pre": None}
        if fresh and o.src == "tag" and rid in eye.robot_colors:   # same-frame board fix (independent)
            rec["board"] = board_check(eye, f, rid, o)
        if w is not None and n % 15 == 0 and rid in eye.robot_colors:
            distract.append(distractors(eye, f, rid, o.pose if o is not None else None))
        recs.append(rec)
        n += 1
    eye._stop = True
    return {"run": run.name, "recs": recs, "live": live, "cmds": cmds_arr, "delta": delta, "ms": ms,
            "distract": distract, "tracks": {k: dict(v.n, off=v.off, area_k=round(v.area_k, 2))
                                              for k, v in eye._tracks.items()}}


def board_check(eye, frame, rid, o):
    """Board-only body position on a frame whose tag was decoded (uses the tag heading), for comparison."""
    tr, cam, mount = eye._tracks[rid], eye.perc.cam, eye.mount()
    h = mount["tag_height"]
    tag = eye.perc.debug["tags"][10]
    c = tag.mean(axis=0)
    exp, side = eye._board_px(cam, c, h)
    b = eye._board_in(frame, eye.robot_colors[rid], c, exp * tr.area_k, 0.5 * side)
    if b is None:
        return None
    bx, by = cam.pixel_to_plane([b[:2]], h)[0]
    yaw, ox = o.pose.yaw, mount["tag_offset_x"]
    cs, sn = math.cos(yaw), math.sin(yaw)
    cx, cy = bx + cs * tr.off[0] - sn * tr.off[1], by + sn * tr.off[0] + cs * tr.off[1]
    return float(cx - cs * ox), float(cy - sn * ox)


def distractors(eye, frame, rid, pose):
    """Board-sized blobs of the robot's colour in the field besides one (the robot's own board)."""
    roi = eye.perc.roi_mask
    return max(0, eye._board_candidates(frame, roi, eye.robot_colors[rid]) - 1)


def fit_delta(res) -> float | None:
    pts = [(r["n"] / FPS, *r["pose"]) for r in res["recs"] if r["src"] == "tag"]
    if len(pts) < 5:
        return None
    pts = np.array(pts)
    best = None
    for d in np.arange(0.0, 0.4, 0.01):
        e = [math.hypot(q[0] - p[1], q[1] - p[2]) for p in pts if (q := interp_live(res["live"], p[0] - d, 0.08))]
        if len(e) >= 4 and (best is None or np.median(e) < best[0]):
            best = (float(np.median(e)), float(d))
    return None if best is None else best[1]


def summarize(results: list[dict]) -> dict:
    cov = {}
    pos_live, pos_same, yaw_live, yaw_next, out, ms, dis = [], [], [], [], 0, [], []
    yaw_fast, yaw_next_fast, bms = [], [], []
    live_before = {}
    for res in results:
        recs, live, cmds = res["recs"], res["live"], res["cmds"]
        ms += [m[0] for m in res["ms"][5:]]
        bms += [m[1] for m in res["ms"][5:]]
        dis += res["distract"]
        for k, v in live_tag_rate(live, cmds, 0.0, cmds[-1, 0] if len(cmds) else 0.0).items():
            a = live_before.setdefault(k, [0, 0])
            a[0] += v[0]
            a[1] += v[1]
        for i, r in enumerate(recs):
            if not r["cal"]:
                continue
            k = motion(cmds, r["tdt"])
            for key in ("all", k):
                a = cov.setdefault(key, {"n": 0, "tag": 0, "blob": 0})
                a["n"] += 1
                if r["src"]:
                    a[r["src"]] += 1
            if r["src"] == "blob":
                q = interp_live(live, r["tdt"])
                if q is not None:
                    e = math.hypot(r["pose"][0] - q[0], r["pose"][1] - q[1])
                    pos_live.append(e)
                    out += e > 0.08
                    yaw_live.append(abs(math.degrees(wrap(r["pose"][2] - q[2]))))
                    if k == "fast":
                        yaw_fast.append(yaw_live[-1])
                nxt = next((s for s in recs[i + 1:i + 4] if s["src"] == "tag"), None)
                if nxt is not None:
                    yaw_next.append(abs(math.degrees(wrap(r["pose"][2] - nxt["pose"][2]))))
                    if k == "fast":
                        yaw_next_fast.append(yaw_next[-1])
            if r["src"] == "tag" and r.get("board"):
                pos_same.append(math.hypot(r["board"][0] - r["pose"][0], r["board"][1] - r["pose"][1]))

    def st(v, scale=1.0):
        v = np.asarray(v) * scale
        return (f"median {np.median(v):.1f}  p90 {np.percentile(v, 90):.1f}  max {v.max():.1f}  (n {len(v)})"
                if len(v) else "-")
    lines = []
    for k in ("all", "fast", "other", "spin", "still"):
        a = cov.get(k)
        if a and a["n"]:
            lb = live_before.get(k) if k != "all" else [sum(v[0] for v in live_before.values()),
                                                        sum(v[1] for v in live_before.values())]
            lines.append(f"  {k:6s} frames {a['n']:5d}: tag {a['tag'] / a['n']:5.1%}  board {a['blob'] / a['n']:5.1%}"
                         f"  -> tracked {(a['tag'] + a['blob']) / a['n']:5.1%}"
                         + (f"   | live tag (floor) {lb[0] / lb[1]:5.1%}" if lb and lb[1] else ""))
    lines += [f"  pos err vs live tag (cm):   {st(pos_live, 100)}",
              f"  pos err vs same-frame tag:  {st(pos_same, 100)}",
              f"  yaw err vs live tag (deg):  {st(yaw_live)}",
              f"  yaw err vs next tag (deg):  {st(yaw_next)}",
              f"    while fast: vs live {st(yaw_fast)} | vs next tag {st(yaw_next_fast)}",
              f"  outliers (> 8 cm from live): {out} of {len(pos_live)}",
              f"  distractor blobs in field: {sum(dis)} in {len(dis)} checked frames",
              f"  Eye.process ms: {st(ms)}",
              f"  of which board tracking ms: {st(bms)}"]
    return {"lines": lines, "cov": cov}


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--no-blob", action="store_true", help="tag only (the Eye before board tracking)")
    ap.add_argument("--colors", default=str(E.ROBOT_COLORS_PATH), help="robot_colors.json, or 'auto'")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--delta", type=float, default=None, help="video->log time offset (default: fitted)")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--set", nargs="*", default=[], metavar="K=V", help="eye params to try, e.g. eye_travel_tau_s=0")
    a = ap.parse_args(argv)
    PARAMS_EXTRA.update({k: float(v) for k, v in (kv.split("=", 1) for kv in a.set)})
    runs = [Path(p) for r in a.runs for p in sorted(glob.glob(r)) if (Path(p) / "overhead.mp4").is_file()]
    results = []
    for run in runs:
        res = replay(run, a)
        d = fit_delta(res) if a.delta is None else a.delta
        if d is not None and abs(d - res["delta"]) > 0.005:   # re-run with the fitted offset (commands too)
            a2 = argparse.Namespace(**{**vars(a), "delta": d})
            res = replay(run, a2)
        results.append(res)
        s = summarize([res])
        print(f"{run.name}  delta {res['delta']:.2f}s  tracks {res['tracks']}")
        print("\n".join(s["lines"][:1] + [ln for ln in s["lines"][1:5] if "fast" in ln]))
    print(f"\nALL ({len(results)} runs, {'tag only' if a.no_blob else 'tag + board'})")
    print("\n".join(summarize(results)["lines"]))


if __name__ == "__main__":
    main()
