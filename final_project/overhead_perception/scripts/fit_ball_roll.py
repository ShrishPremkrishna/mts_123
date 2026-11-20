#!/usr/bin/env python3
"""Measure how fast the real ball slows down on the real floor: the one number the roll prediction needs.

Run the camera pipeline (live, or replaying a recording), then in a second terminal:

  python scripts/fit_ball_roll.py

Roll the ball by hand across the field a few times and let it stop on its own, away
from the robots and the edges. Each roll is fitted with "slows at a constant rate" and
printed; Ctrl-C (or --seconds) prints the value to put in config/field.yaml:

  prediction:
    roll_decel: <value>

Read the per-roll lines, not only the summary. If fast rolls give a clearly larger value
than slow ones, or the fit error is several centimetres, a constant rate does not describe
this ball on this floor and the predictions will be off by about that much.
"""
import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.ballistics import RollModel, RollTracker, fit_decel  # noqa: E402
from fieldcam.config import load_config  # noqa: E402
from fieldcam.net import StateSubscriber  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=None, help="UDP port (default: net.port in the config)")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = until Ctrl-C)")
    ap.add_argument("--skip", type=int, default=2, help="measurements dropped at each end of a roll (the push and the last wobble)")
    args = ap.parse_args()

    cfg = load_config()
    sub = StateSubscriber(args.port or cfg["net"]["port"])
    tracker = RollTracker(RollModel.from_config(cfg))
    roll, fits, last_t, t0 = [], [], None, time.time()
    print("listening. Roll the ball and let it stop by itself; Ctrl-C when done.", flush=True)

    def finish():
        nonlocal roll
        fit = fit_decel(roll[args.skip:len(roll) - args.skip]) if len(roll) > 2 * args.skip else None
        roll = []
        if fit is None:
            return
        if fit["decel"] <= 0:
            print(f"  roll ignored: it did not slow down (fit {fit['decel']:+.2f} m/s^2); pushed or on a slope?")
            return
        fits.append(fit)
        print(f"  roll {len(fits)}: started at {fit['v0']:.2f} m/s, {fit['distance']:.2f} m in {fit['duration']:.1f} s, "
              f"{fit['points']} measurements -> slows at {fit['decel']:.2f} m/s^2 (fit error {fit['rms_m'] * 100:.1f} cm)", flush=True)

    try:
        while not args.seconds or time.time() - t0 < args.seconds:
            state = sub.latest()
            ball = (state or {}).get("ball") or {}
            when = ball.get("measured_at")
            if not ball.get("measured_this_frame") or when is None or when == last_t or not state.get("calibration_valid"):
                time.sleep(0.005)
                continue
            last_t = when
            back = state["stamp"] - when                      # the published position is extrapolated to "now": undo that
            xy = (ball["x"] - ball["vx"] * back, ball["y"] - ball["vy"] * back)
            tracker.update(xy, when)
            est = tracker.estimate(when)
            if est and est["moving"] and len(tracker.hist) >= 3:
                roll.append((when, *xy))
            elif roll:
                finish()
    except KeyboardInterrupt:
        pass
    if roll:
        finish()
    if not fits:
        print("\nno complete roll seen. Is the pipeline running and calibrated, and did the ball roll for about a second?")
        return
    values = sorted(f["decel"] for f in fits)
    print(f"\n{len(fits)} roll(s): slows at {statistics.median(values):.2f} m/s^2 (median; range {values[0]:.2f} to {values[-1]:.2f})")
    print("put this in config/field.yaml:\n\nprediction:\n  roll_decel: " + f"{statistics.median(values):.2f}")
    if len(fits) < 3:
        print("\nonly a few rolls: do at least five, at different speeds, before trusting it.")


if __name__ == "__main__":
    main()
