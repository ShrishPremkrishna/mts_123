#!/usr/bin/env python3
"""Walk to a ball at known relative coordinates and kick it with one front leg.

The ball position comes from outside the robot: the team's field camera, a tape measure, or
`ball_follow.sh right --measure`. X is the ball centre's distance ahead of the robot's centre
(base_link) and Y its distance to the left, in metres. The program

  1. turns in place so that the ball lies on the kicking paw's line,
  2. walks straight until the ball is at the chest (walker model measured on this robot),
  3. plays the leg kick and returns to the walker.

All three steps are one job of the API's kick_at tool (lab/part9_ball_kick.py), so the kick
starts in the same tick in which the walk ends. This file holds the plan and the walker model
that the tool uses, and the command that calls it. No camera in the loop. Run it on the Pupper
while the Gemini Pupper API runs and the robot stands (stand_up):

    python3 kick_at.py 0.60 0.10                 # leg chosen from the side the ball is on
    python3 kick_at.py 0.60 0.10 --side right
    python3 kick_at.py 0.60 0.10 --dry-run       # turn and walk, no kick
    python3 kick_at.py 0.60 0.10 --fwd-gain 1.2  # walker travel per commanded metre on this floor

Ctrl-C, a refused tool call or an error sends Stop. Standard library only.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ball_seek import PAW_Y, Abort, HttpClient  # noqa: E402

TARGET_X = 0.248  # m: ball centre at the chest after the walk (kick contact is 0.258; a long walk
#                   pushes the ball and still kicks it, a short one can miss it)
FWD_GAIN = 1.31  # walker distance / commanded distance at 0.2 m/s (robot, 2026-11-18)
DEAD_S = 0.14  # walker start delay, s (robot, 2026-11-18)
SPEED = 0.2  # m/s
TURN_GAIN = 1.0  # the API turn tool is timed at 60 deg/s; not measured
MIN_TURN_DEG = 3.0
MIN_WALK_M = 0.01
MAX_RANGE_M = 3.0


def choose_side(y: float, side: str) -> str:
    if side in PAW_Y:
        return side
    if side != "auto":
        raise ValueError("side must be right, left or auto")
    return "left" if y > 0 else "right"


def plan(x: float, y: float, side: str) -> tuple[float, float]:
    """(turn in degrees, + is left; walk in metres, + is forward) that put the ball centre at
    (TARGET_X, paw line) in front of the robot."""
    paw = PAW_Y[side]
    r, phi = math.hypot(x, y), math.atan2(y, x)
    if not math.isfinite(r) or r > MAX_RANGE_M:
        raise ValueError(f"ball {r:.2f} m away: farther than {MAX_RANGE_M} m")
    if r <= abs(paw) + 0.05:
        raise ValueError("ball under the robot: move it out first")
    # Turning by theta (counter-clockwise) moves the ball to angle phi - theta. On the paw
    # line, r * sin(phi - theta) = paw.
    theta = phi - math.asin(paw / r)
    x_after = r * math.cos(phi - theta)
    return math.degrees(theta), x_after - TARGET_X


def run(client, x: float, y: float, side: str, dry_run: bool = False, log=print,
        gain: float = FWD_GAIN, delay: float = DEAD_S) -> str:
    side = choose_side(y, side)
    turn_deg, walk_m = plan(x, y, side)  # the tool makes the same plan; this one is for the log and refusals
    log(f"ball x {x:.3f} y {y:+.3f}: {side} leg; turn {turn_deg:+.1f} deg, walk {walk_m:+.3f} m"
        + (" (dry run)" if dry_run else ", then kick"))
    return client.kick_at(x, y, side, dry_run, gain, delay) or f"kicked with the {side} leg"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("x", type=float, help="ball centre ahead of the robot centre, m")
    ap.add_argument("y", type=float, help="ball centre left of the robot centre, m (right is negative)")
    ap.add_argument("--side", default="auto", choices=["right", "left", "auto"])
    ap.add_argument("--dry-run", action="store_true", help="turn and walk, no kick")
    ap.add_argument("--fwd-gain", type=float, default=FWD_GAIN, help="walker travel per commanded metre at 0.2 m/s")
    ap.add_argument("--dead", type=float, default=DEAD_S, help="walker start delay, s")
    ap.add_argument("--url", default="http://127.0.0.1:18090")
    args = ap.parse_args(argv)
    stamp = lambda text: print(time.strftime("%H:%M:%S"), text, flush=True)
    client = HttpClient(args.url)
    try:
        stamp("DONE: " + run(client, args.x, args.y, args.side, args.dry_run, stamp, args.fwd_gain, args.dead))
        return 0
    except ValueError as error:
        stamp(f"REFUSED: {error}")
        return 2
    except KeyboardInterrupt:
        client.stop()
        stamp("STOPPED (Ctrl-C): Stop sent")
        return 130
    except Abort as error:
        client.stop()
        stamp(f"ABORT: {error} (Stop sent)")
        return 1
    except Exception as error:  # an unexpected fault must still stop the robot
        client.stop()
        stamp(f"ERROR: {type(error).__name__} {error} (Stop sent)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
