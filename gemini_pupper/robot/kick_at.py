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

With --angle or --point robot the program goes to a pose and kicks there: turn to the point, walk
straight to it, turn to the kick heading, kick. X Y is then where the robot stands (--point robot,
no ball necessary) or where the ball is (--point ball: the robot stops behind the ball on the kick
heading). The angle is in degrees from the heading at the start, left positive:

    python3 kick_at.py 2 -1 --ft --point robot --angle 0 --side right    # 2 ft ahead, 1 ft right
    python3 kick_at.py 0.8 0.3 --point ball --angle -20                  # shoot 20 deg to the right

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
FOOT_M = 0.3048
POINTS = ("ball", "robot")


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


def wrap_deg(angle: float) -> float:
    """The same direction in (-180, 180]."""
    return 0.0 - ((-angle + 180.0) % 360.0 - 180.0)


def plan_pose(x: float, y: float, angle_deg: float | None, side: str, point: str = "robot") -> tuple[float, float, float]:
    """(first turn in degrees, walk in metres, second turn in degrees) for a kick at a pose:
    turn to the stand point, walk straight to it, turn to the kick heading.

    point "robot": the robot centre goes to (x, y). point "ball": the ball centre is at (x, y), and
    the robot stops behind it on the kick heading, with the ball at (TARGET_X, paw line). The kick
    heading is angle_deg from the heading at the start; None is the direction to the point."""
    if point not in POINTS:
        raise ValueError("point must be ball or robot")
    if angle_deg is None:
        angle_deg = math.degrees(math.atan2(y, x))
    if not math.isfinite(angle_deg) or abs(angle_deg) > 180.0:
        raise ValueError("angle must be from -180 to 180 degrees")
    if point == "ball":
        h, paw = math.radians(angle_deg), PAW_Y[side]
        x -= TARGET_X * math.cos(h) - paw * math.sin(h)
        y -= TARGET_X * math.sin(h) + paw * math.cos(h)
    r = math.hypot(x, y)
    if not math.isfinite(r) or r > MAX_RANGE_M:
        raise ValueError(f"point {r:.2f} m away: farther than {MAX_RANGE_M} m")
    if r < MIN_WALK_M:
        return 0.0, 0.0, wrap_deg(angle_deg)
    first = math.degrees(math.atan2(y, x))
    return first, r, wrap_deg(angle_deg - first)


def steps(x: float, y: float, side: str, angle_deg: float | None = None, point: str = "ball") -> list[tuple[str, float]]:
    """The motion before the kick as ("turn", degrees) and ("walk", metres) steps. With no angle and
    point "ball", plan() (one turn onto the paw line, then the walk); else plan_pose()."""
    if angle_deg is None and point == "ball":
        turn_deg, walk_m = plan(x, y, side)
        return [("turn", turn_deg), ("walk", walk_m)]
    first, walk_m, second = plan_pose(x, y, angle_deg, side, point)
    return [("turn", first), ("walk", walk_m), ("turn", second)]


def run(client, x: float, y: float, side: str, dry_run: bool = False, log=print,
        gain: float = FWD_GAIN, delay: float = DEAD_S, angle: float | None = None, point: str = "ball",
        speed: float = SPEED, settle: float | None = None) -> str:
    side = choose_side(y, side)
    path = steps(x, y, side, angle, point)  # the tool makes the same plan; this one is for the log and refusals
    log(f"{point} x {x:.3f} y {y:+.3f}: {side} leg; "
        + ", ".join(f"{kind} {value:+.1f} deg" if kind == "turn" else f"walk {value:+.3f} m" for kind, value in path)
        + (" (dry run)" if dry_run else ", then kick"))
    extra = {k: v for k, v in (("angle", angle), ("settle", settle)) if v is not None}
    if point != "ball":
        extra["point"] = point
    if speed != SPEED:
        extra["speed"] = speed
    return client.kick_at(x, y, side, dry_run, gain, delay, **extra) or f"kicked with the {side} leg"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("x", type=float, help="point ahead of the robot centre, m")
    ap.add_argument("y", type=float, help="point left of the robot centre, m (right is negative)")
    ap.add_argument("--side", default="auto", choices=["right", "left", "auto"])
    ap.add_argument("--angle", type=float, help="kick heading, degrees from the heading at the start, left positive")
    ap.add_argument("--point", default="ball", choices=list(POINTS),
                    help="X Y is the ball centre (default) or where the robot stands")
    ap.add_argument("--ft", action="store_true", help="X and Y are in feet")
    ap.add_argument("--speed", type=float, default=SPEED, help="walk speed command, m/s (the gain was measured at 0.2)")
    ap.add_argument("--settle", type=float, help="zero-speed time before the kick, s (default 0.15)")
    ap.add_argument("--dry-run", action="store_true", help="turn and walk, no kick")
    ap.add_argument("--fwd-gain", type=float, default=FWD_GAIN, help="walker travel per commanded metre at 0.2 m/s")
    ap.add_argument("--dead", type=float, default=DEAD_S, help="walker start delay, s")
    ap.add_argument("--url", default="http://127.0.0.1:18090")
    args = ap.parse_args(argv)
    stamp = lambda text: print(time.strftime("%H:%M:%S"), text, flush=True)
    client = HttpClient(args.url)
    scale = FOOT_M if args.ft else 1.0
    try:
        stamp("DONE: " + run(client, args.x * scale, args.y * scale, args.side, args.dry_run, stamp, args.fwd_gain,
                             args.dead, args.angle, args.point, args.speed, args.settle))
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
