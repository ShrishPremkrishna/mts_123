#!/usr/bin/env python3
"""Fake version, no camera: a pretend Pupper lines up behind a pretend ball and runs through it.

Each step the fake field hands over only two things, both relative to the robot:
where the ball is (metres ahead, metres left) and where the goal is. From those
the approach logic works out the preparatory moves (walk to a point behind the
ball, face the goal, creep forward onto the shot line, then run straight into the
ball without stopping) and a velocity command. The dashboard plays it out.

  python scripts/fake_approach.py                  # watch it
  python scripts/fake_approach.py --headless --seconds 30   # just the numbers

In the window:
  left click   put the ball there          right click  put the robot there
  space        pause / resume              r            new random ball and robot
  g            attack the other goal       z            zoom       q  quit

The same state goes out over UDP, so scripts/dashboard.py and scripts/listen.py
see it exactly as they would see the real camera pipeline.
This is a cartoon for trying the logic. It does not predict a real Pupper or ball.
"""
import argparse
import math
import sys
import time
from collections import deque
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.approach import ApproachController, Params  # noqa: E402
from fieldcam.config import load_config  # noqa: E402
from fieldcam.fakeworld import FakeWorld  # noqa: E402
from fieldcam.net import StatePublisher  # noqa: E402
from fieldcam.topdown import PANEL_W, render, view_mapping  # noqa: E402

WINDOW = "pupper fc fake approach"
NAME = "striker"


def describe(rel: dict, out: dict) -> str:
    """One terminal line: the two relative points going in, and what comes out."""
    def point(p):
        return f"{p['x']:+.2f} m ahead, {p['y']:+.2f} m left"
    ball = point(rel["ball"]) if "ball" in rel else "not seen"
    facing = f", faces {math.degrees(rel['goal']['facing']):+.0f} deg"
    c = out["cmd"]
    return (f"ball: {ball} | goal: {point(rel['goal'])}{facing} | {out['phase']:8s} "
            f"cmd vx {c['vx']:+.2f} vy {c['vy']:+.2f} wz {c['wz']:+.2f} | {out['note']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--headless", action="store_true", help="no window: print and exit after --seconds")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this much simulated time (0 = until q)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--random", action="store_true", help="start from a random ball and robot position")
    ap.add_argument("--run-up", type=float, default=None, help="line-up distance behind the ball, m (default 0.60)")
    ap.add_argument("--noise", type=float, default=0.0, help="position noise added to what the planner sees, m")
    ap.add_argument("--sloppy", action="store_true",
                    help="the fake robot follows commands badly: 80%% speed, veers left, creeps while strafing")
    ap.add_argument("--delay", type=float, default=0.0, help="the planner sees positions this many seconds late")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (2 = twice as fast)")
    ap.add_argument("--snapshot", default=None, help="write the last picture to this image file")
    args = ap.parse_args()

    cfg = load_config()
    f = cfg["field"]
    params = Params(contact=0.17 + cfg["ball"]["radius"])
    for key, value in (cfg.get("approach_plan") or {}).items():   # optional overrides in config/field.yaml
        if not hasattr(params, key):
            raise SystemExit(f"config approach_plan: unknown setting {key!r}")
        setattr(params, key, math.radians(value) if key in ("yaw_tol", "turn_first") else float(value))
    if args.run_up:
        params.run_up = args.run_up
    world = FakeWorld(f["length"], f["width"], f["goal_width"], cfg["ball"]["radius"], seed=args.seed, noise=args.noise,
                      sloppy=args.sloppy)
    if args.random:
        world.random_scene()
    ctrl = ApproachController(params)
    pub = StatePublisher(cfg["net"]["host"], cfg["net"]["port"])
    print(f"fake field {f['length']:.2f} x {f['width']:.2f} m, ball radius {cfg['ball']['radius'] * 100:.0f} cm, "
          f"line-up {params.run_up:.2f} m behind the ball, run speed {params.run_speed:.2f} m/s, "
          f"{'SLOPPY' if args.sloppy else 'perfect'} robot (never reverses), planner sees positions {args.delay * 1000:.0f} ms late")

    dt, zoom, paused, state, click = 1 / 30, "field", False, None, []
    late = deque()  # what the planner is allowed to see, args.delay seconds behind the truth
    size = (1280, 800)

    def on_mouse(event, x, y, _flags, _param):
        if event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN) and state is not None and x < size[0] - PANEL_W:
            click.append((event, x, y))

    last_phase, last_steps, t_print, img, mouse_ready = None, None, -1.0, None, False
    while True:
        t_wall = time.time()
        state = world.state(NAME)
        late.append(state)
        seen = late[0] if len(late) <= round(args.delay / dt) else late.popleft()
        rel = seen["relative"][NAME]
        ball = (rel["ball"]["x"], rel["ball"]["y"]) if "ball" in rel else None
        goal = (rel["goal"]["x"], rel["goal"]["y"], rel["goal"]["facing"])
        out = ctrl.tick(ball, goal, world.t)               # <- the only inputs: two robot-relative points
        state["approach"] = {NAME: out}
        pub.send(state)

        steps = (out["plan"] or {}).get("steps")
        if out["phase"] != last_phase or world.event:
            if world.event:
                print(f"  [{world.t:6.1f} s] {world.event}")
            if out["phase"] != last_phase:
                print(f"  [{world.t:6.1f} s] phase -> {out['phase']}: {out['note']}")
                if out["phase"] in ("APPROACH", "ALIGN") and last_phase in (None, "BRAKE", "RUN") and steps:
                    print("            preparatory moves from here:")
                    for k, step in enumerate(steps, 1):
                        print(f"              {k}. {step}")
            last_phase = out["phase"]
        if world.t - t_print >= 1.0:
            t_print = world.t
            print(f"[{world.t:6.1f} s] {describe(rel, out)}", flush=True)

        if not paused:
            world.step(out["cmd"], dt)

        if not args.headless or args.snapshot:
            img = render(state, size, zoom)
        if not args.headless:
            cv2.imshow(WINDOW, img)
            if not mouse_ready:
                cv2.setMouseCallback(WINDOW, on_mouse)
                mouse_ready = True
            key = cv2.waitKey(max(1, int(1000 * (dt / args.speed - (time.time() - t_wall))))) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                paused = not paused
            if key == ord("z"):
                zoom = "fit" if zoom == "field" else "field"
            if key == ord("r"):
                world.random_scene()
                ctrl.reset()
            if key == ord("g"):
                world.attacks = "west" if world.attacks == "east" else "east"
                ctrl.reset()
            while click:
                event, x, y = click.pop(0)
                to_world = view_mapping(state, (size[0] - PANEL_W, size[1]), zoom)[1]
                wx, wy = to_world(x, y)
                if event == cv2.EVENT_LBUTTONDOWN:
                    world.place_ball(wx, wy)
                else:
                    world.place_robot(wx, wy)
                ctrl.reset()
        if args.seconds and world.t >= args.seconds:
            break

    print(f"\n{world.t:.1f} s simulated: {world.kicks} bumper hits, {world.goals_scored} goals in the {world.attacks} goal")
    if args.snapshot and img is not None:
        cv2.imwrite(args.snapshot, img)
        print("wrote", args.snapshot)


if __name__ == "__main__":
    main()
