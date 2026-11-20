#!/usr/bin/env python3
"""Fake version, no camera: a pretend Pupper lines up behind a pretend ball and runs through it.

Each step the fake field hands over only two things, both relative to the robot:
where the ball is (metres ahead, metres left) and where the goal is. From those
the approach logic works out the preparatory moves (walk to a point behind the
ball, face the goal, creep forward onto the shot line, then run straight into the
ball without stopping) and a velocity command. The dashboard plays it out.

  python scripts/fake_approach.py                  # watch it
  python scripts/fake_approach.py --headless --seconds 30   # just the numbers
  python scripts/fake_approach.py --rolling        # every new scene starts with the ball rolling
  python scripts/fake_approach.py --compare 40     # 40 rolling-ball scenes: predicting vs chasing

A rolling ball: the ball's path is predicted from its last few positions (pink dashed
line, a dot each second, a ring where it ends) and the robot lines up for where the ball
WILL be. --chase turns that off: the robot then aims at where the ball is right now.

In the window:
  left click   put the ball there          right click  put the robot there
  space        pause / resume              r            new random ball and robot
  k            roll the ball (random)      g            attack the other goal
  z            zoom                        q            quit

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
from fieldcam.ballistics import RollModel, RollTracker  # noqa: E402
from fieldcam.config import load_config  # noqa: E402
from fieldcam.fakeworld import FakeWorld  # noqa: E402
from fieldcam.intercept import RobotModel, plan_robot  # noqa: E402
from fieldcam.net import StatePublisher  # noqa: E402
from fieldcam.relative import body_point  # noqa: E402
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


class Brain:
    """Everything between "what perception reports" and "the velocity command".

    It sees only ball and robot POSITIONS (as the camera would give them), works out where
    the ball is going, and hands the approach logic the ball it should line up for: the
    predicted one while the ball rolls (predict=True), otherwise the ball as it is now.
    """

    def __init__(self, world: FakeWorld, params: Params, predict: bool = True):
        self.world, self.params, self.predict = world, params, predict
        self.ctrl = ApproachController(params)
        # The fake ball's own rules, so the prediction is tested on the same physics it rolls with.
        self.roll_model = RollModel(decel=world.ball_friction, walls=True, restitution=0.4,
                                    meas_sigma=max(world.noise, 0.005))
        self.tracker = RollTracker(self.roll_model)
        self.robot_model = RobotModel(walk_speed=params.walk_speed, strafe_speed=params.strafe_speed,
                                      turn_rate=params.turn_rate, clearance=params.ball_clearance,
                                      standoff=params.run_up, aim_depth=params.aim_depth)

    def reset(self):
        self.ctrl.reset()
        self.tracker.reset()

    def step(self, seen: dict, now: float) -> dict:
        """seen: a perception-shaped state. Adds `prediction` and `intercepts` to it and returns the approach output."""
        w, rel, robot = self.world, seen["relative"][NAME], seen["robots"][NAME]
        ball = (rel["ball"]["x"], rel["ball"]["y"]) if "ball" in rel else None
        goal = (rel["goal"]["x"], rel["goal"]["y"], rel["goal"]["facing"])
        hold = False
        if seen["ball"].get("visible"):
            self.tracker.update((seen["ball"]["x"], seen["ball"]["y"]), seen["stamp"])
            predicted = self.tracker.predict(seen["stamp"], w.length, w.width, w.goal_width, w.ball_radius)
            if predicted is not None:
                seen["prediction"], traj = predicted
                goals = seen["goals"]
                own = next(g for side, g in goals.items() if side != w.attacks)
                plans = plan_robot(robot, traj, goals[w.attacks], own, w.length, w.width, self.robot_model,
                                   area=seen["prediction"]["area"])
                seen["intercepts"] = {NAME: plans}
                strike = plans["strike"]
                if self.predict and traj.moving and strike["feasible"] and self.ctrl.phase != "RUN":
                    target = body_point(robot, strike["ball"])     # line up for where the ball will be
                    ball = (target["x"], target["y"])
                    hold = True                                    # ... and wait there until it has stopped
        else:
            self.tracker.reset()
        out = self.ctrl.tick(ball, goal, now)
        if hold and out["phase"] == "RUN":
            # Lined up for the predicted spot while the ball is still rolling: do not run at an empty spot.
            self.ctrl.reset()
            out = {**out, "phase": "ALIGN", "cmd": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                   "note": "lined up: waiting for the ball to stop, then run"}
        return out


def compare(cfg: dict, params: Params, n: int, args) -> None:
    """n rolling-ball scenes, each run twice from the same start: predicting, and chasing the ball as it is now."""
    f = cfg["field"]
    dt, limit = 1 / 30, 60.0
    rows = {"predict": [], "chase": []}
    for seed in range(n):
        for mode in rows:
            world = FakeWorld(f["length"], f["width"], f["goal_width"], cfg["ball"]["radius"], seed=args.seed + seed,
                              noise=args.noise, sloppy=args.sloppy)
            world.random_scene()
            world.roll_ball()
            brain = Brain(world, params, predict=mode == "predict")
            late, walked, last, hit_t, goal_t, hit_walk = deque(), 0.0, world.robot[:2].copy(), None, None, None
            while world.t < limit and goal_t is None:
                state = world.state(NAME)
                late.append(state)
                seen = late[0] if len(late) <= round(args.delay / dt) else late.popleft()
                out = brain.step(seen, world.t)
                world.step(out["cmd"], dt)
                walked += float(math.hypot(*(world.robot[:2] - last)))
                last = world.robot[:2].copy()
                if hit_t is None and world.kicks:
                    hit_t, hit_walk = world.t, walked
                if world.event.startswith("GOAL"):
                    goal_t = world.t if world.goals_scored else None
                    if goal_t is None:
                        break                              # own goal or the roll itself went in: not a result
            rows[mode].append((hit_t, goal_t, hit_walk))
    print(f"\n{n} rolling-ball scenes ({'sloppy' if args.sloppy else 'perfect'} robot, {args.noise * 100:.0f} cm position noise, "
          f"{args.delay * 1000:.0f} ms delay), ball starts at 0.6-1.5 m/s in a random direction, {limit:.0f} s limit")
    print(f"{'':10s} {'first hit':>22s} {'goal':>22s} {'walked to first hit':>22s}")
    for mode, r in rows.items():
        hits = [h for h, _, _ in r if h is not None]
        goals = [g for _, g, _ in r if g is not None]
        med = lambda v: f"{sorted(v)[len(v) // 2]:.1f} s" if v else "-"  # noqa: E731
        walks = [w for _, _, w in r if w is not None]
        print(f"{mode:10s} {len(hits):3d}/{n} median {med(hits):>8s} {len(goals):3d}/{n} median {med(goals):>8s} "
              f"{'median ' + med(walks)[:-1] + 'm':>22s}")
    both = [(a[0], b[0]) for a, b in zip(rows["predict"], rows["chase"]) if a[0] is not None and b[0] is not None]
    if both:
        saved = sorted(b - a for a, b in both)
        print(f"same scene, both hit ({len(both)}): predicting reaches the ball {saved[len(saved) // 2]:+.1f} s sooner (median), "
              f"{sum(saved) / len(saved):+.1f} s (mean), best {saved[-1]:+.1f} s, worst {saved[0]:+.1f} s; "
              f"sooner in {sum(d > 0.2 for d in saved)}, later in {sum(d < -0.2 for d in saved)}")


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
    ap.add_argument("--rolling", action="store_true", help="every new scene starts with the ball rolling")
    ap.add_argument("--chase", action="store_true",
                    help="do not predict: line up for where the ball is now, not where it will be")
    ap.add_argument("--compare", type=int, default=0, metavar="N",
                    help="no window: run N rolling-ball scenes predicting and chasing, and print how they did")
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
    if args.compare:
        compare(cfg, params, args.compare, args)
        return
    world = FakeWorld(f["length"], f["width"], f["goal_width"], cfg["ball"]["radius"], seed=args.seed, noise=args.noise,
                      sloppy=args.sloppy)
    if args.random or args.rolling:
        world.random_scene()
    if args.rolling:
        world.roll_ball()
    brain = Brain(world, params, predict=not args.chase)
    ctrl = brain.ctrl
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
        out = brain.step(seen, world.t)                    # <- positions in, a velocity command out
        state["approach"] = {NAME: out}
        state["prediction"], state["intercepts"] = seen.get("prediction"), seen.get("intercepts", {})
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
                if args.rolling:
                    world.roll_ball()
                brain.reset()
            if key == ord("k"):
                world.roll_ball()
            if key == ord("g"):
                world.attacks = "west" if world.attacks == "east" else "east"
                brain.reset()
            while click:
                event, x, y = click.pop(0)
                to_world = view_mapping(state, (size[0] - PANEL_W, size[1]), zoom)[1]
                wx, wy = to_world(x, y)
                if event == cv2.EVENT_LBUTTONDOWN:
                    world.place_ball(wx, wy)
                else:
                    world.place_robot(wx, wy)
                brain.reset()
        if args.seconds and world.t >= args.seconds:
            break

    print(f"\n{world.t:.1f} s simulated: {world.kicks} bumper hits, {world.goals_scored} goals in the {world.attacks} goal")
    if args.snapshot and img is not None:
        cv2.imwrite(args.snapshot, img)
        print("wrote", args.snapshot)


if __name__ == "__main__":
    main()
