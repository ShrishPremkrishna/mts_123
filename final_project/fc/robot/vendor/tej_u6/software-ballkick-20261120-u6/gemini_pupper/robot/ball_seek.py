#!/usr/bin/env python3
"""Find the ball with the camera, put it in front of one front paw, and kick it.

Run on the Pupper while the Gemini Pupper API runs (port 18090):

    python3 ball_seek.py right          # kick with the right front leg
    python3 ball_seek.py left           # left front leg
    python3 ball_seek.py auto           # the leg on the side where the ball is first seen
    python3 ball_seek.py right --dry-run    # line up, then stop: no creep, no kick

Each step: look with the camera (the locate_ball tool, about 1.2 s, no motion), then
one short motion through the move or turn tool, then look again.

  * No ball (or farther than --max-range): turn left in 40 deg steps. After a full
    turn with no ball, step away (the ball can be in the blind zone near the body)
    and search again: back 0.45 m, then left 0.6 m, then right 1.2 m. The turn is
    timed, not measured, so the heading after a full turn is not known. Three
    directions make sure that one step goes away from the ball.
  * Ball seen: turn until it is ahead on the paw's line, then step forward, backward
    and sideways until its centre is 0.55 m ahead and on the paw's line.
  * Lined up: walk the last 0.29 m blind (the camera cannot see the ball 0.30 to
    0.45 m away), timed with the walker model measured on this robot, then play the
    leg kick (the kick_in_place tool) and return to the walker.

Ctrl-C sends Stop. Any refused or failed tool call sends Stop and ends the program.
Standard library only.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field

PAW_Y = {"right": -0.046, "left": 0.040}  # ball centre y at the kick (leg kick handoff, 2026-11-16)


@dataclass
class SeekParams:
    stage_x: float = 0.55  # m, ball centre x before the blind creep (the camera sees >= ~0.47 m)
    tol_x: float = 0.04  # m
    tol_y: float = 0.010  # m (each 1 cm sideways turns the kicked ball by roughly 7 to 13 deg)
    contact_x: float = 0.258  # m, ball centre x at the kick
    creep_margin: float = 0.008  # m, stop this much short (a ball under the body cannot be kicked)
    jump_m: float = 0.25  # a look this far from where the last step should have put the ball is
                          # not trusted until a second look agrees with it (a wrong object)
    min_x: float = 0.47  # m, nearer fixes are not trusted (blind zone edge)
    max_range: float = 1.4  # m, farther fixes are ignored (place the ball near the robot)
    heading_tol_deg: float = 12.0  # bearing to the paw line above this: turn
    min_turn_deg: float = 15.0  # the turn tool's smallest useful angle
    speed: float = 0.2  # m/s, forward, backward and sideways steps (the move tool's minimum)
    fwd_gain: float = 1.31  # achieved / commanded forward speed, measured 2026-11-18 (dry run 1)
    side_gain: float = 1.0  # prior; learned from each sideways step
    turn_gain: float = 1.0  # prior; learned from each turn
    dead_s: float = 0.14  # s, walker start delay, measured 2026-11-18
    gain_bounds: tuple[float, float] = (0.5, 2.0)
    search_step_deg: float = 40.0
    search_turn_deg: float = 360.0  # commanded turn before a back-up
    backup_m: float = 0.45
    side_step_m: float = 0.6
    max_backups: int = 3  # after each full turn with no ball: backward, left, right (blind zone)
    max_steps: int = 60
    max_time_s: float = 240.0
    rehearse: bool = True  # before the creep: back up and step forward the creep's own duration,
                           # measured by the camera, so that the creep repeats a measured step
    auto_deadband: float = 0.02  # m: auto side, a ball within this of the middle gets the right leg
    retries: int = 3  # looks per step while the ball was seen recently (one frame can miss it)
    near_side_y: float = 0.25  # m: "too near" backs up only for a ball this close to the middle


@dataclass
class Obs:
    x: float  # m, ball centre forward of base_link
    y: float  # m, left
    method: str = "fit"

    @property
    def range(self) -> float:
        return math.hypot(self.x, self.y)


class Abort(Exception):
    pass


class Seeker:
    """The decisions. ``client`` gives look(), turn(deg), walk(vx, vy, seconds), kick(side),
    stop() and now(); see HttpClient. Motions block until the robot API reports them done."""

    def __init__(self, client, side: str, params: SeekParams = SeekParams(), dry_run: bool = False, log=print):
        if side not in ("right", "left", "auto"):
            raise ValueError("side must be right, left or auto")
        self.c, self.p, self.dry_run, self.log = client, params, dry_run, log
        self.requested = side
        self.side: str | None = None if side == "auto" else side
        self.gains = {"fwd": params.fwd_gain, "side": params.side_gain, "turn": params.turn_gain}
        self.dead = {"fwd": params.dead_s, "side": params.dead_s}  # learned per axis
        self.scale = {"fwd": 1.0, "side": 1.0}  # correction damping per axis (shrinks when steps reverse)
        self.last_sign = {"fwd": 0.0, "side": 0.0}
        self.reversals = {"fwd": 0, "side": 0}  # after 4, the tolerance on that axis doubles
        self.rehearsed = not params.rehearse
        self.frozen_fwd = False
        self.steps = 0
        self.expect: Obs | None = None  # where the ball should be after the last step
        self.recent = False  # the ball was seen on the last step
        self.suspect: Obs | None = None  # a look that disagreed with expect

    # ------------------------------------------------------------- helpers
    def note(self, text: str) -> None:
        self.log(f"[{self.steps:2d}] {text}")

    def seconds(self, distance: float, axis: str) -> float:
        """Step time for a distance: dead time + distance / (gain * speed)."""
        return self.dead[axis] + abs(distance) / (self.gains[axis] * self.p.speed)

    def learn(self, key: str, commanded: float, moved: float) -> None:
        """Blend a measured gain (moved / commanded effective travel) into the model. A step that
        moved much less than expected raises that axis's dead time instead (small steps did nothing)."""
        if key in self.dead and commanded >= 0.005 and moved < 0.4 * commanded * self.gains[key]:
            self.dead[key] = min(0.45, self.dead[key] + 0.05)
            self.note(f"{key} step moved {moved:.3f} m of {commanded * self.gains[key]:.3f}: dead time now {self.dead[key]:.2f} s")
            return
        if key in self.dead and moved > 0.01 and commanded < 0.005:
            # A step shorter than the modelled dead time moved anyway: the dead time is shorter.
            self.dead[key] = max(0.0, self.dead[key] - 0.05)
            self.note(f"{key} step moved {moved:.3f} m inside the dead time: dead time now {self.dead[key]:.2f} s")
            return
        if key in self.dead and 0.005 <= commanded < 0.05 and moved > 1.6 * commanded * self.gains[key]:
            self.dead[key] = max(0.0, self.dead[key] - 0.04)
            self.note(f"{key} step moved {moved:.3f} m of {commanded * self.gains[key]:.3f}: dead time now {self.dead[key]:.2f} s")
            return
        if key == "fwd" and self.frozen_fwd:
            return  # set by the practice step, which measured the creep's own duration
        if commanded < {"fwd": 0.05, "side": 0.03}.get(key, 2.0) or moved <= 0.0:
            return  # short steps are mostly dead time and noise
        lo, hi = self.p.gain_bounds
        measured = min(hi, max(lo, moved / commanded))
        self.gains[key] = 0.5 * self.gains[key] + 0.5 * measured
        self.note(f"{key} gain {measured:.2f} measured, model now {self.gains[key]:.2f}")

    def look(self) -> Obs | None:
        """One camera look, checked against where the last step should have put the ball."""
        obs = self.c.look()
        if obs is None or obs.range > self.p.max_range:
            return None
        if self.expect is not None and math.hypot(obs.x - self.expect.x, obs.y - self.expect.y) > self.p.jump_m:
            if self.suspect is None or math.hypot(obs.x - self.suspect.x, obs.y - self.suspect.y) > self.p.jump_m:
                self.note(f"look at x {obs.x:.2f} y {obs.y:+.2f} is far from the expected "
                          f"x {self.expect.x:.2f} y {self.expect.y:+.2f}: look again")
                self.suspect = obs
                obs = self.c.look()
                if obs is None or obs.range > self.p.max_range:
                    return None
                if math.hypot(obs.x - self.expect.x, obs.y - self.expect.y) <= self.p.jump_m:
                    self.suspect = None
                    return obs
                if math.hypot(obs.x - self.suspect.x, obs.y - self.suspect.y) > self.p.jump_m:
                    return None  # two looks that agree with nothing: no ball this step
            self.note("two looks agree on a new position: the ball moved, or the model was wrong")
        self.suspect = None
        return obs

    def damped(self, axis: str, error: float) -> float:
        """The step for an error on one axis: smaller after each reversal (no endless left-right)."""
        sign = math.copysign(1.0, error)
        if self.last_sign[axis] and sign != self.last_sign[axis]:
            self.scale[axis] = max(0.25, self.scale[axis] * 0.6)
            self.reversals[axis] += 1
        elif self.last_sign[axis]:
            self.scale[axis] = min(1.0, self.scale[axis] * 1.2)
        self.last_sign[axis] = sign
        return error * self.scale[axis]

    def predict(self, obs: Obs, dx: float = 0.0, dy: float = 0.0, dtheta_deg: float = 0.0) -> None:
        """Where the ball should be after a step that moves the robot (dx, dy) and turns it."""
        x, y = obs.x - dx, obs.y - dy
        a = -math.radians(dtheta_deg)
        self.expect = Obs(math.cos(a) * x - math.sin(a) * y, math.sin(a) * x + math.cos(a) * y)

    # ------------------------------------------------------------- main loop
    def run(self) -> str:
        p = self.p
        start = self.c.now()
        searched, backups, ever_seen = 0.0, 0, False
        prev: tuple[str, float, Obs] | None = None  # (axis, commanded effective travel or angle, obs before)
        while True:
            self.steps += 1
            if self.steps > p.max_steps or self.c.now() - start > p.max_time_s:
                raise Abort(f"gave up after {self.steps - 1} steps / {self.c.now() - start:.0f} s")
            obs = self.look()
            tries = 1
            while obs is None and self.recent and tries < p.retries:  # a missed frame, not a lost ball
                tries += 1
                obs = self.look()
            self.recent = obs is not None
            if obs is not None and prev is not None:
                axis, commanded, before = prev
                if axis == "fwd" and commanded > 0:  # backward steps can differ: forward gain only
                    self.learn("fwd", commanded, before.x - obs.x)
                elif axis == "side":
                    self.learn("side", max(0.0, abs(commanded)), math.copysign(1, commanded) * (before.y - obs.y))
                elif axis == "turn":
                    turned = math.degrees(math.atan2(before.y, before.x) - math.atan2(obs.y, obs.x))
                    self.learn("turn", abs(commanded), math.copysign(1, commanded) * turned)
            prev = None
            if obs is None:
                if searched >= p.search_turn_deg:
                    if backups >= p.max_backups:
                        raise Abort(f"ball not found (stepped away {backups} times and searched by turning)")
                    backups += 1
                    searched = 0.0
                    # With a correct turn, these end at (-0.45, 0), (-0.45, +0.6), (-0.45, -0.6) from the
                    # start. A side step of 0.6 m clears a ball that the back-up pushed behind the body.
                    vx, vy, axis, name, d = ((-p.speed, 0.0, "fwd", "back up", p.backup_m),
                                             (0.0, p.speed, "side", "step left", p.side_step_m),
                                             (0.0, -p.speed, "side", "step right", 2 * p.side_step_m))[(backups - 1) % 3]
                    self.note(f"no ball after a full turn: {name} {d:.2f} m")
                    self.expect = None
                    self.c.walk(vx, vy, self.seconds(d, axis))
                    continue
                self.note(f"no ball: turn left {p.search_step_deg:.0f} deg")
                self.expect = None
                self.c.turn(p.search_step_deg)
                searched += p.search_step_deg
                continue
            searched, ever_seen = 0.0, True
            if self.side is None:
                self.side = "left" if obs.y > p.auto_deadband else "right"
                self.note(f"auto side: {self.side} (ball first seen at y {obs.y:+.3f} m)")
            ty = PAW_Y[self.side]
            ex, ey = obs.x - p.stage_x, obs.y - ty
            bearing = math.degrees(math.atan2(obs.y - ty, obs.x))
            self.note(f"ball x {obs.x:.3f} y {obs.y:+.3f} ({obs.method}); off by x {ex:+.3f} y {ey:+.3f}, "
                      f"bearing {bearing:+.0f} deg")
            if obs.x < p.min_x and abs(obs.y) < p.near_side_y:
                back = min(0.5, p.stage_x - obs.x)
                self.note(f"ball too near to measure: back up {back:.2f} m")
                t = self.seconds(back, "fwd")
                self.c.walk(-p.speed, 0.0, t)
                prev = ("fwd", -(t - self.dead["fwd"]) * p.speed, obs)
                self.predict(obs, dx=-back)
                continue
            if abs(bearing) > p.heading_tol_deg and abs(bearing) / self.gains["turn"] >= p.min_turn_deg:
                angle = math.copysign(abs(bearing) / self.gains["turn"], bearing)
                self.note(f"turn {'left' if angle > 0 else 'right'} {abs(angle):.0f} deg")
                self.c.turn(angle)
                prev = ("turn", angle, obs)
                self.predict(obs, dtheta_deg=bearing)
                continue
            tol_x = p.tol_x * (2.0 if self.reversals["fwd"] >= 4 else 1.0)
            tol_y = p.tol_y * (2.0 if self.reversals["side"] >= 4 else 1.0)
            if abs(ex) > tol_x:
                step = self.damped("fwd", ex)
                t = self.seconds(step, "fwd")
                self.note(f"step {'forward' if ex > 0 else 'back'} {abs(step):.3f} m of {abs(ex):.3f} ({t:.2f} s)")
                self.c.walk(math.copysign(p.speed, ex), 0.0, t)
                prev = ("fwd", math.copysign((t - self.dead["fwd"]) * p.speed, ex), obs)
                self.predict(obs, dx=step)
                continue
            if abs(ey) > tol_y:
                step = self.damped("side", ey)
                t = self.seconds(step, "side")
                self.note(f"step {'left' if ey > 0 else 'right'} {abs(step):.3f} m of {abs(ey):.3f} ({t:.2f} s)")
                self.c.walk(0.0, math.copysign(p.speed, ey), t)
                prev = ("side", math.copysign((t - self.dead["side"]) * p.speed, ey), obs)
                self.predict(obs, dy=step)
                continue
            # In tolerance: a second look must agree; the creep starts from their average.
            again = self.look()
            if again is None or abs(again.x - p.stage_x) > tol_x or abs(again.y - ty) > tol_y:
                self.note("second look does not confirm the line-up: measure again")
                self.expect = obs if again is None else Obs(again.x, again.y)
                continue
            third = self.look()  # the creep is blind: average three looks for its start
            looks = [o for o in (obs, again, third) if o is not None]
            obs = Obs(sum(o.x for o in looks) / len(looks), sum(o.y for o in looks) / len(looks), obs.method)
            creep = obs.x - p.contact_x - p.creep_margin
            if not self.rehearsed and not self.dry_run:
                self.rehearse(obs, creep)
                continue
            if self.dry_run:
                self.note(f"lined up for the {self.side} leg; dry run: no creep {creep:.3f} m, no kick")
                return "lined up (dry run)"
            t = self.seconds(creep, "fwd")
            self.note(f"lined up: creep {creep:.3f} m blind ({t:.2f} s), then kick with the {self.side} leg")
            self.c.walk(p.speed, 0.0, t)
            outcome = self.c.kick(self.side)
            self.note(f"kick: {outcome}")
            return f"kicked with the {self.side} leg"


def _mean_look(seeker: "Seeker", tries: int = 4) -> Obs | None:
    """The mean of two valid looks out of up to ``tries`` (one valid look if only one comes)."""
    got = []
    for _ in range(tries):
        obs = seeker.look()
        if obs is not None:
            got.append(obs)
            if len(got) == 2:
                break
    if not got:
        return None
    return Obs(sum(o.x for o in got) / len(got), sum(o.y for o in got) / len(got), got[0].method)


def _rehearse(self: "Seeker", obs: Obs, creep: float) -> None:
    """Back up the creep's duration, then step forward the same duration while the camera
    still sees the ball. The measured travel sets the forward model the creep uses, so the
    creep repeats a measured step (gain and dead-time errors cancel)."""
    p = self.p
    self.rehearsed = True
    t = self.seconds(creep, "fwd")
    self.note(f"practice step: back up {t:.2f} s, then the creep's {t:.2f} s forward, measured")
    planned = (t - self.dead["fwd"]) * p.speed * self.gains["fwd"]
    self.c.walk(-p.speed, 0.0, t)
    self.predict(obs, dx=-planned)
    start = _mean_look(self)
    if start is None:
        self.note("practice step: ball not seen after backing up; the creep uses the model")
        return
    self.c.walk(p.speed, 0.0, t)
    self.predict(start, dx=planned)
    end = _mean_look(self)
    if end is None:
        self.note("practice step: ball not seen after the step; the creep uses the model")
        return
    moved = start.x - end.x
    if not 0.7 * planned <= moved <= 1.4 * planned:
        self.note(f"practice step moved {moved:.3f} m of {planned:.3f}: not used")
        return
    # Keep the dead time, set the gain so that duration t gives the measured travel.
    self.gains["fwd"] = moved / ((t - self.dead["fwd"]) * p.speed)
    self.frozen_fwd = True
    self.note(f"practice step moved {moved:.3f} m in {t:.2f} s: forward gain now {self.gains['fwd']:.2f}")
    self.expect = end


Seeker.rehearse = _rehearse


class HttpClient:
    """The Gemini Pupper API (localhost). Every motion waits until the queue is idle again."""

    def __init__(self, url: str = "http://127.0.0.1:18090", poll_s: float = 0.2, job_timeout_s: float = 60.0):
        self.url, self.poll_s, self.job_timeout_s = url.rstrip("/"), poll_s, job_timeout_s

    def now(self) -> float:
        return time.monotonic()

    def _request(self, method: str, path: str, body: dict | None = None, epoch: str | None = None) -> dict:
        data = None if body is None else json.dumps(body).encode()
        headers = {"Content-Type": "application/json"}
        if epoch:
            headers["X-Pupper-Command-Epoch"] = epoch
        req = urllib.request.Request(self.url + path, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                return json.load(r)
        except urllib.error.HTTPError as e:
            try:
                return json.loads(e.read() or b"{}")
            except ValueError:
                return {"success": False, "message": f"HTTP {e.code}"}

    def call(self, tool: str, args: dict | None = None) -> dict:
        epoch = self._request("GET", "/health").get("command_epoch")
        return self._request("POST", "/tools/" + tool, args or {}, epoch)

    def status(self) -> dict:
        return self._request("POST", "/tools/robot_status", {}).get("details") or {}

    def _run(self, tool: str, args: dict) -> dict:
        res = self.call(tool, args)
        if not res.get("success"):
            raise Abort(f"{tool} refused: {res.get('message')}")
        job = ((res.get("details") or {}).get("submission") or {}).get("job_id")
        deadline = self.now() + self.job_timeout_s
        while self.now() < deadline:
            time.sleep(self.poll_s)
            st = self.status()
            if st.get("current_action") is None and not st.get("queued"):
                last = st.get("last_outcome") or {}
                if job and last.get("job_id") != job:
                    continue  # this job's outcome is not recorded yet
                if not last.get("success"):
                    raise Abort(f"{tool} failed: {last.get('message')}")
                return last
        raise Abort(f"{tool} did not finish in {self.job_timeout_s:.0f} s")

    def look(self) -> Obs | None:
        res = self.call("locate_ball")
        ball = (res.get("details") or {}).get("ball") or {}
        if not res.get("success") or not ball.get("seen"):
            return None
        return Obs(float(ball["x"]), float(ball["y"]), str(ball.get("method", "fit")))

    def turn(self, degrees: float) -> None:
        self._run("turn", {"direction": "left" if degrees > 0 else "right", "angle": round(abs(degrees), 1)})

    def walk(self, vx: float, vy: float, seconds: float) -> None:
        if vx:
            direction, speed = ("forward" if vx > 0 else "backward"), abs(vx)
        else:
            direction, speed = ("left" if vy > 0 else "right"), abs(vy)
        self._run("move", {"direction": direction, "speed": speed, "duration": round(max(0.05, seconds), 2)})

    def kick(self, side: str) -> str:
        return self._run("kick_in_place", {"side": side}).get("message", "")

    def kick_at(self, x: float, y: float, side: str, dry_run: bool = False,
                gain: float | None = None, delay: float | None = None, **extra) -> str:
        """Turn, walk and kick as one job of the API (no wait between the walk and the kick).
        extra: the tool's optional angle, point, speed and settle."""
        args = {"x": x, "y": y, "side": side, "dry_run": dry_run, **extra}
        args.update({k: v for k, v in (("gain", gain), ("delay", delay)) if v is not None})
        return self._run("kick_at", args).get("message", "")

    def stop(self) -> None:
        try:
            self._request("POST", "/tools/stop", {})
        except Exception:
            pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("side", choices=["right", "left", "auto"])
    ap.add_argument("--dry-run", action="store_true", help="line up, then stop (no creep, no kick)")
    ap.add_argument("--url", default="http://127.0.0.1:18090")
    ap.add_argument("--fwd-gain", type=float, default=SeekParams.fwd_gain)
    ap.add_argument("--dead", type=float, default=SeekParams.dead_s)
    ap.add_argument("--stage", type=float, default=SeekParams.stage_x)
    args = ap.parse_args(argv)
    params = SeekParams(fwd_gain=args.fwd_gain, dead_s=args.dead, stage_x=args.stage)
    client = HttpClient(args.url)
    seeker = Seeker(client, args.side, params, args.dry_run,
                    log=lambda text: print(time.strftime("%H:%M:%S"), text, flush=True))
    try:
        result = seeker.run()
        print("DONE:", result)
        return 0
    except KeyboardInterrupt:
        client.stop()
        print("STOPPED (Ctrl-C): Stop sent")
        return 130
    except Abort as error:
        client.stop()
        print("ABORT:", error, "(Stop sent)")
        return 1
    except Exception as error:  # an unexpected fault must still stop the robot
        client.stop()
        print("ERROR:", type(error).__name__, error, "(Stop sent)")
        return 2


if __name__ == "__main__":
    sys.exit(main())
