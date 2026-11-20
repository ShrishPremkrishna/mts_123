"""Get behind the ball, converge onto the shot line while moving forward, then run through the ball.

Everything here works from two robot-relative points and nothing else:

    ball = (x ahead, y left)            metres, in the robot's body frame
    goal = (x ahead, y left[, facing])  metres, centre of the goal being attacked; `facing` is
                                        the direction INTO the goal as a turn from the robot's
                                        heading (radians, + = left)

so it can run on whatever supplies those two points (the overhead camera's
`relative` output, or the fake world in fakeworld.py).

Built for a robot that only takes speed commands, follows them imprecisely,
walks sideways slowly and cannot walk backwards:

  - It is closed-loop. Every tick starts from where the camera says things are
    now, so a robot that is slow or veers gets corrected instead of drifting.
  - It never commands backward motion. If it ends up too close, it walks round
    and comes in again.
  - It does not stop on an exact spot. It only has to be ON the shot line and
    FACING along it, anywhere in a window behind the ball; being 15 cm nearer
    or farther does not matter, being 4 cm to the side does.
  - It keeps correcting sideways and in heading during the run, up to contact,
    and it builds up a small sideways push against a robot that sits steadily
    to one side of the line (a modest help in tests; learning the turn bias the
    same way made things worse, so that is not done).

The phases:
  1. APPROACH  walk to the entry point, a little further back than the line-up
               point, going round the ball if it is in the way; turn to face the goal.
  2. ALIGN     creep forward along the shot line, shuffling sideways onto it.
  3. RUN       the moment it is on the line and facing along it: run speed,
               still correcting, through the ball. No pause in between.
  4. BRAKE     stop, then start again from wherever the ball ended up.

`plan()` is the geometry for one instant plus the moves in plain words.
`ApproachController` steps through the phases and gives a body-frame velocity
command each tick. All numbers in Params are starting guesses, not measured.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .filters import wrap


@dataclass
class Params:
    run_up: float = 0.60          # m, nominal line-up distance behind the ball's centre
    entry_extra: float = 0.35     # m, walk to a point this much further back, then come forward onto the line
    run_min: float = 0.45         # m, the run may start anywhere between run_min and run_max behind the ball
    run_max: float = 1.20
    aim_depth: float = 0.40       # m, aim at a point this far BEHIND the goal line (needs goal facing)
    ball_clearance: float = 0.35  # m, keep the body centre this far from the ball while walking round it
    lat_tol: float = 0.04         # m, sideways distance from the shot line that counts as "on it"
    yaw_tol: float = math.radians(5)   # rad, facing along the shot line closely enough
    corridor: float = 0.15        # m, abort the run if the robot gets this far off the line before contact
    turn_first: float = math.radians(40)  # rad, heading error above this: turn on the spot before walking
    walk_speed: float = 0.35      # m/s
    creep_speed: float = 0.15     # m/s, forward speed while converging onto the line
    strafe_speed: float = 0.20    # m/s, sideways (the Pupper strafes slowly)
    run_strafe: float = 0.15      # m/s, sideways correction allowed during the run
    turn_rate: float = 1.2        # rad/s
    run_speed: float = 0.45       # m/s, the stock-gait charge speed used by the shooting baseline
    contact: float = 0.25         # m, robot centre to ball centre when the bumper touches
    follow_through: float = 0.25  # m, keep running this far past the contact point
    brake_s: float = 0.6          # s, stand still after the run
    kp_pos: float = 1.5           # 1/s
    kp_yaw: float = 2.5           # 1/s
    kp_lat: float = 2.0           # 1/s, sideways correction onto the line
    ki_lat: float = 0.5           # 1/s^2, builds up against a steady sideways drift (0 = off)
    line_lean: float = 1.5        # 1/m, how hard to lean the heading back toward the line


@dataclass
class Plan:
    """One instant's geometry, all in the robot's body frame (x ahead, y left)."""
    ball: tuple[float, float]
    goal: tuple[float, float]
    line_up: tuple[float, float]          # nominal start of the run, run_up behind the ball
    entry: tuple[float, float]            # where to get to first, further back on the same line
    shot_heading: float                   # turn (rad, + = left) that faces along the shot line
    behind: float                         # robot is this far behind the ball along the line (- = goal side)
    left_of_line: float                   # robot is this far left of the line, looking at the goal
    path: list[tuple[float, float]]       # robot (0,0) -> [detour ->] entry point
    run_end: tuple[float, float]          # where the run finishes, past the ball
    detour: bool
    steps: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {"ball": list(self.ball), "goal": list(self.goal), "line_up": list(self.line_up),
                "entry": list(self.entry), "shot_heading": self.shot_heading, "behind": self.behind,
                "left_of_line": self.left_of_line, "path": [list(q) for q in self.path] + [list(self.line_up)],
                "run_end": list(self.run_end), "detour": self.detour, "steps": list(self.steps)}


def _side(angle: float) -> str:
    return f"{abs(math.degrees(angle)):.0f} deg {'left' if angle >= 0 else 'right'}"


def plan(ball, goal, p: Params = Params()) -> Plan | None:
    """The plan for a ball and goal given relative to the robot. None if ball and aim point coincide."""
    bx, by = float(ball[0]), float(ball[1])
    gx, gy = float(goal[0]), float(goal[1])
    # Aim through the goal centre at a point behind the goal line. Aiming AT the centre makes the
    # shot direction swing wildly once the ball is close to it; a point behind stays steady
    # right up to the line. Without the goal's facing, fall back to the centre itself.
    ax, ay = gx, gy
    if len(goal) > 2 and goal[2] is not None:
        ax, ay = gx + p.aim_depth * math.cos(goal[2]), gy + p.aim_depth * math.sin(goal[2])
    d = math.hypot(ax - bx, ay - by)
    if d < 1e-6:
        return None
    ux, uy = (ax - bx) / d, (ay - by) / d                 # unit vector along the shot line
    shot_heading = math.atan2(uy, ux)
    line_up = (bx - p.run_up * ux, by - p.run_up * uy)
    back = p.run_up + p.entry_extra
    entry = (bx - back * ux, by - back * uy)
    run_end = (bx + (p.follow_through - p.contact) * ux, by + (p.follow_through - p.contact) * uy)
    behind = bx * ux + by * uy                            # robot (at the origin) relative to the ball
    left_of_line = bx * uy - by * ux

    # Straight to the entry point, unless that walks through the ball: then go round it,
    # on the side the robot is already on.
    path, detour = [(0.0, 0.0), entry], False
    seg = math.hypot(*entry)
    if seg > 1e-6:
        t = max(0.0, min(1.0, (bx * entry[0] + by * entry[1]) / seg ** 2))
        if math.hypot(bx - t * entry[0], by - t * entry[1]) < p.ball_clearance and 0.0 < t < 1.0:
            nx, ny = -entry[1] / seg, entry[0] / seg      # left normal of the straight path
            side = 1.0 if (-bx * nx - by * ny) >= 0 else -1.0
            way = (bx + side * nx * p.ball_clearance * 1.3, by + side * ny * p.ball_clearance * 1.3)
            path, detour = [(0.0, 0.0), way, entry], True

    # The same thing as a list of moves in plain words.
    steps, heading = [], 0.0
    on_line = behind > p.run_min - 0.03 and abs(left_of_line) < 0.35 and abs(shot_heading) < math.radians(50)
    if not on_line:
        for a, b in zip(path, path[1:]):
            leg = math.hypot(b[0] - a[0], b[1] - a[1])
            if leg < 0.05:
                continue
            direction = math.atan2(b[1] - a[1], b[0] - a[0])
            turn = wrap(direction - heading)
            if abs(turn) > math.radians(3):
                steps.append(f"turn {_side(turn)}")
            steps.append(f"walk {leg:.2f} m " + ("to go round the ball" if (detour and b is not entry) else "to get behind the ball"))
            heading = direction
        final_turn = wrap(shot_heading - heading)
        if abs(final_turn) > math.radians(3):
            steps.append(f"turn {_side(final_turn)} to face the goal")
        steps.append("creep forward onto the shot line")
    else:
        if abs(left_of_line) >= p.lat_tol:
            steps.append(f"shuffle {abs(left_of_line) * 100:.0f} cm {'right' if left_of_line > 0 else 'left'} onto the shot line")
        if abs(shot_heading) >= p.yaw_tol:
            steps.append(f"turn {_side(shot_heading)} to face the goal")
    steps.append("run through the ball once on the line")
    return Plan((bx, by), (gx, gy), line_up, entry, shot_heading, behind, left_of_line, path, run_end, detour, steps)


class ApproachController:
    """Steps through APPROACH -> ALIGN -> RUN -> BRAKE and gives a velocity command each tick.

    tick(ball, goal, now) takes the two robot-relative points (ball may be None
    if it is not seen) and returns
        {"phase", "cmd": {vx, vy, wz}, "plan": Plan-as-dict or None, "note"}
    cmd is in the body frame: vx forward, vy left (m/s), wz counter-clockwise (rad/s).
    vx is never negative.
    """

    def __init__(self, params: Params = Params()):
        self.p = params
        self.drift = 0.0              # learned sideways correction (m/s); survives between runs on purpose
        self.reset()

    def reset(self):
        self.phase = "APPROACH"
        self._run_started = None      # when the current run began
        self._through_until = None    # set at contact: keep driving straight until then
        self._brake_until = None
        self._last_t = None
        self.note = ""

    def _out(self, vx=0.0, vy=0.0, wz=0.0, plan_=None):
        return {"phase": self.phase, "cmd": {"vx": max(float(vx), 0.0), "vy": float(vy), "wz": float(wz)},
                "plan": plan_.as_dict() if plan_ else None, "note": self.note}

    def tick(self, ball, goal, now: float) -> dict:
        p = self.p
        clamp = lambda v, lim: max(-lim, min(lim, v))  # noqa: E731
        dt = 0.0 if self._last_t is None else max(0.0, min(now - self._last_t, 0.2))
        self._last_t = now

        if self.phase == "BRAKE":
            if now < self._brake_until:
                return self._out()
            self.reset()
            self._last_t = now

        pl = plan(ball, goal, p) if ball is not None else None

        if self.phase == "RUN" and self._through_until is not None:
            # Past contact: straight ahead for the follow-through, whatever the ball does now.
            if now >= self._through_until:
                self.phase, self._brake_until, self.note = "BRAKE", now + p.brake_s, "run finished: stopping"
                return self._out()
            self.note = "through the ball: following through"
            return self._out(p.run_speed, 0.0, 0.0, pl)

        if pl is None:
            self.phase, self._run_started = "APPROACH", None
            self.note = "ball not seen: standing still" if ball is None else "ball is on the aim point: nothing to line up"
            return self._out()

        # Lean the heading back toward the line: on it, face along it; off to one side, angle in.
        steer = pl.shot_heading - math.atan(p.line_lean * pl.left_of_line)
        tracking = self.phase in ("ALIGN", "RUN") and abs(pl.left_of_line) < 0.10
        if tracking:  # a robot that veers sits a little off the line for ever: build up a push against it
            self.drift = clamp(self.drift - p.ki_lat * pl.left_of_line * dt, p.run_strafe)
        side = -p.kp_lat * pl.left_of_line + self.drift    # sideways speed that closes the gap to the line
        turn = p.kp_yaw * steer                            # turn rate that brings the heading round

        if self.phase == "RUN":
            if ball[0] < p.contact + 0.06:                # the bumper is at the ball
                self._through_until = now + p.follow_through / p.run_speed
                self.note = "contact"
                return self._out(p.run_speed, 0.0, 0.0, pl)
            too_long = now - self._run_started > (p.run_max + 0.5) / p.run_speed
            if abs(pl.left_of_line) > p.corridor or pl.behind < 0.1 or too_long:
                self.phase, self._run_started = "APPROACH", None
                self.note = "lost the shot line during the run: lining up again"
                return self._out()                        # stop this tick; the next one plans afresh
            self.note = f"running at the ball: {ball[0] - p.contact:.2f} m to contact, still correcting"
            return self._out(p.run_speed, clamp(side, p.run_strafe), clamp(turn, 0.8), pl)

        in_window = p.run_min <= pl.behind <= p.run_max
        if in_window and abs(pl.left_of_line) < p.lat_tol and abs(pl.shot_heading) < p.yaw_tol:
            # On the line, facing along it, a sensible distance back: go. No pause.
            self.phase, self._run_started = "RUN", now
            self.note = "on the shot line and facing the goal: running at the ball"
            return self._out(p.run_speed, clamp(side, p.run_strafe), clamp(turn, 0.8), pl)

        # Roughly behind the ball and roughly facing the goal: converge onto the line going forward.
        slack = 1.3 if self.phase == "ALIGN" else 1.0     # once converging, stay in this mode through small wobbles
        if (pl.behind > p.run_min - 0.03 and abs(pl.left_of_line) < 0.35 * slack
                and abs(pl.shot_heading) < math.radians(50) * slack):
            self.phase = "ALIGN"
            creep = p.creep_speed if pl.behind > p.run_min + 0.15 else 0.0   # do not creep closer than the window allows
            self.note = (f"converging onto the shot line: {abs(pl.left_of_line) * 100:.0f} cm "
                         f"{'left' if pl.left_of_line > 0 else 'right'} of it, {math.degrees(abs(pl.shot_heading)):.0f} deg off")
            return self._out(creep, clamp(side, p.strafe_speed), clamp(turn, p.turn_rate), pl)

        # Otherwise: get behind the ball. Walk forward to the entry point (round the ball if needed).
        self.phase = "APPROACH"
        wx, wy = pl.path[1]
        leg, bearing = math.hypot(wx, wy), math.atan2(wy, wx)
        if leg < 0.12 and not pl.detour:                  # there: turn to face along the line
            self.note = "behind the ball: turning to face the goal"
            return self._out(0.0, 0.0, clamp(p.kp_yaw * pl.shot_heading, p.turn_rate), pl)
        wz = clamp(p.kp_yaw * bearing, p.turn_rate)
        if abs(bearing) > p.turn_first:
            self.note = "turning toward " + ("the way round the ball" if pl.detour else "the point behind the ball")
            return self._out(0.0, 0.0, wz, pl)
        self.note = "walking " + ("round the ball" if pl.detour else "to get behind the ball")
        return self._out(clamp(p.kp_pos * leg, p.walk_speed) * math.cos(bearing), 0.0, wz, pl)
