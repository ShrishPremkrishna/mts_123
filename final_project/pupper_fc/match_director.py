"""One camera-side match referee. Fan out each return value with its original frame."""
import math
import time

from .controller import MAX_BALL_SPEED
from final_project.overhead_perception.fieldcam.observations import capture_measurement
from final_project.overhead_perception.fieldcam.pitch import Bounds, Pitch

FRESH = .30
ROLE_ADVANTAGE = .12  # metres of distance advantage before the attacker relinquishes the ball
HANDOVER = .35        # expires the former attacker's final 300 ms capture lease


def number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def measured(row, capture, age):
    try:
        estimate = capture_measurement(row, capture, age)
    except ValueError:
        return None
    return (estimate["x"], estimate["y"]) if estimate is not None else None


def directive_ok(row, epoch, roster, seq):
    if (not isinstance(row, dict) or row.get("epoch") != epoch or type(seq) is not int
            or type(row.get("seq")) is not int or row["seq"] != seq or row.get("roster") != roster
            or type(row.get("rally")) is not int or row["rally"] < 1
            or type(row.get("role_cycle")) is not int or row["role_cycle"] < 0
            or type(row.get("reset_ready")) is not bool or type(row.get("goal_sign")) is not int):
        return False
    phase, sign, who = row.get("phase"), row["goal_sign"], row.get("attacker")
    return ((phase == "playing" and who in roster and sign == 0 and not row["reset_ready"])
            or (phase == "goal" and who is None and sign in (-1, 1))
            or (phase in ("holding","pending") and who is None and sign == 0 and not row["reset_ready"]))



def goal_geometry(goals, length, width, radius):
    """Measured goal planes must sit on opposing halves and enclose the ball."""
    if (not isinstance(goals, dict) or not all(number(v) and v > 0 for v in (length, width, radius))
            or 2 * radius >= min(length, width)):
        return False
    for name, sign in (("west", -1), ("east", 1)):
        row = goals.get(name)
        if not isinstance(row, dict) or not all(number(row.get(k)) for k in ("x", "y_min", "y_max")):
            return False
        if not (0 < sign * row["x"] <= length / 2 + radius
                and -width / 2 <= row["y_min"] < row["y_max"] <= width / 2
                and row["y_max"] - row["y_min"] > 2 * radius):
            return False
    return True


class MatchDirector:
    def __init__(self, epoch, tracks, attack_signs, *, goal_lines=None, pending_start=False, pitch=None):
        if (not isinstance(epoch, str) or not 0 < len(epoch) <= 96 or not isinstance(tracks, dict) or not tracks
                or len(set(tracks.values())) != len(tracks) or not set(tracks.values()) <= set(attack_signs)
                or set(attack_signs) != {"pupper-1", "pupper-2"} or set(attack_signs.values()) != {-1, 1}):
            raise ValueError("match requires one or two distinct team tracks and opposite attack directions")
        self.epoch, self.tracks, self.signs = epoch, dict(tracks), dict(attack_signs)
        if type(pending_start) is not bool:
            raise ValueError("pending_start must be boolean")
        self.pending_start, self.pending_capture = pending_start, None
        self.balance_paused = False
        self.pitch = Pitch.read(pitch) if pitch is not None else None
        if self.pitch is not None and goal_lines is not None and goal_lines != self.pitch.goals:
            raise ValueError("referee goals differ from the selected pitch")
        self.goal_lines = self.pitch.goals if self.pitch is not None else goal_lines
        self.kick_owner = None
        self.retired_kicks = set()
        self.revision = None
        self.calibration_changed = False
        self.score = dict.fromkeys(attack_signs, 0)
        self.last_seq, self.last_capture = -1, -math.inf
        self.previous = self.owner = self.challenger = self.switch_at = None
        self.crossing = {-1: False, 1: False}
        self.rally, self.role_cycle, self.goal_sign, self.reset_ready = 1, 0, 0, False
        self.state = self._state(-1, "pending" if pending_start else "holding", None)

    def _state(self, seq, phase, attacker):
        fenced = self.balance_paused or phase in ("pending", "holding")
        result = {"epoch": self.epoch, "seq": seq, "phase": phase, "attacker": attacker,
                "roster": sorted(self.tracks.values()), "tracks": dict(self.tracks), "rally": self.rally, "role_cycle": self.role_cycle,
                "score": dict(self.score), "goal_sign": 0 if fenced else self.goal_sign,
                "reset_ready": False if fenced else self.reset_ready}
        if self.pitch is not None:
            result["pitch"] = self.pitch.record()
        return result

    def _missing(self):
        self.previous = None
        self.crossing = {-1: False, 1: False}
        self.reset_ready = False
        self.challenger = self.switch_at = None

    def observe_kick(self, event):
        """Keep the current attacking role through its native clip and return to walking."""
        if not isinstance(event, dict) or event.get("epoch") != self.epoch:
            return False
        robot, attempt = event.get("robot_id"), event.get("attempt_id")
        if robot not in self.tracks.values() or not isinstance(attempt, str):
            return False
        edge = (robot, attempt)
        if event.get("state") == "engaged":
            if (self.owner != robot or edge in self.retired_kicks or self.kick_owner not in (None, edge)
                    or not attempt.startswith(f"r{self.rally}-p{self.role_cycle}-k")):
                return False
            self.kick_owner = edge
            self.challenger = self.switch_at = None
            return True
        if event.get("state") in ("completed", "retired") and self.kick_owner == edge:
            self.retired_kicks.add(edge)
            self.kick_owner = None
            return True
        return False

    def _roles(self, positions, ball, capture):
        if self.kick_owner is not None:
            return self.kick_owner[0]
        distances = {r: math.dist(p, ball) for r, p in positions.items()}
        nearest = min(distances, key=lambda r: (distances[r], r))
        if self.owner is None:
            self.owner = nearest
        if nearest == self.owner or distances[nearest] + ROLE_ADVANTAGE >= distances[self.owner]:
            self.challenger = self.switch_at = None
            return self.owner
        if self.challenger != nearest:
            self.challenger, self.switch_at = nearest, capture
        if capture - self.switch_at < HANDOVER:
            return None
        self.owner, self.challenger, self.switch_at = nearest, None, None
        self.role_cycle += 1
        return self.owner

    def _ball(self, capture, point, goals, radius, width):
        bounds = self.pitch.playable_bounds if self.pitch is not None else Bounds(
            goals["west"]["x"], goals["east"]["x"], -width / 2, width / 2)
        inside = bounds.contains(*point, margin=radius + 1e-12)
        if self.goal_sign:
            self.reset_ready = inside
        elif self.previous is not None:
            previous_at, prev = self.previous
            dt = capture - previous_at
            if dt > FRESH or math.dist(prev, point) > MAX_BALL_SPEED * dt:
                self.crossing = {-1: False, 1: False}
            else:
                for sign, name in ((-1, "west"), (1, "east")):
                    goal = goals[name]
                    a, b = sign * (prev[0] - goal["x"]), sign * (point[0] - goal["x"])
                    if b <= -radius:
                        self.crossing[sign] = False
                        continue
                    if b == a and self.crossing[sign]:
                        self.crossing[sign] = all(goal["y_min"] + radius <= y <= goal["y_max"] - radius
                                                  for y in (prev[1], point[1]))
                        continue
                    if b <= a or a > radius:
                        self.crossing[sign] = False
                        continue
                    if a <= -radius < b:
                        self.crossing[sign] = True
                    if not self.crossing[sign]:
                        continue
                    def ordinate(x):
                        return prev[1] + (point[1] - prev[1]) * ((x - a) / (b - a))
                    ends = [ordinate(max(-radius, a)), ordinate(min(radius, b))]
                    if min(ends) < goal["y_min"] + radius or max(ends) > goal["y_max"] - radius:
                        self.crossing[sign] = False
                    if b >= radius and self.crossing[sign]:
                        self.goal_sign = sign
                        scorer = next(r for r, direction in self.signs.items() if direction == sign)
                        self.score[scorer] += 1
                        self.reset_ready = False
                        break
        self.previous = (capture, point)

    def update(self, frame, *, now=None):
        now = time.time() if now is None else now
        seq, capture, age = frame.get("seq"), frame.get("captured_at"), frame.get("frame_age_s")
        if (type(seq) is not int or seq <= self.last_seq or not number(capture) or capture <= self.last_capture):
            return self._state(seq, "holding", None)
        self.last_capture, self.last_seq = capture, seq
        if self.balance_paused:
            self.state = self._state(seq, "holding", None)
            return self.state
        phase, attacker = "holding", None
        f, goals = frame.get("field", {}), self.goal_lines or frame.get("goals", {})
        geometry = isinstance(f, dict) and goal_geometry(goals, f.get("length"), f.get("width"), f.get("ball_radius"))
        if self.pitch is not None:
            geometry = (isinstance(f, dict) and frame.get("pitch") == self.pitch.record()
                        and f.get("length") == self.pitch.marker_bounds.length
                        and f.get("width") == self.pitch.marker_bounds.width
                        and f.get("ball_radius") == self.pitch.ball_radius)
            if frame.get("pitch") != self.pitch.record():
                self.calibration_changed = True
        revision = frame.get("calibration_revision")
        if self.revision is not None and revision != self.revision:
            self.calibration_changed = True
        good = (geometry and not self.calibration_changed and frame.get("calibration_valid") is True
                and type(revision) is int and revision > 0 and number(age) and 0 <= age <= FRESH
                and number(now) and 0 <= now - capture <= FRESH)
        if good and self.revision is None:
            self.revision = revision
        robots = frame.get("robots")
        ball = measured(frame.get("ball"), capture, age) if good else None
        robots = robots if isinstance(robots, dict) else {}
        poses = {robot: measured(robots.get(track), capture, age) for track, robot in self.tracks.items()} if good else {}
        if self.pending_start:
            phase = "pending"
            bounds = self.pitch.playable_bounds if self.pitch is not None else (
                Bounds(-f["length"]/2, f["length"]/2, -f["width"]/2, f["width"]/2) if good else None)
            within = (ball is not None and poses and all(poses.values())
                      and all(bounds.contains(x, y) for x,y in [ball,*poses.values()]))
            self.pending_capture = capture if within else None
        elif ball is None:
            self._missing()
        else:
            self._ball(capture, ball, goals, f["ball_radius"], f["width"])
            if not self.goal_sign and all(poses.values()):
                attacker = self._roles(poses, ball, capture)
                if attacker:
                    phase = "playing"
            elif not all(poses.values()):
                self.challenger = self.switch_at = None
        if self.goal_sign and not self.pending_start:
            phase = "goal"
        self.state = self._state(seq, phase, attacker)
        if geometry:
            self.state["goal_lines"] = {name: {key: goals[name][key] for key in ("x", "y_min", "y_max")}
                                        for name in ("west", "east")}
        return self.state

    def pause_balance(self):
        self.balance_paused = True
        self._missing()
        self.state = self._state(self.last_seq, "holding", None)

    def after_balance(self, epoch):
        """A fresh authority starts behind the kickoff fence with the same scoreboard."""
        if not self.balance_paused or self.calibration_changed or self.kick_owner is not None:
            raise ValueError("End the match and check the field or native kick before starting again")
        director = MatchDirector(epoch, self.tracks, self.signs, goal_lines=self.goal_lines, pending_start=True,
                                 pitch=self.pitch.record() if self.pitch is not None else None)
        director.score = dict(self.score)
        director.goal_sign, director.rally, director.role_cycle = self.goal_sign, self.rally, self.role_cycle
        director.state = director._state(-1, "pending", None)
        return director

    def commit_start(self, *, now=None):
        if self.balance_paused:
            raise ValueError("The match is paused for balance feedback")
        if not self.pending_start:
            return self.state
        now = time.time() if now is None else now
        if (self.pending_capture is None or not number(now) or self.calibration_changed
                or not 0 <= now - self.pending_capture <= FRESH):
            raise ValueError("kickoff requires fresh measured participants and ball")
        self.pending_start, self.pending_capture = False, None
        self.state = self._state(self.last_seq, "holding", None)
        return self.state

    def resume(self, *, now=None):
        now = time.time() if now is None else now
        if self.kick_owner is not None or not self.goal_sign or not self.reset_ready or not 0 <= now - self.last_capture <= FRESH:
            raise ValueError("a fresh measured ball reset inside the field is required")
        self.rally += 1
        self.goal_sign, self.reset_ready = 0, False
        self.previous = self.owner = self.challenger = self.switch_at = None
        self.crossing = {-1: False, 1: False}
        self.state = self._state(self.last_seq, "holding", None)
        return self.state
