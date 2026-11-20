"""What is going on, in words: a small state machine for the ball and one for each Pupper.

They read the prediction (ballistics.py) and the plans (intercept.py) and reduce them to a
named state with a reason, so a person watching the dashboard, or the code that will drive
the robots, can see the situation at a glance:

  ball     NOT_SEEN  HIDDEN  STILL  HIT  ROLLING  KNOCKED
  Pupper   role:   OFFENSE | DEFENSE
           state:  HOLD  SEARCH  GO_TO_KICK_POSE  WAIT_FOR_BALL  WIND_UP  KICKING  BLOCK  GUARD  WAIT
           decision (on offence, ball rolling):  intercept | wait_for_stop

Each machine remembers how long it has been in its state and does not change its mind on a
single frame: a new state has to persist for a moment (events such as a hit, and a threat to
the robot's own goal, are taken at once). Nothing here moves a robot.

The two Puppers are on opposite teams: each attacks one goal and defends the other. So "who
is on offence" is decided per robot: it attacks if it can get to the ball first, or if the
ball is in the other half; it defends when the ball is heading into its own goal, or when the
other robot will reach a ball in its half first.
"""
from __future__ import annotations

import math

BALL_STATES = ("NOT_SEEN", "HIDDEN", "STILL", "HIT", "ROLLING", "KNOCKED")
ROBOT_STATES = ("HOLD", "SEARCH", "GO_TO_KICK_POSE", "WAIT_FOR_BALL", "WIND_UP", "KICKING", "BLOCK", "GUARD", "WAIT")


class _Machine:
    """A state with the time it was entered; changes need `dwell_s` of persistence unless forced."""

    def __init__(self, first: str, dwell_s: float):
        self.state, self.why, self.since = first, "", None
        self.dwell_s = dwell_s
        self._want, self._want_since = first, None

    def _go(self, want: str, why: str, now: float, at_once: bool = False) -> None:
        if self.since is None:
            self.since = now
        if want == self.state:
            self.why, self._want = why, want
            return
        if want != self._want:
            self._want, self._want_since = want, now
        if at_once or now - self._want_since >= self.dwell_s:
            self.state, self.why, self.since = want, why, now

    def since_s(self, now: float) -> float:
        return 0.0 if self.since is None else max(now - self.since, 0.0)


def _ending(pred: dict) -> str:
    t = pred.get("end_in_s", 0.0)
    where = f"({pred['end'][0] * 100:+.0f}, {pred['end'][1] * 100:+.0f}) cm"
    return {"stops": f"will stop at {where} in {t:.1f} s", "out": f"rolling out of the field in {t:.1f} s",
            "goal_east": f"rolling into the EAST goal in {t:.1f} s", "goal_west": f"rolling into the WEST goal in {t:.1f} s",
            "horizon": "still rolling beyond the prediction"}.get(pred.get("ends"), "")


class BallMachine(_Machine):
    """NOT_SEEN -> STILL -> HIT -> ROLLING (-> KNOCKED -> ROLLING) -> STILL, with HIDDEN while it is out of sight."""

    def __init__(self, dwell_s: float = 0.15, event_s: float = 0.45):
        super().__init__("NOT_SEEN", dwell_s)
        self.event_s = event_s            # how long a hit or a knock stays on display

    def update(self, pred: dict | None, now: float) -> dict:
        if pred is None:
            self._go("NOT_SEEN", "no ball in view", now)
        elif pred.get("hidden"):
            self._go("HIDDEN", f"not seen for {pred['seen_ago_s']:.1f} s: going by the prediction", now)
        elif not pred["moving"]:
            self._go("STILL", "standing still", now)
        else:
            fit = pred.get("fit") or {}
            knock_ago = fit.get("knock_ago_s")
            recent = knock_ago is not None and knock_ago < self.event_s
            if fit.get("speeding_up") or (recent and fit.get("knocks", 0) == 0):
                self._go("HIT", "speeding up: being pushed or hit", now, at_once=True)
            elif recent:
                self._go("KNOCKED", f"knocked off course {knock_ago:.1f} s ago: direction refitted", now, at_once=True)
            else:
                self._go("ROLLING", f"slowing at {pred['decel']:.2f} m/s2", now)
        out = {"state": self.state, "since_s": self.since_s(now), "why": self.why, "going": "",
               "moving": bool(pred is not None and pred["moving"])}
        if pred is not None and pred["moving"]:
            out["going"] = _ending(pred)
            out["speed"] = pred["speed"]
            out["heading_deg"] = None if pred.get("heading") is None else math.degrees(pred["heading"])
        return out


class RobotMachine(_Machine):
    """One Pupper: which role it should take and what it should be doing about the ball right now."""

    def __init__(self, name: str, windup_s: float = 2.3, dwell_s: float = 0.3, role_dwell_s: float = 1.0,
                 first_margin_s: float = 1.0):
        super().__init__("HOLD", dwell_s)
        self.name = name
        self.role = _Machine("OFFENSE", role_dwell_s)
        self.windup_s, self.first_margin_s = windup_s, first_margin_s
        self._commit_t = None             # when the wind-up was started
        self._at_pose_since = None        # how long it has been standing on the kick pose
        self.settle_s = 0.2

    def update(self, now: float, calibrated: bool, robot: dict | None, ball: dict, plans: dict | None,
               others: dict, own_goal: dict | None = None) -> dict:
        """ball: BallMachine.update()'s output. plans: this robot's intercepts entry.
        others: {name: plans} of the other robots in view (their kick contact times decide who is first)."""
        plans = plans or {}
        kick, block, guard = plans.get("kick") or {}, plans.get("block"), plans.get("guard") or {}
        mine = kick.get("contact_in_s") if kick.get("feasible") else None
        theirs = min((p["kick"]["contact_in_s"] for p in others.values()
                      if (p or {}).get("kick", {}).get("feasible")), default=None)
        first = mine is not None and (theirs is None or mine <= theirs)

        # ---- role ----
        if block is not None:
            self.role._go("DEFENSE", f"the ball is heading into its own goal in {block['enters_in_s']:.1f} s", now, at_once=True)
        elif mine is None:
            self.role._go(self.role.state, "no kick possible right now: keeping its role", now)
        elif theirs is None:
            self.role._go("OFFENSE", "no other robot in view", now)
        else:
            # Stick with the current role unless the other robot is clearly ahead (or behind).
            margin = self.first_margin_s if self.role.state == "OFFENSE" else -self.first_margin_s
            ball_x = kick["ball"][0]
            own_half = own_goal is not None and ball_x * own_goal["x"] > 0
            if mine <= theirs + margin:
                self.role._go("OFFENSE", f"reaches the ball first: {mine:.1f} s against {theirs:.1f} s", now)
            elif own_half:
                self.role._go("DEFENSE", f"the other robot reaches the ball {mine - theirs:.1f} s sooner, in its own half", now)
            else:
                self.role._go("OFFENSE", f"the other robot is {mine - theirs:.1f} s sooner, but the ball is in the other half: press", now)

        # ---- what to do ----
        decision, target = "", None
        kicking = self._commit_t is not None and now - self._commit_t < self.windup_s + 0.6
        if self.state in ("WIND_UP", "KICKING") and not kicking and self._commit_t is not None:
            self.state, self.since, self._commit_t = "WAIT", now, None      # the kick is over: decide afresh
        self._at_pose_since = (self._at_pose_since or now) if kick.get("at_pose") else None
        settled = self._at_pose_since is not None and now - self._at_pose_since >= self.settle_s
        if not calibrated:
            self._go("HOLD", "calibration is not valid: no positions", now, at_once=True)
        elif robot is None or not robot.get("visible"):
            self._go("HOLD", "this robot's tag is not seen", now)
        elif kicking:
            left = self.windup_s - (now - self._commit_t)
            self._go("KICKING", f"wind-up under way: paw reaches the ball in {max(left, 0.0):.1f} s" if left > 0
                     else "kick done: looking again", now, at_once=True)
        elif self.role.state == "DEFENSE" and block is not None:
            target = block["point"] and [*block["point"], None]
            late = block["spare_s"] is not None and block["spare_s"] < 0
            self._go("BLOCK", "stand in the ball's path" + (f": {-block['spare_s']:.1f} s too late, go anyway" if late else
                     f": {block['spare_s']:.1f} s to spare" if block["spare_s"] is not None else ""), now, at_once=True)
        elif ball["state"] == "NOT_SEEN" and not plans:
            self._go("SEARCH", "the ball is not in view: stand still", now)
        elif self.role.state == "DEFENSE":
            target = guard.get("pose")
            self._go("GUARD", "in place between the ball and its goal" if guard.get("at_pose")
                     else "go and stand between the ball and its goal", now)
        elif not kick.get("feasible"):
            self._go("WAIT", kick.get("note", "no kick possible"), now)
        else:
            moving = ball.get("moving", False) and kick["ball_ready_in_s"] > 0
            if moving:
                # The choice the plan has made, in words: commit the wind-up before the ball stops, or not yet.
                decision = "intercept" if kick["sure"] else "wait_for_stop"
            target = kick["pose"] if kick["sure"] else kick["stage"]
            if kick["action"] == "wind_up" and settled:
                self._commit_t = now
                self._go("WIND_UP", f"start the {kick['paw']}-paw kick now", now, at_once=True)
            elif kick["action"] == "wind_up":
                self._go("WAIT_FOR_BALL", "on the pose: settling before the wind-up", now)
            elif kick["action"] == "wait":
                self._go("WAIT_FOR_BALL", (f"in place: the wind-up starts in {kick['commit_in_s']:.1f} s"
                                           + (", before the ball stops" if moving else "")) if kick["sure"]
                         else f"waiting clear of the ball: {kick['unsure_why']}", now)
            else:
                self._go("GO_TO_KICK_POSE", (f"{kick['paw']} paw: " + ("stopping point known, go to the pose" if kick["sure"]
                                              else f"go to the waiting point ({kick['unsure_why']})")), now)
        return {"role": self.role.state, "role_why": self.role.why, "role_since_s": self.role.since_s(now),
                "state": self.state, "why": self.why, "since_s": self.since_s(now), "decision": decision,
                "target": target, "first_to_ball": bool(first), "contact_in_s": mine}
