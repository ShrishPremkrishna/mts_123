#!/usr/bin/env python3
"""Rehearse the proposed Executor/Controller local-docking seam on a temporary copy.

The controller and integration modules belong to another owner, so this script
does not edit them. It copies the current final_project/pupper_fc package to a
temporary directory, applies the exact anchored insertions below (failing loudly
if an anchor moved), and runs one synthetic closed loop through the real
Executor and Controller:

  overhead ball 4.5 cm short of the truth (the range error a radius mismatch
  causes), robot docked on that biased ball, Pi-local measurements of the true
  body-frame offset at their own capture times.

Without the seam the copy reaches kick_ready off the true contact window. With
a synthetic local profile it must not; it steps toward the true paw offset
using local evidence that expires on motion, and is kick_ready only inside the
true window. With the default live profile it behaves exactly as without it.
With overhead 0.20 m off (beyond the agreement budget) an enabled profile
blocks every kick: overhead docking alone never authorizes one.
All values are synthetic; nothing here is physical evidence.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]

CONTROLLER = [
    ("             preserve_target_heading: bool = False, avoid_ball: bool = False) -> Decision:",
     "             preserve_target_heading: bool = False, avoid_ball: bool = False,\n"
     "             local_docking=None) -> Decision:"),
    ("            return self._paw_attack(robot, ball, goal, observation.captured_at, now, robot_id)",
     "            return self._paw_attack(robot, ball, goal, observation.captured_at, now, robot_id, local_docking)"),
    ("    def _paw_attack(self, robot, ball, goal, captured_at, now, robot_id):",
     "    def _paw_attack(self, robot, ball, goal, captured_at, now, robot_id, local_docking=None):"),
    ("        heading_error = _wrap(pose.yaw - robot.yaw)\n"
     "        # Enter near the middle of the contact window;",
     "        heading_error = _wrap(pose.yaw - robot.yaw)\n"
     "        if local_docking is not None and local_docking.status == \"local\":\n"
     "            return self._local_dock(local_docking, robot, pose, target, heading_error, forward_m, now, robot_id)\n"
     "        # Enter near the middle of the contact window;"),
    ("            if ready:\n"
     "                cmd = replace(cmd, expires_at=min(cmd.expires_at, captured_at + MAX_CAPTURE_GAP_S))",
     "            if ready and local_docking is not None and local_docking.status == \"blocked\":\n"
     "                ready = False  # an enabled local profile: overhead alone cannot authorize the kick\n"
     "            if ready:\n"
     "                cmd = replace(cmd, expires_at=min(cmd.expires_at, captured_at + MAX_CAPTURE_GAP_S))"),
    ("    def _play(self, robot, ball, now, robot_id, captured_at, shot_target=None):",
     "    def _local_dock(self, local, robot, pose, target, heading_error, forward_m, now, robot_id):\n"
     "        \"\"\"Dock on a separately timed body-frame ball; overhead keeps pose and heading.\"\"\"\n"
     "        from .local_ball import body_contact_window, local_kick_ready\n"
     "        from .paw_approach import PAW_LEFT_M\n"
     "        distance = math.hypot(pose.x - robot.x, pose.y - robot.y)\n"
     "        self._paw.docked = body_contact_window(local.forward, local.lateral, heading_error, self.kick_side,\n"
     "                                               scale=1.0 if self._paw.docked else 0.55, forward_m=forward_m)\n"
     "        if self._paw.docked:\n"
     "            cmd = self._paw.ramp(0, 0, 0, self.limits, now, robot_id)\n"
     "            ready = (not (cmd.vx or cmd.vy or cmd.wz)\n"
     "                     and local_kick_ready(local, now, heading_error, self.kick_side, forward_m=forward_m))\n"
     "            cmd = replace(cmd, expires_at=min(cmd.expires_at, local.expires_at))\n"
     "            return Decision(\"kick_ready\" if ready else \"align\",\n"
     "                            \"local paw ready\" if ready else \"confirming a steady local ball\",\n"
     "                            cmd, target, pose.yaw, distance, heading_error)\n"
     "        ahead, left = local.forward - forward_m, local.lateral - PAW_LEFT_M[self.kick_side]\n"
     "        c, s = math.cos(robot.yaw), math.sin(robot.yaw)\n"
     "        corrected = (robot.x + c * ahead - s * left, robot.y + s * ahead + c * left)\n"
     "        decision = self._drive(\"align\", \"local ball docking correction\", robot, corrected, pose.yaw,\n"
     "                               ALIGN_SPEED, now, robot_id)\n"
     "        return replace(decision, command=replace(decision.command,\n"
     "                                                 expires_at=min(decision.command.expires_at, local.expires_at)))\n"
     "\n"
     "    def _play(self, robot, ball, now, robot_id, captured_at, shot_target=None):"),
]

INTEGRATION = [
    ("    def __init__(self, fields=None, limits=Limits(), *, obstacle_config=None,",
     "    def __init__(self, fields=None, limits=Limits(), *, obstacle_config=None, local_docking_profile=None,"),
    ("        self.controllers = {token(r, \"robot_id\"): Controller(",
     "        from .local_ball import LocalDocking\n"
     "        self.local_docking = {r: LocalDocking(local_docking_profile) for r in fields}\n"
     "        self.controllers = {token(r, \"robot_id\"): Controller("),
    ("    def _clear(self):\n",
     "    def _clear(self):\n"
     "        for docking in self.local_docking.values():\n"
     "            docking.reset(\"executor cleared\")  # Stop and new epochs end body-frame evidence\n"),
    ("    def _hold_external(self, robot_id, now):\n",
     "    def _hold_external(self, robot_id, now):\n"
     "        self.local_docking[robot_id].unknown_motion(now, \"external hold: motion outside this executor\")\n"),
    ("        self.track_identity.update(identity_updates)\n",
     "        self.track_identity.update(identity_updates)\n"
     "        for robot_id, observation in observations.items():\n"
     "            self.local_docking[robot_id].note_overhead(observation.robot, observation.captured_at, ball, stamp)\n"),
    ("                    decision = controller.tick(observation, now, **kwargs)",
     "                    if self.kick_side is not None and \"target_pose\" not in kwargs:\n"
     "                        kwargs[\"local_docking\"] = self.local_docking[robot_id].evidence(\n"
     "                            now, observation.robot, observation.captured_at)\n"
     "                    decision = controller.tick(observation, now, **kwargs)"),
    ("                if action == \"finish_kick\":\n",
     "                if action == \"finish_kick\":\n"
     "                    self.local_docking[robot_id].unknown_motion(now, \"kick clip moves the body\")\n"),
    ("        for robot_id, decision in decisions.items():\n            intent = self.intents[robot_id]\n",
     "        for robot_id, decision in decisions.items():\n"
     "            command = decision.command\n"
     "            self.local_docking[robot_id].note_command(now, command.vx, command.vy, command.wz)\n"
     "            intent = self.intents[robot_id]\n"),
    ("    def handle(self, event):\n",
     "    def observe_local_ball(self, robot_id, row, now):\n"
     "        \"\"\"Trusted Pi-local adapter input: one MetricBallReceiver row, separately timed.\"\"\"\n"
     "        if robot_id not in self.controllers:\n"
     "            raise ValueError(\"unknown robot_id\")\n"
     "        return self.local_docking[robot_id].observe_row(row, number(now, \"now\"))\n"
     "\n"
     "    def handle(self, event):\n"),
]

SCENARIO = r'''
import json, math, sys
from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.local_ball import live_profile, synthetic_profile
from final_project.pupper_fc.model import Field
from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M, PAW_LEFT_M, contact_window
from final_project.pupper_fc.model import Ball, Pose

FWD, LEFT = COMBINED_PAW_FORWARD_M["left"], PAW_LEFT_M["left"]
GOAL = (1.2, 0.)
fields = {"pupper-1": Field(ball_radius=.11), "pupper-2": Field(attack_sign=-1, ball_radius=.11)}

def local_row(t, x, y, seq):
    return {"state": "measured", "selected": 0, "captured_at_mono_ns": round(t * 1e9), "producer_instance": "cam",
            "sequence": seq, "candidates": [{"method": "pinkfloor_fit", "x": x, "y": y, "sigma_xy_m": .002}],
            "identity": {"profile_id": "a" * 64, "calibration_id": "b" * 64, "ball_radius_m": .11,
                         "status": "synthetic", "radius_measured": False, "mount_measured": False}}

def run(mode, BIAS=.045):
    profile = {"none": None, "live": live_profile(fields["pupper-1"]),
               "synthetic": synthetic_profile(.11), "synthetic-disagree": synthetic_profile(.11)}[mode]
    ex = Executor(fields, kick_side="left", paw_forward_m=FWD, local_docking_profile=profile)
    t, dt = 10., .05
    ex.handle({"event": "arm", "epoch": "e", "clock": "c", "now": t})
    truth = Ball(0., 0.)
    robot = Pose(-FWD - BIAS, -LEFT, 0.)  # docked on the biased overhead ball
    seq, phases, first_ready = 0, [], None
    for tick in range(160):
        t += dt
        if tick % 2 == 0:  # 10 Hz overhead and local captures, both at time t
            seq += 1
            c, s = math.cos(robot.yaw), math.sin(robot.yaw)
            seen = Ball(truth.x - c * BIAS, truth.y - s * BIAS)
            ex.handle({"event": "frame", "epoch": "e", "clock": "c", "seq": seq, "captured_at": t, "now": t,
                       "robots": {"pupper-1": {"x": robot.x, "y": robot.y, "yaw": robot.yaw, "captured_at": t}},
                       "ball": {"x": seen.x, "y": seen.y}})
            ex.handle({"event": "intent", "epoch": "e", "clock": "c", "seq": seq, "robot_id": "pupper-1",
                       "intent_id": "shot", "kind": "shoot", "target": {"x": GOAL[0], "y": GOAL[1]},
                       "issued_at": t, "valid_until": t + .4, "now": t})
            if profile is not None:
                dx, dy = truth.x - robot.x, truth.y - robot.y
                ex.observe_local_ball("pupper-1", local_row(t, dx * c + dy * s, -dx * s + dy * c, seq), t + .01)
        decision = ex._plan_tick(t)["pupper-1"]
        phases.append(decision.phase)
        cmd = decision.command
        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        robot = Pose(robot.x + (c * cmd.vx - s * cmd.vy) * dt, robot.y + (s * cmd.vx + c * cmd.vy) * dt,
                     robot.yaw + cmd.wz * dt)
        if decision.phase == "kick_ready" and first_ready is None:
            heading = math.atan2(GOAL[1] - truth.y, GOAL[0] - truth.x) - robot.yaw
            first_ready = dict(t=round(t - 10, 2), in_true_window=contact_window(robot, truth, heading, "left",
                                                                                 forward_m=FWD),
                               offset=[round(v, 4) for v in ((truth.x - robot.x), (truth.y - robot.y))],
                               reason=decision.reason)
    return {"mode": mode, "first_kick_ready": first_ready,
            "phases": {p: phases.count(p) for p in dict.fromkeys(phases)}}

print(json.dumps([run(m) for m in ("none", "live", "synthetic")] + [run("synthetic-disagree", .20)]))
'''


def apply(path, insertions):
    text = path.read_text()
    for anchor, replacement in insertions:
        if text.count(anchor) != 1:
            raise SystemExit(f"seam anchor moved or repeated in {path.name}: {anchor.splitlines()[0]!r}")
        text = text.replace(anchor, replacement)
    path.write_text(text)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        package = Path(tmp) / "final_project/pupper_fc"
        shutil.copytree(ROOT / "final_project/pupper_fc", package, ignore=shutil.ignore_patterns("__pycache__", "tests",
                                                                                               "static", "media"))
        apply(package / "controller.py", CONTROLLER)
        apply(package / "integration.py", INTEGRATION)
        # The patched pupper_fc copy shadows the original; other final_project
        # namespace portions (e.g. overhead_perception) still load from the tree.
        prelude = f"import sys; sys.path[:0] = [{tmp!r}, {str(ROOT)!r}]\n"
        result = subprocess.run([sys.executable, "-I", "-c", prelude + SCENARIO],
                                cwd=tmp, text=True, capture_output=True)
        if result.returncode:
            raise SystemExit(result.stderr)
    runs = json.loads(result.stdout)
    print(json.dumps(runs, indent=1))
    none, live, synthetic, disagree = runs
    ok = (none["first_kick_ready"] is not None and not none["first_kick_ready"]["in_true_window"]
          and live == {**none, "mode": "live"}
          and synthetic["first_kick_ready"] is not None and synthetic["first_kick_ready"]["in_true_window"]
          and disagree["first_kick_ready"] is None)
    print("seam rehearsal:", "as expected" if ok else "UNEXPECTED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
