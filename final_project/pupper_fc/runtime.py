"""Interactive sandbox orchestration. All motion stays inside Simulator."""
from collections import deque
from dataclasses import asdict, replace
import hashlib
import math
from pathlib import Path

from .controller import Controller
from .model import Decision, Field, Limits, zero
from .simulation import SCENARIOS, Simulator

DT = 0.05
TRACE_LIMIT = 2400


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be between {low} and {high}")
    return float(value)


class Sandbox:
    """One shared local practice session, independently of the viewer."""

    def __init__(self, scenario="center", seed=7, attack_sign=1):
        files = ("model.py", "controller.py", "simulation.py", "runtime.py")
        self.source_hashes = {
            name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in files
        }
        self.settings = dict(delay_ms=0.0, noise_cm=0.0, max_speed=0.65,
                             ball_visible=True, freeze=False)
        self.reset(scenario, seed, attack_sign)

    def reset(self, scenario, seed, attack_sign=1):
        if scenario not in SCENARIOS:
            raise ValueError("Unknown scenario")
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**32:
            raise ValueError("seed must be an integer between 0 and 4294967295")
        if isinstance(attack_sign, bool) or attack_sign not in (-1, 1):
            raise ValueError("attack_sign must be -1 or 1")
        self.scenario, self.seed = scenario, seed
        self.field = Field(attack_sign=attack_sign)
        self.limits = replace(Limits(), vx=self.settings["max_speed"])
        self.sim = Simulator(field=self.field, limits=self.limits, seed=seed, scenario=scenario)
        self.controller = Controller(self.field, self.limits)
        self.running, self.stopped, self.settling = False, False, False
        self.events, self.frames = deque(maxlen=30), deque(maxlen=TRACE_LIMIT)
        self.steps = self.records = 0
        self.metrics = dict(shots=0, goals=0, own_goals=0, simulated_seconds=0.0, phase_changes=0)
        self.observation = self._observe()
        self.decision = Decision("ready", "Press Start, or Step for one control tick.", zero(0))
        self._event()
        self._record()

    def _observe(self):
        return self.sim.observe(delay=self.settings["delay_ms"] / 1000,
                                noise=self.settings["noise_cm"] / 100,
                                ball_visible=self.settings["ball_visible"],
                                frozen=self.settings["freeze"])

    def _event(self):
        self.events.append(dict(time=round(self.sim.time, 3), phase=self.decision.phase,
                                reason=self.decision.reason))

    def _set_decision(self, decision):
        if decision.phase != self.decision.phase:
            self.metrics["phase_changes"] += 1
            if decision.phase == "strike":
                self.metrics["shots"] += 1
            self.decision = decision
            self._event()
        else:
            self.decision = decision

    def advance(self, force=False):
        if not self.running and not force and not self.settling:
            return
        self.observation = self._observe()
        if self.stopped:
            decision = Decision("stopped", "Operator Stop latched. Start explicitly to resume.",
                                zero(self.sim.time))
        else:
            decision = self.controller.tick(self.observation, self.sim.time)
        self._set_decision(decision)
        previous_goal = self.sim.goal
        self.sim.step(decision.command, DT)
        if not previous_goal and self.sim.goal:
            self.metrics["goals" if self.sim.goal > 0 else "own_goals"] += 1
        self.steps += 1
        self.metrics["simulated_seconds"] = round(self.sim.time, 3)
        if self.stopped:
            v = self.sim.velocity
            self.settling = abs(v.vx) + abs(v.vy) + abs(v.wz) > 1e-5
        self._record()

    def snapshot(self):
        raw = self.sim.snapshot()
        # Keep truth available to the display and metrics, never to Controller.
        truth = {key: raw[key] for key in ("robot", "ball", "velocity", "goal", "fallen")}
        return dict(time=round(self.sim.time, 4), running=self.running, stopped=self.stopped,
                    scenario=self.scenario, seed=self.seed, field=asdict(self.field), truth=truth,
                    observation=asdict(self.observation), decision=asdict(self.decision),
                    settings=dict(self.settings), metrics=dict(self.metrics),
                    events=list(self.events), trace_length=len(self.frames))

    def _record(self):
        frame = self.snapshot()
        frame.pop("events", None)
        frame.pop("trace_length", None)
        self.frames.append(frame)
        self.records += 1

    def trace(self):
        return dict(schema="pupper_fc_trace_v1", scope="Approximate 2D sandbox; no robot or policy rollout",
                    source_sha256=dict(self.source_hashes), dt=DT, seed=self.seed, scenario=self.scenario,
                    frames=list(self.frames), frame_limit=TRACE_LIMIT,
                    truncated=self.records > TRACE_LIMIT)

    def control(self, request):
        if not isinstance(request, dict):
            raise ValueError("Control request must be an object")
        action = request.get("action")
        if action == "start":
            self.stopped, self.running, self.settling = False, True, False
            self.controller.reset()
        elif action == "pause":
            self.running = False  # Deliberate time freeze, unlike Stop's braking.
        elif action == "stop":
            self.running, self.stopped, self.settling = False, True, True
            self.controller.reset()
            self._set_decision(Decision("stopped", "Operator Stop latched; simulated braking continues.",
                                        zero(self.sim.time)))
            self._record()
        elif action == "step":
            if self.stopped:
                raise ValueError("Stop is latched; press Start to release it")
            self.running = False
            self.advance(force=True)
        elif action == "reset":
            self.reset(request.get("scenario", self.scenario), request.get("seed", self.seed),
                       request.get("attack_sign", self.field.attack_sign))
        elif action == "resume":
            if self.stopped:
                raise ValueError("Stop is latched; press Start first")
            if not self.sim.goal or self.decision.phase != "await_reset":
                raise ValueError("Wait for the robot to return home after a goal")
            self.sim.restart_play()
            self.controller.reset()
            self.running = True
        elif action == "configure":
            values = request.get("settings")
            if not isinstance(values, dict) or set(values) - set(self.settings):
                raise ValueError("Unknown or missing settings")
            new = dict(self.settings)
            for key, val in values.items():
                if key in ("ball_visible", "freeze"):
                    if not isinstance(val, bool):
                        raise ValueError(f"{key} must be true or false")
                    new[key] = val
                else:
                    lo, hi = {"delay_ms": (0, 800), "noise_cm": (0, 8), "max_speed": (0.15, 1)}[key]
                    new[key] = number(val, key, lo, hi)
            if new["max_speed"] != self.settings["max_speed"]:
                self.limits = replace(self.limits, vx=new["max_speed"])
                self.sim.limits = self.limits
                self.controller = Controller(self.field, self.limits)
            self.settings = new
        elif action in ("move_ball", "move_robot"):
            x = number(request.get("x"), "x", -self.field.length/2, self.field.length/2)
            y = number(request.get("y"), "y", -self.field.width/2, self.field.width/2)
            if action == "move_ball":
                self.sim.move_ball(x, y)
            else:
                yaw = request.get("yaw")
                if yaw is not None:
                    yaw = number(yaw, "yaw", -math.tau, math.tau)
                self.sim.move_robot(x, y, yaw)
                self.controller.reset()
        elif action == "nudge_ball":
            self.sim.nudge_ball(number(request.get("vx", 0.0), "vx", -2, 2),
                                number(request.get("vy", 0.8), "vy", -2, 2))
        elif action == "fall":
            value = request.get("value")
            if not isinstance(value, bool):
                raise ValueError("value must be true or false")
            self.sim.set_fallen(value)
        else:
            raise ValueError("Unknown control action")
        # Refresh display packets only when time is paused; gameplay updates them
        # on its next tick. A frozen packet deliberately keeps its capture time.
        if not self.running:
            self.observation = self._observe()
        if action not in ("reset", "step", "stop"):
            self._record()
        return self.snapshot()
