"""Interactive sandbox orchestration. All motion stays inside Simulator."""
from collections import deque
from copy import copy, deepcopy
from dataclasses import asdict, replace
import hashlib
import math
from pathlib import Path

from .integration import Executor
from .kick_handoff import KickHandoff
from .model import Decision, Field, Limits, zero
from .simulation import SCENARIOS, SimulatedKick, Simulator

DT = 0.05
TRACE_LIMIT = 2400
ROBOT_ID = "pupper-1"
INTENT_LEASE = 0.5


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be between {low} and {high}")
    return float(value)


class Sandbox:
    """One shared local practice session, independently of the viewer."""

    def __init__(self, scenario="center", seed=7, attack_sign=1, *, kick_side=None):
        self.kick_side = kick_side
        files = ("model.py", "controller.py", "simulation.py", "runtime.py",
                 "integration.py", "tactics.py", "obstacles.py")
        if kick_side is not None:
            files += ("paw_approach.py", "kick_handoff.py")
        self.source_hashes = {
            name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in files
        }
        self.settings = dict(delay_ms=0.0, noise_cm=0.0, max_speed=0.30 if kick_side else 0.65,
                             ball_visible=True, freeze=False, renew_intent=True)
        self._epoch_sequence = 0
        self.reset(scenario, seed, attack_sign)

    def reset(self, scenario, seed, attack_sign=1):
        if scenario not in SCENARIOS:
            raise ValueError("Unknown scenario")
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**32:
            raise ValueError("seed must be an integer between 0 and 4294967295")
        if type(attack_sign) is not int or attack_sign not in (-1, 1):
            raise ValueError("attack_sign must be -1 or 1")
        self.scenario, self.seed = scenario, seed
        self.field = Field(attack_sign=attack_sign)
        self.limits = replace(Limits(), vx=self.settings["max_speed"])
        if self.kick_side:
            self.field = replace(self.field, goal_width=.6096, ball_radius=.11)
            self.limits = replace(self.limits, vy=self.limits.vx, wz=1.0,
                                  acceleration=.65, yaw_acceleration=2.0)
        self.sim = Simulator(field=self.field, limits=self.limits, seed=seed, scenario=scenario,
                             bumper_contact=self.kick_side is None)
        self.executor = Executor(fields={ROBOT_ID: self.field}, limits=self.limits, kick_side=self.kick_side)
        self.handoff = KickHandoff(ROBOT_ID) if self.kick_side else None
        self.actuator = SimulatedKick(self.handoff) if self.handoff else None
        self.frame_seq = self.intent_seq = 0
        self._last_observation = None
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

    @property
    def controller(self):
        """The executor's single controller; no separate gameplay engine."""
        return self.executor.controllers[ROBOT_ID]

    def _arm(self):
        self._epoch_sequence += 1
        self.executor.arm(f"practice-{self._epoch_sequence}", "simulation", self.sim.time)
        self.frame_seq = self.intent_seq = 0
        self._last_observation = None

    def _frame(self, executor, observation, seq):
        robot = None if observation.robot is None else {
            **asdict(observation.robot), "captured_at": observation.captured_at,
            "fallen": observation.fallen, "goal": observation.goal}
        return dict(event="frame", now=self.sim.time, epoch=executor.epoch,
                    clock=executor.clock, seq=seq, captured_at=observation.captured_at,
                    robots={ROBOT_ID: robot},
                    ball=asdict(observation.ball) if observation.ball is not None else None,
                    goals={"left": dict(x=-self.field.length / 2, y=0),
                           "right": dict(x=self.field.length / 2, y=0)})

    def _admit_observation(self):
        # Only identical packets are cached evidence. An operator edit at the
        # same capture must reach core admission and invalidate conflicting data.
        if self.observation == self._last_observation:
            return
        self.frame_seq += 1
        result = self.executor.handle(self._frame(self.executor, self.observation, self.frame_seq))
        if result["accepted"]:
            self._last_observation = self.observation

    def _renew_intent(self):
        if not self.settings["renew_intent"]:
            return
        self.intent_seq += 1
        self.executor.handle(dict(event="intent", now=self.sim.time,
            epoch=self.executor.epoch, clock=self.executor.clock, seq=self.intent_seq,
            robot_id=ROBOT_ID, intent_id="practice-auto", kind="auto",
            issued_at=self.sim.time, valid_until=self.sim.time + INTENT_LEASE))

    def _resume_candidate(self):
        """Validate a simulator-only replace-and-resume transaction on copies.

        Current observed home/heading/freshness must pass before replacement can
        produce its new capture. No cue renews authority or fabricates a clock.
        History entries are frozen dataclasses: copy its deque, not every frame.
        """
        if self.stopped or not self.executor.armed:
            raise ValueError("Start practice before resuming a goal")
        if not self.sim.goal:
            raise ValueError("No goal is waiting for replacement and resume")
        intent = self.executor.intents[ROBOT_ID]
        if intent is None or self.sim.time >= intent.valid_until:
            raise ValueError("Resume requires current play instructions; enable renewal and Step")
        history = copy(self.sim._history)
        sim = deepcopy(self.sim, {id(self.sim._history): history})
        # The candidate may validate input but cannot emit a kick. Keep the
        # live receipt owner if this transaction replaces the executor.
        executor = deepcopy(self.executor, {id(self.handoff): self.handoff} if self.handoff else {})
        observed = sim.observe(delay=self.settings["delay_ms"] / 1000,
                               noise=self.settings["noise_cm"] / 100,
                               ball_visible=self.settings["ball_visible"],
                               frozen=self.settings["freeze"])
        # Clearing a goal is the proposed operator replacement, not permission
        # to bypass the current observed pose, fall state or capture age.
        executor.controllers[ROBOT_ID].check_resume_after_goal(
            replace(observed, goal=0), self.sim.time)
        if self.sim.time <= executor.capture_highwater:
            raise ValueError("Resume needs a new simulated capture; Step once")
        sim.restart_play()
        observation = sim.observe(delay=self.settings["delay_ms"] / 1000,
                                  noise=self.settings["noise_cm"] / 100,
                                  ball_visible=self.settings["ball_visible"],
                                  frozen=self.settings["freeze"])
        seq = self.frame_seq + 1
        admitted = executor.handle(self._frame(executor, observation, seq))
        if not admitted["accepted"]:
            raise ValueError(admitted["reason"])
        executor.check_resume(ROBOT_ID, seq, self.sim.time)
        result = executor.handle(dict(event="resume", now=self.sim.time,
            epoch=executor.epoch, clock=executor.clock, robot_id=ROBOT_ID, frame_seq=seq))
        if not result["accepted"]:
            raise ValueError(result["reason"])
        return sim, executor, observation, seq

    def _execution(self):
        intent = self.executor.intents[ROBOT_ID]
        live_intent = intent if intent is not None and self.sim.time < intent.valid_until else None
        frame = self.executor.frame
        try:
            self._resume_candidate()
            available, reason = True, "Ready to replace the simulated ball and resume"
        except ValueError as exc:
            available, reason = False, str(exc)
        return dict(armed=self.executor.armed, epoch=self.executor.epoch,
                    frame_captured_at=frame[1][ROBOT_ID].captured_at if frame else None,
                    frame_seq=self.executor.frame_seq if frame else None,
                    action=live_intent.kind if live_intent else None,
                    intent_id=live_intent.identity[0] if live_intent else None,
                    intent_valid_until=intent.valid_until if intent else None,
                    lease_remaining_s=max(0.0, intent.valid_until - self.sim.time) if intent else 0.0,
                    external_hold=self.executor.external_holds[ROBOT_ID],
                    resume_available=available, resume_reason=reason)

    def _event(self):
        intent = self.executor.intents[ROBOT_ID]
        self.events.append(dict(time=round(self.sim.time, 3), phase=self.decision.phase,
                                role=self.decision.role, action=intent.kind if intent else None,
                                reason=self.decision.reason))

    def _set_decision(self, decision):
        old = self.decision
        if decision.phase != old.phase:
            self.metrics["phase_changes"] += 1
            if decision.phase == "strike":
                self.metrics["shots"] += 1
        self.decision = decision
        # A role/hold-reason change matters even when the movement phase does not.
        if (decision.phase != old.phase or decision.role != old.role or
                ("hold" in decision.phase and decision.reason != old.reason)):
            self._event()

    def _control_tick(self):
        if self.handoff is None:
            return self.executor.tick(self.sim.time)[ROBOT_ID]
        step = self.executor.kick_step(self.handoff, self.sim.time)
        self.actuator.update(step, self.sim.time)
        if step.request is not None:
            self.metrics["shots"] += 1
        return step.decision

    def advance(self, force=False):
        if not self.running and not force and not self.settling:
            return
        self.observation = self._observe()
        if self.executor.armed:
            self._admit_observation()
            self._renew_intent()
        decision = self._control_tick()
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
                    scenario=self.scenario, seed=self.seed, kick_side=self.kick_side,
                    field=asdict(self.field), truth=truth,
                    observation=asdict(self.observation), decision=asdict(self.decision),
                    execution=self._execution(),
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
            if not self.executor.armed:
                self._arm()
            self.stopped, self.running, self.settling = False, True, False
        elif action == "pause":
            self.running = False  # Deliberate time freeze, unlike Stop's braking.
        elif action == "stop":
            self.running, self.stopped, self.settling = False, True, True
            self.executor.stop()
            self._set_decision(self._control_tick())
            self._record()
        elif action == "step":
            if self.stopped:
                raise ValueError("Stop is latched; press Start to release it")
            if not self.executor.armed:
                self._arm()
            self.running = False
            self.advance(force=True)
        elif action == "reset":
            self.reset(request.get("scenario", self.scenario), request.get("seed", self.seed),
                       request.get("attack_sign", self.field.attack_sign))
        elif action == "resume":
            sim, executor, observation, seq = self._resume_candidate()
            self.sim, self.executor, self.observation = sim, executor, observation
            self.frame_seq, self._last_observation = seq, observation
            self.running = True
            self._set_decision(self._control_tick())
        elif action == "configure":
            values = request.get("settings")
            if not isinstance(values, dict) or set(values) - set(self.settings):
                raise ValueError("Unknown or missing settings")
            new = dict(self.settings)
            for key, val in values.items():
                if key in ("ball_visible", "freeze", "renew_intent"):
                    if not isinstance(val, bool):
                        raise ValueError(f"{key} must be true or false")
                    new[key] = val
                else:
                    lo, hi = {"delay_ms": (0, 800), "noise_cm": (0, 8), "max_speed": (0.15, 1)}[key]
                    new[key] = number(val, key, lo, hi)
            if new["max_speed"] != self.settings["max_speed"]:
                self.limits = replace(self.limits, vx=new["max_speed"])
                if self.kick_side:
                    self.limits = replace(self.limits, vy=new["max_speed"])
                self.sim.limits = self.limits
                self.controller.limits = self.limits
                if self.kick_side is None:
                    self.controller.cancel()
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
                self.controller.cancel()  # Teleport invalidates transient progress, not referee state.
        elif action == "nudge_ball":
            self.sim.nudge_ball(number(request.get("vx", 0.0), "vx", -2, 2),
                                number(request.get("vy", 0.8), "vy", -2, 2))
        elif action == "fall":
            value = request.get("value")
            if not isinstance(value, bool):
                raise ValueError("value must be true or false")
            self.sim.set_fallen(value)
        elif action == "external_hold":
            held = request.get("held")
            if not isinstance(held, bool):
                raise ValueError("held must be true or false")
            self.executor.set_external_hold(ROBOT_ID, held, self.sim.time,
                                            "simulated front-camera hold")
            self._set_decision(self._control_tick())
        else:
            raise ValueError("Unknown control action")
        # Truth/settings preview operator edits immediately. Observation remains
        # the last control input; paused UI actions must not consume sensor/RNG
        # state or pair an old decision with a newly sampled packet.
        if action not in ("reset", "step", "stop"):
            self._record()
        return self.snapshot()
