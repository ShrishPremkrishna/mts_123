"""Single-scheduler FC field execution on a preselected Gemini walk/kick release.

The owner supplies a same-host, guarded deadline publisher and live graph/guard/
controller evidence. Only the chosen Gemini process may own the native switch
client. No HTTP command server is started. All methods run on its asyncio loop.
"""
from __future__ import annotations

import asyncio
from dataclasses import replace
import math

from final_project.pupper_fc.kick_handoff import KickHandoff, KickStep
from final_project.pupper_fc.model import Decision, zero
from final_project.pupper_fc.operator_kick_receipts import OperatorKickReceipts

from operator_kick import kick_in_place
from motion_contract import CALL_STOPS, LOCOMOTION_CONTROLLER


class CombinedRuntime:
    def __init__(self, robot, executor, robot_id, *, velocity_sink, path_problem, walker_ready=None,
                 hard_stop=None):
        if robot_id not in executor.controllers or executor.kick_side not in ("left", "right"):
            raise ValueError("field executor must select a robot and a kicking foot")
        self.robot = robot
        self.executor = executor
        self.robot_id = robot_id
        self.velocity_sink = velocity_sink
        self.path_problem = path_problem
        self.walker_ready = walker_ready
        self.hard_stop = hard_stop
        self.handoff = KickHandoff(robot_id)
        self.receipts = OperatorKickReceipts(self.handoff, robot.command_instance)
        self.opened = False
        self.stop_reason = None
        self._generation = None
        self._submission = None
        self._cleanup = None
        self._authorized_until = -math.inf
        self._bound = False

    async def open(self):
        """Check the existing walker and loaded kick, without any activation."""
        if self.opened or self.stop_reason:
            raise RuntimeError("combined runtime has already opened or stopped")
        r = self.robot
        await r.ready.wait()
        if r.follow_available or r.following or set(r.tricks) != {"leg_kick"}:
            raise RuntimeError("combined mode requires only leg_kick and the follower disabled")
        kick = r.tricks["leg_kick"]
        if not kick.operator_only or not r.trick_available(kick):
            raise RuntimeError("selected operator-only leg kick is unavailable")
        generation = r.stops
        while True:
            states = await r.controller_states()
            active = {name for name in r.leg_controllers if states.get(name) == "active"}
            if active != {LOCOMOTION_CONTROLLER} or states.get(kick.controller) != "inactive":
                raise RuntimeError("requires sole active gemini_walk and loaded inactive kick")
            if r.current is not None or not r.commands.empty():
                raise RuntimeError("another job owns the robot")
            if generation != r.stops or r.fallen:
                raise RuntimeError("robot reported a Stop or fall during startup")
            waited = await r.wait_for_initial_stop_observation()
            if generation != r.stops or r.fallen:
                raise RuntimeError("robot reported a Stop or fall during startup")
            reason = r.motion_block_reason()
            if reason:
                raise RuntimeError("robot has a motion veto: " + reason)
            if not waited:
                break
            # Recheck ownership if a controller could have switched during DDS discovery.
        r.claim_observed_session(generation)
        self._generation = generation
        self.opened = True

    def _gate(self, job, *, entry):
        now = self.robot.now()
        if (self.stop_reason or self.robot.stops != self._generation or self.robot.fallen
                or self.handoff.active != job or now >= self._authorized_until
                or (entry and not job.issued_at <= now < job.start_before)
                or not self.executor.kick_authorized(self.handoff, now, require_ready=entry)):
            return "field kick authority expired or was revoked before controller activation"
        return self.path_problem(not entry)

    async def _submit(self, job):
        reason = self._gate(job, entry=True)
        if reason:
            raise RuntimeError(reason)
        if self.robot.current is not None or not self.robot.commands.empty():
            raise RuntimeError("kick cannot wait behind another robot job")
        token = CALL_STOPS.set(self._generation)
        try:
            result = await kick_in_place(self.robot, job.side,
                                         entry_gate=lambda: self._gate(job, entry=True),
                                         return_gate=lambda: self._gate(job, entry=False),
                                         on_entry=lambda clip: self.handoff.receipt(
                                             job, "clip_started", clip=clip),
                                         on_clip=lambda clip: self.handoff.receipt(
                                             job, "clip_confirmed", clip=clip))
        finally:
            CALL_STOPS.reset(token)
        if not result.ok or not self.receipts.bind(job, result.as_dict()):
            raise RuntimeError("kick submission unconfirmed: " + result.message)
        self._bound = True

    def _observe_job(self):
        if self._submission is not None and self._submission.done():
            task, self._submission = self._submission, None
            if not task.cancelled():
                try:
                    task.result()
                except Exception as exc:
                    self.stop(str(exc))
        if self._bound:
            status = {"session_context": self.robot.status()["session_context"],
                      "recent_outcomes": self.robot.recent_outcomes()}
            try:
                if self.receipts.observe(status, walker_ready=self.walker_ready):
                    self._bound = False
            except RuntimeError as exc:
                self.stop(str(exc))

    def stop(self, reason="operator Stop"):
        """Revoke synchronously; cancel and reconcile native ownership asynchronously."""
        if self.stop_reason:
            return
        self.stop_reason = str(reason)
        self.executor.stop()
        if self.opened and self.hard_stop is not None:
            # Fence queued gateway activations before waiting on the local job.
            self.hard_stop(self.stop_reason)
        if self._submission is not None and not self._submission.done():
            self._submission.cancel()
        # Start cancellation even when the output transport has become unavailable.
        if self.opened and self.handoff.active is not None:
            self._cleanup = asyncio.create_task(self.robot.immediate_stop(relax_playing_trick=True))
        if self.opened:
            self.velocity_sink(zero(self.robot.now(), self.robot_id))

    def tick(self, now=None, *, manual_command=None):
        """Publish one bounded command: the executor's decision, or an operator drive command.

        manual_command is an expiring teleop command for this robot. It may not
        overlap an armed executor or a kick; overlap is treated as a fault.
        """
        if not self.opened:
            raise RuntimeError("combined runtime must be opened against the selected controllers")
        self._observe_job()
        if self.stop_reason:
            self.executor.stop()
        elif self.robot.stops != self._generation or self.robot.fallen or self.robot.motion_block_reason():
            self.stop("robot reported Stop, a fall, or a motion fault")
        elif manual_command is not None and (self.executor.armed or self.handoff.active is not None
                                             or manual_command.robot_id != self.robot_id):
            self.stop("manual drive conflicts with field control, kick, or robot identity")
        elif self.executor.armed or manual_command is not None:
            problem = self.path_problem(self.handoff.active is not None)
            if problem:
                self.stop(problem)
            elif self.handoff.active is None and (self.robot.current is not None or not self.robot.commands.empty()):
                self.stop("unexpected competing job")
        if now is None:
            now = self.robot.now()  # A camera hold applied above has already advanced the field clock.
        if manual_command is None:
            step = self.executor.kick_step(self.handoff, now)
        else:
            if now >= manual_command.expires_at or self.executor.external_holds[self.robot_id]:
                manual_command = zero(now, self.robot_id)
            step = KickStep(Decision("teleop", "operator drive", manual_command))
        if step.cancel is not None:
            self.stop("in-flight kick revoked")
        command = step.decision.command
        if any((command.vx, command.vy, command.wz)):
            problem = self.path_problem(False)
            if problem:
                self.stop(problem)
            elif self.executor.external_holds[self.robot_id] and self.handoff.active is None:
                step = self._held(step, self.executor.external_hold_reasons[self.robot_id], now)
            elif self.robot.now() >= command.expires_at:
                if manual_command is None:
                    self.stop("walking command expired before output")
                else:
                    step = replace(step, decision=replace(step.decision, command=zero(now, self.robot_id)))
        if self.stop_reason:
            step = replace(step, request=None, decision=replace(step.decision, phase="stopped",
                           reason=self.stop_reason, command=zero(now, self.robot_id)))
        self._authorized_until = step.decision.command.expires_at
        try:
            hold = self.velocity_sink(step.decision.command)
        except Exception:
            self.stop("guarded velocity publication failed")
            raise
        if hold:
            # The sink found camera evidence expired at the send boundary and
            # already published zero. The walker stays available; no kick starts.
            step = self._held(step, hold, now)
        if step.request is not None:
            self._submission = asyncio.create_task(self._submit(step.request))
        return step

    def _held(self, step, reason, now):
        return replace(step, request=None, decision=replace(step.decision, phase="external_hold",
                       reason=reason, command=zero(now, self.robot_id)))

    async def close(self):
        if not self.opened:
            return
        try:
            self.stop("combined runtime closed")
        finally:
            if self._submission is not None:
                await asyncio.gather(self._submission, return_exceptions=True)
            if self._cleanup is not None:
                result = await self._cleanup
                if not result.ok:
                    raise RuntimeError(result.message)
            self.opened = False
