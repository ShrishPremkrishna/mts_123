"""Single-robot kick ownership. The caller serializes ticks and backend receipts.

A request is an edge, never a walking velocity or an instruction to queue for
later. A backend must bind it to one job for this exact request, start before
its original deadline, and report that job's evidence. This module does no I/O.
A confirmed clip and walker restoration do not establish ball contact.
"""

from dataclasses import dataclass, replace
import math

from .model import Decision

# The combined runner rechecks kick authority on its 50 Hz output boundary.
# A job whose authority ends before the next check would be revoked at once.
KICK_CONTROL_PERIOD_S = 0.020


@dataclass(frozen=True)
class KickRequest:
    robot_id: str
    epoch: str
    attempt_id: str | int
    side: str
    issued_at: float
    start_before: float


@dataclass(frozen=True)
class KickStep:
    decision: Decision
    request: KickRequest | None = None
    cancel: KickRequest | None = None


class KickHandoff:
    """Arbitrate walking and a single nonblocking kick job for one robot.

    A new attack uses a distinct attempt ID in its epoch; lease renewals reuse
    the ID. Keep this object alive across Stop/re-arm until a previously active
    policy has acknowledged returning the legs. Transport loss cannot establish
    release. A job exceeding its budget requests cancellation and keeps holding.
    """

    def __init__(self, robot_id: str, *, job_timeout_s: float = 15.0):
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError("robot_id must be a nonempty string")
        if not math.isfinite(job_timeout_s) or job_timeout_s <= 0:
            raise ValueError("job_timeout_s must be finite and positive")
        self.robot_id = robot_id
        self.job_timeout_s = job_timeout_s
        self.active: KickRequest | None = None
        self._terminal: dict[tuple[str, str | int], str] = {}
        self.clip_started = False
        self._clip_seen = False
        self._ending: str | None = None
        self._cancel_sent = False
        self._restored = False
        self._authority_until = -math.inf
        # (request, outcome) for the most recently retired job. Outcome is
        # "kick_complete" only for a confirmed clip with walker restoration.
        self.last_retired: tuple[KickRequest, str] | None = None

    @property
    def clip_confirmed(self):
        return self.active is not None and self._clip_seen

    def receipt(self, request: KickRequest, phase: str, *, clip: str | None = None,
                walker_active: bool = False) -> bool:
        """Accept evidence for the current job only. False leaves ownership intact.

        'clip_confirmed' means the backend verified the requested active clip
        through the agreed exit frame. 'walker_restored' means the backend
        confirmed this terminal job returned exclusive ownership to the walker.
        A restoration received before clip evidence proves only that walking
        may resume. The kick is retired as unverified and cannot later pass.
        """
        if self.active is None or request != self.active or self._restored:
            return False
        if phase in ("clip_started", "clip_confirmed"):
            if self._ending or clip != f"kick_{request.side}":
                return False
            self.clip_started = True
            if phase == "clip_confirmed":
                self._clip_seen = True
            return True
        if phase == "failed":
            self._ending = self._ending or "kick_failed"
            return True
        if phase != "walker_restored" or walker_active is not True:
            return False
        if not self._clip_seen:
            self._ending = self._ending or "kick_failed"
        # Reconcile the receipt against the last decision's authority on the
        # next tick. A renewal after a control-loop gap cannot certify success
        # during the unobserved interval.
        self._restored = True
        return True

    def step(self, decision: Decision, now: float, *, epoch: str | None,
             attempt_id: str | int | None, side: str | None, permitted: bool) -> KickStep:
        """Consume an already authority-checked control decision.

        The caller grants 'permitted' only for a current offensive attempt,
        with fresh field and intent evidence and no veto. Other modes can walk
        normally once a pending kick returns the legs. The decision's original
        velocity expiry remains the outer authority bound for dispatch.
        """
        command = decision.command
        if command.robot_id != self.robot_id:
            raise ValueError("kick handoff received a decision for a different robot")
        has_id = isinstance(epoch, str) and bool(epoch) and (
            type(attempt_id) is int or isinstance(attempt_id, str) and bool(attempt_id))
        authorized = (permitted is True and has_id and side in ("left", "right")
                      and math.isfinite(now) and math.isfinite(command.expires_at)
                      and 0 <= now < command.expires_at)
        stopped = replace(command, vx=0.0, vy=0.0, wz=0.0)
        key = (epoch, attempt_id)

        if self.active is not None:
            current = self.active
            same = (epoch, attempt_id, side) == (current.epoch, current.attempt_id, current.side)
            if not authorized or not same or now < current.issued_at or now >= self._authority_until:
                self._ending = self._ending or "kick_cancelled"
            elif now - current.issued_at >= self.job_timeout_s:
                self._ending = self._ending or "kick_failed"
            else:
                self._authority_until = command.expires_at
            if self._restored:
                outcome = self._ending or "kick_complete"
                self._terminal[current.epoch, current.attempt_id] = outcome
                self.last_retired = (current, outcome)
                self.active = None
            else:
                cancel = None
                if self._ending and not self._cancel_sent:
                    self._cancel_sent = True
                    cancel = current
                # Preserve executor veto reasons, including Stop and falls.
                if decision.phase in ("stopped", "external_hold", "obstacle_hold", "tracking_lost",
                                      "intent_lost", "recovery_required", "participant_hold",
                                      "tactical_hold", "blocked", "invalid_observation", "invalid_intent"):
                    return KickStep(replace(decision, command=stopped), cancel=cancel)
                phase = "kick_cancelling" if self._ending else "kick_in_flight"
                return KickStep(replace(decision, command=stopped, phase=phase,
                                        reason="awaiting the kick backend's walker receipt"), cancel=cancel)

        if authorized and key in self._terminal:
            return KickStep(replace(decision, command=stopped, phase=self._terminal[key],
                                    reason="this kick attempt is retired"))
        if decision.phase != "kick_ready":
            return KickStep(decision)
        if not authorized or any((command.vx, command.vy, command.wz)):
            return KickStep(replace(decision, command=stopped, phase="kick_hold",
                                    reason="kick requires a current stationary authorized decision"))
        if command.expires_at <= now + KICK_CONTROL_PERIOD_S:
            return KickStep(replace(decision, command=stopped, phase="kick_hold",
                                    reason="waiting for evidence that lasts past the next kick check"))
        self.active = KickRequest(self.robot_id, epoch, attempt_id, side, now, command.expires_at)
        self.clip_started = False
        self._clip_seen = False
        self._ending = None
        self._cancel_sent = False
        self._restored = False
        self._authority_until = command.expires_at
        return KickStep(replace(decision, command=stopped, phase="kick_in_flight",
                                reason="kick requested; waiting for matching clip and walker receipts"),
                        request=self.active)
