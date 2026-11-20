"""Consume /operator/kick_in_place acknowledgements and /queue/status locally.

The caller owns the trusted, robot-specific endpoint and correlates each POST
response with its request. No requests are sent here. Starting before expiry
and cancelling a physical job require a separately qualified executor.
"""
import re

from .kick_handoff import KickHandoff, KickRequest


class OperatorKickReceipts:
    """Keep one instance-bound queue binding per active robot handoff."""

    def __init__(self, handoff: KickHandoff, server_instance: str):
        if not isinstance(server_instance, str) or not re.fullmatch("[a-f0-9]{32}", server_instance):
            raise ValueError("expected a verified Gemini server instance")
        self.handoff = handoff
        self.server_instance = server_instance
        self._largest_job_number = 0
        self._request = None
        self._job_id = None
        self._observed = False

    def bind(self, request: KickRequest, response: dict) -> bool:
        """Bind the direct submission acknowledgement once; it proves no motion."""
        if request is None or request != self.handoff.active or not isinstance(response, dict):
            return False
        details = response.get("details")
        row = details.get("submission") if isinstance(details, dict) else None
        if (response.get("success") is not True or not isinstance(row, dict)
                or row.get("schema") != "pupper_submission_v1" or row.get("state") != "queued"
                or row.get("name") != f"kick_in_place {request.side}"
                or row.get("server_instance") != self.server_instance):
            return False
        job_id = row.get("job_id")
        if not isinstance(job_id, str):
            return False
        match = re.fullmatch(self.server_instance + r":([1-9][0-9]*)", job_id)
        if match is None:
            return False
        if self._request == request:
            return job_id == self._job_id
        try:
            job_number = int(match.group(1))
        except ValueError:
            return False
        if job_number <= self._largest_job_number:
            return False
        self._request, self._job_id = request, job_id
        self._largest_job_number = job_number
        self._observed = False
        return True

    def observe(self, status: dict) -> bool:
        """Return True for consumption of a terminal report, including failure.

        The handoff decision communicates success, cancellation or unresolved
        walker ownership. Missing history, unsupported evidence and replacement
        servers cannot grant walking authority.
        """
        request = self._request
        if self._observed or request is None or request != self.handoff.active or not isinstance(status, dict):
            return False
        session = status.get("session_context")
        history = status.get("recent_outcomes")
        if (not isinstance(session, dict) or session.get("schema") != "pupper_session_context_v1"
                or session.get("scope") != "this_server_session" or session.get("server_instance") != self.server_instance
                or not isinstance(history, dict) or not isinstance(history.get("outcomes"), list)):
            return False
        rows = [row for row in history["outcomes"] if isinstance(row, dict) and row.get("job_id") == self._job_id]
        if len(rows) != 1:
            return False
        row = rows[0]
        if (row.get("name") != f"kick_in_place {request.side}"
                or row.get("server_instance") != self.server_instance or row.get("execution_started") is not True):
            return False
        evidence = row.get("operator_kick")
        verified_walker = (isinstance(evidence, dict) and evidence.get("schema") == "pupper_operator_kick_result_v1"
                           and evidence.get("side") == request.side and evidence.get("restored_controller") == "gemini_walk"
                           and "confirmed_clip" in evidence and evidence["confirmed_clip"] in (None, f"kick_{request.side}"))
        terminal = row.get("terminal_state")
        if terminal == "succeeded" and row.get("success") is True:
            if not verified_walker or evidence.get("confirmed_clip") != f"kick_{request.side}":
                return False
            # A veto may have retired this attempt while its true result was in
            # flight. Walker restoration still releases the legs without scoring success.
            self.handoff.receipt(request, "clip_confirmed", clip=evidence["confirmed_clip"])
        elif terminal in ("failed", "cancelled") and row.get("success") is False:
            if not self.handoff.receipt(request, "failed"):
                return False
            if not verified_walker:
                self._observed = True
                return True  # Failure observed; exclusive ownership is still unresolved.
        else:
            return False
        self._observed = self.handoff.receipt(request, "walker_restored", walker_active=True)
        return self._observed
