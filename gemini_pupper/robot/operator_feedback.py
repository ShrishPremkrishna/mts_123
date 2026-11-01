"""Closed receipt wording plus best-effort English screening of tool-free chat."""
import json
import math
from pathlib import Path
import re
import unicodedata

CONVERSATION_GUARD = json.loads((Path(__file__).resolve().parent.parent / "conversation_guard.json").read_text())
_CONVERSATION_PATTERNS = tuple(re.compile(pattern, re.IGNORECASE) for pattern in CONVERSATION_GUARD["patterns"])
CONVERSATION_BLOCKED = "No confirmed robot action in this reply; check server status."


def normalize_conversation_text(text):
    return " ".join(unicodedata.normalize("NFKC", text).replace("’", "'").replace("‘", "'").split())


def conversation_claim_detected(text):
    """A conservative English heuristic, never a universal semantic guarantee."""
    normalized = normalize_conversation_text(text)
    return any(pattern.search(normalized) for pattern in _CONVERSATION_PATTERNS)


LABELS = {
    "stand_up": "stand", "lie_down": "leg relaxation", "move": "move",
    "turn": "turn", "arc": "arc", "wait": "wait", "stop": "Stop",
    "robot_status": "status", "play_trick": "trick",
    "interactive_action": "interactive action", "follow": "follow",
    "unlock": "unlock", "lock_tricks": "lock",
}
QUEUE_TOOLS = frozenset({"stand_up", "lie_down", "move", "turn", "arc", "wait",
                         "play_trick", "interactive_action"})


def tool_label(name):
    return LABELS.get(name, "robot") if isinstance(name, str) else "robot"


def sending_feedback(name):
    return f"Sending {tool_label(name)} request."



def _number(value):
    if type(value) not in (int, float) or not 0 <= value <= 1_000_000 or not math.isfinite(value) or 0 < value < 0.000001:
        return None
    return str(int(value)) if value == int(value) else str(value)


def _motion_reason(details):
    reason = details.get("refusal") if isinstance(details, dict) else None
    if not isinstance(reason, dict) or reason.get("schema") != "pupper_motion_refusal_v1":
        return None
    limits = reason.get("limits")
    if not isinstance(limits, dict):
        return None
    code = reason.get("code")
    fields = {
        "speed_limit": ("forward_max", "backward_max", "sideways_max", "turn_max"),
        "minimum_speed": ("forward_min", "sideways_min", "turn_min"),
        "duration_limit": ("duration_max",), "distance_limit": ("distance_max",), "radius_limit": ("radius_min",),
        "invalid_values": (),
    }
    if not isinstance(code, str) or code not in fields:
        return None
    numbers = [_number(limits.get(key)) for key in fields[code]]
    if any(n is None for n in numbers):
        return None
    if code == "speed_limit":
        return f"speed exceeds the limit: {numbers[0]} m/s forward, {numbers[1]} backward, {numbers[2]} sideways, {numbers[3]} deg/s turning."
    if code == "minimum_speed":
        return f"speed below minimum. Use at least {numbers[0]} m/s forward/backward, {numbers[1]} sideways or {numbers[2]} deg/s turning."
    if code == "duration_limit":
        return f"duration must be above zero and at most {numbers[0]} seconds."
    if code == "distance_limit":
        return f"commanded distance exceeds {numbers[0]} metres."
    if code == "radius_limit":
        return f"arc radius must be at least {numbers[0]} metres at that speed."
    return "motion values must be finite numbers."


def _status_feedback(details):
    if not isinstance(details, dict) or type(details.get("running")) is not bool:
        return None
    queued = details.get("queued")
    if _number(queued) is None or queued != int(queued):
        return None
    state = details.get("state")
    modes = {"locomotion": "locomotion", "motors idle": "motors idle", "fallen": "fall stop latched", "locked trick": "locked skill", "sit_stay_shake": "Sit controller selected", "secret_dance": "generated-motion controller selected"}
    mode = modes.get(state) if isinstance(state, str) else None
    observed_modes = {"locomotion": "locomotion", "none": "no active command controller", "unknown": "unknown", "sit_stay_shake": "Sit", "secret_dance": "generated motion", "other_skill": "skill", "walking_skill": "walking skill", "locked_skill": "locked skill"}
    observed = details.get("observed_controller_mode")
    observation = observed_modes.get(observed) if isinstance(observed, str) else None
    parts = ([f"Controller: {observation}."] if observation else
             [f"Cached mode: {mode}." if mode else "Cached status:", "Controller unverified."])
    current = details.get("current_action")
    if details["running"]:
        name = current.get("name") if isinstance(current, dict) and current.get("state") == "running" else None
        kind = "command"
        if isinstance(name, str):
            if name.startswith("move "): kind = "movement command"
            elif name.startswith("wait "): kind = "wait command"
            elif name.startswith("trick ") or name == "locked trick": kind = "skill command"
            elif name == "stand up": kind = "stand command"
            elif name == "lie down": kind = "leg-relaxation command"
        parts.append(f"Running a {kind}; {int(queued)} queued.")
    elif current is not None:
        parts.append(f"Current action unavailable; {int(queued)} queued.")
    else:
        parts.append(f"No command running; {int(queued)} queued.")
    if type(details.get("following")) is bool:
        parts.append("Following on." if details["following"] else "Following off.")
    if isinstance(details.get("controller_fault"), str) and details["controller_fault"]:
        parts.append("Controller fault reported.")
    context = details.get("session_context")
    if (isinstance(context, dict) and context.get("schema") == "pupper_session_context_v1"
            and context.get("scope") == "this_server_session"
            and isinstance(context.get("server_instance"), str)
            and re.fullmatch(r"[a-f0-9]{32}", context["server_instance"])
            and _number(context.get("completed_sit_entries")) is not None
            and context["completed_sit_entries"] == int(context["completed_sit_entries"])):
        parts.append(f"Confirmed Sit-entry playbacks in this server session: {int(context['completed_sit_entries'])}.")
    return " ".join(parts)

def receipt_feedback(name, result, *, current=True, dispatched=True):
    """A matched reply is evidence of an API result, never physical completion."""
    label = tool_label(name)
    spoken = label[0].upper() + label[1:]
    if not current or (isinstance(result, dict) and result.get("superseded") is True):
        return "That request was superseded."
    if not dispatched:
        return f"{spoken} request not sent."
    if not isinstance(result, dict):
        return "I could not confirm that request."
    flags = ("superseded", "outcome_unknown", "not_dispatched")
    if any(flag in result and type(result[flag]) is not bool for flag in flags) or result.get("outcome_unknown") is True:
        return "I could not confirm that request."
    if result.get("not_dispatched") is True:
        return f"{spoken} request not sent."
    if type(result.get("success")) is not bool:
        return "I could not confirm that request."
    if not result["success"]:
        reason = _motion_reason(result.get("details")) if name in ("move", "turn", "arc") else None
        if reason:
            return f"{spoken} rejected: {reason}"
        if name not in ("stop", "robot_status") and result.get("command_epoch_required") is True:
            return f"{spoken} rejected: command authorization is stale or missing. Check status and make a new request."
        return "That request failed. Check the controls for details."
    if name == "robot_status":
        status = _status_feedback(result.get("details"))
        if status:
            return status
    details = result.get("details")
    submission = details.get("submission") if isinstance(details, dict) else None
    queued = (isinstance(name, str) and name in QUEUE_TOOLS and isinstance(submission, dict)
              and submission.get("schema") == "pupper_submission_v1"
              and submission.get("state") == "queued")
    if name == "stop":
        return "Stop requested."
    return f"{spoken} {'queued' if queued else 'request accepted'}."
