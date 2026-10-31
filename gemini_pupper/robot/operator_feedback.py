"""Closed operator wording. Model prose and raw server messages are never facts here."""

LABELS = {
    "stand_up": "stand", "lie_down": "leg relaxation", "move": "move",
    "turn": "turn", "arc": "arc", "wait": "wait", "stop": "Stop",
    "robot_status": "status", "play_trick": "trick",
    "interactive_action": "interactive action", "follow": "follow",
    "unlock": "unlock", "lock_tricks": "lock",
}
QUEUE_TOOLS = frozenset({"stand_up", "lie_down", "move", "turn", "arc", "wait",
                         "play_trick", "interactive_action"})
CONVERSATION_TOOLS = frozenset({"robot_status", "stop"})


def tool_label(name):
    return LABELS.get(name, "robot") if isinstance(name, str) else "robot"


def sending_feedback(name):
    return f"Sending {tool_label(name)} request."


def receipt_feedback(name, result, *, current=True, dispatched=True):
    """A matched reply is evidence of an API result, never physical completion."""
    label = tool_label(name)
    if not current or (isinstance(result, dict) and result.get("superseded") is True):
        return "This request was superseded. Earlier execution is possible."
    if not dispatched:
        return f"The {label} request was not dispatched."
    if not isinstance(result, dict):
        return f"The {label} response is unknown. Check server status and the robot."
    flags = ("superseded", "outcome_unknown", "not_dispatched")
    if any(flag in result and type(result[flag]) is not bool for flag in flags) or result.get("outcome_unknown") is True:
        return f"The {label} response is unknown. Check server status and the robot."
    if result.get("not_dispatched") is True:
        return f"The {label} request was not dispatched."
    if type(result.get("success")) is not bool:
        return f"The {label} response is unknown. Check server status and the robot."
    if not result["success"]:
        return f"Server reported the {label} request failed. Earlier effects are not ruled out."
    details = result.get("details")
    submission = details.get("submission") if isinstance(details, dict) else None
    queued = (isinstance(name, str) and name in QUEUE_TOOLS and isinstance(submission, dict)
              and submission.get("schema") == "pupper_submission_v1"
              and submission.get("state") == "queued")
    text = f"Server {'queued' if queued else 'accepted'} the {label} request. Completion is unconfirmed."
    if name == "stop":
        text += " Physical stopping is unverified."
    return text
