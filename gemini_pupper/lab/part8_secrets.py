"""Optional Lab 3: code-enforced unlock, with no movement on unlock itself.

Tests check (python3 lab/test.py secrets):
  Locked:   play_trick's {tricks} list, the enums and robot_status name no secret
            trick; play_trick refuses one with "This trick is locked. Ask for the
            password, then use unlock(password)." and nothing moves.
  Wrong:    unlock fails, repeats neither the guess nor the password, says not to
            guess; a sentence that only contains the password also fails.
  Right:    unlock succeeds ("Bananas." matches "bananas"), never says the password
            (if a trick name or description contains it, unlock refuses instead),
            names the secret tricks now available, and moves nothing; play_trick
            then plays them.
  Relock:   lie_down, stop, a wrong password, lock_tricks and a restart lock again.
"""

import logging

from robot_api import Result, Robot
from tool_registry import Param, tool

log = logging.getLogger("behaviors")
REFUSED = "Unlock refused. Ask the owner for the password; do not guess or repeat it."


@tool("unlock", "Check the password the person just supplied, without moving. If accepted, hidden tricks become"
      " available until stop, lie down or restart. Never guess passwords or claim this authenticates a person.",
      password=Param("string", "Only the password itself, as the person just said it, not the sentence around it;"
                     " never invent or echo it."))
async def unlock_tool(robot: Robot, password: str) -> Result:
    if not robot.secret_gate.unlock(password):
        return Result(False, REFUSED)
    available = "; ".join(f"{name}: {t.description}" for name, t in robot.available_tricks().items())
    actions = "; ".join(f"{aid} (in {t.name}): {t.actions[clip]}" for aid, (t, clip) in robot.action_ids().items())
    reply = (f"Unlocked for this robot session. No motion command was issued by unlock. Available tricks: {available}."
             f" Interactive actions: {actions or 'none'}. Play the matching trick before an action.")
    if robot.secret_gate.appears_in(reply):
        # The password is part of a trick name, a description or this text, so saying the reply
        # would say the password. Stay locked; the desk test on the robot reports this.
        robot.secret_gate.lock()
        log.warning("unlock refused: the password appears in a trick name, description or reply; choose another")
        return Result(False, REFUSED)
    return Result(True, reply)


@tool("lock_tricks", "Lock hidden tricks again without moving; pending hidden actions will be refused."
      " If a trick is already holding a pose, use stand_up to leave it or stop to halt movement.")
async def lock_tricks_tool(robot: Robot) -> Result:
    robot.secret_gate.lock()
    return Result(True, "Hidden tricks locked. No motion command was issued by lock.")
