"""Optional Lab 3: code-enforced unlock, with no movement on unlock itself."""

from robot_api import Result, Robot
from tool_registry import Param, tool


@tool("unlock", "Check the password the person just supplied, without moving. If accepted, hidden tricks become"
      " available until stop, lie down or restart. Never guess passwords or claim this authenticates a person.",
      password=Param("string", "The password the person supplied for this request; never invent or echo it."))
async def unlock_tool(robot: Robot, password: str) -> Result:
    if not robot.secret_gate.unlock(password):
        return Result(False, "Unlock refused. Ask the owner for the password; do not guess or repeat it.")
    available = "; ".join(f"{name}: {t.description}" for name, t in robot.available_tricks().items())
    actions = "; ".join(f"{aid} (in {t.name}): {t.actions[clip]}" for aid, (t, clip) in robot.action_ids().items())
    return Result(True, f"Unlocked for this robot session. Nothing moved. Available tricks: {available}."
                       f" Interactive actions: {actions or 'none'}. Play the matching trick before an action.")


@tool("lock_tricks", "Lock hidden tricks again without moving; pending hidden actions will be refused."
      " If a trick is already holding a pose, use stand_up to leave it or stop to halt movement.")
async def lock_tricks_tool(robot: Robot) -> Result:
    robot.secret_gate.lock()
    return Result(True, "Hidden tricks locked. This did not move the robot.")
