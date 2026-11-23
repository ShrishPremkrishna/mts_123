"""Pure queued-motion result and authority contract shared by live and replay adapters.

The robot transport imports these same ContextVar instances. This module performs
no ROS import, initialization or I/O; it does not grant motion authority itself.
"""
from __future__ import annotations

import contextvars
from dataclasses import dataclass
from typing import Callable, TYPE_CHECKING

from tricks import WALK_CONTROLLER

if TYPE_CHECKING:
    from robot_api import Robot

LOCOMOTION_CONTROLLER = WALK_CONTROLLER
LOCOMOTION = "locomotion"

FELL_MESSAGE = (
    "a tilt/fall stop was reported; physical posture and motor torque have not been measured. "
    "A person should check and set the robot upright on its feet; then call stand_up when ready"
)


@dataclass
class Result:
    """What a tool tells Gemini: did it work, and what happened (or why not)."""

    ok: bool
    message: str
    details: dict | None = None

    def as_dict(self) -> dict:
        result = {"success": self.ok, "message": self.message}
        if self.details is not None:
            result["details"] = self.details
        return result


async def prepare_motion_handoff(robot: Robot) -> str | Result:
    """Release any pending follower ownership before a new motion is queued."""
    result = await robot.prepare_follow_handoff()
    if not result.ok:
        return result
    return "Stopped following the person first." if result.details["following"]["changed"] else ""


def with_note(note: str, res: Result) -> Result:
    """``res`` with ``note`` (e.g. "Stopped following the person first.") in front."""
    return Result(res.ok, f"{note.strip()} {res.message}", res.details) if note.strip() else res


# A queued request remains bound to the Stop generation of its original call.
CALL_STOPS: contextvars.ContextVar[int | None] = contextvars.ContextVar("call_stops", default=None)

# An in-process FC kick can lose its field authority while ROS service
# discovery or parameter writes yield. Recheck at the native submission site.
ACTIVATION_GATE: contextvars.ContextVar[Callable[[], str | None] | None] = contextvars.ContextVar(
    "activation_gate", default=None)
