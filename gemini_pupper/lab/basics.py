"""Basics: read this file first. Everything here is finished.

Every lab/ file has the same two halves:

  WHAT THE ROBOT DOES (a behavior): is the request allowed, do it, report what
  happened. Every motion has the same shape:

      async def behavior(robot, ...):
          (1) NOW: check what can be checked right away; refuse with a reason
          async def job():
              (2) LATER, when the queue gets here: check again, act, report the outcome
          (3) return await robot.queue(name, job, seconds, then=state)   # what Gemini hears now

      then= is the state Pupper is in once the job is done: LOCOMOTION,
      MOTORS_IDLE, or a trick. It becomes robot.state_after_queue, so a request
      made now knows the state it will start from.

  WHAT GEMINI READS (a tool): an @tool(...) line with the tool's name, a
  description, and a description of each parameter. Gemini decides when to
  call a tool, and what to pass, from these words alone. The function under it
  turns Gemini's words into numbers and calls a behavior. Whatever it returns
  (a Result) goes back to Gemini: it is how Gemini gets feedback on what happened.

Why a queue? Gemini often asks for several things in one breath ("sit, then
shake"). The queue runs them one at a time, in order, while Gemini keeps
talking. So a tool replies as soon as its job is *accepted*; the job's
*outcome* (done, fell over, controller failed) goes to the debug log and to
robot_status.

The Robot API (robot.queue, robot.switch_to, ...) is listed at the top of
robot/robot_api.py. Directions use the robot's convention: positive y is LEFT,
and a positive turn is to the LEFT (counter-clockwise).

After you change a lab/ file: python3 lab/test.py, try it in text with
python3 lab/chat.py, then restart (robot/stop.sh && robot/start.sh).
"""

from __future__ import annotations

from part3_follow import prepare_to_move
from robot_api import LOCOMOTION, LOCOMOTION_CONTROLLER, MOTORS_IDLE, WALK_READY_S, Result, Robot, with_note
from tool_registry import Param, tool

# Added to the description of every tool that calls prepare_to_move, which ends follow mode (Part 3).
PREPARES_MOVE = (
    " If follow mode is on, this ends it: first tell the user,"
    ' e.g. "Let me stop following you and ...".'
)


# ============================================================ what the robot does
async def stand_up(robot: Robot) -> Result:
    """Stand up in locomotion mode (also: stand up after a trick)."""
    note = await prepare_to_move(robot)  # (1) now: get ready (does nothing until your TODO 5, Part 3)
    if isinstance(note, Result):
        return note

    async def job() -> Result:  # (2) later: give the legs to the locomotion policy
        return await robot.switch_to(LOCOMOTION_CONTROLLER)

    return with_note(note, await robot.queue("stand up", job, WALK_READY_S, then=LOCOMOTION))  # (3)


async def lie_down(robot: Robot) -> Result:
    """Relax every motor, so the robot lies down."""
    robot.secret_gate.lock()
    note = await prepare_to_move(robot)
    if isinstance(note, Result):
        return note

    async def job() -> Result:
        return await robot.switch_to(None)  # None: no controller holds the legs

    return with_note(note, await robot.queue("lie down", job, 1.0, then=MOTORS_IDLE))


# ============================================================ what Gemini reads
@tool("stand_up", "Queue a transition to the locomotion controller for an explicit stand-up request or supervised fall recovery. "
      "Move, turn and arc already prepare their controller and exit an active trick when needed; "
      "do not add stand_up before or after another action unless requested." + PREPARES_MOVE)
async def stand_up_tool(robot: Robot) -> Result:
    return await stand_up(robot)


@tool(
    "lie_down",
    "Queue leg-controller deactivation when the user explicitly asks to sleep, relax"
    " or lie down. Controller deactivation does not verify physical posture." + PREPARES_MOVE,
)
async def lie_down_tool(robot: Robot) -> Result:
    return await lie_down(robot)


@tool(
    "stop",
    "Cancel queued actions, end following and request zero walking speed immediately."
    " Interrupt a playing trick by deactivating its controller. On a later explicit"
    " movement request, move/turn/arc prepare their controller automatically if readiness"
    " checks pass; a clean Stop does not require a separate stand_up. A reported fall"
    " still requires supervised recovery. The reply confirms commands, not physical"
    " stillness or posture. This is an interrupt, never a delayed final step."
    " Only report cancellation of a named action when a tool/status result confirms"
    " it; never invent cancellation of an unsubmitted action. Cleared queued-command"
    " counts do not describe running-job cancellation; check robot_status for that.",
)
async def stop_tool(robot: Robot) -> Result:
    return await robot.immediate_stop(relax_playing_trick=True)


@tool(
    "wait",
    "Queue a pause after earlier queued actions, keeping the current mode."
    " The reply means accepted, not that time has elapsed; use robot_status for"
    " completion. This does not stop following, relax motors or prove physical"
    " stillness. Stop interrupts this pause and is never delayed by it.",
    duration=Param("number", "Pause duration in seconds, greater than 0 and at most 60."),
)
async def wait_tool(robot: Robot, duration: float) -> Result:
    return await robot.wait(duration)


@tool("robot_status", "Check active walking/trick controllers, following, queued work and detected falls. Controller status does not measure physical stopping or posture.")
async def robot_status_tool(robot: Robot) -> Result:
    return await robot.mode()
