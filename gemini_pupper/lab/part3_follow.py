"""Part 3: Follow. TODO 5 (the rule that ends follow mode), then TODO 6 (the follow tool).

The person follower itself is provided (robot.set_following). The rule is yours:

    Questions and conversation keep following. Any motion (walk, turn, arc, trick,
    stand up, lie down) ends follow mode, and Pupper says so first:
    "Let me stop following you and ..."
"""

from __future__ import annotations

from robot_api import Result, Robot, not_written  # noqa: F401
from tool_registry import Param, tool


# ============================================================ what the robot does
async def prepare_to_move(robot: Robot) -> str | Result:
    """Get the robot ready for a new motion. Every motion calls this first,
    before it queues anything: look for it in stand_up (basics.py) and walk
    (part2_walk.py).

    What it is for: stopping anything that runs on its own, outside the queue,
    and would fight with the new motion. In Gemini Pupper that is the person
    follower, so its one job is to end follow mode; it is also where anything
    like it you add later goes (a patrol loop, dancing to music). Getting the
    legs ready is a different job, done later in the job by switch_to.

    TODO 5 (Part 3) ------------------------------------------------------------
    The rule:            talking and questions keep following. Any motion (walk,
                         turn, trick, stand up, lie down) ends follow mode right
                         away, not when the queue gets to the motion.
    Returns:             a note that goes in front of the tool's reply, e.g.
                         "Stopped following the person first.", or "" if the
                         robot wasn't following.
    Why right away:      the person asked for something else *now*; the robot
                         shouldn't keep chasing them until the queue gets there.
    Why a note:          Gemini can only say "Let me stop following you and ..."
                         if the reply tells it.
    Use:                 robot.following, await robot.stop_following(),
                         robot.send_speed(0, 0, 0) (stand still)
    Tests check:         - stand_up, lie_down and walk stop the follower right
                           away, before the queue gets to them
                         - their reply contains the word "follow".
                           Passes: "Stopped following the person first. activate walking queued; ..."
                           Fails:  "Done. activate walking queued; ..."
                         - when not following, the reply does NOT contain "follow"
                         - robot_status (a question) keeps following
    Test:                python3 lab/test.py todo5
    """
    # ========== TODO 5: YOUR CODE HERE ==========
    # The shared handoff handles both queued follow intent and active following.
    # A superseded entry can finish its owned exit before the motion executes;
    # a submitted follow activation must be confirmed off before admission.
    # Pending FIFO entry is superseded without cancelling earlier ordinary
    # jobs. Only confirmed active-follow shutdown warrants the spoken note.
    result = await robot.prepare_follow_handoff()
    if not result.ok:
        return result
    return "Stopped following the person first." if result.details["following"]["changed"] else ""
    # ====================================


# ============================================================ what Gemini reads
# TODO 6: describe follow for Gemini, and write its body: robot.set_following(enabled)
# (provided) starts or stops the person follower. Gemini should know what keeps
# follow mode on and what ends it (your rule in TODO 5).
#   Tests check:
#   - follow's description is 20+ characters, enabled is described, and the
#     description says what ends it: it contains "end" or "stop".
#     Passes: "... any motion tool (walk, turn, ...) ends it."  Fails: "Follow the person."
#   - prompt.md's Follow mode section contains "end" or "stop"
#   - follow(enabled=True) starts the follower, follow(enabled=False) stops it
#   Test: python3 lab/test.py todo6
# ========== TODO 6: YOUR CODE HERE ==========
@tool(
    "follow",
    "Queue following the nearest person using the camera after earlier commands, or stop following immediately."
    " Queued is acceptance, not active following. Questions, wait and conversation keep follow mode on;"
    " A later takeover tool (move, turn, arc, play_trick, stand_up, lie_down or shake_hand)"
    " cancels pending follow entry or ends active following, preserving earlier queued commands."
    " Requires locomotion; stand_up first to leave a walking trick such as the spider."
    " Use only for a requested following behavior, not find me, come to me, approach an object,"
    " or search the room. This does not authenticate identity or provide collision avoidance;"
    " never replace unavailable following with move/turn/arc calls.",
    enabled=Param("boolean", "True queues following after earlier commands; false cancels pending following, stops active following and requests zero speed."),
)
async def follow_tool(robot: Robot, enabled: bool) -> Result:
    # One line: robot.set_following(enabled) (provided) starts or stops following, and
    # returns the Result Gemini should hear. Await it and return it.
    return await robot.set_following(enabled)
# ====================================
