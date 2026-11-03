"""Part 5: interactive actions. TODO 9 (interactive_action), then TODO 10 (what Gemini reads about it).

An interactive trick (part4_tricks.py explains them) is one Pupper stays in
and that takes requests: its *actions*, clips Gemini asks for while Pupper is in
it. interactive_action plays them, for every interactive trick: the shakes and
the stand-up of sit_stay_shake (Part 5), and spider_up, the spider's way back
to standing (Part 6). You train sit_stay_shake in the notebook and fetch it with
robot/get_trick.py; until then, the tests use a made-up interactive trick
called hold_pose.
"""

from __future__ import annotations

from part3_follow import prepare_to_move
from part4_tricks import play_policy_trick
from robot_api import FELL_MESSAGE, LOCOMOTION, LOCOMOTION_CONTROLLER, WALK_READY_S, Result, Robot, not_written, with_note  # noqa: F401
from tool_registry import ACTION_IDS, Param, handshake_trick, tool


# ============================================================ what the robot does
async def interactive_action(robot: Robot, action: str) -> Result:
    """Play one of an interactive trick's actions (e.g. shake a paw while sitting).

    TODO 9 (Part 5) ------------------------------------------------------------
    Now:                 robot.action_ids() maps each action to (trick, clip). An
                         unknown action fails, listing the real ones. An action
                         only works in its trick's state: refuse unless
                         robot.state_after_queue is trick, and tell Gemini which
                         trick to play first.
    Later (the job):     check again, about right now: robot.state_now is trick
                         (Pupper may have fallen, been e-stopped or stopped
                         since). Then play the clip with robot.play_clip. The
                         exit clip is different: it leaves the trick, so instead
                         hand the legs back with
                         robot.switch_to(LOCOMOTION_CONTROLLER), which plays the
                         exit clip on the way out.
    Gemini hears now:    robot.queue(f"{trick.name} {clip}", job,
                         trick.clip_time_estimate(clip), then=...):
                         LOCOMOTION after the exit clip, else trick.
    Why check twice:     "sit, then shake" said in one breath queues both before
                         the sit has happened (state_after_queue knows it will);
                         a fall or a stop can still happen in between (state_now
                         knows when the job runs).
    Use:                 robot.action_ids, robot.state_after_queue,
                         robot.state_now, robot.play_clip, robot.switch_to
    Tests check:         - an unknown action fails, and the message lists the real ones.
                           Passes: "unknown interactive action fetch_the_ball; available: ['wave_left', ...]"
                         - an action while Pupper isn't (going to be) in its trick fails
                           right away, names the trick, and queues nothing.
                           Passes: "wave_left only works while hold_pose is running;
                                    play it first (play_trick hold_pose)"
                           Fails:  "not sitting" (Gemini can't tell which trick to play)
                         - in the trick: the clip plays and Pupper stays in the trick
                         - the exit clip: locomotion takes the legs back, and later
                           actions are refused (then=LOCOMOTION)
                         - "sit, then shake" queued back to back plays both
                           (the now-check must use state_after_queue)
                         - if the trick was relaxed before the queue got there, the
                           job refuses (the later check: robot.state_now)
    Test:                python3 lab/test.py todo9
    """
    # ========== TODO 9: YOUR CODE HERE ==========
    # NOW:
    # 1. found = robot.action_ids().get(action). If None: fail, listing list(robot.action_ids()).
    # 2. trick, clip = found
    # 3. If robot.state_after_queue is not trick: fail, and say which trick to play first
    #    (e.g. f"... play it first (play_trick {trick.name})").
    # THE JOB:
    # 4. async def job() -> Result:
    #        a. If robot.state_now is not trick: fail, saying which trick must run first.
    #        b. The exit clip (clip == trick.exit_clip) leaves the trick:
    #           res = await robot.switch_to(LOCOMOTION_CONTROLLER) (it plays the exit clip first),
    #           then report how that went.
    #        c. Any other clip: return await robot.play_clip(trick, clip)
    # QUEUE IT:
    # 5. return await robot.queue(f"{trick.name} {clip}", job, trick.clip_time_estimate(clip),
    #                              then=...)   LOCOMOTION after the exit clip, else trick.
    found = robot.action_ids().get(action)
    if found is None:
        return Result(False, f"unknown interactive action {action}; available: {list(robot.action_ids())}")
    trick, clip = found
    first = f"{action} only works while {trick.name} is running; play it first (play_trick {trick.name})"
    if robot.state_after_queue is not trick:
        return Result(False, first)

    async def job() -> Result:
        if robot.state_now is not trick:
            return Result(False, first)
        if not robot.trick_available(trick):
            return Result(False, "The trick is locked; unlock it before requesting an action, or stand up to leave it.")
        if clip == trick.exit_clip:
            res = await robot.switch_to(LOCOMOTION_CONTROLLER)
            return Result(res.ok, f"played {clip}; back in locomotion mode" if res.ok else res.message)
        return await robot.play_clip(trick, clip)

    # The exit action includes switch_to's existing walking initialization/fade.
    estimate = trick.clip_time_estimate(clip) + (WALK_READY_S if clip == trick.exit_clip else 0.0)
    return await robot.queue(f"{trick.name} {clip}", job, estimate,
                             then=LOCOMOTION if clip == trick.exit_clip else trick)
    # ====================================


# ============================================================ what Gemini reads
# TODO 10: describe interactive_action for Gemini, and write its body (it calls
# interactive_action above). {actions} in a description becomes the list of installed
# actions, each with its trick and its description from trick.yaml. Say what has
# to happen first, and what ends it. (Also TODO 10: the words in
# tricks/sit_stay_shake/trick.yaml, once you have fetched your policy.)
#   Tests check:
#   - the description is 20+ characters, and action is described
#   - it contains {actions} (each action's description appears in it)
#   - action keeps enum=ACTION_IDS, and the tool plays the clip
#   - trick.yaml (once fetched): no TODO left, description 20+ characters, and
#     every action described in 10+ characters.
#     Passes: shake_left: "Shakes the left paw, then keeps sitting."  Fails: shake_left: "left"
#   Test: python3 lab/test.py todo10
# ========== TODO 10: YOUR CODE HERE ==========
@tool(
    "interactive_action",
    "Play an action of the interactive trick Pupper is in, or is already queued to enter. Play the trick first."
    " Shaking a paw keeps its pose; the exit action returns to standing. Other motion tools leave a seated trick."
    " Actions:\n{actions}",
    action=Param("string", "Exact action id from the available list; only usable during its trick.", enum=ACTION_IDS),
)
async def interactive_action_tool(robot: Robot, action: str) -> Result:
    # One line: call interactive_action(robot, action) (your TODO 9), await it, and return it.
    return await interactive_action(robot, action)
# =====================================


@tool(
    "shake_hand",
    "Offer exactly one explicitly named paw. The server queues Sit entry if needed, then the paw action,"
    " and leaves the Sit controller selected. Use this single call for a named handshake; no prior status"
    " or play_trick call is needed. Ask which paw if unclear. Does not stand afterward."
    " A queued reply is acceptance; completion requires the final paw's controller playback confirmation.",
    paw=Param("string", "Requested robot paw, left or right; do not infer an unclear side.", enum=["left", "right"]),
    _ends_following=True,
)
async def shake_hand_tool(robot: Robot, paw: str) -> Result:
    """One cancellable queue job, ending only after the requested paw completes."""
    if paw not in ("left", "right"):
        return Result(False, "paw must be left or right")
    trick = handshake_trick(robot)
    if trick is None:
        return Result(False, "The installed, unlocked Sit-and-both-paws capability is unavailable.")
    if robot.fallen:
        return Result(False, FELL_MESSAGE)
    action = f"shake_{paw}"
    note = await prepare_to_move(robot)
    if isinstance(note, Result):
        return note
    entry_planned = robot.state_after_queue is not trick
    estimate = trick.clip_time_estimate(action)
    if entry_planned:
        estimate += trick.total_time + 0.3 + robot.transition_time_estimate(trick.controller)

    async def job() -> Result:
        if handshake_trick(robot) is not trick:
            return Result(False, "The handshake capability was locked or changed before playback.")
        if robot.fallen or robot.stopped_during_call():
            return Result(False, "Handshake interrupted by a fall or stop.")
        observed = await robot.mode()
        mode = (observed.details or {}).get("observed_controller_mode")
        if not observed.ok or mode in (None, "unknown", "locked_skill"):
            return Result(False, "Handshake refused: current controller ownership is unavailable or unknown.")
        # Do not let play_policy_trick's idempotent early return mistake a stale
        # cached Sit selection for an active Sit controller (or replay an entry
        # after external selection). Refuse the mismatch rather than guess.
        if (robot.state_now is trick) != (mode == "sit_stay_shake"):
            return Result(False, "Handshake refused: observed Sit selection differs from the queued controller state.")
        entry = await play_policy_trick(robot, trick)
        if not entry.ok:
            return entry
        if handshake_trick(robot) is not trick:
            return Result(False, "The handshake capability was locked or changed before the paw action.")
        if robot.state_now is not trick or robot.fallen or robot.stopped_during_call():
            return Result(False, "Handshake interrupted before the paw action.")
        result = await robot.play_clip(trick, action)
        if not result.ok:
            return result
        return Result(True, f"{action} controller playback confirmed; Sit controller remains selected.")

    queued = await robot.queue(f"sit_stay_shake {action}", job, estimate, then=trick)
    if not queued.ok:
        return queued
    details = {**queued.details, "handshake": {
        "schema": "pupper_handshake_submission_v1", "paw": paw, "action": action}}
    return with_note(note, Result(True, queued.message, details))
