"""Play one explicitly selected soccer kick from an already active walker."""

from part3_follow import prepare_to_move
from robot_api import ACTIVATION_GATE, FELL_MESSAGE, LOCOMOTION, LOCOMOTION_CONTROLLER, Result, Robot, with_note
from tricks import TRICK_FADE_S, TRICK_INIT_S, WALK_READY_S, Trick


# At 2.6 s the exported kicking foot and weight shift have returned to the
# standing reference. Its remaining 0.4 s is a still tail.
MOTION_END_S = 2.6
SETTLE_S = 0.15


def interrupted(robot: Robot) -> str | None:
    if robot.fallen:
        return FELL_MESSAGE
    if robot.stopped_during_call():
        return "operator kick interrupted by Stop"
    return robot.motion_block_reason()


async def confirm_kick_progress(robot: Robot, trick: Trick, clip: str) -> Result:
    """Require an in-range frame of this selected clip during this activation."""
    controller = trick.controller
    began = robot.policy_activation_since.get(controller)
    token = robot.policy_execution_requests.get(controller)
    if began is None or not token or token == "none":
        return Result(False, f"missing activation identity for {clip}")
    spec = trick.clip(clip)
    minimum_frame = spec["start"] + round(MOTION_END_S * trick.fps)
    earliest = began + TRICK_INIT_S + TRICK_FADE_S + MOTION_END_S
    while True:
        if reason := interrupted(robot):
            return Result(False, reason)
        now = robot.now()
        receipt = robot.clip_receipts.get(controller)
        if receipt:
            name, holding, frame, received, activation, request, queued = receipt
            if (name == clip and not holding and minimum_frame <= frame < spec["end"]
                    and token == activation == robot.policy_execution_requests.get(controller)
                    and request == "none" and queued == 0 and received >= earliest
                    and 0 <= now - received <= 0.25):
                return Result(True, f"controller confirmed {clip} at frame {frame}")
        if now >= earliest + 1.0:
            return Result(False, f"controller did not confirm {clip} through frame {minimum_frame}")
        await robot.sleep(0.02)


async def kick_in_place(robot: Robot, side: str, *, entry_gate=None, return_gate=None,
                        on_entry=None, on_clip=None) -> Result:
    if side not in ("left", "right"):
        return Result(False, "side must be left or right")
    trick = robot.tricks.get("leg_kick")
    if trick is None or not trick.operator_only or not robot.trick_available(trick):
        return Result(False, "operator leg kick unavailable in the selected session")
    clip = f"kick_{side}"
    clips = {spec["name"] for spec in trick.clips}
    if trick.kind != "policy" or clip not in clips or trick.clip_length(clip) < MOTION_END_S:
        return Result(False, f"selected policy lacks the required {clip} motion")
    if reason := interrupted(robot):
        return Result(False, reason)
    note = await prepare_to_move(robot)
    if isinstance(note, Result):
        return note

    async def job() -> Result:
        if robot.tricks.get("leg_kick") is not trick or not robot.trick_available(trick):
            return Result(False, "operator kick selection changed before execution")
        walker = LOCOMOTION_CONTROLLER
        ready = await robot.switch_to(walker, expected_active=walker)
        if not ready.ok:
            return ready
        robot.send_speed(0, 0, 0)
        if entry_gate is None:
            await robot.sleep(SETTLE_S)
        # The FC caller observed stillness at the paw over successive captures.
        # Its ticket expires during an extra fixed settle, so check that ticket
        # through to the actual switch instead. HTTP has no way to set a gate.
        gate = ACTIVATION_GATE.set(entry_gate)
        try:
            entry = await robot.switch_to(trick.controller, start_clip=clip, expected_active=walker)
        finally:
            ACTIVATION_GATE.reset(gate)
        if not entry.ok:
            return Result(False, f"{clip} entry failed: {entry.message}")
        if on_entry is not None:
            on_entry(clip)
        try:
            playback = await confirm_kick_progress(robot, trick, clip)
        except Exception as error:
            # The queue's generic failure handler only zeros walking velocity.
            # This job still owns an autonomous policy and must hand it back.
            playback = Result(False, f"{clip} playback check failed: {type(error).__name__}: {error}")
        if reason := interrupted(robot):
            return Result(False, f"{playback.message}. Walker restoration unconfirmed: {reason}")
        if playback.ok and on_clip is not None:
            on_clip(clip)
        gate = ACTIVATION_GATE.set(return_gate)
        try:
            back = await robot.switch_to(walker, expected_active=trick.controller)
        finally:
            ACTIVATION_GATE.reset(gate)
        if not back.ok:
            message = f"{playback.message}. Walker restoration unconfirmed: {back.message}"
            if not interrupted(robot):
                # A refused walker switch can leave the kick policy running.
                # Release only if the inventory still identifies it as sole owner.
                release = await robot.switch_to(None, expected_active=trick.controller)
                message += f". Kick controller release request: {release.message}"
            return Result(False, message)
        states = await robot.controller_states()
        if reason := interrupted(robot):
            return Result(False, f"{playback.message}. Walker restoration unconfirmed: {reason}")
        if {c for c in robot.leg_controllers if states.get(c) == "active"} != {walker}:
            return Result(False, f"{playback.message}. Could not verify the walker as the active leg controller.")
        evidence = {"schema": "pupper_operator_kick_result_v1", "side": side,
                    "confirmed_clip": clip if playback.ok else None, "restored_controller": walker}
        return Result(playback.ok, f"{playback.message}. Walker restoration confirmed. Ball contact unmeasured.",
                      {"operator_kick": evidence})

    estimate = (robot.transition_time_estimate(LOCOMOTION_CONTROLLER) + SETTLE_S
                + TRICK_INIT_S + TRICK_FADE_S + MOTION_END_S + WALK_READY_S)
    name = f"kick_in_place {side}"
    result = await robot.queue(name, job, estimate, then=LOCOMOTION)
    if result.ok:
        result.details["submission"]["name"] = name
    return with_note(note, result)
