"""External trick quantity contract; bounded routines, no physical repetition counter."""
from robot_api import Result


def validate_trick_quantity(count, count_unit, *, trick=None) -> Result | None:
    """Validate before preparing movement. Only finite swim accepts two routines."""
    repeatable_swim = (trick is not None and trick.name == "swim" and trick.kind == "policy"
                       and not trick.interactive and not trick.walks and trick.end_pose == "lying")
    code = None
    if type(count) is not int or not 1 <= count <= 10 or count_unit not in ("routine_playback", "physical_repetition"):
        code = "invalid_quantity"
        message = "count must be an integer from 1 to 10, with count_unit routine_playback or physical_repetition"
    elif count_unit == "physical_repetition":
        code = "physical_count_unsupported"
        message = "Exact physical repetition counts are not supported. A recorded routine may contain multiple reference cycles; it is not one measured repetition. Ask before offering one routine instead."
    elif count != 1 and not (count == 2 and repeatable_swim):
        code = "playback_count_unsupported"
        message = "Requested routine playback count is unsupported. Only one playback is supported, except two playbacks of the installed finite swim routine. Do not split an unsupported count into repeated calls or substitute one playback."
    if code:
        return Result(False, message, {"refusal": {"schema": "pupper_trick_quantity_refusal_v1", "code": code}})
    return None
