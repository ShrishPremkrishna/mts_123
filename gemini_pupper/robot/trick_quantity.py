"""External trick quantity contract; no physical repetition counter exists."""
from robot_api import Result


def validate_trick_quantity(count, count_unit) -> Result | None:
    """Validate before preparing movement; an accepted quantity is one routine."""
    code = None
    if type(count) is not int or not 1 <= count <= 10 or count_unit not in ("routine_playback", "physical_repetition"):
        code = "invalid_quantity"
        message = "count must be an integer from 1 to 10, with count_unit routine_playback or physical_repetition"
    elif count_unit == "physical_repetition":
        code = "physical_count_unsupported"
        message = "Exact physical repetition counts are not supported. A recorded routine may contain multiple reference cycles; it is not one measured repetition. Ask before offering one routine instead."
    elif count != 1:
        code = "playback_count_unsupported"
        message = "Only one routine playback is supported per request. Do not split this unsupported count into repeated calls or substitute one playback."
    if code:
        return Result(False, message, {"refusal": {"schema": "pupper_trick_quantity_refusal_v1", "code": code}})
    return None
