"""Camera freshness checks shared by the viewer and service (no ROS imports)."""
import math

MAX_FRAME_AGE_S = 1.0  # matches the tracking state's perception timeout


def image_stamp_ns(message):
    """Read a ROS image stamp without treating absent test/adaptor metadata as new evidence."""
    try:
        stamp = message.header.stamp
        sec, nanosec = int(stamp.sec), int(stamp.nanosec)
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            return None
        return sec * 1_000_000_000 + nanosec
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def fresh_frame(received_at, now, max_age=MAX_FRAME_AGE_S):
    return (math.isfinite(received_at) and math.isfinite(now)
            and 0 <= now - received_at <= max_age)
