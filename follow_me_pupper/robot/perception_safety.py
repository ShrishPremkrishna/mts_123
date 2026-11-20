"""Camera freshness checks shared by the viewer and service (no ROS imports)."""
import math

MAX_FRAME_AGE_S = 1.0  # matches the tracking state's perception timeout
MAX_DETECTION_FPS = 20.0  # existing viewer's Detect FPS control


class DetectionCadence:
    """Phase-preserving admission of latest frames, without catch-up bursts.

    Call ``due`` once per available frame with a local monotonic clock. A true
    result reserves one scheduling slot; it is not proof of worker acceptance
    or completed inference. Pause clears the phase; resume admits a fresh frame
    immediately. Invalid inputs leave the schedule unchanged and refuse work.
    """

    def __init__(self):
        self._last_now = None
        self._period = None
        self._next_due = None
        self._last_admitted = None

    def due(self, now, fps, enabled=True):
        if (type(enabled) is not bool or isinstance(now, bool) or isinstance(fps, bool)
                or not isinstance(now, (int, float)) or not isinstance(fps, (int, float))):
            return False
        try:
            if not math.isfinite(now) or now < 0 or not math.isfinite(fps) or not 1 <= fps <= MAX_DETECTION_FPS:
                return False
        except OverflowError:
            return False
        if self._last_now is not None and now < self._last_now:
            return False
        period = 1.0 / fps
        if now + period <= now:
            return False  # this clock value cannot represent the requested cadence
        self._last_now = now
        if not enabled:
            self._next_due = self._period = self._last_admitted = None
            return False
        if self._next_due is None:
            self._next_due = now
        elif period != self._period:
            self._next_due = self._last_admitted + period
        self._period = period
        # Only roundoff at the scheduled boundary is a tie, not camera jitter.
        tolerance = 4 * math.ulp(max(now, self._next_due, period))
        if now + tolerance < self._next_due:
            return False
        missed = max(1, math.floor((now - self._next_due) / period) + 1)
        self._next_due += missed * period
        self._last_admitted = now
        return True


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
