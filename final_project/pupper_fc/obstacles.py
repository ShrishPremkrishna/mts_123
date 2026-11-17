"""Transport-free front-camera stop/hold gate; no motion or detour commands.

Boxes use normalized ``(left, top, right, bottom)`` coordinates in the fixed
700 x 572 equirectangular detection image (220-degree horizontal projection).
An adapter must normalize that image, not raw fisheye or arbitrary crops. All
``captured_at`` and ``now`` values share ONE local monotonic time basis; source
capture time must never be replaced with inference completion or arrival time.
ROS/wall-clock timestamps need an explicit adapter mapping before admission.

Defaults are provisional replay assumptions, NOT calibrated stopping distance,
physical acceptance, or coverage outside the visible forward corridor. Every
class, including people and balls, can block. Missing detections in a healthy
frame are only the detector's claim of clear space, not proof of free space.

A producer sequence reset requires a new gate and fresh clearance. Its caller
must fence packets from the previous producer/session before admission. Calls
are synchronous; the caller owns serialization, scheduling and publication.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal


BOX_FORMAT = "normalized_xyxy_v1"


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("must be a finite number")
    try:
        result = float(value)
    except OverflowError:
        raise ValueError("must be a finite number") from None
    if not math.isfinite(result):
        raise ValueError("must be a finite number")
    return result


@dataclass(frozen=True)
class Detection:
    box: tuple[float, float, float, float]
    confidence: float
    class_id: str = ""


@dataclass(frozen=True)
class DetectionFrame:
    sequence: int
    captured_at: float
    detections: tuple[Detection, ...] = ()
    valid: bool = True
    healthy: bool = True
    format: str = BOX_FORMAT


@dataclass(frozen=True)
class ObstacleConfig:
    """Provisional geometry/confidence policy; all image dimensions normalized."""

    corridor_left: float = 0.35
    corridor_right: float = 0.65
    min_confidence: float = 0.4
    stop_bottom: float = 0.70
    release_bottom: float = 0.65
    max_frame_age_s: float = 0.35
    clear_frames: int = 3

    def __post_init__(self):
        for name in ("corridor_left", "corridor_right", "min_confidence",
                     "stop_bottom", "release_bottom", "max_frame_age_s"):
            try:
                _number(getattr(self, name))
            except ValueError as exc:
                raise ValueError(f"{name} {exc}") from None
        if not 0 <= self.corridor_left < self.corridor_right <= 1:
            raise ValueError("corridor must satisfy 0 <= left < right <= 1")
        if not 0 <= self.min_confidence <= 1:
            raise ValueError("min_confidence must be in [0, 1]")
        if not 0 <= self.release_bottom <= self.stop_bottom <= 1:
            raise ValueError("bottom thresholds must satisfy 0 <= release <= stop <= 1")
        if self.max_frame_age_s <= 0:
            raise ValueError("max_frame_age_s must be positive")
        if type(self.clear_frames) is not int or self.clear_frames < 1:
            raise ValueError("clear_frames must be a positive integer")


@dataclass(frozen=True)
class ObstacleDecision:
    state: Literal["clear", "blocked", "stale", "invalid"]
    reason: str
    clear_frames: int
    captured_at: float | None
    sequence: int | None
    valid_until: float

    @property
    def permitted(self):
        """Only a clear decision permits the caller's otherwise-authorized action."""
        return self.state == "clear"


class ForwardObstacleGate:
    """Fail closed until distinct, fresh clear frames earn clearance.

    ``evaluate`` must also run when frames stop arriving. A clear decision's
    ``valid_until`` is its source capture time plus the configured age limit;
    callers must clip command expiry to that deadline and reevaluate afterward.
    Invalid, stale and blocked decisions permit no motion. They do not request
    controller deactivation, reverse, turning, or hardware initialization.
    """

    def __init__(self, config=ObstacleConfig()):
        if not isinstance(config, ObstacleConfig):
            raise TypeError("config must be ObstacleConfig")
        self.config = config
        self._last_now = None
        self._sequence = -1
        self._capture_highwater = -math.inf
        self._last_capture = None
        self._clear_count = 0
        self._holding = True
        self._decision = ObstacleDecision("stale", "no detection frame received", 0, None, None, 0.0)

    def _hold(self, state, reason, now):
        self._clear_count = 0
        self._holding = True
        self._decision = ObstacleDecision(
            state, reason, 0, self._last_capture,
            self._sequence if self._sequence >= 0 else None, now)
        return self._decision

    def _clock(self, now):
        try:
            now = _number(now)
            if now < 0:
                raise ValueError("must be nonnegative")
            if self._last_now is not None and now < self._last_now:
                raise ValueError("moved backwards")
        except ValueError as exc:
            self._hold("invalid", f"local monotonic clock {exc}", self._last_now or 0.0)
            return None
        self._last_now = now
        return now

    def invalidate(self, reason, now):
        """Reject malformed adapter input without inventing a capture timestamp.

        Sequence/capture highwaters survive, so rejected packets cannot make a
        repeated old frame eligible for clearance again.
        """
        now = self._clock(now)
        if now is None:
            return self._decision
        return self._hold("invalid", str(reason), now)

    def _expire(self, now):
        if (self._last_capture is not None and
                now >= self._last_capture + self.config.max_frame_age_s):
            self._hold("stale", "source detection frame expired", now)

    def evaluate(self, now):
        now = self._clock(now)
        if now is None:
            return self._decision
        self._expire(now)
        return self._decision

    def update(self, frame, now):
        now = self._clock(now)
        if now is None:
            return self._decision
        # A fresh arrival after a gap cannot retain previous clearance credit.
        self._expire(now)
        try:
            if not isinstance(frame, DetectionFrame):
                raise ValueError("expected DetectionFrame")
            if frame.format != BOX_FORMAT:
                raise ValueError("unexpected detection coordinate format")
            if type(frame.valid) is not bool or type(frame.healthy) is not bool:
                raise ValueError("valid and healthy must be explicit booleans")
            if type(frame.sequence) is not int or frame.sequence < 0:
                raise ValueError("sequence must be a nonnegative integer")
            captured_at = _number(frame.captured_at)
            if captured_at < 0 or captured_at > now:
                raise ValueError("source capture time is negative or in the future")
            if frame.sequence <= self._sequence:
                raise ValueError("repeated or reordered sequence")
            if captured_at <= self._capture_highwater:
                raise ValueError("repeated or reordered source capture time")
            # Even an unhealthy frame with well-formed new metadata consumes its
            # identity. Re-sending that identity cannot become new clear evidence.
            self._sequence = frame.sequence
            self._capture_highwater = captured_at
            self._last_capture = captured_at
            if not frame.valid or not frame.healthy:
                raise ValueError("detector reports an invalid or unhealthy frame")
            if not isinstance(frame.detections, tuple):
                raise ValueError("detections must be an immutable tuple")
            for detection in frame.detections:
                self._validate_detection(detection)
        except ValueError as exc:
            return self._hold("invalid", str(exc), now)

        if now >= captured_at + self.config.max_frame_age_s:
            return self._hold("stale", "source detection frame expired on arrival", now)

        bottom = self.config.release_bottom if self._holding else self.config.stop_bottom
        for detection in frame.detections:
            left, _, right, lower = detection.box
            # Include boundary contact and boxes whose center lies outside the
            # corridor. Image-border clipping is not evidence to exempt an object.
            if (detection.confidence >= self.config.min_confidence and
                    right >= self.config.corridor_left and
                    left <= self.config.corridor_right and lower >= bottom):
                label = detection.class_id or "unknown class"
                return self._hold("blocked", f"near detection overlaps forward corridor ({label})", now)

        self._clear_count = min(self.config.clear_frames, self._clear_count + 1)
        clear = self._clear_count >= self.config.clear_frames
        self._holding = not clear
        self._decision = ObstacleDecision(
            "clear" if clear else "blocked",
            "fresh forward corridor clear" if clear else
            f"awaiting distinct clear frames ({self._clear_count}/{self.config.clear_frames})",
            self._clear_count, captured_at, frame.sequence,
            captured_at + self.config.max_frame_age_s if clear else now)
        return self._decision

    @staticmethod
    def _validate_detection(detection):
        if not isinstance(detection, Detection):
            raise ValueError("expected Detection")
        if not isinstance(detection.box, tuple) or len(detection.box) != 4:
            raise ValueError("box must be an immutable normalized xyxy tuple")
        left, top, right, bottom = (_number(value) for value in detection.box)
        if not (0 <= left < right <= 1 and 0 <= top < bottom <= 1):
            raise ValueError("box must have positive area within normalized image bounds")
        confidence = _number(detection.confidence)
        if not 0 <= confidence <= 1:
            raise ValueError("confidence must be in [0, 1]")
        if not isinstance(detection.class_id, str):
            raise ValueError("class_id must be a string")
