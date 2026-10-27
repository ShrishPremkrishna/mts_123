"""Pure PD-control helpers; units are radians, seconds and N m."""
import math
from collections import deque


def sine_target(t, amplitude=0.2, frequency=1 / (2 * math.pi), center=0.0):
    if not all(math.isfinite(v) for v in (t, amplitude, frequency, center)):
        raise ValueError("trajectory parameters must be finite")
    omega = 2 * math.pi * frequency
    return center + amplitude * math.sin(omega * t), amplitude * omega * math.cos(omega * t)


def pd_torque(position, velocity, target_position, target_velocity, kp, kd, limit):
    values = (position, velocity, target_position, target_velocity, kp, kd, limit)
    if not all(math.isfinite(v) for v in values):
        return 0.0
    if kp < 0 or kd < 0 or limit <= 0:
        raise ValueError("gains must be nonnegative and torque limit positive")
    return max(-limit, min(limit, kp * (target_position-position) + kd * (target_velocity-velocity)))


class SampleDelay:
    """Explicit sampled delay. Warmup returns None, zero delay returns today's sample."""
    def __init__(self, seconds, rate_hz):
        if not math.isfinite(seconds) or seconds < 0 or not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("delay must be nonnegative and sample rate positive")
        self.steps = int(round(seconds * rate_hz))
        if self.steps > 2000:
            raise ValueError("delay exceeds experiment buffer limit")
        self.buffer = deque(maxlen=self.steps + 1)

    def clear(self):
        self.buffer.clear()

    def push(self, sample):
        self.buffer.append(sample)
        return self.buffer[0] if len(self.buffer) == self.buffer.maxlen else None
