"""Bounded-memory motor inference and nonblocking one-second contact pauses."""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import time
import numpy as np
from .data import CHANNELS, sha256


class MotorWindow:
    def __init__(self, window=100, sample_hz=1000., max_gap_s=.025):
        if window < 2 or sample_hz <= 0 or max_gap_s <= 0:
            raise ValueError("Invalid streaming window configuration")
        self.window, self.sample_hz, self.max_gap_s = window, sample_hz, max_gap_s
        self.samples = deque(maxlen=max(4096, window * 4))
        self.last_status = "warming_up"

    def push(self, timestamp, values):
        values = np.asarray(values, dtype=np.float32)
        if values.shape != (9,) or not np.isfinite(values).all() or not np.isfinite(timestamp):
            self.samples.clear()
            self.last_status = "invalid_sample"
            return None
        if self.samples and timestamp <= self.samples[-1][0]:
            self.last_status = "duplicate_timestamp" if timestamp == self.samples[-1][0] else "time_rewind"
            self.samples.clear()  # Time rewinds/duplicates invalidate temporal evidence.
            return None
        self.samples.append((float(timestamp), values))
        start = timestamp - (self.window - 1) / self.sample_hz
        while len(self.samples) > 2 and self.samples[1][0] < start:
            self.samples.popleft()
        if len(self.samples) < 2 or self.samples[0][0] > start:
            self.last_status = "warming_up"
            return None
        t = np.asarray([s[0] for s in self.samples])
        if np.diff(t).max() > self.max_gap_s:
            self.last_status = "sensor_gap"
            return None
        x = np.stack([s[1] for s in self.samples])
        grid = start + np.arange(self.window) / self.sample_hz
        self.last_status = "ready"
        return np.stack([np.interp(grid, t, x[:, i]) for i in range(9)], axis=1).astype(np.float32)


def motor_channels(joint_names, positions, velocities, efforts):
    """Map published JointState channels without inventing a torque measurement.

    The local class hardware driver publishes a PD-based effort estimate. The
    historical dataset driver's identity is unknown; live provenance, gains and
    scaling require verification. Preserve the published values and named order.
    """
    wanted = [f"leg_front_r_{i}" for i in (1,2,3)]
    if len(set(joint_names)) != len(joint_names):
        raise ValueError("Duplicate joint names")
    indices = [list(joint_names).index(name) for name in wanted]
    if any(len(values) != len(joint_names) for values in (positions,velocities,efforts)):
        raise ValueError("Position, velocity and published effort channels must all be present")
    return np.asarray([values[i] for values in (positions,velocities,efforts) for i in indices], dtype=np.float32)


class ContactPredictor:
    def __init__(self, directory, inference_hz=20.):
        import torch
        directory=Path(directory)
        self.metadata=json.loads((directory/"metadata.json").read_text())
        if self.metadata["channels"] != CHANNELS or sha256(directory/"contact.pt") != self.metadata["model_sha256"]:
            raise ValueError("Export channel contract or checksum mismatch")
        self.model=torch.jit.load(str(directory/"contact.pt"),map_location="cpu").eval()
        torch.set_num_threads(1)
        self.window=MotorWindow(self.metadata["window"],self.metadata["sample_hz"],self.metadata["max_gap_s"])
        if not np.isfinite(inference_hz) or inference_hz <= 0: raise ValueError("inference_hz must be finite and positive")
        self.period=1./inference_hz
        self.last_inference=-float("inf")
        self.last_timestamp=None
        self.last_status="warming_up"

    def invalidate(self, reason="invalid_sample"):
        """Discard history after a missing/stale input; never bridge invalid evidence."""
        self.window.samples.clear()
        self.window.last_status=reason
        self.last_timestamp=None
        self.last_inference=-float("inf")
        self.last_status=reason

    def push(self,timestamp,values):
        import torch
        if self.last_timestamp is not None and timestamp <= self.last_timestamp:
            self.last_inference=-float("inf")
        self.last_timestamp=timestamp
        raw=self.window.push(timestamp,values)
        if raw is None:
            self.last_status=self.window.last_status
            return None  # Missing evidence must never be interpreted as contact.
        if timestamp-self.last_inference < self.period:
            self.last_status="inference_throttled"
            return None
        begin=time.perf_counter()
        with torch.inference_mode():
            contact,incline,material=self.model(torch.from_numpy(raw[None]))
        self.last_inference=timestamp
        self.last_status="predicted"
        return dict(timestamp=timestamp,contact=float(contact[0]),incline=incline[0].tolist(),material=material[0].tolist(),
                    inference_ms=(time.perf_counter()-begin)*1000.)


@dataclass
class ContactPauseClock:
    """Advance trajectory time, freezing once per contact event. Does not block ROS."""
    pause_s: float = 1.
    on_threshold: float = .8
    off_threshold: float = .3
    trigger_samples: int = 2
    release_samples: int = 3
    cooldown_s: float = .5
    phase_time: float = 0.
    last_time: float | None = None
    pause_until: float = -float("inf")
    armed: bool = True
    on_count: int = 0
    off_count: int = 0

    def __post_init__(self):
        if not 0 <= self.off_threshold < self.on_threshold <= 1 or self.pause_s <= 0 or min(self.trigger_samples,self.release_samples) < 1 or self.cooldown_s < 0:
            raise ValueError("Invalid contact hysteresis/pause parameters")

    def advance(self, now, probability=None):
        if not np.isfinite(now): raise ValueError("Nonfinite time")
        if self.last_time is not None:
            if now < self.last_time:
                self.last_time=now; self.pause_until=now; self.armed=False; self.on_count=0; self.off_count=0
                return self.phase_time,False
            # Count only elapsed time after the pause ended, including a long tick.
            self.phase_time+=max(0.,now-max(self.last_time,self.pause_until))
        self.last_time=now
        event=False
        if probability is not None:
            if not np.isfinite(probability) or not 0 <= probability <= 1:
                raise ValueError("Invalid contact probability")
            self.on_count=self.on_count+1 if probability >= self.on_threshold else 0
            self.off_count=self.off_count+1 if probability <= self.off_threshold else 0
            if not self.armed and self.off_count >= self.release_samples and now >= self.pause_until+self.cooldown_s:
                self.armed=True
            if self.armed and self.on_count >= self.trigger_samples:
                self.pause_until=now+self.pause_s; self.armed=False; self.on_count=0; event=True
        return self.phase_time,event
