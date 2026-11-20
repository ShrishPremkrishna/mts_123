"""Display-only wire contract for a ball seen by the onboard camera."""
import json
import math

TOPIC = "/fc/onboard_ball"
MODE_TOPIC = "/fc/onboard_ball_mode"
SCHEMA = "fc_onboard_ball_v1"
MODES = ("pink", "confirmed", "off")
MAX_AGE_NS = 350_000_000


def capture_is_current(stamp, now_ns):
    return type(stamp) is int and stamp > 0 and 0 <= now_ns - stamp <= MAX_AGE_NS


class OnboardBallReceiver:
    """Age the original measurement on the Pi, including periods of ROS silence."""

    def __init__(self):
        self.row = None
        self.arrival = self.initial_age_ns = None

    def accept(self, raw, ros_now_ns, mono_now):
        try:
            if not isinstance(raw, str) or len(raw) > 4096:
                return False
            row = json.loads(raw)
            mode, state, stamp = row["mode"], row["state"], row["captured_at_ros_ns"]
            allowed = ("off",) if mode == "off" else ("tracked", "searching", "ambiguous")
            if (row["schema"] != SCHEMA or mode not in MODES or state not in allowed
                    or not capture_is_current(stamp, ros_now_ns)
                    or type(row["sequence"]) is not int or row["sequence"] < 0):
                return False
            if self.row is not None and stamp <= self.row["captured_at_ros_ns"]:
                return False
            # Both ROS processes run on this Pi. Carry the monotonic deadline
            # across DDS so a paused or stepped ROS clock cannot renew a frame.
            mono_stamp = row["captured_at_mono_ns"]
            mono_age = round(mono_now * 1e9) - mono_stamp
            if type(mono_stamp) is not int or not 0 <= mono_age <= MAX_AGE_NS:
                return False
            item = dict(state=state, mode=mode, captured_at_ros_ns=stamp, sequence=row["sequence"])
            if state == "tracked":
                width = row["projection_width"]
                if type(width) is not int or not 2 <= width <= 16384:
                    return False
                box, source = row["box"], row["source"]
                if (not isinstance(box, list) or len(box) != 4
                        or any(type(v) not in (int, float) or not math.isfinite(v) for v in box)
                        or not (0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1)
                        or source not in ("pink", "pink+yolo") or (mode == "confirmed" and source != "pink+yolo")):
                    return False
                cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                # The box uses pixel edges. The centres of the first and last
                # pixels are longitude endpoints in create_equirectangular_rays.
                longitude = ((cx * width - .5) / (width - 1) - .5) * 220
                item.update(source=source, box=box, center=[cx, cy], bearing_deg=round(longitude, 1),
                            bearing_reference="camera_right_approx")
            self.row, self.arrival, self.initial_age_ns = item, mono_now, max(ros_now_ns - stamp, mono_age)
            return True
        except (KeyError, ValueError, TypeError, OverflowError):
            return False

    def snapshot(self, ros_now_ns, mono_now):
        result = dict(state="stale", mode=None, age_ms=None, limit_ms=MAX_AGE_NS // 1_000_000)
        if self.row is None:
            return result
        stamp = self.row["captured_at_ros_ns"]
        elapsed = mono_now - self.arrival
        age_ns = max(ros_now_ns - stamp, self.initial_age_ns + round(elapsed * 1e9))
        result.update(mode=self.row["mode"], sequence=self.row["sequence"], captured_at_ros_ns=str(stamp),
                      age_ms=math.ceil(age_ns / 1e6))
        if elapsed >= 0 and stamp <= ros_now_ns and age_ns <= MAX_AGE_NS:
            result.update({k: v for k, v in self.row.items() if k != "captured_at_ros_ns"})
        return result
