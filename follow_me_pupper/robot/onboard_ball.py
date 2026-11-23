"""Wire contracts for a ball seen by the onboard camera.

fc_onboard_ball_v1 is display-only bearing telemetry. fc_onboard_ball_metric_v1
carries Ball Eye body-frame measurements (ball_eye.py) with their original
capture times and profile identity; its consumer decides whether they may
refine docking. Neither contract carries prediction or global pose.
"""
import json
import math
import re

TOPIC = "/fc/onboard_ball"
MODE_TOPIC = "/fc/onboard_ball_mode"
SCHEMA = "fc_onboard_ball_v1"
MODES = ("pink", "confirmed", "off")
MAX_AGE_NS = 350_000_000

METRIC_TOPIC = "/fc/onboard_ball_metric"
METRIC_SCHEMA = "fc_onboard_ball_metric_v1"
METRIC_METHODS = ("eye_fit", "eye_box", "pink_fit", "pink_box", "pinkfloor_fit")
METRIC_STATES = ("measured", "searching", "conflict")
PROFILE_STATUSES = ("synthetic", "estimated", "measured")
# Image-noise propagation only; radius, mount, lens and exposure errors are unknown.
UNCERTAINTY_SCOPE = "image_only"
_HEX64 = re.compile(r"[0-9a-f]{64}")


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


def metric_payload(*, producer_instance, sequence, frame_sequence, captured_at_ros_ns, captured_at_mono_ns,
                   identity, model_sha256, state, selected, candidates, processing_ms):
    """Serialize one Ball Eye frame result. The capture times are the frame's own."""
    return json.dumps(dict(schema=METRIC_SCHEMA, producer_instance=producer_instance, sequence=sequence,
                           frame_sequence=frame_sequence, captured_at_ros_ns=captured_at_ros_ns,
                           captured_at_mono_ns=captured_at_mono_ns, identity=identity,
                           model_sha256=model_sha256, state=state, selected=selected, candidates=candidates,
                           uncertainty_scope=UNCERTAINTY_SCOPE, processing_ms=processing_ms),
                      allow_nan=False, separators=(",", ":"))


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _identity(raw):
    if (not isinstance(raw, dict) or set(raw) != {"profile_id", "calibration_id", "status", "ball_radius_m",
                                                   "radius_measured", "mount_measured"}
            or not all(isinstance(raw[k], str) and _HEX64.fullmatch(raw[k]) for k in ("profile_id", "calibration_id"))
            or raw["status"] not in PROFILE_STATUSES or not _number(raw["ball_radius_m"]) or raw["ball_radius_m"] <= 0
            or type(raw["radius_measured"]) is not bool or type(raw["mount_measured"]) is not bool):
        raise ValueError("invalid profile identity")
    return dict(raw)


def _candidate(raw):
    keys = {"method", "x", "y", "z", "range_m", "rho_rad", "circle", "score", "sigma_px", "sigma_xy_m",
            "sigma_range_m", "fit_rms_px", "fit_points"}
    if not isinstance(raw, dict) or set(raw) != keys or raw["method"] not in METRIC_METHODS:
        raise ValueError("invalid metric candidate")
    numbers = [raw[k] for k in ("x", "y", "z", "range_m", "rho_rad", "score", "sigma_px", "sigma_xy_m",
                                "sigma_range_m")]
    circle = raw["circle"]
    if (not all(_number(v) for v in numbers) or not isinstance(circle, list) or len(circle) != 3
            or not all(_number(v) for v in circle) or circle[2] <= 0
            or min(raw["range_m"], raw["rho_rad"], raw["sigma_px"], raw["sigma_xy_m"], raw["sigma_range_m"]) <= 0
            or not 0 <= raw["score"] <= 1):
        raise ValueError("metric candidate numbers are invalid")
    fitted = raw["method"].endswith("_fit")
    if fitted != (_number(raw["fit_rms_px"]) and type(raw["fit_points"]) is int and raw["fit_points"] > 0):
        raise ValueError("fit statistics must accompany exactly the fitted methods")
    if not fitted and (raw["fit_rms_px"] is not None or raw["fit_points"] is not None):
        raise ValueError("box methods carry no fit statistics")
    return dict(raw)


class MetricBallReceiver:
    """Admit Ball Eye rows by their source capture time; arrival never renews them.

    Both clocks travel with the row: the camera's ROS stamp and the same
    capture converted to this Pi's monotonic clock by the camera node. Rows
    must be current on both and strictly newer than the last admitted row; a
    refused row changes nothing. Searching and conflict rows are evidence too:
    they replace an older measurement.
    """

    def __init__(self):
        self.row = None
        self.reason = None
        self.rejected = 0

    def _refuse(self, reason):
        self.reason = reason
        self.rejected += 1
        return False

    def accept(self, raw, ros_now_ns, mono_now):
        try:
            if not isinstance(raw, str) or len(raw) > 8192:
                return self._refuse("missing or oversized row")
            row = json.loads(raw)
            if not isinstance(row, dict) or row.get("schema") != METRIC_SCHEMA:
                return self._refuse("wrong schema")
            stamp, mono_stamp = row["captured_at_ros_ns"], row["captured_at_mono_ns"]
            if not capture_is_current(stamp, ros_now_ns):
                return self._refuse("capture stamp is missing, future or stale")
            if type(mono_stamp) is not int or not 0 <= round(mono_now * 1e9) - mono_stamp <= MAX_AGE_NS:
                return self._refuse("monotonic capture is missing, future or stale")
            if self.row is not None and (stamp <= self.row["captured_at_ros_ns"]
                                         or mono_stamp <= self.row["captured_at_mono_ns"]):
                return self._refuse("duplicate or reordered capture")
            producer, sequence, frame_sequence = row["producer_instance"], row["sequence"], row["frame_sequence"]
            if (not isinstance(producer, str) or not 0 < len(producer) <= 64
                    or type(sequence) is not int or sequence < 0 or type(frame_sequence) is not int
                    or frame_sequence < 0):
                return self._refuse("invalid producer or sequence")
            if (self.row is not None and producer == self.row["producer_instance"]
                    and sequence <= self.row["sequence"]):
                return self._refuse("duplicate or reordered sequence")
            model = row["model_sha256"]
            if model is not None and not (isinstance(model, str) and _HEX64.fullmatch(model)):
                return self._refuse("invalid model identity")
            if row["uncertainty_scope"] != UNCERTAINTY_SCOPE:
                return self._refuse("unknown uncertainty scope")
            state, selected, candidates = row["state"], row["selected"], row["candidates"]
            if state not in METRIC_STATES or not isinstance(candidates, list) or len(candidates) > len(METRIC_METHODS):
                return self._refuse("invalid state or candidates")
            candidates = [_candidate(c) for c in candidates]
            if len({c["method"] for c in candidates}) != len(candidates):
                return self._refuse("repeated method")
            if state == "measured":
                if type(selected) is not int or not 0 <= selected < len(candidates):
                    return self._refuse("measured row needs a selected candidate")
            elif selected is not None or (state == "searching" and candidates) or (
                    state == "conflict" and len(candidates) < 2):
                return self._refuse("selection does not match state")
            processing = row["processing_ms"]
            if not _number(processing) or processing < 0:
                return self._refuse("invalid processing time")
            self.row = dict(producer_instance=producer, sequence=sequence, frame_sequence=frame_sequence,
                            captured_at_ros_ns=stamp, captured_at_mono_ns=mono_stamp,
                            identity=_identity(row["identity"]), model_sha256=model, state=state,
                            selected=selected, candidates=candidates, processing_ms=processing)
            self.reason = None
            return True
        except (KeyError, ValueError, TypeError, OverflowError) as exc:
            return self._refuse(str(exc)[:120] or "malformed row")

    def latest(self, ros_now_ns, mono_now, row=None):
        """The newest admitted row while its capture is current on both clocks, else None."""
        row = self.row if row is None else row
        if row is None or not capture_is_current(row["captured_at_ros_ns"], ros_now_ns):
            return None
        if not 0 <= round(mono_now * 1e9) - row["captured_at_mono_ns"] <= MAX_AGE_NS:
            return None
        return row

    def snapshot(self, ros_now_ns, mono_now):
        """Telemetry: the current row's measurement and identity, labelled unqualified."""
        result = dict(state="stale", age_ms=None, limit_ms=MAX_AGE_NS // 1_000_000, rejected=self.rejected,
                      last_reject=self.reason, uncertainty_scope=UNCERTAINTY_SCOPE)
        admitted = self.row  # read once: a callback on another thread may replace it
        row = self.latest(ros_now_ns, mono_now, admitted) if admitted is not None else None
        if admitted is not None:
            result.update(sequence=admitted["sequence"],
                          age_ms=math.ceil((round(mono_now * 1e9) - admitted["captured_at_mono_ns"]) / 1e6))
        if row is None:
            return result
        result.update(state=row["state"], identity=row["identity"], model_sha256=row["model_sha256"],
                      captured_at_ros_ns=str(row["captured_at_ros_ns"]), processing_ms=row["processing_ms"],
                      methods=[c["method"] for c in row["candidates"]])
        if row["selected"] is not None:
            chosen = row["candidates"][row["selected"]]
            result.update(method=chosen["method"], x=chosen["x"], y=chosen["y"], range_m=chosen["range_m"],
                          sigma_xy_m=chosen["sigma_xy_m"])
        return result
