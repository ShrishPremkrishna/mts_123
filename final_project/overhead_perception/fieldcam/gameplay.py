"""Bridge to the gameplay controller: perception state -> final_project.pupper_fc.model.Observation.

Both sides already use the same field frame (metres, origin at the field
centre, +x toward one goal, +y left of it, yaw counter-clockwise from +x), so
this only reshapes the data and is strict about time and missing data:

- The perception state is extrapolated to "now" for consumers that want a
  prediction. Gameplay wants what the camera actually saw and when, so this
  hands back the pose at CAPTURE time with that capture time.
- Anything not measured in this very frame is reported as missing (None). A
  coasting track is a guess, and the controller decides what to do without it.
- Calibration validity must be explicit; drift/reconnect holds and legacy
  packets without measurement provenance produce no robot or ball observation.

`fallen` and `goal` are not observed by the overhead camera; they stay at the
caller's values (default: not fallen, no goal).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]  # .../final_project/overhead_perception/fieldcam -> repo root


def _model():
    try:
        from final_project.pupper_fc import model
    except ModuleNotFoundError:  # run from inside overhead_perception/: add the repo root
        sys.path.insert(0, str(REPO_ROOT))
        from final_project.pupper_fc import model
    return model


def to_observation(state: dict, robot: str = "striker", robot_id: str = "pupper-1",
                   clock_offset: float = 0.0, fallen: bool = False, goal: int = 0):
    """One perception state (the dict published over UDP) -> one gameplay Observation.

    robot:        which perception robot this controller drives ("striker" / "keeper").
    robot_id:     the id gameplay uses for it.
    clock_offset: add this to perception's clock to get the controller's clock
                  (0 when both read time.time() on the same machine).
    """
    m = _model()
    # Timestamp fields are retained at full precision on the wire. Rounded ages
    # cannot establish whether a measurement came from this frame (e.g. at 60 Hz).
    frame_capture = state.get("captured_at")
    capture_valid = (isinstance(frame_capture, (float, int)) and not isinstance(frame_capture, bool)
                     and math.isfinite(frame_capture))
    # Invalid provenance stays invalid even when display timing looks plausible.
    captured_at = frame_capture + clock_offset if capture_valid else math.nan
    frame_age = state["stamp"] - frame_capture if capture_valid else state["frame_age_s"]

    def from_this_frame(obj) -> bool:
        measured_at = obj.get("measured_at")
        return (capture_valid and obj.get("visible") is True and obj.get("measured_this_frame") is True
                and isinstance(measured_at, (float, int)) and not isinstance(measured_at, bool)
                and math.isfinite(measured_at) and measured_at == frame_capture)

    pose, ball = None, None
    r = state["robots"].get(robot, {})
    if state.get("calibration_valid") is True and from_this_frame(r):
        # Undo the extrapolation to "now": report where it was when the photons arrived.
        yaw = r.get("yaw_at_capture")
        if "yaw_at_capture" not in r:
            tracking = state.get("tracking", "tags")
            if tracking == "colors":
                yaw = r["yaw"]
            elif tracking == "tags":
                yaw = r["yaw"] - r["wz"] * frame_age
        if type(yaw) in (float, int) and math.isfinite(yaw):
            pose = m.Pose(r["x"] - r["vx"] * frame_age, r["y"] - r["vy"] * frame_age,
                          math.atan2(math.sin(yaw), math.cos(yaw)))
    b = state["ball"]
    if state.get("calibration_valid") is True and from_this_frame(b):
        ball = m.Ball(b["x"] - b["vx"] * frame_age, b["y"] - b["vy"] * frame_age, b["vx"], b["vy"])
    return m.Observation(captured_at=captured_at, robot=pose, ball=ball, robot_id=robot_id, fallen=fallen, goal=goal)
