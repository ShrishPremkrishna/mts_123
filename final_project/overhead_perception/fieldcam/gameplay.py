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
from .observations import capture_measurement, finite_number

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
    pose, ball = None, None
    captured_at = math.nan
    try:
        capture = finite_number(state.get("captured_at"), "capture time")
        age = finite_number(state.get("stamp"), "state time") - capture
        captured_at = finite_number(capture + finite_number(clock_offset, "clock offset"), "mapped capture")
    except ValueError:
        return m.Observation(captured_at=captured_at, robot=None, ball=None, robot_id=robot_id, fallen=fallen, goal=goal)

    def admitted(row, heading=False):
        if state.get("calibration_valid") is not True:
            return None
        try:
            return capture_measurement(row, capture, age, heading=heading,
                                       tracking=state.get("tracking", "tags"))
        except ValueError:
            return None

    robots = state.get("robots")
    r = admitted(robots.get(robot) if isinstance(robots, dict) else None, heading=True)
    if r is not None:
        pose = m.Pose(r["x"], r["y"], r["yaw"])
    b = admitted(state.get("ball"))
    if b is not None:
        ball = m.Ball(**b)
    return m.Observation(captured_at=captured_at, robot=pose, ball=ball, robot_id=robot_id, fallen=fallen, goal=goal)
