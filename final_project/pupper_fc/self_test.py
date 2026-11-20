"""Run the installed laptop's perception and kickoff logic on a synthetic field."""
import json
import math

from final_project.overhead_perception.fieldcam.config import SIM_CONFIG, load_config, set_field_size
from final_project.overhead_perception.fieldcam.corner_anchor import anchor_geometry
from final_project.overhead_perception.fieldcam.pipeline import Perception
from final_project.overhead_perception.scripts.run_perception import SimSource
from .match_director import MatchDirector


def check():
    cfg = load_config(SIM_CONFIG)
    size = (1280, 800)
    source = SimSource(cfg, size)
    perception = Perception(cfg, *size)
    tracks = {"striker": "pupper-1", "keeper": "pupper-2"}
    director = MatchDirector("offline-check", tracks, {"pupper-1": -1, "pupper-2": 1}, pending_start=True)
    empty, _, _ = source.read()
    if not perception.calibrate(empty):
        raise ValueError("Could not calibrate the synthetic field")
    for expected in ("pending", "playing"):
        image, captured, _ = source.read()
        frame = perception.process(image, captured, captured, match_only=True)
        if (not set(cfg["tags"]["field"]).issubset(perception.debug["tags"])
                or not all(frame["robots"][track]["measured_this_frame"] for track in tracks)
                or not frame["ball"]["measured_this_frame"]):
            raise ValueError("Missing a field tag, robot or ball in the synthetic image")
        state = director.update(frame, now=captured)
        if state["phase"] != expected:
            raise ValueError(f"Expected {expected} at kickoff, received {state['phase']}")
        if expected == "pending":
            director.commit_start(now=captured)
    anchor = dict(mode="corner", corner="west_north", tag_id=0, length=2.6, width=1.6,
                  inset_x=.15, inset_y=.15, rotation=30., tag_size=.40, goal_width=.6)
    cfg = load_config(SIM_CONFIG)
    set_field_size(cfg, anchor["length"], anchor["width"])
    cfg.update(field_setup=anchor, robot_tracking="colors")
    cfg["tags"]["field"] = {0: anchor_geometry(anchor).mean(axis=0)}
    source = SimSource(cfg, size)
    source.robots = {"striker": [-.65, .25, -.65], "keeper": [.65, -.25, 2.50]}
    perception = Perception(cfg, *size)
    empty, _, _ = source.read()
    if not perception.calibrate(empty):
        raise ValueError(perception.calib_fail or "Could not calibrate the single simulated corner")
    image, captured, _ = source.read()
    state = perception.process(image, captured, captured, match_only=True)
    for name, (x, y, yaw) in source.robots.items():
        rb = state["robots"][name]
        if (not rb["measured_this_frame"] or math.hypot(rb["x"]-x, rb["y"]-y) > .04
                or abs(math.remainder(rb["yaw"]-yaw, 2*math.pi)) > math.radians(5)):
            raise ValueError("Team patch position or signed nose heading failed for " + name)
    return {"ok": True, "summary": "Synthetic field passed. Six floor tags, both robots, the ball and kickoff verified. "
            "One corner tag and both teams’ colour-patch positions and signed headings also passed."}


if __name__ == "__main__":
    print(json.dumps(check()))
