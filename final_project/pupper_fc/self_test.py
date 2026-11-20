"""Run the installed laptop's perception and kickoff logic on a synthetic field."""
import json

from final_project.overhead_perception.fieldcam.config import SIM_CONFIG, load_config
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
    return {"ok": True, "summary": "Synthetic field passed. Six floor tags, both robots, the ball and kickoff verified."}


if __name__ == "__main__":
    print(json.dumps(check()))
