"""Physical goal fronts are independent of the marker rectangle."""
from copy import deepcopy

import pytest

from fieldcam.camera_model import default_intrinsics
from fieldcam.config import SIM_CONFIG, load_config, set_field_size
from fieldcam.pipeline import Perception
from fieldcam.pitch import Pitch
from fieldcam.sim import SimArena
from final_project.pupper_fc.match_director import MatchDirector


def selected_pitch():
    field = dict(length=3., width=2., goal_width=.6096, ball_radius=.11)
    goals = {"west": dict(x=-1.8, y_min=-.3048, y_max=.3048),
             "east": dict(x=1.8, y_min=-.3048, y_max=.3048)}
    return Pitch.from_field(field, goals)


def test_pitch_keeps_marker_coordinates_and_playable_goal_strip_distinct():
    pitch = selected_pitch()
    assert Pitch.read(pitch.record()) == pitch
    assert not pitch.marker_bounds.contains(1.65, 0.)
    assert pitch.playable_bounds.contains(1.65, 0.)
    assert pitch.ball_search_bounds.contains(1.8 + 2 * .11, 0.)
    bad = pitch.record()
    bad["playable_bounds"]["x_max"] = 1.5
    with pytest.raises(ValueError, match="end lines"):
        Pitch.read(bad)


def test_rendered_ball_beyond_markers_crosses_surveyed_front_and_scores_once():
    pitch = selected_pitch()
    cfg = load_config(SIM_CONFIG)
    cfg["camera"]["intrinsics"] = "config/_none_.yaml"
    cfg["ball"]["radius"] = pitch.ball_radius
    cfg["field"]["goal_width"] = pitch.goal_width
    set_field_size(cfg, 3., 2.)
    cfg["pitch"] = pitch.record()
    size = (1920, 1080)
    K, _ = default_intrinsics(*size, cfg["camera"]["hfov_deg"])
    arena = SimArena(cfg, K, cam_pos=(0., -.2, 3.5), size=size, noise_sigma=0.)
    perception = Perception(cfg, *size)
    assert perception.calibrate(arena.render({}, None))
    director = MatchDirector("geometry-v2", {"striker": "pupper-1"},
                             {"pupper-1": 1, "pupper-2": -1}, pitch=pitch.record())
    robot = {"striker": (-.6, -.3, .2)}
    saw_strip = False
    for k in range(12):
        capture = 100. + .05 * k
        x = min(1.95, 1.40 + .05 * k)
        frame = perception.process(arena.render(robot, (x, 0.)), capture, capture + .08)
        assert frame["ball"]["measured_this_frame"], (k, x, frame["ball"])
        result = director.update(frame, now=frame["stamp"])
        if 1.55 <= x <= 1.65:
            assert result["phase"] == "playing"
            saw_strip = True
    assert saw_strip and result["phase"] == "goal"
    assert result["score"] == {"pupper-1": 1, "pupper-2": 0}
    for k in range(3):
        capture += .05
        frame = perception.process(arena.render(robot, (1.95, 0.)), capture, capture + .08)
        assert director.update(frame, now=frame["stamp"])["score"]["pupper-1"] == 1
    changed = deepcopy(frame)
    changed["seq"] += 1
    changed["captured_at"] += .05
    changed["pitch"]["goals"]["east"]["x"] += .1
    director.update(changed, now=changed["captured_at"] + .08)
    assert director.calibration_changed
