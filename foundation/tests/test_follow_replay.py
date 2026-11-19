"""Closed-loop follower scenes on a fake world; no ROS, camera or Hailo required.

These check decision logic through the real state machine. The camera geometry
in follow_replay.py is assumed, so they establish no physical distance or speed.
"""
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
for relative in ('follow_me_pupper/lab', 'follow_me_pupper/robot'):
    sys.path.insert(0, str(ROOT / relative))
import follow_replay as replay
import target_lock
from lab_checks import FakeDetection

SEEDS = range(5)
SCENES = {scene.name: scene for scene in replay.scenes()}
OFF, ON = False, True


@pytest.fixture(autouse=True)
def default_color_setting():
    yield
    target_lock.USE_COLOR = False


def runs(name, color):
    return [replay.run_scene(SCENES[name], color, seed) for seed in SEEDS]


@pytest.mark.parametrize('name', ['approach_ahead', 'approach_off_axis', 'walk_away', 'short_occlusion',
                                  'background_walker', 'side_by_side', 'stranger_in_front', 'legs_hidden',
                                  'corrupt_frames', 'slow_detector', 'dropouts'])
@pytest.mark.parametrize('color', [OFF, ON])
def test_following_survives_what_should_not_stop_it(name, color):
    for score in runs(name, color):
        assert score['first_lock'] == 'me'
        assert score['wrong_s'] == 0
        assert score['final_state'] == 'TRACK' and score['final_locked_on'] == 'me'


@pytest.mark.parametrize('name', ['long_occlusion_bystander', 'gone_then_stranger', 'crossing_behind',
                                  'crossing_same_color', 'stranger_same_color', 'long_occlusion_alone'])
def test_without_color_a_lost_target_is_never_replaced_by_a_guess(name):
    for score in runs(name, OFF):
        assert score['wrong_s'] < 1.0  # a crossing can slip for a few frames before it stops
        assert score['final_state'] == 'LOST'
        assert score['final_cmd'] == (0, 0)


@pytest.mark.parametrize('name', ['long_occlusion_alone', 'long_occlusion_bystander', 'crossing', 'crossing_behind'])
def test_color_keeps_or_finds_the_target(name):
    for score in runs(name, ON):
        assert score['wrong_s'] == 0
        assert score['final_state'] == 'TRACK' and score['final_locked_on'] == 'me'


def test_same_color_crossing_stops_instead_of_guessing():
    for score in runs('crossing_same_color', ON):
        assert score['wrong_s'] < 1.0
        assert score['final_state'] == 'LOST'


def test_known_limit_color_cannot_reject_a_stranger_in_the_same_color():
    # Recorded so that this limit stays visible; it is not a goal.
    for score in runs('stranger_same_color', ON):
        assert score['final_locked_on'] == 'other'


@pytest.mark.parametrize('name', ['gone_forever_alone', 'gone_then_stranger'])
def test_search_ends_after_one_turn_and_the_robot_stands_still(name):
    for score in runs(name, ON):
        assert score['final_state'] == 'LOST'
        assert score['spin_longest_s'] <= replay.fm.SEARCH_MAX_TIME + 2 * replay.TICK_DT
        assert score['final_cmd'] == (0, 0)


def test_lost_camera_stops_the_walk_and_color_resumes_it():
    assert all(s['walked_blind_m'] < 0.05 for s in runs('camera_freeze', ON))
    assert all(s['final_state'] == 'TRACK' for s in runs('camera_freeze', ON))
    assert all(s['final_state'] == 'LOST' for s in runs('camera_freeze', OFF))


@pytest.mark.parametrize('name', list(SCENES))
def test_nobody_is_walked_into(name):
    # One tick: a person can step into the zone between two commands.
    assert all(s['pushed_person_s'] <= 3 * replay.TICK_DT for s in runs(name, OFF))


def test_a_person_too_wide_for_the_target_cannot_take_the_lock():
    lock = target_lock.Lock()
    target_lock.remember(lock, FakeDetection(300, 200, 360, 300, class_id=0))
    near = FakeDetection(260, 100, 400, 420, class_id=0)
    same = FakeDetection(305, 205, 365, 305, class_id=0)
    assert target_lock.plausible_size([near, same], lock) == [same]
    legs_hidden = FakeDetection(305, 205, 365, 250, class_id=0)
    assert target_lock.plausible_size([legs_hidden], lock) == [legs_hidden]


def test_recorded_log_replays_through_the_state_machine(tmp_path):
    log = tmp_path / 'run.jsonl'
    with log.open('w') as out:
        for i in range(30):
            out.write(json.dumps({'t': i * 0.1, 'frame': [[350.0, 250.0, 60.0, 200.0, 0, 0.8, '200,40,40']]}) + '\n')
        for i in range(30, 60):
            out.write(json.dumps({'t': i * 0.1, 'frame': []}) + '\n')
        out.write(json.dumps({'t': 6.0, 'tick': ['LOST', 0.0, 350.0]}) + '\n')
    result = replay.replay_log(str(log), use_color=False)
    assert result['frames'] == 60 and result['frames_without_target'] == 30
    assert result['jumps'] == [] and result['final_state'] == 'LOST'
