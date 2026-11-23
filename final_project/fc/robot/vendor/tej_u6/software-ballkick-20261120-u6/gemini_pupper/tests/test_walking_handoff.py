"""A structurally valid velocity policy must not change the trick's handoff pose."""
import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

ROBOT = Path(__file__).resolve().parents[1] / 'robot'
sys.path.insert(0, str(ROBOT))
from tricks import ROBOT_JOINT_NAMES, load_tricks


@pytest.fixture
def pair(tmp_path):
    # Actual exported Spider stance, rounded to the trick export's six decimals.
    pose = [-1.037, 1.076, .709, 1.037, -1.076, -.709,
            -.414, 1.14, -.337, .414, -1.14, .337]
    base = json.loads((ROBOT.parent / 'tricks/sit_stay_shake/policy.json').read_text())
    standing = base['default_joint_pos']
    clip = {key: base[key] for key in ('contract_version', 'joint_lower_limits', 'joint_upper_limits')}
    clip.update(trick={'name': 'spider_walk', 'duration_s': 9 / 50, 'fps': 50, 'frames': 9,
        'clips': [{'name': 'down', 'start': 0, 'end': 2, 'next': 'idle'},
                  {'name': 'idle', 'start': 2, 'end': 7, 'loop': True},
                  {'name': 'up', 'start': 7, 'end': 9}]},
        motion={'joint_pos': [standing, *[pose[:] for _ in range(7)], standing],
                'joint_vel': [[0.] * 12 for _ in range(9)], 'anchor_quat_wxyz': [[1.,0.,0.,0.]] * 9})
    walk = {key: base[key] for key in ('contract_version', 'joint_lower_limits', 'joint_upper_limits')}
    walk.update(default_joint_pos=pose[:], gait_reference={'joint_names': ROBOT_JOINT_NAMES[:]},
                command_clip={'vx': [-.15,.15], 'vy': [-.1,.1], 'wz': [-.6,.6]})
    descriptor = {'type': 'policy', 'policy': 'policy.json', 'end_pose': 'spider',
        'description': 'test fixture', 'exit_clip': 'up', 'actions': {'up': 'stand'},
        'walk': {'policy': 'walk_policy.json', 'speed': .04, 'max_forward': .08,
                 'max_sideways': .04, 'turn': 15., 'run': False}}
    folder = tmp_path / 'tricks/spider_walk'
    folder.mkdir(parents=True)
    def save():
        (folder / 'policy.json').write_text(json.dumps(clip))
        (folder / 'walk_policy.json').write_text(json.dumps(walk))
        (folder / 'trick.yaml').write_text(yaml.safe_dump(descriptor))
    save()
    return folder, clip, walk, descriptor, save, base


def test_matching_pose_accepts_export_rounding_without_changing_seated_catalog(pair):
    folder, clip, walk, descriptor, save, _ = pair
    walk['default_joint_pos'][0] = -1.0369999408721924  # actual float32 walking export
    save()
    assert load_tricks(folder.parent)['spider_walk'].walks
    # The new check applies only to dual-policy walking tricks.
    assert load_tricks()['sit_stay_shake'].exit_clip == 'stand_up'


@pytest.mark.parametrize('index,label', [(1, 'entry endpoint'), (4, 'idle loop'), (7, 'exit start')])
def test_each_handoff_boundary_and_idle_interior_must_match(pair, index, label):
    folder, clip, walk, descriptor, save, _ = pair
    clip['motion']['joint_pos'][index][0] += 2e-6
    save()
    with pytest.raises(ValueError, match=label):
        load_tricks(folder.parent)


@pytest.mark.parametrize('fault', ['wrong_walking_order', 'unnamed_walking_order', 'explicit_wrong_clip_order',
                                   'limits', 'old_contract', 'entry_cycle'])
def test_joint_order_bounds_and_reachable_idle_are_required(pair, fault):
    folder, clip, walk, descriptor, save, _ = pair
    if fault == 'wrong_walking_order':
        walk['gait_reference']['joint_names'][0:2] = list(reversed(ROBOT_JOINT_NAMES[0:2]))
    elif fault == 'unnamed_walking_order':
        del walk['gait_reference']['joint_names']
    elif fault == 'explicit_wrong_clip_order':
        clip['motion']['joint_names'] = list(reversed(ROBOT_JOINT_NAMES))
    elif fault == 'limits':
        walk['joint_lower_limits'] = walk['joint_lower_limits'][:]
        walk['joint_lower_limits'][0] += .01
    elif fault == 'old_contract':
        clip['contract_version'] = 1
    else:
        clip['trick']['clips'][0]['next'] = 'down'
    save()
    with pytest.raises(ValueError, match='incompatible walking handoff'):
        load_tricks(folder.parent)


def test_real_upright_policy_cannot_replace_spider_pair_and_preserves_previous_install(pair, tmp_path, monkeypatch):
    import get_trick
    folder, clip, walk, descriptor, save, base = pair
    source = tmp_path / 'clip-export.json'
    exported = copy.deepcopy(base)
    exported.update(clip)
    source.write_text(json.dumps(exported))
    upright = ROBOT.parents[1] / 'pupper_gait_deploy/policies/default_policy.json'
    limits = tmp_path / 'limits.json'
    limits.write_text(json.dumps({'evaluation_source': 'test fixture; never hardware',
        'walk_policy_sha256': hashlib.sha256(upright.read_bytes()).hexdigest(),
        **{k: descriptor['walk'][k] for k in ('speed','max_forward','max_sideways','turn')}}))
    previous = {p.name: p.read_bytes() for p in folder.iterdir()}
    monkeypatch.setattr(get_trick, 'TRICK_DIR', folder.parent)
    monkeypatch.setattr(sys, 'argv', ['get_trick.py', '--name', 'spider_walk', '--policy-file', str(source),
        '--walk-policy-file', str(upright), '--walk-limits', str(limits)])
    # Both exports pass the real native validator; rejection is the pair contract.
    with pytest.raises(ValueError, match='incompatible walking handoff'):
        get_trick.main()
    assert {p.name: p.read_bytes() for p in folder.iterdir()} == previous
    assert not (folder.parent / '.backups').exists()
