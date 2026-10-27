"""Malformed trick metadata is rejected before it reaches the controller."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("policy_download_validation", ROOT / "pupper_gait_deploy/download_policy.py")
validation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validation)


def policy():
    components = [("motion_command", 24), ("motion_anchor_ori_b", 6), ("base_ang_vel", 3),
                  ("joint_pos_rel", 12), ("joint_vel_rel", 12), ("last_action", 12)]
    return {"in_shape": [None, 69], "layers": [{"shape": [69, 12]}],
            "observation_history": 1, "single_observation_size": 69,
            "obs_spec": {"frame_order": "newest_first", "components": [{"name": n, "size": s} for n, s in components]},
            "default_joint_pos": [0]*12, "joint_lower_limits": [-2]*12, "joint_upper_limits": [2]*12,
            "trick": {"name": "test", "fps": 50},
            "motion": {"joint_pos": [[0]*12]*2, "joint_vel": [[0]*12]*2,
                       "anchor_quat_wxyz": [[1, 0, 0, 0]]*2}}


def test_standalone_motion_without_explicit_clips_remains_supported(tmp_path):
    path = tmp_path / "policy.json"
    path.write_text(json.dumps(policy()))
    assert validation.validate(path)["trick"]["fps"] == 50


@pytest.mark.parametrize("defect", ["clock", "empty_clips", "empty_motion", "quaternion", "nonfinite", "overflow"])
def test_malformed_motion_rejected(tmp_path, defect):
    data = copy.deepcopy(policy())
    if defect == "clock": data["trick"]["fps"] = 0
    elif defect == "empty_clips": data["trick"]["clips"] = []
    elif defect == "empty_motion": data["motion"] = {key: [] for key in data["motion"]}
    elif defect == "quaternion": data["motion"]["anchor_quat_wxyz"][0] = [0, 0, 0, 0]
    else: data["motion"]["joint_pos"][0][0] = float("nan") if defect == "nonfinite" else 1e300
    path = tmp_path / "policy.json"
    path.write_text(json.dumps(data))
    with pytest.raises(SystemExit): validation.validate(path)
