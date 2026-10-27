"""Held-out reference preparation and the generated-file training preflight."""

import json
import numpy as np
import pytest

from mjlab.scripts.pupper_foundation import check_generated_motion, prepare_stress
from mjlab.tasks.tracking.config.pupper import motion_design as md


@pytest.mark.parametrize("profile,idle", [("sit_stay_shake", "sit_idle"), ("spider_clips", "spider_idle")])
def test_stress_reference_contains_real_extended_idle_not_restarted_loops(profile, idle, tmp_path):
  path = prepare_stress(profile, tmp_path, idle_seconds=15)
  clips = {clip["name"]: clip for clip in md.read_clips(path)}
  clip = clips[idle]
  assert clip["end"] - clip["start"] == 751
  with np.load(path, allow_pickle=False) as data:
    assert len(data["joint_pos"]) > 2000
    held = data["joint_pos"][clip["start"] : clip["end"]]
    # The shared boundary frame may differ by small IK rounding; subsequent
    # frames are exactly the stationary pose the policy must hold continuously.
    np.testing.assert_allclose(held[1:], np.repeat(held[1:2], len(held)-1, axis=0), atol=1e-7)


def test_training_preflight_reads_actual_motion_not_only_sidecar(tmp_path):
  motion = md.build_motion([md.stand(0), md.stand(0.2)])
  good = motion.save_npz(tmp_path / "good.npz")
  check_generated_motion(good)
  with np.load(good, allow_pickle=False) as source:
    arrays = {name: source[name].copy() for name in source.files}
  arrays["joint_pos"][:, 0] = 99
  broken = tmp_path / "broken.npz"
  np.savez(broken, **arrays)
  with pytest.raises(ValueError, match="joint angles"):
    check_generated_motion(broken)


@pytest.mark.parametrize("loops,expected_resets", [(1, 0), (2, 1)])
def test_physics_score_counts_only_actual_whole_reference_wraps(tmp_path, monkeypatch, loops, expected_resets):
  import torch
  from mjlab.tasks.tracking.config.pupper.play_motion import main

  monkeypatch.setattr(torch.cuda, "is_available", lambda: False)
  path = md.build_motion([md.stand(0), md.stand(0.2)]).save_npz(tmp_path / "stand.npz")
  output = tmp_path / "score.json"
  main(path, loops=loops, report_file=output)
  report = json.loads(output.read_text())
  assert report["reference_resets"] == expected_resets
  assert len(report["samples"]) == 11 * loops - 1
  assert report["samples"][0]["reference_frame"] == 1
  assert report["samples"][-1]["reference_frame"] == 10
  assert report["peak_motor_torque_nm"] >= 0
  assert 0 <= report["peak_body_tilt_from_upright_deg"] <= 180
  assert 0 <= report["fraction_frames_without_foot_contact"] <= 1
  assert all(0 <= sample["foot_contact_count"] <= 4 for sample in report["samples"])
  assert not report["approved_for_hardware"]


def test_video_uses_recorded_physics_pose_and_aligned_reference(tmp_path, monkeypatch):
  import torch
  import imageio.v3 as iio
  from mjlab.tasks.tracking.config.pupper import unimate as U
  from mjlab.tasks.tracking.config.pupper.play_motion import main

  monkeypatch.setattr(torch.cuda, "is_available", lambda: False)
  captured = {}
  def render(fits, **kwargs):
    captured["fits"] = fits
    assert not kwargs["skeleton"]
    return np.zeros((10, 2, 4, 3), dtype=np.uint8)
  monkeypatch.setattr(U, "render", render)
  monkeypatch.setattr(iio, "imwrite", lambda path, frames, fps: captured.update(video=str(path), fps=fps))
  path = md.build_motion([md.stand(0), md.stand(.2)]).save_npz(tmp_path / "stand.npz")
  main(path, video_file=tmp_path / "video.mp4")
  reference, actual = [fit.motion for fit in captured["fits"]]
  assert len(reference.joint_pos) == len(actual.joint_pos) == 10
  assert not np.array_equal(reference.joint_pos, actual.joint_pos)
  assert captured["fps"] == 50
