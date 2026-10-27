"""Foundation invariants: actual motion geometry, transitions, and bad inputs."""

from __future__ import annotations

import json

import numpy as np
import pytest

from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import SPIDER_HEIGHT, SPIDER_POSE
from mjlab.scripts.pupper_foundation import load_profile
from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper import motion_ops as ops
from mjlab.tasks.tracking.config.pupper import unimate as U
from mjlab.tasks.tracking.config.pupper.tricks import sit_stay_shake, spider


def motion(frames=11):
  return md.Motion(
    fps=50.0,
    root_pos=np.repeat([[0.0, 0.0, md.STAND_HEIGHT]], frames, axis=0),
    root_quat=np.repeat([[1.0, 0.0, 0.0, 0.0]], frames, axis=0),
    joint_pos=np.repeat([md.DEFAULT_POSE], frames, axis=0),
  )


def test_cut_and_hold_round_times_keep_shared_boundary_without_aliasing():
  source = motion()
  a, b = ops.cut(source, 0, 0.061), ops.cut(source, 0.061, 0.2)
  assert len(a.root_pos) == 4 and len(a.root_pos) + len(b.root_pos) - 1 == 11
  np.testing.assert_array_equal(a.joint_pos[-1], b.joint_pos[0])
  a.joint_pos[0, 0] = 99
  assert source.joint_pos[0, 0] != 99
  held = ops.hold(source, 0.061, 0.101)
  assert len(held.root_pos) == 6
  held.root_pos[0, 0] = 9
  assert held.root_pos[1, 0] == source.root_pos[3, 0] == 0


@pytest.mark.parametrize("start,end", [(-0.01, 0.1), (0.1, 0.3), (0.1, 0.0), (0, float("nan"))])
def test_cut_rejects_invalid_time_instead_of_numpy_negative_slicing(start, end):
  with pytest.raises(ValueError):
    ops.cut(motion(), start, end)


@pytest.mark.parametrize("duration", [-1, 0, 0.001, float("nan"), float("inf")])
def test_hold_rejects_invalid_duration(duration):
  with pytest.raises(ValueError):
    ops.hold(motion(), 0, duration)


def test_mirror_reflects_world_foot_geometry_not_only_joint_array():
  source = motion(2)
  source.root_pos[:, 1] = 0.04
  source.root_quat[:] = md.rpy_to_quat((8, -12, 15))
  source.joint_pos[:, :3] += np.array([0.04, 0.1, -0.06])
  reflected = ops.mirror(source)
  k = md.kinematics()
  original_feet = k.feet(source.root_pos[0], source.root_quat[0], source.joint_pos[0])
  mirrored_feet = k.feet(reflected.root_pos[0], reflected.root_quat[0], reflected.joint_pos[0])
  for leg in md.LEGS:
    opposite = leg[:-1] + ("l" if leg.endswith("r") else "r")
    # The XML stores mirrored rotations with finite decimal precision.
    np.testing.assert_allclose(mirrored_feet[opposite], original_feet[leg] * [1, -1, 1], atol=1e-6)
  restored = ops.mirror(reflected)
  for name in ("root_pos", "root_quat", "joint_pos"):
    np.testing.assert_allclose(getattr(restored, name), getattr(source, name))


def test_reverse_copies_all_arrays_and_preserves_duration():
  source = motion()
  source.root_pos[:, 0] = np.arange(11) / 100
  reversed_motion = ops.reverse(source)
  assert reversed_motion.duration == source.duration
  np.testing.assert_array_equal(reversed_motion.root_pos[0], source.root_pos[-1])
  reversed_motion.root_pos[0, 0] = 99
  assert source.root_pos[-1, 0] != 99


@pytest.mark.parametrize("field", ["root_pos", "root_quat", "joint_pos"])
def test_motion_helpers_reject_nonfinite_data(field):
  source = motion()
  getattr(source, field)[0, 0] = float("nan")
  with pytest.raises(ValueError):
    ops.reverse(source)


def test_clip_library_rejects_missing_names_empty_order_and_unknown_next():
  with pytest.raises(ValueError):
    md.concat_clips([])
  with pytest.raises(ValueError):
    md.concat_clips([md.Clip("idle", motion())], order=[])
  with pytest.raises(ValueError):
    md.concat_clips([md.Clip("idle", motion()), md.Clip("idle", motion())])
  with pytest.raises(ValueError):
    md.concat_clips([md.Clip("idle", motion(), next="missing")])


def test_clip_library_rejects_root_teleport_and_broken_idle_loop():
  shifted = motion()
  shifted.root_pos[:, 0] = 0.02
  with pytest.raises(ValueError, match="root discontinuity"):
    md.concat_clips([md.Clip("a", motion()), md.Clip("b", shifted)])
  bad_loop = motion()
  bad_loop.joint_pos[-1, 0] += 0.1
  with pytest.raises(ValueError, match="rad away"):
    md.concat_clips([md.Clip("idle", bad_loop, loop=True)])


@pytest.mark.parametrize("module", [sit_stay_shake, spider])
def test_real_interactive_libraries_cover_edges_and_export_compatible_motion(module, tmp_path):
  clips = module.clips()
  library = md.concat_clips(clips, order=module.ORDER)
  transitions = set(zip(module.ORDER, module.ORDER[1:]))
  for clip in clips:
    if clip.next:
      assert (clip.name, clip.next) in transitions
    if clip.loop:
      assert (clip.name, clip.name) in transitions
  for clip in clips:
    if clip.name.startswith("shake"):
      assert ("sit_idle", clip.name) in transitions
  path = library.save_npz(tmp_path / "library.npz")
  with np.load(path, allow_pickle=False) as saved:
    assert np.isfinite(saved["joint_vel"]).all()
    assert saved["joint_pos"].shape[1] == 12
  assert md.read_clips(path) == library.clips
  k = md.kinematics()
  assert np.all(library.joint_pos >= k.lower - 1e-7)
  assert np.all(library.joint_pos <= k.upper + 1e-7)
  assert library.report["lowest_body_point_m"] >= -0.001
  assert library.report["lowest_foot_bottom_m"] >= -0.001
  assert library.report["max_joint_speed_rad_s"] < 15


def test_spider_handoff_has_exact_walking_pose_and_height():
  clips = {c.name: c.motion for c in spider.clips()}
  for name, frame in (("spider_down", -1), ("spider_idle", 0), ("spider_up", 0)):
    np.testing.assert_allclose(clips[name].joint_pos[frame], SPIDER_POSE, atol=1e-8)
    assert clips[name].root_pos[frame, 2] == pytest.approx(SPIDER_HEIGHT, abs=1e-8)


def test_spider_profile_does_not_penalize_its_required_splay():
  profile = load_profile("spider_walk")
  assert profile["rewards"].get("abduction_angle", 0) == 0
  assert profile["rewards"]["base_height"] > 0
  assert profile["rewards"]["gait_tracking"] > 0
  assert profile["training_command_bounds"]["forward_m_s"] == 0.15
  json.dumps(profile, allow_nan=False)


@pytest.mark.parametrize("kwargs", [
  {"overlap": 60}, {"overlap": -1}, {"slow": 0}, {"slow": float("nan")},
  {"stride": -1}, {"trim_start": -1}, {"hold": -1}, {"durations": [0]},
])
def test_unimate_rejects_invalid_fit_options_before_ik(kwargs):
  raw = np.repeat(np.asarray(U.SKELETON["rest"])[None], 60, axis=0)
  with pytest.raises(ValueError):
    U.fit_pupper(raw, **kwargs)


def test_unimate_rejects_degenerate_and_nonfinite_skeleton():
  with pytest.raises(ValueError, match="coincident"):
    U.fit_pupper(np.zeros((60, 19, 3)))
  with pytest.raises(ValueError, match="finite"):
    U.fit_pupper(np.full((60, 19, 3), float("nan")))


@pytest.mark.parametrize("prompts,options", [([], {}), ([" "], {}), (["bow"], {"n": 0}),
                                            (["bow"], {"overlap": 60}), (["bow"], {"cfg": float("nan")})])
def test_unimate_rejects_invalid_generation_before_calling_worker(prompts, options):
  worker = U.UniMate.__new__(U.UniMate)
  with pytest.raises(ValueError):
    worker.generate(prompts, **options)
