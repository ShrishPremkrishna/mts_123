import numpy as np

from mjlab.scripts.pupper_unimate_candidates import candidate_assessment
from mjlab.tasks.tracking.config.pupper import motion_design as md


def sample(moving=True):
  motion = md.Motion(fps=50, root_pos=np.repeat([[0, 0, .161]], 11, axis=0),
                     root_quat=np.repeat([[1, 0, 0, 0]], 11, axis=0),
                     joint_pos=np.repeat([md.DEFAULT_POSE], 11, axis=0))
  if moving:
    motion.joint_pos[:, 0] += np.linspace(0, .2, 11)
  motion.report = {"geometry_issues": []}
  return motion


def score(**overrides):
  return {"reference_resets": 0, "peak_body_tilt_from_upright_deg": 10,
          "rms_position_error_m": .15, "mean_orientation_error_deg": 6,
          "fraction_frames_without_foot_contact": 0, **overrides}


def test_tracking_drift_alone_is_not_a_reason_to_discard_rl_candidate():
  report = candidate_assessment(sample(), score())
  assert report["candidate_for_rl"]
  assert not report["trained"] and not report["approved_for_hardware"]


def test_still_motion_and_toppling_are_not_nominated_as_standing_dance():
  assert not candidate_assessment(sample(False), score())["candidate_for_rl"]
  report = candidate_assessment(sample(), score(peak_body_tilt_from_upright_deg=70))
  assert not report["candidate_for_rl"]
  assert "60 degrees" in report["reasons"][0]


def test_geometry_error_and_reference_teleport_remain_disqualifying():
  motion = sample()
  motion.report["geometry_issues"] = ["joint angles exceed model limits"]
  report = candidate_assessment(motion, score(reference_resets=1))
  assert len(report["reasons"]) == 2
  assert not report["candidate_for_rl"]
