"""The automatic repair must preserve an easy case and expose impossible ones."""

import numpy as np
import pytest

from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper.unimate.repair import geometry_issues, repair_motion


def standing():
  return md.Motion(
    fps=50.0,
    root_pos=np.repeat([[0.0, 0.0, md.STAND_HEIGHT]], 11, axis=0),
    root_quat=np.repeat([[1.0, 0.0, 0.0, 0.0]], 11, axis=0),
    joint_pos=np.repeat([md.DEFAULT_POSE], 11, axis=0),
  )


def test_repair_adds_standing_bookends_and_preserves_original():
  original = standing()
  before = original.root_pos.copy()
  repaired = repair_motion(original, transition_s=0.5)
  assert repaired.duration >= 1.2
  assert repaired.report["geometry_pass"]
  assert not repaired.report["physics_validated"]
  assert not repaired.report["approved_for_hardware"]
  np.testing.assert_array_equal(before, original.root_pos)
  # md.stand pins feet to the floor; its IK solution differs slightly from the
  # raw nominal motor angles, whose foot-centre height was rounded in the model.
  grounded_stand = md.build_motion([md.stand(0), md.stand(1)])
  np.testing.assert_allclose(repaired.joint_pos[[0, -1]], grounded_stand.joint_pos[[0, -1]], atol=1e-5)
  assert all(n == 11 for n in repaired.report["anchored_frames"].values())


def test_geometry_rejects_ground_penetration_and_joint_overshoot():
  broken = standing()
  broken.root_pos[:, 2] -= 0.05
  broken.joint_pos[:, 0] = md.kinematics().upper[0] + 0.1
  issues = geometry_issues(broken)
  assert any("ground" in s for s in issues)
  assert any("joint angles" in s for s in issues)


@pytest.mark.parametrize("kwargs", [{"transition_s": 0}, {"contact_height_m": -1},
                                    {"release_height_m": 0.001}, {"max_joint_speed_rad_s": 21}])
def test_repair_rejects_unsafe_or_nonsensical_options(kwargs):
  with pytest.raises(ValueError):
    repair_motion(standing(), **kwargs)
