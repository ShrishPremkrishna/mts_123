"""Conservative cleanup of a generated motion before physics replay.

This is a starting repair, not a claim that arbitrary text motions are feasible.
It preserves the original outside this function, anchors near-ground feet during
contact, solves IK again, adds standing entry/exit, and slows excessive motion.
Read the report and inspect physics before accepting a sample for training.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper.motion_ops import validate


def geometry_issues(motion: md.Motion) -> list[str]:
  """Actual violations; touching a model joint limit is reported, not conflated
  with exceeding it. Contact dynamics and stability require physics evaluation."""
  validate(motion)
  k = md.kinematics()
  report = md.check_motion(motion)
  issues = []
  if np.any(motion.joint_pos < k.lower - 1e-6) or np.any(motion.joint_pos > k.upper + 1e-6):
    issues.append("joint angles exceed model limits")
  if min(report["lowest_foot_bottom_m"], report["lowest_body_point_m"]) < -0.001:
    issues.append("reference penetrates the ground by more than 1 mm")
  if report["max_joint_speed_rad_s"] > 15:
    issues.append("reference joint speed exceeds the 15 rad/s preparation limit")
  if motion.report.get("max_ik_error_m", 0) > 0.002:
    issues.append("a planted foot target is unreachable by more than 2 mm")
  return issues


def repair_motion(
  motion: md.Motion,
  transition_s: float = 1.5,
  contact_height_m: float = 0.012,
  release_height_m: float = 0.025,
  max_joint_speed_rad_s: float = 12.0,
) -> md.Motion:
  validate(motion)
  if len(motion.root_pos) < 2:
    raise ValueError("repair needs at least two frames")
  if (not np.isfinite([transition_s, contact_height_m, release_height_m, max_joint_speed_rad_s]).all()
      or transition_s <= 0 or contact_height_m < 0 or release_height_m <= contact_height_m
      or not 0 < max_joint_speed_rad_s <= 15):
    raise ValueError("invalid transition, contact hysteresis, or speed settings")
  k = md.kinematics()
  roots = motion.root_pos.copy()
  roots[:, :2] -= roots[0, :2]
  angles = Rotation.from_quat(motion.root_quat[:, [1, 2, 3, 0]]).as_euler("xyz", degrees=True)
  # Euler coordinates wrap at +/-180 degrees even for a small physical turn.
  # build_motion interpolates these coordinates, so remove representation jumps
  # before creating keyframes rather than introducing an artificial full spin.
  angles = np.rad2deg(np.unwrap(np.deg2rad(angles), axis=0))
  keys = [md.stand(0.0, yaw=float(angles[0, 2]))]
  locked = {}
  anchored_frames = {leg: 0 for leg in md.LEGS}
  for i in range(len(roots)):
    feet = k.feet(roots[i], motion.root_quat[i], motion.joint_pos[i])
    targets = {}
    for leg, foot in feet.items():
      clearance = foot[2] - md.FOOT_RADIUS
      if leg in locked and clearance > release_height_m:
        del locked[leg]
      if leg not in locked and clearance <= contact_height_m:
        locked[leg] = np.array([foot[0], foot[1], md.FOOT_RADIUS])
      targets[leg] = tuple(locked.get(leg, foot))
      anchored_frames[leg] += int(leg in locked)
    keys.append(md.Keyframe(transition_s + i / motion.fps, tuple(roots[i]), tuple(angles[i]), feet=targets))
  keys.append(md.stand(2 * transition_s + motion.duration,
                       x=float(roots[-1, 0]), y=float(roots[-1, 1]), yaw=float(angles[-1, 2])))
  repaired = md.build_motion(keys, fps=motion.fps, keep_above_ground=True)
  slowdown = max(1.0, repaired.report["max_joint_speed_rad_s"] / max_joint_speed_rad_s * 1.05)
  if slowdown > 1.0:
    for key in keys:
      key.t *= slowdown
    repaired = md.build_motion(keys, fps=motion.fps, keep_above_ground=True)
  repaired.report.update({
    "repair": "standing bookends, contact hysteresis/anchoring, IK, ground clearance, uniform slowdown",
    "slowdown": slowdown, "anchored_frames": anchored_frames,
    "physics_validated": False, "approved_for_hardware": False,
  })
  repaired.report["geometry_issues"] = geometry_issues(repaired)
  repaired.report["geometry_pass"] = not repaired.report["geometry_issues"]
  return repaired
