"""Frame-preserving operations shared by the foundation's interactive tricks.

Times are rounded to the nearest frame, with both boundary frames included.
Operations return independent arrays and discard clip tables/reports, whose frame
indices and statistics would be stale after a transformation.
"""

from __future__ import annotations

import math

import numpy as np

from mjlab.tasks.tracking.config.pupper import motion_design as md


def validate(motion: md.Motion) -> None:
  if not math.isfinite(motion.fps) or motion.fps <= 0:
    raise ValueError("motion fps must be finite and positive")
  count = len(motion.root_pos)
  if count == 0:
    raise ValueError("a motion must contain at least one frame")
  for name, width in (("root_pos", 3), ("root_quat", 4), ("joint_pos", 12)):
    values = getattr(motion, name)
    if values.shape != (count, width) or not np.isfinite(values).all():
      raise ValueError(f"{name} must be finite with shape ({count}, {width})")
  if not np.allclose(np.linalg.norm(motion.root_quat, axis=1), 1.0, atol=1e-4):
    raise ValueError("root quaternions must be normalized")


def _frame(motion: md.Motion, time_s: float) -> int:
  if not math.isfinite(time_s) or not 0 <= time_s <= motion.duration + 1e-9:
    raise ValueError(f"time must be within [0, {motion.duration}] seconds")
  return round(time_s * motion.fps)


def cut(motion: md.Motion, start_s: float, end_s: float) -> md.Motion:
  validate(motion)
  if start_s > end_s:
    raise ValueError("cut start must not follow its end")
  start, end = _frame(motion, start_s), _frame(motion, end_s)
  return md.Motion(
    fps=motion.fps,
    root_pos=motion.root_pos[start : end + 1].copy(),
    root_quat=motion.root_quat[start : end + 1].copy(),
    joint_pos=motion.joint_pos[start : end + 1].copy(),
  )


def hold(motion: md.Motion, at_s: float, seconds: float) -> md.Motion:
  validate(motion)
  at = _frame(motion, at_s)
  if not math.isfinite(seconds) or seconds <= 0 or round(seconds * motion.fps) < 1:
    raise ValueError("hold duration must span at least one frame interval")
  count = round(seconds * motion.fps) + 1
  return md.Motion(
    fps=motion.fps,
    root_pos=np.repeat(motion.root_pos[at : at + 1], count, axis=0),
    root_quat=np.repeat(motion.root_quat[at : at + 1], count, axis=0),
    joint_pos=np.repeat(motion.joint_pos[at : at + 1], count, axis=0),
  )


def mirror(motion: md.Motion) -> md.Motion:
  validate(motion)
  opposite = [
    md.JOINT_NAMES.index(n.replace("_l_", "_r_") if "_l_" in n else n.replace("_r_", "_l_"))
    for n in md.JOINT_NAMES
  ]
  return md.Motion(
    fps=motion.fps,
    root_pos=motion.root_pos * np.array([1.0, -1.0, 1.0]),
    root_quat=motion.root_quat * np.array([1.0, -1.0, 1.0, -1.0]),
    # All three motor axes are mirrored in the Pupper model, not just abduction.
    joint_pos=-motion.joint_pos[:, opposite],
  )


def reverse(motion: md.Motion) -> md.Motion:
  validate(motion)
  return md.Motion(
    fps=motion.fps,
    root_pos=motion.root_pos[::-1].copy(),
    root_quat=motion.root_quat[::-1].copy(),
    joint_pos=motion.joint_pos[::-1].copy(),
  )
