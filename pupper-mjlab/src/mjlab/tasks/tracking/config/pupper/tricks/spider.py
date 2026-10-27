"""Lab 6, Part 6: the spider, a trick that walks.

The spider trick has two policies. This file builds the first one's clips:
  spider_down  from standing: lie down, splay the legs, wiggle them, then settle
               into the spider-walk stance (torso 1 cm off the ground)
  spider_idle  the stance, still (repeats while nothing else is asked for)
  spider_up    from the stance: wiggle, then push up to standing
On the robot, the walking policy (Mjlab-SpiderWalk-Flat-Pupper-v3, the second
policy) takes the legs between spider_down and spider_up.

You get the recorded spider trick (recorded(): 11.3 s, stand -> lie down ->
wiggle -> lie still) and settle_into_stance(). This completed engineering
version shares tested motion helpers with Part 5. Build the library with:
  uv run python -m mjlab.tasks.tracking.config.pupper.tricks.spider
It checks your code and writes spider_walk.npz, the file you train on.

Motions are the same three arrays as in Part 5 (root_pos, root_quat, joint_pos;
50 frames per second).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import SPIDER_HEIGHT, SPIDER_POSE
from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper.motion_ops import cut, hold, reverse, validate

HERE = Path(__file__).resolve().parent
FPS = 50.0

# In recorded(), the legs are splayed and wiggling from about 3.5 s to 8.5 s.
WIGGLE_UNTIL_S = 7.0  # where spider_down leaves the recording for the stance


# --------------------------------------------------------------------- provided
def recorded() -> md.Motion:
  """The Pupper monorepo's recorded spider trick, at 50 Hz."""
  r = np.load(HERE / "spider_recorded.npz")
  return md.Motion(
    fps=float(r["fps"][0]),
    root_pos=r["root_pos"].copy(),
    root_quat=r["root_quat"].copy(),
    joint_pos=r["joint_pos"].copy(),
  )


def settle_into_stance(motion: md.Motion, seconds: float = 0.6) -> md.Motion:
  """The motion, then a smooth blend from its last frame into the spider-walk
  stance (SPIDER_POSE, body level at SPIDER_HEIGHT): the pose the walking policy
  starts from, so the hand-off between the two policies is seamless."""
  validate(motion)
  if not np.isfinite(seconds) or seconds <= 0 or round(seconds * motion.fps) < 1:
    raise ValueError("stance blend must span at least one frame interval")
  n = round(seconds * motion.fps)
  s = (0.5 - 0.5 * np.cos(np.linspace(0, np.pi, n + 1)[1:]))[:, None]
  p0, q0, j0 = motion.root_pos[-1], motion.root_quat[-1], motion.joint_pos[-1]
  w, x, y, z = q0
  yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
  q1 = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])  # level, same heading
  q1 = q1 if np.dot(q0, q1) >= 0 else -q1
  p1 = np.array([p0[0], p0[1], SPIDER_HEIGHT])
  quat = (1 - s) * q0 + s * q1
  quat /= np.linalg.norm(quat, axis=1, keepdims=True)
  return md.Motion(
    fps=motion.fps,
    root_pos=np.concatenate([motion.root_pos, (1 - s) * p0 + s * p1]),
    root_quat=np.concatenate([motion.root_quat, quat]),
    joint_pos=np.concatenate(
      [motion.joint_pos, (1 - s) * j0 + s * np.array(SPIDER_POSE)]
    ),
  )


# -------------------------------------------------------------------- clip library
def clips() -> list[md.Clip]:
  """Use the exact walking stance at both controller hand-off boundaries."""
  down = settle_into_stance(cut(recorded(), 0.0, WIGGLE_UNTIL_S))
  # The hardware recording has small encoder overshoots (up to ~0.016 rad)
  # beyond model limits and ~2 mm floor penetration. Keep recorded() untouched;
  # repair the training reference explicitly rather than teaching impossible poses.
  k = md.kinematics()
  down.joint_pos = np.clip(down.joint_pos, k.lower, k.upper)
  for i in range(len(down.root_pos)):
    feet = k.feet(down.root_pos[i], down.root_quat[i], down.joint_pos[i])
    lowest = min(k.lowest_body_point(), min(p[2] for p in feet.values()) - md.FOOT_RADIUS)
    # Ignore sub-10-micrometre model rounding so the final root height remains
    # exactly the walking controller's SPIDER_HEIGHT.
    if lowest < -1e-5:
      down.root_pos[i, 2] -= lowest
  return [
    md.Clip("spider_down", down, next="spider_idle"),
    md.Clip("spider_idle", hold(down, down.duration, 1.0), loop=True),
    md.Clip("spider_up", reverse(down)),
  ]


ORDER = ["spider_down", "spider_idle", "spider_idle", "spider_up"]


# ----------------------------------------------------------------------- checks
def check_helpers() -> bool:
  """Checks reverse (and your pasted cut and hold). Prints ok or FAIL for each."""
  ok = True

  def report(name, passed, detail=""):
    nonlocal ok
    ok &= passed
    print(
      f"  {'ok  ' if passed else 'FAIL'} {name}" + ("" if passed else f": {detail}")
    )

  s = recorded()
  a = cut(s, 0.0, 2.0)
  report("cut: 0-2 s is 2 s long", abs(a.duration - 2.0) < 1e-9, f"{a.duration:.3f} s")
  h = hold(s, 2.0, 1.0)
  report(
    "hold: 1 s long, never moving",
    abs(h.duration - 1.0) < 1e-9 and np.ptp(h.joint_pos, axis=0).max() < 1e-12,
  )
  r = reverse(a)
  report(
    "reverse: same length",
    len(r.joint_pos) == len(a.joint_pos),
    f"{len(r.joint_pos)} vs {len(a.joint_pos)} frames",
  )
  report(
    "reverse: starts where the motion ends, and ends where it starts",
    np.allclose(r.joint_pos[0], a.joint_pos[-1])
    and np.allclose(r.root_pos[-1], a.root_pos[0]),
  )
  report(
    "reverse: twice gives the original back",
    np.allclose(reverse(r).joint_pos, a.joint_pos)
    and np.allclose(reverse(r).root_quat, a.root_quat),
  )
  return ok


def main() -> None:
  print("Checking cut, hold and reverse:")
  if not check_helpers():
    raise SystemExit("Fix the failing checks first.")
  if not ORDER:
    raise SystemExit("TODO S2b: set ORDER, the training sequence.")
  library = md.concat_clips(clips(), order=ORDER)
  for key, value in library.report.items():
    print(f"{key:36} {value}")
  for c in library.clips:
    print(
      f"  clip {c['name']:12} frames {c['start']:4}-{c['end']:4} "
      f"({(c['end'] - c['start']) / FPS:.1f} s) loop={c['loop']} next={c['next']}"
    )
  print("wrote", library.save_npz(HERE / "spider_walk.npz"))


if __name__ == "__main__":
  main()
