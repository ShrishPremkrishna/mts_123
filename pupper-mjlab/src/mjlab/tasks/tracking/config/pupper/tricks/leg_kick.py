"""Front-leg swing kick: from standing, shift the weight, swing a front paw
forward through the ball, put it down, stand.

The right kick (front_r swings) is designed with keyframes; the left kick is its
mirror image (motion_ops.mirror, as in sit_stay_shake.py). The robot is on three
feet during the swing, so the body first moves away from the kicking leg until
the centre of mass is well inside the triangle of the three planted feet.

    uv run python -m mjlab.tasks.tracking.config.pupper.tricks.leg_kick

writes, next to this file:
  leg_kick_right.{csv,npz} and leg_kick_right_robot.csv   one kick, right paw
  leg_kick_left.{csv,npz}  and leg_kick_left_robot.csv    one kick, left paw
  leg_kick.npz   clip library for one tracking policy that does both kicks:
                 stand_idle (loops) -> kick_right or kick_left -> stand_idle

World frame = the standing robot's frame at t = 0: x forward from base_link,
y left, z up from the ground. The ball positions below are in that frame,
because the walker places the ball before the trick starts.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import mujoco
import numpy as np

from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper.env_cfgs import get_spec_with_hulls
from mjlab.tasks.tracking.config.pupper.motion_ops import hold, mirror

HERE = Path(__file__).resolve().parent
FPS = 50.0


@dataclasses.dataclass(frozen=True)
class Design:
  """All settings of the right kick; the defaults are the chosen kick."""

  # Weight shift away from the right front leg (m, degrees). Positive roll lifts
  # the left side, so a negative roll unloads the right front leg.
  shift_x: float = 0.0
  shift_y: float = 0.055
  shift_dz: float = -0.005
  shift_roll: float = -3.0
  shift_pitch: float = 0.0  # positive: nose down, which lowers the shoulder
  # Strike: where the paw (foot-ball centre) is fastest, in the start frame. The
  # paw moves on a straight line of length swing_len through this point, rising
  # by swing_rise, in swing_s seconds (minimum jerk, so the speed peaks in the
  # middle). Sideways, the paw keeps its standing offset from the shoulder.
  strike_x: float = 0.145
  strike_z: float = 0.068
  swing_len: float = 0.13
  swing_rise: float = 0.015
  swing_s: float = 0.20
  # Timing (s): stand still, shift, lift to the wind-up, hold, swing, hold the
  # follow-through, bring the paw back above its spot, put it down, unshift.
  t_still: float = 0.25
  shift_s: float = 0.60
  lift_s: float = 0.30
  windup_hold_s: float = 0.15
  follow_hold_s: float = 0.10
  retract_s: float = 0.30
  down_s: float = 0.25
  unshift_s: float = 0.55
  end_hold_s: float = 0.30


DESIGN = Design()

# Ball: diameter approximately 0.15 m (sim nominal 0.075 m radius, 0.10 kg).
BALL_RADIUS = 0.075


def keyframes(d: Design = DESIGN) -> list[md.Keyframe]:
  stand = md.stand_feet()
  h = md.STAND_HEIGHT
  shifted = (d.shift_x, d.shift_y, h + d.shift_dz)
  rpy = (d.shift_roll, d.shift_pitch, 0.0)
  fr = np.array(stand["front_r"])
  y = fr[1] + d.shift_y  # the paw keeps its sideways offset from the shoulder
  half = np.array([d.swing_len / 2, 0.0, d.swing_rise / 2])
  mid = np.array([d.strike_x, y, d.strike_z])
  windup, follow = tuple(mid - half), tuple(mid + half)
  above = (fr[0], fr[1], 0.06)  # retract to above the standing spot
  planted = {leg: stand[leg] for leg in ("front_l", "back_r", "back_l")}

  def feet(paw):
    return {**planted, "front_r": tuple(paw)}

  t = [0.0, d.t_still]
  for dt in (
    d.shift_s, d.lift_s, d.windup_hold_s, d.swing_s, d.follow_hold_s,
    d.retract_s, d.down_s, d.unshift_s, d.end_hold_s,
  ):
    t.append(t[-1] + dt)
  t = [round(v, 4) for v in t]
  down = stand["front_r"]
  return [
    md.stand(t[0]),
    md.stand(t[1]),
    md.Keyframe(t[2], shifted, rpy, feet=feet(down)),
    md.Keyframe(t[3], shifted, rpy, feet=feet(windup)),
    md.Keyframe(t[4], shifted, rpy, feet=feet(windup)),
    md.Keyframe(t[5], shifted, rpy, feet=feet(follow)),
    md.Keyframe(t[6], shifted, rpy, feet=feet(follow)),
    md.Keyframe(t[7], shifted, rpy, feet=feet(above)),
    md.Keyframe(t[8], shifted, rpy, feet=feet(down)),
    md.Keyframe(t[9], (0.0, 0.0, h), feet=feet(down)),
    md.stand(t[10]),
  ]


def kick_right(d: Design = DESIGN) -> md.Motion:
  return md.build_motion(keyframes(d), fps=FPS)


# ------------------------------------------------------------------- measurements
def paw_track(motion: md.Motion, leg: str):
  """Paw (foot-ball centre) position and velocity in the world frame, per frame.

  Velocity is the central difference of the frame positions (the motion's own
  50 Hz samples, which are the targets the robot receives).
  """
  k = md.kinematics()
  pos = np.array(
    [
      k.feet(motion.root_pos[i], motion.root_quat[i], motion.joint_pos[i])[leg]
      for i in range(len(motion.joint_pos))
    ]
  )
  vel = np.gradient(pos, 1.0 / motion.fps, axis=0)
  return pos, vel


def strike(motion: md.Motion, leg: str = "front_r") -> dict:
  """The reference paw at its fastest forward speed, and where the ball must rest
  so that the paw first touches it at that instant.

  The ball rests on the ground (centre height BALL_RADIUS) in line with the paw,
  one ball radius plus one foot radius from the paw centre. The contact normal
  (paw centre to ball centre) sets the push direction; a positive elevation
  means the paw hits below the ball's equator and pushes it slightly upward.
  """
  pos, vel = paw_track(motion, leg)
  i = int(np.argmax(vel[:, 0]))
  p, v = pos[i], vel[i]
  reach = BALL_RADIUS + md.FOOT_RADIUS
  dz = BALL_RADIUS - p[2]
  ball = np.array([p[0] + np.sqrt(reach**2 - dz**2), p[1], BALL_RADIUS])
  R = md.quat_to_mat(motion.root_quat[i])
  return {
    "strike_time_s": round(i / motion.fps, 2),
    "paw_speed_m_s": round(float(np.linalg.norm(v)), 2),
    "paw_velocity_world_m_s": np.round(v, 2).tolist(),
    "paw_start_frame_m": np.round(p, 3).tolist(),  # standing robot at t = 0
    "paw_body_frame_at_strike_m": np.round(R.T @ (p - motion.root_pos[i]), 3).tolist(),
    "ball_centre_start_frame_m": np.round(ball, 3).tolist(),
    "contact_normal_elevation_deg": round(float(np.degrees(np.arcsin(dz / reach))), 1),
  }


def ball_clearance(
  motion: md.Motion, ball_centre, until_s: float, swing_leg: str = "front_r"
) -> float:
  """Smallest gap (m) between a ball resting at ``ball_centre`` and the robot's
  collision geoms (convex hulls, knee and foot spheres), from t = 0 to
  ``until_s``. The swing leg's lower leg is left out: it is meant to hit the ball."""
  m, d, ball, geoms = _ball_scene(swing_leg)
  k = md.kinematics()
  d.mocap_pos[0] = ball_centre
  gap, fromto = np.inf, np.zeros(6)
  for i in range(int(until_s * motion.fps) + 1):
    d.qpos[:3] = motion.root_pos[i]
    d.qpos[3:7] = motion.root_quat[i]
    d.qpos[k.qadr] = motion.joint_pos[i]
    mujoco.mj_kinematics(m, d)
    for g in geoms:
      gap = min(gap, mujoco.mj_geomDistance(m, d, g, ball, 0.5, fromto))
  return round(float(gap), 4)


_SCENES: dict = {}


def _ball_scene(swing_leg: str):
  """The trick model plus a mocap ball; the robot geoms that must not touch it."""
  if swing_leg not in _SCENES:
    spec = get_spec_with_hulls()
    body = spec.worldbody.add_body(name="kick_ball", mocap=True)
    body.add_geom(
      name="kick_ball", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[BALL_RADIUS, 0, 0],
      contype=0, conaffinity=0,
    )
    m = spec.compile()
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in range(m.ngeom)]
    geoms = [
      g for g, n in enumerate(names)
      if n.endswith(("_hull", "_collision")) and not n.startswith(f"leg_{swing_leg}_3")
    ]
    _SCENES[swing_leg] = (m, mujoco.MjData(m), names.index("kick_ball"), geoms)
  return _SCENES[swing_leg]


def support_margin(motion: md.Motion, swing_leg: str = "front_r") -> float:
  """Smallest distance (m) of the whole-body centre of mass inside the triangle of
  the three other feet, over the frames where the swing paw is off the ground.
  Negative: the centre of mass is outside (the robot tips toward the paw)."""
  k = md.kinematics()
  worst = np.inf
  for i in range(len(motion.joint_pos)):
    k.set_state(motion.root_pos[i], motion.root_quat[i], motion.joint_pos[i])
    feet = {leg: k.data.site_xpos[s].copy() for leg, s in k.foot_site.items()}
    if feet[swing_leg][2] < md.FOOT_RADIUS + 0.005:
      continue
    com = k.data.subtree_com[1][:2]
    tri = [feet[leg][:2] for leg in md.LEGS if leg != swing_leg]
    d = []
    for j in range(3):
      p, q, r = tri[j], tri[(j + 1) % 3], tri[(j + 2) % 3]
      e = (q - p) / np.linalg.norm(q - p)
      nrm = np.array([-e[1], e[0]])
      if np.dot(r - p, nrm) < 0:
        nrm = -nrm
      d.append(float(np.dot(com - p, nrm)))
    worst = min(worst, min(d))
  return round(worst, 4)


# ----------------------------------------------------------------------- outputs
def clip_library(right: md.Motion, left: md.Motion) -> md.Motion:
  idle = hold(right, 0.0, 1.0)
  return md.concat_clips(
    [
      md.Clip("stand_idle", idle, loop=True),
      md.Clip("kick_right", right, next="stand_idle"),
      md.Clip("kick_left", left, next="stand_idle"),
    ],
    order=["stand_idle", "kick_right", "stand_idle", "kick_left", "stand_idle"],
  )


def main() -> None:
  right = kick_right()
  left = mirror(right)
  left.report = md.check_motion(left)
  left.report["max_ik_error_m"] = right.report["max_ik_error_m"]
  for name, motion, leg in (("right", right, "front_r"), ("left", left, "front_l")):
    print(f"--- leg_kick_{name}")
    for key, value in motion.report.items():
      print(f"{key:36} {value}")
    print(f"{'support_margin_m':36} {support_margin(motion, leg)}")
    hit = strike(motion, leg)
    for key, value in hit.items():
      print(f"{key:36} {value}")
    gap = ball_clearance(motion, hit["ball_centre_start_frame_m"], hit["strike_time_s"] - 0.1, leg)
    print(f"{'ball_gap_before_strike_m':36} {gap}")
    stem = HERE / f"leg_kick_{name}"
    print("wrote", motion.save_csv(stem.with_suffix(".csv")))
    print("wrote", motion.save_npz(stem.with_suffix(".npz")))
    print("wrote", motion.save_robot_csv(HERE / f"leg_kick_{name}_robot.csv"))
  library = clip_library(right, left)
  print("--- leg_kick (clip library)")
  for key, value in library.report.items():
    print(f"{key:36} {value}")
  for c in library.clips:
    print(
      f"  clip {c['name']:11} frames {c['start']:4}-{c['end']:4} "
      f"({(c['end'] - c['start']) / FPS:.2f} s) loop={c['loop']} next={c['next']}"
    )
  print("wrote", library.save_npz(HERE / "leg_kick.npz"))


if __name__ == "__main__":
  main()
