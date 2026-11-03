"""Optional Lab 3, Part 2: clean up a UniMate dance so Pupper can track it.

A UniMate sample is a first draft. It starts anywhere, its feet drift, and it
carries a 2-8 Hz tremor on top of the move. The team's first generated trick
(secret_dance, a 1-degree sway) jittered on the robot for that reason
(foundation/results/generated-motion-*-20261101.json). This cleanup keeps what
the sentence asked for, the motion of the body, and rebuilds the legs:

1. Take the root trajectory of the fitted sample, measured from its lowest foot
   (UniMate's skeletons float), in the sample's own heading. Drop yaw, and remove
   travel drift (a linear trend).
2. Scale the body's deviation from the stand by ``gain`` and limit it softly
   (tanh), so no limit makes a corner.
3. Add the timeline: stand still for ``hold`` seconds at both ends, ``blend``
   seconds of minimum-jerk motion in and out, and (``there_and_back``) the move
   played backwards with a ``pause`` at each turn.
4. Low-pass the whole timeline (zero phase, ``cutoff_hz``): the tremor goes, and
   the joins between the stand, the blends and the move get no velocity steps.
5. Pin all four feet where the default stand puts them: nothing slides or floats.
6. Solve the legs frame by frame with IK, each frame seeded from the last; lower
   the gain by 15 % steps until every frame is within ``max_ik_error_m``.

Only upstream APIs are used (``motion_design``), so the same file runs in the
course Colab notebook. ``smoothness`` and ``visibility`` print the numbers a
reference is judged on before training.
"""

from __future__ import annotations

import numpy as np
from scipy.signal import butter, detrend, filtfilt
from scipy.spatial.transform import Rotation

from mjlab.tasks.tracking.config.pupper import motion_design as md

STAND = np.array([0.0, 0.0, md.STAND_HEIGHT])


def root_rpy(motion: md.Motion) -> np.ndarray:
  """(T, 3) roll, pitch, yaw in degrees, the convention of md.rpy_to_quat."""
  rot = Rotation.from_quat(motion.root_quat[:, [1, 2, 3, 0]])  # scipy wants x, y, z, w
  return rot.as_euler("ZYX", degrees=True)[:, ::-1]


def lowpass(x: np.ndarray, fps: float, cutoff_hz: float) -> np.ndarray:
  """Zero-phase Butterworth low-pass along time (axis 0)."""
  b, a = butter(2, cutoff_hz / (fps / 2.0))
  return filtfilt(b, a, x, axis=0, padlen=min(len(x) - 1, 3 * max(len(a), len(b))))


def _min_jerk(n: int) -> np.ndarray:
  s = np.linspace(0.0, 1.0, n + 2)[1:-1]
  return 10 * s**3 - 15 * s**4 + 6 * s**5


def clean_up(
  motion: md.Motion,
  cutoff_hz: float = 1.2,
  gain: float = 1.0,
  hold: float = 0.8,
  blend: float = 1.0,
  max_pitch_deg: float = 22.0,
  max_roll_deg: float = 12.0,
  max_shift_m: float = 0.03,
  z_range: tuple[float, float] = (0.12, 0.165),
  max_ik_error_m: float = 0.002,
  there_and_back: int = 0,
  pause: float = 0.5,
) -> md.Motion:
  """A fitted UniMate motion -> a smooth, standing-bookended, feet-planted motion.

  report["gain"] is the gain actually used and report["ik_ok"] whether every frame
  reached the planted feet within ``max_ik_error_m`` (after at most 10 tries).
  """
  fps = motion.fps
  k = md.kinematics()
  pos = motion.root_pos.copy()
  # Height above the lowest foot: UniMate's skeleton often floats, and a floating
  # root would put the body higher than the legs reach.
  lowest_foot = np.array([
    min(p[2] for p in k.feet(motion.root_pos[i], motion.root_quat[i],
                             motion.joint_pos[i]).values())
    for i in range(len(pos))
  ])
  pos[:, 2] -= lowest_foot - md.FOOT_RADIUS
  rpy = root_rpy(motion)
  past = np.abs(rpy[:, 0]) > 90  # past vertical: same pose, other Euler branch
  rpy[past, 0] -= 180 * np.sign(rpy[past, 0])
  rpy[past, 1] = 180 * np.sign(rpy[past, 1]) - rpy[past, 1]
  rpy[:, :2] = np.degrees(np.unwrap(np.radians(rpy[:, :2]), axis=0))
  yaw0 = np.radians(rpy[0, 2])
  heading = np.array([[np.cos(yaw0), np.sin(yaw0)], [-np.sin(yaw0), np.cos(yaw0)]])
  pos[:, :2] = detrend((pos[:, :2] - pos[0, :2]) @ heading.T, axis=0, type="linear")
  rpy[:, 2] = 0.0  # no turning: yaw with pinned feet only twists the legs
  if there_and_back:
    n = int(round(pause * fps))
    pos = _there_and_back(pos, n, there_and_back)
    rpy = _there_and_back(rpy, n, there_and_back)

  for i in range(10):
    g = gain * 0.85**i
    out = _build(pos, rpy, fps, g, hold, blend, cutoff_hz, max_pitch_deg,
                 max_roll_deg, max_shift_m, z_range)
    if out.report["max_ik_error_m"] <= max_ik_error_m:
      break
  out.report["gain"] = round(g, 3)
  out.report["ik_ok"] = out.report["max_ik_error_m"] <= max_ik_error_m
  return out


def _there_and_back(x: np.ndarray, n: int, times: int) -> np.ndarray:
  """x, held n frames, x backwards, held n frames; ``times`` times."""
  return np.concatenate(
    [x, np.repeat(x[-1:], n, 0), x[::-1], np.repeat(x[:1], n, 0)] * times
  )


def _soft(x: np.ndarray, lim: float, mid: float = 0.0) -> np.ndarray:
  """A limit with no corner: x near mid is unchanged, |x - mid| never reaches lim."""
  return mid + lim * np.tanh((x - mid) / lim)


def _build(pos, rpy, fps, gain, hold, blend, cutoff_hz, max_pitch_deg,
           max_roll_deg, max_shift_m, z_range) -> md.Motion:
  pos = STAND + gain * (pos - STAND)
  pos[:, :2] = _soft(pos[:, :2], max_shift_m)
  z_mid, z_half = (z_range[0] + z_range[1]) / 2, (z_range[1] - z_range[0]) / 2
  pos[:, 2] = _soft(pos[:, 2], z_half, z_mid)
  rpy = gain * rpy
  rpy[:, 0] = _soft(rpy[:, 0], max_roll_deg)
  rpy[:, 1] = _soft(rpy[:, 1], max_pitch_deg)

  # Timeline: stand, blend in, the move, blend out, stand; then one low-pass over it.
  n_hold, n_blend = int(round(hold * fps)), int(round(blend * fps))
  zero = np.zeros(3)
  s_in, s_out = _min_jerk(n_blend)[:, None], _min_jerk(n_blend)[::-1, None]
  pos = np.concatenate([
    np.repeat(STAND[None], n_hold, 0), STAND + s_in * (pos[0] - STAND), pos,
    STAND + s_out * (pos[-1] - STAND), np.repeat(STAND[None], n_hold, 0),
  ])
  rpy = np.concatenate([
    np.repeat(zero[None], n_hold, 0), s_in * rpy[0], rpy, s_out * rpy[-1],
    np.repeat(zero[None], n_hold, 0),
  ])
  pos, rpy = lowpass(pos, fps, cutoff_hz), lowpass(rpy, fps, cutoff_hz)

  # Legs: all four feet stay where the default stand puts them.
  k = md.kinematics()
  feet = {leg: np.array(p) for leg, p in md.stand_feet(tuple(STAND)).items()}
  q = np.array(md.DEFAULT_POSE, dtype=float)
  quats = np.array([md.rpy_to_quat(r) for r in rpy])
  joints = np.zeros((len(pos), 12))
  worst, raised = 0.0, 0
  for i in range(len(pos)):
    for attempt in range(5):  # raise the body if a hull would go below the floor
      errs = []
      for leg in md.LEGS:
        q, err = k.solve_leg(leg, feet[leg], pos[i], quats[i], q, iters=100)
        errs.append(err)
      k.set_state(pos[i], quats[i], q)
      low = k.lowest_body_point()
      if low >= -1e-4 or attempt == 4:
        break
      pos[i, 2] -= low  # then solve the legs again at the new height
      raised += attempt == 0
    worst = max(worst, *errs)
    joints[i] = q

  out = md.Motion(fps=fps, root_pos=pos, root_quat=quats, joint_pos=joints)
  out.report = md.check_motion(out)
  out.report["max_ik_error_m"] = round(worst, 4)
  out.report["frames_raised_for_clearance"] = raised
  return out


def smoothness(motion: md.Motion) -> dict:
  """How much of the joint motion is tremor. The supplied tricks that ran on the
  robot score ratio <= 0.35, about 1 reversal/s or fewer, and slow:tremor >= 9."""
  q, fps = motion.joint_pos, motion.fps
  d1, d2 = np.diff(q, axis=0), np.diff(q, n=2, axis=0)
  a, b = d1[:-1], d1[1:]
  big = (np.abs(a) > 0.002) & (np.abs(b) > 0.002)
  reversals = np.sum((np.sign(a) != np.sign(b)) & big)
  w = np.hanning(len(q))
  power = np.abs(np.fft.rfft(detrend(q, axis=0) * w[:, None], axis=0)) ** 2
  power /= np.sum(w**2) * len(q) / 2  # one-sided: the bands sum to the variance
  f = np.fft.rfftfreq(len(q), 1.0 / fps)

  def band_deg(lo, hi):
    return float(np.degrees(np.sqrt(np.mean(power[(f >= lo) & (f < hi)].sum(0)))))

  slow, tremor = band_deg(0.1, 2.0), band_deg(2.0, 8.0)
  ratio = np.sqrt(np.mean(d2**2)) / max(np.sqrt(np.mean(d1**2)), 1e-12)
  return {
    "d2_over_d1": round(float(ratio), 3),
    "reversals_per_s": round(float(reversals) / motion.duration, 2),
    "slow_0.1-2Hz_deg": round(slow, 3),
    "tremor_2-8Hz_deg": round(tremor, 3),
    "slow_to_tremor": round(slow / max(tremor, 1e-9), 1),
  }


def visibility(motion: md.Motion) -> dict:
  """How big the move is: the robot shows about half of a small commanded range."""
  rpy, q, xy = root_rpy(motion), motion.joint_pos, motion.root_pos[:, :2]
  start = np.abs(q[0] - np.array(md.DEFAULT_POSE)).max()
  rot = Rotation.from_quat(motion.root_quat[:, [1, 2, 3, 0]])
  chest_up = 100 * rot.apply(np.subtract(md.CHEST, md.BUTT))[:, 2]  # < 0: a bow
  speed = np.abs(np.diff(q, axis=0)).max() * motion.fps
  return {
    "pitch_range_deg": round(float(np.ptp(rpy[:, 1])), 1),
    "pitch_min_max_deg": tuple(round(float(f(rpy[:, 1])), 1) for f in (min, max)),
    "chest_minus_rear_cm": tuple(round(float(f(chest_up)), 1) for f in (min, max)),
    "roll_range_deg": round(float(np.ptp(rpy[:, 0])), 1),
    "height_range_cm": round(float(100 * np.ptp(motion.root_pos[:, 2])), 1),
    "travel_cm": round(float(100 * np.linalg.norm(xy[-1] - xy[0])), 1),
    "largest_joint_range_rad": round(float(np.ptp(q, axis=0).max()), 2),
    "peak_joint_speed_rad_s": round(float(speed), 2),
    "start_pose_from_stand_rad": round(float(start), 3),
  }
