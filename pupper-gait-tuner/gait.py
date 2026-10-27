"""Parameterized quadruped gait generator for Pupper V3.

Pure numpy, no ROS. Turns a small set of human-tunable gait parameters into a
cyclic cache of 12 joint angles (4 legs x 3 joints) that can be streamed to the
forward_command_controller, plus the foot trajectories for visualization.

Kinematics (fk_batch) are a vectorized, batch-evaluated copy of the per-leg
forward kinematics in lab_4_fall_2025_solutions/lab_4.py, verified to match the
original scalar FK to 1e-9. IK is batched Gauss-Newton: ~5 iterations, sub-micron
EE error, ~25 ms for a full 50-frame cache on a Pi 5 (vs 9.3 s for the original
gradient-descent precompute).

Joint order matches forward_command_controller in lab_4.yaml:
    leg_front_r_1..3, leg_front_l_1..3, leg_back_r_1..3, leg_back_l_1..3
"""

from dataclasses import dataclass, field, replace
import numpy as np


# --------------------------------------------------------------------------
# Homogeneous transforms (constant ones return 4x4; Rz is batched)
# --------------------------------------------------------------------------
def rotation_x(a):
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(a), -np.sin(a), 0],
                     [0, np.sin(a), np.cos(a), 0],
                     [0, 0, 0, 1]])


def rotation_y(a):
    return np.array([[np.cos(a), 0, np.sin(a), 0],
                     [0, 1, 0, 0],
                     [-np.sin(a), 0, np.cos(a), 0],
                     [0, 0, 0, 1]])


def translation(x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]], dtype=float)


def _Rz_batch(a):
    """Batched rotation about z. a: (B,) -> (B,4,4)."""
    B = a.shape[0]
    c, s = np.cos(a), np.sin(a)
    M = np.zeros((B, 4, 4))
    M[:, 0, 0] = c; M[:, 0, 1] = -s
    M[:, 1, 0] = s; M[:, 1, 1] = c
    M[:, 2, 2] = 1.0
    M[:, 3, 3] = 1.0
    return M


def _const(M, B):
    return np.broadcast_to(M, (B, 4, 4))


# --------------------------------------------------------------------------
# Per-leg geometry (identical to lab_4.py). left legs negate joints 0 and 2.
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Leg:
    name: str
    base: np.ndarray   # 4x4 translation to the hip mount, in body frame
    ee_y: float        # +1 / -1 sign on the end-effector y offset
    left: bool         # left legs negate theta[0] and theta[2]


LEGS = [
    Leg("front_r", translation(0.07500, -0.08350, 0), +1, False),
    Leg("front_l", translation(0.07500,  0.08350, 0), -1, True),
    Leg("back_r",  translation(-0.07500, -0.07250, 0), +1, False),
    Leg("back_l",  translation(-0.07500,  0.07250, 0), -1, True),
]

NAME_TO_INDEX = {leg.name: i for i, leg in enumerate(LEGS)}


def _scale(scales, idx):
    """Per-leg multiplier lookup; 1.0 when no per-leg scales are set."""
    return 1.0 if scales is None else scales[idx]

# constant sub-transforms, precomputed
_T12 = rotation_y(-1.57080)
_T23 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080)


def fk_batch(theta, leg: Leg):
    """Forward kinematics for one leg, batched over B poses.

    theta: (B,3) joint angles -> returns (B,3) foot positions in BODY frame.
    """
    B = theta.shape[0]
    s0 = -1.0 if leg.left else 1.0
    s2 = -1.0 if leg.left else 1.0
    A = _const(leg.base @ rotation_x(1.57080), B) @ _Rz_batch(s0 * theta[:, 0])
    Bm = _const(_T12, B) @ _Rz_batch(theta[:, 1])
    Cm = _const(_T23, B) @ _Rz_batch(s2 * theta[:, 2])
    D = _const(translation(0.06231, -0.06216, leg.ee_y * 0.01800), B)
    T = A @ Bm @ Cm @ D
    return T[:, :3, 3]


def ik_batch(targets, leg: Leg, init=None, iters=6, eps=1e-4):
    """Batched Gauss-Newton IK. targets: (B,3) body-frame foot positions.

    Returns (B,3) joint angles. Converges to ~0 EE error in ~5 iterations.
    """
    B = targets.shape[0]
    th = np.zeros((B, 3)) if init is None else init.copy()
    for _ in range(iters):
        f0 = fk_batch(th, leg)
        r = f0 - targets                      # residual (B,3)
        J = np.empty((B, 3, 3))               # numerical Jacobian dpos/dtheta
        for d in range(3):
            tp = th.copy(); tp[:, d] += eps
            J[:, :, d] = (fk_batch(tp, leg) - f0) / eps
        th -= np.linalg.solve(J, r[:, :, None])[:, :, 0]
    return th


# --------------------------------------------------------------------------
# Gait parameters
# --------------------------------------------------------------------------
# Per-leg nominal foot offset in body frame (fore/aft, lateral). Matches the
# stance posture baked into lab_4.py's keyframes.
NOMINAL_XY = {
    "front_r": (0.06, -0.09),
    "front_l": (0.06,  0.09),
    "back_r":  (-0.11, -0.09),
    "back_l":  (-0.11,  0.09),
}

# Phase offset (fraction of a cycle) for each leg, per gait pattern.
GAIT_PATTERNS = {
    "pronk": {"front_r": 0.0, "front_l": 0.0, "back_r": 0.0, "back_l": 0.0},
    "crawl_reverse": {"front_r": 0.0, "front_l": 0.5, "back_r": 0.75, "back_l": 0.25},
    # diagonal pairs move together
    "trot":  {"front_r": 0.0, "front_l": 0.5, "back_r": 0.5, "back_l": 0.0},
    # lateral pairs move together
    "pace":  {"front_r": 0.0, "front_l": 0.5, "back_r": 0.0, "back_l": 0.5},
    # front pair, then back pair
    "bound": {"front_r": 0.0, "front_l": 0.0, "back_r": 0.5, "back_l": 0.5},
    # even 4-beat walk (most statically stable)
    "walk":  {"front_r": 0.0, "front_l": 0.5, "back_r": 0.25, "back_l": 0.75},
}


@dataclass
class GaitParams:
    body_height: float = 0.14    # m, stance distance from hip to foot (taller = bigger)
    step_length: float = 0.10    # m, fore/aft stride reach (peak-to-peak)
    step_height: float = 0.09    # m, foot clearance during swing
    stance_width: float = 0.0    # m, extra lateral spread added to nominal
    frequency: float = 2.0       # Hz, gait cycles per second
    duty: float = 0.67           # fraction of cycle each foot is on the ground
    pattern: str = "trot"        # named preset (used when phase_offsets is None)
    # explicit per-leg phase offsets in LEG order (front_r, front_l, back_r,
    # back_l), each a fraction of a cycle. Overrides `pattern` when set.
    phase_offsets: tuple = None
    # per-leg multipliers on step length / height, in LEG order. None = all 1.0.
    step_length_scale: tuple = None
    step_height_scale: tuple = None
    swing_profile: str = "cycloid"  # sine remains selectable for comparison
    n_frames: int = 50           # cache resolution per cycle

    def clamped(self):
        """Return a copy with all values clamped to safe ranges."""
        if self.swing_profile not in ("sine", "cycloid"):
            raise ValueError("swing_profile must be sine or cycloid")
        offs = self.phase_offsets
        if offs is not None:
            offs = tuple(float(x) % 1.0 for x in offs)

        def clamp_scale(t):
            if t is None:
                return None
            return tuple(float(np.clip(x, 0.0, 2.0)) for x in t)

        return replace(
            self,
            body_height=float(np.clip(self.body_height, 0.08, 0.18)),
            step_length=float(np.clip(self.step_length, 0.0, 0.16)),
            step_height=float(np.clip(self.step_height, 0.0, 0.12)),
            stance_width=float(np.clip(self.stance_width, -0.04, 0.06)),
            frequency=float(np.clip(self.frequency, 0.2, 4.0)),
            duty=float(np.clip(self.duty, 0.4, 0.9)),
            pattern=self.pattern if self.pattern in GAIT_PATTERNS else "trot",
            phase_offsets=offs,
            step_length_scale=clamp_scale(self.step_length_scale),
            step_height_scale=clamp_scale(self.step_height_scale),
            n_frames=int(np.clip(self.n_frames, 10, 200)),
        )


def effective_offsets(p: "GaitParams"):
    """Per-leg phase offsets as a {leg_name: offset} dict.

    Uses explicit phase_offsets when present, else the named pattern preset.
    """
    if p.phase_offsets is not None:
        return {leg.name: p.phase_offsets[i] for i, leg in enumerate(LEGS)}
    return dict(GAIT_PATTERNS[p.pattern])


# --------------------------------------------------------------------------
# Foot trajectory: continuous function of cycle phase
# --------------------------------------------------------------------------
def foot_position(phase, leg_name, p: GaitParams):
    """Foot position (body frame) for one leg at a given cycle phase [0,1).

    Stance: foot on ground, sweeps fore->aft (pushing the body forward).
    Swing:  foot lifts in a sine arc and returns aft->fore.
    """
    i = NAME_TO_INDEX[leg_name]
    step_length = p.step_length * _scale(p.step_length_scale, i)
    step_height = p.step_height * _scale(p.step_height_scale, i)

    nom_x, nom_y = NOMINAL_XY[leg_name]
    nom_y += np.sign(nom_y) * p.stance_width
    z_stance = -p.body_height
    half = step_length / 2.0

    ph = phase % 1.0
    if ph < p.duty:
        # stance: +half -> -half, foot planted
        u = ph / p.duty
        x = half - u * step_length
        z = z_stance
    else:
        # swing: -half -> +half with a lifted sine arc
        u = (ph - p.duty) / (1.0 - p.duty)
        if p.swing_profile == "cycloid":
            x = -half + step_length * (u - np.sin(2 * np.pi * u) / (2 * np.pi))
            z = z_stance + step_height * (1 - np.cos(2 * np.pi * u)) / 2
        else:
            x = -half + u * step_length
            z = z_stance + step_height * np.sin(np.pi * u)
    return np.array([nom_x + x, nom_y, z])


def foot_trajectories(p: GaitParams):
    """(n_frames, 4, 3) commanded foot positions, one column per leg in LEG order."""
    p = p.clamped()
    offsets = effective_offsets(p)
    phases = np.arange(p.n_frames) / p.n_frames
    feet = np.empty((p.n_frames, 4, 3))
    for li, leg in enumerate(LEGS):
        off = offsets[leg.name]
        for fi, base_ph in enumerate(phases):
            feet[fi, li] = foot_position(base_ph + off, leg.name, p)
    return feet


def generate_cache(p: GaitParams):
    """Build the joint-angle cache for one full gait cycle.

    Returns:
        joints: (n_frames, 12) joint angles, controller order.
        feet:   (n_frames, 4, 3) commanded foot positions (for viz).
    """
    p = p.clamped()
    feet = foot_trajectories(p)
    joints = np.empty((p.n_frames, 12))
    init = None
    for li, leg in enumerate(LEGS):
        th = ik_batch(feet[:, li, :], leg, init=init)
        joints[:, 3 * li:3 * li + 3] = th
    return joints, feet


if __name__ == "__main__":
    import time
    p = GaitParams()
    t = time.time()
    j, f = generate_cache(p)
    dt = time.time() - t
    # report worst-case EE error
    err = 0.0
    for li, leg in enumerate(LEGS):
        err = max(err, np.abs(fk_batch(j[:, 3*li:3*li+3], leg) - f[:, li, :]).max())
    print(f"cache {j.shape} in {dt*1000:.0f} ms, max EE error {err*1e3:.4f} mm")
