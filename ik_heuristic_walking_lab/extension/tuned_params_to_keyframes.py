"""Extension: port a gait you tuned in the gait tuner back into walking.py.

The tuner describes a gait with continuous parameters (body height, step length,
step height, frequency, duty, per-leg phase offsets). walking.py describes a gait with
six keyframes per leg. This script converts the first into the second, so the gait
you liked in the browser becomes the gait your own lab code walks with.

Usage (after `git clone https://github.com/ankushDhawan5812/pupper-gait-tuner`):

    python3 tuned_params_to_keyframes.py --pattern trot --step-length 0.10 \
        --step-height 0.09 --body-height 0.14 --frequency 2.0 --duty 0.67

It prints a block of Python you can paste over the keyframes in walking.py,
plus the ``ik_timer_period`` that reproduces the frequency you tuned.
"""

import argparse
import os
import sys

import numpy as np

# The tuner is a separate repo; point this at wherever you cloned it.
TUNER_PATH = os.path.expanduser('~/pupper-gait-tuner')
sys.path.insert(0, TUNER_PATH)

try:
    from gait import GaitParams, LEGS, NOMINAL_XY, effective_offsets, foot_position
except ImportError:
    sys.exit(f"Could not import the gait tuner from {TUNER_PATH}.\n"
             "Clone it first:  git clone https://github.com/ankushDhawan5812/pupper-gait-tuner")

# walking.py holds six keyframes per leg.
N_KEYFRAMES = 6
# ...and resamples them into a 50-entry joint cache (np.arange(0, 1, 0.02)).
N_CACHE_FRAMES = 50

# Variable names in walking.py, in LEGS order (front_r, front_l, back_r, back_l).
VAR_NAMES = ['rf_ee_triangle_positions', 'lf_ee_triangle_positions',
             'rb_ee_triangle_positions', 'lb_ee_triangle_positions']
OFFSET_NAMES = ['rf_ee_offset', 'lf_ee_offset', 'rb_ee_offset', 'lb_ee_offset']


def nominal_offset(leg_name, params):
    """The leg's hip offset, i.e. the ``rf_ee_offset``-style vector in walking.py."""
    nom_x, nom_y = NOMINAL_XY[leg_name]
    nom_y += np.sign(nom_y) * params.stance_width
    return np.array([nom_x, nom_y, 0.0])


def keyframes_for_leg(leg_name, params, n=N_KEYFRAMES):
    """Return an (n, 3) array of keyframes for one leg, relative to its hip offset.

    walking.py adds the per-leg offset separately (``+ rf_ee_offset``), and
    ``interpolate_triangle`` walks the keyframes at a constant rate over one cycle.
    So keyframe k is the tuner's foot position at phase k/n, with the leg's nominal
    offset subtracted back out.
    """
    ################################################################################################
    # TODO 11: Implement the conversion.
    #   - Look up this leg's phase offset with effective_offsets(params)[leg_name].
    #   - For k in range(n), sample foot_position(k / n + offset, leg_name, params).
    #   - Subtract nominal_offset(leg_name, params) so the result is expressed the same way
    #     as the keyframes in walking.py.
    #   - HINT: the phase offset is what makes the legs step at different times. Sampling
    #     the same trajectory at a shifted phase is exactly the "rotate the keyframe list"
    #     trick you used by hand when you wrote the trot in Part 5.
    ################################################################################################
    return np.zeros((n, 3))


def emit_snippet(params):
    """Print the keyframe block to paste into walking.py."""
    lines = []
    for i, leg in enumerate(LEGS):
        kf = keyframes_for_leg(leg.name, params)
        off = nominal_offset(leg.name, params)
        lines.append(f'        {OFFSET_NAMES[i]} = np.array([{off[0]:.4f}, {off[1]:.4f}, 0])')
        lines.append(f'        {VAR_NAMES[i]} = np.array([')
        for row in kf:
            lines.append(f'            [{row[0]: .4f}, {row[1]: .4f}, {row[2]: .4f}],')
        lines.append(f'        ]) + {OFFSET_NAMES[i]}')
        lines.append('')
    print('\n'.join(lines))

    period = 1.0 / (N_CACHE_FRAMES * params.frequency)
    print(f'        # {params.frequency:.2f} Hz gait cycle over a {N_CACHE_FRAMES}-frame cache')
    print(f'        self.ik_timer_period = {period:.6f}')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pattern', default='trot', help='trot | walk | pace | bound | (your own)')
    p.add_argument('--body-height', type=float, default=0.14)
    p.add_argument('--step-length', type=float, default=0.10)
    p.add_argument('--step-height', type=float, default=0.09)
    p.add_argument('--stance-width', type=float, default=0.0)
    p.add_argument('--frequency', type=float, default=2.0)
    p.add_argument('--duty', type=float, default=0.67)
    args = p.parse_args()

    params = GaitParams(
        body_height=args.body_height,
        step_length=args.step_length,
        step_height=args.step_height,
        stance_width=args.stance_width,
        frequency=args.frequency,
        duty=args.duty,
        pattern=args.pattern,
    ).clamped()

    emit_snippet(params)


if __name__ == '__main__':
    main()
