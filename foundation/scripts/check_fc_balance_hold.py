"""Compare fresh-feedback zero gait with holding the last targets on a free Pupper.

Use the same numerical environment as verify_fc_policy_cycle.py. This experiment
covers one level surface and a modeled actuator delay. It cannot qualify a Pi,
a second sensor, drive continuity, or a mid-kick recovery.
"""
import argparse
import json
import math
import os
import tempfile
from pathlib import Path

import mujoco
import numpy as np

from verify_fc_policy_cycle import HZ, PolicyPlant, add_soccer_ball, build_controller, make_model, selected_profile, selection


def trial(model, binary, params, walking_ticks, strategy):
    body = PolicyPlant(model, binary, "left", params)
    try:
        body.activate()
        for _ in range(5 * HZ):
            body.advance([0., 0., 0.])
        for _ in range(walking_ticks):
            body.advance([.24, 0., 0.])
        q = body.data.qpos
        origin = q[body.rq:body.rq + 2].copy()
        low = q[body.rq + 2]
        peak_tilt = 0.
        farthest = 0.
        for _ in range(2 * HZ):
            if strategy == "fresh_zero_gait":
                body.advance([0., 0., 0.])
            else:
                # Freeze the last applied targets and gains. Encoders, all drives
                # and their coordinates remain perfect throughout this branch.
                mujoco.mj_step(model, body.data)
            _, x, y, _ = q[body.rq + 3:body.rq + 7]
            peak_tilt = max(peak_tilt, math.degrees(math.acos(np.clip(1 - 2*(x*x + y*y), -1., 1.))))
            low = min(low, q[body.rq + 2])
            farthest = max(farthest, np.linalg.norm(q[body.rq:body.rq + 2] - origin))
        return dict(walking_ticks=walking_ticks, strategy=strategy, peak_tilt_deg=round(peak_tilt, 2),
                    minimum_base_height_m=round(float(low), 3), peak_displacement_m=round(float(farthest), 3))
    finally:
        body.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--staged-release", type=Path, help="verify selected application bytes against a staged release")
    args = parser.parse_args()
    os.environ.update(PUPPER_FC_RECORDING="events", PUPPER_FC_IMU_RECOVERY="field_redundant_imu")
    model, contact = make_model("full_hulls", configure_scene=add_soccer_ball)
    with tempfile.TemporaryDirectory() as work:
        root = Path(work)
        params, release = selected_profile(root, args.staged_release, selection())
        binary, _ = build_controller(root)
        trials = [trial(model, binary, params, ticks, strategy)
                  for ticks in (0, 280, 340, 400, 460, 520)
                  for strategy in ("fresh_zero_gait", "frozen_joint_targets")]
    result = dict(model=mujoco.__version__, contact_model=contact, release=release,
                  duration_s=2, rate_hz=HZ, modeled_actuator_delay_s=.016, trials=trials)
    args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
