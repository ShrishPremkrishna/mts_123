---
date: 2026-10-19T10:14
tags:
  - cs123
  - lab4
---
# Shrish's Lab 4 training handoff

The completed experiment is **Step 7 StableGait**, following the Step 6 lift-gait design. It is not the earlier Step 3 tracking-only run or the team's best Step 4 from-scratch experiment.

## Start here

- [Completed W&B run: 4hkyvuxh](https://wandb.ai/shrishp-stanford/mjlab/runs/4hkyvuxh) — existing results and run files.
- [Open configured notebook in Colab](https://colab.research.google.com/github/ShrishPremkrishna/mts_123/blob/main/pupper-mjlab/lab4_runs/shrish_stablegait/CS123_StableGait_Shrish.ipynb) — use this copy for sharing or starting a new run.
- [Exact recorded settings](training-settings.json).
- [Step 6 reference video](https://drive.google.com/file/d/1tM5Or-W3KuLJunFk66fl2r_e2eaG7vdX/view?usp=drive_link).
- [Step 7 final-policy video](https://drive.google.com/file/d/1R9TXiXDxXtrec42yhgUol9gsqaotVXrV/view?usp=drive_link).
- [Shared lab answers](https://docs.google.com/document/d/1SXTJbsNtFzzQWN5G0x6bqWesDSliujptZeMbVfN2CCk/edit) — Q17–22 and the StableGait side of Q23.

The configured notebook was reconstructed after training from the saved settings and the course notebook. It contains no saved execution outputs or credentials. The existing results belong to the W&B run above; running this copy creates a new experiment under the account that signs in.

## Training settings

| Setting | Value |
| --- | --- |
| Task | `Mjlab-StableGait-Flat-Pupper-v3` |
| Parallel simulated robots | 4,096 |
| Training iterations | 3,000 |
| Random seed | 42 |
| Training attempts | 1 |
| Run name | `2026-10-19_06-36-06` |
| Course source commit | `9cfcb148c6514d70ceb9851b98897d2b9e68079b` |
| Final checkpoint used in video | `model_2999.pt` |

All PPO and environment settings not overridden in the notebook use the course defaults at the pinned source commit. The first approximately 500 iterations use the course's supported-body gait-learning phase; afterward the body is released and the full selected reward is active.

### Lift gait — notebook section 4

```python
LIFT_STRIDE = 0.0
LIFT_TOUCHDOWN = (0.0, 0.5, 0.5, 0.0)  # FR, FL, BR, LB
LIFT_STANCE_Z = -0.14
LIFT_SWING_LIFT = 0.09
```

The reference passed IK and produced a `(100, 12)` table. Run this section before StableGait training in every fresh runtime.

### Rewards — notebook section 5

| Reward term | Weight |
| --- | ---: |
| `track_linear_velocity` | 10 |
| `track_yaw_velocity` | 7 |
| `gait_tracking` | 10 |
| `upright` | 1 |
| `lin_vel_z_l2` | -0.5 |
| `ang_vel_xy_l2` | -0.05 |
| `action_rate_l2` | -0.02 |
| `knee_ground_contact` | -1 |

All other reward weights are **0**. There are eight nonzero terms.

### Randomization — notebook section 6

```python
KP_MULTIPLIER_RANGE = (0.6, 1.1)
KD_MULTIPLIER_RANGE = (0.8, 1.5)
FRICTION_RANGE = (0.6, 1.4)
```

## Completed results

Final W&B summary after the run reported Finished:

| Metric | Value |
| --- | ---: |
| `error_vel_xy` | 0.1231348 |
| `error_vel_yaw` | 0.3761393 |
| Mean episode length | 500 steps |
| Mean reward | 223.00849 |

The 30-second final-policy clip has six five-second tests: forward, backward, sideways, left turn, right turn, and zero-command standing. Each segment starts with a reset; no termination/reset occurred within a segment. The robot stays upright, steps during motion, turns both ways, and settles near stillness. These are short simulator demonstrations, not proof of hardware robustness.

To inspect the trained model, open the existing W&B run's **Files** tab and locate its exported `policy.json`. The course notebook's section 9 describes deployment. The local Colab checkpoint was `/content/pupper-mjlab/logs/rsl_rl/pupper_stable_gait/2026-10-19_06-36-06/model_2999.pt`; that temporary runtime path is not accessible from someone else's runtime. The model weights are not bundled in this GitHub handoff.

## Team handoff

- Member C can inspect settings and results immediately; no rerun is needed just to review them.
- To run a new experiment, open the Colab link, select a GPU runtime, complete sections 1–6, and run the **training** cell in section 7. The preceding live-viewer cell is optional and does not train.
- Q23 still needs the teammate's best Step 4 from-scratch run: link, iteration budget, final tracking errors, video, reward weights, and total training attempts.
- Q21 needs confirmation of the exact standing-pose reward term used in that Step 4 run.

Prepared with AI assistance from the recorded run settings, completed W&B summary, and actual simulator recordings. No physical-robot testing was performed for this experiment.
