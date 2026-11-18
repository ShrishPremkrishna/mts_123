# CS123 Opt Lab 2: run lineage, checkpoint inventory and metrics (recovered from WandB)

Source: WandB project `photon_mz-stanford-university/mjlab`. All runs: host 129-213-162-127, 1x NVIDIA A10,
`-m mjlab.scripts.pupper_locomotion train --profile baseline --num-envs 4096 --seed 42` (except the s43 seed runs),
plus `--deployment-target-clamp --compose-com-randomization --locomotion-torso-contact --driver-limits ../foundation/configs/class-driver-joint-limits.json`.
Repo: `github.com/ShrishPremkrishna/mts_123` (commits below). Generated 2026-11-18.

Note: WandB `_step` equals the global PPO iteration, including in resumed runs. For example, p2g-r2 logs `_step` 3497–3996.

## 1. Lineage

`[ref = --reference-dir basename; w = --weights basename; +N = --iterations]`. The final iteration is the last `model_*.pt` by number.
All `--checkpoint` paths were absolute under `/home/ubuntu/mts_123/pupper-mjlab/artifacts/opt-lab-2/runs/<run>/`.

```
FROM SCRATCH (Part 1)
baseline-s42-wandb      59db016  ref=default        +3000 -> 2999   finished
 └─ model_2999 ─> baseline-s42-cont4500   42a4f01  ref=default  +1500 -> 4498   finished
baseline-s43            42a4f01  ref=default        +3000 -> 2999   finished  (seed 43)
fast_trot-s42           355b46a  ref=fast_trot      +3000 -> 2999   finished
fast_stagger-s42        355b46a  ref=fast_stagger   +3000 -> 2999   finished
fast_rotary-s42         355b46a  ref=fast_rotary    +3000 -> CRASHED at it 1089 (last ckpt model_1050), never resumed
fast_wide-s42           42a4f01  ref=fast_wide      +3000 -> 2999   finished
fast_long-s42           42a4f01  ref=fast_long      +3000 -> 2999   finished
fast_long-s43           42a4f01  ref=fast_long      +3000 -> 2999   finished  (seed 43)

FIRST PART-2 ATTEMPT (from fast_long-s42/model_2999). All CRASHED and none were resumed.
fast_long-s42/model_2999
 ├─ p2-r1-control       42a4f01  ref=fast_long                        +1000  CRASHED at 3660 (ckpt 3650)
 ├─ p2-r1-jitter        27808ec  ref=fast_long   w=stand_still        +1000  CRASHED at ~3099 (ckpt 3100)
 ├─ p2-r1-naive         27808ec  ref=r1_naive                         +1000  CRASHED at ~3097 (ckpt 3050)
 ├─ p2-r1-phase         27808ec  ref=r1_phase                         +1000  CRASHED at ~3099 (ckpt 3100)
 ├─ p2-r1-sym           27808ec  ref=r1_sym                           +1000  CRASHED at ~3099 (ckpt 3100)
 └─ p2-r1-sym-protect   27808ec  ref=r1_sym      w=protect_symmetry   +1000  CRASHED at ~3099 (ckpt 3100)
    (6 concurrent runs. All stopped at 18:39 UTC; no output.log was uploaded.)

GALLOP (Part 1 gallop variants; dd213ab, w=splay). Killed by a shared CUDA fault, then resumed as "-r2".
gallop_a-s42  ref=fast_gallop_a  +3000  FAILED at 2095 ─ model_2050 ─> gallop_a-s42-r2  +949 -> 2998 finished
gallop_b-s42  ref=fast_gallop_b  +3000  FAILED at 2124 ─ model_2100 ─> gallop_b-s42-r2  +899 -> 2998 finished
gallop_c-s42  ref=fast_gallop_c  +3000  FAILED at 2098 ─ model_2050 ─> gallop_c-s42-r2  +949 -> 2998 finished

PART 2 (all descend from gallop_c-s42-r2/model_2998)
gallop_c-s42-r2/model_2998
 ├─ p2g-r1-naive       ba6a5f9 ref=p2r1_gallop_c_naive  w=splay             +1000 FAILED at 3222 ─ model_3200 ─> p2g-r1-naive-r2      +798 -> 3997
 ├─ p2g-r1-jitter      ba6a5f9 ref=p2r1_gallop_c_lag    w=splay_stand_still +1000 FAILED at 3220 ─ model_3200 ─> p2g-r1-jitter-r2     +798 -> 3997
 ├─ p2g-r1-bootstrap   ba6a5f9 ref=p2r1_gallop_c_lag    w=splay             +1000 FAILED at 3224 ─ model_3200 ─> p2g-r1-bootstrap-r2  +798 -> 3997
 ├─ p2g-ctrl-check     ba6a5f9 ref=fast_gallop_c        w=splay             +300  -> 3297 finished
 ├─ p2g-r1b-bootstrap  c1f269c ref=p2r1b_gallop_c_lag_m2 w=splay            +500  -> 3497 finished
 ├─ p2g-r1b-jitter     c1f269c ref=p2r1b_gallop_c_lag_m2 w=splay_stand_still +500 -> 3497 finished
 │   ├─ p2g-r2         dc21aeb ref=p2r2b_lag_m2          w=splay_stand_still +500 -> 3996 finished
 │   │   ├─ p2g-r3     7aa76f8 ref=p2r3_lag_m2           w=splay_stand_still +500 -> 4495 finished
 │   │   └─ p2g-r2cont dc21aeb ref=p2r2b_lag_m2          w=splay_stand_still +500 -> 4495 finished
 │   └─ p2g-r1cont-a   c1f269c ref=p2r1b_gallop_c_lag_m2 w=splay_stand_still +500 -> 3996 finished
 │       └─ p2g-r1cont-b c1f269c (same ref/weights)                         +500 -> 4495 finished
 └─ p2g-control1000    dc21aeb ref=fast_gallop_c         w=splay_stand_still +1000 -> 3997 finished
     └─ p2g-ctrl1500   dc21aeb (same ref/weights)                            +500 -> 4496 finished
```

**Crashes and resumes.** The WandB logs of every "failed" run end in
`torch.AcceleratorError: CUDA error: an illegal memory access was encountered`. The error surfaces at a different
call site in each run (rsl_rl `ppo.update`, the `body_link_quat_w` reward, the contact-sensor reset, `_resample_command`).
That points to a fault shared by all processes on the GPU, as with an MPS server fault, not to a code bug. In every
batch the concurrent runs died together: gallop_a/b/c at 19:51, p2g-r1-* at 20:34 (after ~458 s), and the six p2-r1-* at 18:39.
The p2-r1-* and fast_rotary-s42 runs are "crashed": the process was hard-killed and uploaded no output.log.
The resumed runs are gallop_{a,b,c}-s42-r2 (from the last 50-iter checkpoint of the failed run) and p2g-r1-{naive,jitter,bootstrap}-r2 (from model_3200).
The p2-r1-* family and fast_rotary-s42 were never resumed. The p2-r1 study was redone as p2g-* on the gallop_c parent.

**Reward-weight differences between configs** (everything else in the reward and command configs is identical):

| term | baseline / fast_* | gallop_* (w=splay) | p2g-* with w=splay_stand_still |
|---|---|---|---|
| abduction_angle | 0 | -0.1 | -0.1 |
| roll_asymmetry | 0 | -1 | -1 |
| stand_still_joint_velocity | 0 | 0 | -0.1 |
| stand_still_pose | 0 | 0 | -0.5 |

baseline/fast_* also use a `pretrain_rewards` curriculum (only gait_tracking until step 12000) and a `hold_base_airborne` warmup.
The resumed gallop and p2g runs do not, because they start past that point.
gait_tracking is scored against each run's own reference, so **total reward is not strictly comparable across reference dirs**.
error_vel_xy/yaw and fell_over are comparable.

## 2. Checkpoint inventory (`~/recovery/checkpoints/<run>/`)

All files were downloaded and their sizes match WandB. Every `.pt` loads with `torch.load(..., weights_only=False)` (torch 2.15 CPU).

| run | role | .pt (iter) | .pt size | load ok | .onnx | config.yaml |
|---|---|---|---|---|---|---|
| baseline-s42-cont4500 | final | `model_4498.pt` (4498) | 4,857,077 | yes | baseline-s42-cont4500.onnx (1,264,070) | yes |
| baseline-s42-wandb | final | `model_2999.pt` (2999) | 4,856,565 | yes | baseline-s42-wandb.onnx (1,264,067) | yes |
| fast_long-s42 | final | `model_2999.pt` (2999) | 4,856,565 | yes | fast_long-s42.onnx (1,264,062) | yes |
| fast_stagger-s42 | final | `model_2999.pt` (2999) | 4,856,565 | yes | fast_stagger-s42.onnx (1,264,065) | yes |
| fast_trot-s42 | final | `model_2999.pt` (2999) | 4,856,565 | yes | fast_trot-s42.onnx (1,264,062) | yes |
| fast_wide-s42 | final | `model_2999.pt` (2999) | 4,856,565 | yes | fast_wide-s42.onnx (1,264,062) | yes |
| gallop_a-s42-r2 | final | `model_2998.pt` (2998) | 4,857,077 | yes | gallop_a-s42-r2.onnx (1,264,064) | yes |
| gallop_a-s42 | intermediate parent (failed run) | `model_2050.pt` (2050) | 4,856,501 | yes | — | yes |
| gallop_b-s42-r2 | final | `model_2998.pt` (2998) | 4,857,077 | yes | gallop_b-s42-r2.onnx (1,264,064) | yes |
| gallop_b-s42 | intermediate parent (failed run) | `model_2100.pt` (2100) | 4,856,501 | yes | — | yes |
| gallop_c-s42-r2 | final | `model_2998.pt` (2998) | 4,857,077 | yes | gallop_c-s42-r2.onnx (1,264,064) | yes |
| gallop_c-s42 | intermediate parent (failed run) | `model_2050.pt` (2050) | 4,856,501 | yes | — | yes |
| p2g-control1000 | final | `model_3997.pt` (3997) | 4,857,077 | yes | p2g-control1000.onnx (1,264,064) | yes |
| p2g-ctrl1500 | final | `model_4496.pt` (4496) | 4,857,077 | yes | p2g-ctrl1500.onnx (1,264,061) | yes |
| p2g-r1-bootstrap | intermediate parent (failed run) | `model_3200.pt` (3200) | 4,857,077 | yes | — | yes |
| p2g-r1-jitter | intermediate parent (failed run) | `model_3200.pt` (3200) | 4,857,077 | yes | — | yes |
| p2g-r1-naive | intermediate parent (failed run) | `model_3200.pt` (3200) | 4,857,077 | yes | — | yes |
| p2g-r1b-jitter | final | `model_3497.pt` (3497) | 4,857,077 | yes | p2g-r1b-jitter.onnx (1,264,063) | yes |
| p2g-r1cont-a | final | `model_3996.pt` (3996) | 4,857,077 | yes | p2g-r1cont-a.onnx (1,264,061) | yes |
| p2g-r1cont-b | final | `model_4495.pt` (4495) | 4,857,077 | yes | p2g-r1cont-b.onnx (1,264,061) | yes |
| p2g-r2 | final | `model_3996.pt` (3996) | 4,857,077 | yes | p2g-r2.onnx (1,264,055) | yes |
| p2g-r2cont | final | `model_4495.pt` (4495) | 4,857,077 | yes | p2g-r2cont.onnx (1,264,059) | yes |
| p2g-r3 | final | `model_4495.pt` (4495) | 4,857,077 | yes | p2g-r3.onnx (1,264,055) | yes |

**Checkpoint format.** This is the newer rsl_rl layout. The top-level keys are `actor_state_dict`, `critic_state_dict`,
`optimizer_state_dict`, `iter`, `infos`. There is **no `model_state_dict`**. A loader that expects the old layout
(`model_state_dict`) needs the matching rsl_rl version, which is pinned in the run's `requirements.txt` under `~/recovery/wandb_files/<run>/`.
`infos` = `{env_state: {common_step_counter}, pupper_encoder_observation: {contract: pupper_encoder_observation_v1, mode: legacy_true, ...}, pupper_encoder_observation_source_checkpoint: {checkpoint_sha256, ...}}`.
All 23 checkpoints share one architecture:

```
actor  (obs dim 960 = 48 features x history_length 20; 12 actions; ELU MLP 256-128-128-128)
    obs_normalizer._mean         [1, 960]
    obs_normalizer._var          [1, 960]
    obs_normalizer._std          [1, 960]
    obs_normalizer.count         []
    distribution.std_param       [12]
    mlp.0.weight                 [256, 960]
    mlp.0.bias                   [256]
    mlp.2.weight                 [128, 256]
    mlp.2.bias                   [128]
    mlp.4.weight                 [128, 128]
    mlp.4.bias                   [128]
    mlp.6.weight                 [128, 128]
    mlp.6.bias                   [128]
    mlp.8.weight                 [12, 128]
    mlp.8.bias                   [12]
critic (obs dim 84, no history)
    obs_normalizer._mean         [1, 84]
    obs_normalizer._var          [1, 84]
    obs_normalizer._std          [1, 84]
    obs_normalizer.count         []
    mlp.0.weight                 [256, 84]
    mlp.0.bias                   [256]
    mlp.2.weight                 [128, 256]
    mlp.2.bias                   [128]
    mlp.4.weight                 [128, 128]
    mlp.4.bias                   [128]
    mlp.6.weight                 [128, 128]
    mlp.6.bias                   [128]
    mlp.8.weight                 [1, 128]
    mlp.8.bias                   [1]
```

The optimizer state includes Adam moments and the adaptive LR at save time, e.g. 1.14e-4 to 1.71e-4 for the p2g finals.

## 3. Metrics

Saved to `~/recovery/metrics/<run>.csv` and `.parquet` for all 37 runs created since 2026-11-09, one row per iteration (`_step` = iteration).
`~/recovery/metrics/_summary_last50.csv` holds the summary below with every reward term.

**Logged keys (40 to 41 per run):**
- `Episode_Reward/*` (22): abduction_angle, action_rate_l2, air_time, ang_vel_xy_l2, base_height, foot_slip, gait_tracking,
  joint_acc_l2, joint_torques_l2, knee_ground_contact, lin_vel_z_l2, orientation_l2, roll_asymmetry, self_collision_l,
  self_collision_r, stand_still_joint_velocity, stand_still_pose, termination, track_angular_velocity,
  track_linear_velocity, track_yaw_velocity, upright
- `Episode_Metrics/mean_action_acc`; `Episode_Termination/fell_over`, `Episode_Termination/time_out`
- `Metrics/twist/error_vel_xy`, `Metrics/twist/error_vel_yaw`
- `Loss/entropy`, `Loss/learning_rate`, `Loss/surrogate`, `Loss/value`; `Policy/mean_std`
- `Perf/collection_time`, `Perf/learning_time`, `Perf/total_fps`
- `Train/mean_reward`, `Train/mean_episode_length`
- `Curriculum/pretrain_rewards/active` (from-scratch runs only); `_runtime`, `_step`, `_timestamp`

These terms are identically 0 in every run: air_time, base_height, foot_slip, joint_acc_l2, joint_torques_l2, orientation_l2, self_collision_l, self_collision_r, termination, track_angular_velocity.

### 3a. Per-run summary (mean of last 50 logged iterations)
reward = Train/mean_reward; ep_len in steps (max 500 = 10 s); fell = Episode_Termination/fell_over;
s_iter = collection + learning time. s_iter and fps depend on how many runs shared the GPU (see section 4).

| run | state | iters | reward | ep_len | err_xy | err_yaw | fell | s_iter | fps |
|---|---|---|---|---|---|---|---|---|---|
| baseline-s42-wandb | finished | 2950-2999 | 164.039 | 494.718 | 0.530 | 0.465 | 0.217 | 6.419 | 15,383 |
| fast_rotary-s42 | crashed | 1040-1089 | 122.535 | 457.960 | 0.523 | 0.807 | 1.537 | 1.824 | 54,076 |
| fast_stagger-s42 | finished | 2950-2999 | 171.206 | 494.847 | 0.223 | 0.574 | 0.219 | 2.229 | 44,269 |
| fast_trot-s42 | finished | 2950-2999 | 177.237 | 492.936 | 0.204 | 0.555 | 0.306 | 2.246 | 43,814 |
| fast_wide-s42 | finished | 2950-2999 | 164.948 | 490.605 | 0.235 | 0.661 | 0.342 | 1.419 | 69,452 |
| fast_long-s42 | finished | 2950-2999 | 164.314 | 492.391 | 0.236 | 0.635 | 0.272 | 1.259 | 79,537 |
| baseline-s42-cont4500 | finished | 4449-4498 | 166.972 | 488.717 | 0.342 | 0.506 | 0.418 | 1.148 | 88,270 |
| baseline-s43 | finished | 2950-2999 | 163.225 | 495.147 | 0.587 | 0.450 | 0.181 | 1.050 | 94,176 |
| fast_long-s43 | finished | 2950-2999 | 161.601 | 490.795 | 0.237 | 0.642 | 0.382 | 1.079 | 91,323 |
| p2-r1-control | crashed | 3611-3660 | 165.125 | 490.936 | 0.229 | 0.622 | 0.310 | 3.412 | 28,877 |
| p2-r1-jitter | crashed | 3050-3099 | 160.817 | 489.299 | 0.238 | 0.635 | 0.345 | 3.437 | 28,612 |
| p2-r1-naive | crashed | 3048-3097 | 81.458 | 456.099 | 0.759 | 1.244 | 1.682 | 3.377 | 29,210 |
| p2-r1-phase | crashed | 3050-3099 | 80.208 | 459.178 | 0.770 | 1.322 | 1.463 | 3.423 | 28,738 |
| p2-r1-sym-protect | crashed | 3050-3099 | 82.222 | 464.301 | 0.793 | 1.091 | 1.290 | 3.443 | 28,565 |
| p2-r1-sym | crashed | 3050-3099 | 81.736 | 464.441 | 0.794 | 1.079 | 1.333 | 3.423 | 28,733 |
| gallop_a-s42 | failed | 2046-2095 | 129.888 | 476.932 | 0.554 | 0.723 | 0.880 | 1.827 | 53,948 |
| gallop_b-s42 | failed | 2075-2124 | 135.820 | 476.716 | 0.289 | 0.790 | 0.840 | 1.795 | 54,939 |
| gallop_c-s42 | failed | 2049-2098 | 143.609 | 479.924 | 0.271 | 0.781 | 0.775 | 1.815 | 54,357 |
| gallop_a-s42-r2 | finished | 2949-2998 | 140.523 | 487.316 | 0.562 | 0.651 | 0.524 | 1.570 | 63,197 |
| gallop_b-s42-r2 | finished | 2949-2998 | 144.117 | 483.289 | 0.259 | 0.719 | 0.636 | 1.820 | 54,127 |
| gallop_c-s42-r2 | finished | 2949-2998 | 151.838 | 486.401 | 0.251 | 0.714 | 0.523 | 1.333 | 73,815 |
| p2g-r1-bootstrap | failed | 3175-3224 | 75.492 | 477.933 | 0.822 | 0.944 | 0.953 | 2.008 | 49,027 |
| p2g-r1-jitter | failed | 3171-3220 | 76.673 | 477.279 | 0.779 | 0.905 | 0.931 | 2.014 | 48,912 |
| p2g-r1-naive | failed | 3173-3222 | 73.804 | 463.165 | 0.768 | 0.993 | 1.300 | 2.016 | 48,841 |
| p2g-r1-bootstrap-r2 | finished | 3948-3997 | 90.033 | 489.573 | 0.789 | 0.535 | 0.380 | 3.258 | 30,201 |
| p2g-r1-naive-r2 | finished | 3948-3997 | 92.142 | 491.618 | 0.790 | 0.575 | 0.282 | 3.034 | 32,926 |
| p2g-r1-jitter-r2 | finished | 3948-3997 | 89.498 | 490.121 | 0.776 | 0.540 | 0.312 | 2.456 | 46,749 |
| p2g-ctrl-check | finished | 3248-3297 | 153.815 | 488.536 | 0.246 | 0.704 | 0.450 | 3.152 | 31,210 |
| p2g-r1b-bootstrap | finished | 3448-3497 | 160.216 | 488.329 | 0.235 | 0.690 | 0.448 | 2.302 | 43,002 |
| p2g-r1b-jitter | finished | 3448-3497 | 162.021 | 490.436 | 0.233 | 0.685 | 0.383 | 2.254 | 44,319 |
| p2g-r2 | finished | 3947-3996 | 164.859 | 491.571 | 0.225 | 0.661 | 0.316 | 2.275 | 43,229 |
| p2g-control1000 | finished | 3948-3997 | 159.398 | 491.223 | 0.237 | 0.657 | 0.318 | 2.101 | 46,796 |
| p2g-r3 | finished | 4446-4495 | 166.812 | 492.821 | 0.219 | 0.642 | 0.289 | 3.116 | 31,566 |
| p2g-ctrl1500 | finished | 4447-4496 | 161.598 | 491.758 | 0.232 | 0.639 | 0.322 | 4.794 | 20,512 |
| p2g-r1cont-a | finished | 3947-3996 | 163.864 | 490.823 | 0.227 | 0.664 | 0.346 | 4.774 | 20,618 |
| p2g-r2cont | finished | 4446-4495 | 168.231 | 493.241 | 0.218 | 0.642 | 0.263 | 4.721 | 20,852 |
| p2g-r1cont-b | finished | 4446-4495 | 167.112 | 491.718 | 0.223 | 0.643 | 0.311 | 2.904 | 33,857 |

### 3b. Reward terms (last-50 mean, non-zero terms only)

| run | abduction_angle | action_rate_l2 | ang_vel_xy_l2 | gait_tracking | knee_ground_contact | lin_vel_z_l2 | roll_asymmetry | stand_still_joint_velocity | stand_still_pose | track_linear_velocity | track_yaw_velocity | upright |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline-s42-wandb | 0.000 | -0.192 | -0.182 | 7.743 | -0.000 | -0.009 | 0.000 | 0.000 | 0.000 | 4.103 | 3.955 | 0.943 |
| fast_rotary-s42 | 0.000 | -0.317 | -0.361 | 6.284 | -0.010 | -0.019 | 0.000 | 0.000 | 0.000 | 3.604 | 2.322 | 0.809 |
| fast_stagger-s42 | 0.000 | -0.291 | -0.249 | 6.867 | -0.003 | -0.012 | 0.000 | 0.000 | 0.000 | 6.432 | 3.469 | 0.926 |
| fast_trot-s42 | 0.000 | -0.298 | -0.227 | 6.947 | -0.002 | -0.010 | 0.000 | 0.000 | 0.000 | 6.841 | 3.535 | 0.932 |
| fast_wide-s42 | 0.000 | -0.247 | -0.244 | 6.662 | -0.008 | -0.011 | 0.000 | 0.000 | 0.000 | 6.321 | 3.108 | 0.924 |
| fast_long-s42 | 0.000 | -0.336 | -0.243 | 6.534 | -0.009 | -0.011 | 0.000 | 0.000 | 0.000 | 6.325 | 3.224 | 0.922 |
| baseline-s42-cont4500 | 0.000 | -0.290 | -0.216 | 7.068 | -0.000 | -0.010 | 0.000 | 0.000 | 0.000 | 5.502 | 3.697 | 0.927 |
| baseline-s43 | 0.000 | -0.191 | -0.167 | 7.885 | -0.000 | -0.008 | 0.000 | 0.000 | 0.000 | 3.735 | 4.063 | 0.946 |
| fast_long-s43 | 0.000 | -0.351 | -0.251 | 6.445 | -0.009 | -0.011 | 0.000 | 0.000 | 0.000 | 6.240 | 3.163 | 0.914 |
| p2-r1-control | 0.000 | -0.350 | -0.235 | 6.498 | -0.009 | -0.011 | 0.000 | 0.000 | 0.000 | 6.452 | 3.289 | 0.923 |
| p2-r1-jitter | 0.000 | -0.330 | -0.247 | 6.393 | -0.011 | -0.011 | 0.000 | -0.042 | -0.010 | 6.267 | 3.207 | 0.916 |
| p2-r1-naive | 0.000 | -0.302 | -0.245 | 2.994 | -0.271 | -0.015 | 0.000 | 0.000 | 0.000 | 3.083 | 2.038 | 0.782 |
| p2-r1-phase | 0.000 | -0.306 | -0.252 | 3.007 | -0.321 | -0.014 | 0.000 | 0.000 | 0.000 | 3.095 | 2.016 | 0.791 |
| p2-r1-sym-protect | 0.000 | -0.300 | -0.270 | 3.071 | -0.275 | -0.013 | -0.030 | 0.000 | 0.000 | 3.113 | 2.183 | 0.776 |
| p2-r1-sym | 0.000 | -0.302 | -0.274 | 3.029 | -0.271 | -0.013 | 0.000 | 0.000 | 0.000 | 3.101 | 2.193 | 0.771 |
| gallop_a-s42 | -0.015 | -0.426 | -0.358 | 6.133 | -0.059 | -0.022 | -0.007 | 0.000 | 0.000 | 4.103 | 2.765 | 0.840 |
| gallop_b-s42 | -0.015 | -0.477 | -0.378 | 5.887 | -0.055 | -0.014 | -0.008 | 0.000 | 0.000 | 5.182 | 2.617 | 0.840 |
| gallop_c-s42 | -0.015 | -0.438 | -0.346 | 6.228 | -0.050 | -0.013 | -0.008 | 0.000 | 0.000 | 5.466 | 2.671 | 0.856 |
| gallop_a-s42-r2 | -0.016 | -0.450 | -0.339 | 6.332 | -0.050 | -0.021 | -0.007 | 0.000 | 0.000 | 4.580 | 3.131 | 0.868 |
| gallop_b-s42-r2 | -0.017 | -0.503 | -0.355 | 5.949 | -0.050 | -0.013 | -0.008 | 0.000 | 0.000 | 5.704 | 2.881 | 0.857 |
| gallop_c-s42-r2 | -0.016 | -0.474 | -0.329 | 6.325 | -0.044 | -0.012 | -0.008 | 0.000 | 0.000 | 5.925 | 2.946 | 0.872 |
| p2g-r1-bootstrap | -0.017 | -0.527 | -0.304 | 2.717 | -0.103 | -0.016 | -0.009 | 0.000 | 0.000 | 2.799 | 2.329 | 0.706 |
| p2g-r1-jitter | -0.016 | -0.510 | -0.296 | 2.681 | -0.116 | -0.015 | -0.010 | -0.047 | -0.013 | 2.831 | 2.371 | 0.679 |
| p2g-r1-naive | -0.016 | -0.521 | -0.315 | 2.732 | -0.103 | -0.018 | -0.015 | 0.000 | 0.000 | 2.829 | 2.158 | 0.715 |
| p2g-r1-bootstrap-r2 | -0.024 | -0.673 | -0.217 | 2.636 | -0.034 | -0.013 | -0.008 | 0.000 | 0.000 | 2.829 | 3.811 | 0.647 |
| p2g-r1-naive-r2 | -0.027 | -0.677 | -0.234 | 2.920 | -0.101 | -0.018 | -0.025 | 0.000 | 0.000 | 2.988 | 3.498 | 0.869 |
| p2g-r1-jitter-r2 | -0.025 | -0.662 | -0.211 | 2.651 | -0.106 | -0.011 | -0.011 | -0.057 | -0.019 | 2.898 | 3.740 | 0.772 |
| p2g-ctrl-check | -0.017 | -0.500 | -0.319 | 6.344 | -0.041 | -0.012 | -0.008 | 0.000 | 0.000 | 6.052 | 3.010 | 0.880 |
| p2g-r1b-bootstrap | -0.016 | -0.489 | -0.328 | 6.691 | -0.044 | -0.011 | -0.007 | 0.000 | 0.000 | 6.281 | 3.067 | 0.886 |
| p2g-r1b-jitter | -0.016 | -0.476 | -0.322 | 6.776 | -0.039 | -0.011 | -0.007 | -0.039 | -0.011 | 6.334 | 3.097 | 0.892 |
| p2g-r2 | -0.017 | -0.483 | -0.323 | 6.784 | -0.039 | -0.010 | -0.007 | -0.035 | -0.015 | 6.529 | 3.199 | 0.898 |
| p2g-control1000 | -0.017 | -0.484 | -0.320 | 6.459 | -0.042 | -0.011 | -0.007 | -0.038 | -0.012 | 6.329 | 3.209 | 0.892 |
| p2g-r3 | -0.017 | -0.510 | -0.329 | 6.786 | -0.038 | -0.010 | -0.007 | -0.027 | -0.017 | 6.647 | 3.293 | 0.899 |
| p2g-ctrl1500 | -0.017 | -0.492 | -0.315 | 6.505 | -0.042 | -0.011 | -0.007 | -0.038 | -0.012 | 6.408 | 3.273 | 0.893 |
| p2g-r1cont-a | -0.017 | -0.484 | -0.319 | 6.774 | -0.040 | -0.010 | -0.007 | -0.032 | -0.014 | 6.449 | 3.190 | 0.893 |
| p2g-r2cont | -0.017 | -0.483 | -0.317 | 6.850 | -0.039 | -0.010 | -0.007 | -0.028 | -0.017 | 6.669 | 3.303 | 0.904 |
| p2g-r1cont-b | -0.017 | -0.476 | -0.315 | 6.828 | -0.041 | -0.010 | -0.007 | -0.029 | -0.019 | 6.591 | 3.302 | 0.900 |

### 3c. Part 1: baseline vs fast_* vs gallop_* at 3000 iterations

| run | iters | reward | ep_len | err_xy | err_yaw | fell | gait_tracking | track_linear_velocity | track_yaw_velocity |
|---|---|---|---|---|---|---|---|---|---|
| baseline-s42-wandb | 2950-2999 | 164.039 | 494.718 | 0.530 | 0.465 | 0.217 | 7.743 | 4.103 | 3.955 |
| baseline-s43 | 2950-2999 | 163.225 | 495.147 | 0.587 | 0.450 | 0.181 | 7.885 | 3.735 | 4.063 |
| fast_long-s42 | 2950-2999 | 164.314 | 492.391 | 0.236 | 0.635 | 0.272 | 6.534 | 6.325 | 3.224 |
| fast_long-s43 | 2950-2999 | 161.601 | 490.795 | 0.237 | 0.642 | 0.382 | 6.445 | 6.240 | 3.163 |
| fast_trot-s42 | 2950-2999 | 177.237 | 492.936 | 0.204 | 0.555 | 0.306 | 6.947 | 6.841 | 3.535 |
| fast_stagger-s42 | 2950-2999 | 171.206 | 494.847 | 0.223 | 0.574 | 0.219 | 6.867 | 6.432 | 3.469 |
| fast_wide-s42 | 2950-2999 | 164.948 | 490.605 | 0.235 | 0.661 | 0.342 | 6.662 | 6.321 | 3.108 |
| gallop_a-s42-r2 | 2949-2998 | 140.523 | 487.316 | 0.562 | 0.651 | 0.524 | 6.332 | 4.580 | 3.131 |
| gallop_b-s42-r2 | 2949-2998 | 144.117 | 483.289 | 0.259 | 0.719 | 0.636 | 5.949 | 5.704 | 2.881 |
| gallop_c-s42-r2 | 2949-2998 | 151.838 | 486.401 | 0.251 | 0.714 | 0.523 | 6.325 | 5.925 | 2.946 |
| baseline-s42-cont4500 | 4449-4498 | 166.972 | 488.717 | 0.342 | 0.506 | 0.418 | 7.068 | 5.502 | 3.697 |

- At 3000 iterations, **fast_trot-s42** has the highest total reward (177.2) and the lowest error_vel_xy (0.204). fast_stagger-s42 is second (171.2 / 0.223).
- The baseline (default reference) tracks linear velocity much worse (err_xy 0.53 to 0.59 on two seeds) but has the best yaw error (0.45 to 0.47).
- The gallop_* runs score lower on total reward because they carry extra penalty terms and a harder reference. gallop_c has the best gallop numbers (151.8, err_xy 0.251), and its fell_over rate (~0.52) is about twice that of fast_*.
- The two seeds agree closely (baseline 164.0 vs 163.2; fast_long 164.3 vs 161.6).

### 3d. Part 2: iteration-matched comparison (parent gallop_c-s42-r2 at 2998)
Each row is the mean over the 50 iterations ending at the listed iteration. The +500 control row is p2g-control1000 at iteration 3497.

| Δiter | run | iters | ref / weights | reward | ep_len | err_xy | err_yaw | fell | gait_tracking | track_linear_velocity | track_yaw_velocity | action_rate_l2 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| parent | gallop_c-s42-r2 | 2949-2998 | fast_gallop_c / splay | 151.838 | 486.401 | 0.251 | 0.714 | 0.523 | 6.325 | 5.925 | 2.946 | -0.474 |
| +300 | p2g-ctrl-check | 3248-3297 | fast_gallop_c / splay (control check) | 153.815 | 488.536 | 0.246 | 0.704 | 0.450 | 6.344 | 6.052 | 3.010 | -0.500 |
| +500 | p2g-r1b-jitter | 3448-3497 | p2r1b_gallop_c_lag_m2 / splay_stand_still | 162.021 | 490.436 | 0.233 | 0.685 | 0.383 | 6.776 | 6.334 | 3.097 | -0.476 |
| +500 | p2g-r1b-bootstrap | 3448-3497 | p2r1b_gallop_c_lag_m2 / splay | 160.216 | 488.329 | 0.235 | 0.690 | 0.448 | 6.691 | 6.281 | 3.067 | -0.489 |
| +500 | p2g-control1000 | 3448-3497 | fast_gallop_c / splay_stand_still (control, mid-run) | 156.426 | 490.510 | 0.241 | 0.680 | 0.368 | 6.396 | 6.192 | 3.106 | -0.488 |
| +1000 | p2g-r2 | 3947-3996 | p2r2b_lag_m2 / splay_stand_still | 164.859 | 491.571 | 0.225 | 0.661 | 0.316 | 6.784 | 6.529 | 3.199 | -0.483 |
| +1000 | p2g-r1cont-a | 3947-3996 | p2r1b_gallop_c_lag_m2 (r1b continued) | 163.864 | 490.823 | 0.227 | 0.664 | 0.346 | 6.774 | 6.449 | 3.190 | -0.484 |
| +1000 | p2g-r1-jitter-r2 | 3948-3997 | p2r1_gallop_c_lag / splay_stand_still (old r1) | 89.498 | 490.121 | 0.776 | 0.540 | 0.312 | 2.651 | 2.898 | 3.740 | -0.662 |
| +1000 | p2g-r1-bootstrap-r2 | 3948-3997 | p2r1_gallop_c_lag / splay (old r1) | 90.033 | 489.573 | 0.789 | 0.535 | 0.380 | 2.636 | 2.829 | 3.811 | -0.673 |
| +1000 | p2g-r1-naive-r2 | 3948-3997 | p2r1_gallop_c_naive / splay (old r1) | 92.142 | 491.618 | 0.790 | 0.575 | 0.282 | 2.920 | 2.988 | 3.498 | -0.677 |
| +1000 | p2g-control1000 | 3948-3997 | fast_gallop_c / splay_stand_still (control) | 159.398 | 491.223 | 0.237 | 0.657 | 0.318 | 6.459 | 6.329 | 3.209 | -0.484 |
| +1500 | p2g-r3 | 4446-4495 | p2r3_lag_m2 / splay_stand_still | 166.812 | 492.821 | 0.219 | 0.642 | 0.289 | 6.786 | 6.647 | 3.293 | -0.510 |
| +1500 | p2g-r2cont | 4446-4495 | p2r2b_lag_m2 (r2 continued) | 168.231 | 493.241 | 0.218 | 0.642 | 0.263 | 6.850 | 6.669 | 3.303 | -0.483 |
| +1500 | p2g-r1cont-b | 4446-4495 | p2r1b_gallop_c_lag_m2 (r1b continued x2) | 167.112 | 491.718 | 0.223 | 0.643 | 0.311 | 6.828 | 6.591 | 3.302 | -0.476 |
| +1500 | p2g-ctrl1500 | 4447-4496 | fast_gallop_c / splay_stand_still (control) | 161.598 | 491.758 | 0.232 | 0.639 | 0.322 | 6.505 | 6.408 | 3.273 | -0.492 |

- **Iteration-matched gains over the control (fast_gallop_c reference, same splay_stand_still weights):**
  - At +500: p2g-r1b-jitter has err_xy 0.233 vs 0.241 for the control, and total reward 162.0 vs 156.4.
  - At +1000: p2g-r2 has 0.225 vs 0.237, and 164.9 vs 159.4.
  - At +1500: p2g-r3 has 0.219 vs 0.232, and 166.8 vs 161.6. p2g-r2cont has the best numbers overall: err_xy 0.218, reward 168.2, fell 0.263.
- **Yaw error.** At +1000 and +1500 the yaw error matches the control (~0.64 to 0.66). The gain is in xy tracking and stability.
- **Continuations vs a new reference stage.** The continuations (r1cont-a/b, r2cont) are statistically indistinguishable from the r2/r3 stage changes at matched iterations. Gains of about 0.005 in err_xy are within noise.
- **The old p2r1 references failed.** p2g-r1-*-r2 used the p2r1_gallop_c_{lag,naive} references and collapsed: err_xy 0.78, gait_tracking about 2.7. The "_m2" reference fixed this.

## 4. Throughput (4096 envs, one A10)

Each logged iteration is tagged with the number of WandB runs active at that wall-clock moment (`conc`, which counts the run itself).
Iterations over 60 s were dropped as resume/first-iteration outliers.

| conc | n_iters | runs | s_iter_med | coll_med | learn_med | wall_med | fps_med | agg_fps |
|---|---|---|---|---|---|---|---|---|
| 1.00 | 4,691 | 19.00 | 1.21 | 0.97 | 0.22 | 1.25 | 81,570 | 81,570 |
| 2.00 | 7,761 | 23.00 | 1.08 | 0.88 | 0.20 | 1.11 | 90,900 | 181,800 |
| 3.00 | 18,770 | 27.00 | 1.82 | 1.51 | 0.31 | 1.86 | 54,026 | 162,076 |
| 4.00 | 8,734 | 9.00 | 1.83 | 1.53 | 0.29 | 1.87 | 53,657 | 214,628 |
| 5.00 | 1,708 | 11.00 | 2.23 | 1.86 | 0.35 | 2.28 | 44,056 | 220,278 |
| 6.00 | 577.00 | 6.00 | 3.44 | 2.85 | 0.56 | 3.50 | 28,600 | 171,600 |

agg_fps = per-run fps × conc (approximate total GPU throughput).

**Within-run comparison** (the same run at different concurrency levels):

| run | conc | n | s_iter | fps |
|---|---|---|---|---|
| baseline-s42-wandb | 1 | 2423 | 0.79 | 125,181 |
| baseline-s42-wandb | 3 | 129 | 3.36 | 29,285 |
| baseline-s42-wandb | 4 | 367 | 5.29 | 18,567 |
| baseline-s42-wandb | 5 | 81 | 6.56 | 14,991 |
| fast_long-s42 | 2 | 26 | 1.09 | 90,502 |
| fast_long-s42 | 3 | 1319 | 1.45 | 67,987 |
| fast_long-s42 | 4 | 1281 | 1.83 | 53,592 |
| fast_long-s42 | 5 | 374 | 2.19 | 44,939 |
| fast_trot-s42 | 3 | 280 | 1.49 | 65,855 |
| fast_trot-s42 | 4 | 2352 | 1.83 | 53,672 |
| fast_trot-s42 | 5 | 368 | 2.25 | 43,658 |
| fast_wide-s42 | 3 | 1340 | 1.44 | 68,202 |
| fast_wide-s42 | 4 | 1285 | 1.84 | 53,551 |
| fast_wide-s42 | 5 | 375 | 2.19 | 44,925 |
| gallop_a-s42-r2 | 1 | 26 | 1.32 | 74,420 |
| gallop_a-s42-r2 | 2 | 64 | 1.48 | 66,234 |
| gallop_a-s42-r2 | 3 | 859 | 1.86 | 52,931 |
| gallop_b-s42-r2 | 2 | 23 | 1.49 | 65,939 |
| gallop_b-s42-r2 | 3 | 876 | 1.83 | 53,648 |
| gallop_c-s42-r2 | 1 | 48 | 1.32 | 74,350 |
| gallop_c-s42-r2 | 2 | 39 | 1.48 | 66,403 |
| gallop_c-s42-r2 | 3 | 862 | 1.85 | 53,040 |
| p2-r1-control | 1 | 564 | 1.30 | 75,422 |
| p2-r1-control | 6 | 98 | 3.44 | 28,545 |
| p2g-control1000 | 1 | 500 | 2.11 | 46,610 |
| p2g-control1000 | 2 | 500 | 3.12 | 31,538 |
| p2g-ctrl-check | 1 | 68 | 1.24 | 79,152 |
| p2g-ctrl-check | 3 | 232 | 3.18 | 30,882 |
| p2g-r1-naive-r2 | 2 | 23 | 2.82 | 34,896 |
| p2g-r1-naive-r2 | 3 | 775 | 3.81 | 25,804 |
| p2g-r1b-bootstrap | 2 | 269 | 2.32 | 42,350 |
| p2g-r1b-bootstrap | 3 | 231 | 3.18 | 30,923 |

- **Solo speed.** One run alone does **~0.8 s/iter (~125k fps)**; baseline-s42-wandb did this for its first 2400 iterations. Later solo runs did **~1.2 to 1.3 s/iter (~75 to 80k fps)**: p2-r1-control, gallop_c-s42-r2, p2g-ctrl-check, p2g-r2.
- **Concurrency slows each run.** Each run gets slower as more runs share the GPU:
  - 2 runs: ~1.1 to 1.5 s/iter
  - 3 runs: ~1.45 to 1.85 s/iter
  - 4 runs: ~1.83 s/iter
  - 5 runs: ~2.2 s/iter
  - 6 runs: ~3.4 s/iter
- **Total throughput still rises.** Aggregate throughput goes up to ~215 to 220k fps at 4 to 5 runs, compared with 80 to 125k solo. Packing runs helps total throughput but delays each run's wall-clock time.
- **Uneven penalty.** The longest-running job can be hit much harder. baseline-s42-wandb dropped from 0.79 to **5.3 to 6.6 s/iter** when the four fast_* runs started, while the fast_* runs themselves ran at ~1.8.
- **Unexplained slow periods.** Some periods are slow beyond what WandB-visible concurrency explains. p2g-control1000 ran at 2.1 s/iter while it was the only WandB run. The 01:36 batch (ctrl1500, r1cont-a, r2cont) ran at ~5.9 s/iter with 3 runs, versus ~1.85 for the gallop trio. p2g-r1-*-r2 ran at ~3.8 with 3 runs. This suggests other GPU work that did not log to WandB (eval, play or video export, or a non-WandB run).
- **Crashes.** Every crash or failure happened in a batch of 3 to 6 concurrent runs, and the whole batch died at once.

**Per-run timing:**

| run | start | end | conc_med | conc_max | s_iter_med | fps_med |
|---|---|---|---|---|---|---|
| baseline-s42-wandb | 11-09 13:42 | 11-09 15:09 | 1.00 | 5 | 0.79 | 124,676 |
| fast_rotary-s42 | 11-09 14:20 | 11-09 14:53 | 4.00 | 4 | 1.77 | 55,688 |
| fast_stagger-s42 | 11-09 14:20 | 11-09 15:55 | 4.00 | 5 | 1.83 | 53,630 |
| fast_trot-s42 | 11-09 14:20 | 11-09 15:54 | 4.00 | 5 | 1.83 | 53,602 |
| fast_wide-s42 | 11-09 15:00 | 11-09 16:28 | 4.00 | 5 | 1.75 | 56,255 |
| fast_long-s42 | 11-09 15:00 | 11-09 16:29 | 4.00 | 5 | 1.76 | 55,882 |
| baseline-s42-cont4500 | 11-09 15:49 | 11-09 16:29 | 3.00 | 5 | 1.44 | 68,079 |
| baseline-s43 | 11-09 16:32 | 11-09 17:28 | 2.00 | 2 | 1.08 | 91,218 |
| fast_long-s43 | 11-09 16:32 | 11-09 17:28 | 2.00 | 2 | 1.08 | 91,196 |
| p2-r1-control | 11-09 18:20 | 11-09 18:39 | 1.00 | 6 | 1.31 | 75,219 |
| p2-r1-jitter | 11-09 18:33 | 11-09 18:39 | 6.00 | 6 | 3.44 | 28,579 |
| p2-r1-naive | 11-09 18:33 | 11-09 18:39 | 6.00 | 6 | 3.42 | 28,762 |
| p2-r1-phase | 11-09 18:33 | 11-09 18:39 | 6.00 | 6 | 3.43 | 28,628 |
| p2-r1-sym-protect | 11-09 18:33 | 11-09 18:39 | 6.00 | 6 | 3.45 | 28,525 |
| p2-r1-sym | 11-09 18:33 | 11-09 18:39 | 6.00 | 6 | 3.44 | 28,578 |
| gallop_a-s42 | 11-09 18:46 | 11-09 19:51 | 3.00 | 3 | 1.81 | 54,284 |
| gallop_b-s42 | 11-09 18:46 | 11-09 19:51 | 3.00 | 3 | 1.80 | 54,733 |
| gallop_c-s42 | 11-09 18:46 | 11-09 19:51 | 3.00 | 3 | 1.81 | 54,352 |
| gallop_a-s42-r2 | 11-09 19:52 | 11-09 20:22 | 3.00 | 3 | 1.85 | 53,095 |
| gallop_b-s42-r2 | 11-09 19:53 | 11-09 20:21 | 3.00 | 3 | 1.83 | 53,697 |
| gallop_c-s42-r2 | 11-09 19:53 | 11-09 20:23 | 3.00 | 3 | 1.85 | 53,192 |
| p2g-r1-bootstrap | 11-09 20:27 | 11-09 20:34 | 3.00 | 3 | 2.00 | 49,031 |
| p2g-r1-jitter | 11-09 20:27 | 11-09 20:34 | 3.00 | 3 | 2.02 | 48,616 |
| p2g-r1-naive | 11-09 20:27 | 11-09 20:34 | 3.00 | 3 | 2.01 | 48,837 |
| p2g-r1-bootstrap-r2 | 11-09 20:35 | 11-09 21:30 | 3.00 | 3 | 3.84 | 25,578 |
| p2g-r1-naive-r2 | 11-09 20:36 | 11-09 21:31 | 3.00 | 3 | 3.80 | 25,841 |
| p2g-r1-jitter-r2 | 11-09 20:36 | 11-09 21:31 | 3.00 | 3 | 3.84 | 25,570 |
| p2g-ctrl-check | 11-09 21:37 | 11-09 21:52 | 3.00 | 3 | 3.17 | 30,995 |
| p2g-r1b-bootstrap | 11-09 21:39 | 11-09 22:02 | 2.00 | 3 | 2.34 | 41,925 |
| p2g-r1b-jitter | 11-09 21:39 | 11-09 22:02 | 2.00 | 3 | 2.34 | 41,991 |
| p2g-r2 | 11-09 22:09 | 11-09 22:20 | 1.00 | 1 | 1.24 | 79,108 |
| p2g-control1000 | 11-09 22:26 | 11-09 23:12 | 1.50 | 2 | 2.87 | 34,251 |
| p2g-r3 | 11-09 22:29 | 11-09 22:56 | 2.00 | 2 | 3.11 | 31,597 |
| p2g-ctrl1500 | 11-10 01:36 | 11-10 02:24 | 3.00 | 3 | 5.90 | 16,666 |
| p2g-r1cont-a | 11-10 01:36 | 11-10 02:24 | 3.00 | 3 | 5.94 | 16,535 |
| p2g-r2cont | 11-10 01:36 | 11-10 02:24 | 3.00 | 3 | 5.91 | 16,631 |
| p2g-r1cont-b | 11-10 02:25 | 11-10 02:56 | 1.00 | 1 | 2.94 | 33,439 |


## 5. Recommended warm-start checkpoints

1. **Part 2 (gallop_c lineage), best overall:** `~/recovery/checkpoints/p2g-r2cont/model_4495.pt`. It has err_xy 0.218, reward 168.2 and the lowest fell_over (0.263). Its ONNX is `p2g-r2cont.onnx`, and it was trained on ref `p2r2b_lag_m2` with w=`splay_stand_still`.
   - **Alternative, if the next stage should build on the p2r3 reference:** `p2g-r3/model_4495.pt` (err_xy 0.219, 166.8).
2. **Part 2 restart point / clean parent:** `gallop_c-s42-r2/model_2998.pt`. Every p2g run descends from it. Use it to re-run the matched comparisons, and use `p2g-control1000/model_3997.pt` or `p2g-ctrl1500/model_4496.pt` as control endpoints.
3. **Part 1 (non-gallop gait), best overall:** `fast_trot-s42/model_2999.pt` (reward 177.2, err_xy 0.204). The baseline continuation is `baseline-s42-cont4500/model_4498.pt`.

**Warm-start notes:**
- Reuse the same `config.yaml` (the actor obs dim is 960 and needs history 20).
- Pass the matching `--reference-dir` and `--weights` files. These live in the mts_123 repo at the listed commit, under `pupper-mjlab/configs/opt-lab-2/...`, and are **not** stored on WandB.
- Run 2 to 3 runs at once at most. Every crash on record happened with 3 or more concurrent runs.
