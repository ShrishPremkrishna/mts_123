# Reward shaping for the Part 3 bumpy policy (simulation only)

2026-11-19. Candidate changes to fix the problems seen in **Bumpy +2000**
(`~/runs/opt-lab-2/rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt`). Nothing here has been trained yet.
All numbers are from simulation.

## The short version

- **Problems 1 and 3 are one defect.** While standing, the bumpy policy tucks its **left hind leg in by
  0.38 rad** and puts 59 % of its weight on the left feet. Shoves toward its left knock it over 97 % of the
  time at 0.65 m/s; shoves toward its right, 2 %. The 0.49 "standing push" fall rate is the average of
  those two numbers.
- **Standing is barely trained.** Only 2 % of training commands are "stand still" (`rel_standing_envs=0.02`),
  so every stand-only reward term is about 1 % of the return. The terms that should prevent the lean
  (`roll_asymmetry`, `stand_still_pose`) exist but are too weak where they act.
- **Training never shoves the robot** (`push_robot` is removed in `pupper_env_cfg.py:302`).
- **Action rate is 18–28 % higher than the flat control while walking** and 63 % higher while standing.

Top changes, in priority order: (1) raise the standing share of commands, (2) `roll_asymmetry` −1 → −10,
(3) turn on linear shoves in training, (4) `stand_still_pose` −0.5 → −2, (5) `action_rate_l2` −0.02 → −0.05.

## Terms used

| Term | Meaning |
|---|---|
| Policy names | **Bumpy** = Bumpy +2000 (model_6492). **Flat ctrl** = flat control +2000 (same weights and iterations, flat ground). **P** = Part 2 final (model_4496), the warm start for both. |
| flat_dr / flat_nodr | Flat-ground test with Lab 4 randomization / at nominal motor gains (kp 5.5, kd 0.25, no mechanical randomization). |
| Motor 2 | The leg-roll (abduction) joint, `leg_*_2`. It moves the foot in or out sideways. In the default pose it is 0. |
| Hind pair sum | `q(leg_back_r_2) + q(leg_back_l_2)` in rad. A left-right mirrored pose gives 0 (the left joints are the negated right joints). A sustained non-zero value means one hind leg is rolled and its partner is not. This is exactly what the `roll_asymmetry` term measures. |
| Left load share | Left feet's share of total foot contact force. 0.50 is balanced. |
| Action rate | Σ over the 12 joints of (a_t − a_{t−1})², per 0.02 s control step, on the raw policy output. This is the quantity the `action_rate_l2` term penalizes. |
| Standing push fall rate | Robot stands still, gets one sideways velocity kick; share of kicks followed by a fall (body tilt > 30°) within 2 s. |
| v50 | Kick speed at which half of standing robots fall (from `push_sweep/summary.md`). |
| Weight, reward per second | mjlab multiplies each term by weight × dt, and logs `Episode_Reward/<term>` as the episode sum ÷ 10 s. So a logged value is that term's average contribution **per second**. The positive terms total about **17 per second** in the bumpy run. |
| Stand-gated | A term that is only non-zero when the command is near zero (|command| < 0.05–0.1). |

## What was measured

Two new short evals (each under 4 minutes, run through `gpu_slot_run.py`, eval seed 1234; scripts and raw
output in [`reward_shaping_probe/`](reward_shaping_probe/)):

- `shape_probe.py`: 96 robots per fixed command (stand, ±0.5 and ±1.5 m/s, yaw ±1 rad/s, sidestep
  ±0.3 m/s), 20 s, first 3 s dropped, flat_dr and flat_nodr. No robot fell.
- `push_side_probe.py`: 2048 standing robots, one kick at t = 5 s, 0.55 or 0.65 m/s, half to the robot's
  left and half to its right, flat_dr.

### Standing posture (flat_dr; flat_nodr in brackets)

| Metric | Bumpy | Flat ctrl | P |
|---|---|---|---|
| Hind pair sum (rad) | **0.43** [0.38] | 0.04 [0.05] | 0.15 [0.17] |
| Front pair sum (rad) | −0.10 | 0.06 | 0.05 |
| Mean deviation of `leg_back_l_2` from default (rad) | **+0.38** (tucked in) | +0.09 | +0.13 |
| Mirrored hind-foot position mismatch (cm) | **5.8** | 1.0 | 2.1 |
| Left load share | **0.59** | 0.51 | 0.53 |
| Body roll (deg, + = left side down) | 1.5 | 0.0 | −0.2 |
| Stance width front / hind (cm) | **12.6 / 13.7** | 15.1 / 16.3 | 14.8 / 16.7 |
| Total joint deviation from default, L1 (rad; what `stand_still_pose` penalizes) | **1.26** | 0.79 | 0.73 |
| Action rate | **0.65** [0.64] | 0.39 [0.40] | 0.40 [0.40] |
| Share of action power above 6 Hz | 0.93 | 0.94 | 0.91 |
| Joint speed, mean abs (rad/s; = existing "jitter" metric) | 0.105 [0.141] | 0.098 [0.142] | 0.140 [0.202] |

The lean is systematic: every one of the 96 robots leans the same way (mean |pair sum| per robot equals
the signed mean). It survives nominal gains, so it is not caused by soft random motors.

### Standing push, split by kick direction (flat_dr, 512 kicks per cell)

| Kick | Bumpy | Flat ctrl | P |
|---|---|---|---|
| 0.55 m/s to the left | **0.59** | 0.00 | 0.00 |
| 0.55 m/s to the right | 0.00 | 0.00 | 0.00 |
| 0.65 m/s to the left | **0.97** | 0.13 | 0.19 |
| 0.65 m/s to the right | 0.02 | 0.04 | 0.00 |
| Both sides at 0.65 (compare: ranked metric) | 0.50 (0.49) | 0.08 (0.09) | 0.09 (0.10) |

The bottom row reproduces the ranked push numbers, so the probe matches the standard harness. P already had
a weak left side (0.19 vs 0.00) with a smaller lean (0.15 rad); bumpy training tripled the lean.

### Walking

| Action rate | Bumpy | Flat ctrl | P |
|---|---|---|---|
| fwd 0.5 m/s | **7.9** | 6.7 | 6.4 |
| fwd 1.5 m/s | **13.6** | 10.6 | 11.1 |
| back 0.5 m/s | 7.1 | 6.3 | 6.0 |
| back 1.5 m/s | **15.8** | 12.4 | 13.4 |
| sidestep −0.3 m/s | 4.9 | 3.5 | 3.6 |

While walking, the left-right pose is symmetric for all three (|hind pair sum| ≤ 0.05 rad at ±0.5 and
±1.5 m/s). One asymmetry is common to all three and therefore not bumpy-specific: at +0.5 m/s the right hind
foot is down 77 % of the time and the left hind 53 %. It likely comes from the shared gallop C reference.

### Why the current terms do not stop this (training logs, last 100 iterations of leg 2, per second)

From `diag/training_log_diag.json`, bumpy arm. Positive terms total 17.1 per second.

| Term | Weight | Logged per second | Share of positive reward |
|---|---|---|---|
| `action_rate_l2` | −0.02 | −0.31 (flat ctrl −0.24) | 1.8 % |
| `stand_still_joint_velocity` | −0.1 | −0.041 | 0.2 % |
| `stand_still_pose` | −0.5 | −0.010 | 0.06 % |
| `roll_asymmetry` | −1 | −0.0066 | 0.04 % |
| `abduction_angle` | −0.1 | −0.012 | 0.07 % |

While standing, the lean costs the bumpy policy only 0.097 per second through `roll_asymmetry`
(((0.43)² + (0.10)²) / 2 × 1). Standing is about 2 % of training time, so averaged over training the cost is
near zero. Bumpy training also ran with more exploration noise: action std 0.43 vs 0.34 for the flat control.

## Candidate changes

Sign: − = penalty, + = bonus. "Per second at stand" uses the bumpy standing numbers above.

| # | Change | Sign | Current → proposed | Targets | Mechanism (one line) | Evidence | Risk / side effect |
|---|---|---|---|---|---|---|---|
| 1 | **Standing share of commands** `rel_standing_envs` (**not a reward**) | – | 0.02 → **0.15–0.25** | 1, 2, 3 | 8–12× more standing experience, so every stand-gated term and stand-time fall finally carries weight | All three standing defects are absent while walking; stand-gated terms are ≤ 0.2 % of reward | Less walking data per iteration; check ±1.5 m/s tracking error. Needs a CLI/config flag (not in the weights JSON) |
| 2 | `roll_asymmetry` | − | −1 → **−10** (range −5 to −20) | 1, 3 | Makes the sustained one-leg-tucked lean cost about 1 per second at stand (6 % of reward) instead of 0.1 | Hind pair sum 0.43 vs 0.04 (flat ctrl); left kicks fall 0.97 vs right 0.02 | Sidestep legitimately holds a pair sum of ~0.09 (cost 0.08/s at −10, small). May reduce leg adaptation on bumps. The term only watches motor 2 |
| 3 | **Shoves in training**: re-enable `push_robot`, linear x/y only, ±0.5 m/s, every 2–4 s, ramp to ±0.8 (**not a reward**) | – | off → on | 3, 1 | Lets falls after a shove cost the policy its remaining episode reward; a one-sided weakness then gets punished directly | Shoves are off (`pupper_env_cfg.py:302`); flat ctrl without shoves already holds 0.08, so the gap is learnable | Course disabled the stock event as too harsh (it also kicks z and roll/pitch/yaw). Falls during training and tracking error may rise early. Needs a config flag |
| 4 | `stand_still_pose` | − | −0.5 → **−2** (range −1 to −3) | 1, 2, 3 | Pulls the standing pose back to default, which is symmetric and 2.5 cm wider | L1 deviation 1.26 vs 0.79 (flat ctrl); stance 12.6/13.7 vs 15.1/16.3 cm | The original run found stand-still terms **worsened** standing push (6.7–10.4 % vs 5.4 %, REPRO_PLAN §2.5): it also penalizes recovery steps. Pair with #3, not alone |
| 5 | `action_rate_l2` | − | −0.02 → **−0.05** (range −0.04 to −0.08) | 5, 4, 2 | Raises the cost of target changes from 1.8 % to ~4.5 % of reward | Walking action rate +18–28 % vs flat ctrl and P; training term −0.31 vs −0.24 per second | Gallop at 1.5 m/s needs fast joints (16.5 % of fast time at the 3 Nm limit); may raise tracking error and slow reactive steps. Do not go 10× |
| 6 | **New term** `stand_still_action_rate`: `action_rate_l2` × [command < 0.05] | − | (none) → **−0.5 to −1** | 2 | Targets standing buzz without taxing walking | Standing action rate 0.65 vs 0.39 (flat ctrl); 93 % of standing action power is above 6 Hz | New code (a 3-line gate on an existing function). Could make the robot slow to react to a shove while standing |
| 7 | `joint_torques_l2` | − | 0 → **−1e-4** (Part 1 `FAST_WEIGHTS` value) | 4 | Discourages riding the 3 Nm limit, where sim and real motors differ most | Torque at limit 16.5 % of fast time (flat ctrl 14.3 %); torque² mean 1.33 Nm² | Can cut top speed; at 1.33 Nm² × 12 joints the cost is small, so expect a small effect |
| 8 | **New term** `action_acc_l2` (second difference of actions; function already in `mjlab/envs/mdp/rewards.py`, not registered) | − | (none) → **−0.005 to −0.01** | 5, 2, 4 | Penalizes buzz (fast sign flips) more than smooth fast motion | Hypothesis. Walking high-frequency share 0.28 vs 0.24 (flat ctrl), a small gap | Registration code needed; weight needs a short calibration run |
| 9 | `foot_slip` | − | 0 → **−0.05** (range up to −0.1, the `FAST_WEIGHTS` value) | 4 | Penalizes foot speed while in contact | Slip 0.40 m/s for all three policies, so it is not bumpy-specific | Gated on moving commands only; does not help standing. Lowest priority |
| 10 | **Mirror-symmetry augmentation** in PPO (`rsl_rl/extensions/symmetry.py`, **not a reward**) | – | off → on | 1 | Trains on mirrored copies of each sample, so a one-sided bias cannot persist | Hypothesis | Gallop C is a rotary gallop (footfall LH → RH → RF → LF), which is not mirror-symmetric, so the gait reference has to be mirrored with a phase shift. Highest engineering cost |

### Considered and not proposed

| Term | Why not |
|---|---|
| `termination` | mjlab scales it by dt, so weight −200 costs 4 per fall. A fall already forfeits ~17 per second of remaining episode. Little extra signal |
| `orientation_l2`, `upright` | Standing body roll is only 1.5°; sin²(1.5°) ≈ 0.0007, so even weight −10 costs 0.007/s |
| `base_height` | Standing height is the same for all three (0.148–0.150 m) |
| `air_time` | Targets swing timing while walking; walking is not where the problems are |
| `joint_acc_l2` | Standing joint speed is not worse than flat ctrl (0.105 vs 0.098); the buzz is in the commanded targets, not the joints |
| `ang_vel_xy_l2` | No measurement shows extra body wobble in the bumpy policy |
| `pose` | Removed by the gait tasks (`pupper_gait_env_cfg.py:320`); `stand_still_pose` and `gait_tracking` replace it |
| `torso_support` | Penalizes torso-on-ground load; the policy does not rest on its torso |
| `abduction_angle` | Pulls each motor 2 to 0 separately; it does not see left-right imbalance as directly as `roll_asymmetry` |

## How to score each problem

Use the probes in `reward_shaping_probe/` plus the standard suite. Thresholds are proposed, to be fixed before
launch.

| Problem | Metric (flat_dr, seed 1234 unless noted) | Now: Bumpy / Flat ctrl | Fixed if |
|---|---|---|---|
| 1. Not symmetric | Standing hind pair sum; left load share; left/right standing push at 0.65 m/s | 0.43 / 0.04; 0.59 / 0.51; 0.97–0.02 / 0.13–0.04 | pair sum ≤ 0.10 rad, load share 0.47–0.53, \|left − right\| push ≤ 0.10 |
| 2. Jitters while standing | Standing action rate (`shape_probe`, stand) and existing `stand_jitter_rad_s` | 0.65 / 0.39; 0.105 / 0.096 | action rate ≤ 0.45 and jitter not worse than 0.11 |
| 3. Knocked over easily | Ranked `R:push_stand@0.65`, two-seed mean (HOLD(c)); v50 from the push sweep | 0.49 / 0.09; 0.65 / 0.74 m/s | ≤ 0.209 (the registered HOLD(c) limit) and v50 ≥ 0.74 m/s |
| 4. Sim-to-real risk | HOLD(a)/(b) (nominal-gain falls vs randomized and vs P); torque at limit at 1.5 m/s; peak foot force; standing push **at nominal gains** (never measured so far) | CLEAR; 0.165 / 0.143; 2.60 / 2.85 BW | still CLEAR, torque at limit ≤ 0.165, foot force ≤ 2.6 BW, nominal-gain push ≤ 0.209 |
| 5. Action rate too fast | Walking action rate at ±1.5 m/s (`shape_probe`) | 13.6, 15.8 / 10.6, 12.4 | ≤ 11 and ≤ 13 (flat ctrl + 5 %) |

**Guardrails (any arm failing these is out, whatever it fixes):** flat_dr falls ≤ Bumpy +2000 + 10
(10 → ≤ 20), bumpy_dr falls ≤ 18 + 10, tracking error at ±1.5 m/s no more than 0.01 m/s worse
(0.094 / 0.112). Differences under 10 falls are noise (REPRO_PLAN §3).

## Proposed sweep

**Budget.** At ~1.07 it/s, 45 minutes is ~2,900 iterations. Bumpy steps run about 1.1–1.2× slower than flat
(`decisions/2026-11-18-w4-finetune-budget.md`), and startup plus checkpoint saves take a few minutes, so
**+2000 iterations per arm (~32–37 min)** fits with margin. It also matches the existing Bumpy +2000 and
flat control +2000, so every comparison is at equal iterations. Runs go one at a time: the trainer is bound by
one CPU thread, and two concurrent runs give 10 % less total throughput (`report/infra/gpu_bottleneck.md`).

**Fixed for every arm:** warm start from P (`rp-p2-s42-r2-cont-1500/model_4496.pt`), reference
`configs/opt-lab-2/p2/s42/r2-lag`, `Mjlab-MixedGaits-Bumpy-Pupper-v3`, WIDEN kp/kd ranges, seed 42,
4096 envs, one 2000-iteration leg.

| Arm | Change vs A0 | Question it answers |
|---|---|---|
| A0 | none (`splay_stand_still.json`) | Control. The existing Bumpy +2000 ran as four 500-iteration legs with learning-rate restarts, so it is not an exact match |
| A1 | #1 standing share 0.2 | Is lack of standing data enough to explain the lean? |
| A2 | #1 + #2 `roll_asymmetry` −10 + #4 `stand_still_pose` −2 | Do the stand rewards fix the lean once they are active? |
| A3 | #3 shoves on | Do shoves alone fix push, and the lean as a side effect? |
| A4 | #5 `action_rate_l2` −0.05 | What does smoother action cost in tracking and falls? |
| A5 | A2 + A3 + A4 | Do the fixes combine? Candidate if it passes |

Six arms × ~37 min ≈ 3.7 GPU-hours, plus ~5 min of evaluation per arm (flat_dr and bumpy_dr suites, flat_nodr,
`shape_probe`, `push_side_probe`, ranked push). If the budget is cut, **cut arms, not iterations**: drop A4,
then A1. Evaluate +1000 checkpoints as diagnostics only. Second seed (4321) evals only for an arm that passes.

**Code needed before launch** (none of it is a reward weight, so the weights JSON alone cannot express it):
a flag for `rel_standing_envs`, a flag to re-add a linear-only `push_robot` event, and a flag for the warm-start
leg length. Changes #6, #8 and #10 need new code and belong in a second round.

## Caveats

- One eval seed, one training seed for each existing policy. The posture and push-direction effects are large
  (0.43 vs 0.04 rad; 0.97 vs 0.02), far beyond the ±0.02 run-to-run push spread.
- "Left" means the robot's own left (heading-perpendicular kick, positive = toward its left side).
- The weight ranges are calibrated from per-second reward shares, not from training runs. Treat them as
  starting points.
- All results are simulation only.
