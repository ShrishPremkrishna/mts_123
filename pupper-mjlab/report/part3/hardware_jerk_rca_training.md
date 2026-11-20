# Why the Part 3 policy shook on the test stand: training-side root-cause analysis

2026-11-20. Simulation and W&B only; no training, no hardware. Scripts and raw output: [`hardware_jerk_rca_training/`](hardware_jerk_rca_training/).

**Question.** On the first hardware test (Pupper on a stand, feet in the air) the Part 3 Bumpy policy (`23pt1499`, `model_6492`) was "extremely jerky, shaking the table". The older policy `fasxqy4z` transferred well. Which training differences explain it? Is the user's hypothesis, "no L2 action-rate penalty", the cause?

## Answer in brief

| Rank | Cause | Key number (sim, nominal motor gains) | Verdict |
|---|---|---|---|
| 1 | **Smoothing penalties too weak, plus abduction joints chattering against the action clip.** `action_rate_l2` is −0.02 (fasxqy4z: −0.06). There is no torque penalty (fasxqy4z: −0.0005). The front roll joints sit on the clip, and their exploration noise never shrank. | Standing on the stand: raw action rate **0.65 vs 0.064 (10×)**. The front roll joints move **0.39–0.41 vs 0.04–0.05** per step and sit on the clip **65–85 % vs 0 %** of the time. Their learned std is **0.96–0.98 vs 0.16**. | **Main cause.** The user's hypothesis is partly right: the penalty is there, but it is 3× weaker. The defect already exists in the Part 1 parent (0.50), and Bumpy training made it 50 % worse than the flat control (0.65 vs 0.43). |
| 2 | **Gallop reference at ≥ 0.5 m/s.** At 0.5 m/s and above, our policy switches to the captured gallop, clocked at 2.25× = 3.0 strides/s. fasxqy4z never gallops: it trots at 1.5 Hz with commands capped at 0.7 m/s. | At 0.5 m/s on the stand: **77 % vs 25 %** of action power is above 5 Hz. Action rate is **7.6 vs 2.2**, and samples at the 3 Nm torque limit are **10 % vs 3 %**. | **Big contributor if any command reached 0.5 m/s.** Full stick is 1.5 m/s for our policy. |
| 3 | **High position-feedback gain, which makes the policy fragile to real latency and stiffness.** Our policy reacts 2.5–4× more strongly to joint-angle readings. | ∂action/∂joint-angle (2-norm, standing): **7.5 vs 1.9**. With 20 ms of extra latency, standing target jitter is **0.26 vs 0.15 rad per step**. At kp ×1.6 the action rate rises 31 % vs 9 %. | **Amplifier.** It explains why hardware looks worse than sim's 1.5× gap in motor targets. |
| – | WIDEN kp/kd | The Part 1 parent (Lab 4 ranges, no WIDEN) is as jerky (0.50). The WIDEN flat control is the smoothest of ours (0.43). | Not a cause |
| – | Test stand is out of distribution | The same policy jitters the same in the air and on the ground: 0.650 vs 0.649. The stand raises joint speed about 2.6× for *every* policy. | Not a differentiator. It makes jitter more visible. |
| – | `last_action` feedback loop | Loop gain (spectral radius) is 0.82 vs 0.72. Both are below 1, and the diagonal is about −0.5 for both. | Not a cause |
| – | Action std never annealing | Mean 0.44 vs 0.18. The deployed policy is deterministic, so std adds no noise on the robot. | A symptom of #1, not a separate cause |

**Bottom line.** Raising `action_rate_l2` alone will help #1 but not #2 or #3. The cheapest hardware test of the ranking:
1. Cap the forward command below 0.5 m/s (the controller has a speed-cap parameter).
2. Record `policy_output` on the stand at zero command.
3. Compare it with the sim numbers below (same metric definitions, see Methods).

## Terms

| Term | Meaning |
|---|---|
| **fasxqy4z** | The good-transfer reference: W&B `photon_mz-stanford-university/mjlab/fasxqy4z`, `model_2999`. It is a Colab "Step 8" StableGait-style run on course commit `9cfcb14`, 2026-10-20: 3000 iterations from scratch, bumpy terrain. It is **not** the same run as the team's Lab 4 handoff `4hkyvuxh` (`lab4_runs/shrish_stablegait`), which used `action_rate_l2` −0.02. |
| **bumpy** | Part 3 Bumpy +2000, `23pt1499` / `model_6492`: the policy that shook on hardware. |
| **flatctrl** | Part 3 flat control +2000, `13m4vcia` / `model_6492`. Same everything as bumpy, but flat ground. |
| **p1** | Part 1 parent `1c5r0n8c` / `model_2999`: gallop_c from scratch, `splay.json`. Every Part 2/3 run descends from it. |
| Action *a* | Raw 12-dim network output. Joint target = default pose + 0.75 · *a*, then clamped. |
| **Action clip** | Our env clamps each joint target to the class-driver limits (`foundation/configs/class-driver-joint-limits.json`; roll joints ±0.32 rad). The real driver does the same, unverified on the installed robot. fasxqy4z trained with **no clip**. |
| Motor 2 / roll joint | `leg_*_2`, the abduction joint that swings the foot sideways. FR2 = front-right roll, and so on. |
| **Action rate** | Σ over the 12 joints of (*a*ₜ − *a*ₜ₋₁)² per 0.02 s step, the quantity `action_rate_l2` penalizes. "Mean \|Δa\|" is the per-joint mean absolute change per step. |
| **Target step** | Mean \|Δ joint target\| per step in rad, after the clamp: what the motor actually receives. |
| **HF share** | Share of action power (mean removed, 0.2–25 Hz) above 5 Hz. At zero command every policy is >90 % HF, because only jitter remains, so compare *amplitudes* there. |
| **Test stand (sim)** | The base is pinned upright and motionless 0.30 m above the floor every control step: the training `hold_base_airborne` event, never released. Foot contact is 0 % in every run. |
| nodr / dr | Nominal mechanics (kp 5.5, kd 0.25, no mechanical DR) / Lab 4 domain randomization. |
| **Jq, Ja** | Finite-difference Jacobians of the action with respect to the newest observed joint angles (Jq) and the newest observed last action (Ja), each 12×12. Measured at t = 8 s. |
| std | The learned exploration-noise standard deviation of the policy's Gaussian, per joint, in raw-action units (`distribution.std_param`). |
| WIDEN | Part 3 PD-gain DR, kp ×[0.6, 1.3] and kd ×[0.7, 1.5]. Lab 4's is kp ×[0.6, 1.1] and kd ×[0.8, 1.5]. |

## 1. Full configuration diff

Sources: the W&B `config.yaml` of each run (diffed key by key over about 1,300 flattened keys; re-pull with `wandb.Api().run(...).config`), the W&B history, and `policy.json`.

**Ranked by plausible effect on high-frequency action oscillation** (most likely first):

| # | Setting | fasxqy4z (good) | p1 `1c5r0n8c` | bumpy `23pt1499` | flatctrl `13m4vcia` | Why it matters for jitter |
|---|---|---|---|---|---|---|
| 1 | `action_rate_l2` weight | **−0.06** | −0.02 | −0.02 | −0.02 | The only term that directly charges step-to-step change. It is 3× weaker in our runs. |
| 2 | `joint_torques_l2` weight | **−0.0005** | 0 | 0 | 0 | It penalizes bang-bang targets that drive the motor to its limit. |
| 3 | Action clip on targets | **none** | driver limits (roll ±0.32) | same | same | With a clip, noise beyond it does not reach physics. The policy parks the roll joints on the clip, and their std grows unchecked. |
| 4 | Final learned action std (mean / max joint) | **0.18 / 0.25** | 0.42 / 0.80 | 0.44 / **0.98** | 0.33 / 0.73 | It never annealed in our line. Max on the front roll joints. Table 2.4. |
| 5 | Command range vx | **±0.7 m/s** | ±1.5 | ±1.5 | ±1.5 | Ours must cover galloping. Full stick = 1.5 m/s. |
| 6 | Gait reference, clock | trot (course table) + lift-in-place, **1.5 Hz**. `gallop_speed` 1000 = never gallops. | gallop_c: 1.33 Hz below 0.5 m/s, **3.0 Hz captured gallop at ≥ 0.5 m/s** (2.25×) | p2/s42/r2-lag captured refs, same clocks | same as bumpy | The 3 Hz gallop puts its 2nd harmonic at 6 Hz. Abrupt switch at 0.5 m/s. |
| 7 | Fall termination | **30°, `termination` −10** | 45°, 0 | 45°, 0 | 45°, 0 | fasxqy4z is pushed toward conservative motion. |
| 8 | Other regularizers | `foot_slip` −0.05, `self_collision_{l,r}` −1, `stand_still_joint_velocity` −0.01 | none of these | `stand_still_joint_velocity` −0.1, `stand_still_pose` −0.5, `roll_asymmetry` −1 | same as bumpy | Ours has stronger stand-only terms, but standing is 2 % of commands (`reward_shaping.md`). |
| 9 | `abduction_angle` weight | −0.05 | −0.1 | −0.1 | −0.1 | Ours pulls the roll joints to 0 harder, yet they sit at the clip. |
| 10 | Training length, chain | 3000 it, from scratch, 1 run | 3000, scratch | 6500 total: 3000 + 1500 + 2000, **7 warm-start restarts** (LR reset each time) | same, flat Part 3 | Std carried and never reset (lineage doc §1.3). |
| 11 | Terrain | Bumpy, Perlin 0–5 cm, grid 0.05 m | Flat | Bumpy, same Perlin, **grid 0.1 m** (checked wrapper) | Flat | Bumpy raised std 0.37 → 0.44 and action rate +50 % at stand. |
| 12 | PD-gain DR | Lab 4 kp ×[0.6, 1.1], kd ×[0.8, 1.5] | Lab 4 | **WIDEN** kp ×[0.6, 1.3], kd ×[0.7, 1.5] | WIDEN | Tested in §2.5: not a cause. |
| 13 | Inertial DR, torso contact | `pseudo_inertia`, stock spec | density (CoM-preserving), torso contact | same | same | Minor. |
| — | **Identical in all four** | action scale 0.75. Nominal kp 5.5 / kd 0.25. Obs noise (ang vel 0.1, gravity 0.05, joint pos 0.05, last action 0.01). IMU latency [0.5, 0.5]. No obs delay. History 20 × 48. Same observation layout. dt 0.02 s (decimation 5 × 4 ms). Episode 10 s. Friction [0.6, 1.4]. Encoder bias ±0.015. PPO: lr 1e-3 adaptive (KL 0.01), 5 epochs × 4 mini-batches, γ 0.97, λ 0.95, clip 0.2, **entropy coef 0.01**, **init std 1.0** (scalar type), net [256, 128, 128, 128] ELU. 4096 envs, seed 42. 500-iteration airborne pretrain in both from-scratch runs (fasxqy4z, p1). | | | | |

The fasxqy4z code diff (`pupper-mjlab.diff` on W&B) only fills in the lift gait: touchdown (0, 0.5, 0.5, 0), stride 0, stance z −0.14, lift 0.09.

**Training-time signals** (W&B, mean of the last 50 iterations):

| | fasxqy4z | p1 | P (Part 2 final) | bumpy +2000 | flatctrl +2000 |
|---|---|---|---|---|---|
| `Policy/mean_std` | **0.18** | 0.42 | 0.37 | 0.44 | 0.33 |
| Logged `action_rate_l2` per s ÷ weight = training action rate | 0.235 / 0.06 = **3.9** | 0.301 / 0.02 = 15 | 13.5 | 0.331 / 0.02 = **16.5** | 11.7 |
| `Episode_Metrics/mean_action_acc` | **0.63** | 1.32 | 1.22 | 1.37 | 1.13 |

How std evolved:
- **fasxqy4z:** 0.19 at the end of pretrain → 0.26 after release → 0.18 at the end. It annealed.
- **p1:** 0.29 at the end of pretrain → 0.48 at iteration 1000 → 0.42. It never came back down.
- **Part 3 Bumpy** then raised it again: 0.37 → 0.44.

## 2. Simulation measurements

**Setup.**
- 64 robots per command and condition. Eval seed 1234.
- 15 s per run, first 3 s dropped. Commands are vx = 0, 0.1, 0.3, 0.5 m/s.
- Every policy runs on **the same plant**: our current deployment-contract physics (driver target clamp, torso contact, composed CoM DR).
- Each policy observes **its own gait reference and clock**. For fasxqy4z, the four tables from its `policy.json` are loaded, at 1.5 Hz with no gallop. For ours, the tables built from the reference dir match their `policy.json` exactly (max difference 0.0).
- **Observation layout.** All four policies use the identical 960-dim contract: 20 frames × 48. The checkpoints load strictly into our runner, so no remapping was needed. In sim the history is oldest-first; the exporter reverses it to newest-first for the robot, the same way for both policies.
- **Run time.** Each sim job took 2–4 min, launched through `gpu_slot_run.py`.

### 2.1 Test stand, nominal gains (`stand_nodr`): the headline table

| Metric | Command | fasxqy4z | bumpy | flatctrl | p1 |
|---|---|---|---|---|---|
| Action rate (Σ Δa²) | 0 | **0.064** | **0.650** | 0.430 | 0.501 |
| | 0.1 | 1.40 | 2.58 | 2.15 | 2.12 |
| | 0.3 | 1.63 | 2.80 | 2.32 | 2.30 |
| | 0.5 | 2.22 | **7.65** | 6.40 | 6.26 |
| Mean \|Δa\| per joint per step | 0 | 0.057 | 0.149 | 0.127 | 0.155 |
| | 0.5 | 0.302 | 0.603 | 0.560 | 0.562 |
| Target step (rad, what the motor gets) | 0 | 0.043 | 0.064 | 0.083 | 0.097 |
| | 0.1 / 0.3 | 0.162 / 0.175 | 0.145 / 0.154 | 0.153 / 0.156 | 0.152 / 0.157 |
| | 0.5 | 0.212 | **0.338** | 0.342 | 0.337 |
| HF share (> 5 Hz) | 0 | 0.92 | 0.95 | 0.97 | 0.96 |
| | 0.1 / 0.3 | 0.15 / 0.18 | 0.17 / 0.18 | 0.19 / 0.19 | 0.18 / 0.18 |
| | 0.5 | **0.25** | **0.77** | 0.76 | 0.67 |
| Action spectrum peak (Hz) | 0 / 0.3 / 0.5 | 14 / 1.5 / 1.5 | 21.5 / 1.3 / **6.0** | 14 / 1.3 / 6.0 | 10 / 1.3 / 6.0 |
| Joint speed, mean / p99 (rad/s) | 0 | 0.19 / 0.98 | 0.37 / 2.06 | 0.50 / 2.77 | 0.60 / 2.39 |
| Samples at 3 Nm torque limit | 0 / 0.3 / 0.5 | 0 / 2.1 % / 3.2 % | 0 / 2.0 % / **10.3 %** | 0 / 1.9 % / 10.7 % | 0 / 1.8 % / 11.0 % |
| Targets at the action clip | 0 | **0 %** | **20 %** | 5.6 % | 10 % |
| | 0.1 / 0.3 / 0.5 | 5.6 / 6.0 / 6.7 % | 31 / 32 / 21 % | 27 / 28 / 16 % | 30 / 30 / 22 % |
| Joint within 0.05 rad of a joint limit | all | 0 | 0 | 0 | 0 |

Reading it:
- **Standing.** Our raw jitter is 10× fasxqy4z's. After the clip absorbs part of it, the motor still sees **1.5×** larger target steps and **2×** the joint speed, at 14–22 Hz.
- **Slow walking (0.1–0.3 m/s).** Raw action rate is 1.7× higher, but motor-target steps are *no larger* than fasxqy4z's. The clip absorbs the excess, and fasxqy4z's lift trot moves more by design.
- **At 0.5 m/s.** The gallop takes over: 6 Hz peak, 3.5× action rate, 1.6× target step, and 3× torque saturation.
- No policy ever reached a joint's mechanical limit.

The same pattern holds with Lab 4 DR on the stand, and on flat and bumpy ground. Standing action rate on the ground (flat_nodr / flat_dr / bumpy_dr):

| Policy | flat_nodr | flat_dr | bumpy_dr |
|---|---|---|---|
| fasxqy4z | 0.065 | 0.066 | 0.066 |
| bumpy | 0.649 | 0.649 | 0.652 |
| flatctrl | — | 0.391 | 0.400 |
| p1 | — | 0.501 | 0.507 |

On the ground at 0.5 m/s, the HF share is 0.73 for bumpy vs 0.26 for fasxqy4z. Full tables: `python summarize.py`.

### 2.2 Where the standing jitter lives: per joint (stand, nodr, command 0)

| Joint | FR1 | **FR2** | FR3 | FL1 | **FL2** | FL3 | BR1 | BR2 | BR3 | BL1 | **BL2** | BL3 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| \|Δa\|, fasxqy4z | 0.07 | 0.04 | 0.07 | 0.07 | 0.05 | 0.06 | 0.06 | 0.03 | 0.06 | 0.07 | 0.03 | 0.06 |
| \|Δa\|, bumpy | 0.09 | **0.39** | 0.09 | 0.08 | **0.41** | 0.10 | 0.09 | 0.11 | 0.07 | 0.08 | 0.15 | 0.13 |
| At clip, bumpy | 0 | **85 %** | 0 | 0 | **65 %** | 0 | 0 | 0 | 0 | 0 | **91 %** | 0 |
| Mean raw *a*, bumpy | 0.02 | **−0.73** | −0.15 | 0.03 | **+0.56** | 0.03 | −0.02 | −0.07 | −0.08 | −0.03 | **+0.58** | 0.13 |
| Learned std, bumpy | 0.23 | **0.98** | 0.33 | 0.23 | **0.96** | 0.34 | 0.24 | 0.48 | 0.33 | 0.24 | **0.52** | 0.36 |
| Learned std, fasxqy4z | 0.15 | 0.17 | 0.24 | 0.15 | 0.16 | 0.24 | 0.16 | 0.14 | 0.25 | 0.16 | 0.14 | 0.25 |

What it shows:
- **The front roll joints carry the jitter**: 0.39–0.41 vs about 0.09 for the other joints.
- Their mean command is **beyond the clip**. A raw action of −0.73 asks for −0.55 rad, and the clip holds the target at −0.32. The output keeps falling back inside the clip and being pushed out again.
- These are exactly the joints with the largest learned std. They also hold the standing lean seen in `reward_shaping.md` (BL2 is tucked).
- flatctrl and p1 show the same pattern with smaller numbers. FR2/FL2 \|Δa\|: 0.30/0.32 (flatctrl), 0.30/0.23 (p1). At clip: 31/35 % and 48/67 %.

**Mechanism (inference, consistent with the numbers).**
1. Past the clip, extra action has no physical effect. Exploration noise there is half-hidden from every task reward.
2. The only term that still charges it is `action_rate_l2`, which acts on raw actions. At −0.02 and std 0.98, that charge is 0.02 × 2 × 0.98² ≈ 0.04 per second per joint: under 0.3 % of the 17 per second of positive reward.
3. The entropy bonus therefore wins: std on those joints rose from 0.72 (P) to 0.98.
4. The deterministic mean learns to sit past the clip and flick back and forth.

fasxqy4z had no clip, 3× the action-rate weight and a torque penalty. Its roll std is 0.16 and its roll joints never touch a limit.

### 2.3 Feedback gains (Jacobians at t = 8 s, stand, nodr)

| Policy | Ja spectral radius (last-action loop) | Ja diagonal mean | Jq diagonal mean | Jq 2-norm |
|---|---|---|---|---|
| fasxqy4z | 0.72–0.76 | −0.43 to −0.50 | **−0.78 to −0.92** | **1.9–2.5** |
| bumpy | 0.82–0.88 | −0.47 to −0.56 | **−1.49 to −1.92** | **5.5–7.5** |
| flatctrl | 0.75–0.87 | −0.50 to −0.55 | −1.38 to −1.55 | 4.4–5.5 |
| p1 | 0.78–0.92 | −0.51 to −0.61 | −1.75 to −2.09 | 4.7–5.6 |

Ranges span the four commands; the largest Jq is at command 0.

- **The last-action loop is similar and stable** in all four: spectral radius < 1, about −0.5 per joint. It is not what separates them.
- **The joint-angle loop is 2–4× stronger in ours.**
  - At command 0, bumpy moves its target 0.75 × 1.92 = **1.44 rad per rad** of measured joint error, on top of the PD loop. fasxqy4z moves it 0.59.
  - If a joint reached its target within one 20 ms step, a per-step loop gain above 1 would grow into a period-2 (25 Hz) oscillation.
  - In sim, PD lag damps this. On hardware, any extra delay or stiffness moves it toward instability.
  - This is a heuristic, not a stability proof.

### 2.4 Learned action std (from the checkpoints, `raw/policy_std.json`)

| Checkpoint | Mean | Min | Max | Front roll FR2 / FL2 |
|---|---|---|---|---|
| fasxqy4z model_2999 | **0.184** | 0.14 | 0.25 | 0.17 / 0.16 |
| p1 model_2999 | 0.417 | 0.24 | 0.80 | 0.80 / 0.78 |
| P model_4496 | 0.372 | 0.21 | 0.75 | 0.72 / 0.75 |
| Bumpy +500 model_4995 | 0.430 | 0.24 | 0.89 | 0.87 / 0.89 |
| **Bumpy +2000 model_6492** | **0.436** | 0.23 | **0.98** | **0.98 / 0.96** |
| Flat ctrl +2000 model_6492 | 0.331 | 0.19 | 0.73 | 0.66 / 0.73 |

### 2.5 Stress tests (stand; all mechanics nominal except the one changed; kd nominal)

Action rate at command 0, with target step (rad) in brackets:

| Condition | fasxqy4z | bumpy | flatctrl | p1 |
|---|---|---|---|---|
| kp ×0.6 | 0.063 [0.042] | 0.614 [0.062] | 0.380 [0.081] | 0.471 [0.096] |
| kp ×1.0 | 0.064 [0.043] | 0.640 [0.064] | 0.424 [0.083] | 0.498 [0.097] |
| kp ×1.3 | 0.066 [0.043] | 0.705 [0.067] | 0.542 [0.088] | 0.604 [0.103] |
| kp ×1.6 | 0.070 [0.044] | **0.840** [0.072] | 0.789 [0.100] | 0.933 [0.123] |
| kp ×1.0 + 20 ms obs latency | 1.56 [**0.152**] | 6.02 [**0.261**] | 1.98 [0.173] | 12.4 [0.503] |

From kp ×1.0 to ×1.6, joint speed at command 0 rises 0.19 → 0.50 rad/s (fasxqy4z) and 0.36 → 0.89 (bumpy).

- **Stiffer motors.** They amplify our jitter (+31 % for bumpy, +87 % for p1 from kp 1.0 to 1.6) and barely touch fasxqy4z's (+9 %). If the real motors are effectively stiffer than sim's average (WIDEN mean kp 0.95), ours gets worse.
- **WIDEN is not the cause.** The p1 parent never saw WIDEN and is at least as jittery.
- **Latency.** One extra 20 ms step of observation latency makes *every* policy shake on the stand. Our bumpy policy reaches a 1.7× larger target step, and p1 3.3×.
- **Latency on the ground** at command 0: fasxqy4z is unaffected, while p1 topples in 41 of 64 runs. Walking with the extra lag, all four fall often (12–59 of 64), so latency robustness is not what made fasxqy4z transfer.

### 2.6 Side check: the robot-side clamp

`pupper_gait_deploy/src/neural_controller.cpp` clamps targets to the `policy.json` joint limits (roll ±0.42). The driver then clamps to ±0.32 (local source; `foundation/results/local-driver-position-limit-audit.json`, installed robot **not verified**).

If the installed driver does *not* clamp:
- The standing target step rises 0.064 → 0.076 rad (+19 %).
- The bumpy policy drives FR2 into the ±0.42 joint limit 72 % of the time.
- fasxqy4z is unaffected (0 %).

This is worth one look on the robot. It is not a training cause.

## 3. Verdict on each candidate

| Candidate | For | Against | Rank |
|---|---|---|---|
| **Weak `action_rate_l2` (user's hypothesis)** | 3× weaker than fasxqy4z (and no torque penalty). Standing action rate 10× higher, training action rate 4× higher. | We do have −0.02, the same as the team's Lab 4 run `4hkyvuxh`. At walking speeds the motor-target steps are not larger. A heavier penalty alone does not fix the gallop or the clip-parking. | **#1, jointly with the clip + std mechanism (§2.2)** |
| Action std never annealing | 0.44 vs 0.18; 0.98 on exactly the jittering joints. | Deployment uses the mean, so std itself adds no hardware noise. | Part of #1 (symptom) |
| **High-frequency gallop reference** | At ≥ 0.5 m/s: HF 0.77 vs 0.25, 6 Hz peak, 10 % torque saturation, 1.6× target step. | Below 0.5 m/s there is no gallop effect. It depends on what was commanded. | **#2** |
| **Proprioceptive gain / latency sensitivity** | Jq 2–4× larger. +20 ms latency → 0.26 vs 0.15 rad target step. kp ×1.6 → +31 % vs +9 %. | Latency hurts fasxqy4z too, and while walking it falls as often. | **#3 (sim-to-real amplifier)** |
| kp/kd WIDEN | — | p1 (Lab 4 DR) is as bad. flatctrl (WIDEN) is our smoothest. | Not a cause |
| Test stand out of distribution | The stand roughly doubles joint speed for the same jitter: bumpy 0.37 vs 0.14 rad/s, fasxqy4z 0.19 vs 0.07. | Action jitter is identical on the stand and the ground (0.650 vs 0.649). Both policies saw 500 airborne pretrain iterations. Affects both equally. | Not a differentiator |
| `last_action` feedback loop | — | Loop gain 0.82 vs 0.72, both < 1; same per-joint sign and size. | Not a cause |

## 4. What this suggests (not done here)

Training side, in order:
1. `action_rate_l2` −0.02 → −0.05 to −0.06, plus `joint_torques_l2` −0.0005: the fasxqy4z recipe. This overlaps candidate 5 in `reward_shaping.md`.
2. Stop the roll joints parking on the clip. Options:
   - A small penalty on the raw action beyond the clip, \|a − clip(a)\|².
   - A per-joint cap on std.
   - Training without the clip, as fasxqy4z did. That is a deployment-contract change and needs a decision.
3. For a first hardware test, keep commands below 0.5 m/s (the controller's forward-speed cap) or test the gallop separately.
4. Consider one step of observation-latency randomization (`delay_max_lag` = 1) in training.

Hardware side, cheap:
- Record `policy_output` on the stand at zero command. Compute mean \|Δa\| and the FR2/FL2 values.
- Sim predicts **0.15** for bumpy (0.39/0.41 on FR2/FL2) vs **0.06** for fasxqy4z.
- A much larger hardware number points to rank #3 (latency or stiffness).

## 5. Limitations

- **Simulation only.** The 20 ms latency and kp sweeps are crude stand-ins for the real actuator. The Jacobian loop-gain argument is a heuristic.
- **The hardware command sequence is unknown.** Rank #2 matters only if 0.5 m/s or more was commanded.
- **Training settings are confounded.** fasxqy4z differs in many settings at once: penalties, clip, commands, gait, termination. Separating #1 from the clip would take training arms, which this task excludes.
- **fasxqy4z ran on our plant, not its own.** Our clip and DR are applied to it. It never touches the clip (0 % at command 0, about 6 % while walking), so the effect is small.
- One eval seed (1234). Differences of 2× or more are far above seed noise; the ±10 % ones are not.

## Files

- `hardware_jerk_rca_training/jerk_probe.py`: the sim probe, one policy per process. Conditions: stand, ground, kp sweep, latency.
- `hardware_jerk_rca_training/policy_std.py`: per-joint std from the checkpoints.
- `hardware_jerk_rca_training/summarize.py`: tables from `raw/`.
- `hardware_jerk_rca_training/raw/`:
  - `jerk_<policy>.json`: 5 base conditions.
  - `stress_<policy>.json`: kp and latency tests, plus per-joint data.
  - `hwclamp_<policy>.json`: robot-clamp check.
  - `policy_std.json`, plus logs.
- fasxqy4z checkpoint, `policy.json` and W&B config, copied to `~/runs/opt-lab-2/ext-fasxqy4z/` (outside git).
