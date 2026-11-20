# Part 3 bumpy policy jerked on the test stand: deployment-side causes

**Run:** `photon_mz-stanford-university/mjlab/23pt1499` (rp-p3-final-bumpyft-kpwide-leg4, it 6492). The `policy.json` sha256 is `42d4863b…`.
**Symptom (2026-11-20, our Pupper, on the stand with feet in the air):** "extremely jerky, shaking the table".
**Scope:** deployment, controller and test-stand causes. Training causes are covered in a separate analysis. Nothing here touched the robot. The simulation and offline evidence is listed under Evidence.
**No hardware log of the jerky run is available.** No bag, launch log or stick input was recorded, so the ranking below is inferred. The last section lists what to record next time.

## Ranked causes

| # | Cause | Confidence | Fix or check |
|---|---|---|---|
| 1 | **Thin stability margin with unloaded legs.** The policy has a limit cycle through its own `last_action` and `joint_pos` feedback. It appears when the joint inertia is small, which is the case with the feet in the air. | medium-high (sim/offline) | Test on the ground, or slow and stiff. Retrain with a stronger action-rate penalty (see the training analysis). |
| 2 | **The fast (gallop) branch can be reached on Square, and its trot↔gallop seam is a 0.8 rad reference jump.** In replay, stick tremor at about 1/3 stick gives 3.7× the jerk of trot and 13.6% of samples at the torque limit. | medium (depends on the unrecorded stick input) | Cap Square at `max_vx_forward: 0.49` for stand tests, as the X slot already is. |
| 3 | **Control-loop latency and rate are unmeasured on the Pi.** There is no RT scheduling, and the camera and foxglove share the CPU. The margin from #1 shrinks as latency grows. | low-medium (unmeasured) | Record `policy_inference_latency_seconds`, the `/joint_states` rate and the `policy_output` rate. |
| 4 | IMU dropout (as on the loaner) or vibration of the stand into the gyro | low (no log) | Check the launch log for `BNO055`. Use a rigid stand. |
| 5 | `/cmd_vel` expiry: Bluetooth gaps over 0.25 s step the command to 0 | low | Record `/joy` and `/cmd_vel` |

**Ruled out.** The observation and action contract matches training (item-by-item audit below). bd2e88c changes no runtime behaviour on the `legacy_twist` default. kp and kd reach the motors as 5.5 and 0.25.

---

## 1. Limit cycle with unloaded legs (rank 1)

**Mechanism.** The policy feeds back on its previous action (`last_action`, the raw pre-clamp output) and on the joint positions, at 50 Hz with a 16 ms bus delay. On the ground, load and friction damp that loop. With the feet in the air, the knee and hip carry only the rotor (armature 0.0016 kg·m²) and a light link. That raises the PD bandwidth and leaves less phase margin.

**Evidence A: the exported artifact in the controller's semantics.** Script: `diag/hardware_jerk_rca/loop_json.py`.
- The policy is run through `json_forward`.
- The 20-frame, 48-dim history is built newest-first, as the controller builds it.
- `last_action` is the raw output. The target is `clamp(0.75·a + default)`. The command is zero and the base is pinned.
- Each joint is a PD-driven inertia I: kp 5.5, kd 0.25, ±3 Nm, 0.125 Nm friction, a pure 16 ms target delay and 1 kHz integration.
- Back-left gets a 0.15 rad kick at t = 1 s. Sensor noise is zero.

| policy | I = 0.002 | I = 0.003 | I = 0.004 |
|---|---|---|---|
| **23pt1499** mean \|Δa\|/step, 4–8 s after the kick | **0.071 (sustained)** | **0.054 (sustained)** | 0.000 (decays) |
| **fasxqy4z** (old good run) | 0.000 (decays) | 0.000 | 0.000 |

The same harness at 36 ms delay with hardware-like noise (joint 0.002 rad, gyro 0.01 rad/s):
- 23pt1499 chatters at 0.16 (I = 0.002) and 0.075 (I = 0.006).
- fasxqy4z stays at 0.002 until 56 ms, where it goes unstable as well.

The plant is crude: independent joints, no leg gravity. The absolute threshold is therefore uncertain. The relative result is robust: in identical conditions 23pt1499 self-oscillates where fasxqy4z settles.

**Evidence B: the training plant (mjlab), stand vs ground.** Script: `stand_replay.py`.
- The base is pinned upright at 0.40 m every control step, using the mechanism of the training airborne warm-up (`tasks/pupper_gait/mdp/warmup.py`).
- DR is on. There are 4 replicas per cell over 10 s. The actuator lag is fixed per env.
- Zero command, mean \|Δa\| (with training obs noise / with noise off):

| | ground lag 16 ms | stand lag 16 ms | stand lag 56 ms |
|---|---|---|---|
| 23pt1499, training noise | 0.154 (87% of power above 8 Hz, peak about 20 Hz) | 0.145; joint velocity RMS 0.41 vs 0.15 rad/s on the ground | 0.154; 0.49 rad/s |
| 23pt1499, noise off | 0.001 | 0.001 | 0.001; 0.05 rad/s |
| fasxqy4z, training noise | 0.057 | 0.056; 0.20 rad/s | 0.058 |

Reading:
- The full multibody training plant does **not** show a noise-free limit cycle on the stand. The real unloaded leg is lighter and has backlash, so it sits between plants A and B.
- With training-level noise, 23pt1499 turns observation noise into **2.7× more output chatter than fasxqy4z**. That chatter is a 20 Hz buzz in the targets, about 0.08 rad std.
- With the feet in the air, the same chatter becomes **2.7× more joint motion** than on the ground (0.41 vs 0.15 rad/s).
- Anything noisy on the real robot excites it: a shaking stand feeding the gyro, encoder quantization, or backlash.

**Training-side corroboration (W&B summaries; pointer for the training analysis).**
- 23pt1499 penalizes action rate at `action_rate_l2 = -0.02`; fasxqy4z uses -0.06. So the action-rate penalty is not missing, but it is 3× weaker.
- 23pt1499 also has `joint_torques_l2 0` (fasxqy4z: -0.0005) and `foot_slip 0` (-0.05).
- `Episode_Metrics/mean_action_acc` is **1.38 vs 0.63**.
- Command ranges differ (±1.5 m/s with a fast gait vs ±0.7 trot only), so this is not a like-for-like comparison.

## 2. The gallop branch is reachable on Square (rank 2)

- **Slot asymmetry.** The X slot carries `game_buttons: false` and `max_vx_forward: 0.49` (`pupper_gait_deploy/config.yaml:321-322`), so X never enters the fast branch. Square (`neural_controller_student`, `config.yaml:189-256`) has no cap.
- **Stick mapping.** Teleop scales the stick to 1.5 m/s (`config.yaml:431`). So a third of stick deflection reaches `gallop_speed 0.5`. Above it the reference plays the captured fast table at 2.25 × 1.333 = 3.0 Hz.
- **Seam jump.** It is computed with `refjump.py`, a port of `gait_reference.hpp:94-131`, on the shipped tables.

| transition between two ticks | max-joint reference jump (mean over phase) |
|---|---|
| normal per-tick change, trot at 0.45 | 0.18 rad |
| normal per-tick change, fast gait at 0.55 | 0.30 rad |
| **trot↔gallop flip at vx 0.49/0.50** | **0.82 rad** (the phase clock and the table both switch) |
| fwd/back flip at vx ±0.06 | 0.48 rad |
| trot↔lift flip at vx 0.049/0.050, or lift direction flip at yaw ±0.02 | 0.15–0.16 rad |

- **Hover risk.** A thumb holding about a third of the stick, with ±1–2 stick counts of tremor (about ±0.012–0.024 m/s), can flip the branch on many ticks. Training holds each command for 3–8 s, so it never sees this.
- **Old run.** fasxqy4z has `gallop_speed 1000` and command clip ±0.7, so it has no such seam.
- **Closed-loop replay.** This is the training plant on the pinned stand, noise off, 16 ms lag, with per-tick command noise drawn uniformly (`stand_replay.py --cmdset seam`):

| command on the stand | 23pt1499 \|Δa\| | \|Δ²a\| | share of samples at 3 Nm | share of power above 8 Hz | fasxqy4z \|Δa\| |
|---|---|---|---|---|---|
| exact zero | 0.001 | 0.001 | 0 | — | 0.000 |
| **zero ± 0.03 (vx, yaw), a resting thumb** | **0.30** | 0.52 | 1.5% | 61% | 0.21 |
| trot 0.45 steady | 0.36 | 0.37 | 1.1% | 4% | 0.31 |
| gallop 0.6 steady | 0.64 | 0.84 | 7.4% | 27% | 0.35 (no fast branch) |
| **vx 0.5 ± 0.02, the seam flipping most ticks** | **0.83** | **1.35** | **13.6%** | **44%** | 0.32 |

- **Seam effect.** At the seam, jerk (\|Δ²a\|) is **3.7× steady trot**, with 12× the torque-saturation time. The 44% high-frequency share is the signature of "jerky".
- **Near-zero stick.** A stick resting slightly off centre (beyond the joy deadzone, default 0.05) already makes 23pt1499 step and shuffle at 0.30. With blend_speed 0.1, a 0.03 command is a 30% gait. joy_node normally zeroes a centred stick, so this needs drift or a light thumb.
- **Fast gait in the air.** In the training plant the 1.0 m/s fast gait already has an action rate of 0.73 on the stand. That is 2.2× trot (0.35), with **10% of samples at the 3 Nm torque limit** and 27% of action power above 8 Hz. Run by itself in the air, it is violent.

## 3. Control-loop timing and latency (rank 3)

- **Policy rate.** It is set by tick count: `update_rate: 520` with `repeat_action: 10` (`config.yaml:5,191`). Inference runs on every 10th `update()` (`neural_controller.cpp:834-845`). A slower or jittery loop therefore lowers the policy rate and adds latency. The gait phase stays on wall-clock time (`:1198-1201`), so it drifts relative to the policy step.
- **No RT scheduling.** The Pi run gets none (`docs/07`: `ulimit -r` = 0, FIFO refused).
- **CPU load.** The launch also starts `camera_node` at 1400×1050 RGB888 and `foxglove_bridge` (`launch.py:258-309`).
- **Training assumption.** Training uses a fixed 16 ms command lag (`env.yaml` actuator `delay_min/max_lag 4` at 4 ms).
- **Sensitivity.** In the replay, 16→36→56 ms raises the trot action rate 0.35→0.41→0.49 on the stand. In plant A, 36 ms is enough to make 23pt1499 chatter at I = 0.006 as well.
- **Logging fix.** 5d4e906, the version we validated, still had a synchronous `RCLCPP_INFO` on every inference. main (2977ef3, in bd2e88c and 454d3ef) removed it. The Pi build is 454d3ef or later, so the deployed controller is better in this respect.

## 4–5. Lower-ranked

- **IMU.** On the loaner, the BNO055 died the moment Square was activated (`docs/07`). According to `docs/06`, the stock driver then keeps serving stale data. On a stand, stale orientation is nearly right, but a stuck nonzero gyro value would feed the policy a constant ang_vel. A stand that shakes also puts its vibration straight into `base_ang_vel`, which closes the loop of rank 1 through the IMU. Check the launch log for `BNO055` lines and `/imu_sensor_broadcaster/imu`.
- **`/cmd_vel` expiry.** It applies on the gait velocity path (`neural_controller.cpp:293, 826-859`; 0.25 s). joy_node autorepeats at 50 Hz (`config.yaml:447`) and teleop republishes on every message, so expiry only fires on a Bluetooth gap. The effect is one step to zero (stand), not a continuous jerk. Heading hold is harmless on a stand: yaw is constant, so the error is about 0.
- **kp trims.** `robot_trims.yaml`, which is gitignored, can scale kp per joint at install (`install.py:239-275`). The startup log prints `kp_scale for … is …` if any trim is applied. kp_max is 10 (`components.xacro`), so 5.5 is not clamped.

## Controller vs training audit

| item | training (`params/env.yaml`, `pupper/mdp/observations.py`) | controller (`neural_controller.cpp`, main bd2e88c) | match |
|---|---|---|---|
| policy rate | decimation 5 × 4 ms = 50 Hz | 520 Hz / 10 = 52 Hz; gait phase on wall clock | yes (4%; see rank 3 for jitter) |
| action latency | fixed 4 physics steps = 16 ms | bus about 15.4 ms (per the cfg comment); unmeasured on this Pi | assumed |
| action processing | target = 0.75·a + default; driver-limit clip in sim | `clamp(0.75·a + default, json limits)` then driver clamp (`:1339-1388`); no filter; slew off (no `position_target_slew` in the JSON) | yes |
| kp / kd | 5.5 / 0.25 (DR kp ×0.6–1.3, kd ×0.7–1.5) | JSON 5.5 / 0.25 × gain_multiplier 1.0 × kp_scale (`:133-134, 1380-1387`) | yes (unless trims) |
| last_action | `env.action_manager.action` = raw output (`observations.py:139`) | raw output after fade-in, pre-clamp (`:1348-1354`) | yes |
| joint_pos | true q − default (`legacy_true`) | encoder q − default (`:1132-1143`) | yes up to encoder bias (static) |
| projected gravity / ang_vel | body frame, IMU-lagged 0–1 step, noise | from the IMU quaternion and gyro, body frame (`:1083-1118`) | yes |
| history | 20 frames, newest-first in export | `std::rotate` right; frame 0 newest (`:1321-1324`) | yes (export audit 7e-6) |
| gait reference | mixed reference from the emitted command | `compute_gait_reference_offset` on the filtered command; golden test 48/48 | yes |
| command clip / heading hold | ±1.5/±0.5/±2.0; hold kp 1, clip 0.3 | from the JSON (`:313-350`) | yes |

## Does bd2e88c or later change runtime behaviour? (`git diff 5d4e906 origin/main -- pupper_gait_deploy`)

- **`cmd_vel_contract` (new, read-only, default `legacy_twist`).** `config.yaml` doesn't set it. On `legacy_twist`, `cmd_vel_live()` and the subscriptions are unchanged. All `fc_deadline_v1` code is gated by `command_deadline_mode_`.
- **Removed per-inference `RCLCPP_INFO` (2977ef3).** Timing gets better. The control law is unchanged.
- **`install.py`.** It copies `command_deadline.hpp`.
- **Everything else.** Tests and headers only.

So the deployed control law is identical to the one we validated. Since the fork base d96d287, the only gait-relevant addition is the 0.25 s `/cmd_vel` expiry. The fasxqy4z era (2026-10-20, the upstream cs123 controller before the fork) ran the same obs/gait-reference/heading-hold path; `faded_position_action` is bit-identical for gait policies. **No deployment record of fasxqy4z exists in this repo**, so whether it ran with the same camera, foxglove and stand setup is unknown.

## Test stand vs sim

- **Training.** 23pt1499's own run had no suspended episodes: its events have no `hold_base_airborne`. Only the from-scratch ancestor had an airborne warm-up, which is tracking-only and comes early.
- **This replay.** The training plant with feet in the air at zero command is still calm without noise. With noise, it moves the joints 2.7× more than on the ground.
- **The stand itself.** The stand turns policy chatter into leg motion that nothing damps. It also feeds the reaction forces back into the IMU if the stand is not rigid.
- **CS123 guidance.** Stand first, then floor (`docs/04`). A smooth policy on the stand is therefore the expected baseline, and the X default passing first is the control.

## Record on the next hardware test

Run `scripts/record.sh` plus the additions below. MCAP, everything at native rate.

| topic or data | why |
|---|---|
| `/neural_controller_student/observation` (960 floats, every inference) | Exact replay offline through `json_forward`. Separates obs → action gain from plant effects. |
| `/neural_controller_student/policy_output` and `/position_command` | Action rate and spectrum at the control rate. Shows clamp hits. |
| `/joint_states` (position, velocity, effort) | Tracking lag. Measures leg inertia and backlash on the stand. Tells a limit cycle apart from noise. |
| `/imu_sensor_broadcaster/imu` and `/neural_controller_student/imu_latency_seconds` | Gyro vibration from the stand. Stale IMU. |
| `/neural_controller_student/policy_inference_latency_seconds`; rates of `policy_output` and `joint_states` (`ros2 topic hz`) | Real policy rate and jitter (rank 3) |
| `/joy` and `/cmd_vel` | Stick input. Whether the gallop seam (vx about 0.5) or expiry was hit. |
| `/rosout` plus the launch log (`BNO055`, `kp_scale`, `Command clip`, `/cmd_vel expires`, `Could not enable FIFO`) | Config as actually loaded |
| Video at ≥60 fps, side view | Ground truth for "jerky" |

**Protocol (the cheapest decisive A/B):**
1. Run X and Square on the stand, each with the stick at **zero** for 10 s.
2. Then trot at 0.3 m/s (about 20% stick) for 10 s.
3. Then turn.
4. **Do not exceed 1/3 stick** on the stand, or set Square `max_vx_forward: 0.49` first.

Reading the result:
- Square chatters at zero stick while X is calm → rank 1. Confirm offline by replaying the recorded `observation` through `json_forward`.
- Square jerks only above about 1/3 stick → rank 2.
- If both chatter, suspect timing or the IMU (ranks 3–4).
- Optional: repeat Square at zero stick with the camera and foxglove off, to isolate CPU load.

## Evidence

All results are simulation and offline only.

- Scripts and summaries: `report/part3/diag/hardware_jerk_rca/`.
  - `stand_replay.py`: the mjlab closed loop, ground vs pinned stand, lag 4/9/14, commands.
  - `loop_json.py`: the deploy-artifact loop.
  - `refjump.py`: seam discontinuities.
  - `stand_*.json` and `seam_*.json`: per-condition metrics.
- GPU jobs ran through `scripts/opt_lab_2/gpu_slot_run.py`. Each took under 5 min with 96–80 envs.
- fasxqy4z was fetched from W&B (`model_2999.pt`, `policy.json`). `loop_json.py` expects its `policy.json` at `/tmp/fasx/policy.json`. The env reference tables match its `policy.json` exactly; the same check passed for 23pt1499.
