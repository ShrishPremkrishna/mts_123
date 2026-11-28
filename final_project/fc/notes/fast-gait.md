# Fast gait for the demo (2026-11-23)

**Answer:** no retraining needed. The deployed walker (`default_policy.json`, sha `ecf3a4ab`) was trained for vx up to ±1.5 m/s and has a fast gait that switches on at |vx| ≥ 0.5. Config caps keep the robot below that. **Raise the caps and command 0.75 m/s:** that gives about 0.7 m/s, roughly 2.7× today, with margin in sim and earlier hardware evidence.

## 1. Trained range and controller limits

- **Policy JSON** (`pupper_gait_deploy/policies/`, same file on mts):
  - trained range (`command_clip`): vx ±1.5, vy ±0.5, wz ±2.0;
  - `gallop_speed` 0.5: trot below it, fast gait (2.5× cadence) at or above it;
  - `jump_slot`: walk cap 0.49; run cap 1.5 only while the run button is held, which needs `game_buttons: true`;
  - the training run itself is not documented in the repo.
- **Controller order** (mts `neural_controller.cpp:652–693, 875`; values from `launch/config.yaml:258–326`):
  1. clip to the trained range;
  2. `max_vx_forward` 0.49, `max_vx_backward` 0.3;
  3. jump-slot cap: with `game_buttons: false` this is 0.49 on vx **and** vy, unless `speed_cap` > 0 replaces it;
  4. heading hold (kp 1.5, clip 1.0 rad/s);
  - `max_body_angle` is 1.0 rad.
- **Result:** the deployed range is vx [−0.3, 0.49], vy ±0.49, wz ±2.0. `tej_u6`'s yaml has the same caps. So the robot **never reaches its fast gait**. The FC app cuts further, to 0.3 / 0.15 / 1.0 (`config/params.json`).

## 2. Deployed policy in MuJoCo

**Setup:** production C++ inference and gait-reference headers, MuJoCo at 520 Hz with full-hull contact, 16 ms delay, cap raised to 1.5. Each run is 6 s, scored after 2 s, with 3–8 replicas. "Harsh" means:
- gyro noise σ 0.1, bias 0.05, X-axis spikes of 3 rad/s;
- friction ×0.6, kp ×0.85, +0.3 kg.

| cmd vx | achieved | xy err | yaw err | slip | roll/pitch rms° | max tilt° | falls | harsh (falls) |
|---|---|---|---|---|---|---|---|---|
| 0.30 | 0.26 | 0.07 | 0.08 | 0.04 | 1.4/1.5 | 5 | 0/3 | |
| 0.49 (today's max) | 0.36 | 0.16 | 0.14 | 0.11 | 3.1/3.4 | 10 | 0/3 | |
| 0.60 (fast gait) | 0.62 | 0.12 | 0.55 | 0.22 | 1.3/3.5 | 12 | 0/8 | 0.61 (0/3) |
| **0.75** | **0.73** | 0.13 | 0.49 | 0.22 | 1.7/5.3 | 17 | 0/8 | **0.68 (0/8)** |
| 1.00 | 0.86 | 0.21 | 0.46 | 0.29 | 2.7/7.4 | 23 | 0/8 | 0.72 (0/8) |
| 1.50 | 1.14 | 0.39 | 0.58 | 0.46 | 4.0/8.6 | 37 | 0/3 | |

- **Turning:**
  - wz 1.0 / 1.5 / 2.0 gives 1.02 / 1.55 / 2.04 rad/s; commands of 2.5 and 3 are clipped to 2.0;
  - arc at (1.0 m/s, 2.0 rad/s) gives 0.92 / 1.98, max tilt 12°.
- **Stopping**, from the moment zero reaches the policy: from 0.75, 0.11 m in 0.40 s; from 1.0, 0.12 m.
- **12 s game sequence** (fast, stop, spin, arc, reverse, sidestep): 0/6 falls nominal, 0/6 harsh.
- **IMU faults:**
  - gyro noise, bias and spikes change nothing measurable;
  - **orientation error is the real risk:** a 20° roll error at 0.75 gave 3 falls in 3; 10° is fine;
  - keep ImuHealth's tilt→bad stop.
- **Yaw wobble:** the fast gait wobbles ±0.5 rad/s with heading hold on or off. Steer from the overhead camera.
- **Ball** (indicative only):
  - 0.11 m / 0.43 kg: the robot **stalls and rears** at 0.3; it pushes at 0.49 (ball 2.2 m); at 0.75 the robot slows to 0.25 m/s;
  - falls with the ball: 1/3 at 0.6, 1/4 at 0.49 with heading hold off;
  - a 0.15 kg ball pushes cleanly;
  - test the real ball; the app's `push_vx` 0.3 may stall.
- **Hardware evidence** (`foundation/results/movement-live-20261116.json`): the same policy with cap 1.5 ran **1.0 m/s for 3 s**: "straight travel, clean stop, slight wobble". Sim reads slightly low in the trot.

## 3. Other policies

| policy (vx limit) | 0.6 | 0.75/0.7 | 1.0 | harsh 0.75 | harsh 1.0 | notes |
|---|---|---|---|---|---|---|
| stock (1.5) | 0.62 | 0.73 | 0.86 | 0.68 | 0.72 | tilt 23° at 1.0 |
| **`23pt1499`** (1.5, mts `~/policies`) | 0.56 | 0.72 | **1.00** | 0.71 | **0.96** | tilt ≤ 14°, less yaw wobble |
| `13m4vcia` (1.5) | 0.47 | 0.66 | 1.00 | 0.67 | 0.95 | tilt 32° in trot sequence |
| tej_velocityfs (0.7) | – | 0.73 | – | 0.68 | – | limited to 0.7 |
| markus ×4, ryan, shrish (0.7) | – | 0.57–0.65 | – | 0–3 falls in 3 | – | not faster |

- `legacy_walk` can't run in this harness.
- `23pt1499` ran about 24 min on mts on 11-19 in the student slot. No notes on how it behaved physically were recorded.

## 4. Retraining: prepared, not run

**Package:** `/private/tmp/claude-501/-Users-shrish--claude-spares-spare-KtfIa4/aad0fdc8-5407-448d-92f9-73d18c7992ca/scratchpad/fastgait/train_pkg/` (**copy it somewhere durable**):
- `imu_push_dr.patch`: opt-in robustness training. Gyro bias ±0.1 rad/s, gyro-X spikes ±3 rad/s at 1% per step, tilt bias ±6°, and shoves of ±0.25 m/s.
- `train_seeded.py`: trains on `23pt1499`'s exact reference tables. This checkout's built-in tables differ by 0.09 rad.
- `colab_fast_gait.py` and `fast_gait_colab.ipynb`: the Colab launcher and notebook.
- `rewards_base.json`: reward weights.
- `fg_eval.py` and `host.cpp`: the screening harness. Raw results are in `out*/`.

**Checked:** the patch applies; a 2-iteration CPU run exported a `policy.json` whose tables match `23pt1499` exactly and that passes `download_policy.py --validate-only`; `run_host_test.sh` passes (2724 checks, 0 failures).

**Training needs:** an NVIDIA GPU with CUDA 12.8, torch ≥ 2.7, warp ≥ 1.14 and mujoco-warp 3.10 (via `uv sync --extra cu128`). No JAX. On G4 or A100: about 30 min for a warm-start fine-tune, about 75 min from scratch. Avoid T4.

**What you'd do:**
1. Get `model_<N>.pt` and the reward weights from W&B run `23pt1499`.
2. In Colab: upload the package, add the secret `GITHUB_TOKEN` (and optionally `WANDB_API_KEY`), and run the cells.
3. Validate locally: `--validate-only`, then `fg_eval.py` against stock and `23pt1499`. Pass bar: 0 falls in 8 runs at 0.6, 0.75 and 1.0, nominal and harsh. Then `run_host_test.sh`.
4. Deploy through an overlay and floor-test both robots.

## 5. Recommendation

**Caps.** Raise `speed_cap` and `max_vx_forward` to 0.8 for `neural_controller_default`. Use an overlay, the same way `stock_hh0` builds its kp=0 overlay (also for `tej_u6`); never edit the installed config in place.

**App settings.**
- `max_vx` **0.75** (sim ≈ 0.7 m/s, 0 falls in 8 runs nominal and harsh).
- Command either ≤ 0.45 or ≥ 0.6, so the robot doesn't hover at the 0.5 gait switch.
- `max_wz` 2.0.
- 1.0 only as an optional open-field sprint.
- Crossing the 2.63 m field drops from about 10 s to about 4 s.

**Before Dec 3, floor-test both robots:**
1. 2 s bursts at 0.6, 0.75 and 1.0, plus arcs and stops.
2. The real ball at 0.45 and 0.6. Ball contact is the main fall risk.
3. Keep the tilt guard.

**Retraining isn't worth it in 9 days.**
- Gyro faults don't hurt the current policy in sim, and training on ±6° tilt wouldn't cover the 20° drift that does.
- Any new policy needs its own floor checks on both robots.
- If stock wobbles at 0.75, swap in `23pt1499` before retraining.
- Keep the package as a fallback and launch it only if both fail by about Nov 28.

## 6. Top speeds 1.0–1.5 (2026-11-27, sim)

Same harness (`fastgait_train_pkg_20261124/fg_eval.py` + `host.cpp`, rebuilt), stock `default_policy.json`, cap 1.5. Straight: 4 seeds × 6 replicas = 24 runs per cell; arcs and stops 6 runs. Harsh = gyro σ 0.1, bias 0.05, X spikes 3 rad/s at 1 Hz, friction ×0.6, kp ×0.85, +0.3 kg. Stops are measured from the moment zero reaches the policy ("step"), or with the brain's `decel_vx` 2.0 ramp ("ramp"); add ~0.15 s pipeline lag × speed on the floor.

| cmd | achieved nom / harsh | falls nom / harsh | max tilt° | stop step nom / harsh (m) | stop ramp nom / harsh (m) | arc wz 1.0: wz, radius nom (harsh) |
|---|---|---|---|---|---|---|
| 1.0 | 0.86 / 0.74 | 0/24 / 0/24 | 23 (33 one seed) | 0.13 / 0.19 | 0.35 / 0.31 | 0.99, 0.96 m (0.90) |
| 1.2 | 0.99 / 0.83 | 0/24 / 2/24 | 25 | 0.18 / 0.18 | 0.49 / 0.46 | 0.93, 1.16 m (1.14) |
| 1.4 | 1.11 / 0.88 | 0/24 / 0/24 | 25 | 0.24 / 0.19 | 0.68 / 0.64 | 0.78, 1.54 m (1.33) |
| 1.5 | 1.14 / 0.88 | 0/24 / 3/24 | 37 | 0.22 / 0.25 | 0.70 / 0.54 | 0.71, 1.71 m (1.25) |

- Harsh falls need friction loss **and** gyro faults together (friction-only and gyro-only: 0/6 at every speed); 1.2 vs 1.4 differences are noise.
- Above 1.4 the gait saturates (+0.03 m/s) while the pitch spike grows to 37° (fallen cutoff 45° for 0.5 s).
- wz tracking falls with speed: at 1.5 a wz 1.0 command gives 0.71 rad/s.
- **Fence guard** (`brain._fence_guard`, wall_brake 0.8, act_lag_s 0.15) budgets 0.15·v + v²/1.6 = 0.78 / 1.08 / 1.44 / 1.63 m at 1.0 / 1.2 / 1.4 / 1.5; sim ramp + lag gives 0.48 / 0.64 / 0.85 / 0.87 m. The guard is conservative; it also means speeds above ~1.2 toward a fence are only reachable >1.1 m from it.
