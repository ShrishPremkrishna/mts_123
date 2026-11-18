# Part 1 lab notebook: authoring the fast gait (reproduction, W1)

Task `Mjlab-MixedGaits-Flat-Pupper-v3`. Spec: [Opt Lab 2, Part 1](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-2.html).
Started 2026-11-18. This redoes Part 1 after the first machine's gait parameters and reference files were lost
(see [REPRO_PLAN.md](REPRO_PLAN.md) §1–2). Everything below is **simulation evidence only**.

Tags: **[R]** recovered from the old run, **[N]** new in this reproduction, **[M]** measured here.

Status: designs, training, evaluation done (sections 3–6). Part 2 parent: see section 7.


## 1. The question and the constraint budget

The fast slot plays when `|vx| >= 0.5` m/s (`TROTGALLOP_ONSET`), for commands up to ±1.5 m/s. Its phase clock runs at
`1.333 Hz × frequency_mult` regardless of the commanded speed (`mixed_gaits.py`: `phase = frac(t · f · mult)`), and
backward commands play the same table time-reversed. Without a reference dir the slot falls back to the stock trot at 1×.

The design rule from the spec is **speed = stride length × cadence**. In stance the foot slides back by the full stride
`2h` in `duty × T` seconds, so the reference's no-slip body speed is

    v_ref = 2h / (duty × T),   T = 0.75 s / frequency_mult,   duty = 4 / (5 + n_swing_keyframes)

What bounds each factor on Pupper [M, from `scripts/opt_lab_2/part1_gait_check.py` FK sweeps over 27 designs]:

| Lever | Limit | Evidence |
|---|---|---|
| Stride `2h` | Leg reach. At stance depth −0.12 m, `h = 0.09` drives the hind knee toward full extension and peak joint speed jumps from 31 to 46 rad/s (54 with 3 swing keyframes). At −0.13 m, `h = 0.09` reaches 77–99 rad/s (an IK near-singularity). A longer *rear* half-stride (0.10) is worse: hind knee 76–128 rad/s. | sweep `z ∈ {−.11,−.12,−.13} × h ∈ {.07,.08,.09} × kf ∈ {3,5,7}` |
| Stance depth (crouch) | The **joint-limit clamp**: at −0.11 m the front knee needs 0.096 rad beyond the class driver limit `[-0.61, 2.69]`, so `--deployment-target-clamp` would clip the reference. −0.12 m is the deepest crouch that fits (margin 0.03 rad). | `driver_limit_min_margin_rad` |
| Cadence (`frequency_mult`) | **3 Nm actuators.** The course's feasibility proxy is `kp × max |Δq|` per 20 ms control step (kp = 5.5): above 3 Nm the PD cannot follow the reference even from rest. The extended trot at 2× is 2.89 Nm, so 2× is the ceiling for a 0.16 m stride with a 0.4 duty. Damping alone costs `kd × q̇ = 0.25 × 31 = 7.7 Nm` at the reference's peak swing speed, so all fast references are only approximately trackable. | `kp_dq_torque_proxy_nm`, `peak_joint_speed_rad_s` |
| Duty factor | Shorter stance (more swing keyframes) raises `v_ref` at fixed stride and cadence *and* slows the swing (lower joint speed), but removes support: below 0.5 the gait has flight or single-support phases, and stance forces rise as `1/duty`. | footfall diagrams |

**So what limits the trot at speed?** The stock trot (h = 0.05, duty 2/3, 1×) implies **0.20 m/s**, 7.5× short of
1.5 m/s. Its stride is also out of the driver range (front knee 0.10 rad past the limit, so the clamp clips it) [M].
No amount of training fixes a footprint that short: the policy must either slide its feet or abandon the reference.
Inside the reach, clamp and torque limits above, the best feasible designs imply about **1.1–1.3 m/s**. None reaches
1.5 m/s at ≤ 3 Nm, so above that the policy has to lengthen its stride or cadence beyond the reference.

## 2. Designs

All references were generated with `bootstrap.py design`, extended in this workstream with
`--touchdown FR FL BR LB`, `--x-shift`, `--y-shift`, `--half-stride-back` and `--method` (the defaults still reproduce
`configs/optional2/extended_trot` bit-for-bit; checked, max difference 7e-15). The design path produces only the
forward table. `reach_back` is its exact time reversal, as in the old run [R]. Reference dirs (npz + manifest) were committed
and pushed before training at `e01364f` (`configs/opt-lab-2/{fast_trot,gallop_a,gallop_b,gallop_c}`).

Footfall convention: rotary gallop LH → RH → RF → LF (hind pair first, then the fore pair, with the lead on the
side of the second hind foot) [R]. Leg order in the code is FR, FL, BR, LB.

| Arm | Touchdown FR/FL/BR/LB | Half-stride | Stance z | Lift | Swing kf → duty | Mult (cycle) | Lateral | `v_ref` | Peak q̇ | kp·Δq | Min / mean feet down | Flight |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline (stock trot, no ref dir) | 0 / .5 / .5 / 0 | 0.05 | −0.14 | 0.09 | 1 → 0.67 | 1× (0.75 s) | – | 0.20 m/s | 23 rad/s | 2.40 Nm | 2 / 2.67 | 0 % |
| fast_trot | 0 / .5 / .5 / 0 | 0.08 | −0.12 | 0.05 | 5 → 0.40 | 2× (0.375 s) | – | 1.07 | 31 | 2.89 | 0 / 1.62 | 18 % |
| gallop_a (aggressive) | .45 / .53 / .08 / 0 | 0.08 | −0.12 | 0.05 | 7 → 0.33 | 2× (0.375 s) | – | 1.28 | 29 | 2.66 | 0 / 1.34 | 17 % |
| gallop_b (middle) | .50 / .65 / .15 / 0 | 0.08 | −0.12 | 0.05 | 5 → 0.40 | 2× (0.375 s) | – | 1.07 | 31 | 2.89 | 1 / 1.62 | 0 % |
| gallop_c (conservative) | .50 / .65 / .15 / 0 | 0.08 | −0.12 | 0.04 | 2 → 0.57 | 2.25× (0.333 s) | ±0.015 m wider | 0.84 | 45 | 3.82 | **2** / 2.30 | 0 % |

(Duty and feet-down are measured from FK of the stored table with a 1 mm stance tolerance, so duty reads 0.41 for the nominal 0.40.)

Reasoning per arm:

- **baseline**: the stock trot in the fast slot (no `--reference-dir`). It is the comparator the spec asks for.
- **fast_trot**: the extended-reach trot. It uses the parameters of the shipped `configs/optional2/extended_trot` [R template]:
  the stride is lengthened to the reach limit at the deepest clamp-legal crouch, 5 swing keyframes lengthen the swing
  (duty 0.4), and the clock is 2×. That is the fastest diagonal trot inside the 3 Nm proxy. It has an 18 % double-flight
  phase (both diagonals in the air), which a trot at duty < 0.5 cannot avoid.
- **gallop_a (aggressive, long swing / short stance)**: 7 swing keyframes (duty 1/3) raise `v_ref` to 1.28 m/s, the
  highest of any design under 3 Nm, and give the lowest joint speed, since the swing is the longest. The hind pair lands only 0.08
  apart and so does the fore pair, so it is close to a **bound**: a 17 % flight phase and 32 % single support. Expected
  risks: pitching and tipping forward on the fore-pair landing, and dragging feet. These were the old gallop A's failure modes [R].
- **gallop_b (middle)**: the same leg kinematics as fast_trot (same stride, duty and cadence). Only the touchdown phases
  differ: a 4-beat rotary sequence with 0.15 lags inside each pair and the fore pair half a cycle after the hind pair. This
  makes fast_trot vs gallop_b a controlled comparison of the footfall pattern alone. Staggering the touchdowns turns
  the trot's two 18 % flight phases into 38 % single support with no flight at all.
- **gallop_c (conservative, at least 2 feet always down)**: duty 4/7 > 0.5, with the same 4-beat lags as b, so that
  at least two feet are down at every phase (70 % two feet, 30 % three). Every foot keeps the same 0.16 m stride. The stance is
  1.5 cm wider per side for roll margin, and the swing lift is 1 cm lower (it shortens the swing path and adds 0.2 rad of
  knee margin to the driver limit). The cost: a short stance at constant stride caps `v_ref` (0.75 m/s at 2×). I raised the
  clock to 2.25× to recover part of it (0.84 m/s), which makes c the most joint-speed-hungry design (45 rad/s,
  kp·Δq 3.8 Nm, over the 3 Nm proxy). This reproduces the old finding that "C needed the most joint speed" [R].
  It is the arm that relies most on the policy to stretch the reference toward 1.5 m/s.

**What does staggering the touchdowns gain?** At equal stride, duty and cadence (fast_trot vs gallop_b), the
4-beat sequence never has all feet in the air: 0 % flight vs 18 %, with 1–2 feet always down. It spreads the impacts over four
landings per cycle instead of two. Open loop with gravity (below), b pitches 1° vs the trot's −4.7°.
**What does a longer swing gain, and what does it cost?** At a fixed stride and clock, a longer swing raises `v_ref`
(a: 1.28 vs b: 1.07 m/s) and lowers the peak joint speed (29 vs 31 rad/s). The cost is stance time: duty 0.33 means
17 % flight, single support, and per-foot stance forces about 1/duty times body weight.

### Visualizer checks (before training)

Each reference was checked in two ways. (1) **FK plots** of the stored joint table: foot x–z paths and a footfall
diagram, `report/part1/<arm>.png`. (2) **Open-loop playback with gravity on** (the check `visualize_reference`
recommends), `report/part1/<arm>_openloop.gif`. Forward is on the left and backward (time-reversed) on the right. The
gifs show 1.0–3.4 s at half speed. `visualize_reference` itself is interactive (native/viser), so
`part1_gait_check.py render` reproduces its `ReferencePolicy` loop on the same `Mjlab-VelocityFS-Flat-Pupper-v3`
scene offscreen, with the fast-slot cadence from the manifest.

| Arm | FK plot | Open-loop gif | OL fwd vx / pitch / max tilt | OL back vx / pitch / max tilt |
|---|---|---|---|---|
| baseline | [png](part1/baseline.png) | [gif](part1/baseline_openloop.gif) | +0.10 m/s / 7.2° / 21° | +0.13 / 12.0° / 30° |
| fast_trot | [png](part1/fast_trot.png) | [gif](part1/fast_trot_openloop.gif) | +0.36 / −4.7° / 10° | −0.20 / 9.4° / 18° |
| gallop_a | [png](part1/gallop_a.png) | [gif](part1/gallop_a_openloop.gif) | +0.08 / **13.3°** / 29° | −0.06 / 5.7° / 30° |
| gallop_b | [png](part1/gallop_b.png) | [gif](part1/gallop_b_openloop.gif) | +0.17 / 1.0° / 11° | −0.06 / 5.7° / 19° |
| gallop_c | [png](part1/gallop_c.png) | [gif](part1/gallop_c_openloop.gif) | +0.14 / 1.0° / **9°** | −0.14 / 4.0° / **10°** |

(Positive pitch is nose down. Open loop is not a policy: the joints lag the reference badly at these cadences, and
achieved speeds are a small fraction of `v_ref`. The check only flags references that tip the body by themselves.)

Reading: **gallop_a pitches 13° nose-down open loop**, the old A's "tips forward" failure, reproduced before any
training. gallop_c is the only design that stays within 10° of tilt in both directions and moves symmetrically
forward and backward (±0.14 m/s). The stock trot drifts *forward* even when played backward: at 1× its stride is
too short to carry the body, so the robot just shuffles.

## 3. Training runs [M]

Recipe [R]: `pupper_locomotion train --profile baseline --iterations 3000 --num-envs 4096 --seed 42 --logger wandb
--deployment-target-clamp --compose-com-randomization --locomotion-torso-contact --driver-limits
../foundation/configs/class-driver-joint-limits.json --weights configs/opt-lab-2/weights/splay.json [--reference-dir <dir>]`,
launched through `scripts/opt_lab_2/launch.py` (pushed-input check; reference dir and weights logged to WandB as artifacts)
by `scripts/opt_lab_2/part1_queue.py` (at most 3 concurrent processes, no MPS, auto-resume from the last 50-iteration
checkpoint on crash). **No run crashed, so none was resumed.** Checkpoints are in `~/runs/opt-lab-2/rp-p1-<arm>-s<seed>/` (and WandB).

**Weights deviation [N]:** every arm here, baseline and fast_trot included, uses `splay.json` (abduction −0.1,
roll_asymmetry −1), so the five arms differ only in the reference. The old run used the bare `baseline` profile for
baseline/fast_* and `splay.json` only for the gallops (`recovery/A_code_recovery.md`). The old baseline/fast_trot
numbers below are therefore not weight-matched.

| Arm | WandB run | Reference dir @ commit | Launch commit | Wall clock |
|---|---|---|---|---|
| baseline s42 | `7rvis31p` (`rp-p1-baseline-s42`) | none (stock trot fallback, 1×) | `2e7f5b9` | 13:41–16:03 |
| fast_trot s42 | `ozyh16k2` (`rp-p1-fast_trot-s42`) | `configs/opt-lab-2/fast_trot` @ `e01364f` | `2e7f5b9` | 13:40–16:02 |
| gallop_a s42 | `dhz6j34z` (`rp-p1-gallop_a-s42`) | `configs/opt-lab-2/gallop_a` @ `e01364f` | `2e7f5b9` | 11:12–13:40 |
| gallop_b s42 | `biewl0aa` (`rp-p1-gallop_b-s42`) | `configs/opt-lab-2/gallop_b` @ `e01364f` | `2e7f5b9` | 11:12–13:40 |
| gallop_c s42 | `1c5r0n8c` (`rp-p1-gallop_c-s42`) | `configs/opt-lab-2/gallop_c` @ `e01364f` | `2e7f5b9` | 11:12–13:40 |
| gallop_c s43 | `y38xkkvh` (`rp-p1-gallop_c-s43`) | `configs/opt-lab-2/gallop_c` @ `e01364f` | `a042c72` | 13:52–16:11 |

(The launch commit is HEAD at launch. Reference files are unchanged since `e01364f`, and the launcher records their blob hashes and sha256 in each run's `config.opt_lab_2`.)

### Training log vs the recovered old runs (mean of the last 50 iterations at 3000)

| Arm | New: reward / err_xy / err_yaw / fell | Old [R]: reward / err_xy / err_yaw / fell |
|---|---|---|
| baseline | 164.9 / 0.569 / 0.452 / 0.12 | 164.0 / 0.530 / 0.465 / 0.22 (bare profile) |
| fast_trot | 168.9 / 0.236 / 0.582 / 0.33 | 177.2 / 0.204 / 0.555 / 0.31 (bare profile, unknown params) |
| gallop_a | 144.2 / 0.575 / 0.577 / 0.45 | 140.6 / 0.562 / 0.651 / 0.52 |
| gallop_b | 144.6 / **0.584** / 0.609 / 0.52 | 144.1 / **0.259** / 0.718 / 0.64 |
| gallop_c s42 | **173.4 / 0.217** / 0.604 / 0.28 | 151.8 / 0.251 / 0.714 / 0.52 |
| gallop_c s43 | 173.8 / 0.220 / 0.596 / 0.25 | – |

Curves: [part1/curves.png](part1/curves.png) (solid = new, dotted = old). The data is in `part1/curves.json`. The baseline and gallop_a reproduce the
old numbers to within a few percent. The re-invented gallop_c beats the old gallop_c (err_xy 0.217 vs 0.251, reward 173 vs 152). Its seed 43 lands
within 1.5 % of seed 42 on every training metric. fast_trot is slightly worse than the old fast_trot (0.236 vs 0.204), whose parameters are unknown.
**gallop_b does not reproduce:** the old b reached 0.259, the new b stays at 0.584, because it never learned to run backward (below).
Reward is not comparable across reference dirs (gait_tracking scores each run against its own reference).

## 4. Evaluation [M]

Two harnesses, every checkpoint with the reference dir it trained on, play-mode physics (startup randomization and obs noise on).

1. **Stock `pupper_locomotion evaluate`** via `scripts/opt_lab_2/part1_eval.py`: (a) the default 16-segment sequence (no
   resets, heading hold on); (b) a sweep of ±0.3, ±0.75, ±1.0, ±1.5 m/s from a fresh reset each, 64 envs × 8 s, 3 s warm-up.
   The script adds 3 Nm saturation, foot force in body weights (nominal 3.17 kg), and the contact-measured footfall
   (`scripts/opt_lab_2/part1_footfall.py`: 1-sample debounce, touchdown lags relative to the left hind foot, circular mean).
   Raw outputs: `~/runs/opt-lab-2/p1-evals/`. Summaries: `report/part1/eval/stock/` and `report/part1/eval/stock_table.json`.
2. **W2 extended harness** (`mjlab.scripts.part2_eval suite` + `push`, landed on origin mid-run; definitions in
   `report/part2/eval/README.md` and `report/decisions/2026-11-18-w2-eval-metrics.md`). The suite runs 384 robot-runs over 16 scripted schedules
   with heading hold **off**, falls by category, steady ±1.5 windows, 3 Nm saturation, foot force in actual body weights, drift, splay and slip. The push test
   gives lateral impulses of 0.5/0.65/0.8 m/s. All arms were run through it. Outputs: `report/part1/eval/w2/`.

### 4a. High-speed table (spec deliverable; stock sweep, steady state, mean of +1.5 and −1.5 m/s)

| Arm (WandB) | err_xy ±1.5 (mean / +1.5 / −1.5) | achieved vx +1.5 / −1.5 | err_yaw ±1.5 | mean τ² ±1.5 [Nm²] | slip ±1.5 [m/s] | 3 Nm sat ±1.5 | foot force, median robot max ±1.5 [BW] |
|---|---|---|---|---|---|---|---|
| old baseline (recovered ckpt, bare profile) (`old run baseline-s42-wandb`) | **1.272** / 1.240 / 1.304 | +0.26 / -0.20 | 0.189 | 0.76 | 0.175 | 1.7 % | 1.69 |
| baseline (`7rvis31p`) | **1.303** / 1.288 / 1.317 | +0.21 / -0.19 | 0.195 | 0.74 | 0.163 | 1.4 % | 1.58 |
| fast_trot (`ozyh16k2`) | **0.173** / 0.171 / 0.175 | +1.36 / -1.46 | 0.374 | 2.06 | 0.640 | 12.6 % | 3.48 |
| gallop_a (`dhz6j34z`) | **1.036** / 0.278 / 1.795 | +1.26 / +0.29 | 0.292 | 1.79 | 0.493 | 9.0 % | 2.96 |
| gallop_b (`biewl0aa`) | **1.070** / 0.231 / 1.910 | +1.32 / +0.41 | 0.346 | 1.89 | 0.462 | 9.8 % | 3.23 |
| gallop_c (`1c5r0n8c`) | **0.133** / 0.134 / 0.132 | +1.55 / -1.48 | 0.357 | 2.81 | 0.661 | 17.9 % | 3.23 |

### 4b. W2 suite and push (384 robot-runs; ranked columns per the W2 decision: falls, F_runmax_med_BW, push_stand@0.65, push_move@0.5)

| name | falls | rev | stop | steady | turn | err+1.5 | err-1.5 | sat@1.5 | F_runmax_med_BW | F_p99_BW | drift+ | drift- | splay+ | splay- | slip | top+ | top- | tau2 | R:push_stand@0.65 | R:push_move@0.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| p1-baseline-s42 | 1 | 0 | 0 | 1 | 0 | 1.3 | 1.31 | 0.0128 | 1.85 | 1.23 | 21.8 | 20.3 | -0.0466 | -0.137 | 0.185 | 0.589 | -0.463 | 0.634 | 0.0359 | 0.0942 |
| p1-fast_trot-s42 | 53 | 8 | 44 | 1 | 0 | 0.174 | 0.176 | 0.126 | 3.17 | 1.86 | 45 | 65.9 | -0.134 | 0.00108 | 0.353 | 1.48 | -1.86 | 1.02 | 0.681 | 0.00594 |
| p1-gallop_a-s42 | 69 | 2 | 53 | 14 | 0 | 0.275 | 1.76 | 0.0883 | 2.83 | 1.63 | 64.9 | 64.1 | -0.145 | -0.161 | 0.325 | 1.56 | -0.422 | 1 | 0.85 | 0.00494 |
| p1-gallop_b-s42 | 43 | 2 | 41 | 0 | 0 | 0.221 | 1.89 | 0.0977 | 2.96 | 1.68 | 63.7 | 73.7 | -0.1 | -0.114 | 0.311 | 1.54 | -0.422 | 1.05 | 0.54 | 0.00691 |
| p1-gallop_c-s42 | 44 | 2 | 41 | 1 | 0 | 0.13 | 0.146 | 0.179 | 3.03 | 1.88 | 67.6 | 65.7 | 0.0352 | 0.115 | 0.382 | 1.75 | -1.76 | 1.35 | 0.453 | 0.00197 |
| p1-gallop_c-s43 | 41 | 6 | 35 | 0 | 0 | 0.123 | 0.129 | 0.176 | 3.09 | 1.93 | 53.8 | 51.3 | 0.0469 | 0.106 | 0.382 | 1.69 | -1.72 | 1.37 | 0.00901 | 0 |

(err, sat, F, drift [deg/10 s, heading hold off], splay [rad, + = hind feet outward], slip, top [m/s], tau2 [Nm²].
GPU rollouts are not bit-reproducible: treat fall gaps under ~10–15 as ties. `R:push_stand@0.65` is very seed-sensitive:
gallop_c gives 0.453 at s42 and 0.009 at s43.)

### 4c. Footfall measured from contacts (stock sweep; label, stride frequency, achieved vx, flight fraction)

| Arm | +0.30 | +0.75 | +1.00 | +1.50 | -0.30 | -0.75 | -1.00 | -1.50 |
|---|---|---|---|---|---|---|---|---|
| old baseline (recovered ckpt, bare profile) | trot (1.4 Hz, vx +0.23, fl 0%) | other* (3.1 Hz, vx +0.61, fl 0%) | other* (3.1 Hz, vx +0.59, fl 0%) | trot (1.4 Hz, vx +0.26, fl 0%) | trot* (2.1 Hz, vx -0.29, fl 0%) | trot* (3.3 Hz, vx -0.39, fl 0%) | trot (2.2 Hz, vx -0.34, fl 0%) | other (1.6 Hz, vx -0.20, fl 0%) |
| baseline | trot (1.4 Hz, vx +0.24, fl 0%) | other* (2.9 Hz, vx +0.46, fl 0%) | trot* (2.4 Hz, vx +0.42, fl 0%) | trot (1.4 Hz, vx +0.21, fl 0%) | trot* (3.1 Hz, vx -0.29, fl 0%) | trot* (2.9 Hz, vx -0.33, fl 0%) | trot (1.6 Hz, vx -0.29, fl 0%) | other (1.6 Hz, vx -0.19, fl 0%) |
| fast_trot | trot (1.4 Hz, vx +0.24, fl 0%) | trot (2.6 Hz, vx +0.74, fl 0%) | trot (2.6 Hz, vx +0.97, fl 2%) | trot (2.6 Hz, vx +1.36, fl 6%) | trot (1.4 Hz, vx -0.23, fl 0%) | trot (2.6 Hz, vx -0.74, fl 0%) | trot (2.6 Hz, vx -0.96, fl 0%) | trot (3.3 Hz, vx -1.46, fl 6%) |
| gallop_a | trot (1.4 Hz, vx +0.22, fl 0%) | bound (2.6 Hz, vx +0.77, fl 2%) | bound (2.6 Hz, vx +0.96, fl 6%) | bound* (2.6 Hz, vx +1.26, fl 24%) | trot (1.4 Hz, vx -0.22, fl 0%) | other (4.5 Hz, vx +0.33, fl 16%) | other (4.2 Hz, vx +0.33, fl 15%) | other (4.2 Hz, vx +0.29, fl 15%) |
| gallop_b | trot (1.4 Hz, vx +0.22, fl 0%) | rotary LH>RH>RF>LF (2.6 Hz, vx +0.73, fl 1%) | rotary LH>RH>RF>LF (2.6 Hz, vx +0.96, fl 1%) | rotary LH>RH>RF>LF (2.6 Hz, vx +1.32, fl 7%) | trot (1.4 Hz, vx -0.24, fl 0%) | other (2.8 Hz, vx +0.46, fl 10%) | other (2.8 Hz, vx +0.45, fl 10%) | other (2.6 Hz, vx +0.41, fl 10%) |
| gallop_c | trot (1.4 Hz, vx +0.23, fl 0%) | rotary LH>RH>RF>LF (3.1 Hz, vx +0.77, fl 0%) | rotary LH>RH>RF>LF* (4.2 Hz, vx +1.01, fl 0%) | other* (4.2 Hz, vx +1.55, fl 5%) | trot (1.6 Hz, vx -0.23, fl 0%) | rotary RH>LH>LF>RF (2.9 Hz, vx -0.76, fl 0%) | rotary RH>LH>LF>RF* (3.1 Hz, vx -1.00, fl 0%) | transverse RH>LH>RF>LF* (4.5 Hz, vx -1.48, fl 1%) |

`*` = lag concentration < 0.5 for some foot (an irregular cycle; the label is the best guess). The reference cadences are
1.33 Hz (trot slot, |v| < 0.5), 2.67 Hz (fast_trot, a, b) and 3.0 Hz (c).

## 5. Commentary: what the designs got right and wrong

- **The footprint rule held.** The stock trot implies 0.20 m/s, and both baselines top out at about 0.2–0.26 m/s at a ±1.5
  command (err_xy 1.27–1.30, the old and new baselines agree). The robot falls back to the 1.33 Hz reference cadence and gives up on speed.
  Every authored reference with `v_ref` ≥ 0.84 m/s reaches 1.26–1.55 m/s forward. Training *stretched* the references:
  gallop_c's reference implies 0.84 m/s, and the policy runs 1.55 m/s by raising its cadence from the reference's 3.0 Hz
  to 4.2 Hz. fast_trot keeps the reference's 2.6 Hz and lengthens its stride.
- **Gait identity survived training.** fast_trot trots at every speed. gallop_b gallops forward with the designed
  rotary footfall LH→RH→RF→LF and lags (BR 0.13–0.14, FR 0.53–0.56, FL 0.66 vs the designed 0.15/0.5/0.65).
  gallop_a runs a near-**bound**, as designed, with 24 % flight at +1.5. gallop_c trots at ±0.3 and gallops at ≥ 0.75 in both
  directions; backward it runs the time-reversed (right-lead) rotary gallop RH→LH→LF→RF. This matches the old C [R] ("trotted at ±0.3, galloped
  at ≥ 0.75"). At ±1.5 its cycle loses regularity (lag concentration 0.2–0.5).
- **Wrong: gallops a and b cannot run backward.** Commanded −0.75…−1.5 m/s, both move **forward** at +0.3–0.46 m/s,
  in a 4–4.5 Hz (a) or 2.8 Hz (b) shuffle with 10–16 % flight (err_xy 1.1–1.9). This is the old gallop A's failure [R]
  reproduced, and here it also hits b. Only c, the design with at least two feet always down, learned the time-reversed gait. Hypothesis
  (untested): playing a gait with flight or single-support phases in reverse turns each landing into a push-off from the
  wrong leg pair, and the policy finds forward motion that still matches the reference better than slow backward motion.
  A test would be b's footfall with duty ≥ 0.5.
- **Wrong: gallop_a tips forward.** Open loop it pitches 13° nose-down (§2). Trained, it has the most falls (69 in the suite,
  including 14 steady-state falls; 27 in the stock sequence, mostly when standing), the worst standing push rate (0.85) and the
  largest single-sample foot forces (12–16 BW, flag only). This is the old A's "tips forward" [R].
- **The cost of speed is torque.** Every arm that runs fast saturates the 3 Nm actuators 9–18 % of the time at ±1.5
  (baseline 1.4 %). gallop_c is the hungriest (17.9 %, τ² 2.8 Nm²), which matches its design (highest reference joint speed,
  kp·Δq 3.8 Nm > 3 Nm). fast_trot is at 12.6 % and 2.1 Nm².
- **Heading drift:** with heading hold off, every fast policy drifts 45–74 deg/10 s at ±1.5 (baseline 20–22). It is large and
  similar across arms, so it does not separate them, but it is a sim-to-real risk.
- **Splay:** the gallops pull the hind feet in going fast (a, b: −0.10…−0.16 rad), while c holds them slightly outward
  (+0.04/+0.12). The splay weight is the same in every arm.

## 6. Failures and surprises

- gallop_b did not reproduce the old b's backward running (old training err_xy 0.259 vs new 0.584). The old b's
  parameters are lost, so this is a different b, not a regression.
- The stock trot reference exceeds the robot's joint range (front knee, 0.09 rad past the model limit, 0.10 past the driver
  limit) and is only usable because `_reference_table` clamps it. `bootstrap.py design` refuses such tables, so the
  baseline's FK analysis used the clamped table.
- `visualize_reference` is interactive only. The gifs come from an offscreen re-implementation of its `ReferencePolicy`
  loop (`part1_gait_check.py render`).
- The first `part2_eval batch` call failed on argument order (`--modes` swallowed the entries). It was rerun, and nothing was lost.
- Throughput: the GPU is flat at about 1.25 it/s in total, so three concurrent runs took 2.5 h each (3–4 s/iter). Stock evals
  co-scheduled with training slowed it from 3.1 to 4.0 s/iter, so later evals were batched.

## 7. Part 2 parent (provisional)

**Provisional parent: `gallop_c` seed 42** (`1c5r0n8c`, `~/runs/opt-lab-2/rp-p1-gallop_c-s42/model_2999.pt`, reference
`configs/opt-lab-2/gallop_c` @ `e01364f`). **Top 2 for seed 43: gallop_c and fast_trot.** This is a human decision point made
lazily: three independent graders agreed 3/3, and it is pending human review. See
[decisions/2026-11-18-p1-parent-gait.md](decisions/2026-11-18-p1-parent-gait.md) for the reasoning, flagged risks and flip conditions.

In short: only fast_trot and gallop_c can run at both +1.5 and −1.5 m/s, which the Part 2 capture needs. On the ranked W2 metrics, c
leads fast_trot on every one, consistently over two seeds, but each gap is within noise. The tie-breaks are better ±1.5 tracking and fewer
reversal falls, and c is the same parent the old run chose (REPRO_PLAN §2). Main risks: 3 Nm saturation (17.9 % vs 12.6 % at ±1.5),
and an irregular 4.2 Hz cycle against a 3.0 Hz clock, which may smear the fold. Check the capture before Part 2 round 1.

Seed 43: gallop_c s43 (`y38xkkvh`) was launched early (13:52) because a GPU slot was free and c led a/b clearly on the stock
sequence. That was a scheduling call, not a pick. It reproduces s42 within 1.5 % on training metrics and on W2 tracking and foot force
(err ±1.5 0.123/0.129, F_runmax_med 3.09 BW, 41 falls). fast_trot s43 (`pgsoywpz`) was launched at 16:24 after the decision.
Its result is listed under "what would flip this decision".


## Appendix: surviving fragment of the lost notebook (b916f17, recovered from W&B diffs) [R]

Kept verbatim for comparison. The "pick" below refers to the *old* designs, whose parameters are lost.

<!-- RECOVERY GAP: lines 1-88 of the b916f17 version are not in any W&B diff. -->
The backward reference is correct (checked from the reference table). With a 0.1
<!-- RECOVERY NOTE: the line above is only the truncated hunk-header context of line 88; its remainder is unknown. -->

#### The pick (old run)

Pick by sim-to-real readouts first, then tracking and slip ([criteria](part1_gallop_design.md#design-choices)). `fast_gallop_a` is out: it can't run backward. b and c track equally and saturate torque equally. **c** lands softer (5.9 vs 6.7 body weights), drifts less going forward (1.5 vs 7.8 deg per 10 s) and always keeps two feet down. b has fewer stop falls (9 vs 15 of 96) and less backward drift (-0.6 vs -3.5); c has fewer reversal falls (15 vs 19 of 160). c's weak point is hind splay and body roll going backward (-0.14 rad, 5.1 deg). Part 2 round 1 references were captured from c (commit `ba6a5f9`).

#### Superseded design: fast trot (old run)

<!-- RECOVERY GAP: everything after line 95 of the b916f17 version is not in any W&B diff. -->
