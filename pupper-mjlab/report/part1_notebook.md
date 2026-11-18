# Part 1 lab notebook: authoring the fast gait (reproduction, W1)

Task `Mjlab-MixedGaits-Flat-Pupper-v3`. Spec: [Opt Lab 2, Part 1](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-2.html).
Started 2026-11-18. This redoes Part 1 after the first machine's gait parameters and reference files were lost
(see [REPRO_PLAN.md](REPRO_PLAN.md) §1–2). Everything below is **simulation evidence only**.

Tags: **[R]** recovered from the old run, **[N]** new in this reproduction, **[M]** measured here.

<!-- RESULTS-PENDING -->

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

## Appendix: surviving fragment of the lost notebook (b916f17, recovered from W&B diffs) [R]

Kept verbatim for comparison. The "pick" below refers to the *old* designs, whose parameters are lost.

<!-- RECOVERY GAP: lines 1-88 of the b916f17 version are not in any W&B diff. -->
The backward reference is correct (checked from the reference table). With a 0.1
<!-- RECOVERY NOTE: the line above is only the truncated hunk-header context of line 88; its remainder is unknown. -->

#### The pick (old run)

Pick by sim-to-real readouts first, then tracking and slip ([criteria](part1_gallop_design.md#design-choices)). `fast_gallop_a` is out: it can't run backward. b and c track equally and saturate torque equally. **c** lands softer (5.9 vs 6.7 body weights), drifts less going forward (1.5 vs 7.8 deg per 10 s) and always keeps two feet down. b has fewer stop falls (9 vs 15 of 96) and less backward drift (-0.6 vs -3.5); c has fewer reversal falls (15 vs 19 of 160). c's weak point is hind splay and body roll going backward (-0.14 rad, 5.1 deg). Part 2 round 1 references were captured from c (commit `ba6a5f9`).

#### Superseded design: fast trot (old run)

<!-- RECOVERY GAP: everything after line 95 of the b916f17 version is not in any W&B diff. -->
