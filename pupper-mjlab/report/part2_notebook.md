# Part 2 lab notebook: reference bootstrapping on gallop_c (reproduction, W3)

Task `Mjlab-MixedGaits-Flat-Pupper-v3`. Spec: [Opt Lab 2, Part 2](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-2.html).
Started 2026-11-18. This redoes the lost Part 2 ([REPRO_PLAN.md](REPRO_PLAN.md) §2.3–2.5) with a second seed added. Everything
below is **simulation evidence only**.

Tags: **[R]** recovered from the old run, **[N]** new in this reproduction, **[M]** measured here.

<!-- STATUS -->

## 1. Setup

| Item | Value |
|---|---|
| Parent (s42 lineage) | `gallop_c` s42, `~/runs/opt-lab-2/rp-p1-gallop_c-s42/model_2999.pt`, WandB `1c5r0n8c` ([decision](decisions/2026-11-18-p1-parent-gait.md)) |
| Parent (s43 lineage) | `gallop_c` s43, `~/runs/opt-lab-2/rp-p1-gallop_c-s43/model_2999.pt`, WandB `y38xkkvh` ([decision](decisions/2026-11-18-w3-s43-lineage.md): own parent, own capture, 2/2 graders) |
| Parent reference | `configs/opt-lab-2/gallop_c` @ `e01364f` (3.0 Hz clock, `frequency_mult` 2.25) |
| Weights (every Part 2 leg) | `configs/opt-lab-2/weights/splay_stand_still.json` = the parent's `splay.json` + `stand_still_joint_velocity` −0.1 + `stand_still_pose` −0.5 [R] |
| Leg | 500 PPO iterations, 4096 envs, `--profile baseline`, standard physics flags, warm start (optimizer state kept; no airborne pretrain) via `scripts/opt_lab_2/launch.py` |
| Fold (every round) | `lag`, fixed 2-step shift, median, 6 harmonics, no symmetrize, all four `leg_*_2` pinned to the parent ([decision](decisions/2026-11-18-w2-fold-default.md)) |
| Capture | ±1.5 m/s constant command, 64 envs × 15 s each, first 3 s after a change/reset dropped, heading hold off (`part2_reference capture`) |
| Eval | W2 suite (384 robot-runs) + push test, each checkpoint with the reference it trained on ([metrics decision](decisions/2026-11-18-w2-eval-metrics.md)) |
| Orchestration | `scripts/opt_lab_2/part2_queue.py` (DAG: capture → fold → gate → commit+push reference → leg; GPU cap 4 across agents; crash resume), `part2_evalq.py`, `part2_analysis.py` |

**Why a control.** The parent trained on `splay.json`; every Part 2 leg uses `splay_stand_still.json`. The control
arm (parent reference, same new weights, same extra iterations) isolates the reference change from the weight change
and from the extra training (REPRO_PLAN §3).

## 2. Round 0: is gallop_c foldable? [M]

The parent decision flagged the fold as the top risk: Part 1's contact-based footfall labelling reported gallop_c
stepping at 4.2–4.5 Hz against a 3.0 Hz clock with an irregular cycle, so a clock-phase fold might smear it.
I captured both parents at ±1.5 m/s and folded each with the default, plus three diagnostic variants
(auto lag, naive, unpinned). Extra diagnostics come from `scripts/opt_lab_2/part2_foldcheck.py`
(`report/part2/foldcheck/p0-gallop_c-s4{2,3}*.json`).

| Check (per direction fwd / back) | s42 capture | s43 capture | Pass rule |
|---|---|---|---|
| Steady samples, clean cycles, falls, vx | 38.4k / 38.3k, 2175 / 2174 cycles, 2 / 3 falls, vx +1.56 / −1.53 | 38.3k / 38.3k, 2172 / 2176, 5 / 3 falls, vx +1.55 / −1.51 | ≥ 20 clean cycles |
| Joint spectrum (512-step Hann windows, 0.1 Hz bins) | peak **3.03 Hz** (= clock), 2nd harmonic 6.0 Hz at 0.64–0.74 of the peak, **power at 4.2 Hz ≈ 0.001** | same (dominant 2.93 Hz at 0.2 Hz bins) | joints locked to the 3.0 Hz clock |
| Debounced touchdowns per foot | 3.5 Hz (raw contact edges 5.6–7.4 Hz: two contacts per stance) | similar | — |
| Measured lag (auto, steps) | 2.09 / 1.83 | 2.13 / 1.76 | \|auto − 2\| ≤ 0.5 |
| Fold dispersion, wrap closure (rad) | 0.061 / 0.068, 0.003 / 0.004 | 0.060 / 0.061, 0.003 / 0.002 | — |
| Flagged channels (default 4 pins) | none | none | none |
| Unpinned fold flags | all four `_2` fwd, front `_2` back (captured `_2` range 0.29–0.36 rad vs parent 0.015 / 0.059) | same | pin set confirmed |
| Fold residual, RMS q − ref(clock − 2 steps) | **0.097 / 0.094** (parent table 0.141 / 0.144) | **0.097 / 0.087** (parent 0.144 / 0.144) | fold explains the capture better than the parent |
| R² of captured motion (moving channels) | default 0.68 / 0.64; unpinned 0.89 / 0.85; parent 0.59 / 0.50 | 0.68 / 0.64; 0.89 / 0.87; 0.58 / 0.49 | — |
| Amplitude retention (table range / captured 5–95 % range) | unpinned 0.96 / 0.95 (default 0.70, lowered by the pins only) | 0.95 / 0.95 | no smearing |
| kp·Δq proxy (kp 5.5, 20 ms) | **2.17 / 2.53 Nm** (parent 3.82) | **2.21 / 2.55 Nm** | < 3 Nm |
| Peak joint speed | 20.5 / 24.4 rad/s (parent 44.8) | 20.6 / 23.8 | — |
| Driver-limit margin | ≥ 0.31 rad | ≥ 0.31 rad | ≥ 0.05 rad |
| Loader verify | ok | ok | ok |

**Verdict: the fold is not broken.** The "4.2 Hz" in Part 1 came from contact-edge counting: the policy makes two
contacts per stance, which the footfall classifier read as a faster stride. The joint trajectories are locked to the
3.0 Hz reference clock (the spectral peak sits on the clock bin and there is no power at 4.2 Hz), so a clock-phase fold
is the right tool. Its residual is lower than the parent table's, nothing is smeared, the lag fit brackets 2.0, and the
folded table needs **less** torque than the parent reference (kp·Δq 2.2–2.5 vs 3.8 Nm): the policy runs a lower-amplitude
version of the authored gallop. No fold-variant decision was needed, and the parent-gait flip condition ("capture
fails periodicity, residual or limits, or kp·Δq rises well above 4 Nm") is not met.

Two checks confirm the timing convention independently. The parent table predicts the capture with R² 0.59 at a 2-step
shift and −0.24 at zero shift; the naive fold predicts it with R² 0.63 at zero shift (`*-naive-lag0.json`).

The pins carry a real cost: the policy moves its abduction joints 0.29–0.36 rad (5–95 % range) while the pinned reference
moves them 0.015–0.06 rad, so the pins keep 30 % of the captured variance out of the reference. That is the decided
trade-off (the unpinned fold reproduces the recovered crouch-collapse signature on all four `_2` channels).


## 3. Rounds: hypothesis, comparator, metric

Written before any Part 2 eval landed. "Better" means a two-seed mean delta vs the same-seed control that agrees in sign
at both seeds and is outside the noise band (single-eval falls ±10–15; other bands from the repeat evals in §5).
**sat@1.5 is a hard gate**: an arm that raises it beyond noise vs its matched control cannot be the final policy.

| Arm (both seeds unless noted) | Built from | Hypothesis | Comparator (matched) | Primary metric | Secondary |
|---|---|---|---|---|---|
| `r1-lag` | capture of the parent, default fold | A reference the policy can actually follow (lower amplitude, kp·Δq 2.2–2.5 Nm) improves high-speed tracking without costing falls or torque [R: 0.241 → 0.233 training err_xy] | `ctrl-500` | err ±1.5 | falls, sat@1.5, F_runmax, push |
| `r1-naive` | same capture, no lag shift | Without the 2-step shift the reference runs 2 steps (≈ 12 % of the 16.7-step cycle) late, which the policy must undo, so it tracks worse than `r1-lag` | `r1-lag`, `ctrl-500` | err ±1.5 | gait_tracking reward |
| `r1-backpin` (s42 only) | default fold with only the back `_2` pinned | Ablation of the front-abduction pin: unpinned front `_2` shifts by ≈ its range; expected tie or worse splay | `r1-lag`, `ctrl-500` | splay, falls | err ±1.5 |
| `ctrl-500/1000/1500` | parent reference | Effect of the stand-still terms + longer training alone [R: most fall/jitter gains come from here] | parent (+0) for the weight change; itself across levels | falls, jitter | push_stand@0.65 |
| `r2` | capture of `r1-lag` | A second capture moves the reference less (fixed point near) and adds a smaller tracking gain | `r1-cont-1000`, `ctrl-1000` | err ±1.5 | falls, sat@1.5 |
| `r1-cont-1000/1500` | `r1-lag` continued, same reference | Separates "new reference" from "more iterations on the round-1 reference" | `r2` / `r3`, `r2-cont-1500` | err ±1.5 | falls |
| `r3` | capture of `r2` | Diminishing returns: r3 ≈ r2-cont [R] | `r2-cont-1500`, `r1-cont-1500`, `ctrl-1500` | err ±1.5, falls | sat@1.5, F_runmax |
| `r2-cont-1500` | `r2` continued | The old final pick [R] | `r3`, `ctrl-1500` | as above | |
| stand-still terms (all arms vs parent) | — | Stand-still terms make standing push recovery worse [R: 6.7–10.4 % vs 5.4 % falls/push] | parent (+0, `splay.json`) — no iteration-matched `splay.json` arm exists | push_stand@0.65 | jitter |


## 4. Runs

<!-- RUNS -->

## 5. Matched results

<!-- MATCHED -->

## 6. Reward-term trajectories

<!-- REWARDS -->

## 7. Did the recovered findings reproduce?

<!-- FINDINGS -->

## 8. Where we stopped, and the final policy

<!-- FINAL -->

## 9. Failures, superseded designs and surprises

<!-- FAILURES -->
