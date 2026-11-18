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

All 21 legs ran to their target iteration with **no crash and no resume** (one segment each), 4096 envs,
`splay_stand_still.json`, 500 iterations each, chained so the final indices are 3498 / 3997 / 4496 at +500 / +1000 / +1500.
Run dirs `~/runs/opt-lab-2/rp-<name>/`; WandB project `photon_mz-stanford-university/mjlab`, tags `rp, part2, w3`.
`launch.py` refused any unpushed input, logged the reference dir and weights as WandB artifacts, and recorded the
git SHA. No training code changed between launches (the only `src/` changes on the branch in this window are W4's new
`part3_*.py` scripts, which training does not import).

| Leg | WandB | Warm start from | Reference (commit) | Launch HEAD | Wall clock (UTC) |
|---|---|---|---|---|---|
| p2-s42-r1-lag | `t68nrwkw` | rp-p1-gallop_c-s42/model_2999 | p2/s42/r1-lag (`1611a75`) | 1611a75 | 17:32–18:02 |
| p2-s42-r1-naive | `xoit2y5d` | parent s42 | p2/s42/r1-naive (`1611a75`) | 1611a75 | 17:33–18:04 |
| p2-s42-r1-backpin | `fx4oyomc` | parent s42 | p2/s42/r1-backpin (`1611a75`) | 1611a75 | 17:33–18:07 |
| p2-s42-ctrl-500 | `fqt1qabi` | parent s42 | gallop_c (`e01364f`) | 1611a75 | 17:33–18:04 |
| p2-s42-r2 | `2i9g5098` | p2-s42-r1-lag/3498 | p2/s42/r2-lag (`60ce5a4`) | 2015e16 | 18:17–18:48 |
| p2-s42-r1-cont-1000 | `jkh7iqck` | p2-s42-r1-lag/3498 | p2/s42/r1-lag | 27c48ff | 18:04–18:31 |
| p2-s42-ctrl-1000 | `62culz7u` | p2-s42-ctrl-500/3498 | gallop_c | e926ff8 | 18:42–19:14 |
| p2-s42-r3 | `rrdkrrzw` | p2-s42-r2/3997 | p2/s42/r3-lag (`90b3e59`) | 90b3e59 | 18:55–19:27 |
| p2-s42-r2-cont-1500 | `wmhcp24q` | p2-s42-r2/3997 | p2/s42/r2-lag | a898f59 | 19:43–20:14 |
| p2-s42-r1-cont-1500 | `e79oihhl` | p2-s42-r1-cont-1000/3997 | p2/s42/r1-lag | a898f59 | 19:47–20:17 |
| p2-s42-ctrl-1500 | `eoohixa0` | p2-s42-ctrl-1000/3997 | gallop_c | a898f59 | 19:49–20:19 |
| p2-s43-r1-lag | `bcsvvl03` | rp-p1-gallop_c-s43/model_2999 | p2/s43/r1-lag (`27c48ff`) | 27c48ff | 18:07–18:37 |
| p2-s43-r1-naive | `vhbultky` | parent s43 | p2/s43/r1-naive (`27c48ff`) | e926ff8 | 18:33–19:06 |
| p2-s43-ctrl-500 | `e9wyyh4s` | parent s43 | gallop_c | 637f16e | 18:09–18:40 |
| p2-s43-r2 | `v3srag94` | p2-s43-r1-lag/3498 | p2/s43/r2-lag (`05b3371`) | 05b3371 | 18:45–19:17 |
| p2-s43-r1-cont-1000 | `l9fv0ek9` | p2-s43-r1-lag/3498 | p2/s43/r1-lag | 90b3e59 | 19:14–19:43 |
| p2-s43-ctrl-1000 | `2q2wjdt4` | p2-s43-ctrl-500/3498 | gallop_c | 90b3e59 | 19:15–19:43 |
| p2-s43-r3 | `adn6hd33` | p2-s43-r2/3997 | p2/s43/r3-lag (`a898f59`) | a898f59 | 19:27–19:56 |
| p2-s43-r2-cont-1500 | `rwfqzxv2` | p2-s43-r2/3997 | p2/s43/r2-lag | a898f59 | 19:56–20:26 |
| p2-s43-r1-cont-1500 | `0jknmncx` | p2-s43-r1-cont-1000/3997 | p2/s43/r1-lag | a898f59 | 20:18–20:44 |
| p2-s43-ctrl-1500 | `4r1w7euc` | p2-s43-ctrl-1000/3997 | gallop_c | a898f59 | 20:19–20:44 |

**Reference provenance per round** (captured by `part2_queue.py`, gated, committed and pushed before the leg that uses it):

| Reference | Captured from | Measured lag fwd / back (steps) | RMS change vs the reference it was captured with (rad) | kp·Δq fwd / back (Nm) |
|---|---|---|---|---|
| p2/s42/r1-lag | parent s42 | 2.09 / 1.83 | 0.099 / 0.109 (vs gallop_c) | 2.17 / 2.53 |
| p2/s42/r2-lag | p2-s42-r1-lag/3498 | 2.00 / 1.91 | 0.031 / 0.044 | 2.24 / 2.45 |
| p2/s42/r3-lag | p2-s42-r2/3997 | 2.05 / 1.86 | 0.026 / 0.038 | 2.17 / 2.36 |
| p2/s43/r1-lag | parent s43 | 2.13 / 1.76 | 0.102 / 0.116 | 2.21 / 2.55 |
| p2/s43/r2-lag | p2-s43-r1-lag/3498 | 2.10 / 1.87 | 0.037 / 0.043 | 2.25 / 2.35 |
| p2/s43/r3-lag | p2-s43-r2/3997 | 2.05 / 1.92 | 0.032 / 0.042 | 2.34 / 2.12 |

The capture → fold loop converges: round 1 moves the table 0.10–0.12 rad from the authored gallop, rounds 2 and 3
move it only 0.03–0.04 rad, and every table stays under the 3 Nm proxy with no flagged channel and a lag inside 2 ± 0.25.
No gate ever blocked.

**Scheduling notes.** The DAG ran the critical path (r1-lag → capture → r2 → capture → r3) first. W4 held one GPU slot
for Part 3 evals most of the time, so W3 ran three trainers. Aggregate throughput was about 1.1 it/s. Training ran
17:32–20:44 (3 h 12 min for 10.5k iterations), and the evals and repeats finished at 20:48.


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
