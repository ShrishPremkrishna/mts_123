# Which Part 2 checkpoint is the final policy that goes to Part 3?
Status: DECIDED by grader (pending synthesis) · 2026-11-18 · Requested by: W3 · Grader 1, lens: devil's advocate (front-runner: `p2-s42-r2-cont-1500`)
Criticality: Critical. It feeds the report's headline Part 2 claim, it is the +0 of the Part 3 fine-tune, and it may later be prepared for hardware.
Reversible?: Before Part 3 launches, yes, for free. After launch, a re-pick costs the Part 3 arm pair (~2 × 1500 it, ~40 GPU-min) plus the step-0 and guard evals (~1 h).

## Conclusion
**The front-runner stands.** The final is `~/runs/opt-lab-2/rp-p2-s42-r2-cont-1500/model_4496.pt` (WandB `wmhcp24q`, seed 42, +1500 past the designated parent `1c5r0n8c`), with reference `configs/opt-lab-2/p2/s42/r2-lag`. It goes to Part 3 with two flags: (1) foot impact regressed in s42, and (2) its nominal-PD gap is above 15, so Part 3 runs the WIDEN branch and the HOLD rule stays live.

## Evidence used
- `matched_summary.md` at +1500 (per-seed delta vs the same-seed ctrl-1500; arm s42/s43):
  - **falls:** 16/15, Δ −19/−3, mean −11.
  - **R:push_stand@0.65:** 0.099/0.148 vs ctrl 0.298/0.498, Δ −0.199/−0.351. This is the largest ranked effect, it agrees in sign, and it is far above the repeat max of 0.040.
  - **sat@1.5:** 0.160/0.157, Δ −0.025/−0.024. The gate passes.
  - **F_runmax_med_BW:** 3.04/2.96, Δ +0.136/−0.037. The two-seed mean is 3.00, the worst +1500 arm (r3 2.83, r1-cont 2.99, ctrl 2.95).
  - **push_move@0.5:** 0 for every arm.
- `repeat_spread.md`:
  - falls max |diff| is 13 (s43-r3 gave 16 then 29), and F_runmax max |diff| is 0.076.
  - Averaged over the two repeats, the front-runner's s42 F_runmax delta is +0.186 and its falls delta is −17.5/−4.
- `nompd_check.md` (verified against `w3-nompd/*/ablate/*.md`). Nominal-PD falls, with nompd − flat_dr in parentheses:

  | Arm | s42 | s43 |
  |---|---|---|
  | front-runner (r2-cont) | 39/37 (+22.5) | 44/30 (+17.5) |
  | r3 | 52/41 (+24.0) | 30/37 (+9.0) |
  | ctrl | 68/57 (+29.5) | 31/31 (+12.0) |

  Two-seed arm means of nompd falls are: r2-cont 37.5, r3 40.0, r1-cont 46.5, ctrl 46.75. The s42 parent (W4) is 99/79 (+48.5).
- `matched_summary.md` at +1000: the front-runner's own parent, `r2`, had **34/39 falls (Δ +3/+15)**, the worst arm at +1000.
- `~/recovery/C_lineage_metrics.md` §3d: in the old run, r2cont ≈ r3, a near tie, and r2cont was picked.

## Case against the front-runner (strongest form)
1. **The falls lead is one seed.** At s43 the delta is −3, a tie (−4 on the repeat average). The two-seed mean of −11 sits inside the 10–15 tie band.
2. **Foot impact gets worse on a ranked metric, beyond noise.** In s42, F_runmax is +0.136 vs ctrl (+0.186 on the repeat average), which is about 2× the repeat max of 0.076. It is the worst of all +1500 arms on the two-seed mean. r3 is better at both seeds (−0.018/−0.226) and also leads on p99, p99.9, tau2 and drift−. The front-runner's F_runmax (3.04) only matches the parent; it does not improve on it.
3. **The nompd gap is above 15 at both seeds** (+25/+20 in s42). That is exactly the deployment HOLD condition (a) in `w4-kp-coverage`. s43-r3 (+9) and s43-ctrl-1500 (+12) are the only checkpoints under 15, and they have the lowest nompd falls (33.5, 31.0).
4. **Snapshot luck.** One 500-iteration leg moved this lineage from 34/39 falls to 16/15. The repeat evals measure only eval noise, all on eval seed 1234, so checkpoint-to-checkpoint training noise is unmeasured.
5. **The other seed's checkpoint** (s43 r2-cont-1500) ties on falls (15) and on nompd (37). It has lower F_runmax (2.955 vs 3.043, and 2.955 vs 3.077 on the repeat average) and a smaller nompd gap (+17.5).

## Why the case fails
- **The nompd advantages do not replicate across seeds.**
  - r3's gap is +24.0 at s42 but +9.0 at s43, and ctrl's nompd is 62.5 vs 31.0. These are parent effects, not arm effects. Per the s43-lineage review, a sign disagreement means "not robust to the parent".
  - r2-cont is the only arm whose nompd level is the same at both seeds (38 / 37), and it has the best two-seed arm mean (37.5).
  - Its gap is still above 15, but it is half the parent's gap (+48.5), and nompd falls drop from 89 to 38.
  - W4 already chose WIDE from the parent's step 0, so Part 3's plan does not change.
- **On standing push, the front-runner's lead is large, agrees across seeds and is beyond noise.**
  - r3 and s43-ctrl lose here by 0.3–0.4 falls/push (r3 0.407/0.334; s43-ctrl 0.498).
  - The front-runner's F_runmax penalty (0.17 BW, about 6 %) is small next to that, and it does not regress vs the parent.
- **The s43 r2-cont checkpoint** gains about 0.09–0.12 BW in F_runmax but loses 0.05 on push_stand (above the 0.04 repeat max). It descends from `y38xkkvh`, not the designated parent, and only s42 is comparable with the recovered §3d. That is not enough to switch.
- **Keeping the parent is dominated** at matched readouts:
  - falls: 43 vs 16;
  - push_stand: 0.455 vs 0.099;
  - sat@1.5: 0.179 vs 0.160;
  - nompd: 99/79 vs 39/37.

  F_runmax is equal (3.04).

## Alternatives considered
| Option | Why not chosen |
|---|---|
| s42 r3 (`p2/s42/r3-lag`) | It is better on F_runmax (2.89) and ties on sat@1.5, but it is worse on push_stand (0.407 vs 0.099) and nompd (46.5 vs 38). Its falls are a tie. |
| s43 r3 | It has the best nompd gap (+9), but that does not replicate at s42 (+24). It has push_stand 0.334 and descends from the non-designated parent. |
| s43 r2-cont-1500 | It is the same arm and ties on falls and nompd. It has lower F_runmax but worse push_stand, and it is the off-lineage parent. It is the runner-up. |
| s43 ctrl-1500 | Its nompd is lowest (31), but at s42 it is 62.5. Its push_stand is 0.498, and it has no bootstrapping, so it cannot carry the Part 2 claim. |
| Keep the parent `1c5r0n8c` | It is dominated on falls, push, saturation and nompd. |

## Severe flagged issues
- **F_runmax regresses in s42** (+0.136 vs ctrl, beyond noise). Report it next to the headline; do not hide it behind the two-seed mean.
- **nompd − flat_dr is +22.5**, which is above the HOLD threshold. Part 3 must run WIDEN, and hardware is HOLD until a fine-tuned checkpoint clears both (a) and (b), with nompd(P) = 38.0.
- The falls ranking is effectively s42-only. Training-snapshot noise is unmeasured (r2 34/39 → r2-cont 16/15 in one leg), and every eval used seed 1234.
- Simulation only.

## What would change my mind
- A second eval seed (4321) on the W2 suite and push test showing that the front-runner's push_stand lead over r3 shrinks below ~0.1, or that its falls lead disappears. Then r3 (lower impact) would be the pick.
- Neighbouring checkpoints (e.g. model_4400 or 4450) of r2-cont-1500 vs r3 showing that the falls/push ordering flips between snapshots. That would mean the +1500 pick is snapshot luck.
- A hardware or actuator finding that foot impact (F_runmax) is the binding risk, rather than standing push or nominal-gain falls.
- A ruling that Part 3 should run KEEP (Lab 4 PD ranges): then a checkpoint with gap ≤ 15 that replicates across seeds would be needed, and none exists.
