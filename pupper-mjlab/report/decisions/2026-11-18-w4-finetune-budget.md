# How many iterations should the Part 3 Bumpy + Lab 4 DR fine-tune run?
Status: DECIDED by graders (2/2 agree; pending human review) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: Medium. It sets the one Part 3 training run (about 0.4–0.75 GPU-hours) and nothing else is matched against it.
Reversible?: yes. 50-iteration checkpoints are kept, so a shorter budget is a re-selection and a longer one is a continuation.
Grader docs: [grader-1](2026-11-18-w4-finetune-budget/grader-1.md) · [grader-2](2026-11-18-w4-finetune-budget/grader-2.md)

## Conclusion
**1500 iterations** from the Part 2 final checkpoint (P → P+1500): Bumpy, Lab 4 DR, the same reference, `splay_stand_still.json`, 4096 envs, seed 42.
**Evaluate at +500, +1000 and +1500** and ship the **+1500** checkpoint unless a guard fires. The guards are fixed before launch, and the intermediate evals are diagnostics, not a menu.
`part3_finetune.py` now has `FULL_ITERATIONS = 1500`.

Guards (shared by both graders unless marked):
- **Invalidate / stop early:** any nonzero physics-overflow bit, NaN or divergence. G2 adds a stop if, at +500, flat_dr falls are ≥ 2× the +0 value and bumpy_dr is no better than +0.
- **Fall back to +1000:**
  - G1: only if flat_dr falls or the flat_dr seam verdict regress beyond +1000 by more than 10 falls.
  - G2: if +1500 is worse than +1000 by more than 10 falls on bumpy_dr **or** flat_dr, or the flat_dr seam verdict flips to FAIL.
  - G2's trigger contains G1's, and the runbook implements G2's. G1 also asks to confirm on a second eval seed and to label the result a selection, and that is implemented too.
- **Extend** by +500 (hard cap +3000) only if bumpy_dr falls drop by more than 10 from +1000 to +1500 while flat_dr stays within 10 falls of +0.

## Evidence used (from the grader docs)
- The only Bumpy fine-tune precedent is the [optional2 checked-Bumpy continuation](../../configs/optional2/README.md) (1500 iterations, 0.10 m grid). Failures over 3 seeds went from 101 to 81 and xy error from ~0.093 to ~0.085 m/s. It was not promoted because standing torque, slip, yaw and some per-command failures got worse.
- The smoke run (`rp-p3-parent-bumpyft-smoke`, W&B `c3rfre1p`) took 4.3–4.7 s/iter while the GPU was shared with 4 Part 2 trainers. Bumpy is about 1.1–1.2× Flat per iteration. Training stayed stable through the end (ep length ~470, err_xy ~0.23), and there was zero overflow.
- [REPRO_PLAN](../REPRO_PLAN.md) §3–4 and [num-envs](2026-11-18-infra-num-envs.md): fixed budgets at 4096 envs, and aggregate throughput is flat at ~1.25 it/s, so 1500 iterations is ~20 GPU-minutes of the shared pie.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| 500 | Shorter than the only precedent and the same as a Part 2 leg. The +50 smoke still showed a transient. |
| 1000 | Has no precedent. The extra 500 iterations cost ~8 min, and +1000 stays available through the fallback. |
| 3000 | That is a from-scratch default. It spends longer forgetting flat ground and standing, and it is still reachable through the extension rule. |
| 8192 envs with fewer iterations | Barred by the num-envs decision. The smoke shows no sample starvation. |

## Severe flagged issues (union)
- **No control arm (REPRO_PLAN §3).** Without a matched +1500 **flat continuation** of the same Part 2 final, a robustness gain can't be credited to Bumpy + DR rather than to training longer (G1, G2). → The runbook now trains and evaluates this control by default (`control` stage; ~0.4 GPU-hours).
- **The smoke is a confounded proxy.** It used the Part 1 parent, switched reward weights (splay → splay_stand_still), ran only 50 iterations, had an incomplete eval when graded, and changed the seam harness. Don't read its regressions (flat_dr 40→63 falls, bumpy_dr 81→91) as the expected outcome, but expect a transient (G1, G2).
- **One training seed and one eval seed.** Selecting among checkpoints on the same eval seed is optimistic. Confirm the shipped checkpoint on a second eval seed and report any fallback as a selection (G1, G2).
- **Check the precedent's side costs directly:** standing torque, slip, yaw and per-command failures. Don't rank on aggregate falls or reward alone (G2).
- **Concurrency:** the smoke overlapped 4 Part 2 trainers (5 GPU processes). The runbook waits for fewer than 4 before each launch, but other agents can start jobs after it does (G1).

## What would flip this decision
- At +500, bumpy_dr is already at its best (≥ 10 falls better than +0) and flat by +1000 → ship +1000 or +500.
- A solo measurement shows Bumpy costs ≥ 2× flat per iteration → 1000 with the same guards.
- The matched flat control shows the same gains → the Bumpy budget is not what drives them; revisit.
- Training is visibly sample-starved → separate num-envs decision (infra flip condition).
- The Part 2 schedule slips and W4 becomes the critical path → 1000 with the same guards.
