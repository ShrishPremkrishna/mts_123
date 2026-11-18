# How many PPO iterations should the Part 3 Bumpy + Lab 4 DR fine-tune run?
Status: grader-1 recommendation (pending agreement check / human review) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: Medium. It sets the cost of the one Part 3 training run, and nothing is matched against it.
Reversible?: Yes. Checkpoints are saved every 50 iterations, so a shorter budget can be read off the same run and a longer one resumed (+500 iterations ≈ 7 GPU-min of the shared pie). Restarting from scratch costs ≤ ~0.5 GPU-h.

## Conclusion
Run **1500 iterations** (Part 2 final ckpt → +1500), 4096 envs, seed 42, `save_interval=50`. Fix the budget and the shipping rule before launch:
- **Evaluate at +500, +1000 and +1500**: the full part3_eval suite + seam + push on flat_dr / flat_nodr / bumpy_dr / bumpy_nodr, eval seed 1234, also scored against the warm-start parent.
- **Ship the +1500 checkpoint** unless a guard below fires. The +500 and +1000 evals are diagnostics, not a menu to pick the best seed-1234 number from.

## Evidence used
- **Old precedent.** `configs/optional2/README.md` and `foundation/results/optional2-checked-bumpy-review.json` cover the only earlier Bumpy continuation, which ran **1500 iterations** on the checked 0.10 m grid with 0 overflow. Failures went 36/29/36 → 25/19/37 over three eval seeds (101 → 81 total), and mean xy error went ~0.093 → ~0.085 m/s. Gains were real but not uniform (seed 404 got +1, standing costs got worse).
- **Smoke run** (`rp-p3-parent-bumpyft-smoke`, Part 1 parent, +50 iterations; parent → it3048, eval seed 1234):
  - Falls got worse everywhere: flat_dr 40 → 63, bumpy_dr 81 → 91, flat_nodr 100 → 114. Most of the new falls are in "stop" (flat_dr 37 → 61, bumpy_dr 37 → 55). Steady-gait falls on Bumpy went down (34 → 25).
  - The bumpy_dr seam verdict went from PASS to **FAIL** (sweep_fwd_down incidents 12/191).
  - Training reached err_xy 0.228 and fell_over 0.79 at it 3048. The training overflow audit is clean.
  - Reading: 50 iterations sits inside a transient disruption. The budget has to be long enough to get past it.
- **Smoke throughput.** The smoke averaged **4.36 s/iter** (collection 4.0 s, learning 0.34 s) while sharing the GPU with 4 Part 2 trainers. Concurrent flat Part 2 legs ran at 2.2–4.5 s/iter at the same moment, so Bumpy (0.10 m grid) is not noticeably dearer per iteration.
  - REPRO_PLAN §4.3: aggregate ≈ 1.25 it/s. 1500 iterations ≈ **20 GPU-min of the pie**, ≈ 1.5–1.8 h wall at 4–5-way sharing.
  - Each full eval took ~9–10 min wall (part3 runbook: eval stage 564 s).
- **Part 2 learning curves** (`C_lineage_metrics.md` §3d, warm-start legs of 500): control err_xy 0.241 / 0.237 / 0.232 at +500 / +1000 / +1500, and the continuations keep improving to +1500. Gains of ~0.005 sit within noise. +500 alone is clearly not converged.
- **Lab 4 guidance** (notebook, `lab4_runs/shrish_stablegait`): 3000 iterations is the *from-scratch* budget ("walking usually shows up well before this"). It also says to judge a run by its tracking errors and to cut it at ~1000 if something is being farmed.

## Thought process
1. **Enough iterations to adapt.** The smoke run shows a real regression at +50. The Part 2 and optional2 evidence shows warm-start gains still accruing at +1000–1500. That rules out 500.
2. **Ground it in precedent, not invention.** 1500 is the one recovered Bumpy-continuation budget that produced measurable robustness gains. It also matches the Part 2 horizon (+1500), so Part 3 follows the same warm-start budget as Part 2.
3. **Cost is not the binding constraint.** The options cost 7 / 13 / 20 / 40 GPU-min for 500 / 1000 / 1500 / 3000 iterations. Part 2 has GPU priority, so the run would queue; wall-clock and forgetting risk matter more than GPU time.
4. **Fixed budget, iteration-matched (§3).** Nothing is compared against this run, but the shipping checkpoint is still fixed in advance so that choosing an iteration count after seeing the results doesn't become selection on one eval seed.

Guards (decide these before launch):
- **Stop or invalidate**:
  - Any nonzero `physics_overflow_audit` invalidates the run.
  - If at both +500 and +1000 bumpy_dr falls are no better than the parent's **and** flat_dr falls are > parent + 10, stop. Ship the Part 2 parent with a "Bumpy pass did not help" result (report it, §3).
- **Fall back to +1000**: allowed only if, at +1500, flat_dr falls or the flat_dr seam verdict regresses beyond the +1000 eval by more than the 10-fall noise band. Then confirm on a second eval seed and label the result as selected.
- **Extend once, +500 → 2000 (hard cap 3000)**: only if bumpy_dr falls drop by > 10 from +1000 to +1500 while flat_dr is within noise of the parent (the "visibly sample-starved" flip in the num-envs decision). Record the extension in the run name/notes.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| 500 | The +50 smoke is still regressing; Part 2 warm-start legs are not converged at +500. Too short to clear the transient. |
| 1000 | Defensible and ~7 GPU-min cheaper, but it has no recovered Bumpy precedent. The plan still evaluates +1000, so a 1000-iteration fallback stays available. |
| 3000 | This is the from-scratch Lab 4/Part 1 budget, and the warm start doesn't need it. It doubles wall-clock behind Part 2 and raises the risk of forgetting the flat/seam skill. Available only via the extension rule. |
| Run until eval stops improving | Breaks the fixed-budget principle and turns eval seed 1234 into a selection set. |

## Severe flagged issues
- **No control arm (§3 "always run a control").** Without a flat continuation of the same Part 2 checkpoint at matched +1500, any gain can't be split between the Bumpy terrain and simply training longer. Run one if a slot frees up (~20 GPU-min); otherwise report this as a limitation.
- **The smoke and parent evals use the Part 1 parent, not the Part 2 final policy.** The +50 regression (flat_dr +23 falls, bumpy_dr seam PASS → FAIL) may play out differently on the real warm start. The +500 eval is the first real evidence.
- **One training seed and one eval seed (1234).** Fall deltas under ~10 are noise. Confirm the shipped checkpoint on at least one more eval seed, as the optional2 precedent did with three.
- **The smoke eval was incomplete when I read it.** bumpy_nodr and push were still being written; flat_dr, flat_nodr, bumpy_dr and their seams were present.
- **Possible concurrency overrun.** The smoke (18:33–18:37) overlapped 4 Part 2 trainers, which looks like 5 trainers against the cap of 4. That explains its 4.4 s/iter. Make sure the full run waits for a slot.

## What would flip this decision
- At +500 on the real warm start, bumpy_dr falls already ≤ the parent's −10 and are flat by +1000. Then 1000 would have been enough; ship +1000 only if +1500 also regresses on flat.
- Bumpy turns out to cost ≥ 2× flat per iteration in a solo measurement. Then 1000 would be the better trade.
- A matched flat control shows the same gains as the Bumpy run. Then the Bumpy budget isn't what's driving the gains; revisit.
- Part 2 schedule slips so that W4 wall-clock becomes the critical path. Then use 1000 and keep the same guards.
