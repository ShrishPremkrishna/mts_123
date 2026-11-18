# Devil's-advocate review: W4 fine-tune budget (1500 iterations)
Reviewer: red-team agent · 2026-11-18 · Decision not yet executed (runbook stages `finetune`/`control` have not run for `p3-final`).

**Verdict: STANDS WITH NEW CAVEAT.** Named caveat: *the guards break iteration matching with the control arm, and the "stop early" guard cannot fire early.* The 1500 number is fine, but the plumbing around it has to be fixed before launch.

## Strongest counter-case
1. **The fallback and extension rules break the matched-iteration rule (REPRO_PLAN §3, user rule c).**
   - In `scripts/opt_lab_2/part3_run.py`, `eval-ft` evaluates the control **only at its final checkpoint** (lines 493–496).
   - `select` can still ship Bumpy **+1000** (lines 167–176). The selection table would then print Bumpy +1000 next to a row labelled "flat control +1500 (matched)" (line 514).
   - The extension rule continues **only the Bumpy arm** to +2000 as "a new run" (notebook §7). That leaves Bumpy +2000 against control +1500.
   - Both of these outcomes are likely:
     - Part 2's control err_xy was still falling at +1500 (0.241 / 0.237 / 0.232 at +500 / +1000 / +1500), and so were all arms (`recovery/C_lineage_metrics.md` §3d). So "still improving at +1500" is the expected case, which is exactly what triggers the extension.
     - The fallback is close to a coin flip under the null hypothesis (point 3).
2. **The "stop early" guard is not early.**
   - The stage order is `finetune` → `control` → `eval-ft` → `select`, one GPU process at a time. The +500 G2 check (line 163) only runs after **both** 1500-iteration arms have finished.
   - So it saves no compute. The decision lists it under "Invalidate / stop early".
3. **One eval seed makes the fallback a near-coin-flip.**
   - It switches to +1000 if any of flat_dr, bumpy_dr or nompd (nompd was added by kp-coverage) is more than 10 falls worse, or the seam verdict flips.
   - Same-checkpoint differences between seeds are already at or above 10:
     - parent nompd 99 vs 79;
     - smoke bumpy_dr 91 vs 101;
     - flat_dr 63 vs 62 (`p3-parent_kp_coverage.md`, `p3-parent-bumpyft-smoke-it3048-seed4321/summary.md`).
   - With roughly 0.2 per metric across three metrics, about half of null outcomes would "fall back". At that rate "ship +1500 unless a guard fires" is really best-of-two on seed 1234. The second seed only confirms the pick; it doesn't decide it.
4. **The cost is understated.**
   - The header says "the one Part 3 training run (0.4–0.75 GPU-h) and nothing else is matched against it". The same doc then adds a matched control, so the real figure is at least 2 × 1500 iterations plus nompd evals at every point.
   - Measured speeds: Bumpy smoke 4.36 s/iter mean (5 GPU processes); flat-control smoke 3.40; Part 2 legs at 4-way 3.3–3.5; Part 1 at 3-way 2.6–2.8. Solo is 0.78–0.9.
   - Aggregate throughput is ~1.1–1.2 it/s, not 1.25.
   - Wall clock at realistic 4-way sharing:
     - training ≈ 1500×3.9 + 1500×3.4 s ≈ **3.0 h**;
     - pre-stages measured ≈ 50 min (`p3-parent-runbook*.log`: export 452 s, eval 564 s, video 1183 s, kpcheck 303 s, smoke-wide 516 s);
     - post-evals ≈ 45–75 min (a single +50 pair took 669–1494 s).
     - Total ≈ **4.5–5 h end to end**, not the notebook's 2.5–3.5 h.
   - If KEEP → HOLD → WIDEN escalates, add another ~4 h.
   - Solo it is ≈ 45 min of training and ~2.5 h end to end.
   - "The extra 500 iterations cost ~8 min" is in pie-minutes. In wall clock it is ~1 h for both arms at 4-way. That still doesn't argue for 1000, but the stated trade-off is off by about 7×.
5. **The precedent and the smoke are weak anchors.**
   - optional2's 1500 was that project's default continuation length (every optional2 leg is 1500). Its gain was 101 → 81 falls over 3 seeds (~7 per seed, inside the noise band). It had no intermediate evals and was not promoted.
   - The smoke's "transient" used to reject 500 is shared by the flat control (flat_dr 63 vs 63) and is mostly the splay → splay_stand_still weight switch. The Part 2 final already trains with that switch, so it won't happen in the real run.
   - Of the smoke's 50 iterations, ~20 are episode-buffer fill (ep_len 20 → 480 by iteration 3020), so it says nothing about where convergence lies.

## What the graders missed or under-weighted
- **The WIDEN branch is likely.** The parent's Δ is 48.5, against a threshold of 15. That makes this a different fine-tune from the precedent: kp ×[0.6, 1.3] plus terrain.
  - The WIDEN smoke has a deeper transient than the Lab-4 smoke at the same point (reward 142 vs 162, fell_over 1.29 vs 0.79).
  - The extension rule looks only at bumpy_dr. It ignores nompd, the metric that motivates WIDEN.
- The interaction with kp-coverage (nompd entering the fallback) postdates the graders. It makes point 3 worse.
- G1's "ship the Part 2 parent if no gain at +500 and +1000" stop rule is not implemented. Only G2's flag is.

## Flip conditions
None is met:
- +500 needs the run;
- there is no solo Bumpy:flat measurement (concurrent ratio 1.05–1.3×, not ≥ 2×);
- the matched control at +50 shows the same transient but no gains yet;
- there are no signs of sample starvation;
- Part 2 training is finishing now (the last s43 legs were at ~4450–4480/4497 at 20:43), so W4 is not on the critical path.

The "→ 1000 if the schedule slips" flip cuts iterations rather than arms, which is the opposite of user rule (c).

## Cost of reopening
About one re-decision, with no GPU time lost, since nothing has run. The fixes below keep 1500. That means no change to:
- `FULL_ITERATIONS`;
- the kp-coverage decision, which assumes 1500 for both arms;
- the frozen seam thresholds.

## Bottom line: concrete amended budget (same 1500 per arm)
1. Train the **two arms concurrently** (2 processes; aggregate throughput is flat, so this costs the same total). Then the matched +500 / +1000 points arrive together.
2. Evaluate the **control at +500 / +1000 / +1500 with the same light suite + nompd as Bumpy**, about 20–30 min more. Any fallback is then still matched.
3. Run the +500 G2 check **as soon as both +500 checkpoints exist**, while training continues.
4. **Drop the Bumpy-only extension.** Either extend both arms in lockstep or report "still improving at +1500" as a result.
5. A fallback has to **replicate on the pooled seeds 1234 + 4321** before it switches the shipped checkpoint.
