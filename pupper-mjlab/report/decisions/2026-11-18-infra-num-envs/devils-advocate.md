# Devil's-advocate review: keep `--num-envs 4096` (infra-num-envs, 3/3 High)
Reviewer: red-team, retroactive · 2026-11-18 ~20:45 UTC · CPU-only analysis of `~/runs/opt-lab-2/rp-*.log` + `*.launch.json` (no GPU jobs).

**Verdict: STANDS WITH NEW CAVEAT.** Budget at the *measured production* rate (≈ 1.0–1.1 it/s for the whole GPU, about 0.95 GPU-s per 4096 iteration), not the 1.27 it/s benchmark. If 8192 is ever weighed again (Part 4 or another new lineage), price "8192 at equal iterations" at **+0 to +15 % GPU-time for 2× samples per update**, not as a plain loss.

## Verified numbers (per-run median of "Iteration time"; wall = launch→log mtime ÷ iterations)
| Window (UTC) | Concurrent trainers | per-run median s/iter | wall s/iter | aggregate it/s (wall) |
|---|---|---|---|---|
| 11:12–13:40 gallop_{a,b,c}-s42 | 3 | 2.61 / 2.61 / 2.61 | 2.96 | 1.01 |
| 13:40–16:10 fast_trot-s42, baseline-s42, gallop_c-s43 | 3 | 2.60 / 2.60 / 2.61 | 2.77–2.84 | 1.06 |
| 16:24–17:13 fast_trot-s43 | **1** (evals likely present) | **0.86** | 0.97 | 1.03 |
| 17:32–20:18 Part 2, 21 legs × 500 | 4 (5 during P3 smokes) | 3.38–3.52 (2.85–2.89 for the 2 s43 +1500 legs still running) | 3.4–3.9 | 1.09 (r1 window); 0.96 whole span |

All 7 Part 1 runs and 21 Part 2 legs launched with `--num-envs 4096` (launch.json). All 37 recovered runs in `~/recovery/runs_meta` are 4096 too.
Medians imply 1.15–1.16 it/s; wall-clock gives 1.01–1.09 it/s. Benchmark (`infra/a100_throughput.md`): 1.29 (N=1), 1.22 (N=3), 1.23 (N=4).

## Strongest honest counter-case
1. **4096 is context-bound, not compute-bound.** One 8192 process ran 218 k env-steps/s. That is 1.7× the *whole GPU* at 4096 under any concurrency, and about 2× the ~105–115 k env-steps/s production actually got. Every 4096 run therefore used roughly half the sample throughput the card can give.
2. **The graders priced 8192 solo vs 4096 solo** (0.90 vs 0.78). Under a flat pie the right 4096 figure is 1/aggregate. That is 0.79 s on the benchmark, so the +14 % holds on clean numbers. In production it was ~0.92–0.99 s. If 8192 suffered no worse drag from evals and startup, equal-iteration 8192 run serially would have cost about the same wall-clock for 2× the batch. That stays unproven: 17 clean iterations, never measured under contention.
3. **The schedule premise was optimistic.** "≈ 8.5 GPU-h" assumed 1.27 it/s. Part 1 took 6.0 h wall for 21 k iterations (0.97 it/s overall), against ~4.6 h implied.
4. **2048 per run, or more runs:** no gain. Production shows N = 1, 3 and 4 all at ~1.0–1.1 it/s, so smaller batches would only halve samples at the same it/s. No env-count sweep exists below 8192: the table sweeps N at 4096 only, plus one solo 8192 point.

**Why it still fails:** under the user's fixed-iteration rule, the only option that saves wall-clock is 8192 with halved iterations. That is "cutting iterations". It halves gradient steps (20 per iteration either way) and moves the iteration-indexed curricula (500-iteration pretrain, yaw stages). Equal-iteration 8192 saves no time at best and changes the batch, mini-batch (24.6 k → 49 k) and adaptive-lr path. That confounds the ~0.01 err_xy Part 2 effects, and §7 "did the recovered findings reproduce" is still unwritten and depends on the 4096 table. The new timing fact moves the cost-benefit by a few percent. It does not touch the comparability criterion that decided the case.

## What the graders missed or under-weighted
- They never checked their own throughput premise against running jobs. The first Part 1 hour (2.6 s/iter at N=3, versus 2.46 benchmarked) already showed a ~6 % median drag and ~17 % on wall, mostly evals, startup and foreign contexts.
- "Learning per iteration at 8192 is unknown" (G3) was the only real upside, and nobody gave it a price. Part 1 arms plateau by 3000 (baseline vs cont4500), so the upside is probably small for Part 1. It is untested for Bumpy/DR.
- The concurrent-8192 benchmark was "required before revisiting" but was never scheduled, so flip condition 1 was not testable.

## Flip conditions status
1. Matched 8192×1500 vs 4096×3000 calibration **plus** concurrent-8192 ≥ 1.5–2×: **not met**, never run. Production data suggests concurrency adds nothing at any env count, so "≥ 1.5× concurrent" is likely unreachable. Only solo 8192 (1.7×, 17 iterations) exists.
2. Hard deadline: **not met.** No deadline in REPRO_PLAN, and Part 1 and Part 2 training is essentially done.
3. Part 3 sample-starved: **not met.** `w4-finetune-budget` reports the smoke stable (ep len ~470, err_xy ~0.23). The current HOLD (nompd − flat_dr = +43.5) is a robustness and seam gap from a 50-iteration smoke, not evidence of batch starvation. Don't let this be re-argued as one without a DR-variance diagnostic.

## Cost of reopening now
All 7 Part 1 runs (21 k iterations, ~6 h) and 21 Part 2 legs (10.45 k iterations, ~3 h) are at 4096. Comparability with them or with the recovered table would mean re-running ~9 GPU-h at observed rates. The only run left is the Part 3 fine-tune (1500 iterations ≈ 25–30 min at 4096). Halving it at 8192 saves ≈ 10 min, swaps the deployed policy's recipe, and voids the 4096-tuned Adam/KL state it warm-starts from. Benefit ≈ 0, cost high.

## Factual nits in the decision doc
- "~8.5 GPU-hours" and the 1.4–1.5 h saving both use the 1.27 it/s benchmark. Observed ≈ 0.97–1.09 it/s, so absolute times run ~20–30 % low. The ratios are unchanged.
- "Part 1 arms are single-seed (s42)" is now superseded: gallop_c and fast_trot have s43.
- (REPRO_PLAN §4 item 4, not the decision doc, says "Use concurrency for throughput", which contradicts its own item 3.)

## Side remark on W0 concurrency
The new fact **confirms** "throughput is flat": near-solo fast_trot-s43 (1.03 it/s wall) ≈ 3-way (1.01–1.06) ≈ 4-way (1.09). The gap to the benchmark's 1.25 is eval, foreign-context and startup drag, not a concurrency penalty. The cap of 4 is harmless, but it buys no throughput.
