# `--num-envs` and iteration budget for all remaining Opt Lab 2 training on the 1× A100-40GB?
Status: DECIDED by grader (pending human review) · 2026-11-18 · Requested by: infra (W0)
Criticality: High. It sets the PPO numerics for every Part 1/2/3 run and gates the Part 2 matched table and the Part 3 deploy policy. Redoing them would cost > 3 GPU-hours.
Reversible?: Only for runs that have not started. Switching mid-part breaks iteration-matched sets: all of Part 1 (≈ 21 k iterations, ≈ 4.6 h) or a whole Part 2 lineage (≈ 14 k iterations, ≈ 3 h) would need re-running.

## Conclusion
**Keep the recovered recipe for every remaining run: `--num-envs 4096` with unrescaled iterations.** That means 3000 for each Part 1 arm (the 5-job s42 queue that is running now, plus s43 for the top 2), 500 per Part 2 leg (candidates, control and continuations to +1500, at s42 and s43), and 4096 for the Part 3 MixedGaits/Bumpy run, at whatever iteration budget W4 sets. If the schedule slips, **cut arms, not iterations or batch** (§3). Do not use 8192+ in any run that feeds a comparison or the deployed policy.

## Evidence used
- `infra/a100_throughput.md`: at 4096 envs, aggregate throughput is flat at ≈ 1.25–1.3 it/s for 1–6 processes (127 k env-steps/s solo). At 8192, one process runs 0.90 s/iter ≈ 218 k env-steps/s (1.7×) and uses 6.7 GB. That figure is **one solo process, n = 17 iterations**. 8192 under concurrency was never measured.
- `pupper/rl_cfg.py` (and `A_code_recovery.md`): 24 steps/env, 5 epochs × 4 mini-batches, adaptive lr with desired_kl 0.01. At 4096 a batch is 98 k samples (mini-batch 24.6 k). At 8192 it is 197 k (mini-batch 49 k). In both cases there are 20 gradient steps per iteration, so 8192 with halved iterations gives **half the optimizer steps** for the same env-steps.
- `C_lineage_metrics.md`: every recovered run used 4096. The §3c (3000 iterations) and §3d (+500/+1000/+1500 matched Part 2) tables are the reproduction targets. Gaps between arms there are small: err_xy 0.218–0.237 at +1500, and continuations are within noise of the r2/r3 stages. A batch-size confound could easily swamp them.
- Running queue (`part1_queue.py`, started minutes ago): p1-gallop_{c,b,a}, fast_trot, baseline at s42, each `--num-envs 4096 --iterations 3000`.
- REPRO_PLAN §4.3 budget: Part 1 ≈ 24 k iterations ≈ 5 h, Part 2 ≈ 14 k ≈ 3 h. Lab spec: Part 3 needs one full-MixedGaits training run. The spec states no iteration or env count.

## Thought process
Priorities, in order:
1. **Comparability with the recovered numbers** (§3: fixed budgets, compare at equal iterations, "use the recovered metrics as targets", §4.6). The reproduction asks whether bootstrapping beats an iteration-matched control as it did before. That claim lives in a 4096-env table.
2. **Internal comparability.** Part 1 is already running at 4096, so its s43 runs must match. Part 2 warm-starts from a 4096 parent, and the checkpoints carry Adam moments and an adaptive lr fitted to 24.6 k mini-batches.
3. **Sim-to-real.** The Part 3 policy goes to hardware. Ship it from the validated recipe, not from one that has 17 iterations of evidence.
4. **Wall-clock.** Remaining work at 4096 ≈ 21 k (Part 1) + 14 k (Part 2) + ~3 k (Part 3) ≈ 38 k iterations ≈ 8.5 h plus evals. Moving Part 2 and Part 3 to 8192 with halved iterations would save only ≈ 1.5 h (≈ 18 %). Even that rests on extrapolating the solo 0.90 s/iter to 4 concurrent processes. The saving is real but small next to the cost: an unrecoverable confound in the headline comparison.
5. **Statistical power.** 4096 already affords 2 seeds for Part 2 (≈ 3 h). If a 3rd seed is wanted, pay for it by dropping low-value arms (e.g. a redundant continuation), not by changing the batch.

"Mix by part" was the strongest alternative: 4096 for Part 1, 8192 for Part 2/3. It fails on (1) and (2). Part 2 is exactly where matched-control gaps of ~0.01 err_xy are the result. Part 3 saves ≈ 20 min on one run and would ship to hardware a policy trained with a different recipe from the rest of its lineage.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| 8192+ envs, same iterations | ≈ 1.15× wall-clock per run (0.90 vs 0.78 s/iter) and 2× env-steps, so it is slower overall. Breaks comparability with every recovered number. Concurrency at 8192 is unbenchmarked |
| 8192, halved iterations (env-step matched) | Half the gradient steps, a different mini-batch, and Part 2 legs of 250 iterations that cannot be laid against the recovered +500/+1000/+1500 table. Saves ≈ 1.5 h at most |
| Restart Part 1 at 8192 | Throws away a running queue and breaks the match with the recovered §3c table and with s43. No benefit that outweighs this |
| Mix: Part 1 at 4096, Part 2/3 at 8192 | Puts a batch confound at the parent→leg boundary and into the deployed policy (see Thought process) |

## Severe flagged issues
- The budget is ≈ 8.5 h of GPU time plus evals, and only if the max-4 cap and eval batching hold. Co-scheduled evals cost a trainer up to 27 %. If the schedule cannot absorb this, cut arms (e.g. Part 1 s43 only for the top 1, or fewer continuations), never iterations within a matched set.
- Any 8192 run that does get launched (e.g. a side study) must never enter a table with 4096 runs, and must be labelled in its WandB config and run name.
- Part 2 matched sets (candidate + control × s42/s43) must start from the same parent checkpoint with identical `--num-envs 4096` and `--iterations`. The launch wrapper should log `num_envs` in the run config.

## What would flip this decision
- A matched A/B (e.g. baseline at 8192 × 1500 vs 4096 × 3000, both seeds) that shows no difference on the sim-to-real eval readouts (falls, peak foot force, drift, err_xy within the ~10 % seed spread), **plus** an 8192 concurrency benchmark showing ≥ 1.5× aggregate env-steps/s. Then 8192 could be adopted for **new, self-contained** lineages, such as a fresh Part 2 lineage or Part 4.
- A hard deadline that cannot be met with 4096 even after cutting arms. Then the first change should be Part 3 alone (a single run with no recovered comparator), not Part 2.
- Part 1 crashing or needing a full restart before Part 2 begins removes the sunk-cost argument, but not the comparability one.
