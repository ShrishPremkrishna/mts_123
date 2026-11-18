# `--num-envs` and iteration budget for all remaining training (Parts 1–3) on the 1× A100-40GB?
Status: DECIDED by grader (pending human review) · 2026-11-18 · Requested by: infra (W0)
Criticality: High. It sets the PPO numerics of every remaining run (Part 1 parent, Part 2 matched table, Part 3 final policy), and redoing them costs > 3 GPU-hours.
Reversible?: Partly. Part 1 started at 4096 envs minutes ago, so switching now would waste only minutes. A switch partway through a Part 2 lineage cannot be cleanly reversed, because it mixes batch sizes inside one matched comparison. Redoing Part 2 costs ≈ 3 GPU-hours.

## Conclusion
**Keep the recovered recipe everywhere: `--num-envs 4096`, `num_steps_per_env 24`, and unchanged iteration budgets.**
- Part 1: every arm runs 3000 iterations at 4096 envs. This covers the queue that is already running (gallop_c/b/a, fast_trot, baseline at s42; leave it alone) and the s43 repeats of the top 2.
- Part 2: every leg runs 500 iterations at 4096 envs. Candidates, controls and continuations are matched at +500/+1000/+1500 at s42 and s43.
- Part 3: every training or fine-tuning run (MixedGaits sweep, Bumpy, DR pass) uses 4096 envs and starts from the 4096-env Part 2 final.

If time runs short, **cut arms or seeds, not iterations** (§3). Do not adopt 8192 envs in any form for this reproduction.

## Evidence used
- `infra/a100_throughput.md`: at 4096 envs the GPU delivers ≈ 1.27 it/s in aggregate, flat for N = 1–6 (127 k env-steps/s). One 8192-env process ran at 0.90 s/iter ≈ 218 k env-steps/s (1.7×), but from only n = 17 iterations, solo, before a foreign process arrived. Aggregate throughput of *concurrent* 8192 runs was never measured.
- `rl_cfg.py` (pupper): 5 epochs × 4 mini-batches and adaptive lr (desired_kl 0.01). At 4096 envs a mini-batch is 24 576 samples; at 8192 it is 49 152. Same 20 gradient steps per iteration, but the KL-adaptive lr and gradient noise change.
- `pupper_gait_env_cfg.py`: curricula (yaw stages, pretrain_rewards until step 12000 = 500 iterations) are indexed by `common_step_counter` = iterations × 24. Rescaling iterations would move every curriculum stage relative to training progress.
- `C_lineage_metrics.md` §3c/§3d: every recovered target is at 4096 envs and equal iterations. Examples: gallop_c-s42-r2 reward 151.8 / err_xy 0.251 at 2998; Part 2 gains over control are small (err_xy 0.233 vs 0.241 at +500, 0.219 vs 0.232 at +1500). baseline at 2999 vs cont4500 shows Part 1 arms are near plateau by 3000.
- Running queue (`part1_queue.py`, 3 trainers at 3.7 GB each): Part 1 at 4096 started ≈ 11:11 UTC. Remaining workload: Part 1 ≈ 21 k iterations ≈ 4.6 h, Part 2 ≈ 14 k ≈ 3.1 h.

## Thought process
Criteria, in priority order:
1. **Validity of the Part 2 claim.** The headline finding to reproduce or refute is an err_xy gap of about 0.01, close to the seed noise floor (§3: "treat ~10 % one-seed gaps as noise"). Any change in PPO numerics inside or across matched arms is a confound of the same size as the effect, and the recovered table is the only external yardstick. Iteration matching with an unchanged recipe (§3) keeps both internal and external comparability.
2. **Lineage consistency.** Part 2 warm-starts from the Part 1 parent, which is already training at 4096, and Part 3 warm-starts from Part 2. Switching env count partway through changes the batch, mini-batch and adaptive-lr trajectory of a resumed optimizer. 4096 is the only value that stays consistent from end to end.
3. **Wall-clock.** The real gain is small. At equal iterations, 8192 is *slower* (0.90 vs 0.78 s/iter solo). It only pays off with halved iterations (Part 2 ≈ 7 k × 0.90 ≈ 1.75 h vs 3.1 h, saving ≈ 1.4 h). That halves the number of PPO updates per leg (500 → 250) and breaks the recovered +500 grid. To justify it we would also need a 1500-vs-3000 equivalence check (≈ 0.5 GPU-h, one seed, inconclusive at the 0.01 level). Part 1 cannot be sped up without killing it.
4. **Statistical power.** Seeds, not batch size, are the binding constraint. The flat 1.27 it/s budget already affords s42 + s43 for every Part 2 arm, which removes last time's one-seed caveat. Larger batches don't add independent seeds.
5. **Sim-to-real.** No evidence that 8192 envs yields a better or worse hardware policy. The recovered 4096 policies are the known-good deployment reference.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| 8192+ envs, same iterations | Doubles env-steps but is slower per iteration (0.90 vs 0.78 s). No wall-clock gain, and it breaks comparability and lineage consistency |
| 8192 envs, halved iterations (env-step matched) | Saves ≈ 1.4 h on Part 2 only. Halves PPO updates, shifts the iteration-indexed curricula, breaks the +500/+1000/+1500 grid against recovered numbers, and needs its own validation run |
| Mixed by part (4096 for Part 1, 8192 for Part 2/3) | Worst of both: switches numerics at the warm-start boundary, exactly where the matched comparison happens |
| Kill Part 1 and restart at 8192 | Throws away queue progress for an unvalidated recipe, and the Part 1 numbers would no longer match the recovered 3000-iteration targets |
| 4096 envs, fewer iterations to save time | Violates "cut arms, not iterations" (§3) |

## Severe flagged issues
- The 8192 throughput figure rests on 17 solo iterations. If anyone revisits this, the concurrent-8192 aggregate must be measured first, and nobody has done that.
- Whatever the recipe, a single run of mixed env counts within one Part 2 matched set (candidate vs control, s42 vs s43) would invalidate that comparison. The launch wrapper should record `num_envs` and the analysis should assert that it matches.
- The Part 1 queue runs at `--max-concurrent 3`, not the cap of 4. Throughput is unaffected (it's flat), so this is fine.

## What would flip this decision
- The remaining schedule cannot fit before the deadline even after cutting arms and seeds. Then use 8192 envs with halved iterations **only for new, self-contained lineages** (e.g. a Part 3 from-scratch side study), with its own 4096 control. Never partway through a lineage.
- A measured concurrent-8192 aggregate of ≥ 2× the 4096 rate, together with a 2-seed env-step-matched test (8192 × 1500 vs 4096 × 3000) showing indistinguishable sim-to-real metrics. Then re-decide for runs not yet started.
- Evidence that 4096-env policies fail on hardware in a way that larger batches plausibly fix (e.g. jitter tied to gradient noise).
