# Which `--num-envs` and iteration budget should the remaining training use?
Status: DECIDED by graders (3/3 agree; pending human review) · 2026-11-18 · Requested by: manager (from W0's throughput finding)
Criticality: High. It applies to every remaining training run (about 8.5 GPU-hours) and to Part 2's matched comparisons.
Reversible?: Yes for runs not yet started. A switch mid-lineage would cost the matched Part 2 comparison.

## Conclusion
Keep `--num-envs 4096` and the recovered iteration budgets for all remaining runs:
- Part 1 arms: 3000 iterations. The running s42 queue finishes untouched, and the s43 repeats of the top 2 use the same recipe.
- Part 2 legs: 500 iterations, plus controls and continuations matched at +500/+1000/+1500, on seeds 42 and 43.
- Part 3: 4096 envs.

If time runs short, cut arms or seeds, never iterations or batch size.

Grader docs: [grader-1](2026-11-18-infra-num-envs/grader-1.md) · [grader-2](2026-11-18-infra-num-envs/grader-2.md) · [grader-3](2026-11-18-infra-num-envs/grader-3.md)

## Shared reasoning (all three)
- At the same iteration count, 8192 envs saves **no** wall-clock: 0.90 vs 0.78 s/iter solo. It only saves time if iterations are cut, which breaks "cut arms, not iterations".
- Cutting iterations halves the PPO updates per leg and shifts the curricula, which count in iterations (e.g. the 500-iteration pretraining phase).
- The Part 2 effects under test are ~0.01 in velocity error, close to seed noise, so a batch-size change would confound them.
- The best case saves about 1.4–1.5 of ~8.5 GPU-hours. That isn't worth the comparability loss.

## Severe flagged issues (union)
- **Mixed env counts inside a matched set invalidate it.** The launch wrapper must log `num_envs`, and the Part 2 analysis must assert it is identical across candidate, control and seeds. (G1, G2)
- **The 8192 speed figure is thin:** 17 solo iterations, and concurrent-8192 throughput is unmeasured. Measure before revisiting. (G1, G2, G3)
- **Part 1 arms are single-seed (s42).** Gaps under ~10 % or ~10 falls are noise. Fund seeds by cutting arms. (G3)
- **Schedule risk:** ≈ 8.5 GPU-hours plus evals. A co-scheduled eval slows a trainer by ~27 %, so batch evals. The Part 1 queue will take longer than 3.3 h. (G1, G3)
- Any 8192 side study must be labelled in its run name and config and kept out of 4096 tables. (G1)

## What would flip this decision
- A matched calibration on both seeds (e.g. baseline 8192 × 1500 vs 4096 × 3000) that shows no loss on the sim-to-real readouts, **plus** a concurrent-8192 aggregate of ≥ 1.5–2× the 4096 rate. Then 8192 would be adopted only for new, self-contained lineages.
- A hard deadline the 4096 plan can't meet even after cutting arms and seeds.
- Part 3 Bumpy/DR training that is visibly sample-starved. That would mean a separate decision for that uncompared fine-tune only.
