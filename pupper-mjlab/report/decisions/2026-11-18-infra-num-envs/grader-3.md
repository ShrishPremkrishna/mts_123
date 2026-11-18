# num_envs and iteration budget for all remaining training (Part 1 arms, Part 2 legs, Part 3) on 1× A100-40GB?
Status: DECIDED by grader (pending human review) · 2026-11-18 · Requested by: infra (W0)
Criticality: High. It gates every downstream comparison (Part 1 parent pick, Part 2 matched table, the deployed policy), and redoing it costs > 3 GPU-hours.
Reversible?: Yes, cheaply now: nothing has to be undone, since 4096 is what is already running. Switching later costs re-running every arm it would be compared against (Part 1 ≈ 5 h, Part 2 ≈ 3 h).

## Conclusion
**Keep the recovered recipe: `--num-envs 4096` with unchanged iteration budgets for every remaining training run.** Do not mix by part.
- **Part 1:** let the in-flight queue (baseline, fast_trot, gallop_a/b/c at s42, max-concurrent 3) finish as launched, at 4096 × 3000. Do not restart it. The s43 runs for the top candidates also use 4096 × 3000.
- **Part 2:** every bootstrapping leg, continuation and control uses 4096 × 500 per leg, warm-started from the 4096 parent, at s42 and s43. Compare at +500/+1000/+1500.
- **Part 3:** use 4096. Any arms that are compared run equal iterations.
- **If compute gets short:** cut arms or seeds (§3), never iterations or envs. `evaluate` keeps its own small `--num-envs`, since this decision covers training only.

## Evidence used
- `infra/a100_throughput.md`
  - 4096 envs solo: 0.78 s/iter (127 k env-steps/s). The aggregate is flat at ≈ 1.25 it/s for N = 1–6 processes.
  - 8192 envs solo: 0.90 s/iter ≈ 218 k env-steps/s (1.7×). This rests on **n = 17 iterations, one process**. Concurrent 8192 jobs were never measured.
- `C_lineage_metrics.md`: every recovered run used 4096 envs. Those runs give the sanity targets in §4.6: gallop_c-s42-r2 reward 151.8 and err_xy 0.251 at iteration 2998, plus the full §3d matched Part 2 table. The old A10 also did 0.79 s/iter solo, so the bottleneck is per-step host overhead.
- `rl_cfg.py` and `A_code_recovery.md`:
  - PPO collects 24 steps/env per iteration and runs 5 epochs × 4 mini-batches, so there are always **20 gradient steps per iteration**. The minibatch is 24,576 samples at 4096 envs and 49,152 at 8192.
  - lr is adaptive on desired_kl 0.01.
- `pupper_gait_env_cfg.py`: the curricula are counted **in iterations** (`common_step_counter` = iterations × 24, independent of num_envs). That covers the airborne pretrain (`GAIT_PRETRAIN_ITERS`, step 12000 = 500 iterations) and the yaw-command stages.
- Live state: 3 trainers at 3.7 GB each plus one eval are on the GPU at 100 % util. The `part1_queue.py` job list holds 5 arms.
- Lab spec: it sets neither num_envs nor iteration counts. Deliverables are tables against a baseline and per round.

## Thought process
Priority 1 is internal comparability (§3: fixed budgets, equal-iteration comparisons, controls with the same extra iterations). Priority 2 is comparability with the recovered numbers, which are this reproduction's only sanity check. Priority 3 is wall-clock and seeds.

1. **8192 at the same iterations** keeps the curricula intact and doubles data per run. It **does not save time**: each run gets ~15 % slower, and concurrent behaviour is unknown. It also breaks comparison with every recovered number and gives no deliverable a reason to need it. Training is host-bound, so the real throughput gain only appears if iterations are cut.
2. **8192 at half the iterations** (env-step-matched) is the only option that buys wall-clock. That time could fund a third seed, which is the best argument for it. Against it:
   - It halves gradient steps (30 k vs 60 k).
   - It turns the 500-iteration pretrain from 1/6 of training into 1/3, and compresses the yaw curriculum, unless the code is changed.
   - It makes Part 2's "500-iteration legs" mean something different from the recovered table.
   - It is exactly "cutting iterations", which §3 forbids.
   - The 1.7× figure is one 17-iteration solo sample.
3. **Mix by part** is worse than either pure option. Part 2 warm-starts from Part 1 checkpoints whose Adam and adaptive-lr state are tuned to a 98 k batch. A switch at the Part 1→2 or 2→3 seam adds a confound inside the deployed policy's lineage. Part 3 is the sim-to-real step, the last place to change two things at once.
4. **Cost of the status quo is small:** Part 1 started minutes ago at 4096 and the recipe fits the budget (§4.3: ≈ 5 h + 3 h). Keeping it loses nothing already spent and needs no code change.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| 8192+ envs, same iterations | No wall-clock gain (~+15 % per run). Breaks comparison with the recovered numbers. Concurrent scaling not measured. Part 1 would need a restart. |
| 8192 envs, halved iterations (env-step-matched) | Violates "cut arms, not iterations". Distorts the iteration-indexed curricula (pretrain 500 iterations, yaw stages) and the meaning of a 500-iteration Part 2 leg. Halves gradient steps. Rests on one 17-iteration sample. |
| Mix by part (e.g. 4096 for Part 1, 8192 for Parts 2/3) | Adds a batch-size change mid-lineage in warm-started runs. Part 2 controls and treatments would still need to match, so the switch only adds a confound. Part 3 is too close to hardware for an untested change. |
| Restart Part 1 now at 8192 | Throws away running work and makes the recovered Part 1 table unusable as a target, for no wall-clock gain. |

## Severe flagged issues
- The Part 1 arms are **single-seed (s42)**. Under §3, gaps under ~10 % or ~10 falls are noise. Seed time should come from cutting arms, not from shrinking runs.
- Concurrency at 3 trainers plus evals: each eval context costs a trainer ~27 % (`a100_throughput.md`). Expect the 5-arm queue to take longer than 15 000 iterations / 1.25 it/s ≈ 3.3 h.
- Nobody has measured whether 8192 envs learn better per iteration. If it matters, it is unknown, not "worse".

## What would flip this decision
- A calibration pair, baseline at 8192 × 3000 vs 4096 × 3000 (or × 1500, with the curricula rescaled), showing clearly better sim-to-real readouts (falls, foot force, drift) beyond ~10 % seed noise. That would justify 8192 for a **new, fully re-run** lineage, never a mixed one.
- A hard deadline that the 4096 plan cannot meet even after cutting arms or seeds.
- Part 3 Bumpy/DR training being visibly sample-starved (unstable or plateaued with high variance). That would justify a separate decision on 8192 for the final, uncompared Part 3 fine-tune only.
