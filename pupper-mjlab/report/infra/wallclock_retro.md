# Wall-clock retrospective (as of 2026-11-18 20:40 UTC)

Source: `~/runs/opt-lab-2/*.launch.json` (`launched_at`), final-checkpoint mtimes, and the `Iteration time` lines in each run log.

## Where the time went
- **Session start → first training run:** 10:36 → 11:11 (recovery, repo setup, writing the references).
- **GPU work done:** Part 1 was 7 runs × 3000 iterations = 21 000; Part 2 was 21 legs × 500 = 10 500.
  - Total: 31 500 iterations.
  - At the benchmarked ceiling of about 1.28 it/s total (0.78 s/iter solo, flat with concurrency), that is about **6.8 GPU-hours**.
  - Actual: 11:11 → 20:38 = **9.45 h**, so about 2.6 h of overhead.
- **Measured aggregate throughput:**

| Phase | Concurrent runs | s/iter per run | Aggregate (it/s) |
|---|---|---|---|
| Part 1 | 3 | 2.6–2.8 | ≈ 1.1 |
| fast_trot s43 | 1 (solo) | 0.85 | 1.18 |
| Part 2 (with evals and Part 3 smokes) | 4 | 3.0–3.6 | ≈ 0.94–1.2 (−25 % vs ceiling at the low end) |

## Critical path (to the final Part 2 policy)

| Span | Duration | What it was |
|---|---|---|
| 11:11 → 13:40 | 2.5 h | gallop a/b/c s42 (3 concurrent). The Part 1 queue took gallops first, and only 3 of the 4 slots. |
| 13:39 → 16:02 | 2.4 h | fast_trot s42, baseline, gallop_c s43. **The decision-relevant comparator (fast_trot) started 2.5 h after the gallops.** |
| 16:02 → 16:25 | 0.4 h | Evals, notebook, 3 graders → parent = gallop_c s42 |
| 16:24 → 17:12 | 0.8 h | fast_trot s43 solo: confirmatory only (it could not change the parent) |
| 16:25 → 17:32 | 1.1 h | **Manager latency:** W3 was spawned only when W1 reported (after fast_trot s43), not when the decision landed. GPU had no training 16:10–16:24 and 17:12–17:32. |
| 17:32 → 20:38 | 3.1 h | Part 2: 21 legs (an ideal 2.3 h at the ceiling; lost about 0.8 h to co-scheduled evals, capture gaps of 8–14 min between rounds, and Part 3 smokes) |
| 20:38 → … | ~1 h? | Remaining evals + 4 lenses + synthesizer |

## What forethought would have removed (≈ 3–4 h of the 9.5 h)
1. **A critical-path priority queue (≈ 3 h).** We already knew from recovered facts that gallop A drags its feet and never learned to run backward, and that gallop C was the old pick. Run the decision set first: {gallop_c, fast_trot} × {s42, s43} = 12 000 iterations ≈ 2.9 h, finishing around 14:05.
   - Decide by about 14:30 and start Part 2 at top priority.
   - Run the report-only arms (gallop_a, gallop_b, baseline: 9000 iterations) in the background or after Part 2.
   - The final Part 2 policy would have been ready around 17:15 instead of after 20:40, for the same total GPU work.
2. **Speculative seeds (≈ 0.8 h).** Launch s43 for the prior top 2 at the start instead of after the decision. That removes the serial, confirmation-only fast_trot s43 run.
3. **Event-driven hand-off (≈ 1 h).** A decision doc landing should trigger the downstream workstream at once (the agent messages the manager on push). Don't wait for the upstream agent to finish everything.
   - Also, run cheap prerequisites (the round-0 fold check on the leading candidate) speculatively as soon as its checkpoint exists (13:40, not 17:20).
4. **Fewer GPU-iterations (≈ 0.5–1.4 h of GPU).** Early-kill arms whose failure mode is known: gallop_a/b move forward when commanded backward.
   - Kill rule: at ~1000 iterations, kill an arm that cannot reach −0.5 m/s, and report it as "abandoned at 1000: no backward gait".
   - This cuts arms, not matched iterations, but it should be decided before launch.
5. **Eval scheduling (≈ 0.5–0.8 h).** Co-scheduled evals cost ~25 % of trainer throughput. Instead, put evals in a dedicated window or in natural GPU gaps (16:10–16:24, 17:12–17:32 were idle), and evaluate intermediate checkpoints with a light suite only.
6. **Put deployed-parameter evals in the harness spec from the start (risk, not hours yet).** The nominal-PD-gain fall rate (parent: 99/79 vs 40 under Lab 4 DR) was found at 19:22, 8 h in, by the Part 3 agent.
   - In W2's spec at 12:00, it would have been a Part 1 selection readout and could have informed the Part 2 training DR.
   - It may now force the wider-kp Part 3 fine-tune, or a hardware HOLD.
   - Related: start the Part 3 tooling agent at the start (it used only the baseline and parent checkpoints), not at 17:30.
7. **Uncertain:** since every run was redone from scratch, the comparability argument for 4096 envs (vs the recovered numbers) was weaker than assumed. 8192 envs gives 1.7× env-steps/s, but it only saves wall clock if iterations are cut, and that was never calibrated. The retroactive devil's-advocate review of `2026-11-18-infra-num-envs` covers this.

## Rules for the next run
- Before launching, write down the **decision set** for each gate and give it top queue priority. Report-only arms fill the leftover capacity.
- Agents message the manager the moment a decision or gate artifact is pushed. The manager pre-briefs downstream agents to start on that file.
- The eval spec includes "deployed parameters" (nominal gains, no DR) from day one.
- Kill rules for arms are pre-registered before launch.
