# Max concurrent 4096-env training processes on the shared A100-40GB (no MPS)?
Status: DECIDED by grader (pending human review) · 2026-11-18 · Requested by: W0
Criticality: Low. It is a scheduling cap, it does not change training numerics, and it can be changed at any time.
Reversible?: yes. Change the one-line file. Runs in flight are unaffected, or can be stopped and resumed from `save_interval=50` checkpoints (≤ 50 iterations lost, i.e. minutes).

## Conclusion
**4.** At most 4 concurrent `--num-envs 4096` training processes on the GPU, counted across all agents. Small `evaluate` jobs
do not count against the cap, but there should be at most ~2 at a time. Treat 4 as a ceiling, not a target: when lanes are contended,
critical-path runs (whatever the next decision or capture waits on) go first.

## Evidence used
- `report/infra/a100_throughput.md`: aggregate rate is flat across N. 1.29 / 1.16 / 1.22 / 1.23 / 1.26 it/s for N = 1/2/3/4/6
  (≈ 114–127 k env-steps/s, spread within noise; N=2 had only 14 clean samples). Per-process s/iter 0.78 / 1.72 / 2.46 / 3.25 / 4.76.
- Same file: 3.7 GB per process (≈ 10 fit in 40 GB), so memory is not binding. Startup 34–42 s per process.
  One ~570 MiB eval costs a solo trainer 27 % (0.78 → 1.07 s/iter), because time-slicing is per context.
- `REPRO_PLAN.md` §2/§5: Part 1 ≈ 6 arms × 3000 at s42 + 2 × 3000 at s43 ≈ 24 k iterations. Part 2 ≈ 14 legs × 500 × 2 seeds ≈ 14 k iterations,
  with legs sequential inside a lineage (r1 → capture → r2 → …). §4.2 says no MPS and to resume from checkpoints. §4.4 says keep 4096 envs.

## Thought process
Throughput can't separate the options, because the GPU delivers ≈ 1.25 it/s total for any N in 1–6. So the cap should rest on the secondary criteria:
1. **Critical-path latency (Part 2).** Each process runs at ≈ 1.25/N it/s. A 500-iteration leg takes ≈ 7 / 20 / 27 / 40 min at N = 1/3/4/6.
   Part 2 has a chain of roughly 5–6 dependent legs (r1, capture, r2, r3, continuations). At N=4 that chain takes ≈ 2.5–3 h, about the
   ≈ 3.1 h the total work needs anyway. At N=6 the chain (≈ 4 h) becomes the bottleneck. Lower N does not shorten the makespan.
2. **Keep the GPU fed despite agent latency and startup.** Agents notice finished runs and launch the next ones with a delay of minutes, plus a
   ~40 s startup. N=1 would leave the GPU idle in every such gap and force all agents (W1, W2 evals, W3) into one queue.
   With N ≥ 3, other processes fill those gaps.
3. **Stay inside the measured range.** Benchmarks only go up to 6 contexts. With 4 trainers plus ≤ 2 evals there are ≤ 6 contexts, which is still
   measured territory. A cap of 6 plus evals would be extrapolation.
4. **Natural experiment unit.** A matched Part 2 comparison (candidate and control × s42/s43) is exactly 4 runs. They start and finish together
   at equal iterations (§3 "compare only at equal iterations", "always run a control"). Part 1's 6 + 2 runs fit in two waves of 4.
5. **Blast radius / isolation (§4.2).** Without MPS a fault should stay in its own context. Even a device-wide fault costs ≤ 50 iterations
   for each of ≤ 4 runs.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| 1 | Same aggregate rate, but idle gaps at every launch, all agents serialized, and a single eval costs 27 % |
| 2 | Lowest measured aggregate (1.16, noisy). Too few lanes to run a matched 2-seed candidate + control set together |
| 3 | Close second. Faster per-run (2.46 s/iter), but splits the 4-run matched set across waves and leaves less room for agents sharing the GPU |
| 6 | Critical-path legs slow to ≈ 4.8 s/iter (3000 iterations ≈ 4 h per run), and with evals it goes past the measured ≤ 6 contexts. No throughput gain |
| 8192 envs | Out of scope: it changes PPO numerics and is a separate [H] decision (§4.4) |

## Severe flagged issues
- **The compute budget in REPRO_PLAN §5 is optimistic.** Since throughput is flat, Part 1 ≈ 24 k iterations / 1.25 ≈ **5.3 h** (not 1.5–2 h), Part 2 ≈ 14 k / 1.25 ≈ **3.1 h**,
  plus evals. Schedules should use `total iterations / 1.25 s`, whatever the concurrency.
- Evals take a disproportionate share of the GPU (per-context time-slicing). Batch them and avoid many concurrent eval processes.
- The cap must be enforced across agents (shared count via `nvidia-smi --query-compute-apps`), not per agent.

## What would flip this decision
- A benchmark showing aggregate throughput clearly rising or falling with N (> ~10 %), e.g. contention or a crash at 4 trainers + 2 evals.
- Repeated co-scheduled faults that kill several runs at once → lower the cap to 2–3.
- An adopted 8192-env change (6.7 GB/proc, different host/GPU balance) → re-benchmark and re-decide.
- Part 2 turning out to need fewer independent lanes (e.g. single seed) → 3 is enough.
