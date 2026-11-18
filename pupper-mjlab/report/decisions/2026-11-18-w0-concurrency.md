# Max concurrent 4096-env training processes on the shared A100-40GB (no MPS)?
Status: DECIDED by grader (1/1, pending human review) · 2026-11-18 · Requested by: W0
Criticality: Low. It is a scheduling cap and does not change training numerics.
Reversible?: yes. Edit `~/recovery/CONCURRENCY`. In-flight runs can resume from `save_interval=50` checkpoints.

## Conclusion
**4** concurrent `--num-envs 4096` training processes, counted **across all agents**. This is a ceiling, not a target. Small `evaluate`
jobs are outside the cap, but keep them to ≤ ~2 at once. Critical-path runs go first.

Grader doc: [2026-11-18-w0-concurrency/grader-1.md](2026-11-18-w0-concurrency/grader-1.md).
Evidence: [`report/infra/a100_throughput.md`](../infra/a100_throughput.md). The aggregate is flat at ≈ 1.25 it/s for N = 1–6, with 3.7 GB per process.

## Severe flagged issues (union)
- The REPRO_PLAN §5 compute budget was optimistic. With flat throughput, Part 1 ≈ 5 h and Part 2 ≈ 3 h, plus evals. §4.3 has been updated.
- Evals take a disproportionate time slice (per-context time-slicing): one ~570 MiB eval costs a solo trainer 27 %.
- The cap must be enforced across agents, e.g. by counting `nvidia-smi --query-compute-apps` training PIDs.
