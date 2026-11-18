# Decision log (lazy human decisions)

Every decision that would normally wait for a human (tagged **[H]** in `../REPRO_PLAN.md`) is made
**lazily**. The work does not block. Instead:

1. **Rate criticality.** The agent that hits the decision rates it with the table below, which sets how many
   independent graders run.
2. **Independent graders.** Spawn N **fresh grader agents** in parallel. Each gets no prior conversation, only
   the question, pointers to the evidence (files, run IDs, eval tables) and the template. They get **neither the
   requester's preferred answer nor each other's output.** Each writes `<slug>/grader-<k>.md`.
3. **Agreement check.** Graders agree when their Conclusions pick the same option. Wording and minor caveats
   don't count as disagreement.
   - **N = 1, or all agree:** the requester writes `<slug>.md`, a short consolidation: the shared conclusion, links to the
     grader docs, and the **union** of their severe flagged issues. It must not alter the conclusion.
   - **Any disagreement:** spawn a **fresh arbitrator agent**. It gets the evidence pointers and all grader docs,
     mediates the differences (which disagreements are factual, which are about criteria), checks the disputed evidence
     itself, and writes the final `<slug>.md` with an extra **Disagreement & resolution** section.
4. Commit and push only the decision files to `markus/opt-lab-2`. Work continues with the final decision.
   The human reviews afterwards and steers or overrides. An override is recorded as an edit to the **Status** line.

| Criticality | When | Graders |
|---|---|---|
| Low | Cheap to reverse (< ~1 GPU-hour), nothing downstream depends on it | 1 |
| Medium | Shapes one workstream's results or costs ~1–3 GPU-hours to redo | 2 |
| High | Gates downstream work (e.g. Part 1 parent gait, Part 2 final policy), or costs > 3 GPU-hours to redo | 3 |
| Critical | Hard to reverse, feeds the submitted report's headline claims, or prepares anything physical or safety-related | 4 |

Names: `YYYY-MM-DD-<workstream>-<slug>.md` for the final doc, plus a folder `YYYY-MM-DD-<workstream>-<slug>/`
for the grader and arbitrator docs. The manager keeps the index below up to date.

## Template (keep it under ~1 page)

```markdown
# <Decision question, one line>
Status: DECIDED by grader | graders (k/N agree) | arbitrator (pending human review) · Date · Requested by: <workstream>
Criticality: Low | Medium | High | Critical, with a one-line reason
Reversible?: yes/no, and what it costs to reverse (GPU-hours, rework)

## Conclusion
<The decision in 1–3 sentences.>

## Evidence used
- <file / run ID / table row>, with the specific numbers that mattered

## Thought process
<Criteria and their priority (cite REPRO_PLAN §3 principles when used), how the evidence scored, and the reasoning.>

## Alternatives considered
| Option | Why not chosen |
|---|---|

## Severe flagged issues
- <Anything that could invalidate the decision or the experiment: confounds, unmatched iterations,
  one-seed noise, broken metrics, sim-to-real risk, safety. Write "None" if there are none.>

## What would flip this decision
- <Concrete evidence that should trigger a re-decision.>
```

## Index

| Date | Workstream | Decision | Conclusion | Human review |
|---|---|---|---|---|
| 2026-11-18 | W0 | Max concurrent training jobs ([doc](2026-11-18-w0-concurrency.md)) | 4 across all agents (throughput is flat; no MPS) | |
| 2026-11-18 | infra | num_envs / iteration budget ([doc](2026-11-18-infra-num-envs.md)) | Keep 4096 envs and the recovered budgets; cut arms, not iterations (3/3 graders) | |
| 2026-11-18 | W2 | Part 2 eval metric definitions ([doc](2026-11-18-w2-eval-metrics.md)) | Keep fall categories (+ per-fall log); push 0.5/0.65/0.8 m/s, ranked on standing@0.65 (pilot-frozen) and moving@0.5; foot force ranked on the median of per-run max (arbitrated) | |
| 2026-11-18 | W2 | Default fold for Part 2 rounds ([doc](2026-11-18-w2-fold-default.md)) | `lag` with a fixed 2-step shift, median, 6 harmonics, no symmetrize; pin all 4 abduction channels to the parent (arbitrated) | |
| 2026-11-18 | W1 | Part 1 parent gait + seed-43 arms ([doc](2026-11-18-p1-parent-gait.md)) | Provisional parent `gallop_c` s42 (`1c5r0n8c`); s43 for gallop_c + fast_trot; post-s43 check: two-seed falls 42.5 vs 55.5, pick stands (3/3 graders) | |
