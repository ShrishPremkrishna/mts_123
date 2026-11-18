# Decision log (lazy human decisions)

Every decision that would normally wait for a human (tagged **[H]** in `../REPRO_PLAN.md`) is made
**lazily**. The work does not block. Instead:

1. **Rate criticality.** The agent that hits the decision rates it with the table below, which sets how many
   independent graders run.
2. **Independent graders, each with a different view (added 2026-11-18 20:45 UTC).** Spawn N **fresh grader agents** in parallel.
   Each gets no prior conversation, only the question, the template, and its own **lens**: a different primary slice and
   framing of the evidence, assigned in this order:

   | # | Lens | Primary view | Instruction |
   |---|---|---|---|
   | 1 | **Devil's advocate** | Everything, plus the name of the current front-runner (the requester's default, or the top option on the ranked metric) | Build the strongest case *against* the front-runner, then conclude honestly. It may still pick the front-runner. |
   | 2 | **Raw data only** | Tables, metric CSVs and eval JSONs, but **not** the requester's notebook prose or hypotheses | Decide from the numbers without the requester's framing. |
   | 3 | **Sim-to-real / safety skeptic** | Hardware-relevant readouts: saturation, foot force, nominal-parameter evals, push, falls | Weigh deployment risk first and discount reward and tracking. |
   | 4 | **Noise skeptic** | Seed spread, eval repeats, matched-iteration checks | Ask, for each gap, whether it is real. Prefer "tie → cheaper, safer, or more reversible option". |

   **Lenses change emphasis, never facts.** Every pointer is true, and every grader may open any evidence it asks for. Each is
   told its view is deliberately partial and must list what it would need to see to change its mind.
   - **N = 1 (Low):** a neutral grader, with a required "Strongest case for the runner-up" section.
   - Graders never see the requester's preference (except the front-runner name given to lens 1) or each other's output.
     Each writes `<slug>/grader-<k>-<lens>.md`.
3. **Synthesis (always for N ≥ 2).** A **fresh synthesizer agent** gets all grader docs plus the evidence pointers. It:
   - separates factual disputes from disputes about criteria;
   - checks disputed or decisive evidence itself;
   - writes the final `<slug>.md` with a **Perspectives & synthesis** section saying which lenses agreed and which objections survive.

   Unanimity across different lenses counts as strong evidence. Unanimity alone is not a reason to skip synthesis.
   If the lenses split, the synthesizer is the arbitrator, and a **Disagreement & resolution** section is required.
4. Commit and push only the decision files to `markus/opt-lab-2`. Work continues with the final decision.
   The human reviews afterwards and steers or overrides. An override is recorded as an edit to the **Status** line.

_Before 20:45 UTC on 2026-11-18, graders were symmetric (same view, no lens), and unanimous decisions were consolidated
without a synthesizer. Those decisions get a retroactive devil's-advocate review in `<slug>/devils-advocate.md`._

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
| 2026-11-18 | W3 | Seed-43 Part 2 lineage start ([doc](2026-11-18-w3-s43-lineage.md)) | Start s43 from the gallop_c s43 parent (`y38xkkvh`) with its own capture/r1 refs; rank on per-seed deltas vs the same-seed control (2/2 graders) | |
| 2026-11-18 | W4 | Part 3 seam stumble definition + verdict rule ([doc](2026-11-18-w4-seam-thresholds.md)) | Incident = fall or knee/torso contact or tilt > 30° in [−0.2, +1.0] s; per-cell excess vs no-seam control > 0.05 with Fisher p < 0.01, or pooled falls > 2 % → FAIL (arbitrated) | |
| 2026-11-18 | W4 | Part 3 Bumpy + DR fine-tune budget ([doc](2026-11-18-w4-finetune-budget.md)) | 1500 iterations from the Part 2 final, evals at +500/+1000/+1500, ship +1500 unless a pre-registered guard fires; matched flat-continuation control (2/2 graders) | |
| 2026-11-18 | W4 | Part 3 PD-gain DR coverage ([doc](2026-11-18-w4-kp-coverage.md)) | Measure the final's nominal-gain vs flat_dr falls; gap > 15 → fine-tune + control train with kp ×[0.6, 1.3], kd ×[0.7, 1.5], else Lab 4. Eval always on Lab 4; nominal-gain falls gate selection and hardware HOLD (arbitrated, 2–1) | |
