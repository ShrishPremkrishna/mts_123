# Decision log (lazy human decisions)

Every decision that would normally wait for a human (tagged **[H]** in `../REPRO_PLAN.md`) is made
**lazily**. The work does not block. Instead:

1. The agent that hits the decision spawns a **fresh grader agent**. It gets no prior conversation, only
   pointers to the evidence (files, run IDs, eval tables) and the question. **The requesting agent does not
   pass its own preferred answer**, so the grader isn't anchored on it.
2. The grader reads the evidence, decides, and writes one doc here using the template below. It then commits
   and pushes that doc (only that file) to `markus/opt-lab-2`.
3. Work continues with the grader's decision. The human reviews these docs afterwards and steers or overrides.
   An override is recorded as an edit to the doc's **Status** line.

File name: `YYYY-MM-DD-<workstream>-<slug>.md`, e.g. `2026-11-18-p1-parent-gait.md`.
The manager keeps the index below up to date.

## Template (keep it under ~1 page)

```markdown
# <Decision question, one line>
Status: DECIDED by grader (pending human review) · Date · Requested by: <workstream>
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
| | | | | |
