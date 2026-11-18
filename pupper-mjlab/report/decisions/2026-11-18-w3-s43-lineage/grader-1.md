# Where does the Part 2 seed-43 lineage start: the gallop_c s43 parent (A) or the s42 parent re-seeded (B)?
Status: DECIDED by grader (pending consolidation) · 2026-11-18 · Requested by: W3
Criticality: Medium. It decides what the s43 half of every two-seed mean measures, which costs about 1.1 GPU-hours (10 legs × 500 iterations) plus evals to redo.
Reversible?: yes. Before the first s43 leg starts it costs nothing. After that, the whole s43 lineage (≈ 67 min of GPU at 1.25 it/s, plus evals) must be rerun.

## Conclusion
**Option A.** Start the seed-43 lineage from the gallop_c **s43** Part 1 parent (`rp-p1-gallop_c-s43/model_2999.pt`, `y38xkkvh`). Use its own ±1.5 m/s capture (`capture-p0-gallop_c-s43`) and its own round-1 references, folded with the default arguments into `configs/opt-lab-2/p2/s43/r1-{lag,naive}`. Its control also warm-starts from the s43 parent. Report each arm as a **paired delta against the same-seed control**, plus the two-seed mean of those deltas.

## Evidence used
- **The s43 capture is clean and passes the fold-default gate.** Source: `capture-p0-gallop_c-s43/capture.json`, `fold0/s43-*/fold_summary.json`, `part2/foldcheck/p0-gallop_c-s43*.json`.
  - Capture: vx +1.548 / −1.509 m/s, 5 / 3 falls (s42: +1.561 / −1.529, 2 / 3), with about 2,170 clean cycles per direction.
  - Auto lag is 2.13 / 1.76 steps, inside the gate of 2 ± 0.5 (s42: 2.09 / 1.83). No flagged channels. Dispersion is 0.060 / 0.061 rad (s42: 0.061 / 0.068).
  - Default-fold residual against its own capture is 0.097 / 0.087 rad (s42: 0.097 / 0.094). Naive at lag 0 gives 0.099 / 0.088 (s42: 0.098 / 0.094).
  - kp·Δq is 2.21 / 2.55 Nm, against 3.82 for the parent design. Limit margin is at least 0.314 rad.
- **The two parents fold to similar references, though not identical ones.** RMS difference between `fold0/s42-default` and `s43-default` is 0.027 / 0.034 rad (max 0.098 / 0.102), against an r1 change from the design of 0.099 / 0.109 (s42) and 0.102 / 0.116 (s43). Under A, the s43 lineage gets the same treatment re-derived, not a different one. `p2/s42/r1-lag` is byte-identical to `fold0/s42-default` (sha `153acbcd…`).
- **The parents differ most on readouts that Part 2 claims are about.** Source: `part1/eval/w2/compare_all.md`, s42 vs s43.
  - R:push_stand@0.65 is 0.453 vs 0.009, drift is 67.6/65.7 vs 53.8/51.3 deg/10 s, and jitter is 0.158 vs 0.210.
  - Falls are 44 vs 41, err ±1.5 is 0.130/0.146 vs 0.123/0.129, and sat@1.5 is 0.179 vs 0.176.
- **Recovered old Part 2:** single lineage from the s42 parent only (`C_lineage_metrics.md` §1, §3d). The effects are about 0.005–0.013 in err_xy, which is close to noise.
- **Cost.** The s43 capture already exists (78 s) and the folds take about 27 s each. Both options run the same number of legs (`plans/s42.json`).

## Thought process
Priorities: (1) the second seed must remove the one-seed caveat for the findings we report (§4.3, §3: "one-seed gaps are noise"); (2) matched, confound-free comparisons inside each seed (§3); (3) cost; (4) continuity with the recovered s42 lineage.
- **(1) favours A.** The findings to reproduce ("bootstrapping helps tracking only", "stand-still hurts standing push", "r3 ≈ r2-cont") are claims about the *procedure*. Under B, both seeds share the parent, the capture and the r1 references, so the second seed measures only 500-iteration fine-tune noise. The two-seed mean would still be a single-parent result, which is exactly the old run's limitation.
  - The parent is where the large spread lives: push_stand differs by 0.44 between the parents. Under A, the push claim is tested from two very different baselines (0.453 and 0.009). Under B it is tested from one.
- **(2) is met by both.** Each arm and its control share a warm-start and a reference lineage within a seed, so arm − control cancels the parent offset under A. B is a hybrid: r1 reuses the s42 capture, but r2 and r3 capture from s43-lineage checkpoints. It is neither a clean replicate nor a clean fine-tune-noise test.
- **(3) is a wash.** A adds about 1 min of CPU folding and one commit. **(4) is kept by both**, because the s42 lineage is identical under either option and is the comparator to §3d.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| B: s42 parent re-seeded, reusing the s42 r1 refs | The two seeds share the parent, capture and r1 references, so the two-seed mean understates the variance that matters and stays a single-parent claim. The r1 legs differ only in PPO and env seed, and r2/r3 differ in capture, so the design is mixed. Its one advantage (every final candidate descends from the designated s42 parent) is addressed below. |
| B': s42 parent re-seeded, with a fresh s42 capture at a new eval seed | It adds capture noise but not parent variance. It has the same core problem as B. |

## Severe flagged issues
- **Rank on paired deltas, not on pooled absolute means.** The parents differ by 0.44 in push_stand and by about 14 deg/10 s in drift. A pooled absolute mean mixes the parent offset into the arm ranking. The s43 push_stand starts near the floor (0.009), so it can only show worsening. Treat its gains as uninformative.
- **Install the s43 references before launch** (§4.1). Refold into `configs/opt-lab-2/p2/s43/r1-lag` and `r1-naive` with the exact default arguments (naive with the same 4 pins), then commit and push. Verify that the r1-lag sha matches `fold0/s43-default` (`0f5bbbab…`). The fold0 outputs are scratch. Do not run the back-pin arm at s43 (it is s42-only by the fold decision).
- **The final Critical pick may land in the s43 lineage**, which descends from a parent the parent decision did not designate. That parent passes the same gate and is at least as good on falls and tracking (41 falls, err 0.123/0.129). The Critical decision should allow either lineage, and the Part 3 notebook must name the parent seed.
- **Code and config parity.** The s42 legs launched at head `1611a75`. The s43 legs must use the same weights JSON and `num_envs=4096`, and the analysis must assert both.
- **Old-number comparability.** Only the s42 lineage is comparable to the recovered §3d. The s43 lineage is a replication, not a second recovered comparator. Single-eval fall gaps under about 10–15 remain ties even in two-seed means.

## What would flip this decision
- The committed s43 r1 refold differs from the fold0 diagnostics: a flagged channel, |auto − 2| > 0.5, or the loader rejects the table.
- The Critical final-pick decision is pre-registered to require descent from the s42 parent and wants the two seeds as fine-tune replicates of that parent. Then B answers that narrower question.
- GPU time forces cutting the s43 lineage to r1 + control only. A is still preferred, but its evidence for r2/r3 becomes single-seed either way.
