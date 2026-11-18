# Where does the Part 2 seed-43 lineage start: the gallop_c s43 parent with its own capture (A), or the s42 parent re-seeded with the s42 r1 references (B)?
Status: DECIDED by grader (pending consolidation) · 2026-11-18 · Requested by: W3
Criticality: Medium. It decides what the "two-seed" Part 2 means are evidence of, and it shapes every s43 arm. Redoing the s43 lineage costs about 1.1 GPU-hours (10 legs × 500 it at ≈1.25 it/s).
Reversible?: yes. No s43 leg has launched yet (`~/runs/opt-lab-2/w3/` has only `status-s42.json` and `plans/s42.json`), so switching now is free. After launch it costs the s43 legs already run.

## Conclusion
**Option A.** Start the s43 lineage from `~/runs/opt-lab-2/rp-p1-gallop_c-s43/model_2999.pt` (`y38xkkvh`). Its round-1 references are the default and naive folds of its own capture, `capture-p0-gallop_c-s43` (already folded in `fold0/s43-default`, sha `0f5bbbab…`, and `fold0/s43-naive`). Install them as `configs/opt-lab-2/p2/s43/r1-{lag,naive}` and commit and push them before launch. Use the same fold arguments, pins and weights as s42. `mkplan.py 43 <s43 parent> configs/opt-lab-2/p2/s43 0` already supports this.

## Evidence used
- **The s43 capture is as clean as the s42 one** (`capture.json`, `fold0/*/fold_summary.json`, `foldcheck/p0-gallop_c-s4{2,3}.json`):
  - Achieved vx: +1.548 / −1.509 for s43, against +1.561 / −1.529 for s42. Falls in the capture: 5 / 3 against 2 / 3.
  - Clean cycles: 2172 / 2176 against 2175 / 2174.
  - Default fold, s43 vs s42 (fwd / back): measured lag 2.13 / 1.76 vs 2.09 / 1.83, so |auto − 2| ≤ 0.5 and the fold-default gate passes. Dispersion 0.060 / 0.061 vs 0.061 / 0.068 rad. Flagged channels: none in either.
  - Same fold, s43 vs s42: fold residual (`rms_resid`) 0.097 / 0.087 vs 0.097 / 0.094. kp·Δq 2.21 / 2.55 vs 2.17 / 2.53 Nm. Peak speed 20.6 / 23.8 vs 20.5 / 24.4 rad/s.
- **The two r1 references are close but not identical.** I computed this directly from the npz files: s42-default vs s43-default differ by an RMS of **0.027 / 0.034 rad**. Each one moves **0.099–0.116 rad** from the parent. Two independent captures give about the same update, so A is a faithful replicate and not a different experiment.
- **Both parents are valid, and they differ** (`part1/eval/w2/compare_all.md`, s42 / s43):
  - Same arm, recipe and reference blob `855c59eb`. W1 checked that the code diff between the two launch commits touches no training code.
  - falls 44 / 41; err ±1.5 0.130/0.146 vs 0.123/0.129; F_runmax_med 3.03 / 3.09 BW; sat@1.5 0.179 / 0.176.
  - **push_stand@0.65 0.453 / 0.009**; drift 68/66 vs 54/51 deg/10 s; jitter 0.158 / 0.21.
  - Training err_xy 0.217 / 0.220 (`part1_notebook.md`).
- **The effects under test are about as large as the gap between the parents.** In `C_lineage_metrics.md` §3d the bootstrapping gain over the control is 0.008–0.013 err_xy. The two parents' err ±1.5 differ by 0.007–0.017.

## Thought process
1. **What a second seed must replicate.** The claims to reproduce ("bootstrapping improves tracking only", "r3 ≈ r2-continued", and the stand-still push effect) are claims about the *procedure*: take a converged gait policy, capture it, fold it, warm-start. The recovered run tested this on one parent only.
   - Under A, each seed is an independent run of the whole pipeline: parent, capture, fold, round 1, round 2, round 3.
   - Under B, the two s43 r1 arms share their initial weights and their reference with the s42 r1 arms. They differ only in PPO and environment RNG, which makes them close to repeat measurements (pseudo-replication).
   - B would therefore report a two-seed mean whose spread understates the real variance, and it would still test only one parent. That contradicts the purpose of §3 ("one-seed gaps are noise") and REPRO_PLAN §4.3 (two seeds to remove the one-seed caveat).
2. **Matched comparisons survive A.** Every s43 arm has an s43 control that starts from the same parent, uses the same weights and runs the same iterations, so the parent's offset cancels within each seed (§3: always run a control at equal iterations). B's main advantage, lower variance from a shared starting point, is precision about one parent instance. A buys the same precision through within-seed paired deltas.
3. **Self-consistency.** Bootstrapping means each reference is the policy's own gait. Under A every reference is captured from its own lineage. Under B the s43 round 1 is trained on another policy's gait, while its rounds 2 and 3 are self-captured, so the rounds are not alike.
4. **Cost and risk.** The GPU cost is identical. A adds two committed folds (about 30 s of CPU each), and both already exist in `fold0`. The s43 capture passes every pre-r1 gate of the fold-default decision.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| B: s42 parent re-seeded, s42 r1 references | Pseudo-replication. Round 1 differs from s42 only in RNG, so the two-seed means understate variance and generalise to one parent only. The rounds of the s43 lineage would also be inconsistent (a foreign r1, then self-captured r2 and r3). Its only gain, a tighter spread, is already provided by the within-seed controls |
| A, but re-capturing or re-folding with s43-specific settings | That would confound seed with fold settings. Use the identical default arguments and pins |

## Severe flagged issues
- **Do not rank on raw two-seed means of levels.** The parents differ by 0.44 on push_stand@0.65 (0.453 vs 0.009) and by about 0.01 on err ±1.5, which is as large as the effects. Report each arm as its delta from the same-seed control at the same iteration, then average the deltas over the two seeds. Report per-seed signs, and call it a reproduction only when both seeds agree in sign.
- **The s43 parent's standing push is at the floor (0.009).** "Stand-still worsens standing push" can show up as a rise at s43, but no improvement can. The fixed design has no iteration-matched splay-weight comparator, so this claim is compared against the parent (+0 iterations) only. State that as a caveat.
- **Provenance for the Critical final pick.** Under A, a winning s43 checkpoint descends from `y38xkkvh`, not from the designated `1c5r0n8c` parent (same arm, recipe and reference; s43 is equal or better on the ranked readouts). That decision must name the lineage and cite that parent's Part 1 numbers.
- Commit and push `p2/s43/r1-*` before launch, and log the reference sha (§4.1). Assert `num_envs` 4096 across seeds and arms. The `r1-backpin` arm stays s42-only.
- Both folds shrink amplitude (retention about 0.70, peak speed 45 → about 21 rad/s). This affects both lineages equally and is not seed-specific, but watch for crouch or stride collapse in r1 on either seed.

## What would flip this decision
- The s43 r1 fold, as installed, fails verification or flags a channel, or round-1 legs at s43 collapse while s42 does not, with the cause traced to the s43 capture rather than to RNG. Then fall back to B for round 1 and record it.
- The final-pick (Critical) decision rules that Part 3 must descend from `1c5r0n8c` only and that s43 serves purely as a noise estimate. Then B's tighter spread would be the right tool.
- A code change between the s42 and s43 launches is found that touches training (that would void the W1 commit check).
