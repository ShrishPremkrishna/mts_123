# Devil's advocate: seed-43 Part 2 lineage (A: own s43 parent)
Reviewer: red-team, retroactive · 2026-11-18 ~20:45 UTC · read-only CPU check of run dirs, launch.json, W3 eval JSONs (`~/wt/w3/.../report/part2/eval/w3*/`).

**Verdict: STANDS WITH NEW CAVEAT.** The graders' main premise (the parent holds the large variance, so B would measure "only fine-tune noise") is not supported by the data. A is still the safer design. The final pick must not inherit the s42 parent's nominal-PD and Part 3 dry-run evidence.

## Strongest counter-case
- **Fine-tune noise is as large as the parent gap.** The 0.44 push_stand@0.65 gap between the parents (0.453 vs 0.009) was the graders' headline reason for A. Inside each lineage, legs only 500 iterations apart already span that range:
  - s42: 0.10–0.57 (SD 0.17).
  - s43: 0.00–0.55 (SD 0.20). s43 r1-naive scores 0.00, r1-cont-1000 0.04, ctrl-500 0.55.
  - The parent difference disappears after one leg. ctrl-500 gives push_stand 0.568 / 0.547, err 0.112/0.129 vs 0.115/0.130, and sat@1.5 0.180 / 0.183 (s42 / s43).
  - So B would not have measured a small "fine-tune noise". That noise is the dominant variance.
- **Under that reading, B was an adequate replicate and kept two things A gives up:**
  - every final candidate descends from the designated `1c5r0n8c`;
  - the s42 lineage stays comparable with the recovered §3d numbers.
- **What A gives up is now concrete.** s43 r2-cont-1500 ties for best (15 falls, err 0.087/0.114 vs s42 16, 0.096/0.115), so the final may well come from the s43 lineage.
  - Every nominal-PD and Part 3 dry-run number comes from the s42 Part 1 parent: 99 / 79 nominal-PD falls on eval seeds 1234 / 4321, kp step 0 Δ 48.5, and the seam and ablation results (`part3_notebook.md` ~179–240).
  - The s43 parent `y38xkkvh` has **never been evaluated at nominal PD gains**.

## Why it still stands (checked, not taken from prose)
- **Genuine replicate, not confounded.** Lineage verified from launch.json `--checkpoint` and sha256:
  - r1-lag, r1-naive and ctrl-500 start from `rp-p1-gallop_c-s43/model_2999.pt` (`ef7ff92b`).
  - The r2, r3 and cont legs chain from the s43 legs only.
  - The installed r1-lag npz equals `fold0/s43-default` (`0f5bbbab`).
  - Parent train_argv is identical to s42 except the seed. Same reference `855c59eb`, same weights sha `4a9ce0a4`, 4096 envs, 500 iterations per leg.
  - `git diff` 2e7f5b9..a042c72 and 1611a75..a898f59 touch no training code. The only change is `launch.py`'s opt-in `--checked-bumpy`, which these legs do not use.
- **Iterations are matched.** Every leg has 1 segment (no crash or resume), with targets 3498 / 3997 / 4496 at both seeds.
- **A is a superset.** It samples both the parent variance and the fine-tune variance, so its sign-agreement rule is the more conservative one. The cost of that is losing designated-parent provenance, which is fixable.
- **Results look like the design working:**
  - Paired tracking deltas agree in sign: r1-lag − ctrl-500 err back −0.015 / −0.014; r2 − ctrl-1000 −0.019/−0.006 vs −0.018/−0.016.
  - sat@1.5 drops at both seeds.
  - r3 ≈ r2-cont reproduces: Δerr within ±0.007 at both seeds.
  - Fall deltas flip sign across seeds (r1-cont-1000 − ctrl-1000: −9 vs +12), which is exactly the one-seed trap a second seed should expose.

## What the graders missed or under-weighted
1. **They never compared the parent gap with within-lineage spread.** They read a 2-point parent difference as "parent variance". It is mostly checkpoint-level volatility.
   - push_stand ranks on one eval of one checkpoint and swings by 0.5 between neighbours.
   - The "s43 at the floor" caveat is moot for paired deltas, because the s43 controls sit at 0.50–0.55.
2. **Nominal-PD robustness was not a criterion.** The lineage choice decides whether the final's ancestry has any sim-to-real evidence.
3. **The repeat evals reuse eval seed 1234.** All 58 Part 2 JSONs use it, including `-rep2`, so the "±10–15 falls" band is GPU nondeterminism only.
   - The s43 parent already reads 33 / 41 / 43 falls on that one seed.
   - The s42 parent reads 99 vs 79 at nominal across two eval seeds.
4. **Minor:** grader-1 says "the s42 legs launched at head `1611a75`". Only 4 of 11 did. The others launched at `27c48ff`, `2015e16`, `e926ff8`, `90b3e59` and `a898f59`. This is harmless, since no training code changed.

## Flip conditions status (none met)
- **s43 r1 fold:** verified (sha match, lag 2.13 / 1.76, no flags).
- **No r1 collapse:** s43 r1-lag 49 vs ctrl 42 falls is inside the noise band, and err improved.
- **No final-pick decision exists yet** (`decisions/README.md`).
- **No training-code change** between the launches.

## Cost of reopening now
- **State:** all 10 s43 legs have launched. 8 are done, and ctrl-1500 and r1-cont-1500 are at iteration ~4471 of 4497. 8 of 10 have suite+push evals.
- **Redo as B:** about 1.1 GPU-h of training (10 × 500 iterations × 0.78 s/iter solo, or about 4 h wall at about 3 s/iter while sharing the GPU), plus 2 captures and folds, about 25 min of evals, and rework of the analysis and notebook.
- **Benefit:** none in validity, by the argument above.

## Bottom line
Keep A. **New caveat for the Critical final-pick decision:** if the pick is in the s43 lineage:
- measure nominal-PD vs flat_dr falls on that actual checkpoint, on ≥ 2 eval seeds, before applying the kp-coverage branch;
- do not cite the s42 parent's 99 / 79 or the Part 3 dry runs as evidence for it.

In every lineage, rank on paired deltas and treat any single-checkpoint push_stand gap below about 0.3 as noise.
