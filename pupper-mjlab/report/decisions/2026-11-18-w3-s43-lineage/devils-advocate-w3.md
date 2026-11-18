# Devil's-advocate review: seed-43 lineage from the s43 parent (A) vs the s42 parent re-seeded (B)
Status: retroactive review (README step 2, lens 1) · 2026-11-18 · Front-runner reviewed: **A** (as decided, 2/2 graders)
Verdict: **A stands.** Two reading rules change (see the last section).

## Strongest case against A
1. **"Seed" now means parent + capture + folded refs + RNG, all at once.** When the two seeds disagree, A cannot tell parent dependence from fine-tune noise. Disagreements exist:
   - falls: r1-lag −2/+7 and r1-cont-1000 −9/+12;
   - F_runmax_med: r1-naive, r2 and r1-cont-1000 (r2 +0.12 / −0.01 BW);
   - drift− at +1000.

   B would have isolated the fine-tune noise on the designated parent, and that variance is what limits the decision of which arm to ship.
2. **Parent-relative claims cannot replicate under A.** The parents' standing push is genuinely different, not eval noise: R:push_stand@0.65 is 0.455 and 0.470 for s42 (eval and repeat) against 0.009 and 0.006 for s43. The decision's own fallback was to compare stand-still against the parent, and that comparison flips sign by construction. r3 minus parent is −0.05 at s42 and +0.33 at s43.
3. **Provenance drift.** A Part 2 final from the s43 lineage descends from `y38xkkvh`, not from the parent the Critical Part 1 decision designated (`1c5r0n8c`). The s43 parent's distinctive trait does not survive Part 2: its own control is at 0.547 push_stand by +500. Choosing s43 would bring in a different parent and nothing to show for it.

## What the data now say (`part2_analysis.py matched`, run into a scratch dir; chain check passed for 21 arms)
- **The parent offset washes out by +500.** On push_stand the controls are 0.568 / 0.547 at +500 and 0.498 / 0.497 at +1000. On err±1.5 they are 0.120 / 0.123 and 0.120 / 0.125. The 0.44 parent gap, the main risk A carried, does not reach the matched comparisons.
- **The deltas reproduce across parents on everything above noise.** At +500 and +1000, all four arms agree in sign at both seeds on:
  - err±1.5: −0.006 to −0.017;
  - sat@1.5: −0.006 to −0.033;
  - push_stand: worse in every arm, −0.04 to −0.55;
  - drift+ at +1000: −4 to −12.

  Every sign disagreement is a fall tie (|Δ| < 15) or a small change in F_runmax or drift−. That replication survived a change of parent, capture and fold. B could only have shown that it survives a change of RNG, so A's evidence is strictly stronger.
- **Parity holds.** The s42 and s43 legs ran at heads `1611a75` to `a898f59`. The diff between them touches only analysis and Part 3 scripts, plus an opt-in `--checked-bumpy` launch flag that Part 2 does not use. There is no training-code change.
- **Missing:** the `p2-s43-ctrl-1500` and `p2-s43-r1-cont-1500` evals (trained at 20:44, evals pending) and the push eval for `p2-s43-r2-cont-1500-rep2`. Every +1500 delta (r3, r2-cont-1500, r1-cont-1500) is therefore s42-only (n=1).
  - Arm-vs-arm at +1500 already agrees across seeds. r3 is worse than r2-cont-1500 on standing push: 0.407 vs 0.099 at s42 and 0.334 vs 0.148 at s43.
  - Falls tie: 23 / 16 for s42 and 16 / 15 for s43. The s43 r3 repeat gave 29, so one-eval falls swing about 13.

## Verdict and consequences for reading the s43 lineage
**Stands.** Objection 1 costs something only on readouts that are already ties. Objection 2 is an argument against parent-relative comparisons, not against A. Objection 3 is a provenance note, not a defect. Reversing now would cost about 1.1 GPU-hours and would give a weaker replication.
- **Read s43 only as same-seed deltas or within-seed arm-vs-arm comparisons.** Never pool levels across seeds. **Drop** parent-relative push claims instead of caveating them. Use the matched controls, which now exist at every iteration.
- **A sign disagreement means "not robust to the parent"**, not "noise". Do not attribute it either way.
- **Report +1500 as single-seed until the s43 ctrl-1500 eval lands**, then re-run `matched`.
- **If the final comes from s43, name `y38xkkvh`.** The s43 parent's push_stand of 0.009 is not a reason to prefer that lineage, because no descendant keeps it.

## What would change my mind
- The s43 ctrl-1500 eval makes a +1500 delta on err±1.5, sat@1.5 or push_stand disagree in sign with s42, and the cause traces to the s43 r2/r3 captures. That is a capture artefact, which B would partly have avoided.
- The two lineages' choice between the leading candidates (r2-cont-1500 vs r3) hinges on a readout where they disagree, so attribution matters for the pick. A small B-style check would then settle it: r1-lag plus ctrl-500 from `1c5r0n8c` at seed 43, about 0.25 GPU-hours.
- The Critical final-pick decision requires descent from `1c5r0n8c`.
