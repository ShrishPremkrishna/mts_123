# Devil's advocate: is `gallop_c` s42 the right Part 2 parent? (retroactive review, 2026-11-18)
**Verdict: STANDS WITH NEW CAVEAT.** The parent was ranked only under Lab 4 DR. At nominal PD gains it loses 4× more falls (+48.5) than it gained over fast_trot (−13), and fast_trot has never been measured at nominal PD. Run that eval (≈15–20 GPU-min) before the final policy is selected.

## Strongest counter-case (numbers checked in JSON, not prose)
- **The deciding axis was chosen under the wrong physics.** Every Part 1 number (W2 suite, push, stock sweep) keeps the play-config DR events (`part2_eval._apply_physics_defaults`; `provenance.standard_physics: true`). c's advantage there is 42.5 vs 55.5 falls (two seeds each, `part1/eval/w2/compare_all.csv`). At nominal kp = kd = 1.0×, c s42 falls 99 / 79 against 40 / 41 under DR (eval seeds 1234 / 4321; `part3/eval/p3-parent_kp_coverage.md`, `p3-parent/ablate/*pd_gains*.json`). Its sat@1.5 also rises from 17.9 % to 23.0 % and it overshoots to ±1.63 m/s. That penalty (mean +48.5) is about 4× the margin that decided the pick. If fast_trot's penalty is even ~15–20 falls smaller, the ranking under deployed gains flips. Nobody knows, because fast_trot was never run at nominal PD.
- **The only metrics that separate cleanly beyond seed and repeat noise split 2–1 against c, and the decision ranked them out.** All arms are matched at 3000 iterations and 4096 envs, so the comparison itself is fair.
  - sat@1.5: 0.179 / 0.176 vs 0.126 / 0.124 (c is 43 % higher).
  - All-alive sat: 0.067 / 0.065 vs 0.043 / 0.043.
  - τ²: 1.35 / 1.37 vs 1.02 / 1.02.
  - Within-arm spread on all three is ≤ 0.003; on these c is worse.
  - Tracking (err ±1.5 0.132 vs 0.173) is the clean separator in c's favour.
  - The ranked metrics that decided the pick are the noisy ones:
    - falls: a 13-fall gap, against repeat evals of one c s43 checkpoint that span 33–43 (`~/wt/w3/.../part2/eval/w3{,-rep}/p1-gallop_c-s43*.json`);
    - push_stand@0.65: within-arm spread 0.44 (c) and 0.37 (ft), larger than the 0.27 gap;
    - F_runmax: 3.06 vs 3.18 BW, ≈ 4 %.
- **Torque headroom is plausibly what nominal PD exposes.** Higher kp means bigger torque requests, and c is the arm with the least headroom: its reference kp·Δq is 3.82 Nm vs fast_trot's 2.89 Nm, and its saturation is already the highest. So "a recipe-level issue that fast_trot surely shares" is a prior, not a measurement.
- **The 16-segment sequence is a tie, not a c loss or a c win.** Failures were ft 2 + 4 and c 6 + 0 (s42 + s43, `stock/*/seq_metrics.json`). The graders cited only the s42 numbers (2 vs 6).

## What the graders missed or under-weighted
- **Nominal-parameter robustness** was never a criterion. All three graders treated "Part 3 DR / torque-limit tests" as a future flip, but no grader asked for a DR-off eval of the two finalists, which would have cost about 2 minutes per checkpoint.
- **Saturation was demoted to "unranked"** even though it was the one sim-to-real readout that reproduced across seeds, and REPRO_PLAN §3 lists it as a sim-to-real readout.
- **The "irregular 4.2–4.5 Hz cycle" flag against c was an artefact.** Part 2 round 0 (`part2_notebook.md` §2) shows c's joints lock to the 3.03 Hz clock, with no power at 4.2 Hz; contact-edge double counting produced the 4.2 Hz figure. The same contact classifier produced fast_trot's "regular, clock-locked" label, so neither side's footfall-regularity argument should have carried weight.

## Flip conditions, status now
1. **fast_trot two-seed mean 10–15+ falls below c: NOT met.** It is 55.5 vs 42.5, i.e. it goes the other way.
2. **c capture fails periodicity / residual / limits, or kp·Δq ≫ 4 Nm: NOT met.** The fold is clean, with residual 0.09–0.10 and kp·Δq 2.2–2.5 Nm (`part2_notebook.md` §2).
3. **Round 1 raises sat@1.5 or falls beyond noise: NOT met.**
   - r1-lag vs ctrl-500: sat@1.5 0.167 / 0.166 vs 0.180 / 0.183, falls 27 / 49 vs 29 / 42.
   - By r3 and r2-cont-1500, sat@1.5 is 0.15–0.16 and falls are 15–23 (W3 evals, not yet in the notebook).
4. **"Part 3 DR / torque-limit tests show c degrading where fast_trot does not": HALF met.** c degrades sharply at nominal PD, but the fast_trot half is unmeasured, so this condition can be neither confirmed nor cleared without one eval.

## Cost of reopening
- **Cheap partial reopening (recommended, no rework).**
  - Command: `part3_eval ablate --events pd_gains` on fast_trot s42 and s43 at seeds 1234 and 4321, plus `part3_eval run --conditions flat_dr --modes suite` at the same two seeds, for a harness-matched Δ. That is 8 suite evals.
  - Adding c s43 for parity (its nominal-PD number is also unmeasured) brings it to 12.
  - Each takes about 36–70 s (`ablate/*.json` `wall_clock_s`), so ≈ 15–20 GPU-min solo, 0.3 GPU-h at most, and ≈ 30–45 min of wall clock while the GPU is shared.
  - Pre-register the rule before running. Re-open only if fast_trot's two-seed (nompd − flat_dr) Δ is ≤ 15, the kp-coverage KEEP threshold, while c's stays > 15, **and** fast_trot's nominal-PD falls are below c's by more than ~20 (c's own eval-seed spread is 99 vs 79).
- **Full reopening now.**
  - Part 2 already has 21 legs on c: about 10.4k iterations, i.e. ≈ 2.3 GPU-h solo or ≈ 8.7 h at 3 s/iter under concurrency.
  - It also has ~28 suite + push evals, 4+ captures, the fold checks, and the Part 3 parent dry run with its 4 smokes.
  - Redoing all of that on fast_trot costs ≈ 3.5–4 GPU-h, ≈ 1 day of wall clock while sharing, and a rewrite of the whole Part 2 notebook plus Part 3 §4.
  - It would also throw away a Part 2 that is passing its own gates.

## Bottom line
On the evidence the graders had, the unanimous pick was defensible. The fold risk they feared turned out to be an artefact, and Part 2 on c is improving sat@1.5 and falls. It was not groupthink so much as a shared blind spot: all three ranked on DR-on falls and treated torque headroom and nominal-gain robustness as "later". The new nominal-PD result makes that blind spot the largest single number in the comparison. The decision stands, but with this caveat: **"the parent was chosen without any nominal-PD evidence; c loses 48.5 falls at nominal gains (≈ 4× its DR-on margin) and fast_trot is unmeasured."** Close it with the ≈ 0.3 GPU-h eval above, before W4's kp branch and the final selection, not after. Kp-WIDEN is gait-agnostic, so it is the mitigation either way.
