# Which Part 1 arm is the provisional Part 2 parent, and which two arms get seed 43?
Status: DECIDED by graders (3/3 agree) (pending human review) · 2026-11-18 · Requested by: W1
Criticality: High. Every Part 2 capture, fold and warm-start descends from this parent, and redoing Part 2 costs more than 3 GPU-hours.
Reversible?: yes. Before Part 2 round 1 starts, reversing costs nothing. After that it costs the rounds built on the wrong parent (about 3 GPU-hours at 2 seeds) plus a re-capture and re-fold.
Grader docs: [grader-1](2026-11-18-p1-parent-gait/grader-1.md), [grader-2](2026-11-18-p1-parent-gait/grader-2.md), [grader-3](2026-11-18-p1-parent-gait/grader-3.md). All three pick the same parent and the same top 2.

## Conclusion
**Provisional parent: `gallop_c` seed 42** (WandB `1c5r0n8c`, `~/runs/opt-lab-2/rp-p1-gallop_c-s42/model_2999.pt`, reference
`configs/opt-lab-2/gallop_c` @ `e01364f`). **Top 2 for seed 43: `gallop_c` and `fast_trot`.** gallop_c s43 (`y38xkkvh`) already
exists, so the only new run is `fast_trot` s43. It was launched by W1 at 16:24 UTC on 2026-11-18 (WandB `pgsoywpz`, launch commit `512eb0c`).

## Evidence used (shared by all three graders)
- Eligibility gate: Part 2 captures at ±1.5 m/s, so the parent must run both ways. gallop_a and gallop_b move *forward* when
  commanded −1.5 (vx +0.29 / +0.41, err_xy 1.79 / 1.91). baseline tops out near 0.2 m/s. Only fast_trot (+1.36 / −1.46) and
  gallop_c (+1.55 / −1.48; s43 +1.53 / −1.48) qualify (`report/part1/eval/stock_table.json`, `eval/w2/compare_all.md`).
- Ranked W2 readouts, fast_trot s42 vs gallop_c s42 / s43: falls 53 vs 44 / 41 (reversal 8 vs 2 / 6); F_runmax_med 3.17 vs
  3.03 / 3.09 BW; push_stand@0.65 0.681 vs 0.453 / 0.009; push_move@0.5 0.006 vs 0.002 / 0. Every ranked gap is inside or
  near the noise band. c leads on all of them, consistently across its two seeds.
- Unranked readouts: err ±1.5 0.174 / 0.176 vs 0.130 / 0.146; sat@1.5 0.126 vs 0.179 / 0.176; τ² 1.02 vs 1.35; hind splay +
  −0.13 vs +0.04 rad; drift is similar (45–68 deg/10 s, heading hold off).
- c also matches the recovered selection logic (REPRO_PLAN §2: it runs backward, keeps 2 or more feet down in the design, has fewer reversal falls, and was the old parent),
  which keeps Part 2 comparable with the recovered numbers.

## Severe flagged issues (union of the grader docs)
- **3 Nm saturation:** c saturates 17.9 % of samples at ±1.5 (21 % at −1.5), against 12.6 % for fast_trot, and its reference already
  exceeds the kp·Δq proxy (3.82 Nm). Part 2 must report sat@1.5 for every round and must not accept a round that raises it. This is the
  largest sim-to-real risk of the pick (G1, G2, G3).
- **Capture and fold risk:** at ±1.5 c steps at 4.2–4.5 Hz against a 3.0 Hz reference clock, and its cycle is irregular (lag
  concentration down to 0.18; 4.7 % zero-feet time at +1.5). A clock-phase fold may smear it. Check the fold diagnostics
  (periodicity, dispersion, lag fit, joint limits, kp·Δq) on the c capture **before** round 1 (G1, G2, G3).
- **Seed asymmetry and noise:** fast_trot has one seed and c has two. GPU rollouts are not bit-reproducible (fall gaps under 10–15 are ties).
  push_stand@0.65 swings from 0.453 to 0.009 between c's seeds, so it must not decide anything alone (G1, G2, G3).
- **Weights:** every arm uses `splay.json`, so the old baseline and fast_trot numbers (bare profile) are not weight-matched comparators (G1).
- **Commit check (G3):** c s43 launched at `a042c72`, s42 at `2e7f5b9`. Checked by W1: `git diff 2e7f5b9 a042c72 -- pupper-mjlab/src`
  adds only `src/mjlab/scripts/part2_reference.py` (W2's capture/fold CLI, not imported by training), and the reference blobs are identical.
  The two seeds are valid to pool.

## What would flip this decision
- fast_trot s43 lands, and its two-seed mean is 10–15 or more falls below c's two-seed mean (42.5), with foot force and standing push no worse.
- The gallop_c ±1.5 capture fails its periodicity, fold-residual or joint-limit check, or its folded kp·Δq rises well above ~4 Nm,
  while a fast_trot capture folds cleanly.
- Part 2 round 1 on c raises sat@1.5 or falls beyond the control by more than the noise band, or collapses despite the abduction pin.
- Saturation is promoted to a ranked metric, or Part 3 DR / torque-limit tests show c degrading where fast_trot does not.
