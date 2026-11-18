# Default fold configuration for the Part 2 bootstrapping rounds
Status: DECIDED by grader 2 of 3 (pending consolidation) · 2026-11-18 · Requested by: W2 (fold tooling, feeds W3 rounds)
Criticality: High. It gates every Part 2 round; a wrong default costs all r1–r3 legs (> 3 GPU-hours) to redo.
Reversible?: yes, but only by refolding (about 30 s CPU) and retraining every leg trained on the bad reference.

## Conclusion
Default: **`lag` with a fixed `--lag-steps 1.8`** (= `GAIT_REWARD_PHASE_LEAD_STEPS`), with clock alignment, median reduction, 6 harmonics and **no symmetrization**. Keep the **back-abduction pin on** (`leg_back_r_2,leg_back_l_2`). Pin nothing else by default. Front abduction (`leg_front_*_2`) stays unpinned but must be gated (see the flip rules). Also run the `auto` fold on every capture as a diagnostic only. **Comparison arm:** in round 1 only, run `naive` with the same pin at s42. This is the recovered r1 design (naive vs lag). Rounds 2–3 run the default alone, next to the mandatory control arm.

## Evidence used
- `report/part2/fold_variants.md`: RMS diff vs parent is naive 0.241/0.195 against lag 0.148/0.105 rad (fwd/back). The lag fit cuts the zero-shift RMS from 0.233 to 0.134 and from 0.176 to 0.064. Measured lag is 2.18/1.94 steps against the 1.8-step reward lead. Dispersion and wrap are identical for naive and lag.
- `mixed_gaits.py` l.146–157 and `pupper_gait_env_cfg.py` l.224–241: the reward grades q(k) against ref(clock − 1.8 steps). Folding with shift = 1.8 therefore makes the captured behaviour an exact reward fixed point (newref(p) = q(p + 1.8)). A naive fold stores the gait about 2 steps late, and the delay compounds each round.
- My CPU-only refold of `/home/ubuntu/runs/w2/capture-cont4500/trace.npz` with `fold_direction` (no pin):
  - **fixed 1.8 ≈ auto**. RMS vs parent is 0.157 vs 0.154 forward and 0.142 vs 0.142 backward, so the choice between them is only about robustness.
  - **Unpinned back abduction** moves by −0.069/+0.095 rad forward and −0.229/+0.232 rad backward, on a parent range of 0.006 rad. That is 11–40× its range: the crouch-collapse signature.
  - **Front abduction** moves by −0.16/+0.15 rad forward and −0.22/+0.16 rad backward, on a parent range of 0.147 rad. That is 1.1–1.5× its range. It is a level shift, and the captured range (0.11–0.20) is similar to the parent's.
- The actor observation includes the 12-D reference pose (`pupper_gait_env_cfg.py` l.282, "36 + 12 = 48"). A shifted reference channel is therefore a direct out-of-distribution input, which is the mechanism behind the recovered crouch collapse (`REPRO_PLAN.md` §2.3).
- `part2_reference.py` `measure_mirror_shift`/`mirror_pairs` use **one** pair offset for both the front and back pairs. The test covers only a 0.5 (trot) offset.
- The `phase` variant cannot fold forward on this parent (feet land every ~8 steps vs the 37.5-step cycle), and its backward dispersion rises from 0.045 to 0.071.

## Thought process
The criteria, in priority order:
1. **Do not break the warm start.** A reference must not push the actor observation out of distribution (§3: "captured references are data: sanitize").
2. **Consistency with the reward's timing**, so that the rounds measure gait-shape bootstrapping and not a phase artifact.
3. **Fidelity to the recovered design**: r1 compared naive vs lag, all final rounds used `lag_m2`, and the back pin was the recovered fix.
4. **Robustness on an unknown real parent** (W1's pick is pending and was a rotary gallop last time).

`lag` wins criteria 2 and 3. Naive mis-times the reach slots by about 5 % of a cycle each round, which also desynchronizes them from the `trot`/`lift` slots at the mode seams that Part 3 tests.

Fixed 1.8 beats auto on criterion 4 for three reasons. On this capture the two give the same table. Fixed 1.8 is defined by the reward and not by a cross-correlation against the parent's *shape*, which gets ill-posed when the policy does not step on the clock (as forward here) or when the gallop shape diverges. And it cannot fail at the search edge. Auto stays as a cross-check.

`sym` brings almost nothing here (RMS 0.147 vs 0.148; asymmetry 0.04 rad), and it is structurally wrong for a rotary gallop. With footfall LH→RH→RF→LF, the hind and fore pairs lead on opposite sides, so no single mirror shift fits both. Mirror plus time shift does not map a rotary gallop onto itself.

Back pin: the data confirms the recovered fix. Weakly, `lag_m2` may even mean "lag + motor-2 pin".

Front pin: the shift is of the same order as the channel's own motion, and the channel really varies. Pinning it would also suppress a real captured feature. So it is gated rather than pinned.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| `naive` default | Stores the gait about 2 steps late against the reward lead, compounds over rounds, and has RMS 0.24 vs 0.15. Kept only as the r1 comparison arm |
| `lag` auto | Gives the same table here, but depends on a parent-shape fit that can be ill-posed or hit the search edge on a new gallop. Used as a diagnostic |
| `phase` | Refuses or falls back forward when the policy is off the clock. Backward dispersion is worse (0.071 vs 0.045) |
| `sym` | Marginal gain on a trot. A single shared pair offset is invalid for a rotary gallop parent |
| Also pin front abduction | Shift is ~1× its natural range, not ~10×. Pinning hides a real captured posture. It becomes a fallback (see the flip rules) |
| No pin | Back abduction moves 11–40× its range: this reproduces the known crouch collapse |

## Severe flagged issues
- **All the evidence comes from a smoke parent** (stock-trot fast slot). It steps at ~6 Hz at +1.5 m/s, off the 1.33 Hz clock, so the forward fold is a smeared median. It reaches only −0.149 m/s at −1.5 m/s, with 17 falls. Repeat the variant table and the flag check on the real W1 parent before r1.
- **Never run `sym` on a rotary-gallop parent** without a per-pair offset. As implemented, it would average incompatible pairs.
- **Front abduction is unpinned but flagged** in both directions. The splay metric is ranked, and bootstrapping may reward splay. Report it as a result, not as noise.
- The meaning of `lag_m2` is unknown, so fidelity to the old final rounds is not guaranteed.
- The naive arm is one seed only. Gaps under ~10 % or ~10 falls are ties (§3).

## What would flip this decision
- On the real parent, any unpinned channel moves by more than 2× its parent range, or the r1 lag leg shows a crouch or splay signature within the first ~200 iterations (base height falling, front `_2` drifting, splay worse than control by >10 %). Then refold with the front-abduction pin and restart r1.
- `auto` differs from 1.8 by more than ~1 step on the real parent, or its residual stays close to its zero-shift RMS. Investigate the clock and capture alignment before training.
- The r1 naive arm beats lag by more than the noise bands on tracking and falls. Then re-decide for r2.
- The real parent is a trot-like gait with measured asymmetry above ~0.1 rad. Then reconsider `sym`.
