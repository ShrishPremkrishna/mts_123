# Default fold configuration for the Part 2 bootstrapping rounds
Status: DECIDED by grader 1 of 3 (pending consolidation) · 2026-11-18 · Requested by: W2
Criticality: High. It sets the reference for every Part 2 round (r1–r3 plus continuations), and a collapsed round costs > 3 GPU-hours to redo.
Reversible?: The fold itself is cheap to redo (~27 s CPU per variant). Rounds already trained on a bad reference are not: each 500-iteration leg × arms × 2 seeds has to be rerun.

## Conclusion
Default: **`lag` with a fixed `--lag-steps 1.8`** (= `GAIT_REWARD_PHASE_LEAD_STEPS`), median reduction, 6 harmonics, and
the pin **widened to all four motor-2 (abduction/roll) channels**: `leg_back_r_2, leg_back_l_2, leg_front_r_2, leg_front_l_2`.
Log the auto-measured lag as a diagnostic, not as the shift. Run **one fold comparison arm, `naive` (same pin)**, in r1 only,
as the recovered plan does (r1 naive vs lag). It does not replace the mandatory no-new-reference control. Do not run `phase` or `sym` as arms.

## Evidence used
- `report/part2/fold_variants.md`, smoke capture (cont4500, stock-trot parent). Lag RMS vs parent is 0.148 / 0.105 rad, against 0.241 / 0.195 for naive and phase.
  The lag-fit residual is 0.134 vs 0.233 zero-shift (fwd) and 0.064 vs 0.176 (back). Auto lag is 2.18 / 1.94 steps, so 1.8 is within 0.4 step (about 1 % of a 37.5-step cycle).
  Sym ≈ lag (0.147 / 0.103). Phase refused forward because the policy steps every ~8 steps, not every 37.5, so it fell back to the clock. Backward phase dispersion rose from 0.045 to 0.071 rad.
- `pupper_gait_env_cfg.py:225-241` and `mixed_gaits.py:146-162`: the actor observes ref(k) with zero lead, and gait_tracking grades it against ref(k − 1.8).
  A policy that tracks perfectly therefore sits at q(t) = ref(clock − 1.8). With the fold shifted by exactly 1.8, that behaviour is a **fixed point**.
  Naive adds about 2 steps of delay every round, and that delay compounds across r1→r3.
- `diff_report.md` (all four variants): `leg_front_{r,l}_2` is flagged with the crouch-collapse signature (`flag_near_constant_shift`).
  Its mean offset is −0.160 / +0.148 rad (fwd) and −0.220 / +0.164 rad (back), larger than the parent's whole range of 0.147 rad. Pinning does not touch it.
- `C_lineage_metrics.md` §3a: every unpinned round 1 collapsed whatever the alignment.
  - The fast_long rounds (naive, phase, sym, sym-protect) ended at reward 80–82 and err_xy 0.76–0.79, against the control's 165 and 0.229.
  - The gallop rounds (naive, lag; `p2g-r1-*`) ended at reward 74–77 and err_xy 0.77–0.82.
  - Only `p2g-r1b … lag_m2` recovered, at 160 and 0.235. Alignment never rescued a run; the pin did.
- `A_code_recovery.md` §capture variants: the final rounds all used `lag_m2`, whose semantics were not recovered. `roll_asymmetry` in `mixed_gaits.py` calls `_2` "motor 2".
  So **m2 = motor 2 pinned** is a plausible reading [I, moderate]. That reading is consistent with "lag + abduction pin", but it does not tell us whether the front channels were pinned too.

## Thought process
Priority 1 is to avoid another collapse, because a collapse gates the whole workstream; §3 says "pin near-constant channels".
Priority 2 is a principled, round-stable alignment.
Priority 3 is the fewest arms ("cut arms, not iterations") with a control always present.
- **Alignment.** Lag is the only variant consistent with how the reward is defined, and the data confirm it: the auto lag ≈ 1.8 and the residual roughly halves.
  I prefer a fixed 1.8 over auto:
  - It makes the parent's tracked motion an exact fixed point.
  - It is the same constant in every round and arm, so the auto value is not a capture-dependent confound.
  - Auto is a cross-correlation against the *parent*. It therefore pulls the timing back toward the parent, which defeats bootstrapping.
  - Auto can also hit the search edge, or fit a smeared cycle (as at +1.5 here), on a gallop parent whose shape differs.
- **Phase** depends on the policy stepping on the clock, which it did not do at +1.5. It adds dispersion and keeps the naive delay.
- **Sym** assumes a fixed L/R pair offset. The likely parent, gallop C, is a rotary gallop (LH→RH→RF→LF), so a mirror average would distort it. `roll_asymmetry` and `abduction_angle` already police symmetry through the reward.
- **Pin.** Keep the back-abduction pin; it is the recovered fix and has no downside.
  Extend it to the front `_2` channels, because the flag fires on them in every variant with the exact collapse signature.
  Pinning them costs little:
  - The policy already deviates from the parent's front `_2` under the parent reference, so a pinned channel leaves its current behaviour unchanged.
  - Splay is penalised toward default (`abduction_angle` on `leg_.*_2`), so a moved `_2` reference would fight that reward anyway.
  - If the real parent does not shift front `_2`, the pin is a no-op.
  The `_1` and `_3` channels carry the gait and must stay free; that is the point of bootstrapping.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| naive | Adds about 2 steps of delay per round (RMS vs parent 0.241 vs 0.148), and the delay compounds. Kept only as the r1 comparison arm, because the recovered plan compares naive vs lag |
| lag (auto) | Nearly identical table on this capture. It is capture-dependent, anchored to the parent, and can fail on a gallop. Keep it as a logged diagnostic |
| phase | Refuses or falls back when the policy does not step on the clock, raises dispersion 0.045→0.071, and keeps the naive delay |
| sym | ≈ lag on a trot. It would impose L/R mirror structure on a rotary gallop |
| back-only pin (recovered) | Leaves the flagged front `_2` shift (> parent range) in the reference: the same mechanism that collapsed every unpinned r1. It is a fallback only if the front pin is shown to hurt |
| no pin | Every unpinned round 1 in the old lineage collapsed (reward ~75–82 vs ~160) |

## Severe flagged issues
- The evidence comes from a **smoke parent**, not W1's gait. That parent stepped at about 6 Hz against the 1.33 Hz clock at +1.5, so its clock fold is a smeared average (dispersion 0.068).
  At −1.5 it had 17 falls and vx −0.149, so it was barely walking. **Rerun the fold on the real parent before r1.** Check three things:
  - touchdown cadence ≈ cycle steps;
  - |auto lag − 1.8| < 1 step;
  - which channels are flagged. Pin any newly flagged near-constant channel.
- The front-`_2` pin is an extrapolation, not a recovered fact. The recovered fix names back abduction only, and `lag_m2`'s exact pin set is unknown.
- `GAIT_STD`'s comment calls `_1` "abduction", but the FK note in `roll_asymmetry` says `_2` is the lateral joint. The pin targets `_2`, which is correct by both the FK note and the parent ranges (0.006 rad for back `_2`).

## What would flip this decision
- On the real parent the auto lag differs from 1.8 by more than ~1 step, or the lag fit's residual does not drop. Re-examine the alignment, and use auto if the policy's real latency differs.
- r1 shows naive ≥ lag on the sim-to-real readouts beyond noise (> ~10 falls or ~10 %). Then the alignment choice does not matter, and naive is simpler.
- The front-`_2` pin is shown to block a real improvement (for example, splay or heading drift is better without it in a matched arm), or the real parent shows no front-`_2` shift. Then revert to back-only.
- W1's parent turns out to be a symmetric trot rather than a rotary gallop. Then sym becomes a reasonable alternative.
