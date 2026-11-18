# Default fold configuration (variant, lag, pins) for the Part 2 bootstrapping rounds?
Status: DECIDED by grader 3 of 3 (pending consolidation) · 2026-11-18 · Requested by: W2 (Part 2 tooling) for W3
Criticality: High. Every Part 2 round (r1–r3, s42 + s43) installs a reference folded this way, and a wrong default compounds across rounds. Redoing them costs > 3 GPU-hours.
Reversible?: Before r1 launches, yes, for free (a fold takes ~26 s of CPU). After that, each round has to be redone: 500 iterations × 2 seeds ≈ 21 min per round, plus every later round built on it.

## Conclusion
**Default = `--variant lag --lag-steps 2` (a fixed 2.0 control steps, not 1.8 and not auto), median reduction, 6 harmonics, no symmetrize. Pin all four abduction channels** (`leg_back_r_2,leg_back_l_2` as recovered, plus `leg_front_r_2,leg_front_l_2`). Rule for each real capture: also pin any channel that the diff report's near-constant-shift flag hits, or justify it in writing. Run auto-lag as a diagnostic only, and stop if it is more than 0.5 steps from 2.0.
**Comparison arms, in r1 only:** (a) `naive` with the same pins, at s42 and s43. This is the recovered r1 design, and the only clean test of the lag hypothesis. (b) `lag` with the back-only pin (the recovered recipe), at s42 only, as an ablation of the new front pin. r2 and r3 run the default plus the control.

## Evidence used
- `mixed_gaits.py:146–149`: `mixed_gait_tracking` rewinds `episode_length_buf` (a `torch.long`) by `(k − 1.8).to(long)`, which truncates to **k − 2**. I checked: `[2,10,37] → [0,8,35]`. Single-gait `gait.py:189` uses a float 1.8, but the MixedGaits reward that grades the reach tables uses an **effective 2.0-step lead**. `pupper_locomotion.py:275` records phase and joint_pos at that same buffer value, before `step`, so a perfect tracker folds as `ref(p − 2 steps)`.
- `fold_variants.md`: the measured auto-lag is **2.18 / 1.94** steps (fwd/back), which brackets 2.0. Correcting the lag removes most of the parent diff (RMS **0.241→0.148 / 0.195→0.105** rad), and the lag-fit residual falls from 0.233 to 0.134 / 0.176 to 0.064. A naive fold re-adds ≈ 2 steps per round, about 5 % of a 37.5-step cycle per round and more on a faster gallop clock.
- `phase`: forward had to fall back to the clock, because the feet land every ~8 steps rather than 37.5. Backward dispersion got worse (**0.045 → 0.071**). It adds no lag correction.
- `sym`: RMS 0.147/0.103 vs lag 0.148/0.105, so no measurable gain. `mirror_pairs` applies **one** pair shift to both the front and back pairs, which is valid for a trot (offset 0.5). A rotary gallop (recovered pick C: LH→RH→RF→LF) has opposite lead orders in the front and back pairs, so mirror averaging would smear its footfall sequence.
- Unpinned abduction, read from `~/runs/w2/capture-cont4500/trace.npz` (steady samples): back `_2` mean **−0.07/+0.10 rad fwd and −0.23/+0.24 rad back**, against a parent range of 0.006. The pin is clearly needed. Front `_2` is shifted **−0.16/+0.15 fwd and −0.22/+0.16 back** against a parent range of 0.147. It is flagged in every variant, and it is the same crouch-collapse signature.
- `C_lineage_metrics.md`: the old r1 naive and r1 lag runs (no pin) both collapsed, to reward ≈ 74–75 vs ≈ 160 for r1b `lag_m2`. So the old naive-vs-lag comparison was confounded, and no recovered evidence on lag exists.

## Thought process
Criteria: (1) do not inject known artifacts or out-of-distribution reference inputs into a warm-started policy that observes the reference (§3, "captured references are data: sanitize, pin near-constant channels"). (2) Keep the reference faithful to the policy's real motion, which is the point of bootstrapping. (3) Make it reproducible across rounds and seeds. (4) Follow the recovered design where it is known (§2.3, W3 r1 naive vs lag).
- The 2-step delay is an artifact of the reward's own definition, so subtracting exactly that and nothing else satisfies (1) and (2).
- Auto-lag fits the timing *toward the parent*. That absorbs genuine timing changes, depends on a cross-correlation that is weak when the policy is off the clock (forward at +1.5), and can vary by seed or round. The fixed value is deterministic, and the two differ by ≤ 0.2 steps here, so nothing is lost.
- 1.8 is the *nominal* constant, but it under-corrects the code that actually runs by 0.2 steps per round.
- The front-abduction pin keeps that channel at status quo: the policy is already graded against the parent's front abduction. The splay penalty (`abduction_angle`, joints `leg_.*_2`) already pulls toward default, and a flagged shift larger than the channel's entire range is exactly how round 1 collapsed. The cost is losing a 0.15 rad posture change that bootstrapping is not trying to teach. Arm (b) measures that cost cheaply.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| naive | Adds ≈ 2 steps of delay per round; 0.09 rad more RMS diff vs parent, all of it timing artifact. Kept as an r1 arm only. |
| lag, auto | Equivalent here (2.18/1.94), but it biases timing toward the parent, is fragile off-clock, and is non-deterministic across seeds. Keep it as a diagnostic. |
| lag, fixed 1.8 | Does not match the effective rewind of `mixed_gait_tracking` (2 steps), so 0.2 steps of under-correction per round. |
| phase | Unusable forward (off-clock policy), worse dispersion backward, no lag correction. |
| sym | No gain on a trot parent. Its single-shift mirror is wrong for a rotary gallop parent. |
| back-only pin (recovered) | The front `_2` shift (0.15–0.22 rad > its 0.147 rad range) trips the same flag. Kept as r1 ablation arm (b). |
| no pin | Back `_2` would move 0.07–0.24 rad against a 0.006 rad range: the recovered crouch collapse. |

## Severe flagged issues
- **Evidence is from a smoke parent** (stock trot, which steps off-clock at +1.5 with 17 backward falls), not W1's Part 1 pick. Re-check lag, flags and the abduction shifts on the real parent's capture before r1.
- The 2-step effective lead comes from integer truncation in `mixed_gait_tracking`. If anyone "fixes" it to a float, the fixed lag must become 1.8. The lag value should be derived from the code, not copied.
- If the real parent is a gallop with `frequency_mult ≠ 1`, the lag stays 2 steps but becomes a larger fraction of the cycle, which makes naive drift worse.
- Arm (b) is one seed. Read it against the ~10 % / ~10-fall noise floor.

## What would flip this decision
- On the real parent capture, auto-lag differs from 2.0 by more than 0.5 steps, or hits the search edge. Then investigate before choosing auto or fixed.
- r1 shows naive ≥ lag on tracking and falls at both seeds. Then the lag correction is immaterial and either is fine.
- Arm (b) beats the default on tracking or falls beyond noise, with no collapse. Then drop the front pin.
- The real parent is a trot-family gait with a residual L/R asymmetry above ~0.1 rad (splay or roll). Then reconsider `sym` as an arm.
