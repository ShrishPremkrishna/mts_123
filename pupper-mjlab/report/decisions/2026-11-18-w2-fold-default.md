# Default fold configuration (variant, lag, pins) and comparison arms for the Part 2 bootstrapping rounds
Status: DECIDED by arbitrator (pending human review) · 2026-11-18 · Requested by: W2 (Part 2 fold tooling, feeds the W3 rounds)
Criticality: High. Every Part 2 round (r1–r3, s42 + s43) installs a reference folded this way, errors compound across rounds, and redoing the rounds costs > 3 GPU-hours.
Reversible?: Before r1 starts, yes: a refold costs about 30 s of CPU. After that, every 500-iteration leg built on a bad reference has to be retrained (about 21 min per round at 2 seeds, plus every later round).
Grader docs: [grader-1](2026-11-18-w2-fold-default/grader-1.md) · [grader-2](2026-11-18-w2-fold-default/grader-2.md) · [grader-3](2026-11-18-w2-fold-default/grader-3.md)

## Conclusion
**Default fold:**
`part2_reference fold <capture> <out> --variant lag --lag-steps 2 --pin leg_back_r_2,leg_back_l_2,leg_front_r_2,leg_front_l_2 --reduce median --harmonics 6 --no-symmetrize`
(clock alignment and `--limit-policy error` stay at their defaults).

Keep the same pin set in every round. On each real capture, also run `--lag-steps auto` into a scratch directory as a diagnostic only.

**Arms.** Round 1 runs four arms:
- the control, s42 + s43 (mandatory);
- the default, s42 + s43;
- **`--variant naive` with the same four pins**, s42 + s43. This is the recovered r1 design, "naive vs lag".
- **lag 2 with the back-only pin** (`--pin leg_back_r_2,leg_back_l_2`), s42 only. It ablates the new front pin, and it is the first arm to cut if GPU time runs short.

Rounds 2 and 3 run only the default and the control. Do not run `phase` or `sym` arms.

## Evidence used
- **Effective lead is 2 steps (checked).** `mixed_gait_tracking.__call__` sets `episode_length_buf = (saved.float() − 1.8).to(saved.dtype)`, and the buffer is `torch.long` (`manager_based_rl_env.py:232`).
  I ran this on CPU with `GAIT_REWARD_PHASE_LEAD_STEPS` = 1.8. Buffer values 0…5 map to −1, 0, 0, 1, 2, 3, and `b − rewound` = **2 for every k ≥ 2**.
  Only the first two steps of an episode use a 1-step lead, and the 3 s capture warmup excludes them.
  The single-gait `gait.py:189` keeps a float 1.8, but the MixedGaits and Mystery tasks, which grade the reach tables, use `mixed_gait_tracking`.
- **Recording convention (checked).** `env.step` runs the physics, increments `episode_length_buf` to k, and then computes the reward: q(after physics) is graded against ref(k−2).
  `pupper_locomotion evaluate` records `phase` from `episode_length_buf` and `joint_pos` together, before the next `step`, so each recorded pair is (phase(k), q(k)) for that same graded state.
  A perfect tracker therefore satisfies q(k) = ref(k−2). The fold builds the new reference at phase p from where the robot was at clock p + shift. A shift of 2 steps makes that tracker an exact fixed point.
  The fold applies `--lag-steps` in control steps (`shift = lag / cycle_steps`), so 2 stays correct under any `frequency_mult`.
- `fold_variants.md` (smoke capture cont4500), lag vs naive:
  - Auto lag is **2.18 / 1.94** steps (fwd / back). That brackets 2.0 (mean 2.06), not 1.8.
  - RMS vs parent is 0.148 / 0.105 for lag vs 0.241 / 0.195 rad for naive.
  - The lag-fit residual drops from 0.233 to 0.134 and from 0.176 to 0.064.
- `fold_variants.md`, phase and sym:
  - `phase` falls back to the clock forward (feet land about every 8 steps) and gets worse backward (dispersion 0.045 → 0.071).
  - `sym` matches lag (0.147 / 0.103). It applies a single pair offset (0.5) to both the front and back pairs.
- `smoke_cont4500_lag/diff_report.md`: `leg_front_{r,l}_2` is flagged `flag_near_constant_shift` (rule: parent range < 0.2 and |mean offset| > max(0.05, range)) in every variant.
  The offsets are −0.160 / +0.147 rad fwd and −0.220 / +0.164 rad back, against a parent range of 0.147. Unpinned, back `_2` moves 0.07–0.24 rad against a range of 0.006 (graders 2 and 3, refold of the trace).
- `C_lineage_metrics.md`, `A_code_recovery.md`:
  - The old unpinned r1 runs (`p2g-r1-naive`, `p2g-r1-bootstrap`, `p2g-r1-jitter`) collapsed or failed. Only the `*_lag_m2` rounds recovered. The meaning of "m2" was not recovered.
  - The old naive-vs-lag comparison was also confounded by weight sets (`splay` vs `splay_stand_still`). So no clean recovered evidence on lag exists.

## Thought process
Priorities:
1. Do not hand a warm-started policy an out-of-distribution reference input (§3: "captured references are data: sanitize, pin near-constant channels").
2. Align with the reward that actually runs, so that each round measures gait shape and not a timing artifact.
3. Keep every round and arm deterministic and identical, so differences are not confounded.
4. Run few arms with a control, and two seeds where the claim matters (§3; REPRO_PLAN §5 says two seeds per Part 2 arm is affordable).

`lag` with a fixed shift wins criteria 2 and 3. Auto measures the lag against the parent's shape: that pulls timing back toward the parent, varies with the capture, and fails off-clock or at the search edge.

Of the fixed values, 2 is the one the executed reward implies. Using 1.8 would store the gait 0.2 steps late in every round, and that compounds.

Pinning all four `_2` channels follows criteria 1 and 3:
- The front channels trip the same flag that encodes the collapse signature.
- A pinned channel keeps the status quo: the policy is already graded against the parent's front `_2`.
- `abduction_angle` and `roll_asymmetry` already police `_2`. Splay is a ranked sim-to-real readout, so writing the policy's splay into the reference is the wrong direction.
- A pin set chosen per round would be a cross-round confound.

The cost is extrapolating beyond the recovered back-only fix. One cheap single-seed arm (b) measures that cost.

Naive gets two seeds because "naive vs lag" is a recovered claim to reproduce or refute, and one seed cannot resolve gaps under ~10 %.

## Disagreement & resolution
| Point | Type | Graders | Resolution |
|---|---|---|---|
| Shift 1.8 vs 2.0 vs auto | **Factual** (fixed vs auto is criteria) | G1, G2: fixed 1.8. G3: fixed 2 | **2.** I verified the integer truncation and the recording convention myself (see Evidence). G1 and G2 read the nominal constant and missed the `.to(saved.dtype)` cast. All three agree that auto is a diagnostic only. |
| Pins: back only vs all four `_2` | Criteria, on agreed facts (front offset ≈ 1–1.5× its range, flagged) | G1, G3: all four. G2: back only plus a gate | **All four** (2/3). The status-quo, splay and determinism arguments outweigh fidelity to the captured front posture. G2's concern is kept as arm (b) and as a flip rule. |
| Comparison arms | Criteria (cost vs coverage) | G1, G2: naive at s42 only. G3: naive at s42 + s43, plus the back-only ablation at s42 | **G3's set.** It costs 3 extra 500-iteration legs, about 30 GPU-min. Arms are cut, not iterations: if GPU time runs short, drop arm (b) first, then naive s43. |

## Alternatives considered
| Option | Why not chosen |
|---|---|
| lag, fixed 1.8 | Off by 0.2 steps against the reward that runs, so each round under-corrects |
| lag, auto | Same table here, but pulled toward the parent and capture-dependent. Kept as a diagnostic |
| naive | Re-adds about 2 steps (≈ 5 % of a 37.5-step cycle) every round. Kept as the r1 arm |
| phase | Unusable off-clock forward; worse dispersion backward |
| sym | No gain on a trot. A single pair offset is wrong for a rotary gallop (gallop C) |
| back-only pin | Leaves a flagged front `_2` shift in the reference. Kept as ablation arm (b) |
| no pin | Back `_2` moves 11–40× its range: the recovered crouch collapse |

## Severe flagged issues
- **All evidence comes from a smoke parent**, not W1's pick. The stock-trot parent stepped off-clock at +1.5 m/s, and at −1.5 it reached −0.149 m/s with 17 falls.
  Before r1, rerun the fold on the real parent's capture and check:
  - cadence ≈ cycle;
  - |auto − 2| ≤ 0.5 step, and not at the search edge;
  - which channels are flagged. Pin any newly flagged channel, or justify it in writing.
- The 2-step lead exists **only because of integer truncation** in `mixed_gait_tracking`. If anyone makes it float, `--lag-steps` must become 1.8. Derive the value from the code; do not copy it.
- The front-`_2` pin is an extrapolation. The recovered fix names back abduction only, and the meaning of `lag_m2` is unknown, so fidelity to the old final rounds is not guaranteed.
- Never run `sym` on a rotary-gallop parent: it uses one shared pair offset for both pairs.
- With a gallop clock (`frequency_mult` > 1), 2 steps is a larger fraction of a cycle, so naive drift is worse.
- Single-seed arms (arm b, and naive s43 if it is cut): treat gaps under ~10 % or ~10 falls as noise.
- The old naive-vs-lag result is confounded (collapse and different weights), so r1 is effectively the first test.
- The `GAIT_STD` comment calls `_1` "abduction", but the FK note and the parent ranges show that `_2` is the lateral (abduction) joint. Pin by name, as above.

## What would flip this decision
- On the real parent, auto lag differs from 2 by more than 0.5 step, or its residual does not drop. Investigate the clock and capture alignment before choosing a value.
- `mixed_gait_tracking` is changed to a float rewind. Then use `--lag-steps 1.8`.
- Arm (b) beats the default beyond noise on tracking, falls or splay, with no collapse. Then drop the front pin from r2 on.
  Conversely, if any unpinned channel moves by more than 2× its range on the real parent, add it to the pin set.
- r1 naive ≥ lag at both seeds. Then the lag correction is immaterial; keep lag for r2–r3 for consistency.
- The real parent is a trot-family gait with L/R asymmetry above ~0.1 rad. Then reconsider `sym` as an arm.
