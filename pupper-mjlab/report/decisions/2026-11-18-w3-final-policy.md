# Which Part 2 checkpoint is the final policy that goes to Part 3?
Status: DECIDED by graders (4/4 agree) + synthesizer (pending human review) · 2026-11-18 · Requested by: W3
Criticality: Critical. It carries the report's headline Part 2 claim, it is P (the starting point) of the Part 3 fine-tune, and it is the lineage that may later be prepared for hardware.
Reversible?: Yes, at no cost before Part 3 launches, because this is only a pointer and every +1500 checkpoint and eval exists. After launch, a re-pick costs the Part 3 arm pair (~2 × 1500 it, ~40 GPU-min), about 1 h of step-0 and guard evals, and a report rewrite.

Grader docs: [1 devil's advocate](2026-11-18-w3-final-policy/grader-1-devils-advocate.md) · [2 raw data](2026-11-18-w3-final-policy/grader-2-raw-data.md) · [3 sim-to-real](2026-11-18-w3-final-policy/grader-3-sim-to-real.md) · [4 noise skeptic](2026-11-18-w3-final-policy/grader-4-noise-skeptic.md)

## Conclusion
The final Part 2 policy is **`~/runs/opt-lab-2/rp-p2-s42-r2-cont-1500/model_4496.pt`** (arm r2-cont-1500, seed 42, WandB **`wmhcp24q`**, sha256 `ce977121…`, root `1c5r0n8c`, +1500). Its reference dir is **`configs/opt-lab-2/p2/s42/r2-lag`** (fold sha `794b0615`, commit `60ce5a4`). It replaces the parent and goes to Part 3 as P. It is **not hardware-eligible as-is**: its nominal-PD gap is +22.5 > 15, so Part 3 runs the WIDEN branch and the HOLD rule applies.

## Evidence used
Every number below was checked against the raw JSONs (`eval/w3`, `w3-rep`, `w3-nompd`) and `analysis/*.md`. `chain_check.json` reports 0 violations, and the eval's reference hash matches `fold_summary.json`.
- **Matched deltas at +1500** vs the same-seed ctrl-1500 (s42 / s43, averaged over the original eval and its repeat; `repeat_spread.md`):
  - **sat@1.5 gate:** −0.025 / −0.023 (band 0.0006). Every arm passes; r3 is −0.026 / −0.029.
  - **push_stand@0.65:** −0.203 / −0.350. This is the only ranked effect that is above the band (0.040) in both seeds for any arm. r3 is +0.102 / −0.187 (the seeds disagree in sign).
  - **Falls:** −17.5 / −4, mean −10.75. The s43 delta is a tie. r3 is −8.5 / +3.
  - **F_runmax:** +0.186 / −0.017 (band 0.076). The s42 delta is a regression beyond noise. r3 is +0.002 / −0.182.
  - **push_move@0.5:** 0 for every arm.
- **Levels for the pick** (original / repeat): falls 16 / 17; push_stand@0.65 0.099 / 0.090 (@0.8: 0.525 / 0.541); F_runmax 3.04 / 3.11 BW; sat@1.5 0.160. The s42 parent reads 43 / 44, 0.455 / 0.470, 3.04 / 2.96 and 0.179.
- **Nominal PD** (`nompd_check.md`, eval seeds 1234 / 4321):
  - **Pick:** flat_dr 14 / 17, nompd 39 / 37 (mean 38), gap +22.5. Under nompd, F_runmax is 3.32 / 3.40 and sat@1.5 is 0.209 / 0.210.
  - **Arm means over both seeds:** r2-cont 37.5, r3 40.0, r1-cont 46.5, ctrl 46.75.
  - Only s43-r3 (+9.0) and s43-ctrl (+12.0) are under 15. s42-r3 is +24.0, and the s43 parent is 68 at nominal PD (W3; W4's re-measure gives 67 / 61). The low nompd of the s43 checkpoints is therefore a lineage trait, not an r3 effect.
  - s42 parent (W4): nompd 99 / 79, gap +48.5.
- **Runner-up s43-r3:** falls 16 / 29 (the largest repeat swing); push_stand@0.65 0.334 / 0.294; push_stand@0.5 0.018 / 0.034; 1 / 7 falls outside the push windows; F_runmax 2.77 / 2.81; sat@1.5 0.151.
- **New context.** W4's real-gain check (`w4/.../part3/parent_check/result.md`) leaves the Part 1 parent pick standing. fast_trot's nompd is 88.8 vs gallop_c's 76.5, and its excess is +35.8. The s42 lineage stays the designated one.

## Thought process
I followed REPRO_PLAN §3: rank on sim-to-real readouts, compare only at matched iterations, use two-seed paired deltas, and treat single-eval fall gaps under 10–15 as ties.
1. **Gate.** sat@1.5 passes for all three r-arms, by 18–47× the band, at matched or higher speed (r2-cont reaches −1.54 vs ctrl −1.48).
2. **Ranked readouts.** Only one effect is beyond noise and agrees in sign across lineages: r2-cont-1500's standing-push gain. It is also the only arm with a falls mean beyond 10.
3. **r3's edges.** Its advantages are F_runmax and slightly lower sat. They are real, but r3 trades them for a push_stand that is 0.19–0.31 worse. r3's nompd advantage does not replicate at s42.
4. **nompd.** W4 makes nompd part of selection. On the arm mean, r2-cont is the lowest (37.5), but the arms differ by less than single-seed noise, so it counts as "not worse".
5. **Seed within r2-cont.** Falls (16 vs 15) and nompd (38 vs 37) tie. push_stand@0.65 favours s42 by 0.05 (just above the band), and F_runmax favours s43 by 0.12 BW.

   I chose s42 because it descends from the designated parent (which W4 has now re-confirmed), its readouts are the most stable on repeats and eval seeds, and it matches Part 3's seed. Both seeds land in WIDEN, so Part 3's plan is the same either way.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| s43 r2-cont-1500 (`p2/s43/r2-lag`, `rwfqzxv2`) | Runner-up within the arm. It ties on falls and nompd, has lower F_runmax (2.95), and is worse on push_stand (0.15 vs 0.095). Its nompd swings 44 / 30 across eval seeds, and it descends from the non-designated parent. |
| s43 r3 (`p2/s43/r3-lag`, `adn6hd33`) | Runner-up across arms. It has the lowest F_runmax, sat and nompd gap (+9, the only r-arm that passes HOLD(a)). That gap is a lineage trait. Its push_stand is 3× worse, it falls at a 0.5 m/s standing push, and it has the noisiest repeats (16 / 29). |
| s42 r3 (`rrdkrrzw`) | Lower F_runmax (2.89), but push_stand is 0.407, nompd 46.5 and the gap +24. |
| r1-cont-1500 | Falls tie and disagree in sign. It has the weakest sat gain and nompd 46.5. |
| ctrl-1500 | It is the baseline, so it carries no Part 2 contribution. s42-ctrl nompd is 62.5, and its gallop_c reference needs 3.82 Nm (above 3 Nm). |
| Keep the parent `1c5r0n8c` | Worse on every hardware readout: falls 43, push_stand 0.455, sat 0.179, nompd 89. |
| +500 r1-naive (−15.5 falls) | Not continued to +1500. It has no nompd and no repeats, so it is not iteration-matched. |

## Severe flagged issues
- **The pick fails HOLD(a) as-is** (gap +22.5 > 15). Part 3 must run WIDEN. HOLD(b) caps the fine-tuned checkpoint's nompd at 48 (nompd(P) 38 + 10). A HOLD in the WIDEN branch goes to a human re-decision; it does not trigger more training.
- **The falls claim is effectively s42-only.** The deltas are −17.5 / −4, and at +1000 the parent run r2 was *worse* than its control (+3 / +15). The headline must rest on push_stand and sat@1.5, not on falls.
- **Foot impact regresses in s42.** F_runmax is +0.186 BW vs its control (2.4× the band). It is the highest of any +1500 checkpoint, 3.32–3.40 under nompd. It only matches the parent; it does not improve on it. Report it next to the headline.
- **Noise is understated.**
  - The W2 suite and push test used only eval seed 1234, so the repeats measure GPU nondeterminism only.
  - There is one training seed per lineage.
  - push_stand swings up to 0.28 between consecutive checkpoints (s43 r1: 0.32 → 0.04 → 0.29).
  - The neighbouring checkpoints (model_4400 / 4450) exist but were never evaluated.
- **Push was never measured at nominal PD gains.** The pick's main safety advantage is DR-only evidence.
- **Torque reality.** sat@1.5 is about 0.16 against the 3 Nm limit. The robot-side back-EMF model caps torque near 1.37 Nm (unvalidated). Correction to grader 3: "every candidate ~15–16 %" holds only for r2-cont and r3 (0.151–0.160); r1-cont is 0.170–0.174 and ctrl 0.181–0.186.
- **Attribution.** Part of the parent-to-final gain also appears in the control (s42 ctrl: falls 34, nompd 62.5). Only arm − ctrl deltas are Part 2 effects.

## What would flip this decision
- **Eval seed or neighbouring checkpoints.** W2 + push on eval seed 4321, or at model_4400 / 4450, for s42 / s43 r2-cont and r3. Re-decide on F_runmax and sat (toward r3) if:
  - r2-cont's push_stand@0.65 lead over r3 drops below 0.10 in either seed; or
  - r3 reads ≤ 0.20 or r2-cont reads ≥ 0.30 at a neighbouring checkpoint.
- **Push at nominal PD.** If s42-r2c's standing@0.65 advantage over s43-r3 shrinks within 0.05, switch to s43-r3.
- **A third nompd eval seed (777).** Switch if either holds:
  - s43-r3 reads ≥ 10 falls below s42-r2c; or
  - s43-r2-cont reads > 10 below s42-r2c while push_stand stays within 0.04.
- **Part 3 step 0 on this checkpoint.** If nompd deviates > 15 from 38, re-measure and re-rank the arms on nompd.
- **A human or TA ruling** that the Part 2 final must itself have a gap ≤ 15. Then s43-r3 is the only r-arm candidate, after a third-seed confirmation.
- **A hardware-side limit** (validated EMF cap, gearbox or foot load) that separates ~3.1 BW / 0.160 from ~2.8 BW / 0.151. Then s43-r3, or s43-r2-cont (2.95 BW).

## Perspectives & synthesis
**Agreement.** All four lenses pick the same checkpoint, and their facts agree. I re-derived every decisive number and found no wrong values. The only correction is grader 3's saturation generalization, noted above.
- **Devil's advocate:** its strongest case (falls lead from one seed, s42 impact regression, nompd gap, snapshot luck) fails, because r3's and s43's nompd advantages do not replicate across lineages.
- **Raw data:** r2-cont is the only arm with a cross-seed push_stand effect beyond the band.
- **Sim-to-real:** r2-cont is never worse on falls beyond noise at either plausible gain, and it is 3× better on standing push. Its larger gap comes from being better under DR, not worse at nominal.
- **Noise skeptic:** most gaps are ties. The push_stand gap is the only real separator, at roughly 1.5–2σ. Ties break toward the simpler, more reproducible artefact on the designated lineage.

**Disputes were about criteria only.**
- How much to weight F_runmax against push_stand: graders 1, 3 and 4 note r3 or s43 wins on impact.
- Whether a gap ≤ 15 should be required of the Part 2 final: all four reject this, and it would favour s43-r3.
- s42 vs s43 within r2-cont: a tie broken by provenance and stability.

No lens split on the pick, so no arbitration was needed.

**Objections that survive:** the s42-only falls effect, the F_runmax regression, the failed HOLD(a), the single eval seed and unevaluated neighbouring checkpoints, and push never measured at nominal gains. The new W4 parent check strengthens the s42 tie-break. It also shows that same-seed nompd re-measures move up to 6 falls (s43 parent: 67 / 61 vs 69 / 67).

**For Part 3**
- P is this checkpoint, with nompd(P) = 38.
- Step 0 should reproduce a gap of about +22.5, which selects WIDEN (kp ×[0.6, 1.3], kd ×[0.7, 1.5]); evaluation stays on the Lab 4 ranges.
- Track F_runmax at every Part 3 guard eval.
- Add push at nominal gains to the final Part 3 evals.

**For hardware preparation**
- Nothing from Part 2 goes to the robot.
- Only a Part 3 checkpoint that clears HOLD(a) (gap ≤ 15) and HOLD(b) (nompd ≤ 48) is eligible, and it still needs hardware checklist item 5 and course approval.
- First supervised trials should cap commanded speed below ±1.5 m/s, because the 1.37 Nm EMF cap and about 16 % saturation at 3 Nm are unresolved.
- Simulation evidence only.
