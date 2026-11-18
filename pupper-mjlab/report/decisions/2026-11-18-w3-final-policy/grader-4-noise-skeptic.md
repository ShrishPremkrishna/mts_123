# Which Part 2 checkpoint is the final policy that goes to Part 3? (grader 4, lens: noise skeptic)
Status: DECIDED by grader (pending synthesis) · 2026-11-18 · Requested by: W3 (Part 2)
Criticality: Critical. It feeds the report's headline Part 2 claim, it is P for the Part 3 fine-tune, and it may later go toward hardware.
Reversible?: yes, before Part 3 launches (it is only a pointer; all +1500 checkpoints and evals exist). After launch, reversing costs the Part 3 pair (~2 × 1500 it, ~40 GPU-min) plus its evals.

## Conclusion
**`~/runs/opt-lab-2/rp-p2-s42-r2-cont-1500/model_4496.pt`** (arm r2-cont-1500, seed 42, WandB `wmhcp24q`, root parent `1c5r0n8c`), reference **`configs/opt-lab-2/p2/s42/r2-lag`**.
The +1500 ranked readouts split between r2-cont-1500 and r3, and most of the gaps are ties. I break the tie on reproducibility (s42-r2-cont has the tightest repeats of any candidate on every suite) and on the designated lineage. The only ranked gap that is beyond eval noise and agrees in sign in both lineages is standing push, and it favours r2-cont.

## Evidence used
All candidates are at +1500 (model_4496). `chain_check.json` has no violations (4096 envs, same root, cumulative = level). Two-seed means of s42 / s43; "rep" = the repeat eval with the same eval seed 1234 (`repeat_spread.md`).
- **sat@1.5 gate (all pass).** Deltas vs ctrl-1500: r3 −0.0266 / −0.0297, r2-cont −0.0254 / −0.0236, r1-cont −0.0115 / −0.0107. The repeat max |diff| is 0.0006. r2-cont runs faster at −1.5 (−1.54 vs ctrl −1.48), so it is not passing by running slower.
- **W2 falls (orig / rep).** r2-cont s42 16/17, s43 15/16. r3 s42 23/28, s43 **16/29**. ctrl s42 35/33, s43 18/21. Repeat-averaged delta vs ctrl: r2-cont −10.75 (−17.5 / −4), r3 −2.75 (−8.5 / +3).
- **flat_dr falls (eval seeds 1234 / 4321).** r2-cont s42 14/17, s43 23/16. r3 21/24, 22/27. ctrl 30/36, 18/20.
- **Nominal-PD falls (eval seeds 1234 / 4321).** r2-cont s42 39/37, s43 44/30 (mean 37.5). r3 52/41, 30/37 (40.0). ctrl 68/57, 31/31 (46.75). Parent s42 99/79.
  - nompd − flat_dr: s42-r2-cont +22.5, s43-r2-cont +17.5, s42-r3 +24.0, s43-r3 +9.0.
- **R:push_stand@0.65 (falls per push, ~332 pushes, SE ≈ 0.02–0.03).** r2-cont s42 0.099 (33/332, rep 0.090), s43 0.148 (rep 0.155). r3 s42 0.407 (rep 0.393), s43 0.334 (rep 0.294). ctrl s42 0.298, s43 0.498. The repeat max is 0.040.
- **F_runmax_med_BW (orig / rep).** r2-cont s42 3.043/3.110, s43 2.955/2.954. r3 s42 2.889/2.896, s43 2.766/2.814. ctrl s42 2.907/2.873, s43 2.992/2.952. Parent s42 3.038. The repeat max is 0.076.
- **Push moving@0.5.** 0/1013 for every r* arm in both seeds, a tie.

## Thought process
Rules: rank on sim-to-real readouts, gate sat@1.5, compare only at equal iterations, use two-seed means and paired deltas, and treat fall gaps under 10–15 as ties. For each gap, I asked whether it survives (i) eval repeats, (ii) the second lineage, and (iii) checkpoint-to-checkpoint volatility.

**Treated as ties**
- **Falls, r2-cont vs r3** (W2 16 vs 24 repeat-averaged; flat_dr 17.5 vs 23.5; nompd 37.5 vs 40). Every gap is under 10. s43-r3 alone moved 13 falls between identical evals, which is the largest repeat spread of the 10 checkpoints. r2-cont is lower in 5 of 6 lineage × suite cells, but the cells share checkpoints, so this is a lean, not evidence.
- **nompd − flat_dr.** s43-r3's +9 is the only value under the WIDEN threshold of 15. s42-r3 is +24, so r3 does not replicate it. s43-r3 is also within ~8.5 of s43-r2-cont (+17.5), and single-checkpoint nompd moves up to 14 between eval seeds (s43-r2-cont 44/30). I do not treat it as an arm property, so it does not justify choosing the KEEP branch.
- **sat@1.5, r3 vs r2-cont** (−0.028 vs −0.024). This is beyond eval noise, but it is a 0.004 difference on a gate that both pass by about 40× the band.
- **Push moving.** All zero.

**Possibly real gaps, which split**
- **push_stand favours r2-cont** over r3 by 0.31 (s42) and 0.19 (s43), and over ctrl in both seeds. The gap is 5–8× the repeat band and agrees in sign in both lineages. Caveat: this metric swings by up to ~0.28 between consecutive 500-iteration checkpoints of one lineage (s43 r1: 0.32 → 0.04 → 0.29) and by 0.44 between parents. With n = 2 lineages, it is roughly a 1.5–2σ effect. It is the strongest separator, but it is not conclusive.
- **F_runmax favours r3** by 0.18 BW (s42) and 0.16 BW (s43), repeat-averaged, which is above the 0.076 band in both seeds. But r2-cont's excess over its own ctrl is s42-only (+0.19 vs −0.02, sign disagreement). s42-r2-cont sits at the parent's level (3.04), not above it.

**Tie-break: cheaper, safer, more reversible**
- **Arm.** r2-cont-1500 and r3 share the same +1000 warm start (r2) and differ only in the last 500 iterations' reference. r2-cont is the simpler and more reproducible artefact: one fewer capture/fold round, and no r3-lag reference to defend in the report.
- **Lineage.** Within r2-cont, s42 vs s43 is a tie: falls 16 vs 15, nompd 38 vs 37, push_stand 0.095 vs 0.15, F_runmax 3.08 vs 2.95. Both land in the WIDEN branch (+22.5 vs +17.5). I take s42 because:
  - its readouts are the most stable of all candidates (W2 16/17, flat_dr 14/17, nompd 39/37, push 0.099/0.090);
  - it descends from the designated `1c5r0n8c`;
  - it matches Part 3's seed-42 fine-tune;
  - it avoids the s43 provenance caveats in `w3-s43-lineage/devils-advocate*.md`.
- **Within s42**, r2-cont beats ctrl beyond noise on falls (−17.5 repeat-averaged), flat_dr (15.5 vs 33), nompd (38 vs 62.5), push_stand and sat.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| s42-r3 / s43-r3 (best F_runmax, lowest sat@1.5; s43-r3 alone has nompd Δ ≤ 15) | Falls and nompd tie with r2-cont. push_stand is 0.2–0.3 worse in both seeds. s43-r3 has the noisiest repeats (16/29), and its Δ = 9 does not replicate in s42 (+24). |
| s43-r2-cont-1500 | Ties s42-r2-cont. Its eval-seed nompd spread (44/30) is near the 15 "add a third seed" trigger. It is non-designated provenance. Its lower F_runmax is the one real point in its favour. |
| r1-cont-1500 (either seed) | Falls tie with ctrl (−2.5, signs disagree). It has the weakest sat@1.5 gain and worse push_stand than r2-cont. |
| ctrl-1500 (s43 is strongest on nompd: 31, Δ +12) | It is the control, so it carries no Part 2 contribution. Within s43, it ties r2-cont on falls and nompd and is far worse on push_stand (0.50) and sat (0.181). |
| Keep the parent (no replacement) | It is not iteration-matched to anything at +1500. It is worse on every hardware readout (W2 43, nompd 99/79, push_stand 0.455). |
| +500 / +1000 arms (e.g. r1-naive, −15.5 falls vs ctrl-500) | They are not matched to the +1500 candidates, and the protocol fixes the budget at +1500. |

## Severe flagged issues
- **The decisive gap rests on n = 2 lineages and one checkpoint per arm.** push_stand moves up to ~0.28 between consecutive checkpoints, so the 0.19–0.31 r2-cont advantage could be a checkpoint lottery. No neighbouring checkpoint (e.g. model_4450) was evaluated, although they exist on disk.
- **The repeat evals only measure GPU nondeterminism** (same eval seed 1234). Eval-seed spread exists only for flat_dr and nompd (4321), where single-checkpoint gaps reach 14 falls, so the W2 noise band is understated.
- **s42-r2-cont has the highest F_runmax of any +1500 checkpoint** (3.04/3.11, about +0.19 BW over its matched control). It does not replicate in s43, but it is a hardware-relevant flag to watch in Part 3.
- **nompd − flat_dr = +22.5 > 15**, so the kp-coverage rule puts Part 3 in the WIDEN branch. HOLD (a) is likely unless the fine-tune closes the gap.

## What would change my mind
- Evaluate model_4400 and model_4450 of s42/s43 r3 and r2-cont (W2 + push). If r3's push_stand is ≤ 0.2 at its neighbours, or r2-cont's is ≥ 0.3, the push gap is checkpoint noise. Then F_runmax decides, and I would pick r3.
- W2 + push repeats on eval seed 4321 for the four r2-cont/r3 checkpoints. If the falls gap opens past 15 either way, or the push gap shrinks below 0.1, I would re-decide.
- A third lineage (e.g. s44) for r2-cont vs r3. If the push_stand sign flips, I would treat the arms as tied and pick on F_runmax and sat (r3).
- If s43-r3's nompd Δ stays ≤ 15 on a third eval seed and s42-r3's falls to ≤ 15, Δ becomes an arm property, and I would prefer s43-r3 to stay in the KEEP branch.
- Any evidence that F_runmax ~3.1 BW is a hardware risk at the Pupper's feet would push me to s43-r2-cont (2.95).
