# Part 3 candidate, HOLD eligibility, and whether push recovery should gate hardware (grader 4, noise skeptic)
Status: grader doc (1 of 4; pending synthesis) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: Critical. It carries the headline Part 3 claims and decides whether anything is prepared for a physical robot.
Reversible?: (a) yes: a re-selection among saved checkpoints, no GPU. (b)/(c) a HOLD is fully reversible (a confirming eval is about 15 GPU-min). Clearing a policy for hardware is the irreversible direction.

## Conclusion
- **(a)** The Part 3 simulation candidate is **Bumpy +2000** (`rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt`), as the pre-registered rule selected. Its edge over +1500 is a tie. +500 is dominated on two eval seeds and was never a pre-registered option.
- **(b)** Under the pre-registered HOLD rule it is **CLEAR**, and the result is not noise-sensitive: every measured checkpoint in leg 4 also clears.
- **(c)** **Yes, add a push condition.** The standing-push regression is the one gap in this decision that is unambiguously real. Proposed rule, HOLD(c): hold if `R:push_stand@0.65` on flat_dr, averaged over eval seeds 1234 and 4321, exceeds P's two-seed mean by more than 0.10 (P = 0.109, so the limit is about 0.21). +2000 reads 0.493, so it is **HOLD under the amended rule**. Per kp-coverage, a HOLD in the WIDEN branch goes to a human re-decision, not to more training.

## Evidence used
- `p3-final-kpwide_summary.md`, `_selection.json`, `_deploy_hold.json`, `diag/README.md`, `diag/neighbours/*/ablate`, `diag/push/*`, `part2/eval/README.md` (push pilot), and `w3-final-policy.md` (push band 0.040; push_stand swings up to 0.28 between neighbouring checkpoints).
- **+2000 vs +1500 (seed 1234).** All three gaps are ties under §3:
  - flat_dr: 10 vs 11.
  - bumpy_dr: 18 vs 23.
  - nompd: 7 vs 14.
  - push_stand: 0.489 vs 0.533.
- **+500 vs +2000, on both eval seeds.** +500 is worse on both seeds:
  - bumpy_dr: 31/36 vs 18/23 (13 worse on each seed).
  - nompd: 25/31 vs 7/12 (18–19 worse).
  - push_stand: 0.798 vs 0.489 (worse).
- **HOLD margins (two-seed means).**
  - (a) nompd − flat_dr = 9.5 − 10.5 = −1.0, against a limit of 15.
  - (b) nompd − P's nompd = 9.5 − 38.0 = −28.5, against a limit of 10.
  - Control flat_dr − P's flat_dr = −2, against a limit of 10.
  - The leg-4 neighbourhood (6000–6492) gives flat_dr 4–16 and nompd 6–17. Its worst nompd − flat_dr is about 8, and its worst nompd is 17, which is 21 below P.
- **Standing push @0.65 m/s (about 340 pushes per cell).**
  - P: 0.096 / 0.122 (seeds 1234 / 4321).
  - +2000: 0.489 / 0.497. The seed difference is 0.008.
  - Bumpy arm overall: 0.489–0.856 across 9 checkpoint/seed readings and both terrains.
  - Flat control: 0.024–0.163.
  - The z-score is about 17 on the two-seed pooled data (scratch `graders/cand-g4/push_z.txt`).
  - The lowest Bumpy reading sits 0.33 above the highest P or control reading. That is larger than the largest neighbour swing ever observed (0.28).
  - The regression also appears at the gentlest rung: stand@0.5 is 0.151 / 0.158 vs P's 0 / 0. At 0.8 it is 0.91 vs 0.54.
  - push_move@0.5 is 0 everywhere, so moving robots show no regression.
- **Matched +2000, Bumpy vs flat control.**
  - flat_dr: 10 vs 12 (tie).
  - bumpy_dr: 18/23 vs 29. The control has one seed only. The gap of 6–11 is in the band.
  - nompd: 7/12 vs 20 (one seed). The control's nompd moves 28 → 34 → 51 → 20 across its legs.
  - push_stand: 0.489 vs 0.091, a real gap in the control's favour.

## Thought process
I asked of each gap whether it is real, ranking on sim-to-real readouts (§3). Gaps under about 10–15 falls are ties, and a tie goes to the safer or more reversible option.
1. **The (a) choice is a tie between +1500 and +2000.** The pre-registered rule breaks the tie toward +2000, and post-hoc re-selection buys nothing.
   - +500 loses on two seeds and on push, so choosing it would be post-hoc selection of a worse checkpoint.
   - P fails HOLD(a): its step-0 Δ is 23 > 15. It is not a hardware fallback either.
2. **The HOLD CLEAR is robust.** Margins of 16 and 38 falls exceed eval-seed noise plus checkpoint noise (±7–10), and every neighbour clears.
3. **Push is a different matter.** Two eval seeds, two terrains, three rungs and nine checkpoints agree, so this effect is not noise.
   - Most of +2000's fall advantages over the matched control are band-level, while its push deficit is about 12σ.
   - On noise-robust evidence, the Bumpy fine-tune trades a real standing-push loss for a gain that is clearly real only against P on nompd, and the flat control mostly shares that gain (nompd 20 at one seed).
   - It also reverses the only replicated Part 2 gain: push_stand went from 0.455 at the parent to 0.10 at P, and +2000 is back at 0.49.
4. **Adding HOLD(c) now is post-hoc.** I accept that because it can only move a policy toward HOLD, which is the reversible direction. A HOLD costs a re-decision; a fall on hardware is a physical outcome. Selection for the simulation claims stays pre-registered and unchanged.
5. **The threshold is set by noise, not by the data.** +0.10 is 2.5× the W2/W3 push band (0.040), about 4× the two-seed binomial SE, and about 4× the observed seed-to-seed differences (≤ 0.026).
   - It does not depend on where +2000 happens to sit: every Bumpy checkpoint ever measured fails it by at least 0.27, and P and the control pass.
   - It uses flat_dr because that is the measured cell with two seeds. The bumpy_dr value (0.486) agrees.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| Bumpy +1500 | Ties +2000 on every readout; the rule says ship the last leg. |
| Bumpy +500 | Post-hoc. Worse on bumpy_dr and nompd on both seeds, and worse on push. |
| P | Fails HOLD(a) (Δ 23). It is not a Part 3 (Bumpy) policy. |
| Flat control +2000 as the candidate | Post-hoc and not the spec's Bumpy robustness pass. Its nompd has one seed and swings by 31 across legs. It is the obvious input to the human re-decision (see flips), but not something I can pick here. |
| No push condition (rule as registered) | It would clear a policy whose only noise-robust ranked difference from its control is a 5× worse standing push. |
| Push threshold absolute (e.g. ≤ 0.25) or tied to the control | Equivalent here, since both thresholds give the same verdicts. Anchoring on P matches HOLD(b), and P has two seeds. |

## Severe flagged issues
- **Push was never measured at nominal PD gains** (the W3 flag stands). HOLD(c) uses DR-only evidence.
- The headline claim "Bumpy fine-tune improves robustness vs control" is not noise-robust at matched +2000:
  - bumpy_dr is +11 on one control seed and nompd about 10, both band-level;
  - at +1500 the gap was large (29 / 37), but control +1500 looks like a dip (nompd 51).
  - Report the claim with that caveat.
- P has no seed-4321 bumpy_dr reading, so the bumpy_dr gain vs P (35 → 18/23) rests on P's single seed.
- HOLD(c) is a post-hoc addition. It is defensible only because it is conservative, and the report must say so.

## What would flip this decision
- A seed-4321 suite + nompd + push on control +2000, plus a neighbour (6450). Either of these results bears on the claim and the re-decision:
  - Bumpy beats the control on bumpy_dr by more than 15 on two seeds: that firms up the Bumpy claim.
  - The control holds nompd ≤ 20 and push ≤ 0.15: that makes it the natural hardware candidate.
- Push at nominal PD shows the Bumpy arm within 0.10 of P → drop HOLD(c) for +2000.
- A Bumpy checkpoint, or a Bumpy run with push events in training, that reads push_stand@0.65 ≤ 0.21 on two seeds while keeping its HOLD(a)/(b) margins → eligible.
- A human or TA ruling that standing push is not a deployment-relevant risk for the hallway speed test → (b) as registered, CLEAR.
