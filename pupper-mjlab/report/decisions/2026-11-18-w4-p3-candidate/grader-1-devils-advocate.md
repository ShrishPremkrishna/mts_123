# Part 3 candidate, its HOLD status, and whether hardware eligibility needs a push condition
Status: grader 1 of 4 (lens: devil's advocate vs front-runner Bumpy +2000) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: Critical. It feeds the headline Part 3 claims and decides whether anything is prepared for a physical robot.
Reversible?: Yes for the sim pick (it is a re-label, and evals exist for every point). A hardware HOLD costs nothing to reverse. Lifting it later needs only ~10 GPU-min of push evals.

## Conclusion
**(a)** The Part 3 **simulation** candidate stays **Bumpy +2000** (`rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt`). It is the pre-registered pick, and the case against it does not produce a better *pre-registered* alternative.
**(b)** Under the HOLD rule **as written**, it is **CLEAR**. I re-checked the arithmetic: (a) 9.5 − 10.5 = −1, (b) 9.5 − 38 = −28.5, and the control check gives 12 − 14 = −2.
**(c)** **Yes, add a push condition, as a hardware-only gate that does not change selection:**
- **HOLD if** the two-seed mean (eval seeds 1234 and 4321, flat_dr) of `R:push_stand@0.65` exceeds P's two-seed mean **+ 0.15**. P's mean is 0.109, so the threshold is 0.259.
- **Also HOLD if** the two-seed `stand@0.5` exceeds 0.05.
- Bumpy +2000 reads **0.493** and 0.155, so it is **HOLD**. Per kp-coverage, a HOLD in the WIDEN branch means no more training and a human re-decision.

**Net: nothing is hardware-eligible from this decision.**

## Evidence used
- **Push (`diag/README.md`, `*_flat_dr_push.md`).** `R:push_stand@0.65`:
  - P: 0.096 / 0.122.
  - Bumpy +2000: 0.489 / 0.497.
  - Flat control +2000: 0.092 (one seed).
  - Every Bumpy checkpoint measured from +500 on reads 0.49–0.86, including the +2000 neighbours 6400 (0.563) and 6450 (0.721), so +2000 is the *best* push in its own neighbourhood.
  - The gap is about 10× the W3 band (0.040), and the push repeat spread is ≤ 0.024.
  - `stand@0.5`: P 0, control 0.003, +2000 0.151 / 0.158, neighbours 0.22 / 0.30. A 0.5 m/s lateral shove knocks over about 15 % of standing robots.
- **Falls and nompd (`_summary.md`, `ablate*.md`).**
  - Bumpy +2000: flat_dr 10 / 11, bumpy_dr 18 / 23, nompd 7 / 12.
  - The +2000 neighbours read flat_dr 4–7 and nompd 6–14.
  - P: 14 / 16, bumpy_dr 35, nompd 39 / 37. P **fails HOLD(a)** (gap +22.5).
  - Control +2000: 12, 29, 20 (seed 1234 only). Its nompd was 51 at +1500, which would fail HOLD(b).
- **W3 final-policy decision.** The Part 2 headline rests on push_stand going from 0.455 (parent) to 0.095. That was the only ranked effect that replicated across seeds.

## Thought process
**The case against +2000** (REPRO_PLAN §3: rank on sim-to-real readouts, push recovery included, and always compare with the matched control):
1. **Against the matched control at +2000, the Bumpy-specific gains sit at the noise band.** On seed 1234: flat_dr −2 (a tie), bumpy_dr −11 and nompd −13, both at the edge of the 10–15 tie band. The push cost is +0.40, about 10× its band, and it replicates on two seeds and on 7 checkpoints.
2. **+2000 erases Part 2's one replicated result.** Its standing push (0.49) is *worse than the Part 1 parent* (0.455). So the report's deployable policy would contradict the Part 2 headline.
3. **The risk was foreseeable.** The budget decision's G2 flagged that standing-side costs regressed in the only Bumpy precedent. The guards and the extension rule still looked only at falls, the seam and nompd.
4. **The push numbers are optimistic and incomplete.** +2000 is at the lucky end of its neighbourhood, and push was only ever measured under Lab 4 DR (soft kp), never at nominal gain.
5. **A robot that stands still is the most common hardware state** (start-up, between commands, on the operator's stand), and it falls about 15 % of the time from a gentle shove.

**Why I still pick +2000 for sim.** Every alternative is worse or post-hoc:
- **+500:** post-hoc, and its push is worse (0.80).
- **P:** fails HOLD(a).
- **Flat control:** a comparator, not the spec's Bumpy robustness pass. It is post-hoc, has one seed, and its nompd is volatile (34 → 51 → 20).

+2000 is also the best on falls in every condition, has the lowest F_runmax (2.60 BW), passes the seam and passes the export audit.

**Why a post-hoc push gate is legitimate.** A gate that can only *restrict* deployment does not create a forking-paths selection bias. Push was a pre-registered ranked metric (W2) that the Part 3 rules simply omitted. The 0.15 margin is about 4× the band, which allows for checkpoint swings. The gate is anchored on P, because the fine-tune should not give back the Part 2 gain.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| Ship +2000 as hardware-eligible (rule as written) | Ignores a ranked safety readout that is about 10× noise worse, replicated, and present at every Bumpy checkpoint |
| Switch the candidate to flat control +2000 | Post-hoc. One seed on suite, nompd and push. Not the spec's Bumpy pass. Worth measuring for the human re-decision |
| Bumpy +500 | Not pre-registered, and push 0.80 (worse) |
| P | HOLD(a) fails (nompd gap +22.5) |
| Push gate relative to the matched control | The control is single-seed and volatile. P has two seeds and is the Part 2 claim |

## Severe flagged issues
- **Standing push regression** of 0.10 → 0.49, about 15 % falls at 0.5 m/s. It replicates across seeds and checkpoints. The likely cause is terrain, since the control with the same PD ranges keeps its push recovery.
- **Push was never measured at nominal PD.** The direction of the error is unknown.
- **The Bumpy-vs-control falls advantage at +2000 is borderline noise.** The control has only seed 1234, so the "Bumpy improves robustness" claim must be stated as Bumpy vs P, with the control alongside.
- The selection guards are one-seed comparisons, and the +1000 dip (flat_dr 34) shows training is non-monotonic.

## What would flip this decision
- **Push at nominal PD** (both seeds) with +2000 standing@0.65 ≤ 0.26: the push HOLD lifts, and +2000 becomes eligible.
- **Flat control +2000 on seed 4321** (suite, nompd, push) that clears HOLD, with bumpy_dr within 10 of Bumpy +2000 on the two-seed mean: candidate for the human re-decision.
- **A human or TA ruling that push is out of scope for Part 3 eligibility:** +2000 is eligible under the rule as written.
