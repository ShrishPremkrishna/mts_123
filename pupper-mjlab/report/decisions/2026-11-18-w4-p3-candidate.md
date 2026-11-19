# Part 3 sim candidate, its HOLD status, and whether hardware eligibility also needs a push condition
Status: DECIDED by synthesizer (4/4 lenses agree; pending human review) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: Critical. It picks the policy behind the headline Part 3 claims and decides whether anything is prepared for a physical robot.
Reversible?: Yes. (a) is a re-label among saved, already-evaluated checkpoints (0 GPU). A hardware HOLD costs nothing to lift. The irreversible direction is clearing a policy that then falls on the robot.

Graders: [1 devil's advocate](2026-11-18-w4-p3-candidate/grader-1-devils-advocate.md) · [2 raw data](2026-11-18-w4-p3-candidate/grader-2-raw-data.md) · [3 sim-to-real](2026-11-18-w4-p3-candidate/grader-3-sim-to-real.md) · [4 noise skeptic](2026-11-18-w4-p3-candidate/grader-4-noise-skeptic.md)

## Conclusion
- **(a) Sim candidate:** Bumpy +2000, `~/runs/opt-lab-2/rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt`, the pre-registered pick. Every Part 3 claim about it must state its standing-push regression alongside.
- **(b) Pre-registered HOLD rule:** **CLEAR**. (a) 9.5 − 10.5 = −1.0 (limit 15). (b) 9.5 − 38.0 = −28.5 (limit 10). WIDEN check 12 − 14 = −2 (limit 10).
- **(c) Push condition: added, for hardware eligibility only.** It never changes sim selection.
  - **HOLD(c):** hold if `R:push_stand@0.65` (standing falls/push at 0.65 m/s, flat_dr) averaged over eval seeds 1234 and 4321 exceeds P's two-seed mean **+ 0.10**. P's mean is (0.096 + 0.122)/2 = 0.109, so the limit is **0.209**.
  - +2000 reads 0.489 / 0.497, mean **0.493**, so it is **HOLD. Nothing is hardware-eligible from this decision.**
  - **HOLD(c) is added post hoc**, after the push data was seen. It is legitimate only because it can make the hardware decision *more conservative*, never less. It does not touch selection, and its metric was already pre-registered as ranked (W2).
  - Per kp-coverage, a HOLD in the WIDEN branch means **no more training** and a **human re-decision**. That re-decision gets:
    1. **Ratify or reject HOLD(c).** If rejected, +2000 is eligible under the rule as registered.
    2. **Default: hold everything.**
    3. **Alternative: flat control +2000** (`rp-p3-final-flatctrl-kpwide-leg4/model_6492.pt`) as a hardware-preparation candidate. That would be a labelled deviation: it was not a pre-registered option and it is not the spec's Bumpy pass. It reads push_stand 0.091 and nompd 20, but it still lacks:
       - a seed-4321 suite, nompd and push;
       - the two-seed HOLD(a)/(b)/(c) check, against the same limits as above;
       - an export round-trip/limit audit (none exists in `part3/export/`);
       - an explanation of its single-sample 62 BW F_peak flag on bumpy_dr;
       - push at nominal PD, which no policy has.

## Evidence used (verified by the synthesizer)
- **HOLD and selection.** `p3-final-kpwide_deploy_hold.json` and `_selection.json` match the conclusion.
  - +2000 two-seed means: flat_dr 10 / 11, nompd 7 / 12.
  - P nompd: 39 / 37.
  - The guard notes say "ship +2000 (no regression vs +1500 on seed 1234)".
- **Push.** Recomputed from all 20 Part 3 `*_push.json` files, `by_condition.stand`.

  | Policy | stand@0.65 | stand@0.5 |
  |---|---|---|
  | P | 32/334 = 0.096; 40/329 = 0.122 (seed 4321) | 0 / 0 |
  | Bumpy +2000 | 160/327 = 0.489; 159/320 = 0.497 (seed 4321); bumpy_dr 0.486 | 0.151 / 0.158 |
  | Every other Bumpy point | +500 0.798, +1000 0.754, +1500 0.533; neighbours 5950 / 6050 / 6400 / 6450: 0.856 / 0.710 / 0.563 / 0.721 | – |
  | Flat control | +1500 0.024, +2000 0.091 (one seed); bumpy_dr 0.163 | +2000: 0.003 |

  - The +2000 vs P gap is 0.384. Its z-score is about 17 on the pooled two-seed data. That is about 10× the W2/W3 band (0.040) and above the largest neighbour swing seen (0.28).
- **Other hardware readouts.** +2000 is best or tied on falls in every condition, on F_runmax (2.60 vs P 3.03 BW) and on sat@1.5 (0.165 vs 0.161). The seam check passes and so does the +2000 export audit.

## Perspectives & synthesis
- **Agreement (all four lenses):**
  - +2000 is the sim candidate. +500 is post hoc and dominated (push 0.80, nompd 25 / 31). P fails HOLD(a) (gap +22.5).
  - +2000 is CLEAR as registered.
  - A push gate is needed and +2000 fails it.
- **No factual disputes.** The graders' numbers agree with each other and with the JSONs. The SE figures differ only because some use one seed (about 12 SE) and some use two (about 17).
- **Surviving objections** (the report must carry them):
  1. Bumpy training regresses standing push at every measured checkpoint, back past the Part 1 parent (0.455). That erases Part 2's one replicated gain.
  2. At matched +2000, Bumpy beats the control only at band level: bumpy_dr 18 / 23 vs 29, nompd 7 / 12 vs 20. The control is single-seed and its nompd is volatile (28 → 34 → 51 → 20). "Bumpy improves robustness" should be stated vs P, with the control alongside.
  3. Push was never measured at nominal PD.
  4. `foundation/HARDWARE_CHECKLIST.md` has no item 5 to record the HOLD in.
- **Synthesizer's own check.** The rule text applies the WIDEN check to the control **at +1500**. The tool applied it at +2000 (−2). At +1500 it reads 23 − 14 = **+9**, which is still ≤ 10, so the verdict is still CLEAR, but with a 1-fall margin. This does not change (b).

## Disagreement & resolution
| Point | Positions | Resolution |
|---|---|---|
| Push margin over P | +0.10 (graders 2, 3, 4); +0.15 (grader 1) | **+0.10.** That is 2.5× the W2 band and about 4× the two-seed SE, and it mirrors HOLD(b)'s "no worse than P" form. Every verdict is the same for any margin in (0, 0.38), so this is not tuned to +2000. |
| Extra `stand@0.5 ≤ 0.05` sub-condition | Grader 1 only | **Not added.** stand@0.5 is informational under W2, not ranked, and it is redundant here (+2000 fails HOLD(c) anyway). Report it as a flag. |
| Anchor | P (all four); absolute or control-relative noted as equivalent | P: two seeds, and the HOLD rule already compares against it. |
| Flat control's role | Grader 3 proposes it for hardware after a confirm. Graders 1, 2 and 4 make it input to the human. | **Human re-decision only, no automatic promotion.** Graders' confirm thresholds differed (nompd ≤ 15 or ≤ 20; push ≤ 0.15–0.21). They are replaced by the same HOLD(a)/(b)/(c) used above. |
| Nominal-PD push "lifting" HOLD(c) | All four treat it as an automatic flip | **Not automatic.** HOLD(c) is defined on flat_dr. Moving it to nominal PD after seeing data would be a non-conservative post hoc change, so it goes to the human as evidence. |

## Alternatives considered
| Option | Why not chosen |
|---|---|
| +2000 hardware-eligible as registered | It clears a policy that falls on about 49 % of 0.65 m/s and about 15 % of 0.5 m/s standing shoves, about 5× P. |
| +500 / +1000 / +1500 | Post hoc (not on the menu), and all are worse on push (0.53–0.80). +1500 ties +2000 on falls. |
| P | Fails HOLD(a); it is not a Part 3 policy. |
| Flat control +2000 as the candidate | Post hoc, not the Bumpy pass, single seed. It goes to the human re-decision instead (above). |

## Severe flagged issues
- HOLD(c) is a post hoc rule (conservative-only) and must be ratified by the human.
- Push was never measured at nominal PD for any policy.
- The Bumpy-vs-control robustness claim is band-level at +2000.
- The control has no seed 4321 at +2000 and no export audit.
- The WIDEN check at the literal +1500 point has a 1-fall margin.

## What would flip this decision
- **Human or TA rules standing push out of scope** (e.g. the robot is tethered or on a stand whenever near people): +2000 becomes eligible under (b) as registered.
- **Two-seed nominal-PD push** with +2000 ≤ 0.209: evidence for the human to lift HOLD(c).
- **Control +2000 seed-4321 confirm** that passes HOLD(a)/(b)/(c) and the export audit: the control becomes the hardware-preparation option (labelled deviation). If it fails, nothing is hardware-eligible.
- **A Bumpy checkpoint** reading ≤ 0.209 on two seeds while keeping its HOLD(a)/(b) margins (none measured so far).
