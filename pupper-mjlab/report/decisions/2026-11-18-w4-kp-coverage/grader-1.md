# Should the Part 3 Bumpy fine-tune change its PD-gain DR coverage and/or gate on nominal gains?
Status: DECIDED by grader (grader-1 of 3; pending consolidation and human review) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: High. It sets the training distribution of the one run that produces the policy meant for the robot, and it addresses the largest sim-to-real flag so far.
Reversible?: yes, at a cost. Re-running both arms with the exact Lab 4 ranges costs 2 × 1500 iterations (~40 GPU-min of the shared pie) plus ~1 h of evals. No hardware is touched.

## Conclusion
**(D) Widen to a superset of the Lab 4 ranges and add a nominal-PD gate.** Both the Bumpy fine-tune and its matched flat control train with **kp ×[0.6, 1.3] and kd ×[0.6, 1.5]**. Friction stays at [0.6, 1.4], and the other task DR is unchanged.
Add the condition `flat_nompd` (kp = kd = 1.0×, other DR on; the existing `ablate pd_gains` condition) to the evals and to selection. Evaluation for the spec still uses the exact Lab 4 ranges (`flat_dr`, `bumpy_dr`).
**Pre-launch exception:** if the Part 2 final at +0 already has `flat_nompd` falls within 15 of `flat_dr`, the flag has gone away. In that case keep the Lab 4 ranges and keep only the gate (option B).

## Evidence used
- Notebook §4 ablation (`report/part3/eval/p3-parent/ablate/ablate.md`). This is the Part 1 parent, flat, 384 episodes, eval seed 1234.
  - Setting **pd_gains alone to nominal: 99 falls vs 40** with all DR on, 88 of them in stop contexts.
  - Speed overshoots: vx ±1.63 vs +1.54/−1.50. Saturation at 1.5 m/s: 23.0 % vs 17.9 %.
  - Every other event set to nominal lands at 21–50 falls. The 59-fall gap is far above the ~10–15-fall noise band.
  - With all DR off, the seam FAILs: dither back-up excess +0.138, p = 1e-11.
- `dr.pd_gains` (`envs/mdp/dr/actuator.py`) draws **one independent multiplier per joint (12 per env)**. The event is `mode="startup"` (`pupper_env_cfg.py:353`), so a run trains on 4096 robots fixed at launch. The course notebook says "sampled per episode", which is wrong.
- **Monte Carlo of the 12-joint mean** (my scratch `graders/grader-1-kp/mc.txt`):

  | Ranges | Mean kp across 12 joints | P(mean kp ≥ 1.0) | Envs with mean kp ≥ 0.95 and mean kd ≤ 1.05 |
  |---|---|---|---|
  | Lab 4 | 0.85 ± 0.042 | 1.1e-4 (≈ 0.5 of 4096 envs) | ≈ 1.4 envs |
  | Proposed | 0.95 ± 0.058 | 0.20 | 25 % of envs |

  - Mean kd across 12 joints is 1.15 ± 0.058 under Lab 4. Nominal 1.0 sits 2.6 σ below it.
  - With the proposed ranges, 71 % of per-joint kp draws still fall inside the Lab 4 range.
  - So 1.0× is *inside* the per-joint Lab 4 range, yet a robot at nominal gains is essentially absent from training.
- Deploy side:
  - The policy commands `policy.json` kp 5.5 and kd 0.25 (= 1.0×). `robot_trims` `kp_scale` defaults to 1.0. Changing it on the robot is a human decision.
  - The new kp 1.3× works out to 7.15. That is sim-only and stays below `kp_max` 10.
  - The export carries nominal gains, not the DR draws.
- Hardware: there is **no measurement** of effective motor stiffness. `course-capability-coverage.json` says "Actual gains … remain untested on this robot", and the rear under-tracking has no identified source (`generated-motion-clearer-reference-20261101.json`).
  - The robot-side sim models back-EMF torque limits (`actuator_model.hpp`, kt 0.04, R 0.7, 24 V) that mjlab does not. That plausibly motivates the course's sub-1.0 kp bias, but nothing has verified it.
- Spec: "Run the Bumpy variant with your lab 4 DR ranges as the robustness pass." The Lab 4 record (`training-settings.json`) equals the course notebook defaults.

## Thought process
1. **Rank on sim-to-real readouts** (REPRO_PLAN §3).
   - The deployed gain is a known quantity. The real motor's effective gain is not, and could sit anywhere from about 0.85× (the course's implied belief) to 1.0× (as commanded).
   - A robustness pass should cover both ends rather than bet on one. The Lab 4 policy is 2.5× worse at the commanded value.
2. **Spec fidelity.**
   - A superset keeps every Lab 4 draw in the training distribution.
   - The spec's evaluation conditions stay exactly Lab 4, so the deviation is small and reportable.
   - Recentring, for example to kp [0.75, 1.25], would drop the weak-motor coverage the course chose deliberately, so I rejected it.
3. **Cost and budget.**
   - Training costs the same 2 × 1500 iterations. The extra evals take ~10–15 GPU-min.
   - Gating alone (B) cannot fix anything: every candidate shares the same training distribution. If B's gate failed, you would pay for a second full run anyway.
4. **Matched control** (§3). The control uses the same widened ranges, so Bumpy vs flat stays the only difference.
5. **kd.**
   - The ablation does not separate kp from kd.
   - Nominal kd is 2.6 σ out in aggregate, so widening kd costs little and removes that ambiguity.

**Gate (fixed before launch):**
- **Evals.** Run `flat_nompd` at +0, +500, +1000 and +1500, and on the control's end checkpoint. On the shipped candidate also run `bumpy_nompd` and the `flat_nompd` seam. Include `flat_nompd` in the seed-4321 confirm.
- **Fallback.** Add `flat_nompd` to the "worse by > 10 falls" fallback list. That is, fall back to +1000 if +1500 is worse by more than 10 falls there.
- **Hardware readiness of the shipped checkpoint.** It needs a `flat_nompd` seam verdict of PASS, and `flat_nompd` falls no more than `flat_dr` falls + 15.
  - If either fails, re-decide; do not ship silently to hardware.
  - If both pass, record it in hardware checklist item 5.
- **Smoke** (≤ 50 iterations, both arms) must show all of the following:
  - The config check reports kp (0.6, 1.3), kd (0.6, 1.5) and friction (0.6, 1.4). The Lab 4 record file is byte-unchanged.
  - The `check_lab4_dr` eval path still asserts the exact Lab 4 ranges.
  - The realized `actuator_gainprm` / 5.5 spans ≈ 0.6–1.3, and ≈ 20 % of envs have a 12-joint mean kp ≥ 1.0. The same check holds for kd.
  - Overflow is 0 and there are no NaNs. Episode length and err_xy are not collapsing relative to the Lab 4 smoke `c3rfre1p` (~470/500, ~0.23).
  - The auto-exported `policy.json` still says kp 5.5 and kd 0.25, and the round-trip and limit audit PASS.
  - The `flat_nompd` eval and the extended `select` run end to end. This is a pipeline check, not a result.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| (A) Keep Lab 4, report only | It ships a policy known (on the parent) to fall 2.5× more, overshoot speed and fail the backward seam at the gain it actually commands. |
| (B) Keep Lab 4 + nominal gate | Measures but does not fix the problem, since all candidates share the deficient distribution. Kept as the fallback if the +0 check shows no gap. |
| (C) Recentre, e.g. kp [0.75, 1.25] | Drops the course's weak-motor coverage (0.6–0.75), and Lab 4 is no longer a subset. |
| kp-only widening, kd unchanged | Only 2.2 % of envs would be near nominal on both gains, and the ablation cannot rule kd out. |
| Trim kp on the robot to 0.85 | Changes a reviewed deploy value (human decision), and the robot's real gain is unknown. |

## Severe flagged issues
- **The evidence is from the Part 1 parent, one eval seed.** The Part 2 final may differ, which is why the pre-launch +0 check can revert to B.
- **Spec deviation.** Training uses a superset of the "Lab 4 DR ranges". State it in the report and evaluate under the exact Lab 4 ranges.
- **Confound.** Both arms change the DR. A nominal-gain gain vs +0 cannot be split between "wider DR" and "1500 more iterations" without a Lab 4-range control arm, which the budget does not allow.
- **Sim-only.** Which gain regime the real robot is in is a hardware fact. Neither range is hardware-validated. Seam fragility on backward speed-ups at nominal gains is a safety item for the hardware checklist.
- **kp and kd are not separated.** The ablation sets both to nominal together.

## What would flip this decision
- The Part 2 final at +0 has `flat_nompd` falls ≤ `flat_dr` + 15 → keep the Lab 4 ranges and only gate (B).
- A hardware stiffness or step-response measurement shows the robot behaves like ≈ 0.85× → Lab 4 is adequate (B).
- At +1500, Lab 4 conditions (`flat_dr`, `bumpy_dr`) are more than 10 falls worse than +0 while `flat_nompd` improves → a trade-off that needs a human decision.
- A TA or human rules that the spec requires the exact Lab 4 ranges → re-run both arms with Lab 4 (~0.7 GPU-h + evals) and keep the gate.
