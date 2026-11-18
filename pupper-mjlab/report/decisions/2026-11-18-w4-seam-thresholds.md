# Part 3 seam test: what counts as a stumble, and what pass/fail rule decides "stumbles at the seam"?
Status: DECIDED by arbitrator (pending human review) · 2026-11-18 · Requested by: W4 (Part 3) · Graders: [grader-1](2026-11-18-w4-seam-thresholds/grader-1.md), [grader-2](2026-11-18-w4-seam-thresholds/grader-2.md) (agree on (a), differ on (b))
Criticality: Medium. It sets the Part 3 seam verdict in the report; nothing downstream depends on it.
Reversible?: yes, at 0 GPU cost. Raw arrays are saved, and `part3_seam metrics` re-scores any run on CPU.

## Conclusion
**(a) Stumble definition (unchanged; both graders):** window [−0.2 s, +1.0 s] around the crossing, invalid if the env reset < 1.0 s before the window starts. **Incident** = fall OR (knee contact OR torso contact OR body tilt > **30°**). Score incidents, not `stumble` (which excludes falls).
**(b) Rule for `verdict(metrics, criteria)`, applied per condition:**
1. **Control:** all `anchor != "seam"` windows (`control_all`, both directions and sham levels pooled). c = incidents, n_c = windows.
2. **Cells (8):** block ∈ {`sweep` = kind sweep with rate ≠ inf (0.25/0.5/1 pooled), `dither`} × direction ∈ {fwd, back} × crossing ∈ {up, down}. Parent n ≈ 120 (sweep) and 160 (dither).
3. **FAIL** if either holds: (i) pooled `seam_sweep` or `seam_dither` **fall rate > 0.02**; (ii) any cell has **incident rate − c/n_c > 0.05 and** one-sided Fisher exact p < **0.01** (`scipy.stats.fisher_exact([[k, n−k], [c, n_c−c]], alternative="greater")`). A cell with n < 60 or n_c = 0 gives NO DATA.
4. `seam_sweep_step` and `seam_sweep_r05` are reported, not judged.
5. **Condition roles:** `flat_dr` is the **headline** ("does it stumble at the seam?"). `bumpy_dr` gets its own **robustness** verdict, printed next to the headline and not merged into it. `flat_nodr` and `bumpy_nodr` are scored the same way and reported as **diagnostics**. A diagnostic FAIL must be stated in the report text, but it does not change the verdict.
6. **Close calls:** if a gating cell (flat_dr or bumpy_dr) has an excess in [0.03, 0.07], rerun that condition with a second `--seed` and judge on the pooled windows of both runs.

`SEAM_CRITERIA = dict(max_fall_rate=0.02, max_incident_excess=0.05, fisher_alpha=0.01, min_cell_n=60)`. Store the 8 cells in `_summary()` so they survive without `--keep-windows`.

## Evidence used (my recomputation: `scratchpad/graders/arbitrator-seam/{cells.txt,null_sim.txt}`)
- **Both graders' numbers reproduce exactly.** There is no factual dispute. On flat_nodr the dither up crossing has 25/320 incidents (grader-1's 7.8 %). 22 of those 25 are in the back-up cell: 22/160 vs 0/320 control, p = 1e-11 (grader-2's 13.8 %). Fwd-up is 3/160. Back-up events are tilt 31–42° (22) plus 1 knee contact.
- **Final rule on the parent** (max cell excess, p): flat_dr **PASS** (0.008, p = 0.27). bumpy_dr **PASS** (sweep fwd-down 4/118 vs 2/319: 0.028, p = 0.048). flat_nodr **FAIL** (dither back-up 0.138; sweep fwd-up 0.058, p = 1e-4; fwd-down 0.067, p = 3e-5). bumpy_nodr **FAIL** (dither back-up 16/160 vs 6/319: 0.081, p = 1.3e-4). Pooled falls are ≤ 0.6 % everywhere.
- **The direction split matters.** On bumpy_nodr, grader-1's direction-pooled rule gives 19/320 − 6/319 = 0.041 (PASS). The per-direction cell is 0.081 (FAIL). The same failure is hidden by the same dilution that grader-1 itself used to justify splitting up and down.
- **Statistical behaviour of the rule** (simulated with 8 cells under a null of equal rates, at parent n): the family-wise false-FAIL rate is 0 / 0.03 % / 0.7 % / 2.2 % at a true rate of 0 / 1 / 2 / 3 %. At 80 envs per schedule it is ≤ 0.1 %. Power for a 10 % cell vs a 2 % control is 0.86 (0.96 at 80 envs per schedule). The Fisher gate costs almost no power. It only bites when the control is non-zero (bumpy).
- Control max tilt is 17.5–23.4° in all 4 conditions (1,278 windows). Seam events reach 27–45°. The step-sweep incidents are almost all down crossings (the 1.5→0 hard stop), as both graders found.

## Disagreement & resolution
| Point | Type | Resolution |
|---|---|---|
| Stumble definition, window, limits 0.02 / 0.05, incidents not stumbles, step sweep not judged, DR-off as diagnostic | — | Agreed; adopted. |
| Dither split by crossing (G1) vs by direction × crossing (G2) | Criteria; facts agree | **G2.** Forward and backward are different gaits with different seam mechanics. The evidence shows a one-direction failure (22 vs 3 events). Splitting by direction follows the same logic G1 used for up vs down. |
| Judge `r05` only (G1) vs all finite rates pooled per cell (G2) | Criteria | **G2.** Per-rate cells are no worse at r1 than at r0.25. Pooling triples n, so a 0.05 excess means about 6–8 events instead of 2. |
| Fisher p < 0.01 gate (G2) vs none (G1) | Criteria | **Adopt.** It matches the project's §3 rule that small gaps are ties, keeps the 8-cell family-wise false-FAIL rate under 1 %, and costs about 0 power when the control is near 0. |
| bumpy_dr: AND-gate on the headline (G1) vs a separate verdict (G2) | Criteria (spec reading) | **Separate.** The spec asks the seam question, then names Bumpy + DR as "the robustness pass", which is a different check. Bumpy controls are noisy (2–6 incidents/319). Both verdicts are always printed together, so a bumpy failure stays visible. |
| bumpy_nodr "borderline pass" (G1) vs FAIL (G2) | Follows from the cell choice | FAIL under the adopted cells. It is a diagnostic either way. |

## Severe flagged issues (union of both graders)
- **The parent fails both DR-off conditions on a backward trot→fast pitch transient.** It passes both gating conditions. The report must say "seam-sensitive at nominal physics (backward up-crossing)", not just "PASS". This is a sim-to-real risk.
- **The current `verdict()` (pooled blocks, `stumble`) is not fit for purpose.** Implement the rule above before scoring the final policy. Freeze `SEAM_CRITERIA` before looking at final-policy numbers.
- **The dither has no step-matched control** (the control uses 0.5 m/s² ramps). This is harmless while the control is about 0, but not guaranteed for the final policy.
- **n is small and GPU rollouts are not bit-reproducible.** Thresholds were set on the Part 1 parent, not the final policy.

## (c) Before scoring the final policy
Run `--envs-per-schedule 80` (960 envs). Add step-matched dither controls (trot-only 0.2↔0.4 and fast-only 0.8↔1.0) to `control_<dir>`. Print 25°/35° sensitivity lines next to the verdict. Optionally add the within-env pre-crossing window ([−1.4, −0.2] s) as a second control.

## What would flip this decision
- The final policy's control max tilt exceeds about 25°: re-derive the threshold (control max + 5°).
- The step-matched dither control shows similar incident rates: treat the dither cells as step effects and judge on the sweep cells only.
- Seam events appear after +1.0 s: lengthen the post window.
- The human treats nominal physics as the closer hardware proxy: make `flat_nodr` gating (the parent then FAILs).
