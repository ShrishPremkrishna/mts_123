# Which checkpoint is the Part 3 sim candidate, is it hardware-eligible, and should eligibility also need a push condition?
Status: grader 2 of 4 (lens: raw data only; notebook and diag README not read) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: Critical. It feeds the Part 3 headline claims and decides what may be prepared for a physical robot.
Reversible?: yes for (a). Re-selecting from the saved checkpoints costs about 0 GPU. (b) and (c) are reversible only *before* any deployment. A hardware fall is not reversible.

## Conclusion
**(a)** The candidate is **Bumpy +2000** (`rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt`). The pre-registered guards pick it, and the raw numbers back that pick: it is best or tied-best on every falls readout and on foot force.
**(b)** Under the HOLD rule as written it is **CLEAR**. I recomputed it from the tables: (a) −1.0 vs 15, (b) −28.5 vs 10, WIDEN −2 vs 10.
**(c)** **Yes, add a push condition, for hardware only (not for sim selection).** The rule: HOLD if the two-seed mean `R:push_stand@0.65` on flat_dr exceeds P's two-seed mean by more than 0.10. **Under that rule +2000 is HOLD** (+0.38). It remains the sim candidate, but nothing should be prepared for hardware until a human re-decision.

## Evidence used
- **Falls** (`p3-final-kpwide_summary.md`, `_selection.md`, `ablate/*.md`), given as seed 1234 / 4321:

  | | flat_dr | bumpy_dr | nompd | F_runmax (BW) |
  |---|---|---|---|---|
  | +2000 | 10 / 11 | 18 / 23 | 7 / 12 | 2.60 |
  | P | 14 / 16 | 35 | 39 / 37 | 3.03 |
  | control +2000 | 12 | 29 | 20 | 2.85 |

  - sat@1.5 0.165 vs P 0.161, and tracking error is unchanged.
  - The seam verdict is PASS in all 4 conditions. The worst cell is dither_back_up, +0.020 (p = 6e-6) and +0.016 on seed 4321, below the 0.05 rule.
  - **Neighbours** (`diag/neighbours`): the leg-4 Bumpy checkpoints 6000–6450 read flat_dr 4–16 and nompd 6–17. Leg 2 reads 34–48 and 41–60. The +2000 falls advantage is a property of the whole leg-4 region, not of one lucky save.
- **Push, standing falls/push at 0.65 m/s on flat_dr.** Sources: all 26 `*_push.json`, tabulated in `scratchpad/graders/cand-g2/push_table.tsv`.
  - P: 32/334 = **0.096**, and 0.122 on seed 4321.
  - +2000: 160/327 = **0.489**, and 0.497 on seed 4321. It reads 0.486 on bumpy_dr.
  - Control: +1500 **0.024**, +2000 **0.091**.
  - Every other Bumpy checkpoint measured: +500 0.798, +1000 0.754, +1500 0.533, and the neighbours 5950/6050/6400/6450 at 0.856/0.710/0.563/0.721. The range is **0.49–0.86, and no Bumpy checkpoint gets below 0.48**.
  - At the 0.5 m/s rung: +2000 reads 0.151 / 0.158, while P reads 0.000 / 0.000 and control +2000 reads 0.003.
  - The Part 1 parent reads 0.455, so +2000 stands *worse than the Part 1 parent*.
  - Moving push @0.5 is 0 for every policy on flat (no signal).
- **Possible mechanism (correlate only).** Standing `hind_splay_rad_stand` is −0.16 to −0.26 across every Bumpy checkpoint. P reads −0.054 and the control −0.03 to −0.07.

## Thought process
**Criteria.** Sim-to-real readouts over reward (§3), and gaps judged against measured noise (§3; falls < 10–15 are ties).

- **Falls.**
  - +2000 vs P: flat_dr is a tie. bumpy_dr (−12 to −17) is borderline-real. nompd (−28.5, two seeds) is clearly real.
  - +2000 vs control: flat_dr ties, and bumpy_dr (−11) and nompd (−13) are borderline. The control was also noisy over legs (nompd 51 → 20).
- **Push.**
  - +2000 vs P is +0.38 falls/push. The SE of the difference is about 0.033 at ~330 pushes, so that is about 12 SE.
  - It is about 10× the band of a replicated Part 2 effect (0.040), and larger than the worst checkpoint-to-checkpoint swing seen (0.28).
  - It replicates across eval seeds, across terrain, and across all 10 Bumpy evals. The matched control does not show it, so the cause is the Bumpy training, not iterations or kp-WIDEN.
  - It is the largest, cleanest effect in the whole table.
- **(a)** Within the Bumpy arm, +2000 is also its *least*-bad push checkpoint. Switching to +500 would be post-hoc, and +500 is worse on push (0.80) and nompd (25), so there is no trade to make. Neither P nor the control is the spec's "Bumpy + Lab 4 DR robustness pass".
- **(c)** push_stand is a pre-registered **ranked** metric. The Part 2 final was chosen mainly for its standing-push gain (0.455 → 0.10), and +2000 erases that gain. A hardware gate that ignores the one hardware-relevant readout this policy fails is a gap, not a design choice. The threshold uses the same form as HOLD(b): a delta vs P on two-seed means. The 0.10 margin is about 2.5× the Part 2 band and about 3 SE. The verdict is the same for any margin from 0.05 to 0.35, so the threshold was not tuned to the result.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| +500 | Post-hoc (not a menu option). Worse on push (0.80), nompd (25), bumpy_dr (31 / 36) |
| P (no fine-tune) | Already HOLD on nompd (38, gap +22.5). It is not the Part 3 robustness pass |
| Control +2000 as candidate | Its push (0.091) and nompd (20) would pass both gates, but this is post-hoc, it is not the Bumpy pass, and it has a single eval seed. **Best input for the human re-decision** |
| (c) No push gate | Ignores a ~12-SE regression on a ranked safety metric for a physical robot |
| (c) Absolute cap (e.g. ≤ 0.25) | Workable, and gives the same verdict, but it is not anchored to the policy the HOLD rule already compares against |

## Severe flagged issues
- **The push gate is added after seeing the data.** It is defensible only because the metric was pre-registered as ranked and the verdict does not depend on the threshold. The human must ratify it.
- Push was never measured at **nominal PD gains**, for any policy. The hardware-relevant push risk of +2000 (and of the control) is unmeasured.
- Control +2000 has one eval seed for suite, nompd and push. Its bumpy_dr and nompd gaps to +2000 (11 and 13) sit at the noise band.
- Bumpy training damages standing lateral robustness at every checkpoint. That is a Part 3 result to report, not to hide.

## What would flip this decision
- Push at nominal PD gains, with ≥ 2 seeds, showing +2000 within 0.10 of P: drop the push HOLD.
- A human/TA rule that hardware tests never involve lateral disturbance while standing (e.g. tethered hallway runs only): the gate becomes informational.
- Control +2000 on seed 4321 with nompd ≤ ~15 and push ≤ 0.2: make it the hardware-preparation candidate, labelled as a deviation.
- A second push run of +2000 below 0.25: re-check against the noise estimate. Ten evals at 0.49–0.86 make this unlikely.
