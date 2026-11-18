# Which Part 2 checkpoint is the final policy that goes to Part 3? (grader 2, lens: raw data only)
Status: DECIDED by grader (pending synthesis) · 2026-11-18 · Requested by: W3
Criticality: Critical. It feeds the report's headline Part 2 claim and is the +0 of the Part 3 fine-tune that may be prepared for hardware.
Reversible?: yes before Part 3 launches (no cost). After that, swapping the final means re-running Part 3 step 0 plus both 1500-iteration arms (~40 GPU-min) and their evals.

## Conclusion
**Final = `r2-cont-1500`, seed 42: `~/runs/opt-lab-2/rp-p2-s42-r2-cont-1500/model_4496.pt` (WandB `wmhcp24q`), reference `configs/opt-lab-2/p2/s42/r2-lag` (commit `60ce5a4`).** It descends from the designated parent `1c5r0n8c`. It replaces the parent. r2-cont-1500 is the only +1500 arm that beats its matched control on falls and on push_stand@0.65 in both seeds' direction, and it has the lowest nominal-PD falls. I chose s42 over s43 because s42 is clearly better on push_stand, ties on falls and nompd, and has the stable nompd measurement.

## Evidence used
I rebuilt every number below from the raw JSONs: `eval/w3/*.json` and `*_push.json`, `eval/w3-rep/*-rep2*`, and `eval/w3-nompd/*/{*-flat_dr,ablate/*nominal_pd_gains*}.json`. They match `matched_summary.md`, `repeat_spread.md` and `nompd_check.md`. `chain_check.json` reports 0 violations. Every +1500 arm is 1500 iterations past its own root at 4096 envs, `reference_check: ok`. All fold summaries report `verify.ok`, with measured lag 1.87–2.13 steps.

**Arms at +1500.** Each cell is the two-seed mean of (arm − same-seed ctrl-1500), with the s42/s43 values in brackets. Suite and push cells average the original eval and its repeat. Noise bands are the max |diff| between repeat evals.

| Readout | r2-cont-1500 | r3 | r1-cont-1500 | ctrl-1500 level, s42/s43 |
|---|---|---|---|---|
| sat@1.5, gate (band 0.0006) | **−0.024** (−0.025/−0.023) | −0.028 (−0.026/−0.029) | −0.011 (−0.011/−0.011) | 0.186 / 0.181 |
| falls (band 13) | **−10.75** (−17.5/−4) | −2.75 (−8.5/+3) | −2.5 (−9.5/+4.5) | 34 / 19.5 |
| push_stand@0.65 (band 0.04) | **−0.277** (−0.203/−0.350) | −0.043 (+0.102/−0.187) | −0.110 (+0.012/−0.232) | 0.298 / 0.502 |
| push_move@0.5 | ≈0 (every arm ≤ 0.001) | ≈0 | ≈0 | 0 / 0.0005 |
| F_runmax_med_BW (band 0.076) | +0.084 (+0.186/−0.017) | **−0.090** (+0.002/−0.182) | +0.075 (+0.126/+0.024) | 2.89 / 2.97 |
| nompd falls, mean of eval seeds 1234 and 4321 | **37.5** (38/37) | 40 (46.5/33.5) | 46.5 (44/49) | 46.75 (62.5/31) |
| nompd − flat_dr | 20 (22.5/17.5) | 16.5 (24/9) | 19.75 (18/21.5) | 20.75 (29.5/12) |

**Choosing the seed within r2-cont-1500.** Values are s42 vs s43, written original/repeat.
- **Falls:** tied, 16/17 vs 15/16.
- **sat@1.5:** 0.160 vs 0.157.
- **push_stand@0.65:** 0.099/0.090 vs 0.148/0.155. The gap is ≈0.055, above the 0.04 band.
- **push_stand@0.8 (informational):** 0.525/0.541 vs 0.980/0.960.
- **F_runmax:** 3.04/3.11 vs 2.96/2.95. s43 is lower by ≈0.12.
- **nompd:** 39/37 vs 44/30. s43's two eval seeds differ by 14, just under the 15 "too noisy" line in kp-coverage.
- **flat_dr:** 14/17 vs 23/16.
- **Tracking:** vx at ±1.5 is 1.52/−1.54 vs 1.51/−1.54.

**Against the parent.** The s42 parent reads falls 43/44, sat 0.179, push_stand 0.455/0.470, and nompd 99/79 with flat_dr 40/41 (`part3/eval/p3-parent_kp_coverage.md`).

## Thought process
1. **Gate.** sat@1.5 is the hard gate. All three arms *lower* sat vs their control in both seeds, by 18 to 45 times the band. None fails, and the matched speeds are within ±0.06 m/s.
2. **Ranked readouts, two-seed means vs the matched control.** I counted only effects that are both outside the repeat band and not contradicted by the other seed.
   - r2-cont-1500 has the only push_stand@0.65 effect that holds in both seeds (−0.20 and −0.35, 5 to 9 times the band).
   - It also has the only falls mean beyond 10 with sign agreement. The s42 falls effect is −17.5; the s43 effect (−4) is a tie.
   - r3's single advantage is F_runmax, and it comes from s43 alone.
   - push_move@0.5 is saturated at 0 for everyone.
3. **Nominal PD (kp-coverage makes it part of selection).** r2-cont has the lowest two-lineage nompd mean (37.5). The arms differ by less than the single-seed noise, though, so I count this as "not worse", not as a win.
4. **Seed.** Falls, sat and nompd tie. push_stand is the one ranked readout that separates the two seeds beyond the band, and it favours s42. F favours s43 by 0.12 BW (≈4 %, about 1.5 times the band). I weight the push_stand gap above that, as the W2 metric doc ranks it.

   Two non-metric tie-breakers also point to s42:
   - s42 descends from the designated parent.
   - Its nompd reads stably across the two eval seeds (39/37 vs 44/30).

   Both seeds give step-0 Δ > 15 (22.5 and 17.5), so the Part 3 WIDEN branch is the same either way.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| s43 r2-cont-1500 | Ties on falls and nompd. Worse push_stand@0.65 (+0.055 over the band). Its nompd is noisy across eval seeds (44 vs 30). Lower F_runmax is its only edge. |
| r3 (either seed) | Falls tie (−2.75). The push_stand seeds disagree (+0.10/−0.19). The F win is s43-only. s42-r3 nompd is 46.5 and s42-r3 push_stand@0.8 is 0.99. A third round bought nothing over continuing r2. |
| r1-cont-1500 | Falls tie. push_stand disagrees between seeds. Highest nompd of the arms (46.5). |
| ctrl-1500 | It is the baseline. Every Part 2 arm beats it on sat@1.5. s42-ctrl nompd is 62.5. |
| Keep the parent | r2-cont s42 is better on every ranked readout and on the gate (falls 16.5 vs 43.5; push_stand 0.095 vs 0.463; nompd 38 vs 89). |
| r1-naive +500 (best +500 arm: −15.5 falls, −0.29 push_stand) | Not continued to +1500, no nompd, no repeats. It cannot be compared at matched iterations. |

## Severe flagged issues
- **The falls win is one level and one seed deep.** At +1000 the parent run r2 was *worse* than its control (+3/+15, mean +9). At +1500 the falls effect is −17.5 for s42 and −4 for s43. Only the push_stand advantage of the lag lineage is consistent at every level and in both seeds: −0.20 at +500 (r1-lag), −0.28 at +1000, −0.28 at +1500. The headline should rest on push_stand and sat, not on falls.
- **Nominal-PD robustness is not fixed by Part 2.** For the pick, nompd − flat_dr = 22.5 (> 15). Part 3 step 0 → WIDEN, and HOLD rule (a) would fire unless Part 3 closes the gap. Every arm, the control included, sits at 16.5–20.75.
- **F_runmax rises for s42 r2-cont vs its control** (+0.19, outside the band). The value is still at the parent's level (3.04/3.11 vs 3.04/2.96). Watch it in Part 3.
- **Noise.** Every suite and push number uses eval seed 1234, so the repeats measure GPU nondeterminism only (s43-r3 swung 16 → 29 falls). There is one training seed per lineage.
- Much of the gain over the parent also appears in the control (s42 ctrl falls 34, nompd 62.5). The parent-to-final gap is not a Part 2 effect; only the arm − ctrl deltas are.

## What would change my mind
- A W2 suite plus push run on a second eval seed (e.g. 4321) for the +1500 arms that shrinks s42 r2-cont's falls delta into a tie **and** flips its push_stand@0.65 lead over s43 r2-cont or over r3. Then the pick reverts to the safest gate and nompd candidate.
- A repeat nompd for s43 r2-cont, or a third eval seed, putting it more than 10 falls below s42 r2-cont while push_stand stays within 0.04. Then I would pick the s43 seed.
- Part 3 step 0 on this checkpoint measuring nompd more than 15 away from 38. Re-measure, and re-rank the arms on nompd.
- A training-seed replicate of s42 r2-cont-1500 (same parent) whose falls delta vs ctrl is near zero and whose push_stand delta is above −0.1.
- A +1500 continuation of r1-naive that beats r2-cont-1500 on falls by more than 15 **and** on push_stand by more than 0.04, at no worse sat@1.5 or nompd.
