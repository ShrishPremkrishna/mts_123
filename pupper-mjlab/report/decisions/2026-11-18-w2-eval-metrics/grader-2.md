# Are the Part 2 extended-eval metric definitions (fall categories, push test, foot force) fit for ranking?
Status: DECIDED by grader 2 of 2 (pending consolidation) · 2026-11-18 · Requested by: W2
Criticality: Medium. It shapes how Part 2 arms are ranked; a re-score costs about 1 GPU-hour.
Reversible?: yes. The changes are in `part2_eval.py` only; any checkpoint can be re-scored (suite + push take minutes per checkpoint).

## Conclusion
**(a) Fall categories: keep as is.** One non-blocking addition: tag each fall with its episode age and report falls less than 2 s after a reset separately.
**(b) Push test: change it.** Replace the 1.0 m/s rung with 0.75 m/s (magnitudes 0.5 and 0.75), and rank on the per-magnitude × per-condition rates. The pooled `push` / `push_stand` / `push_move` columns stop being ranking metrics.
**(c) Foot force: keep the per-env actual-mass normalization and the 1 s post-reset exclusion, but stop ranking on the single-sample peak.** Rank on a robust tail: the median over robot-runs of each run's max (plus p99.9). Keep p99 and peak as informational columns.

## Evidence used
- `part2/eval/*.json`, (a): every fall from the stop and reversal schedules lands inside its 2 s window. cont4500 has `stops_fwd` 42 + `mixed` 22 = 64 stop falls, and reversal 2 + 8 + 1 = 11. Only 3 falls are labelled steady (`steady_fwd`), and none are labelled turn. So the windows catch the failure they are meant to catch. Exposure is identical across policies (events: reversal 192, stop 168; robot-runs: 96 / 72). The two baselines separate clearly (stop falls 0 vs 64).
- (b) Push: 1.0 m/s fails 257/257 standing pushes for **both** baselines, and the pooled rate is 0.452 vs 0.453 for two very different policies. The 0.5 m/s standing rate is 0/256 vs 2/255, i.e. at the floor. Moving conditions do discriminate: 0.5 m/s at −1.5 gives 0.12 vs 0.27, and 1.0 m/s at +1.5 gives 0.75 vs 0.58. The impulse self-check works (applied Δv 0.46 / 0.93 after one step). Training has no pushes (`push_robot` popped; mjx kicks are 0.2 m/s). The old figures (5.4 % parent; 6.7–10.4 % with stand-still terms) imply a softer protocol.
- (c) Foot force: identical-seed reruns gave peaks of 4.9–8.6 BW (README), while p99 is 1.67 vs 1.67. Standing total is 1.04 BW in both runs, which confirms the normalization is correct. Mass is 3.06–3.84 kg (mean 3.46 vs nominal 3.17), and the eval seed gives both policies the same masses. The `*_incl_reset_drop` peak equals the filtered peak, and p99 shifts by ≤ 0.02. Forces are sampled at 50 Hz, i.e. 1 of every 5 physics steps (timestep 0.004, decimation 5), so impact transients are aliased.

## Thought process
The criteria come from REPRO_PLAN §3: rank on sim-to-real readouts; treat gaps under ~10 falls / ~10 % as noise; report max-statistics next to p99. A ranking metric has to (1) measure the intended failure, (2) vary between policies, and (3) have less run-to-run noise than the gaps we act on.
(a) passes all three. The context labels are attached at termination, and the falls land in the expected windows. The remaining risk is a robot re-spawned into a ±1.5 m/s hold that falls and is counted as reversal or steady (INDEX.md: fast_trot's falls were "mostly within 1 s of a reset"). That is cheap to expose with the age array that is already computed, but it does not change any category.
(b) fails criterion 2 as built. Half of all pushes sit at a rung where standing is a guaranteed fall and the other rung is near zero, so every pooled column is roughly constant. This blinds the eval to exactly the old finding to be reproduced: stand-still terms worsen standing push recovery. 0.75 m/s brackets the standing transition (between 0 % at 0.5 and 100 % at 1.0), and 0.5 stays as the anchor for moving gaits.
(c) The normalization and the exclusion are correct and harmless. Ranking on peak fails criterion 3: a 3.7 BW spread from noise alone dwarfs the 0.8 BW gap (5.9 vs 6.7) that once helped pick gallop C. p99 (≈ 1.3–1.7 BW) is stable but measures stance load, not impact. The median of per-run maxima still targets impact, but it averages 384 extremes instead of taking one.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| (a) Event-normalized rates or a wider 3 s window | Exposure is already identical across policies, and no falls leak out of the 2 s windows. Raw counts are easier to compare with the old "x of 160". |
| (b) Keep 0.5 and 1.0, rank on by-condition rates only | The standing comparison stays blind (0 % / 100 %). |
| (b) Ladder 0.25 / 0.5 / 0.75 / 1.0 | 0.25 is at the floor for both baselines. Prefer 2 rungs × ~1000 pushes for precision. Add 0.25 only if Part 2 policies prove much weaker. |
| (c) Nominal-mass normalization | Same eval seed, so the policy ranking is unchanged. Actual mass is physically right. Old BW figures were probably nominal (about +9 % offset), so do not compare them directly. |
| (c) Record per-substep max force | Fixes the aliasing but requires hooking the decimation loop. Worth doing if it is cheap; it is not needed for a robust ranking statistic. |

## Severe flagged issues
- The pooled push columns (`push`, `push_stand`, `push_move`) and the single-sample `F_peak_BW` are **not fit for ranking as implemented**. Fix them before scoring any Part 2 checkpoint, then re-score both baselines with the same code.
- Eval nondeterminism: identical runs gave 62–78 falls. Repeat-run spread for the push test and for the proposed foot-force statistic is unmeasured. Run ≥ 2 repeats per checkpoint (or measure the spread once) before treating a gap as real.
- The recovered old numbers (push 5.4 %, peak 5.9 / 6.7 BW, reversal falls of 160) come from lost protocols and are not comparable. Use them only qualitatively.

## What would flip this decision
- (b) A 0.75 m/s pilot on the baselines leaves standing at 0 % or 100 %: move the rung to the measured transition.
- (c) Repeat runs show the median-of-run-max spreading as much as the peak: then rank on p99.9 only, or add per-substep max capture.
- (a) The age tag shows a sizeable share (> ~20 %) of falls within 2 s of a reset: exclude those falls from the categories.
