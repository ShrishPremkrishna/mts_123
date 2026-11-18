# Are the Part 2 extended-eval metric definitions (fall categories, push test, foot force) fit to rank Part 2 policies?
Status: DECIDED by arbitrator (pending human review) · 2026-11-18 · Requested by: W2
Criticality: Medium. It shapes how Part 2 arms are ranked; re-scoring every Part 2 checkpoint costs about 1–2 GPU-hours.
Reversible?: yes. Changes are in `part2_eval.py` only; any checkpoint can be re-scored (suite about 70 s, push about 2–4 min).
Grader docs: [grader-1](2026-11-18-w2-eval-metrics/grader-1.md), [grader-2](2026-11-18-w2-eval-metrics/grader-2.md). They agree on (a) and (c) and disagree on (b).

## Conclusion
**(a) Keep the fall categories.** Non-blocking addition: log each fall's episode age and time since the last command change, and report falls that happen less than 2 s after a reset as a separate count. The categories themselves stay as they are.
**(b) Change the push test.** Use magnitudes **{0.5, 0.65, 0.8} m/s** (drop 1.0) with **128 envs per condition** (512 envs, about 340 pushes per condition×magnitude cell). Keep the mechanism, the 2 s window and the reset exclusion. **Ranked:** (1) standing falls/push at one pre-registered *standing rung*, and (2) moving falls/push at 0.5 m/s (pooled over the three moving conditions, with each condition also reported). Every other cell, and the pooled `push`, `push_stand`, `push_move` and `push<m>` columns, is informational only. The standing rung is fixed by a pilot before any Part 2 scoring (see Thought process).
**(c) Keep the per-env actual-mass normalization and the 1 s post-reset exclusion.** Stop ranking on the single-sample peak. **Rank on the median over the 384 robot-runs of each run's max**. Report p99.9 and p99 next to it, and keep `F_peak_BW` and `peak_where` as a flag only.

## Evidence used
- `*_push.json`: standing robots fall 0/256 and 2/255 times at 0.5 m/s and 257/257 at 1.0 m/s for both baselines. The pooled rate is 0.452 vs 0.453. At 0.5 m/s while moving there is signal: 0.12 vs 0.27 at −1.5 m/s. At 1.0 m/s, −1.5 m/s is near ceiling (0.82 / 0.95). The impulse self-check gives 0.46 / 0.93 m/s. Wall clock is 130 s at 256 envs × 1500 steps (about 87 ms/step); the suite takes 63–70 s at 384 envs × 1000 steps. Step latency, not env count, dominates both.
- `REPRO_PLAN.md` §2.5 / §3: the target finding is a standing push-fall rate in the 5–10 % range (5.4 % vs 6.7–10.4 %), from a lost protocol (`A_code_recovery.md` l.151). Gaps under about 10 % are noise.
- Suite JSONs: cont4500 falls are reversal 11, stop 64, steady 3, turn 0 (`stops_fwd` 42, `mixed` 23, `steady_fwd` 3). Wandb has 5 falls, all reversal. Event exposure is identical across policies (reversal 192, stop 168).
- Foot force: the peak is 4.9–8.6 BW across identical reruns while p99 stays at 1.67 (README). Wandb's 8.57 BW peak is a single sample at a 0.835 m/s ramp, from a policy that barely moves. Standing total is 1.042 / 1.044 BW. Mass is 3.06–3.84 kg (mean 3.455 vs 3.17 nominal) and the draws are identical across policies. The incl/excl-reset peaks are equal, and p99 differs by 0.02 or less.

## Thought process
The criteria (REPRO_PLAN §3) are: a metric must measure the intended sim-to-real failure, vary between policies, and have less run-to-run noise than the gaps acted on. (a) passes all three. In (c), the normalization is physically right (standing check 1.04 BW; nominal mass would read about 1.13) and the exclusion is harmless. The peak fails the noise criterion, while the median of per-run maxima still targets impacts and averages 384 extremes instead of taking one.
**(b) protocol.** Both baselines go from 0 % at 0.5 to 100 % at 1.0 while standing, so the standing transition lies somewhere in (0.5, 1.0) and nothing locates it more precisely. 0.65 and 0.8 sample it twice for the cost of one rung, because dropping 1.0 frees that rung's pushes. The 1.0 rung adds no standing information, and for −1.5 m/s it is near ceiling. 0.5 stays as the moving-gait anchor and keeps continuity with the baseline numbers already measured. With 128 envs per condition, each cell gets about 340 pushes (SE ≤ 0.027), so a 10 % gap is about 2.6 SE of the difference.
**Pilot (frozen before Part 2 scoring).** Run both baselines twice under the new protocol (4 runs, about 15 GPU-min). This also measures push repeat spread. The standing rung is whichever of {0.65, 0.8} gives a mean baseline standing rate in 5–60 % and closest to 25 %. That leaves room in both directions, since the hypothesis is that stand-still terms make standing *worse*. If neither rung qualifies, shift the two middle rungs into the observed transition and repeat the pilot. If the 512-env run takes more than 200 s, use 96 envs per condition (about 255 per cell). Record the frozen rung in the code constant and in this file's Status line.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| Keep {0.5, 1.0}, rank per cell | Standing stays blind (0 % / 100 %), and the §2.5 finding cannot be tested |
| G2: {0.5, 0.75}, 64 envs per condition | One middle rung is a guess at an unlocated threshold and may land on another cliff. The "~1000 pushes per rung" precision is pooled over conditions; a ranked cell gets about 255 |
| G1: {0.5, 0.65, 0.8, 1.0}, 128 envs per condition | 1.0 is saturated for standing and near ceiling at −1.5; it spends 25 % of the pushes on little information |
| Per-env magnitude sweep with a v50 estimate | Better in principle, but more code and less comparable to the old falls/push figures. Use it as the fallback if the pilot fails twice |
| Rank foot force on p99 only, or on nominal mass | p99 misses impacts (§2.5 tracks a 6.1 BW spike). Nominal mass fails the standing check; it gives the same ranking but the wrong absolute BW |

## Severe flagged issues
- The pooled push columns and the single-sample `F_peak_BW` are **not fit for ranking as implemented**. Fix them, then re-score both baselines with the same code before scoring any Part 2 checkpoint (G1, G2).
- **The Part 1 pick's peak gap (5.9 vs 6.7 BW) lies inside a single policy's 4.9–8.6 BW rerun range.** A peak-based re-pick is noise (G1, G2).
- GPU rollouts are not bit-reproducible (62–78 falls on identical runs). Gaps under 10–15 falls are ties. The repeat spread for the push test and the new foot-force statistic is unmeasured until the pilot. Close calls need repeats or the second seed (G1, G2).
- Old push and foot-force numbers (5.4 %, 6.7–10.4 %, 5.9/6.7/6.1 BW, "x of 160") come from lost protocols with unknown normalization. Compare them in direction only (G1, G2).
- Contact force is sampled at 50 Hz (1 in 5 physics steps), so impact transients are aliased low. The bias is the same across policies (G1, G2).
- Arbitrator: moving-condition push rates are confounded by achieved speed (wandb reaches only about 0.25 m/s at a ±1.5 command). Report achieved vx next to the moving rates.

## Disagreement & resolution
- **Factual:** none on the data; both graders read the same counts correctly. Two claims were checked and qualified. G2's statement that "0.75 brackets the transition" is an assumption, because the data bound the transition only to (0.5, 1.0). G2's "~1000 pushes per rung" is pooled over conditions, not per ranked cell. G1's "about 4 min at 512 envs" is unmeasured; the per-step timings suggest it is plausible, and the 200 s fallback covers the risk.
- **Criteria:** G1 weighs robustness to an unknown threshold (ladder plus pilot); G2 weighs per-cell precision and simplicity (2 rungs). Resolution: a 3-rung ladder without 1.0 keeps G1's two probes of the transition and G2's removal of the saturated rung. 128 envs per condition gives better per-cell precision than either proposal's cells (about 340 vs about 255), and a pre-registered single standing rung keeps the ranking as simple as G2 wanted.
- **(c) wording:** G1 ranks on p99.9 plus median-per-run-max next to p99; G2 ranks on median-per-run-max (plus p99.9). This is resolved to a single ranked impact statistic, the median per-run max, to avoid picking among correlated metrics after seeing the data.

## What would flip this decision
- (b) Both pilot attempts fail to put a rung in 5–60 % standing: switch to a per-env magnitude sweep and a standing v50.
- (b) The pilot's push repeat spread exceeds about 10 % per cell: rank only with repeats, or raise the envs.
- (c) The median per-run max spreads across reruns about as much as the peak does: rank on p99.9, or add per-substep max capture.
- (a) The age tag shows more than about 20 % of category falls within 2 s of a reset: exclude those falls from the categories.
