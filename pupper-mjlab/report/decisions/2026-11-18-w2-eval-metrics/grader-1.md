# Are the Part 2 extended-eval metric definitions (fall categories, push test, foot-force normalization) fit to rank Part 2 policies?
Status: DECIDED by grader 1 of 2 (pending consolidation) · 2026-11-18 · Requested by: W2
Criticality: Medium. It shapes how Part 2 policies are ranked; re-scoring costs about 1 GPU-hour.
Reversible?: yes. Each policy takes about 70 s for the suite and 130 s for the push test, so re-scoring all Part 2 checkpoints costs under about 1 GPU-hour.

## Conclusion
(a) **Keep the fall categories as they are.** (b) **Change the push test before scoring:** replace the {0.5, 1.0} m/s magnitudes with a ladder {0.5, 0.65, 0.8, 1.0} m/s, double the envs to 128 per condition, and stop ranking on the pooled `push` rate. Rank on standing falls/push at the middle magnitudes and moving falls/push at 0.5 m/s. (c) **Keep per-env actual mass and the 1 s post-reset exclusion, but stop ranking on the single-sample peak.** Add two robust tail statistics, p99.9 and the median over robot-runs of each run's max, and rank on those next to p99. Keep `F_peak_BW` and `peak_where` as a reported flag only (§3).

## Evidence used
- `part2/eval/*_push.json`: standing robots fall 0/256 and 2/255 times at 0.5 m/s, and 257/257 at 1.0 m/s, for both baselines. The overall `push` rate is 0.452 (wandb-2999) vs 0.453 (cont4500), although the two policies behave very differently. At 0.5 m/s while moving there is some signal: 0.06 vs 0.10 overall, and 0.12 vs 0.27 at −1.5 m/s.
- `REPRO_PLAN.md` §2.5 and `A_code_recovery.md` l.151: the finding to reproduce is a standing push rate of 5.4 % vs 6.7–10.4 %, a mid-range rate. The old protocol (`commands_push.json`) is lost, and training has pushes disabled (`B_repo_map.md` l.340), so 0.5/1.0 m/s carries no recovered provenance.
- Eval README: peak foot force ranged 4.9–8.6 BW across four identical suite runs of the same checkpoint, while p99 was stable at 1.67. The Part 1 pick relied on a peak gap of 5.9 vs 6.7 BW.
- Suite JSONs: mass is 3.06–3.84 kg (mean 3.455) vs 3.17 kg nominal. Total standing force is 1.044 and 1.043 BW, a correct normalization check; nominal mass would give about 1.13. The mass draws are identical across policies (same seed). Excluding the first 1 s after reset changes nothing for these baselines: peak incl./excl. are equal, and p99 differs by 0.02 or less.
- Fall categories (`part2_eval.py` `label_schedule`): there are 192 reversal and 168 stop events over the 384 robot-runs. Only 3 of 78 falls (cont4500) and 0 of 5 (wandb) land in `steady`, so little leaks out of the 2 s windows. For cont4500, 42 of the 64 stop falls come from `stops_fwd` and 0 from `stops_back`, which is consistent with its −0.15 m/s backward speed.

## Thought process
The governing principle is §3: rank on sim-to-real readouts that **discriminate between policies**, and treat noise-sized gaps as ties. I graded each metric on validity (does it measure the intended thing?), discriminating power, and noise.

**(a)** The labels are mechanically correct. Ramps to zero and the 0.5→−1.0 flip are excluded by design, the priority order reversal > stop > turn > steady is documented, and leakage into `steady` is small. Raw counts are comparable because the schedule is fixed and hashed. The weakness is run-to-run noise (62–78 total falls; 56–64 stop falls). That is a ranking-rule issue the README already handles (gaps under 10–15 falls are ties). The definitions are fine.

**(b)** A metric that saturates at 0 % and 100 % for standing cannot reproduce the 5–10 % standing finding. The pooled rate averages a floor arm and a ceiling arm, which is why two very different policies tie at 0.452 vs 0.453. Mechanism and protocol are sound: the code adds to the current velocity rather than overwriting it, a one-step dv self-check runs, the fall window is 2 s, pushes within 2 s of a reset are excluded, and only 0–2 falls happen outside push windows. So only the magnitudes and the headline need to change. With 128 envs per condition, each condition×magnitude cell gets about 256 pushes, so a 50 % rate has a standard error of about 0.03, and the run costs about 4 min. **Pilot on the two baselines:** if standing rates at 0.65 and 0.8 are both below 5 % or both above 95 %, move the two middle levels within (0.5, 1.0) and freeze them before any Part 2 scoring.

**(c)** Per-env mass is correct, as the 1.04 BW standing check shows. Because the mass draws are shared across policies, ranking would not change either way, but the absolute BW values would be off by about 9 % with nominal mass. The 1 s exclusion is harmless insurance against the spawn drop. A single-sample max that varies by 3.7 BW across identical runs cannot rank gaps of about 1 BW. p99 is stable but pooled over all stance samples, so it reads typical loading at speed and misses rare hard landings, which are the reason §2.5/§3 track peaks. A per-robot-run max with a median over 384 runs, and p99.9, keep sensitivity to impacts while averaging out single-event luck.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| Keep push at {0.5, 1.0} and rank on the 0.5 m/s rate | Standing at 0.5 m/s is about 0 % for both baselines, so the standing stand-still trade-off from §2.5 becomes invisible |
| Replace 1.0 with a single 0.75 m/s | The standing threshold is unknown, and one middle level risks another cliff. The ladder plus pilot costs only minutes more |
| Rank foot force on p99 alone | Blind to impact spikes such as the 6.1 BW round-3 spike, which §2.5 asks us to reproduce |
| Normalize by nominal mass (old method?) | Fails the standing check (about 1.13 BW). Ranking is unchanged anyway because the draws are shared |
| Widen or narrow the 2 s fall-context windows | No evidence of leakage (0–3 steady falls). Optional: log time-since-onset for each fall to confirm |

## Severe flagged issues
- **The Part 1 pick's peak gap (5.9 vs 6.7 BW) falls inside the 4.9–8.6 BW run-to-run range of a single policy.** If the new run re-picks on peak, that evidence is noise.
- GPU rollouts are not bit-reproducible, so fall gaps under 10–15 are ties. Close Part 2 calls need repeat runs or a second training seed.
- The old push and foot-force numbers (5.4 %, 6.7–10.4 %, 5.9/6.7/6.1 BW) came from lost protocols with unknown normalization. Compare them in direction only, never in absolute terms.
- Contact forces are sampled at 50 Hz, the policy rate, so impact transients between physics substeps are missed. This is consistent across policies but biases all impact values low.

## What would flip this decision
- (b) The pilot shows that no magnitude in (0.5, 1.0) gives a non-saturated standing rate. Then switch to a per-env magnitude sweep with a v50 (50 %-fall velocity) estimate.
- (c) Repeat runs show p99.9 or median-per-run-max varying as much as the raw peak. Then rank on p99 only and treat impacts as a qualitative flag.
- (a) A time-since-onset histogram shows many stop or reversal falls landing just after the 2 s window. Then widen the window, for example to 3 s.
