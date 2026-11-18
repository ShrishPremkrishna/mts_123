# Which Part 1 arm is the provisional Part 2 parent, and which two arms get seed 43?
Status: DECIDED by grader (pending consolidation) · 2026-11-18 · Requested by: W1
Criticality: High. The parent's ±1.5 m/s gait becomes every Part 2 reference, so this choice gates all of Part 2.
Reversible?: yes. Reversing it means redoing Part 2 on another parent: about 3 GPU-hours (14 legs × 500 iterations × 2 seeds, REPRO_PLAN §4.3) plus evals and captures.

## Conclusion
**Parent: `gallop_c` (s42, `1c5r0n8c`), provisionally.** **Top 2 for seed 43: `gallop_c` and `fast_trot`.** I chose gallop_c on its s42 evidence alone. Its existing s43 run (`y38xkkvh`) already fills its second-seed slot, so the only new job is **`fast_trot` s43**, about 40 min solo. gallop_a, gallop_b and baseline get no second seed.

## Evidence used
- Eligibility, from `stock_table.json` and notebook §4a/4c. Part 2 captures at ±1.5 m/s, so the parent has to run both ways. Achieved vx at the +1.5 / −1.5 commands: baseline +0.21/−0.19, gallop_a +1.26/**+0.29**, gallop_b +1.32/**+0.41**. a and b move *forward* when commanded backward, with err_xy at −1.5 of 1.79 and 1.91. fast_trot reaches +1.36/−1.46 (err 0.171/0.175) and gallop_c reaches +1.55/−1.48 (err 0.134/0.132).
- Ranked W2 readouts (`w2/compare_all.md`, s42), fast_trot vs gallop_c: falls **53 vs 44** (stop 44 vs 41, reversal 8 vs 2). F_runmax_med **3.17 vs 3.03 BW**. push_stand@0.65 **0.681 vs 0.453**. push_move@0.5 **0.006 vs 0.002**.
- Informational W2 readouts: sat@1.5 **0.126 vs 0.179**, tau2 1.02 vs 1.35, F_peak (flag only) 14.5 vs 6.91. Drift ±: 45/65.9 vs 67.6/65.7 deg/10 s. Splay ±: −0.134/+0.001 vs +0.035/+0.115 rad. Slip 0.353 vs 0.382.
- Stock 16-segment sequence (`stock/<arm>/seq_metrics.json`), failures: fast_trot 2, gallop_c 6 (5 of them in segment 11, a yaw turn right after −1.5), gallop_b 1, gallop_a 27, baseline 3.
- gallop_c s43 (evidence only): falls 41, F_runmax 3.09, sat@1.5 0.176, err ±1.5 0.123/0.129, training err_xy 0.220 vs 0.217 (notebook §3). push_stand@0.65 drops to **0.009** from 0.453, which shows this metric swings by 0.44 between seeds of the same arm.
- Footfall (notebook §4c): fast_trot trots at a regular 2.6 Hz at +1.5 (3.3 Hz at −1.5). gallop_c runs at 4.2–4.5 Hz against a 3.0 Hz reference clock, and its ±1.5 cycles are marked irregular (`*`, lag concentration < 0.5).
- `REPRO_PLAN.md` §2.2: the old team picked gallop_c on sim-to-real grounds. The recovered Part 2 targets (0.159 → 0.124) were all measured on a gallop_c parent.

## Thought process
The criteria, in priority order:
1. **Hard gate.** The parent must actually run at both +1.5 and −1.5 m/s, or the capture has nothing to fold. That leaves fast_trot and gallop_c.
2. **Ranked sim-to-real readouts** (REPRO_PLAN §3, W2 metrics decision). Falls differ by 9, which is under the 10–15 noise band, so they tie. Foot force is within 5 %, also a tie. Standing push favours c at s42, but s43 shows that metric is seed noise. On ranked metrics the two arms tie, with every point estimate in c's favour.
3. **Unranked risks.** fast_trot clearly wins on 3 Nm saturation (12.6 % vs 17.9 %, a gap above the 10 % noise threshold, which matches c's kp·Δq of 3.82 Nm > 3 Nm). c wins on tracking (0.133 vs 0.173), hind splay (outward rather than inward) and peak impact (flag only).
4. **Tie-breaks.** c has the lineage the old team used, so Part 2 results stay comparable with the recovered numbers. It also has the design property the old team valued, at least two feet down at all times.

Reward played no part (REPRO_PLAN §3). Since the ranked metrics tie and the main risk (saturation) falls on c, the pick is close, which is why it is provisional. That same closeness is why the second seed should test fast_trot, the real alternative. Spending it on b would only probe whether b can learn to run backward, a science question rather than a parent decision.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| fast_trot as parent | It ties on ranked metrics and has 5 pp less saturation and a cleaner ±1.5 cycle. But it has 9 more falls (noise-level), worse standing push at s42, worse tracking, hind feet pulled inward, and a lineage that breaks comparability with the recovered Part 2. It is the strongest runner-up, so it gets the s43 run. |
| gallop_b (as parent, or in the top 2) | It cannot run backward (vx +0.41 at −1.5), so a −1.5 capture would fold a forward shuffle. Its training err_xy of 0.584 also failed to reproduce the old b (0.259). |
| gallop_a | It cannot run backward, has the most falls (69, including 14 steady), 27 failures in the sequence, a 13° nose-down pitch open loop, and push_stand 0.85. |
| baseline | It tops out at about 0.2 m/s at ±1.5 (err 1.30), so there is no fast gait to capture. It stays the comparator. |
| Re-run gallop_c s43 | Not needed. `y38xkkvh` uses the same recipe and the same reference @ `e01364f`. |

## Severe flagged issues
- **Capture and fold quality for c.** c strides at about 4.2 Hz against a 3.0 Hz reference clock, and its ±1.5 cycle is irregular. A phase-indexed fold may blur the captured reference. This is the most likely way the pick could hurt Part 2.
- **3 Nm saturation.** c saturates 17.9 % of the time at ±1.5 (18 % at s43), against 12.6 % for fast_trot. This is the largest sim-to-real risk for hardware.
- **One seed for every non-c arm.** GPU rollouts are not bit-reproducible, so fall gaps under 10–15 are ties. push_stand@0.65 swings between 0.45 and 0.01 across c's two seeds, so it must not decide anything alone.
- gallop_c s43 was launched at `a042c72`, not `2e7f5b9`. The reference is identical (blob hash recorded). Confirm the training code did not change between the two commits before pooling the seeds.

## What would flip this decision
- fast_trot s43 lands at least 10–15 falls below c's mean (42.5) with foot force and standing push no worse: switch the parent to fast_trot.
- The first gallop_c capture fails its periodicity or fold-residual check, or round 1 collapses even with the abduction pin, while a fast_trot capture folds cleanly: switch.
- Saturation is promoted to a ranked metric, or Part 3 DR / torque-limit testing shows c degrading where fast_trot does not.
- The diff between `a042c72` and `2e7f5b9` touches training code, so c s43 is not a valid second seed: run c s43 again before relying on it.
