# Which Part 1 arm is the provisional Part 2 parent, and which two arms get seed 43 as the "top 2"?
Status: DECIDED by grader (pending consolidation) · 2026-11-18 · Requested by: W1
Criticality: High. The parent's ±1.5 m/s gait becomes every Part 2 reference, so a wrong pick invalidates all of Part 2.
Reversible?: yes. Reversing means re-capturing and re-folding from another parent and redoing the Part 2 legs (about 3 GPU-hours plus evals and notebook rework).

## Conclusion
**Parent: `gallop_c` (s42, WandB `1c5r0n8c`, `model_2999.pt`).** **Top 2: `gallop_c` and `fast_trot`.** gallop_c already has its seed 43 (`y38xkkvh`), and that run agrees with s42, so the only new GPU job is **`fast_trot` s43**. No other arm gets a second seed.

## Evidence used
- **Gate: can it run ±1.5 m/s both ways?** (W2 `compare_all.md`, vx+1.5 / vx−1.5): baseline 0.207 / −0.197; gallop_a 1.26 / **+0.254**; gallop_b 1.33 / **+0.384**; fast_trot 1.35 / −1.48; gallop_c 1.54 / −1.50 (s43: 1.53 / −1.48). The stock sweep (`stock_table.json`) agrees: gallop_a and gallop_b move *forward* when commanded −0.75 to −1.5 m/s.
- **Ranked W2 metrics, fast_trot vs gallop_c s42 / s43** (`compare_all.md`): falls 53 vs 44 / 41; F_runmax_med 3.17 vs 3.03 / 3.09 BW; push_stand@0.65 0.681 vs 0.453 / 0.009; push_move@0.5 0.006 vs 0.002 / 0.
- **Unranked readouts** (`compare_all.md`): sat@1.5 0.126 vs 0.179 / 0.176; tau2 1.02 vs 1.35 / 1.37; err ±1.5 0.174/0.176 vs 0.130/0.146 (s43 0.123/0.129); drift± 45/66 vs 68/66 (s43 54/51) deg/10 s; hind splay+ −0.134 vs +0.035 / +0.047 rad; F_peak flag 14.5 vs 6.9 / 9.0 BW.
- **Stock sweep at ±1.5** (`stock_table.json`): 3 Nm saturation 10.2/14.9 % (fast_trot) vs 14.9/21.0 % (c). Lag concentration 0.61–0.97 (fast_trot) vs **0.18–0.61** (c). Share of time with 0 feet down: 5.7/6.0 % vs 4.7/1.2 %. The 16-segment sequence (`stock/*/seq_metrics.json`) gives 2 failures for fast_trot and 6 for c (5 of them in the yaw ±1 rad/s segment); gallop_a has 27.
- **Seed agreement for c** (`part1_notebook.md` §3): training err_xy 0.217 vs 0.220 and reward 173.4 vs 173.8. Design files (`part1/gallop_c.json`): reference kp·Δq is 3.82 Nm (> 3 Nm proxy), peak q̇ 45 rad/s, at least 2 feet down by design.

## Thought process
The capture happens at ±1.5 m/s, so a parent must hold a real gait in both directions. That is a hard gate, and it removes baseline (about 0.2 m/s) and gallop_a/gallop_b (they run forward on a backward command; a capture would fold a forward shuffle into the backward table). Two arms remain. I ranked them on the REPRO_PLAN §3 sim-to-real readouts, not reward, using the W2 ranked columns and treating one-seed gaps under about 10 falls or 10 % as noise:
- **Falls:** 53 vs 44/41 is a 9–12 gap, at the noise line. It is a tie that leans toward c, because both c seeds sit below fast_trot.
- **Foot impact (median of run maxima):** a tie (4 % gap).
- **Standing push:** c is better on both seeds (0.453 and 0.009 vs 0.681). This is the only ranked metric with a gap well above noise. It is seed-sensitive, but even c's worse seed beats fast_trot by 23 points.
- **Moving push:** a tie at the floor.
- **Against c:** 3 Nm saturation is about 5 points higher (≈ 40 % relative), and it reproduces on both seeds. τ² is about 30 % higher.
- **For c:** tracking is about 25 % better on both seeds. Hind splay is outward rather than inward, so the old C's backward-splay weakness is gone. Heading drift is similar.

Taken together, c wins or ties every ranked metric and pays for it in torque. It is also the only arm with two-seed evidence, which matters for a parent that gates 3 GPU-hours. It also keeps continuity with the recovered old Part 2 lineage (`C_lineage_metrics.md` §3d, which descends from the old gallop_c), so the old Part 2 findings can be checked against a matched new parent. fast_trot is the clear runner-up. It is the only other arm that passes the gate, so it gets the second seed.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| fast_trot as parent | Ranked metrics tie or lose to c (standing push 0.681 vs ≤ 0.453; falls 53 vs 41–44). Only one seed. Its advantages (lower saturation, a regular cycle at ±1.5) are real and are what would flip the decision |
| gallop_b (parent or top 2) | Fails the gate: vx −1.5 = +0.38 m/s. Training err_xy 0.584 does not reproduce the old b. A second seed would test seed luck, not a parent candidate |
| gallop_a | Fails the gate (vx −1.5 = +0.25). Most falls (69, including 14 steady). Worst standing push (0.85). 27 sequence failures. Tips 13° nose-down open loop |
| baseline | Achieves only about 0.2 m/s at ±1.5 (err 1.30); nothing to capture. It stays the comparator |
| Count gallop_c's s43 as "done" and pick a third arm for a new s43 | No third arm passes the gate. Spending a seed on a or b buys nothing for Part 2 |

## Severe flagged issues
- **Torque: c saturates 3 Nm 17.6–17.9 % of the time at ±1.5 (W2), with 21 % at −1.5 in the stock sweep, and its reference already exceeds the kp·Δq proxy (3.82 Nm).** The policy runs at 4.2–4.5 Hz against a 3.0 Hz reference, so a captured reference will likely push the proxy higher still. Part 2 must check the captured table against the 3 Nm proxy and report saturation for every round. This is the main sim-to-real risk of the pick.
- **Irregular cycle at ±1.5:** c's lag concentration falls to 0.18 (FL, −1.5) and 0.43–0.44 (+1.5). Its "≥ 2 feet always down" property holds only in the design; at +1.5 the trained gait has 4.7 % zero-feet and 26 % one-foot time. Phase-folding an irregular cycle can smear it into a reference the policy has never played. Run the fold periodicity check before any warm-start.
- **Asymmetric evidence:** c has 2 seeds and fast_trot has 1, and push_stand swings 0.453 → 0.009 between c's seeds. The standing-push lead is direction-only until fast_trot s43 lands.
- GPU rollouts are not bit-reproducible (W2 decision: 62–78 falls on identical runs), so the fall gap is a tie. All five arms use `splay.json`, so the old baseline and fast_trot numbers are not weight-matched comparators.

## What would flip this decision
- fast_trot s43 lands, and its two-seed mean beats c's (s42 + s43) by more than about 10 falls **and** matches or beats c on push_stand@0.65: switch the parent to fast_trot.
- The gallop_c capture at ±1.5 fails periodicity or joint-limit sanitization, or its kp·Δq proxy rises well above about 4 Nm after folding, while fast_trot's fold is clean: switch to fast_trot.
- A re-run of the W2 push test shows c's standing-push lead within the per-cell repeat spread: re-decide on saturation, which favours fast_trot.
