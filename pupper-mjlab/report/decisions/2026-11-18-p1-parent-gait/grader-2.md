# Which Part 1 arm is the provisional Part 2 parent, and which two arms get seed 43?
Status: DECIDED by grader (pending consolidation) · 2026-11-18 · Requested by: W1
Criticality: High. Every Part 2 round captures, folds and warm-starts from this parent, so a wrong pick invalidates all of Part 2.
Reversible?: yes. It costs the Part 2 rounds built on the wrong parent (> 3 GPU-hours at 2 seeds) plus the re-capture and re-fold. Before round 1 starts it costs nothing.

## Conclusion
**Provisional parent: `gallop_c` s42** (`1c5r0n8c`). **Top 2 for seed 43: `gallop_c` and `fast_trot`.** gallop_c s43 (`y38xkkvh`) already exists, so the only new run needed is **fast_trot s43**. The pick is provisional: no ranked metric separates c from fast_trot beyond the noise rule. c leads on every one of them, and its lead holds across two seeds, but its torque saturation is clearly worse.

## Evidence used
- `eval/w2/compare_all.md` (ranked columns). Each row lists fast_trot s42 / c s42 / c s43:
  - **falls** 53 / 44 / 41;
  - **reversal falls** 8 / 2 / 6, **stop falls** 44 / 41 / 35;
  - **F_runmax_med_BW** 3.17 / 3.03 / 3.09;
  - **push_stand@0.65** 0.681 / 0.453 / 0.009;
  - **push_move@0.5** 0.006 / 0.002 / 0;
  - other arms: gallop_a 69 falls (14 steady) and push_stand 0.85; gallop_b 43 falls; baseline 1 fall but err±1.5 = 1.30.
- Same table, unranked columns, in the same order:
  - **err±1.5** 0.174/0.176, 0.130/0.146, 0.123/0.129;
  - **sat@1.5** 0.126 / 0.179 / 0.176;
  - **tau2** 1.02 / 1.35 / 1.37;
  - **drift±** 45/66, 68/66, 54/51 deg/10 s;
  - **splay±** −0.134/+0.001, +0.035/+0.115, +0.047/+0.106;
  - **F_peak (flag)** 14.5 / 6.9 / 9.0 BW.
- `eval/stock_table.json` (stock sweep, s42):
  - At ±1.5, fast_trot vs c: err 0.173 vs 0.133; sat 12.6 % vs 17.9 % (c reaches 21.0 % at −1.5); foot-force max 16.3 vs 6.4 BW.
  - At matched speed c still saturates about 2× as often as fast_trot: at ±0.75, 0.079/0.098 vs 0.028/0.035; at ±1.0, 0.100/0.131 vs 0.049/0.070.
  - Gait regularity at ±1.5: c is labelled "irregular" (lag concentration FL 0.43 at +1.5, 0.18 at −1.5) and steps at 4.2/4.5 Hz against a 3.0 Hz reference clock. fast_trot trots regularly at 2.6 Hz, locked to its 2.67 Hz clock, at +1.5, but steps at 3.3 Hz at −1.5 (concentration 0.61).
  - gallop_a and gallop_b move *forward* at +0.29/+0.41 m/s when commanded −1.5 (err 1.80/1.91).
- `stock/<arm>/seq_metrics.json` (16-segment sequence): failures are baseline 3, fast_trot 2, gallop_a 27, gallop_b 1, gallop_c 6 (5 of them in a single zero-command segment after −1.5).
- `part1_notebook.md` §2–3:
  - Design: c's kp·Δq is 3.82 Nm (over the 3 Nm proxy), with peak q̇ 45 rad/s; fast_trot is 2.89 Nm, 31 rad/s.
  - c has ≥ 2 feet down at all times and is the only design with < 10° open-loop tilt both ways.
  - Training: c s42 vs s43 err_xy is 0.217 vs 0.220, within 1.5 % on every metric.
- `REPRO_PLAN.md` §2.2: the old team picked c (sim-to-real readouts, runs backward, ≥ 2 feet down). The recovered Part 2 targets in `recovery/C_lineage_metrics.md` all descend from gallop_c.

## Thought process
**Criteria**, in this order:
1. **Hard gate:** the arm must actually run at ±1.5 in both directions, because Part 2 captures at ±1.5.
2. **Ranked W2 sim-to-real readouts** (REPRO_PLAN §3: falls, foot impact, push), with gaps under ~10–15 falls or ~10 % treated as noise.
3. **The other §3 readouts:** 3 Nm saturation, drift and splay.
4. **Tracking.**
5. **Comparability with the recovered Part 2 lineage** (tie-breaker only).

**Gate.** It eliminates baseline (0.2 m/s), gallop_a and gallop_b (they cannot run backward). That leaves fast_trot vs gallop_c, which are therefore the top 2 regardless of the parent choice.

**Ranked readouts.** c is ahead of fast_trot on every one:
- Falls are 9–12 lower, at the edge of the noise band.
- Reversal falls are 2/6 vs 8.
- F_runmax is 3–4 % lower (a tie).
- Standing push is lower at both seeds, but that metric is extremely seed-sensitive (0.45 vs 0.01).

No single gap is decisive. What matters is that c leads on all of them and both of its seeds agree, while fast_trot has only one seed.

**Secondary readouts and tracking.**
- Saturation favours fast_trot by a real margin (≈ 40 % relative at ±1.5 and ≈ 2× at matched speed). This is the strongest argument against c.
- Splay favours c, which holds the hind feet outward. Drift is a tie.
- Tracking favours c (≈ 25 % lower err±1.5, and it reaches the full ±1.5).
- The single-sample impact flag (6–9 vs 14.5–16.3 BW) is consistent across both harnesses. It is not ranked, but it points the same way.

**Weighing it.** I weigh saturation below falls, impact and push because training already lives with it: c's falls are not higher and its peak forces are lower. The tie-breaker (the recovered lineage and its Part 2 numbers are all gallop_c) also favours c. Hence c is the parent and is provisional, with fast_trot as the named fallback.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| fast_trot as parent | It saturates less and has a regular, clock-locked trot at +1.5, but it is behind c on every ranked readout and on tracking, it is one seed only, and it breaks comparability with the recovered c lineage. It is the fallback (see "What would flip"). |
| gallop_b (in the top 2) | It runs forward when commanded −1.25 to −1.5 (err −1.5 = 1.89); capture at −1.5 is impossible. Its 43 falls match c's only because it never reverses at speed. |
| gallop_a | Same backward failure, plus the most falls (69, 14 steady), push_stand 0.85, 27 sequence failures and a 13° nose-down pitch open loop. |
| baseline | It does not run: 0.2 m/s at ±1.5 (err 1.30). Its low falls and saturation come from not moving. |
| Skip fast_trot s43 because c s43 already exists | Then the "top 2" would compare a 2-seed arm with a 1-seed arm, and the only close contest would go unresolved. |

## Severe flagged issues
- **3 Nm saturation.** c saturates 17.9 % of samples at ±1.5 (21 % at −1.5), and its reference already exceeds the kp·Δq proxy (3.82 Nm). Capturing at ±1.5 will fold a saturating gait into the next reference. W3 must track sat@1.5 per round and must not accept a round that raises it.
- **Capture and fold risk.** At ±1.5, c steps at 4.2–4.5 Hz against a 3.0 Hz clock, and its cycle is irregular (lag concentration down to 0.18). The default fold (`decisions/2026-11-18-w2-fold-default.md`) folds by clock phase, so the captured reference may come out smeared. Check the fold diagnostics (lag-fit residual, dispersion, `--lag-steps auto`) on the c capture **before** launching round 1.
- **Seed asymmetry and noise.** The comparison is fast_trot (1 seed) vs c (2 seeds). The fall gap (9–12) is within the 10–15 noise band, and push_stand moves by 0.44 between c's seeds. GPU rollouts are not bit-reproducible.
- **Drift.** All fast arms drift 45–74 deg/10 s with heading hold off. This does not separate the arms, but it is a sim-to-real risk.

## What would flip this decision
- fast_trot s43 lands clearly ahead of c on the ranked metrics: falls ≥ 15 below both c seeds, or push_stand at or below c's two-seed mean (~0.23) together with fewer falls. Then make fast_trot the parent.
- The c capture at ±1.5 folds badly (dispersion or lag-fit residual well above the cont4500 smoke values in `fold_variants.md`, or the folded table is rejected by the limit check), while fast_trot's capture folds cleanly. Then switch to fast_trot.
- Round 1 on c raises sat@1.5 or falls beyond the control by more than the noise band.
