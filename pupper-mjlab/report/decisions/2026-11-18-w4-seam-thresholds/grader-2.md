# Part 3 seam test: what counts as a stumble, and what pass/fail criteria decide "stumbles at the seam"?
Status: grader-2 proposal (pending agreement check / human review) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: Medium. It shapes the Part 3 seam verdict in the report. No other workstream depends on it.
Reversible?: yes. (a) and (b) can be recomputed offline from the saved raw arrays at 0 GPU cost. The (c) changes need one seam rerun per condition, about 20–50 s of GPU each.

## Conclusion
**(a) Keep the definition. Change how it is scored.** A stumble is any non-fall window in [−0.2 s, +1.0 s] around the crossing that has a knee *or* torso ground contact, or a body tilt **> 30°**. Windows less than 1.0 s after a reset are invalid. Score **incidents = fall OR stumble**, because the current stumble count excludes falls.
**(b) Judge 8 cells, not pooled blocks.** The cells are direction (fwd/back) × crossing (up/down), for the finite-rate sweeps (rates 0.25/0.5/1.0 pooled, n≈120) and for the dither (n=160). A run **FAILs** if either of these holds:
1. A pooled block's fall rate (`seam_sweep` or `seam_dither`) is above **0.02** (unchanged).
2. Any cell's incident rate exceeds the pooled `control_all` incident rate by more than **0.05**, *and* a one-sided Fisher test against the control gives **p < 0.01**.

`seam_sweep_step` and `seam_sweep_r05` are reported but not judged. The **headline verdict is on flat_dr**. **bumpy_dr gets its own verdict** with the same criteria (the spec's "robustness pass"). The DR-off conditions are scored and reported as diagnostics and are not part of either verdict.

## Evidence used
- Offline recomputation (`scratchpad/graders/grader-2-seam/{grid.json,cells.txt}`) on the parent's raw arrays.
- **The pooled verdict hides a real seam failure.** Under flat_nodr at 30°, the backward trot→fast dither cell has **22/160 stumbles vs 0/320 control (Fisher p≈1e-11)**. Pooled over the four dither cells, that becomes an excess of 0.039 < 0.05, so the run reads PASS. The same happens in bumpy_nodr: 16/160 vs 6/319 (p≈1e-4).
- **Max tilt in the no-seam control windows.** Over the non-fall windows of all 4 conditions (1,273 windows), the maximum is ≤ 23.4° (flat_dr 17.5, flat_nodr 22.0, bumpy_dr 23.4, bumpy_nodr 20.8; p99 16–20.5°). Seam windows reach 27–42°. For settled steady gait, tilt p99 is 13.5–20.4°. Termination is at 45°.
- **Effect of the tilt threshold.** At 25°, flat_dr's worst cell (sweep fwd down) is 6/120, an excess of exactly 0.050 (knife-edge). At 30° it is 1/120. At 20°, false positives in the control appear on bumpy (6/319).
- **Effect of the window length.** post 0.6/1.0/1.5 s changes the seam counts by ≤ 3 per block. At 1.5 s, the control drops from 320 to 240 windows (the sham-anchor drop rule) and the window overlaps the next dither step (steps are 1.5 s apart).
- **Contacts.** Knee contacts occur in 0.0005 of steady steps, and contacts occur in 0/320 flat control windows and 2/319 bumpy ones. They are specific signals.
- **Step sweep.** Falls are 13–70 % per cell, almost all on the *down* (1.5→0 hard stop) crossing. The Part 2 suite shows the same stop mode (flat_dr: 37 of 40 falls are stops). This is not a seam effect.
- **New-criteria verdicts on the parent** (30°): flat_dr **PASS** (max cell excess 0.008), bumpy_dr **PASS** (0.028), flat_nodr **FAIL** (dither back-up 0.138; sweep fwd up/down 0.058/0.067), bumpy_nodr **FAIL** (dither back-up 0.081).

## Thought process
**Priorities.**
1. The metric must measure the failure the spec names, a stumble *at the seam*. That means it is control-referenced, and a failure confined to one direction or crossing must not be averaged away.
2. Its false-positive rate must be low against the noise (REPRO_PLAN §3: small gaps are ties; GPU reruns are not bit-exact).
3. It must be pre-registered from the no-seam distribution, not tuned to the verdict.

**Tilt threshold.** 30° sits about 7° above every control window's maximum tilt, and holds up if the final policy pitches somewhat more. 25° is more sensitive, but the margin is only 1.6°, and it flips the flat_dr parent to knife-edge on tilt-only lurches. Per-cell judging recovers the sensitivity that matters: the nodr backward-seam failure is caught at 30°.

**Pooling.** Pooling across up/down and fwd/back is the main defect. Benign down crossings dilute a concentrated failure by about 4×. Pooling the finite rates inside a direction×crossing cell is fine (the r1 cells are not worse than r0.25), and it triples n to about 120. The Fisher gate keeps 8 cells × noise from producing false FAILs: at a true rate of 0.02, P(≥9/120) is about 0.1 % per cell.

**Conditions.** The spec asks "does it stumble at the seam?" (viewer test) and then separately asks for "Bumpy … with your lab 4 DR ranges as the robustness pass". Hence two verdicts. DR-off is a mechanics corner outside the training distribution (suite falls 100 vs 40). It is a sim-to-real warning, so it is reported but does not gate.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| Keep the pooled blocks (status quo) | PASSes a 13.8 % vs 0 % backward-seam stumble (flat_nodr) |
| Tilt 25° | The margin over control is only 1.6°; flat_dr becomes knife-edge on tilt-only events |
| Tilt 35–40° | Loses most tilt events (flat_nodr dither 25→4); too close to 45° termination |
| Control-relative threshold (control p99 + k) | Adaptive but noisy at n=320 and harder to pre-register; kept as a guard instead (see flip) |
| Per (rate, dir, cross) cells, n=40 | 2 events = 0.05; too noisy |
| Gate on all 4 conditions | DR-off is not the training or deployment distribution; reported instead |
| Post window 1.5 s | Overlaps the next dither step; loses 80 control windows; no extra seam signal |

## Severe flagged issues
- **The pooled verdict as implemented is not fit for purpose.** `verdict()` must be changed to the per-cell incident rule before the final policy is scored. Until then, the parent's four "PASS" lines are misleading for nodr.
- `stumble` excludes falls, so stumble excess can improve when stumbles become falls. Use fall|stumble.
- **The dither has no step-matched control.** The control uses 0.5 m/s² ramps, while the dither uses 0.2 m/s steps. (c) Add trot-only (0.2↔0.4) and fast-only (0.9↔1.1) step dithers to `control_<dir>`.
- **Noise.** GPU rollouts are not bit-reproducible, and n=120–160 per cell. (c) For the final policy, run `--envs-per-schedule 80` (960 envs) or two `--seed` repeats per condition, at under 2 GPU-min in total.
- The thresholds were set on the Part 1 parent, not the final policy. Parent nodr already fails, which is a sim-to-real risk at the backward seam. Report it.

## What would flip this decision
- In the final policy, the control windows' p99 max tilt exceeds about 25°: the 30° threshold has lost its margin. Re-derive it, for example as control max + 5°.
- A step-matched dither control shows comparable incident rates: then the dither failures are step effects, not seam effects. Judge on the sweeps only.
- Repeat runs disagree on a cell's FAIL: require the FAIL to hold in pooled repeats.
