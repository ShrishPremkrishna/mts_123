# What stumble definition and pass/fail criteria decide whether a policy "stumbles at the seam" (Part 3 seam test)?
Status: DECIDED by grader (pending consolidation) · 2026-11-18 · Requested by: W4
Criticality: Medium. It sets the Part 3 seam verdict that goes into the report.
Reversible?: yes, at zero GPU cost. Raw arrays are saved, so `part3_seam metrics` re-scores any run on CPU in seconds. The proposed n increase in (c) costs about 40 s of GPU per condition.

## Conclusion
**(a) Keep the stumble definition as it is:** body tilt **> 30°**, knee contact and torso contact all count, window **[-0.2 s, +1.0 s]**, and a window is invalid if the env reset less than 1.0 s before it.
**(b) Keep the limits:** `max_fall_rate` **0.02** per crossing and `max_stumble_excess` **0.05** over the no-seam control. Keep the judged blocks (falls on `seam_sweep` and `seam_dither`; excess on `seam_sweep_r05` and `seam_dither`), with two changes:
- Judge the dither excess **separately for up (trot→fast) and down crossings**. Pooling them halves a real up-crossing effect.
- Compute the excess on **incidents** (fall or stumble), so a fall can never hide a stumble.

`seam_sweep_step` stays reported but not judged. The **verdict is judged on `flat_dr` (headline) and `bumpy_dr`** (the spec's robustness pass); both must pass. DR-off conditions are scored with the same criteria and reported as a diagnostic ("seam-sensitive at nominal physics") that does not change the headline verdict.

## Evidence used
- Dry runs on the parent policy (`p3-parent-*_seam.md`, offline re-scoring in the scratchpad `grader-1-seam/`): all four conditions PASS under the provisional rules. Finite-sweep falls are 0.2–0.6 %. **The control has 0 tilt>30° windows in all 4 conditions** (319–320 windows each). Knee/torso events in the control: 0 on flat, 2/319 on bumpy_dr, 1/319 on bumpy_nodr.
- Tilt sensitivity, `flat_nodr` stumbles: dither 6.6 / 3.9 / 1.7 / 0.6 % at 25 / 30 / 35 / 40°; sweep 5.4 / 3.3 / 2.7 / 2.3 %. At 25° the verdict flips to FAIL, at 30° and above it passes. The knee/torso part does not depend on the threshold (dither 0.6 %).
- Settled tilt p99 is at most 20° in every regime. Flat trot and fast-gait p99.9 is 20–25°. Bumpy steady-fast p99.9 reaches 30–38°, but the 1.0 m/s fast-regime control windows still never exceed 30°. That makes 30° the lowest threshold with margin.
- The tilt events are tied to the seam, not noise. On `flat_nodr` dither up-crossings, 22/320 windows exceed 30° (onsets 0.22–0.32 s after the crossing). The same envs' own trot hold just before ([-1.4, -0.2] s) has 0/320. On `bumpy_nodr` it is 14/319 vs 1/319, and on `flat_dr` 0 vs 0. Down crossings have 0 events. `flat_nodr` dither stumbles: up 7.8 %, pooled 3.9 %.
- Window length: every dither event starts by +0.56 s (except one at +0.96 s on bumpy). Going from post 1.0 to 1.5 or 2.0 s adds at most 3 sweep stumbles (plus 4 late falls at +2.0 s on `bumpy_dr`), but it shrinks the control from 320 to 240/160 windows, because sham anchors near the seam are dropped, and it lets unrelated control falls in. Pre 0.5 s adds nothing and doubles the invalid windows (24 → 54).
- Step sweep: the falls are on down crossings (25–70 % of them), and that crossing is a 1.5 → 0 hard stop: the reference goes fast → lift/zero-blend, not fast → trot. It matches the Part 2 suite's "stop" falls (37 of 40 falls on `flat_dr`). The step-up knee contacts (5–13 %) come from a 0 → 1.5 start and are confounded. The suite's `seam_sweep_step` therefore does not measure the seam.
- Proposed rules applied to the parent:
  - `flat_dr`: PASS. Dither up/down incidents 0/0; r0.5 incident excess 0.6 %.
  - `bumpy_dr`: PASS. Dither up excess 0.3 %; r0.5 excess 2.5 %.
  - `flat_nodr`: diagnostic FAIL (dither up 7.8 %).
  - `bumpy_nodr`: borderline. The incident excess is 5.9 − 1.9 = 4.0 % (the control includes 5 falls in a single group); stumble-only, it is 5.3 %.

## Thought process
The criteria, in priority order:
1. The metric must measure the spec's question, "does it stumble at the seam".
2. It must attribute events to the seam, not to acceleration or terrain (REPRO_PLAN §3: always a control).
3. The false-positive rate must be about 0 on the no-seam control.
4. The limits must sit above the noise (§3: gaps under about 10 falls or about 10 % are noise).

30° scores best on criteria 1–3. It equals the trot task's old terminal angle and is far above the settled p99. It fires on the control 0 times out of about 1280 windows, yet it catches a real seam-locked pitch transient that 35–40° partly miss. 25° sits inside the trot p99.9 tail (24.7° on `flat_nodr`).

Knee and torso contact are binary, cannot depend on a threshold, and are about 0 at baseline, so both stay.

The limits fit criterion 4. 0.02 × 480 ≈ 10 falls, which is the project's noise floor. A 5 % excess at n ≈ 320 is about 16 events against a control that sits near 0.

Up and down crossings are different mechanisms. On the up crossing, the table and the phase clock both jump to the faster clock, with a reference jump of 1.4 against a typical step of 0.45. Pooling the two dilutes the only effect seen. Judging per crossing is a pre-registered split into two (n ≈ 320 each, 640 after (c)), not a post-hoc search.

`flat_dr` is the training distribution and the spec's viewer test. `bumpy_dr` is the robustness pass the spec names. DR-off is one physics instance with extra settings removed (encoder bias, COM, inertia), so it is reported in full but does not gate the verdict.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| Tilt 25° | Inside the trot p99.9 tail (24.7°); flips `flat_nodr` to FAIL on tilt alone |
| Tilt 35° or 40° | Misses part of a seam-locked transient (30–37° at about 0.25 s, 0/320 pre-crossing) that the 45° termination would also miss |
| Post 1.5 or 2.0 s, or pre 0.5 s | Adds no seam events; shrinks the control and adds invalid windows |
| Judge the step sweep | Its failures are a hard 1.5 → 0 stop and a 0 → 1.5 start, not the seam |
| Pooled dither only (current) | Halves the up-crossing effect (7.8 % → 3.9 %) |
| Judge all 4 conditions as gates | DR-off is not the training distribution or the spec's condition; report it instead of hiding it |
| Judge on the Wilson lower bound | Too lenient at n = 160–320; use larger n and repeats for close calls instead |

## Severe flagged issues
- **The seam is condition-sensitive.** The parent passes on DR-on but shows a seam-locked 30–37° pitch transient on 6–8 % of dither up-crossings with DR off (14 % in `dither back up` on `flat_nodr`). The report must state this, not just "PASS".
- **The dither has no step-matched control.** It is compared against a 0.5 m/s² ramp control. This is harmless while the control is about 0, but not guaranteed for the final policy.
- **n is small and GPU rollouts are not bit-reproducible.** `r05` has n = 160, so 1 event = 0.6 %. `bumpy_nodr` sits at 4.0–5.3 % against the 5 % limit, which is a close call. Results within ±0.02 of a limit need a repeat (second rollout seed) before a verdict.
- All evidence comes from the Part 1 parent, not the final policy. Freeze these criteria in `SEAM_CRITERIA` and the defaults **before** scoring the final policy.

## (c) Design changes before use on the final policy
1. Set `--envs-per-schedule 80` (960 envs, under the 1024 cap, about 40 s), or run 2 rollout seeds.
2. Add step-matched dither controls (0.2 ↔ 0.4 trot only, and 0.8 ↔ 1.0 fast only) and the free within-env pre-crossing window.
3. Implement per-crossing dither and incident excess in `verdict()`.
4. Print 25° and 35° sensitivity lines next to the verdict.

## What would flip this decision
- On the final policy, the control's tilt>30° rate rises above about 1 %: re-derive the threshold from the control's p99.9.
- Seam events appear after +1.0 s: lengthen the post window.
- A repeat run moves a judged block by more than 0.02: raise n further, or judge on the mean of the repeats.
- The human considers nominal physics (DR-off) the closer proxy for hardware: make `flat_nodr` a gating condition (the parent would then FAIL).
