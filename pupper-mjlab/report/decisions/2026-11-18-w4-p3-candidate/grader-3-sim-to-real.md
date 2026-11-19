# Part 3 candidate, hardware eligibility, and a push condition (grader 3, sim-to-real / safety lens)
Status: DECIDED by grader 3 of 4 (lens: sim-to-real / safety skeptic; pending synthesis and human review) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: Critical. It picks the policy behind the headline Part 3 claims and decides what may be prepared for a physical robot.
Reversible?: (a) yes, re-selecting among saved checkpoints costs no GPU. A HOLD costs nothing to reverse. Clearing a policy that then falls on the robot cannot be undone.

## Conclusion
**(a)** The simulation candidate is **Bumpy +2000** (`rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt`), the pre-registered pick. Report its standing-push regression as a result next to it.
**(b)** Under the pre-registered HOLD rule it is **CLEAR**. I re-derived every number.
**(c)** **Yes.** Add a push condition: HOLD if `R:push_stand@0.65` on `flat_dr`, averaged over eval seeds 1234 and 4321, is **more than P's mean + 0.10** (P = 0.109, so the limit is 0.21). +2000 measures **0.489 / 0.497**, so it is **HOLD for hardware**. Under the WIDEN branch a HOLD means no more training and a re-decision with the human. For that re-decision I would put forward the matched **flat control +2000** for hardware preparation, but only after it passes a seed-4321 confirm.

## Evidence used
- **HOLD inputs** (`p3-final-kpwide_deploy_hold.md`, `ablate/`). nompd for +2000 is 7 / 12 (mean 9.5) and flat_dr is 10 / 11 (mean 10.5).
  - (a) gap −1.0 vs a limit of 15.
  - (b) 9.5 − 38 = −28.5 vs a limit of 10.
  - WIDEN check: control flat_dr 12 vs P's 14, a difference of −2.
- **Standing push, falls per push** (`*_push.md`, `diag/push/`, about 340 pushes per cell; W2's band is 0.040 and the ranked-cell repeat spread is ≤ 0.024).

  | Policy | Standing push @0.65 m/s (flat_dr) | Standing push @0.5 m/s |
  |---|---|---|
  | P | 0.096 / 0.122 | 0 / 0 |
  | Bumpy +2000 | 0.489 / 0.497 (Bumpy terrain 0.486) | 0.151 / 0.158 |
  | Bumpy neighbours 6400 / 6450 | 0.563 / 0.721 | – |
  | Bumpy, every other measured checkpoint (+500 to +1500) | 0.53–0.86 | – |
  | Flat control +2000 | 0.0915 (Bumpy terrain 0.163) | 0.003 |

  - Moving push @0.5 m/s is 0 to 0.023 everywhere, so the loss is specific to standing.
  - For reference, the Part 1 parent reads 0.455. The Bumpy arm has erased the one Part 2 gain that replicated across seeds (W3 final decision: push_stand −0.20 / −0.35).
- **Other hardware readouts, +2000 vs P** (seed 1234, flat_dr unless noted). +2000 is better or tied on everything except push.

  | Readout | P | Bumpy +2000 | Verdict |
  |---|---|---|---|
  | Stop/reversal falls | 9 / 5 | 4 / 6 | better or tie |
  | bumpy_dr falls | 35 | 18 / 23 | better |
  | sat@1.5 | 0.161 | 0.165 | tie |
  | F_runmax median | 3.03 BW | 2.60 BW | better |
  | F_runmax median under nompd | 3.40 BW | 2.81 BW | better |
  | Drift (deg/10 s) | 33 / 49 | 31 / 41 | tie |
  | Seam | PASS, 0 seam falls | PASS, 0 seam falls | tie |

  The F_peak flag under nompd is 10.4–11.3 BW for +2000 vs 7.2–8.3 for P. That statistic is single-sample and noisy, so it is a flag only.
- **Flat control +2000** (one seed): flat_dr 12, bumpy_dr 29, nompd 20 (gap +8), sat@1.5 0.143, F_runmax 2.85.
  - Its nompd swings from leg to leg: 28, 34, 51, 20.
  - Its bumpy_dr F_peak flag is 62 BW (a single sample at steady +1.5 m/s on bumps).

## Thought process
I rank by REPRO_PLAN §3 sim-to-real readouts and ignore reward and tracking.

**What makes a policy unsafe to hand to an operator.** It should not fall when a person standing next to it gives it a moderate nudge. Standing is when people are closest: at startup, between trials, and while the robot is being handled. The checklist confirms the robot has no self-righting skill, so every fall means a person has to intervene and the robot takes the impact.

**How bad +2000 is on this.** It falls on about 1 in 2 lateral 0.65 m/s shoves and about 1 in 6 at 0.5 m/s, where P and the control almost never fall.

**Why the gap is real.** Ten Bumpy-arm measurements, across checkpoints, eval seeds and terrain, all fall between 0.49 and 0.86, against 0.09–0.16 for P and the control. The gap is more than 10 SE.

**What causes it.** Terrain, not the PD range: the control trains with the same widened gains and keeps its push recovery.

**Why add a rule after the data.** The rule is post hoc, and I say so. It is still justified for three reasons.
1. The metric was already a pre-registered ranked metric (W2). It was not invented after seeing the data.
2. The rule can only make the outcome more conservative.
3. The verdict does not depend on the exact threshold. Any limit between 0.17 and 0.48 gives the same answer for P, the control and +2000.

**Why the limit is set relative to P.** The form mirrors HOLD(b): a policy may not end up worse than its starting point. The margin of 0.10 is 2.5 times W2's band.

**Why +2000 still stays the simulation candidate.** It is pre-registered, and it wins on the readouts the spec's robustness pass targets: Bumpy terrain, nominal PD and transient falls. The push regression is a result to report, not a reason to re-select.

**Why not +500.** Choosing it would be post hoc, and it is also worse on push (0.80).

**Hardware status of the other options.** P fails HOLD(a) (gap +22.5). The control is the only policy that is clean on every hardware readout, but it was not a pre-registered option, it is not the Bumpy policy, and it has only one nompd seed.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| +2000 hardware-eligible as-is (pre-registered rule only) | Standing push fails at 0.49 / 0.50, 5× P. That is unsafe next to a human operator. |
| P for hardware | It fails HOLD(a), with a nompd gap of +22.5 and 38 falls at the commanded gain. |
| +500 / +1000 / +1500 | Post hoc, and every one of them is worse on push (0.53–0.80). +1000 also regressed on falls. |
| Flat control +2000 as the Part 3 candidate | It is not the spec's Bumpy pass, and on bumpy_dr it is worse than +2000 (29 vs 18 / 23) and nompd (20 vs 9.5). It is offered only as the hardware option for the human re-decision. |
| Push condition on the moving cell or the 0.8 m/s rung | Moving push is 0 for everyone, so it carries no signal. 0.8 is near ceiling (0.54–1.0). |

## Severe flagged issues
- **The pre-registration gap is real.** The selection guards and HOLD ignore W2's ranked push metric, and the policy they clear loses the Part 2 headline gain. Every report claim about +2000 must state the push regression.
- **Push has never been measured at nominal PD,** for any policy. All push evidence comes from DR conditions.
- **The control +2000's hardware case rests on one nompd seed,** and its nompd swings by up to 31 between legs. Its 62 BW peak flag is unexplained.
- `kp-coverage` says to "record the outcome in hardware checklist item 5", but `foundation/HARDWARE_CHECKLIST.md` has no numbered item 5. The HOLD outcome currently has nowhere to land.
- Sim only. Nothing here touches hardware, and physical deployment still needs a human and TA approval.

## What would flip this decision
- **Push at nominal PD.** If +2000's standing push at nominal PD is ≤ 0.21 on two eval seeds, drop the push HOLD for +2000. If P's or the control's push degrades past +2000's, re-decide which policy goes forward.
- **Hardware evidence that standing disturbances are rare or harmless.** For example, if the operator protocol keeps the robot on a stand or tether whenever it stands near people, the push condition could become a flag instead of a HOLD.
- **The control fails its confirm.** If the control +2000's seed-4321 confirm gives a nompd − flat_dr gap > 15, or push_stand > 0.21, then nothing is hardware-eligible and the hardware track waits for a human re-decision.
- **A Bumpy checkpoint recovers push.** A Bumpy-arm checkpoint with push_stand ≤ 0.21 that keeps nompd and bumpy_dr would reopen (a) and (b). None has been measured so far.
