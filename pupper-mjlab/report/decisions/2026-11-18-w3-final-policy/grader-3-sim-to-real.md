# Which Part 2 checkpoint is the final policy that goes to Part 3? (grader 3, lens: sim-to-real / safety skeptic)
Status: DECIDED by grader (pending synthesis) · 2026-11-18 · Requested by: W3
Criticality: Critical. It feeds the report's headline claim and is the policy Part 3 fine-tunes and may later prepare for hardware.
Reversible?: yes before Part 3 launches, at no cost. After launch, it costs the Part 3 arm pair (~2 × 1500 it, ~40 GPU-min) plus ~1 h of evals, and a rewrite of the report.

## Conclusion
**`~/runs/opt-lab-2/rp-p2-s42-r2-cont-1500/model_4496.pt`** (arm r2-cont-1500, seed 42, WandB `wmhcp24q`, sha256 `ce977121…`).
- Reference dir: **`configs/opt-lab-2/p2/s42/r2-lag`**.
- Lineage: the designated parent `1c5r0n8c`.
- Condition: it is **not hardware-eligible as-is**. Its nominal-PD gap (+22.5) sends Part 3 step 0 to WIDEN, and only a Part 3 checkpoint that clears both HOLD rules may go to the robot.

## Evidence used
- **Falls at both plausible robot gains.** The robot's effective PD gain is unknown, so falls are read at both ends. Each pair is seeds 1234 / 4321 (`nompd_check.md`, `w3-nompd/`).

  | Checkpoint | At nominal gain (nompd) | At soft DR gain (flat_dr) | Gap |
  |---|---|---|---|
  | s42-r2-cont-1500 | 39 / 37 = 38 | 14 / 17 | +22.5 |
  | s43-r3 | 30 / 37 = 33.5 | 22 / 27 | +9.0 |

  - The 4.5-fall difference at nominal gain is a tie.
  - Across its four DR evals (W2 suite + repeat, flat_dr × 2), s42-r2c falls 16, 17, 14, 17 (mean 16). s43-r3 falls 16, 29, 22, 27 (mean 23.5); its 16 → 29 is the largest repeat swing in `repeat_spread.md`.
  - Stops dominate the falls in both: nompd stop falls are 27 / 30 for s42-r2c and 28 / 31 for s43-r3.
- **sat@1.5 gate** (repeat band 0.0006).
  - The r2-cont-1500 arm is −0.025 / −0.024 vs ctrl-1500 (both seeds agree), so it passes the gate.
  - Levels: s42-r2c 0.160 vs s43-r3 0.151. Under nompd: 0.209 vs 0.196.
  - tau2: 1.357 vs 1.328.
- **Foot force** (band 0.076 BW). s42-r2c is the worst +1500 r-arm here.
  - Median per-run max (F_runmax): s42-r2c 3.04 / 3.11 BW (its own control: 2.91 / 2.87); s43-r3 2.77 / 2.81. Under nompd: 3.32 / 3.40 vs 3.12 / 3.13.
  - p99: 1.83 vs 1.75.
  - Peak flags: 6.9 / 7.0 vs 5.5 / 7.4. The 12.0 for s43-r3 is the incl-reset-drop value only, not policy behaviour.
- **Push** (band 0.040; `*_push.json`).

  | Checkpoint | Standing @0.65 | Standing @0.8 | Standing @0.5 |
  |---|---|---|---|
  | s42-r2c | 0.099 / 0.090 | 0.53 / 0.54 | 0 |
  | s43-r3 | 0.334 / 0.294 | 0.97 | 0.018 / 0.034 |

  - s43-r3 is the only r-arm that falls at 0.5 m/s while standing. It also has 1 / 7 falls outside the push windows.
  - Moving @0.5: 0 for both.
  - Arm level: r2-cont-1500 is the only +1500 arm that improves on both seeds, on falls (−19 / −3) and push_stand (−0.20 / −0.35). r3 is a fall tie (−12 / −2), and its push_stand disagrees in sign.
- **Drift and splay.**
  - Drift+ is 31 / 32 °/10 s for s42-r2c vs 41 / 41 for s43-r3. Drift− is 47 vs 44.
  - Hind splay at fwd / back is 0.027 / 0.079 vs 0.044 / 0.100 rad.
- **Reference margins** (`foldcheck.json`).
  - s42 r2-lag: kp·Δq 2.24 / 2.45 Nm, limit margin 0.336 / 0.328 rad, peak 21–23 rad/s, with no flags or warnings.
  - s43 r3-lag: 2.34 / 2.12 Nm and 0.345 / 0.315 rad.
  - gallop_c (all ctrl arms): 3.82 Nm, above the 3 Nm limit, 0.245 rad and 44.8 rad/s.

## Thought process
I ranked deployment risk first (REPRO_PLAN §3), in this order:
1. falls at the gains the robot may actually have;
2. the sat gate;
3. push;
4. impact and torque load.

Tracking and reward were discounted.

**Falls.** The decisive point is that the robot's gain is unknown, at about 1.0× or about 0.85× (`w4-kp-coverage`).
- At nominal gain, s42-r2c and s43-r3 tie (38 vs 33.5).
- At DR gains, s42-r2c is lower in all four evals, and it is the more repeatable of the two.
- So on absolute falls, s42-r2c is never worse beyond noise in either hardware scenario. Its larger gap comes from being *better under DR*, not worse at nominal.

**What s43-r3 has going for it.** Its gap of +9 and its low nompd are a trait of the s43 lineage, not something r3 produced:
- s43-ctrl-1500 also has nompd 31 / 31 and a gap of +12;
- s42-r3 has 46.5 and +24.

s43-r3's real advantages are about 0.009 on sat@1.5 and about 0.27 BW on F_runmax. Both are measurable but modest (about 6–9 %).

**Standing push.** s42-r2c is about 3× better at 0.65 m/s and stays below 1.0 at 0.8 m/s. A bump while standing is a common hardware failure, so I weighted this more heavily.

**The gap is not ignored.** It routes Part 3 to WIDEN, which is already selected and smoke-PASSED for the s42 parent (Δ 48.5). The HOLD rules then guard the robot.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| s43-r3 (`rp-p2-s43-r3/model_4496.pt`, `p2/s43/r3-lag`) | Strongest runner-up. It has the lowest sat@1.5, F_runmax and nompd sat, and the only HOLD(a)-passing gap. But its nompd falls tie with the pick, it falls more under DR and less repeatably, its standing push is 3× worse, and it is the only r-arm that falls at a 0.5 m/s standing push. Its gap advantage is lineage-wide, not r3's. It also switches parent lineage. |
| s43-r2-cont-1500 | Same arm, but weaker: falls 15 / 16 and flat_dr 23 / 16, push @0.8 0.97, F_runmax 2.96. It has no advantage over s42-r2c beyond the gap (+17.5, still > 15). |
| s42-r3 | nompd 46.5 (gap +24) and push_stand 0.40. |
| r1-cont-1500 (either seed) | sat@1.5 0.170–0.174 (the smallest gate margin), nompd 44 / 49, push_stand 0.25–0.32. |
| s43-ctrl-1500 | Has the lowest nompd (31), but sat@1.5 is 0.181, push_stand 0.50, and it falls at a 0.5 m/s standing push. Its gallop_c reference demands 3.82 Nm, above the limit. |
| Keep the s42 parent | Worse on every hardware readout: nompd 99 / 79, gap +48.5, sat@1.5 0.179, push_stand 0.455. |
| +500 r1-naive (−15.5 falls) | It has no nominal-PD evidence, and the W4 rule makes nompd part of selection. |

## Severe flagged issues
- **The pick fails HOLD(a) as-is** (+22.5 > 15). It must not reach hardware without Part 3 WIDEN closing the gap, and HOLD(b) then caps nompd at 48.
- **Push was never measured at nominal gains.** The pick's main safety advantage (standing push) is DR-only evidence, and stop falls dominate at nominal for every candidate.
- **The pick has the highest foot force among the r-arm candidates**: +0.19 BW vs its control (2.5× the band) and +0.24 under nompd vs s43-r3.
- **Torque reality.** Every candidate saturates on about 15–16 % of samples at ±1.5 m/s against 3 Nm. The robot-side back-EMF model caps torque at about 1.37 Nm at stall (unvalidated), so first hardware trials should cap commanded speed.
- **Thin eval seeds.** W2 and push used only seed 1234. nompd has two seeds and was measured at +1500 only. The cause of s43-r3's 7 falls outside the push windows is unknown.

## What would change my mind
- **Push at nominal gains** (standing@0.65, moving@0.5) on s42-r2c and s43-r3. If s42-r2c's standing advantage falls inside about 0.05, I would switch to s43-r3, which has the lower torque, impact and gap.
- **A third nompd seed** (e.g. 777) in which s43-r3 is ≥ 10 falls below s42-r2c.
- **A hardware stiffness measurement** showing the robot sits at about 1.0× kp, combined with a Part 3 WIDEN result that still HOLDs s42-r2c's descendant.
- **A foot-force or torque limit from the hardware side** (gearbox, link, or a validated EMF cap) that separates about 2.8 BW / 0.151 from about 3.1 BW / 0.160.
- **A ruling** that the Part 2 final must itself pass nompd − flat_dr ≤ 15. Then s43-r3 is the only r-arm that does.
