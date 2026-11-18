# Should the Part 3 Bumpy fine-tune change its PD-gain DR coverage and/or gate selection on nominal gains?
Status: DECIDED by grader-3 (1 of 3; pending agreement check and human review) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: High. It sets the training distribution of the one run that produces the policy meant for the robot.
Reversible?: yes. The added evals cost ~10–15 GPU-minutes. A recentred re-fine-tune later (2 × 1500 it) costs ~40 GPU-min plus ~1 h of evals and needs no other rework.

## Conclusion
**Option (B): keep the Lab 4 ranges (kp ×[0.6, 1.1], kd ×[0.8, 1.5]) for both the fine-tune and its matched flat control, and add a nominal-PD readout that can block hardware but does not change checkpoint selection.**
- Run `part3_eval ablate --events pd_gains` (flat suite, kp = kd = 1.0×, other DR events still randomized) at +0, +1000, +1500 and on the +1500 control. Repeat it on seed 4321 for +0 and for the selected checkpoint.
- Selection keeps the budget decision's guards on flat_dr, bumpy_dr and the flat_dr seam verdict.
- **G-nom (hardware flag).** It fires if, on both eval seeds, the selected checkpoint has more than 15 more nominal-PD falls than +0. If it fires, deployment waits for a re-decision. Report the nominal-PD and flat_nodr numbers in the summary table and in hardware checklist item 5.

## Evidence used
- **Ablation** (`report/part3/eval/p3-parent/ablate/ablate.md`, 384 robot runs, seed 1234, Part 1 parent only):
  - nominal pd_gains: 99 falls vs 40 with DR on (88 of the 99 are stops), vx ±1.63 for a ±1.5 command, sat@1.5 23 % vs 17.9 %;
  - every other event at nominal stays within noise: 21–50 falls.
  - The effect is real in sim, roughly 2.5× and far outside the 10–15-fall band.
- **Why nominal falls outside the training distribution** (my computation, `scratchpad/graders/grader-3-kp/joint_mean_coverage.txt`). `dr.pd_gains` draws each of the 12 joints independently, once per env (`mode="startup"`). Per joint, 1.0× sits *inside* [0.6, 1.1], at the 80th percentile. The whole-robot mean gain is another matter:
  - Lab 4: the 12-joint kp mean has p0.1/p50/p99.9 = 0.725/0.85/0.975, so a robot uniformly at 1.0× is never sampled. kd = 1.0× is at the 0.4 % tail.
  - Recentring to [0.8, 1.2] would move the mean to 0.90–1.10, which drops a robot uniformly at 0.85× out of coverage in the same way.
  - Widening to [0.6, 1.3] (mean 0.775–1.125) would cover both 0.85× and 1.0×.
- **Course prior.** The course notebook (cell 11–12: "real motors run weaker or stronger than modeled") ships (0.6, 1.1), centred below 1.0, and its deploy pipeline commands kp 5.5 with that DR. The team's Lab 4 record copies it unchanged.
- **Actuator models.** The course's robot-side MuJoCo model (`pupperv3_mujoco_sim/lib/actuator_model.hpp` with the `pupper_v3_mujoco.ros2_control.xacro` params: 24 V, kt 0.04, R 0.7 Ω) caps torque by back-EMF/current at ≈1.37 Nm (1.35 at 10 rad/s). The training sim uses an ideal 3 Nm `BuiltinPositionActuator`. If that model is right, the real actuator is *weaker* than the sim's nominal, most of all at gallop speed. I have not verified it against hardware.
- **No hardware evidence on gains or motor strength.** The `foundation/results` hits are kp settings and stationary sessions only.
- **Spec** (`part3.txt`): "Run the Bumpy variant with *your lab 4 DR ranges* as the robustness pass."

## Thought process
Criteria in priority order:
1. **Sim-to-real readouts** (REPRO_PLAN §3).
2. **Keep sim and hardware evidence separate** (AGENTS.md). Do not retarget the one deploy run on a sim-only hypothesis.
3. **Spec fidelity.**
4. **Matched control and a single change per run** (§3).

The finding shows the policy is fragile *if* the robot is uniformly at the model's nominal stiffness. It does not say whether the robot is. The three available priors (the course's chosen centre, its robot-side actuator model, and current/back-EMF limits at 1.5 m/s) all point to effective stiffness at or below nominal. That makes sim-nominal closer to a pessimistic corner than to the expected robot.

Recentring bets the opposite way. It would open the same coverage hole at 0.85×. It also changes the training distribution in the same run as Bumpy, which confounds the robustness credit. The control would have to change too, so neither arm would match the Part 2 distribution. And it departs from the spec's explicit wording.

Widening is the best hedge if a change were needed. It still deviates from the spec, risks the "timid crouch", and gets only 1500 iterations to adapt. Without hardware evidence, that cost is not justified.

Measuring is nearly free. G-nom makes sure the fine-tune does not quietly make the deploy-gain behaviour *worse* than the Part 2 final, and that the human sees the number before deploying. G-nom stays out of automatic selection. One eval seed ±10–15 falls would turn it into selection on noise, and the budget decision treats intermediate evals as diagnostics, not a menu.

**Smoke:** training does not change, so no training smoke is needed. Before the full runbook, check the new stage on the existing smoke checkpoints (`rp-p3-parent-bumpyft-smoke` and `-flatctrl-smoke` model_3048). It should write `ablate.md` with a pd_gains row for each, and `check_lab4_dr` should still pass.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| (A) Keep, report only | Leaves no measurement on the final policy. A fine-tune that worsens nominal-gain behaviour could ship unnoticed. |
| (C1) Recentre kp to [0.8, 1.2] (or kd to [0.7, 1.3]) | Moves the coverage hole to 0.85×, against the course prior, with no hardware evidence. Confounds Bumpy with DR in the one run. Deviates from the spec. |
| (C2) Widen kp to [0.6, 1.3], kd to [0.7, 1.5], both arms | The best hedge if hardware or a calibrated actuator model shows ≈1.0×. Today it is a spec deviation with an untested cost on DR-on metrics within 1500 iterations. Kept as the pre-specified contingency. |
| (B) with G-nom as an automatic re-selection | One-seed noise. It also overrides the budget decision's "+1500 unless a guard fires" on an untrained-for condition. |

## Severe flagged issues
- **The ablation sets kp and kd together.** The notebook says "kp", but kd = 1.0× is just as far out (0.4 % tail of the joint-mean kd). Attribution is unproven. Run kp-only and kd-only variants at +0 if possible; this needs a small `ablate` extension.
- **Independent per-joint sampling gives almost no whole-robot stiffness coverage** (σ ≈ 0.04 around 0.85). That is the structural cause, and changing the range does not fix it; a correlated global multiplier would. Flag it for future training, not for this run.
- **Parent-only, one seed.** The final policy may differ.
- The notebook describes nominal as "the edge" of the distribution. Per joint, it is the interior. For the whole robot, it is beyond the 99.9th percentile. The text should say this.
- The course notebook says gains are "sampled per episode", but the code samples once at startup.
- The robot-side 1.37 Nm torque cap is far below the 3 Nm training limit. If that model is right, saturation at 1.5 m/s on hardware is underestimated. This is a separate sim-to-real flag.
- On the robot, `robot_trims` kp_scale 0.85 (kp 4.675) would match the training mean. That changes a reviewed value, so it is a human decision and is not taken here.

## What would flip this decision
- Hardware shows the predicted signature: speed overshoot above the command, or pitching on backward speed-ups.
- A measured joint step response or stiffness at kp 5.5 matches the sim at 1.0×.
- The course's actuator model is shown not to apply.

Any of these → choose (C2) for a re-fine-tune, with both arms on the same widened ranges and a 50-iteration smoke showing no overflow and ep length ≥ 400 at the end.
- G-nom fires on the final → re-decide before hardware.
