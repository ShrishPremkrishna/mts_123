# Should the Part 3 Bumpy fine-tune change its PD-gain DR coverage and/or add a nominal-gain gate?
Status: grader-2 opinion (input to consolidation) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: High. It sets the training distribution of the one run that produces the robot policy.
Reversible?: Yes, and cheaply. A widened-kp fine-tune pair is ~2 × 1500 it ≈ 40 GPU-min of the shared pie plus about 1 h of evals. A suite eval takes about 1 min (`wall_clock_s` 36–65 s).

## Conclusion
**Option B with a non-regression guard.** Keep the Lab 4 ranges (kp ×[0.6, 1.1], kd ×[0.8, 1.5], friction [0.6, 1.4]) for **both** the Bumpy arm and the matched flat control. Add a **nominal-PD evaluation** at +0, +500, +1000, +1500, on the control and in `confirm`. The evaluation is flat, with `pd_gains` removed and everything else randomized, as in `part3_eval ablate --events pd_gains`. Add a **kp-only sweep** (all 12 joints fixed at kp ×0.85 / 0.95 / 1.0, kd still randomized) at +0 and on the shipped checkpoint. Add **one selection guard**: fall back to +1000 if +1500 is worse than +1000 by more than 10 falls on flat-nominal-PD. Nominal PD does not otherwise gate shipping. It is a reported sim-to-real risk with a pre-registered escalation (see the last section).

## Evidence used
- Parent ablation (`report/part3/eval/p3-parent/ablate/ablate.md`, eval seed 1234, 384 robot-runs): nominal PD gives **99 falls vs 40** at DR on (88 of them in stop contexts), vx +1.63/−1.63 vs +1.54/−1.50, and sat@1.5 23 % vs 17.9 %. Each other event is within ±10 falls. DR fully off gives 100. The ablation moves kp and kd together, so it cannot say which one matters. This is the Part 1 parent, not the final policy.
- `dr.pd_gains` (`envs/mdp/dr/actuator.py`) draws kp **independently per joint**, **once at startup** (mode `"startup"`; the notebook's "per episode" wording is wrong). My Monte Carlo (`scratchpad/graders/grader-2-kp/mc.txt`) gives the robot-average kp as 0.850 ± 0.042, so only **~31 of 4096** training robots average ≥ 0.95 and **~3** average ≥ 0.98. The nominal robot is out of distribution in aggregate stiffness, not merely "at the edge".
- The ranges are not team-specific. They are the course notebook default ("real motors run weaker or stronger than modeled") **and** upstream `pupperv3-monorepo/ai/rl` `position_control_kp_multiplier_range: [0.6, 1.1]`, around a kp that upstream also deploys at 1.0×. The deliberately soft-skewed range is the original authors' hardware prior.
- The real motor is plausibly softer than the ideal mjlab PD. The on-robot sim (`pupperv3_mujoco_sim/lib/actuator_model.hpp`, `pupper_v3_mujoco.ros2_control.xacro`) models back-EMF/current torque limiting and backlash, which mjlab's `BuiltinPositionActuator` (3 Nm clip only) lacks. I found **no hardware measurement** of effective gain in `foundation/results/`, where every gain field is a configuration value and `hardware_validated: false`.
- The spec says to run Bumpy "with your lab 4 DR ranges". The Lab 4 record, the notebook and the task defaults all agree. The deploy side keeps kp 5.5 and has `kp_scale`/`gain_multiplier` knobs, and changing those is a human decision.

## Thought process
Criteria, in priority order:
1. Spec compliance and a clean comparison. Control arm, one change per run, matched iterations (REPRO_PLAN §3).
2. Rank on sim-to-real readouts, and keep sim and hardware evidence separate (§3, AGENTS.md).
3. Cost and reversibility.

The finding is real *in sim*: it is a 2.5× fall increase, far outside the 10–15 noise band. But "nominal" means an idealised actuator, and every unmodelled mechanism I found (backlash, back-EMF, kt drift) pushes the real loop *softer*. So it is unknown whether the real robot sits near ×1.0 or near ×0.85. Simulation cannot settle that (the notebook says so too).

Recentring (e.g. ×[0.75, 1.25]) only moves the hole: robot-average 0.85 becomes a 3.6σ outlier instead. Widening enough to cover both (e.g. ×[0.6, 1.3], robot mean 0.95 ± 0.058) deviates from the spec's explicit instruction, and so does any recentring. Either one also adds a second untested change to the single robot-bound run, with no smoke evidence of its cost: the notebook warns that "too wide trains a timid crouch".

Keeping the ranges costs nothing, and measuring is nearly free (about 1 min per suite). The remedy is cheap later (< 1 GPU-h) if the final policy or the robot shows the problem. A gate on nominal PD can only choose among checkpoints trained on the same distribution, so it is a non-regression guard, not a fix. The kp-only sweep separates kp from kd and locates the cliff between 0.85 and 1.0. That tells the hardware operator how sharp the risk is.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| A: report only | Leaves the final policy unmeasured at the deployed gain, and the measurement costs about 1 min. |
| C: recentre to ×[0.75, 1.25] | Moves the coverage hole to the soft side, which the evidence says is likelier. Spec deviation. Untested. |
| C: widen to ×[0.6, 1.3] (both arms) | Best coverage, but it breaks the spec, confounds the robustness pass with a DR change (+0 was trained on Lab 4), and has no smoke. Kept as the escalation path. |
| Hard nominal-PD gate (block shipping) | The fine-tune cannot fix a coverage hole, so blocking stalls Part 3 without curing anything. |
| Per-robot scalar kp + per-joint jitter | The right shape for aggregate coverage, but it needs new DR code. Not for this run. |

## Severe flagged issues
- **Sim-to-real (most severe).** If the robot's effective gain is near ×1.0, the parent sim predicts speed overshoot, backward-seam pitching and about 2.5× falls at speed. Hardware checklist item 5 must stay mandatory: tethered, ramped speed, watch for overshoot and pitch.
- The evidence is the parent on one eval seed, and the ablation conflates kp and kd. Re-measure on the Part 2 final at +0 before reading anything into it.
- Under Lab 4, aggregate kp ≥ 0.95 is ~0.75 % of training robots. This is a structural DR property (independent per-joint draws), not just a range issue.

## What would flip this decision
- **Escalate to the C arm pair** if, on the Part 2 final at +0 or on the selected checkpoint, flat-nominal-PD falls are ≥ 2× flat_dr **and** > 15 more. That means a new 1500-iteration pair at kp ×[0.6, 1.3], kd unchanged, with the flat control at the same ranges. Gate it on flat_dr and flat-nominal-PD within 10 falls of each other. Before the full run, a ≤ 50-iteration smoke must show zero overflow, no NaN and ep length ≥ 450, and its flat_dr must stay within 15 falls of the matched Lab 4 smoke. This is a new run that needs a fresh decision or human sign-off, because it departs from the spec wording.
- Hardware shows overshoot or pitching on backward speed-ups, or joint-tracking logs imply an effective kp near ×1.0 → same escalation.
- The kp-only sweep is flat from 0.85 to 1.0 (the damage comes from kd) → revisit kd coverage instead.
