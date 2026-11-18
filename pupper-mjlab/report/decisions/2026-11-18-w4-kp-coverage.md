# Should the Part 3 Bumpy fine-tune change its PD-gain DR coverage and/or gate on nominal gains?
Status: DECIDED by arbitrator (pending human review) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: High. It sets the training distribution of the one run that produces the robot policy, and it addresses the largest sim-to-real flag so far.
Reversible?: yes. A re-run of both arms is ~2 × 1500 it (~40 GPU-min of the shared pie) plus ~1 h of evals. The added evals cost ~10 GPU-min. No hardware is involved.
Grader docs: [grader-1](2026-11-18-w4-kp-coverage/grader-1.md) (D: widen + gate) · [grader-2](2026-11-18-w4-kp-coverage/grader-2.md) (B + guard, escalate to widen) · [grader-3](2026-11-18-w4-kp-coverage/grader-3.md) (B + hardware flag, widen as contingency). Arbitrator scratch: `scratchpad/graders/arbitrator-kp/mc.txt`.

## Conclusion
**(D) as a pre-registered branch, decided by one measurement before launch.**

**Step 0, before launch.** On the Part 2 final checkpoint P (the fine-tune's +0), run `flat_dr` (suite) and `part3_eval ablate --events pd_gains` ("nompd") on eval seeds 1234 and 4321. Then compute Δ = nompd falls − flat_dr falls, per seed and averaged over the two seeds.

**If mean Δ > 15: WIDEN.**
- Both arms (the Bumpy fine-tune and its matched flat control) train with **kp ×[0.6, 1.3], kd ×[0.7, 1.5]**.
- Friction stays at [0.6, 1.4], and every other DR event is unchanged.
- Evaluation stays on the exact Lab 4 ranges: `flat_dr`, `bumpy_dr` and the seam, with `check_lab4_dr` still asserting them.
- The wider training range is reported as a deviation. It is a superset of the Lab 4 ranges.

**If mean Δ ≤ 15: KEEP.** Both arms train with the Lab 4 ranges (option B).

**In both branches**, nompd runs at the following points:
- **Seed 1234:** fine-tune checkpoints +500, +1000 and +1500, plus the control at +1500.
- **Seed 4321:** the selected checkpoint (added to `confirm`).

nompd joins **selection**: fall back to +1000 if +1500 has more than 10 more nompd falls than +1000. This sits alongside the budget decision's guards on flat_dr, bumpy_dr and the seam.

**Deployment HOLD.** The selected checkpoint is held from hardware if either of these holds, using two-seed means:
- (a) nompd − flat_dr > 15;
- (b) nompd(selected) > nompd(P) + 10.

**Escalation.**
- A HOLD in the KEEP branch triggers the WIDEN arm pair. This doc pre-authorizes it, with the same smoke and rules.
- A HOLD in the WIDEN branch triggers no more training. It goes to a re-decision with the human.
- In the WIDEN branch only: if the flat control's flat_dr at +1500 has more than 10 more falls than P's flat_dr (seed 1234), the widening has cost Lab 4 robustness. That is also a HOLD and a re-decision.
- Record the outcome in hardware checklist item 5.

**Smoke (WIDEN only).** Run ≤ 50 iterations per arm from P. It must show all of the following:
1. The run config and W&B tags record pd_gains kp (0.6, 1.3) and kd (0.7, 1.5), with the other DR unchanged. `lab4_runs/shrish_stablegait/training-settings.json` is byte-unchanged, and the eval paths still pass `check_lab4_dr`.
2. The realized `actuator gain / 5.5` spans ≈ 0.6–1.3, and kd/0.25 spans ≈ 0.7–1.5. Between 17 % and 23 % of envs have a 12-joint mean kp ≥ 1.0 (Monte Carlo: 19.9 %).
3. Overflow is 0, there are no NaNs, the final episode length is ≥ 450, and err_xy is ≤ 0.30. The Lab 4 smoke `c3rfre1p` reached ~470 and ~0.23.
4. The exported `policy.json` still says kp 5.5 and kd 0.25, and the round trip and limit audit PASS.
5. nompd and the extended `select` run end to end on the smoke checkpoints. This is a pipeline check, not a result.

In the KEEP branch, training is unchanged, so only item 5 is needed (on the existing `*-smoke` model_3048).

Implementation:
- Add a training-only PD-gain DR override to `part3_finetune.py`, used by both arms, with the tag `p3dr-widekp` replacing `lab4dr`.
- Add the nompd runs and the guards to `part3_run.py` (`eval-ft`, `select`, `confirm`).

## Evidence used
- **Parent ablation** (`report/part3/eval/p3-parent/ablate/ablate.md`; Part 1 parent, seed 1234, 384 runs). nompd gives **99 falls vs 40**, with 88 in stops. Speed reaches vx ±1.63 for a ±1.5 command, and sat@1.5 is 23 % vs 17.9 %. Every other event at nominal gives 21–50 falls. DR fully off gives 100.
- **Code checks** (I verified these).
  - `dr.pd_gains` draws one independent multiplier per joint, once at `startup`.
  - Nominal is kp 5.5 and kd 0.25 (`pupper_constants.py`), which equals what `policy.json` commands. `gain_multiplier` and `kp_scale` are 1.0 in `config.yaml`.
  - `ablate` only removes the event: the other DR is still randomized, and it runs the suite only (no seam).
  - The Part 2 final has not landed, so no +0 nompd number exists.
- **Monte Carlo of the 12-joint mean gain** (`mc.txt`):

  | Ranges | Mean kp | Mean kd | Envs near nominal out of 4096 (kp 0.95–1.05 and kd 0.9–1.1) | Envs on the soft side (mean kp 0.8–0.9) |
  |---|---|---|---|---|
  | Lab 4 | 0.850 ± 0.042 | 1.150 ± 0.058 | 6 | 3138 |
  | kp [0.6, 1.3], kd [0.7, 1.5] | 0.950 ± 0.058 | 1.100 ± 0.067 | 936 | 795 |
  | kp [0.8, 1.2] (recentre) | – | – | 701 | 4 |

- **Robot-side actuator model** (`actuator_model.hpp` with the `ros2_control.xacro` parameters 24 V, 0.7 Ω, kt 0.04). It caps torque by back-EMF at 1.37 Nm at stall, against a 3 Nm limit in training. That is a torque cap, not a lower small-signal stiffness, and nothing has validated it on hardware. No measurement of effective gain exists in `foundation/results/`.
- **Upstream** `ai/rl/pupper_mjx_rl_use_this.py` also uses kp ×(0.6, 1.1) and kd ×(0.8, 1.5). The Lab 4 ranges are the course prior, not a team choice.

## Thought process
Criteria, in order:
1. Sim-to-real readouts (REPRO_PLAN §3).
2. Keep sim and hardware evidence separate, and don't bet the robot-bound run on an unmeasured hypothesis.
3. Spec fidelity.
4. Matched control and noise rules (§3).

**The robot's effective gain is unknown.** It could be soft, as the course prior suggests, or 1.0×, as commanded.

**Recentring loses the soft side.** It moves the coverage hole to the soft side, which leaves 4 envs there.

**Widening covers both.** It keeps 795 envs on the soft side and adds 936 near nominal. Every Lab 4 per-joint draw stays in range, so it is the hedge that does not bet against grader-3's soft-robot prior.

**A gate alone cannot fix anything.** Every checkpoint shares the same training distribution.

**Why measure +0 first.** All three graders already make their answer depend on a +0 nompd number: grader-1's exception, grader-2's escalation, grader-3's flag. Measuring it before the spend costs ~4 suite-minutes. Measuring it after would cost a second 40-GPU-min pair.

**Thresholds.**
- 15 falls on a two-seed mean is above the one-seed noise band of 10–15.
- 10 falls for checkpoint-to-checkpoint guards matches the budget decision.

**kd lower bound.** I chose 0.7, not 0.6. It is enough to put nominal kd inside the aggregate bulk: P(mean kd ≤ 1.0) is 6.7 %, and the near-nominal count is 936. A lower bound of 0.6 would add an unmeasured low-damping corner.

**The matched control uses the same ranges** as the fine-tune, so Bumpy vs flat stays a single difference.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| (A) Report only | It ships a policy that, on the parent, is 2.5× worse at the commanded gain, and leaves that unmeasured. |
| (B) unconditionally (grader-2, grader-3) | It commits the robot-bound run before the deciding number exists. If the gap persists, a second pair is needed. |
| (D) unconditionally (grader-1) | It deviates from the spec even if the Part 2 final no longer shows the gap. Grader-1 itself carved out this exception. |
| (C) Recentre to kp [0.8, 1.2] | Drops the soft side: 4 envs at mean kp 0.8–0.9. Lab 4 is no longer a subset. |
| kd [0.6, 1.5] (grader-1) or kd unchanged (grader-2) | 0.6 adds an untested low-damping corner. Unchanged leaves nominal kd at the 0.4 % tail. |
| nompd as hardware-only, out of selection (grader-3) | 10-fall one-seed guards already govern selection on flat_dr and bumpy_dr. The HOLD uses two-seed means. |

## Disagreement & resolution
- **Factual (resolved; no dispute after checking).** Effect size, startup and per-joint sampling, deploy gain = 1.0×, no hardware gain measurement, and the spec wording were the same in all three graders' docs, and I confirmed them.
  - Grader-3's 1.37 Nm figure is arithmetically correct (24/0.7 × 0.04).
  - But that model limits torque; it does not lower stiffness, and it is unvalidated. So it does not show the robot sits at ≈ 0.85×.
  - Grader-2's upstream-range claim is correct.
- **Criteria (resolved by sequencing).**
  - Grader-1 ranks coverage of the commanded gain first.
  - Graders 2 and 3 rank spec fidelity and "don't change the one run without evidence" first.
  - All three make their choice depend on the same +0 nompd measurement, so I moved it before launch with a pre-registered threshold.
  - A superset range answers grader-3's soft-robot concern. Evaluating on exact Lab 4 answers the spec concern.
- **Minor.**
  - The kd bound takes the middle of the three positions.
  - nompd enters selection, as graders 1 and 2 wanted.
  - Grader-1's seam-on-nompd requirement is dropped: the tooling does not support it.

## Severe flagged issues
- The evidence is from the Part 1 parent, one seed. kp and kd are not separated, because `ablate` sets both to nominal. Step 0 re-measures on P.
- WIDEN is a spec deviation, reported as a training-range superset. Evaluation stays exact Lab 4.
- WIDEN confound: a nompd gain vs P cannot be split between "wider DR" and "1500 more iterations". There is no Lab 4 control arm, by the budget decision.
- Sim only. Whether the robot is at 1.0× or ≈ 0.85× is a hardware fact. The 1.37 Nm EMF cap is a separate, unvalidated saturation flag. Hardware checklist item 5 stays mandatory.
- Independent per-joint sampling is the structural cause. A correlated per-robot multiplier would fix it, but that is future work, not this run.
- The course notebook says gains are sampled "per episode", but the code samples them at startup.

## What would flip this decision
- A hardware step-response or stiffness measurement at kp 5.5 → set ranges centred on the measured gain (re-decide).
- In the WIDEN smoke, any item 1–4 fails → do not launch. Fall back to KEEP and re-decide.
- A TA or human rules that the training ranges must be exactly Lab 4 → KEEP, with the HOLD rule unchanged.
- nompd at P differs by more than 15 between the two seeds → the measurement is too noisy. Add a third seed (e.g. 777) before branching.
