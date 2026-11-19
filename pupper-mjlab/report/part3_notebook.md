# Part 3 lab notebook: multi-gait distillation (simulation side, W4)

Spec: [Opt Lab 2, Part 3](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-2.html).
Started 2026-11-18. **Everything here is simulation evidence. No policy was deployed to hardware.**
Hardware steps need a human operator and TA approval (section 6).

Status (2026-11-19): **Part 3 sim run on the Part 2 final policy is DONE** (section 0, the final results).
- **Sim candidate:** Bumpy (WIDEN) +2000.
- **Hardware eligibility is on HOLD** pending a human (post-hoc push gate; [decision](decisions/2026-11-18-w4-p3-candidate.md)).

Sections 1–8 describe the tools and the earlier **dry run on the Part 1 parent** (labelled NOT final).

Tags: **[M]** measured here · **[R]** recovered from the old run / repo records · **[D]** lazy decision (doc linked).

## 0. FINAL RESULTS: Part 3 on the Part 2 final policy (2026-11-18/19) [M]

Everything in this section is the real run. The dry run on the Part 1 parent is section 4 and is **not** final.
Simulation only; nothing was deployed.

**Input.**
- P = W3's Part 2 final, `~/runs/opt-lab-2/rp-p2-s42-r2-cont-1500/model_4496.pt`: W&B `wmhcp24q`, sha256 `ce977121…`,
  decision `ac5f63a`.
- Reference `configs/opt-lab-2/p2/s42/r2-lag` @ `60ce5a4`.

**Run.**
- One command: `scripts/opt_lab_2/part3_run.py --checkpoint <P> --reference-dir <ref> --name p3-final`.
- Log `~/runs/opt-lab-2/p3-final-runbook.log`. Every launch went through `gpu_slot.py`.
- Wall clock ≈ 2.5 h, because the GPU was otherwise free: export 1 min, eval 6 min, video 8 min, kpcheck 2 min,
  WIDEN smoke 5 min, training + gates 2.0 h (4 paired legs at ~2.0 s/iteration per arm), confirm 4 min, export 1 min.
- Outputs:
  - `report/part3/eval/p3-final*` (tables: `p3-final-kpwide_summary.md`, `_gates.md`, `_selection.md`, `_deploy_hold.md`);
  - `report/part3/export/p3-final*.md`;
  - diagnostics in `report/part3/diag/` (README).

**0.1 P before Part 3** (seed 1234; `report/part3/eval/p3-final/summary.md`):

| Condition | falls (rev/stop/steady/turn) | err +1.5 / −1.5 | push stand@0.65 / move@0.5 | seam |
|---|---|---|---|---|
| flat, Lab 4 DR | 14 (5/9/0/0) | 0.096 / 0.116 | 0.096 / 0 | PASS (headline) |
| flat, DR off | 24 | 0.185 / 0.217 | – | PASS |
| Bumpy, Lab 4 DR | 35 (2/18/15/0) | 0.124 / 0.131 | 0.159 / 0.022 | PASS (robustness) |
| Bumpy, DR off | 30 | 0.163 / 0.168 | – | PASS |

- **Export of P:** PASS. JSON vs torch 5.5e-6, ONNX 3.5e-6, limits unchanged.
- **Seam:** P does not stumble at the seam, under any condition. Its videos are in `~/runs/opt-lab-2/p3-videos/p3-final/`.

**0.2 kp coverage, step 0** [D]. flat_dr falls 14 / 16 vs nominal-PD falls 39 / 37 (seeds 1234 / 4321), a mean Δ of
**+23.0**, which is > 15, so the branch is **WIDEN**. Both arms trained with kp ×[0.6, 1.3] and kd ×[0.7, 1.5]; evals
stayed on Lab 4 ranges. This matches W3's independent +22.5. The WIDEN smoke (50 + 50 iterations) **PASSED** every check
(`p3-final_kpwide_smoke.md`).

**0.3 Training: paired 500-iteration legs, Bumpy arm vs matched flat control** (seed 1234; flat_dr / bumpy_dr /
nominal-PD falls; `p3-final-kpwide_gates.md`):

| point | Bumpy (WIDEN) | flat control (WIDEN) | gate |
|---|---|---|---|
| +0 (P) | 14 / 35 / 39 | – | |
| +500 | 14 / 31 / 25 | 16 / 38 / 28 | continue |
| +1000 | **34 / 46 / 42** | 18 / 47 / 34 | continue |
| +1500 | 11 / 23 / 14 | 23 / 52 / 51 | continue. Extension rule: bumpy_dr −23 > 10, flat_dr within 10 → **extend both** |
| +2000 | **10 / 18 / 7** | 12 / 29 / 20 | stop (bumpy_dr −5) |

- **The +1000 dip is real, not eval noise:**
  - Seed 4321 replicates it: Bumpy 19 → 39 flat_dr, 36 → 59 bumpy_dr.
  - Its neighbouring checkpoints are bad too: flat_dr 31–48.
  - The extra falls are all in stop/reversal transients.
  - Training-log metrics (reward, episode length, std, learning rate, realized gains) show no drift.
- **+1500 / +2000 are genuinely better.**
  - Neighbours of +1500: flat_dr 4–21, nominal-PD 9–34.
  - Neighbours of +2000: 6 / 14 and 4 / 6.
  - Checkpoint-to-checkpoint variance is ~±7–10 falls on top of eval noise (`report/part3/diag/README.md`).

**0.4 Selection and gates** [D].
- **Selection:** the pre-registered matched selection shipped **+2000** (`rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt`,
  sha256 `2632970b…`) with no regression vs +1500.
- **Seed-4321 confirm:** flat_dr 11, bumpy_dr 23, nominal-PD 12.
- **Seam:** PASS on flat_dr and bumpy_dr for both seeds and on the pooled windows. DR-off also PASS (flat 4 falls, Bumpy 18).
- **Pre-registered HOLD:** **CLEAR**.
  - (a) nominal-PD − flat_dr = −1.0 (limit 15);
  - (b) nominal-PD vs P = −28.5 (limit 10);
  - WIDEN control check −2 (limit 10).
- **Export of +2000:** PASS. JSON 7.1e-6, ONNX 3.9e-6, golden 1.1e-5, limits unchanged (kp 5.5, kd 0.25, 3 Nm, ±1.5 clip),
  deploy validator and C++ gait test pass. The W&B `policy.json` is absent (runner bug, section 5); no upload was made.

**0.5 What Bumpy + DR bought, and what it cost** (Bumpy +2000 vs P, seeds 1234/4321, and vs the matched control +2000):
- **Better on falls and nominal gains:**
  - flat_dr 10/11 vs P 14/16;
  - bumpy_dr 18/23 vs P 35;
  - nominal-PD 7/12 vs P 39/37 (control 12 / 29 / 20).
  - Against the matched control, the Bumpy-specific gain is within noise on one control seed.
- **Worse on standing push recovery** (a ranked W2 metric). Falls per push at 0.65 m/s:
  - Bumpy +2000: **0.489 / 0.497**;
  - P: 0.096 / 0.122; control: 0.091;
  - every Bumpy checkpoint from +500 on: 0.49–0.86; Bumpy neighbours: 0.56–0.86;
  - moving push is unchanged (≈ 0–0.02).

  The matched control (same PD ranges) keeps push, so the Bumpy terrain is the likely cause. Neither the Part 3
  selection guards nor the HOLD rule included push. **That gap in the pre-registration is acknowledged here.**
- **Critical decision** [D] (4 lenses agree, synthesizer; [`2026-11-18-w4-p3-candidate.md`](decisions/2026-11-18-w4-p3-candidate.md)):
  - (a) The **sim candidate is Bumpy +2000**. Its push regression must be stated next to every claim about it.
  - (b) It is CLEAR under the pre-registered HOLD.
  - (c) A post-hoc push gate **HOLD(c)** was added, and it can only add conservatism: two-seed flat_dr push_stand@0.65 must be ≤ P + 0.10 = 0.209.
    +2000 reads 0.493, so it is **HOLD**. Nothing is hardware-eligible, and it goes to a human. That human decides
    whether to ratify HOLD(c), with hold-all as the default and the flat control +2000 as the alternative (section 0.6).

**0.6 Option (ii) evidence: flat control +2000** (evals and export only, no training;
[addendum](decisions/2026-11-18-w4-p3-candidate/addendum-flatctrl-2000.md)):
- **Two seeds:**
  - flat_dr 12 / 13;
  - bumpy_dr 29 / 34;
  - nominal-PD 20 / 18;
  - push_stand@0.65 0.091 / 0.089 (Bumpy 0.163 / 0.151);
  - seam PASS everywhere.
- **Two-seed HOLD:** (a) +6.5, (b) −19.0, WIDEN check −2, HOLD(c) 0.090 ≤ 0.209. That is **CLEAR on all four**.
- **Export:** PASS (JSON 1.07e-5, ONNX 3.4e-6, limits unchanged, deploy validator and C++ test pass).
- **The 62 BW F_peak** is a single sample, the impact of a robot that fell 0.32 s later. p99.9 is 2.58 BW, the same as P.
- **Caveats:** it is not the spec's Bumpy pass and was not pre-registered, and its bumpy_dr is not better than P's
  beyond noise. Whether to prepare it for hardware is the human's call.

**0.7 Spec deliverables (sim side).**
- **Transition videos:**
  - P: `~/runs/opt-lab-2/p3-videos/p3-final/`;
  - selected +2000: `~/runs/opt-lab-2/p3-videos/p3-final-selected/`.
  - Both are flat and Bumpy, 0 → ±1.5 → 0 at 0.5 m/s² plus a seam dither, with no falls.
- **Bumpy + Lab 4 DR robustness pass:** done, with the documented WIDEN deviation on training PD ranges.
- **Export:** ready and audited.
- **Hardware deploy and the measured hallway top speed:** not done. Human-only, and on HOLD.

## 1. What the spec asks

Part 3, "Multi-Gait Distillation": train the full MixedGaits task (trot fore/aft, lift for turns, the
bootstrapped gait in the fast branch) into one policy, then make it survive reality:

1. **Transitions.** "Sweep the command from 0 through the fast-branch threshold and back. The blend logic
   hands the policy a different choreography mid-stride; does it stumble at the seam?" Report: transition
   videos (sim).
2. **Robustness.** "Run the `Bumpy` variant with your lab 4 DR ranges as the robustness pass."
3. **Deploy.** The controller rebuilds every reference table from `policy.json` with its own phase clock
   (zero controller changes). The trained ±1.5 m/s command clip ships inside the export.
4. **Measure.** Tape measure, stopwatch, straight hallway: actual m/s on the robot vs commanded.
   Report: deployed videos, measured top speed vs commanded, and where the sim-to-real gap bit hardest.

Items 1–3 (sim side) are this notebook. The "one policy" already exists: Part 1/2 train the full
`Mjlab-MixedGaits-Flat-Pupper-v3` task, with the trot, the lift and the authored/bootstrapped fast gait
in one network. Part 3's training work is therefore the Bumpy + DR robustness pass. Item 4 and the hardware
half of item 3 are human-only (section 6).

What the seam is, mechanically (`mdp/mixed_gaits.py`, constants in `pupper_gait_env_cfg.py`) [M, source]:
the fast slot plays when `|vx| >= 0.5` (`TROTGALLOP_ONSET`) and `|vx| >= 0.05`. At that instant **both the
table and the phase jump**: the trot runs `phase = frac(t · 1.333 Hz)`, the fast slot `frac(t · 1.333 · 2.25 Hz)`
(gallop_c's `frequency_mult`), from the same episode clock, so the reference is not continuous across the
switch. Backward commands play the time-reversed phase with the separate `reach_back` table.

## 2. Tools

All under `pupper-mjlab/`, run with `uv run --no-sync python ...`, `MUJOCO_GL=egl`. Each one reuses W2's
`part2_eval` for the rollout and the Part 2 metrics instead of duplicating it.

| Tool | What it does | How it was validated |
|---|---|---|
| `src/mjlab/scripts/part3_common.py` | Part 3 conditions on top of `part2_eval._Rollout`: asserts the Lab 4 DR ranges on the live config, applies the checked Bumpy contract (0.10 m grid + overflow audit), DR off (`nominal_mechanics`), one-event ablation, read-only torso-contact sensor. | Every output stamps `part3_condition`. Lab 4 record test in `tests/test_part3_seam.py`. Overflow audit is 0 on every Bumpy run. |
| `src/mjlab/scripts/part3_seam.py` | The seam test. `run` does the rollout, `metrics` re-scores offline from saved raw arrays, `pool` judges several eval seeds together (close-call rule), `video` renders the transition video. | 12 unit tests on synthetic arrays (`tests/test_part3_seam.py`). Parent dry run (section 4). |
| `src/mjlab/scripts/part3_eval.py` | Runs the matrix flat/Bumpy × DR on/off: Part 2 suite (384 robot-runs), push, seam. `table` summarizes; `ablate` sets one DR event at a time to nominal. | Parent dry run (section 4). |
| `scripts/opt_lab_2/part3_finetune.py` | Launches the Bumpy + Lab 4 DR fine-tune, or the matched flat control (`--flat-control`), through `launch.py`. | Config checks + 50-iteration smoke (section 3). |
| `scripts/opt_lab_2/launch.py --checked-bumpy` | New launcher option: trains through `foundation/scripts/bumpy_checked_cli.py`. | Smoke run: `physics_overflow_audit.json` zero. |
| `src/mjlab/scripts/part3_export.py` | Exports `policy.json` + the C++ golden fixture. Round-trips actions on recorded observations (torch / JSON / run auto-export / ONNX / W&B copy) and audits limits. Runs the deploy repo's validator and C++ gait-reference test. `--upload-wandb` uploads only on PASS and verifies the read-back. | 8 unit tests (`tests/test_part3_export.py`). Parent and smoke checkpoints PASS (section 5). |
| `scripts/opt_lab_2/part3_run.py` | The runbook. It runs every stage in order and skips finished work. The two training arms run together in gated 500-iteration legs; evaluations run one at a time. `--dry-run` prints the plan. | The parent dry run *is* this script. The amended leg/gate/selection logic has 6 unit tests (`tests/test_part3_run.py`) and a `--dry-run`. |
| `scripts/opt_lab_2/gpu_slot.py` | Atomic GPU slot: W3's `launch.lock` flock + the cap from `~/recovery/CONCURRENCY`, held until the process registers. | Unit tests for the cap and for lock blocking. |

### 2.1 The seam test (`part3_seam`)

**Schedules.** 16 schedules × 64 envs = 1024 robots. Every robot starts after a random 0–0.75 s delay
(one trot cycle), so its seam crossings land at uniformly spread gait phases, mid-stride.
- `sweep_<dir>_r<rate>`: 0 → ±1.5 → 0 at 0.25, 0.5, 1.0 and ∞ (step) m/s².
- `dither_<dir>`: 0.4 ↔ 0.6 m/s steps, eight crossings right at the threshold. This is the cleanest isolation
  of the seam.
- Controls with the same accelerations and no reference switch:
  - `control_<dir>`: 0 → 0.45 → 0, and 0.8 → 1.5 → 0.8, at 0.5 m/s².
  - `control_dither_{trot,fast}_<dir>`: 0.2 ↔ 0.4 and 0.8 ↔ 1.0 steps.

**Per crossing.** The window is [−0.2, +1.0] s around the first sample on the new side; it is invalid if the robot reset
less than 1 s before. Each window records:
- fall (termination)
- incident: a fall, a knee or torso ground contact, or body tilt > 30°
- 3 Nm saturation fraction
- peak foot force in actual body weights
- tracking error (in the window and in the 0.5 s before it)
- the jump in the 12-dim reference the policy observes

**Verdict** ([decision](decisions/2026-11-18-w4-seam-thresholds.md), arbitrated). Eight cells: {finite-rate sweep,
dither} × {fwd, back} × {up, down}, each compared against the pooled no-seam control. **FAIL** if either holds:
- pooled falls > 2 %;
- any cell's incident rate exceeds the control's by > 0.05 with one-sided Fisher p < 0.01.

`flat_dr` is the headline verdict, `bumpy_dr` the robustness verdict. The DR-off verdicts are diagnostics that
must be stated but don't gate. The step sweep is reported, not judged: it is a 0 ↔ 1.5 m/s jump, i.e. a hard start/stop.

## 3. Bumpy + Lab 4 DR robustness pass (recipe)

**No new physics was invented** [M, source]:
- **Bumpy** is the repo's `Mjlab-MixedGaits-Bumpy-Pupper-v3`: Perlin bumps 0–5 cm on 8 m patches. Its critic
  observation matches flat, so flat checkpoints warm-start it.
- **Lab 4 DR ranges.** The team's Lab 4 record (`lab4_runs/shrish_stablegait/training-settings.json`) is kp ×[0.6, 1.1],
  kd ×[0.8, 1.5], foot friction [0.6, 1.4]. These are the notebook defaults *and* the task defaults
  (`tasks/pupper/pupper_env_cfg.py`). Every Part 1/2 run therefore already trained with them, alongside the task's
  COM, inertia (density ×[0.9, 1.3]) and encoder-bias DR. `check_lab4_dr` asserts the ranges on the live config
  before any Part 3 launch or eval.
- **Checked terrain** [R]. With the torso hull on (a standard flag here), the stock 0.05 m heightfield overflows
  MuJoCo-Warp's per-pair triangle buffer and silently truncates collisions. The old run's Bumpy results were
  invalidated for exactly this (`configs/optional2/README.md`). The reviewed contract is a 0.10 m grid plus an
  accumulated overflow audit (`foundation/scripts/bumpy_checked_cli.py`), now reachable as `launch.py --checked-bumpy`.
  Every Bumpy run and eval here reports zero overflow.

**Recipe** (`part3_finetune.py`): warm start from the final checkpoint, **the same reference dir**,
`configs/opt-lab-2/weights/splay_stand_still.json` (the Part 2 weights), `--num-envs 4096`, seed 42, standard physics
flags, pushed-input checks, and the reference/weights logged to W&B by `launch.py`.

**Budget** [D]: **1500 iterations per arm** ([decision](decisions/2026-11-18-w4-finetune-budget.md), 2/2 graders).
Its [devil's-advocate review](decisions/2026-11-18-w4-finetune-budget/devils-advocate.md) found it "stands with caveat":
the 1500 holds, but the guards broke iteration matching and the stop-early guard could not fire early. The runbook was
**amended (2026-11-18)** as follows:
- **Both arms train together, in 500-iteration legs.** The Bumpy fine-tune and its matched **flat-continuation control**
  (same warm start, weights, seed and PD ranges; `--flat-control`) run as two concurrent processes. Each leg warm-starts
  from the arm's previous leg. 3 legs = 1500.
  - Caveat: each leg boundary re-initializes the episode buffers, a transient of ~20 iterations. The Part 2 legs
    have the same caveat.
- **Gate between legs.** After every leg, both arms' checkpoints get the *same* evaluation: flat_dr + bumpy_dr suite +
  seam plus the nominal-PD suite, and the full matrix + push from leg 3 on.
  - Overflow or NaN stops training.
  - The **+500 G2 rule** (flat_dr ≥ 2× the +0 value and bumpy_dr no better) now **stops before leg 2** instead of
    being a flag after the fact.
- **Matched selection.** Ship the last leg unless it regresses vs the previous leg (> 10 falls on flat_dr, bumpy_dr or
  nompd, or the flat_dr seam flipping PASS → FAIL) on seed 1234 **and the same regression replicates on seed 4321**.
  The seed-4321 evals are run on demand. The control is always compared at the **same** leg.
- **Extension in lockstep:**
  - both arms get one more leg (cap +3000) if bumpy_dr is still dropping by more than 10 falls over the last leg and
    flat_dr stays within 10 of +0;
  - otherwise neither arm extends;
  - the old Bumpy-only "+500 as a new run" is removed.
- Not implemented: G1's "ship the Part 2 parent if no gain at +500 and +1000" stop rule (the review notes only G2's
  flag was implemented). The +0 row in the gates/selection tables makes it visible to a human.
- **Cost (corrected, measured):**
  - ~3 h of training (2 arms × 1500 iterations at 4-way sharing, ~3.4–3.9 s/iter);
  - ~50 min of pre-stages;
  - ~45–75 min of gate/selection evals.

  That is **about 4.5–5 h end to end** (~2.5 h on an otherwise idle GPU). A KEEP → HOLD → WIDEN escalation adds ~4 h.

**Smoke** [M] (`rp-p3-parent-bumpyft-smoke`, W&B `c3rfre1p`, 50 iterations from the Part 1 parent, model_2999 → model_3048):
- Ran end to end through `launch.py --checked-bumpy`, with physics overflow 0.
- Speed was 3.8–4.7 s/iter while sharing the GPU with 4 Part 2 trainers.
- Training stayed stable: ep length ~470/500, err_xy ~0.23 at the end.
- The auto-exported `policy.json` and ONNX were written.
- Its evaluation is a pipeline check, not a result. It used the Part 1 parent, switched weights
  splay → splay_stand_still and ran only 50 iterations: flat_dr falls 40 → 63, bumpy_dr 81 → 91.
- **The control arm already earns its keep.** The matched 50-iteration *flat* continuation
  (`rp-p3-parent-flatctrl-smoke`) shows the same transient (flat_dr 63, bumpy_dr 82 falls). The +50 regression
  comes from the warm start and the weight switch, not from Bumpy.
- **The full chain ran** (`report/part3/eval/p3-parent_selection.md`, `p3-parent_summary.md`):
  - `select` applied the guards and picked +49, the only point;
  - `confirm` re-evaluated on seed 4321;
  - `export-ft` exported it.
- **Seed noise in practice.** The smoke's bumpy_dr seam verdict is FAIL on seed 1234: `sweep fwd-down`,
  12/191 vs 23/2548, excess 0.054, a close call. It is PASS on seed 4321. Judged on both seeds' pooled windows (the
  decision's close-call rule, `part3_seam pool`) it is **PASS**. The runbook's `confirm` stage now does this
  pooling automatically.

**Exact full-run command** (the runbook runs it; standalone):

```bash
cd pupper-mjlab && export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
# one leg of each arm (the runbook runs 3 such pairs, gating between them; legs 2-3 start from the previous leg's last model)
uv run --no-sync python scripts/opt_lab_2/part3_finetune.py --checkpoint <FINAL.pt> --reference-dir <FINAL_REF_DIR> \
    --name p3-final-bumpyft-leg1 --iterations 500 [--pd-ranges wide] &          # Bumpy, 4096 envs, s42
uv run --no-sync python scripts/opt_lab_2/part3_finetune.py --checkpoint <FINAL.pt> --reference-dir <FINAL_REF_DIR> \
    --name p3-final-flatctrl-leg1 --iterations 500 [--pd-ranges wide] --flat-control &   # matched control arm
```

## 4. Dry run on the Part 1 parent: **Part 1 parent, NOT the final policy** (final results: section 0)

Checkpoint `~/runs/opt-lab-2/rp-p1-gallop_c-s42/model_2999.pt` (W&B `1c5r0n8c`), reference `configs/opt-lab-2/gallop_c`
@ `e01364f`. All numbers are simulation only, from one eval seed (1234). GPU rollouts are not bit-reproducible, so
treat fall gaps under ~10–15 as ties. The parent's seam runs used the first harness version (40 envs × 12 schedules,
no step-matched dither control) and were re-scored offline with the frozen rule. The final runs use 64 × 16.
Outputs are in `report/part3/eval/p3-parent/` and the table is `report/part3/eval/p3-parent_summary.md`.

| Condition | Suite falls (rev/stop/steady/turn) | err ±1.5 | vx ±1.5 | sat@1.5 | F run-max med [BW] | drift ± [°/10 s] | push stand@0.65 / move@0.5 | Seam: worst cell (excess, p) | Seam verdict |
|---|---|---|---|---|---|---|---|---|---|
| flat, DR on | 40 (3/37/0/0) | 0.129 / 0.141 | +1.54 / −1.50 | 17.9 % | 3.03 | 70 / 65 | 0.455 / 0.000 | sweep fwd-up +0.008 (0.27) | **PASS** (headline) |
| flat, DR off | 100 (13/87/0/0) | 0.234 / 0.283 | +1.71 / −1.75 | 22.4 % | 3.86 | 113 / 36 | – | dither back-up +0.138 (1e-11) | FAIL (diagnostic) |
| Bumpy, DR on | 81 (8/37/34/2) | 0.167 / 0.170 | +1.46 / −1.43 | 18.0 % | 3.19 | 90 / 81 | 0.455 / 0.024 | sweep fwd-down +0.028 (0.05) | **PASS** (robustness) |
| Bumpy, DR off | 107 (13/78/16/0) | 0.208 / 0.233 | +1.64 / −1.65 | 22.4 % | 3.90 | 124 / 34 | – | dither back-up +0.081 (1e-4) | FAIL (diagnostic) |

**Does it stumble at the seam?** On the parent, at Lab 4 DR: **no** [M].
- **Flat.**
  - Finite-rate sweeps: 1 fall and 3 incidents in 480 crossings.
  - Step dither straddling 0.5 m/s: 0 incidents in 640 crossings.
  - No-seam control: 0 incidents in 320.
- **Bumpy.**
  - Sweeps: 13 incidents in 475.
  - Dither: 5 in 638.
  - Control: 2 in 319.
  - Nothing is significant per cell.
- **The choreography jump is real but absorbed.** At a crossing the observed reference jumps by 1.42 rad (L2 over
  12 joints), against a typical 0.36–0.52 rad per-step change. In the window after an up crossing:
  - 3 Nm saturation rises to 8 % (6 % in the controls);
  - foot force rises to a 2.26 BW median peak (1.65);
  - tracking error rises from 0.105 to 0.165 m/s.

  The policy pays for the switch in torque and tracking, not in falls.
- **The risky command is a hard stop, not the seam.**
  - The step sweep (0 ↔ 1.5 m/s in one step) falls on 17 % of crossings (flat) and 13 % (Bumpy), almost all on
    1.5 → 0.
  - The Part 2 suite agrees: 37 of 40 flat falls are in "stop" contexts.
- **At nominal physics (DR off) the seam *is* fragile.** The backward trot → fast switch (`dither back-up`) pitches the
  body past 30° on 22 of 160 crossings (flat), against 0 in 320 control windows. DR off is also worse
  everywhere else:
  - 100 vs 40 suite falls;
  - tracking error 0.23 vs 0.13;
  - and it overshoots the command (+1.71 vs +1.5 m/s).

  "DR off" means nominal parameters, which sit at the *edge* of the DR distribution:
  - kp = 1.0 against the DR mean of 0.85;
  - kd = 1.0 against 1.15;
  - mass 3.17 kg against a DR mean of 3.46 kg.

  The policy is tuned to a softer, heavier robot than the nominal model. The ablation below says which parameter
  matters. **This is the biggest sim-to-real flag from the dry run**: if the real robot is closer to nominal than to
  the DR mean, expect speed overshoot and backward-seam pitching. Re-check on the final policy.
- **Bumpy costs** 41 falls (40 → 81), mostly steady-state at ±1.5 (34). Tracking degrades 0.13 → 0.17, with no
  overflow. This is the motivation for the fine-tune.

**Which parameter?** The DR ablation sets one event to nominal at a time on flat ground, with the rest randomized
(`part3_eval ablate`, `report/part3/eval/p3-parent/ablate/ablate.md`) [M]:

| Nominal event (others randomized) | Falls | err +1.5 / −1.5 | vx ±1.5 | sat@1.5 |
|---|---|---|---|---|
| none (flat, DR on) | 40 | 0.129 / 0.141 | +1.54 / −1.50 | 17.9 % |
| **pd_gains** (kp = kd = 1.0×) | **99** | 0.174 / 0.201 | +1.63 / −1.63 | 23.0 % |
| inertial (density 1.0×) | 50 | 0.150 / 0.144 | +1.59 / −1.52 | 17.4 % |
| foot_friction | 21 | 0.129 / 0.147 | +1.56 / −1.52 | 17.5 % |
| encoder_bias | 34 | 0.135 / 0.143 | +1.54 / −1.48 | 17.8 % |
| base_com | 41 | 0.130 / 0.139 | +1.55 / −1.52 | 17.8 % |

**The nominal PD gains alone reproduce the whole DR-off failure.** That means 99 falls, the speed overshoot and the
saturation. The other events are within the ~10–15-fall noise band.

Lab 4's kp range ×[0.6, 1.1] is centered at 0.85, so the policy is mostly trained on *weaker* position loops than
the model's kp 5.5. `policy.json` (and therefore the robot) uses kp 5.5, i.e. the edge of the training distribution.
Whether the real motors behave like 1.0× or like 0.85× is a hardware fact sim cannot settle.

Prediction to check on the robot: if it runs faster than commanded and pitches on backward speed-ups, this is
why. The `robot_trims` per-joint kp multipliers in `pupper_gait_deploy` are the existing knob. Changing kp on the
robot changes a reviewed value, though, so it needs a human decision.

**Decision [D, High, arbitrated]: [kp coverage](decisions/2026-11-18-w4-kp-coverage.md).** The three graders split
2 KEEP-and-measure vs 1 WIDEN. The arbitrator made the training range a pre-registered branch of one measurement
on the final, taken before launch:
- **WIDEN** if mean (nominal-PD falls − flat_dr falls) over seeds 1234 and 4321 is > 15.
  - Both the Bumpy arm and its matched flat control then train with kp ×[0.6, 1.3] and kd ×[0.7, 1.5].
  - This is a superset of Lab 4. It applies to this fine-tune only, is reported as a deviation from the spec's
    "your lab 4 DR ranges", and leaves the team record untouched.
- **KEEP** the Lab 4 ranges otherwise.
- **In both branches:**
  - evaluation stays on the exact Lab 4 ranges;
  - nominal-PD falls enter checkpoint selection;
  - a deployment **HOLD** applies if, on two-seed means, nompd − flat_dr > 15 or nompd(selected) > nompd(+0) + 10.

The reviewed robot values do not change: `policy.json` kp 5.5, kd 0.25, 3 Nm, ±1.5 clip.

Why the parent is out of distribution: kp is drawn per joint, so under Lab 4 the robot-average kp is 0.85 ± 0.04.
Only ~6 of 4096 training robots are near the all-1.0× gain the robot commands (arbitrator's Monte Carlo). On the
parent, step 0 (`report/part3/eval/p3-parent_kp_coverage.md`) gives:
- flat_dr falls 40 / 41 and nominal-PD falls 99 / 79 on the two seeds;
- a mean Δ of **48.5**, so the parent would take the **WIDEN** branch.

**The WIDEN smoke PASSES every decision check** (`report/part3/eval/p3-parent_kpwide_smoke.md`):
- Lab 4 record byte-unchanged;
- override and `p3dr-widekp` tag recorded;
- realized kp/5.5 spans [0.600, 1.300] and kd/0.25 spans [0.700, 1.500];
- 19.1 % / 20.1 % of envs have a mean kp ≥ 1.0;
- Bumpy arm at the end: ep length 461, err_xy 0.25, zero overflow;
- export PASS with kp 5.5 / kd 0.25;
- nompd ran.

Smoke sizes: the coordinator capped validation at one ≤ 50-iteration smoke, so this was 45 iterations (Bumpy arm)
+ 5 (control arm; gain-range and config checks only). The final runs 50 + 50 per the decision.

The selection guard and HOLD path were also exercised on the Lab 4 smoke checkpoint (`p3-parent_deploy_hold.md`).
- (a) nompd − flat_dr = 106.0 − 62.5 = +43.5;
- (b) nompd(selected) − nompd(+0) = 106.0 − 89.0 = +17.0.

That gives **HOLD**, which in the KEEP branch launches the WIDEN pair. Escalation was suppressed here with
`--no-escalate`, since this was a pipeline check.

**Transition videos (sim)**: `~/runs/opt-lab-2/p3-videos/p3-parent/p3-parent-{flat_dr,bumpy_dr}_seam_sweep.mp4` (not
committed: run outputs). Each is 31.6 s, real time, 640×480. It shows 0 → +1.5 → 0 → −1.5 → 0 at 0.5 m/s², then a
0.4/0.6 dither, with an overlay of the command and the active reference slot. No falls in either.

## 5. Export and round trip [M]

Report: `report/part3/export/p3-parent.md`. Verdict **PASS**.
- **Actions** on 9,600 recorded observations (seam sweeps, dithers, turns, sidesteps; 23 % in the fast slot):

  | Comparison | Max error |
  |---|---|
  | exported `policy.json` (RTNeural re-implementation, robot order) vs torch | 7.2e-6 (tolerance 1e-3) |
  | the run's auto-exported `policy.json` | 7.2e-6, field-identical to the fresh export |
  | the run's ONNX | 3.1e-6 |

  The gait-reference golden fixture matches the training term to 1.4e-5.
- **Deploy-side checks.** The deploy repo's `download_policy.py --validate-only` passes. The host build of its C++
  `test_gait_reference.cpp` reproduces all 48 golden cases from the new fixture (worst 6e-8) and parses the policy's
  tables.
- **Limits unchanged** from the reviewed values:
  - joint limits, kp 5.5, kd 0.25, action scale 0.75 and the default pose are identical to the shipped `default_policy.json`;
  - kp ≤ `kp_max` 10 and kd ≤ `kd_max` 1;
  - `effort_max` is 3.0 Nm = sim `PUPPER_EFFORT_LIMIT`;
  - the class driver limits equal the robot description, whose sha256 matches the one they were reviewed against;
  - the exported limits contain the driver limits (0.1 rad margin);
  - the command clip is vx ±1.5;
  - all four reviewed files are unchanged vs `origin/main`.
- The smoke fine-tune checkpoint (`report/part3/export/p3-parent-bumpyft-smoke-it3048.md`) also passes. That covers
  the Bumpy-trained policy, its auto-export and its ONNX; the max error is 5.3e-6.
- **W&B finding** [M]: the runner logs "Deploy JSON replaced on W&B Files as policy.json" at every save, but neither the
  parent run (`1c5r0n8c`) nor the smoke run (`c3rfre1p`) has a `policy.json` on W&B after finishing (HTTP 404). The
  W&B ONNX is the *first* save's. So `deploy.sh <run-id>` would fail, or it would fetch nothing current.
  - Cause: with wandb 0.22.3, `wandb.Api().run(...).upload_file`, which `PupperOnPolicyRunner._export_deploy_json`
    calls, returns a size-0 handle and persists nothing. A probe file showed the same.
  - Fix, without touching the runner that W3 is training with: `part3_export --wandb-run <run> --upload-wandb`.
    After all checks PASS, it resumes the *finished* run, `wandb.save`s the verified file, and requires a
    byte-identical read-back.
  - Verified on the smoke run `c3rfre1p`: sha256 `3de79f1e…` on upload, on read-back, **and** from
    `pupper_gait_deploy/download_policy.py photon_mz-stanford-university/mjlab/c3rfre1p`, which is what `deploy.sh` runs.
  - The runner bug itself should be fixed separately: reported, not changed here.

## 6. Hardware: prepared, not done (human operator + TA approval required)

Nothing in this notebook was run on the robot. Simulation results are not evidence of hardware behavior
(root `AGENTS.md`). The repo's physical-session procedure is
[`foundation/HARDWARE_CHECKLIST.md`](../../foundation/HARDWARE_CHECKLIST.md): one operator owns motion,
camera and services; establish ownership and the running release first; a new policy needs simulation
first (done here) and **course/TA approval** before supervised use; preserve joint/current limits.

Checklist for the operator (each item names the evidence it produces):

1. **Approval.** TA approval for the Part 3 policy (fast gait up to ±1.5 m/s). Record who approved and when.
2. **Pick the file.** Use only a `policy.json` whose `report/part3/export/<name>.md` verdict is PASS
   (round trip ≤ 1e-3, golden residual < 1e-4, limit audit, deploy validator, C++ gait-reference test).
   Record its sha256 from that report.
3. **Get it onto the robot.** `./deploy.sh <entity>/mjlab/<run-id>` downloads the run's W&B `policy.json`.
   **Finding [M]: the Part 1 parent's W&B run has no `policy.json`** (HTTP 404), although its training log
   reports uploads at every save; its W&B ONNX is the iteration-0 upload. So either upload the verified file
   (`part3_export --wandb-run <run> --upload-wandb`, which uploads only on PASS and re-downloads to check
   the sha256), or copy the verified file to `pupper_gait_deploy/policies/test_policy.json` and run
   `python3 download_policy.py --validate-only policies/test_policy.json` on the robot. Either way, compare
   the sha256 on the robot with the export report.
4. **Limits unchanged.** The export carries kp 5.5, kd 0.25, action scale 0.75 and the model joint limits,
   identical to the shipped `default_policy.json`; the hardware interface still clamps to the class driver
   limits, `effort_max` 3.0 Nm, `kp_max` 10, `kd_max` 1 (robot description). The audit checks all of this;
   do not edit gains or limits on the robot.
5. **Command clip.** The export clamps teleop to vx ±1.5, vy ±0.5, wz ±2.0. Start at walking speed, approach
   the seam (0.5 m/s) on a tether/stand first, then increase speed in steps. The sim seam test says where
   it is fragile (section 4): a stop from full speed (step 1.5 → 0) is the riskiest command, not the
   slow-to-fast switch itself. Ramp the stick down rather than snapping it to zero. Watch for **speed overshoot and
   pitching on backward speed-ups**: in sim this is what happens when the PD gains are at their nominal
   (deployed) value rather than the Lab 4 DR mean (section 4, ablation). Do not deploy a checkpoint whose
   `report/part3/eval/<name>[-kpwide]_deploy_hold.md` says HOLD. Record which kp branch (KEEP or WIDEN) the
   deployed policy came from.
6. **Measure** (spec): tape-measured straight hallway, stopwatch or a fixed camera with floor marks, actual
   m/s at several commanded speeds (0.3, 0.5, 0.75, 1.0, 1.5), forward and backward, ≥ 3 trials each. Report
   achieved vs commanded and where the gap is largest. Keep the MCAP/joint/IMU logs per
   `HARDWARE_CHECKLIST.md` ("one capture for each persistent session").
7. **Stop/shutdown** per the checklist; record observed outcomes separately from controller receipts.

## 7. Runbook: when W3's final lands

One command, from `pupper-mjlab/` on the A100. Fill in the final checkpoint and its reference dir from W3's notebook
or decision doc:

```bash
cd ~/wt/w4/pupper-mjlab && git pull --rebase origin markus/opt-lab-2 && uv sync
FINAL=~/runs/opt-lab-2/<W3 final run>/model_<N>.pt; REF=configs/opt-lab-2/bootstrap/<final reference dir>
nohup uv run --no-sync python scripts/opt_lab_2/part3_run.py --checkpoint $FINAL --reference-dir $REF \
    --name p3-final > ~/runs/opt-lab-2/p3-final-runbook.log 2>&1 &
```

Stages:
1. `export`: export + round trip + limit audit of the final.
2. `eval`: flat/Bumpy × DR on/off suite + seam, push on DR on.
3. `video`: transition videos.
4. `kpcheck`: [kp-coverage](decisions/2026-11-18-w4-kp-coverage.md) step 0 on the final. It compares flat_dr with
   nominal-PD falls on seeds 1234 and 4321. A mean Δ > 15 selects **WIDEN** (both arms train kp ×[0.6, 1.3],
   kd ×[0.7, 1.5]); otherwise **KEEP** the Lab 4 ranges. Output: `p3-final_kp_coverage.md`.
5. `smoke-wide` (WIDEN only): a ≤ 50-iteration smoke per arm plus the decision's checks. A failure stops the runbook.
6. `train`: the Bumpy arm and the matched flat control **together**, in 500-iteration legs (3 = 1500), PD ranges per
   the branch. Between legs:
   - both arms are evaluated identically (light + nompd; full + push from leg 3);
   - the gate stops on overflow/NaN or G2 at +500;
   - after leg 3 the lockstep extension rule may add a leg to **both** arms (cap +3000).

   Output: `p3-final[-kpwide]_gates.md`.
7. `select`: matched selection. A fallback must replicate on eval seeds 1234 **and** 4321. Output:
   `report/part3/eval/p3-final[-kpwide]_selection.md`.
8. `confirm`: second eval seed (with nompd) and the pooled seam verdict.
9. `hold`: the kp decision's deployment HOLD, using the matched control → `p3-final[-kpwide]_deploy_hold.md`.
   - A HOLD in KEEP launches the pre-authorized WIDEN pair by itself (`--no-escalate` to stop).
   - A HOLD in WIDEN stops for a human.
10. `export-ft`: the selected checkpoint.
11. `table` → `report/part3/eval/p3-final[-kpwide]_summary.md`.

**GPU discipline.** Every launch goes through `scripts/opt_lab_2/gpu_slot.py`. It takes the **same `flock` as W3's
`part2_queue.py`** (`~/runs/opt-lab-2/w3/launch.lock`) and reads the cap from `~/recovery/CONCURRENCY`. It counts the
processes and launches only below the cap, then keeps the lock until the new process appears in `nvidia-smi` (≤ 150 s),
so check + launch are one step. `part3_finetune.py` uses it too when run standalone. W4 runs at most two GPU processes
(the two arms) and evaluations one at a time.

**Expect about 4.5–5 h end to end** while sharing the GPU (~2.5 h if it is idle). Re-running skips finished legs and
stages, and `--dry-run` prints the whole plan without launching anything.
- Add `--upload-wandb` only once a human wants the verified `policy.json` on W&B for `deploy.sh`.

Afterwards: commit `report/part3/` and update sections 4–5 with the final numbers, labelled final. Keep this parent
section as the dry run.

## 8. Decisions, open issues

- [D] [Seam thresholds and verdict rule](decisions/2026-11-18-w4-seam-thresholds.md): arbitrated (graders split on
  pooling vs per-cell). Frozen before any final-policy numbers.
- [D] [Fine-tune budget: 1500 iterations + guards + control arm](decisions/2026-11-18-w4-finetune-budget.md): 2/2 graders.
- [D] [kp coverage: measure on the final, then WIDEN (kp ×[0.6, 1.3], kd ×[0.7, 1.5]) or KEEP; nompd selection guard and
  deployment HOLD](decisions/2026-11-18-w4-kp-coverage.md): High, 3 graders split 2–1, arbitrated.
- Lab 4 DR ranges: **not** a decision. The record, the notebook and the task defaults agree.
- Open:
  - Single training seed for the fine-tune.
  - The seam thresholds were set on the parent.
  - The smoke overlapped 4 W3 trainers (5 GPU processes, because another job launched after the runbook's check).
  - The W&B `policy.json` persistence bug in `PupperOnPolicyRunner` is unfixed (worked around, not fixed).
  - The seam decision's flip check "control max tilt > 25°" reads the max over control windows. With the new
    step-matched dither controls there are ~2,500 control windows per condition, and that max is set by rare falls
    (45°). `part3_seam` now excludes windows that end in a fall from that statistic. A human may prefer p99.
  - Nominal-PD sensitivity (section 4) is ultimately a hardware question: no measurement of the real motors'
    effective gain exists. The kp decision hedges in sim. Any kp trim on the robot needs a human decision.
  - The ablation sets kp and kd to nominal together, so blaming kp alone is unproven (kp-coverage grader-3).
  - Hardware (section 6) is human-only.
