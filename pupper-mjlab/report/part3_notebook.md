# Part 3 lab notebook: multi-gait distillation (simulation side, W4)

Spec: [Opt Lab 2, Part 3](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-2.html).
Started 2026-11-18. **Everything here is simulation evidence. No policy was deployed to hardware.**
Hardware steps need a human operator and TA approval (section 6).

Status (2026-11-18 evening): all tools built, unit-tested and validated end to end on the **Part 1 parent**
(`gallop_c` s42, `1c5r0n8c`) as a dry run. That includes a 50-iteration Bumpy smoke fine-tune with its matched control
arm and a verified W&B upload.
The Part 2 final policy (W3) has not landed yet. When it does, Part 3 is the one command in section 7.

Tags: **[M]** measured here · **[R]** recovered from the old run / repo records · **[D]** lazy decision (doc linked).

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
| `scripts/opt_lab_2/part3_run.py` | The runbook. Runs every stage in order, one GPU process at a time, waits while the GPU has ≥ 4 processes, and skips finished stages. | The parent dry run *is* this script (two passes). |

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

**Budget** [D]: **1500 iterations** ([decision](decisions/2026-11-18-w4-finetune-budget.md), 2/2 graders).
- Evaluate at +500, +1000 and the end. Ship the end unless a pre-registered guard fires:
  - overflow or NaN invalidates the run;
  - fall back to +1000 if the end is worse than +1000 by more than 10 falls on flat_dr or bumpy_dr, or the flat_dr
    seam verdict flips to FAIL;
  - extend by +500 (cap +3000) only if bumpy_dr is still improving by more than 10 falls while flat_dr holds.
- Both graders flagged the missing **control arm** (REPRO_PLAN §3). The runbook therefore also trains the matched
  **flat continuation** (same warm start, weights, seed and 1500 iterations; `--flat-control`), so a robustness gain
  can be credited to Bumpy + DR rather than to training longer.
- Cost: about 2 × 1500 iterations at the shared ~1.25 it/s, i.e. ~40 GPU-minutes of the pie (1.5–2 h wall clock while
  W3 trains), plus ~1 h of evals.

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
uv run --no-sync python scripts/opt_lab_2/part3_finetune.py --checkpoint <FINAL.pt> \
    --reference-dir <FINAL_REF_DIR> --name p3-final-bumpyft            # 1500 it, Bumpy, Lab 4 DR, 4096 envs, s42
uv run --no-sync python scripts/opt_lab_2/part3_finetune.py --checkpoint <FINAL.pt> \
    --reference-dir <FINAL_REF_DIR> --name p3-final-flatctrl --flat-control   # matched control arm
```

## 4. Dry run on the Part 1 parent: **Part 1 parent, NOT the final policy**

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
   (deployed) value rather than the Lab 4 DR mean (section 4, ablation).
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
4. `finetune`: 1500 iterations on Bumpy + Lab 4 DR.
5. `control`: the 1500-iteration flat continuation.
6. `eval-ft`: +500/+1000 light, end full, control full.
7. `select`: the decision's guards → `report/part3/eval/p3-final_selection.md`.
8. `confirm`: second eval seed.
9. `export-ft`: the selected checkpoint.
10. `table` → `report/part3/eval/p3-final_summary.md`.

It waits whenever the GPU has ≥ 4 processes and runs one GPU process at a time. Expect ~1 h of evals plus ~40 GPU-minutes
of training, i.e. 2.5–3.5 h of wall clock while sharing. Re-running skips finished stages.
- Add `--upload-wandb` only once a human wants the verified `policy.json` on W&B for `deploy.sh`.
- Add `--no-control` only if GPU time forces it, and then state the confound.
- If `select` flags "extension rule met", continue +500 from the final fine-tune checkpoint with `part3_finetune.py
  --checkpoint <ft final> --iterations 500`. That is a new run; record it.

Afterwards: commit `report/part3/` and update sections 4–5 with the final numbers, labelled final. Keep this parent
section as the dry run.

## 8. Decisions, open issues

- [D] [Seam thresholds and verdict rule](decisions/2026-11-18-w4-seam-thresholds.md): arbitrated (graders split on
  pooling vs per-cell). Frozen before any final-policy numbers.
- [D] [Fine-tune budget: 1500 iterations + guards + control arm](decisions/2026-11-18-w4-finetune-budget.md): 2/2 graders.
- Lab 4 DR ranges: **not** a decision. The record, the notebook and the task defaults agree.
- Open:
  - Single training seed for the fine-tune.
  - The seam thresholds were set on the parent.
  - The smoke overlapped 4 W3 trainers (5 GPU processes, because another job launched after the runbook's check).
  - The W&B `policy.json` persistence bug in `PupperOnPolicyRunner` is unfixed (worked around, not fixed).
  - The seam decision's flip check "control max tilt > 25°" reads the max over control windows. With the new
    step-matched dither controls there are ~2,500 control windows per condition, and that max is set by rare falls
    (45°). `part3_seam` now excludes windows that end in a fall from that statistic. A human may prefer p99.
  - Nominal-PD sensitivity (section 4) is a hardware question. Any kp trim on the robot needs a human decision.
  - Hardware (section 6) is human-only.
